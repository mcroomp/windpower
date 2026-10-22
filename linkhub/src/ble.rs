use std::{io, pin::Pin, time::Duration};

use btleplug::{
    api::{
        Central, CentralEvent, CharPropFlags, Manager as _, Peripheral as _, ScanFilter, WriteType,
    },
    platform::{Manager, Peripheral},
};
use futures_util::StreamExt;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt, DuplexStream},
    time,
};
use uuid::Uuid;

pub const DRONEBRIDGE_SERVICE_UUID: &str = "0000db32-0000-1000-8000-00805f9b34fb";
pub const DRONEBRIDGE_WRITE_UUID: &str = "0000db33-0000-1000-8000-00805f9b34fb";
pub const DRONEBRIDGE_NOTIFY_UUID: &str = "0000db34-0000-1000-8000-00805f9b34fb";

const STREAM_BUFFER_SIZE: usize = 64 * 1024;
const DEFAULT_ATT_MTU: usize = 23;

#[derive(Clone, Debug)]
pub struct BleConfig {
    pub device_name: String,
    pub scan_timeout: Duration,
}

#[derive(Debug)]
pub struct BleConnection {
    pub device_name: String,
    pub stream: DuplexStream,
}

impl BleConfig {
    pub fn new(device_name: String, scan_timeout: Duration) -> Result<Self, String> {
        if device_name.trim().is_empty() {
            return Err("BLE device name must not be empty".to_owned());
        }
        if scan_timeout.is_zero() {
            return Err("BLE scan timeout must be positive".to_owned());
        }
        Ok(Self {
            device_name,
            scan_timeout,
        })
    }
}

pub async fn connect(config: &BleConfig) -> Result<BleConnection, String> {
    let manager = Manager::new().await.map_err(|error| error.to_string())?;
    let adapter = manager
        .adapters()
        .await
        .map_err(|error| error.to_string())?
        .into_iter()
        .next()
        .ok_or_else(|| "no Bluetooth adapter found".to_owned())?;
    let events = adapter.events().await.map_err(|error| error.to_string())?;
    let service_uuid = Uuid::parse_str(DRONEBRIDGE_SERVICE_UUID)
        .map_err(|error| format!("invalid DroneBridge service UUID: {error}"))?;
    let write_uuid = Uuid::parse_str(DRONEBRIDGE_WRITE_UUID)
        .map_err(|error| format!("invalid DroneBridge write UUID: {error}"))?;
    let notify_uuid = Uuid::parse_str(DRONEBRIDGE_NOTIFY_UUID)
        .map_err(|error| format!("invalid DroneBridge notify UUID: {error}"))?;

    adapter
        .start_scan(ScanFilter::default())
        .await
        .map_err(|error| format!("could not start BLE scan: {error}"))?;
    let peripheral = match find_peripheral(&adapter, &config.device_name, config.scan_timeout).await
    {
        Ok(peripheral) => peripheral,
        Err(error) => {
            if let Err(stop_error) = adapter.stop_scan().await {
                return Err(format!("{error}; could not stop BLE scan: {stop_error}"));
            }
            return Err(error);
        }
    };
    adapter
        .stop_scan()
        .await
        .map_err(|error| format!("could not stop BLE scan: {error}"))?;

    peripheral
        .connect()
        .await
        .map_err(|error| format!("could not connect to {}: {error}", config.device_name))?;
    if let Err(error) = peripheral.discover_services().await {
        return Err(disconnect_after_setup_error(&peripheral, error.to_string()).await);
    }

    let characteristics = peripheral.characteristics();
    if !peripheral
        .services()
        .iter()
        .any(|service| service.uuid == service_uuid)
    {
        return Err(disconnect_after_setup_error(
            &peripheral,
            "DroneBridge GATT serial service was not discovered".to_owned(),
        )
        .await);
    }
    let write_characteristic = characteristics
        .iter()
        .find(|characteristic| {
            characteristic.uuid == write_uuid
                && characteristic.service_uuid == service_uuid
                && characteristic.properties.contains(CharPropFlags::WRITE)
        })
        .cloned();
    let notify_characteristic = characteristics
        .iter()
        .find(|characteristic| {
            characteristic.uuid == notify_uuid
                && characteristic.service_uuid == service_uuid
                && characteristic.properties.contains(CharPropFlags::NOTIFY)
        })
        .cloned();
    let (Some(write_characteristic), Some(notify_characteristic)) =
        (write_characteristic, notify_characteristic)
    else {
        return Err(disconnect_after_setup_error(
            &peripheral,
            "DroneBridge GATT service is missing its write or notify characteristic".to_owned(),
        )
        .await);
    };

    let notifications = match peripheral.notifications().await {
        Ok(notifications) => notifications,
        Err(error) => {
            return Err(disconnect_after_setup_error(&peripheral, error.to_string()).await);
        }
    };
    if let Err(error) = peripheral.subscribe(&notify_characteristic).await {
        return Err(disconnect_after_setup_error(&peripheral, error.to_string()).await);
    }
    let mtu = usize::from(peripheral.mtu()).max(DEFAULT_ATT_MTU);
    let write_chunk_size = mtu.saturating_sub(3).max(1);
    let device_id = peripheral.id();
    let (stream, relay_stream) = tokio::io::duplex(STREAM_BUFFER_SIZE);
    tokio::spawn(relay(
        manager,
        adapter,
        peripheral,
        device_id,
        write_characteristic,
        notify_uuid,
        events,
        notifications,
        relay_stream,
        write_chunk_size,
    ));

    Ok(BleConnection {
        device_name: config.device_name.clone(),
        stream,
    })
}

async fn find_peripheral(
    adapter: &btleplug::platform::Adapter,
    name: &str,
    scan_timeout: Duration,
) -> Result<Peripheral, String> {
    let mut seen = std::collections::BTreeSet::new();
    let mut unnamed = std::collections::BTreeSet::new();
    let scan = async {
        loop {
            for peripheral in adapter
                .peripherals()
                .await
                .map_err(|error| error.to_string())?
            {
                let address = peripheral.address().to_string();
                if address.eq_ignore_ascii_case(name) {
                    return Ok(peripheral);
                }
                let Some(local_name) = peripheral
                    .properties()
                    .await
                    .map_err(|error| error.to_string())?
                    .and_then(|properties| properties.local_name)
                else {
                    unnamed.insert(address);
                    continue;
                };
                if local_name.eq_ignore_ascii_case(name) {
                    return Ok(peripheral);
                }
                seen.insert(format!("{local_name} ({address})"));
            }
            time::sleep(Duration::from_millis(100)).await;
        }
    };
    let result = time::timeout(scan_timeout, scan).await;
    match result {
        Ok(result) => result,
        Err(_) => Err(format!(
            "BLE device {name:?} was not found before scan timeout; named devices seen: {}; \
             unnamed devices seen: {}",
            if seen.is_empty() {
                "none".to_owned()
            } else {
                seen.into_iter().collect::<Vec<_>>().join(", ")
            },
            unnamed.len()
        )),
    }
}

async fn disconnect_after_setup_error(peripheral: &Peripheral, error: String) -> String {
    match peripheral.disconnect().await {
        Ok(()) => format!("could not initialize DroneBridge BLE link: {error}"),
        Err(disconnect_error) => format!(
            "could not initialize DroneBridge BLE link: {error}; disconnect also failed: \
             {disconnect_error}"
        ),
    }
}

#[allow(clippy::too_many_arguments)]
async fn relay(
    // Dropping the manager or adapter closes the adapter event stream.
    _manager: Manager,
    _adapter: btleplug::platform::Adapter,
    peripheral: Peripheral,
    device_id: btleplug::platform::PeripheralId,
    write_characteristic: btleplug::api::Characteristic,
    notify_uuid: Uuid,
    mut events: Pin<Box<dyn futures_util::Stream<Item = CentralEvent> + Send>>,
    mut notifications: Pin<
        Box<dyn futures_util::Stream<Item = btleplug::api::ValueNotification> + Send>,
    >,
    stream: DuplexStream,
    write_chunk_size: usize,
) {
    let (mut reader, mut writer) = tokio::io::split(stream);
    let mut buffer = vec![0_u8; 4096];
    let result = 'relay: loop {
        tokio::select! {
            notification = notifications.next() => {
                let Some(notification) = notification else {
                    break Err(io::Error::new(
                        io::ErrorKind::UnexpectedEof,
                        "BLE notification stream ended",
                    ));
                };
                if notification.uuid == notify_uuid
                    && let Err(error) = writer.write_all(&notification.value).await
                {
                    break Err(error);
                }
            }
            event = events.next() => {
                match event {
                    Some(CentralEvent::DeviceDisconnected(id)) if id == device_id => {
                        break Err(io::Error::new(
                            io::ErrorKind::ConnectionAborted,
                            "DroneBridge BLE device disconnected",
                        ));
                    }
                    Some(_) => {}
                    None => {
                        break Err(io::Error::new(
                            io::ErrorKind::UnexpectedEof,
                            "BLE adapter event stream ended",
                        ));
                    }
                }
            }
            read = reader.read(&mut buffer) => {
                let length = match read {
                    Ok(0) => break Ok(()),
                    Ok(length) => length,
                    Err(error) => break Err(error),
                };
                for chunk in buffer[..length].chunks(write_chunk_size) {
                    if let Err(error) = peripheral
                        .write(&write_characteristic, chunk, WriteType::WithResponse)
                        .await
                    {
                        break 'relay Err(io::Error::other(format!(
                            "DroneBridge BLE GATT write failed: {error}"
                        )));
                    }
                }
            }
        }
    };
    if let Err(error) = result {
        tracing::warn!(%error, "DroneBridge BLE transport ended");
    }
    if let Err(error) = peripheral.disconnect().await {
        tracing::warn!(%error, "could not disconnect DroneBridge BLE peripheral");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_requires_nonempty_device_name_and_positive_timeout() {
        assert!(BleConfig::new(" ".to_owned(), Duration::from_secs(1)).is_err());
        assert!(BleConfig::new("DroneBridge".to_owned(), Duration::ZERO).is_err());
        assert!(BleConfig::new("DroneBridge".to_owned(), Duration::from_secs(1)).is_ok());
    }

    #[test]
    fn dronebridge_uuids_match_documented_gatt_service() {
        assert_eq!(
            DRONEBRIDGE_SERVICE_UUID,
            "0000db32-0000-1000-8000-00805f9b34fb"
        );
        assert_eq!(
            DRONEBRIDGE_WRITE_UUID,
            "0000db33-0000-1000-8000-00805f9b34fb"
        );
        assert_eq!(
            DRONEBRIDGE_NOTIFY_UUID,
            "0000db34-0000-1000-8000-00805f9b34fb"
        );
    }
}
