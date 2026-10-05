use std::{
    collections::BTreeMap,
    io,
    net::SocketAddr,
    sync::{
        Arc,
        atomic::{AtomicU8, Ordering},
    },
    time::Duration,
};

use mavlink::{
    MavHeader, ReadVersion,
    async_peek_reader::AsyncPeekReader,
    dialects::ardupilotmega::{
        HEARTBEAT_DATA, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType,
    },
    read_versioned_raw_message_async,
};
use serde::Serialize;
use serde_json::{Map, Value};
use tokio::{
    io::{AsyncRead, AsyncWrite, AsyncWriteExt},
    net::TcpStream,
    sync::{broadcast, mpsc, oneshot, watch},
    task::JoinHandle,
    time,
};
use tokio_serial::SerialPortBuilderExt;

use crate::{
    codec::{
        CodecError, DecodedMessage, decode_raw, decode_raw_bytes, encode_message, serialize_message,
    },
    journal::{JournalError, JournalHandle},
    records::{Direction, MavlinkFrame, RecordPayload, wall_time_ns},
};

const HEARTBEAT_MESSAGE_ID: u32 = 0;

#[derive(Clone, Debug, Default, Serialize)]
pub struct LinkStatus {
    pub connected: bool,
    pub ready: bool,
    pub connection: String,
    pub clock_epoch: u64,
    pub target_system: u8,
    pub target_component: u8,
    pub base_mode: u64,
    pub custom_mode: u64,
    pub system_status: u64,
    pub latest_time_boot_ms: u64,
    pub received_messages: u64,
    pub transmitted_messages: u64,
    pub framing_errors: u64,
    pub discarded_bytes: u64,
    pub last_received_ns: Option<u64>,
    pub error: Option<String>,
}

#[derive(Clone, Debug, Serialize)]
pub struct ComponentInfo {
    pub system_id: u8,
    pub component_id: u8,
    pub vehicle_type: u64,
    pub autopilot: u64,
    pub base_mode: u64,
    pub custom_mode: u64,
    pub system_status: u64,
    pub last_heartbeat_ns: u64,
}

#[derive(Clone, Debug)]
pub struct MavlinkLinkConfig {
    pub id: String,
    pub transport: MavlinkTransport,
    pub source_system: u8,
    pub source_component: u8,
    pub heartbeat_interval: Duration,
    pub reconnect_interval: Duration,
    pub command_capacity: usize,
}

#[derive(Clone, Debug)]
pub enum MavlinkTransport {
    Tcp(SocketAddr),
    Serial { port: String, baud: u32 },
}

impl std::fmt::Display for MavlinkTransport {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Tcp(address) => write!(formatter, "tcp:{address}"),
            Self::Serial { port, baud } => write!(formatter, "serial:{port}:{baud}"),
        }
    }
}

impl MavlinkLinkConfig {
    #[must_use]
    pub fn sitl(address: SocketAddr) -> Self {
        Self {
            id: "mavlink".to_owned(),
            transport: MavlinkTransport::Tcp(address),
            source_system: 255,
            source_component: 0,
            heartbeat_interval: Duration::from_secs(1),
            reconnect_interval: Duration::from_secs(1),
            command_capacity: 1024,
        }
    }

    #[must_use]
    pub fn serial(port: impl Into<String>, baud: u32) -> Self {
        Self {
            id: "mavlink".to_owned(),
            transport: MavlinkTransport::Serial {
                port: port.into(),
                baud,
            },
            source_system: 255,
            source_component: 0,
            heartbeat_interval: Duration::from_secs(1),
            reconnect_interval: Duration::from_secs(1),
            command_capacity: 1024,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ReceivedMessage {
    pub journal_sequence: u64,
    pub ingest_time_ns: u64,
    pub link_id: String,
    pub system_id: u8,
    pub component_id: u8,
    pub sequence: u8,
    pub message_id: u32,
    pub name: String,
    pub fields: Map<String, Value>,
    pub message: Arc<MavMessage>,
}

enum OutboundPayload {
    Raw(Vec<u8>),
    Message {
        message: Box<MavMessage>,
        source_system: Option<u8>,
        source_component: Option<u8>,
    },
}

struct Outbound {
    payload: OutboundPayload,
    reply: oneshot::Sender<Result<u64, LinkError>>,
}

#[derive(Clone)]
pub struct MavlinkLinkHandle {
    outbound: mpsc::Sender<Outbound>,
    received: broadcast::Sender<Arc<ReceivedMessage>>,
    status: watch::Receiver<LinkStatus>,
    components: watch::Receiver<Vec<ComponentInfo>>,
}

impl MavlinkLinkHandle {
    #[must_use]
    pub fn status(&self) -> LinkStatus {
        self.status.borrow().clone()
    }

    pub fn subscribe_status(&self) -> watch::Receiver<LinkStatus> {
        self.status.clone()
    }

    pub fn subscribe_messages(&self) -> broadcast::Receiver<Arc<ReceivedMessage>> {
        self.received.subscribe()
    }

    #[must_use]
    pub fn components(&self) -> Vec<ComponentInfo> {
        self.components.borrow().clone()
    }

    pub async fn send_raw(&self, frame: Vec<u8>) -> Result<u64, LinkError> {
        let decoded = decode_raw_bytes(&frame)?;
        if decoded.raw != frame {
            return Err(LinkError::InvalidFrame(
                "frame contains leading or trailing bytes".to_owned(),
            ));
        }
        self.send(OutboundPayload::Raw(frame)).await
    }

    pub async fn send_message(
        &self,
        name: &str,
        fields: &Map<String, Value>,
        source_system: Option<u8>,
        source_component: Option<u8>,
    ) -> Result<u64, LinkError> {
        let message = encode_message(name, fields)?;
        self.send(OutboundPayload::Message {
            message: Box::new(message),
            source_system,
            source_component,
        })
        .await
    }

    async fn send(&self, payload: OutboundPayload) -> Result<u64, LinkError> {
        let (reply, response) = oneshot::channel();
        self.outbound
            .send(Outbound { payload, reply })
            .await
            .map_err(|_| LinkError::Unavailable)?;
        response.await.map_err(|_| LinkError::Unavailable)?
    }
}

#[derive(Debug, thiserror::Error)]
pub enum LinkError {
    #[error("MAVLink link is unavailable")]
    Unavailable,
    #[error("invalid MAVLink frame: {0}")]
    InvalidFrame(String),
    #[error("MAVLink receive failed: {0}")]
    Receive(String),
    #[error("MAVLink I/O failed: {0}")]
    Io(#[from] io::Error),
    #[error(transparent)]
    Codec(#[from] CodecError),
    #[error(transparent)]
    Journal(#[from] JournalError),
}

pub fn start_link(
    config: MavlinkLinkConfig,
    journal: JournalHandle,
) -> (MavlinkLinkHandle, JoinHandle<()>) {
    assert!(
        !config.heartbeat_interval.is_zero(),
        "heartbeat_interval must be positive"
    );
    assert!(
        !config.reconnect_interval.is_zero(),
        "reconnect_interval must be positive"
    );
    let (outbound, receiver) = mpsc::channel(config.command_capacity);
    let (received, _) = broadcast::channel(config.command_capacity);
    let initial_status = LinkStatus {
        connection: config.transport.to_string(),
        ..LinkStatus::default()
    };
    let (status, status_rx) = watch::channel(initial_status);
    let (components, components_rx) = watch::channel(Vec::new());
    let handle = MavlinkLinkHandle {
        outbound,
        received: received.clone(),
        status: status_rx,
        components: components_rx,
    };
    let task = tokio::spawn(run_link(
        config, journal, receiver, received, status, components,
    ));
    (handle, task)
}

async fn run_link(
    config: MavlinkLinkConfig,
    journal: JournalHandle,
    mut outbound: mpsc::Receiver<Outbound>,
    received: broadcast::Sender<Arc<ReceivedMessage>>,
    status_tx: watch::Sender<LinkStatus>,
    components_tx: watch::Sender<Vec<ComponentInfo>>,
) {
    let tx_sequence = AtomicU8::new(0);
    let mut components = BTreeMap::new();
    loop {
        match &config.transport {
            MavlinkTransport::Tcp(address) => match TcpStream::connect(address).await {
                Ok(stream) => {
                    stream.set_nodelay(true).ok();
                    let (reader, writer) = stream.into_split();
                    run_transport(
                        ConnectionContext {
                            config: &config,
                            journal: &journal,
                            outbound: &mut outbound,
                            received: &received,
                            status_tx: &status_tx,
                            components_tx: &components_tx,
                            components: &mut components,
                            tx_sequence: &tx_sequence,
                        },
                        reader,
                        writer,
                    )
                    .await;
                }
                Err(error) => record_connect_error(&status_tx, error),
            },
            MavlinkTransport::Serial { port, baud } => {
                match tokio_serial::new(port, *baud).open_native_async() {
                    Ok(stream) => {
                        let (reader, writer) = tokio::io::split(stream);
                        run_transport(
                            ConnectionContext {
                                config: &config,
                                journal: &journal,
                                outbound: &mut outbound,
                                received: &received,
                                status_tx: &status_tx,
                                components_tx: &components_tx,
                                components: &mut components,
                                tx_sequence: &tx_sequence,
                            },
                            reader,
                            writer,
                        )
                        .await;
                    }
                    Err(error) => record_connect_error(&status_tx, error),
                }
            }
        }
        time::sleep(config.reconnect_interval).await;
    }
}

struct ConnectionContext<'a> {
    config: &'a MavlinkLinkConfig,
    journal: &'a JournalHandle,
    outbound: &'a mut mpsc::Receiver<Outbound>,
    received: &'a broadcast::Sender<Arc<ReceivedMessage>>,
    status_tx: &'a watch::Sender<LinkStatus>,
    components_tx: &'a watch::Sender<Vec<ComponentInfo>>,
    components: &'a mut BTreeMap<(u8, u8), ComponentInfo>,
    tx_sequence: &'a AtomicU8,
}

async fn run_transport<R, W>(context: ConnectionContext<'_>, reader: R, writer: W)
where
    R: AsyncRead + Unpin + Send + 'static,
    W: AsyncWrite + Unpin + Send,
{
    let clock_epoch = match context.journal.begin_clock_epoch().await {
        Ok(epoch) => epoch,
        Err(error) => {
            context.status_tx.send_modify(|status| {
                status.connected = false;
                status.ready = false;
                status.error = Some(error.to_string());
            });
            tracing::error!(link = %context.config.id, %error, "could not start MAVLink clock epoch");
            return;
        }
    };
    context.status_tx.send_modify(|status| {
        status.connected = true;
        status.clock_epoch = clock_epoch;
        status.latest_time_boot_ms = 0;
        status.error = None;
    });
    let link_id = context.config.id.clone();
    let status_tx = context.status_tx.clone();
    if let Err(error) = run_connected(context, reader, writer).await {
        if status_tx.borrow().ready {
            tracing::warn!(link = %link_id, %error, "MAVLink connection lost");
        } else {
            tracing::debug!(link = %link_id, %error, "MAVLink acquisition retry");
        }
        status_tx.send_modify(|status| {
            status.connected = false;
            status.ready = false;
            status.error = Some(error.to_string());
        });
    }
}

fn record_connect_error(status_tx: &watch::Sender<LinkStatus>, error: impl std::fmt::Display) {
    status_tx.send_modify(|status| {
        status.connected = false;
        status.ready = false;
        status.error = Some(error.to_string());
    });
}

async fn run_connected<R, W>(
    context: ConnectionContext<'_>,
    reader: R,
    mut writer: W,
) -> Result<(), LinkError>
where
    R: AsyncRead + Unpin + Send + 'static,
    W: AsyncWrite + Unpin + Send,
{
    let ConnectionContext {
        config,
        journal,
        outbound,
        received,
        status_tx,
        components_tx,
        components,
        tx_sequence,
    } = context;
    let (receive_tx, mut receive_rx) = mpsc::channel(256);
    let receive_task = tokio::spawn(receive_messages(reader, receive_tx));
    let mut heartbeat = time::interval(config.heartbeat_interval);
    heartbeat.set_missed_tick_behavior(time::MissedTickBehavior::Delay);

    let result = loop {
        tokio::select! {
            received_frame = receive_rx.recv() => {
                let Some(received_frame) = received_frame else {
                    break Err(LinkError::Receive("MAVLink receive task stopped".to_owned()));
                };
                let decoded = match received_frame {
                    Ok(decoded) => decoded,
                    Err(error) => break Err(LinkError::Receive(error)),
                };
                archive_received(
                    config,
                    journal,
                    received,
                    status_tx,
                    components_tx,
                    components,
                    decoded,
                ).await?;
            }
            command = outbound.recv() => {
                let Some(command) = command else {
                    break Err(LinkError::Unavailable);
                };
                let result = send_outbound(
                    config,
                    journal,
                    &mut writer,
                    command.payload,
                    status_tx,
                    tx_sequence,
                ).await;
                let failed = result.is_err();
                let _ = command.reply.send(result);
                if failed {
                    break Err(LinkError::Receive("MAVLink send failed".to_owned()));
                }
            }
            _ = heartbeat.tick() => {
                send_typed(
                    config,
                    journal,
                    &mut writer,
                    heartbeat_message(),
                    (None, None),
                    status_tx,
                    tx_sequence,
                ).await?;
            }
        }
    };
    receive_task.abort();
    result
}

async fn receive_messages<R>(reader: R, sender: mpsc::Sender<Result<DecodedMessage, String>>)
where
    R: AsyncRead + Unpin,
{
    let mut reader = AsyncPeekReader::new(reader);
    loop {
        let result =
            read_versioned_raw_message_async::<MavMessage, _>(&mut reader, ReadVersion::Any)
                .await
                .map_err(|error| error.to_string())
                .and_then(|raw| decode_raw(raw).map_err(|error| error.to_string()));
        let failed = result.is_err();
        if sender.send(result).await.is_err() || failed {
            return;
        }
    }
}

async fn archive_received(
    config: &MavlinkLinkConfig,
    journal: &JournalHandle,
    received: &broadcast::Sender<Arc<ReceivedMessage>>,
    status_tx: &watch::Sender<LinkStatus>,
    components_tx: &watch::Sender<Vec<ComponentInfo>>,
    components: &mut BTreeMap<(u8, u8), ComponentInfo>,
    decoded: DecodedMessage,
) -> Result<(), LinkError> {
    let now = wall_time_ns();
    let is_heartbeat = decoded.message_id == HEARTBEAT_MESSAGE_ID;
    let system_id = decoded.system_id;
    let component_id = decoded.component_id;
    let time_boot_ms = decoded.fields.get("time_boot_ms").and_then(Value::as_u64);
    let heartbeat_state = is_heartbeat.then(|| {
        (
            decoded.fields["base_mode"].as_u64().unwrap_or_default(),
            decoded.fields["custom_mode"].as_u64().unwrap_or_default(),
            decoded.fields["system_status"].as_u64().unwrap_or_default(),
        )
    });
    if is_heartbeat {
        update_component_registry(components, system_id, component_id, &decoded.fields, now)?;
        let _ = components_tx.send(components.values().cloned().collect());
    }
    let record = decoded_frame(&config.id, Direction::Rx, &decoded);
    let payload = RecordPayload::MavlinkFrame(record);
    let journal_sequence = match time_boot_ms {
        Some(time_boot_ms) => {
            journal
                .append_with_time_boot_ms(payload, None, time_boot_ms)
                .await?
        }
        None => journal.append(payload, None).await?,
    };
    let _ = received.send(Arc::new(ReceivedMessage {
        journal_sequence,
        ingest_time_ns: now,
        link_id: config.id.clone(),
        system_id,
        component_id,
        sequence: decoded.sequence,
        message_id: decoded.message_id,
        name: decoded.name,
        fields: decoded.fields,
        message: Arc::new(decoded.message),
    }));
    status_tx.send_modify(|status| {
        status.received_messages += 1;
        status.last_received_ns = Some(now);
        if is_heartbeat && system_id != config.source_system {
            status.target_system = system_id;
            status.target_component = component_id;
            let (base_mode, custom_mode, system_status) =
                heartbeat_state.expect("heartbeat state was captured");
            status.base_mode = base_mode;
            status.custom_mode = custom_mode;
            status.system_status = system_status;
            status.ready = true;
        }
        if let Some(time_boot_ms) = time_boot_ms {
            status.latest_time_boot_ms = status.latest_time_boot_ms.max(time_boot_ms);
        }
    });
    Ok(())
}

fn update_component_registry(
    components: &mut BTreeMap<(u8, u8), ComponentInfo>,
    system_id: u8,
    component_id: u8,
    fields: &Map<String, Value>,
    timestamp_ns: u64,
) -> Result<(), LinkError> {
    let integer = |name| {
        fields
            .get(name)
            .and_then(Value::as_u64)
            .ok_or_else(|| LinkError::Receive(format!("HEARTBEAT missing numeric {name}")))
    };
    components.insert(
        (system_id, component_id),
        ComponentInfo {
            system_id,
            component_id,
            vehicle_type: integer("type")?,
            autopilot: integer("autopilot")?,
            base_mode: integer("base_mode")?,
            custom_mode: integer("custom_mode")?,
            system_status: integer("system_status")?,
            last_heartbeat_ns: timestamp_ns,
        },
    );
    Ok(())
}

async fn send_outbound(
    config: &MavlinkLinkConfig,
    journal: &JournalHandle,
    writer: &mut (impl AsyncWrite + Unpin),
    payload: OutboundPayload,
    status_tx: &watch::Sender<LinkStatus>,
    tx_sequence: &AtomicU8,
) -> Result<u64, LinkError> {
    match payload {
        OutboundPayload::Raw(bytes) => send_bytes(config, journal, writer, bytes, status_tx).await,
        OutboundPayload::Message {
            message,
            source_system,
            source_component,
        } => {
            send_typed(
                config,
                journal,
                writer,
                *message,
                (source_system, source_component),
                status_tx,
                tx_sequence,
            )
            .await
        }
    }
}

async fn send_typed(
    config: &MavlinkLinkConfig,
    journal: &JournalHandle,
    writer: &mut (impl AsyncWrite + Unpin),
    message: MavMessage,
    source: (Option<u8>, Option<u8>),
    status_tx: &watch::Sender<LinkStatus>,
    tx_sequence: &AtomicU8,
) -> Result<u64, LinkError> {
    let header = MavHeader {
        system_id: source.0.unwrap_or(config.source_system),
        component_id: source.1.unwrap_or(config.source_component),
        sequence: tx_sequence.fetch_add(1, Ordering::Relaxed),
    };
    let bytes = serialize_message(&message, header)?;
    send_bytes(config, journal, writer, bytes, status_tx).await
}

async fn send_bytes(
    config: &MavlinkLinkConfig,
    journal: &JournalHandle,
    writer: &mut (impl AsyncWrite + Unpin),
    bytes: Vec<u8>,
    status_tx: &watch::Sender<LinkStatus>,
) -> Result<u64, LinkError> {
    let decoded = decode_raw_bytes(&bytes)?;
    if decoded.raw != bytes {
        return Err(LinkError::InvalidFrame(
            "frame contains leading or trailing bytes".to_owned(),
        ));
    }
    writer.write_all(&bytes).await?;
    let frame = decoded_frame(&config.id, Direction::Tx, &decoded);
    let sequence = journal
        .append(RecordPayload::MavlinkFrame(frame), None)
        .await?;
    status_tx.send_modify(|status| status.transmitted_messages += 1);
    Ok(sequence)
}

fn decoded_frame(link_id: &str, direction: Direction, decoded: &DecodedMessage) -> MavlinkFrame {
    MavlinkFrame {
        link_id: link_id.to_owned(),
        direction,
        protocol_version: decoded.mavlink_version,
        sequence: decoded.sequence,
        system_id: decoded.system_id,
        component_id: decoded.component_id,
        message_id: decoded.message_id,
        message_name: decoded.name.clone(),
        fields: decoded.fields.clone(),
        signed: decoded.signed,
        frame: decoded.raw.clone(),
    }
}

fn heartbeat_message() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use tempfile::TempDir;
    use tokio::{
        io::{AsyncReadExt, AsyncWriteExt},
        net::TcpListener,
    };
    use uuid::Uuid;

    use crate::{codec::serialize_message, journal::JournalConfig};

    fn heartbeat(sequence: u8, system_id: u8, component_id: u8) -> Vec<u8> {
        serialize_message(
            &heartbeat_message(),
            MavHeader {
                sequence,
                system_id,
                component_id,
            },
        )
        .expect("heartbeat")
    }

    #[test]
    fn generated_heartbeat_is_complete_v2_frame() {
        let bytes = heartbeat(9, 255, 0);
        let frame = decode_raw_bytes(&bytes).expect("heartbeat frame");

        assert_eq!(
            bytes,
            [
                0xfd, 0x09, 0x00, 0x00, 0x09, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x06, 0x08, 0x00, 0x04, 0x03, 0x51, 0x82,
            ]
        );
        assert_eq!(frame.mavlink_version, 2);
        assert_eq!(frame.sequence, 9);
        assert_eq!(frame.system_id, 255);
        assert_eq!(frame.message_id, HEARTBEAT_MESSAGE_ID);
        assert_eq!(frame.raw, bytes);
    }

    #[test]
    fn component_registry_matches_http_contract_and_refreshes_heartbeats() {
        let mut components = BTreeMap::new();
        let fields = serde_json::from_value(json!({
            "type": 2,
            "autopilot": 3,
            "base_mode": 129,
            "custom_mode": 4,
            "system_status": 4,
        }))
        .expect("heartbeat fields");

        update_component_registry(&mut components, 1, 1, &fields, 100).expect("first heartbeat");
        update_component_registry(&mut components, 1, 1, &fields, 200).expect("new heartbeat");

        let value = serde_json::to_value(components.get(&(1, 1)).expect("component"))
            .expect("serialize component");
        assert_eq!(
            value,
            json!({
                "system_id": 1,
                "component_id": 1,
                "vehicle_type": 2,
                "autopilot": 3,
                "base_mode": 129,
                "custom_mode": 4,
                "system_status": 4,
                "last_heartbeat_ns": 200,
            })
        );
        assert_eq!(components.len(), 1);
    }

    #[tokio::test]
    async fn link_archives_rx_and_tx_on_one_journal() {
        let server = TcpListener::bind("127.0.0.1:0")
            .await
            .expect("test listener");
        let address = server.local_addr().expect("listener address");
        let peer = tokio::spawn(async move {
            let (mut socket, _) = server.accept().await.expect("accept link");
            socket
                .write_all(&heartbeat(1, 1, 1))
                .await
                .expect("send vehicle heartbeat");
            let mut received = [0_u8; 21];
            socket
                .read_exact(&mut received)
                .await
                .expect("read GCS heartbeat");
            decode_raw_bytes(&received).expect("GCS heartbeat")
        });

        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let journal_config = JournalConfig::for_directory(temp.path(), run_id);
        let (journal, journal_task) = JournalHandle::start(journal_config).await.expect("journal");
        let mut link_config = MavlinkLinkConfig::sitl(address);
        link_config.heartbeat_interval = Duration::from_millis(10);
        let (link, link_task) = start_link(link_config, journal.clone());

        time::timeout(Duration::from_secs(2), async {
            let mut status = link.subscribe_status();
            while !status.borrow().ready {
                status.changed().await.expect("status update");
            }
        })
        .await
        .expect("link ready");
        let transmitted = peer.await.expect("peer task");
        time::timeout(Duration::from_secs(2), async {
            loop {
                if journal.tail() >= 2 {
                    break;
                }
                time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("journal records");
        let records = journal.records_after(0).await.expect("journal records");

        assert_eq!(transmitted.system_id, 255);
        assert!(records.iter().any(|record| matches!(
            &record.payload,
            RecordPayload::MavlinkFrame(frame) if frame.direction == Direction::Rx
        )));
        assert!(records.iter().any(|record| matches!(
            &record.payload,
            RecordPayload::MavlinkFrame(frame) if frame.direction == Direction::Tx
        )));
        assert!(records.iter().all(|record| record.sim_clock.epoch == 1));
        assert_eq!(link.status().clock_epoch, 1);
        assert_eq!(link.status().target_system, 1);
        assert_eq!(link.status().base_mode, 0);
        assert_eq!(link.status().custom_mode, 0);
        assert_eq!(link.status().system_status, 4);

        link_task.abort();
        journal.shutdown().await.expect("shutdown");
        journal_task.await.expect("journal task");
    }
}
