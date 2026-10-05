use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use thiserror::Error;
use tokio::sync::Mutex as AsyncMutex;
use tokio::task::JoinHandle;

pub const NUS_SERVICE_UUID: &str = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
pub const NUS_RX_UUID: &str = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
const SAFE_STOP_COMMAND: &str = "S:0;E:0;B:1\n";

#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum MotorDirection {
    #[default]
    Forward,
    Reverse,
}

impl MotorDirection {
    const fn wire_value(self) -> char {
        match self {
            Self::Forward => 'F',
            Self::Reverse => 'R',
        }
    }
}

#[derive(Clone, Debug)]
pub struct MotorConfig {
    pub name_prefix: String,
    pub scan_timeout: Duration,
    pub heartbeat_interval: Duration,
    pub maximum_command_timeout_ms: u64,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            name_prefix: "BLDC".to_owned(),
            scan_timeout: Duration::from_secs(10),
            heartbeat_interval: Duration::from_millis(500),
            maximum_command_timeout_ms: 10_000,
        }
    }
}

impl MotorConfig {
    fn validate(&self) -> Result<(), MotorError> {
        if self.name_prefix.is_empty() {
            return Err(MotorError::InvalidConfiguration(
                "name_prefix must not be empty",
            ));
        }
        if self.scan_timeout.is_zero() {
            return Err(MotorError::InvalidConfiguration(
                "scan_timeout must be positive",
            ));
        }
        if self.heartbeat_interval.is_zero() {
            return Err(MotorError::InvalidConfiguration(
                "heartbeat_interval must be positive",
            ));
        }
        if self.maximum_command_timeout_ms == 0 {
            return Err(MotorError::InvalidConfiguration(
                "maximum_command_timeout_ms must be positive",
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MotorStatus {
    pub available: bool,
    pub connected: bool,
    pub device: Option<String>,
    pub running: bool,
    pub speed_percent: u8,
    pub direction: MotorDirection,
    pub command_expires_in_ms: Option<u64>,
    pub error: Option<String>,
}

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum MotorError {
    #[error("{0}")]
    InvalidConfiguration(&'static str),
    #[error("speed_percent must be in 1..100")]
    InvalidSpeed,
    #[error("timeout_ms must be in 1..{maximum_ms}")]
    InvalidTimeout { maximum_ms: u64 },
    #[error("Bluetooth motor is disconnected")]
    Disconnected,
    #[error("Bluetooth motor operation failed: {0}")]
    Backend(String),
}

pub trait MotorBackend: Send + 'static {
    fn connect(
        &mut self,
        name_prefix: &str,
        scan_timeout: Duration,
    ) -> impl Future<Output = Result<String, MotorError>> + Send;

    fn write(&mut self, command: &str) -> impl Future<Output = Result<(), MotorError>> + Send;

    fn disconnect(&mut self) -> impl Future<Output = Result<(), MotorError>> + Send;
}

#[derive(Debug)]
struct State {
    connected: bool,
    device: Option<String>,
    running: bool,
    speed_percent: u8,
    direction: MotorDirection,
    expires_at: Option<Instant>,
    error: Option<String>,
}

impl Default for State {
    fn default() -> Self {
        Self {
            connected: false,
            device: None,
            running: false,
            speed_percent: 0,
            direction: MotorDirection::Forward,
            expires_at: None,
            error: None,
        }
    }
}

struct Inner<B> {
    backend: AsyncMutex<B>,
    config: MotorConfig,
    state: Mutex<State>,
    supervisor: Mutex<Option<JoinHandle<()>>>,
}

pub struct MotorController<B> {
    inner: Arc<Inner<B>>,
}

pub type MotorHandle = Arc<dyn MotorService>;

pub trait MotorService: Send + Sync {
    fn status(&self) -> MotorStatus;

    fn set_running(
        &self,
        speed_percent: u8,
        direction: MotorDirection,
        timeout_ms: u64,
    ) -> Pin<Box<dyn Future<Output = Result<(), MotorError>> + Send + '_>>;

    fn stop(&self) -> Pin<Box<dyn Future<Output = Result<(), MotorError>> + Send + '_>>;

    fn reconnect(&self) -> Pin<Box<dyn Future<Output = Result<String, MotorError>> + Send + '_>>;

    fn close(&self) -> Pin<Box<dyn Future<Output = Result<(), MotorError>> + Send + '_>>;
}

impl<B: MotorBackend> MotorController<B> {
    pub fn with_backend(backend: B, config: MotorConfig) -> Result<Self, MotorError> {
        config.validate()?;
        Ok(Self {
            inner: Arc::new(Inner {
                backend: AsyncMutex::new(backend),
                config,
                state: Mutex::new(State::default()),
                supervisor: Mutex::new(None),
            }),
        })
    }

    pub async fn connect(&self) -> Result<String, MotorError> {
        self.stop_supervisor();
        let result = {
            let mut backend = self.inner.backend.lock().await;
            let device = backend
                .connect(
                    &self.inner.config.name_prefix,
                    self.inner.config.scan_timeout,
                )
                .await?;
            if let Err(error) = backend.write(SAFE_STOP_COMMAND).await {
                let _ = backend.disconnect().await;
                return self.record_error(error);
            }
            device
        };

        {
            let mut state = lock(&self.inner.state);
            state.connected = true;
            state.device = Some(result.clone());
            mark_stopped(&mut state);
            state.error = None;
        }
        self.start_supervisor();
        Ok(result)
    }

    pub fn status(&self) -> MotorStatus {
        let state = lock(&self.inner.state);
        MotorStatus {
            available: true,
            connected: state.connected,
            device: state.device.clone(),
            running: state.running,
            speed_percent: state.speed_percent,
            direction: state.direction,
            command_expires_in_ms: state.expires_at.map(|expires_at| {
                let millis = expires_at
                    .saturating_duration_since(Instant::now())
                    .as_millis();
                u64::try_from(millis).unwrap_or(u64::MAX)
            }),
            error: state.error.clone(),
        }
    }

    pub async fn set_running(
        &self,
        speed_percent: u8,
        direction: MotorDirection,
        timeout_ms: u64,
    ) -> Result<(), MotorError> {
        if !(1..=100).contains(&speed_percent) {
            return Err(MotorError::InvalidSpeed);
        }
        if !(1..=self.inner.config.maximum_command_timeout_ms).contains(&timeout_ms) {
            return Err(MotorError::InvalidTimeout {
                maximum_ms: self.inner.config.maximum_command_timeout_ms,
            });
        }

        let command = format!("D:{};B:0;S:{speed_percent};E:1\n", direction.wire_value());
        self.write(&command).await?;
        let mut state = lock(&self.inner.state);
        state.running = true;
        state.speed_percent = speed_percent;
        state.direction = direction;
        state.expires_at = Some(Instant::now() + Duration::from_millis(timeout_ms));
        state.error = None;
        Ok(())
    }

    pub async fn stop(&self) -> Result<(), MotorError> {
        self.write(SAFE_STOP_COMMAND).await?;
        mark_stopped(&mut lock(&self.inner.state));
        Ok(())
    }

    pub async fn reconnect(&self) -> Result<String, MotorError> {
        self.disconnect(true).await?;
        self.connect().await
    }

    pub async fn close(&self) -> Result<(), MotorError> {
        self.disconnect(true).await
    }

    async fn write(&self, command: &str) -> Result<(), MotorError> {
        if !lock(&self.inner.state).connected {
            return Err(MotorError::Disconnected);
        }
        let result = self.inner.backend.lock().await.write(command).await;
        if let Err(error) = result {
            return self.record_error(error);
        }
        Ok(())
    }

    async fn disconnect(&self, safe_stop: bool) -> Result<(), MotorError> {
        self.stop_supervisor();
        let connected = lock(&self.inner.state).connected;
        let result = {
            let mut backend = self.inner.backend.lock().await;
            let stop_result = if safe_stop && connected {
                backend.write(SAFE_STOP_COMMAND).await
            } else {
                Ok(())
            };
            let disconnect_result = backend.disconnect().await;
            stop_result.and(disconnect_result)
        };
        {
            let mut state = lock(&self.inner.state);
            state.connected = false;
            mark_stopped(&mut state);
            if let Err(error) = &result {
                state.error = Some(error.to_string());
            }
        }
        result
    }

    fn record_error<T>(&self, error: MotorError) -> Result<T, MotorError> {
        let mut state = lock(&self.inner.state);
        state.connected = false;
        mark_stopped(&mut state);
        state.error = Some(error.to_string());
        Err(error)
    }

    fn start_supervisor(&self) {
        let weak = Arc::downgrade(&self.inner);
        let interval = self.inner.config.heartbeat_interval;
        *lock(&self.inner.supervisor) = Some(tokio::spawn(supervise(weak, interval)));
    }

    fn stop_supervisor(&self) {
        if let Some(task) = lock(&self.inner.supervisor).take() {
            task.abort();
        }
    }
}

impl<B: MotorBackend> MotorService for MotorController<B> {
    fn status(&self) -> MotorStatus {
        MotorController::status(self)
    }

    fn set_running(
        &self,
        speed_percent: u8,
        direction: MotorDirection,
        timeout_ms: u64,
    ) -> Pin<Box<dyn Future<Output = Result<(), MotorError>> + Send + '_>> {
        Box::pin(MotorController::set_running(
            self,
            speed_percent,
            direction,
            timeout_ms,
        ))
    }

    fn stop(&self) -> Pin<Box<dyn Future<Output = Result<(), MotorError>> + Send + '_>> {
        Box::pin(MotorController::stop(self))
    }

    fn reconnect(&self) -> Pin<Box<dyn Future<Output = Result<String, MotorError>> + Send + '_>> {
        Box::pin(MotorController::reconnect(self))
    }

    fn close(&self) -> Pin<Box<dyn Future<Output = Result<(), MotorError>> + Send + '_>> {
        Box::pin(MotorController::close(self))
    }
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

fn mark_stopped(state: &mut State) {
    state.running = false;
    state.speed_percent = 0;
    state.expires_at = None;
}

async fn supervise<B: MotorBackend>(weak: Weak<Inner<B>>, interval: Duration) {
    loop {
        tokio::time::sleep(interval).await;
        let Some(inner) = weak.upgrade() else {
            return;
        };

        let expired = {
            let state = lock(&inner.state);
            state.running
                && state
                    .expires_at
                    .is_some_and(|expiry| Instant::now() >= expiry)
        };
        let mut backend = inner.backend.lock().await;
        let result = async {
            if expired {
                backend.write(SAFE_STOP_COMMAND).await?;
            }
            backend.write("H\n").await
        }
        .await;
        if result.is_err() {
            let _ = backend.write(SAFE_STOP_COMMAND).await;
        }
        drop(backend);

        let mut state = lock(&inner.state);
        match result {
            Ok(()) if expired => mark_stopped(&mut state),
            Ok(()) => {}
            Err(error) => {
                state.connected = false;
                mark_stopped(&mut state);
                state.error = Some(error.to_string());
                return;
            }
        }
    }
}

#[cfg(feature = "bluetooth")]
mod bluetooth {
    use super::{MotorBackend, MotorError, NUS_RX_UUID, NUS_SERVICE_UUID};
    use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter, WriteType};
    use btleplug::platform::{Adapter, Manager, Peripheral};
    use std::time::Duration;
    use uuid::Uuid;

    pub struct BtleplugBackend {
        adapter: Adapter,
        peripheral: Option<Peripheral>,
    }

    impl BtleplugBackend {
        pub async fn new() -> Result<Self, MotorError> {
            let manager = Manager::new().await.map_err(backend_error)?;
            let adapter = manager
                .adapters()
                .await
                .map_err(backend_error)?
                .into_iter()
                .next()
                .ok_or_else(|| MotorError::Backend("no Bluetooth adapter found".to_owned()))?;
            Ok(Self {
                adapter,
                peripheral: None,
            })
        }
    }

    impl MotorBackend for BtleplugBackend {
        async fn connect(
            &mut self,
            name_prefix: &str,
            scan_timeout: Duration,
        ) -> Result<String, MotorError> {
            self.adapter
                .start_scan(ScanFilter::default())
                .await
                .map_err(backend_error)?;
            let deadline = tokio::time::Instant::now() + scan_timeout;
            let found = 'scan: loop {
                for peripheral in self.adapter.peripherals().await.map_err(backend_error)? {
                    let properties = peripheral.properties().await.map_err(backend_error)?;
                    let name = properties.and_then(|value| value.local_name);
                    if name
                        .as_deref()
                        .is_some_and(|name| name.starts_with(name_prefix))
                    {
                        break 'scan (peripheral, name.unwrap_or_else(|| name_prefix.to_owned()));
                    }
                }
                if tokio::time::Instant::now() >= deadline {
                    self.adapter.stop_scan().await.map_err(backend_error)?;
                    return Err(MotorError::Backend(format!(
                        "No Bluetooth device named {name_prefix}* found"
                    )));
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            };
            self.adapter.stop_scan().await.map_err(backend_error)?;

            found.0.connect().await.map_err(backend_error)?;
            found.0.discover_services().await.map_err(backend_error)?;
            let service_uuid = Uuid::parse_str(NUS_SERVICE_UUID)
                .map_err(|error| MotorError::Backend(error.to_string()))?;
            if !found
                .0
                .services()
                .iter()
                .any(|service| service.uuid == service_uuid)
            {
                let _ = found.0.disconnect().await;
                return Err(MotorError::Backend(format!(
                    "{} does not expose the Nordic UART service",
                    found.1
                )));
            }
            self.peripheral = Some(found.0);
            Ok(found.1)
        }

        async fn write(&mut self, command: &str) -> Result<(), MotorError> {
            let peripheral = self.peripheral.as_ref().ok_or(MotorError::Disconnected)?;
            if !peripheral.is_connected().await.map_err(backend_error)? {
                return Err(MotorError::Disconnected);
            }
            let rx_uuid = Uuid::parse_str(NUS_RX_UUID)
                .map_err(|error| MotorError::Backend(error.to_string()))?;
            let characteristic = peripheral
                .characteristics()
                .into_iter()
                .find(|characteristic| characteristic.uuid == rx_uuid)
                .ok_or_else(|| {
                    MotorError::Backend("Nordic UART RX characteristic is missing".to_owned())
                })?;
            peripheral
                .write(
                    &characteristic,
                    command.as_bytes(),
                    WriteType::WithoutResponse,
                )
                .await
                .map_err(backend_error)
        }

        async fn disconnect(&mut self) -> Result<(), MotorError> {
            let Some(peripheral) = self.peripheral.take() else {
                return Ok(());
            };
            if peripheral.is_connected().await.map_err(backend_error)? {
                peripheral.disconnect().await.map_err(backend_error)?;
            }
            Ok(())
        }
    }

    fn backend_error(error: impl std::fmt::Display) -> MotorError {
        MotorError::Backend(error.to_string())
    }
}

#[cfg(feature = "bluetooth")]
pub use bluetooth::BtleplugBackend;

#[cfg(feature = "bluetooth")]
pub type BleMotorController = MotorController<BtleplugBackend>;

#[cfg(feature = "bluetooth")]
impl MotorController<BtleplugBackend> {
    pub async fn bluetooth(config: MotorConfig) -> Result<Self, MotorError> {
        Self::with_backend(BtleplugBackend::new().await?, config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Default)]
    struct MockBackend {
        calls: Arc<Mutex<Vec<String>>>,
    }

    impl MotorBackend for MockBackend {
        async fn connect(
            &mut self,
            name_prefix: &str,
            _scan_timeout: Duration,
        ) -> Result<String, MotorError> {
            lock(&self.calls).push(format!("connect:{name_prefix}"));
            Ok(format!("{name_prefix}-mock"))
        }

        async fn write(&mut self, command: &str) -> Result<(), MotorError> {
            lock(&self.calls).push(command.to_owned());
            Ok(())
        }

        async fn disconnect(&mut self) -> Result<(), MotorError> {
            lock(&self.calls).push("disconnect".to_owned());
            Ok(())
        }
    }

    fn test_controller(
        heartbeat_interval: Duration,
        maximum_command_timeout_ms: u64,
    ) -> (MotorController<MockBackend>, Arc<Mutex<Vec<String>>>) {
        let backend = MockBackend::default();
        let calls = Arc::clone(&backend.calls);
        let config = MotorConfig {
            heartbeat_interval,
            maximum_command_timeout_ms,
            ..MotorConfig::default()
        };
        (
            MotorController::with_backend(backend, config).unwrap(),
            calls,
        )
    }

    #[tokio::test]
    async fn command_expiry_sends_safe_stop() {
        let (motor, calls) = test_controller(Duration::from_millis(5), 100);
        assert_eq!(motor.connect().await.unwrap(), "BLDC-mock");

        motor
            .set_running(10, MotorDirection::Forward, 20)
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(40)).await;

        assert!(lock(&calls).contains(&"D:F;B:0;S:10;E:1\n".to_owned()));
        assert!(
            lock(&calls)
                .iter()
                .filter(|call| *call == SAFE_STOP_COMMAND)
                .count()
                >= 2
        );
        assert!(!motor.status().running);
        motor.close().await.unwrap();
    }

    #[tokio::test]
    async fn validates_commands_before_touching_backend() {
        let (motor, calls) = test_controller(Duration::from_secs(1), 10_000);

        assert_eq!(
            motor.set_running(0, MotorDirection::Forward, 1_000).await,
            Err(MotorError::InvalidSpeed)
        );
        assert_eq!(
            motor.set_running(101, MotorDirection::Forward, 1_000).await,
            Err(MotorError::InvalidSpeed)
        );
        assert_eq!(
            motor.set_running(10, MotorDirection::Forward, 0).await,
            Err(MotorError::InvalidTimeout { maximum_ms: 10_000 })
        );
        assert_eq!(
            motor.set_running(10, MotorDirection::Forward, 10_001).await,
            Err(MotorError::InvalidTimeout { maximum_ms: 10_000 })
        );
        assert!(lock(&calls).is_empty());
    }

    #[tokio::test]
    async fn stop_and_reconnect_are_safe() {
        let (motor, calls) = test_controller(Duration::from_secs(1), 10_000);
        motor.connect().await.unwrap();
        motor
            .set_running(25, MotorDirection::Reverse, 1_000)
            .await
            .unwrap();
        assert_eq!(
            motor.status(),
            MotorStatus {
                available: true,
                connected: true,
                device: Some("BLDC-mock".to_owned()),
                running: true,
                speed_percent: 25,
                direction: MotorDirection::Reverse,
                command_expires_in_ms: motor.status().command_expires_in_ms,
                error: None,
            }
        );

        motor.stop().await.unwrap();
        assert!(!motor.status().running);
        assert_eq!(motor.reconnect().await.unwrap(), "BLDC-mock");

        let calls = lock(&calls);
        assert_eq!(
            calls.iter().filter(|call| *call == "connect:BLDC").count(),
            2
        );
        assert!(
            calls
                .iter()
                .filter(|call| *call == SAFE_STOP_COMMAND)
                .count()
                >= 4
        );
        assert_eq!(calls.iter().filter(|call| *call == "disconnect").count(), 1);
    }
}
