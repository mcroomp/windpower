use std::collections::VecDeque;
use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::time::Duration;

use serde::{Deserialize, Serialize};
use thiserror::Error;
use tokio::sync::Mutex as AsyncMutex;
use tokio::task::JoinHandle;
use tokio::time::Instant;

pub const NUS_SERVICE_UUID: &str = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
pub const NUS_RX_UUID: &str = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
const SAFE_STOP_COMMAND: &str = "S:0;E:0;B:0\n";
const PROFILE_INTERVAL: Duration = Duration::from_millis(20);
// A full-scale move has 1 s acceleration, 3 s cruise, and 1 s deceleration.
const MAX_COMMAND_RATE: f64 = 25.0;
const MAX_COMMAND_ACCELERATION: f64 = 25.0;

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
pub struct MotorDevice {
    pub id: String,
    pub name: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MotorStatus {
    pub available: bool,
    pub connected: bool,
    pub device: Option<String>,
    pub device_id: Option<String>,
    pub running: bool,
    pub speed_percent: u8,
    pub target_speed_percent: u8,
    pub stopping: bool,
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
    #[error("This motor supports forward direction only")]
    InvalidDirection,
    #[error("Motor is stopping; wait until it is off before starting again")]
    Stopping,
    #[error("timeout_ms must be in 1..{maximum_ms}")]
    InvalidTimeout { maximum_ms: u64 },
    #[error("device_id must not be empty")]
    InvalidDevice,
    #[error("Stop the motor before scanning for Bluetooth devices")]
    Running,
    #[error("The selected Bluetooth motor is no longer connected")]
    DeviceChanged,
    #[error("Bluetooth motor is disconnected")]
    Disconnected,
    #[error("Bluetooth motor operation failed: {0}")]
    Backend(String),
}

pub trait MotorBackend: Send + 'static {
    fn scan(
        &mut self,
        name_prefix: &str,
        scan_timeout: Duration,
    ) -> impl Future<Output = Result<Vec<MotorDevice>, MotorError>> + Send;

    fn connect(
        &mut self,
        name_prefix: &str,
        device_id: Option<&str>,
        scan_timeout: Duration,
    ) -> impl Future<Output = Result<MotorDevice, MotorError>> + Send;

    fn write(&mut self, command: &str) -> impl Future<Output = Result<(), MotorError>> + Send;

    fn disconnect(&mut self) -> impl Future<Output = Result<(), MotorError>> + Send;
}

#[derive(Debug)]
struct Segment {
    seconds: f64,
    acceleration: f64,
}

#[derive(Debug, Default)]
struct MotionProfile {
    position: f64,
    velocity: f64,
    target: u8,
    segments: VecDeque<Segment>,
}

impl MotionProfile {
    fn retarget(&mut self, target: u8) {
        if target == self.target {
            return;
        }
        self.target = target;
        self.segments.clear();
        let mut position = self.position;
        let mut velocity = self.velocity;
        let distance = f64::from(target) - position;
        let stopping_distance = velocity * velocity / (2.0 * MAX_COMMAND_ACCELERATION);
        // A nearby or opposite target cannot instantaneously reverse the ramp.
        if velocity * distance < 0.0 || stopping_distance > distance.abs() {
            self.push_segment(
                velocity.abs() / MAX_COMMAND_ACCELERATION,
                -velocity.signum() * MAX_COMMAND_ACCELERATION,
            );
            position += velocity * velocity.abs() / (2.0 * MAX_COMMAND_ACCELERATION);
            velocity = 0.0;
        }

        let distance = f64::from(target) - position;
        let direction = distance.signum();
        let initial_rate = velocity * direction;
        let peak_rate = (MAX_COMMAND_ACCELERATION * distance.abs()
            + initial_rate * initial_rate / 2.0)
            .sqrt()
            .min(MAX_COMMAND_RATE);
        self.push_segment(
            (peak_rate - initial_rate) / MAX_COMMAND_ACCELERATION,
            direction * MAX_COMMAND_ACCELERATION,
        );
        let cruise_distance = distance.abs()
            - (2.0 * peak_rate * peak_rate - initial_rate * initial_rate)
                / (2.0 * MAX_COMMAND_ACCELERATION);
        if peak_rate > 0.0 {
            self.push_segment(cruise_distance / peak_rate, 0.0);
        }
        self.push_segment(
            peak_rate / MAX_COMMAND_ACCELERATION,
            -direction * MAX_COMMAND_ACCELERATION,
        );
    }

    fn push_segment(&mut self, seconds: f64, acceleration: f64) {
        if seconds > 1e-9 {
            self.segments.push_back(Segment {
                seconds,
                acceleration,
            });
        }
    }

    fn advance(&mut self, mut seconds: f64) {
        while let Some(segment) = self.segments.front_mut() {
            let step = seconds.min(segment.seconds);
            self.position += self.velocity * step + segment.acceleration * step * step / 2.0;
            self.velocity += segment.acceleration * step;
            segment.seconds -= step;
            seconds -= step;
            if segment.seconds <= 1e-9 {
                self.segments.pop_front();
            }
            if seconds <= 1e-9 {
                break;
            }
        }
        if self.segments.is_empty() {
            self.position = f64::from(self.target);
            self.velocity = 0.0;
        }
    }
}

#[derive(Debug)]
struct State {
    connected: bool,
    device: Option<String>,
    device_id: Option<String>,
    running: bool,
    speed_percent: u8,
    direction: MotorDirection,
    expires_at: Option<Instant>,
    error: Option<String>,
    profile: MotionProfile,
    stopping: bool,
    stop_epoch: u64,
    heartbeat_at: Instant,
    profile_at: Instant,
}

impl Default for State {
    fn default() -> Self {
        Self {
            connected: false,
            device: None,
            device_id: None,
            running: false,
            speed_percent: 0,
            direction: MotorDirection::Forward,
            expires_at: None,
            error: None,
            profile: MotionProfile::default(),
            stopping: false,
            stop_epoch: 0,
            heartbeat_at: Instant::now(),
            profile_at: Instant::now(),
        }
    }
}

struct Inner<B> {
    operation: AsyncMutex<()>,
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

    fn scan(
        &self,
    ) -> Pin<Box<dyn Future<Output = Result<Vec<MotorDevice>, MotorError>> + Send + '_>>;

    fn connect_device(
        &self,
        device_id: String,
    ) -> Pin<Box<dyn Future<Output = Result<String, MotorError>> + Send + '_>>;

    fn set_running(
        &self,
        device_id: String,
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
                operation: AsyncMutex::new(()),
                backend: AsyncMutex::new(backend),
                config,
                state: Mutex::new(State::default()),
                supervisor: Mutex::new(None),
            }),
        })
    }

    pub async fn connect(&self) -> Result<String, MotorError> {
        let _operation = self.inner.operation.lock().await;
        if lock(&self.inner.state).connected {
            self.disconnect().await?;
        }
        self.connect_inner().await
    }

    pub async fn scan(&self) -> Result<Vec<MotorDevice>, MotorError> {
        let _operation = self.inner.operation.lock().await;
        if lock(&self.inner.state).running {
            return Err(MotorError::Running);
        }
        self.inner
            .backend
            .lock()
            .await
            .scan(
                &self.inner.config.name_prefix,
                self.inner.config.scan_timeout,
            )
            .await
    }

    pub async fn connect_device(&self, device_id: String) -> Result<String, MotorError> {
        if device_id.trim().is_empty() {
            return Err(MotorError::InvalidDevice);
        }
        let _operation = self.inner.operation.lock().await;
        self.disconnect().await?;
        lock(&self.inner.state).device_id = Some(device_id);
        self.connect_inner().await
    }

    async fn connect_inner(&self) -> Result<String, MotorError> {
        self.stop_supervisor();
        let device_id = lock(&self.inner.state).device_id.clone();
        let result = {
            let mut backend = self.inner.backend.lock().await;
            let device = match backend
                .connect(
                    &self.inner.config.name_prefix,
                    device_id.as_deref(),
                    self.inner.config.scan_timeout,
                )
                .await
            {
                Ok(device) => device,
                Err(error) => return self.record_error(error),
            };
            if let Err(error) = backend.write(SAFE_STOP_COMMAND).await {
                let _ = backend.disconnect().await;
                return self.record_error(error);
            }
            device
        };

        {
            let mut state = lock(&self.inner.state);
            state.connected = true;
            state.device = Some(result.name.clone());
            state.device_id = Some(result.id);
            mark_stopped(&mut state);
            state.error = None;
        }
        self.start_supervisor();
        Ok(result.name)
    }

    pub fn status(&self) -> MotorStatus {
        let state = lock(&self.inner.state);
        MotorStatus {
            available: true,
            connected: state.connected,
            device: state.device.clone(),
            device_id: state.device_id.clone(),
            running: state.running,
            speed_percent: state.speed_percent,
            target_speed_percent: state.profile.target,
            stopping: state.stopping,
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
        device_id: &str,
        speed_percent: u8,
        direction: MotorDirection,
        timeout_ms: u64,
    ) -> Result<(), MotorError> {
        let epoch = {
            let state = lock(&self.inner.state);
            if state.stopping {
                return Err(MotorError::Stopping);
            }
            state.stop_epoch
        };
        let _operation = self.inner.operation.lock().await;
        if !(1..=100).contains(&speed_percent) {
            return Err(MotorError::InvalidSpeed);
        }
        if !(1..=self.inner.config.maximum_command_timeout_ms).contains(&timeout_ms) {
            return Err(MotorError::InvalidTimeout {
                maximum_ms: self.inner.config.maximum_command_timeout_ms,
            });
        }
        if direction != MotorDirection::Forward {
            return Err(MotorError::InvalidDirection);
        }
        if lock(&self.inner.state).device_id.as_deref() != Some(device_id) {
            return Err(MotorError::DeviceChanged);
        }
        let mut state = lock(&self.inner.state);
        if !state.connected {
            return Err(MotorError::Disconnected);
        }
        if state.stop_epoch != epoch || state.stopping {
            return Err(MotorError::Stopping);
        }
        if state
            .expires_at
            .is_some_and(|expiry| Instant::now() >= expiry)
        {
            begin_stop(&mut state);
            return Err(MotorError::Stopping);
        }
        state.running = true;
        state.profile.retarget(speed_percent);
        state.direction = direction;
        state.expires_at = Some(Instant::now() + Duration::from_millis(timeout_ms));
        state.error = None;
        Ok(())
    }

    pub async fn stop(&self) -> Result<(), MotorError> {
        let _operation = self.inner.operation.lock().await;
        self.smooth_stop().await
    }

    pub async fn reconnect(&self) -> Result<String, MotorError> {
        let _operation = self.inner.operation.lock().await;
        self.disconnect().await?;
        self.connect_inner().await
    }

    pub async fn close(&self) -> Result<(), MotorError> {
        let _operation = self.inner.operation.lock().await;
        self.disconnect().await
    }

    async fn smooth_stop(&self) -> Result<(), MotorError> {
        if !lock(&self.inner.state).connected {
            return Err(MotorError::Disconnected);
        }
        begin_stop(&mut lock(&self.inner.state));
        while lock(&self.inner.state).running {
            tokio::time::sleep(PROFILE_INTERVAL).await;
            advance_motor(&self.inner, PROFILE_INTERVAL).await?;
        }
        Ok(())
    }

    async fn disconnect(&self) -> Result<(), MotorError> {
        if lock(&self.inner.state).connected {
            self.smooth_stop().await?;
        }
        self.stop_supervisor();
        let result = self.inner.backend.lock().await.disconnect().await;
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
        record_motor_error(&self.inner, &error);
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

    fn scan(
        &self,
    ) -> Pin<Box<dyn Future<Output = Result<Vec<MotorDevice>, MotorError>> + Send + '_>> {
        Box::pin(MotorController::scan(self))
    }

    fn connect_device(
        &self,
        device_id: String,
    ) -> Pin<Box<dyn Future<Output = Result<String, MotorError>> + Send + '_>> {
        Box::pin(MotorController::connect_device(self, device_id))
    }

    fn set_running(
        &self,
        device_id: String,
        speed_percent: u8,
        direction: MotorDirection,
        timeout_ms: u64,
    ) -> Pin<Box<dyn Future<Output = Result<(), MotorError>> + Send + '_>> {
        Box::pin(async move {
            MotorController::set_running(self, &device_id, speed_percent, direction, timeout_ms)
                .await
        })
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
    state.profile = MotionProfile::default();
    state.stopping = false;
}

fn begin_stop(state: &mut State) {
    if !state.stopping {
        state.stop_epoch += 1;
        state.stopping = true;
        state.expires_at = None;
        state.profile.retarget(0);
    }
    if !state.running {
        state.stopping = false;
    }
}

fn record_motor_error<B>(inner: &Inner<B>, error: &MotorError) {
    let mut state = lock(&inner.state);
    state.connected = false;
    state.expires_at = None;
    state.error = Some(error.to_string());
    tracing::error!(%error, "Bluetooth motor state is unconfirmed");
}

async fn advance_motor<B: MotorBackend>(
    inner: &Inner<B>,
    step: Duration,
) -> Result<(), MotorError> {
    let (command, finished) = {
        let mut state = lock(&inner.state);
        if !state.connected {
            return Err(MotorError::Disconnected);
        }
        if state
            .expires_at
            .is_some_and(|expiry| Instant::now() >= expiry)
        {
            begin_stop(&mut state);
        }
        if !state.running {
            (None, false)
        } else {
            // Never catch up after a stalled task/link with a large command jump.
            let now = Instant::now();
            let elapsed = now.saturating_duration_since(state.profile_at).min(step);
            state.profile_at = now;
            state.profile.advance(elapsed.as_secs_f64());
            let finished = state.stopping && state.profile.segments.is_empty();
            let speed = state.profile.position.round().clamp(0.0, 100.0) as u8;
            let command = if finished {
                Some(SAFE_STOP_COMMAND.to_owned())
            } else if speed != state.speed_percent {
                Some(format!(
                    "D:{};B:0;S:{speed};E:1\n",
                    state.direction.wire_value()
                ))
            } else {
                None
            };
            state.speed_percent = speed;
            (command, finished)
        }
    };
    let result = async {
        let mut backend = inner.backend.lock().await;
        if let Some(command) = command {
            backend.write(&command).await?;
        }
        if Instant::now() >= lock(&inner.state).heartbeat_at {
            backend.write("H\n").await?;
            lock(&inner.state).heartbeat_at = Instant::now() + inner.config.heartbeat_interval;
        }
        Ok::<_, MotorError>(())
    }
    .await;
    if let Err(error) = result {
        record_motor_error(inner, &error);
        return Err(error);
    }
    if finished {
        mark_stopped(&mut lock(&inner.state));
    }
    Ok(())
}

async fn supervise<B: MotorBackend>(weak: Weak<Inner<B>>, interval: Duration) {
    let step = PROFILE_INTERVAL.min(interval);
    loop {
        tokio::time::sleep(step).await;
        let Some(inner) = weak.upgrade() else {
            return;
        };
        let _operation = inner.operation.lock().await;

        if advance_motor(&inner, step).await.is_err() {
            return;
        }
    }
}

mod bluetooth {
    use super::{MotorBackend, MotorDevice, MotorError, NUS_RX_UUID, NUS_SERVICE_UUID};
    use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter, WriteType};
    use btleplug::platform::{Adapter, Manager, Peripheral};
    use std::time::Duration;
    use uuid::Uuid;

    pub struct BtleplugBackend {
        adapter: Option<Adapter>,
        peripheral: Option<Peripheral>,
    }

    impl BtleplugBackend {
        pub async fn new() -> Result<Self, MotorError> {
            Ok(Self {
                adapter: None,
                peripheral: None,
            })
        }

        async fn adapter(&mut self) -> Result<&Adapter, MotorError> {
            if self.adapter.is_none() {
                let manager = Manager::new().await.map_err(backend_error)?;
                let adapter = manager
                    .adapters()
                    .await
                    .map_err(backend_error)?
                    .into_iter()
                    .next()
                    .ok_or_else(|| MotorError::Backend("no Bluetooth adapter found".to_owned()))?;
                self.adapter = Some(adapter);
            }
            self.adapter
                .as_ref()
                .ok_or_else(|| MotorError::Backend("no Bluetooth adapter found".to_owned()))
        }

        async fn devices(
            &mut self,
            name_prefix: &str,
        ) -> Result<Vec<(Peripheral, MotorDevice)>, MotorError> {
            let mut devices = Vec::new();
            for peripheral in self
                .adapter()
                .await?
                .peripherals()
                .await
                .map_err(backend_error)?
            {
                let properties = peripheral.properties().await.map_err(backend_error)?;
                if let Some(name) = properties.and_then(|value| value.local_name)
                    && name.starts_with(name_prefix)
                {
                    let id = peripheral.id().to_string();
                    devices.push((peripheral, MotorDevice { id, name }));
                }
            }
            devices.sort_by(|a, b| a.1.id.cmp(&b.1.id));
            Ok(devices)
        }
    }

    impl MotorBackend for BtleplugBackend {
        async fn scan(
            &mut self,
            name_prefix: &str,
            scan_timeout: Duration,
        ) -> Result<Vec<MotorDevice>, MotorError> {
            self.adapter()
                .await?
                .start_scan(ScanFilter::default())
                .await
                .map_err(backend_error)?;
            tokio::time::sleep(scan_timeout).await;
            let result = self.devices(name_prefix).await;
            self.adapter()
                .await?
                .stop_scan()
                .await
                .map_err(backend_error)?;
            Ok(result?.into_iter().map(|(_, device)| device).collect())
        }

        async fn connect(
            &mut self,
            name_prefix: &str,
            device_id: Option<&str>,
            scan_timeout: Duration,
        ) -> Result<MotorDevice, MotorError> {
            self.adapter()
                .await?
                .start_scan(ScanFilter::default())
                .await
                .map_err(backend_error)?;
            let deadline = tokio::time::Instant::now() + scan_timeout;
            let result = async {
                loop {
                    for (peripheral, device) in self.devices(name_prefix).await? {
                        if device_id.is_none_or(|id| id == device.id) {
                            return Ok((peripheral, device));
                        }
                    }
                    if tokio::time::Instant::now() >= deadline {
                        return Err(MotorError::Backend(format!(
                            "No Bluetooth motor found for {}",
                            device_id.unwrap_or(name_prefix)
                        )));
                    }
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            }
            .await;
            self.adapter()
                .await?
                .stop_scan()
                .await
                .map_err(backend_error)?;
            let found = result?;

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
                    found.1.name
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

pub use bluetooth::BtleplugBackend;

pub type BleMotorController = MotorController<BtleplugBackend>;

impl MotorController<BtleplugBackend> {
    pub async fn bluetooth(config: MotorConfig) -> Result<Self, MotorError> {
        Self::with_backend(BtleplugBackend::new().await?, config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::{
        Router,
        body::{Body, to_bytes},
        http::{Request, StatusCode},
    };
    use serde_json::{Value, json};
    use tower::ServiceExt;

    #[derive(Clone, Default)]
    struct MockBackend {
        calls: Arc<Mutex<Vec<String>>>,
        fail_writes: Arc<Mutex<bool>>,
    }

    impl MotorBackend for MockBackend {
        async fn scan(
            &mut self,
            name_prefix: &str,
            _scan_timeout: Duration,
        ) -> Result<Vec<MotorDevice>, MotorError> {
            lock(&self.calls).push("scan".to_owned());
            Ok(vec![MotorDevice {
                id: "motor-1".to_owned(),
                name: format!("{name_prefix}-mock"),
            }])
        }

        async fn connect(
            &mut self,
            name_prefix: &str,
            device_id: Option<&str>,
            _scan_timeout: Duration,
        ) -> Result<MotorDevice, MotorError> {
            lock(&self.calls).push(format!("connect:{}", device_id.unwrap_or(name_prefix)));
            Ok(MotorDevice {
                id: device_id.unwrap_or("motor-1").to_owned(),
                name: format!("{name_prefix}-mock"),
            })
        }

        async fn write(&mut self, command: &str) -> Result<(), MotorError> {
            lock(&self.calls).push(command.to_owned());
            if *lock(&self.fail_writes) {
                return Err(MotorError::Backend("mock write failure".to_owned()));
            }
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

    async fn elapse(milliseconds: u64) {
        tokio::task::yield_now().await;
        for _ in 0..milliseconds / 20 {
            tokio::time::advance(PROFILE_INTERVAL).await;
            tokio::task::yield_now().await;
        }
    }

    fn check_profile_step(profile: &mut MotionProfile) {
        let position = profile.position;
        let velocity = profile.velocity;
        profile.advance(0.02);
        assert!(
            (-1e-8..=100.0 + 1e-8).contains(&profile.position),
            "{profile:?}"
        );
        assert!(profile.velocity.abs() <= MAX_COMMAND_RATE + 1e-8);
        assert!((profile.velocity - velocity).abs() <= MAX_COMMAND_ACCELERATION * 0.02 + 1e-8);
        assert!((profile.position - position).abs() <= MAX_COMMAND_RATE * 0.02 + 1e-8);
    }

    #[test]
    fn full_scale_profile_takes_five_seconds_in_both_directions() {
        let mut profile = MotionProfile::default();
        for target in [100, 0] {
            profile.retarget(target);
            for _ in 0..249 {
                check_profile_step(&mut profile);
                assert!(!profile.segments.is_empty());
            }
            check_profile_step(&mut profile);
            assert!(profile.segments.is_empty());
            assert_eq!(profile.position, f64::from(target));
            assert_eq!(profile.velocity, 0.0);
        }
    }

    #[test]
    fn arbitrary_retargeting_preserves_command_and_rate_limits() {
        let mut profile = MotionProfile::default();
        let mut random = 17_u32;
        for _ in 0..5_000 {
            random = random.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let position = profile.position;
            let velocity = profile.velocity;
            profile.retarget((random % 101) as u8);
            assert_eq!(profile.position, position);
            assert_eq!(profile.velocity, velocity);
            for _ in 0..7 {
                check_profile_step(&mut profile);
            }
        }
        profile.retarget(0);
        for _ in 0..350 {
            check_profile_step(&mut profile);
        }
        assert_eq!(profile.position, 0.0);
        assert!(profile.segments.is_empty());
    }

    #[test]
    fn renewing_the_same_target_does_not_restart_the_profile() {
        let mut profile = MotionProfile::default();
        profile.retarget(100);
        for _ in 0..250 {
            profile.retarget(100);
            check_profile_step(&mut profile);
        }
        assert_eq!(profile.position, 100.0);
        assert!(profile.segments.is_empty());
    }

    #[tokio::test(start_paused = true)]
    async fn run_targets_are_applied_gradually_and_renewals_preserve_progress() {
        let (motor, calls) = test_controller(Duration::from_millis(500), 10_000);
        motor.connect().await.unwrap();
        motor
            .set_running("motor-1", 100, MotorDirection::Forward, 2_000)
            .await
            .unwrap();
        assert_eq!(motor.status().speed_percent, 0);
        assert_eq!(motor.status().target_speed_percent, 100);
        assert!(!lock(&calls).iter().any(|call| call.starts_with("D:")));
        for _ in 0..10 {
            elapse(500).await;
            motor
                .set_running("motor-1", 100, MotorDirection::Forward, 2_000)
                .await
                .unwrap();
        }
        assert_eq!(motor.status().speed_percent, 100);
        {
            let calls = lock(&calls);
            let speeds: Vec<u8> = calls
                .iter()
                .filter_map(|call| {
                    call.strip_prefix("D:F;B:0;S:")?
                        .strip_suffix(";E:1\n")?
                        .parse()
                        .ok()
                })
                .collect();
            assert_eq!(speeds, (1..=100).collect::<Vec<_>>());
        }
        motor.close().await.unwrap();
    }

    #[tokio::test(start_paused = true)]
    async fn expired_lease_cannot_be_renewed_before_supervisor_notices_it() {
        let (motor, _) = test_controller(Duration::from_millis(500), 10_000);
        motor.connect().await.unwrap();
        motor
            .set_running("motor-1", 100, MotorDirection::Forward, 20)
            .await
            .unwrap();
        // The supervisor has not been scheduled yet.
        tokio::time::advance(Duration::from_millis(40)).await;
        assert_eq!(
            motor
                .set_running("motor-1", 100, MotorDirection::Forward, 2_000)
                .await,
            Err(MotorError::Stopping)
        );
        assert_eq!(motor.status().target_speed_percent, 0);
        motor.close().await.unwrap();
    }

    #[tokio::test(start_paused = true)]
    async fn command_expiry_ramps_down_before_safe_stop() {
        let (motor, calls) = test_controller(Duration::from_millis(500), 10_000);
        assert_eq!(motor.connect().await.unwrap(), "BLDC-mock");
        motor
            .set_running("motor-1", 100, MotorDirection::Forward, 1_200)
            .await
            .unwrap();
        elapse(1_180).await;
        assert!(motor.status().running);
        assert!(motor.status().speed_percent > 0);
        elapse(40).await;
        assert!(motor.status().stopping);
        assert_eq!(motor.status().target_speed_percent, 0);
        assert!(motor.status().running);
        assert_eq!(
            lock(&calls)
                .iter()
                .filter(|call| *call == SAFE_STOP_COMMAND)
                .count(),
            1
        );
        assert_eq!(
            motor
                .set_running("motor-1", 100, MotorDirection::Forward, 2_000)
                .await,
            Err(MotorError::Stopping)
        );
        elapse(6_000).await;
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

    #[tokio::test(start_paused = true)]
    async fn cancelled_stop_is_completed_by_the_supervisor() {
        let (motor, calls) = test_controller(Duration::from_millis(500), 10_000);
        let motor = Arc::new(motor);
        motor.connect().await.unwrap();
        motor
            .set_running("motor-1", 100, MotorDirection::Forward, 10_000)
            .await
            .unwrap();
        elapse(5_000).await;
        assert_eq!(motor.status().speed_percent, 100);
        let stopping = {
            let motor = motor.clone();
            tokio::spawn(async move { motor.stop().await })
        };
        tokio::task::yield_now().await;
        assert!(motor.status().stopping);
        stopping.abort();
        assert!(stopping.await.unwrap_err().is_cancelled());
        elapse(1_000).await;
        assert!(motor.status().running);
        assert!(motor.status().speed_percent < 100);
        assert_eq!(
            lock(&calls)
                .iter()
                .filter(|call| *call == SAFE_STOP_COMMAND)
                .count(),
            1
        );
        elapse(4_200).await;
        assert!(!motor.status().running);
        assert_eq!(motor.status().speed_percent, 0);
        assert_eq!(
            lock(&calls)
                .iter()
                .filter(|call| *call == SAFE_STOP_COMMAND)
                .count(),
            2
        );
        motor.close().await.unwrap();
    }

    #[tokio::test(start_paused = true)]
    async fn explicit_stop_keeps_heartbeats_and_ramps_before_disabling_without_braking() {
        let (motor, calls) = test_controller(Duration::from_millis(500), 10_000);
        motor.connect().await.unwrap();
        motor
            .set_running("motor-1", 100, MotorDirection::Forward, 10_000)
            .await
            .unwrap();
        elapse(5_000).await;
        let before = lock(&calls).len();
        let started = Instant::now();
        motor.stop().await.unwrap();
        assert!(started.elapsed() >= Duration::from_secs(5));
        let calls = lock(&calls);
        assert_eq!(
            calls.iter().filter(|call| *call == "S:0;E:0;B:0\n").count(),
            2
        );
        assert!(calls.iter().all(|call| !call.contains("B:1")));
        let stopping = &calls[before..];
        assert!(stopping.iter().filter(|call| *call == "H\n").count() >= 9);
        let speeds: Vec<u8> = stopping
            .iter()
            .filter_map(|call| {
                call.strip_prefix("D:F;B:0;S:")?
                    .strip_suffix(";E:1\n")?
                    .parse()
                    .ok()
            })
            .collect();
        assert_eq!(speeds.first(), Some(&99));
        assert_eq!(speeds.last(), Some(&0));
        assert!(speeds.windows(2).all(|pair| pair[0] - pair[1] == 1));
        assert_eq!(
            stopping
                .iter()
                .filter(|call| *call == SAFE_STOP_COMMAND)
                .count(),
            1
        );
    }

    #[tokio::test(start_paused = true)]
    async fn delayed_supervisor_does_not_catch_up_with_a_large_jump() {
        let (motor, _) = test_controller(Duration::from_millis(500), 10_000);
        motor.connect().await.unwrap();
        motor
            .set_running("motor-1", 100, MotorDirection::Forward, 10_000)
            .await
            .unwrap();
        elapse(1_000).await;
        let operation = motor.inner.operation.lock().await;
        let before = motor.status().speed_percent;
        tokio::time::advance(Duration::from_secs(2)).await;
        tokio::task::yield_now().await;
        drop(operation);
        tokio::task::yield_now().await;
        assert!(motor.status().speed_percent <= before + 1);
        motor.close().await.unwrap();
    }

    #[tokio::test(start_paused = true)]
    async fn failed_ramp_write_reports_uncertainty_without_abrupt_brake() {
        let backend = MockBackend::default();
        let failures = backend.fail_writes.clone();
        let calls = backend.calls.clone();
        let motor = MotorController::with_backend(backend, MotorConfig::default()).unwrap();
        motor.connect().await.unwrap();
        motor
            .set_running("motor-1", 100, MotorDirection::Forward, 10_000)
            .await
            .unwrap();
        elapse(5_000).await;
        *lock(&failures) = true;
        assert!(motor.stop().await.is_err());
        assert!(!motor.status().connected);
        assert!(motor.status().running);
        assert!(motor.status().error.unwrap().contains("mock write failure"));
        assert_eq!(
            lock(&calls)
                .iter()
                .filter(|call| *call == SAFE_STOP_COMMAND)
                .count(),
            1
        );
    }

    #[tokio::test]
    async fn validates_commands_before_touching_backend() {
        let (motor, calls) = test_controller(Duration::from_secs(1), 10_000);

        assert_eq!(
            motor
                .set_running("motor-1", 10, MotorDirection::Reverse, 1_000)
                .await,
            Err(MotorError::InvalidDirection)
        );
        assert_eq!(
            motor
                .set_running("motor-1", 0, MotorDirection::Forward, 1_000)
                .await,
            Err(MotorError::InvalidSpeed)
        );
        assert_eq!(
            motor
                .set_running("motor-1", 101, MotorDirection::Forward, 1_000)
                .await,
            Err(MotorError::InvalidSpeed)
        );
        assert_eq!(
            motor
                .set_running("motor-1", 10, MotorDirection::Forward, 0)
                .await,
            Err(MotorError::InvalidTimeout { maximum_ms: 10_000 })
        );
        assert_eq!(
            motor
                .set_running("motor-1", 10, MotorDirection::Forward, 10_001)
                .await,
            Err(MotorError::InvalidTimeout { maximum_ms: 10_000 })
        );
        assert!(lock(&calls).is_empty());
    }

    #[tokio::test(start_paused = true)]
    async fn stop_and_reconnect_are_safe() {
        let (motor, calls) = test_controller(Duration::from_secs(1), 10_000);
        motor.connect().await.unwrap();
        motor
            .set_running("motor-1", 25, MotorDirection::Forward, 1_000)
            .await
            .unwrap();
        assert_eq!(
            motor.status(),
            MotorStatus {
                available: true,
                connected: true,
                device: Some("BLDC-mock".to_owned()),
                device_id: Some("motor-1".to_owned()),
                running: true,
                speed_percent: 0,
                target_speed_percent: 25,
                stopping: false,
                direction: MotorDirection::Forward,
                command_expires_in_ms: motor.status().command_expires_in_ms,
                error: None,
            }
        );

        motor.stop().await.unwrap();
        assert!(!motor.status().running);
        assert_eq!(motor.reconnect().await.unwrap(), "BLDC-mock");

        let calls = lock(&calls);
        assert_eq!(
            calls
                .iter()
                .filter(|call| *call == "connect:motor-1")
                .count(),
            1
        );
        assert!(
            calls
                .iter()
                .filter(|call| *call == SAFE_STOP_COMMAND)
                .count()
                >= 3
        );
        assert_eq!(calls.iter().filter(|call| *call == "disconnect").count(), 1);
    }

    #[tokio::test(start_paused = true)]
    async fn device_selection_stops_previous_motor_and_never_starts_selected_motor() {
        let (motor, calls) = test_controller(Duration::from_secs(1), 10_000);
        assert_eq!(motor.scan().await.unwrap()[0].id, "motor-1");
        motor.connect().await.unwrap();
        motor
            .set_running("motor-1", 100, MotorDirection::Forward, 10_000)
            .await
            .unwrap();
        elapse(5_000).await;
        assert_eq!(motor.scan().await, Err(MotorError::Running));
        let started = Instant::now();
        motor.connect_device("motor-2".to_owned()).await.unwrap();
        assert!(started.elapsed() >= Duration::from_secs(5));
        assert!(!motor.status().running);
        assert_eq!(motor.status().device_id.as_deref(), Some("motor-2"));
        let calls: Vec<_> = lock(&calls)
            .iter()
            .filter(|call| *call != "H\n")
            .cloned()
            .collect();
        let selection = calls
            .iter()
            .position(|call| call == "connect:motor-2")
            .unwrap();
        assert_eq!(
            &calls[selection - 2..=selection + 1],
            &[
                SAFE_STOP_COMMAND.to_owned(),
                "disconnect".to_owned(),
                "connect:motor-2".to_owned(),
                SAFE_STOP_COMMAND.to_owned(),
            ]
        );
    }

    #[tokio::test(start_paused = true)]
    async fn reconnect_and_close_wait_for_the_same_ramp() {
        let (motor, calls) = test_controller(Duration::from_millis(500), 10_000);
        motor.connect().await.unwrap();
        for reconnect in [true, false] {
            motor
                .set_running("motor-1", 100, MotorDirection::Forward, 10_000)
                .await
                .unwrap();
            elapse(5_000).await;
            let started = Instant::now();
            if reconnect {
                motor.reconnect().await.unwrap();
            } else {
                motor.close().await.unwrap();
            }
            assert!(started.elapsed() >= Duration::from_secs(5));
            assert!(!motor.status().running);
            assert_eq!(motor.status().connected, reconnect);
        }
        let calls: Vec<_> = lock(&calls)
            .iter()
            .filter(|call| *call != "H\n")
            .cloned()
            .collect();
        for (index, call) in calls.iter().enumerate() {
            if call == "disconnect" {
                assert_eq!(calls[index - 1], "S:0;E:0;B:0\n");
                assert_eq!(calls[index - 2], "D:F;B:0;S:0;E:1\n");
            }
        }
        assert!(calls.iter().all(|call| !call.contains("B:1")));
    }

    #[tokio::test(start_paused = true)]
    async fn a_run_queued_before_stop_cannot_restart_the_motor_afterwards() {
        let (motor, _) = test_controller(Duration::from_millis(500), 10_000);
        let motor = Arc::new(motor);
        motor.connect().await.unwrap();
        let operation = motor.inner.operation.lock().await;
        let pending = {
            let motor = motor.clone();
            tokio::spawn(async move {
                motor
                    .set_running("motor-1", 100, MotorDirection::Forward, 10_000)
                    .await
            })
        };
        tokio::task::yield_now().await;
        begin_stop(&mut lock(&motor.inner.state));
        drop(operation);
        assert_eq!(pending.await.unwrap(), Err(MotorError::Stopping));
        assert!(!motor.status().running);
        motor.close().await.unwrap();
    }

    #[tokio::test]
    async fn empty_device_selection_does_not_touch_backend() {
        let (motor, calls) = test_controller(Duration::from_secs(1), 10_000);
        assert_eq!(
            motor.connect_device(" ".to_owned()).await,
            Err(MotorError::InvalidDevice)
        );
        assert!(lock(&calls).is_empty());
    }

    #[tokio::test]
    async fn stale_commands_cannot_power_a_different_device() {
        let (motor, calls) = test_controller(Duration::from_secs(1), 10_000);
        motor.connect_device("motor-2".to_owned()).await.unwrap();
        let before = lock(&calls).len();
        assert_eq!(
            motor
                .set_running("motor-1", 25, MotorDirection::Forward, 2_000)
                .await,
            Err(MotorError::DeviceChanged)
        );
        assert_eq!(lock(&calls).len(), before);
        assert!(!motor.status().running);
    }

    #[tokio::test]
    async fn http_device_selection_and_leased_commands() {
        async fn call(app: &Router, method: &str, path: &str, body: Value) -> (StatusCode, Value) {
            let response = app
                .clone()
                .oneshot(
                    Request::builder()
                        .method(method)
                        .uri(path)
                        .header("content-type", "application/json")
                        .body(Body::from(body.to_string()))
                        .unwrap(),
                )
                .await
                .unwrap();
            let status = response.status();
            let bytes = to_bytes(response.into_body(), usize::MAX).await.unwrap();
            (status, serde_json::from_slice(&bytes).unwrap())
        }

        let directory = tempfile::TempDir::new().unwrap();
        let (journal, task) = crate::journal::JournalHandle::start(
            crate::journal::JournalConfig::for_directory(directory.path(), uuid::Uuid::new_v4()),
        )
        .await
        .unwrap();
        let (motor, _) = test_controller(Duration::from_secs(1), 10_000);
        let motor = Arc::new(motor);
        let app = crate::http::router_with_motor(journal.clone(), None, Some(motor.clone()));

        let (code, devices) = call(&app, "GET", "/v1/motor/devices", json!({})).await;
        assert_eq!(code, StatusCode::OK);
        assert_eq!(devices["devices"][0]["id"], "motor-1");

        let (code, status) = call(
            &app,
            "POST",
            "/v1/motor/connect",
            json!({"device_id": "motor-1"}),
        )
        .await;
        assert_eq!(code, StatusCode::OK);
        assert_eq!(status["connected"], true);
        assert_eq!(status["running"], false);
        assert_eq!(status["device_id"], "motor-1");

        let (code, _) = call(&app, "PUT", "/v1/motor", json!({
            "device_id": "wrong", "speed_percent": 10, "direction": "forward", "timeout_ms": 2000
        })).await;
        assert_eq!(code, StatusCode::CONFLICT);
        let (code, error) = call(&app, "PUT", "/v1/motor", json!({
            "device_id": "motor-1", "speed_percent": 10, "direction": "reverse", "timeout_ms": 2000
        })).await;
        assert_eq!(code, StatusCode::BAD_REQUEST);
        assert_eq!(error["error"], "invalid_motor_command");
        let (code, status) = call(&app, "PUT", "/v1/motor", json!({
            "device_id": "motor-1", "speed_percent": 10, "direction": "forward", "timeout_ms": 2000
        })).await;
        assert_eq!(code, StatusCode::OK);
        assert_eq!(status["running"], true);
        let (code, _) = call(&app, "GET", "/v1/motor/devices", json!({})).await;
        assert_eq!(code, StatusCode::CONFLICT);
        let (code, status) = call(&app, "POST", "/v1/motor/stop", json!({})).await;
        assert_eq!(code, StatusCode::OK);
        assert_eq!(status["running"], false);
        motor.close().await.unwrap();
        journal.shutdown().await.unwrap();
        task.await.unwrap();
    }
}
