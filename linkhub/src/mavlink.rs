use std::{
    collections::{BTreeMap, VecDeque},
    io,
    net::SocketAddr,
    sync::{
        Arc,
        atomic::{AtomicU8, Ordering},
    },
    time::Duration,
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

use crate::{
    codec::{
        CodecError, DecodedMessage, FrameReader, MavHeader, MavMessage, decode_raw_bytes,
        dialect::{HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavState, MavType},
        message_from_fields, serialize_message,
    },
    discovery::{DiscoveryConfig, ScanReport},
    journal::{JournalError, JournalHandle},
    link_events::{FailureRepeats, LinkEvent, LinkEvents},
    records::{Direction, MavlinkFrame, RecordPayload, wall_time_ns},
};

pub(crate) const HEARTBEAT_MESSAGE_ID: u32 = 0;

/// Where the link is in its acquire/use/reconnect cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum LinkPhase {
    #[default]
    Idle,
    /// Enumerating and probing serial ports for a heartbeat.
    Scanning,
    /// Opening the resolved serial port or TCP address.
    Opening,
    /// Transport open, waiting for the first vehicle heartbeat.
    Acquiring,
    /// Vehicle heartbeat received; the link is usable.
    Ready,
    /// Waiting `reconnect_interval` after a failure before retrying.
    Backoff,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct LinkStatus {
    pub connected: bool,
    pub ready: bool,
    pub phase: LinkPhase,
    pub connection: String,
    /// Serial port currently in use, if connected over serial. `None` while
    /// scanning, disconnected, or connected over TCP — not connected is an
    /// ordinary, frequently-occurring state, not an error.
    pub port: Option<String>,
    /// Baud rate currently in use, if connected over serial.
    pub baud: Option<u32>,
    pub clock_epoch: u64,
    pub target_system: u8,
    pub target_component: u8,
    pub base_mode: MavModeFlag,
    pub custom_mode: u32,
    pub system_status: MavState,
    pub latest_time_boot_ms: u64,
    pub received_messages: u64,
    pub transmitted_messages: u64,
    pub received_bytes: u64,
    pub transmitted_bytes: u64,
    pub rx_bps: Option<f64>,
    pub tx_bps: Option<f64>,
    /// Lifetime validated frame bytes per MAVLink message name.
    pub received_bytes_by_message: BTreeMap<String, u64>,
    pub transmitted_bytes_by_message: BTreeMap<String, u64>,
    /// Windowed bits per second per message name; idle types are omitted.
    pub rx_bps_by_message: BTreeMap<String, f64>,
    pub tx_bps_by_message: BTreeMap<String, f64>,
    /// Frames that passed their CRC but did not decode as a typed dialect
    /// message (for example an enum value outside the dialect) and were dropped.
    pub dropped_frames: u64,
    pub last_received_ns: Option<u64>,
    pub error: Option<String>,
    /// Which step produced `error`: `scan`, `open`, `acquire`, or `link`.
    pub last_error_stage: Option<String>,
    pub last_error_ns: Option<u64>,
    /// Transports opened since start (each successful serial/TCP open).
    pub attempts: u64,
    /// Failed scans/opens/sessions since the link was last ready.
    pub consecutive_failures: u64,
    /// When the current transport was opened, if connected.
    pub connected_since_ns: Option<u64>,
    /// Most recent serial discovery scan, with per-port probe outcomes.
    pub last_scan: Option<ScanReport>,
}

struct LinkThroughput {
    samples: VecDeque<(time::Instant, u64, u64)>,
}

impl LinkThroughput {
    fn new(now: time::Instant, rx: u64, tx: u64) -> Self {
        Self {
            samples: VecDeque::from([(now, rx, tx)]),
        }
    }

    fn sample(&mut self, now: time::Instant, rx: u64, tx: u64) -> (f64, f64) {
        self.samples.push_back((now, rx, tx));
        let cutoff = now - Duration::from_secs(3);
        while self.samples.len() > 2 && self.samples[1].0 <= cutoff {
            self.samples.pop_front();
        }
        let &(start, start_rx, start_tx) = self.samples.front().expect("initial sample");
        let mut rx_delta = (rx - start_rx) as f64;
        let mut tx_delta = (tx - start_tx) as f64;
        let window_start = start.max(cutoff);
        if start < cutoff {
            let &(next, next_rx, next_tx) = &self.samples[1];
            let fraction = cutoff.duration_since(start).as_secs_f64()
                / next.duration_since(start).as_secs_f64();
            rx_delta -= (next_rx - start_rx) as f64 * fraction;
            tx_delta -= (next_tx - start_tx) as f64 * fraction;
        }
        let elapsed = now.duration_since(window_start).as_secs_f64();
        (rx_delta * 8.0 / elapsed, tx_delta * 8.0 / elapsed)
    }
}

type ByteCounts = BTreeMap<String, u64>;

/// Per-message-name counterpart of [`LinkThroughput`], using the same
/// 3-second window and boundary interpolation.
struct MessageThroughput {
    samples: VecDeque<(time::Instant, ByteCounts)>,
}

impl MessageThroughput {
    fn new(now: time::Instant, counts: &ByteCounts) -> Self {
        Self {
            samples: VecDeque::from([(now, counts.clone())]),
        }
    }

    fn sample(&mut self, now: time::Instant, counts: &ByteCounts) -> BTreeMap<String, f64> {
        self.samples.push_back((now, counts.clone()));
        let cutoff = now - Duration::from_secs(3);
        while self.samples.len() > 2 && self.samples[1].0 <= cutoff {
            self.samples.pop_front();
        }
        let (start, start_counts) = self.samples.front().expect("initial sample");
        let window_start = (*start).max(cutoff);
        let elapsed = now.duration_since(window_start).as_secs_f64();
        let rolled_off = (*start < cutoff).then(|| {
            let (next, next_counts) = &self.samples[1];
            let fraction = cutoff.duration_since(*start).as_secs_f64()
                / next.duration_since(*start).as_secs_f64();
            (fraction, next_counts)
        });
        counts
            .iter()
            .filter_map(|(name, &total)| {
                let base = start_counts.get(name).copied().unwrap_or(0);
                let mut delta = (total - base) as f64;
                if let Some((fraction, next_counts)) = rolled_off {
                    let next_total = next_counts.get(name).copied().unwrap_or(0);
                    delta -= (next_total - base) as f64 * fraction;
                }
                (delta > 0.0).then(|| (name.clone(), delta * 8.0 / elapsed))
            })
            .collect()
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct ComponentInfo {
    pub system_id: u8,
    pub component_id: u8,
    pub vehicle_type: MavType,
    pub autopilot: MavAutopilot,
    pub base_mode: MavModeFlag,
    pub custom_mode: u32,
    pub system_status: MavState,
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
    /// Discover the serial port by scanning for a MAVLink heartbeat. This is
    /// the normal way a serial link is acquired; a port and/or baud rate in
    /// `DiscoveryConfig` only restrict which candidates are probed, they do
    /// not skip the heartbeat confirmation. The scan repeats on every
    /// reconnect, so an unplugged or renumbered port is rediscovered.
    Serial(DiscoveryConfig),
}

impl std::fmt::Display for MavlinkTransport {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Tcp(address) => write!(formatter, "tcp:{address}"),
            Self::Serial(discovery) => match &discovery.port_filter {
                Some(port) => write!(formatter, "auto:{port}"),
                None => write!(formatter, "auto"),
            },
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

    /// Discover the serial port, rescanning whenever the link drops. Restrict
    /// candidates via `discovery.port_filter` / `discovery.bauds` if needed.
    #[must_use]
    pub fn serial(discovery: DiscoveryConfig) -> Self {
        Self {
            id: "mavlink".to_owned(),
            transport: MavlinkTransport::Serial(discovery),
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
    pub message: MavMessage,
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

    /// Sends a typed message, journaling it on the shared link timeline.
    pub async fn send_message(
        &self,
        message: MavMessage,
        source_system: Option<u8>,
        source_component: Option<u8>,
    ) -> Result<u64, LinkError> {
        self.send(OutboundPayload::Message {
            message: Box::new(message),
            source_system,
            source_component,
        })
        .await
    }

    /// Builds a message from its dialect name and typed JSON fields (see
    /// [`message_from_fields`]) and sends it.
    pub async fn send_message_fields(
        &self,
        name: &str,
        fields: &Map<String, Value>,
        source_system: Option<u8>,
        source_component: Option<u8>,
    ) -> Result<u64, LinkError> {
        let message = message_from_fields(name, fields)?;
        self.send_message(message, source_system, source_component)
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

impl LinkError {
    /// Short classification for diagnostics, e.g. `io:TimedOut`.
    #[must_use]
    pub fn kind_name(&self) -> String {
        match self {
            Self::Unavailable => "unavailable".to_owned(),
            Self::InvalidFrame(_) => "invalid_frame".to_owned(),
            Self::Receive(_) => "receive".to_owned(),
            Self::Io(error) => format!("io:{:?}", error.kind()),
            Self::Codec(_) => "codec".to_owned(),
            Self::Journal(_) => "journal".to_owned(),
        }
    }
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
    let events = LinkEvents::new(journal.clone(), &config.id);
    let mut repeats = FailureRepeats::default();
    loop {
        if let Some(resolved) =
            resolve_transport(&config.transport, &status_tx, &events, &mut repeats).await
        {
            set_phase(&status_tx, LinkPhase::Opening);
            let target = resolved.target();
            let opened = match resolved {
                ResolvedTransport::Tcp(address) => match TcpStream::connect(address).await {
                    Ok(stream) => {
                        stream.set_nodelay(true).ok();
                        let (reader, writer) = stream.into_split();
                        Ok((
                            Box::new(reader) as Box<dyn AsyncRead + Unpin + Send>,
                            Box::new(writer) as Box<dyn AsyncWrite + Unpin + Send>,
                        ))
                    }
                    Err(error) => Err((error.to_string(), format!("{:?}", error.kind()))),
                },
                // Discovery hands over the already-open probe stream.
                ResolvedTransport::Serial(found) => {
                    let (reader, writer) = tokio::io::split(found.stream);
                    Ok((
                        Box::new(crate::discovery::SerialReader::new(reader))
                            as Box<dyn AsyncRead + Unpin + Send>,
                        Box::new(writer) as Box<dyn AsyncWrite + Unpin + Send>,
                    ))
                }
            };
            match opened {
                Ok((reader, writer)) => {
                    events
                        .emit(LinkEvent::Opened {
                            target: target.clone(),
                        })
                        .await;
                    let reached_ready = run_transport(
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
                        &events,
                        &mut repeats,
                        &target,
                        reader,
                        writer,
                    )
                    .await;
                    if reached_ready {
                        repeats.reset();
                    }
                }
                Err((error, error_kind)) => {
                    record_failure(&status_tx, "open", format!("{target}: {error}"));
                    if let Some(repeat_count) =
                        repeats.observe("session", format!("open:{target}:{error}"))
                    {
                        events
                            .emit(LinkEvent::OpenFailed {
                                target,
                                error,
                                error_kind,
                                repeat_count,
                            })
                            .await;
                    }
                }
            }
        }
        set_phase(&status_tx, LinkPhase::Backoff);
        time::sleep(config.reconnect_interval).await;
    }
}

fn millis(duration: Duration) -> u64 {
    u64::try_from(duration.as_millis()).unwrap_or(u64::MAX)
}

/// A transport the link can actually connect to, after discovery.
enum ResolvedTransport {
    Tcp(SocketAddr),
    Serial(crate::discovery::DiscoveredSerial),
}

impl ResolvedTransport {
    fn target(&self) -> String {
        match self {
            Self::Tcp(address) => format!("tcp:{address}"),
            Self::Serial(found) => format!("serial:{}:{}", found.port, found.baud),
        }
    }
}

fn set_phase(status_tx: &watch::Sender<LinkStatus>, phase: LinkPhase) {
    status_tx.send_if_modified(|status| {
        let changed = status.phase != phase;
        status.phase = phase;
        changed
    });
}

/// Resolve the configured transport for one connection attempt.
///
/// For serial transports this rescans the serial ports on every attempt
/// (restricted by `DiscoveryConfig::port_filter` / `bauds` if set), so a port
/// that disappears (unplugged cable, renumbered COM port, autopilot not yet
/// powered on) is replaced by whichever port is currently emitting MAVLink
/// heartbeats. Finding nothing is an ordinary, frequently-occurring outcome,
/// not an error condition: the caller simply waits and rescans.
async fn resolve_transport(
    transport: &MavlinkTransport,
    status_tx: &watch::Sender<LinkStatus>,
    events: &LinkEvents,
    repeats: &mut FailureRepeats,
) -> Option<ResolvedTransport> {
    match transport {
        MavlinkTransport::Tcp(address) => Some(ResolvedTransport::Tcp(*address)),
        MavlinkTransport::Serial(discovery) => {
            status_tx.send_modify(|status| {
                status.connected = false;
                status.ready = false;
                status.phase = LinkPhase::Scanning;
                status.connection = "auto:scanning".to_owned();
                status.port = None;
                status.baud = None;
            });
            tracing::debug!("scanning serial ports for MAVLink");
            match crate::discovery::discover_serial_transport(discovery).await {
                Ok(found) => {
                    status_tx.send_modify(|status| {
                        status.connection = format!("serial:{}:{}", found.port, found.baud);
                        status.port = Some(found.port.clone());
                        status.baud = Some(found.baud);
                        status.last_scan = Some(found.report.clone());
                    });
                    // A rediscovery after repeated failures is always worth
                    // recording; a steady discover/open loop is not.
                    if repeats
                        .observe("scan", format!("discovered:{}:{}", found.port, found.baud))
                        .is_some()
                    {
                        events
                            .emit(LinkEvent::Discovered {
                                port: found.port.clone(),
                                baud: found.baud,
                                scan: found.report.clone(),
                            })
                            .await;
                    }
                    Some(ResolvedTransport::Serial(found))
                }
                Err(failure) => {
                    tracing::debug!(error = %failure, "serial discovery found no MAVLink device yet");
                    record_failure(status_tx, "scan", &failure.message);
                    status_tx.send_modify(|status| {
                        status.last_scan = Some(failure.report.clone());
                    });
                    if let Some(repeat_count) =
                        repeats.observe("scan", format!("scan:{}", failure.report.signature()))
                    {
                        events
                            .emit(LinkEvent::ScanFailed {
                                error: failure.message,
                                repeat_count,
                                scan: failure.report,
                            })
                            .await;
                    }
                    None
                }
            }
        }
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

/// Run one opened transport until it fails. Returns whether the session ever
/// received a vehicle heartbeat.
async fn run_transport<R, W>(
    context: ConnectionContext<'_>,
    events: &LinkEvents,
    repeats: &mut FailureRepeats,
    target: &str,
    reader: R,
    writer: W,
) -> bool
where
    R: AsyncRead + Unpin + Send + 'static,
    W: AsyncWrite + Unpin + Send,
{
    let clock_epoch = match context.journal.begin_clock_epoch().await {
        Ok(epoch) => epoch,
        Err(error) => {
            record_failure(context.status_tx, "open", &error);
            context
                .status_tx
                .send_modify(|status| status.connected = false);
            tracing::error!(link = %context.config.id, %error, "could not start MAVLink clock epoch");
            return false;
        }
    };
    let opened_ns = wall_time_ns();
    let opened_at = time::Instant::now();
    let received_before = context.status_tx.borrow().received_messages;
    context.status_tx.send_modify(|status| {
        status.connected = true;
        status.phase = LinkPhase::Acquiring;
        status.clock_epoch = clock_epoch;
        status.latest_time_boot_ms = 0;
        status.attempts += 1;
        status.connected_since_ns = Some(opened_ns);
        status.rx_bps = None;
        status.tx_bps = None;
        status.rx_bps_by_message.clear();
        status.tx_bps_by_message.clear();
        status.error = None;
    });
    let status_tx = context.status_tx.clone();
    let journal = context.journal.clone();
    let mut ready_rx = status_tx.subscribe();
    let session = run_connected(context, reader, writer);
    tokio::pin!(session);
    let mut reached_ready = false;
    let result = loop {
        tokio::select! {
            result = &mut session => break result,
            changed = ready_rx.changed(), if !reached_ready => {
                if changed.is_err() || !ready_rx.borrow_and_update().ready {
                    continue;
                }
                reached_ready = true;
                status_tx.send_modify(|status| {
                    status.phase = LinkPhase::Ready;
                    status.consecutive_failures = 0;
                });
                let (target_system, target_component) = {
                    let status = status_tx.borrow();
                    (status.target_system, status.target_component)
                };
                events
                    .emit(LinkEvent::Ready {
                        target: target.to_owned(),
                        ms_to_first_heartbeat: millis(opened_at.elapsed()),
                        target_system,
                        target_component,
                    })
                    .await;
            }
        }
    };
    let Err(error) = result else {
        return reached_ready;
    };
    // The link was live and just broke, forcing a reconnect: bump the
    // generation immediately (rather than waiting for reacquisition,
    // which can take much longer than one request) so a caller mid
    // multi-step sequence is told to abort right away instead of
    // lingering on a link that is no longer trustworthy.
    let clock_epoch = journal.begin_clock_epoch().await.unwrap_or(clock_epoch);
    let stage = if reached_ready { "link" } else { "acquire" };
    record_failure(&status_tx, stage, &error);
    let (received_messages, last_received_ns) = {
        let status = status_tx.borrow();
        (status.received_messages, status.last_received_ns)
    };
    status_tx.send_modify(|status| {
        status.connected = false;
        status.clock_epoch = clock_epoch;
        status.connected_since_ns = None;
        status.rx_bps = None;
        status.tx_bps = None;
        status.rx_bps_by_message.clear();
        status.tx_bps_by_message.clear();
    });
    let target = target.to_owned();
    let error_kind = error.kind_name();
    let error = error.to_string();
    let connected_ms = millis(opened_at.elapsed());
    let received_messages = received_messages.saturating_sub(received_before);
    if reached_ready {
        events
            .emit(LinkEvent::Lost {
                target,
                error,
                error_kind,
                connected_ms,
                received_messages,
                ms_since_last_received: last_received_ns
                    .map(|ns| wall_time_ns().saturating_sub(ns) / 1_000_000),
            })
            .await;
    } else if let Some(repeat_count) =
        repeats.observe("session", format!("acquire:{target}:{error}"))
    {
        events
            .emit(LinkEvent::AcquireFailed {
                target,
                error,
                error_kind,
                connected_ms,
                received_messages,
                repeat_count,
            })
            .await;
    }
    reached_ready
}

/// Record a failed scan/open/session in the status. `stage` is one of
/// `scan`, `open`, `acquire`, or `link`.
fn record_failure(
    status_tx: &watch::Sender<LinkStatus>,
    stage: &str,
    error: impl std::fmt::Display,
) {
    status_tx.send_modify(|status| {
        status.connected = false;
        status.ready = false;
        status.port = None;
        status.baud = None;
        status.error = Some(error.to_string());
        status.last_error_stage = Some(stage.to_owned());
        status.last_error_ns = Some(wall_time_ns());
        status.consecutive_failures += 1;
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
    let receive_task = tokio::spawn(receive_messages(reader, receive_tx, status_tx.clone()));
    let mut heartbeat = time::interval(config.heartbeat_interval);
    heartbeat.set_missed_tick_behavior(time::MissedTickBehavior::Delay);
    let now = time::Instant::now();
    let (mut throughput, mut rx_by_message, mut tx_by_message) = {
        let status = status_tx.borrow();
        (
            LinkThroughput::new(now, status.received_bytes, status.transmitted_bytes),
            MessageThroughput::new(now, &status.received_bytes_by_message),
            MessageThroughput::new(now, &status.transmitted_bytes_by_message),
        )
    };
    let mut throughput_tick =
        time::interval_at(now + Duration::from_secs(1), Duration::from_secs(1));
    throughput_tick.set_missed_tick_behavior(time::MissedTickBehavior::Skip);

    let result = loop {
        tokio::select! {
            _ = throughput_tick.tick() => {
                let sampled_at = time::Instant::now();
                let (rx, tx, rx_messages, tx_messages) = {
                    let status = status_tx.borrow();
                    let (rx, tx) = throughput.sample(
                        sampled_at,
                        status.received_bytes,
                        status.transmitted_bytes,
                    );
                    (
                        rx,
                        tx,
                        rx_by_message.sample(sampled_at, &status.received_bytes_by_message),
                        tx_by_message.sample(sampled_at, &status.transmitted_bytes_by_message),
                    )
                };
                status_tx.send_modify(|status| {
                    status.rx_bps = Some(rx);
                    status.tx_bps = Some(tx);
                    status.rx_bps_by_message = rx_messages;
                    status.tx_bps_by_message = tx_messages;
                });
            }
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
                let failure = result.as_ref().err().map(ToString::to_string);
                let _ = command.reply.send(result);
                if let Some(failure) = failure {
                    break Err(LinkError::Receive(format!("MAVLink send failed: {failure}")));
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

async fn receive_messages<R>(
    reader: R,
    sender: mpsc::Sender<Result<DecodedMessage, String>>,
    status_tx: watch::Sender<LinkStatus>,
) where
    R: AsyncRead + Unpin,
{
    let mut receiver = FrameReader::new(reader);
    loop {
        let result = receiver.recv().await.map_err(|error| error.to_string());
        let dropped = receiver.take_dropped_frames();
        if dropped > 0 {
            status_tx.send_modify(|status| status.dropped_frames += dropped);
        }
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
    let frame_bytes = decoded.raw.len() as u64;
    let message_name = decoded.name.clone();
    let system_id = decoded.system_id;
    let component_id = decoded.component_id;
    let time_boot_ms = decoded.fields.get("time_boot_ms").and_then(Value::as_u64);
    let heartbeat = match &decoded.message {
        MavMessage::HEARTBEAT(heartbeat) => Some(heartbeat.clone()),
        _ => None,
    };
    if let Some(heartbeat) = &heartbeat {
        update_component_registry(components, system_id, component_id, heartbeat, now);
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
        message: decoded.message,
    }));
    status_tx.send_modify(|status| {
        status.received_messages += 1;
        status.received_bytes += frame_bytes;
        *status
            .received_bytes_by_message
            .entry(message_name)
            .or_default() += frame_bytes;
        status.last_received_ns = Some(now);
        // Only an autopilot's heartbeat describes the vehicle. Other components
        // (a telemetry radio such as DroneBridge, a gimbal, a companion computer)
        // heartbeat with autopilot INVALID and must not retarget the link or
        // overwrite its mode and armed state.
        if system_id != config.source_system
            && let Some(heartbeat) = &heartbeat
            && heartbeat.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID
        {
            status.target_system = system_id;
            status.target_component = component_id;
            status.base_mode = heartbeat.base_mode;
            status.custom_mode = heartbeat.custom_mode;
            status.system_status = heartbeat.system_status;
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
    heartbeat: &HEARTBEAT_DATA,
    timestamp_ns: u64,
) {
    components.insert(
        (system_id, component_id),
        ComponentInfo {
            system_id,
            component_id,
            vehicle_type: heartbeat.mavtype,
            autopilot: heartbeat.autopilot,
            base_mode: heartbeat.base_mode,
            custom_mode: heartbeat.custom_mode,
            system_status: heartbeat.system_status,
            last_heartbeat_ns: timestamp_ns,
        },
    );
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
    let bytes = serialize_message(&message, header);
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
    status_tx.send_modify(|status| {
        status.transmitted_bytes += bytes.len() as u64;
        *status
            .transmitted_bytes_by_message
            .entry(decoded.name.clone())
            .or_default() += bytes.len() as u64;
    });
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

pub(crate) fn heartbeat_message() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    })
}

/// A heartbeat as the vehicle's autopilot sends it, for tests that play the peer.
#[cfg(test)]
pub(crate) fn autopilot_heartbeat_message() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_HELICOPTER,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_STANDBY,
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

    #[test]
    fn throughput_uses_bits_and_actual_elapsed_time() {
        let start = time::Instant::now();
        let mut throughput = LinkThroughput::new(start, 100, 200);
        assert_eq!(
            throughput.sample(start + Duration::from_secs(1), 1100, 700),
            (8000.0, 4000.0)
        );
        assert_eq!(
            throughput.sample(start + Duration::from_secs(3), 3100, 1700),
            (8000.0, 4000.0)
        );
        assert_eq!(
            throughput.sample(start + Duration::from_secs(6), 6100, 3200),
            (8000.0, 4000.0)
        );
    }

    #[test]
    fn throughput_rolls_off_traffic_while_idle() {
        let start = time::Instant::now();
        let mut throughput = LinkThroughput::new(start, 0, 0);
        assert_eq!(
            throughput.sample(start + Duration::from_secs(1), 300, 600),
            (2400.0, 4800.0)
        );
        assert_eq!(
            throughput.sample(start + Duration::from_secs(2), 300, 600),
            (1200.0, 2400.0)
        );
        assert_eq!(
            throughput.sample(start + Duration::from_secs(3), 300, 600),
            (800.0, 1600.0)
        );
        assert_eq!(
            throughput.sample(start + Duration::from_secs(4), 300, 600),
            (0.0, 0.0)
        );
    }

    #[test]
    fn throughput_new_connection_excludes_previous_traffic() {
        let start = time::Instant::now();
        let mut throughput = LinkThroughput::new(start, 1_000_000, 2_000_000);
        assert_eq!(
            throughput.sample(start + Duration::from_secs(1), 1_000_100, 2_000_050),
            (800.0, 400.0)
        );
        let status = LinkStatus::default();
        assert!(status.rx_bps.is_none());
        assert!(status.tx_bps.is_none());
    }

    #[test]
    fn throughput_interpolates_window_boundary_after_scheduler_delay() {
        let start = time::Instant::now();
        let mut throughput = LinkThroughput::new(start, 0, 0);
        throughput.sample(start + Duration::from_secs(2), 200, 400);
        // The window starts halfway through the first bucket; only half counts.
        assert_eq!(
            throughput.sample(start + Duration::from_secs(4), 200, 400),
            (800.0 / 3.0, 1600.0 / 3.0)
        );
    }

    #[test]
    fn message_throughput_splits_rates_by_name_and_rolls_off_idle_types() {
        let start = time::Instant::now();
        let counts = |attitude, rpm| {
            ByteCounts::from([("ATTITUDE".to_owned(), attitude), ("RPM".to_owned(), rpm)])
        };
        let mut throughput = MessageThroughput::new(start, &counts(1000, 0));
        let rates = throughput.sample(start + Duration::from_secs(1), &counts(1300, 100));
        assert_eq!(rates["ATTITUDE"], 2400.0);
        assert_eq!(rates["RPM"], 800.0);
        let rates = throughput.sample(start + Duration::from_secs(4), &counts(1300, 100));
        assert!(rates.is_empty());
    }

    #[tokio::test]
    async fn link_publishes_throughput_and_clears_it_on_disconnect() {
        let server = TcpListener::bind("127.0.0.1:0").await.expect("listener");
        let address = server.local_addr().expect("address");
        let (disconnect, disconnected) = oneshot::channel();
        let peer = tokio::spawn(async move {
            let (mut socket, _) = server.accept().await.expect("accept");
            socket
                .write_all(&autopilot_heartbeat(1, 1, 1))
                .await
                .expect("heartbeat");
            let mut buffer = [0; 21];
            socket.read_exact(&mut buffer).await.expect("GCS heartbeat");
            disconnected.await.expect("disconnect signal");
        });
        let temp = TempDir::new().expect("temp directory");
        let (journal, journal_task) =
            JournalHandle::start(JournalConfig::for_directory(temp.path(), Uuid::new_v4()))
                .await
                .expect("journal");
        let (link, link_task) = start_link(MavlinkLinkConfig::sitl(address), journal.clone());
        let mut status = link.subscribe_status();
        time::timeout(Duration::from_secs(3), async {
            loop {
                let snapshot = status.borrow_and_update().clone();
                if let (Some(rx), Some(tx)) = (snapshot.rx_bps, snapshot.tx_bps) {
                    assert!(snapshot.connected);
                    assert_eq!(snapshot.received_bytes, 21);
                    assert!(snapshot.transmitted_bytes >= 21);
                    assert!(rx > 0.0 && tx > 0.0);
                    assert_eq!(snapshot.received_bytes_by_message["HEARTBEAT"], 21);
                    assert!(snapshot.transmitted_bytes_by_message["HEARTBEAT"] >= 21);
                    assert!(snapshot.rx_bps_by_message["HEARTBEAT"] > 0.0);
                    assert!(snapshot.tx_bps_by_message["HEARTBEAT"] > 0.0);
                    break;
                }
                status.changed().await.expect("status");
            }
        })
        .await
        .expect("published rates");
        disconnect.send(()).expect("disconnect");
        peer.await.expect("peer");
        time::timeout(Duration::from_secs(2), async {
            loop {
                let snapshot = status.borrow_and_update().clone();
                if !snapshot.connected {
                    assert!(snapshot.rx_bps.is_none() && snapshot.tx_bps.is_none());
                    assert!(snapshot.rx_bps_by_message.is_empty());
                    assert!(snapshot.tx_bps_by_message.is_empty());
                    assert_eq!(snapshot.received_bytes, 21);
                    break;
                }
                status.changed().await.expect("status");
            }
        })
        .await
        .expect("cleared rates");
        link_task.abort();
        journal.shutdown().await.expect("shutdown");
        journal_task.await.expect("journal");
    }

    #[tokio::test]
    async fn link_counts_undecodable_frames_without_dropping_the_connection() {
        let server = TcpListener::bind("127.0.0.1:0").await.expect("listener");
        let address = server.local_addr().expect("address");
        let (disconnect, disconnected) = oneshot::channel();
        let peer = tokio::spawn(async move {
            let (mut socket, _) = server.accept().await.expect("accept");
            // A HEARTBEAT whose MAV_TYPE byte is outside the dialect, with a valid CRC.
            let mut invalid = autopilot_heartbeat(1, 1, 1);
            invalid[14] = 250;
            let crc = linkhub_dialect::calculate_crc(&invalid[1..19], 50);
            invalid[19..21].copy_from_slice(&crc.to_le_bytes());
            socket.write_all(&invalid).await.expect("invalid frame");
            socket
                .write_all(&heartbeat(2, 1, 1))
                .await
                .expect("heartbeat");
            disconnected.await.expect("disconnect signal");
        });
        let temp = TempDir::new().expect("temp directory");
        let (journal, journal_task) =
            JournalHandle::start(JournalConfig::for_directory(temp.path(), Uuid::new_v4()))
                .await
                .expect("journal");
        let (link, link_task) = start_link(MavlinkLinkConfig::sitl(address), journal.clone());
        let mut status = link.subscribe_status();
        time::timeout(Duration::from_secs(3), async {
            loop {
                let snapshot = status.borrow_and_update().clone();
                if snapshot.dropped_frames == 1 && snapshot.received_bytes == 21 {
                    assert!(snapshot.connected);
                    break;
                }
                status.changed().await.expect("status");
            }
        })
        .await
        .expect("dropped frame counted");
        disconnect.send(()).expect("disconnect");
        peer.await.expect("peer");
        link_task.abort();
        journal.shutdown().await.expect("shutdown");
        journal_task.await.expect("journal");
    }

    #[tokio::test]
    async fn non_autopilot_heartbeats_do_not_retarget_the_link() {
        let server = TcpListener::bind("127.0.0.1:0").await.expect("listener");
        let address = server.local_addr().expect("address");
        let (disconnect, disconnected) = oneshot::channel();
        let peer = tokio::spawn(async move {
            let (mut socket, _) = server.accept().await.expect("accept");
            // The autopilot, then a telemetry radio (component 68) that injects its own
            // heartbeat with autopilot INVALID, as DroneBridge does.
            socket
                .write_all(&autopilot_heartbeat(1, 1, 1))
                .await
                .expect("autopilot heartbeat");
            socket
                .write_all(&heartbeat(2, 1, 68))
                .await
                .expect("radio heartbeat");
            disconnected.await.expect("disconnect signal");
        });
        let temp = TempDir::new().expect("temp directory");
        let (journal, journal_task) =
            JournalHandle::start(JournalConfig::for_directory(temp.path(), Uuid::new_v4()))
                .await
                .expect("journal");
        let (link, link_task) = start_link(MavlinkLinkConfig::sitl(address), journal.clone());
        let mut status_rx = link.subscribe_status();
        time::timeout(Duration::from_secs(3), async {
            while link.components().len() < 2 {
                status_rx.changed().await.expect("status");
            }
        })
        .await
        .expect("both components seen");
        let status = link.status();
        assert_eq!(
            (status.target_system, status.target_component),
            (1, 1),
            "the radio heartbeat must not retarget the link"
        );
        assert_eq!(
            status.system_status,
            MavState::MAV_STATE_STANDBY,
            "the radio heartbeat must not overwrite the vehicle state"
        );
        disconnect.send(()).expect("disconnect");
        peer.await.expect("peer");
        link_task.abort();
        journal.shutdown().await.expect("shutdown");
        journal_task.await.expect("journal");
    }

    fn heartbeat(sequence: u8, system_id: u8, component_id: u8) -> Vec<u8> {
        serialize_message(
            &heartbeat_message(),
            MavHeader {
                sequence,
                system_id,
                component_id,
            },
        )
    }

    fn autopilot_heartbeat(sequence: u8, system_id: u8, component_id: u8) -> Vec<u8> {
        serialize_message(
            &autopilot_heartbeat_message(),
            MavHeader {
                sequence,
                system_id,
                component_id,
            },
        )
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
        let heartbeat = HEARTBEAT_DATA {
            custom_mode: 4,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED
                | MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        };

        update_component_registry(&mut components, 1, 1, &heartbeat, 100);
        update_component_registry(&mut components, 1, 1, &heartbeat, 200);

        let value = serde_json::to_value(components.get(&(1, 1)).expect("component"))
            .expect("serialize component");
        assert_eq!(
            value,
            json!({
                "system_id": 1,
                "component_id": 1,
                "vehicle_type": {"type": "MAV_TYPE_QUADROTOR"},
                "autopilot": {"type": "MAV_AUTOPILOT_ARDUPILOTMEGA"},
                "base_mode": "MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
                "custom_mode": 4,
                "system_status": {"type": "MAV_STATE_ACTIVE"},
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
                .write_all(&autopilot_heartbeat(1, 1, 1))
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
        // Capture status while still connected: once `peer` below returns, it
        // drops the socket, and the link's disconnect-triggered epoch bump
        // (see `run_transport`) would otherwise race with these assertions.
        let connected_status = link.status();
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
        assert!(
            records
                .iter()
                .filter(|record| matches!(record.payload, RecordPayload::MavlinkFrame(_)))
                .all(|record| record.sim_clock.epoch == 1)
        );
        assert_eq!(connected_status.clock_epoch, 1);
        assert_eq!(connected_status.received_bytes, 21);
        time::timeout(Duration::from_secs(2), async {
            while link.status().transmitted_bytes < 21 {
                time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("TX byte counter");
        assert_eq!(connected_status.target_system, 1);
        assert_eq!(connected_status.base_mode, MavModeFlag::empty());
        assert_eq!(connected_status.custom_mode, 0);
        assert_eq!(connected_status.system_status, MavState::MAV_STATE_STANDBY);

        link_task.abort();
        journal.shutdown().await.expect("shutdown");
        journal_task.await.expect("journal task");
    }

    #[tokio::test]
    async fn link_journals_lifecycle_diagnostics() {
        let server = TcpListener::bind("127.0.0.1:0")
            .await
            .expect("test listener");
        let address = server.local_addr().expect("listener address");
        let peer = tokio::spawn(async move {
            let (mut socket, _) = server.accept().await.expect("accept link");
            socket
                .write_all(&autopilot_heartbeat(1, 1, 1))
                .await
                .expect("send vehicle heartbeat");
            let mut received = [0_u8; 21];
            socket
                .read_exact(&mut received)
                .await
                .expect("read GCS heartbeat");
        });

        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let journal_config = JournalConfig::for_directory(temp.path(), run_id);
        let (journal, journal_task) = JournalHandle::start(journal_config).await.expect("journal");
        let mut link_config = MavlinkLinkConfig::sitl(address);
        link_config.heartbeat_interval = Duration::from_millis(10);
        let (link, link_task) = start_link(link_config, journal.clone());
        peer.await.expect("peer task");

        let events = time::timeout(Duration::from_secs(3), async {
            loop {
                let events: Vec<_> = journal
                    .records_after(0)
                    .await
                    .expect("journal records")
                    .into_iter()
                    .filter_map(|record| match record.payload {
                        RecordPayload::Diagnostic(event) => Some(*event),
                        RecordPayload::MavlinkFrame(_) => None,
                    })
                    .collect();
                if events.iter().any(|event| event.event == "link.lost") {
                    return events;
                }
                time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("link.lost diagnostic");

        let names: Vec<_> = events.iter().map(|event| event.event.as_str()).collect();
        assert_eq!(&names[..3], ["link.opened", "link.ready", "link.lost"]);
        assert!(events.iter().all(|event| event.source == "linkhub.link"));
        let ready = &events[1];
        assert_eq!(ready.fields["target_system"], json!(1));
        let lost = &events[2];
        assert!(lost.fields.contains_key("error_kind"));
        assert!(lost.fields.contains_key("connected_ms"));

        let status = link.status();
        assert_eq!(status.attempts, 1);
        assert!(status.consecutive_failures >= 1);
        assert!(status.last_error_ns.is_some());
        assert!(status.connected_since_ns.is_none());

        link_task.abort();
        journal.shutdown().await.expect("shutdown");
        journal_task.await.expect("journal task");
    }
}
