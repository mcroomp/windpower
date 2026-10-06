//! Serial-port discovery for MAVLink links.
//!
//! Discovery is the normal way LinkHub acquires a serial MAVLink link: it scans
//! the local serial ports and keeps the first candidate that emits a MAVLink
//! heartbeat, so operators do not have to know which COM port or baud rate the
//! autopilot enumerated on. A specific port and/or baud rate only *restrict*
//! which candidates are probed; the link still confirms a live heartbeat
//! before it is used, and still rescans from scratch on every reconnect so an
//! unplugged or renumbered port is rediscovered automatically. Being
//! unconnected while no candidate answers is an ordinary, expected state, not
//! an error.

use std::{
    future::Future,
    io,
    pin::Pin,
    task::{Context, Poll, ready},
    time::Duration,
};

use serde::Serialize;
use tokio::{
    io::{AsyncRead, ReadBuf},
    time,
};
use tokio_serial::{
    ClearBuffer, SerialPort, SerialPortBuilderExt, SerialPortInfo, SerialPortType, SerialStream,
};

use crate::{codec::FrameReader, mavlink::HEARTBEAT_MESSAGE_ID, records::wall_time_ns};

/// Baud rates probed when the caller does not restrict them, ordered by how
/// commonly the RAWES hardware uses them.
pub const DEFAULT_DISCOVERY_BAUDS: &[u32] = &[115_200, 57_600, 38_400, 19_200, 9_600];

/// Time a single port/baud candidate is given to deliver a heartbeat.
pub const DEFAULT_PROBE_TIMEOUT: Duration = Duration::from_secs(3);

/// How a serial link discovers its port.
///
/// `port_filter` and `bauds` only narrow the search; they never bypass the
/// heartbeat probe. Leave `port_filter` unset and `bauds` at the full default
/// list to scan every port.
#[derive(Clone, Debug)]
pub struct DiscoveryConfig {
    pub port_filter: Option<String>,
    pub bauds: Vec<u32>,
    pub probe_timeout: Duration,
}

impl Default for DiscoveryConfig {
    fn default() -> Self {
        Self {
            port_filter: None,
            bauds: DEFAULT_DISCOVERY_BAUDS.to_vec(),
            probe_timeout: DEFAULT_PROBE_TIMEOUT,
        }
    }
}

/// A discovered, heartbeat-confirmed serial candidate, still open.
#[derive(Debug)]
pub struct DiscoveredSerial {
    pub port: String,
    pub baud: u32,
    /// The probe's open stream, handed to the live link (never reopened).
    pub stream: SerialStream,
    pub report: ScanReport,
}

/// A scan that found no usable MAVLink candidate.
#[derive(Clone, Debug)]
pub struct ScanFailure {
    pub message: String,
    pub report: ScanReport,
}

impl std::fmt::Display for ScanFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.message)
    }
}

/// Everything one discovery scan saw, for status and journal diagnostics.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize)]
pub struct ScanReport {
    pub started_ns: u64,
    pub elapsed_ms: u64,
    /// Ports that were probed, in probe order.
    pub ports: Vec<PortReport>,
    /// Enumerated ports excluded by type or by the port filter.
    pub skipped_ports: Vec<PortReport>,
    pub probes: Vec<ProbeReport>,
}

impl ScanReport {
    /// Stable summary of the failure shape, used to suppress identical
    /// repeated scan failures in the journal.
    #[must_use]
    pub fn signature(&self) -> String {
        self.probes
            .iter()
            .map(|probe| {
                format!(
                    "{}@{}:{:?}:{}",
                    probe.port,
                    probe.baud,
                    probe.stage,
                    probe.error.as_deref().unwrap_or("ok")
                )
            })
            .chain(
                self.skipped_ports
                    .iter()
                    .map(|port| format!("skip:{}", port.port)),
            )
            .collect::<Vec<_>>()
            .join("|")
    }
}

/// One enumerated serial port and its USB identity, if any.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct PortReport {
    pub port: String,
    pub kind: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vid_pid: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub serial_number: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub manufacturer: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub product: Option<String>,
}

impl PortReport {
    fn from_info(info: &SerialPortInfo) -> Self {
        let mut report = Self {
            port: info.port_name.clone(),
            kind: match info.port_type {
                SerialPortType::UsbPort(_) => "usb",
                SerialPortType::PciPort => "pci",
                SerialPortType::BluetoothPort => "bluetooth",
                SerialPortType::Unknown => "unknown",
            },
            vid_pid: None,
            serial_number: None,
            manufacturer: None,
            product: None,
        };
        if let SerialPortType::UsbPort(usb) = &info.port_type {
            report.vid_pid = Some(format!("{:04X}:{:04X}", usb.vid, usb.pid));
            report.serial_number.clone_from(&usb.serial_number);
            report.manufacturer.clone_from(&usb.manufacturer);
            report.product.clone_from(&usb.product);
        }
        report
    }
}

/// How far a single port/baud probe got before it succeeded or failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ProbeStage {
    /// The OS refused to open the port (busy, access denied, vanished).
    Open,
    /// The port opened but reading or framing failed.
    Read,
    /// The port opened but no heartbeat arrived within the probe timeout.
    Timeout,
    /// A MAVLink heartbeat was received.
    Heartbeat,
}

/// Outcome of probing one port at one baud rate.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProbeReport {
    pub port: String,
    pub baud: u32,
    pub stage: ProbeStage,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_kind: Option<String>,
    /// Raw bytes received; distinguishes a silent port from a wrong baud.
    pub bytes_read: u64,
    /// Complete MAVLink frames parsed before the heartbeat or failure.
    pub frames: u64,
    pub elapsed_ms: u64,
}

/// Scan the local serial ports for a device that speaks MAVLink.
///
/// Returns the first candidate that delivers a heartbeat, or a failure
/// carrying the per-candidate report.
pub async fn discover_serial_transport(
    discovery: &DiscoveryConfig,
) -> Result<DiscoveredSerial, ScanFailure> {
    let started = time::Instant::now();
    let mut report = ScanReport {
        started_ns: wall_time_ns(),
        ..ScanReport::default()
    };
    let fail = |message: String, mut report: ScanReport| {
        report.elapsed_ms = elapsed_ms(started);
        Err(ScanFailure { message, report })
    };
    if discovery.bauds.is_empty() {
        return fail("no discovery baud rates were configured".to_owned(), report);
    }
    let ports = match available_ports(discovery.port_filter.as_deref()).await {
        Ok(ports) => ports,
        Err(message) => return fail(message, report),
    };
    report.skipped_ports = ports.skipped;
    report.ports = ports.candidates;
    if report.ports.is_empty() {
        let message = match &discovery.port_filter {
            Some(filter) => format!("serial port {filter:?} was not found"),
            None => "no serial ports were found".to_owned(),
        };
        return fail(message, report);
    }

    let candidates: Vec<String> = report.ports.iter().map(|port| port.port.clone()).collect();
    for port in &candidates {
        let initial_baud = discovery.bauds[0];
        let mut stream = match tokio_serial::new(port, initial_baud).open_native_async() {
            Ok(stream) => stream,
            Err(error) => {
                report.probes.push(ProbeReport {
                    port: port.clone(),
                    baud: initial_baud,
                    stage: ProbeStage::Open,
                    error: Some(error.to_string()),
                    error_kind: Some(serial_error_kind(&error)),
                    bytes_read: 0,
                    frames: 0,
                    elapsed_ms: 0,
                });
                continue;
            }
        };
        for baud in &discovery.bauds {
            tracing::debug!(port = %port, baud, "probing serial port for MAVLink");
            let probe = probe_candidate(&mut stream, port, *baud, discovery.probe_timeout).await;
            let heartbeat_found = probe.stage == ProbeStage::Heartbeat;
            if !heartbeat_found {
                tracing::debug!(
                    port = %port,
                    baud,
                    stage = ?probe.stage,
                    error = probe.error.as_deref().unwrap_or(""),
                    bytes_read = probe.bytes_read,
                    "serial probe failed"
                );
            }
            report.probes.push(probe);
            if heartbeat_found {
                tracing::info!(port = %port, baud, "MAVLink heartbeat found");
                report.elapsed_ms = elapsed_ms(started);
                return Ok(DiscoveredSerial {
                    port: port.clone(),
                    baud: *baud,
                    stream,
                    report,
                });
            }
        }
    }
    let failures = report
        .probes
        .iter()
        .map(|probe| {
            format!(
                "{}@{}: {}",
                probe.port,
                probe.baud,
                probe.error.as_deref().unwrap_or("unknown")
            )
        })
        .collect::<Vec<_>>()
        .join("; ");
    fail(
        format!("no MAVLink heartbeat was found on any serial port ({failures})"),
        report,
    )
}

fn elapsed_ms(started: time::Instant) -> u64 {
    u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX)
}

struct EnumeratedPorts {
    candidates: Vec<PortReport>,
    skipped: Vec<PortReport>,
}

/// List candidate serial ports, USB devices first, then alphabetically.
async fn available_ports(port_filter: Option<&str>) -> Result<EnumeratedPorts, String> {
    let ports = tokio::task::spawn_blocking(tokio_serial::available_ports)
        .await
        .map_err(|error| format!("serial port enumeration failed: {error}"))?
        .map_err(|error| format!("could not list serial ports: {error}"))?;
    let (mut candidates, skipped): (Vec<_>, Vec<_>) = ports.into_iter().partition(|port| {
        is_candidate(port)
            && port_filter.is_none_or(|filter| port.port_name.eq_ignore_ascii_case(filter))
    });
    candidates.sort_by(|left, right| {
        is_usb(right)
            .cmp(&is_usb(left))
            .then_with(|| left.port_name.cmp(&right.port_name))
    });
    Ok(EnumeratedPorts {
        candidates: candidates.iter().map(PortReport::from_info).collect(),
        skipped: skipped.iter().map(PortReport::from_info).collect(),
    })
}

fn is_candidate(port: &SerialPortInfo) -> bool {
    !matches!(port.port_type, SerialPortType::Unknown)
}

fn is_usb(port: &SerialPortInfo) -> bool {
    matches!(port.port_type, SerialPortType::UsbPort(_))
}

/// Human-readable classification of a serial open error, e.g.
/// `Io(PermissionDenied)` for a port another handle still holds.
#[must_use]
pub fn serial_error_kind(error: &tokio_serial::Error) -> String {
    format!("{:?}", error.kind)
}

/// Probe one baud on an already-open port. The caller keeps the handle open
/// across baud changes and transfers it directly to the live link on success.
async fn probe_candidate(
    stream: &mut SerialStream,
    port: &str,
    baud: u32,
    probe_timeout: Duration,
) -> ProbeReport {
    let started = time::Instant::now();
    let mut report = ProbeReport {
        port: port.to_owned(),
        baud,
        stage: ProbeStage::Open,
        error: None,
        error_kind: None,
        bytes_read: 0,
        frames: 0,
        elapsed_ms: 0,
    };
    if let Err(error) = stream.set_baud_rate(baud) {
        report.error_kind = Some(serial_error_kind(&error));
        report.error = Some(format!("could not set baud rate: {error}"));
        report.elapsed_ms = elapsed_ms(started);
        return report;
    }
    if let Err(error) = stream.clear(ClearBuffer::Input) {
        report.error_kind = Some(serial_error_kind(&error));
        report.error = Some(format!("could not clear serial input: {error}"));
        report.elapsed_ms = elapsed_ms(started);
        return report;
    }
    let mut reader = SerialReader::new(stream);
    let mut frames = 0;
    let outcome = time::timeout(probe_timeout, wait_for_heartbeat(&mut reader, &mut frames)).await;
    report.bytes_read = reader.bytes_read();
    report.frames = frames;
    match outcome {
        Ok(Ok(())) => {
            report.stage = ProbeStage::Heartbeat;
        }
        Ok(Err(error)) => {
            report.stage = ProbeStage::Read;
            report.error = Some(error);
        }
        Err(_) => {
            report.stage = ProbeStage::Timeout;
            report.error = Some("no heartbeat within the probe timeout".to_owned());
        }
    }
    report.elapsed_ms = elapsed_ms(started);
    report
}

/// Back-off before re-polling a serial port that returned no bytes.
const EMPTY_READ_BACKOFF: Duration = Duration::from_millis(5);

/// Silence after which an open serial port is treated as lost. ArduPilot
/// always emits a 1 Hz heartbeat, so this only trips on a dead port.
pub const SERIAL_SILENCE_TIMEOUT: Duration = Duration::from_secs(5);

/// Serial reader that treats an empty read as "no data yet", not EOF.
///
/// On Windows the async serial port completes a read with zero bytes when the
/// line is idle; the MAVLink reader would report that as "early eof". This
/// happens whenever the autopilot only sends its 1 Hz heartbeat (e.g. just
/// after a reboot, before streams are requested). Empty reads are retried
/// after a short back-off, and only prolonged silence is reported as an error.
pub struct SerialReader<R> {
    inner: R,
    backoff: Option<Pin<Box<time::Sleep>>>,
    last_data: time::Instant,
    bytes_read: u64,
}

impl<R> SerialReader<R> {
    pub fn new(inner: R) -> Self {
        Self {
            inner,
            backoff: None,
            last_data: time::Instant::now(),
            bytes_read: 0,
        }
    }

    #[must_use]
    pub const fn bytes_read(&self) -> u64 {
        self.bytes_read
    }
}

impl<R: AsyncRead + Unpin> AsyncRead for SerialReader<R> {
    fn poll_read(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        let this = self.get_mut();
        loop {
            if let Some(backoff) = this.backoff.as_mut() {
                ready!(backoff.as_mut().poll(cx));
                this.backoff = None;
            }
            let before = buf.filled().len();
            match ready!(Pin::new(&mut this.inner).poll_read(cx, buf)) {
                Ok(()) => {}
                Err(error) if error.kind() == io::ErrorKind::TimedOut => {
                    if this.last_data.elapsed() >= SERIAL_SILENCE_TIMEOUT {
                        return Poll::Ready(Err(io::Error::new(
                            io::ErrorKind::TimedOut,
                            format!("serial port silent for {SERIAL_SILENCE_TIMEOUT:?}"),
                        )));
                    }
                    this.backoff = Some(Box::pin(time::sleep(EMPTY_READ_BACKOFF)));
                    continue;
                }
                Err(error) => return Poll::Ready(Err(error)),
            }
            if buf.filled().len() > before || buf.remaining() == 0 {
                this.bytes_read += (buf.filled().len() - before) as u64;
                this.last_data = time::Instant::now();
                return Poll::Ready(Ok(()));
            }
            if this.last_data.elapsed() >= SERIAL_SILENCE_TIMEOUT {
                return Poll::Ready(Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!("serial port silent for {SERIAL_SILENCE_TIMEOUT:?}"),
                )));
            }
            this.backoff = Some(Box::pin(time::sleep(EMPTY_READ_BACKOFF)));
        }
    }
}

async fn wait_for_heartbeat<R>(reader: R, frames: &mut u64) -> Result<(), String>
where
    R: AsyncRead + Unpin,
{
    let mut receiver = FrameReader::new(reader);
    loop {
        let decoded = receiver.recv().await.map_err(|error| error.to_string())?;
        *frames += 1;
        if decoded.message_id == HEARTBEAT_MESSAGE_ID {
            return Ok(());
        }
    }
}

/// Parse a comma-separated baud list such as `115200,57600`.
pub fn parse_bauds(value: &str) -> Result<Vec<u32>, String> {
    let bauds = value
        .split(',')
        .map(str::trim)
        .filter(|entry| !entry.is_empty())
        .map(|entry| {
            entry
                .parse::<u32>()
                .map_err(|error| format!("invalid baud rate {entry:?}: {error}"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    if bauds.is_empty() {
        return Err("at least one baud rate is required".to_owned());
    }
    Ok(bauds)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_baud_lists_and_rejects_empty_input() {
        assert_eq!(
            parse_bauds(" 115200, 57600 ").expect("baud list"),
            vec![115_200, 57_600]
        );
        assert!(parse_bauds("").is_err());
        assert!(parse_bauds("fast").is_err());
    }

    #[tokio::test]
    async fn discovery_requires_baud_candidates() {
        let discovery = DiscoveryConfig {
            bauds: Vec::new(),
            ..DiscoveryConfig::default()
        };
        let error = discover_serial_transport(&discovery)
            .await
            .expect_err("empty baud list");
        assert!(error.message.contains("baud"));
    }

    #[tokio::test]
    async fn discovery_reports_missing_filtered_port() {
        let discovery = DiscoveryConfig {
            port_filter: Some("COM-does-not-exist".to_owned()),
            ..DiscoveryConfig::default()
        };
        let error = discover_serial_transport(&discovery)
            .await
            .expect_err("filtered port absent");
        assert!(error.message.contains("COM-does-not-exist"));
        assert!(error.report.ports.is_empty());
        assert!(error.report.probes.is_empty());
    }

    /// Yields `empty_reads` zero-byte reads (like an idle Windows serial
    /// port), then the payload, then zero-byte reads forever.
    struct IdleThenData {
        empty_reads: usize,
        payload: Option<Vec<u8>>,
    }

    impl AsyncRead for IdleThenData {
        fn poll_read(
            self: Pin<&mut Self>,
            _cx: &mut Context<'_>,
            buf: &mut ReadBuf<'_>,
        ) -> Poll<io::Result<()>> {
            let this = self.get_mut();
            if this.empty_reads > 0 {
                this.empty_reads -= 1;
            } else if let Some(payload) = this.payload.take() {
                buf.put_slice(&payload);
            }
            Poll::Ready(Ok(()))
        }
    }

    #[tokio::test(start_paused = true)]
    async fn serial_reader_retries_empty_reads() {
        use tokio::io::AsyncReadExt;
        let mut reader = SerialReader::new(IdleThenData {
            empty_reads: 3,
            payload: Some(vec![0xfd, 0x09]),
        });
        let mut bytes = [0u8; 2];
        reader
            .read_exact(&mut bytes)
            .await
            .expect("data after idle reads");
        assert_eq!(bytes, [0xfd, 0x09]);
        assert_eq!(reader.bytes_read(), 2);
    }

    struct TimeoutThenData {
        timeout_reads: usize,
        payload: Option<Vec<u8>>,
    }

    impl AsyncRead for TimeoutThenData {
        fn poll_read(
            self: Pin<&mut Self>,
            _cx: &mut Context<'_>,
            buf: &mut ReadBuf<'_>,
        ) -> Poll<io::Result<()>> {
            let this = self.get_mut();
            if this.timeout_reads > 0 {
                this.timeout_reads -= 1;
                return Poll::Ready(Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "transient Windows operation aborted",
                )));
            }
            if let Some(payload) = this.payload.take() {
                buf.put_slice(&payload);
            }
            Poll::Ready(Ok(()))
        }
    }

    #[tokio::test(start_paused = true)]
    async fn serial_reader_retries_transient_timeouts() {
        use tokio::io::AsyncReadExt;
        let mut reader = SerialReader::new(TimeoutThenData {
            timeout_reads: 2,
            payload: Some(vec![0xfd, 0x09]),
        });
        let mut bytes = [0u8; 2];
        reader
            .read_exact(&mut bytes)
            .await
            .expect("data after transient timeouts");
        assert_eq!(bytes, [0xfd, 0x09]);
        assert_eq!(reader.bytes_read(), 2);
    }

    #[test]
    fn scan_signature_ignores_timing_but_tracks_outcomes() {
        let probe = |error: &str, elapsed_ms| ProbeReport {
            port: "COM5".to_owned(),
            baud: 115_200,
            stage: ProbeStage::Open,
            error: Some(error.to_owned()),
            error_kind: None,
            bytes_read: 0,
            frames: 0,
            elapsed_ms,
        };
        let report = |error: &str, elapsed_ms| ScanReport {
            probes: vec![probe(error, elapsed_ms)],
            ..ScanReport::default()
        };
        assert_eq!(
            report("Access is denied.", 1).signature(),
            report("Access is denied.", 7).signature()
        );
        assert_ne!(
            report("Access is denied.", 1).signature(),
            report("not found", 1).signature()
        );
    }

    #[tokio::test(start_paused = true)]
    async fn serial_reader_reports_prolonged_silence() {
        use tokio::io::AsyncReadExt;
        let mut reader = SerialReader::new(IdleThenData {
            empty_reads: 0,
            payload: None,
        });
        let mut byte = [0u8; 1];
        let error = reader.read_exact(&mut byte).await.expect_err("silence");
        assert_eq!(error.kind(), io::ErrorKind::TimedOut);
    }
}
