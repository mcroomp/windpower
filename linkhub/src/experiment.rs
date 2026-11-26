//! Operator experiments: a named journal window saved into its own directory.
//!
//! Starting creates `<directory>/<UTC start>_<slug>_<id>/` holding
//! `experiment.txt` (the details entered at start) and journals an
//! `experiment.started` diagnostic marker. It then captures, in the
//! background, the vehicle's firmware version, the packed parameter table
//! (`param.pck`, decoded into `parameters.json`) and every Lua script under
//! `/APM/scripts` (into `scripts/`), summarised in `snapshot.json` and on the
//! active experiment's `snapshot` status. Stopping journals
//! `experiment.stopped` and streams every journal record between the two
//! markers (inclusive) into `mavlog.json` in that directory. The active
//! experiment lives in memory; a LinkHub restart ends it without writing
//! `mavlog.json`, but the run journal still holds the records.
//!
//! Phone cameras register independently of experiments with periodic
//! heartbeats; a phone seen within [`CAMERA_TIMEOUT`] is listed as connected.
//! While an experiment runs, each phone uploads its recording as ordered
//! chunks appended to `video-<stream>.<ext>` in the experiment directory.
//! Uploads stay accepted for [`VIDEO_GRACE`] after stop so phones can deliver
//! their final chunk. FFmpeg remuxes completed recordings without re-encoding
//! to write duration and seek indexes.

use std::{
    collections::BTreeMap,
    future::Future,
    net::IpAddr,
    path::{Component, Path, PathBuf},
    pin::Pin,
    sync::{
        Arc, Mutex, MutexGuard,
        atomic::{AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};

use mavlink_protocol::client::{
    PARAM_PACK_PATH, TransferObserver, TransferSummary, decode_param_pack,
};
use serde::Serialize;
use serde_json::{Map, Value, json};
use sha2::{Digest, Sha256};
use tokio::{
    fs,
    io::{AsyncWriteExt, BufWriter},
};
use uuid::Uuid;

use crate::{
    journal::{JournalError, JournalHandle},
    operations::MavlinkOperations,
    records::{DiagnosticEvent, DiagnosticLevel, SCHEMA_VERSION, format_cursor, wall_time_ns},
    transfer::TransferState,
};

const FILE_SCHEMA_VERSION: u16 = 1;
const MAX_NAME_CHARS: usize = 200;
const MAX_DESCRIPTION_CHARS: usize = 10_000;
const MAX_SLUG_CHARS: usize = 48;
const READ_BATCH_RECORDS: usize = 10_000;
const DETAILS_FILE: &str = "experiment.txt";
const MAVLOG_FILE: &str = "mavlog.json";
const SNAPSHOT_FILE: &str = "snapshot.json";
const PARAMETERS_FILE: &str = "parameters.json";
const PARAM_PACK_FILE: &str = "param.pck";
const SCRIPTS_DIRECTORY: &str = "scripts";
const VEHICLE_SCRIPTS: &str = "/APM/scripts";
const SNAPSHOT_REQUEST_TIMEOUT: Duration = Duration::from_secs(10);
const SNAPSHOT_STALL_TIMEOUT: Duration = Duration::from_secs(15);
const MAX_MARK_CHARS: usize = 200;
const ONBOARD_LOGS_FILE: &str = "onboard_logs.json";
const LOG_PACKET_TIMEOUT: Duration = Duration::from_secs(2);
const LOG_MAX_RETRIES: u32 = 10;
const LOG_POLL_INTERVAL: Duration = Duration::from_millis(500);
const MAX_PROFILE_CHARS: usize = 40;
/// Streams whose vehicle-side interval the snapshot records, so analysis knows
/// what was requested rather than inferring it from what arrived.
const SNAPSHOT_STREAMS: [&str; 8] = [
    "ATTITUDE_QUATERNION",
    "ATTITUDE_TARGET",
    "SERVO_OUTPUT_RAW",
    "PID_TUNING",
    "RPM",
    "ESC_TELEMETRY_1_TO_4",
    "VIBRATION",
    "BATTERY_STATUS",
];
const MAX_STREAM_ID_CHARS: usize = 32;
pub const VIDEO_GRACE: Duration = Duration::from_secs(120);
pub const CAMERA_TIMEOUT: Duration = Duration::from_secs(10);
const MAX_CAMERA_LABEL_CHARS: usize = 40;
const VIDEO_FINALIZE_TIMEOUT: Duration = Duration::from_secs(120);

/// Where experiments are saved and how phones reach the camera page.
#[derive(Clone, Debug)]
pub struct ExperimentConfig {
    pub directory: PathBuf,
    pub camera: Option<CameraConfig>,
}

#[derive(Clone, Copy, Debug)]
pub struct CameraConfig {
    pub listen: IpAddr,
    pub https_port: u16,
}

#[derive(Clone, Debug, Serialize)]
pub struct VideoStream {
    pub stream: String,
    pub file: String,
    pub bytes: u64,
    pub chunks: u64,
    pub updated_at_ns: u64,
    pub finished: bool,
}

/// A phone camera page that has recently sent a heartbeat.
#[derive(Clone, Debug, Serialize)]
pub struct CameraPhone {
    pub stream: String,
    pub label: String,
    /// The experiment the phone reports it is recording, if any.
    pub recording: Option<Uuid>,
    pub last_seen_ns: u64,
}

struct VideoState {
    info: VideoStream,
    path: PathBuf,
    final_sequence: Option<u64>,
}

struct VideoSession {
    experiment_id: Uuid,
    directory: PathBuf,
    stopped_at: Option<Instant>,
    streams: BTreeMap<String, Arc<tokio::sync::Mutex<VideoState>>>,
}

#[derive(Default)]
struct VideoSessions {
    current: Option<VideoSession>,
    previous: Option<VideoSession>,
}

impl VideoSessions {
    fn session(&mut self, experiment_id: Uuid) -> Option<&mut VideoSession> {
        if self
            .current
            .as_ref()
            .is_some_and(|session| session.experiment_id == experiment_id)
        {
            return self.current.as_mut();
        }
        self.previous.as_mut().filter(|session| {
            session.experiment_id == experiment_id
                && session
                    .stopped_at
                    .is_some_and(|stopped| stopped.elapsed() < VIDEO_GRACE)
        })
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct ActiveExperiment {
    pub id: Uuid,
    pub name: String,
    pub description: String,
    pub run_id: Uuid,
    pub started_at_ns: u64,
    pub start_cursor: String,
    pub directory: PathBuf,
    /// The client's telemetry profile when it started the experiment.
    pub telemetry_profile: Option<String>,
    /// Vehicle configuration captured when the experiment started.
    pub snapshot: SnapshotStatus,
    /// Onboard logs present at the start; logs new or grown since are saved at stop.
    #[serde(skip)]
    onboard_logs_at_start: Result<Vec<(u16, u32)>, String>,
    #[serde(skip)]
    start_sequence: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SnapshotState {
    Capturing,
    Complete,
    Failed,
}

#[derive(Clone, Debug, Serialize)]
pub struct ScriptFile {
    pub name: String,
    pub bytes: u64,
    pub sha256: String,
}

/// Progress of the start-of-experiment parameter, script and firmware capture.
#[derive(Clone, Debug, Serialize)]
pub struct SnapshotStatus {
    pub state: SnapshotState,
    pub error: Option<String>,
    pub parameter_count: Option<usize>,
    pub scripts: Vec<ScriptFile>,
    pub completed_at_ns: Option<u64>,
}

impl SnapshotStatus {
    fn capturing() -> Self {
        Self {
            state: SnapshotState::Capturing,
            error: None,
            parameter_count: None,
            scripts: Vec::new(),
            completed_at_ns: None,
        }
    }
}

type SnapshotFuture<'a, T> = Pin<Box<dyn Future<Output = Result<T, String>> + Send + 'a>>;

/// The vehicle reads experiments need; implemented over MAVLink by
/// [`MavlinkOperations`].
pub trait ExperimentVehicle: Send + Sync + 'static {
    fn read_firmware(&self) -> SnapshotFuture<'_, Value>;
    /// Regular file names in `directory`.
    fn list_scripts<'a>(&'a self, directory: &'a str) -> SnapshotFuture<'a, Vec<String>>;
    fn read_file<'a>(&'a self, path: &'a str, verify_crc: bool) -> SnapshotFuture<'a, Vec<u8>>;
    /// The vehicle's current interval for `message` on this link.
    fn read_message_interval<'a>(&'a self, message: &'a str) -> SnapshotFuture<'a, Value>;
    /// Onboard (DataFlash) logs as `(id, size)`.
    fn list_onboard_logs(&self) -> SnapshotFuture<'_, Vec<(u16, u32)>>;
    fn download_onboard_log<'a>(
        &'a self,
        id: u16,
        progress: OnboardLogProgress<'a>,
    ) -> SnapshotFuture<'a, Vec<u8>>;
}

pub struct OnboardLogProgress<'a> {
    journal: &'a JournalHandle,
    sequence: &'a AtomicU64,
    experiment_id: Uuid,
    id: u16,
    reported_step: u64,
}

impl OnboardLogProgress<'_> {
    async fn update(&mut self, received: u64, total: u64) {
        if total == 0 {
            return;
        }
        let received = received.min(total);
        let percent = (received * 100 / total).min(99);
        let step = percent / 10;
        if step > self.reported_step {
            self.reported_step = step;
            journal_onboard_logs(
                self.journal,
                self.sequence,
                self.experiment_id,
                json!({
                    "state": "progress",
                    "id": self.id,
                    "received_bytes": received,
                    "total_bytes": total,
                    "percent": percent,
                }),
            )
            .await;
        }
    }
}

/// Snapshot file reads run outside the transfer registry and are never cancelled.
struct UnobservedTransfer;

impl TransferObserver for UnobservedTransfer {
    fn is_cancelled(&self) -> bool {
        false
    }

    fn cancelled(&self) -> mavlink_protocol::transport::BoxFuture<'_, ()> {
        Box::pin(std::future::pending())
    }

    fn start(&self, _total_bytes: u64) -> mavlink_protocol::transport::BoxFuture<'_, ()> {
        Box::pin(async {})
    }

    fn progress(&self, _done: u64, _received: u64, _stats: mavlink_protocol::stats::TransferStats) {
    }
}

impl ExperimentVehicle for MavlinkOperations {
    fn read_firmware(&self) -> SnapshotFuture<'_, Value> {
        Box::pin(async move {
            self.autopilot_version(None, None, SNAPSHOT_REQUEST_TIMEOUT)
                .await
                .map_err(|error| format!("AUTOPILOT_VERSION: {error}"))
        })
    }

    fn list_scripts<'a>(&'a self, directory: &'a str) -> SnapshotFuture<'a, Vec<String>> {
        Box::pin(async move {
            let entries = (**self)
                .list_files(directory, SNAPSHOT_REQUEST_TIMEOUT)
                .await
                .map_err(|error| format!("list {directory}: {error}"))?;
            Ok(entries
                .iter()
                .filter(|entry| entry["is_dir"] != Value::Bool(true))
                .filter_map(|entry| entry["name"].as_str().map(str::to_owned))
                .collect())
        })
    }

    fn read_file<'a>(&'a self, path: &'a str, verify_crc: bool) -> SnapshotFuture<'a, Vec<u8>> {
        Box::pin(async move {
            let mut summary = TransferSummary::default();
            self.download_file(
                &UnobservedTransfer,
                path,
                verify_crc,
                SNAPSHOT_STALL_TIMEOUT,
                &mut summary,
            )
            .await
            .map_err(|error| format!("download {path}: {error}"))
        })
    }

    fn read_message_interval<'a>(&'a self, message: &'a str) -> SnapshotFuture<'a, Value> {
        Box::pin(async move {
            self.message_interval(message, SNAPSHOT_REQUEST_TIMEOUT)
                .await
                .map_err(|error| format!("{message} interval: {error}"))
        })
    }

    fn list_onboard_logs(&self) -> SnapshotFuture<'_, Vec<(u16, u32)>> {
        Box::pin(async move {
            let entries = self
                .list_logs(SNAPSHOT_REQUEST_TIMEOUT)
                .await
                .map_err(|error| format!("list onboard logs: {error}"))?;
            entries
                .iter()
                .map(|entry| {
                    let id = entry["id"].as_u64().and_then(|id| u16::try_from(id).ok());
                    let size = entry["size"]
                        .as_u64()
                        .and_then(|size| u32::try_from(size).ok());
                    id.zip(size)
                        .ok_or_else(|| format!("invalid LOG_ENTRY {entry}"))
                })
                .collect()
        })
    }

    /// Runs as an ordinary registry transfer, so it is listed and cancellable
    /// at `/v1/mavlink/transfers` like any other log download.
    fn download_onboard_log<'a>(
        &'a self,
        id: u16,
        mut progress: OnboardLogProgress<'a>,
    ) -> SnapshotFuture<'a, Vec<u8>> {
        Box::pin(async move {
            let transfer = self
                .start_log_download(id, LOG_PACKET_TIMEOUT, LOG_MAX_RETRIES)
                .id;
            loop {
                tokio::time::sleep(LOG_POLL_INTERVAL).await;
                let info = self
                    .transfers()
                    .get(transfer)
                    .ok_or_else(|| format!("log {id} transfer {transfer} was forgotten"))?;
                match info.state {
                    TransferState::Complete => {
                        return self
                            .transfers()
                            .content(transfer)
                            .map(|content| content.as_ref().clone())
                            .map_err(|error| format!("log {id}: {error}"));
                    }
                    TransferState::Failed => {
                        return Err(format!(
                            "log {id}: {}",
                            info.error.unwrap_or_else(|| "failed".to_owned())
                        ));
                    }
                    TransferState::Cancelled => return Err(format!("log {id}: cancelled")),
                    TransferState::Running => {
                        progress.update(info.received_bytes, info.total_bytes).await;
                    }
                    TransferState::Queued => {}
                }
            }
        })
    }
}

#[derive(Debug, Serialize)]
pub struct SavedExperiment {
    pub experiment: ActiveExperiment,
    pub stopped_at_ns: u64,
    pub end_cursor: String,
    pub record_count: u64,
    pub path: PathBuf,
}

#[derive(Debug, thiserror::Error)]
pub enum ExperimentError {
    #[error("experiment name is required")]
    MissingName,
    #[error("experiment name must not exceed {MAX_NAME_CHARS} characters")]
    NameTooLong,
    #[error("experiment description must not exceed {MAX_DESCRIPTION_CHARS} characters")]
    DescriptionTooLong,
    #[error("telemetry profile must not exceed {MAX_PROFILE_CHARS} characters")]
    ProfileTooLong,
    #[error("mark label is required")]
    MissingLabel,
    #[error("mark label must not exceed {MAX_MARK_CHARS} characters")]
    LabelTooLong,
    #[error("experiment {0:?} is already running")]
    AlreadyRunning(String),
    #[error("no experiment is running")]
    NotRunning,
    #[error(transparent)]
    Journal(#[from] JournalError),
    #[error("{0}")]
    InvalidVideo(String),
    #[error("{0}")]
    VideoOutOfOrder(String),
    #[error("experiment {0} is not accepting video")]
    VideoClosed(Uuid),
    #[error("could not finalize video {path}: {message}")]
    VideoFinalize { path: PathBuf, message: String },
    #[error("could not write experiment file {path}: {source}")]
    Write {
        path: PathBuf,
        source: std::io::Error,
    },
}

pub struct ExperimentStore {
    directory: PathBuf,
    journal: JournalHandle,
    snapshot_source: Option<Arc<dyn ExperimentVehicle>>,
    active: Arc<Mutex<Option<ActiveExperiment>>>,
    /// Serializes start/stop so a slow save cannot race a new start.
    operation: tokio::sync::Mutex<()>,
    marker_sequence: Arc<AtomicU64>,
    /// Stream-local locks serialize writes and finalization without blocking other phones.
    videos: tokio::sync::Mutex<VideoSessions>,
    cameras: Mutex<BTreeMap<String, (CameraPhone, Instant)>>,
}

fn validate_stream_id(stream: &str) -> Result<(), ExperimentError> {
    if stream.is_empty()
        || stream.len() > MAX_STREAM_ID_CHARS
        || !stream
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
    {
        return Err(ExperimentError::InvalidVideo(format!(
            "stream id must be 1-{MAX_STREAM_ID_CHARS} letters, digits, '-' or '_'"
        )));
    }
    Ok(())
}

impl ExperimentStore {
    /// Without a `snapshot_source` every experiment's snapshot fails, so
    /// clients that gate on it refuse arm and motor actions.
    #[must_use]
    pub fn new(
        directory: PathBuf,
        journal: JournalHandle,
        snapshot_source: Option<Arc<dyn ExperimentVehicle>>,
    ) -> Self {
        Self {
            directory,
            journal,
            snapshot_source,
            active: Arc::new(Mutex::new(None)),
            operation: tokio::sync::Mutex::new(()),
            marker_sequence: Arc::new(AtomicU64::new(0)),
            videos: tokio::sync::Mutex::new(VideoSessions::default()),
            cameras: Mutex::new(BTreeMap::new()),
        }
    }

    /// Records a heartbeat from a phone camera page and returns the
    /// experiment it should be recording, if any.
    pub fn camera_heartbeat(
        &self,
        stream: &str,
        label: &str,
        recording: Option<Uuid>,
    ) -> Result<Option<ActiveExperiment>, ExperimentError> {
        validate_stream_id(stream)?;
        let label: String = label.trim().chars().take(MAX_CAMERA_LABEL_CHARS).collect();
        let phone = CameraPhone {
            stream: stream.to_owned(),
            label: if label.is_empty() {
                stream.to_owned()
            } else {
                label
            },
            recording,
            last_seen_ns: wall_time_ns(),
        };
        self.cameras
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .insert(stream.to_owned(), (phone, Instant::now()));
        Ok(self.active())
    }

    /// Removes a phone's registration; returns whether it was registered.
    pub fn camera_disconnect(&self, stream: &str) -> bool {
        self.cameras
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .remove(stream)
            .is_some()
    }

    /// Phones that sent a heartbeat within [`CAMERA_TIMEOUT`].
    pub fn cameras(&self) -> Vec<CameraPhone> {
        let mut cameras = self
            .cameras
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        cameras.retain(|_, (_, seen)| seen.elapsed() < CAMERA_TIMEOUT);
        cameras.values().map(|(phone, _)| phone.clone()).collect()
    }

    /// Video streams of the given experiment, active or recently stopped.
    pub async fn videos(&self, experiment_id: Uuid) -> Vec<VideoStream> {
        let streams: Vec<_> = self
            .videos
            .lock()
            .await
            .session(experiment_id)
            .map(|session| session.streams.values().cloned().collect())
            .unwrap_or_default();
        let mut videos = Vec::with_capacity(streams.len());
        for stream in streams {
            videos.push(stream.lock().await.info.clone());
        }
        videos
    }

    /// Appends chunk `sequence` of a phone recording. Chunks must arrive in
    /// order starting at 0; a repeated chunk is acknowledged without being
    /// written again so phones can retry after a lost response.
    pub async fn append_video(
        &self,
        experiment_id: Uuid,
        stream: &str,
        sequence: u64,
        last: bool,
        content_type: &str,
        data: &[u8],
    ) -> Result<VideoStream, ExperimentError> {
        validate_stream_id(stream)?;
        let mut sessions = self.videos.lock().await;
        let session = sessions
            .session(experiment_id)
            .ok_or(ExperimentError::VideoClosed(experiment_id))?;
        if !session.streams.contains_key(stream) {
            if sequence != 0 {
                return Err(ExperimentError::VideoOutOfOrder(format!(
                    "stream {stream} must start at chunk 0, got {sequence}"
                )));
            }
            let extension = video_extension(content_type).ok_or_else(|| {
                ExperimentError::InvalidVideo(format!(
                    "unsupported video content type {content_type:?}"
                ))
            })?;
            let file = format!("video-{stream}.{extension}");
            let path = session.directory.join(&file);
            fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&path)
                .await
                .map_err(|source| ExperimentError::Write {
                    path: path.clone(),
                    source,
                })?;
            session.streams.insert(
                stream.to_owned(),
                Arc::new(tokio::sync::Mutex::new(VideoState {
                    info: VideoStream {
                        stream: stream.to_owned(),
                        file,
                        bytes: 0,
                        chunks: 0,
                        updated_at_ns: wall_time_ns(),
                        finished: false,
                    },
                    path,
                    final_sequence: None,
                })),
            );
        }
        let state = Arc::clone(session.streams.get(stream).expect("stream inserted"));
        drop(sessions);
        let mut state = state.lock().await;
        if sequence < state.info.chunks {
            if state.final_sequence == Some(sequence) && !state.info.finished {
                state.info.bytes = finalize_video(&state.path).await?;
                state.info.finished = true;
                state.info.updated_at_ns = wall_time_ns();
            }
            return Ok(state.info.clone());
        }
        if state.final_sequence.is_some() {
            return Err(ExperimentError::VideoOutOfOrder(format!(
                "stream {stream} is already finished"
            )));
        }
        if sequence > state.info.chunks {
            return Err(ExperimentError::VideoOutOfOrder(format!(
                "stream {stream} expects chunk {}, got {sequence}",
                state.info.chunks
            )));
        }
        let write_error = |source| ExperimentError::Write {
            path: state.path.clone(),
            source,
        };
        let mut file = fs::OpenOptions::new()
            .append(true)
            .open(&state.path)
            .await
            .map_err(write_error)?;
        file.write_all(data).await.map_err(write_error)?;
        file.sync_data().await.map_err(write_error)?;
        drop(file);
        state.info.bytes += data.len() as u64;
        state.info.chunks += 1;
        state.info.updated_at_ns = wall_time_ns();
        if last {
            state.final_sequence = Some(sequence);
            state.info.bytes = finalize_video(&state.path).await?;
            state.info.finished = true;
            state.info.updated_at_ns = wall_time_ns();
        }
        Ok(state.info.clone())
    }

    #[must_use]
    pub fn active(&self) -> Option<ActiveExperiment> {
        self.lock_active().clone()
    }

    pub async fn start(
        &self,
        name: &str,
        description: &str,
        telemetry_profile: Option<&str>,
    ) -> Result<ActiveExperiment, ExperimentError> {
        let name = name.trim();
        let description = description.trim();
        let telemetry_profile = telemetry_profile
            .map(str::trim)
            .filter(|profile| !profile.is_empty());
        if name.is_empty() {
            return Err(ExperimentError::MissingName);
        }
        if name.chars().count() > MAX_NAME_CHARS {
            return Err(ExperimentError::NameTooLong);
        }
        if description.chars().count() > MAX_DESCRIPTION_CHARS {
            return Err(ExperimentError::DescriptionTooLong);
        }
        if telemetry_profile.is_some_and(|profile| profile.chars().count() > MAX_PROFILE_CHARS) {
            return Err(ExperimentError::ProfileTooLong);
        }
        let _operation = self.operation.lock().await;
        if let Some(active) = self.active() {
            return Err(ExperimentError::AlreadyRunning(active.name));
        }
        let id = Uuid::new_v4();
        let started_at_ns = wall_time_ns();
        let run_id = self.journal.run_id();
        let directory = self.directory.join(directory_name(id, name, started_at_ns));
        create_experiment_directory(
            &self.directory,
            &directory,
            &details_text(
                id,
                name,
                description,
                run_id,
                started_at_ns,
                telemetry_profile,
            ),
        )
        .await?;
        let mut fields = Map::new();
        fields.insert("experiment_id".to_owned(), json!(id));
        fields.insert("name".to_owned(), json!(name));
        fields.insert("description".to_owned(), json!(description));
        fields.insert("directory".to_owned(), json!(directory));
        fields.insert("telemetry_profile".to_owned(), json!(telemetry_profile));
        let marker = self
            .journal
            .append_diagnostic(marker(
                &self.journal,
                &self.marker_sequence,
                id,
                DiagnosticLevel::Info,
                "experiment.started",
                format!("Experiment {name:?} started"),
                fields,
            ))
            .await;
        let start_sequence = match marker {
            Ok(record) => record.sequence,
            Err(error) => {
                let _ = fs::remove_dir_all(&directory).await;
                return Err(error.into());
            }
        };
        let experiment = ActiveExperiment {
            id,
            name: name.to_owned(),
            description: description.to_owned(),
            run_id,
            started_at_ns,
            start_cursor: format_cursor(start_sequence),
            directory,
            telemetry_profile: telemetry_profile.map(str::to_owned),
            snapshot: SnapshotStatus::capturing(),
            onboard_logs_at_start: Err("not listed yet".to_owned()),
            start_sequence,
        };
        self.videos.lock().await.current = Some(VideoSession {
            experiment_id: id,
            directory: experiment.directory.clone(),
            stopped_at: None,
            streams: BTreeMap::new(),
        });
        *self.lock_active() = Some(experiment.clone());
        tokio::spawn(record_snapshot(
            self.snapshot_source.clone(),
            experiment.directory.clone(),
            id,
            self.active.clone(),
            self.journal.clone(),
            self.marker_sequence.clone(),
        ));
        Ok(experiment)
    }

    /// Journals a labelled `experiment.mark` event so analysis can split the
    /// experiment into named segments.
    pub async fn mark(&self, label: &str) -> Result<Value, ExperimentError> {
        let label = label.trim();
        if label.is_empty() {
            return Err(ExperimentError::MissingLabel);
        }
        if label.chars().count() > MAX_MARK_CHARS {
            return Err(ExperimentError::LabelTooLong);
        }
        let experiment = self.active().ok_or(ExperimentError::NotRunning)?;
        let mut fields = Map::new();
        fields.insert("experiment_id".to_owned(), json!(experiment.id));
        fields.insert("label".to_owned(), json!(label));
        let record = self
            .journal
            .append_diagnostic(marker(
                &self.journal,
                &self.marker_sequence,
                experiment.id,
                DiagnosticLevel::Info,
                "experiment.mark",
                format!("Mark: {label}"),
                fields,
            ))
            .await?;
        Ok(json!({
            "label": label,
            "cursor": format_cursor(record.sequence),
            "elapsed_s": wall_time_ns().saturating_sub(experiment.started_at_ns) as f64 / 1e9,
        }))
    }

    /// Writes the experiment log, then clears the active experiment. On a
    /// write failure the experiment stays active so the caller can retry.
    pub async fn stop(&self) -> Result<SavedExperiment, ExperimentError> {
        let _operation = self.operation.lock().await;
        let experiment = self.active().ok_or(ExperimentError::NotRunning)?;
        let stopped_at_ns = wall_time_ns();
        let mut fields = Map::new();
        fields.insert("experiment_id".to_owned(), json!(experiment.id));
        fields.insert("name".to_owned(), json!(experiment.name));
        let end_sequence = self
            .journal
            .append_diagnostic(marker(
                &self.journal,
                &self.marker_sequence,
                experiment.id,
                DiagnosticLevel::Info,
                "experiment.stopped",
                format!("Experiment {:?} stopped", experiment.name),
                fields,
            ))
            .await?
            .sequence;
        let path = experiment.directory.join(MAVLOG_FILE);
        let header = json!({
            "schema_version": FILE_SCHEMA_VERSION,
            "experiment": {
                "id": experiment.id,
                "name": experiment.name,
                "description": experiment.description,
                "run_id": experiment.run_id,
                "started_at_ns": experiment.started_at_ns,
                "stopped_at_ns": stopped_at_ns,
                "start_cursor": experiment.start_cursor,
                "end_cursor": format_cursor(end_sequence),
                "snapshot": experiment.snapshot,
                "telemetry_profile": experiment.telemetry_profile,
            },
        });
        let record_count = self
            .write_log(&path, &header, experiment.start_sequence, end_sequence)
            .await?;
        {
            let mut sessions = self.videos.lock().await;
            if let Some(mut session) = sessions.current.take() {
                session.stopped_at = Some(Instant::now());
                sessions.previous = Some(session);
            }
        }
        *self.lock_active() = None;
        tokio::spawn(collect_onboard_logs(
            self.snapshot_source.clone(),
            experiment.directory.clone(),
            experiment.id,
            experiment.onboard_logs_at_start.clone(),
            self.journal.clone(),
            self.marker_sequence.clone(),
        ));
        Ok(SavedExperiment {
            experiment,
            stopped_at_ns,
            end_cursor: format_cursor(end_sequence),
            record_count,
            path,
        })
    }

    async fn write_log(
        &self,
        path: &Path,
        header: &Value,
        first_sequence: u64,
        last_sequence: u64,
    ) -> Result<u64, ExperimentError> {
        let temporary = path.with_extension("json.partial");
        let write_error = |source| ExperimentError::Write {
            path: path.to_owned(),
            source,
        };
        let result = async {
            let file = fs::File::create(&temporary).await.map_err(write_error)?;
            let mut writer = BufWriter::new(file);
            let mut opening = serde_json::to_string(header).expect("header is serializable");
            opening.pop();
            opening.push_str(",\"records\":[");
            writer
                .write_all(opening.as_bytes())
                .await
                .map_err(write_error)?;
            let mut count = 0_u64;
            let mut cursor = first_sequence.saturating_sub(1);
            'read: loop {
                let read = self
                    .journal
                    .read_after_bounded(cursor, READ_BATCH_RECORDS)
                    .await?;
                if read.records.is_empty() {
                    break;
                }
                for record in read.records {
                    if record.sequence > last_sequence {
                        break 'read;
                    }
                    cursor = record.sequence;
                    let mut line = if count == 0 { "\n" } else { ",\n" }.to_owned();
                    line.push_str(
                        &serde_json::to_string(&record.to_api_value())
                            .expect("record is serializable"),
                    );
                    writer
                        .write_all(line.as_bytes())
                        .await
                        .map_err(write_error)?;
                    count += 1;
                }
            }
            writer
                .write_all(format!("\n],\"record_count\":{count}}}\n").as_bytes())
                .await
                .map_err(write_error)?;
            writer.flush().await.map_err(write_error)?;
            writer.get_ref().sync_all().await.map_err(write_error)?;
            drop(writer);
            fs::rename(&temporary, path).await.map_err(write_error)?;
            Ok(count)
        }
        .await;
        if result.is_err() {
            let _ = fs::remove_file(&temporary).await;
        }
        result
    }

    fn lock_active(&self) -> MutexGuard<'_, Option<ActiveExperiment>> {
        self.active
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

fn marker(
    journal: &JournalHandle,
    sequence: &AtomicU64,
    experiment_id: Uuid,
    level: DiagnosticLevel,
    event: &str,
    message: String,
    fields: Map<String, Value>,
) -> DiagnosticEvent {
    DiagnosticEvent {
        schema_version: SCHEMA_VERSION,
        run_id: journal.run_id(),
        source: "linkhub".to_owned(),
        source_instance: "linkhub-experiment".to_owned(),
        source_sequence: sequence.fetch_add(1, Ordering::Relaxed) + 1,
        source_wall_time_ns: wall_time_ns(),
        source_monotonic_ns: None,
        sim_time_ns: None,
        sim_time_quality: None,
        level,
        category: "experiment".to_owned(),
        event: event.to_owned(),
        message,
        correlation_id: Some(experiment_id),
        causation_id: None,
        fields,
        related_records: Vec::new(),
    }
}

/// Captures the snapshot, publishes its status on the active experiment (if
/// it is still the same one) and journals the outcome.
async fn record_snapshot(
    source: Option<Arc<dyn ExperimentVehicle>>,
    directory: PathBuf,
    experiment_id: Uuid,
    active: Arc<Mutex<Option<ActiveExperiment>>>,
    journal: JournalHandle,
    sequence: Arc<AtomicU64>,
) {
    let status = match &source {
        Some(source) => capture_snapshot(source.as_ref(), &directory).await,
        None => failed_snapshot(&directory, "no MAVLink link is configured".to_owned()).await,
    };
    let onboard_logs = match &source {
        Some(source) => source.list_onboard_logs().await,
        None => Err("no MAVLink link is configured".to_owned()),
    };
    let (level, message) = match &status.error {
        None => (
            DiagnosticLevel::Info,
            format!(
                "Snapshot captured: {} parameters, {} scripts",
                status.parameter_count.unwrap_or_default(),
                status.scripts.len()
            ),
        ),
        Some(error) => (DiagnosticLevel::Error, format!("Snapshot failed: {error}")),
    };
    let mut fields = Map::new();
    fields.insert("experiment_id".to_owned(), json!(experiment_id));
    fields.insert("snapshot".to_owned(), json!(status));
    let event = marker(
        &journal,
        &sequence,
        experiment_id,
        level,
        "experiment.snapshot",
        message,
        fields,
    );
    if let Err(error) = journal.append_diagnostic(event).await {
        tracing::warn!(%error, "could not journal experiment snapshot outcome");
    }
    // Published after journaling so a completed status implies the record exists.
    if let Some(experiment) = active
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .as_mut()
        .filter(|experiment| experiment.id == experiment_id)
    {
        experiment.snapshot = status;
        experiment.onboard_logs_at_start = onboard_logs;
    }
}

/// Saves every onboard log that is new or has grown since the experiment
/// started as `onboard-<id>.BIN`, summarised in `onboard_logs.json`. Runs after
/// stop, so its outcome is journaled outside the experiment's `mavlog.json`.
async fn collect_onboard_logs(
    source: Option<Arc<dyn ExperimentVehicle>>,
    directory: PathBuf,
    experiment_id: Uuid,
    at_start: Result<Vec<(u16, u32)>, String>,
    journal: JournalHandle,
    sequence: Arc<AtomicU64>,
) {
    let summary_path = directory.join(ONBOARD_LOGS_FILE);
    let write_summary = |summary: Value| {
        let path = summary_path.clone();
        async move {
            let bytes = serde_json::to_vec_pretty(&summary).expect("summary is serializable");
            if let Err(error) = write_file(&path, &bytes).await {
                tracing::warn!(%error, "could not record onboard log download");
            }
        }
    };
    let summary = match (source, at_start) {
        (None, _) => json!({"state": "skipped", "error": "no MAVLink link is configured"}),
        (Some(_), Err(error)) => json!({
            "state": "skipped",
            "error": format!("onboard logs were not listed at the start: {error}"),
        }),
        (Some(source), Ok(at_start)) => {
            write_summary(json!({"state": "downloading", "at_start": at_start})).await;
            match source.list_onboard_logs().await {
                Err(error) => json!({"state": "failed", "error": error, "at_start": at_start}),
                Ok(now) => {
                    let changed: Vec<_> = now
                        .into_iter()
                        .filter(|entry| !at_start.contains(entry))
                        .collect();
                    if !changed.is_empty() {
                        journal_onboard_logs(
                            &journal,
                            &sequence,
                            experiment_id,
                            json!({"state": "downloading", "count": changed.len()}),
                        )
                        .await;
                    }
                    let mut logs = Vec::new();
                    let mut failed = false;
                    for (id, size) in changed {
                        let file = format!("onboard-{id}.BIN");
                        let progress = OnboardLogProgress {
                            journal: &journal,
                            sequence: &sequence,
                            experiment_id,
                            id,
                            reported_step: 0,
                        };
                        let saved = match source.download_onboard_log(id, progress).await {
                            Ok(bytes) => write_file(&directory.join(&file), &bytes)
                                .await
                                .map(|()| bytes.len()),
                            Err(error) => Err(error),
                        };
                        logs.push(match saved {
                            Ok(bytes) => {
                                json!({"id": id, "size": size, "bytes": bytes, "file": file})
                            }
                            Err(error) => {
                                failed = true;
                                json!({"id": id, "size": size, "error": error})
                            }
                        });
                    }
                    json!({
                        "state": if failed { "failed" } else { "complete" },
                        "logs": logs,
                        "at_start": at_start,
                    })
                }
            }
        }
    };
    write_summary(summary.clone()).await;
    journal_onboard_logs(&journal, &sequence, experiment_id, summary).await;
}

async fn journal_onboard_logs(
    journal: &JournalHandle,
    sequence: &AtomicU64,
    experiment_id: Uuid,
    summary: Value,
) {
    let level = if matches!(
        summary["state"].as_str(),
        Some("complete" | "downloading" | "progress")
    ) {
        DiagnosticLevel::Info
    } else {
        DiagnosticLevel::Warning
    };
    let mut fields = Map::new();
    fields.insert("experiment_id".to_owned(), json!(experiment_id));
    fields.insert("onboard_logs".to_owned(), summary.clone());
    let event = marker(
        journal,
        sequence,
        experiment_id,
        level,
        "experiment.onboard_logs",
        format!(
            "Onboard logs {}",
            summary["state"].as_str().unwrap_or("unknown")
        ),
        fields,
    );
    if let Err(error) = journal.append_diagnostic(event).await {
        tracing::warn!(%error, "could not journal onboard log download");
    }
}

fn snapshot_script_path(directory: &Path, name: &str) -> Result<PathBuf, String> {
    let mut components = Path::new(name).components();
    if name.contains(['\\', ':'])
        || name.starts_with('.')
        || !matches!(
            (components.next(), components.next()),
            (Some(Component::Normal(_)), None)
        )
    {
        return Err(format!("refusing unsafe script name {name:?}"));
    }
    Ok(directory.join(name))
}

/// Reads firmware, the packed parameter table and every Lua script, writing
/// them and `snapshot.json` into `directory`.
async fn capture_snapshot(source: &dyn ExperimentVehicle, directory: &Path) -> SnapshotStatus {
    let mut status = SnapshotStatus::capturing();
    let result: Result<Map<String, Value>, String> = async {
        let firmware = source.read_firmware().await?;
        let packed = source.read_file(PARAM_PACK_PATH, false).await?;
        let parameters = decode_param_pack(&packed).map_err(|error| error.to_string())?;
        write_file(&directory.join(PARAM_PACK_FILE), &packed).await?;
        let parameter_json = serde_json::to_vec_pretty(&json!({ "parameters": parameters }))
            .expect("parameters are serializable");
        write_file(&directory.join(PARAMETERS_FILE), &parameter_json).await?;
        status.parameter_count = Some(parameters.len());

        let scripts = directory.join(SCRIPTS_DIRECTORY);
        fs::create_dir_all(&scripts)
            .await
            .map_err(|error| format!("create {}: {error}", scripts.display()))?;
        let mut names: Vec<String> = source
            .list_scripts(VEHICLE_SCRIPTS)
            .await?
            .into_iter()
            .filter(|name| name.ends_with(".lua"))
            .collect();
        names.sort();
        for name in names {
            let path = snapshot_script_path(&scripts, &name)?;
            let content = source
                .read_file(&format!("{VEHICLE_SCRIPTS}/{name}"), true)
                .await?;
            let mut file = fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&path)
                .await
                .map_err(|error| format!("create {}: {error}", path.display()))?;
            file.write_all(&content)
                .await
                .map_err(|error| format!("write {}: {error}", path.display()))?;
            status.scripts.push(ScriptFile {
                bytes: content.len() as u64,
                sha256: format!("{:x}", Sha256::digest(&content)),
                name,
            });
        }
        // Rates are informative; a vehicle that rejects one query still has a
        // complete parameter and script snapshot.
        let mut intervals = Vec::with_capacity(SNAPSHOT_STREAMS.len());
        for message in SNAPSHOT_STREAMS {
            intervals.push(match source.read_message_interval(message).await {
                Ok(reply) => json!({
                    "message": message,
                    "interval_us": reply["interval_us"],
                    "rate_hz": reply["rate_hz"],
                }),
                Err(error) => json!({ "message": message, "error": error }),
            });
        }
        let mut details = Map::new();
        details.insert("firmware".to_owned(), firmware);
        details.insert("message_intervals".to_owned(), Value::Array(intervals));
        Ok(details)
    }
    .await;
    match result {
        Ok(details) => {
            status.state = SnapshotState::Complete;
            status.completed_at_ns = Some(wall_time_ns());
            if let Err(error) = write_snapshot(directory, &status, details).await {
                status.state = SnapshotState::Failed;
                status.error = Some(error);
            }
            status
        }
        Err(error) => failed_snapshot(directory, error).await,
    }
}

async fn failed_snapshot(directory: &Path, error: String) -> SnapshotStatus {
    let status = SnapshotStatus {
        state: SnapshotState::Failed,
        error: Some(error),
        completed_at_ns: Some(wall_time_ns()),
        ..SnapshotStatus::capturing()
    };
    if let Err(error) = write_snapshot(directory, &status, Map::new()).await {
        tracing::warn!(%error, "could not record failed experiment snapshot");
    }
    status
}

/// Writes `snapshot.json`: the status plus `firmware` and `message_intervals`
/// when they were captured.
async fn write_snapshot(
    directory: &Path,
    status: &SnapshotStatus,
    details: Map<String, Value>,
) -> Result<(), String> {
    let mut document = serde_json::to_value(status).expect("snapshot is serializable");
    let object = document
        .as_object_mut()
        .expect("snapshot status is an object");
    object.insert("firmware".to_owned(), Value::Null);
    object.insert("message_intervals".to_owned(), Value::Null);
    object.extend(details);
    object.insert(
        "parameters_file".to_owned(),
        json!(status.parameter_count.map(|_| PARAMETERS_FILE)),
    );
    let bytes = serde_json::to_vec_pretty(&document).expect("snapshot is serializable");
    write_file(&directory.join(SNAPSHOT_FILE), &bytes).await
}

async fn write_file(path: &Path, content: &[u8]) -> Result<(), String> {
    fs::write(path, content)
        .await
        .map_err(|error| format!("write {}: {error}", path.display()))
}

async fn finalize_video(path: &Path) -> Result<u64, ExperimentError> {
    let extension = path.extension().expect("video extension").to_string_lossy();
    let indexed = path.with_extension(format!("indexed.{extension}"));
    let failure = |message| ExperimentError::VideoFinalize {
        path: path.to_owned(),
        message,
    };
    let result = async {
        let mut command = tokio::process::Command::new("ffmpeg");
        command
            .kill_on_drop(true)
            .args(["-nostdin", "-hide_banner", "-loglevel", "error", "-y", "-i"])
            .arg(path)
            .args(["-map", "0", "-c", "copy"]);
        if extension == "mp4" {
            command.args(["-movflags", "+faststart"]);
        } else {
            command.args(["-cues_to_front", "1"]);
        }
        let output = tokio::time::timeout(VIDEO_FINALIZE_TIMEOUT, command.arg(&indexed).output())
            .await
            .map_err(|_| failure("FFmpeg timed out after 120 seconds".to_owned()))?
            .map_err(|error| failure(format!("run FFmpeg (required on PATH): {error}")))?;
        if !output.status.success() {
            return Err(failure(format!(
                "FFmpeg {}: {}",
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            )));
        }
        let file = fs::OpenOptions::new()
            .write(true)
            .open(&indexed)
            .await
            .map_err(|error| failure(error.to_string()))?;
        file.sync_all()
            .await
            .map_err(|error| failure(error.to_string()))?;
        let bytes = file
            .metadata()
            .await
            .map_err(|error| failure(error.to_string()))?
            .len();
        drop(file);
        fs::rename(&indexed, path)
            .await
            .map_err(|error| failure(error.to_string()))?;
        Ok(bytes)
    }
    .await;
    if result.is_err()
        && let Err(error) = fs::remove_file(&indexed).await
        && error.kind() != std::io::ErrorKind::NotFound
    {
        tracing::warn!(path = %indexed.display(), %error, "could not remove incomplete indexed video");
    }
    result
}

fn video_extension(content_type: &str) -> Option<&'static str> {
    let essence = content_type
        .split(';')
        .next()
        .unwrap_or_default()
        .trim()
        .to_ascii_lowercase();
    match essence.as_str() {
        "video/webm" => Some("webm"),
        "video/mp4" => Some("mp4"),
        "video/x-matroska" => Some("mkv"),
        _ => None,
    }
}

fn directory_name(id: Uuid, name: &str, started_at_ns: u64) -> String {
    let id = id.simple().to_string();
    format!("{}_{}_{}", utc_stamp(started_at_ns), slug(name), &id[..8])
}

fn details_text(
    id: Uuid,
    name: &str,
    description: &str,
    run_id: Uuid,
    started_at_ns: u64,
    telemetry_profile: Option<&str>,
) -> String {
    let description = if description.is_empty() {
        "(none)"
    } else {
        description
    };
    format!(
        "Name: {name}\nStarted (UTC): {}\nExperiment ID: {id}\nLinkHub run ID: {run_id}\nTelemetry profile: {}\n\nWhat is being tested:\n{description}\n",
        utc_iso(started_at_ns),
        telemetry_profile.unwrap_or("(not reported)"),
    )
}

/// Creates the experiment directory and its details file. `create_dir`
/// (not `_all`) on the leaf refuses to reuse an existing directory.
async fn create_experiment_directory(
    root: &Path,
    directory: &Path,
    details: &str,
) -> Result<(), ExperimentError> {
    let write_error = |path: &Path| {
        let path = path.to_owned();
        move |source| ExperimentError::Write { path, source }
    };
    fs::create_dir_all(root).await.map_err(write_error(root))?;
    fs::create_dir(directory)
        .await
        .map_err(write_error(directory))?;
    let details_path = directory.join(DETAILS_FILE);
    if let Err(error) = fs::write(&details_path, details).await {
        let _ = fs::remove_dir_all(directory).await;
        return Err(write_error(&details_path)(error));
    }
    Ok(())
}

fn slug(name: &str) -> String {
    let mut slug = String::new();
    for character in name.chars() {
        if character.is_ascii_alphanumeric() {
            slug.push(character.to_ascii_lowercase());
        } else if !slug.is_empty() && !slug.ends_with('-') {
            slug.push('-');
        }
        if slug.len() >= MAX_SLUG_CHARS {
            break;
        }
    }
    let slug = slug.trim_end_matches('-');
    if slug.is_empty() {
        "experiment".to_owned()
    } else {
        slug.to_owned()
    }
}

fn utc_stamp(wall_time_ns: u64) -> String {
    let (year, month, day, hour, minute, second) = utc_parts(wall_time_ns);
    format!("{year:04}{month:02}{day:02}T{hour:02}{minute:02}{second:02}Z")
}

fn utc_iso(wall_time_ns: u64) -> String {
    let (year, month, day, hour, minute, second) = utc_parts(wall_time_ns);
    format!("{year:04}-{month:02}-{day:02}T{hour:02}:{minute:02}:{second:02}Z")
}

fn utc_parts(wall_time_ns: u64) -> (i64, u32, u32, u64, u64, u64) {
    let seconds = wall_time_ns / 1_000_000_000;
    let (year, month, day) = civil_from_days((seconds / 86_400) as i64);
    let second_of_day = seconds % 86_400;
    (
        year,
        month,
        day,
        second_of_day / 3_600,
        second_of_day % 3_600 / 60,
        second_of_day % 60,
    )
}

/// Proleptic Gregorian date for days since 1970-01-01 (H. Hinnant's algorithm).
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    let shifted = days + 719_468;
    let era = shifted.div_euclid(146_097);
    let day_of_era = shifted.rem_euclid(146_097);
    let year_of_era =
        (day_of_era - day_of_era / 1_460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let month_index = (5 * day_of_year + 2) / 153;
    let day = (day_of_year - (153 * month_index + 2) / 5 + 1) as u32;
    let month = if month_index < 10 {
        month_index + 3
    } else {
        month_index - 9
    } as u32;
    let year = year_of_era + era * 400 + i64::from(month <= 2);
    (year, month, day)
}

#[cfg(test)]
mod tests {
    use std::process::Output;

    use tempfile::TempDir;

    use super::*;
    use crate::{
        journal::JournalConfig,
        records::{Direction, MavlinkFrame, RecordPayload},
    };

    fn heartbeat() -> RecordPayload {
        RecordPayload::MavlinkFrame(MavlinkFrame {
            link_id: "test".to_owned(),
            direction: Direction::Rx,
            protocol_version: 2,
            sequence: 0,
            system_id: 1,
            component_id: 1,
            message_id: 0,
            message_name: "HEARTBEAT".to_owned(),
            fields: Map::new(),
            signed: false,
            frame: vec![0xfd],
        })
    }

    #[test]
    fn names_files_by_utc_start_and_slug() {
        assert_eq!(utc_stamp(0), "19700101T000000Z");
        assert_eq!(utc_stamp(1_735_689_599_000_000_000), "20241231T235959Z");
        assert_eq!(utc_stamp(951_782_400_000_000_000), "20000229T000000Z");
        assert_eq!(utc_iso(1_735_689_599_000_000_000), "2024-12-31T23:59:59Z");
        assert_eq!(slug("  Rotor spin-up #3 (5 m/s)!"), "rotor-spin-up-3-5-m-s");
        assert_eq!(slug("***"), "experiment");
    }

    #[tokio::test]
    async fn saves_only_the_experiment_window_and_rejects_overlap() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path().join("journal"), run_id);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        let store = ExperimentStore::new(temp.path().join("experiments"), journal.clone(), None);
        journal.append(heartbeat(), None).await.expect("before");

        let started = store
            .start(" Spin test ", "bench", None)
            .await
            .expect("start");
        assert_eq!(started.name, "Spin test");
        let directory_name = started
            .directory
            .file_name()
            .unwrap()
            .to_string_lossy()
            .into_owned();
        assert!(directory_name.ends_with(&format!(
            "_spin-test_{}",
            &started.id.simple().to_string()[..8]
        )));
        assert_eq!(
            started.directory.parent(),
            Some(temp.path().join("experiments").as_path())
        );
        let details =
            std::fs::read_to_string(started.directory.join(DETAILS_FILE)).expect("details");
        assert!(details.starts_with("Name: Spin test\nStarted (UTC): "));
        assert!(details.contains(&format!("Experiment ID: {}\n", started.id)));
        assert!(details.ends_with("What is being tested:\nbench\n"));
        assert!(!started.directory.join(MAVLOG_FILE).exists());
        assert!(matches!(
            store.start("other", "", None).await,
            Err(ExperimentError::AlreadyRunning(_))
        ));
        let snapshot = settled_snapshot(&store).await;
        assert_eq!(snapshot.state, SnapshotState::Failed);
        assert_eq!(
            snapshot.error.as_deref(),
            Some("no MAVLink link is configured")
        );
        journal.append(heartbeat(), None).await.expect("during");
        assert!(matches!(
            store.mark("  ").await,
            Err(ExperimentError::MissingLabel)
        ));
        let mark = store.mark(" collective high ").await.expect("mark");
        assert_eq!(mark["label"], "collective high");
        journal.append(heartbeat(), None).await.expect("during");
        let saved = store.stop().await.expect("stop");
        journal.append(heartbeat(), None).await.expect("after");

        assert!(store.active().is_none());
        assert!(matches!(
            store.stop().await,
            Err(ExperimentError::NotRunning)
        ));
        assert!(matches!(
            store.mark("late").await,
            Err(ExperimentError::NotRunning)
        ));
        assert_eq!(saved.record_count, 6);
        assert_eq!(saved.path, started.directory.join(MAVLOG_FILE));
        let log: Value =
            serde_json::from_slice(&std::fs::read(&saved.path).expect("log")).expect("log JSON");
        assert_eq!(log["experiment"]["name"], "Spin test");
        assert_eq!(log["experiment"]["description"], "bench");
        assert_eq!(log["experiment"]["snapshot"]["state"], "failed");
        assert_eq!(log["experiment"]["telemetry_profile"], Value::Null);
        assert_eq!(log["record_count"], 6);
        let records = log["records"].as_array().expect("records");
        assert_eq!(records.len(), 6);
        assert_eq!(records[0]["data"]["event"], "experiment.started");
        assert_eq!(records[1]["data"]["event"], "experiment.snapshot");
        assert_eq!(records[2]["data"]["message_name"], "HEARTBEAT");
        assert_eq!(records[3]["data"]["event"], "experiment.mark");
        assert_eq!(records[3]["data"]["fields"]["label"], "collective high");
        assert_eq!(records[5]["data"]["event"], "experiment.stopped");

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn appends_ordered_video_chunks_until_after_stop() {
        let temp = TempDir::new().expect("temp directory");
        let config = JournalConfig::for_directory(temp.path().join("journal"), Uuid::new_v4());
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        let store = ExperimentStore::new(temp.path().join("experiments"), journal.clone(), None);
        let started = store.start("Video", "", None).await.expect("start");
        let id = started.id;
        let webm = "video/webm;codecs=vp8,opus";
        let recording = video_fixture(temp.path(), "webm").await;
        let split = recording.len() / 2;

        assert!(matches!(
            store.append_video(id, "../x", 0, false, webm, b"a").await,
            Err(ExperimentError::InvalidVideo(_))
        ));
        assert!(matches!(
            store
                .append_video(id, "p1", 0, false, "text/plain", b"a")
                .await,
            Err(ExperimentError::InvalidVideo(_))
        ));
        assert!(matches!(
            store.append_video(id, "p1", 1, false, webm, b"a").await,
            Err(ExperimentError::VideoOutOfOrder(_))
        ));
        store
            .append_video(id, "p1", 0, false, webm, &recording[..split])
            .await
            .expect("chunk 0");
        let repeated = store
            .append_video(id, "p1", 0, false, webm, &recording[..split])
            .await
            .expect("retry");
        assert_eq!((repeated.chunks, repeated.bytes), (1, split as u64));
        assert!(matches!(
            store.append_video(id, "p1", 2, false, webm, b"c").await,
            Err(ExperimentError::VideoOutOfOrder(_))
        ));
        store
            .append_video(id, "p2", 0, false, "video/mp4", b"m")
            .await
            .expect("phone 2");

        store.stop().await.expect("stop");
        let last = store
            .append_video(id, "p1", 1, true, webm, &recording[split..])
            .await
            .expect("final");
        assert!(last.finished);
        let path = started.directory.join("video-p1.webm");
        let finalized = std::fs::read(&path).expect("finalized");
        let retry = store
            .append_video(id, "p1", 1, true, webm, &recording[split..])
            .await
            .expect("final retry");
        assert!(retry.finished);
        assert_eq!(retry.chunks, 2);
        assert_eq!(retry.bytes, finalized.len() as u64);
        assert_eq!(std::fs::read(&path).expect("retry file"), finalized);
        assert!(matches!(
            store.append_video(id, "p1", 2, false, webm, b"e").await,
            Err(ExperimentError::VideoOutOfOrder(_))
        ));
        assert!(matches!(
            store
                .append_video(Uuid::new_v4(), "p1", 0, false, webm, b"a")
                .await,
            Err(ExperimentError::VideoClosed(_))
        ));
        assert_ne!(finalized, recording);
        assert_eq!(
            std::fs::read(started.directory.join("video-p2.mp4")).expect("p2"),
            b"m"
        );
        let streams = store.videos(id).await;
        assert_eq!(streams.len(), 2);

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    async fn ffmpeg(arguments: &[&str]) -> Output {
        let output = tokio::process::Command::new("ffmpeg")
            .args(["-nostdin", "-hide_banner", "-y"])
            .args(arguments)
            .output()
            .await
            .expect("FFmpeg must be installed on PATH for video tests");
        assert!(
            output.status.success(),
            "FFmpeg {}: {}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        );
        output
    }

    async fn video_fixture(directory: &Path, extension: &str) -> Vec<u8> {
        let path = directory.join(format!("input.{extension}"));
        let mut arguments = vec![
            "-v",
            "error",
            "-f",
            "lavfi",
            "-i",
            "testsrc2=size=32x32:rate=10",
            "-t",
            "2",
            "-an",
            "-g",
            "5",
            "-c:v",
        ];
        if extension == "mp4" {
            arguments.extend(["libx264", "-movflags", "+frag_keyframe+empty_moov"]);
        } else {
            arguments.extend(["libvpx", "-live", "1"]);
        }
        arguments.push(path.to_str().expect("fixture path"));
        ffmpeg(&arguments).await;
        std::fs::read(path).expect("fixture")
    }

    fn frame_hashes(output: &Output) -> Vec<String> {
        String::from_utf8(output.stdout.clone())
            .expect("frame hashes")
            .lines()
            .filter(|line| !line.starts_with('#') && !line.is_empty())
            .map(|line| line.rsplit(',').next().expect("hash").trim().to_owned())
            .collect()
    }

    fn ebml_children(mut bytes: &[u8]) -> Vec<(u64, &[u8])> {
        let mut elements = Vec::new();
        while !bytes.is_empty() {
            let id_len = bytes[0].leading_zeros() as usize + 1;
            let id = bytes[..id_len]
                .iter()
                .fold(0_u64, |value, byte| (value << 8) | u64::from(*byte));
            bytes = &bytes[id_len..];
            let size_len = bytes[0].leading_zeros() as usize + 1;
            let size = bytes[1..size_len].iter().fold(
                usize::from(bytes[0] & (0xff_u16 >> size_len) as u8),
                |value, byte| (value << 8) | usize::from(*byte),
            );
            bytes = &bytes[size_len..];
            elements.push((id, &bytes[..size]));
            bytes = &bytes[size..];
        }
        elements
    }

    #[tokio::test]
    async fn finalizes_video_with_seek_indexes_and_unchanged_packets() {
        let temp = TempDir::new().expect("temp directory");
        for extension in ["webm", "mp4", "mkv"] {
            let recording = video_fixture(temp.path(), extension).await;
            let path = temp.path().join(format!("video.{extension}"));
            std::fs::write(&path, &recording).expect("recording");
            let original = temp.path().join(format!("input.{extension}"));
            let original = original.to_str().expect("original path");
            let path_str = path.to_str().expect("video path");
            let packet_args = [
                "-v",
                "error",
                "-i",
                original,
                "-map",
                "0:v",
                "-c",
                "copy",
                "-f",
                "framehash",
                "-",
            ];
            let original_packets = frame_hashes(&ffmpeg(&packet_args).await);
            assert_eq!(original_packets.len(), 20);

            let bytes = finalize_video(&path).await.expect("finalize");
            let finalized = std::fs::read(&path).expect("indexed video");
            assert_eq!(bytes, finalized.len() as u64);
            assert!(!path.with_extension(format!("indexed.{extension}")).exists());
            if extension == "mp4" {
                let mut boxes = Vec::new();
                let mut remaining = finalized.as_slice();
                while !remaining.is_empty() {
                    let size =
                        u32::from_be_bytes(remaining[..4].try_into().expect("box size")) as usize;
                    boxes.push(&remaining[4..8]);
                    remaining = &remaining[size..];
                }
                let moov = boxes.iter().position(|id| *id == b"moov").expect("moov");
                let mdat = boxes.iter().position(|id| *id == b"mdat").expect("mdat");
                assert!(moov < mdat);
                assert!(!boxes.contains(&b"moof".as_slice()));
            } else {
                let segment = ebml_children(&finalized)
                    .into_iter()
                    .find(|(id, _)| *id == 0x18538067)
                    .expect("Segment")
                    .1;
                let elements = ebml_children(segment);
                let cues = elements
                    .iter()
                    .position(|(id, _)| *id == 0x1c53bb6b)
                    .expect("Cues");
                let cluster = elements
                    .iter()
                    .position(|(id, _)| *id == 0x1f43b675)
                    .expect("Cluster");
                assert!(cues < cluster);
                let info = elements
                    .iter()
                    .find(|(id, _)| *id == 0x1549a966)
                    .expect("Info")
                    .1;
                assert!(
                    ebml_children(info).iter().any(|(id, _)| *id == 0x4489),
                    "Duration"
                );
            }
            let indexed_packets = ffmpeg(&[
                "-v",
                "error",
                "-i",
                path_str,
                "-map",
                "0:v",
                "-c",
                "copy",
                "-f",
                "framehash",
                "-",
            ])
            .await;
            assert_eq!(frame_hashes(&indexed_packets), original_packets);
            let decoded = ffmpeg(&["-v", "error", "-i", path_str, "-f", "framemd5", "-"]).await;
            let sought = ffmpeg(&[
                "-v",
                "error",
                "-ss",
                "1.4",
                "-i",
                path_str,
                "-frames:v",
                "1",
                "-f",
                "framemd5",
                "-",
            ])
            .await;
            assert_eq!(
                frame_hashes(&sought),
                vec![frame_hashes(&decoded)[14].clone()]
            );
        }
    }

    #[tokio::test]
    async fn failed_video_finalization_preserves_upload_and_retries_without_appending() {
        let temp = TempDir::new().expect("temp directory");
        let config = JournalConfig::for_directory(temp.path().join("journal"), Uuid::new_v4());
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        let store = ExperimentStore::new(temp.path().join("experiments"), journal.clone(), None);
        let started = store.start("Invalid video", "", None).await.expect("start");
        let path = started.directory.join("video-phone.webm");
        for _ in 0..2 {
            assert!(matches!(
                store
                    .append_video(started.id, "phone", 0, true, "video/webm", b"invalid")
                    .await,
                Err(ExperimentError::VideoFinalize { .. })
            ));
            assert_eq!(std::fs::read(&path).expect("original"), b"invalid");
            let status = store.videos(started.id).await;
            assert!(!status[0].finished);
            assert_eq!((status[0].chunks, status[0].bytes), (1, 7));
            assert!(!path.with_extension("indexed.webm").exists());
        }
        assert!(matches!(
            store
                .append_video(started.id, "phone", 1, false, "video/webm", b"later")
                .await,
            Err(ExperimentError::VideoOutOfOrder(_))
        ));
        let recording = video_fixture(temp.path(), "webm").await;
        std::fs::write(&path, recording).expect("restore recording");
        let status = store
            .append_video(started.id, "phone", 0, true, "video/webm", b"invalid")
            .await
            .expect("retry finalization");
        assert!(status.finished);
        assert_eq!(status.chunks, 1);
        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    async fn settled_snapshot(store: &ExperimentStore) -> SnapshotStatus {
        for _ in 0..500 {
            let snapshot = store.active().expect("active").snapshot;
            if snapshot.state != SnapshotState::Capturing {
                return snapshot;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        panic!("snapshot did not settle");
    }

    struct FakeVehicle {
        files: BTreeMap<String, Vec<u8>>,
        /// Onboard logs as `(id, size, content)`; tests change it after start.
        logs: Mutex<Vec<(u16, u32, Vec<u8>)>>,
    }

    impl ExperimentVehicle for FakeVehicle {
        fn read_firmware(&self) -> SnapshotFuture<'_, Value> {
            Box::pin(async { Ok(json!({"flight_sw_version": 67_570_175})) })
        }

        fn list_scripts<'a>(&'a self, directory: &'a str) -> SnapshotFuture<'a, Vec<String>> {
            Box::pin(async move {
                Ok(self
                    .files
                    .keys()
                    .filter_map(|path| path.strip_prefix(&format!("{directory}/")))
                    .map(str::to_owned)
                    .collect())
            })
        }

        fn read_file<'a>(&'a self, path: &'a str, _verify: bool) -> SnapshotFuture<'a, Vec<u8>> {
            Box::pin(async move {
                self.files
                    .get(path)
                    .cloned()
                    .ok_or_else(|| format!("no {path}"))
            })
        }

        fn read_message_interval<'a>(&'a self, message: &'a str) -> SnapshotFuture<'a, Value> {
            Box::pin(async move {
                if message == "VIBRATION" {
                    return Err("VIBRATION interval: timed out".to_owned());
                }
                Ok(json!({"message": message, "interval_us": 5_000, "rate_hz": 200.0}))
            })
        }

        fn list_onboard_logs(&self) -> SnapshotFuture<'_, Vec<(u16, u32)>> {
            Box::pin(async move {
                Ok(self
                    .logs
                    .lock()
                    .unwrap()
                    .iter()
                    .map(|(id, size, _)| (*id, *size))
                    .collect())
            })
        }

        fn download_onboard_log<'a>(
            &'a self,
            id: u16,
            mut progress: OnboardLogProgress<'a>,
        ) -> SnapshotFuture<'a, Vec<u8>> {
            Box::pin(async move {
                progress.update(0, 0).await;
                progress.update(49, 500).await;
                progress.update(50, 500).await;
                progress.update(51, 500).await;
                self.logs
                    .lock()
                    .unwrap()
                    .iter()
                    .find(|(log, _, _)| *log == id)
                    .map(|(_, _, content)| content.clone())
                    .ok_or_else(|| format!("no log {id}"))
            })
        }
    }

    #[test]
    fn snapshot_scripts_require_plain_filenames_on_every_platform() {
        let directory = Path::new("snapshot");
        for name in [
            "C:rawes.lua",
            "C:\\rawes.lua",
            "\\\\server\\share\\rawes.lua",
            "rawes.lua:payload.lua",
            "../rawes.lua",
            "/rawes.lua",
            "sub/rawes.lua",
            "sub\\rawes.lua",
            ".hidden.lua",
        ] {
            assert!(
                snapshot_script_path(directory, name).is_err(),
                "accepted unsafe filename {name:?}"
            );
        }
        assert_eq!(
            snapshot_script_path(directory, "rawes.lua").expect("plain filename"),
            directory.join("rawes.lua")
        );
    }

    #[tokio::test]
    async fn snapshot_scripts_do_not_overwrite_existing_files() {
        let temp = TempDir::new().expect("temp directory");
        let scripts = temp.path().join(SCRIPTS_DIRECTORY);
        fs::create_dir(&scripts).await.expect("scripts directory");
        let path = scripts.join("rawes.lua");
        fs::write(&path, b"preserved")
            .await
            .expect("existing script");
        let vehicle = FakeVehicle {
            files: BTreeMap::from([
                (PARAM_PACK_PATH.to_owned(), vec![0x1b, 0x67, 0, 0, 0, 0]),
                ("/APM/scripts/rawes.lua".to_owned(), b"replacement".to_vec()),
            ]),
            logs: Mutex::new(Vec::new()),
        };

        let snapshot = capture_snapshot(&vehicle, temp.path()).await;
        assert_eq!(snapshot.state, SnapshotState::Failed);
        assert!(
            snapshot.error.as_deref().is_some_and(|error| {
                error.starts_with("create ") && error.contains("rawes.lua")
            }),
            "{:?}",
            snapshot.error
        );
        assert_eq!(
            fs::read(&path).await.expect("existing script"),
            b"preserved"
        );
    }

    #[tokio::test]
    async fn captures_parameters_scripts_and_firmware_at_start() {
        let temp = TempDir::new().expect("temp directory");
        let config = JournalConfig::for_directory(temp.path().join("journal"), Uuid::new_v4());
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        let mut packed = Vec::new();
        packed.extend_from_slice(&0x671b_u16.to_le_bytes());
        packed.extend_from_slice(&1_u16.to_le_bytes());
        packed.extend_from_slice(&1_u16.to_le_bytes());
        packed.extend([4, 0xc0]);
        packed.extend(b"ATC_RAT_YAW_P");
        packed.extend(0.18_f32.to_le_bytes());
        let vehicle = Arc::new(FakeVehicle {
            files: BTreeMap::from([
                (PARAM_PACK_PATH.to_owned(), packed.clone()),
                ("/APM/scripts/rawes.lua".to_owned(), b"-- rawes".to_vec()),
                ("/APM/scripts/notes.txt".to_owned(), b"ignored".to_vec()),
            ]),
            logs: Mutex::new(vec![(4, 100, b"old".to_vec()), (5, 200, b"open".to_vec())]),
        });
        let store = ExperimentStore::new(
            temp.path().join("experiments"),
            journal.clone(),
            Some(vehicle.clone()),
        );
        let started = store
            .start("Snapshot", "", Some("wifi"))
            .await
            .expect("start");
        assert_eq!(started.snapshot.state, SnapshotState::Capturing);

        let snapshot = settled_snapshot(&store).await;
        assert_eq!(
            snapshot.state,
            SnapshotState::Complete,
            "{:?}",
            snapshot.error
        );
        assert_eq!(snapshot.parameter_count, Some(1));
        assert_eq!(snapshot.scripts.len(), 1);
        assert_eq!(snapshot.scripts[0].name, "rawes.lua");
        assert_eq!(
            snapshot.scripts[0].sha256,
            format!("{:x}", Sha256::digest(b"-- rawes"))
        );
        let directory = &started.directory;
        assert_eq!(
            std::fs::read(directory.join(PARAM_PACK_FILE)).unwrap(),
            packed
        );
        assert_eq!(
            std::fs::read(directory.join("scripts").join("rawes.lua")).unwrap(),
            b"-- rawes"
        );
        assert!(!directory.join("scripts").join("notes.txt").exists());
        let parameters: Value =
            serde_json::from_slice(&std::fs::read(directory.join(PARAMETERS_FILE)).unwrap())
                .unwrap();
        assert_eq!(parameters["parameters"][0]["name"], "ATC_RAT_YAW_P");
        let summary: Value =
            serde_json::from_slice(&std::fs::read(directory.join(SNAPSHOT_FILE)).unwrap()).unwrap();
        assert_eq!(summary["state"], "complete");
        assert_eq!(summary["firmware"]["flight_sw_version"], 67_570_175);
        let intervals = summary["message_intervals"].as_array().expect("intervals");
        assert_eq!(intervals.len(), SNAPSHOT_STREAMS.len());
        assert_eq!(intervals[0]["message"], "ATTITUDE_QUATERNION");
        assert_eq!(intervals[0]["interval_us"], 5_000);
        assert!(
            intervals
                .iter()
                .any(|entry| entry["message"] == "VIBRATION" && entry["error"].is_string())
        );
        assert!(
            std::fs::read_to_string(directory.join(DETAILS_FILE))
                .unwrap()
                .contains("Telemetry profile: wifi\n")
        );

        // Log 5 grew and log 6 is new during the experiment; log 4 is untouched.
        *vehicle.logs.lock().unwrap() = vec![
            (4, 100, b"old".to_vec()),
            (5, 300, b"open+more".to_vec()),
            (6, 50, b"new".to_vec()),
        ];
        let mut events = journal.subscribe();
        store.stop().await.expect("stop");
        let states = tokio::time::timeout(Duration::from_secs(5), async {
            let mut states = Vec::new();
            loop {
                let record = events.recv().await.expect("journal event");
                if let RecordPayload::Diagnostic(event) = &record.payload {
                    if event.event != "experiment.onboard_logs" {
                        continue;
                    }
                    let summary = &event.fields["onboard_logs"];
                    states.push((
                        summary["state"].as_str().unwrap().to_owned(),
                        summary["percent"].as_u64(),
                    ));
                    if summary["state"] == "complete" {
                        return states;
                    }
                }
            }
        })
        .await
        .expect("log download events");
        assert_eq!(
            states,
            [
                ("downloading".to_owned(), None),
                ("progress".to_owned(), Some(10)),
                ("progress".to_owned(), Some(10)),
                ("complete".to_owned(), None),
            ]
        );
        let mut onboard = Value::Null;
        for _ in 0..500 {
            if let Ok(bytes) = std::fs::read(directory.join(ONBOARD_LOGS_FILE)) {
                onboard = serde_json::from_slice(&bytes).unwrap();
                if onboard["state"] != "downloading" {
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert_eq!(onboard["state"], "complete", "{onboard}");
        let ids: Vec<_> = onboard["logs"]
            .as_array()
            .unwrap()
            .iter()
            .map(|log| log["id"].as_u64().unwrap())
            .collect();
        assert_eq!(ids, [5, 6]);
        assert_eq!(
            std::fs::read(directory.join("onboard-5.BIN")).unwrap(),
            b"open+more"
        );
        assert_eq!(
            std::fs::read(directory.join("onboard-6.BIN")).unwrap(),
            b"new"
        );
        assert!(!directory.join("onboard-4.BIN").exists());

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }
}
