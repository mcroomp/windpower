//! Operator experiments: a named journal window saved into its own directory.
//!
//! Starting creates `<directory>/<UTC start>_<slug>_<id>/` holding
//! `experiment.txt` (the details entered at start) and journals an
//! `experiment.started` diagnostic marker. Stopping journals
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
//! their final chunk.

use std::{
    collections::BTreeMap,
    net::IpAddr,
    path::{Path, PathBuf},
    sync::{
        Mutex, MutexGuard,
        atomic::{AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};

use serde::Serialize;
use serde_json::{Map, Value, json};
use tokio::{
    fs,
    io::{AsyncWriteExt, BufWriter},
};
use uuid::Uuid;

use crate::{
    journal::{JournalError, JournalHandle},
    records::{DiagnosticEvent, DiagnosticLevel, SCHEMA_VERSION, format_cursor, wall_time_ns},
};

const FILE_SCHEMA_VERSION: u16 = 1;
const MAX_NAME_CHARS: usize = 200;
const MAX_DESCRIPTION_CHARS: usize = 10_000;
const MAX_SLUG_CHARS: usize = 48;
const READ_BATCH_RECORDS: usize = 10_000;
const DETAILS_FILE: &str = "experiment.txt";
const MAVLOG_FILE: &str = "mavlog.json";
const MAX_STREAM_ID_CHARS: usize = 32;
pub const VIDEO_GRACE: Duration = Duration::from_secs(120);
pub const CAMERA_TIMEOUT: Duration = Duration::from_secs(10);
const MAX_CAMERA_LABEL_CHARS: usize = 40;

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
}

struct VideoSession {
    experiment_id: Uuid,
    directory: PathBuf,
    stopped_at: Option<Instant>,
    streams: BTreeMap<String, VideoState>,
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
    #[serde(skip)]
    start_sequence: u64,
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
    #[error("could not write experiment file {path}: {source}")]
    Write {
        path: PathBuf,
        source: std::io::Error,
    },
}

pub struct ExperimentStore {
    directory: PathBuf,
    journal: JournalHandle,
    active: Mutex<Option<ActiveExperiment>>,
    /// Serializes start/stop so a slow save cannot race a new start.
    operation: tokio::sync::Mutex<()>,
    marker_sequence: AtomicU64,
    /// Held across chunk writes so each stream's file grows in order.
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
    #[must_use]
    pub fn new(directory: PathBuf, journal: JournalHandle) -> Self {
        Self {
            directory,
            journal,
            active: Mutex::new(None),
            operation: tokio::sync::Mutex::new(()),
            marker_sequence: AtomicU64::new(0),
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
        self.videos
            .lock()
            .await
            .session(experiment_id)
            .map(|session| {
                session
                    .streams
                    .values()
                    .map(|state| state.info.clone())
                    .collect()
            })
            .unwrap_or_default()
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
                VideoState {
                    info: VideoStream {
                        stream: stream.to_owned(),
                        file,
                        bytes: 0,
                        chunks: 0,
                        updated_at_ns: wall_time_ns(),
                        finished: false,
                    },
                    path,
                },
            );
        }
        let state = session.streams.get_mut(stream).expect("stream inserted");
        if sequence < state.info.chunks {
            return Ok(state.info.clone());
        }
        if state.info.finished {
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
        state.info.bytes += data.len() as u64;
        state.info.chunks += 1;
        state.info.updated_at_ns = wall_time_ns();
        state.info.finished = last;
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
    ) -> Result<ActiveExperiment, ExperimentError> {
        let name = name.trim();
        let description = description.trim();
        if name.is_empty() {
            return Err(ExperimentError::MissingName);
        }
        if name.chars().count() > MAX_NAME_CHARS {
            return Err(ExperimentError::NameTooLong);
        }
        if description.chars().count() > MAX_DESCRIPTION_CHARS {
            return Err(ExperimentError::DescriptionTooLong);
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
            &details_text(id, name, description, run_id, started_at_ns),
        )
        .await?;
        let mut fields = Map::new();
        fields.insert("experiment_id".to_owned(), json!(id));
        fields.insert("name".to_owned(), json!(name));
        fields.insert("description".to_owned(), json!(description));
        fields.insert("directory".to_owned(), json!(directory));
        let marker = self
            .journal
            .append_diagnostic(self.marker(
                id,
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
            start_sequence,
        };
        self.videos.lock().await.current = Some(VideoSession {
            experiment_id: id,
            directory: experiment.directory.clone(),
            stopped_at: None,
            streams: BTreeMap::new(),
        });
        *self.lock_active() = Some(experiment.clone());
        Ok(experiment)
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
            .append_diagnostic(self.marker(
                experiment.id,
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

    fn marker(
        &self,
        experiment_id: Uuid,
        event: &str,
        message: String,
        fields: Map<String, Value>,
    ) -> DiagnosticEvent {
        DiagnosticEvent {
            schema_version: SCHEMA_VERSION,
            run_id: self.journal.run_id(),
            source: "linkhub".to_owned(),
            source_instance: "linkhub-experiment".to_owned(),
            source_sequence: self.marker_sequence.fetch_add(1, Ordering::Relaxed) + 1,
            source_wall_time_ns: wall_time_ns(),
            source_monotonic_ns: None,
            sim_time_ns: None,
            sim_time_quality: None,
            level: DiagnosticLevel::Info,
            category: "experiment".to_owned(),
            event: event.to_owned(),
            message,
            correlation_id: Some(experiment_id),
            causation_id: None,
            fields,
            related_records: Vec::new(),
        }
    }

    fn lock_active(&self) -> MutexGuard<'_, Option<ActiveExperiment>> {
        self.active
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
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
) -> String {
    let description = if description.is_empty() {
        "(none)"
    } else {
        description
    };
    format!(
        "Name: {name}\nStarted (UTC): {}\nExperiment ID: {id}\nLinkHub run ID: {run_id}\n\nWhat is being tested:\n{description}\n",
        utc_iso(started_at_ns)
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
        let store = ExperimentStore::new(temp.path().join("experiments"), journal.clone());
        journal.append(heartbeat(), None).await.expect("before");

        let started = store.start(" Spin test ", "bench").await.expect("start");
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
            store.start("other", "").await,
            Err(ExperimentError::AlreadyRunning(_))
        ));
        journal.append(heartbeat(), None).await.expect("during");
        journal.append(heartbeat(), None).await.expect("during");
        let saved = store.stop().await.expect("stop");
        journal.append(heartbeat(), None).await.expect("after");

        assert!(store.active().is_none());
        assert!(matches!(
            store.stop().await,
            Err(ExperimentError::NotRunning)
        ));
        assert_eq!(saved.record_count, 4);
        assert_eq!(saved.path, started.directory.join(MAVLOG_FILE));
        let log: Value =
            serde_json::from_slice(&std::fs::read(&saved.path).expect("log")).expect("log JSON");
        assert_eq!(log["experiment"]["name"], "Spin test");
        assert_eq!(log["experiment"]["description"], "bench");
        assert_eq!(log["record_count"], 4);
        let records = log["records"].as_array().expect("records");
        assert_eq!(records.len(), 4);
        assert_eq!(records[0]["data"]["event"], "experiment.started");
        assert_eq!(records[1]["data"]["message_name"], "HEARTBEAT");
        assert_eq!(records[3]["data"]["event"], "experiment.stopped");

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn appends_ordered_video_chunks_until_after_stop() {
        let temp = TempDir::new().expect("temp directory");
        let config = JournalConfig::for_directory(temp.path().join("journal"), Uuid::new_v4());
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        let store = ExperimentStore::new(temp.path().join("experiments"), journal.clone());
        let started = store.start("Video", "").await.expect("start");
        let id = started.id;
        let webm = "video/webm;codecs=vp8,opus";

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
            .append_video(id, "p1", 0, false, webm, b"ab")
            .await
            .expect("chunk 0");
        let repeated = store
            .append_video(id, "p1", 0, false, webm, b"ab")
            .await
            .expect("retry");
        assert_eq!((repeated.chunks, repeated.bytes), (1, 2));
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
            .append_video(id, "p1", 1, true, webm, b"cd")
            .await
            .expect("final");
        assert!(last.finished);
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
        assert_eq!(
            std::fs::read(started.directory.join("video-p1.webm")).expect("p1"),
            b"abcd"
        );
        assert_eq!(
            std::fs::read(started.directory.join("video-p2.mp4")).expect("p2"),
            b"m"
        );
        let streams = store.videos(id).await;
        assert_eq!(streams.len(), 2);

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }
}
