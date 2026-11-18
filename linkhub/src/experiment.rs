//! Operator experiments: a named journal window saved as one JSON file.
//!
//! Start and stop are journaled as `experiment.started`/`experiment.stopped`
//! diagnostic markers. Stopping streams every journal record between the two
//! markers (inclusive) into `<directory>/<UTC start>_<slug>_<id>.json`. The
//! active experiment lives in memory; a LinkHub restart ends it unsaved, but
//! the run journal still holds the records.

use std::{
    path::{Path, PathBuf},
    sync::{
        Mutex, MutexGuard,
        atomic::{AtomicU64, Ordering},
    },
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

#[derive(Clone, Debug, Serialize)]
pub struct ActiveExperiment {
    pub id: Uuid,
    pub name: String,
    pub description: String,
    pub run_id: Uuid,
    pub started_at_ns: u64,
    pub start_cursor: String,
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
    #[error("could not write experiment log {path}: {source}")]
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
        }
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
        let mut fields = Map::new();
        fields.insert("experiment_id".to_owned(), json!(id));
        fields.insert("name".to_owned(), json!(name));
        fields.insert("description".to_owned(), json!(description));
        let start_sequence = self
            .journal
            .append_diagnostic(self.marker(
                id,
                "experiment.started",
                format!("Experiment {name:?} started"),
                fields,
            ))
            .await?
            .sequence;
        let experiment = ActiveExperiment {
            id,
            name: name.to_owned(),
            description: description.to_owned(),
            run_id: self.journal.run_id(),
            started_at_ns,
            start_cursor: format_cursor(start_sequence),
            start_sequence,
        };
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
        let path = self.directory.join(file_name(&experiment));
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
        fs::create_dir_all(&self.directory)
            .await
            .map_err(write_error)?;
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

fn file_name(experiment: &ActiveExperiment) -> String {
    let id = experiment.id.simple().to_string();
    format!(
        "{}_{}_{}.json",
        utc_stamp(experiment.started_at_ns),
        slug(&experiment.name),
        &id[..8]
    )
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
    let seconds = wall_time_ns / 1_000_000_000;
    let (year, month, day) = civil_from_days((seconds / 86_400) as i64);
    let second_of_day = seconds % 86_400;
    format!(
        "{year:04}{month:02}{day:02}T{:02}{:02}{:02}Z",
        second_of_day / 3_600,
        second_of_day % 3_600 / 60,
        second_of_day % 60
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
        let name = saved
            .path
            .file_name()
            .unwrap()
            .to_string_lossy()
            .into_owned();
        assert!(name.ends_with(&format!(
            "_spin-test_{}.json",
            &started.id.simple().to_string()[..8]
        )));
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
}
