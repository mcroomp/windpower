use std::{
    collections::BTreeMap,
    sync::{
        Arc, Mutex,
        atomic::{AtomicU64, Ordering},
    },
    time::Instant,
};

use serde::Serialize;
use serde_json::Value;
use thiserror::Error;
use tokio::sync::watch;

use super::TransferStats;
use crate::{
    journal::JournalHandle,
    records::{DiagnosticEvent, DiagnosticLevel, wall_time_ns},
};

/// Finished transfers kept for inspection.
const KEPT_FINISHED: usize = 32;
/// Downloaded bytes kept for collection, oldest evicted first.
const KEPT_CONTENT_BYTES: usize = 128 * 1024 * 1024;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TransferKind {
    FtpDownload,
    LogDownload,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TransferState {
    /// Waiting for another transfer to release the link's file or log session.
    Queued,
    Running,
    Complete,
    Failed,
    Cancelled,
}

impl TransferState {
    #[must_use]
    pub fn is_finished(self) -> bool {
        matches!(self, Self::Complete | Self::Failed | Self::Cancelled)
    }
}

/// How a transfer ended.
#[derive(Clone, Debug)]
pub enum Outcome {
    /// The verified content, held for collection.
    Complete(Vec<u8>),
    Failed(String),
    Cancelled,
}

#[derive(Clone, Debug, Serialize)]
pub struct TransferInfo {
    pub id: u64,
    pub kind: TransferKind,
    /// The remote path or log being read.
    pub target: String,
    pub state: TransferState,
    pub total_bytes: u64,
    /// Bytes available in order from the start.
    pub done_bytes: u64,
    /// Bytes that have arrived anywhere, including beyond a hole.
    pub received_bytes: u64,
    pub created_ns: u64,
    /// Time spent running (not queued).
    pub elapsed_ms: u64,
    /// `done_bytes` over the elapsed time.
    pub rate_bytes_per_s: f64,
    pub stats: TransferStats,
    pub error: Option<String>,
    /// The verified content can still be fetched.
    pub content_available: bool,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub enum ContentError {
    #[error("no such transfer")]
    NotFound,
    #[error("the transfer is not complete")]
    NotReady,
    #[error("the transfer's content is no longer held")]
    Gone,
}

/// What [`TransferRegistry::cancel`] did.
#[derive(Debug, Eq, PartialEq)]
pub enum CancelResult {
    NotFound,
    /// A running or queued transfer was told to stop.
    Requested,
    /// A finished transfer was forgotten.
    Removed,
}

struct Job {
    info: TransferInfo,
    running_since: Option<Instant>,
    cancel: watch::Sender<bool>,
    content: Option<Arc<Vec<u8>>>,
}

impl Job {
    fn refresh(&mut self) {
        if let Some(since) = self.running_since {
            let elapsed = since.elapsed();
            self.info.elapsed_ms = elapsed.as_millis() as u64;
            self.info.rate_bytes_per_s = rate(self.info.done_bytes, elapsed.as_secs_f64());
        }
    }
}

fn rate(bytes: u64, seconds: f64) -> f64 {
    if seconds > 0.0 {
        bytes as f64 / seconds
    } else {
        0.0
    }
}

#[derive(Default)]
struct Inner {
    next_id: u64,
    jobs: BTreeMap<u64, Job>,
}

impl Inner {
    /// Keeps the newest finished transfers and the newest content within budget.
    fn trim(&mut self) {
        let finished: Vec<u64> = self
            .jobs
            .iter()
            .filter(|(_, job)| job.info.state.is_finished())
            .map(|(id, _)| *id)
            .collect();
        for id in &finished[..finished.len().saturating_sub(KEPT_FINISHED)] {
            self.jobs.remove(id);
        }
        let mut held = 0_usize;
        for job in self.jobs.values_mut().rev() {
            if let Some(content) = &job.content {
                held += content.len();
                if held > KEPT_CONTENT_BYTES {
                    job.content = None;
                    job.info.content_available = false;
                }
            }
        }
    }
}

/// Tracks transfers from submission to collection, serves them from
/// `/v1/mavlink/transfers`, and journals a diagnostic when each one starts and
/// ends (`linkhub query <dir> diagnostics --source linkhub.transfer`).
#[derive(Clone)]
pub struct TransferRegistry {
    inner: Arc<Mutex<Inner>>,
    journal: JournalHandle,
    sequence: Arc<AtomicU64>,
}

impl TransferRegistry {
    #[must_use]
    pub fn new(journal: JournalHandle) -> Self {
        Self {
            inner: Arc::new(Mutex::new(Inner::default())),
            journal,
            sequence: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Registers a queued transfer.
    #[must_use]
    pub fn submit(&self, kind: TransferKind, target: impl Into<String>) -> TransferHandle {
        let mut inner = self.inner.lock().expect("transfer registry lock");
        inner.next_id += 1;
        let id = inner.next_id;
        let (cancel, cancel_rx) = watch::channel(false);
        inner.jobs.insert(
            id,
            Job {
                info: TransferInfo {
                    id,
                    kind,
                    target: target.into(),
                    state: TransferState::Queued,
                    total_bytes: 0,
                    done_bytes: 0,
                    received_bytes: 0,
                    created_ns: wall_time_ns(),
                    elapsed_ms: 0,
                    rate_bytes_per_s: 0.0,
                    stats: TransferStats::default(),
                    error: None,
                    content_available: false,
                },
                running_since: None,
                cancel,
                content: None,
            },
        );
        TransferHandle {
            id,
            registry: self.clone(),
            cancel: cancel_rx,
            finished: false,
        }
    }

    /// Running and queued transfers first (newest first), then finished ones.
    #[must_use]
    pub fn list(&self) -> Vec<TransferInfo> {
        let mut inner = self.inner.lock().expect("transfer registry lock");
        let mut all: Vec<TransferInfo> = inner
            .jobs
            .values_mut()
            .map(|job| {
                job.refresh();
                job.info.clone()
            })
            .collect();
        all.sort_by_key(|info| (info.state.is_finished(), std::cmp::Reverse(info.id)));
        all
    }

    #[must_use]
    pub fn get(&self, id: u64) -> Option<TransferInfo> {
        let mut inner = self.inner.lock().expect("transfer registry lock");
        inner.jobs.get_mut(&id).map(|job| {
            job.refresh();
            job.info.clone()
        })
    }

    pub fn content(&self, id: u64) -> Result<Arc<Vec<u8>>, ContentError> {
        let inner = self.inner.lock().expect("transfer registry lock");
        let job = inner.jobs.get(&id).ok_or(ContentError::NotFound)?;
        if job.info.state != TransferState::Complete {
            return Err(ContentError::NotReady);
        }
        job.content.clone().ok_or(ContentError::Gone)
    }

    /// Stops a queued or running transfer, or forgets a finished one.
    pub fn cancel(&self, id: u64) -> CancelResult {
        let mut inner = self.inner.lock().expect("transfer registry lock");
        let Some(job) = inner.jobs.get(&id) else {
            return CancelResult::NotFound;
        };
        if job.info.state.is_finished() {
            inner.jobs.remove(&id);
            CancelResult::Removed
        } else {
            let _ = job.cancel.send(true);
            CancelResult::Requested
        }
    }

    fn update(&self, id: u64, apply: impl FnOnce(&mut Job)) {
        let mut inner = self.inner.lock().expect("transfer registry lock");
        if let Some(job) = inner.jobs.get_mut(&id) {
            apply(job);
        }
    }

    async fn emit(&self, info: &TransferInfo, event: &str, level: DiagnosticLevel) {
        let message = match (info.state, &info.error) {
            (TransferState::Running, _) => format!(
                "{:?} of {} started ({} bytes)",
                info.kind, info.target, info.total_bytes
            ),
            (TransferState::Failed, Some(error)) => {
                format!("{:?} of {} failed: {error}", info.kind, info.target)
            }
            _ => format!(
                "{:?} of {} {:?}: {} of {} bytes in {} ms ({:.1} KiB/s, {:.1}% duplicates)",
                info.kind,
                info.target,
                info.state,
                info.done_bytes,
                info.total_bytes,
                info.elapsed_ms,
                info.rate_bytes_per_s / 1024.0,
                info.stats.duplicate_ratio() * 100.0
            ),
        };
        let Value::Object(fields) =
            serde_json::to_value(info).expect("TransferInfo serializes as an object")
        else {
            unreachable!("TransferInfo is a struct");
        };
        match level {
            DiagnosticLevel::Warning | DiagnosticLevel::Error => {
                tracing::warn!(event, ?fields, "{message}");
            }
            _ => tracing::info!(event, ?fields, "{message}"),
        }
        let diagnostic = DiagnosticEvent {
            schema_version: 1,
            run_id: self.journal.run_id(),
            source: "linkhub.transfer".to_owned(),
            source_instance: "linkhub.transfer".to_owned(),
            source_sequence: self.sequence.fetch_add(1, Ordering::Relaxed) + 1,
            source_wall_time_ns: wall_time_ns(),
            source_monotonic_ns: None,
            sim_time_ns: None,
            sim_time_quality: None,
            level,
            category: "transfer".to_owned(),
            event: event.to_owned(),
            message,
            correlation_id: None,
            causation_id: None,
            fields,
            related_records: Vec::new(),
        };
        if let Err(error) = self.journal.append_diagnostic(diagnostic).await {
            tracing::warn!(%error, "could not journal transfer diagnostic");
        }
    }
}

/// The worker's side of a transfer. Dropping it without calling
/// [`finish`](Self::finish) records the transfer as cancelled.
pub struct TransferHandle {
    id: u64,
    registry: TransferRegistry,
    cancel: watch::Receiver<bool>,
    finished: bool,
}

impl TransferHandle {
    #[must_use]
    pub fn id(&self) -> u64 {
        self.id
    }

    #[must_use]
    pub fn is_cancelled(&self) -> bool {
        *self.cancel.borrow()
    }

    /// Resolves when the transfer is cancelled (for waits that must not outlive it).
    pub async fn cancelled(&mut self) {
        while !*self.cancel.borrow_and_update() {
            if self.cancel.changed().await.is_err() {
                // The registry forgot this transfer; nothing will cancel it.
                std::future::pending::<()>().await;
            }
        }
    }

    /// The transfer left the queue; `total_bytes` is the size to expect.
    pub async fn start(&self, total_bytes: u64) {
        let mut started = None;
        self.registry.update(self.id, |job| {
            job.info.state = TransferState::Running;
            job.info.total_bytes = total_bytes;
            job.running_since = Some(Instant::now());
            started = Some(job.info.clone());
        });
        if let Some(info) = started {
            self.registry
                .emit(&info, "transfer.started", DiagnosticLevel::Info)
                .await;
        }
    }

    pub fn progress(&self, done_bytes: u64, received_bytes: u64, stats: TransferStats) {
        self.registry.update(self.id, |job| {
            job.info.done_bytes = done_bytes;
            job.info.received_bytes = received_bytes;
            job.info.stats = stats;
        });
    }

    pub async fn finish(
        mut self,
        outcome: Outcome,
        done_bytes: u64,
        received_bytes: u64,
        stats: TransferStats,
    ) {
        self.finished = true;
        let (event, level, finished) = {
            let mut inner = self.registry.inner.lock().expect("transfer registry lock");
            let Some(job) = inner.jobs.get_mut(&self.id) else {
                return;
            };
            job.info.done_bytes = done_bytes;
            job.info.received_bytes = received_bytes;
            job.info.stats = stats;
            job.refresh();
            let (state, event, level) = match outcome {
                Outcome::Complete(content) => {
                    job.info.content_available = true;
                    job.content = Some(Arc::new(content));
                    (
                        TransferState::Complete,
                        "transfer.complete",
                        DiagnosticLevel::Info,
                    )
                }
                Outcome::Failed(error) => {
                    job.info.error = Some(error);
                    (
                        TransferState::Failed,
                        "transfer.failed",
                        DiagnosticLevel::Warning,
                    )
                }
                Outcome::Cancelled => (
                    TransferState::Cancelled,
                    "transfer.cancelled",
                    DiagnosticLevel::Warning,
                ),
            };
            job.info.state = state;
            let info = job.info.clone();
            inner.trim();
            (event, level, info)
        };
        self.registry.emit(&finished, event, level).await;
    }
}

impl Drop for TransferHandle {
    fn drop(&mut self) {
        if !self.finished {
            self.registry.update(self.id, |job| {
                if !job.info.state.is_finished() {
                    job.info.state = TransferState::Cancelled;
                }
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use tempfile::TempDir;
    use uuid::Uuid;

    use super::*;
    use crate::journal::JournalConfig;

    async fn registry() -> (TransferRegistry, TempDir, JournalHandle) {
        let temp = TempDir::new().expect("temp directory");
        let (journal, _task) =
            JournalHandle::start(JournalConfig::for_directory(temp.path(), Uuid::new_v4()))
                .await
                .expect("journal");
        (TransferRegistry::new(journal.clone()), temp, journal)
    }

    #[tokio::test]
    async fn a_transfer_moves_from_queued_to_complete_and_keeps_its_content() {
        let (registry, _temp, _journal) = registry().await;
        let handle = registry.submit(TransferKind::FtpDownload, "/APM/LOGS/1.BIN");
        let id = handle.id();
        assert_eq!(registry.get(id).unwrap().state, TransferState::Queued);
        assert_eq!(registry.content(id), Err(ContentError::NotReady));

        handle.start(1000).await;
        let stats = TransferStats {
            packets_received: 5,
            duplicate_packets: 1,
            ..TransferStats::default()
        };
        handle.progress(400, 600, stats);
        let running = registry.get(id).unwrap();
        assert_eq!(running.state, TransferState::Running);
        assert_eq!((running.done_bytes, running.received_bytes), (400, 600));
        assert_eq!(running.total_bytes, 1000);

        handle
            .finish(Outcome::Complete(vec![7; 1000]), 1000, 1000, stats)
            .await;
        let done = registry.get(id).unwrap();
        assert_eq!(done.state, TransferState::Complete);
        assert!(done.content_available);
        assert_eq!(registry.content(id).unwrap().len(), 1000);
    }

    #[tokio::test]
    async fn cancelling_signals_the_worker_and_a_finished_transfer_is_forgotten() {
        let (registry, _temp, _journal) = registry().await;
        let mut handle = registry.submit(TransferKind::LogDownload, "log 7");
        let id = handle.id();
        assert!(!handle.is_cancelled());
        assert_eq!(registry.cancel(id), CancelResult::Requested);
        handle.cancelled().await;
        assert!(handle.is_cancelled());
        handle
            .finish(Outcome::Cancelled, 0, 0, TransferStats::default())
            .await;
        assert_eq!(registry.get(id).unwrap().state, TransferState::Cancelled);
        assert_eq!(registry.cancel(id), CancelResult::Removed);
        assert!(registry.get(id).is_none());
        assert_eq!(registry.cancel(id), CancelResult::NotFound);
    }

    #[tokio::test]
    async fn a_dropped_handle_is_recorded_as_cancelled() {
        let (registry, _temp, _journal) = registry().await;
        let id = {
            let handle = registry.submit(TransferKind::LogDownload, "log 7");
            handle.id()
        };
        assert_eq!(registry.get(id).unwrap().state, TransferState::Cancelled);
    }

    #[tokio::test]
    async fn failures_carry_their_error_and_active_transfers_list_first() {
        let (registry, _temp, _journal) = registry().await;
        let failed = registry.submit(TransferKind::FtpDownload, "/a");
        failed.start(10).await;
        failed
            .finish(
                Outcome::Failed("stalled".to_owned()),
                0,
                0,
                TransferStats::default(),
            )
            .await;
        let _queued = registry.submit(TransferKind::LogDownload, "log 1");
        let list = registry.list();
        assert_eq!(list[0].state, TransferState::Queued);
        assert_eq!(list[1].error.as_deref(), Some("stalled"));
    }

    #[tokio::test]
    async fn old_content_is_evicted_but_the_record_remains() {
        let (registry, _temp, _journal) = registry().await;
        let mut ids = Vec::new();
        for index in 0..3 {
            let handle = registry.submit(TransferKind::FtpDownload, format!("/f{index}"));
            ids.push(handle.id());
            handle.start(1).await;
            let big = vec![0_u8; KEPT_CONTENT_BYTES / 2 + 1];
            handle
                .finish(Outcome::Complete(big), 1, 1, TransferStats::default())
                .await;
        }
        // Only the newest content fits the budget.
        assert_eq!(registry.content(ids[0]), Err(ContentError::Gone));
        assert!(!registry.get(ids[0]).unwrap().content_available);
        assert!(registry.content(ids[2]).is_ok());
    }
}
