use std::{
    collections::{HashMap, HashSet},
    io,
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
    time::Duration,
};

use serde::{Deserialize, Serialize};
use tokio::{
    fs,
    sync::{broadcast, mpsc, oneshot},
    task::JoinHandle,
    time,
};
use uuid::Uuid;

use crate::records::{DiagnosticEvent, JournalRecord, RecordPayload};

const CHUNK_MAGIC: &[u8; 8] = b"LHCHNK01";
const CHUNK_VERSION: u16 = 1;

#[derive(Clone, Debug)]
pub struct JournalConfig {
    pub directory: PathBuf,
    pub run_id: Uuid,
    pub max_chunk_bytes: usize,
    pub max_chunk_records: usize,
    pub flush_interval: Duration,
    pub command_capacity: usize,
    pub live_capacity: usize,
}

impl JournalConfig {
    #[must_use]
    pub fn for_directory(directory: impl Into<PathBuf>, run_id: Uuid) -> Self {
        Self {
            directory: directory.into(),
            run_id,
            max_chunk_bytes: 4 * 1024 * 1024,
            max_chunk_records: 4096,
            flush_interval: Duration::from_millis(50),
            command_capacity: 8192,
            live_capacity: 8192,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Chunk {
    version: u16,
    run_id: Uuid,
    first_sequence: u64,
    last_sequence: u64,
    records: Vec<JournalRecord>,
}

#[derive(Debug, thiserror::Error)]
pub enum JournalError {
    #[error("journal task is unavailable")]
    Unavailable,
    #[error("journal I/O failed: {0}")]
    Io(#[from] io::Error),
    #[error("journal encoding failed: {0}")]
    Encode(#[from] rmp_serde::encode::Error),
    #[error("journal decoding failed: {0}")]
    Decode(#[from] rmp_serde::decode::Error),
    #[error("diagnostic event run ID does not match this journal")]
    WrongRun,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
pub struct AppendResult {
    pub sequence: u64,
    pub appended: bool,
}

enum Command {
    Append {
        payload: RecordPayload,
        correlation_id: Option<Uuid>,
        reply: oneshot::Sender<Result<AppendResult, JournalError>>,
    },
    Snapshot {
        reply: oneshot::Sender<JournalSnapshot>,
    },
    Flush {
        reply: oneshot::Sender<Result<(), JournalError>>,
    },
    Shutdown {
        reply: oneshot::Sender<Result<(), JournalError>>,
    },
}

struct JournalSnapshot {
    tail: u64,
    pending: Vec<JournalRecord>,
}

#[derive(Clone)]
pub struct JournalHandle {
    run_id: Uuid,
    directory: Arc<PathBuf>,
    sender: mpsc::Sender<Command>,
    live: broadcast::Sender<Arc<JournalRecord>>,
    tail: Arc<AtomicU64>,
}

impl JournalHandle {
    pub async fn start(config: JournalConfig) -> Result<(Self, JoinHandle<()>), JournalError> {
        validate_config(&config);
        fs::create_dir_all(&config.directory).await?;
        let (sender, receiver) = mpsc::channel(config.command_capacity);
        let (live, _) = broadcast::channel(config.live_capacity);
        let tail = Arc::new(AtomicU64::new(0));
        let handle = Self {
            run_id: config.run_id,
            directory: Arc::new(config.directory.clone()),
            sender,
            live: live.clone(),
            tail: Arc::clone(&tail),
        };
        let task = tokio::spawn(run_journal(config, receiver, live, tail));
        Ok((handle, task))
    }

    #[must_use]
    pub const fn run_id(&self) -> Uuid {
        self.run_id
    }

    #[must_use]
    pub fn tail(&self) -> u64 {
        self.tail.load(Ordering::Acquire)
    }

    pub fn subscribe(&self) -> broadcast::Receiver<Arc<JournalRecord>> {
        self.live.subscribe()
    }

    pub async fn append(
        &self,
        payload: RecordPayload,
        correlation_id: Option<Uuid>,
    ) -> Result<u64, JournalError> {
        Ok(self.append_record(payload, correlation_id).await?.sequence)
    }

    async fn append_record(
        &self,
        payload: RecordPayload,
        correlation_id: Option<Uuid>,
    ) -> Result<AppendResult, JournalError> {
        let (reply, response) = oneshot::channel();
        self.sender
            .send(Command::Append {
                payload,
                correlation_id,
                reply,
            })
            .await
            .map_err(|_| JournalError::Unavailable)?;
        response.await.map_err(|_| JournalError::Unavailable)?
    }

    pub async fn append_diagnostic(
        &self,
        event: DiagnosticEvent,
    ) -> Result<AppendResult, JournalError> {
        if event.run_id != self.run_id {
            return Err(JournalError::WrongRun);
        }
        let correlation_id = event.correlation_id;
        self.append_record(RecordPayload::Diagnostic(Box::new(event)), correlation_id)
            .await
    }

    pub async fn records_after(&self, after: u64) -> Result<Vec<JournalRecord>, JournalError> {
        let (reply, response) = oneshot::channel();
        self.sender
            .send(Command::Snapshot { reply })
            .await
            .map_err(|_| JournalError::Unavailable)?;
        let snapshot = response.await.map_err(|_| JournalError::Unavailable)?;
        let mut records = read_flushed_after(&self.directory, after).await?;
        records.retain(|record| record.sequence <= snapshot.tail);
        let flushed_sequences: HashSet<u64> =
            records.iter().map(|record| record.sequence).collect();
        records.extend(snapshot.pending.into_iter().filter(|record| {
            record.sequence > after && !flushed_sequences.contains(&record.sequence)
        }));
        records.sort_unstable_by_key(|record| record.sequence);
        Ok(records)
    }

    pub async fn flush(&self) -> Result<(), JournalError> {
        self.request_flush(false).await
    }

    pub async fn shutdown(&self) -> Result<(), JournalError> {
        self.request_flush(true).await
    }

    async fn request_flush(&self, shutdown: bool) -> Result<(), JournalError> {
        let (reply, response) = oneshot::channel();
        let command = if shutdown {
            Command::Shutdown { reply }
        } else {
            Command::Flush { reply }
        };
        self.sender
            .send(command)
            .await
            .map_err(|_| JournalError::Unavailable)?;
        response.await.map_err(|_| JournalError::Unavailable)?
    }
}

async fn run_journal(
    config: JournalConfig,
    mut receiver: mpsc::Receiver<Command>,
    live: broadcast::Sender<Arc<JournalRecord>>,
    tail: Arc<AtomicU64>,
) {
    let mut pending = Vec::with_capacity(config.max_chunk_records);
    let mut pending_bytes = 0_usize;
    let mut diagnostic_sequences: HashMap<(String, u64), u64> = HashMap::new();
    let mut ticker = time::interval(config.flush_interval);
    ticker.set_missed_tick_behavior(time::MissedTickBehavior::Delay);
    ticker.tick().await;

    loop {
        tokio::select! {
            biased;
            command = receiver.recv() => {
                let Some(command) = command else {
                    let _ = flush_pending(&config, &mut pending).await;
                    return;
                };
                match command {
                    Command::Append { payload, correlation_id, reply } => {
                        let diagnostic_key = match &payload {
                            RecordPayload::Diagnostic(event) => Some((
                                event.source_instance.clone(),
                                event.source_sequence,
                            )),
                            RecordPayload::MavlinkFrame(_) => None,
                        };
                        if let Some(existing) = diagnostic_key
                            .as_ref()
                            .and_then(|key| diagnostic_sequences.get(key))
                        {
                            let _ = reply.send(Ok(AppendResult {
                                sequence: *existing,
                                appended: false,
                            }));
                            continue;
                        }
                        let sequence = tail.fetch_add(1, Ordering::AcqRel) + 1;
                        let record = JournalRecord::new(sequence, correlation_id, payload);
                        pending_bytes += estimated_size(&record);
                        pending.push(record.clone());
                        let _ = live.send(Arc::new(record));
                        if let Some(key) = diagnostic_key {
                            diagnostic_sequences.insert(key, sequence);
                        }
                        let _ = reply.send(Ok(AppendResult {
                            sequence,
                            appended: true,
                        }));
                        if pending.len() >= config.max_chunk_records
                            || pending_bytes >= config.max_chunk_bytes
                        {
                            if let Err(error) = flush_pending(&config, &mut pending).await {
                                tracing::error!(%error, "journal chunk flush failed");
                            }
                            pending_bytes = 0;
                        }
                    }
                    Command::Snapshot { reply } => {
                        let _ = reply.send(JournalSnapshot {
                            tail: tail.load(Ordering::Acquire),
                            pending: pending.clone(),
                        });
                    }
                    Command::Flush { reply } => {
                        let result = flush_pending(&config, &mut pending).await;
                        if result.is_ok() {
                            pending_bytes = 0;
                        }
                        let _ = reply.send(result);
                    }
                    Command::Shutdown { reply } => {
                        let result = flush_pending(&config, &mut pending).await;
                        let _ = reply.send(result);
                        return;
                    }
                }
            }
            _ = ticker.tick() => {
                if let Err(error) = flush_pending(&config, &mut pending).await {
                    tracing::error!(%error, "periodic journal chunk flush failed");
                } else {
                    pending_bytes = 0;
                }
            }
        }
    }
}

async fn flush_pending(
    config: &JournalConfig,
    pending: &mut Vec<JournalRecord>,
) -> Result<(), JournalError> {
    if pending.is_empty() {
        return Ok(());
    }
    let records = std::mem::take(pending);
    let first_sequence = records[0].sequence;
    let last_sequence = records[records.len() - 1].sequence;
    let chunk = Chunk {
        version: CHUNK_VERSION,
        run_id: config.run_id,
        first_sequence,
        last_sequence,
        records,
    };
    let encoded = match rmp_serde::to_vec_named(&chunk) {
        Ok(encoded) => encoded,
        Err(error) => {
            pending.extend(chunk.records);
            return Err(JournalError::Encode(error));
        }
    };
    let final_path = chunk_path(&config.directory, first_sequence, last_sequence);
    let temporary_path = final_path.with_extension("lhc.tmp");
    let mut bytes = Vec::with_capacity(CHUNK_MAGIC.len() + encoded.len());
    bytes.extend_from_slice(CHUNK_MAGIC);
    bytes.extend_from_slice(&encoded);
    let result = async {
        fs::write(&temporary_path, bytes).await?;
        fs::rename(temporary_path, final_path).await
    }
    .await;
    match result {
        Ok(()) => Ok(()),
        Err(error) => {
            pending.extend(chunk.records);
            Err(JournalError::Io(error))
        }
    }
}

async fn read_flushed_after(
    directory: &Path,
    after: u64,
) -> Result<Vec<JournalRecord>, JournalError> {
    let mut entries = fs::read_dir(directory).await?;
    let mut paths = Vec::new();
    while let Some(entry) = entries.next_entry().await? {
        let path = entry.path();
        if path.extension().is_some_and(|extension| extension == "lhc") {
            paths.push(path);
        }
    }
    paths.sort_unstable();

    let mut records = Vec::new();
    for path in paths {
        if chunk_last_sequence(&path).is_some_and(|last_sequence| last_sequence <= after) {
            continue;
        }
        let bytes = fs::read(path).await?;
        if !bytes.starts_with(CHUNK_MAGIC) {
            continue;
        }
        let chunk: Chunk = rmp_serde::from_slice(&bytes[CHUNK_MAGIC.len()..])?;
        if chunk.last_sequence <= after {
            continue;
        }
        records.extend(
            chunk
                .records
                .into_iter()
                .filter(|record| record.sequence > after),
        );
    }
    Ok(records)
}

fn validate_config(config: &JournalConfig) {
    assert!(
        config.max_chunk_bytes > 0,
        "max_chunk_bytes must be positive"
    );
    assert!(
        config.max_chunk_records > 0,
        "max_chunk_records must be positive"
    );
    assert!(
        !config.flush_interval.is_zero(),
        "flush_interval must be positive"
    );
    assert!(
        config.command_capacity > 0,
        "command_capacity must be positive"
    );
    assert!(config.live_capacity > 0, "live_capacity must be positive");
}

fn estimated_size(record: &JournalRecord) -> usize {
    match &record.payload {
        RecordPayload::MavlinkFrame(frame) => 128 + frame.frame.len(),
        RecordPayload::Diagnostic(event) => 256 + event.message.len() + event.fields.len() * 32,
    }
}

fn chunk_path(directory: &Path, first_sequence: u64, last_sequence: u64) -> PathBuf {
    directory.join(format!("{first_sequence:020}-{last_sequence:020}.lhc"))
}

fn chunk_last_sequence(path: &Path) -> Option<u64> {
    path.file_stem()?.to_str()?.split_once('-')?.1.parse().ok()
}

#[cfg(test)]
mod tests {
    use serde_json::{Map, json};
    use tempfile::TempDir;

    use super::*;
    use crate::records::{DiagnosticLevel, wall_time_ns};

    fn diagnostic(run_id: Uuid, source_sequence: u64) -> DiagnosticEvent {
        let mut fields = Map::new();
        fields.insert("value".to_owned(), json!(42));
        DiagnosticEvent {
            schema_version: 1,
            run_id,
            source: "test".to_owned(),
            source_instance: "test-1".to_owned(),
            source_sequence,
            source_wall_time_ns: wall_time_ns(),
            source_monotonic_ns: Some(10),
            sim_time_ns: Some(20),
            sim_time_quality: None,
            level: DiagnosticLevel::Info,
            category: "test".to_owned(),
            event: "test.recorded".to_owned(),
            message: "recorded".to_owned(),
            correlation_id: None,
            causation_id: None,
            fields,
            related_records: Vec::new(),
        }
    }

    #[tokio::test]
    async fn assigns_sequences_and_reads_flushed_chunks() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let mut config = JournalConfig::for_directory(temp.path(), run_id);
        config.max_chunk_records = 2;
        config.flush_interval = Duration::from_secs(60);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");

        assert_eq!(
            journal
                .append_diagnostic(diagnostic(run_id, 1))
                .await
                .expect("append")
                .sequence,
            1,
        );
        assert_eq!(
            journal
                .append_diagnostic(diagnostic(run_id, 2))
                .await
                .expect("append")
                .sequence,
            2,
        );
        assert_eq!(
            journal
                .append_diagnostic(diagnostic(run_id, 3))
                .await
                .expect("append")
                .sequence,
            3,
        );

        let records = journal.records_after(1).await.expect("read records");
        assert_eq!(
            records
                .iter()
                .map(|record| record.sequence)
                .collect::<Vec<_>>(),
            vec![2, 3]
        );

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn replay_skips_chunks_ending_at_or_before_cursor() {
        let temp = TempDir::new().expect("temp directory");
        let path = chunk_path(temp.path(), 1, 10);
        let mut invalid_chunk = CHUNK_MAGIC.to_vec();
        invalid_chunk.push(0xff);
        fs::write(path, invalid_chunk).await.expect("old chunk");

        let records = read_flushed_after(temp.path(), 10)
            .await
            .expect("old chunk must not be decoded");

        assert!(records.is_empty());
    }

    #[tokio::test]
    async fn replay_remains_complete_during_concurrent_flushes() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let mut config = JournalConfig::for_directory(temp.path(), run_id);
        config.max_chunk_records = 17;
        config.flush_interval = Duration::from_millis(1);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");

        let flushing = journal.clone();
        let flusher = tokio::spawn(async move {
            for _ in 0..100 {
                flushing.flush().await.expect("concurrent flush");
                tokio::task::yield_now().await;
            }
        });

        for source_sequence in 1..=256 {
            journal
                .append_diagnostic(diagnostic(run_id, source_sequence))
                .await
                .expect("append");
            let records = journal.records_after(0).await.expect("replay");
            assert_eq!(records.len(), source_sequence as usize);
            assert_eq!(
                records.last().map(|record| record.sequence),
                Some(source_sequence),
            );
        }

        flusher.await.expect("flusher task");
        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn rejects_diagnostics_from_another_run() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path(), run_id);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");

        let error = journal
            .append_diagnostic(diagnostic(Uuid::new_v4(), 1))
            .await
            .expect_err("wrong run should fail");
        assert!(matches!(error, JournalError::WrongRun));

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn deduplicates_diagnostic_source_sequences() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path(), run_id);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");

        let first = journal
            .append_diagnostic(diagnostic(run_id, 10))
            .await
            .expect("first append");
        let duplicate = journal
            .append_diagnostic(diagnostic(run_id, 10))
            .await
            .expect("duplicate append");

        assert!(first.appended);
        assert!(!duplicate.appended);
        assert_eq!(first.sequence, duplicate.sequence);
        assert_eq!(journal.tail(), 1);

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }
}
