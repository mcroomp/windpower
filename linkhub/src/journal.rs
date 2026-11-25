use std::{
    collections::HashMap,
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
    io::AsyncReadExt,
    sync::{broadcast, mpsc, oneshot},
    task::JoinHandle,
    time,
};
use uuid::Uuid;

use crate::records::{DiagnosticEvent, JournalRecord, RecordPayload, SimClock, SimTimeQuality};

// Chunk file layout: magic, min ingest ns (u64 LE), max ingest ns (u64 LE),
// then the MessagePack `Chunk`. The time header lets range reads skip files
// without decoding them; min/max (not first/last) tolerate wall-clock steps.
const CHUNK_MAGIC: &[u8; 8] = b"LHCHNK02";
const CHUNK_HEADER_LEN: usize = CHUNK_MAGIC.len() + 16;
const CHUNK_VERSION: u16 = 2;

/// Inclusive host-ingest-time window, in wall-clock nanoseconds.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TimeWindow {
    pub since_ns: Option<u64>,
    pub until_ns: Option<u64>,
}

impl TimeWindow {
    fn overlaps(self, min_ns: u64, max_ns: u64) -> bool {
        self.since_ns.is_none_or(|since| max_ns >= since)
            && self.until_ns.is_none_or(|until| min_ns <= until)
    }
}

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
            flush_interval: Duration::from_secs(60),
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

#[derive(Default)]
struct ChunkIndex {
    entries: Vec<ChunkEntry>,
}

struct ChunkEntry {
    path: PathBuf,
    last_sequence: u64,
    min_ns: u64,
    max_ns: u64,
    max_ns_through: u64,
}

impl ChunkIndex {
    async fn load(directory: &Path) -> Result<Self, JournalError> {
        let mut index = Self::default();
        for path in chunk_paths(directory).await? {
            let Some((first, last)) = chunk_sequences(&path) else {
                return Err(JournalError::UnsupportedChunk(path));
            };
            if first <= index.tail() || last < first {
                return Err(JournalError::UnsupportedChunk(path));
            }
            let (min_ns, max_ns) = read_chunk_times(&path).await?;
            index.push(path, last, min_ns, max_ns);
        }
        Ok(index)
    }

    fn tail(&self) -> u64 {
        self.entries.last().map_or(0, |entry| entry.last_sequence)
    }

    fn push(&mut self, path: PathBuf, last_sequence: u64, min_ns: u64, max_ns: u64) {
        // The running maximum stays sorted even when the wall clock steps back.
        let max_ns_through = self
            .entries
            .last()
            .map_or(max_ns, |entry| max_ns.max(entry.max_ns_through));
        self.entries.push(ChunkEntry {
            path,
            last_sequence,
            min_ns,
            max_ns,
            max_ns_through,
        });
    }

    fn paths_after(&self, after: u64, window: TimeWindow) -> Vec<PathBuf> {
        let start = self.entries.partition_point(|entry| {
            entry.last_sequence <= after
                || window
                    .since_ns
                    .is_some_and(|since| entry.max_ns_through < since)
        });
        self.entries[start..]
            .iter()
            .filter(|entry| window.overlaps(entry.min_ns, entry.max_ns))
            .map(|entry| entry.path.clone())
            .collect()
    }
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
    #[error("{0} is not a supported LinkHub journal chunk")]
    UnsupportedChunk(PathBuf),
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
        time_boot_ms: Option<u64>,
        reply: oneshot::Sender<Result<AppendResult, JournalError>>,
    },
    BeginClockEpoch {
        reply: oneshot::Sender<u64>,
    },
    Snapshot {
        after: u64,
        window: TimeWindow,
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
    sim_clock: SimClock,
    flushed: Vec<PathBuf>,
    pending: Vec<JournalRecord>,
}

pub struct JournalRead {
    pub records: Vec<JournalRecord>,
    pub tail_clock: SimClock,
}

#[derive(Clone)]
pub struct JournalHandle {
    run_id: Uuid,
    sender: mpsc::Sender<Command>,
    live: broadcast::Sender<Arc<JournalRecord>>,
    tail: Arc<AtomicU64>,
}

impl JournalHandle {
    pub async fn start(config: JournalConfig) -> Result<(Self, JoinHandle<()>), JournalError> {
        validate_config(&config);
        fs::create_dir_all(&config.directory).await?;
        let index = ChunkIndex::load(&config.directory).await?;
        let (sender, receiver) = mpsc::channel(config.command_capacity);
        let (live, _) = broadcast::channel(config.live_capacity);
        let tail = Arc::new(AtomicU64::new(index.tail()));
        let handle = Self {
            run_id: config.run_id,
            sender,
            live: live.clone(),
            tail: Arc::clone(&tail),
        };
        let task = tokio::spawn(run_journal(config, receiver, live, tail, index));
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
        Ok(self
            .append_record(payload, correlation_id, None)
            .await?
            .sequence)
    }

    pub async fn append_with_time_boot_ms(
        &self,
        payload: RecordPayload,
        correlation_id: Option<Uuid>,
        time_boot_ms: u64,
    ) -> Result<u64, JournalError> {
        Ok(self
            .append_record(payload, correlation_id, Some(time_boot_ms))
            .await?
            .sequence)
    }

    pub async fn begin_clock_epoch(&self) -> Result<u64, JournalError> {
        let (reply, response) = oneshot::channel();
        self.sender
            .send(Command::BeginClockEpoch { reply })
            .await
            .map_err(|_| JournalError::Unavailable)?;
        response.await.map_err(|_| JournalError::Unavailable)
    }

    async fn append_record(
        &self,
        payload: RecordPayload,
        correlation_id: Option<Uuid>,
        time_boot_ms: Option<u64>,
    ) -> Result<AppendResult, JournalError> {
        let (reply, response) = oneshot::channel();
        self.sender
            .send(Command::Append {
                payload,
                correlation_id,
                time_boot_ms,
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
        self.append_record(
            RecordPayload::Diagnostic(Box::new(event)),
            correlation_id,
            None,
        )
        .await
    }

    pub async fn records_after(&self, after: u64) -> Result<Vec<JournalRecord>, JournalError> {
        Ok(self.read_after(after).await?.records)
    }

    pub async fn read_after(&self, after: u64) -> Result<JournalRead, JournalError> {
        self.read_after_bounded(after, usize::MAX).await
    }

    pub async fn read_after_bounded(
        &self,
        after: u64,
        max_records: usize,
    ) -> Result<JournalRead, JournalError> {
        self.read_window_bounded(after, TimeWindow::default(), max_records)
            .await
    }

    /// Like `read_after_bounded`, but skips flushed chunks whose ingest-time
    /// header lies entirely outside `window`. Records inside a returned chunk
    /// are not filtered by time; callers apply exact per-record bounds.
    pub async fn read_window_bounded(
        &self,
        after: u64,
        window: TimeWindow,
        max_records: usize,
    ) -> Result<JournalRead, JournalError> {
        assert!(max_records > 0, "max_records must be positive");
        let snapshot = self.snapshot(after, window).await?;
        let mut records = read_flushed_after(snapshot.flushed, after, max_records).await?;
        let remaining = max_records - records.len();
        records.extend(snapshot.pending.into_iter().take(remaining));
        let tail_clock = records
            .last()
            .map_or(snapshot.sim_clock, |record| record.sim_clock);
        Ok(JournalRead {
            records,
            tail_clock,
        })
    }

    pub async fn checkpoint(&self) -> Result<(u64, SimClock), JournalError> {
        let snapshot = self.snapshot(u64::MAX, TimeWindow::default()).await?;
        Ok((snapshot.tail, snapshot.sim_clock))
    }

    async fn snapshot(
        &self,
        after: u64,
        window: TimeWindow,
    ) -> Result<JournalSnapshot, JournalError> {
        let (reply, response) = oneshot::channel();
        self.sender
            .send(Command::Snapshot {
                after,
                window,
                reply,
            })
            .await
            .map_err(|_| JournalError::Unavailable)?;
        response.await.map_err(|_| JournalError::Unavailable)
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
    mut index: ChunkIndex,
) {
    let mut pending = Vec::with_capacity(config.max_chunk_records);
    let mut pending_bytes = 0_usize;
    let mut diagnostic_sequences: HashMap<(String, u64), u64> = HashMap::new();
    let mut sim_clock = SimClock::default();
    let mut ticker = time::interval(config.flush_interval);
    ticker.set_missed_tick_behavior(time::MissedTickBehavior::Delay);
    ticker.tick().await;

    loop {
        tokio::select! {
            biased;
            command = receiver.recv() => {
                let Some(command) = command else {
                    let _ = flush_pending(&config, &mut pending, &mut index).await;
                    return;
                };
                match command {
                    Command::Append {
                        payload,
                        correlation_id,
                        time_boot_ms,
                        reply,
                    } => {
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
                        let record_clock = match time_boot_ms {
                            Some(observed) if sim_clock.time_boot_ms.is_none_or(|last| observed >= last) => {
                                sim_clock.time_boot_ms = Some(observed);
                                SimClock {
                                    quality: Some(SimTimeQuality::Exact),
                                    ..sim_clock
                                }
                            }
                            _ if sim_clock.time_boot_ms.is_some() => SimClock {
                                quality: Some(SimTimeQuality::LastObserved),
                                ..sim_clock
                            },
                            _ => sim_clock,
                        };
                        sim_clock = record_clock;
                        let record = JournalRecord::new(
                            sequence,
                            correlation_id,
                            record_clock,
                            payload,
                        );
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
                            if let Err(error) = flush_pending(&config, &mut pending, &mut index).await {
                                tracing::error!(%error, "journal chunk flush failed");
                            }
                            pending_bytes = 0;
                        }
                    }
                    Command::BeginClockEpoch { reply } => {
                        sim_clock = SimClock {
                            epoch: sim_clock.epoch + 1,
                            ..SimClock::default()
                        };
                        let _ = reply.send(sim_clock.epoch);
                    }
                    Command::Snapshot { after, window, reply } => {
                        let pending_start = pending.partition_point(|record| record.sequence <= after);
                        let _ = reply.send(JournalSnapshot {
                            tail: tail.load(Ordering::Acquire),
                            sim_clock,
                            flushed: index.paths_after(after, window),
                            pending: pending[pending_start..].to_vec(),
                        });
                    }
                    Command::Flush { reply } => {
                        let result = flush_pending(&config, &mut pending, &mut index).await;
                        if result.is_ok() {
                            pending_bytes = 0;
                        }
                        let _ = reply.send(result);
                    }
                    Command::Shutdown { reply } => {
                        let result = flush_pending(&config, &mut pending, &mut index).await;
                        let _ = reply.send(result);
                        return;
                    }
                }
            }
            _ = ticker.tick() => {
                if let Err(error) = flush_pending(&config, &mut pending, &mut index).await {
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
    index: &mut ChunkIndex,
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
    let (min_ns, max_ns) =
        chunk
            .records
            .iter()
            .fold((u64::MAX, u64::MIN), |(min_ns, max_ns), record| {
                (
                    min_ns.min(record.ingest_time_ns),
                    max_ns.max(record.ingest_time_ns),
                )
            });
    let mut bytes = Vec::with_capacity(CHUNK_HEADER_LEN + encoded.len());
    bytes.extend_from_slice(CHUNK_MAGIC);
    bytes.extend_from_slice(&min_ns.to_le_bytes());
    bytes.extend_from_slice(&max_ns.to_le_bytes());
    bytes.extend_from_slice(&encoded);
    let result = async {
        fs::write(&temporary_path, bytes).await?;
        fs::rename(temporary_path, &final_path).await
    }
    .await;
    match result {
        Ok(()) => {
            index.push(final_path, last_sequence, min_ns, max_ns);
            Ok(())
        }
        Err(error) => {
            pending.extend(chunk.records);
            Err(JournalError::Io(error))
        }
    }
}

async fn chunk_paths(directory: &Path) -> Result<Vec<PathBuf>, JournalError> {
    let mut entries = fs::read_dir(directory).await?;
    let mut paths = Vec::new();
    while let Some(entry) = entries.next_entry().await? {
        let path = entry.path();
        if path.extension().is_some_and(|extension| extension == "lhc") {
            paths.push(path);
        }
    }
    paths.sort_unstable();
    Ok(paths)
}

async fn read_flushed_after(
    paths: impl IntoIterator<Item = PathBuf>,
    after: u64,
    max_records: usize,
) -> Result<Vec<JournalRecord>, JournalError> {
    let mut records = Vec::new();
    for path in paths {
        let bytes = fs::read(&path).await?;
        if bytes.len() < CHUNK_HEADER_LEN || !bytes.starts_with(CHUNK_MAGIC) {
            return Err(JournalError::UnsupportedChunk(path));
        }
        let chunk: Chunk = rmp_serde::from_slice(&bytes[CHUNK_HEADER_LEN..])?;
        if chunk.last_sequence <= after {
            continue;
        }
        records.extend(
            chunk
                .records
                .into_iter()
                .filter(|record| record.sequence > after)
                .take(max_records - records.len()),
        );
        if records.len() == max_records {
            break;
        }
    }
    Ok(records)
}

async fn read_chunk_times(path: &Path) -> Result<(u64, u64), JournalError> {
    let mut header = [0_u8; CHUNK_HEADER_LEN];
    let mut file = fs::File::open(path).await?;
    if file.read_exact(&mut header).await.is_err() || !header.starts_with(CHUNK_MAGIC) {
        return Err(JournalError::UnsupportedChunk(path.to_owned()));
    }
    let time = |offset: usize| {
        u64::from_le_bytes(
            header[offset..offset + 8]
                .try_into()
                .expect("header slice is eight bytes"),
        )
    };
    Ok((time(CHUNK_MAGIC.len()), time(CHUNK_MAGIC.len() + 8)))
}

pub async fn read_persisted(
    directory: &Path,
    after: u64,
    through: Option<u64>,
) -> Result<Vec<JournalRecord>, JournalError> {
    let paths = chunk_paths(directory).await?.into_iter().filter(|path| {
        chunk_sequences(path).is_none_or(|(_, last_sequence)| last_sequence > after)
    });
    let mut records = read_flushed_after(paths, after, usize::MAX).await?;
    if let Some(through) = through {
        records.retain(|record| record.sequence <= through);
    }
    records.sort_unstable_by_key(|record| record.sequence);
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

fn chunk_sequences(path: &Path) -> Option<(u64, u64)> {
    let (first, last) = path.file_stem()?.to_str()?.split_once('-')?;
    Some((first.parse().ok()?, last.parse().ok()?))
}

#[cfg(test)]
mod tests {
    use serde_json::{Map, json};
    use tempfile::TempDir;

    use super::*;
    use crate::records::{DiagnosticLevel, Direction, MavlinkFrame, SimTimeQuality, wall_time_ns};

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

    fn frame() -> MavlinkFrame {
        MavlinkFrame {
            link_id: "test".to_owned(),
            direction: Direction::Rx,
            protocol_version: 2,
            sequence: 1,
            system_id: 1,
            component_id: 1,
            message_id: 30,
            message_name: "ATTITUDE".to_owned(),
            fields: Map::new(),
            signed: false,
            frame: vec![1, 2, 3],
        }
    }

    #[tokio::test]
    async fn stamps_records_with_epoch_clock_watermarks() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path(), run_id);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");

        journal
            .append_diagnostic(diagnostic(run_id, 1))
            .await
            .expect("startup diagnostic");
        assert_eq!(journal.begin_clock_epoch().await.expect("epoch"), 1);
        journal
            .append_with_time_boot_ms(RecordPayload::MavlinkFrame(frame()), None, 1_250)
            .await
            .expect("timestamped frame");
        journal
            .append_diagnostic(diagnostic(run_id, 2))
            .await
            .expect("diagnostic");
        journal
            .append_with_time_boot_ms(RecordPayload::MavlinkFrame(frame()), None, 1_200)
            .await
            .expect("out-of-order frame");
        assert_eq!(journal.begin_clock_epoch().await.expect("epoch"), 2);
        journal
            .append_diagnostic(diagnostic(run_id, 3))
            .await
            .expect("new epoch diagnostic");

        let records = journal.records_after(0).await.expect("records");
        assert_eq!(records[0].sim_clock, SimClock::default());
        assert_eq!(
            records[1].sim_clock,
            SimClock {
                epoch: 1,
                time_boot_ms: Some(1_250),
                quality: Some(SimTimeQuality::Exact),
            }
        );
        assert_eq!(
            records[2].sim_clock,
            SimClock {
                epoch: 1,
                time_boot_ms: Some(1_250),
                quality: Some(SimTimeQuality::LastObserved),
            }
        );
        assert_eq!(records[3].sim_clock, records[2].sim_clock);
        assert_eq!(
            records[4].sim_clock,
            SimClock {
                epoch: 2,
                time_boot_ms: None,
                quality: None,
            }
        );

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
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
    async fn bounded_reads_advance_without_materializing_the_remaining_tail() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let mut config = JournalConfig::for_directory(temp.path(), run_id);
        config.max_chunk_records = 2;
        config.flush_interval = Duration::from_secs(60);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");

        for source_sequence in 1..=5 {
            journal
                .append_diagnostic(diagnostic(run_id, source_sequence))
                .await
                .expect("append");
        }

        let first = journal
            .read_after_bounded(0, 3)
            .await
            .expect("first bounded read");
        let second = journal
            .read_after_bounded(3, 3)
            .await
            .expect("second bounded read");

        assert_eq!(
            first
                .records
                .iter()
                .map(|record| record.sequence)
                .collect::<Vec<_>>(),
            vec![1, 2, 3]
        );
        assert_eq!(
            second
                .records
                .iter()
                .map(|record| record.sequence)
                .collect::<Vec<_>>(),
            vec![4, 5]
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

        let records = read_persisted(temp.path(), 10, None)
            .await
            .expect("old chunk must not be decoded");

        assert!(records.is_empty());
    }

    #[tokio::test]
    async fn indexed_window_reads_do_not_open_old_chunks() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path(), run_id);
        let mut index = ChunkIndex::default();
        // Chunk 3 was recorded after a backward wall-clock step.
        for (sequence, ingest_time_ns) in [(1, 10), (2, 30), (3, 20)] {
            let mut record = JournalRecord::new(
                sequence,
                None,
                SimClock::default(),
                RecordPayload::MavlinkFrame(frame()),
            );
            record.ingest_time_ns = ingest_time_ns;
            flush_pending(&config, &mut vec![record], &mut index)
                .await
                .expect("seed chunk");
        }

        let (journal, task) = JournalHandle::start(config).await.expect("reopen journal");
        assert_eq!(journal.tail(), 3);
        let sequence = journal
            .append(RecordPayload::MavlinkFrame(frame()), None)
            .await
            .expect("append after indexed tail");
        assert_eq!(sequence, 4);
        journal.flush().await.expect("flush");

        // Removing an already indexed, out-of-window chunk detects even a
        // header read, not just an attempt to decode its payload.
        fs::remove_file(chunk_path(temp.path(), 1, 1))
            .await
            .expect("remove old chunk");
        fs::write(chunk_path(temp.path(), 5, 5), b"unindexed")
            .await
            .expect("place unindexed file to detect directory rescans");

        let window = TimeWindow {
            since_ns: Some(20),
            until_ns: None,
        };
        let read = journal
            .read_window_bounded(0, window, 4)
            .await
            .expect("windowed read");
        assert_eq!(
            read.records
                .iter()
                .map(|record| record.sequence)
                .collect::<Vec<_>>(),
            vec![2, 3, 4]
        );

        let read = journal
            .read_window_bounded(
                0,
                TimeWindow {
                    since_ns: Some(25),
                    until_ns: Some(30),
                },
                4,
            )
            .await
            .expect("read across backward clock step");
        assert_eq!(read.records.len(), 1);
        assert_eq!(read.records[0].sequence, 2);

        let error = journal
            .records_after(0)
            .await
            .expect_err("an in-range missing chunk must still fail");
        assert!(matches!(error, JournalError::Io(_)));

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn rejects_chunks_without_current_magic() {
        let temp = TempDir::new().expect("temp directory");
        fs::write(chunk_path(temp.path(), 1, 1), b"LHCHNK01\x80")
            .await
            .expect("legacy chunk");
        let error = read_persisted(temp.path(), 0, None)
            .await
            .expect_err("legacy chunk");
        assert!(matches!(error, JournalError::UnsupportedChunk(_)));
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
            assert_eq!(
                records
                    .iter()
                    .map(|record| record.sequence)
                    .collect::<Vec<_>>(),
                (1..=source_sequence).collect::<Vec<_>>(),
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
