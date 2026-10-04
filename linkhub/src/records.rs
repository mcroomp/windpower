use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use uuid::Uuid;

pub const SCHEMA_VERSION: u16 = 2;

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Direction {
    Rx,
    Tx,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DiagnosticLevel {
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Critical,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SimTimeQuality {
    Exact,
    LastObserved,
    Estimated,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct SimClock {
    pub epoch: u64,
    pub time_boot_ms: Option<u64>,
    pub quality: Option<SimTimeQuality>,
}

impl Default for SimClock {
    fn default() -> Self {
        Self {
            epoch: 0,
            time_boot_ms: None,
            quality: None,
        }
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct DiagnosticEvent {
    #[serde(default = "schema_version")]
    pub schema_version: u16,
    pub run_id: Uuid,
    pub source: String,
    pub source_instance: String,
    pub source_sequence: u64,
    pub source_wall_time_ns: u64,
    pub source_monotonic_ns: Option<u64>,
    pub sim_time_ns: Option<u64>,
    pub sim_time_quality: Option<SimTimeQuality>,
    pub level: DiagnosticLevel,
    pub category: String,
    pub event: String,
    pub message: String,
    pub correlation_id: Option<Uuid>,
    pub causation_id: Option<Uuid>,
    #[serde(default)]
    pub fields: Map<String, Value>,
    #[serde(default)]
    pub related_records: Vec<u64>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct MavlinkFrame {
    pub link_id: String,
    pub direction: Direction,
    pub protocol_version: u8,
    pub sequence: u8,
    pub system_id: u8,
    pub component_id: u8,
    pub message_id: u32,
    pub message_name: String,
    #[serde(default)]
    pub fields: Map<String, Value>,
    pub signed: bool,
    #[serde(with = "serde_bytes")]
    pub frame: Vec<u8>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "type", content = "data", rename_all = "snake_case")]
pub enum RecordPayload {
    MavlinkFrame(MavlinkFrame),
    Diagnostic(Box<DiagnosticEvent>),
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct JournalRecord {
    pub schema_version: u16,
    pub sequence: u64,
    pub ingest_time_ns: u64,
    pub correlation_id: Option<Uuid>,
    pub sim_clock: SimClock,
    pub payload: RecordPayload,
}

impl JournalRecord {
    #[must_use]
    pub fn new(
        sequence: u64,
        correlation_id: Option<Uuid>,
        sim_clock: SimClock,
        payload: RecordPayload,
    ) -> Self {
        Self {
            schema_version: SCHEMA_VERSION,
            sequence,
            ingest_time_ns: wall_time_ns(),
            correlation_id,
            sim_clock,
            payload,
        }
    }
}

const fn schema_version() -> u16 {
    SCHEMA_VERSION
}

#[must_use]
pub fn wall_time_ns() -> u64 {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    u64::try_from(nanos).unwrap_or(u64::MAX)
}
