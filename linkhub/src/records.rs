use std::{
    hash::{Hash, Hasher},
    time::{SystemTime, UNIX_EPOCH},
};

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

#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
pub struct SimClock {
    pub epoch: u64,
    pub time_boot_ms: Option<u64>,
    pub quality: Option<SimTimeQuality>,
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

#[derive(Clone, Debug, Eq)]
pub struct MavlinkStateKey {
    link_id: String,
    direction: Direction,
    system_id: u8,
    component_id: u8,
    message_id: u32,
    discriminator: Option<String>,
}

impl PartialEq for MavlinkStateKey {
    fn eq(&self, other: &Self) -> bool {
        self.link_id == other.link_id
            && self.direction == other.direction
            && self.system_id == other.system_id
            && self.component_id == other.component_id
            && self.message_id == other.message_id
            && self.discriminator == other.discriminator
    }
}

impl Hash for MavlinkStateKey {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.link_id.hash(state);
        self.direction.hash(state);
        self.system_id.hash(state);
        self.component_id.hash(state);
        self.message_id.hash(state);
        self.discriminator.hash(state);
    }
}

impl Hash for Direction {
    fn hash<H: Hasher>(&self, state: &mut H) {
        (*self as u8).hash(state);
    }
}

impl MavlinkFrame {
    #[must_use]
    pub fn state_key(&self) -> Option<MavlinkStateKey> {
        let discriminator_field = match self.message_name.as_str() {
            "NAMED_VALUE_FLOAT" | "NAMED_VALUE_INT" | "DEBUG_VECT" => Some("name"),
            "PID_TUNING" => Some("axis"),
            "BATTERY_STATUS" => Some("id"),
            _ if is_snapshot_message(&self.message_name) => None,
            _ => return None,
        };
        let discriminator = discriminator_field
            .and_then(|field| self.fields.get(field))
            .map(Value::to_string);
        if discriminator_field.is_some() && discriminator.is_none() {
            return None;
        }
        Some(MavlinkStateKey {
            link_id: self.link_id.clone(),
            direction: self.direction,
            system_id: self.system_id,
            component_id: self.component_id,
            message_id: self.message_id,
            discriminator,
        })
    }
}

fn is_snapshot_message(message_name: &str) -> bool {
    matches!(
        message_name,
        "HEARTBEAT"
            | "SYS_STATUS"
            | "SYSTEM_TIME"
            | "GPS_RAW_INT"
            | "RAW_IMU"
            | "SCALED_IMU"
            | "SCALED_IMU2"
            | "SCALED_IMU3"
            | "SCALED_PRESSURE"
            | "SCALED_PRESSURE2"
            | "SCALED_PRESSURE3"
            | "ATTITUDE"
            | "ATTITUDE_QUATERNION"
            | "LOCAL_POSITION_NED"
            | "GLOBAL_POSITION_INT"
            | "RC_CHANNELS"
            | "RC_CHANNELS_RAW"
            | "SERVO_OUTPUT_RAW"
            | "VFR_HUD"
            | "HIGHRES_IMU"
            | "ATTITUDE_TARGET"
            | "POSITION_TARGET_LOCAL_NED"
            | "POSITION_TARGET_GLOBAL_INT"
            | "ESTIMATOR_STATUS"
            | "VIBRATION"
            | "HOME_POSITION"
            | "EXTENDED_SYS_STATE"
            | "ESC_TELEMETRY_1_TO_4"
            | "ESC_TELEMETRY_5_TO_8"
            | "ESC_TELEMETRY_9_TO_12"
    )
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
