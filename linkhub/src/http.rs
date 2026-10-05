use std::{collections::HashMap, path::PathBuf, sync::Arc, time::Duration};

use axum::{
    Json, Router,
    body::{Body, Bytes},
    extract::{Path, Query, State},
    http::{
        HeaderValue, StatusCode,
        header::{CACHE_CONTROL, CONTENT_TYPE},
    },
    response::{IntoResponse, Response},
    routing::get,
};
use base64::{Engine as _, engine::general_purpose::STANDARD as BASE64};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use tokio::time::Instant;
use tower_http::{services::ServeDir, set_header::SetResponseHeaderLayer};
use uuid::Uuid;

use crate::{
    journal::{JournalError, JournalHandle, TimeWindow},
    mavlink::{LinkError, LinkStatus, MavlinkLinkHandle},
    motor::{MotorDirection, MotorError, MotorHandle},
    operations::{MavlinkOperations, OperationError},
    protocol,
    records::{
        DiagnosticEvent, DiagnosticLevel, JournalRecord, RecordPayload, SimClock, wall_time_ns,
    },
};

#[derive(Clone)]
pub struct AppState {
    journal: JournalHandle,
    link: Option<MavlinkLinkHandle>,
    operations: Option<MavlinkOperations>,
    motor: Option<MotorHandle>,
}

fn collapse_state_records(records: Vec<JournalRecord>) -> Vec<JournalRecord> {
    let mut collapsed: Vec<Option<JournalRecord>> = Vec::with_capacity(records.len());
    let mut state_positions = HashMap::new();
    for record in records {
        let key = match &record.payload {
            RecordPayload::MavlinkFrame(frame) => frame.state_key(),
            RecordPayload::Diagnostic(_) => None,
        };
        if let Some(key) = key
            && let Some(previous) = state_positions.insert(key, collapsed.len())
        {
            collapsed[previous] = None;
        }
        collapsed.push(Some(record));
    }
    collapsed.into_iter().flatten().collect()
}

#[derive(Debug, thiserror::Error)]
enum ApiError {
    #[error("{0}")]
    BadRequest(String),
    #[error("{0}")]
    NotFound(String),
    #[error(transparent)]
    Journal(#[from] JournalError),
    #[error(transparent)]
    Link(#[from] LinkError),
    #[error(transparent)]
    Operation(#[from] OperationError),
    #[error(transparent)]
    Motor(#[from] MotorError),
    /// The caller's `expected_generation` no longer matches LinkHub's live
    /// generation: the service restarted, the serial link reconnected, or
    /// the vehicle rebooted since the caller captured its baseline. A
    /// multi-step sequence polling for a state transition (e.g. confirming
    /// arm/disarm) should abort rather than keep trusting this link.
    #[error("LinkHub generation changed ({expected} -> {current})")]
    GenerationChanged {
        expected: String,
        current: String,
        cursor: String,
    },
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        let (status, code) = match self {
            Self::BadRequest(_) => (StatusCode::BAD_REQUEST, "invalid_request"),
            Self::NotFound(_) => (StatusCode::NOT_FOUND, "not_found"),
            Self::Journal(_) => (StatusCode::SERVICE_UNAVAILABLE, "journal_unavailable"),
            Self::Link(_) => (StatusCode::SERVICE_UNAVAILABLE, "link_unavailable"),
            Self::Operation(OperationError::Invalid(_)) => {
                (StatusCode::BAD_REQUEST, "invalid_operation")
            }
            Self::Operation(OperationError::NotFound(_)) => (StatusCode::NOT_FOUND, "not_found"),
            Self::Operation(OperationError::Timeout) => {
                (StatusCode::GATEWAY_TIMEOUT, "operation_timeout")
            }
            Self::Operation(OperationError::MavFtp(_))
            | Self::Operation(OperationError::DataFlash(_)) => {
                (StatusCode::BAD_GATEWAY, "mavlink_transfer_failed")
            }
            Self::Operation(_) => (StatusCode::SERVICE_UNAVAILABLE, "operation_failed"),
            Self::Motor(MotorError::InvalidSpeed | MotorError::InvalidTimeout { .. }) => {
                (StatusCode::BAD_REQUEST, "invalid_motor_command")
            }
            Self::Motor(_) => (StatusCode::SERVICE_UNAVAILABLE, "motor_unavailable"),
            Self::GenerationChanged { .. } => (StatusCode::CONFLICT, "generation_changed"),
        };
        let mut body = json!({
            "error": code,
            "message": self.to_string(),
        });
        if let Self::GenerationChanged {
            expected,
            current,
            cursor,
        } = &self
        {
            let object = body.as_object_mut().expect("body is an object");
            object.insert(
                "expected_generation".to_owned(),
                Value::String(expected.clone()),
            );
            object.insert(
                "current_generation".to_owned(),
                Value::String(current.clone()),
            );
            object.insert("cursor".to_owned(), Value::String(cursor.clone()));
        }
        (status, Json(body)).into_response()
    }
}

pub fn router(journal: JournalHandle, link: Option<MavlinkLinkHandle>) -> Router {
    router_with_motor(journal, link, None)
}

pub fn router_with_motor(
    journal: JournalHandle,
    link: Option<MavlinkLinkHandle>,
    motor: Option<MotorHandle>,
) -> Router {
    router_with_motor_and_static(journal, link, motor, None, false)
}

pub fn router_with_motor_and_static(
    journal: JournalHandle,
    link: Option<MavlinkLinkHandle>,
    motor: Option<MotorHandle>,
    static_dir: Option<PathBuf>,
    no_cache: bool,
) -> Router {
    let operations = link.clone().map(MavlinkOperations::new);
    let router = Router::new()
        .route("/health/live", get(live))
        .route("/health/ready", get(ready))
        .route("/v1/status", get(status))
        .route("/v1/schema", get(protocol_schema))
        .route("/v1/records", get(records))
        .route("/v1/journal/flush", axum::routing::post(flush_journal))
        .route(
            "/v1/diagnostics/events",
            get(diagnostic_events).post(ingest_diagnostics),
        )
        .route(
            "/v1/mavlink/frames",
            get(mavlink_frames).post(send_mavlink_frame),
        )
        .route(
            "/v1/mavlink/messages",
            get(mavlink_messages).post(send_mavlink_message),
        )
        .route("/v1/mavlink/commands", axum::routing::post(execute_command))
        .route(
            "/v1/mavlink/message-requests",
            axum::routing::post(request_message),
        )
        .route("/v1/mavlink/version", get(autopilot_version))
        .route("/v1/mavlink/capabilities", get(capabilities))
        .route("/v1/mavlink/components", get(components))
        .route(
            "/v1/mavlink/parameters",
            get(list_parameters).put(set_parameters),
        )
        .route(
            "/v1/mavlink/parameters/{name}",
            get(get_parameter).put(set_parameter),
        )
        .route(
            "/v1/mavlink/message-rates",
            axum::routing::put(set_message_rates),
        )
        .route(
            "/v1/mavlink/message-rates/{message}",
            get(get_message_interval),
        )
        .route(
            "/v1/mavlink/files",
            get(files).put(upload_file).delete(remove_file),
        )
        .route(
            "/v1/mavlink/directories",
            axum::routing::post(create_directory),
        )
        .route("/v1/mavlink/logs", get(list_logs))
        .route("/v1/mavlink/logs/{id}", get(download_log))
        .route("/v1/motor", get(motor_status).put(set_motor))
        .route("/v1/motor/stop", axum::routing::post(stop_motor))
        .route("/v1/motor/reconnect", axum::routing::post(reconnect_motor))
        .route("/v1/mavlink/status", get(mavlink_status))
        .with_state(Arc::new(AppState {
            journal,
            link,
            operations,
            motor,
        }));
    let router = match static_dir {
        Some(directory) => {
            router.fallback_service(ServeDir::new(directory).append_index_html_on_directories(true))
        }
        None => router,
    };
    if !no_cache {
        return router;
    }
    router.layer(SetResponseHeaderLayer::overriding(
        CACHE_CONTROL,
        HeaderValue::from_static("no-store"),
    ))
}

async fn live() -> Json<Value> {
    Json(json!({"status": "live"}))
}

async fn ready(State(state): State<Arc<AppState>>) -> Response {
    let Some(link) = &state.link else {
        return (
            StatusCode::SERVICE_UNAVAILABLE,
            Json(json!({"status": "not_ready", "reason": "mavlink_not_configured"})),
        )
            .into_response();
    };
    let status = link.status();
    if status.connected && status.ready {
        Json(json!({"status": "ready"})).into_response()
    } else {
        (
            StatusCode::SERVICE_UNAVAILABLE,
            Json(json!({
                "status": "not_ready",
                "reason": status.error.unwrap_or_else(|| "awaiting_heartbeat".to_owned()),
            })),
        )
            .into_response()
    }
}

async fn status(State(state): State<Arc<AppState>>) -> Json<Value> {
    Json(json!({
        "service": "linkhub",
        "api_version": 1,
        "run_id": state.journal.run_id(),
        "cursor": format_cursor(state.journal.tail()),
        "mavlink": state.link.as_ref().map(MavlinkLinkHandle::status),
    }))
}

async fn protocol_schema() -> Json<Value> {
    Json(protocol::json_schema())
}

async fn flush_journal(State(state): State<Arc<AppState>>) -> Result<Json<Value>, ApiError> {
    state.journal.flush().await?;
    Ok(Json(json!({
        "cursor": format_cursor(state.journal.tail()),
    })))
}

/// LinkHub's opaque generation token: changes whenever the service restarts
/// (new `run_id`) or the MAVLink link is replaced (new `clock_epoch` from a
/// serial reconnect or vehicle reboot). Callers polling for a state
/// transition across multiple requests should discard that state, or abort
/// outright, when this token changes mid-sequence.
fn generation_token(run_id: Uuid, clock_epoch: u64) -> String {
    format!("v1:{run_id}:{clock_epoch}")
}

async fn mavlink_status(State(state): State<Arc<AppState>>) -> Result<Json<Value>, ApiError> {
    let status = state.link.as_ref().map_or_else(
        || LinkStatus {
            error: Some("MAVLink is not configured".to_owned()),
            ..LinkStatus::default()
        },
        MavlinkLinkHandle::status,
    );
    let (cursor, sim_clock) = state.journal.checkpoint().await?;
    let mut value = serde_json::to_value(status).expect("LinkStatus is serializable");
    let object = value
        .as_object_mut()
        .expect("LinkStatus serializes as an object");
    object.insert("cursor".to_owned(), Value::String(format_cursor(cursor)));
    object.insert(
        "sim_clock".to_owned(),
        serde_json::to_value(sim_clock).expect("SimClock is serializable"),
    );
    let clock_epoch = object["clock_epoch"].as_u64().unwrap_or_default();
    object.insert(
        "generation".to_owned(),
        Value::String(generation_token(state.journal.run_id(), clock_epoch)),
    );
    Ok(Json(value))
}

#[derive(Debug, Default, Deserialize)]
struct RecordQuery {
    after: Option<String>,
    classes: Option<String>,
    source: Option<String>,
    event: Option<String>,
    level: Option<DiagnosticLevel>,
    direction: Option<String>,
    message_ids: Option<String>,
    messages: Option<String>,
    #[serde(default)]
    wait_ms: u64,
    limit: Option<usize>,
}

const DEFAULT_MESSAGE_BATCH_LIMIT: usize = 1_000;
const MAX_MESSAGE_BATCH_LIMIT: usize = 10_000;
const MAX_JOURNAL_SCAN_RECORDS: usize = 10_000;
const MAX_MESSAGE_WAIT_MS: u64 = 30_000;
const MAX_MESSAGE_LAG_MS: u64 = 60_000;

#[derive(Debug, Default, Deserialize)]
struct MessageQuery {
    after: Option<String>,
    direction: Option<String>,
    message_ids: Option<String>,
    messages: Option<String>,
    #[serde(default)]
    wait_ms: u64,
    limit: Option<usize>,
    #[serde(default)]
    collapse: bool,
    max_lag_ms: Option<u64>,
    /// Generation token the caller captured before starting a multi-step
    /// sequence (e.g. confirming arm/disarm). If LinkHub's live generation
    /// no longer matches, the request fails with 409 `generation_changed`
    /// instead of returning a batch, so the caller can abort immediately
    /// rather than keep polling a link that is no longer trustworthy.
    expected_generation: Option<String>,
    /// Range-query bounds. Any of these selects a one-shot range scan.
    since_ns: Option<u64>,
    until_ns: Option<u64>,
    last_ms: Option<u64>,
    through: Option<String>,
    /// Comma-separated `field=value` pairs; values parse as JSON, else string.
    eq: Option<String>,
    /// Case-insensitive substring over the message name and decoded fields.
    contains: Option<String>,
}

impl MessageQuery {
    fn is_range(&self) -> bool {
        self.since_ns.is_some()
            || self.until_ns.is_some()
            || self.last_ms.is_some()
            || self.through.is_some()
    }
}

#[derive(Debug, Serialize)]
struct MessageBatch {
    records: Vec<Value>,
    next_cursor: String,
    next_clock: SimClock,
}

#[derive(Debug, Serialize)]
struct RecordBatch {
    records: Vec<Value>,
    next_cursor: String,
    next_clock: SimClock,
}

#[derive(Clone, Debug, Default)]
struct RecordFilter {
    diagnostics: bool,
    mavlink: bool,
    source: Option<String>,
    event: Option<String>,
    level: Option<DiagnosticLevel>,
    direction: Option<String>,
    message_ids: Option<Vec<u32>>,
    messages: Option<Vec<String>>,
    equal: Vec<(String, Value)>,
    contains: Option<String>,
}

impl RecordFilter {
    fn from_query(query: &RecordQuery) -> Result<Self, ApiError> {
        let mut filter = Self {
            diagnostics: true,
            mavlink: true,
            source: query.source.clone(),
            event: query.event.clone(),
            level: query.level,
            direction: query.direction.clone(),
            message_ids: None,
            messages: None,
            equal: Vec::new(),
            contains: None,
        };
        if let Some(classes) = &query.classes {
            filter.diagnostics = false;
            filter.mavlink = false;
            for class in classes.split(',').map(str::trim) {
                match class {
                    "diagnostic" | "diagnostic.event" => filter.diagnostics = true,
                    "mavlink" | "mavlink.frame" | "mavlink.rx" | "mavlink.tx" => {
                        filter.mavlink = true;
                    }

                    "" => {}
                    other => {
                        return Err(ApiError::BadRequest(format!(
                            "unknown record class {other:?}"
                        )));
                    }
                }
            }
        }
        if let Some(direction) = &filter.direction
            && direction != "rx"
            && direction != "tx"
        {
            return Err(ApiError::BadRequest(
                "direction must be 'rx' or 'tx'".to_owned(),
            ));
        }
        if let Some(ids) = &query.message_ids {
            filter.message_ids = Some(
                ids.split(',')
                    .filter(|value| !value.is_empty())
                    .map(|value| {
                        value.trim().parse::<u32>().map_err(|_| {
                            ApiError::BadRequest(format!("invalid MAVLink message ID {value:?}"))
                        })
                    })
                    .collect::<Result<Vec<_>, _>>()?,
            );
        }
        if let Some(names) = &query.messages {
            filter.messages = Some(
                names
                    .split(',')
                    .map(str::trim)
                    .filter(|name| !name.is_empty())
                    .map(str::to_uppercase)
                    .collect(),
            );
        }
        Ok(filter)
    }

    fn from_message_query(query: &MessageQuery) -> Result<Self, ApiError> {
        let mut filter = Self::from_query(&RecordQuery {
            after: None,
            classes: Some("mavlink".to_owned()),
            source: None,
            event: None,
            level: None,
            direction: query.direction.clone(),
            message_ids: query.message_ids.clone(),
            messages: query.messages.clone(),
            wait_ms: 0,
            limit: None,
        })?;
        if let Some(pairs) = &query.eq {
            for pair in pairs
                .split(',')
                .map(str::trim)
                .filter(|pair| !pair.is_empty())
            {
                let Some((field, expected)) = pair.split_once('=') else {
                    return Err(ApiError::BadRequest(format!(
                        "eq expects field=value, got {pair:?}"
                    )));
                };
                let expected = serde_json::from_str(expected)
                    .unwrap_or_else(|_| Value::String(expected.to_owned()));
                filter.equal.push((field.trim().to_owned(), expected));
            }
        }
        filter.contains = query
            .contains
            .as_ref()
            .filter(|needle| !needle.is_empty())
            .map(|needle| needle.to_lowercase());
        Ok(filter)
    }

    fn matches(&self, record: &JournalRecord) -> bool {
        match &record.payload {
            RecordPayload::Diagnostic(event) => {
                self.diagnostics
                    && self
                        .source
                        .as_ref()
                        .is_none_or(|source| source == &event.source)
                    && self.event.as_ref().is_none_or(|name| name == &event.event)
                    && self.level.is_none_or(|level| level == event.level)
            }
            RecordPayload::MavlinkFrame(frame) => {
                self.mavlink
                    && self.direction.as_ref().is_none_or(|direction| {
                        direction
                            == match frame.direction {
                                crate::records::Direction::Rx => "rx",
                                crate::records::Direction::Tx => "tx",
                            }
                    })
                    && self
                        .message_ids
                        .as_ref()
                        .is_none_or(|ids| ids.contains(&frame.message_id))
                    && self
                        .messages
                        .as_ref()
                        .is_none_or(|names| names.contains(&frame.message_name))
                    && self
                        .equal
                        .iter()
                        .all(|(field, expected)| frame.fields.get(field) == Some(expected))
                    && self.contains.as_ref().is_none_or(|needle| {
                        frame.message_name.to_lowercase().contains(needle)
                            || Value::Object(frame.fields.clone())
                                .to_string()
                                .to_lowercase()
                                .contains(needle)
                    })
            }
        }
    }
}

async fn records(
    State(state): State<Arc<AppState>>,
    Query(query): Query<RecordQuery>,
) -> Result<Json<RecordBatch>, ApiError> {
    read_record_batch(state, query, None).await
}

async fn diagnostic_events(
    State(state): State<Arc<AppState>>,
    Query(query): Query<RecordQuery>,
) -> Result<Json<RecordBatch>, ApiError> {
    read_record_batch(state, query, Some("diagnostic.event")).await
}

async fn mavlink_frames(
    State(state): State<Arc<AppState>>,
    Query(query): Query<RecordQuery>,
) -> Result<Json<RecordBatch>, ApiError> {
    read_record_batch(state, query, Some("mavlink.frame")).await
}

/// Fail fast if the caller's `expected_generation` baseline no longer
/// matches LinkHub's live generation. Only enforced when the caller actually
/// passes a baseline and a MAVLink link is configured; otherwise there is
/// nothing to protect and the check is a no-op.
async fn check_generation(state: &AppState, expected: Option<&str>) -> Result<(), ApiError> {
    let (Some(expected), Some(link)) = (expected, state.link.as_ref()) else {
        return Ok(());
    };
    let current = generation_token(state.journal.run_id(), link.status().clock_epoch);
    if current != expected {
        let (tail, _) = state.journal.checkpoint().await?;
        return Err(ApiError::GenerationChanged {
            expected: expected.to_owned(),
            current,
            cursor: format_cursor(tail),
        });
    }
    Ok(())
}

async fn mavlink_messages(
    State(state): State<Arc<AppState>>,
    Query(query): Query<MessageQuery>,
) -> Result<Json<MessageBatch>, ApiError> {
    let filter = RecordFilter::from_message_query(&query)?;
    let mut cursor = parse_cursor(query.after.as_deref())?;
    let limit = query.limit.unwrap_or(DEFAULT_MESSAGE_BATCH_LIMIT);
    if limit == 0 || limit > MAX_MESSAGE_BATCH_LIMIT {
        return Err(ApiError::BadRequest(format!(
            "limit must be between 1 and {MAX_MESSAGE_BATCH_LIMIT}"
        )));
    }
    if query.is_range() {
        return read_message_range(&state, &query, &filter, cursor, limit).await;
    }
    if query.wait_ms > MAX_MESSAGE_WAIT_MS {
        return Err(ApiError::BadRequest(format!(
            "wait_ms must not exceed {MAX_MESSAGE_WAIT_MS}"
        )));
    }
    if query
        .max_lag_ms
        .is_some_and(|value| value > MAX_MESSAGE_LAG_MS)
    {
        return Err(ApiError::BadRequest(format!(
            "max_lag_ms must not exceed {MAX_MESSAGE_LAG_MS}"
        )));
    }
    let mut live = state.journal.subscribe();
    let deadline = Instant::now() + Duration::from_millis(query.wait_ms);

    loop {
        check_generation(&state, query.expected_generation.as_deref()).await?;
        let read = state
            .journal
            .read_after_bounded(cursor, MAX_JOURNAL_SCAN_RECORDS)
            .await?;
        if let (Some(max_lag_ms), Some(first)) = (query.max_lag_ms, read.records.first())
            && wall_time_ns().saturating_sub(first.ingest_time_ns)
                > max_lag_ms.saturating_mul(1_000_000)
        {
            let (tail, tail_clock) = state.journal.checkpoint().await?;
            return Ok(Json(MessageBatch {
                records: Vec::new(),
                next_cursor: format_cursor(tail),
                next_clock: tail_clock,
            }));
        }
        let mut next_clock = read.tail_clock;
        let mut records = Vec::new();
        for record in read.records {
            cursor = cursor.max(record.sequence);
            next_clock = record.sim_clock;
            if filter.matches(&record) {
                records.push(record);
                if records.len() == limit {
                    break;
                }
            }
        }
        if query.collapse {
            records = collapse_state_records(records);
        }
        let records: Vec<Value> = records.iter().filter_map(encode_telemetry_record).collect();
        if !records.is_empty()
            || query.wait_ms == 0
            || Instant::now() >= deadline
            || cursor < state.journal.tail()
        {
            return Ok(Json(MessageBatch {
                records,
                next_cursor: format_cursor(cursor),
                next_clock,
            }));
        }

        match tokio::time::timeout_at(deadline, live.recv()).await {
            Ok(Ok(_)) | Ok(Err(tokio::sync::broadcast::error::RecvError::Lagged(_))) => {}
            Ok(Err(tokio::sync::broadcast::error::RecvError::Closed)) | Err(_) => {
                return Ok(Json(MessageBatch {
                    records: Vec::new(),
                    next_cursor: format_cursor(cursor),
                    next_clock,
                }));
            }
        }
    }
}

/// One-shot historical scan bounded by ingest time and/or cursor. Scans until
/// `limit` matches, the range end, or the journal tail. A batch with fewer
/// than `limit` records means the range is exhausted; otherwise continue with
/// `after=next_cursor` and the same bounds.
async fn read_message_range(
    state: &AppState,
    query: &MessageQuery,
    filter: &RecordFilter,
    mut cursor: u64,
    limit: usize,
) -> Result<Json<MessageBatch>, ApiError> {
    if query.wait_ms != 0 || query.max_lag_ms.is_some() {
        return Err(ApiError::BadRequest(
            "range queries (since_ns/until_ns/last_ms/through) do not accept wait_ms or max_lag_ms"
                .to_owned(),
        ));
    }
    if query.since_ns.is_some() && query.last_ms.is_some() {
        return Err(ApiError::BadRequest(
            "since_ns and last_ms are mutually exclusive".to_owned(),
        ));
    }
    let window = TimeWindow {
        since_ns: query.since_ns.or_else(|| {
            query
                .last_ms
                .map(|ms| wall_time_ns().saturating_sub(ms.saturating_mul(1_000_000)))
        }),
        until_ns: query.until_ns,
    };
    let through = query
        .through
        .as_deref()
        .map(|value| parse_cursor(Some(value)))
        .transpose()?;
    check_generation(state, query.expected_generation.as_deref()).await?;

    let mut records = Vec::new();
    let mut next_clock = None;
    let mut fallback_clock;
    'scan: loop {
        let read = state
            .journal
            .read_window_bounded(cursor, window, MAX_JOURNAL_SCAN_RECORDS)
            .await?;
        fallback_clock = read.tail_clock;
        if read.records.is_empty() {
            break;
        }
        for record in read.records {
            if through.is_some_and(|through| record.sequence > through)
                || window
                    .until_ns
                    .is_some_and(|until| record.ingest_time_ns > until)
            {
                break 'scan;
            }
            cursor = record.sequence;
            next_clock = Some(record.sim_clock);
            if window
                .since_ns
                .is_some_and(|since| record.ingest_time_ns < since)
            {
                continue;
            }
            if filter.matches(&record) {
                records.push(record);
                if records.len() == limit {
                    break 'scan;
                }
            }
        }
    }
    if query.collapse {
        records = collapse_state_records(records);
    }
    Ok(Json(MessageBatch {
        records: records.iter().filter_map(encode_telemetry_record).collect(),
        next_cursor: format_cursor(cursor),
        next_clock: next_clock.unwrap_or(fallback_clock),
    }))
}

async fn read_record_batch(
    state: Arc<AppState>,
    mut query: RecordQuery,
    forced_class: Option<&str>,
) -> Result<Json<RecordBatch>, ApiError> {
    if let Some(forced_class) = forced_class {
        query.classes = Some(forced_class.to_owned());
    }
    let filter = RecordFilter::from_query(&query)?;
    let mut cursor = parse_cursor(query.after.as_deref())?;
    let limit = query.limit.unwrap_or(DEFAULT_MESSAGE_BATCH_LIMIT);
    if limit == 0 || limit > MAX_MESSAGE_BATCH_LIMIT {
        return Err(ApiError::BadRequest(format!(
            "limit must be between 1 and {MAX_MESSAGE_BATCH_LIMIT}"
        )));
    }
    if query.wait_ms > MAX_MESSAGE_WAIT_MS {
        return Err(ApiError::BadRequest(format!(
            "wait_ms must not exceed {MAX_MESSAGE_WAIT_MS}"
        )));
    }
    let mut live = state.journal.subscribe();
    let deadline = Instant::now() + Duration::from_millis(query.wait_ms);

    loop {
        let read = state
            .journal
            .read_after_bounded(cursor, MAX_JOURNAL_SCAN_RECORDS)
            .await?;
        let mut next_clock = read.tail_clock;
        let mut records = Vec::new();
        for record in read.records {
            cursor = cursor.max(record.sequence);
            next_clock = record.sim_clock;
            if filter.matches(&record) {
                records.push(encode_record(&record));
                if records.len() == limit {
                    break;
                }
            }
        }
        if !records.is_empty()
            || query.wait_ms == 0
            || Instant::now() >= deadline
            || cursor < state.journal.tail()
        {
            return Ok(Json(RecordBatch {
                records,
                next_cursor: format_cursor(cursor),
                next_clock,
            }));
        }

        match tokio::time::timeout_at(deadline, live.recv()).await {
            Ok(Ok(_)) | Ok(Err(tokio::sync::broadcast::error::RecvError::Lagged(_))) => {}
            Ok(Err(tokio::sync::broadcast::error::RecvError::Closed)) | Err(_) => {
                return Ok(Json(RecordBatch {
                    records: Vec::new(),
                    next_cursor: format_cursor(cursor),
                    next_clock,
                }));
            }
        }
    }
}

fn encode_record(record: &JournalRecord) -> Value {
    let (kind, data) = match &record.payload {
        RecordPayload::Diagnostic(event) => (
            "diagnostic.event",
            serde_json::to_value(event).expect("DiagnosticEvent is serializable"),
        ),
        RecordPayload::MavlinkFrame(frame) => (
            match frame.direction {
                crate::records::Direction::Rx => "mavlink.rx",
                crate::records::Direction::Tx => "mavlink.tx",
            },
            json!({
                "link_id": frame.link_id,
                "direction": frame.direction,
                "protocol_version": frame.protocol_version,
                "sequence": frame.sequence,
                "system_id": frame.system_id,
                "component_id": frame.component_id,
                "message_id": frame.message_id,
                "message_name": frame.message_name,
                "fields": frame.fields,
                "signed": frame.signed,
                "frame_base64": BASE64.encode(&frame.frame),
            }),
        ),
    };
    json!({
        "schema_version": record.schema_version,
        "sequence": record.sequence,
        "cursor": format_cursor(record.sequence),
        "ingest_time_ns": record.ingest_time_ns,
        "correlation_id": record.correlation_id,
        "sim_clock": record.sim_clock,
        "kind": kind,
        "data": data,
    })
}

fn encode_telemetry_record(record: &JournalRecord) -> Option<Value> {
    let RecordPayload::MavlinkFrame(frame) = &record.payload else {
        return None;
    };
    Some(json!({
        "received_time": record.ingest_time_ns.to_string(),
        "received_time_ns": record.ingest_time_ns,
        "direction": frame.direction,
        "system_id": frame.system_id,
        "component_id": frame.component_id,
        "message": frame.message_name,
        "fields": frame.fields,
        "cursor": format_cursor(record.sequence),
        "sim_clock": record.sim_clock,
    }))
}

#[derive(Debug, Deserialize)]
struct DiagnosticBatch {
    events: Vec<DiagnosticEvent>,
}

#[derive(Debug, Serialize)]
struct DiagnosticBatchResult {
    accepted: usize,
    duplicates: usize,
    cursor: String,
}

async fn ingest_diagnostics(
    State(state): State<Arc<AppState>>,
    Json(batch): Json<DiagnosticBatch>,
) -> Result<(StatusCode, Json<DiagnosticBatchResult>), ApiError> {
    if batch.events.is_empty() {
        return Err(ApiError::BadRequest(
            "events must contain at least one event".to_owned(),
        ));
    }
    let mut accepted = 0;
    let mut duplicates = 0;
    let mut last_sequence = state.journal.tail();
    for event in batch.events {
        validate_diagnostic(&event)?;
        let result = state.journal.append_diagnostic(event).await?;
        last_sequence = last_sequence.max(result.sequence);
        if result.appended {
            accepted += 1;
        } else {
            duplicates += 1;
        }
    }
    Ok((
        StatusCode::ACCEPTED,
        Json(DiagnosticBatchResult {
            accepted,
            duplicates,
            cursor: format_cursor(last_sequence),
        }),
    ))
}

fn validate_diagnostic(event: &DiagnosticEvent) -> Result<(), ApiError> {
    if event.source.trim().is_empty()
        || event.source_instance.trim().is_empty()
        || event.category.trim().is_empty()
        || event.event.trim().is_empty()
    {
        return Err(ApiError::BadRequest(
            "source, source_instance, category, and event must be non-empty".to_owned(),
        ));
    }
    Ok(())
}

#[derive(Debug, Deserialize)]
struct SendFrameRequest {
    frame_base64: String,
}

#[derive(Debug, Deserialize)]
struct SendMessageRequest {
    message: String,
    fields: serde_json::Map<String, Value>,
    source_system: Option<u8>,
    source_component: Option<u8>,
}

#[derive(Debug, Deserialize)]
struct CommandRequest {
    command: u32,
    #[serde(default)]
    params: Vec<f64>,
    target_system: Option<u8>,
    target_component: Option<u8>,
    #[serde(default = "default_timeout_ms")]
    timeout_ms: u64,
}

async fn execute_command(
    State(state): State<Arc<AppState>>,
    Json(request): Json<CommandRequest>,
) -> Result<Json<Value>, ApiError> {
    let operations = require_operations(&state)?;
    let result = operations
        .command(
            request.command,
            &request.params,
            request.target_system,
            request.target_component,
            operation_timeout(request.timeout_ms)?,
        )
        .await?;
    Ok(Json(
        serde_json::to_value(result).expect("command result is serializable"),
    ))
}

#[derive(Debug, Default, Deserialize)]
struct OperationQuery {
    timeout_ms: Option<u64>,
    target_system: Option<u8>,
    target_component: Option<u8>,
}

async fn get_parameter(
    State(state): State<Arc<AppState>>,
    Path(name): Path<String>,
    Query(query): Query<OperationQuery>,
) -> Result<Json<Value>, ApiError> {
    let result = require_operations(&state)?
        .get_parameter(
            &name,
            operation_timeout(query.timeout_ms.unwrap_or(default_timeout_ms()))?,
        )
        .await?;
    Ok(Json(
        serde_json::to_value(result).expect("parameter result is serializable"),
    ))
}

#[derive(Debug, Deserialize)]
struct SetParameterRequest {
    value: f64,
    #[serde(rename = "type")]
    param_type: u64,
    timeout_ms: Option<u64>,
}

async fn set_parameter(
    State(state): State<Arc<AppState>>,
    Path(name): Path<String>,
    Json(request): Json<SetParameterRequest>,
) -> Result<Json<Value>, ApiError> {
    let result = require_operations(&state)?
        .set_parameter(
            &name,
            request.value,
            request.param_type,
            operation_timeout(request.timeout_ms.unwrap_or(default_timeout_ms()))?,
        )
        .await?;
    Ok(Json(
        serde_json::to_value(result).expect("parameter result is serializable"),
    ))
}

async fn list_parameters(
    State(state): State<Arc<AppState>>,
    Query(query): Query<OperationQuery>,
) -> Result<Json<Value>, ApiError> {
    let parameters = require_operations(&state)?
        .list_parameters(operation_timeout(query.timeout_ms.unwrap_or(15_000))?)
        .await?;
    Ok(Json(json!({"parameters": parameters})))
}

#[derive(Debug, Deserialize)]
struct SetParametersRequest {
    parameters: Vec<Value>,
    #[serde(default = "default_batch_timeout_ms")]
    timeout_ms: u64,
    #[serde(default = "default_parameter_retries")]
    retries: usize,
}

async fn set_parameters(
    State(state): State<Arc<AppState>>,
    Json(request): Json<SetParametersRequest>,
) -> Result<Json<Value>, ApiError> {
    let parameters = require_operations(&state)?
        .set_parameters(
            &request.parameters,
            operation_timeout(request.timeout_ms)?,
            request.retries,
        )
        .await?;
    Ok(Json(json!({"parameters": parameters})))
}

#[derive(Debug, Deserialize)]
struct RequestMessageRequest {
    message: Value,
    target_system: Option<u8>,
    target_component: Option<u8>,
    #[serde(default = "default_timeout_ms")]
    timeout_ms: u64,
}

async fn request_message(
    State(state): State<Arc<AppState>>,
    Json(request): Json<RequestMessageRequest>,
) -> Result<Json<Value>, ApiError> {
    let message = request
        .message
        .as_str()
        .ok_or_else(|| ApiError::BadRequest("message must be a message name".to_owned()))?;
    let result = require_operations(&state)?
        .request_message(
            message,
            request.target_system,
            request.target_component,
            operation_timeout(request.timeout_ms)?,
        )
        .await?;
    Ok(Json(result))
}

async fn autopilot_version(
    State(state): State<Arc<AppState>>,
    Query(query): Query<OperationQuery>,
) -> Result<Json<Value>, ApiError> {
    let result = require_operations(&state)?
        .autopilot_version(
            query.target_system,
            query.target_component,
            operation_timeout(query.timeout_ms.unwrap_or(default_timeout_ms()))?,
        )
        .await?;
    Ok(Json(result))
}

async fn capabilities(
    State(state): State<Arc<AppState>>,
    Query(query): Query<OperationQuery>,
) -> Result<Json<Value>, ApiError> {
    let result = require_operations(&state)?
        .capabilities(operation_timeout(
            query.timeout_ms.unwrap_or(default_timeout_ms()),
        )?)
        .await?;
    Ok(Json(result))
}

async fn components(State(state): State<Arc<AppState>>) -> Json<Value> {
    Json(json!({
        "components": state
            .link
            .as_ref()
            .map_or_else(Vec::new, MavlinkLinkHandle::components),
    }))
}

async fn get_message_interval(
    State(state): State<Arc<AppState>>,
    Path(message): Path<String>,
    Query(query): Query<OperationQuery>,
) -> Result<Json<Value>, ApiError> {
    let result = require_operations(&state)?
        .message_interval(
            &message,
            operation_timeout(query.timeout_ms.unwrap_or(default_timeout_ms()))?,
        )
        .await?;
    Ok(Json(result))
}

#[derive(Debug, Deserialize)]
struct FileQuery {
    path: Option<String>,
    #[serde(default)]
    download: bool,
    #[serde(default = "default_true")]
    verify_crc: bool,
}

async fn files(
    State(state): State<Arc<AppState>>,
    Query(query): Query<FileQuery>,
) -> Result<Response, ApiError> {
    let path = required_path(query.path)?;
    if !query.download {
        let files = require_operations(&state)?
            .list_files(&path, Duration::from_secs(30))
            .await?;
        return Ok(Json(json!({"files": files})).into_response());
    }
    let download = require_operations(&state)?
        .download_file(&path, query.verify_crc, Duration::from_secs(60))
        .await?;
    Response::builder()
        .status(StatusCode::OK)
        .header(CONTENT_TYPE, "application/octet-stream")
        .header("X-MAVFTP-Path", download.path)
        .header("X-MAVFTP-CRC32", format!("{:08x}", download.crc32))
        .header("X-LinkHub-After-Cursor", download.after_cursor)
        .body(Body::from(download.content))
        .map_err(|error| ApiError::BadRequest(error.to_string()))
}

async fn upload_file(
    State(state): State<Arc<AppState>>,
    Query(query): Query<FileQuery>,
    body: Bytes,
) -> Result<Json<Value>, ApiError> {
    let path = required_path(query.path)?;
    let bytes = require_operations(&state)?
        .upload_file(&path, body.to_vec(), Duration::from_secs(60))
        .await?;
    Ok(Json(json!({"path": path, "bytes": bytes})))
}

async fn remove_file(
    State(state): State<Arc<AppState>>,
    Query(query): Query<FileQuery>,
) -> Result<Json<Value>, ApiError> {
    let path = required_path(query.path)?;
    require_operations(&state)?
        .remove_file(&path, Duration::from_secs(30))
        .await?;
    Ok(Json(json!({"removed": path})))
}

#[derive(Debug, Deserialize)]
struct DirectoryRequest {
    path: String,
}

async fn create_directory(
    State(state): State<Arc<AppState>>,
    Json(request): Json<DirectoryRequest>,
) -> Result<Json<Value>, ApiError> {
    if request.path.is_empty() {
        return Err(ApiError::BadRequest("path is required".to_owned()));
    }
    let created = require_operations(&state)?
        .create_directory(&request.path, Duration::from_secs(30))
        .await?;
    Ok(Json(json!({"path": request.path, "created": created})))
}

#[derive(Debug, Deserialize)]
struct LogQuery {
    #[serde(default = "default_log_timeout_ms")]
    timeout_ms: u64,
    #[serde(default = "default_log_retries")]
    max_retries: u32,
}

async fn list_logs(
    State(state): State<Arc<AppState>>,
    Query(query): Query<LogQuery>,
) -> Result<Json<Value>, ApiError> {
    let logs = require_operations(&state)?
        .list_logs(operation_timeout(query.timeout_ms)?)
        .await?;
    Ok(Json(json!({"logs": logs})))
}

async fn download_log(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u16>,
    Query(query): Query<LogQuery>,
) -> Result<Response, ApiError> {
    let download = require_operations(&state)?
        .download_log(id, operation_timeout(query.timeout_ms)?, query.max_retries)
        .await?;
    Response::builder()
        .status(StatusCode::OK)
        .header(CONTENT_TYPE, "application/octet-stream")
        .header(
            "Content-Disposition",
            format!("attachment; filename=\"dataflash-{}.BIN\"", download.id),
        )
        .header("X-DataFlash-Log-Id", download.id.to_string())
        .header("X-DataFlash-Time-UTC", download.time_utc.to_string())
        .header("X-LinkHub-After-Cursor", download.after_cursor)
        .body(Body::from(download.content))
        .map_err(|error| ApiError::BadRequest(error.to_string()))
}

async fn motor_status(State(state): State<Arc<AppState>>) -> Json<Value> {
    Json(state.motor.as_ref().map_or_else(
        || json!({"available": false}),
        |motor| serde_json::to_value(motor.status()).expect("motor status is serializable"),
    ))
}

#[derive(Debug, Deserialize)]
struct SetMotorRequest {
    speed_percent: u8,
    direction: MotorDirection,
    timeout_ms: u64,
}

async fn set_motor(
    State(state): State<Arc<AppState>>,
    Json(request): Json<SetMotorRequest>,
) -> Result<Json<Value>, ApiError> {
    let motor = require_motor(&state)?;
    motor
        .set_running(request.speed_percent, request.direction, request.timeout_ms)
        .await?;
    Ok(Json(
        serde_json::to_value(motor.status()).expect("motor status is serializable"),
    ))
}

async fn stop_motor(State(state): State<Arc<AppState>>) -> Result<Json<Value>, ApiError> {
    let motor = require_motor(&state)?;
    motor.stop().await?;
    Ok(Json(
        serde_json::to_value(motor.status()).expect("motor status is serializable"),
    ))
}

async fn reconnect_motor(State(state): State<Arc<AppState>>) -> Result<Json<Value>, ApiError> {
    let motor = require_motor(&state)?;
    let device = motor.reconnect().await?;
    let mut status = serde_json::to_value(motor.status()).expect("motor status is serializable");
    status
        .as_object_mut()
        .expect("motor status is an object")
        .insert("device".to_owned(), Value::String(device));
    Ok(Json(status))
}

async fn set_message_rates(
    State(state): State<Arc<AppState>>,
    Json(rates): Json<serde_json::Map<String, Value>>,
) -> Result<Json<Value>, ApiError> {
    if rates.is_empty() {
        return Err(ApiError::BadRequest(
            "at least one message rate is required".to_owned(),
        ));
    }
    let configured = require_operations(&state)?
        .set_message_rates(&rates, std::time::Duration::from_secs(10))
        .await?;
    Ok(Json(json!({"configured": configured})))
}

async fn send_mavlink_message(
    State(state): State<Arc<AppState>>,
    Json(request): Json<SendMessageRequest>,
) -> Result<(StatusCode, Json<Value>), ApiError> {
    let link = state
        .link
        .as_ref()
        .ok_or_else(|| ApiError::BadRequest("MAVLink is not configured".to_owned()))?;
    let sequence = link
        .send_message(
            &request.message.to_uppercase(),
            &request.fields,
            request.source_system,
            request.source_component,
        )
        .await
        .map_err(|error| match error {
            LinkError::Codec(_) | LinkError::InvalidFrame(_) => {
                ApiError::BadRequest(error.to_string())
            }
            other => ApiError::Link(other),
        })?;
    Ok((
        StatusCode::ACCEPTED,
        Json(json!({
            "accepted": true,
            "after_cursor": format_cursor(sequence),
        })),
    ))
}

fn require_operations(state: &AppState) -> Result<&MavlinkOperations, ApiError> {
    state
        .operations
        .as_ref()
        .ok_or_else(|| ApiError::BadRequest("MAVLink is not configured".to_owned()))
}

fn require_motor(state: &AppState) -> Result<&MotorHandle, ApiError> {
    state
        .motor
        .as_ref()
        .ok_or_else(|| ApiError::NotFound("Bluetooth motor support is not configured".to_owned()))
}

const fn default_timeout_ms() -> u64 {
    3_000
}

const fn default_batch_timeout_ms() -> u64 {
    15_000
}

const fn default_parameter_retries() -> usize {
    2
}

const fn default_log_timeout_ms() -> u64 {
    1_000
}

const fn default_log_retries() -> u32 {
    5
}

const fn default_true() -> bool {
    true
}

fn required_path(path: Option<String>) -> Result<String, ApiError> {
    path.filter(|path| !path.is_empty())
        .ok_or_else(|| ApiError::BadRequest("path is required".to_owned()))
}

fn operation_timeout(milliseconds: u64) -> Result<std::time::Duration, ApiError> {
    if milliseconds == 0 || milliseconds > 120_000 {
        return Err(ApiError::BadRequest(
            "timeout_ms must be between 1 and 120000".to_owned(),
        ));
    }
    Ok(std::time::Duration::from_millis(milliseconds))
}

async fn send_mavlink_frame(
    State(state): State<Arc<AppState>>,
    Json(request): Json<SendFrameRequest>,
) -> Result<(StatusCode, Json<Value>), ApiError> {
    let link = state
        .link
        .as_ref()
        .ok_or_else(|| ApiError::BadRequest("MAVLink is not configured".to_owned()))?;
    let frame = BASE64
        .decode(request.frame_base64)
        .map_err(|error| ApiError::BadRequest(format!("invalid frame_base64: {error}")))?;
    let frame_size = frame.len();
    let sequence = link.send_raw(frame).await?;
    Ok((
        StatusCode::ACCEPTED,
        Json(json!({
            "accepted": true,
            "frame_size": frame_size,
            "after_cursor": format_cursor(sequence),
        })),
    ))
}

fn format_cursor(sequence: u64) -> String {
    format!("v1:{sequence}")
}

fn parse_cursor(value: Option<&str>) -> Result<u64, ApiError> {
    let Some(value) = value else {
        return Ok(0);
    };
    let sequence = value.strip_prefix("v1:").unwrap_or(value);
    sequence
        .parse()
        .map_err(|_| ApiError::BadRequest("cursor must be 'v1:<sequence>'".to_owned()))
}

#[cfg(test)]
mod tests {
    use axum::{
        body::{Body, to_bytes},
        http::Request,
    };
    use serde_json::Map;
    use tempfile::TempDir;
    use tower::ServiceExt;
    use uuid::Uuid;

    use mavlink::MavHeader;
    use tokio::{
        io::{AsyncReadExt, AsyncWriteExt},
        net::TcpListener,
        time::timeout,
    };

    use super::*;
    use crate::{
        codec::serialize_message,
        journal::JournalConfig,
        mavlink::{MavlinkLinkConfig, heartbeat_message, start_link},
        records::{DiagnosticLevel, Direction, MavlinkFrame, RecordPayload, wall_time_ns},
    };

    fn heartbeat_frame(sequence: u8, system_id: u8, component_id: u8) -> Vec<u8> {
        serialize_message(
            &heartbeat_message(),
            MavHeader {
                sequence,
                system_id,
                component_id,
            },
        )
        .expect("heartbeat frame")
    }

    fn event(run_id: Uuid) -> DiagnosticEvent {
        DiagnosticEvent {
            schema_version: 1,
            run_id,
            source: "groundstation".to_owned(),
            source_instance: "gcs-1".to_owned(),
            source_sequence: 1,
            source_wall_time_ns: wall_time_ns(),
            source_monotonic_ns: Some(10),
            sim_time_ns: Some(20),
            sim_time_quality: None,
            level: DiagnosticLevel::Info,
            category: "operation".to_owned(),
            event: "operation.requested".to_owned(),
            message: "requested".to_owned(),
            correlation_id: Some(Uuid::new_v4()),
            causation_id: None,
            fields: Map::new(),
            related_records: Vec::new(),
        }
    }

    fn mavlink_frame(
        message_id: u32,
        message_name: &str,
        fields: Map<String, Value>,
    ) -> MavlinkFrame {
        MavlinkFrame {
            link_id: "test".to_owned(),
            direction: Direction::Rx,
            protocol_version: 2,
            sequence: 1,
            system_id: 1,
            component_id: 1,
            message_id,
            message_name: message_name.to_owned(),
            fields,
            signed: false,
            frame: Vec::new(),
        }
    }

    #[tokio::test]
    async fn ingests_deduplicates_and_batches_diagnostics() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path(), run_id);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        let app = router(journal.clone(), None);
        let body = serde_json::to_vec(&json!({"events": [event(run_id)]})).expect("request JSON");

        let first = app
            .clone()
            .oneshot(
                Request::post("/v1/diagnostics/events")
                    .header(CONTENT_TYPE, "application/json")
                    .body(Body::from(body.clone()))
                    .expect("request"),
            )
            .await
            .expect("response");
        let duplicate = app
            .clone()
            .oneshot(
                Request::post("/v1/diagnostics/events")
                    .header(CONTENT_TYPE, "application/json")
                    .body(Body::from(body))
                    .expect("request"),
            )
            .await
            .expect("response");
        let read = app
            .oneshot(
                Request::get("/v1/diagnostics/events")
                    .body(Body::empty())
                    .expect("request"),
            )
            .await
            .expect("response");

        assert_eq!(first.status(), StatusCode::ACCEPTED);
        let duplicate_body = to_bytes(duplicate.into_body(), usize::MAX)
            .await
            .expect("duplicate body");
        assert_eq!(
            serde_json::from_slice::<Value>(&duplicate_body).expect("duplicate JSON")["duplicates"],
            1
        );
        let batch_body = to_bytes(read.into_body(), usize::MAX)
            .await
            .expect("batch body");
        let batch: Value = serde_json::from_slice(&batch_body).expect("diagnostic batch");
        assert_eq!(batch["records"][0]["kind"], "diagnostic.event");
        assert_eq!(batch["records"][0]["data"]["source"], "groundstation");
        assert_eq!(batch["next_cursor"], "v1:1");

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn serves_the_generated_protocol_schema() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path(), run_id);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        let app = router(journal.clone(), None);

        let response = app
            .oneshot(
                Request::get("/v1/schema")
                    .body(Body::empty())
                    .expect("request"),
            )
            .await
            .expect("response");
        let body = to_bytes(response.into_body(), usize::MAX)
            .await
            .expect("schema body");
        let schema: Value = serde_json::from_slice(&body).expect("schema JSON");

        assert_eq!(schema["properties"]["schema_version"]["const"], 1);
        assert_eq!(
            schema["$defs"]["MavLandedState"]["x-enum-varnames"][2],
            "IN_AIR"
        );
        assert_eq!(
            schema["$defs"]["ExtendedSysState"]["properties"]["landed_state"]["$ref"],
            "#/$defs/MavLandedState"
        );

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn serves_static_files_without_shadowing_api_routes() {
        let temp = TempDir::new().expect("temp directory");
        let static_dir = temp.path().join("static");
        std::fs::create_dir(&static_dir).expect("static directory");
        std::fs::write(static_dir.join("index.html"), "linkhub ui").expect("index");
        std::fs::write(static_dir.join("app.js"), "console.log('ui')").expect("script");

        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path().join("journal"), run_id);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        let app = router_with_motor_and_static(
            journal.clone(),
            None,
            None,
            Some(static_dir.clone()),
            true,
        );

        let index = app
            .clone()
            .oneshot(Request::get("/").body(Body::empty()).expect("request"))
            .await
            .expect("response");
        let script = app
            .clone()
            .oneshot(
                Request::get("/app.js")
                    .body(Body::empty())
                    .expect("request"),
            )
            .await
            .expect("response");
        let api = app
            .oneshot(
                Request::get("/health/live")
                    .body(Body::empty())
                    .expect("request"),
            )
            .await
            .expect("response");

        assert_eq!(index.status(), StatusCode::OK);
        assert_eq!(index.headers()[CACHE_CONTROL], "no-store");
        assert_eq!(api.headers()[CACHE_CONTROL], "no-store");
        assert_eq!(script.headers()[CACHE_CONTROL], "no-store");
        assert_eq!(
            to_bytes(index.into_body(), usize::MAX)
                .await
                .expect("index body"),
            "linkhub ui"
        );
        assert_eq!(script.status(), StatusCode::OK);
        assert_eq!(script.headers()[CONTENT_TYPE], "text/javascript");
        assert_eq!(api.status(), StatusCode::OK);

        let cached =
            router_with_motor_and_static(journal.clone(), None, None, Some(static_dir), false)
                .oneshot(Request::get("/").body(Body::empty()).expect("request"))
                .await
                .expect("response");
        assert!(cached.headers().get(CACHE_CONTROL).is_none());

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn message_batch_advances_past_nonmatching_records() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path(), run_id);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        journal
            .append(RecordPayload::Diagnostic(Box::new(event(run_id))), None)
            .await
            .expect("diagnostic");
        let app = router(journal.clone(), None);

        let empty = app
            .clone()
            .oneshot(
                Request::get("/v1/mavlink/messages?after=v1:0&messages=STATUSTEXT")
                    .body(Body::empty())
                    .expect("request"),
            )
            .await
            .expect("response");
        let empty_body = to_bytes(empty.into_body(), usize::MAX)
            .await
            .expect("empty body");
        let empty_batch: Value = serde_json::from_slice(&empty_body).expect("empty message batch");
        assert_eq!(empty_batch["records"], json!([]));
        assert_eq!(empty_batch["next_cursor"], "v1:1");

        journal
            .append(
                RecordPayload::MavlinkFrame(MavlinkFrame {
                    link_id: "test".to_owned(),
                    direction: Direction::Rx,
                    protocol_version: 2,
                    sequence: 1,
                    system_id: 1,
                    component_id: 1,
                    message_id: 253,
                    message_name: "STATUSTEXT".to_owned(),
                    fields: serde_json::Map::from_iter([
                        ("severity".to_owned(), json!(6)),
                        ("text".to_owned(), json!("ready")),
                    ]),
                    signed: false,
                    frame: Vec::new(),
                }),
                None,
            )
            .await
            .expect("STATUSTEXT");

        let matched = app
            .oneshot(
                Request::get("/v1/mavlink/messages?after=v1:1&messages=STATUSTEXT&limit=1")
                    .body(Body::empty())
                    .expect("request"),
            )
            .await
            .expect("response");
        let matched_body = to_bytes(matched.into_body(), usize::MAX)
            .await
            .expect("matched body");
        let matched_batch: Value =
            serde_json::from_slice(&matched_body).expect("matched message batch");
        assert_eq!(matched_batch["records"][0]["message"], "STATUSTEXT");
        assert_eq!(matched_batch["records"][0]["fields"]["text"], "ready");
        assert_eq!(matched_batch["next_cursor"], "v1:2");

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn message_range_query_filters_by_time_cursor_fields_and_text() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let mut config = JournalConfig::for_directory(temp.path(), run_id);
        config.max_chunk_records = 2;
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        let frame = |name: &str, fields: Value| {
            RecordPayload::MavlinkFrame(MavlinkFrame {
                link_id: "test".to_owned(),
                direction: Direction::Rx,
                protocol_version: 2,
                sequence: 1,
                system_id: 1,
                component_id: 1,
                message_id: 0,
                message_name: name.to_owned(),
                fields: fields.as_object().expect("fields").clone(),
                signed: false,
                frame: Vec::new(),
            })
        };
        journal
            .append(
                frame("STATUSTEXT", json!({"severity": 6, "text": "early"})),
                None,
            )
            .await
            .expect("early");
        tokio::time::sleep(Duration::from_millis(5)).await;
        let since_ns = wall_time_ns();
        for (name, fields) in [
            (
                "STATUSTEXT",
                json!({"severity": 0, "text": "Crash: Disarming"}),
            ),
            (
                "NAMED_VALUE_FLOAT",
                json!({"name": "RAWES_PEN", "value": 1.0}),
            ),
            ("NAMED_VALUE_FLOAT", json!({"name": "YFF_U", "value": 0.2})),
            ("STATUSTEXT", json!({"severity": 6, "text": "late"})),
        ] {
            journal
                .append(frame(name, fields), None)
                .await
                .expect("append");
        }
        let app = router(journal.clone(), None);
        let get = |uri: String| {
            let app = app.clone();
            async move {
                let response = app
                    .oneshot(Request::get(uri).body(Body::empty()).expect("request"))
                    .await
                    .expect("response");
                let status = response.status();
                let body = to_bytes(response.into_body(), usize::MAX)
                    .await
                    .expect("body");
                (
                    status,
                    serde_json::from_slice::<Value>(&body).expect("json"),
                )
            }
        };
        let texts = |batch: &Value| -> Vec<String> {
            batch["records"]
                .as_array()
                .expect("records")
                .iter()
                .map(|record| {
                    record["fields"]["text"]
                        .as_str()
                        .or(record["fields"]["name"].as_str())
                        .expect("text or name")
                        .to_owned()
                })
                .collect()
        };

        let (status, batch) = get(format!("/v1/mavlink/messages?since_ns={since_ns}")).await;
        assert_eq!(status, StatusCode::OK);
        assert_eq!(
            texts(&batch),
            ["Crash: Disarming", "RAWES_PEN", "YFF_U", "late"]
        );

        let (_, batch) = get("/v1/mavlink/messages?through=v1:3".to_owned()).await;
        assert_eq!(texts(&batch), ["early", "Crash: Disarming", "RAWES_PEN"]);
        assert_eq!(batch["next_cursor"], "v1:3");

        let (_, batch) =
            get("/v1/mavlink/messages?last_ms=60000&eq=name=RAWES_PEN".to_owned()).await;
        assert_eq!(texts(&batch), ["RAWES_PEN"]);

        let (_, batch) =
            get("/v1/mavlink/messages?last_ms=60000&eq=severity=0&contains=crash".to_owned()).await;
        assert_eq!(texts(&batch), ["Crash: Disarming"]);

        let (_, batch) = get("/v1/mavlink/messages?last_ms=60000&limit=2".to_owned()).await;
        assert_eq!(texts(&batch), ["early", "Crash: Disarming"]);
        let (_, batch) = get(format!(
            "/v1/mavlink/messages?last_ms=60000&limit=2&after={}",
            batch["next_cursor"].as_str().expect("cursor")
        ))
        .await;
        assert_eq!(texts(&batch), ["RAWES_PEN", "YFF_U"]);

        let (status, _) = get("/v1/mavlink/messages?last_ms=1000&wait_ms=10".to_owned()).await;
        assert_eq!(status, StatusCode::BAD_REQUEST);
        let (status, _) = get(format!(
            "/v1/mavlink/messages?last_ms=1000&since_ns={since_ns}"
        ))
        .await;
        assert_eq!(status, StatusCode::BAD_REQUEST);

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn message_batch_skips_stale_backlog_to_the_current_tail() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path(), run_id);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        journal
            .append(RecordPayload::Diagnostic(Box::new(event(run_id))), None)
            .await
            .expect("diagnostic");
        let app = router(journal.clone(), None);

        let response = app
            .oneshot(
                Request::get("/v1/mavlink/messages?after=v1:0&max_lag_ms=0")
                    .body(Body::empty())
                    .expect("request"),
            )
            .await
            .expect("response");
        let body = to_bytes(response.into_body(), usize::MAX)
            .await
            .expect("response body");
        let batch: Value = serde_json::from_slice(&body).expect("message batch");

        assert_eq!(batch["records"], json!([]));
        assert_eq!(batch["next_cursor"], "v1:1");

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn message_batch_rejects_stale_expected_generation() {
        let server = TcpListener::bind("127.0.0.1:0")
            .await
            .expect("test listener");
        let address = server.local_addr().expect("listener address");
        let peer = tokio::spawn(async move {
            let (mut socket, _) = server.accept().await.expect("accept link");
            socket
                .write_all(&heartbeat_frame(1, 1, 1))
                .await
                .expect("send vehicle heartbeat");
            // Keep the socket open for the rest of the test so the link
            // stays connected instead of racing its own disconnect-epoch bump.
            let mut sink = Vec::new();
            let _ = socket.read_to_end(&mut sink).await;
        });

        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path(), run_id);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        let mut link_config = MavlinkLinkConfig::sitl(address);
        link_config.heartbeat_interval = Duration::from_millis(10);
        let (link, link_task) = start_link(link_config, journal.clone());

        timeout(Duration::from_secs(2), async {
            let mut status = link.subscribe_status();
            while !status.borrow().ready {
                status.changed().await.expect("status update");
            }
        })
        .await
        .expect("link ready");

        let current_generation = format!("v1:{run_id}:1");
        let app = router(journal.clone(), Some(link));

        let ok = app
            .clone()
            .oneshot(
                Request::get(format!(
                    "/v1/mavlink/messages?after=v1:0&expected_generation={current_generation}"
                ))
                .body(Body::empty())
                .expect("request"),
            )
            .await
            .expect("response");
        assert_eq!(ok.status(), StatusCode::OK);

        let stale = app
            .oneshot(
                Request::get(
                    "/v1/mavlink/messages?after=v1:0&expected_generation=v1:\
                     00000000-0000-0000-0000-000000000000:1",
                )
                .body(Body::empty())
                .expect("request"),
            )
            .await
            .expect("response");
        assert_eq!(stale.status(), StatusCode::CONFLICT);
        let body = to_bytes(stale.into_body(), usize::MAX)
            .await
            .expect("conflict body");
        let value: Value = serde_json::from_slice(&body).expect("conflict json");
        assert_eq!(value["error"], "generation_changed");
        assert_eq!(value["current_generation"], current_generation);
        assert_eq!(
            value["expected_generation"],
            "v1:00000000-0000-0000-0000-000000000000:1"
        );
        assert!(value["cursor"].as_str().is_some());

        link_task.abort();
        peer.abort();
        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }

    #[tokio::test]
    async fn message_batch_collapses_state_but_preserves_events() {
        let temp = TempDir::new().expect("temp directory");
        let run_id = Uuid::new_v4();
        let config = JournalConfig::for_directory(temp.path(), run_id);
        let (journal, task) = JournalHandle::start(config).await.expect("journal");
        for payload in [
            mavlink_frame(
                30,
                "ATTITUDE",
                Map::from_iter([("roll".to_owned(), json!(1.0))]),
            ),
            mavlink_frame(
                253,
                "STATUSTEXT",
                Map::from_iter([("text".to_owned(), json!("first"))]),
            ),
            mavlink_frame(
                30,
                "ATTITUDE",
                Map::from_iter([("roll".to_owned(), json!(2.0))]),
            ),
            mavlink_frame(
                253,
                "STATUSTEXT",
                Map::from_iter([("text".to_owned(), json!("second"))]),
            ),
        ] {
            journal
                .append(RecordPayload::MavlinkFrame(payload), None)
                .await
                .expect("MAVLink frame");
        }
        let app = router(journal.clone(), None);

        let response = app
            .oneshot(
                Request::get("/v1/mavlink/messages?after=v1:0&limit=10&collapse=true")
                    .body(Body::empty())
                    .expect("request"),
            )
            .await
            .expect("response");
        let body = to_bytes(response.into_body(), usize::MAX)
            .await
            .expect("response body");
        let batch: Value = serde_json::from_slice(&body).expect("message batch");

        assert_eq!(batch["next_cursor"], "v1:4");
        assert_eq!(batch["records"].as_array().map(Vec::len), Some(3));
        assert_eq!(batch["records"][0]["fields"]["text"], "first");
        assert_eq!(batch["records"][1]["message"], "ATTITUDE");
        assert_eq!(batch["records"][1]["fields"]["roll"], 2.0);
        assert_eq!(batch["records"][2]["fields"]["text"], "second");

        journal.shutdown().await.expect("shutdown");
        task.await.expect("journal task");
    }
}
