use std::collections::{HashMap, HashSet, VecDeque};

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value, json};
use tokio::sync::Mutex;

use crate::{
    journal::{JournalError, JournalHandle},
    records::{DiagnosticEvent, DiagnosticLevel, format_cursor, wall_time_ns},
};

pub(crate) const MAX_CSV_BYTES: usize = 64 * 1024;
const RECENT_ROWS: usize = 5;
const SECOND_NS: u64 = 1_000_000_000;
const DAY_NS: i128 = 86_400 * SECOND_NS as i128;

#[derive(Debug, thiserror::Error)]
pub enum CsvError {
    #[error("{0}")]
    Invalid(String),
    #[error(transparent)]
    Journal(#[from] JournalError),
}

#[derive(Debug)]
pub struct CsvMapping(MappingFile);

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MappingFile {
    source: String,
    sequence_column: String,
    time_column: Option<String>,
    columns: Vec<CsvColumn>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CsvColumn {
    column: String,
    field: String,
    #[serde(rename = "type")]
    kind: ColumnType,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
enum ColumnType {
    U64,
    Number,
    String,
}

impl CsvMapping {
    pub fn from_json(json: &str) -> Result<Self, CsvError> {
        let mapping: MappingFile = serde_json::from_str(json)
            .map_err(|error| CsvError::Invalid(format!("invalid CSV mapping: {error}")))?;
        if mapping.source.trim().is_empty()
            || mapping.source.len() > 128
            || mapping.source.chars().any(char::is_control)
        {
            return Err(CsvError::Invalid(
                "mapping source must contain 1..128 bytes of non-blank text without control characters".to_owned(),
            ));
        }
        if mapping.columns.is_empty() || mapping.columns.len() > 64 {
            return Err(CsvError::Invalid(
                "CSV mapping must contain 1..64 columns".to_owned(),
            ));
        }
        let mut columns = HashSet::new();
        let mut fields = HashSet::new();
        for column in &mapping.columns {
            for name in [&column.column, &column.field] {
                if name.trim().is_empty() || name.len() > 128 || name.chars().any(char::is_control)
                {
                    return Err(CsvError::Invalid(
                        "mapping names must contain 1..128 bytes without control characters"
                            .to_owned(),
                    ));
                }
            }
            if !columns.insert(&column.column) || !fields.insert(&column.field) {
                return Err(CsvError::Invalid(
                    "mapping column names and output fields must be unique".to_owned(),
                ));
            }
        }
        if !mapping.columns.iter().any(|column| {
            column.column == mapping.sequence_column && column.kind == ColumnType::U64
        }) {
            return Err(CsvError::Invalid(
                "sequence_column must name a mapped u64 column".to_owned(),
            ));
        }
        if let Some(name) = &mapping.time_column
            && !mapping
                .columns
                .iter()
                .any(|column| column.column == *name && column.kind == ColumnType::String)
        {
            return Err(CsvError::Invalid(
                "time_column must name a mapped string column containing HH:MM:SS[.fraction]"
                    .to_owned(),
            ));
        }
        Ok(Self(mapping))
    }

    fn parse(&self, body: &str) -> Result<Vec<ParsedSample>, CsvError> {
        let mut samples = Vec::new();
        let mut found_content = false;
        let mut reader = csv::ReaderBuilder::new()
            .has_headers(false)
            .from_reader(body.as_bytes());
        for (index, record) in reader.records().enumerate() {
            let invalid =
                |message: String| CsvError::Invalid(format!("CSV record {}: {message}", index + 1));
            let record = record.map_err(|error| invalid(error.to_string()))?;
            found_content = true;
            if record.len() != self.0.columns.len() {
                return Err(invalid(format!(
                    "expected {} CSV fields, got {}",
                    self.0.columns.len(),
                    record.len()
                )));
            }
            if record
                .iter()
                .zip(&self.0.columns)
                .all(|(value, column)| value == column.column)
            {
                continue;
            }
            let mut values = Map::new();
            let mut sequence = None;
            let mut time_of_day_ns = None;
            for (value, column) in record.iter().zip(&self.0.columns) {
                let error = || {
                    invalid(format!(
                        "invalid {:?} value for {}",
                        column.kind, column.column
                    ))
                };
                if self.0.time_column.as_deref() == Some(column.column.as_str()) {
                    time_of_day_ns = Some(parse_time_of_day(value).ok_or_else(|| {
                        invalid(format!(
                            "invalid HH:MM:SS[.fraction] value for {}",
                            column.column
                        ))
                    })?);
                }
                let value = match column.kind {
                    ColumnType::U64 => {
                        let number = value.trim().parse::<u64>().map_err(|_| error())?;
                        if column.column == self.0.sequence_column {
                            sequence = Some(number);
                        }
                        Value::from(number)
                    }
                    ColumnType::Number => {
                        let number = value.trim().parse::<f64>().map_err(|_| error())?;
                        if !number.is_finite() {
                            return Err(error());
                        }
                        json!(number)
                    }
                    ColumnType::String => Value::String(value.to_owned()),
                };
                values.insert(column.field.clone(), value);
            }
            samples.push(ParsedSample {
                sequence: sequence.expect("validated mapping has a u64 sequence column"),
                time_of_day_ns,
                values,
            });
        }
        if !found_content {
            return Err(CsvError::Invalid("CSV body must not be empty".to_owned()));
        }
        Ok(samples)
    }
}

struct ParsedSample {
    sequence: u64,
    time_of_day_ns: Option<u64>,
    values: Map<String, Value>,
}

#[derive(Clone, Debug, Serialize)]
pub(crate) struct CsvReading {
    source: String,
    filename: String,
    received_time_ns: u64,
    sample_time_ns: Option<u64>,
    time_quality: CsvTimeQuality,
    clock_epoch: u64,
    sequence: u64,
    source_sequence: u64,
    values: Map<String, Value>,
}

#[derive(Debug, Serialize)]
pub(crate) struct CsvBatchResult {
    accepted: usize,
    duplicates: usize,
    cursor: String,
}

pub(crate) struct CsvStore {
    journal: JournalHandle,
    mapping: CsvMapping,
    state: Mutex<CsvState>,
}

#[derive(Default)]
struct CsvState {
    rows_received: u64,
    readings: VecDeque<CsvReading>,
    clocks: HashMap<String, CsvClock>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum CsvTimeQuality {
    Estimated,
    ClockReset,
    OutOfOrder,
    Unavailable,
}

struct CsvTiming {
    sample_time_ns: Option<u64>,
    quality: CsvTimeQuality,
    epoch: u64,
}

#[derive(Clone)]
struct CsvClock {
    sequence: u64,
    time_of_day_ns: u64,
    epoch: u64,
    reset_sequence: Option<u64>,
}

fn parse_time_of_day(value: &str) -> Option<u64> {
    let (whole, fraction) = value.trim().split_once('.').unwrap_or((value.trim(), ""));
    let mut parts = whole.split(':');
    let mut component = || {
        let part = parts.next()?;
        (part.len() == 2 && part.bytes().all(|byte| byte.is_ascii_digit()))
            .then(|| part.parse::<u64>().ok())
            .flatten()
    };
    let hour = component()?;
    let minute = component()?;
    let second = component()?;
    if parts.next().is_some()
        || hour >= 24
        || minute >= 60
        || second >= 60
        || fraction.len() > 9
        || !fraction.bytes().all(|byte| byte.is_ascii_digit())
        || (value.trim().contains('.') && fraction.is_empty())
    {
        return None;
    }
    let fractional_ns = if fraction.is_empty() {
        0
    } else {
        fraction.parse::<u64>().ok()? * 10_u64.pow(9 - fraction.len() as u32)
    };
    Some((hour * 3600 + minute * 60 + second) * SECOND_NS + fractional_ns)
}

fn clock_delta(current: u64, previous: u64) -> i128 {
    (i128::from(current) - i128::from(previous) + DAY_NS / 2).rem_euclid(DAY_NS) - DAY_NS / 2
}

impl CsvClock {
    fn estimate(
        previous: Option<&Self>,
        sequence: u64,
        time_of_day_ns: u64,
        batch_anchor_ns: u64,
        received_time_ns: u64,
    ) -> Result<(CsvTiming, Self), CsvError> {
        let delta = previous
            .map(|clock| clock_delta(time_of_day_ns, clock.time_of_day_ns))
            .unwrap_or(0);
        let sequence_restarted =
            previous.is_some_and(|clock| sequence < clock.sequence) && delta >= 0;
        let out_of_order =
            previous.is_some_and(|clock| sequence < clock.sequence) && !sequence_restarted;
        let age_ns = clock_delta(batch_anchor_ns, time_of_day_ns);
        let reset = !out_of_order
            && (sequence_restarted || age_ns < 0 || (previous.is_some() && delta < 0));
        let mut clock = match previous {
            Some(clock) if !reset => clock.clone(),
            _ => Self {
                sequence,
                time_of_day_ns,
                epoch: previous.map_or(1, |clock| clock.epoch + 1),
                reset_sequence: reset.then_some(sequence),
            },
        };
        let quality = if out_of_order {
            CsvTimeQuality::OutOfOrder
        } else if reset {
            CsvTimeQuality::ClockReset
        } else {
            CsvTimeQuality::Estimated
        };
        let ambiguous = out_of_order && clock.reset_sequence.is_some_and(|start| sequence < start);
        // Each periodic batch gets a fresh receipt anchor; the ESP's absolute clock is not trusted.
        let sample_time_ns = if ambiguous || age_ns < 0 {
            None
        } else {
            Some(
                u64::try_from(i128::from(received_time_ns) - age_ns).map_err(|_| {
                    CsvError::Invalid("estimated CSV sample timestamp is out of range".to_owned())
                })?,
            )
        };
        if !out_of_order {
            clock.sequence = sequence;
            clock.time_of_day_ns = time_of_day_ns;
        }
        Ok((
            CsvTiming {
                sample_time_ns,
                quality,
                epoch: clock.epoch,
            },
            clock,
        ))
    }
}

impl CsvStore {
    pub(crate) fn new(journal: JournalHandle, mapping: CsvMapping) -> Self {
        Self {
            journal,
            mapping,
            state: Mutex::new(CsvState::default()),
        }
    }

    pub(crate) async fn status(&self) -> Value {
        let state = self.state.lock().await;
        let columns: Vec<_> = self
            .mapping
            .0
            .columns
            .iter()
            .map(|column| &column.field)
            .collect();
        json!({
            "available": true,
            "columns": columns,
            "rows_received": state.rows_received,
            "latest": state.readings.back(),
            "recent": &state.readings,
        })
    }

    pub(crate) async fn ingest(
        &self,
        filename: &str,
        body: &str,
    ) -> Result<CsvBatchResult, CsvError> {
        if filename.is_empty()
            || filename.len() > 255
            || !filename
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || b"._-".contains(&byte))
        {
            return Err(CsvError::Invalid(
                "filename must contain 1..255 ASCII letters, digits, dots, underscores or hyphens"
                    .to_owned(),
            ));
        }
        let samples = self.mapping.parse(body)?;
        let received_time_ns = wall_time_ns();
        let batch_anchor_ns = samples.last().and_then(|sample| sample.time_of_day_ns);
        let mut state = self.state.lock().await;
        let mut result = CsvBatchResult {
            accepted: 0,
            duplicates: 0,
            cursor: format_cursor(self.journal.tail()),
        };
        for sample in samples {
            let (timing, clock) = match (sample.time_of_day_ns, batch_anchor_ns) {
                (Some(time), Some(anchor)) => {
                    let (timing, clock) = CsvClock::estimate(
                        state.clocks.get(filename),
                        sample.sequence,
                        time,
                        anchor,
                        received_time_ns,
                    )?;
                    (timing, Some(clock))
                }
                _ => (
                    CsvTiming {
                        sample_time_ns: None,
                        quality: CsvTimeQuality::Unavailable,
                        epoch: 0,
                    },
                    None,
                ),
            };
            let mut fields = Map::new();
            fields.insert("filename".to_owned(), Value::String(filename.to_owned()));
            fields.insert("values".to_owned(), Value::Object(sample.values.clone()));
            fields.insert("sample_time_ns".to_owned(), json!(timing.sample_time_ns));
            fields.insert("time_quality".to_owned(), json!(timing.quality));
            fields.insert("clock_epoch".to_owned(), json!(timing.epoch));
            let appended = self
                .journal
                .append_diagnostic(DiagnosticEvent {
                    schema_version: crate::records::SCHEMA_VERSION,
                    run_id: self.journal.run_id(),
                    source: self.mapping.0.source.clone(),
                    source_instance: format!("csv:{}:{filename}", self.mapping.0.source),
                    source_sequence: sample.sequence,
                    source_wall_time_ns: received_time_ns,
                    source_monotonic_ns: None,
                    sim_time_ns: None,
                    sim_time_quality: None,
                    level: if timing.quality == CsvTimeQuality::ClockReset {
                        DiagnosticLevel::Warning
                    } else {
                        DiagnosticLevel::Info
                    },
                    category: "csv".to_owned(),
                    event: "csv.sample".to_owned(),
                    message: "CSV sample".to_owned(),
                    correlation_id: None,
                    causation_id: None,
                    fields,
                    related_records: Vec::new(),
                })
                .await?;
            if appended.appended {
                if timing.quality == CsvTimeQuality::ClockReset {
                    tracing::warn!(
                        filename,
                        source_sequence = sample.sequence,
                        "CSV clock discontinuity: starting a new estimated-time epoch"
                    );
                }
                if let Some(clock) = clock {
                    state.clocks.insert(filename.to_owned(), clock);
                }
                result.accepted += 1;
                state.rows_received += 1;
                state.readings.push_back(CsvReading {
                    source: self.mapping.0.source.clone(),
                    filename: filename.to_owned(),
                    received_time_ns,
                    sample_time_ns: timing.sample_time_ns,
                    time_quality: timing.quality,
                    clock_epoch: timing.epoch,
                    sequence: appended.sequence,
                    source_sequence: sample.sequence,
                    values: sample.values,
                });
                if state.readings.len() > RECENT_ROWS {
                    state.readings.pop_front();
                }
            } else {
                result.duplicates += 1;
            }
            result.cursor = format_cursor(self.journal.tail());
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_loadcell_mapping_preserves_counts_modes_and_absolute_time() {
        let mapping = CsvMapping::from_json(include_str!("../csv-map.json")).unwrap();
        let samples = mapping
            .parse(concat!(
                "_rownum,_time,_time_unix_us,input,raw,gain\n",
                "1,00:00:00.123,1735689600123456,A,-8388608,128\n",
                "2,00:00:00.123,1735689600123456,B,8388607,128\n",
                "3,00:00:00.223,1735689600223456,A1,17,64\n",
                "4,00:00:00.223,1735689600223456,B1,-42,64\n",
                "5,00:00:00.823,1735689600823456,A2,18,32\n",
                "6,00:00:00.823,1735689600823456,B2,-43,32\n",
            ))
            .unwrap();
        assert_eq!(samples.len(), 6);
        assert_eq!(samples[0].values["raw"], json!(-8388608.0));
        assert_eq!(samples[1].values["raw"], json!(8388607.0));
        assert_eq!(
            samples[0].values["device_time_unix_us"],
            json!(1735689600123456_u64)
        );
        assert_eq!(samples[0].time_of_day_ns, Some(123_000_000));
        assert_eq!(samples[1].sequence, 2);
        for (sample, label, gain) in [
            (&samples[0], "A", 128),
            (&samples[1], "B", 128),
            (&samples[2], "A1", 64),
            (&samples[3], "B1", 64),
            (&samples[4], "A2", 32),
            (&samples[5], "B2", 32),
        ] {
            assert_eq!(sample.values["input"], json!(label));
            assert_eq!(sample.values["gain"], json!(gain));
        }
    }

    #[test]
    fn csv_clock_preserves_spacing_and_reanchors_every_periodic_batch() {
        let anchor = parse_time_of_day("12:00:00.400").unwrap();
        let (first, clock) = CsvClock::estimate(
            None,
            1,
            parse_time_of_day("12:00:00.100").unwrap(),
            anchor,
            100 * SECOND_NS,
        )
        .unwrap();
        assert_eq!(first.sample_time_ns, Some(100 * SECOND_NS - 300_000_000));
        let (last, clock) =
            CsvClock::estimate(Some(&clock), 2, anchor, anchor, 100 * SECOND_NS).unwrap();
        assert_eq!(last.sample_time_ns, Some(100 * SECOND_NS));
        assert_eq!(last.quality, CsvTimeQuality::Estimated);

        let next_anchor = parse_time_of_day("12:00:00.900").unwrap();
        let (next, _) = CsvClock::estimate(
            Some(&clock),
            3,
            parse_time_of_day("12:00:00.500").unwrap(),
            next_anchor,
            101 * SECOND_NS,
        )
        .unwrap();
        assert_eq!(next.sample_time_ns, Some(101 * SECOND_NS - 400_000_000));
        assert_eq!(next.epoch, 1);
    }

    #[test]
    fn csv_clock_distinguishes_midnight_resets_and_delayed_rows() {
        let anchor = parse_time_of_day("00:00:00.100").unwrap();
        let (first, clock) = CsvClock::estimate(
            None,
            1,
            parse_time_of_day("23:59:59.900").unwrap(),
            anchor,
            100 * SECOND_NS,
        )
        .unwrap();
        assert_eq!(first.sample_time_ns, Some(100 * SECOND_NS - 200_000_000));
        let (last, clock) =
            CsvClock::estimate(Some(&clock), 2, anchor, anchor, 100 * SECOND_NS).unwrap();
        assert_eq!(last.epoch, 1);
        let reset_anchor = parse_time_of_day("00:00:00.050").unwrap();
        let (reset, clock) =
            CsvClock::estimate(Some(&clock), 3, reset_anchor, reset_anchor, 101 * SECOND_NS)
                .unwrap();
        assert_eq!(reset.quality, CsvTimeQuality::ClockReset);
        assert_eq!(reset.epoch, 2);
        assert_eq!(reset.sample_time_ns, Some(101 * SECOND_NS));
        let (delayed, unchanged) = CsvClock::estimate(
            Some(&clock),
            1,
            parse_time_of_day("23:59:59.900").unwrap(),
            reset_anchor,
            102 * SECOND_NS,
        )
        .unwrap();
        assert_eq!(delayed.quality, CsvTimeQuality::OutOfOrder);
        assert_eq!(delayed.sample_time_ns, None);
        assert_eq!(unchanged.sequence, 3);
    }

    #[test]
    fn csv_clock_does_not_invent_future_acquisition_times() {
        let (timing, _) = CsvClock::estimate(
            None,
            1,
            parse_time_of_day("12:00:00.900").unwrap(),
            parse_time_of_day("12:00:00.100").unwrap(),
            100 * SECOND_NS,
        )
        .unwrap();
        assert_eq!(timing.sample_time_ns, None);
        assert_eq!(timing.quality, CsvTimeQuality::ClockReset);
    }

    #[test]
    fn csv_clock_handles_counter_restart_inside_a_periodic_batch() {
        let anchor = parse_time_of_day("12:00:00.300").unwrap();
        let (first, clock) = CsvClock::estimate(
            None,
            19000,
            parse_time_of_day("12:00:00.100").unwrap(),
            anchor,
            100 * SECOND_NS,
        )
        .unwrap();
        let (restart, clock) = CsvClock::estimate(
            Some(&clock),
            1,
            parse_time_of_day("12:00:00.200").unwrap(),
            anchor,
            100 * SECOND_NS,
        )
        .unwrap();
        let (last, _) =
            CsvClock::estimate(Some(&clock), 2, anchor, anchor, 100 * SECOND_NS).unwrap();
        assert_eq!(first.sample_time_ns, Some(100 * SECOND_NS - 200_000_000));
        assert_eq!(restart.sample_time_ns, Some(100 * SECOND_NS - 100_000_000));
        assert_eq!(restart.quality, CsvTimeQuality::ClockReset);
        assert_eq!(restart.epoch, 2);
        assert_eq!(last.sample_time_ns, Some(100 * SECOND_NS));
        assert_eq!(last.epoch, 2);
    }

    #[test]
    fn csv_time_parser_accepts_fractional_precision_and_rejects_invalid_clocks() {
        assert_eq!(parse_time_of_day("00:00:00.123456789"), Some(123_456_789));
        assert_eq!(parse_time_of_day("23:59:59"), Some(86_399 * SECOND_NS));
        for value in [
            "24:00:00",
            "12:60:00",
            "12:00:60",
            "12:00:00.",
            "1:00:00",
            "12:00:00.1234567890",
            "12:00:00.x",
            "not a clock",
        ] {
            assert_eq!(parse_time_of_day(value), None, "{value}");
        }
        let invalid = r#"{"source":"test","sequence_column":"n","time_column":"n",
            "columns":[{"column":"n","field":"n","type":"u64"}]}"#;
        assert!(CsvMapping::from_json(invalid).is_err());
    }

    #[test]
    fn csv_mapping_changes_names_types_and_sequence_position_without_parser_changes() {
        let mapping = CsvMapping::from_json(
            r#"{
                "source": "bench.temperature",
                "sequence_column": "sample",
                "columns": [
                    {"column": "temperature", "field": "temperature_c", "type": "number"},
                    {"column": "sample", "field": "index", "type": "u64"},
                    {"column": "label", "field": "sensor", "type": "string"}
                ]
            }"#,
        )
        .unwrap();
        let samples = mapping
            .parse("temperature,sample,label\r\n -12.5 ,42,\" bench, \"\"left\"\"\n \"\r\n")
            .unwrap();
        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0].sequence, 42);
        assert_eq!(samples[0].values["temperature_c"], -12.5);
        assert_eq!(samples[0].values["index"], 42);
        assert_eq!(samples[0].values["sensor"], " bench, \"left\"\n ");
        assert!(mapping.parse("inf,43,bench").is_err());
    }

    #[test]
    fn csv_mapping_rejects_ambiguous_columns_and_invalid_sequence_roles() {
        for mapping in [
            json!({
                "source": "test",
                "sequence_column": "n",
                "columns": [{"column": "n", "field": "n", "type": "number"}]
            }),
            json!({
                "source": "test",
                "sequence_column": "missing",
                "columns": [{"column": "n", "field": "n", "type": "u64"}]
            }),
            json!({
                "source": "test",
                "sequence_column": "n",
                "columns": [
                    {"column": "n", "field": "value", "type": "u64"},
                    {"column": "lift", "field": "value", "type": "number"}
                ]
            }),
            json!({
                "source": "test",
                "sequence_column": "n",
                "columns": [
                    {"column": "n", "field": "n", "type": "u64"},
                    {"column": "n", "field": "other", "type": "number"}
                ]
            }),
        ] {
            assert!(CsvMapping::from_json(&mapping.to_string()).is_err());
        }
    }
}
