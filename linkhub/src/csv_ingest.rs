use std::collections::{HashSet, VecDeque};

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value, json};
use tokio::sync::Mutex;

use crate::{
    journal::{JournalError, JournalHandle},
    records::{DiagnosticEvent, DiagnosticLevel, format_cursor, wall_time_ns},
};

pub(crate) const MAX_CSV_BYTES: usize = 64 * 1024;
const RECENT_ROWS: usize = 5;

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
            for (value, column) in record.iter().zip(&self.0.columns) {
                let error = || {
                    invalid(format!(
                        "invalid {:?} value for {}",
                        column.kind, column.column
                    ))
                };
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
    values: Map<String, Value>,
}

#[derive(Clone, Debug, Serialize)]
pub(crate) struct CsvReading {
    source: String,
    filename: String,
    received_time_ns: u64,
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
    readings: Mutex<VecDeque<CsvReading>>,
}

impl CsvStore {
    pub(crate) fn new(journal: JournalHandle, mapping: CsvMapping) -> Self {
        Self {
            journal,
            mapping,
            readings: Mutex::new(VecDeque::new()),
        }
    }

    pub(crate) async fn status(&self) -> Value {
        let readings = self.readings.lock().await;
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
            "latest": readings.back(),
            "recent": &*readings,
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
        let mut readings = self.readings.lock().await;
        let mut result = CsvBatchResult {
            accepted: 0,
            duplicates: 0,
            cursor: format_cursor(self.journal.tail()),
        };
        for sample in samples {
            let received_time_ns = wall_time_ns();
            let mut fields = Map::new();
            fields.insert("filename".to_owned(), Value::String(filename.to_owned()));
            fields.insert("values".to_owned(), Value::Object(sample.values.clone()));
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
                    level: DiagnosticLevel::Info,
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
                result.accepted += 1;
                readings.push_back(CsvReading {
                    source: self.mapping.0.source.clone(),
                    filename: filename.to_owned(),
                    received_time_ns,
                    sequence: appended.sequence,
                    source_sequence: sample.sequence,
                    values: sample.values,
                });
                if readings.len() > RECENT_ROWS {
                    readings.pop_front();
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
