use std::{
    cmp::Ordering,
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use clap::{Args, Subcommand, ValueEnum};
use serde_json::{Map, Value, json};

use crate::{
    journal::{JournalError, read_persisted},
    records::{Direction, JournalRecord, RecordPayload},
};

#[derive(Clone, Copy, Debug, ValueEnum)]
pub enum QueryDirection {
    Rx,
    Tx,
}

#[derive(Debug, Args)]
pub struct QueryArgs {
    #[arg(value_name = "JOURNAL")]
    journal: PathBuf,
    #[arg(long, default_value_t = 0)]
    after: u64,
    #[arg(long)]
    through: Option<u64>,
    #[command(subcommand)]
    command: QueryCommand,
}

#[derive(Debug, Subcommand)]
enum QueryCommand {
    Types,
    Show {
        #[command(flatten)]
        filters: Filters,
        #[arg(long)]
        fields: Option<String>,
        #[arg(long)]
        limit: Option<usize>,
        #[arg(long)]
        json: bool,
    },
    Count {
        #[command(flatten)]
        filters: Filters,
        #[arg(long)]
        by: Option<String>,
    },
    Stats {
        #[command(flatten)]
        filters: Filters,
        #[arg(long)]
        field: String,
    },
    Armed,
    Statustext {
        #[arg(long)]
        since: Option<f64>,
        #[arg(long)]
        until: Option<f64>,
        #[arg(long)]
        min_severity: Option<u64>,
    },
    Nvf {
        #[arg(long)]
        name: Option<String>,
        #[arg(long = "dir")]
        direction: Option<QueryDirection>,
        #[arg(long)]
        since: Option<f64>,
        #[arg(long)]
        until: Option<f64>,
    },
    Param {
        #[arg(long)]
        id: Option<String>,
        #[arg(long)]
        since: Option<f64>,
        #[arg(long)]
        until: Option<f64>,
    },
    Diagnostics {
        #[arg(long)]
        source: Option<String>,
        #[arg(long)]
        event: Option<String>,
        #[arg(long)]
        level: Option<String>,
        #[arg(long)]
        contains: Option<String>,
        #[arg(long)]
        limit: Option<usize>,
        #[arg(long)]
        json: bool,
    },
}

#[derive(Debug, Args, Default)]
struct Filters {
    #[arg(short = 't', long = "type")]
    message_types: Vec<String>,
    #[arg(long = "dir")]
    direction: Option<QueryDirection>,
    #[arg(long)]
    since: Option<f64>,
    #[arg(long)]
    until: Option<f64>,
    #[arg(long = "eq")]
    equal: Vec<String>,
    #[arg(long)]
    contains: Option<String>,
}

#[derive(Debug, thiserror::Error)]
pub enum QueryError {
    #[error(transparent)]
    Journal(#[from] JournalError),
    #[error("invalid journal path: {0}")]
    InvalidJournal(String),
    #[error("invalid query: {0}")]
    InvalidQuery(String),
}

struct Message {
    relative_s: f64,
    direction: Direction,
    message_type: String,
    fields: Map<String, Value>,
    sequence: u64,
    ingest_time_ns: u64,
    sim_epoch: u64,
    time_boot_ms: Option<u64>,
    time_quality: Option<String>,
}

pub async fn execute(args: QueryArgs) -> Result<(), QueryError> {
    let journal = resolve_journal(&args.journal)?;
    let records = read_persisted(&journal, args.after, args.through).await?;
    let messages = project_messages(&records);
    match args.command {
        QueryCommand::Types => print_types(&messages),
        QueryCommand::Show {
            filters,
            fields,
            limit,
            json,
        } => print_show(&messages, &filters, fields.as_deref(), limit, json)?,
        QueryCommand::Count { filters, by } => {
            print_count(&messages, &filters, by.as_deref())?;
        }
        QueryCommand::Stats { filters, field } => {
            print_stats(&messages, &filters, &field)?;
        }
        QueryCommand::Armed => print_armed(&messages),
        QueryCommand::Statustext {
            since,
            until,
            min_severity,
        } => print_statustext(&messages, since, until, min_severity),
        QueryCommand::Nvf {
            name,
            direction,
            since,
            until,
        } => print_nvf(&messages, name.as_deref(), direction, since, until),
        QueryCommand::Param { id, since, until } => {
            print_params(&messages, id.as_deref(), since, until);
        }
        QueryCommand::Diagnostics {
            source,
            event,
            level,
            contains,
            limit,
            json,
        } => print_diagnostics(
            &records,
            source.as_deref(),
            event.as_deref(),
            level.as_deref(),
            contains.as_deref(),
            limit,
            json,
        ),
    }
    Ok(())
}

fn resolve_journal(path: &Path) -> Result<PathBuf, QueryError> {
    if path.is_dir() && path.join("journal").is_dir() {
        return Ok(path.join("journal"));
    }
    if path.is_dir()
        && std::fs::read_dir(path)
            .map_err(JournalError::from)?
            .flatten()
            .any(|entry| entry.path().extension().is_some_and(|ext| ext == "lhc"))
    {
        return Ok(path.to_path_buf());
    }
    let candidates: Vec<PathBuf> = std::fs::read_dir(path)
        .map_err(JournalError::from)?
        .flatten()
        .map(|entry| entry.path().join("journal"))
        .filter(|candidate| candidate.is_dir())
        .collect();
    match candidates.as_slice() {
        [journal] => Ok(journal.clone()),
        [] => Err(QueryError::InvalidJournal(format!(
            "{} contains no journal chunks",
            path.display()
        ))),
        _ => Err(QueryError::InvalidJournal(format!(
            "{} contains multiple runs; select one run directory",
            path.display()
        ))),
    }
}

fn project_messages(records: &[JournalRecord]) -> Vec<Message> {
    let first_ns = records.iter().find_map(|record| {
        matches!(record.payload, RecordPayload::MavlinkFrame(_)).then_some(record.ingest_time_ns)
    });
    let Some(first_ns) = first_ns else {
        return Vec::new();
    };
    records
        .iter()
        .filter_map(|record| {
            let RecordPayload::MavlinkFrame(frame) = &record.payload else {
                return None;
            };
            Some(Message {
                relative_s: record.ingest_time_ns.saturating_sub(first_ns) as f64 / 1e9,
                direction: frame.direction,
                message_type: frame.message_name.clone(),
                fields: frame.fields.clone(),
                sequence: record.sequence,
                ingest_time_ns: record.ingest_time_ns,
                sim_epoch: record.sim_clock.epoch,
                time_boot_ms: record.sim_clock.time_boot_ms,
                time_quality: record
                    .sim_clock
                    .quality
                    .map(|quality| format!("{quality:?}").to_lowercase()),
            })
        })
        .collect()
}

fn direction_name(direction: Direction) -> &'static str {
    match direction {
        Direction::Rx => "rx",
        Direction::Tx => "tx",
    }
}

fn message_value(message: &Message, field: &str) -> Option<Value> {
    match field {
        "mavpackettype" => Some(json!(message.message_type)),
        "_dir" => Some(json!(direction_name(message.direction))),
        "t_rel" => Some(json!(message.relative_s)),
        "_t_wall" => Some(json!(message.ingest_time_ns as f64 / 1e9)),
        "_sequence" => Some(json!(message.sequence)),
        "_ingest_time_ns" => Some(json!(message.ingest_time_ns)),
        "_sim_epoch" => Some(json!(message.sim_epoch)),
        "_sim_time_boot_ms" => Some(json!(message.time_boot_ms)),
        "_sim_time_quality" => Some(json!(message.time_quality)),
        _ => message.fields.get(field).cloned(),
    }
}

fn message_json(message: &Message) -> Value {
    let mut object = message.fields.clone();
    object.insert("_dir".to_owned(), json!(direction_name(message.direction)));
    object.insert("mavpackettype".to_owned(), json!(message.message_type));
    object.insert("t_rel".to_owned(), json!(message.relative_s));
    object.insert(
        "_t_wall".to_owned(),
        json!(message.ingest_time_ns as f64 / 1e9),
    );
    object.insert("_sequence".to_owned(), json!(message.sequence));
    object.insert("_ingest_time_ns".to_owned(), json!(message.ingest_time_ns));
    object.insert("_sim_epoch".to_owned(), json!(message.sim_epoch));
    object.insert("_sim_time_boot_ms".to_owned(), json!(message.time_boot_ms));
    object.insert("_sim_time_quality".to_owned(), json!(message.time_quality));
    Value::Object(object)
}

fn matches(message: &Message, filters: &Filters) -> Result<bool, QueryError> {
    if !filters.message_types.is_empty()
        && !filters
            .message_types
            .iter()
            .any(|name| name.eq_ignore_ascii_case(&message.message_type))
    {
        return Ok(false);
    }
    if filters.direction.is_some_and(|direction| {
        direction_name(message.direction)
            != match direction {
                QueryDirection::Rx => "rx",
                QueryDirection::Tx => "tx",
            }
    }) {
        return Ok(false);
    }
    if filters
        .since
        .is_some_and(|since| message.relative_s < since)
        || filters
            .until
            .is_some_and(|until| message.relative_s > until)
    {
        return Ok(false);
    }
    for specification in &filters.equal {
        let Some((field, expected)) = specification.split_once('=') else {
            return Err(QueryError::InvalidQuery(format!(
                "--eq expects FIELD=VALUE, got {specification:?}"
            )));
        };
        let actual = message_value(message, field);
        let expected =
            serde_json::from_str(expected).unwrap_or_else(|_| Value::String(expected.to_owned()));
        if actual.as_ref() != Some(&expected) {
            return Ok(false);
        }
    }
    if let Some(needle) = &filters.contains
        && !message_json(message)
            .to_string()
            .to_lowercase()
            .contains(&needle.to_lowercase())
    {
        return Ok(false);
    }
    Ok(true)
}

fn filtered<'a>(
    messages: &'a [Message],
    filters: &Filters,
) -> Result<Vec<&'a Message>, QueryError> {
    messages
        .iter()
        .filter_map(|message| match matches(message, filters) {
            Ok(true) => Some(Ok(message)),
            Ok(false) => None,
            Err(error) => Some(Err(error)),
        })
        .collect()
}

fn print_types(messages: &[Message]) {
    let mut groups: BTreeMap<(&str, &str), Vec<f64>> = BTreeMap::new();
    for message in messages {
        groups
            .entry((
                message.message_type.as_str(),
                direction_name(message.direction),
            ))
            .or_default()
            .push(message.relative_s);
    }
    println!(
        "{:<26} {:<4} {:>7} {:>10} {:>10}",
        "TYPE", "DIR", "COUNT", "FIRST_T", "LAST_T"
    );
    for ((message_type, direction), times) in groups {
        println!(
            "{message_type:<26} {direction:<4} {:>7} {:>10.3} {:>10.3}",
            times.len(),
            times[0],
            times[times.len() - 1]
        );
    }
    if let Some(last) = messages.last() {
        println!(
            "\nTotal messages: {}   duration: {:.3}s",
            messages.len(),
            last.relative_s
        );
    } else {
        println!("empty log");
    }
}

fn print_show(
    messages: &[Message],
    filters: &Filters,
    fields: Option<&str>,
    limit: Option<usize>,
    as_json: bool,
) -> Result<(), QueryError> {
    let rows = filtered(messages, filters)?;
    let fields: Option<Vec<&str>> = fields.map(|value| value.split(',').map(str::trim).collect());
    for message in rows.iter().take(limit.unwrap_or(usize::MAX)) {
        if as_json {
            println!("{}", message_json(message));
            continue;
        }
        let values = fields.as_ref().map_or_else(
            || {
                message
                    .fields
                    .iter()
                    .map(|(name, value)| format!("{name}={value}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            },
            |fields| {
                fields
                    .iter()
                    .map(|field| {
                        format!(
                            "{field}={}",
                            message_value(message, field).unwrap_or(Value::Null)
                        )
                    })
                    .collect::<Vec<_>>()
                    .join(" ")
            },
        );
        println!(
            "[{:>9.3}s] {:<3} {:<24} {}",
            message.relative_s,
            direction_name(message.direction),
            message.message_type,
            values
        );
    }
    eprintln!("{} message(s)", rows.len().min(limit.unwrap_or(usize::MAX)));
    Ok(())
}

fn print_count(
    messages: &[Message],
    filters: &Filters,
    by: Option<&str>,
) -> Result<(), QueryError> {
    let rows = filtered(messages, filters)?;
    let Some(by) = by else {
        println!("{}", rows.len());
        return Ok(());
    };
    let mut groups: BTreeMap<String, usize> = BTreeMap::new();
    for message in rows {
        let value = message_value(message, by).unwrap_or(Value::Null);
        *groups.entry(value.to_string()).or_default() += 1;
    }
    let mut groups: Vec<_> = groups.into_iter().collect();
    groups.sort_by(|left, right| right.1.cmp(&left.1).then(left.0.cmp(&right.0)));
    for (value, count) in groups {
        println!("{value:<30} {count}");
    }
    Ok(())
}

fn print_stats(messages: &[Message], filters: &Filters, field: &str) -> Result<(), QueryError> {
    let rows = filtered(messages, filters)?;
    let mut values: Vec<f64> = rows
        .iter()
        .filter_map(|message| message_value(message, field)?.as_f64())
        .collect();
    if values.is_empty() {
        println!("No numeric values for field {field:?}");
        return Ok(());
    }
    values.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
    let count = values.len();
    let mean = values.iter().sum::<f64>() / count as f64;
    let median = if count.is_multiple_of(2) {
        (values[count / 2 - 1] + values[count / 2]) / 2.0
    } else {
        values[count / 2]
    };
    let variance = values
        .iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f64>()
        / count as f64;
    println!("field={field}  n={count}");
    println!(
        "  min={:.6}  max={:.6}  mean={mean:.6}",
        values[0],
        values[count - 1]
    );
    println!("  median={median:.6}  stdev={:.6}", variance.sqrt());
    Ok(())
}

fn print_armed(messages: &[Message]) {
    let mut previous: Option<(bool, u64)> = None;
    for message in messages
        .iter()
        .filter(|message| message.direction == Direction::Rx && message.message_type == "HEARTBEAT")
    {
        let base_mode = message
            .fields
            .get("base_mode")
            .and_then(Value::as_u64)
            .unwrap_or_default();
        let custom_mode = message
            .fields
            .get("custom_mode")
            .and_then(Value::as_u64)
            .unwrap_or_default();
        let armed = base_mode & 128 != 0;
        if previous == Some((armed, custom_mode)) {
            continue;
        }
        println!(
            "[{:>9.3}s] armed={:<5} mode={}",
            message.relative_s,
            armed,
            copter_mode(custom_mode)
        );
        previous = Some((armed, custom_mode));
    }
}

fn copter_mode(mode: u64) -> String {
    match mode {
        0 => "STABILIZE".to_owned(),
        1 => "ACRO".to_owned(),
        2 => "ALT_HOLD".to_owned(),
        3 => "AUTO".to_owned(),
        4 => "GUIDED".to_owned(),
        5 => "LOITER".to_owned(),
        6 => "RTL".to_owned(),
        9 => "LAND".to_owned(),
        16 => "POSHOLD".to_owned(),
        20 => "GUIDED_NOGPS".to_owned(),
        _ => format!("mode{mode}"),
    }
}

fn in_window(message: &Message, since: Option<f64>, until: Option<f64>) -> bool {
    !since.is_some_and(|value| message.relative_s < value)
        && !until.is_some_and(|value| message.relative_s > value)
}

fn clean_string(value: Option<&Value>) -> String {
    value
        .and_then(Value::as_str)
        .unwrap_or_default()
        .split('\0')
        .next()
        .unwrap_or_default()
        .to_owned()
}

fn print_statustext(
    messages: &[Message],
    since: Option<f64>,
    until: Option<f64>,
    minimum_severity: Option<u64>,
) {
    let mut pending: Option<(u64, f64, u64, BTreeMap<u64, String>)> = None;
    let flush = |pending: &mut Option<(u64, f64, u64, BTreeMap<u64, String>)>| {
        if let Some((_, time, severity, chunks)) = pending.take() {
            let text = chunks.into_values().collect::<String>();
            if minimum_severity.is_none_or(|limit| severity <= limit) {
                println!("[{time:>9.3}s] {:<9} {text}", severity_name(severity));
            }
        }
    };
    for message in messages.iter().filter(|message| {
        message.direction == Direction::Rx
            && message.message_type == "STATUSTEXT"
            && in_window(message, since, until)
    }) {
        let id = message
            .fields
            .get("id")
            .and_then(Value::as_u64)
            .unwrap_or(0);
        let severity = message
            .fields
            .get("severity")
            .and_then(Value::as_u64)
            .unwrap_or(u64::MAX);
        let text = clean_string(message.fields.get("text"));
        if id == 0 {
            flush(&mut pending);
            if minimum_severity.is_none_or(|limit| severity <= limit) {
                println!(
                    "[{:>9.3}s] {:<9} {text}",
                    message.relative_s,
                    severity_name(severity)
                );
            }
            continue;
        }
        if pending.as_ref().is_none_or(|current| current.0 != id) {
            flush(&mut pending);
            pending = Some((id, message.relative_s, severity, BTreeMap::new()));
        }
        if let Some((_, _, _, chunks)) = &mut pending {
            let sequence = message
                .fields
                .get("chunk_seq")
                .and_then(Value::as_u64)
                .unwrap_or_default();
            chunks.insert(sequence, text);
        }
    }
    flush(&mut pending);
}

fn severity_name(severity: u64) -> &'static str {
    match severity {
        0 => "EMERGENCY",
        1 => "ALERT",
        2 => "CRITICAL",
        3 => "ERROR",
        4 => "WARNING",
        5 => "NOTICE",
        6 => "INFO",
        7 => "DEBUG",
        _ => "UNKNOWN",
    }
}

fn print_nvf(
    messages: &[Message],
    name: Option<&str>,
    direction: Option<QueryDirection>,
    since: Option<f64>,
    until: Option<f64>,
) {
    for message in messages.iter().filter(|message| {
        message.message_type == "NAMED_VALUE_FLOAT"
            && in_window(message, since, until)
            && direction.is_none_or(|expected| {
                matches!(
                    (expected, message.direction),
                    (QueryDirection::Rx, Direction::Rx) | (QueryDirection::Tx, Direction::Tx)
                )
            })
    }) {
        let actual_name = clean_string(message.fields.get("name"));
        if name.is_some_and(|expected| expected != actual_name) {
            continue;
        }
        println!(
            "[{:>9.3}s] {actual_name:<12} = {}",
            message.relative_s,
            message.fields.get("value").unwrap_or(&Value::Null)
        );
    }
}

fn print_params(messages: &[Message], id: Option<&str>, since: Option<f64>, until: Option<f64>) {
    for message in messages.iter().filter(|message| {
        matches!(message.message_type.as_str(), "PARAM_SET" | "PARAM_VALUE")
            && in_window(message, since, until)
    }) {
        let parameter_id = clean_string(message.fields.get("param_id"));
        if id.is_some_and(|expected| expected != parameter_id) {
            continue;
        }
        println!(
            "[{:>9.3}s] {:<3} {:<12} {parameter_id:<16} = {}",
            message.relative_s,
            direction_name(message.direction),
            message.message_type,
            message.fields.get("param_value").unwrap_or(&Value::Null)
        );
    }
}

#[allow(clippy::too_many_arguments)]
fn print_diagnostics(
    records: &[JournalRecord],
    source: Option<&str>,
    event_name: Option<&str>,
    level: Option<&str>,
    contains: Option<&str>,
    limit: Option<usize>,
    as_json: bool,
) {
    let first_ns = records.first().map_or(0, |record| record.ingest_time_ns);
    let needle = contains.map(str::to_lowercase);
    let mut printed = 0;
    for record in records {
        let RecordPayload::Diagnostic(event) = &record.payload else {
            continue;
        };
        let level_name = format!("{:?}", event.level).to_lowercase();
        if source.is_some_and(|value| value != event.source)
            || event_name.is_some_and(|value| value != event.event)
            || level.is_some_and(|value| !value.eq_ignore_ascii_case(&level_name))
            || needle.as_ref().is_some_and(|value| {
                !serde_json::to_string(event)
                    .unwrap_or_default()
                    .to_lowercase()
                    .contains(value)
            })
        {
            continue;
        }
        if as_json {
            println!("{}", serde_json::to_string(record).unwrap_or_default());
        } else {
            let relative_s = record.ingest_time_ns.saturating_sub(first_ns) as f64 / 1e9;
            println!(
                "[{relative_s:>9.3}s] {:<8} {:<24} {:<30} {}",
                level_name.to_uppercase(),
                event.source,
                event.event,
                event.message
            );
        }
        printed += 1;
        if printed == limit.unwrap_or(usize::MAX) {
            break;
        }
    }
    eprintln!("{printed} diagnostic event(s)");
}
