//! Link lifecycle diagnostics, journaled as `DiagnosticEvent`s with
//! `source = "linkhub.link"` (`linkhub query <dir> diagnostics --source
//! linkhub.link`) and mirrored to the process log.

use std::{
    collections::BTreeMap,
    sync::atomic::{AtomicU64, Ordering},
};

use serde::Serialize;
use serde_json::Value;

use crate::{
    discovery::ScanReport,
    journal::JournalHandle,
    records::{DiagnosticEvent, DiagnosticLevel, wall_time_ns},
};

/// Link lifecycle diagnostic. The variant's fields become the journaled
/// `DiagnosticEvent.fields` object.
#[derive(Debug, Serialize)]
#[serde(untagged)]
pub(crate) enum LinkEvent {
    Discovered {
        port: String,
        baud: u32,
        scan: ScanReport,
    },
    ScanFailed {
        error: String,
        repeat_count: u64,
        scan: ScanReport,
    },
    Opened {
        target: String,
    },
    OpenFailed {
        target: String,
        error: String,
        error_kind: String,
        repeat_count: u64,
    },
    Ready {
        target: String,
        ms_to_first_heartbeat: u64,
        target_system: u8,
        target_component: u8,
    },
    /// A ready session failed.
    Lost {
        target: String,
        error: String,
        error_kind: String,
        connected_ms: u64,
        received_messages: u64,
        ms_since_last_received: Option<u64>,
    },
    /// A session closed before any vehicle heartbeat.
    AcquireFailed {
        target: String,
        error: String,
        error_kind: String,
        connected_ms: u64,
        received_messages: u64,
        repeat_count: u64,
    },
}

impl LinkEvent {
    pub(crate) const fn name(&self) -> &'static str {
        match self {
            Self::Discovered { .. } => "link.discovered",
            Self::ScanFailed { .. } => "link.scan_failed",
            Self::Opened { .. } => "link.opened",
            Self::OpenFailed { .. } => "link.open_failed",
            Self::Ready { .. } => "link.ready",
            Self::Lost { .. } => "link.lost",
            Self::AcquireFailed { .. } => "link.acquire_failed",
        }
    }

    pub(crate) const fn level(&self) -> DiagnosticLevel {
        match self {
            Self::Discovered { .. } | Self::Opened { .. } | Self::Ready { .. } => {
                DiagnosticLevel::Info
            }
            Self::ScanFailed { .. } | Self::Lost { .. } | Self::AcquireFailed { .. } => {
                DiagnosticLevel::Warning
            }
            Self::OpenFailed { .. } => DiagnosticLevel::Error,
        }
    }

    pub(crate) fn message(&self) -> String {
        match self {
            Self::Discovered { port, baud, .. } => {
                format!("MAVLink heartbeat found on {port} at {baud} baud")
            }
            Self::ScanFailed { error, .. } => error.clone(),
            Self::Opened { target } => format!("opened {target}"),
            Self::OpenFailed {
                target,
                error,
                error_kind,
                ..
            } => format!("could not open {target}: {error} ({error_kind})"),
            Self::Ready {
                target,
                target_system,
                target_component,
                ..
            } => format!("vehicle heartbeat from {target_system}/{target_component} on {target}"),
            Self::Lost { target, error, .. } => {
                format!("MAVLink connection to {target} lost: {error}")
            }
            Self::AcquireFailed { target, error, .. } => {
                format!("{target} closed before a vehicle heartbeat: {error}")
            }
        }
    }
}

/// Journals link lifecycle diagnostics (`linkhub query <dir> diagnostics
/// --source linkhub.link`) and mirrors them to the process log.
pub(crate) struct LinkEvents {
    journal: JournalHandle,
    link_id: String,
    sequence: AtomicU64,
}

impl LinkEvents {
    pub(crate) fn new(journal: JournalHandle, link_id: &str) -> Self {
        Self {
            journal,
            link_id: link_id.to_owned(),
            sequence: AtomicU64::new(0),
        }
    }

    pub(crate) async fn emit(&self, event: LinkEvent) {
        let link = &self.link_id;
        let name = event.name();
        let level = event.level();
        let message = event.message();
        let Value::Object(fields) =
            serde_json::to_value(&event).expect("LinkEvent serializes as an object")
        else {
            unreachable!("LinkEvent variants are structs");
        };
        match level {
            DiagnosticLevel::Error | DiagnosticLevel::Critical => {
                tracing::error!(%link, event = name, ?fields, "{message}");
            }
            DiagnosticLevel::Warning => tracing::warn!(%link, event = name, ?fields, "{message}"),
            _ => tracing::info!(%link, event = name, ?fields, "{message}"),
        }
        let diagnostic = DiagnosticEvent {
            schema_version: 1,
            run_id: self.journal.run_id(),
            source: "linkhub.link".to_owned(),
            source_instance: format!("linkhub.link.{link}"),
            source_sequence: self.sequence.fetch_add(1, Ordering::Relaxed) + 1,
            source_wall_time_ns: wall_time_ns(),
            source_monotonic_ns: None,
            sim_time_ns: None,
            sim_time_quality: None,
            level,
            category: "link".to_owned(),
            event: name.to_owned(),
            message,
            correlation_id: None,
            causation_id: None,
            fields,
            related_records: Vec::new(),
        };
        if let Err(error) = self.journal.append_diagnostic(diagnostic).await {
            tracing::warn!(%link, %error, "could not journal link diagnostic");
        }
    }
}
/// Suppresses identical consecutive failures (the link retries every
/// `reconnect_interval`) so the journal records each distinct failure, plus
/// a periodic reminder while it persists. Each channel (`scan`, `session`)
/// is tracked separately so an alternating discover/open-fail loop is still
/// collapsed.
#[derive(Default)]
pub(crate) struct FailureRepeats {
    channels: BTreeMap<&'static str, (String, u64)>,
}

impl FailureRepeats {
    pub(crate) const REMIND_EVERY: u64 = 60;

    /// Returns the repeat count if this failure should be emitted.
    pub(crate) fn observe(&mut self, channel: &'static str, signature: String) -> Option<u64> {
        match self.channels.get_mut(channel) {
            Some((last, count)) if *last == signature => {
                *count += 1;
                (*count % Self::REMIND_EVERY == 0).then_some(*count)
            }
            _ => {
                self.channels.insert(channel, (signature, 1));
                Some(1)
            }
        }
    }

    pub(crate) fn reset(&mut self) {
        self.channels.clear();
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    #[test]
    fn link_events_serialize_as_flat_field_objects() {
        let event = LinkEvent::OpenFailed {
            target: "tcp:127.0.0.1:5760".to_owned(),
            error: "connection refused".to_owned(),
            error_kind: "ConnectionRefused".to_owned(),
            repeat_count: 1,
        };
        assert_eq!(event.name(), "link.open_failed");
        assert_eq!(
            serde_json::to_value(&event).expect("serialize"),
            json!({
                "target": "tcp:127.0.0.1:5760",
                "error": "connection refused",
                "error_kind": "ConnectionRefused",
                "repeat_count": 1,
            })
        );
    }

    #[test]
    fn failure_repeats_collapse_identical_failures_per_channel() {
        let mut repeats = FailureRepeats::default();
        assert_eq!(repeats.observe("scan", "a".to_owned()), Some(1));
        // An interleaved channel does not break the scan run.
        assert_eq!(repeats.observe("session", "x".to_owned()), Some(1));
        for _ in 2..FailureRepeats::REMIND_EVERY {
            assert_eq!(repeats.observe("scan", "a".to_owned()), None);
            assert_eq!(repeats.observe("session", "x".to_owned()), None);
        }
        assert_eq!(
            repeats.observe("scan", "a".to_owned()),
            Some(FailureRepeats::REMIND_EVERY)
        );
        assert_eq!(repeats.observe("scan", "b".to_owned()), Some(1));
        repeats.reset();
        assert_eq!(repeats.observe("session", "x".to_owned()), Some(1));
    }
}
