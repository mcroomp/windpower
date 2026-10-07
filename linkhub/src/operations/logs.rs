//! DataFlash log listing and loss-tolerant log downloads, run as observable
//! transfer jobs.

use std::{collections::BTreeMap, time::Duration};

use serde_json::{Value, json};
use tokio::time;

use super::{
    MavlinkOperations, OperationError, ReceivedMessage, format_cursor,
    job::{Failure, acquire},
    wait_for_message,
};
use crate::{
    codec::{
        MavMessage,
        dialect::{
            LOG_DATA_DATA, LOG_REQUEST_DATA_DATA, LOG_REQUEST_END_DATA, LOG_REQUEST_LIST_DATA,
        },
    },
    dataflash::{
        DataFlashError, DataPacket, LogDownload, LogEntry, LogList, Target as DataFlashTarget,
    },
    transfer::{Outcome, TransferHandle, TransferInfo, TransferKind, TransferStats},
};

/// What a download had achieved when it ended, for the transfer record.
#[derive(Default)]
struct Summary {
    done: u64,
    received: u64,
    stats: TransferStats,
}

impl Summary {
    fn capture(&mut self, download: &LogDownload) {
        self.done = u64::from(download.offset());
        self.received = download.received_bytes();
        self.stats = download.stats();
    }
}

impl MavlinkOperations {
    pub async fn list_logs(&self, timeout: Duration) -> Result<Vec<Value>, OperationError> {
        let _guard = self.log_lock.lock().await;
        Ok(self
            .request_log_entries(0, u16::MAX, timeout)
            .await?
            .into_iter()
            .map(|(entry, journal_sequence)| {
                json!({
                    "id": entry.id,
                    "size": entry.size,
                    "time_utc": entry.time_utc,
                    "num_logs": entry.num_logs,
                    "last_log_num": entry.last_log_num,
                    "after_cursor": format_cursor(journal_sequence),
                })
            })
            .collect())
    }

    /// Queues a download of log `log_id` and returns its record at once. The
    /// transfer runs in the background: poll [`transfers`](Self::transfers) for
    /// progress and collect the content when it completes.
    ///
    /// `packet_timeout` is how long the vehicle may stay silent before a
    /// request is repeated; the transfer fails after `max_retries` repeats
    /// without progress.
    #[must_use]
    pub fn start_log_download(
        &self,
        log_id: u16,
        packet_timeout: Duration,
        max_retries: u32,
    ) -> TransferInfo {
        let handle = self
            .transfers
            .submit(TransferKind::LogDownload, format!("log {log_id}"));
        let info = self
            .transfers
            .get(handle.id())
            .expect("a submitted transfer is registered");
        tokio::spawn(
            self.clone()
                .run_log_download(handle, log_id, packet_timeout, max_retries),
        );
        info
    }

    async fn run_log_download(
        self,
        mut handle: TransferHandle,
        log_id: u16,
        packet_timeout: Duration,
        max_retries: u32,
    ) {
        let mut summary = Summary::default();
        let result = match acquire(self.log_lock.clone(), &mut handle).await {
            Ok(_guard) => {
                self.read_log(&handle, log_id, packet_timeout, max_retries, &mut summary)
                    .await
            }
            Err(failure) => Err(failure),
        };
        let outcome = result.map_or_else(Failure::into_outcome, Outcome::Complete);
        handle
            .finish(outcome, summary.done, summary.received, summary.stats)
            .await;
    }

    async fn read_log(
        &self,
        handle: &TransferHandle,
        log_id: u16,
        packet_timeout: Duration,
        max_retries: u32,
        summary: &mut Summary,
    ) -> Result<Vec<u8>, Failure> {
        let (entry, _) = self
            .request_log_entries(log_id, log_id, packet_timeout)
            .await?
            .into_iter()
            .find(|(entry, _)| entry.id == log_id)
            .ok_or_else(|| OperationError::NotFound(format!("DataFlash log {log_id}")))?;
        handle.start(u64::from(entry.size)).await;
        let (target_system, target_component) = self.targets(None, None);
        let target = DataFlashTarget {
            system: target_system,
            component: target_component,
        };
        let mut download = LogDownload::new(target, log_id, entry.size, max_retries);
        let mut receiver = self.link.subscribe_messages();

        let transferred = async {
            while !download.is_complete() {
                if handle.is_cancelled() {
                    return Err(Failure::Cancelled);
                }
                if let Some(request) = download.next_request() {
                    self.link
                        .send_message(
                            MavMessage::LOG_REQUEST_DATA(LOG_REQUEST_DATA_DATA {
                                ofs: request.offset,
                                count: request.count,
                                id: request.id,
                                target_system: request.target.system,
                                target_component: request.target.component,
                            }),
                            None,
                            None,
                        )
                        .await?;
                }
                match wait_for_message(&mut receiver, packet_timeout, |message| {
                    log_data(message).is_some_and(|data| data.id == log_id)
                })
                .await
                {
                    Ok(message) => match download.receive(data_packet(&message)?) {
                        Err(DataFlashError::EarlyEnd { .. }) => {
                            time::sleep(packet_timeout).await;
                            download.on_timeout()?;
                        }
                        result => result?,
                    },
                    Err(OperationError::Timeout) => download.on_timeout()?,
                    Err(error) => return Err(error.into()),
                }
                handle.progress(
                    u64::from(download.offset()),
                    download.received_bytes(),
                    download.stats(),
                );
            }
            Ok(())
        }
        .await;
        summary.capture(&download);

        // End the stream on the vehicle whether or not the transfer finished.
        let ended = self
            .link
            .send_message(
                MavMessage::LOG_REQUEST_END(LOG_REQUEST_END_DATA {
                    target_system: target.system,
                    target_component: target.component,
                }),
                None,
                None,
            )
            .await;
        transferred?;
        ended?;
        download
            .into_bytes()
            .ok_or_else(|| OperationError::Invalid("log transfer is incomplete".to_owned()).into())
    }

    /// The log entries the vehicle reported, each with the journal sequence of
    /// the `LOG_ENTRY` message it came from.
    async fn request_log_entries(
        &self,
        start: u16,
        end: u16,
        timeout: Duration,
    ) -> Result<Vec<(LogEntry, u64)>, OperationError> {
        let (target_system, target_component) = self.targets(None, None);
        let target = DataFlashTarget {
            system: target_system,
            component: target_component,
        };
        let mut list = LogList::new(target, start, end)?;
        let mut receiver = self.link.subscribe_messages();
        self.link
            .send_message(
                MavMessage::LOG_REQUEST_LIST(LOG_REQUEST_LIST_DATA {
                    start,
                    end,
                    target_system: target.system,
                    target_component: target.component,
                }),
                None,
                None,
            )
            .await?;
        let mut entries = BTreeMap::new();
        while !list.is_complete() {
            let message = wait_for_message(&mut receiver, timeout, |message| {
                matches!(message.message, MavMessage::LOG_ENTRY(_))
                    && message.system_id == target.system
                    && (target.component == 0 || message.component_id == target.component)
            })
            .await?;
            let entry = log_entry(&message)?;
            list.receive(entry.clone())?;
            if entry.num_logs == 0 {
                return Ok(Vec::new());
            }
            entries.insert(entry.id, (entry, message.journal_sequence));
        }
        Ok(entries.into_values().collect())
    }
}

fn log_data(message: &ReceivedMessage) -> Option<&LOG_DATA_DATA> {
    match &message.message {
        MavMessage::LOG_DATA(data) => Some(data),
        _ => None,
    }
}

fn log_entry(message: &ReceivedMessage) -> Result<LogEntry, OperationError> {
    let MavMessage::LOG_ENTRY(entry) = &message.message else {
        return Err(OperationError::Invalid(
            "message is not LOG_ENTRY".to_owned(),
        ));
    };
    Ok(LogEntry {
        id: entry.id,
        size: entry.size,
        time_utc: entry.time_utc,
        num_logs: entry.num_logs,
        last_log_num: entry.last_log_num,
    })
}

fn data_packet(message: &ReceivedMessage) -> Result<DataPacket, OperationError> {
    let data = log_data(message)
        .ok_or_else(|| OperationError::Invalid("message is not LOG_DATA".to_owned()))?;
    Ok(DataPacket {
        id: data.id,
        offset: data.ofs,
        count: data.count,
        data: data.data,
    })
}
