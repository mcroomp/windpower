//! DataFlash log listing and loss-tolerant downloads with host progress hooks.

use std::{collections::BTreeMap, time::Duration};

use crate::codec::{
    MavMessage,
    dialect::{LOG_DATA_DATA, LOG_REQUEST_DATA_DATA, LOG_REQUEST_END_DATA, LOG_REQUEST_LIST_DATA},
};
use serde_json::{Value, json};
use tokio::time;

use super::{
    OperationError, ProtocolClient, ReceivedMessage, TransferObserver, TransferSummary,
    transfer::{TransferError as Failure, acquire, merge_cleanup},
    wait_for_message,
};
use crate::dataflash::{
    DataFlashError, DataPacket, LogDownload, LogEntry, LogList, Target as DataFlashTarget,
};

impl TransferSummary {
    fn capture_log(&mut self, download: &LogDownload) {
        self.done = u64::from(download.offset());
        self.received = download.received_bytes();
        self.stats = download.stats();
    }
}

impl ProtocolClient {
    pub(super) async fn list_logs_inner(
        &self,
        timeout: Duration,
    ) -> Result<Vec<Value>, OperationError> {
        let _guard = self.log_lock.lock().await;
        Ok(self
            .request_log_entries(0, u16::MAX, timeout)
            .await?
            .into_iter()
            .map(|(entry, cursor)| {
                json!({
                    "id": entry.id,
                    "size": entry.size,
                    "time_utc": entry.time_utc,
                    "num_logs": entry.num_logs,
                    "last_log_num": entry.last_log_num,
                    "after_cursor": cursor,
                })
            })
            .collect())
    }

    /// `packet_timeout` bounds silence before a request is repeated. Failure
    /// follows `max_retries` repeats without progress.
    pub(super) async fn download_log_inner(
        &self,
        handle: &dyn TransferObserver,
        log_id: u16,
        packet_timeout: Duration,
        max_retries: u32,
        summary: &mut TransferSummary,
    ) -> Result<Vec<u8>, Failure> {
        match acquire(self.log_lock.clone(), handle).await {
            Ok(_guard) => {
                let result = self
                    .read_log(handle, log_id, packet_timeout, max_retries, summary)
                    .await;
                let (target_system, target_component) = self.targets(None, None);
                // Release the vehicle's stream even if metadata lookup or the
                // data phase failed or was cancelled. Keep the lock until done.
                let ended = self
                    .link
                    .send_message(
                        MavMessage::LOG_REQUEST_END(LOG_REQUEST_END_DATA {
                            target_system,
                            target_component,
                        }),
                        None,
                        None,
                    )
                    .await;
                merge_cleanup(result, ended.map(|_| ()).map_err(OperationError::from))
            }
            Err(failure) => Err(failure),
        }
    }

    async fn read_log(
        &self,
        handle: &dyn TransferObserver,
        log_id: u16,
        packet_timeout: Duration,
        max_retries: u32,
        summary: &mut TransferSummary,
    ) -> Result<Vec<u8>, Failure> {
        let entries = tokio::select! {
            result = self.request_log_entries(log_id, log_id, packet_timeout) => result?,
            () = handle.cancelled() => return Err(Failure::Cancelled),
        };
        let (entry, _) = entries
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
                let received = tokio::select! {
                    result = wait_for_message(&mut receiver, packet_timeout, |message| {
                        log_data(message).is_some_and(|data| data.id == log_id)
                    }) => result,
                    () = handle.cancelled() => return Err(Failure::Cancelled),
                };
                match received {
                    Ok(message) => match download.receive(data_packet(&message)?) {
                        Err(DataFlashError::EarlyEnd { .. }) => {
                            tokio::select! {
                                () = time::sleep(packet_timeout) => {},
                                () = handle.cancelled() => return Err(Failure::Cancelled),
                            }
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
        summary.capture_log(&download);

        transferred?;
        download
            .into_bytes()
            .ok_or_else(|| OperationError::Invalid("log transfer is incomplete".to_owned()).into())
    }

    /// Entries paired with the opaque host cursor of their `LOG_ENTRY`.
    async fn request_log_entries(
        &self,
        start: u16,
        end: u16,
        timeout: Duration,
    ) -> Result<Vec<(LogEntry, String)>, OperationError> {
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
            entries.insert(entry.id, (entry, message.cursor.clone()));
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
