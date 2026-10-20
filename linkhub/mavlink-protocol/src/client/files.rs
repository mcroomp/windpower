//! MAVFTP operations: directory listing, uploads, and loss-tolerant file
//! downloads built on [`FtpReader`], independent of host job scheduling.

use std::time::{Duration, Instant};

use crate::codec::{MavMessage, dialect::FILE_TRANSFER_PROTOCOL_DATA};
use serde_json::{Value, json};
use tokio::{sync::broadcast::error::RecvError, time};

use super::{
    OperationError, ProtocolClient, ReceivedMessage, TransferObserver, TransferSummary,
    transfer::{TransferError as Failure, acquire, merge_cleanup},
    wait_for_message,
};
use crate::{
    ftp_read::{FtpReader, Timing},
    mavftp::{
        MavFtpError, Operation as MavFtpOperation, OperationResult as MavFtpResult, Packet,
        mavftp_crc32,
    },
    stats::TransferStats,
};

const MAVFTP_SOURCE_COMPONENT: u8 = 190;
const MAVFTP_PACKET_RETRIES: usize = 3;

impl TransferSummary {
    fn capture_file(&mut self, reader: &FtpReader) {
        self.done = reader.contiguous_bytes();
        self.received = reader.received_bytes();
        self.stats = reader.stats();
    }
}

impl ProtocolClient {
    pub(super) async fn list_files_inner(
        &self,
        path: &str,
        timeout: Duration,
    ) -> Result<Vec<Value>, OperationError> {
        let result = self
            .execute_mavftp(|sequence| MavFtpOperation::list(sequence, path), timeout)
            .await?;
        let MavFtpResult::Files(files) = result else {
            return Err(OperationError::Invalid(
                "MAVFTP list returned an unexpected result".to_owned(),
            ));
        };
        Ok(files
            .into_iter()
            .map(|entry| {
                json!({
                    "name": entry.name,
                    "is_dir": entry.is_dir,
                    "size": entry.size,
                })
            })
            .collect())
    }

    pub(super) async fn remove_file_inner(
        &self,
        path: &str,
        timeout: Duration,
    ) -> Result<(), OperationError> {
        let result = self
            .execute_mavftp(|sequence| MavFtpOperation::remove(sequence, path), timeout)
            .await?;
        if result != MavFtpResult::Removed {
            return Err(OperationError::Invalid(
                "MAVFTP remove returned an unexpected result".to_owned(),
            ));
        }
        Ok(())
    }

    pub(super) async fn create_directory_inner(
        &self,
        path: &str,
        timeout: Duration,
    ) -> Result<bool, OperationError> {
        let result = self
            .execute_mavftp(|sequence| MavFtpOperation::mkdir(sequence, path), timeout)
            .await?;
        let MavFtpResult::DirectoryCreated(created) = result else {
            return Err(OperationError::Invalid(
                "MAVFTP mkdir returned an unexpected result".to_owned(),
            ));
        };
        Ok(created)
    }

    /// Downloads while holding the shared FTP session lock. The stall timeout
    /// bounds time without gaining a contiguous byte, not total transfer time.
    pub(super) async fn download_file_inner(
        &self,
        handle: &dyn TransferObserver,
        path: &str,
        verify_crc: bool,
        stall_timeout: Duration,
        summary: &mut TransferSummary,
    ) -> Result<Vec<u8>, Failure> {
        match acquire(self.ftp_sequence.clone(), handle).await {
            Ok(mut sequence) => {
                self.read_file(
                    &mut sequence,
                    handle,
                    path,
                    verify_crc,
                    stall_timeout,
                    summary,
                )
                .await
            }
            Err(failure) => Err(failure),
        }
    }

    /// Uploads and verifies the remote CRC before reporting success.
    pub(super) async fn upload_file_inner(
        &self,
        handle: &dyn TransferObserver,
        path: &str,
        content: Vec<u8>,
        stall_timeout: Duration,
        summary: &mut TransferSummary,
    ) -> Result<(), Failure> {
        let total = content.len() as u64;
        let mut done = 0;
        let mut stats = TransferStats::default();
        let result = match acquire(self.ftp_sequence.clone(), handle).await {
            Ok(mut sequence) => {
                handle.start(total).await;
                let local_crc = mavftp_crc32(&content);
                let mut upload = MavFtpOperation::upload(*sequence, path, content);
                let uploaded = self
                    .drive_upload(&mut upload, handle, stall_timeout, &mut done, &mut stats)
                    .await;
                *sequence = upload.next_sequence();
                let uploaded = match uploaded {
                    Ok(bytes) => Ok(bytes),
                    Err(failure) => {
                        let cleanup = self.cleanup_upload(&upload, &mut sequence).await;
                        merge_cleanup(Err(failure), cleanup)
                    }
                };
                match uploaded {
                    Ok(bytes) if bytes == total as usize => {
                        let mut crc = MavFtpOperation::crc32(*sequence, path);
                        let remote = tokio::select! {
                            result = self.run_mavftp(&mut crc, stall_timeout) => result.map_err(Failure::from),
                            () = handle.cancelled() => Err(Failure::Cancelled),
                        };
                        *sequence = crc.next_sequence();
                        match remote {
                            Ok(MavFtpResult::Crc32(remote_crc)) if remote_crc == local_crc => {
                                Ok(())
                            }
                            Ok(MavFtpResult::Crc32(remote_crc)) => Err(MavFtpError::CrcMismatch {
                                remote: remote_crc,
                                local: local_crc,
                            }
                            .into()),
                            Ok(_) => Err(OperationError::Invalid(
                                "MAVFTP CRC returned an unexpected result".to_owned(),
                            )
                            .into()),
                            Err(error) => Err(error),
                        }
                    }
                    Ok(bytes) => Err(OperationError::Invalid(format!(
                        "MAVFTP upload wrote {bytes} of {total} bytes"
                    ))
                    .into()),
                    Err(failure) => Err(failure),
                }
            }
            Err(failure) => Err(failure),
        };
        summary.done = done;
        summary.received = done;
        summary.stats = stats;
        result
    }

    async fn cleanup_upload(
        &self,
        operation: &MavFtpOperation,
        sequence: &mut u16,
    ) -> Result<(), OperationError> {
        let Some(session) = operation.active_session() else {
            return Ok(());
        };
        let mut terminate = MavFtpOperation::terminate(*sequence, session);
        let result = self
            .run_mavftp(&mut terminate, Duration::from_secs(3))
            .await;
        *sequence = terminate.next_sequence();
        result.map(|_| ())
    }

    async fn drive_upload(
        &self,
        operation: &mut MavFtpOperation,
        handle: &dyn TransferObserver,
        stall_timeout: Duration,
        done: &mut u64,
        stats: &mut TransferStats,
    ) -> Result<usize, Failure> {
        let mut deadline = time::Instant::now() + stall_timeout;
        let (target_system, target_component) = self.targets(None, None);
        let mut receiver = self.link.subscribe_messages();
        loop {
            if let Some(result) = operation.result().cloned() {
                let MavFtpResult::Uploaded(bytes) = result else {
                    return Err(OperationError::Invalid(
                        "MAVFTP upload returned an unexpected result".to_owned(),
                    )
                    .into());
                };
                return Ok(bytes);
            }
            if handle.is_cancelled() {
                return Err(Failure::Cancelled);
            }
            let packet = operation.next_request()?.ok_or_else(|| {
                OperationError::Invalid("MAVFTP upload ended without a result".to_owned())
            })?;
            let payload = packet.encode()?;
            let expected_reply_sequence = packet.sequence.wrapping_add(1);
            let expected_request_opcode = packet.opcode as u8;
            let message = MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA {
                target_network: 0,
                target_system,
                target_component,
                payload,
            });
            let mut retries = 0;
            let reply = loop {
                if retries == 0 {
                    stats.requests_sent += 1;
                } else {
                    stats.retransmits += 1;
                }
                self.link
                    .send_message(message.clone(), None, Some(MAVFTP_SOURCE_COMPONENT))
                    .await?;
                let remaining = deadline.saturating_duration_since(time::Instant::now());
                if remaining.is_zero() {
                    return Err(OperationError::Timeout.into());
                }
                let waiting = wait_for_message(
                    &mut receiver,
                    remaining.min(Duration::from_secs(2)),
                    |candidate| {
                        ftp_payload(candidate).is_some_and(|(target, payload)| {
                            candidate.component_id == target_component
                                && target == MAVFTP_SOURCE_COMPONENT
                                && ftp_reply_matches(
                                    payload,
                                    expected_reply_sequence,
                                    expected_request_opcode,
                                )
                        })
                    },
                );
                match tokio::select! {
                    result = waiting => result,
                    () = handle.cancelled() => return Err(Failure::Cancelled),
                } {
                    Ok(message) => break message,
                    Err(OperationError::Timeout) if retries < MAVFTP_PACKET_RETRIES => {
                        retries += 1;
                        stats.timeouts += 1;
                    }
                    Err(error) => return Err(error.into()),
                }
            };
            stats.packets_received += 1;
            let (_, reply_payload) = ftp_payload(&reply).expect("wait predicate selected FTP");
            let reply_packet = Packet::decode(reply_payload)?;
            if reply_packet.opcode == crate::mavftp::Opcode::Nak {
                stats.naks += 1;
            }
            operation.handle_reply(reply_packet)?;
            if packet.opcode == crate::mavftp::Opcode::WriteFile {
                *done = u64::from(packet.offset) + packet.data.len() as u64;
                handle.progress(*done, *done, *stats);
                deadline = time::Instant::now() + stall_timeout;
            }
        }
    }

    /// Opens, reads, releases and verifies `path`, holding the FTP session.
    async fn read_file(
        &self,
        sequence: &mut u16,
        handle: &dyn TransferObserver,
        path: &str,
        verify_crc: bool,
        stall_timeout: Duration,
        summary: &mut TransferSummary,
    ) -> Result<Vec<u8>, Failure> {
        let mut open = MavFtpOperation::open_read(*sequence, path);
        let opened = tokio::select! {
            result = self.run_mavftp(&mut open, stall_timeout) => result.map_err(Failure::from),
            () = handle.cancelled() => Err(Failure::Cancelled),
        };
        *sequence = open.next_sequence();
        let MavFtpResult::Opened { session, size } = opened? else {
            return Err(OperationError::Invalid(
                "MAVFTP open returned an unexpected result".to_owned(),
            )
            .into());
        };
        handle.start(u64::from(size)).await;

        let mut reader =
            FtpReader::new(session, size, *sequence, Timing::default(), Instant::now());
        let driven = self.drive_reader(&mut reader, handle, stall_timeout).await;
        *sequence = reader.next_sequence();
        summary.capture_file(&reader);
        // Release the file whatever happened: the autopilot refuses a new open
        // while a session it believes active still holds one.
        let mut terminate = MavFtpOperation::terminate(*sequence, session);
        let terminated = self
            .run_mavftp(&mut terminate, Duration::from_secs(3))
            .await;
        *sequence = terminate.next_sequence();
        merge_cleanup(driven, terminated.map(|_| ()))?;

        if verify_crc {
            let mut crc = MavFtpOperation::crc32(*sequence, path);
            let remote = tokio::select! {
                result = self.run_mavftp(&mut crc, Duration::from_secs(30)) => result.map_err(Failure::from),
                () = handle.cancelled() => Err(Failure::Cancelled),
            };
            *sequence = crc.next_sequence();
            let MavFtpResult::Crc32(remote) = remote? else {
                return Err(OperationError::Invalid(
                    "MAVFTP CRC returned an unexpected result".to_owned(),
                )
                .into());
            };
            let local = mavftp_crc32(reader.data());
            if remote != local {
                return Err(MavFtpError::CrcMismatch { remote, local }.into());
            }
        }
        Ok(reader.into_data())
    }

    /// Feeds the reader until the file is complete, the transfer stalls, or it
    /// is cancelled.
    async fn drive_reader(
        &self,
        reader: &mut FtpReader,
        handle: &dyn TransferObserver,
        stall_timeout: Duration,
    ) -> Result<(), Failure> {
        let mut receiver = self.link.subscribe_messages();
        let mut best = 0;
        let mut progressed_at = Instant::now();
        loop {
            for packet in reader.poll(Instant::now()) {
                self.send_ftp(&packet).await?;
            }
            handle.progress(
                reader.contiguous_bytes(),
                reader.received_bytes(),
                reader.stats(),
            );
            if reader.is_complete() {
                return Ok(());
            }
            if handle.is_cancelled() {
                return Err(Failure::Cancelled);
            }
            let now = Instant::now();
            if reader.contiguous_bytes() > best {
                best = reader.contiguous_bytes();
                progressed_at = now;
            } else if now.saturating_duration_since(progressed_at) > stall_timeout {
                return Err(OperationError::Timeout.into());
            }
            // Other traffic keeps arriving, so silence cannot be left to the
            // receive timeout: expire the reader's deadline explicitly.
            let deadline = reader.next_deadline(now).unwrap_or(now);
            if deadline <= now {
                reader.on_tick(now)?;
                continue;
            }
            let received = tokio::select! {
                result = time::timeout(deadline - now, receiver.recv()) => result,
                () = handle.cancelled() => return Err(Failure::Cancelled),
            };
            match received {
                Ok(Ok(message)) => {
                    if let Some(packet) = self.ftp_reply(&message) {
                        reader.on_reply(Instant::now(), &packet)?;
                    }
                }
                // Messages the receiver skipped are just more lost packets.
                Ok(Err(RecvError::Lagged(_))) | Err(_) => {}
                Ok(Err(RecvError::Closed)) => return Err(OperationError::ReceiveClosed.into()),
            }
        }
    }

    async fn execute_mavftp(
        &self,
        create: impl FnOnce(u16) -> MavFtpOperation,
        timeout: Duration,
    ) -> Result<MavFtpResult, OperationError> {
        let mut sequence = self.ftp_sequence.lock().await;
        let mut operation = create(*sequence);
        let result = self.run_mavftp(&mut operation, timeout).await;
        *sequence = operation.next_sequence();
        result
    }

    /// Runs a stop-and-wait operation: each request is repeated until its
    /// reply arrives or the operation stops making progress for `stall_timeout`.
    async fn run_mavftp(
        &self,
        operation: &mut MavFtpOperation,
        stall_timeout: Duration,
    ) -> Result<MavFtpResult, OperationError> {
        let mut deadline = time::Instant::now() + stall_timeout;
        let (target_system, target_component) = self.targets(None, None);
        let mut receiver = self.link.subscribe_messages();
        loop {
            if let Some(result) = operation.result().cloned() {
                return Ok(result);
            }
            let packet = operation.next_request()?.ok_or_else(|| {
                OperationError::Invalid("MAVFTP operation ended without a result".to_owned())
            })?;
            let payload = packet.encode()?;
            let expected_reply_sequence = packet.sequence.wrapping_add(1);
            let expected_request_opcode = packet.opcode as u8;
            let message = MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA {
                target_network: 0,
                target_system,
                target_component,
                payload,
            });
            let mut retries = 0;
            let reply = loop {
                self.link
                    .send_message(message.clone(), None, Some(MAVFTP_SOURCE_COMPONENT))
                    .await?;
                let remaining = deadline.saturating_duration_since(time::Instant::now());
                if remaining.is_zero() {
                    return Err(OperationError::Timeout);
                }
                match wait_for_message(
                    &mut receiver,
                    remaining.min(Duration::from_secs(2)),
                    |candidate| {
                        ftp_payload(candidate).is_some_and(|(target, payload)| {
                            candidate.component_id == target_component
                                && target == MAVFTP_SOURCE_COMPONENT
                                && ftp_reply_matches(
                                    payload,
                                    expected_reply_sequence,
                                    expected_request_opcode,
                                )
                        })
                    },
                )
                .await
                {
                    Ok(message) => break message,
                    Err(OperationError::Timeout) if retries < MAVFTP_PACKET_RETRIES => {
                        retries += 1;
                    }
                    Err(error) => return Err(error),
                }
            };
            let (_, reply_payload) = ftp_payload(&reply).expect("wait predicate selected FTP");
            operation.handle_reply(Packet::decode(reply_payload)?)?;
            deadline = time::Instant::now() + stall_timeout;
        }
    }

    async fn send_ftp(&self, packet: &Packet) -> Result<(), OperationError> {
        let (target_system, target_component) = self.targets(None, None);
        let message = MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA {
            target_network: 0,
            target_system,
            target_component,
            payload: packet.encode()?,
        });
        self.link
            .send_message(message, None, Some(MAVFTP_SOURCE_COMPONENT))
            .await?;
        Ok(())
    }

    /// The MAVFTP packet in `message` if it is addressed to this client.
    fn ftp_reply(&self, message: &ReceivedMessage) -> Option<Packet> {
        let (_, target_component) = self.targets(None, None);
        let (target, payload) = ftp_payload(message)?;
        if message.component_id != target_component || target != MAVFTP_SOURCE_COMPONENT {
            return None;
        }
        Packet::decode(payload).ok()
    }
}

/// The target component and 251-byte payload of a FILE_TRANSFER_PROTOCOL message.
fn ftp_payload(message: &ReceivedMessage) -> Option<(u8, &[u8])> {
    match &message.message {
        MavMessage::FILE_TRANSFER_PROTOCOL(ftp) => Some((ftp.target_component, &ftp.payload)),
        _ => None,
    }
}

fn ftp_reply_matches(payload: &[u8], expected_sequence: u16, expected_request_opcode: u8) -> bool {
    Packet::decode(payload).ok().is_some_and(|reply| {
        reply.sequence == expected_sequence && reply.request_opcode == expected_request_opcode
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mavftp_correlation_rejects_delayed_prior_operation_reply() {
        let packet = Packet {
            sequence: 11,
            session: 0,
            opcode: crate::mavftp::Opcode::Ack,
            size: 0,
            request_opcode: crate::mavftp::Opcode::CreateDirectory as u8,
            burst_complete: 0,
            offset: 0,
            data: Vec::new(),
        };
        let payload = packet.encode().expect("packet");

        assert!(!ftp_reply_matches(
            &payload,
            11,
            crate::mavftp::Opcode::CreateFile as u8
        ));
        assert!(ftp_reply_matches(
            &payload,
            11,
            crate::mavftp::Opcode::CreateDirectory as u8
        ));
    }
}
