use std::{collections::BTreeMap, sync::Arc, time::Duration};

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value, json};
use tokio::{
    sync::{Mutex, MutexGuard, broadcast},
    time,
};

use crate::{
    codec::{
        MavMessage,
        dialect::{
            AUTOPILOT_VERSION_DATA, COMMAND_ACK_DATA, COMMAND_LONG_DATA,
            FILE_TRANSFER_PROTOCOL_DATA, LOG_DATA_DATA, LOG_REQUEST_DATA_DATA,
            LOG_REQUEST_END_DATA, LOG_REQUEST_LIST_DATA, MavCmd, MavParamType,
            MavProtocolCapability, MavResult, PARAM_REQUEST_LIST_DATA, PARAM_REQUEST_READ_DATA,
            PARAM_SET_DATA, PARAM_VALUE_DATA,
        },
        message_id_from_name,
    },
    dataflash::{
        DataFlashError, DataPacket, LogDownload, LogEntry, LogList, Target as DataFlashTarget,
    },
    mavftp::{MavFtpError, Operation as MavFtpOperation, OperationResult as MavFtpResult, Packet},
    mavlink::{LinkError, MavlinkLinkHandle, ReceivedMessage},
    records::wall_time_ns,
};
use linkhub_dialect::types::CharArray;

const MAVFTP_SOURCE_COMPONENT: u8 = 190;
const MAVFTP_PACKET_RETRIES: usize = 3;

// ArduPilot still answers the superseded MAV_CMD_GET_MESSAGE_INTERVAL; it
// returns the interval as a MESSAGE_INTERVAL message.
#[allow(deprecated)]
const GET_MESSAGE_INTERVAL: MavCmd = MavCmd::MAV_CMD_GET_MESSAGE_INTERVAL;

// Bit 2: the dialect deprecates PARAM_FLOAT, but it is a different bit from its
// suggested replacement PARAM_ENCODE_C_CAST (131072), and vehicles still set it.
#[allow(deprecated)]
const PARAM_FLOAT: MavProtocolCapability =
    MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;

#[derive(Clone)]
pub struct MavlinkOperations {
    link: MavlinkLinkHandle,
    command_lock: Arc<Mutex<()>>,
    parameter_lock: Arc<Mutex<()>>,
    ftp_sequence: Arc<Mutex<u16>>,
    log_lock: Arc<Mutex<()>>,
}

#[derive(Clone, Debug, Serialize)]
pub struct CommandResult {
    pub command: MavCmd,
    pub result: MavResult,
    pub progress: u8,
    pub status: &'static str,
    pub after_cursor: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct ParameterResult {
    pub name: String,
    pub value: f64,
    #[serde(rename = "type")]
    pub param_type: MavParamType,
    pub index: i64,
    pub count: u64,
    pub after_cursor: String,
}

/// One requested parameter write; `type` defaults to `MAV_PARAM_TYPE_REAL32`.
#[derive(Clone, Debug, Deserialize)]
pub struct ParameterSetting {
    pub name: String,
    pub value: f64,
    #[serde(rename = "type", default = "default_parameter_type")]
    pub param_type: MavParamType,
}

fn default_parameter_type() -> MavParamType {
    MavParamType::MAV_PARAM_TYPE_REAL32
}

#[derive(Clone, Debug)]
pub struct DownloadedFile {
    pub path: String,
    pub content: Vec<u8>,
    pub crc32: u32,
    pub after_cursor: String,
}

#[derive(Clone, Debug)]
pub struct DownloadedLog {
    pub id: u16,
    pub size: u32,
    pub time_utc: u32,
    pub content: Vec<u8>,
    pub after_cursor: String,
}

#[derive(Debug, thiserror::Error)]
pub enum OperationError {
    #[error(transparent)]
    Link(#[from] LinkError),
    #[error("MAVLink operation timed out")]
    Timeout,
    #[error("MAVLink receive stream closed")]
    ReceiveClosed,
    #[error("MAVLink receive stream lagged by {0} messages")]
    ReceiveLagged(u64),
    #[error("invalid operation: {0}")]
    Invalid(String),
    #[error("MAVLink resource was not found: {0}")]
    NotFound(String),
    #[error(transparent)]
    MavFtp(#[from] MavFtpError),
    #[error(transparent)]
    DataFlash(#[from] DataFlashError),
}

impl MavlinkOperations {
    #[must_use]
    pub fn new(link: MavlinkLinkHandle) -> Self {
        Self {
            link,
            command_lock: Arc::new(Mutex::new(())),
            parameter_lock: Arc::new(Mutex::new(())),
            ftp_sequence: Arc::new(Mutex::new(wall_time_ns() as u16)),
            log_lock: Arc::new(Mutex::new(())),
        }
    }

    pub async fn command(
        &self,
        command: MavCmd,
        params: &[f64],
        target_system: Option<u8>,
        target_component: Option<u8>,
        timeout: Duration,
    ) -> Result<CommandResult, OperationError> {
        if params.len() > 7 {
            return Err(OperationError::Invalid(
                "commands accept at most seven parameters".to_owned(),
            ));
        }
        let deadline = time::Instant::now() + timeout;
        let _guard = lock_until(&self.command_lock, deadline).await?;
        let (target_system, target_component) = self.targets(target_system, target_component);

        let mut receiver = self.link.subscribe_messages();
        let mut confirmation = 0_u8;
        let message = loop {
            self.link
                .send_message(
                    command_long(
                        command,
                        params,
                        target_system,
                        target_component,
                        confirmation,
                    ),
                    None,
                    None,
                )
                .await?;
            let remaining = deadline.saturating_duration_since(time::Instant::now());
            if remaining.is_zero() {
                return Err(OperationError::Timeout);
            }
            match wait_for_message(
                &mut receiver,
                remaining.min(Duration::from_secs(1)),
                |message| acknowledges(message, command),
            )
            .await
            {
                Ok(message) => break message,
                Err(OperationError::Timeout) if time::Instant::now() < deadline => {
                    confirmation = confirmation.saturating_add(1);
                }
                Err(error) => return Err(error),
            }
        };
        let ack = command_ack(&message).expect("wait predicate selected a COMMAND_ACK");
        Ok(CommandResult {
            command,
            result: ack.result,
            progress: ack.progress,
            status: "acknowledged",
            after_cursor: format_cursor(message.journal_sequence),
        })
    }

    pub async fn get_parameter(
        &self,
        name: &str,
        timeout: Duration,
    ) -> Result<ParameterResult, OperationError> {
        let deadline = time::Instant::now() + timeout;
        let _guard = lock_until(&self.parameter_lock, deadline).await?;
        let normalized = normalize_parameter_name(name)?;
        let (target_system, target_component) = self.targets(None, None);
        let mut receiver = self.link.subscribe_messages();
        self.link
            .send_message(
                MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
                    param_index: -1,
                    target_system,
                    target_component,
                    param_id: CharArray::from(normalized.as_str()),
                }),
                None,
                None,
            )
            .await?;
        let message = wait_for_message(&mut receiver, remaining_until(deadline)?, |message| {
            parameter_matches(message, &normalized, target_system, target_component)
        })
        .await?;
        parameter_result(&message)
    }

    pub async fn set_parameter(
        &self,
        name: &str,
        value: f64,
        param_type: MavParamType,
        timeout: Duration,
    ) -> Result<ParameterResult, OperationError> {
        let deadline = time::Instant::now() + timeout;
        let _guard = lock_until(&self.parameter_lock, deadline).await?;
        let normalized = normalize_parameter_name(name)?;
        let (target_system, target_component) = self.targets(None, None);
        let mut receiver = self.link.subscribe_messages();
        self.link
            .send_message(
                param_set(
                    &normalized,
                    value,
                    param_type,
                    target_system,
                    target_component,
                ),
                None,
                None,
            )
            .await?;
        let message = wait_for_message(&mut receiver, remaining_until(deadline)?, |message| {
            parameter_matches(message, &normalized, target_system, target_component)
        })
        .await?;
        parameter_result(&message)
    }

    pub async fn list_parameters(
        &self,
        timeout: Duration,
    ) -> Result<Vec<ParameterResult>, OperationError> {
        let deadline = time::Instant::now() + timeout;
        let _guard = lock_until(&self.parameter_lock, deadline).await?;
        let (target_system, target_component) = self.targets(None, None);
        let mut receiver = self.link.subscribe_messages();
        self.link
            .send_message(
                MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
                    target_system,
                    target_component,
                }),
                None,
                None,
            )
            .await?;

        let mut parameters = BTreeMap::new();
        loop {
            let remaining = deadline.saturating_duration_since(time::Instant::now());
            if remaining.is_zero() {
                return Err(OperationError::Timeout);
            }

            let message = recv_message(&mut receiver, remaining).await?;
            if param_value(&message).is_none()
                || message.system_id != target_system
                || message.component_id != target_component
            {
                continue;
            }
            let result = parameter_result(&message)?;
            let expected = result.count as usize;
            parameters.insert(result.name.clone(), result);
            if parameters.len() >= expected {
                return Ok(parameters.into_values().collect());
            }
        }
    }

    pub async fn set_parameters(
        &self,
        parameters: &[ParameterSetting],
        timeout: Duration,
        retries: usize,
    ) -> Result<Vec<ParameterResult>, OperationError> {
        let deadline = time::Instant::now() + timeout;
        let _guard = lock_until(&self.parameter_lock, deadline).await?;
        if parameters.is_empty() {
            return Ok(Vec::new());
        }
        let (target_system, target_component) = self.targets(None, None);
        let mut requests: Vec<(String, f64, MavParamType)> = Vec::with_capacity(parameters.len());
        for parameter in parameters {
            let name = normalize_parameter_name(&parameter.name)?;
            if requests.iter().any(|(existing, _, _)| existing == &name) {
                return Err(OperationError::Invalid(format!(
                    "duplicate parameter {name}"
                )));
            }
            requests.push((name, parameter.value, parameter.param_type));
        }

        let mut receiver = self.link.subscribe_messages();
        let mut pending: BTreeMap<String, (f64, MavParamType)> = requests
            .iter()
            .map(|(name, value, param_type)| (name.clone(), (*value, *param_type)))
            .collect();
        let mut results = BTreeMap::new();
        for attempt in 0..=retries {
            for (name, (value, param_type)) in &pending {
                self.link
                    .send_message(
                        param_set(name, *value, *param_type, target_system, target_component),
                        None,
                        None,
                    )
                    .await?;
            }
            let attempt_deadline = deadline
                .min(time::Instant::now() + timeout.div_f64((retries.saturating_add(1)) as f64));
            while !pending.is_empty() {
                let remaining = attempt_deadline.saturating_duration_since(time::Instant::now());
                if remaining.is_zero() {
                    break;
                }
                let message = match recv_message(&mut receiver, remaining).await {
                    Ok(message) => message,
                    Err(OperationError::Timeout) => break,
                    Err(error) => return Err(error),
                };
                if param_value(&message).is_none()
                    || message.system_id != target_system
                    || message.component_id != target_component
                {
                    continue;
                }
                let result = parameter_result(&message)?;
                if pending.remove(&result.name).is_some() {
                    results.insert(result.name.clone(), result);
                }
            }
            if pending.is_empty() {
                return requests
                    .iter()
                    .map(|(name, _, _)| {
                        results.remove(name).ok_or_else(|| {
                            OperationError::Invalid(format!("missing result for {name}"))
                        })
                    })
                    .collect();
            }
            if attempt == retries {
                break;
            }
        }
        Err(OperationError::Timeout)
    }

    pub async fn request_message(
        &self,
        message: &str,
        target_system: Option<u8>,
        target_component: Option<u8>,
        timeout: Duration,
    ) -> Result<Value, OperationError> {
        let message_name = message.to_uppercase();
        let (ack, response) = self
            .request_response(&message_name, target_system, target_component, timeout)
            .await?;
        Ok(json!({
            "message": message_name,
            "system_id": response.system_id,
            "component_id": response.component_id,
            "fields": response.fields,
            "ack_cursor": format_cursor(ack.journal_sequence),
            "after_cursor": format_cursor(response.journal_sequence),
        }))
    }

    /// Sends `MAV_CMD_REQUEST_MESSAGE` and returns the acknowledgement together
    /// with the requested message.
    async fn request_response(
        &self,
        message_name: &str,
        target_system: Option<u8>,
        target_component: Option<u8>,
        timeout: Duration,
    ) -> Result<(Arc<ReceivedMessage>, Arc<ReceivedMessage>), OperationError> {
        let message_id = message_id_from_name(message_name).ok_or_else(|| {
            OperationError::Invalid(format!("unknown MAVLink message {message_name:?}"))
        })?;
        let deadline = time::Instant::now() + timeout;
        let _guard = lock_until(&self.command_lock, deadline).await?;
        let (target_system, target_component) = self.targets(target_system, target_component);
        let mut receiver = self.link.subscribe_messages();
        self.send_command_long(
            MavCmd::MAV_CMD_REQUEST_MESSAGE,
            &[f64::from(message_id)],
            target_system,
            target_component,
        )
        .await?;
        let (ack, response) = wait_for_command_response(
            &mut receiver,
            remaining_until(deadline)?,
            MavCmd::MAV_CMD_REQUEST_MESSAGE,
            |candidate| {
                candidate.name == message_name
                    && candidate.system_id == target_system
                    && (target_component == 0 || candidate.component_id == target_component)
            },
        )
        .await?;
        ensure_accepted(&ack, &format!("request for {message_name}"))?;
        Ok((ack, response))
    }

    pub async fn autopilot_version(
        &self,
        target_system: Option<u8>,
        target_component: Option<u8>,
        timeout: Duration,
    ) -> Result<Value, OperationError> {
        let (ack, response) = self
            .request_response(
                "AUTOPILOT_VERSION",
                target_system,
                target_component,
                timeout,
            )
            .await?;
        let version = autopilot_version_data(&response)?;
        let mut result = json!({
            "message": "AUTOPILOT_VERSION",
            "system_id": response.system_id,
            "component_id": response.component_id,
            "fields": response.fields,
            "ack_cursor": format_cursor(ack.journal_sequence),
            "after_cursor": format_cursor(response.journal_sequence),
        });
        let object = result.as_object_mut().expect("version result is an object");
        object.extend(response.fields.clone());
        object.insert(
            "capability_names".to_owned(),
            capability_names(version.capabilities),
        );
        Ok(result)
    }

    pub async fn capabilities(&self, timeout: Duration) -> Result<Value, OperationError> {
        let (_, response) = self
            .request_response("AUTOPILOT_VERSION", None, None, timeout)
            .await?;
        let version = autopilot_version_data(&response)?;
        let capabilities = version.capabilities;
        Ok(json!({
            "system_id": response.system_id,
            "component_id": response.component_id,
            "capabilities": capabilities,
            "capability_names": capability_names(capabilities),
            "services": capability_services(capabilities),
            "firmware": {
                "flight_sw_version": version.flight_sw_version,
                "middleware_sw_version": version.middleware_sw_version,
                "os_sw_version": version.os_sw_version,
                "board_version": version.board_version,
                "vendor_id": version.vendor_id,
                "product_id": version.product_id,
                "uid": version.uid,
            },
        }))
    }

    pub async fn message_interval(
        &self,
        message: &str,
        timeout: Duration,
    ) -> Result<Value, OperationError> {
        let message_name = message.to_uppercase();
        let message_id = message_id_from_name(&message_name).ok_or_else(|| {
            OperationError::Invalid(format!("unknown MAVLink message {message:?}"))
        })?;
        let deadline = time::Instant::now() + timeout;
        let _guard = lock_until(&self.command_lock, deadline).await?;
        let (target_system, target_component) = self.targets(None, None);
        let mut receiver = self.link.subscribe_messages();
        self.send_command_long(
            GET_MESSAGE_INTERVAL,
            &[f64::from(message_id)],
            target_system,
            target_component,
        )
        .await?;
        let (ack, response) = wait_for_command_response(
            &mut receiver,
            remaining_until(deadline)?,
            GET_MESSAGE_INTERVAL,
            |candidate| {
                matches!(
                    &candidate.message,
                    MavMessage::MESSAGE_INTERVAL(interval)
                        if u32::from(interval.message_id) == message_id
                )
            },
        )
        .await?;
        ensure_accepted(&ack, &format!("interval request for {message_name}"))?;
        let MavMessage::MESSAGE_INTERVAL(interval) = &response.message else {
            return Err(OperationError::Invalid(
                "MESSAGE_INTERVAL is invalid".to_owned(),
            ));
        };
        let interval_us = i64::from(interval.interval_us);
        Ok(json!({
            "message": message_name,
            "message_id": message_id,
            "interval_us": interval_us,
            "rate_hz": if interval_us > 0 {
                Some(1_000_000.0 / interval_us as f64)
            } else {
                None
            },
            "ack_cursor": format_cursor(ack.journal_sequence),
            "after_cursor": format_cursor(response.journal_sequence),
        }))
    }

    pub async fn list_files(
        &self,
        path: &str,
        timeout: Duration,
    ) -> Result<Vec<Value>, OperationError> {
        let (result, _) = self
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

    pub async fn download_file(
        &self,
        path: &str,
        verify_crc: bool,
        timeout: Duration,
    ) -> Result<DownloadedFile, OperationError> {
        let (result, cursor) = self
            .execute_mavftp(
                |sequence| MavFtpOperation::download(sequence, path, verify_crc),
                timeout,
            )
            .await?;
        let MavFtpResult::Download { content, crc32 } = result else {
            return Err(OperationError::Invalid(
                "MAVFTP download returned an unexpected result".to_owned(),
            ));
        };
        Ok(DownloadedFile {
            path: path.to_owned(),
            content,
            crc32,
            after_cursor: format_cursor(cursor),
        })
    }

    pub async fn upload_file(
        &self,
        path: &str,
        content: Vec<u8>,
        timeout: Duration,
    ) -> Result<usize, OperationError> {
        let (result, _) = self
            .execute_mavftp(
                |sequence| MavFtpOperation::upload(sequence, path, content),
                timeout,
            )
            .await?;
        let MavFtpResult::Uploaded(bytes) = result else {
            return Err(OperationError::Invalid(
                "MAVFTP upload returned an unexpected result".to_owned(),
            ));
        };
        Ok(bytes)
    }

    pub async fn remove_file(&self, path: &str, timeout: Duration) -> Result<(), OperationError> {
        let (result, _) = self
            .execute_mavftp(|sequence| MavFtpOperation::remove(sequence, path), timeout)
            .await?;
        if result != MavFtpResult::Removed {
            return Err(OperationError::Invalid(
                "MAVFTP remove returned an unexpected result".to_owned(),
            ));
        }
        Ok(())
    }

    pub async fn create_directory(
        &self,
        path: &str,
        timeout: Duration,
    ) -> Result<bool, OperationError> {
        let (result, _) = self
            .execute_mavftp(|sequence| MavFtpOperation::mkdir(sequence, path), timeout)
            .await?;
        let MavFtpResult::DirectoryCreated(created) = result else {
            return Err(OperationError::Invalid(
                "MAVFTP mkdir returned an unexpected result".to_owned(),
            ));
        };
        Ok(created)
    }

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

    pub async fn download_log(
        &self,
        log_id: u16,
        packet_timeout: Duration,
        max_retries: u32,
    ) -> Result<DownloadedLog, OperationError> {
        let _guard = self.log_lock.lock().await;
        let (entry, _) = self
            .request_log_entries(log_id, log_id, packet_timeout)
            .await?
            .into_iter()
            .find(|(entry, _)| entry.id == log_id)
            .ok_or_else(|| OperationError::NotFound(format!("DataFlash log {log_id}")))?;
        let (size, time_utc) = (entry.size, entry.time_utc);
        let (target_system, target_component) = self.targets(None, None);
        let target = DataFlashTarget {
            system: target_system,
            component: target_component,
        };
        let mut download = LogDownload::new(target, log_id, size, max_retries);
        let mut receiver = self.link.subscribe_messages();

        let transfer = async {
            while !download.is_complete() {
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
                            tokio::time::sleep(packet_timeout).await;
                            download.on_timeout()?;
                        }
                        result => result?,
                    },
                    Err(OperationError::Timeout) => download.on_timeout()?,
                    Err(error) => return Err(error),
                }
            }
            download
                .into_bytes()
                .ok_or_else(|| OperationError::Invalid("log transfer is incomplete".to_owned()))
        }
        .await;

        let end_cursor = self
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
        let content = transfer?;
        Ok(DownloadedLog {
            id: log_id,
            size,
            time_utc,
            content,
            after_cursor: format_cursor(end_cursor?),
        })
    }

    async fn execute_mavftp(
        &self,
        create: impl FnOnce(u16) -> MavFtpOperation,
        timeout: Duration,
    ) -> Result<(MavFtpResult, u64), OperationError> {
        let mut sequence = self.ftp_sequence.lock().await;
        let mut operation = create(*sequence);
        let result = self.run_mavftp(&mut operation, timeout).await;
        *sequence = operation.next_sequence();
        result
    }

    async fn run_mavftp(
        &self,
        operation: &mut MavFtpOperation,
        timeout: Duration,
    ) -> Result<(MavFtpResult, u64), OperationError> {
        let deadline = time::Instant::now() + timeout;
        let (target_system, target_component) = self.targets(None, None);
        let mut receiver = self.link.subscribe_messages();
        let mut last_cursor = 0;
        loop {
            if let Some(result) = operation.result().cloned() {
                return Ok((result, last_cursor));
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
            last_cursor = reply.journal_sequence;
            let (_, reply_payload) = ftp_payload(&reply).expect("wait predicate selected FTP");
            operation.handle_reply(Packet::decode(reply_payload)?)?;
        }
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

    pub async fn set_message_rates(
        &self,
        rates: &Map<String, Value>,
        timeout: Duration,
    ) -> Result<Vec<Value>, OperationError> {
        let mut configured = Vec::with_capacity(rates.len());
        for (requested_name, rate) in rates {
            let message_name = requested_name.to_uppercase();
            let message_id = message_id_from_name(&message_name).ok_or_else(|| {
                OperationError::Invalid(format!("unknown MAVLink message {requested_name:?}"))
            })?;
            let (rate_hz, interval_us) = if rate.is_null() {
                (Value::Null, 0_i64)
            } else {
                let rate_hz = rate.as_f64().ok_or_else(|| {
                    OperationError::Invalid(format!(
                        "rate for {requested_name} must be numeric or null"
                    ))
                })?;
                let interval = if rate_hz == 0.0 {
                    -1
                } else if rate_hz > 0.0 {
                    (1_000_000.0 / rate_hz).round() as i64
                } else {
                    return Err(OperationError::Invalid(format!(
                        "rate for {requested_name} cannot be negative"
                    )));
                };
                (Value::from(rate_hz), interval)
            };
            let result = self
                .command(
                    MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                    &[f64::from(message_id), interval_us as f64],
                    None,
                    None,
                    timeout,
                )
                .await?;
            if result.result != MavResult::MAV_RESULT_ACCEPTED {
                return Err(OperationError::Invalid(format!(
                    "vehicle rejected {message_name} rate with {:?}",
                    result.result
                )));
            }
            configured.push(json!({
                "message": message_name,
                "rate_hz": rate_hz,
                "interval_us": interval_us,
                "after_cursor": result.after_cursor,
            }));
        }
        Ok(configured)
    }

    fn targets(&self, target_system: Option<u8>, target_component: Option<u8>) -> (u8, u8) {
        let status = self.link.status();
        (
            target_system.unwrap_or(if status.target_system == 0 {
                1
            } else {
                status.target_system
            }),
            target_component.unwrap_or(if status.target_component == 0 {
                1
            } else {
                status.target_component
            }),
        )
    }

    async fn send_command_long(
        &self,
        command: MavCmd,
        params: &[f64],
        target_system: u8,
        target_component: u8,
    ) -> Result<u64, OperationError> {
        Ok(self
            .link
            .send_message(
                command_long(command, params, target_system, target_component, 0),
                None,
                None,
            )
            .await?)
    }
}

async fn lock_until<'a>(
    lock: &'a Mutex<()>,
    deadline: time::Instant,
) -> Result<MutexGuard<'a, ()>, OperationError> {
    time::timeout_at(deadline, lock.lock())
        .await
        .map_err(|_| OperationError::Timeout)
}

fn remaining_until(deadline: time::Instant) -> Result<Duration, OperationError> {
    let remaining = deadline.saturating_duration_since(time::Instant::now());
    if remaining.is_zero() {
        Err(OperationError::Timeout)
    } else {
        Ok(remaining)
    }
}

async fn wait_for_message(
    receiver: &mut broadcast::Receiver<Arc<ReceivedMessage>>,
    timeout: Duration,
    predicate: impl Fn(&ReceivedMessage) -> bool,
) -> Result<Arc<ReceivedMessage>, OperationError> {
    let deadline = time::Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(time::Instant::now());
        if remaining.is_zero() {
            return Err(OperationError::Timeout);
        }
        let message = recv_message(receiver, remaining).await?;
        if predicate(&message) {
            return Ok(message);
        }
    }
}

async fn wait_for_command_response(
    receiver: &mut broadcast::Receiver<Arc<ReceivedMessage>>,
    timeout: Duration,
    command: MavCmd,
    response_predicate: impl Fn(&ReceivedMessage) -> bool,
) -> Result<(Arc<ReceivedMessage>, Arc<ReceivedMessage>), OperationError> {
    let deadline = time::Instant::now() + timeout;
    let mut acknowledgement = None;
    let mut response = None;
    loop {
        if acknowledgement.is_some() && response.is_some() {
            return Ok((
                acknowledgement.take().expect("acknowledgement is present"),
                response.take().expect("response is present"),
            ));
        }
        let remaining = deadline.saturating_duration_since(time::Instant::now());
        if remaining.is_zero() {
            return Err(OperationError::Timeout);
        }
        let message = recv_message(receiver, remaining).await?;
        if acknowledges(&message, command) {
            acknowledgement = Some(message);
        } else if response_predicate(&message) {
            response = Some(message);
        }
    }
}

async fn recv_message(
    receiver: &mut broadcast::Receiver<Arc<ReceivedMessage>>,
    timeout: Duration,
) -> Result<Arc<ReceivedMessage>, OperationError> {
    match time::timeout(timeout, receiver.recv()).await {
        Err(_) => Err(OperationError::Timeout),
        Ok(Ok(message)) => Ok(message),
        Ok(Err(broadcast::error::RecvError::Closed)) => Err(OperationError::ReceiveClosed),
        Ok(Err(broadcast::error::RecvError::Lagged(count))) => {
            Err(OperationError::ReceiveLagged(count))
        }
    }
}

fn command_long(
    command: MavCmd,
    params: &[f64],
    target_system: u8,
    target_component: u8,
    confirmation: u8,
) -> MavMessage {
    let param = |index: usize| params.get(index).copied().unwrap_or_default() as f32;
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: param(0),
        param2: param(1),
        param3: param(2),
        param4: param(3),
        param5: param(4),
        param6: param(5),
        param7: param(6),
        command,
        target_system,
        target_component,
        confirmation,
    })
}

fn param_set(
    name: &str,
    value: f64,
    param_type: MavParamType,
    target_system: u8,
    target_component: u8,
) -> MavMessage {
    MavMessage::PARAM_SET(PARAM_SET_DATA {
        param_value: value as f32,
        target_system,
        target_component,
        param_id: CharArray::from(name),
        param_type,
    })
}

fn command_ack(message: &ReceivedMessage) -> Option<&COMMAND_ACK_DATA> {
    match &message.message {
        MavMessage::COMMAND_ACK(ack) => Some(ack),
        _ => None,
    }
}

fn autopilot_version_data(
    message: &ReceivedMessage,
) -> Result<&AUTOPILOT_VERSION_DATA, OperationError> {
    match &message.message {
        MavMessage::AUTOPILOT_VERSION(version) => Ok(version),
        _ => Err(OperationError::Invalid(
            "response is not AUTOPILOT_VERSION".to_owned(),
        )),
    }
}

fn acknowledges(message: &ReceivedMessage, command: MavCmd) -> bool {
    command_ack(message).is_some_and(|ack| ack.command == command)
}

fn ensure_accepted(ack: &ReceivedMessage, what: &str) -> Result<(), OperationError> {
    let result = command_ack(ack).map_or(MavResult::MAV_RESULT_FAILED, |ack| ack.result);
    if result == MavResult::MAV_RESULT_ACCEPTED {
        Ok(())
    } else {
        Err(OperationError::Invalid(format!(
            "vehicle rejected {what} with {result:?}"
        )))
    }
}

fn param_value(message: &ReceivedMessage) -> Option<&PARAM_VALUE_DATA> {
    match &message.message {
        MavMessage::PARAM_VALUE(value) => Some(value),
        _ => None,
    }
}

fn log_data(message: &ReceivedMessage) -> Option<&LOG_DATA_DATA> {
    match &message.message {
        MavMessage::LOG_DATA(data) => Some(data),
        _ => None,
    }
}

/// The target component and 251-byte payload of a FILE_TRANSFER_PROTOCOL message.
fn ftp_payload(message: &ReceivedMessage) -> Option<(u8, &[u8])> {
    match &message.message {
        MavMessage::FILE_TRANSFER_PROTOCOL(ftp) => Some((ftp.target_component, &ftp.payload)),
        _ => None,
    }
}

fn parameter_matches(
    message: &ReceivedMessage,
    name: &str,
    target_system: u8,
    target_component: u8,
) -> bool {
    message.system_id == target_system
        && message.component_id == target_component
        && param_value(message).is_some_and(|value| value.param_id.to_str() == Ok(name))
}

fn parameter_result(message: &ReceivedMessage) -> Result<ParameterResult, OperationError> {
    let value = param_value(message)
        .ok_or_else(|| OperationError::Invalid("message is not PARAM_VALUE".to_owned()))?;
    Ok(ParameterResult {
        name: value
            .param_id
            .to_str()
            .map_err(|_| OperationError::Invalid("PARAM_VALUE param_id is not text".to_owned()))?
            .to_owned(),
        value: f64::from(value.param_value),
        param_type: value.param_type,
        index: i64::from(value.param_index),
        count: u64::from(value.param_count),
        after_cursor: format_cursor(message.journal_sequence),
    })
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

fn ftp_reply_matches(payload: &[u8], expected_sequence: u16, expected_request_opcode: u8) -> bool {
    Packet::decode(payload).ok().is_some_and(|reply| {
        reply.sequence == expected_sequence && reply.request_opcode == expected_request_opcode
    })
}

fn normalize_parameter_name(name: &str) -> Result<String, OperationError> {
    let name = name.trim().to_uppercase();
    let mut characters = name.bytes();
    let valid = characters
        .next()
        .is_some_and(|character| character.is_ascii_uppercase())
        && characters.all(|character| {
            character.is_ascii_uppercase() || character.is_ascii_digit() || character == b'_'
        });
    if name.len() > 16 || !valid {
        return Err(OperationError::Invalid(
            "parameter names must match [A-Z][A-Z0-9_]{0,15}".to_owned(),
        ));
    }
    Ok(name)
}

fn format_cursor(sequence: u64) -> String {
    format!("v1:{sequence}")
}

fn capability_names(capabilities: MavProtocolCapability) -> Value {
    Value::Array(
        capabilities
            .iter_names()
            .map(|(name, _)| Value::from(name))
            .collect(),
    )
}

fn capability_services(capabilities: MavProtocolCapability) -> Value {
    json!({
        "mavftp": capabilities.contains(MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_FTP),
        "mission_int": capabilities
            .contains(MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_INT),
        "parameter_float": capabilities.contains(PARAM_FLOAT),
        "command_int": capabilities
            .contains(MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codec::message_fields;

    fn received(message: MavMessage, journal_sequence: u64) -> Arc<ReceivedMessage> {
        use linkhub_dialect::Message as _;
        Arc::new(ReceivedMessage {
            journal_sequence,
            ingest_time_ns: journal_sequence,
            link_id: "test".to_owned(),
            system_id: 1,
            component_id: 1,
            sequence: journal_sequence as u8,
            message_id: message.message_id(),
            name: message.message_name().to_owned(),
            fields: message_fields(&message).expect("message fields"),
            message,
        })
    }

    fn command_ack_message(command: MavCmd, result: MavResult) -> MavMessage {
        MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command,
            result,
            ..COMMAND_ACK_DATA::default()
        })
    }

    #[tokio::test]
    async fn command_response_accepts_response_before_acknowledgement() {
        let (sender, _) = broadcast::channel(4);
        let mut receiver = sender.subscribe();
        sender
            .send(received(
                MavMessage::AUTOPILOT_VERSION(Default::default()),
                10,
            ))
            .expect("response");
        sender
            .send(received(
                command_ack_message(
                    MavCmd::MAV_CMD_REQUEST_MESSAGE,
                    MavResult::MAV_RESULT_ACCEPTED,
                ),
                11,
            ))
            .expect("acknowledgement");

        let (acknowledgement, response) = wait_for_command_response(
            &mut receiver,
            Duration::from_millis(100),
            MavCmd::MAV_CMD_REQUEST_MESSAGE,
            |message| message.name == "AUTOPILOT_VERSION",
        )
        .await
        .expect("transaction");

        assert_eq!(acknowledgement.journal_sequence, 11);
        assert_eq!(response.journal_sequence, 10);
    }

    #[test]
    fn rejected_acknowledgement_reports_the_typed_result() {
        let ack = received(
            command_ack_message(
                MavCmd::MAV_CMD_REQUEST_MESSAGE,
                MavResult::MAV_RESULT_DENIED,
            ),
            1,
        );

        let error = ensure_accepted(&ack, "request for HOME_POSITION").unwrap_err();

        assert_eq!(
            error.to_string(),
            "invalid operation: vehicle rejected request for HOME_POSITION with MAV_RESULT_DENIED"
        );
    }

    #[tokio::test]
    async fn serialized_operation_lock_honors_deadline() {
        let lock = Mutex::new(());
        let _guard = lock.lock().await;
        let started = time::Instant::now();

        let result = lock_until(&lock, started + Duration::from_millis(25)).await;

        assert!(matches!(result, Err(OperationError::Timeout)));
        assert!(started.elapsed() < Duration::from_millis(250));
    }

    #[test]
    fn parameter_names_are_normalized_and_bounded() {
        assert_eq!(
            normalize_parameter_name(" rawes_mode ").expect("valid name"),
            "RAWES_MODE"
        );
        assert!(normalize_parameter_name("").is_err());
        assert!(normalize_parameter_name("12345678901234567").is_err());
        assert!(normalize_parameter_name("RAWES_\u{2603}").is_err());
        assert!(normalize_parameter_name("=5000").is_err());
        assert!(normalize_parameter_name("RAWES MODE").is_err());
        assert!(normalize_parameter_name("RAWES-MODE").is_err());
        assert!(normalize_parameter_name("_RAWES_MODE").is_err());
    }

    #[test]
    fn parameter_results_are_typed() {
        let message = received(
            MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                param_value: 4.5,
                param_count: 700,
                param_index: 12,
                param_id: CharArray::from("RAWES_MODE"),
                param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
            }),
            3,
        );

        let result = parameter_result(&message).expect("parameter");

        assert_eq!(result.name, "RAWES_MODE");
        assert_eq!(result.value, 4.5);
        assert_eq!(result.param_type, MavParamType::MAV_PARAM_TYPE_REAL32);
        assert_eq!((result.index, result.count), (12, 700));
        assert!(parameter_matches(&message, "RAWES_MODE", 1, 1));
        assert!(!parameter_matches(&message, "RAWES_OTHER", 1, 1));
    }

    #[test]
    fn capability_names_use_official_mavlink_flags() {
        let names =
            capability_names(PARAM_FLOAT | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_FTP);
        let names = names.as_array().expect("names are an array");
        assert_eq!(names.len(), 2);
        assert!(names.contains(&Value::String("MAV_PROTOCOL_CAPABILITY_FTP".to_owned())));
        assert!(names.contains(&Value::String(
            "MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT".to_owned()
        )));
    }

    #[test]
    fn capability_services_map_to_their_mavlink_bits() {
        // Bits 2 (PARAM_FLOAT) and 32 (FTP): the combination a real ArduCopter reports.
        let services =
            capability_services(PARAM_FLOAT | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_FTP);

        assert_eq!(
            services,
            json!({
                "mavftp": true,
                "mission_int": false,
                "parameter_float": true,
                "command_int": false,
            })
        );
        assert_eq!(PARAM_FLOAT.bits(), 2);
    }

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
