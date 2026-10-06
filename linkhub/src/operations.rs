use std::{collections::BTreeMap, sync::Arc, time::Duration};

use serde::Serialize;
use serde_json::{Map, Value, json};
use tokio::{
    sync::{Mutex, MutexGuard, broadcast},
    time,
};

use crate::{
    codec::message_id_from_name,
    dataflash::{
        DataFlashError, DataPacket, LogDownload, LogEntry, LogList, Target as DataFlashTarget,
    },
    mavftp::{MavFtpError, Operation as MavFtpOperation, OperationResult as MavFtpResult, Packet},
    mavlink::{LinkError, MavlinkLinkHandle, ReceivedMessage},
    records::wall_time_ns,
};
use linkhub_mavio_dialect::dialects::ardupilotmega::enums::MavProtocolCapability;

const MAV_CMD_SET_MESSAGE_INTERVAL: u32 = 511;
const MAV_CMD_GET_MESSAGE_INTERVAL: u32 = 510;
const MAV_CMD_REQUEST_MESSAGE: u32 = 512;
const MAVFTP_SOURCE_COMPONENT: u8 = 190;
const MAVFTP_PACKET_RETRIES: usize = 3;

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
    pub command: u32,
    pub result: u64,
    pub progress: u64,
    pub status: &'static str,
    pub after_cursor: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct ParameterResult {
    pub name: String,
    pub value: f64,
    #[serde(rename = "type")]
    pub param_type: u64,
    pub index: i64,
    pub count: u64,
    pub after_cursor: String,
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
        command: u32,
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
        let mut fields = Map::new();
        fields.insert("target_system".to_owned(), Value::from(target_system));
        fields.insert("target_component".to_owned(), Value::from(target_component));
        fields.insert("command".to_owned(), Value::from(command));
        for index in 0..7 {
            fields.insert(
                format!("param{}", index + 1),
                Value::from(params.get(index).copied().unwrap_or_default()),
            );
        }

        let mut receiver = self.link.subscribe_messages();
        let mut confirmation = 0_u8;
        let message = loop {
            fields.insert("confirmation".to_owned(), Value::from(confirmation));
            self.link
                .send_message("COMMAND_LONG", &fields, None, None)
                .await?;
            let remaining = deadline.saturating_duration_since(time::Instant::now());
            if remaining.is_zero() {
                return Err(OperationError::Timeout);
            }
            match wait_for_message(
                &mut receiver,
                remaining.min(Duration::from_secs(1)),
                |message| {
                    message.name == "COMMAND_ACK"
                        && message.fields.get("command").and_then(Value::as_u64)
                            == Some(u64::from(command))
                },
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
        Ok(CommandResult {
            command,
            result: message
                .fields
                .get("result")
                .and_then(Value::as_u64)
                .unwrap_or_default(),
            progress: message
                .fields
                .get("progress")
                .and_then(Value::as_u64)
                .unwrap_or_default(),
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
                "PARAM_REQUEST_READ",
                &object(json!({
                    "target_system": target_system,
                    "target_component": target_component,
                    "param_id": normalized,
                    "param_index": -1,
                }))?,
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
        param_type: u64,
        timeout: Duration,
    ) -> Result<ParameterResult, OperationError> {
        let deadline = time::Instant::now() + timeout;
        let _guard = lock_until(&self.parameter_lock, deadline).await?;
        let normalized = normalize_parameter_name(name)?;
        let (target_system, target_component) = self.targets(None, None);
        let mut receiver = self.link.subscribe_messages();
        self.link
            .send_message(
                "PARAM_SET",
                &object(json!({
                    "target_system": target_system,
                    "target_component": target_component,
                    "param_id": normalized,
                    "param_value": value,
                    "param_type": param_type,
                }))?,
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
                "PARAM_REQUEST_LIST",
                &object(json!({
                    "target_system": target_system,
                    "target_component": target_component,
                }))?,
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
            if message.name != "PARAM_VALUE"
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
        parameters: &[Value],
        timeout: Duration,
        retries: usize,
    ) -> Result<Vec<ParameterResult>, OperationError> {
        let deadline = time::Instant::now() + timeout;
        let _guard = lock_until(&self.parameter_lock, deadline).await?;
        if parameters.is_empty() {
            return Ok(Vec::new());
        }
        let (target_system, target_component) = self.targets(None, None);
        let mut requests = Vec::with_capacity(parameters.len());
        for parameter in parameters {
            let parameter = parameter.as_object().ok_or_else(|| {
                OperationError::Invalid("each parameter must be an object".to_owned())
            })?;
            let name = normalize_parameter_name(
                parameter
                    .get("name")
                    .and_then(Value::as_str)
                    .ok_or_else(|| {
                        OperationError::Invalid("parameter name is required".to_owned())
                    })?,
            )?;
            if requests.iter().any(|(existing, _, _)| existing == &name) {
                return Err(OperationError::Invalid(format!(
                    "duplicate parameter {name}"
                )));
            }
            let value = parameter
                .get("value")
                .and_then(Value::as_f64)
                .ok_or_else(|| OperationError::Invalid(format!("{name} value must be numeric")))?;
            let param_type = parameter.get("type").and_then(Value::as_u64).unwrap_or(9);
            requests.push((name, value, param_type));
        }

        let mut receiver = self.link.subscribe_messages();
        let mut pending: BTreeMap<String, (f64, u64)> = requests
            .iter()
            .map(|(name, value, param_type)| (name.clone(), (*value, *param_type)))
            .collect();
        let mut results = BTreeMap::new();
        for attempt in 0..=retries {
            for (name, (value, param_type)) in &pending {
                self.link
                    .send_message(
                        "PARAM_SET",
                        &object(json!({
                            "target_system": target_system,
                            "target_component": target_component,
                            "param_id": name,
                            "param_value": value,
                            "param_type": param_type,
                        }))?,
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
                if message.name != "PARAM_VALUE"
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
        let message_id = message_id_from_name(&message_name).ok_or_else(|| {
            OperationError::Invalid(format!("unknown MAVLink message {message:?}"))
        })?;
        let deadline = time::Instant::now() + timeout;
        let _guard = lock_until(&self.command_lock, deadline).await?;
        let (target_system, target_component) = self.targets(target_system, target_component);
        let mut receiver = self.link.subscribe_messages();
        self.send_command_long(
            MAV_CMD_REQUEST_MESSAGE,
            &[f64::from(message_id)],
            target_system,
            target_component,
        )
        .await?;
        let (ack, response) = wait_for_command_response(
            &mut receiver,
            remaining_until(deadline)?,
            MAV_CMD_REQUEST_MESSAGE,
            |candidate| {
                candidate.name == message_name
                    && candidate.system_id == target_system
                    && (target_component == 0 || candidate.component_id == target_component)
            },
        )
        .await?;
        let result = ack
            .fields
            .get("result")
            .and_then(Value::as_u64)
            .unwrap_or_default();
        if result != 0 {
            return Err(OperationError::Invalid(format!(
                "vehicle rejected request for {message_name} with MAV_RESULT {result}"
            )));
        }
        Ok(json!({
            "message": message_name,
            "system_id": response.system_id,
            "component_id": response.component_id,
            "fields": response.fields,
            "ack_cursor": format_cursor(ack.journal_sequence),
            "after_cursor": format_cursor(response.journal_sequence),
        }))
    }

    pub async fn autopilot_version(
        &self,
        target_system: Option<u8>,
        target_component: Option<u8>,
        timeout: Duration,
    ) -> Result<Value, OperationError> {
        let mut result = self
            .request_message(
                "AUTOPILOT_VERSION",
                target_system,
                target_component,
                timeout,
            )
            .await?;
        let object = result
            .as_object_mut()
            .ok_or_else(|| OperationError::Invalid("invalid version response".to_owned()))?;
        let fields = object
            .get("fields")
            .and_then(Value::as_object)
            .cloned()
            .ok_or_else(|| OperationError::Invalid("version fields are missing".to_owned()))?;
        object.extend(fields.clone());
        let capabilities = fields
            .get("capabilities")
            .and_then(Value::as_u64)
            .ok_or_else(|| OperationError::Invalid("capabilities are missing".to_owned()))?;
        object.insert(
            "capability_names".to_owned(),
            Value::Array(capability_names(capabilities)),
        );
        Ok(result)
    }

    pub async fn capabilities(&self, timeout: Duration) -> Result<Value, OperationError> {
        let version = self.autopilot_version(None, None, timeout).await?;
        let capabilities = version
            .get("capabilities")
            .and_then(Value::as_u64)
            .ok_or_else(|| OperationError::Invalid("capabilities are missing".to_owned()))?;
        Ok(json!({
            "system_id": version["system_id"],
            "component_id": version["component_id"],
            "capabilities": capabilities,
            "capability_names": capability_names(capabilities),
            "services": {
                "mavftp": capabilities & 32 != 0,
                "mission_int": capabilities & 4 != 0,
                "parameter_float": capabilities & 2 != 0,
                "command_int": capabilities & 8 != 0,
            },
            "firmware": {
                "flight_sw_version": version["flight_sw_version"],
                "middleware_sw_version": version["middleware_sw_version"],
                "os_sw_version": version["os_sw_version"],
                "board_version": version["board_version"],
                "vendor_id": version["vendor_id"],
                "product_id": version["product_id"],
                "uid": version["uid"],
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
            MAV_CMD_GET_MESSAGE_INTERVAL,
            &[f64::from(message_id)],
            target_system,
            target_component,
        )
        .await?;
        let (ack, response) = wait_for_command_response(
            &mut receiver,
            remaining_until(deadline)?,
            MAV_CMD_GET_MESSAGE_INTERVAL,
            |candidate| {
                candidate.name == "MESSAGE_INTERVAL"
                    && candidate.fields.get("message_id").and_then(Value::as_u64)
                        == Some(u64::from(message_id))
            },
        )
        .await?;
        let result = ack
            .fields
            .get("result")
            .and_then(Value::as_u64)
            .unwrap_or_default();
        if result != 0 {
            return Err(OperationError::Invalid(format!(
                "vehicle rejected interval request for {message_name} with MAV_RESULT {result}"
            )));
        }
        let interval_us = response
            .fields
            .get("interval_us")
            .and_then(Value::as_i64)
            .ok_or_else(|| OperationError::Invalid("MESSAGE_INTERVAL is invalid".to_owned()))?;
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
        self.request_log_entries(0, u16::MAX, timeout).await
    }

    pub async fn download_log(
        &self,
        log_id: u16,
        packet_timeout: Duration,
        max_retries: u32,
    ) -> Result<DownloadedLog, OperationError> {
        let _guard = self.log_lock.lock().await;
        let entries = self
            .request_log_entries(log_id, log_id, packet_timeout)
            .await?;
        let entry = entries
            .iter()
            .find(|entry| entry.get("id").and_then(Value::as_u64) == Some(u64::from(log_id)))
            .ok_or_else(|| OperationError::NotFound(format!("DataFlash log {log_id}")))?;
        let entry = entry
            .as_object()
            .ok_or_else(|| OperationError::Invalid("invalid log entry".to_owned()))?;
        let size = value_u32(entry, "size")?;
        let time_utc = value_u32(entry, "time_utc")?;
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
                            "LOG_REQUEST_DATA",
                            &object(json!({
                                "target_system": request.target.system,
                                "target_component": request.target.component,
                                "id": request.id,
                                "ofs": request.offset,
                                "count": request.count,
                            }))?,
                            None,
                            None,
                        )
                        .await?;
                }
                match wait_for_message(&mut receiver, packet_timeout, |message| {
                    message.name == "LOG_DATA"
                        && message.fields.get("id").and_then(Value::as_u64)
                            == Some(u64::from(log_id))
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
                "LOG_REQUEST_END",
                &object(json!({
                    "target_system": target.system,
                    "target_component": target.component,
                }))?,
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
            let fields = object(json!({
                "target_network": 0,
                "target_system": target_system,
                "target_component": target_component,
                "payload": payload.to_vec(),
            }))?;
            let mut retries = 0;
            let reply = loop {
                self.link
                    .send_message(
                        "FILE_TRANSFER_PROTOCOL",
                        &fields,
                        None,
                        Some(MAVFTP_SOURCE_COMPONENT),
                    )
                    .await?;
                let remaining = deadline.saturating_duration_since(time::Instant::now());
                if remaining.is_zero() {
                    return Err(OperationError::Timeout);
                }
                match wait_for_message(
                    &mut receiver,
                    remaining.min(Duration::from_secs(2)),
                    |message| {
                        message.name == "FILE_TRANSFER_PROTOCOL"
                            && message.component_id == target_component
                            && message
                                .fields
                                .get("target_component")
                                .and_then(Value::as_u64)
                                == Some(u64::from(MAVFTP_SOURCE_COMPONENT))
                            && ftp_reply_matches(
                                message,
                                expected_reply_sequence,
                                expected_request_opcode,
                            )
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
            operation.handle_reply(Packet::decode(&byte_array(&reply, "payload", 251)?)?)?;
        }
    }

    async fn request_log_entries(
        &self,
        start: u16,
        end: u16,
        timeout: Duration,
    ) -> Result<Vec<Value>, OperationError> {
        let (target_system, target_component) = self.targets(None, None);
        let target = DataFlashTarget {
            system: target_system,
            component: target_component,
        };
        let mut list = LogList::new(target, start, end)?;
        let mut receiver = self.link.subscribe_messages();
        self.link
            .send_message(
                "LOG_REQUEST_LIST",
                &object(json!({
                    "target_system": target.system,
                    "target_component": target.component,
                    "start": start,
                    "end": end,
                }))?,
                None,
                None,
            )
            .await?;
        let mut entries = BTreeMap::new();
        while !list.is_complete() {
            let message = wait_for_message(&mut receiver, timeout, |message| {
                message.name == "LOG_ENTRY"
                    && message.system_id == target.system
                    && (target.component == 0 || message.component_id == target.component)
            })
            .await?;
            let entry = log_entry(&message)?;
            list.receive(entry.clone())?;
            if entry.num_logs == 0 {
                return Ok(Vec::new());
            }
            entries.insert(
                entry.id,
                json!({
                    "id": entry.id,
                    "size": entry.size,
                    "time_utc": entry.time_utc,
                    "num_logs": entry.num_logs,
                    "last_log_num": entry.last_log_num,
                    "after_cursor": format_cursor(message.journal_sequence),
                }),
            );
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
                    MAV_CMD_SET_MESSAGE_INTERVAL,
                    &[f64::from(message_id), interval_us as f64],
                    None,
                    None,
                    timeout,
                )
                .await?;
            if result.result != 0 {
                return Err(OperationError::Invalid(format!(
                    "vehicle rejected {message_name} rate with MAV_RESULT {}",
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
        command: u32,
        params: &[f64],
        target_system: u8,
        target_component: u8,
    ) -> Result<u64, OperationError> {
        let mut fields = Map::new();
        fields.insert("target_system".to_owned(), Value::from(target_system));
        fields.insert("target_component".to_owned(), Value::from(target_component));
        fields.insert("command".to_owned(), Value::from(command));
        fields.insert("confirmation".to_owned(), Value::from(0));
        for index in 0..7 {
            fields.insert(
                format!("param{}", index + 1),
                Value::from(params.get(index).copied().unwrap_or_default()),
            );
        }
        Ok(self
            .link
            .send_message("COMMAND_LONG", &fields, None, None)
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
    command: u32,
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
        if message.name == "COMMAND_ACK"
            && message.fields.get("command").and_then(Value::as_u64) == Some(u64::from(command))
        {
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

fn parameter_matches(
    message: &ReceivedMessage,
    name: &str,
    target_system: u8,
    target_component: u8,
) -> bool {
    message.name == "PARAM_VALUE"
        && message.system_id == target_system
        && message.component_id == target_component
        && message.fields.get("param_id").and_then(Value::as_str) == Some(name)
}

fn parameter_result(message: &ReceivedMessage) -> Result<ParameterResult, OperationError> {
    let field = |name| {
        message
            .fields
            .get(name)
            .ok_or_else(|| OperationError::Invalid(format!("PARAM_VALUE missing {name}")))
    };
    Ok(ParameterResult {
        name: field("param_id")?
            .as_str()
            .ok_or_else(|| OperationError::Invalid("PARAM_VALUE param_id is not text".to_owned()))?
            .to_owned(),
        value: field("param_value")?.as_f64().ok_or_else(|| {
            OperationError::Invalid("PARAM_VALUE value is not numeric".to_owned())
        })?,
        param_type: field("param_type")?
            .as_u64()
            .ok_or_else(|| OperationError::Invalid("PARAM_VALUE type is not numeric".to_owned()))?,
        index: field("param_index")?.as_i64().ok_or_else(|| {
            OperationError::Invalid("PARAM_VALUE index is not numeric".to_owned())
        })?,
        count: field("param_count")?.as_u64().ok_or_else(|| {
            OperationError::Invalid("PARAM_VALUE count is not numeric".to_owned())
        })?,
        after_cursor: format_cursor(message.journal_sequence),
    })
}

fn log_entry(message: &ReceivedMessage) -> Result<LogEntry, OperationError> {
    Ok(LogEntry {
        id: value_u16(&message.fields, "id")?,
        size: value_u32(&message.fields, "size")?,
        time_utc: value_u32(&message.fields, "time_utc")?,
        num_logs: value_u16(&message.fields, "num_logs")?,
        last_log_num: value_u16(&message.fields, "last_log_num")?,
    })
}

fn data_packet(message: &ReceivedMessage) -> Result<DataPacket, OperationError> {
    let bytes = byte_array(message, "data", crate::dataflash::LOG_DATA_LEN)?;
    let mut data = [0; crate::dataflash::LOG_DATA_LEN];
    data.copy_from_slice(&bytes);
    Ok(DataPacket {
        id: value_u16(&message.fields, "id")?,
        offset: value_u32(&message.fields, "ofs")?,
        count: value_u8(&message.fields, "count")?,
        data,
    })
}

fn byte_array(
    message: &ReceivedMessage,
    field: &'static str,
    expected: usize,
) -> Result<Vec<u8>, OperationError> {
    let array = message
        .fields
        .get(field)
        .and_then(Value::as_array)
        .ok_or_else(|| OperationError::Invalid(format!("{} missing {field}", message.name)))?;
    if array.len() != expected {
        return Err(OperationError::Invalid(format!(
            "{} {field} has {} bytes, expected {expected}",
            message.name,
            array.len()
        )));
    }

    array
        .iter()
        .map(|value| {
            value
                .as_u64()
                .and_then(|value| u8::try_from(value).ok())
                .ok_or_else(|| {
                    OperationError::Invalid(format!("{} {field} is not byte data", message.name))
                })
        })
        .collect()
}

fn ftp_reply_matches(
    message: &ReceivedMessage,
    expected_sequence: u16,
    expected_request_opcode: u8,
) -> bool {
    byte_array(message, "payload", 251)
        .ok()
        .and_then(|payload| Packet::decode(&payload).ok())
        .is_some_and(|reply| {
            reply.sequence == expected_sequence && reply.request_opcode == expected_request_opcode
        })
}

fn value_u8(fields: &Map<String, Value>, name: &'static str) -> Result<u8, OperationError> {
    fields
        .get(name)
        .and_then(Value::as_u64)
        .and_then(|value| u8::try_from(value).ok())
        .ok_or_else(|| OperationError::Invalid(format!("{name} is not an unsigned byte")))
}

fn value_u16(fields: &Map<String, Value>, name: &'static str) -> Result<u16, OperationError> {
    fields
        .get(name)
        .and_then(Value::as_u64)
        .and_then(|value| u16::try_from(value).ok())
        .ok_or_else(|| OperationError::Invalid(format!("{name} is not an unsigned 16-bit integer")))
}

fn value_u32(fields: &Map<String, Value>, name: &'static str) -> Result<u32, OperationError> {
    fields
        .get(name)
        .and_then(Value::as_u64)
        .and_then(|value| u32::try_from(value).ok())
        .ok_or_else(|| OperationError::Invalid(format!("{name} is not an unsigned 32-bit integer")))
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

fn object(value: Value) -> Result<Map<String, Value>, OperationError> {
    value
        .as_object()
        .cloned()
        .ok_or_else(|| OperationError::Invalid("internal request is not an object".to_owned()))
}

fn format_cursor(sequence: u64) -> String {
    format!("v1:{sequence}")
}

fn capability_names(bits: u64) -> Vec<Value> {
    MavProtocolCapability::from_bits_retain(bits as u32)
        .iter_names()
        .map(|(name, _)| Value::String(format!("MAV_PROTOCOL_CAPABILITY_{name}")))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn received(name: &str, fields: Value, journal_sequence: u64) -> Arc<ReceivedMessage> {
        Arc::new(ReceivedMessage {
            journal_sequence,
            ingest_time_ns: journal_sequence,
            link_id: "test".to_owned(),
            system_id: 1,
            component_id: 1,
            sequence: journal_sequence as u8,
            message_id: 0,
            name: name.to_owned(),
            fields: fields.as_object().expect("message fields").clone(),
        })
    }

    #[tokio::test]
    async fn command_response_accepts_response_before_acknowledgement() {
        let (sender, _) = broadcast::channel(4);
        let mut receiver = sender.subscribe();
        sender
            .send(received(
                "AUTOPILOT_VERSION",
                json!({"capabilities": 32}),
                10,
            ))
            .expect("response");
        sender
            .send(received(
                "COMMAND_ACK",
                json!({"command": MAV_CMD_REQUEST_MESSAGE, "result": 0}),
                11,
            ))
            .expect("acknowledgement");

        let (acknowledgement, response) = wait_for_command_response(
            &mut receiver,
            Duration::from_millis(100),
            MAV_CMD_REQUEST_MESSAGE,
            |message| message.name == "AUTOPILOT_VERSION",
        )
        .await
        .expect("transaction");

        assert_eq!(acknowledgement.journal_sequence, 11);
        assert_eq!(response.journal_sequence, 10);
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
    fn capability_names_use_official_mavlink_flags() {
        let names = capability_names(2 | 32);
        assert!(names.contains(&Value::String(
            "MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT".to_owned()
        )));
        assert!(names.contains(&Value::String("MAV_PROTOCOL_CAPABILITY_FTP".to_owned())));
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
        let message = received(
            "FILE_TRANSFER_PROTOCOL",
            json!({"payload": packet.encode().expect("packet").to_vec()}),
            1,
        );

        assert!(!ftp_reply_matches(
            &message,
            11,
            crate::mavftp::Opcode::CreateFile as u8
        ));
        assert!(ftp_reply_matches(
            &message,
            11,
            crate::mavftp::Opcode::CreateDirectory as u8
        ));
    }
}
