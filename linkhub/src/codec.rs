use std::io::Cursor;

use mavio::{
    Frame, Receiver,
    error::{Error as MavioError, FrameError},
    io::StdIoReader,
    protocol::{MavLinkVersion, V2, Versionless},
};
use mavspec::rust::spec::{Dialect, IntoPayload, MessageSpec};
use serde_json::{Map, Value};
use thiserror::Error;

pub use linkhub_mavio_dialect::dialects::ardupilotmega::Ardupilotmega as DialectMessage;
use linkhub_mavio_dialect::{
    dialects::ardupilotmega::{enums, messages},
    message_info,
};

#[derive(Clone, Copy, Debug)]
pub struct MavHeader {
    pub system_id: u8,
    pub component_id: u8,
    pub sequence: u8,
}

#[derive(Debug)]
pub struct DecodedMessage {
    pub raw: Vec<u8>,
    pub mavlink_version: u8,
    pub signed: bool,
    pub system_id: u8,
    pub component_id: u8,
    pub sequence: u8,
    pub message_id: u32,
    pub name: String,
    pub fields: Map<String, Value>,
}

#[derive(Debug, Error)]
pub enum CodecError {
    #[error("invalid MAVLink frame: {0}")]
    Mavio(#[from] MavioError),
    #[error("cannot serialize MAVLink frame: {0}")]
    Frame(#[from] FrameError),
    #[error("invalid MAVLink message: {0}")]
    Spec(String),
    #[error("message fields must be a JSON object")]
    FieldsNotObject,
    #[error("unsupported outbound MAVLink message {0}")]
    UnsupportedMessage(String),
    #[error("unsupported inbound MAVLink message ID {0}")]
    UnsupportedMessageId(u32),
    #[error("missing or invalid field {field} for {message}")]
    InvalidField {
        message: String,
        field: &'static str,
    },
    #[error("invalid enum value {value} for {field} in {message}")]
    InvalidEnum {
        message: String,
        field: &'static str,
        value: u64,
    },
    #[error("cannot project MAVLink message as JSON: {0}")]
    Json(#[from] serde_json::Error),
}

pub fn decode_raw_bytes(bytes: &[u8]) -> Result<DecodedMessage, CodecError> {
    let reader = StdIoReader::new(Cursor::new(bytes));
    let mut receiver = Receiver::versionless(reader);
    decode_raw(receiver.recv()?)
}

pub fn decode_raw(frame: Frame<Versionless>) -> Result<DecodedMessage, CodecError> {
    let message_id = frame.message_id();
    let (name, crc_extra) =
        message_info(message_id).ok_or(CodecError::UnsupportedMessageId(message_id))?;
    frame
        .validate_checksum_with_crc_extra(crc_extra)
        .map_err(|error| CodecError::Mavio(error.into()))?;
    let fields = if DialectMessage::message_info(message_id).is_ok() {
        let message = frame.decode::<DialectMessage>()?;
        let (decoded_name, fields) = project_fields(&message)?;
        debug_assert_eq!(decoded_name, name);
        fields
    } else {
        Map::new()
    };
    let mut bytes = vec![0; frame.size()];
    frame.serialize(&mut bytes)?;
    let mavlink_version = match frame.version() {
        MavLinkVersion::V1 => 1,
        MavLinkVersion::V2 => 2,
    };

    Ok(DecodedMessage {
        raw: bytes,
        mavlink_version,
        signed: frame.signature().is_some(),
        system_id: frame.system_id(),
        component_id: frame.component_id(),
        sequence: frame.sequence(),
        message_id,
        name: name.to_owned(),
        fields,
    })
}

pub fn encode_message(
    name: &str,
    fields: &Map<String, Value>,
) -> Result<DialectMessage, CodecError> {
    let message = match name {
        "COMMAND_LONG" => DialectMessage::CommandLong(messages::CommandLong {
            param1: f32_field(name, fields, "param1")?,
            param2: f32_field(name, fields, "param2")?,
            param3: f32_field(name, fields, "param3")?,
            param4: f32_field(name, fields, "param4")?,
            param5: f32_field(name, fields, "param5")?,
            param6: f32_field(name, fields, "param6")?,
            param7: f32_field(name, fields, "param7")?,
            command: enum_u16_field(name, fields, "command")?,
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
            confirmation: u8_field(name, fields, "confirmation")?,
        }),
        "NAMED_VALUE_FLOAT" => DialectMessage::NamedValueFloat(messages::NamedValueFloat {
            time_boot_ms: u32_field(name, fields, "time_boot_ms")?,
            value: f32_field(name, fields, "value")?,
            name: byte_string(name, fields, "name")?,
        }),
        "NAMED_VALUE_INT" => DialectMessage::NamedValueInt(messages::NamedValueInt {
            time_boot_ms: u32_field(name, fields, "time_boot_ms")?,
            value: i32_field(name, fields, "value")?,
            name: byte_string(name, fields, "name")?,
        }),
        "PARAM_SET" => DialectMessage::ParamSet(messages::ParamSet {
            param_value: f32_field(name, fields, "param_value")?,
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
            param_id: byte_string(name, fields, "param_id")?,
            param_type: enum_u8_field(name, fields, "param_type")?,
        }),
        "PARAM_REQUEST_READ" => DialectMessage::ParamRequestRead(messages::ParamRequestRead {
            param_index: i16_field(name, fields, "param_index")?,
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
            param_id: byte_string(name, fields, "param_id")?,
        }),
        "PARAM_REQUEST_LIST" => DialectMessage::ParamRequestList(messages::ParamRequestList {
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
        }),
        "REQUEST_DATA_STREAM" => DialectMessage::RequestDataStream(messages::RequestDataStream {
            req_message_rate: u16_field(name, fields, "req_message_rate")?,
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
            req_stream_id: u8_field(name, fields, "req_stream_id")?,
            start_stop: u8_field(name, fields, "start_stop")?,
        }),
        "SET_ATTITUDE_TARGET" => DialectMessage::SetAttitudeTarget(messages::SetAttitudeTarget {
            time_boot_ms: u32_field(name, fields, "time_boot_ms")?,
            q: f32_array_4(name, fields, "q")?,
            body_roll_rate: f32_field(name, fields, "body_roll_rate")?,
            body_pitch_rate: f32_field(name, fields, "body_pitch_rate")?,
            body_yaw_rate: f32_field(name, fields, "body_yaw_rate")?,
            thrust: f32_field(name, fields, "thrust")?,
            thrust_body: [0.0; 3],
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
            type_mask: enums::AttitudeTargetTypemask::from_bits_retain(u8_field(
                name,
                fields,
                "type_mask",
            )?),
        }),
        "FILE_TRANSFER_PROTOCOL" => {
            DialectMessage::FileTransferProtocol(messages::FileTransferProtocol {
                target_network: u8_field(name, fields, "target_network")?,
                target_system: u8_field(name, fields, "target_system")?,
                target_component: u8_field(name, fields, "target_component")?,
                payload: u8_array_251(name, fields, "payload")?,
            })
        }
        "LOG_REQUEST_LIST" => DialectMessage::LogRequestList(messages::LogRequestList {
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
            start: u16_field(name, fields, "start")?,
            end: u16_field(name, fields, "end")?,
        }),
        "LOG_REQUEST_DATA" => DialectMessage::LogRequestData(messages::LogRequestData {
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
            id: u16_field(name, fields, "id")?,
            ofs: u32_field(name, fields, "ofs")?,
            count: u32_field(name, fields, "count")?,
        }),
        "LOG_REQUEST_END" => DialectMessage::LogRequestEnd(messages::LogRequestEnd {
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
        }),
        _ => return Err(CodecError::UnsupportedMessage(name.to_owned())),
    };
    Ok(message)
}

#[must_use]
pub fn message_id_from_name(name: &str) -> Option<u32> {
    Some(match name {
        "ATTITUDE" => messages::Attitude::message_id(),
        "ATTITUDE_QUATERNION" => messages::AttitudeQuaternion::message_id(),
        "ATTITUDE_TARGET" => messages::AttitudeTarget::message_id(),
        "AUTOPILOT_VERSION" => messages::AutopilotVersion::message_id(),
        "BATTERY_STATUS" => messages::BatteryStatus::message_id(),
        "COMMAND_ACK" => messages::CommandAck::message_id(),
        "COMMAND_LONG" => messages::CommandLong::message_id(),
        "EKF_STATUS_REPORT" => messages::EkfStatusReport::message_id(),
        "EXTENDED_SYS_STATE" => messages::ExtendedSysState::message_id(),
        "FILE_TRANSFER_PROTOCOL" => messages::FileTransferProtocol::message_id(),
        "GLOBAL_POSITION_INT" => messages::GlobalPositionInt::message_id(),
        "HEARTBEAT" => messages::Heartbeat::message_id(),
        "LOCAL_POSITION_NED" => messages::LocalPositionNed::message_id(),
        "LOG_DATA" => messages::LogData::message_id(),
        "LOG_ENTRY" => messages::LogEntry::message_id(),
        "LOG_REQUEST_DATA" => messages::LogRequestData::message_id(),
        "LOG_REQUEST_END" => messages::LogRequestEnd::message_id(),
        "LOG_REQUEST_LIST" => messages::LogRequestList::message_id(),
        "MESSAGE_INTERVAL" => messages::MessageInterval::message_id(),
        "NAMED_VALUE_FLOAT" => messages::NamedValueFloat::message_id(),
        "NAMED_VALUE_INT" => messages::NamedValueInt::message_id(),
        "PARAM_REQUEST_LIST" => messages::ParamRequestList::message_id(),
        "PARAM_REQUEST_READ" => messages::ParamRequestRead::message_id(),
        "PARAM_SET" => messages::ParamSet::message_id(),
        "PARAM_VALUE" => messages::ParamValue::message_id(),
        "PID_TUNING" => messages::PidTuning::message_id(),
        "RC_CHANNELS" => messages::RcChannels::message_id(),
        "REQUEST_DATA_STREAM" => messages::RequestDataStream::message_id(),
        "SERVO_OUTPUT_RAW" => messages::ServoOutputRaw::message_id(),
        "SET_ATTITUDE_TARGET" => messages::SetAttitudeTarget::message_id(),
        "STATUSTEXT" => messages::Statustext::message_id(),
        "SYS_STATUS" => messages::SysStatus::message_id(),
        _ => return None,
    })
}

pub fn serialize_message(
    message: &DialectMessage,
    header: MavHeader,
) -> Result<Vec<u8>, CodecError> {
    let payload = message
        .encode(MavLinkVersion::V2)
        .map_err(|error| CodecError::Spec(format!("{error:?}")))?;
    let frame = Frame::builder()
        .sequence(header.sequence)
        .system_id(header.system_id)
        .component_id(header.component_id)
        .version(V2)
        .message_id(message.id())
        .payload(payload.bytes())
        .crc_extra(message.crc_extra())
        .build();
    let mut bytes = vec![0; frame.size()];
    frame.serialize(&mut bytes)?;
    Ok(bytes)
}

pub fn project_fields(
    message: &DialectMessage,
) -> Result<(&'static str, Map<String, Value>), CodecError> {
    let (name, value) = match message {
        DialectMessage::Heartbeat(data) => ("HEARTBEAT", serde_json::to_value(data)?),
        DialectMessage::ParamSet(data) => ("PARAM_SET", serde_json::to_value(data)?),
        DialectMessage::ExtendedSysState(data) => {
            ("EXTENDED_SYS_STATE", serde_json::to_value(data)?)
        }
        DialectMessage::AttitudeQuaternion(data) => {
            ("ATTITUDE_QUATERNION", serde_json::to_value(data)?)
        }
        DialectMessage::ParamRequestList(data) => {
            ("PARAM_REQUEST_LIST", serde_json::to_value(data)?)
        }
        DialectMessage::CommandLong(data) => ("COMMAND_LONG", serde_json::to_value(data)?),
        DialectMessage::AutopilotVersion(data) => {
            ("AUTOPILOT_VERSION", serde_json::to_value(data)?)
        }
        DialectMessage::Attitude(data) => ("ATTITUDE", serde_json::to_value(data)?),
        DialectMessage::LocalPositionNed(data) => {
            ("LOCAL_POSITION_NED", serde_json::to_value(data)?)
        }
        DialectMessage::SetAttitudeTarget(data) => {
            ("SET_ATTITUDE_TARGET", serde_json::to_value(data)?)
        }
        DialectMessage::LogRequestData(data) => ("LOG_REQUEST_DATA", serde_json::to_value(data)?),
        DialectMessage::ServoOutputRaw(data) => ("SERVO_OUTPUT_RAW", serde_json::to_value(data)?),
        DialectMessage::ParamRequestRead(data) => {
            ("PARAM_REQUEST_READ", serde_json::to_value(data)?)
        }
        DialectMessage::SysStatus(data) => ("SYS_STATUS", serde_json::to_value(data)?),
        DialectMessage::NamedValueFloat(data) => ("NAMED_VALUE_FLOAT", serde_json::to_value(data)?),
        DialectMessage::RcChannels(data) => ("RC_CHANNELS", serde_json::to_value(data)?),
        DialectMessage::Statustext(data) => ("STATUSTEXT", serde_json::to_value(data)?),
        DialectMessage::BatteryStatus(data) => ("BATTERY_STATUS", serde_json::to_value(data)?),
        DialectMessage::NamedValueInt(data) => ("NAMED_VALUE_INT", serde_json::to_value(data)?),
        DialectMessage::LogRequestList(data) => ("LOG_REQUEST_LIST", serde_json::to_value(data)?),
        DialectMessage::MessageInterval(data) => ("MESSAGE_INTERVAL", serde_json::to_value(data)?),
        DialectMessage::AttitudeTarget(data) => ("ATTITUDE_TARGET", serde_json::to_value(data)?),
        DialectMessage::ParamValue(data) => ("PARAM_VALUE", serde_json::to_value(data)?),
        DialectMessage::PidTuning(data) => ("PID_TUNING", serde_json::to_value(data)?),
        DialectMessage::LogEntry(data) => ("LOG_ENTRY", serde_json::to_value(data)?),
        DialectMessage::GlobalPositionInt(data) => {
            ("GLOBAL_POSITION_INT", serde_json::to_value(data)?)
        }
        DialectMessage::CommandAck(data) => ("COMMAND_ACK", serde_json::to_value(data)?),
        DialectMessage::LogData(data) => ("LOG_DATA", serde_json::to_value(data)?),
        DialectMessage::LogRequestEnd(data) => ("LOG_REQUEST_END", serde_json::to_value(data)?),
        DialectMessage::EkfStatusReport(data) => ("EKF_STATUS_REPORT", serde_json::to_value(data)?),
        DialectMessage::RequestDataStream(data) => {
            ("REQUEST_DATA_STREAM", serde_json::to_value(data)?)
        }
        DialectMessage::FileTransferProtocol(data) => {
            ("FILE_TRANSFER_PROTOCOL", serde_json::to_value(data)?)
        }
    };
    let mut object = value
        .as_object()
        .cloned()
        .ok_or(CodecError::FieldsNotObject)?;

    match message {
        DialectMessage::Heartbeat(data) => {
            put_number(&mut object, "type", data.type_ as u64);
            put_number(&mut object, "autopilot", data.autopilot as u64);
            put_number(&mut object, "base_mode", u64::from(data.base_mode.bits()));
            put_number(&mut object, "system_status", data.system_status as u64);
        }
        DialectMessage::CommandAck(data) => {
            put_number(&mut object, "command", data.command as u64);
            put_number(&mut object, "result", data.result as u64);
        }
        DialectMessage::ParamValue(data) => {
            put_number(&mut object, "param_type", data.param_type as u64);
            put_text(&mut object, "param_id", &data.param_id);
        }
        DialectMessage::AutopilotVersion(data) => {
            put_number(
                &mut object,
                "capabilities",
                u64::from(data.capabilities.bits()),
            );
        }
        DialectMessage::Statustext(data) => {
            put_number(&mut object, "severity", data.severity as u64);
            put_text(&mut object, "text", &data.text);
        }
        DialectMessage::PidTuning(data) => {
            put_number(&mut object, "axis", data.axis as u64);
        }
        DialectMessage::EkfStatusReport(data) => {
            put_number(&mut object, "flags", u64::from(data.flags.bits()));
        }
        DialectMessage::SysStatus(data) => {
            put_number(
                &mut object,
                "onboard_control_sensors_present",
                u64::from(data.onboard_control_sensors_present.bits()),
            );
            put_number(
                &mut object,
                "onboard_control_sensors_enabled",
                u64::from(data.onboard_control_sensors_enabled.bits()),
            );
            put_number(
                &mut object,
                "onboard_control_sensors_health",
                u64::from(data.onboard_control_sensors_health.bits()),
            );
        }
        DialectMessage::AttitudeTarget(data) => {
            put_number(&mut object, "type_mask", u64::from(data.type_mask.bits()));
        }
        DialectMessage::ExtendedSysState(data) => {
            put_number(&mut object, "vtol_state", data.vtol_state as u64);
            put_number(&mut object, "landed_state", data.landed_state as u64);
        }
        DialectMessage::NamedValueFloat(data) => put_text(&mut object, "name", &data.name),
        DialectMessage::NamedValueInt(data) => put_text(&mut object, "name", &data.name),
        DialectMessage::ParamSet(data) => {
            put_number(&mut object, "param_type", data.param_type as u64);
            put_text(&mut object, "param_id", &data.param_id);
        }
        DialectMessage::ParamRequestRead(data) => {
            put_text(&mut object, "param_id", &data.param_id);
        }
        _ => {}
    }

    Ok((name, object))
}

fn put_number(object: &mut Map<String, Value>, name: &str, value: u64) {
    object.insert(name.to_owned(), Value::from(value));
}

fn put_text(object: &mut Map<String, Value>, name: &str, bytes: &[u8]) {
    let end = bytes
        .iter()
        .position(|byte| *byte == 0)
        .unwrap_or(bytes.len());
    object.insert(
        name.to_owned(),
        Value::from(String::from_utf8_lossy(&bytes[..end]).into_owned()),
    );
}

fn value<'a>(
    message: &str,
    fields: &'a Map<String, Value>,
    field: &'static str,
) -> Result<&'a Value, CodecError> {
    fields.get(field).ok_or_else(|| CodecError::InvalidField {
        message: message.to_owned(),
        field,
    })
}

fn u64_field(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<u64, CodecError> {
    value(message, fields, field)?
        .as_u64()
        .ok_or_else(|| CodecError::InvalidField {
            message: message.to_owned(),
            field,
        })
}

fn u8_field(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<u8, CodecError> {
    u64_field(message, fields, field)?
        .try_into()
        .map_err(|_| CodecError::InvalidField {
            message: message.to_owned(),
            field,
        })
}

fn u16_field(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<u16, CodecError> {
    u64_field(message, fields, field)?
        .try_into()
        .map_err(|_| CodecError::InvalidField {
            message: message.to_owned(),
            field,
        })
}

fn u32_field(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<u32, CodecError> {
    u64_field(message, fields, field)?
        .try_into()
        .map_err(|_| CodecError::InvalidField {
            message: message.to_owned(),
            field,
        })
}

fn i64_field(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<i64, CodecError> {
    value(message, fields, field)?
        .as_i64()
        .ok_or_else(|| CodecError::InvalidField {
            message: message.to_owned(),
            field,
        })
}

fn i16_field(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<i16, CodecError> {
    i64_field(message, fields, field)?
        .try_into()
        .map_err(|_| CodecError::InvalidField {
            message: message.to_owned(),
            field,
        })
}

fn i32_field(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<i32, CodecError> {
    i64_field(message, fields, field)?
        .try_into()
        .map_err(|_| CodecError::InvalidField {
            message: message.to_owned(),
            field,
        })
}

fn f32_field(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<f32, CodecError> {
    value(message, fields, field)?
        .as_f64()
        .map(|value| value as f32)
        .ok_or_else(|| CodecError::InvalidField {
            message: message.to_owned(),
            field,
        })
}

fn str_field<'a>(
    message: &str,
    fields: &'a Map<String, Value>,
    field: &'static str,
) -> Result<&'a str, CodecError> {
    value(message, fields, field)?
        .as_str()
        .ok_or_else(|| CodecError::InvalidField {
            message: message.to_owned(),
            field,
        })
}

fn byte_string<const N: usize>(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<[u8; N], CodecError> {
    let source = str_field(message, fields, field)?.as_bytes();
    if source.len() > N {
        return Err(CodecError::InvalidField {
            message: message.to_owned(),
            field,
        });
    }
    let mut result = [0; N];
    result[..source.len()].copy_from_slice(source);
    Ok(result)
}

fn enum_u8_field<T>(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<T, CodecError>
where
    T: TryFrom<u8>,
{
    let number = u64_field(message, fields, field)?;
    u8::try_from(number)
        .ok()
        .and_then(|number| T::try_from(number).ok())
        .ok_or(CodecError::InvalidEnum {
            message: message.to_owned(),
            field,
            value: number,
        })
}

fn enum_u16_field<T>(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<T, CodecError>
where
    T: TryFrom<u16>,
{
    let number = u64_field(message, fields, field)?;
    u16::try_from(number)
        .ok()
        .and_then(|number| T::try_from(number).ok())
        .ok_or(CodecError::InvalidEnum {
            message: message.to_owned(),
            field,
            value: number,
        })
}

fn f32_array_4(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<[f32; 4], CodecError> {
    let array =
        value(message, fields, field)?
            .as_array()
            .ok_or_else(|| CodecError::InvalidField {
                message: message.to_owned(),
                field,
            })?;
    if array.len() != 4 {
        return Err(CodecError::InvalidField {
            message: message.to_owned(),
            field,
        });
    }

    let mut result = [0.0; 4];
    for (target, source) in result.iter_mut().zip(array) {
        *target =
            source
                .as_f64()
                .map(|value| value as f32)
                .ok_or_else(|| CodecError::InvalidField {
                    message: message.to_owned(),
                    field,
                })?;
    }
    Ok(result)
}

fn u8_array_251(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<[u8; 251], CodecError> {
    let array =
        value(message, fields, field)?
            .as_array()
            .ok_or_else(|| CodecError::InvalidField {
                message: message.to_owned(),
                field,
            })?;
    if array.len() != 251 {
        return Err(CodecError::InvalidField {
            message: message.to_owned(),
            field,
        });
    }
    let mut result = [0; 251];
    for (target, source) in result.iter_mut().zip(array) {
        *target = source
            .as_u64()
            .and_then(|number| number.try_into().ok())
            .ok_or_else(|| CodecError::InvalidField {
                message: message.to_owned(),
                field,
            })?;
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavio::protocol::V1;

    #[test]
    fn heartbeat_matches_pymavlink_and_projects_numeric_enums() {
        let message = DialectMessage::Heartbeat(messages::Heartbeat {
            custom_mode: 0,
            type_: enums::MavType::Gcs,
            autopilot: enums::MavAutopilot::Invalid,
            base_mode: enums::MavModeFlag::empty(),
            system_status: enums::MavState::Active,
            mavlink_version: 3,
        });
        let bytes = serialize_message(
            &message,
            MavHeader {
                system_id: 255,
                component_id: 0,
                sequence: 9,
            },
        )
        .unwrap();

        assert_eq!(
            hex::encode(&bytes),
            "fd09000009ff000000000000000006080004035182"
        );
        let decoded = decode_raw_bytes(&bytes).unwrap();
        assert_eq!(decoded.raw, bytes);
        assert_eq!(decoded.name, "HEARTBEAT");
        assert_eq!(decoded.fields["type"], 6);
        assert_eq!(decoded.fields["autopilot"], 8);
        assert_eq!(decoded.fields["base_mode"], 0);
        assert_eq!(decoded.fields["system_status"], 4);
    }

    #[test]
    fn corrupt_crc_is_rejected() {
        let mut bytes = hex::decode("fd09000009ff000000000000000006080004035182").unwrap();
        let last = bytes.len() - 1;
        bytes[last] ^= 0xff;

        assert!(decode_raw_bytes(&bytes).is_err());
    }

    #[test]
    fn validates_and_preserves_untyped_ardupilotmega_frames() {
        let frame = Frame::builder()
            .sequence(7)
            .system_id(1)
            .component_id(1)
            .version(V2)
            .message_id(242)
            .payload(&[1, 2, 3])
            .crc_extra(104)
            .build();
        let mut bytes = vec![0; frame.size()];
        frame.serialize(&mut bytes).unwrap();

        let decoded = decode_raw_bytes(&bytes).unwrap();

        assert_eq!(decoded.name, "HOME_POSITION");
        assert_eq!(decoded.raw, bytes);
        assert!(decoded.fields.is_empty());
    }

    #[test]
    fn autopilot_capabilities_project_as_numeric_bits() {
        let message = DialectMessage::AutopilotVersion(messages::AutopilotVersion {
            capabilities: enums::MavProtocolCapability::FTP
                | enums::MavProtocolCapability::COMMAND_INT,
            flight_sw_version: 1,
            middleware_sw_version: 2,
            os_sw_version: 3,
            board_version: 4,
            flight_custom_version: [0; 8],
            middleware_custom_version: [0; 8],
            os_custom_version: [0; 8],
            vendor_id: 5,
            product_id: 6,
            uid: 7,
            uid2: [0; 18],
        });

        let (_, fields) = project_fields(&message).expect("project version");

        assert_eq!(fields["capabilities"], 40);
    }

    #[test]
    fn extended_system_state_projects_numeric_enums() {
        let message = DialectMessage::ExtendedSysState(messages::ExtendedSysState {
            vtol_state: enums::MavVtolState::Mc,
            landed_state: enums::MavLandedState::InAir,
        });

        let (_, fields) = project_fields(&message).expect("project extended state");

        assert_eq!(fields["vtol_state"], 3);
        assert_eq!(fields["landed_state"], 2);
    }

    #[test]
    fn decodes_and_preserves_v1_frames() {
        let message = DialectMessage::Heartbeat(messages::Heartbeat {
            custom_mode: 0,
            type_: enums::MavType::Gcs,
            autopilot: enums::MavAutopilot::Invalid,
            base_mode: enums::MavModeFlag::empty(),
            system_status: enums::MavState::Active,
            mavlink_version: 3,
        });
        let payload = message.encode(MavLinkVersion::V1).unwrap();
        let frame = Frame::builder()
            .sequence(4)
            .system_id(255)
            .component_id(0)
            .version(V1)
            .message_id(message.id())
            .payload(payload.bytes())
            .crc_extra(message.crc_extra())
            .build();
        let mut bytes = vec![0; frame.size()];
        frame.serialize(&mut bytes).unwrap();

        let decoded = decode_raw_bytes(&bytes).unwrap();
        assert_eq!(decoded.mavlink_version, 1);
        assert_eq!(decoded.sequence, 4);
        assert_eq!(decoded.raw, bytes);
    }

    #[test]
    fn encodes_calibration_command_long() {
        let fields = serde_json::json!({
            "target_system": 1,
            "target_component": 1,
            "command": 400,
            "confirmation": 0,
            "param1": 1.0,
            "param2": 0.0,
            "param3": 0.0,
            "param4": 0.0,
            "param5": 0.0,
            "param6": 0.0,
            "param7": 0.0
        });
        let message = encode_message("COMMAND_LONG", fields.as_object().unwrap()).unwrap();

        assert!(matches!(message, DialectMessage::CommandLong(_)));
    }
}
