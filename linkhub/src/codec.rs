use std::io::Cursor;

use mavlink::{
    MAVLinkMessageRaw, MavHeader, MavlinkVersion, Message, ReadVersion,
    dialects::ardupilotmega::{self, AttitudeTargetTypemask},
    peek_reader::PeekReader,
    read_versioned_raw_message, write_versioned_msg,
};
use num_traits::FromPrimitive;
use serde_json::{Map, Value};
use thiserror::Error;

pub type DialectMessage = ardupilotmega::MavMessage;

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
    pub message: DialectMessage,
}

#[derive(Debug, Error)]
pub enum CodecError {
    #[error("invalid MAVLink frame: {0}")]
    Read(#[from] mavlink::error::MessageReadError),
    #[error("cannot serialize MAVLink message: {0}")]
    Write(#[from] mavlink::error::MessageWriteError),
    #[error("cannot decode MAVLink message {message_id}: {source}")]
    Parse {
        message_id: u32,
        source: mavlink::error::ParserError,
    },
    #[error("message fields must be a JSON object")]
    FieldsNotObject,
    #[error("unsupported outbound MAVLink message {0}")]
    UnsupportedMessage(String),
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
    let mut reader = PeekReader::new(Cursor::new(bytes));
    let raw = read_versioned_raw_message::<DialectMessage, _>(&mut reader, ReadVersion::Any)?;
    decode_raw(raw)
}

pub fn decode_raw(raw: MAVLinkMessageRaw) -> Result<DecodedMessage, CodecError> {
    let message_id = raw.message_id();
    let message = DialectMessage::parse(raw.version(), message_id, raw.payload())
        .map_err(|source| CodecError::Parse { message_id, source })?;
    let fields = project_fields(&message)?;
    let (bytes, mavlink_version, signed) = match &raw {
        MAVLinkMessageRaw::V1(frame) => (frame.raw_bytes().to_vec(), 1, false),
        MAVLinkMessageRaw::V2(frame) => (
            frame.raw_bytes().to_vec(),
            2,
            frame.incompatibility_flags() & 0x01 != 0,
        ),
    };

    Ok(DecodedMessage {
        raw: bytes,
        mavlink_version,
        signed,
        system_id: raw.system_id(),
        component_id: raw.component_id(),
        sequence: raw.sequence(),
        message_id,
        name: message.message_name().to_owned(),
        fields,
        message,
    })
}

#[allow(deprecated)]
pub fn encode_message(
    name: &str,
    fields: &Map<String, Value>,
) -> Result<DialectMessage, CodecError> {
    use ardupilotmega::MavMessage;

    let message = match name {
        "COMMAND_LONG" => MavMessage::COMMAND_LONG(ardupilotmega::COMMAND_LONG_DATA {
            param1: f32_field(name, fields, "param1")?,
            param2: f32_field(name, fields, "param2")?,
            param3: f32_field(name, fields, "param3")?,
            param4: f32_field(name, fields, "param4")?,
            param5: f32_field(name, fields, "param5")?,
            param6: f32_field(name, fields, "param6")?,
            param7: f32_field(name, fields, "param7")?,
            command: enum_field(name, fields, "command")?,
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
            confirmation: u8_field(name, fields, "confirmation")?,
        }),
        "NAMED_VALUE_FLOAT" => {
            MavMessage::NAMED_VALUE_FLOAT(ardupilotmega::NAMED_VALUE_FLOAT_DATA {
                time_boot_ms: u32_field(name, fields, "time_boot_ms")?,
                value: f32_field(name, fields, "value")?,
                name: str_field(name, fields, "name")?.into(),
            })
        }
        "NAMED_VALUE_INT" => MavMessage::NAMED_VALUE_INT(ardupilotmega::NAMED_VALUE_INT_DATA {
            time_boot_ms: u32_field(name, fields, "time_boot_ms")?,
            value: i32_field(name, fields, "value")?,
            name: str_field(name, fields, "name")?.into(),
        }),
        "PARAM_SET" => MavMessage::PARAM_SET(ardupilotmega::PARAM_SET_DATA {
            param_value: f32_field(name, fields, "param_value")?,
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
            param_id: str_field(name, fields, "param_id")?.into(),
            param_type: enum_field(name, fields, "param_type")?,
        }),
        "PARAM_REQUEST_READ" => {
            MavMessage::PARAM_REQUEST_READ(ardupilotmega::PARAM_REQUEST_READ_DATA {
                param_index: i16_field(name, fields, "param_index")?,
                target_system: u8_field(name, fields, "target_system")?,
                target_component: u8_field(name, fields, "target_component")?,
                param_id: str_field(name, fields, "param_id")?.into(),
            })
        }
        "PARAM_REQUEST_LIST" => {
            MavMessage::PARAM_REQUEST_LIST(ardupilotmega::PARAM_REQUEST_LIST_DATA {
                target_system: u8_field(name, fields, "target_system")?,
                target_component: u8_field(name, fields, "target_component")?,
            })
        }
        "REQUEST_DATA_STREAM" => {
            MavMessage::REQUEST_DATA_STREAM(ardupilotmega::REQUEST_DATA_STREAM_DATA {
                req_message_rate: u16_field(name, fields, "req_message_rate")?,
                target_system: u8_field(name, fields, "target_system")?,
                target_component: u8_field(name, fields, "target_component")?,
                req_stream_id: u8_field(name, fields, "req_stream_id")?,
                start_stop: u8_field(name, fields, "start_stop")?,
            })
        }
        "SET_ATTITUDE_TARGET" => {
            MavMessage::SET_ATTITUDE_TARGET(ardupilotmega::SET_ATTITUDE_TARGET_DATA {
                time_boot_ms: u32_field(name, fields, "time_boot_ms")?,
                q: f32_array_4(name, fields, "q")?,
                body_roll_rate: f32_field(name, fields, "body_roll_rate")?,
                body_pitch_rate: f32_field(name, fields, "body_pitch_rate")?,
                body_yaw_rate: f32_field(name, fields, "body_yaw_rate")?,
                thrust: f32_field(name, fields, "thrust")?,
                thrust_body: [0.0; 3],
                target_system: u8_field(name, fields, "target_system")?,
                target_component: u8_field(name, fields, "target_component")?,
                type_mask: AttitudeTargetTypemask::from_bits_retain(u8_field(
                    name,
                    fields,
                    "type_mask",
                )?),
            })
        }
        "FILE_TRANSFER_PROTOCOL" => {
            MavMessage::FILE_TRANSFER_PROTOCOL(ardupilotmega::FILE_TRANSFER_PROTOCOL_DATA {
                target_network: u8_field(name, fields, "target_network")?,
                target_system: u8_field(name, fields, "target_system")?,
                target_component: u8_field(name, fields, "target_component")?,
                payload: u8_array_251(name, fields, "payload")?,
            })
        }
        "LOG_REQUEST_LIST" => MavMessage::LOG_REQUEST_LIST(ardupilotmega::LOG_REQUEST_LIST_DATA {
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
            start: u16_field(name, fields, "start")?,
            end: u16_field(name, fields, "end")?,
        }),
        "LOG_REQUEST_DATA" => MavMessage::LOG_REQUEST_DATA(ardupilotmega::LOG_REQUEST_DATA_DATA {
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
            id: u16_field(name, fields, "id")?,
            ofs: u32_field(name, fields, "ofs")?,
            count: u32_field(name, fields, "count")?,
        }),
        "LOG_REQUEST_END" => MavMessage::LOG_REQUEST_END(ardupilotmega::LOG_REQUEST_END_DATA {
            target_system: u8_field(name, fields, "target_system")?,
            target_component: u8_field(name, fields, "target_component")?,
        }),
        _ => return Err(CodecError::UnsupportedMessage(name.to_owned())),
    };
    Ok(message)
}

pub fn serialize_message(
    message: &DialectMessage,
    header: MavHeader,
) -> Result<Vec<u8>, CodecError> {
    let mut bytes = Vec::with_capacity(280);
    write_versioned_msg(&mut bytes, MavlinkVersion::V2, header, message)?;
    Ok(bytes)
}

pub fn project_fields(message: &DialectMessage) -> Result<Map<String, Value>, CodecError> {
    use ardupilotmega::MavMessage;

    let mut object = serde_json::to_value(message)?
        .as_object()
        .cloned()
        .ok_or(CodecError::FieldsNotObject)?;
    object.remove("type");

    match message {
        MavMessage::HEARTBEAT(data) => {
            put_number(&mut object, "type", data.mavtype as u64);
            put_number(&mut object, "autopilot", data.autopilot as u64);
            put_number(&mut object, "base_mode", u64::from(data.base_mode.bits()));
            put_number(&mut object, "system_status", data.system_status as u64);
        }
        MavMessage::COMMAND_ACK(data) => {
            put_number(&mut object, "command", data.command as u64);
            put_number(&mut object, "result", data.result as u64);
        }
        MavMessage::PARAM_VALUE(data) => {
            put_number(&mut object, "param_type", data.param_type as u64);
        }
        MavMessage::AUTOPILOT_VERSION(data) => {
            put_number(&mut object, "capabilities", data.capabilities.bits());
        }
        MavMessage::STATUSTEXT(data) => {
            put_number(&mut object, "severity", data.severity as u64);
        }
        MavMessage::PID_TUNING(data) => {
            put_number(&mut object, "axis", data.axis as u64);
        }
        MavMessage::EKF_STATUS_REPORT(data) => {
            put_number(&mut object, "flags", u64::from(data.flags.bits()));
        }
        MavMessage::SYS_STATUS(data) => {
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
        MavMessage::ATTITUDE_TARGET(data) => {
            put_number(&mut object, "type_mask", u64::from(data.type_mask.bits()));
        }
        _ => {}
    }

    Ok(object)
}

fn put_number(object: &mut Map<String, Value>, name: &str, value: u64) {
    object.insert(name.to_owned(), Value::from(value));
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

fn enum_field<T>(
    message: &str,
    fields: &Map<String, Value>,
    field: &'static str,
) -> Result<T, CodecError>
where
    T: FromPrimitive,
{
    let number = u64_field(message, fields, field)?;
    T::from_u64(number).ok_or(CodecError::InvalidEnum {
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
    use mavlink::dialects::ardupilotmega::{
        AUTOPILOT_VERSION_DATA, HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavProtocolCapability,
        MavState, MavType,
    };

    #[test]
    fn heartbeat_matches_pymavlink_and_projects_numeric_enums() {
        let message = DialectMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GCS,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
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
    fn autopilot_capabilities_project_as_numeric_bits() {
        let message = DialectMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
            capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_FTP
                | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT,
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

        let fields = project_fields(&message).expect("project version");

        assert_eq!(fields["capabilities"], 40);
    }

    #[test]
    fn decodes_and_preserves_v1_frames() {
        let message = DialectMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GCS,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        });
        let mut bytes = Vec::new();
        write_versioned_msg(
            &mut bytes,
            MavlinkVersion::V1,
            MavHeader {
                system_id: 255,
                component_id: 0,
                sequence: 4,
            },
            &message,
        )
        .unwrap();

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

        assert!(matches!(message, DialectMessage::COMMAND_LONG(_)));
    }
}
