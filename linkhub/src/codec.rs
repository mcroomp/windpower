use std::{collections::HashSet, io::Cursor};

pub use dialect::MavMessage;
use linkhub_dialect::{
    MAVLinkMessageRaw, MAVLinkV2MessageRaw, Message,
    async_peek_reader::AsyncPeekReader,
    error::{MessageReadError, ParserError},
    peek_reader::PeekReader,
    read_any_raw_message, read_any_raw_message_async,
};
pub use linkhub_dialect::{MavHeader, dialects::ardupilotmega as dialect};
use serde_json::{Map, Value};
use thiserror::Error;
use tokio::io::AsyncRead;

/// JSON key that the dialect's serde representation uses to tag the message.
/// A dialect field literally named `type` is exposed as `mavtype`.
const MESSAGE_TAG: &str = "type";

// MAVLink v2 incompatibility flag marking a trailing signature block.
const IFLAG_SIGNED: u8 = 0x01;

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
    pub message: MavMessage,
}

#[derive(Debug, Error)]
pub enum CodecError {
    #[error("cannot read MAVLink frame: {0}")]
    Read(#[from] MessageReadError),
    #[error("cannot parse MAVLink message: {0}")]
    Parse(#[from] ParserError),
    #[error("cannot serialize MAVLink message: {0}")]
    Json(#[from] serde_json::Error),
    #[error("unknown MAVLink message {0:?}")]
    UnknownMessage(String),
    #[error("invalid fields for {message}: {reason}")]
    InvalidFields { message: String, reason: String },
}

pub fn decode_raw_bytes(bytes: &[u8]) -> Result<DecodedMessage, CodecError> {
    let mut reader = PeekReader::new(Cursor::new(bytes));
    decode_raw(&read_any_raw_message::<MavMessage, _>(&mut reader)?)
}

/// Reads MAVLink v1/v2 frames from an async byte stream.
///
/// Corrupt frames are skipped by `mavlink` while resynchronizing. Frames that
/// pass the CRC but do not decode as a typed message, such as an enum value
/// outside the dialect, are dropped here without ending the stream.
pub struct FrameReader<R> {
    reader: AsyncPeekReader<R>,
    dropped_frames: u64,
    reported_ids: HashSet<u32>,
}

impl<R: AsyncRead + Unpin> FrameReader<R> {
    pub fn new(reader: R) -> Self {
        Self {
            reader: AsyncPeekReader::new(reader),
            dropped_frames: 0,
            reported_ids: HashSet::new(),
        }
    }

    /// Frames dropped since the previous call, which resets the count.
    pub fn take_dropped_frames(&mut self) -> u64 {
        std::mem::take(&mut self.dropped_frames)
    }

    pub async fn recv(&mut self) -> Result<DecodedMessage, CodecError> {
        loop {
            let frame = read_any_raw_message_async::<MavMessage, _>(&mut self.reader).await?;
            match decode_raw(&frame) {
                Ok(decoded) => return Ok(decoded),
                Err(error) => {
                    self.dropped_frames += 1;
                    if self.reported_ids.insert(frame.message_id()) {
                        tracing::warn!(
                            message_id = frame.message_id(),
                            %error,
                            "dropping undecodable MAVLink frame"
                        );
                    }
                }
            }
        }
    }
}

fn decode_raw(frame: &MAVLinkMessageRaw) -> Result<DecodedMessage, CodecError> {
    let message_id = frame.message_id();
    let message = MavMessage::parse(frame.version(), message_id, frame.payload())?;
    let (mavlink_version, raw, signed) = match frame {
        MAVLinkMessageRaw::V1(frame) => (1, frame.raw_bytes(), false),
        MAVLinkMessageRaw::V2(frame) => (
            2,
            frame.raw_bytes(),
            frame.incompatibility_flags() & IFLAG_SIGNED != 0,
        ),
    };

    Ok(DecodedMessage {
        raw: raw.to_vec(),
        mavlink_version,
        signed,
        system_id: frame.system_id(),
        component_id: frame.component_id(),
        sequence: frame.sequence(),
        message_id,
        name: message.message_name().to_owned(),
        fields: message_fields(&message)?,
        message,
    })
}

/// The message's field object: `mavlink`'s serde JSON without the type tag.
///
/// Enumerations are `{"type": "NAME"}` objects, bitmasks are `"A | B"` strings
/// (empty when no flag is set), and a dialect field named `type` is `mavtype`.
pub fn message_fields(message: &MavMessage) -> Result<Map<String, Value>, CodecError> {
    let Value::Object(mut fields) = serde_json::to_value(message)? else {
        unreachable!("MavMessage serializes as a tagged object");
    };
    fields.remove(MESSAGE_TAG);
    Ok(fields)
}

/// Builds a typed message from its dialect name and a field object in the
/// representation produced by [`message_fields`].
pub fn message_from_fields(
    name: &str,
    fields: &Map<String, Value>,
) -> Result<MavMessage, CodecError> {
    if MavMessage::message_id_from_name(name).is_none() {
        return Err(CodecError::UnknownMessage(name.to_owned()));
    }
    if fields.contains_key(MESSAGE_TAG) {
        return Err(CodecError::InvalidFields {
            message: name.to_owned(),
            reason: "`type` is reserved; a dialect field named `type` is called `mavtype`"
                .to_owned(),
        });
    }
    let mut object = fields.clone();
    object.insert(MESSAGE_TAG.to_owned(), Value::String(name.to_owned()));
    serde_json::from_value(Value::Object(object)).map_err(|error| CodecError::InvalidFields {
        message: name.to_owned(),
        reason: error.to_string(),
    })
}

#[must_use]
pub fn message_id_from_name(name: &str) -> Option<u32> {
    MavMessage::message_id_from_name(name)
}

#[must_use]
pub fn serialize_message(message: &MavMessage, header: MavHeader) -> Vec<u8> {
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message(header, message);
    frame.raw_bytes().to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;
    use linkhub_dialect::{
        MAVLinkV1MessageRaw, calculate_crc,
        dialects::ardupilotmega::{
            AUTOPILOT_VERSION_DATA, COMMAND_LONG_DATA, EXTENDED_SYS_STATE_DATA, HEARTBEAT_DATA,
            MavAutopilot, MavCmd, MavLandedState, MavModeFlag, MavProtocolCapability, MavState,
            MavType, MavVtolState,
        },
    };
    use serde_json::json;

    fn object(value: Value) -> Map<String, Value> {
        value.as_object().expect("JSON object").clone()
    }

    fn header(sequence: u8) -> MavHeader {
        MavHeader {
            system_id: 255,
            component_id: 0,
            sequence,
        }
    }

    fn heartbeat() -> MavMessage {
        MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GCS,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        })
    }

    #[test]
    fn heartbeat_matches_pymavlink_and_projects_typed_enums() {
        let bytes = serialize_message(&heartbeat(), header(9));

        assert_eq!(
            hex::encode(&bytes),
            "fd09000009ff000000000000000006080004035182"
        );
        let decoded = decode_raw_bytes(&bytes).unwrap();
        assert_eq!(decoded.raw, bytes);
        assert_eq!(decoded.name, "HEARTBEAT");
        assert_eq!(decoded.fields["mavtype"], json!({"type": "MAV_TYPE_GCS"}));
        assert_eq!(
            decoded.fields["autopilot"],
            json!({"type": "MAV_AUTOPILOT_INVALID"})
        );
        assert_eq!(decoded.fields["base_mode"], "");
        assert_eq!(
            decoded.fields["system_status"],
            json!({"type": "MAV_STATE_ACTIVE"})
        );
        assert!(!decoded.fields.contains_key("type"));
    }

    #[test]
    fn corrupt_crc_is_rejected() {
        let mut bytes = hex::decode("fd09000009ff000000000000000006080004035182").unwrap();
        let last = bytes.len() - 1;
        bytes[last] ^= 0xff;

        assert!(decode_raw_bytes(&bytes).is_err());
    }

    #[test]
    fn every_dialect_message_round_trips_through_json() {
        let mut checked = 0;
        for id in 0..u32::from(u16::MAX) {
            let Some(message) = MavMessage::default_message_from_id(id) else {
                continue;
            };
            let fields = message_fields(&message).unwrap();
            let rebuilt = message_from_fields(message.message_name(), &fields).unwrap();
            assert_eq!(rebuilt.message_id(), id, "{}", message.message_name());
            assert_eq!(message_fields(&rebuilt).unwrap(), fields);
            checked += 1;
        }
        assert!(checked > 250, "only {checked} messages checked");
    }

    #[test]
    fn bitmask_capabilities_project_as_flag_names() {
        let message = MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
            capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT
                | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_FTP,
            ..AUTOPILOT_VERSION_DATA::default()
        });
        let bytes = serialize_message(&message, header(0));

        assert_eq!(
            decode_raw_bytes(&bytes).unwrap().fields["capabilities"],
            "MAV_PROTOCOL_CAPABILITY_COMMAND_INT | MAV_PROTOCOL_CAPABILITY_FTP"
        );
    }

    #[test]
    fn extended_system_state_projects_typed_enums() {
        let message = MavMessage::EXTENDED_SYS_STATE(EXTENDED_SYS_STATE_DATA {
            vtol_state: MavVtolState::MAV_VTOL_STATE_MC,
            landed_state: MavLandedState::MAV_LANDED_STATE_IN_AIR,
        });
        let fields = decode_raw_bytes(&serialize_message(&message, header(0)))
            .unwrap()
            .fields;

        assert_eq!(fields["vtol_state"], json!({"type": "MAV_VTOL_STATE_MC"}));
        assert_eq!(
            fields["landed_state"],
            json!({"type": "MAV_LANDED_STATE_IN_AIR"})
        );
    }

    #[test]
    fn decodes_and_preserves_v1_frames() {
        let mut frame = MAVLinkV1MessageRaw::new();
        frame.serialize_message(header(4), &heartbeat());
        let bytes = frame.raw_bytes().to_vec();

        let decoded = decode_raw_bytes(&bytes).unwrap();
        assert_eq!(decoded.mavlink_version, 1);
        assert_eq!(decoded.sequence, 4);
        assert_eq!(decoded.raw, bytes);
    }

    #[test]
    fn builds_calibration_command_long_from_typed_fields() {
        let fields = object(json!({
            "target_system": 1,
            "target_component": 1,
            "command": {"type": "MAV_CMD_COMPONENT_ARM_DISARM"},
            "confirmation": 0,
            "param1": 1.0,
            "param2": 0.0,
            "param3": 0.0,
            "param4": 0.0,
            "param5": 0.0,
            "param6": 0.0,
            "param7": 0.0,
        }));

        let MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            command, param1, ..
        }) = message_from_fields("COMMAND_LONG", &fields).unwrap()
        else {
            panic!("expected COMMAND_LONG");
        };
        assert_eq!(command, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM);
        assert_eq!(param1, 1.0);
    }

    #[test]
    fn numeric_enums_unknown_names_and_reserved_type_key_are_rejected() {
        let mut fields = object(json!({"command": 400}));
        assert!(matches!(
            message_from_fields("COMMAND_LONG", &fields),
            Err(CodecError::InvalidFields { .. })
        ));
        assert!(matches!(
            message_from_fields("NOT_A_MESSAGE", &fields),
            Err(CodecError::UnknownMessage(_))
        ));
        fields.insert("type".to_owned(), json!("HEARTBEAT"));
        assert!(matches!(
            message_from_fields("COMMAND_LONG", &fields),
            Err(CodecError::InvalidFields { .. })
        ));
    }

    /// A HEARTBEAT whose MAV_TYPE byte is outside the dialect but whose CRC is valid.
    fn heartbeat_with_unknown_type(sequence: u8) -> Vec<u8> {
        let mut bytes = serialize_message(&heartbeat(), header(sequence));
        let payload_start = 10;
        bytes[payload_start + 4] = 250;
        let crc_end = payload_start + 9;
        let crc = calculate_crc(&bytes[1..crc_end], 50);
        bytes[crc_end..crc_end + 2].copy_from_slice(&crc.to_le_bytes());
        bytes
    }

    #[test]
    fn unknown_enum_value_drops_only_that_frame() {
        let invalid = heartbeat_with_unknown_type(1);
        assert!(matches!(
            decode_raw_bytes(&invalid),
            Err(CodecError::Parse(_))
        ));

        let valid = serialize_message(&heartbeat(), header(2));
        let stream = [invalid, valid.clone()].concat();
        let decoded = tokio::runtime::Builder::new_current_thread()
            .build()
            .unwrap()
            .block_on(async {
                let mut reader = FrameReader::new(Cursor::new(stream));
                let decoded = reader.recv().await.unwrap();
                assert_eq!(reader.take_dropped_frames(), 1);
                assert_eq!(reader.take_dropped_frames(), 0);
                decoded
            });

        assert_eq!(decoded.sequence, 2);
        assert_eq!(decoded.raw, valid);
    }
}
