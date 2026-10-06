use std::io::Cursor;

pub use linkhub_mavio_dialect::EncodedMessage;
use linkhub_mavio_dialect::{
    CodecError as DialectError, ProtocolVersion, decode_message,
    encode_message as encode_dialect_message, message_info_by_id, message_info_by_name,
};
use mavio::{
    Frame, Receiver,
    error::{Error as MavioError, FrameError},
    io::StdIoReader,
    protocol::{MavLinkVersion, V2, Versionless},
};
use serde_json::{Map, Value};
use thiserror::Error;

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
    #[error(transparent)]
    Dialect(#[from] DialectError),
    #[error("unsupported inbound MAVLink message ID {0}")]
    UnsupportedMessageId(u32),
}

pub fn decode_raw_bytes(bytes: &[u8]) -> Result<DecodedMessage, CodecError> {
    let reader = StdIoReader::new(Cursor::new(bytes));
    let mut receiver = Receiver::versionless(reader);
    decode_raw(receiver.recv()?)
}

pub fn decode_raw(frame: Frame<Versionless>) -> Result<DecodedMessage, CodecError> {
    let message_id = frame.message_id();
    let info =
        message_info_by_id(message_id).ok_or(CodecError::UnsupportedMessageId(message_id))?;
    frame
        .validate_checksum_with_crc_extra(info.crc_extra)
        .map_err(|error| CodecError::Mavio(error.into()))?;
    let protocol_version = match frame.version() {
        MavLinkVersion::V1 => ProtocolVersion::V1,
        MavLinkVersion::V2 => ProtocolVersion::V2,
    };
    let fields = decode_message(message_id, frame.payload().bytes(), protocol_version)?;
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
        name: info.name.to_owned(),
        fields,
    })
}

pub fn encode_message(
    name: &str,
    fields: &Map<String, Value>,
) -> Result<EncodedMessage, CodecError> {
    Ok(encode_dialect_message(name, fields)?)
}

#[must_use]
pub fn message_id_from_name(name: &str) -> Option<u32> {
    message_info_by_name(name).map(|info| info.id)
}

pub fn serialize_message(
    message: &EncodedMessage,
    header: MavHeader,
) -> Result<Vec<u8>, CodecError> {
    let frame = Frame::builder()
        .sequence(header.sequence)
        .system_id(header.system_id)
        .component_id(header.component_id)
        .version(V2)
        .message_id(message.message_id)
        .payload(&message.payload)
        .crc_extra(message.crc_extra)
        .build();
    let mut bytes = vec![0; frame.size()];
    frame.serialize(&mut bytes)?;
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavio::protocol::V1;
    use serde_json::json;

    fn object(value: Value) -> Map<String, Value> {
        value.as_object().expect("JSON object").clone()
    }

    fn heartbeat() -> EncodedMessage {
        encode_message(
            "HEARTBEAT",
            &object(json!({
                "custom_mode": 0,
                "type": 6,
                "autopilot": 8,
                "base_mode": 0,
                "system_status": 4,
                "mavlink_version": 3,
            })),
        )
        .expect("encode heartbeat")
    }

    #[test]
    fn heartbeat_matches_pymavlink_and_projects_numeric_enums() {
        let bytes = serialize_message(
            &heartbeat(),
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
    fn decodes_all_ardupilotmega_messages_from_embedded_metadata() {
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
        assert!(!decoded.fields.is_empty());
    }

    #[test]
    fn autopilot_capabilities_project_as_numeric_bits() {
        let message = encode_message(
            "AUTOPILOT_VERSION",
            &object(json!({
                "capabilities": 40,
                "flight_sw_version": 1,
                "middleware_sw_version": 2,
                "os_sw_version": 3,
                "board_version": 4,
                "flight_custom_version": [0, 0, 0, 0, 0, 0, 0, 0],
                "middleware_custom_version": [0, 0, 0, 0, 0, 0, 0, 0],
                "os_custom_version": [0, 0, 0, 0, 0, 0, 0, 0],
                "vendor_id": 5,
                "product_id": 6,
                "uid": 7
            })),
        )
        .unwrap();
        let bytes = serialize_message(
            &message,
            MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            },
        )
        .unwrap();

        assert_eq!(decode_raw_bytes(&bytes).unwrap().fields["capabilities"], 40);
    }

    #[test]
    fn extended_system_state_projects_numeric_enums() {
        let message = encode_message(
            "EXTENDED_SYS_STATE",
            &object(json!({"vtol_state": 3, "landed_state": 2})),
        )
        .unwrap();
        let bytes = serialize_message(
            &message,
            MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            },
        )
        .unwrap();
        let fields = decode_raw_bytes(&bytes).unwrap().fields;

        assert_eq!(fields["vtol_state"], 3);
        assert_eq!(fields["landed_state"], 2);
    }

    #[test]
    fn decodes_and_preserves_v1_frames() {
        let message = heartbeat();
        let frame = Frame::builder()
            .sequence(4)
            .system_id(255)
            .component_id(0)
            .version(V1)
            .message_id(message.message_id)
            .payload(&message.payload)
            .crc_extra(message.crc_extra)
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
        let message = encode_message(
            "COMMAND_LONG",
            &object(json!({
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
                "param7": 0.0,
            })),
        )
        .unwrap();

        assert_eq!(message.message_id, 76);
        assert_eq!(message.crc_extra, 152);
    }
}
