use mavinspect::protocol::{Dialect, MavType, Message};
use serde_json::{Map, Value};
use thiserror::Error;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProtocolVersion {
    V1,
    V2,
}

#[derive(Clone, Copy, Debug)]
pub struct MessageInfo {
    pub id: u32,
    pub name: &'static str,
    pub crc_extra: u8,
}

#[derive(Clone, Debug)]
pub struct EncodedMessage {
    pub message_id: u32,
    pub crc_extra: u8,
    pub payload: Vec<u8>,
}

#[derive(Debug, Error)]
pub enum CodecError {
    #[error("unknown ArduPilotMega message {0}")]
    UnknownMessage(String),
    #[error("unknown ArduPilotMega message ID {0}")]
    UnknownMessageId(u32),
    #[error("invalid field {field} for {message}: {reason}")]
    InvalidField {
        message: String,
        field: String,
        reason: String,
    },
    #[error("invalid payload length {actual} for {message}; expected no more than {maximum} bytes")]
    InvalidPayloadLength {
        message: String,
        actual: usize,
        maximum: usize,
    },
}

#[must_use]
pub fn message_info_by_id(id: u32) -> Option<MessageInfo> {
    dialect().get_message_by_id(id).map(message_info)
}

#[must_use]
pub fn message_info_by_name(name: &str) -> Option<MessageInfo> {
    dialect().get_message_by_name(name).map(message_info)
}

pub fn decode_message(
    message_id: u32,
    payload: &[u8],
    version: ProtocolVersion,
) -> Result<Map<String, Value>, CodecError> {
    let message = message_by_id(message_id)?;
    let maximum = match version {
        ProtocolVersion::V1 => message.size_v1(),
        ProtocolVersion::V2 => message.size_v2(),
    };
    if payload.len() > maximum {
        return Err(CodecError::InvalidPayloadLength {
            message: message.name().to_owned(),
            actual: payload.len(),
            maximum,
        });
    }

    let mut fields = Map::new();
    let mut offset = 0;
    for field in message.fields_v2() {
        fields.insert(
            field.name().to_owned(),
            read_value(field.r#type(), payload, offset),
        );
        offset += field.r#type().size();
    }
    Ok(fields)
}

pub fn encode_message(
    name: &str,
    fields: &Map<String, Value>,
) -> Result<EncodedMessage, CodecError> {
    let message = dialect()
        .get_message_by_name(name)
        .ok_or_else(|| CodecError::UnknownMessage(name.to_owned()))?;

    for field in fields.keys() {
        if message.get_field_by_name(field).is_none() {
            return Err(invalid_field(
                message,
                field,
                "field is not defined by the dialect",
            ));
        }
    }

    let mut payload = vec![0_u8; message.size_v2()];
    let mut offset = 0;
    for field in message.fields_v2() {
        match fields.get(field.name()) {
            Some(value) => write_value(
                message,
                field.name(),
                field.r#type(),
                value,
                &mut payload,
                offset,
            )?,
            None if field.extension() => {}
            None => {
                return Err(invalid_field(
                    message,
                    field.name(),
                    "required field is missing",
                ));
            }
        }
        offset += field.r#type().size();
    }
    while payload.len() > 1 && payload.last() == Some(&0) {
        payload.pop();
    }

    Ok(EncodedMessage {
        message_id: message.id(),
        crc_extra: message.crc_extra(),
        payload,
    })
}

pub fn enum_bit_names(name: &str, bits: u64) -> Result<Vec<String>, CodecError> {
    let definition = dialect()
        .get_enum_by_name(name)
        .ok_or_else(|| CodecError::UnknownMessage(name.to_owned()))?;
    Ok(definition
        .entries()
        .iter()
        .filter(|entry| {
            let value = u64::from(entry.value());
            value != 0 && bits & value == value
        })
        .map(|entry| entry.name().to_owned())
        .collect())
}

fn dialect() -> &'static Dialect {
    mavlink_message_definitions::protocol()
        .get_dialect_by_name("ardupilotmega")
        .expect("embedded ArduPilotMega dialect is available")
}

fn message_by_id(id: u32) -> Result<&'static Message, CodecError> {
    dialect()
        .get_message_by_id(id)
        .ok_or(CodecError::UnknownMessageId(id))
}

fn message_info(message: &'static Message) -> MessageInfo {
    MessageInfo {
        id: message.id(),
        name: message.name(),
        crc_extra: message.crc_extra(),
    }
}

fn read_value(field_type: &MavType, payload: &[u8], offset: usize) -> Value {
    match field_type {
        MavType::UInt8 | MavType::UInt8MavlinkVersion | MavType::Char => {
            Value::from(read_bytes::<1>(payload, offset)[0])
        }
        MavType::UInt16 => Value::from(u16::from_le_bytes(read_bytes(payload, offset))),
        MavType::UInt32 => Value::from(u32::from_le_bytes(read_bytes(payload, offset))),
        MavType::UInt64 => Value::from(u64::from_le_bytes(read_bytes(payload, offset))),
        MavType::Int8 => Value::from(i8::from_le_bytes(read_bytes(payload, offset))),
        MavType::Int16 => Value::from(i16::from_le_bytes(read_bytes(payload, offset))),
        MavType::Int32 => Value::from(i32::from_le_bytes(read_bytes(payload, offset))),
        MavType::Int64 => Value::from(i64::from_le_bytes(read_bytes(payload, offset))),
        MavType::Float => Value::from(f32::from_le_bytes(read_bytes(payload, offset))),
        MavType::Double => Value::from(f64::from_le_bytes(read_bytes(payload, offset))),
        MavType::Array(element, length) if element.base_type() == &MavType::Char => {
            let available_end = payload.len().min(offset.saturating_add(*length));
            let available = payload.get(offset..available_end).unwrap_or_default();
            let text_end = available
                .iter()
                .position(|byte| *byte == 0)
                .unwrap_or(available.len());
            Value::from(String::from_utf8_lossy(&available[..text_end]).into_owned())
        }
        MavType::Array(element, length) => {
            let element_size = element.size();
            Value::Array(
                (0..*length)
                    .map(|index| read_value(element, payload, offset + index * element_size))
                    .collect(),
            )
        }
    }
}

fn read_bytes<const N: usize>(payload: &[u8], offset: usize) -> [u8; N] {
    let mut bytes = [0; N];
    if offset < payload.len() {
        let count = N.min(payload.len() - offset);
        bytes[..count].copy_from_slice(&payload[offset..offset + count]);
    }
    bytes
}

fn write_value(
    message: &Message,
    field: &str,
    field_type: &MavType,
    value: &Value,
    payload: &mut [u8],
    offset: usize,
) -> Result<(), CodecError> {
    match field_type {
        MavType::UInt8 | MavType::UInt8MavlinkVersion | MavType::Char => {
            write_unsigned::<u8>(message, field, value, payload, offset)
        }
        MavType::UInt16 => write_unsigned::<u16>(message, field, value, payload, offset),
        MavType::UInt32 => write_unsigned::<u32>(message, field, value, payload, offset),
        MavType::UInt64 => write_unsigned::<u64>(message, field, value, payload, offset),
        MavType::Int8 => write_signed::<i8>(message, field, value, payload, offset),
        MavType::Int16 => write_signed::<i16>(message, field, value, payload, offset),
        MavType::Int32 => write_signed::<i32>(message, field, value, payload, offset),
        MavType::Int64 => write_signed::<i64>(message, field, value, payload, offset),
        MavType::Float => {
            let number = finite_float(message, field, value)?;
            let number = number as f32;
            if !number.is_finite() {
                return Err(invalid_field(message, field, "value is outside f32 range"));
            }
            payload[offset..offset + 4].copy_from_slice(&number.to_le_bytes());
            Ok(())
        }
        MavType::Double => {
            let number = finite_float(message, field, value)?;
            payload[offset..offset + 8].copy_from_slice(&number.to_le_bytes());
            Ok(())
        }
        MavType::Array(element, length) if element.base_type() == &MavType::Char => {
            let text = value
                .as_str()
                .ok_or_else(|| invalid_field(message, field, "expected a string"))?;
            if text.len() > *length {
                return Err(invalid_field(
                    message,
                    field,
                    format!("string exceeds fixed length {length}"),
                ));
            }
            payload[offset..offset + text.len()].copy_from_slice(text.as_bytes());
            Ok(())
        }
        MavType::Array(element, length) => {
            let values = value
                .as_array()
                .ok_or_else(|| invalid_field(message, field, "expected an array"))?;
            if values.len() != *length {
                return Err(invalid_field(
                    message,
                    field,
                    format!(
                        "expected {length} array elements, received {}",
                        values.len()
                    ),
                ));
            }
            let element_size = element.size();
            for (index, element_value) in values.iter().enumerate() {
                write_value(
                    message,
                    field,
                    element,
                    element_value,
                    payload,
                    offset + index * element_size,
                )?;
            }
            Ok(())
        }
    }
}

trait LittleEndianUnsigned: TryFrom<u64> {
    const SIZE: usize;
    fn write_le(self, target: &mut [u8]);
}

macro_rules! impl_unsigned {
    ($($type:ty),+ $(,)?) => {
        $(
            impl LittleEndianUnsigned for $type {
                const SIZE: usize = size_of::<Self>();

                fn write_le(self, target: &mut [u8]) {
                    target.copy_from_slice(&self.to_le_bytes());
                }
            }
        )+
    };
}

impl_unsigned!(u8, u16, u32, u64);

fn write_unsigned<T: LittleEndianUnsigned>(
    message: &Message,
    field: &str,
    value: &Value,
    payload: &mut [u8],
    offset: usize,
) -> Result<(), CodecError> {
    let number = value
        .as_u64()
        .and_then(|number| T::try_from(number).ok())
        .ok_or_else(|| invalid_field(message, field, "expected an in-range unsigned integer"))?;
    number.write_le(&mut payload[offset..offset + T::SIZE]);
    Ok(())
}

trait LittleEndianSigned: TryFrom<i64> {
    const SIZE: usize;
    fn write_le(self, target: &mut [u8]);
}

macro_rules! impl_signed {
    ($($type:ty),+ $(,)?) => {
        $(
            impl LittleEndianSigned for $type {
                const SIZE: usize = size_of::<Self>();

                fn write_le(self, target: &mut [u8]) {
                    target.copy_from_slice(&self.to_le_bytes());
                }
            }
        )+
    };
}

impl_signed!(i8, i16, i32, i64);

fn write_signed<T: LittleEndianSigned>(
    message: &Message,
    field: &str,
    value: &Value,
    payload: &mut [u8],
    offset: usize,
) -> Result<(), CodecError> {
    let number = value
        .as_i64()
        .and_then(|number| T::try_from(number).ok())
        .ok_or_else(|| invalid_field(message, field, "expected an in-range signed integer"))?;
    number.write_le(&mut payload[offset..offset + T::SIZE]);
    Ok(())
}

fn finite_float(message: &Message, field: &str, value: &Value) -> Result<f64, CodecError> {
    value
        .as_f64()
        .filter(|number| number.is_finite())
        .ok_or_else(|| invalid_field(message, field, "expected a finite number"))
}

fn invalid_field(
    message: &Message,
    field: impl Into<String>,
    reason: impl Into<String>,
) -> CodecError {
    CodecError::InvalidField {
        message: message.name().to_owned(),
        field: field.into(),
        reason: reason.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_every_embedded_message_definition() {
        for definition in dialect().messages() {
            let fields = definition
                .fields()
                .iter()
                .map(|field| (field.name().to_owned(), zero_value(field.r#type())))
                .collect();
            let encoded = encode_message(definition.name(), &fields)
                .unwrap_or_else(|error| panic!("encode {}: {error}", definition.name()));
            let decoded = decode_message(definition.id(), &encoded.payload, ProtocolVersion::V2)
                .unwrap_or_else(|error| panic!("decode {}: {error}", definition.name()));

            assert_eq!(encoded.message_id, definition.id());
            assert_eq!(encoded.crc_extra, definition.crc_extra());
            assert_eq!(decoded.len(), definition.fields().len());
        }
    }

    fn zero_value(field_type: &MavType) -> Value {
        match field_type {
            MavType::Float | MavType::Double => Value::from(0.0),
            MavType::Array(element, _) if element.base_type() == &MavType::Char => Value::from(""),
            MavType::Array(element, length) => {
                Value::Array((0..*length).map(|_| zero_value(element)).collect())
            }
            _ => Value::from(0),
        }
    }
}
