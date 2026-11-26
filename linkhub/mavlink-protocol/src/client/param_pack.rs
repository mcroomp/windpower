//! Decoder for ArduPilot's MAVFTP parameter file `@PARAM/param.pck`, as
//! written by `AP_Filesystem_Param::pack_param`.

use serde::Serialize;

use super::{OperationError, ParameterValue};

/// Path of the packed parameter file, including default values that differ.
pub const PARAM_PACK_PATH: &str = "@PARAM/param.pck?withdefaults=1";

const MAGIC: u16 = 0x671b;
const MAGIC_WITH_DEFAULTS: u16 = 0x671c;
const FLAG_HAS_DEFAULT: u8 = 1;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct PackedParameter {
    pub name: String,
    #[serde(flatten)]
    pub value: ParameterValue,
    /// The firmware default, present only when it differs from `value`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub default: Option<ParameterValue>,
}

/// Decodes a complete `param.pck` and checks it holds every parameter.
pub fn decode_param_pack(bytes: &[u8]) -> Result<Vec<PackedParameter>, OperationError> {
    let invalid = |message: String| OperationError::Invalid(format!("param.pck: {message}"));
    let word = |offset: usize| {
        bytes
            .get(offset..offset + 2)
            .map(|pair| u16::from_le_bytes([pair[0], pair[1]]))
            .ok_or_else(|| invalid("truncated header".to_owned()))
    };
    let magic = word(0)?;
    if magic != MAGIC && magic != MAGIC_WITH_DEFAULTS {
        return Err(invalid(format!("unknown magic {magic:#06x}")));
    }
    let count = usize::from(word(2)?);
    let total = usize::from(word(4)?);
    if count != total {
        return Err(invalid(format!("holds {count} of {total} parameters")));
    }
    let mut parameters = Vec::with_capacity(count);
    let mut previous = String::new();
    let mut offset = 6;
    while offset < bytes.len() {
        // Zero bytes pad entries so values do not straddle FTP read packets.
        if bytes[offset] == 0 {
            offset += 1;
            continue;
        }
        let truncated = || invalid(format!("truncated entry at byte {offset}"));
        let kind = bytes[offset] & 0x0f;
        let flags = bytes[offset] >> 4;
        let lengths = *bytes.get(offset + 1).ok_or_else(truncated)?;
        let common = usize::from(lengths & 0x0f);
        let suffix = usize::from(lengths >> 4) + 1;
        if common > previous.len() || common + suffix > 16 {
            return Err(invalid(format!("invalid name lengths at byte {offset}")));
        }
        let name_start = offset + 2;
        let suffix_bytes = bytes
            .get(name_start..name_start + suffix)
            .ok_or_else(truncated)?;
        let suffix_text = std::str::from_utf8(suffix_bytes)
            .map_err(|_| invalid(format!("non-UTF-8 name at byte {offset}")))?;
        let name = format!("{}{suffix_text}", &previous[..common]);
        let width = match kind {
            1 => 1,
            2 => 2,
            3 | 4 => 4,
            _ => return Err(invalid(format!("unknown type {kind} for {name}"))),
        };
        let value_start = name_start + suffix;
        let decode = |start: usize| -> Result<ParameterValue, OperationError> {
            let data = bytes.get(start..start + width).ok_or_else(truncated)?;
            Ok(match kind {
                1 => ParameterValue::Int8(i8::from_le_bytes([data[0]])),
                2 => ParameterValue::Int16(i16::from_le_bytes([data[0], data[1]])),
                3 => {
                    ParameterValue::Int32(i32::from_le_bytes([data[0], data[1], data[2], data[3]]))
                }
                _ => {
                    ParameterValue::Real32(f32::from_le_bytes([data[0], data[1], data[2], data[3]]))
                }
            })
        };
        let value = decode(value_start)?;
        let has_default = flags & FLAG_HAS_DEFAULT != 0;
        if has_default && magic != MAGIC_WITH_DEFAULTS {
            return Err(invalid(format!(
                "{name} has a default without the defaults magic"
            )));
        }
        let default = has_default
            .then(|| decode(value_start + width))
            .transpose()?;
        offset = value_start + width * if has_default { 2 } else { 1 };
        parameters.push(PackedParameter {
            name: name.clone(),
            value,
            default,
        });
        previous = name;
    }
    if parameters.len() != count {
        return Err(invalid(format!(
            "header lists {count} parameters but {} were decoded",
            parameters.len()
        )));
    }
    Ok(parameters)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(kind: u8, flags: u8, common: u8, suffix: &str, data: &[u8]) -> Vec<u8> {
        let mut bytes = vec![
            kind | (flags << 4),
            common | ((suffix.len() as u8 - 1) << 4),
        ];
        bytes.extend_from_slice(suffix.as_bytes());
        bytes.extend_from_slice(data);
        bytes
    }

    #[test]
    fn decodes_shared_prefixes_padding_types_and_defaults() {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&MAGIC_WITH_DEFAULTS.to_le_bytes());
        bytes.extend_from_slice(&3_u16.to_le_bytes());
        bytes.extend_from_slice(&3_u16.to_le_bytes());
        bytes.extend(entry(
            4,
            1,
            0,
            "ATC_RAT_YAW_P",
            &[0.12_f32.to_le_bytes(), 0.18_f32.to_le_bytes()].concat(),
        ));
        bytes.extend([0, 0]);
        bytes.extend(entry(1, 0, 8, "YAW_D", &[0xfe]));
        bytes.extend(entry(3, 0, 0, "SERVO9_FUNCTION", &36_i32.to_le_bytes()));

        let parameters = decode_param_pack(&bytes).expect("decoded");
        assert_eq!(
            parameters,
            vec![
                PackedParameter {
                    name: "ATC_RAT_YAW_P".to_owned(),
                    value: ParameterValue::Real32(0.12),
                    default: Some(ParameterValue::Real32(0.18)),
                },
                PackedParameter {
                    name: "ATC_RAT_YAW_D".to_owned(),
                    value: ParameterValue::Int8(-2),
                    default: None,
                },
                PackedParameter {
                    name: "SERVO9_FUNCTION".to_owned(),
                    value: ParameterValue::Int32(36),
                    default: None,
                },
            ]
        );
    }

    #[test]
    fn rejects_incomplete_or_inconsistent_files() {
        let mut header = Vec::new();
        header.extend_from_slice(&MAGIC.to_le_bytes());
        header.extend_from_slice(&1_u16.to_le_bytes());
        header.extend_from_slice(&2_u16.to_le_bytes());
        assert!(decode_param_pack(&header).is_err());

        let mut bytes = Vec::new();
        bytes.extend_from_slice(&MAGIC.to_le_bytes());
        bytes.extend_from_slice(&2_u16.to_le_bytes());
        bytes.extend_from_slice(&2_u16.to_le_bytes());
        bytes.extend(entry(1, 0, 0, "A", &[1]));
        assert!(decode_param_pack(&bytes).is_err());
        bytes.extend(entry(4, 0, 0, "B", &[0, 0]));
        assert!(decode_param_pack(&bytes).is_err());
    }
}
