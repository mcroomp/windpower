use serde::{Deserialize, Deserializer, Serialize, Serializer, ser::SerializeStruct};

use super::{MavParamType, OperationError};
use crate::codec::json as dialect_json;

/// ArduPilot scalar values, independent of classic-message or packed-file encoding.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParameterValue {
    Int8(i8),
    Int16(i16),
    Int32(i32),
    Real32(f32),
}

impl ParameterValue {
    pub const fn param_type(self) -> MavParamType {
        match self {
            Self::Int8(_) => MavParamType::MAV_PARAM_TYPE_INT8,
            Self::Int16(_) => MavParamType::MAV_PARAM_TYPE_INT16,
            Self::Int32(_) => MavParamType::MAV_PARAM_TYPE_INT32,
            Self::Real32(_) => MavParamType::MAV_PARAM_TYPE_REAL32,
        }
    }

    fn from_number(value: f64, param_type: MavParamType) -> Result<Self, OperationError> {
        if !value.is_finite() {
            return Err(OperationError::Invalid(
                "parameter value must be finite".to_owned(),
            ));
        }
        let bounds = match param_type {
            MavParamType::MAV_PARAM_TYPE_INT8 => (f64::from(i8::MIN), f64::from(i8::MAX)),
            MavParamType::MAV_PARAM_TYPE_INT16 => (f64::from(i16::MIN), f64::from(i16::MAX)),
            MavParamType::MAV_PARAM_TYPE_INT32 => (f64::from(i32::MIN), f64::from(i32::MAX)),
            MavParamType::MAV_PARAM_TYPE_REAL32 => {
                let value = value as f32;
                if !value.is_finite() {
                    return Err(OperationError::Invalid(
                        "parameter value exceeds REAL32 range".to_owned(),
                    ));
                }
                return Ok(Self::Real32(value));
            }
            _ => {
                return Err(OperationError::Invalid(format!(
                    "ArduPilot parameter type {param_type:?} is unsupported"
                )));
            }
        };
        if value.fract() != 0.0 || value < bounds.0 || value > bounds.1 {
            return Err(OperationError::Invalid(format!(
                "parameter value {value} is not a valid {param_type:?}"
            )));
        }
        Ok(match param_type {
            MavParamType::MAV_PARAM_TYPE_INT8 => Self::Int8(value as i8),
            MavParamType::MAV_PARAM_TYPE_INT16 => Self::Int16(value as i16),
            MavParamType::MAV_PARAM_TYPE_INT32 => Self::Int32(value as i32),
            _ => unreachable!("integer parameter type was checked above"),
        })
    }

    pub(super) fn decode_classic(
        value: f32,
        param_type: MavParamType,
    ) -> Result<Self, OperationError> {
        Self::from_number(f64::from(value), param_type)
    }

    /// ArduPilot numerically converts classic parameters rather than packing
    /// integer bits into the float field. Refuse lossy writes.
    pub(super) fn encode_classic(self) -> Result<f32, OperationError> {
        let value = match self {
            Self::Int8(value) => f32::from(value),
            Self::Int16(value) => f32::from(value),
            Self::Int32(value) => {
                let encoded = value as f32;
                if f64::from(encoded) != f64::from(value) {
                    return Err(OperationError::Invalid(format!(
                        "integer parameter value {value} cannot be represented exactly by ArduPilot's float encoding"
                    )));
                }
                encoded
            }
            Self::Real32(value) => value,
        };
        if !value.is_finite() {
            return Err(OperationError::Invalid(
                "parameter value must be finite".to_owned(),
            ));
        }
        Ok(value)
    }
}

impl Serialize for ParameterValue {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        if matches!(self, Self::Real32(value) if !value.is_finite()) {
            return Err(serde::ser::Error::custom("parameter value must be finite"));
        }
        let mut state = serializer.serialize_struct("ParameterValue", 2)?;
        match self {
            Self::Int8(value) => state.serialize_field("value", value)?,
            Self::Int16(value) => state.serialize_field("value", value)?,
            Self::Int32(value) => state.serialize_field("value", value)?,
            Self::Real32(value) => state.serialize_field("value", value)?,
        }
        let param_type = dialect_json::parameter_type::to_value(&self.param_type())
            .map_err(serde::ser::Error::custom)?;
        state.serialize_field("type", &param_type)?;
        state.end()
    }
}

impl<'de> Deserialize<'de> for ParameterValue {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        struct WireValue {
            value: f64,
            #[serde(
                rename = "type",
                default = "default_type",
                with = "dialect_json::parameter_type"
            )]
            param_type: MavParamType,
        }
        fn default_type() -> MavParamType {
            MavParamType::MAV_PARAM_TYPE_REAL32
        }
        let wire = WireValue::deserialize(deserializer)?;
        Self::from_number(wire.value, wire.param_type).map_err(serde::de::Error::custom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn typed_values_roundtrip_flat_parameter_json() {
        for value in [
            ParameterValue::Int8(i8::MIN),
            ParameterValue::Int8(i8::MAX),
            ParameterValue::Int16(i16::MIN),
            ParameterValue::Int16(i16::MAX),
            ParameterValue::Int32(i32::MIN),
            ParameterValue::Int32(i32::MAX),
            ParameterValue::Real32(1.25),
        ] {
            let setting = super::super::ParameterSetting {
                name: "TEST".to_owned(),
                value,
            };
            let encoded = serde_json::to_value(value).unwrap();
            assert_eq!(
                encoded["type"],
                dialect_json::parameter_type::to_value(&value.param_type()).unwrap()
            );
            assert_eq!(
                serde_json::from_value::<ParameterValue>(encoded.clone()).unwrap(),
                value
            );
            let mut named = encoded;
            named["name"] = json!("TEST");
            let decoded: super::super::ParameterSetting = serde_json::from_value(named).unwrap();
            assert_eq!(decoded.value, setting.value);
            let result = super::super::ParameterResult {
                name: setting.name,
                value,
                index: 1,
                count: 2,
                after_cursor: "v1:1".to_owned(),
            };
            let json = serde_json::to_value(result).unwrap();
            assert_eq!(
                json["value"].is_i64(),
                !matches!(value, ParameterValue::Real32(_))
            );
            assert_eq!(
                json["type"],
                dialect_json::parameter_type::to_value(&value.param_type()).unwrap()
            );
            assert_eq!(json["after_cursor"], "v1:1");
        }
    }

    #[test]
    fn json_preserves_int32_values_that_classic_messages_cannot_encode() {
        let value = ParameterValue::Int32(16_777_217);
        let encoded = serde_json::to_value(value).unwrap();

        assert_eq!(encoded["value"].as_i64(), Some(16_777_217));
        assert_eq!(
            serde_json::from_value::<ParameterValue>(encoded).unwrap(),
            value
        );
        assert!(value.encode_classic().is_err());
    }

    #[test]
    fn classic_encoding_is_numeric_and_rejects_lossy_integer_writes() {
        for (value, wire) in [
            (ParameterValue::Int8(-128), -128.0),
            (ParameterValue::Int16(32767), 32767.0),
            (ParameterValue::Int32(16_777_216), 16_777_216.0),
            (ParameterValue::Int32(16_777_218), 16_777_218.0),
            (ParameterValue::Int32(i32::MIN), i32::MIN as f32),
            (ParameterValue::Real32(1.25), 1.25),
        ] {
            assert_eq!(value.encode_classic().unwrap(), wire);
            assert_eq!(
                ParameterValue::decode_classic(wire, value.param_type()).unwrap(),
                value
            );
        }
        for value in [16_777_217, -16_777_217, i32::MAX] {
            assert!(ParameterValue::Int32(value).encode_classic().is_err());
        }
    }

    #[test]
    fn invalid_and_unsupported_values_are_not_silently_coerced() {
        for (param_type, value) in [
            (MavParamType::MAV_PARAM_TYPE_INT8, 128.0),
            (MavParamType::MAV_PARAM_TYPE_INT8, -129.0),
            (MavParamType::MAV_PARAM_TYPE_INT16, 32768.0),
            (MavParamType::MAV_PARAM_TYPE_INT16, -32769.0),
            (MavParamType::MAV_PARAM_TYPE_INT32, 2147483648.0),
            (MavParamType::MAV_PARAM_TYPE_INT32, -2147483904.0),
            (MavParamType::MAV_PARAM_TYPE_INT32, 1.5),
            (MavParamType::MAV_PARAM_TYPE_UINT8, 1.0),
            (MavParamType::MAV_PARAM_TYPE_UINT16, 1.0),
            (MavParamType::MAV_PARAM_TYPE_UINT32, 1.0),
            (MavParamType::MAV_PARAM_TYPE_UINT64, 1.0),
            (MavParamType::MAV_PARAM_TYPE_INT64, 1.0),
            (MavParamType::MAV_PARAM_TYPE_REAL64, 1.0),
            (MavParamType::MAV_PARAM_TYPE_REAL32, f64::MAX),
        ] {
            assert!(ParameterValue::from_number(value, param_type).is_err());
            assert!(
                serde_json::from_value::<ParameterValue>(json!({
                    "value": value,
                    "type": dialect_json::parameter_type::to_value(&param_type).unwrap()
                }))
                .is_err()
            );
        }
        for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert!(ParameterValue::Real32(value).encode_classic().is_err());
            assert!(
                ParameterValue::decode_classic(value, MavParamType::MAV_PARAM_TYPE_REAL32).is_err()
            );
            assert!(serde_json::to_value(ParameterValue::Real32(value)).is_err());
        }
    }
}
