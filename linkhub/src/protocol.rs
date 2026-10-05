use std::fmt::Write as _;

use serde_json::{Map, Value, json};

pub const PROTOCOL_SCHEMA_VERSION: u16 = 1;

struct EnumSpec {
    name: &'static str,
    values: &'static [(&'static str, i64)],
}

#[derive(Clone, Copy)]
enum FieldType {
    Int,
    Float,
    String,
    OptionalInt,
    OptionalFloat,
    IntTuple,
    FloatSequence,
    Enum(&'static str),
}

struct FieldSpec {
    name: &'static str,
    kind: FieldType,
    default: Option<&'static str>,
}

struct MessageSpec {
    class_name: &'static str,
    wire_names: &'static [&'static str],
    fields: &'static [FieldSpec],
    decode: bool,
}

const ENUMS: &[EnumSpec] = &[
    EnumSpec {
        name: "MavSeverity",
        values: &[
            ("EMERGENCY", 0),
            ("ALERT", 1),
            ("CRITICAL", 2),
            ("ERROR", 3),
            ("WARNING", 4),
            ("NOTICE", 5),
            ("INFO", 6),
            ("DEBUG", 7),
        ],
    },
    EnumSpec {
        name: "MavType",
        values: &[("HELICOPTER", 4), ("GCS", 6)],
    },
    EnumSpec {
        name: "MavAutopilot",
        values: &[("ARDUPILOTMEGA", 3), ("INVALID", 8)],
    },
    EnumSpec {
        name: "MavState",
        values: &[("STANDBY", 3), ("ACTIVE", 4)],
    },
    EnumSpec {
        name: "MavVtolState",
        values: &[
            ("UNDEFINED", 0),
            ("TRANSITION_TO_FW", 1),
            ("TRANSITION_TO_MC", 2),
            ("MC", 3),
            ("FW", 4),
        ],
    },
    EnumSpec {
        name: "MavLandedState",
        values: &[
            ("UNDEFINED", 0),
            ("ON_GROUND", 1),
            ("IN_AIR", 2),
            ("TAKEOFF", 3),
            ("LANDING", 4),
        ],
    },
    EnumSpec {
        name: "MavResult",
        values: &[
            ("ACCEPTED", 0),
            ("TEMPORARILY_REJECTED", 1),
            ("DENIED", 2),
            ("UNSUPPORTED", 3),
            ("FAILED", 4),
        ],
    },
    EnumSpec {
        name: "PidAxis",
        values: &[("ROLL", 1), ("PITCH", 2), ("YAW", 3), ("ACCEL_Z", 4)],
    },
    EnumSpec {
        name: "MavParamType",
        values: &[("INT8", 2), ("INT16", 4), ("INT32", 6), ("REAL32", 9)],
    },
];

macro_rules! field {
    ($name:literal, $kind:expr) => {
        FieldSpec {
            name: $name,
            kind: $kind,
            default: None,
        }
    };
    ($name:literal, $kind:expr, $default:literal) => {
        FieldSpec {
            name: $name,
            kind: $kind,
            default: Some($default),
        }
    };
}

const MESSAGES: &[MessageSpec] = &[
    MessageSpec {
        class_name: "StatusText",
        wire_names: &["STATUSTEXT"],
        fields: &[
            field!("text", FieldType::String),
            field!(
                "severity",
                FieldType::Enum("MavSeverity"),
                "MavSeverity.INFO"
            ),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "Attitude",
        wire_names: &["ATTITUDE"],
        fields: &[
            field!("roll", FieldType::Float),
            field!("pitch", FieldType::Float),
            field!("yaw", FieldType::Float),
            field!("rollspeed", FieldType::Float),
            field!("pitchspeed", FieldType::Float),
            field!("yawspeed", FieldType::Float),
            field!("time_boot_ms", FieldType::Int, "0"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "AttitudeQuaternion",
        wire_names: &["ATTITUDE_QUATERNION"],
        fields: &[
            field!("q1", FieldType::Float),
            field!("q2", FieldType::Float),
            field!("q3", FieldType::Float),
            field!("q4", FieldType::Float),
            field!("rollspeed", FieldType::Float, "0.0"),
            field!("pitchspeed", FieldType::Float, "0.0"),
            field!("yawspeed", FieldType::Float, "0.0"),
            field!("time_boot_ms", FieldType::Int, "0"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "LocalPositionNed",
        wire_names: &["LOCAL_POSITION_NED"],
        fields: &[
            field!("x", FieldType::Float),
            field!("y", FieldType::Float),
            field!("z", FieldType::Float),
            field!("vx", FieldType::Float, "0.0"),
            field!("vy", FieldType::Float, "0.0"),
            field!("vz", FieldType::Float, "0.0"),
            field!("time_boot_ms", FieldType::Int, "0"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "GlobalPositionInt",
        wire_names: &["GLOBAL_POSITION_INT"],
        fields: &[
            field!("lat", FieldType::Int),
            field!("lon", FieldType::Int),
            field!("alt", FieldType::Int),
            field!("relative_alt", FieldType::Int),
            field!("vx", FieldType::Int, "0"),
            field!("vy", FieldType::Int, "0"),
            field!("vz", FieldType::Int, "0"),
            field!("hdg", FieldType::Int, "0"),
            field!("time_boot_ms", FieldType::Int, "0"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "EkfStatusReport",
        wire_names: &["EKF_STATUS_REPORT"],
        fields: &[
            field!("flags", FieldType::Int),
            field!("velocity_variance", FieldType::Float, "0.0"),
            field!("pos_horiz_variance", FieldType::Float, "0.0"),
            field!("pos_vert_variance", FieldType::Float, "0.0"),
            field!("compass_variance", FieldType::Float, "0.0"),
            field!("terrain_alt_variance", FieldType::Float, "0.0"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "BatteryStatus",
        wire_names: &["BATTERY_STATUS"],
        fields: &[
            field!("current_battery", FieldType::Int, "-1"),
            field!("battery_remaining", FieldType::Int, "-1"),
            field!("voltages", FieldType::IntTuple, "()"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "SysStatus",
        wire_names: &["SYS_STATUS"],
        fields: &[
            field!("onboard_control_sensors_present", FieldType::Int, "0"),
            field!("onboard_control_sensors_enabled", FieldType::Int, "0"),
            field!("onboard_control_sensors_health", FieldType::Int, "0"),
            field!("load", FieldType::Int, "0"),
            field!("voltage_battery", FieldType::Int, "65535"),
            field!("current_battery", FieldType::Int, "-1"),
            field!("battery_remaining", FieldType::Int, "-1"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "Heartbeat",
        wire_names: &["HEARTBEAT"],
        fields: &[
            field!("type", FieldType::Enum("MavType")),
            field!("autopilot", FieldType::Enum("MavAutopilot")),
            field!("base_mode", FieldType::Int),
            field!("custom_mode", FieldType::Int),
            field!("system_status", FieldType::Enum("MavState")),
            field!("mavlink_version", FieldType::Int, "3"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "ExtendedSysState",
        wire_names: &["EXTENDED_SYS_STATE"],
        fields: &[
            field!("vtol_state", FieldType::Enum("MavVtolState")),
            field!("landed_state", FieldType::Enum("MavLandedState")),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "NamedValueFloat",
        wire_names: &["NAMED_VALUE_FLOAT"],
        fields: &[
            field!("name", FieldType::String),
            field!("value", FieldType::Float),
            field!("time_boot_ms", FieldType::Int, "0"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "NamedValueInt",
        wire_names: &["NAMED_VALUE_INT"],
        fields: &[
            field!("name", FieldType::String),
            field!("value", FieldType::Int),
            field!("time_boot_ms", FieldType::Int, "0"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "CommandAck",
        wire_names: &["COMMAND_ACK"],
        fields: &[
            field!("command", FieldType::Int),
            field!("result", FieldType::Enum("MavResult")),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "CommandLong",
        wire_names: &["COMMAND_LONG"],
        fields: &[
            field!("target_system", FieldType::Int),
            field!("target_component", FieldType::Int),
            field!("command", FieldType::Int),
            field!("confirmation", FieldType::Int, "0"),
            field!("param1", FieldType::Float, "0.0"),
            field!("param2", FieldType::Float, "0.0"),
            field!("param3", FieldType::Float, "0.0"),
            field!("param4", FieldType::Float, "0.0"),
            field!("param5", FieldType::Float, "0.0"),
            field!("param6", FieldType::Float, "0.0"),
            field!("param7", FieldType::Float, "0.0"),
        ],
        decode: false,
    },
    MessageSpec {
        class_name: "SetAttitudeTarget",
        wire_names: &["SET_ATTITUDE_TARGET", "ATTITUDE_TARGET"],
        fields: &[
            field!("target_system", FieldType::Int, "0"),
            field!("target_component", FieldType::Int, "0"),
            field!("type_mask", FieldType::Int, "0"),
            field!("q", FieldType::FloatSequence, "()"),
            field!("body_roll_rate", FieldType::Float, "0.0"),
            field!("body_pitch_rate", FieldType::Float, "0.0"),
            field!("body_yaw_rate", FieldType::Float, "0.0"),
            field!("thrust", FieldType::Float, "0.0"),
            field!("time_boot_ms", FieldType::Int, "0"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "RcChannels",
        wire_names: &["RC_CHANNELS"],
        fields: &[
            field!("chan1_raw", FieldType::OptionalInt, "None"),
            field!("chan2_raw", FieldType::OptionalInt, "None"),
            field!("chan3_raw", FieldType::OptionalInt, "None"),
            field!("chan4_raw", FieldType::OptionalInt, "None"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "ServoOutputRaw",
        wire_names: &["SERVO_OUTPUT_RAW"],
        fields: &[
            field!("servo1_raw", FieldType::Int, "0"),
            field!("servo2_raw", FieldType::Int, "0"),
            field!("servo3_raw", FieldType::Int, "0"),
            field!("servo4_raw", FieldType::Int, "0"),
            field!("servo5_raw", FieldType::Int, "0"),
            field!("servo6_raw", FieldType::Int, "0"),
            field!("servo7_raw", FieldType::Int, "0"),
            field!("servo8_raw", FieldType::Int, "0"),
            field!("servo9_raw", FieldType::Int, "0"),
            field!("servo10_raw", FieldType::Int, "0"),
            field!("servo11_raw", FieldType::Int, "0"),
            field!("servo12_raw", FieldType::Int, "0"),
            field!("servo13_raw", FieldType::Int, "0"),
            field!("servo14_raw", FieldType::Int, "0"),
            field!("servo15_raw", FieldType::Int, "0"),
            field!("servo16_raw", FieldType::Int, "0"),
            field!("port", FieldType::Int, "0"),
            field!("time_usec", FieldType::Int, "0"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "PidTuning",
        wire_names: &["PID_TUNING"],
        fields: &[
            field!("axis", FieldType::Enum("PidAxis"), "PidAxis(-1)"),
            field!("desired", FieldType::OptionalFloat, "None"),
            field!("achieved", FieldType::OptionalFloat, "None"),
            field!("FF", FieldType::OptionalFloat, "None"),
            field!("P", FieldType::OptionalFloat, "None"),
            field!("I", FieldType::OptionalFloat, "None"),
            field!("D", FieldType::OptionalFloat, "None"),
            field!("PDmod", FieldType::OptionalFloat, "None"),
            field!("SRate", FieldType::OptionalFloat, "None"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "ParamSet",
        wire_names: &["PARAM_SET"],
        fields: &[
            field!("target_system", FieldType::Int),
            field!("target_component", FieldType::Int),
            field!("param_id", FieldType::String),
            field!("param_value", FieldType::Float),
            field!("param_type", FieldType::Enum("MavParamType")),
        ],
        decode: false,
    },
    MessageSpec {
        class_name: "ParamValue",
        wire_names: &["PARAM_VALUE"],
        fields: &[
            field!("param_id", FieldType::String),
            field!("param_value", FieldType::Float),
            field!(
                "param_type",
                FieldType::Enum("MavParamType"),
                "MavParamType(0)"
            ),
            field!("param_count", FieldType::Int, "0"),
            field!("param_index", FieldType::Int, "0"),
        ],
        decode: true,
    },
    MessageSpec {
        class_name: "ParamRequestRead",
        wire_names: &["PARAM_REQUEST_READ"],
        fields: &[
            field!("target_system", FieldType::Int),
            field!("target_component", FieldType::Int),
            field!("param_id", FieldType::String),
            field!("param_index", FieldType::Int, "-1"),
        ],
        decode: false,
    },
    MessageSpec {
        class_name: "RequestDataStream",
        wire_names: &["REQUEST_DATA_STREAM"],
        fields: &[
            field!("target_system", FieldType::Int),
            field!("target_component", FieldType::Int),
            field!("req_stream_id", FieldType::Int),
            field!("req_message_rate", FieldType::Int),
            field!("start_stop", FieldType::Int, "1"),
        ],
        decode: false,
    },
];

#[must_use]
pub fn json_schema() -> Value {
    let mut definitions = Map::new();
    for enumeration in ENUMS {
        definitions.insert(
            enumeration.name.to_owned(),
            json!({
                "type": "integer",
                "enum": enumeration.values.iter().map(|(_, value)| value).collect::<Vec<_>>(),
                "x-enum-varnames": enumeration.values.iter().map(|(name, _)| name).collect::<Vec<_>>(),
            }),
        );
    }
    for message in MESSAGES {
        let mut properties = Map::new();
        let mut required = Vec::new();
        for field in message.fields {
            properties.insert(field.name.to_owned(), field_schema(field.kind));
            if field.default.is_none() {
                required.push(field.name);
            }
        }
        definitions.insert(
            message.class_name.to_owned(),
            json!({
                "type": "object",
                "x-mavlink-message-names": message.wire_names,
                "properties": properties,
                "required": required,
                "additionalProperties": false,
            }),
        );
    }
    json!({
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "https://rawes.dev/linkhub/protocol-v1.schema.json",
        "title": "LinkHub protocol",
        "type": "object",
        "properties": {
            "schema_version": {"const": PROTOCOL_SCHEMA_VERSION},
        },
        "required": ["schema_version"],
        "$defs": definitions,
    })
}

fn field_schema(kind: FieldType) -> Value {
    match kind {
        FieldType::Int => json!({"type": "integer"}),
        FieldType::Float => json!({"type": "number"}),
        FieldType::String => json!({"type": "string"}),
        FieldType::OptionalInt => json!({"type": ["integer", "null"]}),
        FieldType::OptionalFloat => json!({"type": ["number", "null"]}),
        FieldType::IntTuple => json!({"type": "array", "items": {"type": "integer"}}),
        FieldType::FloatSequence => json!({"type": "array", "items": {"type": "number"}}),
        FieldType::Enum(name) => json!({"$ref": format!("#/$defs/{name}")}),
    }
}

#[must_use]
pub fn python_types() -> String {
    let mut output = String::from(
        r#""""Generated from LinkHub's Rust protocol descriptor. Do not edit."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import IntEnum
from typing import Any, ClassVar


class WireIntEnum(IntEnum):
    @classmethod
    def _missing_(cls, value: object):
        if not isinstance(value, int):
            return None
        member = int.__new__(cls, value)
        member._name_ = f"UNKNOWN_{value}"
        member._value_ = value
        return member


"#,
    );
    for enumeration in ENUMS {
        writeln!(output, "class {}(WireIntEnum):", enumeration.name).unwrap();
        for (name, value) in enumeration.values {
            writeln!(output, "    {name} = {value}").unwrap();
        }
        output.push('\n');
    }
    output.push_str(
        r#"
@dataclass(frozen=True)
class Message:
    MAVLINK_TYPE: ClassVar[str]

    def get_type(self) -> str:
        return self.MAVLINK_TYPE

    def to_request(self) -> tuple[str, dict[str, Any]]:
        return self.MAVLINK_TYPE, asdict(self)


"#,
    );
    for message in MESSAGES {
        output.push_str("@dataclass(frozen=True)\n");
        writeln!(output, "class {}(Message):", message.class_name).unwrap();
        writeln!(
            output,
            "    MAVLINK_TYPE: ClassVar[str] = {:?}",
            message.wire_names[0]
        )
        .unwrap();
        for field in message.fields {
            write!(output, "    {}: {}", field.name, python_type(field.kind)).unwrap();
            if let Some(default) = field.default {
                write!(output, " = {default}").unwrap();
            }
            output.push('\n');
        }
        output.push('\n');
    }
    output.push_str("\nMESSAGE_TYPES: dict[str, type[Message]] = {\n");
    for message in MESSAGES.iter().filter(|message| message.decode) {
        for wire_name in message.wire_names {
            writeln!(output, "    {wire_name:?}: {},", message.class_name).unwrap();
        }
    }
    output.push_str("}\n\nENUM_FIELDS: dict[type[Message], dict[str, type[WireIntEnum]]] = {\n");
    for message in MESSAGES.iter().filter(|message| message.decode) {
        let enum_fields: Vec<_> = message
            .fields
            .iter()
            .filter_map(|field| match field.kind {
                FieldType::Enum(name) => Some((field.name, name)),
                _ => None,
            })
            .collect();
        if !enum_fields.is_empty() {
            writeln!(output, "    {}: {{", message.class_name).unwrap();
            for (field, enumeration) in enum_fields {
                writeln!(output, "        {field:?}: {enumeration},").unwrap();
            }
            output.push_str("    },\n");
        }
    }
    output.push_str("}\n\nTUPLE_FIELDS: dict[type[Message], frozenset[str]] = {\n");
    for message in MESSAGES.iter().filter(|message| message.decode) {
        let tuple_fields: Vec<_> = message
            .fields
            .iter()
            .filter(|field| matches!(field.kind, FieldType::IntTuple))
            .map(|field| field.name)
            .collect();
        if !tuple_fields.is_empty() {
            writeln!(
                output,
                "    {}: frozenset({tuple_fields:?}),",
                message.class_name
            )
            .unwrap();
        }
    }
    output.push_str("}\n");
    output
}

#[must_use]
pub fn typescript_types() -> String {
    let mut output = String::from(
        r#"// Generated from LinkHub's Rust protocol descriptor. Do not edit.

export interface MavlinkMessage<TFields extends object> {
  readonly message: string;
  readonly fields: TFields;
}

"#,
    );
    for enumeration in ENUMS {
        writeln!(output, "export enum {} {{", enumeration.name).unwrap();
        for (name, value) in enumeration.values {
            writeln!(output, "  {name} = {value},").unwrap();
        }
        output.push_str("}\n\n");
    }
    for message in MESSAGES {
        writeln!(output, "export interface {}Fields {{", message.class_name).unwrap();
        for field in message.fields {
            let optional = if field.default.is_some() { "?" } else { "" };
            writeln!(
                output,
                "  readonly {}{}: {};",
                field.name,
                optional,
                typescript_type(field.kind)
            )
            .unwrap();
        }
        output.push_str("}\n\n");
        writeln!(
            output,
            "export class {} implements MavlinkMessage<{}Fields> {{",
            message.class_name, message.class_name
        )
        .unwrap();
        writeln!(output, "  readonly message = {:?};", message.wire_names[0]).unwrap();
        writeln!(
            output,
            "  constructor(readonly fields: {}Fields) {{}}",
            message.class_name
        )
        .unwrap();
        output.push_str("}\n\n");
    }
    output.push_str("export const MESSAGE_CLASSES = {\n");
    for message in MESSAGES.iter().filter(|message| message.decode) {
        for wire_name in message.wire_names {
            writeln!(output, "  {wire_name:?}: {},", message.class_name).unwrap();
        }
    }
    output.push_str("} as const;\n");
    output
}

fn python_type(kind: FieldType) -> &'static str {
    match kind {
        FieldType::Int => "int",
        FieldType::Float => "float",
        FieldType::String => "str",
        FieldType::OptionalInt => "int | None",
        FieldType::OptionalFloat => "float | None",
        FieldType::IntTuple => "tuple[int, ...]",
        FieldType::FloatSequence => "tuple[float, ...] | list[float]",
        FieldType::Enum(name) => name,
    }
}

fn typescript_type(kind: FieldType) -> &'static str {
    match kind {
        FieldType::Int | FieldType::Float => "number",
        FieldType::String => "string",
        FieldType::OptionalInt | FieldType::OptionalFloat => "number | null",
        FieldType::IntTuple | FieldType::FloatSequence => "readonly number[]",
        FieldType::Enum(name) => name,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schema_and_python_artifacts_are_current() {
        let schema = serde_json::to_string_pretty(&json_schema()).expect("schema") + "\n";
        assert_eq!(schema, include_str!("../schema/protocol-v1.schema.json"));
        assert_eq!(
            python_types(),
            include_str!("../../linkhub_client/src/linkhub_client/generated_protocol.py")
        );
        assert_eq!(
            typescript_types(),
            include_str!("../../linkhub-ui/src/generated/protocol.ts")
        );
    }
}
