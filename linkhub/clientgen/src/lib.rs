//! Python and TypeScript types for every message and enumeration in the ArduPilot
//! MAVLink definitions LinkHub is built against.
//!
//! The model is bindgen's own parse of the vendored XML, so names, field order,
//! extension fields and bitmask detection cannot drift from the generated Rust
//! dialect. The tests also compare the shapes declared here with what the dialect
//! serializes for every message.

use std::{collections::HashSet, fmt::Write as _, path::Path};

use mavlink_bindgen::{
    BindGenError,
    parser::{MavEnum, MavField, MavMessage, MavProfile, MavType, parse_profile},
};

const DEFINITIONS_FILE: &str = "ardupilotmega.xml";

const PYTHON_KEYWORDS: &[&str] = &[
    "False", "None", "True", "and", "as", "assert", "async", "await", "break", "class", "continue",
    "def", "del", "elif", "else", "except", "finally", "for", "from", "global", "if", "import",
    "in", "is", "lambda", "nonlocal", "not", "or", "pass", "raise", "return", "try", "while",
    "with", "yield",
];

/// Parses `ardupilotmega.xml` and the files it includes from `definitions_dir`.
pub fn load_profile(definitions_dir: &Path) -> Result<MavProfile, BindGenError> {
    parse_profile(
        definitions_dir,
        Path::new(DEFINITIONS_FILE),
        &mut HashSet::new(),
    )
}

/// How LinkHub's JSON represents a field (the serde shape of the Rust dialect).
#[derive(Clone, Copy)]
enum Kind<'a> {
    Int,
    Float,
    Text,
    IntArray,
    FloatArray,
    /// `{"type": "<ENTRY_NAME>"}`
    Enum(&'a MavEnum),
    /// `"<ENTRY_NAME> | <ENTRY_NAME>"`, empty when no flag is set.
    Flags(&'a MavEnum),
}

fn is_float(mavtype: &MavType) -> bool {
    matches!(mavtype, MavType::Float | MavType::Double)
}

fn field_kind<'a>(profile: &'a MavProfile, field: &MavField) -> Kind<'a> {
    match &field.mavtype {
        MavType::CharArray(_) => Kind::Text,
        // The dialect generates arrays of enum values as plain primitive arrays.
        MavType::Array(element, _) if is_float(element) => Kind::FloatArray,
        MavType::Array(..) => Kind::IntArray,
        scalar => match field
            .enumtype
            .as_deref()
            .and_then(|name| profile.enums.get(name))
        {
            Some(enumeration) if enumeration.bitmask => Kind::Flags(enumeration),
            Some(enumeration) => Kind::Enum(enumeration),
            None if is_float(scalar) => Kind::Float,
            None => Kind::Int,
        },
    }
}

struct Member<'a> {
    name: String,
    wire: &'a str,
}

fn is_identifier(name: &str) -> bool {
    let mut chars = name.chars();
    chars
        .next()
        .is_some_and(|first| first.is_ascii_alphabetic())
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

/// Member names drop the prefix shared by every entry (`MAV_CMD_`), keeping the
/// full entry name where that would not be a valid or unique identifier.
fn members(enumeration: &MavEnum) -> Vec<Member<'_>> {
    let names: Vec<&str> = enumeration
        .entries
        .iter()
        .map(|entry| entry.name.as_str())
        .collect();
    let mut prefix_len = 0;
    if let [first, rest @ ..] = names.as_slice()
        && !rest.is_empty()
    {
        let shared = rest.iter().fold(first.len(), |len, name| {
            first
                .bytes()
                .zip(name.bytes())
                .take(len)
                .take_while(|(a, b)| a == b)
                .count()
        });
        prefix_len = first[..shared].rfind('_').map_or(0, |index| index + 1);
    }
    let mut taken = HashSet::new();
    names
        .into_iter()
        .map(|wire| {
            let short = &wire[prefix_len..];
            let name = if is_identifier(short) && !taken.contains(short) {
                short
            } else {
                wire
            };
            taken.insert(name.to_owned());
            Member {
                name: name.to_owned(),
                wire,
            }
        })
        .collect()
}

/// The member a freshly constructed message holds, like the Rust `Default`.
fn default_member<'a>(
    enumeration: &'a MavEnum,
    members: &'a [Member<'a>],
) -> Option<&'a Member<'a>> {
    enumeration
        .entries
        .iter()
        .position(|entry| entry.value == Some(0))
        .or(if members.is_empty() { None } else { Some(0) })
        .map(|index| &members[index])
}

fn pascal_case(name: &str) -> String {
    name.split('_')
        .filter(|part| !part.is_empty())
        .map(|part| {
            let mut chars = part.chars();
            chars.next().map_or_else(String::new, |first| {
                first.to_ascii_uppercase().to_string() + &chars.as_str().to_ascii_lowercase()
            })
        })
        .collect()
}

fn messages_by_id(profile: &MavProfile) -> Vec<&MavMessage> {
    let mut messages: Vec<&MavMessage> = profile.messages.values().collect();
    messages.sort_by_key(|message| message.id);
    messages
}

/// Every generated top-level name must be unique across enums and messages.
fn check_names(profile: &MavProfile, reserved: &[&str]) {
    let mut seen: HashSet<String> = reserved.iter().map(|name| (*name).to_owned()).collect();
    for name in profile
        .enums
        .values()
        .map(|enumeration| enumeration.name.clone())
    {
        assert!(seen.insert(name.clone()), "duplicate generated name {name}");
    }
    for message in profile.messages.values() {
        let name = pascal_case(&message.name);
        assert!(seen.insert(name.clone()), "duplicate generated name {name}");
    }
}

const PYTHON_PREAMBLE: &str = r#""""Generated from ArduPilot's MAVLink definitions (linkhub/dialect). Do not edit.

Regenerate with `cargo run --manifest-path linkhub/Cargo.toml -p linkhub-clientgen`.

MAVLink enumerations are `{"type": "<NAME>"}` objects on the wire and bitmasks
are `" | "`-joined name strings. Enumerations decode to `WireEnum` members and
bitmasks to `frozenset`s of members; names this file does not list are kept
verbatim as members named after the wire name. Every message field defaults to
its zero value, like the Rust dialect's `Default`.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from enum import StrEnum
from typing import Any, ClassVar


class WireEnum(StrEnum):
    @classmethod
    def _missing_(cls, value: object):
        if not isinstance(value, str):
            return None
        member = str.__new__(cls, value)
        member._name_ = value
        member._value_ = value
        return member


def parse_flags(enum_type: type[WireEnum], value: object) -> frozenset[Any]:
    """Decode a wire bitmask string (or an iterable of names) to enum members."""
    names = value.split("|") if isinstance(value, str) else value
    return frozenset(
        enum_type(name.strip()) for name in names if str(name).strip()  # type: ignore[union-attr]
    )


def encode_flags(flags: frozenset[WireEnum]) -> str:
    return " | ".join(sorted(str(flag) for flag in flags))

"#;

const PYTHON_MESSAGE_BASE: &str = r#"
@dataclass(frozen=True)
class Message:
    MAVLINK_TYPE: ClassVar[str]
    MAVLINK_ID: ClassVar[int]

    def get_type(self) -> str:
        return self.MAVLINK_TYPE

    def to_request(self) -> tuple[str, dict[str, Any]]:
        """The message name and its wire field object."""
        values: dict[str, Any] = {}
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, WireEnum):
                value = {"type": str(value)}
            elif isinstance(value, frozenset):
                value = encode_flags(value)
            elif isinstance(value, tuple):
                value = list(value)
            values[field.name] = value
        return self.MAVLINK_TYPE, values

"#;

pub fn python_types(profile: &MavProfile) -> String {
    check_names(profile, &["Message", "WireEnum"]);
    let mut out = String::from(PYTHON_PREAMBLE);
    for enumeration in profile.enums.values() {
        let _ = writeln!(out, "\nclass {}(WireEnum):", enumeration.name);
        let members = members(enumeration);
        if members.is_empty() {
            out.push_str("    pass\n");
        }
        for member in &members {
            let _ = writeln!(out, "    {} = \"{}\"", member.name, member.wire);
        }
    }
    out.push_str(PYTHON_MESSAGE_BASE);

    let messages = messages_by_id(profile);
    let mut enum_fields = String::new();
    let mut flag_fields = String::new();
    let mut tuple_fields = String::new();
    for message in &messages {
        let class = pascal_case(&message.name);
        let _ = writeln!(out, "\n@dataclass(frozen=True, kw_only=True)");
        let _ = writeln!(out, "class {class}(Message):");
        let _ = writeln!(
            out,
            "    MAVLINK_TYPE: ClassVar[str] = \"{}\"",
            message.name
        );
        let _ = writeln!(out, "    MAVLINK_ID: ClassVar[int] = {}", message.id);
        let (mut enums, mut flags, mut tuples) = (String::new(), String::new(), Vec::new());
        for field in &message.fields {
            assert!(
                !PYTHON_KEYWORDS.contains(&field.name.as_str()),
                "{}.{} is a Python keyword",
                message.name,
                field.name
            );
            let name = &field.name;
            match field_kind(profile, field) {
                Kind::Int => {
                    let _ = writeln!(out, "    {name}: int = 0");
                }
                Kind::Float => {
                    let _ = writeln!(out, "    {name}: float = 0.0");
                }
                Kind::Text => {
                    let _ = writeln!(out, "    {name}: str = \"\"");
                }
                Kind::IntArray => {
                    let _ = writeln!(out, "    {name}: tuple[int, ...] = ()");
                    tuples.push(name.as_str());
                }
                Kind::FloatArray => {
                    let _ = writeln!(out, "    {name}: tuple[float, ...] = ()");
                    tuples.push(name.as_str());
                }
                Kind::Enum(enumeration) => {
                    let members = members(enumeration);
                    let default = default_member(enumeration, &members).map_or_else(
                        || format!("{}(\"\")", enumeration.name),
                        |member| format!("{}.{}", enumeration.name, member.name),
                    );
                    let _ = writeln!(out, "    {name}: {} = {default}", enumeration.name);
                    let _ = writeln!(enums, "        \"{name}\": {},", enumeration.name);
                }
                Kind::Flags(enumeration) => {
                    let _ = writeln!(
                        out,
                        "    {name}: frozenset[{}] = frozenset()",
                        enumeration.name
                    );
                    let _ = writeln!(flags, "        \"{name}\": {},", enumeration.name);
                }
            }
        }
        if !enums.is_empty() {
            let _ = writeln!(enum_fields, "    {class}: {{\n{enums}    }},");
        }
        if !flags.is_empty() {
            let _ = writeln!(flag_fields, "    {class}: {{\n{flags}    }},");
        }
        if !tuples.is_empty() {
            let names: Vec<String> = tuples.iter().map(|name| format!("\"{name}\"")).collect();
            let _ = writeln!(
                tuple_fields,
                "    {class}: frozenset([{}]),",
                names.join(", ")
            );
        }
    }

    out.push_str("\n\nMESSAGE_TYPES: dict[str, type[Message]] = {\n");
    for message in &messages {
        let _ = writeln!(
            out,
            "    \"{}\": {},",
            message.name,
            pascal_case(&message.name)
        );
    }
    out.push_str("}\n\n");
    let _ = writeln!(
        out,
        "ENUM_FIELDS: dict[type[Message], dict[str, type[WireEnum]]] = {{\n{enum_fields}}}\n"
    );
    let _ = writeln!(
        out,
        "FLAG_FIELDS: dict[type[Message], dict[str, type[WireEnum]]] = {{\n{flag_fields}}}\n"
    );
    let _ = writeln!(
        out,
        "TUPLE_FIELDS: dict[type[Message], frozenset[str]] = {{\n{tuple_fields}}}"
    );
    out
}

const TYPESCRIPT_PREAMBLE: &str = r#"// Generated from ArduPilot's MAVLink definitions (linkhub/dialect). Do not edit.
// Regenerate with `cargo run --manifest-path linkhub/Cargo.toml -p linkhub-clientgen`.

export interface MavlinkMessage<TFields extends object> {
  readonly message: string;
  readonly fields: TFields;
}

/** A MAVLink enumeration value on the wire; names outside the listed members occur. */
export interface MavEnum<TName extends string> {
  readonly type: TName | string;
}

/** A MAVLink bitmask on the wire: member names joined by " | ", empty when no flag is set. */
export type MavFlags = string;
"#;

pub fn typescript_types(profile: &MavProfile) -> String {
    check_names(profile, &["MavlinkMessage", "MavEnum", "MavFlags"]);
    let mut out = String::from(TYPESCRIPT_PREAMBLE);
    for enumeration in profile.enums.values() {
        let _ = writeln!(out, "\nexport enum {} {{", enumeration.name);
        for member in members(enumeration) {
            let _ = writeln!(out, "  {} = \"{}\",", member.name, member.wire);
        }
        out.push_str("}\n");
    }
    for message in messages_by_id(profile) {
        let class = pascal_case(&message.name);
        let _ = writeln!(out, "\nexport interface {class}Fields {{");
        for field in &message.fields {
            let ty = match field_kind(profile, field) {
                Kind::Int | Kind::Float => "number".to_owned(),
                Kind::Text => "string".to_owned(),
                Kind::IntArray | Kind::FloatArray => "readonly number[]".to_owned(),
                Kind::Enum(enumeration) => format!("MavEnum<{}>", enumeration.name),
                Kind::Flags(_) => "MavFlags".to_owned(),
            };
            // The dialect defaults extension fields when a request omits them.
            let optional = if field.is_extension { "?" } else { "" };
            let _ = writeln!(out, "  readonly {}{optional}: {ty};", field.name);
        }
        out.push_str("}\n\n");
        let _ = writeln!(
            out,
            "export class {class} implements MavlinkMessage<{class}Fields> {{\n  readonly message = \"{}\";\n  constructor(readonly fields: {class}Fields) {{}}\n}}",
            message.name
        );
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use linkhub_dialect::{Message as _, dialects::ardupilotmega::MavMessage as DialectMessage};
    use serde_json::Value;

    fn profile() -> MavProfile {
        load_profile(Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../dialect/definitions"
        )))
        .expect("ardupilotmega profile")
    }

    #[test]
    fn committed_artifacts_are_current() {
        let profile = profile();
        assert_eq!(
            python_types(&profile),
            include_str!("../../../linkhub_client/src/linkhub_client/generated_protocol.py"),
            "regenerate with `cargo run -p linkhub-clientgen`"
        );
        assert_eq!(
            typescript_types(&profile),
            include_str!("../../../linkhub-ui/src/generated/protocol.ts"),
            "regenerate with `cargo run -p linkhub-clientgen`"
        );
    }

    #[test]
    fn declared_shapes_match_what_the_dialect_serializes_for_every_message() {
        let profile = profile();
        assert!(profile.messages.len() > 250, "profile looks truncated");
        for message in profile.messages.values() {
            let default = DialectMessage::default_message_from_id(message.id)
                .unwrap_or_else(|| panic!("{} is not in the dialect", message.name));
            let json = serde_json::to_value(&default).expect("message JSON");
            let object = json.as_object().expect("message object");
            assert_eq!(object["type"], message.name.as_str());

            let mut wire_fields: Vec<&str> = object
                .keys()
                .map(String::as_str)
                .filter(|key| *key != "type")
                .collect();
            let mut declared: Vec<&str> = message.fields.iter().map(|f| f.name.as_str()).collect();
            wire_fields.sort_unstable();
            declared.sort_unstable();
            assert_eq!(wire_fields, declared, "{} fields", message.name);

            for field in &message.fields {
                let value = &object[&field.name];
                let context = format!("{}.{} = {value}", message.name, field.name);
                match field_kind(&profile, field) {
                    Kind::Int | Kind::Float => assert!(value.is_number(), "{context}"),
                    Kind::Text => assert!(value.is_string(), "{context}"),
                    Kind::IntArray | Kind::FloatArray => assert!(
                        value
                            .as_array()
                            .is_some_and(|items| items.iter().all(Value::is_number)),
                        "{context}"
                    ),
                    Kind::Enum(enumeration) => {
                        let name = value["type"].as_str().expect(&context);
                        assert!(
                            enumeration.entries.iter().any(|entry| entry.name == name),
                            "{context} is not an entry of {}",
                            enumeration.name
                        );
                    }
                    Kind::Flags(_) => assert!(value.is_string(), "{context}"),
                }
            }
        }
    }

    #[test]
    fn member_names_drop_the_shared_prefix_and_stay_unique() {
        let profile = profile();
        let command = members(&profile.enums["MavCmd"]);
        assert!(command.iter().any(|m| m.name == "COMPONENT_ARM_DISARM"));
        let axis = members(&profile.enums["PidTuningAxis"]);
        assert!(
            axis.iter()
                .any(|m| m.name == "ROLL" && m.wire == "PID_TUNING_ROLL")
        );
        for enumeration in profile.enums.values() {
            let mut seen = HashSet::new();
            for member in members(enumeration) {
                assert!(
                    is_identifier(&member.name) && seen.insert(member.name.clone()),
                    "{}.{}",
                    enumeration.name,
                    member.name
                );
            }
        }
    }
}
