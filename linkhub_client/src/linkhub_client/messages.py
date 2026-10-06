"""Runtime decoding around LinkHub's generated protocol types.

LinkHub's JSON carries MAVLink enumerations as ``{"type": "<NAME>"}`` objects
and bitmasks as ``" | "``-joined name strings (see ``generated_protocol``).
This module turns them into ``WireEnum`` members and ``frozenset``s of members.
"""

from __future__ import annotations

from dataclasses import MISSING, dataclass, fields
from typing import Any, TypeVar

from .generated_protocol import (
    ENUM_FIELDS,
    FLAG_FIELDS,
    MESSAGE_TYPES,
    TUPLE_FIELDS,
    Attitude,
    AttitudeQuaternion,
    AttitudeTarget,
    AttitudeTargetTypemask,
    BatteryStatus,
    CommandAck,
    CommandLong,
    DebugFloatArray,
    EkfStatusFlags,
    EkfStatusReport,
    ExtendedSysState,
    GlobalPositionInt,
    Heartbeat,
    LocalPositionNed,
    MavAutopilot,
    MavCmd,
    MavDataStream,
    MavLandedState,
    MavModeFlag,
    MavParamType,
    MavResult,
    MavSeverity,
    MavState,
    MavSysStatusSensor,
    MavType,
    MavVtolState,
    Message,
    NamedValueFloat,
    NamedValueInt,
    ParamRequestRead,
    ParamSet,
    ParamValue,
    PidTuningAxis,
    PidTuning,
    RcChannels,
    RequestDataStream,
    ServoOutputRaw,
    SetAttitudeTarget,
    Statustext,
    SysStatus,
    WireEnum,
    encode_flags,
    parse_flags,
)

_E = TypeVar("_E", bound=WireEnum)


def decode_enum(enum_type: type[_E], value: Any) -> _E:
    """Decode a wire enumeration: ``{"type": NAME}`` (or a bare name)."""
    if isinstance(value, dict):
        value = value["type"]
    return enum_type(value)


def encode_enum(value: WireEnum | str) -> dict[str, str]:
    """Encode an enumeration value for LinkHub requests and message fields."""
    return {"type": str(value)}


@dataclass(frozen=True)
class RawMessage:
    message_type: str
    values: dict[str, Any]

    def get_type(self) -> str:
        return self.message_type

    def __getattr__(self, name: str) -> Any:
        try:
            return self.values[name]
        except KeyError as exc:
            raise AttributeError(name) from exc


@dataclass(frozen=True)
class EscTelemetry:
    message_name: str
    first_channel: int
    rpm: tuple[int, ...] = ()
    voltage: tuple[int, ...] = ()
    current: tuple[int, ...] = ()
    temperature: tuple[int, ...] = ()

    def get_type(self) -> str:
        return self.message_name


_ESC_CHANNEL_BASE_BY_TYPE = {
    "ESC_TELEMETRY_1_TO_4": 1,
    "ESC_TELEMETRY_5_TO_8": 5,
    "ESC_TELEMETRY_9_TO_12": 9,
}


def decode_message(message: Any) -> Any:
    """Decode a LinkHub raw message or compatible test double."""
    message_name = message.get_type()
    if message_name in _ESC_CHANNEL_BASE_BY_TYPE:
        return EscTelemetry(
            message_name=message_name,
            first_channel=_ESC_CHANNEL_BASE_BY_TYPE[message_name],
            rpm=tuple(int(value) for value in getattr(message, "rpm", ())),
            voltage=tuple(int(value) for value in getattr(message, "voltage", ())),
            current=tuple(int(value) for value in getattr(message, "current", ())),
            temperature=tuple(
                int(value) for value in getattr(message, "temperature", ())
            ),
        )
    message_type = MESSAGE_TYPES.get(message_name)
    if message_type is None:
        return message
    values: dict[str, Any] = {}
    enum_fields = ENUM_FIELDS.get(message_type, {})
    flag_fields = FLAG_FIELDS.get(message_type, {})
    tuple_fields = TUPLE_FIELDS.get(message_type, frozenset())
    for field in fields(message_type):
        if field.default is not MISSING:
            value = getattr(message, field.name, field.default)
        elif field.default_factory is not MISSING:  # type: ignore[misc]
            value = getattr(message, field.name, field.default_factory())
        else:
            value = getattr(message, field.name)
        if field.name in enum_fields:
            value = decode_enum(enum_fields[field.name], value)
        elif field.name in flag_fields:
            value = parse_flags(flag_fields[field.name], value)
        elif field.name in tuple_fields:
            value = tuple(value)
        values[field.name] = value
    return message_type(**values)


__all__ = [
    "Attitude",
    "AttitudeQuaternion",
    "AttitudeTarget",
    "AttitudeTargetTypemask",
    "BatteryStatus",
    "CommandAck",
    "CommandLong",
    "DebugFloatArray",
    "EkfStatusFlags",
    "EkfStatusReport",
    "EscTelemetry",
    "ExtendedSysState",
    "GlobalPositionInt",
    "Heartbeat",
    "LocalPositionNed",
    "MavAutopilot",
    "MavCmd",
    "MavDataStream",
    "MavLandedState",
    "MavModeFlag",
    "MavParamType",
    "MavResult",
    "MavSeverity",
    "MavState",
    "MavSysStatusSensor",
    "MavType",
    "MavVtolState",
    "Message",
    "NamedValueFloat",
    "NamedValueInt",
    "ParamRequestRead",
    "ParamSet",
    "ParamValue",
    "PidTuningAxis",
    "PidTuning",
    "RawMessage",
    "RcChannels",
    "RequestDataStream",
    "ServoOutputRaw",
    "SetAttitudeTarget",
    "Statustext",
    "SysStatus",
    "WireEnum",
    "decode_enum",
    "decode_message",
    "encode_enum",
    "encode_flags",
    "parse_flags",
]
