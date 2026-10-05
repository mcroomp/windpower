"""Runtime decoding around LinkHub's generated protocol types."""

from __future__ import annotations

from dataclasses import MISSING, dataclass, fields
from typing import Any

from .generated_protocol import (
    ENUM_FIELDS,
    MESSAGE_TYPES,
    TUPLE_FIELDS,
    Attitude,
    AttitudeQuaternion,
    BatteryStatus,
    CommandAck,
    CommandLong,
    EkfStatusReport,
    ExtendedSysState,
    GlobalPositionInt,
    Heartbeat,
    LocalPositionNed,
    MavAutopilot,
    MavLandedState,
    MavParamType,
    MavResult,
    MavSeverity,
    MavState,
    MavType,
    MavVtolState,
    Message,
    NamedValueFloat,
    NamedValueInt,
    ParamRequestRead,
    ParamSet,
    ParamValue,
    PidAxis,
    PidTuning,
    RcChannels,
    RequestDataStream,
    ServoOutputRaw,
    SetAttitudeTarget,
    StatusText,
    SysStatus,
    WireIntEnum,
)


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
    tuple_fields = TUPLE_FIELDS.get(message_type, frozenset())
    for field in fields(message_type):
        if field.default is not MISSING:
            value = getattr(message, field.name, field.default)
        elif field.default_factory is not MISSING:  # type: ignore[misc]
            value = getattr(message, field.name, field.default_factory())
        else:
            value = getattr(message, field.name)
        if field.name in enum_fields:
            value = enum_fields[field.name](value)
        elif field.name in tuple_fields:
            value = tuple(value)
        values[field.name] = value
    if message_type is SetAttitudeTarget and "q" in values:
        values["q"] = list(values["q"]) if values["q"] else []
    return message_type(**values)


__all__ = [
    "Attitude",
    "AttitudeQuaternion",
    "BatteryStatus",
    "CommandAck",
    "CommandLong",
    "EkfStatusReport",
    "EscTelemetry",
    "ExtendedSysState",
    "GlobalPositionInt",
    "Heartbeat",
    "LocalPositionNed",
    "MavAutopilot",
    "MavLandedState",
    "MavParamType",
    "MavResult",
    "MavSeverity",
    "MavState",
    "MavType",
    "MavVtolState",
    "Message",
    "NamedValueFloat",
    "NamedValueInt",
    "ParamRequestRead",
    "ParamSet",
    "ParamValue",
    "PidAxis",
    "PidTuning",
    "RawMessage",
    "RcChannels",
    "RequestDataStream",
    "ServoOutputRaw",
    "SetAttitudeTarget",
    "StatusText",
    "SysStatus",
    "WireIntEnum",
    "decode_message",
]
