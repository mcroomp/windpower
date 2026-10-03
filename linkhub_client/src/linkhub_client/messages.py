"""Typed messages used by LinkHub HTTP clients."""

from __future__ import annotations

from dataclasses import MISSING, dataclass, fields
from typing import Any, ClassVar


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


class _Message:
    MAVLINK_TYPE: ClassVar[str]

    def get_type(self) -> str:
        return self.MAVLINK_TYPE


@dataclass(frozen=True)
class StatusText(_Message):
    MAVLINK_TYPE: ClassVar[str] = "STATUSTEXT"
    text: str
    severity: int = 0


@dataclass(frozen=True)
class Attitude(_Message):
    MAVLINK_TYPE: ClassVar[str] = "ATTITUDE"
    roll: float
    pitch: float
    yaw: float
    rollspeed: float
    pitchspeed: float
    yawspeed: float
    time_boot_ms: int = 0


@dataclass(frozen=True)
class AttitudeQuaternion(_Message):
    MAVLINK_TYPE: ClassVar[str] = "ATTITUDE_QUATERNION"
    q1: float
    q2: float
    q3: float
    q4: float
    rollspeed: float = 0.0
    pitchspeed: float = 0.0
    yawspeed: float = 0.0
    time_boot_ms: int = 0


@dataclass(frozen=True)
class LocalPositionNed(_Message):
    MAVLINK_TYPE: ClassVar[str] = "LOCAL_POSITION_NED"
    x: float
    y: float
    z: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    time_boot_ms: int = 0


@dataclass(frozen=True)
class GlobalPositionInt(_Message):
    MAVLINK_TYPE: ClassVar[str] = "GLOBAL_POSITION_INT"
    lat: int
    lon: int
    alt: int
    relative_alt: int
    vx: int = 0
    vy: int = 0
    vz: int = 0
    hdg: int = 0
    time_boot_ms: int = 0


@dataclass(frozen=True)
class EkfStatusReport(_Message):
    MAVLINK_TYPE: ClassVar[str] = "EKF_STATUS_REPORT"
    flags: int
    velocity_variance: float = 0.0
    pos_horiz_variance: float = 0.0
    pos_vert_variance: float = 0.0
    compass_variance: float = 0.0
    terrain_alt_variance: float = 0.0


@dataclass(frozen=True)
class BatteryStatus(_Message):
    MAVLINK_TYPE: ClassVar[str] = "BATTERY_STATUS"
    current_battery: int = -1
    battery_remaining: int = -1
    voltages: tuple[int, ...] = ()


@dataclass(frozen=True)
class SysStatus(_Message):
    MAVLINK_TYPE: ClassVar[str] = "SYS_STATUS"
    onboard_control_sensors_present: int = 0
    onboard_control_sensors_enabled: int = 0
    onboard_control_sensors_health: int = 0
    load: int = 0
    voltage_battery: int = 65535
    current_battery: int = -1
    battery_remaining: int = -1


@dataclass(frozen=True)
class Heartbeat(_Message):
    MAVLINK_TYPE: ClassVar[str] = "HEARTBEAT"
    type: int
    autopilot: int
    base_mode: int
    custom_mode: int
    system_status: int
    mavlink_version: int = 3


@dataclass(frozen=True)
class NamedValueFloat(_Message):
    MAVLINK_TYPE: ClassVar[str] = "NAMED_VALUE_FLOAT"
    name: str
    value: float
    time_boot_ms: int = 0

    def to_request(self) -> tuple[str, dict[str, Any]]:
        return self.MAVLINK_TYPE, {
            "name": self.name,
            "value": self.value,
            "time_boot_ms": self.time_boot_ms,
        }


@dataclass(frozen=True)
class NamedValueInt(_Message):
    MAVLINK_TYPE: ClassVar[str] = "NAMED_VALUE_INT"
    name: str
    value: int
    time_boot_ms: int = 0

    def to_request(self) -> tuple[str, dict[str, Any]]:
        return self.MAVLINK_TYPE, {
            "name": self.name,
            "value": self.value,
            "time_boot_ms": self.time_boot_ms,
        }


@dataclass(frozen=True)
class CommandAck(_Message):
    MAVLINK_TYPE: ClassVar[str] = "COMMAND_ACK"
    command: int
    result: int


@dataclass(frozen=True)
class CommandLong(_Message):
    MAVLINK_TYPE: ClassVar[str] = "COMMAND_LONG"
    target_system: int
    target_component: int
    command: int
    confirmation: int = 0
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    param5: float = 0.0
    param6: float = 0.0
    param7: float = 0.0


@dataclass(frozen=True)
class SetAttitudeTarget(_Message):
    MAVLINK_TYPE: ClassVar[str] = "SET_ATTITUDE_TARGET"
    target_system: int = 0
    target_component: int = 0
    type_mask: int = 0
    q: tuple[float, ...] | list[float] = ()
    body_roll_rate: float = 0.0
    body_pitch_rate: float = 0.0
    body_yaw_rate: float = 0.0
    thrust: float = 0.0
    time_boot_ms: int = 0


@dataclass(frozen=True)
class RcChannels(_Message):
    MAVLINK_TYPE: ClassVar[str] = "RC_CHANNELS"
    chan1_raw: int | None = None
    chan2_raw: int | None = None
    chan3_raw: int | None = None
    chan4_raw: int | None = None


@dataclass(frozen=True)
class ServoOutputRaw(_Message):
    MAVLINK_TYPE: ClassVar[str] = "SERVO_OUTPUT_RAW"
    servo1_raw: int = 0
    servo2_raw: int = 0
    servo3_raw: int = 0
    servo4_raw: int = 0
    servo5_raw: int = 0
    servo6_raw: int = 0
    servo7_raw: int = 0
    servo8_raw: int = 0
    servo9_raw: int = 0
    servo10_raw: int = 0
    servo11_raw: int = 0
    servo12_raw: int = 0
    servo13_raw: int = 0
    servo14_raw: int = 0
    servo15_raw: int = 0
    servo16_raw: int = 0
    port: int = 0
    time_usec: int = 0


@dataclass(frozen=True)
class PidTuning(_Message):
    MAVLINK_TYPE: ClassVar[str] = "PID_TUNING"
    axis: int = -1
    desired: float | None = None
    achieved: float | None = None
    FF: float | None = None
    P: float | None = None
    I: float | None = None
    D: float | None = None
    PDmod: float | None = None
    SRate: float | None = None


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


@dataclass(frozen=True)
class ParamSet(_Message):
    MAVLINK_TYPE: ClassVar[str] = "PARAM_SET"
    target_system: int
    target_component: int
    param_id: str
    param_value: float
    param_type: int


@dataclass(frozen=True)
class ParamValue(_Message):
    MAVLINK_TYPE: ClassVar[str] = "PARAM_VALUE"
    param_id: str
    param_value: float
    param_type: int = 0
    param_count: int = 0
    param_index: int = 0


@dataclass(frozen=True)
class ParamRequestRead(_Message):
    MAVLINK_TYPE: ClassVar[str] = "PARAM_REQUEST_READ"
    target_system: int
    target_component: int
    param_id: str
    param_index: int = -1


@dataclass(frozen=True)
class RequestDataStream(_Message):
    MAVLINK_TYPE: ClassVar[str] = "REQUEST_DATA_STREAM"
    target_system: int
    target_component: int
    req_stream_id: int
    req_message_rate: int
    start_stop: int = 1


_MESSAGE_TYPES = {
    message_type.MAVLINK_TYPE: message_type
    for message_type in (
        StatusText,
        Attitude,
        AttitudeQuaternion,
        LocalPositionNed,
        GlobalPositionInt,
        EkfStatusReport,
        BatteryStatus,
        SysStatus,
        Heartbeat,
        NamedValueFloat,
        NamedValueInt,
        CommandAck,
        SetAttitudeTarget,
        RcChannels,
        ServoOutputRaw,
        PidTuning,
        ParamValue,
    )
}
_MESSAGE_TYPES["ATTITUDE_TARGET"] = SetAttitudeTarget
_ESC_CHANNEL_BASE_BY_TYPE = {
    "ESC_TELEMETRY_1_TO_4": 1,
    "ESC_TELEMETRY_5_TO_8": 5,
    "ESC_TELEMETRY_9_TO_12": 9,
}


def decode_message(message: Any) -> Any:
    """Decode a raw wire message into one of the dataclasses above.

    Works on any object exposing ``get_type()`` plus per-field attribute
    access -- not only ``RawMessage`` -- so lightweight test doubles (see
    tests/unit/test_calibrate_passive_controls.py's ``_RawAttitudeQuaternion``)
    decode the same way real LinkHub records do.
    """
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
    message_type = _MESSAGE_TYPES.get(message_name)
    if message_type is None:
        return message
    values: dict[str, Any] = {}
    for field in fields(message_type):
        if field.default is not MISSING:
            value = getattr(message, field.name, field.default)
        elif field.default_factory is not MISSING:  # type: ignore[misc]
            value = getattr(message, field.name, field.default_factory())
        else:
            value = getattr(message, field.name)
        values[field.name] = value
    if isinstance(values.get("voltages"), list):
        values["voltages"] = tuple(values["voltages"])
    if message_type is SetAttitudeTarget and "q" in values:
        values["q"] = list(values["q"]) if values["q"] else []
    return message_type(**values)
