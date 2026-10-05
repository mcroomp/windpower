"""Generated from LinkHub's Rust protocol descriptor. Do not edit."""
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


class MavSeverity(WireIntEnum):
    EMERGENCY = 0
    ALERT = 1
    CRITICAL = 2
    ERROR = 3
    WARNING = 4
    NOTICE = 5
    INFO = 6
    DEBUG = 7

class MavType(WireIntEnum):
    HELICOPTER = 4
    GCS = 6

class MavAutopilot(WireIntEnum):
    ARDUPILOTMEGA = 3
    INVALID = 8

class MavState(WireIntEnum):
    STANDBY = 3
    ACTIVE = 4

class MavVtolState(WireIntEnum):
    UNDEFINED = 0
    TRANSITION_TO_FW = 1
    TRANSITION_TO_MC = 2
    MC = 3
    FW = 4

class MavLandedState(WireIntEnum):
    UNDEFINED = 0
    ON_GROUND = 1
    IN_AIR = 2
    TAKEOFF = 3
    LANDING = 4

class MavResult(WireIntEnum):
    ACCEPTED = 0
    TEMPORARILY_REJECTED = 1
    DENIED = 2
    UNSUPPORTED = 3
    FAILED = 4

class PidAxis(WireIntEnum):
    ROLL = 1
    PITCH = 2
    YAW = 3
    ACCEL_Z = 4

class MavParamType(WireIntEnum):
    INT8 = 2
    INT16 = 4
    INT32 = 6
    REAL32 = 9


@dataclass(frozen=True)
class Message:
    MAVLINK_TYPE: ClassVar[str]

    def get_type(self) -> str:
        return self.MAVLINK_TYPE

    def to_request(self) -> tuple[str, dict[str, Any]]:
        return self.MAVLINK_TYPE, asdict(self)


@dataclass(frozen=True)
class StatusText(Message):
    MAVLINK_TYPE: ClassVar[str] = "STATUSTEXT"
    text: str
    severity: MavSeverity = MavSeverity.INFO

@dataclass(frozen=True)
class Attitude(Message):
    MAVLINK_TYPE: ClassVar[str] = "ATTITUDE"
    roll: float
    pitch: float
    yaw: float
    rollspeed: float
    pitchspeed: float
    yawspeed: float
    time_boot_ms: int = 0

@dataclass(frozen=True)
class AttitudeQuaternion(Message):
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
class LocalPositionNed(Message):
    MAVLINK_TYPE: ClassVar[str] = "LOCAL_POSITION_NED"
    x: float
    y: float
    z: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    time_boot_ms: int = 0

@dataclass(frozen=True)
class GlobalPositionInt(Message):
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
class EkfStatusReport(Message):
    MAVLINK_TYPE: ClassVar[str] = "EKF_STATUS_REPORT"
    flags: int
    velocity_variance: float = 0.0
    pos_horiz_variance: float = 0.0
    pos_vert_variance: float = 0.0
    compass_variance: float = 0.0
    terrain_alt_variance: float = 0.0

@dataclass(frozen=True)
class BatteryStatus(Message):
    MAVLINK_TYPE: ClassVar[str] = "BATTERY_STATUS"
    current_battery: int = -1
    battery_remaining: int = -1
    voltages: tuple[int, ...] = ()

@dataclass(frozen=True)
class SysStatus(Message):
    MAVLINK_TYPE: ClassVar[str] = "SYS_STATUS"
    onboard_control_sensors_present: int = 0
    onboard_control_sensors_enabled: int = 0
    onboard_control_sensors_health: int = 0
    load: int = 0
    voltage_battery: int = 65535
    current_battery: int = -1
    battery_remaining: int = -1

@dataclass(frozen=True)
class Heartbeat(Message):
    MAVLINK_TYPE: ClassVar[str] = "HEARTBEAT"
    type: MavType
    autopilot: MavAutopilot
    base_mode: int
    custom_mode: int
    system_status: MavState
    mavlink_version: int = 3

@dataclass(frozen=True)
class ExtendedSysState(Message):
    MAVLINK_TYPE: ClassVar[str] = "EXTENDED_SYS_STATE"
    vtol_state: MavVtolState
    landed_state: MavLandedState

@dataclass(frozen=True)
class NamedValueFloat(Message):
    MAVLINK_TYPE: ClassVar[str] = "NAMED_VALUE_FLOAT"
    name: str
    value: float
    time_boot_ms: int = 0

@dataclass(frozen=True)
class NamedValueInt(Message):
    MAVLINK_TYPE: ClassVar[str] = "NAMED_VALUE_INT"
    name: str
    value: int
    time_boot_ms: int = 0

@dataclass(frozen=True)
class CommandAck(Message):
    MAVLINK_TYPE: ClassVar[str] = "COMMAND_ACK"
    command: int
    result: MavResult

@dataclass(frozen=True)
class CommandLong(Message):
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
class SetAttitudeTarget(Message):
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
class RcChannels(Message):
    MAVLINK_TYPE: ClassVar[str] = "RC_CHANNELS"
    chan1_raw: int | None = None
    chan2_raw: int | None = None
    chan3_raw: int | None = None
    chan4_raw: int | None = None

@dataclass(frozen=True)
class ServoOutputRaw(Message):
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
class PidTuning(Message):
    MAVLINK_TYPE: ClassVar[str] = "PID_TUNING"
    axis: PidAxis = PidAxis(-1)
    desired: float | None = None
    achieved: float | None = None
    FF: float | None = None
    P: float | None = None
    I: float | None = None
    D: float | None = None
    PDmod: float | None = None
    SRate: float | None = None

@dataclass(frozen=True)
class ParamSet(Message):
    MAVLINK_TYPE: ClassVar[str] = "PARAM_SET"
    target_system: int
    target_component: int
    param_id: str
    param_value: float
    param_type: MavParamType

@dataclass(frozen=True)
class ParamValue(Message):
    MAVLINK_TYPE: ClassVar[str] = "PARAM_VALUE"
    param_id: str
    param_value: float
    param_type: MavParamType = MavParamType(0)
    param_count: int = 0
    param_index: int = 0

@dataclass(frozen=True)
class ParamRequestRead(Message):
    MAVLINK_TYPE: ClassVar[str] = "PARAM_REQUEST_READ"
    target_system: int
    target_component: int
    param_id: str
    param_index: int = -1

@dataclass(frozen=True)
class RequestDataStream(Message):
    MAVLINK_TYPE: ClassVar[str] = "REQUEST_DATA_STREAM"
    target_system: int
    target_component: int
    req_stream_id: int
    req_message_rate: int
    start_stop: int = 1


MESSAGE_TYPES: dict[str, type[Message]] = {
    "STATUSTEXT": StatusText,
    "ATTITUDE": Attitude,
    "ATTITUDE_QUATERNION": AttitudeQuaternion,
    "LOCAL_POSITION_NED": LocalPositionNed,
    "GLOBAL_POSITION_INT": GlobalPositionInt,
    "EKF_STATUS_REPORT": EkfStatusReport,
    "BATTERY_STATUS": BatteryStatus,
    "SYS_STATUS": SysStatus,
    "HEARTBEAT": Heartbeat,
    "EXTENDED_SYS_STATE": ExtendedSysState,
    "NAMED_VALUE_FLOAT": NamedValueFloat,
    "NAMED_VALUE_INT": NamedValueInt,
    "COMMAND_ACK": CommandAck,
    "SET_ATTITUDE_TARGET": SetAttitudeTarget,
    "ATTITUDE_TARGET": SetAttitudeTarget,
    "RC_CHANNELS": RcChannels,
    "SERVO_OUTPUT_RAW": ServoOutputRaw,
    "PID_TUNING": PidTuning,
    "PARAM_VALUE": ParamValue,
}

ENUM_FIELDS: dict[type[Message], dict[str, type[WireIntEnum]]] = {
    StatusText: {
        "severity": MavSeverity,
    },
    Heartbeat: {
        "type": MavType,
        "autopilot": MavAutopilot,
        "system_status": MavState,
    },
    ExtendedSysState: {
        "vtol_state": MavVtolState,
        "landed_state": MavLandedState,
    },
    CommandAck: {
        "result": MavResult,
    },
    PidTuning: {
        "axis": PidAxis,
    },
    ParamValue: {
        "param_type": MavParamType,
    },
}

TUPLE_FIELDS: dict[type[Message], frozenset[str]] = {
    BatteryStatus: frozenset(["voltages"]),
}
