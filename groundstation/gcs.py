"""
gcs.py — MAVLink GCS client for RAWES simulation control.

Replaces a human Mission Planner operator for automated simulation scenarios.
Handles: connection, background heartbeat, parameter set, arm, mode change,
EKF health check, and GUIDED position targets.

Usage
-----
    gcs = RawesGCS()
    gcs.connect()
    gcs.start_heartbeat()
    gcs.set_param("ARMING_CHECK", 0)
    gcs.wait_ekf_ok()
    gcs.arm()
    gcs.set_mode(GUIDED)
    gcs.send_position_target_ned(north=0.0, east=35.4, down=14.6)
    pos = gcs.recv_local_position()
    gcs.close()
"""

import collections
import logging
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar, Protocol, TypeVar, cast

from pymavlink import mavutil

from groundstation.mavlink_log import MavlinkLogWriter

log = logging.getLogger(__name__)


class MavSenderLike(Protocol):
    def __getattr__(self, name: str) -> Any: ...

    def set_send_callback(self, callback) -> None: ...


class MavConnectionLike(Protocol):
    mav: MavSenderLike
    target_system: int
    target_component: int

    def recv_match(self, *args, **kwargs) -> Any: ...

    def wait_heartbeat(self, *args, **kwargs) -> Any: ...

    def close(self) -> None: ...

# ArduCopter custom mode numbers
STABILIZE    = 0
ACRO         = 1
ALT_HOLD     = 2
AUTO         = 3
GUIDED       = 4
LOITER       = 5
GUIDED_NOGPS = 20

# SET_POSITION_TARGET_LOCAL_NED type_mask: ignore everything except x, y, z
# (bits set = ignore that field)
_POS_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


class SimClock:
    """
    Simulation-time clock driven by ArduPilot MAVLink messages.

    **Sim time vs wall-clock time**

    ArduPilot SITL uses a lockstep physics protocol: the flight-controller
    binary sends a servo packet and then blocks until the physics backend
    replies with a state packet.  Because of this lockstep, ArduPilot's
    internal clock (``time_boot_ms``) advances only when the physics loop is
    running — it is *not* a real-time clock.

    - At SITL speedup=1 (default), sim time ≈ wall-clock time (roughly 1:1),
      but the two are never guaranteed to be equal.
    - Every received MAVLink message carries ``time_boot_ms``; this class
      advances monotonically from those timestamps so callers using
      ``sim_now()`` stay synchronised with ArduPilot's internal time rather
      than with wall-clock.
    - **All messages advance the clock — including discarded ones.**  When
      ``_recv(type="ATTITUDE")`` is waiting and a HEARTBEAT arrives first,
      the clock is updated from the HEARTBEAT's ``time_boot_ms`` before the
      HEARTBEAT is thrown away.  The type filter controls what is *returned*
      to the caller; it does not affect clock updates.

    No lock is needed: only the main test thread reads and writes this clock
    (the heartbeat thread only *sends* MAVLink messages, never receives them).
    """

    def __init__(self) -> None:
        self._t_ms: int = 0

    def update(self, time_boot_ms: int) -> None:
        """Advance the clock to *time_boot_ms*.

        Messages with time_boot_ms=0 are silently skipped (not all message
        types carry a meaningful timestamp).  Any non-zero timestamp that is
        less than the current clock value is a bug — messages must arrive in
        non-decreasing order.
        """
        if time_boot_ms == 0:
            return
        assert time_boot_ms >= self._t_ms, (
            f"sim clock went backward: {time_boot_ms} ms < {self._t_ms} ms"
        )
        self._t_ms = time_boot_ms

    def now(self) -> float:
        """
        Current simulation time in seconds (ArduPilot ``time_boot_ms / 1000``).

        Returns 0.0 before the first MAVLink message is received.
        """
        return self._t_ms / 1000.0

    def now_ms(self) -> int:
        """Current simulation time in milliseconds (raw ``time_boot_ms``)."""
        return self._t_ms


class WallClock:
    """
    Wall-clock drop-in for SimClock — for use outside SITL (e.g. hardware calibration).

    Uses ``time.monotonic()`` so all deadline arithmetic in RawesGCS works correctly
    on a real serial connection where ``time_boot_ms`` starts at an arbitrary uptime
    value rather than near zero.
    """

    def __init__(self) -> None:
        self._t0 = time.monotonic()

    def update(self, time_boot_ms: int) -> None:  # noqa: ARG002
        pass  # wall clock is self-advancing; ignore MAVLink timestamps

    def now(self) -> float:
        return time.monotonic() - self._t0

    def now_ms(self) -> int:
        return int((time.monotonic() - self._t0) * 1000)


# ==========================================================================
# MAVLink message dataclasses
# ==========================================================================
# One dataclass per MAVLink message this codebase sends and/or receives.
# Field names/types intentionally mirror the generated C++ struct ArduPilot
# itself uses (mavlink/include/mavlink/v2.0/common/mavlink_msg_<name>.h,
# e.g. `mavlink_attitude_t`) -- these are the same names the MAVLink XML
# defines, so they also match pymavlink's message attributes 1:1.  This is
# additive: existing call sites (RawesGCS methods, observe() callbacks) are
# unchanged for now and can adopt these incrementally.
#
# MAVLINK_TYPE mirrors msg.get_type() (e.g. "ATTITUDE") for future dispatch.
# decode(msg) parses a raw pymavlink message into the dataclass; send(mav)
# serializes the dataclass back out over a pymavlink connection.  Messages
# this codebase only ever receives get decode() only; messages it only ever
# sends get send() only.

def _decode_mavlink_name(msg) -> str:
    """Decode a 10/16-byte null-padded name field (bytes or str) into a str.

    Shared by NamedValueFloat/NamedValueInt (name, 10 bytes) and would also
    suit ParamValue/ParamSet (param_id, 16 bytes) if/when those adopt decode().
    """
    raw_name = getattr(msg, "name", "")
    name = (
        raw_name.decode("ascii", errors="ignore")
        if isinstance(raw_name, bytes) else str(raw_name)
    )
    return name.rstrip("\x00").strip()


@dataclass(frozen=True)
class StatusText:
    """MAVLink STATUSTEXT (#253) -- mirrors mavlink_statustext_t."""
    MAVLINK_TYPE: ClassVar[str] = "STATUSTEXT"

    text: str
    severity: int = 0  # MAV_SEVERITY

    @staticmethod
    def decode(msg) -> "StatusText":
        return StatusText(
            text=str(msg.text).rstrip("\x00").strip(),
            severity=int(getattr(msg, "severity", 0)),
        )


@dataclass(frozen=True)
class Attitude:
    """MAVLink ATTITUDE (#30) -- mirrors mavlink_attitude_t."""
    MAVLINK_TYPE: ClassVar[str] = "ATTITUDE"

    roll:       float  # rad
    pitch:      float  # rad
    yaw:        float  # rad
    rollspeed:  float  # rad/s
    pitchspeed: float  # rad/s
    yawspeed:   float  # rad/s
    time_boot_ms: int = 0

    @staticmethod
    def decode(msg) -> "Attitude":
        return Attitude(
            roll=float(msg.roll),
            pitch=float(msg.pitch),
            yaw=float(msg.yaw),
            rollspeed=float(msg.rollspeed),
            pitchspeed=float(msg.pitchspeed),
            yawspeed=float(msg.yawspeed),
            time_boot_ms=int(getattr(msg, "time_boot_ms", 0)),
        )


@dataclass(frozen=True)
class LocalPositionNed:
    """MAVLink LOCAL_POSITION_NED (#32) -- mirrors mavlink_local_position_ned_t."""
    MAVLINK_TYPE: ClassVar[str] = "LOCAL_POSITION_NED"

    x: float  # m, North
    y: float  # m, East
    z: float  # m, Down
    vx: float = 0.0  # m/s
    vy: float = 0.0  # m/s
    vz: float = 0.0  # m/s
    time_boot_ms: int = 0

    @staticmethod
    def decode(msg) -> "LocalPositionNed":
        return LocalPositionNed(
            x=float(msg.x), y=float(msg.y), z=float(msg.z),
            vx=float(getattr(msg, "vx", 0.0)),
            vy=float(getattr(msg, "vy", 0.0)),
            vz=float(getattr(msg, "vz", 0.0)),
            time_boot_ms=int(getattr(msg, "time_boot_ms", 0)),
        )


@dataclass(frozen=True)
class GlobalPositionInt:
    """MAVLink GLOBAL_POSITION_INT (#33) -- mirrors mavlink_global_position_int_t."""
    MAVLINK_TYPE: ClassVar[str] = "GLOBAL_POSITION_INT"

    lat:          int  # degE7
    lon:          int  # degE7
    alt:          int  # mm, AMSL
    relative_alt: int  # mm, above home
    vx:           int = 0  # cm/s
    vy:           int = 0  # cm/s
    vz:           int = 0  # cm/s
    hdg:          int = 0  # cdeg
    time_boot_ms: int = 0

    @staticmethod
    def decode(msg) -> "GlobalPositionInt":
        return GlobalPositionInt(
            lat=int(msg.lat), lon=int(msg.lon), alt=int(msg.alt),
            relative_alt=int(msg.relative_alt),
            vx=int(getattr(msg, "vx", 0)),
            vy=int(getattr(msg, "vy", 0)),
            vz=int(getattr(msg, "vz", 0)),
            hdg=int(getattr(msg, "hdg", 0)),
            time_boot_ms=int(getattr(msg, "time_boot_ms", 0)),
        )


@dataclass(frozen=True)
class EkfStatusReport:
    """MAVLink EKF_STATUS_REPORT (#193) -- mirrors mavlink_ekf_status_report_t."""
    MAVLINK_TYPE: ClassVar[str] = "EKF_STATUS_REPORT"

    flags:                 int    # EKF_STATUS_FLAGS bitmask
    velocity_variance:     float = 0.0
    pos_horiz_variance:    float = 0.0
    pos_vert_variance:     float = 0.0
    compass_variance:      float = 0.0
    terrain_alt_variance:  float = 0.0

    @staticmethod
    def decode(msg) -> "EkfStatusReport":
        return EkfStatusReport(
            flags=int(msg.flags),
            velocity_variance=float(getattr(msg, "velocity_variance", 0.0)),
            pos_horiz_variance=float(getattr(msg, "pos_horiz_variance", 0.0)),
            pos_vert_variance=float(getattr(msg, "pos_vert_variance", 0.0)),
            compass_variance=float(getattr(msg, "compass_variance", 0.0)),
            terrain_alt_variance=float(getattr(msg, "terrain_alt_variance", 0.0)),
        )


@dataclass(frozen=True)
class BatteryStatus:
    """MAVLink BATTERY_STATUS (#147) -- mirrors mavlink_battery_status_t."""
    MAVLINK_TYPE: ClassVar[str] = "BATTERY_STATUS"

    current_battery: int = -1
    battery_remaining: int = -1
    voltages: tuple[int, ...] = ()

    @staticmethod
    def decode(msg) -> "BatteryStatus":
        return BatteryStatus(
            current_battery=int(getattr(msg, "current_battery", -1)),
            battery_remaining=int(getattr(msg, "battery_remaining", -1)),
            voltages=tuple(int(v) for v in getattr(msg, "voltages", ())),
        )


@dataclass(frozen=True)
class SysStatus:
    """MAVLink SYS_STATUS (#1) -- mirrors mavlink_sys_status_t."""
    MAVLINK_TYPE: ClassVar[str] = "SYS_STATUS"

    onboard_control_sensors_present: int = 0
    onboard_control_sensors_enabled: int = 0
    onboard_control_sensors_health: int = 0
    load: int = 0
    voltage_battery: int = 65535
    current_battery: int = -1
    battery_remaining: int = -1

    @staticmethod
    def decode(msg) -> "SysStatus":
        return SysStatus(
            onboard_control_sensors_present=int(getattr(msg, "onboard_control_sensors_present", 0)),
            onboard_control_sensors_enabled=int(getattr(msg, "onboard_control_sensors_enabled", 0)),
            onboard_control_sensors_health=int(getattr(msg, "onboard_control_sensors_health", 0)),
            load=int(getattr(msg, "load", 0)),
            voltage_battery=int(getattr(msg, "voltage_battery", 65535)),
            current_battery=int(getattr(msg, "current_battery", -1)),
            battery_remaining=int(getattr(msg, "battery_remaining", -1)),
        )


@dataclass(frozen=True)
class GpsRawInt:
    """MAVLink GPS_RAW_INT (#24) -- mirrors mavlink_gps_raw_int_t."""
    MAVLINK_TYPE: ClassVar[str] = "GPS_RAW_INT"

    fix_type: int = 0
    satellites_visible: int = 0
    eph: int = 65535
    epv: int = 65535
    lat: int = 0
    lon: int = 0
    alt: int = 0

    @staticmethod
    def decode(msg) -> "GpsRawInt":
        return GpsRawInt(
            fix_type=int(getattr(msg, "fix_type", 0)),
            satellites_visible=int(getattr(msg, "satellites_visible", 0)),
            eph=int(getattr(msg, "eph", 65535)),
            epv=int(getattr(msg, "epv", 65535)),
            lat=int(getattr(msg, "lat", 0)),
            lon=int(getattr(msg, "lon", 0)),
            alt=int(getattr(msg, "alt", 0)),
        )


@dataclass(frozen=True)
class PowerStatus:
    """MAVLink POWER_STATUS (#125) -- mirrors mavlink_power_status_t."""
    MAVLINK_TYPE: ClassVar[str] = "POWER_STATUS"

    Vcc: int = 0
    Vservo: int = 0
    flags: int = 0

    @staticmethod
    def decode(msg) -> "PowerStatus":
        return PowerStatus(
            Vcc=int(getattr(msg, "Vcc", 0)),
            Vservo=int(getattr(msg, "Vservo", 0)),
            flags=int(getattr(msg, "flags", 0)),
        )


@dataclass(frozen=True)
class MemInfo:
    """MAVLink MEMINFO (#152) -- mirrors mavlink_meminfo_t."""
    MAVLINK_TYPE: ClassVar[str] = "MEMINFO"

    freemem: int = 0
    freemem32: int | None = None

    @staticmethod
    def decode(msg) -> "MemInfo":
        freemem32 = getattr(msg, "freemem32", None)
        return MemInfo(
            freemem=int(getattr(msg, "freemem", 0)),
            freemem32=int(freemem32) if freemem32 is not None else None,
        )


@dataclass(frozen=True)
class McuStatus:
    """MAVLink MCU_STATUS (#11039) -- mirrors mavlink_mcu_status_t."""
    MAVLINK_TYPE: ClassVar[str] = "MCU_STATUS"

    MCU_temperature: int = 0
    MCU_voltage: int = 0
    MCU_voltage_min: int = 0
    MCU_voltage_max: int = 0

    @staticmethod
    def decode(msg) -> "McuStatus":
        return McuStatus(
            MCU_temperature=int(getattr(msg, "MCU_temperature", 0)),
            MCU_voltage=int(getattr(msg, "MCU_voltage", 0)),
            MCU_voltage_min=int(getattr(msg, "MCU_voltage_min", 0)),
            MCU_voltage_max=int(getattr(msg, "MCU_voltage_max", 0)),
        )


@dataclass(frozen=True)
class ServoOutputRaw:
    """MAVLink SERVO_OUTPUT_RAW (#36) -- mirrors mavlink_servo_output_raw_t."""
    MAVLINK_TYPE: ClassVar[str] = "SERVO_OUTPUT_RAW"

    servo1_raw: int = 0
    servo2_raw: int = 0
    servo3_raw: int = 0
    servo4_raw: int = 0
    servo5_raw: int = 0
    servo6_raw: int = 0
    servo7_raw: int = 0
    servo8_raw: int = 0
    servo9_raw:  int = 0
    servo10_raw: int = 0
    servo11_raw: int = 0
    servo12_raw: int = 0
    servo13_raw: int = 0
    servo14_raw: int = 0
    servo15_raw: int = 0
    servo16_raw: int = 0
    port:       int = 0
    time_usec:  int = 0

    @staticmethod
    def decode(msg) -> "ServoOutputRaw":
        return ServoOutputRaw(**{
            f: int(getattr(msg, f, 0))
            for f in (
                "servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw",
                "servo5_raw", "servo6_raw", "servo7_raw", "servo8_raw",
                "servo9_raw", "servo10_raw", "servo11_raw", "servo12_raw",
                "servo13_raw", "servo14_raw", "servo15_raw", "servo16_raw",
                "port", "time_usec",
            )
        })


@dataclass(frozen=True)
class ParamValue:
    """MAVLink PARAM_VALUE (#22) -- mirrors mavlink_param_value_t."""
    MAVLINK_TYPE: ClassVar[str] = "PARAM_VALUE"

    param_id:    str
    param_value: float
    param_type:  int = 0  # MAV_PARAM_TYPE
    param_count: int = 0
    param_index: int = 0

    @staticmethod
    def decode(msg) -> "ParamValue":
        return ParamValue(
            param_id=str(msg.param_id).rstrip("\x00").strip(),
            param_value=float(msg.param_value),
            param_type=int(getattr(msg, "param_type", 0)),
            param_count=int(getattr(msg, "param_count", 0)),
            param_index=int(getattr(msg, "param_index", 0)),
        )


@dataclass(frozen=True)
class CommandAck:
    """MAVLink COMMAND_ACK (#77) -- mirrors mavlink_command_ack_t."""
    MAVLINK_TYPE: ClassVar[str] = "COMMAND_ACK"

    command: int  # MAV_CMD
    result:  int  # MAV_RESULT

    @staticmethod
    def decode(msg) -> "CommandAck":
        return CommandAck(command=int(msg.command), result=int(msg.result))


@dataclass(frozen=True)
class Heartbeat:
    """MAVLink HEARTBEAT (#0) -- mirrors mavlink_heartbeat_t."""
    MAVLINK_TYPE: ClassVar[str] = "HEARTBEAT"

    type:            int  # MAV_TYPE
    autopilot:       int  # MAV_AUTOPILOT
    base_mode:       int  # MAV_MODE_FLAG bitmask
    custom_mode:     int
    system_status:   int  # MAV_STATE
    mavlink_version: int = 3

    @staticmethod
    def decode(msg) -> "Heartbeat":
        return Heartbeat(
            type=int(msg.type),
            autopilot=int(msg.autopilot),
            base_mode=int(msg.base_mode),
            custom_mode=int(msg.custom_mode),
            system_status=int(msg.system_status),
            mavlink_version=int(getattr(msg, "mavlink_version", 3)),
        )

    def send(self, mav) -> None:
        mav.mav.heartbeat_send(
            self.type, self.autopilot, self.base_mode,
            self.custom_mode, self.system_status,
        )


@dataclass(frozen=True)
class NamedValueFloat:
    """MAVLink NAMED_VALUE_FLOAT (#251) -- mirrors mavlink_named_value_float_t."""
    MAVLINK_TYPE: ClassVar[str] = "NAMED_VALUE_FLOAT"

    name:  str
    value: float
    time_boot_ms: int = 0

    @staticmethod
    def decode(msg) -> "NamedValueFloat":
        return NamedValueFloat(
            name=_decode_mavlink_name(msg),
            value=float(getattr(msg, "value", float("nan"))),
            time_boot_ms=int(getattr(msg, "time_boot_ms", 0)),
        )

    def send(self, mav) -> None:
        name_b = self.name.encode("ascii")[:10].ljust(10, b"\x00")
        mav.mav.named_value_float_send(self.time_boot_ms, name_b, float(self.value))


@dataclass(frozen=True)
class NamedValueInt:
    """MAVLink NAMED_VALUE_INT (#252) -- mirrors mavlink_named_value_int_t."""
    MAVLINK_TYPE: ClassVar[str] = "NAMED_VALUE_INT"

    name:  str
    value: int
    time_boot_ms: int = 0

    @staticmethod
    def decode(msg) -> "NamedValueInt":
        return NamedValueInt(
            name=_decode_mavlink_name(msg),
            value=int(getattr(msg, "value", 0)),
            time_boot_ms=int(getattr(msg, "time_boot_ms", 0)),
        )

    def send(self, mav) -> None:
        name_b = self.name.encode("ascii")[:10].ljust(10, b"\x00")
        mav.mav.named_value_int_send(self.time_boot_ms, name_b, int(self.value))


@dataclass(frozen=True)
class CommandLong:
    """MAVLink COMMAND_LONG (#76) -- mirrors mavlink_command_long_t.

    Generic command envelope used by arm/disarm (MAV_CMD_COMPONENT_ARM_DISARM),
    set_mode (MAV_CMD_DO_SET_MODE), and set_message_interval
    (MAV_CMD_SET_MESSAGE_INTERVAL) -- the command field selects behaviour.
    """
    MAVLINK_TYPE: ClassVar[str] = "COMMAND_LONG"

    target_system:    int
    target_component: int
    command:          int  # MAV_CMD
    confirmation:     int = 0
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    param5: float = 0.0
    param6: float = 0.0
    param7: float = 0.0

    def send(self, mav) -> None:
        mav.mav.command_long_send(
            self.target_system, self.target_component,
            self.command, self.confirmation,
            self.param1, self.param2, self.param3,
            self.param4, self.param5, self.param6, self.param7,
        )


@dataclass(frozen=True)
class SetPositionTargetLocalNed:
    """MAVLink SET_POSITION_TARGET_LOCAL_NED (#84) -- mirrors
    mavlink_set_position_target_local_ned_t."""
    MAVLINK_TYPE: ClassVar[str] = "SET_POSITION_TARGET_LOCAL_NED"

    target_system:    int
    target_component: int
    coordinate_frame: int
    type_mask:        int
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    afx: float = 0.0
    afy: float = 0.0
    afz: float = 0.0
    yaw: float = 0.0
    yaw_rate: float = 0.0
    time_boot_ms: int = 0

    def send(self, mav) -> None:
        mav.mav.set_position_target_local_ned_send(
            self.time_boot_ms, self.target_system, self.target_component,
            self.coordinate_frame, self.type_mask,
            self.x, self.y, self.z,
            self.vx, self.vy, self.vz,
            self.afx, self.afy, self.afz,
            self.yaw, self.yaw_rate,
        )


@dataclass(frozen=True)
class SetAttitudeTarget:
    """MAVLink SET_ATTITUDE_TARGET (#82) -- mirrors
    mavlink_set_attitude_target_t."""
    MAVLINK_TYPE: ClassVar[str] = "SET_ATTITUDE_TARGET"

    target_system:    int
    target_component: int
    type_mask:        int
    q:                list[float]
    body_roll_rate:   float = 0.0
    body_pitch_rate:  float = 0.0
    body_yaw_rate:    float = 0.0
    thrust:           float = 0.0
    time_boot_ms:     int = 0

    def send(self, mav) -> None:
        mav.mav.set_attitude_target_send(
            self.time_boot_ms,
            self.target_system,
            self.target_component,
            self.type_mask,
            self.q,
            self.body_roll_rate,
            self.body_pitch_rate,
            self.body_yaw_rate,
            self.thrust,
        )

    @staticmethod
    def decode(msg) -> "SetAttitudeTarget":
        return SetAttitudeTarget(
            target_system=int(getattr(msg, "target_system", 0)),
            target_component=int(getattr(msg, "target_component", 0)),
            type_mask=int(getattr(msg, "type_mask", 0)),
            q=list(getattr(msg, "q", ()) or ()),
            body_roll_rate=float(getattr(msg, "body_roll_rate", 0.0)),
            body_pitch_rate=float(getattr(msg, "body_pitch_rate", 0.0)),
            body_yaw_rate=float(getattr(msg, "body_yaw_rate", 0.0)),
            thrust=float(getattr(msg, "thrust", 0.0)),
            time_boot_ms=int(getattr(msg, "time_boot_ms", 0)),
        )


@dataclass(frozen=True)
class RcChannels:
    """MAVLink RC_CHANNELS (#65) -- mirrors mavlink_rc_channels_t."""
    MAVLINK_TYPE: ClassVar[str] = "RC_CHANNELS"

    chan1_raw: int | None = None
    chan2_raw: int | None = None
    chan3_raw: int | None = None
    chan4_raw: int | None = None

    @staticmethod
    def decode(msg) -> "RcChannels":
        return RcChannels(
            chan1_raw=getattr(msg, "chan1_raw", None),
            chan2_raw=getattr(msg, "chan2_raw", None),
            chan3_raw=getattr(msg, "chan3_raw", None),
            chan4_raw=getattr(msg, "chan4_raw", None),
        )


@dataclass(frozen=True)
class PidTuning:
    """MAVLink PID_TUNING (#194) -- mirrors mavlink_pid_tuning_t."""
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

    @staticmethod
    def decode(msg) -> "PidTuning":
        return PidTuning(
            axis=int(getattr(msg, "axis", -1)),
            desired=getattr(msg, "desired", None),
            achieved=getattr(msg, "achieved", None),
            FF=getattr(msg, "FF", None),
            P=getattr(msg, "P", None),
            I=getattr(msg, "I", None),
            D=getattr(msg, "D", None),
            PDmod=getattr(msg, "PDmod", None),
            SRate=getattr(msg, "SRate", None),
        )


_ESC_CHANNEL_BASE_BY_TYPE = {
    "ESC_TELEMETRY_1_TO_4": 1,
    "ESC_TELEMETRY_5_TO_8": 5,
    "ESC_TELEMETRY_9_TO_12": 9,
}


@dataclass(frozen=True)
class EscTelemetry:
    """MAVLink ESC_TELEMETRY_* messages -- common decoded shape."""

    message_name: str
    first_channel: int
    rpm: tuple[int, ...] = ()
    voltage: tuple[int, ...] = ()
    current: tuple[int, ...] = ()
    temperature: tuple[int, ...] = ()

    @staticmethod
    def decode(msg) -> "EscTelemetry":
        name = msg.get_type()
        return EscTelemetry(
            message_name=name,
            first_channel=_ESC_CHANNEL_BASE_BY_TYPE.get(name, 0),
            rpm=tuple(int(v) for v in getattr(msg, "rpm", ())),
            voltage=tuple(int(v) for v in getattr(msg, "voltage", ())),
            current=tuple(int(v) for v in getattr(msg, "current", ())),
            temperature=tuple(int(v) for v in getattr(msg, "temperature", ())),
        )


@dataclass(frozen=True)
class ParamSet:
    """MAVLink PARAM_SET (#23) -- mirrors mavlink_param_set_t."""
    MAVLINK_TYPE: ClassVar[str] = "PARAM_SET"

    target_system:    int
    target_component: int
    param_id:         str
    param_value:      float
    param_type:       int  # MAV_PARAM_TYPE

    def send(self, mav) -> None:
        mav.mav.param_set_send(
            self.target_system, self.target_component,
            self.param_id.encode("utf-8"), float(self.param_value), self.param_type,
        )


@dataclass(frozen=True)
class ParamRequestRead:
    """MAVLink PARAM_REQUEST_READ (#20) -- mirrors mavlink_param_request_read_t."""
    MAVLINK_TYPE: ClassVar[str] = "PARAM_REQUEST_READ"

    target_system:    int
    target_component: int
    param_id:         str
    param_index:      int = -1

    def send(self, mav) -> None:
        mav.mav.param_request_read_send(
            self.target_system, self.target_component,
            self.param_id.encode("utf-8"), self.param_index,
        )


@dataclass(frozen=True)
class ParamRequestList:
    """MAVLink PARAM_REQUEST_LIST (#21) -- mirrors mavlink_param_request_list_t."""
    MAVLINK_TYPE: ClassVar[str] = "PARAM_REQUEST_LIST"

    target_system:    int
    target_component: int

    def send(self, mav) -> None:
        mav.mav.param_request_list_send(self.target_system, self.target_component)


@dataclass(frozen=True)
class RequestDataStream:
    """MAVLink REQUEST_DATA_STREAM (#66) -- mirrors mavlink_request_data_stream_t."""
    MAVLINK_TYPE: ClassVar[str] = "REQUEST_DATA_STREAM"

    target_system:      int
    target_component:   int
    req_stream_id:      int  # MAV_DATA_STREAM
    req_message_rate:   int  # Hz
    start_stop:         int = 1  # 1 = start, 0 = stop

    def send(self, mav) -> None:
        mav.mav.request_data_stream_send(
            self.target_system, self.target_component,
            self.req_stream_id, self.req_message_rate, self.start_stop,
        )


_DecodedMessageT = TypeVar("_DecodedMessageT", covariant=True)


class _DecodableMessageClass(Protocol[_DecodedMessageT]):
    MAVLINK_TYPE: str

    @staticmethod
    def decode(msg) -> _DecodedMessageT: ...

_MESSAGE_CLASS_BY_TYPE: dict[str, object] = {
    cls.MAVLINK_TYPE: cls
    for cls in (
        StatusText,
        Attitude,
        LocalPositionNed,
        GlobalPositionInt,
        EkfStatusReport,
        BatteryStatus,
        SysStatus,
        GpsRawInt,
        PowerStatus,
        MemInfo,
        McuStatus,
        RcChannels,
        ServoOutputRaw,
        ParamValue,
        CommandAck,
        Heartbeat,
        NamedValueFloat,
        NamedValueInt,
        SetAttitudeTarget,
        PidTuning,
    )
}
_MESSAGE_CLASS_BY_TYPE.update({name: EscTelemetry for name in _ESC_CHANNEL_BASE_BY_TYPE})


def decode_message(msg):
    """Decode a raw pymavlink message into a registered dataclass when possible.

    Returns the original object unchanged when this module has no dataclass
    wrapper for the message type yet.
    """
    cls = _MESSAGE_CLASS_BY_TYPE.get(msg.get_type())
    if cls is None:
        return msg
    return cast(_DecodableMessageClass[object], cls).decode(msg)


def decode_as(msg, message_cls: "_DecodableMessageClass[_DecodedMessageT]") -> "_DecodedMessageT":
    """Decode *msg* as *message_cls*, with a type check on msg.get_type()."""
    if msg.get_type() != message_cls.MAVLINK_TYPE:
        raise TypeError(
            f"Expected {message_cls.MAVLINK_TYPE}, got {msg.get_type()}"
        )
    return message_cls.decode(msg)


class RawesGCS:
    """
    Minimal MAVLink GCS client for RAWES SITL control.

    Parameters
    ----------
    address : str
        pymavlink connection string.  Default connects to the SITL MAVLink
        output port (udpin = we listen, SITL sends).
    source_system : int
        MAVLink system ID to use for outgoing messages (GCS = 255).
    baud : int
        Baud rate for serial connections (ignored for UDP).  Default 115200.
    clock : SimClock | WallClock | None
        Clock to use for all deadline arithmetic.  Pass a ``WallClock()``
        instance when connecting to real hardware over USB/serial; omit (or
        pass None) for SITL where the sim clock is driven by ``time_boot_ms``.
    """

    def __init__(
        self,
        address: str = "udpin:localhost:14550",
        source_system: int = 255,
        mavlog_path: "str | Path | None" = None,
        baud: int = 115200,
        clock: "SimClock | WallClock | None" = None,
        watchdog=None,
    ):
        self._address = address
        self._source_system = source_system
        self._baud = baud
        self._mav: MavConnectionLike | None = None
        self._target_system = 1
        self._target_component = 1
        self._watchdog = watchdog  # nullary callable; raises if process is dead
        self._hb_thread: threading.Thread | None = None
        self._hb_stop = threading.Event()
        self._sim_clock = clock if clock is not None else SimClock()
        # Internal receive buffer — messages drained from the network socket but
        # not yet returned to a caller.  Populated by _recv; popped in FIFO order.
        # Clock is advanced only when a message is popped, so sim_now() stays
        # consistent with the message that caused _recv to return.
        self._recv_buf: collections.deque = collections.deque()
        self._armed: bool = False
        # JSON message log — every received MAVLink message written as one line
        self._mavlog: MavlinkLogWriter | None = None
        if mavlog_path is not None:
            self._mavlog = MavlinkLogWriter.open(mavlog_path)

    def _register_send_logger(self) -> None:
        """Tap outgoing MAVLink so sent messages are logged alongside received.

        pymavlink invokes the send callback on every ``mav.<msg>_send`` call,
        giving us a single choke point for all TX regardless of which helper
        sent it.  The callback is a no-op while ``self._mavlog`` is None.
        """
        if self._mav is None:
            return
        try:
            self._mav.mav.set_send_callback(self._on_send)
        except Exception:
            pass

    def _on_send(self, msg, *args, **kwargs) -> None:
        if self._mavlog is not None:
            self._mavlog.write(msg, self._sim_clock.now_ms(), direction="tx")

    def start_mavlog(self, path: "str | Path") -> "MavlinkLogWriter":
        """Begin logging ALL MAVLink traffic (rx + tx) as NDJSON to *path*.

        Received messages are logged by ``_recv``; sent messages by the send
        callback.  Safe to call after ``connect()``.  Replaces any existing log.
        """
        if self._mavlog is not None:
            self._mavlog.close()
        self._mavlog = MavlinkLogWriter.open(path)
        self._register_send_logger()
        return self._mavlog

    def stop_mavlog(self) -> None:
        """Close the MAVLink log (if any) and stop logging traffic."""
        if self._mavlog is not None:
            self._mavlog.close()
            self._mavlog = None

    # ------------------------------------------------------------------
    # Simulation time
    # ------------------------------------------------------------------

    @property
    def is_armed(self) -> bool:
        """True when the last received HEARTBEAT had MAV_MODE_FLAG_SAFETY_ARMED set."""
        return self._armed

    def sim_now(self) -> float:
        """
        Current ArduPilot simulation time in seconds.

        Derived from the ``time_boot_ms`` field of the most recently received
        MAVLink message.  This is ArduPilot's internal clock — it advances in
        lockstep with the physics backend, not with wall-clock time.

        At SITL speedup=1 (default) sim time ≈ wall time, but they are never
        guaranteed equal.  Returns 0.0 before the first message is received.
        """
        return self._sim_clock.now()

    def sim_sleep(self, duration_s: float, check=None) -> None:
        """
        Block until *duration_s* of **simulation** time has elapsed.

        **How it works (lockstep-aware)**

        ``sim_sleep`` does *not* call ``time.sleep``.  Instead it pumps
        ``_recv(blocking=True, timeout=0.1)`` in a tight loop.  Each call
        waits up to 0.1 wall-clock seconds for the next MAVLink message;
        when one arrives its ``time_boot_ms`` advances the sim-clock.  The
        loop exits once the sim-clock has advanced by *duration_s*.

        Because ArduPilot SITL uses a lockstep physics protocol (the
        flight-controller binary blocks until the physics backend replies),
        the physics worker must keep running and replying while ``sim_sleep``
        is active — otherwise ArduPilot stalls and the sim-clock never
        advances.  ``sim_sleep`` itself is purely a *consumer* of MAVLink
        messages; it never sends physics state.

        All STATUSTEXT, EKF_STATUS_REPORT, GPS_RAW_INT, and other messages
        that arrive during the wait are consumed by ``_recv()`` and written
        to the MAVLink log.  This means tests can inspect ``gcs_log``
        (the JSON MAVLink log) after ``sim_sleep`` to check what happened.

        **Sim time vs wall time**

        At SITL speedup=1 (default), 1 sim-second takes approximately 1
        wall-clock second, so ``sim_sleep(70)`` takes ~70 s in real time.
        The two clocks are not guaranteed equal — use sim time for all
        test deadlines and assertions.

        **Lockstep anti-pattern**

        Do NOT use ``sim_now()`` as a rate-limiting clock inside a physics
        worker.  If the worker skips replying to servo packets in order to
        "wait for sim time to advance", ArduPilot will block waiting for a
        reply, ``time_boot_ms`` will stop advancing, ``sim_now()`` will
        never cross the threshold, and the test will deadlock.

        Parameters
        ----------
        duration_s : float
            Number of simulation seconds to wait.
        check : callable | None
            Zero-argument callable invoked after each ~0.1 s recv poll.
            Use it to detect process crashes or other failure conditions.
            Exceptions raised by *check* propagate immediately.
        """
        deadline = self._sim_clock.now() + duration_s
        while self._sim_clock.now() < deadline:
            self._recv(blocking=True, timeout=0.1)
            if check is not None:
                check()

    def _recv(
        self,
        type=None,
        blocking: bool = True,
        timeout: float = 1.0,
    ):
        """Receive the next matching MAVLink message, advancing the sim-clock.

        Drains all available bytes from the network socket into an internal
        deque, then pops messages from that deque one at a time.  For each
        popped message the clock is advanced and the message is logged; if it
        matches *type* it is returned immediately.  Any messages still in the
        deque are left for the next call, so ``sim_now()`` is always
        consistent with the message that caused this call to return.

        The *timeout* parameter is a **wall-clock** deadline (``time.monotonic``),
        not a sim-time deadline.  All test logic that should be tied to sim
        time should use ``sim_now()`` + ``sim_sleep()`` instead.

        Parameters
        ----------
        type : str | list[str] | None
            Message type(s) to accept.  None accepts any type.
        blocking : bool
            If True, keep reading until a match is found or *timeout* expires.
        timeout : float
            Maximum wall-clock seconds to wait (only used when blocking=True).
        """
        if self._mav is None:
            raise RuntimeError("RawesGCS is not connected")

        mav = self._mav
        type_set = None
        if type is not None:
            type_set = set(type) if isinstance(type, list) else {type}
        deadline = time.monotonic() + (timeout or 0.0)
        while True:
            # Drain all currently available messages from the network into the
            # internal buffer without blocking.
            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
                self._recv_buf.append(msg)

            # Process the buffer in arrival order.  Clock and log are updated
            # as each message is popped; unprocessed messages stay for next call.
            while self._recv_buf:
                msg = self._recv_buf.popleft()
                self._sim_clock.update(getattr(msg, 'time_boot_ms', 0))
                if self._mavlog is not None:
                    self._mavlog.write(msg, self._sim_clock.now_ms())
                if msg.get_type() == "HEARTBEAT":
                    self._armed = bool(msg.base_mode & 128)  # MAV_MODE_FLAG_SAFETY_ARMED
                    if hasattr(msg, '_header'):
                        self._target_system    = msg._header.srcSystem
                        self._target_component = msg._header.srcComponent
                if type_set is None or msg.get_type() in type_set:
                    return msg
                # Non-matching: clock updated + logged; discard and continue.

            # Buffer exhausted.
            if not blocking or time.monotonic() >= deadline:
                return None
            time.sleep(0.002)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect_nowait(self) -> None:
        """Open the MAVLink socket without blocking for a heartbeat.

        Use this in single-threaded lockstep tests where the main loop must
        drive SITL sensor packets before ArduPilot can send any MAVLink.
        Target system/component default to 1/1 and are corrected automatically
        when the first HEARTBEAT is processed by _recv().
        """
        self._mav = cast(MavConnectionLike, mavutil.mavlink_connection(
            self._address,
            baud=self._baud,
            source_system=self._source_system,
        ))
        self._register_send_logger()
        log.info("GCS socket open (no heartbeat wait) — target sys=%d comp=%d",
                 self._target_system, self._target_component)

    def connect(self, timeout: float = 30.0, watchdog=None) -> None:
        """Connect and wait for the first heartbeat from the vehicle.

        Uses wall-clock time: no MAVLink messages have been received yet,
        so the sim-clock is unavailable.

        Each attempt tries for up to 1 s.  After every failed attempt the
        watchdog is called — if the process is already dead we get an
        immediate failure rather than waiting out the full timeout.

        The watchdog defaults to ``self._watchdog`` set at construction time
        (via the ``watchdog=`` argument to ``__init__``).  Passing a different
        callable here overrides it for this call only.

        Args:
            timeout:  wall-clock seconds to keep retrying before raising TimeoutError.
            watchdog: liveness callable; overrides the instance watchdog for this
                      call.  Pass ``None`` to suppress liveness checks entirely.
        """
        _wd = watchdog if watchdog is not None else self._watchdog
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            # Open the TCP connection.  ECONNREFUSED = SITL not up yet; retry.
            # ECONNTIMEDOUT can happen in Docker when no process holds the port;
            # it is caught the same way.
            try:
                self._mav = cast(MavConnectionLike, mavutil.mavlink_connection(
                    self._address,
                    baud=self._baud,
                    source_system=self._source_system,
                ))
            except Exception as exc:
                log.debug("Connect attempt failed: %s", exc)
                if _wd is not None:
                    _wd()
                time.sleep(0.5)
                continue
            self._register_send_logger()

            # Connection open — poll for the first heartbeat in 0.5 s chunks,
            # calling the watchdog between each chunk so a crashed process is
            # detected within ~0.5 s rather than after the full timeout.
            hb_deadline = min(deadline, time.monotonic() + 5.0)
            while time.monotonic() < hb_deadline:
                if _wd is not None:
                    _wd()
                try:
                    hb = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
                except Exception as exc:
                    log.debug("Heartbeat recv failed: %s", exc)
                    hb = None
                if hb is not None:
                    self._target_system    = self._mav.target_system
                    self._target_component = self._mav.target_component
                    log.info(
                        "GCS connected — vehicle sys=%d comp=%d",
                        self._target_system, self._target_component,
                    )
                    return

            # No heartbeat within 5 s on this connection — close and retry.
            try:
                self._mav.close()
            except Exception:
                pass
            self._mav = None

        raise TimeoutError(
            f"Could not connect to vehicle at {self._address!r} within {timeout:.0f}s"
        )

    def close(self) -> None:
        """Stop heartbeat thread and close socket."""
        self.stop_heartbeat()
        if self._mav is not None:
            try:
                self._mav.close()
            except Exception:
                pass
            self._mav = None
        if self._mavlog is not None:
            self._mavlog.close()
            self._mavlog = None

    # ------------------------------------------------------------------
    # Heartbeat
    # ------------------------------------------------------------------

    def start_heartbeat(self, rate_hz: float = 1.0) -> None:
        """
        Start a background thread that sends GCS heartbeats.

        SITL requires periodic GCS heartbeats to stay in non-failsafe states.
        Must be called after connect().
        """
        self._hb_stop.clear()
        self._hb_thread = threading.Thread(
            target=self._heartbeat_worker,
            args=(rate_hz,),
            daemon=True,
            name="gcs-heartbeat",
        )
        self._hb_thread.start()
        log.debug("GCS heartbeat thread started at %.1f Hz", rate_hz)

    def stop_heartbeat(self) -> None:
        self._hb_stop.set()
        if self._hb_thread is not None:
            self._hb_thread.join(timeout=2.0)
            self._hb_thread = None

    def _heartbeat_worker(self, rate_hz: float) -> None:
        interval = 1.0 / rate_hz
        while not self._hb_stop.wait(interval):
            try:
                self.send_message(Heartbeat(
                    type=mavutil.mavlink.MAV_TYPE_GCS,
                    autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    base_mode=0,
                    custom_mode=0,
                    system_status=0,
                ))
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def set_param(
        self,
        name: str,
        value: "int | float",
        timeout: float = 5.0,
        retries: int = 3,
    ) -> bool:
        """
        Set a parameter and confirm via PARAM_VALUE acknowledgement.

        Pass an ``int`` value to send MAV_PARAM_TYPE_INT32 (required for bitmask
        and enum params such as SERVO_BLH_MASK).  Pass a ``float`` for AP_Float
        params.  ArduPilot silently ignores a REAL32 set on an INT32 param.

        If no ACK arrives within *timeout* seconds, verifies the current value
        with a PARAM_REQUEST_READ and retries the set up to *retries* times.
        Returns True if the parameter is confirmed at the requested value.
        """
        param_type = (
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
            if isinstance(value, int)
            else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

        for attempt in range(retries):
            if attempt > 0:
                log.debug("set_param %s retry %d/%d", name, attempt, retries - 1)

            self.send_message(ParamSet(
                target_system=self._target_system,
                target_component=self._target_component,
                param_id=name,
                param_value=float(value),
                param_type=param_type,
            ))

            deadline = self.sim_now() + timeout
            while self.sim_now() < deadline:
                msg = self._recv(
                    type="PARAM_VALUE", blocking=True, timeout=1.0
                )
                if msg is None:
                    continue
                pv = ParamValue.decode(msg)
                pid = pv.param_id
                log.debug(
                    "PARAM_VALUE received: %s = %g (looking for %s)",
                    pid, pv.param_value, name,
                )
                if pid == name:
                    log.info("Param %-20s = %g", name, pv.param_value)
                    return True

            # ACK not received — read back to check if the value was applied silently
            log.debug("No ACK for %s — verifying via PARAM_REQUEST_READ", name)
            self.send_message(ParamRequestRead(
                target_system=self._target_system,
                target_component=self._target_component,
                param_id=name,
                param_index=-1,
            ))
            verify_deadline = self.sim_now() + 2.0
            while self.sim_now() < verify_deadline:
                msg = self._recv(
                    type="PARAM_VALUE", blocking=True, timeout=0.5
                )
                if msg is None:
                    continue
                pv = ParamValue.decode(msg)
                pid = pv.param_id
                if pid == name:
                    if pv.param_value == float(value):
                        log.info(
                            "Param %-20s = %g (set confirmed via readback)",
                            name, pv.param_value,
                        )
                        return True
                    log.debug(
                        "Param %s readback = %g (wanted %g) — will retry set",
                        name, pv.param_value, value,
                    )
                    break

        log.warning("Failed to set %s = %g after %d attempts", name, value, retries)
        return False

    def get_param(self, name: str, timeout: float = 5.0) -> float | None:
        """
        Read a parameter value via PARAM_REQUEST_READ.

        Returns the float value, or None if no response within *timeout*.
        """
        self.send_message(ParamRequestRead(
            target_system=self._target_system,
            target_component=self._target_component,
            param_id=name,
            param_index=-1,
        ))
        deadline = self.sim_now() + timeout
        while self.sim_now() < deadline:
            msg = self._recv(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                continue
            pv = ParamValue.decode(msg)
            pid = pv.param_id
            if pid == name:
                log.debug("get_param %s = %g", name, pv.param_value)
                return float(pv.param_value)
        log.warning("get_param %s: no response within %.1f s", name, timeout)
        return None

    def fetch_all_params(self, timeout: float = 30.0) -> "dict[str, float]":
        """Fetch all parameters via PARAM_REQUEST_LIST.

        Returns a dict mapping param name to value.  Completes when the full
        param_count is received or *timeout* expires (whichever comes first).
        """
        self.send_message(ParamRequestList(
            target_system=self._target_system,
            target_component=self._target_component,
        ))
        params: "dict[str, float]" = {}
        total: "int | None" = None
        deadline = self.sim_now() + timeout
        while self.sim_now() < deadline:
            msg = self._recv(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                if total is not None and len(params) >= total:
                    break
                continue
            pv = ParamValue.decode(msg)
            pid = pv.param_id
            params[pid] = float(pv.param_value)
            if total is None:
                total = pv.param_count
            if len(params) >= total:
                break
        log.debug("fetch_all_params: got %d/%s params", len(params), total)
        return params

    # ------------------------------------------------------------------
    # EKF health
    # ------------------------------------------------------------------

    def wait_ekf_attitude(self, timeout: float = 45.0) -> bool:
        """
        Block until a finite ATTITUDE message is received (EKF tilt aligned).

        This is a weaker condition than wait_ekf_ok() — it only requires
        attitude, not position.  Sufficient for force-arm in simulation.

        Returns True on success, False on timeout.
        """
        deadline = self.sim_now() + timeout
        while self.sim_now() < deadline:
            msg = self._recv(
                type=["ATTITUDE", "STATUSTEXT", "EKF_STATUS_REPORT"],
                blocking=True, timeout=1.0,
            )
            if msg is None:
                continue
            match decode_message(msg):
                case StatusText(text=text):
                    log.info("[t=%.1f] EKF wait STATUSTEXT: %s", self.sim_now(), text)
                case EkfStatusReport(flags=flags):
                    log.debug("EKF_STATUS flags=0x%04x", flags)
                case Attitude() as att:
                    r = math.degrees(att.roll)
                    p = math.degrees(att.pitch)
                    y = math.degrees(att.yaw)
                    if all(math.isfinite(v) for v in (r, p, y)):
                        log.info("[t=%.1f] EKF attitude ready rpy=(%.1f, %.1f, %.1f)deg",
                                 self.sim_now(), r, p, y)
                        return True
        return False

    def wait_ekf_ok(self, timeout: float = 60.0) -> None:
        """
        Block until the EKF reports a healthy position and velocity estimate.

        EKF_STATUS_REPORT flags checked:
          attitude, velocity_horiz, velocity_vert,
          pos_horiz_rel, pos_horiz_abs, pos_vert_abs
        """
        NEEDED = (
            mavutil.mavlink.EKF_ATTITUDE
            | mavutil.mavlink.EKF_VELOCITY_HORIZ
            | mavutil.mavlink.EKF_VELOCITY_VERT
            | mavutil.mavlink.EKF_POS_HORIZ_REL
            | mavutil.mavlink.EKF_POS_HORIZ_ABS
            | mavutil.mavlink.EKF_POS_VERT_ABS
        )
        deadline = self.sim_now() + timeout
        while self.sim_now() < deadline:
            msg = self._recv(
                type="EKF_STATUS_REPORT", blocking=True, timeout=2.0
            )
            if msg is None:
                continue
            ekf = EkfStatusReport.decode(msg)
            if (ekf.flags & NEEDED) == NEEDED:
                log.info("[t=%.1f] EKF healthy (flags=0x%04x)", self.sim_now(), ekf.flags)
                return
            log.debug("EKF not ready yet (flags=0x%04x)", ekf.flags)
        raise TimeoutError(f"EKF not healthy after {timeout:.0f}s")

    # ------------------------------------------------------------------
    # Arm
    # ------------------------------------------------------------------

    def arm(
        self,
        timeout: float = 15.0,
        force: bool = False,
    ) -> None:
        """Send arm command and confirm via HEARTBEAT armed flag.

        Parameters
        ----------
        force : bool
            Pass the ArduPilot force-arm magic number (21196) as param2 to
            bypass all remaining pre-arm safety checks.  Use in simulation
            when hardware-specific interlocks (motor interlock, RC failsafe)
            are irrelevant.
        The stack uses GUIDED setpoint APIs for control. RC channel overrides
        are intentionally not part of arm sequencing.
        """
        param2 = 21196.0 if force else 0.0
        log.info("Sending arm command (force=%s) …", force)
        self.send_message(CommandLong(
            target_system=self._target_system,
            target_component=self._target_component,
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=1,
            param2=param2,
        ))
        deadline        = self.sim_now() + timeout
        t_last_arm_send = self.sim_now()
        _poll = 0.5
        while self.sim_now() < deadline:
            msg = self._recv(
                type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT", "ATTITUDE"],
                blocking=True, timeout=max(_poll, 0.005),
            )
            if msg is None:
                continue
            match decode_message(msg):
                case StatusText(text=text):
                    log.warning("[t=%.1f] STATUSTEXT during arm: %s", self.sim_now(), text)
                    continue
                case CommandAck(command=command, result=result):
                    if command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            log.info("[t=%.1f] Arm command ACCEPTED — waiting for armed heartbeat ...",
                                     self.sim_now())
                        elif result in (
                            mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
                            mavutil.mavlink.MAV_RESULT_FAILED,
                        ):
                            log.info("[t=%.1f] Arm rejected (result=%d) — retrying",
                                     self.sim_now(), result)
                            self.sim_sleep(1.0)
                            self.send_message(CommandLong(
                                target_system=self._target_system,
                                target_component=self._target_component,
                                command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                confirmation=0,
                                param1=1,
                                param2=param2,
                            ))
                            t_last_arm_send = self.sim_now()
                        else:
                            raise RuntimeError(f"Arm rejected by vehicle (result={result})")
                    elif command != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                        log.debug("COMMAND_ACK for cmd=%d result=%d (not arm)", command, result)
                case Heartbeat() as hb:
                    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    log.info("[t=%.1f] HEARTBEAT: sysid=%d base_mode=0x%02x armed=%s custom_mode=%d",
                             self.sim_now(), msg.get_srcSystem(), hb.base_mode, armed, hb.custom_mode)
                    if armed:
                        log.info("[t=%.1f] Vehicle is armed.", self.sim_now())
                        return

        raise TimeoutError(f"Vehicle did not confirm armed within {timeout:.0f}s")

    def disarm(self, timeout: float = 15.0, force: bool = True) -> None:
        """Send disarm command and confirm via HEARTBEAT not-armed flag.

        Parameters
        ----------
        force : bool
            Pass the ArduPilot force-disarm magic number (21196) as param2 to
            bypass the "vehicle is flying / motors running" refusal.  Default
            True so a mid-run disarm succeeds in simulation.
        """
        param2 = 21196.0 if force else 0.0
        log.info("Sending disarm command (force=%s) …", force)
        self.send_message(CommandLong(
            target_system=self._target_system,
            target_component=self._target_component,
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=0,
            param2=param2,
        ))
        deadline = self.sim_now() + timeout
        while self.sim_now() < deadline:
            msg = self._recv(
                type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"],
                blocking=True, timeout=0.5,
            )
            if msg is None:
                continue
            match decode_message(msg):
                case StatusText(text=text):
                    log.info("[t=%.1f] STATUSTEXT during disarm: %s", self.sim_now(), text)
                    continue
                case Heartbeat(base_mode=base_mode):
                    if not bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                        log.info("[t=%.1f] Vehicle is disarmed.", self.sim_now())
                        return
        raise TimeoutError(f"Vehicle did not confirm disarmed within {timeout:.0f}s")

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------

    def set_mode(
        self,
        mode_id: int,
        timeout: float = 10.0,
    ) -> None:
        """Set ArduCopter flight mode by custom mode number.

        The stack uses GUIDED setpoint APIs for control. RC channel overrides
        are intentionally not part of mode switching.
        """
        log.info("Setting mode %d …", mode_id)
        t_last_send     = self.sim_now()
        self.send_message(CommandLong(
            target_system=self._target_system,
            target_component=self._target_component,
            command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            confirmation=0,
            param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=float(mode_id),
        ))
        deadline = self.sim_now() + timeout
        _poll = 0.5
        while self.sim_now() < deadline:
            msg = self._recv(
                type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT", "ATTITUDE"],
                blocking=True,
                timeout=max(_poll, 0.005),
            )
            if msg is None:
                continue
            match decode_message(msg):
                case Heartbeat(custom_mode=custom_mode):
                    if custom_mode == mode_id:
                        log.info("[t=%.1f] Mode confirmed: %d", self.sim_now(), mode_id)
                        return
                    log.debug("Heartbeat custom_mode=%d (waiting for %d)", custom_mode, mode_id)
                case CommandAck(command=command, result=result) if command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        log.debug("MAV_CMD_DO_SET_MODE accepted, waiting for heartbeat confirmation")
                    elif result == mavutil.mavlink.MAV_RESULT_FAILED:
                        log.debug("Mode %d rejected (result=%d) — retrying", mode_id, result)
                        self.sim_sleep(1.0)
                        self.send_message(CommandLong(
                            target_system=self._target_system,
                            target_component=self._target_component,
                            command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                            confirmation=0,
                            param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            param2=float(mode_id),
                        ))
                        t_last_send = self.sim_now()
                    else:
                        raise RuntimeError(
                            f"Mode change rejected by vehicle (mode={mode_id}, result={result})"
                        )
                case StatusText(text=text):
                    log.warning("[t=%.1f] STATUSTEXT during mode set: %s", self.sim_now(), text)
        raise TimeoutError(f"Mode {mode_id} not confirmed within {timeout:.0f}s")

    # ------------------------------------------------------------------
    # GUIDED position target
    # ------------------------------------------------------------------

    def send_position_target_ned(
        self,
        north: float,
        east:  float,
        down:  float,
        yaw:   float = 0.0,
    ) -> None:
        """
        Send SET_POSITION_TARGET_LOCAL_NED in MAV_FRAME_LOCAL_NED.

        Coordinate origin: EKF home (where SITL was launched).
        Positive down = below home altitude.

        Parameters
        ----------
        north, east, down : float   Target position [m]
        yaw               : float   Target yaw [rad], default 0 (ignored in mask)
        """
        self.send_message(SetPositionTargetLocalNed(
            target_system=self._target_system,
            target_component=self._target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask=_POS_ONLY_MASK,
            x=north,
            y=east,
            z=down,
            yaw=yaw,
            yaw_rate=0.0,
            time_boot_ms=0,
        ))
        log.info(
            "Position target sent: N=%.1f E=%.1f D=%.1f m", north, east, down
        )

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def request_stream(self, stream_id: int, rate_hz: int) -> None:
        """
        Request a MAVLink data stream from the vehicle.

        Parameters
        ----------
        stream_id : int
            MAV_DATA_STREAM_* constant (e.g. mavutil.mavlink.MAV_DATA_STREAM_POSITION)
        rate_hz : int
            Requested message rate in Hz.  Call once; ArduPilot sustains the rate.
        """
        self.send_message(RequestDataStream(
            target_system=self._target_system,
            target_component=self._target_component,
            req_stream_id=stream_id,
            req_message_rate=rate_hz,
            start_stop=1,
        ))
        log.debug("Requested stream id=%d at %d Hz", stream_id, rate_hz)

    def set_message_interval(self, msg_id: int, interval_us: int) -> None:
        """
        Set the send interval for a single MAVLink message (MAV_CMD_SET_MESSAGE_INTERVAL).

        Overrides the stream-rate grouping for one specific message id, so an
        unwanted high-rate message can be dropped while others in the same
        stream keep flowing.

        Parameters
        ----------
        msg_id : int
            MAVLink message id (e.g. mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS).
        interval_us : int
            Interval between messages in microseconds.  ``-1`` disables the
            message; ``0`` restores the stream/param default rate.
        """
        self.send_message(CommandLong(
            target_system=self._target_system,
            target_component=self._target_component,
            command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            confirmation=0,
            param1=float(msg_id),
            param2=float(interval_us),
        ))
        log.debug("SET_MESSAGE_INTERVAL id=%d interval_us=%d", msg_id, interval_us)

    def send_rc_override(self, channels: dict[int, int]) -> None:
        """
        RC overrides are disabled in the stack.

        Policy: flight control is GUIDED-only, and the only permitted override
        is Lua-managed CH8 interlock in rawes.lua.
        """
        raise RuntimeError(
            "RC_CHANNELS_OVERRIDE is disabled. Use GUIDED setpoints; only "
            "rawes.lua may hold CH8 override."
        )

    def send_message(self, msg) -> None:
        """Send any MAVLink message dataclass (NamedValueFloat, NamedValueInt,
        Heartbeat, CommandLong, ...) that defines a `send(mav)` method.

        Replaces the old per-message send_named_float()/send_named_int()
        wrappers -- construct the dataclass and pass it here instead:
            gcs.send_message(NamedValueFloat("RAWES_THR", 0.5))
            gcs.send_message(NamedValueInt("RAWES_LAT", lat_e7))
        """
        msg.send(self._mav)
        log.debug("%s sent: %r", type(msg).__name__, msg)

    def recv_decoded(
        self,
        message_cls: "_DecodableMessageClass[_DecodedMessageT]",
        *,
        blocking: bool = True,
        timeout: float = 1.0,
    ) -> "_DecodedMessageT | None":
        """Receive one message of *message_cls* and decode it immediately."""
        msg = self._recv(
            type=message_cls.MAVLINK_TYPE,
            blocking=blocking,
            timeout=timeout,
        )
        if msg is None:
            return None
        return message_cls.decode(msg)


    # ------------------------------------------------------------------
    # Telemetry receive
    # ------------------------------------------------------------------

    def recv_local_position(
        self, timeout: float = 2.0
    ) -> tuple[float, float, float] | None:
        """
        Return current LOCAL_POSITION_NED (x=N, y=E, z=D) or None on timeout.
        """
        msg = self._recv(
            type="LOCAL_POSITION_NED", blocking=True, timeout=timeout
        )
        if msg:
            pos = LocalPositionNed.decode(msg)
            return (pos.x, pos.y, pos.z)
        return None

    def recv_local_position_latest(self) -> tuple[float, float, float] | None:
        """
        Non-blocking poll: drain buffered LOCAL_POSITION_NED messages and
        return the most recent one, or None if none are buffered.
        Used in the 10 Hz ground loop to read the downlinked hub altitude.
        """
        latest = None
        while True:
            msg = self._recv(type="LOCAL_POSITION_NED", blocking=False)
            if msg is None:
                break
            latest = LocalPositionNed.decode(msg)
        if latest is not None:
            return (latest.x, latest.y, latest.z)
        return None

    def recv_attitude(
        self, timeout: float = 2.0
    ) -> tuple[float, float, float] | None:
        """Return (roll, pitch, yaw) radians or None on timeout."""
        msg = self._recv(
            type="ATTITUDE", blocking=True, timeout=timeout
        )
        if msg:
            att = Attitude.decode(msg)
            return (att.roll, att.pitch, att.yaw)
        return None
