"""
calibrate/run.py -- Observation loop engine, run command.
"""
from __future__ import annotations

import csv
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from linkhub_client.client import (
    LinkHubError,
    LinkHubMotorController,
)
from linkhub_client.messages import (
    AttitudeTarget,
    MavCmd,
    MavDataStream,
    MavModeFlag,
    MavState,
    PidTuningAxis,
    decode_message,
)
from groundstation.rawes_diag import diag_values
from groundstation.rawes_modes import (
    CMD_ENTER_GUIDED,
    CMD_ENTER_PASSIVE,
    enter_passive_params,
    send_rawes_command,
)

# msvcrt is Windows stdlib -- used for non-blocking ESC-key abort.
# Falls back to a stub on non-Windows so the rest of the module still imports.
try:
    import msvcrt
except ImportError:
    class _MsvcrtStub:
        def kbhit(self):    return False
        def getch(self):    return b""
    msvcrt = _MsvcrtStub()  # type: ignore[assignment]

from .constants import (
    Attitude,
    AttitudeQuaternion,
    ExtendedSysState,
    Heartbeat,
    LocalPositionNed,
    MavLandedState,
    EscTelemetry,
    PidTuning,
    RcChannels,
    LinkHubClient,
    NamedValueFloat,
    CommandLong,
    DebugFloatArray,
    Statustext,
    SERVO_MOTOR, MOTOR_OFF_US, MOTOR_ESC_CHANNEL,
    _ESC_TELEM_MSGS,
    _RUN_MODES, _IC_TRIM_KEYS, _PASSIVE_IC_THRUST,
    _COPTER_MODES,
)
from .hw import (
    _arm, _disarm, _send_set_servo, _set_safe_off_state,
    _esc_telem_msg_for_channel, _esc_erpm, _rpm_triplet,
)
from .messages import read_one
from .util import (
    _fmt, _log_path, _parse_kv_list, _parse_flags,
    _RunLog, _esc_check, _poll_keys,
)

# Diagnostic-array key -> observation-state slot (None: not displayed).
_DIAG_STATE_KEYS = {
    "YFF_T": "yff_t", "YFF_U": "yff_u", "YFF_GZ": "yff_gz",
    "OL_RSP": "ol_rsp", "OL_PSP": "ol_psp", "OL_YSP": "ol_ysp",
    "OL_RER": "ol_rer", "OL_PER": "ol_per", "OL_YER": "ol_yer",
    "OL_AP": "ol_ap", "OL_AI": "ol_ai", "OL_AD": "ol_ad",
    "OL_COL": "ol_col", "OL_TEN": "ol_ten",
}


# ---------------------------------------------------------------------------
# Logging helpers
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Arm / pre-run helpers
# ---------------------------------------------------------------------------

_PASSIVE_ANGLE_STEP_DEG = 5.0
_PASSIVE_ANGLE_LIMIT_DEG = 30.0
_CONTROL_THRUST_STEP = 0.05
_PASSIVE_RUNUP_MARGIN_S = 0.5
_PASSIVE_EKF_SETTLE_S = 3.0
_PASSIVE_EKF_TIMEOUT_S = 15.0
_PASSIVE_SETTLE_RATE_RADS = 0.05
_PASSIVE_ATTITUDE_MAX_AGE_S = 0.5
_PASSIVE_RUNUP_TIMEOUT_MARGIN_S = 5.0
_PASSIVE_LAND_CLEAR_TIMEOUT_S = 3.0
_LUA_COMMAND_TIMEOUT_S = 10.0
_GUID_OPTIONS_THRUST_AS_THRUST = 1 << 3
_PASSIVE_PROTOCOL_SEQUENCE = (
    ("capture actual", "capture", 0.0),
    ("roll +5 deg", "roll", 5.0),
    ("attitude baseline", "attitude", 0.0),
    ("roll -5 deg", "roll", -5.0),
    ("attitude baseline", "attitude", 0.0),
    ("pitch +5 deg", "pitch", 5.0),
    ("attitude baseline", "attitude", 0.0),
    ("pitch -5 deg", "pitch", -5.0),
    ("attitude baseline", "attitude", 0.0),
    ("collective +0.05", "collective", 0.05),
    ("collective baseline", "collective", 0.0),
    ("collective -0.05", "collective", -0.05),
    ("collective baseline", "collective", 0.0),
)


@dataclass
class _PassiveTarget:
    initial_q: tuple[float, float, float, float]
    thrust: float
    roll_offset_deg: float = 0.0
    pitch_offset_deg: float = 0.0
    yaw_offset_deg: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class PassiveRunOptions:
    duration_s: float | None = None
    force: bool = False
    thrust: float = _PASSIVE_IC_THRUST
    protocol_debug: bool = False
    log_dir: str | Path | None = None
    roll_offset_deg: float = 0.0
    pitch_offset_deg: float = 0.0
    yaw_offset_deg: float = 0.0
    settle_rate_deg_s: float = math.degrees(_PASSIVE_SETTLE_RATE_RADS)
    settle_time_s: float = _PASSIVE_EKF_SETTLE_S
    settle_timeout_s: float = _PASSIVE_EKF_TIMEOUT_S


def run_passive(
    session: LinkHubClient,
    options: PassiveRunOptions,
    *,
    stop_requested=None,
) -> None:
    """Run the production passive startup using typed options."""
    args = ["passive", "--trim", f"thr={options.thrust:.17g}"]
    if options.duration_s is not None:
        args.extend(("--duration", f"{options.duration_s:.17g}"))
    if options.force:
        args.append("--force")
    if options.protocol_debug:
        args.append("--protocol-debug")
    for flag, value in (
        ("--roll", options.roll_offset_deg),
        ("--pitch", options.pitch_offset_deg),
        ("--yaw", options.yaw_offset_deg),
    ):
        if value != 0.0:
            args.extend((flag, f"{value:.17g}"))
    args.extend((
        "--settle-rate-deg-s", f"{options.settle_rate_deg_s:.17g}",
        "--settle-time", f"{options.settle_time_s:.17g}",
        "--settle-timeout", f"{options.settle_timeout_s:.17g}",
    ))
    _cmd_run(
        session,
        args,
        stop_requested=stop_requested,
        log_dir=options.log_dir,
    )


def _quat_normalize(
    q: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    length = math.sqrt(sum(value * value for value in q))
    if length <= 1e-9:
        raise ValueError("Quaternion length is zero")
    return tuple(value / length for value in q)


def _quat_multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_from_euler_deg(
    roll_deg: float, pitch_deg: float, yaw_deg: float,
) -> tuple[float, float, float, float]:
    roll, pitch, yaw = map(
        math.radians, (roll_deg, pitch_deg, yaw_deg)
    )
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return _quat_normalize((
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ))


def _quat_to_euler_deg(
    q: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    w, x, y, z = _quat_normalize(q)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return tuple(map(math.degrees, (roll, pitch, yaw)))


def _passive_target_messages(
    target: _PassiveTarget,
) -> list[tuple[str, float]]:
    relative_q = _quat_from_euler_deg(
        target.roll_offset_deg,
        target.pitch_offset_deg,
        target.yaw_offset_deg,
    )
    target_q = _quat_normalize(_quat_multiply(target.initial_q, relative_q))
    target.roll_deg, target.pitch_deg, target.yaw_deg = _quat_to_euler_deg(target_q)
    return [
        ("RAWES_ROFF", math.radians(target.roll_offset_deg)),
        ("RAWES_POFF", math.radians(target.pitch_offset_deg)),
        ("RAWES_YOFF", math.radians(target.yaw_offset_deg)),
    ]


def _decode_flight_control_key(
    key: bytes, arrow_pending: list[bool],
) -> tuple[str, int] | None:
    if arrow_pending[0]:
        arrow_pending[0] = False
        return {
            b"K": ("roll", -1),
            b"M": ("roll", 1),
            b"H": ("pitch", 1),
            b"P": ("pitch", -1),
        }.get(key)
    if key in (b"\xe0", b"\x00"):
        arrow_pending[0] = True
        return None
    return {
        b"-": ("collective", -1),
        b"=": ("collective", 1),
    }.get(key)


def _adjust_passive_target(
    target: _PassiveTarget, axis: str, direction: int,
) -> list[tuple[str, float]]:
    if axis == "collective":
        target.thrust = max(
            0.0, min(1.0, target.thrust + direction * _CONTROL_THRUST_STEP)
        )
        return [("RAWES_THR", target.thrust)]

    offset_name = f"{axis}_offset_deg"
    offset = getattr(target, offset_name) + direction * _PASSIVE_ANGLE_STEP_DEG
    if axis == "yaw":
        offset = (
            offset + 180.0
        ) % 360.0 - 180.0
    else:
        offset = max(
            -_PASSIVE_ANGLE_LIMIT_DEG,
            min(_PASSIVE_ANGLE_LIMIT_DEG, offset),
        )
    setattr(target, offset_name, offset)
    return _passive_target_messages(target)


def _set_passive_target_to_actual(
    target: _PassiveTarget,
    actual_q: tuple[float, float, float, float],
) -> list[tuple[str, float]]:
    """Reset requested offsets; Lua owns the onboard anchor capture."""
    target.roll_offset_deg = 0.0
    target.pitch_offset_deg = 0.0
    target.yaw_offset_deg = 0.0
    return _passive_target_messages(target)


def _decode_passive_control_key(
    key: bytes, arrow_pending: list[bool],
) -> tuple[str, int] | None:
    yaw_change = {
        b",": ("yaw", -1),
        b"<": ("yaw", -1),
        b".": ("yaw", 1),
        b">": ("yaw", 1),
    }.get(key)
    if yaw_change is not None:
        return yaw_change
    return _decode_flight_control_key(key, arrow_pending)


def _ensure_guided_thrust_option(session: LinkHubClient) -> bool:
    options = session.get_param("GUID_OPTIONS")
    if options is None:
        print("  [FAIL] Could not read GUID_OPTIONS.")
        return False
    required = int(round(options)) | _GUID_OPTIONS_THRUST_AS_THRUST
    if required == int(round(options)):
        return True
    if not session.set_param("GUID_OPTIONS", required):
        print("  [FAIL] GUID_OPTIONS thrust-as-thrust was not acknowledged.")
        return False
    print("  GUID_OPTIONS bit 3 enabled (GUIDED attitude targets use thrust).")
    return True


def _send_lua_command(
    session: LinkHubClient,
    command: int,
    params: list[float],
    label: str,
) -> bool:
    """Send a RAWES Lua command; True only when Lua acknowledges ACCEPTED."""
    try:
        send_rawes_command(session, command, params, timeout=_LUA_COMMAND_TIMEOUT_S)
    except (LinkHubError, RuntimeError) as exc:
        print(f"  [FAIL] {label}: {exc}")
        return False
    print(f"  [OK] {label} accepted by Lua.")
    return True


def _wait_for_armed(session: LinkHubClient, timeout_s: float = 15.0) -> bool:
    """Block until armed heartbeat or timeout.  Prints STATUSTEXT inline."""
    deadline = time.monotonic() + timeout_s
    cursor = session.current_cursor()
    while time.monotonic() < deadline:
        msg, cursor = read_one(
            session, cursor, ["HEARTBEAT", "STATUSTEXT"], wait=0.5,
        )
        if msg is None:
            continue
        decoded = decode_message(msg)
        if isinstance(decoded, Statustext):
            print(f"  [FC] {decoded.text}")
        elif isinstance(decoded, Heartbeat):
            if MavModeFlag.SAFETY_ARMED in decoded.base_mode:
                return True
    return False


def _wait_for_passive_runup(
    session: LinkHubClient,
    *,
    stop_requested=None,
) -> bool:
    """Wait in ACRO until ArduPilot's configured heli runup estimate completes."""
    ramp_s = session.get_param("H_RSC_RAMP_TIME")
    runup_s = session.get_param("H_RSC_RUNUP_TIME")
    if ramp_s is None or runup_s is None:
        print("  [FAIL] Could not read heli ramp/runup timing parameters.")
        return False
    if ramp_s <= 0.0 or runup_s <= 0.0:
        print(
            "  [FAIL] Invalid heli ramp/runup timing: "
            f"H_RSC_RAMP_TIME={ramp_s}, H_RSC_RUNUP_TIME={runup_s}."
        )
        return False

    expected_s = max(float(ramp_s), float(runup_s)) + _PASSIVE_RUNUP_MARGIN_S
    timeout_s = expected_s + _PASSIVE_RUNUP_TIMEOUT_MARGIN_S
    print(
        f"  Waiting in ACRO for ArduPilot runup completion "
        f"(ramp={ramp_s:.1f}s, runup={runup_s:.1f}s) ..."
    )
    deadline = time.monotonic() + timeout_s
    cursor = session.current_cursor()
    while time.monotonic() < deadline:
        if stop_requested is not None and stop_requested():
            print("  [REMOTE] stop requested during heli runup.")
            return False
        remaining = deadline - time.monotonic()
        msg, cursor = read_one(
            session,
            cursor,
            ["HEARTBEAT", "STATUSTEXT"],
            wait=min(0.2, remaining),
        )
        if msg is not None:
            decoded = decode_message(msg)
            if isinstance(decoded, Statustext):
                print(f"  [FC] {decoded.text}")
                if "runup complete" in decoded.text.lower():
                    print("  [OK] ArduPilot reports heli runup complete.")
                    return True
            elif isinstance(decoded, Heartbeat) and (
                MavModeFlag.SAFETY_ARMED not in decoded.base_mode
            ):
                print("  [FAIL] Vehicle disarmed during heli runup.")
                return False
    print(
        f"  [FAIL] ArduPilot did not report runup complete within "
        f"{timeout_s:.1f}s."
    )
    return False


def _wait_for_passive_ekf_settle(
    session: LinkHubClient,
    *,
    stop_requested=None,
    timeout_s: float = _PASSIVE_EKF_TIMEOUT_S,
    settle_s: float = _PASSIVE_EKF_SETTLE_S,
    rate_limit_rads: float = _PASSIVE_SETTLE_RATE_RADS,
    require_active: bool = True,
) -> bool:
    """Wait for an armed quiet interval after EKF yaw alignment.

    Lua holds the attitude captured by ENTER_GUIDED throughout this wait.
    """
    session.request_data_stream(MavDataStream.EXTRA1, 10)
    deadline = time.monotonic() + timeout_s
    quiet_since: float | None = None
    last_attitude_at: float | None = None
    active = not require_active
    if require_active:
        print("  Waiting for GUIDED ACTIVE and settled EKF yaw ...")
    else:
        print("  Waiting for settled EKF yaw in ACRO before GUIDED ...")
    cursor = session.current_cursor()
    while time.monotonic() < deadline:
        if stop_requested is not None and stop_requested():
            print("  [REMOTE] stop requested during EKF settling.")
            return False
        msg, cursor = read_one(
            session,
            cursor,
            ["HEARTBEAT", "STATUSTEXT", "ATTITUDE", "ATTITUDE_QUATERNION"],
            wait=0.2,
        )
        now = time.monotonic()
        if msg is not None:
            decoded = decode_message(msg)
            if isinstance(decoded, Heartbeat):
                if MavModeFlag.SAFETY_ARMED not in decoded.base_mode:
                    print("  [FAIL] Vehicle disarmed during EKF settling.")
                    return False
                if require_active and (
                    decoded.system_status == MavState.ACTIVE
                    and not active
                ):
                    active = True
                    quiet_since = None
                    print("  [FC] GUIDED is ACTIVE; monitoring yaw settling.")
            elif isinstance(decoded, Statustext):
                print(f"  [FC] {decoded.text}")
                if "yaw alignment complete" in decoded.text.lower():
                    quiet_since = None
            elif isinstance(decoded, (Attitude, AttitudeQuaternion)) and active:
                last_attitude_at = now
                if max(
                    abs(decoded.rollspeed),
                    abs(decoded.pitchspeed),
                    abs(decoded.yawspeed),
                ) > rate_limit_rads:
                    quiet_since = None
                elif quiet_since is None:
                    quiet_since = now
        attitude_fresh = (
            last_attitude_at is not None
            and now - last_attitude_at <= _PASSIVE_ATTITUDE_MAX_AGE_S
        )
        if (
            active
            and quiet_since is not None
            and attitude_fresh
            and now - quiet_since >= settle_s
        ):
            print(
                f"  [OK] EKF attitude quiet for {settle_s:.1f}s; "
                "capturing passive target."
            )
            return True
    phase = "GUIDED/EKF" if require_active else "ACRO/EKF"
    print(f"  [FAIL] Timed out waiting for {phase} attitude to settle.")
    return False


def _configure_passive_startup_telemetry(session: LinkHubClient) -> None:
    """Enable evidence streams before arming so the handoff is fully logged."""
    for stream in (
        MavDataStream.EXTRA1,
        MavDataStream.RC_CHANNELS,
    ):
        session.request_data_stream(stream, 25)
    for message_id in (
        AttitudeQuaternion.MAVLINK_ID,
        AttitudeTarget.MAVLINK_ID,
        ExtendedSysState.MAVLINK_ID,
    ):
        session.send_message(CommandLong(
            target_system=session._target_system,
            target_component=session._target_component,
            command=MavCmd.SET_MESSAGE_INTERVAL,
            param1=float(message_id),
            param2=40000.0,
        ))


def _wait_for_passive_land_clear(
    session: LinkHubClient,
    *,
    stop_requested=None,
    timeout_s: float = _PASSIVE_LAND_CLEAR_TIMEOUT_S,
) -> bool:
    """Wait for ArduPilot to clear land_complete during armed ACRO runup."""
    deadline = time.monotonic() + timeout_s
    cursor = session.current_cursor()
    while time.monotonic() < deadline:
        if stop_requested is not None and stop_requested():
            print("  [REMOTE] stop requested while clearing landed state.")
            return False
        msg, cursor = read_one(
            session,
            cursor,
            ["EXTENDED_SYS_STATE", "HEARTBEAT", "STATUSTEXT"],
            wait=min(0.2, deadline - time.monotonic()),
        )
        if msg is None:
            continue
        decoded = decode_message(msg)
        if isinstance(decoded, ExtendedSysState):
            if decoded.landed_state is MavLandedState.IN_AIR:
                print("  [OK] ArduPilot landed state cleared in ACRO.")
                return True
        elif isinstance(decoded, Heartbeat) and (
            MavModeFlag.SAFETY_ARMED not in decoded.base_mode
        ):
            print("  [FAIL] Vehicle disarmed while clearing landed state.")
            return False
        elif isinstance(decoded, Statustext):
            print(f"  [FC] {decoded.text}")
    print("  [FAIL] ArduPilot remained landed after ACRO collective staging.")
    return False


def _wait_for_disarmed(session: LinkHubClient, timeout_s: float) -> bool:
    """Wait for Lua or ArduPilot to confirm disarm via heartbeat."""
    status = session.vehicle_status()
    base_mode = status.get("base_mode")
    if base_mode is not None and MavModeFlag.SAFETY_ARMED not in base_mode:
        print("  [OK] Vehicle already disarmed.")
        return True

    deadline = time.monotonic() + timeout_s
    cursor = session.current_cursor()
    while time.monotonic() < deadline:
        msg, cursor = read_one(
            session, cursor, ["HEARTBEAT", "STATUSTEXT"], wait=0.2,
        )
        if msg is None:
            continue
        decoded = decode_message(msg)
        if isinstance(decoded, Statustext):
            print(f"  [FC] {decoded.text}")
        elif isinstance(decoded, Heartbeat):
            if MavModeFlag.SAFETY_ARMED not in decoded.base_mode:
                print("  [OK] Vehicle disarmed by Lua.")
                return True
    return False


def _take_servo4(
    session: LinkHubClient,
    restore_function: "float | None" = None,
) -> "float | None":
    """Release the motor output and return the function needed during flight."""
    servo_key = f"SERVO{SERVO_MOTOR}_FUNCTION"
    saved = session.get_param(servo_key)
    if saved is None:
        return None
    if saved != 0:
        session.set_param(servo_key, 0)
        print(f"  {servo_key} {saved:.0f} -> 0 (released from DDFP)")
        return float(saved)
    return restore_function


def _ensure_passive_tail_setup(session: LinkHubClient) -> bool:
    """Verify that AP-owned DDFP tail control was mapped at boot."""
    expected_tail = 3.0
    tail = session.get_param("H_TAIL_TYPE")
    if tail is None:
        print("  [FAIL] H_TAIL_TYPE unreadable; refusing passive run.")
        return False
    elif abs(float(tail) - expected_tail) > 1e-4:
        print(
            f"  [FAIL] H_TAIL_TYPE={tail:.6g}; passive requires "
            f"{expected_tail:.0f} (DDFP CW) at boot."
        )
        return False

    s4f = session.get_param(f"SERVO{SERVO_MOTOR}_FUNCTION")
    if s4f is None:
        print(
            f"  [FAIL] SERVO{SERVO_MOTOR}_FUNCTION unreadable; "
            "refusing passive run."
        )
        return False
    if int(round(float(s4f))) != 36:
        print(
            f"  [FAIL] SERVO{SERVO_MOTOR}_FUNCTION={s4f:.6g}; "
            "set it to 36 and reboot before passive flight."
        )
        return False
    print(
        f"  [OK] SERVO{SERVO_MOTOR}_FUNCTION=36 "
        "(DDFP mapping established at boot)."
    )
    return True


def _safety_shutdown(session: LinkHubClient, *,
                     saved_overrides: "dict[str, float] | None" = None,
                     skip_motor_off: bool = False) -> None:
    """Stop Lua control, disarm if needed, then enter ACRO safe-off."""
    print("  [SAFETY] shutting down ...")
    try:
        session.set_param("RAWES_MODE", 0)
        print("  [SAFETY] RAWES_MODE -> 0 (Lua mode none)")
    except Exception as e:
        print(f"  [SAFETY] failed to set RAWES_MODE=0: {e}")
    if not skip_motor_off:
        try:
            _send_set_servo(session, SERVO_MOTOR, MOTOR_OFF_US)
            print(f"  [SAFETY] SERVO{SERVO_MOTOR} -> {MOTOR_OFF_US} us (motor off)")
        except Exception as e:
            print(f"  [SAFETY] failed to drive SERVO{SERVO_MOTOR} off: {e}")
    try:
        disarmed = _wait_for_disarmed(session, timeout_s=0.2)
    except Exception as e:
        print(f"  [SAFETY] could not read current arm state: {e}")
        disarmed = False

    if disarmed:
        _set_safe_off_state(session, rawes_mode_released=True)
    else:
        print("  [SAFETY] vehicle armed -- force-disarming bench run")
        try:
            if not _disarm(session, timeout=5.0, force=True):
                print("  [SAFETY] force-disarm not confirmed")
        except Exception as e:
            print(f"  [SAFETY] force-disarm command failed: {e}")
    for param, orig in (saved_overrides or {}).items():
        if param == "RAWES_MODE":
            print("  [SAFETY] RAWES_MODE remains 0 in safe-off state")
            continue
        try:
            session.set_param(param, orig)
            print(f"  [SAFETY] {param} restored to {orig:.6g}")
        except Exception as e:
            print(f"  [SAFETY] failed to restore {param}: {e}")


# ---------------------------------------------------------------------------
# Key polling
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Shared observation loop engine
# ---------------------------------------------------------------------------

# Yaw rate-PID telemetry rate. ArduPilot emits PID_TUNING only for the axes
# enabled in GCS_PID_MASK (hardware/rawes_hardware_defaults.parm sets yaw) and
# only after a rate is requested; this is low enough to stay cheap on a radio.
_PID_TUNING_HZ = 4.0


def _request_pid_tuning(session: LinkHubClient, hz: float = _PID_TUNING_HZ) -> None:
    session.send_message(CommandLong(
        target_system=session._target_system,
        target_component=session._target_component,
        command=MavCmd.SET_MESSAGE_INTERVAL,
        param1=float(PidTuning.MAVLINK_ID),
        param2=1_000_000.0 / hz,
    ))


def _observation_loop(session: LinkHubClient, *,
                      duration_s: "float | None",
                      msg_types: list[str],
                      streams: list[tuple[MavDataStream, int]],
                      handle_msg,
                      render_row,
                      header_cols: list[str],
                      log: "_RunLog | None" = None,
                      print_period_s: float = 1.0,
                      header_print_cols: "list[str] | None" = None,
                      on_tick=None,
                      suppress_status: bool = False,
                      key_handler=None,
                      setup_hook=None,
                      loop_hook=None,
                      stop_requested=None,
                      ) -> tuple[int, bool]:
    """Run the standard observation loop.

    handle_msg(state, msg, t_rel) -> updates state dict in place.
    render_row(state, t_rel)      -> list of CSV values (None entries -> blank).
    header_cols                   -> CSV column names.
    header_print_cols (optional)  -> if set, used for the live console table
                                     header.  Defaults to header_cols.
    on_tick(t_rel)    (optional)  -> called once per loop iteration; useful for
                                     scheduled side-effects (e.g. oscillating
                                     trim NVFs).

    Returns (n_rows, aborted).
    """
    for stream, hz in streams:
        session.request_data_stream(stream, hz)
    _request_pid_tuning(session)

    if setup_hook is not None:
        setup_hook()

    if log is not None:
        log.write_header(header_cols)

    print_hdr = header_print_cols if header_print_cols is not None else header_cols
    widths = [max(6, len(c) + 1) for c in print_hdr]
    print("  " + "  ".join(f"{c:>{w}}" for c, w in zip(print_hdr, widths)))
    print("  " + "  ".join("-" * w for w in widths))

    state = {"armed": True, "pending_text": []}
    t0 = time.monotonic()
    deadline = (t0 + duration_s) if duration_s else None
    last_print = -1.0
    aborted = False
    cursor = session.current_cursor()
    print("  Press ESC (or Ctrl-C) to abort.")
    try:
        while True:
            if deadline and time.monotonic() >= deadline:
                break
            if stop_requested is not None and stop_requested():
                aborted = True
                print("\n  [REMOTE] stop requested -- running safety shutdown ...")
                break
            keys = _poll_keys()
            if b"\x1b" in keys:
                aborted = True
                print("\n  [ESC] abort -- running safety shutdown ...")
                break
            if key_handler is not None:
                for k in keys:
                    key_handler(k)
            batch = session.read_messages(
                cursor,
                msg_types,
                direction="rx",
                wait=0.1,
                limit=1_000,
            )
            cursor = batch.next_cursor
            t_rel = time.monotonic() - t0
            if on_tick is not None:
                on_tick(t_rel)
            for msg in batch.messages:
                decoded = decode_message(msg)
                if isinstance(decoded, Heartbeat):
                    state["armed"] = MavModeFlag.SAFETY_ARMED in decoded.base_mode
                elif isinstance(decoded, Statustext):
                    if not suppress_status:
                        text = decoded.text
                        if text:
                            state["pending_text"].append(text)
                else:
                    row = handle_msg(state, decoded, t_rel)
                    if row is not None and log is not None:
                        log.row(row)
            if loop_hook is not None and not loop_hook(state, t_rel):
                aborted = True
                print("\n  [VIEW] closed -- running safety shutdown ...")
                break
            if t_rel - last_print >= print_period_s:
                last_print = t_rel
                cells = render_row(state, t_rel)
                txt = state["pending_text"].pop(0) if state["pending_text"] else ""
                cell_strs = [f"{v}" if v is not None else "n/a" for v in cells]
                line = "  ".join(f"{c:>{w}}" for c, w in zip(cell_strs, widths))
                print(f"  {line}  {txt}")
                while state["pending_text"]:
                    print(f"  {' '*sum(widths)}  {state['pending_text'].pop(0)}")
    except KeyboardInterrupt:
        print()
        aborted = True
    return (log.n_rows if log else 0, aborted)


# ---------------------------------------------------------------------------
# Generic run observation loop
# ---------------------------------------------------------------------------

def _run_observation(session: LinkHubClient, mode_name: str,
                     duration: "float | None", log: _RunLog,
                     on_tick=None, keep_rc: bool = False,
                     manual_controls: "dict[str, float] | None" = None,
                     passive_target: "_PassiveTarget | None" = None,
                     rotor_motor: "LinkHubMotorController | None" = None,
                     protocol_debug: bool = False,
                     auto_sequence_hold_s: float | None = None,
                     stop_requested=None) -> None:
    """Generic observation loop for `run` modes.  Stream + columns are the
    same across all modes; the row content is whatever the FC reports.  Per-
    mode NVFs (e.g. YAW_I, YAW_OUT) appear as columns when emitted.

    on_tick(t_rel) is called once per loop iteration (~10 Hz); used e.g. by
    the `motor` command to refresh PWM.

    Serial bandwidth is tight (57600 SiK ~= 5760 B/s, ~70% used by default),
    which drops NVF/telemetry.  So we trim streams we do not need for tuning:
    AHRS2 (EXTRA3) off, EXTENDED_STATUS down to 1 Hz, and RC_CHANNELS disabled
    (keeping SERVO_OUTPUT_RAW in the same stream) unless keep_rc is set."""
    from .util import _fmt   # pure utility, no cycle risk

    cols = ["t_s", "armed",
            "mav_att_roll_deg", "mav_att_pitch_deg", "mav_att_yaw_deg",
            "mav_att_yaw_rate_rads",
            "mav_att_target_roll_deg", "mav_att_target_pitch_deg", "mav_att_target_yaw_deg",
            "mav_att_target_roll_rate_rads", "mav_att_target_pitch_rate_rads", "mav_att_target_yaw_rate_rads",
            "mav_att_target_thrust",
            "mav_att_q_w", "mav_att_q_x", "mav_att_q_y", "mav_att_q_z",
            "mav_att_target_q_w", "mav_att_target_q_x", "mav_att_target_q_y", "mav_att_target_q_z",
            "mav_att_qerr_w", "mav_att_qerr_x", "mav_att_qerr_y", "mav_att_qerr_z",
            "mav_att_qerr_deg", "mav_att_qerr_yaw_deg",
            "mav_pos_n_m", "mav_pos_e_m", "mav_pos_d_m",
            "ch1_us", "ch2_us", "ch3_us", "ch4_us",
            "mav_servo1_us", "mav_servo2_us", "mav_servo3_us", "mav_servo9_us",
            "vbat_v", "current_a",
            "mav_nvf_yff_trim", "mav_nvf_yff_u", "mav_nvf_yff_gz",
            "mav_nvf_ol_rsp", "mav_nvf_ol_psp", "mav_nvf_ol_ysp",
            "mav_nvf_ol_rer", "mav_nvf_ol_per", "mav_nvf_ol_yer",
            "mav_nvf_ol_ap", "mav_nvf_ol_ai", "mav_nvf_ol_ad",
            "mav_nvf_ol_col", "mav_nvf_ol_ten",
            "mav_pid_roll_des", "mav_pid_roll_ach", "mav_pid_roll_ff", "mav_pid_roll_p", "mav_pid_roll_i", "mav_pid_roll_d",
            "mav_pid_pitch_des", "mav_pid_pitch_ach", "mav_pid_pitch_ff", "mav_pid_pitch_p", "mav_pid_pitch_i", "mav_pid_pitch_d",
            "mav_pid_yaw_des", "mav_pid_yaw_ach", "mav_pid_yaw_ff", "mav_pid_yaw_p", "mav_pid_yaw_i", "mav_pid_yaw_d",
            "erpm", "mech_rpm", "rotor_rpm"]

    # Live table: H_YAW_TRIM output (out=trim), motor throttle command (u=YFF_U),
    # swashplate PWMs (s1..s3), GB4008 motor PWM with 5 s rolling average, rotor RPM.
    if mode_name == "acro-manual":
        print_cols = [
            "t(s)", "armed", "rc1", "rc2", "rc3", "rc4",
            "s1", "s2", "s3", "mot", "yaw(d)", "yrate_d",
        ]
    elif mode_name == "passive":
        print_cols = [
            "t(s)", "armed", "roll", "pitch", "trg_r", "trg_p", "thr",
            "yaw", "qerr(d)", "qyaw(d)", "s1", "s2", "s3", "mot",
        ]
    else:
        print_cols = ["t(s)", "armed", "yaw(d)", "yrate_d", "out", "u",
                      "s1", "s2", "s3", "mot", "mot~5s", "qerr(d)", "qyaw(d)", "mRPM"]

    mot_window_s = 5.0   # rolling average window for motor (SERVO_MOTOR) PWM
    protocol_started = time.monotonic()
    protocol_sequence = 0
    protocol_pending_sequence: int | None = None
    protocol_pending_until = 0.0
    protocol_last_target_q = None
    auto_sequence_index = -1
    auto_sequence_baseline_thrust = (
        passive_target.thrust if passive_target is not None else 0.5
    )

    def _protocol_print(message: str) -> None:
        if protocol_debug:
            elapsed = time.monotonic() - protocol_started
            print(f"\n  [PROTO {elapsed:8.3f}] {message}")

    def _protocol_state(label: str) -> None:
        actual = state["att_q"]
        target = state["att_target_q"]
        _, _, _, _, qerr_deg, _ = _quat_error(actual, target)
        actual_text = (
            "n/a" if actual is None
            else " ".join(f"{value:+.6f}" for value in actual)
        )
        target_text = (
            "n/a" if target is None
            else " ".join(f"{value:+.6f}" for value in target)
        )
        _protocol_print(
            f"{label}\n"
            f"    actual q  {actual_text}\n"
            f"    target q  {target_text}\n"
            f"    qerr       {'n/a' if qerr_deg is None else f'{qerr_deg:.3f} deg'}\n"
            f"    servo pwm  {state['s1']} / {state['s2']} / {state['s3']}"
        )

    passive_reset_handler = None
    if mode_name == "none":
        passive_control_handler = None
        # In none mode (Lua idle) the yaw PID is inert, so the live keys tune the
        # static DDFP trim H_YAW_TRIM directly.  Step = 0.005 (~5 us over the
        # SERVO_MOTOR 1000-2000 us range); clamp [0, 1].
        _trim = {"param": "H_YAW_TRIM", "step": 0.005, "val": 0.0}
        _tv = session.get_param(_trim["param"])
        _trim["val"] = float(_tv) if _tv is not None else 0.0
        _pwm = 1000 + _trim["val"] * 1000.0
        print(f"  Yaw trim: H_YAW_TRIM={_trim['val']:.4f}  (~{_pwm:.0f} us)")
        print("  tune keys:  UP/DOWN arrows  or  '-'/'='   (H_YAW_TRIM +/- 0.005)")
        _arrow_pending = [False]

        def key_handler(k: bytes) -> None:
            # Windows arrow keys arrive as a two-byte sequence: a 0xe0/0x00
            # prefix followed by 'H' (up) / 'P' (down).  _poll_keys yields the
            # bytes separately, so latch the prefix and decode on the next byte.
            sign = 0
            if _arrow_pending[0]:
                _arrow_pending[0] = False
                if k == b"H":
                    sign = +1
                elif k == b"P":
                    sign = -1
                else:
                    return
            elif k in (b"\xe0", b"\x00"):
                _arrow_pending[0] = True
                return
            elif k == b"=":
                sign = +1
            elif k == b"-":
                sign = -1
            else:
                return
            old = _trim["val"]
            new = min(1.0, max(0.0, old + sign * _trim["step"]))
            ok = session.set_param(_trim["param"], new)
            _trim["val"] = new
            pwm = 1000 + new * 1000.0
            tag = "" if ok else "  [FAIL]"
            print(f"  TRIM {old:.4f} -> {new:.4f}  (~{pwm:.0f} us){tag}")
    elif mode_name == "acro-manual":
        passive_control_handler = None
        if manual_controls is None:
            raise ValueError("manual_controls required for acro-manual")
        print("  ACRO manual controls:")
        print("    arrows: LEFT/RIGHT roll, UP/DOWN pitch")
        print("    -/= (no Shift): collective down/up")
        print("    step=0.05; ESC exits and disarms")
        _arrow_pending = [False]

        def _send_manual(name: str) -> None:
            wire_name = {
                "roll": "RAWES_RLL",
                "pitch": "RAWES_PIT",
                "collective": "RAWES_COL",
            }[name]
            session.send_message(NamedValueFloat(name=wire_name, value=manual_controls[name]))

        def _show_manual() -> None:
            print(
                "  MANUAL "
                f"roll={manual_controls['roll']:+.2f}  "
                f"pitch={manual_controls['pitch']:+.2f}  "
                f"collective={manual_controls['collective']:.2f}"
            )

        def key_handler(k: bytes) -> None:
            change = _decode_flight_control_key(k, _arrow_pending)
            if change is None:
                return
            axis, direction = change

            lower, upper = (0.0, 1.0) if axis == "collective" else (-1.0, 1.0)
            manual_controls[axis] = max(
                lower, min(upper, manual_controls[axis] + direction * 0.05)
            )
            _send_manual(axis)
            _show_manual()
    elif mode_name == "passive":
        if passive_target is None:
            raise ValueError("passive_target required for passive")
        print("  PASSIVE target controls:")
        print("    arrows: LEFT/RIGHT target roll, UP/DOWN target pitch")
        print("    ,/. (or </>): target yaw left/right")
        print("    -/= (no Shift): held thrust down/up")
        print("    Space: set target attitude to current actual attitude")
        print("    angle step=5 deg, roll/pitch travel=+/-30 deg from start; thrust step=0.05")
        if rotor_motor is not None:
            print("  ROTOR MOTOR controls:")
            print("    m: start/stop, [/]: speed -/+ 5, 0: emergency stop")
            print(
                f"    speed={rotor_motor.speed}%  "
                f"direction={'CW' if rotor_motor.direction == 'F' else 'CCW'}"
            )
        print("    ESC exits, disarms, and leaves RAWES_MODE off")
        _arrow_pending = [False]

        def _show_passive_target() -> None:
            print(
                "  TARGET "
                f"relative=({passive_target.roll_offset_deg:+.1f}, "
                f"{passive_target.pitch_offset_deg:+.1f}, "
                f"{passive_target.yaw_offset_deg:+.1f}) deg  "
                f"target_rpy=({passive_target.roll_deg:+.1f}, "
                f"{passive_target.pitch_deg:+.1f}, "
                f"{passive_target.yaw_deg:+.1f}) deg  "
                f"thrust={passive_target.thrust:.2f}"
            )

        _show_passive_target()

        def passive_control_handler(axis: str, direction: int) -> None:
            nonlocal protocol_sequence, protocol_pending_sequence
            nonlocal protocol_pending_until, protocol_last_target_q
            protocol_sequence += 1
            protocol_pending_sequence = protocol_sequence
            protocol_pending_until = time.monotonic() + 1.0
            protocol_last_target_q = None
            _protocol_state(
                f"COMMAND #{protocol_sequence}: {axis} "
                f"{'+' if direction > 0 else '-'}"
            )
            for wire_name, value in _adjust_passive_target(
                passive_target, axis, direction
            ):
                session.send_message(NamedValueFloat(name=wire_name, value=value))
                _protocol_print(
                    f"TX #{protocol_sequence} NAMED_VALUE_FLOAT "
                    f"{wire_name}={value:+.7f}"
                )
            _show_passive_target()

        def passive_reset_handler(
            actual_q: tuple[float, float, float, float],
        ) -> None:
            nonlocal protocol_sequence, protocol_pending_sequence
            nonlocal protocol_pending_until, protocol_last_target_q
            protocol_sequence += 1
            protocol_pending_sequence = protocol_sequence
            protocol_pending_until = time.monotonic() + 1.0
            protocol_last_target_q = None
            _protocol_state(
                f"COMMAND #{protocol_sequence}: SPACE target=current onboard AHRS"
            )
            for wire_name, value in _set_passive_target_to_actual(
                passive_target, actual_q
            ):
                session.send_message(NamedValueFloat(name=wire_name, value=value))
                _protocol_print(
                    f"TX #{protocol_sequence} NAMED_VALUE_FLOAT "
                    f"{wire_name}={value:+.1f} (onboard atomic capture)"
                )
            _show_passive_target()

        def key_handler(k: bytes) -> None:
            if rotor_motor is not None:
                if k in (b"m", b"M"):
                    if rotor_motor.enabled:
                        rotor_motor.stop()
                        print("  ROTOR MOTOR stopped")
                    else:
                        rotor_motor.start()
                        print(f"  ROTOR MOTOR running at {rotor_motor.speed}%")
                    return
                if k in (b"[", b"]"):
                    delta = -5 if k == b"[" else 5
                    rotor_motor.set_speed(
                        max(0, min(100, rotor_motor.speed + delta))
                    )
                    print(f"  ROTOR MOTOR speed={rotor_motor.speed}%")
                    return
                if k == b"0":
                    rotor_motor.stop()
                    print("  ROTOR MOTOR emergency stop")
                    return
            if k == b" ":
                actual_q = state["att_q"]
                if actual_q is None:
                    print("  [FAIL] No actual quaternion available for target capture.")
                    return
                passive_reset_handler(actual_q)
                return
            change = _decode_passive_control_key(k, _arrow_pending)
            if change is not None:
                passive_control_handler(*change)
    elif mode_name in ("steady", "pumping"):
        passive_control_handler = None
        _yaw_pid = {
            "P": {"param": "ATC_RAT_YAW_P", "step": 0.002,  "val": 0.0},
            "I": {"param": "ATC_RAT_YAW_I", "step": 0.0005, "val": 0.0},
            "D": {"param": "ATC_RAT_YAW_D", "step": 0.001,  "val": 0.0},
        }
        for axis in _yaw_pid.values():
            _tv = session.get_param(axis["param"])
            axis["val"] = float(_tv) if _tv is not None else 0.0
        print("  Yaw PID:")
        print(f"    P={_yaw_pid['P']['val']:.4f}  I={_yaw_pid['I']['val']:.4f}  D={_yaw_pid['D']['val']:.4f}")
        print("  tune keys:  q/a = P +/- 0.002,  w/s = I +/- 0.0005,  e/d = D +/- 0.001")

        def _bump(axis_name: str, sign: int) -> None:
            axis = _yaw_pid[axis_name]
            old = axis["val"]
            new = max(0.0, old + sign * axis["step"])
            ok = session.set_param(axis["param"], new)
            axis["val"] = new
            tag = "" if ok else "  [FAIL]"
            print(f"  {axis['param']} {old:.4f} -> {new:.4f}{tag}")

        def key_handler(k: bytes) -> None:
            if k == b"q":
                _bump("P", +1)
            elif k == b"a":
                _bump("P", -1)
            elif k == b"w":
                _bump("I", +1)
            elif k == b"s":
                _bump("I", -1)
            elif k == b"e":
                _bump("D", +1)
            elif k == b"d":
                _bump("D", -1)
    else:
        passive_control_handler = None
        def key_handler(k: bytes) -> None:  # no live key tuning for this mode
            pass

    state = {
        "roll": None, "pitch": None, "yaw": None, "yaw_rate": None,
        "att_target_roll": None, "att_target_pitch": None, "att_target_yaw": None,
        "att_target_roll_rate": None, "att_target_pitch_rate": None, "att_target_yaw_rate": None,
        "att_target_thrust": None,
        "att_q": None,          # actual attitude quaternion (w, x, y, z) from ATTITUDE_QUATERNION
        "att_target_q": None,   # target attitude quaternion (w, x, y, z) from ATTITUDE_TARGET
        "pos_ned": None,
        "ch1": None, "ch2": None, "ch3": None, "ch4": None,
        "s1": None, "s2": None, "s3": None, "smot": None,
        "smot_hist": [],
        "mrpm_hist": [],    # (t_rel, mech_rpm) for the 5 s rolling average on screen
        "vbat": None, "curr": None,
        "erpm": None,
        "yff_t": None, "yff_u": None, "yff_gz": None,
        "ol_rsp": None, "ol_psp": None, "ol_ysp": None,
        "ol_rer": None, "ol_per": None, "ol_yer": None,
        "ol_ap": None, "ol_ai": None, "ol_ad": None,
        "ol_col": None, "ol_ten": None,
        "yff_t_ts": None, "yff_u_ts": None, "yff_gz_ts": None,
        "pid_roll_des": None, "pid_roll_ach": None, "pid_roll_ff": None, "pid_roll_p": None, "pid_roll_i": None, "pid_roll_d": None,
        "pid_pitch_des": None, "pid_pitch_ach": None, "pid_pitch_ff": None, "pid_pitch_p": None, "pid_pitch_i": None, "pid_pitch_d": None,
        "pid_yaw_des": None, "pid_yaw_ach": None, "pid_yaw_ff": None, "pid_yaw_p": None, "pid_yaw_i": None, "pid_yaw_d": None,
    }

    def _quat_to_rpy_deg(q) -> tuple[float, float, float] | tuple[None, None, None]:
        if q is None or len(q) != 4:
            return None, None, None
        w, x, y, z = [float(v) for v in q]
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
        else:
            pitch = math.degrees(math.asin(sinp))

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        return roll, pitch, yaw

    def _quat_conj(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        w, x, y, z = q
        return (w, -x, -y, -z)

    def _quat_mul(a: tuple[float, float, float, float],
                  b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    def _quat_error(q_actual, q_target):
        """Body-frame rotation error q_err = conj(q_actual) (x) q_target --
        i.e. the rotation that takes the actual attitude onto the target
        attitude.  Using the quaternion error directly (rather than
        differencing the two Euler yaws) avoids the +-180 deg wraparound
        ambiguity Euler subtraction has at the heading-lock boundary.

        Returns (qerr_w, qerr_x, qerr_y, qerr_z, total_err_deg, yaw_err_deg)
        or a tuple of Nones if either input is missing.  yaw_err_deg uses the
        small-roll/pitch-error approximation 2*atan2(z, w), which is exact
        when the error is a pure yaw rotation (the case for a yaw/heading
        lock holding roll/pitch elsewhere) and near-exact otherwise."""
        if q_actual is None or q_target is None:
            return (None,) * 6
        qe = _quat_mul(_quat_conj(q_actual), q_target)
        w, x, y, z = qe
        total_deg = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, abs(w)))))
        yaw_deg = math.degrees(2.0 * math.atan2(z, w))
        return w, x, y, z, total_deg, yaw_deg

    def handle_msg(st, msg, t_rel):
        if isinstance(msg, Attitude):
            state["roll"]     = math.degrees(msg.roll)
            state["pitch"]    = math.degrees(msg.pitch)
            state["yaw"]      = math.degrees(msg.yaw)
            state["yaw_rate"] = msg.yawspeed          # rad/s (mav_att_yaw_rate_rads)
            # Emit one CSV row per ATTITUDE message (typically 10-50 Hz)
            _erpm, _mech, _rotor = _rpm_triplet(state["erpm"])
            _qw, _qx, _qy, _qz, _qerr_deg, _qerr_yaw_deg = _quat_error(state["att_q"], state["att_target_q"])
            return [
                f"{t_rel:.4f}", int(st["armed"]),
                _fmt(state["roll"]), _fmt(state["pitch"]), _fmt(state["yaw"]),
                _fmt(state["yaw_rate"]),
                _fmt(state["att_target_roll"]), _fmt(state["att_target_pitch"]), _fmt(state["att_target_yaw"]),
                _fmt(state["att_target_roll_rate"]), _fmt(state["att_target_pitch_rate"]), _fmt(state["att_target_yaw_rate"]),
                _fmt(state["att_target_thrust"]),
                *(_fmt(v) for v in (state["att_q"] or (None, None, None, None))),
                *(_fmt(v) for v in (state["att_target_q"] or (None, None, None, None))),
                _fmt(_qw), _fmt(_qx), _fmt(_qy), _fmt(_qz),
                _fmt(_qerr_deg), _fmt(_qerr_yaw_deg),
                *(_fmt(v) for v in (state["pos_ned"] or (None, None, None))),
                state["ch1"], state["ch2"], state["ch3"], state["ch4"],
                state["s1"], state["s2"], state["s3"], state["smot"],
                _fmt(state["vbat"]), _fmt(state["curr"]),
                _fmt(state["yff_t"]), _fmt(state["yff_u"]), _fmt(state["yff_gz"]),
                _fmt(state["ol_rsp"]), _fmt(state["ol_psp"]), _fmt(state["ol_ysp"]),
                _fmt(state["ol_rer"]), _fmt(state["ol_per"]), _fmt(state["ol_yer"]),
                _fmt(state["ol_ap"]), _fmt(state["ol_ai"]), _fmt(state["ol_ad"]),
                _fmt(state["ol_col"]), _fmt(state["ol_ten"]),
                _fmt(state["pid_roll_des"]), _fmt(state["pid_roll_ach"]), _fmt(state["pid_roll_ff"]), _fmt(state["pid_roll_p"]), _fmt(state["pid_roll_i"]), _fmt(state["pid_roll_d"]),
                _fmt(state["pid_pitch_des"]), _fmt(state["pid_pitch_ach"]), _fmt(state["pid_pitch_ff"]), _fmt(state["pid_pitch_p"]), _fmt(state["pid_pitch_i"]), _fmt(state["pid_pitch_d"]),
                _fmt(state["pid_yaw_des"]), _fmt(state["pid_yaw_ach"]), _fmt(state["pid_yaw_ff"]), _fmt(state["pid_yaw_p"]), _fmt(state["pid_yaw_i"]), _fmt(state["pid_yaw_d"]),
                _fmt(_erpm), _fmt(_mech), _fmt(_rotor),
            ]
        if isinstance(msg, AttitudeTarget):
            nonlocal protocol_last_target_q
            att_r, att_p, att_y = _quat_to_rpy_deg(msg.q)
            state["att_target_roll"] = att_r
            state["att_target_pitch"] = att_p
            state["att_target_yaw"] = att_y
            state["att_target_roll_rate"] = msg.body_roll_rate
            state["att_target_pitch_rate"] = msg.body_pitch_rate
            state["att_target_yaw_rate"] = msg.body_yaw_rate
            state["att_target_thrust"] = msg.thrust
            if msg.q is not None and len(msg.q) == 4:
                state["att_target_q"] = tuple(float(v) for v in msg.q)
                target_q = state["att_target_q"]
                if (
                    protocol_debug
                    and protocol_pending_sequence is not None
                    and time.monotonic() <= protocol_pending_until
                    and target_q != protocol_last_target_q
                ):
                    _, _, _, _, qerr_deg, _ = _quat_error(
                        state["att_q"], target_q
                    )
                    _protocol_print(
                        f"RX #{protocol_pending_sequence} ATTITUDE_TARGET\n"
                        f"    target q   "
                        f"{' '.join(f'{value:+.6f}' for value in target_q)}\n"
                        f"    rates rad/s {msg.body_roll_rate:+.4f} "
                        f"{msg.body_pitch_rate:+.4f} {msg.body_yaw_rate:+.4f}\n"
                        f"    thrust     {msg.thrust:.4f}\n"
                        f"    qerr       "
                        f"{'n/a' if qerr_deg is None else f'{qerr_deg:.3f} deg'}"
                    )
                    protocol_last_target_q = target_q
        elif isinstance(msg, AttitudeQuaternion):
            state["att_q"] = (msg.q1, msg.q2, msg.q3, msg.q4)
        elif isinstance(msg, LocalPositionNed):
            state["pos_ned"] = (msg.x, msg.y, msg.z)
        elif isinstance(msg, RcChannels):
            state["ch1"] = msg.chan1_raw
            state["ch2"] = msg.chan2_raw
            state["ch3"] = msg.chan3_raw
            state["ch4"] = msg.chan4_raw
        elif hasattr(msg, "servo1_raw"):
            state["s1"] = getattr(msg, "servo1_raw", None)
            state["s2"] = getattr(msg, "servo2_raw", None)
            state["s3"] = getattr(msg, "servo3_raw", None)
            # GB4008 motor is on output SERVO_MOTOR (AUX 1); SERVO4 is unused now.
            state["smot"] = getattr(msg, f"servo{SERVO_MOTOR}_raw", None)
            if state["smot"] is not None:
                state["smot_hist"].append((t_rel, float(state["smot"])))
                cutoff = t_rel - mot_window_s
                while state["smot_hist"] and state["smot_hist"][0][0] < cutoff:
                    state["smot_hist"].pop(0)
        elif hasattr(msg, "voltages"):
            cells = [v for v in msg.voltages if v != 65535]
            if cells:
                state["vbat"] = sum(cells) / 1000.0
            if msg.current_battery >= 0:
                state["curr"] = msg.current_battery / 100.0
        elif hasattr(msg, "voltage_battery"):
            if state["vbat"] is None and msg.voltage_battery != 65535:
                state["vbat"] = msg.voltage_battery / 1000.0
            if state["curr"] is None and msg.current_battery >= 0:
                state["curr"] = msg.current_battery / 100.0
        elif isinstance(msg, EscTelemetry):
            erpm = _esc_erpm(msg, MOTOR_ESC_CHANNEL)
            if erpm is not None:
                state["erpm"] = erpm
                _, _m, _ = _rpm_triplet(erpm)
                if _m is not None:
                    state["mrpm_hist"].append((t_rel, _m))
                    cutoff = t_rel - mot_window_s
                    while state["mrpm_hist"] and state["mrpm_hist"][0][0] < cutoff:
                        state["mrpm_hist"].pop(0)
        elif isinstance(msg, DebugFloatArray):
            for nm, value in diag_values(msg.array_id, msg.data).items():
                state_key = _DIAG_STATE_KEYS.get(nm)
                if state_key is None:
                    continue
                state[state_key] = value
                if state_key in ("yff_t", "yff_u", "yff_gz"):
                    state[f"{state_key}_ts"] = t_rel
        elif isinstance(msg, PidTuning):
            axis = msg.axis
            prefix = {
                PidTuningAxis.ROLL: "pid_roll",
                PidTuningAxis.PITCH: "pid_pitch",
                PidTuningAxis.YAW: "pid_yaw",
            }.get(axis)
            if prefix is not None:
                state[f"{prefix}_des"] = msg.desired
                state[f"{prefix}_ach"] = msg.achieved
                state[f"{prefix}_ff"] = msg.FF
                state[f"{prefix}_p"] = msg.P
                state[f"{prefix}_i"] = msg.I
                state[f"{prefix}_d"] = msg.D
        return None

    def render_row(st, t_rel):
        def _fresh_or_stale(value, value_ts, fmt):
            if value is None:
                return None
            if value_ts is None or (t_rel - value_ts) > 2.0:
                return "stale"
            return fmt(value)

        yaw_s = f"{state['yaw']:+6.1f}" if state["yaw"] is not None else None
        yrate_s = f"{math.degrees(state['yaw_rate']):+6.1f}" if state["yaw_rate"] is not None else None
        out_s = _fresh_or_stale(state["yff_t"], state["yff_t_ts"], lambda v: f"{v:+.3f}")
        i_s = _fresh_or_stale(state["yff_u"], state["yff_u_ts"], lambda v: f"{v:+.3f}")
        mot_avg_s = None
        if state["smot_hist"]:
            mot_avg_s = f"{sum(v for _, v in state['smot_hist']) / len(state['smot_hist']):.0f}"
        _e, _m, _rotor = _rpm_triplet(state["erpm"])
        mrpm_avg_s = None
        if state["mrpm_hist"]:
            mrpm_avg_s = f"{sum(v for _, v in state['mrpm_hist']) / len(state['mrpm_hist']):.0f}"
        _, _, _, _, _qerr_deg, _qerr_yaw_deg = _quat_error(state["att_q"], state["att_target_q"])
        qerr_s = f"{_qerr_deg:+6.2f}" if _qerr_deg is not None else None
        qyaw_s = f"{_qerr_yaw_deg:+6.2f}" if _qerr_yaw_deg is not None else None
        if mode_name == "acro-manual":
            return [
                f"{t_rel:.1f}", "YES" if st["armed"] else "no",
                state["ch1"], state["ch2"], state["ch3"], state["ch4"],
                state["s1"], state["s2"], state["s3"], state["smot"],
                yaw_s, yrate_s,
            ]
        if mode_name == "passive":
            return [
                f"{t_rel:.1f}",
                "YES" if st["armed"] else "no",
                _fmt(state["roll"]),
                _fmt(state["pitch"]),
                _fmt(state["att_target_roll"]),
                _fmt(state["att_target_pitch"]),
                _fmt(state["att_target_thrust"]),
                yaw_s,
                qerr_s,
                qyaw_s,
                state["s1"],
                state["s2"],
                state["s3"],
                state["smot"],
            ]
        return [
            f"{t_rel:.1f}",
            "YES" if st["armed"] else "no",
            yaw_s,
            yrate_s,
            out_s,
            i_s,
            state["s1"], state["s2"], state["s3"], state["smot"], mot_avg_s,
            qerr_s,
            qyaw_s,
            mrpm_avg_s,
        ]

    live_view = None
    if mode_name == "passive" and not protocol_debug:
        if passive_control_handler is None or passive_reset_handler is None:
            raise RuntimeError("passive control handlers were not initialized")
        from viz3d.passive_live import PassiveLiveView, PassiveViewData
        live_view = PassiveLiveView(
            passive_control_handler,
            passive_reset_handler,
        )

        def _update_live_view(st, t_rel: float) -> bool:
            actual_q = state["att_q"] or (1.0, 0.0, 0.0, 0.0)
            _, _, _, _, qerr_deg, _ = _quat_error(
                state["att_q"], state["att_target_q"]
            )
            _, _, rotor_rpm = _rpm_triplet(state["erpm"])
            return live_view.update(PassiveViewData(
                t=t_rel,
                actual_q=actual_q,
                target_q=state["att_target_q"],
                servo_pwm=(state["s1"], state["s2"], state["s3"]),
                motor_pwm=state["smot"],
                rotor_rpm=None if rotor_rpm is None else float(rotor_rpm),
                pos_ned=state["pos_ned"],
                actual_rpy=(state["roll"], state["pitch"], state["yaw"]),
                target_rpy=(
                    state["att_target_roll"],
                    state["att_target_pitch"],
                    state["att_target_yaw"],
                ),
                target_thrust=state["att_target_thrust"],
                quaternion_error_deg=qerr_deg,
            ))
    else:
        def _update_live_view(_st, _t_rel: float) -> bool:
            return True

    if protocol_debug:
        print("  Protocol debug: text-only, no 3D renderer.")
        print("  Each key logs the pre-command actual/target quaternion,")
        print("  quaternion error, servo outputs, and every transmitted NVF.")
        print("  Space resets the relative roll/pitch/yaw offsets to zero.")
        if auto_sequence_hold_s is not None:
            print(
                f"  Automatic sequence: {len(_PASSIVE_PROTOCOL_SEQUENCE)} "
                f"steps x {auto_sequence_hold_s:.1f} s."
            )

    def _run_auto_sequence(t_rel: float) -> None:
        nonlocal auto_sequence_index, protocol_sequence
        nonlocal protocol_pending_sequence, protocol_pending_until
        nonlocal protocol_last_target_q
        if auto_sequence_hold_s is None or passive_target is None:
            return
        requested_index = min(
            int(t_rel / auto_sequence_hold_s),
            len(_PASSIVE_PROTOCOL_SEQUENCE) - 1,
        )
        if requested_index == auto_sequence_index:
            return
        label, kind, value = _PASSIVE_PROTOCOL_SEQUENCE[requested_index]
        if kind == "capture":
            actual_q = state["att_q"]
            if actual_q is None:
                return
            passive_reset_handler(actual_q)
        elif kind in ("roll", "pitch", "attitude"):
            protocol_sequence += 1
            protocol_pending_sequence = protocol_sequence
            protocol_pending_until = time.monotonic() + 1.0
            protocol_last_target_q = None
            passive_target.roll_offset_deg = value if kind == "roll" else 0.0
            passive_target.pitch_offset_deg = value if kind == "pitch" else 0.0
            _protocol_state(
                f"AUTO COMMAND #{protocol_sequence}, "
                f"step {requested_index + 1}: {label}"
            )
            for wire_name, wire_value in _passive_target_messages(passive_target):
                session.send_message(NamedValueFloat(name=wire_name, value=wire_value))
                _protocol_print(
                    f"TX #{protocol_sequence} NAMED_VALUE_FLOAT "
                    f"{wire_name}={wire_value:+.7f}"
                )
        elif kind == "collective":
            protocol_sequence += 1
            protocol_pending_sequence = protocol_sequence
            protocol_pending_until = time.monotonic() + 1.0
            protocol_last_target_q = None
            passive_target.thrust = max(
                0.0, min(1.0, auto_sequence_baseline_thrust + value)
            )
            _protocol_state(
                f"AUTO COMMAND #{protocol_sequence}, "
                f"step {requested_index + 1}: {label}"
            )
            session.send_message(
                NamedValueFloat(name="RAWES_THR", value=passive_target.thrust)
            )
            _protocol_print(
                f"TX #{protocol_sequence} NAMED_VALUE_FLOAT "
                f"RAWES_THR={passive_target.thrust:+.4f}"
            )
        auto_sequence_index = requested_index

    def _combined_loop_hook(st, t_rel: float) -> bool:
        _run_auto_sequence(t_rel)
        return _update_live_view(st, t_rel)

    def _trim_streams() -> None:
        # Runs right after the stream requests so the RC_CHANNELS disable wins
        # over the RC_CHANNELS stream (which we keep for SERVO_OUTPUT_RAW).
        # Also request the motor's ESC telemetry (bidir DShot RPM) at 5 Hz.
        _esc_name, _esc_id = _esc_telem_msg_for_channel(MOTOR_ESC_CHANNEL)
        session.send_message(CommandLong(
            target_system=session._target_system,
            target_component=session._target_component,
            command=MavCmd.SET_MESSAGE_INTERVAL,
            param1=float(_esc_id),
            param2=200000.0,
        ))  # 5 Hz

        # ATTITUDE_QUATERNION (#31) isn't guaranteed to ride along with the
        # legacy EXTRA1 REQUEST_DATA_STREAM group on every AP build, so request
        # it explicitly at the same rate as ATTITUDE -- needed for the
        # quaternion heading-lock deviation columns (mav_att_qerr_*).
        session.send_message(CommandLong(
            target_system=session._target_system,
            target_component=session._target_component,
            command=MavCmd.SET_MESSAGE_INTERVAL,
            param1=float(AttitudeQuaternion.MAVLINK_ID),
            param2=40000.0,
        ))  # 25 Hz

        if mode_name == "passive":
            session.send_message(CommandLong(
                target_system=session._target_system,
                target_component=session._target_component,
                command=MavCmd.SET_MESSAGE_INTERVAL,
                param1=float(LocalPositionNed.MAVLINK_ID),
                param2=40000.0,
            ))  # 25 Hz

        # ATTITUDE_TARGET (#83) -- the FC's telemetry echo of the active GUIDED
        # angle target -- is likewise NOT part of the EXTRA1 stream group and
        # was previously only listed in the msg_types receive filter with no
        # request ever sent for it, so it never arrived (mav_att_target_q_*/
        # mav_att_qerr_* stayed empty even while a target was actively being
        # held). Request it explicitly, same as ATTITUDE_QUATERNION above.
        session.send_message(CommandLong(
            target_system=session._target_system,
            target_component=session._target_component,
            command=MavCmd.SET_MESSAGE_INTERVAL,
            param1=float(AttitudeTarget.MAVLINK_ID),
            param2=40000.0,
        ))  # 25 Hz

        if not keep_rc:
            session.send_message(CommandLong(
                target_system=session._target_system,
                target_component=session._target_component,
                command=MavCmd.SET_MESSAGE_INTERVAL,
                param1=float(RcChannels.MAVLINK_ID),
                param2=-1.0,
            ))
            print("  Stream trim: RC_CHANNELS off, AHRS2 off, EXTENDED_STATUS 1 Hz "
                  f"(use --rc to keep RC_CHANNELS); {_esc_name} 5 Hz")
        else:
            print("  Stream trim: AHRS2 off, EXTENDED_STATUS 1 Hz (RC_CHANNELS kept); "
                  f"{_esc_name} 5 Hz")

    try:
        _observation_loop(
            session,
            duration_s=duration,
            msg_types=["ATTITUDE", "RC_CHANNELS", "SERVO_OUTPUT_RAW",
                       "ATTITUDE_TARGET", "ATTITUDE_QUATERNION",
                       "LOCAL_POSITION_NED", "PID_TUNING",
                       "HEARTBEAT", "STATUSTEXT", "BATTERY_STATUS", "SYS_STATUS",
                       "DEBUG_FLOAT_ARRAY",
                       _esc_telem_msg_for_channel(MOTOR_ESC_CHANNEL)[0]],
            streams=[
                (MavDataStream.EXTRA1,          25),
                (MavDataStream.RC_CHANNELS,     25),
                (MavDataStream.EXTENDED_STATUS, 1),
                (MavDataStream.EXTRA3,          0),
            ],
            handle_msg=handle_msg,
            render_row=render_row,
            header_cols=cols,
            header_print_cols=print_cols,
            log=log,
            print_period_s=0.25 if mode_name in ("acro-manual", "passive") else 1.0,
            on_tick=on_tick,
            suppress_status=not protocol_debug,
            key_handler=key_handler,
            setup_hook=_trim_streams,
            loop_hook=_combined_loop_hook,
            stop_requested=stop_requested,
        )
    finally:
        if live_view is not None:
            live_view.close()
    # Restore telemetry the trim disabled (best-effort; resets on FC reboot).
    if not keep_rc:
        session.send_message(CommandLong(
            target_system=session._target_system,
            target_component=session._target_component,
            command=MavCmd.SET_MESSAGE_INTERVAL,
            param1=float(RcChannels.MAVLINK_ID),
            param2=0.0,
        ))
    session.request_data_stream(MavDataStream.EXTENDED_STATUS, 2)
    session.request_data_stream(MavDataStream.EXTRA3, 2)

    # Report final H_YAW_TRIM.
    _tv = session.get_param("H_YAW_TRIM")
    if _tv is not None:
        _pwm = 1000 + float(_tv) * 1000.0
        print(f"  Yaw trim final: H_YAW_TRIM={float(_tv):.4f}  (~{_pwm:.0f} us)")


# ---------------------------------------------------------------------------
# `run <mode>` command
# ---------------------------------------------------------------------------

def _cmd_run(
    session: LinkHubClient,
    args: list[str],
    *,
    stop_requested=None,
    log_dir: str | Path | None = None,
) -> None:
    """run <mode> [--duration N] [--trim thr=V]"""
    schema = {
        "--duration":         "float",
        "--force":            "bool",    # explicitly bypass ArduPilot pre-arm checks
        "--trim":             "kv",
        "--yaw":              "float",   # passive yaw offset from Lua anchor [deg]
        "--roll":             "float",   # passive roll offset from Lua anchor [deg]
        "--pitch":            "float",   # passive pitch offset from Lua anchor [deg]
        "--rc":               "bool",    # keep RC_CHANNELS stream (mixer diagnosis)
        "--protocol-debug":   "bool",    # text-only passive command trace
        "--auto-sequence":    "bool",    # automatic passive protocol exercise
        "--step-hold":        "float",   # seconds per auto-sequence step
        "--settle-rate-deg-s": "float",  # max body rate during passive qualification
        "--settle-time":       "float",  # required continuous quiet interval
        "--settle-timeout":    "float",  # maximum qualification wait
        "--rotor-motor":       "bool",   # connect external BLDC rotor drive
        "--rotor-speed":       "int",    # initial BLDC speed percentage
        "--rotor-direction":   "str",    # cw or ccw
    }
    if not args:
        print("  Usage: run <name> [--duration N] [--trim thr=V] [--protocol-debug]")
        print("  Modes:")
        for name, cfg in _RUN_MODES.items():
            print(f"    {name:<8} -- {cfg['doc']}")
        return
    try:
        pos, flags = _parse_flags(args, schema)
    except ValueError as e:
        print(f"  Error: {e}"); return
    if len(pos) != 1:
        print("  Usage: run <name> [--duration N] [--trim thr=V] [--protocol-debug]")
        return
    name = pos[0].lower()
    cfg = _RUN_MODES.get(name)
    if cfg is None:
        print(f"  Unknown mode {name!r}  (valid: {', '.join(_RUN_MODES)})")
        return
    duration         = flags.get("--duration")
    force_arm        = bool(flags.get("--force", False))
    protocol_debug   = bool(flags.get("--protocol-debug", False))
    auto_sequence    = bool(flags.get("--auto-sequence", False))
    step_hold_s      = float(flags.get("--step-hold", 2.0))
    settle_rate_deg_s = float(flags.get(
        "--settle-rate-deg-s",
        math.degrees(_PASSIVE_SETTLE_RATE_RADS),
    ))
    settle_s = float(flags.get("--settle-time", _PASSIVE_EKF_SETTLE_S))
    settle_timeout_s = float(flags.get(
        "--settle-timeout",
        _PASSIVE_EKF_TIMEOUT_S,
    ))
    use_rotor_motor = bool(flags.get("--rotor-motor", False))
    rotor_speed = int(flags.get("--rotor-speed", 10))
    rotor_direction_name = str(flags.get("--rotor-direction", "cw")).lower()
    if auto_sequence:
        protocol_debug = True
    if protocol_debug and name != "passive":
        print("  [FAIL] --protocol-debug is only valid with run passive.")
        return
    if auto_sequence and step_hold_s <= 0.0:
        print("  [FAIL] --step-hold must be greater than zero.")
        return
    if settle_rate_deg_s <= 0.0:
        print("  [FAIL] --settle-rate-deg-s must be greater than zero.")
        return
    if settle_s <= 0.0:
        print("  [FAIL] --settle-time must be greater than zero.")
        return
    if settle_timeout_s < settle_s:
        print("  [FAIL] --settle-timeout must be at least --settle-time.")
        return
    if use_rotor_motor and name != "passive":
        print("  [FAIL] --rotor-motor is only valid with run passive.")
        return
    if ("--rotor-speed" in flags or "--rotor-direction" in flags) and not use_rotor_motor:
        print("  [FAIL] --rotor-speed/--rotor-direction require --rotor-motor.")
        return
    if not 0 <= rotor_speed <= 100:
        print("  [FAIL] --rotor-speed must be within 0..100.")
        return
    if rotor_direction_name not in ("cw", "ccw"):
        print("  [FAIL] --rotor-direction must be cw or ccw.")
        return
    if auto_sequence and duration is None:
        duration = len(_PASSIVE_PROTOCOL_SEQUENCE) * step_hold_s + 1.0
    trim             = flags.get("--trim", {}) or {}
    manual_controls = (
        {"roll": 0.0, "pitch": 0.0, "collective": 0.5}
        if cfg.get("manual_control")
        else None
    )
    passive_target = None

    # Validate trim keys: ic-seed thrust key (thr, passive only)
    allowed_trim = set(_IC_TRIM_KEYS) if cfg.get("ic_seed") else set()
    bad = [k for k in trim if k not in allowed_trim]
    if bad:
        print(f"  Unknown --trim keys: {bad}  (valid: {', '.join(sorted(allowed_trim))})")
        return

    journal_start_cursor = session.current_cursor()
    meta = {
        "verb":            "run",
        "name":            name,
        "duration_s":      duration if duration is not None else "",
        "trim_deg":        ", ".join(f"{k}={v}" for k, v in trim.items()),
        "run_start_local": datetime.now().isoformat(timespec="seconds"),
        "run_start_utc":   datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "RAWES_MODE":      cfg["rawes_mode"],
        "protocol_debug":  protocol_debug,
        "auto_sequence":   auto_sequence,
        "step_hold_s":     step_hold_s if auto_sequence else "",
        "rotor_motor":     use_rotor_motor,
        "rotor_speed":     rotor_speed if use_rotor_motor else "",
        "rotor_direction": rotor_direction_name if use_rotor_motor else "",
        "linkhub_start_cursor": journal_start_cursor,
    }
    log = _RunLog.open("run", name, meta, directory=log_dir)
    print(f"  Logging to {log.path}")
    print(f"  LinkHub journal starts at {journal_start_cursor}")
    if name == "passive":
        _configure_passive_startup_telemetry(session)

    rotor_motor = None
    done_ok = False
    try:
        if name == "passive" and not _ensure_guided_thrust_option(session):
            return
        if use_rotor_motor:
            rotor_motor = LinkHubMotorController(
                session,
                speed=rotor_speed,
                direction="F" if rotor_direction_name == "cw" else "R",
            )
            print("  Scanning for BLDC Bluetooth controller ...")
            try:
                device_name = rotor_motor.connect()
            except LinkHubError as exc:
                print(f"  [FAIL] BLDC Bluetooth connection failed: {exc}")
                return
            print(f"  [OK] BLDC Bluetooth connected: {device_name} (motor stopped)")

        if name == "passive" and not _ensure_passive_tail_setup(session):
            return

        if not session.set_param("H_SV_MAN", 0):
            print("  [FAIL] H_SV_MAN=0 was not acknowledged; refusing to run.")
            return
        print("  H_SV_MAN -> 0 (automated heli control for run)")

        _fm = cfg.get("flight_mode")
        if name == "passive":
            session.set_param("RAWES_MODE", 0)
            print("  RAWES_MODE -> 0 (runup preparation)")
            session.set_mode(1)
            print("  Flight mode -> ACRO (1) for aligned heli runup")
        else:
            session.set_param("RAWES_MODE", cfg["rawes_mode"])
            print(f"  RAWES_MODE -> {cfg['rawes_mode']} ({name} mode)")
            if _fm is not None:
                session.set_mode(_fm)
                print(f"  Flight mode -> {_COPTER_MODES.get(_fm, _fm)} ({_fm})")

        if manual_controls is not None:
            flybar = session.get_param("H_FLYBAR_MODE")
            if flybar is None or round(flybar) != 1:
                print("  [FAIL] ACRO manual requires H_FLYBAR_MODE=1")
                return
            col_expo = session.get_param("IM_ACRO_COL_EXP")
            if col_expo is None or abs(col_expo) > 1e-6:
                print("  [FAIL] ACRO manual requires IM_ACRO_COL_EXP=0 so "
                      "normalized collective matches GUIDED")
                return
            for wire_name, value in (
                ("RAWES_RLL", manual_controls["roll"]),
                ("RAWES_PIT", manual_controls["pitch"]),
                ("RAWES_COL", manual_controls["collective"]),
            ):
                session.send_message(NamedValueFloat(name=wire_name, value=value))
            print("  Manual seed: roll=+0.00 pitch=+0.00 collective=0.50")

        def seed_passive_target() -> bool:
            nonlocal passive_target
            thr = float(trim.get("thr", _PASSIVE_IC_THRUST))
            if not 0.0 <= thr <= 1.0:
                print(f"  [FAIL] Passive thrust must be within [0,1], got {thr}.")
                return False
            passive_target = _PassiveTarget(
                initial_q=_quat_from_euler_deg(0.0, 0.0, 0.0),
                thrust=thr,
                roll_offset_deg=float(flags.get("--roll", 0.0)),
                pitch_offset_deg=float(flags.get("--pitch", 0.0)),
                yaw_offset_deg=float(flags.get("--yaw", 0.0)),
            )
            print("  Seeding passive thrust and relative offsets:")
            session.send_message(NamedValueFloat(name="RAWES_THR", value=thr))
            print(f"    RAWES_THR = {thr:.3f}  (thrust [0..1])")
            for wire_name, value in _passive_target_messages(passive_target):
                session.send_message(NamedValueFloat(name=wire_name, value=value))
                print(f"    {wire_name} = {value:+.6f}")
            # thr was consumed by the IC seed -- don't re-send it via the trim block.
            trim.pop("thr", None)
            return True

    # Arm.
        if stop_requested is not None and stop_requested():
            print("  [REMOTE] stop requested before arm.")
            return
        if force_arm:
            print("  [WARN] Force-arm enabled: ArduPilot pre-arm checks are bypassed.")
        if not _arm(session, force=force_arm):
            return
        print("  [OK] Armed.")

        if name == "passive":
            if not _wait_for_passive_runup(
                session,
                stop_requested=stop_requested,
            ):
                return
            thr = float(trim.get("thr", _PASSIVE_IC_THRUST))
            if not 0.0 <= thr <= 1.0:
                print(f"  [FAIL] Passive thrust must be within [0,1], got {thr}.")
                return
            session.send_message(NamedValueFloat(name="RAWES_THR", value=thr))
            print(f"  Staging RAWES_THR = {thr:.3f} for passive hold")
            for wire_name, value in (
                ("RAWES_RLL", 0.0),
                ("RAWES_PIT", 0.0),
                ("RAWES_COL", thr),
            ):
                session.send_message(NamedValueFloat(name=wire_name, value=value))
            session.set_param("RAWES_MODE", 2)
            print(
                "  RAWES_MODE -> 2 "
                "(neutral-cyclic ACRO collective clears landed state)"
            )
            if not _wait_for_passive_land_clear(
                session,
                stop_requested=stop_requested,
            ):
                return
            if not _send_lua_command(
                session,
                CMD_ENTER_GUIDED,
                [],
                "ENTER_GUIDED (Lua captures attitude and enters GUIDED_NOGPS)",
            ):
                return
            if not _wait_for_passive_ekf_settle(
                session,
                stop_requested=stop_requested,
                timeout_s=settle_timeout_s,
                settle_s=settle_s,
                rate_limit_rads=math.radians(settle_rate_deg_s),
            ):
                return
            if not seed_passive_target():
                return
            session.set_param("RAWES_MODE", cfg["rawes_mode"])
            print(
                f"  RAWES_MODE -> {cfg['rawes_mode']} "
                "(Lua passive mode; GUIDED entry hold continues)"
            )
            if not _send_lua_command(
                session,
                CMD_ENTER_PASSIVE,
                enter_passive_params(),
                "ENTER_PASSIVE (Lua captures the passive anchor)",
            ):
                return

        _run_observation(session, name, duration, log,
                         keep_rc=bool(flags.get("--rc", False) or manual_controls),
                         manual_controls=manual_controls,
                         passive_target=passive_target,
                         rotor_motor=rotor_motor,
                         protocol_debug=protocol_debug,
                         auto_sequence_hold_s=(
                             step_hold_s if auto_sequence else None
                         ),
                         stop_requested=stop_requested)
        done_ok = True
    finally:
        try:
            try:
                if rotor_motor is not None:
                    rotor_motor.close()
                    print("  BLDC rotor motor stopped and disconnected.")
            finally:
                _safety_shutdown(
                    session,
                    skip_motor_off=(name == "passive"),
                )
        finally:
            journal_end_cursor = session.current_cursor()
            log.close()
            print(f"  Wrote {log.n_rows} rows to {log.path}")
            print(
                "  LinkHub journal range: "
                f"{journal_start_cursor}..{journal_end_cursor}"
            )
    if done_ok:
        print("  Done.")
