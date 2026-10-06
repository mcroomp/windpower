"""
calibrate/hw.py -- ESC telemetry, swashplate mix, MAVLink send helpers,
arm/disarm, monitor_esc, sweep, and status/drain.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from linkhub_client.messages import (
    EkfStatusFlags,
    MavCmd,
    MavDataStream,
    MavModeFlag,
    MavParamType,
    MavResult,
    MavSysStatusSensor,
    ParamSet,
    ServoOutputRaw,
)

from .constants import (
    LinkHubClient,
    LinkHubGenerationChanged,
    CommandAck,
    EscTelemetry,
    PidTuning,
    RequestDataStream,
    RcChannels,
    CommandLong,
    decode_message,
    Heartbeat,
    SetAttitudeTarget,
    Statustext,
    GB4008_KV, GB4008_POLE_PAIRS, GB4008_KT, GB4008_GEAR_RATIO,
    SERVO_S1, SERVO_S2, SERVO_S3, SERVO_MOTOR,
    SWASH_SERVOS,
    MOTOR_OFF_US, MOTOR_FULL_US, MOTOR_ESC_CHANNEL,
    _ESC_TELEM_MSGS,
    PWM_MIN, PWM_MAX, PWM_NEUTRAL,
    _AZ_S1, _AZ_S2, _AZ_S3,
    _COPTER_MODES, _LUA_MODES,
    _KEY_PARAM_NAMES, _TAIL_PARAM_NAMES, _MOTOR_PATH_PARAM_NAMES,
)
from .messages import read_one

# ---------------------------------------------------------------------------
# Mutable global: eRPM -> RPM divisor (pole-pairs)
# Seeded from the GB4008 default; overridden by _refresh_pole_pairs().
# ---------------------------------------------------------------------------
_motor_pole_pairs = GB4008_POLE_PAIRS

# ---------------------------------------------------------------------------
# ESC telemetry helpers
# ---------------------------------------------------------------------------

def _esc_telem_msg_for_channel(channel: int) -> "tuple[str, int]":
    """(msg_name, msg_id) of the ESC_TELEMETRY block covering 1-based `channel`."""
    base = ((channel - 1) // 4) * 4 + 1
    for name, (mid, b) in _ESC_TELEM_MSGS.items():
        if b == base:
            return name, mid
    raise ValueError(f"no ESC_TELEMETRY message for output channel {channel}")


def _esc_erpm(msg, channel: int) -> "float | None":
    """eRPM for 1-based output `channel` from an ESC_TELEMETRY_* msg, else None."""
    if isinstance(msg, EscTelemetry):
        base = msg.first_channel
        idx = channel - base
        if msg.rpm is None or not (0 <= idx < len(msg.rpm)):
            return None
        return msg.rpm[idx]
    info = _ESC_TELEM_MSGS.get(msg.get_type())
    if info is None:
        return None
    _mid, base = info
    idx = channel - base
    rpm = getattr(msg, "rpm", None)
    if rpm is None or not (0 <= idx < len(rpm)):
        return None
    return rpm[idx]


def _rpm_triplet(erpm: "float | None") -> tuple:
    """eRPM -> (erpm, mech_rpm, rotor_rpm).  (None, None, None) if erpm is None.
    Uses the live SERVO_BLH_POLES-derived pole-pair count (_motor_pole_pairs)."""
    if erpm is None:
        return None, None, None
    mech_rpm = erpm / _motor_pole_pairs
    rotor_rpm = mech_rpm / GB4008_GEAR_RATIO
    return erpm, mech_rpm, rotor_rpm


def _refresh_pole_pairs(session: LinkHubClient) -> None:
    """Set the eRPM->RPM divisor from the FC's SERVO_BLH_POLES (poles/2), so the
    RPM readout follows the param instead of a hardcode.  Falls back to the
    GB4008 default if the param is unreadable."""
    global _motor_pole_pairs
    poles = session.get_param("SERVO_BLH_POLES")
    if poles is not None and poles >= 2:
        _motor_pole_pairs = int(round(poles)) // 2


# ---------------------------------------------------------------------------
# H3-120 forward mixer
# ---------------------------------------------------------------------------

def _h3_forward_mix(coll: float, tilt_lon: float, tilt_lat: float):
    """
    Convert collective + cyclic tilts (all normalised -1..+1) to
    individual HR3-120 servo positions (normalised -1..+1).

    Physical bench layout:
        S1 at -120 deg  (right-rear)
        S2 at +120 deg  (left-rear)
        S3 at    0 deg  (front, longitudinal axis)

    This uses the physical HR3 servo azimuths because direct MAV_CMD_DO_SET_SERVO
    commands bypass ArduPilot's H3-120 reversal-based HR3 mapping. It then applies
    the user-side sign convention (tlat > 0 = roll-right; tlon > 0 = nose-DOWN):

        AP mixer:        out = -sin(az)*roll_cmd + cos(az)*pitch_cmd + coll
        Map user input:  roll_cmd = tlat,  pitch_cmd = -tlon
        Result:          out = -sin(az)*tlat - cos(az)*tlon + coll

    For S3 at 0 deg with tlon > 0 (nose-down), its command decreases while the
    two rear commands increase.
    """
    def _mix(az):
        return coll - math.sin(az) * tilt_lat - math.cos(az) * tilt_lon
    return _mix(_AZ_S1), _mix(_AZ_S2), _mix(_AZ_S3)


def _norm_to_pwm(v: float) -> int:
    """Normalised [-1, 1] -> PWM [1000, 2000] us, clamped."""
    return int(max(PWM_MIN, min(PWM_MAX, round(PWM_NEUTRAL + v * 500.0))))


# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------

def _send_set_servo(session: LinkHubClient, instance: int, pwm: int) -> None:
    """Send MAV_CMD_DO_SET_SERVO (works while disarmed)."""
    session.send_message(CommandLong(
        target_system=session._target_system,
        target_component=session._target_component,
        command=MavCmd.DO_SET_SERVO,
        confirmation=0,
        param1=float(instance),
        param2=float(pwm),
    ))


def _send_motor_test(session: LinkHubClient, instance: int,
                     throttle_pct: float, timeout_s: float = 3.0) -> None:
    """
    Send MAV_CMD_DO_MOTOR_TEST.

    instance      : motor output number (1-indexed)
    throttle_pct  : 0-100  (MOTOR_TEST_THROTTLE_PERCENT = 0)
    timeout_s     : test duration; 0 = run until next command
    """
    session.send_message(CommandLong(
        target_system=session._target_system,
        target_component=session._target_component,
        command=MavCmd.DO_MOTOR_TEST,
        confirmation=0,
        param1=float(instance),
        param2=0.0,
        param3=float(throttle_pct),
        param4=float(timeout_s),
    ))


# ---------------------------------------------------------------------------
# Status snapshot
# ---------------------------------------------------------------------------

def _print_status(session: LinkHubClient) -> None:
    """Unified status: vehicle, battery, EKF, servo outputs, key params."""
    from .params import _config_target_params   # avoid circular at module level
    expected_params = _config_target_params(use_all=True)

    sep = "-" * 50
    cursor = session.current_cursor()

    # --- vehicle -------------------------------------------------------------
    print(f"\n{sep}")
    print("VEHICLE")
    print(sep)
    hb, cursor = read_one(session, cursor, "HEARTBEAT", wait=5.0)
    if hb is None:
        print("  (no HEARTBEAT received)")
    else:
        armed   = MavModeFlag.SAFETY_ARMED in hb.base_mode
        mode_id = hb.custom_mode
        mode    = _COPTER_MODES.get(mode_id, f"MODE_{mode_id}")
        status  = str(hb.system_status).removeprefix("MAV_STATE_")
        print(f"  Armed      : {'YES  <--' if armed else 'no'}")
        print(f"  Mode       : {mode} ({mode_id})")
        print(f"  Sys status : {status}")

    # --- battery -------------------------------------------------------------
    print(f"\n{sep}")
    print("BATTERY")
    print(sep)
    batt, cursor = read_one(session, cursor, "BATTERY_STATUS", wait=2.0)
    if batt:
        cells = [v for v in batt.voltages if v != 65535]
        total_v   = sum(cells) / 1000.0 if cells else None
        current_a = batt.current_battery / 100.0 if batt.current_battery >= 0 else None
        remaining = batt.battery_remaining
        v_str = f"{total_v:.2f} V" if total_v else "n/a"
        i_str = f"  {current_a:.2f} A" if current_a is not None else ""
        r_str = f"  {remaining}%" if remaining >= 0 else ""
        print(f"  {v_str}{i_str}{r_str}")
        if len(cells) > 1:
            print("  cells: " + "  ".join(f"{v/1000.0:.3f}V" for v in cells))
        if total_v and len(cells) >= 3 and total_v / len(cells) < 3.5:
            print(f"  [WARN] avg cell {total_v/len(cells):.3f} V -- low")
    else:
        ss, cursor = read_one(session, cursor, "SYS_STATUS", wait=1.0)
        if ss and ss.voltage_battery != 65535:
            v = ss.voltage_battery / 1000.0
            i = ss.current_battery / 100.0 if ss.current_battery >= 0 else None
            r = ss.battery_remaining
            print(f"  {v:.2f} V" + (f"  {i:.2f} A" if i is not None else "") +
                  (f"  {r}%" if r >= 0 else ""))
        else:
            print("  (no battery data)")

    # --- EKF -----------------------------------------------------------------
    print(f"\n{sep}")
    print("EKF")
    print(sep)
    ekf, cursor = read_one(session, cursor, "EKF_STATUS_REPORT", wait=2.0)
    if ekf:
        flags  = ekf.flags
        att_ok = EkfStatusFlags.ATTITUDE in flags
        vel_ok = EkfStatusFlags.VELOCITY_HORIZ in flags
        pos_ok = EkfStatusFlags.POS_HORIZ_REL in flags
        health = "OK" if (att_ok and vel_ok) else "DEGRADED"
        flag_names = ",".join(
            sorted(str(flag).removeprefix("EKF_") for flag in flags)
        ) or "none"
        print(f"  Flags: {flag_names}  att={att_ok}  vel={vel_ok}  pos_rel={pos_ok}  {health}")
    else:
        print("  (no EKF_STATUS_REPORT received)")

    # --- servo outputs -------------------------------------------------------
    print(f"\n{sep}")
    print("SERVO OUTPUTS")
    print(sep)
    session.send_message(RequestDataStream(
        target_system=session._target_system,
        target_component=session._target_component,
        req_stream_id=MavDataStream.RC_CHANNELS,
        req_message_rate=10,
    ))
    srv, cursor = read_one(session, cursor, "SERVO_OUTPUT_RAW", wait=2.0)
    if srv:
        for i in range(1, 13):
            val = getattr(srv, f"servo{i}_raw", 0)
            if not val:
                continue
            tag = {SERVO_S1: "  <- S1 (-120 deg, right-rear)",
                   SERVO_S2: "  <- S2 (+120 deg, left-rear)",
                   SERVO_S3: "  <- S3 (0 deg, front/elevator)"}.get(i, "")
            if i == SERVO_MOTOR:
                if val <= MOTOR_OFF_US:
                    tag = "  <- GB4008 off"
                else:
                    pct = (val - MOTOR_OFF_US) / (MOTOR_FULL_US - MOTOR_OFF_US) * 100
                    tag = f"  <- GB4008 {pct:.0f}%"
            print(f"  Ch {i}: {val} us{tag}")
    else:
        print("  (no SERVO_OUTPUT_RAW received)")

    # --- key params ----------------------------------------------------------
    print(f"\n{sep}")
    print("KEY PARAMS")
    print(sep)
    for name in _KEY_PARAM_NAMES:
        aliases = (name,)
        resolved_name = name
        if name == "ARMING_SKIPCHK":
            aliases = ("ARMING_SKIPCHK", "ARMING_CHECK")

        val = None
        for candidate in aliases:
            val = session.get_param(candidate)
            if val is not None:
                resolved_name = candidate
                break

        expected = expected_params.get(resolved_name)
        if val is None:
            print(f"  {name:<22} NOT FOUND")
            continue
        if name == "RAWES_MODE":
            lua_name = _LUA_MODES.get(int(val), f"mode_{int(val)}")
            print(f"  {name:<22} {val:<8.4g}  {lua_name}")
        elif expected is not None and abs(val - float(expected)) > 1e-4:
            suffix = ""
            if resolved_name != name:
                suffix = f"  ({resolved_name})"
            print(f"  {name:<22} {val:<8.4g}  [DIFF] expected {expected}{suffix}")
        else:
            suffix = ""
            if resolved_name != name:
                suffix = f"  ({resolved_name})"
            print(f"  {name:<22} {val:<8.4g}  OK{suffix}")

    ss2, cursor = read_one(session, cursor, "SYS_STATUS", wait=2.0)
    if ss2:
        present = MavSysStatusSensor.SENSOR_MOTOR_OUTPUTS in ss2.onboard_control_sensors_present
        enabled = MavSysStatusSensor.SENSOR_MOTOR_OUTPUTS in ss2.onboard_control_sensors_enabled
        healthy = MavSysStatusSensor.SENSOR_MOTOR_OUTPUTS in ss2.onboard_control_sensors_health
        health  = "OK" if healthy else "[WARN] unhealthy"
        print(f"  {'motor outputs':<22} present={present}  enabled={enabled}  {health}")
        print(f"  {'CPU load':<22} {ss2.load/10.0:.1f}%")

    # --- interlock / dshot path ---------------------------------------------
    print(f"\n{sep}")
    print("INTERLOCK / DSHOT")
    print(sep)
    for name in _MOTOR_PATH_PARAM_NAMES:
        expected = expected_params.get(name)
        val = session.get_param(name)
        if val is None:
            print(f"  {name:<22} NOT FOUND")
        elif (
            name == f"SERVO{SERVO_MOTOR}_FUNCTION"
            and round(val) == round(_SAFE_OFF_MOTOR_FUNCTION)
        ):
            print(f"  {name:<22} {val:<10.4g}  SAFE-OFF")
        elif expected is not None and abs(val - float(expected)) > 1e-4:
            print(f"  {name:<22} {val:<10.4g}  [DIFF] expected {expected}")
        else:
            print(f"  {name:<22} {val:<10.4g}")

    # --- yaw control ---------------------------------------------------------
    print(f"\n{sep}")
    print("YAW CONTROL")
    print(sep)
    for name in _TAIL_PARAM_NAMES:
        expected = expected_params.get(name)
        val = session.get_param(name)
        if val is None:
            print(f"  {name:<22} NOT FOUND")
        elif expected is not None and abs(val - float(expected)) > 1e-4:
            print(f"  {name:<22} {val:<10.4g}  [DIFF] expected {expected}")
        else:
            print(f"  {name:<22} {val:<10.4g}")

    print(f"\n{sep}")


# ---------------------------------------------------------------------------
# Drain helper
# ---------------------------------------------------------------------------

def _drain(session: LinkHubClient, msg_types, duration: float) -> list:
    """Collect all messages of given types for `duration` wall-clock seconds."""
    msgs = []
    deadline = time.monotonic() + duration
    cursor = session.current_cursor()
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg, cursor = read_one(
            session, cursor, msg_types, wait=min(0.2, remaining),
        )
        if msg:
            msgs.append(msg)
    return msgs


# ---------------------------------------------------------------------------
# Lua scripting restart
# ---------------------------------------------------------------------------

def _restart_scripting(session: LinkHubClient) -> None:
    """Restart Lua scripting engine by toggling SCR_ENABLE (no reboot needed)."""
    print("  Restarting scripting engine (SCR_ENABLE 1->0->1) ...")
    session.set_param("SCR_ENABLE", 0)
    time.sleep(0.5)
    session.set_param("SCR_ENABLE", 1)
    print("  Scripting engine restarted.")


# ---------------------------------------------------------------------------
# ESC monitor
# ---------------------------------------------------------------------------

def _monitor_esc(session: LinkHubClient, duration: float = 10.0) -> None:
    """
    Stream ESC telemetry continuously for `duration` seconds.
    """
    print(f"  Monitoring ESC telemetry for {duration:.0f} s  (Ctrl-C to stop)")
    print(f"  {'t(s)':<6} {'eRPM':<8} {'Mech RPM':<10} {'Rotor RPM':<11}"
          f" {'Current(A)':<12} {'Torque(Nm)':<12} {'Volt(V)':<9} {'Temp(C)'}")
    print(f"  {'-'*90}")

    esc_name, esc_id = _esc_telem_msg_for_channel(MOTOR_ESC_CHANNEL)
    idx = (MOTOR_ESC_CHANNEL - 1) % 4
    session.send_message(CommandLong(
        target_system=session._target_system,
        target_component=session._target_component,
        command=MavCmd.SET_MESSAGE_INTERVAL,
        param1=float(esc_id),
        param2=100000.0,
    ))  # 10 Hz
    deadline = time.monotonic() + duration
    last_print = 0.0
    cursor = session.current_cursor()
    try:
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg, cursor = read_one(
                session, cursor, esc_name, wait=min(0.5, remaining),
            )
            if msg is None:
                continue
            now = time.monotonic()
            if now - last_print < 0.25:   # print at ~4 Hz max
                continue
            last_print = now

            try:
                rpm_e    = msg.rpm[idx]
                volt     = msg.voltage[idx] / 100.0
                curr     = msg.current[idx] / 100.0
                temp     = msg.temperature[idx]
            except (IndexError, TypeError):
                continue

            mech_rpm  = rpm_e / _motor_pole_pairs
            rotor_rpm = mech_rpm / GB4008_GEAR_RATIO
            torque    = curr * GB4008_KT / GB4008_GEAR_RATIO
            elapsed   = duration - (deadline - now)
            print(f"  {elapsed:<6.1f} {rpm_e:<8} {mech_rpm:<10.0f} {rotor_rpm:<11.1f}"
                  f" {curr:<12.2f} {torque:<12.4f} {volt:<9.2f} {temp}")
    except KeyboardInterrupt:
        print("  Monitoring stopped.")


# ---------------------------------------------------------------------------
# Arm / Disarm
# ---------------------------------------------------------------------------

_SAFE_OFF_FLIGHT_MODE = 1  # ArduCopter ACRO with RC passthrough selected.
_SAFE_OFF_FLYBAR_MODE = 1.0
_SAFE_OFF_SERVO_MODE = 0.0  # Automated mixer; Lua mode 0 supplies neutral RC inputs.
_SAFE_OFF_MOTOR_FUNCTION = 36.0
_SAFE_OFF_YAW_TRIM = 0.0


@dataclass(frozen=True)
class SafeOffReport:
    armed: bool
    flight_mode: int
    rawes_mode: float | None
    flybar_mode: float | None
    servo_mode: float | None
    motor_function: float | None
    yaw_trim: float | None
    motor_output_raw: int | None
    errors: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.errors


def verify_safe_off(
    session: LinkHubClient,
    *,
    output_timeout_s: float = 1.0,
) -> SafeOffReport:
    """Read and validate the canonical disarmed hardware state."""
    status = session.vehicle_status()
    base_mode = status.get("base_mode", frozenset())
    armed = MavModeFlag.SAFETY_ARMED in base_mode
    flight_mode = int(status.get("custom_mode", -1))
    params = {
        name: session.get_param(name)
        for name in (
            "RAWES_MODE",
            "H_FLYBAR_MODE",
            "H_SV_MAN",
            f"SERVO{SERVO_MOTOR}_FUNCTION",
            "H_YAW_TRIM",
            f"SERVO{SERVO_MOTOR}_MIN",
        )
    }

    session.send_message(CommandLong(
        target_system=session._target_system,
        target_component=session._target_component,
        command=MavCmd.SET_MESSAGE_INTERVAL,
        param1=float(ServoOutputRaw.MAVLINK_ID),
        param2=100_000.0,
    ))
    cursor = session.current_cursor()
    motor_output_raw = None
    deadline = time.monotonic() + output_timeout_s
    while motor_output_raw is None:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        batch = session.read_messages(
            cursor,
            ["SERVO_OUTPUT_RAW"],
            direction="rx",
            wait=remaining,
            limit=1,
        )
        cursor = batch.next_cursor
        if not batch.messages:
            continue
        decoded = decode_message(batch.messages[0])
        if isinstance(decoded, ServoOutputRaw):
            motor_output_raw = int(
                getattr(decoded, f"servo{SERVO_MOTOR}_raw")
            )

    errors: list[str] = []
    if armed:
        errors.append("vehicle is armed")
    if flight_mode != _SAFE_OFF_FLIGHT_MODE:
        errors.append(f"flight mode is {flight_mode}, expected ACRO (1)")

    expected_params = {
        "RAWES_MODE": 0.0,
        "H_FLYBAR_MODE": _SAFE_OFF_FLYBAR_MODE,
        "H_SV_MAN": _SAFE_OFF_SERVO_MODE,
        f"SERVO{SERVO_MOTOR}_FUNCTION": _SAFE_OFF_MOTOR_FUNCTION,
        "H_YAW_TRIM": _SAFE_OFF_YAW_TRIM,
    }
    for name, expected in expected_params.items():
        actual = params[name]
        if actual is None:
            errors.append(f"{name} is unreadable")
        elif not math.isclose(float(actual), expected, abs_tol=1e-6):
            errors.append(f"{name}={actual:.6g}, expected {expected:.6g}")

    motor_min = params[f"SERVO{SERVO_MOTOR}_MIN"]
    if motor_output_raw is None:
        errors.append("output 9 is unreadable")
    elif motor_min is None:
        errors.append(f"SERVO{SERVO_MOTOR}_MIN is unreadable")
    elif motor_output_raw not in (0, round(float(motor_min))):
        errors.append(
            f"output 9 is {motor_output_raw}, expected off "
            f"(0 or {round(float(motor_min))})"
        )

    return SafeOffReport(
        armed=armed,
        flight_mode=flight_mode,
        rawes_mode=params["RAWES_MODE"],
        flybar_mode=params["H_FLYBAR_MODE"],
        servo_mode=params["H_SV_MAN"],
        motor_function=params[f"SERVO{SERVO_MOTOR}_FUNCTION"],
        yaw_trim=params["H_YAW_TRIM"],
        motor_output_raw=motor_output_raw,
        errors=tuple(errors),
    )


def _set_safe_off_state(
    session: LinkHubClient,
    *,
    rawes_mode_released: bool = False,
) -> SafeOffReport:
    """Apply the one canonical disarmed hardware state."""
    if rawes_mode_released:
        print("  [OK] Safe-off RAWES_MODE=0 (Lua control released).")
    else:
        try:
            if not session.set_param("RAWES_MODE", 0):
                print("  [FAIL] Safe-off RAWES_MODE=0 was not acknowledged.")
            else:
                print("  [OK] Safe-off RAWES_MODE=0 (Lua control released).")
        except Exception as e:
            print(f"  [FAIL] Could not release Lua control for safe-off: {e}")

    motor_function = f"SERVO{SERVO_MOTOR}_FUNCTION"
    try:
        configured_function = session.get_param(motor_function)
        if configured_function is None:
            print(f"  [FAIL] Could not read {motor_function} for safe-off.")
        elif round(configured_function) != round(_SAFE_OFF_MOTOR_FUNCTION):
            print(
                f"  [FAIL] Safe-off requires {motor_function}=36 at boot; "
                f"found {configured_function:.6g}. Configure it and reboot."
            )
        else:
            print(
                f"  [OK] Safe-off {motor_function}=36 "
                "(DDFP mapped; disarm holds output off)."
            )
    except Exception as e:
        print(f"  [FAIL] Could not disconnect motor output for safe-off: {e}")

    try:
        yaw_trim = session.get_param("H_YAW_TRIM")
        if yaw_trim is None:
            print("  [FAIL] Could not read H_YAW_TRIM for safe-off.")
        elif abs(yaw_trim - _SAFE_OFF_YAW_TRIM) > 1e-6:
            if not session.set_param("H_YAW_TRIM", _SAFE_OFF_YAW_TRIM):
                print("  [FAIL] Safe-off H_YAW_TRIM=0 was not acknowledged.")
            else:
                print("  [OK] Safe-off H_YAW_TRIM=0 (no stale motor trim).")
        else:
            print("  [OK] Safe-off H_YAW_TRIM=0 (no stale motor trim).")
    except Exception as e:
        print(f"  [FAIL] Could not clear yaw trim for safe-off: {e}")

    try:
        flybar_mode = session.get_param("H_FLYBAR_MODE")
        if flybar_mode is None:
            print("  [FAIL] Could not read H_FLYBAR_MODE for safe-off.")
        elif round(flybar_mode) != round(_SAFE_OFF_FLYBAR_MODE):
            if not session.set_param("H_FLYBAR_MODE", _SAFE_OFF_FLYBAR_MODE):
                print("  [FAIL] Safe-off H_FLYBAR_MODE=1 was not acknowledged.")
            else:
                print("  [OK] Safe-off H_FLYBAR_MODE=1 (ACRO RC passthrough).")
        else:
            print("  [OK] Safe-off H_FLYBAR_MODE=1 (ACRO RC passthrough).")
    except Exception as e:
        print(f"  [FAIL] Could not configure ACRO RC passthrough for safe-off: {e}")

    try:
        servo_mode = session.get_param("H_SV_MAN")
        if servo_mode is None:
            print("  [FAIL] Could not read H_SV_MAN for safe-off.")
        elif round(servo_mode) != round(_SAFE_OFF_SERVO_MODE):
            if not session.set_param("H_SV_MAN", _SAFE_OFF_SERVO_MODE):
                print("  [FAIL] Safe-off H_SV_MAN=0 was not acknowledged.")
            else:
                print("  [OK] Safe-off H_SV_MAN=0 (Lua neutral-input hold).")
        else:
            print("  [OK] Safe-off H_SV_MAN=0 (Lua neutral-input hold).")
    except Exception as e:
        print(f"  [FAIL] Could not enable automated swash control for safe-off: {e}")

    try:
        session.set_mode(_SAFE_OFF_FLIGHT_MODE)
        print("  [OK] Safe-off flight mode ACRO (Lua neutral-input hold).")
    except Exception as e:
        print(f"  [FAIL] Could not select ACRO safe-off mode: {e}")

    report = verify_safe_off(session)
    if report.ok:
        print(
            f"  [OK] Safe-off verified; output {SERVO_MOTOR}="
            f"{report.motor_output_raw}."
        )
    else:
        for error in report.errors:
            print(f"  [FAIL] Safe-off verification: {error}.")
    return report


def _arm(session: LinkHubClient, force: bool = False,
         timeout: float = 15.0) -> bool:
    """
    Arm sequence:
            1. Send MAV_CMD_COMPONENT_ARM_DISARM.
            2. Wait for armed heartbeat.
    The DShot ESC self-arms from the idle throttle once armed -- no special
    ESC pre-arm pulse is needed.  Returns True if vehicle confirms armed.
    """
    print("  Sending arm command ...")
    cursor = session.current_cursor()
    # Arming should never cause a reboot, so any generation change mid-wait
    # (service restart, serial reconnect, vehicle reboot) means something
    # unexpected happened -- abort rather than keep polling a link that is
    # no longer trustworthy.
    generation = session.generation
    param2 = 21196.0 if force else 0.0
    result = session.command(
        MavCmd.COMPONENT_ARM_DISARM,
        [1.0, param2],
        timeout=timeout,
    )
    result_code = result.get("result")
    if result_code != MavResult.ACCEPTED:
        print(f"  [FAIL] Arm rejected: {result_code}.")
        reason_deadline = time.monotonic() + 1.0
        while time.monotonic() < reason_deadline:
            msg, cursor = read_one(
                session,
                cursor,
                ["STATUSTEXT"],
                wait=min(0.2, reason_deadline - time.monotonic()),
                expected_generation=generation,
            )
            if msg is None:
                continue
            decoded = decode_message(msg)
            if isinstance(decoded, Statustext):
                print(f"  [FC] {decoded.text}")
                break
        return False
    print("  Arm command accepted -- waiting for armed heartbeat ...")

    deadline = time.monotonic() + timeout
    armed = False
    while time.monotonic() < deadline:
        try:
            msg, cursor = read_one(
                session, cursor, ["HEARTBEAT", "STATUSTEXT"], wait=0.5,
                expected_generation=generation,
            )
        except LinkHubGenerationChanged as exc:
            print(f"  [FAIL] Aborting arm: {exc}")
            return False
        if msg is None:
            continue
        match decode_message(msg):
            case Statustext(text=text):
                print(f"  [FC] {text}")
            case Heartbeat(base_mode=base_mode):
                if MavModeFlag.SAFETY_ARMED in base_mode:
                    print("  [OK] Vehicle armed.")
                    armed = True
                    break

    if not armed:
        print("  [FAIL] Arm timed out.")
        return False

    # DShot ESCs self-arm from the idle throttle ArduPilot streams once armed, so
    # no special "hold min throttle" pre-arm pulse is needed.
    return True


def _disarm(session: LinkHubClient, timeout: float = 10.0,
            force: bool = False) -> bool:
    """Send disarm command. Returns True if vehicle confirms disarmed."""
    print("  Sending disarm command ...")
    cursor = session.current_cursor()
    # Disarming does not itself trigger a reboot (that happens afterward, as a
    # separate documented step -- see _cmd_disarm), so a generation change
    # while still waiting for the disarmed heartbeat is just as unexpected as
    # during arm, and is handled the same way.
    generation = session.generation
    param2 = 21196.0 if force else 0.0
    result = session.command(
        MavCmd.COMPONENT_ARM_DISARM,
        [0.0, param2],
        timeout=timeout,
    )
    if result.get("result") != MavResult.ACCEPTED:
        print(f"  [FAIL] Disarm rejected: result={result.get('result')}")
        return False
    print("  Disarm command accepted -- waiting for disarmed heartbeat ...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            msg, cursor = read_one(
                session, cursor, ["HEARTBEAT", "STATUSTEXT"], wait=1.0,
                expected_generation=generation,
            )
        except LinkHubGenerationChanged as exc:
            print(f"  [FAIL] Aborting disarm: {exc}")
            return False
        if msg is None:
            continue
        match decode_message(msg):
            case Statustext(text=text):
                print(f"  [FC] {text}")
                continue
            case Heartbeat(base_mode=base_mode):
                if MavModeFlag.SAFETY_ARMED not in base_mode:
                    print("  [OK] Vehicle disarmed.")
                    _set_safe_off_state(session)
                    return True

    print("  [FAIL] Disarm timed out waiting for disarmed heartbeat.")
    return False


# ---------------------------------------------------------------------------
# Servo sweep
# ---------------------------------------------------------------------------

def _set_servo_function(session: LinkHubClient, output: int, value: float) -> bool:
    servo_function = f"SERVO{output}_FUNCTION"
    return session.set_param(
        servo_function,
        value,
        param_type=MavParamType.INT16,
    )


def _set_heli_servo_mode(session: LinkHubClient, value: float) -> bool:
    for _attempt in range(3):
        session.set_param(
            "H_SV_MAN",
            value,
            param_type=MavParamType.INT8,
        )
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            if session.get_param("H_SV_MAN", timeout=0.5) == value:
                return True
            time.sleep(0.1)
    return False


def _release_servo_functions(
    session: LinkHubClient, outputs: tuple[int, ...]
) -> "dict[int, float] | None":
    """Set outputs to disabled and return their original functions."""
    saved_functions: dict[int, float] = {}
    changed = False
    for output in outputs:
        servo_function = f"SERVO{output}_FUNCTION"
        saved_function = session.get_param(servo_function)
        if saved_function is None:
            print(f"  [FAIL] {servo_function} unreadable; command aborted")
            _restore_servo_functions(session, saved_functions)
            return None
        saved_functions[output] = saved_function
        if saved_function != 0:
            if not _set_servo_function(session, output, 0.0):
                print(f"  [FAIL] could not disconnect {servo_function}; command aborted")
                _restore_servo_functions(session, saved_functions)
                return None
            print(f"  {servo_function} {saved_function:.0f} -> 0 (manual passthrough)")
            changed = True
    if changed:
        time.sleep(1.2)
    return saved_functions


def _restore_servo_functions(
    session: LinkHubClient,
    saved_functions: dict[int, float],
    *,
    exclude_outputs: tuple[int, ...] = (),
) -> None:
    for output, saved_function in saved_functions.items():
        if output in exclude_outputs:
            continue
        if saved_function != 0:
            servo_function = f"SERVO{output}_FUNCTION"
            if _set_servo_function(session, output, saved_function):
                print(f"  {servo_function} restored to {saved_function:.0f}")
            else:
                print(f"  [FAIL] could not restore {servo_function} to {saved_function:.0f}")
