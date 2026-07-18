"""
calibrate/hw.py -- ESC telemetry, swashplate mix, MAVLink send helpers,
arm/disarm, probe/ping, monitor_esc, sweep, status/drain.
"""
from __future__ import annotations

import math
import time
from typing import cast

from pymavlink import mavutil
from groundstation.gcs import MavConnectionLike

from .constants import (
    RawesGCS,
    CommandAck,
    EscTelemetry,
    PidTuning,
    RequestDataStream,
    RcChannels,
    CommandLong,
    decode_message,
    Heartbeat,
    SetAttitudeTarget,
    StatusText,
    GB4008_KV, GB4008_POLE_PAIRS, GB4008_KT, GB4008_GEAR_RATIO,
    SERVO_S1, SERVO_S2, SERVO_S3, SERVO_MOTOR,
    MOTOR_OFF_US, MOTOR_FULL_US, MOTOR_ESC_CHANNEL,
    _ESC_TELEM_MSGS,
    PWM_MIN, PWM_MAX, PWM_NEUTRAL,
    _AZ_S1, _AZ_S2, _AZ_S3,
    _COPTER_MODES, _SYS_STATUS, _LUA_MODES,
    _KEY_PARAM_NAMES, _TAIL_PARAM_NAMES, _MOTOR_PATH_PARAM_NAMES,
    _FALLBACK_BAUDS,
)

# ---------------------------------------------------------------------------
# Mutable global: eRPM -> RPM divisor (pole-pairs)
# Seeded from the GB4008 default; overridden by _refresh_pole_pairs().
# ---------------------------------------------------------------------------
_motor_pole_pairs = GB4008_POLE_PAIRS

# Per-session saved SERVO{n}_FUNCTION values for release/restore.
_saved_servo_functions: dict[int, float] = {}


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


def _refresh_pole_pairs(session: RawesGCS) -> None:
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
    individual H3-120 servo positions (normalised -1..+1).

    Bench layout (must match H_SW_H3_SV*_POS on the FC -- see design/ardupilot_swashplate.md):
        S1 at -60 deg  (front-right)
        S2 at +60 deg  (front-left)
        S3 at 180 deg  (back, longitudinal axis)

    Mirrors AP's add_servo_angle() mixer, then applies the user-side sign
    convention (tlat > 0 = roll-right; tlon > 0 = nose-DOWN disk = forward
    stick = NEGATIVE pitch command in AP frame):

        AP mixer:        out = -sin(az)*roll_cmd + cos(az)*pitch_cmd + coll
        Map user input:  roll_cmd = tlat,  pitch_cmd = -tlon
        Result:          out = -sin(az)*tlat - cos(az)*tlon + coll

    For S3 at 180 deg with tlon > 0 (nose-down), -cos(180) * tlon = +tlon
    -> S3 PWM rises, matching observed flybar-passthrough behaviour.
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

def _send_set_servo(session: RawesGCS, instance: int, pwm: int) -> None:
    """Send MAV_CMD_DO_SET_SERVO (works while disarmed)."""
    session.send_message(CommandLong(
        target_system=session._target_system,
        target_component=session._target_component,
        command=mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        confirmation=0,
        param1=float(instance),
        param2=float(pwm),
    ))


def _send_motor_test(session: RawesGCS, instance: int,
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
        command=mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        confirmation=0,
        param1=float(instance),
        param2=0.0,
        param3=float(throttle_pct),
        param4=float(timeout_s),
    ))


# ---------------------------------------------------------------------------
# Status snapshot
# ---------------------------------------------------------------------------

def _print_status(session: RawesGCS) -> None:
    """Unified status: vehicle, battery, EKF, servo outputs, key params."""
    from .params import _CONFIG_TARGET_PARAMS_ALL   # avoid circular at module level

    sep = "-" * 50

    # --- vehicle -------------------------------------------------------------
    print(f"\n{sep}")
    print("VEHICLE")
    print(sep)
    hb = session._recv(type="HEARTBEAT", blocking=True, timeout=5.0)
    if hb is None:
        print("  (no HEARTBEAT received)")
    else:
        armed   = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode_id = hb.custom_mode
        mode    = _COPTER_MODES.get(mode_id, f"MODE_{mode_id}")
        status  = _SYS_STATUS.get(hb.system_status, str(hb.system_status))
        print(f"  Armed      : {'YES  <--' if armed else 'no'}")
        print(f"  Mode       : {mode} ({mode_id})")
        print(f"  Sys status : {status}")

    # --- battery -------------------------------------------------------------
    print(f"\n{sep}")
    print("BATTERY")
    print(sep)
    batt = session._recv(type="BATTERY_STATUS", blocking=True, timeout=2.0)
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
        ss = session._recv(type="SYS_STATUS", blocking=True, timeout=1.0)
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
    ekf = session._recv(type="EKF_STATUS_REPORT", blocking=True, timeout=2.0)
    if ekf:
        flags  = ekf.flags
        att_ok = bool(flags & 0x01)
        vel_ok = bool(flags & 0x02)
        pos_ok = bool(flags & 0x04)
        health = "OK" if (att_ok and vel_ok) else "DEGRADED"
        print(f"  Flags: 0x{flags:04X}  att={att_ok}  vel={vel_ok}  pos_rel={pos_ok}  {health}")
    else:
        print("  (no EKF_STATUS_REPORT received)")

    # --- servo outputs -------------------------------------------------------
    print(f"\n{sep}")
    print("SERVO OUTPUTS")
    print(sep)
    session.send_message(RequestDataStream(
        target_system=session._target_system,
        target_component=session._target_component,
        req_stream_id=mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        req_message_rate=10,
    ))
    srv = session._recv(type="SERVO_OUTPUT_RAW", blocking=True, timeout=2.0)
    if srv:
        for i in range(1, 13):
            val = getattr(srv, f"servo{i}_raw", 0)
            if not val:
                continue
            tag = {SERVO_S1: "  <- S1 (-60 deg, front-right)",
                   SERVO_S2: "  <- S2 (+60 deg, front-left)",
                   SERVO_S3: "  <- S3 (180 deg, back)"}.get(i, "")
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

        expected = _CONFIG_TARGET_PARAMS_ALL.get(resolved_name)
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

    ss2 = session._recv(type="SYS_STATUS", blocking=True, timeout=2.0)
    if ss2:
        motor_bit = 0x000200
        present = bool(ss2.onboard_control_sensors_present & motor_bit)
        enabled = bool(ss2.onboard_control_sensors_enabled & motor_bit)
        healthy = bool(ss2.onboard_control_sensors_health  & motor_bit)
        health  = "OK" if healthy else "[WARN] unhealthy"
        print(f"  {'motor outputs':<22} present={present}  enabled={enabled}  {health}")
        print(f"  {'CPU load':<22} {ss2.load/10.0:.1f}%")

    # --- interlock / dshot path ---------------------------------------------
    print(f"\n{sep}")
    print("INTERLOCK / DSHOT")
    print(sep)
    for name in _MOTOR_PATH_PARAM_NAMES:
        expected = _CONFIG_TARGET_PARAMS_ALL.get(name)
        val = session.get_param(name)
        if val is None:
            print(f"  {name:<22} NOT FOUND")
        elif expected is not None and abs(val - float(expected)) > 1e-4:
            print(f"  {name:<22} {val:<10.4g}  [DIFF] expected {expected}")
        else:
            print(f"  {name:<22} {val:<10.4g}")

    # --- yaw control ---------------------------------------------------------
    print(f"\n{sep}")
    print("YAW CONTROL")
    print(sep)
    for name in _TAIL_PARAM_NAMES:
        expected = _CONFIG_TARGET_PARAMS_ALL.get(name)
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

def _drain(session: RawesGCS, msg_types, duration: float) -> list:
    """Collect all messages of given types for `duration` wall-clock seconds."""
    msgs = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = session._recv(type=msg_types, blocking=True,
                            timeout=min(0.2, remaining))
        if msg:
            msgs.append(msg)
    return msgs


# ---------------------------------------------------------------------------
# Lua scripting restart
# ---------------------------------------------------------------------------

def _restart_scripting(session: RawesGCS) -> None:
    """Restart Lua scripting engine by toggling SCR_ENABLE (no reboot needed)."""
    print("  Restarting scripting engine (SCR_ENABLE 1->0->1) ...")
    session.set_param("SCR_ENABLE", 0)
    time.sleep(0.5)
    session.set_param("SCR_ENABLE", 1)
    print("  Scripting engine restarted.")


# ---------------------------------------------------------------------------
# COM port scanner
# ---------------------------------------------------------------------------

def _probe_port(port: str, baud: int, timeout: float) -> tuple:
    """Try one port at one baud. Returns (ok, sysid) — closes connection before returning."""
    conn: MavConnectionLike | None = None
    try:
        conn = cast(MavConnectionLike, mavutil.mavlink_connection(port, baud=baud, autoreconnect=False))
        hb = conn.wait_heartbeat(timeout=timeout)
        if hb:
            return True, conn.target_system
        return False, None
    except Exception:
        return False, None
    finally:
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass


def _ping_ports(baud: int = 115200, timeout: float = 3.0) -> list:
    """
    Enumerate all COM ports and probe each for a MAVLink HEARTBEAT.
    If the primary baud yields no heartbeat, retries with lower baud rates.
    Returns list of dicts: {port, description, ok, sysid, baud, detail}.
    """
    try:
        import serial.tools.list_ports as _list_ports
        ports = list(_list_ports.comports())
    except ImportError:
        print("  ERROR: pyserial not installed")
        return []

    if not ports:
        print("  No COM ports found.")
        return []

    fallbacks = [b for b in _FALLBACK_BAUDS if b < baud]
    all_bauds = [baud] + fallbacks
    print(f"  Scanning {len(ports)} port(s) at {baud} baud ({timeout:.0f} s each) ...")
    if fallbacks:
        print(f"  Fallback baud rates if no heartbeat: {fallbacks}")
    print()
    results = []
    for info in sorted(ports, key=lambda p: p.device):
        port = info.device
        desc = (info.description or "").strip()
        print(f"  {port:<12} {desc:<40} ", end="", flush=True)
        entry = {"port": port, "description": desc, "ok": False, "sysid": None, "baud": None, "detail": ""}
        found = False
        for try_baud in all_bauds:
            ok, sysid = _probe_port(port, try_baud, timeout)
            if ok:
                entry.update(ok=True, sysid=sysid, baud=try_baud, detail=f"sysid={sysid} baud={try_baud}")
                marker = f"({try_baud})" if try_baud != baud else ""
                print(f"[OK]  ArduPilot  sysid={sysid}  {try_baud} baud {marker}".rstrip())
                found = True
                break
            if try_baud != baud:
                print(f"\n  {port:<12} {'':40} retry {try_baud} baud ... ", end="", flush=True)
        if not found:
            tried = "/".join(str(b) for b in all_bauds)
            entry["detail"] = f"no HEARTBEAT (tried {tried})"
            print(f"[--]  no HEARTBEAT (tried {tried})")
        results.append(entry)

    print()
    ok = [r for r in results if r["ok"]]
    if ok:
        print(f"  [OK] Found {len(ok)} ArduPilot device(s):")
        for r in ok:
            print(f"       {r['port']}  {r['description']}  ({r['detail']})")
    else:
        print("  No ArduPilot devices found on any port.")
    return results


# ---------------------------------------------------------------------------
# ESC monitor
# ---------------------------------------------------------------------------

def _monitor_esc(session: RawesGCS, duration: float = 10.0) -> None:
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
        command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        param1=float(esc_id),
        param2=100000.0,
    ))  # 10 Hz
    deadline = time.monotonic() + duration
    last_print = 0.0
    try:
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg = session._recv(type=esc_name,
                                blocking=True, timeout=min(0.5, remaining))
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

def _arm(session: RawesGCS, force: bool = False,
         timeout: float = 15.0, esc_arm: bool = True) -> bool:
    """
    Arm sequence:
            1. Send MAV_CMD_COMPONENT_ARM_DISARM.
            2. Wait for armed heartbeat.
    The DShot ESC self-arms from the idle throttle once armed -- no special
    ESC pre-arm pulse is needed.  Returns True if vehicle confirms armed.
    """
    print("  Sending arm command ...")
    param2 = 21196.0 if force else 0.0
    session.send_message(CommandLong(
        target_system=session._target_system,
        target_component=session._target_component,
        command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation=0,
        param1=1.0,
        param2=param2,
    ))

    deadline = time.monotonic() + timeout
    armed = False
    while time.monotonic() < deadline:
        msg = session._recv(
            type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"],
            blocking=True, timeout=0.5,
        )
        if msg is None:
            continue
        match decode_message(msg):
            case StatusText(text=text):
                print(f"  [FC] {text}")
            case CommandAck(command=command, result=result) if command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("  Arm command accepted -- waiting for armed heartbeat ...")
                elif result == mavutil.mavlink.MAV_RESULT_DENIED:
                    print("  [FAIL] Arm denied -- check pre-arm messages above")
                    return False
            case Heartbeat(base_mode=base_mode):
                if bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    print("  [OK] Vehicle armed.")
                    armed = True
                    break

    if not armed:
        print("  [FAIL] Arm timed out.")
        return False

    # DShot ESCs self-arm from the idle throttle ArduPilot streams once armed, so
    # no special "hold min throttle" pre-arm pulse is needed.  esc_arm is accepted
    # for call-site compatibility only.
    _ = esc_arm
    return True


def _disarm(session: RawesGCS, timeout: float = 10.0,
            force: bool = False) -> bool:
    """Send disarm command. Returns True if vehicle confirms disarmed."""
    print("  Sending disarm command ...")
    param2 = 21196.0 if force else 0.0
    session.send_message(CommandLong(
        target_system=session._target_system,
        target_component=session._target_component,
        command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation=0,
        param1=0.0,
        param2=param2,
    ))
    deadline = time.monotonic() + timeout
    ack_seen = False
    while time.monotonic() < deadline:
        msg = session._recv(
            type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"],
            blocking=True, timeout=1.0,
        )
        if msg is None:
            continue
        match decode_message(msg):
            case StatusText(text=text):
                print(f"  [FC] {text}")
                continue
            case CommandAck(command=command, result=result) if command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                ack_seen = True
                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("  Disarm command accepted -- waiting for disarmed heartbeat ...")
                elif result == mavutil.mavlink.MAV_RESULT_DENIED:
                    print("  [FAIL] Disarm denied by FC.")
                    return False
                elif result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                    print("  [FAIL] Disarm temporarily rejected by FC.")
                    return False
                elif result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
                    print("  [FAIL] Disarm unsupported by FC.")
                    return False
                elif result == mavutil.mavlink.MAV_RESULT_FAILED:
                    print("  [FAIL] Disarm failed on FC.")
                    return False
                continue
            case Heartbeat(base_mode=base_mode):
                if not bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    print("  [OK] Vehicle disarmed.")
                    return True

    if ack_seen:
        print("  [FAIL] Disarm timed out waiting for disarmed heartbeat.")
    else:
        print("  [FAIL] Disarm timed out (no disarm ACK).")
    return False


# ---------------------------------------------------------------------------
# Servo sweep
# ---------------------------------------------------------------------------

def _sweep(session: RawesGCS, instance: int, step_ms: int = 5) -> None:
    """Sweep a servo from 1000 to 2000 and back, step_ms ms per PWM step."""
    print(f"  Sweeping output {instance}: 1500 -> 2000 -> 1000 -> 1500  (Ctrl-C to abort)")
    delay = step_ms / 1000.0
    try:
        for pwm in range(1500, 2001, 1):
            _send_set_servo(session, instance, pwm)
            time.sleep(delay)
        for pwm in range(2000, 999, -1):
            _send_set_servo(session, instance, pwm)
            time.sleep(delay)
        for pwm in range(1000, 1501, 1):
            _send_set_servo(session, instance, pwm)
            time.sleep(delay)
    except KeyboardInterrupt:
        _send_set_servo(session, instance, PWM_NEUTRAL)
        print("  Sweep interrupted -- servo returned to neutral")
