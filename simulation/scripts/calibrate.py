#!/usr/bin/env python3
"""
calibrate.py -- Interactive servo and motor calibration for RAWES Pixhawk 6C.

Connects to the Pixhawk over USB (or SiK radio) and provides a simple REPL
for manually setting servo PWM outputs and running motor tests WITHOUT
arming the vehicle.

Usage
-----
    python simulation/scripts/calibrate.py              # default COM4 115200
    python simulation/scripts/calibrate.py COM3
    python simulation/scripts/calibrate.py COM3 57600   # SiK radio

RAWES output channel mapping (ArduCopter Heli)
-----------------------------------------------
  Output 1  S1  (swashplate, 0 deg   / East)     SERVO1_FUNCTION = 33
  Output 2  S2  (swashplate, 120 deg)              SERVO2_FUNCTION = 34
  Output 3  S3  (swashplate, 240 deg)              SERVO3_FUNCTION = 35
  Output 9  GB4008 anti-rotation motor             SERVO9_FUNCTION = 36 (Motor4, AUX OUT 1)

PWM range: 1000 us (min) ... 1500 us (neutral) ... 2000 us (max)

Commands
--------
  servo <n> <pwm>                 Set output n (1-16) to pwm us directly
  neutral [n]                     Set output n (or all of 1-3) to 1500 us
  swash <coll_%> [lon_%] [lat_%]  Set S1/S2/S3 via H3-120 mixer
                                   coll/lon/lat are -100..+100 (% of full throw)
  motor <pct>                     Run GB4008 motor test at pct % (0-100)
  motor off                       Stop motor test
  sweep <n> [step_ms]             Slowly sweep output n: 1000->2000->1000
  status                          Print current SERVO_OUTPUT_RAW
  help                            Show this list
  quit                            Exit

Notes
-----
  MAV_CMD_DO_SET_SERVO works while DISARMED -- no arming required.
  MAV_CMD_DO_MOTOR_TEST also works while disarmed (designed for bench checks).
  Keep a safe distance from the rotor when testing the motor.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

from pymavlink import mavutil

try:
    from pymavlink import mavftp as _mavftp_mod
    _HAS_MAVFTP = True
except ImportError:
    _HAS_MAVFTP = False

# ---------------------------------------------------------------------------
# GB4008 motor constants (used in diag torque estimates)
# ---------------------------------------------------------------------------
# Confirm pole count from motor spec sheet (typical large outrunner = 14 poles)
GB4008_KV          = 66.0      # rev/min/V
GB4008_POLES       = 22        # rotor magnets: 24N22P configuration
GB4008_POLE_PAIRS  = GB4008_POLES // 2   # 11
GB4008_GEAR_RATIO  = 80.0 / 44.0  # motor shaft turns per output shaft turn

# Kt = 60 / (2*pi*Kv)  [N*m/A at motor shaft]
GB4008_KT = 60.0 / (2.0 * math.pi * GB4008_KV)  # ~0.144 N*m/A

# ---------------------------------------------------------------------------
# RAWES servo output numbers
# ---------------------------------------------------------------------------
SERVO_S1     = 1
SERVO_S2     = 2
SERVO_S3     = 3
SERVO_MOTOR         = 9   # GB4008 physical output channel — AUX OUT 1 (output 9)
MOTOR_TEST_INSTANCE = 4   # Rover logical motor instance — MOTOR_TEST_THROTTLE_RIGHT (drives k_motor4)

SWASH_SERVOS = (SERVO_S1, SERVO_S2, SERVO_S3)

# H3-120 forward mix constants (same geometry as swashplate.py)
_COS120 = math.cos(math.radians(120.0))   # -0.5
_SIN120 = math.sin(math.radians(120.0))   #  0.866
_COS240 = math.cos(math.radians(240.0))   # -0.5
_SIN240 = math.sin(math.radians(240.0))   # -0.866

PWM_MIN     = 1000
PWM_NEUTRAL = 1500
PWM_MAX     = 2000


# ---------------------------------------------------------------------------
# H3-120 forward mixer
# ---------------------------------------------------------------------------

def _h3_forward_mix(coll: float, tilt_lon: float, tilt_lat: float):
    """
    Convert collective + cyclic tilts (all normalised -1..+1) to
    individual H3-120 servo positions (normalised -1..+1).

    S1 at 0 deg, S2 at 120 deg, S3 at 240 deg.
    """
    s1 = coll + tilt_lat
    s2 = coll + tilt_lat * _COS120 + tilt_lon * _SIN120
    s3 = coll + tilt_lat * _COS240 + tilt_lon * _SIN240
    return s1, s2, s3


def _norm_to_pwm(v: float) -> int:
    """Normalised [-1, 1] -> PWM [1000, 2000] us, clamped."""
    return int(max(PWM_MIN, min(PWM_MAX, round(PWM_NEUTRAL + v * 500.0))))


# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------

def _send_set_servo(mav, instance: int, pwm: int) -> None:
    """Send MAV_CMD_DO_SET_SERVO (works while disarmed)."""
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,              # confirmation
        float(instance),
        float(pwm),
        0, 0, 0, 0, 0,
    )


def _send_motor_test(mav, instance: int, throttle_pct: float, timeout_s: float = 3.0) -> None:
    """
    Send MAV_CMD_DO_MOTOR_TEST.

    instance      : motor output number (1-indexed)
    throttle_pct  : 0-100  (MOTOR_TEST_THROTTLE_PERCENT = 0)
    timeout_s     : test duration; 0 = run until next command
    """
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        float(instance),      # param1: motor instance
        0.0,                  # param2: throttle type 0 = PERCENT
        float(throttle_pct),  # param3: throttle value
        float(timeout_s),     # param4: test duration [s]
        0, 0, 0,
    )


_COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
    13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE",
    18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
}

_SYS_STATUS = {0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY",
               4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF"}


def _get_mav_type(mav) -> int:
    """Return MAV_TYPE from the next HEARTBEAT (0 if none received)."""
    hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=5.0)
    return hb.type if hb is not None else 0


def _is_rover(mav) -> bool:
    return _get_mav_type(mav) == mavutil.mavlink.MAV_TYPE_GROUND_ROVER


def _print_vehicle_status(mav) -> None:
    """Print armed state, flight mode, EKF health, and battery from live MAVLink."""
    hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=5.0)
    if hb is None:
        print("  No HEARTBEAT received"); return

    armed   = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    mode_id = hb.custom_mode
    mode    = _COPTER_MODES.get(mode_id, f"MODE_{mode_id}")
    status  = _SYS_STATUS.get(hb.system_status, str(hb.system_status))

    print(f"  Armed        : {'YES  <--' if armed else 'no'}")
    print(f"  Mode         : {mode} ({mode_id})")
    print(f"  Sys status   : {status}")

    sys_status = mav.recv_match(type="SYS_STATUS", blocking=True, timeout=2.0)
    if sys_status:
        v = sys_status.voltage_battery / 1000.0 if sys_status.voltage_battery != 65535 else None
        i = sys_status.current_battery / 100.0  if sys_status.current_battery >= 0    else None
        r = sys_status.battery_remaining
        batt = f"{v:.2f} V" if v else "n/a"
        if i is not None and i >= 0:
            batt += f"  {i:.2f} A"
        if r >= 0:
            batt += f"  {r}%"
        print(f"  Battery      : {batt}")

    ekf = mav.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2.0)
    if ekf:
        flags   = ekf.flags
        att_ok  = bool(flags & 0x01)
        vel_ok  = bool(flags & 0x02)
        pos_ok  = bool(flags & 0x04)
        healthy = att_ok and vel_ok
        print(f"  EKF flags    : 0x{flags:04X}  att={att_ok}  vel={vel_ok}  pos_rel={pos_ok}  {'OK' if healthy else 'DEGRADED'}")


def _print_status(mav) -> None:
    """Print the most recent SERVO_OUTPUT_RAW message."""
    msg = mav.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=2.0)
    if msg is None:
        print("  (no SERVO_OUTPUT_RAW received)")
        return
    print(f"  {'Ch':<4} {'PWM (us)':<10}")
    print(f"  {'-'*14}")
    for i in range(1, 9):
        val = getattr(msg, f"servo{i}_raw", 0)
        if val:
            tag = ""
            if i == SERVO_S1:    tag = "  <- S1 (0 deg)"
            elif i == SERVO_S2:  tag = "  <- S2 (120 deg)"
            elif i == SERVO_S3:  tag = "  <- S3 (240 deg)"
            elif i == SERVO_MOTOR: tag = "  <- GB4008 motor"
            print(f"  {i:<4} {val:<10}{tag}")


# ---------------------------------------------------------------------------
# Parameter set helper
# ---------------------------------------------------------------------------

def _set_param(mav, name: str, value: float, timeout: float = 5.0) -> bool:
    """Set a parameter and wait for ACK. Returns True on success."""
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        name.encode(),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg and msg.param_id.rstrip("\x00") == name:
            return True
    return False


# ---------------------------------------------------------------------------
# Lua script upload / management
# ---------------------------------------------------------------------------

SCRIPTS_DIR = "/APM/scripts"


def _restart_scripting(mav) -> None:
    """Restart Lua scripting engine by toggling SCR_ENABLE (no reboot needed)."""
    print("  Restarting scripting engine (SCR_ENABLE 1->0->1) ...")
    _set_param(mav, "SCR_ENABLE", 0)
    time.sleep(0.5)
    _set_param(mav, "SCR_ENABLE", 1)
    print("  Scripting engine restarted.")


def _list_scripts(mav) -> None:
    """List files in /APM/scripts via MAVLink FTP."""
    if not _HAS_MAVFTP:
        print("  ERROR: pymavlink.mavftp not available -- upgrade pymavlink")
        return
    print(f"  Listing {SCRIPTS_DIR} ...")
    try:
        ftp = _mavftp_mod.MAVFTP(
            mav,
            target_system=mav.target_system,
            target_component=mav.target_component,
        )
        result = ftp.cmd_list([SCRIPTS_DIR])
        if ftp.list_result:
            for entry in ftp.list_result:
                if entry.is_dir:
                    print(f"    D {entry.name}/")
                else:
                    print(f"    F {entry.name}  ({entry.size_b} bytes)")
        else:
            print(f"  (no files found or directory does not exist; result={result})")
    except Exception as exc:
        print(f"  FTP list failed: {exc}")


def _remove_script(mav, filename: str) -> None:
    """Remove a single file from /APM/scripts via MAVLink FTP."""
    if not _HAS_MAVFTP:
        print("  ERROR: pymavlink.mavftp not available -- upgrade pymavlink")
        return
    remote = f"{SCRIPTS_DIR}/{os.path.basename(filename)}"
    print(f"  Removing {remote} ...")
    try:
        ftp = _mavftp_mod.MAVFTP(
            mav,
            target_system=mav.target_system,
            target_component=mav.target_component,
        )
        result = ftp.cmd_rm([remote])
        if result.error_code == 0:
            print(f"  [OK] Removed.")
        else:
            print(f"  [FAIL] {result}")
    except Exception as exc:
        print(f"  FTP operation failed: {exc}")


def _upload_script(mav, local_path: str, restart: bool = True) -> None:
    """
    Upload a Lua script to /APM/scripts/ via MAVLink FTP.

    local_path  : path to .lua file on this machine
    restart     : if True, toggle SCR_ENABLE after upload to reload scripts
    """
    if not _HAS_MAVFTP:
        print("  ERROR: pymavlink.mavftp not available -- upgrade pymavlink")
        print("  Alternative: use Mission Planner -> Config -> MAVFtp")
        return
    if not os.path.isfile(local_path):
        print(f"  ERROR: file not found: {local_path}")
        return

    remote_path = f"{SCRIPTS_DIR}/{os.path.basename(local_path)}"
    print(f"  Uploading {local_path}")
    print(f"         -> {remote_path} ...")

    for attempt in range(1, 4):
        try:
            time.sleep(1.0)  # let connection settle before FTP
            ftp = _mavftp_mod.MAVFTP(
                mav,
                target_system=mav.target_system,
                target_component=mav.target_component,
            )
            put_ret = ftp.cmd_put([local_path, remote_path])
            if put_ret.error_code != 0:
                print(f"  Attempt {attempt}: cmd_put rejected: {put_ret}")
                continue
            # Pump the message loop until all write blocks are ACKed and the
            # session is terminated.  timeout must be > idle_detection_time (3.7 s).
            result = ftp.process_ftp_reply('CreateFile', timeout=30)
            if result.error_code == 0:
                print(f"  Upload OK (attempt {attempt}).")
                break
            print(f"  Attempt {attempt}: transfer incomplete: {result}")
        except Exception as exc:
            print(f"  Attempt {attempt} failed: {exc}")
    else:
        print("  WARNING: upload may not have completed -- verify with Mission Planner MAVFtp.")
        return

    if restart:
        _restart_scripting(mav)


# ---------------------------------------------------------------------------
# Motor / ESC diagnostic helpers
# ---------------------------------------------------------------------------

# Parameters that must be correct for DShot to reach the motor
_DSHOT_PARAMS = [
    ("SERVO9_FUNCTION",    "Must be 36 (Motor4) for GB4008 on AUX OUT 1 (output 9)"),
    ("SERVO_BLH_MASK",     "Must be 256 (bit 8) to enable DShot on output 9 (AUX OUT 1)"),
    ("SERVO_BLH_OTYPE",    "DShot type: 5=DShot300  6=DShot600  (0=PWM, won't work)"),
    ("SERVO_BLH_AUTO",     "1 = auto-configure BLHeli outputs"),
    ("H_TAIL_TYPE",        "Must be 4 (DDFP) for GB4008"),
    ("H_RSC_MODE",         "RSC mode: 0=servo  3=setpoint  4=passthrough  (0 means motor may be gated)"),
    ("H_RSC_SETPOINT",     "RSC setpoint [0-100%] when H_RSC_MODE=3"),
    ("H_RSC_RUNUP_TIME",   "Motor runup time [s] -- motor won't respond until elapsed"),
    ("BRD_SAFETYENABLE",   "1=safety switch required  0=disabled  (if 1, press safety before any output)"),
    ("ARMING_SKIPCHK",     "Skip arming checks flag"),
    ("SERVO_BLH_TELE_PORT","UART port for KISS telemetry wire (-1=disabled)"),
    ("RPM1_TYPE",          "RPM sensor type: 5=ESC telemetry"),
    ("RPM1_MIN",           "Minimum RPM to report"),
]


def _recv_param(mav, name: str, timeout: float = 3.0):
    """Read a single parameter. Returns float or None."""
    mav.mav.param_request_read_send(
        mav.target_system, mav.target_component,
        name.encode(), -1,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip("\x00") == name:
            return float(msg.param_value)
    return None


def _drain(mav, msg_types, duration: float) -> list:
    """Collect all messages of given types for `duration` seconds."""
    msgs = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = mav.recv_match(type=msg_types, blocking=True,
                             timeout=min(0.2, remaining))
        if msg:
            msgs.append(msg)
    return msgs


def _diag_motor(mav) -> None:
    """
    Full motor / AM32 / DShot diagnostic dump.

    Checks every layer from ArduPilot parameter configuration through to
    live ESC telemetry.  Reports a likely cause for each failure condition.
    """
    sep = "-" * 60

    # ── 1. Parameter audit ──────────────────────────────────────────────
    print(f"\n{sep}")
    print("1. PARAMETER AUDIT  (DShot / motor chain)")
    print(sep)
    params = {}
    for name, note in _DSHOT_PARAMS:
        val = _recv_param(mav, name)
        params[name] = val
        val_str = f"{val:.0f}" if val is not None else "NOT FOUND"
        print(f"  {name:<26} = {val_str:<8}  ({note})")

    # Highlight likely misconfigurations
    print()
    issues = []
    v = params.get
    if v("SERVO9_FUNCTION") not in (36.0, 38.0):
        issues.append("SERVO9_FUNCTION is not 36 (Motor4) -- output 9 not mapped to GB4008")
    blh_mask = v("SERVO_BLH_MASK")
    if blh_mask is not None and not (int(blh_mask) & 256):
        issues.append("SERVO_BLH_MASK does not include bit 8 (=256) -- DShot not enabled on output 9 (AUX OUT 1)")
    blh_type = v("SERVO_BLH_OTYPE")
    if blh_type is not None and blh_type < 4:
        issues.append(f"SERVO_BLH_OTYPE={blh_type:.0f} -- not a DShot type (need 5=DShot300 or 6=DShot600)")
    if v("H_TAIL_TYPE") != 4.0:
        issues.append(f"H_TAIL_TYPE={v('H_TAIL_TYPE')} -- expected 4 (DDFP); motor may not be driven")
    if v("BRD_SAFETYENABLE") == 1.0:
        issues.append("BRD_SAFETYENABLE=1 -- safety switch must be pressed before outputs are active")
    if v("H_RSC_MODE") == 0.0:
        issues.append("H_RSC_MODE=0 (servo PWM) -- motor output gated by RSC; won't run via motor test")

    if issues:
        print("  [WARN] Likely configuration issues:")
        for iss in issues:
            print(f"    * {iss}")
    else:
        print("  [OK] No obvious parameter misconfigurations found")

    # ── 2. Safety switch / arming state ─────────────────────────────────
    print(f"\n{sep}")
    print("2. SAFETY SWITCH / ARMING STATE")
    print(sep)
    hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=3.0)
    if hb:
        armed   = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        enabled = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY)
        print(f"  Armed         : {armed}")
        print(f"  Safety active : {enabled}")
        print(f"  Custom mode   : {hb.custom_mode}  (1=ACRO)")
        print(f"  System status : {hb.system_status}  (3=STANDBY  4=ACTIVE)")
    else:
        print("  (no HEARTBEAT received)")

    # ── 3. Servo output snapshot ─────────────────────────────────────────
    print(f"\n{sep}")
    print("3. SERVO OUTPUT RAW  (what ArduPilot is sending to the ESC)")
    print(sep)
    srv = mav.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=2.0)
    if srv:
        for i in range(1, 9):
            val = getattr(srv, f"servo{i}_raw", 0)
            if val:
                note = ""
                if i == 4:
                    if val == 0:
                        note = "  <-- ZERO: ESC receiving no signal"
                    elif val < 1100:
                        note = "  <-- below DShot min (ESC may not arm)"
                    else:
                        note = f"  <-- GB4008 motor ({(val-1000)/10:.0f}% throttle range)"
                print(f"  Output {i}: {val} us{note}")
    else:
        print("  (no SERVO_OUTPUT_RAW received -- check data stream rate)")

    # ── 4. Motor test + ACK ──────────────────────────────────────────────
    print(f"\n{sep}")
    print("4. MOTOR TEST COMMAND ACK  (5% throttle, 3 s)")
    print(sep)
    print("  Sending MAV_CMD_DO_MOTOR_TEST at 5% ...")
    _send_motor_test(mav, MOTOR_TEST_INSTANCE, 5.0, timeout_s=3.0)
    ack = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=3.0)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
        result_names = {0: "ACCEPTED", 1: "TEMP_REJECTED", 2: "DENIED",
                        3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}
        rname = result_names.get(ack.result, f"code={ack.result}")
        print(f"  ACK result: {rname}")
        if ack.result != 0:
            print("  [WARN] Motor test not accepted -- check arming state or H_RSC_MODE")
        else:
            print("  Command accepted -- listening for ESC telemetry for 3 s ...")
    else:
        print("  (no COMMAND_ACK received within 3 s)")

    # ── 5. ESC telemetry ─────────────────────────────────────────────────
    print(f"\n{sep}")
    print("5. ESC TELEMETRY  (listening 3 s for ESC_TELEMETRY_1_TO_4 and RPM)")
    print(sep)
    msgs = _drain(mav, ["ESC_TELEMETRY_1_TO_4", "ESC_TELEMETRY_5_TO_8",
                         "RPM", "ESC_INFO"], duration=3.0)
    esc_msgs    = [m for m in msgs if "ESC_TELEMETRY" in m.get_type()]
    rpm_msgs    = [m for m in msgs if m.get_type() == "RPM"]
    info_msgs   = [m for m in msgs if m.get_type() == "ESC_INFO"]

    if esc_msgs:
        last = esc_msgs[-1]
        print(f"  ESC telemetry received ({len(esc_msgs)} msgs in 3 s)")
        # ESC_TELEMETRY_1_TO_4 fields are arrays of 4
        for i in range(4):
            try:
                rpm_e  = last.rpm[i]
                volt   = last.voltage[i] / 100.0
                curr   = last.current[i] / 100.0
                temp   = last.temperature[i]
                if rpm_e == 0 and volt == 0 and curr == 0:
                    continue
                mech_rpm   = rpm_e / GB4008_POLE_PAIRS if rpm_e else 0
                rotor_rpm  = mech_rpm / GB4008_GEAR_RATIO
                torque_nm  = curr * GB4008_KT / GB4008_GEAR_RATIO
                print(f"  ESC {i+1}:")
                print(f"    eRPM       = {rpm_e}  -> motor {mech_rpm:.0f} RPM"
                      f"  -> rotor {rotor_rpm:.1f} RPM")
                print(f"    Voltage    = {volt:.2f} V")
                print(f"    Current    = {curr:.2f} A"
                      f"  -> shaft torque ~{torque_nm:.3f} N*m")
                print(f"    Temp (ESC) = {temp} deg C")
            except (IndexError, TypeError):
                pass
        if all(
            (esc_msgs[-1].rpm[i] == 0 and esc_msgs[-1].current[i] == 0)
            for i in range(4)
            if esc_msgs[-1].voltage[i] != 0
        ):
            print("  [WARN] Telemetry received but RPM=0 and current=0")
            print("         Motor is not spinning despite ESC communication OK")
            print("         Check: DShot signal reaching ESC? ESC armed?")
    else:
        print("  [WARN] No ESC telemetry received")
        print("         Check: SERVO_BLH_MASK=256 (output 9 AUX OUT 1), ESC powered,")
        print("         DShot signal wire on AUX OUT 1, and AM32 Extended Telemetry enabled")
        print("         Note: bidirectional DShot is auto-detected in ArduPilot 4.6+")
        print("         (SERVO_BLH_BDSHOT parameter does not exist in this firmware)")

    if rpm_msgs:
        last_rpm = rpm_msgs[-1]
        print(f"  RPM sensor: rpm1={last_rpm.rpm1:.0f}  rpm2={last_rpm.rpm2:.0f}")
    else:
        print("  RPM message: none (RPM1_TYPE may not be set to 5)")

    if info_msgs:
        last_info = info_msgs[-1]
        print(f"  ESC_INFO: error_count={last_info.error_count}"
              f"  failure_flags=0x{last_info.failure_flags:04X}")

    # Stop motor test
    _send_motor_test(mav, MOTOR_TEST_INSTANCE, 0.0, timeout_s=0.0)

    # ── 6. STATUSTEXT drain ──────────────────────────────────────────────
    print(f"\n{sep}")
    print("6. STATUSTEXT  (error/warning messages from ArduPilot, last 3 s)")
    print(sep)
    status_msgs = _drain(mav, ["STATUSTEXT"], duration=3.0)
    if status_msgs:
        for m in status_msgs:
            sev_names = {0:"EMERGENCY", 1:"ALERT", 2:"CRITICAL", 3:"ERROR",
                         4:"WARNING",   5:"NOTICE", 6:"INFO",    7:"DEBUG"}
            sev = sev_names.get(m.severity, str(m.severity))
            print(f"  [{sev}] {m.text.rstrip(chr(0)).strip()}")
    else:
        print("  (none)")

    # ── 7. SYS_STATUS motor health ───────────────────────────────────────
    print(f"\n{sep}")
    print("7. SYS_STATUS  (motor outputs health)")
    print(sep)
    ss = mav.recv_match(type="SYS_STATUS", blocking=True, timeout=2.0)
    if ss:
        motor_bit = 0x000200
        present = bool(ss.onboard_control_sensors_present & motor_bit)
        enabled = bool(ss.onboard_control_sensors_enabled & motor_bit)
        healthy = bool(ss.onboard_control_sensors_health  & motor_bit)
        print(f"  Motor outputs: present={present}  enabled={enabled}  healthy={healthy}")
        if not healthy:
            print("  [WARN] Motor outputs NOT healthy -- ESC communication failure")
        print(f"  CPU load: {ss.load/10.0:.1f}%  Voltage: {ss.voltage_battery/1000.0:.2f} V")
    else:
        print("  (no SYS_STATUS received)")

    # ── 8. Battery status ────────────────────────────────────────────────
    print(f"\n{sep}")
    print("8. BATTERY STATUS")
    print(sep)
    batt = mav.recv_match(type="BATTERY_STATUS", blocking=True, timeout=2.0)
    if batt:
        # voltages[] is per-cell in mV; UINT16_MAX (65535) = not populated
        cells = [v for v in batt.voltages if v != 65535]
        total_v = sum(cells) / 1000.0 if cells else None
        current_a = batt.current_battery / 100.0 if batt.current_battery >= 0 else None
        remaining = batt.battery_remaining
        consumed_mah = batt.current_consumed if batt.current_consumed >= 0 else None
        print(f"  Voltage      : {total_v:.2f} V" if total_v else "  Voltage      : n/a")
        if len(cells) > 1:
            cell_str = "  ".join(f"{v/1000.0:.3f}V" for v in cells)
            print(f"  Cells ({len(cells)}S)   : {cell_str}")
        print(f"  Current      : {current_a:.2f} A" if current_a is not None else "  Current      : n/a")
        print(f"  Remaining    : {remaining} %" if remaining >= 0 else "  Remaining    : n/a")
        if consumed_mah is not None:
            print(f"  Consumed     : {consumed_mah} mAh")
        if total_v and len(cells) >= 3:
            cell_avg = total_v / len(cells)
            if cell_avg < 3.5:
                print(f"  [WARN] Average cell voltage {cell_avg:.3f} V -- battery low")
    else:
        ss = mav.recv_match(type="SYS_STATUS", blocking=True, timeout=1.0)
        if ss and ss.voltage_battery != 65535:
            v = ss.voltage_battery / 1000.0
            i = ss.current_battery / 100.0 if ss.current_battery >= 0 else None
            r = ss.battery_remaining
            print(f"  Voltage      : {v:.2f} V")
            print(f"  Current      : {i:.2f} A" if i is not None else "  Current      : n/a")
            print(f"  Remaining    : {r} %" if r >= 0 else "  Remaining    : n/a")
        else:
            print("  (no battery data received)")

    print(f"\n{sep}")
    print("Diagnostic complete.")
    print(sep)


def _dshot_blheli_test(mav, motors: list[int] | None = None,
                       timeout_per_motor: float = 25.0) -> None:
    """
    Run SERVO_BLH_TEST on each motor and interpret the results.

    What this does under the hood
    ──────────────────────────────
    ArduPilot's AP_BLHeli::run_connection_test() temporarily repurposes the
    DShot signal wire as a 19200-baud serial line and sends the BLHeli
    bootloader handshake sequence.  For ARM ESCs (BLHeli32 / AM32) it then
    reads the esc_status struct at flash address 0xEB00 which contains:
        - protocol   (0=none  1=PWM  2=OneShot125  5=DShot)
        - good_frames / bad_frames   (since last power cycle)

    All intermediate results are sent as STATUSTEXT with prefix "ESC: " when
    SERVO_BLH_DEBUG=1 (which this function sets and then restores).

    motors
        List of 1-indexed output channel numbers to test.
        If None, derived automatically from the SERVO_BLH_MASK bitmask.
    timeout_per_motor
        Maximum seconds to wait for a single motor test to complete.
        Each test can take up to 5 × 3 s = 15 s in the worst case.
    """
    sep = "-" * 60

    # ── 0. Work out which motors to test ─────────────────────────────────
    # SERVO_BLH_TEST takes a 1-indexed position within the BLHeli-registered
    # channel list (ordered by bit position in SERVO_BLH_MASK), NOT the raw
    # output number.  E.g. SERVO_BLH_MASK=8 (only output 4) → motor index 1.
    if motors is None:
        mask_val = _recv_param(mav, "SERVO_BLH_MASK")
        auto_val = _recv_param(mav, "SERVO_BLH_AUTO")
        if mask_val is not None and int(mask_val) != 0:
            # Build output→blheli_index mapping (1-indexed)
            output_channels = [i + 1 for i in range(32) if int(mask_val) & (1 << i)]
            # SERVO_BLH_TEST wants the 1-based index within registered channels
            motors = list(range(1, len(output_channels) + 1))
            # Keep display mapping handy
            _blh_channel_map = {blh_idx: out for blh_idx, out in
                                enumerate(output_channels, start=1)}
        elif auto_val is not None and int(auto_val) == 1:
            print("  SERVO_BLH_MASK=0 but SERVO_BLH_AUTO=1 -- testing motor 1")
            motors = [1]
            _blh_channel_map = {1: SERVO_MOTOR}
        else:
            print("  [WARN] SERVO_BLH_MASK=0 and SERVO_BLH_AUTO=0")
            print("         No BLHeli channels are registered.")
            print("         Set SERVO_BLH_MASK or SERVO_BLH_AUTO=1 first.")
            return
    else:
        _blh_channel_map = {m: m for m in motors}  # user passed indices directly

    print(f"\n{sep}")
    print(f"SERVO_BLH_TEST  --  DShot / AM32 bootloader connection test")
    motor_desc = ", ".join(
        f"BLH#{m}->OUT{_blh_channel_map.get(m, '?')}" for m in motors)
    print(f"Motors to test: {motor_desc}")
    print(sep)

    # ── 1. Enable verbose debug output ───────────────────────────────────
    saved_debug = _recv_param(mav, "SERVO_BLH_DEBUG") or 0.0
    if not _set_param(mav, "SERVO_BLH_DEBUG", 1):
        print("  [WARN] Could not set SERVO_BLH_DEBUG=1 -- "
              "esc_status detail will be missing")

    results: dict[int, dict] = {}

    for motor_num in motors:
        out_ch = _blh_channel_map.get(motor_num, motor_num)
        print(f"\n  Testing BLHeli motor {motor_num} (output channel {out_ch}) ...")

        # Trigger the test.  SERVO_BLH_TEST is 1-indexed and self-clears.
        if not _set_param(mav, "SERVO_BLH_TEST", float(motor_num)):
            print(f"  [FAIL] Could not set SERVO_BLH_TEST={motor_num}")
            results[motor_num] = {"passed": False, "error": "param set failed"}
            continue

        # Collect STATUSTEXT until we see the final "Test PASSED/FAILED"
        # or the timeout expires.
        collected: list[str] = []
        deadline = time.monotonic() + timeout_per_motor
        final_verdict = None

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg = mav.recv_match(type="STATUSTEXT", blocking=True,
                                 timeout=min(0.3, remaining))
            if msg is None:
                continue
            text = msg.text.rstrip("\x00").strip()
            if not text.startswith("ESC:"):
                continue
            collected.append(text)
            if "Test PASSED" in text:
                final_verdict = "PASSED"
                break
            if "Test FAILED" in text:
                final_verdict = "FAILED"
                break

        # ── Parse the collected messages ──────────────────────────────────
        iface_type   = None   # imATM_BLB / imSIL_BLB / imARM_BLB
        protocol     = None   # int: 0/1/2/5
        good_frames  = None
        bad_frames   = None
        connect_chan = None

        for line in collected:
            # "ESC: BL_ConnectEx 0/1 at 3"
            if "BL_ConnectEx" in line:
                parts = line.split()
                for j, p in enumerate(parts):
                    if "/" in p:
                        connect_chan = p   # e.g. "0/1"
            # "ESC: Interface type imARM_BLB"
            if "Interface type" in line:
                iface_type = line.split("Interface type")[-1].strip()
            # "ESC: Prot 5 Good 12483 Bad 0 ..."
            if line.startswith("ESC: Prot"):
                try:
                    tokens = line.split()
                    protocol    = int(tokens[2])
                    good_frames = int(tokens[4])
                    bad_frames  = int(tokens[6])
                except (IndexError, ValueError):
                    pass

        # ── Interpret and display ─────────────────────────────────────────
        PROTO_NAMES = {0: "NONE", 1: "PWM", 2: "OneShot125",
                       5: "DShot", 99: "unknown"}

        if final_verdict is None:
            verdict_str = "TIMEOUT (no response within "
            verdict_str += f"{timeout_per_motor:.0f} s)"
            passed = False
        else:
            verdict_str = final_verdict
            passed = (final_verdict == "PASSED")

        results[motor_num] = {
            "passed":      passed,
            "verdict":     verdict_str,
            "iface":       iface_type,
            "protocol":    protocol,
            "good_frames": good_frames,
            "bad_frames":  bad_frames,
            "raw":         collected,
        }

        print(f"  Result       : {verdict_str}")
        if iface_type:
            print(f"  MCU type     : {iface_type}")
        if protocol is not None:
            pname = PROTO_NAMES.get(protocol, f"unknown ({protocol})")
            print(f"  ESC protocol : {protocol} = {pname}")
            if protocol != 5:
                print(f"  [WARN] ESC is not in DShot mode!")
                print(f"         Check MOT_PWM_TYPE (Rover) / SERVO_BLH_OTYPE (Heli)")
        if good_frames is not None:
            print(f"  Good frames  : {good_frames}")
            print(f"  Bad frames   : {bad_frames}")
            if good_frames == 0:
                print("  [WARN] ESC has received ZERO good DShot frames")
                print("         DShot signal is not arriving at the ESC")
                print("         Check: output channel, wiring, timer conflict")
            elif bad_frames is not None and bad_frames > 0:
                ratio = bad_frames / max(1, good_frames + bad_frames)
                print(f"  [WARN] {ratio*100:.1f}% frame error rate -- "
                      f"signal integrity problem (noise / marginal timing)")
            else:
                print("  [OK] Clean DShot frames -- signal path is good")
                if not passed:
                    print("       ESC receives frames but motor won't spin:")
                    print("       Check arming state, MOT_SPIN_MIN, "
                          "H_RSC_MODE, ESC calibration")

        if not passed and protocol is None:
            print("  No esc_status read (non-ARM MCU or connection failed)")
            print("  Possible causes:")
            print("    * Wrong output channel / SERVO_BLH_MASK")
            print("    * ESC not powered")
            print("    * Wiring fault on the signal wire")
            print("    * SERVO_BLH_DEBUG=0 (re-run after confirming it's 1)")

        # Raw log for deeper inspection
        if collected:
            print(f"  Raw STATUSTEXT ({len(collected)} lines):")
            for line in collected:
                print(f"    {line}")

    # ── Restore SERVO_BLH_DEBUG ───────────────────────────────────────────
    _set_param(mav, "SERVO_BLH_DEBUG", saved_debug)

    # ── Summary ───────────────────────────────────────────────────────────
    print(f"\n{sep}")
    print("SUMMARY")
    print(sep)
    for motor_num, r in results.items():
        status = "PASS" if r["passed"] else "FAIL"
        proto  = r.get("protocol")
        good   = r.get("good_frames")
        bad    = r.get("bad_frames")
        proto_str = f"  proto={proto}" if proto is not None else ""
        frames_str = (f"  good={good} bad={bad}"
                      if good is not None else "")
        print(f"  Motor {motor_num}: [{status}]{proto_str}{frames_str}")
    print(sep)


def _monitor_esc(mav, duration: float = 10.0) -> None:
    """
    Stream ESC telemetry continuously for `duration` seconds.
    Prints one line per second: RPM, current, voltage, temperature.
    """
    print(f"  Monitoring ESC telemetry for {duration:.0f} s  (Ctrl-C to stop)")
    print(f"  {'t(s)':<6} {'eRPM':<8} {'Mech RPM':<10} {'Rotor RPM':<11}"
          f" {'Current(A)':<12} {'Torque(Nm)':<12} {'Volt(V)':<9} {'Temp(C)'}")
    print(f"  {'-'*90}")

    deadline = time.monotonic() + duration
    last_print = 0.0
    try:
        while time.monotonic() < deadline:
            msg = mav.recv_match(type="ESC_TELEMETRY_1_TO_4",
                                 blocking=True, timeout=0.5)
            if msg is None:
                continue
            now = time.monotonic()
            if now - last_print < 0.25:   # print at ~4 Hz max
                continue
            last_print = now

            # Use ESC index 0 = first BLHeli-registered motor = GB4008
            # SERVO_BLH_MASK=256 (only output 9) -> GB4008 is BLHeli motor #1 -> index 0
            i = 0
            try:
                rpm_e    = msg.rpm[i]
                volt     = msg.voltage[i] / 100.0
                curr     = msg.current[i] / 100.0
                temp     = msg.temperature[i]
            except (IndexError, TypeError):
                continue

            mech_rpm  = rpm_e / GB4008_POLE_PAIRS
            rotor_rpm = mech_rpm / GB4008_GEAR_RATIO
            torque    = curr * GB4008_KT / GB4008_GEAR_RATIO
            elapsed   = duration - (deadline - now)
            print(f"  {elapsed:<6.1f} {rpm_e:<8} {mech_rpm:<10.0f} {rotor_rpm:<11.1f}"
                  f" {curr:<12.2f} {torque:<12.4f} {volt:<9.2f} {temp}")
    except KeyboardInterrupt:
        print("  Monitoring stopped.")


# ---------------------------------------------------------------------------
# Interactive REPL
# ---------------------------------------------------------------------------

_HELP = """
Commands:
  servo <n> <pwm>                 Set output n (1-16) to pwm us directly
  neutral [n]                     Set output n (or all swash servos) to 1500
  swash <coll_%> [lon_%] [lat_%]  Set S1/S2/S3 via H3-120 mixer (-100..+100)
  motor <pct>                     Run GB4008 motor test at pct % (0-100)
  motor off                       Stop motor test
  sweep <n> [step_ms]             Slowly sweep output n: 1000->2000->1000
  status                          Print SERVO_OUTPUT_RAW
  vehicle                         Print armed state, mode, EKF health, battery
  diag                            Full motor/ESC/DShot diagnostic dump
  blhtest [motor ...]             BLHeli/AM32 bootloader connection test
                                   (auto-detects from SERVO_BLH_MASK; or pass
                                    specific 1-indexed output numbers)
  monitor [seconds]               Stream live ESC telemetry (default 10 s)
  listen [seconds]                Stream STATUSTEXT + armed state (Ctrl-C to stop)
  mode <0-8>                      Set SCR_USER6 mode and listen 10 s to confirm active
  arm                             Send throttle=800 RC override (PWM) then force-arm vehicle
  disarm                          Disarm vehicle
  hold <pwm> [seconds]            Arm, set output 9 (AUX OUT 1) to pwm, keep alive (Ctrl-C to stop)
  reboot                          Reboot ArduPilot
  bench-mode                      Output 9: RCPassThru (direct PWM, no ArduPilot control)
  flight-mode                     Output 9: Script 1 / Motor4 (Lua or direct DShot)
  config dshot|pwm [--apply]      Diff (or apply) full RAWES param set; --apply writes + reboots
  getparam <name>                 Read a parameter value
  setparam <name> <value>         Write a parameter value
  ftp-list                        List files in /APM/scripts on SD card
  ftp-remove <file>               Remove a file from /APM/scripts
  ftp-upload <file> [--no-restart] Upload .lua file to /APM/scripts and restart
  help                            Show this list
  quit                            Exit
"""


def _send_rc_override(mav, channels: dict) -> None:
    """Send RC_CHANNELS_OVERRIDE. Channels not listed are set to 0 (no override)."""
    pwm = [0] * 8
    for ch, val in channels.items():
        if 1 <= ch <= 8:
            pwm[ch - 1] = val
    mav.mav.rc_channels_override_send(
        mav.target_system, mav.target_component,
        *pwm,  # channels 1-8
    )


def _arm(mav, force: bool = False, timeout: float = 15.0) -> bool:
    """
    Arm sequence:
      1. Set throttle RC override to 1000 (pre-arm requirement).
      2. Send MAV_CMD_COMPONENT_ARM_DISARM; wait for armed heartbeat.
      3. ESC arming sequence on output 4: 1500 us for 2 s, then 1000 us for 5 s.
    Returns True if vehicle confirms armed.
    """
    print("  Setting throttle (CH3) override to 1000 ...")
    _send_rc_override(mav, {3: 1000})
    time.sleep(0.5)

    print("  Sending arm command ...")
    param2 = 21196.0 if force else 0.0
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0,     # param1: 1 = arm
        param2,  # param2: 21196 = force-arm (bypass pre-arm checks)
        0, 0, 0, 0, 0,
    )

    deadline = time.monotonic() + timeout
    armed = False
    while time.monotonic() < deadline:
        # Keep refreshing throttle override so ArduPilot doesn't expire it
        _send_rc_override(mav, {3: 1000})
        msg = mav.recv_match(type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"],
                             blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "STATUSTEXT":
            print(f"  [FC] {msg.text.rstrip()}")
        elif t == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("  Arm command accepted -- waiting for armed heartbeat ...")
            elif msg.result == mavutil.mavlink.MAV_RESULT_DENIED:
                print("  [FAIL] Arm denied -- check pre-arm messages above")
                return False
        elif t == "HEARTBEAT":
            if bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("  [OK] Vehicle armed.")
                armed = True
                break

    if not armed:
        print("  [FAIL] Arm timed out.")
        return False

    # ESC arming sequence — PWM only.
    # DShot auto-arms on the first valid packet; no sequence needed.
    blh_mask = _recv_param(mav, "SERVO_BLH_MASK") or 0.0
    dshot_active = int(blh_mask) & (1 << (SERVO_MOTOR - 1))

    if not dshot_active:
        # PWM arming: hold 800 us so ESC recognises the low endpoint (AM32 minimum = 800 us)
        print(f"  ESC arm (PWM): output {SERVO_MOTOR} -> 800 us for 5 s ...")
        t_end = time.monotonic() + 5.0
        while time.monotonic() < t_end:
            _send_set_servo(mav, SERVO_MOTOR, 800)
            time.sleep(0.1)
        print("  ESC arm sequence complete -- motor ready.")

    return True


def _disarm(mav, timeout: float = 10.0) -> bool:
    """Send disarm command. Returns True if vehicle confirms disarmed."""
    print("  Sending disarm command ...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0,    # param1: 0 = disarm
        0, 0, 0, 0, 0, 0,
    )
    # Clear throttle override
    _send_rc_override(mav, {})
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                print("  [OK] Vehicle disarmed.")
                return True
    print("  [FAIL] Disarm timed out.")
    return False


def _sweep(mav, instance: int, step_ms: int = 5) -> None:
    """Sweep a servo from 1000 to 2000 and back, step_ms ms per PWM step."""
    print(f"  Sweeping output {instance}: 1500 -> 2000 -> 1000 -> 1500  (Ctrl-C to abort)")
    delay = step_ms / 1000.0
    try:
        for pwm in range(1500, 2001, 1):
            _send_set_servo(mav, instance, pwm)
            time.sleep(delay)
        for pwm in range(2000, 999, -1):
            _send_set_servo(mav, instance, pwm)
            time.sleep(delay)
        for pwm in range(1000, 1501, 1):
            _send_set_servo(mav, instance, pwm)
            time.sleep(delay)
    except KeyboardInterrupt:
        _send_set_servo(mav, instance, PWM_NEUTRAL)
        print("  Sweep interrupted -- servo returned to neutral")


def _run_command(mav, tokens: list[str], force: bool = False) -> bool:
    """
    Execute one calibration command.

    tokens : command + arguments, e.g. ["motor", "10"]
    force  : skip interactive confirmation prompts (for CLI / scripted use)

    Returns True if the command was recognised, False for unknown commands.
    """
    if not tokens:
        return True
    cmd = tokens[0].lower()

    # ── status ──────────────────────────────────────────────────────────────
    if cmd == "status":
        _print_status(mav)

    # ── vehicle ──────────────────────────────────────────────────────────────
    elif cmd == "vehicle":
        _print_vehicle_status(mav)

    # ── servo <n> <pwm> ─────────────────────────────────────────────────────
    elif cmd == "servo":
        if len(tokens) < 3:
            print("  Usage: servo <n> <pwm>"); return True
        try:
            n, pwm = int(tokens[1]), int(tokens[2])
        except ValueError:
            print("  Error: n and pwm must be integers"); return True
        if not (1 <= n <= 16):
            print("  Error: n must be 1-16"); return True
        if not (PWM_MIN <= pwm <= PWM_MAX):
            print(f"  Error: pwm must be {PWM_MIN}-{PWM_MAX}"); return True
        _send_set_servo(mav, n, pwm)
        print(f"  Output {n} -> {pwm} us")

    # ── neutral [n] ─────────────────────────────────────────────────────────
    elif cmd == "neutral":
        if len(tokens) >= 2:
            try:
                targets = [int(tokens[1])]
            except ValueError:
                print("  Usage: neutral [n]"); return True
        else:
            targets = list(SWASH_SERVOS)
        for n in targets:
            _send_set_servo(mav, n, PWM_NEUTRAL)
        print(f"  Output(s) {targets} -> {PWM_NEUTRAL} us")

    # ── swash <coll_%> [lon_%] [lat_%] ──────────────────────────────────────
    elif cmd == "swash":
        if len(tokens) < 2:
            print("  Usage: swash <coll_%> [lon_%] [lat_%]"); return True
        try:
            coll = float(tokens[1]) / 100.0
            lon  = float(tokens[2]) / 100.0 if len(tokens) > 2 else 0.0
            lat  = float(tokens[3]) / 100.0 if len(tokens) > 3 else 0.0
        except ValueError:
            print("  Error: values must be numbers"); return True
        s1n, s2n, s3n = _h3_forward_mix(coll, lon, lat)
        pwm1, pwm2, pwm3 = _norm_to_pwm(s1n), _norm_to_pwm(s2n), _norm_to_pwm(s3n)
        _send_set_servo(mav, SERVO_S1, pwm1)
        _send_set_servo(mav, SERVO_S2, pwm2)
        _send_set_servo(mav, SERVO_S3, pwm3)
        print(f"  swash coll={coll*100:.0f}% lon={lon*100:.0f}% lat={lat*100:.0f}%")
        print(f"    S1={pwm1} us  S2={pwm2} us  S3={pwm3} us")

    # ── motor <pct> | motor off ──────────────────────────────────────────────
    elif cmd == "motor":
        if len(tokens) < 2:
            print("  Usage: motor <pct>  OR  motor off"); return True
        arg = tokens[1].lower()
        if arg in ("off", "stop", "0"):
            _send_motor_test(mav, MOTOR_TEST_INSTANCE, 0.0, timeout_s=0.0)
            print("  Motor test stopped")
        else:
            try:
                pct = float(arg)
            except ValueError:
                print("  Error: pct must be a number (0-100) or 'off'"); return True
            if not (0.0 <= pct <= 100.0):
                print("  Error: pct must be 0-100"); return True
            if pct > 30.0 and not force:
                confirm = input(f"  WARNING: {pct:.0f}% throttle on GB4008. Confirm (y/N): ")
                if confirm.strip().lower() != "y":
                    print("  Cancelled."); return True
            _send_motor_test(mav, MOTOR_TEST_INSTANCE, pct, timeout_s=5.0)
            print(f"  Motor test: {pct:.0f}% for 5 s  (send 'motor off' to stop early)")

    # ── sweep <n> [step_ms] ──────────────────────────────────────────────────
    elif cmd == "sweep":
        if len(tokens) < 2:
            print("  Usage: sweep <n> [step_ms]"); return True
        try:
            n       = int(tokens[1])
            step_ms = int(tokens[2]) if len(tokens) > 2 else 5
        except ValueError:
            print("  Error: n and step_ms must be integers"); return True
        if not (1 <= n <= 16):
            print("  Error: n must be 1-16"); return True
        _sweep(mav, n, step_ms)

    # ── diag ────────────────────────────────────────────────────────────────
    elif cmd == "diag":
        _diag_motor(mav)

    # ── blhtest [motor ...] ──────────────────────────────────────────────────
    elif cmd == "blhtest":
        try:
            motors = [int(t) for t in tokens[1:]] if len(tokens) > 1 else None
        except ValueError:
            print("  Usage: blhtest [motor ...]  (motor = 1-indexed output number)")
            return True
        _dshot_blheli_test(mav, motors=motors)

    # ── monitor [seconds] ────────────────────────────────────────────────────
    elif cmd == "monitor":
        try:
            secs = float(tokens[1]) if len(tokens) > 1 else 10.0
        except ValueError:
            print("  Usage: monitor [seconds]"); return True
        _monitor_esc(mav, duration=secs)

    # ── mode <n> ─────────────────────────────────────────────────────────────
    elif cmd == "mode":
        _MODE_NAMES = {
            0: "none", 1: "steady_noyaw", 2: "yaw", 3: "steady",
            4: "landing_noyaw", 5: "pumping_noyaw", 6: "arm_hold_noyaw",
            7: "yaw_test", 8: "yaw_limited",
        }
        if len(tokens) < 2:
            print("  Usage: mode <0-8>")
            print("  Modes: " + "  ".join(f"{k}={v}" for k, v in _MODE_NAMES.items()))
            return True
        try:
            n = int(tokens[1])
        except ValueError:
            print("  Error: mode must be an integer 0-8"); return True
        if n not in _MODE_NAMES:
            print(f"  Error: mode must be 0-8"); return True
        if not _set_param(mav, "SCR_USER6", float(n)):
            print("  [FAIL] Could not set SCR_USER6"); return True
        name = _MODE_NAMES[n]
        print(f"  [OK] SCR_USER6={n} ({name}) -- listening 10 s for STATUSTEXT ...")
        t0 = time.monotonic()
        seen = []
        try:
            while time.monotonic() - t0 < 10.0:
                msg = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
                if msg is None:
                    continue
                text = msg.text.rstrip()
                t = time.monotonic() - t0
                print(f"  [{t:5.1f}s] {text}")
                seen.append(text)
        except KeyboardInterrupt:
            print()
        if seen:
            print(f"  [OK] {len(seen)} message(s) received -- mode appears active.")
        else:
            print("  [WARN] No STATUSTEXT received -- script may not be running.")

    # ── listen [seconds] ─────────────────────────────────────────────────────
    elif cmd == "listen":
        try:
            secs = float(tokens[1]) if len(tokens) > 1 else None
        except ValueError:
            print("  Usage: listen [seconds]"); return True
        print("  Listening for STATUSTEXT (Ctrl-C to stop) ...")
        t0 = time.monotonic()
        last_armed = None
        try:
            while True:
                if secs and (time.monotonic() - t0) >= secs:
                    break
                msg = mav.recv_match(type=["STATUSTEXT", "HEARTBEAT"],
                                     blocking=True, timeout=1.0)
                if msg is None:
                    continue
                mt = msg.get_type()
                t = time.monotonic() - t0
                if mt == "STATUSTEXT":
                    print(f"  [{t:6.1f}s] {msg.text.rstrip()}")
                elif mt == "HEARTBEAT":
                    armed = bool(msg.base_mode & 0x80)
                    if armed != last_armed:
                        state = "ARMED" if armed else "DISARMED"
                        print(f"  [{t:6.1f}s] *** {state} ***")
                        last_armed = armed
        except KeyboardInterrupt:
            print()

    # ── hold <pwm> [seconds] ─────────────────────────────────────────────────
    elif cmd == "hold":
        if len(tokens) < 2:
            print("  Usage: hold <pwm> [seconds]"); return True
        try:
            pwm = int(tokens[1])
            duration = float(tokens[2]) if len(tokens) > 2 else None
        except ValueError:
            print("  Error: pwm must be an integer"); return True
        # In PWM mode (no DShot) the stop value is 800; in DShot it is 1000.
        blh_mask = _recv_param(mav, "SERVO_BLH_MASK") or 0.0
        dshot_active = int(blh_mask) & (1 << (SERVO_MOTOR - 1))
        stop_pwm = PWM_MIN if dshot_active else 800
        low = stop_pwm
        if not (low <= pwm <= PWM_MAX):
            print(f"  Error: pwm must be {low}-{PWM_MAX}"); return True
        # Arm first
        if not _arm(mav, force=True):
            return True
        print(f"  Holding output {SERVO_MOTOR} at {pwm} us  (Ctrl-C to stop) ...")
        _send_set_servo(mav, SERVO_MOTOR, pwm)
        t0 = time.monotonic()
        last_armed = True
        try:
            while True:
                if duration and (time.monotonic() - t0) >= duration:
                    break
                _send_rc_override(mav, {3: 1000})
                _send_set_servo(mav, SERVO_MOTOR, pwm)
                elapsed = time.monotonic() - t0
                # Drain all pending messages; print interesting ones
                while True:
                    msg = mav.recv_match(blocking=True, timeout=0.1)
                    if msg is None:
                        break
                    mt = msg.get_type()
                    if mt == "STATUSTEXT":
                        print(f"\r  [{elapsed:5.1f}s] STATUSTEXT: {msg.text}")
                    elif mt == "HEARTBEAT":
                        armed = bool(msg.base_mode & 0x80)
                        if armed != last_armed:
                            state = "ARMED" if armed else "DISARMED"
                            print(f"\r  [{elapsed:5.1f}s] *** {state} ***")
                            last_armed = armed
                print(f"  t={elapsed:5.1f}s  output{SERVO_MOTOR}={pwm} us", end="\r")
        except KeyboardInterrupt:
            print()
        _send_set_servo(mav, SERVO_MOTOR, stop_pwm)
        print(f"  Output {SERVO_MOTOR} -> {stop_pwm} us (safe)")

    # ── reboot ───────────────────────────────────────────────────────────────
    elif cmd == "reboot":
        print("  Sending reboot command ...")
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        # USB will drop -- swallow the serial error
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
                if msg is None:
                    break
            except Exception:
                break
        print("  Pixhawk rebooting -- reconnect in a few seconds.")

    # ── bench-mode / flight-mode ─────────────────────────────────────────────
    elif cmd == "bench-mode":
        # Full PWM mode: disable DShot on output 9 and use RCPassThru
        rover = _is_rover(mav)
        steps = [
            ("SERVO9_FUNCTION",  1),   # RCPassThru — direct PWM control
            ("SERVO_BLH_MASK",   0),   # no DShot output
            ("SERVO_BLH_OTYPE",  0),   # PWM (not DShot)
            ("BRD_IO_DSHOT",     0),   # not needed (FMU output) — set explicitly for safety
            ("SERVO9_MIN",       800), # AM32 PWM minimum
        ]
        if rover:
            steps.append(("MOT_PWM_TYPE", 0))  # normal PWM
        ok = all(_set_param(mav, name, val) for name, val in steps)
        if ok:
            print("  [OK] Bench/PWM mode set:")
            extra = "  MOT_PWM_TYPE=0" if rover else ""
            print(f"       SERVO9_FUNCTION=1  SERVO_BLH_MASK=0  SERVO_BLH_OTYPE=0  BRD_IO_DSHOT=0{extra}")
            print("  Rebooting (SERVO9_FUNCTION/SERVO_BLH_MASK changes require reboot) ...")
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0, 1, 0, 0, 0, 0, 0, 0,
            )
            print("  Pixhawk rebooting -- reconnect in ~5 s then run 'arm' + 'hold'.")
        else:
            print("  [FAIL] Could not set all params")

    elif cmd == "flight-mode":
        # Restore DShot300 for flight.
        # Rover uses MOT_PWM_TYPE=6 (DShot300) instead of SERVO_BLH_OTYPE.
        # SERVO_BLH_BDMASK is left at 0 (one-way DShot) -- do NOT set to 256
        # until AM32 EDT is enabled on the ESC; bidir without EDT causes bad-frame
        # accumulation in ArduPilot and silently suppresses motor output.
        rover = _is_rover(mav)
        if rover:
            steps = [
                ("SERVO9_FUNCTION",  36),  # Motor4 on AUX 1 (Rover motor test path)
                ("SERVO_BLH_MASK",   256), # DShot on output 9 (AUX 1)
                ("SERVO_BLH_BDMASK", 0),   # one-way DShot (enable after AM32 EDT active)
                ("SERVO_BLH_POLES",  22),  # GB4008 24N22P
                ("SERVO_DSHOT_ESC",  3),   # AM32
                ("MOT_PWM_TYPE",     6),   # DShot300 (Rover motor driver)
            ]
            summary = ("SERVO9_FUNCTION=36  SERVO_BLH_MASK=256  SERVO_BLH_BDMASK=0"
                       "  SERVO_BLH_POLES=22  SERVO_DSHOT_ESC=3  MOT_PWM_TYPE=6")
        else:
            # Heli/Copter + rawes.lua: Lua owns output 9 via Script 1 (SERVO9_FUNCTION=94).
            # SERVO9_FUNCTION=36 (Motor4) would bypass Lua -- never use for Lua-controlled flight.
            steps = [
                ("SERVO9_FUNCTION",  94),  # Script 1: Lua writes GB4008 PWM via SRV_Channels
                ("SERVO_BLH_MASK",   256), # DShot on output 9 (AUX 1, FMU -- no BRD_IO_DSHOT needed)
                ("SERVO_BLH_BDMASK", 0),   # one-way DShot (enable after AM32 EDT active)
                ("SERVO_BLH_OTYPE",  5),   # DShot300
                ("SERVO_BLH_POLES",  22),  # GB4008 24N22P
                ("SERVO_DSHOT_ESC",  3),   # AM32
            ]
            summary = ("SERVO9_FUNCTION=94  SERVO_BLH_MASK=256  SERVO_BLH_BDMASK=0"
                       "  SERVO_BLH_OTYPE=5  SERVO_BLH_POLES=22  SERVO_DSHOT_ESC=3")
        ok = all(_set_param(mav, name, val) for name, val in steps)
        if ok:
            print(f"  [OK] Flight/DShot mode set ({'Rover' if rover else 'Heli/Copter'}):")
            print(f"       {summary}")
            print("  Rebooting ...")
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0, 1, 0, 0, 0, 0, 0, 0,
            )
            print("  Pixhawk rebooting -- reconnect in ~5 s.")
        else:
            print("  [FAIL] Could not set all params")

    # ── config dshot|pwm [--apply] ───────────────────────────────────────────
    elif cmd == "config":
        # Usage: config dshot [--apply]   -- DShot/Lua flight config
        #        config pwm   [--apply]   -- PWM bench config
        # Without --apply: read-only diff (shows what would change, touches nothing).
        # With    --apply: writes params then reboots.
        remaining = [t.lower() for t in tokens[1:]]
        apply = "--apply" in remaining
        sub = next((t for t in remaining if t != "--apply"), "")
        if sub not in ("dshot", "pwm"):
            print("  Usage: config dshot [--apply] | config pwm [--apply]")
            print("  --apply writes params and reboots; omit to preview diff only.")
            return True

        rover = _is_rover(mav)

        common = [
            ("BRD_SAFETY_DEFLT", 0),
            ("ARMING_CHECK",     0),
            ("BRD_IO_DSHOT",     0),
            ("H_COL_ANG_MIN",    -10),
            ("H_COL_ANG_MAX",    10),
            ("GPS1_TYPE",        0),
            ("GPS2_TYPE",        0),
            ("SCR_ENABLE",       1),  # scripting on -- script loaded but inactive until SCR_USER6 set
            ("SCR_USER6",        0),  # Lua mode 0 = none -- safe default
        ]

        if sub == "dshot":
            specific = [
                ("SERVO9_FUNCTION",  36  if rover else 94),
                ("SERVO9_MIN",       1000),
                ("SERVO9_MAX",       2000),
                ("SERVO9_TRIM",      1000),
                ("SERVO_BLH_MASK",   256),
                ("SERVO_BLH_BDMASK", 0),
                ("SERVO_BLH_OTYPE",  5),
                ("SERVO_BLH_POLES",  22),
                ("SERVO_BLH_TRATE",  10),
                ("SERVO_BLH_AUTO",   0),
                ("SERVO_DSHOT_ESC",  3),
                ("SERVO_DSHOT_RATE", 0),
            ]
            if rover:
                specific.append(("MOT_PWM_TYPE", 6))
        else:  # pwm
            specific = [
                ("SERVO9_FUNCTION",  1),
                ("SERVO9_MIN",       800),
                ("SERVO9_MAX",       2000),
                ("SERVO9_TRIM",      1500),
                ("SERVO_BLH_MASK",   0),
                ("SERVO_BLH_OTYPE",  0),
            ]
            if rover:
                specific.append(("MOT_PWM_TYPE", 0))

        steps = specific + common
        frame_str = "Rover" if rover else "Heli/Copter"
        action = "Applying" if apply else "Preview (read-only -- add --apply to write)"
        print(f"  Mode: {sub.upper()}  Frame: {frame_str}  [{action}]")
        print()
        print(f"  {'Parameter':<25} {'Expected':>8}  {'Actual':>8}  Status")
        print(f"  {'-'*25}  {'-'*8}  {'-'*8}  ------")
        any_diff = False
        any_fail = False
        for name, expected in steps:
            actual = _recv_param(mav, name)
            if actual is None:
                row_status = "[FAIL] not found"
                any_fail = True
            elif abs(actual - float(expected)) < 0.5:
                row_status = "[OK]"
            else:
                row_status = f"[DIFF] {actual:.0f} -> {expected}"
                any_diff = True
            actual_str = f"{actual:.0f}" if actual is not None else "N/A"
            print(f"  {name:<25} {expected:>8}  {actual_str:>8}  {row_status}")

        print()
        if any_fail:
            print("  [FAIL] One or more parameters not found on this firmware.")
        elif not any_diff:
            print("  [OK] All parameters already correct.")
        elif not apply:
            print("  [DIFF] Run with --apply to write changes.")
        else:
            print("  Writing changes ...")
            for name, val in steps:
                _set_param(mav, name, val)
            print("  [OK] Parameters written. Rebooting ...")
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0, 1, 0, 0, 0, 0, 0, 0,
            )
            print("  Pixhawk rebooting -- reconnect in ~5 s.")

    # ── arm ──────────────────────────────────────────────────────────────────
    elif cmd == "arm":
        # Always force-arm: calibrate.py is a bench tool; blades/collective
        # calibration (H_COL_ANG_MIN/MAX) will not be set on the bench.
        _arm(mav, force=True)

    # ── disarm ───────────────────────────────────────────────────────────────
    elif cmd == "disarm":
        _disarm(mav)

    # ── getparam <name> ──────────────────────────────────────────────────────
    elif cmd == "getparam":
        if len(tokens) < 2:
            print("  Usage: getparam <name>"); return True
        name = tokens[1].upper()
        val = _recv_param(mav, name)
        if val is None:
            print(f"  {name}: no response (parameter not found?)")
        else:
            print(f"  {name} = {val}")

    # ── setparam <name> <value> ───────────────────────────────────────────────
    elif cmd == "setparam":
        if len(tokens) < 3:
            print("  Usage: setparam <name> <value>"); return True
        name = tokens[1].upper()
        try:
            value = float(tokens[2])
        except ValueError:
            print("  Error: value must be a number"); return True
        ok = _set_param(mav, name, value)
        if ok:
            actual = _recv_param(mav, name)
            print(f"  [OK]   {name} = {actual}")
        else:
            print(f"  [FAIL] {name}: no ACK within timeout")

    # ── ftp-list ─────────────────────────────────────────────────────────────
    elif cmd == "ftp-list":
        _list_scripts(mav)

    # ── ftp-remove <file> ────────────────────────────────────────────────────
    elif cmd == "ftp-remove":
        if len(tokens) < 2:
            print("  Usage: ftp-remove <file>"); return True
        _remove_script(mav, tokens[1])

    # ── ftp-upload <file> [--no-restart] ─────────────────────────────────────
    elif cmd == "ftp-upload":
        if len(tokens) < 2:
            print("  Usage: ftp-upload <file> [--no-restart]"); return True
        local_path = tokens[1]
        restart    = "--no-restart" not in tokens
        _upload_script(mav, local_path, restart=restart)

    else:
        return False

    return True


def _repl(mav) -> None:
    print("\nConnected. Type 'help' for commands, 'quit' to exit.\n")
    while True:
        try:
            raw = input("calibrate> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not raw:
            continue
        tokens = raw.split()
        cmd    = tokens[0].lower()
        if cmd in ("quit", "exit", "q"):
            break
        if cmd == "help":
            print(_HELP)
            continue
        if not _run_command(mav, tokens, force=False):
            print(f"  Unknown command: {cmd!r}  (type 'help')")


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="RAWES calibration tool — servo, motor, and Lua script management",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_HELP,
    )
    p.add_argument("--port", "-p", default="COM4",
                   help="Serial port or MAVLink URL (default: COM4)")
    p.add_argument("--baud", "-b", type=int, default=115200,
                   help="Baud rate (default: 115200)")
    p.add_argument("--force", "-f", action="store_true",
                   help="Skip confirmation prompts (for scripted/CI use)")
    p.add_argument("command", nargs="?", default=None,
                   help="Command to run non-interactively; omit for interactive REPL")
    p.add_argument("args", nargs=argparse.REMAINDER,
                   help="Arguments for the command")
    return p


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _connect(port: str, baud: int):
    print(f"Connecting to {port} at {baud} baud ...")
    mav = mavutil.mavlink_connection(port, baud=baud, source_system=255)
    mav.wait_heartbeat(timeout=15)
    print(f"Connected: sysid={mav.target_system} compid={mav.target_component}")

    def _hb():
        while True:
            try:
                mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
            except Exception:
                break
            time.sleep(1.0)
    threading.Thread(target=_hb, daemon=True).start()

    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 10, 1,
    )
    return mav


def main() -> None:
    args = _build_parser().parse_args()

    mav = _connect(args.port, args.baud)
    exit_code = 0
    try:
        if args.command:
            # Non-interactive: run one command and exit
            tokens = [args.command] + list(args.args)
            ok = _run_command(mav, tokens, force=args.force)
            if not ok:
                print(f"Unknown command: {args.command!r}")
                _build_parser().print_help()
                exit_code = 1
        else:
            # Interactive REPL
            _repl(mav)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        mav.close()
        print("Disconnected.")

    if exit_code:
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
