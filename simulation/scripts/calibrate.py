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
  Output 4  GB4008 anti-rotation motor             SERVO4_FUNCTION = 36 (DDFP tail)

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
SERVO_MOTOR  = 4   # GB4008 anti-rotation motor

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
        ftp.cmd_list([SCRIPTS_DIR])
    except Exception as exc:
        print(f"  FTP list failed: {exc}")


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
    try:
        ftp = _mavftp_mod.MAVFTP(
            mav,
            target_system=mav.target_system,
            target_component=mav.target_component,
        )
        ftp.cmd_put([local_path, remote_path])
        print("  Upload complete.")
    except Exception as exc:
        print(f"  FTP upload failed: {exc}")
        return

    if restart:
        _restart_scripting(mav)


# ---------------------------------------------------------------------------
# Motor / ESC diagnostic helpers
# ---------------------------------------------------------------------------

# Parameters that must be correct for DShot to reach the motor
_DSHOT_PARAMS = [
    ("SERVO4_FUNCTION",    "Must be 36 (DDFP tail) for GB4008 on output 4"),
    ("SERVO_BLH_MASK",     "Must include bit 3 (=8) to enable DShot on output 4"),
    ("SERVO_BLH_OTYPE",    "DShot type: 5=DShot300  6=DShot600  (0=PWM, won't work)"),
    ("SERVO_BLH_BDSHOT",   "1 = bidirectional DShot enabled (needed for RPM telemetry)"),
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
    if v("SERVO4_FUNCTION") not in (36.0, 38.0):
        issues.append("SERVO4_FUNCTION is not 36 (DDFP) -- output 4 not mapped to tail motor")
    blh_mask = v("SERVO_BLH_MASK")
    if blh_mask is not None and not (int(blh_mask) & 8):
        issues.append("SERVO_BLH_MASK does not include bit 3 -- DShot not enabled on output 4")
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
    _send_motor_test(mav, SERVO_MOTOR, 5.0, timeout_s=3.0)
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
        print("         Either SERVO_BLH_BDSHOT=0, SERVO_BLH_MASK wrong,")
        print("         or ESC not responding to DShot (wiring/power issue)")

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
    _send_motor_test(mav, SERVO_MOTOR, 0.0, timeout_s=0.0)

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

    print(f"\n{sep}")
    print("Diagnostic complete.")
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

            # Use ESC index 3 (0-based) = output 4 = GB4008
            i = SERVO_MOTOR - 1
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
  diag                            Full motor/ESC/DShot diagnostic dump
  monitor [seconds]               Stream live ESC telemetry (default 10 s)
  scripts                         List files in /APM/scripts on SD card
  upload <file> [--no-restart]    Upload .lua file to /APM/scripts and restart
  help                            Show this list
  quit                            Exit
"""


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
            _send_motor_test(mav, SERVO_MOTOR, 0.0, timeout_s=0.0)
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
            _send_motor_test(mav, SERVO_MOTOR, pct, timeout_s=5.0)
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

    # ── monitor [seconds] ────────────────────────────────────────────────────
    elif cmd == "monitor":
        try:
            secs = float(tokens[1]) if len(tokens) > 1 else 10.0
        except ValueError:
            print("  Usage: monitor [seconds]"); return True
        _monitor_esc(mav, duration=secs)

    # ── scripts ──────────────────────────────────────────────────────────────
    elif cmd == "scripts":
        _list_scripts(mav)

    # ── upload <file> [--no-restart] ─────────────────────────────────────────
    elif cmd == "upload":
        if len(tokens) < 2:
            print("  Usage: upload <file> [--no-restart]"); return True
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

    if args.command:
        # Non-interactive: run one command and exit
        tokens = [args.command] + list(args.args)
        ok = _run_command(mav, tokens, force=args.force)
        if not ok:
            print(f"Unknown command: {args.command!r}")
            _build_parser().print_help()
            sys.exit(1)
    else:
        # Interactive REPL
        _repl(mav)

    mav.close()
    print("Disconnected.")


if __name__ == "__main__":
    main()
