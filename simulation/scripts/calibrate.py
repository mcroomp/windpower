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
  Output 1  S1  (swashplate, -60 deg  / front-right)  SERVO1_FUNCTION = 33
  Output 2  S2  (swashplate, +60 deg  / front-left)   SERVO2_FUNCTION = 34
  Output 3  S3  (swashplate, 180 deg  / back)         SERVO3_FUNCTION = 35
  Output 9  GB4008 anti-rotation motor (AUX 1)        SERVO9_FUNCTION = 36, bidir DShot

  See CLAUDE.md "Swashplate geometry" for the canonical azimuth table.
  The motor output is set by the SERVO_MOTOR constant (see design/dshot.md).

Motor (SERVO9) PWM range: 1000 us (off) ... 2000 us (full throttle)
Swashplate PWM range: 1000 us (min) ... 1500 us (neutral) ... 2000 us (max)

Commands
--------
  See 'help' for the full list.  Two long-running verbs (run, watch) log to
  simulation/logs/calibrate/.  Key commands:
    run <mode> [--duration N] [--trim K=V] [--gain K=V]
                                      Activate Lua mode + arm + observe + log
                                      Modes: passive, yaw, steady, pumping, landing
    watch <stream> [--duration N]     Read-only observation (servos, esc, attitude,
                                      power, text)
    status                            One-shot snapshot
    set / get <param> [value]         Parameter R/W
    swash, servo, motor               Hardware calibration
    arm / disarm / reboot             Operations
    script upload/list/remove         Lua script deployment
    config check / config fix         Param sync (default: rawes_common overrides only;
                                      use --all for full copter-heli+rawes_common)
    ping                              COM port discovery

Notes
-----
  MAV_CMD_DO_SET_SERVO works while DISARMED -- no arming required.
  MAV_CMD_DO_MOTOR_TEST also works while disarmed (designed for bench checks).
  Keep a safe distance from the rotor when testing the motor.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
import queue
import sys
import time
import threading
from datetime import datetime, timezone

# msvcrt is Windows stdlib -- used by yawmanual for non-blocking ESC-key abort.
# Falls back to a stub on non-Windows so the rest of the script still imports.
try:
    import msvcrt
except ImportError:
    class _MsvcrtStub:
        def kbhit(self):    return False
        def getch(self):    return b""
    msvcrt = _MsvcrtStub()

# Allow importing gcs.py from the parent directory (simulation/)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR = os.path.abspath(os.path.join(_SCRIPT_DIR, '..'))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)
from gcs       import RawesGCS, WallClock   # noqa: E402
from param_defaults import load_ap_params   # noqa: E402
from servo_pwm import (SWASH_PWM_MIN, SWASH_PWM_NEUTRAL, SWASH_PWM_MAX,
                        MOTOR_PWM_MIN, MOTOR_PWM_MAX)  # noqa: E402

from pymavlink import mavutil

try:
    from pymavlink import mavftp as _mavftp_mod
    _HAS_MAVFTP = True
except ImportError:
    _HAS_MAVFTP = False

# ---------------------------------------------------------------------------
# GB4008 motor constants (used in diag torque estimates)
# ---------------------------------------------------------------------------
GB4008_KV          = 66.0      # rev/min/V
GB4008_POLES       = 22        # rotor magnet poles = SERVO_BLH_POLES (compile-time default)
GB4008_POLE_PAIRS  = GB4008_POLES // 2   # 11 (overridden at connect from FC's SERVO_BLH_POLES)
GB4008_GEAR_RATIO  = 80.0 / 44.0  # motor shaft turns per output shaft turn

# Kt = 60 / (2*pi*Kv)  [N*m/A at motor shaft]
GB4008_KT = 60.0 / (2.0 * math.pi * GB4008_KV)  # ~0.144 N*m/A

# ---------------------------------------------------------------------------
# RAWES servo output numbers
# ---------------------------------------------------------------------------
SERVO_S1     = 1
SERVO_S2     = 2
SERVO_S3     = 3
# GB4008 anti-rotation motor: bidirectional DShot on AUX 1 = SERVO9 (output 9).
# Single source of truth for the motor output location -- see design/dshot.md.
# Change this one constant if the motor is ever re-cabled to another output.
SERVO_MOTOR         = 9
MOTOR_TEST_INSTANCE = 4   # Heli tail motor NUMBER (Motor4); independent of output

# DShot motor throttle endpoints.  ArduPilot maps SERVO<motor>_MIN..MAX -> DShot
# 0..2000, so MIN = motor off (idle) and the ESC SELF-ARMS from that idle command.
# The old PWM/REVVitRC "hold min throttle for 5 s to arm the ESC" dance is NOT
# needed on DShot.  Use these constants instead of bare 800/2000 literals so a
# future re-cable (or a return to PWM) is a one-line change.
MOTOR_OFF_US  = 1000   # DShot idle = throttle 0 = motor off
MOTOR_FULL_US = 2000   # DShot full throttle

SWASH_SERVOS = (SERVO_S1, SERVO_S2, SERVO_S3)

# ---------------------------------------------------------------------------
# DShot RPM telemetry
# ---------------------------------------------------------------------------
# The GB4008 runs bidirectional DShot on SERVO_MOTOR (AUX 1 = output 9), so its
# ESC telemetry arrives in ESC_TELEMETRY_9_TO_12 at index 0 (NOT _1_TO_4).  See
# design/dshot.md.  eRPM -> mech RPM (/pole-pairs) -> rotor RPM (/1.818 gear).
MOTOR_ESC_CHANNEL = SERVO_MOTOR

# eRPM -> mechanical-RPM divisor (pole-pairs).  Seeded from the GB4008 default
# but OVERRIDDEN from the FC's SERVO_BLH_POLES on connect (_refresh_pole_pairs)
# so the RPM readout tracks the param and is never a stale hardcode.
_motor_pole_pairs = GB4008_POLE_PAIRS

# ESC_TELEMETRY_x_TO_y decode: message name -> (numeric id, first output channel).
# Numeric ids from the v20 ardupilotmega dialect (the connection is MAVLink2).
_ESC_TELEM_MSGS = {
    "ESC_TELEMETRY_1_TO_4":  (11030, 1),
    "ESC_TELEMETRY_5_TO_8":  (11031, 5),
    "ESC_TELEMETRY_9_TO_12": (11032, 9),
}


def _esc_telem_msg_for_channel(channel: int) -> "tuple[str, int]":
    """(msg_name, msg_id) of the ESC_TELEMETRY block covering 1-based `channel`."""
    base = ((channel - 1) // 4) * 4 + 1
    for name, (mid, b) in _ESC_TELEM_MSGS.items():
        if b == base:
            return name, mid
    raise ValueError(f"no ESC_TELEMETRY message for output channel {channel}")


def _esc_erpm(msg, channel: int) -> "float | None":
    """eRPM for 1-based output `channel` from an ESC_TELEMETRY_* msg, else None."""
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
        print(f"  eRPM->RPM: SERVO_BLH_POLES={int(round(poles))} "
              f"-> {_motor_pole_pairs} pole-pairs")

# H3-120 forward mix constants -- bench rig azimuths.
# Must stay in sync with simulation/swashplate.py and AP's H_SW_H3_SV*_POS.
# See CLAUDE.md "Swashplate geometry" for the canonical layout.
_AZ_S1 = math.radians(-60.0)   # SV1: front-right
_AZ_S2 = math.radians( 60.0)   # SV2: front-left
_AZ_S3 = math.radians(180.0)   # SV3: back

# PWM range constants — imported from servo_pwm.py; local aliases for brevity.
PWM_MIN     = SWASH_PWM_MIN
PWM_NEUTRAL = SWASH_PWM_NEUTRAL
PWM_MAX     = SWASH_PWM_MAX

# Per-session saved SERVO{n}_FUNCTION values for release/restore.
_saved_servo_functions: dict[int, float] = {}


# ---------------------------------------------------------------------------
# H3-120 forward mixer
# ---------------------------------------------------------------------------

def _h3_forward_mix(coll: float, tilt_lon: float, tilt_lat: float):
    """
    Convert collective + cyclic tilts (all normalised -1..+1) to
    individual H3-120 servo positions (normalised -1..+1).

    Bench layout (must match H_SW_H3_SV*_POS on the FC -- see CLAUDE.md
    "Swashplate geometry"):
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
    session._mav.mav.command_long_send(
        session._target_system,
        session._target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,              # confirmation
        float(instance),
        float(pwm),
        0, 0, 0, 0, 0,
    )


def _send_motor_test(session: RawesGCS, instance: int,
                     throttle_pct: float, timeout_s: float = 3.0) -> None:
    """
    Send MAV_CMD_DO_MOTOR_TEST.

    instance      : motor output number (1-indexed)
    throttle_pct  : 0-100  (MOTOR_TEST_THROTTLE_PERCENT = 0)
    timeout_s     : test duration; 0 = run until next command
    """
    session._mav.mav.command_long_send(
        session._target_system,
        session._target_component,
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




_AP_BASE_PARM_PATH = os.path.join(_SIM_DIR, "tests", "sitl", "copter-heli.parm")
_RAWES_COMMON_PARM_PATH = os.path.join(_SIM_DIR, "tests", "sitl", "rawes_common_defaults.parm")

# Never push hardware-calibrated sensor values from defaults to a real FC.
# These are measured per-airframe and must come from on-device calibration.
_CALIBRATION_PARAM_PREFIXES = (
    "INS_ACCOFFS_",
    "INS_ACCSCAL_",
    "INS_ACC2OFFS_",
    "INS_ACC2SCAL_",
    "INS_ACC3OFFS_",
    "INS_ACC3SCAL_",
    "INS_GYROFFS_",
    "INS_GYR2OFFS_",
    "INS_GYR3OFFS_",
    "COMPASS_OFS",
    "COMPASS_DIA",
    "COMPASS_ODI",
    "COMPASS_MOT",
    "BARO1_GND_PRESS",
    "BARO2_GND_PRESS",
    "BARO3_GND_PRESS",
    "BARO1_GND_TEMP",
    "BARO2_GND_TEMP",
    "BARO3_GND_TEMP",
    "GND_ABS_PRESS",
    "GND_TEMP",
    "AHRS_TRIM_",
)


def _is_calibration_param(name: str) -> bool:
    for prefix in _CALIBRATION_PARAM_PREFIXES:
        if name.startswith(prefix):
            return True
    return False


def _load_shared_hw_target_params() -> dict[str, float]:
    """Load hardware target params from shared sources, excluding SITL-only overrides."""
    raw = load_ap_params([_AP_BASE_PARM_PATH, _RAWES_COMMON_PARM_PATH])
    return {k: v for k, v in raw.items() if not _is_calibration_param(k)}


def _load_common_override_target_params() -> dict[str, float]:
    """Load only params explicitly overridden in rawes_common_defaults.parm."""
    raw = load_ap_params([_RAWES_COMMON_PARM_PATH])
    return {k: v for k, v in raw.items() if not _is_calibration_param(k)}


_CONFIG_TARGET_PARAMS_ALL = _load_shared_hw_target_params()
_CONFIG_TARGET_PARAMS_COMMON = _load_common_override_target_params()

# Compact status snapshot groups shown in `status` output.
_KEY_PARAM_NAMES = (
    "FRAME_CLASS",
    "INITIAL_MODE",
    "H_TAIL_TYPE",
    "SCR_ENABLE",
    "SCR_USER6",
    "ARMING_CHECK",
    "BRD_SAFETY_DEFLT",
    "ACRO_TRAINER",
)

_TAIL_PARAM_NAMES = (
    "H_YAW_TRIM",
    "ATC_RAT_YAW_P",
    "ATC_RAT_YAW_I",
    "ATC_RAT_YAW_D",
    "ATC_RAT_YAW_IMAX",
    "ATC_RAT_YAW_FLTT",
    "ATC_RAT_YAW_FLTE",
    "ATC_RAT_YAW_FLTD",
    "H_RSC_MODE",
    "H_RSC_RUNUP_TIME",
    f"SERVO{SERVO_MOTOR}_MIN",
    f"SERVO{SERVO_MOTOR}_MAX",
    f"SERVO{SERVO_MOTOR}_TRIM",
    f"SERVO{SERVO_MOTOR}_FUNCTION",
)

_LUA_MODES = {0: "none", 1: "steady", 3: "passive", 4: "landing", 5: "pumping"}


def _print_status(session: RawesGCS) -> None:
    """Unified status: vehicle, battery, EKF, servo outputs, key params."""
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
    # SERVO_OUTPUT_RAW rides on the RC_CHANNELS data stream.  Without this
    # request the message often hasn't arrived in the 2 s window after connect,
    # so the readout shows "(no SERVO_OUTPUT_RAW received)" even though the FC
    # is broadcasting fine on other streams.
    session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)
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
        expected = _CONFIG_TARGET_PARAMS_ALL.get(name)
        val = session.get_param(name)
        if val is None:
            print(f"  {name:<22} NOT FOUND")
            continue
        if name == "SCR_USER6":
            lua_name = _LUA_MODES.get(int(val), f"mode_{int(val)}")
            print(f"  {name:<22} {val:<8.4g}  {lua_name}")
        elif expected is not None and abs(val - float(expected)) > 1e-4:
            print(f"  {name:<22} {val:<8.4g}  [DIFF] expected {expected}")
        else:
            print(f"  {name:<22} {val:<8.4g}  OK")

    ss2 = session._recv(type="SYS_STATUS", blocking=True, timeout=2.0)
    if ss2:
        motor_bit = 0x000200
        present = bool(ss2.onboard_control_sensors_present & motor_bit)
        enabled = bool(ss2.onboard_control_sensors_enabled & motor_bit)
        healthy = bool(ss2.onboard_control_sensors_health  & motor_bit)
        health  = "OK" if healthy else "[WARN] unhealthy"
        print(f"  {'motor outputs':<22} present={present}  enabled={enabled}  {health}")
        print(f"  {'CPU load':<22} {ss2.load/10.0:.1f}%")

    # --- tail PID ------------------------------------------------------------
    print(f"\n{sep}")
    print("TAIL PID  (GB4008 yaw / DDFP)")
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
# Drain helper — collect all matching messages for a fixed wall-clock window
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
# Lua script upload / management
# ---------------------------------------------------------------------------

SCRIPTS_DIR = "/APM/scripts"


def _restart_scripting(session: RawesGCS) -> None:
    """Restart Lua scripting engine by toggling SCR_ENABLE (no reboot needed)."""
    print("  Restarting scripting engine (SCR_ENABLE 1->0->1) ...")
    session.set_param("SCR_ENABLE", 0)
    time.sleep(0.5)
    session.set_param("SCR_ENABLE", 1)
    print("  Scripting engine restarted.")


def _download_latest_log(session: RawesGCS, dest_dir: str = ".") -> "str | None":
    """
    Download the most recent dataflash log using the MAVLink log-download protocol
    (LOG_REQUEST_LIST / LOG_REQUEST_DATA / LOG_DATA).

    Avoids pymavlink mavftp which uses /tmp/ internally (broken on Windows).
    Sends LOG_REQUEST_DATA with count=0xFFFFFFFF, reassembles out-of-order
    LOG_DATA packets, re-requests any gaps, and writes a .BIN file to dest_dir.
    """
    mav = session._mav
    sys_id  = session._target_system
    comp_id = session._target_component

    # -- 1. Enumerate logs ----------------------------------------------------
    print("  Requesting log list ...")
    mav.mav.log_request_list_send(sys_id, comp_id, 0, 0xFFFF)
    entries: dict[int, object] = {}
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        msg = session._recv(type="LOG_ENTRY", blocking=True, timeout=0.5)
        if msg is None:
            continue
        entries[msg.id] = msg
        if msg.id == msg.last_log_num:
            break
    if not entries:
        print("  No log entries returned.")
        return None

    latest   = entries[max(entries)]
    log_id   = latest.id
    log_size = latest.size
    print(f"  Latest log: id={log_id}  size={log_size} bytes")

    # -- 2. Download ----------------------------------------------------------
    os.makedirs(dest_dir, exist_ok=True)
    local = os.path.join(dest_dir, f"{log_id:08d}.BIN")

    def _request(ofs: int, count: int) -> None:
        mav.mav.log_request_data_send(sys_id, comp_id, log_id, ofs, count)

    print(f"  Downloading log {log_id} -> {local} ...")
    data = bytearray(log_size)
    received: set[int] = set()   # set of offsets written
    write_ptr = 0                # contiguous bytes confirmed written
    pending: dict[int, bytes] = {}  # out-of-order chunks: ofs -> bytes

    _request(0, 0xFFFFFFFF)
    last_progress = -1
    deadline = time.monotonic() + 120.0

    while write_ptr < log_size and time.monotonic() < deadline:
        msg = session._recv(type="LOG_DATA", blocking=True, timeout=2.0)
        if msg is None:
            _request(write_ptr, log_size - write_ptr)
            continue
        if msg.id != log_id or msg.count == 0:
            continue
        chunk = bytes(msg.data[:msg.count])
        ofs   = msg.ofs
        end   = ofs + len(chunk)
        if end <= log_size:
            data[ofs:end] = chunk
            pending[ofs] = chunk

        # Advance write_ptr over contiguous chunks
        while write_ptr in pending:
            write_ptr += len(pending.pop(write_ptr))

        pct = write_ptr * 100 // log_size
        if pct != last_progress and pct % 5 == 0:
            print(f"    {pct}%  ({write_ptr}/{log_size} bytes)", end="\r")
            last_progress = pct

    mav.mav.log_request_end_send(sys_id, comp_id)
    print()

    if write_ptr < log_size:
        print(f"  [WARN] Incomplete: {write_ptr}/{log_size} bytes")

    with open(local, "wb") as fh:
        fh.write(data[:write_ptr])

    size = os.path.getsize(local)
    print(f"  [OK] {size} bytes -> {local}")
    return local


def _cmd_logs(session: RawesGCS, args: list[str]) -> None:
    """logs [list] [fetch [--id N] [--dir PATH]]

    list          Print all dataflash log entries (id, size, date) the FC
                  reports via LOG_ENTRY.  No download.
    fetch         Download one log via the MAVLink LOG_REQUEST_DATA protocol.
                  --id N    log id to download (default: latest)
                  --dir D   destination directory (default: simulation/logs/calibrate)
    """
    schema = {"--id": "int", "--dir": "str"}
    sub = args[0].lower() if args else "fetch"
    rest = args[1:] if args else []
    try:
        _pos, flags = _parse_flags(rest, schema)
    except ValueError as e:
        print(f"  Error: {e}"); return

    mav     = session._mav
    sys_id  = session._target_system
    comp_id = session._target_component

    # ── enumerate logs ────────────────────────────────────────────────────────
    print("  [DBG] sending LOG_REQUEST_LIST (start=0 end=0xFFFF) ...")
    mav.mav.log_request_list_send(sys_id, comp_id, 0, 0xFFFF)
    entries: dict[int, object] = {}
    deadline = time.monotonic() + 10.0
    packets_seen = 0
    while time.monotonic() < deadline:
        msg = session._recv(type="LOG_ENTRY", blocking=True, timeout=0.5)
        if msg is None:
            print(f"  [DBG] timeout waiting for LOG_ENTRY (got {packets_seen} so far) "
                  f"-- {10.0 - (time.monotonic() - (deadline - 10.0)):.1f}s left")
            continue
        packets_seen += 1
        entries[msg.id] = msg
        print(f"  [DBG] LOG_ENTRY id={msg.id}  size={msg.size}  "
              f"last_log_num={msg.last_log_num}  "
              f"num_logs={msg.num_logs}")
        if msg.id == msg.last_log_num:
            print(f"  [DBG] received final entry (id == last_log_num={msg.last_log_num})")
            break

    if not entries:
        print("  [FAIL] No LOG_ENTRY messages received.")
        print("  Check: FC armed? dataflash enabled? MAVLink log stream active?")
        return

    # ── print list ────────────────────────────────────────────────────────────
    print(f"\n  {len(entries)} log(s) on FC:")
    for eid in sorted(entries):
        e = entries[eid]
        print(f"    id={eid:4d}  size={e.size:>10,} bytes")

    if sub == "list":
        return

    # ── select log to fetch ───────────────────────────────────────────────────
    if "--id" in flags:
        wanted = int(flags["--id"])
        if wanted not in entries:
            print(f"  [FAIL] Log id={wanted} not found (available: {sorted(entries)})")
            return
        entry = entries[wanted]
    else:
        entry = entries[max(entries)]

    log_id   = entry.id
    log_size = entry.size
    dest_dir = str(flags.get("--dir", os.path.join("simulation", "logs", "calibrate")))

    print(f"\n  Fetching log id={log_id}  size={log_size:,} bytes -> {dest_dir}/")
    if log_size == 0:
        print("  [FAIL] Log size is 0 -- nothing to download.")
        return

    os.makedirs(dest_dir, exist_ok=True)
    local = os.path.join(dest_dir, f"{log_id:08d}.BIN")

    def _request(ofs: int, count: int) -> None:
        print(f"  [DBG] LOG_REQUEST_DATA id={log_id} ofs={ofs} count={count}")
        mav.mav.log_request_data_send(sys_id, comp_id, log_id, ofs, count)

    data     = bytearray(log_size)
    pending: dict[int, bytes] = {}
    write_ptr = 0
    retries   = 0
    last_pct  = -1
    deadline  = time.monotonic() + 180.0

    _request(0, 0xFFFFFFFF)

    while write_ptr < log_size and time.monotonic() < deadline:
        msg = session._recv(type="LOG_DATA", blocking=True, timeout=2.0)
        if msg is None:
            retries += 1
            print(f"  [DBG] timeout waiting for LOG_DATA  write_ptr={write_ptr}  "
                  f"retries={retries}")
            _request(write_ptr, log_size - write_ptr)
            continue
        if msg.id != log_id or msg.count == 0:
            print(f"  [DBG] skipping LOG_DATA id={msg.id} count={msg.count}")
            continue
        chunk = bytes(msg.data[:msg.count])
        ofs   = msg.ofs
        end   = ofs + len(chunk)
        if end <= log_size:
            data[ofs:end] = chunk
            pending[ofs]  = chunk
        while write_ptr in pending:
            write_ptr += len(pending.pop(write_ptr))
        pct = write_ptr * 100 // log_size
        if pct != last_pct and pct % 10 == 0:
            print(f"    {pct:3d}%  ({write_ptr:,}/{log_size:,} bytes)", end="\r")
            last_pct = pct

    mav.mav.log_request_end_send(sys_id, comp_id)
    print()

    if write_ptr < log_size:
        print(f"  [WARN] Incomplete transfer: {write_ptr:,}/{log_size:,} bytes "
              f"({retries} retries)")
    with open(local, "wb") as fh:
        fh.write(data[:write_ptr])
    size = os.path.getsize(local)
    print(f"  [OK] {size:,} bytes -> {local}")


def _list_scripts(session: RawesGCS) -> None:
    """List files in /APM/scripts via MAVLink FTP."""
    if not _HAS_MAVFTP:
        print("  ERROR: pymavlink.mavftp not available -- upgrade pymavlink")
        return
    print(f"  Listing {SCRIPTS_DIR} ...")
    try:
        ftp = _mavftp_mod.MAVFTP(
            session._mav,
            target_system=session._target_system,
            target_component=session._target_component,
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


def _remove_script(session: RawesGCS, filename: str) -> None:
    """Remove a single file from /APM/scripts via MAVLink FTP."""
    if not _HAS_MAVFTP:
        print("  ERROR: pymavlink.mavftp not available -- upgrade pymavlink")
        return
    remote = f"{SCRIPTS_DIR}/{os.path.basename(filename)}"
    print(f"  Removing {remote} ...")
    try:
        ftp = _mavftp_mod.MAVFTP(
            session._mav,
            target_system=session._target_system,
            target_component=session._target_component,
        )
        result = ftp.cmd_rm([remote])
        if result.error_code == 0:
            print(f"  [OK] Removed.")
        else:
            print(f"  [FAIL] {result}")
    except Exception as exc:
        print(f"  FTP operation failed: {exc}")


def _upload_script(session: RawesGCS, local_path: str,
                   restart: bool = True) -> None:
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
                session._mav,
                target_system=session._target_system,
                target_component=session._target_component,
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
        _restart_scripting(session)


# ---------------------------------------------------------------------------
# COM port scanner
# ---------------------------------------------------------------------------

_FALLBACK_BAUDS = [57600, 38400, 19200, 9600]


def _probe_port(port: str, baud: int, timeout: float) -> tuple:
    """Try one port at one baud. Returns (ok, sysid) — closes connection before returning."""
    conn = None
    try:
        conn = mavutil.mavlink_connection(port, baud=baud, autoreconnect=False)
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
    session.set_message_interval(esc_id, 100000)   # 10 Hz
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
# Interactive REPL
# ---------------------------------------------------------------------------

_HELP = """
RAWES calibration tool.  Two long-running verbs (run, watch) handle all
time-bounded operations and log to simulation/logs/calibrate/*.csv.  The
rest are one-shot.

Long-running (always log; ESC or Ctrl-C aborts):
  run <name> [--duration N] [--trim K=V,...] [--gain K=V,...]
        Activate a Lua mode + arm via RAWES_ARM + stream observation rows.
        On exit (timer / ESC / Ctrl-C) every overridden param is restored.

        --duration N       run for N seconds; omit for unbounded (5-min ARM)
        --trim K=V,K=V     cyclic + collective held by the Lua; sent as
                           NAMED_VALUE_FLOAT to rawes.lua's MODE_PASSIVE /
                           MODE_YAW.  Repeatable.  **All values in degrees.**
                           The wire converts to radians automatically.
                             tlon  longitudinal cyclic [deg].  >0 = nose-down
                                   (forward-stick); <0 = nose-up.
                                   Range +/- H_CYC_MAX_cd/100 (passive: +/- 10).
                                   Typical bench: 0.5 .. 3 deg.
                             tlat  lateral cyclic [deg].  >0 = roll-right.
                                   Same range/limits as tlon.
                             col   IC collective [deg of blade pitch].
                                   COL_MIN_RAD (-0.28 rad = -16 deg)
                                   .. COL_MAX_RAD (+0.10 rad = +5.7 deg)
                                   linearly maps to RC3 PWM 1000..2000.
                                   Typical values:
                                     -5    zero-thrust / cruise neutral
                                     -8.6  modest negative (autorotation feed)
                                      0    neutral collective
                                     +3    light positive thrust
        --gain K=V,K=V     per-run AP param overrides (yaw mode only).
                           Repeatable.  Run `run` (no args) for the per-mode
                           gain-key table.
        --exclude-saturate (yaw mode only) After the run, print a PID-tuning
                           report computed ONLY from samples where the loop
                           was not saturated (output unclamped, integrator
                           not at IMAX).  Use when the motor is hitting its
                           limit so often that overall stats are dominated
                           by open-loop drift.
        --osc TARGET       Walk a sequence of trim setpoints, 5 s/step;
                           overrides --trim.  Targets:
                             all  full 13-step sweep through tlon/tlat/col
                                  extremes (~65 s)
                             s1   isolated S1 up/down (~25 s, 5 steps)
                             s2   isolated S2 up/down
                             s3   isolated S3 up/down (longitudinal axis)
                           The s1/s2/s3 sequences use mixer-isolated
                           combinations so the target servo dominates while
                           the other two stay near center.

                Modes (run with no args to see force_params per mode):
          passive   armed but quiet in GUIDED_NOGPS (matches the SITL passive
                    test).  Seeds the IC (RAWES_COL/RIC/PIC) and holds the IC
                    attitude via the GUIDED angle API; DDFP yaw motor stays
                    under AP + the Lua H_YAW_TRIM observer.
          steady    steady flight (alt hold + VZ PI collective)
          pumping   De Schutter pumping cycle
          landing   landing (reserved)

        Examples (IC angles in DEGREES):
          # Hold the level IC (roll=pitch=0, col=-8.6 deg) on the bench for 30 s
          run passive --duration 30 --trim col=-8.6

                    # Hold a fixed yaw IC as well
                    run passive --duration 20 --yaw 90 --trim col=-8.6

          # Hold a tilted IC: 3 deg roll, -25 deg pitch, IC collective
          run passive --duration 20 --roll 3 --pitch -25 --trim col=-8.6

          # Unbounded passive session (ESC to stop, 5-min RAWES_ARM fallback)
          run passive

          # Yaw tuning: gentler P, lower motor max, IC operating point loaded
          run yaw --duration 60 --gain p=0.015,i=0.005,imax=0.7,servo_max=1100 \\
                  --trim tlon=1.15,col=-8.6

          # Full oscillation sweep through every axis extreme (~65 s)
          run passive --osc all

          # Isolated S2 swashplate-servo test (~25 s, S2 dominant up/down)
          run passive --osc s2

  analyze yaw <csv> [--include-saturate]
      Offline PID-tuning report on a saved run_ctrl_*.csv (no FC connection
        required).  Default: filters out samples where the PID output or
        integrator was saturated (the closed loop wasn't operating).  Pass
        --include-saturate to score all samples together.

  analyze motor <log>
        Offline motor discrete-speed (PWM-quantisation) detector -- no FC
        connection required.  Takes a run/passive *.mavlink.jsonl (or its .csv
        sibling; the JSONL is auto-resolved).  Identifies the yaw limit cycle,
        overlays and coherently averages many matched cycles (phase-locked
        ensemble average) to pull the repeatable SERVO4-PWM -> yaw-response
        curve out of the chaotic body dynamics, then tests that high-SNR curve
        for discrete torque steps.  Reports the limit-cycle frequency, cycles
        averaged, the phase-averaged waveform, and a DISCRETE / continuous /
        INCONCLUSIVE verdict (with an open-loop-ramp recommendation when the
        closed-loop PWM excursion is too narrow to judge).

  watch <stream> [--duration N]
        Read-only observation; no state change.  Default duration 10 s.
        streams: servos    SERVO_OUTPUT_RAW for ch1..8
                 esc       ESC_TELEMETRY (rpm/volt/current/temp)
                 text      STATUSTEXT only
                 attitude  ATTITUDE (roll/pitch/yaw + body rates)
                 power     BATTERY_STATUS / SYS_STATUS (vbat / current / W)

One-shot:
  status                          Vehicle / battery / EKF / servos / key params
  set <name> <value>              Write a parameter (read-back verified)
  get <name>                      Read a parameter
  swash <coll%> [lon%] [lat%]     H3-120 manual mixer (-100..+100 each)
  swash range <min> <max>         Set H_COL_MIN / H_COL_MAX (heli swash range)
  swash neutral [n]               Drive S1/S2/S3 (or n) to 1500 us
  swash info                      Print current swashplate geometry + factors
  servo <ch> <pwm>                Set channel ch to pwm directly
  servo sweep <ch> [--step-ms N]  Slowly sweep ch: 1500 -> 2000 -> 1000 -> 1500
  servo hold <ch> <pwm> [--duration N]  Arm, hold ch at pwm for N s
  motor <pwm_us> [--duration N]   Arm (RAWES_ARM) + drive the motor output at
                                  pwm_us for N s (default 5).  DShot ESC self-arms
                                  from idle -- no ESC pre-arm hold.
                                  pwm_us must be within [SERVO<motor>_MIN, _MAX];
                                  >5% of that range prompts unless --force.
                                  Logs to CSV like `run`.
  motor off                       motor -> idle (off) + disarm immediately
  arm [--duration N]              ACRO + RAWES_ARM (no Lua mode change)
  disarm                          Disarm vehicle
  script upload <file>            Upload .lua to /APM/scripts and restart engine
  script list                     List /APM/scripts
  script remove <name>            Remove from /APM/scripts
    config check [--all]            Diff params against defaults
                                                                    default: rawes_common_defaults.parm overrides only
                                                                    --all: copter-heli.parm + rawes_common_defaults.parm
    config fix [--all]              Write the DIFFs (same scope rules as check)
    config show/apply               Compatibility aliases for check/fix
  logs list                       List all dataflash logs on the FC (id / size)
  logs fetch [--id N] [--dir D]   Download a dataflash .BIN log (default: latest)
                                  --id N   specific log id; omit for latest
                                  --dir D  destination dir (default: simulation/logs/calibrate)
  reboot                          Reboot ArduPilot
  ping [baud]                     Scan COM ports for ArduPilot heartbeats
  help                            Show this list
  quit                            Exit (REPL only)
"""


def _arm(session: RawesGCS, force: bool = False,
         timeout: float = 15.0, esc_arm: bool = True) -> bool:
    """
    Arm sequence:
      1. Set throttle RC override to 1000 (CH3 interlock low).
      2. Send MAV_CMD_COMPONENT_ARM_DISARM; wait for armed heartbeat.
    The DShot ESC self-arms from the idle throttle once armed -- no special
    ESC pre-arm pulse is needed.  Returns True if vehicle confirms armed.
    """
    print("  Setting throttle (CH3) override to 1000 ...")
    session.send_rc_override({3: 1000})
    time.sleep(0.5)

    print("  Sending arm command ...")
    param2 = 21196.0 if force else 0.0
    session._mav.mav.command_long_send(
        session._target_system, session._target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0,     # param1: 1 = arm
        param2,  # param2: 21196 = force-arm
        0, 0, 0, 0, 0,
    )

    deadline = time.monotonic() + timeout
    armed = False
    while time.monotonic() < deadline:
        session.send_rc_override({3: 1000})
        msg = session._recv(
            type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"],
            blocking=True, timeout=0.5,
        )
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

    # DShot ESCs self-arm from the idle throttle ArduPilot streams once armed, so
    # no special "hold min throttle" pre-arm pulse is needed (unlike the old PWM
    # REVVitRC path).  esc_arm is accepted for call-site compatibility only.
    _ = esc_arm
    return True


def _disarm(session: RawesGCS, timeout: float = 10.0,
            force: bool = False) -> bool:
    """Send disarm command. Returns True if vehicle confirms disarmed."""
    print("  Sending disarm command ...")
    param2 = 21196.0 if force else 0.0
    session._mav.mav.command_long_send(
        session._target_system, session._target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0,    # param1: 0 = disarm
        param2, # param2: 21196 = force-disarm
        0, 0, 0, 0, 0,
    )
    session.send_rc_override({})   # clear all RC overrides
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = session._recv(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                print("  [OK] Vehicle disarmed.")
                return True
    print("  [FAIL] Disarm timed out.")
    return False


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


# =============================================================================
# Unified command framework (run / watch + shared engine)
# =============================================================================
# Three concepts share one engine:
#   run <name>    -- activates a Lua mode (sets SCR_USER6), arms via RAWES_ARM,
#                    streams observation rows, logs to CSV, runs safety shutdown
#   watch <stream> -- read-only observation, no state change, logs to CSV
#   one-shot verbs -- set / get / swash / servo / motor / arm / disarm /
#                    script / config / reboot / ping / status
#
# Every long-running command logs to simulation/logs/calibrate/.

_LOG_DIR = os.path.join(_SIM_DIR, "logs", "calibrate")


def _log_path(verb: str, name: str) -> str:
    """Timestamped CSV path under simulation/logs/calibrate/.  Creates the dir."""
    os.makedirs(_LOG_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(_LOG_DIR, f"{verb}_{name}_{ts}.csv")


def _parse_kv_list(s: str) -> dict[str, float]:
    """Parse 'a=1,b=2.5' -> {'a': 1.0, 'b': 2.5}.  Trailing commas ignored."""
    out: dict[str, float] = {}
    for tok in s.split(","):
        tok = tok.strip()
        if not tok:
            continue
        if "=" not in tok:
            raise ValueError(f"key=value expected, got {tok!r}")
        k, _, v = tok.partition("=")
        out[k.strip().lower()] = float(v.strip())
    return out


def _parse_flags(tokens: list[str],
                 schema: dict[str, str]) -> tuple[list[str], dict]:
    """Split tokens into (positionals, flags).
    schema maps '--flag' -> 'float' | 'int' | 'kv' | 'bool' | 'str'.
    Unknown flags raise ValueError so typos surface immediately."""
    pos: list[str] = []
    flags: dict[str, object] = {}
    i = 0
    while i < len(tokens):
        t = tokens[i]
        if t.startswith("--"):
            kind = schema.get(t)
            if kind is None:
                raise ValueError(f"Unknown flag {t!r}  (valid: {', '.join(sorted(schema))})")
            if kind == "bool":
                flags[t] = True
                i += 1
                continue
            if i + 1 >= len(tokens):
                raise ValueError(f"Flag {t} needs a value")
            val = tokens[i + 1]
            if kind == "float":
                flags[t] = float(val)
            elif kind == "int":
                flags[t] = int(val)
            elif kind == "kv":
                # Repeatable: accumulate dicts into one.
                prev: dict[str, float] = flags.get(t, {})  # type: ignore[assignment]
                prev.update(_parse_kv_list(val))
                flags[t] = prev
            else:  # "str"
                flags[t] = val
            i += 2
        else:
            pos.append(t)
            i += 1
    return pos, flags


# -- Logging --------------------------------------------------------------

class _RunLog:
    """CSV log with metadata header.  Header is '# k: v' lines, then data row.

    Lifecycle:
        log = _RunLog.open("run", "yaw", meta={"duration_s": 30, ...})
        log.write_header(["t_s", "armed", "yaw_dps", "s4_us"])
        log.row([0.0, 1, +0.02, 800])
        log.close()
    """
    def __init__(self, path: str, fh, writer):
        self.path = path
        self._fh = fh
        self._w  = writer
        self.n_rows = 0

    @classmethod
    def open(cls, verb: str, name: str, meta: dict) -> "_RunLog":
        path = _log_path(verb, name)
        fh = open(path, "w", newline="")
        fh.write(f"# {verb}.csv -- written by calibrate.py {verb}\n")
        for k, v in meta.items():
            fh.write(f"# {k}: {v}\n")
        fh.write("#\n")
        w = csv.writer(fh)
        return cls(path, fh, w)

    def write_header(self, cols: list[str]) -> None:
        self._w.writerow(cols)

    def row(self, values: list) -> None:
        self._w.writerow(values)
        self.n_rows += 1

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


# -- Shared run engine: arm via RAWES_ARM, observe, safety shutdown ----

def _wait_for_armed(session: RawesGCS, timeout_s: float = 15.0) -> bool:
    """Block until armed heartbeat or timeout.  Prints STATUSTEXT inline."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = session._recv(type=["HEARTBEAT", "STATUSTEXT"],
                            blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"  [FC] {msg.text.rstrip()}")
        elif msg.get_type() == "HEARTBEAT":
            if bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
    return False


def _take_servo4(session: RawesGCS) -> "float | None":
    """Set SERVO<motor>_FUNCTION=0 (release from DDFP).  Returns saved value for restore."""
    servo_key = f"SERVO{SERVO_MOTOR}_FUNCTION"
    saved = session.get_param(servo_key)
    if saved is not None and saved != 0:
        session.set_param(servo_key, 0)
        print(f"  {servo_key} {saved:.0f} -> 0 (released from DDFP)")
        return float(saved)
    return None


def _ensure_passive_tail_setup(session: RawesGCS) -> None:
    """Ensure passive mode uses AP-owned DDFP tail control on channel 4."""
    expected_tail = 3.0
    tail = session.get_param("H_TAIL_TYPE")
    if tail is None:
        print("  [WARN] H_TAIL_TYPE unreadable; cannot enforce passive tail setup")
    elif abs(float(tail) - expected_tail) > 1e-4:
        ok = session.set_param("H_TAIL_TYPE", expected_tail)
        tag = "[OK]" if ok else "[FAIL]"
        print(f"  {tag} H_TAIL_TYPE: {tail:.6g} -> {expected_tail:.0f} (passive expects DDFP CW)")

    s4f = session.get_param(f"SERVO{SERVO_MOTOR}_FUNCTION")
    if s4f is None:
        print(f"  [WARN] SERVO{SERVO_MOTOR}_FUNCTION unreadable; cannot verify motor-channel ownership")
    elif int(round(float(s4f))) == 0:
        print(f"  [WARN] SERVO{SERVO_MOTOR}_FUNCTION=0 (output {SERVO_MOTOR} released). "
              "Passive expects AP-owned tail output.")


def _restore_servo4(session: RawesGCS, saved: "float | None") -> None:
    if saved is None:
        return
    servo_key = f"SERVO{SERVO_MOTOR}_FUNCTION"
    try:
        session.set_param(servo_key, saved)
        print(f"  [SAFETY] {servo_key} restored to {saved:.0f}")
    except Exception as e:
        print(f"  [SAFETY] failed to restore {servo_key}: {e}")


def _safety_shutdown(session: RawesGCS, *,
                     saved_servo4_fn: "float | None" = None,
                     saved_overrides: "dict[str, float] | None" = None,
                     skip_motor_off: bool = False) -> None:
    """Unified post-run shutdown.  Order: stop Lua -> wait -> motor off ->
    disarm -> restore SERVO4_FUNCTION -> restore param overrides.  Every step
    is best-effort: one failure does not skip the next."""
    print("  [SAFETY] shutting down ...")
    try:
        session.set_param("SCR_USER6", 0)
        print("  [SAFETY] SCR_USER6 -> 0 (Lua mode none)")
    except Exception as e:
        print(f"  [SAFETY] failed to set SCR_USER6=0: {e}")
    time.sleep(0.30)   # let SRV_Channels override timeout expire
    if not skip_motor_off:
        try:
            _send_set_servo(session, SERVO_MOTOR, MOTOR_OFF_US)
            print(f"  [SAFETY] SERVO{SERVO_MOTOR} -> {MOTOR_OFF_US} us (motor off)")
        except Exception as e:
            print(f"  [SAFETY] failed to drive SERVO{SERVO_MOTOR} off: {e}")
    try:
        if not _disarm(session, timeout=5.0):
            print("  [SAFETY] disarm not confirmed within 5 s -- retrying force-disarm")
            if not _disarm(session, timeout=5.0, force=True):
                print("  [SAFETY] force-disarm not confirmed")
    except Exception as e:
        print(f"  [SAFETY] disarm command failed: {e}")
    _restore_servo4(session, saved_servo4_fn)
    for param, orig in (saved_overrides or {}).items():
        try:
            session.set_param(param, orig)
            print(f"  [SAFETY] {param} restored to {orig:.6g}")
        except Exception as e:
            print(f"  [SAFETY] failed to restore {param}: {e}")


def _esc_check() -> bool:
    """Non-blocking ESC-key check.  Returns True if ESC seen."""
    seen = False
    while msvcrt.kbhit():
        if msvcrt.getch() == b"\x1b":
            seen = True
    return seen


def _poll_keys() -> "list[bytes]":
    """Non-blocking: drain and return all pending keypresses (one byte each)."""
    keys = []
    while msvcrt.kbhit():
        keys.append(msvcrt.getch())
    return keys


def _observation_loop(session: RawesGCS, *,
                      duration_s: "float | None",
                      msg_types: list[str],
                      streams: list[tuple],
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
    for s_id, hz in streams:
        session.request_stream(s_id, hz)

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
    deadline = (t0 + duration_s + 5.0) if duration_s else None
    last_print = -1.0
    aborted = False
    print("  Press ESC (or Ctrl-C) to abort.")
    try:
        while True:
            if deadline and time.monotonic() >= deadline:
                break
            keys = _poll_keys()
            if b"\x1b" in keys:
                aborted = True
                print("\n  [ESC] abort -- running safety shutdown ...")
                break
            if key_handler is not None:
                for k in keys:
                    key_handler(k)
            msg = session._recv(type=msg_types, blocking=True, timeout=0.1)
            t_rel = time.monotonic() - t0
            if on_tick is not None:
                on_tick(t_rel)
            if msg is not None:
                if msg.get_type() == "HEARTBEAT":
                    state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif msg.get_type() == "STATUSTEXT":
                    if not suppress_status:
                        text = msg.text.rstrip("\x00").strip()
                        if text:
                            state["pending_text"].append(text)
                else:
                    row = handle_msg(state, msg, t_rel)
                    if row is not None and log is not None:
                        log.row(row)
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


# -- Oscillation sequences for `--osc {all|s1|s2|s3}` -------------------
#
# Walk through a fixed set of trim setpoints, 5 s per step.  Each step
# tuple is (tlon_deg, tlat_deg, col_deg, label).
#
# `all` exercises one axis at a time (tlon/tlat/col extremes with center
# between).  `s1`/`s2`/`s3` use combinations that isolate one swash servo
# while keeping the other two near center -- derived from the H3-120
# mixer formula (see CLAUDE.md "Swashplate geometry"):
#
#     out_i = -sin(az_i)*0.45*tlat_norm - cos(az_i)*0.45*tlon_norm + coll_norm
#
# For each target servo S<i> we want out_i = Delta (~ 0.3) and the other
# two outputs = 0.  Solving the 2x2 system gives:
#
#     S1 alone:  coll_norm = D/3,  tlon_norm = -0.741*D,  tlat_norm = +1.282*D
#     S2 alone:  coll_norm = D/3,  tlon_norm = -0.741*D,  tlat_norm = -1.282*D
#     S3 alone:  coll_norm = D/3,  tlon_norm = +1.481*D,  tlat_norm = 0
#
# With H_CYC_MAX = 1000 cd (10 deg full stick) and the collective half-range
# of (COL_MAX - COL_MIN) / 2 ~ 10.8 deg, scaling Delta = 0.3 gives the
# numbers below.  Values stay well inside the +/-5 deg cyclic and -16..+5.7 deg
# collective bounds.
_OSC_BASE_COL_DEG = -8.6   # IC operating-point collective baseline
_OSC_DELTA_COL    =  1.08  # Half collective swing (deg) at Delta = 0.3

_OSCILLATE_STEPS_ALL = [
    # (tlon_deg, tlat_deg, col_deg, label)
    ( 0.0,  0.0,  -8.6, "center"),
    (+5.0,  0.0,  -8.6, "tlon +5 (nose-down)"),
    ( 0.0,  0.0,  -8.6, "center"),
    (-5.0,  0.0,  -8.6, "tlon -5 (nose-up)"),
    ( 0.0,  0.0,  -8.6, "center"),
    ( 0.0, +5.0,  -8.6, "tlat +5 (roll-right)"),
    ( 0.0,  0.0,  -8.6, "center"),
    ( 0.0, -5.0,  -8.6, "tlat -5 (roll-left)"),
    ( 0.0,  0.0,  -8.6, "center"),
    ( 0.0,  0.0,  +3.0, "col +3 (positive)"),
    ( 0.0,  0.0,  -8.6, "center"),
    ( 0.0,  0.0, -12.0, "col -12 (negative)"),
    ( 0.0,  0.0,  -8.6, "center"),
]

_OSCILLATE_STEPS_S1 = [
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                 "center"),
    ( -2.22,  +3.85, _OSC_BASE_COL_DEG + _OSC_DELTA_COL, "S1 UP"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                 "center"),
    ( +2.22,  -3.85, _OSC_BASE_COL_DEG - _OSC_DELTA_COL, "S1 DOWN"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                 "center"),
]
_OSCILLATE_STEPS_S2 = [
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                 "center"),
    ( -2.22,  -3.85, _OSC_BASE_COL_DEG + _OSC_DELTA_COL, "S2 UP"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                 "center"),
    ( +2.22,  +3.85, _OSC_BASE_COL_DEG - _OSC_DELTA_COL, "S2 DOWN"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                 "center"),
]
_OSCILLATE_STEPS_S3 = [
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                 "center"),
    ( +4.44,    0.0, _OSC_BASE_COL_DEG + _OSC_DELTA_COL, "S3 UP"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                 "center"),
    ( -4.44,    0.0, _OSC_BASE_COL_DEG - _OSC_DELTA_COL, "S3 DOWN"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                 "center"),
]

_OSCILLATE_TARGETS = {
    "all": _OSCILLATE_STEPS_ALL,
    "s1":  _OSCILLATE_STEPS_S1,
    "s2":  _OSCILLATE_STEPS_S2,
    "s3":  _OSCILLATE_STEPS_S3,
}
_OSCILLATE_STEP_S = 5.0


# -- `run <mode>` config table -------------------------------------------

# Each mode declares:
#   scr_user6      : SCR_USER6 value to activate
#   take_servo4    : True -> shuffle SERVO4_FUNCTION to 0 (and restore on exit)
#   gain_keys      : {flag-key -> AP_PARAM_NAME} for --gain (param overrides)
#   force_params   : {AP_PARAM_NAME -> value}; written on entry, restored on exit
#                    (e.g. H_FLYBAR_MODE=1 to bypass the rate PID for passive)
_TRIM_NVF = {"tlon": "RAWES_TLN", "tlat": "RAWES_TLT", "col": "RAWES_COL"}

# Default passive IC collective [deg blade pitch] when --trim col is not given.
# -8.6 deg ~ -0.15 rad, the RAWES operating-point collective.
_PASSIVE_IC_COL_DEG = -8.6

_RUN_MODES = {
    "none": {
        "scr_user6":   0,
        "take_servo4": False,
        # Lua is idle in mode 0; the AP yaw PID is off (P=I=D=0) and the DDFP
        # tail path (H_TAIL_TYPE=3) passes H_YAW_TRIM straight through to the yaw
        # motor.  The only tunable is that static trim, so --gain trim=<value>
        # sets a constant yaw-motor throttle while everything else stays quiet.
        "gain_keys": {"trim": "H_YAW_TRIM"},
        "doc":        "Lua idle (mode 0), armed-but-quiet; only --gain trim=<value> (H_YAW_TRIM) changes the static yaw-motor throttle.",
    },
    "passive": {
        "scr_user6":   3,
        # Match the SITL passive arming flow: boot into GUIDED_NOGPS and hold the
        # IC attitude via the GUIDED angle API -- NOT the legacy flybar/RC path.
        "flight_mode": 20,       # GUIDED_NOGPS (ArduCopter mode 20)
        "ic_seed":     True,     # seed RAWES_COL/RIC/PIC so PASSIVE holds the IC
        "take_servo4": False,    # DDFP yaw motor (Motor4) stays under AP +
                                 # the Lua H_YAW_TRIM observer -- do NOT reassign it.
        # We tune the Lua yaw PID (SCR_USER1/2/3) ONLY.  Force the AP ATC_RAT_YAW
        # PID to ZERO on entry so the Lua H_YAW_TRIM observer is the sole yaw
        # regulator (restored on exit).  No --gain here: passive does not tune the
        # AP loop -- use `run yaw` for that.
        "gain_keys": {},
        "force_params": {
            "ATC_RAT_YAW_P":  0.0,
            "ATC_RAT_YAW_I":  0.0,
            "ATC_RAT_YAW_D":  0.0,
            "ATC_RAT_YAW_FF": 0.0,
        },
        "doc":        "armed-but-quiet in GUIDED_NOGPS (matches the SITL passive test): seeds the IC (RAWES_COL/RIC/PIC) and holds the IC attitude via the GUIDED angle API.  Forces the AP yaw PID (ATC_RAT_YAW_P/I/D/FF) to ZERO so the Lua yaw PID (SCR_USER1/2/3) is the sole yaw regulator -- tune Lua ONLY.  IC via --trim col=<deg> --roll <deg> --pitch <deg>.",
    },
    "steady": {
        "scr_user6":  1,
        "take_servo4": False,
        "gain_keys":  {},
        "doc":        "steady flight: altitude hold + VZ PI collective",
    },
    "pumping": {
        "scr_user6":  1,
        "take_servo4": False,
        "gain_keys":  {},
        "doc":        "De Schutter pumping cycle (runs in steady mode; ground varies tension)",
    },
    "landing": {
        "scr_user6":  4,
        "take_servo4": False,
        "gain_keys":  {},
        "doc":        "landing (reserved)",
    },
}


# Sensible starting Lua yaw-PID gains (SCR_USER1/2/3 = KP/KI/KD).  `run` seeds
# these ONLY when all three are ~0 (fresh/unconfigured FC) so the vehicle holds
# yaw out of the box; any nonzero (tuned) value is preserved.  The values are
# sourced from rawes_common_defaults.parm (via _CONFIG_TARGET_PARAMS_COMMON) so
# calibrate.py and the SITL defaults are a single source of truth and cannot drift.
_YAW_PID_KEYS = ("SCR_USER1", "SCR_USER2", "SCR_USER3")
_YAW_PID_DEFAULTS = {
    k: float(_CONFIG_TARGET_PARAMS_COMMON[k])
    for k in _YAW_PID_KEYS
    if k in _CONFIG_TARGET_PARAMS_COMMON
}


def _seed_yaw_pid_defaults(session: RawesGCS) -> None:
    """Seed SCR_USER1/2/3 to the rawes_common_defaults.parm values iff all three
    are ~0 (unconfigured).

    Preserves an already-tuned set (any nonzero gain) so iterative bench tuning
    across runs is not clobbered.
    """
    if len(_YAW_PID_DEFAULTS) != len(_YAW_PID_KEYS):
        missing = [k for k in _YAW_PID_KEYS if k not in _YAW_PID_DEFAULTS]
        print(f"  [WARN] {', '.join(missing)} absent from rawes_common_defaults.parm "
              "-- yaw PID not seeded")
        return
    cur = {n: session.get_param(n) for n in _YAW_PID_DEFAULTS}
    if any(v is None for v in cur.values()):
        print("  [WARN] could not read SCR_USER1/2/3 -- yaw PID not seeded")
        return
    if all(abs(float(v)) < 1e-9 for v in cur.values()):
        print("  Seeding Lua yaw PID from rawes_common_defaults.parm "
              "(SCR_USER1/2/3 were all 0):")
        for n, val in _YAW_PID_DEFAULTS.items():
            session.set_param(n, val)
            print(f"    {n} = {val:g}")
    else:
        kept = "/".join(f"{float(cur[n]):.4g}" for n in _YAW_PID_KEYS)
        print(f"  Lua yaw PID gains kept (SCR_USER1/2/3 = {kept})")


# AP yaw-rate PID params that MUST be zero while tuning the Lua yaw loop alone.
_AP_YAW_ZERO_PARAMS = ("ATC_RAT_YAW_P", "ATC_RAT_YAW_I",
                       "ATC_RAT_YAW_D", "ATC_RAT_YAW_FF")


def _verify_passive_yaw_setup(session: RawesGCS) -> bool:
    """Confirm the passive-mode yaw config before the run: the AP ATC_RAT_YAW
    PID must read back all ~0 (so the Lua H_YAW_TRIM observer is the sole yaw
    regulator), and report the Lua yaw-rate PID gains (SCR_USER1/2/3 = KP/KI/KD)
    that we are about to tune.  Returns True iff the AP yaw PID is all zero.
    """
    print("  Yaw setup check (tuning Lua ONLY -- AP yaw PID must be 0):")
    ok = True
    for n in _AP_YAW_ZERO_PARAMS:
        v = session.get_param(n)
        if v is None:
            print(f"    [WARN] {n}: unreadable")
            ok = False
            continue
        good = abs(float(v)) < 1e-9
        ok = ok and good
        tag = "[OK]  " if good else "[FAIL]"
        print(f"    {tag} {n} = {float(v):.6g}   (expect 0)")
    kp = session.get_param("SCR_USER1")
    ki = session.get_param("SCR_USER2")
    kd = session.get_param("SCR_USER3")
    _f = lambda x: "n/a" if x is None else f"{float(x):.4g}"
    print(f"    Lua yaw PID (tune these): SCR_USER1(KP)={_f(kp)}  "
          f"SCR_USER2(KI)={_f(ki)}  SCR_USER3(KD)={_f(kd)}")
    if not ok:
        print("    [WARN] AP yaw PID is NOT all zero -- Lua tuning will be "
              "contaminated by the AP rate loop.  Aborting recommended.")
    return ok


def _cmd_run(session: RawesGCS, args: list[str]) -> None:
    """run <mode> [--duration N] [--trim K=V,...] [--gain K=V,...] [--osc TARGET]"""
    schema = {
        "--duration":         "float",
        "--trim":             "kv",
        "--gain":             "kv",
        "--osc":              "str",
        "--yaw":              "float",   # passive fixed yaw IC [deg] (RAWES_YIC)
        "--roll":             "float",   # passive IC roll  [deg] (RAWES_RIC)
        "--pitch":            "float",   # passive IC pitch [deg] (RAWES_PIC)
        "--rc":               "bool",    # keep RC_CHANNELS stream (mixer diagnosis)
        "--noluayaw":         "bool",    # disable Lua yaw-trim FF (RAWES_YFK=0)
        "--exclude-saturate": "bool",
    }
    if not args:
        print("  Usage: run <name> [--duration N] [--trim K=V,...] "
              "[--gain K=V,...] [--osc {all|s1|s2|s3}]")
        print("  Modes:")
        for name, cfg in _RUN_MODES.items():
            print(f"    {name:<8} -- {cfg['doc']}")
        return
    try:
        pos, flags = _parse_flags(args, schema)
    except ValueError as e:
        print(f"  Error: {e}"); return
    if len(pos) != 1:
        print("  Usage: run <name> [--duration N] [--trim K=V,...] "
              "[--gain K=V,...] [--osc {all|s1|s2|s3}]")
        return
    name = pos[0].lower()
    cfg = _RUN_MODES.get(name)
    if cfg is None:
        print(f"  Unknown mode {name!r}  (valid: {', '.join(_RUN_MODES)})")
        return
    duration         = flags.get("--duration")
    trim             = flags.get("--trim", {}) or {}
    gain             = flags.get("--gain", {}) or {}
    osc              = flags.get("--osc")
    exclude_saturate = bool(flags.get("--exclude-saturate", False))
    if exclude_saturate and name != "yaw":
        print(f"  [WARN] --exclude-saturate is yaw-mode-only; ignored for {name!r}")
        exclude_saturate = False

    # `--osc TARGET` walks a step sequence (5 s/step), sending fresh trim
    # NVFs on each transition.  Targets:
    #   all  -- 13-step sweep through tlon/tlat/col extremes (~65 s)
    #   s1   -- 5-step isolated S1 up/down (~25 s)
    #   s2   -- 5-step isolated S2 up/down
    #   s3   -- 5-step isolated S3 up/down
    # When --osc is set, --trim is ignored.
    osc_steps = None
    if osc is not None:
        osc = osc.lower()
        if osc not in _OSCILLATE_TARGETS:
            print(f"  Unknown --osc target {osc!r}  "
                  f"(valid: {', '.join(_OSCILLATE_TARGETS)})")
            return
        osc_steps = _OSCILLATE_TARGETS[osc]
        if duration is None:
            duration = len(osc_steps) * _OSCILLATE_STEP_S
        if trim:
            print(f"  [WARN] --trim {list(trim)} ignored because --osc is set")
            trim = {}

    # Validate trim keys
    bad = [k for k in trim if k not in _TRIM_NVF]
    if bad:
        print(f"  Unknown --trim keys: {bad}  (valid: {', '.join(_TRIM_NVF)})")
        return
    # Validate gain keys
    gain_map = cfg["gain_keys"]
    bad = [k for k in gain if k not in gain_map]
    if bad:
        if not gain_map:
            print(f"  Mode {name!r} does not accept --gain")
        else:
            print(f"  Unknown --gain keys for {name!r}: {bad}  (valid: {list(gain_map)})")
        return

    # Apply per-run param overrides (restored on exit)
    saved_overrides: dict[str, float] = {}
    if gain:
        print("  Per-run overrides:")
        for k, v in gain.items():
            ap_name = gain_map[k]
            orig = session.get_param(ap_name)
            if orig is None:
                print(f"    [WARN] {ap_name}: could not read -- skipping")
                continue
            saved_overrides[ap_name] = float(orig)
            ok = session.set_param(ap_name, float(v))
            tag = "[OK]  " if ok else "[FAIL]"
            print(f"    {tag} {ap_name}: {orig:.6g} -> {v:.6g}")

    # Mode-required param overrides (e.g. H_FLYBAR_MODE=1 for passthrough).
    # Restored on exit via the same saved_overrides path.
    force_params = cfg.get("force_params", {})
    if force_params:
        print("  Mode-required overrides:")
        for ap_name, target in force_params.items():
            orig = session.get_param(ap_name)
            if orig is None:
                print(f"    [WARN] {ap_name}: could not read -- skipping")
                continue
            if ap_name not in saved_overrides:
                saved_overrides[ap_name] = float(orig)
            ok = session.set_param(ap_name, float(target))
            tag = "[OK]  " if ok else "[FAIL]"
            print(f"    {tag} {ap_name}: {orig:.6g} -> {target}")

    # Passive mode requires AP-owned DDFP tail control on channel 4.
    if name == "passive":
        _ensure_passive_tail_setup(session)

    # SERVO4 ownership shuffle if the mode needs it
    saved_fn = _take_servo4(session) if cfg["take_servo4"] else None

    # Activate Lua mode
    session.set_param("SCR_USER6", cfg["scr_user6"])
    print(f"  SCR_USER6 -> {cfg['scr_user6']} ({name} mode)")

    # Seed the Lua yaw-rate PID gains to a sensible starting point on first use
    # (modes that run the yaw PID: passive=3, steady/pumping=1).  Preserves any
    # already-tuned gains.
    if cfg["scr_user6"] in (1, 3):
        _seed_yaw_pid_defaults(session)

    # Passive tunes the Lua yaw loop ONLY: confirm the AP yaw PID was forced to
    # zero (via force_params above) and report the Lua gains we are tuning.
    if name == "passive":
        _verify_passive_yaw_setup(session)

    # Flight mode required by this Lua mode (e.g. GUIDED_NOGPS for passive) --
    # matches the SITL passive arming flow.  Set BEFORE arming so we arm in the
    # right mode, just like the SITL test (INITIAL_MODE=GUIDED_NOGPS).
    _fm = cfg.get("flight_mode")
    if _fm is not None:
        session.set_mode(_fm)
        print(f"  Flight mode -> {_COPTER_MODES.get(_fm, _fm)} ({_fm})")

    # Seed the IC (RAWES_COL/RIC/PIC) BEFORE arming so PASSIVE holds a defined
    # attitude (mirrors the SITL passive_init seed).  Collective comes from
    # --trim col (deg blade pitch), roll/pitch from --roll/--pitch (deg), yaw
    # from --yaw (deg), all sent on the wire in radians.  Without a seed PASSIVE
    # just zero-rates.
    if cfg.get("ic_seed"):
        col_deg = float(trim.get("col", _PASSIVE_IC_COL_DEG))
        print("  Seeding IC (deg -> rad on the wire):")
        session.send_named_float("RAWES_COL", math.radians(col_deg))
        print(f"    RAWES_COL = {col_deg:+7.3f} deg  ({math.radians(col_deg):+.4f} rad)")

        if "--yaw" in flags:
            yaw_deg = float(flags["--yaw"])
            session.send_named_float("RAWES_YIC", math.radians(yaw_deg))
            print(f"    RAWES_YIC = {yaw_deg:+7.3f} deg  ({math.radians(yaw_deg):+.4f} rad)")

        # Only send RIC/PIC if the user explicitly provided --roll/--pitch.
        if "--roll" in flags:
            roll_deg = float(flags["--roll"])
            session.send_named_float("RAWES_RIC", math.radians(roll_deg))
            print(f"    RAWES_RIC = {roll_deg:+7.3f} deg  ({math.radians(roll_deg):+.4f} rad)")
        if "--pitch" in flags:
            pitch_deg = float(flags["--pitch"])
            session.send_named_float("RAWES_PIC", math.radians(pitch_deg))
            print(f"    RAWES_PIC = {pitch_deg:+7.3f} deg  ({math.radians(pitch_deg):+.4f} rad)")
        # col was consumed by the IC seed -- don't re-send it via the trim block.
        trim.pop("col", None)

    # Send NVF trims.  --trim values are user-facing DEGREES; convert to
    # radians for the wire (rawes.lua receives RAWES_TLN/TLT/COL in radians).
    if trim:
        print("  Sending trim NVFs (deg -> rad on the wire):")
        for k, v_deg in trim.items():
            v_rad = math.radians(float(v_deg))
            session.send_named_float(_TRIM_NVF[k], v_rad)
            print(f"    {_TRIM_NVF[k]} = {v_deg:+7.3f} deg  ({v_rad:+.4f} rad)")

    # Optionally disable the Lua yaw-trim feedforward so the AP yaw loop is the
    # ONLY thing driving SERVO4 (clean P-tuning isolation).  RAWES_YFK<=0 makes
    # run_yaw_trim_ff return early (stops writing H_YAW_TRIM); done BEFORE arming
    # so the Lua never writes the trim, then H_YAW_TRIM is zeroed and sticks.
    no_lua_yaw = bool(flags.get("--noluayaw", False))
    if no_lua_yaw:
        session.send_named_float("RAWES_YFK", 0.0)
        session.set_param("H_YAW_TRIM", 0.0)
        print("  Lua yaw-trim DISABLED (RAWES_YFK=0, H_YAW_TRIM=0) -- AP yaw loop only")

    # Arm. Passive mode keeps channel 4 under AP/Lua tail ownership, so skip
    # direct DO_SET_SERVO pre-arm pulses on SERVO4 to avoid ownership conflicts.
    esc_arm = (name != "passive")
    if not _arm(session, force=True, esc_arm=esc_arm):
        _safety_shutdown(session, saved_servo4_fn=saved_fn,
                         saved_overrides=saved_overrides)
        return
    print("  [OK] Armed.")

    # Snapshot key params for the log header
    meta = {
        "verb":            "run",
        "name":            name,
        "duration_s":      duration if duration is not None else "",
        "trim_deg":        ", ".join(f"{k}={v}" for k, v in trim.items()),
        "gain":            ", ".join(f"{k}={v}" for k, v in gain.items()),
        "osc":             osc if osc is not None else "",
        "run_start_local": datetime.now().isoformat(timespec="seconds"),
        "run_start_utc":   datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "SCR_USER6":       cfg["scr_user6"],
    }
    # Add per-mode AP param snapshot
    for ap_name in gain_map.values():
        v = session.get_param(ap_name)
        meta[ap_name] = float(v) if v is not None else ""

    log = _RunLog.open("run", name, meta)
    print(f"  Logging to {log.path}")

    # Parallel MAVLink traffic log (rx + tx) alongside the CSV, for diagnosing
    # what actually went over the wire (NVFs sent, YFF telemetry received, etc.).
    mavlog_path = log.path[:-4] + ".mavlink.jsonl" if log.path.endswith(".csv") \
        else log.path + ".mavlink.jsonl"
    session.start_mavlog(mavlog_path)
    print(f"  MAVLink log: {mavlog_path}")

    # Build the oscillate tick callback if --osc was set
    on_tick = None
    if osc_steps is not None:
        on_tick = _make_oscillate_tick(session, osc_steps)
        total_s = len(osc_steps) * _OSCILLATE_STEP_S
        print(f"  Oscillate target={osc!r}: walking {len(osc_steps)} steps "
              f"x {_OSCILLATE_STEP_S:.0f}s each (total {total_s:.0f}s).")

    try:
        _run_observation(session, name, duration, log, on_tick=on_tick,
                         keep_rc=bool(flags.get("--rc", False)))
    finally:
        session.stop_mavlog()
        log.close()
        print(f"  Wrote {log.n_rows} rows to {log.path}")
        if no_lua_yaw:
            session.send_named_float("RAWES_YFK", 1.0)   # re-enable Lua yaw-trim FF
            print("  Lua yaw-trim re-enabled (RAWES_YFK=1)")
        _safety_shutdown(session, saved_servo4_fn=saved_fn,
                         saved_overrides=saved_overrides)
    if exclude_saturate:
        _analyze_yaw_csv(log.path, exclude_saturate=True)
    print("  Done.")


def _make_oscillate_tick(session: RawesGCS, steps: list):
    """Return an on_tick(t_rel) callback that advances through `steps`
    (a list of (tlon_deg, tlat_deg, col_deg, label) tuples) at
    _OSCILLATE_STEP_S sec/step.  Sends RAWES_TLN/TLT/COL NVFs on each
    step boundary.  Once the sequence ends the callback is a no-op (the
    duration timer in _observation_loop expires shortly after)."""
    last_step = [None]
    def _tick(t_rel: float) -> None:
        idx = int(t_rel / _OSCILLATE_STEP_S)
        if idx >= len(steps):
            return
        if idx == last_step[0]:
            return
        last_step[0] = idx
        tlon_d, tlat_d, col_d, label = steps[idx]
        session.send_named_float("RAWES_TLN", math.radians(tlon_d))
        session.send_named_float("RAWES_TLT", math.radians(tlat_d))
        session.send_named_float("RAWES_COL", math.radians(col_d))
        print(f"  [{t_rel:6.1f}s] osc {idx+1}/{len(steps)}  "
              f"tlon={tlon_d:+5.1f}  tlat={tlat_d:+5.1f}  col={col_d:+5.1f}  "
              f"({label})")
    return _tick


def _run_observation(session: RawesGCS, mode_name: str,
                     duration: "float | None", log: _RunLog,
                     on_tick=None, keep_rc: bool = False) -> None:
    """Generic observation loop for `run` modes.  Stream + columns are the
    same across all modes; the row content is whatever the FC reports.  Per-
    mode NVFs (e.g. YAW_I, YAW_OUT) appear as columns when emitted.

    on_tick(t_rel) is called once per loop iteration (~10 Hz); used by
    oscillate mode to advance the trim sequence.

    Serial bandwidth is tight (57600 SiK ~= 5760 B/s, ~70% used by default),
    which drops NVF/telemetry.  So we trim streams we do not need for tuning:
    AHRS2 (EXTRA3) off, EXTENDED_STATUS down to 1 Hz, and RC_CHANNELS disabled
    (keeping SERVO_OUTPUT_RAW in the same stream) unless keep_rc is set."""
    cols = ["t_s", "armed", "roll_deg", "pitch_deg", "yaw_deg", "yaw_rate_dps",
            "ch1_us", "ch2_us", "ch3_us", "ch4_us",
            "s1_us", "s2_us", "s3_us", "mot_us",
            "vbat_v", "current_a",
            "yff_t_lua", "yff_i_lua", "yff_gz_lua",
            "yff_kp_lua", "yff_ki_lua", "yff_kd_lua",
            "erpm", "mech_rpm", "rotor_rpm"]

    # Live table shows the yaw-PID telemetry (out=H_YAW_TRIM output, I=integral)
    # plus the swashplate PWMs (s1..s3) and the GB4008 motor output (mot =
    # SERVO_MOTOR, bidir DShot) with its 5 s rolling average, and rotor RPM.
    # The full CSV also has ch1..ch4, the streamed gains (yff_kp/ki/kd) and the
    # eRPM / motor-shaft RPM so each row records the tuning point.
    print_cols = ["t(s)", "armed", "yaw(d)", "yrate", "out", "I",
                  "s1", "s2", "s3", "mot", "mot~5s", "V", "A", "mRPM"]

    mot_window_s = 5.0   # rolling average window for motor (SERVO_MOTOR) PWM

    if mode_name == "none":
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
    else:
        # Live tuning of the Lua yaw-rate PID gains (SCR_USER1/2/3 = KP/KI/KD).
        #   P: '-' / '='     I: '[' / ']'     D: ',' / '.'
        # These are our own SCR_USER params (persist; reset on FC reboot); the run
        # prints the final values so you can keep them.  (Disable the AP yaw PID
        # with --gain p=0,i=0,d=0 so the Lua PID owns yaw.)
        _PID_KEYS = {
            "P": {"param": "SCR_USER1", "step": 0.005},
            "I": {"param": "SCR_USER2", "step": 0.005},
            "D": {"param": "SCR_USER3", "step": 0.0005},
        }
        for _g in _PID_KEYS.values():
            _v = session.get_param(_g["param"])
            _g["val"] = float(_v) if _v is not None else 0.0
        print(f"  Lua yaw PID: P={_PID_KEYS['P']['val']:.4f} "
              f"I={_PID_KEYS['I']['val']:.4f} D={_PID_KEYS['D']['val']:.4f}")
        print("  tune keys:  P '-'/'='    I '['/']'    D ','/'.'")
        _PID_KEYMAP = {
            b"-": ("P", -1), b"=": ("P", +1),
            b"[": ("I", -1), b"]": ("I", +1),
            b",": ("D", -1), b".": ("D", +1),
        }

        def key_handler(k: bytes) -> None:
            m = _PID_KEYMAP.get(k)
            if m is None:
                return
            name, sign = m
            g = _PID_KEYS[name]
            old = g["val"]
            new = old + sign * g["step"]
            if new < 0.0:
                new = 0.0
            ok = session.set_param(g["param"], new)
            g["val"] = new
            tag = "" if ok else "  [FAIL]"
            print(f"  YAW {name} {old:.4f} -> {new:.4f}{tag}")

    state = {
        "roll": None, "pitch": None, "yaw": None, "yaw_rate": None,
        "ch1": None, "ch2": None, "ch3": None, "ch4": None,
        "s1": None, "s2": None, "s3": None, "smot": None,
        "smot_hist": [],
        "mrpm_hist": [],    # (t_rel, mech_rpm) for the 5 s rolling average on screen
        "vbat": None, "curr": None,
        "erpm": None,
        "yff_t": None, "yff_i": None, "yff_gz": None,
        "yff_kp": None, "yff_ki": None, "yff_kd": None,
        "yff_t_ts": None, "yff_i_ts": None, "yff_gz_ts": None,
    }

    def handle_msg(st, msg, t_rel):
        mt = msg.get_type()
        if mt == "ATTITUDE":
            state["roll"]  = math.degrees(msg.roll)
            state["pitch"] = math.degrees(msg.pitch)
            state["yaw"]   = math.degrees(msg.yaw)
            state["yaw_rate"] = math.degrees(msg.yawspeed)
            # Emit one CSV row per ATTITUDE message (typically 10-50 Hz)
            _erpm, _mech, _rotor = _rpm_triplet(state["erpm"])
            return [
                f"{t_rel:.4f}", int(st["armed"]),
                _fmt(state["roll"]), _fmt(state["pitch"]), _fmt(state["yaw"]),
                _fmt(state["yaw_rate"]),
                state["ch1"], state["ch2"], state["ch3"], state["ch4"],
                state["s1"], state["s2"], state["s3"], state["smot"],
                _fmt(state["vbat"]), _fmt(state["curr"]),
                _fmt(state["yff_t"]), _fmt(state["yff_i"]), _fmt(state["yff_gz"]),
                _fmt(state["yff_kp"]), _fmt(state["yff_ki"]), _fmt(state["yff_kd"]),
                _fmt(_erpm), _fmt(_mech), _fmt(_rotor),
            ]
        elif mt == "RC_CHANNELS":
            state["ch1"] = getattr(msg, "chan1_raw", None)
            state["ch2"] = getattr(msg, "chan2_raw", None)
            state["ch3"] = getattr(msg, "chan3_raw", None)
            state["ch4"] = getattr(msg, "chan4_raw", None)
        elif mt == "SERVO_OUTPUT_RAW":
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
        elif mt == "BATTERY_STATUS":
            cells = [v for v in msg.voltages if v != 65535]
            if cells:
                state["vbat"] = sum(cells) / 1000.0
            if msg.current_battery >= 0:
                state["curr"] = msg.current_battery / 100.0
        elif mt == "SYS_STATUS":
            if state["vbat"] is None and msg.voltage_battery != 65535:
                state["vbat"] = msg.voltage_battery / 1000.0
            if state["curr"] is None and msg.current_battery >= 0:
                state["curr"] = msg.current_battery / 100.0
        elif mt in _ESC_TELEM_MSGS:
            erpm = _esc_erpm(msg, MOTOR_ESC_CHANNEL)
            if erpm is not None:
                state["erpm"] = erpm
                _, _m, _ = _rpm_triplet(erpm)
                if _m is not None:
                    state["mrpm_hist"].append((t_rel, _m))
                    cutoff = t_rel - mot_window_s
                    while state["mrpm_hist"] and state["mrpm_hist"][0][0] < cutoff:
                        state["mrpm_hist"].pop(0)
        elif mt == "NAMED_VALUE_FLOAT":
            nm = msg.name.rstrip("\x00").strip() if isinstance(msg.name, str) else \
                 msg.name.decode("ascii", errors="replace").rstrip("\x00").strip()
            if nm == "YFF_T":
                state["yff_t"] = float(msg.value)
                state["yff_t_ts"] = t_rel
            elif nm == "YFF_I":
                state["yff_i"] = float(msg.value)
                state["yff_i_ts"] = t_rel
            elif nm == "YFF_GZ":
                state["yff_gz"] = float(msg.value)
                state["yff_gz_ts"] = t_rel
            elif nm == "YFF_KP":
                state["yff_kp"] = float(msg.value)
            elif nm == "YFF_KI":
                state["yff_ki"] = float(msg.value)
            elif nm == "YFF_KD":
                state["yff_kd"] = float(msg.value)
        return None

    def render_row(st, t_rel):
        def _fresh_or_stale(value, value_ts, fmt):
            if value is None:
                return None
            if value_ts is None or (t_rel - value_ts) > 2.0:
                return "stale"
            return fmt(value)

        yaw_s = f"{state['yaw']:+6.1f}" if state["yaw"] is not None else None
        yrate_s = f"{state['yaw_rate']:+6.1f}" if state["yaw_rate"] is not None else None
        out_s = _fresh_or_stale(state["yff_t"], state["yff_t_ts"], lambda v: f"{v:+.3f}")
        i_s = _fresh_or_stale(state["yff_i"], state["yff_i_ts"], lambda v: f"{v:+.3f}")
        mot_avg_s = None
        if state["smot_hist"]:
            mot_avg_s = f"{sum(v for _, v in state['smot_hist']) / len(state['smot_hist']):.0f}"
        _e, _m, _rotor = _rpm_triplet(state["erpm"])
        mrpm_avg_s = None
        if state["mrpm_hist"]:
            mrpm_avg_s = f"{sum(v for _, v in state['mrpm_hist']) / len(state['mrpm_hist']):.0f}"
        return [
            f"{t_rel:.1f}",
            "YES" if st["armed"] else "no",
            yaw_s,
            yrate_s,
            out_s,
            i_s,
            state["s1"], state["s2"], state["s3"], state["smot"], mot_avg_s,
            f"{state['vbat']:.2f}" if state["vbat"] is not None else None,
            f"{state['curr']:.2f}" if state["curr"] is not None else None,
            mrpm_avg_s,
        ]

    def _trim_streams() -> None:
        # Runs right after the stream requests so the RC_CHANNELS disable wins
        # over the RC_CHANNELS stream (which we keep for SERVO_OUTPUT_RAW).
        # Also request the motor's ESC telemetry (bidir DShot RPM) at 5 Hz.
        _esc_name, _esc_id = _esc_telem_msg_for_channel(MOTOR_ESC_CHANNEL)
        session.set_message_interval(_esc_id, 200000)   # 5 Hz
        if not keep_rc:
            session.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, -1)
            print("  Stream trim: RC_CHANNELS off, AHRS2 off, EXTENDED_STATUS 1 Hz "
                  f"(use --rc to keep RC_CHANNELS); {_esc_name} 5 Hz")
        else:
            print("  Stream trim: AHRS2 off, EXTENDED_STATUS 1 Hz (RC_CHANNELS kept); "
                  f"{_esc_name} 5 Hz")

    _observation_loop(
        session,
        duration_s=duration,
        msg_types=["ATTITUDE", "RC_CHANNELS", "SERVO_OUTPUT_RAW",
                   "HEARTBEAT", "STATUSTEXT", "BATTERY_STATUS", "SYS_STATUS",
                   "NAMED_VALUE_FLOAT",
                   _esc_telem_msg_for_channel(MOTOR_ESC_CHANNEL)[0]],
        streams=[
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,          25),
            (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,     25),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,          0),
        ],
        handle_msg=handle_msg,
        render_row=render_row,
        header_cols=cols,
        header_print_cols=print_cols,
        log=log,
        on_tick=on_tick,
        suppress_status=True,
        key_handler=key_handler,
        setup_hook=_trim_streams,
    )
    # Restore telemetry the trim disabled (best-effort; resets on FC reboot).
    if not keep_rc:
        session.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 0)
    session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2)
    session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2)

    # Report final Lua yaw-PID gains (SCR_USER params persist across the session).
    if mode_name == "none":
        _tv = session.get_param("H_YAW_TRIM")
        if _tv is not None:
            _pwm = 1000 + float(_tv) * 1000.0
            print(f"  Yaw trim final: H_YAW_TRIM={float(_tv):.4f}  (~{_pwm:.0f} us)")
    else:
        print(f"  Lua yaw PID final: P={_PID_KEYS['P']['val']:.4f} "
              f"I={_PID_KEYS['I']['val']:.4f} D={_PID_KEYS['D']['val']:.4f}  "
              f"(SCR_USER1/2/3 -- persisted)")


def _fmt(v):
    """CSV cell formatter: '' for None, otherwise the float's natural repr."""
    if v is None:
        return ""
    if isinstance(v, float):
        return f"{v:.6g}"
    return v


# -- `run yaw` post-run PID tuning analysis -------------------------------

def _read_yaw_csv(csv_path: str) -> tuple[dict, list[dict]]:
    """Parse a run_yaw_*.csv file.  Returns (meta, rows).
       meta: {key: str} from "# key: value" header lines.
       rows: list of {col: float|None} dicts (numeric where parseable)."""
    meta: dict[str, str] = {}
    rows: list[dict] = []
    header_cols: list[str] | None = None
    with open(csv_path, "r", newline="") as fh:
        for line in fh:
            line = line.rstrip("\r\n")
            if not line:
                continue
            if line.startswith("#"):
                body = line.lstrip("#").strip()
                if ":" in body:
                    k, _, v = body.partition(":")
                    meta[k.strip()] = v.strip()
                continue
            if header_cols is None:
                header_cols = [c.strip() for c in line.split(",")]
                continue
            cells = line.split(",")
            row: dict = {}
            for col, cell in zip(header_cols, cells):
                cell = cell.strip()
                if cell == "":
                    row[col] = None
                else:
                    try:
                        row[col] = float(cell)
                    except ValueError:
                        row[col] = cell
            rows.append(row)
    return meta, rows


def _meta_float(meta: dict, key: str, default: float) -> float:
    """Pull a float from the CSV header; return default if missing/unparseable."""
    raw = meta.get(key)
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _analyze_yaw_csv(csv_path: str, exclude_saturate: bool = True) -> None:
    """Print a PID-tuning report for a run_yaw_*.csv.

    With exclude_saturate=True (the --exclude-saturate path), residual-error
    metrics are computed ONLY from samples where neither the integrator nor
    the output is clamped -- so they reflect closed-loop dynamics rather than
    open-loop drift while the actuator is pegged.

    Yaw rate is recovered from the logged PID internals:
        output_unclamp = kp*err + i + kd*derr/dt + trim
    Assuming kd=0 (the production case), err = (output_unclamp - i - trim)/kp
    and yaw_rate = -err.  This matches the gyro signal MODE_YAW uses, so we
    do not need a separate gyro log column."""
    print("")
    print("  PID tuning analysis (yaw mode)" +
          ("  --  excluding saturated samples" if exclude_saturate else ""))
    print("  " + "=" * 60)

    try:
        meta, rows = _read_yaw_csv(csv_path)
    except OSError as e:
        print(f"  [FAIL] cannot read {csv_path}: {e}")
        return

    kp        = _meta_float(meta, "ATC_RAT_YAW_P",    0.1)
    ki        = _meta_float(meta, "ATC_RAT_YAW_I",    0.0)
    kd        = _meta_float(meta, "ATC_RAT_YAW_D",    0.0)
    imax      = _meta_float(meta, "ATC_RAT_YAW_IMAX", 0.7)
    trim      = _meta_float(meta, "H_YAW_TRIM",       0.0)
    servo_min = _meta_float(meta, f"SERVO{SERVO_MOTOR}_MIN",
                            _meta_float(meta, "SERVO4_MIN", 800.0))
    servo_max = _meta_float(meta, f"SERVO{SERVO_MOTOR}_MAX",
                            _meta_float(meta, "SERVO4_MAX", 2000.0))

    print(f"  source ........ {csv_path}")
    print(f"  gains ......... P={kp:.4g}  I={ki:.4g}  D={kd:.4g}"
          f"  IMAX={imax:.3g}  TRIM={trim:.3g}")
    print(f"  SERVO{SERVO_MOTOR} range .. {servo_min:.0f} .. {servo_max:.0f} us")

    # Drop early rows that arrived before the first NVF pair.
    samples = [r for r in rows
               if isinstance(r.get("yaw_out_lua"), (int, float))
               and isinstance(r.get("yaw_i_lua"), (int, float))]
    if not samples:
        print("  [FAIL] no PID rows found (yaw_out_lua / yaw_i_lua missing).")
        return

    # Saturation = the PID demand could not be applied on that Lua tick.
    # Uses Lua-internal signals (same tick) rather than s4_us (25 Hz async):
    #   - output_unclamp >= 1.0  (output clamped at top)
    #   - output_unclamp <= 0.0  (output clamped at zero)
    #   - yaw_i within I_EPS of IMAX (integrator pegged at top)
    #   - yaw_i ~ 0 AND output ~ 0   (both pegged at zero, one-sided clamp)
    I_EPS  = max(1e-4, abs(imax) * 1e-3)
    OUT_HI = 1.0
    OUT_LO = 0.0
    sat_rows: list[dict] = []
    clean_rows: list[dict] = []
    for r in samples:
        o = float(r["yaw_out_lua"])
        i = float(r["yaw_i_lua"])
        saturated = (o >= OUT_HI) or (o <= OUT_LO) \
                    or (i >= imax - I_EPS) \
                    or (ki > 0 and i <= I_EPS and o <= OUT_LO + 1e-3)
        (sat_rows if saturated else clean_rows).append(r)

    n_total = len(samples)
    n_sat   = len(sat_rows)
    n_keep  = len(clean_rows)
    pct_sat = 100.0 * n_sat / n_total

    print("")
    print("  Sample budget:")
    print(f"    total ..... {n_total}")
    print(f"    saturated . {n_sat}  ({pct_sat:.1f}%)")
    print(f"    kept ...... {n_keep}  ({100.0 - pct_sat:.1f}%)")

    # Loud warning when most of the run was saturated -- the loop simply
    # wasn't operating, so PID gains can't be meaningfully judged.
    if pct_sat >= 50.0:
        print("")
        print("  [WARN] Actuator saturated >50% of the run.  PID gains")
        print("         cannot be judged from this data: the loop spent")
        print("         most of its time open-loop at the actuator limit.")
        print("         Likely causes:")
        print("           - ESC throttle range mis-calibrated (PWM 1999")
        print("             not delivering full motor power -- check the")
        print("             current draw column)")
        print("           - DShot channel receiving raw PWM (SERVO_BLH_BDMASK")
        print("             bit set for ch4 but Lua writes raw us)")
        print("           - Motor undersized for the disturbance torque")
        print("         Fix actuator authority FIRST, then re-run.")

    eval_rows = clean_rows if exclude_saturate else samples
    scope_lbl = "unsaturated subset" if exclude_saturate else "all samples"

    if not eval_rows:
        print("")
        print("  [WARN] No non-saturated samples -- nothing to analyse.")
        return
    if kp <= 0:
        print("  [FAIL] kp <= 0 -- cannot recover yaw rate from PID output.")
        return
    if kd != 0.0:
        print("  [NOTE] kd != 0 -- recovered yaw rate includes D-term effect.")

    err_list: list[float]   = []
    p_out_list: list[float] = []
    i_out_list: list[float] = []
    out_list:  list[float]  = []
    mot_list:  list[float]  = []
    t_list:    list[float]  = []
    for r in eval_rows:
        o = float(r["yaw_out_lua"])
        i = float(r["yaw_i_lua"])
        err = (o - i - trim) / kp        # rad/s; err = -yaw_rate
        err_list.append(err)
        p_out_list.append(kp * err)
        i_out_list.append(i)
        out_list.append(o)
        mot = r.get("mot_us", r.get("s4_us"))
        if isinstance(mot, (int, float)):
            mot_list.append(float(mot))
        t = r.get("t_s")
        if isinstance(t, (int, float)):
            t_list.append(float(t))

    def _mean(xs: list[float]) -> float:
        return sum(xs) / len(xs)

    def _rms(xs: list[float]) -> float:
        return math.sqrt(sum(x * x for x in xs) / len(xs))

    def _pct(xs: list[float], p: float) -> float:
        s = sorted(xs)
        k = max(0, min(len(s) - 1, int(round(p * (len(s) - 1)))))
        return s[k]

    DEG = 180.0 / math.pi
    yaw_rate_dps = [-e * DEG for e in err_list]
    abs_dps      = [abs(v) for v in yaw_rate_dps]

    print("")
    print(f"  Yaw rate residual ({scope_lbl}, deg/s):")
    print(f"    mean ........ {_mean(yaw_rate_dps):+8.2f}")
    print(f"    RMS ......... {_rms(yaw_rate_dps):8.2f}")
    print(f"    p95 |.| ..... {_pct(abs_dps, 0.95):8.2f}")
    print(f"    max |.| ..... {max(abs_dps):8.2f}")

    print("")
    print(f"  PID effort ({scope_lbl}, output units in [0,1]):")
    print(f"    mean p_out .. {_mean(p_out_list):+.4f}  (= kp * err)")
    print(f"    mean i_out .. {_mean(i_out_list):+.4f}  (integrator state)")
    print(f"    mean output . {_mean(out_list):+.4f}  (pre-clamp)")
    if mot_list:
        print(f"    mean mot_us . {_mean(mot_list):8.1f}  (motor PWM, "
              f"{min(mot_list):.0f} .. {max(mot_list):.0f})")

    # Zero-crossing oscillation estimate (cheap, no FFT).  Order-of-magnitude.
    f_hz = 0.0
    if len(yaw_rate_dps) >= 4 and len(t_list) >= 2:
        zc = sum(1 for a, b in zip(yaw_rate_dps[:-1], yaw_rate_dps[1:])
                 if (a > 0) != (b > 0))
        span_s = max(1e-6, t_list[-1] - t_list[0])
        f_hz = zc / (2.0 * span_s)
        print("")
        print("  Oscillation (rough, sign changes in kept subset):")
        print(f"    crossings ... {zc} over {span_s:.2f} s")
        print(f"    ~frequency .. {f_hz:.2f} Hz")

    # -- Heuristic tuning advice ------------------------------------------
    print("")
    print("  Suggestions:")
    advice: list[str] = []

    if pct_sat >= 50.0:
        advice.append("Saturation dominates -- fix actuator authority "
                      "before judging PID tuning.  See the [WARN] above.")
    else:
        mean_err_dps = _mean(yaw_rate_dps)
        rms_err_dps  = _rms(yaw_rate_dps)
        mean_i       = _mean(i_out_list)
        mean_p       = _mean(p_out_list)

        if abs(mean_err_dps) > max(5.0, 0.2 * rms_err_dps):
            if ki <= 0.0:
                advice.append(
                    f"Mean yaw rate biased ({mean_err_dps:+.1f} deg/s) and "
                    f"ATC_RAT_YAW_I = 0.  Add some I (try {kp*0.2:.3g}) or "
                    "raise H_YAW_TRIM to feed the steady torque demand.")
            elif mean_i >= imax - I_EPS:
                advice.append(
                    "Integrator pegged at IMAX while yaw still biased -- "
                    f"raise ATC_RAT_YAW_IMAX (currently {imax:.2g}, try "
                    f"{imax * 1.5:.2g}).")
            else:
                advice.append(
                    f"Mean yaw rate biased ({mean_err_dps:+.1f} deg/s) -- "
                    f"raise ATC_RAT_YAW_I (currently {ki:.3g}, try "
                    f"{ki * 1.5:.3g}).")

        if f_hz > 3.0:
            advice.append(
                f"Fast oscillation (~{f_hz:.1f} Hz) -- reduce ATC_RAT_YAW_P "
                f"(currently {kp:.3g}, try {kp * 0.6:.3g}) or add D "
                "(ATC_RAT_YAW_D) and lower ATC_RAT_YAW_FLTD.")

        if rms_err_dps > 20.0 and abs(mean_p) < 0.3 and kp < 1.0:
            advice.append(
                f"Large residual ({rms_err_dps:.1f} deg/s RMS) with modest "
                f"P effort -- raise ATC_RAT_YAW_P (currently {kp:.3g}, try "
                f"{kp * 1.5:.3g}).")

        if mot_list:
            margin_up = servo_max - max(mot_list)
            mean_pwm  = _mean(mot_list)
            if margin_up < 20 and pct_sat < 50.0:
                advice.append(
                    f"PWM headroom thin at the top (max {max(mot_list):.0f} "
                    f"vs {servo_max:.0f}) -- raise SERVO{SERVO_MOTOR}_MAX if the ESC "
                    "accepts it.")
            if mean_pwm < servo_min + 0.1 * (servo_max - servo_min):
                advice.append(
                    f"Mean PWM ({mean_pwm:.0f}) sits near SERVO{SERVO_MOTOR}_MIN -- "
                    "raise H_YAW_TRIM so the PID is not fighting a static "
                    "offset.")

    if not advice:
        advice.append("No tuning concerns flagged.  Residuals are small "
                      "and the actuator is not saturating.")

    for k, line in enumerate(advice, start=1):
        head = f"    {k}. "
        wrap = "       "
        cur = head
        for w in line.split():
            if len(cur) + 1 + len(w) > 78 and cur not in (head, wrap):
                print(cur)
                cur = wrap + " " + w
            else:
                cur = cur + (" " if cur not in (head, wrap) else "") + w
        if cur.strip():
            print(cur)
    print("")


# -- `analyze motor` -- motor discrete-speed (PWM-quantisation) detector -----
#
# Theory under test: the GB4008 + ESC may not honour the full PWM resolution but
# only produce a few discrete speeds.  No ESC RPM telemetry exists, so the motor
# output is inferred from the yaw dynamics (SERVO4 PWM -> yaw angular accel).
#
# Raw closed-loop yaw data is chaotic, so a single-sample PWM->accel scatter is
# pure noise.  BUT the yaw loop limit-cycles at a stable frequency; by phase-
# locking to that cycle and coherently averaging many cycles, the random body
# chaos averages out (~1/sqrt(N)) and the repeatable motor-response waveform
# emerges at high SNR.  Steps/plateaus in the phase-averaged PWM->accel curve
# are the signature of discrete motor speeds.

def _read_motor_jsonl(jsonl_path: str, motor_off_us: float = 820.0):
    """Parse a *.mavlink.jsonl.  Align SERVO4 PWM (command) with ATTITUDE
    yaw-rate on the shared wall clock (_t_wall) -- the two streams use different
    time_boot_ms bases, so wall clock is the only common timebase.

    Returns (rows, fs) where rows = [(t_rel, pwm, yawrate, yaw_accel), ...] with
    yaw_accel from a 3-point central difference, motor-off samples dropped, and
    fs = mean ATTITUDE sample rate (Hz)."""
    att: list[tuple] = []
    srv: list[tuple] = []
    with open(jsonl_path, "r") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                m = json.loads(line)
            except ValueError:
                continue
            mt = m.get("mavpackettype")
            tw = m.get("_t_wall")
            if tw is None:
                continue
            if mt == "ATTITUDE":
                att.append((tw, m.get("yawspeed")))
            elif mt == "SERVO_OUTPUT_RAW":
                srv.append((tw, m.get(f"servo{SERVO_MOTOR}_raw", m.get("servo4_raw"))))
    att = [x for x in att if x[1] is not None]
    srv = [x for x in srv if x[1] is not None]
    att.sort(); srv.sort()
    if len(att) < 30 or len(srv) < 30:
        return [], 0.0

    srv_t = [s[0] for s in srv]

    def _pwm_at(tt: float):
        i = bisect.bisect_right(srv_t, tt) - 1
        return srv[i][1] if i >= 0 else None

    t0 = att[0][0]
    rows: list[tuple] = []
    for i in range(1, len(att) - 1):
        ta, wa = att[i - 1]
        tb, wb = att[i]
        tc, wc = att[i + 1]
        span = tc - ta
        if span <= 0.0 or span > 0.2:
            continue
        pwm = _pwm_at(tb)
        if pwm is None or pwm <= motor_off_us:
            continue
        rows.append((tb - t0, float(pwm), float(wb), (wc - wa) / span))
    if len(rows) < 30:
        return rows, 0.0
    fs = (len(rows) - 1) / (rows[-1][0] - rows[0][0])
    return rows, fs


def _smooth_ma(x: list[float], half: int) -> list[float]:
    """Centered moving-average low-pass, half-width `half` samples."""
    n = len(x)
    out = [0.0] * n
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out[i] = sum(x[lo:hi]) / (hi - lo)
    return out


def _autocorr_period(t: list[float], y: list[float],
                     tmin: float, tmax: float) -> "float | None":
    """Dominant period (s) of y via peak of the unbiased autocorrelation over
    the lag window [tmin, tmax]."""
    n = len(t)
    if n < 8:
        return None
    fs = (n - 1) / (t[-1] - t[0])
    ym = sum(y) / n
    yy = [v - ym for v in y]
    lo = max(1, int(tmin * fs))
    hi = min(int(tmax * fs), n - 2)
    best_lag = None
    best_val = None
    for lag in range(lo, hi):
        s = 0.0
        for i in range(n - lag):
            s += yy[i] * yy[i + lag]
        s /= (n - lag)
        if best_val is None or s > best_val:
            best_val = s
            best_lag = lag
    return best_lag / fs if best_lag else None


def _pearson(x: list[float], y: list[float]) -> float:
    mx = sum(x) / len(x)
    my = sum(y) / len(y)
    sx = math.sqrt(sum((xi - mx) ** 2 for xi in x))
    sy = math.sqrt(sum((yi - my) ** 2 for yi in y))
    if sx * sy < 1e-12:
        return 0.0
    return sum((xi - mx) * (yi - my) for xi, yi in zip(x, y)) / (sx * sy)


def _dp_piecewise_constant(y: list[float], w: list[float], kmax: int):
    """Optimal weighted piecewise-constant fit of `y` (weights `w`) for
    K = 1..kmax segments.  Returns list of (K, sse, breakpoint_indices)."""
    n = len(y)
    W = [0.0] * (n + 1)
    WY = [0.0] * (n + 1)
    WYY = [0.0] * (n + 1)
    for i in range(n):
        W[i + 1] = W[i] + w[i]
        WY[i + 1] = WY[i] + w[i] * y[i]
        WYY[i + 1] = WYY[i] + w[i] * y[i] * y[i]

    def seg(a: int, b: int) -> float:  # weighted SSE of points [a, b)
        ww = W[b] - W[a]
        if ww <= 0:
            return 0.0
        wy = WY[b] - WY[a]
        return (WYY[b] - WYY[a]) - wy * wy / ww

    INF = float("inf")
    cost = [[INF] * (n + 1) for _ in range(kmax + 1)]
    brk = [[0] * (n + 1) for _ in range(kmax + 1)]
    cost[0][0] = 0.0
    for k in range(1, kmax + 1):
        for j in range(1, n + 1):
            for i in range(k - 1, j):
                c = cost[k - 1][i] + seg(i, j)
                if c < cost[k][j]:
                    cost[k][j] = c
                    brk[k][j] = i
    out = []
    for k in range(1, kmax + 1):
        js = [n]
        kk = k
        while kk > 0:
            js.append(brk[kk][js[-1]])
            kk -= 1
        js.reverse()
        out.append((k, cost[k][n], js))
    return out


def _analyze_motor_quantization(jsonl_path: str, *, motor_off_us: float = 820.0,
                                phase_pts: int = 120) -> None:
    """Detect whether the SERVO4 motor produces discrete speed levels rather
    than honouring the continuous PWM command, using limit-cycle ensemble
    averaging of the yaw response.  Reads the high-rate *.mavlink.jsonl."""
    print("")
    print("  Motor discrete-speed detector (limit-cycle ensemble averaging)")
    print("  " + "=" * 62)

    rows, fs = _read_motor_jsonl(jsonl_path, motor_off_us=motor_off_us)
    print(f"  source ........ {jsonl_path}")
    if not rows or fs <= 0.0:
        print("  [FAIL] too few aligned ATTITUDE/SERVO_OUTPUT_RAW samples "
              "(need the .mavlink.jsonl, motor running).")
        return

    t = [r[0] for r in rows]
    u = [r[1] for r in rows]
    w = [r[2] for r in rows]
    a = [r[3] for r in rows]
    DEG = 180.0 / math.pi
    print(f"  samples ....... {len(rows)}  @ {fs:.1f} Hz  "
          f"({t[-1] - t[0]:.1f} s, motor > {motor_off_us:.0f} us)")
    print(f"  PWM span ...... {min(u):.0f} .. {max(u):.0f} us "
          f"(commanded, full run)")

    # 1. Find the limit-cycle period from the yaw rate.
    per = _autocorr_period(t, w, 0.35, 2.0)
    if per is None or per <= 0.0:
        print("  [FAIL] no limit-cycle period found in yaw rate.")
        return
    print(f"  limit cycle ... {per:.3f} s  ({1.0 / per:.2f} Hz)")

    # 2. Trigger cycles on smoothed yaw-rate zero up-crossings (low-pass first
    #    to remove the fast harmonics that would over-segment).
    half = max(1, int(0.10 * fs))
    ws = _smooth_ma(w, half)
    cross = [i for i in range(1, len(ws)) if ws[i - 1] < 0.0 <= ws[i]]

    P = phase_pts
    cycles: list[list] = []          # each: [dur, u[P], w[P], a[P]]
    for k in range(len(cross) - 1):
        i0, i1 = cross[k], cross[k + 1]
        dur = t[i1] - t[i0]
        if dur < 0.6 * per or dur > 1.4 * per or (i1 - i0) < 10:
            continue
        st, su, sw, sa = t[i0:i1 + 1], u[i0:i1 + 1], w[i0:i1 + 1], a[i0:i1 + 1]
        ph = [(st[j] - st[0]) / (st[-1] - st[0]) for j in range(len(st))]
        ru = [0.0] * P
        rw = [0.0] * P
        ra = [0.0] * P
        for p in range(P):
            x = p / (P - 1)
            j = bisect.bisect_left(ph, x)
            if j <= 0:
                ru[p], rw[p], ra[p] = su[0], sw[0], sa[0]
            elif j >= len(ph):
                ru[p], rw[p], ra[p] = su[-1], sw[-1], sa[-1]
            else:
                f = (x - ph[j - 1]) / (ph[j] - ph[j - 1] + 1e-12)
                ru[p] = su[j - 1] + f * (su[j] - su[j - 1])
                rw[p] = sw[j - 1] + f * (sw[j] - sw[j - 1])
                ra[p] = sa[j - 1] + f * (sa[j] - sa[j - 1])
        cycles.append([dur, ru, rw, ra])

    if len(cycles) < 6:
        print(f"  [FAIL] only {len(cycles)} usable cycles -- need >= 6.  The run "
              "may be too short or not limit-cycling cleanly.")
        return

    def _ens(idx: int, cs: list) -> list:
        return [sum(c[idx][p] for c in cs) / len(cs) for p in range(P)]

    # 3. Reject shape outliers: keep cycles whose yaw-rate waveform correlates
    #    with the ensemble mean (repeat to converge).
    kept = cycles
    for _ in range(4):
        wbar = _ens(2, kept)
        nk = [c for c in kept if _pearson(c[2], wbar) > 0.7]
        if len(nk) < 6 or len(nk) == len(kept):
            kept = nk if len(nk) >= 6 else kept
            break
        kept = nk
    N = len(kept)
    snr_gain = math.sqrt(N)
    print(f"  cycles ........ {len(cycles)} found, {N} kept (shape-matched); "
          f"averaging cuts noise ~{snr_gain:.1f}x")

    Um = _ens(1, kept)
    Wm = _ens(2, kept)
    Am = _ens(3, kept)
    Asd = [math.sqrt(sum((c[3][p] - Am[p]) ** 2 for c in kept) / N) for p in range(P)]

    # Coherent (phase-locked) PWM excursion: how much of the PWM range actually
    # repeats cycle-to-cycle.  This caps what we can learn about the motor map.
    coh_span = max(Um) - min(Um)
    print("")
    print(f"  Phase-averaged limit cycle (N={N}, high-SNR):")
    print(f"    {'phase':>6}  {'pwm_us':>7}  {'yawrate_dps':>11}  "
          f"{'accel_dps2':>10}  {'+/- SE':>7}")
    for p in range(0, P, max(1, P // 12)):
        print(f"    {p / (P - 1):>6.2f}  {Um[p]:>7.1f}  "
              f"{Wm[p] * DEG:>+11.1f}  {Am[p] * DEG:>+10.0f}  {Asd[p] * DEG / snr_gain:>7.0f}")

    # 4. Phase-averaged motor curve: accel vs PWM (both phase-locked, so this is
    #    the repeatable component only).  Sort by PWM, bin, weight by count.
    order = sorted(range(P), key=lambda p: Um[p])
    bw = 8.0
    binned: dict = {}
    for p in order:
        b = round(Um[p] / bw) * bw
        binned.setdefault(b, []).append(Am[p] * DEG)
    curve_x = sorted(binned)
    curve_y = [sum(binned[b]) / len(binned[b]) for b in curve_x]
    curve_n = [float(len(binned[b])) for b in curve_x]

    print("")
    print("  Phase-averaged motor curve  (yaw accel vs commanded PWM):")
    for bx, by, bn in zip(curve_x, curve_y, curve_n):
        print(f"    {bx:>7.0f} us   {by:>+9.0f} deg/s^2   (n={int(bn)})")

    # 5. Decide: is the coherent PWM excursion wide enough to judge the motor
    #    map?  If not, no amount of averaging can prove/disprove quantisation
    #    from this log -- recommend an open-loop ramp.
    print("")
    USABLE_SPAN_US = 60.0
    if coh_span < USABLE_SPAN_US or len(curve_x) < 5:
        print(f"  VERDICT: INCONCLUSIVE from this log.")
        print(f"    The phase-locked PWM only swings ~{coh_span:.0f} us "
              f"(< {USABLE_SPAN_US:.0f} us needed).")
        print("    The PID holds SERVO4 in a narrow band and the body chaos that")
        print("    drives the larger PWM excursions does NOT repeat cycle-to-cycle,")
        print("    so it averages away.  Ensemble averaging cleaned the cycle (good")
        print(f"    ~{snr_gain:.0f}x SNR) but cannot map the motor over enough PWM range.")
        print("")
        print("    To test discrete-speed directly, capture an OPEN-LOOP ramp:")
        print("      - Disable the yaw loop so PWM is commanded, not servoed:")
        print("          run passive --noluayaw   (passive forces AP yaw=0; --noluayaw also disables Lua yaw)")
        print("        or drive the motor output directly while logging yaw:")
        print("          motor <pwm> --duration 4      (step several PWMs 1050..1350)")
        print("      - Step SERVO4 in small increments (e.g. +10 us / 3 s) across")
        print("        1050..1350 us; a discrete motor shows the yaw rate settling")
        print("        onto the SAME few plateaus regardless of the fine PWM value.")
        print("      - Re-run 'analyze motor <log>' on that capture.")
        return

    # 6. Staircase detection on the (now high-SNR) motor curve.
    kmax = min(6, len(curve_x) - 1)
    fits = _dp_piecewise_constant(curve_y, curve_n, kmax)
    neff = sum(curve_n)
    best_k, best_bic = 1, None
    for k, sse, _js in fits:
        sse = max(sse, 1e-9)
        params = 2 * k - 1
        bic = neff * math.log(sse / neff) + params * math.log(max(neff, 2))
        if best_bic is None or bic < best_bic:
            best_bic, best_k = bic, k
    # linear reference fit (weighted)
    sx = sum(n * x for n, x in zip(curve_n, curve_x))
    sy = sum(n * y for n, y in zip(curve_n, curve_y))
    sxx = sum(n * x * x for n, x in zip(curve_n, curve_x))
    sxy = sum(n * x * y for n, x, y in zip(curve_n, curve_x, curve_y))
    den = neff * sxx - sx * sx
    if abs(den) > 1e-9:
        slope = (neff * sxy - sx * sy) / den
        icpt = (sy - slope * sx) / neff
    else:
        slope, icpt = 0.0, sy / neff
    sse_lin = sum(n * (y - (slope * x + icpt)) ** 2
                  for n, x, y in zip(curve_n, curve_x, curve_y))
    sse_step = max(next(sse for k, sse, _ in fits if k == best_k), 1e-9)
    step_vs_lin = sse_lin / sse_step
    # level jump vs residual noise
    levels = [curve_y[i] for i in range(best_k)]
    _js = next(js for k, _s, js in fits if k == best_k)
    seg_means = []
    for s in range(best_k):
        lo, hi = _js[s], _js[s + 1]
        seg = curve_y[lo:hi]
        wts = curve_n[lo:hi]
        seg_means.append(sum(v * ww for v, ww in zip(seg, wts)) / sum(wts))
    jumps = [abs(seg_means[i + 1] - seg_means[i]) for i in range(len(seg_means) - 1)]
    noise = math.sqrt(sse_step / neff)
    jump_to_noise = (min(jumps) / noise) if jumps and noise > 0 else 0.0

    print(f"  Staircase test on the motor curve:")
    print(f"    best step model .. K={best_k} level(s)")
    print(f"    step vs linear ... {step_vs_lin:.2f}x lower SSE "
          f"({'steps win' if step_vs_lin > 1.2 else 'line as good'})")
    if best_k >= 2:
        print(f"    level accels ..... " +
              ", ".join(f"{v:+.0f}" for v in seg_means) + " deg/s^2")
        print(f"    smallest jump .... {min(jumps):.0f} deg/s^2  "
              f"({jump_to_noise:.1f}x noise)")

    discrete = (best_k >= 2 and step_vs_lin > 1.3 and jump_to_noise > 2.0)
    print("")
    if discrete:
        print(f"  VERDICT: DISCRETE motor speeds LIKELY -- {best_k} levels, the "
              "step")
        print("    model fits markedly better than a smooth ramp.  Confirm with an")
        print("    open-loop PWM ramp (see below) before trusting the level count.")
    else:
        print("  VERDICT: NO clear discreteness -- the motor curve is consistent")
        print("    with a continuous (smooth) PWM->torque map over the tested span.")
    print("")
    print("    For a definitive test, capture an open-loop ramp: step SERVO4 in")
    print("    small increments (motor <pwm> --duration 4, PWM 1050..1350) and")
    print("    re-run 'analyze motor <log>'.")


def _cmd_analyze(args: list[str]) -> None:
    """analyze yaw <csv> [--include-saturate]
       analyze motor <log>            (limit-cycle motor discrete-speed detector)

    Offline analysis of a saved run log (no FC connection required)."""
    if not args:
        print("  Usage: analyze yaw <csv>  [--include-saturate]")
        print("         analyze motor <log>   (.mavlink.jsonl or its .csv sibling)")
        return
    target = args[0].lower()

    if target == "motor":
        try:
            pos, _flags = _parse_flags(args[1:], {})
        except ValueError as e:
            print(f"  Error: {e}"); return
        if len(pos) != 1:
            print("  Usage: analyze motor <log>   (.mavlink.jsonl or its .csv sibling)")
            return
        path = pos[0]
        # Accept a .csv path and auto-resolve the sibling high-rate JSONL.
        if path.endswith(".csv"):
            cand = path[:-4] + ".mavlink.jsonl"
            if os.path.exists(cand):
                path = cand
            else:
                print(f"  [FAIL] need the high-rate MAVLink log; not found: {cand}")
                return
        if not os.path.exists(path):
            print(f"  [FAIL] file not found: {path}")
            return
        _analyze_motor_quantization(path)
        return

    if target != "yaw":
        print(f"  Unknown analyze target {args[0]!r}  (valid: yaw, motor)")
        return

    schema = {"--include-saturate": "bool"}
    try:
        pos, flags = _parse_flags(args[1:], schema)
    except ValueError as e:
        print(f"  Error: {e}"); return
    if len(pos) != 1:
        print("  Usage: analyze yaw <csv>  [--include-saturate]")
        return
    csv_path = pos[0]
    if not os.path.exists(csv_path):
        print(f"  [FAIL] file not found: {csv_path}")
        return
    exclude = not bool(flags.get("--include-saturate", False))
    _analyze_yaw_csv(csv_path, exclude_saturate=exclude)


# -- `watch <stream>` -----------------------------------------------------

_WATCH_STREAMS = {
    "servos":   "Stream SERVO_OUTPUT_RAW for ch1..8",
    "esc":      "Stream ESC_TELEMETRY for rpm/volt/current/temp",
    "text":     "Stream STATUSTEXT only",
    "attitude": "Stream ATTITUDE (roll/pitch/yaw + body rates)",
    "power":    "Stream BATTERY_STATUS / SYS_STATUS (vbat / current / power)",
}


def _cmd_watch(session: RawesGCS, args: list[str]) -> None:
    """watch <stream> [--duration N]"""
    schema = {"--duration": "float"}
    if not args:
        print("  Usage: watch <stream> [--duration N]")
        print("  Streams:")
        for k, v in _WATCH_STREAMS.items():
            print(f"    {k:<10} -- {v}")
        return
    try:
        pos, flags = _parse_flags(args, schema)
    except ValueError as e:
        print(f"  Error: {e}"); return
    if len(pos) != 1:
        print("  Usage: watch <stream> [--duration N]"); return
    stream = pos[0].lower()
    if stream not in _WATCH_STREAMS:
        print(f"  Unknown stream {stream!r}  (valid: {', '.join(_WATCH_STREAMS)})")
        return
    duration = flags.get("--duration", 10.0)

    meta = {
        "verb":            "watch",
        "stream":          stream,
        "duration_s":      duration,
        "run_start_local": datetime.now().isoformat(timespec="seconds"),
        "run_start_utc":   datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    log = _RunLog.open("watch", stream, meta)
    print(f"  Logging to {log.path}")

    try:
        if stream == "servos":
            _watch_servos(session, duration, log)
        elif stream == "esc":
            _watch_esc(session, duration, log)
        elif stream == "text":
            _watch_text(session, duration, log)
        elif stream == "attitude":
            _watch_attitude(session, duration, log)
        elif stream == "power":
            _watch_power(session, duration, log)
    finally:
        log.close()
        print(f"  Wrote {log.n_rows} rows to {log.path}")
    print("  Done.")


def _watch_servos(session, duration, log):
    cols = ["t_s"] + [f"s{i}_us" for i in range(1, 9)]
    state = {f"s{i}": None for i in range(1, 9)}

    def handle(st, msg, t_rel):
        mt = msg.get_type()
        if mt == "SERVO_OUTPUT_RAW":
            for i in range(1, 9):
                state[f"s{i}"] = getattr(msg, f"servo{i}_raw", None)
            return [f"{t_rel:.4f}"] + [state[f"s{i}"] for i in range(1, 9)]
        return None

    def render(st, t_rel):
        return [f"{t_rel:.1f}"] + [state[f"s{i}"] for i in range(1, 9)]

    _observation_loop(
        session, duration_s=duration,
        msg_types=["SERVO_OUTPUT_RAW", "HEARTBEAT", "STATUSTEXT"],
        streams=[(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)],
        handle_msg=handle, render_row=render,
        header_cols=cols,
        header_print_cols=["t(s)"] + [f"s{i}" for i in range(1, 9)],
        log=log,
    )


def _watch_esc(session, duration, log):
    # GB4008 bidir DShot telemetry (AUX 1 = SERVO9 -> ESC_TELEMETRY_9_TO_12[0]).
    cols = ["t_s", "erpm", "mech_rpm", "rotor_rpm", "voltage_v", "current_a", "temp_c"]
    esc_name, esc_id = _esc_telem_msg_for_channel(MOTOR_ESC_CHANNEL)
    idx = (MOTOR_ESC_CHANNEL - 1) % 4
    state = {"erpm": None, "volt": None, "curr": None, "temp": None}

    def handle(st, msg, t_rel):
        if msg.get_type() != esc_name:
            return None
        erpm = _esc_erpm(msg, MOTOR_ESC_CHANNEL)
        volt = msg.voltage[idx] / 100.0 if hasattr(msg, "voltage") else None
        curr = msg.current[idx] / 100.0 if hasattr(msg, "current") else None
        temp = msg.temperature[idx] if hasattr(msg, "temperature") else None
        state["erpm"], state["volt"], state["curr"], state["temp"] = \
            erpm, volt, curr, temp
        _e, mech, rotor = _rpm_triplet(erpm)
        return [f"{t_rel:.4f}", _fmt(_e), _fmt(mech), _fmt(rotor),
                _fmt(volt), _fmt(curr), temp]

    def render(st, t_rel):
        _e, mech, rotor = _rpm_triplet(state["erpm"])
        return [
            f"{t_rel:.1f}",
            f"{_e:.0f}" if _e is not None else None,
            f"{mech:.0f}" if mech is not None else None,
            f"{rotor:.0f}" if rotor is not None else None,
            f"{state['volt']:.2f}" if state["volt"] is not None else None,
            f"{state['curr']:.2f}" if state["curr"] is not None else None,
            state["temp"],
        ]

    def _req_esc() -> None:
        session.set_message_interval(esc_id, 100000)   # 10 Hz

    _observation_loop(
        session, duration_s=duration,
        msg_types=[esc_name, "HEARTBEAT", "STATUSTEXT"],
        streams=[(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)],
        handle_msg=handle, render_row=render,
        header_cols=cols,
        header_print_cols=["t(s)", "eRPM", "mRPM", "rotRPM", "V", "A", "T"],
        setup_hook=_req_esc,
        log=log,
    )


def _watch_text(session, duration, log):
    cols = ["t_s", "severity", "text"]

    def handle(st, msg, t_rel):
        # STATUSTEXTs are handled by the engine and surfaced via state["pending_text"]
        # We still log them here so the CSV captures everything.
        if msg.get_type() == "STATUSTEXT":
            text = msg.text.rstrip("\x00").strip()
            if text:
                return [f"{t_rel:.4f}", int(getattr(msg, "severity", 6)), text]
        return None

    def render(st, t_rel):
        return [f"{t_rel:.1f}"]

    _observation_loop(
        session, duration_s=duration,
        msg_types=["HEARTBEAT", "STATUSTEXT"],
        streams=[],
        handle_msg=handle, render_row=render,
        header_cols=cols,
        header_print_cols=["t(s)"],
        log=log,
    )


def _watch_attitude(session, duration, log):
    cols = ["t_s", "roll_deg", "pitch_deg", "yaw_deg",
            "omega_x_dps", "omega_y_dps", "omega_z_dps"]
    state = {"roll": None, "pitch": None, "yaw": None,
             "wx": None, "wy": None, "wz": None}

    def handle(st, msg, t_rel):
        if msg.get_type() == "ATTITUDE":
            state["roll"]  = math.degrees(msg.roll)
            state["pitch"] = math.degrees(msg.pitch)
            state["yaw"]   = math.degrees(msg.yaw)
            state["wx"]    = math.degrees(msg.rollspeed)
            state["wy"]    = math.degrees(msg.pitchspeed)
            state["wz"]    = math.degrees(msg.yawspeed)
            return [f"{t_rel:.4f}",
                    f"{state['roll']:.3f}", f"{state['pitch']:.3f}", f"{state['yaw']:.3f}",
                    f"{state['wx']:.3f}", f"{state['wy']:.3f}", f"{state['wz']:.3f}"]
        return None

    def render(st, t_rel):
        return [
            f"{t_rel:.1f}",
            f"{state['roll']:+.1f}"  if state["roll"]  is not None else None,
            f"{state['pitch']:+.1f}" if state["pitch"] is not None else None,
            f"{state['yaw']:+.1f}"   if state["yaw"]   is not None else None,
            f"{state['wx']:+.1f}"    if state["wx"]    is not None else None,
            f"{state['wy']:+.1f}"    if state["wy"]    is not None else None,
            f"{state['wz']:+.1f}"    if state["wz"]    is not None else None,
        ]

    _observation_loop(
        session, duration_s=duration,
        msg_types=["ATTITUDE", "HEARTBEAT", "STATUSTEXT"],
        streams=[(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 25)],
        handle_msg=handle, render_row=render,
        header_cols=cols, log=log,
    )


def _watch_power(session, duration, log):
    cols = ["t_s", "vbat_v", "current_a", "power_w"]
    state = {"vbat": None, "curr": None}

    def handle(st, msg, t_rel):
        mt = msg.get_type()
        if mt == "BATTERY_STATUS":
            cells = [v for v in msg.voltages if v != 65535]
            if cells:
                state["vbat"] = sum(cells) / 1000.0
            if msg.current_battery >= 0:
                state["curr"] = msg.current_battery / 100.0
        elif mt == "SYS_STATUS":
            if state["vbat"] is None and msg.voltage_battery != 65535:
                state["vbat"] = msg.voltage_battery / 1000.0
            if state["curr"] is None and msg.current_battery >= 0:
                state["curr"] = msg.current_battery / 100.0
        else:
            return None
        power = state["vbat"] * state["curr"] if (state["vbat"] is not None and state["curr"] is not None) else None
        return [f"{t_rel:.4f}", _fmt(state["vbat"]), _fmt(state["curr"]), _fmt(power)]

    def render(st, t_rel):
        p = state["vbat"] * state["curr"] if (state["vbat"] is not None and state["curr"] is not None) else None
        return [
            f"{t_rel:.1f}",
            f"{state['vbat']:.2f}" if state["vbat"] is not None else None,
            f"{state['curr']:.2f}" if state["curr"] is not None else None,
            f"{p:.1f}" if p is not None else None,
        ]

    _observation_loop(
        session, duration_s=duration,
        msg_types=["BATTERY_STATUS", "SYS_STATUS", "HEARTBEAT", "STATUSTEXT"],
        streams=[(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5)],
        handle_msg=handle, render_row=render,
        header_cols=cols, log=log,
    )


# =============================================================================
# Dispatch
# =============================================================================

def _run_command(session: RawesGCS, tokens: list[str],
                 force: bool = False) -> bool:
    """
    Execute one calibration command.

    tokens : verb + arguments, e.g. ["run", "passive", "--duration", "30"]
    force  : skip interactive confirmation prompts (for CLI / scripted use)

    Returns True if the verb was recognised, False otherwise.
    """
    if not tokens:
        return True
    verb = tokens[0].lower()
    args = tokens[1:]

    if   verb == "status":   _print_status(session)
    elif verb == "ping":     _cmd_ping(args)
    elif verb == "reboot":   _cmd_reboot(session)
    elif verb == "disarm":   _disarm(session)
    elif verb == "arm":      _cmd_arm(session, args)
    elif verb == "set":      _cmd_set(session, args)
    elif verb == "get":      _cmd_get(session, args)
    elif verb == "swash":    _cmd_swash(session, args)
    elif verb == "servo":    _cmd_servo(session, args)
    elif verb == "motor":    _cmd_motor(session, args, force=force)
    elif verb == "run":      _cmd_run(session, args)
    elif verb == "logs":        _cmd_logs(session, args)
    elif verb == "watch":    _cmd_watch(session, args)
    elif verb == "analyze":  _cmd_analyze(args)
    elif verb == "script":   _cmd_script(session, args)
    elif verb == "config":   _cmd_config(session, args)
    elif verb == "help":     print(_HELP)
    else:
        return False
    return True


# -- One-shot verb implementations ----------------------------------------

def _cmd_ping(args: list[str]) -> None:
    """ping [baud]"""
    try:
        baud = int(args[0]) if args else 115200
    except ValueError:
        print("  Usage: ping [baud]"); return
    _ping_ports(baud=baud)


def _cmd_reboot(session: RawesGCS) -> None:
    print("  Sending reboot command ...")
    session._mav.mav.command_long_send(
        session._target_system, session._target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    print("  Pixhawk rebooting -- reconnect in ~5 s.")


def _cmd_set(session: RawesGCS, args: list[str]) -> None:
    """set <name> <value>"""
    if len(args) < 2:
        print("  Usage: set <name> <value>"); return
    name = args[0].upper()
    try:
        value = float(args[1])
    except ValueError:
        print("  Error: value must be a number"); return
    ok = session.set_param(name, value)
    if not ok:
        print(f"  [FAIL] {name}: no ACK within timeout"); return
    actual = session.get_param(name)
    if actual is None:
        print(f"  [WARN] {name}: set ACK'd but readback failed")
    elif abs(actual - value) > 1e-6:
        print(f"  [FAIL] {name}: wrote {value}, read back {actual}  (likely silently rejected)")
    else:
        print(f"  [OK]   {name} = {actual}")


def _cmd_get(session: RawesGCS, args: list[str]) -> None:
    """get <name>"""
    if not args:
        print("  Usage: get <name>"); return
    name = args[0].upper()
    v = session.get_param(name)
    if v is None:
        print(f"  [FAIL] {name}: not found")
    else:
        print(f"  {name} = {v}")


def _print_swash_layout(session: RawesGCS) -> None:
    """Print the current H3-120 swashplate geometry as read from the FC.

    Reads every parameter that affects swash response (H_SW_TYPE, the three
    servo azimuths, phase angle, collective range, cyclic max, FC mounting
    rotation), then renders an ASCII top-view layout and a roll/pitch factor
    table.  Also samples current S1..S4 PWMs from SERVO_OUTPUT_RAW.

    See CLAUDE.md \"Swashplate geometry\" for the canonical invariant.
    """
    def g(name, default=None):
        v = session.get_param(name)
        return float(v) if v is not None else default

    sw_type   = g("H_SW_TYPE")
    sv1_pos   = g("H_SW_H3_SV1_POS")
    sv2_pos   = g("H_SW_H3_SV2_POS")
    sv3_pos   = g("H_SW_H3_SV3_POS")
    phang     = g("H_SW_H3_PHANG")
    ahrs_orn  = g("AHRS_ORIENTATION")
    col_min   = g("H_COL_MIN")
    col_max   = g("H_COL_MAX")
    col_mid   = g("H_COL_MID")
    cyc_max   = g("H_CYC_MAX")
    flybar    = g("H_FLYBAR_MODE")
    sv_man    = g("H_SV_MAN")

    def _quad(az):
        """Rough physical quadrant label from azimuth (deg, CCW from front)."""
        if az is None: return "?"
        az = ((az + 180.0) % 360.0) - 180.0   # -180..+180
        if   -30 <= az <=  30:  return "front"
        elif  30 <  az <  90:   return "front-left"
        elif  az >= 90 and az <= 150:  return "left-back"
        elif az > 150 or az < -150:    return "back"
        elif -90 <= az < -30:   return "front-right"
        else:                    return "right-back"

    def _factors(az):
        if az is None: return (None, None)
        a = math.radians(az)
        return (-math.sin(a), math.cos(a))   # (roll_factor, pitch_factor)

    print()
    print("RAWES H3-120 swashplate layout")
    print("==============================")
    print()
    print("ArduPilot params:")
    print(f"  H_SW_TYPE         = {sw_type!s:<8}  (3 = H3-120 generic)")
    print(f"  H_SW_H3_SV1_POS   = {sv1_pos!s:<8}  ({_quad(sv1_pos)})")
    print(f"  H_SW_H3_SV2_POS   = {sv2_pos!s:<8}  ({_quad(sv2_pos)})")
    print(f"  H_SW_H3_SV3_POS   = {sv3_pos!s:<8}  ({_quad(sv3_pos)})")
    print(f"  H_SW_H3_PHANG     = {phang!s:<8}  (deg of phase correction)")
    print(f"  AHRS_ORIENTATION  = {ahrs_orn!s:<8}  (0 = forward; 4 = YAW_180)")
    print()
    print(f"  H_COL_MIN = {col_min!s:<6}  H_COL_MAX = {col_max!s:<6}  "
          f"H_COL_MID = {col_mid!s:<6}")
    if cyc_max is not None:
        print(f"  H_CYC_MAX = {cyc_max:.0f}  cd  ({cyc_max/100:.1f} deg of swash tilt at full stick)")
    else:
        print(f"  H_CYC_MAX = ?")
    print(f"  H_FLYBAR_MODE = {flybar!s:<4}  (1 = ACRO passthrough, 0 = rate PID)")
    print(f"  H_SV_MAN      = {sv_man!s:<4}  (0 = AUTOMATED, !=0 = manual setup mode)")
    print()
    print("Servo factors  (AP mixer: roll = -sin(az), pitch = cos(az)):")
    print(f"  {'Servo':<6} {'Azimuth':>8}  {'Position':<14}  {'roll_f':>8}  {'pitch_f':>8}")
    for label, az in (("S1", sv1_pos), ("S2", sv2_pos), ("S3", sv3_pos)):
        rf, pf = _factors(az)
        rf_s = f"{rf:+.3f}" if rf is not None else "  n/a"
        pf_s = f"{pf:+.3f}" if pf is not None else "  n/a"
        az_s = f"{az:+.1f}" if az is not None else " n/a"
        print(f"  {label:<6} {az_s:>8}  {_quad(az):<14}  {rf_s:>8}  {pf_s:>8}")
    print()
    print("Layout (top view, looking down at the swashplate):")
    print()
    print("                FRONT (nose, +x)")
    print("                      ^")
    print("                      |")
    print("       SV2  *    [FC] *  SV1")
    print(f"     ({_fmt_az(sv2_pos)})         ({_fmt_az(sv1_pos)})")
    print(f"     {_quad(sv2_pos):<12}       {_quad(sv1_pos):<12}")
    print("                      |")
    print("                      *  SV3")
    print(f"                    ({_fmt_az(sv3_pos)})")
    print(f"                    {_quad(sv3_pos)}")
    print("                      v")
    print("                BACK (tail)")
    print()
    print("Sign convention (CLAUDE.md):")
    print("  tlat > 0 = roll right;  tlon > 0 = nose-DOWN disk;  col > 0 = positive thrust")
    print()
    # Live PWMs
    session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)
    srv = session._recv(type="SERVO_OUTPUT_RAW", blocking=True, timeout=2.0)
    if srv:
        s1 = getattr(srv, "servo1_raw", 0)
        s2 = getattr(srv, "servo2_raw", 0)
        s3 = getattr(srv, "servo3_raw", 0)
        smot = getattr(srv, f"servo{SERVO_MOTOR}_raw", 0)
        print(f"Current PWMs:  S1={s1} us  S2={s2} us  S3={s3} us  "
              f"S{SERVO_MOTOR}(motor)={smot} us")
    else:
        print("Current PWMs:  (no SERVO_OUTPUT_RAW received)")
    print()


def _fmt_az(az):
    if az is None:
        return "  ?  "
    return f"{az:+.0f} deg"


def _cmd_swash(session: RawesGCS, args: list[str]) -> None:
    """swash <coll%> [lon%] [lat%]
       swash range <min> <max>
       swash neutral [n]
       swash info"""
    if not args:
        print("  Usage: swash <coll%> [lon%] [lat%]")
        print("         swash range <min_us> <max_us>")
        print("         swash neutral [n]")
        print("         swash info")
        return
    sub = args[0].lower()
    if sub == "info":
        _print_swash_layout(session)
        return
    if sub == "range":
        if len(args) != 3:
            print("  Usage: swash range <min_us> <max_us>"); return
        try:
            lo = int(args[1]); hi = int(args[2])
        except ValueError:
            print("  Error: min and max must be integers"); return
        if not (800 <= lo < hi <= 2200):
            print(f"  Error: need 800 <= min < max <= 2200 (got {lo}..{hi})"); return
        # NOTE: H_COL_MIN/MAX is the parameter the heli mixer respects; SERVOn_MIN/MAX
        # is overwritten by the heli library every output tick (see AP_MotorsHeli_Swash).
        for nm, val in (("H_COL_MIN", lo), ("H_COL_MAX", hi)):
            session.set_param(nm, float(val))
            actual = session.get_param(nm)
            tag = "[OK]  " if actual is not None and abs(actual - val) < 1.0 else "[FAIL]"
            print(f"  {tag} {nm} = {actual}")
        return
    if sub == "neutral":
        targets = list(SWASH_SERVOS)
        if len(args) >= 2:
            try:
                targets = [int(args[1])]
            except ValueError:
                print("  Usage: swash neutral [n]"); return
        for n in targets:
            _send_set_servo(session, n, PWM_NEUTRAL)
        print(f"  Output(s) {targets} -> {PWM_NEUTRAL} us")
        return
    # Positional: swash <coll%> [lon%] [lat%]
    try:
        coll = float(args[0]) / 100.0
        lon  = float(args[1]) / 100.0 if len(args) > 1 else 0.0
        lat  = float(args[2]) / 100.0 if len(args) > 2 else 0.0
    except ValueError:
        print("  Error: values must be numbers"); return
    s1n, s2n, s3n = _h3_forward_mix(coll, lon, lat)
    pwm1, pwm2, pwm3 = _norm_to_pwm(s1n), _norm_to_pwm(s2n), _norm_to_pwm(s3n)
    _send_set_servo(session, SERVO_S1, pwm1)
    _send_set_servo(session, SERVO_S2, pwm2)
    _send_set_servo(session, SERVO_S3, pwm3)
    print(f"  swash coll={coll*100:.0f}% lon={lon*100:.0f}% lat={lat*100:.0f}%")
    print(f"    S1={pwm1} us  S2={pwm2} us  S3={pwm3} us")


def _cmd_servo(session: RawesGCS, args: list[str]) -> None:
    """servo <ch> <pwm>
       servo sweep <ch> [--step-ms N]
       servo hold <ch> <pwm> [--duration N]"""
    if not args:
        print("  Usage: servo <ch> <pwm>")
        print("         servo sweep <ch> [--step-ms N]")
        print("         servo hold <ch> <pwm> [--duration N]")
        return
    sub = args[0].lower()
    if sub == "sweep":
        try:
            pos, flags = _parse_flags(args[1:], {"--step-ms": "int"})
        except ValueError as e:
            print(f"  Error: {e}"); return
        if len(pos) != 1:
            print("  Usage: servo sweep <ch> [--step-ms N]"); return
        try:
            ch = int(pos[0])
        except ValueError:
            print("  Error: ch must be an integer"); return
        step = flags.get("--step-ms", 5)
        _sweep(session, ch, step_ms=step)
        return
    if sub == "hold":
        try:
            pos, flags = _parse_flags(args[1:], {"--duration": "float"})
        except ValueError as e:
            print(f"  Error: {e}"); return
        if len(pos) != 2:
            print("  Usage: servo hold <ch> <pwm> [--duration N]"); return
        try:
            ch = int(pos[0]); pwm = int(pos[1])
        except ValueError:
            print("  Error: ch and pwm must be integers"); return
        if not (800 <= pwm <= PWM_MAX):
            print(f"  Error: pwm must be 800-{PWM_MAX}"); return
        duration = flags.get("--duration", 60.0)
        if not _arm(session, force=True):
            _safety_shutdown(session, skip_motor_off=(ch != SERVO_MOTOR))
            return
        print("  [OK] Armed.")
        deadline = time.monotonic() + duration
        try:
            while time.monotonic() < deadline:
                _send_set_servo(session, ch, pwm)
                time.sleep(0.1)
                if _esc_check():
                    print("\n  [ESC] abort"); break
        except KeyboardInterrupt:
            print()
        finally:
            _safety_shutdown(session, skip_motor_off=(ch != SERVO_MOTOR))
        return
    # Positional: servo <ch> <pwm>
    if len(args) < 2:
        print("  Usage: servo <ch> <pwm>"); return
    try:
        ch  = int(args[0]); pwm = int(args[1])
    except ValueError:
        print("  Error: ch and pwm must be integers"); return
    if not (1 <= ch <= 16):
        print("  Error: ch must be 1-16"); return
    if not (PWM_MIN <= pwm <= PWM_MAX):
        print(f"  Error: pwm must be {PWM_MIN}-{PWM_MAX}"); return
    _send_set_servo(session, ch, pwm)
    print(f"  Output {ch} -> {pwm} us")


def _cmd_motor(session: RawesGCS, args: list[str], *, force: bool) -> None:
    """motor <pwm_us> [--duration N]
       motor off

    Run-style lifecycle: arms via RAWES_ARM, releases the motor output from any
    AP mixer, then drives the motor output at the requested PWM for `duration`
    seconds while logging telemetry to
    simulation/logs/calibrate/motor_<pwm>_<ts>.csv.  On exit (timer / ESC /
    Ctrl-C / exception): motor -> idle (off), disarm, SERVO<motor>_FUNCTION restored."""
    if not args:
        print("  Usage: motor <pwm_us> [--duration N]  OR  motor off"); return
    if args[0].lower() in ("off", "stop"):
        # Immediate stop: force the motor to idle (off) and disarm.
        try:
            _send_set_servo(session, SERVO_MOTOR, MOTOR_OFF_US)
            print(f"  SERVO{SERVO_MOTOR} -> {MOTOR_OFF_US} us (motor off)")
        except Exception as e:
            print(f"  [WARN] failed to drive SERVO{SERVO_MOTOR} off: {e}")
        try:
            _disarm(session, timeout=5.0)
        except Exception as e:
            print(f"  [WARN] disarm failed: {e}")
        return
    try:
        pos, flags = _parse_flags(args, {"--duration": "float"})
    except ValueError as e:
        print(f"  Error: {e}"); return
    if not pos:
        print("  Usage: motor <pwm_us> [--duration N]"); return
    try:
        pwm = int(pos[0])
    except ValueError:
        print("  Error: pwm must be an integer (microseconds)"); return
    secs = flags.get("--duration", 5.0)

    # Read the live SERVO<motor>_MIN / SERVO<motor>_MAX caps so the prompt + clamp
    # warning reflect the per-bench safety cap (e.g. MAX=1100 during early tuning).
    mot_min_key = f"SERVO{SERVO_MOTOR}_MIN"
    mot_max_key = f"SERVO{SERVO_MOTOR}_MAX"
    s4_min = int(session.get_param(mot_min_key) or MOTOR_OFF_US)
    s4_max = int(session.get_param(mot_max_key) or MOTOR_FULL_US)
    if not (s4_min <= pwm <= s4_max):
        print(f"  Error: pwm must be in [{s4_min}, {s4_max}]  ({mot_min_key}/{mot_max_key}); got {pwm}")
        return
    # Express PWM as fraction of the configured range for the safety prompt.
    if s4_max > s4_min:
        pct_for_prompt = (pwm - s4_min) / (s4_max - s4_min) * 100.0
    else:
        pct_for_prompt = 0.0
    if not force and pct_for_prompt > 5.0:
        confirm = input(
            f"  WARNING: SERVO{SERVO_MOTOR} = {pwm} us "
            f"({pct_for_prompt:.0f}% of [{s4_min},{s4_max}]) for {secs:.0f}s. "
            f"Confirm (y/N): ")
        if confirm.strip().lower() != "y":
            print("  Cancelled."); return

    # Same shuffle as `run yaw`: release the motor output from any AP mixer so our
    # DO_SET_SERVO commands win.
    saved_fn = _take_servo4(session)

    # MODE_PASSIVE / MODE_YAW would also drive SERVO4 -- force Lua to NONE.
    saved_overrides: dict[str, float] = {}
    saved_scr = session.get_param("SCR_USER6")
    if saved_scr is not None and int(saved_scr) != 0:
        saved_overrides["SCR_USER6"] = float(saved_scr)
        session.set_param("SCR_USER6", 0)
        print(f"  SCR_USER6 {int(saved_scr)} -> 0 (motor needs direct SERVO{SERVO_MOTOR} control)")

    if not _arm(session, force=True):
        _safety_shutdown(session, saved_servo4_fn=saved_fn, saved_overrides=saved_overrides)
        return
    print("  [OK] Armed.")

    # DShot self-arms from idle -- no ESC pre-arm hold needed.
    print(f"  Motor: SERVO{SERVO_MOTOR} = {pwm} us for {secs:.1f}s "
          f"(SERVO{SERVO_MOTOR} range [{s4_min}, {s4_max}]).")

    # Snapshot params for the log header
    meta = {
        "verb":             "motor",
        "pwm_us":           pwm,
        "duration_s":       secs,
        mot_min_key:        s4_min,
        mot_max_key:        s4_max,
        "run_start_local":  datetime.now().isoformat(timespec="seconds"),
        "run_start_utc":    datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    log = _RunLog.open("motor", f"{pwm}us", meta)
    print(f"  Logging to {log.path}")

    # On_tick refreshes the motor PWM ~twice per second while t_rel < secs.
    # MAV_CMD_DO_SET_SERVO persists for 30 s on AP, so once we cross the
    # duration boundary we must explicitly drive it back to idle (motor off) --
    # the loop continues observing for the standard +5 s post-window so we get
    # spin-down telemetry before safety_shutdown disarms.
    last_send = [-10.0]
    stopped   = [False]
    def on_tick(t_rel: float) -> None:
        target = pwm if t_rel < secs else MOTOR_OFF_US
        if t_rel >= secs and not stopped[0]:
            _send_set_servo(session, SERVO_MOTOR, MOTOR_OFF_US)
            stopped[0] = True
            last_send[0] = t_rel
            return
        if t_rel - last_send[0] >= 0.5:
            _send_set_servo(session, SERVO_MOTOR, target)
            last_send[0] = t_rel

    try:
        _run_observation(session, "motor", secs, log, on_tick=on_tick)
    finally:
        log.close()
        print(f"  Wrote {log.n_rows} rows to {log.path}")
        _safety_shutdown(session, saved_servo4_fn=saved_fn,
                         saved_overrides=saved_overrides)
    print("  Done.")


def _cmd_arm(session: RawesGCS, args: list[str]) -> None:
    """arm [--duration N]   -- ACRO + direct arm (no Lua mode change)"""
    try:
        pos, flags = _parse_flags(args, {"--duration": "float"})
    except ValueError as e:
        print(f"  Error: {e}"); return
    if pos:
        print("  Usage: arm [--duration N]  (use --duration, not positional)"); return
    print("  Setting ACRO mode ...")
    session.set_mode(1)
    if not _arm(session, force=True):
        print("  [WARN] Arm failed.")
    else:
        print("  [OK] Armed.")


def _cmd_script(session: RawesGCS, args: list[str]) -> None:
    """script upload <file>
       script list
       script remove <name>"""
    if not args:
        print("  Usage: script upload <file>")
        print("         script list")
        print("         script remove <name>")
        return
    sub = args[0].lower()
    if sub == "upload":
        if len(args) != 2:
            print("  Usage: script upload <file>"); return
        _upload_script(session, args[1])
        return
    if sub == "list":
        _list_scripts(session); return
    if sub == "remove":
        if len(args) != 2:
            print("  Usage: script remove <name>"); return
        _remove_script(session, args[1]); return
    print(f"  Unknown script subcommand {sub!r}  (valid: upload, list, remove)")


def _cmd_config(session: RawesGCS, args: list[str]) -> None:
    """config check|show
       config fix|apply"""
    if not args:
        print("  Usage: config check [--all]  OR  config fix [--all]"); return
    sub = args[0].lower()
    alias = {"show": "check", "apply": "fix"}
    sub = alias.get(sub, sub)
    if sub not in ("check", "fix"):
        print(f"  Unknown config subcommand {sub!r}  (valid: check, fix)"); return
    opt_tokens = args[1:]
    use_all = False
    for tok in opt_tokens:
        if tok == "--all":
            use_all = True
        else:
            print(f"  Unknown option {tok!r} (valid: --all)")
            return
    apply = (sub == "fix")
    target = _CONFIG_TARGET_PARAMS_ALL if use_all else _CONFIG_TARGET_PARAMS_COMMON
    scope = "all shared defaults" if use_all else "rawes_common overrides only"
    action = "Applying" if apply else "Preview -- 'config fix' to write"
    print(f"  RAWES config  [{action}]")
    print(f"  Scope: {scope}")
    print(f"  Source: {_AP_BASE_PARM_PATH}")
    print(f"          {_RAWES_COMMON_PARM_PATH}")
    print("          (SITL-only rawes_sitl_defaults.parm excluded)")
    print("          (hardware calibration params excluded)")
    print()
    print(f"  {'Parameter':<25} {'Expected':>8}  {'Actual':>10}  Status")
    print(f"  {'-'*25}  {'-'*8}  {'-'*10}  ------")
    any_diff = False
    any_fail = False
    for name in sorted(target):
        expected = target[name]
        actual = session.get_param(name)
        if actual is None:
            print(f"  {name:<25} {str(expected):>8}  {'N/A':>10}  [FAIL] not found")
            any_fail = True
        elif abs(actual - float(expected)) < 1e-4:
            print(f"  {name:<25} {str(expected):>8}  {actual:>10.4g}  OK")
        else:
            any_diff = True
            if apply:
                ok = session.set_param(name, float(expected))
                if ok:
                    print(f"  {name:<25} {str(expected):>8}  {actual:>10.4g}  -> SET")
                else:
                    print(f"  {name:<25} {str(expected):>8}  {actual:>10.4g}  [FAIL] no ACK")
                    any_fail = True
            else:
                print(f"  {name:<25} {str(expected):>8}  {actual:>10.4g}  DIFF")
    print()
    if apply and any_diff and not any_fail:
        print("  Done -- consider 'reboot' to apply any boot-time params.")
    elif any_fail:
        print("  Done with failures -- check above.")
    elif any_diff:
        print("  Done -- run 'config fix' to write the DIFFs.")
    else:
        print("  Done -- everything matches.")


def _repl(session: RawesGCS) -> None:
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
        if not _run_command(session, tokens, force=False):
            print(f"  Unknown command: {cmd!r}  (type 'help')")


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="RAWES calibration tool -- servo, motor, and Lua script management",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_HELP,
    )
    p.add_argument("--port", "-p", default=None,
                   help="Serial port (e.g. COM4), or 'sitl' / 'tcp:localhost:5760' for SITL")
    p.add_argument("--baud", "-b", "--rate", dest="baud", type=int, default=115200,
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

def _resolve_port(port: "str | None", baud: int) -> tuple:
    """
    Return (port, baud) to use for the connection.

    TCP/UDP addresses and the 'sitl' shorthand bypass serial scanning entirely.
    If port is None, scans all COM ports and returns the first that gives a
    heartbeat (trying baud then _FALLBACK_BAUDS in order).
    If port is given, probes that port with baud fallbacks until a heartbeat
    is received, then returns the working (port, baud).
    Raises SystemExit if nothing responds.
    """
    # SITL shorthand and raw TCP/UDP strings go straight to pymavlink.
    if port is not None:
        if port == "sitl":
            print("Using SITL shorthand -> tcp:localhost:5760")
            return "tcp:localhost:5760", baud
        if port.startswith(("tcp:", "udp:", "udpin:", "tcpin:")):
            return port, baud
    try:
        import serial.tools.list_ports as _list_ports
    except ImportError:
        raise SystemExit("pyserial not installed -- cannot scan COM ports")

    if port is None:
        ports = sorted(_list_ports.comports(), key=lambda p: p.device)
        if not ports:
            raise SystemExit("No COM ports found.")
        candidates = [(info.device, (info.description or "").strip()) for info in ports]
        print(f"No port specified -- scanning {len(candidates)} COM port(s) ...")
    else:
        candidates = [(port, "")]

    all_bauds = [baud] + [b for b in _FALLBACK_BAUDS if b < baud]
    for dev, desc in candidates:
        label = f"{dev}  {desc}".strip()
        for try_baud in all_bauds:
            suffix = f" (fallback)" if try_baud != baud else ""
            print(f"  {label:<50} {try_baud} baud{suffix} ... ", end="", flush=True)
            ok, sysid = _probe_port(dev, try_baud, timeout=3.0)
            if ok:
                print(f"[OK] sysid={sysid}")
                return dev, try_baud
            print("--")

    tried = "/".join(str(b) for b in all_bauds)
    if port is None:
        raise SystemExit(f"No ArduPilot heartbeat on any COM port (tried {tried}).")
    raise SystemExit(f"No heartbeat from {port} at {tried}.")


def _connect(port: "str | None", baud: int) -> RawesGCS:
    port, baud = _resolve_port(port, baud)
    is_tcp = port.startswith(("tcp:", "udp:", "udpin:", "tcpin:"))
    if is_tcp:
        print(f"Connecting to {port} ...")
    else:
        print(f"Connecting to {port} at {baud} baud ...")
    session = RawesGCS(address=port, baud=baud, clock=WallClock())
    session.connect(timeout=15.0)
    print(f"Connected: sysid={session._target_system} compid={session._target_component}")
    session.start_heartbeat()
    session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 10)
    _refresh_pole_pairs(session)
    return session


def _extract_late_connection_flags(tokens: list[str],
                                   port: "str | None",
                                   baud: int) -> tuple[list[str], "str | None", int]:
    """Pull connection flags from command args so users can place them after verbs.

    Accepts: --port/-p <port>, --baud/-b/--rate <baud>
    Returns remaining tokens plus resolved (port, baud).
    """
    out: list[str] = []
    i = 0
    while i < len(tokens):
        t = tokens[i]
        if t in ("--port", "-p"):
            if i + 1 >= len(tokens):
                raise SystemExit("Missing value for --port")
            port = tokens[i + 1]
            i += 2
            continue
        if t in ("--baud", "-b", "--rate"):
            if i + 1 >= len(tokens):
                raise SystemExit(f"Missing value for {t}")
            try:
                baud = int(tokens[i + 1])
            except ValueError:
                raise SystemExit(f"Invalid baud value for {t}: {tokens[i + 1]!r}")
            i += 2
            continue
        out.append(t)
        i += 1
    return out, port, baud


def main() -> None:
    args = _build_parser().parse_args()

    cmd_args = list(args.args)
    if args.command is not None:
        cmd_args, late_port, late_baud = _extract_late_connection_flags(
            cmd_args, args.port, args.baud)
        args.port = late_port
        args.baud = late_baud

    if args.command == "ping":
        _ping_ports(baud=args.baud)
        return

    if args.command == "analyze":
        # Offline CSV analysis; no FC connection required.
        _cmd_analyze(list(args.args))
        return

    session = _connect(args.port, args.baud)
    exit_code = 0
    try:
        if args.command:
            tokens = [args.command] + cmd_args
            ok = _run_command(session, tokens, force=args.force)
            if not ok:
                print(f"Unknown command: {args.command!r}")
                _build_parser().print_help()
                exit_code = 1
        else:
            _repl(session)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        session.close()
        print("Disconnected.")

    if exit_code:
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
