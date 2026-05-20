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
  Output 2  S2  (swashplate, 120 deg)             SERVO2_FUNCTION = 34
  Output 3  S3  (swashplate, 240 deg)             SERVO3_FUNCTION = 35
  Output 4  GB4008 anti-rotation motor            H_TAIL_TYPE = 4 (DDFP CCW)

SERVO4 PWM range: 800 us (off) ... 2000 us (full throttle)
Swashplate PWM range: 1000 us (min) ... 1500 us (neutral) ... 2000 us (max)

Commands
--------
  See 'help' for the full list. Key commands:
  servo/neutral/swash/motor/sweep  -- hardware calibration
  status                           -- vehicle state, battery, EKF, servo outputs, key params
  monitor/listen                   -- live telemetry streams
  arm/disarm/reboot/mode           -- operations
  getparam/setparam                -- parameter R/W
  ftp-upload/ftp-list/ftp-remove   -- Lua script deployment
  ping                             -- COM port discovery

Notes
-----
  MAV_CMD_DO_SET_SERVO works while DISARMED -- no arming required.
  MAV_CMD_DO_MOTOR_TEST also works while disarmed (designed for bench checks).
  Keep a safe distance from the rotor when testing the motor.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
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
SERVO_MOTOR         = 4   # GB4008 anti-rotation motor -- MAIN OUT 4 (DDFP tail)
MOTOR_TEST_INSTANCE = 4   # Motor test instance for SERVO4 (heli tail output)

SWASH_SERVOS = (SERVO_S1, SERVO_S2, SERVO_S3)

# H3-120 forward mix constants (same geometry as swashplate.py)
_COS120 = math.cos(math.radians(120.0))   # -0.5
_SIN120 = math.sin(math.radians(120.0))   #  0.866
_COS240 = math.cos(math.radians(240.0))   # -0.5
_SIN240 = math.sin(math.radians(240.0))   # -0.866

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




_PARAMS_JSON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rawes_params.json")

def _load_params_json() -> dict:
    with open(_PARAMS_JSON_PATH) as fh:
        return json.load(fh)

def _params_as_tuples(entries: list) -> list:
    return [(e["name"], e["expected"], e["note"]) for e in entries]

_PARAMS_JSON   = _load_params_json()
_KEY_PARAMS    = _params_as_tuples(_PARAMS_JSON["key"])
_TAIL_PARAMS   = _params_as_tuples(_PARAMS_JSON["tail_pid"])

_LUA_MODES = {0: "none", 1: "steady", 2: "yaw", 4: "landing", 5: "pumping"}

# Short explanation of what each Lua mode does.  Kept next to _LUA_MODES so the
# two never drift apart.  Used by `mode` and `help` commands.
_LUA_MODE_DOC = {
    0: "passive (default on boot): no RC overrides; periodic logging only",
    1: "steady flight: cyclic altitude hold (RAWES_ALT) + VZ PI collective at 50 Hz. No yaw control -- AP tail PID is disabled in hardware config (SERVO4_FUNCTION=0)",
    2: "Lua yaw PID: holds yaw rate = 0 by driving SERVO4 directly. Hardware default SERVO4_FUNCTION=0 means no `yawmanual` SERVO4_FUNCTION shuffle needed",
    4: "landing (reserved -- not yet implemented in rawes.lua)",
    5: "De Schutter pumping cycle: phase via NAMED_VALUE_FLOAT('RAWES_SUB',N), tension PID on collective. No yaw control (AP tail PID disabled)",
}


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
    srv = session._recv(type="SERVO_OUTPUT_RAW", blocking=True, timeout=2.0)
    if srv:
        for i in range(1, 9):
            val = getattr(srv, f"servo{i}_raw", 0)
            if not val:
                continue
            tag = {SERVO_S1: "  <- S1 (0 deg)", SERVO_S2: "  <- S2 (120 deg)",
                   SERVO_S3: "  <- S3 (240 deg)"}.get(i, "")
            if i == SERVO_MOTOR:
                if val <= 800:
                    tag = "  <- GB4008 off"
                else:
                    pct = (val - 800) / (2000 - 800) * 100
                    tag = f"  <- GB4008 {pct:.0f}%"
            print(f"  Ch {i}: {val} us{tag}")
    else:
        print("  (no SERVO_OUTPUT_RAW received)")

    # --- key params ----------------------------------------------------------
    print(f"\n{sep}")
    print("KEY PARAMS")
    print(sep)
    for name, expected, note in _KEY_PARAMS:
        val = session.get_param(name)
        if val is None:
            print(f"  {name:<22} NOT FOUND  ({note})")
            continue
        if name == "SCR_USER6":
            lua_name = _LUA_MODES.get(int(val), f"mode_{int(val)}")
            print(f"  {name:<22} {val:<8.4g}  {lua_name}  ({note})")
        elif expected is not None and abs(val - float(expected)) > 1e-4:
            print(f"  {name:<22} {val:<8.4g}  [DIFF] expected {expected}")
        else:
            print(f"  {name:<22} {val:<8.4g}  OK  ({note})")

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
    for name, expected, note in _TAIL_PARAMS:
        val = session.get_param(name)
        if val is None:
            print(f"  {name:<22} NOT FOUND  ({note})")
        elif abs(val - float(expected)) > 1e-4:
            print(f"  {name:<22} {val:<10.4g}  [DIFF] expected {expected}  ({note})")
        else:
            print(f"  {name:<22} {val:<10.4g}  ({note})")

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

    deadline = time.monotonic() + duration
    last_print = 0.0
    try:
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg = session._recv(type="ESC_TELEMETRY_1_TO_4",
                                blocking=True, timeout=min(0.5, remaining))
            if msg is None:
                continue
            now = time.monotonic()
            if now - last_print < 0.25:   # print at ~4 Hz max
                continue
            last_print = now

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
  status                          Vehicle state, battery, EKF, servo outputs, key params
  monitor [seconds]               Stream live ESC telemetry (default 10 s)
  listen [seconds]                Stream STATUSTEXT + armed state (Ctrl-C to stop)
  mode <n>                        Set SCR_USER6 Lua mode and listen 10 s to confirm active
                                    0=none (passive)   1=steady (alt hold + VZ PI)
                                    2=yaw  (manual Lua yaw PID, drives SERVO4 directly)
                                    4=landing (reserved)   5=pumping (De Schutter cycle)
  arm [seconds]                   Set ACRO mode, send RAWES_ARM (default 10 s)
  hold <pwm> [seconds]            Arm, hold GB4008 output at pwm us (Ctrl-C to stop)
  yawmanual [s] [key=V ...]       Arm, set mode 2 (Lua yaw PID); logs tuning_<ts>.csv.
                                    Per-run param overrides (restored on exit):
                                      p, i, d, ff, imax, trim, flte, fltt, fltd,
                                      accelmax, servo_min, servo_max
                                    Cyclic + collective trim (sent as NAMED_VALUE_FLOAT
                                    so MODE_YAW holds the IC operating point and rotates
                                    the cyclic to stay aligned with world-down as the
                                    body yaws):
                                      tlon=<rad>  -> RAWES_TLN  (cyclic trim longitudinal)
                                      tlat=<rad>  -> RAWES_TLT  (cyclic trim lateral)
                                      col=<rad>   -> RAWES_COL  (IC collective)
                                    e.g. yawmanual 30 p=0.015 servo_max=1100 tlon=0.02 col=-0.15
  disarm                          Disarm vehicle
  pidtune [key=V ...]   Read or set yaw PID params (keys: p i d ff imax trim flte fltt fltd accelmax)
  config [--apply]                Diff (or write) all params from rawes_params.json; --apply writes + reboots
  reboot                          Reboot ArduPilot
  getparam <name>                 Read a parameter value
  setparam <name> <value>         Write a parameter value
  ftp-list                        List files in /APM/scripts on SD card
  ftp-remove <file>               Remove a file from /APM/scripts
  ftp-upload <file> [--no-restart] Upload .lua file to /APM/scripts and restart
  ping [baud]                     Scan all COM ports and report which have ArduPilot
  help                            Show this list
  quit                            Exit
"""


def _arm(session: RawesGCS, force: bool = False,
         timeout: float = 15.0) -> bool:
    """
    Arm sequence:
      1. Set throttle RC override to 1000 (CH3 interlock low).
      2. Send MAV_CMD_COMPONENT_ARM_DISARM; wait for armed heartbeat.
      3. ESC arm: hold output 4 at 800 us for 5 s so the REVVitRC arms.
    Returns True if vehicle confirms armed.
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

    print(f"  ESC arm: output {SERVO_MOTOR} -> 800 us for 5 s ...")
    t_end = time.monotonic() + 5.0
    while time.monotonic() < t_end:
        _send_set_servo(session, SERVO_MOTOR, 800)
        time.sleep(0.1)
    print("  ESC arm sequence complete -- motor ready.")

    return True


def _disarm(session: RawesGCS, timeout: float = 10.0) -> bool:
    """Send disarm command. Returns True if vehicle confirms disarmed."""
    print("  Sending disarm command ...")
    session._mav.mav.command_long_send(
        session._target_system, session._target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0,    # param1: 0 = disarm
        0, 0, 0, 0, 0, 0,
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


def _run_command(session: RawesGCS, tokens: list[str],
                 force: bool = False) -> bool:
    """
    Execute one calibration command.

    tokens : command + arguments, e.g. ["motor", "10"]
    force  : skip interactive confirmation prompts (for CLI / scripted use)

    Returns True if the command was recognised, False for unknown commands.
    """
    if not tokens:
        return True
    cmd = tokens[0].lower()

    # -- status -------------------------------------------------------------
    if cmd == "status":
        _print_status(session)

    # -- servo <n> <pwm> ----------------------------------------------------
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
        _send_set_servo(session, n, pwm)
        print(f"  Output {n} -> {pwm} us")

    # -- neutral [n] --------------------------------------------------------
    elif cmd == "neutral":
        if len(tokens) >= 2:
            try:
                targets = [int(tokens[1])]
            except ValueError:
                print("  Usage: neutral [n]"); return True
        else:
            targets = list(SWASH_SERVOS)
        for n in targets:
            _send_set_servo(session, n, PWM_NEUTRAL)
        print(f"  Output(s) {targets} -> {PWM_NEUTRAL} us")

    # -- swash <coll_%> [lon_%] [lat_%] -------------------------------------
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
        _send_set_servo(session, SERVO_S1, pwm1)
        _send_set_servo(session, SERVO_S2, pwm2)
        _send_set_servo(session, SERVO_S3, pwm3)
        print(f"  swash coll={coll*100:.0f}% lon={lon*100:.0f}% lat={lat*100:.0f}%")
        print(f"    S1={pwm1} us  S2={pwm2} us  S3={pwm3} us")

    # -- motor <pct> | motor off --------------------------------------------
    elif cmd == "motor":
        if len(tokens) < 2:
            print("  Usage: motor <pct>  OR  motor off"); return True
        arg = tokens[1].lower()
        if arg in ("off", "stop", "0"):
            _send_motor_test(session, MOTOR_TEST_INSTANCE, 0.0, timeout_s=0.0)
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
            _send_motor_test(session, MOTOR_TEST_INSTANCE, pct, timeout_s=5.0)
            print(f"  Motor test: {pct:.0f}% for 5 s  (send 'motor off' to stop early)")

    # -- sweep <n> [step_ms] ------------------------------------------------
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
        _sweep(session, n, step_ms)

    # -- monitor [seconds] --------------------------------------------------
    elif cmd == "monitor":
        try:
            secs = float(tokens[1]) if len(tokens) > 1 else 10.0
        except ValueError:
            print("  Usage: monitor [seconds]"); return True
        _monitor_esc(session, duration=secs)

    # -- mode <n> -----------------------------------------------------------
    elif cmd == "mode":
        if len(tokens) < 2:
            print("  Usage: mode <n>   (valid: " +
                  ", ".join(str(k) for k in sorted(_LUA_MODES)) + ")")
            for k in sorted(_LUA_MODES):
                print(f"    {k} = {_LUA_MODES[k]:<8} -- {_LUA_MODE_DOC[k]}")
            return True
        try:
            n = int(tokens[1])
        except ValueError:
            print("  Error: mode must be an integer"); return True
        if n not in _LUA_MODES:
            print(f"  Error: unknown mode {n}  (valid: {sorted(_LUA_MODES)})"); return True
        if not session.set_param("SCR_USER6", float(n)):
            print("  [FAIL] Could not set SCR_USER6"); return True
        name = _LUA_MODES[n]
        print(f"  [OK] SCR_USER6={n} ({name}) -- {_LUA_MODE_DOC[n]}")
        print(f"  Listening 10 s for STATUSTEXT ...")
        t0 = time.monotonic()
        seen = []
        try:
            while time.monotonic() - t0 < 10.0:
                remaining = 10.0 - (time.monotonic() - t0)
                msg = session._recv(type="STATUSTEXT", blocking=True,
                                    timeout=min(0.5, remaining))
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

    # -- listen [seconds] ---------------------------------------------------
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
                msg = session._recv(type=["STATUSTEXT", "HEARTBEAT"],
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

    # -- hold <pwm> [seconds] -----------------------------------------------
    elif cmd == "hold":
        if len(tokens) < 2:
            print("  Usage: hold <pwm> [seconds]"); return True
        try:
            pwm = int(tokens[1])
            duration = float(tokens[2]) if len(tokens) > 2 else None
        except ValueError:
            print("  Error: pwm must be an integer"); return True
        if not (800 <= pwm <= PWM_MAX):
            print(f"  Error: pwm must be 800-{PWM_MAX}"); return True

        # Arm via RAWES_ARM (same mechanism as the arm command).
        # Give Lua enough countdown to cover the hold duration plus a 10 s buffer.
        arm_ms = int((duration + 10.0) * 1000) if duration else 300_000
        print(f"  Sending RAWES_ARM={arm_ms} ms ...")
        session.send_named_float("RAWES_ARM", float(arm_ms))
        print("  Waiting for armed state ...")
        arm_deadline = time.monotonic() + 15.0
        armed_now = False
        while time.monotonic() < arm_deadline:
            msg = session._recv(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                print(f"  [FC] {msg.text.rstrip()}")
            elif msg.get_type() == "HEARTBEAT":
                if bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    armed_now = True
                    print("  [OK] Vehicle armed.")
                    break
        if not armed_now:
            print("  [FAIL] Did not arm within 15 s.")
            return True

        # Release ArduPilot's ownership of the output for the duration of the hold.
        servo_key = f"SERVO{SERVO_MOTOR}_FUNCTION"
        saved_fn = session.get_param(servo_key)
        if saved_fn is not None and saved_fn != 0:
            session.set_param(servo_key, 0)
            print(f"  {servo_key} {saved_fn:.0f} -> 0 (released)")
        else:
            saved_fn = None  # nothing to restore

        print(f"  Holding output {SERVO_MOTOR} at {pwm} us  (Ctrl-C to stop) ...")
        _send_set_servo(session, SERVO_MOTOR, pwm)
        t0 = time.monotonic()
        last_armed = True
        try:
            while True:
                if duration and (time.monotonic() - t0) >= duration:
                    break
                _send_set_servo(session, SERVO_MOTOR, pwm)
                elapsed = time.monotonic() - t0
                while True:
                    msg = session._recv(blocking=True, timeout=0.1)
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
        _send_set_servo(session, SERVO_MOTOR, 800)
        print(f"  Output {SERVO_MOTOR} -> 800 us (safe)")
        if saved_fn is not None:
            session.set_param(servo_key, saved_fn)
            print(f"  {servo_key} restored to {saved_fn:.0f}")

    # -- yawmanual [seconds] [key=value ...] -------------------------------
    elif cmd == "yawmanual":
        # Per-run parameter overrides.  Keys mirror `pidtune` plus SERVO4_MIN/MAX
        # for PWM caps.  Each override is restored on exit (normal, ESC, Ctrl-C,
        # exception) via the same safety-shutdown path that handles disarm.
        _YAW_OVERRIDES = {
            "p":         "ATC_RAT_YAW_P",
            "i":         "ATC_RAT_YAW_I",
            "d":         "ATC_RAT_YAW_D",
            "ff":        "ATC_RAT_YAW_FF",
            "imax":      "ATC_RAT_YAW_IMAX",
            "trim":      "H_YAW_TRIM",
            "flte":      "ATC_RAT_YAW_FLTE",
            "fltt":      "ATC_RAT_YAW_FLTT",
            "fltd":      "ATC_RAT_YAW_FLTD",
            "accelmax":  "ATC_ACCEL_Y_MAX",
            "servo_min": "SERVO4_MIN",
            "servo_max": "SERVO4_MAX",
        }
        # Lua-side trim values sent via NAMED_VALUE_FLOAT (NOT params).
        # Consumed by rawes.lua MODE_YAW's set_trim_ic_rc_overrides for
        # cyclic + collective hold, rotated to current body yaw each tick.
        _YAW_NVF_TRIM = {
            "tlon": "RAWES_TLN",   # cyclic trim longitudinal [rad]
            "tlat": "RAWES_TLT",   # cyclic trim lateral      [rad]
            "col":  "RAWES_COL",   # IC collective            [rad]
        }
        secs: float | None = None
        overrides: dict[str, float] = {}
        nvf_trim:  dict[str, float] = {}
        args = tokens[1:]
        # First positional arg may be a duration (no "=").  Everything else
        # must be key=value.
        if args and "=" not in args[0]:
            try:
                secs = float(args[0])
                args = args[1:]
            except ValueError:
                print("  Usage: yawmanual [seconds] [key=value ...]")
                print(f"  Param keys: {', '.join(sorted(_YAW_OVERRIDES))}")
                print(f"  NVF trim keys (rad): {', '.join(sorted(_YAW_NVF_TRIM))}")
                return True
        for tok in args:
            if "=" not in tok:
                print("  Usage: yawmanual [seconds] [key=value ...]")
                print(f"  Param keys: {', '.join(sorted(_YAW_OVERRIDES))}")
                print(f"  NVF trim keys (rad): {', '.join(sorted(_YAW_NVF_TRIM))}")
                return True
            key, _, val_str = tok.partition("=")
            key = key.lower()
            if key in _YAW_OVERRIDES:
                target = overrides
                map_to = _YAW_OVERRIDES[key]
            elif key in _YAW_NVF_TRIM:
                target = nvf_trim
                map_to = _YAW_NVF_TRIM[key]
            else:
                valid = sorted(list(_YAW_OVERRIDES) + list(_YAW_NVF_TRIM))
                print(f"  Unknown key {key!r}  (valid: {', '.join(valid)})")
                return True
            try:
                target[map_to] = float(val_str)
            except ValueError:
                print(f"  Error: value for {key!r} must be a number")
                return True

        # Apply overrides BEFORE the gains snapshot so the CSV metadata header
        # reflects the values actually used during the run.
        saved_overrides: dict[str, float] = {}
        if overrides:
            print("  Per-run overrides:")
            for param, new_val in overrides.items():
                orig = session.get_param(param)
                if orig is None:
                    print(f"    [WARN] {param}: could not read current value -- skipping override")
                    continue
                saved_overrides[param] = float(orig)
                ok = session.set_param(param, new_val)
                tag = "[OK]  " if ok else "[FAIL]"
                print(f"    {tag} {param}: {orig:.6g} -> {new_val:.6g}")

        # Release SERVO4 from DDFP ownership so Lua owns it exclusively
        servo_key = f"SERVO{SERVO_MOTOR}_FUNCTION"
        saved_fn = session.get_param(servo_key)
        if saved_fn is not None and saved_fn != 0:
            session.set_param(servo_key, 0)
            print(f"  {servo_key} {saved_fn:.0f} -> 0 (released from DDFP)")
        else:
            saved_fn = None

        # Activate yaw mode
        session.set_param("SCR_USER6", 2)
        print("  SCR_USER6 -> 2 (yaw mode)")

        # Send any cyclic-trim / collective NVFs the user requested.  These
        # populate _trim_lon / _trim_lat / _ic_col in rawes.lua so MODE_YAW
        # holds cyclic + collective at the operating point (rotated by
        # accumulated gyro:z() since the NVF arrived, so the world-frame
        # disk-tilt direction stays constant regardless of body yaw).
        if nvf_trim:
            print("  Sending trim NVFs:")
            for nv_name, nv_val in nvf_trim.items():
                session.send_named_float(nv_name, nv_val)
                print(f"    {nv_name} = {nv_val:+.4f} rad")

        # Arm via RAWES_ARM
        arm_ms = int((secs + 10.0) * 1000) if secs else 300_000
        print(f"  Sending RAWES_ARM={arm_ms} ms ...")
        session.send_named_float("RAWES_ARM", float(arm_ms))

        arm_deadline = time.monotonic() + 15.0
        armed_now = False
        while time.monotonic() < arm_deadline:
            msg = session._recv(type=["HEARTBEAT", "STATUSTEXT"],
                                blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                print(f"  [FC] {msg.text.rstrip()}")
            elif msg.get_type() == "HEARTBEAT":
                if bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    armed_now = True
                    print("  [OK] Armed.")
                    break
        if not armed_now:
            print("  [FAIL] Did not arm within 15 s.")
            # Mirror the post-run safety shutdown.  Vehicle is unarmed already
            # but we still stop Lua, force the motor PWM to 800, and restore
            # SERVO4_FUNCTION so the next command starts from a known state.
            print("  [SAFETY] shutting down ...")
            try:
                session.set_param("SCR_USER6", 0)
                print("  [SAFETY] SCR_USER6 -> 0 (Lua mode none)")
            except Exception as e:
                print(f"  [SAFETY] failed to set SCR_USER6=0: {e}")
            time.sleep(0.30)
            try:
                _send_set_servo(session, SERVO_MOTOR, 800)
                print(f"  [SAFETY] SERVO{SERVO_MOTOR} -> 800 us (motor off)")
            except Exception as e:
                print(f"  [SAFETY] failed to drive SERVO{SERVO_MOTOR} off: {e}")
            if saved_fn is not None:
                try:
                    session.set_param(servo_key, saved_fn)
                    print(f"  [SAFETY] {servo_key} restored to {saved_fn:.0f}")
                except Exception as e:
                    print(f"  [SAFETY] failed to restore {servo_key}: {e}")
            for param, orig in saved_overrides.items():
                try:
                    session.set_param(param, orig)
                    print(f"  [SAFETY] {param} restored to {orig:.6g}")
                except Exception as e:
                    print(f"  [SAFETY] failed to restore {param}: {e}")
            return True

        # Snapshot Lua yaw-PID gains so the CSV records the gains that produced
        # the captured input/output trace.  rawes.lua reads these every tick, so
        # if the user changes them with `pidtune` during the run the trace will
        # mix gain values -- run a fresh `yawmanual` after retuning for a clean
        # dataset.  Gains are re-read once at exit and the final values printed.
        def _read_yaw_gains() -> dict:
            return {
                "kp":         session.get_param("ATC_RAT_YAW_P")    or 0.0,
                "ki":         session.get_param("ATC_RAT_YAW_I")    or 0.0,
                "kd":         session.get_param("ATC_RAT_YAW_D")    or 0.0,
                "imax":       session.get_param("ATC_RAT_YAW_IMAX") or 0.0,
                "h_yaw_trim": session.get_param("H_YAW_TRIM")       or 0.0,
            }
        gains = _read_yaw_gains()
        print(f"  Gains snapshot: P={gains['kp']:.4f} I={gains['ki']:.4f} "
              f"D={gains['kd']:.4f} IMAX={gains['imax']:.3f} "
              f"H_YAW_TRIM={gains['h_yaw_trim']:.3f}")

        # Timestamped filename so consecutive runs accumulate instead of
        # clobbering.  UTC stamp avoids local-time / DST ambiguity in archives.
        ts_local = datetime.now()
        ts_utc   = datetime.now(timezone.utc)
        ts_tag   = ts_local.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.abspath(f"tuning_{ts_tag}.csv")

        # Snapshot a wider parameter set for the metadata header.  These are the
        # values that determine the loop behaviour but don't change inside the
        # run -- captured here once so the analyzer can find them without
        # querying the FC.  Anything that CAN change tick-to-tick (yaw_i,
        # output_unclamp) is logged per row from NAMED_VALUE_FLOAT.
        def _safe_param(name: str) -> float | None:
            try:
                v = session.get_param(name)
                return None if v is None else float(v)
            except Exception:
                return None
        meta = {
            "run_start_local":   ts_local.isoformat(timespec="seconds"),
            "run_start_utc":     ts_utc.isoformat(timespec="seconds"),
            "duration_arg_s":    secs if secs is not None else "",
            "ATC_RAT_YAW_P":     gains["kp"],
            "ATC_RAT_YAW_I":     gains["ki"],
            "ATC_RAT_YAW_D":     gains["kd"],
            "ATC_RAT_YAW_IMAX":  gains["imax"],
            "H_YAW_TRIM":        gains["h_yaw_trim"],
            "SERVO4_MIN":        _safe_param("SERVO4_MIN"),
            "SERVO4_MAX":        _safe_param("SERVO4_MAX"),
            "SERVO4_TRIM":       _safe_param("SERVO4_TRIM"),
            "SERVO4_FUNCTION":   _safe_param("SERVO4_FUNCTION"),
            "SCR_USER6":         _safe_param("SCR_USER6"),
            "H_TAIL_TYPE":       _safe_param("H_TAIL_TYPE"),
        }

        csv_fh = open(csv_path, "w", newline="")
        # Metadata header: one "# key: value" line per parameter.  Lines start
        # with "#" so the analyzer can split metadata from data, and pandas /
        # csv.reader can skip them via comment='#' (or filter in the loader).
        csv_fh.write("# tuning.csv -- written by calibrate.py yawmanual\n")
        for k, v in meta.items():
            csv_fh.write(f"# {k}: {v}\n")
        csv_fh.write("#\n")
        csv_w = csv.writer(csv_fh)
        csv_w.writerow([
            "t_s", "armed", "yaw_rate_dps", "target_dps", "servo4_pwm",
            "vbat_v", "current_a", "power_w",
            "yaw_i_lua", "yaw_out_unclamp_lua",
            "kp", "ki", "kd", "imax", "h_yaw_trim",
        ])
        print(f"  Logging to {csv_path}")

        # ATTITUDE comes from EXTRA1; BATTERY_STATUS / SYS_STATUS from
        # EXTENDED_STATUS.  Battery is whole-pack (not per-motor) -- on a
        # static bench rig the GB4008 dominates draw, so this is a good
        # proxy for yaw-actuator power.
        session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50)            # ATTITUDE
        session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 50)       # SERVO_OUTPUT_RAW
        session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 10)   # BATTERY_STATUS / SYS_STATUS
        print()
        print(f"  {'t(s)':<6}  {'armed':<5}  {'yaw(d/s)':>9}  {'S4(us)':>6}  {'V':>5}  {'A':>5}  {'W':>5}  STATUSTEXT")
        print(f"  {'-'*6}  {'-'*5}  {'-'*9}  {'-'*6}  {'-'*5}  {'-'*5}  {'-'*5}  ----------")

        t0           = time.monotonic()
        deadline     = (t0 + secs + 5.0) if secs else None
        last_print   = -1.0
        yaw_rate     = None
        servo4_pwm   = None
        vbat_v       = None
        current_a    = None
        yaw_i_lua    = None    # cached from NAMED_VALUE_FLOAT "YAW_I"
        yaw_out_lua  = None    # cached from NAMED_VALUE_FLOAT "YAW_OUT"
        armed_state  = True
        pending_text: list[str] = []
        n_rows = 0
        aborted = False
        print("  Press ESC (or Ctrl-C) to abort.")
        try:
            while True:
                if deadline and time.monotonic() >= deadline:
                    break
                # Non-blocking ESC check.  Polled every loop iteration (~10 Hz
                # given the 0.1 s _recv timeout below), so abort latency is
                # bounded by that timeout.  Drains any other queued keys
                # silently so they don't accumulate in the console buffer.
                while msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key == b"\x1b":
                        aborted = True
                        break
                if aborted:
                    print("\n  [ESC] abort -- running safety shutdown ...")
                    break
                msg = session._recv(
                    type=["ATTITUDE", "SERVO_OUTPUT_RAW", "HEARTBEAT", "STATUSTEXT",
                          "BATTERY_STATUS", "SYS_STATUS", "NAMED_VALUE_FLOAT"],
                    blocking=True, timeout=0.1,
                )
                if msg is not None:
                    mt = msg.get_type()
                    if mt == "ATTITUDE":
                        yaw_rate = math.degrees(msg.yawspeed)
                        # Log one CSV row per ATTITUDE sample (50 Hz stream).
                        # SERVO_OUTPUT_RAW and ATTITUDE arrive on the same
                        # cadence; battery updates at ~10 Hz so we use the
                        # most recent cached vbat/current for the row.
                        # yaw_i_lua / yaw_out_lua arrive at 50 Hz from
                        # rawes.lua's NAMED_VALUE_FLOAT emissions.
                        elapsed_now = time.monotonic() - t0
                        power_w = (vbat_v * current_a) if (vbat_v is not None and current_a is not None) else None
                        csv_w.writerow([
                            f"{elapsed_now:.4f}",
                            int(armed_state),
                            f"{yaw_rate:.4f}",
                            "0.0",   # target rate for mode-2 yaw hold
                            servo4_pwm if servo4_pwm is not None else "",
                            f"{vbat_v:.3f}"    if vbat_v    is not None else "",
                            f"{current_a:.3f}" if current_a is not None else "",
                            f"{power_w:.2f}"   if power_w   is not None else "",
                            f"{yaw_i_lua:.6f}"   if yaw_i_lua   is not None else "",
                            f"{yaw_out_lua:.6f}" if yaw_out_lua is not None else "",
                            f"{gains['kp']:.6f}",
                            f"{gains['ki']:.6f}",
                            f"{gains['kd']:.6f}",
                            f"{gains['imax']:.6f}",
                            f"{gains['h_yaw_trim']:.6f}",
                        ])
                        n_rows += 1
                    elif mt == "SERVO_OUTPUT_RAW":
                        servo4_pwm = getattr(msg, "servo4_raw", None)
                    elif mt == "NAMED_VALUE_FLOAT":
                        # Internal PID state from rawes.lua mode 2.  Caches by
                        # name; unrecognized names are ignored so this is safe
                        # if other components emit named floats on the bus.
                        nv_name = msg.name.rstrip("\x00").strip() if isinstance(msg.name, str) else \
                                  msg.name.decode("ascii", errors="replace").rstrip("\x00").strip()
                        if nv_name == "YAW_I":
                            yaw_i_lua = float(msg.value)
                        elif nv_name == "YAW_OUT":
                            yaw_out_lua = float(msg.value)
                    elif mt == "BATTERY_STATUS":
                        # voltages[] in mV; sum the cells that are reported.
                        # current_battery is cA (centi-amperes); -1 means n/a.
                        cells = [v for v in msg.voltages if v != 65535]
                        if cells:
                            vbat_v = sum(cells) / 1000.0
                        if msg.current_battery >= 0:
                            current_a = msg.current_battery / 100.0
                    elif mt == "SYS_STATUS":
                        # Fallback path -- only used if BATTERY_STATUS is
                        # absent.  Same scaling: voltage_battery in mV,
                        # current_battery in cA.
                        if vbat_v is None and msg.voltage_battery != 65535:
                            vbat_v = msg.voltage_battery / 1000.0
                        if current_a is None and msg.current_battery >= 0:
                            current_a = msg.current_battery / 100.0
                    elif mt == "HEARTBEAT":
                        armed_state = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    elif mt == "STATUSTEXT":
                        text = msg.text.rstrip("\x00").strip()
                        if text:
                            pending_text.append(text)
                elapsed = time.monotonic() - t0
                if elapsed - last_print >= 1.0:
                    last_print   = elapsed
                    yr_str  = f"{yaw_rate:+.2f}" if yaw_rate   is not None else "    n/a"
                    pw_str  = f"{servo4_pwm}"     if servo4_pwm is not None else "  n/a"
                    ar_str  = "YES" if armed_state else "no"
                    v_str   = f"{vbat_v:.2f}"    if vbat_v    is not None else "  n/a"
                    a_str   = f"{current_a:.2f}" if current_a is not None else "  n/a"
                    p_w_val = (vbat_v * current_a) if (vbat_v is not None and current_a is not None) else None
                    p_str   = f"{p_w_val:.1f}"   if p_w_val   is not None else "  n/a"
                    txt_str = pending_text.pop(0) if pending_text else ""
                    print(f"  {elapsed:<6.1f}  {ar_str:<5}  {yr_str:>9}  {pw_str:>6}  "
                          f"{v_str:>5}  {a_str:>5}  {p_str:>5}  {txt_str}")
                    while pending_text:
                        print(f"  {'':6}  {'':5}  {'':9}  {'':6}  {'':5}  {'':5}  {'':5}  {pending_text.pop(0)}")
        except KeyboardInterrupt:
            print()
        finally:
            csv_fh.close()
            gains_end = _read_yaw_gains()
            if gains_end != gains:
                print(f"  [WARN] Gains changed during run -- CSV holds the "
                      f"start snapshot. End values: P={gains_end['kp']:.4f} "
                      f"I={gains_end['ki']:.4f} D={gains_end['kd']:.4f} "
                      f"IMAX={gains_end['imax']:.3f} "
                      f"H_YAW_TRIM={gains_end['h_yaw_trim']:.3f}")
            print(f"  Wrote {n_rows} rows to {csv_path}")

            # Safety shutdown -- runs whether the loop exited normally,
            # via Ctrl-C, or via an exception.  Order matters:
            #   1. Stop Lua first so it cannot re-assert SERVO4 between commands.
            #   2. Wait > 200 ms so the SRV_Channels override timeout expires.
            #   3. Force SERVO4 to 800 us via DO_SET_SERVO (works while
            #      SERVO4_FUNCTION is still 0 -- we restore it after).
            #   4. Disarm the vehicle.
            #   5. Restore the saved SERVO4_FUNCTION.
            # Every step is best-effort: a failure in one does not skip the next.
            print("  [SAFETY] shutting down ...")
            try:
                session.set_param("SCR_USER6", 0)
                print("  [SAFETY] SCR_USER6 -> 0 (Lua mode none)")
            except Exception as e:
                print(f"  [SAFETY] failed to set SCR_USER6=0: {e}")
            time.sleep(0.30)
            try:
                _send_set_servo(session, SERVO_MOTOR, 800)
                print(f"  [SAFETY] SERVO{SERVO_MOTOR} -> 800 us (motor off)")
            except Exception as e:
                print(f"  [SAFETY] failed to drive SERVO{SERVO_MOTOR} off: {e}")
            try:
                if not _disarm(session, timeout=5.0):
                    print("  [SAFETY] disarm not confirmed within 5 s")
            except Exception as e:
                print(f"  [SAFETY] disarm command failed: {e}")
            if saved_fn is not None:
                try:
                    session.set_param(servo_key, saved_fn)
                    print(f"  [SAFETY] {servo_key} restored to {saved_fn:.0f}")
                except Exception as e:
                    print(f"  [SAFETY] failed to restore {servo_key}: {e}")
            # Restore per-run parameter overrides so they don't leak into the
            # next command or persist after a Ctrl-C / ESC.
            for param, orig in saved_overrides.items():
                try:
                    session.set_param(param, orig)
                    print(f"  [SAFETY] {param} restored to {orig:.6g}")
                except Exception as e:
                    print(f"  [SAFETY] failed to restore {param}: {e}")
        print("  Done.")

    # -- reboot -------------------------------------------------------------
    elif cmd == "reboot":
        print("  Sending reboot command ...")
        session._mav.mav.command_long_send(
            session._target_system, session._target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                msg = session._recv(type="HEARTBEAT", blocking=True, timeout=1.0)
                if msg is None:
                    break
            except Exception:
                break
        print("  Pixhawk rebooting -- reconnect in a few seconds.")

    # -- config [--apply] ---------------------------------------------------
    elif cmd == "config":
        apply = "--apply" in tokens
        # Collect all entries with a non-null expected value from rawes_params.json.
        all_entries = []
        for group in _PARAMS_JSON.values():
            for e in group:
                if e["expected"] is not None:
                    all_entries.append(e)

        action = "Applying" if apply else "Preview -- add --apply to write"
        print(f"  RAWES config  [{action}]")
        print(f"  Source: {_PARAMS_JSON_PATH}")
        print()
        print(f"  {'Parameter':<25} {'Expected':>8}  {'Actual':>10}  Status")
        print(f"  {'-'*25}  {'-'*8}  {'-'*10}  ------")
        any_diff = False
        any_fail = False
        for e in all_entries:
            name, expected = e["name"], e["expected"]
            actual = session.get_param(name)
            if actual is None:
                print(f"  {name:<25} {str(expected):>8}  {'N/A':>10}  [FAIL] not found")
                any_fail = True
            elif abs(actual - float(expected)) < 1e-4:
                print(f"  {name:<25} {str(expected):>8}  {f'{actual:.4g}':>10}  [OK]")
            else:
                print(f"  {name:<25} {str(expected):>8}  {f'{actual:.4g}':>10}  [DIFF]")
                any_diff = True

        print()
        if any_fail:
            print("  [FAIL] One or more parameters not found on this firmware.")
        elif not any_diff:
            print("  [OK] All parameters already correct.")
        elif not apply:
            print("  [DIFF] Run 'config --apply' to write changes.")
        else:
            print("  Writing changes ...")
            for e in all_entries:
                session.set_param(e["name"], e["expected"])
            print("  [OK] Parameters written. Rebooting ...")
            session._mav.mav.command_long_send(
                session._target_system, session._target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0, 1, 0, 0, 0, 0, 0, 0,
            )
            print("  Pixhawk rebooting -- reconnect in ~5 s.")

    # -- arm [seconds] [--download] -----------------------------------------
    elif cmd == "arm":
        try:
            secs = float(tokens[1]) if len(tokens) > 1 else 10.0
        except ValueError:
            print("  Usage: arm [seconds]"); return True
        print("  Setting ACRO mode ...")
        session.set_mode(1)   # ACRO = 1 in ArduCopter
        armon_ms = int(secs * 1000)
        print(f"  Sending RAWES_ARM={armon_ms} ms -- Lua will arm and hold for {secs:.0f} s ...")
        session.send_named_float("RAWES_ARM", float(armon_ms))
        # Request ATTITUDE + SERVO_OUTPUT_RAW streams
        session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10)  # ATTITUDE
        session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)
        # Enable yaw PID_TUNING stream (GCS_PID_MASK bit 2 = yaw rate); restore on exit
        old_pid_mask = session.get_param("GCS_PID_MASK")
        session.set_param("GCS_PID_MASK", 4)
        print()
        print(f"  {'t(s)':<6}  {'armed':<5}  {'sp(d/s)':>8}  {'act(d/s)':>8}  {'P':>7}  {'I':>7}  {'D':>7}  {'FF':>7}  {'S4(us)':>6}  STATUSTEXT")
        print(f"  {'-'*6}  {'-'*5}  {'-'*8}  {'-'*8}  {'-'*7}  {'-'*7}  {'-'*7}  {'-'*7}  {'-'*6}  ----------")
        t0        = time.monotonic()
        deadline  = t0 + secs + 5.0   # extra 5 s margin past armon expiry
        last_print  = -1.0
        yaw_rate    = None
        servo4_pwm  = None
        pid_p = pid_i = pid_d = pid_ff = pid_sp = pid_act = None
        armed_state = False
        pending_text: list[str] = []
        try:
            while time.monotonic() < deadline:
                msg = session._recv(
                    type=["ATTITUDE", "SERVO_OUTPUT_RAW", "HEARTBEAT", "STATUSTEXT",
                          "PID_TUNING"],
                    blocking=True, timeout=0.1,
                )
                if msg is not None:
                    mt = msg.get_type()
                    if mt == "ATTITUDE":
                        yaw_rate = math.degrees(msg.yawspeed)
                    elif mt == "SERVO_OUTPUT_RAW":
                        servo4_pwm = getattr(msg, "servo4_raw", None)
                    elif mt == "HEARTBEAT":
                        armed_state = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    elif mt == "STATUSTEXT":
                        text = msg.text.rstrip("\x00").strip()
                        if text:
                            pending_text.append(text)
                    elif mt == "PID_TUNING" and msg.axis == 3:  # 3 = yaw
                        pid_sp  = math.degrees(msg.desired)
                        pid_act = math.degrees(msg.achieved)
                        pid_p   = msg.P
                        pid_i   = msg.I
                        pid_d   = msg.D
                        pid_ff  = msg.FF
                elapsed = time.monotonic() - t0
                if elapsed - last_print >= 1.0:
                    last_print = elapsed
                    # act falls back to ATTITUDE yawspeed until PID_TUNING arrives
                    act_val = pid_act if pid_act is not None else yaw_rate
                    sp_str  = f"{pid_sp:+.2f}"  if pid_sp  is not None else "     n/a"
                    act_str = f"{act_val:+.2f}" if act_val is not None else "     n/a"
                    p_str   = f"{pid_p:+.4f}"   if pid_p   is not None else "    n/a"
                    i_str   = f"{pid_i:+.4f}"   if pid_i   is not None else "    n/a"
                    d_str   = f"{pid_d:+.4f}"   if pid_d   is not None else "    n/a"
                    ff_str  = f"{pid_ff:+.4f}"  if pid_ff  is not None else "    n/a"
                    pw_str  = f"{servo4_pwm}"    if servo4_pwm is not None else "   n/a"
                    ar_str  = "YES" if armed_state else "no"
                    txt_str = pending_text.pop(0) if pending_text else ""
                    print(f"  {elapsed:<6.1f}  {ar_str:<5}  {sp_str:>8}  {act_str:>8}  {p_str:>7}  {i_str:>7}  {d_str:>7}  {ff_str:>7}  {pw_str:>6}  {txt_str}")
                    while pending_text:
                        print(f"  {'':6}  {'':5}  {'':8}  {'':8}  {'':7}  {'':7}  {'':7}  {'':7}  {'':6}  {pending_text.pop(0)}")
        except KeyboardInterrupt:
            print()
        finally:
            if old_pid_mask is not None:
                session.set_param("GCS_PID_MASK", old_pid_mask)
        print(f"  Done.")

    # -- pidtune [p=V] [i=V] [d=V] [imax=V] [trim=V] ----------------------
    elif cmd == "pidtune":
        _YAW_PID_PARAMS = [
            ("p",      "ATC_RAT_YAW_P"),
            ("i",      "ATC_RAT_YAW_I"),
            ("d",      "ATC_RAT_YAW_D"),
            ("ff",     "ATC_RAT_YAW_FF"),
            ("imax",   "ATC_RAT_YAW_IMAX"),
            ("trim",   "H_YAW_TRIM"),
            ("flte",    "ATC_RAT_YAW_FLTE"),
            ("fltt",    "ATC_RAT_YAW_FLTT"),
            ("fltd",    "ATC_RAT_YAW_FLTD"),
            ("accelmax","ATC_ACCEL_Y_MAX"),
        ]
        valid_keys = {k for k, _ in _YAW_PID_PARAMS}
        updates = {}
        parse_ok = True
        for tok in tokens[1:]:
            if "=" not in tok:
                print(f"  Usage: pidtune [p=V] [i=V] [d=V] [imax=V] [trim=V]")
                print(f"  No args = show current values.")
                parse_ok = False
                break
            key, _, val_str = tok.partition("=")
            key = key.lower()
            if key not in valid_keys:
                print(f"  Unknown key {key!r}  (valid: {', '.join(sorted(valid_keys))})")
                parse_ok = False
                break
            try:
                updates[key] = float(val_str)
            except ValueError:
                print(f"  Error: value for {key!r} must be a number")
                parse_ok = False
                break
        if not parse_ok:
            return True
        for key, param in _YAW_PID_PARAMS:
            if key in updates:
                session.set_param(param, updates[key])
        print(f"\n  {'Key':<6}  {'Parameter':<22}  {'Value':>10}")
        print(f"  {'-'*6}  {'-'*22}  {'-'*10}")
        for key, param in _YAW_PID_PARAMS:
            val = session.get_param(param)
            marker = "  <-- updated" if key in updates else ""
            val_str = f"{val:.6g}" if val is not None else "NOT FOUND"
            print(f"  {key:<6}  {param:<22}  {val_str:>10}{marker}")
        print()

    # -- disarm -------------------------------------------------------------
    elif cmd == "disarm":
        _disarm(session)

    # -- getparam <name> ----------------------------------------------------
    elif cmd == "getparam":
        if len(tokens) < 2:
            print("  Usage: getparam <name>"); return True
        name = tokens[1].upper()
        val = session.get_param(name)
        if val is None:
            print(f"  {name}: no response (parameter not found?)")
        else:
            print(f"  {name} = {val}")

    # -- setparam <name> <value> --------------------------------------------
    elif cmd == "setparam":
        if len(tokens) < 3:
            print("  Usage: setparam <name> <value>"); return True
        name = tokens[1].upper()
        try:
            value = float(tokens[2])
        except ValueError:
            print("  Error: value must be a number"); return True
        ok = session.set_param(name, value)
        if ok:
            actual = session.get_param(name)
            print(f"  [OK]   {name} = {actual}")
        else:
            print(f"  [FAIL] {name}: no ACK within timeout")

    # -- ftp-list -----------------------------------------------------------
    elif cmd == "ftp-list":
        _list_scripts(session)

    # -- ftp-remove <file> --------------------------------------------------
    elif cmd == "ftp-remove":
        if len(tokens) < 2:
            print("  Usage: ftp-remove <file>"); return True
        _remove_script(session, tokens[1])

    # -- ftp-upload <file> [--no-restart] -----------------------------------
    elif cmd == "ftp-upload":
        if len(tokens) < 2:
            print("  Usage: ftp-upload <file> [--no-restart]"); return True
        local_path = tokens[1]
        restart    = "--no-restart" not in tokens
        _upload_script(session, local_path, restart=restart)

    # -- release [n] --------------------------------------------------------
    elif cmd == "release":
        try:
            n = int(tokens[1]) if len(tokens) > 1 else SERVO_MOTOR
        except ValueError:
            print("  Usage: release [n]"); return True
        key = f"SERVO{n}_FUNCTION"
        val = session.get_param(key)
        if val is None:
            print(f"  [FAIL] Could not read {key}"); return True
        _saved_servo_functions[n] = val
        if not session.set_param(key, 0):
            print(f"  [FAIL] Could not set {key}=0"); return True
        print(f"  [OK] {key} was {val:.0f}, now 0 -- output {n} released for manual control")
        print(f"       Run 'restore {n}' to give it back to ArduPilot")

    # -- restore [n] --------------------------------------------------------
    elif cmd == "restore":
        try:
            n = int(tokens[1]) if len(tokens) > 1 else SERVO_MOTOR
        except ValueError:
            print("  Usage: restore [n]"); return True
        if n not in _saved_servo_functions:
            print(f"  [WARN] No saved function for output {n} -- use 'restore {n}' after 'release {n}'")
            return True
        val = _saved_servo_functions.pop(n)
        key = f"SERVO{n}_FUNCTION"
        if not session.set_param(key, val):
            print(f"  [FAIL] Could not restore {key}={val:.0f}"); return True
        print(f"  [OK] {key} restored to {val:.0f}")

    # -- ping [baud] --------------------------------------------------------
    elif cmd == "ping":
        try:
            baud = int(tokens[1]) if len(tokens) > 1 else 115200
        except ValueError:
            print("  Usage: ping [baud]"); return True
        _ping_ports(baud=baud)

    else:
        return False

    return True


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
                   help="Serial port (e.g. COM4); omit to auto-detect")
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

def _resolve_port(port: "str | None", baud: int) -> tuple:
    """
    Return (port, baud) to use for the connection.

    If port is None, scans all COM ports and returns the first that gives a
    heartbeat (trying baud then _FALLBACK_BAUDS in order).
    If port is given, probes that port with baud fallbacks until a heartbeat
    is received, then returns the working (port, baud).
    Raises SystemExit if nothing responds.
    """
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
    print(f"Connecting to {port} at {baud} baud ...")
    session = RawesGCS(address=port, baud=baud, clock=WallClock())
    session.connect(timeout=15.0)
    print(f"Connected: sysid={session._target_system} compid={session._target_component}")
    session.start_heartbeat()
    session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 10)
    return session


def main() -> None:
    args = _build_parser().parse_args()

    if args.command == "ping":
        _ping_ports(baud=args.baud)
        return

    session = _connect(args.port, args.baud)
    exit_code = 0
    try:
        if args.command:
            tokens = [args.command] + list(args.args)
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
