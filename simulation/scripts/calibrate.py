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
  Output 4  GB4008 anti-rotation motor                H_TAIL_TYPE = 3 (DDFP CW)

  See CLAUDE.md "Swashplate geometry" for the canonical azimuth table.

SERVO4 PWM range: 800 us (off) ... 2000 us (full throttle)
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
    config show / config apply        Bulk parameter sync vs rawes_params.json
    ping                              COM port discovery

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




_PARAMS_JSON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rawes_params.json")

def _load_params_json() -> dict:
    with open(_PARAMS_JSON_PATH) as fh:
        return json.load(fh)

def _params_as_tuples(entries: list) -> list:
    return [(e["name"], e["expected"], e["note"]) for e in entries]

_PARAMS_JSON   = _load_params_json()
_KEY_PARAMS    = _params_as_tuples(_PARAMS_JSON["key"])
_TAIL_PARAMS   = _params_as_tuples(_PARAMS_JSON["tail_pid"])

_LUA_MODES = {0: "none", 1: "steady", 2: "yaw", 3: "passive", 4: "landing", 5: "pumping"}


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
        for i in range(1, 9):
            val = getattr(srv, f"servo{i}_raw", 0)
            if not val:
                continue
            tag = {SERVO_S1: "  <- S1 (-60 deg, front-right)",
                   SERVO_S2: "  <- S2 (+60 deg, front-left)",
                   SERVO_S3: "  <- S3 (180 deg, back)"}.get(i, "")
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
          passive   armed but quiet -- pins ch1/ch2/ch3 at IC trim, SERVO4
                    at 800 us.  Sets H_FLYBAR_MODE=1 (RC passthrough),
                    H_CYC_MAX=1000 (10 deg cap), H_SV_MAN=0.
          yaw       manual Lua yaw PID (SERVO4 direct).  Accepts --gain for
                    full yaw PID + filters + servo limits.
          steady    steady flight (alt hold + VZ PI collective)
          pumping   De Schutter pumping cycle
          landing   landing (reserved)

        Examples (all trim values in DEGREES):
          # Hold IC cyclic + collective on the bench for 30 s
          run passive --duration 30 --trim tlon=1.15,col=-8.6

          # Pure lateral cyclic: 2.9 deg roll-right, collective neutral
          run passive --duration 20 --trim tlat=2.9,col=0

          # Pure longitudinal: 1.7 deg nose-down for 5 s
          run passive --duration 5 --trim tlon=1.7

          # Yaw tuning: gentler P, lower motor max, IC operating point loaded
          run yaw --duration 60 --gain p=0.015,i=0.005,imax=0.7,servo_max=1100 \\
                  --trim tlon=1.15,col=-8.6

          # Unbounded passive session (ESC to stop, 5-min RAWES_ARM fallback)
          run passive --trim tlon=1.15,tlat=-0.6,col=-8.6

          # Full oscillation sweep through every axis extreme (~65 s)
          run passive --osc all

          # Isolated S2 swashplate-servo test (~25 s, S2 dominant up/down)
          run passive --osc s2

  analyze yaw <csv> [--include-saturate]
        Offline PID-tuning report on a saved run_yaw_*.csv (no FC connection
        required).  Default: filters out samples where the PID output or
        integrator was saturated (the closed loop wasn't operating).  Pass
        --include-saturate to score all samples together.

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
  motor <pwm_us> [--duration N]   Arm (RAWES_ARM) + ESC pre-arm (5 s @ SERVO4_MIN) +
                                  hold SERVO4 at pwm_us for N s (default 5).
                                  pwm_us must be within [SERVO4_MIN, SERVO4_MAX];
                                  >5% of that range prompts unless --force.
                                  Logs to CSV like `run`.
  motor off                       SERVO4 -> 800 us + disarm immediately
  arm [--duration N]              ACRO + RAWES_ARM (no Lua mode change)
  disarm                          Disarm vehicle
  script upload <file>            Upload .lua to /APM/scripts and restart engine
  script list                     List /APM/scripts
  script remove <name>            Remove from /APM/scripts
  config show                     Diff all params from rawes_params.json
  config apply                    Write the DIFFs
  reboot                          Reboot ArduPilot
  ping [baud]                     Scan COM ports for ArduPilot heartbeats
  help                            Show this list
  quit                            Exit (REPL only)
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

def _rawes_arm_ms(duration_s: "float | None") -> int:
    """ARM the Lua for duration+10s, or 5 min if unbounded."""
    return int((duration_s + 10.0) * 1000) if duration_s else 300_000


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
    """Set SERVO4_FUNCTION=0 (release from DDFP).  Returns saved value for restore."""
    servo_key = f"SERVO{SERVO_MOTOR}_FUNCTION"
    saved = session.get_param(servo_key)
    if saved is not None and saved != 0:
        session.set_param(servo_key, 0)
        print(f"  {servo_key} {saved:.0f} -> 0 (released from DDFP)")
        return float(saved)
    return None


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
            _send_set_servo(session, SERVO_MOTOR, 800)
            print(f"  [SAFETY] SERVO{SERVO_MOTOR} -> 800 us (motor off)")
        except Exception as e:
            print(f"  [SAFETY] failed to drive SERVO{SERVO_MOTOR} off: {e}")
    try:
        if not _disarm(session, timeout=5.0):
            print("  [SAFETY] disarm not confirmed within 5 s")
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
            if _esc_check():
                aborted = True
                print("\n  [ESC] abort -- running safety shutdown ...")
                break
            msg = session._recv(type=msg_types, blocking=True, timeout=0.1)
            t_rel = time.monotonic() - t0
            if on_tick is not None:
                on_tick(t_rel)
            if msg is not None:
                if msg.get_type() == "HEARTBEAT":
                    state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif msg.get_type() == "STATUSTEXT":
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

_RUN_MODES = {
    "passive": {
        "scr_user6":  3,
        "take_servo4": True,
        "gain_keys":  {},
        "force_params": {
            # Flybar passthrough: ACRO sends RC1/RC2 stick deflection straight to
            # the swash (passthrough_bf_roll_pitch_rate_yaw) instead of through
            # the rate PID -- trim becomes a direct cyclic command.
            "H_FLYBAR_MODE": 1,
            # Cap cyclic at 10 deg of swash tilt at full stick.  Bench-safe
            # bound (default 25 deg is too aggressive when servo travel is
            # mechanically limited).  Trim_rad_to_pwm scales relative to this
            # value, so the SAME trim NVF maps to a known PWM offset.
            "H_CYC_MAX": 1000,
            # Disable manual servo override -- we want the live mixer driving
            # the swash, not a fixed pose.  H_SV_MAN > 0 is for setup only and
            # must be 0 during any actual run.
            "H_SV_MAN": 0,
        },
        "doc":        "armed but quiet: pins ch1/ch2/ch3 at IC trim, pins SERVO4 at 800 us, H_FLYBAR_MODE=1, H_CYC_MAX=1000, H_SV_MAN=0",
    },
    "yaw": {
        "scr_user6":  2,
        "take_servo4": True,
        "force_params": {
            # Same passthrough requirement as passive: trim cyclic is set via
            # direct RC override in the Lua, not via rate-PID bias.
            "H_FLYBAR_MODE": 1,
        },
        "gain_keys": {
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
        },
        "doc":        "manual Lua yaw PID (drives SERVO4 directly)",
    },
    "steady": {
        "scr_user6":  1,
        "take_servo4": False,
        "gain_keys":  {},
        "doc":        "steady flight: altitude hold + VZ PI collective",
    },
    "pumping": {
        "scr_user6":  5,
        "take_servo4": False,
        "gain_keys":  {},
        "doc":        "De Schutter pumping cycle",
    },
    "landing": {
        "scr_user6":  4,
        "take_servo4": False,
        "gain_keys":  {},
        "doc":        "landing (reserved)",
    },
}


def _cmd_run(session: RawesGCS, args: list[str]) -> None:
    """run <mode> [--duration N] [--trim K=V,...] [--gain K=V,...] [--osc TARGET]"""
    schema = {
        "--duration":         "float",
        "--trim":             "kv",
        "--gain":             "kv",
        "--osc":              "str",
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

    # SERVO4 ownership shuffle if the mode needs it
    saved_fn = _take_servo4(session) if cfg["take_servo4"] else None

    # Activate Lua mode
    session.set_param("SCR_USER6", cfg["scr_user6"])
    print(f"  SCR_USER6 -> {cfg['scr_user6']} ({name} mode)")

    # Send NVF trims.  --trim values are user-facing DEGREES; convert to
    # radians for the wire (rawes.lua receives RAWES_TLN/TLT/COL in radians).
    if trim:
        print("  Sending trim NVFs (deg -> rad on the wire):")
        for k, v_deg in trim.items():
            v_rad = math.radians(float(v_deg))
            session.send_named_float(_TRIM_NVF[k], v_rad)
            print(f"    {_TRIM_NVF[k]} = {v_deg:+7.3f} deg  ({v_rad:+.4f} rad)")

    # Arm via RAWES_ARM
    arm_ms = _rawes_arm_ms(duration)
    print(f"  Sending RAWES_ARM={arm_ms} ms ...")
    session.send_named_float("RAWES_ARM", float(arm_ms))
    if not _wait_for_armed(session, timeout_s=15.0):
        print("  [FAIL] Did not arm within 15 s.")
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

    # Build the oscillate tick callback if --osc was set
    on_tick = None
    if osc_steps is not None:
        on_tick = _make_oscillate_tick(session, osc_steps)
        total_s = len(osc_steps) * _OSCILLATE_STEP_S
        print(f"  Oscillate target={osc!r}: walking {len(osc_steps)} steps "
              f"x {_OSCILLATE_STEP_S:.0f}s each (total {total_s:.0f}s).")

    try:
        _run_observation(session, name, duration, log, on_tick=on_tick)
    finally:
        log.close()
        print(f"  Wrote {log.n_rows} rows to {log.path}")
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
                     on_tick=None) -> None:
    """Generic observation loop for `run` modes.  Stream + columns are the
    same across all modes; the row content is whatever the FC reports.  Per-
    mode NVFs (e.g. YAW_I, YAW_OUT) appear as columns when emitted.

    on_tick(t_rel) is called once per loop iteration (~10 Hz); used by
    oscillate mode to advance the trim sequence."""
    cols = ["t_s", "armed", "roll_deg", "pitch_deg", "yaw_deg",
            "ch1_us", "ch2_us", "ch3_us", "ch4_us",
            "s1_us", "s2_us", "s3_us", "s4_us",
            "vbat_v", "current_a", "yaw_i_lua", "yaw_out_lua"]

    # Live table shows the actual servo PWMs (S1/S2/S3 = swash mixer output,
    # S4 = GB4008 motor).  The full CSV also has ch1..ch4 (the RC overrides
    # the Lua wrote) for diagnosing the mixer.
    print_cols = ["t(s)", "armed", "yaw(d)", "s1", "s2", "s3", "s4", "V", "A"]

    state = {
        "roll": None, "pitch": None, "yaw": None,
        "ch1": None, "ch2": None, "ch3": None, "ch4": None,
        "s1": None, "s2": None, "s3": None, "s4": None,
        "vbat": None, "curr": None,
        "yaw_i": None, "yaw_out": None,
    }

    def handle_msg(st, msg, t_rel):
        mt = msg.get_type()
        if mt == "ATTITUDE":
            state["roll"]  = math.degrees(msg.roll)
            state["pitch"] = math.degrees(msg.pitch)
            state["yaw"]   = math.degrees(msg.yaw)
            # Emit one CSV row per ATTITUDE message (typically 10-50 Hz)
            return [
                f"{t_rel:.4f}", int(st["armed"]),
                _fmt(state["roll"]), _fmt(state["pitch"]), _fmt(state["yaw"]),
                state["ch1"], state["ch2"], state["ch3"], state["ch4"],
                state["s1"], state["s2"], state["s3"], state["s4"],
                _fmt(state["vbat"]), _fmt(state["curr"]),
                _fmt(state["yaw_i"]), _fmt(state["yaw_out"]),
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
            state["s4"] = getattr(msg, "servo4_raw", None)
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
        elif mt == "NAMED_VALUE_FLOAT":
            nm = msg.name.rstrip("\x00").strip() if isinstance(msg.name, str) else \
                 msg.name.decode("ascii", errors="replace").rstrip("\x00").strip()
            if nm == "YAW_I":
                state["yaw_i"] = float(msg.value)
            elif nm == "YAW_OUT":
                state["yaw_out"] = float(msg.value)
        return None

    def render_row(st, t_rel):
        yaw_s = f"{state['yaw']:+6.1f}" if state["yaw"] is not None else None
        return [
            f"{t_rel:.1f}",
            "YES" if st["armed"] else "no",
            yaw_s,
            state["s1"], state["s2"], state["s3"], state["s4"],
            f"{state['vbat']:.2f}" if state["vbat"] is not None else None,
            f"{state['curr']:.2f}" if state["curr"] is not None else None,
        ]

    _observation_loop(
        session,
        duration_s=duration,
        msg_types=["ATTITUDE", "RC_CHANNELS", "SERVO_OUTPUT_RAW",
                   "HEARTBEAT", "STATUSTEXT", "BATTERY_STATUS", "SYS_STATUS",
                   "NAMED_VALUE_FLOAT"],
        streams=[
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,          25),
            (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,     25),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5),
        ],
        handle_msg=handle_msg,
        render_row=render_row,
        header_cols=cols,
        header_print_cols=print_cols,
        log=log,
        on_tick=on_tick,
    )


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
    servo_min = _meta_float(meta, "SERVO4_MIN",       800.0)
    servo_max = _meta_float(meta, "SERVO4_MAX",       2000.0)

    print(f"  source ........ {csv_path}")
    print(f"  gains ......... P={kp:.4g}  I={ki:.4g}  D={kd:.4g}"
          f"  IMAX={imax:.3g}  TRIM={trim:.3g}")
    print(f"  SERVO4 range .. {servo_min:.0f} .. {servo_max:.0f} us")

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
    s4_list:   list[float]  = []
    t_list:    list[float]  = []
    for r in eval_rows:
        o = float(r["yaw_out_lua"])
        i = float(r["yaw_i_lua"])
        err = (o - i - trim) / kp        # rad/s; err = -yaw_rate
        err_list.append(err)
        p_out_list.append(kp * err)
        i_out_list.append(i)
        out_list.append(o)
        s4 = r.get("s4_us")
        if isinstance(s4, (int, float)):
            s4_list.append(float(s4))
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
    if s4_list:
        print(f"    mean s4_us .. {_mean(s4_list):8.1f}  (PWM, "
              f"{min(s4_list):.0f} .. {max(s4_list):.0f})")

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

        if s4_list:
            margin_up = servo_max - max(s4_list)
            mean_pwm  = _mean(s4_list)
            if margin_up < 20 and pct_sat < 50.0:
                advice.append(
                    f"PWM headroom thin at the top (max {max(s4_list):.0f} "
                    f"vs {servo_max:.0f}) -- raise SERVO4_MAX if the ESC "
                    "accepts it.")
            if mean_pwm < servo_min + 0.1 * (servo_max - servo_min):
                advice.append(
                    f"Mean PWM ({mean_pwm:.0f}) sits near SERVO4_MIN -- "
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


def _cmd_analyze(args: list[str]) -> None:
    """analyze yaw <csv> [--include-saturate]
       Offline PID-tuning report on a saved run_yaw_*.csv."""
    schema = {"--include-saturate": "bool"}
    if not args:
        print("  Usage: analyze yaw <csv>  [--include-saturate]")
        return
    if args[0].lower() != "yaw":
        print(f"  Unknown analyze target {args[0]!r}  (valid: yaw)")
        return
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
    # ESC_TELEMETRY is broadcast as ESC_TELEMETRY_5_TO_8 / _1_TO_4
    cols = ["t_s", "rpm", "voltage", "current", "temp"]

    def handle(st, msg, t_rel):
        mt = msg.get_type()
        if mt in ("ESC_TELEMETRY_1_TO_4", "ESC_TELEMETRY_5_TO_8"):
            # Take index 3 = SERVO4 (the GB4008) when present
            rpm = msg.rpm[3]      if hasattr(msg, "rpm") else None
            volt = msg.voltage[3] if hasattr(msg, "voltage") else None
            curr = msg.current[3] if hasattr(msg, "current") else None
            temp = msg.temperature[3] if hasattr(msg, "temperature") else None
            return [f"{t_rel:.4f}", rpm, volt, curr, temp]
        return None

    def render(st, t_rel):
        return [f"{t_rel:.1f}", "?", "?", "?", "?"]

    _observation_loop(
        session, duration_s=duration,
        msg_types=["ESC_TELEMETRY_1_TO_4", "ESC_TELEMETRY_5_TO_8",
                   "HEARTBEAT", "STATUSTEXT"],
        streams=[(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)],
        handle_msg=handle, render_row=render,
        header_cols=cols,
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
        s4 = getattr(srv, "servo4_raw", 0)
        print(f"Current PWMs:  S1={s1} us  S2={s2} us  S3={s3} us  S4={s4} us")
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
        arm_ms = _rawes_arm_ms(duration)
        print(f"  Sending RAWES_ARM={arm_ms} ms ...")
        session.send_named_float("RAWES_ARM", float(arm_ms))
        if not _wait_for_armed(session, timeout_s=15.0):
            print("  [FAIL] Did not arm within 15 s.")
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

    Run-style lifecycle: arms via RAWES_ARM, releases SERVO4 from any AP
    mixer, holds SERVO4_MIN for ESC arming, then drives SERVO4 at the
    requested PWM for `duration` seconds while logging telemetry to
    simulation/logs/calibrate/motor_<pwm>_<ts>.csv.  On exit (timer / ESC /
    Ctrl-C / exception): SERVO4 -> 800, disarm, SERVO4_FUNCTION restored."""
    if not args:
        print("  Usage: motor <pwm_us> [--duration N]  OR  motor off"); return
    if args[0].lower() in ("off", "stop"):
        # Immediate stop: force SERVO4 to 800 and disarm.
        try:
            _send_set_servo(session, SERVO_MOTOR, 800)
            print(f"  SERVO{SERVO_MOTOR} -> 800 us (motor off)")
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

    # Read the live SERVO4_MIN / SERVO4_MAX caps so the prompt + clamp warning
    # reflect the per-bench safety cap (e.g. SERVO4_MAX=1100 during early tuning).
    s4_min = int(session.get_param("SERVO4_MIN") or 800)
    s4_max = int(session.get_param("SERVO4_MAX") or 2000)
    if not (s4_min <= pwm <= s4_max):
        print(f"  Error: pwm must be in [{s4_min}, {s4_max}]  (SERVO4_MIN/MAX); got {pwm}")
        return
    # Express PWM as fraction of the configured range for the safety prompt.
    if s4_max > s4_min:
        pct_for_prompt = (pwm - s4_min) / (s4_max - s4_min) * 100.0
    else:
        pct_for_prompt = 0.0
    if not force and pct_for_prompt > 5.0:
        confirm = input(
            f"  WARNING: SERVO4 = {pwm} us "
            f"({pct_for_prompt:.0f}% of [{s4_min},{s4_max}]) for {secs:.0f}s. "
            f"Confirm (y/N): ")
        if confirm.strip().lower() != "y":
            print("  Cancelled."); return
    arm_hold_s = 5.0   # ESC arming: hold SERVO4 at min for this long after arm

    # Same shuffle as `run yaw`: release SERVO4 from any AP mixer so our
    # DO_SET_SERVO commands win.
    saved_fn = _take_servo4(session)

    # MODE_PASSIVE / MODE_YAW would also drive SERVO4 -- force Lua to NONE.
    saved_overrides: dict[str, float] = {}
    saved_scr = session.get_param("SCR_USER6")
    if saved_scr is not None and int(saved_scr) != 0:
        saved_overrides["SCR_USER6"] = float(saved_scr)
        session.set_param("SCR_USER6", 0)
        print(f"  SCR_USER6 {int(saved_scr)} -> 0 (motor needs direct SERVO4 control)")

    # Arm via Lua RAWES_ARM, same as `run`.  Total arm time covers ESC
    # arming hold + driven duration + safety margin.
    total_arm_s = arm_hold_s + secs
    arm_ms = _rawes_arm_ms(total_arm_s)
    print(f"  Sending RAWES_ARM={arm_ms} ms ...")
    session.send_named_float("RAWES_ARM", float(arm_ms))
    if not _wait_for_armed(session, timeout_s=15.0):
        print("  [FAIL] Did not arm within 15 s.")
        _safety_shutdown(session, saved_servo4_fn=saved_fn,
                         saved_overrides=saved_overrides)
        return
    print("  [OK] Armed.")

    # ESC arming sequence: hold SERVO4 at min PWM (=ESC "off") long enough
    # for the REVVitRC ESC to detect the signal and arm.
    print(f"  ESC arm: SERVO{SERVO_MOTOR} = {s4_min} us for {arm_hold_s:.0f} s ...")
    t_arm_end = time.monotonic() + arm_hold_s
    while time.monotonic() < t_arm_end:
        _send_set_servo(session, SERVO_MOTOR, s4_min)
        time.sleep(0.2)
    print(f"  Motor: SERVO4 = {pwm} us for {secs:.1f}s "
          f"(SERVO4 range [{s4_min}, {s4_max}]).")

    # Snapshot params for the log header
    meta = {
        "verb":             "motor",
        "pwm_us":           pwm,
        "duration_s":       secs,
        "arm_hold_s":       arm_hold_s,
        "SERVO4_MIN":       s4_min,
        "SERVO4_MAX":       s4_max,
        "run_start_local":  datetime.now().isoformat(timespec="seconds"),
        "run_start_utc":    datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    log = _RunLog.open("motor", f"{pwm}us", meta)
    print(f"  Logging to {log.path}")

    # On_tick refreshes the SERVO4 PWM ~twice per second while t_rel < secs.
    # MAV_CMD_DO_SET_SERVO persists for 30 s on AP, so once we cross the
    # duration boundary we must explicitly drive it back to 800 us -- the
    # loop continues observing for the standard +5 s post-window so we get
    # spin-down telemetry before safety_shutdown disarms.
    last_send = [-10.0]
    stopped   = [False]
    def on_tick(t_rel: float) -> None:
        target = pwm if t_rel < secs else 800
        if t_rel >= secs and not stopped[0]:
            _send_set_servo(session, SERVO_MOTOR, 800)
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
    """arm [--duration N]   -- ACRO + RAWES_ARM (no Lua mode change)"""
    try:
        pos, flags = _parse_flags(args, {"--duration": "float"})
    except ValueError as e:
        print(f"  Error: {e}"); return
    if pos:
        print("  Usage: arm [--duration N]  (use --duration, not positional)"); return
    secs = flags.get("--duration", 10.0)
    print("  Setting ACRO mode ...")
    session.set_mode(1)
    armon_ms = int(secs * 1000)
    print(f"  Sending RAWES_ARM={armon_ms} ms ...")
    session.send_named_float("RAWES_ARM", float(armon_ms))
    if not _wait_for_armed(session, timeout_s=secs + 5.0):
        print(f"  [WARN] Not armed within {secs+5:.0f}s.")
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
    """config show
       config apply"""
    if not args:
        print("  Usage: config show  OR  config apply"); return
    sub = args[0].lower()
    if sub not in ("show", "apply"):
        print(f"  Unknown config subcommand {sub!r}  (valid: show, apply)"); return
    apply = (sub == "apply")
    all_entries = []
    for group in _PARAMS_JSON.values():
        for e in group:
            if e["expected"] is not None:
                all_entries.append(e)
    action = "Applying" if apply else "Preview -- 'config apply' to write"
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
        print("  Done -- run 'config apply' to write the DIFFs.")
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

    if args.command == "analyze":
        # Offline CSV analysis; no FC connection required.
        _cmd_analyze(list(args.args))
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
