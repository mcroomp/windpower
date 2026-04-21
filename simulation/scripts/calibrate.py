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
import math
import os
import sys
import time

# Allow importing gcs.py from the parent directory (simulation/)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR = os.path.abspath(os.path.join(_SCRIPT_DIR, '..'))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)
from gcs import RawesGCS, WallClock  # noqa: E402

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

PWM_MIN     = 1000
PWM_NEUTRAL = 1500
PWM_MAX     = 2000

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




_KEY_PARAMS = [
    ("H_TAIL_TYPE",      4,   "DDFP CCW for GB4008"),
    ("SCR_ENABLE",       1,   "Lua scripting on"),
    ("SCR_USER6",        None, "Lua mode (0=none 1=steady 4=landing 5=pumping)"),
    ("ARMING_CHECK",     0,   "prearm checks disabled"),
    ("BRD_SAFETY_DEFLT", 0,   "safety switch disabled"),
]

_LUA_MODES = {0: "none", 1: "steady", 4: "landing", 5: "pumping"}


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
  mode <0,1,4,5>                  Set SCR_USER6 mode and listen 10 s to confirm active
  arm [seconds]                   Set ACRO mode, send RAWES_ARM (default 10 s)
  hold <pwm> [seconds]            Arm, hold GB4008 output at pwm us (Ctrl-C to stop)
  disarm                          Disarm vehicle
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
        _MODE_NAMES = {
            0: "none", 1: "steady", 4: "landing", 5: "pumping",
        }
        if len(tokens) < 2:
            print("  Usage: mode <0,1,4,5>")
            print("  Modes: " + "  ".join(f"{k}={v}" for k, v in _MODE_NAMES.items()))
            return True
        try:
            n = int(tokens[1])
        except ValueError:
            print("  Error: mode must be an integer 0-8"); return True
        if n not in _MODE_NAMES:
            print(f"  Error: unknown mode {n}  (valid: {sorted(_MODE_NAMES)})"); return True
        if not session.set_param("SCR_USER6", float(n)):
            print("  [FAIL] Could not set SCR_USER6"); return True
        name = _MODE_NAMES[n]
        print(f"  [OK] SCR_USER6={n} ({name}) -- listening 10 s for STATUSTEXT ...")
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
        apply = "--apply" in [t.lower() for t in tokens[1:]]
        # Full RAWES hardware parameter set matching the SITL torque test stack.
        # GB4008 on output 4, PWM 800-2000, DDFP CCW (H_TAIL_TYPE=4).
        # Lua rawes.lua handles arming (RAWES_ARM); ArduPilot DDFP PID drives yaw.
        steps = [
            # DDFP tail: H_TAIL_TYPE=4 (DDFP CCW) drives SERVO4 for yaw.
            ("H_TAIL_TYPE",       4),
            ("H_COL2YAW",         0),
            # SERVO4: GB4008 on output 4.  800 us = off, 2000 us = full throttle.
            ("SERVO4_MIN",        800),
            ("SERVO4_MAX",        2000),
            ("SERVO4_TRIM",       800),
            # Yaw PID matching SITL torque tests.
            ("H_YAW_TRIM",        0.02),
            ("ATC_RAT_YAW_P",     0.015),
            ("ATC_RAT_YAW_I",     0.01),
            ("ATC_RAT_YAW_D",     0.0),
            ("ATC_RAT_YAW_IMAX",  0.7),
            # RSC: CH8 interlock (instant runup when CH8=2000).
            ("H_RSC_MODE",        1),
            ("H_RSC_RUNUP_TIME",  2),
            # Swashplate collective limits.
            ("H_COL_ANG_MIN",     -10),
            ("H_COL_ANG_MAX",     10),
            # Lua scripting: rawes.lua loaded, mode 0 (arming only).
            ("SCR_ENABLE",        1),
            ("SCR_USER6",         0),
            # Arming: disable prearm checks so Lua arming:arm() succeeds.
            ("ARMING_CHECK",      0),
            ("BRD_SAFETY_DEFLT",  0),
        ]

        action = "Applying" if apply else "Preview (read-only -- add --apply to write)"
        print(f"  RAWES config  [{action}]")
        print()
        print(f"  {'Parameter':<25} {'Expected':>8}  {'Actual':>10}  Status")
        print(f"  {'-'*25}  {'-'*8}  {'-'*10}  ------")
        any_diff = False
        any_fail = False
        for name, expected in steps:
            actual = session.get_param(name)
            if actual is None:
                row_status = "[FAIL] not found"
                any_fail = True
                actual_str = "N/A"
            elif abs(actual - float(expected)) < 1e-4:
                row_status = "[OK]"
                actual_str = f"{actual:.4g}"
            else:
                row_status = f"[DIFF] {actual:.4g} -> {expected}"
                any_diff = True
                actual_str = f"{actual:.4g}"
            print(f"  {name:<25} {str(expected):>8}  {actual_str:>10}  {row_status}")

        print()
        if any_fail:
            print("  [FAIL] One or more parameters not found on this firmware.")
        elif not any_diff:
            print("  [OK] All parameters already correct.")
        elif not apply:
            print("  [DIFF] Run 'config --apply' to write changes.")
        else:
            print("  Writing changes ...")
            for name, val in steps:
                session.set_param(name, val)
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
        print()
        print(f"  {'t(s)':<6} {'armed':<6} {'yaw_rate(deg/s)':>16}  {'SERVO4(us)':>10}  STATUSTEXT")
        print(f"  {'-'*6}  {'-'*6}  {'-'*16}  {'-'*10}  ----------")
        t0        = time.monotonic()
        deadline  = t0 + secs + 5.0   # extra 5 s margin past armon expiry
        last_print  = -1.0
        yaw_rate    = None
        servo4_pwm  = None
        armed_state = False
        pending_text: list[str] = []
        try:
            while time.monotonic() < deadline:
                msg = session._recv(
                    type=["ATTITUDE", "SERVO_OUTPUT_RAW", "HEARTBEAT", "STATUSTEXT"],
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
                elapsed = time.monotonic() - t0
                if elapsed - last_print >= 1.0:
                    last_print = elapsed
                    yr_str  = f"{yaw_rate:+.2f}" if yaw_rate  is not None else "   n/a"
                    pw_str  = f"{servo4_pwm}"     if servo4_pwm is not None else "n/a"
                    ar_str  = "YES" if armed_state else "no"
                    txt_str = "  " + pending_text.pop(0) if pending_text else ""
                    print(f"  {elapsed:<6.1f}  {ar_str:<6}  {yr_str:>16}  {pw_str:>10}{txt_str}")
                    # Drain remaining pending texts on separate lines
                    while pending_text:
                        print(f"  {'':<6}  {'':<6}  {'':>16}  {'':>10}  {pending_text.pop(0)}")
        except KeyboardInterrupt:
            print()
        print(f"  Done.")

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
