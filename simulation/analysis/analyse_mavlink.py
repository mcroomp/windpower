#!/usr/bin/env python3
"""
analyse_mavlink.py -- Structured summary of a test run's mavlink.jsonl log.

Reads simulation/logs/<test_name>/mavlink.jsonl and prints a report covering:

  * Message inventory (type counts, total rows, time span)
  * Chronological event timeline (STATUSTEXT, EKF transitions, GPS events,
    ArduPilot SIGFPE crashes timestamped from tumble onset in ATTITUDE data)
  * GPS acquisition: fix_type progression, first fix, GPS_GLOBAL_ORIGIN
  * EKF flag analysis: transitions with correct ArduPilot flag names,
    const_pos_mode and gps_glitching detection
  * GPS position vs SIMSTATE position (detects position mismatch)
  * Attitude rate anomalies (tumble / spin detection)
  * EKF variance peaks

Crash timestamp estimation (arducopter.log):
  arducopter.log has no timestamps.  The crash time is estimated by finding
  the first ATTITUDE message where max(|rollspeed|, |pitchspeed|) > 200 deg/s
  (tumble onset).  The SIGFPE in EKFGSF_yaw::updateRotMat fires within a few
  EKF cycles of receiving the extreme sensor data that caused it.

This is the first tool to run after a test failure -- it gives a faster
picture of what ArduPilot saw than reading gcs.log manually.

Usage:
  # Windows (from repo root):
  simulation/.venv/Scripts/python.exe simulation/analysis/analyse_mavlink.py test_lua_flight_steady

  # Inside container:
  python3 /rawes/simulation/analysis/analyse_mavlink.py test_lua_flight_steady

  # Verbose: print every EKF_STATUS_REPORT (not just transitions)
  simulation/.venv/Scripts/python.exe simulation/analysis/analyse_mavlink.py test_lua_flight_steady --ekf-all

  # Suppress sections you don't need:
  simulation/.venv/Scripts/python.exe simulation/analysis/analyse_mavlink.py test_lua_flight_steady --no-gps-detail

Output sections (all enabled by default):
  INVENTORY        message type counts + time span
  TIMELINE         chronological key events (STATUSTEXT, EKF, GPS origin)
  GPS ACQUISITION  GPS_RAW_INT fix_type progression + position quality
  EKF FLAGS        transitions with correct flag names; const_pos/glitch warnings
  GPS vs SIMSTATE  lat/lon/alt from GPS_RAW_INT vs SIMSTATE ground truth
  GPS CONSISTENCY  vel/cog vs position-derived velocity; sAcc; acceleration; GSF observability
  ATTITUDE RATES   peak roll/pitch/yaw rates; tumble detection
  EKF VARIANCES    pos_horiz, pos_vert, vel, compass variance peaks

EKF flag bit definitions (EKF_STATUS_FLAGS MAVLink enum, from ardupilotmega.h):
  Source: AP_NavEKF3/AP_NavEKF3_Outputs.cpp  mavlink_msg_ekf_status_report_send_struct()
  These are NOT a direct copy of nav_filter_status bits -- they go through a named-constant mapping.

  0x0001  attitude           EKF_ATTITUDE          -- attitude estimate valid
  0x0002  horiz_vel          EKF_VELOCITY_HORIZ    -- horizontal velocity valid
  0x0004  vert_vel           EKF_VELOCITY_VERT     -- vertical velocity valid
  0x0008  horiz_pos_rel      EKF_POS_HORIZ_REL     -- relative horizontal position valid
  0x0010  horiz_pos_abs      EKF_POS_HORIZ_ABS     -- absolute horizontal position valid
  0x0020  vert_pos           EKF_POS_VERT_ABS      -- vertical position valid
  0x0040  terrain_alt        EKF_POS_VERT_AGL      -- terrain height valid
  0x0080  const_pos_mode     EKF_CONST_POS_MODE    <- EKF in constant-position fallback (no GPS)
  0x0100  pred_horiz_pos_rel EKF_PRED_POS_HORIZ_REL
  0x0200  pred_horiz_pos_abs EKF_PRED_POS_HORIZ_ABS
  0x0400  uninitialized      EKF_UNINITIALIZED     <- EKF has NEVER been healthy (!initalized)
  0x8000  gps_glitching      (1<<15) hardcoded     <- GPS glitch active (nav_filter_status.gps_glitching)

  NOTE: 0x0800 is NOT used -- no internal flag maps to it in the send code.
  NOTE: nav_filter_status bits 10-18 (takeoff_detected, using_gps, gps_quality_good, etc.)
        are NOT sent in EKF_STATUS_REPORT -- only gps_glitching gets a special (1<<15) slot.

HEARTBEAT base_mode flags (bitmask):
  0x80  ARMED              <- vehicle is armed
  0x40  MANUAL_INPUT
  0x10  STABILIZE_ENABLED
  0x08  GUIDED_ENABLED
  0x04  AUTO_ENABLED
  0x01  CUSTOM_MODE_ENABLED  <- custom_mode field is valid

ArduCopter custom_mode values (flight mode):
  0  STABILIZE   1  ACRO   2  ALT_HOLD   3  AUTO
  4  GUIDED      5  LOITER 6  RTL        9  LAND
  16 POSHOLD
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_SIM_DIR  = Path(__file__).resolve().parents[1]
_LOGS_DIR = _SIM_DIR / "logs"

import sys as _sys
if str(_SIM_DIR) not in _sys.path:
    _sys.path.insert(0, str(_SIM_DIR))

from ekf_flags import (
    EKF_FLAGS   as _EKF_FLAGS,
    EKF_WARN    as _EKF_WARN,
    GPS_FIX     as _GPS_FIX,
    MAV_MODE_ARMED  as _MAV_MODE_ARMED,
    ARDU_MODES  as _ARDU_MODES,
    MAV_STATE   as _MAV_STATE,
    decode_flags as _decode_flags,
    flag_diff    as _flag_diff,
    has_warn     as _has_warn,
)

# ---------------------------------------------------------------------------
# Load + index
# ---------------------------------------------------------------------------

def _load(path: Path) -> list[dict]:
    records = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return records

def _build_time_map(records: list[dict]) -> tuple[float | None, float | None]:
    """Return (t0_wall, t0_boot_s) for the first SYSTEM_TIME packet."""
    for d in records:
        if d["mavpackettype"] == "SYSTEM_TIME":
            return d["_t_wall"], d.get("time_boot_ms", 0) / 1000.0
    return None, None

def to_sim(t_wall: float, t0_wall: float, t0_boot: float) -> float:
    return (t_wall - t0_wall) + t0_boot


def _read_crash_info(log_dir: Path) -> list[dict]:
    """Parse arducopter.log for ArduPilot SIGFPE crashes.

    arducopter.log has no timestamps.  Each 'Floating point exception' line
    marks one crash.  We scan the following stack dump for the first EKFGSF_yaw
    or NavEKF3 frame to identify the crash location.

    Returns a list of dicts: [{"function": "EKFGSF_yaw::updateRotMat"}, ...]
    """
    log_path = log_dir / "arducopter.log"
    if not log_path.exists():
        return []
    try:
        lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return []

    crashes: list[dict] = []
    i = 0
    while i < len(lines):
        if "Floating point exception" in lines[i]:
            function = "unknown"
            # Scan forward in the stack dump (up to 300 lines) for the crash frame.
            # The first EKFGSF_yaw:: frame is the innermost (closest to the crash).
            for j in range(i + 1, min(i + 300, len(lines))):
                l = lines[j]
                for marker in ("EKFGSF_yaw::", "NavEKF3_core::", "NavEKF3::"):
                    if marker in l:
                        idx = l.find(marker)
                        end = idx + len(marker)
                        while end < len(l) and (l[end].isalnum() or l[end] == "_"):
                            end += 1
                        function = l[idx:end]
                        break
                if function != "unknown":
                    break
            crashes.append({"function": function})
        i += 1
    return crashes


def _find_tumble_onset(records: list[dict], t0_wall: float, t0_boot: float,
                       threshold_deg_s: float = 200.0) -> "float | None":
    """Return first sim time where an angular rate exceeds threshold.

    Used to timestamp ArduPilot crashes whose arducopter.log entry has no
    timestamp.  The SIGFPE in EKFGSF_yaw fires within a few EKF cycles of
    receiving the extreme sensor data that triggers it.

    Default threshold 200 deg/s is well above the ~5 deg/s orbital rate but
    below the 1000-1500 deg/s rates seen during full tumble.
    """
    for d in records:
        if d["mavpackettype"] != "ATTITUDE":
            continue
        st = to_sim(d["_t_wall"], t0_wall, t0_boot)
        rs = abs(math.degrees(d.get("rollspeed",  0.0)))
        ps = abs(math.degrees(d.get("pitchspeed", 0.0)))
        if max(rs, ps) > threshold_deg_s:
            return st
    return None


# ---------------------------------------------------------------------------
# Sections
# ---------------------------------------------------------------------------

def print_flight_state(records: list[dict], t0_wall: float, t0_boot: float) -> None:
    """HEARTBEAT transitions: arm state, flight mode, system status."""
    _section("FLIGHT STATE  (from HEARTBEAT)")

    prev_key: tuple | None = None
    t_armed: float | None  = None
    t_acro:  float | None  = None

    for d in records:
        if d["mavpackettype"] != "HEARTBEAT":
            continue
        st      = to_sim(d["_t_wall"], t0_wall, t0_boot)
        armed   = bool(d["base_mode"] & _MAV_MODE_ARMED)
        mode_n  = d.get("custom_mode", 0)
        mode_s  = _ARDU_MODES.get(mode_n, str(mode_n))
        sys_n   = d.get("system_status", 0)
        sys_s   = _MAV_STATE.get(sys_n, str(sys_n))
        key     = (armed, mode_n, sys_n)

        if key != prev_key:
            arm_tag  = "[ARMED]  " if armed  else "[disarmed]"
            mode_tag = f"mode={mode_n}({mode_s:<10})"
            sys_tag  = f"sys={sys_s}"
            warn = ""
            if armed and t_armed is None:
                t_armed = st
                warn = "  <- ARM event"
            if mode_n == 1 and t_acro is None and armed:
                t_acro = st
                warn = "  <- ACRO + ARMED"
            print(f"  t={st:7.2f}s  {arm_tag}  {mode_tag}  {sys_tag}  base_mode=0x{d['base_mode']:02x}{warn}")
            prev_key = key

    print()
    print(f"  First arm  : {'t='+str(round(t_armed,2))+'s' if t_armed is not None else '[!!] never armed'}")
    print(f"  First ACRO : {'t='+str(round(t_acro,2))+'s'  if t_acro  is not None else '[!!] never reached ACRO while armed'}")


def _section(title: str) -> None:
    print()
    print("=" * 72)
    print(f"  {title}")
    print("=" * 72)

def _ok(flag: bool, ok_text: str = "[OK]", fail_text: str = "[!!]") -> str:
    return ok_text if flag else fail_text

# ---------------------------------------------------------------------------

def print_inventory(records: list[dict], t0_wall: float, t0_boot: float) -> None:
    _section("INVENTORY")
    counts: dict[str, int] = {}
    for d in records:
        t = d["mavpackettype"]
        counts[t] = counts.get(t, 0) + 1

    t_first = to_sim(records[0]["_t_wall"],  t0_wall, t0_boot) if records else 0
    t_last  = to_sim(records[-1]["_t_wall"], t0_wall, t0_boot) if records else 0

    print(f"  Total records : {len(records)}")
    print(f"  Sim time span : {t_first:.1f}s -- {t_last:.1f}s  ({t_last-t_first:.1f}s)")
    print()
    print(f"  {'Message type':<36}  {'Count':>6}")
    print(f"  {'-'*36}  {'-'*6}")
    for k, v in sorted(counts.items(), key=lambda x: -x[1]):
        print(f"  {k:<36}  {v:>6}")

# ---------------------------------------------------------------------------

def print_timeline(records: list[dict], t0_wall: float, t0_boot: float,
                   crashes: "list[dict] | None" = None,
                   crash_t: "float | None" = None) -> None:
    """Key events in chronological order.

    If crashes and crash_t are provided (from _read_crash_info / _find_tumble_onset),
    the crash is injected at the correct chronological position rather than shown
    at t=0 (arducopter.log has no timestamps; crash_t is estimated from tumble onset).
    """
    _section("EVENT TIMELINE")

    crashes = crashes or []
    crash_inserted = False

    prev_ekf = 0
    for d in records:
        st = to_sim(d["_t_wall"], t0_wall, t0_boot)
        mt = d["mavpackettype"]

        # Inject crash at chronological position (tumble onset estimate)
        if crash_t is not None and not crash_inserted and st >= crash_t:
            for crash in crashes:
                fn = crash.get("function", "unknown")
                print(f"  t={crash_t:7.2f}s  CRASH    [!!] ArduPilot SIGFPE: {fn}")
            print(f"             (timestamp estimated: first attitude rate > 200 deg/s;"
                  f"  see arducopter.log for stack)")
            crash_inserted = True

        if mt == "STATUSTEXT":
            txt = d.get("text", "").rstrip("\x00").strip()
            sev = d.get("severity", 6)
            marker = "[!!]" if sev <= 3 else "[WN]" if sev <= 4 else "    "
            print(f"  t={st:7.2f}s  STATUSTEXT  {marker} {txt}")

        elif mt == "GPS_GLOBAL_ORIGIN":
            print(f"  t={st:7.2f}s  GPS_GLOBAL_ORIGIN  lat={d['latitude']/1e7:.6f}  lon={d['longitude']/1e7:.6f}  time_usec={d['time_usec']}")

        elif mt == "EKF_STATUS_REPORT":
            flags = d.get("flags", 0)
            if flags != prev_ekf:
                diff  = _flag_diff(prev_ekf, flags)
                warn  = " [!!]" if _has_warn(flags) else ""
                print(f"  t={st:7.2f}s  EKF_FLAGS   0x{flags:04x}  {diff}{warn}")
                prev_ekf = flags

        elif mt == "GPS_RAW_INT":
            pass  # GPS fix timeline handled in print_gps()

    # Crash after all records (e.g. test ended before tumble was logged)
    if crash_t is not None and not crash_inserted and crashes:
        for crash in crashes:
            fn = crash.get("function", "unknown")
            print(f"  t={crash_t:7.2f}s  CRASH    [!!] ArduPilot SIGFPE: {fn}")
        print(f"             (timestamp estimated: first attitude rate > 200 deg/s;"
              f"  see arducopter.log for stack)")

# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# GPS quality check thresholds (EKF3 defaults, EK3_CHECK_SCALE=100)
# These must all pass for 10 continuous seconds for gpsGoodToAlign to become true.
# ---------------------------------------------------------------------------

_GPS_QUALITY_CHECKS: list[tuple[str, float, str, str]] = [
    # (field,         fail_threshold, unit,    check_mask)
    ("h_acc",         5000.0,  "mm",    "MASK_GPS_POS_ERR  hAcc>5m"),
    ("v_acc",         7500.0,  "mm",    "MASK_GPS_POS_ERR  vAcc>7.5m"),
    ("vel_acc",       1000.0,  "mm/s",  "MASK_GPS_SPD_ERR  sAcc>1m/s"),
    ("eph",            250.0,  "x100",  "MASK_GPS_HDOP     HDOP>2.5"),
    ("satellites_visible", 6,  "",      "MASK_GPS_NSATS    sats<6"),
]

def _gps_quality_fails(d: dict) -> list[str]:
    """Return list of failing GPS quality check descriptions for one GPS_RAW_INT record."""
    fails = []
    for field, thresh, unit, label in _GPS_QUALITY_CHECKS:
        val = d.get(field, None)
        if val is None:
            continue
        if field == "satellites_visible":
            if val < thresh:
                fails.append(f"{label} (={val})")
        else:
            if val > thresh:
                fails.append(f"{label} (={val}{unit})")
    return fails


def print_gps(records: list[dict], t0_wall: float, t0_boot: float,
              verbose: bool = True) -> None:
    _section("GPS ACQUISITION")

    prev_fix = -1
    first_good_printed = False
    rows = []
    for d in records:
        if d["mavpackettype"] != "GPS_RAW_INT":
            continue
        st  = to_sim(d["_t_wall"], t0_wall, t0_boot)
        fix = d.get("fix_type", 0)
        rows.append((st, d))

        # Print fix_type transitions
        if fix != prev_fix:
            name  = _GPS_FIX.get(fix, str(fix))
            sats  = d.get("satellites_visible", 0)
            eph   = d.get("eph", 0)
            h_acc = d.get("h_acc", 0)
            v_acc = d.get("v_acc", 0)
            s_acc = d.get("vel_acc", 0)
            lat   = d.get("lat", 0) / 1e7
            lon   = d.get("lon", 0) / 1e7
            warn  = " [!!]" if fix < 3 and fix != prev_fix else (" [OK]" if fix >= 3 else "")
            fails = _gps_quality_fails(d)
            fail_s = f"  QUALITY_FAIL: {'; '.join(fails)}" if fails else ""
            print(f"  t={st:7.2f}s  fix={fix} ({name:<10})  sats={sats:2d}  eph={eph:5d}"
                  f"  h_acc={h_acc:5d}mm  v_acc={v_acc:5d}mm  s_acc={s_acc:5d}mm/s"
                  f"  lat={lat:.6f}  lon={lon:.7f}{warn}{fail_s}")
            prev_fix = fix

        if fix >= 3 and not first_good_printed:
            first_good_printed = True
            h_acc = d.get("h_acc", 0)
            v_acc = d.get("v_acc", 0)
            s_acc = d.get("vel_acc", 0)
            fails = _gps_quality_fails(d)
            fail_s = f"  QUALITY_FAIL: {'; '.join(fails)}" if fails else "  quality [OK]"
            print(f"  --> First good fix at t={st:.2f}s  h_acc={h_acc}mm  v_acc={v_acc}mm  s_acc={s_acc}mm/s{fail_s}")

    if not first_good_printed:
        print("  [!!] GPS never achieved fix_type >= 3")

    # GPS quality checks reference
    print()
    print("  GPS quality thresholds (EK3 defaults, all must pass 10s for gpsGoodToAlign):")
    for *_, label in _GPS_QUALITY_CHECKS:
        print(f"    {label}")

    # GPS position timeline (every ~10 s)
    if verbose and rows:
        print()
        print(f"  {'t_sim':>7}  {'fix':>3}  {'sats':>4}  {'h_acc_m':>8}  {'v_acc_m':>8}  {'s_acc_ms':>9}  {'lat':>12}  {'lon':>12}  quality")
        print(f"  {'-'*7}  {'-'*3}  {'-'*4}  {'-'*8}  {'-'*8}  {'-'*9}  {'-'*12}  {'-'*12}  {'-'*20}")
        last_printed = -999.0
        for st, d in rows:
            if st - last_printed >= 10.0 or d.get("fix_type", 0) < 3:
                lat   = d.get("lat", 0) / 1e7
                lon   = d.get("lon", 0) / 1e7
                h_acc = d.get("h_acc", 0) / 1000.0
                v_acc = d.get("v_acc", 0) / 1000.0
                s_acc = d.get("vel_acc", 0) / 1000.0
                fails = _gps_quality_fails(d)
                qstr  = "FAIL: " + " | ".join(f.split()[0] for f in fails) if fails else "OK"
                print(f"  {st:7.1f}  {d['fix_type']:3d}  {d.get('satellites_visible',0):4d}"
                      f"  {h_acc:8.3f}  {v_acc:8.3f}  {s_acc:9.3f}"
                      f"  {lat:12.7f}  {lon:12.7f}  {qstr}")
                last_printed = st

# ---------------------------------------------------------------------------

def print_ekf_flags(records: list[dict], t0_wall: float, t0_boot: float,
                    show_all: bool = False) -> None:
    _section("EKF FLAGS")

    print("  Flag bit reference:")
    for mask, name in _EKF_FLAGS.items():
        warn = "  <- [problem]" if mask in _EKF_WARN else ""
        print(f"    0x{mask:04x}  {name}{warn}")
    print()

    print("  Transitions:")
    prev = 0
    first = True
    for d in records:
        if d["mavpackettype"] != "EKF_STATUS_REPORT":
            continue
        st    = to_sim(d["_t_wall"], t0_wall, t0_boot)
        flags = d.get("flags", 0)

        if flags != prev or first:
            diff  = _flag_diff(prev, flags) if not first else f"init [{_decode_flags(flags)}]"
            warn  = "  [!!]" if _has_warn(flags) else ""
            print(f"    t={st:7.2f}s  0x{flags:04x}  {diff}{warn}")
            prev  = flags
            first = False
        elif show_all:
            print(f"    t={st:7.2f}s  0x{flags:04x}  (unchanged)")

    # Final state summary
    print()
    has_pos    = bool(prev & 0x0018)
    has_vel    = bool(prev & 0x0006)
    has_att    = bool(prev & 0x0001)
    const_pos  = bool(prev & 0x0080)
    gps_glitching = bool(prev & 0x8000)
    uninit        = bool(prev & 0x0400)

    # Determine whether any gps_glitching is post-fusion transient
    t_fusion = _find_const_pos_clear_time(records, t0_wall, t0_boot)
    if gps_glitching and t_fusion is not None:
        settle_end = t_fusion + _POST_FUSION_SETTLE_S
        glitch_outside = any(
            to_sim(d["_t_wall"], t0_wall, t0_boot) > settle_end
            for d in records
            if d["mavpackettype"] == "EKF_STATUS_REPORT" and d.get("flags", 0) & 0x8000
        )
        glitch_status = (
            "[!!] GPS glitch active" if glitch_outside
            else f"[OK] post-fusion transient (within {_POST_FUSION_SETTLE_S:.0f}s of GPS fusion at t={t_fusion:.1f}s)"
        )
    else:
        glitch_status = "[!!] GPS glitch active" if gps_glitching else "[OK] not set"

    print(f"  Final flags: 0x{prev:04x}")
    print(f"    attitude        : {_ok(has_att)}")
    print(f"    velocity        : {_ok(has_vel)}")
    print(f"    horiz position  : {_ok(has_pos)}  {'(neither rel nor abs)' if not has_pos else ''}")
    print(f"    const_pos_mode  : {'[!!] EKF in const-pos -- get_relative_position_NED_origin() may return nil' if const_pos else '[OK] not set'}")
    print(f"    uninitialized   : {'[!!] EKF has never been healthy' if uninit else '[OK] not set'}")
    print(f"    gps_glitching   : {glitch_status}")

# ---------------------------------------------------------------------------

def print_gps_vs_simstate(records: list[dict], t0_wall: float, t0_boot: float) -> None:
    """Compare GPS_RAW_INT position against SIMSTATE ground truth."""
    _section("GPS vs SIMSTATE")

    # Pair GPS_RAW_INT and SIMSTATE by closest timestamp
    gps_rows: list[tuple[float, dict]]  = []
    sim_rows: list[tuple[float, dict]]  = []

    for d in records:
        st = to_sim(d["_t_wall"], t0_wall, t0_boot)
        if d["mavpackettype"] == "GPS_RAW_INT" and d.get("fix_type", 0) >= 3:
            gps_rows.append((st, d))
        elif d["mavpackettype"] == "SIMSTATE":
            sim_rows.append((st, d))

    if not gps_rows:
        print("  No valid GPS_RAW_INT records (fix_type >= 3).")
        return
    if not sim_rows:
        print("  No SIMSTATE records.")
        return

    # For each GPS record find closest SIMSTATE
    print(f"  {'t_sim':>7}  {'gps_lat':>12}  {'sim_lat':>12}  {'dlat_m':>8}  {'gps_lon':>13}  {'sim_lon':>13}  {'dlon_m':>8}")
    print(f"  {'-'*7}  {'-'*12}  {'-'*12}  {'-'*8}  {'-'*13}  {'-'*13}  {'-'*8}")

    COS_LAT = math.cos(math.radians(51.5))
    M_PER_DEG_LAT = 111111.0
    M_PER_DEG_LON = 111111.0 * COS_LAT

    last_printed = -999.0
    max_err = 0.0
    max_err_t = 0.0

    sim_idx = 0
    for st, g in gps_rows:
        if st - last_printed < 5.0:
            continue
        # Find nearest SIMSTATE
        while sim_idx + 1 < len(sim_rows) and sim_rows[sim_idx + 1][0] <= st:
            sim_idx += 1
        if sim_idx >= len(sim_rows):
            break
        ss = sim_rows[sim_idx][1]
        gps_lat = g["lat"] / 1e7
        gps_lon = g["lon"] / 1e7
        sim_lat = ss["lat"] / 1e7
        sim_lon = ss["lng"] / 1e7
        dlat_m  = (gps_lat - sim_lat) * M_PER_DEG_LAT
        dlon_m  = (gps_lon - sim_lon) * M_PER_DEG_LON
        err_m   = math.sqrt(dlat_m**2 + dlon_m**2)
        flag    = " [!!]" if err_m > 50 else " [!] " if err_m > 10 else ""
        print(f"  {st:7.1f}  {gps_lat:12.7f}  {sim_lat:12.7f}  {dlat_m:8.1f}  {gps_lon:13.7f}  {sim_lon:13.7f}  {dlon_m:8.1f}{flag}")
        if err_m > max_err:
            max_err = err_m
            max_err_t = st
        last_printed = st

    print()
    flag = "[!!]" if max_err > 50 else "[!] " if max_err > 10 else "[OK]"
    print(f"  Max GPS position error: {max_err:.1f} m at t={max_err_t:.1f}s  {flag}")

# ---------------------------------------------------------------------------

def print_sensor_vs_simstate(records: list[dict], t0_wall: float, t0_boot: float) -> None:
    """Compare each sensor stream ArduPilot reads against SIMSTATE ground truth.

    Checks:
      1. IMU accelerometer (RAW_IMU vs SIMSTATE) -- scaled from mg to m/s^2
      2. IMU gyroscope    (RAW_IMU vs SIMSTATE) — scaled from mrad/s to rad/s
      3. EKF attitude     (ATTITUDE vs SIMSTATE) — roll/pitch/yaw error in degrees
    """
    _section("SENSOR vs SIMSTATE")

    _G = 9.80665  # m/s^2 per g

    # --- build time-indexed lookups -----------------------------------------
    sim_by_t:  list[tuple[float, dict]] = []
    imu_by_t:  list[tuple[float, dict]] = []
    att_by_t:  list[tuple[float, dict]] = []

    for d in records:
        mt = d.get("mavpackettype", "")
        t  = to_sim(d["_t_wall"], t0_wall, t0_boot)
        if mt == "SIMSTATE":
            sim_by_t.append((t, d))
        elif mt == "RAW_IMU":
            imu_by_t.append((t, d))
        elif mt == "ATTITUDE":
            att_by_t.append((t, d))

    if not sim_by_t:
        print("  No SIMSTATE records — cannot compare sensors.")
        return

    def _nearest_sim(t: float) -> "dict | None":
        if not sim_by_t:
            return None
        lo, hi = 0, len(sim_by_t) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if sim_by_t[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid
        # pick whichever of lo-1 / lo is closer
        best = lo
        if lo > 0 and abs(sim_by_t[lo-1][0] - t) < abs(sim_by_t[lo][0] - t):
            best = lo - 1
        dt = abs(sim_by_t[best][0] - t)
        return sim_by_t[best][1] if dt < 0.1 else None

    # --- IMU accelerometer --------------------------------------------------
    print("  IMU ACCELEROMETER  (RAW_IMU mg -> m/s^2  vs  SIMSTATE m/s^2)")
    acc_errs: dict[str, list[float]] = {"x": [], "y": [], "z": []}
    acc_max:  dict[str, tuple[float, float]] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0)}
    for t, d in imu_by_t:
        s = _nearest_sim(t)
        if s is None:
            continue
        # RAW_IMU is in mg (milli-g); SIMSTATE is in m/s^2 (specific force, body frame)
        for ax, sk in (("x", "xacc"), ("y", "yacc"), ("z", "zacc")):
            raw_ms2 = d.get(sk, 0) / 1000.0 * _G
            sim_ms2 = s.get(sk, 0)
            err = abs(raw_ms2 - sim_ms2)
            acc_errs[ax].append(err)
            if err > acc_max[ax][0]:
                acc_max[ax] = (err, t)

    if acc_errs["x"]:
        for ax in ("x", "y", "z"):
            errs = acc_errs[ax]
            med  = sorted(errs)[len(errs)//2]
            mx, t_mx = acc_max[ax]
            flag = "[!!]" if mx > 2.0 else "[!] " if mx > 0.5 else "[OK]"
            print(f"    accel_{ax}:  median={med:.3f} m/s^2  max={mx:.3f} m/s^2 at t={t_mx:.1f}s  {flag}")
    else:
        print("    (no RAW_IMU records)")

    # --- IMU gyroscope ------------------------------------------------------
    print("  IMU GYROSCOPE  (RAW_IMU mrad/s -> rad/s  vs  SIMSTATE rad/s)")
    gyr_errs: dict[str, list[float]] = {"x": [], "y": [], "z": []}
    gyr_max:  dict[str, tuple[float, float]] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0)}
    for t, d in imu_by_t:
        s = _nearest_sim(t)
        if s is None:
            continue
        for ax, gk in (("x", "xgyro"), ("y", "ygyro"), ("z", "zgyro")):
            raw_rads = d.get(gk, 0) / 1000.0
            sim_rads = s.get(gk, 0)
            err = abs(raw_rads - sim_rads)
            gyr_errs[ax].append(err)
            if err > gyr_max[ax][0]:
                gyr_max[ax] = (err, t)

    if gyr_errs["x"]:
        for ax in ("x", "y", "z"):
            errs = gyr_errs[ax]
            med  = sorted(errs)[len(errs)//2]
            mx, t_mx = gyr_max[ax]
            flag = "[!!]" if mx > 0.1 else "[!] " if mx > 0.02 else "[OK]"
            print(f"    gyro_{ax}:   median={med:.4f} rad/s  max={mx:.4f} rad/s at t={t_mx:.1f}s  {flag}")
    else:
        print("    (no RAW_IMU records)")

    # --- EKF attitude vs SIMSTATE -------------------------------------------
    print("  EKF ATTITUDE  (ATTITUDE vs SIMSTATE, degrees)")
    att_errs: dict[str, list[float]] = {"roll": [], "pitch": [], "yaw": []}
    att_max:  dict[str, tuple[float, float]] = {"roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0)}
    for t, d in att_by_t:
        s = _nearest_sim(t)
        if s is None:
            continue
        for ax in ("roll", "pitch", "yaw"):
            ekf_deg = math.degrees(d.get(ax, 0.0))
            sim_deg = math.degrees(s.get(ax, 0.0))
            # wrap to [-180, 180]
            err = abs(((ekf_deg - sim_deg) + 180) % 360 - 180)
            att_errs[ax].append(err)
            if err > att_max[ax][0]:
                att_max[ax] = (err, t)

    if att_errs["roll"]:
        for ax in ("roll", "pitch", "yaw"):
            errs = att_errs[ax]
            med  = sorted(errs)[len(errs)//2]
            mx, t_mx = att_max[ax]
            flag = "[!!]" if mx > 20 else "[!] " if mx > 5 else "[OK]"
            note = "  (yaw from GPS-vel, not compass)" if ax == "yaw" else ""
            print(f"    {ax:5s}:  median={med:5.1f} deg  max={mx:5.1f} deg at t={t_mx:.1f}s  {flag}{note}")
    else:
        print("    (no ATTITUDE records)")

# ---------------------------------------------------------------------------

def print_gps_physical_consistency(records: list[dict], t0_wall: float, t0_boot: float,
                                    t_stable_end: "float | None" = None) -> None:
    """Check whether GPS positions and velocities are mutually consistent and
    physically plausible.

    Checks:
      1. Position-derived velocity  vs  GPS-reported vel/cog
      2. vel_acc (sAcc) — zero is a SITL artifact (commented-out assignment in AP_GPS_SITL.cpp);
         ArduPilot floors noise at 0.5 m/s in EKFGSF so this has no real impact on GSF
      3. Implied acceleration between consecutive samples
      4. Velocity variability (GSF needs direction changes to determine heading)
    """
    import math as _math
    _section("GPS PHYSICAL CONSISTENCY")

    rows: list[tuple[float, dict]] = []
    for d in records:
        if d.get("mavpackettype") != "GPS_RAW_INT":
            continue
        if d.get("fix_type", 0) < 3:
            continue
        st = to_sim(d["_t_wall"], t0_wall, t0_boot)
        if t_stable_end is not None and st >= t_stable_end:
            continue
        rows.append((st, d))

    if not rows:
        print("  No GPS_RAW_INT records with fix>=3 in stable window.")
        return

    REF_LAT_DEG = rows[0][1]["lat"] / 1e7
    M_PER_DEG_LAT = 111320.0
    M_PER_DEG_LON = 111320.0 * _math.cos(_math.radians(REF_LAT_DEG))

    # ------------------------------------------------------------------ #
    # vel_acc (sAcc) distribution                                         #
    # ------------------------------------------------------------------ #
    vel_accs = [d.get("vel_acc", 0) for _, d in rows]
    n_zero_vacc = sum(1 for v in vel_accs if v == 0)
    max_vacc = max(vel_accs)
    print()
    if n_zero_vacc == len(rows):
        print(f"  vel_acc (sAcc): [!]  ALL {len(rows)} samples report 0 mm/s  (SITL artifact)")
        print(f"       AP_GPS_SITL.cpp sets have_speed_accuracy=true but leaves speed_accuracy=0")
        print(f"       because the assignment is commented out (pkt.horizontal_vel_accuracy).")
        print(f"       NOTE: this does NOT inflate GSF innovation scores -- ArduPilot has two floors:")
        print(f"         AP_NavEKF3_Measurements.cpp: gpsSpdAccuracy = MAX(raw, EK3_VELNE_M_NSE=0.3)")
        print(f"         EKFGSF_yaw.cpp:139:          velObsVar = sq(fmax(velAcc, 0.5)) = 0.25 m2/s2")
        print(f"       vel_acc=0 is cosmetic only; the EKF always uses at least 0.5 m/s noise.")
    elif n_zero_vacc > 0:
        print(f"  vel_acc (sAcc): [!]  {n_zero_vacc}/{len(rows)} samples report 0 mm/s  (max={max_vacc} mm/s)")
    else:
        print(f"  vel_acc (sAcc): [OK] range {min(vel_accs)}–{max_vacc} mm/s across {len(rows)} samples")

    # ------------------------------------------------------------------ #
    # Build per-sample NE velocity from position differences              #
    # ------------------------------------------------------------------ #
    t_first_fix = rows[0][0]
    SKIP_AFTER_FIX_S = 3.0   # ignore pos-derived velocity for the first 3s after fix
                              # (SITL GPS often has a stale-position jump on first fix)

    pos_derived: list[tuple[float, float, float]] = []   # (t, vN, vE)
    rep_vne: list[tuple[float, float, float]] = []        # (t, vN, vE)

    for i, (st, d) in enumerate(rows):
        vel_ms  = d.get("vel", 0) / 100.0          # cm/s → m/s
        cog_deg = d.get("cog", 0) / 100.0          # cdeg → deg
        vN_rep  =  vel_ms * _math.cos(_math.radians(cog_deg))
        vE_rep  =  vel_ms * _math.sin(_math.radians(cog_deg))
        rep_vne.append((st, vN_rep, vE_rep))

        if i > 0:
            pt, pd = rows[i - 1]
            dt = st - pt
            if dt > 1e-6:
                dlat_m = (d["lat"] - pd["lat"]) / 1e7 * M_PER_DEG_LAT
                dlon_m = (d["lon"] - pd["lon"]) / 1e7 * M_PER_DEG_LON
                pos_derived.append((st, dlat_m / dt, dlon_m / dt))

    # ------------------------------------------------------------------ #
    # Position vs reported velocity agreement                             #
    # ------------------------------------------------------------------ #
    discrepancies = []
    for i, ((t, vN_pos, vE_pos), (_, vN_rep, vE_rep)) in enumerate(
            zip(pos_derived, rep_vne[1:])):
        diff = _math.hypot(vN_pos - vN_rep, vE_pos - vE_rep)
        in_skip = t < (t_first_fix + SKIP_AFTER_FIX_S)
        discrepancies.append((t, diff, vN_pos, vE_pos, vN_rep, vE_rep, in_skip))

    normal_disc = [x for x in discrepancies if not x[6]]
    if normal_disc:
        median_disc = sorted(d[1] for d in normal_disc)[len(normal_disc) // 2]
        max_disc    = max(d[1] for d in normal_disc)
        bad = [x for x in normal_disc if x[1] > 1.0]
        print()
        if bad:
            print(f"  Position vs velocity agreement:  median={median_disc:.3f} m/s  max={max_disc:.3f} m/s")
            print(f"  [!!] {len(bad)} samples where pos-derived velocity differs from reported by >1 m/s:")
            for t, diff, vN_pos, vE_pos, vN_rep, vE_rep, _ in bad[:10]:
                print(f"    t={t:7.2f}s  pos=(N={vN_pos:.2f} E={vE_pos:.2f})  rep=(N={vN_rep:.2f} E={vE_rep:.2f})  diff={diff:.2f} m/s")
        else:
            print(f"  Position vs velocity agreement:  [OK] median={median_disc:.3f} m/s  max={max_disc:.3f} m/s")

    # ------------------------------------------------------------------ #
    # Implied acceleration                                                #
    # ------------------------------------------------------------------ #
    MAX_REASONABLE_ACCEL = 20.0   # m/s^2 — well above copter limit
    accel_violations: list[tuple[float, float]] = []
    for i in range(1, len(pos_derived)):
        t0s, vN0, vE0 = pos_derived[i - 1]
        t1s, vN1, vE1 = pos_derived[i]
        if t1s < (t_first_fix + SKIP_AFTER_FIX_S):
            continue   # skip fix-acquisition window
        dt = t1s - t0s
        if dt < 0.1:
            continue   # gap too short — dt < 100ms produces noisy acceleration estimates
        acc = _math.hypot(vN1 - vN0, vE1 - vE0) / dt
        if acc > MAX_REASONABLE_ACCEL:
            accel_violations.append((t1s, acc))

    print()
    if accel_violations:
        print(f"  Implied acceleration: [!!] {len(accel_violations)} samples exceed {MAX_REASONABLE_ACCEL} m/s^2:")
        for t, a in accel_violations[:10]:
            print(f"    t={t:7.2f}s  acc={a:.1f} m/s^2  (position jump)")
    else:
        print(f"  Implied acceleration: [OK] no samples exceed {MAX_REASONABLE_ACCEL} m/s^2")

    # ------------------------------------------------------------------ #
    # Velocity variability — GSF needs direction changes to converge      #
    # ------------------------------------------------------------------ #
    all_vN = [vN for _, vN, _ in rep_vne]
    all_vE = [vE for _, _, vE in rep_vne]
    all_spd = [_math.hypot(vN, vE) for vN, vE in zip(all_vN, all_vE)]

    spd_sorted = sorted(all_spd)
    spd_median = spd_sorted[len(spd_sorted) // 2]
    spd_max    = spd_sorted[-1]
    vN_range   = max(all_vN) - min(all_vN)
    vE_range   = max(all_vE) - min(all_vE)

    # Fraction of time at near-constant velocity (speed change < 0.1 m/s from median)
    n_constant = sum(1 for s in all_spd if abs(s - spd_median) < 0.1)

    print()
    print(f"  Velocity distribution (reported vel/cog):")
    print(f"    Speed: median={spd_median:.2f} m/s  max={spd_max:.2f} m/s")
    print(f"    vN range: {min(all_vN):.2f} to {max(all_vN):.2f} m/s  (spread={vN_range:.2f} m/s)")
    print(f"    vE range: {min(all_vE):.2f} to {max(all_vE):.2f} m/s  (spread={vE_range:.2f} m/s)")
    print(f"    {n_constant}/{len(all_spd)} samples within 0.1 m/s of median speed  ({100*n_constant/len(all_spd):.0f}% near-constant)")

    gsf_obs = vN_range + vE_range   # rough total velocity variation
    print()
    if gsf_obs < 0.5:
        print(f"  GSF observability: [!!] Velocity direction almost never changes (total spread={gsf_obs:.2f} m/s).")
        print(f"       EKFGSF_yaw cannot distinguish heading from constant straight-line flight.")
    elif gsf_obs < 2.0:
        print(f"  GSF observability: [!]  Low velocity variation (total spread={gsf_obs:.2f} m/s) — slow convergence.")
    else:
        print(f"  GSF observability: [OK] Sufficient velocity variation for heading estimation ({gsf_obs:.2f} m/s spread).")


# ---------------------------------------------------------------------------

def print_attitude(records: list[dict], t0_wall: float, t0_boot: float) -> None:
    _section("ATTITUDE RATES")

    peak_roll  = 0.0
    peak_pitch = 0.0
    peak_yaw   = 0.0
    t_roll = t_pitch = t_yaw = 0.0

    for d in records:
        if d["mavpackettype"] != "ATTITUDE":
            continue
        st = to_sim(d["_t_wall"], t0_wall, t0_boot)
        rs = abs(math.degrees(d.get("rollspeed",  0.0)))
        ps = abs(math.degrees(d.get("pitchspeed", 0.0)))
        ys = abs(math.degrees(d.get("yawspeed",   0.0)))
        if rs > peak_roll:   peak_roll  = rs;  t_roll  = st
        if ps > peak_pitch:  peak_pitch = ps;  t_pitch = st
        if ys > peak_yaw:    peak_yaw   = ys;  t_yaw   = st

    def rate_flag(r: float) -> str:
        if r > 1000: return "[!!] tumbling"
        if r > 200:  return "[!]  high"
        return "[OK]"

    print(f"  Peak roll  rate: {peak_roll:8.1f} deg/s  at t={t_roll:.1f}s   {rate_flag(peak_roll)}")
    print(f"  Peak pitch rate: {peak_pitch:8.1f} deg/s  at t={t_pitch:.1f}s   {rate_flag(peak_pitch)}")
    print(f"  Peak yaw   rate: {peak_yaw:8.1f} deg/s  at t={t_yaw:.1f}s   {rate_flag(peak_yaw)}")

# ---------------------------------------------------------------------------

def print_variances(records: list[dict], t0_wall: float, t0_boot: float) -> None:
    _section("EKF VARIANCES")

    fields = [
        ("pos_horiz_variance",   2.0,  "m^2   horiz pos"),
        ("pos_vert_variance",    4.0,  "m^2   vert pos"),
        ("velocity_variance",    1.0,  "(m/s)^2 vel"),
        ("compass_variance",     0.09, "rad^2 compass"),
        ("terrain_alt_variance", 4.0,  "m^2   terrain alt"),
    ]

    peaks: dict[str, tuple[float, float]] = {fname: (0.0, 0.0) for fname, _, _ in fields}

    for d in records:
        if d["mavpackettype"] != "EKF_STATUS_REPORT":
            continue
        st = to_sim(d["_t_wall"], t0_wall, t0_boot)
        for fname, _, _ in fields:
            v = d.get(fname, 0.0)
            if v > peaks[fname][0]:
                peaks[fname] = (v, st)

    t_fusion   = _find_const_pos_clear_time(records, t0_wall, t0_boot)
    settle_end = (t_fusion + _POST_FUSION_SETTLE_S) if t_fusion is not None else None

    for fname, thresh, label in fields:
        peak_val, peak_t = peaks[fname]
        if settle_end is not None and t_fusion is not None and t_fusion <= peak_t <= settle_end:
            flag = "[OK]"
            note = f"  (post-fusion transient, expected within {_POST_FUSION_SETTLE_S:.0f}s of GPS fusion)"
        else:
            flag = "[!!]" if peak_val > thresh * 2 else "[!] " if peak_val > thresh else "[OK]"
            note = ""
        print(f"  {fname:<30}  peak={peak_val:.3f} {label}  at t={peak_t:.1f}s  {flag}{note}")

# ---------------------------------------------------------------------------
# DataFlash loader
# ---------------------------------------------------------------------------

# XKF4.GPS bitmask — same layout as gpsCheckStatus in NavEKF3_core.h
_DF_GPS_CHECK_BITS: dict[int, str] = {
    0:  "bad_sAcc",
    1:  "bad_hAcc",
    2:  "bad_vAcc",
    3:  "bad_yaw",
    4:  "bad_sats",
    5:  "bad_VZ",
    6:  "bad_horiz_drift",
    7:  "bad_hdop",
    8:  "bad_vert_vel",
    9:  "bad_fix",
    10: "bad_horiz_vel",
}

_MAG_FUSION_NAMES: dict[int, str] = {
    0: "none (no compass)",
    1: "compass",
    2: "learning",
    3: "forced",
}


def load_dataflash(log_dir: "Path") -> "dict | None":
    """Load EKF-relevant fields from dataflash.BIN.

    Returns a dict of time-sorted lists, or None if unavailable.
    Silently returns None when pymavlink is not installed.
    """
    df_path: "Path | None" = None
    for p in log_dir.iterdir():
        if p.suffix.upper() == ".BIN":
            df_path = p
            break
    if df_path is None:
        return None

    try:
        from pymavlink import DFReader  # type: ignore
    except ImportError:
        return None

    log = DFReader.DFReader_binary(str(df_path), zero_time_base=True)

    data: dict = {
        "path":  df_path,
        "xkf1":  [],   # (t, gx, gy, gz, yaw)             gyro bias, heading
        "xkf3":  [],   # (t, iyaw)                         yaw innovation
        "xkf4":  [],   # (t, ss, gps_chk, erp)             solution status, GPS checks
        "xkfs":  [],   # (t, gta, chk_wait, mag_fusion)    GPS good-to-align, mag mode
        "xky0":  [],   # (t, yc, ycs)                      GSF central yaw, score
    }

    want = ["XKF1", "XKF3", "XKF4", "XKFS", "XKY0"]
    while True:
        m = log.recv_match(type=want)
        if m is None:
            break
        if not hasattr(m, "C") or m.C != 0:
            continue  # primary EKF core only
        t  = m.TimeUS / 1_000_000.0
        mt = m.get_type()
        if mt == "XKF1":
            data["xkf1"].append((t, m.GX, m.GY, m.GZ, m.Yaw))
        elif mt == "XKF3":
            data["xkf3"].append((t, m.IYAW))
        elif mt == "XKF4":
            data["xkf4"].append((t, m.SS, m.GPS, m.errRP))
        elif mt == "XKFS":
            data["xkfs"].append((t, m.GPS_GTA, m.GPS_CHK_WAIT, m.MAG_FUSION))
        elif mt == "XKY0":
            data["xky0"].append((t, m.YC, m.YCS))

    return data


def _print_df_const_pos_analysis(df: dict, t_stable_end: "float | None" = None) -> dict:
    """Print DataFlash-derived EKF analysis and return a conclusions dict.

    t_stable_end: if given, all DataFlash records at or after this time are
                  discarded before analysis.  Typically set to the tumble-onset
                  time so post-crash noise does not corrupt convergence checks.

    The conclusions dict has keys:
      has_compass       bool
      gps_quality_ok    bool   gpsGoodToAlign stayed true after it first turned on
      gyro_bias_ok      bool   delAngBiasLearned almost certainly true
      gsf_converged     bool   GSF yaw score ever reached acceptable level
      yaw_align_cause   str    human-readable root cause of yawAlignComplete=false
    """
    print()
    print(f"  --- DataFlash: {df['path'].name} ---")
    if t_stable_end is not None:
        print(f"  (records clipped at t={t_stable_end:.2f}s -- tumble onset; post-crash data excluded)")
    else:
        print(f"  (no tumble detected -- using full log)")

    # Clip all series at t_stable_end so every section sees the same window.
    def _clip(lst: list) -> list:
        if t_stable_end is None:
            return lst
        return [r for r in lst if r[0] < t_stable_end]

    conclusions: dict = {}

    # ------------------------------------------------------------------ #
    # XKFS: GPS good-to-align + compass mode                             #
    # ------------------------------------------------------------------ #
    xkfs = _clip(df["xkfs"])
    if xkfs:
        print()
        print("  XKFS  (GPS_GTA=gpsGoodToAlign, MAG_FUSION):")
        prev = None
        for t, gta, wait, mag in xkfs:
            row = (gta, wait, mag)
            if row != prev:
                mag_s  = _MAG_FUSION_NAMES.get(mag, str(mag))
                gta_s  = "[OK]" if gta else "[!!]"
                wait_s = "  [waiting for 10s window]" if wait else ""
                print(f"    t={t:7.2f}s  GPS_GTA={gta}{gta_s}  MAG_FUSION={mag}({mag_s}){wait_s}")
                prev = row

        # Summarise
        mag_vals   = [mag for _, _, _, mag in xkfs]
        has_compass = any(m > 0 for m in mag_vals)
        gta_vals   = [gta for _, gta, _, _ in xkfs]
        # GPS quality is "sustained" if it turned on and stayed on (never dropped back to 0 after first 1)
        first_on   = next((i for i, v in enumerate(gta_vals) if v == 1), None)
        gps_quality_ok = (first_on is not None and
                          all(v == 1 for v in gta_vals[first_on:]))

        conclusions["has_compass"]    = has_compass
        conclusions["gps_quality_ok"] = gps_quality_ok

        compass_s = "compass in use" if has_compass else "NO compass (MAG_FUSION=0) -- yaw from GSF only"
        gps_s     = "sustained once set" if gps_quality_ok else "intermittent (oscillated)"
        print(f"    => Compass: {compass_s}")
        print(f"    => gpsGoodToAlign: {gps_s}")

    # ------------------------------------------------------------------ #
    # XKF4: GPS check bitmask -- which checks were failing and when       #
    # ------------------------------------------------------------------ #
    xkf4 = _clip(df["xkf4"])
    if xkf4:
        gps_chk_vals = [gc for _, _, gc, _ in xkf4]
        ever_failing  = any(gc != 0 for gc in gps_chk_vals)
        print()
        print("  XKF4.GPS check bitmask  (0 = all passing):")
        prev_gc = -1
        for t, _, gc, _ in xkf4:
            if gc != prev_gc:
                if gc:
                    fails = [name for bit, name in _DF_GPS_CHECK_BITS.items() if gc & (1 << bit)]
                    print(f"    t={t:7.2f}s  0x{gc:04x}  FAILING: {', '.join(fails)}")
                else:
                    print(f"    t={t:7.2f}s  0x0000  all passing")
                prev_gc = gc
        if not ever_failing:
            print("    => GPS checks passed throughout  [OK]")

    # ------------------------------------------------------------------ #
    # XKF1: gyro bias convergence                                        #
    # delAngBiasLearned is set when bias *variance* (not value) falls    #
    # below a threshold.  Proxy: bias should be small and stable during  #
    # early flight.  Ignore end-of-flight records corrupted by crashes.  #
    # ------------------------------------------------------------------ #
    xkf1 = _clip(df["xkf1"])
    if xkf1:
        mags   = [(gx**2 + gy**2 + gz**2) ** 0.5 for _, gx, gy, gz, _ in xkf1]
        mags_sorted = sorted(mags)
        median_mag  = mags_sorted[len(mags_sorted) // 2]
        max_mag     = mags_sorted[-1]
        _, gx0, gy0, gz0, _ = xkf1[0]
        _, gxm, gym, gzm, _ = xkf1[-1]
        change = max(abs(gxm - gx0), abs(gym - gy0), abs(gzm - gz0))
        # Clearly converged: median < 1 deg/s AND change over analysis window < 0.5 deg/s
        gyro_ok = median_mag < 1.0 and change < 0.5
        conclusions["gyro_bias_ok"] = gyro_ok
        status = ("[OK] converged -- delAngBiasLearned was NOT the blocker"
                  if gyro_ok else "[!!] large or still changing")
        print()
        t_xkf1_end = xkf1[-1][0]
        print(f"  XKF1 gyro bias (stable window, up to t={t_xkf1_end:.1f}s):")
        print(f"    median |bias|={median_mag:.4f} deg/s  max={max_mag:.4f}  change={change:.4f} deg/s")
        print(f"    => {status}")

    # ------------------------------------------------------------------ #
    # XKF3: yaw innovation                                               #
    # ------------------------------------------------------------------ #
    xkf3 = _clip(df["xkf3"])
    if xkf3:
        max_iyaw  = max(abs(v) for _, v in xkf3)
        mean_iyaw = sum(abs(v) for _, v in xkf3) / len(xkf3)
        print()
        print(f"  XKF3.IYAW (yaw innovation): max={max_iyaw:.3f}deg  mean_abs={mean_iyaw:.4f}deg")
        if max_iyaw < 0.1:
            print(f"    => Near-zero: EKF not fusing yaw observations (expected in AID_NONE/const_pos_mode)")
        else:
            print(f"    => [!!] Non-zero yaw innovation (compass or GPS-yaw being rejected)")

    # ------------------------------------------------------------------ #
    # XKY0: GSF yaw estimator convergence                                #
    # ------------------------------------------------------------------ #
    xky0 = _clip(df["xky0"])
    gsf_converged = False
    if xky0:
        ycs_vals  = [ycs for _, _, ycs in xky0]
        ycs_sorted = sorted(ycs_vals)
        min_ycs    = ycs_sorted[0]
        median_ycs = ycs_sorted[len(ycs_sorted) // 2]
        # In ArduPilot EKFGSF, YCS is the weighted innovation NIS.
        # For 2-DOF (velocity NE), chi-squared 95th pct = ~6.
        # yawAlignComplete is set when the yaw variance < (15deg)^2.
        # Empirically: YCS consistently > 5 means no hypothesis fits well.
        # Use median (not min) so a brief tumble/crash transient doesn't
        # falsely indicate convergence when stable-flight YCS is high.
        gsf_converged = median_ycs < 3.0
        conclusions["gsf_converged"] = gsf_converged

        print()
        ycs_thresh_s = "median < 3.0 needed for convergence"
        gsf_s = (f"[OK] converged (median={median_ycs:.2f}, min={min_ycs:.2f})" if gsf_converged
                 else f"[!!] NEVER converged (median={median_ycs:.2f}, min={min_ycs:.2f}, {ycs_thresh_s})")
        print(f"  XKY0 GSF yaw estimator: first record t={xky0[0][0]:.2f}s  YCS {gsf_s}")

        # Sample at 5s intervals
        last_t = -999.0
        for t, yc, ycs in xky0:
            if t - last_t >= 5.0:
                flag = "[OK]" if ycs < 3.0 else "[!!]"
                print(f"    t={t:7.2f}s  YC={yc:8.2f}deg  YCS={ycs:7.4f} {flag}")
                last_t = t

        if not gsf_converged:
            # Diagnose why GSF failed
            yc_vals  = [yc for _, yc, _ in xky0]
            yc_range = max(yc_vals) - min(yc_vals)
            # If YC oscillates wildly the GSF is jumping between hypotheses
            if yc_range > 90:
                print(f"    => YC range={yc_range:.1f}deg: GSF jumping between hypotheses (vehicle motion insufficient to resolve yaw)")
            else:
                print(f"    => YC stable at ~{sum(yc_vals)/len(yc_vals):.1f}deg but innovations too large")
                print(f"       Check: GPS velocity noise too high, or true heading differs from GSF estimate")
    else:
        conclusions["gsf_converged"] = False
        print()
        print(f"  XKY0 GSF yaw: no records -- GSF not running (GPS not yet available when needed)")

    # ------------------------------------------------------------------ #
    # Root cause conclusion                                               #
    # ------------------------------------------------------------------ #
    has_compass     = conclusions.get("has_compass", False)
    gps_quality_ok  = conclusions.get("gps_quality_ok", False)
    gyro_ok         = conclusions.get("gyro_bias_ok", True)
    gsf_conv        = conclusions.get("gsf_converged", False)

    print()
    print("  DataFlash root cause:")
    if not gps_quality_ok:
        cause = "gpsGoodToAlign oscillated -- GPS quality checks kept resetting the 10s window"
    elif not gyro_ok:
        cause = "delAngBiasLearned=false -- gyro bias did not converge"
    elif not has_compass and not gsf_conv:
        cause = ("yawAlignComplete=false: no compass (MAG_FUSION=0) and GSF yaw never converged.\n"
                 "         The EKF had no reliable yaw source and readyToUseGPS() returned false throughout.")
    elif has_compass and not gsf_conv:
        cause = "yawAlignComplete=false: compass in use but yaw alignment did not complete (check mag calibration)"
    else:
        cause = "Cannot determine definitively -- all known blockers appear resolved in DataFlash"

    conclusions["yaw_align_cause"] = cause
    print(f"    {cause}")
    return conclusions


# ---------------------------------------------------------------------------
# const_pos_mode diagnosis
# ---------------------------------------------------------------------------

def print_const_pos_diagnosis(records: list[dict], t0_wall: float, t0_boot: float,
                               df: "dict | None" = None,
                               t_stable_end: "float | None" = None) -> None:
    """Explain why the EKF was/is in const_pos_mode.

    Works through the causal chain from const_pos_mode.md:
      1. GPS quality checks (gpsGoodToAlign / 10s window)
      2. validOrigin not yet set
      3. tiltAlignComplete / yawAlignComplete / delAngBiasLearned
      4. In-flight fallback (attAidLossCritical)

    t_stable_end: passed to DataFlash analysis to clip post-crash records.
    """
    _section("CONST_POS_MODE DIAGNOSIS")

    # ------------------------------------------------------------------ #
    # Build time-sorted series of EKF flags and GPS records               #
    # ------------------------------------------------------------------ #
    ekf_series: list[tuple[float, int, dict]] = []
    for d in records:
        if d["mavpackettype"] == "EKF_STATUS_REPORT":
            ekf_series.append((to_sim(d["_t_wall"], t0_wall, t0_boot), d.get("flags", 0), d))
    ekf_series.sort(key=lambda x: x[0])

    gps_series: list[tuple[float, dict]] = []
    for d in records:
        if d["mavpackettype"] == "GPS_RAW_INT":
            gps_series.append((to_sim(d["_t_wall"], t0_wall, t0_boot), d))
    gps_series.sort(key=lambda x: x[0])

    # ------------------------------------------------------------------ #
    # Find const_pos_mode intervals                                        #
    # ------------------------------------------------------------------ #
    intervals: list[tuple[float, float | None]] = []  # (start, end_or_None)
    in_cpm = False
    t_start = 0.0
    for t, flags, _ in ekf_series:
        if (flags & 0x0080) and not in_cpm:
            in_cpm  = True
            t_start = t
        elif not (flags & 0x0080) and in_cpm:
            intervals.append((t_start, t))
            in_cpm = False
    if in_cpm:
        intervals.append((t_start, None))   # still active

    if not intervals:
        print("  const_pos_mode was never active.  [OK]")
        return

    t_log_end = ekf_series[-1][0] if ekf_series else 0.0
    total_s   = sum((e if e is not None else t_log_end) - s for s, e in intervals)
    print(f"  Intervals where const_pos_mode (0x0080) was set:")
    for i, (s, e) in enumerate(intervals):
        if e is None:
            print(f"    [{i+1}] t={s:.2f}s -- {t_log_end:.2f}s  ({t_log_end-s:.1f}s)  [!! still active at end of log]")
        else:
            print(f"    [{i+1}] t={s:.2f}s -- {e:.2f}s  ({e-s:.1f}s)")
    print(f"  Total time in const_pos_mode: {total_s:.1f}s  out of {t_log_end:.1f}s log")

    # Focus diagnostics on the most significant interval:
    #   - prefer one that is still active (never cleared) at end of log
    #   - otherwise pick the longest one
    focus_idx = next(
        (i for i, (_, e) in enumerate(intervals) if e is None),
        max(range(len(intervals)), key=lambda i: (intervals[i][1] or t_log_end) - intervals[i][0]),
    )
    cpm_start, cpm_end = intervals[focus_idx]
    cpm_end_eff = cpm_end if cpm_end is not None else t_log_end
    never_cleared = cpm_end is None
    if len(intervals) > 1:
        print(f"\n  (Detailed analysis below focuses on interval [{focus_idx+1}],"
              f" the {'still-active' if never_cleared else 'longest'} one)")

    print()

    # ------------------------------------------------------------------ #
    # GPS status at const_pos onset                                        #
    # ------------------------------------------------------------------ #
    def gps_at(t: float) -> "dict | None":
        """GPS_RAW_INT record nearest to time t."""
        best, best_dt = None, float("inf")
        for gt, gd in gps_series:
            dt = abs(gt - t)
            if dt < best_dt:
                best_dt, best = dt, gd
        return best

    onset_gps = gps_at(cpm_start)
    onset_fix = onset_gps.get("fix_type", 0) if onset_gps else 0
    onset_sats = onset_gps.get("satellites_visible", 0) if onset_gps else 0
    print(f"  GPS at const_pos onset (t={cpm_start:.2f}s):")
    if onset_fix == 0:
        print(f"    fix_type=0  sats={onset_sats}  -- no GPS data / no fix  [!!]")
    else:
        print(f"    fix_type={onset_fix} ({_GPS_FIX.get(onset_fix,'?')})  sats={onset_sats}")

    # ------------------------------------------------------------------ #
    # When did GPS first become good (fix >= 3)?                          #
    # ------------------------------------------------------------------ #
    first_good_gps_t: float | None = None
    first_good_gps_d: dict = {}
    for gt, gd in gps_series:
        if gt >= cpm_start and gd.get("fix_type", 0) >= 3:
            first_good_gps_t = gt
            first_good_gps_d = gd
            break

    print()
    print(f"  GPS fix acquisition during const_pos_mode:")
    if first_good_gps_t is None:
        print(f"    [!!] GPS never reached fix_type >= 3 while in const_pos_mode.")
        print(f"         CAUSE: gpsDataToFuse=false or gpsGoodToAlign=false (no usable GPS)")
    else:
        h_acc = first_good_gps_d.get("h_acc", 0)
        v_acc = first_good_gps_d.get("v_acc", 0)
        s_acc = first_good_gps_d.get("vel_acc", 0)
        sats  = first_good_gps_d.get("satellites_visible", 0)
        fails = _gps_quality_fails(first_good_gps_d)
        fail_s = "  QUALITY_FAIL: " + "; ".join(fails) if fails else "  quality [OK]"
        delay = first_good_gps_t - cpm_start
        print(f"    fix_type >= 3 at t={first_good_gps_t:.2f}s (+{delay:.1f}s after const_pos onset)")
        print(f"    h_acc={h_acc}mm  v_acc={v_acc}mm  s_acc={s_acc}mm/s  sats={sats}{fail_s}")

        persist_after_fix = cpm_end_eff - first_good_gps_t
        if persist_after_fix <= 0:
            print(f"    [OK] const_pos_mode cleared before/at GPS fix -- normal cold-start")
        elif persist_after_fix < 12.0:
            print(f"    [WN] const_pos_mode persisted {persist_after_fix:.1f}s after GPS fix.")
            print(f"         LIKELY CAUSE: gpsGoodToAlign 10s quality-check window not yet complete.")
            print(f"         (GPS needs 10 continuous seconds passing all quality checks)")
        else:
            print(f"    [!!] const_pos_mode persisted {persist_after_fix:.1f}s after GPS fix -- 10s window alone does not explain this.")

    # ------------------------------------------------------------------ #
    # EKF origin                                                          #
    # ------------------------------------------------------------------ #
    origin_t: float | None = None
    for d in records:
        if d["mavpackettype"] == "STATUSTEXT":
            txt = d.get("text", "").rstrip("\x00").strip().lower()
            if "origin set" in txt or "origin" in txt and "ekf" in txt:
                t = to_sim(d["_t_wall"], t0_wall, t0_boot)
                if cpm_start <= t <= cpm_end_eff:
                    origin_t = t
                    break

    print()
    print(f"  EKF origin (validOrigin):")
    if origin_t is None and never_cleared:
        print(f"    [!!] No 'origin set' message found while in const_pos_mode.")
        print(f"         => validOrigin=false throughout => readyToUseGPS() always false.")
        print(f"         Most likely cause: GPS quality checks never passed the 10s window.")
    elif origin_t is not None:
        after_fix_s = f"+{origin_t - first_good_gps_t:.1f}s after GPS fix" if first_good_gps_t else "before GPS fix"
        print(f"    Origin set at t={origin_t:.2f}s ({after_fix_s})")
        if first_good_gps_t and abs(origin_t - first_good_gps_t - 10.0) < 3.0:
            print(f"    [OK] ~10s gap matches gpsGoodToAlign 10s check window.")
        remaining = cpm_end_eff - origin_t
        if remaining > 1.0:
            print(f"    [!!] const_pos_mode persisted {remaining:.1f}s AFTER origin was set.")
            print(f"         => validOrigin=true but EKF still did not enter AID_ABSOLUTE.")
            print(f"         Remaining conditions in readyToUseGPS() that could be blocking:")
            print(f"           - yawAlignComplete = false  (heading not aligned)")
            print(f"           - delAngBiasLearned = false  (gyro bias not converged, ~30s after boot)")
            print(f"           - gpsGoodToAlign went false again after origin set")
            print(f"         Check: XKF3.IYW (yaw innovation), XKF2.magX/Y/Z (compass), XKF4.TE (tilt error)")
        else:
            print(f"    [OK] const_pos_mode cleared promptly after origin set.")

    # ------------------------------------------------------------------ #
    # EKF flags OR-mask analysis during const_pos_mode                   #
    # ------------------------------------------------------------------ #
    flags_or = 0
    for t, flags, _ in ekf_series:
        if cpm_start <= t <= cpm_end_eff:
            flags_or |= flags

    print()
    print(f"  EKF flags seen (OR) while in const_pos_mode: 0x{flags_or:04x}")
    _INFERENCE = {
        0x0001: ("tiltAlignComplete likely true",       "tiltAlignComplete=false -- IMU not yet converged"),
        0x0002: ("horiz velocity available",             "no horiz velocity (expected in AID_NONE)"),
        0x0004: ("vert velocity available",              "no vert velocity"),
        0x0008: ("AID_RELATIVE entered (flow/odometry)", "AID_RELATIVE never entered -- no optical flow / body odometry"),
        0x0010: ("AID_ABSOLUTE entered (GPS nav OK)",    "AID_ABSOLUTE never entered -- EKF never used GPS"),
        0x0020: ("vert_pos valid",                       "vert_pos never valid (baro issue?)"),
        0x0100: ("rel-position predicted (flow ready)",  "pred_horiz_pos_rel never set -- no optical flow expected"),
        0x0200: ("GPS quality good (gpsGoodToAlign)",    "pred_horiz_pos_abs never set -- gpsGoodToAlign never sustained"),
        0x0400: ("EKF uninitialized during interval",    None),
        0x8000: ("GPS glitch active during interval",    None),
    }
    for mask, (if_set, if_not) in _INFERENCE.items():
        present = bool(flags_or & mask)
        name    = _EKF_FLAGS.get(mask, f"0x{mask:04x}")
        if present:
            print(f"    [set]   0x{mask:04x} {name:<22}  => {if_set}")
        elif if_not:
            marker = "[!!]" if mask in (0x0010, 0x0200) else "[  ]"
            print(f"    {marker} 0x{mask:04x} {name:<22}  => {if_not}")

    # ------------------------------------------------------------------ #
    # GPS quality check stability during const_pos_mode                  #
    # ------------------------------------------------------------------ #
    if gps_series:
        print()
        print(f"  GPS quality checks during const_pos_mode (EK3 default thresholds):")
        fail_counts: dict[str, int] = {f: 0 for f, *_ in _GPS_QUALITY_CHECKS}
        total_gps = 0
        for gt, gd in gps_series:
            if cpm_start <= gt <= cpm_end_eff:
                total_gps += 1
                for fname, thresh, _, _ in _GPS_QUALITY_CHECKS:
                    val = gd.get(fname, None)
                    if val is None:
                        continue
                    if fname == "satellites_visible":
                        if val < thresh:
                            fail_counts[fname] += 1
                    elif val > thresh:
                        fail_counts[fname] += 1
        if total_gps:
            for fname, _, _, label in _GPS_QUALITY_CHECKS:
                pct = 100.0 * fail_counts[fname] / total_gps
                flag = "[!!]" if pct > 50 else "[!] " if pct > 5 else "[OK]"
                print(f"    {flag} {label:<40}  failing {pct:5.1f}% of samples ({fail_counts[fname]}/{total_gps})")

    # ------------------------------------------------------------------ #
    # pos_horiz_variance trend (growing = drifting without aiding)       #
    # ------------------------------------------------------------------ #
    var_series = [(to_sim(d["_t_wall"], t0_wall, t0_boot),
                   d.get("pos_horiz_variance", 0.0))
                  for d in records
                  if d["mavpackettype"] == "EKF_STATUS_REPORT"]
    var_during = [(t, v) for t, v in var_series if cpm_start <= t <= cpm_end_eff]
    if var_during:
        v_start = var_during[0][1]
        v_end   = var_during[-1][1]
        v_max   = max(v for _, v in var_during)
        t_max   = next(t for t, v in var_during if v == v_max)
        print()
        print(f"  pos_horiz_variance during const_pos_mode:")
        print(f"    start={v_start:.3f}  end={v_end:.3f}  max={v_max:.3f} at t={t_max:.1f}s  (m^2)")
        if v_end > v_start * 2 or v_max > 2.0:
            print(f"    [!!] Variance growing -- position estimate is drifting (expected in AID_NONE)")
        else:
            print(f"    [OK] Variance stable")

    # ------------------------------------------------------------------ #
    # Relevant STATUSTEXT messages                                        #
    # ------------------------------------------------------------------ #
    _relevant = ("ekf", "gps", "ahrs", "imu", "origin", "arm", "accel", "gyro", "compass", "yaw")
    status_during = []
    for d in records:
        if d["mavpackettype"] != "STATUSTEXT":
            continue
        t   = to_sim(d["_t_wall"], t0_wall, t0_boot)
        txt = d.get("text", "").rstrip("\x00").strip()
        if cpm_start <= t <= cpm_end_eff and any(k in txt.lower() for k in _relevant):
            status_during.append((t, txt))
    status_during.sort(key=lambda x: x[0])
    if status_during:
        print()
        print(f"  Relevant STATUSTEXT messages during const_pos_mode:")
        for t, txt in status_during:
            print(f"    t={t:7.2f}s  {txt}")

    # ------------------------------------------------------------------ #
    # DataFlash analysis (when available)                                #
    # ------------------------------------------------------------------ #
    df_conclusions: dict = {}
    if df is not None:
        df_conclusions = _print_df_const_pos_analysis(df, t_stable_end=t_stable_end)

    # ------------------------------------------------------------------ #
    # Summary                                                             #
    # ------------------------------------------------------------------ #
    print()
    print(f"  SUMMARY:")
    if not never_cleared:
        print(f"    [OK] const_pos_mode cleared at t={cpm_end:.2f}s -- EKF left AID_NONE successfully.")
    elif df_conclusions.get("yaw_align_cause"):
        # DataFlash gave us a definitive answer
        print(f"    ROOT CAUSE (from DataFlash):")
        print(f"    {df_conclusions['yaw_align_cause']}")
    else:
        # MAVLink-only inference
        causes = []
        if first_good_gps_t is None:
            causes.append("GPS never got a fix -- check GPS antenna/wiring or EK3_SRC1_POSXY config")
        elif origin_t is None:
            causes.append("EKF origin never set -- GPS quality checks never passed the 10s window")
            causes.append("  Check h_acc, v_acc, vel_acc, HDOP, sats above (all must pass simultaneously)")
        else:
            if not (flags_or & 0x0200):
                causes.append("pred_horiz_pos_abs never set after origin: yawAlignComplete or delAngBiasLearned false")
                causes.append("  => Compass not calibrated, or GSF yaw estimator not converged")
                causes.append("  => Or gpsGoodToAlign went false again (GPS quality intermittent)")
                causes.append("  (Add dataflash.BIN to log dir for a definitive answer)")
            else:
                causes.append("GPS quality was intermittent -- gpsGoodToAlign oscillated without clearing const_pos")
        if not causes:
            causes.append("Could not determine cause from MAVLink data alone -- check DataFlash XKF4.GPS bitmask")
        for i, c in enumerate(causes, 1):
            marker = f"  [{i}]" if not c.startswith("  ") else "     "
            print(f"  {marker} {c}")


# ---------------------------------------------------------------------------
# Programmatic validation API
# ---------------------------------------------------------------------------

# How long after GPS fusion gps_glitching / GPS-Glitch STATUSTEXT are expected.
# When the EKF exits const_pos_mode and starts accepting GPS it has accumulated
# position uncertainty; the first measurements may exceed the innovation gate
# briefly before the filter settles.
_POST_FUSION_SETTLE_S = 5.0

# How long after GPS fusion const_pos_mode may still be set (clearing transient).
# EKF_STATUS_REPORT is logged at ~10 Hz; const_pos_mode clears within 1-3 frames
# (~0.1-0.3 s) of the 'is using GPS' STATUSTEXT.  Using 2 s is conservative.
_CONST_POS_SETTLE_S = 2.0


def _find_const_pos_clear_time(
    records: list[dict], t0_wall: float, t0_boot: float
) -> "float | None":
    """Return the last sim-time at which const_pos_mode (0x0080) cleared.

    This is the EKF-flags view of GPS fusion -- equivalent to the STATUSTEXT
    'EKF3 IMU0 is using GPS' but derived from EKF_STATUS_REPORT transitions.
    Returns None if const_pos_mode never appeared or never cleared.
    """
    t_clear: "float | None" = None
    prev = 0
    for d in records:
        if d["mavpackettype"] != "EKF_STATUS_REPORT":
            continue
        flags = d.get("flags", 0)
        if (prev & 0x0080) and not (flags & 0x0080):
            t_clear = to_sim(d["_t_wall"], t0_wall, t0_boot)
        prev = flags
    return t_clear


def _check_ekf_records(
    records: list[dict],
    t0_wall: float,
    t0_boot: float,
    t_start_s: float,
    t_end_s: float,
) -> list[str]:
    """Check EKF health in window [t_start_s, t_end_s] (boot-seconds).

    t_start_s is the GPS fusion time.  gps_glitching flag and related GPS Glitch
    STATUSTEXT within the first _POST_FUSION_SETTLE_S seconds of t_start_s are
    treated as a normal EKF settling transient (filter absorbing GPS after
    const_pos_mode exit) and do NOT produce failures.

    Returns a list of failure strings.  Empty = clean.
    """
    settle_end = t_start_s + _POST_FUSION_SETTLE_S

    failures:      list[str]   = []
    glitch_late:   list[float] = []   # gps_glitching after the settle window
    const_pos_times: list[float] = []
    att_loss_times:  list[float] = []

    const_pos_settle_end = t_start_s + _CONST_POS_SETTLE_S

    for d in records:
        st = to_sim(d["_t_wall"], t0_wall, t0_boot)
        if st < t_start_s or st > t_end_s:
            continue

        mt = d["mavpackettype"]

        if mt == "EKF_STATUS_REPORT":
            flags = d.get("flags", 0)
            if (flags & 0x8000) and st > settle_end:
                glitch_late.append(st)
            if (flags & 0x0080) and st > const_pos_settle_end:
                const_pos_times.append(st)
            if not (flags & 0x0001):
                att_loss_times.append(st)

        elif mt == "STATUSTEXT":
            txt = d.get("text", "").rstrip("\x00").strip()
            if "yaw reset" in txt.lower():
                failures.append(f"t={st:.1f}s  EKF yaw reset: {txt!r}")
            if st > settle_end and ("gps glitch" in txt.lower() or "compass error" in txt.lower()):
                failures.append(f"t={st:.1f}s  GPS/compass problem: {txt!r}")

    if glitch_late:
        t0f, t1f = glitch_late[0], glitch_late[-1]
        failures.append(
            f"gps_glitching (0x8000) active after {_POST_FUSION_SETTLE_S:.0f}s settle window: "
            f"{len(glitch_late)} frames  (first t={t0f:.1f}s, last t={t1f:.1f}s)"
        )
    if const_pos_times:
        t0f, t1f = const_pos_times[0], const_pos_times[-1]
        failures.append(
            f"const_pos_mode (0x0080) active for {len(const_pos_times)} "
            f"EKF frames in window  (first t={t0f:.1f}s, last t={t1f:.1f}s)"
        )
    if att_loss_times:
        t0f, t1f = att_loss_times[0], att_loss_times[-1]
        failures.append(
            f"attitude flag (0x0001) lost for {len(att_loss_times)} "
            f"EKF frames in window  (first t={t0f:.1f}s, last t={t1f:.1f}s)"
        )
    return failures


def validate_ekf_window(
    mavlink_path: "Path | str",
    t_start_s: float,
    t_end_s: float,
) -> list[str]:
    """Check EKF health in a sim-time window [t_start_s, t_end_s] (boot seconds).

    t_start_s should be the GPS fusion time (e.g. sim_now() when 'is using GPS'
    STATUSTEXT arrives).  gps_glitching and related STATUSTEXT within the first
    _POST_FUSION_SETTLE_S seconds are treated as a normal post-fusion transient.

    Returns a list of failure strings.  Empty = clean.

    Example
    -------
    issues = validate_ekf_window(ctx.mavlink_log, t_gps_fused_s, t_exit_s)
    assert not issues, "EKF problems after GPS fusion:\\n" + "\\n".join(issues)
    """
    mavlink_path = Path(mavlink_path)
    if mavlink_path.is_dir():
        mavlink_path = mavlink_path / "mavlink.jsonl"
    if not mavlink_path.exists():
        return [f"mavlink.jsonl not found: {mavlink_path}"]
    records = _load(mavlink_path)
    if not records:
        return ["mavlink.jsonl is empty"]
    _t0_wall, _t0_boot = _build_time_map(records)
    if _t0_wall is None or _t0_boot is None:
        return ["cannot determine time origin from mavlink.jsonl (no SYSTEM_TIME packet)"]
    return _check_ekf_records(records, _t0_wall, _t0_boot, t_start_s, t_end_s)


# ---------------------------------------------------------------------------
# Brief AI-optimised summary
# ---------------------------------------------------------------------------

def print_brief_diagnosis(
    records: list[dict],
    t0_wall: float,
    t0_boot: float,
    log_dir: "Path",
    crashes: list[dict],
    crash_t: "float | None",
    df: "dict | None",
    t_stable_end: "float | None",
) -> None:
    """~50-line targeted summary for AI diagnosis.  Skips verbose tables."""

    # ---- collect key STATUSTEXT and EKF events ----------------------------
    arm_t: float | None = None
    acro_t: float | None = None
    gps_fix_t: float | None = None
    origin_t: float | None = None
    yaw_align_t: float | None = None
    gps_fuse_t: float | None = None
    gps_glitch_t: float | None = None
    const_pos_start: float | None = None
    const_pos_end: float | None = None
    lua_capture_t: float | None = None
    lua_gps_t: float | None = None
    lua_gps_tlen: str = ""
    lua_gps_bz: str = ""

    key_statustext: list[tuple[float, str]] = []  # (t, text) for events we want to show
    prev_ekf = 0
    prev_armed = False

    for d in records:
        mt = d.get("mavpackettype", "")
        st = to_sim(d["_t_wall"], t0_wall, t0_boot)

        if mt == "HEARTBEAT":
            armed = bool(d.get("base_mode", 0) & _MAV_MODE_ARMED)
            mode  = d.get("custom_mode", 0)
            if armed and not prev_armed and arm_t is None:
                arm_t = st
            if armed and mode == 1 and acro_t is None:
                acro_t = st
            prev_armed = armed

        elif mt == "GPS_RAW_INT":
            if gps_fix_t is None and d.get("fix_type", 0) >= 3:
                gps_fix_t = st

        elif mt == "EKF_STATUS_REPORT":
            flags = d.get("flags", 0)
            was_cpm = bool(prev_ekf & 0x0080)
            is_cpm  = bool(flags   & 0x0080)
            if is_cpm and not was_cpm and const_pos_start is None:
                const_pos_start = st
            if not is_cpm and was_cpm and const_pos_end is None:
                const_pos_end = st
            prev_ekf = flags

        elif mt == "STATUSTEXT":
            txt = d.get("text", "").rstrip("\x00").strip()
            tl  = txt.lower()
            if "origin set" in tl:
                origin_t = st
                key_statustext.append((st, txt))
            elif "yaw aligned using gps" in tl:
                yaw_align_t = st
                key_statustext.append((st, txt))
            elif "is using gps" in tl:
                gps_fuse_t = st
                key_statustext.append((st, txt))
            elif "gps glitch" in tl or "compass error" in tl:
                if gps_glitch_t is None:
                    gps_glitch_t = st
                key_statustext.append((st, f"[!!] {txt}"))
            elif "rawes flight: captured" in tl:
                lua_capture_t = st
                key_statustext.append((st, txt))
            elif "rawes flight: gps" in tl:
                lua_gps_t = st
                # extract tlen and bz from message
                import re as _re
                m = _re.search(r"tlen=([\d.]+)", txt)
                if m:
                    lua_gps_tlen = m.group(1)
                m2 = _re.search(r"bz=\(([^)]+)\)", txt)
                if m2:
                    lua_gps_bz = m2.group(1)
                key_statustext.append((st, txt))
            elif "arming motors" in tl:
                key_statustext.append((st, txt))

    # ---- GSF summary from DataFlash ---------------------------------------
    gsf_line = "n/a (no DataFlash)"
    gyro_line = "n/a"
    df_root_cause = ""
    if df is not None:
        def _clip(lst: list) -> list:
            if t_stable_end is None:
                return lst
            return [r for r in lst if r[0] < t_stable_end]
        xky0 = _clip(df.get("xky0", []))
        if xky0:
            ycs_vals = [ycs for _, _, ycs in xky0]
            med_ycs  = sorted(ycs_vals)[len(ycs_vals)//2]
            min_ycs  = min(ycs_vals)
            yc_vals  = [yc for _, yc, _ in xky0]
            yc_range = max(yc_vals) - min(yc_vals)
            conv     = med_ycs < 3.0
            gsf_line = (
                f"[OK] converged (median YCS={med_ycs:.2f})" if conv
                else f"[!!] NEVER converged  median_YCS={med_ycs:.2f}  min={min_ycs:.2f}  YC_range={yc_range:.0f}deg"
            )
        xkf1 = _clip(df.get("xkf1", []))
        if xkf1:
            mags = [(gx**2+gy**2+gz**2)**0.5 for _, gx, gy, gz, _ in xkf1]
            med  = sorted(mags)[len(mags)//2]
            _, gx0,gy0,gz0,_ = xkf1[0]
            _, gxm,gym,gzm,_ = xkf1[-1]
            chg  = max(abs(gxm-gx0), abs(gym-gy0), abs(gzm-gz0))
            ok   = med < 1.0 and chg < 0.5
            gyro_line = (
                f"[OK] converged (median={med:.3f} deg/s)" if ok
                else f"[!!] large/not converged  median={med:.3f} deg/s  change={chg:.3f} deg/s"
            )
            if not ok:
                df_root_cause = "delAngBiasLearned=false: gyro bias too large or still drifting"
        if not df_root_cause:
            xky0_all = df.get("xky0", [])
            if xky0_all:
                ycs_all = [ycs for _, _, ycs in xky0_all]
                if sorted(ycs_all)[len(ycs_all)//2] >= 3.0:
                    df_root_cause = "yawAlignComplete=false: GSF yaw never converged (no compass, heading unresolved)"
            if not df_root_cause:
                df_root_cause = "all known blockers OK in DataFlash -- check GPS gate sizes"

    # ---- peak attitude rates (tumble detection) ---------------------------
    peak_rp = 0.0
    peak_rp_t = 0.0
    for d in records:
        if d["mavpackettype"] != "ATTITUDE":
            continue
        st = to_sim(d["_t_wall"], t0_wall, t0_boot)
        rp = max(abs(math.degrees(d.get("rollspeed", 0))),
                 abs(math.degrees(d.get("pitchspeed", 0))))
        if rp > peak_rp:
            peak_rp, peak_rp_t = rp, st

    # ---- print -------------------------------------------------------------
    print("=" * 70)
    print("  BRIEF DIAGNOSIS")
    print("=" * 70)

    print()
    print("KEY EVENTS:")
    if arm_t is not None:
        print(f"  t={arm_t:6.1f}s  armed")
    if acro_t is not None:
        print(f"  t={acro_t:6.1f}s  ACRO mode")
    if gps_fix_t is not None:
        print(f"  t={gps_fix_t:6.1f}s  GPS first good fix")
    if origin_t is not None:
        delay = (origin_t - gps_fix_t) if gps_fix_t else 0.0
        print(f"  t={origin_t:6.1f}s  EKF origin set  (+{delay:.1f}s after fix)")
    for t, txt in sorted(key_statustext, key=lambda x: x[0]):
        # skip ones we already printed above
        tl = txt.lower()
        if any(k in tl for k in ("origin set", "arming")):
            continue
        print(f"  t={t:6.1f}s  {txt}")
    if crash_t is not None:
        fn = crashes[0]["function"] if crashes else "unknown"
        print(f"  t={crash_t:6.1f}s  [!!] CRASH: SIGFPE in {fn}")

    print()
    print("GPS FUSION:")
    if const_pos_start is not None and const_pos_end is not None:
        total_blocked = const_pos_end - const_pos_start
        delay_after_fix = (const_pos_end - gps_fix_t) if gps_fix_t else 0.0
        print(f"  const_pos_mode: t={const_pos_start:.1f}s .. t={const_pos_end:.1f}s  ({total_blocked:.1f}s total)")
        print(f"  GPS fused at t={const_pos_end:.1f}s  ({delay_after_fix:.1f}s after first fix)")
    elif const_pos_start is not None:
        print(f"  [!!] const_pos_mode never cleared (started t={const_pos_start:.1f}s)")
    if gps_glitch_t is not None and gps_fuse_t is not None:
        gap = gps_glitch_t - gps_fuse_t
        print(f"  GPS glitch at t={gps_glitch_t:.1f}s  ({gap:.2f}s after fusion)  [!!]" if gap < 1.0
              else f"  GPS glitch at t={gps_glitch_t:.1f}s  ({gap:.1f}s after fusion)")

    print()
    print("EKF INTERNALS (DataFlash):")
    print(f"  GSF yaw:    {gsf_line}")
    print(f"  Gyro bias:  {gyro_line}")
    if df_root_cause:
        print(f"  Root cause: {df_root_cause}")

    if lua_gps_tlen:
        tlen_f = float(lua_gps_tlen)
        flag = "  [!!] expected ~100m" if tlen_f > 150 or tlen_f < 20 else ""
        print()
        print(f"LUA GPS CAPTURE:")
        print(f"  tlen={lua_gps_tlen} m{flag}")
        print(f"  bz=({lua_gps_bz})")

    # ---- first failure event: GPS glitch or SIGFPE, whichever is earlier ----
    # crash_t is tumble onset (consequence); the root failure is usually the
    # GPS glitch that precedes it.  Report the earliest failure event.
    first_failure_t: float | None = None
    first_failure_label = ""
    if gps_glitch_t is not None:
        first_failure_t = gps_glitch_t
        first_failure_label = "GPS glitch"
    if crash_t is not None:
        if first_failure_t is None or crash_t < first_failure_t:
            first_failure_t = crash_t
            first_failure_label = "SIGFPE/tumble"

    print()
    print("OUTCOME:")
    if first_failure_t is not None:
        fn = crashes[0]["function"] if crashes else None
        if gps_glitch_t is not None and crash_t is not None and gps_glitch_t <= crash_t:
            fn_str = f"  -> SIGFPE {crashes[0]['function']} at t={crash_t:.1f}s" if fn else ""
            print(f"  FAILURE start t={gps_glitch_t:.1f}s  (GPS glitch){fn_str}")
        elif crash_t is not None:
            fn_str = f" in {fn}" if fn else ""
            print(f"  CRASH at t={crash_t:.1f}s  (SIGFPE{fn_str})")
        print(f"  Peak attitude rate: {peak_rp:.0f} deg/s at t={peak_rp_t:.1f}s" +
              ("  [!!] tumbling" if peak_rp > 200 else "  [OK]"))
    else:
        print(f"  No failure detected  peak_rate={peak_rp:.0f} deg/s  [OK]")
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Summarise mavlink.jsonl from a stack test run.",
        epilog="Example: analyse_mavlink.py test_lua_flight_steady",
    )
    ap.add_argument("test", nargs="?",
                    help="Test name (in simulation/logs/) or absolute path to mavlink.jsonl")
    ap.add_argument("--full", action="store_true",
                    help="Print full verbose report (default is brief AI-optimised summary)")
    ap.add_argument("--ekf-all", action="store_true",
                    help="Print every EKF_STATUS_REPORT row, not just transitions (--full only)")
    ap.add_argument("--no-gps-detail",    action="store_true", help="Skip GPS timeline table")
    ap.add_argument("--no-simstate",      action="store_true", help="Skip GPS vs SIMSTATE section")
    ap.add_argument("--no-sensor-simstate", action="store_true", help="Skip sensor vs SIMSTATE section")
    ap.add_argument("--no-gps-consistency", action="store_true", help="Skip GPS physical consistency section")
    ap.add_argument("--no-attitude",      action="store_true", help="Skip attitude rate section")
    ap.add_argument("--no-variances",     action="store_true", help="Skip EKF variance section")
    ap.add_argument("--no-flight-state",  action="store_true", help="Skip HEARTBEAT flight-state section")
    ap.add_argument("--no-const-pos-diag", action="store_true", help="Skip const_pos_mode diagnosis section")
    ap.add_argument("--no-dataflash",      action="store_true", help="Skip DataFlash .BIN analysis even if file present")
    args = ap.parse_args()

    # -- Locate file ---------------------------------------------------------
    if args.test is None:
        # List available test directories
        dirs = sorted(_LOGS_DIR.iterdir()) if _LOGS_DIR.exists() else []
        print("Available test directories:")
        for d in dirs:
            if (d / "mavlink.jsonl").exists():
                print(f"  {d.name}")
        return 0

    candidate = Path(args.test)
    if candidate.is_file():
        mav_path = candidate
    elif candidate.is_dir() and (candidate / "mavlink.jsonl").exists():
        mav_path = candidate / "mavlink.jsonl"
    else:
        # Try under logs/
        mav_path = _LOGS_DIR / args.test / "mavlink.jsonl"

    if not mav_path.exists():
        print(f"ERROR: Cannot find mavlink.jsonl for '{args.test}'", file=sys.stderr)
        print(f"       Tried: {mav_path}", file=sys.stderr)
        return 1

    records = _load(mav_path)
    if not records:
        print("ERROR: No records loaded.", file=sys.stderr)
        return 1

    _tw, _tb = _build_time_map(records)
    t0_wall: float = _tw if _tw is not None else records[0]["_t_wall"]
    t0_boot: float = _tb if _tb is not None else 0.0

    # Crash info: read arducopter.log for SIGFPE, estimate time from tumble onset.
    # Always detect tumble onset (not just when a SIGFPE is present) so that the
    # stable-flight window is consistent across all analysis sections.
    _log_dir  = mav_path.parent
    _crashes  = _read_crash_info(_log_dir)
    _crash_t  = _find_tumble_onset(records, t0_wall, t0_boot)
    # Stable-flight end: 5 s before tumble onset (margin for pre-tumble noise).
    # Used to clip DataFlash series so post-crash data never enters convergence checks.
    _STABLE_MARGIN_S = 5.0
    _t_stable_end: "float | None" = (_crash_t - _STABLE_MARGIN_S) if _crash_t is not None else None

    # DataFlash .BIN (optional -- used for definitive EKF diagnosis)
    _df_data: "dict | None" = None
    if not args.no_dataflash:
        _df_data = load_dataflash(_log_dir)

    if not args.full:
        # Brief mode (default): ~50-line AI-optimised summary
        print(f"Log: {mav_path}")
        if _df_data is not None:
            print(f"DataFlash: {_df_data['path'].name}  ({sum(len(v) for _, v in _df_data.items() if isinstance(v, list))} EKF records)")
        print(f"Full report: analyse_mavlink.py {args.test} --full")
        print()
        print_brief_diagnosis(
            records, t0_wall, t0_boot,
            log_dir=_log_dir,
            crashes=_crashes,
            crash_t=_crash_t,
            df=_df_data,
            t_stable_end=_t_stable_end,
        )
        return 0

    # Full verbose report
    print(f"MAVLink log : {mav_path}")
    if _df_data is not None:
        print(f"DataFlash   : {_df_data['path'].name}  ({sum(len(v) for _, v in _df_data.items() if isinstance(v, list))} EKF records)")
    else:
        print("DataFlash   : not found (add dataflash.BIN for definitive EKF diagnosis)")

    print_inventory(records, t0_wall, t0_boot)
    if not args.no_flight_state:
        print_flight_state(records, t0_wall, t0_boot)
    print_timeline(records, t0_wall, t0_boot, crashes=_crashes, crash_t=_crash_t)
    print_gps(records, t0_wall, t0_boot, verbose=not args.no_gps_detail)
    print_ekf_flags(records, t0_wall, t0_boot, show_all=args.ekf_all)
    if not args.no_const_pos_diag:
        print_const_pos_diagnosis(records, t0_wall, t0_boot, df=_df_data,
                                  t_stable_end=_t_stable_end)
    if not args.no_simstate:
        print_gps_vs_simstate(records, t0_wall, t0_boot)
    if not args.no_sensor_simstate:
        print_sensor_vs_simstate(records, t0_wall, t0_boot)
    if not args.no_gps_consistency:
        print_gps_physical_consistency(records, t0_wall, t0_boot, t_stable_end=_t_stable_end)
    if not args.no_attitude:
        print_attitude(records, t0_wall, t0_boot)
    if not args.no_variances:
        print_variances(records, t0_wall, t0_boot)

    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
