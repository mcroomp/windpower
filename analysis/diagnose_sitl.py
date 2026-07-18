#!/usr/bin/env python3
"""
diagnose_sitl.py -- Mandatory first-look diagnosis after any SITL stack run.

Run this BEFORE making any decision about a SITL stack-test outcome.  It answers
the two questions that gate every downstream conclusion, in order:

  CHECK 1 -- EKF / GPS-capture readiness through the 60 s kinematic phase.
             Did we reach kinematic exit with a fully functional EKF, or were we
             gated (const_pos_mode / uninitialized / gps_glitching / missing GPS
             position flags / failsafe)?  Without a healthy EKF the Lua can never
             capture GPS, so nothing after this point is trustworthy.

  CHECK 2 -- Kinematic-exit state vs the IC (steady_state_starting.json).
             At kinematic exit, are we at the IC position, with the right disk
             tilt (body_z), and the right rotor RPM?  A bad hand-off makes the
             post-release flight failure a hand-off bug, not a controller bug.

If CHECK 1 fails, fix the EKF/GPS path first -- do not analyse the flight.
If CHECK 1 passes but CHECK 2 fails, fix the kinematic hand-off first.
Only when both pass is a post-release flight failure a real controller bug.

Usage:
  python analysis/diagnose_sitl.py test_lua_flight_steady_sitl
    .venv/Scripts/python.exe analysis/diagnose_sitl.py test_lua_flight_steady_sitl

With no test name, lists available test directories in simulation/logs/.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Optional

import simulation as _simulation_pkg
_SIM_DIR = Path(_simulation_pkg.__file__).resolve().parent  # simulation/
_LOG_DIR = _SIM_DIR / "logs"
_IC_PATH = _SIM_DIR / "steady_state_starting.json"

from simulation.ekf_flags import EKF_FLAGS, has_warn, decode_flags  # noqa: E402

from analysis.flight_log import FlightLog  # noqa: E402


# ---------------------------------------------------------------------------
# Thresholds
# ---------------------------------------------------------------------------

# EKF flags that must ALL be set before the Lua can capture GPS position.
#   attitude | horiz_vel | vert_vel | horiz_pos_rel | horiz_pos_abs | vert_pos
_CAPTURE_READY_MASK = 0x0001 | 0x0002 | 0x0004 | 0x0008 | 0x0010 | 0x0020

_KIN_PHASE_S = 60.0     # nominal kinematic-startup duration

# nav_filter_status bits (dataflash XKF4.SS).
# Source: ardupilot/libraries/AP_NavEKF/AP_Nav_Common.h
_NAV_STATUS_BITS: dict[int, str] = {
    0: "attitude",        1: "horiz_vel",       2: "vert_vel",
    3: "horiz_pos_rel",   4: "horiz_pos_abs",   5: "vert_pos",
    6: "terrain_alt",     7: "const_pos_mode",  8: "pred_horiz_pos_rel",
    9: "pred_horiz_pos_abs", 10: "takeoff_detected", 11: "takeoff",
    12: "touchdown",      13: "using_gps",       14: "gps_glitching",
    15: "gps_quality_good", 16: "initialized",   17: "rejecting_airspeed",
    18: "dead_reckoning",
}

# gps_check_status bits (dataflash XKF4.GPS) -- which pre-flight GPS check fails.
# Source: ardupilot/libraries/AP_NavEKF/AP_Nav_Common.h
_GPS_CHECK_BITS: dict[int, str] = {
    0: "bad_sAcc",  1: "bad_hAcc",       2: "bad_yaw",     3: "bad_sats",
    4: "bad_VZ",    5: "bad_horiz_drift", 6: "bad_hdop",   7: "bad_vert_vel",
    8: "bad_fix",   9: "bad_horiz_vel",  10: "bad_vAcc",
}

# AP_GPS fix-type names (dataflash GPS.Status).
_GPS_FIX_DF: dict[int, str] = {
    0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D",
    4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED",
}

# CHECK 2 pass thresholds (kinematic exit vs IC).
_POS_TOL_M       = 3.0   # position error magnitude [m]
_ALT_TOL_M       = 2.0   # altitude error [m]
_TILT_TOL_DEG    = 10.0  # body_z geodesic angle error [deg]
_RPM_TOL_RAD_S   = 2.0   # rotor speed error [rad/s]
_TENSION_TOL_N   = 75.0  # tether tension error [N]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _missing_flag_names(flags: int, mask: int) -> str:
    """Names of required flags that are NOT set in `flags`."""
    missing = [name for bit, name in EKF_FLAGS.items()
               if (mask & bit) and not (flags & bit)]
    return ", ".join(missing) if missing else "none"


def _warn_flag_names(flags: int) -> str:
    """Names of the set warning flags (const_pos_mode / uninitialized / glitch)."""
    warn = [name for bit, name in EKF_FLAGS.items()
            if (bit in (0x0080, 0x0400, 0x8000)) and (flags & bit)]
    return ", ".join(warn) if warn else "none"


def _is_ready(flags: int) -> bool:
    """True if EKF has every GPS-capture flag and no warning flag."""
    return (flags & _CAPTURE_READY_MASK) == _CAPTURE_READY_MASK and not has_warn(flags)


def _set_bit_names(value: int, table: dict[int, str]) -> list[str]:
    """Names of the bits set in `value` per `table` (ascending bit order)."""
    value &= 0xFFFFFFFF
    return [name for bit, name in sorted(table.items()) if value & (1 << bit)]


def _unit(v: tuple) -> tuple:
    n = math.sqrt(sum(c * c for c in v))
    if n == 0.0:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _angle_between_deg(a: tuple, b: tuple) -> float:
    ua, ub = _unit(a), _unit(b)
    dot = max(-1.0, min(1.0, sum(x * y for x, y in zip(ua, ub))))
    return math.degrees(math.acos(dot))


def _verdict(ok: bool) -> str:
    return "[PASS]" if ok else "[FAIL]"


# ---------------------------------------------------------------------------
# CHECK 1 -- EKF / GPS-capture readiness through the kinematic phase
# ---------------------------------------------------------------------------

def check_ekf_readiness(fl: FlightLog) -> bool:
    """Was the EKF healthy (GPS aiding) at the t=60s kinematic exit? If not, why?

    Everything after the kinematic exit is ignored -- the only question is
    whether the EKF reached full GPS-aiding health by the 60s release moment,
    because without it the Lua can never capture GPS. When it is not healthy,
    the dataflash EKF blocker chain pinpoints the broken link.
    """
    print("=" * 72)
    print("CHECK 1 -- EKF health at kinematic exit (60s); if unhealthy, why")
    print("=" * 72)

    # --- Kinematic exit time (the gate) --------------------------------------
    kin_exit_rows = [r for r in fl.tel_rows if r.note == "kinematic_exit"]
    if kin_exit_rows:
        t_gate = kin_exit_rows[0].t_sim
        reached = True
    else:
        t_gate = None
        for i in range(1, len(fl.tel_rows)):
            if fl.tel_rows[i - 1].damp_alpha > 0 and fl.tel_rows[i].damp_alpha == 0:
                t_gate = fl.tel_rows[i].t_sim
                break
        reached = t_gate is not None
    if t_gate is None:
        t_gate = _KIN_PHASE_S

    t_last = fl.tel_rows[-1].t_sim if fl.tel_rows else 0.0
    if not reached:
        print(f"  kinematic exit   : NOT REACHED  (telemetry ends at t={t_last:.1f}s)")
        print(f"                     gate forced to t={t_gate:.1f}s for EKF health check.")
    else:
        tag = "[OK]" if t_gate >= _KIN_PHASE_S - 1.0 else "[!!] early exit"
        print(f"  kinematic exit   : t={t_gate:.1f}s  {tag}")

    # --- EKF_STATUS_REPORT (MAVLink) at the gate -----------------------------
    prior = [r for r in fl._ekf_rows if r["t_sim"] <= t_gate + 0.5]
    mav_ready: Optional[bool] = None
    if prior:
        f_gate = prior[-1]["flags"]
        mav_ready = _is_ready(f_gate)
        tag = "capture-ready [OK]" if mav_ready else "NOT ready [!!]"
        print(f"  EKF flags @gate  : 0x{f_gate:04x} [{decode_flags(f_gate)}]  {tag}")
        if not mav_ready:
            print(f"    missing        : {_missing_flag_names(f_gate, _CAPTURE_READY_MASK)}")
            print(f"    warning        : {_warn_flag_names(f_gate)}")
    else:
        print("  EKF flags @gate  : no EKF_STATUS_REPORT in mavlink.jsonl")

    # --- Dataflash EKF blocker chain (authoritative why) ---------------------
    df_ok = _diagnose_ekf_blocker(fl.log_dir, t_gate)

    # Verdict: prefer the dataflash answer; fall back to MAVLink flags.
    if df_ok is None:
        ok = bool(reached and mav_ready)
    else:
        ok = bool(reached and df_ok)

    print(f"\n  CHECK 1 verdict  : {_verdict(ok)}  "
          + ("EKF healthy and GPS-aiding at the 60s gate."
             if ok else
             "EKF NOT healthy at the 60s gate -- GPS capture impossible. Fix this first."))
    return ok


def _read_ekf_dataflash(log_dir: Path, t_max: float) -> Optional[dict]:
    """Read GPS / ORGN / XKF4 / MSG up to t_max from dataflash.BIN.

    Returns None if pymavlink or the dataflash file is unavailable.
    Times are boot seconds relative to the first logged message; SITL is
    lockstep so this tracks sim time closely enough for the 60s gate.
    """
    path = log_dir / "dataflash.BIN"
    if not path.exists():
        return None
    try:
        from pymavlink import DFReader
    except ImportError:
        return None

    gps: list = []    # (t, status, nsats, hdop)
    orgn: list = []   # (t, type)
    xkf4: list = []   # (t, ss, gps_check, fs, ts)
    msgs: list = []   # (t, text) -- collected across the WHOLE log
    log = DFReader.DFReader_binary(str(path))
    msg = log.recv_msg()
    while msg is not None:
        tu = getattr(msg, "TimeUS", None)
        if tu is not None:
            # Dataflash TimeUS is ArduPilot boot-seconds. In lockstep SITL the
            # sim drives ArduPilot's clock, so boot-seconds == sim time (to within
            # the ~1 s ArduPilot boot before sim t=0). Do NOT re-zero to the first
            # message: GPS/EKF logging only starts ~14 s after boot, so subtracting
            # that first-message time would shift everything ~14 s earlier than the
            # sim-time gate and wrongly pull post-release data inside the window.
            t = tu / 1e6
            mt = msg.get_type()
            # Alignment/aiding messages can fire AFTER the gate; keep them all
            # so the gate breakdown can report when (if ever) a gate cleared.
            if mt == "MSG":
                txt = getattr(msg, "Message", "")
                if any(k in txt for k in ("EKF", "GPS", "origin", "yaw", "Yaw",
                                          "tilt", "aiding", "aligned")):
                    msgs.append((t, txt))
            elif t <= t_max + 0.5:
                if mt == "GPS":
                    gps.append((t, int(getattr(msg, "Status", 0)),
                                int(getattr(msg, "NSats", 0)),
                                float(getattr(msg, "HDop", 0.0))))
                elif mt == "ORGN":
                    orgn.append((t, int(getattr(msg, "Type", -1))))
                elif mt == "XKF4":
                    xkf4.append((t,
                                 int(getattr(msg, "SS", 0)) & 0xFFFFFFFF,
                                 int(getattr(msg, "GPS", 0)) & 0xFFFFFFFF,
                                 int(getattr(msg, "FS", 0)) & 0xFFFFFFFF,
                                 int(getattr(msg, "TS", 0)) & 0xFFFFFFFF))
        msg = log.recv_msg()

    return {"gps": gps, "orgn": orgn, "xkf4": xkf4, "msgs": msgs}


# EK3_SRC*_POSXY source enum (AP_NavEKF_Source::SourceXY).
_POSXY_SRC: dict[int, str] = {
    0: "None", 3: "GPS", 4: "Beacon", 6: "ExternalNav", 7: "WheelEncoder",
}
# EK3_SRC*_YAW source enum (AP_NavEKF_Source::SourceYaw).
_YAW_SRC: dict[int, str] = {
    0: "None", 1: "Compass", 2: "GPS", 3: "GPS_COMPASS_FALLBACK",
    6: "ExternalYaw", 8: "ExternalYaw_COMPASS_FALLBACK",
}


def _load_params(log_dir: Path) -> dict:
    """Load params.json from the log dir; returns {} if absent/unreadable."""
    p = log_dir / "params.json"
    if not p.exists():
        return {}
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except (json.JSONDecodeError, OSError):
        return {}


def _const_pos_exit_gates(df: dict, params: dict, t_gate: float) -> None:
    """Print every readyToUseGPS() gate the EKF must clear to leave const_pos_mode.

    Mirrors NavEKF3_core::readyToUseGPS() (AP_NavEKF3_Control.cpp): the EKF stays
    in AID_NONE / const_pos_mode until ALL of these are satisfied. Each gate is
    scored from the best available dataflash/param evidence at the 60s gate.
    """
    xkf4 = df["xkf4"]
    gate_xkf4 = next((r for r in reversed(xkf4) if r[0] <= t_gate), None)
    ss = gate_xkf4[1] if gate_xkf4 else 0
    gps_check = gate_xkf4[2] if gate_xkf4 else None
    msgs = df["msgs"]

    def first_msg(*subs: str):
        for (t, txt) in msgs:
            if any(s in txt for s in subs):
                return (t, txt)
        return None

    def tf(ok: bool) -> str:
        return "OK" if ok else "!!"

    print("    -- readyToUseGPS() gates to leave const_pos_mode (AID_NONE) --")

    # 1. PosXY source must be GPS.
    posxy = params.get("EK3_SRC1_POSXY")
    g1 = (posxy == 3.0)
    pname = _POSXY_SRC.get(int(posxy), str(posxy)) if posxy is not None else "unknown"
    print(f"      [{tf(g1)}] PosXY source = GPS   : EK3_SRC1_POSXY={pname}")

    # 2. validOrigin.
    orgn0 = next(((t, ty) for (t, ty) in df["orgn"] if ty == 0), None)
    g2 = orgn0 is not None and orgn0[0] <= t_gate
    print(f"      [{tf(g2)}] validOrigin          : "
          + (f"set t={orgn0[0]:.1f}s" if orgn0 else "never set"))

    # 3. tiltAlignComplete (attitude valid; tiltErrVar < (5 deg)^2).
    tilt = first_msg("tilt alignment")
    attitude_bit = bool(ss & 0x1)
    g3 = (tilt is not None and tilt[0] <= t_gate) or attitude_bit
    print(f"      [{tf(g3)}] tiltAlignComplete    : "
          + (f"msg t={tilt[0]:.1f}s" if tilt else f"attitude_bit={int(attitude_bit)}"))

    # 4. yawAlignComplete -- with EK3_SRC1_YAW=GPS this needs GPS-yaw (RELPOSNED).
    yaw_src = params.get("EK3_SRC1_YAW")
    yname = _YAW_SRC.get(int(yaw_src), str(yaw_src)) if yaw_src is not None else "unknown"
    yaw = first_msg("yaw aligned", "yaw alignment")
    g4 = yaw is not None and yaw[0] <= t_gate
    if yaw is None:
        ydetail = f"NO yaw-align msg (yaw src={yname})"
    elif yaw[0] <= t_gate:
        ydetail = f"aligned t={yaw[0]:.1f}s (src={yname})"
    else:
        ydetail = f"NOT before gate; first align t={yaw[0]:.1f}s AFTER gate (src={yname})"
    print(f"      [{tf(g4)}] yawAlignComplete     : {ydetail}")

    # 5. delAngBiasLearned -- gyro bias converged. Not directly logged.
    print("      [ ?] delAngBiasLearned     : not logged (gyro bias; usually clears fast)")

    # 6. gpsGoodToAlign -- GPS quality checks pass + gps_quality_good bit.
    g6 = bool(ss & (1 << 15)) and (gps_check == 0)
    print(f"      [{tf(g6)}] gpsGoodToAlign       : "
          f"gps_quality_good={int(bool(ss & (1 << 15)))} "
          f"gps_check=0x{(gps_check or 0):x}")

    # 7. gpsDataToFuse -- fresh GPS data near the gate.
    near = [t for (t, s, n, h) in df["gps"] if t <= t_gate]
    g7 = bool(near) and (t_gate - near[-1] <= 1.0)
    print(f"      [{tf(g7)}] gpsDataToFuse        : "
          + (f"last GPS sample t={near[-1]:.1f}s" if near else "no GPS samples"))

    # Blocking gate(s).
    blocking = []
    if not g1: blocking.append("PosXY source != GPS")
    if not g2: blocking.append("origin not set")
    if not g3: blocking.append("tilt not aligned")
    if not g4: blocking.append("yaw not aligned")
    if not g6: blocking.append("GPS quality checks")
    if not g7: blocking.append("no fresh GPS data")
    if blocking:
        print("      => BLOCKING gate(s): " + ", ".join(blocking))
    using = first_msg("is using GPS")
    if using:
        late = "  [AFTER 60s gate]" if using[0] > t_gate else ""
        print(f"      => AID_ABSOLUTE engaged at t={using[0]:.1f}s (EKF is-using-GPS){late}")


def _diagnose_ekf_blocker(log_dir: Path, t_gate: float) -> Optional[bool]:
    """Print the dataflash EKF blocker chain at t_gate. Returns health bool.

    Walks the GPS -> checks -> origin -> aiding dependency in order and flags
    the FIRST broken link. Returns:
      True  -- EKF using GPS, out of const_pos_mode, horiz_pos_abs valid
      False -- some link broken (reason printed)
      None  -- dataflash unavailable (caller falls back to MAVLink flags)
    """
    df = _read_ekf_dataflash(log_dir, t_gate)
    if df is None:
        print("  EKF blocker chain: dataflash.BIN unavailable (pymavlink/file missing)")
        return None

    print(f"  -- EKF blocker chain (dataflash, t<={t_gate:.0f}s) --")

    # Link 1 -- GPS 3D fix.
    gps = df["gps"]
    fix3d = next(((t, s, n, h) for (t, s, n, h) in gps if s >= 3), None)
    gate_gps = gps[-1] if gps else None
    link1 = fix3d is not None
    if link1:
        ft = fix3d[0]
        if gate_gps:
            gname = _GPS_FIX_DF.get(gate_gps[1], str(gate_gps[1]))
            gsats, ghdop = gate_gps[2], gate_gps[3]
        else:
            gname, gsats, ghdop = "?", 0, 0.0
        print(f"    1. GPS 3D fix  : t={ft:.1f}s  @gate fix={gname} sats={gsats} "
              f"hdop={ghdop:.2f}   [OK]")
    else:
        print("    1. GPS 3D fix  : NEVER reached 3D fix   [!!]")

    # Link 2 -- GPS pre-arm checks (XKF4.GPS bitmask).
    xkf4 = df["xkf4"]
    gate_xkf4 = next((r for r in reversed(xkf4) if r[0] <= t_gate), None)
    link2 = False
    if gate_xkf4 is not None:
        gps_check = gate_xkf4[2]
        if gps_check == 0:
            link2 = True
            print("    2. GPS checks  : all pass (0x0)   [OK]")
        else:
            fails = ", ".join(_set_bit_names(gps_check, _GPS_CHECK_BITS))
            print(f"    2. GPS checks  : FAILING 0x{gps_check:x} ({fails})   [!!]")
    else:
        print("    2. GPS checks  : no XKF4 records   [!!]")

    # Link 3 -- EKF origin set (ORGN Type==0).
    orgn0 = next(((t, ty) for (t, ty) in df["orgn"] if ty == 0), None)
    link3 = orgn0 is not None
    if link3:
        print(f"    3. EKF origin  : set at t={orgn0[0]:.1f}s   [OK]")
    else:
        print("    3. EKF origin  : NEVER set   [!!]")

    # Link 4 -- nav filter status (XKF4.SS) at the gate.
    link4 = False
    using_gps = const_pos = horiz_abs = glitch = False
    if gate_xkf4 is not None:
        ss = gate_xkf4[1]
        using_gps = bool(ss & (1 << 13))
        const_pos = bool(ss & (1 << 7))
        horiz_abs = bool(ss & (1 << 4))
        glitch    = bool(ss & (1 << 14))
        init      = bool(ss & (1 << 16))
        gqual     = bool(ss & (1 << 15))
        link4 = using_gps and horiz_abs and not const_pos and not glitch
        summary = (f"init={int(init)} gps_good={int(gqual)} using_gps={int(using_gps)} "
                   f"const_pos={int(const_pos)} horiz_pos_abs={int(horiz_abs)} "
                   f"glitch={int(glitch)}")
        tag = "[OK]" if link4 else "[!!]"
        print(f"    4. nav status  : 0x{ss:05x}  {summary}  {tag}")
    else:
        print("    4. nav status  : no XKF4 records   [!!]")

    # First broken link -> the actual blocker.
    reason: Optional[str] = None
    if not link1:
        reason = "GPS never achieved a 3D fix"
    elif not link2:
        reason = "GPS pre-arm quality checks failing"
    elif not link3:
        reason = "EKF origin never set"
    elif not link4:
        if const_pos and not using_gps:
            reason = ("EKF had healthy GPS + origin but never left const_pos_mode "
                      "(AID_NONE) -- GPS aiding never started; check EK3_SRC posxy/yaw "
                      "source and GPS-yaw (RELPOSNED) alignment")
        elif glitch:
            reason = "EKF flagged gps_glitching at the gate"
        else:
            reason = "EKF horizontal position estimate not valid at the gate"

    if reason:
        print(f"    -> blocker     : {reason}")

    # When stuck in const_pos_mode (AID_NONE), enumerate EVERY readyToUseGPS()
    # gate so the exact blocking condition(s) are visible.
    if const_pos and not using_gps:
        _const_pos_exit_gates(df, _load_params(log_dir), t_gate)

    # EKF/GPS text messages inside the window (often name the exact gate).
    if df["msgs"]:
        print("    EKF/GPS msgs   :")
        for (t, txt) in df["msgs"][:12]:
            print(f"      t={t:5.1f}s  {txt}")
        if len(df["msgs"]) > 12:
            print(f"      ... and {len(df['msgs']) - 12} more")

    return link1 and link2 and link3 and link4


# ---------------------------------------------------------------------------
# CHECK 2 -- Kinematic-exit state vs the IC
# ---------------------------------------------------------------------------

def _load_ic(ic_path: Path) -> Optional[dict]:
    if not ic_path.exists():
        return None
    try:
        return json.loads(ic_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None


def check_kinematic_exit_vs_ic(fl: FlightLog, ic_path: Path) -> bool:
    """At kinematic exit, are we at the IC position, tilt, and RPM?"""
    print()
    print("=" * 72)
    print("CHECK 2 -- Kinematic-exit state vs IC (steady_state_starting.json)")
    print("=" * 72)

    ic = _load_ic(ic_path)
    if ic is None:
        print(f"  IC file          : NOT FOUND / unreadable at {ic_path}")
        print(f"\n  CHECK 2 verdict  : {_verdict(False)}  (no IC reference)")
        return False

    kin_exit_rows = [r for r in fl.tel_rows if r.note == "kinematic_exit"]
    tr = kin_exit_rows[0] if kin_exit_rows else None
    if tr is None:
        # Fall back to first free-flight row.
        free = [r for r in fl.tel_rows if r.damp_alpha == 0.0]
        tr = free[0] if free else None
    if tr is None:
        print("  kinematic exit   : NO free-flight row -- kite never released")
        print(f"\n  CHECK 2 verdict  : {_verdict(False)}  (no kinematic exit)")
        return False

    # --- IC references --------------------------------------------------------
    ic_pos = ic["pos"]                       # anchor-origin NED [m]
    ic_alt = -ic_pos[2]
    R0 = ic["R0"]                            # body->NED (steady IC)
    ic_bz = (R0[0][2], R0[1][2], R0[2][2])  # body_z column in NED
    ic_omega = ic["omega_spin"]
    ic_tension = ic.get("tension_eq_n", float("nan"))

    # --- Exit state -----------------------------------------------------------
    ex_pos = (tr.pos_x, tr.pos_y, tr.pos_z)
    ex_alt = -tr.pos_z
    ex_bz = (tr.r02, tr.r12, tr.r22)        # body_z column in NED
    ex_omega = tr.omega_rotor
    ex_tension = tr.tether_tension

    # --- Errors ---------------------------------------------------------------
    pos_err = math.sqrt(sum((a - b) ** 2 for a, b in zip(ex_pos, ic_pos)))
    alt_err = abs(ex_alt - ic_alt)
    tilt_err = _angle_between_deg(ex_bz, ic_bz)
    rpm_err = abs(ex_omega - ic_omega)
    tension_err = (abs(ex_tension - ic_tension)
                   if not math.isnan(ic_tension) else float("nan"))

    pos_ok = pos_err <= _POS_TOL_M
    alt_ok = alt_err <= _ALT_TOL_M
    tilt_ok = tilt_err <= _TILT_TOL_DEG
    rpm_ok = rpm_err <= _RPM_TOL_RAD_S
    tension_ok = (math.isnan(tension_err)) or (tension_err <= _TENSION_TOL_N)

    print(f"  exit time        : t={tr.t_sim:.1f}s")
    print(f"  {'quantity':<16}{'exit':>14}{'IC':>14}{'error':>12}   verdict")
    print(f"  {'-'*16}{'-'*14:>14}{'-'*14:>14}{'-'*12:>12}   -------")
    print(f"  {'position (m)':<16}"
          f"{f'[{ex_pos[0]:.1f},{ex_pos[1]:.1f},{ex_pos[2]:.1f}]':>14}"
          f"{f'[{ic_pos[0]:.1f},{ic_pos[1]:.1f},{ic_pos[2]:.1f}]':>14}"
          f"{pos_err:>10.2f} m   {'[OK]' if pos_ok else '[!!]'}")
    print(f"  {'altitude (m)':<16}{ex_alt:>14.2f}{ic_alt:>14.2f}"
          f"{alt_err:>10.2f} m   {'[OK]' if alt_ok else '[!!]'}")
    print(f"  {'body_z tilt':<16}"
          f"{f'[{ex_bz[0]:.2f},{ex_bz[1]:.2f},{ex_bz[2]:.2f}]':>14}"
          f"{f'[{ic_bz[0]:.2f},{ic_bz[1]:.2f},{ic_bz[2]:.2f}]':>14}"
          f"{tilt_err:>8.2f} deg   {'[OK]' if tilt_ok else '[!!]'}")
    print(f"  {'rotor (rad/s)':<16}{ex_omega:>14.2f}{ic_omega:>14.2f}"
          f"{rpm_err:>8.2f} r/s   {'[OK]' if rpm_ok else '[!!]'}")
    if not math.isnan(ic_tension):
        print(f"  {'tension (N)':<16}{ex_tension:>14.1f}{ic_tension:>14.1f}"
              f"{tension_err:>10.1f} N   {'[OK]' if tension_ok else '[!!]'}")
    else:
        print(f"  {'tension (N)':<16}{ex_tension:>14.1f}{'n/a':>14}"
              f"{'n/a':>12}   [--]")

    ok = pos_ok and alt_ok and tilt_ok and rpm_ok and tension_ok
    print(f"\n  CHECK 2 verdict  : {_verdict(ok)}  "
          + ("kite handed off at the IC state."
             if ok else
             "hand-off state deviates from IC -- fix the kinematic hand-off first."))
    return ok


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _list_test_dirs() -> list:
    if not _LOG_DIR.exists():
        return []
    dirs = [d for d in _LOG_DIR.iterdir()
            if d.is_dir() and (d / "telemetry.csv").exists()]
    return sorted(dirs, key=lambda d: d.stat().st_mtime, reverse=True)


def diagnose(log_dir: Path, ic_path: Path) -> bool:
    """Run both checks against a log directory. Returns True if BOTH pass."""
    fl = FlightLog.load(log_dir)
    print(f"\nPost-run SITL diagnosis: {log_dir.name}")
    print(f"  telemetry rows   : {len(fl.tel_rows)}")
    if fl.tel_rows:
        print(f"  sim time span    : {fl.tel_rows[0].t_sim:.1f} .. "
              f"{fl.tel_rows[-1].t_sim:.1f} s")
    print()

    ok1 = check_ekf_readiness(fl)
    ok2 = check_kinematic_exit_vs_ic(fl, ic_path)

    print()
    print("=" * 72)
    print("SUMMARY")
    print("=" * 72)
    print(f"  CHECK 1 (EKF health at 60s gate)        : {_verdict(ok1)}")
    print(f"  CHECK 2 (kinematic exit vs IC)          : {_verdict(ok2)}")
    if not ok1:
        print("\n  -> Fix the EKF/GPS path first. Without a healthy EKF the Lua")
        print("     cannot capture GPS, so the post-release flight is meaningless.")
    elif not ok2:
        print("\n  -> EKF is healthy, but the kinematic hand-off is off-IC.")
        print("     Fix the hand-off before blaming the flight controller.")
    else:
        print("\n  -> Both gates pass. A post-release flight failure is a real")
        print("     controller/physics bug; proceed to analyse_run.py.")
    return ok1 and ok2


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("test_name", nargs="?", default=None,
                        help="Test directory name under simulation/logs/.")
    parser.add_argument("--log-dir", default=None,
                        help="Explicit log directory (overrides test_name).")
    parser.add_argument("--ic", default=None,
                        help="Path to the IC JSON (default: "
                             "simulation/steady_state_starting.json).")
    args = parser.parse_args()

    ic_path = Path(args.ic) if args.ic else _IC_PATH

    if args.log_dir:
        log_dir = Path(args.log_dir)
    elif args.test_name:
        log_dir = _LOG_DIR / args.test_name
    else:
        dirs = _list_test_dirs()
        if not dirs:
            print(f"No test directories with telemetry found in {_LOG_DIR}")
        else:
            print(f"Available SITL runs in {_LOG_DIR}  (newest first):")
            for d in dirs:
                print(f"  {d.name}")
            print("\nUsage: python diagnose_sitl.py <test_name>")
        sys.exit(2)

    if not log_dir.exists():
        print(f"Log directory not found: {log_dir}")
        sys.exit(2)

    ok = diagnose(log_dir, ic_path)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
