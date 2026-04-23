#!/usr/bin/env python3
"""
analyse_run.py -- Post-run analysis of a stack test run.

Reads the per-test log directory written by every stack test and prints a
structured report covering:

  * Damping phase: position drift, min altitude, velocity convergence
  * EKF timeline: when GPS locked, origin set, any failsafes
  * Setup steps: which of the 6 setup steps completed and when
  * Hold results: position / attitude / servo sample counts, drift, altitude
  * Test outcomes: PASSED / FAILED per test

Each stack test writes its logs to simulation/logs/{test_name}/:
  telemetry.csv   -- physics frames (TelRow CSV, primary source for this script)
  mediator.log    -- run metadata (RUN_ID, sensor mode, damping config)
  gcs.log         -- GCS / MAVLink events (STATUSTEXT, EKF flags, setup steps)

Usage:
  bash sim.sh exec 'python3 /rawes/simulation/analysis/analyse_run.py test_acro_armed'
  bash sim.sh exec 'python3 /rawes/simulation/analysis/analyse_run.py test_pumping_cycle --plot'

With no test name, lists available test directories in simulation/logs/.
"""

from __future__ import annotations

import argparse
import math
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

_SIM_DIR = Path(__file__).resolve().parents[1]   # simulation/

_LOG_DIR = _SIM_DIR / "logs"

sys.path.insert(0, str(_SIM_DIR))
from telemetry_csv import TelRow, read_csv  # noqa: E402
from mavlink_log import iter_messages as _iter_mavlink  # noqa: E402
from ekf_flags import has_warn as _ekf_has_warn  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from flight_log import FlightLog  # noqa: E402


# ---------------------------------------------------------------------------
# Public API — importable from tests
# ---------------------------------------------------------------------------

@dataclass
class SteadyMetrics:
    """Physics-based stability metrics for steady-orbit assessment.

    All values are derived from the telemetry CSV (physics ground truth), not
    from EKF estimates.  Computed by compute_steady_metrics().
    """
    min_phys_alt:     float   # m   minimum altitude in free-flight
    max_stable_s:     float   # s   longest continuous window >= stable_alt_m (sim time)
    floor_hits:       int     # #   frames at altitude <= 1.05 m
    mean_alt:         float   # m   mean free-flight altitude
    max_tension:      float   # N   peak tether tension in free-flight
    free_flight_rows: int     # #   telemetry rows in free-flight phase


def compute_steady_metrics(
    tel_rows: list,
    stable_alt_m: float = 3.0,
) -> SteadyMetrics:
    """Compute stability metrics from TelRow list (physics ground truth).

    Considers only free-flight rows (damp_alpha == 0).  The stable window is
    measured in simulation seconds so it is independent of wall-clock timing.

    Args:
        tel_rows:     list[TelRow] from read_csv() or load_telemetry()
        stable_alt_m: altitude floor for the continuous-stable window [m]

    Returns:
        SteadyMetrics with physics-based pass/fail values.
    """
    free_rows = [r for r in tel_rows if r.damp_alpha == 0.0]
    if not free_rows:
        return SteadyMetrics(
            min_phys_alt=0.0, max_stable_s=0.0, floor_hits=0,
            mean_alt=0.0, max_tension=0.0, free_flight_rows=0,
        )

    alts     = [-r.pos_z           for r in free_rows]
    tensions = [r.tether_tension   for r in free_rows]

    min_phys_alt = min(alts)
    floor_hits   = sum(1 for a in alts if a <= 1.05)
    mean_alt     = sum(alts) / len(alts)
    max_tension  = max(tensions)

    # Longest continuous window above stable_alt_m (sim seconds).
    max_stable_s = 0.0
    win_start_t: Optional[float] = None
    for r, alt in zip(free_rows, alts):
        if alt >= stable_alt_m:
            if win_start_t is None:
                win_start_t = r.t_sim
            else:
                max_stable_s = max(max_stable_s, r.t_sim - win_start_t)
        else:
            win_start_t = None
    if win_start_t is not None:
        max_stable_s = max(max_stable_s, free_rows[-1].t_sim - win_start_t)

    return SteadyMetrics(
        min_phys_alt=min_phys_alt,
        max_stable_s=max_stable_s,
        floor_hits=floor_hits,
        mean_alt=mean_alt,
        max_tension=max_tension,
        free_flight_rows=len(free_rows),
    )


def print_flight_report(log_dir: Path, bucket_s: float = 5.0) -> None:
    """Load logs from log_dir and print the full flight analysis report.

    Produces a unified per-bucket analysis covering physics, attitude tracking,
    EKF health, and important events (GPS fusion, EKF transitions, arming).
    bucket_s controls the time-window size: larger = higher-level overview,
    smaller = fine-grained debugging.

    Called from stack tests after the observation window.
    """
    log_dir  = Path(log_dir)
    ekf_path = log_dir / "mavlink.jsonl"

    # Load FlightLog (all sources)
    fl = FlightLog.load(log_dir)

    # Also build RunReport for sections that still use it
    report = RunReport()
    load_telemetry(log_dir / "telemetry.csv", report)
    parse_mediator(log_dir / "mediator.log", report)
    parse_pytest(log_dir / "gcs.log", report)
    parse_arducopter(log_dir / "arducopter.log", report)

    _print_mediator(report, fl)
    _print_sitl_crash(report)
    _print_tel_events(report)
    _print_bucketed_report(fl, bucket_s)
    _print_sensor_consistency(report)
    _print_yaw_divergence(report)
    _print_ekf_state(ekf_path)
    _print_ekf(report)
    _print_landing(report)
    _print_setup(report)
    _print_hold(report)
    _print_results(report)


# Keep the old name as an alias so existing callers don't break.
print_steady_report = print_flight_report


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class EkfEvent:
    timestamp: str
    t_wall:    float   # seconds-of-day from log timestamp
    text:      str


@dataclass
class SitlCrash:
    """ArduPilot crash detected in arducopter.log."""
    error_line: str          # the "ERROR: ..." line
    stack:      list         # list of '#N  <frame>' strings from dumpstack output


@dataclass
class RunReport:
    tel_rows:         list          = field(default_factory=list)    # list[TelRow]
    statustext:       list          = field(default_factory=list)    # (ts, text)
    ekf_flags:        list          = field(default_factory=list)    # (ts, flags)
    setup_steps:      list          = field(default_factory=list)    # (ts, msg)
    test_results:     list          = field(default_factory=list)    # (name, PASSED/FAILED)
    hold_stats:       dict          = field(default_factory=dict)
    sensor_mode:      Optional[str] = None
    damp_T:           Optional[float] = None
    k_ang:            Optional[float] = None
    mediator_run_id:  Optional[int]   = None
    pytest_run_id:    Optional[int]   = None
    kin_launch_pos:   Optional[tuple] = None   # (x, y, z) NED [m]
    kin_vel:          Optional[tuple] = None   # (vx, vy, vz) [m/s]
    sitl_crash:       Optional[SitlCrash] = None
    # Timing anchors for crash localisation
    mediator_t0_wall_s:    Optional[float] = None  # wall-sec of mediator t=0 (first log line)
    gcs_last_ap_wall_s:    Optional[float] = None  # wall-sec of last AP-originated GCS message
    gcs_obs_start_wall_s:  Optional[float] = None  # wall-sec of "observing X s of free flight"
    gcs_last_obs_t_s:      Optional[float] = None  # last obs-loop heartbeat sim-offset (e.g. 20s)


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

_RE_RUN_ID      = re.compile(r"RUN_ID=(\d+)")
# Two STATUSTEXT formats in gcs.log:
#   fixture logger:  STATUSTEXT [sev=6]: <text>
#   test logger:     STATUSTEXT [t=1.7s]: <text>
_RE_STATUSTEXT  = re.compile(r"STATUSTEXT \[(?:sev=\d+|t=[\d.]+s)\]:\s*(.+)$")
_RE_STATUSTEXT_T = re.compile(r"STATUSTEXT \[t=([\d.]+)s\]:\s*(.+)$")
_RE_EKF_FLAGS   = re.compile(r"EKF_STATUS\s+flags=(0x[0-9a-fA-F]+)")
_RE_SETUP_STEP  = re.compile(r"\[setup\s+\d+/\d+\]\s+(.+)$")
_RE_HOLD_STAT   = re.compile(r"Hold complete\.\s+pos=(\d+)\s+att=(\d+)\s+servo=(\d+)")
_RE_PASS        = re.compile(r"(PASSED|FAILED)\s+(tests/\S+)")
_RE_SENSOR_MODE = re.compile(r"Sensor mode:\s+(\S+)")
_RE_DAMP_CFG    = re.compile(r"Startup damping:.*?T=([\d.]+)s.*?k_ang=([\d.]+)")
_RE_KIN_START   = re.compile(
    r"Kinematic startup:.*?T=([\d.]+)s\s+"
    r"launch_pos=\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]\s+"
    r"vel=\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]"
)
_RE_TIMESTAMP   = re.compile(r"^(\d+:\d+:\d+)")
# Lines in gcs.log that prove ArduPilot was alive (heartbeat or STATUSTEXT received).
# Covers all logger formats: [sev=6], [t=Xs], [post-kin drain], [drain], etc.
_RE_AP_ALIVE    = re.compile(
    r"HEARTBEAT: sysid=1|"
    r"STATUSTEXT[^:]*:|"
    r"EKF_STATUS flags="
)
# Observation-loop heartbeat line: "t=20s  max_activity=667 PWM ..."
_RE_OBS_TICK    = re.compile(r"\bt=(\d+)s\s+max_activity=\d+\s+PWM")
# Start of free-flight observation
_RE_OBS_START   = re.compile(r"observing [\d.]+ s of free flight")


def _wall_seconds(ts: str) -> float:
    """Convert 'HH:MM:SS' log timestamp to seconds-of-day."""
    parts = ts.split(":")
    if len(parts) == 3:
        return int(parts[0]) * 3600 + int(parts[1]) * 60 + int(parts[2])
    return 0.0


def load_telemetry(path: Path, report: RunReport) -> None:
    """Load TelRow list from the mediator telemetry CSV."""
    if not path.exists():
        print(f"  [!] telemetry CSV not found: {path}")
        return
    report.tel_rows = read_csv(path)


def parse_mediator(path: Path, report: RunReport) -> None:
    """Extract run metadata from the mediator log and events.jsonl (RUN_ID, sensor mode, damping config).

    Physics frames are no longer read here -- use load_telemetry() instead.
    Transition state is computed from the telemetry CSV in _print_mediator().
    RUN_ID is read from events.jsonl (startup event) if not present in the prose log.
    """
    if not path.exists():
        print(f"  [!] mediator log not found: {path}")
    else:
        for line in path.read_text(errors="replace").splitlines():
            ts_m = _RE_TIMESTAMP.match(line)
            if ts_m and report.mediator_t0_wall_s is None:
                report.mediator_t0_wall_s = _wall_seconds(ts_m.group(1))
            if m := _RE_RUN_ID.search(line):
                report.mediator_run_id = int(m.group(1))
            if m := _RE_SENSOR_MODE.search(line):
                report.sensor_mode = m.group(1)
            if m := _RE_DAMP_CFG.search(line):
                report.damp_T = float(m.group(1))
                report.k_ang  = float(m.group(2))
            if m := _RE_KIN_START.search(line):
                report.kin_launch_pos = (float(m.group(2)), float(m.group(3)), float(m.group(4)))
                report.kin_vel        = (float(m.group(5)), float(m.group(6)), float(m.group(7)))

    # Fall back to events.jsonl for RUN_ID (structured log, replaces prose log.info for metadata)
    if report.mediator_run_id is None:
        events_path = path.parent / "events.jsonl"
        if events_path.exists():
            for line in events_path.read_text(errors="replace").splitlines():
                try:
                    ev = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if ev.get("event") == "startup" and "run_id" in ev:
                    report.mediator_run_id = int(ev["run_id"])
                    break


def parse_arducopter(path: Path, report: RunReport) -> None:
    """Detect ArduPilot crashes and extract stack traces from arducopter.log."""
    if not path.exists():
        return
    text = path.read_text(errors="replace")
    lines = text.splitlines()

    error_line = None
    for ln in lines:
        if "ERROR:" in ln and ("exception" in ln.lower() or "abort" in ln.lower()
                               or "signal" in ln.lower()):
            error_line = ln.strip()
            break
    if error_line is None:
        return

    # Extract stack from dumpstack output
    stack = []
    in_dump = False
    for ln in lines:
        if "begin dumpstack" in ln:
            in_dump = True
            continue
        if "end dumpstack" in ln:
            in_dump = False
            continue
        if in_dump:
            s = ln.strip()
            if s.startswith("#") and s:
                stack.append(s)

    report.sitl_crash = SitlCrash(error_line=error_line, stack=stack)


def parse_pytest(path: Path, report: RunReport) -> None:
    if not path.exists():
        print(f"  [!] pytest log not found: {path}")
        return
    t0 = None
    for line in path.read_text(errors="replace").splitlines():
        ts_m = _RE_TIMESTAMP.match(line)
        ts   = ts_m.group(1) if ts_m else ""
        ts_s = _wall_seconds(ts) if ts else None
        if ts and t0 is None:
            t0 = _wall_seconds(ts)

        if m := _RE_RUN_ID.search(line):
            report.pytest_run_id = int(m.group(1))

        if m := _RE_STATUSTEXT.search(line):
            # Prefer obs-time from [t=X.Xs] format when present; fall back to wall clock ts
            tm = _RE_STATUSTEXT_T.search(line)
            obs_label = f"t={tm.group(1)}s" if tm else ts
            report.statustext.append((obs_label, m.group(1).strip()))

        if m := _RE_EKF_FLAGS.search(line):
            report.ekf_flags.append((ts, int(m.group(1), 16)))

        if m := _RE_SETUP_STEP.search(line):
            msg = m.group(1).strip()
            # Deduplicate: only keep first occurrence of each step
            if not any(msg == s for _, s in report.setup_steps):
                report.setup_steps.append((ts, msg))

        if m := _RE_HOLD_STAT.search(line):
            report.hold_stats = {
                "pos":   int(m.group(1)),
                "att":   int(m.group(2)),
                "servo": int(m.group(3)),
            }

        if m := _RE_PASS.search(line):
            report.test_results.append((m.group(2), m.group(1)))

        # Crash timing: track last AP-originated message and obs-loop progress
        if ts_s is not None:
            if _RE_AP_ALIVE.search(line):
                report.gcs_last_ap_wall_s = ts_s
            if _RE_OBS_START.search(line):
                report.gcs_obs_start_wall_s = ts_s
            if m := _RE_OBS_TICK.search(line):
                report.gcs_last_obs_t_s = float(m.group(1))


# ---------------------------------------------------------------------------
# EKF flag helpers
# ---------------------------------------------------------------------------

_EKF_FLAG_NAMES = {
    0x0001: "attitude",
    0x0002: "horiz vel",
    0x0004: "vert vel",
    0x0008: "horiz pos (rel)",
    0x0010: "horiz pos (abs)",
    0x0020: "vert pos (abs)",
    0x0040: "terrain alt",
    0x0080: "const pos mode",   # EKF in constant-position fallback (no GPS pos fix)
    0x0100: "pred horiz pos rel",
    0x0200: "pred horiz pos abs",
    0x0400: "GPS glitch",       # GPS position innovation rejected
}

def _decode_flags(flags: int) -> str:
    bits = [name for mask, name in _EKF_FLAG_NAMES.items() if flags & mask]
    return ", ".join(bits) if bits else "none"


def _flag_transitions(ekf_flags: list) -> list:
    """Return list of (timestamp, old_flags, new_flags) on any change."""
    transitions = []
    prev = None
    for ts, f in ekf_flags:
        if f != prev:
            transitions.append((ts, prev if prev is not None else 0, f))
            prev = f
    return transitions


# ---------------------------------------------------------------------------
# Report printing
# ---------------------------------------------------------------------------

_W = 72

def _rule(char="-"):
    print(char * _W)

def _header(title: str):
    print()
    _rule("=")
    print(f"  {title}")
    _rule("=")


def _print_mediator(r: RunReport, fl: "FlightLog | None" = None) -> None:
    rows = r.tel_rows
    if not rows:
        print("  (no telemetry rows loaded)")
        return

    _header("MEDIATOR PHYSICS")

    mode = r.sensor_mode or "?"
    print(f"  sensor_mode : {mode}")
    if r.damp_T is not None:
        k_ang_str = f"  k_ang={r.k_ang:.1f}" if r.k_ang is not None else ""
        print(f"  damping     : T={r.damp_T:.0f}s{k_ang_str}")

    damp_rows = [row for row in rows if row.damp_alpha > 0.0]
    free_rows  = [row for row in rows if row.damp_alpha == 0.0]

    # Damping phase
    if damp_rows:
        print()
        damp_end_t = damp_rows[-1].t_sim
        print(f"  -- Damping phase  (t=0 - {damp_end_t:.0f}s) --")
        min_alt   = min(-row.pos_z for row in damp_rows)
        last_damp = damp_rows[-1]

        if r.kin_launch_pos and r.kin_vel:
            lp = r.kin_launch_pos
            kv = r.kin_vel
            deviations = []
            for row in damp_rows:
                ex = lp[0] + kv[0] * row.t_sim
                ey = lp[1] + kv[1] * row.t_sim
                ez = lp[2] + kv[2] * row.t_sim
                deviations.append(
                    ((row.pos_x - ex)**2 + (row.pos_y - ey)**2 + (row.pos_z - ez)**2)**0.5
                )
            max_dev = max(deviations)
            dT = damp_end_t
            arr_err = (
                (last_damp.pos_x - (lp[0] + kv[0]*dT))**2 +
                (last_damp.pos_y - (lp[1] + kv[1]*dT))**2 +
                (last_damp.pos_z - (lp[2] + kv[2]*dT))**2
            )**0.5
            print(f"    trajectory type                : constant-velocity kinematic")
            print(f"    max deviation from linear traj : {max_dev:.3f} m  "
                  + ("[OK]" if max_dev < 0.5 else "[!!]"))
            print(f"    arrival error at t=T           : {arr_err:.3f} m  "
                  + ("[OK]" if arr_err < 0.5 else "[!!]"))
        else:
            first = damp_rows[0]
            max_drift = max(
                ((row.pos_x - first.pos_x)**2 + (row.pos_y - first.pos_y)**2 +
                 (row.pos_z - first.pos_z)**2)**0.5
                for row in damp_rows
            )
            print(f"    max pos drift from equilibrium : {max_drift:.3f} m  "
                  + ("[OK]" if max_drift < 0.1 else "[!!] spring may not be active"))

        print(f"    min altitude                   : {min_alt:.2f} m  "
              + ("[OK]" if min_alt > 5.0 else "[!!] hub hit ground"))
        print(f"    final thrust at damp end       : {last_damp.aero_T:.1f} N")
        print(f"    final tether tension           : {last_damp.tether_tension:.0f} N")

        # Swashplate tilt post-arm — must stay near zero in ACRO.
        # Use the arm timestamp from MAVLink events (most precise); fall back to
        # a 30 s fixed offset if no arm event is recorded (e.g. static mediator tests).
        _TILT_WARN = 0.10   # normalised [-1,1]; same threshold as test_tail_tilt
        arm_t: "float | None" = None
        if fl is not None:
            for ev in fl.events:
                if ev.category == "ARM" and ev.text.strip() == "ARMED":
                    arm_t = ev.t_sim
                    break
        _SETTLE_S   = 30.0
        t0_damp     = damp_rows[0].t_sim
        window_start = arm_t if arm_t is not None else t0_damp + _SETTLE_S
        post_arm_rows = [row for row in damp_rows if row.t_sim >= window_start]
        if post_arm_rows:
            max_tilt_lon = max(abs(row.tilt_lon) for row in post_arm_rows)
            max_tilt_lat = max(abs(row.tilt_lat) for row in post_arm_rows)
            _tilt_ok = max_tilt_lon < _TILT_WARN and max_tilt_lat < _TILT_WARN
            label = f"post-arm (t>={window_start:.0f}s)"
            print(f"    max swashplate tilt ({label}): lon={max_tilt_lon:.3f}  lat={max_tilt_lat:.3f}  "
                  + ("[OK]" if _tilt_ok else f"[!!] exceeds {_TILT_WARN} -- ACRO/I-term issue?"))

    # Kinematic exit (transition) — row stamped note="kinematic_exit" in CSV;
    # fall back to first free-flight row (damp_alpha==0) for older logs.
    _tr_rows = [r for r in rows if r.note == "kinematic_exit"]
    tr = _tr_rows[0] if _tr_rows else (free_rows[0] if free_rows else None)
    if tr is not None:
        bz_elev_deg = math.degrees(math.asin(max(-1.0, min(1.0, -tr.r22))))
        upward_frac = -tr.r22   # fraction of thrust pointing up (NED Z is down)
        print()
        print(f"  -- Kinematic exit (t={tr.t_sim:.1f}s) --")
        print(f"    body_z elevation : {bz_elev_deg:.1f} deg  "
              + ("[OK]" if bz_elev_deg >= 15.0 else "[!!] too horizontal, insufficient lift"))
        print(f"    upward fraction  : {upward_frac:.3f}  "
              + ("[OK]" if upward_frac >= 0.25 else "[!!] < 0.25 -- hub will fall"))
        print(f"    collective_rad   : {tr.collective_rad:.4f} rad")
        print(f"    tether tension   : {tr.tether_tension:.1f} N  "
              + ("[OK]" if tr.tether_tension > 50 else "[!!] slack or very low"))
        print(f"    altitude         : {-tr.pos_z:.2f} m")
        print(f"    vel_z (down+)    : {tr.vel_z:.3f} m/s  "
              + ("[OK]" if tr.vel_z < 1.0 else "[!!] falling fast at exit"))

    # Sensor discontinuity at kinematic exit
    # Compare last kinematic row vs first free-flight row.
    # EKF uses sens_accel/gyro for IMU integration and rpy/vel/pos for GPS.
    # Any discontinuity in these values at the boundary is a physics bug.
    if damp_rows and free_rows:
        last_kin  = damp_rows[-1]
        first_flt = free_rows[0]
        print()
        print(f"  -- Sensor continuity at kinematic exit --")
        def _ang_deg(a: float, b: float) -> float:
            import math as _m
            d = _m.degrees(b - a)
            # wrap to (-180, 180]
            while d >  180.0: d -= 360.0
            while d <= -180.0: d += 360.0
            return d
        def _ck(label: str, delta: float, warn: float, unit: str = "") -> None:
            tag = "[OK]" if abs(delta) <= warn else "[!!]"
            print(f"    d({label:<18}) = {delta:+8.4f}{unit:5}  {tag}")

        # Position: should be continuous (stationary hold → pos unchanged)
        dp = ((first_flt.pos_x - last_kin.pos_x)**2 +
              (first_flt.pos_y - last_kin.pos_y)**2 +
              (first_flt.pos_z - last_kin.pos_z)**2) ** 0.5
        tag = "[OK]" if dp <= 0.01 else "[!!]"
        print(f"    d(pos_ned)           = {dp:+8.4f} m       {tag}")

        # Velocity: should be continuous (stationary hold → vel=0 initially)
        dv = ((first_flt.vel_x - last_kin.vel_x)**2 +
              (first_flt.vel_y - last_kin.vel_y)**2 +
              (first_flt.vel_z - last_kin.vel_z)**2) ** 0.5
        tag = "[OK]" if dv <= 0.05 else "[!!]"
        print(f"    d(vel_ned)           = {dv:+8.4f} m/s     {tag}")

        # Attitude (rpy): equilibrium hold should have zero change
        _ck("rpy_roll  (deg)",  _ang_deg(last_kin.rpy_roll,  first_flt.rpy_roll),  2.0, " deg")
        _ck("rpy_pitch (deg)",  _ang_deg(last_kin.rpy_pitch, first_flt.rpy_pitch), 2.0, " deg")
        _ck("rpy_yaw   (deg)",  _ang_deg(last_kin.rpy_yaw,   first_flt.rpy_yaw),   2.0, " deg")

        # Body-frame accelerometer (what EKF integrates for attitude/velocity)
        _ck("sens_accel_x(m/s2)", first_flt.sens_accel_x - last_kin.sens_accel_x, 1.0, " m/s2")
        _ck("sens_accel_y(m/s2)", first_flt.sens_accel_y - last_kin.sens_accel_y, 1.0, " m/s2")
        _ck("sens_accel_z(m/s2)", first_flt.sens_accel_z - last_kin.sens_accel_z, 1.0, " m/s2")

        # Body-frame gyroscope (what EKF integrates for attitude)
        _ck("sens_gyro_x(rad/s)", first_flt.sens_gyro_x - last_kin.sens_gyro_x, 0.05, " rad/s")
        _ck("sens_gyro_y(rad/s)", first_flt.sens_gyro_y - last_kin.sens_gyro_y, 0.05, " rad/s")
        _ck("sens_gyro_z(rad/s)", first_flt.sens_gyro_z - last_kin.sens_gyro_z, 0.05, " rad/s")

        # Heading gap (should not spike at transition)
        dhdg = abs(first_flt.heading_gap_deg) - abs(last_kin.heading_gap_deg)
        tag = "[OK]" if abs(dhdg) <= 15.0 else "[!!] heading gap spike at exit"
        print(f"    d(heading_gap  (deg)) = {dhdg:+8.3f} deg     {tag}")

        # Force and moment balance at boundary vs equilibrium snapshot
        # Load expected values from steady_state_starting.json (generated by
        # test_steady_flight.py — same rotor/wind/collective as the stack test).
        import json as _json
        _ic_path = Path(__file__).resolve().parents[1] / "steady_state_starting.json"
        _eq = {}
        if _ic_path.exists():
            try:
                _eq = _json.loads(_ic_path.read_text()).get("eq_physics", {})
            except Exception:
                pass

        print()
        if _eq:
            print(f"    eq_physics: collective={_eq.get('collective_rad',0):.4f} rad  "
                  f"tilt_lon={_eq.get('tilt_lon',0):.3f}  tilt_lat={_eq.get('tilt_lat',0):.3f}  "
                  f"omega_spin={_eq.get('omega_spin',0):.2f} rad/s")
            print(f"    stack exit: collective={first_flt.collective_rad:.4f} rad  "
                  f"tilt_lon={first_flt.tilt_lon:.3f}  tilt_lat={first_flt.tilt_lat:.3f}  "
                  f"omega_spin={first_flt.omega_rotor:.2f} rad/s")
            d_col = first_flt.collective_rad - _eq.get("collective_rad", 0)
            d_om  = first_flt.omega_rotor    - _eq.get("omega_spin", 0)
            print(f"    delta:      collective={d_col:+.4f} rad  "
                  f"omega_spin={d_om:+.2f} rad/s"
                  + ("  [!!] spin mismatch — forces will differ" if abs(d_om) > 2.0 else ""))

        hdr = f"    {'':20}  {'last_kin':>10}  {'first_flt':>10}  {'delta':>10}"
        if _eq:
            hdr += f"  {'eq_expect':>10}  {'err_flt':>10}"
        print(hdr)

        def _frow(label, a, b, eq_val=None, warn=5.0):
            delta = b - a
            tag = "[!!]" if abs(delta) > warn else ""
            row = f"    {label:<20}  {a:>10.1f}  {b:>10.1f}  {delta:>+10.1f}  {tag}"
            if eq_val is not None:
                err = b - eq_val
                etag = "[!!]" if abs(err) > warn else ""
                row += f"  {eq_val:>10.1f}  {err:>+10.1f}  {etag}"
            print(row)

        print(f"    -- translational (N) --")
        for label, a, b, ek in [
            ("aero_fx",   last_kin.aero_fx,   first_flt.aero_fx,   _eq.get("aero_fx")),
            ("aero_fy",   last_kin.aero_fy,   first_flt.aero_fy,   _eq.get("aero_fy")),
            ("aero_fz",   last_kin.aero_fz,   first_flt.aero_fz,   _eq.get("aero_fz")),
            ("tether_fx", last_kin.tether_fx, first_flt.tether_fx, _eq.get("tether_fx")),
            ("tether_fy", last_kin.tether_fy, first_flt.tether_fy, _eq.get("tether_fy")),
            ("tether_fz", last_kin.tether_fz, first_flt.tether_fz, _eq.get("tether_fz")),
            ("F_net_x",   last_kin.F_x,       first_flt.F_x,       _eq.get("F_net_x")),
            ("F_net_y",   last_kin.F_y,       first_flt.F_y,       _eq.get("F_net_y")),
            ("F_net_z",   last_kin.F_z,       first_flt.F_z,       _eq.get("F_net_z")),
        ]:
            _frow(label, a, b, ek)

        print(f"    -- rotational (N*m) --")
        for label, a, b, ek in [
            ("aero_mx",   last_kin.aero_mx,   first_flt.aero_mx,   _eq.get("aero_mx")),
            ("aero_my",   last_kin.aero_my,   first_flt.aero_my,   _eq.get("aero_my")),
            ("aero_mz",   last_kin.aero_mz,   first_flt.aero_mz,   _eq.get("aero_mz")),
            ("tether_mx", last_kin.tether_mx, first_flt.tether_mx, _eq.get("tether_mx")),
            ("tether_my", last_kin.tether_my, first_flt.tether_my, _eq.get("tether_my")),
            ("tether_mz", last_kin.tether_mz, first_flt.tether_mz, _eq.get("tether_mz")),
        ]:
            _frow(label, a, b, ek, warn=2.0)

    # Free-flight phase
    if free_rows:
        t_start = free_rows[0].t_sim
        print()
        print(f"  -- Free-flight phase  (t={t_start:.0f}s onward) --")
        min_alt_free = min(-row.pos_z for row in free_rows)
        max_alt_free = max(-row.pos_z for row in free_rows)
        min_r = min((row.pos_x**2 + row.pos_y**2 + row.pos_z**2)**0.5 for row in free_rows)
        max_r = max((row.pos_x**2 + row.pos_y**2 + row.pos_z**2)**0.5 for row in free_rows)
        omega_mean = sum(row.omega_rotor for row in free_rows) / len(free_rows)
        print(f"    altitude range   : {min_alt_free:.1f} - {max_alt_free:.1f} m  "
              + ("[OK]" if min_alt_free > 3.0 else "[!!] hub too low"))
        print(f"    tether length    : {min_r:.1f} - {max_r:.1f} m")
        print(f"    mean rotor spin  : {omega_mean:.1f} rad/s")

        # First 10s of free flight — crash rate check
        dt_tel = 1.0 / 400.0  # default 400 Hz
        if len(free_rows) > 1:
            dt_tel = (free_rows[-1].t_sim - free_rows[0].t_sim) / max(len(free_rows) - 1, 1)
        n_early = min(len(free_rows), max(1, int(10.0 / dt_tel)))
        early = free_rows[:n_early]
        if early:
            min_alt_early = min(-r.pos_z for r in early)
            max_vz_early  = max(r.vel_z for r in early)
            t_early_end   = early[-1].t_sim
            if min_alt_early < 3.0 or max_vz_early > 3.0:
                print(f"    [!!] first {t_early_end - t_start:.0f}s: "
                      f"min_alt={min_alt_early:.1f}m  "
                      f"max_vel_down={max_vz_early:.1f} m/s  (crash indicator)")

        floor_hits = sum(1 for row in free_rows if -row.pos_z <= 1.05)
        if floor_hits:
            print(f"    [!!] floor hits  : {floor_hits} frames at altitude<=1.05 m")


def _print_tel_events(r: RunReport) -> None:
    """Print timeline of note-stamped events from the telemetry CSV."""
    events = [(row.t_sim, row.note) for row in r.tel_rows if row.note]
    if not events:
        return
    _header("TELEMETRY EVENTS (from CSV notes)")
    for t_ev, note in events:
        print(f"  t={t_ev:7.2f}s  {note}")


def _print_ekf(r: RunReport) -> None:
    _header("EKF TIMELINE")

    # GPS fusion keywords — shown with [OK]/[!!] flags for quick triage
    GPS_GOOD = {"origin set", "is using gps", "yaw aligned", "tilt alignment"}
    GPS_BAD  = {"gps glitch", "ekf failsafe", "compass error", "yaw reset", "emergency yaw"}
    keywords = [
        "initialised", "tilt alignment", "origin set", "GPS Glitch",
        "EKF Failsafe", "yaw aligned", "GPS 1:", "AHRS:", "is using GPS",
        "Compass error", "emergency yaw", "yaw reset", "RAWES flight: GPS",
    ]
    relevant = [(ts, txt) for ts, txt in r.statustext
                if any(k.lower() in txt.lower() for k in keywords)]
    if relevant:
        for ts, txt in relevant:
            tl = txt.lower()
            flag = ""
            if any(k in tl for k in GPS_GOOD):
                flag = "  [OK]"
            elif any(k in tl for k in GPS_BAD):
                flag = "  [!!]"
            print(f"  {ts}  {txt}{flag}")
    else:
        print("  (no EKF / GPS STATUSTEXT found)")

    transitions = _flag_transitions(r.ekf_flags)
    if transitions:
        print()
        print("  EKF_STATUS flag transitions:")
        for ts, old_f, new_f in transitions:
            gained  = new_f & ~old_f
            lost    = old_f & ~new_f
            changes = []
            if gained:
                changes.append(f"+[{_decode_flags(gained)}]")
            if lost:
                changes.append(f"-[{_decode_flags(lost)}]")
            label = "  ".join(changes) if changes else "(no change)"
            print(f"    {ts}  0x{new_f:04x}  {label}")

    if r.ekf_flags:
        final_ts, final_f = r.ekf_flags[-1]
        print()
        print(f"  Final EKF flags @ {final_ts}: 0x{final_f:04x}")
        has_pos = bool(final_f & 0x0018)
        has_vel = bool(final_f & 0x0006)
        print(f"    horiz position valid : {'[OK]' if has_pos else '[!!]'}")
        print(f"    velocity valid       : {'[OK]' if has_vel else '[!!]'}")
        print(f"    GPS yaw valid        : {'[OK]' if final_f & 0x0080 else '[!!]'}")


def _print_mode_check(fl: FlightLog) -> None:
    """
    Verify that ArduPilot was in ACRO mode at arm and stayed there.

    Reads MODE messages from dataflash.BIN.  Prints a single [OK] line if
    ACRO was the first mode after arm and was never left.  If a violation is
    found, prints [!!] plus the full mode timeline so the cause is obvious.
    """
    _header("ACRO MODE CHECK")

    rows = fl._df_mode_rows
    if not rows:
        print("  (no MODE messages in dataflash.BIN -- cannot verify)")
        return

    # Find arm time from events (category="ARM", text exactly "ARMED").
    # Must match exactly to avoid "DISARMED" (which contains "ARMED").
    arm_t = None
    for ev in fl.events:
        if ev.category == "ARM" and ev.text.strip() == "ARMED":
            arm_t = ev.t_sim
            break

    # Always print the full mode timeline (compact)
    print("  Mode timeline (dataflash MODE messages):")
    for r in rows:
        marker = ""
        if arm_t is not None and abs(r["t_s"] - arm_t) < 0.5:
            marker = "  <-- arm"
        tag = "" if r["mode"] == 1 else "  [!!] NOT ACRO"
        print(f"    t={r['t_s']:7.2f}s  {r['mode_name']:12s} (mode={r['mode']:2d}){marker}{tag}")

    # Check: after arm, was mode always ACRO?
    if arm_t is not None:
        post_arm = [r for r in rows if r["t_s"] >= arm_t - 0.5]
    else:
        post_arm = rows  # no arm event found -- check everything

    non_acro = [r for r in post_arm if r["mode"] != 1]

    print()
    if not non_acro:
        if arm_t is not None:
            print(f"  [OK] ACRO mode active at arm (t={arm_t:.1f}s) and maintained throughout.")
        else:
            print("  [OK] ACRO mode throughout (no arm event found to anchor check).")
    else:
        print(f"  [!!] NON-ACRO mode detected after arm:")
        for r in non_acro:
            print(f"       t={r['t_s']:.2f}s  {r['mode_name']} (mode={r['mode']})")
        print()
        print("  This is the tilt contamination root cause: ArduPilot in STABILIZE")
        print("  actively commands roll/pitch toward 0 deg, driving constant cyclic")
        print("  at 65-deg tether equilibrium. Fix: set ACRO before arm.")

    # Time-alignment cross-check: first dataflash MODE=ACRO vs MAVLink ACRO event.
    # Both are ArduPilot boot seconds so they should agree within ~1 s.
    # A large gap means the SYSTEM_TIME anchor conversion is off.
    mav_acro = next(
        (ev.t_sim for ev in fl.events
         if ev.category == "MODE" and "ACRO" in ev.text and arm_t is not None and ev.t_sim >= arm_t - 1.0),
        None,
    )
    df_acro = next(
        (r["t_s"] for r in rows if r["mode"] == 1 and arm_t is not None and r["t_s"] >= arm_t - 1.0),
        None,
    )
    if mav_acro is not None and df_acro is not None:
        delta = abs(df_acro - mav_acro)
        ok = delta < 2.0
        print(f"\n  Time alignment (MAVLink vs dataflash, post-arm ACRO):")
        print(f"    MAVLink ACRO : t={mav_acro:.2f}s")
        print(f"    Dataflash ACRO: t={df_acro:.2f}s")
        print(f"    delta        : {delta:.2f}s  " + ("[OK]" if ok else "[!!] >2 s -- SYSTEM_TIME anchor may be off"))


def _print_landing(r: RunReport) -> None:
    """Print landing-specific diagnostics (descent, tension, yaw tracking)."""
    rows = r.tel_rows
    descent_rows = [row for row in rows if row.phase == "descent"]
    final_rows   = [row for row in rows if row.phase == "final_drop"]
    if not descent_rows and not final_rows:
        return

    _header("LANDING DIAGNOSTICS")

    if descent_rows:
        t0   = descent_rows[0].t_sim
        t1   = descent_rows[-1].t_sim
        alts = [-row.pos_z for row in descent_rows]
        tens = [row.tether_tension for row in descent_rows]
        vhs  = [row.v_horiz_ms     for row in descent_rows]
        orbs = [math.degrees(row.orb_yaw_rad) for row in descent_rows]
        # velocity-derived yaw (what EKF got, approximated from rpy)
        rpys = [math.degrees(row.rpy_yaw) for row in descent_rows]
        # yaw gap: difference between orb yaw and sent rpy yaw
        gaps = [abs(((o - s + 180) % 360) - 180)
                for o, s in zip(orbs, rpys)]
        horiz_dists = [(row.pos_x**2 + row.pos_y**2)**0.5 for row in descent_rows]

        print(f"  Descent phase: t={t0:.1f}s to t={t1:.1f}s  ({len(descent_rows)} rows)")
        print(f"    altitude      : {min(alts):.2f} m .. {max(alts):.2f} m")
        print(f"    descent rate  : {(alts[0]-alts[-1])/(t1-t0):.3f} m/s avg  "
              + "(positive = down)")
        print(f"    tether tension: peak={max(tens):.0f} N  mean={sum(tens)/len(tens):.0f} N")
        print(f"    v_horiz       : max={max(vhs):.2f} m/s  mean={sum(vhs)/len(vhs):.2f} m/s")
        print(f"    horiz dist    : max={max(horiz_dists):.2f} m from anchor")
        print(f"    orb_yaw       : {min(orbs):.0f} .. {max(orbs):.0f} deg")
        print(f"    rpy_yaw (sent): {min(rpys):.0f} .. {max(rpys):.0f} deg")
        print(f"    yaw gap       : max={max(gaps):.1f} deg  mean={sum(gaps)/len(gaps):.1f} deg")

        # Sample the first 60s of descent at ~1 Hz
        dt_tel = 1.0
        if len(descent_rows) > 1:
            dt_tel = (descent_rows[-1].t_sim - descent_rows[0].t_sim) / max(len(descent_rows)-1, 1)
        stride = max(1, int(1.0 / dt_tel))
        sampled = descent_rows[::stride][:60]
        if sampled:
            print()
            print("  Descent sample (1s stride, first 60s):")
            print("  t_sim    alt     v_horiz  orb_yaw  rpy_yaw  yaw_gap  tension  pos_x  pos_y")
            for row in sampled:
                alt   = -row.pos_z
                oy    = math.degrees(row.orb_yaw_rad)
                ry    = math.degrees(row.rpy_yaw)
                gap   = abs(((oy - ry + 180) % 360) - 180)
                print(f"  {row.t_sim:6.1f}  {alt:5.1f}   {row.v_horiz_ms:6.2f}   "
                      f"{oy:6.1f}   {ry:6.1f}   {gap:5.1f}   "
                      f"{row.tether_tension:7.0f}  {row.pos_x:5.2f}  {row.pos_y:5.2f}")

    if final_rows:
        alt_fd = min(-row.pos_z for row in final_rows)
        print(f"\n  final_drop phase: {len(final_rows)} rows  "
              f"min_alt={alt_fd:.2f} m")


def _print_setup(r: RunReport) -> None:
    _header("SETUP STEPS")
    if not r.setup_steps:
        print("  (no setup steps found)")
        return
    for ts, msg in r.setup_steps:
        print(f"  {ts}  {msg}")


def _print_hold(r: RunReport) -> None:
    _header("HOLD RESULTS")
    if not r.hold_stats:
        print("  (no hold stats found -- test may not have reached hold phase)")
        return
    pos   = r.hold_stats.get("pos",   0)
    att   = r.hold_stats.get("att",   0)
    servo = r.hold_stats.get("servo", 0)
    print(f"  LOCAL_POSITION_NED samples : {pos:5d}  "
          + ("[OK]" if pos > 10 else "[!!] GPS position never fused"))
    print(f"  ATTITUDE samples           : {att:5d}  "
          + ("[OK]" if att > 10 else "[!!]"))
    print(f"  SERVO_OUTPUT_RAW samples   : {servo:5d}  "
          + ("[OK]" if servo > 10 else "[!!]"))


def _print_sitl_crash(r: RunReport) -> None:
    if r.sitl_crash is None:
        return
    _header("ARDUPILOT CRASH")
    print(f"  {r.sitl_crash.error_line}")

    # Estimate sim-time of crash from GCS log timing anchors
    t0 = r.mediator_t0_wall_s
    if t0 is not None:
        # Last message ArduPilot sent (heartbeat, STATUSTEXT, EKF_STATUS)
        if r.gcs_last_ap_wall_s is not None:
            last_ap_sim = r.gcs_last_ap_wall_s - t0
            print(f"  last AP message: ~t={last_ap_sim:.0f}s sim")

        # Last obs-loop heartbeat gives a tight upper bound on crash time
        if r.gcs_obs_start_wall_s is not None and r.gcs_last_obs_t_s is not None:
            crash_upper_sim = (r.gcs_obs_start_wall_s - t0) + r.gcs_last_obs_t_s + 5.0
            ff_upper = crash_upper_sim - (r.damp_T or 0.0)
            print(
                f"  crash detected: ~t={crash_upper_sim:.0f}s sim"
                f"  (~{ff_upper:.0f}s into free flight)"
            )
        elif r.gcs_obs_start_wall_s is not None:
            obs_sim = r.gcs_obs_start_wall_s - t0
            print(f"  obs started: ~t={obs_sim:.0f}s sim  (crash somewhere after)")

    if r.sitl_crash.stack:
        print("  Stack trace:")
        for frame in r.sitl_crash.stack:
            print(f"    {frame}")
    else:
        print("  (no stack trace available — ptrace blocked or dumpstack failed)")


def _print_results(r: RunReport) -> None:
    _header("TEST OUTCOMES")
    if not r.test_results:
        print("  (no PASSED/FAILED lines found)")
        return
    for name, outcome in r.test_results:
        mark = "[PASS]" if outcome == "PASSED" else "[FAIL]"
        print(f"  {mark} {outcome}  {name}")


def _print_all_statustext(r: RunReport) -> None:
    _header("ALL STATUSTEXT")
    if not r.statustext:
        print("  (none)")
        return
    for ts, txt in r.statustext:
        print(f"  {ts}  {txt}")


# ---------------------------------------------------------------------------
# Physics yaw divergence (from existing telemetry.csv)
# ---------------------------------------------------------------------------

def _print_yaw_divergence(r: RunReport) -> None:
    """
    Gap between orb_yaw_rad (actual R_orb orientation yaw) and rpy_yaw
    (attitude yaw sent to SITL as Euler angle).

    With honest sensors (current codebase): rpy_yaw == orb_yaw_rad, so this
    gap should always be ~0.  A non-zero gap here means sensor.py is
    overriding rpy[2] with something other than the R_orb yaw (regression).

    For the compass-vs-GPS-velocity heading gap that blocks GPS fusion, see
    the SENSOR CONSISTENCY section above (heading_gap_deg column).
    """
    rows = r.tel_rows
    if not rows:
        return

    def _yaw_gap_deg(row: TelRow) -> float:
        o = math.degrees(row.orb_yaw_rad)
        s = math.degrees(row.rpy_yaw)
        return abs(((o - s + 180) % 360) - 180)

    _header("YAW DIVERGENCE  (orb_yaw vs rpy_yaw -- should be ~0 with honest sensors)")
    print("  With honest sensors rpy_yaw == orb_yaw; gap > 0 indicates a sensor regression.")
    print("  For GPS fusion diagnostics see SENSOR CONSISTENCY above.")

    # Group into three logical segments: kinematic, free-flight, pumping phases
    segments = [
        ("kinematic (damp_alpha>0)",  [r for r in rows if r.damp_alpha > 0.0]),
        ("free-flight (damp_alpha=0, no pump phase)",
         [r for r in rows if r.damp_alpha == 0.0
                          and r.phase not in ("reel-out", "reel-in", "descent", "final_drop")]),
        ("pumping (reel-out/reel-in)", [r for r in rows if r.phase in ("reel-out", "reel-in")]),
        ("landing (descent/final_drop)", [r for r in rows if r.phase in ("descent", "final_drop")]),
    ]

    for label, seg_rows in segments:
        if not seg_rows:
            continue
        gaps = [_yaw_gap_deg(row) for row in seg_rows]
        mean_gap  = sum(gaps) / len(gaps)
        max_gap   = max(gaps)
        max_row   = seg_rows[gaps.index(max_gap)]
        hi_frac   = sum(1 for g in gaps if g > 15.0) / len(gaps)

        flag = "[OK]"
        if max_gap > 30.0:
            flag = "[!!] LARGE"
        elif max_gap > 15.0:
            flag = "[!]  check"

        print()
        print(f"  {label}  ({len(seg_rows)} rows)")
        print(f"    mean gap : {mean_gap:.1f} deg")
        print(f"    max gap  : {max_gap:.1f} deg  at t={max_row.t_sim:.1f}s  "
              f"(v_horiz={max_row.v_horiz_ms:.2f} m/s)  {flag}")
        print(f"    >15 deg  : {hi_frac*100:.0f}% of frames")


# ---------------------------------------------------------------------------
# Sensor consistency (compass heading vs GPS velocity heading)
# ---------------------------------------------------------------------------

def _print_sensor_consistency(r: RunReport) -> None:
    """
    Compass-vs-GPS-velocity heading gap analysis.

    ArduPilot EKF3 cross-checks compass yaw against the heading implied by
    GPS velocity (atan2(vel_E, vel_N)).  When the gap is large (typically
    > ~30 deg) the EKF blocks GPS velocity fusion and may trigger an
    emergency yaw reset.

    With honest sensors (EK3_SRC1_YAW=1):
      compass heading = rpy_yaw = R_orb yaw (built by build_orb_frame())
      vel heading     = atan2(vel_ned_E, vel_ned_N)  (what GPS sees)
      heading_gap_deg = compass - vel_heading  (wrapped to -180..180)

    These are the 'heading_gap_deg' and 'vel_heading_deg' columns added to
    telemetry in the sensor-columns refactor.  If absent (old telemetry),
    this section is skipped.
    """
    rows = r.tel_rows
    if not rows:
        return

    # Check if new sensor columns are populated (all-zero means old telemetry)
    has_cols = any(row.heading_gap_deg != 0.0 or row.vel_heading_deg != 0.0
                   for row in rows)
    if not has_cols:
        # Old telemetry: try to derive from rpy_yaw + physics vel directly.
        # orb_yaw = math.degrees(row.orb_yaw_rad)
        # vel_heading = math.degrees(math.atan2(row.vel_y, row.vel_x))
        # This approximation uses physics world vel, not sensor vel_ned.
        # Only run if orbit speeds are non-trivial (|v| > 0.1 m/s).
        derived = []
        for row in rows:
            v = math.hypot(row.vel_x, row.vel_y)
            if v < 0.1:
                derived.append(None)
                continue
            comp = math.degrees(row.rpy_yaw)
            vhd  = math.degrees(math.atan2(row.vel_y, row.vel_x))
            gap  = ((comp - vhd + 180.0) % 360.0) - 180.0
            derived.append(gap)
        if not any(g is not None for g in derived):
            return
        # Fall through: build synthetic rows list with derived gap
        _header("SENSOR CONSISTENCY  (derived from rpy_yaw vs physics vel -- old telemetry)")
        print("  NOTE: 'heading_gap_deg' column absent; gap derived from rpy_yaw vs vel_x/vel_y.")
        print("  For accurate sensor diagnostics, re-run the test (new telemetry schema).")
        good_gaps = [g for g in derived if g is not None]
        mean_gap = sum(abs(g) for g in good_gaps) / len(good_gaps)
        max_gap  = max(abs(g) for g in good_gaps)
        hi_frac  = sum(1 for g in good_gaps if abs(g) > 30.0) / len(good_gaps)
        flag = ("[!!] GPS fusion likely blocked" if max_gap > 90.0
                else ("[!]  check" if max_gap > 30.0 else "[OK]"))
        print(f"  mean |gap| : {mean_gap:.1f} deg")
        print(f"  max  |gap| : {max_gap:.1f} deg  {flag}")
        print(f"  >30 deg    : {hi_frac*100:.0f}%")
        return

    _header("SENSOR CONSISTENCY  (compass vs GPS velocity heading)")
    print("  ArduPilot EKF blocks GPS fusion when |compass - vel_heading| is large.")
    print("  compass heading = rpy_yaw = R_orb yaw  (what SITL compass reads)")
    print("  vel heading     = atan2(vel_ned_E, vel_ned_N)  (what GPS velocity implies)")
    print("  heading_gap_deg = compass - vel_heading  wrapped to (-180, 180]")

    segments = [
        ("kinematic",
         [row for row in rows if row.damp_alpha > 0.0]),
        ("free-flight (no pump/landing)",
         [row for row in rows if row.damp_alpha == 0.0
          and row.phase not in ("reel-out", "reel-in", "descent", "final_drop")]),
        ("pumping (reel-out/reel-in)",
         [row for row in rows if row.phase in ("reel-out", "reel-in")]),
        ("landing (descent/final_drop)",
         [row for row in rows if row.phase in ("descent", "final_drop")]),
    ]

    for label, seg in segments:
        if not seg:
            continue
        gaps     = [abs(row.heading_gap_deg) for row in seg]
        mean_gap = sum(gaps) / len(gaps)
        max_gap  = max(gaps)
        max_row  = seg[gaps.index(max_gap)]
        hi_frac  = sum(1 for g in gaps if g > 30.0) / len(gaps)

        flag = ("[!!] GPS fusion likely blocked" if max_gap > 90.0
                else ("[!]  check" if max_gap > 30.0 else "[OK]"))

        print()
        print(f"  {label}  ({len(seg)} rows)")
        print(f"    mean |gap|     : {mean_gap:.1f} deg")
        print(f"    max  |gap|     : {max_gap:.1f} deg  at t={max_row.t_sim:.1f}s  {flag}")
        frac_str = f"{hi_frac*100:.0f}%"
        print(f"    >30 deg frames : {frac_str}  "
              + ("[OK]" if hi_frac < 0.05 else "[!!] EKF unlikely to fuse GPS this phase"))

        # 5-point sample across the segment
        stride = max(1, len(seg) // 5)
        sample = seg[::stride][:5]
        print(f"    {'t_sim':>8}  {'compass':>8}  {'vel_head':>8}  "
              f"{'gap':>7}  {'v_horiz':>7}  {'sens_vel_n':>10}  {'sens_vel_e':>10}")
        for sr in sample:
            comp  = math.degrees(sr.rpy_yaw)
            vhd   = sr.vel_heading_deg
            gap   = sr.heading_gap_deg
            print(f"    {sr.t_sim:8.1f}  {comp:8.1f}  {vhd:8.1f}  "
                  f"{gap:7.1f}  {sr.v_horiz_ms:7.2f}  "
                  f"{sr.sens_vel_n:10.3f}  {sr.sens_vel_e:10.3f}")


# ---------------------------------------------------------------------------
# validate_ekf_window (migrated from analyse_mavlink.py)
# Public API: imported by test_kinematic_gps.py
# ---------------------------------------------------------------------------

_POST_FUSION_SETTLE_S = 5.0
_CONST_POS_SETTLE_S   = 2.0


def _mavlink_load(path: Path) -> list[dict]:
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


def _check_ekf_records(
    records: list[dict],
    t_start_s: float,
    t_end_s: float,
) -> list[str]:
    """Check EKF health in window [t_start_s, t_end_s] (boot-seconds).

    gps_glitching and related STATUSTEXT within _POST_FUSION_SETTLE_S of
    t_start_s are treated as a normal post-fusion transient.
    Returns a list of failure strings. Empty = clean.
    """
    settle_end       = t_start_s + _POST_FUSION_SETTLE_S
    const_settle_end = t_start_s + _CONST_POS_SETTLE_S

    failures:      list[str] = []
    glitch_late:   list[float] = []
    const_times:   list[float] = []
    att_loss:      list[float] = []

    for d in records:
        st = d.get("time_boot_ms", 0) / 1000.0
        if st < t_start_s or st > t_end_s:
            continue
        mt = d.get("mavpackettype", "")
        if mt == "EKF_STATUS_REPORT":
            flags = d.get("flags", 0)
            if (flags & 0x8000) and st > settle_end:
                glitch_late.append(st)
            if (flags & 0x0080) and st > const_settle_end:
                const_times.append(st)
            if not (flags & 0x0001):
                att_loss.append(st)
        elif mt == "STATUSTEXT":
            txt = d.get("text", "").rstrip("\x00").strip()
            if "yaw reset" in txt.lower():
                failures.append(f"t={st:.1f}s  EKF yaw reset: {txt!r}")
            if st > settle_end and ("gps glitch" in txt.lower()
                                    or "compass error" in txt.lower()):
                failures.append(f"t={st:.1f}s  GPS/compass problem: {txt!r}")

    if glitch_late:
        t0f, t1f = glitch_late[0], glitch_late[-1]
        failures.append(
            f"gps_glitching (0x8000) active after {_POST_FUSION_SETTLE_S:.0f}s settle window: "
            f"{len(glitch_late)} frames  (first t={t0f:.1f}s, last t={t1f:.1f}s)"
        )
    if const_times:
        t0f, t1f = const_times[0], const_times[-1]
        failures.append(
            f"const_pos_mode (0x0080) active for {len(const_times)} EKF frames in window "
            f"(first t={t0f:.1f}s, last t={t1f:.1f}s)"
        )
    if att_loss:
        t0f, t1f = att_loss[0], att_loss[-1]
        failures.append(
            f"attitude flag (0x0001) lost for {len(att_loss)} EKF frames in window "
            f"(first t={t0f:.1f}s, last t={t1f:.1f}s)"
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
    assert not issues, "EKF problems:\\n" + "\\n".join(issues)
    """
    mavlink_path = Path(mavlink_path)
    if mavlink_path.is_dir():
        mavlink_path = mavlink_path / "mavlink.jsonl"
    if not mavlink_path.exists():
        return [f"mavlink.jsonl not found: {mavlink_path}"]
    records = _mavlink_load(mavlink_path)
    if not records:
        return ["mavlink.jsonl is empty"]
    return _check_ekf_records(records, t_start_s, t_end_s)


# ---------------------------------------------------------------------------
# Bucketed flight analysis report
# ---------------------------------------------------------------------------

def _print_bucketed_report(fl: FlightLog, bucket_s: float = 5.0) -> None:
    """Print a per-bucket flight analysis table with events interleaved.

    Each row covers bucket_s seconds of simulation time and shows:
      physics (altitude, tension, collective, aero thrust),
      actual attitude (from ATTITUDE MAVLink),
      attitude error vs target (from ATTITUDE_TARGET MAVLink),
      heading gap (sensor consistency),
      EKF flags.

    Important events (EKF transitions, GPS fusion, arm/disarm, crashes) are
    printed between the row they fall in and the next row.
    """
    buckets = fl.buckets(bucket_s)
    if not buckets:
        print("  (no data)")
        return

    _header(f"FLIGHT ANALYSIS  (bucket={bucket_s:.1f}s, {len(buckets)} windows)")

    has_att = any(b.att_roll_deg is not None for b in buckets)
    has_tgt = any(b.tgt_roll_deg is not None for b in buckets)
    has_ekf = any(b.ekf_flags is not None for b in buckets)

    # Build header
    hdr = (f"  {'t(s)':>8}  {'phase':<12}  {'alt':>5}  {'T_N':>5}  "
           f"{'col_r':>6}  {'aeroN':>5}")
    if has_att:
        hdr += f"  {'roll':>5}  {'pit':>5}  {'yaw':>5}"
    if has_tgt:
        hdr += f"  {'t_err':>5}"
    hdr += f"  {'hdg':>4}"
    if has_ekf:
        hdr += f"  {'EKF':>8}"
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    SEV_CHAR = {"OK": "+", "WARN": "!", "ERROR": "!!", "INFO": " "}

    for b in buckets:
        # Build data row
        if b.n_rows > 0:
            phase_str = f"[{b.phase}]" if b.is_kinematic else b.phase
            row = (f"  {b.t_start:8.1f}  {phase_str:<12}  {b.alt_mean:5.1f}  "
                   f"{b.tether_tension_mean:5.0f}  {b.collective_rad_mean:6.3f}  "
                   f"{b.aero_T_mean:5.0f}")
        else:
            row = (f"  {b.t_start:8.1f}  {'--':<12}  {'--':>5}  {'--':>5}  "
                   f"{'--':>6}  {'--':>5}")

        if has_att:
            if b.att_roll_deg is not None:
                row += (f"  {b.att_roll_deg:5.1f}  {b.att_pitch_deg:5.1f}  "
                        f"{b.att_yaw_deg:5.1f}")
            else:
                row += f"  {'--':>5}  {'--':>5}  {'--':>5}"
        if has_tgt:
            row += (f"  {b.att_err_deg:5.1f}" if b.att_err_deg is not None
                    else f"  {'--':>5}")
        row += (f"  {b.heading_gap_deg_mean:4.1f}" if b.n_rows > 0
                else f"  {'--':>4}")
        if has_ekf:
            if b.ekf_flags is not None:
                flag_s = f"0x{b.ekf_flags:04x}"
                if _ekf_has_warn(b.ekf_flags):
                    flag_s += "!"
                row += f"  {flag_s:>8}"
            else:
                row += f"  {'--':>8}"

        print(row)

        # Events in this bucket
        for ev in b.events:
            sc  = SEV_CHAR.get(ev.severity, " ")
            cat = f"[{ev.category}]"
            txt = ev.text[:64]
            print(f"    {cat:<8} t={ev.t_sim:7.1f}s  [{sc}] {txt}")


# ---------------------------------------------------------------------------
# EKF telemetry (from mavlink.jsonl written by RawesGCS)
# ---------------------------------------------------------------------------

def _load_mavlink_ekf_rows(path: Path) -> list[dict]:
    """
    Read mavlink.jsonl and build one merged row per ATTITUDE message.

    Each row contains fields from ATTITUDE + latest LOCAL_POSITION_NED +
    latest EKF_STATUS_REPORT, keyed the same way as the old ekf_telemetry.csv.
    """
    rows = []
    last_pos: dict = {}
    last_ekf: dict = {}
    for msg in _iter_mavlink(path, types=["ATTITUDE", "LOCAL_POSITION_NED", "EKF_STATUS_REPORT"]):
        mt = msg.get("mavpackettype", "")
        if mt == "LOCAL_POSITION_NED":
            last_pos = {
                "pos_n": msg.get("x", 0.0), "pos_e": msg.get("y", 0.0),
                "pos_d": msg.get("z", 0.0), "vel_n": msg.get("vx", 0.0),
                "vel_e": msg.get("vy", 0.0), "vel_d": msg.get("vz", 0.0),
            }
        elif mt == "EKF_STATUS_REPORT":
            last_ekf = {
                "ekf_flags":                int(msg.get("flags", 0)),
                "ekf_vel_variance":         msg.get("velocity_variance", 0.0),
                "ekf_pos_horiz_variance":   msg.get("pos_horiz_variance", 0.0),
                "ekf_pos_vert_variance":    msg.get("pos_vert_variance", 0.0),
                "ekf_compass_variance":     msg.get("compass_variance", 0.0),
                "ekf_terrain_alt_variance": msg.get("terrain_alt_variance", 0.0),
            }
        elif mt == "ATTITUDE":
            row: dict = {
                "time_boot_ms":   int(msg.get("time_boot_ms", 0)),
                "att_roll":       msg.get("roll", 0.0),
                "att_pitch":      msg.get("pitch", 0.0),
                "att_yaw":        msg.get("yaw", 0.0),
                "att_rollspeed":  msg.get("rollspeed", 0.0),
                "att_pitchspeed": msg.get("pitchspeed", 0.0),
                "att_yawspeed":   msg.get("yawspeed", 0.0),
            }
            row.update(last_pos)
            row.update(last_ekf)
            rows.append(row)
    return rows


def _print_ekf_state(mavlink_log_path: Path) -> None:
    """
    Print EKF health metrics from mavlink.jsonl.

    Covers:
      - Flag transitions (which EKF capabilities were gained/lost and when)
      - Variance spikes (position / velocity / compass uncertainty)
      - Attitude rate peaks (indication of oscillation or divergence)
    """
    rows = _load_mavlink_ekf_rows(mavlink_log_path)
    if not rows:
        print(f"\n  (mavlink.jsonl not found or empty: {mavlink_log_path.name})")
        return

    _header("EKF TELEMETRY  (from mavlink.jsonl)")

    t0_ms = rows[0].get("time_boot_ms", 0.0)
    t1_ms = rows[-1].get("time_boot_ms", 0.0)
    dur_s = (t1_ms - t0_ms) / 1000.0
    print(f"  rows: {len(rows)}   span: {t0_ms/1000:.1f}s - {t1_ms/1000:.1f}s"
          f"  ({dur_s:.0f}s)")

    # ── Flag transitions ──────────────────────────────────────────────────
    EKF_FLAG_NAMES = {
        0x0001: "attitude",
        0x0002: "horiz vel",
        0x0004: "vert vel",
        0x0008: "horiz pos (rel)",
        0x0010: "horiz pos (abs)",
        0x0020: "vert pos (abs)",
        0x0040: "terrain alt",
        0x0080: "GPS yaw",
        0x0100: "pred horiz pos",
        0x0200: "const pos mode",
    }

    def _decode(flags: int) -> str:
        return ", ".join(n for m, n in EKF_FLAG_NAMES.items() if flags & m) or "none"

    print()
    print("  EKF flag transitions:")
    prev_flags = None
    for row in rows:
        flags = row.get("ekf_flags")
        if flags is None:
            continue
        if flags != prev_flags:
            tbs = row.get("time_boot_ms", 0)
            gained = 0 if prev_flags is None else (flags & ~prev_flags)
            lost   = 0 if prev_flags is None else (~flags & prev_flags)
            changes = []
            if prev_flags is None:
                changes.append(f"init [{_decode(flags)}]")
            else:
                if gained:
                    changes.append(f"+[{_decode(gained)}]")
                if lost:
                    changes.append(f"-[{_decode(lost)}]")
            label = "  ".join(changes) if changes else "(no change)"
            print(f"    t={tbs/1000:7.1f}s  0x{flags:04x}  {label}")
            prev_flags = flags

    # ── Variance peaks ────────────────────────────────────────────────────
    # Thresholds tuned to ArduPilot EKF3 default innovation gate sizes.
    VAR_THRESHOLDS = {
        "ekf_pos_horiz_variance":   (1.0, 5.0, "m^2  horiz pos"),
        "ekf_pos_vert_variance":    (0.5, 2.0, "m^2  vert pos"),
        "ekf_vel_variance":         (0.5, 2.0, "(m/s)^2  vel"),
        "ekf_compass_variance":     (0.2, 0.5, "rad^2  compass"),
        "ekf_terrain_alt_variance": (1.0, 5.0, "m^2  terrain alt"),
    }

    print()
    print("  EKF variance peaks:")
    any_spike = False
    for col, (warn, crit, unit) in VAR_THRESHOLDS.items():
        eligible = [row for row in rows if col in row]
        if not eligible:
            continue
        peak_row = max(eligible, key=lambda r: r[col])
        peak   = peak_row[col]
        peak_t = peak_row.get("time_boot_ms", 0)
        flag = "[OK]" if peak < warn else ("[!!]" if peak >= crit else "[! ]")
        if peak >= warn:
            any_spike = True
        print(f"    {col:<35} peak={peak:.3f} {unit}  at t={peak_t/1000:.1f}s  {flag}")
    if not any_spike:
        print("    (all variances within normal range)")

    # ── Attitude rate peaks ───────────────────────────────────────────────
    rate_cols = ["att_rollspeed", "att_pitchspeed", "att_yawspeed"]
    rate_vals = {c: [abs(row[c]) for row in rows if c in row] for c in rate_cols}
    if any(rate_vals.values()):
        print()
        print("  EKF attitude rate peaks (rad/s):")
        for c, vals in rate_vals.items():
            if not vals:
                continue
            peak = max(vals)
            flag = "[OK]" if peak < 1.5 else ("[!!]" if peak > 5.0 else "[! ]")
            print(f"    {c:<20} peak={peak:.2f} rad/s  {flag}")


# ---------------------------------------------------------------------------
# Optional plot
# ---------------------------------------------------------------------------

def _plot(r: RunReport) -> None:
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("matplotlib not available -- skipping plot")
        return

    rows = r.tel_rows
    if not rows:
        return

    ts     = np.array([row.t_sim       for row in rows])
    alt    = np.array([-row.pos_z      for row in rows])   # altitude = -NED_Z
    horiz  = np.array([(row.pos_x**2 + row.pos_y**2)**0.5 for row in rows])
    omega  = np.array([row.omega_rotor for row in rows])
    alphas = np.array([row.damp_alpha  for row in rows])
    roll   = np.array([math.degrees(row.rpy_roll)  for row in rows])
    pitch  = np.array([math.degrees(row.rpy_pitch) for row in rows])

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"Run analysis -- sensor_mode={r.sensor_mode or '?'}", fontsize=12)

    ax = axes[0]
    ax.plot(ts, alt, label="altitude (-NED Z)", color="steelblue")
    ax.axhline(1.0, color="red", linestyle="--", lw=0.8, label="floor=1m")
    ax.set_ylabel("Altitude (m)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(ts, horiz, label="Horiz radius", color="darkorange")
    ax.set_ylabel("Horiz dist from origin (m)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(ts, roll,  label="roll",  color="crimson")
    ax.plot(ts, pitch, label="pitch", color="seagreen")
    ax.set_ylabel("Attitude (deg)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[3]
    ax2 = ax.twinx()
    ax.plot(ts, omega, label="omega_rotor (rad/s)", color="purple")
    ax2.plot(ts, alphas, label="damp_alpha", color="gray", linestyle="--")
    ax.set_ylabel("Rotor spin (rad/s)")
    ax2.set_ylabel("damp_alpha")
    ax.set_xlabel("t (s)")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = _LOG_DIR / "last_run_analysis.png"
    plt.savefig(out, dpi=120)
    print(f"\n  Plot saved -> {out}")
    plt.show()


# ---------------------------------------------------------------------------
# Attitude focus report (--focus attitude)
# ---------------------------------------------------------------------------

def _fmt(v: Optional[float], w: int = 7, d: int = 1, signed: bool = False) -> str:
    if v is None:
        return (" " * (w - 2) + "--").rjust(w)
    fmt = f"{'+' if signed else ''}{w}.{d}f"
    return format(v, fmt)


def _print_attitude_focus(fl: FlightLog, bucket_s: float) -> None:
    """Per-bucket attitude vs desired, swashplate outputs, and rate PID breakdown.

    Table 1 — ATT + SWSH:
      t_s, DesRoll, Roll, dR, DesPitch, Pitch, dP, DesYaw, Yaw, Col%, PCyc%, RCyc%

    Table 2 — PIDR/PIDP rate PID breakdown (roll / pitch):
      t_s, RateTar, RateAct, P, FF, I, D  (roll then pitch)
      Identifies which PID term drives swashplate contamination.
      FF ≈ 0 means the contamination is pure P-on-rate-error;
      FF >> P means the attitude error is generating a rate FF term.
    """
    buckets = fl.buckets(bucket_s)
    has_df = any(b.df_roll_deg is not None for b in buckets)
    if not has_df:
        print("  (no dataflash ATT data -- dataflash.BIN absent or no ATT messages)")
        return

    _header(f"ATTITUDE FOCUS  (bucket={bucket_s:.1f}s, dataflash ATT + SWSH + PID)")

    # ── Table 1: ATT + SWSH ──────────────────────────────────────────────────
    print()
    print("  Table 1: Attitude error vs swashplate output")
    print("  Roll/Pitch in degrees.  dR/dP = actual - desired.  SWSH outputs in %.")
    print()
    hdr = (f"  {'t_s':>6}  "
           f"{'DesRoll':>7}  {'Roll':>7}  {'dR':>7}  "
           f"{'DesPitch':>8}  {'Pitch':>7}  {'dP':>7}  "
           f"{'DesYaw':>7}  {'Yaw':>7}  "
           f"{'Col%':>6}  {'PCyc%':>6}  {'RCyc%':>6}")
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    for b in buckets:
        if b.df_roll_deg is None:
            continue
        col_s  = f"{b.df_swsh_col:6.1f}" if b.df_swsh_col is not None else "  --  "
        pcyc_s = f"{b.df_swsh_pcyc:6.1f}" if b.df_swsh_pcyc is not None else "  --  "
        rcyc_s = f"{b.df_swsh_rcyc:6.1f}" if b.df_swsh_rcyc is not None else "  --  "
        dr = b.df_roll_err_deg  or 0.0
        dp = b.df_pitch_err_deg or 0.0
        flag = "  [!!]" if abs(dr) > 30.0 or abs(dp) > 30.0 else ""
        print(f"  {b.t_start:6.1f}  "
              f"{b.df_des_roll_deg:7.1f}  {b.df_roll_deg:7.1f}  {dr:+7.1f}  "
              f"{b.df_des_pitch_deg:8.1f}  {b.df_pitch_deg:7.1f}  {dp:+7.1f}  "
              f"{b.df_des_yaw_deg:7.1f}  {b.df_yaw_deg:7.1f}  "
              f"{col_s}  {pcyc_s}  {rcyc_s}{flag}")
        for ev in b.events:
            print(f"    [{ev.category}] t={ev.t_sim:.1f}s  {ev.text[:60]}")

    # ── Table 2: Rate PID breakdown ──────────────────────────────────────────
    has_pidr = any(b.df_pidr_tar is not None for b in buckets)
    has_pidp = any(b.df_pidp_tar is not None for b in buckets)
    if not has_pidr and not has_pidp:
        print()
        print("  (no PIDR/PIDP dataflash messages -- PID breakdown unavailable)")
        return

    print()
    print("  Table 2: Rate PID breakdown (PIDR=roll, PIDP=pitch)")
    print("  Tar=target rate (deg/s), Act=actual rate.  P/FF/I/D are PID terms.")
    print("  If FF dominates: contamination = attitude_error -> rate_FF_feedforward.")
    print("  If P dominates:  contamination = rate_error -> P gain.")
    print()
    hdr2 = (f"  {'t_s':>6}  "
            f"{'R.Tar':>6}  {'R.Act':>6}  {'R.P':>6}  {'R.FF':>6}  {'R.I':>6}  {'R.D':>6}  "
            f"{'P.Tar':>6}  {'P.Act':>6}  {'P.P':>6}  {'P.FF':>6}  {'P.I':>6}  {'P.D':>6}")
    print(hdr2)
    print("  " + "-" * (len(hdr2) - 2))
    for b in buckets:
        if b.df_pidr_tar is None and b.df_pidp_tar is None:
            continue
        print(f"  {b.t_start:6.1f}  "
              f"{_fmt(b.df_pidr_tar)}  {_fmt(b.df_pidr_act)}  "
              f"{_fmt(b.df_pidr_p,6,3)}  {_fmt(b.df_pidr_ff,6,3)}  "
              f"{_fmt(b.df_pidr_i,6,3)}  {_fmt(b.df_pidr_d,6,3)}  "
              f"{_fmt(b.df_pidp_tar)}  {_fmt(b.df_pidp_act)}  "
              f"{_fmt(b.df_pidp_p,6,3)}  {_fmt(b.df_pidp_ff,6,3)}  "
              f"{_fmt(b.df_pidp_i,6,3)}  {_fmt(b.df_pidp_d,6,3)}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _list_test_dirs() -> list:
    """Return per-test subdirectories in logs/, sorted by modification time (newest first)."""
    if not _LOG_DIR.exists():
        return []
    return sorted(
        [d for d in _LOG_DIR.iterdir() if d.is_dir()],
        key=lambda d: d.stat().st_mtime,
        reverse=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("test_name", nargs="?", default=None,
                        help="Test name to analyse (e.g. test_acro_armed). "
                             "Reads logs/{test_name}/telemetry.csv and mediator.log. "
                             "Omit to list available test directories.")
    parser.add_argument("--telemetry", default=None,
                        help="Override telemetry CSV path")
    parser.add_argument("--mediator", default=None,
                        help="Override mediator log path")
    parser.add_argument("--plot", action="store_true",
                        help="Save + show a 4-panel position/attitude/spin plot")
    parser.add_argument("--all-statustext", action="store_true",
                        help="Print every STATUSTEXT line (not just EKF/GPS)")
    parser.add_argument("--bucket", type=float, default=5.0, metavar="S",
                        help="Bucket size in seconds for the per-window analysis "
                             "table (default: 5.0). Use a larger value for a "
                             "high-level overview, smaller for detailed debugging.")
    parser.add_argument("--focus", default=None, metavar="MODE",
                        help="Focus mode: 'attitude' shows per-bucket "
                             "DesRoll/Roll/dR, DesPitch/Pitch/dP, and "
                             "swashplate PCyc/RCyc/Col from dataflash.BIN.")
    args = parser.parse_args()

    if args.test_name is None:
        dirs = _list_test_dirs()
        if not dirs:
            print(f"No test directories found in {_LOG_DIR}")
        else:
            print(f"Available test runs in {_LOG_DIR}  (newest first):")
            for d in dirs:
                has_tel = (d / "telemetry.csv").exists()
                has_med = (d / "mediator.log").exists()
                tags = " ".join(filter(None, [
                    "telemetry.csv" if has_tel else "",
                    "mediator.log"  if has_med else "",
                ]))
                print(f"  {d.name:<45}  {tags}")
            print(f"\nUsage: python analyse_run.py <test_name>")
        return

    test_dir = _LOG_DIR / args.test_name
    if not test_dir.exists():
        print(f"[ERROR] Test directory not found: {test_dir}")
        dirs = _list_test_dirs()
        if dirs:
            print("Available:", ", ".join(d.name for d in dirs[:5]))
        return

    print(f"\nTest dir : {test_dir}")

    # Load unified FlightLog
    fl = FlightLog.load(test_dir)

    # Also build RunReport for sections that still use it
    report = RunReport()
    tel_path = Path(args.telemetry) if args.telemetry else test_dir / "telemetry.csv"
    med_path = Path(args.mediator)  if args.mediator  else test_dir / "mediator.log"
    load_telemetry(tel_path, report)
    parse_mediator(med_path, report)
    parse_pytest(test_dir / "gcs.log", report)
    parse_arducopter(test_dir / "arducopter.log", report)

    print(f"  telemetry rows   : {len(report.tel_rows)}")
    print(f"  MAVLink events   : {len(fl.events)}")
    print(f"  EKF_STATUS lines : {len(report.ekf_flags)}")

    # RUN_ID cross-validation
    m_id = report.mediator_run_id
    p_id = report.pytest_run_id
    if m_id is None and p_id is None:
        print("  RUN_ID           : not found in either log (old run?)")
    elif m_id is None:
        print(f"  RUN_ID           : pytest={p_id}  mediator=MISSING")
    elif p_id is None:
        print(f"  RUN_ID           : mediator={m_id}  pytest=MISSING")
    elif m_id == p_id:
        print(f"  RUN_ID           : {m_id}  [OK] logs match")
    else:
        print(f"  RUN_ID           : MISMATCH -- mediator={m_id}  pytest={p_id}")
        print("  WARNING: logs are from different runs -- analysis may be misleading!")

    if args.focus == "attitude":
        _print_attitude_focus(fl, args.bucket)
    else:
        _print_mediator(report, fl)
        _print_sitl_crash(report)
        _print_tel_events(report)
        _print_bucketed_report(fl, args.bucket)
        _print_ekf(report)
        _print_mode_check(fl)
        _print_sensor_consistency(report)
        _print_yaw_divergence(report)
        _print_ekf_state(test_dir / "mavlink.jsonl")
        _print_landing(report)
        _print_setup(report)
        _print_hold(report)
        _print_results(report)

        if args.all_statustext:
            _print_all_statustext(report)

        if args.plot:
            _plot(report)

    print()
    _rule()


if __name__ == "__main__":
    main()
