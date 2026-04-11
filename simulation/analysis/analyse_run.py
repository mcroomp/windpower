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
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

_SIM_DIR = Path(__file__).resolve().parents[1]   # simulation/

_LOG_DIR     = _SIM_DIR / "logs"
_PYTEST_LOG  = _LOG_DIR / "pytest_last_run.log"

sys.path.insert(0, str(_SIM_DIR))
from telemetry_csv import TelRow, read_csv  # noqa: E402


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class EkfEvent:
    timestamp: str
    t_wall:    float   # seconds-of-day from log timestamp
    text:      str


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


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

_RE_RUN_ID      = re.compile(r"RUN_ID=(\d+)")
_RE_STATUSTEXT  = re.compile(r"STATUSTEXT \[sev=\d+\]:\s*(.+)$")
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
    """Extract run metadata from the mediator log (RUN_ID, sensor mode, damping config).

    Physics frames are no longer read here -- use load_telemetry() instead.
    """
    if not path.exists():
        print(f"  [!] mediator log not found: {path}")
        return
    for line in path.read_text(errors="replace").splitlines():
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


def parse_pytest(path: Path, report: RunReport) -> None:
    if not path.exists():
        print(f"  [!] pytest log not found: {path}")
        return
    t0 = None
    for line in path.read_text(errors="replace").splitlines():
        ts_m = _RE_TIMESTAMP.match(line)
        ts   = ts_m.group(1) if ts_m else ""
        if ts and t0 is None:
            t0 = _wall_seconds(ts)

        if m := _RE_RUN_ID.search(line):
            report.pytest_run_id = int(m.group(1))

        if m := _RE_STATUSTEXT.search(line):
            report.statustext.append((ts, m.group(1).strip()))

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
    0x0080: "GPS yaw",
    0x0100: "pred horiz pos",
    0x0200: "const pos mode",
    0x0400: "pred horiz ok",
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


def _print_mediator(r: RunReport) -> None:
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

        floor_hits = sum(1 for row in free_rows if -row.pos_z <= 1.05)
        if floor_hits:
            print(f"    [!!] floor hits  : {floor_hits} frames at altitude<=1.05 m")


def _print_ekf(r: RunReport) -> None:
    _header("EKF TIMELINE")

    keywords = [
        "initialised", "tilt alignment", "origin set", "GPS Glitch",
        "EKF Failsafe", "yaw aligned", "GPS 1:", "AHRS:",
    ]
    relevant = [(ts, txt) for ts, txt in r.statustext
                if any(k.lower() in txt.lower() for k in keywords)]
    if relevant:
        for ts, txt in relevant:
            print(f"  {ts}  {txt}")
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
    parser.add_argument("--pytest",   default=str(_PYTEST_LOG),
                        help="pytest suite log (default: logs/pytest_last_run.log)")
    parser.add_argument("--plot", action="store_true",
                        help="Save + show a 4-panel position/attitude/spin plot")
    parser.add_argument("--all-statustext", action="store_true",
                        help="Print every STATUSTEXT line (not just EKF/GPS)")
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

    telemetry_path = Path(args.telemetry) if args.telemetry else test_dir / "telemetry.csv"
    mediator_path  = Path(args.mediator)  if args.mediator  else test_dir / "mediator.log"

    report = RunReport()
    print(f"\nTest dir      : {test_dir}")
    print(f"Loading telemetry CSV : {telemetry_path}")
    load_telemetry(telemetry_path, report)
    print(f"Parsing mediator log  : {mediator_path}")
    parse_mediator(mediator_path, report)
    print(f"Parsing pytest log    : {args.pytest}")
    parse_pytest(Path(args.pytest), report)
    print(f"  telemetry rows     : {len(report.tel_rows)}")
    print(f"  STATUSTEXT lines   : {len(report.statustext)}")
    print(f"  EKF_STATUS lines   : {len(report.ekf_flags)}")

    # RUN_ID cross-validation
    m_id = report.mediator_run_id
    p_id = report.pytest_run_id
    if m_id is None and p_id is None:
        print("  RUN_ID             : not found in either log (old run?)")
    elif m_id is None:
        print(f"  RUN_ID             : pytest={p_id}  mediator=MISSING")
    elif p_id is None:
        print(f"  RUN_ID             : mediator={m_id}  pytest=MISSING")
    elif m_id == p_id:
        print(f"  RUN_ID             : {m_id}  [OK] logs match")
    else:
        print(f"  RUN_ID             : MISMATCH -- mediator={m_id}  pytest={p_id}")
        print("  WARNING: logs are from different runs -- analysis may be misleading!")

    _print_mediator(report)
    _print_ekf(report)
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
