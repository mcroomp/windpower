#!/usr/bin/env python3
"""
analyse_run.py — Post-run analysis of the last simulation test run.

Reads the standard log files written by every test run and prints a
structured report covering:

  • Damping phase: position drift, min altitude, velocity convergence
  • EKF timeline: when GPS locked, origin set, any failsafes
  • Setup steps: which of the 6 setup steps completed and when
  • Hold results: position / attitude / servo sample counts, drift, altitude
  • Test outcomes: PASSED / FAILED per test

Usage (from repo root on Windows via WSL):
  wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py'
  wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py --plot'
  wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py --mediator other.log'
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

_SIM_DIR = Path(__file__).resolve().parents[1]   # simulation/

_LOG_DIR          = _SIM_DIR / "logs"
_DEFAULT_MEDIATOR = _LOG_DIR / "mediator_last_run.log"
_DEFAULT_PYTEST   = _LOG_DIR / "pytest_last_run.log"
_DEFAULT_GCS      = _LOG_DIR / "gcs_last_run.log"


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class MediatorFrame:
    t:       float
    pos:     tuple[float, float, float]   # ENU m
    rpy:     tuple[float, float, float]   # deg
    omega:   float                        # rotor rad/s
    thrust:  float                        # N
    tether:  str                          # TAUT / SLACK
    tension: float                        # N
    ext:     float                        # m
    alpha:   Optional[float] = None       # damping alpha (None when inactive)


@dataclass
class EkfEvent:
    timestamp: str
    t_wall:    float   # seconds-of-day from log timestamp
    text:      str


@dataclass
class RunReport:
    mediator_frames:  list[MediatorFrame]    = field(default_factory=list)
    statustext:       list[tuple[str, str]]  = field(default_factory=list)   # (ts, text)
    ekf_flags:        list[tuple[str, int]]  = field(default_factory=list)   # (ts, flags)
    setup_steps:      list[tuple[str, str]]  = field(default_factory=list)   # (ts, msg)
    test_results:     list[tuple[str, str]]  = field(default_factory=list)   # (name, PASSED/FAILED)
    hold_stats:       dict                   = field(default_factory=dict)
    sensor_mode:      Optional[str]          = None
    damp_T:           Optional[float]        = None
    k_vel:            Optional[float]        = None
    k_ang:            Optional[float]        = None
    k_pos:            Optional[float]        = None
    mediator_run_id:  Optional[int]          = None
    pytest_run_id:    Optional[int]          = None
    kin_launch_pos:   Optional[tuple]        = None   # (x, y, z) ENU [m]
    kin_vel:          Optional[tuple]        = None   # (vx, vy, vz) [m/s]


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

# Groups: 1=t  2-4=pos  5-7=rpy  8=omega  9=thrust  10=tether
#          11=tension  12=ext  13=alpha(optional)
_RE_MEDIATOR = re.compile(
    r"t=\s*([\d.]+)s\s+"
    r"pos_ENU=\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]m\s+"
    r"rpy=\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]deg\s+"
    r"omega=([\d.]+)\s+rad/s\s+"
    r"T=([-\d.]+)N\s+"
    r"tether=(\w+)"
    r"(?:\s+T=([\d.]+)N\s+ext=([\d.]+)m)?"
    r"(?:.*?\[DAMP\s+α=([\d.]+))?"
)
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


def parse_mediator(path: Path, report: RunReport) -> None:
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
            report.k_vel  = None
            report.k_pos  = None
        if m := _RE_KIN_START.search(line):
            report.kin_launch_pos = (float(m.group(2)), float(m.group(3)), float(m.group(4)))
            report.kin_vel        = (float(m.group(5)), float(m.group(6)), float(m.group(7)))
        if m := _RE_MEDIATOR.search(line):
            report.mediator_frames.append(MediatorFrame(
                t       = float(m.group(1)),
                pos     = (float(m.group(2)), float(m.group(3)), float(m.group(4))),
                rpy     = (float(m.group(5)), float(m.group(6)), float(m.group(7))),
                omega   = float(m.group(8)),
                thrust  = float(m.group(9)),
                tether  = m.group(10),
                tension = float(m.group(11)) if m.group(11) else 0.0,
                ext     = float(m.group(12)) if m.group(12) else 0.0,
                alpha   = float(m.group(13)) if m.group(13) else None,
            ))


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


def _flag_transitions(ekf_flags: list[tuple[str, int]]) -> list[tuple[str, int, int]]:
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

def _rule(char="─"):
    print(char * _W)

def _header(title: str):
    print()
    _rule("━")
    print(f"  {title}")
    _rule("━")


def _good(val, ok: bool) -> str:
    return f"{val}  {'✓' if ok else '✗'}"


def _print_mediator(r: RunReport) -> None:
    frames = r.mediator_frames
    if not frames:
        print("  (no mediator frames parsed)")
        return

    _header("MEDIATOR PHYSICS")

    # Config
    mode = r.sensor_mode or "?"
    print(f"  sensor_mode : {mode}")
    if r.damp_T is not None:
        print(f"  damping     : T={r.damp_T:.0f}s  (kinematic override)")

    damp_frames = [f for f in frames if f.alpha is not None and f.alpha > 0]
    free_frames = [f for f in frames if f.alpha is None or f.alpha == 0]

    # Equilibrium position (first frame)
    pos0 = frames[0].pos

    # Damping phase
    if damp_frames:
        print()
        damp_end_t = damp_frames[-1].t
        print(f"  ── Damping phase  (t=0 – {damp_end_t:.0f}s) ──")
        min_z     = min(f.pos[2] for f in damp_frames)
        last_damp = damp_frames[-1]

        if r.kin_launch_pos and r.kin_vel:
            # Constant-velocity kinematic: check how closely the hub follows
            # the expected linear trajectory pos(t) = launch_pos + vel * t.
            lp = r.kin_launch_pos
            kv = r.kin_vel
            deviations = []
            for f in damp_frames:
                ex = lp[0] + kv[0] * f.t
                ey = lp[1] + kv[1] * f.t
                ez = lp[2] + kv[2] * f.t
                deviations.append(((f.pos[0]-ex)**2 + (f.pos[1]-ey)**2 + (f.pos[2]-ez)**2)**0.5)
            max_dev = max(deviations)
            # Arrival error: distance from expected pos at end of damp
            dT = damp_end_t
            arr_x = lp[0] + kv[0] * dT
            arr_y = lp[1] + kv[1] * dT
            arr_z = lp[2] + kv[2] * dT
            arr_err = ((last_damp.pos[0]-arr_x)**2 + (last_damp.pos[1]-arr_y)**2 + (last_damp.pos[2]-arr_z)**2)**0.5
            print(f"    trajectory type                : constant-velocity kinematic")
            print(f"    max deviation from linear traj : {max_dev:.3f} m  "
                  + ("✓" if max_dev < 0.5 else "✗"))
            print(f"    arrival error at t=T           : {arr_err:.3f} m  "
                  + ("✓" if arr_err < 0.5 else "✗"))
        else:
            # Legacy: static damping — hub should stay near pos0
            max_drift = max(
                ((f.pos[0]-pos0[0])**2 + (f.pos[1]-pos0[1])**2 + (f.pos[2]-pos0[2])**2)**0.5
                for f in damp_frames
            )
            print(f"    max pos drift from equilibrium : {max_drift:.3f} m  "
                  + ("✓" if max_drift < 0.1 else "✗ (spring may not be active)"))

        print(f"    min ENU Z altitude             : {min_z:.2f} m  "
              + ("✓" if min_z > 5.0 else "✗ hub hit ground"))
        print(f"    final thrust at damp end       : {last_damp.thrust:.1f} N")
        print(f"    final tether tension           : {last_damp.tension:.0f} N")

    # Free-flight phase
    if free_frames:
        t_start = free_frames[0].t
        print()
        print(f"  ── Free-flight phase  (t={t_start:.0f}s onward) ──")
        min_z_free = min(f.pos[2] for f in free_frames)
        max_z_free = max(f.pos[2] for f in free_frames)
        min_r      = min(((f.pos[0])**2+(f.pos[1])**2+(f.pos[2])**2)**0.5 for f in free_frames)
        max_r      = max(((f.pos[0])**2+(f.pos[1])**2+(f.pos[2])**2)**0.5 for f in free_frames)
        omega_mean = sum(f.omega for f in free_frames) / len(free_frames)
        print(f"    ENU Z range      : {min_z_free:.1f} – {max_z_free:.1f} m  "
              + ("✓" if min_z_free > 3.0 else "✗ hub too low"))
        print(f"    tether length    : {min_r:.1f} – {max_r:.1f} m")
        print(f"    mean rotor spin  : {omega_mean:.1f} rad/s")

        floor_hits = sum(1 for f in free_frames if f.pos[2] <= 1.05)
        if floor_hits:
            print(f"    ✗ z_floor hits   : {floor_hits} frames at z≤1.05 m")


def _print_ekf(r: RunReport) -> None:
    _header("EKF TIMELINE")

    # STATUSTEXT events we care about
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

    # Flag transitions
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

    # Final flags
    if r.ekf_flags:
        final_ts, final_f = r.ekf_flags[-1]
        print()
        print(f"  Final EKF flags @ {final_ts}: 0x{final_f:04x}")
        has_pos = bool(final_f & 0x0018)   # horiz pos rel or abs
        has_vel = bool(final_f & 0x0006)   # horiz+vert vel
        print(f"    horiz position valid : {'✓' if has_pos else '✗'}")
        print(f"    velocity valid       : {'✓' if has_vel else '✗'}")
        print(f"    GPS yaw valid        : {'✓' if final_f & 0x0080 else '✗'}")


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
        print("  (no hold stats found — test may not have reached hold phase)")
        return
    pos   = r.hold_stats.get("pos",   0)
    att   = r.hold_stats.get("att",   0)
    servo = r.hold_stats.get("servo", 0)
    print(f"  LOCAL_POSITION_NED samples : {pos:5d}  {'✓' if pos > 10 else '✗ GPS position never fused'}")
    print(f"  ATTITUDE samples           : {att:5d}  {'✓' if att > 10 else '✗'}")
    print(f"  SERVO_OUTPUT_RAW samples   : {servo:5d}  {'✓' if servo > 10 else '✗'}")


def _print_results(r: RunReport) -> None:
    _header("TEST OUTCOMES")
    if not r.test_results:
        print("  (no PASSED/FAILED lines found)")
        return
    for name, outcome in r.test_results:
        mark = "✓" if outcome == "PASSED" else "✗"
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
        print("matplotlib not available — skipping plot")
        return

    frames = r.mediator_frames
    if not frames:
        return

    ts     = np.array([f.t      for f in frames])
    z      = np.array([f.pos[2] for f in frames])
    enu_r  = np.array([((f.pos[0])**2+(f.pos[1])**2)**0.5 for f in frames])
    omega  = np.array([f.omega   for f in frames])
    thrust = np.array([f.thrust  for f in frames])
    roll   = np.array([f.rpy[0]  for f in frames])
    pitch  = np.array([f.rpy[1]  for f in frames])
    alphas = np.array([f.alpha if f.alpha is not None else 0.0 for f in frames])

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"Run analysis — sensor_mode={r.sensor_mode or '?'}", fontsize=12)

    ax = axes[0]
    ax.plot(ts, z, label="Z (ENU)", color="steelblue")
    ax.axhline(1.0, color="red", linestyle="--", lw=0.8, label="z_floor=1m")
    ax.set_ylabel("Altitude (m)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(ts, enu_r, label="Horiz radius", color="darkorange")
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
    ax.plot(ts, omega, label="ω spin (rad/s)", color="purple")
    ax2.plot(ts, alphas, label="damp α", color="gray", linestyle="--")
    ax.set_ylabel("Rotor spin (rad/s)")
    ax2.set_ylabel("Damp α")
    ax.set_xlabel("t (s)")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = _LOG_DIR / "last_run_analysis.png"
    plt.savefig(out, dpi=120)
    print(f"\n  Plot saved → {out}")
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mediator", default=str(_DEFAULT_MEDIATOR),
                        help="mediator log file (default: simulation/mediator_last_run.log)")
    parser.add_argument("--pytest",   default=str(_DEFAULT_PYTEST),
                        help="pytest log file   (default: simulation/pytest_last_run.log)")
    parser.add_argument("--plot", action="store_true",
                        help="Save + show a 4-panel position/attitude/spin plot")
    parser.add_argument("--all-statustext", action="store_true",
                        help="Print every STATUSTEXT line (not just EKF/GPS)")
    args = parser.parse_args()

    report = RunReport()
    print(f"\nParsing mediator log : {args.mediator}")
    parse_mediator(Path(args.mediator), report)
    print(f"Parsing pytest log   : {args.pytest}")
    parse_pytest(Path(args.pytest), report)
    print(f"  mediator frames    : {len(report.mediator_frames)}")
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
        print(f"  RUN_ID             : {m_id}  ✓ logs match")
    else:
        print(f"  RUN_ID             : MISMATCH — mediator={m_id}  pytest={p_id}")
        print("  WARNING: logs are from different runs — analysis may be misleading!")

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
