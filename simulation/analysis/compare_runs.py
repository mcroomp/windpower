#!/usr/bin/env python3
"""
compare_runs.py -- Compare two stack test runs telemetry-by-telemetry.

Useful for finding where test_pumping_cycle_lua diverges from test_lua_flight_steady
when both share the same startup conditions.

Usage (inside Docker):
  python3 /rawes/simulation/analysis/compare_runs.py \\
      lua_full_test_lua_flight_steady \\
      pumping_lua_test_pumping_cycle_lua

  python3 /rawes/simulation/analysis/compare_runs.py \\
      lua_full_test_lua_flight_steady \\
      pumping_lua_test_pumping_cycle_lua \\
      --plot

The script aligns both runs on kinematic exit time (first row with damp_alpha==0),
then prints a side-by-side comparison of key metrics in 5 s windows.

Metrics compared:
  altitude [m]    -- -pos_z (hub altitude above anchor)
  orbit_r [m]     -- sqrt(pos_x^2 + pos_y^2) (horizontal distance from anchor)
  yaw_gap [deg]   -- orb_yaw_rad - rpy_yaw (EKF consistency; large = GPS glitch risk)
  collective [rad] -- Lua / internal controller collective command
  tension [N]     -- tether tension
  tlen [m]        -- tether rest length (winch position)
  bz_eq_z         -- body-z equilibrium z-component (disk orientation)

Divergence is flagged when:
  |altitude_diff|  > 2.0 m
  |orbit_r_diff|   > 5.0 m
  |yaw_gap_diff|   > 15 deg
  |collective_diff| > 0.02 rad
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Optional

_SIM_DIR = Path(__file__).resolve().parents[1]   # simulation/
_LOG_DIR = _SIM_DIR / "logs"
sys.path.insert(0, str(_SIM_DIR))

from telemetry_csv import TelRow, read_csv   # noqa: E402


# ---------------------------------------------------------------------------
# Thresholds for flagging divergence
# ---------------------------------------------------------------------------
_ALT_DIFF_WARN   = 2.0    # m
_ORB_DIFF_WARN   = 5.0    # m
_YAW_DIFF_WARN   = 15.0   # deg
_COL_DIFF_WARN   = 0.02   # rad


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _find_kin_exit_t(rows: list[TelRow]) -> Optional[float]:
    """Return t_sim of first row with damp_alpha == 0 (kinematic exit)."""
    prev = None
    for r in rows:
        if r.damp_alpha == 0.0 and (prev is None or prev.damp_alpha > 0.0):
            return r.t_sim
        prev = r
    return None


def _bucket(rows: list[TelRow], t_ref: float, window_s: float = 5.0) -> dict[int, list[TelRow]]:
    """Bucket rows into windows of `window_s` seconds after `t_ref`."""
    buckets: dict[int, list[TelRow]] = {}
    for r in rows:
        dt = r.t_sim - t_ref
        if dt < -5.0:
            continue   # before kinematic exit (skip kinematic)
        key = int(dt // window_s)
        buckets.setdefault(key, []).append(r)
    return buckets


def _mean(vals: list[float]) -> Optional[float]:
    return sum(vals) / len(vals) if vals else None


def _stats(rows: list[TelRow]) -> dict:
    """Compute mean key metrics for a list of rows."""
    if not rows:
        return {}
    alts    = [-r.pos_z for r in rows]
    orbs    = [math.sqrt(r.pos_x**2 + r.pos_y**2) for r in rows]
    yaw_gap = [math.degrees(r.orb_yaw_rad - r.rpy_yaw) for r in rows]
    cols    = [r.collective_rad for r in rows]
    tens    = [r.tether_tension for r in rows]
    tlens   = [r.tether_length for r in rows]
    bz_ez   = [r.bz_eq_z for r in rows]
    return {
        "alt":    _mean(alts),
        "orb_r":  _mean(orbs),
        "yaw_gap":_mean(yaw_gap),
        "col":    _mean(cols),
        "tension":_mean(tens),
        "tlen":   _mean(tlens),
        "bz_eq_z":_mean(bz_ez),
        "n":      len(rows),
    }


def _diff_flag(a: Optional[float], b: Optional[float],
               warn: float, fmt: str = ".2f") -> str:
    """Return diff string, flagged with * if above threshold."""
    if a is None or b is None:
        return "   n/a"
    d = b - a
    flag = " *" if abs(d) > warn else "  "
    return f"{d:+{fmt}}{flag}"


# ---------------------------------------------------------------------------
# Main report
# ---------------------------------------------------------------------------

def compare(name_a: str, name_b: str, window_s: float = 5.0,
            do_plot: bool = False) -> None:

    dir_a = _LOG_DIR / name_a
    dir_b = _LOG_DIR / name_b

    csv_a = dir_a / "telemetry.csv"
    csv_b = dir_b / "telemetry.csv"

    if not csv_a.exists():
        print(f"[ERROR] Telemetry not found: {csv_a}")
        sys.exit(1)
    if not csv_b.exists():
        print(f"[ERROR] Telemetry not found: {csv_b}")
        sys.exit(1)

    rows_a = read_csv(csv_a)
    rows_b = read_csv(csv_b)
    print(f"\nLoaded: {name_a}  ->  {len(rows_a)} rows")
    print(f"Loaded: {name_b}  ->  {len(rows_b)} rows")

    t_kin_a = _find_kin_exit_t(rows_a)
    t_kin_b = _find_kin_exit_t(rows_b)
    print(f"\nKinematic exit:  {name_a}: t={t_kin_a}s   {name_b}: t={t_kin_b}s")

    if t_kin_a is None or t_kin_b is None:
        print("[WARN] Could not find kinematic exit; using t_sim=0 as reference.")
        t_kin_a = t_kin_a or 0.0
        t_kin_b = t_kin_b or 0.0

    bkts_a = _bucket(rows_a, t_kin_a, window_s)
    bkts_b = _bucket(rows_b, t_kin_b, window_s)

    all_keys = sorted(set(bkts_a) | set(bkts_b))

    # ── Header ────────────────────────────────────────────────────────────────
    col_w = 14
    print(f"\n{'Window':>8}  {'t_rel':>7}  "
          f"{'alt(A)':>{col_w}} {'alt(B)':>{col_w}} {'dAlt':>9}  "
          f"{'orb(A)':>{col_w}} {'orb(B)':>{col_w}} {'dOrb':>9}  "
          f"{'yawGap(A)':>{col_w}} {'yawGap(B)':>{col_w}} {'dYaw':>9}  "
          f"{'col(A)':>{col_w}} {'col(B)':>{col_w}} {'dCol':>9}  "
          f"{'tens(A)':>{col_w}} {'tens(B)':>{col_w}}")
    print("-" * 170)

    first_diverge = None
    metrics_a_list: list[dict] = []
    metrics_b_list: list[dict] = []
    t_rels: list[float] = []

    for key in all_keys:
        t_rel = key * window_s
        ma = _stats(bkts_a.get(key, []))
        mb = _stats(bkts_b.get(key, []))

        metrics_a_list.append(ma)
        metrics_b_list.append(mb)
        t_rels.append(t_rel)

        def _v(m: dict, k: str, fmt: str = ".2f") -> str:
            v = m.get(k)
            return f"{v:{fmt}}" if v is not None else "  ---"

        d_alt  = _diff_flag(ma.get("alt"),     mb.get("alt"),     _ALT_DIFF_WARN,   ".2f")
        d_orb  = _diff_flag(ma.get("orb_r"),   mb.get("orb_r"),   _ORB_DIFF_WARN,   ".2f")
        d_yaw  = _diff_flag(ma.get("yaw_gap"), mb.get("yaw_gap"), _YAW_DIFF_WARN,   ".1f")
        d_col  = _diff_flag(ma.get("col"),     mb.get("col"),     _COL_DIFF_WARN,   ".3f")

        diverged = ("*" in d_alt or "*" in d_orb or "*" in d_yaw or "*" in d_col)
        if diverged and first_diverge is None:
            first_diverge = t_rel

        row = (
            f"{key:>8}  {t_rel:>7.0f}  "
            f"{_v(ma, 'alt'):>{col_w}} {_v(mb, 'alt'):>{col_w}} {d_alt:>9}  "
            f"{_v(ma, 'orb_r'):>{col_w}} {_v(mb, 'orb_r'):>{col_w}} {d_orb:>9}  "
            f"{_v(ma, 'yaw_gap'):>{col_w}} {_v(mb, 'yaw_gap'):>{col_w}} {d_yaw:>9}  "
            f"{_v(ma, 'col'):>{col_w}} {_v(mb, 'col'):>{col_w}} {d_col:>9}  "
            f"{_v(ma, 'tension'):>{col_w}} {_v(mb, 'tension'):>{col_w}}"
        )
        marker = " <-- FIRST DIVERGE" if diverged and t_rel == first_diverge else ""
        print(row + marker)

    print("-" * 170)

    # ── Summary ────────────────────────────────────────────────────────────────
    print(f"\nRun A: {name_a}")
    print(f"Run B: {name_b}")

    if first_diverge is not None:
        print(f"\nFIRST DIVERGENCE at t_rel = {first_diverge:.0f} s after kinematic exit")
        print("  (t_rel = 0 is kinematic exit; negative = during kinematic)")
    else:
        print("\nNo significant divergence detected within the observed window.")

    # ── Per-run summary stats ──────────────────────────────────────────────────
    def _run_summary(rows: list[TelRow], label: str, t_kin: float) -> None:
        free_rows = [r for r in rows if r.damp_alpha == 0.0]
        if not free_rows:
            print(f"\n{label}: no free-flight rows")
            return
        alts    = [-r.pos_z for r in free_rows]
        yaw_gaps= [math.degrees(r.orb_yaw_rad - r.rpy_yaw) for r in free_rows]
        tens    = [r.tether_tension for r in free_rows]
        cols    = [r.collective_rad for r in free_rows]
        tlens   = [r.tether_length for r in free_rows]
        phases  = [r.phase for r in free_rows if r.phase]
        print(f"\n{label}:")
        print(f"  Free-flight rows : {len(free_rows)}")
        print(f"  Altitude         : min={min(alts):.2f}  max={max(alts):.2f}  "
              f"mean={sum(alts)/len(alts):.2f} m")
        print(f"  Yaw gap          : min={min(yaw_gaps):.1f}  max={max(yaw_gaps):.1f}  "
              f"mean={sum(yaw_gaps)/len(yaw_gaps):.1f} deg")
        print(f"  Tension          : min={min(tens):.0f}  max={max(tens):.0f}  "
              f"mean={sum(tens)/len(tens):.0f} N")
        print(f"  Collective       : min={min(cols):.3f}  max={max(cols):.3f}  "
              f"mean={sum(cols)/len(cols):.3f} rad")
        print(f"  Tether length    : min={min(tlens):.2f}  max={max(tlens):.2f} m")
        if phases:
            from collections import Counter
            phase_counts = Counter(phases)
            print(f"  Phases seen      : {dict(phase_counts)}")

    _run_summary(rows_a, f"Run A [{name_a}]", t_kin_a)
    _run_summary(rows_b, f"Run B [{name_b}]", t_kin_b)

    # ── Optional plot ──────────────────────────────────────────────────────────
    if do_plot:
        _plot(rows_a, rows_b, name_a, name_b, t_kin_a, t_kin_b)


def _plot(rows_a: list[TelRow], rows_b: list[TelRow],
          label_a: str, label_b: str,
          t_kin_a: float, t_kin_b: float) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib not available -- skipping plot")
        return

    def _ts(rows: list[TelRow], t_kin: float) -> list[float]:
        return [r.t_sim - t_kin for r in rows if r.damp_alpha == 0.0]

    ta = _ts(rows_a, t_kin_a)
    tb = _ts(rows_b, t_kin_b)

    def _get(rows: list[TelRow], attr: str) -> list[float]:
        return [getattr(r, attr) for r in rows if r.damp_alpha == 0.0]

    alt_a = [-r.pos_z for r in rows_a if r.damp_alpha == 0.0]
    alt_b = [-r.pos_z for r in rows_b if r.damp_alpha == 0.0]
    orb_a = [math.sqrt(r.pos_x**2 + r.pos_y**2) for r in rows_a if r.damp_alpha == 0.0]
    orb_b = [math.sqrt(r.pos_x**2 + r.pos_y**2) for r in rows_b if r.damp_alpha == 0.0]
    yaw_a = [math.degrees(r.orb_yaw_rad - r.rpy_yaw) for r in rows_a if r.damp_alpha == 0.0]
    yaw_b = [math.degrees(r.orb_yaw_rad - r.rpy_yaw) for r in rows_b if r.damp_alpha == 0.0]
    col_a = _get(rows_a, "collective_rad")
    col_b = _get(rows_b, "collective_rad")
    ten_a = _get(rows_a, "tether_tension")
    ten_b = _get(rows_b, "tether_tension")
    tln_a = _get(rows_a, "tether_length")
    tln_b = _get(rows_b, "tether_length")

    fig, axes = plt.subplots(5, 1, figsize=(14, 18), sharex=True)
    fig.suptitle(f"Comparison: {label_a}  vs  {label_b}\n(t=0 = kinematic exit)")

    ax = axes[0]
    ax.plot(ta, alt_a, label=label_a, color="steelblue")
    ax.plot(tb, alt_b, label=label_b, color="tomato", alpha=0.8)
    ax.set_ylabel("Altitude [m]")
    ax.axhline(1.0, color="k", linestyle="--", linewidth=0.7, label="crash floor (1m)")
    ax.legend(fontsize=8)
    ax.grid(True)

    ax = axes[1]
    ax.plot(ta, orb_a, color="steelblue")
    ax.plot(tb, orb_b, color="tomato", alpha=0.8)
    ax.set_ylabel("Orbit radius [m]")
    ax.axhline(5.0, color="orange", linestyle="--", linewidth=0.7, label="5m threshold")
    ax.legend(fontsize=8)
    ax.grid(True)

    ax = axes[2]
    ax.plot(ta, yaw_a, color="steelblue")
    ax.plot(tb, yaw_b, color="tomato", alpha=0.8)
    ax.set_ylabel("Yaw gap [deg]\n(orb_yaw - rpy_yaw)")
    ax.axhline( 15, color="orange", linestyle="--", linewidth=0.7)
    ax.axhline(-15, color="orange", linestyle="--", linewidth=0.7)
    ax.grid(True)

    ax = axes[3]
    ax.plot(ta, col_a, color="steelblue")
    ax.plot(tb, col_b, color="tomato", alpha=0.8)
    ax.set_ylabel("Collective [rad]")
    ax.axhline(-0.18, color="gray", linestyle=":", linewidth=0.8, label="COL_CRUISE_FLIGHT")
    ax.axhline(-0.20, color="gray", linestyle="--", linewidth=0.8, label="COL_REEL_OUT")
    ax.legend(fontsize=7)
    ax.grid(True)

    ax = axes[4]
    ax.plot(ta, ten_a, color="steelblue", label="tension A")
    ax.plot(tb, ten_b, color="tomato", alpha=0.8, label="tension B")
    ax2 = ax.twinx()
    ax2.plot(ta, tln_a, color="steelblue", linestyle="--", alpha=0.5, label="tlen A")
    ax2.plot(tb, tln_b, color="tomato",    linestyle="--", alpha=0.5, label="tlen B")
    ax2.set_ylabel("Tether length [m]", color="gray")
    ax.set_ylabel("Tension [N]")
    ax.set_xlabel("t_rel [s] from kinematic exit")
    ax.legend(fontsize=7, loc="upper left")
    ax.grid(True)

    plt.tight_layout()
    out = _LOG_DIR / "compare_runs.png"
    plt.savefig(out, dpi=120)
    print(f"\nPlot saved: {out}")
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_a", help="First test log directory name (in simulation/logs/)")
    parser.add_argument("run_b", help="Second test log directory name (in simulation/logs/)")
    parser.add_argument("--window", type=float, default=5.0,
                        help="Averaging window in seconds (default: 5.0)")
    parser.add_argument("--plot", action="store_true", help="Save comparison plot")
    args = parser.parse_args()
    compare(args.run_a, args.run_b, window_s=args.window, do_plot=args.plot)
