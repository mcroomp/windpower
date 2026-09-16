#!/usr/bin/env python3
"""
diagnose_takeoff.py -- Diagnose test_ground_liftoff (MODE_TAKEOFF -> MODE_STEADY handoff).

Purpose-built companion for tests/simtests/test_ground_liftoff.py.  Loads the
telemetry CSV that test produces (run with RAWES_TEL_HZ set high for full
physics-step resolution -- the default 20 Hz decimation hides fast
slack/taut tether cycling) and reports:

  1. Event timeline: liftoff, takeoff->steady transition (altitude threshold
     crossing), first tether break-load exceedance, floor hit, and the
     FIRST ANOMALY (earliest of break-load breach / floor hit).
  2. Oscillation diagnostics, but ONLY in the window from transition up to
     shortly after the first anomaly -- once something has gone wrong the
     downstream chaotic fallout is not useful signal for root-causing the
     onset, so it is deliberately excluded by default (use --full to see
     the whole post-transition run anyway).
  3. A full-resolution windowed table around a requested time span (default:
     transition through shortly after the first anomaly), formatted as a
     fixed-width table with a single header line so columns line up across
     rows -- this is deliberately NOT a repeated "key=value" dump, since a
     real table is much easier to scan for trends/onsets. Includes the
     wind-aligned downwind/crosswind position & velocity columns so reel-out
     direction/behavior can be checked directly (does the kite actually
     move/reel out downwind after transition?).

Usage:
  RAWES_TEL_HZ=400 .venv/Scripts/python.exe -m pytest tests/simtests/test_ground_liftoff.py -s
  .venv/Scripts/python.exe analysis/diagnose_takeoff.py
  .venv/Scripts/python.exe analysis/diagnose_takeoff.py --test test_ground_liftoff --start 1.7 --end 2.2
  .venv/Scripts/python.exe analysis/diagnose_takeoff.py --alt-threshold 10.0 --break-load 620.0
  .venv/Scripts/python.exe analysis/diagnose_takeoff.py --full   # don't truncate at first anomaly
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import simulation as _simulation_pkg

_SIM_DIR = Path(_simulation_pkg.__file__).resolve().parent  # simulation/
_LOG_DIR = _SIM_DIR / "logs"

from simulation.telemetry_csv import read_csv, TelRow  # noqa: E402
import math  # noqa: E402

# ── Defaults (mirror tests/simtests/test_ground_liftoff.py constants) ───────
DEFAULT_TEST = "test_ground_liftoff"
LIFTOFF_ALT_M = 2.0
TAKEOFF_MIN_ALT_M = 10.0
BREAK_LOAD_N = 620.0
WINDOW_SPAN_S = 0.5  # default table window half-span around a requested --start
ANOMALY_POST_MARGIN_S = 1.0  # how far past the first anomaly to still show, by default
MAX_TABLE_ROWS = 150  # auto-decimate the row dump above this many rows

_HEADER = (
    f"{'t_sim':>8}  {'alt_m':>7}  {'vz_mps':>7}  {'down_m':>8}  {'cross_m':>8}  "
    f"{'vdown':>7}  {'vcross':>7}  {'T_N':>8}  {'L_m':>7}  {'Lrest_m':>7}  "
    f"{'ext_m':>7}  {'slk':>3}  {'coll':>7}  {'omega':>6}  {'elev_deg':>8}  "
    f"{'bz_err':>7}  phase"
)


def _fmt_row(r: TelRow) -> str:
    return (
        f"{r.t_sim:8.4f}  {-r.pos_z:7.3f}  {r.vel_z:7.3f}  {r.pos_downwind_m:8.3f}  "
        f"{r.pos_crosswind_m:8.3f}  {r.vel_downwind_mps:7.3f}  {r.vel_crosswind_mps:7.3f}  "
        f"{r.tether_tension:8.2f}  {r.tether_length:7.3f}  {r.tether_rest_length:7.3f}  "
        f"{r.tether_extension:+7.4f}  {r.tether_slack:>3}  {r.collective_rad:+7.4f}  "
        f"{r.omega_rotor:6.2f}  {math.degrees(r.elevation_rad):8.2f}  "
        f"{r.body_z_err_deg:7.2f}  {r.phase}"
    )


def _find_event(rows: list[TelRow], pred) -> "TelRow | None":
    for r in rows:
        if pred(r):
            return r
    return None


def find_first_anomaly(
    rows: list[TelRow], break_load: float
) -> tuple[str | None, "TelRow | None"]:
    """Earliest of {break-load breach, floor hit} -- the first sign something
    has gone wrong. Returns (label, row) or (None, None) if neither occurs."""
    breach = _find_event(rows, lambda r: r.tether_tension >= break_load)
    floor_hit = _find_event(rows, lambda r: -r.pos_z <= 0.0 and r.t_sim > 0.0)
    candidates = [("break_load_breach", breach), ("floor_hit", floor_hit)]
    candidates = [(label, r) for label, r in candidates if r is not None]
    if not candidates:
        return None, None
    label, row = min(candidates, key=lambda lr: lr[1].t_sim)
    return label, row


def print_timeline(rows: list[TelRow], alt_threshold: float, break_load: float) -> dict:
    """Print the event timeline and return key event rows/indices for later use."""
    liftoff = _find_event(rows, lambda r: -r.pos_z >= LIFTOFF_ALT_M)
    transition = _find_event(rows, lambda r: -r.pos_z >= alt_threshold)
    anomaly_label, anomaly_row = find_first_anomaly(rows, break_load)

    print("-- Event timeline --------------------------------------------------")
    print(
        f"  rows loaded       : {len(rows)}  (t_sim {rows[0].t_sim:.4f} .. {rows[-1].t_sim:.4f})"
        if rows
        else "  rows loaded       : 0"
    )
    if len(rows) >= 2:
        dt_med = rows[1].t_sim - rows[0].t_sim
        print(
            f"  logged dt         : ~{dt_med * 1000:.2f} ms  (~{1.0 / dt_med:.1f} Hz) -- set RAWES_TEL_HZ higher if this is too coarse"
        )
    print(
        f"  liftoff (alt>={LIFTOFF_ALT_M}m)   : "
        + (f"t={liftoff.t_sim:.3f}s" if liftoff else "NEVER")
    )
    print(
        f"  transition (alt>={alt_threshold}m): "
        + (f"t={transition.t_sim:.3f}s" if transition else "NEVER")
    )
    if anomaly_row is not None:
        print(
            f"  FIRST ANOMALY     : {anomaly_label} at t={anomaly_row.t_sim:.3f}s "
            f"(T={anomaly_row.tether_tension:.1f}N, alt={-anomaly_row.pos_z:.2f}m)"
        )
        print(
            "                      -- analysis below is truncated shortly after this; "
            "downstream chaotic fallout is not useful onset signal (use --full to override)"
        )
    else:
        print("  FIRST ANOMALY     : none detected (no break-load breach or floor hit)")
    print()
    return dict(
        liftoff=liftoff,
        transition=transition,
        anomaly_label=anomaly_label,
        anomaly=anomaly_row,
    )


def print_oscillation_stats(rows: list[TelRow], start_t: float, end_t: float) -> None:
    """Slack<->taut flip rate and tension/velocity extremes in [start_t, end_t]."""
    window = [r for r in rows if start_t <= r.t_sim <= end_t]
    if len(window) < 2:
        print("-- Oscillation stats: not enough rows in window ---------------------")
        return
    flips = sum(
        1 for a, b in zip(window, window[1:]) if a.tether_slack != b.tether_slack
    )
    dur = window[-1].t_sim - window[0].t_sim
    tensions = [r.tether_tension for r in window]
    vzs = [r.vel_z for r in window]
    print(
        f"-- Oscillation stats (t={start_t:.3f}s .. {end_t:.3f}s) -----------------------"
    )
    print(
        f"  window            : t={window[0].t_sim:.3f}s .. {window[-1].t_sim:.3f}s  ({dur:.3f}s, {len(window)} rows)"
    )
    print(f"  slack/taut flips  : {flips}  ({flips / max(dur, 1e-6):.1f} flips/s)")
    print(f"  tension           : min={min(tensions):.1f}N  max={max(tensions):.1f}N")
    print(f"  vel_z             : min={min(vzs):.2f}m/s  max={max(vzs):.2f}m/s")
    print()


def print_reelout_check(rows: list[TelRow], transition_t: float, end_t: float) -> None:
    """Focused check: right after transition, is the kite actually reeling
    out and drifting downwind as expected (not stalling/reversing)?"""
    window = [r for r in rows if transition_t <= r.t_sim <= end_t]
    if len(window) < 2:
        print("-- Reel-out/downwind check: not enough rows in window ---------------")
        return
    start, end = window[0], window[-1]
    d_rest = end.tether_rest_length - start.tether_rest_length
    d_downwind = end.pos_downwind_m - start.pos_downwind_m
    print(
        f"-- Reel-out/downwind check (t={start.t_sim:.3f}s .. {end.t_sim:.3f}s) --------------"
    )
    print(
        f"  tether_rest_length: {start.tether_rest_length:.3f}m -> {end.tether_rest_length:.3f}m  (d={d_rest:+.3f}m)"
        + (
            "  [paying out]"
            if d_rest > 0
            else "  [REELING IN -- unexpected]"
            if d_rest < 0
            else "  [no change]"
        )
    )
    print(
        f"  pos_downwind_m    : {start.pos_downwind_m:.3f}m -> {end.pos_downwind_m:.3f}m  (d={d_downwind:+.3f}m)"
        + (
            "  [drifting downwind]"
            if d_downwind > 0
            else "  [drifting UPWIND -- unexpected]"
            if d_downwind < 0
            else "  [no change]"
        )
    )
    print(
        f"  vel_downwind_mps  : min={min(r.vel_downwind_mps for r in window):.3f}  max={max(r.vel_downwind_mps for r in window):.3f}"
    )
    print(
        f"  pos_crosswind_m   : {start.pos_crosswind_m:.3f}m -> {end.pos_crosswind_m:.3f}m"
    )
    print()


def print_window(rows: list[TelRow], start: float, end: float, stride: int) -> None:
    windowed = [r for r in rows if start <= r.t_sim <= end][::stride]
    stride_note = f"  (every {stride}th row)" if stride > 1 else ""
    print(
        f"-- Window t={start:.3f}s .. {end:.3f}s ({len(windowed)} rows){stride_note} --------------------"
    )
    print(_HEADER)
    for r in windowed:
        print(_fmt_row(r))
    print()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--test",
        default=DEFAULT_TEST,
        help="test name (simulation/logs/<test>/telemetry.csv)",
    )
    ap.add_argument(
        "--alt-threshold",
        type=float,
        default=TAKEOFF_MIN_ALT_M,
        help="takeoff->steady altitude threshold [m]",
    )
    ap.add_argument(
        "--break-load",
        type=float,
        default=BREAK_LOAD_N,
        help="tether break-load threshold [N]",
    )
    ap.add_argument(
        "--start",
        type=float,
        default=None,
        help="window start [s] for detailed row dump (default: around transition)",
    )
    ap.add_argument(
        "--end", type=float, default=None, help="window end [s] for detailed row dump"
    )
    ap.add_argument(
        "--span",
        type=float,
        default=WINDOW_SPAN_S,
        help="half-span [s] used when only --start is given, or around transition by default",
    )
    ap.add_argument(
        "--stride",
        type=int,
        default=None,
        help="print every Nth row (default: auto, to keep the table under ~%d rows)"
        % MAX_TABLE_ROWS,
    )
    ap.add_argument(
        "--full",
        action="store_true",
        help="do not truncate stats/window at the first anomaly -- show the whole post-transition run",
    )
    ap.add_argument(
        "--no-window",
        action="store_true",
        help="skip the detailed row dump, print timeline/stats only",
    )
    args = ap.parse_args()

    path = _LOG_DIR / args.test / "telemetry.csv"
    rows = read_csv(path)
    if not rows:
        print(f"No telemetry found at {path}", file=sys.stderr)
        return 1

    events = print_timeline(rows, args.alt_threshold, args.break_load)

    anchor_t = events["transition"].t_sim if events["transition"] else rows[0].t_sim
    anomaly = events["anomaly"]
    if anomaly is not None and not args.full:
        stats_end = anomaly.t_sim + ANOMALY_POST_MARGIN_S
    else:
        stats_end = rows[-1].t_sim
    print_oscillation_stats(rows, anchor_t, stats_end)
    # Reel-out/downwind check stops AT the anomaly itself (not +margin) -- this
    # check is specifically about clean pre-anomaly behavior; including the
    # chaotic aftermath would contaminate the drift-direction read.
    reelout_end = (
        anomaly.t_sim if (anomaly is not None and not args.full) else stats_end
    )
    print_reelout_check(rows, anchor_t, reelout_end)

    if not args.no_window:
        if args.start is not None:
            start = args.start
            end = args.end if args.end is not None else start + args.span
        else:
            start = max(rows[0].t_sim, anchor_t - args.span)
            end = min(
                rows[-1].t_sim,
                anchor_t + args.span,
                stats_end if not args.full else rows[-1].t_sim,
            )
        stride = args.stride
        if stride is None:
            n_rows = sum(1 for r in rows if start <= r.t_sim <= end)
            stride = max(1, n_rows // MAX_TABLE_ROWS)
        print_window(rows, start, end, stride)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
