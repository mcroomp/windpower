"""
compare_pumping.py -- Side-by-side comparison of Python vs Lua pumping telemetry.

Loads two 400 Hz telemetry CSVs (test_pump_cycle_unified and test_lua_pumping_unified)
and prints a table highlighting where they diverge, focusing on the first cycle reel-out.

Usage:
    python simulation/analysis/compare_pumping.py [--phase cycle1_reel-out] [--bucket 1]
    python simulation/analysis/compare_pumping.py --py logs/... --lua logs/...
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

_SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM))

_LOG = _SIM / "logs"

KEY_COLS = [
    "tether_tension",
    "collective_rad",
    "collective_from_tension_ctrl",
    "vib_corr",
    "ten_pi_integral",
    "tension_setpoint",
    "omega_rotor",
    "aero_T",
    "aero_v_inplane",
    "aero_v_axial",
    "pos_z",
    "vel_z",
    "tether_rest_length",
    "winch_speed_ms",
]


def _load(path: Path) -> list[dict]:
    rows = []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            rows.append(row)
    return rows


def _flt(row: dict, col: str) -> float:
    v = row.get(col, "")
    try:
        return float(v)
    except (ValueError, TypeError):
        return float("nan")


def _bucket_rows(rows: list[dict], phase_filter: str,
                 bucket_s: float) -> list[tuple[float, dict]]:
    """Average rows within each time bucket, filtered by phase."""
    from collections import defaultdict
    buckets: dict[int, list[dict]] = defaultdict(list)
    for row in rows:
        if phase_filter and row.get("phase", "") != phase_filter:
            continue
        t = _flt(row, "t_sim")
        b = int(t / bucket_s)
        buckets[b].append(row)
    result = []
    for b in sorted(buckets):
        group = buckets[b]
        t_mid = b * bucket_s + bucket_s / 2
        avg = {col: sum(_flt(r, col) for r in group) / len(group) for col in KEY_COLS}
        result.append((t_mid, avg))
    return result


def _print_comparison(py_rows: list[dict], lua_rows: list[dict],
                      phase: str, bucket_s: float) -> None:
    py_buck  = _bucket_rows(py_rows,  phase, bucket_s)
    lua_buck = _bucket_rows(lua_rows, phase, bucket_s)

    if not py_buck:
        print(f"  [!] No rows for phase '{phase}' in Python telemetry")
        return
    if not lua_buck:
        print(f"  [!] No rows for phase '{phase}' in Lua telemetry")
        return

    # Align by nearest bucket index
    py_dict  = {round(t / bucket_s): v for t, v in py_buck}
    lua_dict = {round(t / bucket_s): v for t, v in lua_buck}
    common   = sorted(set(py_dict) & set(lua_dict))

    if not common:
        print("  [!] No overlapping time buckets between Python and Lua")
        return

    # Print header
    col_w = 12
    hdr = f"{'t_s':>6}  " + "  ".join(f"{'PY_'+c[:8]:>{col_w}} {'LU_'+c[:8]:>{col_w}} {'DIFF':>8}"
                                        for c in KEY_COLS[:6])
    print(hdr)
    print("-" * len(hdr))

    for b in common:
        t = b * bucket_s
        py_v  = py_dict[b]
        lua_v = lua_dict[b]
        line = f"{t:6.1f}  "
        for col in KEY_COLS[:6]:
            pv = py_v[col]
            lv = lua_v[col]
            diff = lv - pv
            line += f"  {pv:{col_w}.3f} {lv:{col_w}.3f} {diff:+8.3f}"
        print(line)

    # Summary: print mean absolute difference per column
    print()
    print("  Mean absolute difference per column:")
    diffs = {col: [] for col in KEY_COLS}
    for b in common:
        py_v  = py_dict[b]
        lua_v = lua_dict[b]
        for col in KEY_COLS:
            diffs[col].append(abs(lua_v[col] - py_v[col]))
    for col in KEY_COLS:
        vals = diffs[col]
        if vals:
            mean_d = sum(vals) / len(vals)
            max_d  = max(vals)
            print(f"    {col:<35}  mean={mean_d:8.4f}  max={max_d:8.4f}")


def _print_time_series(py_rows: list[dict], lua_rows: list[dict],
                       phase: str, t_start: float, t_end: float,
                       cols: list[str]) -> None:
    """Print raw 400 Hz rows side-by-side for a narrow time window."""
    def _filter(rows):
        return [r for r in rows
                if (not phase or r.get("phase", "") == phase)
                and t_start <= _flt(r, "t_sim") <= t_end]

    py_win  = _filter(py_rows)
    lua_win = _filter(lua_rows)

    print(f"\n  Raw rows  t=[{t_start:.2f}, {t_end:.2f}]  phase={phase!r}")
    print(f"  {'t_sim':>7}  " + "  ".join(f"{'PY_'+c[:10]:>13} {'LU_'+c[:10]:>13}" for c in cols))
    print("  " + "-" * (7 + 16 * len(cols)))

    # pair by closest time
    import bisect
    lua_times = [_flt(r, "t_sim") for r in lua_win]
    for r_py in py_win[::4]:   # subsample 4x so output isn't enormous
        t = _flt(r_py, "t_sim")
        idx = bisect.bisect_left(lua_times, t)
        if idx >= len(lua_win):
            idx = len(lua_win) - 1
        r_lua = lua_win[idx]
        line = f"  {t:7.4f}  "
        for col in cols:
            pv = _flt(r_py, col)
            lv = _flt(r_lua, col)
            line += f"  {pv:13.4f} {lv:13.4f}"
        print(line)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--py",    default=str(_LOG / "test_pumping_unified" / "telemetry.csv"))
    ap.add_argument("--lua",   default=str(_LOG / "test_lua_pumping_unified" / "telemetry.csv"))
    ap.add_argument("--phase", default="cycle1_reel-out")
    ap.add_argument("--bucket", type=float, default=2.0)
    ap.add_argument("--window", default=None,
                    help="Narrow time window for raw comparison, e.g. '0,10'")
    args = ap.parse_args()

    py_path  = Path(args.py)
    lua_path = Path(args.lua)

    if not py_path.exists():
        print(f"[!] Python telemetry not found: {py_path}")
        sys.exit(1)
    if not lua_path.exists():
        print(f"[!] Lua telemetry not found: {lua_path}")
        sys.exit(1)

    print(f"Python: {py_path}")
    print(f"Lua:    {lua_path}")
    print()

    py_rows  = _load(py_path)
    lua_rows = _load(lua_path)
    print(f"Rows: py={len(py_rows)}  lua={len(lua_rows)}")

    # Per-phase summaries
    phases = ["cycle1_reel-out", "cycle1_transition", "cycle1_reel-in"]
    for phase in phases:
        py_n  = sum(1 for r in py_rows  if r.get("phase","") == phase)
        lua_n = sum(1 for r in lua_rows if r.get("phase","") == phase)
        print(f"  {phase:<25}  py={py_n:5d}  lua={lua_n:5d}")
    print()

    # Main comparison table
    print(f"=== Bucketed comparison  phase={args.phase!r}  bucket={args.bucket}s ===")
    _print_comparison(py_rows, lua_rows, args.phase, args.bucket)

    # Narrow raw window if requested
    if args.window:
        t0, t1 = [float(x) for x in args.window.split(",")]
        focus_cols = ["tether_tension", "collective_rad",
                      "collective_from_tension_ctrl", "vib_corr",
                      "ten_pi_integral", "omega_rotor"]
        _print_time_series(py_rows, lua_rows, args.phase, t0, t1, focus_cols)


if __name__ == "__main__":
    main()
