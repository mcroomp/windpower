"""
Compare pumping-cycle simtest vs SITL outputs and diagnose divergence.

What this script does:
1. Loads both telemetry CSV files.
2. Aligns SITL time to hold release (kinematic_exit) by default.
3. Compares per-step controller signals (targets, errors, contributions).
4. Compares params.json files (effective startup params) and highlights
   controller-relevant mismatches.
5. Prints a compact diagnostic summary with likely divergence contributors.

Usage:
    python simulation/analysis/compare_pumping_sitl_simtest.py

    python simulation/analysis/compare_pumping_sitl_simtest.py \
        --simtest-dir simulation/logs/test_lua_pumping_unified \
        --sitl-dir simulation/logs/test_pumping_cycle_lua_sitl
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable


DEFAULT_SIMTEST_DIR = Path("simulation/logs/test_lua_pumping_unified")
DEFAULT_SITL_DIR = Path("simulation/logs/test_pumping_cycle_lua_sitl")

# Requested comparison focus: targets, errors, and contribution terms only.
CONTROL_COLS = [
    "roll_sp_rads",
    "pitch_sp_rads",
    "yaw_sp_rads",
    "roll_rate_err_rads",
    "pitch_rate_err_rads",
    "yaw_rate_err_rads",
    "rate_roll_p_contrib",
    "rate_roll_i_contrib",
    "rate_roll_d_contrib",
    "rate_roll_ff_contrib",
    "rate_pitch_p_contrib",
    "rate_pitch_i_contrib",
    "rate_pitch_d_contrib",
    "rate_pitch_ff_contrib",
    "rate_yaw_p_contrib",
    "rate_yaw_i_contrib",
    "rate_yaw_d_contrib",
    "rate_yaw_ff_contrib",
    "lua_ol_alt_p_contrib",
    "lua_ol_alt_i_contrib",
    "lua_ol_alt_d_contrib",
    "lua_ol_col_cmd_rad",
]

PARAM_PREFIX_FOCUS = (
    "ATC_",
    "H_",
    "RAWES_",
    "EK3_",
    "INS_",
    "GPS_",
    "SIM_",
)


def _f(v: object, default: float = float("nan")) -> float:
    if v is None:
        return default
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def _read_json(path: Path) -> dict[str, float]:
    return json.loads(path.read_text(encoding="utf-8"))


def _align_sitl_release(rows: list[dict[str, str]]) -> tuple[list[dict[str, str]], float]:
    """Trim and rebase SITL rows so t=0 starts at kinematic hold release."""
    if not rows:
        return rows, 0.0

    release_t = None
    for r in rows:
        note = (r.get("note") or "").strip()
        if note == "kinematic_exit":
            release_t = _f(r.get("t_sim"), 0.0)
            break

    if release_t is None:
        # Fallback for logs lacking explicit note marker.
        for r in rows:
            if _f(r.get("damp_alpha"), 0.0) <= 0.0:
                release_t = _f(r.get("t_sim"), 0.0)
                break

    if release_t is None:
        release_t = _f(rows[0].get("t_sim"), 0.0)

    out: list[dict[str, str]] = []
    for r in rows:
        t = _f(r.get("t_sim"), 0.0)
        if t < release_t:
            continue
        nr = dict(r)
        nr["t_sim"] = f"{t - release_t:.6f}"
        out.append(nr)
    return out, release_t


def _bucket_mean(rows: list[dict[str, str]], cols: Iterable[str], bucket_s: float) -> dict[int, dict[str, float]]:
    buckets: dict[int, dict[str, list[float]]] = {}
    for r in rows:
        t = _f(r.get("t_sim"), float("nan"))
        if not math.isfinite(t):
            continue
        bi = int(t / bucket_s)
        if bi not in buckets:
            buckets[bi] = {c: [] for c in cols}
        for c in cols:
            v = _f(r.get(c), float("nan"))
            if math.isfinite(v):
                buckets[bi][c].append(v)

    out: dict[int, dict[str, float]] = {}
    for bi, by_col in buckets.items():
        out[bi] = {}
        for c, vals in by_col.items():
            out[bi][c] = sum(vals) / len(vals) if vals else float("nan")
    return out


def _compare_control(sim_rows: list[dict[str, str]], sitl_rows: list[dict[str, str]], bucket_s: float) -> dict[str, dict[str, float]]:
    sim_b = _bucket_mean(sim_rows, CONTROL_COLS, bucket_s)
    sitl_b = _bucket_mean(sitl_rows, CONTROL_COLS, bucket_s)
    common = sorted(set(sim_b) & set(sitl_b))

    stats: dict[str, dict[str, float]] = {}
    for c in CONTROL_COLS:
        diffs: list[float] = []
        for bi in common:
            sv = sim_b[bi][c]
            tv = sitl_b[bi][c]
            if math.isfinite(sv) and math.isfinite(tv):
                diffs.append(tv - sv)
        if not diffs:
            stats[c] = {"count": 0.0, "mean_abs": float("nan"), "max_abs": float("nan"), "mean": float("nan")}
            continue
        mean = sum(diffs) / len(diffs)
        abs_vals = [abs(d) for d in diffs]
        stats[c] = {
            "count": float(len(diffs)),
            "mean_abs": sum(abs_vals) / len(abs_vals),
            "max_abs": max(abs_vals),
            "mean": mean,
        }
    return stats


def _compare_params(sim_p: dict[str, float], sitl_p: dict[str, float], tol: float) -> list[tuple[str, float, float, float]]:
    mismatches: list[tuple[str, float, float, float]] = []
    keys = sorted(set(sim_p) | set(sitl_p))
    for k in keys:
        if not k.startswith(PARAM_PREFIX_FOCUS):
            continue
        sv = _f(sim_p.get(k), float("nan"))
        tv = _f(sitl_p.get(k), float("nan"))
        if not math.isfinite(sv) or not math.isfinite(tv):
            mismatches.append((k, sv, tv, float("nan")))
            continue
        d = tv - sv
        if abs(d) > tol:
            mismatches.append((k, sv, tv, d))
    return mismatches


def _print_top_control(stats: dict[str, dict[str, float]], top_n: int = 12) -> None:
    ranked = [
        (c, s["mean_abs"], s["max_abs"], s["mean"], int(s["count"]))
        for c, s in stats.items()
        if math.isfinite(s["mean_abs"])
    ]
    ranked.sort(key=lambda x: x[1], reverse=True)

    print("\nTop control-signal deltas (SITL - simtest):")
    print(f"{'column':<26} {'mean_abs':>10} {'max_abs':>10} {'mean':>10} {'n':>6}")
    print("-" * 70)
    for c, mean_abs, max_abs, mean, n in ranked[:top_n]:
        print(f"{c:<26} {mean_abs:10.5f} {max_abs:10.5f} {mean:10.5f} {n:6d}")


def _print_param_diff(mismatches: list[tuple[str, float, float, float]], top_n: int = 40) -> None:
    print("\nParameter mismatches (focused prefixes):")
    if not mismatches:
        print("  none (within tolerance)")
        return

    finite = [m for m in mismatches if math.isfinite(m[3])]
    finite.sort(key=lambda x: abs(x[3]), reverse=True)

    print(f"{'param':<26} {'simtest':>12} {'sitl':>12} {'delta':>12}")
    print("-" * 66)
    shown = 0
    for k, sv, tv, d in finite:
        print(f"{k:<26} {sv:12.5g} {tv:12.5g} {d:12.5g}")
        shown += 1
        if shown >= top_n:
            break

    missing = [m for m in mismatches if not math.isfinite(m[3])]
    if missing:
        print("\nMissing/non-numeric params:")
        for k, sv, tv, _ in missing[:top_n]:
            print(f"  {k}: simtest={sv} sitl={tv}")


def _print_initial_state(sim_rows: list[dict[str, str]], sitl_rows: list[dict[str, str]]) -> None:
    print("\nInitial condition snapshot (first aligned row):")
    if not sim_rows or not sitl_rows:
        print("  unavailable")
        return
    s0 = sim_rows[0]
    t0 = sitl_rows[0]
    keys = ["pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "rpy_roll", "rpy_pitch", "rpy_yaw", "tether_rest_length"]
    print(f"{'field':<18} {'simtest':>12} {'sitl':>12} {'delta':>12}")
    print("-" * 58)
    for k in keys:
        sv = _f(s0.get(k), float("nan"))
        tv = _f(t0.get(k), float("nan"))
        d = tv - sv if math.isfinite(sv) and math.isfinite(tv) else float("nan")
        print(f"{k:<18} {sv:12.5g} {tv:12.5g} {d:12.5g}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--simtest-dir", type=Path, default=DEFAULT_SIMTEST_DIR)
    ap.add_argument("--sitl-dir", type=Path, default=DEFAULT_SITL_DIR)
    ap.add_argument("--bucket-s", type=float, default=0.2)
    ap.add_argument("--param-tol", type=float, default=1e-6)
    ap.add_argument("--no-align-release", action="store_true", help="Do not trim SITL to kinematic release")
    args = ap.parse_args()

    sim_tel = args.simtest_dir / "telemetry.csv"
    sitl_tel = args.sitl_dir / "telemetry.csv"
    sim_params = args.simtest_dir / "params.json"
    sitl_params = args.sitl_dir / "params.json"

    if not sim_tel.exists() or not sitl_tel.exists():
        raise SystemExit(f"Missing telemetry file(s): simtest={sim_tel.exists()} sitl={sitl_tel.exists()}")
    if not sim_params.exists() or not sitl_params.exists():
        raise SystemExit(f"Missing params.json file(s): simtest={sim_params.exists()} sitl={sitl_params.exists()}")

    sim_rows = _read_csv(sim_tel)
    sitl_rows = _read_csv(sitl_tel)

    release_t = 0.0
    if not args.no_align_release:
        sitl_rows, release_t = _align_sitl_release(sitl_rows)

    # Rebase simtest time to start from first sample for comparable buckets.
    if sim_rows:
        t0 = _f(sim_rows[0].get("t_sim"), 0.0)
        for r in sim_rows:
            r["t_sim"] = f"{_f(r.get('t_sim'), 0.0) - t0:.6f}"

    print("Pumping compare summary")
    print(f"  simtest dir: {args.simtest_dir}")
    print(f"  sitl dir   : {args.sitl_dir}")
    print(f"  sitl release alignment: {'on' if not args.no_align_release else 'off'} (release_t={release_t:.3f}s)")
    print(f"  rows after alignment: simtest={len(sim_rows)} sitl={len(sitl_rows)}")

    stats = _compare_control(sim_rows, sitl_rows, args.bucket_s)
    _print_top_control(stats)

    sim_p = _read_json(sim_params)
    sitl_p = _read_json(sitl_params)
    mismatches = _compare_params(sim_p, sitl_p, args.param_tol)
    _print_param_diff(mismatches)

    _print_initial_state(sim_rows, sitl_rows)


if __name__ == "__main__":
    main()
