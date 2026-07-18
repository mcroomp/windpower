#!/usr/bin/env python3
"""
compare_runs.py -- Compare two stack test runs telemetry-by-telemetry.

Useful for finding where test_pumping_cycle_lua_sitl diverges from test_lua_flight_steady_sitl
when both share the same startup conditions.

Usage (inside Docker):
  python3 /rawes/simulation/analysis/compare_runs.py \\
      lua_full_test_lua_flight_steady \\
      pumping_lua_test_pumping_cycle_lua

  python3 /rawes/simulation/analysis/compare_runs.py \\
      lua_full_test_lua_flight_steady \\
      pumping_lua_test_pumping_cycle_lua \\
      --plot

The script aligns both runs on kinematic exit time (first row after startup phases end),
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
_REACTION_PRE_S = 1.0
_REACTION_HORIZON_S = 3.0
_KINEMATIC_PHASES = frozenset({"waiting_ekf", "positioning"})


def _is_kinematic_row(r: TelRow) -> bool:
    return str(r.phase or "") in _KINEMATIC_PHASES


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _find_kin_exit_t(rows: list[TelRow]) -> Optional[float]:
    """Return t_sim of first row where startup phase transitions to free-flight."""
    prev = None
    for r in rows:
        if (not _is_kinematic_row(r)) and (prev is None or _is_kinematic_row(prev)):
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


def _mean_finite(vals: list[float]) -> Optional[float]:
    good = [v for v in vals if math.isfinite(v)]
    return (sum(good) / len(good)) if good else None


def _sgn(v: Optional[float], eps: float = 1e-6) -> int:
    if v is None or not math.isfinite(v):
        return 0
    if v > eps:
        return 1
    if v < -eps:
        return -1
    return 0


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


def _bucket_control(rows: list[TelRow], t_ref: float, window_s: float) -> dict[int, dict]:
    """Bucket rate-loop diagnostics after t_ref.

    Returns per-bucket means for error and PID contributions on each axis.
    """
    out: dict[int, dict] = {}
    by_key: dict[int, list[TelRow]] = {}
    for r in rows:
        if _is_kinematic_row(r):
            continue
        dt = r.t_sim - t_ref
        if dt < 0.0:
            continue
        key = int(dt // window_s)
        by_key.setdefault(key, []).append(r)

    axes = [
        ("roll",  "rate_roll_p_contrib",  "rate_roll_i_contrib",  "rate_roll_d_contrib",  "rate_roll_ff_contrib"),
        ("pitch", "rate_pitch_p_contrib", "rate_pitch_i_contrib", "rate_pitch_d_contrib", "rate_pitch_ff_contrib"),
        ("yaw",   "rate_yaw_p_contrib",   "rate_yaw_i_contrib",   "rate_yaw_d_contrib",   "rate_yaw_ff_contrib"),
    ]

    def _axis_error(r: TelRow, axis: str) -> float:
        if axis == "roll":
            return float(r.roll_sp_rads - r.omega_x)
        if axis == "pitch":
            return float(r.pitch_sp_rads - r.omega_y)
        return float(r.yaw_sp_rads - r.omega_z)

    for key, group in by_key.items():
        d: dict = {"n": len(group)}
        for name, p_f, i_f, d_f, ff_f in axes:
            err = _mean_finite([_axis_error(r, name) for r in group])
            p = _mean_finite([getattr(r, p_f) for r in group])
            i = _mean_finite([getattr(r, i_f) for r in group])
            dd = _mean_finite([getattr(r, d_f) for r in group])
            ff = _mean_finite([getattr(r, ff_f) for r in group])
            total = None
            parts = [x for x in [p, i, dd, ff] if x is not None and math.isfinite(x)]
            if parts:
                total = sum(parts)
            d[name] = {
                "err": err,
                "p": p,
                "i": i,
                "d": dd,
                "ff": ff,
                "total": total,
            }
        out[key] = d
    return out


def _print_pid_diagnostics(
    rows_a: list[TelRow], rows_b: list[TelRow],
    label_a: str, label_b: str,
    t_kin_a: float, t_kin_b: float,
    window_s: float,
) -> None:
    """Print whether both runs are correcting the same rate error with similar commands."""
    ca = _bucket_control(rows_a, t_kin_a, window_s)
    cb = _bucket_control(rows_b, t_kin_b, window_s)
    common = sorted(set(ca) & set(cb))
    if not common:
        print("\nPID diagnostics: no overlapping free-flight control buckets.")
        return

    print("\nPID diagnostics (rate loop) -- are both runs correcting the same error?")
    print(f"  Bucket size: {window_s:.1f}s, overlapping buckets: {len(common)}")

    axes = ["roll", "pitch", "yaw"]
    for axis in axes:
        err_sign_match = 0
        cmd_sign_match = 0
        both_correct = 0
        valid_err = 0
        valid_cmd = 0
        valid_correct = 0

        disagree_rows: list[tuple[float, dict, dict]] = []
        abs_means_a = {"p": [], "i": [], "d": [], "ff": []}
        abs_means_b = {"p": [], "i": [], "d": [], "ff": []}

        for key in common:
            a = ca[key][axis]
            b = cb[key][axis]
            t_rel = key * window_s

            for term in ["p", "i", "d", "ff"]:
                va = a.get(term)
                vb = b.get(term)
                if va is not None and math.isfinite(va):
                    abs_means_a[term].append(abs(va))
                if vb is not None and math.isfinite(vb):
                    abs_means_b[term].append(abs(vb))

            se_a = _sgn(a.get("err"))
            se_b = _sgn(b.get("err"))
            if se_a != 0 and se_b != 0:
                valid_err += 1
                if se_a == se_b:
                    err_sign_match += 1

            sc_a = _sgn(a.get("total"))
            sc_b = _sgn(b.get("total"))
            if sc_a != 0 and sc_b != 0:
                valid_cmd += 1
                if sc_a == sc_b:
                    cmd_sign_match += 1

            corr_a = (se_a != 0 and sc_a != 0 and se_a == sc_a)
            corr_b = (se_b != 0 and sc_b != 0 and se_b == sc_b)
            if (se_a != 0 and sc_a != 0 and se_b != 0 and sc_b != 0):
                valid_correct += 1
                if corr_a and corr_b:
                    both_correct += 1
                if (se_a != se_b) or (sc_a != sc_b) or (corr_a != corr_b):
                    disagree_rows.append((t_rel, a, b))

        def _pct(num: int, den: int) -> float:
            return (100.0 * num / den) if den else float("nan")

        dom_a = max(abs_means_a.items(), key=lambda kv: _mean(kv[1]) or -1.0)[0]
        dom_b = max(abs_means_b.items(), key=lambda kv: _mean(kv[1]) or -1.0)[0]

        print(f"\nAxis: {axis}")
        print(
            f"  Error sign match ({label_a} vs {label_b}): "
            f"{err_sign_match}/{valid_err} ({_pct(err_sign_match, valid_err):.1f}%)"
        )
        print(
            f"  Command sign match ({label_a} vs {label_b}): "
            f"{cmd_sign_match}/{valid_cmd} ({_pct(cmd_sign_match, valid_cmd):.1f}%)"
        )
        print(
            f"  Both correcting own error (sign(cmd)=sign(err)): "
            f"{both_correct}/{valid_correct} ({_pct(both_correct, valid_correct):.1f}%)"
        )
        print(
            f"  Dominant mean |term|: {label_a}={dom_a.upper()}  {label_b}={dom_b.upper()}"
        )

        if disagree_rows:
            print("  First disagreement buckets (t_rel s):")
            for t_rel, a, b in disagree_rows[:5]:
                print(
                    f"    t={t_rel:6.1f}  "
                    f"err(A,B)=({a.get('err', float('nan')):+.3f},{b.get('err', float('nan')):+.3f})  "
                    f"cmd(A,B)=({a.get('total', float('nan')):+.3f},{b.get('total', float('nan')):+.3f})"
                )

    print("\nNote: 'correcting own error' uses sign(cmd_total)=sign(rate_error).")


def _rows_in_rel_window(rows: list[TelRow], t_kin: float, t0: float, t1: float) -> list[TelRow]:
    return [r for r in rows if (not _is_kinematic_row(r)) and t0 <= (r.t_sim - t_kin) < t1]


def _mean_attr(rows: list[TelRow], attr: str) -> Optional[float]:
    vals = [getattr(r, attr) for r in rows]
    return _mean_finite(vals)


def _dominant_pid_term(rows: list[TelRow], axis: str) -> str:
    if axis == "roll":
        fields = {
            "P": "rate_roll_p_contrib",
            "I": "rate_roll_i_contrib",
            "D": "rate_roll_d_contrib",
            "FF": "rate_roll_ff_contrib",
        }
    elif axis == "pitch":
        fields = {
            "P": "rate_pitch_p_contrib",
            "I": "rate_pitch_i_contrib",
            "D": "rate_pitch_d_contrib",
            "FF": "rate_pitch_ff_contrib",
        }
    else:
        fields = {
            "P": "rate_yaw_p_contrib",
            "I": "rate_yaw_i_contrib",
            "D": "rate_yaw_d_contrib",
            "FF": "rate_yaw_ff_contrib",
        }
    scored: list[tuple[str, float]] = []
    for name, field in fields.items():
        v = _mean_finite([abs(getattr(r, field)) for r in rows])
        scored.append((name, v if v is not None else -1.0))
    scored.sort(key=lambda x: x[1], reverse=True)
    return scored[0][0]


def _axis_err_cmd(rows: list[TelRow], axis: str) -> tuple[Optional[float], Optional[float]]:
    if axis == "roll":
        e = _mean_finite([r.roll_sp_rads - r.omega_x for r in rows])
        p = _mean_attr(rows, "rate_roll_p_contrib")
        i = _mean_attr(rows, "rate_roll_i_contrib")
        d = _mean_attr(rows, "rate_roll_d_contrib")
        ff = _mean_attr(rows, "rate_roll_ff_contrib")
    elif axis == "pitch":
        e = _mean_finite([r.pitch_sp_rads - r.omega_y for r in rows])
        p = _mean_attr(rows, "rate_pitch_p_contrib")
        i = _mean_attr(rows, "rate_pitch_i_contrib")
        d = _mean_attr(rows, "rate_pitch_d_contrib")
        ff = _mean_attr(rows, "rate_pitch_ff_contrib")
    else:
        e = _mean_finite([r.yaw_sp_rads - r.omega_z for r in rows])
        p = _mean_attr(rows, "rate_yaw_p_contrib")
        i = _mean_attr(rows, "rate_yaw_i_contrib")
        d = _mean_attr(rows, "rate_yaw_d_contrib")
        ff = _mean_attr(rows, "rate_yaw_ff_contrib")
    parts = [x for x in [p, i, d, ff] if x is not None and math.isfinite(x)]
    cmd = sum(parts) if parts else None
    return e, cmd


def _print_first_divergence_reactions(
    rows_a: list[TelRow], rows_b: list[TelRow],
    label_a: str, label_b: str,
    t_kin_a: float, t_kin_b: float,
    first_diverge: Optional[float],
    reaction_pre_s: float,
    reaction_horizon_s: float,
) -> None:
    if first_diverge is None:
        print("\nFirst-divergence reactions: not available (no divergence found).")
        return

    t0 = first_diverge
    pre0 = max(0.0, t0 - reaction_pre_s)
    post1 = t0 + reaction_horizon_s

    a_pre = _rows_in_rel_window(rows_a, t_kin_a, pre0, t0)
    b_pre = _rows_in_rel_window(rows_b, t_kin_b, pre0, t0)
    a_post = _rows_in_rel_window(rows_a, t_kin_a, t0, post1)
    b_post = _rows_in_rel_window(rows_b, t_kin_b, t0, post1)

    print("\nFirst-divergence reactions (pre-noise window)")
    print(f"  First divergence t_rel: {t0:.2f}s")
    print(f"  Window: pre=[{pre0:.2f},{t0:.2f})  post=[{t0:.2f},{post1:.2f})")
    print(f"  Samples: {label_a} pre={len(a_pre)} post={len(a_post)}; {label_b} pre={len(b_pre)} post={len(b_post)}")

    def _safe_delta(post: Optional[float], pre: Optional[float]) -> Optional[float]:
        if post is None or pre is None:
            return None
        return post - pre

    def _fmt(v: Optional[float], fmt: str = "+.3f") -> str:
        if v is None or not math.isfinite(v):
            return "  n/a"
        return format(v, fmt)

    # Command-level reactions likely linked to tension divergence.
    def _cyc_mag(rows: list[TelRow]) -> Optional[float]:
        vals = [math.hypot(r.roll_sp_rads, r.pitch_sp_rads) for r in rows]
        return _mean_finite(vals)

    a_col_pre = _mean_attr(a_pre, "collective_rad")
    a_col_post = _mean_attr(a_post, "collective_rad")
    b_col_pre = _mean_attr(b_pre, "collective_rad")
    b_col_post = _mean_attr(b_post, "collective_rad")

    a_cyc_pre = _cyc_mag(a_pre)
    a_cyc_post = _cyc_mag(a_post)
    b_cyc_pre = _cyc_mag(b_pre)
    b_cyc_post = _cyc_mag(b_post)

    a_ten_pre = _mean_attr(a_pre, "tether_tension")
    a_ten_post = _mean_attr(a_post, "tether_tension")
    b_ten_pre = _mean_attr(b_pre, "tether_tension")
    b_ten_post = _mean_attr(b_post, "tether_tension")

    print("\n  Command/tension reaction summary (post - pre):")
    print(f"    collective_rad  {label_a}: {_fmt(_safe_delta(a_col_post, a_col_pre))}   {label_b}: {_fmt(_safe_delta(b_col_post, b_col_pre))}")
    print(f"    cyclic_sp_mag   {label_a}: {_fmt(_safe_delta(a_cyc_post, a_cyc_pre))}   {label_b}: {_fmt(_safe_delta(b_cyc_post, b_cyc_pre))}")
    print(f"    tether_tension  {label_a}: {_fmt(_safe_delta(a_ten_post, a_ten_pre), '+.1f')}   {label_b}: {_fmt(_safe_delta(b_ten_post, b_ten_pre), '+.1f')}")

    # Axis-wise: are both runs correcting the same error in this early reaction window?
    print("\n  PID reaction at first divergence (post window means):")
    for axis in ["roll", "pitch", "yaw"]:
        e_a, c_a = _axis_err_cmd(a_post, axis)
        e_b, c_b = _axis_err_cmd(b_post, axis)
        corr_a = (_sgn(e_a) != 0 and _sgn(c_a) != 0 and _sgn(e_a) == _sgn(c_a))
        corr_b = (_sgn(e_b) != 0 and _sgn(c_b) != 0 and _sgn(e_b) == _sgn(c_b))
        same_err = (_sgn(e_a) != 0 and _sgn(e_b) != 0 and _sgn(e_a) == _sgn(e_b))
        same_cmd = (_sgn(c_a) != 0 and _sgn(c_b) != 0 and _sgn(c_a) == _sgn(c_b))
        dom_a = _dominant_pid_term(a_post, axis) if a_post else "n/a"
        dom_b = _dominant_pid_term(b_post, axis) if b_post else "n/a"
        print(
            f"    {axis:>5}: "
            f"err(A,B)=({_fmt(e_a)},{_fmt(e_b)})  "
            f"cmd(A,B)=({_fmt(c_a)},{_fmt(c_b)})  "
            f"same_err={same_err} same_cmd={same_cmd} "
            f"correct(A,B)=({corr_a},{corr_b}) "
            f"dom(A,B)=({dom_a},{dom_b})"
        )


# ---------------------------------------------------------------------------
# Main report
# ---------------------------------------------------------------------------

def compare(name_a: str, name_b: str, window_s: float = 5.0,
            do_plot: bool = False,
            reaction_pre_s: float = _REACTION_PRE_S,
            reaction_horizon_s: float = _REACTION_HORIZON_S) -> None:

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
        free_rows = [r for r in rows if not _is_kinematic_row(r)]
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
    _print_pid_diagnostics(rows_a, rows_b, f"A[{name_a}]", f"B[{name_b}]", t_kin_a, t_kin_b, window_s)
    _print_first_divergence_reactions(
        rows_a, rows_b,
        f"A[{name_a}]", f"B[{name_b}]",
        t_kin_a, t_kin_b,
        first_diverge,
        reaction_pre_s,
        reaction_horizon_s,
    )

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
        return [r.t_sim - t_kin for r in rows if not _is_kinematic_row(r)]

    ta = _ts(rows_a, t_kin_a)
    tb = _ts(rows_b, t_kin_b)

    def _get(rows: list[TelRow], attr: str) -> list[float]:
        return [getattr(r, attr) for r in rows if not _is_kinematic_row(r)]

    alt_a = [-r.pos_z for r in rows_a if not _is_kinematic_row(r)]
    alt_b = [-r.pos_z for r in rows_b if not _is_kinematic_row(r)]
    orb_a = [math.sqrt(r.pos_x**2 + r.pos_y**2) for r in rows_a if not _is_kinematic_row(r)]
    orb_b = [math.sqrt(r.pos_x**2 + r.pos_y**2) for r in rows_b if not _is_kinematic_row(r)]
    yaw_a = [math.degrees(r.orb_yaw_rad - r.rpy_yaw) for r in rows_a if not _is_kinematic_row(r)]
    yaw_b = [math.degrees(r.orb_yaw_rad - r.rpy_yaw) for r in rows_b if not _is_kinematic_row(r)]
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
    parser.add_argument("--reaction-pre", type=float, default=_REACTION_PRE_S,
                        help="Seconds before first divergence for reaction baseline (default: 1.0)")
    parser.add_argument("--reaction-horizon", type=float, default=_REACTION_HORIZON_S,
                        help="Seconds after first divergence to analyze reactions (default: 3.0)")
    parser.add_argument("--plot", action="store_true", help="Save comparison plot")
    args = parser.parse_args()
    compare(
        args.run_a,
        args.run_b,
        window_s=args.window,
        do_plot=args.plot,
        reaction_pre_s=args.reaction_pre,
        reaction_horizon_s=args.reaction_horizon,
    )
