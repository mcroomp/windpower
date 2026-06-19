#!/usr/bin/env python3
"""Analyse IC attitude target tracking from simtest telemetry."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

_SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))

from telemetry_csv import read_csv  # noqa: E402


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    an = float(np.linalg.norm(a))
    bn = float(np.linalg.norm(b))
    if an < 1e-9 or bn < 1e-9:
        return 0.0
    return float(math.degrees(math.acos(float(np.clip(np.dot(a, b) / (an * bn), -1.0, 1.0)))))


def _body_z(row) -> np.ndarray:
    return row.R[:, 2]


def _body_z_target(row) -> np.ndarray:
    return row.body_z_eq


def _fmt_range(values: np.ndarray, unit: str = "") -> str:
    if values.size == 0:
        return "n/a"
    suffix = unit if unit else ""
    return f"{float(np.min(values)):.3f}..{float(np.max(values)):.3f}{suffix}"


def _bucket_rows(rows: list, bucket_s: float) -> list[list]:
    if not rows:
        return []
    buckets: list[list] = []
    start = float(rows[0].t_sim)
    current: list = []
    current_i = 0
    for row in rows:
        idx = int((float(row.t_sim) - start) // bucket_s)
        if idx != current_i:
            if current:
                buckets.append(current)
            current = []
            current_i = idx
        current.append(row)
    if current:
        buckets.append(current)
    return buckets


def analyse(path: Path, bucket_s: float) -> int:
    rows = [r for r in read_csv(path) if r.damp_alpha == 0.0]
    if not rows:
        print(f"No free-flight telemetry rows in {path}")
        return 1

    first = rows[0]
    first_target = _body_z_target(first)
    first_body_z = _body_z(first)

    target_drift = np.array([_angle_deg(_body_z_target(r), first_target) for r in rows])
    attitude_err = np.array([r.body_z_err_deg for r in rows], dtype=float)
    tension_ff = np.array([r.tension_feedforward_n for r in rows], dtype=float)
    tension_ic = np.array([r.tension_ic_n for r in rows], dtype=float)
    tension = np.array([r.tether_tension for r in rows], dtype=float)
    roll_sp = np.array([r.roll_sp_rads for r in rows], dtype=float)
    pitch_sp = np.array([r.pitch_sp_rads for r in rows], dtype=float)
    roll_err = np.array([r.roll_rate_err_rads for r in rows], dtype=float)
    pitch_err = np.array([r.pitch_rate_err_rads for r in rows], dtype=float)

    print(f"IC attitude analysis: {path}")
    print(f"rows={len(rows)}  t={rows[0].t_sim:.3f}..{rows[-1].t_sim:.3f}s  bucket={bucket_s:.1f}s")
    print(
        "initial: "
        f"T_ic={first.tension_ic_n:.3f}N  T_ff={first.tension_feedforward_n:.3f}N  "
        f"T_tether={first.tether_tension:.3f}N"
    )
    print(f"initial body_z={first_body_z.tolist()}")
    print(f"initial target={first_target.tolist()}")
    print(
        "ranges: "
        f"T_ff={_fmt_range(tension_ff, 'N')}  "
        f"T_ic={_fmt_range(tension_ic, 'N')}  "
        f"T_tether={_fmt_range(tension, 'N')}  "
        f"target_drift={_fmt_range(target_drift, 'deg')}  "
        f"att_err={_fmt_range(attitude_err, 'deg')}"
    )
    print(
        "rates: "
        f"roll_sp={_fmt_range(roll_sp, 'rad/s')}  "
        f"pitch_sp={_fmt_range(pitch_sp, 'rad/s')}  "
        f"roll_err={_fmt_range(roll_err, 'rad/s')}  "
        f"pitch_err={_fmt_range(pitch_err, 'rad/s')}"
    )

    slack = [r for r in rows if r.tether_slack or r.tether_tension < 1.0]
    if slack:
        first_slack = slack[0]
        print(
            "first_slack: "
            f"t={first_slack.t_sim:.3f}s  T={first_slack.tether_tension:.3f}N  "
            f"att_err={first_slack.body_z_err_deg:.3f}deg  "
            f"target_drift={_angle_deg(_body_z_target(first_slack), first_target):.3f}deg"
        )

    print()
    print(
        "bucket        T_ff_N      T_N    target_deg  att_deg  "
        "roll_sp pitch_sp roll_err pitch_err  downwind crosswind"
    )
    for bucket in _bucket_rows(rows, bucket_s):
        arr_t = np.array([r.t_sim for r in bucket], dtype=float)
        arr_target = np.array([_angle_deg(_body_z_target(r), first_target) for r in bucket], dtype=float)
        arr_att = np.array([r.body_z_err_deg for r in bucket], dtype=float)
        print(
            f"{arr_t[0]:6.2f}-{arr_t[-1]:6.2f}  "
            f"{np.mean([r.tension_feedforward_n for r in bucket]):8.2f}  "
            f"{np.mean([r.tether_tension for r in bucket]):7.2f}  "
            f"{float(np.max(arr_target)):10.3f}  "
            f"{float(np.max(arr_att)):7.3f}  "
            f"{np.mean([r.roll_sp_rads for r in bucket]):7.3f}  "
            f"{np.mean([r.pitch_sp_rads for r in bucket]):8.3f}  "
            f"{np.mean([r.roll_rate_err_rads for r in bucket]):8.3f}  "
            f"{np.mean([r.pitch_rate_err_rads for r in bucket]):9.3f}  "
            f"{np.mean([r.pos_downwind_m for r in bucket]):8.2f}  "
            f"{np.mean([r.pos_crosswind_m for r in bucket]):9.2f}"
        )

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv",
        nargs="?",
        default=str(_SIM_DIR / "logs" / "test_ic_steady_flight" / "telemetry.csv"),
        help="Telemetry CSV path",
    )
    parser.add_argument("--bucket", type=float, default=1.0, help="Bucket width in seconds")
    args = parser.parse_args()
    return analyse(Path(args.csv), args.bucket)


if __name__ == "__main__":
    raise SystemExit(main())