"""Wind-aligned IC steady-state probe -- one-off diagnostic, not a unit test."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SIM = ROOT / "simulation"
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(SIM / "tests" / "simtests"))

from frames import build_orb_frame
from tests.simtests import test_generate_ic as gen


OUT_DIR = SIM / "logs" / "wind_aligned_ic_probe"


def _wind_axes() -> tuple[np.ndarray, np.ndarray]:
    wind_h = np.asarray(gen.WIND[:2], dtype=float)
    norm = float(np.linalg.norm(wind_h))
    if norm < 1e-6:
        raise ValueError("wind_aligned_ic_probe requires horizontal wind")
    down = wind_h / norm
    cross = np.array([-down[1], down[0]])
    return down, cross


def _project_pos(pos: np.ndarray) -> np.ndarray:
    down, _ = _wind_axes()
    tether_length = float(np.linalg.norm(pos))
    alt = float(-pos[2])
    horizontal = float(np.sqrt(max(0.0, tether_length * tether_length - alt * alt)))
    return np.array([horizontal * down[0], horizontal * down[1], -alt], dtype=float)


def _remove_crosswind_velocity(vel: np.ndarray) -> np.ndarray:
    down, _ = _wind_axes()
    vel_h = np.asarray(vel[:2], dtype=float)
    down_v = float(np.dot(vel_h, down))
    return np.array([down_v * down[0], down_v * down[1], vel[2]], dtype=float)


def _summarize_wind_plane(csv_path: Path) -> dict:
    import csv

    with csv_path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    first = rows[0]
    last = rows[-1]

    def f(row: dict, key: str) -> float:
        return float(row[key])

    cross_abs = max(abs(f(row, "pos_crosswind_m")) for row in rows)
    cross_v_abs = max(abs(f(row, "vel_crosswind_mps")) for row in rows)
    tan_v_abs = max(abs(f(row, "vel_tangential_mps")) for row in rows)
    bz_err = max(f(row, "body_z_err_deg") for row in rows)
    return dict(
        cross0=f(first, "pos_crosswind_m"),
        cross1=f(last, "pos_crosswind_m"),
        cross_abs_max=cross_abs,
        cross_vel_abs_max=cross_v_abs,
        tan_vel_abs_max=tan_v_abs,
        downwind0=f(first, "pos_downwind_m"),
        downwind1=f(last, "pos_downwind_m"),
        alt0=-f(first, "pos_z"),
        alt1=-f(last, "pos_z"),
        bz_err_max=bz_err,
    )


def _run_case(label: str, pos: np.ndarray, vel: np.ndarray, base: dict) -> dict:
    tether_dir = -pos / float(np.linalg.norm(pos))
    R0 = build_orb_frame(tether_dir)
    csv_path = OUT_DIR / f"{label}.csv"
    metrics = gen._run_steady(
        pos,
        vel,
        R0,
        float(base["omega_spin"]),
        float(base["rest_length"]),
        float(base.get("tension_eq_n", 300.0)),
        float(base["stack_coll_eq"]),
        label=label,
        csv_path=csv_path,
    )
    return {**metrics, **_summarize_wind_plane(csv_path)}


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    base = json.loads((SIM / "steady_state_starting.json").read_text())
    pos0 = np.asarray(base["pos"], dtype=float)
    vel0 = np.asarray(base["vel"], dtype=float)
    pos_wind = _project_pos(pos0)

    cases = {
        "wind_aligned_keep_velocity": (pos_wind, vel0),
        "wind_aligned_no_cross_velocity": (pos_wind, _remove_crosswind_velocity(vel0)),
        "wind_aligned_zero_velocity": (pos_wind, np.zeros(3)),
    }

    for label, (pos, vel) in cases.items():
        metrics = _run_case(label, pos, vel, base)
        print(f"\n[{label}]")
        print(f"  pos0={pos.tolist()}")
        print(f"  vel0={vel.tolist()}")
        print(f"  tlen_dev={metrics['max_tlen_dev']:.3f} m  alt_range={metrics['alt_range']:.2f} m")
        print(f"  speed mean={metrics['mean_speed']:.3f} m/s  max={metrics['max_speed']:.3f} m/s")
        print(f"  tension mean={metrics['mean_tension']:.1f} N  min={metrics['min_tension']:.1f} N")
        print(f"  cross={metrics['cross0']:.2f}->{metrics['cross1']:.2f} m  max_abs={metrics['cross_abs_max']:.2f} m")
        print(f"  cross_v_max={metrics['cross_vel_abs_max']:.2f} m/s  tan_v_max={metrics['tan_vel_abs_max']:.2f} m/s")
        print(f"  bz_err_max={metrics['bz_err_max']:.2f} deg")


if __name__ == "__main__":
    main()