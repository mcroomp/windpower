"""wind offset from trim - one-off diagnostic, not a unit test."""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SIM = ROOT / "simulation"
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(SIM / "tests" / "simtests"))

from dynbem import RotorInputs, create_aero, solve_trim_cyclic
from tests.simtests._rotor_helpers import load_default_rotor


T_AERO_OFFSET = 45.0


def _f(row: dict, key: str, default: float = 0.0) -> float:
    raw = row.get(key, "")
    return default if raw == "" else float(raw)


def _wrap_pi(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def _mean(rows: list[dict], key: str) -> float:
    return float(np.mean([_f(row, key) for row in rows]))


def _mean_rotation(rows: list[dict]) -> np.ndarray:
    # Tail windows here have sub-degree attitude variation, so elementwise mean
    # followed by SVD projection is enough for this diagnostic.
    mats = []
    for row in rows:
        mats.append(np.array([
            [_f(row, "r00"), _f(row, "r01"), _f(row, "r02")],
            [_f(row, "r10"), _f(row, "r11"), _f(row, "r12")],
            [_f(row, "r20"), _f(row, "r21"), _f(row, "r22")],
        ], dtype=float))
    mat = np.mean(mats, axis=0)
    u, _s, vt = np.linalg.svd(mat)
    rot = u @ vt
    if np.linalg.det(rot) < 0:
        u[:, -1] *= -1.0
        rot = u @ vt
    return rot


def _load_tail(path: Path, tail_rows: int) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError(f"no telemetry rows in {path}")
    return rows[-tail_rows:]


def _wind_vec(speed_mps: float, az_rad: float) -> np.ndarray:
    return np.array([
        speed_mps * math.cos(az_rad),
        speed_mps * math.sin(az_rad),
        0.0,
    ], dtype=float)


def _solve_for_candidate(
    aero,
    rotor_state,
    R: np.ndarray,
    collective_rad: float,
    omega_rad_s: float,
    wind_speed_mps: float,
    wind_az_rad: float,
    tolerance_nm: float,
):
    inputs = RotorInputs(
        collective_rad=collective_rad,
        tilt_lon=0.0,
        tilt_lat=0.0,
        R_hub=R,
        v_hub_world=np.zeros(3),
        wind_world=_wind_vec(wind_speed_mps, wind_az_rad),
        omega_rad_s=omega_rad_s,
        rho_kg_m3=1.225,
    )
    return solve_trim_cyclic(
        aero,
        rotor_state,
        inputs,
        tolerance_Nm=tolerance_nm,
        n_inflow_relax=100,
        dt_relax=0.0025,
    )


def estimate_from_log(path: Path, *, tail_rows: int, wind_speed_mps: float, step_deg: float) -> dict:
    tail = _load_tail(path, tail_rows)
    R = _mean_rotation(tail)
    observed = np.array([
        _mean(tail, "tilt_lon"),
        _mean(tail, "tilt_lat"),
    ], dtype=float)
    collective = _mean(tail, "collective_rad")
    omega = _mean(tail, "omega_rotor")
    orbit_az = _mean(tail, "orbit_azimuth_rad")
    truth_wind = np.array([_mean(tail, "wind_x"), _mean(tail, "wind_y")], dtype=float)
    truth_az = math.atan2(float(truth_wind[1]), float(truth_wind[0])) if np.linalg.norm(truth_wind) > 1e-9 else float("nan")

    rotor = load_default_rotor()
    aero = create_aero(rotor, model="quasi_static")
    base_state = aero.initial_rotor_state()

    rows = []
    best = None
    for offset_deg in np.arange(-90.0, 90.0 + 0.5 * step_deg, step_deg):
        wind_az = orbit_az + math.radians(float(offset_deg))
        trim = _solve_for_candidate(
            aero,
            base_state,
            R,
            collective,
            omega,
            wind_speed_mps,
            wind_az,
            tolerance_nm=0.5,
        )
        pred = np.array([float(trim.tilt_lon), float(trim.tilt_lat)], dtype=float)
        err = pred - observed
        cost = float(np.linalg.norm(err))
        item = {
            "candidate_offset_deg": float(offset_deg),
            "wind_az_deg": math.degrees(wind_az),
            "pred_tilt_lon_deg": math.degrees(pred[0]),
            "pred_tilt_lat_deg": math.degrees(pred[1]),
            "err_tilt_lon_deg": math.degrees(err[0]),
            "err_tilt_lat_deg": math.degrees(err[1]),
            "cost_deg": math.degrees(cost),
            "converged": bool(getattr(trim, "converged", False)),
        }
        rows.append(item)
        if best is None or item["cost_deg"] < best["cost_deg"]:
            best = item

    assert best is not None
    return {
        "path": path,
        "tail_rows": tail_rows,
        "wind_speed_mps": wind_speed_mps,
        "observed_tilt_lon_deg": math.degrees(observed[0]),
        "observed_tilt_lat_deg": math.degrees(observed[1]),
        "collective_deg": math.degrees(collective),
        "omega_rad_s": omega,
        "orbit_az_deg": math.degrees(orbit_az),
        "truth_offset_deg": math.degrees(_wrap_pi(truth_az - orbit_az)) if math.isfinite(truth_az) else float("nan"),
        "best": best,
        "rows": rows,
    }


def _write_sweep(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("telemetry", type=Path, nargs="*", help="telemetry CSV(s)")
    parser.add_argument("--tail", type=int, default=100, help="tail rows to average")
    parser.add_argument("--wind-speed", type=float, default=10.0, help="assumed wind speed [m/s]")
    parser.add_argument("--step-deg", type=float, default=2.0, help="candidate offset sweep step [deg]")
    parser.add_argument("--out-dir", type=Path, default=SIM / "logs" / "wind_offset_from_trim_probe")
    args = parser.parse_args()

    paths = args.telemetry or [
        SIM / "logs" / "test_steady_flight" / "telemetry.csv",
        SIM / "logs" / "test_steady_flight_converges_from_30deg_wind_offset" / "telemetry.csv",
    ]

    for path in paths:
        result = estimate_from_log(
            path,
            tail_rows=args.tail,
            wind_speed_mps=args.wind_speed,
            step_deg=args.step_deg,
        )
        out = args.out_dir / f"{path.parent.name}_sweep.csv"
        _write_sweep(out, result["rows"])
        best = result["best"]
        print("=" * 72)
        print(path)
        print(
            f"observed trim: lon={result['observed_tilt_lon_deg']:+.3f} deg "
            f"lat={result['observed_tilt_lat_deg']:+.3f} deg  "
            f"col={result['collective_deg']:+.3f} deg  omega={result['omega_rad_s']:.2f} rad/s"
        )
        print(
            f"truth wind offset from current orbit azimuth: {result['truth_offset_deg']:+.2f} deg "
            f"(used only for scoring)"
        )
        print(
            f"best model offset: {best['candidate_offset_deg']:+.2f} deg  "
            f"cost={best['cost_deg']:.3f} deg  "
            f"pred lon={best['pred_tilt_lon_deg']:+.3f} deg "
            f"lat={best['pred_tilt_lat_deg']:+.3f} deg"
        )
        print(f"sweep CSV: {out}")


if __name__ == "__main__":
    main()
