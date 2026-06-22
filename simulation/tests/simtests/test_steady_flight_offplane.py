"""Off-plane steady-flight convergence checks for fixed wind-plane azimuth."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

from simtest_ic import load_ic
from test_steady_flight import DT, wind_azimuth_rad, _run_simulation


def _wrap_pi(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def _wind_azimuth_rad() -> float:
    return wind_azimuth_rad()


def _rotate_about_ned_down(angle_rad: float) -> np.ndarray:
    c = math.cos(float(angle_rad))
    s = math.sin(float(angle_rad))
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _with_azimuth_offset(ic, offset_rad: float):
    rot = _rotate_about_ned_down(offset_rad)
    return SimpleNamespace(
        pos=rot @ ic.pos,
        vel=rot @ ic.vel,
        R0=rot @ ic.R0,
        rest_length=ic.rest_length,
        coll_eq_rad=ic.coll_eq_rad,
        omega_spin=ic.omega_spin,
        trim_tilt_lon=ic.trim_tilt_lon,
        trim_tilt_lat=ic.trim_tilt_lat,
    )


def _wind_azimuth_error_deg(pos: np.ndarray) -> np.ndarray:
    wind_az = _wind_azimuth_rad()
    return np.degrees([
        _wrap_pi(math.atan2(float(p[1]), float(p[0])) - wind_az)
        for p in np.asarray(pos, dtype=float)
    ])


def test_steady_flight_converges_from_30deg_wind_offset(simtest_log):
    """From a 30 deg off-wind start, fixed wind-plane azimuth should recover."""
    steps = 12000  # 30 s at 400 Hz
    initial_offset_deg = 30.0
    min_improvement_deg = 10.0
    final_max_deg = 15.0

    ic = _with_azimuth_offset(load_ic(), math.radians(initial_offset_deg))
    data = _run_simulation(simtest_log, steps, ic=ic)

    az_err = _wind_azimuth_error_deg(data["pos"])
    final_window = az_err[-max(1, round(5.0 / DT)):]
    start_abs = abs(float(az_err[0]))
    final_abs = abs(float(np.mean(final_window)))
    improvement = start_abs - final_abs

    simtest_log.write(
        [
            f"wind_az_start={az_err[0]:.2f}deg  "
            f"wind_az_final_mean={float(np.mean(final_window)):.2f}deg  "
            f"improvement={improvement:.2f}deg  steps={steps}"
        ],
        f"wind_az_final={final_abs:.2f}deg improvement={improvement:.2f}deg",
    )

    failures = []
    if not np.all(np.isfinite(data["pos"])):
        failures.append("NaN/inf in position history")
    if not np.all(np.isfinite(data["vel"])):
        failures.append("NaN/inf in velocity history")
    if len(np.where(data["tension"] < 0.01)[0]) > 0:
        failures.append("tether went slack during off-wind convergence test")
    if improvement < min_improvement_deg:
        failures.append(
            f"wind azimuth error improved {improvement:.2f} deg < {min_improvement_deg:.1f} deg "
            f"(start={start_abs:.2f} deg, final={final_abs:.2f} deg)"
        )
    if final_abs > final_max_deg:
        failures.append(
            f"final wind azimuth error {final_abs:.2f} deg > {final_max_deg:.1f} deg"
        )

    assert not failures, "\n  ".join(failures)