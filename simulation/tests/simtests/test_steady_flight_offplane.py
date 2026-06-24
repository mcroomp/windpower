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
from test_steady_flight import DT, _run_simulation


def _wrap_pi(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


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


def _azimuth_error_deg(pos: np.ndarray, ref_az_rad: float) -> np.ndarray:
    """Horizontal azimuth of each position relative to a fixed reference plane."""
    return np.degrees([
        _wrap_pi(math.atan2(float(p[1]), float(p[0])) - ref_az_rad)
        for p in np.asarray(pos, dtype=float)
    ])


def test_steady_flight_holds_offplane_start(simtest_log):
    """From a 30 deg off-wind start, the kite holds its launch plane (bounded).

    With the no-truth-wind plane-keeping estimator the controller does NOT know
    the true wind direction, so it cannot actively recover to the true downwind
    plane. The honest requirement is that the kite holds whatever plane it is
    launched on: the azimuth must not run away (no positive-feedback drift), the
    tether stays taut, and the state stays finite.
    """
    steps = 12000  # 30 s at 400 Hz
    initial_offset_deg = 30.0
    max_drift_deg = 8.0  # azimuth must stay within this of the launch plane

    ic = _with_azimuth_offset(load_ic(), math.radians(initial_offset_deg))
    ref_az = math.atan2(float(ic.pos[1]), float(ic.pos[0]))
    data = _run_simulation(simtest_log, steps, ic=ic)

    az_err = _azimuth_error_deg(data["pos"], ref_az)
    final_window = az_err[-max(1, round(5.0 / DT)):]
    start_abs = abs(float(az_err[0]))
    final_abs = abs(float(np.mean(final_window)))
    peak_abs = float(np.max(np.abs(az_err)))
    drift = final_abs - start_abs

    simtest_log.write(
        [
            f"plane_az_start={az_err[0]:.2f}deg  "
            f"plane_az_final_mean={float(np.mean(final_window)):.2f}deg  "
            f"peak={peak_abs:.2f}deg  drift={drift:.2f}deg  steps={steps}"
        ],
        f"plane_az_final={final_abs:.2f}deg peak={peak_abs:.2f}deg drift={drift:.2f}deg",
    )

    failures = []
    if not np.all(np.isfinite(data["pos"])):
        failures.append("NaN/inf in position history")
    if not np.all(np.isfinite(data["vel"])):
        failures.append("NaN/inf in velocity history")
    if len(np.where(data["tension"] < 0.01)[0]) > 0:
        failures.append("tether went slack during off-plane hold test")
    if peak_abs > max_drift_deg:
        failures.append(
            f"azimuth drifted {peak_abs:.2f} deg from launch plane > {max_drift_deg:.1f} deg "
            f"(plane-keeping failed; start={start_abs:.2f} deg, final={final_abs:.2f} deg)"
        )

    assert not failures, "\n  ".join(failures)