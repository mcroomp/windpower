"""Cyclic direction regression checks at the saved static IC."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from dynbem import RotorInputs, create_aero, rotor_definition


SIM = Path(__file__).resolve().parents[2]
IC_PATH = SIM / "steady_state_starting.json"
ROTOR_PATH = SIM / "rotor_definitions" / "beaupoil_2026.yaml"

RHO = 1.225
T_AERO = 45.0
STACK_COLL = -0.18
WIND = np.array([0.0, 10.0, 0.0], dtype=float)
WIND.flags.writeable = False


def _load_ic() -> SimpleNamespace:
    data = json.loads(IC_PATH.read_text())
    return SimpleNamespace(
        vel=np.array(data["vel"], dtype=float),
        R0=np.array(data["R0"], dtype=float).reshape(3, 3),
        omega_spin=float(data["omega_spin"]),
        trim_tilt_lon=float(data.get("trim_tilt_lon", 0.0)),
        trim_tilt_lat=float(data.get("trim_tilt_lat", 0.0)),
    )


def _moment_body(aero, state, ic: SimpleNamespace, tilt_lon: float, tilt_lat: float) -> np.ndarray:
    inputs = RotorInputs(
        collective_rad=STACK_COLL,
        tilt_lon=tilt_lon,
        tilt_lat=tilt_lat,
        R_hub=ic.R0,
        v_hub_world=ic.vel,
        wind_world=WIND,
        omega_rad_s=ic.omega_spin,
        t=T_AERO,
        rho_kg_m3=RHO,
    )
    result, _ = aero.compute_forces(inputs, state)
    return ic.R0.T @ np.asarray(result.M_orbital, dtype=float)


def _fixture():
    rotor = rotor_definition.load(str(ROTOR_PATH))
    aero = create_aero(rotor, model="quasi_static")
    return rotor, aero, aero.initial_rotor_state(), _load_ic()


def test_cyclic_moment_derivative_signs_at_static_ic():
    """Cyclic commands must create first-motion moments in the documented directions."""
    _rotor, aero, state, ic = _fixture()
    eps = 0.04

    dm_dlon = (
        _moment_body(aero, state, ic, tilt_lon=eps, tilt_lat=0.0)
        - _moment_body(aero, state, ic, tilt_lon=-eps, tilt_lat=0.0)
    ) / (2.0 * eps)
    dm_dlat = (
        _moment_body(aero, state, ic, tilt_lon=0.0, tilt_lat=eps)
        - _moment_body(aero, state, ic, tilt_lon=0.0, tilt_lat=-eps)
    ) / (2.0 * eps)

    # Helicopter convention used throughout RAWES:
    #   tilt_lon > 0 -> nose-down disk -> negative body pitch moment M_y.
    #   tilt_lat > 0 -> roll-right disk -> positive body roll moment M_x.
    assert dm_dlon[1] < -100.0
    assert dm_dlat[0] > 100.0

    assert abs(dm_dlon[0]) < 0.05 * abs(dm_dlon[1])
    assert abs(dm_dlon[2]) < 0.05 * abs(dm_dlon[1])
    assert abs(dm_dlat[1]) < 0.05 * abs(dm_dlat[0])
    assert abs(dm_dlat[2]) < 0.05 * abs(dm_dlat[0])


def test_saved_ic_trim_cyclic_removes_baseline_roll_moment():
    """The saved IC's lateral trim should cancel the wind-driven baseline roll moment."""
    _rotor, aero, state, ic = _fixture()

    zero_moment = _moment_body(aero, state, ic, tilt_lon=0.0, tilt_lat=0.0)
    trim_moment = _moment_body(
        aero,
        state,
        ic,
        tilt_lon=ic.trim_tilt_lon,
        tilt_lat=ic.trim_tilt_lat,
    )

    assert abs(zero_moment[0]) > 10.0
    assert abs(trim_moment[0]) < 1.0
    assert abs(trim_moment[0]) < 0.05 * abs(zero_moment[0])