"""
ic.py — Single source for loading steady-state initial conditions.

Everything that starts a hub from the aerodynamic equilibrium loads the IC
through this module instead of reading ``steady_state_starting.json`` directly:

    - the mediator config defaults (config.py)
    - simtests (via simtest_ic.load_ic, which re-exports from here)
    - the SITL flight stack (stack_infra.py, flight/conftest.py)
    - analysis / diagnosis tools

Torque stack tests use their own initial condition (see mediator_torque.py) and
do NOT go through this module.

The JSON is written (and regenerated) only by
    simulation/tests/simtests/test_generate_ic.py::test_create_ic
Run that once after any aero-model or rotor-geometry change:
    .venv/Scripts/python.exe -m pytest \\
        simulation/tests/simtests/test_generate_ic.py::test_create_ic -s
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Canonical location of the generated IC file (lives next to this module).
IC_JSON_PATH = Path(__file__).resolve().parent / "steady_state_starting.json"


@dataclass
class IC:
    """Steady-state initial conditions loaded from steady_state_starting.json."""
    pos:           np.ndarray   # NED hub position [m]
    vel:           np.ndarray   # NED hub velocity [m/s]
    R0:            np.ndarray   # body-to-NED rotation (3x3); body_z = R0[:, 2]
    R0_kinematic:  np.ndarray   # R0 with body_x North-aligned for GPS/RELPOSNED lock (pure spin around body_z)
    R0_orbit:      np.ndarray   # R0 pre-tilted to cancel gravity_perp at IC; body_z = R0_orbit[:, 2]
    orbit_bz:      np.ndarray   # disk normal for orbit tracking start (= R0_orbit[:, 2])
    omega_spin:    float        # equilibrium rotor spin [rad/s]
    rest_length:   float        # tether rest length [m]
    eq_thrust:     float        # equilibrium thrust [0..1]; IC is settled at this value (~0.263)
    coll_eq_rad:   float        # equilibrium collective [rad]; same operating point as eq_thrust
    trim_tilt_lon: float        # cyclic trim that nulls IC hub moment [rad]
    trim_tilt_lat: float        # cyclic trim that nulls IC hub moment [rad]
    home_z_ned:    float        # GPS home NED Z [m] — 0 = ground level

    @property
    def home_alt_m(self) -> float:
        """Hub altitude above the anchor at launch [m] = -pos_D."""
        return -float(self.pos[2])


def _require_json() -> Path:
    if not IC_JSON_PATH.exists():
        raise FileNotFoundError(
            f"steady_state_starting.json not found at {IC_JSON_PATH}.\n"
            "Run:  python -m pytest "
            "simulation/tests/simtests/test_generate_ic.py::test_create_ic -s"
        )
    return IC_JSON_PATH


def load_ic_dict() -> dict:
    """
    Return the raw ``steady_state_starting.json`` contents as a plain dict.

    Use this when JSON-serializable primitives (lists/floats) are needed, e.g.
    the mediator config defaults or the SITL ``initial_state`` payload.

    Raises FileNotFoundError if the JSON has not been generated yet.
    """
    return json.loads(_require_json().read_text(encoding="utf-8"))


def load_ic() -> IC:
    """
    Load and return the steady-state initial conditions as an ``IC``.

    Raises FileNotFoundError if the JSON has not been generated yet.
    Run test_generate_ic.py::test_create_ic to generate it.
    """
    d = load_ic_dict()

    R0 = np.array(d["R0"], dtype=float).reshape(3, 3)

    # Fall back gracefully when loading older JSON lacking these fields.
    R0_kinematic = (
        np.array(d["R0_kinematic"], dtype=float).reshape(3, 3)
        if "R0_kinematic" in d
        else R0.copy()
    )
    R0_orbit = (
        np.array(d["R0_orbit"], dtype=float).reshape(3, 3)
        if "R0_orbit" in d
        else R0.copy()
    )
    orbit_bz = (
        np.array(d["orbit_bz"], dtype=float)
        if "orbit_bz" in d
        else R0[:, 2].copy()
    )

    return IC(
        pos           = np.array(d["pos"], dtype=float),
        vel           = np.array(d["vel"], dtype=float),
        R0            = R0,
        R0_kinematic  = R0_kinematic,
        R0_orbit      = R0_orbit,
        orbit_bz      = orbit_bz,
        omega_spin    = float(d["omega_spin"]),
        rest_length   = float(d["rest_length"]),
        eq_thrust     = float(d["eq_thrust"]),
        coll_eq_rad   = float(d.get("coll_eq_rad", 0.0)),
        trim_tilt_lon = float(d.get("trim_tilt_lon", 0.0)),
        trim_tilt_lat = float(d.get("trim_tilt_lat", 0.0)),
        home_z_ned    = float(d["home_z_ned"]),
    )
