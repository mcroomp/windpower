"""
simtest_ic.py — Load steady-state initial conditions for simtests.

All simtests that need a hub starting at the aerodynamic equilibrium
should call `load_ic()` rather than hardcoding position/velocity/spin values.

The JSON is generated (and regenerated) by test_generate_ic.py.  When the
aero model or rotor geometry changes, run test_generate_ic.py once to update it:

    simulation/.venv/Scripts/python.exe -m pytest
        simulation/tests/unit/test_generate_ic.py::test_create_ic -s

Usage
-----
    from simtest_ic import load_ic
    ic = load_ic()
    POS0         = ic.pos
    VEL0         = ic.vel
    R0           = ic.R0          # body-to-NED rotation
    BODY_Z0      = ic.R0[:, 2]   # disk normal at design orientation
    ORBIT_BZ0    = ic.orbit_bz   # pre-tilted disk normal for orbit tracking start
    OMEGA_SPIN0  = ic.omega_spin
    REST_LENGTH0 = ic.rest_length
    COLL_EQ      = ic.coll_eq_rad  # equilibrium collective (= stack_coll_eq = -0.18 rad)
"""

from __future__ import annotations
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_JSON_PATH = Path(__file__).resolve().parents[2] / "steady_state_starting.json"


@dataclass
class IC:
    """Steady-state initial conditions loaded from steady_state_starting.json."""
    pos:           np.ndarray   # NED hub position [m]
    vel:           np.ndarray   # NED hub velocity [m/s]
    R0:            np.ndarray   # body-to-NED rotation (3x3); body_z = R0[:, 2]
    R0_orbit:      np.ndarray   # R0 pre-tilted to cancel gravity_perp at IC; body_z = R0_orbit[:, 2]
    orbit_bz:      np.ndarray   # disk normal for orbit tracking start (= R0_orbit[:, 2])
    omega_spin:    float        # equilibrium rotor spin [rad/s]
    rest_length:   float        # tether rest length [m]
    coll_eq_rad:   float        # equilibrium collective [rad] (= stack_coll_eq = -0.18 rad)
    stack_coll_eq: float        # same as coll_eq_rad; kept for clarity in stack-test configs
    home_z_ned:    float        # GPS home NED Z [m] — 0 = ground level


def load_ic() -> IC:
    """
    Load and return the steady-state initial conditions.

    Raises FileNotFoundError if the JSON has not been generated yet.
    Run test_generate_ic.py::test_create_ic to generate it.
    """
    if not _JSON_PATH.exists():
        raise FileNotFoundError(
            f"steady_state_starting.json not found at {_JSON_PATH}.\n"
            "Run:  python -m pytest simulation/tests/unit/test_generate_ic.py::test_create_ic -s"
        )
    d = json.loads(_JSON_PATH.read_text())

    R0 = np.array(d["R0"], dtype=float).reshape(3, 3)

    # R0_orbit and orbit_bz were added by test_generate_ic.py; fall back to R0
    # when loading an older JSON written by test_steady_flight.py.
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

    coll_eq_rad = float(d["coll_eq_rad"])

    return IC(
        pos           = np.array(d["pos"], dtype=float),
        vel           = np.array(d["vel"], dtype=float),
        R0            = R0,
        R0_orbit      = R0_orbit,
        orbit_bz      = orbit_bz,
        omega_spin    = float(d["omega_spin"]),
        rest_length   = float(d["rest_length"]),
        coll_eq_rad   = coll_eq_rad,
        stack_coll_eq = float(d["stack_coll_eq"]),
        home_z_ned    = float(d["home_z_ned"]),
    )
