"""
simtest_ic.py — Load steady-state initial conditions for simtests.

All simtests that need a hub starting at the aerodynamic equilibrium
should call `load()` rather than hardcoding position/velocity/spin values.

The JSON is generated (and regenerated) by test_steady_flight.py.  When the
default aero model changes, run test_steady_flight.py once to update it.

Usage
-----
    from simtest_ic import load_ic
    ic = load_ic()
    POS0   = ic.pos
    VEL0   = ic.vel
    BODY_Z0 = ic.body_z
    OMEGA_SPIN0 = ic.omega_spin
    REST_LENGTH0 = ic.rest_length
    collective_rad = ic.coll_eq_rad   # equilibrium collective [rad]
"""

from __future__ import annotations
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# steady_state_starting.json is two directories above tests/unit/
_JSON_PATH = Path(__file__).resolve().parents[2] / "steady_state_starting.json"


@dataclass
class IC:
    """Steady-state initial conditions loaded from steady_state_starting.json."""
    pos:          np.ndarray   # ENU hub position [m]
    vel:          np.ndarray   # ENU hub velocity [m/s]
    body_z:       np.ndarray   # rotor axle unit vector in world ENU
    omega_spin:   float        # equilibrium spin rate [rad/s]
    rest_length:  float        # tether rest length [m]
    coll_eq_rad:  float        # equilibrium collective [rad]
    home_z_enu:   float        # GPS home altitude [m ENU] — 0 = ground level


def load_ic() -> IC:
    """
    Load and return the steady-state initial conditions.

    Raises FileNotFoundError if the JSON has not been generated yet.
    Run test_steady_flight.py to generate it:
        python -m pytest simulation/tests/unit/test_steady_flight.py
    """
    if not _JSON_PATH.exists():
        raise FileNotFoundError(
            f"steady_state_starting.json not found at {_JSON_PATH}.\n"
            "Run test_steady_flight.py to generate it."
        )
    d = json.loads(_JSON_PATH.read_text())
    return IC(
        pos         = np.array(d["pos"],    dtype=float),
        vel         = np.array(d["vel"],    dtype=float),
        body_z      = np.array(d["body_z"], dtype=float),
        omega_spin  = float(d["omega_spin"]),
        rest_length = float(d.get("rest_length", d.get("tether_rest_length", 50.0))),
        coll_eq_rad = float(d.get("coll_eq_rad", -0.28)),
        home_z_enu  = float(d.get("home_z_enu", 0.0)),
    )
