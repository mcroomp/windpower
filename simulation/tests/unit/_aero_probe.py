"""Test helpers for one-shot aero probes against the new state-based API.

The new aero (e:/repos/aero) exposes compute_forces(RotorInputs, RotorState)
and integrates omega + inflow internally.  These helpers wrap the call so
unit tests that only care about steady-state forces stay readable.

Use ``probe_steady`` for quasi-steady probes.  Quasi-static aero returns
immediately; dynamic-inflow models can still be passed explicitly and are
settled with a short forward-Euler loop at fixed omega.

All R_hub passed in must follow the project FRD convention: R[:,2] points
DOWN through the disk (e.g. R[:,2] = [0,0,+1] for a level hover, or
hub→anchor direction in tethered flight).  No coordinate-frame flip is
applied — windpower and the aero share one convention.
"""
from __future__ import annotations
from pathlib import Path

import numpy as np

from dynbem import RotorInputs, create_aero, rotor_definition as _rd


_ROTOR_DEFS = Path(__file__).resolve().parents[2] / "rotor_definitions"


def load_rotor(name: str = "beaupoil_2026"):
    """Load a project rotor by stem name from simulation/rotor_definitions/."""
    return _rd.load(str(_ROTOR_DEFS / f"{name}.yaml"))


def make_probe(rotor=None):
    """Build the default quasi-static aero model for probing."""
    if rotor is None:
        rotor = load_rotor()
    return create_aero(rotor, "quasi_static")


def probe_steady(
    aero,
    *,
    collective_rad: float,
    tilt_lon: float = 0.0,
    tilt_lat: float = 0.0,
    R_hub: np.ndarray,
    v_hub_world: np.ndarray,
    omega_rotor: float,
    wind_world: np.ndarray,
    t: float = 0.0,
    rho_kg_m3: float = 1.225,
):
    """Return a quasi-steady AeroResult.

    The aero ``step`` API settles its inflow/wake state on the first call, so a
    single step at a fixed operating point yields the steady-state result.
    """
    state = aero.initial_rotor_state()
    inputs = RotorInputs(
        collective_rad=collective_rad,
        tilt_lon=tilt_lon,
        tilt_lat=tilt_lat,
        R_hub=R_hub,
        v_hub_world=np.asarray(v_hub_world, dtype=float),
        wind_world=np.asarray(wind_world, dtype=float),
        omega_rad_s=float(omega_rotor),
        rho_kg_m3=rho_kg_m3,
    )
    result, _ = aero.step(inputs, state, 0.02)
    return result
