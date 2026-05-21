"""Shared simtest helpers for loading the project rotor and reading
physical constants that the leaner new RotorDefinition no longer exposes
as derived properties.

The new ``aero`` RotorDefinition is nested (blade / airfoil / inertia /
control / autorotation) and does not provide ``default()``, ``dynamics_kwargs()``,
or derived control constants like ``body_z_slew_rate_rad_s``.  Simtests pick
those up from here so they have a single source of truth.
"""
from __future__ import annotations
from pathlib import Path

from dynbem import rotor_definition as _rd


_ROTOR_DEFS = Path(__file__).resolve().parents[2] / "rotor_definitions"


def load_default_rotor():
    """Load the project default rotor (beaupoil_2026)."""
    return _rd.load(str(_ROTOR_DEFS / "beaupoil_2026.yaml"))


def load_rotor(name: str):
    """Load a project rotor by stem name (e.g. ``beaupoil_2026``)."""
    return _rd.load(str(_ROTOR_DEFS / f"{name}.yaml"))


def dynamics_kwargs(rotor) -> dict:
    """Mass / inertia kwargs in the shape RigidBodyDynamics expects."""
    inertia = rotor.inertia
    if inertia.mass_kg is None:
        raise ValueError(f"rotor.inertia.mass_kg is None for {rotor.name}")
    I_spin = inertia.I_spin_kgm2 if inertia.I_spin_kgm2 is not None else 0.0
    return dict(
        mass=float(inertia.mass_kg),
        I_body=list(inertia.I_body_kgm2),
        I_spin=float(I_spin),
    )


# ── Project control constants (used to live on RotorDefinition) ──────────────
#
# body_z_slew_rate_rad_s
#   Rate-limit for the body_z planner used by all simtests.
#   Per CLAUDE.md: "body_z_slew_rate = 0.40 rad/s".  Originally derived as
#   ~2% of the gyroscopic max precession rate; pinned here because the
#   downstream tuning depends on this exact value.
BODY_Z_SLEW_RATE_RAD_S: float = 0.40
