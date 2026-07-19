"""Shared rotor-loading helper for the envelope package.

The current (nested blade/airfoil/inertia/control/autorotation)
``dynbem.rotor_definition.RotorDefinition`` has no project ``default()``
factory or ``dynamics_kwargs()`` method -- envelope scripts load the project
rotor explicitly from ``simulation/rotor_definitions/`` and read
``rotor.inertia.mass_kg`` / ``rotor.autorotation.I_ode_kgm2`` directly.
"""
from __future__ import annotations
from pathlib import Path

import simulation
from dynbem import rotor_definition as _rd

_ROTOR_DEFS = Path(simulation.__file__).resolve().parent / "rotor_definitions"


def load_default_rotor():
    """Load the project default rotor (beaupoil_2026)."""
    return _rd.load(str(_ROTOR_DEFS / "beaupoil_2026.yaml"))
