"""Rotor physics helpers shared by simulation and tests."""
from __future__ import annotations
from dynbem import RotorDefinition


def require_mass_kg(rotor: RotorDefinition) -> float:
    """Return configured hub mass or fail explicitly for incomplete definitions."""
    mass = rotor.inertia.mass_kg
    if mass is None:
        raise ValueError(f"rotor.inertia.mass_kg is required for {rotor.name}")
    return float(mass)


def resolve_i_spin_kgm2(rotor: RotorDefinition) -> float:
    """Return rotor spin-axis inertia, deriving it when YAML sets null.

    ``I_spin_kgm2: null`` in the Beaupoil rotor definition means compute the
    inertia of the spinning parts from blade and hub-shell masses.  This helper
    keeps that convention explicit at the physics boundary.
    """
    inertia = rotor.inertia
    if inertia.I_spin_kgm2 is not None:
        return float(inertia.I_spin_kgm2)

    blade = rotor.blade
    blade_mass = inertia.blade_mass_kg
    shell_mass = inertia.spinning_hub_shell_mass_kg
    if blade_mass is None or shell_mass is None:
        raise ValueError(
            "rotor.inertia.I_spin_kgm2 is null, but blade_mass_kg and "
            "spinning_hub_shell_mass_kg are required to derive it"
        )

    radius = float(blade.radius_m)
    root = float(blade.root_cutout_m)
    n_blades = int(blade.n_blades)

    blade_i = float(blade_mass) * (radius * radius + radius * root + root * root) / 3.0
    shell_i = float(shell_mass) * root * root / 2.0
    return n_blades * blade_i + shell_i