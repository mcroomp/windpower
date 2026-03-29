"""
aero.py — Aerodynamic model facade for RAWES simulation.

Re-exports both model implementations and provides a single factory function
so callers only need to change one argument to swap models:

    rotor = rd.default()
    aero  = create_aero(rotor)                    # RotorAero (default, validated)
    aero  = create_aero(rotor, model="deschutter")  # DeSchutterAero (experimental)

Both models expose the same interface:
    forces = aero.compute_forces(collective_rad, tilt_lon, tilt_lat,
                                 R_hub, v_hub_world, omega_rotor,
                                 wind_world, t)
    omega_spin += aero.last_Q_spin / I_spin * dt   # spin ODE
    M_orbital   = forces[3:6] - aero.last_M_spin   # orbital moments only

Model implementations:
    aero_rotor.py      — RotorAero: lumped-blade BEM, empirical H-force, K_cyc cyclic
    aero_deschutter.py — DeSchutterAero: per-blade strip theory, natural cyclic from asymmetry
"""

from aero_rotor      import RotorAero, _DEFAULTS   # noqa: F401  (re-exported)
from aero_deschutter import DeSchutterAero          # noqa: F401  (re-exported)

__all__ = ["RotorAero", "DeSchutterAero", "_DEFAULTS", "create_aero"]


def create_aero(defn=None, model: str = "rotor"):
    """
    Factory: create an aero model from a RotorDefinition.

    Parameters
    ----------
    defn  : RotorDefinition, optional
        Rotor geometry and aerodynamic parameters.  If None, uses rd.default().
    model : str
        "rotor"      (default) — RotorAero lumped-blade BEM (validated, stable)
        "deschutter"           — DeSchutterAero per-blade strip theory (experimental)

    Returns
    -------
    RotorAero or DeSchutterAero instance, constructed via from_definition(defn).

    Example
    -------
    >>> import rotor_definition as rd
    >>> from aero import create_aero
    >>> rotor = rd.default()
    >>> aero = create_aero(rotor)                   # RotorAero (default)
    >>> aero = create_aero(rotor, "deschutter")     # DeSchutterAero (experimental)
    """
    if defn is None:
        import rotor_definition as rd
        defn = rd.default()

    if model == "deschutter":
        return DeSchutterAero.from_definition(defn)
    elif model == "rotor":
        return RotorAero.from_definition(defn)
    else:
        raise ValueError(f"Unknown aero model {model!r} — choose 'deschutter' or 'rotor'")


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys
    import logging
    import numpy as np
    import rotor_definition as rd

    logging.basicConfig(level=logging.WARNING)

    rotor = rd.default()
    wind  = np.array([10.0, 0.0, 0.0])
    v_h   = np.zeros(3)
    R     = np.eye(3)

    for label, aero in [("RotorAero", create_aero(rotor, "rotor")),
                        ("DeSchutterAero", create_aero(rotor, "deschutter"))]:
        print(f"\n--- {label} ---")
        f0 = aero.compute_forces(0.0, 0.0, 0.0, R, v_h, 28.0, wind, t=0.0)
        assert np.allclose(f0, 0.0, atol=1e-10), f"t=0 forces not zero: {f0}"
        print("  t=0 ramp: OK")

        f = aero.compute_forces(0.1, 0.0, 0.0, R, v_h, 28.0, wind, t=10.0)
        print(f"  F={f[:3].round(2)}  M={f[3:].round(2)}")
        assert f[2] > 0, f"Expected Fz > 0, got {f[2]:.2f}"
        print("  Fz > 0: OK")
        print(f"  last_Q_spin={aero.last_Q_spin:.2f} N·m")

    print("\nAll smoke tests passed.")
    sys.exit(0)
