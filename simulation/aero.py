"""
aero.py — Aerodynamic model facade for RAWES simulation.

Re-exports all model implementations and provides a single factory function
so callers only need to change one argument to swap models:

    rotor = rd.default()
    aero  = create_aero(rotor)                            # SkewedWakeBEMJit (default, ~4x faster)
    aero  = create_aero(rotor, model="skewed_wake_numpy") # SkewedWakeBEM (numpy fallback)
    aero  = create_aero(rotor, model="deschutter")        # DeSchutterAero (per-blade)
    aero  = create_aero(rotor, model="prandtl")           # PrandtlBEM (tip-loss)
    aero  = create_aero(rotor, model="glauert")           # GlauertStateBEM (inflow states)

All models expose the same interface:
    result = aero.compute_forces(collective_rad, tilt_lon, tilt_lat,
                                 R_hub, v_hub_world, omega_rotor,
                                 wind_world, t)
    omega_spin += result.Q_spin / I_spin * dt   # spin ODE
    M_orbital   = result.M_orbital              # orbital moments only

Model implementations:
    aero_deschutter.py         — DeSchutterAero:   per-blade, natural H-force + cyclic
    aero/aero_prandtl_bem.py   — PrandtlBEM:       DeSchutter + Prandtl tip/root loss
    aero/aero_skewed_wake.py   — SkewedWakeBEM:    PrandtlBEM + Coleman skewed-wake (DEFAULT)
    aero/aero_glauert_states.py — GlauertStateBEM: lumped + Glauert inflow states
"""

from dataclasses import dataclass
import sys
import os
import numpy as np

# Make simulation/aero/ importable so the three extended models can be re-exported
_AERO_SUBDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "aero")
if _AERO_SUBDIR not in sys.path:
    sys.path.insert(0, _AERO_SUBDIR)


@dataclass
class AeroResult:
    """
    Return type for compute_forces().

    The three fields needed by the mediator loop:
        F_world    — force on hub in NED world frame [N]
        M_orbital  — orbital (tilting) moments, spin-axis component excluded [N·m]
        Q_spin     — net spin torque: dω/dt = Q_spin / I_ode  [N·m]

    Diagnostic (not consumed by mediator but available for telemetry):
        M_spin     — spin-axis moment vector [N·m]

    Backward compatibility: AeroResult supports indexing as [Fx,Fy,Fz,Mx,My,Mz]
    so existing code that does  forces = compute_forces(); forces[2] > 0  still works.
    wrench property gives full 6-element ndarray.
    """
    F_world:   np.ndarray   # [3]
    M_orbital: np.ndarray   # [3]  orbital moments only (no spin component)
    Q_spin:    float        # net spin torque [N·m]
    M_spin:    np.ndarray   # [3]  spin-axis moment (diagnostic)

    def _as_array(self) -> np.ndarray:
        return np.concatenate([self.F_world, self.M_orbital + self.M_spin])

    def __getitem__(self, key):
        return self._as_array()[key]

    def __len__(self) -> int:
        return 6

    def __array__(self, dtype=None, copy=None):
        arr = self._as_array()
        return arr if dtype is None else arr.astype(dtype)

    @property
    def wrench(self) -> np.ndarray:
        """Full 6-DOF wrench [Fx,Fy,Fz,Mx,My,Mz]."""
        return self._as_array()


from aero_deschutter      import DeSchutterAero     # noqa: F401  (re-exported)
from aero_prandtl_bem     import PrandtlBEM          # noqa: F401  (re-exported)
from aero_skewed_wake     import SkewedWakeBEM        # noqa: F401  (re-exported)
from aero_skewed_wake_jit import SkewedWakeBEMJit     # noqa: F401  (re-exported)
from aero_glauert_states  import GlauertStateBEM      # noqa: F401  (re-exported)

__all__ = [
    "AeroResult",
    "DeSchutterAero",
    "PrandtlBEM", "SkewedWakeBEM", "SkewedWakeBEMJit", "GlauertStateBEM",
    "create_aero",
]

_MODELS = {
    "deschutter":       DeSchutterAero,
    "prandtl":          PrandtlBEM,
    "skewed_wake":      SkewedWakeBEMJit,   # JIT is now the default
    "skewed_wake_numpy": SkewedWakeBEM,     # pure-numpy fallback
    "glauert":          GlauertStateBEM,
}


def create_aero(defn=None, model: str = "skewed_wake"):
    """
    Factory: create an aero model from a RotorDefinition.

    Parameters
    ----------
    defn  : RotorDefinition, optional
        Rotor geometry and aerodynamic parameters.  If None, uses rd.default().
    model : str
        "skewed_wake"        (default) — SkewedWakeBEMJit: JIT-compiled, ~4x faster
        "skewed_wake_numpy"          — SkewedWakeBEM: pure-numpy reference
        "prandtl"                    — PrandtlBEM: per-blade + Prandtl tip/root loss
        "deschutter"                 — DeSchutterAero: per-blade strip theory
        "glauert"                    — GlauertStateBEM: lumped + Glauert inflow states

    Returns
    -------
    Model instance constructed via from_definition(defn).

    Example
    -------
    >>> import rotor_definition as rd
    >>> from aero import create_aero
    >>> rotor = rd.default()
    >>> aero = create_aero(rotor)                      # SkewedWakeBEM (default)
    """
    if defn is None:
        import rotor_definition as rd
        defn = rd.default()

    cls = _MODELS.get(model)
    if cls is None:
        raise ValueError(
            f"Unknown aero model {model!r} — choose one of: {list(_MODELS)}"
        )
    return cls.from_definition(defn)


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

    for label, aero in [("DeSchutterAero",  create_aero(rotor, "deschutter")),
                        ("SkewedWakeBEM",   create_aero(rotor, "skewed_wake"))]:
        print(f"\n--- {label} ---")
        f0 = aero.compute_forces(0.0, 0.0, 0.0, R, v_h, 28.0, wind, t=0.0)
        assert np.allclose(f0, 0.0, atol=1e-10), f"t=0 forces not zero: {np.array(f0)}"
        print("  t=0 ramp: OK")

        f = aero.compute_forces(0.1, 0.0, 0.0, R, v_h, 28.0, wind, t=10.0)
        print(f"  F={f[:3].round(2)}  M={f[3:].round(2)}")
        assert f[2] > 0, f"Expected Fz > 0, got {f[2]:.2f}"
        print("  Fz > 0: OK")
        print(f"  last_Q_spin={aero.last_Q_spin:.2f} N·m")

    print("\nAll smoke tests passed.")
    sys.exit(0)
