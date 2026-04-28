"""
aero.py — Aerodynamic model facade for RAWES simulation.

Peters-He BEM with 5-state dynamic inflow is the sole production model.

    rotor = rd.default()
    aero  = create_aero(rotor)               # PetersHeBEMJit (default)
    aero  = create_aero(rotor, model="numpy") # PetersHeBEM (numpy reference)

Warm-start (serialize / restore inflow states):
    state = aero.to_dict()
    aero  = create_aero(rotor, state_dict=state)
"""

from dataclasses import dataclass
import sys
import os
import numpy as np

_AERO_SUBDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "aero")
if _AERO_SUBDIR not in sys.path:
    sys.path.insert(0, _AERO_SUBDIR)


@dataclass
class AeroResult:
    """
    Return type for compute_forces().

    Fields consumed by the mediator loop:
        F_world    — force on hub in NED world frame [N]
        M_orbital  — orbital (tilting) moments, spin-axis component excluded [N·m]
        Q_spin     — net spin torque: dω/dt = Q_spin / I_ode  [N·m]

    Diagnostic:
        M_spin     — spin-axis moment vector [N·m]

    Backward compatibility: supports indexing as [Fx,Fy,Fz,Mx,My,Mz].
    """
    F_world:   np.ndarray   # [3]
    M_orbital: np.ndarray   # [3]
    Q_spin:    float
    M_spin:    np.ndarray   # [3]

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
        return self._as_array()


from aero_peters_he     import PetersHeBEM     # noqa: F401
from aero_peters_he_jit import PetersHeBEMJit  # noqa: F401

__all__ = ["AeroResult", "PetersHeBEM", "PetersHeBEMJit", "create_aero"]


def create_aero(defn=None, model: str = "jit",
                state_dict: dict | None = None):
    """
    Create a Peters-He aero model from a RotorDefinition.

    Parameters
    ----------
    defn       : RotorDefinition, optional  (defaults to rd.default())
    model      : "jit" (default) — PetersHeBEMJit (Numba compiled)
                 "numpy"         — PetersHeBEM (pure-numpy reference)
    state_dict : dict from aero.to_dict() to warm-start inflow states.
    """
    if defn is None:
        import rotor_definition as rd
        defn = rd.default()

    if model in ("jit", "peters_he"):
        cls = PetersHeBEMJit
    elif model in ("numpy", "peters_he_numpy"):
        cls = PetersHeBEM
    else:
        raise ValueError(f"Unknown aero model {model!r} — choose 'jit' or 'numpy'")

    if state_dict is not None:
        return cls.from_definition(defn, state_dict=state_dict)
    return cls.from_definition(defn)
