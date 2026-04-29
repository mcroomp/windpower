"""
aero — Aerodynamic model interface and factory for RAWES simulation.

AeroBase is the abstract interface.  Instantiate any model directly:

    rotor = rd.default()
    aero  = PetersHeBEMJit(rotor)    # production (JIT)
    aero  = PetersHeBEM(rotor)       # numpy reference

Warm-start (serialize / restore inflow states):
    state = aero.to_dict()
    aero  = PetersHeBEMJit(rotor, state_dict=state)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


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


class AeroBase(ABC):
    """
    Abstract interface for all RAWES aero models.

    Instantiate directly by class name:
        aero = PetersHeBEMJit(rotor)
        aero = PetersHeBEMJit(rotor, state_dict=prev.to_dict())

    Every subclass gets a free ``from_definition`` classmethod
    (identical to calling the constructor directly).
    """

    @abstractmethod
    def compute_forces(
        self,
        collective_rad: float,
        tilt_lon:       float,
        tilt_lat:       float,
        R_hub:          np.ndarray,
        v_hub_world:    np.ndarray,
        omega_rotor:    float,
        wind_world:     np.ndarray,
        t:              float,
        spin_angle:     float = 0.0,
    ) -> "AeroResult": ...

    @abstractmethod
    def to_dict(self) -> dict: ...

    @abstractmethod
    def cl_cd(self, alpha_rad: float) -> "tuple[float, float]":
        """Return (CL, CD) for the given angle of attack [rad]."""
        ...

    def is_valid(self) -> bool:
        return True

    @classmethod
    def from_definition(cls, defn: "RotorDefinition",
                        state_dict: "dict | None" = None) -> "AeroBase":
        return cls(defn, state_dict=state_dict)


from .aero_peters_he     import PetersHeBEM     # noqa: F401, E402
from .aero_peters_he_jit import PetersHeBEMJit  # noqa: F401, E402
from . import rotor_definition                   # noqa: F401, E402
from .rotor_definition import (                  # noqa: F401, E402
    RotorDefinition, KamanFlap, ValidationIssue,
)

__all__ = [
    "AeroResult", "AeroBase", "PetersHeBEM", "PetersHeBEMJit", "create_aero",
    "rotor_definition", "RotorDefinition", "KamanFlap", "ValidationIssue",
]


def create_aero(defn=None, model: str = "jit",
                state_dict: dict | None = None) -> AeroBase:
    """
    Create a Peters-He aero model from a RotorDefinition.

    Convenience factory.  Prefer instantiating by class name directly:
        aero = PetersHeBEMJit(rotor)

    Parameters
    ----------
    defn       : RotorDefinition, optional  (defaults to rd.default())
    model      : "jit" (default) — PetersHeBEMJit (Numba compiled)
                 "numpy"         — PetersHeBEM (pure-numpy reference)
    state_dict : dict from aero.to_dict() to warm-start inflow states.
    """
    if defn is None:
        defn = rotor_definition.default()

    if model in ("jit", "peters_he"):
        cls = PetersHeBEMJit
    elif model in ("numpy", "peters_he_numpy"):
        cls = PetersHeBEM
    else:
        raise ValueError(f"Unknown aero model {model!r} — choose 'jit' or 'numpy'")

    return cls(defn, state_dict=state_dict)
