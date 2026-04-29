"""
_helpers.py — Shared constants and utilities for aero/ test suite.

Importable by any test file via:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from _helpers import ALL_MODELS, DESIGN_KWARGS, ...
"""

import sys
import math
from pathlib import Path

import numpy as np

_HERE      = Path(__file__).resolve().parent          # simulation/aero/tests/
_AERO_DIR  = _HERE.parent                             # simulation/aero/
_SIM_DIR   = _AERO_DIR.parent                         # simulation/

for _p in [str(_SIM_DIR), str(_AERO_DIR), str(_HERE)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aero import rotor_definition as rd
from aero.aero_peters_he     import PetersHeBEM
from aero.aero_peters_he_jit import PetersHeBEMJit
from aero.aero_simple_bem    import SimpleBEM
from aero.aero_openfast_bem  import OpenFASTBEM

# ── RAWES design-point constants ──────────────────────────────────────────────
# body_z = [0.305, 0.851, -0.427]: tether-equilibrium disk orientation at 10 m/s (NED)
# v_axial    = dot([0,10,0], body_z) = 8.51 m/s
# v_inplane  = |[0,10,0] - 8.51·body_z| = 5.25 m/s
DESIGN_BODY_Z    = np.array([0.305, 0.851, -0.427])   # NED: T @ [0.851, 0.305, 0.427]
DESIGN_WIND      = np.array([0.0, 10.0, 0.0])          # NED: East wind = Y axis
DESIGN_V_INPLANE = 5.25   # m/s
DESIGN_V_AXIAL   = 8.51   # m/s
DESIGN_OMEGA_EQ  = 20.148  # rad/s (equilibrium spin, from beaupoil_2026.yaml)


def make_R_hub(body_z: np.ndarray) -> np.ndarray:
    """Construct R_hub (3×3) from disk normal (body_z column)."""
    bz = body_z / np.linalg.norm(body_z)
    east = np.array([0.0, 1.0, 0.0])   # NED: East = Y axis
    bx   = east - np.dot(east, bz) * bz
    if np.linalg.norm(bx) < 1e-6:
        north = np.array([1.0, 0.0, 0.0])  # NED: North = X axis
        bx    = north - np.dot(north, bz) * bz
    bx /= np.linalg.norm(bx)
    by = np.cross(bz, bx)
    return np.column_stack([bx, by, bz])


DESIGN_R_HUB = make_R_hub(DESIGN_BODY_Z)

DESIGN_KWARGS = dict(
    collective_rad = 0.1,
    tilt_lon       = 0.0,
    tilt_lat       = 0.0,
    R_hub          = DESIGN_R_HUB,
    v_hub_world    = np.zeros(3),
    omega_rotor    = DESIGN_OMEGA_EQ,
    wind_world     = DESIGN_WIND,
    t              = 10.0,
)

HOVER_KWARGS = dict(
    collective_rad = 0.1,
    tilt_lon       = 0.0,
    tilt_lat       = 0.0,
    R_hub          = np.eye(3),
    v_hub_world    = np.zeros(3),
    omega_rotor    = 20.0,
    wind_world     = np.array([8.66, 0.0, -5.0]),   # NED: North=X, upward=-Z
    t              = 10.0,
)

# Edgewise: disk normal pointing East (NED Y). R_ned = T @ R_enu_edgewise.
EDGEWISE_KWARGS = dict(
    collective_rad = 0.0,
    tilt_lon       = 0.0,
    tilt_lat       = 0.0,
    R_hub          = np.array([
        [0.0,  1.0,  0.0],
        [0.0,  0.0,  1.0],
        [1.0,  0.0,  0.0],
    ], dtype=float),
    v_hub_world    = np.zeros(3),
    omega_rotor    = 20.0,
    wind_world     = np.array([0.0, 10.0, 0.0]),    # NED: East wind = Y axis
    t              = 10.0,
)

ALL_MODELS = [PetersHeBEM, PetersHeBEMJit, OpenFASTBEM]
ALL_MODEL_NAMES = [c.__name__ for c in ALL_MODELS]
