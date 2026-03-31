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

import rotor_definition as rd
from aero import DeSchutterAero
from aero_prandtl_bem    import PrandtlBEM
from aero_skewed_wake    import SkewedWakeBEM
from aero_glauert_states import GlauertStateBEM

# ── RAWES design-point constants ──────────────────────────────────────────────
# body_z = [0.851, 0.305, 0.427]: tether-equilibrium disk orientation at 10 m/s
# v_axial    = dot([10,0,0], body_z) = 8.51 m/s
# v_inplane  = |[10,0,0] - 8.51·body_z| = 5.25 m/s
DESIGN_BODY_Z    = np.array([0.851, 0.305, 0.427])
DESIGN_WIND      = np.array([10.0, 0.0, 0.0])
DESIGN_V_INPLANE = 5.25   # m/s
DESIGN_V_AXIAL   = 8.51   # m/s
DESIGN_OMEGA_EQ  = 20.148  # rad/s (equilibrium spin, from beaupoil_2026.yaml)


def make_R_hub(body_z: np.ndarray) -> np.ndarray:
    """Construct R_hub (3×3) from disk normal (body_z column)."""
    bz = body_z / np.linalg.norm(body_z)
    east = np.array([1.0, 0.0, 0.0])
    bx   = east - np.dot(east, bz) * bz
    if np.linalg.norm(bx) < 1e-6:
        north = np.array([0.0, 1.0, 0.0])
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
    wind_world     = np.array([8.66, 0.0, 5.0]),
    t              = 10.0,
)

EDGEWISE_KWARGS = dict(
    collective_rad = 0.0,
    tilt_lon       = 0.0,
    tilt_lat       = 0.0,
    R_hub          = np.array([
        [0.0,  0.0,  1.0],
        [0.0,  1.0,  0.0],
        [-1.0, 0.0,  0.0],
    ], dtype=float),
    v_hub_world    = np.zeros(3),
    omega_rotor    = 20.0,
    wind_world     = np.array([10.0, 0.0, 0.0]),
    t              = 10.0,
)

ALL_MODELS = [DeSchutterAero, PrandtlBEM, SkewedWakeBEM, GlauertStateBEM]
ALL_MODEL_NAMES = [c.__name__ for c in ALL_MODELS]
