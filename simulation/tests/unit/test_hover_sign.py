"""
test_hover_sign.py — Sanity check: hover with no wind, rotor spinning.

Setup
-----
  - Disk horizontal: disk_normal = R_hub[:,2] = [0,0,-1] (up in NED)
  - No wind: wind_world = [0,0,0]
  - Hub falling: v_hub_world = [0,0,+2] m/s (downward in NED = +Z)
  - Rotor spinning at 28 rad/s
  - collective_rad = 0

Physics
-------
  v_rel = wind - v_hub = [0,0,-2]  (upward relative to hub)
  v_axial = dot(v_rel, disk_normal) = dot([0,0,-2],[0,0,-1]) = +2 m/s
  -> wind flows through disk in disk_normal direction -> rotor sees upward inflow
  -> blades generate lift along disk_normal = [0,0,-1]
  -> F_world[2] < 0  (upward = negative Z in NED)  -- arrests the fall

Both SkewedWakeBEM (original) and SkewedWakeBEM2 (reimplementation) are tested.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aero import (SkewedWakeBEM, SkewedWakeBEMJit, SkewedWakeBEM2,
                  DeSchutterAero, PrandtlBEM, GlauertStateBEM,
                  PetersHeBEM, PetersHeBEMJit)
import rotor_definition as rd

# Horizontal disk: disk_normal = [0,0,-1] (up in NED)
# R_hub[:,2] must equal [0,0,-1].  Use the same convention as _R_tilted(0):
#   [[-1,0,0],[0,1,0],[0,0,-1]]
R_HORIZONTAL = np.array([[-1., 0., 0.],
                          [ 0., 1., 0.],
                          [ 0., 0.,-1.]])

OMEGA    = 28.0                      # rad/s — typical autogyro spin
V_FALL   = np.array([0., 0., 2.0])  # hub falling 2 m/s downward (NED +Z)
NO_WIND  = np.zeros(3)
T_STEADY = 20.0                      # s — past ramp
HUB_W    = 5.0 * 9.81               # N — hub weight (5 kg)


def _make(cls):
    return cls(rd.default())


ALL_MODELS = [
    (SkewedWakeBEM,    "SkewedWakeBEM"),
    (SkewedWakeBEMJit, "SkewedWakeBEMJit"),
    (SkewedWakeBEM2,   "SkewedWakeBEM2"),
    (DeSchutterAero,   "DeSchutterAero"),
    (PrandtlBEM,       "PrandtlBEM"),
    (GlauertStateBEM,  "GlauertStateBEM"),
    (PetersHeBEM,      "PetersHeBEM"),
    (PetersHeBEMJit,   "PetersHeBEMJit"),
]


@pytest.mark.parametrize("cls,label", ALL_MODELS, ids=[m[1] for m in ALL_MODELS])
def test_hover_lift_is_upward(cls, label):
    """Rotor spinning above falling hub must produce upward force (F_world[2] < 0)."""
    aero = _make(cls)
    r = aero.compute_forces(
        collective_rad = 0.0,
        tilt_lon       = 0.0,
        tilt_lat       = 0.0,
        R_hub          = R_HORIZONTAL,
        v_hub_world    = V_FALL,
        omega_rotor    = OMEGA,
        wind_world     = NO_WIND,
        t              = T_STEADY,
    )
    Fz = float(r.F_world[2])
    print(f"\n[{label}] F_world={r.F_world}  Fz={Fz:.2f} N  T={aero.last_T:.2f} N"
          f"  v_ax={aero.last_v_axial:.2f} m/s  chi={aero.last_skew_angle_deg:.1f} deg")
    assert Fz < 0.0, (
        f"[{label}] hover: F_world[2]={Fz:.3f} N — expected negative (upward lift)"
    )


@pytest.mark.parametrize("cls,label", ALL_MODELS, ids=[m[1] for m in ALL_MODELS])
def test_hover_lift_exceeds_weight(cls, label):
    """At OMEGA=28 rad/s with 2 m/s inflow the rotor should support hub weight."""
    aero = _make(cls)
    r = aero.compute_forces(
        collective_rad = 0.0,
        tilt_lon       = 0.0,
        tilt_lat       = 0.0,
        R_hub          = R_HORIZONTAL,
        v_hub_world    = V_FALL,
        omega_rotor    = OMEGA,
        wind_world     = NO_WIND,
        t              = T_STEADY,
    )
    lift = -float(r.F_world[2])   # upward = positive
    print(f"\n[{label}] lift={lift:.1f} N  weight={HUB_W:.1f} N")
    assert lift > HUB_W, (
        f"[{label}] hover lift={lift:.1f} N < weight={HUB_W:.1f} N — rotor cannot support hub"
    )
