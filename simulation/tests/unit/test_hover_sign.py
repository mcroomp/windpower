"""
test_hover_sign.py — hover and autorotation sign checks against the new aero.

Hover (no wind, hub falling)
  Disk horizontal, hub falling 2 m/s downward (+Z NED).  Wind-relative inflow
  is upward through the disk = autorotation/turbine regime in the new aero.
  Rotor must produce an upward force (F_world[2] < 0) that arrests the fall.

Autorotation (in-plane wind)
  Disk horizontal, 10 m/s East wind across the disk (not through it).
  Rotor must produce an upward force.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from frames import build_orb_frame
from tests.unit._aero_probe import make_probe, probe_steady


R_HORIZONTAL = build_orb_frame(np.array([0., 0., 1.]))   # FRD: body_z DOWN through disk
OMEGA    = 28.0
V_FALL   = np.array([0., 0., 2.0])
NO_WIND  = np.zeros(3)
T_STEADY = 20.0
HUB_W    = 5.0 * 9.81

WIND_10E   = np.array([0., 10., 0.])
OMEGA_IC   = 18.11

_AERO = make_probe()


def test_hover_lift_is_upward():
    r = probe_steady(_AERO, collective_rad=0.0,
                     R_hub=R_HORIZONTAL, v_hub_world=V_FALL,
                     omega_rotor=OMEGA, wind_world=NO_WIND, t=T_STEADY)
    Fz = float(r.F_world[2])
    assert Fz < 0.0, (
        f"hover: F_world[2]={Fz:.3f} N -- expected negative (upward lift)"
    )


def test_hover_lift_exceeds_weight():
    """At OMEGA=28 rad/s with 2 m/s inflow the rotor must support hub weight."""
    r = probe_steady(_AERO, collective_rad=0.0,
                     R_hub=R_HORIZONTAL, v_hub_world=V_FALL,
                     omega_rotor=OMEGA, wind_world=NO_WIND, t=T_STEADY)
    lift = -float(r.F_world[2])
    assert lift > HUB_W, (
        f"hover lift={lift:.1f} N < weight={HUB_W:.1f} N"
    )


def test_autorotation_lift_is_upward():
    """In-plane wind with spinning rotor must produce upward force (F[2]<0)."""
    r = probe_steady(_AERO, collective_rad=0.0,
                     R_hub=R_HORIZONTAL, v_hub_world=np.zeros(3),
                     omega_rotor=OMEGA_IC, wind_world=WIND_10E, t=T_STEADY)
    Fz = float(r.F_world[2])
    assert Fz < 0.0, (
        f"autorotation: F[2]={Fz:.3f} N -- expected negative (upward lift)"
    )
