"""
test_collective_sign.py -- Verify collective sign convention.

Uses the same setup as test_hover_sign.py:
  - Disk horizontal: bz = [0,0,-1] (up in NED)
  - Hub falling at 2 m/s downward
  - No wind
  - omega = 28 rad/s

At this operating point, increasing collective should increase thrust
(upward force = more negative F_world[2]).

Sign chain (from test_hover_sign.py / CLAUDE.md):
  upward thrust = dot(F_world, disk_normal) = dot(F_world, [0,0,-1]) = -F_world[2]
  So thrust > 0 means F_world[2] < 0.
  More collective -> more thrust -> F_world[2] more negative.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rotor_definition as rd
from aero import create_aero

R_HORIZONTAL = np.array([[-1.,  0.,  0.],
                          [ 0.,  1.,  0.],
                          [ 0.,  0., -1.]])   # bz = [0,0,-1] (up in NED)

V_FALL  = np.array([0., 0., 2.0])   # hub falling 2 m/s downward
NO_WIND = np.zeros(3)
OMEGA   = 28.0
T_STEADY = 20.0


def _thrust(col):
    rotor = rd.default()
    aero  = create_aero(rotor, model="peters_he")
    r = aero.compute_forces(
        collective_rad = col,
        tilt_lon       = 0.0,
        tilt_lat       = 0.0,
        R_hub          = R_HORIZONTAL,
        v_hub_world    = V_FALL,
        omega_rotor    = OMEGA,
        wind_world     = NO_WIND,
        t              = T_STEADY,
    )
    bz = R_HORIZONTAL[:, 2]          # [0, 0, -1]
    return float(np.dot(r.F_world, bz))   # upward thrust > 0


def test_thrust_is_positive_at_zero_collective():
    """Baseline: col=0, falling hub, spinning rotor must generate upward thrust."""
    t = _thrust(0.0)
    assert t > 0.0, f"expected positive thrust at col=0, got {t:.2f} N"


def test_higher_collective_increases_thrust():
    """Positive collective must produce more upward thrust than negative collective."""
    t_neg = _thrust(-0.10)
    t_pos = _thrust(+0.10)
    assert t_pos > t_neg, (
        f"col=-0.10 -> thrust={t_neg:.1f} N,  col=+0.10 -> thrust={t_pos:.1f} N  "
        f"(expected higher col = more thrust)"
    )


def test_thrust_monotone_with_collective():
    """Thrust must increase monotonically across the collective range."""
    cols   = [-0.20, -0.10, 0.0, 0.10, 0.20]
    thrusts = [_thrust(c) for c in cols]
    for i in range(len(thrusts) - 1):
        assert thrusts[i] < thrusts[i + 1], (
            f"thrust not monotone: col={cols[i]:.2f} -> {thrusts[i]:.1f} N, "
            f"col={cols[i+1]:.2f} -> {thrusts[i+1]:.1f} N"
        )
