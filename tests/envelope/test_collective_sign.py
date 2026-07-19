"""
test_collective_sign.py -- Verify collective sign convention.

Uses the same setup as test_hover_sign.py:
  - Disk horizontal: body_z = [0,0,+1] (down through disk, per project FRD
    convention -- see design/aero_conventions.md)
  - Hub falling at 2 m/s downward
  - No wind
  - omega = 28 rad/s

At this operating point, increasing collective should increase thrust
(upward force = more negative F_world[2]).

Sign chain (from test_hover_sign.py / design/aero_conventions.md):
  upward thrust = -F_world[2].
  More collective -> more thrust -> F_world[2] more negative.
"""
import numpy as np

from simulation.frames import build_orb_frame
from tests.unit._aero_probe import load_rotor, make_probe, probe_steady

R_HORIZONTAL = build_orb_frame(np.array([0., 0., 1.]))   # FRD: body_z DOWN through disk

V_FALL  = np.array([0., 0., 2.0])   # hub falling 2 m/s downward
NO_WIND = np.zeros(3)
OMEGA   = 28.0
T_STEADY = 20.0

_AERO = make_probe(load_rotor("beaupoil_2026"))


def _thrust(col):
    r = probe_steady(
        _AERO,
        collective_rad = col,
        R_hub          = R_HORIZONTAL,
        v_hub_world    = V_FALL,
        omega_rotor    = OMEGA,
        wind_world     = NO_WIND,
    )
    return -float(r.F_world[2])   # upward thrust > 0


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
    """Thrust must increase monotonically across the collective range.

    Kept within the steep attached-flow region: thrust rises steeply from
    col=-0.10 to col=+0.05, then plateaus/rolls off gently as blade AoA
    approaches alpha_stall_deg=13 deg (beaupoil_2026.yaml) at higher
    collective -- that roll-off is a legitimate BEM/stall nonlinearity, not
    a sign-convention bug, so it's out of scope for this check.
    """
    cols   = [-0.10, -0.05, 0.0, 0.02, 0.05]
    thrusts = [_thrust(c) for c in cols]
    for i in range(len(thrusts) - 1):
        assert thrusts[i] < thrusts[i + 1], (
            f"thrust not monotone: col={cols[i]:.2f} -> {thrusts[i]:.1f} N, "
            f"col={cols[i+1]:.2f} -> {thrusts[i+1]:.1f} N"
        )
