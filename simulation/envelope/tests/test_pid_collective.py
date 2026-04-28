"""
test_pid_collective.py -- Verify the collective PI converges to v_target.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from envelope.point_mass import simulate_point


def _converged(history, key="v_along", window=10.0, tol=0.05):
    final = [h[key] for h in history if h["t"] >= history[-1]["t"] - window]
    return len(final) >= 3 and (max(final) - min(final)) < tol


@pytest.mark.parametrize("elevation,tension,v_target", [
    (70.0, 200.0,  0.8),   # reel-in, previously confirmed achievable
    (30.0, 400.0, -0.8),   # reel-out at low elevation
    (80.0, 200.0,  0.8),   # reel-in at high elevation
])
def test_pid_hits_target(elevation, tension, v_target):
    r = simulate_point(
        col=0.0,
        elevation_deg=elevation,
        tension_n=tension,
        wind_speed=10.0,
        v_target=v_target,
        t_end=40.0,
        dt=0.02,
    )
    eq = r["eq"]
    assert _converged(r["history"]), (
        f"el={elevation} T={tension} v_target={v_target:+.2f}: v_along did not converge"
    )
    assert not r["col_saturated"], (
        f"el={elevation} T={tension} v_target={v_target:+.2f}: "
        f"collective saturated at {eq.get('collective_deg', '?'):.1f} deg — "
        f"target may be physically unreachable"
    )
    v_along = eq["v_along"]
    assert abs(v_along - v_target) < 0.1, (
        f"el={elevation} T={tension}: v_along={v_along:+.4f} not within 0.1 of {v_target:+.2f}"
    )
