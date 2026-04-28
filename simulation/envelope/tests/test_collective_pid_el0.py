"""
test_collective_pid_el0.py -- Collective PID sign test at el=0, no gravity.

Setup
-----
  el=0 (horizontal tether pointing East), gravity=0, no wind.
  tether_hat = [0, +1, 0]  (East = toward anchor)
  bz         = [0, -1,  0]  (West = thrust direction)
  v_along    = dot(vel, tether_hat) = vel[1]  (positive = toward anchor)

Force balance at rest:
  tether pulls East  (+Y): +tension_n N
  thrust  pushes West(-Y): -thrust N
  net = 0 when thrust = tension_n

With omega_init=28 rad/s the rotor generates ~225 N at col=0 (from test_hover_sign).
Using tension=200 N keeps the hub near equilibrium without blowing up.

Cyclic PI is disabled (kp_cyc=ki_cyc=0) so it does not fight along-tether motion.

Expected behaviour of collective PI:
  v_target > 0 (toward anchor): PI increases collective to push hub East...

Wait -- thrust points West (bz=[0,-1,0]).  Increasing thrust pushes hub WEST,
i.e. v_along becomes MORE NEGATIVE (away from anchor).

So:
  v_target = -0.5  (reel-out / away from anchor): PI should raise collective
  v_target = +0.5  (reel-in  / toward anchor):    PI should lower collective (reduce thrust)
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from envelope.point_mass import simulate_point


TENSION  = 200.0
EL       = 0.0
WIND     = 5.0    # blows West = axial inflow through disk at el=0 → generates thrust
OMEGA    = 28.0
T_END    = 30.0
DT       = 0.02


def _run(v_target, col_init=0.0):
    return simulate_point(
        col           = col_init,
        elevation_deg = EL,
        tension_n     = TENSION,
        wind_speed    = WIND,
        omega_init    = OMEGA,
        t_end         = T_END,
        dt            = DT,
        gravity       = 0.0,
        kp_cyc        = 0.0,   # disable cyclic — it fights along-tether at el=0
        ki_cyc        = 0.0,
        v_target      = v_target,
        kp_col        = 0.01,
        ki_col        = 0.002,
    )


def _converged(history, key="v_along", window=10.0, tol=0.1):
    final = [h[key] for h in history if h["t"] >= history[-1]["t"] - window]
    return len(final) >= 3 and (max(final) - min(final)) < tol


def test_equilibrium_no_pid():
    """Without PID, col=0 should hold hub near rest (net force ~ 0)."""
    r = simulate_point(
        col=0.0, elevation_deg=EL, tension_n=TENSION,
        wind_speed=WIND, omega_init=OMEGA,
        t_end=T_END, dt=DT, gravity=0.0,
        kp_cyc=0.0, ki_cyc=0.0,
    )
    eq = r["eq"]
    assert abs(eq["v_along"]) < 1.0, (
        f"expected hub near rest without PID, got v_along={eq['v_along']:+.3f} m/s"
    )


def test_pid_reel_out():
    """PID targeting v_along=-0.5 (away from anchor) must converge."""
    r = _run(v_target=-0.5)
    assert _converged(r["history"]), "v_along did not converge for reel-out target"
    assert not r["col_saturated"], "collective saturated — target may be unreachable"
    v = r["eq"]["v_along"]
    assert abs(v - (-0.5)) < 0.15, f"v_along={v:+.3f} not close to -0.5"


def test_pid_reel_in():
    """PID targeting v_along=+0.5 (toward anchor) must converge."""
    r = _run(v_target=+0.5)
    assert _converged(r["history"]), "v_along did not converge for reel-in target"
    assert not r["col_saturated"], "collective saturated — target may be unreachable"
    v = r["eq"]["v_along"]
    assert abs(v - 0.5) < 0.15, f"v_along={v:+.3f} not close to +0.5"


def test_reel_out_uses_higher_collective_than_reel_in():
    """Reel-out (away) needs more thrust (higher col) than reel-in (toward)."""
    r_out = _run(v_target=-0.5)
    r_in  = _run(v_target=+0.5)
    col_out = r_out["eq"].get("collective_rad", float("nan"))
    col_in  = r_in["eq"].get("collective_rad",  float("nan"))
    assert col_out > col_in, (
        f"expected col_reel_out > col_reel_in: "
        f"out={math.degrees(col_out):.2f} deg  in={math.degrees(col_in):.2f} deg"
    )
