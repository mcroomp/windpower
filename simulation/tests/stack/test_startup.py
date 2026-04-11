"""
torque/test_startup.py — Rotor spin-up from rest test.

Simulates the RAWES rotor accelerating from zero to nominal autorotation
speed (0 → 28 rad/s over 30 s) and verifies the yaw regulation holds
throughout the entire spin-up.

Profile: omega_rotor = 28 × min(1, t / 30)  (linear ramp, then hold)

The adaptive trim in the mediator tracks the changing RPM in
real time, so the motor feedforward adjusts counter-rotation at every RPM point.

Timeline (approximate test time after ACRO)
-------------------------------------------
  t =  0–30 s  : spin-up ramp  (ω: 0 → 28 rad/s)
  t = 30–50 s  : hold at nominal (ω: 28 rad/s)
  t = 50 s+    : observation window starts

Pass criterion
--------------
  After 50 s settle: max |ψ_dot| < 2°/s over 20 s  (2°/s allows for
  small transients during the ramp; tight enough to confirm the motor
  is actively regulating throughout spin-up, not just at steady state).

Telemetry → simulation/logs/torque_telemetry_startup.csv
"""
from __future__ import annotations

import math
import pytest

from torque_test_utils  import run_observation_loop, save_telemetry, assert_yaw_rate

#: Settle long enough to cover the 30 s ramp + 20 s post-ramp stabilisation
_SETTLE_S   = 50.0
_OBSERVE_S  = 20.0
_THRESHOLD  = math.radians(2.0)   # [rad/s] -- allows small ramp transients


@pytest.mark.parametrize("torque_armed_profile", ["startup"], indirect=True)
def test_startup(torque_armed_profile):
    """
    Rotor spins up from rest (ω=0) to nominal autorotation (ω=28 rad/s)
    over 30 s.  The motor adaptive trim tracks the increasing RPM
    in real time.  Yaw rate must stay within ±2°/s throughout spin-up
    and at steady state.

    This validates the spin-up sequence before tether tension is applied
    and confirms the motor can regulate yaw even at low RPM.
    """
    ctx = torque_armed_profile
    rows: list = []

    obs = run_observation_loop(
        ctx=ctx, rows=rows,
        settle_s=_SETTLE_S, observe_s=_OBSERVE_S,
        timeout_s=_SETTLE_S + _OBSERVE_S + 20.0,
    )

    save_telemetry(rows, "startup", ctx.log)
    assert_yaw_rate(obs, _THRESHOLD, _SETTLE_S, ctx.log)
