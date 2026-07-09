"""
torque/test_torque_production_vanilla_sitl.py — Production-style torque check.

Goal
----
Exercise the torque rig using the VANILLA SITL boot parameter chain
(copter-heli + rawes_common_defaults + rawes_sitl_defaults), with rawes.lua
loaded in MODE_PASSIVE, then verify yaw-rate regulation while rotor speed varies.

Scenario
--------
- STARTUP hold: stationary hub, arm in GUIDED_NOGPS.
- DYNAMIC spinup: mediator_torque ramps rotor from 0 to 200 RPM over 10 s.
- Constant hold: profile="constant" keeps the rotor at 200 RPM (no variation),
  isolating the yaw-PID limit-cycle behaviour from any rotor-speed disturbance.

Pass criteria
-------------
1) Physics yaw rate stays bounded after settle.
2) Motor throttle responds (non-trivial actuation) in the observation window.
"""
from __future__ import annotations

import math

from torque_test_utils import (
    run_observation_loop,
    save_telemetry,
    assert_physics_yaw_rate,
    assert_motor_throttle_response,
)

# dynamics_t settle time: 15 s startup + 10 s spinup + extra margin for yaw trim
# convergence under varying rotor speed.
_SETTLE_S = 55.0
_OBSERVE_S = 20.0
_MAX_PSI_DOT_RAD_S = math.radians(12.0)


def test_torque_production_vanilla_lua_sitl(torque_production_vanilla_lua):
    """
    Vanilla SITL defaults + Lua PASSIVE at a constant 200 RPM.

    This is intentionally looser than the dedicated torque-tuned tests: it is a
    production-like sanity check, not a torque PID tuning gate.
    """
    ctx = torque_production_vanilla_lua

    _, rows = run_observation_loop(
        ctx,
        settle_s=_SETTLE_S,
        observe_s=_OBSERVE_S,
        timeout_margin_s=25.0,
    )

    save_telemetry(rows, "torque_production_vanilla", ctx.log)

    # Track quality from physics ground-truth.
    assert_physics_yaw_rate(
        ctx.events_log,
        _MAX_PSI_DOT_RAD_S,
        _SETTLE_S,
        _OBSERVE_S,
        ctx.log,
    )

    # Ensure controller actually drives the motor in response to varying torque.
    assert_motor_throttle_response(
        ctx.events_log,
        min_throttle=0.05,
        settle_s=_SETTLE_S,
        observe_s=_OBSERVE_S,
        log=ctx.log,
    )
