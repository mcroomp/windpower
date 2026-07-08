"""
torque/test_torque_motor_delay_sitl.py -- Production-style torque check with a
300 ms motor-response transport delay.

Goal
----
Same scenario as test_torque_production_vanilla_sitl (vanilla SITL boot chain,
rawes.lua in MODE_PASSIVE, rotor spun to 200 RPM with slow_vary variation), but
mediator_torque applies a 300 ms transport delay to the motor throttle before it
drives the yaw ODE.  This exercises the DDFP yaw regulator against a lagged
actuator -- a stand-in for real ESC / motor spin-up latency.

Scenario
--------
- STARTUP hold: stationary hub, arm in GUIDED_NOGPS.
- DYNAMIC spinup: mediator_torque ramps rotor from 0 to 200 RPM over 10 s.
- Slow variation: profile="slow_vary" adds a slow sinusoidal RPM sweep.
- Motor delay: the yaw motor reacts to the throttle commanded 300 ms earlier.

Pass criteria (same gates as the vanilla production test)
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
# convergence under varying rotor speed and the added actuator lag.
_SETTLE_S = 55.0
_OBSERVE_S = 20.0
_MAX_PSI_DOT_RAD_S = math.radians(12.0)


def test_torque_motor_delay_lua_sitl(torque_production_delayed_lua):
    """
    Vanilla SITL defaults + Lua PASSIVE at 200 RPM with slow speed variation and
    a 300 ms motor-response delay.

    Same looseness as the vanilla production check: a production-like sanity gate
    that the yaw regulator stays bounded and keeps driving the motor even with a
    lagged actuator, not a torque PID tuning gate.
    """
    ctx = torque_production_delayed_lua

    _, rows = run_observation_loop(
        ctx,
        settle_s=_SETTLE_S,
        observe_s=_OBSERVE_S,
        timeout_margin_s=25.0,
    )

    save_telemetry(rows, "torque_motor_delay", ctx.log)

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
