"""
torque/test_lua_yaw_trim.py — Lua PI yaw regulation test.

This test validates the full hardware-equivalent control architecture:

  mediator_torque.py       ArduPilot SITL
  -----------------        -----------------------------------------
  Hub yaw dynamics    <--- SERVO9 PWM  (Ch9, Script 1)
  Bearing drag model        ^
  Motor torque model        rawes.lua (SCR_USER6=2, MODE_YAW):
  Sends JSON sensors -----> ahrs:get_gyro().z
  (pos, vel, att,            PI: throttle = BASE + KP*gyro_z + I_term
   gyro, accel)              write PWM to Ch9 via SRV_Channels

Key differences from test_yaw_regulation (which uses mediator adaptive trim):
  - Mediator is pure physics -- no feedforward, linear PWM->throttle
  - rawes.lua (MODE_YAW (SCR_USER6=2)) runs inside SITL, reads gyro.z, runs PI controller
  - The Lua script is identical to what would run on the Pixhawk 6C hardware
  - No RPM feedback: RPM1_TYPE=0 on hardware until AM32 EDT is enabled
  - SERVO9_FUNCTION=94 gives Lua exclusive control of Ch9 (tail motor)

Pass criterion
--------------
  After 65 s settle: max |psi_dot| < 5 deg/s over 10 s observation window.
  The PI-only controller (no RPM feedforward) takes ~50 s from arm for the
  I term to ramp up and brake the hub, then settles to ~3 deg/s residual.
  5 deg/s matches the stability watchdog threshold and is realistic for
  hardware where the ESC manages RPM internally.

Telemetry -> simulation/logs/torque_telemetry_lua.csv
"""
from __future__ import annotations

import math

from torque_test_utils  import (
    run_observation_loop, save_telemetry,
    assert_physics_yaw_rate,
)

_SETTLE_S   = 65.0
_OBSERVE_S  = 10.0
_THRESHOLD  = math.radians(5.0)   # [rad/s] physics ground truth; 5 deg/s (matches stability watchdog)


def test_lua_yaw_trim(torque_armed_lua):
    """
    Yaw rate regulated by rawes.lua (SCR_USER6=2) running inside ArduPilot SITL.

    The Lua script reads gyro.z and runs a PI controller: the I term builds up
    the ~70% equilibrium throttle offset during startup spin-up, compensating
    the bearing drag without RPM feedback.  The mediator applies the throttle
    as a pure motor torque command with no additional feedforward.

    Pass criterion: actual physics psi_dot (from mediator log) stays within
    5 deg/s after 65 s settle.  The I term takes ~50 s from arm to ramp up
    through the startup spin-up phase; steady-state residual is ~3 deg/s.

    This is the closest SITL equivalent to the hardware deployment:
      RPM1_TYPE=0 (RPM unavailable until AM32 EDT enabled).
    """
    ctx = torque_armed_lua
    rows: list = []

    run_observation_loop(
        ctx=ctx, rows=rows,
        settle_s=_SETTLE_S, observe_s=_OBSERVE_S,
        timeout_s=_SETTLE_S + _OBSERVE_S + 10.0,
    )

    save_telemetry(rows, "lua", ctx.log)
    assert_physics_yaw_rate(ctx.events_log, _THRESHOLD, _SETTLE_S, _OBSERVE_S, ctx.log)
