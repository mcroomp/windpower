"""
torque/test_lua_yaw_trim.py — Lua feedforward yaw regulation test.

This test validates the full hardware-equivalent control architecture:

  mediator_torque.py       ArduPilot SITL
  ─────────────────        ─────────────────────────────────────────
  Hub yaw dynamics    ←─── SERVO9 PWM  (Ch9, Script 1)
  Bearing drag model        ↑
  Motor torque model        rawes.lua (SCR_USER6=2, yaw mode):
  Sends JSON sensors ──────→ rpm:get_rpm(1)   ← motor RPM from "rpm" field
  (pos, vel, att,            compute trim(RPM, V_bat)
   gyro, accel, RPM)        + Kp × gyro.z
                             write PWM to Ch9

Key differences from test_yaw_regulation (which uses mediator adaptive trim):
  • Mediator is pure physics — no feedforward computation, linear PWM→throttle
  • rawes.lua (yaw mode) runs inside SITL, reads motor RPM, computes feedforward
  • The Lua script is identical to what would run on the Pixhawk 6C hardware
  • RPM1_TYPE=10 reads motor RPM from the JSON sensor packet (SITL equivalent
    of DSHOT bidirectional telemetry from the AM32 ESC on hardware)
  • SERVO9_FUNCTION=94 gives Lua exclusive control of Ch9 (tail motor)

Pass criterion
--------------
  After 40 s settle: max |ψ_dot| < 1°/s over 20 s observation window.
  Same threshold as the adaptive-trim baseline — Lua should match it.

Telemetry → simulation/logs/torque_telemetry_lua.csv
"""
from __future__ import annotations

import math

from torque_test_utils  import run_observation_loop, save_telemetry, assert_yaw_rate

_SETTLE_S   = 40.0
_OBSERVE_S  = 20.0
_THRESHOLD  = math.radians(1.0)   # [rad/s]


def test_lua_yaw_trim(torque_armed_lua):
    """
    Yaw rate regulated by rawes.lua (SCR_USER6=2) running inside ArduPilot SITL.

    The Lua script reads motor RPM (via SITL JSON → RPM1_TYPE=10), computes
    the equilibrium throttle feedforward, adds a proportional yaw rate
    correction from the onboard gyro, and writes the result directly to
    SERVO9 (Ch9).  The mediator applies this as a pure motor torque command
    with no additional feedforward of its own.

    This is the closest SITL equivalent to the hardware deployment:
      RPM1_TYPE=5 (DSHOT ESC telemetry) on the Pixhawk 6C.
    """
    ctx = torque_armed_lua
    rows: list = []

    obs = run_observation_loop(
        ctx=ctx, rows=rows,
        settle_s=_SETTLE_S, observe_s=_OBSERVE_S,
        timeout_s=_SETTLE_S + _OBSERVE_S + 20.0,
    )

    save_telemetry(rows, "lua", ctx.log)
    assert_yaw_rate(obs, _THRESHOLD, _SETTLE_S, ctx.log)
