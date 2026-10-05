"""
torque/test_yaw_regulation_sitl.py — Counter-torque motor stack test.

Verifies that ArduPilot SITL can hold hub yaw steady while the GB4008
anti-rotation motor counter-rotates against the spinning rotor hub.

Physical scenario
-----------------
  * Rotor hub spins at ~28 rad/s (nominal RAWES autorotation at 10 m/s wind)
  * The motor counter-rotates via the 10:1 gear to maintain inner assembly heading
  * The ESC holds commanded RPM; bearing/swashplate drag only affect power draw
  * ArduPilot (heli frame, GUIDED_NOGPS with Lua PASSIVE) senses the yaw rate via gyro and
    commands the Motor4 output to control GB4008 motor speed

Pass criterion
--------------
  After the model-based yaw trim has converged with neutral sticks:
    * max |psi_dot|  <= 16 deg/s over the last 20 s  (physics ground truth)

  Lua PASSIVE holds its captured attitude through GUIDED_NOGPS. This test
  asserts the physical spin rate about the tilted rotor axis, not EKF Euler
  yawspeed or heading error.

Telemetry
---------
  The test writes a CSV log to simulation/logs/test_yaw_regulation_sitl/telemetry.csv
  after each run.

Run with
------------------------
  bash test.sh stack -n 1 -k test_yaw_regulation_sitl
"""
from __future__ import annotations

import math

import pytest
from tests.sitl.torque.torque_test_utils import (
    run_observation_loop,
    save_telemetry,
    assert_physics_yaw_rate,
)

# Absolute mediator time: startup_hold(15) + 100 s dynamic convergence.
_SETTLE_S          = 115.0
_OBSERVE_S         = 20.0
_MAX_PSI_DOT_RAD_S = math.radians(16.0)   # [rad/s]

pytestmark = pytest.mark.sitl

@pytest.mark.timeout(2400)
def test_yaw_regulation_sitl(torque_armed):
    """
    ArduPilot SITL regulates hub yaw using the DDFP Motor4 output.

    Lua PASSIVE commands its captured attitude through GUIDED_NOGPS. The yaw
    rate PID and model-based trim observer must build enough motor output to
    maintain counter-rotation against the spinning axle.

    Physics ground truth (mediator events log) is used — not ATTITUDE.yawspeed,
    which can carry compass-tilt artefacts.
    """
    ctx = torque_armed

    _, rows = run_observation_loop(ctx, _SETTLE_S, _OBSERVE_S)

    save_telemetry(rows, ctx.test_log_dir, ctx.log)
    assert_physics_yaw_rate(ctx.events_log, _MAX_PSI_DOT_RAD_S, _SETTLE_S, _OBSERVE_S, ctx.log)
