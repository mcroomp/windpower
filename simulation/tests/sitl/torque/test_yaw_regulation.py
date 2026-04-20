"""
torque/test_yaw_regulation.py — Counter-torque motor stack test.

Verifies that ArduPilot SITL can hold hub yaw steady while the GB4008
anti-rotation motor counter-rotates against the spinning rotor hub.

Physical scenario
-----------------
  * Rotor hub spins at ~28 rad/s (nominal RAWES autorotation at 10 m/s wind)
  * The motor counter-rotates via the 80:44 gear to maintain inner assembly heading
  * The ESC holds commanded RPM; bearing/swashplate drag only affect power draw
  * ArduPilot (heli frame, ACRO mode) senses the yaw rate via gyro and
    commands Ch4 (H_TAIL_TYPE=4 DDFP, servo) to control GB4008 motor speed

Pass criterion
--------------
  After 60 s of ACRO with neutral sticks:
    * max |psi_dot|  < 5 deg/s over the last 20 s  (physics ground truth)

  Note: ACRO mode controls yaw RATE (not angle), so the hub may settle at a
  non-zero yaw angle.  Only the rate is asserted.  The yaw angle represents
  the hub's operating heading and has no operational significance.

Telemetry
---------
  The test writes a CSV log to simulation/logs/torque_telemetry_yaw_regulation.csv
  after each run.

Run with (inside Docker)
------------------------
  RAWES_RUN_STACK_INTEGRATION=1 pytest simulation/tests/sitl/torque/test_yaw_regulation.py -v
"""
from __future__ import annotations

import math

from torque_test_utils import (
    run_observation_loop,
    save_telemetry,
    assert_physics_yaw_rate,
)

_SETTLE_S          = 60.0
_OBSERVE_S         = 20.0
_MAX_PSI_DOT_RAD_S = math.radians(5.0)   # [rad/s]

def test_yaw_regulation(torque_armed):
    """
    ArduPilot SITL regulates hub yaw using the DDFP (Ch4) output.

    ACRO mode with neutral sticks commands psi_dot = 0.  The yaw rate PID must
    build enough Ch4 output to maintain counter-rotation against the spinning
    axle and hold |psi_dot| < 5 deg/s after a 60 s settle period.

    Physics ground truth (mediator events log) is used — not ATTITUDE.yawspeed,
    which can carry compass-tilt artefacts.
    """
    ctx = torque_armed

    _, rows = run_observation_loop(ctx, _SETTLE_S, _OBSERVE_S)

    save_telemetry(rows, "yaw_regulation", ctx.log)
    assert_physics_yaw_rate(ctx.events_log, _MAX_PSI_DOT_RAD_S, _SETTLE_S, _OBSERVE_S, ctx.log)
