"""
torque/test_slow_rpm_sitl.py — Varying-RPM torque test.

Uses the canonical torque stack with the ``slow_vary`` mediator profile:
slow sinusoidal rotor-speed variation with no special actuator or non-standard
tail setup. This is the canonical RPM-disturbance test for yaw regulation.

Pass criterion
--------------
  After settle: actual physics |psi_dot| remains bounded throughout the RPM
  sweep window. Physics ground truth is used rather than EKF yawspeed.
"""
from __future__ import annotations

import math

import pytest

pytestmark = pytest.mark.sitl

from torque_test_utils import run_observation_loop, save_telemetry, assert_physics_yaw_rate

_SETTLE_S = 65.0
_OBSERVE_S = 30.0
_THRESHOLD = math.radians(120.0)


@pytest.mark.parametrize("torque_armed_profile", ["slow_vary"], indirect=True)
def test_slow_rpm_sitl(torque_armed_profile):
    """
    Rotor RPM varies slowly around nominal while the yaw loop stays canonical.

    This is the canonical varying-RPM disturbance test for the yaw regulator.
    """
    ctx = torque_armed_profile

    _, rows = run_observation_loop(ctx, _SETTLE_S, _OBSERVE_S)

    save_telemetry(rows, ctx.test_log_dir, ctx.log)
    assert_physics_yaw_rate(ctx.events_log, _THRESHOLD, _SETTLE_S, _OBSERVE_S, ctx.log)