"""
torque/test_wobble_sitl.py — Fixed-RPM wobble torque test.

Uses the canonical torque stack with the ``wobble`` mediator profile:
constant nominal rotor RPM, but aggressive roll/pitch wobble. This keeps the
setup canonical while adding a distinct disturbance class to the yaw loop.

Pass criterion
--------------
  After settle: actual physics |psi_dot| remains bounded during the wobble
  window. Physics ground truth is used rather than EKF yawspeed.
"""
from __future__ import annotations

import math

import pytest

pytestmark = pytest.mark.sitl

from torque_test_utils import run_observation_loop, save_telemetry, assert_physics_yaw_rate

_SETTLE_S = 95.0
_OBSERVE_S = 20.0
_THRESHOLD = math.radians(15.0)


@pytest.mark.parametrize("torque_armed_profile", ["wobble"], indirect=True)
def test_wobble_sitl(torque_armed_profile):
    """
    Rotor RPM stays fixed at nominal while the hub wobbles in roll/pitch.

    This is the canonical fixed-RPM disturbance test for the yaw regulator.
    """
    ctx = torque_armed_profile

    _, rows = run_observation_loop(ctx, _SETTLE_S, _OBSERVE_S)

    save_telemetry(rows, ctx.test_log_dir, ctx.log)
    assert_physics_yaw_rate(ctx.events_log, _THRESHOLD, _SETTLE_S, _OBSERVE_S, ctx.log)