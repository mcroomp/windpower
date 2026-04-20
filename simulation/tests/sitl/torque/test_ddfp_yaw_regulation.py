"""
torque/test_ddfp_yaw_regulation.py — ArduPilot DDFP (H_TAIL_TYPE=4) yaw regulation tests.

Prescribed-yaw tests (open-loop plant, closed-loop control law):
  test_ddfp_zero_yaw  — motor stays near trim when psi_dot=0
  test_ddfp_yaw_ramp  — motor rises above trim when psi_dot ramps 0->10 deg/s

Kinematic closed-loop test (both plant and control in the loop):
  test_ddfp_no_action_at_rest  — psi_dot=0 prescribed → motor near trim (does nothing)
  test_ddfp_responds_to_drift  — kinematic start (psi_dot=28 rad/s) → motor throttle rises
  test_ddfp_kinematic_regulation — kinematic, motor regulates psi_dot to < 5 deg/s

H_TAIL_TYPE=4 (DDFP CCW): hub under-speed (RPM < omega_hub*gear_ratio) causes CW drift
→ yaw error positive → DDFP CCW sign-flip → positive motor throttle → counters CW drift.

Run with (inside Docker)
------------------------
  RAWES_RUN_STACK_INTEGRATION=1 pytest \\
      simulation/tests/sitl/torque/test_ddfp_yaw_regulation.py -v
"""
from __future__ import annotations

import math

from torque_test_utils import (
    run_observation_loop,
    save_telemetry,
    assert_physics_yaw_rate,
    assert_motor_throttle_response,
)

_OBSERVE_S = 10.0
_THRESHOLD = math.radians(5.0)   # 5 deg/s


def test_ddfp_zero_yaw(torque_armed_ddfp_zero):
    """
    Stationary hub — motor should stay near 800 µs (H_YAW_TRIM region).

    Pass criterion: |psi_dot| < 5 deg/s throughout the 10 s observation window
    starting immediately after DYNAMIC begins (no settle needed — hub never moves).
    """
    ctx = torque_armed_ddfp_zero

    _, rows = run_observation_loop(ctx, settle_s=20.0, observe_s=_OBSERVE_S, timeout_margin_s=15.0)

    save_telemetry(rows, "ddfp_zero", ctx.log)
    assert_physics_yaw_rate(ctx.events_log, _THRESHOLD, 20.0, _OBSERVE_S, ctx.log)


def test_ddfp_yaw_ramp(torque_armed_ddfp_ramp):
    """
    DDFP motor response to prescribed yaw ramp: psi_dot prescribed 0→10 deg/s
    over 30 s.  ArduPilot should drive motor throttle above the zero-yaw
    equilibrium value in proportion to the error.

    Prescribed yaw is used (psi_dot set directly) rather than the kinematic model
    driven by ArduPilot throttle, so the test isolates the control-law response.
    Pass criterion: motor throttle >= 0.05 at t_dyn=25-35 s when psi_dot=8-10 deg/s.
    """
    ctx = torque_armed_ddfp_ramp

    _, rows = run_observation_loop(ctx, settle_s=40.0, observe_s=_OBSERVE_S, timeout_margin_s=25.0)

    save_telemetry(rows, "ddfp_ramp", ctx.log)
    assert_motor_throttle_response(ctx.events_log, 0.05, 40.0, _OBSERVE_S, ctx.log)


# ---------------------------------------------------------------------------
# Kinematic closed-loop tests  (torque_armed_ddfp fixture, profile="constant")
# ---------------------------------------------------------------------------

def test_ddfp_no_action_at_rest(torque_armed_ddfp_zero):
    """
    When psi_dot=0 throughout DYNAMIC, motor throttle should stay near the
    equilibrium trim — ArduPilot does nothing extra.

    Reuses torque_armed_ddfp_zero (prescribed psi_dot=0) to establish the
    baseline: motor throttle <= 0.02 above equilibrium at all times.
    """
    ctx = torque_armed_ddfp_zero

    _, rows = run_observation_loop(ctx, settle_s=17.0, observe_s=_OBSERVE_S, timeout_margin_s=13.0)

    save_telemetry(rows, "ddfp_no_action", ctx.log)
    assert_physics_yaw_rate(ctx.events_log, _THRESHOLD, 17.0, _OBSERVE_S, ctx.log)


def test_ddfp_responds_to_drift(torque_armed_ddfp):
    """
    Kinematic model starts with omega_motor=0 so psi_dot = omega_rotor = 28 rad/s.
    ArduPilot must raise motor throttle above zero in response.

    Observe the first 10 s of DYNAMIC: motor throttle must exceed 0.05 at some point.
    """
    ctx = torque_armed_ddfp

    _, rows = run_observation_loop(ctx, settle_s=15.0, observe_s=_OBSERVE_S, timeout_margin_s=15.0)

    save_telemetry(rows, "ddfp_responds", ctx.log)
    assert_motor_throttle_response(ctx.events_log, 0.05, 15.0, _OBSERVE_S, ctx.log)


def test_ddfp_kinematic_regulation(torque_armed_ddfp):
    """
    Full closed-loop kinematic test: ArduPilot must regulate psi_dot to < 5 deg/s.

    The motor starts from rest (omega_motor=0), causing large initial CW drift.
    The DDFP controller drives the motor, the kinematic model feeds psi_dot back
    to ArduPilot via the gyro.  After a settle window, psi_dot must be small.
    """
    ctx = torque_armed_ddfp

    _, rows = run_observation_loop(ctx, settle_s=65.0, observe_s=_OBSERVE_S, timeout_margin_s=15.0)

    save_telemetry(rows, "ddfp_kinematic", ctx.log)
    assert_physics_yaw_rate(ctx.events_log, _THRESHOLD, 65.0, _OBSERVE_S, ctx.log)
