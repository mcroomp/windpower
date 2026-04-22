"""
torque/test_servo_tail_yaw_regulation.py -- H_TAIL_TYPE=0 servo activation test.

Checks that ArduPilot's ATC_RAT_YAW PID actually commands SERVO4 away from
the 1500 µs neutral when the hub drifts (constant-RPM profile).

Pass criterion
--------------
  During the 20 s observation window (after 30 s settle):
    max |servo4_us - 1500| >= 100 µs  (ArduPilot is commanding something)
"""
from __future__ import annotations

from torque_test_utils import run_observation_loop, save_telemetry

_SETTLE_S                = 30.0   # startup_hold(15) + 15 s for PID to build
_OBSERVE_S               = 20.0
_MOTOR_IDLE_US           = 1500   # SERVO4_TRIM: servo neutral
_ACTIVATION_THRESHOLD_US = 100    # µs from neutral


def test_servo_tail_activation(torque_armed_servo_tail):
    """
    H_TAIL_TYPE=0 + SERVO4_TRIM=1500: servo must deviate >= 100 µs from neutral.

    Confirms ArduPilot's ATC_RAT_YAW PID responds to hub yaw drift by
    commanding SERVO4.  Direction (above or below 1500) tells us the sign
    convention for H_TAIL_TYPE=0 relative to the GB4008 motor.
    """
    ctx = torque_armed_servo_tail

    _, rows = run_observation_loop(ctx, _SETTLE_S, _OBSERVE_S)
    save_telemetry(rows, "servo_tail_activation", ctx.log)

    servo4_samples = [r.servo4_us for r in rows if r.t_sim >= _SETTLE_S and r.servo4_us > 0]
    if not servo4_samples:
        raise AssertionError("No SERVO4 samples collected in observation window")

    max_deviation = max(abs(v - _MOTOR_IDLE_US) for v in servo4_samples)
    direction = "above" if max(servo4_samples) - _MOTOR_IDLE_US > _MOTOR_IDLE_US - min(servo4_samples) else "below"

    ctx.log.info(
        "SERVO4: max_deviation=%d µs from %d (%s neutral)  samples=%d",
        max_deviation, _MOTOR_IDLE_US, direction, len(servo4_samples),
    )

    assert max_deviation >= _ACTIVATION_THRESHOLD_US, (
        f"SERVO4 max deviation {max_deviation} µs < {_ACTIVATION_THRESHOLD_US} µs — "
        f"ATC_RAT_YAW PID not driving SERVO4 (H_TAIL_TYPE=0 not wired?)"
    )
