"""
torque/test_disarm_stops_motor_sitl.py — safety: disarm cuts the motor.

Verifies that if the vehicle is DISARMED while the counter-torque rig is
running (rotor spinning, GB4008 motor actively counter-rotating on SERVO4),
the motor output collapses to the OFF PWM (SERVO4_MIN) and stays there.

Scenario
--------
  1. torque_armed fixture: rotor spins up, ArduPilot's DDFP yaw PID drives
     SERVO4 to hold hub yaw -- so the motor is producing real throttle.
  2. Run until SERVO4 is clearly ON (well above SERVO4_MIN).
  3. Force-disarm mid-run.
  4. Confirm SERVO4 falls to ~SERVO4_MIN (motor off) and stays there.

Pass criterion
--------------
  * Before disarm: SERVO4 > _MOTOR_ON_US (motor running).
  * After disarm : SERVO4 <= _MOTOR_OFF_US for the whole post-disarm window.
"""
from __future__ import annotations

from stack_infra import observe

# GB4008 DDFP PWM range: 800 us = off, 2000 us = full (SERVO4_MIN/MAX).
_MOTOR_OFF_US = 850.0    # <= this counts as "off" (SERVO4_MIN=800 + margin)
_MOTOR_ON_US  = 1000.0   # >  this counts as "running"

_SPINUP_RUN_S = 35.0     # observe seconds before disarm (dynamics well underway)
_POST_DISARM_S = 6.0     # observe seconds after disarm


def test_disarm_stops_motor_sitl(torque_armed):
    """Disarming mid-run must cut the GB4008 motor (SERVO4 -> off)."""
    ctx = torque_armed
    gcs = ctx.gcs
    log = ctx.log

    # ── Phase 1: run until the motor is clearly driving ──────────────────────
    last_pwm = [0]

    def observe_running(msg, t_rel):
        if msg is not None and msg.get_type() == "SERVO_OUTPUT_RAW":
            last_pwm[0] = getattr(msg, "servo4_raw", 0) or 0
        # Keep the motor interlock (CH8) high while armed.
        return t_rel >= _SPINUP_RUN_S

    observe(
        ctx, _SPINUP_RUN_S + 10.0, observe_running,
        msg_types=["SERVO_OUTPUT_RAW", "STATUSTEXT"],
        keepalive={8: 2000},
    )
    log.info("Pre-disarm SERVO4 = %d us", last_pwm[0])
    assert last_pwm[0] > _MOTOR_ON_US, (
        f"Motor not running before disarm (SERVO4={last_pwm[0]} us, "
        f"need > {_MOTOR_ON_US:.0f}); cannot verify disarm cuts it."
    )

    # ── Phase 2: force-disarm mid-run ────────────────────────────────────────
    gcs.disarm(force=True)

    # ── Phase 3: motor must be OFF and stay OFF ──────────────────────────────
    # Allow a brief settle for the disarm to propagate to the servo output,
    # then require every sample in the remaining window to be off.
    post_pwm: list[int] = []

    def observe_stopped(msg, t_rel):
        if msg is not None and msg.get_type() == "SERVO_OUTPUT_RAW":
            pwm = getattr(msg, "servo4_raw", 0) or 0
            if t_rel >= 1.0:           # skip the first 1 s propagation transient
                post_pwm.append(pwm)
        return None

    # No CH8 keepalive here -- the vehicle is disarmed and must stay that way.
    observe(
        ctx, _POST_DISARM_S, observe_stopped,
        msg_types=["SERVO_OUTPUT_RAW", "STATUSTEXT"],
    )

    assert post_pwm, "No SERVO_OUTPUT_RAW samples captured after disarm."
    max_pwm = max(post_pwm)
    log.info("Post-disarm SERVO4: n=%d  max=%d us  (limit %.0f)",
             len(post_pwm), max_pwm, _MOTOR_OFF_US)
    assert max_pwm <= _MOTOR_OFF_US, (
        f"Motor did NOT stop after disarm: SERVO4 max={max_pwm} us "
        f"(need <= {_MOTOR_OFF_US:.0f}). Disarm failed to cut the motor."
    )
