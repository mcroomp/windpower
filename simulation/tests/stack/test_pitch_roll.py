"""
torque/test_pitch_roll.py — Hub pitch/roll oscillation with yaw regulation.

Profile: axle speed constant at nominal; hub slowly oscillates in pitch
(±8°, 0.05 Hz) and roll (±12°, 0.08 Hz) to simulate tether-induced swinging.

The yaw rate PID must continue regulating yaw despite the tilted body
frame — gravity projects into the horizontal body axes when tilted, and
the gyro Z component changes with tilt angle.

Pass criterion: max |ψ_dot| < 2°/s after 40 s settle, confirming that
tilt does not break the yaw control loop.

Telemetry → simulation/logs/torque_telemetry_pitch_roll.csv
"""
from __future__ import annotations

import math
import pytest

from torque_test_utils  import run_observation_loop, save_telemetry, assert_yaw_rate

_SETTLE_S   = 40.0
_OBSERVE_S  = 20.0
_THRESHOLD  = math.radians(2.0)   # [rad/s] -- same as constant-RPM test; tilt shouldn't degrade yaw


@pytest.mark.parametrize("torque_armed_profile", ["pitch_roll"], indirect=True)
def test_pitch_roll(torque_armed_profile):
    """
    Hub tilts in pitch (±8°, 0.05 Hz) and roll (±12°, 0.08 Hz) while
    the rotor hub spins at constant nominal RPM.  Gravity projection changes
    the accelerometer and gyro Z in the body frame.

    Pass: yaw rate stays within ±2°/s throughout, confirming that the
    yaw PID is decoupled from the pitch/roll motion.
    """
    ctx = torque_armed_profile
    rows: list = []

    # Disable GPS position/velocity fusion for this test — hub tilt projects
    # gravity into horizontal axes (g·sin θ ≈ 0.5 m/s² at ±5°), which looks
    # like horizontal motion to the EKF and triggers GPS Glitch false positives.
    for pname, pval in [
        ("EK3_SRC1_POSXY", 0),   # no position source
        ("EK3_SRC1_VELXY", 0),   # no velocity source
    ]:
        ok = ctx.gcs.set_param(pname, pval, timeout=3.0)
        ctx.log.info("  %-25s = %g  ACK=%s", pname, pval, ok)

    obs = run_observation_loop(
        ctx=ctx, rows=rows,
        settle_s=_SETTLE_S, observe_s=_OBSERVE_S,
        timeout_s=_SETTLE_S + _OBSERVE_S + 20.0,
    )

    save_telemetry(rows, "pitch_roll", ctx.log)
    assert_yaw_rate(obs, _THRESHOLD, _SETTLE_S, ctx.log)
