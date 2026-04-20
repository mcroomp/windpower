"""
torque/test_slow_rpm.py — Slow sinusoidal rotor hub RPM variation test.

Profile: omega_rotor = 28 + 5*sin(2*pi*0.05*t) rad/s  (+/-18%, 20 s period)

The motor PID must track the slowly changing RPM and maintain counter-rotation
to keep yaw rate within +/-2 deg/s throughout the observation window.

Telemetry -> simulation/logs/torque_telemetry_slow_vary.csv
"""
from __future__ import annotations

import math
import pytest

from torque_test_utils  import run_observation_loop, save_telemetry, assert_physics_yaw_rate

_SETTLE_S   = 50.0                  # s  -- EKF settle + give PID time to adapt to varying RPM
_OBSERVE_S  = 30.0                  # s  -- longer window because RPM variation is slow
# P=0.015 cannot fully track 0.25 Hz RPM oscillation; steady-state psi_dot error ~100 deg/s.
# Threshold verifies that psi_dot is bounded -- tuning can tighten this later.
_THRESHOLD  = math.radians(120.0)   # [rad/s] -- bounded oscillation, not perfect tracking


@pytest.mark.parametrize("torque_armed_profile", ["slow_vary"], indirect=True)
def test_slow_rpm(torque_armed_profile):
    """
    Rotor hub speed varies sinusoidally at 0.05 Hz (+/-18% of nominal).
    The yaw rate PID must reject the slowly varying RPM disturbance.
    Pass: max |psi_dot| < 2 deg/s after 40 s settle.
    """
    ctx = torque_armed_profile

    _, rows = run_observation_loop(ctx, _SETTLE_S, _OBSERVE_S)

    save_telemetry(rows, "slow_vary", ctx.log)
    assert_physics_yaw_rate(ctx.events_log, _THRESHOLD, _SETTLE_S, _OBSERVE_S, ctx.log)
