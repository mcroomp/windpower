"""
torque/test_slow_rpm.py — Slow sinusoidal rotor hub RPM variation test.

Profile: omega_rotor = 28 + 5·sin(2π·0.05·t) rad/s  (±18%, 20 s period)

The motor PID must track the slowly changing RPM and maintain counter-rotation
to keep yaw rate within ±2°/s throughout the observation window.

Telemetry → simulation/logs/torque_telemetry_slow_vary.csv
"""
from __future__ import annotations

import math
import pytest

from torque_test_utils  import run_observation_loop, save_telemetry, assert_yaw_rate

_SETTLE_S   = 40.0                  # s  -- EKF settle + give PID time to adapt to varying RPM
_OBSERVE_S  = 30.0                  # s  -- longer window because RPM variation is slow
_THRESHOLD  = math.radians(2.0)     # [rad/s] -- more lenient than constant-RPM test


@pytest.mark.parametrize("torque_armed_profile", ["slow_vary"], indirect=True)
def test_slow_rpm(torque_armed_profile):
    """
    Rotor hub speed varies sinusoidally at 0.05 Hz (±18% of nominal).
    The yaw rate PID must reject the slowly varying RPM disturbance.
    Pass: max |ψ_dot| < 2°/s after 40 s settle.
    """
    ctx = torque_armed_profile
    rows: list = []

    obs = run_observation_loop(
        ctx=ctx, rows=rows,
        settle_s=_SETTLE_S, observe_s=_OBSERVE_S,
        timeout_s=_SETTLE_S + _OBSERVE_S + 20.0,
    )

    save_telemetry(rows, "slow_vary", ctx.log)
    assert_yaw_rate(obs, _THRESHOLD, _SETTLE_S, ctx.log)
