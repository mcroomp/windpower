"""
torque/test_fast_rpm.py — Fast sinusoidal rotor hub RPM variation test.

Profile: omega_rotor = 28 + 5·sin(2π·0.25·t) rad/s  (±18%, 4 s period)

The motor PID must track rapidly changing RPM and maintain counter-rotation
to keep yaw rate within ±3°/s.  Higher threshold than slow_vary because
the PID has less time to adapt between RPM peaks.

Telemetry → simulation/logs/torque_telemetry_fast_vary.csv
"""
from __future__ import annotations

import math
import pytest

from torque_test_utils  import run_observation_loop, save_telemetry, assert_yaw_rate

_SETTLE_S   = 40.0
_OBSERVE_S  = 20.0
_THRESHOLD  = math.radians(3.0)   # [rad/s] -- more lenient: 4 s period leaves little settling time


@pytest.mark.parametrize("torque_armed_profile", ["fast_vary"], indirect=True)
def test_fast_rpm(torque_armed_profile):
    """
    Rotor hub speed varies sinusoidally at 0.25 Hz (±18% of nominal, 4 s period).
    The yaw rate PID must reject the rapidly oscillating RPM disturbance.
    Pass: max |ψ_dot| < 3°/s after 40 s settle.
    """
    ctx = torque_armed_profile
    rows: list = []

    obs = run_observation_loop(
        ctx=ctx, rows=rows,
        settle_s=_SETTLE_S, observe_s=_OBSERVE_S,
        timeout_s=_SETTLE_S + _OBSERVE_S + 20.0,
    )

    save_telemetry(rows, "fast_vary", ctx.log)
    assert_yaw_rate(obs, _THRESHOLD, _SETTLE_S, ctx.log)
