"""
torque/test_gust_recovery.py — Gust shock and yaw recovery test.

Profile: axle speed is nominal until dynamics_t=10 s, then a 5-second
gust drives it to 120% of nominal, then it returns to nominal.

Timeline (SITL absolute time):
  t =  0–15 s  : STARTUP hold (EKF bias settles)
  t = 25–30 s  : gust hits  — omega_rotor x1.20 for 5 s
  t = 30 s+    : gust ends  — omega_rotor returns to nominal
  t = 48 s+    : observation window starts (~18 s recovery time)

Pass criterion: max |psi_dot| < 2 deg/s in the observation window (physics
ground truth from mediator events log), confirming that the PID recovered
from the RPM-mismatch transient within ~18 s of gust end.

Telemetry -> simulation/logs/torque_telemetry_gust.csv
"""
from __future__ import annotations

import math
import pytest

from torque_test_utils import run_observation_loop, save_telemetry, assert_physics_yaw_rate

# Gust ends at SITL t~30 s (dynamics_t=15 + startup_hold=15).
# Settle = 48 s gives ~18 s recovery after gust end.
_SETTLE_S   = 48.0
_OBSERVE_S  = 20.0
_THRESHOLD  = math.radians(2.0)   # [rad/s] physics ground truth


@pytest.mark.parametrize("torque_armed_profile", ["gust"], indirect=True)
def test_gust_recovery(torque_armed_profile):
    """
    At dynamics_t=10 s the rotor hub spins at 120% nominal for 5 seconds
    (gust), then returns to nominal.  The higher RPM shifts the back-EMF
    equilibrium; the yaw rate PID must recover to |psi_dot| < 2 deg/s
    within ~18 s of gust end.

    Uses physics ground truth (mediator events log) rather than EKF yawspeed,
    which carries compass-tilt artefacts during rapid RPM changes.
    """
    ctx = torque_armed_profile

    _, rows = run_observation_loop(ctx, _SETTLE_S, _OBSERVE_S)

    save_telemetry(rows, "gust", ctx.log)
    assert_physics_yaw_rate(ctx.events_log, _THRESHOLD, _SETTLE_S, _OBSERVE_S, ctx.log)
