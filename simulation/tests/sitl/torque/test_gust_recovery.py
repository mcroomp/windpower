"""
torque/test_gust_recovery.py — Gust shock and yaw recovery test.

Profile: axle speed is nominal until dynamics_t=10 s, then a 5-second
gust drives it to 150% of nominal, then it returns to nominal.

Timeline (approximate test time):
  t =  0–10 s  : EKF bias settles (from 10 s startup hold)
  t = 10–15 s  : gust hits  — omega_rotor suddenly x1.5
  t = 15–20 s  : gust ends  — omega_rotor returns to nominal
  t = 40 s+    : observation window starts (25 s recovery time)

Pass criterion: max |psi_dot| < 2 deg/s in the observation window, confirming
that the PID recovered from the RPM-mismatch transient within 25 s of gust end.

Telemetry -> simulation/logs/torque_telemetry_gust.csv
"""
from __future__ import annotations

import math
import pytest

from torque_test_utils  import run_observation_loop, save_telemetry, assert_yaw_rate

# Gust ends at SITL t~30 s (dynamics_t=15 + startup_hold=15).  Arm at ~t=7 s.
# Settle = 48 s from arm gives ~25 s recovery after gust end (t_rel 48 → SITL 55).
_SETTLE_S   = 48.0
_OBSERVE_S  = 20.0
_THRESHOLD  = math.radians(2.0)   # [rad/s] -- should be recovered and regulated by settle time


@pytest.mark.parametrize("torque_armed_profile", ["gust"], indirect=True)
def test_gust_recovery(torque_armed_profile):
    """
    At dynamics_t=10 s the rotor hub suddenly spins at 150% nominal for 5 seconds
    (gust), then returns to nominal.  The higher RPM shifts the back-EMF equilibrium;
    the adaptive trim and yaw rate PID must track the change and recover to
    |psi_dot| < 2 deg/s within 25 s of gust end.
    """
    ctx = torque_armed_profile

    obs, rows = run_observation_loop(ctx, _SETTLE_S, _OBSERVE_S)

    save_telemetry(rows, "gust", ctx.log)
    assert_yaw_rate(obs, _THRESHOLD, _SETTLE_S, ctx.log)
