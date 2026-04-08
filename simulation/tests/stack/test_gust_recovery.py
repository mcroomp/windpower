"""
torque/test_gust_recovery.py — Gust shock and yaw recovery test.

Profile: axle speed is nominal until dynamics_t=10 s, then a 5-second
gust drives it to 150% of nominal, then it returns to nominal.

Timeline (approximate test time):
  t =  0–10 s  : EKF bias settles (from 10 s startup hold)
  t = 10–15 s  : gust hits  — omega_rotor suddenly ×1.5
  t = 15–20 s  : gust ends  — omega_rotor returns to nominal
  t = 40 s+    : observation window starts (25 s recovery time)

Pass criterion: max |ψ_dot| < 2°/s in the observation window, confirming
that the PID recovered from the bearing-drag spike within 25 s of gust end.

Telemetry → simulation/logs/torque_telemetry_gust.json
"""
from __future__ import annotations

import pytest

from torque_telemetry import TorqueTelemetryRecorder
from torque_test_utils  import run_observation_loop, save_telemetry, assert_yaw_rate

# Gust ends at dynamics_t≈15 s ≈ test t≈25 s.  Settle = 40 s gives ~15 s recovery.
_SETTLE_S   = 40.0
_OBSERVE_S  = 20.0
_THRESHOLD  = 2.0     # °/s — should be recovered and regulated by settle time


@pytest.mark.parametrize("torque_armed_profile", ["gust"], indirect=True)
def test_gust_recovery(torque_armed_profile):
    """
    At dynamics_t=10 s the rotor hub suddenly spins at 150% nominal for 5 seconds
    (gust), then returns to nominal.  The yaw rate PID must reject the bearing-drag
    shock and recover to |ψ_dot| < 2°/s within 25 s of gust end.
    """
    ctx = torque_armed_profile
    rec = TorqueTelemetryRecorder(meta={
        "test":               "gust_recovery",
        "profile":            "gust",
        "omega_rotor_rads":   ctx.omega_rotor,
        "gust_start_dyn_t":   10.0,
        "gust_end_dyn_t":     15.0,
        "gust_multiplier":    1.20,
        "settle_s":           _SETTLE_S,
        "observe_s":          _OBSERVE_S,
        "threshold_degs":     _THRESHOLD,
    })

    obs = run_observation_loop(
        ctx=ctx, rec=rec,
        settle_s=_SETTLE_S, observe_s=_OBSERVE_S,
        timeout_s=_SETTLE_S + _OBSERVE_S + 20.0,
    )

    save_telemetry(rec, "gust", ctx.log)
    assert_yaw_rate(obs, _THRESHOLD, _SETTLE_S, rec, ctx.log)
