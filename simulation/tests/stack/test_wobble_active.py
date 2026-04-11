"""
torque/test_wobble_active.py — High-tilt wobble with active yaw PID.

Same +-20/+-15 degree wobble as test_wobble but with ATC_RAT_YAW_P=0.05
so the motor visibly pulses with the tilt cross-coupling (~150 us swing).

This is for visualisation purposes -- it demonstrates the motor actively
fighting the wobble-induced gyro Z contamination rather than holding steady.

On hardware, the actual P gain will be tuned from measured k_bearing.

NOTE: This test is marked xfail because P=0.05 is a placeholder demo gain
that overcorrects in SITL (oscillates at ~22 deg/s vs 5 deg/s limit).
The real P gain on hardware will be much smaller (tuned from k_bearing).
"""
from __future__ import annotations

import math
import pytest

from torque_test_utils  import run_observation_loop, save_telemetry, assert_yaw_rate

_SETTLE_S   = 40.0
_OBSERVE_S  = 20.0
_THRESHOLD  = math.radians(5.0)   # [rad/s]


@pytest.mark.xfail(
    strict=False,
    reason="P=0.05 is a demo gain for hardware tuning visualisation; overcorrects in SITL",
)
@pytest.mark.parametrize("torque_armed_profile", ["wobble"], indirect=True)
def test_wobble_active(torque_armed_profile):
    """Wobble with P=0.05 -- motor visibly pulses at wobble frequency."""
    ctx = torque_armed_profile

    # Override to higher P so motor variation is visible in the visualiser
    ctx.gcs.set_param("ATC_RAT_YAW_P", 0.05, timeout=3.0)
    ctx.log.info("ATC_RAT_YAW_P set to 0.05 (active PID demo)")

    rows: list = []

    for pname, pval in [
        ("EK3_SRC1_POSXY", 0),
        ("EK3_SRC1_VELXY", 0),
    ]:
        ctx.gcs.set_param(pname, pval, timeout=3.0)

    obs = run_observation_loop(
        ctx=ctx, rows=rows,
        settle_s=_SETTLE_S, observe_s=_OBSERVE_S,
        timeout_s=_SETTLE_S + _OBSERVE_S + 20.0,
    )

    save_telemetry(rows, "wobble_active", ctx.log)
    assert_yaw_rate(obs, _THRESHOLD, _SETTLE_S, ctx.log)
