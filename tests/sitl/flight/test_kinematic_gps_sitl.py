"""
test_kinematic_gps_sitl.py -- GPS fusion during kinematic startup with dual GPS.

With EK3_SRC1_YAW=2 (dual-antenna RELPOSNED) yaw is known from the first GPS
fix — no vehicle motion needed.  The kinematic is a stationary hold at the
tether equilibrium (vel0=[0,0,0]).

Expected timeline (dual GPS, stationary hold):
  t~0.3 s  yaw aligned (RELPOSNED available immediately)
  t~8 s    arm complete
  t~34 s   delAngBiasLearned; GPS fuses ("EKF3 IMU0 is using GPS")
  t~34-74  40 s EKF validation window

Run:
    bash test.sh stack -n 1 -k test_kinematic_gps_sitl
"""
from __future__ import annotations

import sys


from tests.sitl.stack_infra import _acro_stack

from analysis.analyse_run import validate_ekf_window

import pytest
pytestmark = pytest.mark.sitl


def test_kinematic_gps_sitl(tmp_path, request):
    """
    GPS fuses during a stationary kinematic hold; EKF stays clean for 40 s.

    With dual GPS (EK3_SRC1_YAW=2, default in rawes_sitl_defaults.parm):
      - Yaw aligns at ~0.3 s from RELPOSNED — no motion needed.
      - delAngBiasLearned converges at ~34 s with constant-zero gyro.
      - GPS fuses at ~34 s; validate_ekf_window checks 40 s from that point.

    The kinematic is a stationary hold at the tether equilibrium (vel0=[0,0,0]).
    startup_damp=160 gives ~126 s after GPS fusion before free-flight exit.
    """
    extra = {
        # Stationary hold at pos0 (tether equilibrium).
        "vel0":                 [0.0, 0.0, 0.0],
        "kinematic_vel_ramp_s": 0.0,
        "startup_damp_seconds": 160.0,
    }

    with _acro_stack(
        tmp_path,
        test_name    = request.node.name,
        extra_config = extra,
    ) as ctx:
        log = ctx.log
        log.info("Dual GPS kinematic: stationary hold at equilibrium")
        log.info("Waiting up to 90 s for GPS to fuse ...")

        seen: list[str] = []
        t_gps_fused_s: list[float] = []

        def _gps_fused(text: str | None) -> bool:
            if text is not None:
                seen.append(text)
            if text is not None and "is using GPS" in text:
                t_gps_fused_s.append(ctx.gcs.sim_now())
                return True
            return False

        ctx.wait_drain(
            until       = _gps_fused,
            timeout     = 50.0,
            drain_s     = 2.0,
            check_procs = True,
            label       = "gps-fuse",
        )

        gps_origin = any("origin set"   in s for s in seen)
        gps_fused  = any("is using GPS" in s for s in seen)

        log.info("GPS origin : %s", gps_origin)
        log.info("GPS fused  : %s", gps_fused)

        assert gps_origin, "GPS origin never set within 50 s"
        assert gps_fused,  "GPS never fused within 50 s"

        t_fused = t_gps_fused_s[0]
        t_end   = t_fused + 40.0
        log.info("GPS fused at t=%.1f s -- validating EKF for 40 s (until t=%.1f s) ...",
                 t_fused, t_end)

        # Drain until the 40 s validation window has elapsed.
        ctx.wait_drain(
            timeout     = 42.0,    # 40 s window + 2 s buffer
            check_procs = True,
            label       = "ekf-window",
        )

        # Validate EKF was clean from GPS fusion to fusion+40 s.
        # validate_ekf_window reads the already-written MAVLink log.
        issues = validate_ekf_window(ctx.mavlink_log, t_fused, t_end)
        for issue in issues:
            log.warning("EKF issue: %s", issue)
        assert not issues, (
            "EKF problems in 40 s window after GPS fusion (t=%.1f..%.1f s):\n%s"
            % (t_fused, t_end, "\n".join(issues))
        )
        log.info("EKF validation PASSED (%.0f s clean after GPS fusion)", t_end - t_fused)
