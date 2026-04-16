"""
test_kinematic_gps.py -- GPS fusion during kinematic startup.

Tests that GPS fuses reliably during the fast-circle kinematic trajectory
used by test_lua_flight_steady (acro_armed_lua_full fixture):

  kinematic_traj_type = fast_circle
  r_circle  = 5 m,  v_fast = 5.0 m/s  (omega ~ 57 deg/s during accel circle)
  n_fast    = 0  (no constant-speed circles)
  phases    = hold -> accel circle -> decel circle -> large orbit lead-in

This is a strict subset of test_lua_flight_steady: same kinematic config,
same startup_damp_seconds=80.  The test runs the full kinematic trajectory
(including post-fusion lead-in) and then validates EKF health across the
entire window from GPS fusion to kinematic exit.

Run:
  bash simulation/dev.sh test-stack-parallel --fresh -n 1 -k test_kinematic_gps
"""
from __future__ import annotations

import sys
from pathlib import Path

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import _acro_stack

# analyse_mavlink is in simulation/analysis/ — add to path
sys.path.insert(0, str(_SIM_DIR / "analysis"))
from analyse_mavlink import validate_ekf_window


def test_kinematic_gps(tmp_path, request):
    """
    GPS fuses during fast-circle kinematic, then EKF stays clean until kinematic exit.

    Phases (sim seconds):
      t=0-15 s    stationary hold; EKF tilt aligns, GPS detects, vehicle arms
      t=15-27.6 s accel circle: 0 -> 5 m/s over 2pi (57 deg/s heading rate)
      t=27.6-38 s decel circle: 5 -> 0.96 m/s
      t=38-48 s   large orbit lead-in (T_lead=10 s)

    GPS fusion timeline:
      t~7.5 s  GPS first fix (during hold)
      t~16 s   GPS origin set (during hold)
      t~26 s   EKFGSF converges: yawAlignComplete (~10 s of 57 deg/s rotation)
      t~32 s   delAngBiasLearned; GPS fuses

    After GPS fuses the test waits for kinematic exit, then calls
    validate_ekf_window(mavlink_log, t_gps_fused, t_kinematic_exit) to confirm
    no GPS glitch, const_pos re-entry, attitude loss, or EKF yaw reset occurred
    in the window where Lua orbit tracking is active.
    """
    extra = {
        "kinematic_traj_type":     "fast_circle",
        "kinematic_orbital_dir":   1,
        "kinematic_hold_s":        15.0,
        "kinematic_fast_speed":    5.0,
        "kinematic_circle_radius": 5.0,
        "kinematic_fast_circles":  1,
        "kinematic_orbit_lead_s":  10.0,
        "kinematic_exit_speed":    0.96,
        "startup_damp_seconds":    80.0,
    }

    with _acro_stack(
        tmp_path,
        test_name    = request.node.name,
        extra_config = extra,
    ) as ctx:
        log = ctx.log
        log.info("Fast-circle kinematic: r=5 m, v_fast=5.0 m/s, n_fast=0")
        log.info("Waiting up to 70 sim-s for GPS to fuse ...")

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
            timeout     = 70.0,
            drain_s     = 2.0,
            check_procs = True,
            label       = "gps-fuse",
        )

        gps_origin = any("origin set"  in s for s in seen)
        gps_fused  = any("is using GPS" in s for s in seen)

        log.info("GPS origin : %s", gps_origin)
        log.info("GPS fused  : %s", gps_fused)

        assert gps_origin, "GPS origin never set within 70 sim-s"
        assert gps_fused,  "GPS never fused within 70 sim-s of kinematic"

        # Wait for the full kinematic trajectory to complete (large orbit lead-in)
        log.info("GPS fused at t=%.1f s -- waiting for kinematic exit ...", t_gps_fused_s[0])
        exited = ctx.wait_kinematic_done(timeout=60.0)
        assert exited, "kinematic did not exit within 60 s after GPS fusion"

        exit_ev = ctx.events_log.last_event("kinematic_exit")
        assert exit_ev is not None, "kinematic_exit event not in events log"
        t_exit_s = float(exit_ev["t_sim"])
        log.info("Kinematic exit at t=%.1f s", t_exit_s)

        # Stop the mediator before validation — free flight with no controller
        # causes tether tension runaway that crashes SITL.  validate_ekf_window
        # only reads the already-written MAVLink log, so the mediator is not needed.
        log.info("Kinematic exit confirmed — stopping mediator before EKF validation.")
        if ctx.mediator_proc is not None:
            ctx.mediator_proc.terminate()

        # Validate EKF was clean from GPS fusion to kinematic exit
        issues = validate_ekf_window(ctx.mavlink_log, t_gps_fused_s[0], t_exit_s)
        for issue in issues:
            log.warning("EKF issue: %s", issue)
        assert not issues, (
            "EKF problems between GPS fusion (t=%.1f s) and kinematic exit (t=%.1f s):\n%s"
            % (t_gps_fused_s[0], t_exit_s, "\n".join(issues))
        )
