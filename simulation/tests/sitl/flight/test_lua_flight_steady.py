"""
test_lua_flight_steady.py — steady orbit under full ArduPilot + Lua control.

Validates that rawes.lua (SCR_USER6=1) can sustain a stable orbit with
internal_controller=False — ArduPilot + Lua own the physics entirely.

This is M3 Step 1: the foundational gate for pumping and landing tests.

Uses the acro_armed_lua_full fixture (stationary kinematic hold, vel0=[0,0,0]):
  - Hub holds at tether equilibrium for 80 s; no motion required.
  - With dual GPS (EK3_SRC1_YAW=2), yaw is known from the first RELPOSNED fix.
    delAngBiasLearned converges with constant-zero gyro at ~34 s after start.
    GPS fuses at ~34 s; _tdir0 fires; Lua orbit tracking active ~46 s before exit.
  - internal_controller=False (Lua RC overrides drive physics at 50 Hz)
  - SCR_USER6=1 set immediately in fixture (Lua pre-GPS bypass active until fusion).

No RC overrides are sent by this test — Lua owns Ch1/Ch2/Ch3 (cyclic +
collective) and Ch8 (motor interlock keepalive at 100 Hz).

Timing from mediator start (speedup=1):
  t=0..80 s   kinematic stationary hold at pos0 (vel=0)
  t~6 s       GPS first fix; EKF3 origin set
  t~12 s      arm complete; SCR_USER6=1 set; Lua pre-GPS bypass active
  t~12 s      AHRS healthy; Lua DCM capture fires
              "RAWES flight: captured" sent; bz_eq0 = disk_normal_ned()
  t~34 s      GPS fuses; "RAWES flight: GPS bz=(...)" sent; orbit tracking begins
  t~34 s      fixture yields (GPS fusion confirmed)
  t=80 s      kinematic exits; Lua has been tracking orbit for ~46 s
  t~80+       free flight under ArduPilot + Lua

Fixture yields at ~t=34 s (GPS fusion).  Test waits for kinematic to end
(t=80 s), then observes 120 s of free flight.

Pass criteria (all from physics telemetry CSV, not EKF estimates)
-----------------------------------------------------------------
1. "RAWES flight: captured" STATUSTEXT received.
2. Physics altitude >= 1.0 m throughout free flight (no crash).
3. Longest continuous physics altitude >= 3.0 m window >= 60 s.
4. Lua cyclic activity >= 50 PWM peak (orbit-tracking corrections non-trivial).
5. No EKF emergency yaw reset.
6. No CRITICAL errors in mediator log.

Note: criteria 2 and 3 use physics pos_z from telemetry.csv (ground truth),
not EKF LOCAL_POSITION_NED (which can lag physics crashes by tens of seconds).
"""
import logging
import sys
from pathlib import Path

import pytest

_SIM_DIR     = Path(__file__).resolve().parents[3]
_SITL_DIR    = Path(__file__).resolve().parents[1]
_ANALYSIS_DIR = _SIM_DIR / "analysis"
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))
sys.path.insert(0, str(_ANALYSIS_DIR))

from stack_infra import (
    StackContext, dump_startup_diagnostics,
    observe, assert_no_mediator_criticals, get_arducopter_crash_info,
)
from analyse_run import compute_steady_metrics, print_flight_report

# -- Timing -------------------------------------------------------------------
_KINEMATIC_TIMEOUT_S = 45.0   # s from fixture yield to kinematic end (fixture yields at GPS fusion ~44s, kinematic exits ~80s → 36s gap; 45s has margin)
_CAPTURE_TIMEOUT_S   = 30.0   # s from observation start -- Lua captures during kinematic
_OBS_SECONDS         = 120.0  # s of free-flight observation after kinematic ends

# -- Pass thresholds ----------------------------------------------------------
_MIN_ALT_M               = 3.0   # m -- hard floor (no crash; 1 m = sim floor, use 3 m)
_STABLE_ALT_M            = 5.0   # m -- floor for continuous-stable window
_MIN_STABLE_FLIGHT_S     = 60.0  # s -- required continuous sim-time above _STABLE_ALT_M
_MAX_ALT_M               = 40.0  # m -- hub must not escape to unrealistic altitude
_MAX_FLOOR_HITS          = 0     # floor hits at altitude <= 1.05 m
_MAX_TENSION_N           = 1000.0  # N -- tether must not spike (break load = 620 N)
_MIN_CYCLIC_ACTIVITY_PWM = 50    # |servo1-1500| + |servo2-1500| minimum peak

_POS_LOG_INTERVAL        = 5.0   # s between periodic log lines



def test_lua_flight_steady(acro_armed_lua_full: StackContext):
    """
    M3 Step 1: steady orbit under full ArduPilot + Lua control.

    Asserts that the hub sustains stable orbital flight for >= 60 s with
    internal_controller=False and Lua driving cyclic + collective at 50 Hz.
    Pass/fail is determined from physics telemetry (pos_z), not EKF altitude.
    """
    ctx = acro_armed_lua_full
    gcs = ctx.gcs
    log = logging.getLogger("test_lua_flight_steady")

    log.info("=== test_lua_flight_steady: waiting for kinematic phase to end ===")
    log.info("(SCR_USER6=1 active from fixture; GPS capture fires during kinematic ~23 s)")

    if not ctx.wait_kinematic_done(timeout=_KINEMATIC_TIMEOUT_S):
        pytest.fail(
            f"Kinematic phase did not end within {_KINEMATIC_TIMEOUT_S:.0f} s.\n"
            "Check mediator log for 'TRANSITION kinematic->free-flight'."
        )
    log.info("Kinematic phase ended -- Lua orbit tracking already active")

    all_statustext = ctx.all_statustext
    lua_captured   = False

    # all_statustext already includes everything drained during wait_kinematic_done()
    for _prior in all_statustext:
        if "rawes flight" in _prior.lower() and "captured" in _prior.lower():
            lua_captured = True
            ctx.flight_events["Lua captured"] = -1.0
            log.info("Lua DCM capture (in kinematic): %s", _prior)
            break

    # -- Observation loop: liveness + servo activity + STATUSTEXT --------------
    max_cyclic_activity = 0
    ekf_yaw_reset       = False

    t_obs_start        = gcs.sim_now()
    deadline           = t_obs_start + _OBS_SECONDS
    t_last_log         = t_obs_start
    t_capture_deadline = t_obs_start + _CAPTURE_TIMEOUT_S

    log.info("=== test_lua_flight_steady: observing %.0f s of free flight ===", _OBS_SECONDS)

    state = {
        "lua_captured":      lua_captured,
        "ekf_yaw_reset":     ekf_yaw_reset,
        "max_cyclic":        max_cyclic_activity,
        "t_last_log":        t_obs_start,
    }

    def _handle(msg, t_rel):
        now = gcs.sim_now()
        if msg is not None:
            mt = msg.get_type()
            if mt == "SERVO_OUTPUT_RAW":
                activity = abs(msg.servo1_raw - 1500) + abs(msg.servo2_raw - 1500)
                if activity > state["max_cyclic"]:
                    state["max_cyclic"] = activity
            elif mt == "STATUSTEXT":
                text = msg.text.rstrip("\x00").strip()
                log.info("STATUSTEXT [t=%.1fs]: %s", t_rel, text)
                all_statustext.append(text)
                tl = text.lower()
                if "rawes flight" in tl and "captured" in tl:
                    state["lua_captured"] = True
                    ctx.flight_events["Lua captured"] = t_rel
                if "emergency yaw" in tl or "yaw reset" in tl:
                    state["ekf_yaw_reset"] = True
                    log.warning("EKF yaw reset at t=%.1fs: %s", t_rel, text)

        if not state["lua_captured"] and now > t_capture_deadline:
            pytest.fail(
                f"rawes.lua did not capture within {_CAPTURE_TIMEOUT_S:.0f} s.\n"
                f"STATUSTEXT: {all_statustext[-20:]}\n"
                "Checklist:\n"
                "  * SCR_ENABLE=1 baked into eeprom (prime_eeprom=True in fixture)\n"
                "  * rawes.lua installed to /ardupilot/scripts/\n"
                "  * SCR_USER5 (anchor Down) = home_alt_m correct\n"
                "  * SITL log for Lua load errors"
            )
        if now - state["t_last_log"] >= _POS_LOG_INTERVAL:
            log.info("  t=%.0fs  max_activity=%d PWM  captured=%s  yaw_reset=%s",
                     t_rel, state["max_cyclic"], state["lua_captured"], state["ekf_yaw_reset"])
            state["t_last_log"] = now
        return None

    observe(ctx, _OBS_SECONDS, _handle,
            msg_types=["SERVO_OUTPUT_RAW", "STATUSTEXT", "ATTITUDE",
                       "LOCAL_POSITION_NED", "EKF_STATUS_REPORT"],
            label="observation")

    lua_captured      = state["lua_captured"]
    ekf_yaw_reset     = state["ekf_yaw_reset"]
    max_cyclic_activity = state["max_cyclic"]
    log.info("Observation complete: max_activity=%d PWM", max_cyclic_activity)

    try:
        # -- Physics-based pass/fail from telemetry CSV -----------------------
        # print_steady_report also loads telemetry, but we load separately here
        # so we can assert before printing the full report.
        metrics = None
        if ctx.telemetry_log.exists():
            from telemetry_csv import read_csv as _read_csv
            _rows   = _read_csv(ctx.telemetry_log)
            metrics = compute_steady_metrics(_rows, stable_alt_m=_STABLE_ALT_M)
            log.info(
                "Physics: min_alt=%.2f m  stable=%.0f s  floor_hits=%d"
                "  mean_alt=%.1f m  max_tension=%.0f N",
                metrics.min_phys_alt, metrics.max_stable_s, metrics.floor_hits,
                metrics.mean_alt, metrics.max_tension,
            )
        else:
            log.warning("No telemetry CSV -- physics checks skipped")

        # -- Assertions -------------------------------------------------------
        assert lua_captured, (
            "rawes.lua never captured equilibrium.\n"
            f"STATUSTEXT: {all_statustext}"
        )

        if metrics is not None:
            assert metrics.floor_hits <= _MAX_FLOOR_HITS, (
                f"Hub hit floor: {metrics.floor_hits} frames at alt<=1.05 m"
                f" (max allowed={_MAX_FLOOR_HITS})\n"
                f"min_alt={metrics.min_phys_alt:.2f} m  mean_alt={metrics.mean_alt:.1f} m\n"
                f"STATUSTEXT: {all_statustext}"
            )
            assert metrics.min_phys_alt >= _MIN_ALT_M, (
                f"Hub crashed: min physics alt {metrics.min_phys_alt:.2f} m"
                f" < {_MIN_ALT_M:.1f} m\n"
                f"STATUSTEXT: {all_statustext}"
            )
            assert metrics.mean_alt <= _MAX_ALT_M, (
                f"Hub escaped to unrealistic altitude: mean_alt={metrics.mean_alt:.1f} m"
                f" > {_MAX_ALT_M:.0f} m (expected ~14 m orbital altitude)\n"
                f"STATUSTEXT: {all_statustext}"
            )
            assert metrics.max_tension <= _MAX_TENSION_N, (
                f"Tether tension spike: {metrics.max_tension:.0f} N"
                f" > {_MAX_TENSION_N:.0f} N (break load=620 N)\n"
                f"STATUSTEXT: {all_statustext}"
            )
            assert metrics.max_stable_s >= _MIN_STABLE_FLIGHT_S, (
                f"Hub only stable for {metrics.max_stable_s:.0f} s above"
                f" {_STABLE_ALT_M} m (need {_MIN_STABLE_FLIGHT_S:.0f} s).\n"
                f"mean_alt={metrics.mean_alt:.1f} m  floor_hits={metrics.floor_hits}\n"
                f"EKF yaw reset: {ekf_yaw_reset}\n"
                f"STATUSTEXT: {all_statustext}"
            )

        assert max_cyclic_activity >= _MIN_CYCLIC_ACTIVITY_PWM, (
            f"Lua cyclic activity too low: {max_cyclic_activity} PWM "
            f"< {_MIN_CYCLIC_ACTIVITY_PWM}\n"
            f"STATUSTEXT: {all_statustext}"
        )

        assert not ekf_yaw_reset, (
            "EKF emergency yaw reset -- compass/velocity inconsistency.\n"
            f"STATUSTEXT: {all_statustext}"
        )

        assert_no_mediator_criticals(ctx.mediator_log)

        stable_s = metrics.max_stable_s if metrics is not None else 0.0
        log.info(
            "=== test_lua_flight_steady PASSED "
            "(stable=%.0fs  max_activity=%d PWM) ===",
            stable_s, max_cyclic_activity,
        )

        # -- Full physics report ----------------------------------------------
        _log_dir = ctx.telemetry_log.parent
        if _log_dir.exists():
            print_flight_report(_log_dir)

    except Exception as exc:
        dump_startup_diagnostics(ctx)
        crash = get_arducopter_crash_info(ctx)
        if crash:
            raise type(exc)(f"{exc}{crash}") from exc
        raise
