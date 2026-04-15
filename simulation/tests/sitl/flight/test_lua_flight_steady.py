"""
test_lua_flight_steady.py — steady orbit under full ArduPilot + Lua control.

Validates that rawes.lua (SCR_USER6=1) can sustain a stable orbit with
internal_controller=False — ArduPilot + Lua own the physics entirely.

This is M3 Step 1: the foundational gate for pumping and landing tests.

Uses the acro_armed_lua_full fixture (linear kinematic, vel0=East):
  - kinematic_traj_type=linear, vel0=[0, 0.96, 0]: hub moves East at 0.96 m/s
    for 65 s.  GPS fuses at t~23 s (East GPS velocity heading = stable EKF yaw).
    GPS tether recapture fires at t~23 s; orbit tracking starts then.
  - internal_controller=False (Lua RC overrides drive physics at 50 Hz)
  - SCR_USER6=1 set immediately in fixture (Lua active from t~15 s).
  - COL_CRUISE_FLIGHT_RAD=-0.18 rad (matches stack_coll_eq from simtest)

No RC overrides are sent by this test — Lua owns Ch1/Ch2/Ch3 (cyclic +
collective) and Ch8 (motor interlock keepalive at 100 Hz).

Timing from mediator start (speedup=1):
  t=0..60 s   kinematic constant-velocity phase (East at 0.96 m/s)
  t=60..65 s  kinematic ramp-to-zero (vel_ramp_s=5)
  t~15 s      AHRS healthy; Lua DCM capture fires
              "RAWES flight: captured" sent; bz_eq0 = disk_normal_ned()
  t~21 s      EKF3 origin set (GPS first fix)
  t~23 s      GPS fuses; GPS tether recapture fires
              "RAWES flight: GPS bz=(...)" sent; orbit tracking begins
  t=65 s      kinematic exits -- ctx.wait_kinematic_done() returns
  t=65+       free flight under ArduPilot + Lua; Lua already tracking orbit

Fixture yields at ~t=15 s.  Test waits for kinematic end, then drains
buffered STATUSTEXTs and observes 120 s of free flight.

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
import time
from pathlib import Path

import pytest

_SIM_DIR     = Path(__file__).resolve().parents[3]
_SITL_DIR    = Path(__file__).resolve().parents[1]
_ANALYSIS_DIR = _SIM_DIR / "analysis"
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))
sys.path.insert(0, str(_ANALYSIS_DIR))

from stack_infra import StackContext, dump_startup_diagnostics
from analyse_run import compute_steady_metrics, print_flight_report, parse_arducopter, RunReport

# -- Timing -------------------------------------------------------------------
_KINEMATIC_TIMEOUT_S = 90.0   # s to wait for kinematic->free-flight transition
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


def _get_crash_info(ctx: "StackContext") -> str:
    """Return a formatted crash block from arducopter.log, or empty string."""
    acp = ctx.telemetry_log.parent / "arducopter.log"
    if not acp.exists():
        return ""
    rpt = RunReport()
    parse_arducopter(acp, rpt)
    if rpt.sitl_crash is None:
        return ""
    lines = [
        "",
        "--- ArduPilot crash ---",
        f"  {rpt.sitl_crash.error_line}",
    ]
    for frame in rpt.sitl_crash.stack:
        lines.append(f"  {frame}")
    return "\n".join(lines)


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

    t_obs_start        = time.monotonic()
    deadline           = t_obs_start + _OBS_SECONDS
    t_last_log         = t_obs_start
    t_capture_deadline = t_obs_start + _CAPTURE_TIMEOUT_S

    log.info("=== test_lua_flight_steady: observing %.0f s of free flight ===", _OBS_SECONDS)

    try:
        while time.monotonic() < deadline:
            for name, proc, lp in [
                ("mediator", ctx.mediator_proc, ctx.mediator_log),
                ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
            ]:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    crash = _get_crash_info(ctx)
                    pytest.fail(
                        f"{name} exited during observation "
                        f"(rc={proc.returncode}):\n{txt[-3000:]}"
                        f"{crash}"
                    )

            msg = gcs._recv(
                type=["SERVO_OUTPUT_RAW", "STATUSTEXT", "ATTITUDE",
                      "LOCAL_POSITION_NED", "EKF_STATUS_REPORT"],
                blocking=True, timeout=0.2,
            )
            t_rel = time.monotonic() - t_obs_start
            now   = time.monotonic()

            if msg is not None:
                mt = msg.get_type()
                if mt == "SERVO_OUTPUT_RAW":
                    ch1      = msg.servo1_raw
                    ch2      = msg.servo2_raw
                    activity = abs(ch1 - 1500) + abs(ch2 - 1500)
                    if activity > max_cyclic_activity:
                        max_cyclic_activity = activity
                elif mt == "STATUSTEXT":
                    text = msg.text.rstrip("\x00").strip()
                    log.info("STATUSTEXT [t=%.1fs]: %s", t_rel, text)
                    all_statustext.append(text)
                    tl = text.lower()
                    if "rawes flight" in tl and "captured" in tl:
                        lua_captured = True
                        ctx.flight_events["Lua captured"] = t_rel
                    if "emergency yaw" in tl or "yaw reset" in tl:
                        ekf_yaw_reset = True
                        log.warning("EKF yaw reset at t=%.1fs: %s", t_rel, text)

            if not lua_captured and now > t_capture_deadline:
                pytest.fail(
                    f"rawes.lua did not capture within {_CAPTURE_TIMEOUT_S:.0f} s.\n"
                    f"STATUSTEXT: {all_statustext[-20:]}\n"
                    "Checklist:\n"
                    "  * SCR_ENABLE=1 baked into eeprom (prime_eeprom=True in fixture)\n"
                    "  * rawes.lua installed to /ardupilot/scripts/\n"
                    "  * SCR_USER5 (anchor Down) = home_alt_m correct\n"
                    "  * SITL log for Lua load errors"
                )

            if now - t_last_log >= _POS_LOG_INTERVAL:
                log.info(
                    "  t=%.0fs  max_activity=%d PWM  captured=%s  yaw_reset=%s",
                    t_rel, max_cyclic_activity, lua_captured, ekf_yaw_reset,
                )
                t_last_log = now

        log.info("Observation complete: max_activity=%d PWM", max_cyclic_activity)

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

        if ctx.mediator_log.exists():
            med_text = ctx.mediator_log.read_text(encoding="utf-8", errors="replace")
            critical = [ln for ln in med_text.splitlines() if "CRITICAL" in ln]
            assert not critical, (
                "CRITICAL errors in mediator:\n" + "\n".join(critical[:10])
            )

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
        crash = _get_crash_info(ctx)
        if crash:
            raise type(exc)(f"{exc}{crash}") from exc
        raise
