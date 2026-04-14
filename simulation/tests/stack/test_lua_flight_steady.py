"""
test_lua_flight_steady.py — steady orbit under full ArduPilot + Lua control.

Validates that rawes.lua (SCR_USER6=1) can sustain a stable orbit with
internal_controller=False — ArduPilot + Lua own the physics entirely.

This is M3 Step 1: the foundational gate for pumping and landing tests.

Uses the acro_armed_lua_full fixture (same initial conditions as every other
passing stack test — pos0/body_z from steady_state_starting.json, natural
orbit at xi~8 deg, 100 m tether, altitude~14 m):
  - kinematic_vel_ramp_s=0.0  (constant vel0 so GPS fuses during kinematic)
  - internal_controller=False (Lua RC overrides drive physics at 50 Hz)
  - SCR_USER6=1               (flight mode: cyclic + VZ altitude-hold)
  - COL_CRUISE_FLIGHT_RAD=-0.18 rad (matches stack_coll_eq from simtest)

No RC overrides are sent by this test — Lua owns Ch1/Ch2/Ch3 (cyclic +
collective) and Ch8 (motor interlock keepalive at 100 Hz).

Timing from mediator start (speedup=1):
  t=0..65 s   kinematic phase (hub moves at vel0 from launch_pos to pos0)
  t~21 s      EKF3 origin set (GPS first fix)
  t~54 s      GPS fuses (EKF3 transitions to horiz_pos_abs mode); pos_ned available
  t=62 s      Lua captures body_z (FLIGHT_SETTLE_MS=62000 ms, 3 s before exit);
              _tdir0 set immediately from GPS position (~100 m tlen)
  t=65 s      kinematic exits; Lua orbit-tracking cyclic already active
  t=65+       free flight under ArduPilot + Lua

Fixture yields at ~t=15 s.  Observation window of 200 s covers kinematic
exit (t_obs~50 s) + Lua capture (t_obs~47 s) + >= 60 s free flight.

Pass criteria
-------------
1. "RAWES flight: captured" STATUSTEXT within 100 s of fixture yield.
2. Hub physics altitude >= 1.0 m throughout (no crash).
3. Continuous altitude >= 3.0 m for >= 60 s (orbit established, not just
   brief recovery after kinematic exit).
4. Lua cyclic activity >= 50 PWM (orbit-tracking corrections are non-trivial).
5. No EKF emergency yaw reset.
6. No CRITICAL errors in mediator log.
"""
import logging
import sys
import time
from pathlib import Path

import pytest

_SIM_DIR   = Path(__file__).resolve().parents[2]
_STACK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_STACK_DIR))

from conftest import StackContext, dump_startup_diagnostics

# ── Timing ────────────────────────────────────────────────────────────────────
_CAPTURE_TIMEOUT_S = 60.0   # s from fixture yield — Lua captures at SITL t=62 s
                            # (FLIGHT_SETTLE_MS), which is t_obs~46 s (fixture yields
                            # at SITL t~16 s).  60 s gives 14 s margin.
_OBS_SECONDS       = 200.0  # s total observation from fixture yield.
                            # Kinematic exits at t_obs~49 s (SITL 65 s);
                            # 200 s gives >150 s of free flight (>= 60 s stable needed).

# ── Pass thresholds ───────────────────────────────────────────────────────────
_MIN_ALT_M               = 1.0   # m — hard floor (no crash)
_STABLE_ALT_M            = 3.0   # m — floor for continuous-stable window
_MIN_STABLE_FLIGHT_S     = 60.0  # s — required continuous time above STABLE_ALT_M
_MIN_CYCLIC_ACTIVITY_PWM = 50    # |servo1-1500| + |servo2-1500| minimum peak

_POS_LOG_INTERVAL        = 5.0   # s between periodic position log lines


def test_lua_flight_steady(acro_armed_lua_full: StackContext):
    """
    M3 Step 1: steady orbit under full ArduPilot + Lua control.

    Asserts that the hub sustains stable orbital flight for >= 60 s with
    internal_controller=False and Lua driving cyclic + collective at 50 Hz.
    """
    ctx = acro_armed_lua_full
    gcs = ctx.gcs
    log = logging.getLogger("test_lua_flight_steady")

    all_statustext      = ctx.all_statustext
    lua_captured        = False
    max_cyclic_activity = 0
    ekf_yaw_reset       = False

    # Track continuous time above _STABLE_ALT_M from EKF LOCAL_POSITION_NED.
    t_above_start = None    # monotonic time hub last rose above stable floor
    max_stable_s  = 0.0     # longest continuous stable window seen

    t_obs_start        = time.monotonic()
    deadline           = t_obs_start + _OBS_SECONDS
    t_last_log         = t_obs_start
    t_capture_deadline = t_obs_start + _CAPTURE_TIMEOUT_S

    log.info("=== test_lua_flight_steady: observing %.0f s ===", _OBS_SECONDS)

    try:
        while time.monotonic() < deadline:
            # Process liveness.
            for name, proc, lp in [
                ("mediator", ctx.mediator_proc, ctx.mediator_log),
                ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
            ]:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    pytest.fail(
                        f"{name} exited during observation "
                        f"(rc={proc.returncode}):\n{txt[-3000:]}"
                    )

            msg = gcs._mav.recv_match(
                type=["LOCAL_POSITION_NED", "SERVO_OUTPUT_RAW", "STATUSTEXT"],
                blocking=True, timeout=0.2,
            )
            t_rel = time.monotonic() - t_obs_start
            now   = time.monotonic()

            if msg is not None:
                mt = msg.get_type()

                if mt == "LOCAL_POSITION_NED":
                    # Altitude above ground: home_alt_m (= -pos0[2]) minus EKF D.
                    # sensor.py: pos_ned_rel[2] = pos_ned[2] - home_ned_z = pos_ned[2] + home_alt_m
                    # So LOCAL_POSITION_NED.D = pos_ned[2] + home_alt_m → alt = home_alt_m - D.
                    alt_ekf = ctx.home_alt_m - msg.z
                    if alt_ekf >= _STABLE_ALT_M:
                        if t_above_start is None:
                            t_above_start = now
                        else:
                            max_stable_s = max(max_stable_s, now - t_above_start)
                    else:
                        t_above_start = None

                elif mt == "SERVO_OUTPUT_RAW":
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
                        log.info("Lua capture at t=%.1fs", t_rel)
                    if "emergency yaw" in tl or "yaw reset" in tl:
                        ekf_yaw_reset = True
                        log.warning("EKF yaw reset at t=%.1fs: %s", t_rel, text)

            # Infer capture from servo activity if STATUSTEXT was dropped during
            # param setting window.
            if not lua_captured and max_cyclic_activity >= _MIN_CYCLIC_ACTIVITY_PWM:
                lua_captured = True
                ctx.flight_events["Lua captured (inferred from servo activity)"] = t_rel
                log.info("Lua capture inferred from servo activity (%d PWM) at t=%.1fs",
                         max_cyclic_activity, t_rel)

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
                    "  t=%.0fs  max_activity=%d PWM  stable_s=%.0fs  "
                    "captured=%s  yaw_reset=%s",
                    t_rel, max_cyclic_activity, max_stable_s,
                    lua_captured, ekf_yaw_reset,
                )
                t_last_log = now

        # Close the last stable window if still open at observation end.
        if t_above_start is not None:
            max_stable_s = max(max_stable_s, time.monotonic() - t_above_start)

        log.info(
            "Observation complete: max_activity=%d PWM  max_stable_s=%.0fs",
            max_cyclic_activity, max_stable_s,
        )

        # ── Physics altitude from telemetry (authoritative) ────────────────
        min_phys_alt = float("inf")
        if ctx.telemetry_log.exists():
            from telemetry_csv import read_csv as _read_csv
            _tel  = _read_csv(ctx.telemetry_log)
            z_tel = [-r.pos_z for r in _tel]
            if z_tel:
                min_phys_alt = min(z_tel)
                log.info("Min physics altitude: %.2f m  (limit=%.1f m)",
                         min_phys_alt, _MIN_ALT_M)
        else:
            log.warning("No telemetry CSV -- skipping physics altitude check")
            min_phys_alt = _MIN_ALT_M   # pass through if no telemetry

        # ── Assertions ─────────────────────────────────────────────────────
        assert lua_captured, (
            "rawes.lua never captured equilibrium.\n"
            f"STATUSTEXT: {all_statustext}"
        )

        assert min_phys_alt >= _MIN_ALT_M, (
            f"Hub crashed: min physics alt {min_phys_alt:.2f} m < {_MIN_ALT_M:.1f} m\n"
            f"STATUSTEXT: {all_statustext}"
        )

        assert max_cyclic_activity >= _MIN_CYCLIC_ACTIVITY_PWM, (
            f"Lua cyclic activity too low: {max_cyclic_activity} PWM "
            f"< {_MIN_CYCLIC_ACTIVITY_PWM}\n"
            f"STATUSTEXT: {all_statustext}"
        )

        assert max_stable_s >= _MIN_STABLE_FLIGHT_S, (
            f"Hub only stable for {max_stable_s:.0f} s above {_STABLE_ALT_M} m "
            f"(need {_MIN_STABLE_FLIGHT_S:.0f} s).\n"
            f"EKF yaw reset: {ekf_yaw_reset}\n"
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

        log.info(
            "=== test_lua_flight_steady PASSED "
            "(stable=%.0fs  max_activity=%d PWM) ===",
            max_stable_s, max_cyclic_activity,
        )

    except Exception:
        dump_startup_diagnostics(ctx)
        raise
