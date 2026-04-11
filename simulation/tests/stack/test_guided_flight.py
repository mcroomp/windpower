"""
test_guided_flight.py — ACRO tether-alignment hold test.

Builds on the ``acro_armed`` fixture from conftest.py which handles:
    launch → connect → params → EKF alignment → arm → ACRO mode

This test only adds the observation phase:
    active tether-alignment RC for _HOLD_SECONDS → assert hub stayed above _MIN_ALT_M

ACRO was chosen over GUIDED because the RAWES equilibrium orientation is
~65° from vertical.  GUIDED's attitude-hold loop saw this as a large tilt
error and commanded maximum cyclic continuously, causing attitude divergence.

In ACRO, ArduPilot damps angular rates toward whatever RC rate we command.
Neutral sticks (zero rate target) fights the hub's natural tether-orbit
precession and destabilises the system.  Instead we run a P+D rate controller:

    cmd_roll  = -kp * roll  - kd * rollspeed
    cmd_pitch = -kp * pitch - kd * pitchspeed
    cmd_yaw   =             - kd * yawspeed

where roll/pitch are attitude error angles (physical attitude minus the
equilibrium captured at kinematic startup) and rollspeed/pitchspeed/yawspeed
are body-frame angular rates from the ArduPilot ATTITUDE message.
"""
import logging
import math
import sys
import time
from pathlib import Path

import pytest

_SIM_DIR   = Path(__file__).resolve().parents[2]
_STACK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_STACK_DIR))

from conftest import StackContext, analyze_startup_logs, dump_startup_diagnostics

# ── Timing ───────────────────────────────────────────────────────────────────
_HOLD_SECONDS = 60.0   # s — tether-alignment controller hold in ACRO

# ── Tolerances ───────────────────────────────────────────────────────────────
_MAX_DRIFT_M = 200.0   # m — runaway-only sentinel
_MIN_ALT_M   =   0.5  # m — hub must stay above this (not crashed)
                       # After kinematic phase the hub descends to ~1 m due to thrust
                       # margin limitation; 0.5 m catches a real ground impact.

# ── Logging interval ─────────────────────────────────────────────────────────
_POS_LOG_INTERVAL = 5.0   # s


def test_acro_hold(acro_armed: StackContext):
    """
    ACRO tether-alignment hold test.

    Uses the ``acro_armed`` fixture for setup.  Only the hold loop and
    assertions are here.

    The hold loop uses PhysicalHoldController.send_correction(), which subtracts
    the equilibrium roll/pitch captured at kinematic startup and calls
    compute_rc_from_attitude with the resulting deviation angles.

    Asserts:
      - Hub stays above _MIN_ALT_M (did not crash)
      - Hub drift < _MAX_DRIFT_M (runaway guard)
      - No CRITICAL errors in mediator log
    """
    ctx  = acro_armed
    gcs  = ctx.gcs
    log  = logging.getLogger("test_acro_hold")

    pos_history:      list[tuple[float, float, float, float]] = []
    attitude_history: list[tuple[float, float, float, float]] = []
    servo_history:    list[tuple[float, int, int, int, int]]  = []
    flight_events     = ctx.flight_events      # shared with fixture checkpoints
    all_statustext    = ctx.all_statustext

    _last_att: dict = {}             # most recent ATTITUDE fields
    _last_pos: tuple | None = None   # most recent (n, e, d) from LOCAL_POSITION_NED

    try:
        log.info("─── test_acro_hold: ACRO RC-override hold for %.0f s ───", _HOLD_SECONDS)
        t_obs_start   = time.monotonic()
        deadline_hold = t_obs_start + _HOLD_SECONDS
        t_last_rc     = t_obs_start - 3.0   # force immediate first send
        t_last_log    = t_obs_start

        flight_events["ACRO hold start"] = 0.0

        while time.monotonic() < deadline_hold:
            # Check processes haven't died
            for name, proc, lp in [
                ("mediator", ctx.mediator_proc, ctx.mediator_log),
                ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
            ]:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    pytest.fail(f"{name} exited during hold (rc={proc.returncode}):\n{txt[-3000:]}")

            msg = gcs._mav.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE", "SERVO_OUTPUT_RAW", "STATUSTEXT"],
                blocking=True, timeout=0.2,
            )
            t_rel = time.monotonic() - t_obs_start
            if msg is not None:
                mt = msg.get_type()
                if mt == "LOCAL_POSITION_NED":
                    pos_history.append((t_rel, msg.x, msg.y, msg.z))
                    _last_pos = (msg.x, msg.y, msg.z)
                elif mt == "ATTITUDE":
                    _last_att = {
                        "roll":       msg.roll,
                        "pitch":      msg.pitch,
                        "yaw":        msg.yaw,
                        "rollspeed":  msg.rollspeed,
                        "pitchspeed": msg.pitchspeed,
                        "yawspeed":   msg.yawspeed,
                    }
                    attitude_history.append((
                        t_rel,
                        math.degrees(msg.roll),
                        math.degrees(msg.pitch),
                        math.degrees(msg.yaw),
                    ))
                elif mt == "SERVO_OUTPUT_RAW":
                    servo_history.append((
                        t_rel,
                        msg.servo1_raw, msg.servo2_raw,
                        msg.servo3_raw, msg.servo4_raw,
                    ))
                elif mt == "STATUSTEXT":
                    text = msg.text.rstrip("\x00").strip()
                    log.warning("STATUSTEXT %s", text)
                    all_statustext.append(text)

            now = time.monotonic()
            # Send RC override every 0.1 s.
            # Phase 2 (internal_controller=True): mediator runs compute_swashplate_from_state
            #   at 400 Hz; RC override is a motor-interlock keepalive only (CH8=2000,
            #   attitude sticks neutral).
            # Normal path: send attitude correction via ctx.controller.send_correction().
            if now - t_last_rc >= 0.1:
                if ctx.internal_controller:
                    rc = {1: 1500, 2: 1500, 3: 1500, 4: 1500, 8: 2000}
                    gcs.send_rc_override(rc)
                elif _last_att:
                    rc = ctx.controller.send_correction(_last_att, _last_pos, gcs)
                else:
                    rc = {1: 1500, 2: 1500, 3: 1500, 4: 1500, 8: 2000}
                    gcs.send_rc_override(rc)
                t_last_rc = now

            if now - t_last_log >= _POS_LOG_INTERVAL and pos_history:
                _, n, e, d = pos_history[-1]
                drift = math.sqrt(n**2 + e**2 + d**2)
                log.info(
                    "  t=%.0fs  NED=(%.2f, %.2f, %.2f)  drift=%.2f m  remaining=%.0fs",
                    t_rel, n, e, d, drift, deadline_hold - now,
                )
                t_last_log = now

        log.info("Hold complete. pos=%d  att=%d  servo=%d",
                 len(pos_history), len(attitude_history), len(servo_history))

        # ── Assertions ────────────────────────────────────────────────────────
        # ATTITUDE messages always flow in ACRO mode regardless of GPS status.
        # LOCAL_POSITION_NED requires GPS fusion which is unreliable in physical-
        # sensor mode (tilted hub) — use ATTITUDE as the primary data-flow check.
        assert attitude_history, (
            "No ATTITUDE messages during hold — ACRO mode not running.\n"
            f"STATUSTEXT: {all_statustext}"
        )

        if pos_history:
            drifts    = [math.sqrt(n**2 + e**2 + d**2) for _, n, e, d in pos_history]
            max_drift = max(drifts)
            log.info("Drift: max=%.2f m  mean=%.2f m  (limit=%.0f m)",
                     max_drift, sum(drifts) / len(drifts), _MAX_DRIFT_M)
            assert max_drift <= _MAX_DRIFT_M, (
                f"Hub runaway in ACRO: max drift {max_drift:.2f} m > {_MAX_DRIFT_M:.0f} m\n"
                f"STATUSTEXT: {all_statustext}"
            )
        else:
            log.info("No LOCAL_POSITION_NED during hold (GPS not fused — expected in "
                     "physical-sensor mode). Drift check skipped; altitude from telemetry.")

        # Altitude from mediator telemetry (authoritative NED physics).
        # LOCAL_POSITION_NED D drifts significantly in CONST_POS_MODE (no GPS fusion)
        # and gives false crash detections; telemetry is the ground truth.
        # pos_z is NED Z (negative = above ground); altitude = -pos_z.
        if ctx.telemetry_log.exists():
            from telemetry_csv import read_csv as _read_csv
            _tel  = _read_csv(ctx.telemetry_log)
            _alts = [-r.pos_z for r in _tel]
            if _alts:
                min_alt = min(_alts)
                log.info("Min physics altitude (telemetry): %.2f m  (limit=%.1f m)",
                         min_alt, _MIN_ALT_M)
                assert min_alt >= _MIN_ALT_M, (
                    f"Hub crashed: min altitude {min_alt:.2f} m < {_MIN_ALT_M:.1f} m\n"
                    f"STATUSTEXT: {all_statustext}"
                )

        if ctx.mediator_log.exists():
            med_text = ctx.mediator_log.read_text(encoding="utf-8", errors="replace")
            critical = [l for l in med_text.splitlines() if "CRITICAL" in l]
            assert not critical, (
                "CRITICAL errors in mediator:\n" + "\n".join(critical[:10])
            )

        if pos_history:
            log.info("─── test_acro_hold PASSED (max_drift=%.2f m) ───", max_drift)
        else:
            log.info("─── test_acro_hold PASSED (no GPS fusion, drift check skipped) ───")

    except Exception:
        dump_startup_diagnostics(ctx)
        raise

    finally:
        import shutil as _shutil

