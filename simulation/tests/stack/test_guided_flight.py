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

where roll/pitch are the tether-relative attitude angles the mediator reports
(zero at tether equilibrium) and rollspeed/pitchspeed/yawspeed are the
body-frame angular rates from the ArduPilot ATTITUDE message.
"""
import json
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

from flight_report import plot_flight_report as _plot_flight_report_base

# ── Timing ───────────────────────────────────────────────────────────────────
_HOLD_SECONDS = 60.0   # s — tether-alignment controller hold in ACRO

# ── Tolerances ───────────────────────────────────────────────────────────────
_MAX_DRIFT_M = 200.0   # m — runaway-only sentinel
_MIN_ALT_M   =   0.5  # m — hub must stay above this (not crashed)
                       # After kinematic phase the hub descends to ~1 m due to thrust
                       # margin limitation; 0.5 m catches a real ground impact.

# ── Logging interval ─────────────────────────────────────────────────────────
_POS_LOG_INTERVAL = 5.0   # s


def _plot_flight_report(
    pos_history:      list,
    attitude_history: list,
    servo_history:    list,
    events:           dict,
    out_path:         Path,
    telemetry_path:   Path | None = None,
    home_alt_m:       float = 0.0,
) -> None:
    _STARTING_STATE = _SIM_DIR / "steady_state_starting.json"
    if _STARTING_STATE.exists():
        st      = json.loads(_STARTING_STATE.read_text())
        pos_ned = st["pos"]   # NED: hub starting position in physics frame
        # In LOCAL_POSITION_NED, HOME = hub start; anchor at [0, 0, -pos_ned[2]]
        anchor_ned = (0.0, 0.0, -float(pos_ned[2]))
        tether_len = math.sqrt(sum(x**2 for x in pos_ned))
    else:
        anchor_ned = (0.0, 0.0, 0.0)
        tether_len = 50.0

    _plot_flight_report_base(
        pos_history      = pos_history,
        attitude_history = attitude_history,
        servo_history    = servo_history,
        events           = events,
        target           = (0.0, 0.0, 0.0),
        out_path         = out_path,
        telemetry_path   = telemetry_path,
        tether_length_m  = tether_len,
        anchor_ned       = anchor_ned,
    )


@pytest.mark.xfail(
    reason=(
        "GPS horizontal position does not fuse reliably in physical-sensor mode. "
        "With roll=124deg/pitch=-46deg the EKF dead-reckons horizontal drift from "
        "gravity projection during AID_NONE, so GPS innovations exceed the gate when "
        "fusion is first attempted.  Tracked issue: GPS fusion with tilted hub."
    ),
    strict=False,
)
def test_gps_fuses_during_startup(acro_armed: StackContext):
    """
    GPS horizontal position must fuse within 60 s of ACRO setup.

    Asserts:
      - At least one LOCAL_POSITION_NED received within 60 s

    XFAIL: GPS fusion is unreliable in physical-sensor mode (roll=124 deg/
    pitch=-46 deg).  In flat/tether-relative mode this passed; the physical
    mode EKF dead-reckons horizontal drift from gravity projection, preventing
    fusion.  The test is kept so that any future fix is caught immediately.
    """
    ctx = acro_armed
    gcs = ctx.gcs
    log = logging.getLogger("test_gps_fuses")

    log.info("Waiting up to 60 s for LOCAL_POSITION_NED ...")
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        for name, proc, lp in [
            ("mediator", ctx.mediator_proc, ctx.mediator_log),
            ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
        ]:
            if proc.poll() is not None:
                txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                pytest.fail(f"{name} exited during GPS wait (rc={proc.returncode}):\n{txt[-2000:]}")

        msg = gcs._mav.recv_match(
            type=["LOCAL_POSITION_NED"],
            blocking=True, timeout=0.5,
        )
        if msg is not None:
            log.info("GPS fused: LOCAL_POSITION_NED  x=%.2f y=%.2f z=%.2f",
                     msg.x, msg.y, msg.z)
            return  # pass

    pytest.fail(
        "LOCAL_POSITION_NED never received within 60 s after ACRO setup.\n"
        "EKF horizontal position did not fuse."
    )


def test_acro_hold(acro_armed: StackContext):
    """
    ACRO tether-alignment hold test.

    Uses the ``acro_armed`` fixture for setup.  Only the hold loop and
    assertions are here.

    The hold loop runs a P+D rate controller (compute_rc_from_attitude) that
    keeps the hub aligned with the tether direction.  The mediator reports
    tether-relative attitude (roll=pitch=0 at equilibrium), so ATTITUDE.roll
    and .pitch are directly the attitude error angles.

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
    rc_history:       list[tuple[float, int, int, int, int]]  = []  # (t, ch1, ch2, ch3, ch4)
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
                rc_history.append((
                    now - t_obs_start,
                    rc.get(1, 1500), rc.get(2, 1500),
                    rc.get(3, 1500), rc.get(4, 1500),
                ))
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
        # hub_pos_z is NED Z (negative = above ground); altitude = -hub_pos_z.
        if ctx.telemetry_log.exists():
            import csv as _csv
            with ctx.telemetry_log.open(encoding="utf-8") as _f:
                _tel = list(_csv.DictReader(_f))
            _alts = [-float(r["hub_pos_z"]) for r in _tel
                     if r.get("hub_pos_z") not in ("", "None", "nan", None)]
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

        log.info("─── test_acro_hold PASSED (max_drift=%.2f m) ───", max_drift)

    except Exception:
        dump_startup_diagnostics(ctx)
        raise

    finally:
        import shutil as _shutil

        # Save flight data JSON for redraw tool
        if pos_history and ctx.initial_state:
            anchor_ned = [
                -ctx.initial_state["pos"][1],
                -ctx.initial_state["pos"][0],
                float(ctx.initial_state["pos"][2]),
            ]
            flight_data = {
                "pos_history":      [list(r) for r in pos_history],
                "attitude_history": [list(r) for r in attitude_history],
                "servo_history":    [list(r) for r in servo_history],
                "rc_history":       [list(r) for r in rc_history],
                "events":           flight_events,
                "target":           [0.0, 0.0, 0.0],
                "anchor_ned":       anchor_ned,
            }
            with (ctx.sim_dir / "logs" / "flight_data.json").open("w", encoding="utf-8") as f:
                json.dump(flight_data, f)

        if pos_history:
            try:
                _plot_flight_report(
                    pos_history      = pos_history,
                    attitude_history = attitude_history,
                    servo_history    = servo_history,
                    events           = flight_events,
                    out_path         = ctx.sim_dir / "flight_report.png",
                    telemetry_path   = ctx.telemetry_log if ctx.telemetry_log.exists() else None,
                    home_alt_m       = ctx.home_alt_m,
                )
                log.info("Flight report → %s", ctx.sim_dir / "flight_report.png")
            except Exception as exc:
                log.warning("Flight report failed: %s", exc)
