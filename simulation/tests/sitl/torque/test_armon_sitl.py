"""
torque/test_armon_sitl.py — RAWES_ARM timed disarm-timer test.

Validates RAWES_ARM disarm timer behavior with GCS arming:
    Phase 1 — arm via GCS, optional long timer, Lua PI regulates yaw.
    Phase 2 — set short timer, Lua disarms on expiry, psi_dot grows.

Timeline (SITL seconds):
  t = 0..15 s   STARTUP hold — EKF alignment; vehicle unarmed.
    fixture yields (ACRO mode, unarmed).
    test arms via GCS and sends RAWES_ARM(3 600 000 ms) to set countdown.
  t ~ 15 s      DYNAMIC begins — rotor spinning, Lua PI active.
  t ~ 55-65 s   PI settled; observation window 1 opens (psi_dot < threshold).
    test sends RAWES_ARM(5 000 ms) to shorten deadline to 5 s from now.
    Lua disarms → "RAWES disarm timer expired, disarmed" STATUSTEXT.
  observation window 2 — motor at 800 µs, psi_dot growing beyond threshold.

Pass criteria:
  [armed]   max |psi_dot| < 5 deg/s (physics ground truth) over 10 s window.
  [disarmed] motor PWM = 800 us AND max |psi_dot| > 10 deg/s over 8 s window.

Run with (inside Docker):
  RAWES_RUN_STACK_INTEGRATION=1 pytest \\
      simulation/tests/sitl/torque/test_armon_sitl.py -v
"""
from __future__ import annotations

import math
from pathlib import Path

import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]

import sys as _sys
for _p in (str(_SIM_DIR), str(_SITL_DIR)):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)

from stack_infra import observe
from torque_test_utils import save_telemetry
from telemetry_csv import TelRow
from rawes_modes import NV_ARMON_KEY

# ---------------------------------------------------------------------------
# Timing constants
# ---------------------------------------------------------------------------

# Time to wait after arming for the Lua PI integrator to settle.
_SETTLE_S      = 50.0   # s from DYNAMIC start; PI takes ~50 s to reach steady state
_REGULATE_S    = 10.0   # s: observation window while armed (assert psi_dot < threshold)
_EXPIRE_MS     = 5_000  # ms: short deadline sent to trigger expiry
_DRIFT_S       = 8.0    # s: observation window after disarm (assert motor off + drift)

_REGULATE_THRESHOLD_RAD_S = math.radians(5.0)   # 5 deg/s — must hold during regulation
_DRIFT_THRESHOLD_RAD_S    = math.radians(10.0)  # 10 deg/s — must exceed after disarm

_ARMED_ARMON_MS = 3_600_000   # 1 hour: keep armed for settle + regulation phases


def test_armon_sitl(torque_unarmed_lua):
    """
    GCS force-arms; RAWES_ARM sets an optional disarm countdown timer in Lua.
    While the timer is active: Lua PI regulates hub yaw (psi_dot < threshold).
    After a short-deadline re-send: Lua disarms, motor cuts, yaw drifts.
    """
    ctx = torque_unarmed_lua
    log = ctx.log
    gcs = ctx.gcs

    # ── Phase 1 setup: arm via GCS, then set optional long disarm timer ──────
    log.info("Arming via GCS ...")
    gcs.arm(timeout=20.0, force=True)
    log.info("Sending RAWES_ARM=%d ms disarm timer ...", _ARMED_ARMON_MS)
    gcs.send_named_float(NV_ARMON_KEY, float(_ARMED_ARMON_MS))

    # ── Phase 1 observation: regulation ──────────────────────────────────────
    # Collect psi_dot samples via ATTITUDE messages.  settle_s gives the PI
    # time to wind up; the observation window follows.
    log.info("Waiting %g s for PI to settle, then observing %g s regulation ...",
             _SETTLE_S, _REGULATE_S)

    pwm       = [0]
    rows: list[TelRow] = []
    regulated: list[dict] = []
    expire_sent = False

    total_timeout = _SETTLE_S + _REGULATE_S + _DRIFT_S + 30.0  # generous margin

    def handle(msg, t_rel):
        nonlocal expire_sent

        if msg is None:
            return None
        mt = msg.get_type()

        if mt == "STATUSTEXT":
            text = msg.text.rstrip("\x00").strip()
            log.info("t=%.1f  SITL: %s", t_rel, text)
            # Disarm confirmation — end the loop so phase 2 check can begin.
            if "RAWES disarm timer expired, disarmed" in text:
                return True

        elif mt == "SERVO_OUTPUT_RAW":
            ch9 = getattr(msg, "servo4_raw", 0) or 0
            pwm[0] = ch9

        elif mt == "ATTITUDE":
            rows.append(TelRow(
                t_sim=t_rel, phase="DYNAMIC",
                rpy_roll=msg.roll, rpy_pitch=msg.pitch, rpy_yaw=msg.yaw,
                omega_z=msg.yawspeed, omega_rotor=ctx.omega_rotor,
                servo4_us=float(pwm[0]),
            ))

            in_window = _SETTLE_S <= t_rel < _SETTLE_S + _REGULATE_S
            if in_window:
                regulated.append({"t": t_rel, "psi_dot": msg.yawspeed, "pwm": pwm[0]})
                if len(regulated) % 20 == 1:
                    log.info(
                        "t=%6.1f  [regulate]  psi_dot=%+.2f deg/s  pwm=%d",
                        t_rel, math.degrees(msg.yawspeed), pwm[0],
                    )

            # Once regulation window is complete, re-send RAWES_ARM with short deadline.
            if t_rel >= _SETTLE_S + _REGULATE_S and not expire_sent:
                expire_sent = True
                log.info("Regulation window done -- sending RAWES_ARM=%d ms to trigger expiry",
                         _EXPIRE_MS)
                gcs.send_named_float(NV_ARMON_KEY, float(_EXPIRE_MS))

        return None

    observe(
        ctx, total_timeout, handle,
        msg_types=["ATTITUDE", "STATUSTEXT", "SERVO_OUTPUT_RAW"],
        keepalive={},    # no GCS RC override needed
    )

    # ── Assert Phase 1: regulated psi_dot ────────────────────────────────────
    if len(regulated) < 10:
        pytest.fail(
            f"Not enough ATTITUDE samples in regulation window "
            f"(got {len(regulated)}, need >= 10)"
        )
    max_psi_dot = max(abs(s["psi_dot"]) for s in regulated)
    log.info(
        "Regulation window (%g s, %d samples):  max|psi_dot|=%.2f deg/s (limit %.1f)",
        _REGULATE_S, len(regulated),
        math.degrees(max_psi_dot), math.degrees(_REGULATE_THRESHOLD_RAD_S),
    )
    assert max_psi_dot <= _REGULATE_THRESHOLD_RAD_S, (
        f"psi_dot exceeded limit during regulation: "
        f"{math.degrees(max_psi_dot):.2f} deg/s > {math.degrees(_REGULATE_THRESHOLD_RAD_S):.1f} deg/s"
    )
    log.info("PASS [regulated] -- psi_dot held within limit")

    # ── Phase 2 observation: disarmed drift ───────────────────────────────────
    log.info("Observing %g s post-expiry drift ...", _DRIFT_S)

    drift: list[dict] = []
    drift_rows: list[TelRow] = []
    t_drift_start = gcs.sim_now()

    def handle_drift(msg, t_rel):
        if msg is None:
            return None
        mt = msg.get_type()
        if mt == "STATUSTEXT":
            log.info("t=%.1f  SITL: %s", t_rel, msg.text.rstrip("\x00").strip())
        elif mt == "SERVO_OUTPUT_RAW":
            ch9 = getattr(msg, "servo4_raw", 0) or 0
            pwm[0] = ch9
        elif mt == "ATTITUDE":
            elapsed = gcs.sim_now() - t_drift_start
            drift_rows.append(TelRow(
                t_sim=t_rel, phase="DRIFT",
                rpy_roll=msg.roll, rpy_pitch=msg.pitch, rpy_yaw=msg.yaw,
                omega_z=msg.yawspeed, omega_rotor=ctx.omega_rotor,
                servo4_us=float(pwm[0]),
            ))
            drift.append({"t": t_rel, "psi_dot": msg.yawspeed, "pwm": pwm[0]})
            if len(drift) % 20 == 1:
                log.info(
                    "t=%6.1f  [drift]  psi_dot=%+.2f deg/s  pwm=%d",
                    t_rel, math.degrees(msg.yawspeed), pwm[0],
                )
            if elapsed >= _DRIFT_S:
                return True
        return None

    observe(
        ctx, _DRIFT_S + 15.0, handle_drift,
        msg_types=["ATTITUDE", "STATUSTEXT", "SERVO_OUTPUT_RAW"],
        keepalive={},
    )

    save_telemetry(rows + drift_rows, "armon", log)

    # ── Assert Phase 2: motor off and yaw drifting ───────────────────────────
    if len(drift) < 5:
        pytest.fail(
            f"Not enough ATTITUDE samples in drift window "
            f"(got {len(drift)}, need >= 5)"
        )

    # Motor must be at 800 µs after a brief spin-down period.
    # Allow 2 s for the ESC to ramp to idle after disarm before checking.
    _SPINDOWN_S = 2.0
    bad_pwm = [s for s in drift if s["t"] > _SPINDOWN_S and s["pwm"] not in (0, 800)]
    if bad_pwm:
        worst = max(bad_pwm, key=lambda s: s["pwm"])
        pytest.fail(
            f"Motor not at 800 us after disarm+spindown: pwm={worst['pwm']} at t={worst['t']:.1f} s"
        )
    log.info("PASS [motor off] -- all %d drift samples at 800 us", len(drift))

    # psi_dot must exceed threshold (yaw genuinely out of control).
    max_drift = max(abs(s["psi_dot"]) for s in drift)
    log.info(
        "Drift window (%g s, %d samples):  max|psi_dot|=%.2f deg/s (need > %.1f)",
        _DRIFT_S, len(drift),
        math.degrees(max_drift), math.degrees(_DRIFT_THRESHOLD_RAD_S),
    )
    assert max_drift >= _DRIFT_THRESHOLD_RAD_S, (
        f"psi_dot did not grow after disarm: "
        f"{math.degrees(max_drift):.2f} deg/s < {math.degrees(_DRIFT_THRESHOLD_RAD_S):.1f} deg/s"
    )
    log.info("PASS [drift] -- psi_dot reached %.2f deg/s after disarm",
             math.degrees(max_drift))
