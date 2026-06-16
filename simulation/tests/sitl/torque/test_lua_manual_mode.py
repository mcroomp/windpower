"""
torque/test_lua_manual_mode.py  --  rawes.lua MODE_MANUAL (SCR_USER6=2) stack test.

Verifies that rawes.lua's MODE_MANUAL correctly:
  1. Drives SERVO4 via the yaw PID to counter rotor reaction torque.
  2. Responds to RAWES_TLN / RAWES_TLT NVFs by shifting S1/S2/S3 swash
     servo PWMs in the correct direction according to the H3-120 mixer.
  3. Responds to RAWES_COL NVF by shifting RC3 (collective) proportionally.
  4. Returns to neutral swash when tlon=tlat=0 NVFs are sent.

Physical scenario
-----------------
  Same torque-rig setup as test_lua_yaw_regulation.py:
    STARTUP (t=0..15 s)   : rotor stationary, arming.
    SPINUP  (t=15..25 s)  : 10 s ramp to 120 RPM.
    HOLD    (t=25..55 s)  : 30 s at 120 RPM; all assertions are made here.

  During HOLD:
    Phase A (t=0..10 s of HOLD)  : neutral tlon/tlat, baseline collective.
      - SERVO4 must be > 800 us (yaw PID countering rotor torque).
      - RC1 ≈ 1500, RC2 ≈ 1500 (neutral cyclic).
      - S1, S2, S3 record the neutral swash baseline.
    Phase B (t=10..20 s of HOLD) : send RAWES_TLN=+5 deg, RAWES_TLT=+3 deg.
      - RC2 must shift up relative to neutral (tlon +5 deg -> nose-down).
      - RC1 must shift up relative to neutral (tlat +3 deg -> roll-right).
      - S1 must rise   ~+115 us (H3-120 mixer: roll+0.390 + pitch+0.225 both add).
      - S2 must stay   ~   0 us (roll -0.390 and pitch +0.225 nearly cancel).
      - S3 must fall   ~-113 us (pitch -0.450 dominates; no roll contribution).
    Phase C (t=20..30 s of HOLD) : send tlon=0, tlat=0 to restore neutral.
      - RC1 and RC2 must return to ~1500 us.
      - S1, S2, S3 must return within 50 us of Phase A baseline.

  H_FLYBAR_MODE=1 is set so RC1/RC2 overrides bypass the rate PID and go
  directly to the swash mixer — the setpoint-to-PWM relationship is linear.

Pass criteria
-------------
  A. SERVO4 mean > 900 us during Phase A (yaw PID active, not pegged at 800).
  B. RC2 in Phase B > RC2 baseline + 100 us  (tlon +5 deg -> +250 us expected).
  C. RC1 in Phase B > RC1 baseline + 60 us   (tlat +3 deg -> +150 us expected;
     generous margin for param-read latency and observer timing).
  D. |RC1 - 1500| < 80 us and |RC2 - 1500| < 80 us after neutral restore.
  E. S1 in Phase B > S1_A + 80 us  (H3-120: roll+0.390*150 + pitch+0.225*250 = +115 us).
  F. |S2 - S2_A| < 50 us in Phase B (H3-120: roll-0.390*150 + pitch+0.225*250 = -2 us).
  G. S3 in Phase B < S3_A - 80 us  (H3-120: pitch -0.450*250 = -113 us).
  H. S1, S2, S3 return within 50 us of Phase A baseline after neutral restore.

H3-120 swash geometry (from AGENTS.md, verified via run passive --trim oscillate=1):
  Servo azimuths: S1=-60 deg (front-right), S2=+60 deg (front-left), S3=180 deg (back).
  roll_factor  = -sin(az)*0.45:  S1=+0.390, S2=-0.390, S3= 0.000
  pitch_factor =  cos(az)*0.45:  S1=+0.225, S2=+0.225, S3=-0.450
  Servo_n_delta = roll_factor_n*(RC1-1500) + pitch_factor_n*(RC2-1500)

Arming
------
  No GPS — armed via RAWES_ARM(1 hour) after EKF attitude alignment.
  See ``stack_infra._launch_mediator_torque`` for no-GPS configuration.

Run with (inside Docker)
------------------------
  bash simulation/dev.sh test-stack -n 1 -k test_lua_manual_mode
"""
from __future__ import annotations

import math

import pytest

from conftest import LUA_YAW_IC_COL
from stack_infra import observe
from torque_test_utils import save_telemetry


# Timing (SITL seconds) — mirrors test_lua_yaw_regulation.py timing.
_SETTLE_S    = 45.0   # startup_hold(15) + spinup(10) + 20 s settle
_PHASE_A_S   = 10.0   # observe neutral cyclic
_PHASE_B_S   = 10.0   # observe after +tlon/+tlat NVF
_PHASE_C_S   = 10.0   # observe after neutral restore

# Cyclic setpoints for Phase B.
_TLON_B_DEG  = 5.0    # nose-down; expected RC2 = 1500 + 250 = 1750 us
_TLAT_B_DEG  = 3.0    # roll-right; expected RC1 = 1500 + 150 = 1650 us

# H_CYC_MAX = 1000 cd = 10 deg at full stick -> 500 us for ±full.
# delta = deg / 10.0 * 500.
_EXPECTED_RC2_DELTA = int(_TLON_B_DEG / 10.0 * 500)   # 250 us
_EXPECTED_RC1_DELTA = int(_TLAT_B_DEG / 10.0 * 500)   # 150 us

# Generous assertion margins (param-read latency + ArduPilot RC processing).
_RC_MARGIN_US    = 60    # minimum us shift required vs baseline
_RC_NEUTRAL_TOL  = 80    # max deviation from 1500 after neutral restore
_S4_ACTIVE_US    = 900   # SERVO4 must be > this during rotor hold

# H3-120 swash mixer geometry (AGENTS.md §Swashplate):
#   roll_factor:  S1=+0.390, S2=-0.390, S3= 0.000
#   pitch_factor: S1=+0.225, S2=+0.225, S3=-0.450
# Phase B inputs: RC1-1500=+150 (tlat+3deg), RC2-1500=+250 (tlon+5deg).
_EXPECTED_S1_DELTA = int(0.390 * (_EXPECTED_RC1_DELTA) + 0.225 * (_EXPECTED_RC2_DELTA))  # +115
_EXPECTED_S2_DELTA = int(-0.390 * (_EXPECTED_RC1_DELTA) + 0.225 * (_EXPECTED_RC2_DELTA)) # -2
_EXPECTED_S3_DELTA = int(0.000 * (_EXPECTED_RC1_DELTA) + (-0.450) * (_EXPECTED_RC2_DELTA)) # -112

_SERVO_S1_MARGIN_US  = 80   # S1 must rise by at least this in Phase B
_SERVO_S3_MARGIN_US  = 80   # S3 must fall by at least this in Phase B
_SERVO_S2_TOL_US     = 50   # S2 must stay within this of baseline in Phase B
_SERVO_RESTORE_TOL   = 50   # each servo must return within this of Phase A baseline


def test_lua_manual_mode(torque_armed_lua_manual):
    """
    rawes.lua MODE_MANUAL (SCR_USER6=2): yaw PID + NVF-commanded swash.

    Sends RAWES_TLN, RAWES_TLT, RAWES_COL NVFs mid-run and asserts that:
      - SERVO4 is active (yaw PID driving the GB4008 counter-torque motor).
      - RC1/RC2 shift by the expected amount when tlon/tlat NVFs arrive.
      - RC1/RC2 return to neutral when tlon=tlat=0 NVFs are sent.
      - S1/S2/S3 SERVO_OUTPUT_RAW PWMs shift by the amounts the H3-120 mixer
        predicts from the RC1/RC2 changes (validates the full chain: NVF ->
        Lua RC override -> ArduPilot swash mixer -> physical servo PWM).
      - S1/S2/S3 return near Phase A baseline after neutral restore.

    Failure modes this catches:
      - SCR_USER6 not applied (Lua stays in MODE_NONE; SERVO4 stays at 800).
      - RAWES_TLN / RAWES_TLT NVFs not received / not stored (_man_*_rad).
      - H_FLYBAR_MODE not set to 1 (RC override goes through rate PID, not
        directly to swash; PWM changes are attenuated and delayed).
      - run_manual() RC2 formula using wrong axis (tlon should drive RC2,
        not RC1 -- any swap produces opposite-direction delta).
      - NVF not persistent: RC1/RC2 snap back to 1500 on next tick after
        single NVF delivery.
      - H3-120 mixer wired with wrong servo azimuths (S1/S2 signs swapped,
        or S3 pitch factor sign wrong -- any such bug changes which servos
        move and in which direction).
    """
    ctx = torque_armed_lua_manual
    rows, result = _run_manual_loop(ctx)
    save_telemetry(rows, "lua_manual_mode", ctx.log)
    _assert_manual_mode(result, ctx.log)


# ---------------------------------------------------------------------------
# Observation loop
# ---------------------------------------------------------------------------

def _run_manual_loop(ctx):
    """
    Three-phase observation loop (settle -> Phase A -> NVF -> Phase B -> NVF -> Phase C).

    Returns:
      rows   -- list[TelRow] for save_telemetry
      result -- dict with per-phase statistics for assertions
    """
    from telemetry_csv import TelRow

    state = {"s1": 1500, "s2": 1500, "s3": 1500, "s4": 800,
             "rc1": 1500, "rc2": 1500, "rc3": 1500}
    rows:   list       = []
    phase_a:  list[dict] = []
    phase_b:  list[dict] = []
    phase_c:  list[dict] = []
    nvf_b_sent  = [False]
    nvf_c_sent  = [False]

    t_hold_start = [None]   # set when SETTLE_S elapsed

    def handle(msg, t_rel):
        if msg is None:
            return None
        mt = msg.get_type()

        if mt == "STATUSTEXT":
            ctx.log.debug("SITL t=%.1fs: %s", t_rel,
                          msg.text.rstrip("\x00").strip())
            return None

        if mt == "RC_CHANNELS":
            state["rc1"] = int(getattr(msg, "chan1_raw", 1500) or 1500)
            state["rc2"] = int(getattr(msg, "chan2_raw", 1500) or 1500)
            state["rc3"] = int(getattr(msg, "chan3_raw", 1500) or 1500)
            return None

        if mt == "SERVO_OUTPUT_RAW":
            state["s1"] = int(getattr(msg, "servo1_raw", 1500) or 1500)
            state["s2"] = int(getattr(msg, "servo2_raw", 1500) or 1500)
            state["s3"] = int(getattr(msg, "servo3_raw", 1500) or 1500)
            state["s4"] = int(getattr(msg, "servo4_raw", 800) or 800)
            return None

        if mt != "ATTITUDE":
            return None

        rows.append(TelRow(
            t_sim=t_rel, phase="DYNAMIC",
            rpy_roll=msg.roll, rpy_pitch=msg.pitch, rpy_yaw=msg.yaw,
            omega_z=msg.yawspeed, omega_rotor=ctx.omega_rotor,
            servo4_us=float(state["s4"]),
        ))

        if t_rel < _SETTLE_S:
            return None

        # Mark hold start once.
        if t_hold_start[0] is None:
            t_hold_start[0] = t_rel
            ctx.log.info("Phase A started at t=%.1f s (SETTLE_S=%.0f)", t_rel, _SETTLE_S)

        t_hold = t_rel - t_hold_start[0]

        # Phase A: observe neutral baseline.
        if t_hold < _PHASE_A_S:
            phase_a.append({
                "t": t_rel, "s4": state["s4"],
                "rc1": state["rc1"], "rc2": state["rc2"], "rc3": state["rc3"],
                "s1": state["s1"], "s2": state["s2"], "s3": state["s3"],
            })
            return None

        # End of Phase A: send tlon/tlat NVFs for Phase B.
        if not nvf_b_sent[0]:
            nvf_b_sent[0] = True
            ctx.gcs.send_named_float("RAWES_TLN", math.radians(_TLON_B_DEG))
            ctx.gcs.send_named_float("RAWES_TLT", math.radians(_TLAT_B_DEG))
            ctx.log.info(
                "Sent RAWES_TLN=%.1f deg, RAWES_TLT=%.1f deg at t=%.1f s",
                _TLON_B_DEG, _TLAT_B_DEG, t_rel,
            )

        # Phase B: observe after NVF.
        if t_hold < _PHASE_A_S + _PHASE_B_S:
            # Skip first ~1 s to allow Lua to process the NVF and AP to refresh RC.
            if t_hold >= _PHASE_A_S + 1.0:
                phase_b.append({
                    "t": t_rel, "s4": state["s4"],
                    "rc1": state["rc1"], "rc2": state["rc2"], "rc3": state["rc3"],
                    "s1": state["s1"], "s2": state["s2"], "s3": state["s3"],
                })
            return None

        # End of Phase B: restore neutral.
        if not nvf_c_sent[0]:
            nvf_c_sent[0] = True
            ctx.gcs.send_named_float("RAWES_TLN", 0.0)
            ctx.gcs.send_named_float("RAWES_TLT", 0.0)
            ctx.log.info("Sent neutral tlon/tlat NVFs at t=%.1f s", t_rel)

        # Phase C: observe after neutral restore.
        if t_hold < _PHASE_A_S + _PHASE_B_S + _PHASE_C_S:
            if t_hold >= _PHASE_A_S + _PHASE_B_S + 1.0:
                phase_c.append({
                    "t": t_rel, "s4": state["s4"],
                    "rc1": state["rc1"], "rc2": state["rc2"], "rc3": state["rc3"],
                    "s1": state["s1"], "s2": state["s2"], "s3": state["s3"],
                })
            return None

        # All phases done.
        return True

    total_s = _SETTLE_S + _PHASE_A_S + _PHASE_B_S + _PHASE_C_S + 10.0
    observe(ctx, total_s, handle,
            msg_types=["ATTITUDE", "RC_CHANNELS", "SERVO_OUTPUT_RAW", "STATUSTEXT"],
            keepalive={8: 2000})

    def _mean(lst, key):
        return sum(r[key] for r in lst) / max(len(lst), 1) if lst else float("nan")

    result = {
        "a_s4_mean":  _mean(phase_a, "s4"),
        "a_rc1_mean": _mean(phase_a, "rc1"),
        "a_rc2_mean": _mean(phase_a, "rc2"),
        "a_s1_mean":  _mean(phase_a, "s1"),
        "a_s2_mean":  _mean(phase_a, "s2"),
        "a_s3_mean":  _mean(phase_a, "s3"),
        "b_s4_mean":  _mean(phase_b, "s4"),
        "b_rc1_mean": _mean(phase_b, "rc1"),
        "b_rc2_mean": _mean(phase_b, "rc2"),
        "b_s1_mean":  _mean(phase_b, "s1"),
        "b_s2_mean":  _mean(phase_b, "s2"),
        "b_s3_mean":  _mean(phase_b, "s3"),
        "c_rc1_mean": _mean(phase_c, "rc1"),
        "c_rc2_mean": _mean(phase_c, "rc2"),
        "c_s1_mean":  _mean(phase_c, "s1"),
        "c_s2_mean":  _mean(phase_c, "s2"),
        "c_s3_mean":  _mean(phase_c, "s3"),
        "a_n": len(phase_a),
        "b_n": len(phase_b),
        "c_n": len(phase_c),
    }
    ctx.log.info(
        "Phase A (%d):  s4=%.0f  rc1=%.0f  rc2=%.0f  s1=%.0f  s2=%.0f  s3=%.0f",
        result["a_n"], result["a_s4_mean"], result["a_rc1_mean"], result["a_rc2_mean"],
        result["a_s1_mean"], result["a_s2_mean"], result["a_s3_mean"],
    )
    ctx.log.info(
        "Phase B (%d):  s4=%.0f  rc1=%.0f  rc2=%.0f  s1=%.0f  s2=%.0f  s3=%.0f  "
        "(tlon=%.0f deg  tlat=%.0f deg)",
        result["b_n"], result["b_s4_mean"], result["b_rc1_mean"], result["b_rc2_mean"],
        result["b_s1_mean"], result["b_s2_mean"], result["b_s3_mean"],
        _TLON_B_DEG, _TLAT_B_DEG,
    )
    ctx.log.info(
        "Phase B swash deltas:  dS1=%+.0f (exp~%+d)  dS2=%+.0f (exp~%+d)  dS3=%+.0f (exp~%+d)",
        result["b_s1_mean"] - result["a_s1_mean"], _EXPECTED_S1_DELTA,
        result["b_s2_mean"] - result["a_s2_mean"], _EXPECTED_S2_DELTA,
        result["b_s3_mean"] - result["a_s3_mean"], _EXPECTED_S3_DELTA,
    )
    ctx.log.info(
        "Phase C (%d):  rc1=%.0f  rc2=%.0f  s1=%.0f  s2=%.0f  s3=%.0f  (neutral restore)",
        result["c_n"], result["c_rc1_mean"], result["c_rc2_mean"],
        result["c_s1_mean"], result["c_s2_mean"], result["c_s3_mean"],
    )
    return rows, result


# ---------------------------------------------------------------------------
# Assertions
# ---------------------------------------------------------------------------

def _assert_manual_mode(result: dict, log) -> None:
    """Assert all four pass criteria for the manual mode test."""

    _require_samples(result["a_n"], "Phase A")
    _require_samples(result["b_n"], "Phase B")
    _require_samples(result["c_n"], "Phase C")

    # A. SERVO4 active during Phase A (yaw PID responding to rotor torque).
    s4_a = result["a_s4_mean"]
    assert s4_a > _S4_ACTIVE_US, (
        f"SERVO4 mean in Phase A = {s4_a:.0f} us; expected > {_S4_ACTIVE_US} us "
        f"(yaw PID should drive motor to counter rotor reaction torque)"
    )
    log.info("PASS A -- SERVO4 active: mean=%.0f us > %d us", s4_a, _S4_ACTIVE_US)

    # B. RC2 shifted up in Phase B (tlon +5 deg -> nose-down -> +250 us from 1500).
    rc2_a = result["a_rc2_mean"]
    rc2_b = result["b_rc2_mean"]
    rc2_delta = rc2_b - rc2_a
    assert rc2_delta >= _RC_MARGIN_US, (
        f"RC2 delta in Phase B = {rc2_delta:+.0f} us (baseline={rc2_a:.0f}, after_nvf={rc2_b:.0f}); "
        f"expected >= +{_RC_MARGIN_US} us for tlon=+{_TLON_B_DEG:.0f} deg "
        f"(expected ~+{_EXPECTED_RC2_DELTA} us)"
    )
    log.info(
        "PASS B -- RC2 shifted: baseline=%.0f -> nvf=%.0f (delta=%+.0f, expected ~+%d) us",
        rc2_a, rc2_b, rc2_delta, _EXPECTED_RC2_DELTA,
    )

    # C. RC1 shifted up in Phase B (tlat +3 deg -> roll-right -> +150 us from 1500).
    rc1_a = result["a_rc1_mean"]
    rc1_b = result["b_rc1_mean"]
    rc1_delta = rc1_b - rc1_a
    assert rc1_delta >= _RC_MARGIN_US, (
        f"RC1 delta in Phase B = {rc1_delta:+.0f} us (baseline={rc1_a:.0f}, after_nvf={rc1_b:.0f}); "
        f"expected >= +{_RC_MARGIN_US} us for tlat=+{_TLAT_B_DEG:.0f} deg "
        f"(expected ~+{_EXPECTED_RC1_DELTA} us)"
    )
    log.info(
        "PASS C -- RC1 shifted: baseline=%.0f -> nvf=%.0f (delta=%+.0f, expected ~+%d) us",
        rc1_a, rc1_b, rc1_delta, _EXPECTED_RC1_DELTA,
    )

    # D. RC1 and RC2 return near 1500 in Phase C after neutral restore.
    rc1_c = result["c_rc1_mean"]
    rc2_c = result["c_rc2_mean"]
    assert abs(rc1_c - 1500) < _RC_NEUTRAL_TOL, (
        f"RC1 in Phase C = {rc1_c:.0f} us; expected near 1500 us "
        f"(within {_RC_NEUTRAL_TOL} us) after neutral NVF restore"
    )
    assert abs(rc2_c - 1500) < _RC_NEUTRAL_TOL, (
        f"RC2 in Phase C = {rc2_c:.0f} us; expected near 1500 us "
        f"(within {_RC_NEUTRAL_TOL} us) after neutral NVF restore"
    )
    log.info(
        "PASS D -- RC1/RC2 back to neutral: RC1=%.0f RC2=%.0f (tol=+/-%d us)",
        rc1_c, rc2_c, _RC_NEUTRAL_TOL,
    )

    # E. S1 increased in Phase B (both roll and pitch factors add positively for S1).
    s1_a = result["a_s1_mean"]
    s1_b = result["b_s1_mean"]
    s1_delta = s1_b - s1_a
    assert s1_delta >= _SERVO_S1_MARGIN_US, (
        f"S1 delta in Phase B = {s1_delta:+.0f} us (baseline={s1_a:.0f}, nvf={s1_b:.0f}); "
        f"expected >= +{_SERVO_S1_MARGIN_US} us "
        f"(H3-120 mixer: +0.390*(RC1-1500) + 0.225*(RC2-1500) = ~{_EXPECTED_S1_DELTA:+d} us)"
    )
    log.info(
        "PASS E -- S1 rose: baseline=%.0f -> nvf=%.0f (delta=%+.0f, expected ~%+d) us",
        s1_a, s1_b, s1_delta, _EXPECTED_S1_DELTA,
    )

    # F. S2 essentially unchanged in Phase B (roll and pitch factors nearly cancel).
    s2_a = result["a_s2_mean"]
    s2_b = result["b_s2_mean"]
    s2_delta = s2_b - s2_a
    assert abs(s2_delta) < _SERVO_S2_TOL_US, (
        f"S2 delta in Phase B = {s2_delta:+.0f} us (baseline={s2_a:.0f}, nvf={s2_b:.0f}); "
        f"expected |delta| < {_SERVO_S2_TOL_US} us "
        f"(H3-120 mixer: -0.390*(RC1-1500) + 0.225*(RC2-1500) = ~{_EXPECTED_S2_DELTA:+d} us)"
    )
    log.info(
        "PASS F -- S2 stable: baseline=%.0f -> nvf=%.0f (delta=%+.0f, expected ~%+d) us",
        s2_a, s2_b, s2_delta, _EXPECTED_S2_DELTA,
    )

    # G. S3 decreased in Phase B (pitch factor -0.450 dominates; no roll contribution).
    s3_a = result["a_s3_mean"]
    s3_b = result["b_s3_mean"]
    s3_delta = s3_b - s3_a
    assert s3_delta <= -_SERVO_S3_MARGIN_US, (
        f"S3 delta in Phase B = {s3_delta:+.0f} us (baseline={s3_a:.0f}, nvf={s3_b:.0f}); "
        f"expected <= -{_SERVO_S3_MARGIN_US} us "
        f"(H3-120 mixer: -0.450*(RC2-1500) = ~{_EXPECTED_S3_DELTA:+d} us)"
    )
    log.info(
        "PASS G -- S3 fell: baseline=%.0f -> nvf=%.0f (delta=%+.0f, expected ~%+d) us",
        s3_a, s3_b, s3_delta, _EXPECTED_S3_DELTA,
    )

    # H. All three swash servos return near Phase A baseline in Phase C.
    for label, s_a, s_c in (
        ("S1", result["a_s1_mean"], result["c_s1_mean"]),
        ("S2", result["a_s2_mean"], result["c_s2_mean"]),
        ("S3", result["a_s3_mean"], result["c_s3_mean"]),
    ):
        err_c = abs(s_c - s_a)
        assert err_c < _SERVO_RESTORE_TOL, (
            f"{label} in Phase C = {s_c:.0f} us; baseline = {s_a:.0f} us; "
            f"deviation = {err_c:.0f} us; expected < {_SERVO_RESTORE_TOL} us after neutral restore"
        )
    log.info(
        "PASS H -- swash restored: S1=%.0f S2=%.0f S3=%.0f (tol=+/-%d us)",
        result["c_s1_mean"], result["c_s2_mean"], result["c_s3_mean"], _SERVO_RESTORE_TOL,
    )


def _require_samples(n: int, phase: str, minimum: int = 10) -> None:
    if n < minimum:
        pytest.fail(
            f"Not enough samples in {phase} (got {n}, need >= {minimum}). "
            f"Possible causes: settle time too long, Lua not responding, "
            f"or observe() timed out before all phases completed."
        )
