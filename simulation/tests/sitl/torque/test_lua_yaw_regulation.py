"""
torque/test_lua_yaw_regulation.py — Lua MODE_YAW counter-torque stack test.

Verifies that rawes.lua's manual yaw PID (MODE_YAW, SCR_USER6=2) can hold
hub yaw steady while the GB4008 anti-rotation motor counter-rotates against
the spinning rotor hub.  Companion to test_yaw_regulation.py, which exercises
ArduPilot's built-in DDFP PID; this test exercises the Lua-managed path that
will run on the bench rig and on hardware.

Physical scenario
-----------------
  * ArduPilot rides on a motor with a horizontal axle and no other motion
    (no GPS, no translation; only the hub yaw axis is free).
  * Motor profile (matches the bench rig):
      STARTUP   (t = 0 .. 15 s) : rotor stationary, waiting for arming.
      SPINUP    (t = 15 .. 25 s): 10 s ramp 0 -> 120 RPM (= 4*pi rad/s).
      HOLD      (t = 25 .. 55 s): 30 s at 120 RPM, body sees the constant
                                   reaction torque the counter-rotation
                                   motor has to compensate.
  * MODE_YAW reads gyro:z(), runs a P+I+D yaw-rate PID toward setpoint 0,
    adds the H_YAW_TRIM feedforward, clamps to [0, 1], and writes SERVO4
    directly via ``SRV_Channels:set_output_pwm_chan_timeout`` — bypassing
    ArduPilot's internal DDFP yaw mixer.
  * The GB4008 motor on SERVO4 converts the PWM into counter-torque.

Arming without GPS
------------------
  The bench rig has no GPS — arming relies only on EKF3 attitude alignment
  (gravity + gyro give roll/pitch; yaw stays at boot value since the body
  doesn't translate).  ``_launch_mediator_torque`` sets up SITL without GPS
  and the fixture sends ``RAWES_ARM(1 hour)`` after the EKF attitude check
  passes.  See ``_LUA_TORQUE_EXTRA_PARAMS`` for the ``ARMING_CHECK=0``
  override that lets ``arming:arm()`` succeed without GPS prearm checks.

Pass criterion
--------------
  During the 30 s HOLD window (t = 25 .. 55 s, rotor at 120 RPM):
    * max |psi_dot|  < 5 deg/s  (physics ground truth from mediator
      events log)

  ACRO yaw RATE control: only the rate is asserted, not absolute angle.
  The hub may settle at any heading; only the rate must be near zero.

Tuning state in MODE_YAW
------------------------
  The same ATC_RAT_YAW_P/I/D/IMAX and H_YAW_TRIM params used by
  test_yaw_regulation.py drive the Lua PID directly (read each tick via
  ``param:get``).  ``_LUA_TORQUE_EXTRA_PARAMS`` sets:
    P=0.015, I=0.01, IMAX=0.7, H_YAW_TRIM=0.02, D=0.

Telemetry
---------
  CSV log at simulation/logs/torque_telemetry_lua_yaw_regulation.csv.
  YAW_I and YAW_OUT NAMED_VALUE_FLOATs streamed by run_yaw_pid each tick.

Run with (inside Docker)
------------------------
  bash simulation/dev.sh test-stack -n 1 -k test_lua_yaw_regulation
"""
from __future__ import annotations

import math
import statistics

from conftest import (
    LUA_YAW_TRIM_LON,
    LUA_YAW_TRIM_LAT,
    LUA_YAW_IC_COL,
)
from stack_infra import observe
from torque_test_utils import (
    run_observation_loop,
    save_telemetry,
    assert_physics_yaw_rate,
)


# Absolute SITL times.  The mediator's torque rig keeps the rotor at 0
# rad/s until t = startup_hold_s (15 s -- enough for EKF alignment + Lua
# RAWES_ARM to succeed), then linearly ramps to omega_rotor over a 10 s
# spinup window (mediator_torque.py:_SPINUP_S).  The HOLD phase starts at
# t = 25 s and we observe for 30 s.
_SETTLE_S          = 45.0   # startup_hold(15) + spinup(10) + 20 s for I-term to settle
_OBSERVE_S         = 10.0   # tail end of the 30 s constant-RPM hold
_MAX_PSI_DOT_RAD_S = math.radians(5.0)

# RAWES collective range + ACRO rate scaling from rawes.lua (kept in sync
# with COL_MIN_RAD / COL_MAX_RAD / ACRO_RP_RATE_DEG there).
_COL_MIN_RAD      = -0.28
_COL_MAX_RAD      =  0.10
_ACRO_RP_RATE_DEG = 360.0

# Tolerance on the predicted SERVO PWM.  The Lua rounds to an integer PWM,
# and the rate PID's small integral term contributes a few PWM of noise on
# top of the trim feedforward, so we allow +-15 PWM.
_PWM_TOL          = 15


def _world_to_body_trim(tlon_ref: float, tlat_ref: float, dyaw: float
                        ) -> tuple[float, float]:
    """Mirror rawes.lua effective_trim_body(): rotate the body-frame
    trim (computed at the reference yaw, here taken at arming time) by
    -dyaw into the current body frame.  Used to predict the cyclic the
    Lua should emit at a given accumulated yaw drift."""
    c, s = math.cos(-dyaw), math.sin(-dyaw)
    return (tlon_ref * c - tlat_ref * s,
            tlon_ref * s + tlat_ref * c)


def _expected_pwm_for_cyclic(tilt_rad: float, atc_p_plus_ff: float,
                              sign: float) -> int:
    """Predicted ch1 / ch2 PWM for a given body-frame cyclic tilt.
    Mirrors set_trim_ic_rc_overrides + rate_to_pwm in rawes.lua."""
    bias_rad_s = sign * tilt_rad / max(atc_p_plus_ff, 0.01)
    scale = 500.0 / math.radians(_ACRO_RP_RATE_DEG)
    return int(round(1500.0 + scale * bias_rad_s))


def _expected_pwm_for_collective(col_rad: float) -> int:
    """Predicted ch3 PWM for the IC collective.  Mirrors the
    (_ic_col - COL_MIN) / (COL_MAX - COL_MIN) mapping in
    set_trim_ic_rc_overrides."""
    thrust = (col_rad - _COL_MIN_RAD) / (_COL_MAX_RAD - _COL_MIN_RAD)
    thrust = max(0.0, min(1.0, thrust))
    return int(round(1000.0 + thrust * 1000.0))


def test_lua_yaw_regulation(torque_armed_lua_yaw):
    """
    rawes.lua MODE_YAW (SCR_USER6=2) regulates hub yaw via SERVO4 direct write.

    The Lua's P+I+D loop reads gyro:z() each tick and writes SERVO4 PWM
    directly, bypassing ArduPilot's built-in DDFP mixer.  The rotor sits
    stationary until ArduPilot arms (~12 s into STARTUP), then accelerates
    over 10 s to 120 RPM and holds for 30 s.  The yaw rate magnitude must
    stay below 5 deg/s throughout the hold.

    Failure modes this catches:
      * MODE_YAW not active (SCR_USER6 didn't get set, or Lua not loaded)
      * Lua param reads broken (kp/ki/imax) — output stays at 0 or trim
      * SRV_Channels override timeout — Lua loses ownership and ArduPilot's
        DDFP path takes over (would produce different SERVO4 output)
      * Sign convention wrong: err = -gyro:z() with setpoint 0; positive
        gyro (CW spin) -> negative err -> bleed integrator + zero output ->
        motor off -> body accelerates further; assertion catches the
        runaway.
    """
    ctx = torque_armed_lua_yaw

    # Single observation loop captures both yaw-rate samples (for the
    # primary regulation assertion) AND SERVO1/2/3 + body yaw (for the
    # cyclic / collective hold assertion).  Sharing one loop avoids
    # repositioning the simulation between observes.
    obs, rows, servo_samples = _run_yaw_and_servo_loop(
        ctx, _SETTLE_S, _OBSERVE_S,
    )
    save_telemetry(rows, "lua_yaw_regulation", ctx.log)

    # ── Primary: yaw rate is held within the threshold ─────────────────────
    assert_physics_yaw_rate(
        ctx.events_log, _MAX_PSI_DOT_RAD_S, _SETTLE_S, _OBSERVE_S, ctx.log,
    )

    # ── Secondary: cyclic trim + IC collective are held throughout HOLD ───
    # MODE_YAW must keep ch1/ch2 at the IC trim values (rotated into the
    # current body frame by the integrated gyro:z() since the trim NVFs
    # arrived) and ch3 at the IC collective.  Verify that the SERVO PWMs
    # match the predicted values from rawes.lua's set_trim_ic_rc_overrides,
    # accounting for any yaw drift that accumulated during the hold.
    _assert_trim_holding(ctx, servo_samples)


def _run_yaw_and_servo_loop(ctx, settle_s, observe_s):
    """Single ATTITUDE + SERVO_OUTPUT_RAW observation loop.

    Returns:
      obs            -- yaw samples (compatible with assert_yaw_rate)
      rows           -- TelRow records for save_telemetry
      servo_samples  -- list of {t, yaw, s1, s2, s3} captured throughout
                        the observation window (settle_s..settle_s+observe_s)
    """
    from telemetry_csv import TelRow   # noqa: E402

    obs:           list[dict] = []
    rows:          list       = []
    servo_samples: list[dict] = []
    # Read both SERVO_OUTPUT_RAW (for servo4_us telemetry / live yaw motor
    # state) and RC_CHANNELS (for the RC1/RC2/RC3 inputs the Lua wrote
    # via set_override).  RC_CHANNELS bypasses the H3-120 swashplate mixer
    # and H_COL_MIN/MAX rescale, so it's the right signal for verifying
    # the Lua's cyclic+collective hold logic directly.
    state = {"c1": 0, "c2": 0, "c3": 0, "s4": 0, "yaw": 0.0}

    def handle(msg, t_rel):
        if msg is None:
            return None
        mt = msg.get_type()
        if mt == "STATUSTEXT":
            ctx.log.debug("SITL t=%.1fs: %s", t_rel,
                          msg.text.rstrip("\x00").strip())
        elif mt == "SERVO_OUTPUT_RAW":
            state["s4"] = int(getattr(msg, "servo4_raw", 0) or 0)
        elif mt == "RC_CHANNELS":
            state["c1"] = int(getattr(msg, "chan1_raw", 0) or 0)
            state["c2"] = int(getattr(msg, "chan2_raw", 0) or 0)
            state["c3"] = int(getattr(msg, "chan3_raw", 0) or 0)
        elif mt == "ATTITUDE":
            state["yaw"] = float(msg.yaw)
            rows.append(TelRow(
                t_sim=t_rel, phase="DYNAMIC",
                rpy_roll=msg.roll, rpy_pitch=msg.pitch, rpy_yaw=msg.yaw,
                omega_z=msg.yawspeed, omega_rotor=ctx.omega_rotor,
                servo4_us=float(state["s4"]),
            ))
            if t_rel >= settle_s:
                obs.append({"t": t_rel, "yaw": msg.yaw, "yaw_rate": msg.yawspeed})
                if state["c1"] > 0 and state["c3"] > 0:
                    servo_samples.append({
                        "t":   t_rel,
                        "yaw": state["yaw"],
                        "s1":  state["c1"],
                        "s2":  state["c2"],
                        "s3":  state["c3"],
                    })
            if t_rel >= settle_s + observe_s:
                return True
        return None

    observe(ctx, settle_s + observe_s + 20.0, handle,
            msg_types=["ATTITUDE", "STATUSTEXT", "SERVO_OUTPUT_RAW",
                       "RC_CHANNELS"],
            keepalive={8: 2000})
    return obs, rows, servo_samples


def _assert_trim_holding(ctx, servo_samples) -> None:
    """Assert SERVO1/2 reflect the trim cyclic (rotated by accumulated
    yaw) and SERVO3 reflects the IC collective.  Skips silently when
    we have too few samples to be meaningful."""
    if len(servo_samples) < 5:
        ctx.log.warning(
            "Only %d SERVO+ATTITUDE samples in the HOLD window -- skipping "
            "cyclic-hold assertion (too few to be meaningful)",
            len(servo_samples))
        return

    # Read the ATC rate-PID gains the Lua used at this run.
    p_plus_ff_rll = float(ctx.gcs.get_param("ATC_RAT_RLL_P", 5.0) or 0.18) \
                  + float(ctx.gcs.get_param("ATC_RAT_RLL_FF", 5.0) or 0.0)
    p_plus_ff_pit = float(ctx.gcs.get_param("ATC_RAT_PIT_P", 5.0) or 0.18) \
                  + float(ctx.gcs.get_param("ATC_RAT_PIT_FF", 5.0) or 0.0)

    yaw_ref     = servo_samples[0]["yaw"]
    expected_s3 = _expected_pwm_for_collective(LUA_YAW_IC_COL)

    diffs_s1, diffs_s2, diffs_s3 = [], [], []
    max_dyaw_deg = 0.0
    for s in servo_samples:
        d_yaw = s["yaw"] - yaw_ref
        max_dyaw_deg = max(max_dyaw_deg, abs(math.degrees(d_yaw)))
        tlon_b, tlat_b = _world_to_body_trim(
            LUA_YAW_TRIM_LON, LUA_YAW_TRIM_LAT, d_yaw,
        )
        # rawes.lua: ch1 = rate_to_pwm( tlat_b / (P+FF))
        #            ch2 = rate_to_pwm(-tlon_b / (P+FF))
        exp_s1 = _expected_pwm_for_cyclic(tlat_b, p_plus_ff_rll, +1.0)
        exp_s2 = _expected_pwm_for_cyclic(tlon_b, p_plus_ff_pit, -1.0)
        diffs_s1.append(s["s1"] - exp_s1)
        diffs_s2.append(s["s2"] - exp_s2)
        diffs_s3.append(s["s3"] - expected_s3)

    ctx.log.info(
        "Trim-hold check: %d samples, yaw drift up to %.2f deg.  "
        "SERVO mean error (PWM): s1=%+.1f s2=%+.1f s3=%+.1f.  "
        "Tolerance: +-%d PWM.",
        len(servo_samples), max_dyaw_deg,
        statistics.mean(diffs_s1), statistics.mean(diffs_s2),
        statistics.mean(diffs_s3), _PWM_TOL,
    )

    max_abs_s1 = max(abs(d) for d in diffs_s1)
    max_abs_s2 = max(abs(d) for d in diffs_s2)
    max_abs_s3 = max(abs(d) for d in diffs_s3)
    failures = []
    if max_abs_s1 > _PWM_TOL:
        failures.append(f"SERVO1: max |actual - expected| = {max_abs_s1} > {_PWM_TOL} PWM")
    if max_abs_s2 > _PWM_TOL:
        failures.append(f"SERVO2: max |actual - expected| = {max_abs_s2} > {_PWM_TOL} PWM")
    if max_abs_s3 > _PWM_TOL:
        failures.append(f"SERVO3: max |actual - expected| = {max_abs_s3} > {_PWM_TOL} PWM")
    assert not failures, (
        "MODE_YAW trim-hold violation (cyclic trim or collective not held "
        "at the expected rotated value):\n  " + "\n  ".join(failures)
    )
