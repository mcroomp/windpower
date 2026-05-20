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

    _, rows = run_observation_loop(ctx, _SETTLE_S, _OBSERVE_S)

    save_telemetry(rows, "lua_yaw_regulation", ctx.log)
    assert_physics_yaw_rate(
        ctx.events_log, _MAX_PSI_DOT_RAD_S, _SETTLE_S, _OBSERVE_S, ctx.log,
    )
