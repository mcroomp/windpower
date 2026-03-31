"""
torque/torque_test_utils.py — Shared base utilities for all torque stack tests.

All torque stack tests follow the same structure:
  1. Acquire the ``torque_armed_profile`` fixture (ACRO-armed, profile running)
  2. Call ``run_observation_loop()`` to collect ATTITUDE samples + telemetry
  3. Assert pass criteria on the returned samples

This module provides:
  run_observation_loop()  — drives the MAVLink receive loop, keeps CH8 alive,
                            records telemetry, returns the observation samples
  assert_yaw_rate()       — standard assertion with diagnostic logging
"""
from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional

import pytest

from torque_telemetry import TorqueTelemetryRecorder

_SIM_DIR = Path(__file__).resolve().parents[1]


def run_observation_loop(
    ctx,
    rec: TorqueTelemetryRecorder,
    settle_s: float,
    observe_s: float,
    timeout_s: float,
    log_interval_samples: int = 50,
) -> list[dict]:
    """
    Collect ATTITUDE samples from ArduPilot SITL for the full test duration.

    Keeps the motor interlock (CH8=2000) alive every 0.5 s.
    Records every received ATTITUDE frame into ``rec``.
    Returns only the samples that fall within the observation window
    (t >= settle_s … settle_s + observe_s).

    Parameters
    ----------
    ctx              : TorqueStackContext from torque_armed_profile fixture
    rec              : TorqueTelemetryRecorder already initialised by the test
    settle_s         : seconds to let the system settle before observing
    observe_s        : observation window duration [s]
    timeout_s        : hard deadline [s] (should be > settle_s + observe_s)
    log_interval_samples : log every N observation samples (for progress)

    Returns
    -------
    list of {"t", "yaw_deg", "yaw_rate_degs"} dicts covering the window.
    Empty list if test timed out before completing the window.
    """
    gcs      = ctx.gcs
    log      = ctx.log
    obs      = []
    t_start  = time.monotonic()
    deadline = t_start + timeout_s
    t_rc     = time.monotonic()

    # Request SERVO_OUTPUT_RAW so we can record the actual PWM ArduPilot sends.
    # SERVO_OUTPUT_RAW is in MAV_DATA_STREAM_RC_CHANNELS in ArduPilot helicopter.
    try:
        from pymavlink import mavutil as _mavu
        gcs._mav.mav.request_data_stream_send(
            gcs._target_system, gcs._target_component,
            _mavu.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1,
        )
    except Exception:
        pass

    latest_servo_pwm: int = 0   # most recent Ch9 (index 8) or Ch4 (index 3) PWM

    while time.monotonic() < deadline:
        # Keep motor interlock HIGH — ArduPilot expires RC override after ~1 s
        if time.monotonic() - t_rc >= 0.5:
            ctx.gcs.send_rc_override({8: 2000})
            t_rc = time.monotonic()

        # Watch for crashed processes
        for name, proc in [("mediator", ctx.mediator_proc),
                            ("SITL",    ctx.sitl_proc)]:
            if proc.poll() is not None:
                rec.add_meta("result", "process_crash")
                return obs

        msg = gcs._mav.recv_match(
            type=["ATTITUDE", "STATUSTEXT", "SERVO_OUTPUT_RAW"],
            blocking=True,
            timeout=0.5,
        )
        if msg is None:
            continue

        t_rel = time.monotonic() - t_start

        if msg.get_type() == "STATUSTEXT":
            log.debug("SITL t=%.1fs: %s", t_rel, msg.text.rstrip("\x00").strip())
            continue

        if msg.get_type() == "SERVO_OUTPUT_RAW":
            # Ch9 (index 8, servo9_raw) for Lua test; Ch4 (index 3, servo4_raw)
            # for standard tests. Pick whichever is non-neutral (>1050 µs).
            ch9 = getattr(msg, "servo9_raw", 0) or 0
            ch4 = getattr(msg, "servo4_raw", 0) or 0
            # Use Ch9 if Lua is active (pwm >1050), otherwise Ch4
            latest_servo_pwm = ch9 if ch9 > 1050 else ch4
            continue

        if msg.get_type() == "ATTITUDE":
            # Negate yaw (NED→ENU); roll/pitch use same sign (symmetric axes)
            yaw_deg       = -math.degrees(msg.yaw)
            yaw_rate_degs = -math.degrees(msg.yawspeed)
            roll_deg      =  math.degrees(msg.roll)
            pitch_deg     =  math.degrees(msg.pitch)

            rec.record(
                t               = t_rel,
                psi_deg         = yaw_deg,
                psi_dot_degs    = yaw_rate_degs,
                omega_axle_rads = ctx.omega_axle,
                phase           = "DYNAMIC",
                roll_deg        = roll_deg,
                pitch_deg       = pitch_deg,
                servo_pwm_us    = latest_servo_pwm,
            )

            if t_rel >= settle_s:
                obs.append({
                    "t":             t_rel,
                    "yaw_deg":       yaw_deg,
                    "yaw_rate_degs": yaw_rate_degs,
                })

            if obs and len(obs) % log_interval_samples == 0:
                log.info("t=%6.1f s  ψ=%+7.2f°  ψ_dot=%+6.2f°/s",
                         t_rel, yaw_deg, yaw_rate_degs)

            if t_rel >= settle_s + observe_s:
                break

    return obs


def save_telemetry(rec: TorqueTelemetryRecorder,
                   profile: str,
                   log,
                   suffix: str = "") -> None:
    """Save telemetry JSON to simulation/logs/torque_telemetry_<profile>.json."""
    stem = f"torque_telemetry_{profile}{suffix}"
    path = _SIM_DIR / "logs" / f"{stem}.json"
    out  = rec.save(path)
    log.info("Telemetry saved → %s  (%d frames)", out, len(rec.frames))
    log.info("Visualise with:  python simulation/torque/visualize_torque.py %s", out)


def assert_yaw_rate(
    obs: list[dict],
    threshold_degs: float,
    settle_s: float,
    rec: TorqueTelemetryRecorder,
    log,
) -> None:
    """
    Assert that max |ψ_dot| in the observation window is within threshold.
    Attaches PASS/FAIL to rec metadata before asserting.
    """
    if len(obs) < 10:
        rec.add_meta("result", "insufficient_samples")
        pytest.fail(
            f"Not enough ATTITUDE samples in observation window "
            f"(got {len(obs)}, need ≥ 10)"
        )

    max_rate = max(abs(s["yaw_rate_degs"]) for s in obs)
    max_psi  = max(abs(s["yaw_deg"])       for s in obs)
    passed   = max_rate <= threshold_degs

    rec.add_meta("result",           "PASS" if passed else "FAIL")
    rec.add_meta("max_psi_dot_degs", max_rate)
    rec.add_meta("max_psi_deg",      max_psi)
    rec.add_meta("n_obs_samples",    len(obs))

    log.info(
        "Observation window (t > %.0f s, %d samples):  "
        "max |ψ_dot|=%.2f°/s (limit %.1f°/s)  "
        "max |ψ|=%.2f° (informational)",
        settle_s, len(obs), max_rate, threshold_degs, max_psi,
    )

    assert passed, (
        f"Max |ψ_dot| = {max_rate:.2f}°/s exceeded limit {threshold_degs}°/s "
        f"in observation window (t > {settle_s:.0f} s)"
    )
    log.info("PASS — yaw rate held within %.1f°/s limit", threshold_degs)
