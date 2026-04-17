"""
tests/sitl/torque/torque_test_utils.py — Shared base utilities for all torque stack tests.

All torque stack tests follow the same structure:
  1. Acquire the ``torque_armed_profile`` fixture (ACRO-armed, profile running)
  2. Call ``run_observation_loop()`` to collect ATTITUDE samples + telemetry
  3. Assert pass criteria on the returned samples

This module provides:
  run_observation_loop()      — drives the MAVLink receive loop, keeps CH8 alive,
                                records telemetry rows, returns the observation samples
  save_telemetry()            — writes accumulated TelRow list to CSV
  assert_yaw_rate()           — assertion on ArduPilot EKF yawspeed (ATTITUDE messages)
  assert_physics_yaw_rate()   — assertion on actual physics psi_dot from mediator log
"""
from __future__ import annotations

import math
import time
from pathlib import Path

import pytest

_SIM_DIR = Path(__file__).resolve().parents[3]  # tests/sitl/torque/ -> simulation/

# telemetry_csv lives in simulation/
import sys as _sys
if str(_SIM_DIR) not in _sys.path:
    _sys.path.insert(0, str(_SIM_DIR))

from telemetry_csv import TelRow, write_csv
from mediator_events import MediatorEventLog


def run_observation_loop(
    ctx,
    rows: list,
    settle_s: float,
    observe_s: float,
    timeout_s: float,
    log_interval_samples: int = 50,
) -> list[dict]:
    """
    Collect ATTITUDE samples from ArduPilot SITL for the full test duration.

    Keeps the motor interlock (CH8=2000) alive every 0.5 s.
    Appends one TelRow per received ATTITUDE frame to ``rows``.
    Returns only the samples that fall within the observation window
    (t >= settle_s ... settle_s + observe_s).

    Parameters
    ----------
    ctx              : TorqueStackContext from torque_armed_profile fixture
    rows             : list to append TelRow objects to (modified in-place)
    settle_s         : seconds to let the system settle before observing
    observe_s        : observation window duration [s]
    timeout_s        : hard deadline [s] (should be > settle_s + observe_s)
    log_interval_samples : log every N observation samples (for progress)

    Returns
    -------
    list of {"t", "yaw", "yaw_rate"} dicts (NED radians) covering the window.
    Empty list if test timed out before completing the window.
    """
    gcs      = ctx.gcs
    log      = ctx.log
    obs      = []
    t_start  = time.monotonic()
    deadline = t_start + timeout_s
    t_rc     = time.monotonic()

    # Request SERVO_OUTPUT_RAW so we can record the actual PWM ArduPilot sends.
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
        # Keep motor interlock HIGH
        if time.monotonic() - t_rc >= 0.5:
            ctx.gcs.send_rc_override({8: 2000})
            t_rc = time.monotonic()

        # Watch for crashed processes
        for name, proc in [("mediator", ctx.mediator_proc),
                            ("SITL",    ctx.sitl_proc)]:
            if proc.poll() is not None:
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
            ch9 = getattr(msg, "servo9_raw", 0) or 0
            ch4 = getattr(msg, "servo4_raw", 0) or 0
            latest_servo_pwm = ch9 if ch9 > 1050 else ch4
            continue

        if msg.get_type() == "ATTITUDE":
            # Store NED radians in TelRow and obs — consistent with physics codebase
            rows.append(TelRow(
                t_sim       = t_rel,
                phase       = "DYNAMIC",
                rpy_roll    = msg.roll,
                rpy_pitch   = msg.pitch,
                rpy_yaw     = msg.yaw,
                omega_z     = msg.yawspeed,
                omega_rotor = ctx.omega_rotor,
                servo_esc   = float(latest_servo_pwm),
            ))

            if t_rel >= settle_s:
                obs.append({
                    "t":        t_rel,
                    "yaw":      msg.yaw,        # NED [rad]
                    "yaw_rate": msg.yawspeed,   # NED [rad/s]
                })

            if obs and len(obs) % log_interval_samples == 0:
                log.info("t=%6.1f s  psi=%+7.2f deg  psi_dot=%+6.2f deg/s",
                         t_rel, math.degrees(msg.yaw), math.degrees(msg.yawspeed))

            if t_rel >= settle_s + observe_s:
                break

    return obs


def save_telemetry(rows: list,
                   profile: str,
                   log,
                   suffix: str = "") -> None:
    """Save telemetry CSV to simulation/logs/torque_telemetry_<profile>.csv."""
    stem = f"torque_telemetry_{profile}{suffix}"
    path = _SIM_DIR / "logs" / f"{stem}.csv"
    write_csv(rows, path)
    log.info("Telemetry saved -> %s  (%d frames)", path, len(rows))
    log.info("Visualise with:  python simulation/viz3d/visualize_torque.py %s", path)


def assert_yaw_rate(
    obs: list[dict],
    threshold_rad_s: float,
    settle_s: float,
    log,
) -> None:
    """
    Assert that max |psi_dot| in the observation window is within threshold.

    Parameters
    ----------
    obs              : list of {"t", "yaw", "yaw_rate"} dicts (NED, radians)
    threshold_rad_s  : max allowed |yaw_rate| [rad/s]
    settle_s         : settle time used in log message
    """
    if len(obs) < 10:
        pytest.fail(
            f"Not enough ATTITUDE samples in observation window "
            f"(got {len(obs)}, need >= 10)"
        )

    max_rate = max(abs(s["yaw_rate"]) for s in obs)
    max_psi  = max(abs(s["yaw"])      for s in obs)
    passed   = max_rate <= threshold_rad_s

    log.info(
        "Observation window (t > %.0f s, %d samples):  "
        "max |psi_dot|=%.2f deg/s (limit %.1f deg/s)  "
        "max |psi|=%.2f deg (informational)",
        settle_s, len(obs),
        math.degrees(max_rate), math.degrees(threshold_rad_s),
        math.degrees(max_psi),
    )

    assert passed, (
        f"Max |psi_dot| = {math.degrees(max_rate):.2f} deg/s exceeded "
        f"limit {math.degrees(threshold_rad_s):.1f} deg/s "
        f"in observation window (t > {settle_s:.0f} s)"
    )
    log.info("PASS -- yaw rate held within %.1f deg/s limit",
             math.degrees(threshold_rad_s))


# ---------------------------------------------------------------------------
# Physics assertion (reads actual psi_dot from mediator log)
# ---------------------------------------------------------------------------

def read_physics_psi_dot(
    events_log: MediatorEventLog,
    settle_s: float,
    observe_s: float,
) -> list[dict]:
    """
    Extract physics psi_dot samples from the mediator events log.

    Reads heartbeat events with phase=="DYNAMIC" in the observation window.

    Returns a list of {"t": float [s], "psi_dot": float [rad/s]} for samples
    where settle_s <= t <= settle_s + observe_s.
    """
    samples: list[dict] = []
    for ev in events_log.get_events("heartbeat"):
        if ev.get("phase") != "DYNAMIC":
            continue
        t = float(ev.get("t_sim", 0.0))
        if settle_s <= t <= settle_s + observe_s:
            psi_dot_deg_s = float(ev.get("psi_dot_deg_s", 0.0))
            samples.append({"t": t, "psi_dot": math.radians(psi_dot_deg_s)})
    return samples


def assert_physics_yaw_rate(
    events_log: MediatorEventLog,
    threshold_rad_s: float,
    settle_s: float,
    observe_s: float,
    log,
) -> None:
    """
    Assert that the actual physics psi_dot stays within threshold.

    Unlike assert_yaw_rate (which reads ArduPilot EKF ATTITUDE.yawspeed and can
    be inflated by compass-tilt artefacts), this reads the physics simulation
    state directly — it is the ground truth.

    Parameters
    ----------
    events_log      : MediatorEventLog (available as ctx.events_log)
    threshold_rad_s : max allowed |psi_dot| [rad/s]
    settle_s        : start of observation window [s simulation time]
    observe_s       : length of observation window [s]
    log             : test logger
    """
    samples = read_physics_psi_dot(events_log, settle_s, observe_s)

    if len(samples) < 3:
        pytest.fail(
            f"Not enough physics psi_dot samples in events log "
            f"(got {len(samples)}, need >= 3, window {settle_s:.0f}-{settle_s+observe_s:.0f} s). "
            f"Log: {events_log.path}"
        )

    max_rate = max(abs(s["psi_dot"]) for s in samples)
    passed   = max_rate <= threshold_rad_s

    log.info(
        "Physics window (t=%.0f-%.0f s, %d samples):  "
        "max |psi_dot|=%.2f deg/s (limit %.1f deg/s)",
        settle_s, settle_s + observe_s, len(samples),
        math.degrees(max_rate), math.degrees(threshold_rad_s),
    )

    assert passed, (
        f"Physics max |psi_dot| = {math.degrees(max_rate):.2f} deg/s exceeded "
        f"limit {math.degrees(threshold_rad_s):.1f} deg/s "
        f"in window t={settle_s:.0f}-{settle_s+observe_s:.0f} s"
    )
    log.info("PASS -- physics yaw rate held within %.1f deg/s limit",
             math.degrees(threshold_rad_s))
