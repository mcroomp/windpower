"""
tests/sitl/torque/torque_test_utils.py — Shared base utilities for all torque stack tests.

All torque stack tests follow the same structure:
  1. Acquire the ``torque_armed_profile`` fixture (ACRO-armed, profile running)
  2. Call ``run_observation_loop()`` to collect ATTITUDE samples + telemetry
  3. Assert pass criteria on the returned samples

This module provides:
  run_observation_loop()      — drives the MAVLink receive loop, keeps CH8 alive;
                                returns (obs, rows): obs = observation-window dicts,
                                rows = all TelRow records for save_telemetry()
  save_telemetry()            — writes accumulated TelRow list to CSV
  assert_yaw_rate()           — assertion on ArduPilot EKF yawspeed (ATTITUDE messages)
  assert_physics_yaw_rate()   — assertion on actual physics psi_dot from mediator log
"""
from __future__ import annotations

import math
from pathlib import Path

import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]  # tests/sitl/torque/ -> simulation/
_SITL_DIR = Path(__file__).resolve().parents[1]  # tests/sitl/torque/ -> tests/sitl/

import sys as _sys
for _p in (str(_SIM_DIR), str(_SITL_DIR)):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)

from telemetry_csv import TelRow, write_csv
from mediator_events import MediatorEventLog
from simtest_log import BadEventLog
from stack_infra import observe  # noqa: E402


def run_observation_loop(
    ctx,
    settle_s: float,
    observe_s: float,
    timeout_margin_s: float = 20.0,
    log_interval_samples: int = 50,
) -> tuple[list[dict], list]:
    """
    Collect ATTITUDE samples for the full test duration using the shared observe() loop.

    Keeps CH8=2000 alive every 0.5 sim-seconds (motor interlock).
    Returns (obs, rows):
      obs  — samples in the observation window (settle_s .. settle_s+observe_s)
      rows — all TelRow records collected throughout the run (for save_telemetry)
    """
    obs:  list[dict] = []
    rows: list       = []
    pwm = [0]  # latest Ch9/Ch4 PWM, captured by closure
    log = ctx.log

    def handle(msg, t_rel):
        if msg is None:
            return None
        mt = msg.get_type()
        if mt == "STATUSTEXT":
            log.debug("SITL t=%.1fs: %s", t_rel, msg.text.rstrip("\x00").strip())
        elif mt == "SERVO_OUTPUT_RAW":
            ch9 = getattr(msg, "servo9_raw", 0) or 0
            ch4 = getattr(msg, "servo4_raw", 0) or 0
            pwm[0] = ch9 if ch9 > 1050 else ch4
        elif mt == "ATTITUDE":
            rows.append(TelRow(
                t_sim=t_rel, phase="DYNAMIC",
                rpy_roll=msg.roll, rpy_pitch=msg.pitch, rpy_yaw=msg.yaw,
                omega_z=msg.yawspeed, omega_rotor=ctx.omega_rotor,
                servo4_us=float(pwm[0]),
            ))
            if t_rel >= settle_s:
                obs.append({"t": t_rel, "yaw": msg.yaw, "yaw_rate": msg.yawspeed})
                if len(obs) % log_interval_samples == 0:
                    log.info("t=%6.1f s  psi=%+7.2f deg  psi_dot=%+6.2f deg/s  pwm=%d",
                             t_rel, math.degrees(msg.yaw), math.degrees(msg.yawspeed), pwm[0])
            if t_rel >= settle_s + observe_s:
                return True
        return None

    observe(ctx, settle_s + observe_s + timeout_margin_s, handle,
            msg_types=["ATTITUDE", "STATUSTEXT", "SERVO_OUTPUT_RAW"],
            keepalive={8: 2000})
    return obs, rows


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
    settle_s and observe_s are measured from DYNAMIC start (the dynamics_start
    event), so the window is independent of startup_hold_s.

    Returns a list of {"t": float [s dynamics], "psi_dot": float [rad/s]}.
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

    Violations are collected into a BadEventLog so all offending samples are
    reported together rather than stopping at the first breach.

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

    bad = BadEventLog()
    for s in samples:
        if abs(s["psi_dot"]) > threshold_rad_s:
            bad.record(
                "yaw_violation", t=s["t"], phase="DYNAMIC", alt=0.0,
                psi_dot_deg_s=round(math.degrees(s["psi_dot"]), 2),
            )

    max_rate = max(abs(s["psi_dot"]) for s in samples)
    log.info(
        "Physics window (t=%.0f-%.0f s, %d samples):  "
        "max |psi_dot|=%.2f deg/s (limit %.1f deg/s)  violations=%d",
        settle_s, settle_s + observe_s, len(samples),
        math.degrees(max_rate), math.degrees(threshold_rad_s),
        len(bad.of_kind("yaw_violation")),
    )

    assert not bad, (
        f"Physics psi_dot exceeded {math.degrees(threshold_rad_s):.1f} deg/s limit "
        f"in window t={settle_s:.0f}-{settle_s+observe_s:.0f} s: {bad.summary()}"
    )
    log.info("PASS -- physics yaw rate held within %.1f deg/s limit",
             math.degrees(threshold_rad_s))


# ---------------------------------------------------------------------------
# Motor throttle response assertion (reads throttle from mediator heartbeats)
# ---------------------------------------------------------------------------

def assert_motor_throttle_response(
    events_log: MediatorEventLog,
    min_throttle: float,
    settle_s: float,
    observe_s: float,
    log,
) -> None:
    """
    Assert that the motor throttle exceeds min_throttle in the observation window.

    Used for prescribed-yaw tests where psi_dot is non-zero by design, so the
    normal assert_physics_yaw_rate would fail.  Instead, we check that the
    DDFP motor responds to the imposed yaw by increasing throttle above the
    equilibrium (zero-yaw) value.

    Parameters
    ----------
    events_log      : MediatorEventLog (available as ctx.events_log)
    min_throttle    : minimum required motor throttle [0, 1] (e.g. 0.05)
    settle_s        : start of observation window [s dynamics time]
    observe_s       : length of observation window [s]
    log             : test logger
    """
    samples: list[dict] = []
    for ev in events_log.get_events("heartbeat"):
        if ev.get("phase") != "DYNAMIC":
            continue
        t = float(ev.get("t_sim", 0.0))
        if settle_s <= t <= settle_s + observe_s:
            throttle = float(ev.get("throttle", 0.0))
            psi_dot  = float(ev.get("psi_dot_deg_s", 0.0))
            samples.append({"t": t, "throttle": throttle, "psi_dot_deg_s": psi_dot})

    if len(samples) < 3:
        pytest.fail(
            f"Not enough heartbeat samples in events log "
            f"(got {len(samples)}, need >= 3, window {settle_s:.0f}-{settle_s+observe_s:.0f} s). "
            f"Log: {events_log.path}"
        )

    max_throttle = max(s["throttle"] for s in samples)
    passed = max_throttle >= min_throttle

    log.info(
        "Throttle window (t=%.0f-%.0f s, %d samples):  "
        "max throttle=%.3f (need >= %.3f)  "
        "psi_dot range=[%.1f, %.1f] deg/s",
        settle_s, settle_s + observe_s, len(samples),
        max_throttle, min_throttle,
        min(s["psi_dot_deg_s"] for s in samples),
        max(s["psi_dot_deg_s"] for s in samples),
    )

    assert passed, (
        f"Motor throttle max={max_throttle:.3f} did not reach "
        f"min_throttle={min_throttle:.3f} "
        f"in window t={settle_s:.0f}-{settle_s+observe_s:.0f} s"
    )
    log.info("PASS -- motor throttle reached %.3f (>= %.3f limit)", max_throttle, min_throttle)
