"""
torque/test_yaw_regulation.py — Counter-torque motor stack test.

Verifies that ArduPilot SITL can hold hub yaw steady while the GB4008
anti-rotation motor counter-rotates against the spinning rotor hub.

Physical scenario
-----------------
  * Rotor hub spins at ~28 rad/s (nominal RAWES autorotation at 10 m/s wind)
  * The motor counter-rotates via the 80:44 gear to maintain inner assembly heading
  * Bearing and swashplate friction is the load the motor works against
  * ArduPilot (heli frame, ACRO mode) senses the yaw rate via gyro and
    commands Ch4 (H_TAIL_TYPE=0, servo) to control GB4008 motor speed

Pass criterion
--------------
  After 30 s of ACRO with neutral sticks:
    * max |psi_dot|  < 1 deg/s over the last 20 s  (yaw rate nearly zero)

  Note: ACRO mode controls yaw RATE (not angle), so the hub may settle at a
  non-zero yaw angle.  Only the rate is asserted.  The yaw angle represents
  the hub's operating heading and has no operational significance.

  A well-tuned yaw PID should do much better than 1 deg/s.  The point is to
  confirm the control loop closes, not to verify final PID tuning.

Telemetry
---------
  The test writes a CSV log to simulation/logs/torque_telemetry.csv
  after each run.  Load and play back with:
      python simulation/torque/visualize_torque.py
             simulation/logs/torque_telemetry.csv

Run with (inside Docker)
------------------------
  RAWES_RUN_STACK_INTEGRATION=1 pytest simulation/torque/test_yaw_regulation.py -v
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import pytest

_SIM_DIR = Path(__file__).resolve().parents[3]
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from telemetry_csv import TelRow, write_csv

# -- Thresholds ---------------------------------------------------------------

_SETTLE_S = 40.0
_OBSERVE_S = 20.0
_MAX_PSI_DOT_RAD_S = math.radians(1.0)   # [rad/s]
_TEST_TIMEOUT_S = _SETTLE_S + _OBSERVE_S + 20.0

_TELEMETRY_OUT = _SIM_DIR / "logs" / "torque_telemetry.csv"


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_yaw_regulation(torque_armed):
    """
    ArduPilot SITL regulates hub yaw using the tail-rotor (Ch4) output.

    ACRO mode with neutral sticks commands psi_dot = 0.  The yaw rate PID must
    build enough Ch4 output to maintain counter-rotation against the spinning
    axle and hold |psi_dot| < 1 deg/s after a 30 s settle period.

    Telemetry is saved to simulation/logs/torque_telemetry.csv for playback
    with visualize_torque.py.
    """
    ctx = torque_armed
    gcs = ctx.gcs
    log = ctx.log

    rows: list = []

    log.info(
        "test_yaw_regulation: settle=%.0f s  observe=%.0f s  "
        "threshold psi_dot<%.1f deg/s",
        _SETTLE_S, _OBSERVE_S, math.degrees(_MAX_PSI_DOT_RAD_S),
    )

    # -- Collect ATTITUDE messages --------------------------------------------
    observe_samples: list[dict] = []
    t_start   = time.monotonic()
    deadline  = t_start + _TEST_TIMEOUT_S
    t_last_rc = time.monotonic()

    while time.monotonic() < deadline:
        if time.monotonic() - t_last_rc >= 0.5:
            ctx.gcs.send_rc_override({8: 2000})
            t_last_rc = time.monotonic()

        for name, proc in [("mediator", ctx.mediator_proc), ("SITL", ctx.sitl_proc)]:
            if proc.poll() is not None:
                write_csv(rows, _TELEMETRY_OUT)
                pytest.fail(f"{name} exited during test (rc={proc.returncode})")

        msg = gcs._mav.recv_match(
            type=["ATTITUDE", "STATUSTEXT"],
            blocking=True,
            timeout=0.5,
        )
        if msg is None:
            continue

        t_rel = time.monotonic() - t_start

        if msg.get_type() == "STATUSTEXT":
            log.debug("SITL t=%.1fs: %s", t_rel, msg.text.rstrip("\x00").strip())
            continue

        if msg.get_type() == "ATTITUDE":
            rows.append(TelRow(
                t_sim       = t_rel,
                phase       = "DYNAMIC",
                rpy_roll    = msg.roll,
                rpy_pitch   = msg.pitch,
                rpy_yaw     = msg.yaw,
                omega_z     = msg.yawspeed,
                omega_rotor = ctx.omega_rotor,
            ))

            if t_rel >= _SETTLE_S:
                observe_samples.append({
                    "t":        t_rel,
                    "yaw":      msg.yaw,        # NED [rad]
                    "yaw_rate": msg.yawspeed,   # NED [rad/s]
                })

            if len(observe_samples) % 50 == 0 and observe_samples:
                log.info(
                    "t=%6.1f s  psi=%+7.2f deg  psi_dot=%+6.2f deg/s",
                    t_rel, math.degrees(msg.yaw), math.degrees(msg.yawspeed),
                )

            if t_rel >= _SETTLE_S + _OBSERVE_S:
                break

    # -- Evaluate observation window ------------------------------------------
    if len(observe_samples) < 10:
        write_csv(rows, _TELEMETRY_OUT)
        pytest.fail(
            f"Not enough ATTITUDE samples in observation window "
            f"(got {len(observe_samples)}, need >= 10).  "
            f"Total frames recorded: {len(rows)}"
        )

    max_psi_dot = max(abs(s["yaw_rate"]) for s in observe_samples)
    max_psi     = max(abs(s["yaw"])      for s in observe_samples)
    passed = max_psi_dot <= _MAX_PSI_DOT_RAD_S

    log.info(
        "Observation window (t > %.0f s, %d samples):  "
        "max |psi_dot|=%.2f deg/s (limit %.1f deg/s)  "
        "max |psi|=%.2f deg (informational, not asserted)",
        _SETTLE_S, len(observe_samples),
        math.degrees(max_psi_dot), math.degrees(_MAX_PSI_DOT_RAD_S),
        math.degrees(max_psi),
    )

    # -- Save telemetry -------------------------------------------------------
    write_csv(rows, _TELEMETRY_OUT)
    log.info("Telemetry saved -> %s  (%d frames)", _TELEMETRY_OUT, len(rows))
    log.info(
        "Visualise with:  python simulation/torque/visualize_torque.py %s",
        _TELEMETRY_OUT,
    )

    if not passed:
        if len(observe_samples) >= 2:
            rate_first = observe_samples[0]["yaw_rate"]
            rate_last  = observe_samples[-1]["yaw_rate"]
            if abs(rate_last) > abs(rate_first) + math.radians(5.0):
                log.warning(
                    "Yaw rate growing (%+.1f -> %+.1f deg/s) -- motor may be in wrong "
                    "direction.  Try H_TAIL_TYPE=2 (CW) or 3 (CCW) in conftest.py.",
                    math.degrees(rate_first), math.degrees(rate_last),
                )

    assert passed, (
        f"Max |psi_dot| = {math.degrees(max_psi_dot):.2f} deg/s exceeded limit "
        f"{math.degrees(_MAX_PSI_DOT_RAD_S):.1f} deg/s "
        f"in observation window (t > {_SETTLE_S:.0f} s)"
    )

    log.info("PASS -- yaw held within limits for %.0f s observation window", _OBSERVE_S)
