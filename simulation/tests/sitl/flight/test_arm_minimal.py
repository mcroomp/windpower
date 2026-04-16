"""
test_arm_minimal.py — Minimal ArduCopter SITL arm test.

No mediator, no dynamics, no simulation dependencies.
Sends hard-coded sensor values directly to the SITL JSON backend via
SITLInterface (lockstep) and attempts to arm.

Purpose
-------
Isolate the arm problem from everything else.  If this test passes the arm
sequence works and any failure in the full stack is a mediator/sensor issue.
If this test fails, the problem is in our GCS arm sequence or ArduPilot params.

Uses _sitl_stack for all SITL lifecycle, boot params, and log management.
The sensor worker and arm sequence are the only test-specific logic here.

Sensor packet (NED body-frame, level hover at 50 m)
---------------------------------------------------
  pos       — [0, 0, -50]  NED, 50 m above origin
  vel       — [0.1, 0, 0]  tiny N velocity for EKF yaw derivation
  attitude  — [0, 0, 0]    roll=0, pitch=0, yaw=0
  gyro      — [0, 0, 0]    no rotation
  accel_body — [0, 0, -9.81]  gravity reaction in NED body frame

Usage
-----
  bash simulation/dev.sh test-stack -v -k test_arm_minimal
"""

import logging
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import StackConfig, _sitl_stack
from gcs import RawesGCS
from sitl_interface import SITLInterface

log = logging.getLogger("test_arm_minimal")

_POS     = np.array([0.0,  0.0, -50.0])
_VEL     = np.array([0.1,  0.0,   0.0])
_RPY     = np.zeros(3)
_ACCEL   = np.array([0.0,  0.0,  -9.81])
_GYRO    = np.zeros(3)


# ---------------------------------------------------------------------------
# Minimal sensor thread — lockstep via SITLInterface
# ---------------------------------------------------------------------------

def _sensor_worker(stop_event: threading.Event) -> None:
    """
    Level hover at 50 m NED.  Uses SITLInterface lockstep: wait for each
    binary servo packet from SITL, then reply with a static state packet.
    Short watchdog_timeout (0.5 s) so the stop_event is checked frequently
    and teardown is fast.
    """
    with SITLInterface(recv_port=StackConfig.SITL_JSON_PORT, watchdog_timeout=0.5) as sitl:
        log.info("sensor_worker: bound, waiting for SITL servo packets ...")
        while not stop_event.is_set():
            servos = sitl.recv_servos()   # blocks up to 0.5 s
            if servos is None:
                continue                  # timeout — check stop_event and retry
            sitl.send_state(
                pos_ned    = _POS,
                vel_ned    = _VEL,
                rpy_rad    = _RPY,
                accel_body = _ACCEL,
                gyro_body  = _GYRO,
            )


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_arm_minimal(tmp_path, request):
    """
    Minimal arm test: SITL + hardcoded sensor packets.  No mediator.

    Verifies that ArduPilot arms with our production config (ARMING_SKIPCHK=0xFFFF,
    H_RSC_MODE=1, CH8=2000) in isolation from the mediator and physics stack.
    """
    with _sitl_stack(tmp_path, test_name=request.node.name) as ctx:
        stop_sensors = threading.Event()
        sensor_thread = threading.Thread(
            target=_sensor_worker, args=(stop_sensors,),
            daemon=True, name="sensor",
        )
        sensor_thread.start()
        ctx.log.info("Sensor thread started (lockstep, hardcoded static values)")

        gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
        try:
            gcs.connect(timeout=20.0)
            gcs.start_heartbeat()
            gcs.request_stream(0, 10)

            ctx.log.info("Waiting for EKF attitude alignment ...")
            if not gcs.wait_ekf_attitude(timeout=20.0):
                sitl_tail = (
                    ctx.sitl_log.read_text(errors="replace")[-2000:]
                    if ctx.sitl_log.exists() else ""
                )
                pytest.fail(
                    "EKF never aligned — check sensor packet format\n"
                    + (f"\nSITL log tail:\n{sitl_tail}" if sitl_tail else "")
                )

            ctx.log.info("Arming ...")
            gcs.send_rc_override({8: 2000})
            time.sleep(0.2)
            try:
                gcs.arm(timeout=20.0, force=True, rc_override={8: 2000})
            except Exception as exc:
                sitl_tail = (
                    ctx.sitl_log.read_text(errors="replace")[-3000:]
                    if ctx.sitl_log.exists() else ""
                )
                pytest.fail(
                    f"Could not arm: {exc}\n"
                    + (f"\nSITL log tail:\n{sitl_tail}" if sitl_tail else "")
                )

            ctx.log.info("ARMED — production config OK")

        finally:
            stop_sensors.set()
            sensor_thread.join(timeout=1.0)
            try:
                gcs.close()
            except Exception:
                pass
