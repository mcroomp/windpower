"""
test_arm_minimal.py -- Minimal ArduCopter SITL arm test.

No mediator, no dynamics, no simulation dependencies.
A static-sensor mediator subprocess (mediator_static.py) sends hard-coded
sensor values to the SITL JSON backend via lockstep.

Purpose
-------
Isolate the arm sequence from everything else.  If this test passes, the arm
sequence works and any failure in the full stack is a mediator/sensor issue.
If this test fails, the problem is in the GCS arm sequence or ArduPilot params.

Sensor packet (NED body-frame, level hover at 50 m)
---------------------------------------------------
  pos        -- [0, 0, -50]  NED, 50 m above origin
  vel        -- [0.1, 0, 0]  small N velocity for EKF yaw derivation
  attitude   -- [0, 0, 0]    roll=0, pitch=0, yaw=0
  gyro       -- [0, 0, 0]    no rotation
  accel_body -- [0, 0, -9.81] gravity reaction in NED body frame

Usage
-----
  bash simulation/dev.sh test-stack -v -k test_arm_minimal
"""

import logging
import sys
import time
from pathlib import Path

import numpy as np
import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import StackConfig, _static_stack
from gcs import RawesGCS

log = logging.getLogger("test_arm_minimal")

_POS   = np.array([0.0,  0.0, -50.0])
_VEL   = np.array([0.1,  0.0,   0.0])
_RPY   = np.zeros(3)
_ACCEL = np.array([0.0,  0.0,  -9.81])
_GYRO  = np.zeros(3)


def test_arm_minimal(tmp_path, request):
    """
    Minimal arm test: SITL + static-sensor mediator subprocess.  No mediator.

    Verifies that ArduPilot arms with our production config (H_RSC_MODE=1,
    CH8=2000) in isolation from the mediator and physics stack.
    """
    with _static_stack(
        tmp_path, test_name=request.node.name,
        pos=_POS, vel=_VEL, rpy=_RPY, accel_body=_ACCEL, gyro=_GYRO,
    ) as ctx:

        def _assert_alive() -> None:
            if ctx.mediator_proc.poll() is not None:
                tail = (
                    ctx.mediator_log.read_text(errors="replace")[-2000:]
                    if ctx.mediator_log is not None and ctx.mediator_log.exists()
                    else ""
                )
                pytest.fail(
                    f"static mediator exited (rc={ctx.mediator_proc.returncode})"
                    + (f"\n{tail}" if tail else "")
                )

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
                    "EKF never aligned -- check sensor packet format"
                    + (f"\n\nSITL log tail:\n{sitl_tail}" if sitl_tail else "")
                )

            _assert_alive()
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
                    f"Could not arm: {exc}"
                    + (f"\n\nSITL log tail:\n{sitl_tail}" if sitl_tail else "")
                )

            _assert_alive()
            ctx.log.info("ARMED -- production config OK")

        finally:
            try:
                gcs.close()
            except Exception:
                pass
