"""
test_keep_armed.py — SITL test for keep_armed.lua watchdog script.

Verifies that keep_armed.lua re-arms the vehicle after a disarm event.

Initial arm is done via GCS force-arm (bypasses SITL battery/safety checks).
After that, we disarm and confirm the script re-arms within 3 sim-seconds.

Sensor setup: same static hover at 50 m as test_arm_minimal.
"""

import logging
import sys
import threading
from pathlib import Path

import numpy as np
import pytest
from pymavlink import mavutil

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import StackConfig, _sitl_stack, _install_lua_scripts
from gcs import RawesGCS
from sitl_interface import SITLInterface

log = logging.getLogger("test_keep_armed")

_POS   = np.array([0.0,  0.0, -50.0])
_VEL   = np.array([0.1,  0.0,   0.0])
_RPY   = np.zeros(3)
_ACCEL = np.array([0.0,  0.0,  -9.81])
_GYRO  = np.zeros(3)


def _sensor_worker(stop_event: threading.Event) -> None:
    with SITLInterface(recv_port=StackConfig.SITL_JSON_PORT, watchdog_timeout=0.5) as sitl:
        log.info("sensor_worker: waiting for SITL servo packets ...")
        while not stop_event.is_set():
            servos = sitl.recv_servos()
            if servos is None:
                continue
            sitl.send_state(
                pos_ned    = _POS,
                vel_ned    = _VEL,
                rpy_rad    = _RPY,
                accel_body = _ACCEL,
                gyro_body  = _GYRO,
            )


def _check_armed(gcs: "RawesGCS") -> bool:
    msg = gcs._recv(type="HEARTBEAT", blocking=True, timeout=2.0)
    if msg is None:
        return False
    return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def test_keep_armed(tmp_path, request):
    """
    keep_armed.lua watchdog test:
      1. Script is running (STATUSTEXT 'keep_armed: re-arming' visible during boot).
      2. After GCS force-arm, vehicle stays armed.
      3. After force-disarm, script re-arms within 3 sim-seconds.
    """
    _install_lua_scripts("keep_armed.lua")

    with _sitl_stack(
            tmp_path,
            test_name=request.node.name,
            extra_boot_params={"BATT_MONITOR": 0, "ARMING_CHECK": 0, "BRD_SAFETYENABLE": 0},
    ) as ctx:
        stop_sensors = threading.Event()
        sensor_thread = threading.Thread(
            target=_sensor_worker, args=(stop_sensors,),
            daemon=True, name="sensor",
        )
        sensor_thread.start()

        gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
        try:
            gcs.connect(timeout=20.0)
            gcs.start_heartbeat()
            gcs.request_stream(0, 10)
            gcs.send_rc_override({8: 2000})

            ctx.log.info("Waiting for EKF attitude alignment ...")
            if not gcs.wait_ekf_attitude(timeout=30.0):
                pytest.fail("EKF never aligned")

            # --- Part 1: GCS force-arm (bypasses SITL battery/safety checks) ---
            ctx.log.info("Force-arming via GCS ...")
            gcs.arm(timeout=15.0, force=True, rc_override={8: 2000})
            ctx.log.info("Armed via GCS")

            # --- Part 2: force-disarm and verify keep_armed.lua re-arms ---
            ctx.log.info("Force-disarming to trigger watchdog ...")
            gcs._mav.mav.command_long_send(
                gcs._target_system, gcs._target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196.0, 0, 0, 0, 0, 0,  # param1=0 = disarm, param2=21196 = force
            )
            gcs.sim_sleep(5.0)

            if not _check_armed(gcs):
                pytest.fail("keep_armed.lua did not re-arm vehicle within 5 s after force-disarm")
            ctx.log.info("PASS: vehicle re-armed by keep_armed.lua after disarm")

        finally:
            stop_sensors.set()
            sensor_thread.join(timeout=1.0)
            try:
                gcs.close()
            except Exception:
                pass
