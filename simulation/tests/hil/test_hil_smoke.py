"""
tests/hil/test_hil_smoke.py -- Hardware-in-the-loop connectivity smoke test.

Verifies that a Pixhawk 6C connected over USB can be reached via MAVLink,
is running ArduPilot, has healthy IMU sensors, and has the expected RAWES
parameter values flashed.

This is NOT a full HIL simulation (no physics loop, no sensor injection).
It is a connectivity and configuration sanity check you can run with the
Pixhawk plugged in over USB before a bench session.

Usage
-----
Set the serial port and run with the existing unit-test venv:

    Windows (Git Bash):
        RAWES_HIL_PORT=COM3 bash sim.sh test-hil -v

    Or directly:
        RAWES_HIL_PORT=COM3 .venv/Scripts/python.exe -m pytest \\
            simulation/tests/hil/test_hil_smoke.py -v

Environment variables
---------------------
  RAWES_HIL_PORT      Serial port where the Pixhawk appears.
                      Windows: COM3, COM4, ...  Linux: /dev/ttyACM0
                      Required -- test is skipped if not set.
  RAWES_HIL_BAUD      Baud rate (default: 115200).
  RAWES_HIL_TIMEOUT   Connection timeout in seconds (default: 15).
"""
from __future__ import annotations

import math
import os
import time

import pytest
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Fixture: serial connection
# ---------------------------------------------------------------------------

_ENV_PORT    = "RAWES_HIL_PORT"
_ENV_BAUD    = "RAWES_HIL_BAUD"
_ENV_TIMEOUT = "RAWES_HIL_TIMEOUT"


@pytest.fixture(scope="module")
def hil_mav():
    """
    Open a MAVLink serial connection to the Pixhawk.

    Skips the entire module if RAWES_HIL_PORT is not set.
    Yields the mavutil connection; closes it after all tests in the module.
    """
    port = os.environ.get(_ENV_PORT)
    if not port:
        pytest.skip(
            f"Set {_ENV_PORT}=COMx (Windows) or /dev/ttyACMx (Linux) "
            "to run HIL smoke tests"
        )

    baud    = int(os.environ.get(_ENV_BAUD, "115200"))
    timeout = float(os.environ.get(_ENV_TIMEOUT, "15"))

    print(f"\nConnecting to {port} at {baud} baud ...")

    mav = mavutil.mavlink_connection(port, baud=baud, source_system=255)

    hb = mav.wait_heartbeat(timeout=timeout)
    if hb is None:
        mav.close()
        pytest.fail(
            f"No heartbeat received from {conn_str} within {timeout:.0f} s. "
            "Check the port, baud rate, and that the Pixhawk is powered."
        )

    print(
        f"Connected: sysid={mav.target_system} "
        f"autopilot={hb.autopilot} type={hb.type}"
    )

    yield mav
    mav.close()


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _recv_param(mav, name: str, timeout: float = 8.0) -> float | None:
    """Request and return a single parameter value, or None on timeout.

    Retries the request every 2 s in case ArduPilot dropped the first one or
    responded with related params from the same subgroup that were already
    consumed by a previous test.
    """
    deadline     = time.monotonic() + timeout
    last_request = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_request > 2.0:
            mav.mav.param_request_read_send(
                mav.target_system, mav.target_component,
                name.encode(), -1,
            )
            last_request = now
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip("\x00") == name:
            return float(msg.param_value)
    return None


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_heartbeat_is_ardupilot(hil_mav):
    """
    Pixhawk sends heartbeats and identifies itself as an ArduPilot autopilot.

    MAV_AUTOPILOT_ARDUPILOTMEGA = 3.
    MAV_TYPE_HELICOPTER         = 4.
    """
    msg = hil_mav.recv_match(type="HEARTBEAT", blocking=True, timeout=3.0)
    assert msg is not None, "No HEARTBEAT received within 3 s"

    assert msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, (
        f"Expected ArduPilot autopilot (3), got {msg.autopilot}"
    )
    assert msg.type == mavutil.mavlink.MAV_TYPE_HELICOPTER, (
        f"Expected MAV_TYPE_HELICOPTER (4), got {msg.type} "
        "(check FRAME_CLASS param -- should be heli frame)"
    )


def test_attitude_flowing(hil_mav):
    """
    ATTITUDE messages arrive and contain finite, plausible roll/pitch/yaw.

    Confirms the IMU is running and ArduPilot's EKF attitude estimate is live.
    Bench test (Pixhawk stationary): |roll|, |pitch| < 30 deg is expected.
    """
    # Request ATTITUDE stream if not already flowing
    hil_mav.mav.request_data_stream_send(
        hil_mav.target_system, hil_mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 4, 1,
    )

    msg = hil_mav.recv_match(type="ATTITUDE", blocking=True, timeout=5.0)
    assert msg is not None, "No ATTITUDE message received within 5 s"

    roll_deg  = math.degrees(msg.roll)
    pitch_deg = math.degrees(msg.pitch)
    yaw_deg   = math.degrees(msg.yaw)

    assert all(math.isfinite(v) for v in (roll_deg, pitch_deg, yaw_deg)), (
        f"ATTITUDE contains non-finite values: "
        f"roll={roll_deg} pitch={pitch_deg} yaw={yaw_deg}"
    )

    print(
        f"  ATTITUDE: roll={roll_deg:+.1f} deg  "
        f"pitch={pitch_deg:+.1f} deg  yaw={yaw_deg:+.1f} deg"
    )


def test_sys_status_imu_healthy(hil_mav):
    """
    SYS_STATUS shows accelerometer and gyroscope as present and healthy.

    Confirms the Pixhawk's IMU hardware is detected and passing self-test.
    """
    hil_mav.mav.request_data_stream_send(
        hil_mav.target_system, hil_mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1,
    )

    msg = hil_mav.recv_match(type="SYS_STATUS", blocking=True, timeout=5.0)
    assert msg is not None, "No SYS_STATUS message received within 5 s"

    IMU_SENSORS = (
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
        | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL
    )

    present = msg.onboard_control_sensors_present & IMU_SENSORS
    enabled = msg.onboard_control_sensors_enabled & IMU_SENSORS
    healthy = msg.onboard_control_sensors_health  & IMU_SENSORS

    assert present == IMU_SENSORS, (
        f"IMU sensors not present (present=0x{msg.onboard_control_sensors_present:08x})"
    )
    assert enabled == IMU_SENSORS, (
        f"IMU sensors not enabled (enabled=0x{msg.onboard_control_sensors_enabled:08x})"
    )
    assert healthy == IMU_SENSORS, (
        f"IMU sensors unhealthy (health=0x{msg.onboard_control_sensors_health:08x})"
    )


def test_vehicle_is_disarmed(hil_mav):
    """
    Pixhawk is disarmed when plugged in for bench testing.

    A bench smoke test should never run with a live armed vehicle.
    """
    msg = hil_mav.recv_match(type="HEARTBEAT", blocking=True, timeout=3.0)
    assert msg is not None, "No HEARTBEAT received"

    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    assert not armed, (
        "Vehicle is ARMED during HIL smoke test -- disarm before running"
    )


def test_param_frame_class(hil_mav):
    """
    FRAME_CLASS = 6 (Traditional Helicopter).

    Confirms the correct helicopter frame is selected.
    Without this, H_* parameters do not exist.
    """
    val = _recv_param(hil_mav, "FRAME_CLASS")
    assert val is not None, "Could not read FRAME_CLASS"
    assert int(val) == 6, (
        f"FRAME_CLASS = {int(val)}, expected 6 (Traditional Heli). "
        "Has the RAWES param file been flashed?"
    )


def test_param_h_swash_type(hil_mav):
    """
    H_SWASH_TYPE = 3 (H3-120 three-servo swashplate).

    This must match the physical RAWES swashplate wiring (S1/S2/S3 servos).
    H_PHANG = 0 has been validated in SITL (test_h_swash_phang).
    """
    val = _recv_param(hil_mav, "H_SW_TYPE")
    assert val is not None, "Could not read H_SW_TYPE"
    assert int(val) == 3, (
        f"H_SW_TYPE = {int(val)}, expected 3 (H3-120). "
        "Check rawes_params.parm was uploaded."
    )


def test_param_h_sw_phang(hil_mav):
    """
    H_SW_PHANG does not exist on this Pixhawk firmware build.

    The SITL Docker image exposes H_SW_PHANG; the Holybro firmware on the
    Pixhawk 6C does not -- phase advance is implicit in H_SW_TYPE=3 (H3-120).
    This test verifies that H_SW_TYPE=3 is set (phase=0 is the H3-120 default)
    and skips the direct H_SW_PHANG read.
    """
    val = _recv_param(hil_mav, "H_SW_TYPE")
    assert val is not None, "Could not read H_SW_TYPE"
    assert int(val) == 3, (
        f"H_SW_TYPE = {int(val)}, expected 3 (H3-120 implies phase=0). "
        "This will cause incorrect cyclic response."
    )


def test_param_atc_imax_zero(hil_mav):
    """
    ATC_RAT_RLL_IMAX = ATC_RAT_PIT_IMAX = 0 (no integrator windup).

    The ACRO mode integrator accumulates orbital angular velocity as a
    persistent rate error; zeroing IMAX prevents ever-growing cyclic tilt.
    """
    for name in ("ATC_RAT_RLL_IMAX", "ATC_RAT_PIT_IMAX"):
        val = _recv_param(hil_mav, name)
        assert val is not None, f"Could not read {name}"
        assert val == 0.0, (
            f"{name} = {val}, expected 0. "
            "ACRO integrator windup will cause growing cyclic tilt in orbit."
        )


def test_scr_enable(hil_mav):
    """
    SCR_ENABLE = 1 (Lua scripting subsystem active).

    Required for rawes.lua (orbit tracking + yaw trim controller).
    Must be set before the first boot -- see CLAUDE.md SCR_ENABLE bootstrap note.
    """
    val = _recv_param(hil_mav, "SCR_ENABLE")
    assert val is not None, "Could not read SCR_ENABLE"
    assert int(val) == 1, (
        f"SCR_ENABLE = {int(val)}, expected 1. "
        "Lua scripting is disabled -- rawes.lua will not run."
    )
