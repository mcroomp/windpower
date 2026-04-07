"""
test_arm_minimal.py — Minimal ArduCopter SITL arm test.

No mediator, no dynamics, no simulation dependencies.
Sends hard-coded sensor values directly to the SITL JSON backend and
attempts to arm using every known strategy until one succeeds.

Purpose
-------
Isolate the arm problem from everything else.  If this test passes the arm
sequence works and any failure in the full stack is a mediator/sensor issue.
If this test fails, the problem is in our GCS arm sequence or ArduPilot params.

Shared code used
----------------
  conftest.py     — assert_stack_ports_free(), SITL_GCS_PORT, SITL_JSON_PORT
  gcs.py          — RawesGCS (set_param, arm, send_rc_override,
                    wait_ekf_attitude, start_heartbeat, connect)
  test_stack_integration.py — _launch_sitl, _resolve_sim_vehicle,
                              _terminate_process, STACK_ENV_FLAG

Sensor packet (NED body-frame, level hover at 50 m)
---------------------------------------------------
  Required fields (SITL JSON backend):
    timestamp   — simulation time in seconds (incrementing)
    imu/gyro    — [0, 0, 0]  no rotation
    imu/accel_body — [0, 0, -9.81]  gravity reaction in NED body frame
    velocity    — [0.1, 0, 0]  tiny N velocity for EKF yaw derivation
  Optional:
    position    — [0, 0, -50]  NED, 50 m above origin
    attitude    — [0, 0, 0]   roll=0, pitch=0, yaw=0

JSON transport
--------------
  SITL sends servo output packets FROM its ephemeral UDP port TO SITL_JSON_PORT.
  The sensor worker binds to SITL_JSON_PORT, receives those packets to discover
  SITL's address, then sends state packets back to that address.
  This mirrors the mediator role without any dynamics or aerodynamics.

Usage
-----
  # inside container:
  pytest simulation/tests/stack/test_arm_minimal.py -s -v
"""

import logging
import os
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

_SIM_DIR   = Path(__file__).resolve().parents[2]
_STACK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_STACK_DIR))

from conftest import StackConfig
from gcs import RawesGCS
from test_stack_integration import (
    STACK_ENV_FLAG,
    _launch_sitl,
    _resolve_sim_vehicle,
    _terminate_process,
    _kill_by_port,
)

log = logging.getLogger("test_arm_minimal")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_SENSOR_HZ = 100    # sensor packet rate (Hz)
_GRAVITY   = 9.81


# ---------------------------------------------------------------------------
# Minimal sensor thread
# ---------------------------------------------------------------------------

def _sensor_worker(stop_event: threading.Event) -> None:
    """
    Simulate the mediator's JSON transport role — without any dynamics.

    Binds to SITL_JSON_PORT (9002) to receive SITL's servo output packets.
    Captures SITL's source address from the first packet received, then
    sends hard-coded JSON state packets back to that address at _SENSOR_HZ.

    The sensor values are static (level hover at 50 m NED) with a tiny
    north velocity so the EKF can derive yaw.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", StackConfig.SITL_JSON_PORT))
    sock.settimeout(0.1)   # short timeout so we can check stop_event

    sitl_addr = None   # filled in from SITL's first packet
    t0 = time.monotonic()
    t_next_send = t0

    while not stop_event.is_set():
        # Drain incoming packets (SITL servo outputs) — capture SITL's address.
        try:
            _, addr = sock.recvfrom(4096)
            if sitl_addr is None:
                sitl_addr = addr
                log.info("sensor_worker: SITL found at %s:%d", addr[0], addr[1])
        except socket.timeout:
            pass

        now = time.monotonic()
        if sitl_addr is not None and now >= t_next_send:
            ts = now - t0
            pkt = (
                '{"timestamp":' + f'{ts:.6f}' +
                ',"imu":{"gyro":[0,0,0],"accel_body":[0,0,' + f'{-_GRAVITY:.4f}' + ']}' +
                ',"velocity":[0.1,0,0]'
                ',"position":[0,0,-50]'
                ',"attitude":[0,0,0]}\n'
            )
            try:
                sock.sendto(pkt.encode(), sitl_addr)
            except Exception:
                pass
            t_next_send = now + 1.0 / _SENSOR_HZ

    sock.close()


# ---------------------------------------------------------------------------
# Arm strategy helper
# ---------------------------------------------------------------------------

def _try_arm(gcs: RawesGCS, label: str, rc8: int = 2000, timeout: float = 20.0) -> bool:
    """Send RC override then force-arm. Returns True if HEARTBEAT shows armed."""
    log.info("Arming: %s", label)
    gcs.send_rc_override({8: rc8})
    time.sleep(0.2)
    try:
        gcs.arm(timeout=timeout, force=True, rc_override={8: rc8})
        return True
    except Exception as exc:
        log.info("Arm failed (%s): %s", label, exc)
        return False


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_arm_minimal(tmp_path):
    """
    Minimal arm test: SITL + hardcoded sensor packets.  No mediator.

    Verifies that ArduPilot arms with our production config (ARMING_SKIPCHK=0xFFFF,
    H_RSC_MODE=1, CH8=2000) in isolation from the mediator and physics stack.
    If this fails, the issue is in ArduPilot params or the GCS arm sequence,
    not in the mediator or dynamics.
    """
    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run stack tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip("SITL sim_vehicle.py not found — set RAWES_ARDUPILOT_PATH or RAWES_SIM_VEHICLE")

    # Check all stack ports before launching — fast failure instead of 30s timeout.
    StackConfig.verify()

    sitl_log = tmp_path / "sitl.log"
    sitl_proc = _launch_sitl(sim_vehicle, sitl_log)

    # Start sensor immediately — SITL won't progress until the JSON backend
    # receives packets and sends back state.
    stop_sensors = threading.Event()
    sensor_thread = threading.Thread(
        target=_sensor_worker, args=(stop_sensors,), daemon=True, name="sensor",
    )
    sensor_thread.start()
    log.info("Sensor thread started (%d Hz, hardcoded static values)", _SENSOR_HZ)

    gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
    try:
        gcs.connect(timeout=30.0)
        gcs.start_heartbeat()
        gcs.request_stream(0, 10)   # MAV_DATA_STREAM_ALL at 10 Hz

        # Disable failsafes that fire when there's no real RC or GCS heartbeat
        gcs.set_param("FS_THR_ENABLE", 0)
        gcs.set_param("FS_GCS_ENABLE", 0)

        log.info("Waiting for EKF attitude alignment …")
        if not gcs.wait_ekf_attitude(timeout=30.0):
            if sitl_log.exists():
                log.warning("SITL log:\n%s", sitl_log.read_text(errors="replace")[-2000:])
            pytest.fail("EKF never aligned — check sensor packet format")

        # ── Arm with production config ────────────────────────────────────────
        gcs.set_param("ARMING_SKIPCHK", 0xFFFF)
        gcs.set_param("H_RSC_MODE", 1)

        if not _try_arm(gcs, "production config", rc8=2000):
            sitl_tail = sitl_log.read_text(errors="replace")[-3000:] if sitl_log.exists() else ""
            pytest.fail(
                "Could not arm with production config "
                "(ARMING_SKIPCHK=0xFFFF, H_RSC_MODE=1, CH8=2000).\n"
                + (f"\nSITL log tail:\n{sitl_tail}" if sitl_tail else "")
            )

        log.info("ARMED — production config OK")

    finally:
        stop_sensors.set()
        sensor_thread.join(timeout=2.0)
        try:
            gcs.close()
        except Exception:
            pass
        _terminate_process(sitl_proc)
        _kill_by_port(StackConfig.SITL_GCS_PORT)
