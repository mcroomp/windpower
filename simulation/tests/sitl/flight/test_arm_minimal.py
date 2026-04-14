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

Uses _sitl_stack for all SITL lifecycle, boot params, and log management.
The sensor worker and arm sequence are the only test-specific logic here.

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

Usage
-----
  bash simulation/dev.sh test-stack -v -k test_arm_minimal
"""

import logging
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import StackConfig, _sitl_stack
from gcs import RawesGCS

log = logging.getLogger("test_arm_minimal")

_SENSOR_HZ = 100
_GRAVITY   = 9.81


# ---------------------------------------------------------------------------
# Minimal sensor thread
# ---------------------------------------------------------------------------

def _sensor_worker(stop_event: threading.Event) -> None:
    """
    Level hover at 50 m NED, tiny north velocity.
    Binds to SITL_JSON_PORT, discovers SITL address from first packet,
    then sends static JSON state at _SENSOR_HZ.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", StackConfig.SITL_JSON_PORT))
    sock.settimeout(0.1)

    sitl_addr = None
    t0 = time.monotonic()
    t_next = t0

    while not stop_event.is_set():
        try:
            _, addr = sock.recvfrom(4096)
            if sitl_addr is None:
                sitl_addr = addr
                log.info("sensor_worker: SITL at %s:%d", addr[0], addr[1])
        except socket.timeout:
            pass

        now = time.monotonic()
        if sitl_addr is not None and now >= t_next:
            pkt = (
                '{"timestamp":' + f'{now - t0:.6f}' +
                ',"imu":{"gyro":[0,0,0],"accel_body":[0,0,' + f'{-_GRAVITY:.4f}' + ']}' +
                ',"velocity":[0.1,0,0]'
                ',"position":[0,0,-50]'
                ',"attitude":[0,0,0]}\n'
            )
            try:
                sock.sendto(pkt.encode(), sitl_addr)
            except Exception:
                pass
            t_next = now + 1.0 / _SENSOR_HZ

    sock.close()


# ---------------------------------------------------------------------------
# Arm helper
# ---------------------------------------------------------------------------

def _try_arm(gcs: RawesGCS, label: str, rc8: int = 2000, timeout: float = 20.0) -> bool:
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

def test_arm_minimal(tmp_path, request):
    """
    Minimal arm test: SITL + hardcoded sensor packets.  No mediator.

    Verifies that ArduPilot arms with our production config (ARMING_SKIPCHK=0xFFFF,
    H_RSC_MODE=1, CH8=2000) in isolation from the mediator and physics stack.
    """
    with _sitl_stack(
        tmp_path,
        log_name   = "arm_minimal",
        log_prefix = "arm_minimal",
        test_name  = request.node.name,
    ) as ctx:
        stop_sensors = threading.Event()
        sensor_thread = threading.Thread(
            target=_sensor_worker, args=(stop_sensors,),
            daemon=True, name="sensor",
        )
        sensor_thread.start()
        ctx.log.info("Sensor thread started (%d Hz, hardcoded static values)", _SENSOR_HZ)

        gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
        try:
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat()
            gcs.request_stream(0, 10)

            gcs.set_param("FS_THR_ENABLE", 0)
            gcs.set_param("FS_GCS_ENABLE", 0)

            ctx.log.info("Waiting for EKF attitude alignment ...")
            if not gcs.wait_ekf_attitude(timeout=30.0):
                sitl_tail = (
                    ctx.sitl_log.read_text(errors="replace")[-2000:]
                    if ctx.sitl_log.exists() else ""
                )
                pytest.fail(
                    "EKF never aligned — check sensor packet format\n"
                    + (f"\nSITL log tail:\n{sitl_tail}" if sitl_tail else "")
                )

            gcs.set_param("ARMING_SKIPCHK", 0xFFFF)
            gcs.set_param("H_RSC_MODE", 1)

            if not _try_arm(gcs, "production config", rc8=2000):
                sitl_tail = (
                    ctx.sitl_log.read_text(errors="replace")[-3000:]
                    if ctx.sitl_log.exists() else ""
                )
                pytest.fail(
                    "Could not arm with production config "
                    "(ARMING_SKIPCHK=0xFFFF, H_RSC_MODE=1, CH8=2000).\n"
                    + (f"\nSITL log tail:\n{sitl_tail}" if sitl_tail else "")
                )

            ctx.log.info("ARMED — production config OK")

        finally:
            stop_sensors.set()
            sensor_thread.join(timeout=2.0)
            try:
                gcs.close()
            except Exception:
                pass
