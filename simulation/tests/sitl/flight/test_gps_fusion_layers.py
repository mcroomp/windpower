"""
test_gps_fusion_layers.py -- Layer-by-layer GPS fusion diagnostic.

Static sensor worker, no mediator, no dynamics.  Each layer changes one
variable from the baseline to isolate what blocks GPS fusion.

Layer progression:
  L0  Level vehicle, yaw=0, vel=[0.1,0,0] North  -- gap=0 deg  (expect PASS)
  L1  RAWES tilt (65 deg), yaw=0, vel North       -- gap=0 deg  (expect PASS)
  L2  RAWES tilt + RAWES yaw=136.6, vel aligned   -- gap=0 deg  (expect PASS)
  L3  RAWES tilt + yaw=136.6, vel=current cfg     -- gap=152 deg (may fail)
  L4  RAWES tilt + yaw=136.6, vel=East (90 deg)   -- gap=46.6 deg (may fail)
  L5  RAWES tilt + yaw=136.6, vel=North (0 deg)   -- gap=136.6 deg (may fail)

Also: test_gps_fusion_armed -- same L0 sensor, but arms vehicle first,
  then waits for GPS fusion.  Tests hypothesis that GPS fusion requires
  armed state.

Run:
  bash simulation/dev.sh test-stack -v -k test_gps_fusion_layers
  bash simulation/dev.sh test-stack -v -k "test_gps_fusion_layers[L0"
  bash simulation/dev.sh test-stack -v -k test_gps_fusion_armed
"""
from __future__ import annotations

import logging
import math
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

log = logging.getLogger("test_gps_layers")

_GRAVITY = 9.81

# ---------------------------------------------------------------------------
# Rotation matrix helpers
# ---------------------------------------------------------------------------

def _euler_to_R(roll: float, pitch: float, yaw: float):
    """Body-to-NED rotation matrix from ZYX Euler angles [rad]."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return [
        [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,     cp*sr,             cp*cr            ],
    ]


def _accel_body_from_rpy_deg(roll_deg, pitch_deg, yaw_deg):
    """
    IMU specific force at rest in body frame = R^T @ [0, 0, -9.81].
    (NED: gravity = +Z, specific force at rest = -Z in world frame.)
    """
    R = _euler_to_R(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
    g = [0.0, 0.0, -_GRAVITY]
    return [
        R[0][0]*g[0] + R[1][0]*g[1] + R[2][0]*g[2],
        R[0][1]*g[0] + R[1][1]*g[1] + R[2][1]*g[2],
        R[0][2]*g[0] + R[1][2]*g[1] + R[2][2]*g[2],
    ]


# ---------------------------------------------------------------------------
# Layer definitions
# ---------------------------------------------------------------------------
# RAWES equilibrium Euler angles (from build_orb_frame(body_z=[0.277,0.877,-0.392])):
#   roll ~124.74 deg, pitch ~-46.12 deg, yaw ~136.6 deg
#
# Current vel0 = [0.916, -0.257, 0.093]  -> heading = atan2(-0.257, 0.916) = -15.7 deg
# heading_gap = 136.6 - (-15.7) = 152.3 deg  <- EKF blocks GPS here

_RAWES_ROLL  =  124.74
_RAWES_PITCH =  -46.12
_RAWES_YAW   =  136.6
_RAWES_ALT   =  14.12    # m above SITL home
_V           =  0.96     # orbital speed m/s


def _vel_heading(heading_deg: float, speed: float = _V):
    h = math.radians(heading_deg)
    return [speed*math.cos(h), speed*math.sin(h), 0.0]


# (id, description, attitude_deg=[roll,pitch,yaw], vel_ned, position_ned, expected_gap_deg)
_LAYERS = [
    (
        "L0_baseline",
        "Level hover, yaw=0, vel North  (gap=0 deg)",
        [0.0, 0.0, 0.0],
        [0.1, 0.0, 0.0],
        [0.0, 0.0, -50.0],
        0.0,
    ),
    (
        "L1_rawes_tilt_yaw0",
        "RAWES tilt, yaw=0, vel North  (gap=0 deg)",
        [_RAWES_ROLL, _RAWES_PITCH, 0.0],
        [0.1, 0.0, 0.0],
        [0.0, 0.0, -_RAWES_ALT],
        0.0,
    ),
    (
        "L2_rawes_full_vel_aligned",
        "RAWES full attitude, vel aligned with compass  (gap=0 deg)",
        [_RAWES_ROLL, _RAWES_PITCH, _RAWES_YAW],
        _vel_heading(_RAWES_YAW),
        [27.9, 95.0, -_RAWES_ALT],
        0.0,
    ),
    (
        "L3_rawes_vel_current",
        "RAWES full attitude, vel=current cfg  (gap=152 deg)",
        [_RAWES_ROLL, _RAWES_PITCH, _RAWES_YAW],
        [0.916, -0.257, 0.093],
        [27.9, 95.0, -_RAWES_ALT],
        152.3,
    ),
    (
        "L4_rawes_vel_east",
        "RAWES full attitude, vel East  (gap=46.6 deg)",
        [_RAWES_ROLL, _RAWES_PITCH, _RAWES_YAW],
        _vel_heading(90.0),
        [27.9, 95.0, -_RAWES_ALT],
        46.6,
    ),
    (
        "L5_rawes_vel_north",
        "RAWES full attitude, vel North  (gap=136.6 deg)",
        [_RAWES_ROLL, _RAWES_PITCH, _RAWES_YAW],
        _vel_heading(0.0),
        [27.9, 95.0, -_RAWES_ALT],
        136.6,
    ),
]


# ---------------------------------------------------------------------------
# Static sensor worker
# ---------------------------------------------------------------------------

def _sensor_worker(
    stop_event: threading.Event,
    attitude_deg: list,
    vel_ned: list,
    position_ned: list,
    hz: int = 100,
) -> None:
    """Send static sensor packets to SITL (no dynamics, no mediator)."""
    accel   = _accel_body_from_rpy_deg(*attitude_deg)
    rpy_rad = [math.radians(a) for a in attitude_deg]

    def _fmt(v):
        return f"[{v[0]:.6f},{v[1]:.6f},{v[2]:.6f}]"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", StackConfig.SITL_JSON_PORT))
    sock.settimeout(0.1)

    sitl_addr = None
    t0     = time.monotonic()
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
                '{"timestamp":' + f"{now - t0:.6f}" +
                ',"imu":{"gyro":[0,0,0],"accel_body":' + _fmt(accel) + '}' +
                ',"velocity":' + _fmt(vel_ned) +
                ',"position":' + _fmt(position_ned) +
                ',"attitude":' + _fmt(rpy_rad) +
                '}\n'
            )
            try:
                sock.sendto(pkt.encode(), sitl_addr)
            except Exception:
                pass
            t_next = now + 1.0 / hz

    sock.close()


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "layer_id,description,attitude_deg,vel_ned,position_ned,expected_gap",
    _LAYERS,
    ids=[l[0] for l in _LAYERS],
)
def test_gps_fusion_layers(
    tmp_path, request,
    layer_id, description, attitude_deg, vel_ned, position_ned, expected_gap,
):
    """
    Launch SITL with standard boot params, send static sensor packets for 90 s,
    report whether GPS fused.

    Layers with expected_gap < 30 deg assert GPS must fuse.
    Layers with expected_gap >= 30 deg are marked xfail.
    """
    compass_deg  = attitude_deg[2]
    vel_head_deg = math.degrees(math.atan2(vel_ned[1], vel_ned[0]))
    actual_gap   = abs(((compass_deg - vel_head_deg + 180) % 360) - 180)

    with _sitl_stack(
        tmp_path,
        log_name   = f"gps_layers_{layer_id}",
        log_prefix = "gps_layers",
        test_name  = request.node.name,
    ) as ctx:
        _log = ctx.log
        _log.info("")
        _log.info("=" * 60)
        _log.info("  Layer       : %s", layer_id)
        _log.info("  Description : %s", description)
        _log.info("  attitude    : roll=%.1f  pitch=%.1f  yaw=%.1f", *attitude_deg)
        _log.info("  accel_body  : %s", _accel_body_from_rpy_deg(*attitude_deg))
        _log.info("  vel_ned     : %s  heading=%.1f deg", vel_ned, vel_head_deg)
        _log.info("  heading_gap : %.1f deg  (compass=%.1f, vel=%.1f)",
                  actual_gap, compass_deg, vel_head_deg)
        _log.info("=" * 60)

        # Start static sensor worker
        stop_ev = threading.Event()
        worker  = threading.Thread(
            target=_sensor_worker,
            args=(stop_ev, attitude_deg, vel_ned, position_ned),
            daemon=True, name=f"sensor_{layer_id}",
        )
        worker.start()

        gps_origin = False
        gps_fused  = False
        gps_glitch = False

        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            _log.info("Connected. Waiting 90 s for GPS fusion ...")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 90.0:
                msg = gcs._recv(  # type: ignore[union-attr]
                    type=["STATUSTEXT"],
                    blocking=True, timeout=1.0,
                )
                if msg is None:
                    continue
                text = msg.text.strip()
                _log.info("  STATUSTEXT: %s", text)
                tl = text.lower()
                if "origin set" in tl:
                    gps_origin = True
                if "is using gps" in tl:
                    gps_fused = True
                if "gps glitch" in tl or "compass error" in tl:
                    gps_glitch = True

        finally:
            stop_ev.set()
            worker.join(timeout=2.0)
            try:
                gcs.close()
            except Exception:
                pass

        result = ("FUSED"  if gps_fused and not gps_glitch else
                  "GLITCH" if gps_glitch else
                  "NO_GPS")
        _log.info("")
        _log.info("  GPS origin : %s", gps_origin)
        _log.info("  GPS fused  : %s", gps_fused)
        _log.info("  GPS glitch : %s", gps_glitch)
        _log.info("  RESULT     : %s", result)

        _expect_pass = (expected_gap < 30.0)
        if _expect_pass:
            assert gps_origin, f"[{layer_id}] GPS origin never set"
            assert gps_fused,  f"[{layer_id}] GPS never fused (gap={actual_gap:.1f} deg)"
            assert not gps_glitch, f"[{layer_id}] GPS fused then glitched"
        else:
            if not gps_fused or gps_glitch:
                pytest.xfail(
                    f"[{layer_id}] GPS did not fuse as expected (gap={actual_gap:.1f} deg)"
                )


# ---------------------------------------------------------------------------
# "In-between" test: same L0 sensor, but arm vehicle before watching GPS
# ---------------------------------------------------------------------------

def test_gps_fusion_armed(tmp_path, request):
    """
    Hypothesis test: GPS fusion requires armed vehicle state.

    Uses the identical L0_baseline sensor (level, yaw=0, vel=[0.1,0,0] North,
    gap=0 deg) as test_gps_fusion_layers[L0_baseline], but:
      1. Waits for EKF attitude alignment
      2. Force-arms the vehicle (same as test_arm_minimal)
      3. Then waits up to 90 s for "EKF3 IMU0 is using GPS"

    If GPS fuses after arm but not before, the root cause is that EKF3 holds
    GPS fusion pending until the vehicle is armed.  Fix: ensure the full stack
    test arms before checking GPS fusion.

    If GPS still does not fuse after arm, the blocker is elsewhere
    (sensor format, EKF tuning, etc.).
    """
    # L0 sensor setup: level, yaw=0, vel=North (gap=0 deg)
    attitude_deg  = [0.0, 0.0, 0.0]
    vel_ned       = [0.1, 0.0, 0.0]
    position_ned  = [0.0, 0.0, -50.0]

    with _sitl_stack(
        tmp_path,
        log_name   = "gps_fusion_armed",
        log_prefix = "gps_fusion",
        test_name  = request.node.name,
    ) as ctx:
        _log = ctx.log
        _log.info("")
        _log.info("=" * 60)
        _log.info("  test_gps_fusion_armed")
        _log.info("  attitude : level (0,0,0)  vel : North 0.1 m/s  gap=0 deg")
        _log.info("  Strategy : arm vehicle first, then watch for GPS fusion")
        _log.info("=" * 60)

        stop_ev = threading.Event()
        worker  = threading.Thread(
            target=_sensor_worker,
            args=(stop_ev, attitude_deg, vel_ned, position_ned),
            daemon=True, name="sensor_armed",
        )
        worker.start()

        gps_origin = False
        gps_fused  = False
        gps_glitch = False
        armed      = False

        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            # --- step 1: wait for EKF attitude alignment (same as arm_minimal) ---
            _log.info("Step 1: waiting for EKF attitude alignment ...")
            if not gcs.wait_ekf_attitude(timeout=30.0):
                pytest.fail("EKF never aligned — check sensor packet format")

            _log.info("EKF aligned. Configuring for arm ...")
            gcs.set_param("FS_THR_ENABLE", 0)
            gcs.set_param("FS_GCS_ENABLE", 0)
            gcs.set_param("ARMING_SKIPCHK", 0xFFFF)
            gcs.set_param("H_RSC_MODE", 1)

            # --- step 2: arm (same sequence as test_arm_minimal) ---
            _log.info("Step 2: arming vehicle ...")
            gcs.send_rc_override({8: 2000})
            time.sleep(0.2)
            try:
                gcs.arm(timeout=20.0, force=True, rc_override={8: 2000})
                armed = True
                _log.info("ARMED -- now watching for GPS fusion ...")
            except Exception as exc:
                _log.warning("Arm failed: %s -- continuing to watch GPS anyway", exc)

            # --- step 3: watch for GPS fusion for up to 90 s ---
            _log.info("Step 3: waiting up to 90 s for GPS fusion ...")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 90.0:
                msg = gcs._recv(  # type: ignore[union-attr]
                    type=["STATUSTEXT"],
                    blocking=True, timeout=1.0,
                )
                if msg is None:
                    continue
                text = msg.text.strip()
                _log.info("  STATUSTEXT: %s", text)
                tl = text.lower()
                if "origin set" in tl:
                    gps_origin = True
                if "is using gps" in tl:
                    gps_fused = True
                    _log.info("  *** GPS FUSED at t=%.1f s after arm ***", time.monotonic() - t0)
                    break
                if "gps glitch" in tl or "compass error" in tl:
                    gps_glitch = True

        finally:
            stop_ev.set()
            worker.join(timeout=2.0)
            try:
                gcs.close()
            except Exception:
                pass

        result = ("FUSED"  if gps_fused and not gps_glitch else
                  "GLITCH" if gps_glitch else
                  "NO_GPS")
        _log.info("")
        _log.info("  armed      : %s", armed)
        _log.info("  GPS origin : %s", gps_origin)
        _log.info("  GPS fused  : %s", gps_fused)
        _log.info("  GPS glitch : %s", gps_glitch)
        _log.info("  RESULT     : %s", result)

        if not armed:
            pytest.fail("Vehicle did not arm -- cannot draw conclusions about GPS fusion")

        assert gps_origin, "GPS origin never set even after arm"
        assert gps_fused,  "GPS never fused even after arm (hypothesis rejected)"
        assert not gps_glitch, "GPS fused then glitched after arm"
