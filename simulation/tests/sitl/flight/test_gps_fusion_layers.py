"""
test_gps_fusion_layers.py -- GPS fusion diagnostic with dual-GPS moving-baseline.

Two GPS antennas 50 cm apart (±25 cm along body X) give heading from RELPOSNED.
Yaw is known from first GPS fix — no motion needed.

Hub is held at tether equilibrium with constant sensor values (zero vel, zero gyro,
fixed attitude). GPS fuses within ~34 s.

Run:
  bash simulation/dev.sh test-stack-parallel --fresh -n 1 -k test_gps_fusion_dual_gps
"""
from __future__ import annotations

import logging
import math
import sys
import threading
from pathlib import Path

import json
import numpy as np
import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import StackConfig, _sitl_stack
from gcs import RawesGCS
from sitl_interface import SITLInterface

_STEADY_STATE_JSON = _SIM_DIR / "steady_state_starting.json"

log = logging.getLogger("test_gps_layers")

_GRAVITY = 9.81


# ---------------------------------------------------------------------------
# Frame helpers
# ---------------------------------------------------------------------------

def _rpy_from_R(R: np.ndarray) -> np.ndarray:
    """ZYX Euler [roll, pitch, yaw] in radians from a 3x3 body-to-NED matrix."""
    pitch = math.asin(-float(np.clip(R[2, 0], -1.0, 1.0)))
    roll  = math.atan2(float(R[2, 1]), float(R[2, 2]))
    yaw   = math.atan2(float(R[1, 0]), float(R[0, 0]))
    return np.array([roll, pitch, yaw])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wait_statustext(gcs, keywords, timeout, log, label):
    """Poll MAVLink until any keyword matches a STATUSTEXT or timeout."""
    t0         = gcs.sim_now()
    t_last_ekf = t0 - 1.0
    t_last_gps = t0 - 1.0
    t_last_att = t0 - 1.0
    t_last_pos = t0
    last_flags = None

    while gcs.sim_now() - t0 < timeout:
        msg = gcs._recv(
            type=["STATUSTEXT", "EKF_STATUS_REPORT", "LOCAL_POSITION_NED",
                  "GPS_RAW_INT", "ATTITUDE"],
            blocking=True, timeout=0.2,
        )
        now     = gcs.sim_now()
        elapsed = now - t0

        if msg is None:
            continue

        mt = msg.get_type()

        if mt == "STATUSTEXT":
            text = msg.text.strip()
            log.info("  [%s t=%5.1f s] STATUSTEXT: %s", label, elapsed, text)
            tl = text.lower()
            for kw in keywords:
                if kw in tl:
                    return text

        elif mt == "EKF_STATUS_REPORT":
            flags = msg.flags
            if now - t_last_ekf >= 1.0 or flags != last_flags:
                log.info(
                    "  [%s t=%5.1f s] EKF flags=0x%04x vel=%.2f pos_h=%.2f pos_v=%.2f",
                    label, elapsed, flags,
                    msg.velocity_variance, msg.pos_horiz_variance, msg.pos_vert_variance,
                )
                t_last_ekf = now
                last_flags = flags

        elif mt == "GPS_RAW_INT" and now - t_last_gps >= 1.0:
            log.info(
                "  [%s t=%5.1f s] GPS fix=%d sats=%d hdop=%.1f spd=%.2f m/s",
                label, elapsed,
                msg.fix_type, msg.satellites_visible,
                msg.eph / 100.0, msg.vel / 100.0,
            )
            t_last_gps = now

        elif mt == "ATTITUDE" and now - t_last_att >= 1.0:
            log.info(
                "  [%s t=%5.1f s] ATT roll=%.1f pitch=%.1f yaw=%.1f deg",
                label, elapsed,
                math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw),
            )
            t_last_att = now

        elif mt == "LOCAL_POSITION_NED" and now - t_last_pos >= 5.0:
            log.info(
                "  [%s t=%5.1f s] LOCAL_POS z=%.2f vz=%.3f",
                label, elapsed, msg.z, msg.vz,
            )
            t_last_pos = now

    pytest.fail(f"[{label}] timed out after {timeout:.0f} s waiting for: {keywords}")


def _rc_keepalive(gcs, channels: dict, stop_event: threading.Event, interval: float = 0.4):
    """Send RC_CHANNELS_OVERRIDE every interval seconds until stop_event."""
    while not stop_event.is_set():
        try:
            gcs.send_rc_override(channels)
        except Exception:
            pass
        stop_event.wait(interval)


# ---------------------------------------------------------------------------
# test_gps_fusion_dual_gps
# ---------------------------------------------------------------------------

def test_gps_fusion_dual_gps(tmp_path, request):
    """
    GPS fusion using dual-GPS moving-baseline heading (EK3_SRC1_YAW=2).

    --- Why this works ---

    ArduPilot EKF3 requires two things before fusing GPS position/velocity:
      1. Yaw alignment complete (yawAlignComplete flag set).
      2. delAngBiasLearned (gyro delta-angle bias converged).

    With EK3_SRC1_YAW=8 (GPS velocity yaw / EKFGSF), yaw alignment requires
    the vehicle to turn. A stationary body never satisfies this — all EKFGSF
    hypotheses predict identical NED velocities — so GPS never fuses.

    With EK3_SRC1_YAW=2 (dual-antenna RELPOSNED), yaw is derived directly from
    the NED baseline vector between the two antennas. Yaw is known from the
    very first GPS fix — no motion required.

    delAngBiasLearned also converges without any motion. With constant-zero
    gyro input, the EKF bias estimator simply converges to zero bias. GPS fuses
    at ~34 s from test start.

    --- ArduPilot parameters (all defaults in rawes_sitl_defaults.parm) ---

    EK3_SRC1_YAW=2     GPS dual-antenna yaw (RELPOSNED). Replaces EKFGSF.
                       Yaw known from first fix; no turning motion needed.

    GPS1_TYPE=17       F9P RTK_BASE. Sends RTCM corrections to GPS2 over serial.
    GPS2_TYPE=18       F9P RTK_ROVER. Receives RTCM, outputs RELPOSNED message
                       containing the NED baseline vector between antennas.

    SIM_GPS2_DISABLE=0 Enable second SITL GPS instance.
    SIM_GPS2_TYPE=1    GPS2 sim type = UBLOX (required to generate RELPOSNED).
    SIM_GPS2_HDG=1     Tell SITL to generate RELPOSNED heading on GPS2 (the rover).

    GPS1_POS_X=+0.25   EKF lever-arm: GPS1 is +25 cm along body X from hub centre.
    GPS2_POS_X=-0.25   EKF lever-arm: GPS2 is -25 cm along body X (opposite side).
    SIM_GPS_POS_X=+0.25   SITL simulates GPS1 at +25 cm body X offset.
    SIM_GPS2_POS_X=-0.25  SITL simulates GPS2 at -25 cm body X offset.

    GPS_AUTO_CONFIG=0  CRITICAL: prevents ArduPilot from reconfiguring UBLOX chips
                       over serial. In SITL the GPS is simulated — auto-config
                       corrupts the RELPOSNED stream and heading is never received.

    COMPASS_USE/USE2/USE3=0   Compass disabled. GB4008 motor swamps magnetometer on
    COMPASS_ENABLE=0          hardware. In SITL synthetic compasses cycle every 10 s
                              when innovations fail, blocking GPS fusion.

    EK3_GPS_CHECK=0    Skip GPS quality checks (HDOP, speed-accuracy, min-sat).
                       SITL JSON GPS has no quality fields.

    EK3_POS_I_GATE=50  Widen position innovation gate (default 5 -> 50 sigma).
    EK3_VEL_I_GATE=50  Widen velocity innovation gate. Required at 82 deg tilt —
                       innovations are larger than at normal hover attitudes.

    EK3_IMU_MASK=1     Single EKF3 core (IMU0 only). Eliminates Roll/Pitch
                       inconsistency warnings when only one IMU is simulated.

    ATC_RAT_RLL/PIT/YAW_IMAX=0   Zero I-term limits. Prevents windup during
                                  kinematic phase when ArduPilot outputs are ignored.

    H_COL_MIN=1000 / H_COL_MAX=2000   Full collective servo range for Lua.
    H_RSC_MODE=1                       Passthrough: no motor speed control.

    --- Sensor input ---

    Absolutely static: same packet every lockstep step.
      pos  = p_eq (tether equilibrium from steady_state_starting.json)
      vel  = [0, 0, 0]
      gyro = [0, 0, 0]
      accel = R.T @ [0, 0, -g]  (gravity in body frame, consistent with attitude)
      rpy  = fixed from build_orb_frame(body_z=normalize(p_eq))

    body_z = normalize(p_eq) = tether direction (~82 deg from vertical, NED).
    GPS1/GPS2 are ±25 cm along body_x in NED. The RELPOSNED baseline vector
    gives ArduPilot heading = atan2(East, North) of body X.

    --- Timeline ---
      t=0-12 s   Static hold. EKF tilt aligns, GPS acquires first fix.
      t~6 s      GPS origin set.
      t~12 s     Arm.
      t~21 s     delAngBiasLearned converges (zero gyro = zero bias).
      t~21 s     "EKF3 IMU0 is using GPS" — GPS fused.
      Total ~34 s from test start.
    """
    from frames import build_orb_frame

    ss   = json.loads(_STEADY_STATE_JSON.read_text())
    p_eq = np.array(ss["pos"], dtype=float)

    bz         = p_eq / float(np.linalg.norm(p_eq))
    R          = build_orb_frame(bz)
    rpy        = _rpy_from_R(R)
    accel_body = R.T @ np.array([0.0, 0.0, -_GRAVITY])
    pos        = p_eq
    vel        = np.zeros(3)
    gyro       = np.zeros(3)

    stop_ev = threading.Event()

    def _sensor_loop():
        sitl = SITLInterface(recv_port=StackConfig.SITL_JSON_PORT)
        sitl.bind()
        try:
            while not stop_ev.is_set():
                if sitl.recv_servos() is not None:
                    sitl.send_state(
                        pos_ned=pos, vel_ned=vel,
                        rpy_rad=rpy, accel_body=accel_body, gyro_body=gyro,
                    )
        finally:
            sitl.close()

    with _sitl_stack(tmp_path, test_name=request.node.name) as ctx:
        _log = ctx.log

        worker = threading.Thread(target=_sensor_loop, daemon=True, name="sensor")
        worker.start()

        rc_stop = threading.Event()
        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=ctx.mavlink_log)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            gcs.sim_sleep(12.0)

            _log.info("Arming at t=%.1f s ...", gcs.sim_now())
            gcs.set_mode(1, timeout=5.0)
            gcs.send_rc_override({8: 1000})
            gcs.arm(force=True, timeout=8.0, rc_override={8: 1000})
            gcs.send_rc_override({3: 1700, 8: 2000})
            rc_thread = threading.Thread(
                target=_rc_keepalive,
                args=(gcs, {3: 1700, 8: 2000}, rc_stop),
                daemon=True, name="rc_keepalive",
            )
            rc_thread.start()
            _log.info("Armed at t=%.1f s -- waiting for GPS fusion ...", gcs.sim_now())

            _wait_statustext(gcs, ["is using gps"], timeout=40.0,
                             log=_log, label="gps-fuse")
            _log.info("GPS fused at t=%.1f s", gcs.sim_now())

        finally:
            rc_stop.set()
            stop_ev.set()
            worker.join(timeout=2.0)
            try:
                gcs.close()
            except Exception:
                pass
