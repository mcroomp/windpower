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

import json
import logging
import math
import sys
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
from telemetry_csv import TelRow, write_csv

_STEADY_STATE_JSON = _SIM_DIR / "steady_state_starting.json"

log = logging.getLogger("test_gps_layers")

_GRAVITY  = 9.81
_T_ARM    = 12.0   # sim-seconds before arming
_TIMEOUT  = 60.0   # sim-seconds total before failing


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
    the vehicle to turn. A stationary body never satisfies this -- all EKFGSF
    hypotheses predict identical NED velocities -- so GPS never fuses.

    With EK3_SRC1_YAW=2 (dual-antenna RELPOSNED), yaw is derived directly from
    the NED baseline vector between the two antennas. Yaw is known from the
    very first GPS fix -- no motion required.

    delAngBiasLearned also converges without any motion. With constant-zero
    gyro input, the EKF bias estimator simply converges to zero bias. GPS fuses
    at ~34 s from test start.

    --- Sensor input ---

    Absolutely static: same packet every lockstep step.
      pos  = p_eq (tether equilibrium from steady_state_starting.json)
      vel  = [0, 0, 0]
      gyro = [0, 0, 0]
      accel = R.T @ [0, 0, -g]  (gravity in body frame, consistent with attitude)
      rpy  = fixed from build_orb_frame(body_z=normalize(p_eq))

    body_z = normalize(p_eq) = tether direction (~82 deg from vertical, NED).
    GPS1/GPS2 are +-25 cm along body_x in NED. The RELPOSNED baseline vector
    gives ArduPilot heading = atan2(East, North) of body X.

    --- Timeline ---
      t=0-12 s   Static hold. EKF tilt aligns, GPS acquires first fix.
      t~6 s      GPS origin set.
      t~12 s     Arm.
      t~21 s     delAngBiasLearned converges (zero gyro = zero bias).
      t~21 s     "EKF3 IMU0 is using GPS" -- GPS fused.
      Total ~34 s from test start.
    """
    from pymavlink import mavutil

    ss  = json.loads(_STEADY_STATE_JSON.read_text())
    pos = np.array(ss["pos"], dtype=float)
    R   = np.array(ss["R0"], dtype=float).reshape(3, 3)
    bz  = R[:, 2]

    rpy        = _rpy_from_R(R)
    accel_body = R.T @ np.array([0.0, 0.0, -_GRAVITY])
    vel        = np.zeros(3)
    gyro       = np.zeros(3)

    with _sitl_stack(tmp_path, test_name=request.node.name) as ctx:
        gcs  = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=ctx.mavlink_log)
        sitl = SITLInterface(recv_port=StackConfig.SITL_JSON_PORT)
        sitl.bind()

        tel_rows:  list[TelRow] = []
        last_tel_t = -1.0
        last_rc_t  = -999.0
        last_arm_t = -999.0
        state      = "PRE_ARM"

        try:
            gcs.connect_nowait()
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            while True:
                if sitl.recv_servos() is None:
                    pytest.fail("SITL stopped responding (recv_servos timeout)")

                t = sitl._sim_time_s
                sitl.send_state(
                    pos_ned=pos, vel_ned=vel,
                    rpy_rad=rpy, accel_body=accel_body, gyro_body=gyro,
                )

                # --- 1 Hz telemetry ---
                if t - last_tel_t >= 1.0:
                    last_tel_t = t
                    tel_rows.append(TelRow(
                        t_sim        = t,
                        pos_x        = float(pos[0]),
                        pos_y        = float(pos[1]),
                        pos_z        = float(pos[2]),
                        vel_x        = float(vel[0]),
                        vel_y        = float(vel[1]),
                        vel_z        = float(vel[2]),
                        rpy_roll     = float(rpy[0]),
                        rpy_pitch    = float(rpy[1]),
                        rpy_yaw      = float(rpy[2]),
                        sens_vel_n   = float(vel[0]),
                        sens_vel_e   = float(vel[1]),
                        sens_vel_d   = float(vel[2]),
                        sens_accel_x = float(accel_body[0]),
                        sens_accel_y = float(accel_body[1]),
                        sens_accel_z = float(accel_body[2]),
                        sens_gyro_x  = float(gyro[0]),
                        sens_gyro_y  = float(gyro[1]),
                        sens_gyro_z  = float(gyro[2]),
                        bz_eq_x      = float(bz[0]),
                        bz_eq_y      = float(bz[1]),
                        bz_eq_z      = float(bz[2]),
                        r00=float(R[0, 0]), r01=float(R[0, 1]), r02=float(R[0, 2]),
                        r10=float(R[1, 0]), r11=float(R[1, 1]), r12=float(R[1, 2]),
                        r20=float(R[2, 0]), r21=float(R[2, 1]), r22=float(R[2, 2]),
                    ))

                if t > _TIMEOUT:
                    pytest.fail(f"GPS fusion timed out after {_TIMEOUT:.0f} s")

                # --- State machine ---
                if state == "PRE_ARM" and t >= _T_ARM:
                    ctx.log.info("Arming at t=%.1f s", t)
                    gcs._mav.mav.command_long_send(
                        gcs._target_system, gcs._target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 1.0,
                        0, 0, 0, 0, 0,
                    )
                    gcs.send_rc_override({8: 1000})
                    gcs._mav.mav.command_long_send(
                        gcs._target_system, gcs._target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                        1, 21196.0, 0, 0, 0, 0, 0,
                    )
                    last_arm_t = t
                    state = "ARMING"

                elif state == "ARMING":
                    if gcs.is_armed:
                        ctx.log.info("Armed at t=%.1f s -- waiting for GPS fusion", t)
                        gcs.send_rc_override({3: 1700, 8: 2000})
                        last_rc_t = t
                        state = "ARMED"
                    elif t - last_arm_t >= 2.0:
                        gcs._mav.mav.command_long_send(
                            gcs._target_system, gcs._target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1, 21196.0, 0, 0, 0, 0, 0,
                        )
                        last_arm_t = t

                elif state == "ARMED":
                    if t - last_rc_t >= 0.4:
                        gcs.send_rc_override({3: 1700, 8: 2000})
                        last_rc_t = t

                # --- Drain all GCS messages ---
                done = False
                while True:
                    msg = gcs._recv(blocking=False)
                    if msg is None:
                        break
                    if msg.get_type() == "STATUSTEXT":
                        text = msg.text.strip()
                        ctx.log.info("[t=%5.1f] %s", t, text)
                        if "is using gps" in text.lower():
                            ctx.log.info("GPS fused at t=%.1f s", t)
                            done = True
                            break

                if done:
                    break

        finally:
            if tel_rows:
                write_csv(tel_rows, ctx.telemetry_log)
            try:
                gcs.close()
            except Exception:
                pass
            sitl.close()
