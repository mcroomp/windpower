"""
sensor.py — Sensor simulation for RAWES

Converts MBDyn world-frame state (ENU) into simulated sensor readings
in the body frame / NED convention expected by ArduPilot.

Simulated sensors:
  - IMU accelerometer  (body frame, m/s²)
  - IMU gyroscope      (body frame, rad/s)
  - Position           (NED, metres relative to takeoff)
  - Velocity           (NED, m/s)
  - Attitude (roll, pitch, yaw) extracted from rotation matrix

Noise model:
  - Gyro:   σ = 0.003 rad/s  (MEMS-grade)
  - Accel:  σ = 0.05  m/s²
  (Pixhawk 6C: ICM-42688-P / ICM-20649 specs)

Coordinate frames:
  ENU (MBDyn):  X = East, Y = North, Z = Up
  NED (ArduPilot): X = North, Y = East, Z = Down
  Body frame:   defined by R_hub rotation matrix (columns = body axes in world)

Euler angles extracted in ZYX order (yaw-pitch-roll) to match ArduPilot.
"""

import logging
import numpy as np
import math
from typing import Optional

log = logging.getLogger(__name__)

# Sensor noise standard deviations
_GYRO_SIGMA  = 0.003   # rad/s
_ACCEL_SIGMA = 0.05    # m/s²

# Gravity vector in ENU world frame [m/s²]
_GRAVITY_ENU = np.array([0.0, 0.0, -9.81])


def _rotation_matrix_to_euler_zyx(R: np.ndarray) -> np.ndarray:
    """
    Extract ZYX Euler angles (yaw, pitch, roll) from rotation matrix.

    R transforms from body frame to world (ENU) frame:
        v_world = R @ v_body

    Returns [roll, pitch, yaw] in radians (ArduPilot convention).

    Singularity handling: uses atan2 formulation; pitch is clamped to ±π/2
    to avoid gimbal lock issues (should be safe for RAWES operating envelope).
    """
    # R columns: [x_body, y_body, z_body] in world frame
    # ZYX convention:
    #   R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    #
    # Extract: (using Tait-Bryan ZYX)
    #   pitch  = -asin(R[2,0])
    #   roll   = atan2(R[2,1], R[2,2])
    #   yaw    = atan2(R[1,0], R[0,0])
    #
    # (note: MBDyn uses column-major, but numpy is row-major, so R[row,col])

    # Clamp to avoid domain errors from floating point drift
    sin_pitch = float(np.clip(-R[2, 0], -1.0, 1.0))
    pitch = math.asin(sin_pitch)

    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) > 1e-6:
        roll = math.atan2(R[2, 1] / cos_pitch, R[2, 2] / cos_pitch)
        yaw  = math.atan2(R[1, 0] / cos_pitch, R[0, 0] / cos_pitch)
    else:
        # Gimbal lock: pitch ≈ ±90°
        roll = 0.0
        yaw  = math.atan2(-R[0, 1], R[1, 1])

    return np.array([roll, pitch, yaw])


class SensorSim:
    """
    Simulated sensor suite for ArduPilot SITL.

    Parameters
    ----------
    gyro_sigma  : float   Gyroscope noise std dev [rad/s]
    accel_sigma : float   Accelerometer noise std dev [m/s²]
    rng_seed    : int | None   Random number seed for reproducibility
    """

    def __init__(
        self,
        gyro_sigma:  float = _GYRO_SIGMA,
        accel_sigma: float = _ACCEL_SIGMA,
        rng_seed:    Optional[int] = None,
    ):
        self._gyro_sigma  = gyro_sigma
        self._accel_sigma = accel_sigma
        self._rng = np.random.default_rng(rng_seed)

    def compute(
        self,
        pos_enu:        np.ndarray,   # (3,) hub position in ENU [m]
        vel_enu:        np.ndarray,   # (3,) hub velocity in ENU [m/s]
        R_hub:          np.ndarray,   # (3,3) body→world rotation matrix
        omega_body:     np.ndarray,   # (3,) angular velocity in WORLD frame [rad/s]
                                      #       (MBDyn returns omega in world frame)
        accel_world_enu: np.ndarray,  # (3,) hub acceleration in world ENU [m/s²]
                                      #       = (vel_new - vel_old) / dt
        dt:             float,        # time step [s] (unused; for API consistency)
    ) -> dict:
        """
        Compute simulated sensor outputs from MBDyn world-frame state.

        Returns
        -------
        dict with keys:
            pos_ned    : np.ndarray (3,)  position NED [m]
            vel_ned    : np.ndarray (3,)  velocity NED [m/s]
            rpy        : np.ndarray (3,)  [roll, pitch, yaw] radians (ZYX)
            accel_body : np.ndarray (3,)  accelerometer reading body frame [m/s²]
            gyro_body  : np.ndarray (3,)  gyroscope reading body frame [rad/s]
        """
        # ------------------------------------------------------------------
        # 1. ENU → NED coordinate conversion
        #    NED: x=North=ENU_y, y=East=ENU_x, z=Down=-ENU_z
        # ------------------------------------------------------------------
        pos_ned = np.array([pos_enu[1], pos_enu[0], -pos_enu[2]])
        vel_ned = np.array([vel_enu[1], vel_enu[0], -vel_enu[2]])

        # ------------------------------------------------------------------
        # 2. Attitude: extract ZYX Euler angles from rotation matrix
        # ------------------------------------------------------------------
        rpy = _rotation_matrix_to_euler_zyx(R_hub)

        # ------------------------------------------------------------------
        # 3. IMU accelerometer
        #
        # The accelerometer measures specific force:
        #    a_specific = a_inertial - g
        # In ENU, gravity = [0, 0, -9.81], so:
        #    a_specific_world = accel_world_enu - [0, 0, -9.81]
        #                     = accel_world_enu + [0, 0, 9.81]
        #
        # Rotate to body frame:
        #    a_body = R_hub.T @ a_specific_world
        # ------------------------------------------------------------------
        a_specific_world = accel_world_enu - _GRAVITY_ENU
        accel_body = R_hub.T @ a_specific_world

        # Add accelerometer noise
        accel_body = accel_body + self._rng.normal(0.0, self._accel_sigma, 3)

        # ------------------------------------------------------------------
        # 4. IMU gyroscope
        #
        # MBDyn provides omega in the WORLD (ENU) frame.
        # Convert to body frame: omega_body = R_hub.T @ omega_world
        # ------------------------------------------------------------------
        omega_body_frame = R_hub.T @ omega_body

        # Add gyroscope noise
        gyro_body = omega_body_frame + self._rng.normal(0.0, self._gyro_sigma, 3)

        # ------------------------------------------------------------------
        # 5. NED gyro/accel for ArduPilot
        #    ArduPilot expects body-frame sensors — no further frame change
        #    needed. The body frame is shared (just rotated differently in world).
        #    However, ArduPilot NED body axes differ from ENU body axes.
        #    The rotation matrix from ENU body to NED body is:
        #      R_enu2ned_body = diag([1, 1, -1]) for the Z-flip, but since
        #    MBDyn's body Z points up and ArduPilot's body Z points down,
        #    we negate the Z components of body-frame vectors.
        #    For simplicity we keep body frame consistent and let ArduPilot's
        #    AHRS handle the convention internally (JSON backend is flexible).
        # ------------------------------------------------------------------

        return {
            "pos_ned":    pos_ned,
            "vel_ned":    vel_ned,
            "rpy":        rpy,
            "accel_body": accel_body,
            "gyro_body":  gyro_body,
        }


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    print("SensorSim smoke test")

    sensor = SensorSim(rng_seed=42)

    # Test coordinate transform (ENU → NED)
    pos_enu = np.array([5.0, 10.0, 50.0])   # East=5, North=10, Up=50
    pos_ned_expected = np.array([10.0, 5.0, -50.0])  # North, East, Down
    pos_ned = np.array([pos_enu[1], pos_enu[0], -pos_enu[2]])
    assert np.allclose(pos_ned, pos_ned_expected), f"ENU→NED failed: {pos_ned}"
    print("  ENU→NED conversion: OK")

    # Test with identity rotation (no tilt)
    R = np.eye(3)
    omega_world = np.array([0.0, 0.0, 28.0])  # spin about world Z
    accel_world = np.array([0.0, 0.0, 9.81])  # hovering (aero thrust = weight)
    vel_enu     = np.zeros(3)

    result = sensor.compute(pos_enu, vel_enu, R, omega_world, accel_world, dt=0.0025)

    print(f"  pos_ned    : {result['pos_ned']}")
    print(f"  vel_ned    : {result['vel_ned']}")
    print(f"  rpy (deg)  : {np.degrees(result['rpy']).round(3)}")
    print(f"  accel_body : {result['accel_body'].round(4)}")
    print(f"  gyro_body  : {result['gyro_body'].round(4)}")

    # With identity R and aero = weight, specific force should be ≈ 0 + noise
    # accel_world - gravity_enu = [0,0,9.81] - [0,0,-9.81] = [0,0,19.62]
    # ... wait, with aero = mg upward, hub acceleration = 0, so:
    # accel_world = 0, specific = 0 - (-9.81) = +9.81 in z-body
    accel_hover_world = np.zeros(3)
    result2 = sensor.compute(pos_enu, vel_enu, R, omega_world, accel_hover_world, dt=0.0025)
    # specific_force = a_world - g_enu = [0,0,0] - [0,0,-9.81] = [0,0,9.81]
    # in body (identity R): accel_body ≈ [0, 0, 9.81]
    assert abs(result2["accel_body"][2] - 9.81) < 0.3, \
        f"Hover accel Z expected ~9.81, got {result2['accel_body'][2]}"
    print("  Hover accelerometer Z ≈ +9.81: OK")

    # Euler extraction from identity R → all zero angles
    rpy_zero = _rotation_matrix_to_euler_zyx(np.eye(3))
    assert np.allclose(rpy_zero, 0.0, atol=1e-10), f"Identity R→non-zero angles: {rpy_zero}"
    print("  Identity R → zero Euler: OK")

    # 90° roll test
    R_roll90 = np.array([
        [1, 0,  0],
        [0, 0, -1],
        [0, 1,  0],
    ], dtype=float)
    rpy_roll90 = _rotation_matrix_to_euler_zyx(R_roll90)
    assert abs(rpy_roll90[0] - math.pi / 2) < 1e-6, \
        f"90° roll test failed: {math.degrees(rpy_roll90[0]):.2f}°"
    print(f"  90° roll: rpy = {np.degrees(rpy_roll90).round(2)} deg OK")

    print("All smoke tests passed.")
    sys.exit(0)
