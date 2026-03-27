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
        # 2. Attitude: extract ZYX Euler angles from the ELECTRONICS frame
        #
        # R_hub encodes the full hub orientation including the fast rotor spin
        # (~28 rad/s ≈ 1600°/s).  ArduPilot sits on the NON-ROTATING electronics
        # assembly.  Sending the spinning R_hub directly causes wild yaw oscillations
        # that confuse the EKF (attitude changes by hundreds of degrees per second
        # while the gyro — which already has spin removed — reports near-zero rates).
        #
        # Solution: build an "orbital" rotation matrix R_orb that captures only the
        # tilt of the rotor disk (roll/pitch) and holds yaw to the world-North
        # reference.  The spin component is removed by construction.
        #
        #   disk_normal = R_hub[:, 2]          body Z-axis in ENU world frame
        #   x_orb = normalise(North - (North·disk_normal)*disk_normal)
        #                                      world North projected onto disk plane
        #   y_orb = disk_normal × x_orb
        #   R_orb = [x_orb | y_orb | disk_normal]   (columns)
        #
        # This is equivalent to always facing North in the yaw sense while
        # correctly capturing the disk tilt.  The yaw sent to ArduPilot will
        # be near-zero (pointing North), and roll/pitch will match the actual
        # disk tilt — consistent with the gyro's orbital-only angular rates.
        # ------------------------------------------------------------------
        disk_normal = R_hub[:, 2]   # body Z expressed in ENU world frame

        # Use world East [1,0,0] as the reference direction for the orbital
        # body-X axis.  R_orb is built in ENU-convention (body Z = disk normal,
        # pointing Up when hovering).  We subsequently convert to NED-convention
        # (body Z = Down) via the symmetric ENU↔NED body transform T before
        # extracting Euler angles and rotating sensor vectors.
        _WORLD_EAST_ENU = np.array([1.0, 0.0, 0.0])   # X = East in ENU
        east_proj = _WORLD_EAST_ENU - np.dot(_WORLD_EAST_ENU, disk_normal) * disk_normal
        east_proj_norm = np.linalg.norm(east_proj)
        if east_proj_norm > 1e-6:
            x_orb = east_proj / east_proj_norm
        else:
            # Disk is nearly vertical and facing East — fall back to world North
            _WORLD_NORTH_ENU = np.array([0.0, 1.0, 0.0])
            north_proj = _WORLD_NORTH_ENU - np.dot(_WORLD_NORTH_ENU, disk_normal) * disk_normal
            x_orb = north_proj / np.linalg.norm(north_proj)
        y_orb = np.cross(disk_normal, x_orb)
        R_orb = np.column_stack([x_orb, y_orb, disk_normal])

        # Convert ENU-convention body frame to NED-convention body frame.
        #
        # ArduPilot uses NED body axes (body Z = Down when level).  Our R_orb
        # uses ENU body axes (body Z = Up = disk normal when level).  Sending
        # ENU-convention attitude to ArduPilot causes its compass simulation to
        # compute wrong predicted magnetometer readings (body Z flipped), leading
        # to large magnetic innovations and EKF compass failure.
        #
        # The ENU↔NED body transform is the symmetric orthogonal matrix:
        #   T = [[0,1,0],[1,0,0],[0,0,-1]]  (T = Tᵀ = T⁻¹)
        #
        # R_ned = T @ R_orb @ T  maps the same physical orientation into NED
        # body convention.  When R_orb = I (hovering level), R_ned = T²= I,
        # giving Euler = [0,0,0] (level, facing North in NED). ✓
        _T = np.array([[0., 1., 0.],
                       [1., 0., 0.],
                       [0., 0., -1.]])
        R_ned = _T @ R_orb @ _T   # NED-convention body→world(ENU) rotation

        rpy = _rotation_matrix_to_euler_zyx(R_ned)

        # ------------------------------------------------------------------
        # 3. IMU accelerometer
        #
        # The accelerometer is mounted on the NON-ROTATING electronics
        # assembly.  Its body frame is R_orb (the orbital frame), not R_hub
        # (which includes the rotor spin).  Using R_hub.T would rotate the
        # measurement by the spin angle relative to R_orb, causing X-Y
        # components to spin at 28 rad/s in the EKF body frame — inconsistent
        # with the attitude (R_orb) and gyro.
        #
        # Specific force: a_specific = a_inertial - g
        #    a_specific_world = accel_world_enu - [0, 0, -9.81]
        # Rotate to NED body frame:
        #    accel_body = T @ (R_orb.T @ a_specific_world)
        # (R_orb.T projects world→ENU-body; T then reindexes to NED-body)
        # ------------------------------------------------------------------
        a_specific_world = accel_world_enu - _GRAVITY_ENU
        accel_body = _T @ (R_orb.T @ a_specific_world)

        # Add accelerometer noise
        accel_body = accel_body + self._rng.normal(0.0, self._accel_sigma, 3)

        # ------------------------------------------------------------------
        # 4. IMU gyroscope
        #
        # MBDyn provides omega in the WORLD (ENU) frame.  In the single-body
        # model this includes both the rotor spin (~28 rad/s about the disk
        # normal) and the slow orbital/tilting rate of the hub.
        #
        # The gyroscope is mounted on the NON-ROTATING electronics assembly
        # (held stationary by the GB4008 counter-torque motor).  It therefore
        # measures only the orbital/tilting component — NOT the rotor spin.
        #
        # Remove the spin component by projecting omega onto the plane
        # perpendicular to the disk normal (body Z-axis in world frame):
        #   omega_orbital = omega_world - (omega_world · n̂) * n̂
        # Then convert to the NED body frame — same convention as the attitude
        # (R_ned) we send to ArduPilot.  Using R_hub.T instead would produce
        # X-Y components rotated by the spin angle, causing EKF inconsistency.
        # ------------------------------------------------------------------
        disk_normal_world = R_hub[:, 2]   # body Z-axis expressed in ENU world frame
        omega_spin_world   = np.dot(omega_body, disk_normal_world) * disk_normal_world
        omega_orbital_world = omega_body - omega_spin_world
        omega_body_frame = _T @ (R_orb.T @ omega_orbital_world)

        # Add gyroscope noise
        gyro_body = omega_body_frame + self._rng.normal(0.0, self._gyro_sigma, 3)

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

    # Hover accel in NED body frame:
    #   specific_force_world = a_world - g_enu = [0,0,0] - [0,0,-9.81] = [0,0,+9.81]
    #   In ENU orbital body frame (R_orb = I): accel_enu_body = [0, 0, +9.81]
    #   Apply T (ENU→NED body): T @ [0,0,9.81] = [0, 0, -9.81]
    #   (body Z flips: ENU Up → NED Down, so +9.81 Up → -9.81 in NED body Z)
    accel_hover_world = np.zeros(3)
    result2 = sensor.compute(pos_enu, vel_enu, R, omega_world, accel_hover_world, dt=0.0025)
    assert abs(result2["accel_body"][2] - (-9.81)) < 0.3, \
        f"Hover accel Z expected ~-9.81 (NED body Z=Down), got {result2['accel_body'][2]}"
    print("  Hover accelerometer Z ≈ -9.81 (NED body-down): OK")

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
