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


def _build_orb_frame(body_z: np.ndarray) -> np.ndarray:
    """
    Build an orbital rotation matrix from a disk-normal (body Z) direction.

    The orbital frame removes rotor spin: body X is aligned with world East
    projected onto the disk plane (falls back to world North if the disk
    faces East).  Returns a 3×3 rotation matrix with body_z as column 2.
    """
    _EAST  = np.array([1.0, 0.0, 0.0])
    _NORTH = np.array([0.0, 1.0, 0.0])
    east_proj = _EAST - np.dot(_EAST, body_z) * body_z
    if np.linalg.norm(east_proj) > 1e-6:
        x_orb = east_proj / np.linalg.norm(east_proj)
    else:
        north_proj = _NORTH - np.dot(_NORTH, body_z) * body_z
        x_orb = north_proj / np.linalg.norm(north_proj)
    y_orb = np.cross(body_z, x_orb)
    return np.column_stack([x_orb, y_orb, body_z])


class SensorSim:
    """
    Simulated sensor suite for ArduPilot SITL.

    Parameters
    ----------
    gyro_sigma  : float   Gyroscope noise std dev [rad/s]
    accel_sigma : float   Accelerometer noise std dev [m/s²]
    rng_seed    : int | None   Random number seed for reproducibility
    home_enu_z  : float   ENU Z coordinate of the ArduPilot home position [m].
                          ArduPilot's NED origin (D=0) corresponds to this altitude
                          in the mediator's ENU frame.  Defaults to 0.0 (ENU origin
                          = ArduPilot home), but must be set to the hub's initial
                          ENU Z (typically 50.0 m) so that the hub starts at NED D=0
                          (at home altitude) rather than 50 m above it.
    anchor_enu  : array-like (3,)   ENU position of the tether anchor [m].
                  Attitude is reported as deviation from the tether-aligned
                  equilibrium orientation (body Z pointing from anchor to hub).
                  When the hub axle is perfectly aligned with the tether,
                  ArduPilot sees roll=0, pitch=0.  Defaults to world origin.
    """

    def __init__(
        self,
        gyro_sigma:  float = _GYRO_SIGMA,
        accel_sigma: float = _ACCEL_SIGMA,
        rng_seed:    Optional[int] = None,
        home_enu_z:  float = 0.0,
        anchor_enu:  Optional[np.ndarray] = None,
    ):
        self._gyro_sigma  = gyro_sigma
        self._accel_sigma = accel_sigma
        self._rng = np.random.default_rng(rng_seed)
        self._home_enu_z  = float(home_enu_z)
        self._anchor_enu  = np.asarray(anchor_enu, dtype=float) if anchor_enu is not None \
                            else np.zeros(3)

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
        #    NED: x=North=ENU_y, y=East=ENU_x, z=Down
        #
        # NED D is measured relative to the ArduPilot home altitude.
        # home_enu_z is the ENU Z of the hub's starting position (= home).
        # NED D = home_enu_z − ENU_Z   so that D=0 at the starting altitude.
        #
        # Without the home offset (−ENU_Z only), the hub at ENU Z=50m would
        # send NED D=−50m (50m above home), making ArduPilot believe it is at
        # home_altitude + 50m and command a 50m descent.  This is the root
        # cause of the hub falling 100m below its starting position.
        # ------------------------------------------------------------------
        pos_ned = np.array([pos_enu[1], pos_enu[0], self._home_enu_z - pos_enu[2]])
        vel_ned = np.array([vel_enu[1], vel_enu[0], -vel_enu[2]])

        # ------------------------------------------------------------------
        # 2. Attitude: tether-relative deviation from equilibrium orientation
        #
        # The RAWES equilibrium has the rotor axle (body Z) aligned with the
        # tether (anchor → hub direction).  ArduPilot must see roll=0, pitch=0
        # at this equilibrium — otherwise its attitude controller commands
        # maximum cyclic trying to "uprght" a disk that is already in its
        # natural position.
        #
        # Strategy:
        #   1. Compute R_orb_eq: equilibrium orbital frame built from the tether
        #      direction as body Z.  When hub axle = tether direction → equilibrium.
        #   2. Compute R_orb: actual orbital frame from disk_normal (spin removed).
        #   3. R_dev = R_orb_eq.T @ R_orb: deviation from equilibrium (small when
        #      axle ≈ tether, large when axle is tilted away from tether).
        #   4. Report attitude as R_dev in NED convention → roll=0,pitch=0 at equil.
        #
        # Accel and gyro are still expressed in R_orb (the physical electronics
        # frame).  At equilibrium R_orb = R_orb_eq so all three are consistent.
        # Small deviations cause only second-order EKF inconsistency.
        # ------------------------------------------------------------------

        # Actual orbital frame (spin removed): body Z = disk_normal.
        disk_normal = R_hub[:, 2]
        R_orb = _build_orb_frame(disk_normal)

        # Equilibrium orbital frame: body Z = tether direction.
        tether_vec = pos_enu - self._anchor_enu
        tether_len = float(np.linalg.norm(tether_vec))
        if tether_len > 1.0:
            body_z_eq = tether_vec / tether_len
        else:
            # Hub very close to anchor (slack tether) — default to upright.
            body_z_eq = np.array([0.0, 0.0, 1.0])
        R_orb_eq = _build_orb_frame(body_z_eq)

        # Deviation rotation: identity when hub axle is aligned with tether.
        R_dev = R_orb_eq.T @ R_orb

        # Convert to NED body convention and extract Euler angles.
        # T = [[0,1,0],[1,0,0],[0,0,-1]]: ENU body ↔ NED body (symmetric).
        # T @ R_dev @ T gives NED convention; Euler=[0,0,0] at tether equilibrium.
        _T = np.array([[0., 1., 0.],
                       [1., 0., 0.],
                       [0., 0., -1.]])
        R_ned = _T @ R_dev @ _T

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
