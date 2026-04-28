"""
sensor.py — Sensor simulation for RAWES

Converts physics world-frame state (NED) into simulated sensor readings
in the body frame / NED convention expected by ArduPilot.

The physics layer (dynamics, aero, tether, controller) all operate in NED,
so no ENU↔NED conversion is required here.

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
  NED: X = North, Y = East, Z = Down  (world frame + ArduPilot convention)
  Body frame: defined by R_hub rotation matrix (columns = body axes in world NED)

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

# Gravity vector in NED world frame [m/s²]  (Z=Down so gravity is +Z)
_GRAVITY_NED = np.array([0.0, 0.0, 9.81])


def _rotation_matrix_to_euler_zyx(R: np.ndarray) -> np.ndarray:
    """
    Extract ZYX Euler angles (yaw, pitch, roll) from rotation matrix.

    R transforms from body frame to world (NED) frame:
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
    # (note: numpy is row-major, so R[row,col])

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


def _euler_zyx_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Build a ZYX rotation matrix from [roll, pitch, yaw] angles.

    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    Maps body frame to world: v_world = R @ v_body.
    Inverse of _rotation_matrix_to_euler_zyx.
    """
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,     cp*sr,             cp*cr            ],
    ])


# ---------------------------------------------------------------------------
# Physical sensor — reports actual orientation of the Pixhawk electronics
# ---------------------------------------------------------------------------

class PhysicalSensor:
    """
    Simulates the real sensor readings of the Pixhawk mounted on the
    non-rotating RAWES electronics assembly.

    Physically faithful: reports exactly what the real sensors would read.
      - Attitude (rpy): Euler angles of the electronics platform (R_orb).
      - Gyro: full body angular velocity projected into the electronics body frame.
        The GB4008 physically damps spin via K_YAW torque in the dynamics model,
        keeping the body near-zero spin. No stripping is applied here.
      - Accel: specific force projected into the electronics body frame.

    The electronics hub is the fuselage: ``R_hub`` is the full 3-DOF rotation
    matrix of the electronics platform (same as a helicopter fuselage orientation).
    The sensor reads ``R_hub`` directly — no separate yaw state is needed.
    Yaw is the third orientation DOF of ``R_hub``, integrated by the dynamics ODE
    and kept stable by the GB4008 counter-torque (modelled in the mediator).
    """

    def __init__(
        self,
        home_ned_z:  float = 0.0,
        gyro_sigma:  float = _GYRO_SIGMA,
        accel_sigma: float = _ACCEL_SIGMA,
        rng_seed:    Optional[int] = None,
    ):
        self._home_ned_z  = float(home_ned_z)
        self._gyro_sigma  = gyro_sigma
        self._accel_sigma = accel_sigma
        self._rng         = np.random.default_rng(rng_seed)

    def compute(
        self,
        pos_ned:         np.ndarray,
        vel_ned:         np.ndarray,
        R_hub:           np.ndarray,
        omega_body:      np.ndarray,
        accel_world_ned: np.ndarray,
        dt:              float = 0.0,   # retained for API compatibility; not used
    ) -> dict:
        """
        Compute physical sensor outputs.

        Returns
        -------
        pos_ned, vel_ned, rpy, accel_body, gyro_body,
        orb_yaw_rad (= rpy[2]), v_horiz_ms
        """
        # Position and velocity relative to home (already in NED)
        pos_ned_rel = np.array([pos_ned[0], pos_ned[1], pos_ned[2] - self._home_ned_z])

        # Electronics hub is the fuselage: R_hub is its full orientation.
        # Use it directly — no separate yaw state or x_orb reconstruction needed.
        R_orb = R_hub
        rpy   = _rotation_matrix_to_euler_zyx(R_hub)

        # Accelerometer: specific force projected into electronics body frame.
        a_specific_ned = accel_world_ned - _GRAVITY_NED
        accel_body = R_orb.T @ a_specific_ned
        accel_body = accel_body + self._rng.normal(0.0, self._accel_sigma, 3)

        # Gyroscope: full body angular velocity in electronics body frame.
        # omega_body is world-frame angular velocity (dynamics._omega convention).
        # No spin stripping: the GB4008 anti-rotation motor counteracts yaw via a
        # physical torque in the dynamics (K_YAW damping), keeping spin near zero
        # in free flight.  The sensor reads whatever angular velocity the body has —
        # just like a helicopter IMU reads the full body rate regardless of the
        # tail rotor.  Stripping here would double-count the motor's effect.
        gyro_body = R_orb.T @ omega_body
        gyro_body = gyro_body + self._rng.normal(0.0, self._gyro_sigma, 3)

        v_horiz = math.hypot(vel_ned[0], vel_ned[1])

        return {
            "pos_ned":     pos_ned_rel,
            "vel_ned":     vel_ned.copy(),
            "rpy":         rpy,
            "accel_body":  accel_body,
            "gyro_body":   gyro_body,
            "orb_yaw_rad": float(rpy[2]),   # actual hub orientation yaw [rad]
            "v_horiz_ms":  v_horiz,          # horizontal speed [m/s]
        }


# ---------------------------------------------------------------------------
# SpinSensor
# ---------------------------------------------------------------------------

class SpinSensor:
    """
    Simulated rotor spin rate sensor.

    In the simulation, rotor spin speed (omega_spin) is an internal ODE state.
    This class adds optional Gaussian noise to simulate measurement uncertainty.
    On hardware, omega_spin is derived from the GB4008 counter-torque motor eRPM
    reported by the AM32 ESC via telemetry, converted using the 80:44 gear ratio
    and 11 pole pairs: ``omega_spin = eRPM * 2pi/60 / 11 * 44/80``.

    Parameters
    ----------
    sigma    : float -- Gaussian noise std dev [rad/s].  0 = ideal (default).
    rng_seed : int or None -- random seed for reproducibility.
    """

    def __init__(
        self,
        sigma:    float = 0.0,
        rng_seed: "int | None" = None,
        **_ignored,
    ):
        self._sigma = float(sigma)
        self._rng   = np.random.default_rng(rng_seed)

    def measure(self, omega_true: float) -> float:
        """
        Return omega_true with optional Gaussian noise added [rad/s].
        Always returns a non-negative value.
        """
        if self._sigma > 0.0:
            return float(max(0.0, omega_true + self._rng.normal(0.0, self._sigma)))
        return float(max(0.0, omega_true))


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def make_sensor(
    home_ned_z:  float,
    gyro_sigma:  float = _GYRO_SIGMA,
    accel_sigma: float = _ACCEL_SIGMA,
    rng_seed:    Optional[int] = None,
) -> "PhysicalSensor":
    """
    Return a PhysicalSensor for the given home altitude.

    Parameters
    ----------
    home_ned_z  : NED Z of ArduPilot home (LOCAL_POSITION_NED D=0 reference) [m]
    """
    return PhysicalSensor(
        home_ned_z  = home_ned_z,
        gyro_sigma  = gyro_sigma,
        accel_sigma = accel_sigma,
        rng_seed    = rng_seed,
    )


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    print("PhysicalSensor smoke test")

    sensor = make_sensor(
        home_ned_z = -50.0,
        rng_seed   = 42,
    )

    # Test with identity rotation (no tilt)
    # In NED: hub at North=10, East=5, altitude=50 → pos_ned=[10, 5, -50]
    pos_ned = np.array([10.0, 5.0, -50.0])   # North=10, East=5, Down=-50
    R = np.eye(3)
    omega_world = np.array([0.0, 0.0, 28.0])  # spin about NED Z (down)
    accel_world = np.array([0.0, 0.0, 9.81])  # hovering (aero thrust = weight)
    vel_ned_in  = np.zeros(3)

    result = sensor.compute(
        pos_ned=pos_ned, vel_ned=vel_ned_in, R_hub=R,
        omega_body=omega_world,
        accel_world_ned=accel_world, dt=0.0025,
    )

    print(f"  pos_ned    : {result['pos_ned']}")
    print(f"  vel_ned    : {result['vel_ned']}")
    print(f"  rpy (deg)  : {np.degrees(result['rpy']).round(3)}")
    print(f"  accel_body : {result['accel_body'].round(4)}")
    print(f"  gyro_body  : {result['gyro_body'].round(4)}")
    accel_hover_world = np.zeros(3)
    result2 = sensor.compute(
        pos_ned=pos_ned, vel_ned=vel_ned_in, R_hub=R,
        omega_body=omega_world,
        accel_world_ned=accel_hover_world, dt=0.0025,
    )
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
