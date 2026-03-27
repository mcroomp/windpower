import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sensor import SensorSim, _rotation_matrix_to_euler_zyx


def test_rotation_matrix_to_euler_identity_is_zero():
    rpy = _rotation_matrix_to_euler_zyx(np.eye(3))
    np.testing.assert_allclose(rpy, np.zeros(3))


def test_sensor_compute_identity_frame_is_deterministic_without_noise():
    sensor = SensorSim(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=1)

    result = sensor.compute(
        pos_enu=np.array([5.0, 10.0, 50.0]),
        vel_enu=np.array([1.0, 2.0, 3.0]),
        R_hub=np.eye(3),
        omega_body=np.array([0.0, 0.0, 28.0]),
        accel_world_enu=np.array([0.0, 0.0, 0.0]),
        dt=0.0025,
    )

    np.testing.assert_allclose(result["pos_ned"], np.array([10.0, 5.0, -50.0]))
    np.testing.assert_allclose(result["vel_ned"], np.array([2.0, 1.0, -3.0]))
    np.testing.assert_allclose(result["rpy"], np.zeros(3))
    # Specific force in NED body frame when hovering stationary (accel_world=0):
    #   specific = 0 - g_enu = [0,0,+9.81]; rotate ENU body → NED body (T flips Z):
    #   accel_body_ned = [0, 0, -9.81]  (gravity points "up" in NED body = negative Z)
    np.testing.assert_allclose(result["accel_body"], np.array([0.0, 0.0, -9.81]))
    # With R_hub=I the full omega [0,0,28] is pure spin (disk normal = world Z).
    # The electronics are non-rotating so spin is stripped → gyro_body = [0,0,0].
    np.testing.assert_allclose(result["gyro_body"], np.zeros(3))


def test_sensor_rotates_world_omega_into_body_frame():
    sensor = SensorSim(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=1)

    roll_90 = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ])

    result = sensor.compute(
        pos_enu=np.zeros(3),
        vel_enu=np.zeros(3),
        R_hub=roll_90,
        omega_body=np.array([0.0, 0.0, 28.0]),
        accel_world_enu=np.zeros(3),
        dt=0.0025,
    )

    # roll_90 has disk_normal = [0,-1,0] (South in ENU) — disk tilted 90° southward.
    # In NED convention South = -North = -body_X, so this is a 90° forward pitch.
    # After ENU→NED body transform: pitch=π/2, roll=0, yaw=0.
    np.testing.assert_allclose(result["rpy"], np.array([0.0, math.pi / 2, 0.0]), atol=1e-6)
    # omega_body=[0,0,28] is about world Z (Up). disk_normal=[0,-1,0] (South), so
    # dot(omega, disk_normal)=0 — no spin component is stripped. omega_orbital=[0,0,28].
    # Rotated into NED body frame: T @ (R_orb.T @ [0,0,28]) = T @ [0,28,0] = [28,0,0].
    np.testing.assert_allclose(result["gyro_body"], np.array([28.0, 0.0, 0.0]), atol=1e-6)