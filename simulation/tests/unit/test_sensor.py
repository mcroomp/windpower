import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sensor import SensorSim, PhysicalSensor, make_sensor, _rotation_matrix_to_euler_zyx


def test_rotation_matrix_to_euler_identity_is_zero():
    rpy = _rotation_matrix_to_euler_zyx(np.eye(3))
    np.testing.assert_allclose(rpy, np.zeros(3))


def test_sensor_compute_identity_frame_is_deterministic_without_noise():
    # anchor directly below hub so tether direction = world Z = disk_normal → deviation = 0
    # vel_enu = zeros: below the 0.1 m/s threshold so R_dev-derived yaw is used (not velocity yaw).
    pos_enu = np.array([5.0, 10.0, 50.0])
    sensor = SensorSim(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=1,
                       anchor_enu=np.array([5.0, 10.0, 0.0]))

    result = sensor.compute(
        pos_enu=pos_enu,
        vel_enu=np.zeros(3),
        R_hub=np.eye(3),
        omega_body=np.array([0.0, 0.0, 28.0]),
        accel_world_enu=np.array([0.0, 0.0, 0.0]),
        dt=0.0025,
    )

    np.testing.assert_allclose(result["pos_ned"], np.array([10.0, 5.0, -50.0]))
    np.testing.assert_allclose(result["vel_ned"], np.zeros(3))
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


def test_sensor_tether_aligned_orientation_reports_zero_attitude():
    """
    When the hub axle (body Z) is exactly aligned with the tether direction
    (anchor → hub), the reported attitude must be roll=0, pitch=0, yaw=0.
    This is the RAWES equilibrium — ArduPilot should not command cyclic correction.
    """
    import math as _math
    elev = _math.radians(30.0)
    L    = 50.0
    pos  = np.array([L * _math.cos(elev), 0.0, L * _math.sin(elev)])  # ENU

    # Build R_hub with body Z aligned with tether direction
    tether_dir = pos / np.linalg.norm(pos)
    east = np.array([1., 0., 0.])
    bx   = east - np.dot(east, tether_dir) * tether_dir
    bx  /= np.linalg.norm(bx)
    R_hub = np.column_stack([bx, np.cross(tether_dir, bx), tether_dir])

    sensor = SensorSim(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0,
                       anchor_enu=np.zeros(3))

    result = sensor.compute(
        pos_enu=pos,
        vel_enu=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_enu=np.zeros(3),
        dt=0.0025,
    )

    np.testing.assert_allclose(result["rpy"], np.zeros(3), atol=1e-10,
        err_msg="Tether-aligned axle must report zero roll/pitch/yaw to ArduPilot")


# ---------------------------------------------------------------------------
# PhysicalSensor tests
# ---------------------------------------------------------------------------

def test_physical_sensor_returns_correct_keys():
    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    result = sensor.compute(
        pos_enu=np.array([10.0, 5.0, 20.0]),
        vel_enu=np.zeros(3),
        R_hub=np.eye(3),
        omega_body=np.array([0.0, 0.0, 5.0]),
        accel_world_enu=np.zeros(3),
        dt=0.0025,
    )
    for key in ("pos_ned", "vel_ned", "rpy", "accel_body", "gyro_body"):
        assert key in result, f"Missing key: {key}"


def test_physical_sensor_pos_ned_conversion():
    # ENU [E=3, N=7, Z=20], home_enu_z=5
    sensor = PhysicalSensor(home_enu_z=5.0, gyro_sigma=0.0, accel_sigma=0.0)
    result = sensor.compute(
        pos_enu=np.array([3.0, 7.0, 20.0]),
        vel_enu=np.zeros(3),
        R_hub=np.eye(3),
        omega_body=np.zeros(3),
        accel_world_enu=np.zeros(3),
        dt=0.0025,
    )
    # NED: N=ENU_Y=7, E=ENU_X=3, D=-(ENU_Z - home)=-(20-5)=-15
    np.testing.assert_allclose(result["pos_ned"], np.array([7.0, 3.0, -15.0]))


def test_physical_sensor_tether_aligned_reports_nonzero_attitude():
    """
    PhysicalSensor reports actual orbital-frame orientation — at tether equilibrium
    with hub East of anchor at 30° elevation, the NED ZYX Euler angles are:
      roll ≈ +60°, pitch ≈ 0°, yaw ≈ 0°.
    (The axle tilts 60° from NED vertical about the EKF North axis.)
    This is deliberately NOT zero — it is the true physical orientation.
    """
    elev = math.radians(30.0)
    L    = 50.0
    pos  = np.array([L * math.cos(elev), 0.0, L * math.sin(elev)])

    tether_dir = pos / np.linalg.norm(pos)
    east = np.array([1., 0., 0.])
    bx   = east - np.dot(east, tether_dir) * tether_dir
    bx  /= np.linalg.norm(bx)
    R_hub = np.column_stack([bx, np.cross(tether_dir, bx), tether_dir])

    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    result = sensor.compute(
        pos_enu=pos,
        vel_enu=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_enu=np.zeros(3),
        dt=0.0025,
    )
    # For hub East at 30° elevation: roll ≈ 60° = π - π/3, pitch ≈ 0, yaw ≈ 0.
    expected_roll = math.pi / 3   # 60°
    np.testing.assert_allclose(result["rpy"][1], 0.0, atol=1e-10,
        err_msg="Pitch must be 0 for hub directly East of anchor")
    np.testing.assert_allclose(abs(result["rpy"][0]), expected_roll, atol=1e-6,
        err_msg="Roll must be ~60° for 30° tether elevation with hub East of anchor")


def test_physical_sensor_spin_stripped_from_gyro():
    # Pure spin about disk_normal (world Z with R_hub=I) must not appear in gyro output.
    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    result = sensor.compute(
        pos_enu=np.zeros(3),
        vel_enu=np.zeros(3),
        R_hub=np.eye(3),
        omega_body=np.array([0.0, 0.0, 20.0]),   # pure spin
        accel_world_enu=np.zeros(3),
        dt=0.0025,
    )
    np.testing.assert_allclose(result["gyro_body"], np.zeros(3), atol=1e-10)


def test_physical_sensor_accel_stationary():
    # Stationary hub: specific force must be +g in ENU → -g_z in NED body frame.
    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    result = sensor.compute(
        pos_enu=np.zeros(3),
        vel_enu=np.zeros(3),
        R_hub=np.eye(3),
        omega_body=np.zeros(3),
        accel_world_enu=np.zeros(3),  # zero world accel = gravity only
        dt=0.0025,
    )
    # With R_hub=I: R_body (orbital, identity) → body Z points Up in ENU → Down in NED.
    # specific force = 0 - g_enu = [0,0,+9.81] ENU; in NED body (T flips axes) = [0,0,-9.81].
    np.testing.assert_allclose(result["accel_body"], np.array([0.0, 0.0, -9.81]), atol=1e-6)


def test_physical_sensor_velocity_yaw_override():
    # When v_horiz > 0.05 m/s, rpy[2] must match atan2(vE, vN).
    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    vel_enu = np.array([1.0, 0.0, 0.0])   # eastward → vN=0, vE=1 → yaw=90°
    result = sensor.compute(
        pos_enu=np.zeros(3),
        vel_enu=vel_enu,
        R_hub=np.eye(3),
        omega_body=np.zeros(3),
        accel_world_enu=np.zeros(3),
        dt=0.0025,
    )
    expected_yaw = math.atan2(1.0, 0.0)   # = π/2
    np.testing.assert_allclose(result["rpy"][2], expected_yaw, atol=1e-6)


# ---------------------------------------------------------------------------
# make_sensor factory tests
# ---------------------------------------------------------------------------

def test_make_sensor_tether_relative_returns_sensorsim():
    sensor = make_sensor("tether_relative")
    assert isinstance(sensor, SensorSim)


def test_make_sensor_physical_returns_physical_sensor():
    sensor = make_sensor("physical")
    assert isinstance(sensor, PhysicalSensor)


def test_make_sensor_invalid_mode_raises():
    import pytest
    with pytest.raises(ValueError, match="Unknown sensor mode"):
        make_sensor("bogus")