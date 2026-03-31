import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from frames import build_orb_frame
from sensor import SensorSim, PhysicalSensor, make_sensor, _rotation_matrix_to_euler_zyx


def test_rotation_matrix_to_euler_identity_is_zero():
    rpy = _rotation_matrix_to_euler_zyx(np.eye(3))
    np.testing.assert_allclose(rpy, np.zeros(3))


def test_sensor_compute_tether_aligned_is_deterministic_without_noise():
    """
    Hub directly above anchor in NED. body Z points up ([0,0,-1] in NED).
    Tether direction = [0,0,-1] = up in NED. R_orb = R_orb_eq → R_dev = I → rpy=0.
    """
    # Hub at NED [10, 5, -50] (North=10, East=5, 50 m altitude)
    # Anchor at NED [10, 5,  0] (same N/E, at ground)
    anchor_ned = np.array([10.0, 5.0, 0.0])
    pos_ned    = np.array([10.0, 5.0, -50.0])
    # R_hub: body Z = up = [0,0,-1] in NED (tether direction)
    R_hub = build_orb_frame(np.array([0.0, 0.0, -1.0]))

    sensor = SensorSim(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=1,
                       anchor_ned=anchor_ned)

    result = sensor.compute(
        pos_ned=pos_ned,
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.array([0.0, 0.0, -28.0]),  # spin about NED Up = -Z
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )

    # Position relative to home (home_ned_z=0)
    np.testing.assert_allclose(result["pos_ned"], np.array([10.0, 5.0, -50.0]))
    np.testing.assert_allclose(result["vel_ned"], np.zeros(3))
    np.testing.assert_allclose(result["rpy"], np.zeros(3), atol=1e-10)

    # Stationary hover: specific force = a_world - g_ned = [0,0,0] - [0,0,9.81] = [0,0,-9.81]
    # R_body (from rpy=0) = I → accel_body = [0, 0, -9.81]
    np.testing.assert_allclose(result["accel_body"], np.array([0.0, 0.0, -9.81]))

    # omega_body = [0,0,-28]: pure spin along body Z ([0,0,-1] in NED).
    # disk_normal = [0,0,-1]. dot([0,0,-28], [0,0,-1]) = 28. omega_spin = 28*[0,0,-1] = [0,0,-28].
    # omega_orbital = [0,0,-28] - [0,0,-28] = [0,0,0] → gyro = 0.
    np.testing.assert_allclose(result["gyro_body"], np.zeros(3), atol=1e-10)


def test_sensor_spin_stripped_from_gyro():
    """Pure spin about disk normal must not appear in gyro output."""
    # disk normal = [0,0,-1] (up in NED), spin rate = 28 rad/s about this axis
    R_hub = build_orb_frame(np.array([0.0, 0.0, -1.0]))
    sensor = SensorSim(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=1)
    result = sensor.compute(
        pos_ned=np.zeros(3),
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.array([0.0, 0.0, -28.0]),  # pure spin about NED Up
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )
    np.testing.assert_allclose(result["gyro_body"], np.zeros(3), atol=1e-10)


def test_sensor_orbital_omega_appears_in_gyro():
    """Orbital (non-spin) angular velocity must appear in gyro output."""
    R_hub = build_orb_frame(np.array([0.0, 0.0, -1.0]))
    sensor = SensorSim(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=1)
    omega_orbital = np.array([0.3, 0.0, 0.0])  # orbital rate about NED North
    result = sensor.compute(
        pos_ned=np.zeros(3),
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=omega_orbital,   # no spin component
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )
    # Gyro should be non-zero (orbital rate projected into body frame)
    assert np.linalg.norm(result["gyro_body"]) > 0.1


def test_sensor_tether_aligned_orientation_reports_zero_attitude():
    """
    When the hub axle (body Z) is exactly aligned with the tether direction
    (anchor → hub), the reported attitude must be roll=0, pitch=0, yaw=0.
    This is the RAWES equilibrium — ArduPilot should not command cyclic correction.
    """
    elev = math.radians(30.0)
    L    = 50.0
    # Hub position: tethered at 30° elevation toward NED East (Y axis)
    # In NED: East = Y. Hub at [0, L*cos(elev), -L*sin(elev)].
    pos_ned = np.array([0.0, L * math.cos(elev), -L * math.sin(elev)])

    # Build R_hub with body Z aligned with tether direction (anchor→hub)
    tether_dir = pos_ned / np.linalg.norm(pos_ned)
    R_hub = build_orb_frame(tether_dir)

    sensor = SensorSim(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0,
                       anchor_ned=np.zeros(3))

    result = sensor.compute(
        pos_ned=pos_ned,
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )

    np.testing.assert_allclose(result["rpy"], np.zeros(3), atol=1e-10,
        err_msg="Tether-aligned axle must report zero roll/pitch/yaw to ArduPilot")


def test_sensor_hover_accel_is_minus_g_in_body_z():
    """
    Stationary hover with zero world acceleration: accelerometer must read
    [0, 0, -9.81] in body frame when body Z points up (NED body Z = up = negative NED).
    """
    R_hub = build_orb_frame(np.array([0.0, 0.0, -1.0]))  # body Z = up
    sensor = SensorSim(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    result = sensor.compute(
        pos_ned=np.zeros(3),
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )
    np.testing.assert_allclose(result["accel_body"], np.array([0.0, 0.0, -9.81]),
                                atol=1e-6)


def test_sensor_pos_ned_home_offset():
    """Position relative to home: home_ned_z offset is applied to NED Z."""
    # Hub at NED [0, 0, -50] (50 m altitude). home_ned_z = -50 → D=0 at hub start.
    sensor = SensorSim(gyro_sigma=0.0, accel_sigma=0.0, home_ned_z=-50.0)
    R_hub = build_orb_frame(np.array([0.0, 0.0, -1.0]))
    result = sensor.compute(
        pos_ned=np.array([0.0, 0.0, -50.0]),
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )
    # pos_ned_rel = [0, 0, -50 - (-50)] = [0, 0, 0]
    np.testing.assert_allclose(result["pos_ned"], np.array([0.0, 0.0, 0.0]))


# ---------------------------------------------------------------------------
# PhysicalSensor tests
# ---------------------------------------------------------------------------

def test_physical_sensor_returns_correct_keys():
    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    R_hub = build_orb_frame(np.array([0.0, 0.0, -1.0]))
    result = sensor.compute(
        pos_ned=np.array([0.0, 0.0, -20.0]),
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.array([0.0, 0.0, 5.0]),
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )
    for key in ("pos_ned", "vel_ned", "rpy", "accel_body", "gyro_body"):
        assert key in result, f"Missing key: {key}"


def test_physical_sensor_pos_ned_relative_to_home():
    # Hub at NED [7, 3, -20], home_ned_z = -5
    # pos_ned_rel = [7, 3, -20 - (-5)] = [7, 3, -15]
    sensor = PhysicalSensor(home_ned_z=-5.0, gyro_sigma=0.0, accel_sigma=0.0)
    R_hub = build_orb_frame(np.array([0.0, 0.0, -1.0]))
    result = sensor.compute(
        pos_ned=np.array([7.0, 3.0, -20.0]),
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )
    np.testing.assert_allclose(result["pos_ned"], np.array([7.0, 3.0, -15.0]))


def test_physical_sensor_tether_aligned_reports_nonzero_attitude():
    """
    PhysicalSensor reports actual orbital-frame orientation — at tether equilibrium
    the NED Euler angles are NOT zero (rotor axle is ~30° from NED vertical here).
    """
    elev = math.radians(30.0)
    L    = 50.0
    # Hub East of anchor at 30° elevation in NED
    pos_ned = np.array([0.0, L * math.cos(elev), -L * math.sin(elev)])

    tether_dir = pos_ned / np.linalg.norm(pos_ned)
    R_hub = build_orb_frame(tether_dir)

    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    result = sensor.compute(
        pos_ned=pos_ned,
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )
    # At 30° elevation the disk axis is 60° from NED vertical → attitude is non-zero
    rpy_deg = np.degrees(result["rpy"])
    total_tilt = math.sqrt(rpy_deg[0]**2 + rpy_deg[1]**2)
    assert total_tilt > 30.0, \
        f"PhysicalSensor should report large tilt for 30° elevation; got {rpy_deg}"


def test_physical_sensor_spin_stripped_from_gyro():
    # Pure spin about disk_normal must not appear in gyro output.
    R_hub = build_orb_frame(np.array([0.0, 0.0, -1.0]))  # body Z = NED up
    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    result = sensor.compute(
        pos_ned=np.zeros(3),
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.array([0.0, 0.0, -20.0]),   # pure spin about NED up
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )
    np.testing.assert_allclose(result["gyro_body"], np.zeros(3), atol=1e-10)


def test_physical_sensor_accel_stationary():
    # Stationary hub with body Z = NED Up ([0,0,-1]).
    # NED specific force = accel_world - g_ned = [0,0,0] - [0,0,9.81] = [0,0,-9.81].
    # Body Z = [0,0,-1] points up; the upward reaction to gravity is positive along body Z.
    # accel_body[2] = dot([0,0,-9.81], body_Z) = dot([0,0,-9.81], [0,0,-1]) = +9.81.
    R_hub = build_orb_frame(np.array([0.0, 0.0, -1.0]))  # body Z = NED up
    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    result = sensor.compute(
        pos_ned=np.zeros(3),
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_ned=np.zeros(3),  # zero world accel = gravity only
        dt=0.0025,
    )
    # With body Z pointing up, accel_body[2] = +9.81 (reaction force upward)
    np.testing.assert_allclose(result["accel_body"], np.array([0.0, 0.0, 9.81]), atol=1e-6)


def test_physical_sensor_velocity_yaw_override():
    # When v_horiz > 0.05 m/s, rpy[2] must match atan2(vE, vN).
    # NED: vE = vel_ned[1], vN = vel_ned[0].
    # vel_ned = [0, 1, 0] → vN=0, vE=1 → yaw = atan2(1, 0) = π/2
    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    R_hub = build_orb_frame(np.array([0.0, 0.0, -1.0]))
    result = sensor.compute(
        pos_ned=np.zeros(3),
        vel_ned=np.array([0.0, 1.0, 0.0]),  # NED: pure East velocity
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )
    expected_yaw = math.atan2(1.0, 0.0)   # atan2(vE, vN) = π/2
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
