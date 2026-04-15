import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from frames import build_orb_frame
from sensor import PhysicalSensor, make_sensor, _rotation_matrix_to_euler_zyx


def test_rotation_matrix_to_euler_identity_is_zero():
    rpy = _rotation_matrix_to_euler_zyx(np.eye(3))
    np.testing.assert_allclose(rpy, np.zeros(3))


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
    # Stationary hub with disk_normal = NED Up ([0,0,-1]).
    # sensor.py body Z = disk_normal = NED Up ([0,0,-1]).
    # NED specific force = accel_world - g_ned = [0,0,0] - [0,0,9.81] = [0,0,-9.81].
    # Body Z = [0,0,-1] (UP): accel_body[2] = dot([0,0,-9.81], [0,0,-1]) = +9.81.
    R_hub = build_orb_frame(np.array([0.0, 0.0, -1.0]))  # disk_normal = NED up
    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    result = sensor.compute(
        pos_ned=np.zeros(3),
        vel_ned=np.zeros(3),
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_ned=np.zeros(3),  # zero world accel = gravity only
        dt=0.0025,
    )
    # body Z points UP ([0,0,-1] NED), specific force = [0,0,-9.81]: Z component = +9.81
    np.testing.assert_allclose(result["accel_body"], np.array([0.0, 0.0, 9.81]), atol=1e-6)


def test_physical_sensor_yaw_is_orb_frame_not_velocity():
    """
    rpy[2] must equal the orbital-frame yaw from R_orb — NOT the velocity heading.

    Uses disk_normal = [0.5, 0.5, -sqrt(0.5)] so that orb_yaw differs from
    velocity_yaw (velocity heading = atan2(0,1)=0 while orb_yaw > π/4).
    """
    dn = np.array([0.5, 0.5, -math.sqrt(0.5)])   # unit vector
    R_hub = build_orb_frame(dn)

    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    result = sensor.compute(
        pos_ned=np.zeros(3),
        vel_ned=np.array([1.0, 0.0, 0.0]),  # NED: pure North → vel_yaw = 0
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )
    # Expected: orb_yaw from R_orb directly
    expected_rpy = _rotation_matrix_to_euler_zyx(R_hub)
    expected_orb_yaw = expected_rpy[2]
    vel_yaw = math.atan2(0.0, 1.0)   # = 0

    # orb_yaw must differ from velocity_yaw for this test to be meaningful
    assert abs(expected_orb_yaw - vel_yaw) > 0.3, \
        "Test setup error: orb_yaw and vel_yaw are too close to distinguish"

    # rpy[2] must equal orb_yaw, NOT vel_yaw
    np.testing.assert_allclose(result["rpy"][2], expected_orb_yaw, atol=1e-6)


# ---------------------------------------------------------------------------
# Yaw DOF tests — electronics hub as fuselage
# ---------------------------------------------------------------------------

def test_physical_sensor_yaw_comes_from_R_hub():
    """
    rpy[2] must equal the ZYX yaw extracted directly from R_hub.

    The electronics hub is the fuselage: R_hub is its full orientation.
    rpy[2] must equal _rotation_matrix_to_euler_zyx(R_hub)[2] — no override,
    no convention substitution.
    """
    dn = np.array([0.5, 0.5, -math.sqrt(0.5)])
    R_hub = build_orb_frame(dn)
    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    result = sensor.compute(
        pos_ned=np.zeros(3),
        vel_ned=np.array([1.0, 0.0, 0.0]),
        R_hub=R_hub,
        omega_body=np.zeros(3),
        accel_world_ned=np.zeros(3),
        dt=0.0025,
    )
    expected_yaw = _rotation_matrix_to_euler_zyx(R_hub)[2]
    np.testing.assert_allclose(result["rpy"][2], expected_yaw, atol=1e-10,
        err_msg="rpy[2] does not match ZYX yaw from R_hub")


def test_physical_sensor_yaw_changes_with_hub_rotation():
    """
    Rotating R_hub around disk_normal (yaw DOF) must change rpy[2].

    Verifies that yaw is a real tracked DOF — two hub orientations that differ
    only in yaw around disk_normal must produce different rpy[2] values.
    """
    disk_normal = np.array([0.0, 0.0, -1.0])   # upward-pointing axle
    R_hub_0 = build_orb_frame(disk_normal)

    # Rotate R_hub_0 by 30° around disk_normal
    angle = math.radians(30.0)
    c, s = math.cos(angle), math.sin(angle)
    # Rotation matrix around NED-up axis ([0,0,-1]) by 30°:
    R_yaw30 = np.array([[ c, s, 0],
                         [-s, c, 0],
                         [ 0, 0, 1]], dtype=float)
    R_hub_30 = R_yaw30 @ R_hub_0

    sensor = PhysicalSensor(gyro_sigma=0.0, accel_sigma=0.0, rng_seed=0)
    r0 = sensor.compute(pos_ned=np.zeros(3), vel_ned=np.zeros(3), R_hub=R_hub_0,
                        omega_body=np.zeros(3), accel_world_ned=np.zeros(3), dt=0.0025)
    r30 = sensor.compute(pos_ned=np.zeros(3), vel_ned=np.zeros(3), R_hub=R_hub_30,
                         omega_body=np.zeros(3), accel_world_ned=np.zeros(3), dt=0.0025)

    delta_yaw = abs(r30["rpy"][2] - r0["rpy"][2])
    # Wrap to [0, pi]
    delta_yaw = min(delta_yaw, abs(delta_yaw - 2*math.pi))
    assert delta_yaw > 0.1, (
        f"Yaw did not change with 30-deg hub rotation around disk_normal: "
        f"delta_yaw={math.degrees(delta_yaw):.3f} deg"
    )


# ---------------------------------------------------------------------------
# make_sensor factory tests
# ---------------------------------------------------------------------------

def test_make_sensor_returns_physical_sensor():
    sensor = make_sensor(home_ned_z=0.0)
    assert isinstance(sensor, PhysicalSensor)
