import numpy as np
import pytest

from viz3d.passive_live import (
    _DemoControls,
    _KEYCAP_LAYOUT,
    _NEUTRAL_CAMERA_FOCAL_POINT,
    _NEUTRAL_CAMERA_POSITION,
    _NEUTRAL_CAMERA_UP,
    _ROTOR_OFFSET_M,
    _checkerboard_disk,
    _control_key,
    _control_positions,
    _flight_direction_arrow,
    _live_blend_alpha,
    commanded_motor_rpm,
    component_transforms,
    demo_data,
    interpolate_rotation,
    make_frame,
    quaternion_to_rotation,
    rotor_rpm_for_display,
    swash_from_pwm,
)
from simulation.torque_model import GEAR_RATIO, RPM_SCALE
from viz3d.visualize_3d import _T_NED_ENU, _lerp_frame


def test_quaternion_to_rotation_preserves_orthonormal_frame():
    rotation = quaternion_to_rotation((0.5, 0.5, 0.5, 0.5))

    assert rotation.T @ rotation == pytest.approx(np.eye(3), abs=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_equal_servo_pwm_produces_level_swash():
    collective, tilt_lon, tilt_lat = swash_from_pwm((1517.0, 1517.0, 1517.0))

    assert collective == pytest.approx(0.0)
    assert tilt_lon == pytest.approx(0.0)
    assert tilt_lat == pytest.approx(0.0)


def test_equal_servo_pwm_centers_cyclic_and_collective_indicators():
    positions = _control_positions((1517.0, 1517.0, 1517.0))

    assert positions == pytest.approx((0.0, 0.0, 0.0))


def test_missing_servo_pwm_marks_controls_unavailable():
    assert _control_positions((None, None, None)) is None


def test_control_directions_map_to_their_rendered_keyboard_keys():
    assert _control_key("roll", -1) == "LEFT"
    assert _control_key("roll", 1) == "RIGHT"
    assert _control_key("pitch", 1) == "UP"
    assert _control_key("pitch", -1) == "DOWN"
    assert _control_key("yaw", -1) == ","
    assert _control_key("yaw", 1) == "."
    assert _control_key("collective", -1) == "-"
    assert _control_key("collective", 1) == "="
    assert "SPACE" in _KEYCAP_LAYOUT


def test_neutral_camera_definition_is_valid():
    assert len(_NEUTRAL_CAMERA_POSITION) == 3
    assert len(_NEUTRAL_CAMERA_FOCAL_POINT) == 3
    assert _NEUTRAL_CAMERA_POSITION != _NEUTRAL_CAMERA_FOCAL_POINT
    assert _NEUTRAL_CAMERA_UP == (0.0, 0.0, 1.0)


def test_swash_reconstruction_matches_physical_front_elevator_servo_heights():
    pwm = (1267.0, 1517.0, 1767.0)
    collective, tilt_lon, tilt_lat = swash_from_pwm(pwm)
    expected_heights = (np.asarray(pwm) - 1517.0) / 500.0 * 0.7
    angles = np.radians((-120.0, 120.0, 0.0))
    reconstructed = (
        collective * 4.0
        - 0.5 * tilt_lat * np.cos(angles)
        - 0.5 * tilt_lon * np.sin(angles)
    )

    assert reconstructed == pytest.approx(expected_heights)


def test_live_frame_contains_actual_and_target_orientation():
    actual_q = (1.0, 0.0, 0.0, 0.0)
    target_q = (2**-0.5, 0.0, 2**-0.5, 0.0)

    frame = make_frame(
        t=2.0,
        actual_q=actual_q,
        target_q=target_q,
        servo_pwm=(1517.0, 1517.0, 1517.0),
        rotor_rpm=60.0,
        pos_ned=(1.0, 2.0, -3.0),
    )

    assert frame.R == pytest.approx(np.eye(3))
    assert frame.body_z_eq == pytest.approx(np.array([1.0, 0.0, 0.0]))
    assert frame.omega_spin == pytest.approx(2.0 * np.pi)
    assert frame.pos_ned == pytest.approx(np.array([1.0, 2.0, -3.0]))


def test_motor_and_rotor_use_opposite_10_to_1_rotation():
    motor_rpm = commanded_motor_rpm(1500.0)

    assert motor_rpm == pytest.approx(
        0.5 * RPM_SCALE * 60.0 / (2.0 * np.pi)
    )
    assert rotor_rpm_for_display(None, 1500.0) == pytest.approx(
        motor_rpm / GEAR_RATIO
    )


def test_measured_rotor_rpm_takes_priority_over_command_fallback():
    assert rotor_rpm_for_display(123.0, 1500.0) == pytest.approx(123.0)


def test_demo_uses_motor_rpm_ratio_and_moving_instrument_bay_pose():
    start = demo_data(0.0)
    later = demo_data(2.0)

    assert start.rotor_rpm is None
    assert rotor_rpm_for_display(start.rotor_rpm, start.motor_pwm) == pytest.approx(
        commanded_motor_rpm(start.motor_pwm) / GEAR_RATIO
    )
    assert later.pos_ned != start.pos_ned
    assert later.actual_q != start.actual_q
    assert later.servo_pwm != start.servo_pwm


def test_demo_keyboard_controls_move_cyclic_and_collective_positions():
    controls = _DemoControls()
    controls.adjust("roll", 1)
    controls.adjust("pitch", -1)
    controls.adjust("collective", 1)
    data = demo_data(0.0, controls)

    positions = _control_positions(data.servo_pwm)
    assert positions is not None
    cyclic_lr, cyclic_ud, collective_ud = positions
    assert cyclic_lr > 0.0
    assert cyclic_ud < 0.0
    assert collective_ud > 0.0
    assert data.target_thrust > 0.5


def test_checkerboard_disk_has_four_alternating_quadrants():
    disk = _checkerboard_disk(1.0)

    assert set(disk.cell_data["quadrant"]) == {0, 1, 2, 3}
    for quadrant in range(4):
        checker_values = disk.cell_data["checker"][
            disk.cell_data["quadrant"] == quadrant
        ]
        assert set(checker_values) == {quadrant % 2}


def test_flight_direction_arrow_points_along_instrument_bay_positive_x():
    arrow = _flight_direction_arrow()
    tip = arrow.points[np.argmax(arrow.points[:, 0])]

    assert tip[0] > 0.0
    assert tip[1] == pytest.approx(0.0)
    assert np.ptp(arrow.points[:, 2]) == pytest.approx(0.0)


def test_rotor_only_rotates_and_offsets_along_instrument_bay_axle():
    R_ned = quaternion_to_rotation((0.5, 0.5, 0.5, 0.5))
    pos_ned = np.array([1.0, 2.0, -3.0])
    spin_angle = 0.75

    bay, rotor, axle = component_transforms(R_ned, pos_ned, spin_angle)

    assert axle == pytest.approx(bay)
    assert bay[:3, 3] == pytest.approx(_T_NED_ENU @ pos_ned)
    assert rotor[:3, 3] == pytest.approx(
        bay[:3, 3] + bay[:3, 2] * _ROTOR_OFFSET_M
    )
    relative_rotation = bay[:3, :3].T @ rotor[:3, :3]
    assert relative_rotation[:, 2] == pytest.approx(np.array([0.0, 0.0, 1.0]))
    assert relative_rotation[0, 0] == pytest.approx(np.cos(spin_angle))
    assert relative_rotation[1, 0] == pytest.approx(np.sin(spin_angle))


def test_frame_and_target_rotation_interpolation_are_smooth():
    start = make_frame(
        t=0.0,
        actual_q=(1.0, 0.0, 0.0, 0.0),
        target_q=(1.0, 0.0, 0.0, 0.0),
        servo_pwm=(1517.0, 1517.0, 1517.0),
        rotor_rpm=0.0,
    )
    end_R = quaternion_to_rotation((2**-0.5, 0.0, 2**-0.5, 0.0))
    end = make_frame(
        t=1.0,
        actual_q=(2**-0.5, 0.0, 2**-0.5, 0.0),
        target_q=(2**-0.5, 0.0, 2**-0.5, 0.0),
        servo_pwm=(1617.0, 1617.0, 1617.0),
        rotor_rpm=60.0,
        pos_ned=(2.0, 4.0, -6.0),
    )

    midpoint = _lerp_frame(start, end, 0.5)
    target_midpoint = interpolate_rotation(np.eye(3), end_R, 0.5)

    assert midpoint.t == pytest.approx(0.5)
    assert midpoint.swash_collective == pytest.approx(
        (start.swash_collective + end.swash_collective) / 2.0
    )
    assert midpoint.pos_ned == pytest.approx(np.array([1.0, 2.0, -5.5]))
    assert midpoint.R.T @ midpoint.R == pytest.approx(np.eye(3), abs=1e-12)
    assert target_midpoint.T @ target_midpoint == pytest.approx(
        np.eye(3), abs=1e-12
    )


def test_live_smoothing_catches_up_after_a_render_stall():
    assert _live_blend_alpha(0.0) == pytest.approx(0.0)
    assert 0.0 < _live_blend_alpha(1.0 / 30.0) < 1.0
    assert _live_blend_alpha(0.25) > 0.95
