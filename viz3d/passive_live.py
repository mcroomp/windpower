"""Live instrument-bay and rotor pose view for ``calibrate run passive``.

Run ``python -m viz3d.passive_live --demo`` to view synthetic telemetry without
connecting to flight hardware.
"""
from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np
import pyvista as pv
from matplotlib.colors import ListedColormap

from viz3d.telemetry import TelemetryFrame
from viz3d.visualize_3d import (
    R_TIP,
    _T_NED_ENU,
    _hub_to_world,
    _lerp_frame,
    _rz4,
)
from simulation.torque_model import GEAR_RATIO, RPM_SCALE


_DEFAULT_HUB_POS_NED = np.array([0.0, 0.0, -5.0])
_INTERPOLATION_S = 0.08
_MAX_RENDER_FPS = 30.0
_RENDER_INTERVAL_S = 1.0 / _MAX_RENDER_FPS
_SWASH_TRIM_US = 1517.0
_SWASH_SCALE_US = 500.0
_SWASH_DISPLAY_TRAVEL = 0.7
_SWASH_SERVO_ANGLES_DEG = (-120.0, 120.0, 0.0)
_INSTRUMENT_BAY_RADIUS_M = 0.55
_INSTRUMENT_BAY_THICKNESS_M = 0.08
_ROTOR_OFFSET_M = 0.35
_AXLE_MIN_Z_M = -0.35
_AXLE_MAX_Z_M = 0.75
_AXLE_RADIUS_M = 0.055
_INSTRUMENT_BAY_COLORS = ("#ef5350", "#8e0000")
_ROTOR_COLORS = ("#eeeeee", "#263238")
_FLIGHT_ARROW_Z_M = _INSTRUMENT_BAY_THICKNESS_M + 0.006
_KEY_FLASH_S = 0.20
_NEUTRAL_CAMERA_POSITION = (8.0, -10.0, 7.0)
_NEUTRAL_CAMERA_FOCAL_POINT = (0.0, 0.0, 5.0)
_NEUTRAL_CAMERA_UP = (0.0, 0.0, 1.0)
_KEYCAP_LAYOUT = {
    "UP": (820, 185),
    "LEFT": (760, 150),
    "DOWN": (815, 150),
    "RIGHT": (875, 150),
    ",": (815, 98),
    ".": (855, 98),
    "-": (815, 58),
    "=": (855, 58),
    "ESC": (815, 18),
    "SPACE": (875, 18),
}


def commanded_motor_rpm(motor_pwm: float | None) -> float:
    if motor_pwm is None:
        return 0.0
    throttle = float(np.clip((motor_pwm - 1000.0) / 1000.0, 0.0, 1.0))
    return throttle * RPM_SCALE * 60.0 / (2.0 * math.pi)


def rotor_rpm_for_display(
    measured_rotor_rpm: float | None, motor_pwm: float | None,
) -> float:
    if measured_rotor_rpm is not None and measured_rotor_rpm > 0.0:
        return measured_rotor_rpm
    return commanded_motor_rpm(motor_pwm) / GEAR_RATIO


def quaternion_to_rotation(q: tuple[float, float, float, float]) -> np.ndarray:
    values = np.asarray(q, dtype=float)
    norm = float(np.linalg.norm(values))
    if norm <= 1e-9:
        raise ValueError("Quaternion length is zero")
    w, x, y, z = values / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def swash_from_pwm(
    servo_pwm: tuple[float | None, float | None, float | None],
) -> tuple[float, float, float]:
    s1, s2, s3 = servo_pwm
    if s1 is None or s2 is None or s3 is None:
        return 0.0, 0.0, 0.0
    heights = np.array([
        (float(value) - _SWASH_TRIM_US) / _SWASH_SCALE_US
        * _SWASH_DISPLAY_TRAVEL
        for value in (s1, s2, s3)
    ])
    angles = np.radians(_SWASH_SERVO_ANGLES_DEG)
    design = np.column_stack([
        np.ones(3),
        -0.5 * np.cos(angles),
        -0.5 * np.sin(angles),
    ])
    collective_height, tilt_lat, tilt_lon = np.linalg.solve(design, heights)
    return collective_height / 4.0, float(tilt_lon), float(tilt_lat)


def _control_positions(
    servo_pwm: tuple[float | None, float | None, float | None],
) -> tuple[float, float, float] | None:
    """Return normalized cyclic-left/right, cyclic-up/down, and collective."""
    if any(value is None for value in servo_pwm):
        return None
    collective, tilt_lon, tilt_lat = swash_from_pwm(servo_pwm)
    cyclic_lr = float(np.clip(tilt_lat / _SWASH_DISPLAY_TRAVEL, -1.0, 1.0))
    cyclic_ud = float(np.clip(tilt_lon / _SWASH_DISPLAY_TRAVEL, -1.0, 1.0))
    collective_ud = float(np.clip(
        collective / (_SWASH_DISPLAY_TRAVEL / 4.0),
        -1.0,
        1.0,
    ))
    return cyclic_lr, cyclic_ud, collective_ud


def _control_key(axis: str, direction: int) -> str:
    return {
        ("roll", -1): "LEFT",
        ("roll", 1): "RIGHT",
        ("pitch", 1): "UP",
        ("pitch", -1): "DOWN",
        ("yaw", -1): ",",
        ("yaw", 1): ".",
        ("collective", -1): "-",
        ("collective", 1): "=",
    }[(axis, direction)]


def interpolate_rotation(
    start: np.ndarray, end: np.ndarray, alpha: float,
) -> np.ndarray:
    a = float(np.clip(alpha, 0.0, 1.0))
    blended = (1.0 - a) * start + a * end
    x_axis = blended[:, 0] / max(np.linalg.norm(blended[:, 0]), 1e-12)
    y_axis = blended[:, 1] - np.dot(blended[:, 1], x_axis) * x_axis
    y_axis /= max(np.linalg.norm(y_axis), 1e-12)
    return np.column_stack([x_axis, y_axis, np.cross(x_axis, y_axis)])


def _live_blend_alpha(elapsed_s: float) -> float:
    """Frame-rate-independent smoothing that catches up after render stalls."""
    if elapsed_s <= 0.0:
        return 0.0
    return 1.0 - math.exp(-elapsed_s / _INTERPOLATION_S)


def _checkerboard_disk(
    radius: float,
    *,
    inner_radius: float = 0.0,
    z: float = 0.0,
) -> pv.PolyData:
    """Create a disk whose cells are tagged as four alternating quadrants."""
    disk = pv.Disc(
        inner=inner_radius,
        outer=radius,
        normal=(0.0, 0.0, 1.0),
        r_res=1,
        c_res=64,
    )
    disk.translate((0.0, 0.0, z), inplace=True)
    centers = np.asarray(disk.cell_centers().points)
    angles = np.mod(np.arctan2(centers[:, 1], centers[:, 0]), 2.0 * math.pi)
    quadrants = np.floor(angles / (math.pi / 2.0)).astype(np.int8)
    disk.cell_data["checker"] = quadrants % 2
    disk.cell_data["quadrant"] = quadrants
    return disk


def _flight_direction_arrow() -> pv.PolyData:
    """Flat arrow pointing along the instrument bay's FRD +X axis."""
    points = np.array([
        [-0.36, -0.055, _FLIGHT_ARROW_Z_M],
        [0.10, -0.055, _FLIGHT_ARROW_Z_M],
        [0.10, -0.14, _FLIGHT_ARROW_Z_M],
        [0.44, 0.0, _FLIGHT_ARROW_Z_M],
        [0.10, 0.14, _FLIGHT_ARROW_Z_M],
        [0.10, 0.055, _FLIGHT_ARROW_Z_M],
        [-0.36, 0.055, _FLIGHT_ARROW_Z_M],
    ])
    return pv.PolyData(points, np.array([7, 0, 1, 2, 3, 4, 5, 6]))


def _translation_z(distance: float) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[2, 3] = distance
    return transform


def component_transforms(
    R_ned: np.ndarray,
    pos_ned: np.ndarray,
    rotor_spin_angle: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return instrument-bay, rotor, and axle transforms in the display frame."""
    R_viz = _T_NED_ENU @ np.asarray(R_ned, dtype=float)
    pos_viz = _T_NED_ENU @ np.asarray(pos_ned, dtype=float)
    bay_transform = _hub_to_world(R_viz, pos_viz)
    rotor_transform = (
        bay_transform
        @ _translation_z(_ROTOR_OFFSET_M)
        @ _rz4(rotor_spin_angle)
    )
    return bay_transform, rotor_transform, bay_transform


def make_frame(
    *,
    t: float,
    actual_q: tuple[float, float, float, float],
    target_q: tuple[float, float, float, float] | None,
    servo_pwm: tuple[float | None, float | None, float | None],
    rotor_rpm: float,
    pos_ned: tuple[float, float, float] | None = None,
) -> TelemetryFrame:
    actual_R = quaternion_to_rotation(actual_q)
    target_body_z = (
        quaternion_to_rotation(target_q)[:, 2]
        if target_q is not None
        else actual_R[:, 2]
    )
    collective, tilt_lon, tilt_lat = swash_from_pwm(servo_pwm)
    return TelemetryFrame(
        t=t,
        pos_ned=np.asarray(
            pos_ned if pos_ned is not None else _DEFAULT_HUB_POS_NED,
            dtype=float,
        ),
        R=actual_R,
        omega_spin=rotor_rpm * 2.0 * math.pi / 60.0,
        swash_collective=collective,
        swash_tilt_lon=tilt_lon,
        swash_tilt_lat=tilt_lat,
        body_z_eq=target_body_z,
        wind_ned=np.zeros(3),
    )


@dataclass
class PassiveViewData:
    t: float
    actual_q: tuple[float, float, float, float]
    target_q: tuple[float, float, float, float] | None
    servo_pwm: tuple[float | None, float | None, float | None]
    motor_pwm: float | None
    rotor_rpm: float | None
    pos_ned: tuple[float, float, float] | None
    actual_rpy: tuple[float | None, float | None, float | None]
    target_rpy: tuple[float | None, float | None, float | None]
    target_thrust: float | None
    quaternion_error_deg: float | None


@dataclass
class _DemoControls:
    cyclic_lr: float = 0.0
    cyclic_ud: float = 0.0
    collective_ud: float = 0.0
    yaw_offset_deg: float = 0.0

    def adjust(self, axis: str, direction: int) -> None:
        step = 0.12 * direction
        if axis == "roll":
            self.cyclic_lr = float(np.clip(self.cyclic_lr + step, -1.0, 1.0))
        elif axis == "pitch":
            self.cyclic_ud = float(np.clip(self.cyclic_ud + step, -1.0, 1.0))
        elif axis == "collective":
            self.collective_ud = float(np.clip(
                self.collective_ud + step, -1.0, 1.0
            ))
        elif axis == "yaw":
            self.yaw_offset_deg = (
                self.yaw_offset_deg + 5.0 * direction + 180.0
            ) % 360.0 - 180.0
        else:
            raise ValueError(f"Unknown demo control axis: {axis}")

    def reset(self) -> None:
        self.cyclic_lr = 0.0
        self.cyclic_ud = 0.0
        self.collective_ud = 0.0


def _quaternion_from_rpy_deg(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> tuple[float, float, float, float]:
    roll, pitch, yaw = map(
        math.radians, (roll_deg, pitch_deg, yaw_deg)
    )
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def demo_data(t: float, controls: _DemoControls | None = None) -> PassiveViewData:
    """Generate a smooth fake pose and motor command for standalone display."""
    roll = 18.0 * math.sin(0.55 * t)
    pitch = 14.0 * math.sin(0.37 * t + 0.6)
    yaw = math.degrees(0.22 * t) % 360.0
    if yaw > 180.0:
        yaw -= 360.0
    if controls is not None:
        yaw += controls.yaw_offset_deg
    actual_q = _quaternion_from_rpy_deg(roll, pitch, yaw)
    motor_pwm = 1500.0 + 180.0 * math.sin(0.31 * t)
    if controls is None:
        demo_collective = 0.10 * math.sin(0.43 * t)
        demo_tilt_lon = 0.40 * math.sin(0.71 * t)
        demo_tilt_lat = 0.40 * math.cos(0.53 * t)
        target_thrust = 0.5
    else:
        demo_collective = (
            controls.collective_ud * _SWASH_DISPLAY_TRAVEL / 4.0
        )
        demo_tilt_lon = controls.cyclic_ud * _SWASH_DISPLAY_TRAVEL
        demo_tilt_lat = controls.cyclic_lr * _SWASH_DISPLAY_TRAVEL
        target_thrust = (controls.collective_ud + 1.0) / 2.0
    servo_angles = np.radians(_SWASH_SERVO_ANGLES_DEG)
    servo_heights = (
        demo_collective * 4.0
        - 0.5 * demo_tilt_lat * np.cos(servo_angles)
        - 0.5 * demo_tilt_lon * np.sin(servo_angles)
    )
    servo_pwm = tuple(
        _SWASH_TRIM_US
        + float(height) / _SWASH_DISPLAY_TRAVEL * _SWASH_SCALE_US
        for height in servo_heights
    )
    return PassiveViewData(
        t=t,
        actual_q=actual_q,
        target_q=actual_q,
        servo_pwm=servo_pwm,
        motor_pwm=motor_pwm,
        rotor_rpm=None,
        pos_ned=(
            1.5 * math.sin(0.23 * t),
            1.0 * math.cos(0.19 * t),
            -5.0 - 0.5 * math.sin(0.29 * t),
        ),
        actual_rpy=(roll, pitch, yaw),
        target_rpy=(roll, pitch, yaw),
        target_thrust=target_thrust,
        quaternion_error_deg=0.0,
    )


class PassiveLiveView:
    def __init__(
        self,
        on_control: Callable[[str, int], None],
        on_target_actual: (
            Callable[[tuple[float, float, float, float]], None] | None
        ) = None,
    ) -> None:
        self._on_control = on_control
        self._on_target_actual = on_target_actual or (lambda _actual_q: None)
        identity = (1.0, 0.0, 0.0, 0.0)
        initial = make_frame(
            t=0.0,
            actual_q=identity,
            target_q=identity,
            servo_pwm=(None, None, None),
            rotor_rpm=0.0,
        )
        self._to_frame = initial
        self._display_frame = initial
        started = time.monotonic()
        self._spin_angle = 0.0
        self._last_render_at = started - _RENDER_INTERVAL_S
        self._last_signature = None
        self._latest_actual_q = identity
        self._rendered_frames = 0
        self._skipped_frames = 0
        self._restore_camera_after_key = False
        self._key_actors: dict[str, object] = {}
        self._key_flash_deadlines: dict[str, float] = {}
        self.abort_requested = False

        self._plotter = pv.Plotter(
            title="RAWES passive - live instrument bay and rotor",
            window_size=[1100, 760],
        )
        self._plotter.set_background("#101025")

        instrument_bay = _checkerboard_disk(_INSTRUMENT_BAY_RADIUS_M)
        instrument_bay.extrude(
            (0.0, 0.0, _INSTRUMENT_BAY_THICKNESS_M),
            capping=True,
            inplace=True,
        )
        self._instrument_bay_actor = self._plotter.add_mesh(
            instrument_bay,
            scalars="checker",
            cmap=ListedColormap(_INSTRUMENT_BAY_COLORS),
            clim=(0.0, 1.0),
            categories=True,
            show_scalar_bar=False,
            smooth_shading=False,
        )
        self._flight_arrow_actor = self._plotter.add_mesh(
            _flight_direction_arrow(),
            color="#050505",
            smooth_shading=False,
        )

        rotor_disk = _checkerboard_disk(R_TIP, inner_radius=0.14)
        self._rotor_actor = self._plotter.add_mesh(
            rotor_disk,
            scalars="checker",
            cmap=ListedColormap(_ROTOR_COLORS),
            clim=(0.0, 1.0),
            categories=True,
            show_scalar_bar=False,
            smooth_shading=False,
        )

        axle_height = _AXLE_MAX_Z_M - _AXLE_MIN_Z_M
        axle = pv.Cylinder(
            center=(0.0, 0.0, (_AXLE_MIN_Z_M + _AXLE_MAX_Z_M) / 2.0),
            direction=(0.0, 0.0, 1.0),
            radius=_AXLE_RADIUS_M,
            height=axle_height,
            resolution=32,
        )
        self._axle_actor = self._plotter.add_mesh(
            axle, color="#b0bec5", smooth_shading=True
        )

        initial_bay, initial_rotor, initial_axle = component_transforms(
            initial.R, initial.pos_ned, 0.0
        )
        self._instrument_bay_actor.user_matrix = initial_bay
        self._flight_arrow_actor.user_matrix = initial_bay
        self._rotor_actor.user_matrix = initial_rotor
        self._axle_actor.user_matrix = initial_axle
        self._hud = self._plotter.add_text(
            "Waiting for telemetry...",
            position=(10, 500),
            font_size=10,
            color="white",
            shadow=True,
        )
        circle_angle = np.linspace(0.0, 2.0 * math.pi, 121)
        self._cyclic_chart = pv.Chart2D(
            size=(0.22, 0.32),
            loc=(0.70, 0.48),
        )
        self._cyclic_chart.title = "CYCLIC\nUP"
        self._cyclic_chart.background_color = "#101025"
        self._cyclic_chart.border_color = "#aaaaaa"
        self._cyclic_chart.x_range = (-1.15, 1.15)
        self._cyclic_chart.y_range = (-1.15, 1.15)
        self._cyclic_chart.line(
            np.cos(circle_angle),
            np.sin(circle_angle),
            color="#dddddd",
            width=3.0,
        )
        self._cyclic_chart.line(
            (-1.0, 1.0), (0.0, 0.0), color="#666666", width=1.0
        )
        self._cyclic_chart.line(
            (0.0, 0.0), (-1.0, 1.0), color="#666666", width=1.0
        )
        self._cyclic_dot = self._cyclic_chart.scatter(
            (0.0,), (0.0,), color="#ffca28", size=22, style="o"
        )
        self._cyclic_chart.x_axis.label = "LEFT                 RIGHT"
        self._cyclic_chart.y_axis.label = "DOWN"
        self._cyclic_chart.x_axis.tick_labels_visible = False
        self._cyclic_chart.y_axis.tick_labels_visible = False
        self._cyclic_chart.x_axis.grid = False
        self._cyclic_chart.y_axis.grid = False

        self._collective_chart = pv.Chart2D(
            size=(0.07, 0.32),
            loc=(0.92, 0.48),
        )
        self._collective_chart.title = "COLLECTIVE\nUP"
        self._collective_chart.background_color = "#101025"
        self._collective_chart.border_color = "#aaaaaa"
        self._collective_chart.x_range = (-0.5, 0.5)
        self._collective_chart.y_range = (-1.15, 1.15)
        self._collective_chart.line(
            (0.0, 0.0), (-1.0, 1.0), color="#dddddd", width=5.0
        )
        self._collective_dot = self._collective_chart.scatter(
            (0.0,), (0.0,), color="#ffca28", size=22, style="o"
        )
        self._collective_chart.x_axis.visible = False
        self._collective_chart.y_axis.label = "DOWN"
        self._collective_chart.y_axis.tick_labels_visible = False
        self._collective_chart.y_axis.grid = False
        self._plotter.add_chart(self._cyclic_chart, self._collective_chart)
        self._plotter.set_chart_interaction(False)
        self._add_keyboard_help()

        key_map = {
            "Left": ("roll", -1),
            "Right": ("roll", 1),
            "Up": ("pitch", 1),
            "Down": ("pitch", -1),
            "comma": ("yaw", -1),
            "less": ("yaw", -1),
            "period": ("yaw", 1),
            "greater": ("yaw", 1),
            "minus": ("collective", -1),
            "equal": ("collective", 1),
        }
        for key, change in key_map.items():
            self._plotter.add_key_event(
                key, lambda c=change: self._activate_control(c[0], c[1])
            )
        self._plotter.add_key_event("space", self._set_target_to_actual)
        self._plotter.add_key_event("Escape", self._request_abort)
        self._set_neutral_camera()
        self._plotter.show(interactive_update=True, auto_close=False)
        self._disable_default_keyboard_camera_controls()

    @staticmethod
    def _set_keycap_active(actor: object, active: bool) -> None:
        prop = actor.GetTextProperty()
        background = "#ffb300" if active else "#303040"
        foreground = "#101010" if active else "#ffffff"
        prop.SetBackgroundColor(*pv.Color(background).float_rgb)
        prop.SetColor(*pv.Color(foreground).float_rgb)

    def _add_keyboard_help(self) -> None:
        self._plotter.add_text(
            "KEYBOARD CONTROLS",
            position=(700, 218),
            font_size=9,
            color="#cccccc",
        )
        for label, position in (
            ("CYCLIC", (700, 165)),
            ("YAW", (700, 100)),
            ("COLLECTIVE", (700, 60)),
            ("EXIT", (700, 20)),
            ("TARGET=ACTUAL", (840, 20)),
        ):
            self._plotter.add_text(
                label, position=position, font_size=8, color="#aaaaaa"
            )
        for key, position in _KEYCAP_LAYOUT.items():
            actor = self._plotter.add_text(
                f" {key} ",
                position=position,
                font_size=9,
                color="#ffffff",
            )
            prop = actor.GetTextProperty()
            prop.SetBackgroundOpacity(0.95)
            prop.SetFrame(1)
            prop.SetFrameColor(*pv.Color("#999999").float_rgb)
            self._set_keycap_active(actor, False)
            self._key_actors[key] = actor

    def _flash_key(self, key: str) -> None:
        actor = self._key_actors[key]
        self._set_keycap_active(actor, True)
        self._key_flash_deadlines[key] = time.monotonic() + _KEY_FLASH_S

    def _activate_control(self, axis: str, direction: int) -> None:
        self._flash_key(_control_key(axis, direction))
        self._restore_camera_after_key = True
        self._on_control(axis, direction)

    def _set_neutral_camera(self) -> None:
        camera = self._plotter.camera
        camera.position = _NEUTRAL_CAMERA_POSITION
        camera.focal_point = _NEUTRAL_CAMERA_FOCAL_POINT
        camera.up = _NEUTRAL_CAMERA_UP

    def _set_target_to_actual(self) -> None:
        self._flash_key("SPACE")
        self._on_target_actual(self._latest_actual_q)

    def _disable_default_keyboard_camera_controls(self) -> None:
        if self._plotter.iren is None:
            return
        self._plotter.set_chart_interaction(False)
        interactor = self._plotter.iren.interactor
        interactor.RemoveObservers("KeyPressEvent")
        interactor.RemoveObservers("CharEvent")
        interactor.AddObserver(
            "KeyPressEvent",
            self._plotter.iren.key_press_event,
        )

    def _update_key_flashes(self, now: float) -> None:
        expired = [
            key for key, deadline in self._key_flash_deadlines.items()
            if now >= deadline
        ]
        for key in expired:
            self._set_keycap_active(self._key_actors[key], False)
            del self._key_flash_deadlines[key]

    def _request_abort(self) -> None:
        self._flash_key("ESC")
        self.abort_requested = True

    def _is_open(self) -> bool:
        window = self._plotter.render_window
        return window is not None and bool(window.GetGenericContext())

    def update(self, data: PassiveViewData) -> bool:
        if self.abort_requested or not self._is_open():
            return False

        self._latest_actual_q = data.actual_q
        camera = self._plotter.camera
        camera_state = (
            camera.position,
            camera.focal_point,
            camera.up,
        )
        if self._plotter.iren is not None:
            self._plotter.iren.process_events()
        if self._restore_camera_after_key:
            camera.position, camera.focal_point, camera.up = camera_state
            self._restore_camera_after_key = False

        signature = (
            data.actual_q,
            data.target_q,
            data.servo_pwm,
            data.rotor_rpm,
            data.motor_pwm,
            data.pos_ned,
        )
        now = time.monotonic()
        self._update_key_flashes(now)
        if signature != self._last_signature:
            self._to_frame = make_frame(
                t=data.t,
                actual_q=data.actual_q,
                target_q=data.target_q,
                servo_pwm=data.servo_pwm,
                rotor_rpm=rotor_rpm_for_display(data.rotor_rpm, data.motor_pwm),
                pos_ned=data.pos_ned,
            )
            self._last_signature = signature

        elapsed_since_render = now - self._last_render_at
        if elapsed_since_render < _RENDER_INTERVAL_S:
            self._skipped_frames += 1
            return self._is_open() and not self.abort_requested

        alpha = _live_blend_alpha(elapsed_since_render)
        self._display_frame = _lerp_frame(
            self._display_frame, self._to_frame, alpha
        )
        self._spin_angle += (
            self._display_frame.omega_spin * elapsed_since_render
        )
        bay_transform, rotor_transform, axle_transform = component_transforms(
            self._display_frame.R,
            self._display_frame.pos_ned,
            self._spin_angle,
        )
        self._instrument_bay_actor.user_matrix = bay_transform
        self._flight_arrow_actor.user_matrix = bay_transform
        self._rotor_actor.user_matrix = rotor_transform
        self._axle_actor.user_matrix = axle_transform
        self._hud.SetInput(self._hud_text(data))
        control_positions = _control_positions(data.servo_pwm)
        if control_positions is not None:
            cyclic_lr, cyclic_ud, collective_ud = control_positions
            self._cyclic_dot.update((cyclic_lr,), (cyclic_ud,))
            self._collective_dot.update((0.0,), (collective_ud,))
        self._plotter.render()
        self._last_render_at = now
        self._rendered_frames += 1
        return self._is_open() and not self.abort_requested

    @staticmethod
    def _hud_text(data: PassiveViewData) -> str:
        def rpy(values) -> str:
            return "  ".join(
                "  n/a" if value is None else f"{value:+6.1f}"
                for value in values
            )

        s1, s2, s3 = (
            "n/a" if value is None else f"{value:.0f}"
            for value in data.servo_pwm
        )
        motor = "n/a" if data.motor_pwm is None else f"{data.motor_pwm:.0f}"
        motor_rpm = commanded_motor_rpm(data.motor_pwm)
        rotor_rpm = rotor_rpm_for_display(data.rotor_rpm, data.motor_pwm)
        thrust = "n/a" if data.target_thrust is None else f"{data.target_thrust:.3f}"
        position = (
            ("n/a", "n/a", "n/a")
            if data.pos_ned is None
            else tuple(f"{value:+8.3f}" for value in data.pos_ned)
        )
        q = tuple(f"{value:+.5f}" for value in data.actual_q)
        qerr = (
            "n/a"
            if data.quaternion_error_deg is None
            else f"{data.quaternion_error_deg:.2f} deg"
        )
        return "\n".join([
            f"t {data.t:6.2f} s",
            "instrument bay pose (NED)",
            f"position m  N {position[0]}  E {position[1]}  D {position[2]}",
            "             roll   pitch     yaw",
            f"actual     {rpy(data.actual_rpy)}",
            f"target     {rpy(data.target_rpy)}",
            f"quaternion  w {q[0]}  x {q[1]}  y {q[2]}  z {q[3]}",
            f"q error    {qerr}",
            "",
            f"S1/S2/S3   {s1} / {s2} / {s3} us",
            f"yaw motor  {motor} us",
            f"motor cmd  {-motor_rpm:7.0f} rpm",
            f"rotor      {rotor_rpm:7.0f} rpm  ({GEAR_RATIO:.0f}:1 ratio)",
            f"thrust     {thrust}",
            "",
            "red: instrument bay   black arrow: forward   silver: axle",
        ])

    def close(self) -> None:
        if self._plotter.render_window is not None:
            self._plotter.close()


def run_demo() -> None:
    """Run the live view with generated telemetry until Esc/window close."""
    controls = _DemoControls()
    view = PassiveLiveView(controls.adjust)
    started = time.monotonic()
    next_sample = 0.0
    data = demo_data(0.0, controls)
    try:
        while True:
            elapsed = time.monotonic() - started
            if elapsed >= next_sample:
                data = demo_data(elapsed, controls)
                next_sample = elapsed + 1.0 / 25.0
            if not view.update(data):
                break
            time.sleep(1.0 / 60.0)
    finally:
        view.close()


def _main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo",
        action="store_true",
        help="show generated pose and RPM-ratio data without flight hardware",
    )
    args = parser.parse_args()
    if not args.demo:
        parser.error("specify --demo to run without flight hardware")
    run_demo()


if __name__ == "__main__":
    _main()
