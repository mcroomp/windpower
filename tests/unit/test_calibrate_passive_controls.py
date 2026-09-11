import math

import pytest

from calibrate.run import (
    _PassiveTarget,
    _adjust_passive_target,
    _capture_current_quaternion,
    _decode_flight_control_key,
    _decode_passive_control_key,
    _quat_from_euler_deg,
    _quat_multiply,
    _quat_normalize,
    _quat_to_euler_deg,
    _passive_target_messages,
)
from groundstation.gcs import NamedValueFloat
from simulation.rawes_lua_harness import RawesLua


_CAPTURE_Q = _quat_from_euler_deg(12.0, -7.0, 135.0)


class _RawAttitudeQuaternion:
    q1, q2, q3, q4 = _CAPTURE_Q
    rollspeed = 0.0
    pitchspeed = 0.0
    yawspeed = 0.0
    time_boot_ms = 0

    @staticmethod
    def get_type() -> str:
        return "ATTITUDE_QUATERNION"


class _CaptureSession:
    _target_system = 1
    _target_component = 1

    def __init__(self) -> None:
        self.sent = []

    def send_message(self, message) -> None:
        self.sent.append(message)

    def _recv(self, **_kwargs):
        return _RawAttitudeQuaternion()


def test_capture_current_attitude_uses_quaternion_telemetry():
    session = _CaptureSession()

    captured = _capture_current_quaternion(session)

    assert captured == pytest.approx(_CAPTURE_Q)
    assert len(session.sent) == 1


def test_passive_keys_match_acro_manual_layout():
    pending = [False]

    assert _decode_flight_control_key(b"\xe0", pending) is None
    assert _decode_flight_control_key(b"K", pending) == ("roll", -1)
    assert _decode_flight_control_key(b"\xe0", pending) is None
    assert _decode_flight_control_key(b"M", pending) == ("roll", 1)
    assert _decode_flight_control_key(b"\xe0", pending) is None
    assert _decode_flight_control_key(b"H", pending) == ("pitch", 1)
    assert _decode_flight_control_key(b"\xe0", pending) is None
    assert _decode_flight_control_key(b"P", pending) == ("pitch", -1)
    assert _decode_flight_control_key(b"-", pending) == ("collective", -1)
    assert _decode_flight_control_key(b"=", pending) == ("collective", 1)


def test_passive_comma_and_period_keys_adjust_yaw():
    pending = [False]

    assert _decode_passive_control_key(b",", pending) == ("yaw", -1)
    assert _decode_passive_control_key(b"<", pending) == ("yaw", -1)
    assert _decode_passive_control_key(b".", pending) == ("yaw", 1)
    assert _decode_passive_control_key(b">", pending) == ("yaw", 1)


def test_passive_target_composes_offsets_from_initial_quaternion():
    initial_q = _quat_from_euler_deg(-81.3, -82.4, 90.0)
    roll, pitch, yaw = _quat_to_euler_deg(initial_q)
    target = _PassiveTarget(
        initial_q=initial_q,
        thrust=0.98,
        roll_deg=roll,
        pitch_deg=pitch,
        yaw_deg=yaw,
    )

    for _ in range(7):
        messages = _adjust_passive_target(target, "roll", 1)
    for _ in range(7):
        messages = _adjust_passive_target(target, "pitch", -1)
    thrust_messages = _adjust_passive_target(target, "collective", 1)

    assert target.roll_offset_deg == 30.0
    assert target.pitch_offset_deg == -30.0
    assert target.thrust == 1.0
    assert [name for name, _ in messages] == [
        "RAWES_QW", "RAWES_QX", "RAWES_QY", "RAWES_QZ",
    ]
    assert thrust_messages == [("RAWES_THR", 1.0)]

    expected_q = _quat_normalize(_quat_multiply(
        initial_q, _quat_from_euler_deg(30.0, -30.0, 0.0)
    ))
    emitted_q = tuple(value for _, value in messages)
    alignment = abs(sum(a * b for a, b in zip(expected_q, emitted_q)))
    assert alignment == pytest.approx(1.0, abs=1e-7)


def test_passive_yaw_target_wraps_across_180_degrees():
    target = _PassiveTarget(
        initial_q=_quat_from_euler_deg(0.0, 0.0, 0.0),
        thrust=0.5,
        yaw_offset_deg=178.0,
    )

    messages = _adjust_passive_target(target, "yaw", 1)

    assert target.yaw_offset_deg == pytest.approx(-177.0)
    emitted_q = tuple(value for _, value in messages)
    assert emitted_q == pytest.approx(
        _quat_from_euler_deg(0.0, 0.0, -177.0)
    )


def test_passive_lua_applies_incremental_target_updates_and_preserves_yaw():
    sim = RawesLua(mode=3)
    sim.vehicle_mode = 20
    sim.healthy = True
    sim.armed = True
    sim.send_message(NamedValueFloat("RAWES_THR", 0.4))
    initial = _PassiveTarget(
        initial_q=_quat_from_euler_deg(10.0, -5.0, 120.0),
        thrust=0.4,
    )
    for name, value in _passive_target_messages(initial):
        sim.send_message(NamedValueFloat(name, value))
    sim.run(0.1)

    target = _PassiveTarget(
        initial_q=_quat_from_euler_deg(15.0, 0.0, 125.0),
        thrust=0.45,
    )
    quaternion_messages = _passive_target_messages(target)
    for name, value in quaternion_messages[:3]:
        sim.send_message(NamedValueFloat(name, value))
    sim.run(0.1)
    assert sim.guided_target["roll_deg"] == pytest.approx(10.0)

    name, value = quaternion_messages[3]
    sim.send_message(NamedValueFloat(name, value))
    sim.send_message(NamedValueFloat("RAWES_THR", 0.45))
    sim.run(0.1)

    assert sim.guided_target == pytest.approx({
        "roll_deg": 15.0,
        "pitch_deg": 0.0,
        "yaw_deg": 125.0,
        "climbrate": None,
    }, abs=1e-5)
    assert sim.guided_throttle == pytest.approx(0.45)
