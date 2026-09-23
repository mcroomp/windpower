import math
import time

import pytest
from pymavlink import mavutil

import calibrate.run as calibrate_run
from calibrate.run import (
    _PASSIVE_PROTOCOL_SEQUENCE,
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
    _set_passive_target_to_actual,
    _wait_for_passive_ekf_settle,
    _wait_for_passive_runup,
)
from groundstation.gcs import Heartbeat, NamedValueFloat
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


def test_passive_runup_wait_uses_configured_rsc_interval(monkeypatch):
    events = []

    class Session:
        def get_param(self, name):
            return {"H_RSC_RAMP_TIME": 0.01, "H_RSC_RUNUP_TIME": 0.02}[name]

        def _recv(self, *, timeout, **_kwargs):
            events.append(timeout)
            time.sleep(timeout)
            return Heartbeat(
                type=mavutil.mavlink.MAV_TYPE_HELICOPTER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                custom_mode=1,
                system_status=mavutil.mavlink.MAV_STATE_STANDBY,
            )

    monkeypatch.setattr(calibrate_run, "_PASSIVE_RUNUP_MARGIN_S", 0.0)
    monkeypatch.setattr(calibrate_run, "decode_message", lambda message: message)

    assert _wait_for_passive_runup(Session()) is True
    assert events


def test_passive_ekf_settle_waits_for_active_heartbeat(monkeypatch):
    class Session:
        _target_system = 1
        _target_component = 1

        def send_message(self, _message):
            pass

        def _recv(self, **_kwargs):
            return Heartbeat(
                type=mavutil.mavlink.MAV_TYPE_HELICOPTER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                custom_mode=20,
                system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
            )

    monkeypatch.setattr(calibrate_run, "decode_message", lambda message: message)

    assert _wait_for_passive_ekf_settle(
        Session(),
        timeout_s=0.1,
        settle_s=0.0,
    ) is True


def test_passive_startup_runs_up_in_acro_before_capture(monkeypatch):
    events = []

    class Session:
        def set_param(self, name, value):
            events.append(("set-param", name, value))
            return True

        def set_mode(self, mode):
            events.append(("set-mode", mode))

        def send_message(self, message):
            if isinstance(message, NamedValueFloat):
                events.append(("send", message.name, message.value))

        def start_mavlog(self, path):
            events.append(("start-log", path))

        def stop_mavlog(self):
            events.append(("stop-log",))

    class Log:
        path = "passive.csv"
        n_rows = 0

        def close(self):
            events.append(("close-log",))

    monkeypatch.setattr(
        calibrate_run._RunLog,
        "open",
        lambda *_args, **_kwargs: Log(),
    )
    monkeypatch.setattr(
        calibrate_run,
        "_ensure_passive_tail_setup",
        lambda _session: events.append(("tail-setup",)),
    )
    monkeypatch.setattr(
        calibrate_run,
        "_arm",
        lambda *_args, **_kwargs: events.append(("arm",)) or True,
    )
    monkeypatch.setattr(
        calibrate_run,
        "_wait_for_passive_runup",
        lambda *_args, **_kwargs: events.append(("runup",)) or True,
    )
    monkeypatch.setattr(
        calibrate_run,
        "_capture_current_quaternion",
        lambda _session: events.append(("capture",)) or _CAPTURE_Q,
    )
    monkeypatch.setattr(
        calibrate_run,
        "_wait_for_passive_ekf_settle",
        lambda *_args, **_kwargs: events.append(("ekf-settle",)) or True,
    )
    monkeypatch.setattr(
        calibrate_run,
        "_run_observation",
        lambda *_args, **_kwargs: events.append(("observe",)),
    )
    monkeypatch.setattr(
        calibrate_run,
        "_safety_shutdown",
        lambda *_args, **_kwargs: events.append(("shutdown",)),
    )

    calibrate_run._cmd_run(
        Session(),
        ["passive", "--duration", "1", "--trim", "thr=0.342"],
    )

    acro_index = events.index(("set-mode", 1))
    arm_index = events.index(("arm",))
    runup_index = events.index(("runup",))
    passive_index = events.index(("set-param", "RAWES_MODE", 3))
    guided_index = events.index(("set-mode", 20))
    settle_index = events.index(("ekf-settle",))
    capture_index = events.index(("capture",))
    enable_index = next(
        index
        for index, event in enumerate(events)
        if event[:2] == ("send", "RAWES_PEN")
    )
    observe_index = events.index(("observe",))

    assert events.index(("set-param", "RAWES_MODE", 0)) < acro_index
    assert acro_index < arm_index < runup_index < passive_index
    assert passive_index < guided_index < settle_index < capture_index
    assert capture_index < enable_index < observe_index


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


def test_space_sets_passive_target_to_actual_attitude_without_changing_thrust():
    target = _PassiveTarget(
        initial_q=_quat_from_euler_deg(10.0, -5.0, 120.0),
        thrust=0.9,
        roll_offset_deg=20.0,
        pitch_offset_deg=-15.0,
        yaw_offset_deg=35.0,
    )
    actual_q = _quat_from_euler_deg(-12.0, 8.0, -45.0)

    messages = _set_passive_target_to_actual(target, actual_q)

    assert target.roll_offset_deg == 0.0
    assert target.pitch_offset_deg == 0.0
    assert target.yaw_offset_deg == 0.0
    assert target.thrust == 0.9
    assert target.initial_q == pytest.approx(actual_q)
    assert messages == [("RAWES_YIC", -1000.0)]


def test_automatic_protocol_sequence_returns_to_baseline_after_each_axis():
    assert _PASSIVE_PROTOCOL_SEQUENCE == (
        ("capture actual", "capture", 0.0),
        ("roll +5 deg", "roll", 5.0),
        ("attitude baseline", "attitude", 0.0),
        ("roll -5 deg", "roll", -5.0),
        ("attitude baseline", "attitude", 0.0),
        ("pitch +5 deg", "pitch", 5.0),
        ("attitude baseline", "attitude", 0.0),
        ("pitch -5 deg", "pitch", -5.0),
        ("attitude baseline", "attitude", 0.0),
        ("collective +0.05", "collective", 0.05),
        ("collective baseline", "collective", 0.0),
        ("collective -0.05", "collective", -0.05),
        ("collective baseline", "collective", 0.0),
    )


def test_passive_lua_applies_incremental_target_updates_and_preserves_yaw():
    sim = RawesLua(mode=3)
    sim.vehicle_mode = 20
    sim.healthy = True
    sim.armed = True
    sim.tick()
    sim.send_message(NamedValueFloat("RAWES_THR", 0.4))
    initial = _PassiveTarget(
        initial_q=_quat_from_euler_deg(10.0, -5.0, 120.0),
        thrust=0.4,
    )
    for name, value in _passive_target_messages(initial):
        sim.send_message(NamedValueFloat(name, value))
    sim.send_message(NamedValueFloat("RAWES_PEN", 1.0))
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


def test_passive_yaw_trim_waits_until_guided_handoff():
    sim = RawesLua(mode=3)
    sim.vehicle_mode = 1
    sim.healthy = True
    sim.armed = True
    sim.send_message(NamedValueFloat("RAWES_THR", 0.4))
    sim.send_message(NamedValueFloat("RAWES_YFF", 0.3))
    target = _PassiveTarget(
        initial_q=_quat_from_euler_deg(10.0, -5.0, 120.0),
        thrust=0.4,
    )
    for name, value in _passive_target_messages(target):
        sim.send_message(NamedValueFloat(name, value))

    sim.run(0.1)

    assert sim.guided_target is None
    assert sim.get_param("H_YAW_TRIM") == pytest.approx(0.0)

    sim.vehicle_mode = 20
    sim.run(0.1)

    assert sim.guided_target is None
    assert sim.get_param("H_YAW_TRIM") == pytest.approx(0.0)

    sim.send_message(NamedValueFloat("RAWES_PEN", 1.0))
    sim.run(0.1)

    assert sim.guided_target is not None
    assert sim.get_param("H_YAW_TRIM") == pytest.approx(0.3)
