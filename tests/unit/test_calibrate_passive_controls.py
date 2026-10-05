import math
import time

import pytest

import calibrate.run as calibrate_run
from linkhub_client import MessageBatch, SimClock
from linkhub_client.mav_constants import mavutil
from linkhub_client.messages import (
    Attitude,
    ExtendedSysState,
    Heartbeat,
    MavLandedState,
    NamedValueFloat,
    SetAttitudeTarget,
    StatusText,
)
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
    _wait_for_passive_land_clear,
    _wait_for_passive_mode_ack,
    _wait_for_passive_runup,
)
from simulation.rawes_lua_harness import RawesLua


_CAPTURE_Q = _quat_from_euler_deg(12.0, -7.0, 135.0)
_CLOCK = SimClock(epoch=1, time_boot_ms=1, quality=None)


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
        self.read_count = 0

    def send_message(self, message) -> None:
        self.sent.append(message)

    def current_cursor(self):
        return "v1:0"

    def read_messages(self, *_args, **_kwargs):
        self.read_count += 1
        if self.read_count == 1:
            return MessageBatch((), "v1:1", _CLOCK)
        return MessageBatch((_RawAttitudeQuaternion(),), "v1:1", _CLOCK)


def test_capture_current_attitude_uses_quaternion_telemetry():
    session = _CaptureSession()

    captured = _capture_current_quaternion(session)

    assert captured == pytest.approx(_CAPTURE_Q)
    assert len(session.sent) == 1
    assert session.read_count == 2


def test_passive_runup_wait_uses_configured_rsc_interval(monkeypatch):
    events = []

    class Session:
        def get_param(self, name):
            return {"H_RSC_RAMP_TIME": 0.01, "H_RSC_RUNUP_TIME": 0.02}[name]

        def current_cursor(self):
            return "v1:0"

        def read_messages(
            self, _after, _message_types, *, direction, wait, limit
        ):
            assert direction == "rx"
            events.append(wait)
            time.sleep(wait)
            return MessageBatch(
                (StatusText(severity=6, text="Runup Complete"),),
                "v1:1",
                _CLOCK,
            )

    monkeypatch.setattr(calibrate_run, "_PASSIVE_RUNUP_MARGIN_S", 0.0)
    monkeypatch.setattr(calibrate_run, "decode_message", lambda message: message)

    assert _wait_for_passive_runup(Session()) is True
    assert events

    class LandClearSession:
        def current_cursor(self):
            return "v1:0"

        def read_messages(
            self, _after, _message_types, *, direction, wait, limit
        ):
            assert direction == "rx"
            return MessageBatch(
                (
                    ExtendedSysState(
                        vtol_state=0,
                        landed_state=MavLandedState.IN_AIR,
                    ),
                ),
                "v1:1",
                _CLOCK,
            )

    assert _wait_for_passive_land_clear(LandClearSession()) is True


def test_passive_ekf_settle_waits_for_active_heartbeat(monkeypatch):
    messages = iter([
        Heartbeat(
            type=mavutil.mavlink.MAV_TYPE_HELICOPTER,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            custom_mode=20,
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
        ),
        Attitude(
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            rollspeed=0.0,
            pitchspeed=0.0,
            yawspeed=0.0,
        ),
    ])

    class Session:
        _target_system = 1
        _target_component = 1
        receive_types = None

        sent = []

        def send_message(self, message):
            self.sent.append(message)

        def current_cursor(self):
            return "v1:0"

        def read_messages(self, _after, message_types, **_kwargs):
            self.receive_types = message_types
            message = next(messages, None)
            return MessageBatch(
                () if message is None else (message,),
                "v1:1",
                _CLOCK,
            )

    monkeypatch.setattr(calibrate_run, "decode_message", lambda message: message)

    session = Session()
    assert _wait_for_passive_ekf_settle(
        session,
        target_q=_CAPTURE_Q,
        thrust=0.342,
        timeout_s=0.1,
        settle_s=0.0,
    ) is True
    assert "ATTITUDE" in session.receive_types
    assert any(isinstance(message, SetAttitudeTarget) for message in session.sent)


def test_passive_waits_for_lua_to_consume_rawes_mode(monkeypatch):
    messages = iter([
        StatusText(
            severity=6,
            text="RAWES: passive active",
        ),
        StatusText(severity=6, text="RAWES: mode 3 (passive) entered"),
    ])

    class Session:
        _target_system = 1
        _target_component = 1

        def send_message(self, _message):
            pass

        def read_messages(self, after, message_types, **_kwargs):
            assert after in ("v1:10", "v1:11")
            assert message_types == ["HEARTBEAT", "STATUSTEXT"]
            message = next(messages)
            next_cursor = "v1:11" if after == "v1:10" else "v1:12"
            return MessageBatch((message,), next_cursor, _CLOCK)

    monkeypatch.setattr(calibrate_run, "decode_message", lambda message: message)

    assert _wait_for_passive_mode_ack(
        Session(),
        "v1:10",
        target_q=_CAPTURE_Q,
        thrust=0.342,
    ) is True


def test_passive_ekf_settle_rejects_recorded_hardware_spin(monkeypatch):
    class Clock:
        now = 0.0

        def monotonic(self):
            self.now += 0.05
            return self.now

    heartbeat = Heartbeat(
        type=mavutil.mavlink.MAV_TYPE_HELICOPTER,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
        custom_mode=20,
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    spin = Attitude(
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        rollspeed=0.0,
        pitchspeed=0.0,
        yawspeed=6.579623222351074,
    )

    class Session:
        _target_system = 1
        _target_component = 1
        first = True

        def send_message(self, _message):
            pass

        def current_cursor(self):
            return "v1:0"

        def read_messages(self, *_args, **_kwargs):
            if self.first:
                self.first = False
                return MessageBatch((heartbeat,), "v1:1", _CLOCK)
            return MessageBatch((spin,), "v1:2", _CLOCK)

    monkeypatch.setattr(calibrate_run.time, "monotonic", Clock().monotonic)
    monkeypatch.setattr(calibrate_run, "decode_message", lambda message: message)

    assert _wait_for_passive_ekf_settle(
        Session(),
        target_q=_CAPTURE_Q,
        thrust=0.342,
        timeout_s=0.5,
        settle_s=0.1,
    ) is False


def test_passive_startup_runs_up_in_acro_before_capture(monkeypatch, tmp_path):
    events = []
    settle_options = []

    class Session:
        _target_system = 1
        _target_component = 1

        def get_param(self, name):
            return {
                "H_FLYBAR_MODE": 1.0,
                "GUID_OPTIONS": 8.0,
            }[name]

        def set_param(self, name, value):
            events.append(("set-param", name, value))
            return True

        def set_mode(self, mode):
            events.append(("set-mode", mode))

        def send_message(self, message):
            if isinstance(message, NamedValueFloat):
                events.append(("send", message.name, message.value))

        def current_cursor(self):
            return "v1:0"

        def export_mavlog(self, path, after):
            events.append(("export-log", path, after))
            return "v1:1"

    monkeypatch.setattr(
        calibrate_run,
        "_send_set_servo",
        lambda _session, channel, pwm: events.append(("servo", channel, pwm)),
    )
    monkeypatch.setattr(
        calibrate_run,
        "_take_servo4",
        lambda _session, restore_function=None: (
            events.append(("disable-motor-function", restore_function)) or 36.0
        ),
    )
    monkeypatch.setattr(
        calibrate_run,
        "_restore_servo4",
        lambda _session, saved: events.append(("restore-motor-function", saved)),
    )
    class Log:
        path = str(tmp_path / "passive.csv")
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
        lambda *_args, **kwargs: events.append(("arm", kwargs["force"])) or True,
    )
    monkeypatch.setattr(
        calibrate_run,
        "_wait_for_passive_runup",
        lambda *_args, **_kwargs: events.append(("runup",)) or True,
    )
    monkeypatch.setattr(
        calibrate_run,
        "_wait_for_passive_land_clear",
        lambda *_args, **_kwargs: events.append(("land-clear",)) or True,
    )
    monkeypatch.setattr(
        calibrate_run,
        "_wait_for_passive_mode_ack",
        lambda *_args, **_kwargs: events.append(("mode-ack",)) or True,
    )
    monkeypatch.setattr(
        calibrate_run,
        "_capture_current_quaternion",
        lambda _session: events.append(("capture",)) or _CAPTURE_Q,
    )
    monkeypatch.setattr(
        calibrate_run,
        "_wait_for_passive_ekf_settle",
        lambda *_args, **kwargs: (
            events.append(("ekf-settle",)),
            settle_options.append(kwargs),
            True,
        )[-1],
    )
    monkeypatch.setattr(
        calibrate_run,
        "_configure_passive_startup_telemetry",
        lambda _session: events.append(("startup-telemetry",)),
    )
    monkeypatch.setattr(
        calibrate_run,
        "_run_observation",
        lambda *_args, **_kwargs: events.append(("observe",)),
    )
    monkeypatch.setattr(
        calibrate_run,
        "_safety_shutdown",
        lambda *_args, **kwargs: events.append(
            ("shutdown", kwargs["skip_motor_off"])
        ),
    )

    calibrate_run._cmd_run(
        Session(),
        [
            "passive",
            "--duration", "1",
            "--force",
            "--trim", "thr=0.342",
            "--settle-rate-deg-s", "1.5",
            "--settle-time", "4",
            "--settle-timeout", "12",
        ],
    )

    acro_index = events.index(("set-mode", 1))
    arm_index = events.index(("arm", True))
    runup_index = events.index(("runup",))
    capture_index = events.index(("capture",))
    manual_index = events.index(("set-param", "RAWES_MODE", 2))
    land_clear_index = events.index(("land-clear",))
    guided_index = events.index(("set-mode", 20))
    settle_index = events.index(("ekf-settle",))
    enable_index = next(
        index
        for index, event in enumerate(events)
        if event[:2] == ("send", "RAWES_PEN")
    )
    passive_index = events.index(("set-param", "RAWES_MODE", 3))
    mode_ack_index = events.index(("mode-ack",))
    observe_index = events.index(("observe",))

    assert events.index(("set-param", "RAWES_MODE", 0)) < acro_index
    assert not any(
        event[:2] == ("set-param", "H_FLYBAR_MODE")
        for event in events
    )
    assert (
        acro_index
        < arm_index
        < runup_index
        < capture_index
        < manual_index
        < land_clear_index
        < guided_index
        < settle_index
        < enable_index
        < passive_index
        < mode_ack_index
    )
    motor_disable_index = events.index(
        ("disable-motor-function", calibrate_run._PASSIVE_TAIL_FUNCTION)
    )
    motor_off_index = events.index(("servo", calibrate_run.SERVO_MOTOR, calibrate_run.MOTOR_OFF_US))
    restore_index = events.index(("restore-motor-function", 36.0))
    assert motor_disable_index < acro_index < arm_index
    assert land_clear_index < motor_off_index < guided_index
    assert mode_ack_index < restore_index < observe_index
    assert ("shutdown", True) in events
    assert settle_options == [{
        "target_q": _CAPTURE_Q,
        "thrust": 0.342,
        "stop_requested": None,
        "timeout_s": 12.0,
        "settle_s": 4.0,
        "rate_limit_rads": pytest.approx(math.radians(1.5)),
    }]


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
        "RAWES_ROFF", "RAWES_POFF", "RAWES_YOFF",
    ]
    assert thrust_messages == [("RAWES_THR", 1.0)]

    assert dict(messages) == {
        "RAWES_ROFF": pytest.approx(math.radians(30.0)),
        "RAWES_POFF": pytest.approx(math.radians(-30.0)),
        "RAWES_YOFF": pytest.approx(0.0),
    }


def test_passive_yaw_target_wraps_across_180_degrees():
    target = _PassiveTarget(
        initial_q=_quat_from_euler_deg(0.0, 0.0, 0.0),
        thrust=0.5,
        yaw_offset_deg=178.0,
    )

    messages = _adjust_passive_target(target, "yaw", 1)

    assert target.yaw_offset_deg == pytest.approx(-177.0)
    assert messages == [
        ("RAWES_ROFF", 0.0),
        ("RAWES_POFF", 0.0),
        ("RAWES_YOFF", math.radians(-177.0)),
    ]


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
    assert target.initial_q != pytest.approx(actual_q)
    assert messages == [
        ("RAWES_ROFF", 0.0),
        ("RAWES_POFF", 0.0),
        ("RAWES_YOFF", 0.0),
    ]


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


def test_lua_stages_acro_rc_then_uses_neutral_passive_fallback():
    neutral = RawesLua(mode=0)
    neutral.tick()
    neutral_collective_pwm = neutral.ch_out[3]

    sim = RawesLua(mode=2)
    sim.vehicle_mode = 20
    sim.healthy = True
    sim.armed = True
    sim.send_message(NamedValueFloat("RAWES_RLL", 0.0))
    sim.send_message(NamedValueFloat("RAWES_PIT", 0.0))
    sim.send_message(NamedValueFloat("RAWES_COL", 0.342))
    sim.run(0.1)

    assert sim.armed
    assert sim.ch_out[1] is not None
    assert sim.ch_out[2] is not None
    assert sim.ch_out[3] != neutral_collective_pwm

    sim.send_message(NamedValueFloat("RAWES_THR", 0.342))
    sim.send_message(NamedValueFloat("RAWES_PEN", 1.0))
    sim.run(0.1)
    sim.set_param("RAWES_MODE", 3)
    sim.run(0.1)

    assert sim.ch_out[1] == 1500
    assert sim.ch_out[2] == 1500
    assert sim.ch_out[3] == neutral_collective_pwm
    assert sim.guided_target is not None


def test_passive_lua_applies_incremental_target_updates_and_preserves_yaw():
    sim = RawesLua(mode=3)
    sim.vehicle_mode = 20
    sim.healthy = True
    sim.armed = True
    sim.tick()
    sim.send_message(NamedValueFloat("RAWES_THR", 0.4))
    sim.send_message(NamedValueFloat("RAWES_ROFF", math.radians(10.0)))
    sim.send_message(NamedValueFloat("RAWES_POFF", math.radians(-5.0)))
    sim.send_message(NamedValueFloat("RAWES_YOFF", math.radians(120.0)))
    sim.send_message(NamedValueFloat("RAWES_PEN", 1.0))
    sim.run(0.1)

    sim.send_message(NamedValueFloat("RAWES_ROFF", math.radians(20.0)))
    sim.send_message(NamedValueFloat("RAWES_POFF", math.radians(-5.0)))
    sim.send_message(NamedValueFloat("RAWES_YOFF", math.radians(120.0)))
    sim.run(0.1)
    assert sim.guided_target["roll_deg"] == pytest.approx(20.0)
    sim.send_message(NamedValueFloat("RAWES_THR", 0.45))
    sim.run(0.1)

    assert sim.guided_target == pytest.approx({
        "roll_deg": 20.0,
        "pitch_deg": -5.0,
        "yaw_deg": 120.0,
        "roll_rate": 0.0,
        "pitch_rate": 0.0,
        "yaw_rate": 0.0,
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
    sim.run(0.1)

    assert sim.guided_target is None
    assert sim.get_param("H_YAW_TRIM") == pytest.approx(0.0)

    sim.vehicle_mode = 20
    sim.run(0.1)

    assert sim.guided_target is None
    assert sim.guided_rate_target is None
    assert sim.get_param("H_YAW_TRIM") == pytest.approx(0.0)

    sim.send_message(NamedValueFloat("RAWES_PEN", 1.0))
    sim.run(0.1)

    assert sim.guided_target is not None
    assert sim.get_param("H_YAW_TRIM") == pytest.approx(0.3)


def test_passive_waits_for_ground_owned_capture_near_hardware_attitude():
    roll = math.radians(90.6)
    pitch = math.radians(-35.8)
    yaw = math.radians(77.2)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    sim = RawesLua(mode=3)
    sim.vehicle_mode = 20
    sim.healthy = True
    sim.armed = True
    sim.R = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    sim.send_message(NamedValueFloat("RAWES_THR", 0.342))
    sim.run(0.1)

    assert sim.guided_target is None
    assert sim.guided_rate_target is None
    assert sim.guided_throttle is None
    assert sim.fns.passive_anchor_q() is None

    sim.send_message(NamedValueFloat("RAWES_PEN", 1.0))
    sim.run(0.1)

    assert sim.fns.passive_anchor_q() is not None
    assert sim.guided_rate_target is None
    assert sim.guided_target == pytest.approx({
        "roll_deg": 90.6,
        "pitch_deg": -35.8,
        "yaw_deg": 77.2,
        "roll_rate": 0.0,
        "pitch_rate": 0.0,
        "yaw_rate": 0.0,
        "climbrate": None,
    }, abs=1e-5)


def test_passive_anchor_is_captured_in_lua_and_offsets_are_relative():
    sim = RawesLua(mode=3)
    sim.vehicle_mode = 20
    sim.healthy = True
    sim.armed = True
    yaw = math.radians(30.0)
    sim.R = [
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ]
    sim.send_message(NamedValueFloat("RAWES_THR", 0.4))
    sim.send_message(NamedValueFloat("RAWES_PEN", 1.0))
    sim.run(0.1)

    assert sim.fns.passive_anchor_q() is not None
    assert sim.guided_target["yaw_deg"] == pytest.approx(30.0)

    sim.send_message(NamedValueFloat("RAWES_YOFF", math.radians(20.0)))
    sim.run(0.1)

    assert sim.guided_target["yaw_deg"] == pytest.approx(50.0)
