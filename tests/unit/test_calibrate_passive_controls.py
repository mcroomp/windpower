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
    _wait_for_passive_runup,
)
from groundstation.rawes_modes import (
    CMD_ENTER_GUIDED,
    CMD_ENTER_PASSIVE,
    MAV_RESULT_ACCEPTED,
    enter_passive_params,
)
from simulation.rawes_lua_harness import RawesLua


_CAPTURE_Q = _quat_from_euler_deg(12.0, -7.0, 135.0)
_CLOCK = SimClock(epoch=1, time_boot_ms=1, quality=None)


def test_passive_runup_wait_uses_configured_rsc_interval(monkeypatch):
    events = []

    class Session:
        def get_param(self, name):
            return {"H_RSC_RAMP_TIME": 0.01, "H_RSC_RUNUP_TIME": 0.02}[name]

        def current_cursor(self):
            return "v1:0"

        def read_messages(
            self, _after, _message_types, *, direction, wait, limit,
            expected_generation=None,
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
            self, _after, _message_types, *, direction, wait, limit,
            expected_generation=None,
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
        timeout_s=0.1,
        settle_s=0.0,
    ) is True
    assert "ATTITUDE" in session.receive_types
    assert not any(isinstance(message, SetAttitudeTarget) for message in session.sent)


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
        timeout_s=0.5,
        settle_s=0.1,
    ) is False


def test_passive_startup_runs_up_in_acro_before_capture(monkeypatch, tmp_path):
    events = []
    settle_options = []

    typed_calls = []
    with monkeypatch.context() as typed_patch:
        typed_patch.setattr(
            calibrate_run,
            "_cmd_run",
            lambda session, args, **kwargs: typed_calls.append(
                (session, args, kwargs)
            ),
        )
        typed_session = object()
        calibrate_run.run_passive(
            typed_session,
            calibrate_run.PassiveRunOptions(
                duration_s=140.0,
                force=True,
                thrust=0.342,
                protocol_debug=True,
                log_dir=tmp_path,
            ),
        )
    assert len(typed_calls) == 1
    typed_session_actual, typed_args, typed_kwargs = typed_calls[0]
    assert typed_session_actual is typed_session
    assert typed_args[0] == "passive"
    assert "--force" in typed_args
    assert "--protocol-debug" in typed_args
    assert typed_kwargs["log_dir"] == tmp_path

    class TailSession:
        def __init__(self, tail_type, function):
            self.params = {
                "H_TAIL_TYPE": tail_type,
                "SERVO9_FUNCTION": function,
            }

        def get_param(self, name):
            return self.params[name]

    assert calibrate_run._ensure_passive_tail_setup(TailSession(3.0, 36.0))
    assert not calibrate_run._ensure_passive_tail_setup(TailSession(3.0, 0.0))
    assert not calibrate_run._ensure_passive_tail_setup(TailSession(0.0, 36.0))

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
        lambda _session: events.append(("tail-setup",)) or True,
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
        "_send_lua_command",
        lambda _session, command, params, _label: events.append(
            ("command", command, tuple(params))
        ) or True,
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
    manual_index = events.index(("set-param", "RAWES_MODE", 2))
    land_clear_index = events.index(("land-clear",))
    guided_index = events.index(("command", CMD_ENTER_GUIDED, ()))
    settle_index = events.index(("ekf-settle",))
    passive_index = events.index(("set-param", "RAWES_MODE", 3))
    enable_index = events.index(
        ("command", CMD_ENTER_PASSIVE, tuple(enter_passive_params()))
    )
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
        < manual_index
        < land_clear_index
        < guided_index
        < settle_index
        < passive_index
        < enable_index
        < observe_index
    )
    assert ("set-mode", 20) not in events
    assert events.index(("tail-setup",)) < acro_index < arm_index
    assert ("shutdown", True) in events
    assert settle_options == [{
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
    sim.set_param("RAWES_MODE", 3)
    sim.run(0.1)
    sim.send_command(CMD_ENTER_PASSIVE, enter_passive_params())
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
    sim.send_command(CMD_ENTER_PASSIVE, enter_passive_params())
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
    sim.run(0.1)

    assert sim.guided_target is None
    assert sim.get_param("H_YAW_TRIM") == pytest.approx(0.0)

    sim.vehicle_mode = 20
    sim.run(0.1)

    assert sim.guided_target is None
    assert sim.guided_rate_target is None
    assert sim.get_param("H_YAW_TRIM") == pytest.approx(0.0)

    sim.send_command(CMD_ENTER_PASSIVE, enter_passive_params(0.3))
    sim.set_srv_out(sim.fns.YAW_MOTOR_FUNC, 1234)
    sim.run(0.1)

    assert sim.guided_target is not None
    assert sim.get_param("H_YAW_TRIM") == pytest.approx(0.3)
    assert sim.fns.diag_nvf("YFF_U") == pytest.approx(0.234)


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

    sim.send_command(CMD_ENTER_PASSIVE, enter_passive_params())
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
    sim.send_command(CMD_ENTER_PASSIVE, enter_passive_params())
    sim.run(0.1)

    assert sim.fns.passive_anchor_q() is not None
    assert sim.guided_target["yaw_deg"] == pytest.approx(30.0)

    sim.send_message(NamedValueFloat("RAWES_YOFF", math.radians(20.0)))
    sim.run(0.1)

    assert sim.guided_target["yaw_deg"] == pytest.approx(50.0)


def _hold_attitude_r(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _acro_manual_sim():
    sim = RawesLua(mode=2)
    sim.vehicle_mode = 1
    sim.healthy = True
    sim.armed = True
    sim.R = _hold_attitude_r(-134.0, -80.0, -56.0)
    sim.send_message(NamedValueFloat("RAWES_THR", 0.342))
    sim.send_message(NamedValueFloat("RAWES_RLL", 0.0))
    sim.send_message(NamedValueFloat("RAWES_PIT", 0.0))
    sim.send_message(NamedValueFloat("RAWES_COL", 0.342))
    sim.run(0.1)
    return sim


def test_enter_guided_switches_mode_and_installs_captured_attitude_same_tick():
    sim = _acro_manual_sim()
    assert sim.guided_target is None

    sim.send_command(CMD_ENTER_GUIDED)
    sim.tick()

    assert sim.vehicle_mode == 20
    assert sim.command_acks == [{
        "command": CMD_ENTER_GUIDED,
        "result": MAV_RESULT_ACCEPTED,
        "target_system": 255,
        "target_component": 190,
    }]
    assert sim.guided_throttle == pytest.approx(0.342)
    assert sim.guided_target == pytest.approx({
        "roll_deg": -134.0,
        "pitch_deg": -80.0,
        "yaw_deg": -56.0,
        "roll_rate": 0.0,
        "pitch_rate": 0.0,
        "yaw_rate": 0.0,
        "climbrate": None,
    }, abs=1e-4)


def test_guided_entry_hold_persists_until_enter_passive_captures_anchor():
    sim = _acro_manual_sim()
    sim.send_command(CMD_ENTER_GUIDED)
    sim.run(0.1)

    # The body moves during settling; the entry hold keeps the captured target
    # (refreshed by the keepalive).
    sim.R = _hold_attitude_r(-130.0, -82.0, -50.0)
    sim._lua.execute("_mock.guided_target = nil")
    sim.run(1.1)
    assert sim.guided_target["roll_deg"] == pytest.approx(-134.0, abs=1e-4)

    sim.set_param("RAWES_MODE", 3)
    sim._lua.execute("_mock.guided_target = nil")
    sim.run(0.2)
    assert sim.guided_target["roll_deg"] == pytest.approx(-134.0, abs=1e-4)

    sim.send_command(CMD_ENTER_PASSIVE, enter_passive_params(0.0))
    sim.run(0.2)

    assert [ack["result"] for ack in sim.command_acks] == [
        MAV_RESULT_ACCEPTED, MAV_RESULT_ACCEPTED,
    ]
    assert sim.fns.passive_anchor_q() is not None
    assert sim.guided_target["roll_deg"] == pytest.approx(-130.0, abs=1e-4)
    assert sim.guided_target["yaw_deg"] == pytest.approx(-50.0, abs=1e-4)


def _guided_target_calls(sim):
    return sim._lua.eval("_mock.guided_target_calls or 0")


def test_static_holds_call_scheduler_locked_binding_only_on_change_or_keepalive():
    sim = _acro_manual_sim()
    sim.send_command(CMD_ENTER_GUIDED)
    sim.tick()
    assert _guided_target_calls(sim) == 1

    # Unchanged entry hold: only the 1 Hz keepalive.
    sim.run(2.05)
    assert _guided_target_calls(sim) == 3

    sim.set_param("RAWES_MODE", 3)
    sim.send_command(CMD_ENTER_PASSIVE, enter_passive_params(0.0))
    sim.tick()
    calls = _guided_target_calls(sim)
    sim.run(2.05)
    assert _guided_target_calls(sim) - calls == 2

    # A changed target is sent promptly.
    calls = _guided_target_calls(sim)
    sim.send_message(NamedValueFloat("RAWES_YOFF", math.radians(5.0)))
    sim.run(0.1)
    assert _guided_target_calls(sim) == calls + 1


def test_enter_guided_is_denied_outside_armed_acro_manual_staging():
    disarmed = _acro_manual_sim()
    disarmed.armed = False
    disarmed.send_command(CMD_ENTER_GUIDED)
    disarmed.tick()
    assert disarmed.command_acks[0]["result"] == 2
    assert disarmed.vehicle_mode == 1

    wrong_mode = RawesLua(mode=3)
    wrong_mode.vehicle_mode = 1
    wrong_mode.healthy = True
    wrong_mode.armed = True
    wrong_mode.send_message(NamedValueFloat("RAWES_THR", 0.342))
    wrong_mode.send_command(CMD_ENTER_GUIDED)
    wrong_mode.tick()
    assert wrong_mode.command_acks[0]["result"] == 2
    assert wrong_mode.vehicle_mode == 1

    unseeded = RawesLua(mode=2)
    unseeded.vehicle_mode = 1
    unseeded.healthy = True
    unseeded.armed = True
    unseeded.send_command(CMD_ENTER_GUIDED)
    unseeded.tick()
    assert unseeded.command_acks[0]["result"] == 2


def test_enter_guided_reports_failed_mode_change():
    sim = _acro_manual_sim()
    sim._lua.execute("_mock.set_mode_ok = false")
    sim.send_command(CMD_ENTER_GUIDED)
    sim.tick()

    assert sim.command_acks[0]["result"] == 4
    assert sim.guided_target is None


def test_enter_passive_requires_passive_mode_and_retry_does_not_recapture():
    sim = RawesLua(mode=2)
    sim.vehicle_mode = 20
    sim.healthy = True
    sim.armed = True
    sim.send_message(NamedValueFloat("RAWES_THR", 0.342))
    sim.send_command(CMD_ENTER_PASSIVE, enter_passive_params())
    sim.tick()
    assert sim.command_acks[0]["result"] == 2
    assert sim.fns.passive_anchor_q() is None

    sim.set_param("RAWES_MODE", 3)
    sim.R = _hold_attitude_r(0.0, 0.0, 30.0)
    sim.send_command(CMD_ENTER_PASSIVE, enter_passive_params())
    sim.tick()
    assert sim.command_acks[1]["result"] == MAV_RESULT_ACCEPTED

    sim.R = _hold_attitude_r(0.0, 0.0, 60.0)
    sim.send_command(CMD_ENTER_PASSIVE, enter_passive_params(), confirmation=1)
    sim.run(0.1)
    assert sim.command_acks[2]["result"] == MAV_RESULT_ACCEPTED
    assert sim.guided_target["yaw_deg"] == pytest.approx(30.0, abs=1e-4)
