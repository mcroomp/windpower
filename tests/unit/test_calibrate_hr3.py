import pytest

import calibrate.hw as calibrate_hw
import calibrate.repl as calibrate_repl
import calibrate.run as calibrate_run
from calibrate.hw import _disarm, _h3_forward_mix, verify_safe_off
from linkhub_client import MessageBatch, SimClock
from linkhub_client.mav_constants import mavutil
from linkhub_client.messages import Heartbeat, ServoOutputRaw
from calibrate.params import _config_target_params
from calibrate.repl import (
    _bounded_swash_waypoints,
    _fit_swash_range,
    _print_swash_layout,
)

_CLOCK = SimClock(epoch=1, time_boot_ms=1, quality=None)


def test_hardware_profile_uses_front_elevator_hr3_layout():
    expected = {
        "AHRS_ORIENTATION": 0.0,
        "H_SW_TYPE": 3.0,
        "H_SW_COL_DIR": 1.0,
        "SERVO1_FUNCTION": 33.0,
        "SERVO2_FUNCTION": 34.0,
        "SERVO3_FUNCTION": 35.0,
        "SERVO1_REVERSED": 1.0,
        "SERVO2_REVERSED": 1.0,
        "SERVO3_REVERSED": 1.0,
        "SERVO1_TRIM": 1517.0,
        "SERVO2_TRIM": 1517.0,
        "SERVO3_TRIM": 1517.0,
        "H_COL_MIN": 1342.0,
        "H_COL_MAX": 1657.0,
        "H_COL_ANG_MIN": -2.0,
        "H_COL_ANG_MAX": 12.0,
        "H_CYC_MAX": 1000.0,
        "H_FLYBAR_MODE": 1.0,
        "IM_ACRO_COL_EXP": 0.0,
        "INS_POS1_X": -0.08,
        "INS_POS2_X": -0.08,
        "INS_POS3_X": -0.08,
        "SCR_ENABLE": 1.0,
        "SERVO9_FUNCTION": 36.0,
    }

    assert {name: _config_target_params(use_all=False)[name] for name in expected} == expected


def test_arm_command_uses_normal_arm_and_bounded_disarm(monkeypatch):
    events = []

    class Session:
        def set_mode(self, mode):
            events.append(("mode", mode))

    monkeypatch.setattr(
        calibrate_repl,
        "_arm",
        lambda _session, *, force: events.append(("arm", force)) or True,
    )
    monkeypatch.setattr(
        calibrate_repl.time,
        "sleep",
        lambda duration: events.append(("sleep", duration)),
    )
    monkeypatch.setattr(
        calibrate_repl,
        "_disarm",
        lambda _session, force=False: events.append(("disarm", force)) or True,
    )

    calibrate_repl._cmd_arm(Session(), ["--duration", "0.25"])

    assert events == [
        ("mode", 1),
        ("arm", False),
        ("sleep", 0.25),
        ("disarm", False),
    ]


def test_arm_command_can_explicitly_bypass_prearm_checks(monkeypatch):
    events = []

    class Session:
        def set_mode(self, mode):
            events.append(("mode", mode))

    monkeypatch.setattr(
        calibrate_repl,
        "_arm",
        lambda _session, *, force: events.append(("arm", force)) or True,
    )
    monkeypatch.setattr(calibrate_repl.time, "sleep", lambda _duration: None)
    monkeypatch.setattr(
        calibrate_repl,
        "_disarm",
        lambda _session, force=False: events.append(("disarm", force)) or True,
    )

    calibrate_repl._cmd_arm(Session(), ["--duration", "0.25", "--force"])

    assert events == [
        ("mode", 1),
        ("arm", True),
        ("disarm", False),
    ]


def test_disarm_command_falls_back_to_force(monkeypatch):
    events = []
    monkeypatch.setattr(
        calibrate_repl,
        "_disarm",
        lambda _session, force=False: events.append(force) or force,
    )

    calibrate_repl._cmd_disarm(object())

    assert events == [False, True]


def test_verify_safe_off_keeps_reading_after_empty_filtered_batch(monkeypatch):
    class Session:
        _target_system = 1
        _target_component = 1

        def __init__(self):
            self.cursors = []

        def vehicle_status(self):
            return {"base_mode": 0, "custom_mode": 1}

        def get_param(self, name):
            return {
                "RAWES_MODE": 0.0,
                "H_FLYBAR_MODE": 1.0,
                "H_SV_MAN": 0.0,
                "SERVO9_FUNCTION": 36.0,
                "H_YAW_TRIM": 0.0,
                "SERVO9_MIN": 1000.0,
            }[name]

        def send_message(self, _message):
            pass

        def current_cursor(self):
            return "v1:10"

        def read_messages(self, after, _message_types, **_kwargs):
            self.cursors.append(after)
            if len(self.cursors) == 1:
                return MessageBatch((), "v1:11", _CLOCK)
            return MessageBatch(
                (ServoOutputRaw(servo9_raw=1000),),
                "v1:12",
                _CLOCK,
            )

    monkeypatch.setattr(calibrate_hw, "decode_message", lambda message: message)
    session = Session()

    report = verify_safe_off(session)

    assert report.ok
    assert report.motor_output_raw == 1000
    assert session.cursors == ["v1:10", "v1:11"]


def test_disarm_enters_acro_safe_off_after_confirmation(monkeypatch):
    events = []

    class Session:
        _target_system = 1
        _target_component = 1

        def __init__(self):
            self.params = {
                "RAWES_MODE": 3.0,
                "H_YAW_TRIM": 0.25,
                "H_FLYBAR_MODE": 0.0,
                "H_SV_MAN": 0.0,
                "SERVO9_FUNCTION": 36.0,
                "SERVO9_MIN": 1000.0,
            }

        def send_message(self, _message):
            pass

        def command(self, *_args, **_kwargs):
            events.append("disarm-command")
            return {"result": mavutil.mavlink.MAV_RESULT_ACCEPTED}

        def current_cursor(self):
            return "v1:0"

        def read_messages(self, _after, message_types, **_kwargs):
            if "SERVO_OUTPUT_RAW" in message_types:
                return MessageBatch((ServoOutputRaw(servo9_raw=0),), "v1:2", _CLOCK)
            return MessageBatch((Heartbeat(
                type=mavutil.mavlink.MAV_TYPE_HELICOPTER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode=0,
                custom_mode=4,
                system_status=mavutil.mavlink.MAV_STATE_STANDBY,
            ),), "v1:1", _CLOCK)

        def set_param(self, name, value):
            events.append(("set-param", name, value))
            self.params[name] = value
            return True

        def get_param(self, name):
            return self.params[name]

        def set_mode(self, mode):
            events.append(("set-mode", mode))

        def vehicle_status(self):
            return {"base_mode": 0, "custom_mode": 1}

    monkeypatch.setattr(calibrate_hw, "decode_message", lambda message: message)

    assert _disarm(Session()) is True
    assert events == [
        "disarm-command",
        ("set-param", "RAWES_MODE", 0),
        ("set-param", "H_YAW_TRIM", 0.0),
        ("set-param", "H_FLYBAR_MODE", 1.0),
        ("set-mode", 1),
    ]


def test_disarm_corrects_flybar_mode_before_selecting_acro(monkeypatch):
    events = []

    class Session:
        _target_system = 1
        _target_component = 1

        def __init__(self):
            self.params = {
                "RAWES_MODE": 3.0,
                "H_YAW_TRIM": 0.25,
                "H_FLYBAR_MODE": 0.0,
                "H_SV_MAN": 3.0,
                "SERVO9_FUNCTION": 36.0,
                "SERVO9_MIN": 1000.0,
            }

        def send_message(self, _message):
            pass

        def command(self, *_args, **_kwargs):
            return {"result": mavutil.mavlink.MAV_RESULT_ACCEPTED}

        def current_cursor(self):
            return "v1:0"

        def read_messages(self, _after, message_types, **_kwargs):
            if "SERVO_OUTPUT_RAW" in message_types:
                return MessageBatch((ServoOutputRaw(servo9_raw=0),), "v1:2", _CLOCK)
            return MessageBatch((Heartbeat(
                type=mavutil.mavlink.MAV_TYPE_HELICOPTER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode=0,
                custom_mode=4,
                system_status=mavutil.mavlink.MAV_STATE_STANDBY,
            ),), "v1:1", _CLOCK)

        def get_param(self, name):
            return self.params[name]

        def set_param(self, name, value):
            events.append(("set-param", name, value))
            self.params[name] = value
            return True

        def set_mode(self, mode):
            events.append(("set-mode", mode))

        def vehicle_status(self):
            return {"base_mode": 0, "custom_mode": 1}

    monkeypatch.setattr(calibrate_hw, "decode_message", lambda message: message)

    assert _disarm(Session(), force=True) is True
    assert events == [
        ("set-param", "RAWES_MODE", 0),
        ("set-param", "H_YAW_TRIM", 0.0),
        ("set-param", "H_FLYBAR_MODE", 1.0),
        ("set-param", "H_SV_MAN", 0.0),
        ("set-mode", 1),
    ]


def test_safety_shutdown_skips_disarm_when_already_disarmed(monkeypatch):
    events = []

    class Session:
        def set_param(self, name, value):
            events.append(("set-param", name, value))
            return True

    monkeypatch.setattr(
        calibrate_run,
        "_send_set_servo",
        lambda _session, output, pwm: events.append(("servo", output, pwm)),
    )
    monkeypatch.setattr(
        calibrate_run,
        "_wait_for_disarmed",
        lambda _session, timeout_s: events.append(("wait", timeout_s)) or True,
    )
    monkeypatch.setattr(
        calibrate_run,
        "_set_safe_off_state",
        lambda _session, *, rawes_mode_released: events.append(
            ("safe-off", rawes_mode_released)
        ),
    )
    monkeypatch.setattr(
        calibrate_run,
        "_disarm",
        lambda *_args, **_kwargs: pytest.fail("disarm should not be used"),
    )

    calibrate_run._safety_shutdown(Session())

    assert events == [
        ("set-param", "RAWES_MODE", 0),
        ("servo", 9, 1000),
        ("wait", 0.2),
        ("safe-off", True),
    ]


def test_safety_shutdown_force_disarms_when_normal_disarm_fails(monkeypatch):
    events = []

    class Session:
        def set_param(self, _name, _value):
            return True

        def send_message(self, _message):
            pass

    monkeypatch.setattr(calibrate_run, "_send_set_servo", lambda *_args: None)
    monkeypatch.setattr(
        calibrate_run,
        "_wait_for_disarmed",
        lambda _session, timeout_s: False,
    )
    monkeypatch.setattr(
        calibrate_run,
        "_disarm",
        lambda _session, timeout, force: (
            events.append((timeout, force)) or force
        ),
    )

    calibrate_run._safety_shutdown(Session())

    assert events == [(5.0, True)]


def test_hr3_manual_mixer_matches_physical_servo_positions():
    assert _h3_forward_mix(0.5, 0.0, 0.0) == pytest.approx((0.5, 0.5, 0.5))

    s1, s2, s3 = _h3_forward_mix(0.0, 0.5, 0.0)
    assert s1 == pytest.approx(s2)
    assert s3 == pytest.approx(-2.0 * s1)

    s1, s2, s3 = _h3_forward_mix(0.0, 0.0, 0.5)
    assert s1 == pytest.approx(-s2)
    assert s3 == pytest.approx(0.0)


def test_swash_info_uses_current_collective_params_and_hr3_diagram(capsys):
    class ServoOutput:
        servo1_raw = 1245
        servo2_raw = 1062
        servo3_raw = 1000
        servo9_raw = 1000

    class Session:
        _target_system = 1
        _target_component = 1

        def __init__(self):
            self.requested_params = []

        def get_param(self, name):
            self.requested_params.append(name)
            return {
                "H_SW_TYPE": 3,
                "H_SW_COL_DIR": 1,
                "SERVO1_REVERSED": 1,
                "SERVO2_REVERSED": 1,
                "SERVO3_REVERSED": 1,
                "H_SW_H3_SV1_POS": -60,
                "H_SW_H3_SV2_POS": 60,
                "H_SW_H3_SV3_POS": 180,
                "H_SW_H3_PHANG": 0,
                "AHRS_ORIENTATION": 0,
                "H_COL_MIN": 1000,
                "H_COL_MAX": 2000,
                "H_COL_ZERO_THRST": -2,
                "H_COL_HOVER": 0.5,
                "H_CYC_MAX": 1500,
                "H_FLYBAR_MODE": 1,
                "H_SV_MAN": 0,
            }.get(name)

        def send_message(self, _message):
            pass

        def current_cursor(self):
            return "v1:0"

        def read_messages(self, *_args, **_kwargs):
            return MessageBatch((ServoOutput(),), "v1:1", _CLOCK)

    session = Session()
    _print_swash_layout(session)
    output = capsys.readouterr().out

    assert "H_COL_MID" not in session.requested_params
    assert "H_COL_ZERO_THRST" in output
    assert "S3 - FRONT / ELEVATOR" in output
    assert "[1000 us]" in output
    assert "S2 - LEFT-REAR" in output
    assert "S1 - RIGHT-REAR" in output


def test_bounded_swash_sweep_never_exceeds_measured_envelope():
    waypoints = _bounded_swash_waypoints(1257, 1500, 1777)

    assert waypoints[0] == (1500, 1500, 1500)
    assert waypoints[-1] == (1500, 1500, 1500)
    assert all(1257 <= pwm <= 1777 for point in waypoints for pwm in point)


def test_fit_swash_range_converges_from_measured_extrema(monkeypatch):
    class Session:
        def __init__(self):
            self.params = {
                "SERVO1_TRIM": 1600.0,
                "SERVO2_TRIM": 1600.0,
                "SERVO3_TRIM": 1600.0,
                "H_COL_MIN": 1340.0,
                "H_COL_MAX": 1660.0,
                "H_CYC_MAX": 1000.0,
            }

        def set_param(self, name, value):
            self.params[name] = value
            return True

        def get_param(self, name):
            return self.params.get(name)

    measurements = iter(((1258, 1777), (1260, 1774)))
    monkeypatch.setattr(
        "calibrate.repl._run_servo_mode",
        lambda _session, _mode, _duration: next(measurements),
    )
    session = Session()

    _fit_swash_range(session, 1257, 1777, 1000, iterations=3, margin=3)

    assert session.params == {
        "SERVO1_TRIM": 1517.0,
        "SERVO2_TRIM": 1517.0,
        "SERVO3_TRIM": 1517.0,
        "H_COL_MIN": 1342.0,
        "H_COL_MAX": 1657.0,
        "H_CYC_MAX": 1000.0,
    }
