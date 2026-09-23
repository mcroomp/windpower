import pytest

from pymavlink import mavutil

import calibrate.hw as calibrate_hw
import calibrate.run as calibrate_run
from calibrate.hw import _disarm, _h3_forward_mix
from groundstation.gcs import Heartbeat, NamedValueFloat
from calibrate.params import _CONFIG_TARGET_PARAMS_COMMON
from calibrate.repl import (
    _bounded_swash_waypoints,
    _fit_swash_range,
    _print_swash_layout,
)


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
        "H_CYC_MAX": 1000.0,
        "H_FLYBAR_MODE": 1.0,
        "IM_ACRO_COL_EXP": 0.0,
        "INS_POS1_X": -0.08,
        "INS_POS2_X": -0.08,
        "INS_POS3_X": -0.08,
    }

    assert {name: _CONFIG_TARGET_PARAMS_COMMON[name] for name in expected} == expected


def test_disarm_enters_acro_safe_off_after_confirmation(monkeypatch):
    events = []

    class Session:
        _target_system = 1
        _target_component = 1

        def send_message(self, _message):
            events.append("disarm-command")

        def _recv(self, **_kwargs):
            return Heartbeat(
                type=mavutil.mavlink.MAV_TYPE_HELICOPTER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode=0,
                custom_mode=4,
                system_status=mavutil.mavlink.MAV_STATE_STANDBY,
            )

        def set_param(self, name, value):
            events.append(("set-param", name, value))
            return True

        def get_param(self, name):
            return {"H_FLYBAR_MODE": 1.0, "H_SV_MAN": 0.0}[name]

        def set_mode(self, mode):
            events.append(("set-mode", mode))

    monkeypatch.setattr(calibrate_hw, "decode_message", lambda message: message)

    assert _disarm(Session()) is True
    assert events == [
        "disarm-command",
        ("set-param", "RAWES_MODE", 0),
        ("set-param", "H_FLYBAR_MODE", 0.0),
        ("set-param", "H_SV_MAN", 3.0),
        ("set-mode", 1),
    ]


def test_disarm_corrects_flybar_mode_before_selecting_acro(monkeypatch):
    events = []

    class Session:
        _target_system = 1
        _target_component = 1

        def send_message(self, _message):
            pass

        def _recv(self, **_kwargs):
            return Heartbeat(
                type=mavutil.mavlink.MAV_TYPE_HELICOPTER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode=0,
                custom_mode=4,
                system_status=mavutil.mavlink.MAV_STATE_STANDBY,
            )

        def get_param(self, _name):
            return 0.0

        def set_param(self, name, value):
            events.append(("set-param", name, value))
            return True

        def set_mode(self, mode):
            events.append(("set-mode", mode))

    monkeypatch.setattr(calibrate_hw, "decode_message", lambda message: message)

    assert _disarm(Session(), force=True) is True
    assert events == [
        ("set-param", "RAWES_MODE", 0),
        ("set-param", "H_SV_MAN", 3.0),
        ("set-mode", 1),
    ]


def test_safety_shutdown_uses_lua_disarm_before_safe_off(monkeypatch):
    events = []

    class Session:
        def set_param(self, name, value):
            events.append(("set-param", name, value))
            return True

        def send_message(self, message):
            assert isinstance(message, NamedValueFloat)
            events.append(("send", message.name, message.value))

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
        lambda *_args, **_kwargs: pytest.fail("force disarm should not be used"),
    )

    calibrate_run._safety_shutdown(Session())

    assert events == [
        ("set-param", "RAWES_MODE", 0),
        ("servo", 9, 1000),
        ("send", "RAWES_ARM", 1.0),
        ("wait", 1.0),
        ("safe-off", True),
    ]


def test_safety_shutdown_force_disarms_when_lua_does_not_confirm(monkeypatch):
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
        lambda _session, timeout, force: events.append((timeout, force)) or True,
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

        def _recv(self, **_kwargs):
            return ServoOutput()

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
