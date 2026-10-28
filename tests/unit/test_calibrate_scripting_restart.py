from unittest.mock import Mock

import pytest

import calibrate.hw as hw
from linkhub_client.client import LinkHubClient, LinkHubError, LinkHubGenerationChanged
from linkhub_client.messages import MavCmd, MavResult, MavSeverity, Statustext


_RESTARTED = "Scripting: restarted"
_LOADED = "RAWES: loaded  mode=0 (none)"
_READY = "RAWES: mode 0 (none)  disarmed"


@pytest.fixture
def restart(monkeypatch):
    session = Mock(spec=LinkHubClient)
    session.is_armed = False
    session.get_param.side_effect = lambda name: {"SCR_ENABLE": 1, "RAWES_MODE": 0}[name]
    session.current_cursor.return_value = "v1:100"
    session.generation = "v1:test:1"
    session.command.return_value = {"result": MavResult.ACCEPTED}
    pending = [_RESTARTED, _LOADED, _READY]
    now = 0.0
    sequence = 100

    def read(_session, _cursor, _types, *, wait, expected_generation):
        nonlocal now, sequence
        now += wait
        sequence += 1
        message = (
            Statustext(severity=MavSeverity.INFO, text=pending.pop(0))
            if pending else None
        )
        return message, f"v1:{sequence}"

    reader = Mock(side_effect=read)
    monkeypatch.setattr(hw, "read_one", reader)
    monkeypatch.setattr(hw.time, "monotonic", lambda: now)
    return session, pending, reader


def test_restart_sends_explicit_command_and_verifies_fresh_firmware_events(restart, capsys):
    session, _, reader = restart

    hw._restart_scripting(session)

    session.command.assert_called_once_with(MavCmd.SCRIPTING, [3.0])
    session.set_param.assert_not_called()
    assert [entry.args[1] for entry in reader.call_args_list] == [
        "v1:100", "v1:101", "v1:102"
    ]
    assert all(
        entry.kwargs["expected_generation"] == "v1:test:1"
        for entry in reader.call_args_list
    )
    assert "Scripting engine restarted." in capsys.readouterr().out


@pytest.mark.parametrize("result", [MavResult.DENIED, MavResult.FAILED, MavResult.UNSUPPORTED])
def test_restart_rejection_is_not_success(restart, result, capsys):
    session, _, reader = restart
    session.command.return_value = {"result": result}

    with pytest.raises(LinkHubError, match=str(result)):
        hw._restart_scripting(session)

    reader.assert_not_called()
    assert "Scripting engine restarted." not in capsys.readouterr().out


@pytest.mark.parametrize("missing", [_RESTARTED, _LOADED, _READY])
def test_acknowledgement_without_all_reload_events_times_out(restart, missing, capsys):
    session, pending, reader = restart
    pending.remove(missing)

    with pytest.raises(LinkHubError, match="not verified"):
        hw._restart_scripting(session)

    assert "Scripting engine restarted." not in capsys.readouterr().out
    assert sum(entry.kwargs["wait"] for entry in reader.call_args_list) == 10.0


def test_load_and_mode_messages_before_restart_do_not_verify_reload(restart):
    session, pending, _ = restart
    pending[:] = [_LOADED, _READY, _RESTARTED]

    with pytest.raises(LinkHubError, match="not verified"):
        hw._restart_scripting(session)


def test_restart_recovers_an_already_stopped_engine(restart):
    session, pending, _ = restart
    pending.insert(0, "Scripting: stopped")

    hw._restart_scripting(session)

    session.set_param.assert_not_called()


@pytest.mark.parametrize("enabled", [0, None])
def test_disabled_or_unreadable_scripting_requires_reboot(restart, enabled):
    session, _, reader = restart
    session.get_param.side_effect = lambda name: enabled if name == "SCR_ENABLE" else 0

    with pytest.raises(LinkHubError, match="SCR_ENABLE=1"):
        hw._restart_scripting(session)

    session.command.assert_not_called()
    reader.assert_not_called()


def test_restart_refuses_armed_vehicle(restart):
    session, _, _ = restart
    session.is_armed = True

    with pytest.raises(LinkHubError, match="disarmed"):
        hw._restart_scripting(session)

    session.command.assert_not_called()


def test_restart_requires_safe_off_mode(restart):
    session, _, _ = restart
    session.get_param.side_effect = lambda name: 1

    with pytest.raises(LinkHubError, match="RAWES_MODE=0"):
        hw._restart_scripting(session)

    session.command.assert_not_called()


def test_generation_change_aborts_verification(restart):
    session, _, reader = restart
    reader.side_effect = LinkHubGenerationChanged("v1:test:1", "v1:test:2")

    with pytest.raises(LinkHubGenerationChanged):
        hw._restart_scripting(session)


def test_command_transport_failure_does_not_poll(restart):
    session, _, reader = restart
    session.command.side_effect = LinkHubError("link lost")

    with pytest.raises(LinkHubError, match="link lost"):
        hw._restart_scripting(session)

    reader.assert_not_called()
