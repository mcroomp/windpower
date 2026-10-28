from contextlib import nullcontext
from pathlib import Path
from unittest.mock import Mock, call

import pytest

import calibrate.params as params
import calibrate.repl as repl
from linkhub_client.client import LinkHubClient, LinkHubError


@pytest.fixture
def upload(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    source = tmp_path / "rawes.lua"
    source.write_bytes(b"return function() end")
    session = Mock(spec=LinkHubClient)
    session.list_files.return_value = [
        {"name": "scripts", "is_dir": True, "size": 0}
    ]
    session.upload_file.return_value = source.stat().st_size
    session.restart = Mock()
    monkeypatch.setattr(params, "_restart_scripting", session.restart)
    return session, source


def test_existing_scripts_directory_skips_mkdir(upload):
    session, source = upload

    params._upload_script(session, str(source))

    session.list_files.assert_called_once_with("/APM")
    session.create_directory.assert_not_called()
    session.upload_file.assert_called_once_with(str(source), "/APM/scripts/rawes.lua")
    session.restart.assert_called_once_with(session)


def test_absent_scripts_directory_is_created_before_upload(upload):
    session, source = upload
    session.list_files.return_value = []

    params._upload_script(session, str(source), restart=False)

    assert session.method_calls == [
        call.list_files("/APM"),
        call.create_directory("/APM/scripts"),
        call.upload_file(str(source), "/APM/scripts/rawes.lua"),
    ]


def test_scripts_path_must_be_a_directory(upload):
    session, source = upload
    session.list_files.return_value = [
        {"name": "scripts", "is_dir": False, "size": 42}
    ]

    with pytest.raises(NotADirectoryError, match="/APM/scripts"):
        params._upload_script(session, str(source))

    session.create_directory.assert_not_called()
    session.upload_file.assert_not_called()
    session.set_param.assert_not_called()


@pytest.mark.parametrize("operation", ["list_files", "create_directory", "upload_file"])
def test_upload_failures_propagate_without_restart(upload, operation):
    session, source = upload
    session.list_files.return_value = []
    getattr(session, operation).side_effect = LinkHubError(f"{operation} failed")

    with pytest.raises(LinkHubError, match=f"{operation} failed"):
        params._upload_script(session, str(source))

    session.set_param.assert_not_called()
    if operation == "list_files":
        session.create_directory.assert_not_called()
    if operation != "upload_file":
        session.upload_file.assert_not_called()


def test_missing_local_script_fails_before_remote_operations(upload):
    session, source = upload
    source.unlink()

    with pytest.raises(FileNotFoundError):
        params._upload_script(session, str(source))

    assert session.method_calls == []


def test_restart_verification_failure_propagates(upload, capsys):
    session, source = upload
    session.restart.side_effect = LinkHubError("Restart not verified")

    with pytest.raises(LinkHubError, match="Restart not verified"):
        params._upload_script(session, str(source))

    assert "Scripting engine restarted." not in capsys.readouterr().out


@pytest.mark.parametrize("failure", [LinkHubError("FTP failed"), OSError("read failed")])
def test_one_shot_upload_errors_exit_nonzero(upload, monkeypatch, capsys, failure):
    session, source = upload
    session.upload_file.side_effect = failure
    monkeypatch.setattr(
        repl, "ensure_linkhub", lambda *_args, **_kwargs: nullcontext()
    )
    monkeypatch.setattr(repl, "_connect", lambda _server: session)
    monkeypatch.setattr(
        repl.sys, "argv", ["calibrate", "script", "upload", str(source)]
    )

    with pytest.raises(SystemExit) as exc:
        repl.main()

    assert exc.value.code == 1
    assert "[FAIL]" in capsys.readouterr().out
    session.close.assert_called_once()


def test_repl_reports_upload_failure_and_accepts_next_command(upload, monkeypatch, capsys):
    session, source = upload
    session.upload_file.side_effect = LinkHubError("FTP failed")
    commands = iter([f"script upload {source}", "script list", "quit"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(commands))

    repl._repl(session)

    assert "[FAIL] FTP failed" in capsys.readouterr().out
    assert session.list_files.call_args_list == [
        call("/APM"), call("/APM/scripts")
    ]


@pytest.mark.parametrize("args", [[], ["upload"], ["list", "extra"], ["remove"], ["other"]])
def test_invalid_script_commands_raise_usage_error(args):
    with pytest.raises(ValueError, match="script"):
        repl._cmd_script(Mock(spec=LinkHubClient), args)


@pytest.mark.parametrize("operation,args", [
    ("list_files", ["list"]),
    ("remove_file", ["remove", "rawes.lua"]),
])
def test_other_script_operation_errors_propagate(operation, args):
    session = Mock(spec=LinkHubClient)
    getattr(session, operation).side_effect = LinkHubError("FTP failed")

    with pytest.raises(LinkHubError, match="FTP failed"):
        repl._cmd_script(session, args)
