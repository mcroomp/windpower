import logging
import subprocess
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from calibrate import linkhub


@pytest.mark.parametrize("platform", ["nt", "posix"])
def test_owned_linkhub_uses_a_windows_process_group(monkeypatch, platform):
    popen = Mock()
    monkeypatch.setattr(linkhub, "os", SimpleNamespace(name=platform))
    monkeypatch.setattr(linkhub, "_linkhub_binary", lambda: Path("linkhub.exe"))
    monkeypatch.setattr(linkhub.subprocess, "Popen", popen)
    monkeypatch.setattr(linkhub.subprocess, "CREATE_NEW_PROCESS_GROUP", 0x200, raising=False)

    linkhub._start_linkhub("udp:192.168.9.143:14550", None, motor_name_prefix=None)

    args, kwargs = popen.call_args
    assert args[0][-2:] == ["--connection", "udp:192.168.9.143:14550"]
    assert kwargs["creationflags"] == (0x200 if platform == "nt" else 0)
    assert kwargs["cwd"] == linkhub._REPO_ROOT


@pytest.mark.parametrize("platform", ["nt", "posix"])
def test_owned_linkhub_stops_gracefully(monkeypatch, platform):
    process = Mock()
    process.poll.return_value = None
    process.wait.return_value = 0
    monkeypatch.setattr(linkhub, "os", SimpleNamespace(name=platform))
    monkeypatch.setattr(linkhub, "signal", SimpleNamespace(CTRL_BREAK_EVENT=1))

    linkhub._stop_process(process)

    if platform == "nt":
        process.send_signal.assert_called_once_with(1)
        process.terminate.assert_not_called()
    else:
        process.terminate.assert_called_once_with()
        process.send_signal.assert_not_called()
    process.wait.assert_called_once_with(timeout=10)
    process.kill.assert_not_called()


def test_owned_linkhub_reports_failed_cleanup():
    process = Mock()
    process.poll.return_value = None
    process.wait.return_value = 1
    with pytest.raises(RuntimeError, match="shutdown failed with exit code 1"):
        linkhub._stop_process(process)


def test_owned_linkhub_reports_forced_shutdown(caplog):
    process = Mock()
    process.poll.return_value = None
    process.wait.side_effect = [subprocess.TimeoutExpired("linkhub", 10), 1]
    with caplog.at_level(logging.ERROR, logger=linkhub.__name__):
        with pytest.raises(RuntimeError, match="UDP cleanup may be incomplete"):
            linkhub._stop_process(process)
    process.kill.assert_called_once_with()
    assert "forcing exit" in caplog.text


@pytest.mark.parametrize("returncode", [0, 1])
def test_owned_linkhub_handles_already_exited_process(returncode):
    process = Mock()
    process.poll.return_value = returncode
    if returncode:
        with pytest.raises(RuntimeError, match="LinkHub exited with 1"):
            linkhub._stop_process(process)
    else:
        linkhub._stop_process(process)
    process.send_signal.assert_not_called()
    process.terminate.assert_not_called()
