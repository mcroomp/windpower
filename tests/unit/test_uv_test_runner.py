import json
from pathlib import Path
from unittest.mock import Mock

import pytest

from simulation import run_tests


@pytest.mark.parametrize("returncode", [0, 1, 2])
def test_runner_uses_uv_and_preserves_pytest_exit_code(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, returncode: int,
):
    summary = tmp_path / "summary.json"
    monkeypatch.setattr(run_tests.sys, "argv", [
        "run_tests.py", "--filter", "all", "--summary", str(summary),
        "tests/unit/test_controller.py", "-q",
    ])
    proc = Mock(stdout=iter(["1 passed\n" if returncode == 0 else "ERROR: validation failed\n"]))
    proc.returncode = returncode
    popen = Mock(return_value=proc)
    monkeypatch.setattr(run_tests.subprocess, "Popen", popen)

    assert run_tests.main() == returncode

    assert popen.call_args.args[0] == [
        "uv", "run", "--no-sync", "--project", str(run_tests.REPO),
        "python", "-u", "-m", "pytest", "tests/unit/test_controller.py", "-q",
    ]
    assert json.loads(summary.read_text())["exit_code"] == returncode
    proc.wait.assert_called_once()


def test_runner_does_not_wrap_stack_shell_commands(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(run_tests.sys, "argv", ["run_tests.py", "bash", "test.sh", "stack"])
    popen = Mock()
    monkeypatch.setattr(run_tests.subprocess, "Popen", popen)

    assert run_tests.main() == 2
    popen.assert_not_called()
