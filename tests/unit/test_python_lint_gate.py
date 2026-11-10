from pathlib import Path
import subprocess
import sys
from typing import get_type_hints
from unittest.mock import Mock, call

import pytest

from linkhub_client.messages import MavModeFlag, decode_enum
from tests import conftest


@pytest.mark.parametrize("returncode", [1, 2])
@pytest.mark.parametrize("tool", ["Ruff", "Pyright"])
def test_lint_failure_stops_pytest(monkeypatch, returncode, tool):
    results = [subprocess.CompletedProcess([], returncode)]
    if tool == "Pyright":
        results.insert(0, subprocess.CompletedProcess([], 0))
    run = Mock(side_effect=results)
    monkeypatch.setattr(conftest.subprocess, "run", run)

    with pytest.raises(pytest.UsageError, match=f"{tool} failed"):
        conftest.pytest_sessionstart(Mock())
    assert run.call_count == len(results)


def test_lint_gate_uses_test_interpreter_and_repository_root(monkeypatch):
    run = Mock(return_value=subprocess.CompletedProcess([], 0))
    monkeypatch.setattr(conftest.subprocess, "run", run)

    conftest.pytest_sessionstart(Mock())

    root = Path(__file__).resolve().parents[2]
    assert run.call_args_list == [
        call([sys.executable, "-m", "ruff", "check", str(root)], cwd=root, check=False),
        call([
            sys.executable, "-m", "pyright", "--project", str(root / "pyrightconfig.json"),
            "--pythonpath", sys.executable, "--warnings",
        ], cwd=root, check=False),
    ]


def test_modernized_enum_decoder_preserves_runtime_annotations():
    hints = get_type_hints(decode_enum)

    assert hints["return"] is decode_enum.__type_params__[0]
    assert decode_enum(MavModeFlag, {"type": "MAV_MODE_FLAG_SAFETY_ARMED"}) is MavModeFlag.SAFETY_ARMED
