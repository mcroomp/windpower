from pathlib import Path
import subprocess
import sys
from typing import get_type_hints
from unittest.mock import Mock

import pytest

from linkhub_client.messages import MavModeFlag, decode_enum
from tests import conftest


@pytest.mark.parametrize("returncode", [1, 2])
def test_lint_failure_stops_pytest(monkeypatch, returncode):
    run = Mock(return_value=subprocess.CompletedProcess([], returncode))
    monkeypatch.setattr(conftest.subprocess, "run", run)

    with pytest.raises(pytest.UsageError, match="Ruff failed"):
        conftest.pytest_sessionstart(Mock())


def test_lint_gate_uses_test_interpreter_and_repository_root(monkeypatch):
    run = Mock(return_value=subprocess.CompletedProcess([], 0))
    monkeypatch.setattr(conftest.subprocess, "run", run)

    conftest.pytest_sessionstart(Mock())

    root = Path(__file__).resolve().parents[2]
    run.assert_called_once_with(
        [sys.executable, "-m", "ruff", "check", str(root)],
        cwd=root,
        check=False,
    )


def test_modernized_enum_decoder_preserves_runtime_annotations():
    hints = get_type_hints(decode_enum)

    assert hints["return"] is decode_enum.__type_params__[0]
    assert decode_enum(MavModeFlag, {"type": "MAV_MODE_FLAG_SAFETY_ARMED"}) is MavModeFlag.SAFETY_ARMED
