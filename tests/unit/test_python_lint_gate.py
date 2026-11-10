from pathlib import Path
import subprocess
import sys
from typing import get_type_hints
from unittest.mock import Mock

import pytest

from linkhub_client.messages import MavModeFlag, decode_enum
from tests.lint import test_python_checks as lint_checks


@pytest.mark.parametrize("returncode", [1, 2])
@pytest.mark.parametrize("tool", ["Ruff", "Pyright"])
def test_lint_failure_fails_the_check(monkeypatch, returncode, tool):
    run = Mock(return_value=subprocess.CompletedProcess([], returncode))
    monkeypatch.setattr(lint_checks.subprocess, "run", run)

    with pytest.raises(AssertionError, match=f"{tool} failed"):
        lint_checks.test_python_check(tool, [tool.lower()])


@pytest.mark.parametrize(("tool", "args"), lint_checks.CHECKS)
def test_lint_check_uses_test_interpreter_and_repository_root(monkeypatch, tool, args):
    run = Mock(return_value=subprocess.CompletedProcess([], 0))
    monkeypatch.setattr(lint_checks.subprocess, "run", run)

    lint_checks.test_python_check(tool, args)

    root = Path(__file__).resolve().parents[2]
    expected_args = (
        ["ruff", "check", str(root)] if tool == "Ruff" else
        ["pyright", "--project", str(root / "pyrightconfig.json"),
         "--pythonpath", sys.executable, "--warnings"]
    )
    run.assert_called_once_with([sys.executable, "-m", *expected_args], cwd=root, check=False)


@pytest.mark.parametrize(("options", "expected_code", "summary"), [
    ([], 0, "1 passed, 1 skipped"),
    (["--run-lint"], 1, "1 failed, 1 passed"),
    (["--run-lint", "-m", "lint"], 1, "1 failed, 1 deselected"),
    (["--run-lint", "--collect-only"], 0, "2 tests collected"),
])
def test_lint_category_is_explicitly_opt_in(tmp_path, options, expected_code, summary):
    root = Path(__file__).resolve().parents[2]
    (tmp_path / "conftest.py").write_text((root / "tests" / "conftest.py").read_text())
    (tmp_path / "pytest.ini").write_text("[pytest]\nmarkers =\n    lint: opt-in static checks\n")
    (tmp_path / "test_selection.py").write_text(
        "import pytest\n"
        "def test_normal():\n"
        "    pass\n"
        "@pytest.mark.lint\n"
        "def test_lint():\n"
        "    assert False, 'lint was executed'\n"
    )

    result = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", *options],
        cwd=tmp_path, capture_output=True, text=True, check=False,
    )

    assert result.returncode == expected_code, result.stdout + result.stderr
    assert summary in result.stdout, result.stdout + result.stderr


def test_modernized_enum_decoder_preserves_runtime_annotations():
    hints = get_type_hints(decode_enum)

    assert hints["return"] is decode_enum.__type_params__[0]
    assert decode_enum(MavModeFlag, {"type": "MAV_MODE_FLAG_SAFETY_ARMED"}) is MavModeFlag.SAFETY_ARMED
