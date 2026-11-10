"""Opt-in repository-wide static checks."""

from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
CHECKS = (
    pytest.param("Ruff", ["ruff", "check", str(ROOT)], id="ruff"),
    pytest.param("Pyright", [
        "pyright", "--project", str(ROOT / "pyrightconfig.json"),
        "--pythonpath", sys.executable, "--warnings",
    ], id="pyright"),
)


@pytest.mark.lint
@pytest.mark.parametrize(("tool", "args"), CHECKS)
def test_python_check(tool: str, args: list[str]) -> None:
    result = subprocess.run(
        [sys.executable, "-m", *args],
        cwd=ROOT,
        check=False,
    )
    assert result.returncode == 0, f"{tool} failed; fix the reported errors before merging."
