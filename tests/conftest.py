"""Shared validation for every pytest tier."""

from pathlib import Path
import subprocess
import sys

import pytest


def pytest_sessionstart(session: pytest.Session) -> None:
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [sys.executable, "-m", "ruff", "check", str(root)],
        cwd=root,
        check=False,
    )
    if result.returncode:
        raise pytest.UsageError("Ruff failed; fix the reported lint errors before running tests.")
