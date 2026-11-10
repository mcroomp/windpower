"""Shared validation for every pytest tier."""

from pathlib import Path
import subprocess
import sys

import pytest


def pytest_sessionstart(session: pytest.Session) -> None:
    root = Path(__file__).resolve().parents[1]
    checks = (
        ("Ruff", ["ruff", "check", str(root)]),
        ("Pyright", [
            "pyright", "--project", str(root / "pyrightconfig.json"),
            "--pythonpath", sys.executable, "--warnings",
        ]),
    )
    for name, args in checks:
        result = subprocess.run(
            [sys.executable, "-m", *args],
            cwd=root,
            check=False,
        )
        if result.returncode:
            raise pytest.UsageError(f"{name} failed; fix the reported errors before running tests.")
