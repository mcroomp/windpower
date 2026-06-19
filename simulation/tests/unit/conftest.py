"""
conftest.py — pytest configuration for unit tests.

Fast, no-physics tests only. Simtests (full physics loops) live in tests/simtests/.
"""
import sys
from pathlib import Path

import pytest

_SIM_ROOT = Path(__file__).resolve().parents[2]
if str(_SIM_ROOT) not in sys.path:
    sys.path.insert(0, str(_SIM_ROOT))

from tests.common.path_setup import add_simulation_root


add_simulation_root()


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "simtest: full physics simulation loop — lives in tests/simtests/",
    )
