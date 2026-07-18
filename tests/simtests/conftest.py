"""
conftest.py — pytest configuration for simtests.

Simtests are full time-domain physics simulations (seconds to minutes of compute).
Run with:  pytest tests/simtests -m simtest
Skip slow: pytest tests/simtests -m "not simtest"
"""
import sys
from pathlib import Path

import pytest


SIMTEST_TIMEOUT_S = 600


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "simtest: full physics simulation loop — slow, excluded from default unit test run",
    )


def pytest_collection_modifyitems(items):
    for item in items:
        if item.get_closest_marker("simtest") and not item.get_closest_marker("timeout"):
            item.add_marker(pytest.mark.timeout(SIMTEST_TIMEOUT_S))


@pytest.fixture
def simtest_log(request):
    """Per-test SimtestLog: logs land in simulation/logs/<test_name>/."""
    from simulation.simtest_log import SimtestLog
    log = SimtestLog(request.node.name)
    log.dump_params_json()
    return log
