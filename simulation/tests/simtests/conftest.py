"""
conftest.py — pytest configuration for simtests.

Simtests are full time-domain physics simulations (seconds to minutes of compute).
Run with:  pytest simulation/tests/simtests -m simtest
Skip slow: pytest simulation/tests/simtests -m "not simtest"
"""
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
