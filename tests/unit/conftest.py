"""
conftest.py — pytest configuration for unit tests.

Fast, no-physics tests only. Simtests (full physics loops) live in tests/simtests/.
"""
import pytest


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "simtest: full physics simulation loop — lives in tests/simtests/",
    )
    config.addinivalue_line(
        "markers",
        "expensive: loop-heavy unit test; run explicitly with -m expensive",
    )


def pytest_collection_modifyitems(items):
    for item in items:
        if item.get_closest_marker("timeout"):
            continue
        timeout_s = 600 if item.get_closest_marker("expensive") else 10
        item.add_marker(pytest.mark.timeout(timeout_s))
