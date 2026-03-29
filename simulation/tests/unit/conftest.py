"""
conftest.py — pytest configuration for unit tests.

Markers
-------
simtest : full time-domain simulation loop (seconds of compute time).
          Run with:  pytest -m simtest
          Skip with: pytest -m "not simtest"  (default via test_unit.sh)
"""
import pytest


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "simtest: full physics simulation loop — slow, excluded from default test-unit run",
    )
