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
