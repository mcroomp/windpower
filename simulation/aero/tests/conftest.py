"""
conftest.py — pytest fixtures for aero/ test suite.

Shared constants and utilities live in _helpers.py (explicitly importable).
This file only defines pytest fixtures (auto-discovered by pytest).
"""

import sys
from pathlib import Path
import pytest

_HERE      = Path(__file__).resolve().parent
_AERO_DIR  = _HERE.parent
_SIM_DIR   = _AERO_DIR.parent

for _p in [str(_SIM_DIR), str(_AERO_DIR), str(_HERE)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aero import rotor_definition as rd


@pytest.fixture(scope="session")
def beaupoil_rotor():
    return rd.default()


@pytest.fixture(scope="session")
def deschutter_rotor():
    return rd.load("de_schutter_2018")


@pytest.fixture(scope="session")
def pca2_rotor():
    return rd.load("pca2_1934")
