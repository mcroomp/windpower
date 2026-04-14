"""
conftest.py — top-level pytest hooks for RAWES SITL integration tests.

Provides pytest_addoption and pytest_configure so the --sitl-speedup option
is available to all SITL tests (flight/ and torque/).

All shared infrastructure symbols are re-exported from stack_infra via the
wildcard import so any test file that imports conftest at this level gets
the full stack_infra namespace.

Fixture definitions live in:
  sitl/flight/conftest.py  — acro_armed, acro_armed_pumping_lua,
                             acro_armed_landing_lua, acro_armed_lua_full
  sitl/torque/conftest.py  — torque_armed, torque_armed_profile, torque_armed_lua
"""
import math
import os
import time

import pytest

from stack_infra import *  # noqa: F401,F403  — re-export everything for test imports
from stack_infra import (
    _sitl_stack,
    _acro_stack,
    _torque_stack,
    _LUA_TORQUE_EXTRA_PARAMS,
    _BASE_ACRO_PARAMS,
    _RAWES_DEFAULTS_PARM,
    _install_lua_scripts,
    _run_acro_setup,
    _STARTUP_DAMP_S,
)


def pytest_addoption(parser):
    parser.addoption(
        "--sitl-speedup",
        type=int,
        default=1,
        help="Run ArduPilot SITL at Nx real-time speed (--speedup N passed to sim_vehicle.py). "
             "Also sets RAWES_SPEEDUP=N so GCS timeouts and sim_sleep() scale correctly. "
             "Example: --sitl-speedup 2  runs the suite ~2x faster.",
    )


def pytest_configure(config):
    # Sync --sitl-speedup option -> RAWES_SPEEDUP env var so sim_time.wall_s()
    # and sim_sleep() in gcs.py (same process) scale timeouts correctly.
    try:
        sp = config.getoption("--sitl-speedup")
        if sp and sp > 1:
            os.environ["RAWES_SPEEDUP"] = str(float(sp))
    except ValueError:
        pass   # option not registered yet (e.g. during collection without our conftest)
