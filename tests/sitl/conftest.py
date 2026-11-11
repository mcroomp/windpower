"""
conftest.py — top-level pytest hooks for RAWES SITL integration tests.

All shared infrastructure symbols are re-exported from stack_infra via the
wildcard import so any test file that imports conftest at this level gets
the full stack_infra namespace.

Fixture definitions live in:
  sitl/flight/conftest.py  — guided_nogps_armed, guided_nogps_armed_pumping_lua,
                             guided_nogps_armed_landing_lua, guided_nogps_armed_lua_full
  sitl/torque/conftest.py  — torque_armed, torque_armed_profile, torque_armed_lua
"""
from tests.sitl.stack_infra import *  # noqa: F401,F403  — re-export everything for test imports
