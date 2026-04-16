"""
conftest.py — top-level pytest hooks for RAWES SITL integration tests.

All shared infrastructure symbols are re-exported from stack_infra via the
wildcard import so any test file that imports conftest at this level gets
the full stack_infra namespace.

Fixture definitions live in:
  sitl/flight/conftest.py  — acro_armed, acro_armed_pumping_lua,
                             acro_armed_landing_lua, acro_armed_lua_full
  sitl/torque/conftest.py  — torque_armed, torque_armed_profile, torque_armed_lua
"""
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
