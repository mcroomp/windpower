"""
test_ap_lua_constants.py -- Verify MockArdupilot Python-mode constants match
rawes.lua module-level constants.

Any change to gains or limits must be made in both Python and Lua. A failure here
means the Python simtest and the Lua flight controller would use different values.
"""
import sys
from pathlib import Path

import pytest


from simulation.rawes_lua_harness import RawesLua
from tests.common.mock_ardupilot import MockArdupilot


@pytest.fixture(scope="module")
def lua():
    return RawesLua()


class TestTensionApLuaConstants:
    def test_kp_alt(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["KP_ALT"] == pytest.approx(lua.fns.KP_ALT)

    def test_ki_alt(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["KI_ALT"] == pytest.approx(lua.fns.KI_ALT)

    def test_kd_vz(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["KD_VZ"] == pytest.approx(lua.fns.KD_VZ)

    def test_kp_el(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["KP_EL"] == pytest.approx(lua.fns.KP_EL)
