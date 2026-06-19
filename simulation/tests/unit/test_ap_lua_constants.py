"""
test_ap_lua_constants.py -- Verify MockArdupilot Python-mode constants match
rawes.lua module-level constants.

Any change to gains or limits must be made in both Python and Lua. A failure here
means the Python simtest and the Lua flight controller would use different values.
"""
import math
import sys
from pathlib import Path

import pytest


from rawes_lua_harness import RawesLua
from tests.common.mock_ardupilot import MockArdupilot


@pytest.fixture(scope="module")
def lua():
    return RawesLua()


class TestTensionApLuaConstants:
    def test_col_min_rad(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["COL_MIN_RAD"] == pytest.approx(lua.fns.COL_MIN_RAD)

    def test_col_max_rad(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["COL_MAX_RAD"] == pytest.approx(lua.fns.COL_MAX_RAD)

    def test_kp_alt(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["KP_ALT"] == pytest.approx(lua.fns.KP_ALT)

    def test_ki_alt(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["KI_ALT"] == pytest.approx(lua.fns.KI_ALT)

    def test_kd_vz(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["KD_VZ"] == pytest.approx(lua.fns.KD_VZ)

    def test_rate_accel_max(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["RATE_ACCEL_MAX_RADSS"] == pytest.approx(lua.fns.RATE_ACCEL_MAX_RADSS)

    def test_k_vib(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["K_VIB"] == pytest.approx(lua.fns.K_VIB)

    def test_vib_hp_tau(self, lua):
        """Python stores HP cutoff as a frequency; Lua stores the time constant 1/(2π×Hz)."""
        python_tau = 1.0 / (2.0 * math.pi * MockArdupilot.PUMPING_CONSTANTS["VIB_HP_HZ"])
        assert python_tau == pytest.approx(float(lua.fns.VIB_HP_TAU), rel=1e-6)

    def test_vib_vel_tau(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["VIB_VEL_TAU"] == pytest.approx(lua.fns.VIB_VEL_TAU)

    def test_vib_col_max(self, lua):
        assert MockArdupilot.PUMPING_CONSTANTS["VIB_COL_MAX"] == pytest.approx(lua.fns.VIB_COL_MAX)
