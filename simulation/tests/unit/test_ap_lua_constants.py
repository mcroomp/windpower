"""
test_ap_lua_constants.py -- Verify TensionApController / LandingApController class
constants match rawes.lua module-level constants.

Any change to gains or limits must be made in both Python and Lua. A failure here
means the Python simtest and the Lua flight controller would use different values.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ap_controller import TensionApController, LandingApController
from rawes_lua_harness import RawesLua


@pytest.fixture(scope="module")
def lua():
    return RawesLua()


class TestTensionApLuaConstants:

    def test_kp_ten(self, lua):
        assert TensionApController.KP_TEN == pytest.approx(lua.fns.KP_TEN)

    def test_ki_ten(self, lua):
        assert TensionApController.KI_TEN == pytest.approx(lua.fns.KI_TEN)

    def test_kd_ten(self, lua):
        assert TensionApController.KD_TEN == pytest.approx(lua.fns.KD_TEN)

    def test_col_min_rad(self, lua):
        assert TensionApController.COL_MIN_RAD == pytest.approx(lua.fns.COL_MIN_RAD)

    def test_col_max_ten(self, lua):
        assert TensionApController.COL_MAX_TEN == pytest.approx(lua.fns.COL_MAX_TEN)

    def test_k_vib(self, lua):
        assert TensionApController.K_VIB == pytest.approx(lua.fns.K_VIB)

    def test_vib_hp_tau(self, lua):
        """Python stores HP cutoff as a frequency; Lua stores the time constant 1/(2π×Hz)."""
        python_tau = 1.0 / (2.0 * math.pi * TensionApController.VIB_HP_HZ)
        assert python_tau == pytest.approx(float(lua.fns.VIB_HP_TAU), rel=1e-6)

    def test_vib_vel_tau(self, lua):
        assert TensionApController.VIB_VEL_TAU == pytest.approx(lua.fns.VIB_VEL_TAU)

    def test_vib_col_max(self, lua):
        assert TensionApController.VIB_COL_MAX == pytest.approx(lua.fns.VIB_COL_MAX)


class TestLandingApLuaConstants:

    def test_col_min_rad(self, lua):
        assert LandingApController.COL_MIN_RAD == pytest.approx(lua.fns.COL_MIN_RAD)

    def test_col_max_rad(self, lua):
        assert LandingApController.COL_MAX_RAD == pytest.approx(lua.fns.COL_MAX_RAD)

    def test_kp_vz(self, lua):
        assert LandingApController.KP_VZ == pytest.approx(lua.fns.KP_VZ)

    def test_ki_vz(self, lua):
        assert LandingApController.KI_VZ == pytest.approx(lua.fns.KI_VZ)
