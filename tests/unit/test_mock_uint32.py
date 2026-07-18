"""Regression tests for the mock uint32_t type used by millis() in the Lua mock.

On real ArduPilot, millis()/micros() return a uint32_t *userdata*.  Every
arithmetic/comparison op coerces the other operand to uint32_t, and that
coercion FAILS ("Unable to coerce to uint32_t") when the operand has a
fractional part.  This is exactly what makes `millis() * 0.001` crash on
hardware while silently "working" against a plain-number mock.

These tests pin the mock to the same behaviour so the bug class is caught at
unit-test time instead of only in SITL.
"""
from __future__ import annotations

import pytest
from lupa import lua54

from simulation.rawes_lua_harness import RawesLua


class TestMockUint32:
    def test_millis_returns_uint32_with_tofloat_and_toint(self):
        sim = RawesLua()
        sim._mock.millis_val = 12345
        f = sim._lua.eval(
            "function() local v = millis(); return v:tofloat(), v:toint() end"
        )
        tof, toi = f()
        assert tof == 12345.0
        assert toi == 12345

    def test_fractional_multiply_raises(self):
        """`millis() * 0.001` must raise -- the exact bug that hit rawes.lua."""
        sim = RawesLua()
        sim._mock.millis_val = 5000
        mul = sim._lua.eval("function() return millis() * 0.001 end")
        with pytest.raises(lua54.LuaError) as exc:
            mul()
        assert "coerce to uint32" in str(exc.value).lower()

    def test_integer_arithmetic_and_comparison_work(self):
        sim = RawesLua()
        sim._mock.millis_val = 7000
        f = sim._lua.eval(
            "function() "
            "local a = millis(); "
            "local b = a - 2000; "
            "return (a - b):tofloat(), (a >= 5000), (b < a) "
            "end"
        )
        diff, ge, lt = f()
        assert diff == 2000.0
        assert ge is True
        assert lt is True

    def test_negative_operand_wraps_like_hardware(self):
        """`now - (-2000)` must not raise (used by the guided-cmd log throttle)."""
        sim = RawesLua()
        sim._mock.millis_val = 0
        f = sim._lua.eval("function() return (millis() - (-2000)):tofloat() end")
        assert f() == 2000.0
