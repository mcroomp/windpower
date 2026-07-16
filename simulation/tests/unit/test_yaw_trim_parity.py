"""
test_yaw_trim_parity.py -- Cross-check: rawes.lua yaw_trim_step() vs
controller.YawTrimObserver (Python port).

Runs identical (dt, u, psi_dot) sequences through both the real Lua
implementation (via RawesLua) and the Python port, and asserts the trim
state stays numerically identical at every step. See test_yaw_trim_lua.py
for standalone Lua-side behavioral tests of the same function.
"""
from __future__ import annotations

import pytest

from controller import YawTrimObserver
from rawes_lua_harness import RawesLua

_DT = 0.01   # 100 Hz nominal tick


def _sim() -> RawesLua:
    sim = RawesLua()
    sim.fns.yaw_trim_reset()
    return sim


def _python_observer(sim: RawesLua) -> YawTrimObserver:
    """Python port configured with the same defaults as the Lua harness."""
    obs = YawTrimObserver(yff_max=float(sim.fns.YFF_MAX), yff_tau=float(sim.fns.YFF_TAU))
    assert obs.yff_a == pytest.approx(float(sim.fns.YFF_A), rel=1e-9)
    return obs


class TestStepParity:
    def test_zero_rate_matches(self):
        sim = _sim()
        obs = _python_observer(sim)
        for u in (0.0, 0.2, 0.4, 0.7, 1.0):
            lua_out = float(sim.fns.yaw_trim_step(_DT, u, 0.0))
            py_out  = obs.step(_DT, u, 0.0)
            assert py_out == pytest.approx(lua_out, abs=1e-12)

    def test_nonzero_rate_matches(self):
        sim = _sim()
        obs = _python_observer(sim)
        for u, psi_dot in [(0.5, -1.0), (0.3, 2.0), (0.6, -5.0), (0.1, 0.5)]:
            lua_out = float(sim.fns.yaw_trim_step(_DT, u, psi_dot))
            py_out  = obs.step(_DT, u, psi_dot)
            assert py_out == pytest.approx(lua_out, abs=1e-12)

    def test_low_pass_convergence_matches_over_many_steps(self):
        sim = _sim()
        obs = _python_observer(sim)
        u, psi_dot = 0.45, -0.8
        for _ in range(500):
            lua_out = float(sim.fns.yaw_trim_step(_DT, u, psi_dot))
            py_out  = obs.step(_DT, u, psi_dot)
            assert py_out == pytest.approx(lua_out, abs=1e-10)

    def test_clamp_lower_bound_matches(self):
        sim = _sim()
        obs = _python_observer(sim)
        for _ in range(50):
            lua_out = float(sim.fns.yaw_trim_step(_DT, 0.0, 5.0))
            py_out  = obs.step(_DT, 0.0, 5.0)
            assert py_out == pytest.approx(lua_out, abs=1e-12)
            assert py_out >= 0.0

    def test_clamp_upper_bound_matches(self):
        sim = _sim()
        obs = _python_observer(sim)
        ymax = float(sim.fns.YFF_MAX)
        for _ in range(2000):
            lua_out = float(sim.fns.yaw_trim_step(_DT, 0.0, -50.0))
            py_out  = obs.step(_DT, 0.0, -50.0)
            assert py_out == pytest.approx(lua_out, abs=1e-9)
            assert py_out <= ymax + 1e-9

    def test_reset_matches(self):
        sim = _sim()
        obs = _python_observer(sim)
        float(sim.fns.yaw_trim_step(10.0, 0.5, 0.0))
        obs.step(10.0, 0.5, 0.0)
        sim.fns.yaw_trim_reset()
        obs.reset()
        assert float(sim.fns.yaw_ff_trim()) == 0.0
        assert obs.trim == 0.0


class TestClosedLoopEquilibriumParity:
    """Drive the same synthetic plant through both observers and check they
    converge to the identical trim/psi_dot trajectory step-by-step."""

    def test_converges_identically(self):
        sim = _sim()
        obs = _python_observer(sim)
        a     = obs.yff_a
        omega = 28.0
        u_eq  = omega / a

        lua_trim, py_trim = 0.0, 0.0
        lua_psi_dot = py_psi_dot = -omega
        for _ in range(3000):
            lua_trim = float(sim.fns.yaw_trim_step(_DT, lua_trim, lua_psi_dot))
            py_trim  = obs.step(_DT, py_trim, py_psi_dot)
            assert py_trim == pytest.approx(lua_trim, abs=1e-9)
            lua_psi_dot = a * (lua_trim - u_eq)
            py_psi_dot  = a * (py_trim - u_eq)

        assert lua_trim == pytest.approx(u_eq, abs=0.01)
        assert py_trim == pytest.approx(u_eq, abs=0.01)
