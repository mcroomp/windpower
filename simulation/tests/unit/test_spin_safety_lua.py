"""Unit tests for the over-spin auto-disarm safety in rawes.lua.

Safety rule: if the body spin rate about ANY axis exceeds SPIN_LIMIT_RPM (60 RPM
= 2*pi rad/s) continuously for SPIN_LIMIT_MS (20 s), the vehicle auto-disarms.
Runs in every mode while armed; any dip below the limit resets the timer.
"""
from __future__ import annotations

import math

import pytest

from rawes_lua_harness import RawesLua
_OVER  = 7.0    # rad/s  > 2*pi (~6.283) -> over the 60 RPM limit
_UNDER = 6.0    # rad/s  < 2*pi           -> within limits


def _sim() -> RawesLua:
    return RawesLua()   # default SCR_USER6=0 (MODE_NONE) -- safety runs in all modes


class TestOverSpinDisarm:
    def test_sustained_overspin_disarms_after_20s(self):
        sim = _sim()
        sim.armed = True
        sim.gyro = [_OVER, 0.0, 0.0]        # overspin about x
        sim.run(19.0)
        assert sim.armed, "must NOT disarm before 20 s of sustained overspin"
        sim.run(2.0)                         # total 21 s > 20 s
        assert not sim.armed, "must auto-disarm after 20 s of sustained overspin"

    def test_overspin_about_y_and_z_also_trips(self):
        for axis in range(3):
            sim = _sim()
            sim.armed = True
            g = [0.0, 0.0, 0.0]
            g[axis] = -_OVER                 # negative rate must trip too (abs)
            sim.gyro = g
            sim.run(21.0)
            assert not sim.armed, f"axis {axis} overspin should auto-disarm"

    def test_dip_below_resets_timer(self):
        sim = _sim()
        sim.armed = True
        sim.gyro = [_OVER, 0.0, 0.0]
        sim.run(15.0)
        sim.gyro = [0.0, 0.0, 0.0]           # drop below -> resets the latch
        sim.run(1.0)
        assert sim.fns.spin_over_since_ms() is None
        sim.gyro = [_OVER, 0.0, 0.0]         # back over, but only 15 s continuous
        sim.run(15.0)
        assert sim.armed, "timer must reset on a dip; 15 s < 20 s must not disarm"

    def test_under_limit_never_disarms(self):
        sim = _sim()
        sim.armed = True
        sim.gyro = [_UNDER, _UNDER, _UNDER]  # each axis just under the limit
        sim.run(25.0)
        assert sim.armed, "rates below the limit must never disarm"

    def test_no_action_while_disarmed(self):
        sim = _sim()
        sim.armed = False
        sim.gyro = [10.0, 0.0, 0.0]          # huge, but disarmed
        sim.run(25.0)
        assert not sim.armed
        assert sim.fns.spin_over_since_ms() is None

    def test_limit_is_60_rpm(self):
        sim = _sim()
        assert float(sim.fns.SPIN_LIMIT_RPM) == 60.0
        assert float(sim.fns.SPIN_LIMIT_RADS) == pytest.approx(math.pi * 2.0)
        assert int(sim.fns.SPIN_LIMIT_MS) == 20000
