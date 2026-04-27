"""
test_swashplate_servo_model.py — unit tests for SwashplateServoModel.

Covers:
  - Instantiation and initial state
  - Immediate settling when slew rate is very high
  - Slew limiting: collective and cyclic arrive at the correct rate
  - Per-servo clipping: extreme commands are clipped to [-1, +1]
  - Collective / cyclic coupling through hardware limits
  - reset() restores servo positions
  - Property accessors (collective_rad, tilt_lon, tilt_lat)
  - from_rotor() constructor
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from swashplate import SwashplateServoModel, collective_rad_to_out
import rotor_definition as rd

# Physical limits matching test_pump_cycle_unified defaults
_COL_MIN = -0.28  # rad
_COL_MAX =  0.10  # rad

# DS113MG V6.0: 333 deg/s at 6V, 60 deg travel
_SLEW_DEG_S = 333.0
_TRAVEL_DEG =  60.0

# One 400 Hz timestep
_DT = 1.0 / 400.0


def _make() -> SwashplateServoModel:
    return SwashplateServoModel(
        slew_rate_deg_s=_SLEW_DEG_S,
        travel_deg=_TRAVEL_DEG,
        col_min_rad=_COL_MIN,
        col_max_rad=_COL_MAX,
    )


# ---------------------------------------------------------------------------
# Instantiation
# ---------------------------------------------------------------------------

class TestInstantiation:
    def test_initial_collective_at_col_min(self):
        m = _make()
        assert math.isclose(m.collective_rad, _COL_MIN, abs_tol=1e-9)

    def test_initial_cyclic_zero(self):
        m = _make()
        assert math.isclose(m.tilt_lon, 0.0, abs_tol=1e-9)
        assert math.isclose(m.tilt_lat, 0.0, abs_tol=1e-9)

    def test_from_rotor_constructs(self):
        rotor = rd.default()
        m = SwashplateServoModel.from_rotor(rotor, col_min_rad=_COL_MIN, col_max_rad=_COL_MAX)
        assert isinstance(m, SwashplateServoModel)
        assert math.isclose(m.collective_rad, _COL_MIN, abs_tol=1e-9)


# ---------------------------------------------------------------------------
# Immediate settling (infinite slew rate)
# ---------------------------------------------------------------------------

class TestImmediateSettling:
    """With very high slew rate, output equals command after one step."""

    def _make_fast(self) -> SwashplateServoModel:
        return SwashplateServoModel(
            slew_rate_deg_s=1e9,
            travel_deg=60.0,
            col_min_rad=_COL_MIN,
            col_max_rad=_COL_MAX,
        )

    @pytest.mark.parametrize("col_rad", [-0.28, -0.18, -0.10, 0.00, 0.10])
    def test_collective_settles_immediately(self, col_rad):
        m = self._make_fast()
        col_act, _, _ = m.step(col_rad, 0.0, 0.0, _DT)
        assert math.isclose(col_act, col_rad, abs_tol=1e-6)

    @pytest.mark.parametrize("tlon", [-0.5, 0.0, 0.5])
    def test_tilt_lon_settles_immediately(self, tlon):
        m = self._make_fast()
        _, tlon_act, _ = m.step(-0.18, tlon, 0.0, _DT)
        assert math.isclose(tlon_act, tlon, abs_tol=1e-6)

    @pytest.mark.parametrize("tlat", [-0.5, 0.0, 0.5])
    def test_tilt_lat_settles_immediately(self, tlat):
        m = self._make_fast()
        _, _, tlat_act = m.step(-0.18, 0.0, tlat, _DT)
        assert math.isclose(tlat_act, tlat, abs_tol=1e-6)

    def test_round_trip_combined(self):
        m = self._make_fast()
        col_act, tlon_act, tlat_act = m.step(-0.10, 0.3, -0.2, _DT)
        assert math.isclose(col_act,  -0.10, abs_tol=1e-6)
        assert math.isclose(tlon_act,  0.30, abs_tol=1e-6)
        assert math.isclose(tlat_act, -0.20, abs_tol=1e-6)


# ---------------------------------------------------------------------------
# Slew limiting
# ---------------------------------------------------------------------------

class TestSlewLimiting:
    def test_collective_slews_at_correct_rate(self):
        """
        max_servo_rate = 2 * slew_deg_s / travel_deg [norm/s].
        After one DT step the servo moves at most max_servo_rate * DT.
        For small collective commands the output should lag behind.
        """
        m = _make()
        max_rate = 2.0 * _SLEW_DEG_S / _TRAVEL_DEG  # ~11.1 norm/s

        # Start at col_min, command col_max — a large step that exceeds one-step slew
        col_min_out = collective_rad_to_out(_COL_MIN, _COL_MIN, _COL_MAX)   # = 0.0
        # Servo starts at col_min_out; command col_max → servo should move by max_rate*DT
        col_act, _, _ = m.step(_COL_MAX, 0.0, 0.0, _DT)

        # The slew rate is in servo space [-1, +1]; the forward mix rescales output
        # via s = 2*output - 1, so a step of max_rate*dt in servo space corresponds
        # to max_rate*dt/2 in collective_out [0..1] space (with default H_COL params).
        col_out_act = collective_rad_to_out(col_act, _COL_MIN, _COL_MAX)
        expected_step = max_rate * _DT / 2.0
        actual_step = col_out_act - col_min_out
        assert math.isclose(actual_step, expected_step, rel_tol=1e-4), (
            f"collective moved {actual_step:.5f} in one step; expected {expected_step:.5f}"
        )

    def test_small_step_settles_in_one_dt(self):
        """A command that fits within one DT step is applied immediately."""
        m = _make()
        max_rate = 2.0 * _SLEW_DEG_S / _TRAVEL_DEG
        # A tiny step well within one-step limit
        tiny_col_out = collective_rad_to_out(_COL_MIN, _COL_MIN, _COL_MAX) + max_rate * _DT * 0.1
        tiny_col_rad = _COL_MIN + tiny_col_out * (_COL_MAX - _COL_MIN)
        col_act, _, _ = m.step(tiny_col_rad, 0.0, 0.0, _DT)
        assert math.isclose(col_act, tiny_col_rad, abs_tol=1e-6)

    def test_collective_reaches_target_over_time(self):
        """Over enough steps the collective must arrive at the commanded value."""
        m = _make()
        target = -0.10
        for _ in range(2000):   # 5 s at 400 Hz is more than enough
            result = m.step(target, 0.0, 0.0, _DT)
        assert math.isclose(result[0], target, abs_tol=1e-6)

    def test_cyclic_rate_limited_independently(self):
        """Cyclic from 0 to 1.0 is limited the same way as collective."""
        m = _make()
        max_rate = 2.0 * _SLEW_DEG_S / _TRAVEL_DEG
        # Reset at cruise collective so collective is already settled
        m.reset(_COL_MIN, tilt_lon=0.0, tilt_lat=0.0)
        _, tlon_act, _ = m.step(_COL_MIN, 1.0, 0.0, _DT)
        # tilt_lon goes through roll factor; check it's bounded by slew
        assert abs(tlon_act) <= max_rate * _DT + 1e-9


# ---------------------------------------------------------------------------
# Servo clipping
# ---------------------------------------------------------------------------

class TestServoClipping:
    def test_extreme_collective_clipped(self):
        """Commands above col_max are silently clipped to col_max."""
        m = SwashplateServoModel(1e9, 60.0, _COL_MIN, _COL_MAX)
        col_act, _, _ = m.step(0.50, 0.0, 0.0, _DT)   # 0.50 > col_max=0.10
        assert col_act <= _COL_MAX + 1e-9

    def test_extreme_collective_below_min_clipped(self):
        """Commands below col_min are clipped to col_min."""
        m = SwashplateServoModel(1e9, 60.0, _COL_MIN, _COL_MAX)
        col_act, _, _ = m.step(-0.50, 0.0, 0.0, _DT)  # -0.50 < col_min=-0.28
        assert col_act >= _COL_MIN - 1e-9

    def test_combined_saturation_reduces_cyclic(self):
        """
        Max collective + max cyclic saturates the servos.
        The inverse mix then returns a cyclic smaller than commanded.
        """
        m = SwashplateServoModel(1e9, 60.0, _COL_MIN, _COL_MAX)
        # Command extreme collective and extreme cyclic simultaneously
        _, tlon_act, tlat_act = m.step(_COL_MAX, 1.0, 1.0, _DT)
        # With saturation the cyclic must be reduced below the command
        assert abs(tlon_act) < 1.0 or abs(tlat_act) < 1.0


# ---------------------------------------------------------------------------
# reset()
# ---------------------------------------------------------------------------

class TestReset:
    def test_reset_to_col_min(self):
        m = _make()
        m.step(-0.10, 0.5, 0.0, _DT)   # move servos away from initial state
        m.reset()
        assert math.isclose(m.collective_rad, _COL_MIN, abs_tol=1e-9)
        assert math.isclose(m.tilt_lon,       0.0,      abs_tol=1e-9)
        assert math.isclose(m.tilt_lat,       0.0,      abs_tol=1e-9)

    def test_reset_to_given_collective(self):
        m = _make()
        m.reset(-0.18)
        assert math.isclose(m.collective_rad, -0.18, abs_tol=1e-9)

    def test_reset_with_cyclic(self):
        m = SwashplateServoModel(1e9, 60.0, _COL_MIN, _COL_MAX)
        m.reset(-0.18, tilt_lon=0.3, tilt_lat=-0.2)
        assert math.isclose(m.collective_rad, -0.18, abs_tol=1e-6)
        assert math.isclose(m.tilt_lon,        0.3,  abs_tol=1e-6)
        assert math.isclose(m.tilt_lat,       -0.2,  abs_tol=1e-6)


# ---------------------------------------------------------------------------
# Property accessors
# ---------------------------------------------------------------------------

class TestProperties:
    def test_properties_consistent_with_step_output(self):
        m = SwashplateServoModel(1e9, 60.0, _COL_MIN, _COL_MAX)
        col_ret, tlon_ret, tlat_ret = m.step(-0.15, 0.2, -0.1, _DT)
        assert math.isclose(m.collective_rad, col_ret,  abs_tol=1e-9)
        assert math.isclose(m.tilt_lon,       tlon_ret, abs_tol=1e-9)
        assert math.isclose(m.tilt_lat,       tlat_ret, abs_tol=1e-9)

    def test_properties_update_each_step(self):
        m = SwashplateServoModel(1e9, 60.0, _COL_MIN, _COL_MAX)
        m.step(-0.28, 0.0, 0.0, _DT)
        m.step(-0.10, 0.3, 0.0, _DT)
        assert math.isclose(m.collective_rad, -0.10, abs_tol=1e-6)
        assert math.isclose(m.tilt_lon,        0.3,  abs_tol=1e-6)


# ---------------------------------------------------------------------------
# Non-default H_COL_MIN / H_COL_MAX
# ---------------------------------------------------------------------------

class TestCustomColLimits:
    def test_non_default_h_col(self):
        """H_COL_MIN=1100, H_COL_MAX=1900 narrows the collective range."""
        m = SwashplateServoModel(1e9, 60.0, _COL_MIN, _COL_MAX, h_col_min=1100.0, h_col_max=1900.0)
        col_act, _, _ = m.step(-0.18, 0.0, 0.0, _DT)
        assert math.isclose(col_act, -0.18, abs_tol=1e-6)

    def test_h_col_limits_preserved_through_step(self):
        """round-trip with non-default H_COL params does not corrupt collective."""
        for col in (-0.28, -0.18, 0.0, 0.10):
            m = SwashplateServoModel(1e9, 60.0, _COL_MIN, _COL_MAX, h_col_min=1100.0, h_col_max=1900.0)
            col_act, _, _ = m.step(col, 0.0, 0.0, _DT)
            assert math.isclose(col_act, col, abs_tol=1e-6), f"col={col} failed round-trip"
