"""
test_swashplate_servo_directions.py — validates servo output directions for cyclic commands.

This test catches bugs where cyclic parameters are swapped in mixing matrices.
The round-trip tests would miss this because if both forward and inverse are swapped
identically, the round-trip still works.

The key insight: servo positions must move in physically meaningful directions for
cyclic commands, not just round-trip correctly.
"""
import math
import sys
from pathlib import Path

import pytest

from swashplate import SwashplateServoModel, ardupilot_h3_120_forward, collective_rad_to_out


# Physical limits
_COL_MIN = -0.28  # rad
_COL_MAX = 0.10   # rad

# High slew rate for immediate settling
_DT = 1.0 / 400.0


class TestServoDirectionsForCyclicCommands:
    """Verify servo positions move correctly for cyclic commands."""

    def _get_servo_positions(self, col_rad, tilt_lon, tilt_lat):
        """Return servo positions for a given collective and cyclic command."""
        # Use the forward mix directly to get servo positions
        col_out = collective_rad_to_out(col_rad, _COL_MIN, _COL_MAX)
        s1, s2, s3 = ardupilot_h3_120_forward(
            col_out, tilt_lat, tilt_lon,  # Note: CORRECT order is (col, roll_norm, pitch_norm)
            h_col_min=1000.0, h_col_max=2000.0
        )
        return s1, s2, s3

    def test_positive_roll_tilts_disk_right(self):
        """
        Positive tilt_lat (roll right disk) should:
        - Move servo S1 (front-right, -60°) UP (larger/more positive)
        - Move servo S2 (front-left, +60°) DOWN (smaller/more negative)
        - Leave servo S3 (back, 180°) unchanged
        """
        col = -0.18
        # Get baseline servo positions
        s1_base, s2_base, s3_base = self._get_servo_positions(col, tilt_lon=0.0, tilt_lat=0.0)
        
        # Command positive roll (tilt disk right)
        s1_roll, s2_roll, s3_roll = self._get_servo_positions(col, tilt_lon=0.0, tilt_lat=0.3)
        
        # S1 (front-right) should move UP when rolling right
        assert s1_roll > s1_base, (
            f"S1 should increase for positive tilt_lat; "
            f"base={s1_base:.4f}, roll={s1_roll:.4f}"
        )
        
        # S2 (front-left) should move DOWN when rolling right
        assert s2_roll < s2_base, (
            f"S2 should decrease for positive tilt_lat; "
            f"base={s2_base:.4f}, roll={s2_roll:.4f}"
        )
        
        # S3 (back) should not change significantly with roll
        assert math.isclose(s3_roll, s3_base, abs_tol=0.01), (
            f"S3 should not change with roll; base={s3_base:.4f}, roll={s3_roll:.4f}"
        )

    def test_negative_roll_tilts_disk_left(self):
        """
        Negative tilt_lat (roll left disk) should:
        - Move servo S1 (front-right) DOWN (smaller/more negative)
        - Move servo S2 (front-left) UP (larger/more positive)
        - Leave servo S3 (back) unchanged
        """
        col = -0.18
        s1_base, s2_base, s3_base = self._get_servo_positions(col, tilt_lon=0.0, tilt_lat=0.0)
        
        # Command negative roll (tilt disk left)
        s1_roll, s2_roll, s3_roll = self._get_servo_positions(col, tilt_lon=0.0, tilt_lat=-0.3)
        
        # S1 should move DOWN when rolling left
        assert s1_roll < s1_base, (
            f"S1 should decrease for negative tilt_lat; "
            f"base={s1_base:.4f}, roll={s1_roll:.4f}"
        )
        
        # S2 should move UP when rolling left
        assert s2_roll > s2_base, (
            f"S2 should increase for negative tilt_lat; "
            f"base={s2_base:.4f}, roll={s2_roll:.4f}"
        )
        
        # S3 should not change
        assert math.isclose(s3_roll, s3_base, abs_tol=0.01)

    def test_positive_pitch_tilts_disk_nose_down(self):
        """
        Positive tilt_lon (pitch nose-down disk) should:
        - Move servo S3 (back) DOWN (smaller/more negative) — helps nose down
        - Move servo S1 and S2 (front pair) UP (larger/more positive) — helps nose down
        """
        col = -0.18
        s1_base, s2_base, s3_base = self._get_servo_positions(col, tilt_lon=0.0, tilt_lat=0.0)
        
        # Command positive pitch (nose-down tilt)
        s1_pitch, s2_pitch, s3_pitch = self._get_servo_positions(col, tilt_lon=0.3, tilt_lat=0.0)
        
        # S3 (back) should move DOWN for nose-down pitch
        assert s3_pitch < s3_base, (
            f"S3 should decrease for positive tilt_lon; "
            f"base={s3_base:.4f}, pitch={s3_pitch:.4f}"
        )
        
        # S1 and S2 (front pair) should move UP for nose-down pitch
        assert s1_pitch > s1_base, (
            f"S1 should increase for positive tilt_lon; "
            f"base={s1_base:.4f}, pitch={s1_pitch:.4f}"
        )
        assert s2_pitch > s2_base, (
            f"S2 should increase for positive tilt_lon; "
            f"base={s2_base:.4f}, pitch={s2_pitch:.4f}"
        )

    def test_negative_pitch_tilts_disk_nose_up(self):
        """
        Negative tilt_lon (pitch nose-up disk) should:
        - Move servo S3 (back) UP (larger/more positive)
        - Move servo S1 and S2 (front pair) DOWN (smaller/more negative)
        """
        col = -0.18
        s1_base, s2_base, s3_base = self._get_servo_positions(col, tilt_lon=0.0, tilt_lat=0.0)
        
        # Command negative pitch (nose-up tilt)
        s1_pitch, s2_pitch, s3_pitch = self._get_servo_positions(col, tilt_lon=-0.3, tilt_lat=0.0)
        
        # S3 should move UP for nose-up pitch
        assert s3_pitch > s3_base, (
            f"S3 should increase for negative tilt_lon; "
            f"base={s3_base:.4f}, pitch={s3_pitch:.4f}"
        )
        
        # S1 and S2 should move DOWN for nose-up pitch
        assert s1_pitch < s1_base, (
            f"S1 should decrease for negative tilt_lon; "
            f"base={s1_base:.4f}, pitch={s1_pitch:.4f}"
        )
        assert s2_pitch < s2_base, (
            f"S2 should decrease for negative tilt_lon; "
            f"base={s2_base:.4f}, pitch={s2_pitch:.4f}"
        )

    def test_combined_roll_and_pitch(self):
        """
        Combined cyclic: roll right + pitch nose-down should:
        - S1: move UP from pitch, but reduce from roll (net depends on magnitudes)
        - S2: move UP from pitch, move DOWN from roll (net down)
        - S3: move DOWN from pitch (roll doesn't affect it)
        """
        col = -0.18
        s1_base, s2_base, s3_base = self._get_servo_positions(col, 0.0, 0.0)
        
        # Small positive roll, positive pitch
        s1_cmd, s2_cmd, s3_cmd = self._get_servo_positions(col, tilt_lon=0.2, tilt_lat=0.2)
        
        # S3 is pure pitch: should definitely move DOWN
        assert s3_cmd < s3_base, "S3 should decrease with positive pitch"
        
        # S2 gets pitch UP and roll DOWN: net effect should be DOWN
        assert s2_cmd < s2_base, (
            f"S2 affected by pitch (↑) and roll (↓); with equal magnitudes should go down; "
            f"base={s2_base:.4f}, cmd={s2_cmd:.4f}"
        )

    def test_servo_model_step_preserves_servo_directions(self):
        """
        SwashplateServoModel.step() should produce servo positions with
        the same directional properties as direct forward_mix.
        """
        m = SwashplateServoModel(
            slew_rate_deg_s=1e9,  # instant settling
            travel_deg=60.0,
            col_min_rad=_COL_MIN,
            col_max_rad=_COL_MAX,
        )
        
        col = -0.18
        
        # Get baseline from servo model
        m.reset(col, tilt_lon=0.0, tilt_lat=0.0)
        m.step(col, 0.0, 0.0, _DT)
        s1_base = m._s[0]
        s2_base = m._s[1]
        s3_base = m._s[2]
        
        # Step with roll (positional: col, tilt_lon, tilt_lat, dt)
        m.step(col, 0.0, 0.3, _DT)
        s1_roll = m._s[0]
        s2_roll = m._s[1]
        s3_roll = m._s[2]
        
        # S1 should move up, S2 down, S3 unchanged for positive roll
        assert s1_roll > s1_base, "ServoModel: S1 should increase with positive tilt_lat"
        assert s2_roll < s2_base, "ServoModel: S2 should decrease with positive tilt_lat"
        assert math.isclose(s3_roll, s3_base, abs_tol=0.01), "ServoModel: S3 should not change with tilt_lat"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
