"""
test_swashplate.py — unit tests for the ArduPilot H3-120 swashplate clone.

Verifies that ardupilot_h3_120_forward and ardupilot_h3_120_inverse are exact
inverses of each other and produce values that match the ArduPilot signal flow
documented in hardware/ardupilot_swashplate.md.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from swashplate import (
    ardupilot_h3_120_forward,
    ardupilot_h3_120_inverse,
    collective_out_to_rad,
    collective_rad_to_out,
    cyclic_to_blade_pitches,
    pwm_to_normalized,
    normalized_to_pwm,
)

# Physical collective limits matching rawes_sitl_defaults.parm / Lua constants
_COL_MIN = -0.28   # rad
_COL_MAX =  0.10   # rad
_PITCH_GAIN = 0.3  # rad / normalised unit (beaupoil_2026)


# ---------------------------------------------------------------------------
# pwm_to_normalized
# ---------------------------------------------------------------------------

def test_pwm_to_normalized_endpoints():
    assert pwm_to_normalized(1000.0) == -1.0
    assert pwm_to_normalized(1500.0) ==  0.0
    assert pwm_to_normalized(2000.0) ==  1.0


def test_pwm_to_normalized_midpoint():
    assert pwm_to_normalized(1250.0) == -0.5
    assert pwm_to_normalized(1750.0) ==  0.5


def test_normalized_to_pwm_round_trip():
    for pwm in (1000.0, 1263.0, 1500.0, 1750.0, 2000.0):
        assert abs(normalized_to_pwm(pwm_to_normalized(pwm)) - pwm) < 1e-9


# ---------------------------------------------------------------------------
# ardupilot_h3_120_forward — known values from ardupilot_swashplate.md
# ---------------------------------------------------------------------------

def test_forward_pure_collective_equal_servos():
    """Pure collective (zero cyclic) produces equal outputs on all three servos."""
    s1, s2, s3 = ardupilot_h3_120_forward(0.5, 0.0, 0.0)
    assert math.isclose(s1, s2, abs_tol=1e-12)
    assert math.isclose(s1, s3, abs_tol=1e-12)


def test_forward_col_min_maps_to_minus_one():
    """collective_out=0 with H_COL_MIN=1000 → col_scaled=0 → output=0 → s=-1."""
    s1, s2, s3 = ardupilot_h3_120_forward(0.0, 0.0, 0.0, h_col_min=1000, h_col_max=2000)
    assert math.isclose(s1, -1.0, abs_tol=1e-12)
    assert math.isclose(s2, -1.0, abs_tol=1e-12)
    assert math.isclose(s3, -1.0, abs_tol=1e-12)


def test_forward_col_max_maps_to_plus_one():
    """collective_out=1 with H_COL_MAX=2000 → col_scaled=1 → output=1 → s=+1."""
    s1, s2, s3 = ardupilot_h3_120_forward(1.0, 0.0, 0.0, h_col_min=1000, h_col_max=2000)
    assert math.isclose(s1, 1.0, abs_tol=1e-12)
    assert math.isclose(s2, 1.0, abs_tol=1e-12)
    assert math.isclose(s3, 1.0, abs_tol=1e-12)


def test_forward_lua_cruise_collective():
    """Lua COL_CRUISE=-0.18 rad encodes as collective_out=0.2632, servo s≈-0.4737."""
    col_out = collective_rad_to_out(-0.18, _COL_MIN, _COL_MAX)
    assert math.isclose(col_out, 0.10 / 0.38, rel_tol=1e-6)   # = 0.26316
    s1, s2, s3 = ardupilot_h3_120_forward(col_out, 0.0, 0.0)
    expected_s = 2.0 * col_out - 1.0   # = -0.47368 (full H_COL range)
    assert math.isclose(s1, expected_s, abs_tol=1e-6)
    # Corresponding PWM
    pwm = normalized_to_pwm(s1)
    assert math.isclose(pwm, 1000.0 + col_out * 1000.0, abs_tol=0.1)


def test_forward_pure_roll_ch1_ch2_antisymmetric():
    """Pure roll: CH1 and CH2 move equally and oppositely, CH3 unchanged."""
    col_out = 0.5
    roll = 0.4
    s1, s2, s3 = ardupilot_h3_120_forward(col_out, roll, 0.0)
    s1_0, s2_0, s3_0 = ardupilot_h3_120_forward(col_out, 0.0, 0.0)
    # CH1 and CH2 are symmetric about the collective baseline
    assert math.isclose(s1 - s1_0, -(s2 - s2_0), abs_tol=1e-12)
    # CH3 has zero roll factor: pitch factor only → should not change with roll
    assert math.isclose(s3, s3_0, abs_tol=1e-12)


def test_forward_pure_pitch_ch3_changes_most():
    """Pure pitch: CH3 (at 180 deg, pitch factor -0.45) has largest deflection."""
    col_out = 0.5
    pitch = 0.4
    s1, s2, s3 = ardupilot_h3_120_forward(col_out, 0.0, pitch)
    s1_0, s2_0, s3_0 = ardupilot_h3_120_forward(col_out, 0.0, 0.0)
    d3 = abs(s3 - s3_0)
    d1 = abs(s1 - s1_0)
    assert d3 > d1   # CH3 pitch factor=0.45 > CH1/2 pitch factor=0.225


def test_forward_mixing_matrix_values():
    """Verify individual mixing factors against ardupilot_swashplate.md table."""
    # CH1: rollFactor=+0.3897, pitchFactor=+0.2250, collectiveFactor=1.0
    # For col_scaled=0, roll=1, pitch=0: output1 = 0.3897, rescaled = 2*0.3897-1
    s1, _, _ = ardupilot_h3_120_forward(0.0, 1.0, 0.0)
    assert math.isclose(s1, 2.0 * 0.3897 - 1.0, abs_tol=1e-4)

    # CH3: rollFactor=0, pitchFactor=-0.45, collectiveFactor=1.0
    # For col_scaled=0, roll=0, pitch=1: output3 = -0.45, rescaled = 2*(-0.45)-1
    _, _, s3 = ardupilot_h3_120_forward(0.0, 0.0, 1.0)
    assert math.isclose(s3, 2.0 * (-0.45) - 1.0, abs_tol=1e-4)


# ---------------------------------------------------------------------------
# ardupilot_h3_120_inverse — round-trip with forward
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("col_out", [0.0, 0.1, 0.263, 0.5, 0.75, 1.0])
def test_inverse_pure_collective_round_trip(col_out):
    s1, s2, s3 = ardupilot_h3_120_forward(col_out, 0.0, 0.0)
    col_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
    assert math.isclose(col_rt, col_out, abs_tol=1e-10)
    assert math.isclose(roll_rt,  0.0,   abs_tol=1e-10)
    assert math.isclose(pitch_rt, 0.0,   abs_tol=1e-10)


@pytest.mark.parametrize("roll", [-1.0, -0.5, 0.0, 0.5, 1.0])
def test_inverse_pure_roll_round_trip(roll):
    s1, s2, s3 = ardupilot_h3_120_forward(0.5, roll, 0.0)
    col_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
    assert math.isclose(col_rt,   0.5,  abs_tol=1e-10)
    assert math.isclose(roll_rt,  roll, abs_tol=1e-10)
    assert math.isclose(pitch_rt, 0.0,  abs_tol=1e-10)


@pytest.mark.parametrize("pitch", [-1.0, -0.5, 0.0, 0.5, 1.0])
def test_inverse_pure_pitch_round_trip(pitch):
    s1, s2, s3 = ardupilot_h3_120_forward(0.5, 0.0, pitch)
    col_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
    assert math.isclose(col_rt,   0.5,   abs_tol=1e-10)
    assert math.isclose(roll_rt,  0.0,   abs_tol=1e-10)
    assert math.isclose(pitch_rt, pitch, abs_tol=1e-10)


def test_inverse_combined_round_trip():
    """Arbitrary combined command round-trips exactly."""
    col, roll, pitch = 0.263, 0.15, -0.08
    s1, s2, s3 = ardupilot_h3_120_forward(col, roll, pitch)
    col_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
    assert math.isclose(col_rt,   col,   abs_tol=1e-10)
    assert math.isclose(roll_rt,  roll,  abs_tol=1e-10)
    assert math.isclose(pitch_rt, pitch, abs_tol=1e-10)


def test_inverse_neutral_servos():
    """All servos at 1500 µs (s=0) → collective_out=0.5, roll=0, pitch=0."""
    col_out, roll_norm, pitch_norm = ardupilot_h3_120_inverse(0.0, 0.0, 0.0)
    assert math.isclose(col_out,   0.5, abs_tol=1e-12)
    assert math.isclose(roll_norm, 0.0, abs_tol=1e-12)
    assert math.isclose(pitch_norm, 0.0, abs_tol=1e-12)


def test_inverse_no_cross_coupling_pure_roll():
    """Pure roll command produces zero pitch in the inverse (no cross-coupling)."""
    s1, s2, s3 = ardupilot_h3_120_forward(0.5, 0.4, 0.0)
    _, _, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
    assert abs(pitch_rt) < 1e-10, f"Spurious pitch from pure roll: {pitch_rt}"


def test_inverse_no_cross_coupling_pure_pitch():
    """Pure pitch command produces zero roll in the inverse (no cross-coupling)."""
    s1, s2, s3 = ardupilot_h3_120_forward(0.5, 0.0, 0.4)
    _, roll_rt, _ = ardupilot_h3_120_inverse(s1, s2, s3)
    assert abs(roll_rt) < 1e-10, f"Spurious roll from pure pitch: {roll_rt}"


def test_inverse_from_pwm_via_pwm_to_normalized():
    """Decode MAVLink SERVO_OUTPUT_RAW PWM values via pwm_to_normalized."""
    col_out = collective_rad_to_out(-0.18, _COL_MIN, _COL_MAX)  # 0.2632
    s1_fwd, s2_fwd, s3_fwd = ardupilot_h3_120_forward(col_out, 0.0, 0.0)
    # Simulate what ArduPilot would output as PWM
    pwm1 = normalized_to_pwm(s1_fwd)
    pwm2 = normalized_to_pwm(s2_fwd)
    pwm3 = normalized_to_pwm(s3_fwd)
    # Decode via pwm_to_normalized (as done in test_tail_tilt and test_h_phang)
    col_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(
        pwm_to_normalized(pwm1),
        pwm_to_normalized(pwm2),
        pwm_to_normalized(pwm3),
    )
    col_rad_rt = collective_out_to_rad(col_rt, _COL_MIN, _COL_MAX)
    assert math.isclose(col_rad_rt, -0.18, abs_tol=1e-4)
    assert math.isclose(roll_rt,   0.0,   abs_tol=1e-9)
    assert math.isclose(pitch_rt,  0.0,   abs_tol=1e-9)


# ---------------------------------------------------------------------------
# H_COL_MIN / H_COL_MAX scaling
# ---------------------------------------------------------------------------

def test_default_h_col_range_is_full():
    """With H_COL_MIN=1000, H_COL_MAX=2000: collective_out equals collective_out_scaled."""
    # col_out=0.263 → col_scaled=0.263 → s = 2*0.263-1 = -0.474
    col_out = 0.263
    s1, s2, s3 = ardupilot_h3_120_forward(col_out, 0.0, 0.0,
                                           h_col_min=1000, h_col_max=2000)
    assert math.isclose(s1, 2.0 * col_out - 1.0, abs_tol=1e-10)


def test_h_col_defaults_round_trip():
    """With H_COL_MIN=1250, H_COL_MAX=1750 (ArduPilot defaults), round-trip holds."""
    col_out = 0.263
    s1, s2, s3 = ardupilot_h3_120_forward(col_out, 0.1, -0.05,
                                           h_col_min=1250, h_col_max=1750)
    col_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3,
                                                          h_col_min=1250, h_col_max=1750)
    assert math.isclose(col_rt,   col_out, abs_tol=1e-10)
    assert math.isclose(roll_rt,  0.1,     abs_tol=1e-10)
    assert math.isclose(pitch_rt, -0.05,   abs_tol=1e-10)


# ---------------------------------------------------------------------------
# collective_out_to_rad / collective_rad_to_out
# ---------------------------------------------------------------------------

def test_collective_out_to_rad_endpoints():
    assert math.isclose(collective_out_to_rad(0.0, _COL_MIN, _COL_MAX), _COL_MIN)
    assert math.isclose(collective_out_to_rad(1.0, _COL_MIN, _COL_MAX), _COL_MAX)


def test_collective_out_to_rad_midpoint():
    mid = (_COL_MIN + _COL_MAX) / 2.0
    assert math.isclose(collective_out_to_rad(0.5, _COL_MIN, _COL_MAX), mid)


def test_collective_rad_to_out_endpoints():
    assert math.isclose(collective_rad_to_out(_COL_MIN, _COL_MIN, _COL_MAX), 0.0)
    assert math.isclose(collective_rad_to_out(_COL_MAX, _COL_MIN, _COL_MAX), 1.0)


@pytest.mark.parametrize("rad", [-0.28, -0.18, -0.079, 0.0, 0.10])
def test_collective_rad_round_trip(rad):
    out = collective_rad_to_out(rad, _COL_MIN, _COL_MAX)
    back = collective_out_to_rad(out, _COL_MIN, _COL_MAX)
    assert math.isclose(back, rad, abs_tol=1e-12)


def test_collective_out_clamped():
    assert collective_out_to_rad(-0.1, _COL_MIN, _COL_MAX) == _COL_MIN
    assert math.isclose(collective_out_to_rad(1.5, _COL_MIN, _COL_MAX), _COL_MAX)


# ---------------------------------------------------------------------------
# cyclic_to_blade_pitches
# ---------------------------------------------------------------------------

def test_cyclic_zero_tilt_returns_collective_only():
    blade_pitches = cyclic_to_blade_pitches(0.0, 0.0, 28.0, 0.0, _PITCH_GAIN, 0.1)
    np.testing.assert_allclose(blade_pitches, np.full(4, 0.1))


def test_cyclic_nonzero_tilt_modulates_blades():
    pitches = cyclic_to_blade_pitches(0.3, 0.0, 28.0, 0.0, _PITCH_GAIN, 0.0)
    assert not np.allclose(pitches, 0.0), "Non-zero tilt should produce varying blade pitches"
    assert math.isclose(np.mean(pitches), 0.0, abs_tol=1e-10), "Mean of cyclic pitches = collective"
