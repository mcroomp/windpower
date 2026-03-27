import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from swashplate import (
    collective_to_pitch,
    cyclic_to_blade_pitches,
    h3_inverse_mix,
    pwm_to_normalized,
    servos_to_blade_pitches,
)


def test_pwm_to_normalized_maps_expected_range():
    assert pwm_to_normalized(1000.0) == -1.0
    assert pwm_to_normalized(1500.0) == 0.0
    assert pwm_to_normalized(2000.0) == 1.0
    assert pwm_to_normalized(1250.0) == -0.5


def test_h3_inverse_mix_detects_pure_collective():
    collective, tilt_lon, tilt_lat = h3_inverse_mix(1.0, 1.0, 1.0)
    assert collective == 1.0
    assert math.isclose(tilt_lon, 0.0, abs_tol=1e-12)
    assert math.isclose(tilt_lat, 0.0, abs_tol=1e-12)


def test_collective_to_pitch_clamps_to_expected_limits():
    assert collective_to_pitch(0.0) == 0.0
    assert math.isclose(collective_to_pitch(1.0), 0.35)
    assert math.isclose(collective_to_pitch(-1.0), -0.35)


def test_cyclic_to_blade_pitches_zero_tilt_returns_collective_only():
    blade_pitches = cyclic_to_blade_pitches(0.0, 0.0, 28.0, 0.0, 0.1)
    np.testing.assert_allclose(blade_pitches, np.full(4, 0.1))


def test_servos_to_blade_pitches_pipeline_preserves_zero_cyclic_for_equal_servos():
    blade_pitches, collective_rad, tilt_lon, tilt_lat = servos_to_blade_pitches(
        np.array([1750.0, 1750.0, 1750.0]),
        omega=28.0,
        t=0.0,
    )

    assert math.isclose(collective_rad, 0.175)
    assert math.isclose(tilt_lon, 0.0, abs_tol=1e-12)
    assert math.isclose(tilt_lat, 0.0, abs_tol=1e-12)
    np.testing.assert_allclose(blade_pitches, np.full(4, collective_rad))