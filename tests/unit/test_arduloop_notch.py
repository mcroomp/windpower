import math

import numpy as np
import pytest

from arduloop.filters import NotchFilter


def test_notch_attenuates_center_frequency():
    sample_rate = 400
    frequency = 8.3
    filt = NotchFilter(frequency, 4.0, sample_rate, 30.0)
    samples = np.asarray([
        filt.apply(math.sin(math.tau * frequency * i / sample_rate))
        for i in range(sample_rate * 10)
    ])
    amplitude = math.sqrt(2 * float(np.mean(samples[-sample_rate * 5:] ** 2)))
    assert amplitude == pytest.approx(10 ** (-30 / 20), rel=0.02)


def test_notch_reset_preserves_constant_input_and_invalid_frequency_bypasses():
    filt = NotchFilter(8.3, 4.0, 400, 30.0)
    filt.reset(0.7)
    assert [filt.apply(0.7) for _ in range(10)] == pytest.approx([0.7] * 10)
    filt.set(201, 4.0, 400, 30.0)
    assert not filt.enabled
    assert filt.apply(0.7) == 0.7
