"""
Filter primitives matching ArduPilot's `LowPassFilter` and `NotchFilter`.

These are intentionally minimal and stateful: one sample in, one sample out,
identical to how the C++ code is called inside `AC_PID::update_all`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# First-order low-pass — mirrors AP_Math/LowPassFilter.cpp
# ---------------------------------------------------------------------------

class LowPassFilter1p:
    """First-order discrete low-pass with run-time-variable dt.

    Implementation matches AP's `apply` exactly:

        alpha = dt / (dt + RC),   RC = 1 / (2*pi*cutoff_hz)
        y    += alpha * (x - y)

    A cutoff of 0 (or negative) disables filtering (`y = x`).
    """

    def __init__(self, cutoff_hz: float = 0.0):
        self.cutoff_hz = float(cutoff_hz)
        self._y = 0.0
        self._initialised = False

    def set_cutoff(self, cutoff_hz: float) -> None:
        self.cutoff_hz = float(cutoff_hz)

    def reset(self, value: float = 0.0) -> None:
        self._y = float(value)
        self._initialised = True

    def apply(self, x: float, dt: float) -> float:
        if not self._initialised:
            self._y = x
            self._initialised = True
            return self._y
        if self.cutoff_hz <= 0.0 or dt <= 0.0:
            self._y = x
            return self._y
        rc = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = dt / (dt + rc)
        self._y += alpha * (x - self._y)
        return self._y


# ---------------------------------------------------------------------------
# Biquad notch — mirrors Filter/NotchFilter.cpp (direct-form II transposed)
# ---------------------------------------------------------------------------

@dataclass
class _BiquadCoeffs:
    b0: float
    b1: float
    b2: float
    a1: float
    a2: float


def _notch_coeffs(center_hz: float, bandwidth_hz: float, sample_hz: float,
                  attenuation_db: float = 40.0) -> _BiquadCoeffs:
    """Compute biquad notch coefficients.

    Uses the same form as AP's `NotchFilter::init` (with attenuation control).
    """
    if center_hz <= 0.0 or bandwidth_hz <= 0.0 or sample_hz <= 0.0:
        # Pass-through
        return _BiquadCoeffs(1.0, 0.0, 0.0, 0.0, 0.0)
    # Limit centre below Nyquist
    center_hz = min(center_hz, 0.45 * sample_hz)
    omega = 2.0 * math.pi * center_hz / sample_hz
    octaves = bandwidth_hz / center_hz   # fractional bandwidth
    # Convert attenuation dB to amplitude factor A (≥1 → deeper notch)
    A = 10.0 ** (attenuation_db / 40.0)
    Q = center_hz / max(bandwidth_hz, 1e-9)
    alpha = math.sin(omega) / (2.0 * Q)
    cos_w = math.cos(omega)
    a0 = 1.0 + alpha / A
    return _BiquadCoeffs(
        b0=(1.0 + alpha * A) / a0,
        b1=(-2.0 * cos_w) / a0,
        b2=(1.0 - alpha * A) / a0,
        a1=(-2.0 * cos_w) / a0,
        a2=(1.0 - alpha / A) / a0,
    )


class NotchFilter:
    """Biquad notch with online retuning.

    Configure once with `set` (center, bandwidth, sample rate). Call `apply`
    once per sample. `reset(value)` primes the delay line so step changes do
    not cause a transient.
    """

    def __init__(self, center_hz: float = 0.0, bandwidth_hz: float = 0.0,
                 sample_hz: float = 0.0, attenuation_db: float = 40.0):
        self._c = _BiquadCoeffs(1.0, 0.0, 0.0, 0.0, 0.0)
        self._z1 = 0.0
        self._z2 = 0.0
        self.enabled = False
        self.center_hz = center_hz
        self.bandwidth_hz = bandwidth_hz
        self.sample_hz = sample_hz
        self.attenuation_db = attenuation_db
        if center_hz > 0 and bandwidth_hz > 0 and sample_hz > 0:
            self.set(center_hz, bandwidth_hz, sample_hz, attenuation_db)

    def set(self, center_hz: float, bandwidth_hz: float, sample_hz: float,
            attenuation_db: float | None = None) -> None:
        self.center_hz = center_hz
        self.bandwidth_hz = bandwidth_hz
        self.sample_hz = sample_hz
        if attenuation_db is not None:
            self.attenuation_db = attenuation_db
        self._c = _notch_coeffs(center_hz, bandwidth_hz, sample_hz, self.attenuation_db)
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def reset(self, value: float = 0.0) -> None:
        # Prime the transposed direct-form II delay line so steady-state DC
        # passes through cleanly.
        self._z1 = value * (self._c.b1 - self._c.a1)
        self._z2 = value * (self._c.b2 - self._c.a2)

    def apply(self, x: float) -> float:
        if not self.enabled:
            return x
        c = self._c
        y = c.b0 * x + self._z1
        self._z1 = c.b1 * x - c.a1 * y + self._z2
        self._z2 = c.b2 * x - c.a2 * y
        return y
