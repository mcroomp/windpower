"""
Reference / probe signal generators.

These mirror what `probe_open_loop_plant` and `probe_step_response` would
inject on a real rig, so the time series collected from this simulator are
directly comparable to logs collected from ArduPilot SITL or hardware.
"""

from __future__ import annotations

import numpy as np


def step(t: np.ndarray, amplitude: float = 1.0, t0: float = 0.5) -> np.ndarray:
    return np.where(t >= t0, amplitude, 0.0)


def logarithmic_chirp(t: np.ndarray, f0: float, f1: float,
                      amplitude: float = 1.0) -> np.ndarray:
    """Log-frequency sweep from f0 to f1 over t[0]..t[-1]."""
    T = t[-1] - t[0]
    if T <= 0 or f0 <= 0 or f1 <= 0:
        return np.zeros_like(t)
    k = (f1 / f0) ** (1.0 / T)
    phase = 2.0 * np.pi * f0 * ((k ** (t - t[0]) - 1.0) / np.log(k))
    return amplitude * np.sin(phase)


def multisine(t: np.ndarray, freqs_hz, amplitude: float = 1.0,
              seed: int | None = 0) -> np.ndarray:
    """Sum of equal-amplitude sinusoids with randomised phases.

    Good for system-identification because the crest factor stays low and
    every frequency in `freqs_hz` is excited simultaneously.
    """
    rng = np.random.default_rng(seed)
    out = np.zeros_like(t)
    n = len(freqs_hz)
    for f in freqs_hz:
        phi = rng.uniform(0.0, 2.0 * np.pi)
        out += np.sin(2.0 * np.pi * f * t + phi)
    return amplitude * out / max(n, 1)


def doublet(t: np.ndarray, amplitude: float = 1.0,
            t0: float = 0.5, width: float = 0.2) -> np.ndarray:
    s = np.zeros_like(t)
    s[(t >= t0) & (t < t0 + width)] = amplitude
    s[(t >= t0 + width) & (t < t0 + 2 * width)] = -amplitude
    return s
