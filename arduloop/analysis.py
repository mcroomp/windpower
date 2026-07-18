"""
Analysis utilities: empirical Bode estimation from time-domain runs and
step-response scoring.

These are not meant to replace `python-control` — they are dependency-light
helpers that let you get useful tuning feedback with only numpy. Where the
`control` package is installed it can be combined with these traces for
proper margin computation.
"""

from __future__ import annotations

import numpy as np


# ---------------------------------------------------------------------------
# Empirical FRF: input is a chirp / multisine, output is the measured signal.
# Uses Welch-style averaging over overlapping windows for variance reduction.
# ---------------------------------------------------------------------------

def empirical_frf(u: np.ndarray, y: np.ndarray, fs: float,
                  nperseg: int | None = None,
                  noverlap: int | None = None,
                  window: str = "hann") -> tuple[np.ndarray, np.ndarray]:
    """Estimate the frequency response Y(f)/U(f).

    Returns ``(freqs_hz, H_complex)`` where ``H`` is the cross-spectrum / auto-
    spectrum estimate (the H1 estimator — unbiased for noise on the output).
    """
    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(u)
    if nperseg is None:
        nperseg = min(n, 4096)
    if noverlap is None:
        noverlap = nperseg // 2

    if window == "hann":
        win = 0.5 - 0.5 * np.cos(2.0 * np.pi * np.arange(nperseg) / nperseg)
    else:
        win = np.ones(nperseg)
    norm = (win ** 2).sum()

    step = nperseg - noverlap
    Suu = np.zeros(nperseg // 2 + 1, dtype=complex)
    Suy = np.zeros_like(Suu)
    nseg = 0
    for start in range(0, n - nperseg + 1, step):
        seg_u = (u[start:start + nperseg] - u[start:start + nperseg].mean()) * win
        seg_y = (y[start:start + nperseg] - y[start:start + nperseg].mean()) * win
        U = np.fft.rfft(seg_u)
        Y = np.fft.rfft(seg_y)
        Suu += (U.conj() * U)
        Suy += (U.conj() * Y)
        nseg += 1
    if nseg == 0:
        raise ValueError("empirical_frf: signal too short for nperseg")
    Suu /= nseg * norm
    Suy /= nseg * norm
    H = Suy / np.where(Suu.real > 1e-20, Suu, 1.0)
    freqs = np.fft.rfftfreq(nperseg, d=1.0 / fs)
    return freqs, H


def bode_db_phase(H: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mag_db = 20.0 * np.log10(np.maximum(np.abs(H), 1e-20))
    phase_deg = np.unwrap(np.angle(H)) * 180.0 / np.pi
    return mag_db, phase_deg


# ---------------------------------------------------------------------------
# Margins from an empirical loop-gain trace
# ---------------------------------------------------------------------------

def stability_margins_from_loop(freqs: np.ndarray, L: np.ndarray
                                ) -> dict:
    """Estimate gain & phase margins from a loop transfer estimate L(jw).

    Returns a dict with `gm_db`, `pm_deg`, `wc_gain_hz` (gain crossover),
    `wc_phase_hz` (phase crossover). NaN where the crossover doesn't exist
    in the measured band.
    """
    mag_db, phase_deg = bode_db_phase(L)
    # Phase margin at first 0 dB crossover (mag going from + to −)
    wc_gain = np.nan
    pm = np.nan
    for i in range(1, len(mag_db)):
        if mag_db[i - 1] > 0.0 and mag_db[i] <= 0.0:
            # linear interp
            f = freqs[i - 1] + (freqs[i] - freqs[i - 1]) * (
                mag_db[i - 1] / (mag_db[i - 1] - mag_db[i]))
            ph = phase_deg[i - 1] + (phase_deg[i] - phase_deg[i - 1]) * (
                mag_db[i - 1] / (mag_db[i - 1] - mag_db[i]))
            wc_gain = f
            pm = 180.0 + ph
            break
    # Gain margin at first −180° crossover
    wc_phase = np.nan
    gm = np.nan
    for i in range(1, len(phase_deg)):
        if (phase_deg[i - 1] > -180.0 and phase_deg[i] <= -180.0):
            f = freqs[i - 1] + (freqs[i] - freqs[i - 1]) * (
                (phase_deg[i - 1] + 180.0) /
                (phase_deg[i - 1] - phase_deg[i]))
            m = mag_db[i - 1] + (mag_db[i] - mag_db[i - 1]) * (
                (phase_deg[i - 1] + 180.0) /
                (phase_deg[i - 1] - phase_deg[i]))
            wc_phase = f
            gm = -m
            break
    return {"gm_db": gm, "pm_deg": pm,
            "wc_gain_hz": wc_gain, "wc_phase_hz": wc_phase}


# ---------------------------------------------------------------------------
# Step-response scoring
# ---------------------------------------------------------------------------

def step_response_score(t: np.ndarray, y: np.ndarray,
                        target: float = 1.0,
                        settle_band: float = 0.05) -> dict:
    """Compute rise time (10–90%), overshoot, settling time, and a composite
    score (lower is better)."""
    y = np.asarray(y, dtype=float)
    t = np.asarray(t, dtype=float)
    final = target
    # Rise time
    try:
        i10 = np.argmax(y >= 0.1 * final)
        i90 = np.argmax(y >= 0.9 * final)
        rise = t[i90] - t[i10] if i90 > i10 else np.nan
    except Exception:
        rise = np.nan
    peak = y.max() if final >= 0 else y.min()
    overshoot = (peak - final) / abs(final) if final != 0 else np.nan
    # Settling time
    band = settle_band * abs(final)
    settled = np.where(np.abs(y - final) > band)[0]
    settle_t = t[settled[-1]] if len(settled) else 0.0
    iae = np.trapezoid(np.abs(y - final), t)
    score = iae + 2.0 * max(overshoot, 0.0) + 0.5 * settle_t
    return {"rise_time_s": rise,
            "overshoot": overshoot,
            "settle_time_s": settle_t,
            "iae": iae,
            "score": score}
