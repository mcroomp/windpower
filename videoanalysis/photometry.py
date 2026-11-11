"""Spatial illumination compensation without temporal filtering of motion."""

import cv2
import numpy as np

SIGMA_PX = 5.0
STD_FLOOR = 5.0
OUTPUT_MEAN = 128.0
OUTPUT_SCALE = 32.0
TEMPLATE_SIGMA_PX = 1.0

NORMALIZATION_SETTINGS = {
    "mode": "local_mean_std",
    "sigma_px": SIGMA_PX,
    "std_floor": STD_FLOOR,
    "output_mean": OUTPUT_MEAN,
    "output_scale": OUTPUT_SCALE,
    "template_smoothing_sigma_px": TEMPLATE_SIGMA_PX,
    "temporal_filtering": False,
    "display": "normalized_grayscale",
}


def validate_grayscale(gray: cv2.typing.MatLike) -> None:
    if gray.ndim != 2 or gray.dtype != np.uint8 or gray.size == 0:
        raise ValueError("Illumination normalization requires a nonempty 8-bit grayscale image")


def normalize_illumination(gray: cv2.typing.MatLike) -> cv2.typing.MatLike:
    validate_grayscale(gray)
    data = gray.astype(np.float32)
    mean = cv2.GaussianBlur(data, (0, 0), SIGMA_PX)
    variance = cv2.GaussianBlur(data * data, (0, 0), SIGMA_PX) - mean * mean
    scale = np.sqrt(np.maximum(variance, STD_FLOOR**2))
    normalized = OUTPUT_MEAN + OUTPUT_SCALE * (data - mean) / scale
    return np.rint(np.clip(normalized, 0, 255)).astype(np.uint8)


def brightness_diagnostics(
    raw: list[float], tracking: list[float], times: list[float],
) -> dict:
    if len(times) < 16:
        return {"status": "too_short"}
    deltas = np.diff(times)
    period = float(np.median(deltas))
    if period <= 0 or not np.allclose(deltas, period, rtol=1e-3, atol=1e-6):
        return {"status": "irregular_sampling"}
    if np.ptp(raw) < 1e-6:
        return {"status": "flat_reference"}
    window = np.hanning(len(times))
    raw_signal = np.asarray(raw) - np.mean(raw)
    tracking_signal = np.asarray(tracking) - np.mean(tracking)
    raw_power = np.abs(np.fft.rfft(raw_signal * window)) ** 2
    tracking_power = np.abs(np.fft.rfft(tracking_signal * window)) ** 2
    frequency = np.fft.rfftfreq(len(times), period)
    candidates = np.flatnonzero(frequency >= 1.0)
    if not len(candidates):
        return {"status": "insufficient_sample_rate"}
    peak = int(candidates[np.argmax(raw_power[candidates])])
    band = (frequency >= frequency[peak] - 0.5) & (frequency <= frequency[peak] + 0.5)
    before = float(np.sqrt(raw_power[band].sum()))
    after = float(np.sqrt(tracking_power[band].sum()))
    if before < 1e-6:
        return {"status": "no_fast_variation"}
    return {
        "status": "ok",
        "observed_peak_hz": float(frequency[peak]),
        "band_half_width_hz": 0.5,
        "tracking_brightness_amplitude_reduction_percent": 100 * (1 - after / before),
    }
