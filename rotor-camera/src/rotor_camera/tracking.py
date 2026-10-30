"""Track a unique saturated colour marker in image coordinates."""

from collections import deque
from dataclasses import dataclass
import math

import cv2
import numpy as np
from numpy.typing import NDArray

type Frame = NDArray[np.uint8]


@dataclass(frozen=True)
class Settings:
    center: tuple[float, float] | None = None
    hue: int | None = None
    min_area: float = 12.0
    hue_tolerance: int = 12


@dataclass(frozen=True)
class Measurement:
    timestamp: float
    tip: tuple[float, float] | None = None
    azimuth_deg: float | None = None
    rpm: float | None = None
    status: str = "Select the rotor centre and marker."


def marker_hue(frame: Frame, x: int, y: int) -> int:
    if not (0 <= y < frame.shape[0] and 0 <= x < frame.shape[1]):
        raise ValueError("Marker selection is outside the image.")
    hue, saturation, value = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[y, x]
    if saturation < 60 or value < 60:
        raise ValueError("Choose a bright, saturated colour marker, not white/grey/black.")
    return int(hue)


class MarkerTracker:
    def __init__(self) -> None:
        self._settings = Settings()
        self._phase: deque[tuple[float, float]] = deque()
        self._last_angle: float | None = None
        self._last_time: float | None = None
        self._unwrapped = 0.0

    def _reset_phase(self) -> None:
        self._phase.clear()
        self._last_angle = None
        self._last_time = None
        self._unwrapped = 0.0

    def analyze(self, frame: Frame, timestamp: float, settings: Settings) -> Measurement:
        if not math.isfinite(timestamp):
            raise ValueError("Frame timestamp must be finite.")
        if settings != self._settings:
            self._reset_phase()
            self._settings = settings
        if settings.center is None or settings.hue is None:
            self._reset_phase()
            return Measurement(timestamp)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0].astype(np.int16)
        distance = np.abs(hue - settings.hue)
        mask = (
            (np.minimum(distance, 180 - distance) <= settings.hue_tolerance)
            & (hsv[:, :, 1] >= 60)
            & (hsv[:, :, 2] >= 60)
        ).astype(np.uint8) * 255
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = sorted(
            (c for c in contours if cv2.contourArea(c) >= settings.min_area),
            key=cv2.contourArea,
            reverse=True,
        )
        if not candidates:
            self._reset_phase()
            return Measurement(timestamp, status="Marker lost; RPM reset.")
        if len(candidates) > 1 and cv2.contourArea(candidates[1]) > 0.6 * cv2.contourArea(candidates[0]):
            self._reset_phase()
            return Measurement(timestamp, status="Ambiguous colour blobs; isolate one unique marker.")
        moments = cv2.moments(candidates[0])
        tip = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        dx, dy = tip[0] - settings.center[0], tip[1] - settings.center[1]
        if math.hypot(dx, dy) < 5:
            self._reset_phase()
            return Measurement(timestamp, tip=tip, status="Marker too close to selected centre.")
        angle = math.atan2(-dy, dx)
        rpm = None
        status = "Tracking; collecting phase samples."
        if self._last_angle is not None and self._last_time is not None:
            dt = timestamp - self._last_time
            delta = math.atan2(math.sin(angle - self._last_angle), math.cos(angle - self._last_angle))
            if dt <= 0 or dt > 0.5 or abs(delta) >= math.radians(120):
                self._reset_phase()
                status = "Timing/phase discontinuity; RPM reset. Check frame rate and marker."
            else:
                self._unwrapped += delta
        self._last_angle = angle
        self._last_time = timestamp
        self._phase.append((timestamp, self._unwrapped))
        while len(self._phase) > 2 and timestamp - self._phase[0][0] > 0.3:
            self._phase.popleft()
        if len(self._phase) >= 3:
            times, angles = np.array(self._phase).T
            times = times - times.mean()
            denominator = float(times @ times)
            if denominator > 0:
                rpm = float(times @ (angles - angles.mean()) / denominator * 60 / (2 * math.pi))
                status = "Tracking (image-plane phase; aliasing remains possible)."
        return Measurement(timestamp, tip, math.degrees(angle) % 360, rpm, status)
