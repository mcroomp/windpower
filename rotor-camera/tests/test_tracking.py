import math

import cv2
import numpy as np
import pytest

from rotor_camera.tracking import MarkerTracker, Settings, marker_hue


def rotor_frame(angle: float, hue: int = 0):
    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    color = cv2.cvtColor(np.array([[[hue, 255, 255]]], dtype=np.uint8), cv2.COLOR_HSV2BGR)[0, 0]
    point = (round(100 + 70 * math.cos(angle)), round(100 - 70 * math.sin(angle)))
    cv2.circle(frame, point, 6, tuple(int(c) for c in color), -1)
    return frame


@pytest.mark.parametrize("rpm", [60, -120, 300])
def test_known_rpm(rpm):
    tracker = MarkerTracker()
    settings = Settings(center=(100, 100), hue=0)
    for index in range(120):
        timestamp = index / 120
        result = tracker.analyze(rotor_frame(timestamp * rpm * 2 * math.pi / 60), timestamp, settings)
    assert result.rpm == pytest.approx(rpm, abs=1)
    assert result.tip is not None
    assert result.azimuth_deg is not None


def test_hue_wrap_and_selection():
    frame = rotor_frame(0, hue=179)
    assert marker_hue(frame, 170, 100) == 179
    result = MarkerTracker().analyze(frame, 0, Settings(center=(100, 100), hue=0))
    assert result.tip == pytest.approx((170, 100))
    with pytest.raises(ValueError, match="saturated"):
        marker_hue(frame, 0, 0)
    with pytest.raises(ValueError, match="outside"):
        marker_hue(frame, -1, 0)


def test_lost_and_ambiguous_marker_reset_rpm():
    tracker = MarkerTracker()
    settings = Settings(center=(100, 100), hue=0)
    for i in range(5):
        result = tracker.analyze(rotor_frame(i * 0.1), i / 60, settings)
    assert result.rpm is not None
    blank = np.zeros((200, 200, 3), dtype=np.uint8)
    result = tracker.analyze(blank, 0.1, settings)
    assert result.rpm is None
    assert "lost" in result.status
    result = tracker.analyze(rotor_frame(1), 0.12, settings)
    assert result.rpm is None
    frame = rotor_frame(0)
    cv2.circle(frame, (30, 100), 6, (0, 0, 255), -1)
    result = tracker.analyze(frame, 0.14, settings)
    assert result.tip is None
    assert "Ambiguous" in result.status


@pytest.mark.parametrize("timestamp,angle", [(0, 0.1), (1, 0.1), (0.01, math.pi)])
def test_discontinuities(timestamp, angle):
    tracker = MarkerTracker()
    settings = Settings(center=(100, 100), hue=0)
    tracker.analyze(rotor_frame(0), 0, settings)
    result = tracker.analyze(rotor_frame(angle), timestamp, settings)
    assert result.rpm is None
    assert "discontinuity" in result.status


def test_configuration_change_resets_phase():
    tracker = MarkerTracker()
    settings = Settings(center=(100, 100), hue=0)
    for i in range(5):
        result = tracker.analyze(rotor_frame(i * 0.1), i / 60, settings)
    assert result.rpm is not None
    result = tracker.analyze(rotor_frame(0.5), 5 / 60, Settings(center=(99, 100), hue=0))
    assert result.rpm is None


def test_requires_selection_and_finite_timestamp():
    tracker = MarkerTracker()
    assert tracker.analyze(rotor_frame(0), 0, Settings()).tip is None
    with pytest.raises(ValueError, match="finite"):
        tracker.analyze(rotor_frame(0), float("nan"), Settings())
