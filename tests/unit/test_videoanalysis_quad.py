import csv
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

cv2 = pytest.importorskip("cv2", reason="Requires the videoanalysis extra")
import numpy as np  # noqa: E402 - skip before importing optional dependencies

from videoanalysis.__main__ import annotate, analyze, load_regions, select_quadrilateral  # noqa: E402 - skip before importing optional dependencies
from videoanalysis.geometry import Quad, edge_lengths, quad_center, shape_measurements, to_quad, valid_quad  # noqa: E402 - skip before importing optional dependencies
from videoanalysis.photometry import normalize_illumination  # noqa: E402 - skip before importing optional dependencies
from videoanalysis.tracking import Match, PointFlow, QuadTracker, Region  # noqa: E402 - skip before importing optional dependencies


REFERENCE = ((70.0, 40.0), (140.0, 40.0), (140.0, 110.0), (70.0, 110.0))


def marker_frame():
    noise = np.random.default_rng(42).integers(90, 190, (160, 240), dtype=np.uint8)
    image = cv2.GaussianBlur(noise, (3, 3), 0.6)
    cv2.fillConvexPoly(image, np.asarray(REFERENCE, dtype=np.int32), 30)
    return image


def test_geometry_measures_distortion_not_bounding_box():
    current = ((75.0, 42.0), (135.0, 45.0), (146.0, 105.0), (66.0, 108.0))
    values = shape_measurements(current, REFERENCE, (2, 1))
    assert values["relative_corner_0_dx_px"] == 3
    assert values["relative_corner_0_dy_px"] == 1
    assert values["edge_0_px"] == pytest.approx(np.hypot(60, 3))
    assert values["edge_2_px"] == pytest.approx(np.hypot(80, 3))
    assert values["edge_0_ratio"] != pytest.approx(values["edge_2_ratio"])
    assert values["angle_0_delta_deg"] != pytest.approx(0)
    assert values["area_ratio"] == pytest.approx(values["area_px2"] / 4900)
    assert np.linalg.norm(quad_center(current) - np.mean(current, axis=0)) > 1
    without_hub = shape_measurements(current, REFERENCE, None)
    assert without_hub["area_px2"] == values["area_px2"]
    assert "relative_corner_0_dx_px" not in without_hub


@pytest.mark.parametrize("corners", [
    ((70, 40), (140, 110), (140, 40), (70, 110)),
    ((70, 40), (70, 110), (140, 110), (140, 40)),
    ((70, 40), (140, 40), (100, 50), (70, 110)),
    ((70, 40), (70, 40), (140, 110), (70, 110)),
    ((-1, 40), (140, 40), (140, 110), (70, 110)),
    ((70, 40), (140, 40), (float("nan"), 110), (70, 110)),
])
def test_invalid_quadrilaterals(corners: Quad):
    assert not valid_quad(corners, 240, 160)


def test_corner_tracking_recovers_true_projective_shape_under_lighting_changes():
    first = marker_frame()
    tracker = QuadTracker(first, Region.from_corners("tape", REFERENCE))
    initial = tracker.update(first)
    assert initial.corners == REFERENCE
    assert initial.corners is not None
    match = initial
    actual = np.asarray(initial.corners)
    for i in range(1, 9):
        expected = np.asarray(REFERENCE, dtype=np.float32) + i * np.asarray(
            [[0.2, 0.1], [-0.4, 0.15], [0.45, -0.35], [-0.1, -0.15]], dtype=np.float32,
        )
        homography = cv2.getPerspectiveTransform(np.asarray(REFERENCE, dtype=np.float32), expected)
        moved = cv2.warpPerspective(first, homography, (240, 160))
        lighting = np.linspace(0, i, 240)[None, :]
        moved = np.clip(moved.astype(float) * 0.9 + 12 + lighting, 0, 255).astype(np.uint8)
        match = tracker.update(moved)
        assert match.valid, (i, match)
        assert match.method == "corner_flow"
        assert match.tracked_points == 4
        assert match.inlier_ratio is None
        actual = np.asarray(match.corners)
        assert np.max(np.abs(actual - expected)) < 0.75
    assert np.linalg.norm(actual[0] + actual[2] - actual[1] - actual[3]) > 6
    assert match.corners is not None
    assert edge_lengths(match.corners)[0] != pytest.approx(edge_lengths(match.corners)[2], abs=1)


def test_missing_corner_never_becomes_a_translated_square(
    monkeypatch: pytest.MonkeyPatch,
):
    first = marker_frame()
    tracker = QuadTracker(first, Region.from_corners("tape", REFERENCE))
    assert tracker.update(first).valid
    flow = PointFlow(np.asarray(REFERENCE), np.asarray([True, True, False, True]), np.zeros(4))
    monkeypatch.setattr("videoanalysis.tracking.track_points", lambda *_args: flow)
    lost = tracker.update(first)
    assert lost.score > 0.99
    assert not lost.valid
    assert lost.corners is None and lost.dx is None


def test_foreshortened_marker_keeps_independent_corners():
    reference = ((90.0, 40.0), (110.0, 36.0), (107.0, 111.0), (85.0, 108.0))
    first = np.random.default_rng(8).integers(120, 190, (160, 240), dtype=np.uint8)
    first = cv2.GaussianBlur(first, (3, 3), 0.6)
    cv2.fillConvexPoly(first, np.asarray(reference, dtype=np.int32), 30)
    tracker = QuadTracker(first, Region.from_corners("tape", reference))
    assert tracker.update(first).valid
    for i in range(1, 9):
        expected = np.asarray(reference, dtype=np.float32) + i * np.asarray(
            [[0.1, 0.15], [0.0, 0.1], [0.15, -0.15], [-0.05, -0.1]], dtype=np.float32,
        )
        warp = cv2.getPerspectiveTransform(np.asarray(reference, dtype=np.float32), expected)
        match = tracker.update(cv2.warpPerspective(first, warp, (240, 160)))
        assert match.valid, match
        assert np.max(np.abs(np.asarray(match.corners) - expected)) < 0.75


def test_corner_loss_and_verified_reacquisition():
    first = marker_frame()
    tracker = QuadTracker(first, Region.from_corners("tape", REFERENCE))
    assert tracker.update(first).valid
    lost = tracker.update(np.full(first.shape, 127, dtype=np.uint8))
    assert not lost.valid and lost.corners is None
    expected = np.asarray(REFERENCE, dtype=np.float32) + [[1, 0], [-1, 1], [2, -1], [0, -1]]
    homography = cv2.getPerspectiveTransform(np.asarray(REFERENCE, dtype=np.float32), expected.astype(np.float32))
    recovered = tracker.update(cv2.warpPerspective(first, homography, (240, 160)))
    assert recovered.valid
    assert recovered.method == "corner_reacquire"
    assert np.max(np.abs(np.asarray(recovered.corners) - expected)) < 0.75


def test_overlay_uses_measured_corners(monkeypatch: pytest.MonkeyPatch):
    corners = ((75.0, 42.0), (135.0, 45.0), (146.0, 105.0), (66.0, 108.0))
    region = Region.from_corners("tape", REFERENCE)
    polygon = Mock(wraps=cv2.polylines)
    monkeypatch.setattr(cv2, "polylines", polygon)
    match = Match("ok", 1, 1, 0, 0, corners=corners)
    annotate(cv2.cvtColor(marker_frame(), cv2.COLOR_GRAY2BGR), [region], [match], 0, 0)
    assert np.array_equal(polygon.call_args.args[1][0], np.asarray(corners, dtype=np.int32))


def test_zoomed_corner_selection_preserves_order_and_subpixel_coordinates(
    monkeypatch: pytest.MonkeyPatch,
):
    callbacks = []
    monkeypatch.setattr(cv2, "namedWindow", lambda *_args: None)
    monkeypatch.setattr(cv2, "setMouseCallback", lambda _title, callback: callbacks.append(callback))
    monkeypatch.setattr(cv2, "imshow", lambda *_args: None)
    monkeypatch.setattr(cv2, "getWindowProperty", lambda *_args: 1)
    monkeypatch.setattr(cv2, "destroyWindow", lambda *_args: None)

    def enter(_delay: int):
        for x, y in ((81, 80), (640, 80), (600, 560), (120, 520)):
            callbacks[0](cv2.EVENT_LBUTTONDOWN, x, y, 0, None)
        return 13

    monkeypatch.setattr(cv2, "waitKey", enter)
    region = select_quadrilateral(
        np.zeros((200, 300, 3), dtype=np.uint8), Region("", (100, 50, 100, 80)), "tape",
    )
    assert region.corners == ((110.125, 60), (180, 60), (175, 120), (115, 115))


def test_blade_rectangles_are_rejected(tmp_path: Path):
    config = tmp_path / "regions.json"
    config.write_text(json.dumps({
        "frame_size": [240, 160], "start_frame": 0,
        "regions": [{"name": "hub", "rect": [10, 10, 30, 30]}, {"name": "tape", "rect": [70, 40, 70, 70]}],
    }))
    with pytest.raises(ValueError, match="four .*corners"):
        load_regions(config, marker_frame(), 0)


@pytest.mark.parametrize("compensate_flicker", [False, True])
def test_projective_video_exports_corner_distortion_and_blank_lost_geometry(
    tmp_path: Path,
    compensate_flicker: bool,
):
    first = marker_frame()
    # A separate, static textured hub patch supplies the translation reference.
    region = Region.from_corners("tape", REFERENCE)
    video = tmp_path / "projective.mp4"
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter.fourcc(*"mp4v"), 30, (240, 160))
    assert writer.isOpened()
    expected_shapes = []
    try:
        for i in range(6):
            expected = np.asarray(REFERENCE, dtype=np.float32) + i * np.asarray(
                [[0.2, 0.1], [-0.4, 0.15], [0.45, -0.35], [-0.1, -0.15]], dtype=np.float32,
            )
            warp = cv2.getPerspectiveTransform(np.asarray(REFERENCE, dtype=np.float32), expected)
            frame = cv2.warpPerspective(first, warp, (240, 160))
            frame[10:35, 10:35] = first[10:35, 10:35]
            if i == 4:
                frame[30:120, 60:150] = 127
            writer.write(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR))
            expected_shapes.append(to_quad(expected))
    finally:
        writer.release()
    config = tmp_path / "regions.json"
    config.write_text(json.dumps({
        "frame_size": [240, 160], "start_frame": 0,
        "regions": [{"name": "hub", "rect": [10, 10, 25, 25]}, {"name": "tape", "corners": region.corners}],
    }))
    output = tmp_path / "results"
    summary = analyze(video, output, config, compensate_flicker=compensate_flicker)
    with (output / "displacement.csv").open(newline="") as file:
        rows = [r for r in csv.DictReader(file) if r["region"] == "tape"]
    for i, row in enumerate(rows):
        if i == 4:
            assert row["shape_status"] != "ok"
            for key in ("corner_0_x_px", "area_px2", "edge_0_ratio", "angle_0_deg"):
                assert row[key] == ""
        else:
            assert row["shape_status"] == "ok", row
            for corner in range(4):
                assert float(row[f"corner_{corner}_x_px"]) == pytest.approx(expected_shapes[i][corner][0], abs=0.75)
            expected_area = cv2.contourArea(np.asarray(expected_shapes[i], dtype=np.float32))
            assert float(row["area_px2"]) == pytest.approx(expected_area, rel=0.03)
    assert summary["shape_valid_frames"]["tape"] == 5
    assert summary["photometric_normalization"]["mode"] == ("local_mean_std" if compensate_flicker else "none")


def test_periodic_exposure_compensation_preserves_real_motion():
    first = marker_frame()
    time = np.arange(60) / 30
    frames = []
    expected_shapes = []
    for t in time:
        dx, dy = 2 * np.sin(2 * np.pi * 3 * t), np.sin(2 * np.pi * 2 * t)
        transform = np.asarray([[1, 0, dx], [0, 1, dy]], dtype=np.float32)
        moved = cv2.warpAffine(first, transform, (240, 160))
        gain = 1 + 0.25 * np.sin(2 * np.pi * 12.5 * t)
        offset = 12 * np.cos(2 * np.pi * 12.5 * t)
        exposed = np.clip(moved.astype(float) * gain + offset, 0, 255).astype(np.uint8)
        frames.append(normalize_illumination(exposed))
        expected_shapes.append(np.asarray(REFERENCE) + [dx, dy])
    tracker = QuadTracker(frames[0], Region.from_corners("tape", REFERENCE), normalized_input=True)
    measurements = []
    for i, frame in enumerate(frames):
        match = tracker.update(frame)
        if match.valid:
            assert np.max(np.abs(np.asarray(match.corners) - expected_shapes[i])) < 0.75
            measurements.append((time[i], match.dx))
    assert len(measurements) >= 0.95 * len(frames)
    measured = np.asarray(measurements)
    basis = np.column_stack((
        np.sin(2 * np.pi * 3 * measured[:, 0]),
        np.cos(2 * np.pi * 3 * measured[:, 0]),
        np.ones(len(measured)),
    ))
    coefficients = np.linalg.lstsq(basis, measured[:, 1], rcond=None)[0]
    assert np.hypot(coefficients[0], coefficients[1]) == pytest.approx(2, abs=0.2)
