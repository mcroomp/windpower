import csv
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

cv2 = pytest.importorskip("cv2", reason="Requires the videoanalysis extra")
import numpy as np  # noqa: E402 - skip before importing optional dependencies

from videoanalysis.__main__ import CSV_FIELDS, analyze, load_regions, main, select_regions, verify_export  # noqa: E402 - skip before importing optional dependencies
from videoanalysis.tracking import Match, PatchTracker, Region, TemplateTracker, relative_displacement  # noqa: E402 - skip before importing optional dependencies


def textured_frame(seed: int = 0):
    return np.random.default_rng(seed).integers(0, 256, (160, 240), dtype=np.uint8)


def test_tracker_displacement_with_changing_background():
    first = textured_frame()
    region = Region("blade", (80, 60, 32, 24))
    template = first[60:84, 80:112].copy()
    tracker = TemplateTracker(first, region, search_radius=15)
    for dx, dy in [(3, -2), (7, -4), (12, -6)]:
        frame = textured_frame(dx)
        frame[60 + dy:84 + dy, 80 + dx:112 + dx] = template
        match = tracker.update(frame)
        assert match.valid
        assert (match.dx, match.dy) == pytest.approx((dx, dy), abs=0.15)
        assert match.score > 0.99


def test_tracker_reports_loss_and_reacquires_without_integrating_drift():
    first = textured_frame()
    region = Region("blade", (80, 60, 32, 24))
    tracker = TemplateTracker(first, region, search_radius=15)
    lost = tracker.update(textured_frame(4))
    assert lost.status == "low_score"
    assert lost.dx is None and lost.dy is None
    recovered = tracker.update(first)
    assert recovered.valid
    assert (recovered.dx, recovered.dy) == pytest.approx((0, 0), abs=0.15)


def test_subpixel_motion():
    first = cv2.GaussianBlur(textured_frame(), (5, 5), 1)
    transform = np.asarray([[1, 0, 2.4], [0, 1, -1.6]], dtype=np.float32)
    moved = cv2.warpAffine(first, transform, (240, 160))
    tracker = TemplateTracker(first, Region("blade", (80, 60, 32, 24)))
    match = tracker.update(moved)
    assert match.valid
    assert (match.dx, match.dy) == pytest.approx((2.4, -1.6), abs=0.25)


def test_flow_tracks_shape_and_local_illumination_changes():
    first = cv2.GaussianBlur(textured_frame(), (3, 3), 0.6)
    region = Region("blade", (60, 40, 100, 80))
    tracker = PatchTracker(first, region, search_radius=30)
    assert tracker.update(first).valid
    methods = []
    center = np.asarray([110, 80, 1], dtype=np.float32)
    for i in range(1, 13):
        transform = np.asarray([[1, 0.005 * i, i], [0, 1 - 0.012 * i, i / 2]], dtype=np.float32)
        moved = cv2.warpAffine(first, transform, (240, 160))
        illumination = np.linspace(0, i * 2, 240)[None, :]
        moved = np.clip(moved.astype(float) * 0.85 + illumination + 15, 0, 255).astype(np.uint8)
        match = tracker.update(moved)
        assert match.valid, (i, match)
        expected = transform @ center - center[:2]
        assert (match.dx, match.dy) == pytest.approx(expected, abs=1.0)
        methods.append(match.method)
        if match.method == "flow":
            assert match.tracked_points >= 6
            assert match.inlier_ratio is not None
            assert match.fb_error_px is not None
            assert match.inlier_ratio >= 0.7
            assert match.fb_error_px <= 1
    assert "flow" in methods


def test_ambiguous_patch_is_rejected():
    frame = textured_frame()
    frame[60:84, 120:144] = frame[60:84, 80:104]
    tracker = TemplateTracker(frame, Region("blade", (80, 60, 24, 24)), search_radius=50)
    match = tracker.update(frame)
    assert match.status == "ambiguous"
    assert match.dx is None


def test_search_boundary_is_not_accepted_as_valid_motion():
    first = textured_frame()
    frame = textured_frame(4)
    frame[60:84, 90:122] = first[60:84, 80:112]
    tracker = TemplateTracker(first, Region("blade", (80, 60, 32, 24)), search_radius=10)
    assert tracker.update(frame).status == "search_boundary"


def test_frame_edge_can_be_a_valid_match():
    frame = textured_frame()
    tracker = TemplateTracker(frame, Region("hub", (0, 0, 24, 24)))
    assert tracker.update(frame).valid


@pytest.mark.parametrize("rect", [(-1, 0, 10, 10), (0, 0, 4, 8), (230, 0, 20, 20)])
def test_invalid_regions(rect: tuple[int, int, int, int]):
    with pytest.raises(ValueError, match="inside the frame"):
        TemplateTracker(textured_frame(), Region("blade", rect))


def test_texture_and_frame_size_validation():
    with pytest.raises(ValueError, match="insufficient texture"):
        TemplateTracker(np.zeros((40, 40), dtype=np.uint8), Region("hub", (0, 0, 20, 20)))
    tracker = TemplateTracker(textured_frame(), Region("hub", (0, 0, 20, 20)))
    with pytest.raises(ValueError, match="dimensions or pixel type changed"):
        tracker.update(np.zeros((40, 40), dtype=np.uint8))


@pytest.mark.parametrize("score", [0, 1.1, float("nan")])
def test_invalid_score(score: float):
    with pytest.raises(ValueError, match="Minimum score"):
        TemplateTracker(textured_frame(), Region("hub", (0, 0, 20, 20)), min_score=score)


def test_relative_displacement_requires_both_matches():
    blade = Match("ok", 1, 1, 12, 4)
    hub = Match("ok", 1, 1, 3, 5)
    assert relative_displacement(blade, hub) == (9, -1)
    assert relative_displacement(blade, Match("low_score", 0.2, 0.1)) == (None, None)
    assert relative_displacement(Match("ambiguous", 0.9, 0), hub) == (None, None)


@pytest.fixture
def video_case(tmp_path: Path) -> tuple[Path, Path]:
    video = tmp_path / "source.mp4"
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter.fourcc(*"mp4v"), 30, (240, 160))
    assert writer.isOpened()
    first = textured_frame()
    hub = first[90:114, 40:64].copy()
    cv2.rectangle(first, (135, 40), (154, 59), 25, -1)
    blade = first[25:75, 120:170].copy()
    try:
        for i in range(7):
            frame = textured_frame(i + 10)
            frame[90 + i:114 + i, 40 + i:64 + i] = hub
            frame[25:75, 120 + 3 * i:170 + 3 * i] = blade
            if i == 4:
                frame[90 + i:114 + i, 40 + i:64 + i] = 127
            writer.write(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR))
    finally:
        writer.release()
    config = tmp_path / "regions.json"
    config.write_text(json.dumps({
        "frame_size": [240, 160], "start_frame": 0,
        "regions": [
            {"name": "hub", "rect": [40, 90, 24, 24]},
            {"name": "blade_1", "corners": [[135, 40], [154, 40], [154, 59], [135, 59]]},
        ],
    }), encoding="utf-8")
    return video, config


def test_video_csv_and_export(video_case: tuple[Path, Path], tmp_path: Path):
    video, config = video_case
    output = tmp_path / "results"
    summary = analyze(video, output, config, search_radius=12)
    assert summary["frames_processed"] == 7
    assert summary["tracking_status_counts"]["hub"]["insufficient_flow"] == 1
    assert summary["relative_valid_frames"]["blade_1"] == 6
    with (output / "displacement.csv").open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        assert reader.fieldnames == CSV_FIELDS
        rows = list(reader)
    assert len(rows) == 14
    blade_rows = [r for r in rows if r["region"] == "blade_1"]
    for i, row in enumerate(blade_rows):
        assert float(row["time_s"]) == pytest.approx(i / 30, abs=1e-4)
        assert row["status"] == "ok"
        if i == 4:
            assert row["relative_status"] == "hub_insufficient_flow"
            assert row["relative_dx_px"] == row["relative_dy_px"] == ""
        else:
            assert float(row["relative_dx_px"]) == pytest.approx(2 * i, abs=0.3)
            assert float(row["relative_dy_px"]) == pytest.approx(-i, abs=0.3)
        assert row["shape_status"] == "ok"
        assert float(row["corner_0_x_px"]) == pytest.approx(135 + 3 * i, abs=0.3)
        assert float(row["area_ratio"]) == pytest.approx(1, abs=0.03)
        if i == 4:
            assert row["relative_corner_0_dx_px"] == ""
            assert row["area_px2"] != ""
    exported = cv2.VideoCapture(str(output / "annotated.mp4"))
    try:
        decoded = 0
        while True:
            ok, frame = exported.read()
            if not ok:
                break
            assert frame.shape[:2] == (160, 240)
            decoded += 1
        assert decoded == 7
    finally:
        exported.release()
    assert json.loads((output / "summary.json").read_text()) == summary
    with pytest.raises(ValueError, match="Refusing to overwrite"):
        analyze(video, output, config)


def test_frame_limit_and_saved_regions_reuse(
    video_case: tuple[Path, Path],
    tmp_path: Path,
):
    video, config = video_case
    output = tmp_path / "first"
    assert analyze(video, output, config, max_frames=3)["frames_processed"] == 3
    result = analyze(video, tmp_path / "second", output / "regions.json", max_frames=2)
    assert result["frames_processed"] == 2


def test_nonzero_start_uses_decoder_timestamps(
    video_case: tuple[Path, Path],
    tmp_path: Path,
):
    video, config = video_case
    data = json.loads(config.read_text())
    data["start_frame"] = 2
    data["regions"][0]["rect"] = [42, 92, 24, 24]
    data["regions"][1]["corners"] = [[141, 40], [160, 40], [160, 59], [141, 59]]
    config.write_text(json.dumps(data))
    result = analyze(video, tmp_path / "seek", config, start_frame=2, max_frames=2)
    assert result["first_time_s"] == pytest.approx(2 / 30, abs=1e-4)
    assert result["last_time_s"] == pytest.approx(3 / 30, abs=1e-4)


@pytest.mark.parametrize("change", [
    {"frame_size": [1, 1]},
    {"start_frame": 1},
    {"regions": []},
    {"regions": [{"name": "hub", "rect": [True, 0, 24, 24]}, {"name": "b", "rect": [1, 1, 8, 8]}]},
    {"regions": [{"name": "hub", "rect": [0, 0, 24, 24]}, {"name": "hub", "rect": [30, 0, 24, 24]}]},
])
def test_bad_configuration_is_rejected(
    video_case: tuple[Path, Path],
    change: dict[str, object],
):
    _, config = video_case
    data = json.loads(config.read_text())
    data.update(change)
    config.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_regions(config, cv2.cvtColor(textured_frame(), cv2.COLOR_GRAY2BGR), 0)


def test_invalid_video_and_limits(tmp_path: Path):
    with pytest.raises(ValueError, match="does not exist"):
        analyze(tmp_path / "missing.mp4", tmp_path / "output")
    with pytest.raises(ValueError, match="nonnegative"):
        analyze(Path("irrelevant.mp4"), tmp_path, start_frame=-1)


def test_export_verification_rejects_bad_frame_count(
    video_case: tuple[Path, Path],
):
    video, _ = video_case
    with pytest.raises(ValueError, match="expected 8 frames"):
        verify_export(video, 8, (240, 160))


def test_interactive_selection_maps_to_source_pixels(
    monkeypatch: pytest.MonkeyPatch,
):
    rects = iter([(30, 40, 20, 20), (100, 100, 20, 30), (200, 200, 25, 30)])
    keys = iter([ord("a"), ord(" ")])
    monkeypatch.setattr(cv2, "selectROI", lambda *_args, **_kwargs: next(rects))
    monkeypatch.setattr(cv2, "waitKey", lambda *_args: next(keys))
    monkeypatch.setattr(cv2, "imshow", lambda *_args: None)
    monkeypatch.setattr(cv2, "destroyWindow", lambda *_args: None)
    monkeypatch.setattr(cv2, "destroyAllWindows", lambda: None)

    def select_quad(
        _frame: np.ndarray,
        crop: Region,
        name: str,
    ):
        x, y, w, h = crop.rect
        corners = ((x, y), (x + w - 1, y), (x + w - 1, y + h - 1), (x, y + h - 1))
        return Region.from_corners(name, corners)

    monkeypatch.setattr("videoanalysis.__main__.select_quadrilateral", select_quad)
    frame = np.zeros((1700, 2400, 3), dtype=np.uint8)
    regions = select_regions(frame)
    assert regions == [
        Region("hub", (60, 80, 40, 40)),
        Region.from_corners("blade_1", ((200, 200), (239, 200), (239, 259), (200, 259))),
        Region.from_corners("blade_2", ((400, 400), (449, 400), (449, 459), (400, 459))),
    ]


def test_selection_cancellation(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(cv2, "selectROI", lambda *_args, **_kwargs: (0, 0, 0, 0))
    monkeypatch.setattr(cv2, "destroyWindow", lambda *_args: None)
    monkeypatch.setattr(cv2, "destroyAllWindows", lambda: None)
    with pytest.raises(ValueError, match="selection cancelled"):
        select_regions(cv2.cvtColor(textured_frame(), cv2.COLOR_GRAY2BGR))


def test_missing_timestamps_leave_no_success_summary(
    video_case: tuple[Path, Path],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    video, config = video_case
    capture = cv2.VideoCapture(str(video))
    wrapped = Mock(wraps=capture)
    wrapped.get.side_effect = lambda key: 0 if key == cv2.CAP_PROP_POS_MSEC else capture.get(key)
    monkeypatch.setattr(cv2, "VideoCapture", lambda _path: wrapped)
    output = tmp_path / "bad-time"
    with pytest.raises(ValueError, match="timestamp is missing or nonmonotonic"):
        analyze(video, output, config)
    wrapped.release.assert_called_once()
    assert not (output / "summary.json").exists()


def test_cli_failure_has_nonzero_exit_and_explicit_error(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    monkeypatch.setattr("sys.argv", ["videoanalysis", str(tmp_path / "missing.mp4")])
    assert main() == 1
    assert "Video analysis failed" in capsys.readouterr().err
