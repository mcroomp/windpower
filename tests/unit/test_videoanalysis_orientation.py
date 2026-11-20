import csv
import json
import math
from pathlib import Path

import pytest

cv2 = pytest.importorskip("cv2", reason="Requires the videoanalysis extra")
pytest.importorskip("matplotlib", reason="Requires the videoanalysis extra")
import numpy as np  # noqa: E402 - skip before importing optional dependencies

from videoanalysis.geometry import Quad, to_quad  # noqa: E402 - skip before importing optional dependencies
from videoanalysis.orientation import (  # noqa: E402 - skip before importing optional dependencies
    ANGLE_FIELDS, Normal, angle_series, estimate_orientation, export_orientation, main, smooth_angle_series,
)


def projected_square(
    roll: float, pitch: float, spin: float = 0, scale: float = 80,
    translation: tuple[float, float] = (300, 400),
) -> tuple[Quad, Normal]:
    r, p, s = map(math.radians, (roll, pitch, spin))
    rx = np.asarray([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.asarray([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.asarray([[math.cos(s), -math.sin(s), 0], [math.sin(s), math.cos(s), 0], [0, 0, 1]])
    rotation = ry @ rx @ rz
    points = np.asarray([[-0.5, -0.5, 0], [0.5, -0.5, 0], [0.5, 0.5, 0], [-0.5, 0.5, 0]])
    corners = to_quad((points @ rotation.T)[:, :2] * scale + translation)
    normal = (float(rotation[0, 2]), float(rotation[1, 2]), float(rotation[2, 2]))
    return corners, normal


@pytest.mark.parametrize("roll,pitch", [(0, 0), (0, 35), (40, 0), (25, -45), (-60, 30), (80, -70)])
def test_known_orthographic_normal_and_angle_convention(
    roll: float,
    pitch: float,
):
    corners, expected_normal = projected_square(roll, pitch)
    estimate = estimate_orientation(corners, expected_normal)
    assert estimate.status == "approximate"
    assert estimate.normal == pytest.approx(expected_normal, abs=1e-7)
    assert estimate.roll_deg == pytest.approx(roll, abs=1e-5)
    assert estimate.pitch_deg == pytest.approx(pitch, abs=1e-5)
    assert estimate.residual_px < 1e-10
    assert estimate.normal is not None
    assert np.linalg.norm(estimate.normal) == pytest.approx(1)


def test_square_spin_scale_and_translation_do_not_change_normal():
    corners, expected = projected_square(35, -25)
    reference = estimate_orientation(corners, expected)
    for spin, scale, translation in [(23, 30, (10, 20)), (135, 250, (-1000, 2000))]:
        corners, _ = projected_square(35, -25, spin, scale, translation)
        measured = estimate_orientation(corners, expected)
        assert measured.normal == pytest.approx(reference.normal, abs=1e-10)
        assert measured.residual_fraction < 1e-12


def test_mirror_poses_are_indistinguishable_and_both_signs_are_preserved():
    corners, normal = projected_square(30, 40)
    mirrored_corners, mirrored_normal = projected_square(-30, -40)
    assert np.asarray(corners) == pytest.approx(np.asarray(mirrored_corners))
    first = estimate_orientation(corners, normal)
    second = estimate_orientation(corners, mirrored_normal)
    assert first.roll_deg is not None and second.roll_deg is not None
    assert first.pitch_deg is not None and second.pitch_deg is not None
    assert first.roll_deg == pytest.approx(-second.roll_deg)
    assert first.pitch_deg == pytest.approx(-second.pitch_deg)
    assert first.tilt_deg == pytest.approx(second.tilt_deg)


def test_branch_sign_is_stable_across_changing_square_spin():
    previous = None
    for spin in np.linspace(0, 360, 40):
        corners, _ = projected_square(20, -45, float(spin))
        estimate = estimate_orientation(corners, previous)
        assert estimate.normal is not None
        if previous is not None:
            assert np.dot(previous, estimate.normal) > 0.999999
        previous = estimate.normal


def test_excessive_nonparallelogram_error_is_explicitly_rejected():
    estimate = estimate_orientation(((70, 40), (140, 40), (160, 110), (50, 110)))
    assert estimate.status == "perspective_mismatch"
    assert estimate.residual_px == pytest.approx(10)
    assert estimate.residual_fraction == pytest.approx(10 / 70)
    assert estimate.normal is None and estimate.roll_deg is None and estimate.pitch_deg is None


@pytest.mark.parametrize("corners", [
    ((0, 0), (10, 10), (10, 0), (0, 10)),
    ((0, 0), (0, 10), (10, 10), (10, 0)),
    ((0, 0), (10, 0), (20, 0), (30, 0)),
    ((0, 0), (10, 0), (math.nan, 10), (0, 10)),
])
def test_invalid_corner_geometry_is_an_error(corners: Quad):
    with pytest.raises(ValueError, match="convex, clockwise"):
        estimate_orientation(corners)


def make_csv(
    path: Path,
    statuses: tuple[str, ...] = ("ok", "lost", "ok"),
    rolls: tuple[float, ...] | None = None,
):
    fields = ["frame", "time_s", "region", "shape_status"] + [
        f"corner_{i}_{axis}_px" for i in range(4) for axis in ("x", "y")
    ]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for frame, status in enumerate(statuses):
            corners, _ = projected_square(20 if rolls is None else rolls[frame], 40)
            writer.writerow({
                "frame": frame, "time_s": 20 + frame / 30, "region": "hub",
                "shape_status": "not_applicable",
            })
            row = {
                "frame": frame, "time_s": 20 + frame / 30, "region": "tape",
                "shape_status": status,
            }
            if status == "ok":
                row.update({f"corner_{i}_{axis}_px": point[j] for i, point in enumerate(corners)
                            for j, axis in enumerate(("x", "y"))})
            writer.writerow(row)


def test_export_has_real_graph_raw_angles_and_uninterpolated_missing_data(
    tmp_path: Path,
):
    source, output = tmp_path / "displacement.csv", tmp_path / "orientation"
    make_csv(source)
    summary = export_orientation(source, output)
    assert summary["status_counts"] == {"tape": {"approximate": 2, "tracking_missing": 1}}
    with (output / "orientation.csv").open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        rows = list(reader)
        assert reader.fieldnames == ANGLE_FIELDS
    assert len(rows) == 3
    assert rows[1]["tracking_status"] == "lost"
    for key in ("roll_deg", "pitch_deg", "smoothed_roll_deg", "smoothed_pitch_deg", "normal_x", "normal_y", "normal_z"):
        assert rows[1][key] == ""
    for row in (rows[0], rows[2]):
        assert float(row["roll_deg"]) == pytest.approx(20)
        assert float(row["pitch_deg"]) == pytest.approx(40)
        assert float(row["smoothed_roll_deg"]) == pytest.approx(20)
        assert float(row["smoothed_pitch_deg"]) == pytest.approx(40)
        assert "mirror_roll_deg" not in row and "mirror_pitch_deg" not in row
    assert json.loads((output / "orientation_summary.json").read_text())["assumptions"]["lens_distortion"] == "assumed_zero"
    image = cv2.imread(str(output / "normal_orientation.png"))
    assert image is not None and image.shape[:2] == (840, 1680)
    assert image.std() > 5
    assert summary["smoothing"]["method"] == "centered_moving_mean"
    assert summary["smoothing"]["window_s"] == 0.5
    assert summary["smoothing"]["raw_angles_preserved"] is True
    with pytest.raises(ValueError, match="Refusing to overwrite"):
        export_orientation(source, output)


def test_plot_data_has_nan_for_missing_measurements_instead_of_joining_across_gap():
    values = angle_series([{"roll_deg": 10}, {}, {"roll_deg": -20}], "roll_deg")
    assert values[0] == 10 and np.isnan(values[1]) and values[2] == -20


def test_centered_smoothing_uses_seconds_and_truncates_at_run_endpoints():
    times = np.arange(5) / 8
    values = np.asarray([0, 0, 10, 0, 0], dtype=float)
    smoothed = smooth_angle_series(times, values, 0.5)
    assert smoothed == pytest.approx([10 / 3, 2.5, 2, 2.5, 10 / 3])
    assert values.tolist() == [0, 0, 10, 0, 0]
    assert times == pytest.approx(np.arange(5) / 8)


def test_smoothing_never_averages_across_a_missing_angle_or_fills_it():
    times = np.arange(7) / 30
    values = np.asarray([0, 0, np.nan, 100, 100, np.nan, -20])
    smoothed = smooth_angle_series(times, values, 10)
    assert smoothed[[0, 1, 3, 4, 6]] == pytest.approx([0, 0, 100, 100, -20])
    assert np.array_equal(np.isnan(values), np.isnan(smoothed))


def test_smoothing_uses_actual_irregular_timestamps_not_sample_counts():
    times = np.asarray([0, 0.1, 0.4, 0.5])
    values = np.asarray([0, 10, 30, 40], dtype=float)
    assert smooth_angle_series(times, values, 0.25) == pytest.approx([5, 5, 35, 35])


def test_smoothing_reduces_jitter_without_delaying_a_linear_trend():
    times = np.arange(601) / 30
    trend = 5 + 0.1 * times
    jitter = 0.5 * (-1.0) ** np.arange(len(times))
    smoothed = smooth_angle_series(times, trend + jitter, 0.5)
    assert np.std((smoothed - trend)[15:-15]) < 0.05
    assert np.mean((smoothed - trend)[15:-15]) == pytest.approx(0, abs=1e-4)


def test_zero_window_returns_unchanged_copy_and_all_missing_stays_missing():
    values = np.asarray([10, np.nan, -20])
    result = smooth_angle_series(np.arange(3, dtype=float), values, 0)
    assert np.array_equal(result, values, equal_nan=True)
    assert result is not values
    assert np.isnan(smooth_angle_series(np.arange(3, dtype=float), np.full(3, np.nan), 0.5)).all()


@pytest.mark.parametrize("window", [-1, math.nan, math.inf])
def test_invalid_smoothing_window_is_an_error(window: float):
    with pytest.raises(ValueError, match="finite and nonnegative"):
        smooth_angle_series(np.arange(3, dtype=float), np.zeros(3), window)


@pytest.mark.parametrize("times,values", [
    (np.asarray([0, 0, 1]), np.zeros(3)),
    (np.asarray([0, math.nan, 1]), np.zeros(3)),
    (np.arange(3), np.zeros(2)),
    (np.arange(3), np.asarray([0, math.inf, 1])),
])
def test_invalid_smoothing_series_is_an_error(
    times: np.ndarray,
    values: np.ndarray,
):
    with pytest.raises(ValueError, match="increasing finite times"):
        smooth_angle_series(times, values, 0.5)


def test_smoothed_export_preserves_raw_angles_and_source_csv(tmp_path: Path):
    source, output = tmp_path / "displacement.csv", tmp_path / "smoothed"
    make_csv(source, ("ok",) * 5, rolls=(20, 30, 20, 30, 20))
    original = source.read_bytes()
    export_orientation(source, output, smooth_seconds=0.5)
    assert source.read_bytes() == original
    with (output / "orientation.csv").open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    assert [float(row["roll_deg"]) for row in rows] == pytest.approx([20, 30, 20, 30, 20])
    assert [float(row["smoothed_roll_deg"]) for row in rows] == pytest.approx([24] * 5)
    assert [float(row["pitch_deg"]) for row in rows] == pytest.approx([40] * 5)
    assert [float(row["smoothed_pitch_deg"]) for row in rows] == pytest.approx([40] * 5)


def test_cli_wires_user_selected_smoothing_window(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
):
    calls = []

    def export(source: Path, output: Path, smooth_seconds: float):
        calls.append((source, output, smooth_seconds))
        return {"status_counts": {}, "plots": {}}

    source = tmp_path / "displacement.csv"
    monkeypatch.setattr("videoanalysis.orientation.export_orientation", export)
    monkeypatch.setattr("sys.argv", ["videoanalysis.orientation", str(source), "--smooth-seconds", "1.25"])
    assert main() == 0
    assert calls == [(source, source.parent / "orientation", 1.25)]
    assert "1.25 s" in capsys.readouterr().out


def test_no_success_shaped_output_when_all_tracking_is_missing(tmp_path: Path):
    source, output = tmp_path / "displacement.csv", tmp_path / "orientation"
    make_csv(source, ("lost",) * 3)
    with pytest.raises(ValueError, match="No tracked squares"):
        export_orientation(source, output)
    assert not output.exists()


def test_cli_reports_input_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    source = tmp_path / "bad.csv"
    source.write_text("frame,time_s\n0,20\n")
    monkeypatch.setattr("sys.argv", ["videoanalysis.orientation", str(source)])
    assert main() == 1
    assert "Orientation export failed" in capsys.readouterr().err
