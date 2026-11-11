"""Approximate square-normal angles under local orthographic projection."""

import argparse
from collections import Counter
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

import cv2
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import numpy as np

from videoanalysis.geometry import Quad, edge_lengths, to_quad

type Normal = tuple[float, float, float]
OBJECT_CORNERS = np.asarray([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])
DESIGN = np.column_stack((OBJECT_CORNERS, np.ones(4)))
MAX_RESIDUAL_FRACTION = 0.1
DEFAULT_SMOOTH_SECONDS = 0.5
ANGLE_FIELDS = [
    "frame", "time_s", "region", "tracking_status", "orientation_status",
    "normal_x", "normal_y", "normal_z", "roll_deg", "pitch_deg",
    "smoothed_roll_deg", "smoothed_pitch_deg", "tilt_deg",
    "affine_residual_px", "affine_residual_fraction",
]
ASSUMPTIONS = {
    "model": "local_orthographic_flat_square",
    "lens_distortion": "assumed_zero",
    "axes": "image x right, y down, z into image",
    "angle_definition": "normal = Ry(pitch) Rx(roll) [0,0,1]",
    "normal_direction": "positive z",
    "mirror_ambiguity": "(-nx,-ny,nz) is equally possible; only one branch is displayed",
    "displayed_branch": "sign chosen for visual continuity, not physically disambiguated",
    "max_affine_residual_fraction": MAX_RESIDUAL_FRACTION,
    "calibrated_camera_pose": False,
}


@dataclass(frozen=True)
class Orientation:
    status: str
    residual_px: float
    residual_fraction: float
    normal: Normal | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    tilt_deg: float | None = None


def estimate_orientation(corners: Quad, previous_normal: Normal | None = None) -> Orientation:
    points = np.asarray(corners, dtype=np.float64)
    if (
        points.shape != (4, 2) or not np.isfinite(points).all()
        or not cv2.isContourConvex(points.astype(np.float32))
        or cv2.contourArea(points.astype(np.float32), oriented=True) <= 0
    ):
        raise ValueError("Orientation requires four finite, convex, clockwise square corners")
    coefficients = np.linalg.lstsq(DESIGN, points, rcond=None)[0]
    fitted = DESIGN @ coefficients
    residual = float(np.sqrt(np.mean(np.sum((points - fitted) ** 2, axis=1))))
    fraction = residual / float(edge_lengths(corners).min())
    if fraction > MAX_RESIDUAL_FRACTION:
        return Orientation("perspective_mismatch", residual, fraction)
    u, scales, _ = np.linalg.svd(coefficients[:2].T)
    ratio = float(np.clip(scales[1] / scales[0], 0, 1))
    # A projected unit square has singular values s and s*cos(tilt).
    xy = u[:, 1] * math.sqrt(max(0.0, 1 - ratio**2))
    if previous_normal is not None and np.linalg.norm(previous_normal[:2]) > 1e-8:
        if np.dot(xy, previous_normal[:2]) < 0:
            xy = -xy
    elif xy[int(np.argmax(np.abs(xy)))] < 0:
        xy = -xy
    normal = (float(xy[0]), float(xy[1]), ratio)
    roll = math.degrees(math.atan2(-normal[1], math.hypot(normal[0], normal[2])))
    pitch = math.degrees(math.atan2(normal[0], normal[2]))
    return Orientation(
        "approximate", residual, fraction, normal, roll, pitch, math.degrees(math.acos(ratio)),
    )


def angle_series(rows: list[dict], key: str) -> np.ndarray:
    return np.asarray([row.get(key) if row.get(key) is not None else np.nan for row in rows], dtype=float)


def smooth_angle_series(times: np.ndarray, values: np.ndarray, window_s: float) -> np.ndarray:
    if not math.isfinite(window_s) or window_s < 0:
        raise ValueError("Smoothing window must be finite and nonnegative")
    if (
        times.ndim != 1 or values.ndim != 1 or times.shape != values.shape
        or not np.isfinite(times).all() or (np.diff(times) <= 0).any()
        or np.isinf(values).any()
    ):
        raise ValueError("Smoothing requires increasing finite times and matching finite-or-NaN angles")
    if window_s == 0:
        return values.copy()
    smoothed = np.full(values.shape, np.nan, dtype=float)
    valid = np.flatnonzero(np.isfinite(values))
    for indices in np.split(valid, np.flatnonzero(np.diff(valid) > 1) + 1):
        if not len(indices):
            continue
        # A missing angle separates runs, even if it lies inside the time window.
        segment_times = times[indices]
        left = np.searchsorted(segment_times, segment_times - window_s / 2, side="left")
        right = np.searchsorted(segment_times, segment_times + window_s / 2, side="right")
        sums = np.concatenate(([0.0], np.cumsum(values[indices])))
        smoothed[indices] = (sums[right] - sums[left]) / (right - left)
    return smoothed


def plot_orientation(
    rows: list[dict], region: str, path: Path, smooth_seconds: float = DEFAULT_SMOOTH_SECONDS,
) -> None:
    figure = Figure(figsize=(12, 6), dpi=140)
    canvas = FigureCanvasAgg(figure)
    axes = figure.subplots(2, 1, sharex=True)
    times = np.asarray([row["time_s"] for row in rows])
    for axis, key, label, color in zip(
        axes, ("roll_deg", "pitch_deg"), ("Roll", "Pitch"), ("tab:blue", "tab:orange"), strict=True,
    ):
        if smooth_seconds > 0:
            axis.plot(times, angle_series(rows, key), color="0.6", alpha=0.4, linewidth=0.7, label="Raw")
            values = angle_series(rows, f"smoothed_{key}")
            label_text = f"{smooth_seconds:g} s centered mean"
        else:
            values = angle_series(rows, key)
            label_text = "Raw (smoothing disabled)"
        axis.plot(times, values, color=color, linewidth=1.6, marker=".", markersize=2, label=label_text)
        axis.set_ylabel(f"Approx. {label.lower()} (deg)")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
    axes[-1].set_xlabel("Source video time (s)")
    axes[-1].set_xlim(float(times[0]), float(times[-1]) if times[-1] > times[0] else float(times[0]) + 1)
    valid = sum(row["orientation_status"] == "approximate" for row in rows)
    figure.suptitle(f"{region}: square-normal roll / pitch ({valid}/{len(rows)} estimates)", parse_math=False)
    figure.text(
        0.5, 0.02,
        "Approximation: flat square, zero lens distortion, parallel viewing rays. "
        "Displayed sign is a convention; gaps are not filled.",
        ha="center", fontsize=8,
    )
    figure.tight_layout(rect=(0, 0.055, 1, 0.94))
    with path.open("xb") as file:
        canvas.print_png(file)
    figure.clear()


def export_orientation(
    source: Path, output: Path, smooth_seconds: float = DEFAULT_SMOOTH_SECONDS,
) -> dict:
    grouped: dict[str, list[dict]] = {}
    previous: dict[str, Normal] = {}
    last_frame: dict[str, int] = {}
    last_time: dict[str, float] = {}
    required = {"frame", "time_s", "region", "shape_status"} | {
        f"corner_{i}_{axis}_px" for i in range(4) for axis in ("x", "y")
    }
    with source.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("Input must be a videoanalysis displacement CSV with four corner coordinates")
        for line, row in enumerate(reader, start=2):
            if row["shape_status"] == "not_applicable":
                continue
            name, tracking_status = row["region"], row["shape_status"]
            if not name or not tracking_status:
                raise ValueError(f"CSV line {line}: missing region or shape status")
            frame, time_s = int(row["frame"]), float(row["time_s"])
            if (
                frame < 0 or not math.isfinite(time_s) or time_s < 0
                or frame <= last_frame.get(name, -1) or time_s <= last_time.get(name, -math.inf)
            ):
                raise ValueError(f"CSV line {line}: frame indices and times must increase within each region")
            last_frame[name], last_time[name] = frame, time_s
            result = {
                "frame": frame, "time_s": time_s, "region": name,
                "tracking_status": tracking_status, "orientation_status": "tracking_missing",
            }
            if tracking_status == "ok":
                corners = to_quad(np.asarray([
                    [float(row[f"corner_{i}_x_px"]), float(row[f"corner_{i}_y_px"])] for i in range(4)
                ]))
                estimate = estimate_orientation(corners, previous.get(name))
                result.update({
                    "orientation_status": estimate.status,
                    "affine_residual_px": estimate.residual_px,
                    "affine_residual_fraction": estimate.residual_fraction,
                })
                if estimate.normal is not None:
                    assert estimate.roll_deg is not None and estimate.pitch_deg is not None
                    previous[name] = estimate.normal
                    result.update({
                        "normal_x": estimate.normal[0], "normal_y": estimate.normal[1], "normal_z": estimate.normal[2],
                        "roll_deg": estimate.roll_deg, "pitch_deg": estimate.pitch_deg,
                        "tilt_deg": estimate.tilt_deg,
                    })
            grouped.setdefault(name, []).append(result)
    if not any(row["orientation_status"] == "approximate" for rows in grouped.values() for row in rows):
        raise ValueError("No tracked squares satisfy the orthographic approximation; no angle graph produced")
    for rows in grouped.values():
        times = np.asarray([row["time_s"] for row in rows], dtype=float)
        for key in ("roll_deg", "pitch_deg"):
            values = smooth_angle_series(times, angle_series(rows, key), smooth_seconds)
            for row, value in zip(rows, values, strict=True):
                row[f"smoothed_{key}"] = float(value) if math.isfinite(value) else None
    plots = {
        name: "normal_orientation.png" if i == 0 else f"normal_orientation_{i + 1}.png"
        for i, name in enumerate(grouped)
    }
    for name in ("orientation.csv", "orientation_summary.json", *plots.values()):
        if (output / name).exists():
            raise ValueError(f"Refusing to overwrite {output / name}; choose a new --output directory")
    output.mkdir(parents=True, exist_ok=True)
    with (output / "orientation.csv").open("x", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=ANGLE_FIELDS)
        writer.writeheader()
        for rows in grouped.values():
            writer.writerows(rows)
    for name, rows in grouped.items():
        plot_orientation(rows, name, output / plots[name], smooth_seconds)
    summary = {
        "source": str(source.resolve()), "assumptions": ASSUMPTIONS,
        "smoothing": {
            "method": "centered_moving_mean" if smooth_seconds > 0 else "none",
            "window_s": smooth_seconds,
            "gap_handling": "separate contiguous valid runs; never interpolate missing angles",
            "segment_endpoints": "truncate window to available samples",
            "raw_angles_preserved": True,
        },
        "status_counts": {
            name: dict(Counter(row["orientation_status"] for row in rows)) for name, rows in grouped.items()
        },
        "plots": plots,
    }
    with (output / "orientation_summary.json").open("x", encoding="utf-8") as file:
        file.write(json.dumps(summary, indent=2) + "\n")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot approximate square-normal roll/pitch from tracked corners.")
    parser.add_argument("csv", type=Path, help="videoanalysis displacement.csv")
    parser.add_argument("--output", type=Path, help="New directory for orientation.csv and graph(s)")
    parser.add_argument(
        "--smooth-seconds", type=float, default=DEFAULT_SMOOTH_SECONDS,
        help="Centered moving-mean window in seconds (default: 0.5; 0 disables smoothing)",
    )
    args = parser.parse_args()
    output = args.output or args.csv.parent / "orientation"
    try:
        summary = export_orientation(args.csv, output, smooth_seconds=args.smooth_seconds)
    except (ValueError, OSError, cv2.error, np.linalg.LinAlgError) as error:
        print(f"Orientation export failed: {error}\nPartial results may remain in {output}.", file=sys.stderr)
        return 1
    print("Approximate orthographic angles; one tilt sign convention shown, not physically disambiguated.")
    print(f"Smoothing window: {args.smooth_seconds:g} s (0 disables); raw angles preserved.")
    for name, counts in summary["status_counts"].items():
        print(f"  {name}: {counts}; graph: {output / summary['plots'][name]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
