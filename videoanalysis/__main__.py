"""Run with: uv run python -m videoanalysis VIDEO.mp4."""

import argparse
from collections import Counter
import csv
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

from videoanalysis.geometry import SHAPE_FIELDS, shape_measurements, to_quad, valid_quad
from videoanalysis.photometry import NORMALIZATION_SETTINGS, brightness_diagnostics, normalize_illumination
from videoanalysis.tracking import Image, Match, PatchTracker, QuadTracker, Region, relative_displacement


CSV_FIELDS = [
    "frame", "time_s", "elapsed_s", "region", "status", "score", "peak_gap",
    "dx_px", "dy_px", "relative_status", "relative_dx_px", "relative_dy_px",
    "method", "tracked_points", "inlier_ratio", "fb_error_px",
    "shape_status", *SHAPE_FIELDS,
    "raw_patch_median", "tracking_patch_median",
]


def verify_export(path: Path, expected_count: int, size: tuple[int, int]) -> None:
    exported = cv2.VideoCapture(str(path))
    try:
        if not exported.isOpened() or round(exported.get(cv2.CAP_PROP_FRAME_COUNT)) != expected_count:
            raise ValueError(f"Export verification failed: expected {expected_count} frames in {path}")
        for index in {0, expected_count - 1}:
            if not exported.set(cv2.CAP_PROP_POS_FRAMES, index):
                raise ValueError(f"Cannot seek in exported video: {path}")
            ok, frame = exported.read()
            if not ok or frame is None or (frame.shape[1], frame.shape[0]) != size:
                raise ValueError(f"Export verification failed at frame {index}: {path}")
    finally:
        exported.release()


def select_quadrilateral(frame: Image, crop: Region, name: str) -> Region:
    x, y, w, h = crop.rect
    scale = min(8.0, 1000 / w, 750 / h)
    zoom = cv2.resize(frame[y:y + h, x:x + w], (round(w * scale), round(h * scale)))
    points: list[tuple[float, float]] = []
    title = "Click 4 tape corners clockwise: Enter confirms, R resets, C/Esc cancels"

    def click(event: int, px: int, py: int, _flags: int, _param: object) -> None:
        if (
            event == cv2.EVENT_LBUTTONDOWN and len(points) < 4
            and 0 <= px < zoom.shape[1] and 0 <= py < zoom.shape[0]
        ):
            points.append((x + px * w / zoom.shape[1], y + py * h / zoom.shape[0]))

    cv2.namedWindow(title)
    cv2.setMouseCallback(title, click)
    try:
        while True:
            preview = zoom.copy()
            local = np.asarray([
                ((px - x) * zoom.shape[1] / w, (py - y) * zoom.shape[0] / h) for px, py in points
            ], dtype=np.int32).reshape(-1, 2)
            if len(local):
                cv2.polylines(preview, [local], len(local) == 4, (0, 255, 0), 2)
                for i, point in enumerate(local):
                    position = (int(point[0]), int(point[1]))
                    cv2.circle(preview, position, 4, (0, 255, 0), -1)
                    cv2.putText(preview, str(i), position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow(title, preview)
            key = cv2.waitKey(20) & 0xFF
            if key in (27, ord("c"), ord("C")) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                raise ValueError("Region selection cancelled")
            if key in (ord("r"), ord("R")):
                points.clear()
            if key in (10, 13):
                if len(points) == 4:
                    corners = to_quad(np.asarray(points, dtype=np.float64))
                    if valid_quad(corners, frame.shape[1], frame.shape[0]):
                        region = Region.from_corners(name, corners)
                        region.validate(frame)
                        return region
                print("Select four distinct corners clockwise around the tape; R resets.", flush=True)
    finally:
        cv2.destroyWindow(title)


def select_regions(frame: Image) -> list[Region]:
    scale = min(1.0, 1280 / frame.shape[1], 850 / frame.shape[0])
    display = cv2.resize(frame, (round(frame.shape[1] * scale), round(frame.shape[0] * scale)))

    def select(title: str) -> Region:
        print(title, flush=True)
        rect = cv2.selectROI(title, display, showCrosshair=True, fromCenter=False)
        cv2.destroyWindow(title)
        if rect[2] == 0 or rect[3] == 0:
            raise ValueError("Region selection cancelled")
        sx, sy = frame.shape[1] / display.shape[1], frame.shape[0] / display.shape[0]
        x, y, w, h = rect
        left, top = round(x * sx), round(y * sy)
        right, bottom = round((x + w) * sx), round((y + h) * sy)
        return Region("", (left, top, right - left, bottom - top))

    try:
        hub = select("HUB: select a rigid textured patch; Enter confirms, C cancels")
        regions = [Region("hub", hub.rect)]
        while True:
            crop = select("BLADE: box the tape with some margin for zoom; Enter confirms, C cancels")
            regions.append(select_quadrilateral(frame, crop, f"blade_{len(regions)}"))
            print("Add another blade patch? Press A; any other key starts analysis.", flush=True)
            cv2.imshow("A: add patch / any other key: finish", display)
            key = cv2.waitKey(0) & 0xFF
            cv2.destroyWindow("A: add patch / any other key: finish")
            if key not in (ord("a"), ord("A")):
                return regions
    finally:
        cv2.destroyAllWindows()


def load_regions(path: Path, frame: Image, start_frame: int) -> list[Region]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Region configuration must be a JSON object")
    if data.get("frame_size") != [frame.shape[1], frame.shape[0]]:
        raise ValueError("Region configuration frame size does not match the video")
    if data.get("start_frame") != start_frame:
        raise ValueError("Region configuration start_frame does not match --start-frame")
    entries = data.get("regions")
    if not isinstance(entries, list) or len(entries) < 2:
        raise ValueError("Select one hub region and at least one blade region")
    regions = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("Each region must be an object")
        name = entry.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError("Each region needs a nonempty name")
        if name == "hub":
            rect = entry.get("rect")
            if (
                "corners" in entry or not isinstance(rect, list) or len(rect) != 4
                or any(type(value) is not int for value in rect)
            ):
                raise ValueError("Hub needs four integer rect coordinates")
            region = Region(name, (rect[0], rect[1], rect[2], rect[3]))
        else:
            corners = entry.get("corners")
            if (
                "rect" in entry or not isinstance(corners, list) or len(corners) != 4
                or any(
                    not isinstance(point, list) or len(point) != 2
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in point)
                    for point in corners
                )
            ):
                raise ValueError("Blade markers need four [x, y] corners, not a rectangle")
            region = Region.from_corners(name, to_quad(np.asarray(corners, dtype=np.float64)))
        region.validate(frame)
        regions.append(region)
    if regions[0].name != "hub" or len({r.name for r in regions}) != len(regions):
        raise ValueError("First region must be named hub and all names must be unique")
    return regions


def annotate(
    frame: Image, regions: list[Region], matches: list[Match], index: int, time_s: float,
) -> Image:
    annotated = frame.copy()
    text_scale = max(0.5, min(frame.shape[:2]) / 1400)
    cv2.putText(
        annotated, f"frame {index}  t={time_s:.3f}s", (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX, text_scale, (255, 255, 255), 2,
    )
    for row, (region, match) in enumerate(zip(regions, matches, strict=True), start=1):
        x, y, w, h = region.rect
        if region.corners is not None:
            corners = match.corners if match.valid else region.corners
            assert corners is not None
            polygon = np.rint(corners).astype(np.int32)
            color = (0, 220, 0) if match.valid else (0, 0, 255)
            cv2.polylines(annotated, [polygon], True, color, 2)
            for i, point in enumerate(polygon):
                position = (int(point[0]), int(point[1]))
                cv2.circle(annotated, position, 3, color, -1)
                cv2.putText(annotated, str(i), position, cv2.FONT_HERSHEY_SIMPLEX, text_scale, color, 1)
        elif match.valid:
            assert match.dx is not None and match.dy is not None
            x, y = round(x + match.dx), round(y + match.dy)
        color = (0, 220, 0) if match.valid else (0, 0, 255)
        if region.corners is None:
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
        dx, dy = relative_displacement(match, matches[0])
        label = f"{region.name}: {match.status} {match.method}"
        label += f" ncc={match.score:.2f}"
        if match.tracked_points and match.valid:
            label += f" points={match.tracked_points}"
        if region.name != "hub":
            label += f" rel=({dx:.1f},{dy:.1f})px" if dx is not None else " rel=unavailable"
        text_width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, text_scale, 2)[0][0]
        fitted_scale = text_scale * min(1.0, (frame.shape[1] - 24) / max(1, text_width))
        cv2.putText(
            annotated, label, (12, 30 + row * max(24, round(35 * text_scale))),
            cv2.FONT_HERSHEY_SIMPLEX, fitted_scale, color, 2,
        )
        cv2.putText(
            annotated, region.name, (max(0, min(x, frame.shape[1] - 150)), max(50, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX, text_scale, color, 2,
        )
    return annotated


def analyze(
    video: Path,
    output: Path,
    regions_path: Path | None = None,
    start_frame: int = 0,
    max_frames: int | None = None,
    search_radius: int = 60,
    min_score: float = 0.75,
    compensate_flicker: bool = False,
) -> dict:
    if start_frame < 0 or (max_frames is not None and max_frames < 1):
        raise ValueError("Start frame must be nonnegative and max frames must be positive")
    if search_radius < 1 or not math.isfinite(min_score) or not 0 < min_score <= 1:
        raise ValueError("Search radius must be positive and minimum score must be in (0, 1]")
    if not video.is_file():
        raise ValueError(f"Video does not exist: {video}")
    names = ("regions.json", "displacement.csv", "annotated.mp4", "summary.json")
    for name in names:
        if (output / name).exists():
            raise ValueError(f"Refusing to overwrite {output / name}; choose a new --output directory")
    normalization_settings = {**NORMALIZATION_SETTINGS, "backend": "cpu"}
    capture = cv2.VideoCapture(str(video))
    writer = None
    try:
        if not capture.isOpened():
            raise ValueError(f"Cannot open video: {video}")
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        count_value = capture.get(cv2.CAP_PROP_FRAME_COUNT)
        if not math.isfinite(fps) or fps <= 0 or not math.isfinite(count_value) or count_value <= 0:
            raise ValueError("MP4 must report a valid frame rate and frame count")
        total_frames = round(count_value)
        if start_frame >= total_frames:
            raise ValueError(f"Start frame must be less than {total_frames}")
        if start_frame and not capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame):
            raise ValueError(f"Cannot seek to frame {start_frame}")
        ok, frame = capture.read()
        if not ok or frame is None:
            raise ValueError(f"Cannot decode first selected frame {start_frame}")
        if round(capture.get(cv2.CAP_PROP_POS_FRAMES)) != start_frame + 1:
            raise ValueError("Decoder did not seek to the requested frame")
        regions = (
            load_regions(regions_path, frame, start_frame)
            if regions_path is not None else select_regions(frame)
        )
        first_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if compensate_flicker:
            first_gray = normalize_illumination(first_gray)
        trackers = [
            PatchTracker(first_gray, r, search_radius, min_score, normalized_input=compensate_flicker)
            if r.corners is None else
            QuadTracker(first_gray, r, search_radius, min_score, normalized_input=compensate_flicker)
            for r in regions
        ]
        # Validate uniqueness even on the reference frame (e.g. repeating fasteners).
        initial_matches = [tracker.update(first_gray) for tracker in trackers]
        for region, match in zip(regions, initial_matches, strict=True):
            if not match.valid:
                raise ValueError(f"{region.name}: reference patch is {match.status}; select a distinctive patch")
        output.mkdir(parents=True, exist_ok=True)
        height, width = frame.shape[:2]
        if width % 2 or height % 2:
            raise ValueError("MP4 export requires even frame dimensions")
        writer = cv2.VideoWriter(
            str(output / "annotated.mp4"), cv2.VideoWriter.fourcc(*"mp4v"), fps, (width, height),
        )
        if not writer.isOpened():
            raise ValueError("Cannot initialize MP4 writer (mp4v codec)")
        config = {
            "video": str(video.resolve()), "frame_size": [width, height],
            "start_frame": start_frame,
            "regions": [
                {"name": r.name, **({"rect": r.rect} if r.corners is None else {"corners": r.corners})}
                for r in regions
            ],
        }
        (output / "regions.json").write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
        target_count = min(total_frames - start_frame, max_frames or total_frames)
        statuses: dict[str, Counter[str]] = {r.name: Counter() for r in regions}
        methods: dict[str, Counter[str]] = {r.name: Counter() for r in regions}
        relative_valid: Counter[str] = Counter()
        shape_valid: Counter[str] = Counter()
        brightness: dict[str, list[tuple[float, float]]] = {r.name: [] for r in regions}
        sample_times: list[float] = []
        first_time = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000
        previous_time = -math.inf
        with (output / "displacement.csv").open("x", newline="", encoding="utf-8") as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            csv_writer.writeheader()
            for offset in range(target_count):
                index = start_frame + offset
                if offset:
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        raise ValueError(f"Decode failed at frame {index}, before expected EOF {total_frames}")
                time_s = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000
                if not math.isfinite(time_s) or time_s < 0 or time_s <= previous_time:
                    raise ValueError(f"Decoder timestamp is missing or nonmonotonic at frame {index}")
                previous_time = time_s
                sample_times.append(time_s)
                raw_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray = normalize_illumination(raw_gray) if compensate_flicker else raw_gray
                matches = (
                    [replace(m, dx=0.0, dy=0.0) for m in initial_matches]
                    if offset == 0 else [t.update(gray) for t in trackers]
                )
                hub = matches[0]
                for region, match in zip(regions, matches, strict=True):
                    statuses[region.name][match.status] += 1
                    methods[region.name][match.method] += 1
                    dx, dy = relative_displacement(match, hub)
                    relative_status = "ok" if dx is not None else (
                        f"hub_{hub.status}" if not hub.valid else f"patch_{match.status}"
                    )
                    if dx is not None:
                        relative_valid[region.name] += 1
                    shape_status = "not_applicable" if region.corners is None else match.status
                    shape = {}
                    if region.corners is not None and match.valid:
                        assert match.corners is not None
                        hub_displacement = (
                            (hub.dx, hub.dy) if hub.dx is not None and hub.dy is not None and hub.valid else None
                        )
                        shape = shape_measurements(match.corners, region.corners, hub_displacement)
                        shape_valid[region.name] += 1
                    x, y, w, h = region.rect
                    raw_median = float(np.median(raw_gray[y:y + h, x:x + w]))
                    tracking_median = float(np.median(gray[y:y + h, x:x + w]))
                    brightness[region.name].append((raw_median, tracking_median))
                    csv_writer.writerow({
                        "frame": index, "time_s": time_s, "elapsed_s": time_s - first_time,
                        "region": region.name, "status": match.status, "score": match.score,
                        "peak_gap": match.peak_gap, "dx_px": match.dx, "dy_px": match.dy,
                        "relative_status": relative_status, "relative_dx_px": dx, "relative_dy_px": dy,
                        "method": match.method, "tracked_points": match.tracked_points,
                        "inlier_ratio": match.inlier_ratio, "fb_error_px": match.fb_error_px,
                        "shape_status": shape_status, **shape,
                        "raw_patch_median": raw_median, "tracking_patch_median": tracking_median,
                    })
                display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) if compensate_flicker else frame
                writer.write(annotate(display, regions, matches, index, time_s))
                if (offset + 1) % 100 == 0:
                    print(f"{offset + 1}/{target_count} frames", flush=True)
        writer.release()
        writer = None
        verify_export(output / "annotated.mp4", target_count, (width, height))
        summary = {
            "video": str(video.resolve()), "fps": fps, "frame_size": [width, height],
            "start_frame": start_frame, "frames_processed": target_count,
            "source_frames": total_frames, "first_time_s": first_time, "last_time_s": previous_time,
            "timestamp_source": "decoder_pos_msec",
            "search_radius": search_radius, "min_score": min_score,
            "min_peak_gap": trackers[0].anchor.min_peak_gap,
            "hub_flow_gates": {
                "min_points": 6, "max_fb_error_px": 1.0, "max_lk_error": 30,
                "min_surviving_fraction": 0.6, "min_affine_inlier_ratio": 0.7,
                "ransac_error_px": 1.5, "per_frame_scale_range": [0.7, 1.4],
            },
            "corner_flow_gates": {
                "required_corners": 4, "max_fb_error_px": 1.0, "max_lk_error": 30,
                "per_frame_edge_ratio_range": [0.7, 1.4], "min_edge_px": 4, "min_area_px2": 32,
            },
            "tracking_status_counts": {name: dict(counts) for name, counts in statuses.items()},
            "tracking_method_counts": {name: dict(counts) for name, counts in methods.items()},
            "relative_valid_frames": dict(relative_valid),
            "shape_valid_frames": dict(shape_valid),
            "photometric_normalization": normalization_settings if compensate_flicker else {"mode": "none"},
            "brightness_diagnostics": {
                name: brightness_diagnostics(
                    [pair[0] for pair in values], [pair[1] for pair in values], sample_times,
                )
                for name, values in brightness.items()
            },
        }
        (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        return summary
    finally:
        capture.release()
        if writer is not None:
            writer.release()


def main() -> int:
    parser = argparse.ArgumentParser(description="Track rotor patches relative to a rigid hub reference.")
    parser.add_argument("video", type=Path)
    parser.add_argument("--output", type=Path, help="New results directory; existing result files are never overwritten")
    parser.add_argument("--regions", type=Path, help="Saved regions.json; skips interactive selection")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int, help="Process only this many frames")
    parser.add_argument("--search-radius", type=int, default=60, help="Maximum per-frame search in original pixels")
    parser.add_argument("--min-score", type=float, default=0.75, help="Minimum template-anchor correlation, in (0,1]")
    parser.add_argument(
        "--compensate-flicker", action="store_true",
        help="Normalize local brightness/contrast for tracking and export a normalized grayscale analysis view",
    )
    args = parser.parse_args()
    output = args.output or Path("videoanalysis") / "results" / args.video.stem
    try:
        summary = analyze(
            args.video, output, args.regions, args.start_frame,
            args.max_frames, args.search_radius, args.min_score,
            args.compensate_flicker,
        )
    except (ValueError, OSError, cv2.error) as error:
        print(f"Video analysis failed: {error}\nPartial results may remain in {output}.", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print(f"Analysis cancelled. Partial results may remain in {output}.", file=sys.stderr)
        return 130
    print(f"Processed {summary['frames_processed']} frames. Results: {output.resolve()}")
    for name, counts in summary["tracking_status_counts"].items():
        valid = counts.get("ok", 0)
        print(f"  {name}: {valid}/{summary['frames_processed']} valid matches; statuses: {counts}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
