"""Analyze yaw control in LinkHub experiment and motor-session exports offline."""

from __future__ import annotations

import argparse
import bisect
import json
import math
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean

from analysis.linkhub_journal import cursor_sequence
from groundstation.rawes_diag import diag_values

BIN_S = 0.1
HEARTBEAT_MAX_AGE_S = 3.0
TRIM_MAX_AGE_S = 1.0
WINDOW_S = 10.0
# Vehicle-to-session clock offsets are re-estimated per chunk to follow drift
# between the autopilot crystal and the host clock.
CLOCK_CHUNK_S = 30.0


@dataclass(frozen=True, slots=True)
class Point:
    time: float
    value: float
    block: int


def vehicle_time_s(name: str, fields: dict) -> float | None:
    """Vehicle boot time of a message, if it carries one.

    rawes.lua stamps DEBUG_FLOAT_ARRAY.time_usec with its boot time; ArduPilot
    stamps SERVO_OUTPUT_RAW.time_usec and ATTITUDE*.time_boot_ms on the same
    clock. PID_TUNING and RPM carry no timestamp.
    """
    if name in ("DEBUG_FLOAT_ARRAY", "SERVO_OUTPUT_RAW"):
        value = numeric(fields, "time_usec")
        return value / 1e6 if value is not None and value > 0 else None
    if name in ("ATTITUDE", "ATTITUDE_QUATERNION"):
        value = numeric(fields, "time_boot_ms")
        return value / 1e3 if value is not None and value > 0 else None
    return None


class VehicleClock:
    """Maps vehicle boot time onto the session's receive-time axis.

    Per clock epoch and CLOCK_CHUNK_S of vehicle time, the smallest
    receive-minus-vehicle offset is the least-delayed delivery; offsets are
    interpolated between those anchors. Receive time additionally carries
    link batching, which this removes.
    """

    def __init__(self, samples: list[tuple[int, float, float]]):
        lowest: dict[tuple[int, int], tuple[float, float]] = {}
        for epoch, receive_s, vehicle_s in samples:
            key = (epoch, math.floor(vehicle_s / CLOCK_CHUNK_S))
            offset = receive_s - vehicle_s
            if key not in lowest or offset < lowest[key][1]:
                lowest[key] = (vehicle_s, offset)
        self.anchors: dict[int, list[tuple[float, float]]] = defaultdict(list)
        for (epoch, _), anchor in sorted(lowest.items()):
            self.anchors[epoch].append(anchor)
        latencies = [
            receive_s - mapped
            for epoch, receive_s, vehicle_s in samples
            if (mapped := self.session_time(epoch, vehicle_s)) is not None
        ]
        self.latency_ms = {
            "samples": len(latencies),
            "p50": percentile([1000 * value for value in latencies], 0.5),
            "p95": percentile([1000 * value for value in latencies], 0.95),
            "max": max((1000 * value for value in latencies), default=None),
        }

    def session_time(self, epoch: int, vehicle_s: float) -> float | None:
        anchors = self.anchors.get(epoch)
        if not anchors:
            return None
        index = bisect.bisect_left(anchors, (vehicle_s, -math.inf))
        if index == 0:
            return vehicle_s + anchors[0][1]
        if index == len(anchors):
            return vehicle_s + anchors[-1][1]
        (t0, o0), (t1, o1) = anchors[index - 1], anchors[index]
        return vehicle_s + o0 + (o1 - o0) * (vehicle_s - t0) / (t1 - t0)


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lo = math.floor(position)
    hi = math.ceil(position)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (position - lo)


def bins(points: list[Point]) -> dict[tuple[int, int], list[float]]:
    result: dict[tuple[int, int], list[float]] = defaultdict(list)
    for point in points:
        result[point.block, math.floor(point.time / BIN_S + 1e-9)].append(point.value)
    return result


def summarize(points: list[Point], unit: str) -> dict:
    grouped = bins(points)
    means = {key: fmean(values) for key, values in grouped.items()}
    values = list(means.values())
    mean = fmean(values) if values else None
    rms = math.sqrt(fmean(fmean(value * value for value in group) for group in grouped.values())) if values else None
    steps = [
        value - means[block, index - 1]
        for (block, index), value in means.items()
        if (block, index - 1) in means
    ]
    gaps = [
        right.time - left.time
        for left, right in zip(points, points[1:])
        if left.block == right.block
    ]
    longest = run = 0
    previous: tuple[int, int] | None = None
    for key in means:
        run = run + 1 if previous == (key[0], key[1] - 1) else 1
        longest = max(longest, run)
        previous = key
    return {
        "unit": unit,
        "samples": len(points),
        "bins": len(values),
        "span_s": points[-1].time - points[0].time if points else 0,
        "adjacent_bin_duration_s": len(steps) * BIN_S,
        "longest_contiguous_bin_span_s": max(0, longest - 1) * BIN_S,
        "max_sample_gap_s": max(gaps, default=None),
        "mean": mean,
        "rms": rms,
        "std": math.sqrt(max(0, rms * rms - mean * mean)) if rms is not None and mean is not None else None,
        "min": min(values, default=None),
        "max": max(values, default=None),
        "raw_peak_abs": max((abs(point.value) for point in points), default=None),
        "p95_abs": percentile([abs(value) for value in values], 0.95),
        "rms_step_per_100ms": math.sqrt(fmean(step * step for step in steps)) if steps else None,
        "p95_abs_slew_per_s": percentile([abs(step) / BIN_S for step in steps], 0.95),
        "peak_abs_slew_per_s": max((abs(step) / BIN_S for step in steps), default=None),
        "total_variation_per_s": fmean(abs(step) for step in steps) / BIN_S if steps else None,
    }


def numeric(fields: dict, key: str) -> float | None:
    value = fields.get(key)
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None


def reduction(early: float | None, late: float | None) -> float | None:
    return 100 * (early - late) / early if early is not None and early > 1e-12 and late is not None else None


def trends(series: dict[str, list[Point]], modes: dict[int, int]) -> list[dict]:
    result = []
    for block, mode in modes.items():
        error = [point for point in series["rate_error"] if point.block == block]
        if not error or error[-1].time - error[0].time < 8:
            continue
        start, end = error[0].time, error[-1].time
        quarter = (end - start) / 4
        windows = {}
        for name, lo, hi in (("early", start, start + quarter), ("late", end - quarter, end)):
            windows[name] = {
                key: summarize(
                    [point for point in series[key] if point.block == block and lo <= point.time <= hi],
                    "deg/s" if key == "rate_error" else "percentage_points",
                )
                for key in ("rate_error", "pid_feedback", "trim")
            }
        if any(windows[name]["rate_error"]["adjacent_bin_duration_s"] < 1 for name in windows):
            continue
        result.append({
            "block": block,
            "mode": mode,
            "start_s": start,
            "end_s": end,
            "windows": windows,
            "error_rms_reduction_percent": reduction(
                windows["early"]["rate_error"]["rms"], windows["late"]["rate_error"]["rms"],
            ),
            "pid_feedback_rms_reduction_percent": reduction(
                windows["early"]["pid_feedback"]["rms"], windows["late"]["pid_feedback"]["rms"],
            ),
        })
    return result


WINDOW_UNITS = {
    "rate_desired": "deg/s", "rate_achieved": "deg/s", "rate_error": "deg/s",
    "pid_feedback": "percentage_points", "trim": "percentage_points",
    "lua_applied": "percentage_points", "servo9": "us", "motor_rpm": "rpm",
}


def segment_summaries(series: dict[str, list[Point]], marks: list[dict], end_s: float) -> list[dict]:
    """Signals between each `experiment.mark` and the next mark or the end."""
    if not marks:
        return []
    bounds = [mark["time_s"] for mark in marks[1:]] + [end_s]
    return [
        {
            "label": mark["label"],
            "start_s": mark["time_s"],
            "end_s": end,
            "signals": {
                name: summarize(
                    [point for point in series[name] if mark["time_s"] <= point.time < end], unit,
                )
                for name, unit in WINDOW_UNITS.items()
            },
        }
        for mark, end in zip(marks, bounds, strict=True)
    ]


def window_summaries(series: dict[str, list[Point]], modes: dict[int, int]) -> list[dict]:
    units = WINDOW_UNITS
    grouped: dict[tuple[int, int], dict[str, list[Point]]] = defaultdict(lambda: defaultdict(list))
    for name in units:
        for point in series[name]:
            grouped[point.block, math.floor(point.time / WINDOW_S)][name].append(point)
    return [
        {
            "block": block,
            "mode": modes[block],
            "start_s": index * WINDOW_S,
            "end_s": (index + 1) * WINDOW_S,
            "signals": {name: summarize(points.get(name, []), unit) for name, unit in units.items()},
        }
        for (block, index), points in sorted(grouped.items())
    ]


def analyze_session(path: Path, rate_band_deg_s: float = 5.0) -> dict:
    if not math.isfinite(rate_band_deg_s) or rate_band_deg_s <= 0:
        raise ValueError("Rate band must be finite and positive")
    with path.open(encoding="utf-8") as stream:
        session = json.load(stream)
    if "experiment" in session:
        experiment = session["experiment"]
        metadata = {
            **experiment,
            "start_time_ns": experiment["started_at_ns"],
            "end_time_ns": experiment["stopped_at_ns"],
        }
        if session["record_count"] != len(session["records"]):
            raise ValueError(f"{path}: experiment record count does not match records")
    else:
        metadata = session["metadata"]
    records = session["records"]
    expected = cursor_sequence(metadata["start_cursor"])
    identities = {
        (record["data"]["system_id"], record["data"]["component_id"])
        for record in records
        if record["kind"] == "mavlink.rx"
        and record["data"]["message_name"] == "HEARTBEAT"
        and record["data"]["fields"].get("autopilot", {}).get("type") == "MAV_AUTOPILOT_ARDUPILOTMEGA"
    }
    if len(identities) > 1:
        raise ValueError(f"{path}: expected at most one ArduPilot heartbeat identity, found {identities}")
    identity = next(iter(identities), None)
    origin = metadata["start_time_ns"]
    clock = VehicleClock([
        (record["sim_clock"]["epoch"], (record["ingest_time_ns"] - origin) / 1e9, stamp)
        for record in records
        if record["kind"] == "mavlink.rx"
        and (record["data"]["system_id"], record["data"]["component_id"]) == identity
        and (stamp := vehicle_time_s(record["data"]["message_name"], record["data"]["fields"])) is not None
    ])
    series: dict[str, list[Point]] = defaultdict(list)
    counts: Counter[str] = Counter()
    quality: Counter[str] = Counter()
    heartbeat_time: float | None = None
    state: tuple[bool, int, int] | None = None
    trim: tuple[float, float, int] | None = None
    block = 0
    modes: dict[int, int] = {}
    previous_ns = origin
    rpm_peak: float | None = None
    motor_events = []
    marks: list[dict] = []

    def add(name: str, time: float, value: float) -> None:
        series[name].append(Point(time, value, block))

    for record in records:
        if record["sequence"] != expected:
            raise ValueError(f"{path}: non-contiguous journal at {expected}")
        expected += 1
        ns = record["ingest_time_ns"]
        if ns < previous_ns:
            raise ValueError(f"{path}: journal ingestion clock moved backwards")
        previous_ns = ns
        if record["kind"] == "diagnostic.event":
            event = record["data"]
            if event.get("source") == "linkhub.motor" and event.get("event") in (
                "motor.command", "motor.stop_requested", "motor.stopped",
            ):
                motor_events.append({
                    "time_s": (ns - origin) / 1e9,
                    "event": event["event"],
                    "speed_percent": event.get("fields", {}).get("speed_percent"),
                })
            elif event.get("event") == "experiment.mark":
                marks.append({"time_s": (ns - origin) / 1e9, "label": event["fields"]["label"]})
        if record["kind"] != "mavlink.rx":
            continue
        data = record["data"]
        if (data["system_id"], data["component_id"]) != identity:
            continue
        name, fields = data["message_name"], data["fields"]
        counts[name] += 1
        time = (ns - origin) / 1e9
        if name == "RPM":
            rpm = numeric(fields, "rpm1")
            if rpm is not None and rpm >= 0:
                rpm_peak = rpm if rpm_peak is None else max(rpm_peak, rpm)
        if name == "HEARTBEAT":
            flags = fields["base_mode"].split(" | ")
            new_state = (
                "MAV_MODE_FLAG_SAFETY_ARMED" in flags,
                int(fields["custom_mode"]),
                record["sim_clock"]["epoch"],
            )
            if new_state != state or heartbeat_time is None or time - heartbeat_time > HEARTBEAT_MAX_AGE_S:
                block += 1
                trim = None
            state = new_state
            heartbeat_time = time
            if state[0]:
                modes[block] = state[1]
        if name not in ("PID_TUNING", "DEBUG_FLOAT_ARRAY", "SERVO_OUTPUT_RAW", "RPM"):
            continue
        if name == "PID_TUNING" and fields.get("axis", {}).get("type") != "PID_TUNING_YAW":
            continue
        if state is None or heartbeat_time is None or time - heartbeat_time > HEARTBEAT_MAX_AGE_S:
            quality["samples_without_fresh_heartbeat"] += 1
            continue
        if record["sim_clock"]["epoch"] != state[2]:
            quality["samples_from_changed_clock_epoch"] += 1
            continue
        if not state[0]:
            quality["disarmed_samples_excluded"] += 1
            continue
        # Gating above uses receive time; stamped samples are placed at their
        # vehicle time so link batching does not distort their spacing.
        stamp = vehicle_time_s(name, fields)
        if stamp is not None and (mapped := clock.session_time(record["sim_clock"]["epoch"], stamp)) is not None:
            time = mapped
            quality["vehicle_timed_samples"] += 1
        if name == "RPM":
            rpm = numeric(fields, "rpm1")
            if rpm is not None and rpm >= 0:
                add("motor_rpm", time, rpm)
        elif name == "PID_TUNING":
            values = [numeric(fields, key) for key in ("desired", "achieved", "P", "I", "D", "FF")]
            if any(value is None for value in values):
                quality["invalid_pid_samples"] += 1
                continue
            desired, achieved, p, i, d, ff = values
            assert desired is not None and achieved is not None
            assert p is not None and i is not None and d is not None and ff is not None
            add("rate_desired", time, math.degrees(desired))
            add("rate_achieved", time, math.degrees(achieved))
            add("rate_error", time, math.degrees(desired - achieved))
            for key, value in (("pid_p", p), ("pid_i", i), ("pid_d", d), ("pid_ff", ff)):
                add(key, time, value * 100)
            feedback = p + i + d
            add("pid_feedback", time, feedback * 100)
            if trim is not None and trim[2] == block and 0 <= time - trim[0] <= TRIM_MAX_AGE_S:
                add("trim_plus_reported_pid_terms", time, (trim[1] + feedback + ff) * 100)
            else:
                quality["pid_samples_without_recent_trim_frame"] += 1
        elif name == "DEBUG_FLOAT_ARRAY":
            decoded = diag_values(int(fields["array_id"]), fields["data"])
            for key, signal, scale in (
                ("YFF_T", "trim", 100), ("YFF_U", "lua_applied", 100),
                ("YFF_GZ", "lua_gyro", 180 / math.pi),
            ):
                value = numeric(decoded, key)
                if value is not None:
                    add(signal, time, value * scale)
                elif key in decoded:
                    quality["invalid_diag_values"] += 1
            current_trim = numeric(decoded, "YFF_T")
            trim = (time, current_trim, block) if current_trim is not None else None
            applied = numeric(decoded, "YFF_U")
            if applied is not None and current_trim is not None:
                add("applied_minus_trim", time, (applied - current_trim) * 100)
        elif fields.get("port") == 0:
            pwm = numeric(fields, "servo9_raw")
            if pwm is not None and pwm >= 500:
                add("servo9", time, pwm)
            else:
                quality["disabled_or_invalid_servo_samples"] += 1
    if expected - 1 != cursor_sequence(metadata["end_cursor"]):
        raise ValueError(f"{path}: export ends before its declared end cursor")
    for points in series.values():
        points.sort(key=lambda point: point.time)

    names = (
        "rate_desired", "rate_achieved", "rate_error", "pid_p", "pid_i", "pid_d",
        "pid_ff", "pid_feedback", "trim", "lua_applied", "applied_minus_trim",
        "trim_plus_reported_pid_terms", "servo9", "lua_gyro", "motor_rpm",
    )
    summaries = {
        key: summarize(series[key], "us" if key == "servo9" else "rpm" if key == "motor_rpm" else
                       "deg/s" if key.startswith("rate_") or key == "lua_gyro" else "percentage_points")
        for key in names
    }
    error_bins = bins(series["rate_error"])
    in_band = sum(max(abs(value) for value in values) <= rate_band_deg_s for values in error_bins.values())
    warnings = [
        "Observational comparisons cannot isolate the causal benefit of trim from PID, RPM, load or setpoint changes.",
        "100 ms bin means suppress residual timing spikes, but cannot prove high-frequency actuator smoothness; "
        "PID_TUNING and RPM have no vehicle timestamp and stay on receive time.",
        "Missing pre-session state is not reconstructed; only fresh armed vehicle heartbeats enable control analysis.",
        "A recent Lua diagnostic frame does not prove its persisted trim/readback values were recently updated.",
        "Trim plus reported PID terms is not an actuator command: D_FF, feedforward scaling, mixer offsets/gates and mapping are not reconstructed.",
    ]
    if identity is None:
        warnings.append("No ArduPilot heartbeat observed; vehicle control analysis is unavailable.")
    for key in names:
        if not series[key]:
            warnings.append(f"No usable armed {key} samples; metrics are null.")
    if summaries["rate_error"]["longest_contiguous_bin_span_s"] < 2:
        warnings.append("Less than 2 seconds of contiguous armed yaw PID data; effectiveness is inconclusive.")
    if summaries["trim"]["max_sample_gap_s"] is not None and summaries["trim"]["max_sample_gap_s"] > TRIM_MAX_AGE_S:
        warnings.append("Trim frame gaps exceed the age limit; unmatched PID samples do not assume zero trim.")
    applied_values = [point.value for point in series["lua_applied"]]
    servo_values = [point.value for point in series["servo9"]]
    if applied_values and servo_values and max(map(abs, applied_values)) < 1e-6 and max(servo_values) - min(servo_values) > 20:
        warnings.append("Lua reports zero applied throttle while SERVO9 PWM varies by >20 us; inspect readback mapping before judging trim.")
    if quality:
        warnings.append("Excluded or unmatched samples are counted in quality.exclusions.")
    return {
        "file": str(path),
        "metadata": metadata,
        "duration_s": (metadata["end_time_ns"] - origin) / 1e9,
        "vehicle": {"system_id": identity[0], "component_id": identity[1]} if identity else None,
        "peak_motor_rpm": rpm_peak,
        "armed_blocks": [{"block": key, "mode": mode} for key, mode in modes.items()],
        "quality": {
            "vehicle_message_counts": dict(counts),
            "exclusions": dict(quality),
            "receive_latency_ms_after_vehicle_time": clock.latency_ms,
        },
        "signals": summaries,
        "tracking": {
            "rate_band_deg_s": rate_band_deg_s,
            "observed_bins_fully_within_band_percent": 100 * in_band / len(error_bins) if error_bins else None,
        },
        "early_late_comparisons": trends(series, modes),
        "motor_events": motor_events,
        "marks": marks,
        "segments": segment_summaries(series, marks, (metadata["end_time_ns"] - origin) / 1e9),
        "windows": window_summaries(series, modes),
        "warnings": warnings,
    }


def analyze(input_path: Path, rate_band_deg_s: float = 5.0) -> dict:
    if input_path.is_dir() and not (input_path / "index.json").exists():
        source = input_path
        paths = sorted(input_path.glob("*/mavlog.json"))
        if (input_path / "mavlog.json").is_file():
            paths.insert(0, input_path / "mavlog.json")
    else:
        source = input_path / "index.json" if input_path.is_dir() else input_path
        with source.open(encoding="utf-8") as stream:
            data = json.load(stream)
        paths = [source.parent / session["file"] for session in data["sessions"]] if "sessions" in data else [source]
        del data
    if not paths:
        raise ValueError(f"{source}: no sessions")
    return {
        "schema_version": 1,
        "input": str(source),
        "method": {
            "time_basis": (
                "Session seconds from LinkHub ingest_time_ns. DEBUG_FLOAT_ARRAY and SERVO_OUTPUT_RAW "
                "are placed at their vehicle time_usec, mapped per clock epoch by the lowest "
                "receive-minus-vehicle offset in each 30 s chunk; PID_TUNING, RPM and heartbeat "
                "gating use receive time."
            ),
            "bin_seconds": BIN_S,
            "statistics": "Bins have equal weight; RMS uses within-bin mean squares, not squared bin means. Raw peaks retain extremes.",
            "smoothness": "Adjacent bins within one armed/mode/clock block only; missing bins are not bridged.",
            "heartbeat_max_age_s": HEARTBEAT_MAX_AGE_S,
            "trim_max_age_s": TRIM_MAX_AGE_S,
            "early_late": "First/last quarter of each >=8 s armed PID block; each window needs >=1 s adjacent-bin coverage.",
            "windows": "Fixed 10 s session-time windows split by armed/mode/clock block; short or sparse windows are not steady-state proof.",
            "segments": "From each experiment.mark to the next mark or the experiment end; includes the transition after each mark.",
            "pid_terms": "Trim + reported P + I + D + FF is a partial, unclamped sum; it omits D_FF, feedforward scaling, mixer offsets/gates and mapping.",
            "trim_units": "Percentage points of normalized throttle; no assumed servo calibration or observer gain.",
        },
        "sessions": [analyze_session(path, rate_band_deg_s) for path in paths],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Experiment, session export, index.json, or directory of exports")
    parser.add_argument("--output", type=Path, help="Default: yaw_analysis.json beside the input")
    parser.add_argument("--rate-band-deg-s", type=float, default=5.0)
    args = parser.parse_args()
    result = analyze(args.input, args.rate_band_deg_s)
    parent = args.input if args.input.is_dir() else args.input.parent
    output = args.output or parent / "yaw_analysis.json"
    sources = {Path(session["file"]).resolve() for session in result["sessions"]}
    if not args.input.is_dir():
        sources.add(args.input.resolve())
    elif (args.input / "index.json").exists():
        sources.add((args.input / "index.json").resolve())
    if output.resolve() in sources:
        raise ValueError("Output must not overwrite an input session or index")
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print("Session                       error RMS deg/s   PID RMS pp   trim mean pp")
    for session in result["sessions"]:
        signals = session["signals"]
        values = [signals[key][metric] for key, metric in
                  (("rate_error", "rms"), ("pid_feedback", "rms"), ("trim", "mean"))]
        formatted = ["n/a" if value is None else f"{value:.3f}" for value in values]
        path = Path(session["file"])
        label = path.parent.name if path.name == "mavlog.json" else path.stem
        print(f"{label:40} {'  '.join(value.rjust(12) for value in formatted)}")
        for warning in session["warnings"]:
            print(f"  WARNING: {warning}")
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
