import json
import math
from pathlib import Path

import pytest

from analysis.analyze_motor_sessions import Point, analyze, analyze_session, summarize
from groundstation.rawes_diag import DIAG_KEYS


def export(tmp_path: Path, messages: list[tuple[float, str, dict, int]]) -> Path:
    records = [
        {
            "sequence": index, "kind": "mavlink.rx",
            "ingest_time_ns": round(time * 1e9), "sim_clock": {"epoch": 1},
            "data": {"system_id": 1, "component_id": component, "message_name": name, "fields": fields},
        }
        for index, (time, name, fields, component) in enumerate(messages, 1)
    ]
    path = tmp_path / "session.json"
    path.write_text(json.dumps({
        "metadata": {
            "start_cursor": "v1:1", "end_cursor": f"v1:{len(records)}",
            "start_time_ns": 0, "end_time_ns": records[-1]["ingest_time_ns"],
        },
        "records": records,
    }), encoding="utf-8")
    return path


def heartbeat(armed: bool = True, vehicle: bool = True) -> dict:
    return {
        "autopilot": {"type": "MAV_AUTOPILOT_ARDUPILOTMEGA" if vehicle else "MAV_AUTOPILOT_INVALID"},
        "base_mode": "MAV_MODE_FLAG_SAFETY_ARMED" if armed else "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
        "custom_mode": 20,
    }


def pid(error: float = 0.1) -> dict:
    return {
        "axis": {"type": "PID_TUNING_YAW"}, "desired": 0, "achieved": -error,
        "P": error, "I": 0, "D": 0, "FF": 0,
    }


def diag(**values: float) -> dict:
    data = [0.0] * (len(DIAG_KEYS) + 1)
    for name, value in values.items():
        index = DIAG_KEYS.index(name)
        data[0] += 1 << index
        data[index + 1] = value
    return {"array_id": 1, "data": data}


def test_smoothness_uses_fixed_bins_without_crossing_gaps_or_modes():
    points = [
        Point(0.01, 0, 1), Point(0.011, 2, 1),
        Point(0.11, 3, 1), Point(1.11, 100, 1), Point(1.21, 1000, 2),
    ]
    result = summarize(points, "percentage_points")
    assert result["samples"] == 5
    assert result["bins"] == 4
    assert result["mean"] == pytest.approx((1 + 3 + 100 + 1000) / 4)
    assert result["p95_abs_slew_per_s"] == pytest.approx(20)
    assert result["total_variation_per_s"] == pytest.approx(20)
    assert result["adjacent_bin_duration_s"] == pytest.approx(0.1)
    assert result["longest_contiguous_bin_span_s"] == pytest.approx(0.1)
    assert summarize([], "us")["rms"] is None


def test_rms_does_not_cancel_fast_opposite_errors_inside_one_bin():
    result = summarize([Point(0.01, -20, 1), Point(0.02, 20, 1)], "deg/s")
    assert result["mean"] == 0
    assert result["rms"] == 20
    assert result["std"] == 20


def test_filters_bridge_heartbeats_disarmed_samples_and_non_yaw_axes(tmp_path: Path):
    roll = pid()
    roll["axis"] = {"type": "PID_TUNING_ROLL"}
    path = export(tmp_path, [
        (0, "HEARTBEAT", heartbeat(), 1),
        (0.01, "HEARTBEAT", heartbeat(False, False), 191),
        (0.02, "PID_TUNING", pid(), 1),
        (0.12, "PID_TUNING", roll, 1),
        (0.2, "HEARTBEAT", heartbeat(False), 1),
        (0.22, "PID_TUNING", pid(10), 1),
    ])
    result = analyze_session(path)
    assert result["signals"]["rate_error"]["samples"] == 1
    assert result["signals"]["rate_error"]["rms"] == pytest.approx(math.degrees(0.1))
    assert result["signals"]["pid_feedback"]["rms"] == pytest.approx(10)
    assert result["signals"]["trim"]["mean"] is None
    assert result["quality"]["exclusions"]["disarmed_samples_excluded"] == 1
    assert result["tracking"]["observed_bins_fully_within_band_percent"] == 0
    json.dumps(result, allow_nan=False)


def test_diagnostic_masks_zero_trim_and_stale_trim_are_not_conflated(tmp_path: Path):
    path = export(tmp_path, [
        (0, "HEARTBEAT", heartbeat(), 1),
        (0.01, "DEBUG_FLOAT_ARRAY", diag(YFF_T=0, YFF_U=0.1), 1),
        (0.02, "PID_TUNING", pid(), 1),
        (1.2, "PID_TUNING", pid(), 1),
        (1.3, "DEBUG_FLOAT_ARRAY", diag(YFF_U=0.2), 1),
        (1.4, "PID_TUNING", pid(), 1),
        (3.2, "PID_TUNING", pid(), 1),
    ])
    result = analyze_session(path)
    assert result["signals"]["trim"]["mean"] == 0
    assert result["signals"]["trim_plus_reported_pid_terms"]["samples"] == 1
    assert result["signals"]["trim_plus_reported_pid_terms"]["mean"] == pytest.approx(10)
    assert result["quality"]["exclusions"]["pid_samples_without_recent_trim_frame"] == 2
    assert result["quality"]["exclusions"]["samples_without_fresh_heartbeat"] == 1


def test_reports_tracking_and_effort_trends_without_inventing_trim_causality(tmp_path: Path):
    messages = []
    for step in range(101):
        time = step / 10
        if step % 10 == 0:
            messages.append((time, "HEARTBEAT", heartbeat(), 1))
        messages.extend([
            (time, "DEBUG_FLOAT_ARRAY", diag(YFF_T=0.1 + step / 1000, YFF_U=0.2), 1),
            (time, "PID_TUNING", pid(0.2 if step < 50 else 0.05), 1),
        ])
    path = export(tmp_path, messages)
    (tmp_path / "index.json").write_text(json.dumps({"sessions": [{"file": path.name}]}), encoding="utf-8")
    result = analyze(tmp_path)
    session = result["sessions"][0]
    comparison = session["early_late_comparisons"][0]
    assert comparison["error_rms_reduction_percent"] == pytest.approx(75)
    assert comparison["pid_feedback_rms_reduction_percent"] == pytest.approx(75)
    assert comparison["windows"]["late"]["trim"]["mean"] > comparison["windows"]["early"]["trim"]["mean"]
    assert any("causal" in warning for warning in session["warnings"])


def test_rejects_truncated_exports_and_invalid_thresholds(tmp_path: Path):
    path = export(tmp_path, [(0, "HEARTBEAT", heartbeat(), 1), (0.1, "PID_TUNING", pid(), 1)])
    data = json.loads(path.read_text())
    data["records"].pop()
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="declared end"):
        analyze_session(path)
    with pytest.raises(ValueError, match="finite and positive"):
        analyze_session(path, math.nan)


def test_experiment_exports_and_directory_discovery(tmp_path: Path):
    source = export(tmp_path, [
        (0, "HEARTBEAT", heartbeat(), 1),
        (0.1, "PID_TUNING", pid(), 1),
    ])
    original = json.loads(source.read_text())
    metadata = original["metadata"]
    experiment = {
        "id": "experiment-1", "name": "First", "run_id": "run-1",
        "start_cursor": metadata["start_cursor"], "end_cursor": metadata["end_cursor"],
        "started_at_ns": metadata["start_time_ns"], "stopped_at_ns": metadata["end_time_ns"],
    }
    data = {"schema_version": 1, "experiment": experiment, "record_count": 2, "records": original["records"]}
    source.unlink()
    directory = tmp_path / "20261118_first"
    directory.mkdir()
    path = directory / "mavlog.json"
    path.write_text(json.dumps(data), encoding="utf-8")

    for selected in (path, directory, tmp_path):
        sessions = analyze(selected)["sessions"]
        assert len(sessions) == 1
        assert sessions[0]["metadata"]["id"] == "experiment-1"
        assert sessions[0]["signals"]["rate_error"]["samples"] == 1

    data["record_count"] = 3
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="record count"):
        analyze_session(path)


def test_window_metrics_keep_mode_blocks_and_motor_events_separate(tmp_path: Path):
    source = export(tmp_path, [
        (0, "HEARTBEAT", heartbeat(), 1),
        (0.1, "RPM", {"rpm1": 500}, 1),
        (0.2, "PID_TUNING", pid(), 1),
        (0.3, "HEARTBEAT", {**heartbeat(), "custom_mode": 1}, 1),
        (0.4, "PID_TUNING", pid(0.2), 1),
    ])
    data = json.loads(source.read_text())
    data["records"].append({
        "sequence": 6, "kind": "diagnostic.event", "ingest_time_ns": 500_000_000,
        "data": {"source": "linkhub.motor", "event": "motor.command", "fields": {"speed_percent": 40}},
    })
    data["metadata"]["end_cursor"] = "v1:6"
    data["metadata"]["end_time_ns"] = 500_000_000
    source.write_text(json.dumps(data), encoding="utf-8")

    result = analyze_session(source)
    assert result["motor_events"] == [{"time_s": 0.5, "event": "motor.command", "speed_percent": 40}]
    assert [window["mode"] for window in result["windows"]] == [20, 1]
    assert result["windows"][0]["signals"]["motor_rpm"]["mean"] == 500
    assert result["windows"][1]["signals"]["motor_rpm"]["mean"] is None
    assert result["windows"][0]["signals"]["rate_error"]["samples"] == 1


def test_vehicle_stamped_samples_keep_their_vehicle_spacing_despite_batched_delivery(tmp_path: Path):
    source = export(tmp_path, [
        (0, "HEARTBEAT", heartbeat(), 1),
        (0.5, "SERVO_OUTPUT_RAW", {"port": 0, "servo9_raw": 1200, "time_usec": 9_500_000}, 1),
        # Two diagnostics 54 ms apart on the vehicle arrive in one batch; the
        # later one is the least-delayed delivery and fixes the -9.054 s offset.
        (1.0, "DEBUG_FLOAT_ARRAY", {**diag(YFF_T=0.13, YFF_U=0.15), "time_usec": 10_000_000}, 1),
        (1.0, "DEBUG_FLOAT_ARRAY", {**diag(YFF_T=0.14, YFF_U=0.16), "time_usec": 10_054_000}, 1),
        (1.0, "PID_TUNING", pid(), 1),
    ])
    result = analyze_session(source)
    trim = result["signals"]["trim"]
    assert trim["samples"] == 2
    assert trim["span_s"] == pytest.approx(0.054)
    assert result["signals"]["servo9"]["samples"] == 1
    assert result["quality"]["exclusions"]["vehicle_timed_samples"] == 3
    latency = result["quality"]["receive_latency_ms_after_vehicle_time"]
    assert latency["samples"] == 3
    assert latency["max"] == pytest.approx(54)
    # PID_TUNING has no vehicle stamp and stays at receive time.
    assert result["signals"]["rate_error"]["span_s"] == 0


def test_marks_split_the_experiment_into_labelled_segments(tmp_path: Path):
    source = export(tmp_path, [
        (0, "HEARTBEAT", heartbeat(), 1),
        (0.2, "PID_TUNING", pid(0.1), 1),
        (1.2, "PID_TUNING", pid(0.3), 1),
        (1.4, "PID_TUNING", pid(0.3), 1),
    ])
    data = json.loads(source.read_text())
    for time_ns, label in ((100_000_000, "low"), (1_000_000_000, "high")):
        data["records"].append({
            "kind": "diagnostic.event", "ingest_time_ns": time_ns,
            "data": {"source": "linkhub", "event": "experiment.mark", "fields": {"label": label}},
        })
    data["records"].sort(key=lambda record: record["ingest_time_ns"])
    for sequence, record in enumerate(data["records"], 1):
        record["sequence"] = sequence
    data["metadata"]["end_cursor"] = "v1:6"
    data["metadata"]["end_time_ns"] = 1_500_000_000
    source.write_text(json.dumps(data), encoding="utf-8")

    result = analyze_session(source)
    assert result["marks"] == [{"time_s": 0.1, "label": "low"}, {"time_s": 1.0, "label": "high"}]
    low, high = result["segments"]
    assert (low["label"], low["start_s"], low["end_s"]) == ("low", 0.1, 1.0)
    assert (high["label"], high["end_s"]) == ("high", 1.5)
    assert low["signals"]["rate_error"]["samples"] == 1
    assert high["signals"]["rate_error"]["samples"] == 2
    assert high["signals"]["rate_error"]["mean"] == pytest.approx(math.degrees(0.3))


def test_experiment_without_vehicle_heartbeat_is_reported_as_unavailable(tmp_path: Path):
    source = export(tmp_path, [(0, "HEARTBEAT", heartbeat(vehicle=False), 191)])
    result = analyze_session(source)
    assert result["vehicle"] is None
    assert result["peak_motor_rpm"] is None
    assert result["signals"]["rate_error"]["rms"] is None
    assert any("No ArduPilot heartbeat" in warning for warning in result["warnings"])


def test_peak_rpm_distinguishes_unavailable_from_stopped(tmp_path: Path):
    source = export(tmp_path, [
        (0, "HEARTBEAT", heartbeat(), 1),
        (0.1, "RPM", {"rpm1": -1}, 1),
    ])
    assert analyze_session(source)["peak_motor_rpm"] is None
    data = json.loads(source.read_text())
    data["records"][1]["data"]["fields"]["rpm1"] = 0
    source.write_text(json.dumps(data), encoding="utf-8")
    assert analyze_session(source)["peak_motor_rpm"] == 0
