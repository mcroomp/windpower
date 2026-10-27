import csv
import json
import math
from pathlib import Path
from subprocess import CompletedProcess

import pytest

from analysis.export_demo_cycle import export_cycle, quaternion


@pytest.fixture
def recording(tmp_path, monkeypatch):
    rows = []
    for time, phase in [(0, "hold"), (10, "reel-out"), (11, "reel-out"),
                        (12, "reel-in"), (13, "reel-in"), (14, "reel-out"),
                        (15, "reel-in"), (16, "hold")]:
        rows.append({
            "t_sim": time, "phase": phase, "pos_z": -40,
            "servo_s1_us": 1200, "servo_s2_us": 1250, "servo_s3_us": 1300,
            "servo_mot_us": 1000, "omega_rotor": 10 * math.pi,
        })
    with (tmp_path / "telemetry.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (tmp_path / "events.jsonl").write_text(json.dumps({"event": "kinematic_exit", "t_sim": 1}))
    (tmp_path / "params.json").write_text(json.dumps({"RAWES_MODE": 0, "H_COL_MIN": 1100}))
    messages = {
        "HEARTBEAT": {"custom_mode": 20},
        "ATTITUDE": {"roll": 0, "pitch": 0, "yaw": math.pi / 2, "time_boot_ms": 9000},
        "LOCAL_POSITION_NED": {"x": 1, "y": 2, "z": -2},
        "DEBUG_FLOAT_ARRAY": {"array_id": 1, "data": [3, 4, 5, 999]},
    }
    journal = [
        {"mavpackettype": message, "_sim_time_boot_ms": 9000, **fields}
        for message, fields in messages.items()
    ]
    journal.insert(0, {
        "mavpackettype": "PARAM_VALUE", "param_id": "RAWES_YAW_SLP",
        "param_value": 0.0, "param_type": {"type": "MAV_PARAM_TYPE_REAL32"},
    })
    journal.extend([
        {"mavpackettype": "STATUSTEXT", "_sim_time_boot_ms": 11_000, "text": "recorded"},
        {"mavpackettype": "STATUSTEXT", "_sim_time_boot_ms": 14_000, "text": "outside"},
    ])

    def query(command, **kwargs):
        assert command.count("--type") == 8
        assert kwargs["check"] is True
        return CompletedProcess(command, 0, "\n".join(json.dumps(record) for record in journal), "")

    monkeypatch.setattr("analysis.export_demo_cycle.subprocess.run", query)
    return tmp_path


def test_exports_complete_cycle_with_physics_provenance(recording):
    result = export_cycle(recording, recording / "journal", Path("linkhub.exe"), 1, 20)
    assert result["duration"] == 4
    assert result["groundHeight"] == -38
    assert result["phases"] == [{"time": 0, "name": "reel-out"}, {"time": 2, "name": "reel-in"}]
    assert result["parameters"]["RAWES_MODE"] == 1
    assert result["parameters"]["RAWES_YAW_SLP"] == 0.0
    assert result["parameterTypes"]["RAWES_YAW_SLP"] == {"type": "MAV_PARAM_TYPE_REAL32"}
    assert result["source"]["channels"]["unavailable"] == ["ATTITUDE_TARGET", "PID_TUNING", "RPM"]
    assert len(result["source"]["telemetrySha256"]) == 64
    assert [row[0] for row in result["records"]] == sorted(row[0] for row in result["records"])
    assert all(0 <= row[0] < 4 for row in result["records"])
    attitude = next(fields for _, message, fields in result["records"] if message == "ATTITUDE_QUATERNION")
    assert attitude["q1"] == pytest.approx(math.sqrt(0.5))
    assert attitude["q4"] == pytest.approx(math.sqrt(0.5))
    diag = next(fields for _, message, fields in result["records"] if message == "DEBUG_FLOAT_ARRAY")
    assert diag["data"] == [3, 4, 5]
    rotor = next(fields for _, message, fields in result["records"] if message == "NAMED_VALUE_FLOAT")
    assert rotor["name"] == "SIM_RPM"
    assert rotor["value"] == pytest.approx(300)
    assert [fields["text"] for _, message, fields in result["records"] if message == "STATUSTEXT"] == ["recorded"]
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("cycle", [0, 3])
def test_rejects_missing_cycles(recording, cycle):
    with pytest.raises(ValueError, match="Cycle must be"):
        export_cycle(recording, recording / "journal", Path("linkhub.exe"), cycle, 20)


@pytest.mark.parametrize("hz", [0, -1, math.inf, math.nan])
def test_rejects_invalid_sample_rates(recording, hz):
    with pytest.raises(ValueError, match="Sample rate"):
        export_cycle(recording, recording / "journal", Path("linkhub.exe"), 1, hz)


def test_rejects_cycle_before_release(recording):
    (recording / "events.jsonl").write_text(json.dumps({"event": "kinematic_exit", "t_sim": 11}))
    with pytest.raises(ValueError, match="kinematic_exit"):
        export_cycle(recording, recording / "journal", Path("linkhub.exe"), 1, 20)


def test_quaternion_is_normalized_and_roll_pitch_order_is_preserved():
    assert quaternion(0, 0, 0) == [1, 0, 0, 0]
    roll = quaternion(math.pi / 2, 0, 0)
    pitch = quaternion(0, math.pi / 2, 0)
    assert roll == pytest.approx([math.sqrt(0.5), math.sqrt(0.5), 0, 0])
    assert pitch == pytest.approx([math.sqrt(0.5), 0, math.sqrt(0.5), 0])
    assert math.hypot(*quaternion(0.5, -0.3, 2)) == pytest.approx(1)
