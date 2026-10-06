import csv
import json
import math

import pytest

from analysis.enrich_sitl_telemetry import enrich_sitl_telemetry
from groundstation.rawes_diag import DIAG_ARRAY_ID, DIAG_KEYS
from simulation.telemetry_columns import COLUMNS
from simulation.telemetry_csv import read_csv


def test_read_csv_ignores_incomplete_appended_row(tmp_path) -> None:
    path = tmp_path / "telemetry.csv"
    complete = [""] * len(COLUMNS)
    complete[COLUMNS.index("t_sim")] = "1.0"
    partial = ["2.0", "0.0", "2"]
    path.write_text(
        ",".join(COLUMNS) + "\n"
        + ",".join(complete) + "\n"
        + ",".join(partial),
        encoding="utf-8",
    )

    rows = read_csv(path)

    assert [row.t_sim for row in rows] == [1.0]


def _write_physics_csv(path) -> None:
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=COLUMNS)
        writer.writeheader()
        for t_sim in (0.0, 1.0, 2.0):
            row = dict.fromkeys(COLUMNS, 0.0)
            row["t_sim"] = t_sim
            row["sitl_time"] = t_sim
            writer.writerow(row)


def _record(time_boot_ms: int, message: str, **fields):
    return {
        "_t_wall": 0.0,
        "_dir": "rx",
        "_sim_epoch": 1,
        "_sim_time_boot_ms": time_boot_ms,
        "_sim_time_quality": "exact",
        "mavpackettype": message,
        **fields,
    }


def test_enriches_physics_rows_with_latest_mavlink_observation(tmp_path) -> None:
    physics = tmp_path / "telemetry.physics.csv"
    mavlink = tmp_path / "mavlink.jsonl"
    output = tmp_path / "telemetry.csv"
    _write_physics_csv(physics)
    records = [
        {
            "_dir": "rx",
            "_sim_epoch": 1,
            "_sim_time_boot_ms": None,
            "_sim_time_quality": None,
            "mavpackettype": "COMMAND_ACK",
            "command": {"type": "MAV_CMD_SET_MESSAGE_INTERVAL"},
            "result": {"type": "MAV_RESULT_ACCEPTED"},
        },
        _record(
            500,
            "ATTITUDE",
            roll=math.radians(10.0),
            pitch=math.radians(-5.0),
            yaw=math.radians(20.0),
            rollspeed=0.1,
            pitchspeed=0.2,
            yawspeed=0.3,
        ),
        _record(
            1_500,
            "SERVO_OUTPUT_RAW",
            time_usec=1_500_000,
            servo1_raw=1400,
            servo2_raw=1500,
            servo3_raw=1600,
            servo9_raw=1700,
        ),
    ]
    mavlink.write_text(
        "".join(json.dumps(record) + "\n" for record in records),
        encoding="utf-8",
    )

    enrich_sitl_telemetry(physics, mavlink, output)

    with output.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    assert math.isnan(float(rows[0]["mav_att_roll_deg"]))
    assert float(rows[1]["mav_att_roll_deg"]) == pytest.approx(10.0)
    assert math.isnan(float(rows[1]["mav_servo1_us"]))
    assert float(rows[2]["mav_servo1_us"]) == 1400.0
    assert float(rows[2]["mav_att_roll_deg"]) == pytest.approx(10.0)


def test_enrichment_rejects_export_without_simulation_clock_metadata(
    tmp_path,
) -> None:
    physics = tmp_path / "telemetry.physics.csv"
    mavlink = tmp_path / "mavlink.jsonl"
    _write_physics_csv(physics)
    mavlink.write_text(
        json.dumps({
            "_dir": "rx",
            "mavpackettype": "ATTITUDE",
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "rollspeed": 0.0,
            "pitchspeed": 0.0,
            "yawspeed": 0.0,
        })
        + "\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="lacks simulation clock metadata"):
        enrich_sitl_telemetry(physics, mavlink, tmp_path / "telemetry.csv")


def test_enrichment_maps_typed_pid_tuning_axis_to_rate_columns(tmp_path) -> None:
    physics = tmp_path / "telemetry.physics.csv"
    mavlink = tmp_path / "mavlink.jsonl"
    output = tmp_path / "telemetry.csv"
    _write_physics_csv(physics)
    records = [
        _record(500, "PID_TUNING", axis={"type": "PID_TUNING_YAW"}, P=0.25),
        _record(600, "PID_TUNING", axis={"type": "PID_TUNING_ACCZ"}, P=9.0),
    ]
    mavlink.write_text(
        "".join(json.dumps(record) + "\n" for record in records),
        encoding="utf-8",
    )

    enrich_sitl_telemetry(physics, mavlink, output)

    with output.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    assert float(rows[1]["rate_yaw_p_contrib"]) == pytest.approx(0.25)
    assert math.isnan(float(rows[1]["rate_roll_p_contrib"]))
    assert math.isnan(float(rows[1]["rate_pitch_p_contrib"]))


def test_enrichment_maps_diagnostic_array_keys_by_validity_mask(tmp_path) -> None:
    physics = tmp_path / "telemetry.physics.csv"
    mavlink = tmp_path / "mavlink.jsonl"
    output = tmp_path / "telemetry.csv"
    _write_physics_csv(physics)
    keys = list(DIAG_KEYS)
    data = [0.0] * 58
    for key, value in (("YFF_U", 0.25), ("OL_TEN", 12.5), ("OL_AP", 0.0)):
        index = keys.index(key)
        data[0] += 1 << index
        data[index + 1] = value
    records = [
        _record(500, "DEBUG_FLOAT_ARRAY", array_id=DIAG_ARRAY_ID, name="RAWES_DIAG", data=data),
        _record(600, "DEBUG_FLOAT_ARRAY", array_id=DIAG_ARRAY_ID + 1, name="OTHER", data=data),
    ]
    mavlink.write_text(
        "".join(json.dumps(record) + "\n" for record in records),
        encoding="utf-8",
    )

    enrich_sitl_telemetry(physics, mavlink, output)

    with output.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    assert float(rows[1]["mav_nvf_yff_u"]) == pytest.approx(0.25)
    assert float(rows[1]["lua_ol_tension_n"]) == pytest.approx(12.5)
    assert float(rows[1]["lua_ol_alt_p_contrib"]) == 0.0
    assert math.isnan(float(rows[1]["mav_nvf_yff_trim"]))
