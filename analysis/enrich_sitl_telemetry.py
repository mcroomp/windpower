"""Enrich mediator physics telemetry with LinkHub MAVLink observations."""

from __future__ import annotations

import csv
import json
import math
import os
from pathlib import Path
from typing import Any, Iterable

from analysis.linkhub_journal import iter_messages
from linkhub_client.messages import PidTuningAxis, decode_enum
from simulation.telemetry_columns import ASYNC_MAV_COLUMNS


_NVF_FIELDS = {
    "YFF_T": "mav_nvf_yff_trim",
    "YFF_U": "mav_nvf_yff_u",
    "YFF_GZ": "mav_nvf_yff_gz",
    "OL_RSP": "roll_sp_rads",
    "OL_PSP": "pitch_sp_rads",
    "OL_YSP": "yaw_sp_rads",
    "OL_RER": "roll_rate_err_rads",
    "OL_PER": "pitch_rate_err_rads",
    "OL_YER": "yaw_rate_err_rads",
    "OL_AP": "lua_ol_alt_p_contrib",
    "OL_AI": "lua_ol_alt_i_contrib",
    "OL_AD": "lua_ol_alt_d_contrib",
    "OL_COL": "lua_ol_thrust_cmd",
    "OL_TEN": "lua_ol_tension_n",
}

_PID_PREFIXES = {
    PidTuningAxis.ROLL: "rate_roll",
    PidTuningAxis.PITCH: "rate_pitch",
    PidTuningAxis.YAW: "rate_yaw",
}


def _quat_wxyz_to_rpy_deg(value: object) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) < 4:
        return (math.nan, math.nan, math.nan)
    w, x, y, z = (float(value[index]) for index in range(4))
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2.0, sinp)
        if abs(sinp) >= 1.0
        else math.asin(sinp)
    )
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _float_field(record: dict[str, Any], name: str) -> float | None:
    value = record.get(name)
    return None if value is None else float(value)


def _project_record(record: dict[str, Any]) -> dict[str, float]:
    message_type = str(record["mavpackettype"])
    fields: dict[str, float] = {}
    sim_time_boot_ms = record.get("_sim_time_boot_ms")
    if sim_time_boot_ms is not None:
        fields["mav_time_boot_ms"] = float(sim_time_boot_ms)

    if message_type == "ATTITUDE":
        fields.update(
            mav_att_roll_deg=math.degrees(float(record["roll"])),
            mav_att_pitch_deg=math.degrees(float(record["pitch"])),
            mav_att_yaw_deg=math.degrees(float(record["yaw"])),
            mav_att_roll_rate_rads=float(record["rollspeed"]),
            mav_att_pitch_rate_rads=float(record["pitchspeed"]),
            mav_att_yaw_rate_rads=float(record["yawspeed"]),
        )
    elif message_type == "ATTITUDE_TARGET":
        roll, pitch, yaw = _quat_wxyz_to_rpy_deg(record.get("q"))
        fields.update(
            mav_att_target_roll_deg=roll,
            mav_att_target_pitch_deg=pitch,
            mav_att_target_yaw_deg=yaw,
            mav_att_target_roll_rate_rads=float(record["body_roll_rate"]),
            mav_att_target_pitch_rate_rads=float(record["body_pitch_rate"]),
            mav_att_target_yaw_rate_rads=float(record["body_yaw_rate"]),
        )
    elif message_type == "SERVO_OUTPUT_RAW":
        time_usec = _float_field(record, "time_usec")
        if time_usec is not None:
            fields["mav_time_usec"] = time_usec
        for servo in (1, 2, 3, 9):
            value = _float_field(record, f"servo{servo}_raw")
            if value is not None:
                fields[f"mav_servo{servo}_us"] = value
    elif message_type == "NAMED_VALUE_FLOAT":
        target = _NVF_FIELDS.get(str(record.get("name", "")))
        value = _float_field(record, "value")
        if target is not None and value is not None:
            fields[target] = value
    elif message_type == "LOCAL_POSITION_NED":
        fields.update(
            ekf_pos_x=float(record["x"]),
            ekf_pos_y=float(record["y"]),
            ekf_pos_z=float(record["z"]),
        )
    elif message_type == "PID_TUNING":
        axis = record.get("axis")
        prefix = None if axis is None else _PID_PREFIXES.get(decode_enum(PidTuningAxis, axis))
        if prefix is not None:
            for source, suffix in (
                ("P", "p_contrib"),
                ("I", "i_contrib"),
                ("D", "d_contrib"),
                ("FF", "ff_contrib"),
                ("PDmod", "pdmod"),
                ("SRate", "srate"),
            ):
                value = _float_field(record, source)
                if value is not None:
                    fields[f"{prefix}_{suffix}"] = value
    return fields


def _observations_from_records(
    records: Iterable[dict[str, Any]],
    source: str,
) -> list[tuple[int, dict[str, float]]]:
    observations: list[tuple[int, dict[str, float]]] = []
    epochs: set[int] = set()
    for record_number, record in enumerate(records, start=1):
        if record.get("_dir") != "rx":
            continue
        if "_sim_time_boot_ms" not in record or "_sim_epoch" not in record:
            raise ValueError(
                f"{source}:{record_number}: LinkHub record lacks simulation clock metadata"
            )
        time_boot_ms = record.get("_sim_time_boot_ms")
        epoch = record.get("_sim_epoch")
        if time_boot_ms is None or epoch is None:
            continue
        epochs.add(int(epoch))
        fields = _project_record(record)
        if len(fields) > 1 or (
            fields and "mav_time_boot_ms" not in fields
        ):
            observations.append((int(time_boot_ms), fields))

    if not observations:
        raise ValueError(f"{source}: no supported inbound MAVLink observations")
    if len(epochs) != 1:
        raise ValueError(
            f"{source}: expected one simulation clock epoch, got {sorted(epochs)}"
        )
    return observations


def _load_observations(path: Path) -> list[tuple[int, dict[str, float]]]:
    def records() -> Iterable[dict[str, Any]]:
        with path.open(encoding="utf-8") as source:
            for line in source:
                if line.strip():
                    yield json.loads(line)

    return _observations_from_records(records(), str(path))


def enrich_sitl_telemetry_from_journal(
    physics_csv: str | Path,
    journal: str | Path,
    output_csv: str | Path,
    *,
    after: int = 0,
    through: int | None = None,
) -> None:
    observations = _observations_from_records(
        iter_messages(journal, after=after, through=through, direction="rx"),
        str(journal),
    )
    _enrich(Path(physics_csv), Path(output_csv), observations)


def enrich_sitl_telemetry(
    physics_csv: str | Path,
    mavlink_jsonl: str | Path,
    output_csv: str | Path,
) -> None:
    """Sample LinkHub observations onto the mediator's simulation timeline."""
    physics_path = Path(physics_csv)
    mavlink_path = Path(mavlink_jsonl)
    output_path = Path(output_csv)
    observations = _load_observations(mavlink_path)
    _enrich(physics_path, output_path, observations)


def _enrich(
    physics_path: Path,
    output_path: Path,
    observations: list[tuple[int, dict[str, float]]],
) -> None:
    with physics_path.open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        if reader.fieldnames is None:
            raise ValueError(f"{physics_path}: missing CSV header")
        missing = set(ASYNC_MAV_COLUMNS) - set(reader.fieldnames)
        if missing:
            raise ValueError(
                f"{physics_path}: missing autopilot telemetry columns: {sorted(missing)}"
            )
        rows = list(reader)
        fieldnames = reader.fieldnames

    snapshot = {name: math.nan for name in ASYNC_MAV_COLUMNS}
    observation_index = 0
    for row in rows:
        row_time_ms = round(float(row["sitl_time"]) * 1_000.0)
        while (
            observation_index < len(observations)
            and observations[observation_index][0] <= row_time_ms
        ):
            snapshot.update(observations[observation_index][1])
            observation_index += 1
        for name, value in snapshot.items():
            row[name] = str(value)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(f"{output_path.name}.tmp")
    try:
        with temporary_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        os.replace(temporary_path, output_path)
    finally:
        temporary_path.unlink(missing_ok=True)
