"""Export one recorded SITL pumping cycle for the static LinkHub UI."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess


SNAPSHOTS = {
    "HEARTBEAT", "ATTITUDE_QUATERNION", "LOCAL_POSITION_NED",
    "DEBUG_FLOAT_ARRAY", "EKF_STATUS_REPORT", "RC_CHANNELS",
}


def quaternion(roll: float, pitch: float, yaw: float) -> list[float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def export_cycle(run_dir: Path, journal: Path, linkhub: Path, cycle: int, hz: float) -> dict:
    if not math.isfinite(hz) or hz <= 0:
        raise ValueError("Sample rate must be finite and positive")
    with (run_dir / "telemetry.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    transitions = [
        index for index, row in enumerate(rows)
        if index == 0 or row["phase"] != rows[index - 1]["phase"]
    ]
    starts = [index for index in transitions if rows[index]["phase"] == "reel-out"]
    if cycle < 1 or cycle > len(starts):
        raise ValueError(f"Cycle must be in 1..{len(starts)}")
    first = starts[cycle - 1]
    subsequent = [index for index in transitions if index > first]
    if (
        len(subsequent) < 2
        or rows[subsequent[0]]["phase"] != "reel-in"
        or rows[subsequent[1]]["phase"] not in {"reel-out", "hold"}
    ):
        raise ValueError("Selected cycle does not contain complete reel-out and reel-in phases")
    last = subsequent[1]
    start, end = float(rows[first]["t_sim"]), float(rows[last]["t_sim"])
    events = [json.loads(line) for line in (run_dir / "events.jsonl").read_text().splitlines()]
    release = next(event["t_sim"] for event in events if event["event"] == "kinematic_exit")
    if start <= release:
        raise ValueError("Cycle starts before the canonical kinematic_exit boundary")

    result = subprocess.run(
        [
            str(linkhub.resolve()), "query", str(journal), "show",
            *[
                argument
                for message in ("HEARTBEAT", "ATTITUDE", "LOCAL_POSITION_NED", "DEBUG_FLOAT_ARRAY", "EKF_STATUS_REPORT", "RC_CHANNELS", "STATUSTEXT", "PARAM_VALUE")
                for argument in ("--type", message)
            ],
            "--dir", "rx", "--json",
        ],
        capture_output=True, text=True, check=True,
    )
    records: list[list] = []
    initial: dict[str, dict] = {}
    parameters = json.loads((run_dir / "params.json").read_text())
    parameter_types = {}
    past_start = False
    for line in result.stdout.splitlines():
        raw = json.loads(line)
        boot = raw.get("_sim_time_boot_ms")
        if boot is not None and float(boot) / 1000 >= start:
            past_start = True
        if raw["mavpackettype"] == "PARAM_VALUE":
            if not past_start:
                name = raw["param_id"]
                parameters[name] = raw["param_value"]
                parameter_types[name] = raw["param_type"]
            continue
        if boot is None:
            continue
        time = float(boot) / 1000
        message = raw["mavpackettype"]
        fields = {key: value for key, value in raw.items() if not key.startswith("_") and key not in {"mavpackettype", "t_rel"}}
        if message == "ATTITUDE":
            w, x, y, z = quaternion(fields.pop("roll"), fields.pop("pitch"), fields.pop("yaw"))
            fields.update(q1=w, q2=x, q3=y, q4=z)
            message = "ATTITUDE_QUATERNION"
        if message == "DEBUG_FLOAT_ARRAY":
            # Historic Lua sends a fixed-size array; discard uninitialized padding.
            data = fields["data"]
            fields["data"] = data[:1 + int(data[0]).bit_length()]
        if time < start and message in SNAPSHOTS:
            initial[message] = fields
        elif start <= time < end:
            records.append([round(time - start, 6), message, fields])
    required = {"HEARTBEAT", "ATTITUDE_QUATERNION", "LOCAL_POSITION_NED", "DEBUG_FLOAT_ARRAY"}
    if not required.issubset(initial):
        raise ValueError(f"Missing initial journal snapshots: {required - initial.keys()}")
    records.extend([0, message, fields] for message, fields in initial.items())

    next_sample = start
    for row in rows[first:last]:
        time = float(row["t_sim"])
        if time + 1e-8 < next_sample:
            continue
        next_sample += 1 / hz
        fields = {f"servo{index}_raw": float(row[f"servo_s{index}_us"]) for index in (1, 2, 3)}
        fields.update(servo9_raw=float(row["servo_mot_us"]), time_usec=round(time * 1e6), port=0)
        fields.update({f"servo{index}_raw": 0 for index in range(4, 9)})
        records.append([round(time - start, 6), "SERVO_OUTPUT_RAW", fields])
        records.append([
            round(time - start, 6), "NAMED_VALUE_FLOAT",
            {"name": "SIM_RPM", "value": float(row["omega_rotor"]) * 60 / (2 * math.pi), "time_boot_ms": round(time * 1000)},
        ])
    records.sort(key=lambda record: record[0])
    parameters["RAWES_MODE"] = 1
    return {
        "version": 1,
        "source": {
            "test": run_dir.name, "runId": journal.name, "cycle": cycle,
            "startSimSeconds": start, "endSimSeconds": end, "kinematicExit": release,
            "telemetrySha256": hashlib.sha256((run_dir / "telemetry.csv").read_bytes()).hexdigest(),
            "channels": {
                "attitude": "Recorded ArduPilot ATTITUDE converted to quaternion",
                "position": "Recorded ArduPilot LOCAL_POSITION_NED",
                "servos": "Recorded physics applied actuator PWM; AP SERVO_OUTPUT_RAW was unavailable",
                "rotor": "Recorded physics omega_rotor converted to SIM_RPM; not yaw-motor RPM",
                "unavailable": ["ATTITUDE_TARGET", "PID_TUNING", "RPM"],
            },
        },
        "duration": end - start,
        "groundHeight": float(rows[first]["pos_z"]) - float(initial["LOCAL_POSITION_NED"]["z"]),
        "phases": [
            {"time": 0, "name": "reel-out"},
            {"time": float(rows[subsequent[0]]["t_sim"]) - start, "name": "reel-in"},
        ],
        "parameters": parameters,
        "parameterTypes": parameter_types,
        "records": records,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--journal", type=Path, required=True)
    parser.add_argument("--linkhub", type=Path, required=True)
    parser.add_argument("--cycle", type=int, default=2)
    parser.add_argument("--hz", type=float, default=20)
    parser.add_argument("--output", type=Path, default=Path("linkhub-ui") / "src" / "demo" / "pumping-cycle.json")
    args = parser.parse_args()
    if not math.isfinite(args.hz) or args.hz <= 0:
        parser.error("--hz must be finite and positive")
    recording = export_cycle(args.run_dir, args.journal, args.linkhub, args.cycle, args.hz)
    content = json.dumps(recording, separators=(",", ":"), allow_nan=False) + "\n"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(content, encoding="utf-8")
    print(f"Exported {len(recording['records'])} records, {recording['duration']:.3f} s, {len(content)} bytes to {args.output}")


if __name__ == "__main__":
    main()
