"""End-to-end compass-only passive bench startup with torque compensation."""
from __future__ import annotations

import csv
import math
import statistics
from pathlib import Path

import pytest
from pymavlink import mavutil

from analysis.linkhub_journal import cursor_sequence, iter_messages
from calibrate.hw import verify_safe_off
from calibrate.run import PassiveRunOptions, run_passive
from groundstation.ekf_flags import MAV_MODE_ARMED
from groundstation.rawes_modes import CMD_ENTER_GUIDED, CMD_ENTER_PASSIVE
from tests.sitl.stack_infra import StackContext
from tests.sitl.thread_trace import LinuxThreadTrace
from tests.sitl.torque.torque_test_utils import (
    assert_physics_yaw_rate,
    read_physics_psi_dot,
)

pytestmark = pytest.mark.sitl

_RUN_DURATION_S = 140.0
_PHYSICS_SETTLE_S = 110.0
_PHYSICS_OBSERVE_S = 20.0
_MAX_PHYSICS_YAW_RATE_RAD_S = math.radians(16.0)
_HANDOFF_OBSERVE_S = 5.0
_MAX_HANDOFF_QERR_DEG = 5.0
_MAX_SWASH_SPREAD_US = 50.0
_MAX_MOTOR_STEP = 0.15
_MAX_LATE_MOTOR_STDDEV = 0.05
_MIN_PID_CONTRIBUTION = 0.005


def _read_calibrate_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(
            line for line in stream if not line.startswith("#")
        ))


def _first_time(messages: list[dict], predicate) -> float:
    for message in messages:
        if predicate(message):
            return float(message["t_rel"])
    pytest.fail("Expected MAVLink event was not present")


def _is_command(message: dict, command: int) -> bool:
    """Match a journal COMMAND_LONG; LinkHub decodes ``command`` as an enum."""
    return message.get("command") == {
        "type": mavutil.mavlink.enums["MAV_CMD"][command].name
    }


@pytest.mark.timeout(2400)
def test_passive_bench_startup_torque_sitl(
    torque_unarmed_lua_calibrate: StackContext,
) -> None:
    """Run calibration's real passive startup and shutdown against torque SITL."""
    ctx = torque_unarmed_lua_calibrate
    before_csv = set(ctx.test_log_dir.glob("run_passive_*.csv"))
    journal_start_cursor = ctx.gcs.current_cursor()
    scripting_trace = LinuxThreadTrace(
        ctx.test_log_dir / "scripting-thread-trace.jsonl",
        process_name="arducopter-heli",
        thread_name="Scripting",
        stall_snapshot_s=0.5,
    )
    scripting_trace.start()
    try:
        run_passive(
            ctx.gcs,
            PassiveRunOptions(
                duration_s=_RUN_DURATION_S,
                force=True,
                thrust=0.342,
                protocol_debug=True,
                log_dir=ctx.test_log_dir,
            ),
        )
    finally:
        scripting_trace.stop()
    assert scripting_trace.samples >= 100

    csv_paths = set(ctx.test_log_dir.glob("run_passive_*.csv")) - before_csv
    assert len(csv_paths) == 1
    csv_path = csv_paths.pop()
    journal_end_cursor = ctx.gcs.flush_journal()

    dataflash_logs = ctx.gcs.list_logs(timeout=10.0)
    assert dataflash_logs
    latest_log = max(dataflash_logs, key=lambda entry: int(entry["id"]))
    dataflash_path = ctx.gcs.download_log(
        int(latest_log["id"]),
        ctx.test_log_dir / "passive-bench-startup.BIN",
        timeout=2.0,
        max_retries=10,
    )
    assert dataflash_path.stat().st_size > 1024

    rows = _read_calibrate_csv(csv_path)
    assert len(rows) >= 100

    qerr_deg = [
        (float(row["t_s"]), float(row["mav_att_qerr_deg"]))
        for row in rows
        if row["mav_att_qerr_deg"]
    ]
    assert qerr_deg
    handoff_start_s = qerr_deg[0][0]
    handoff_qerr_deg = [
        value
        for timestamp, value in qerr_deg
        if timestamp <= handoff_start_s + _HANDOFF_OBSERVE_S
    ]
    assert handoff_qerr_deg
    assert max(handoff_qerr_deg) <= _MAX_HANDOFF_QERR_DEG

    swash_spreads = []
    for row in rows:
        values = [
            float(row[name])
            for name in ("mav_servo1_us", "mav_servo2_us", "mav_servo3_us")
            if row[name]
        ]
        if len(values) == 3:
            swash_spreads.append(max(values) - min(values))
    assert swash_spreads
    assert max(swash_spreads) <= _MAX_SWASH_SPREAD_US

    trim_samples = [
        float(row["mav_nvf_yff_trim"])
        for row in rows
        if row["mav_nvf_yff_trim"]
    ]
    assert trim_samples
    assert max(trim_samples) > 0.05

    yaw_control_rows = [
        row for row in rows
        if row["mav_nvf_yff_u"]
        and row["mav_nvf_yff_trim"]
        and row["mav_pid_yaw_des"]
        and row["mav_pid_yaw_ach"]
        and row["mav_pid_yaw_p"]
        and row["mav_pid_yaw_i"]
    ]
    assert len(yaw_control_rows) >= 20

    total_commands = [
        float(row["mav_nvf_yff_u"]) for row in yaw_control_rows
    ]
    trim_commands = [
        float(row["mav_nvf_yff_trim"]) for row in yaw_control_rows
    ]
    pid_contributions = [
        total - trim
        for total, trim in zip(total_commands, trim_commands, strict=True)
    ]
    assert max(abs(value) for value in pid_contributions) > _MIN_PID_CONTRIBUTION

    signed_p_samples = []
    for row in yaw_control_rows:
        error = float(row["mav_pid_yaw_des"]) - float(row["mav_pid_yaw_ach"])
        p_term = float(row["mav_pid_yaw_p"])
        if abs(error) > 0.5 and abs(p_term) > 1e-4:
            signed_p_samples.append(error * p_term)
    assert len(signed_p_samples) >= 10
    assert sum(value > 0.0 for value in signed_p_samples) / len(
        signed_p_samples
    ) >= 0.9

    motor_steps = [
        abs(current - previous)
        for previous, current in zip(
            total_commands[:-1],
            total_commands[1:],
            strict=True,
        )
    ]
    assert motor_steps
    assert max(motor_steps) <= _MAX_MOTOR_STEP

    late_start = _RUN_DURATION_S - _PHYSICS_OBSERVE_S
    late_commands = [
        float(row["mav_nvf_yff_u"])
        for row in yaw_control_rows
        if float(row["t_s"]) >= late_start
    ]
    assert len(late_commands) >= 10
    assert statistics.pstdev(late_commands) <= _MAX_LATE_MOTOR_STDDEV

    assert ctx.linkhub_journal is not None
    messages = list(iter_messages(
        ctx.linkhub_journal,
        after=cursor_sequence(journal_start_cursor),
        through=cursor_sequence(journal_end_cursor),
    ))
    armed_acro_at = _first_time(
        messages,
        lambda message: (
            message.get("_dir") == "rx"
            and message.get("mavpackettype") == "HEARTBEAT"
            and int(message.get("base_mode", 0)) & MAV_MODE_ARMED
            and int(message.get("custom_mode", -1)) == 1
        ),
    )
    runup_complete_at = _first_time(
        messages,
        lambda message: (
            message.get("_dir") == "rx"
            and message.get("mavpackettype") == "STATUSTEXT"
            and "runup complete" in str(message.get("text", "")).lower()
        ),
    )
    land_clear_at = _first_time(
        messages,
        lambda message: (
            message.get("_dir") == "rx"
            and message.get("mavpackettype") == "EXTENDED_SYS_STATE"
            and int(message.get("landed_state", -1))
            == 2  # MAV_LANDED_STATE_IN_AIR
        ),
    )
    enter_guided_at = _first_time(
        messages,
        lambda message: (
            message.get("_dir") == "tx"
            and message.get("mavpackettype") == "COMMAND_LONG"
            and _is_command(message, CMD_ENTER_GUIDED)
        ),
    )
    guided_at = _first_time(
        messages,
        lambda message: (
            message.get("_dir") == "rx"
            and message.get("mavpackettype") == "HEARTBEAT"
            and int(message.get("base_mode", 0)) & MAV_MODE_ARMED
            and int(message.get("custom_mode", -1)) == 20
        ),
    )
    passive_enable_at = _first_time(
        messages,
        lambda message: (
            message.get("_dir") == "tx"
            and message.get("mavpackettype") == "COMMAND_LONG"
            and _is_command(message, CMD_ENTER_PASSIVE)
        ),
    )
    assert (
        armed_acro_at
        < runup_complete_at
        < land_clear_at
        < enter_guided_at
        < guided_at
        < passive_enable_at
    )

    assert_physics_yaw_rate(
        ctx.events_log,
        _MAX_PHYSICS_YAW_RATE_RAD_S,
        _PHYSICS_SETTLE_S,
        _PHYSICS_OBSERVE_S,
        ctx.log,
    )
    pre_settle_physics = read_physics_psi_dot(
        ctx.events_log,
        0.0,
        _PHYSICS_SETTLE_S,
    )
    assert pre_settle_physics
    early_physics = read_physics_psi_dot(
        ctx.events_log,
        pre_settle_physics[0]["t"],
        _PHYSICS_OBSERVE_S,
    )
    late_physics = read_physics_psi_dot(
        ctx.events_log,
        _PHYSICS_SETTLE_S,
        _PHYSICS_OBSERVE_S,
    )
    assert early_physics and late_physics
    assert max(abs(row["psi_dot"]) for row in late_physics) < max(
        abs(row["psi_dot"]) for row in early_physics
    )

    safe_off = verify_safe_off(ctx.gcs)
    assert safe_off.ok, safe_off.errors
