from uuid import UUID

import pytest

import linkhub_client.client as client_module
from linkhub_client import (
    DiagnosticEvent,
    DiagnosticLevel,
    DiagnosticRecord,
    LinkHubClient,
    LinkHubError,
    SimClock,
    SimTimeQuality,
)
from linkhub_client.messages import (
    CommandAck,
    EkfStatusFlags,
    EkfStatusReport,
    ExtendedSysState,
    Heartbeat,
    MavAutopilot,
    MavCmd,
    MavLandedState,
    MavModeFlag,
    MavParamType,
    MavResult,
    MavSeverity,
    MavState,
    MavSysStatusSensor,
    MavType,
    MavVtolState,
    ParamSet,
    decode_message,
    encode_enum,
    RawMessage,
)

_CLOCK_DICT = {
    "epoch": 1,
    "time_boot_ms": 12_345,
    "quality": "last_observed",
}


def test_client_waits_for_mavlink_heartbeat_before_connecting(monkeypatch) -> None:
    client = LinkHubClient()
    statuses = iter(
        [
            {
                "connected": True,
                "ready": False,
                "target_system": 0,
                "target_component": 0,
            },
            {
                "connected": True,
                "ready": True,
                "target_system": 4,
                "target_component": 7,
                "cursor": "v1:9",
            },
        ]
    )
    monkeypatch.setattr(client, "_request_json", lambda *_args: next(statuses))
    monkeypatch.setattr(client_module.time, "sleep", lambda _seconds: None)

    client.connect(timeout=1.0)

    assert client._target_system == 4
    assert client._target_component == 7


def test_client_preserves_per_message_rate_results(monkeypatch) -> None:
    client = LinkHubClient()
    configured = [{
        "message": "ATTITUDE",
        "rate_hz": 100.0,
        "interval_us": 10_000,
        "after_cursor": "v1:1",
    }]
    monkeypatch.setattr(
        client,
        "_request_json",
        lambda *_args, **_kwargs: {"configured": configured},
    )

    assert client.set_message_rates({"ATTITUDE": 100.0}) == configured


def test_client_reads_authoritative_vehicle_state(monkeypatch) -> None:
    client = LinkHubClient()
    monkeypatch.setattr(
        client,
        "_request_json",
        lambda *_args, **_kwargs: {
            "base_mode": (
                "MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED"
            ),
            "custom_mode": 4,
            "system_status": {"type": "MAV_STATE_ACTIVE"},
            "sim_clock": _CLOCK_DICT,
        },
    )

    status = client.vehicle_status()

    assert client.sim_now() == 12.345
    assert client.is_armed
    assert status["base_mode"] == {
        MavModeFlag.SAFETY_ARMED,
        MavModeFlag.CUSTOM_MODE_ENABLED,
    }
    assert status["system_status"] is MavState.ACTIVE
    assert status["custom_mode"] == 4


def test_client_reports_disarmed_when_the_armed_flag_is_clear(monkeypatch) -> None:
    client = LinkHubClient()
    monkeypatch.setattr(
        client,
        "_request_json",
        lambda *_args, **_kwargs: {"base_mode": "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED"},
    )
    assert not client.is_armed

    monkeypatch.setattr(
        client, "_request_json", lambda *_args, **_kwargs: {"base_mode": ""}
    )
    assert not client.is_armed


def test_client_decodes_component_heartbeat_state(monkeypatch) -> None:
    client = LinkHubClient()
    monkeypatch.setattr(
        client,
        "_request_json",
        lambda *_args, **_kwargs: {
            "components": [
                {
                    "system_id": 1,
                    "component_id": 1,
                    "vehicle_type": {"type": "MAV_TYPE_HELICOPTER"},
                    "autopilot": {"type": "MAV_AUTOPILOT_ARDUPILOTMEGA"},
                    "base_mode": "MAV_MODE_FLAG_SAFETY_ARMED",
                    "custom_mode": 4,
                    "system_status": {"type": "MAV_STATE_STANDBY"},
                    "last_heartbeat_ns": 5,
                }
            ]
        },
    )

    (component,) = client.components()

    assert component["vehicle_type"] is MavType.HELICOPTER
    assert component["autopilot"] is MavAutopilot.ARDUPILOTMEGA
    assert component["base_mode"] == {MavModeFlag.SAFETY_ARMED}
    assert component["system_status"] is MavState.STANDBY


def test_client_sends_typed_commands_and_decodes_the_acknowledgement(
    monkeypatch,
) -> None:
    client = LinkHubClient()
    requests = []

    def request(method, path, body=None, **kwargs):
        requests.append((method, path, body))
        return {
            "command": {"type": "MAV_CMD_COMPONENT_ARM_DISARM"},
            "result": {"type": "MAV_RESULT_TEMPORARILY_REJECTED"},
            "progress": 0,
            "status": "acknowledged",
            "after_cursor": "v1:3",
        }

    monkeypatch.setattr(client, "_request_json", request)

    result = client.command(MavCmd.COMPONENT_ARM_DISARM, [1.0, 0.0])

    assert requests[0][0:2] == ("POST", "/v1/mavlink/commands")
    assert requests[0][2]["command"] == {"type": "MAV_CMD_COMPONENT_ARM_DISARM"}
    assert requests[0][2]["params"] == [1.0, 0.0]
    assert result["command"] is MavCmd.COMPONENT_ARM_DISARM
    assert result["result"] is MavResult.TEMPORARILY_REJECTED
    assert result["after_cursor"] == "v1:3"


def test_client_rejects_a_refused_arm_command(monkeypatch) -> None:
    client = LinkHubClient()
    monkeypatch.setattr(
        client,
        "_request_json",
        lambda *_args, **_kwargs: {
            "command": {"type": "MAV_CMD_COMPONENT_ARM_DISARM"},
            "result": {"type": "MAV_RESULT_DENIED"},
        },
    )

    with pytest.raises(LinkHubError, match="Arm was rejected"):
        client.arm()


def test_client_reads_finite_server_filtered_message_batch(monkeypatch) -> None:
    client = LinkHubClient()
    record = {
        "received_time": "2026-01-01T00:00:00Z",
        "received_time_ns": 1,
        "direction": "rx",
        "system_id": 1,
        "component_id": 1,
        "message": "STATUSTEXT",
        "fields": {"text": "ready", "severity": {"type": "MAV_SEVERITY_INFO"}},
        "cursor": "v1:12",
        "sim_clock": _CLOCK_DICT,
    }
    requests = []

    def request(method, path, body=None, **kwargs):
        requests.append((method, path, body, kwargs))
        return {
            "records": [record],
            "next_cursor": "v1:15",
            "next_clock": _CLOCK_DICT,
        }

    monkeypatch.setattr(client, "_request_json", request)

    batch = client.read_messages(
        "v1:9",
        "STATUSTEXT",
        direction="rx",
        wait=2.0,
        limit=10,
    )

    assert [message.text for message in batch.messages] == ["ready"]
    assert batch.next_cursor == "v1:15"
    assert batch.next_clock == SimClock(
        epoch=1,
        time_boot_ms=12_345,
        quality=SimTimeQuality.LAST_OBSERVED,
    )
    assert requests == [(
        "GET",
        "/v1/mavlink/messages?"
        "after=v1%3A9&wait_ms=2000&limit=10&messages=STATUSTEXT&direction=rx",
        None,
        {"timeout": 5.0},
    )]

    requests.clear()
    client.read_messages(
        "v1:9",
        ["ATTITUDE", "HEARTBEAT"],
        direction="rx",
        limit=100,
        collapse=True,
    )

    assert requests == [(
        "GET",
        "/v1/mavlink/messages?"
        "after=v1%3A9&wait_ms=0&limit=100&collapse=true"
        "&messages=ATTITUDE%2CHEARTBEAT&direction=rx",
        None,
        {"timeout": 5.0},
    )]


def test_client_decodes_generated_enum_types_and_preserves_unknown_values(
    monkeypatch,
) -> None:
    client = LinkHubClient()
    records = [
        {
            "received_time": "2026-01-01T00:00:00Z",
            "received_time_ns": index,
            "direction": "rx",
            "system_id": 1,
            "component_id": 1,
            "message": "EXTENDED_SYS_STATE",
            "fields": {
                "vtol_state": {"type": "MAV_VTOL_STATE_MC"},
                "landed_state": {"type": landed_state},
            },
            "cursor": f"v1:{index}",
            "sim_clock": _CLOCK_DICT,
        }
        for index, landed_state in (
            (1, "MAV_LANDED_STATE_IN_AIR"),
            (2, "MAV_LANDED_STATE_FUTURE"),
        )
    ]
    monkeypatch.setattr(
        client,
        "_request_json",
        lambda *_args, **_kwargs: {
            "records": records,
            "next_cursor": "v1:2",
            "next_clock": _CLOCK_DICT,
        },
    )

    batch = client.read_messages("v1:0", "EXTENDED_SYS_STATE")

    assert batch.messages == (
        ExtendedSysState(vtol_state=MavVtolState.MC, landed_state=MavLandedState.IN_AIR),
        ExtendedSysState(
            vtol_state=MavVtolState.MC,
            landed_state=MavLandedState("MAV_LANDED_STATE_FUTURE"),
        ),
    )
    assert batch.messages[1].landed_state.name == "MAV_LANDED_STATE_FUTURE"


def test_decode_message_projects_bitmask_and_enumeration_fields() -> None:
    heartbeat = decode_message(
        RawMessage(
            "HEARTBEAT",
            {
                "mavtype": {"type": "MAV_TYPE_HELICOPTER"},
                "autopilot": {"type": "MAV_AUTOPILOT_ARDUPILOTMEGA"},
                "base_mode": "MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
                "custom_mode": 4,
                "system_status": {"type": "MAV_STATE_ACTIVE"},
                "mavlink_version": 3,
            },
        )
    )
    ekf = decode_message(
        RawMessage(
            "EKF_STATUS_REPORT",
            {"flags": "EKF_ATTITUDE | EKF_VELOCITY_VERT | EKF_SOMETHING_NEW"},
        )
    )
    ack = decode_message(
        RawMessage(
            "COMMAND_ACK",
            {
                "command": {"type": "MAV_CMD_DO_SET_MODE"},
                "result": {"type": "MAV_RESULT_ACCEPTED"},
            },
        )
    )

    assert heartbeat == Heartbeat(
        mavtype=MavType.HELICOPTER,
        autopilot=MavAutopilot.ARDUPILOTMEGA,
        base_mode=frozenset({MavModeFlag.SAFETY_ARMED, MavModeFlag.CUSTOM_MODE_ENABLED}),
        custom_mode=4,
        system_status=MavState.ACTIVE,
        mavlink_version=3,
    )
    assert ekf == EkfStatusReport(
        flags=frozenset(
            {
                EkfStatusFlags.ATTITUDE,
                EkfStatusFlags.VELOCITY_VERT,
                EkfStatusFlags("EKF_SOMETHING_NEW"),
            }
        )
    )
    assert ack == CommandAck(command=MavCmd.DO_SET_MODE, result=MavResult.ACCEPTED)


def test_decode_message_projects_an_empty_bitmask_to_an_empty_set() -> None:
    heartbeat = decode_message(
        RawMessage(
            "HEARTBEAT",
            {
                "mavtype": {"type": "MAV_TYPE_GCS"},
                "autopilot": {"type": "MAV_AUTOPILOT_INVALID"},
                "base_mode": "",
                "custom_mode": 0,
                "system_status": {"type": "MAV_STATE_STANDBY"},
            },
        )
    )

    assert heartbeat.base_mode == frozenset()


def test_decode_message_keeps_sensor_flags_including_unknown_names() -> None:
    status = decode_message(
        RawMessage(
            "SYS_STATUS",
            {
                "onboard_control_sensors_present": (
                    "MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS"
                    " | MAV_SYS_STATUS_SENSOR_FUTURE"
                ),
                "onboard_control_sensors_enabled": "",
                "onboard_control_sensors_health": "MAV_SYS_STATUS_SENSOR_GPS",
            },
        )
    )

    assert MavSysStatusSensor.SENSOR_MOTOR_OUTPUTS in status.onboard_control_sensors_present
    assert (
        MavSysStatusSensor("MAV_SYS_STATUS_SENSOR_FUTURE")
        in status.onboard_control_sensors_present
    )
    assert status.onboard_control_sensors_enabled == frozenset()
    assert status.onboard_control_sensors_health == {MavSysStatusSensor.SENSOR_GPS}


def test_generated_messages_encode_enumerations_and_bitmasks_for_requests() -> None:
    message = ParamSet(
        target_system=1,
        target_component=1,
        param_id="RAWES_MODE",
        param_value=2.0,
        param_type=MavParamType.INT32,
    )

    assert message.to_request() == (
        "PARAM_SET",
        {
            "target_system": 1,
            "target_component": 1,
            "param_id": "RAWES_MODE",
            "param_value": 2.0,
            "param_type": {"type": "MAV_PARAM_TYPE_INT32"},
        },
    )
    assert encode_enum(MavSeverity.INFO) == {"type": "MAV_SEVERITY_INFO"}
    assert Heartbeat(
        mavtype=MavType.GCS,
        autopilot=MavAutopilot.INVALID,
        base_mode=frozenset({MavModeFlag.SAFETY_ARMED, MavModeFlag.CUSTOM_MODE_ENABLED}),
        custom_mode=0,
        system_status=MavState.ACTIVE,
    ).to_request()[1]["base_mode"] == (
        "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED"
    )


def test_client_propagates_parameter_operation_timeouts(monkeypatch) -> None:
    client = LinkHubClient()
    requests = []

    def request(method, path, body=None, **kwargs):
        requests.append((method, path, body, kwargs))
        return {"value": 12.5}

    monkeypatch.setattr(client, "_request_json", request)

    assert client.get_param("test_param", timeout=2.5) == 12.5
    assert client.set_param("test_param", 12.5, timeout=4.0)

    assert requests[0] == (
        "GET",
        "/v1/mavlink/parameters/TEST_PARAM?timeout_ms=2500",
        None,
        {"timeout": 3.5},
    )
    assert requests[1][0:2] == (
        "PUT",
        "/v1/mavlink/parameters/TEST_PARAM",
    )
    assert requests[1][2]["timeout_ms"] == 4_000
    assert requests[1][2]["type"] == {"type": "MAV_PARAM_TYPE_REAL32"}
    assert requests[1][3] == {"timeout": 5.0}


def test_client_encodes_and_decodes_parameter_types(monkeypatch) -> None:
    client = LinkHubClient()
    requests = []

    def request(method, path, body=None, **kwargs):
        requests.append((method, path, body))
        return {
            "parameters": [
                {
                    "name": "RAWES_MODE",
                    "value": 2.0,
                    "type": {"type": "MAV_PARAM_TYPE_INT8"},
                    "index": 1,
                    "count": 5,
                }
            ]
        }

    monkeypatch.setattr(client, "_request_json", request)

    records = client.set_params(
        [
            {"name": "RAWES_MODE", "value": 2.0, "type": MavParamType.INT8},
            {"name": "RAWES_THR", "value": 0.5},
        ]
    )

    assert requests[0][2]["parameters"] == [
        {"name": "RAWES_MODE", "value": 2.0, "type": {"type": "MAV_PARAM_TYPE_INT8"}},
        {"name": "RAWES_THR", "value": 0.5},
    ]
    assert records["RAWES_MODE"]["type"] is MavParamType.INT8


def test_client_flushes_journal_and_returns_cursor(monkeypatch) -> None:
    client = LinkHubClient()
    requests = []

    def request(method, path, body=None, **kwargs):
        requests.append((method, path, body, kwargs))
        return {"cursor": "v1:42"}

    monkeypatch.setattr(client, "_request_json", request)

    assert client.flush_journal() == "v1:42"
    assert requests == [("POST", "/v1/journal/flush", {}, {})]


def test_diagnostic_protocol_round_trip() -> None:
    event = DiagnosticEvent(
        schema_version=1,
        run_id=UUID("aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"),
        source="groundstation",
        source_instance="gcs-1",
        source_sequence=12,
        source_wall_time_ns=100,
        source_monotonic_ns=90,
        sim_time_ns=80,
        sim_time_quality=SimTimeQuality.EXACT,
        level=DiagnosticLevel.WARNING,
        category="operation",
        event="operation.failed",
        message="Command failed",
        correlation_id=UUID("bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb"),
        causation_id=None,
        fields={"command": 400},
        related_records=[41, 42],
    )
    wire_record = {
        "sequence": 43,
        "cursor": "v1:43",
        "ingest_time_ns": 110,
        "correlation_id": str(event.correlation_id),
        "sim_clock": _CLOCK_DICT,
        "kind": "diagnostic.event",
        "data": event.to_dict(),
    }

    decoded = DiagnosticRecord.from_dict(wire_record)

    assert decoded.event == event
    assert decoded.cursor == "v1:43"
    assert decoded.correlation_id == event.correlation_id
    assert decoded.sim_clock.time_boot_ms == 12_345
