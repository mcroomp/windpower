from uuid import UUID

import linkhub_client.client as client_module
from linkhub_client import (
    DiagnosticEvent,
    DiagnosticLevel,
    DiagnosticRecord,
    SimClock,
    SimTimeQuality,
    LinkHubClient,
)
from linkhub_client.messages import (
    ExtendedSysState,
    MavLandedState,
    MavVtolState,
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
        "vehicle_status",
        lambda: {
            "base_mode": client_module.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            "sim_clock": _CLOCK_DICT,
        },
    )

    assert client.sim_now() == 12.345
    assert client.is_armed


def test_client_reads_finite_server_filtered_message_batch(monkeypatch) -> None:
    client = LinkHubClient()
    record = {
        "received_time": "2026-01-01T00:00:00Z",
        "received_time_ns": 1,
        "direction": "rx",
        "system_id": 1,
        "component_id": 1,
        "message": "STATUSTEXT",
        "fields": {"text": "ready", "severity": 6},
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
            "fields": {"vtol_state": 3, "landed_state": landed_state},
            "cursor": f"v1:{index}",
            "sim_clock": _CLOCK_DICT,
        }
        for index, landed_state in ((1, 2), (2, 99))
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
        ExtendedSysState(MavVtolState.MC, MavLandedState.IN_AIR),
        ExtendedSysState(MavVtolState.MC, MavLandedState(99)),
    )
    assert batch.messages[1].landed_state.name == "UNKNOWN_99"


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
    assert requests[1][3] == {"timeout": 5.0}


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
