from uuid import UUID

import linkhub_client.client as client_module
from linkhub_client import (
    DiagnosticEvent,
    DiagnosticLevel,
    DiagnosticRecord,
    SimTimeQuality,
    LinkHubClient,
)


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
    stream_started = False

    monkeypatch.setattr(client, "_request_json", lambda *_args: next(statuses))
    monkeypatch.setattr(client_module.time, "sleep", lambda _seconds: None)

    def start_stream() -> None:
        nonlocal stream_started
        stream_started = True

    monkeypatch.setattr(client, "_start_receive_stream", start_stream)

    client.connect(timeout=1.0)

    assert stream_started
    assert client._target_system == 4
    assert client._target_component == 7
    assert client._cursor == "v1:9"


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
        "kind": "diagnostic.event",
        "data": event.to_dict(),
    }

    decoded = DiagnosticRecord.from_dict(wire_record)

    assert decoded.event == event
    assert decoded.cursor == "v1:43"
    assert decoded.correlation_id == event.correlation_id
