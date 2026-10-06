"""Bounded stress waits follow finite batches without extending their deadline."""
from __future__ import annotations

import urllib.parse

import pytest

from scripts import linkhub_stress


def _wait(monkeypatch, batches: list[tuple[float, dict]]) -> tuple[bool, list[dict]]:
    now = [10.0]
    requests = []

    def request(server, method, path, *, timeout):
        query = urllib.parse.parse_qs(urllib.parse.urlsplit(path).query)
        requests.append({"query": query, "timeout": timeout})
        elapsed, batch = batches.pop(0)
        now[0] += elapsed
        return batch

    monkeypatch.setattr(linkhub_stress.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(linkhub_stress, "_json_request", request)
    return linkhub_stress._bounded_attitude_wait("http://linkhub", "v1:10"), requests


def test_empty_progress_batch_does_not_fail_wait(monkeypatch) -> None:
    result, requests = _wait(monkeypatch, [
        (0.25, {"records": [], "next_cursor": "v1:11"}),
        (0.25, {"records": [{"cursor": "v1:12"}], "next_cursor": "v1:12"}),
    ])
    assert result
    assert requests[1]["query"]["after"] == ["v1:11"]
    assert requests[1]["query"]["wait_ms"] == ["1750"]
    assert requests[1]["timeout"] == 2.75


def test_empty_unchanged_batch_can_be_followed_by_telemetry(monkeypatch) -> None:
    result, _ = _wait(monkeypatch, [
        (0.1, {"records": [], "next_cursor": "v1:10"}),
        (0.1, {"records": [{"cursor": "v1:11"}], "next_cursor": "v1:11"}),
    ])
    assert result


def test_empty_batches_do_not_restart_deadline(monkeypatch) -> None:
    result, requests = _wait(monkeypatch, [
        (1.0, {"records": [], "next_cursor": "v1:11"}),
        (1.0, {"records": [], "next_cursor": "v1:12"}),
    ])
    assert not result
    assert len(requests) == 2
    assert requests[1]["query"]["wait_ms"] == ["1000"]


def test_late_telemetry_does_not_pass(monkeypatch) -> None:
    result, _ = _wait(monkeypatch, [
        (2.1, {"records": [{"cursor": "v1:11"}], "next_cursor": "v1:11"}),
    ])
    assert not result


def test_http_failure_is_not_hidden(monkeypatch) -> None:
    def request(*args, **kwargs):
        raise TimeoutError("HTTP request timed out")

    monkeypatch.setattr(linkhub_stress, "_json_request", request)
    with pytest.raises(TimeoutError, match="HTTP request timed out"):
        linkhub_stress._bounded_attitude_wait("http://linkhub", "v1:10")


@pytest.mark.parametrize("batch", [
    {"records": [], "next_cursor": "v1:9"},
    {"records": [{"cursor": "v1:10"}], "next_cursor": "v1:10"},
])
def test_invalid_cursor_progress_fails(monkeypatch, batch: dict) -> None:
    with pytest.raises(AssertionError, match="cursor"):
        _wait(monkeypatch, [(0.1, batch)])
