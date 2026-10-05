#!/usr/bin/env python3
"""Exercise LinkHub's finite HTTP batches and lossless journal under load."""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.parse
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass
from typing import Any

from linkhub_client import LinkHubClient


@dataclass(frozen=True)
class StressResult:
    malformed_requests_rejected: int
    timeout_requests: int
    status_requests_during_timeout: int
    send_rounds: int
    sends_archived: int
    bounded_wait_requests: int
    readers: int
    attitude_records: int
    attitude_rate_hz: float
    final_cursor: str


def _json_request(
    server: str,
    method: str,
    path: str,
    body: Any | None = None,
    *,
    timeout: float = 15.0,
) -> Any:
    content = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(
        server.rstrip("/") + path,
        data=content,
        headers={"Content-Type": "application/json"} if content is not None else {},
        method=method,
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def _expect_http_error(
    server: str,
    method: str,
    path: str,
    content: bytes | None,
    expected: set[int],
) -> None:
    request = urllib.request.Request(
        server.rstrip("/") + path,
        data=content,
        headers={"Content-Type": "application/json"},
        method=method,
    )
    try:
        urllib.request.urlopen(request, timeout=5.0)
    except urllib.error.HTTPError as exc:
        if exc.code not in expected:
            raise AssertionError(
                f"{method} {path} returned HTTP {exc.code}, expected {sorted(expected)}"
            ) from exc
    else:
        raise AssertionError(f"{method} {path} unexpectedly succeeded")


def _cursor_sequence(cursor: str) -> int:
    version, separator, sequence = cursor.partition(":")
    if version != "v1" or not separator:
        raise ValueError(f"unsupported cursor {cursor!r}")
    return int(sequence)


def _read_message_cursors(
    server: str,
    after: str,
    through: int,
    *,
    direction: str,
    message: str,
) -> list[str]:
    cursors: list[str] = []
    cursor = after
    while _cursor_sequence(cursor) < through:
        query = urllib.parse.urlencode({
            "after": cursor,
            "direction": direction,
            "messages": message,
            "limit": 10_000,
        })
        batch = _json_request(
            server,
            "GET",
            f"/v1/mavlink/messages?{query}",
            timeout=30.0,
        )
        for record in batch["records"]:
            if _cursor_sequence(str(record["cursor"])) <= through:
                cursors.append(str(record["cursor"]))
        next_cursor = str(batch["next_cursor"])
        if next_cursor == cursor:
            break
        cursor = next_cursor
    return cursors


def _read_attitudes(server: str, after: str, through: int) -> list[str]:
    return _read_message_cursors(
        server,
        after,
        through,
        direction="rx",
        message="ATTITUDE",
    )


def _bounded_attitude_wait(server: str, after: str) -> bool:
    query = urllib.parse.urlencode({
        "after": after,
        "direction": "rx",
        "messages": "ATTITUDE",
        "wait_ms": 2_000,
        "limit": 1,
    })
    batch = _json_request(
        server,
        "GET",
        f"/v1/mavlink/messages?{query}",
        timeout=3.0,
    )
    return bool(batch["records"]) and str(batch["next_cursor"]) != after


def _send_named_value(server: str, index: int) -> str:
    result = _json_request(
        server,
        "POST",
        "/v1/mavlink/messages",
        {
            "message": "NAMED_VALUE_FLOAT",
            "fields": {
                "time_boot_ms": 0,
                "name": "LHSTRESS",
                "value": float(index),
            },
        },
    )
    return str(result["after_cursor"])


def run_stress(
    server: str,
    *,
    readers: int,
    duration: float,
    attitude_rate: float,
    send_burst: int,
    send_rounds: int = 1,
    malformed_burst: int = 3,
    timeout_requests: int = 1,
) -> StressResult:
    if readers < 2:
        raise ValueError("readers must be at least 2")
    if min(
        duration,
        attitude_rate,
        send_burst,
        send_rounds,
        malformed_burst,
        timeout_requests,
    ) <= 0:
        raise ValueError("all stress dimensions must be positive")

    client = LinkHubClient(server)
    client.connect()
    original_interval = _json_request(
        server, "GET", "/v1/mavlink/message-rates/ATTITUDE"
    )
    interval_us = int(original_interval["interval_us"])
    restore_rate: float | None = (
        1_000_000.0 / interval_us
        if interval_us > 0
        else 0.0
        if interval_us < 0
        else None
    )

    try:
        malformed = [
            (
                "POST",
                "/v1/mavlink/messages",
                b"{",
                {400, 422},
            ),
            (
                "POST",
                "/v1/mavlink/messages",
                json.dumps({"message": "NOT_A_MAVLINK_MESSAGE", "fields": {}}).encode(),
                {400},
            ),
            (
                "GET",
                "/v1/mavlink/messages?after=invalid",
                None,
                {400},
            ),
        ]
        with ThreadPoolExecutor(max_workers=min(malformed_burst, 64)) as pool:
            rejected = list(
                pool.map(
                    lambda index: _expect_http_error(
                        server,
                        *malformed[index % len(malformed)],
                    ),
                    range(malformed_burst),
                )
            )
        if len(rejected) != malformed_burst:
            raise AssertionError("malformed-request burst did not complete")

        with ThreadPoolExecutor(max_workers=timeout_requests + 1) as pool:
            missing = [
                pool.submit(
                    LinkHubClient(server).get_param,
                    f"LH_MISSING_{index}",
                    2.0,
                )
                for index in range(timeout_requests)
            ]
            responsive = 0
            while not all(future.done() for future in missing):
                status = _json_request(server, "GET", "/v1/mavlink/status")
                if status.get("connected"):
                    responsive += 1
                time.sleep(0.05)
            if any(future.result() is not None for future in missing):
                raise AssertionError("missing parameter unexpectedly returned a value")
        if responsive == 0:
            raise AssertionError("status was not responsive during parameter timeout")

        archived = 0
        for round_index in range(send_rounds):
            send_start = _json_request(server, "GET", "/v1/mavlink/status")["cursor"]
            with ThreadPoolExecutor(max_workers=min(send_burst, 64)) as pool:
                send_cursors = list(
                    pool.map(
                        lambda index: _send_named_value(
                            server, round_index * send_burst + index
                        ),
                        range(send_burst),
                    )
                )
            expected_cursors = set(send_cursors)
            if len(expected_cursors) != send_burst:
                raise AssertionError("concurrent sends did not receive unique journal cursors")
            send_end = max(_cursor_sequence(cursor) for cursor in expected_cursors)
            for replay_delay in (0.0, 0.075, 1.1):
                if replay_delay:
                    time.sleep(replay_delay)
                replayed = set(
                    _read_message_cursors(
                        server,
                        send_start,
                        send_end,
                        direction="tx",
                        message="NAMED_VALUE_FLOAT",
                    )
                )
                missing_cursors = expected_cursors - replayed
                unexpected_cursors = replayed - expected_cursors
                if missing_cursors or unexpected_cursors:
                    raise AssertionError(
                        "TX replay mismatch after "
                        f"{replay_delay:.3f}s: missing={len(missing_cursors)} "
                        f"unexpected={len(unexpected_cursors)}"
                    )
            archived += len(expected_cursors)

        for transition_rate in (50.0, 200.0, 100.0, attitude_rate):
            client.set_message_rates({"ATTITUDE": transition_rate})
        wait_start = _json_request(server, "GET", "/v1/mavlink/status")["cursor"]
        wait_count = readers * 2
        with ThreadPoolExecutor(max_workers=wait_count) as pool:
            wait_results = list(
                pool.map(
                    lambda _: _bounded_attitude_wait(server, wait_start),
                    range(wait_count),
                )
            )
        completed_waits = sum(wait_results)
        if completed_waits != wait_count:
            raise AssertionError(
                f"only {completed_waits}/{wait_count} bounded waits received telemetry"
            )

        capture_start = _json_request(server, "GET", "/v1/mavlink/status")["cursor"]
        time.sleep(duration)
        capture_end = _json_request(server, "GET", "/v1/mavlink/status")["cursor"]
        capture_end_sequence = _cursor_sequence(capture_end)
        with ThreadPoolExecutor(max_workers=readers) as pool:
            streams = list(
                pool.map(
                    lambda _: _read_attitudes(
                        server, capture_start, capture_end_sequence
                    ),
                    range(readers),
                )
            )
        first = streams[0]
        if not first:
            raise AssertionError("no ATTITUDE records captured")
        if any(stream != first for stream in streams[1:]):
            raise AssertionError("lossless readers received different ATTITUDE cursors")
        observed_rate = len(first) / duration
        if observed_rate < attitude_rate * 0.7:
            raise AssertionError(
                f"ATTITUDE rate {observed_rate:.1f} Hz is below requested "
                f"{attitude_rate:.1f} Hz"
            )

        final_status = _json_request(server, "GET", "/v1/mavlink/status")
        if not final_status.get("connected") or not final_status.get("ready"):
            raise AssertionError(f"LinkHub unhealthy after stress: {final_status}")
        return StressResult(
            malformed_requests_rejected=malformed_burst,
            timeout_requests=timeout_requests,
            status_requests_during_timeout=responsive,
            send_rounds=send_rounds,
            sends_archived=archived,
            bounded_wait_requests=completed_waits,
            readers=readers,
            attitude_records=len(first),
            attitude_rate_hz=observed_rate,
            final_cursor=str(final_status["cursor"]),
        )
    finally:
        try:
            client.set_message_rates({"ATTITUDE": restore_rate})
        finally:
            client.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="http://127.0.0.1:8999")
    parser.add_argument("--readers", type=int, default=8)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--attitude-rate", type=float, default=100.0)
    parser.add_argument("--send-burst", type=int, default=32)
    parser.add_argument("--send-rounds", type=int, default=1)
    parser.add_argument("--malformed-burst", type=int, default=3)
    parser.add_argument("--timeout-requests", type=int, default=1)
    args = parser.parse_args()
    result = run_stress(
        args.server,
        readers=args.readers,
        duration=args.duration,
        attitude_rate=args.attitude_rate,
        send_burst=args.send_burst,
        send_rounds=args.send_rounds,
        malformed_burst=args.malformed_burst,
        timeout_requests=args.timeout_requests,
    )
    print(json.dumps(asdict(result), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
