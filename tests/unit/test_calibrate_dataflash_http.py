from __future__ import annotations

import contextlib
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

import calibrate.linkhub as calibrate_linkhub
from linkhub_client.client import LinkHubClient, LinkHubError


class _DataFlashHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    content = b"\x01\x02dataflash"
    polls = 0
    started: list[dict] = []
    cancelled: list[str] = []
    final_state = "complete"

    def _reply(self, status: int, body: bytes, content_type: str = "application/json") -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _info(self, state: str) -> bytes:
        return json.dumps({"id": 7, "state": state, "error": "link lost"}).encode()

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", 0))
        type(self).started.append(json.loads(self.rfile.read(length)))
        self._reply(202, self._info("queued"))

    def do_DELETE(self) -> None:
        type(self).cancelled.append(self.path)
        self._reply(200, b'{"id": 7}')

    def do_GET(self) -> None:
        if self.path == "/v1/mavlink/transfers/7":
            type(self).polls += 1
            state = "running" if self.polls < 3 else self.final_state
            self._reply(200, self._info(state))
        elif self.path == "/v1/mavlink/transfers/7/content":
            self._reply(200, self.content, "application/octet-stream")
        elif self.path.startswith("/v1/mavlink/logs?"):
            self._reply(
                200,
                json.dumps({
                    "logs": [{"id": 4, "size": len(self.content), "time_utc": 123}]
                }).encode(),
            )
        else:
            self.send_error(404)

    def log_message(self, _format: str, *_args: object) -> None:
        pass


@contextlib.contextmanager
def _stub_session(final_state: str = "complete"):
    handler = _DataFlashHandler
    handler.polls = 0
    handler.started = []
    handler.cancelled = []
    handler.final_state = final_state
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        yield LinkHubClient(f"http://127.0.0.1:{server.server_port}")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)


def test_default_heartbeat_timeout_tolerates_usb_reboot(monkeypatch) -> None:
    now = 0.0
    stopped = []

    class Process:
        def poll(self):
            return None

    def monotonic() -> float:
        return now

    def sleep(duration: float) -> None:
        nonlocal now
        now += duration

    def service_status(_server: str, path: str) -> int | None:
        if path == "/health/ready" and now >= 16.0:
            return 200
        return None

    monkeypatch.setattr(calibrate_linkhub.time, "monotonic", monotonic)
    monkeypatch.setattr(calibrate_linkhub.time, "sleep", sleep)
    monkeypatch.setattr(calibrate_linkhub, "_service_status", service_status)
    monkeypatch.setattr(calibrate_linkhub, "_read_link_status", lambda _server: {})
    monkeypatch.setattr(
        calibrate_linkhub,
        "_start_linkhub",
        lambda *_args, **_kwargs: Process(),
    )
    monkeypatch.setattr(
        calibrate_linkhub,
        "_stop_process",
        lambda process: stopped.append(process),
    )

    with calibrate_linkhub.ensure_linkhub(
        calibrate_linkhub._DEFAULT_SERVER,
        connection="COM7",
        baud=115_200,
        motor_name_prefix=None,
    ):
        assert now >= 16.0

    assert len(stopped) == 1


def test_calibrate_lists_and_downloads_dataflash_over_http(tmp_path: Path) -> None:
    destination = tmp_path / "dataflash-4.BIN"
    seen: list[str] = []
    with _stub_session() as session:
        logs = session.list_logs(timeout=0.25)
        result = session.download_log(
            4,
            destination,
            timeout=0.25,
            max_retries=3,
            progress=lambda info: seen.append(info["state"]),
        )

    assert logs == [{"id": 4, "size": 11, "time_utc": 123}]
    assert result == destination
    assert destination.read_bytes() == _DataFlashHandler.content
    assert not (tmp_path / "dataflash-4.BIN.part").exists()
    assert _DataFlashHandler.started == [
        {"kind": "log_download", "log_id": 4, "packet_timeout_ms": 250, "max_retries": 3}
    ]
    assert seen[0] == "queued" and seen[-1] == "complete"
    assert _DataFlashHandler.cancelled == ["/v1/mavlink/transfers/7"]


def test_file_download_requests_an_ftp_transfer(tmp_path: Path) -> None:
    destination = tmp_path / "nested" / "log.bin"
    with _stub_session() as session:
        size = session.download_file("/APM/LOGS/1.BIN", destination, stall_timeout=5.0)

    assert size == len(_DataFlashHandler.content)
    assert destination.read_bytes() == _DataFlashHandler.content
    assert _DataFlashHandler.started == [{
        "kind": "ftp_download",
        "path": "/APM/LOGS/1.BIN",
        "verify_crc": True,
        "stall_timeout_ms": 5000,
    }]


def test_failed_transfer_raises_and_leaves_no_file(tmp_path: Path) -> None:
    destination = tmp_path / "dataflash-4.BIN"
    with _stub_session("failed") as session:
        with pytest.raises(LinkHubError, match="link lost"):
            session.download_log(4, destination)

    assert not destination.exists()
    assert not (tmp_path / "dataflash-4.BIN.part").exists()
    assert _DataFlashHandler.cancelled == ["/v1/mavlink/transfers/7"]


def test_connect_preserves_valid_zero_target_component(monkeypatch) -> None:
    session = LinkHubClient()
    monkeypatch.setattr(
        session,
        "_request_json",
        lambda *_args, **_kwargs: {
            "connected": True,
            "ready": True,
            "target_system": 1,
            "target_component": 0,
            "cursor": "segment:1",
        },
    )

    session.connect()

    assert session._target_system == 1
    assert session._target_component == 0
