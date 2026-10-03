from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from linkhub_client.client import LinkHubClient


class _DataFlashHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    content = b"\x01\x02dataflash"

    def do_GET(self) -> None:
        if self.path.startswith("/v1/mavlink/logs/4?"):
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(self.content)))
            self.send_header(
                "X-LinkHub-After-Cursor",
                "20260930T120000.000000000Z.tlh:42",
            )
            self.end_headers()
            self.wfile.write(self.content)
            return
        if self.path.startswith("/v1/mavlink/logs?"):
            body = json.dumps({
                "logs": [{"id": 4, "size": len(self.content), "time_utc": 123}]
            }).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def log_message(self, _format: str, *_args: object) -> None:
        pass


def test_calibrate_lists_and_downloads_dataflash_over_http(tmp_path: Path) -> None:
    server = ThreadingHTTPServer(("127.0.0.1", 0), _DataFlashHandler)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    session = LinkHubClient(f"http://127.0.0.1:{server.server_port}")
    destination = tmp_path / "dataflash-4.BIN"
    try:
        logs = session.list_logs(timeout=0.25)
        result = session.download_log(
            4,
            destination,
            timeout=0.25,
            max_retries=3,
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)

    assert logs == [{"id": 4, "size": 11, "time_utc": 123}]
    assert result == destination
    assert destination.read_bytes() == _DataFlashHandler.content
    assert session._last_operation_cursor == "20260930T120000.000000000Z.tlh:42"
    assert not (tmp_path / "dataflash-4.BIN.part").exists()


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
