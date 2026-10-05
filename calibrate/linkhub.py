from __future__ import annotations

import json
import os
import subprocess
import time
import urllib.error
import urllib.request
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

from .constants import _REPO_ROOT

_DEFAULT_SERVER = "http://127.0.0.1:8999"
# LinkHub scans every serial port against every candidate baud rate (5 by
# default) at up to 3 s per probe, so the wait here must cover a full sweep,
# not just one candidate.
_DEFAULT_HEARTBEAT_TIMEOUT_S = 60.0


def _service_status(server: str, path: str) -> int | None:
    try:
        with urllib.request.urlopen(f"{server}{path}", timeout=0.5) as response:
            response.read()
            return response.status
    except urllib.error.HTTPError as exc:
        exc.read()
        return exc.code
    except (urllib.error.URLError, TimeoutError):
        return None


def _linkhub_binary() -> Path:
    name = "linkhub.exe" if os.name == "nt" else "linkhub"
    linkhub_dir = Path(_REPO_ROOT, "linkhub")
    binary = linkhub_dir / "target" / "release" / name
    if not binary.is_file():
        raise FileNotFoundError(
            f"LinkHub binary not found at {binary}; run setup.cmd first"
        )
    source_paths = [
        linkhub_dir / "Cargo.toml",
        linkhub_dir / "Cargo.lock",
        *linkhub_dir.joinpath("src").glob("*.rs"),
    ]
    if any(path.stat().st_mtime_ns > binary.stat().st_mtime_ns for path in source_paths):
        raise RuntimeError(
            f"LinkHub binary at {binary} is older than its source; run setup.cmd first"
        )
    return binary


def _connection_label(connection: str | None, baud: int | None) -> str:
    label = connection or "auto"
    if baud:
        label += f"@{baud}"
    return label


def _stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def _start_linkhub(
    connection: str | None,
    baud: int | None,
    *,
    motor_name_prefix: str | None,
) -> subprocess.Popen[bytes]:
    command = [
        str(_linkhub_binary()),
        "serve",
        "--data-dir",
        str(Path(_REPO_ROOT, "simulation", "logs", "linkhub")),
    ]
    if connection:
        command.extend(("--connection", connection))
    if baud:
        command.extend(("--baud", str(baud)))
    if motor_name_prefix:
        command.extend(("--motor-name-prefix", motor_name_prefix))
    return subprocess.Popen(command, cwd=_REPO_ROOT)


@contextmanager
def ensure_linkhub(
    server: str,
    *,
    connection: str | None,
    baud: int | None,
    motor_name_prefix: str | None,
    heartbeat_timeout: float = _DEFAULT_HEARTBEAT_TIMEOUT_S,
) -> Iterator[None]:
    if _service_status(server, "/health/ready") == 200:
        yield
        return
    if _service_status(server, "/health/live") is not None:
        raise RuntimeError(f"LinkHub at {server} is running but not ready")
    if server.rstrip("/") != _DEFAULT_SERVER:
        raise RuntimeError(
            "automatic LinkHub startup is available only for "
            f"{_DEFAULT_SERVER}; start the configured remote server explicitly"
        )

    # LinkHub itself discovers the serial port: `connection`/`baud` only
    # restrict which candidates it scans (unset means scan everything), and
    # it keeps rescanning on its own if the link drops. Not being connected
    # yet is an ordinary state here, not a failure, so we just wait.
    print(f"Starting LinkHub on {_connection_label(connection, baud)} ...")
    process = _start_linkhub(connection, baud, motor_name_prefix=motor_name_prefix)
    try:
        deadline = time.monotonic() + heartbeat_timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(
                    f"LinkHub exited with {process.returncode} before finding a Pixhawk"
                )
            if _service_status(server, "/health/ready") == 200:
                status = _read_link_status(server)
                port, found_baud = status.get("port"), status.get("baud")
                if port and found_baud:
                    print(f"LinkHub connected on {port} at {found_baud} baud.")
                else:
                    print("LinkHub connected.")
                yield
                return
            time.sleep(0.1)
        status = _read_link_status(server)
        raise RuntimeError(
            "LinkHub could not find a Pixhawk: "
            + (status.get("error") or "heartbeat timeout")
        )
    finally:
        _stop_process(process)


def _read_link_status(server: str) -> dict[str, object]:
    try:
        with urllib.request.urlopen(f"{server}/v1/mavlink/status", timeout=0.5) as response:
            result = json.load(response)
    except (urllib.error.URLError, TimeoutError, ValueError):
        return {}
    return result if isinstance(result, dict) else {}
