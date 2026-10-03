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

from serial.tools import list_ports

from .constants import _FALLBACK_BAUDS, _REPO_ROOT

_DEFAULT_SERVER = "http://127.0.0.1:8999"


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
    binary = Path(_REPO_ROOT, "linkhub", "target", "release", name)
    if not binary.is_file():
        raise FileNotFoundError(
            f"LinkHub binary not found at {binary}; run setup.cmd first"
        )
    return binary


def _connection_candidates(connection: str | None, baud: int | None) -> list[tuple[str, int]]:
    if connection:
        return [(connection, baud or 115_200)]
    ports = sorted(
        list_ports.comports(),
        key=lambda port: (port.vid is None, port.device),
    )
    bauds = (baud,) if baud else _FALLBACK_BAUDS
    return [(port.device, candidate_baud) for port in ports for candidate_baud in bauds]


def _stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def _start_candidate(
    connection: str,
    baud: int,
    *,
    motor_name_prefix: str | None,
) -> subprocess.Popen[bytes]:
    command = [
        str(_linkhub_binary()),
        "serve",
        "--connection",
        connection,
        "--baud",
        str(baud),
        "--data-dir",
        str(Path(_REPO_ROOT, "simulation", "logs", "linkhub")),
    ]
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
    heartbeat_timeout: float = 15.0,
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

    candidates = _connection_candidates(connection, baud)
    if not candidates:
        raise RuntimeError("no serial ports were found for LinkHub")

    failures: list[str] = []
    process: subprocess.Popen[bytes] | None = None
    try:
        for candidate, candidate_baud in candidates:
            print(f"Starting LinkHub on {candidate} at {candidate_baud} baud ...")
            process = _start_candidate(
                candidate,
                candidate_baud,
                motor_name_prefix=motor_name_prefix,
            )
            deadline = time.monotonic() + heartbeat_timeout
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    failures.append(
                        f"{candidate}@{candidate_baud}: exited with {process.returncode}"
                    )
                    break
                if _service_status(server, "/health/ready") == 200:
                    print(f"LinkHub connected on {candidate} at {candidate_baud} baud.")
                    yield
                    return
                time.sleep(0.1)
            else:
                status = _read_link_status(server)
                failures.append(
                    f"{candidate}@{candidate_baud}: "
                    f"{status.get('error') or 'heartbeat timeout'}"
                )
            _stop_process(process)
            process = None
    finally:
        if process is not None:
            _stop_process(process)

    raise RuntimeError("LinkHub could not find a Pixhawk: " + "; ".join(failures))


def _read_link_status(server: str) -> dict[str, object]:
    try:
        with urllib.request.urlopen(f"{server}/v1/mavlink/status", timeout=0.5) as response:
            result = json.load(response)
    except (urllib.error.URLError, TimeoutError, ValueError):
        return {}
    return result if isinstance(result, dict) else {}
