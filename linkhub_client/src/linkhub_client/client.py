from __future__ import annotations

import json
import collections
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Iterator
from dataclasses import fields
from pathlib import Path
from typing import Any

from . import mav_constants as mavlink
from .messages import (
    CommandLong,
    NamedValueFloat,
    ParamSet,
    RawMessage,
    RequestDataStream,
)
from .records import DiagnosticEvent, DiagnosticRecord, TelemetryRecord


class LinkHubError(RuntimeError):
    pass


class WallClock:
    pass


class LinkHubClient:
    """Dependency-free client for LinkHub's HTTP v1 API."""

    def __init__(
        self,
        address: str = "http://127.0.0.1:8999",
        **_ignored: Any,
    ) -> None:
        self._base_url = address.rstrip("/")
        self._target_system = 1
        self._target_component = 1
        self._cursor: str | None = None
        self._last_operation_cursor: str | None = None
        self._cursor_lock = threading.Lock()
        self._receive_cv = threading.Condition()
        self._receive_queue: collections.deque[RawMessage] = collections.deque()
        self._receive_stop = threading.Event()
        self._receive_thread: threading.Thread | None = None
        self._receive_response = None
        self._receive_error: BaseException | None = None
        self._sim_time_s = 0.0
        self._armed = False
        self._mavlog_path: Path | None = None
        self._mavlog_lock = threading.Lock()
        self._mavlog_file = None

    def connect(self, timeout: float = 15.0) -> None:
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                status = self._request_json("GET", "/v1/mavlink/status")
                if not status.get("connected"):
                    raise LinkHubError(status.get("error") or "MAVLink is disconnected")
                if not status.get("ready"):
                    raise LinkHubError(
                        status.get("error") or "MAVLink is awaiting heartbeat"
                    )
                target_system = status.get("target_system")
                target_component = status.get("target_component")
                self._target_system = int(
                    1 if target_system is None else target_system
                )
                self._target_component = int(
                    1 if target_component is None else target_component
                )
                self._cursor = status.get("cursor")
                self._start_receive_stream()
                return
            except (OSError, LinkHubError) as exc:
                last_error = exc
                time.sleep(0.25)
        raise TimeoutError(
            f"Could not connect to LinkHub at {self._base_url}"
        ) from last_error

    def close(self) -> None:
        self._stop_receive_stream()
        self.stop_mavlog()

    def linkhub_status(self) -> dict[str, Any]:
        """Return LinkHub service, run, journal, and transport status."""
        return dict(self._request_json("GET", "/v1/status"))

    def send_diagnostics(
        self,
        events: list[DiagnosticEvent],
    ) -> dict[str, Any]:
        """Submit one idempotent batch of structured diagnostic events."""
        if not events:
            raise ValueError("events must contain at least one event")
        return dict(
            self._request_json(
                "POST",
                "/v1/diagnostics/events",
                {"events": [event.to_dict() for event in events]},
            )
        )

    def iter_diagnostics(
        self,
        *,
        after: str | None = None,
        follow: bool = False,
        source: str | None = None,
        event: str | None = None,
        level: str | None = None,
    ) -> Iterator[DiagnosticRecord]:
        """Read or follow LinkHub diagnostics from an opaque journal cursor."""
        query = {
            key: value
            for key, value in {
                "after": after,
                "follow": "true" if follow else "false",
                "source": source,
                "event": event,
                "level": level,
            }.items()
            if value is not None
        }
        request = urllib.request.Request(
            self._base_url
            + "/v1/diagnostics/events?"
            + urllib.parse.urlencode(query)
        )
        try:
            with urllib.request.urlopen(request, timeout=60.0) as response:
                for line in response:
                    if line.strip():
                        yield DiagnosticRecord.from_dict(json.loads(line))
        except urllib.error.HTTPError as exc:
            raise LinkHubError(_http_error_message(exc)) from exc

    def send_message(self, message: Any) -> str:
        if hasattr(message, "to_request"):
            message_name, payload = message.to_request()
        else:
            message_name = message.MAVLINK_TYPE
            payload = {
                field.name: getattr(message, field.name)
                for field in fields(message)
            }
        return self._send_raw(message_name, payload)

    def command(
        self,
        command: int,
        params: list[float] | None = None,
        *,
        timeout: float = 3.0,
    ) -> dict[str, Any]:
        result = self._request_json(
            "POST",
            "/v1/mavlink/commands",
            {
                "command": command,
                "params": params or [],
                "target_system": self._target_system,
                "target_component": self._target_component,
                "timeout_ms": round(timeout * 1000),
            },
        )
        self._last_operation_cursor = result.get("after_cursor")
        return result

    def get_param(self, name: str, timeout: float = 3.0) -> float | None:
        query = urllib.parse.urlencode({"timeout_ms": round(timeout * 1000)})
        try:
            result = self._request_json(
                "GET",
                f"/v1/mavlink/parameters/{urllib.parse.quote(name.upper())}?{query}",
                timeout=timeout + 1.0,
            )
        except LinkHubError:
            return None
        self._last_operation_cursor = result.get("after_cursor")
        return float(result["value"])

    def set_param(
        self,
        name: str,
        value: float,
        *,
        timeout: float = 3.0,
        param_type: int | None = None,
        **_ignored: Any,
    ) -> bool:
        try:
            result = self._request_json(
                "PUT",
                f"/v1/mavlink/parameters/{urllib.parse.quote(name.upper())}",
                {
                    "value": value,
                    "type": (
                        mavlink.MAV_PARAM_TYPE_INT32
                        if param_type is None and isinstance(value, int)
                        else mavlink.MAV_PARAM_TYPE_REAL32
                        if param_type is None
                        else param_type
                    ),
                    "timeout_ms": round(timeout * 1000),
                },
                timeout=timeout + 1.0,
            )
        except LinkHubError:
            return False
        self._last_operation_cursor = result.get("after_cursor")
        return abs(float(result["value"]) - float(value)) < 1e-4

    def fetch_all_params(self, timeout: float = 15.0) -> dict[str, float]:
        return {
            name: float(record["value"])
            for name, record in self.fetch_all_param_records(timeout).items()
        }

    def fetch_all_param_records(
        self,
        timeout: float = 15.0,
    ) -> dict[str, dict[str, Any]]:
        result = self._request_json(
            "GET",
            "/v1/mavlink/parameters?"
            + urllib.parse.urlencode({"timeout_ms": round(timeout * 1000)}),
            timeout=timeout + 1.0,
        )
        return {
            str(item["name"]): dict(item)
            for item in result["parameters"]
        }

    def set_params(
        self,
        parameters: list[dict[str, Any]],
        *,
        timeout: float = 15.0,
        retries: int = 2,
    ) -> dict[str, dict[str, Any]]:
        result = self._request_json(
            "PUT",
            "/v1/mavlink/parameters",
            {
                "parameters": parameters,
                "timeout_ms": round(timeout * 1000),
                "retries": retries,
            },
            timeout=timeout + 1.0,
        )
        records = {
            str(item["name"]): dict(item)
            for item in result["parameters"]
        }
        if records:
            self._last_operation_cursor = next(
                reversed(records.values())
            ).get("after_cursor")
        return records

    def set_mode(
        self,
        custom_mode: int,
        *,
        timeout: float = 10.0,
        **_ignored: Any,
    ) -> bool:
        result = self.command(
            mavlink.MAV_CMD_DO_SET_MODE,
            [float(mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), float(custom_mode)],
            timeout=timeout,
        )
        if result.get("result") != mavlink.MAV_RESULT_ACCEPTED:
            raise LinkHubError(f"Mode {custom_mode} was rejected: {result}")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            heartbeat = self._recv(type="HEARTBEAT", blocking=True, timeout=0.5)
            if (
                heartbeat is not None
                and int(getattr(heartbeat, "custom_mode", -1)) == custom_mode
            ):
                return True
        raise TimeoutError(f"Mode {custom_mode} was not confirmed by heartbeat")

    @property
    def is_armed(self) -> bool:
        return self._armed

    def arm(
        self,
        *,
        timeout: float = 10.0,
        force: bool = False,
    ) -> bool:
        result = self.command(
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 21196.0 if force else 0.0],
            timeout=timeout,
        )
        if result.get("result") != mavlink.MAV_RESULT_ACCEPTED:
            raise LinkHubError(f"Arm was rejected: {result}")
        return self._wait_armed(True, timeout)

    def disarm(
        self,
        *,
        timeout: float = 10.0,
        force: bool = False,
    ) -> bool:
        result = self.command(
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [0.0, 21196.0 if force else 0.0],
            timeout=timeout,
        )
        if result.get("result") != mavlink.MAV_RESULT_ACCEPTED:
            raise LinkHubError(f"Disarm was rejected: {result}")
        return self._wait_armed(False, timeout)

    def sim_now(self) -> float:
        return self._sim_time_s

    def sim_sleep(self, duration_s: float, check=None) -> None:
        deadline = self.sim_now() + duration_s
        while self.sim_now() < deadline:
            self._recv(blocking=True, timeout=0.1)
            if check is not None:
                check()

    def set_message_rates(
        self,
        rates: dict[str, float | None],
        *,
        timeout: float = 10.0,
    ) -> list[dict[str, Any]]:
        result = self._request_json(
            "PUT",
            "/v1/mavlink/message-rates",
            rates,
            timeout=timeout,
        )
        return list(result["configured"])

    def _wait_armed(self, expected: bool, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            heartbeat = self._recv(type="HEARTBEAT", blocking=True, timeout=0.5)
            if heartbeat is not None and self._armed == expected:
                return True
        state = "armed" if expected else "disarmed"
        raise TimeoutError(f"Vehicle was not confirmed {state} by heartbeat")

    def set_message_interval(self, message_id: int, interval_us: int) -> None:
        self.command(
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            [float(message_id), float(interval_us)],
        )

    def _recv(
        self,
        type: str | list[str] | tuple[str, ...] | set[str] | None = None,
        blocking: bool = True,
        timeout: float = 1.0,
    ) -> Any | None:
        accepted = None if type is None else {
            name.upper()
            for name in ([type] if isinstance(type, str) else type)
        }
        deadline = time.monotonic() + max(timeout, 0.0)
        with self._receive_cv:
            while True:
                for index, message in enumerate(self._receive_queue):
                    if accepted is None or message.get_type() in accepted:
                        del self._receive_queue[index]
                        self._process_received(message)
                        return message
                if self._receive_error is not None:
                    error = self._receive_error
                    self._receive_error = None
                    raise LinkHubError("Telemetry stream failed") from error
                if not blocking:
                    return None
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._receive_cv.wait(remaining)

    def receive(
        self,
        message_types: str | list[str] | tuple[str, ...] | set[str] | None = None,
        *,
        timeout: float = 1.0,
        blocking: bool = True,
    ) -> RawMessage | None:
        return self._recv(type=message_types, blocking=blocking, timeout=timeout)

    def start_mavlog(self, path: str | Path, **_ignored: Any) -> None:
        self.stop_mavlog()
        self._mavlog_path = Path(path)
        self._mavlog_path.parent.mkdir(parents=True, exist_ok=True)
        self._mavlog_file = self._mavlog_path.open("w", encoding="utf-8")

    def stop_mavlog(self) -> None:
        with self._mavlog_lock:
            if self._mavlog_file is not None:
                self._mavlog_file.close()
            self._mavlog_file = None
            self._mavlog_path = None

    def list_files(self, path: str) -> list[dict[str, Any]]:
        result = self._request_json(
            "GET",
            "/v1/mavlink/files?" + urllib.parse.urlencode({"path": path}),
            timeout=30.0,
        )
        return result["files"]

    def capabilities(self) -> dict[str, Any]:
        return self._request_json(
            "GET",
            "/v1/mavlink/capabilities",
            timeout=5.0,
        )

    def components(self) -> list[dict[str, Any]]:
        result = self._request_json(
            "GET",
            "/v1/mavlink/components",
            timeout=5.0,
        )
        return list(result["components"])

    def download_file(
        self,
        remote_path: str,
        local_path: str | Path,
        *,
        verify_crc: bool = True,
    ) -> int:
        query = urllib.parse.urlencode({
            "path": remote_path,
            "download": "true",
            "verify_crc": str(verify_crc).lower(),
        })
        request = urllib.request.Request(
            f"{self._base_url}/v1/mavlink/files?{query}"
        )
        with urllib.request.urlopen(request, timeout=60.0) as response:
            content = response.read()
        destination = Path(local_path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
        return len(content)

    def upload_file(self, local_path: str | Path, remote_path: str) -> int:
        content = Path(local_path).read_bytes()
        result = self._request_json(
            "PUT",
            "/v1/mavlink/files?" + urllib.parse.urlencode({"path": remote_path}),
            content=content,
            content_type="application/octet-stream",
            timeout=60.0,
        )
        return int(result["bytes"])

    def remove_file(self, path: str) -> None:
        self._request_json(
            "DELETE",
            "/v1/mavlink/files?" + urllib.parse.urlencode({"path": path}),
            timeout=30.0,
        )

    def create_directory(self, path: str) -> bool:
        result = self._request_json(
            "POST",
            "/v1/mavlink/directories",
            {"path": path},
            timeout=30.0,
        )
        return bool(result["created"])

    def list_logs(self, timeout: float = 5.0) -> list[dict[str, Any]]:
        query = urllib.parse.urlencode({"timeout_ms": round(timeout * 1000)})
        result = self._request_json(
            "GET",
            f"/v1/mavlink/logs?{query}",
            timeout=timeout + 1.0,
        )
        return list(result["logs"])

    def download_log(
        self,
        log_id: int,
        destination: str | Path,
        *,
        timeout: float = 1.0,
        max_retries: int = 5,
    ) -> Path:
        destination_path = Path(destination)
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        partial_path = destination_path.with_name(destination_path.name + ".part")
        query = urllib.parse.urlencode({
            "timeout_ms": round(timeout * 1000),
            "max_retries": max_retries,
        })
        request = urllib.request.Request(
            f"{self._base_url}/v1/mavlink/logs/{log_id}?{query}"
        )
        try:
            with urllib.request.urlopen(
                request,
                timeout=max(30.0, timeout * (max_retries + 1)),
            ) as response, partial_path.open("wb") as output:
                while chunk := response.read(64 * 1024):
                    output.write(chunk)
                cursor = response.headers.get("X-LinkHub-After-Cursor")
        except urllib.error.HTTPError as exc:
            partial_path.unlink(missing_ok=True)
            try:
                error = json.load(exc)
                message = error.get("message") or error.get("error")
            except (ValueError, AttributeError):
                message = str(exc)
            raise LinkHubError(message) from exc
        except Exception:
            partial_path.unlink(missing_ok=True)
            raise
        partial_path.replace(destination_path)
        if cursor:
            self._last_operation_cursor = cursor
        return destination_path

    def motor_status(self) -> dict[str, Any]:
        return self._request_json("GET", "/v1/motor")

    def motor_set(
        self,
        speed_percent: int,
        direction: str,
        timeout_ms: int = 2000,
    ) -> dict[str, Any]:
        return self._request_json("PUT", "/v1/motor", {
            "speed_percent": speed_percent,
            "direction": direction,
            "timeout_ms": timeout_ms,
        })

    def motor_stop(self) -> dict[str, Any]:
        return self._request_json("POST", "/v1/motor/stop", {})

    def motor_reconnect(self) -> dict[str, Any]:
        return self._request_json("POST", "/v1/motor/reconnect", {})

    def _send_raw(
        self,
        message_name: str,
        payload: dict[str, Any],
        *,
        update_cursor: bool = True,
    ) -> str:
        result = self._request_json("POST", "/v1/mavlink/messages", {
            "message": message_name,
            "fields": payload,
        })
        cursor = str(result["after_cursor"])
        return cursor

    def _start_receive_stream(self) -> None:
        if self._receive_thread is not None and self._receive_thread.is_alive():
            return
        self._receive_stop.clear()
        self._receive_error = None
        self._receive_thread = threading.Thread(
            target=self._receive_worker,
            daemon=True,
            name="linkhub-client-receive",
        )
        self._receive_thread.start()

    def _stop_receive_stream(self) -> None:
        self._receive_stop.set()
        response = self._receive_response
        if response is not None:
            try:
                response.close()
            except OSError:
                pass
        with self._receive_cv:
            self._receive_cv.notify_all()
        if (
            self._receive_thread is not None
            and self._receive_thread is not threading.current_thread()
        ):
            self._receive_thread.join(timeout=2.0)
        self._receive_thread = None
        self._receive_response = None

    def _receive_worker(self) -> None:
        while not self._receive_stop.is_set():
            query = {"direction": "rx", "follow": "true"}
            with self._cursor_lock:
                if self._cursor:
                    query["after"] = self._cursor
            request = urllib.request.Request(
                self._base_url
                + "/v1/mavlink/messages?"
                + urllib.parse.urlencode(query)
            )
            try:
                with urllib.request.urlopen(request, timeout=5.0) as response:
                    self._receive_response = response
                    for line in response:
                        if self._receive_stop.is_set():
                            return
                        if not line.strip():
                            continue
                        record = TelemetryRecord.from_dict(json.loads(line))
                        message = _decode_record(record)
                        with self._cursor_lock:
                            self._cursor = record.cursor
                        self._write_mavlog_record(record)
                        with self._receive_cv:
                            self._receive_queue.append(message)
                            self._receive_cv.notify_all()
            except (
                AttributeError,
                TimeoutError,
                OSError,
                urllib.error.URLError,
                ValueError,
            ):
                if self._receive_stop.is_set():
                    return
                time.sleep(0.05)
            finally:
                self._receive_response = None

    def _process_received(self, message: RawMessage) -> None:
        boot_ms = getattr(message, "time_boot_ms", 0)
        if boot_ms:
            self._sim_time_s = max(self._sim_time_s, int(boot_ms) / 1000.0)
        if message.get_type() == "HEARTBEAT":
            self._armed = bool(
                int(getattr(message, "base_mode", 0))
                & mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

    def _request_json(
        self,
        method: str,
        path: str,
        body: Any | None = None,
        *,
        content: bytes | None = None,
        content_type: str = "application/json",
        timeout: float = 10.0,
    ) -> Any:
        if content is None and body is not None:
            content = json.dumps(body).encode()
        request = urllib.request.Request(
            self._base_url + path,
            data=content,
            headers={"Content-Type": content_type} if content is not None else {},
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            raise LinkHubError(_http_error_message(exc)) from exc

    def _write_mavlog_record(self, record: TelemetryRecord) -> None:
        with self._mavlog_lock:
            if self._mavlog_file is None:
                return
            output = {
                "_t_wall": record.received_time_ns / 1_000_000_000,
                "_dir": record.direction,
                "mavpackettype": record.message,
                **record.fields,
            }
            self._mavlog_file.write(json.dumps(output) + "\n")
            self._mavlog_file.flush()


class LinkHubMotorController:
    """Compatibility-shaped logical motor client backed only by LinkHub."""

    def __init__(
        self,
        session: LinkHubClient,
        *,
        speed: int = 10,
        direction: str = "F",
    ) -> None:
        if not 0 <= speed <= 100:
            raise ValueError("speed must be in 0..100")
        if direction not in ("F", "R"):
            raise ValueError("direction must be F or R")
        self._session = session
        self.speed = speed
        self.direction = direction
        self.enabled = False
        self._renew_stop = threading.Event()
        self._renew_thread: threading.Thread | None = None

    def connect(self) -> str:
        status = self._session.motor_status()
        if not status.get("available"):
            raise LinkHubError("LinkHub motor support is not configured")
        if not status.get("connected"):
            status = self._session.motor_reconnect()
        self.stop()
        return str(status.get("device") or "BLDC")

    def set_speed(self, speed: int) -> None:
        if not 0 <= speed <= 100:
            raise ValueError("speed must be in 0..100")
        self.speed = speed
        if self.enabled:
            self._renew()

    def start(self) -> None:
        if self.speed <= 0:
            raise ValueError("motor speed must be positive before start")
        self.enabled = True
        self._renew()
        if self._renew_thread is None:
            self._renew_stop.clear()
            self._renew_thread = threading.Thread(
                target=self._renew_worker,
                daemon=True,
                name="calibrate-motor-renew",
            )
            self._renew_thread.start()

    def stop(self) -> None:
        self.enabled = False
        self._session.motor_stop()

    def close(self) -> None:
        self.enabled = False
        self._renew_stop.set()
        if self._renew_thread is not None:
            self._renew_thread.join(timeout=2.0)
        self._renew_thread = None
        self._session.motor_stop()

    def _renew(self) -> None:
        self._session.motor_set(
            self.speed,
            "forward" if self.direction == "F" else "reverse",
            timeout_ms=2000,
        )

    def _renew_worker(self) -> None:
        while not self._renew_stop.wait(1.0):
            if not self.enabled:
                continue
            try:
                self._renew()
            except (OSError, LinkHubError):
                self.enabled = False
                return


def _decode_record(record: TelemetryRecord) -> RawMessage:
    return RawMessage(record.message, dict(record.fields))


def _http_error_message(exc: urllib.error.HTTPError) -> str:
    try:
        error = json.load(exc)
        return str(error.get("message") or error.get("error"))
    except (OSError, ValueError, TypeError, AttributeError):
        return str(exc)
