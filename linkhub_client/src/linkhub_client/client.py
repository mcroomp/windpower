from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

from . import mav_constants as mavlink
from .messages import Message, RawMessage, decode_message
from .records import DiagnosticEvent, DiagnosticRecord, SimClock, TelemetryRecord


class LinkHubError(RuntimeError):
    pass


class LinkHubGenerationChanged(LinkHubError):
    """LinkHub's generation token changed mid-sequence.

    The `generation` token (LinkHub run ID + MAVLink clock epoch) changes
    whenever the service restarts, the serial link reconnects, or the vehicle
    reboots. A multi-step sequence that polls for a state transition (e.g.
    confirming arm/disarm) cannot trust that transition if this happens
    part-way through, so it should stop and surface this instead of
    continuing to poll a possibly different vehicle/boot.
    """

    def __init__(
        self,
        baseline: str | None,
        current: str | None,
        *,
        cursor: str | None = None,
    ) -> None:
        self.baseline = baseline
        self.current = current
        # The journal cursor at the moment the change was observed, so a
        # caller that was reading messages against the old `baseline` can
        # skip straight to the tail instead of draining a backlog that may
        # now belong to a different vehicle boot.
        self.cursor = cursor
        super().__init__(
            f"LinkHub generation changed ({baseline!r} -> {current!r}): the "
            "service restarted, the serial link reconnected, or the vehicle "
            "rebooted mid-sequence"
        )


class WallClock:
    pass


@dataclass(frozen=True)
class MessageBatch:
    messages: tuple[Message | RawMessage, ...]
    next_cursor: str
    next_clock: SimClock


@dataclass(frozen=True)
class DiagnosticBatch:
    records: tuple[DiagnosticRecord, ...]
    next_cursor: str
    next_clock: SimClock


class LinkHubClient:
    """Dependency-free client for LinkHub's HTTP v1 API."""

    def __init__(
        self,
        address: str = "http://127.0.0.1:8999",
    ) -> None:
        self._base_url = address.rstrip("/")
        self._target_system = 1
        self._target_component = 1
        self._generation: str | None = None

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
                self._generation = status.get("generation")
                return
            except (OSError, LinkHubError) as exc:
                last_error = exc
                time.sleep(0.25)
        raise TimeoutError(
            f"Could not connect to LinkHub at {self._base_url}"
        ) from last_error

    def close(self) -> None:
        pass

    @property
    def generation(self) -> str | None:
        """Opaque LinkHub generation token captured at `connect()` time.

        Changes whenever LinkHub's run or MAVLink link is replaced (service
        restart, serial reconnect, or vehicle reboot); see `vehicle_status()`.
        Pass this as `expected_generation` to `read_messages()`/`read_one()`
        inside a multi-step sequence (e.g. confirming arm/disarm) so LinkHub
        itself aborts the read with `LinkHubGenerationChanged` the moment it
        no longer matches, instead of continuing to poll a link that is no
        longer trustworthy. Leave it unset for single, one-shot reads.
        """
        return self._generation

    def linkhub_status(self) -> dict[str, Any]:
        """Return LinkHub service, run, journal, and transport status."""
        return dict(self._request_json("GET", "/v1/status"))

    def vehicle_status(self) -> dict[str, Any]:
        return dict(self._request_json("GET", "/v1/mavlink/status"))

    def current_cursor(self) -> str:
        return str(self.vehicle_status()["cursor"])

    def flush_journal(self) -> str:
        result = self._request_json("POST", "/v1/journal/flush", {})
        return str(result["cursor"])

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

    def read_diagnostics(
        self,
        after: str,
        *,
        wait: float = 0.0,
        limit: int = 1_000,
        source: str | None = None,
        event: str | None = None,
        level: str | None = None,
    ) -> DiagnosticBatch:
        """Read a finite diagnostic batch after an opaque journal cursor."""
        query = {
            key: value
            for key, value in {
                "after": after,
                "wait_ms": round(wait * 1_000),
                "limit": limit,
                "source": source,
                "event": event,
                "level": level,
            }.items()
            if value is not None
        }
        result = self._request_json(
            "GET",
            "/v1/diagnostics/events?" + urllib.parse.urlencode(query),
            timeout=max(1.0, wait + 1.0),
        )
        return DiagnosticBatch(
            records=tuple(
                DiagnosticRecord.from_dict(value) for value in result["records"]
            ),
            next_cursor=str(result["next_cursor"]),
            next_clock=SimClock.from_dict(result["next_clock"]),
        )

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
        return float(result["value"])

    def set_param(
        self,
        name: str,
        value: float,
        *,
        timeout: float = 3.0,
        param_type: int | None = None,
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
        return records

    def set_mode(
        self,
        custom_mode: int,
        *,
        timeout: float = 10.0,
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
            if int(self.vehicle_status().get("custom_mode", -1)) == custom_mode:
                return True
            time.sleep(0.05)
        raise TimeoutError(f"Mode {custom_mode} was not confirmed by heartbeat")

    @property
    def is_armed(self) -> bool:
        return bool(
            int(self.vehicle_status().get("base_mode", 0))
            & mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

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
        clock = SimClock.from_dict(self.vehicle_status()["sim_clock"])
        return float(clock.time_boot_ms or 0) / 1000.0

    def sim_sleep(self, duration_s: float, check=None) -> None:
        deadline = self.sim_now() + duration_s
        while self.sim_now() < deadline:
            if check is not None:
                check()
            time.sleep(0.02)

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
            if self.is_armed == expected:
                return True
            time.sleep(0.05)
        state = "armed" if expected else "disarmed"
        raise TimeoutError(f"Vehicle was not confirmed {state} by heartbeat")

    def set_message_interval(self, message_id: int, interval_us: int) -> None:
        self.command(
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            [float(message_id), float(interval_us)],
        )

    def read_messages(
        self,
        after: str,
        message_types: str | list[str] | tuple[str, ...] | set[str] | None = None,
        *,
        direction: str | None = None,
        wait: float = 0.0,
        limit: int = 1_000,
        collapse: bool = False,
        expected_generation: str | None = None,
    ) -> MessageBatch:
        records, next_cursor, next_clock = self._read_telemetry_batch(
            after,
            message_types=message_types,
            direction=direction,
            wait=wait,
            limit=limit,
            collapse=collapse,
            expected_generation=expected_generation,
        )
        return MessageBatch(
            messages=tuple(_decode_record(record) for record in records),
            next_cursor=next_cursor,
            next_clock=next_clock,
        )

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
    ) -> str:
        result = self._request_json("POST", "/v1/mavlink/messages", {
            "message": message_name,
            "fields": payload,
        })
        cursor = str(result["after_cursor"])
        return cursor

    def _read_telemetry_batch(
        self,
        after: str,
        message_types: str | list[str] | tuple[str, ...] | set[str] | None = None,
        *,
        direction: str | None = None,
        wait: float = 0.0,
        limit: int = 1_000,
        collapse: bool = False,
        request_timeout: float | None = None,
        expected_generation: str | None = None,
    ) -> tuple[list[TelemetryRecord], str, SimClock]:
        if wait < 0.0:
            raise ValueError("wait must be non-negative")
        if limit <= 0:
            raise ValueError("limit must be positive")
        query = {
            "after": after,
            "wait_ms": round(wait * 1_000),
            "limit": limit,
        }
        if collapse:
            query["collapse"] = "true"
        if message_types is not None:
            names = [message_types] if isinstance(message_types, str) else message_types
            query["messages"] = ",".join(sorted(name.upper() for name in names))
        if direction is not None:
            query["direction"] = direction
        if expected_generation is not None:
            query["expected_generation"] = expected_generation
        result = self._request_json(
            "GET",
            "/v1/mavlink/messages?" + urllib.parse.urlencode(query),
            timeout=(
                max(5.0, wait + 2.0)
                if request_timeout is None
                else request_timeout
            ),
        )
        return (
            [TelemetryRecord.from_dict(value) for value in result["records"]],
            str(result["next_cursor"]),
            SimClock.from_dict(result["next_clock"]),
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
            payload = _http_error_payload(exc)
            if payload.get("error") == "generation_changed":
                raise LinkHubGenerationChanged(
                    payload.get("expected_generation"),
                    payload.get("current_generation"),
                    cursor=payload.get("cursor"),
                ) from exc
            raise LinkHubError(_http_error_message(payload, exc)) from exc

class LinkHubMotorController:
    """Logical motor client backed by LinkHub."""

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


def _decode_record(record: TelemetryRecord) -> Message | RawMessage:
    return decode_message(RawMessage(record.message, dict(record.fields)))


def _http_error_payload(exc: urllib.error.HTTPError) -> dict[str, Any]:
    try:
        payload = json.load(exc)
    except (OSError, ValueError, TypeError, AttributeError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _http_error_message(payload: dict[str, Any], exc: urllib.error.HTTPError) -> str:
    message = payload.get("message") or payload.get("error")
    return str(message) if message else str(exc)
