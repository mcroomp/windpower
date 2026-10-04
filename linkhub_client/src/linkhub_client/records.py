from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import StrEnum
from typing import Any
from uuid import UUID


@dataclass(frozen=True)
class TelemetryRecord:
    received_time: str
    received_time_ns: int
    direction: str
    system_id: int | None
    component_id: int | None
    message: str
    fields: dict[str, Any]
    cursor: str
    sim_clock: SimClock

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> TelemetryRecord:
        return cls(
            received_time=str(value["received_time"]),
            received_time_ns=int(value["received_time_ns"]),
            direction=str(value["direction"]),
            system_id=(
                None if value.get("system_id") is None else int(value["system_id"])
            ),
            component_id=(
                None
                if value.get("component_id") is None
                else int(value["component_id"])
            ),
            message=str(value["message"]),
            fields=dict(value["fields"]),
            cursor=str(value["cursor"]),
            sim_clock=SimClock.from_dict(value["sim_clock"]),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class DiagnosticLevel(StrEnum):
    TRACE = "trace"
    DEBUG = "debug"
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"


class SimTimeQuality(StrEnum):
    EXACT = "exact"
    LAST_OBSERVED = "last_observed"
    ESTIMATED = "estimated"


@dataclass(frozen=True)
class SimClock:
    epoch: int
    time_boot_ms: int | None
    quality: SimTimeQuality | None

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> SimClock:
        return cls(
            epoch=int(value["epoch"]),
            time_boot_ms=_optional_int(value.get("time_boot_ms")),
            quality=(
                None
                if value.get("quality") is None
                else SimTimeQuality(value["quality"])
            ),
        )


@dataclass(frozen=True)
class DiagnosticEvent:
    schema_version: int
    run_id: UUID
    source: str
    source_instance: str
    source_sequence: int
    source_wall_time_ns: int
    source_monotonic_ns: int | None
    sim_time_ns: int | None
    sim_time_quality: SimTimeQuality | None
    level: DiagnosticLevel
    category: str
    event: str
    message: str
    correlation_id: UUID | None
    causation_id: UUID | None
    fields: dict[str, Any]
    related_records: list[int]

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> DiagnosticEvent:
        return cls(
            schema_version=int(value["schema_version"]),
            run_id=UUID(str(value["run_id"])),
            source=str(value["source"]),
            source_instance=str(value["source_instance"]),
            source_sequence=int(value["source_sequence"]),
            source_wall_time_ns=int(value["source_wall_time_ns"]),
            source_monotonic_ns=_optional_int(value.get("source_monotonic_ns")),
            sim_time_ns=_optional_int(value.get("sim_time_ns")),
            sim_time_quality=(
                None
                if value.get("sim_time_quality") is None
                else SimTimeQuality(value["sim_time_quality"])
            ),
            level=DiagnosticLevel(value["level"]),
            category=str(value["category"]),
            event=str(value["event"]),
            message=str(value["message"]),
            correlation_id=_optional_uuid(value.get("correlation_id")),
            causation_id=_optional_uuid(value.get("causation_id")),
            fields=dict(value.get("fields", {})),
            related_records=[
                int(sequence) for sequence in value.get("related_records", [])
            ],
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            **asdict(self),
            "run_id": str(self.run_id),
            "sim_time_quality": (
                None
                if self.sim_time_quality is None
                else self.sim_time_quality.value
            ),
            "level": self.level.value,
            "correlation_id": (
                None if self.correlation_id is None else str(self.correlation_id)
            ),
            "causation_id": (
                None if self.causation_id is None else str(self.causation_id)
            ),
        }


@dataclass(frozen=True)
class DiagnosticRecord:
    sequence: int
    cursor: str
    ingest_time_ns: int
    correlation_id: UUID | None
    sim_clock: SimClock
    event: DiagnosticEvent

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> DiagnosticRecord:
        if value.get("kind") != "diagnostic.event":
            raise ValueError("record is not a diagnostic event")
        return cls(
            sequence=int(value["sequence"]),
            cursor=str(value["cursor"]),
            ingest_time_ns=int(value["ingest_time_ns"]),
            correlation_id=_optional_uuid(value.get("correlation_id")),
            sim_clock=SimClock.from_dict(value["sim_clock"]),
            event=DiagnosticEvent.from_dict(value["data"]),
        )


def _optional_int(value: Any) -> int | None:
    return None if value is None else int(value)


def _optional_uuid(value: Any) -> UUID | None:
    return None if value is None else UUID(str(value))
