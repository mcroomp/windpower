"""
mediator_events.py -- structured event log for RAWES mediators.

MediatorEventLog writes one JSON line per event to a .jsonl file and
provides query methods so tests can inspect events without file scraping.

Writer side (mediator process)
-------------------------------
    ev = MediatorEventLog(path)
    with ev:
        ev.write("startup", t_sim=0.0, run_id=42, wind_ned=[5, 0, 0])
        ev.write("heartbeat", t_sim=1.0, psi_deg=0.0, ...)

    # or without context manager:
    ev = MediatorEventLog(path)
    ev.open()
    ev.write(...)
    ev.close()

    # no-op when path is None (events are silently discarded):
    ev = MediatorEventLog(None)
    ev.write(...)   # does nothing

Reader side (test process)
---------------------------
    ev = MediatorEventLog(path)          # no open() needed for reading
    ev.has_event("kinematic_exit")       # bool
    ev.get_events("heartbeat")           # list[dict]
    ev.last_event("heartbeat")           # dict | None
    ev.read_all()                        # list[dict]
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class MediatorEventLog:
    """Write and query a JSONL event log for a RAWES mediator process.

    Each line is a JSON object with at least::

        {"event": "<name>", "t_sim": <float>, ...}

    Additional fields are event-specific and documented by each mediator.

    The same instance can be used for writing (mediator process) and reading
    (test process).  Call ``open()`` before writing; reading works on any
    existing file at the path without opening.
    """

    def __init__(self, path: "Path | str | None") -> None:
        self._path: "Path | None" = Path(path) if path is not None else None
        self._fh = None

    # ── Writer side ──────────────────────────────────────────────────────────

    def open(self) -> "MediatorEventLog":
        """Open the file for writing.  Returns self for use as context manager."""
        if self._path is not None:
            self._fh = open(self._path, "w", encoding="utf-8", buffering=1)
        return self

    def close(self) -> None:
        """Flush and close the file."""
        if self._fh is not None:
            self._fh.close()
            self._fh = None

    def __enter__(self) -> "MediatorEventLog":
        return self.open()

    def __exit__(self, *_: Any) -> None:
        self.close()

    def write(self, event: str, t_sim: float, **fields: Any) -> None:
        """Write a structured event line.  No-op when not open or path is None."""
        if self._fh is None:
            return
        d: dict[str, Any] = {"event": event, "t_sim": round(float(t_sim), 3)}
        d.update(fields)
        self._fh.write(json.dumps(d) + "\n")

    # ── Reader side ──────────────────────────────────────────────────────────

    @property
    def path(self) -> "Path | None":
        return self._path

    def read_all(self) -> list[dict]:
        """Read and parse every event line from the file."""
        if self._path is None or not self._path.exists():
            return []
        out: list[dict] = []
        for line in self._path.read_text(encoding="utf-8", errors="replace").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
        return out

    def has_event(self, event: str) -> bool:
        """Return True if at least one event with the given name exists."""
        return any(e.get("event") == event for e in self.read_all())

    def get_events(self, event: str) -> list[dict]:
        """Return all events with the given name, in order."""
        return [e for e in self.read_all() if e.get("event") == event]

    def last_event(self, event: str) -> "dict | None":
        """Return the last event with the given name, or None."""
        events = self.get_events(event)
        return events[-1] if events else None
