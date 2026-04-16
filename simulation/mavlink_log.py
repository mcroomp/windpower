"""
mavlink_log.py — NDJSON MAVLink message log.

Every received MAVLink message is written as one JSON line:
    {"_t_wall": <float>, "mavpackettype": "<TYPE>", ...fields...}

Writer: MavlinkLogWriter — wrap an open file and call write(msg) per message.
Reader: iter_messages()  — yield dicts from a .jsonl path, with optional type filter.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Iterator


class MavlinkLogWriter:
    """
    Write MAVLink messages as NDJSON to an open file handle.

    Parameters
    ----------
    fh : writable text file
        Must remain open for the lifetime of this object.
    """

    def __init__(self, fh) -> None:
        self._fh = fh

    def write(self, msg, last_time_boot_ms: int) -> None:
        """
        Serialize *msg* (a pymavlink message) as one JSON line.

        Some MAVLink message types (e.g. STATUSTEXT) do not carry a
        ``time_boot_ms`` field.  Pass the last known sim time as
        *last_time_boot_ms* so those entries get a meaningful timestamp.
        """
        try:
            d = msg.to_dict()
            if "time_boot_ms" not in d and last_time_boot_ms > 0:
                d["time_boot_ms"] = last_time_boot_ms
            self._fh.write(json.dumps({"_t_wall": time.time(), **d}) + "\n")
        except Exception:
            pass

    @classmethod
    def open(cls, path: "str | Path") -> "MavlinkLogWriter":
        """Open *path* for writing and return a MavlinkLogWriter."""
        fh = open(Path(path), "w", encoding="utf-8")
        writer = cls(fh)
        writer._owned_fh = fh  # keep reference so caller can close via writer.close()
        return writer

    def close(self) -> None:
        """Close the underlying file if opened via MavlinkLogWriter.open()."""
        fh = getattr(self, "_owned_fh", None)
        if fh is not None:
            try:
                fh.close()
            except Exception:
                pass
            self._owned_fh = None


def iter_messages(
    path: "str | Path",
    types: "list[str] | None" = None,
) -> Iterator[dict]:
    """
    Yield message dicts from a mavlink.jsonl file.

    Parameters
    ----------
    path : str | Path
        Path to the .jsonl file.
    types : list[str] | None
        If given, only yield messages whose ``mavpackettype`` is in this list.
        E.g. ``types=["ATTITUDE", "EKF_STATUS_REPORT"]``.
    """
    p = Path(path)
    if not p.exists():
        return
    type_set = set(types) if types else None
    with p.open(encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except (ValueError, KeyError):
                continue
            if type_set is None or msg.get("mavpackettype") in type_set:
                yield msg
