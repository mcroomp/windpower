"""
mavlink_log.py — NDJSON MAVLink message log.

Every received MAVLink message is written as one JSON line:
    {"_t_wall": <float>, "mavpackettype": "<TYPE>", ...fields...}

Writer: MavlinkLogWriter — wrap an open file and call write(msg) per message.
Reader: iter_messages()  — yield dicts from a .jsonl path, with optional type filter.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Iterator

from pymavlink import mavutil


def _json_safe(value):
    """Convert pymavlink binary fields into deterministic JSON values."""
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


class MavlinkLogWriter:
    """
    Write MAVLink messages as NDJSON to an open file handle.

    Each line records both directions of traffic; a ``_dir`` field marks
    whether the message was received ("rx") or sent ("tx").

    Parameters
    ----------
    fh : writable text file
        Must remain open for the lifetime of this object.
    """

    def __init__(self, fh) -> None:
        self._fh = fh
        self._lock = threading.Lock()

    def write(self, msg, last_time_boot_ms: int, direction: str = "rx") -> None:
        """
        Serialize *msg* (a pymavlink message) as one JSON line.

        Some MAVLink message types (e.g. STATUSTEXT) do not carry a
        ``time_boot_ms`` field.  Pass the last known sim time as
        *last_time_boot_ms* so those entries get a meaningful timestamp.

        *direction* is "rx" for received messages (default) or "tx" for
        messages sent by this GCS; it is recorded as the ``_dir`` field.
        """
        try:
            d = msg.to_dict()
            if "time_boot_ms" not in d and last_time_boot_ms > 0:
                d["time_boot_ms"] = last_time_boot_ms
            line = json.dumps(_json_safe(
                {"_t_wall": time.time(), "_dir": direction, **d}
            )) + "\n"
            with self._lock:
                self._fh.write(line)
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


def convert_raw_to_ndjson(
    raw_path: "str | Path",
    ndjson_path: "str | Path",
) -> int:
    """Replay a native pymavlink raw capture into parser-derived NDJSON.

    The raw capture is the source of truth. Bytes that cannot be parsed are
    intentionally absent from the derived NDJSON and remain available in the
    raw file for diagnosis.
    """
    reader = mavutil.mavlogfile(
        str(raw_path),
        robust_parsing=True,
        notimestamps=True,
    )
    decoded = 0
    wall_start = time.time()
    first_boot_ms: int | None = None
    last_wall = wall_start
    try:
        with Path(ndjson_path).open("w", encoding="utf-8") as fh:
            while True:
                msg = reader.recv_msg()
                if msg is None:
                    break
                if msg.get_type() == "BAD_DATA":
                    continue
                boot_ms = getattr(msg, "time_boot_ms", None)
                if boot_ms is not None:
                    if first_boot_ms is None:
                        first_boot_ms = int(boot_ms)
                    last_wall = wall_start + (int(boot_ms) - first_boot_ms) / 1000.0
                fh.write(json.dumps(_json_safe({
                    "_t_wall": last_wall,
                    "_dir": "rx",
                    "_source": "native_raw",
                    **msg.to_dict(),
                })) + "\n")
                decoded += 1
    finally:
        reader.f.close()
    return decoded
