"""
calibrate/util.py -- Pure-Python utilities with no MAVLink/session dependencies.

These are shared by run.py, watch.py, params.py, and repl.py without risk of
circular imports.
"""
from __future__ import annotations

import csv
import os
from datetime import datetime

# msvcrt is Windows stdlib -- used for non-blocking ESC/key reads.
# Falls back to a stub on non-Windows so the rest of the module still imports.
try:
    import msvcrt
except ImportError:
    class _MsvcrtStub:
        def kbhit(self):    return False
        def getch(self):    return b""
    msvcrt = _MsvcrtStub()  # type: ignore[assignment]

from .constants import _LOG_DIR


# ---------------------------------------------------------------------------
# CSV cell formatter
# ---------------------------------------------------------------------------

def _fmt(v):
    """CSV cell formatter: '' for None, otherwise the value's natural repr."""
    if v is None:
        return ""
    if isinstance(v, float):
        return f"{v:.6g}"
    return v


# ---------------------------------------------------------------------------
# Flag / KV parsers
# ---------------------------------------------------------------------------

def _parse_kv_list(s: str) -> dict[str, float]:
    """Parse 'a=1,b=2.5' -> {'a': 1.0, 'b': 2.5}.  Trailing commas ignored."""
    out: dict[str, float] = {}
    for tok in s.split(","):
        tok = tok.strip()
        if not tok:
            continue
        if "=" not in tok:
            raise ValueError(f"key=value expected, got {tok!r}")
        k, _, v = tok.partition("=")
        out[k.strip().lower()] = float(v.strip())
    return out


def _parse_flags(tokens: list[str],
                 schema: dict[str, str]) -> tuple[list[str], dict]:
    """Split tokens into (positionals, flags).

    schema maps '--flag' -> 'float' | 'int' | 'kv' | 'bool' | 'str'.
    Unknown flags raise ValueError so typos surface immediately.
    """
    pos: list[str] = []
    flags: dict[str, object] = {}
    i = 0
    while i < len(tokens):
        t = tokens[i]
        if t.startswith("--"):
            kind = schema.get(t)
            if kind is None:
                raise ValueError(
                    f"Unknown flag {t!r}  (valid: {', '.join(sorted(schema))})"
                )
            if kind == "bool":
                flags[t] = True
                i += 1
                continue
            if i + 1 >= len(tokens):
                raise ValueError(f"Flag {t} needs a value")
            val = tokens[i + 1]
            if kind == "float":
                flags[t] = float(val)
            elif kind == "int":
                flags[t] = int(val)
            elif kind == "kv":
                prev: dict[str, float] = flags.get(t, {})  # type: ignore[assignment]
                prev.update(_parse_kv_list(val))
                flags[t] = prev
            else:  # "str"
                flags[t] = val
            i += 2
        else:
            pos.append(t)
            i += 1
    return pos, flags


# ---------------------------------------------------------------------------
# CSV run log
# ---------------------------------------------------------------------------

def _log_path(verb: str, name: str) -> str:
    """Timestamped CSV path under simulation/logs/calibrate/.  Creates the dir."""
    os.makedirs(_LOG_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(_LOG_DIR, f"{verb}_{name}_{ts}.csv")


class _RunLog:
    """CSV log with metadata header.  Header is '# k: v' lines, then data rows.

    Lifecycle::

        log = _RunLog.open("run", "passive", meta={"duration_s": 30})
        log.write_header(["t_s", "armed", "yaw_dps"])
        log.row([0.0, 1, +0.02])
        log.close()
    """
    def __init__(self, path: str, fh, writer):
        self.path = path
        self._fh = fh
        self._w  = writer
        self.n_rows = 0

    @classmethod
    def open(cls, verb: str, name: str, meta: dict) -> "_RunLog":
        path = _log_path(verb, name)
        fh = open(path, "w", newline="")
        fh.write(f"# {verb}.csv -- written by calibrate.py {verb}\n")
        for k, v in meta.items():
            fh.write(f"# {k}: {v}\n")
        fh.write("#\n")
        w = csv.writer(fh)
        return cls(path, fh, w)

    def write_header(self, cols: list[str]) -> None:
        self._w.writerow(cols)

    def row(self, values: list) -> None:
        self._w.writerow(values)
        self.n_rows += 1

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Key polling (Windows msvcrt; stub on other platforms)
# ---------------------------------------------------------------------------

def _esc_check() -> bool:
    """Non-blocking ESC-key check.  Returns True if ESC was pressed."""
    seen = False
    while msvcrt.kbhit():
        if msvcrt.getch() == b"\x1b":
            seen = True
    return seen


def _poll_keys() -> list[bytes]:
    """Non-blocking: drain and return all pending keypresses (one byte each)."""
    keys = []
    while msvcrt.kbhit():
        keys.append(msvcrt.getch())
    return keys
