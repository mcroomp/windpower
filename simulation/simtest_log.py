"""
simtest_log.py — shared logging utility for simtests.

Each test module creates one instance at module level:

    from simtest_log import SimtestLog
    _log = SimtestLog(__file__)

Then calls _log.write(lines, summary) to flush a full diagnostic table to disk
and print one concise line to stdout (captured by pytest; visible in run_tests.py
summary output or with -s).

Log files land in simulation/logs/<module_stem>.log so they are easy to find
and read after a run without grepping pytest output.

BadEventLog
-----------
Append bad simulation events (slack, floor_hit, tension_spike) during a run,
then query counts by kind and phase for assertions and summaries.

    events = BadEventLog()
    if tether._last_info.get("slack"):
        events.record("slack", t, phase, alt, tension=tension_now)
    if altitude <= FLOOR_ALT_M:
        events.record("floor_hit", t, phase, alt)
    if tension_now > BREAK_LOAD_N:
        events.record("tension_spike", t, phase, alt, tension=tension_now)
"""
from __future__ import annotations

from pathlib import Path

_LOG_DIR = Path(__file__).parent / "logs"


class BadEventLog:
    """
    Unified tracker for simulation bad events.

    Events are classified by *kind* (slack | floor_hit | tension_spike) and
    *phase* (pumping | reel-out | reel-in | descent | final_drop | ...).
    Both dimensions are needed for assertions: pumping slack is always fatal;
    final_drop floor-contact is expected; descent slack is a controller bug.
    """

    def __init__(self) -> None:
        self._events: list[dict] = []

    # ------------------------------------------------------------------ write
    def record(self, kind: str, t: float, phase: str, alt: float, **extra) -> None:
        """Append one bad event."""
        self._events.append(dict(kind=kind, t=t, phase=phase, alt=alt, **extra))

    # ----------------------------------------------------------------- query
    def of_kind(self, kind: str, phase: str = None) -> list[dict]:
        evs = [e for e in self._events if e["kind"] == kind]
        if phase is not None:
            evs = [e for e in evs if e["phase"] == phase]
        return evs

    def count(self, kind: str, phase: str = None) -> int:
        return len(self.of_kind(kind, phase))

    # ------------------------------------------------------------ diagnostics
    def summary(self) -> str:
        if not self._events:
            return "no bad events"
        parts = []
        for kind in ("slack", "floor_hit", "tension_spike"):
            evs = self.of_kind(kind)
            if not evs:
                continue
            by_phase: dict[str, int] = {}
            for e in evs:
                by_phase[e["phase"]] = by_phase.get(e["phase"], 0) + 1
            detail = "+".join(
                f"{c}({p})" for p, c in sorted(by_phase.items())
            )
            parts.append(f"{kind}={detail}")
        return "  ".join(parts)


class SimtestLog:
    """
    Per-test log directory and diagnostic file for simtests.

    Creates simulation/logs/<module_stem>/ on construction and writes
    a simtest.log file into it when write() is called.  The log_dir
    property gives callers the directory path so they can co-locate
    telemetry.csv and other outputs in the same place.

    Usage::

        _log = SimtestLog(__file__)

        # write telemetry alongside the log
        write_csv(rows, _log.log_dir / "telemetry.csv")

        # flush diagnostic text and print summary line
        _log.write(lines, summary)
    """

    def __init__(self, caller_file: str) -> None:
        _LOG_DIR.mkdir(exist_ok=True)
        self.name = Path(caller_file).stem          # e.g. "test_deschutter_cycle"
        self._log_dir = _LOG_DIR / self.name
        self._log_dir.mkdir(exist_ok=True)
        self.path = self._log_dir / "simtest.log"

    @property
    def log_dir(self) -> Path:
        """Per-test output directory: simulation/logs/<module_stem>/"""
        return self._log_dir

    def write(self, lines: list[str], summary: str) -> None:
        """
        Write *lines* to the log file and print one summary line to stdout.

        Parameters
        ----------
        lines:   full diagnostic content (each element is one line of text)
        summary: compact key-value string shown on stdout, e.g.
                 "net=+1901J  reel-out=211N  reel-in=55N  floor_hits=0"
                 The module name and log path are appended automatically.
        """
        self.path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"[{self.name}] {summary}  log: {self.path}")
