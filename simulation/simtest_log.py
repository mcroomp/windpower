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
"""
from __future__ import annotations

from pathlib import Path

_LOG_DIR = Path(__file__).parent / "logs"


class SimtestLog:
    """Write simtest diagnostic output to simulation/logs/<module_stem>.log."""

    def __init__(self, caller_file: str) -> None:
        _LOG_DIR.mkdir(exist_ok=True)
        self.name = Path(caller_file).stem          # e.g. "test_deschutter_cycle"
        self.path = _LOG_DIR / f"{self.name}.log"

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
