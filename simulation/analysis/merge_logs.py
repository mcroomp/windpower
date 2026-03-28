"""
merge_logs.py — Merge and display RAWES stack log files in timestamp order.

Reads mediator, SITL, GCS, and telemetry logs from the last test run (or a
specified directory) and prints all lines in a single unified timeline.

Usage
-----
  # Last test run logs (auto-discover from pytest tmp_path):
  python simulation/merge_logs.py

  # Specific log directory:
  python simulation/merge_logs.py /path/to/tmp/pytest-xxx/test_yyy/

  # Filter by source:
  python simulation/merge_logs.py --only mediator gcs

  # Show N lines of context around errors:
  python simulation/merge_logs.py --errors-only

Output columns
--------------
  HH:MM:SS.mmm  SOURCE   LEVEL  message
"""

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


# ---------------------------------------------------------------------------
# Log sources
# ---------------------------------------------------------------------------

@dataclass
class LogLine:
    timestamp_s: float      # seconds since epoch (for sorting)
    wall_time:   str        # original HH:MM:SS string (for display)
    source:      str        # mediator / sitl / gcs / telemetry
    level:       str        # INFO / WARNING / ERROR / CRITICAL / DEBUG / ''
    text:        str        # message body

    @property
    def color(self) -> str:
        if self.level in ("ERROR", "CRITICAL"):
            return "\033[31m"
        if self.level == "WARNING":
            return "\033[33m"
        if self.level == "DEBUG":
            return "\033[90m"
        return ""

    @property
    def reset(self) -> str:
        return "\033[0m" if self.color else ""


# ---------------------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------------------

# Pattern for Python logging format:
#   HH:MM:SS,mmm LOGGER_NAME       LEVEL  message
# OR
#   HH:MM:SS LOGGER_NAME           LEVEL  message
_PY_LOG_RE = re.compile(
    r"^(\d{2}:\d{2}:\d{2})(?:[,.](\d+))?\s+"   # HH:MM:SS[,mmm]
    r"(\S+)\s+"                                  # logger name
    r"(DEBUG|INFO|WARNING|ERROR|CRITICAL)\s+"    # level
    r"(.*)$"                                     # message
)

# Pattern for ArduPilot SITL output (no standard format, just text lines)
_SITL_ANY_RE = re.compile(r"^(.*)$")


def _time_to_s(h: int, m: int, s: int, ms: int = 0) -> float:
    return h * 3600 + m * 60 + s + ms / 1000.0


def _parse_py_log(path: Path, source: str) -> list[LogLine]:
    lines = []
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return []

    for raw in text.splitlines():
        m = _PY_LOG_RE.match(raw)
        if m:
            hhmmss, ms_s, _logger, level, msg = m.groups()
            ms = int(ms_s) if ms_s else 0
            h, mn, s = (int(x) for x in hhmmss.split(":"))
            ts = _time_to_s(h, mn, s, ms)
            lines.append(LogLine(ts, hhmmss, source, level, msg))
        elif lines:
            # Continuation line (multiline log message)
            lines[-1] = LogLine(
                lines[-1].timestamp_s, lines[-1].wall_time,
                lines[-1].source, lines[-1].level,
                lines[-1].text + "\n  " + raw,
            )
    return lines


def _parse_sitl_log(path: Path) -> list[LogLine]:
    """
    Parse SITL stdout.  No timestamps — assign monotonic t=0 ordering.
    Lines that look like ArduPilot status messages are flagged as INFO.
    """
    lines = []
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return []

    for i, raw in enumerate(text.splitlines()):
        raw = raw.rstrip()
        if not raw:
            continue
        level = "INFO"
        if any(kw in raw.lower() for kw in ("error", "fail", "critical", "assert")):
            level = "ERROR"
        elif any(kw in raw for kw in ("PreArm", "WARNING", "Warn")):
            level = "WARNING"
        lines.append(LogLine(float(i) / 1e6, "??:??:??", "sitl", level, raw))
    return lines


def _parse_telemetry_csv(path: Path) -> list[LogLine]:
    """Parse mediator telemetry CSV (t_s,fx,fy,fz,...) — just show t=0 summary."""
    lines = []
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return []
    rows = [r for r in text.splitlines() if r and not r.startswith("#")]
    if rows:
        lines.append(LogLine(0.0, "??:??:??", "telemetry", "INFO",
                             f"CSV with {len(rows)} data rows"))
    return lines


# ---------------------------------------------------------------------------
# Discover log files
# ---------------------------------------------------------------------------

def _find_log_dir() -> Path | None:
    """Find the most recent pytest tmpdir for our test."""
    import tempfile, os
    tmp = Path(tempfile.gettempdir())
    # pytest creates /tmp/pytest-of-USER/pytest-NNN/test_name0/
    candidates = sorted(tmp.glob("pytest-of-*/pytest-*/"), key=lambda p: p.stat().st_mtime, reverse=True)
    for c in candidates:
        subdirs = sorted(c.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True)
        for sub in subdirs:
            if (sub / "mediator.log").exists() or (sub / "gcs.log").exists():
                return sub
    # Also try simulation/ directory directly
    sim_dir = Path(__file__).parent
    if (sim_dir / "mediator.log").exists():
        return sim_dir
    return None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description="Merge RAWES stack log files")
    parser.add_argument("log_dir", nargs="?", help="Directory containing log files")
    parser.add_argument("--only", nargs="+",
                        choices=["mediator", "sitl", "gcs", "telemetry"],
                        help="Show only these sources")
    parser.add_argument("--errors-only", action="store_true",
                        help="Show only WARNING/ERROR/CRITICAL lines")
    parser.add_argument("--no-color", action="store_true",
                        help="Disable ANSI color codes")
    args = parser.parse_args()

    if args.log_dir:
        log_dir = Path(args.log_dir)
    else:
        log_dir = _find_log_dir()
        if log_dir is None:
            print("No log directory found.  Run a stack test first or pass a path.",
                  file=sys.stderr)
            return 1
        print(f"Using log dir: {log_dir}", file=sys.stderr)

    # Load all sources
    all_lines: list[LogLine] = []

    if not args.only or "mediator" in args.only:
        all_lines += _parse_py_log(log_dir / "mediator.log",  "mediator")
    if not args.only or "gcs" in args.only:
        all_lines += _parse_py_log(log_dir / "gcs.log",       "gcs     ")
    if not args.only or "sitl" in args.only:
        sitl_lines = _parse_sitl_log(log_dir / "sitl.log")
        if sitl_lines:
            # sitl has no real timestamps; we don't sort it with others unless
            # the user asked for it explicitly
            if args.only and "sitl" in args.only:
                all_lines += sitl_lines
            else:
                print("─── SITL log (unordered) ───", flush=True)
                for ln in sitl_lines:
                    if args.errors_only and ln.level not in ("WARNING", "ERROR", "CRITICAL"):
                        continue
                    print(f"  {ln.text}")
                print()
    if not args.only or "telemetry" in args.only:
        all_lines += _parse_telemetry_csv(log_dir / "telemetry.csv")

    # Sort by timestamp (stable — preserves order within same second)
    all_lines.sort(key=lambda l: l.timestamp_s)

    if args.errors_only:
        all_lines = [l for l in all_lines if l.level in ("WARNING", "ERROR", "CRITICAL")]

    use_color = not args.no_color and sys.stdout.isatty()

    source_w = max((len(l.source) for l in all_lines), default=8)
    level_w  = 8

    for ln in all_lines:
        c  = ln.color if use_color else ""
        rc = ln.reset if use_color else ""
        print(f"{ln.wall_time}  {ln.source:<{source_w}}  {c}{ln.level:<{level_w}}{rc}  {ln.text}")

    total = len(all_lines)
    errs  = sum(1 for l in all_lines if l.level in ("ERROR", "CRITICAL"))
    warns = sum(1 for l in all_lines if l.level == "WARNING")
    print(f"\n─── {total} lines  {errs} errors  {warns} warnings ───", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
