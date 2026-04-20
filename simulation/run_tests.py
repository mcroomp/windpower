#!/usr/bin/env python3
"""
run_tests.py — pytest wrapper with streaming output and filtering.

Usage:
    python simulation/run_tests.py [--filter all|summary|failures]
                                   [--summary FILE] [pytest args...]

Filter modes:
    all       stream every line to terminal (same as running pytest directly)
    summary   show per-test PASSED/FAILED lines + failure sections + final result line
    failures  show only failure sections + final result line

--summary FILE
    Write a small JSON summary (counts + failed test list) to FILE.
    Default: simulation/logs/suite_summary.json

Per-test physics logs (telemetry.csv, mediator.log, etc.) go in
simulation/logs/{test_name}/ and are written by each test fixture.

Examples:
    python simulation/run_tests.py simulation/tests/unit -m "not simtest" -q
    python simulation/run_tests.py --filter failures simulation/tests/unit -v
    python simulation/run_tests.py simulation/tests/unit -k test_aero
"""

import hashlib
import json
import os
import subprocess
import sys
import re
import time
from datetime import datetime
from pathlib import Path

VENV_PYTHON      = "simulation/.venv/Scripts/python.exe"
REQUIREMENTS     = "simulation/requirements.txt"
DEFAULT_SUMMARY  = "simulation/logs/suite_summary.json"

# ANSI colours
RED    = "\033[31m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
CYAN   = "\033[36m"
RESET  = "\033[0m"
BOLD   = "\033[1m"
DIM    = "\033[2m"

_ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')


def _ensure_venv_fresh(python: str) -> None:
    """Install/upgrade packages if requirements.txt changed since the last run.

    Uses a hash stamp stored alongside the venv so the check is near-instant
    (~0 ms) when nothing has changed, and auto-installs only when needed.
    """
    req = Path(REQUIREMENTS)
    if not req.exists():
        return

    digest = hashlib.sha256(req.read_bytes()).hexdigest()
    stamp  = Path(python).parent / ".requirements_hash"

    if stamp.exists() and stamp.read_text().strip() == digest:
        return  # already up to date

    print(f"{YELLOW}[env] requirements.txt changed — running pip install...{RESET}", flush=True)
    result = subprocess.run(
        [python, "-m", "pip", "install", "-q", "-r", str(req)],
        capture_output=False,
    )
    if result.returncode != 0:
        print(f"{RED}[env] pip install failed (exit {result.returncode}){RESET}", flush=True)
    else:
        stamp.write_text(digest)
        print(f"{GREEN}[env] venv up to date{RESET}", flush=True)

def _strip(line: str) -> str:
    return _ANSI_RE.sub('', line)

def _is_test_result(line: str) -> bool:
    s = _strip(line)
    return bool(re.search(r'\s(PASSED|FAILED|ERROR|XPASS|XFAIL|SKIPPED)', s)
                or s.startswith('FAILED ') or s.startswith('ERROR '))

def _is_section_header(line: str) -> bool:
    return bool(re.search(r'^[=\-]{5,}', _strip(line)))

def _is_final_summary(line: str) -> bool:
    return bool(re.search(r'\d+ (passed|failed|error)', _strip(line)))

def _is_failure_section_start(line: str) -> bool:
    s = _strip(line)
    return 'FAILURES' in s or 'ERRORS' in s or 'short test summary' in s

def _extract_test_id(line: str) -> str:
    """Extract bare test ID (path::test_name) from a result line."""
    s = _strip(line).strip()
    m = re.match(r'^(\S+::\S+)\s+(PASSED|FAILED|ERROR)', s)
    if m:
        return m.group(1)
    m = re.match(r'^(?:FAILED|ERROR|PASSED)\s+(\S+)', s)
    if m:
        return m.group(1).rstrip(':')
    return s

def _colorize_result_line(line: str) -> str:
    s = _strip(line)
    if re.search(r'\sPASSED', s) or s.startswith('PASSED '):
        return GREEN + s + RESET
    if re.search(r'\sFAILED', s) or s.startswith('FAILED '):
        return RED + BOLD + s + RESET
    if re.search(r'\sERROR', s) or s.startswith('ERROR '):
        return YELLOW + BOLD + s + RESET
    if re.search(r'\sSKIPPED', s):
        return DIM + s + RESET
    return line


def main() -> int:
    raw_args = sys.argv[1:]
    filter_mode  = "summary"
    summary_file = DEFAULT_SUMMARY
    pytest_args: list[str] = []

    i = 0
    while i < len(raw_args):
        a = raw_args[i]
        if a == "--filter" and i + 1 < len(raw_args):
            filter_mode = raw_args[i + 1]; i += 2
        elif a == "--summary" and i + 1 < len(raw_args):
            summary_file = raw_args[i + 1]; i += 2
        else:
            pytest_args.append(a); i += 1

    if filter_mode not in ("all", "summary", "failures"):
        print(f"Unknown --filter '{filter_mode}'. Use all|summary|failures.", file=sys.stderr)
        return 2

    if pytest_args and pytest_args[0] == "bash":
        print(
            f"{RED}ERROR: run_tests.py does not wrap shell commands.{RESET}\n"
            f"{RED}Run stack tests directly:{RESET}\n"
            f"{CYAN}  bash simulation/dev.sh test-stack [options]{RESET}",
            file=sys.stderr,
        )
        return 2

    if not pytest_args:
        pytest_args = ["simulation/tests/unit", "-m", "not simtest", "-q"]

    _venv  = Path(VENV_PYTHON)
    python = str(_venv.resolve()) if _venv.exists() else sys.executable
    _ensure_venv_fresh(python)
    cmd    = [python, "-u", "-m", "pytest"] + pytest_args

    print(f"{CYAN}{'-'*70}{RESET}")
    print(f"{CYAN}cmd:    {' '.join(cmd)}{RESET}")
    print(f"{CYAN}filter: {filter_mode}{RESET}")
    print(f"{CYAN}{'-'*70}{RESET}")
    print()

    in_detail_block    = False
    passed_tests: list[str] = []
    failed_tests: list[str] = []
    final_summary_line = ""
    t_start = time.monotonic()

    env  = {**os.environ, "PYTHONUNBUFFERED": "1"}
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=env,
        cwd=str(Path.cwd()),
    )

    assert proc.stdout is not None
    for raw_line in proc.stdout:
        bare = raw_line.rstrip("\n")
        s    = _strip(bare)

        if re.search(r'\sPASSED', s):
            passed_tests.append(_extract_test_id(bare))
        elif re.search(r'\sFAILED', s) or s.startswith('FAILED '):
            tid = _extract_test_id(bare)
            if tid not in failed_tests:
                failed_tests.append(tid)

        if _is_final_summary(bare):
            final_summary_line = s
        if _is_failure_section_start(bare):
            in_detail_block = True

        show = False
        if filter_mode == "all":
            show = True
        elif filter_mode == "summary":
            show = (
                _is_test_result(bare)
                or _is_final_summary(bare)
                or _is_section_header(bare)
                or in_detail_block
            )
        elif filter_mode == "failures":
            show = _is_final_summary(bare) or in_detail_block

        if show:
            if _is_test_result(bare):
                print(_colorize_result_line(bare), flush=True)
            else:
                print(bare, flush=True)

    proc.wait()
    elapsed = time.monotonic() - t_start

    m = re.search(r'(\d+) passed', final_summary_line)
    n_pass = int(m.group(1)) if m else len(passed_tests)
    m = re.search(r'(\d+) failed', final_summary_line)
    n_fail = int(m.group(1)) if m else len(failed_tests)
    m = re.search(r'(\d+) error', final_summary_line)
    n_err  = int(m.group(1)) if m else 0
    m = re.search(r'(\d+) skipped', final_summary_line)
    n_skip = int(m.group(1)) if m else 0

    rc = proc.returncode

    # ── Write summary JSON ───────────────────────────────────────────────────
    summary_path = Path(summary_file)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    summary = {
        "timestamp":    timestamp,
        "elapsed_s":    round(elapsed, 2),
        "exit_code":    rc,
        "passed":       n_pass,
        "failed":       n_fail,
        "errors":       n_err,
        "skipped":      n_skip,
        "result":       "passed" if rc == 0 else "failed",
        "failed_tests": failed_tests,
        "cmd":          cmd,
    }
    summary_path.write_text(json.dumps(summary, indent=2))

    _time_str = f"{elapsed:.1f}s"

    print()
    print(f"{CYAN}{'-'*70}{RESET}")
    if rc == 0:
        print(f"{GREEN}{BOLD}  ALL TESTS PASSED{RESET}  "
              f"{GREEN}{n_pass} passed{RESET}"
              + (f", {DIM}{n_skip} skipped{RESET}" if n_skip else "")
              + f"  {DIM}({_time_str}){RESET}")
    else:
        parts = []
        if n_fail: parts.append(f"{RED}{BOLD}{n_fail} failed{RESET}")
        if n_err:  parts.append(f"{YELLOW}{BOLD}{n_err} error{RESET}")
        if n_pass: parts.append(f"{GREEN}{n_pass} passed{RESET}")
        if n_skip: parts.append(f"{DIM}{n_skip} skipped{RESET}")
        print(f"{RED}{BOLD}  TESTS FAILED{RESET}  " + ", ".join(parts)
              + f"  {DIM}({_time_str}){RESET}")
        if failed_tests:
            print(f"{RED}  Failed tests:{RESET}")
            for t in failed_tests:
                print(f"{RED}    - {t}{RESET}")
    print(f"{CYAN}{'-'*70}{RESET}")
    print(f"{CYAN}summary: {summary_path.resolve()}{RESET}")

    return rc


if __name__ == "__main__":
    sys.exit(main())
