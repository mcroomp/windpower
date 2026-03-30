#!/usr/bin/env python3
"""
run_tests.py — pytest wrapper with streaming output, filtering, and log saving.

Usage:
    python simulation/run_tests.py [--log FILE] [--filter all|summary|failures] [pytest args...]

Defaults:
    --log     simulation/pytest_last_run.log
    --filter  summary   (show PASSED/FAILED lines + failure details + final summary)

Filter modes:
    all       stream every line to terminal (same as running pytest directly)
    summary   show per-test PASSED/FAILED lines + failure sections + final result line
    failures  show only failure sections + final result line

Examples:
    python simulation/run_tests.py simulation/tests/unit -m "not simtest" -q
    python simulation/run_tests.py --filter failures simulation/tests/unit -v
    python simulation/run_tests.py --log mytest.log simulation/tests/unit -k test_aero
"""

import subprocess
import sys
import re
import os
from datetime import datetime
from pathlib import Path

VENV_PYTHON = "simulation/tests/unit/.venv/Scripts/python.exe"
DEFAULT_LOG = "simulation/pytest_last_run.log"

# ANSI colours
RED    = "\033[31m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
CYAN   = "\033[36m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

_ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')

def _strip(line: str) -> str:
    return _ANSI_RE.sub('', line)


def _is_test_result(line: str) -> bool:
    s = _strip(line)
    return bool(re.search(r'\s(PASSED|FAILED|ERROR|XPASS|XFAIL|SKIPPED)', s)
                or s.startswith('FAILED ') or s.startswith('ERROR '))


def _is_section_header(line: str) -> bool:
    s = _strip(line)
    return bool(re.search(r'^[=\-]{5,}', s))


def _is_final_summary(line: str) -> bool:
    s = _strip(line)
    return bool(re.search(r'\d+ (passed|failed|error)', s))


def _is_failure_section_start(line: str) -> bool:
    s = _strip(line)
    return 'FAILURES' in s or 'ERRORS' in s or 'short test summary' in s


def main() -> int:
    raw_args = sys.argv[1:]

    log_file = DEFAULT_LOG
    filter_mode = "summary"
    pytest_args = []

    i = 0
    while i < len(raw_args):
        a = raw_args[i]
        if a == "--log" and i + 1 < len(raw_args):
            log_file = raw_args[i + 1]
            i += 2
        elif a == "--filter" and i + 1 < len(raw_args):
            filter_mode = raw_args[i + 1]
            i += 2
        else:
            pytest_args.append(a)
            i += 1

    if filter_mode not in ("all", "summary", "failures"):
        print(f"Unknown --filter value '{filter_mode}'. Use all|summary|failures.", file=sys.stderr)
        return 2

    if not pytest_args:
        pytest_args = ["simulation/tests/unit", "-m", "not simtest", "-q"]

    _venv = Path(VENV_PYTHON)
    python = str(_venv.resolve()) if _venv.exists() else sys.executable
    cmd = [python, "-u", "-m", "pytest"] + pytest_args

    log_path = Path(log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    header = (
        f"# run_tests.py log\n"
        f"# date:    {timestamp}\n"
        f"# command: {' '.join(cmd)}\n"
        f"# filter:  {filter_mode}\n\n"
    )

    print(f"{CYAN}cmd:    {' '.join(cmd)}{RESET}")
    print(f"{CYAN}log:    {log_path.resolve()}{RESET}")
    print(f"{CYAN}filter: {filter_mode}{RESET}")
    print()

    in_detail_block = False   # inside FAILURES / ERRORS / short summary section

    with open(log_path, "w", encoding="utf-8") as lf:
        lf.write(header)

        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=0,          # unbuffered reads from the pipe
            cwd=str(Path.cwd()),
        )

        while True:
            raw_line = proc.stdout.readline()
            if not raw_line:
                break
            bare = raw_line.rstrip("\n")

            # always persist full output
            lf.write(raw_line)
            lf.flush()

            # track entry into failure/error detail blocks
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
                print(bare, flush=True)

        proc.wait()

    rc = proc.returncode
    print()
    if rc == 0:
        print(f"{GREEN}{BOLD}ALL TESTS PASSED{RESET}")
    else:
        print(f"{RED}{BOLD}TESTS FAILED  (exit {rc}){RESET}")
    print(f"{CYAN}log: {log_path.resolve()}{RESET}")

    return rc


if __name__ == "__main__":
    sys.exit(main())
