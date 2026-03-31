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

Output files (alongside --log path):
    pytest_last_run.log         full raw pytest output
    pytest_last_run_passed.log  one line per passing test  (parseable, no ANSI)
    pytest_last_run_failed.log  one line per failing test  (parseable, no ANSI)

Examples:
    python simulation/run_tests.py simulation/tests/unit -m "not simtest" -q
    python simulation/run_tests.py --filter failures simulation/tests/unit -v
    python simulation/run_tests.py --log mytest.log simulation/tests/unit -k test_aero
"""

import json
import os
import subprocess
import sys
import re
import time
from datetime import datetime
from pathlib import Path

VENV_PYTHON  = "simulation/tests/unit/.venv/Scripts/python.exe"
DEFAULT_LOG  = "simulation/logs/pytest_last_run.log"

# ANSI colours
RED    = "\033[31m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
CYAN   = "\033[36m"
RESET  = "\033[0m"
BOLD   = "\033[1m"
DIM    = "\033[2m"

_ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')

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
    # verbose mode: "path::test PASSED" or "path::test FAILED"
    m = re.match(r'^(\S+::\S+)\s+(PASSED|FAILED|ERROR)', s)
    if m:
        return m.group(1)
    # short summary: "FAILED path::test - reason"
    m = re.match(r'^(?:FAILED|ERROR|PASSED)\s+(\S+)', s)
    if m:
        return m.group(1).rstrip(':')
    return s

def _colorize_result_line(line: str) -> str:
    """Add colour to PASSED/FAILED result lines for terminal output."""
    s = _strip(line)
    if re.search(r'\sPASSED', s) or s.startswith('PASSED '):
        return GREEN + s + RESET
    if re.search(r'\sFAILED', s) or s.startswith('FAILED '):
        return RED + BOLD + s + RESET
    if re.search(r'\sERROR', s) or s.startswith('ERROR '):
        return YELLOW + BOLD + s + RESET
    if re.search(r'\sSKIPPED', s):
        return DIM + s + RESET
    return line  # keep original ANSI for other lines


def main() -> int:
    raw_args = sys.argv[1:]
    log_file    = DEFAULT_LOG
    filter_mode = "summary"
    pytest_args: list[str] = []

    i = 0
    while i < len(raw_args):
        a = raw_args[i]
        if a == "--log" and i + 1 < len(raw_args):
            log_file = raw_args[i + 1]; i += 2
        elif a == "--filter" and i + 1 < len(raw_args):
            filter_mode = raw_args[i + 1]; i += 2
        else:
            pytest_args.append(a); i += 1

    if filter_mode not in ("all", "summary", "failures"):
        print(f"Unknown --filter '{filter_mode}'. Use all|summary|failures.", file=sys.stderr)
        return 2

    if not pytest_args:
        pytest_args = ["simulation/tests/unit", "-m", "not simtest", "-q"]

    _venv  = Path(VENV_PYTHON)
    python = str(_venv.resolve()) if _venv.exists() else sys.executable
    cmd    = [python, "-u", "-m", "pytest"] + pytest_args

    log_path        = Path(log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    passed_log_path = log_path.with_name(log_path.stem + "_passed.log")
    failed_log_path = log_path.with_name(log_path.stem + "_failed.log")

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_header = (
        f"# run_tests.py — {timestamp}\n"
        f"# cmd: {' '.join(cmd)}\n\n"
    )

    # ── Header printed before any test output ───────────────────────────────
    print(f"{CYAN}{'-'*70}{RESET}")
    print(f"{CYAN}cmd:    {' '.join(cmd)}{RESET}")
    print(f"{CYAN}log:    {log_path.resolve()}{RESET}")
    print(f"{CYAN}passed: {passed_log_path.resolve()}{RESET}")
    print(f"{CYAN}failed: {failed_log_path.resolve()}{RESET}")
    print(f"{CYAN}filter: {filter_mode}{RESET}")
    print(f"{CYAN}{'-'*70}{RESET}")
    print()

    in_detail_block    = False
    passed_tests: list[str] = []
    failed_tests: list[str] = []
    final_summary_line = ""
    t_start = time.monotonic()

    with (open(log_path,        "w", encoding="utf-8") as lf,
          open(passed_log_path, "w", encoding="utf-8") as pf,
          open(failed_log_path, "w", encoding="utf-8") as ff):

        lf.write(log_header)
        pf.write(f"# Passed — {timestamp}\n# {' '.join(cmd)}\n\n")
        ff.write(f"# Failed — {timestamp}\n# {' '.join(cmd)}\n\n")

        env = {**os.environ, "PYTHONUNBUFFERED": "1"}
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,          # line-buffered on our side
            env=env,
            cwd=str(Path.cwd()),
        )

        assert proc.stdout is not None
        for raw_line in proc.stdout:
            bare = raw_line.rstrip("\n")
            lf.write(raw_line); lf.flush()

            s = _strip(bare)

            # collect test names (verbose: "path::test PASSED/FAILED")
            if re.search(r'\sPASSED', s):
                tid = _extract_test_id(bare)
                passed_tests.append(tid)
                pf.write(tid + "\n"); pf.flush()
            elif re.search(r'\sFAILED', s) or s.startswith('FAILED '):
                tid = _extract_test_id(bare)
                if tid not in failed_tests:     # short summary may duplicate
                    failed_tests.append(tid)
                    ff.write(tid + "\n"); ff.flush()

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

    # parse counts from final summary (works for both -q and -v)
    m = re.search(r'(\d+) passed', final_summary_line)
    n_pass = int(m.group(1)) if m else len(passed_tests)
    m = re.search(r'(\d+) failed', final_summary_line)
    n_fail = int(m.group(1)) if m else len(failed_tests)
    m = re.search(r'(\d+) error', final_summary_line)
    n_err  = int(m.group(1)) if m else 0
    m = re.search(r'(\d+) skipped', final_summary_line)
    n_skip = int(m.group(1)) if m else 0

    rc = proc.returncode

    # ── Write machine-readable JSON summary ──────────────────────────────────
    summary_path = log_path.with_name(log_path.stem + "_summary.json")
    summary = {
        "timestamp":    timestamp,
        "elapsed_s":    round(elapsed, 2),
        "exit_code":    rc,
        "passed":       n_pass,
        "failed":       n_fail,
        "errors":       n_err,
        "skipped":      n_skip,
        "result":       "passed" if rc == 0 else "failed",
        "log":          str(log_path.resolve()),
        "passed_log":   str(passed_log_path.resolve()),
        "failed_log":   str(failed_log_path.resolve()),
        "failed_tests": failed_tests,
        "cmd":          cmd,
    }
    summary_path.write_text(json.dumps(summary, indent=2))

    # ── Human-readable result block ──────────────────────────────────────────
    print()
    print(f"{CYAN}{'-'*70}{RESET}")
    if rc == 0:
        print(f"{GREEN}{BOLD}  ALL TESTS PASSED{RESET}  "
              f"{GREEN}{n_pass} passed{RESET}"
              + (f", {DIM}{n_skip} skipped{RESET}" if n_skip else "")
              + f"  {DIM}({elapsed:.1f}s){RESET}")
    else:
        parts = []
        if n_fail: parts.append(f"{RED}{BOLD}{n_fail} failed{RESET}")
        if n_err:  parts.append(f"{YELLOW}{BOLD}{n_err} error{RESET}")
        if n_pass: parts.append(f"{GREEN}{n_pass} passed{RESET}")
        if n_skip: parts.append(f"{DIM}{n_skip} skipped{RESET}")
        print(f"{RED}{BOLD}  TESTS FAILED{RESET}  " + ", ".join(parts)
              + f"  {DIM}({elapsed:.1f}s){RESET}")
        if failed_tests:
            print(f"{RED}  Failed tests:{RESET}")
            for t in failed_tests:
                print(f"{RED}    - {t}{RESET}")

    print(f"{CYAN}{'-'*70}{RESET}")
    print(f"{CYAN}log:     {log_path.resolve()}{RESET}")
    print(f"{CYAN}summary: {summary_path.resolve()}{RESET}")
    print(f"{GREEN}passed:  {passed_log_path.resolve()}  ({n_pass}){RESET}")
    if n_fail or n_err:
        print(f"{RED}failed:  {failed_log_path.resolve()}  ({n_fail + n_err}){RESET}")
    else:
        print(f"{CYAN}failed:  {failed_log_path.resolve()}  (0){RESET}")

    return rc


if __name__ == "__main__":
    sys.exit(main())
