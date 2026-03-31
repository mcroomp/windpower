Run pytest via `simulation/run_tests.py`, streaming filtered output and saving structured logs.

**Script:** `simulation/run_tests.py`
**Logs directory:** `simulation/logs/`

## Output files (all written on every run)

| File | Content |
|------|---------|
| `simulation/logs/pytest_last_run.log` | Full raw pytest output |
| `simulation/logs/pytest_last_run_passed.log` | One test ID per line for passing tests |
| `simulation/logs/pytest_last_run_failed.log` | One test ID per line for failing tests |
| `simulation/logs/pytest_last_run_summary.json` | **Machine-readable JSON** — counts, file paths, failed test list |

## JSON summary format (for parsing failures programmatically)
```json
{
  "timestamp": "2025-01-01 12:00:00",
  "elapsed_s": 19.4,
  "exit_code": 1,
  "passed": 266,
  "failed": 2,
  "errors": 0,
  "skipped": 5,
  "result": "failed",
  "log": "/abs/path/pytest_last_run.log",
  "passed_log": "/abs/path/pytest_last_run_passed.log",
  "failed_log": "/abs/path/pytest_last_run_failed.log",
  "failed_tests": ["simulation/tests/unit/test_foo.py::test_bar"],
  "cmd": ["python", "-m", "pytest", ...]
}
```

## Filter modes
- `--filter summary` *(default)* — PASSED/FAILED lines + failure details + final result
- `--filter all` — stream every line (verbose)
- `--filter failures` — only failure/error sections + final result

## Override log path
`--log <path>` — companion files (_passed, _failed, _summary) are written alongside it.

---

## Execution

```
simulation/tests/unit/.venv/Scripts/python.exe simulation/run_tests.py $ARGUMENTS
```

Default when `$ARGUMENTS` is empty:
```
simulation/tests/unit/.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/unit -m "not simtest" -q
```

## After the run
1. Report pass/fail status and counts from the final printed summary.
2. For failures: read `simulation/logs/pytest_last_run_summary.json` to get the list of failed test IDs — this is the fastest way to know which tests need fixing.
3. For failure details: read `simulation/logs/pytest_last_run.log`.

**CRITICAL: Always use this skill — never run `python -m pytest ...` directly.**
Direct pytest bypasses logging, loses failure details, and makes post-run diagnosis impossible.

Note: do not pipe the command through `tail`, `head`, or any other filter.
