Run pytest via `simulation/run_tests.py`, streaming filtered output and saving a log.

**Script:** `simulation/run_tests.py`
**Default log:** `simulation/pytest_last_run.log`
**Default filter:** `summary` — shows per-test PASSED/FAILED lines, all failure details, and the final result line.

**Filter modes** (pass as first arg if needed):
- `--filter all` — stream every line (same as running pytest directly)
- `--filter summary` — per-test status lines + failure sections + final summary *(default)*
- `--filter failures` — only failure/error sections + final summary

**Log:** `--log <path>` overrides the output file.

---

Execute using the Bash tool:

```
simulation/tests/unit/.venv/Scripts/python.exe simulation/run_tests.py $ARGUMENTS
```

If `$ARGUMENTS` is empty, default to:
```
simulation/tests/unit/.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/unit -m "not simtest" -q
```

After the run:
1. Report pass/fail status and test counts from the final summary line.
2. State the full path to the log file so the user can reference it.

Note: do not pipe the command through `tail`, `head`, or any other filter — the script already handles output filtering and the full output should be shown as-is.
