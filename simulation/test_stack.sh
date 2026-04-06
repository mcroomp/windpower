#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export RAWES_RUN_STACK_INTEGRATION="${RAWES_RUN_STACK_INTEGRATION:-1}"

if [ -z "${RAWES_SIM_VEHICLE:-}" ] && [ -z "${RAWES_ARDUPILOT_PATH:-}" ]; then
    if [ -f "/ardupilot/Tools/autotest/sim_vehicle.py" ]; then
        export RAWES_SIM_VEHICLE=/ardupilot/Tools/autotest/sim_vehicle.py
    else
        echo "[ERROR] Set RAWES_SIM_VEHICLE or RAWES_ARDUPILOT_PATH before running stack integration tests."
        exit 1
    fi
fi

mkdir -p "$SCRIPT_DIR/logs"

# RAWES_FILTER_MODE: summary (default) | all | failures
# summary = show PASSED/FAILED lines + failure details only (hides per-test log spam)
# all     = full raw output (same as before)
# failures = failure sections only
FILTER_MODE="${RAWES_FILTER_MODE:-summary}"

exec python "$SCRIPT_DIR/run_tests.py" \
    --log "$SCRIPT_DIR/logs/pytest_last_run.log" \
    --filter "$FILTER_MODE" \
    "$SCRIPT_DIR/tests/stack" -s "$@"
