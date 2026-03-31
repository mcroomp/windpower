#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export RAWES_RUN_STACK_INTEGRATION="${RAWES_RUN_STACK_INTEGRATION:-1}"

if [ -z "${RAWES_SIM_VEHICLE:-}" ] && [ -z "${RAWES_ARDUPILOT_PATH:-}" ]; then
    if [ -f "/ardupilot/Tools/autotest/sim_vehicle.py" ]; then
        export RAWES_SIM_VEHICLE=/ardupilot/Tools/autotest/sim_vehicle.py
    else
        echo "[ERROR] Set RAWES_SIM_VEHICLE or RAWES_ARDUPILOT_PATH before running torque stack tests."
        exit 1
    fi
fi

mkdir -p "$SCRIPT_DIR/logs"
PYTEST_LOG="$SCRIPT_DIR/logs/pytest_torque_last_run.log"
echo "[INFO] Running counter-torque motor stack tests in the container ..."
echo "[INFO] Full test output -> $PYTEST_LOG"
python -m pytest -s "$SCRIPT_DIR/torque" "$@" 2>&1 | tee "$PYTEST_LOG"
