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

echo "[INFO] Running stack integration tests in the container ..."
python -m pytest "$SCRIPT_DIR/tests/stack" "$@"
