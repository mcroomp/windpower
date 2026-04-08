#!/usr/bin/env bash
#
# sim.sh — Git Bash entry point for the RAWES simulation toolchain
#
# Works on any drive — no hardcoded paths.
#
# Usage:
#   bash sim.sh setup                     create simulation/.venv and install requirements.txt (Windows)
#   bash sim.sh test-unit [...]           run unit tests (Windows, no Docker)
#   bash sim.sh test-simtest [...]        run simtests (Windows, no Docker)
#   bash sim.sh test-hil [...]            run HIL smoke tests (Windows, Pixhawk on USB)
#   bash sim.sh build                     build Docker image with ArduPilot (~30-60 min)
#   bash sim.sh start | stop | shell      manage the rawes-dev container
#   bash sim.sh test-stack [...]          run stack integration tests (Docker)
#   bash sim.sh test-torque [...]         run torque tests (Docker)
#   bash sim.sh exec <cmd...>             run arbitrary command in container
#
# All extra args for Docker commands are forwarded to dev.sh unchanged.
#
set -euo pipefail

# Prevent Git Bash from converting Linux paths to Windows paths in docker args.
export MSYS_NO_PATHCONV=1

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$REPO_DIR/simulation"

VENV="$SIM_DIR/.venv"
PYTHON="$VENV/Scripts/python.exe"

# Windows Python cannot resolve Git Bash paths (/e/repos/...).
# Convert to Windows-style paths (E:\repos\...) for any argument passed to $PYTHON.
_winpath() { cygpath -w "$1"; }

CMD="${1:-}"
shift || true

case "$CMD" in
    setup)
        echo "[INFO] Creating venv at $VENV ..."
        py -3 -m venv "$(_winpath "$VENV")"
        "$PYTHON" -m pip install --upgrade pip --quiet
        "$PYTHON" -m pip install -r "$(_winpath "$SIM_DIR/requirements.txt")"
        echo "[INFO] Done. Venv ready at simulation/.venv"
        ;;
    test-unit)
        "$PYTHON" -m pytest "$(_winpath "$SIM_DIR/tests/unit")" -m "not simtest" "$@"
        ;;
    test-simtest)
        "$PYTHON" -m pytest "$(_winpath "$SIM_DIR/tests/unit")" -m simtest "$@"
        ;;
    test-hil)
        "$PYTHON" -m pytest "$(_winpath "$SIM_DIR/tests/hil")" "$@"
        ;;
    setup-hw)
        # Interactive Pixhawk setup script.  Requires RAWES_HIL_PORT=COMx.
        "$PYTHON" "$(_winpath "$SIM_DIR/scripts/setup_pixhawk.py")" "$@"
        ;;
    build)
        echo "[INFO] Building rawes-sim with ArduPilot -- expect ~30-60 min ..."
        docker build "$SIM_DIR" -t rawes-sim --build-arg INSTALL_ARDUPILOT=true
        echo "[INFO] Build complete. Run: bash sim.sh start"
        ;;
    "")
        echo "Usage: $0 setup | test-unit [...] | test-simtest [...] | test-hil [...] | setup-hw | build | start | stop | shell | test-stack [...] | test-torque [...] | exec <cmd>"
        ;;
    *)
        bash "$SIM_DIR/dev.sh" "$CMD" "$@"
        ;;
esac
