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
#   bash sim.sh build                     build Docker image with ArduPilot (~30-60 min)
#   bash sim.sh start | stop | shell      manage the rawes-dev container
#   bash sim.sh test-stack [...]          run stack integration tests (Docker)
#   bash sim.sh test-torque [...]         run torque tests (Docker)
#   bash sim.sh exec <cmd...>             run arbitrary command in container
#
# All extra args for Docker commands are forwarded to dev.sh unchanged.
#
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$REPO_DIR/simulation"

# Convert a Git Bash path (e.g. /c/repos/windpower) to a WSL path (/mnt/c/repos/windpower).
_wsl_path() {
    local mixed
    mixed=$(cygpath -m "$1")          # /c/repos/... → C:/repos/...
    wsl.exe bash -c "wslpath '$mixed'" | tr -d '\r'
}

VENV="$SIM_DIR/.venv"
PYTHON="$VENV/Scripts/python.exe"

CMD="${1:-}"
shift || true

case "$CMD" in
    setup)
        echo "[INFO] Creating venv at $VENV ..."
        py -3 -m venv "$VENV"
        "$PYTHON" -m pip install --upgrade pip --quiet
        "$PYTHON" -m pip install -r "$SIM_DIR/requirements.txt"
        echo "[INFO] Done. Venv ready at simulation/.venv"
        ;;
    test-unit)
        "$PYTHON" -m pytest "$SIM_DIR/tests/unit" -m "not simtest" "$@"
        ;;
    test-simtest)
        "$PYTHON" -m pytest "$SIM_DIR/tests/unit" -m simtest "$@"
        ;;
    build)
        WSL_SIM=$(_wsl_path "$SIM_DIR")
        echo "[INFO] Building rawes-sim with ArduPilot -- expect ~30-60 min ..."
        wsl.exe bash -c "docker build '$WSL_SIM' -t rawes-sim --build-arg INSTALL_ARDUPILOT=true"
        echo "[INFO] Build complete. Run: bash sim.sh start"
        ;;
    "")
        echo "Usage: $0 setup | test-unit [...] | test-simtest [...] | build | start | stop | shell | test-stack [...] | test-torque [...] | exec <cmd>"
        ;;
    *)
        WSL_SIM=$(_wsl_path "$SIM_DIR")
        # Use printf %q to safely quote each argument for the inner bash -c string.
        INNER_ARGS=$(printf ' %q' "$CMD" "$@")
        wsl.exe bash -c "bash '$WSL_SIM/dev.sh'$INNER_ARGS"
        ;;
esac
