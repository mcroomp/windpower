#!/usr/bin/env bash
#
# setup.sh -- RAWES one-time setup tasks.  All subcommands are idempotent.
#
# Subcommands:
#   (no args)   create or refresh the Windows venv at simulation/.venv
#                 hash-gated: requirements.txt is re-installed only when changed.
#   build       build the rawes-sim Docker image with ArduPilot (~30-60 min)
#   hw          push canonical params to a real Pixhawk via MAVLink
#                 requires:  RAWES_HIL_PORT=COMx
#
# Run from Git Bash on Windows:
#   bash setup.sh                              # venv
#   bash setup.sh build                        # Docker image
#   RAWES_HIL_PORT=COM4 bash setup.sh hw       # Pixhawk params
#
set -euo pipefail
export MSYS_NO_PATHCONV=1

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$REPO_DIR/simulation"
VENV="$SIM_DIR/.venv"
PYTHON="$VENV/Scripts/python.exe"
REQS="$SIM_DIR/requirements.txt"
STAMP="$VENV/Scripts/.requirements_hash"

_winpath() { cygpath -w "$1"; }

# --- venv (default) ----------------------------------------------------
_setup_venv() {
    if [ -x "$PYTHON" ]; then
        echo "[INFO] Reusing venv at $VENV"
    else
        if [ -e "$VENV" ]; then
            echo "[WARN] $VENV exists but has no python.exe -- recreating"
            rm -rf "$VENV"
        fi
        echo "[INFO] Creating venv at $VENV ..."
        py -3 -m venv "$(_winpath "$VENV")"
        "$PYTHON" -m pip install --upgrade pip --quiet
    fi

    if [ ! -f "$REQS" ]; then
        echo "[WARN] $REQS not found -- skipping requirements install"
    else
        local digest
        digest="$(sha256sum "$REQS" | awk '{print $1}')"
        if [ -f "$STAMP" ] && [ "$(cat "$STAMP")" = "$digest" ]; then
            echo "[INFO] requirements.txt unchanged -- skipping pip install"
        else
            echo "[INFO] Installing requirements ..."
            "$PYTHON" -m pip install -r "$(_winpath "$REQS")"
            echo "$digest" > "$STAMP"
        fi
    fi

    echo "[INFO] Done."
    "$PYTHON" --version
}

# --- Docker image ------------------------------------------------------
_setup_build() {
    echo "[INFO] Building rawes-sim with ArduPilot -- expect ~30-60 min ..."
    docker build "$SIM_DIR" -t rawes-sim --build-arg INSTALL_ARDUPILOT=true
    echo "[INFO] Build complete.  Run: bash test.sh start"
}

# --- Pixhawk hardware --------------------------------------------------
_setup_hw() {
    if [ ! -x "$PYTHON" ]; then
        echo "[ERROR] $PYTHON not found.  Run 'bash setup.sh' first." >&2
        exit 1
    fi
    "$PYTHON" "$(_winpath "$SIM_DIR/scripts/setup_pixhawk.py")" "$@"
}

CMD="${1:-}"
shift || true

case "$CMD" in
    ""|venv)        _setup_venv ;;
    build)          _setup_build ;;
    hw)             _setup_hw "$@" ;;
    -h|--help)
        sed -n '3,17p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
        ;;
    *)
        echo "[ERROR] Unknown subcommand: $CMD" >&2
        echo "Expected: (no args) | build | hw" >&2
        exit 1
        ;;
esac
