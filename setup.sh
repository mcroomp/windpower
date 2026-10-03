#!/usr/bin/env bash
#
# setup.sh -- RAWES one-time setup tasks.  All subcommands are idempotent.
## Subcommands:
#   (no args)   create or refresh the Windows venv at .venv (repo root)
#                 hash-gated: requirements.txt and the editable rawes package
#                 install are each re-installed only when their source
#                 (requirements.txt / pyproject.toml) changes.
#   build       build rawes-sim runtime with ArduPilot (~30-60 min)
#   build-lite  build rawes-sim runtime without ArduPilot (fast)
#   hw          push canonical params to a real Pixhawk via MAVLink
#                 requires:  RAWES_HIL_PORT=COMx
#
# Run from Git Bash on Windows:
#   bash setup.sh                              # venv
#   bash setup.sh build                        # Docker image
#   bash setup.sh build-lite                   # Docker image without ArduPilot
#   RAWES_HIL_PORT=COM4 bash setup.sh hw       # Pixhawk params (config apply)
#
set -euo pipefail
export MSYS_NO_PATHCONV=1

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PIP_INDEX_URL="https://packagefeedproxy.microsoft.io/pypi/simple/"
SIM_DIR="$REPO_DIR/simulation"
VENV="$REPO_DIR/.venv"
PYTHON="$VENV/Scripts/python.exe"
REQS="$SIM_DIR/requirements.txt"
STAMP="$VENV/Scripts/.requirements_hash"
PYPROJECT="$REPO_DIR/pyproject.toml"
PKG_STAMP="$VENV/Scripts/.editable_install_hash"

_winpath() {
    if command -v cygpath &>/dev/null; then
        cygpath -w "$1"
    elif command -v wslpath &>/dev/null; then
        wslpath -w "$1"
    else
        # Plain bash on a native Windows PATH — no conversion needed.
        echo "$1"
    fi
}

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

    # Install the editable package only when its distribution is missing.
    # Re-running pip install -e can invoke the build backend and appear to hang,
    # while an existing editable install remains valid as source files change.
    if [ ! -f "$PYPROJECT" ]; then
        echo "[WARN] $PYPROJECT not found -- skipping editable install"
    elif "$PYTHON" -c "import importlib.metadata as m; m.version('rawes')" >/dev/null 2>&1; then
        echo "[INFO] rawes editable package already installed -- skipping pip install -e"
    else
        local pkg_digest
        pkg_digest="$(sha256sum "$PYPROJECT" | awk '{print $1}')"
        if [ -f "$PKG_STAMP" ] && [ "$(cat "$PKG_STAMP")" = "$pkg_digest" ]; then
            echo "[INFO] pyproject.toml unchanged -- skipping pip install -e"
        else
            echo "[INFO] Installing rawes package (editable) ..."
            "$PYTHON" -m pip install -e "$(_winpath "$REPO_DIR")" --no-deps --quiet
            echo "$pkg_digest" > "$PKG_STAMP"
        fi
    fi

    if ! "$PYTHON" -c "import linkhub_client" >/dev/null 2>&1; then
        echo "[INFO] Installing linkhub-client package (editable) ..."
        "$PYTHON" -m pip install -e "$(_winpath "$REPO_DIR/linkhub_client")" --no-deps --quiet
    fi

    local linkhub="$REPO_DIR/linkhub/target/release/linkhub.exe"
    if [ ! -x "$linkhub" ]; then
        echo "[INFO] Building LinkHub with Bluetooth support ..."
        cargo build \
            --manifest-path "$REPO_DIR/linkhub/Cargo.toml" \
            --release \
            --features bluetooth
    fi
    echo "[INFO] Done."
    "$PYTHON" --version
}

# --- Docker image ------------------------------------------------------
_setup_build() {
    local ardupilot_image="rawes-sim-ardupilot-base:Copter-4.7.0-v1"
    if [ "${RAWES_REBUILD_ARDUPILOT:-0}" = "1" ] \
        || ! docker image inspect "$ardupilot_image" >/dev/null 2>&1; then
        echo "[INFO] Building $ardupilot_image -- expect ~30-60 min ..."
        docker build \
            -f "$SIM_DIR/Dockerfile" \
            "$REPO_DIR" \
            -t "$ardupilot_image" \
            --target runtime-ardupilot-base
    else
        echo "[INFO] Reusing $ardupilot_image"
    fi
    echo "[INFO] Building rawes-sim with the current LinkHub ..."
    docker build \
        -f "$SIM_DIR/Dockerfile" \
        "$REPO_DIR" \
        -t rawes-sim \
        --build-arg "ARDUPILOT_RUNTIME_IMAGE=$ardupilot_image" \
        --target runtime-ardupilot
    echo "[INFO] Build complete.  Run stack tests: bash test.sh -n 8"
}

_setup_build_lite() {
    echo "[INFO] Building rawes-sim (target=runtime, no ArduPilot or LinkHub) ..."
    docker build -f "$SIM_DIR/Dockerfile" "$REPO_DIR" -t rawes-sim --target runtime
    echo "[INFO] Build complete.  Run stack tests: bash test.sh -n 8"
}

# --- Pixhawk hardware --------------------------------------------------
_setup_hw() {
    if [ ! -x "$PYTHON" ]; then
        echo "[ERROR] $PYTHON not found.  Run 'bash setup.sh' first." >&2
        exit 1
    fi

    if [ -z "${RAWES_HIL_PORT:-}" ]; then
        echo "[ERROR] RAWES_HIL_PORT is required (example: COM4)." >&2
        exit 1
    fi

    # Reuse calibrate (python -m calibrate) as the canonical hardware param writer.
    # This checks all expected params from rawes_params.json and writes DIFFs.
    "$PYTHON" -m calibrate \
        --connection "$RAWES_HIL_PORT" \
        --baud "${RAWES_HIL_BAUD:-115200}" \
        config apply
}

CMD="${1:-}"
shift || true

case "$CMD" in
    ""|venv)        _setup_venv ;;
    build)          _setup_build ;;
    build-lite)     _setup_build_lite ;;
    hw)             _setup_hw "$@" ;;
    -h|--help)
        sed -n '3,17p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
        ;;
    *)
        echo "[ERROR] Unknown subcommand: $CMD" >&2
        echo "Expected: (no args) | build | build-lite | hw" >&2
        exit 1
        ;;
esac
