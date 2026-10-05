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

# Docker builds on this Windows workstation are WSL-only. Keep Windows venv
# and hardware setup in Git Bash, but self-reinvoke Docker subcommands in WSL.
if [[ ("${1:-}" == "build" || "${1:-}" == "build-lite") \
      && -n "${MSYSTEM:-}" && -z "${WSL_DISTRO_NAME:-}" ]]; then
    if ! command -v wsl.exe >/dev/null 2>&1; then
        echo "[ERROR] WSL is required for RAWES Docker builds." >&2
        exit 1
    fi
    _drive="${REPO_DIR:1:1}"
    _wsl_dir="/mnt/${_drive,,}${REPO_DIR:2}"
    _rebuild_ardupilot="$(printf '%q' "${RAWES_REBUILD_ARDUPILOT:-0}")"
    exec wsl.exe -e bash -lc \
        "cd '$_wsl_dir' && RAWES_REBUILD_ARDUPILOT=$_rebuild_ardupilot bash setup.sh $(printf '%q ' "$@")"
fi

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

    if ! "$PYTHON" -c "import linkhub_client.client" >/dev/null 2>&1; then
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
    local ardupilot_image="rawes-sim-ardupilot-base:Copter-4.7.1-v1"
    local image_hash
    local existing_hash
    image_hash="$(bash "$REPO_DIR/scripts/docker_image_hash.sh")"
    existing_hash="$(
        docker image inspect rawes-sim \
            --format '{{ index .Config.Labels "org.rawes.image-input-hash" }}' \
            2>/dev/null || true
    )"

    if [ "${RAWES_REBUILD_ARDUPILOT:-0}" != "1" ] && [ "$existing_hash" = "$image_hash" ]; then
        echo "[INFO] rawes-sim build inputs unchanged ($image_hash) -- skipping Docker build"
        docker run --rm --entrypoint /bin/bash rawes-sim -lc \
            '/rawes/.venv/bin/python -c "import sys; assert sys.version_info >= (3, 12), sys.version" \
             && test -x /ardupilot/build/sitl/bin/arducopter-heli \
             && command -v linkhub >/dev/null'
        return
    fi

    local -a cache_args=()
    if [ "${RAWES_REBUILD_ARDUPILOT:-0}" = "1" ]; then
        cache_args+=(--no-cache)
        echo "[INFO] Rebuilding $ardupilot_image without cache -- expect ~30-60 min ..."
    else
        echo "[INFO] Verifying/building $ardupilot_image with Docker layer cache ..."
    fi
    # Always ask Docker to build the base target. An unchanged image is a cheap
    # cache hit, while Python requirements and runtime-stage changes are picked
    # up without rebuilding the independent ArduPilot compilation stage.
    docker build \
        -f "$SIM_DIR/Dockerfile" \
        "$REPO_DIR" \
        -t "$ardupilot_image" \
        --target runtime-ardupilot-base \
        "${cache_args[@]}"
    echo "[INFO] Building rawes-sim with the current LinkHub ..."
    docker build \
        -f "$SIM_DIR/Dockerfile" \
        "$REPO_DIR" \
        -t rawes-sim \
        --build-arg "ARDUPILOT_RUNTIME_IMAGE=$ardupilot_image" \
        --build-arg "RAWES_IMAGE_INPUT_HASH=$image_hash" \
        --target runtime-ardupilot
    local built_hash
    built_hash="$(
        docker image inspect rawes-sim \
            --format '{{ index .Config.Labels "org.rawes.image-input-hash" }}'
    )"
    if [ "$built_hash" != "$image_hash" ]; then
        echo "[ERROR] rawes-sim image hash label was not updated after build" >&2
        echo "        expected: $image_hash" >&2
        echo "        actual:   $built_hash" >&2
        return 1
    fi
    docker run --rm --entrypoint /bin/bash rawes-sim -lc \
        '/rawes/.venv/bin/python -c "import sys; assert sys.version_info >= (3, 12), sys.version" \
         && test -x /ardupilot/build/sitl/bin/arducopter-heli \
         && command -v linkhub >/dev/null'
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
        config fix
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
