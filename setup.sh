#!/usr/bin/env bash
#
# setup.sh -- RAWES one-time setup tasks.  All subcommands are idempotent.
## Subcommands:
#   (no args)   sync Python dependencies and build the UI and LinkHub
#                 Windows setup.cmd bootstraps missing tools with WinGet.
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

SIM_DIR="$REPO_DIR/simulation"
VENV="$REPO_DIR/.venv"
PYTHON="$VENV/bin/python"
if [[ -n "${MSYSTEM:-}" ]]; then
    PYTHON="$VENV/Scripts/python.exe"
fi

# --- venv (default) ----------------------------------------------------
_setup_venv() {
    if [[ -n "${MSYSTEM:-}" ]]; then
        (
            cd "$REPO_DIR"
            cmd.exe /d /c setup.cmd
        )
        return
    fi

    if ! command -v uv >/dev/null 2>&1; then
        echo "[ERROR] uv is required. Install it from https://docs.astral.sh/uv/getting-started/installation/" >&2
        return 1
    fi
    for tool in npm cargo; do
        if ! command -v "$tool" >/dev/null 2>&1; then
            echo "[ERROR] $tool is required for project setup." >&2
            return 1
        fi
    done
    echo "[INFO] Installing uv-managed Python 3.12 ..."
    uv python install 3.12
    echo "[INFO] Synchronizing the project environment ..."
    uv sync --project "$REPO_DIR" --python 3.12 --managed-python --locked --dev --extra simulation

    (
        cd "$REPO_DIR/linkhub-ui"
        npm ci --no-audit --no-fund
        npm run build
    )
    echo "[INFO] Building LinkHub ..."
    cargo build \
        --manifest-path "$REPO_DIR/linkhub/Cargo.toml" \
        --release
    uv run --no-sync --project "$REPO_DIR" python --version
    echo "[INFO] Done."
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
