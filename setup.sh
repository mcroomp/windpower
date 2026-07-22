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

    # Hash-gated editable install: only reinstall when pyproject.toml changes.
    # The editable install is what makes `import simulation`/`groundstation`/etc.
    # work from any cwd or when a script is invoked by path (rather than via
    # `python -c` from repo root) -- but re-running pip install -e on every
    # invocation is unnecessary overhead once it's already registered.
    if [ ! -f "$PYPROJECT" ]; then
        echo "[WARN] $PYPROJECT not found -- skipping editable install"
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

    echo "[INFO] Done."
    "$PYTHON" --version
}

# --- Docker image ------------------------------------------------------
_setup_build() {
    echo "[INFO] Building rawes-sim (target=runtime-ardupilot) -- expect ~30-60 min ..."
    docker build "$SIM_DIR" -t rawes-sim --target runtime-ardupilot
    echo "[INFO] Build complete.  Run stack tests: bash test.sh -n 8"
}

_setup_build_lite() {
    echo "[INFO] Building rawes-sim (target=runtime, no ArduPilot) ..."
    docker build "$SIM_DIR" -t rawes-sim --target runtime
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
        --port "$RAWES_HIL_PORT" \
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
