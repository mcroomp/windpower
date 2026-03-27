#!/usr/bin/env bash
#
# dev.sh — manage the persistent rawes-dev container
#
# Usage (from WSL):
#   ./dev.sh start              start the container (idempotent)
#   ./dev.sh stop               stop and remove the container
#   ./dev.sh shell              interactive bash shell
#   ./dev.sh exec <cmd...>      run a command in the container
#   ./dev.sh test-stack [...]   run stack integration tests (extra args forwarded)
#   ./dev.sh test-unit  [...]   run unit tests in container (extra args forwarded)
#
set -euo pipefail

IMAGE=rawes-sim
CONTAINER=rawes-dev
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

ensure_running() {
    if docker inspect --format "{{.State.Running}}" "$CONTAINER" 2>/dev/null | grep -q "^true$"; then
        return 0
    fi
    # Remove stopped/crashed container if it exists
    docker rm -f "$CONTAINER" 2>/dev/null || true
    echo "[INFO] Starting container '$CONTAINER' (simulation/ mounted at /rawes/simulation) ..."
    docker run -d --name "$CONTAINER" \
        -v "${SCRIPT_DIR}:/rawes/simulation" \
        "$IMAGE" sleep infinity >/dev/null
    echo "[INFO] Container '$CONTAINER' is ready."
}

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

CMD="${1:-}"
shift || true

case "$CMD" in
    start)
        ensure_running
        ;;
    stop)
        echo "[INFO] Stopping and removing container '$CONTAINER' ..."
        docker rm -f "$CONTAINER" 2>/dev/null || true
        echo "[INFO] Done."
        ;;
    shell)
        ensure_running
        docker exec -it "$CONTAINER" bash
        ;;
    exec)
        ensure_running
        docker exec "$CONTAINER" bash -c "$*"
        ;;
    test-stack)
        ensure_running
        docker exec "$CONTAINER" bash /rawes/simulation/test_stack.sh "$@"
        ;;
    test-unit)
        ensure_running
        docker exec "$CONTAINER" bash -c "cd /rawes && .venv/bin/python -m pytest simulation/tests/unit $*"
        ;;
    "")
        echo "Usage: $0 start | stop | shell | exec <cmd...> | test-stack [...] | test-unit [...]"
        exit 0
        ;;
    *)
        echo "[ERROR] Unknown command: $CMD"
        exit 1
        ;;
esac
