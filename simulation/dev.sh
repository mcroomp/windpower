#!/usr/bin/env bash
#
# dev.sh — manage the persistent rawes-dev container
#
# Usage (from WSL):
#   ./dev.sh start                      start the container (idempotent)
#   ./dev.sh stop                       stop and remove the container
#   ./dev.sh shell                      interactive bash shell
#   ./dev.sh exec <cmd...>              run a command in the container
#   ./dev.sh test-stack [...]           run stack integration tests (extra args forwarded)
#   ./dev.sh test-stack --filterstatus  same but only print key status lines (streaming)
#   ./dev.sh test-unit  [...]           run unit tests in container (extra args forwarded)
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
        # Extract --filterstatus from the arg list; pass everything else to pytest.
        # With --filterstatus, stdout is piped through line-buffered grep so only
        # key status lines appear in the terminal.  Full output is still written to
        # pytest_last_run.log inside the container by test_stack.sh.
        _FILTER=0
        _PASS_ARGS=()
        for _arg in "$@"; do
            if [ "$_arg" = "--filterstatus" ]; then
                _FILTER=1
            else
                _PASS_ARGS+=("$_arg")
            fi
        done
        _STATUS_PAT='PASS|FAIL|STATUSTEXT|EKF Failsafe|GPS Glitch|yaw aligned|hold complete|ACRO hold|t=[0-9]|Min ENU alt|Drift:|setup [0-9]/6|acro_armed +INFO +\[setup|acro_armed +WARNING|Hold controller'
        if [ "$_FILTER" = "1" ]; then
            docker exec -t "$CONTAINER" bash /rawes/simulation/test_stack.sh "${_PASS_ARGS[@]}" \
                | grep -E "$_STATUS_PAT" --line-buffered
        else
            docker exec -t "$CONTAINER" bash /rawes/simulation/test_stack.sh "${_PASS_ARGS[@]}"
        fi
        ;;
    test-unit)
        ensure_running
        _FILTER=0
        _PASS_ARGS=()
        for _arg in "$@"; do
            if [ "$_arg" = "--filterstatus" ]; then
                _FILTER=1
            else
                _PASS_ARGS+=("$_arg")
            fi
        done
        _STATUS_PAT='PASS|FAIL|PASSED|FAILED|ERROR|passed|failed|error|warning'
        if [ "$_FILTER" = "1" ]; then
            docker exec -t "$CONTAINER" bash /rawes/simulation/test_unit.sh "${_PASS_ARGS[@]}" \
                | grep -E "$_STATUS_PAT" --line-buffered
        else
            docker exec -t "$CONTAINER" bash /rawes/simulation/test_unit.sh "${_PASS_ARGS[@]}"
        fi
        ;;
    test-simtest)
        ensure_running
        _FILTER=0
        _PASS_ARGS=()
        for _arg in "$@"; do
            if [ "$_arg" = "--filterstatus" ]; then
                _FILTER=1
            else
                _PASS_ARGS+=("$_arg")
            fi
        done
        _STATUS_PAT='PASS|FAIL|PASSED|FAILED|ERROR|passed|failed|error|warning'
        if [ "$_FILTER" = "1" ]; then
            docker exec -t "$CONTAINER" bash /rawes/simulation/test_simtest.sh "${_PASS_ARGS[@]}" \
                | grep -E "$_STATUS_PAT" --line-buffered
        else
            docker exec -t "$CONTAINER" bash /rawes/simulation/test_simtest.sh "${_PASS_ARGS[@]}"
        fi
        ;;
    "")
        echo "Usage: $0 start | stop | shell | exec <cmd...> | test-stack [...] | test-unit [...] | test-simtest [...]"
        exit 0
        ;;
    *)
        echo "[ERROR] Unknown command: $CMD"
        exit 1
        ;;
esac
