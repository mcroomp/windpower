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

# Prevent Git Bash from converting /rawes/... Linux paths to Windows paths
# when passing them as arguments to docker exec.
export MSYS_NO_PATHCONV=1

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
        # Filter modes passed to run_tests.py via RAWES_FILTER_MODE:
        #   (default)      summary  — PASSED/FAILED lines + failure details only
        #   --filterstatus failures — failure sections only
        #   --raw          all      — full unfiltered output
        _FILTER_MODE=summary
        _PASS_ARGS=()
        for _arg in "$@"; do
            case "$_arg" in
                --filterstatus) _FILTER_MODE=failures ;;
                --raw)          _FILTER_MODE=all ;;
                *)              _PASS_ARGS+=("$_arg") ;;
            esac
        done

        _rc=0
        docker exec "$CONTAINER" \
            env RAWES_FILTER_MODE="$_FILTER_MODE" \
            bash /rawes/simulation/test_stack.sh ${_PASS_ARGS[@]+"${_PASS_ARGS[@]}"} || _rc=$?

        _WIN_LOGS=$(cygpath -w "$SCRIPT_DIR/logs" 2>/dev/null || echo "simulation\\logs")
        echo ""
        echo "[LOGS] summary: ${_WIN_LOGS}\\pytest_last_run_summary.json"
        echo "[LOGS] full:    ${_WIN_LOGS}\\pytest_last_run.log"

        exit $_rc
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
    test-torque)
        # Alias for test-stack: torque tests now live in tests/stack/.
        # All args forwarded unchanged so "-k torque_armed" etc. work as before.
        bash "$SCRIPT_DIR/dev.sh" test-stack "$@"
        ;;
    "")
        echo "Usage: $0 start | stop | shell | exec <cmd...> | test-stack [...] | test-unit [...] | test-simtest [...] | test-torque [...]"
        exit 0
        ;;
    *)
        echo "[ERROR] Unknown command: $CMD"
        exit 1
        ;;
esac
