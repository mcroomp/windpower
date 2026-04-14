#!/usr/bin/env bash
#
# dev.sh — manage the rawes-dev container
#
# Code is synced into the container on every start/restart.
# Logs are copied out after test-stack runs.
#
# Usage:
#   ./dev.sh start                        start container and sync code
#   ./dev.sh stop                         stop and remove container
#   ./dev.sh sync                         re-sync code into running container
#   ./dev.sh shell                        interactive bash shell
#   ./dev.sh exec <cmd...>               run a command in the container
#   ./dev.sh test-stack [...]            run stack tests sequentially
#   ./dev.sh test-stack-parallel [-n N]  run stack tests in parallel (default: 4 workers)
#   ./dev.sh test-stack-parallel --fresh [-n N]  one fresh container per test (fully isolated)
#   ./dev.sh test-unit  [...]            run unit tests
#   ./dev.sh test-simtest [...]          run simtests
#
set -euo pipefail

export MSYS_NO_PATHCONV=1

IMAGE=rawes-sim
CONTAINER=rawes-dev
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------------------
# Code sync / log retrieval
# ---------------------------------------------------------------------------

_sync_code() {
    local _c="${1:-$CONTAINER}"
    echo "[INFO] Syncing code to container $_c..."
    docker exec "$_c" mkdir -p /rawes/simulation/logs
    tar -C "$SCRIPT_DIR" \
        --exclude="./logs" \
        --exclude="./__pycache__" \
        --exclude="*/__pycache__" \
        --exclude="./eeprom*.bin" \
        --exclude="./tests/unit" \
        --exclude="./tests/hil" \
        --exclude="./.venv" \
        -cf - . \
    | docker exec -i "$_c" tar -xf - -C /rawes/simulation/
    echo "[INFO] Code sync complete."
}

_retrieve_logs() {
    local _c="${1:-$CONTAINER}"
    mkdir -p "$SCRIPT_DIR/logs"
    # Docker on Windows needs a native Windows path; cygpath converts /e/... → E:\...
    local _host_logs
    _host_logs=$(cygpath -w "$SCRIPT_DIR/logs" 2>/dev/null || echo "$SCRIPT_DIR/logs")
    docker cp "$_c:/rawes/simulation/logs/." "$_host_logs/" 2>/dev/null || true
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_snap_procs() {
    # Snapshot PIDs of RAWES simulation processes inside all running containers.
    # Returns lines of "<container> <pid> <cmdline>" for each match.
    local _out=""
    local _cs
    _cs=$(docker ps --filter "name=rawes-" --format "{{.Names}}" 2>/dev/null || true)
    for _ct in $_cs; do
        local _hits
        # Exclude the pgrep process itself: -f matches full command line, so the
        # pgrep invocation (which contains "mediator\.py") would otherwise match itself.
        _hits=$(docker exec "$_ct" bash -c \
            "pgrep -a -f 'arducopter|sim_vehicle|mediator\.py' 2>/dev/null | grep -v 'pgrep' || true" \
            2>/dev/null || true)
        if [ -n "$_hits" ]; then
            while IFS= read -r _line; do
                _out+="${_ct} ${_line}"$'\n'
            done <<< "$_hits"
        fi
    done
    echo -n "$_out"
}

_warn_new_procs() {
    # Compare a before-snapshot with the current state; warn about new orphans.
    local _before="$1"
    local _after
    _after=$(_snap_procs)
    [ -z "$_after" ] && return

    # Build set of "container:pid" tokens seen before the run
    local _before_keys
    _before_keys=$(echo "$_before" | awk '{print $1 ":" $2}' | sort)

    local _new_lines=""
    while IFS= read -r _line; do
        [ -z "$_line" ] && continue
        local _key
        _key=$(echo "$_line" | awk '{print $1 ":" $2}')
        if ! echo "$_before_keys" | grep -qF "$_key"; then
            _new_lines+="  $_line"$'\n'
        fi
    done <<< "$_after"

    if [ -n "$_new_lines" ]; then
        echo "[WARN] Orphaned simulation processes still running after tests:"
        echo -n "$_new_lines"
    fi
}

_cleanup_orphan_containers() {
    local _orphans
    _orphans=$(docker ps -a --filter "name=rawes-parallel-" --format "{{.Names}}" 2>/dev/null || true)
    if [ -n "$_orphans" ]; then
        echo "[INFO] Removing orphan parallel containers:"
        echo "$_orphans" | sed 's/^/  /'
        echo "$_orphans" | xargs docker rm -f 2>/dev/null || true
    fi
}

ensure_running() {
    _cleanup_orphan_containers
    if docker inspect --format "{{.State.Running}}" "$CONTAINER" 2>/dev/null | grep -q "^true$"; then
        _sync_code "$CONTAINER"
        return 0
    fi
    docker rm -f "$CONTAINER" 2>/dev/null || true
    echo "[INFO] Starting container '$CONTAINER'..."
    docker run -d --name "$CONTAINER" "$IMAGE" sleep infinity >/dev/null
    echo "[INFO] Container '$CONTAINER' is ready."
    _sync_code "$CONTAINER"
}

health_check() {
    local stale=0
    local procs
    procs=$(docker exec "$CONTAINER" bash -c \
        "pgrep -a -f 'arducopter|sim_vehicle|mediator\.py' 2>/dev/null || true" 2>/dev/null || true)
    if [ -n "$procs" ]; then
        echo "[WARN] Stale simulation processes found:"
        echo "$procs" | sed 's/^/  /'
        stale=1
    fi
    local ports
    ports=$(docker exec "$CONTAINER" bash -c \
        "ss -tlnpu 2>/dev/null | grep -E ':5760|:9002' || true" 2>/dev/null || true)
    if [ -n "$ports" ]; then
        echo "[WARN] Test ports still bound inside container:"
        echo "$ports" | sed 's/^/  /'
        stale=1
    fi
    if [ "$stale" = "1" ]; then
        echo "[INFO] Restarting container to clear stale state..."
        docker rm -f "$CONTAINER" 2>/dev/null || true
        docker run -d --name "$CONTAINER" "$IMAGE" sleep infinity >/dev/null
        echo "[INFO] Container restarted and ready."
        _sync_code "$CONTAINER"
    fi
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
    sync)
        _sync_code "$CONTAINER"
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
        health_check
        _FILTER_MODE=summary
        _PASS_ARGS=()
        for _arg in "$@"; do
            case "$_arg" in
                --filterstatus) _FILTER_MODE=failures ;;
                --raw)          _FILTER_MODE=all ;;
                *)              _PASS_ARGS+=("$_arg") ;;
            esac
        done
        # Clear host and container log dirs before the run so only this run's
        # logs are present after retrieval.
        rm -rf "$SCRIPT_DIR/logs"/*/
        docker exec "$CONTAINER" bash -c "rm -rf /rawes/simulation/logs && mkdir -p /rawes/simulation/logs"
        _rc=0
        docker exec "$CONTAINER" \
            env RAWES_FILTER_MODE="$_FILTER_MODE" \
            bash /rawes/simulation/test_stack.sh ${_PASS_ARGS[@]+"${_PASS_ARGS[@]}"} || _rc=$?
        _retrieve_logs "$CONTAINER"
        _WIN_LOGS=$(cygpath -w "$SCRIPT_DIR/logs" 2>/dev/null || echo "simulation\\logs")
        echo ""
        echo "[LOGS] summary: ${_WIN_LOGS}\\suite_summary.json"
        exit $_rc
        ;;

    test-stack-parallel)
        # Parse args: -n N sets worker count, --fresh = one container per test, rest forwarded to pytest
        _N_WORKERS=4
        _FRESH=0
        _PASS_ARGS=()
        while [[ $# -gt 0 ]]; do
            case "$1" in
                -n) shift; _N_WORKERS="$1" ;;
                -n[0-9]*) _N_WORKERS="${1#-n}" ;;
                --fresh) _FRESH=1 ;;
                *) _PASS_ARGS+=("$1") ;;
            esac
            shift
        done

        _RUN_ID=$(date +%s)
        declare -a _CONTAINERS=()
        declare -a _WORKER_LOGS=()

        _parallel_cleanup() {
            echo ""
            echo "[INFO] Cleaning up parallel containers..."
            for _c in "${_CONTAINERS[@]+"${_CONTAINERS[@]}"}"; do
                docker rm -f "$_c" 2>/dev/null || true
            done
        }
        trap _parallel_cleanup EXIT INT TERM

        # Discover test files from the host filesystem (no container needed)
        mapfile -t _ALL_FILES < <(ls "$SCRIPT_DIR/tests/stack/test_"*.py | sort)
        _N_FILES=${#_ALL_FILES[@]}

        # Clear host log dirs before the run so only this run's logs survive.
        rm -rf "$SCRIPT_DIR/logs"/*/

        # Snapshot any simulation processes already running before we start
        _PROCS_BEFORE=$(_snap_procs)

        if [ "$_FRESH" = "1" ]; then
            # Fresh mode: each test file gets its own brand-new container (clean EEPROM,
            # no stale processes).  Run up to _N_WORKERS containers concurrently.
            echo "[INFO] Fresh mode: $_N_FILES tests, max $_N_WORKERS concurrent (run=$_RUN_ID)"

            # Parallel arrays: _ACTIVE_PIDS[i] is the subshell PID for
            # _ACTIVE_CTRS[i].  The parent shell removes containers here so cleanup
            # is guaranteed even when the subshell's docker rm -f fails silently.
            declare -a _ACTIVE_PIDS=()
            declare -a _ACTIVE_CTRS=()
            _RC=0

            _reap_finished() {
                local _still_pids=() _still_ctrs=() _p _pc i
                for i in "${!_ACTIVE_PIDS[@]}"; do
                    _p="${_ACTIVE_PIDS[$i]}"
                    _pc="${_ACTIVE_CTRS[$i]}"
                    if kill -0 "$_p" 2>/dev/null; then
                        _still_pids+=("$_p")
                        _still_ctrs+=("$_pc")
                    else
                        wait "$_p" || _RC=1
                        docker rm -f "$_pc" >/dev/null 2>&1 || true
                    fi
                done
                _ACTIVE_PIDS=("${_still_pids[@]+"${_still_pids[@]}"}")
                _ACTIVE_CTRS=("${_still_ctrs[@]+"${_still_ctrs[@]}"}")
            }

            for j in $(seq 0 $((_N_FILES-1))); do
                # Throttle: poll until a concurrency slot opens.
                while [ "${#_ACTIVE_PIDS[@]}" -ge "$_N_WORKERS" ]; do
                    _reap_finished
                    [ "${#_ACTIVE_PIDS[@]}" -ge "$_N_WORKERS" ] && sleep 0.5
                done

                _c="rawes-parallel-${_RUN_ID}-${j}"
                _CONTAINERS+=("$_c")
                _f="/rawes/simulation/tests/stack/$(basename "${_ALL_FILES[$j]}")"
                _log="/tmp/rawes-parallel-${_RUN_ID}-t${j}.log"
                _WORKER_LOGS+=("$_log")
                _label="$(basename "${_ALL_FILES[$j]}" .py)"

                echo "[t${j}] starting: $_label"
                (
                    docker run -d --name "$_c" "$IMAGE" sleep infinity >/dev/null 2>&1
                    _sync_code "$_c" >/dev/null 2>&1
                    # Clear container logs so each test starts with a blank slate.
                    docker exec "$_c" bash -c "rm -rf /rawes/simulation/logs && mkdir -p /rawes/simulation/logs"
                    # Run pytest, capturing exit code without letting set -e abort the subshell on
                    # test failure (docker exec returns 1 when pytest finds failures; pipefail would
                    # propagate that and skip _retrieve_logs).
                    _test_rc=0
                    docker exec \
                        -e RAWES_RUN_STACK_INTEGRATION=1 \
                        -e RAWES_SIM_VEHICLE=/ardupilot/Tools/autotest/sim_vehicle.py \
                        "$_c" \
                        /rawes/.venv/bin/python -m pytest "$_f" -s -v \
                        ${_PASS_ARGS[@]+"${_PASS_ARGS[@]}"} 2>&1 \
                    | tee "$_log" \
                    | grep --line-buffered -E "PASSED|FAILED|ERROR|passed|failed|error" \
                    | sed "s/^/[${_label}] /" \
                    || _test_rc=$?
                    _retrieve_logs "$_c"
                    # Best-effort in-subshell removal; parent also removes via _reap_finished.
                    docker rm -f "$_c" >/dev/null 2>&1 || true
                    exit $_test_rc
                ) &
                _ACTIVE_PIDS+=($!)
                _ACTIVE_CTRS+=("$_c")
            done

            # Drain remaining jobs, removing their containers as they finish.
            while [ "${#_ACTIVE_PIDS[@]}" -gt 0 ]; do
                _reap_finished
                [ "${#_ACTIVE_PIDS[@]}" -gt 0 ] && sleep 0.5
            done

            _warn_new_procs "$_PROCS_BEFORE"

            # Print per-test summary
            echo ""
            echo "----------------------------------------------------------------------"
            for j in $(seq 0 $((_N_FILES-1))); do
                _log="${_WORKER_LOGS[$j]}"
                _label="$(basename "${_ALL_FILES[$j]}" .py)"
                _summary=$(grep -E "^=+ .* in [0-9]" "$_log" 2>/dev/null | tail -1 || echo "(no summary)")
                printf "[%-42s] %s\n" "$_label" "$_summary"
            done
            echo "----------------------------------------------------------------------"

        else
            # Round-robin mode: N workers share all test files.
            _ACTUAL_WORKERS=$(( _N_WORKERS < _N_FILES ? _N_WORKERS : _N_FILES ))
            echo "[INFO] $_N_FILES test files / $_ACTUAL_WORKERS workers (run=$_RUN_ID)"

            # Start containers
            for i in $(seq 0 $((_ACTUAL_WORKERS-1))); do
                _c="rawes-parallel-${_RUN_ID}-${i}"
                _CONTAINERS+=("$_c")
                docker run -d --name "$_c" "$IMAGE" sleep infinity >/dev/null
            done

            # Sync code to all containers in parallel
            for _c in "${_CONTAINERS[@]}"; do
                _sync_code "$_c" &
            done
            wait
            echo "[INFO] All $_ACTUAL_WORKERS containers ready."

            # Distribute test files round-robin across workers
            declare -a _WORKER_FILE_ARGS
            for i in $(seq 0 $((_ACTUAL_WORKERS-1))); do
                _WORKER_FILE_ARGS[$i]=""
            done
            for j in $(seq 0 $((_N_FILES-1))); do
                _w=$(( j % _ACTUAL_WORKERS ))
                _f="/rawes/simulation/tests/stack/$(basename "${_ALL_FILES[$j]}")"
                _WORKER_FILE_ARGS[$_w]+=" $_f"
            done

            # Launch workers — stream PASS/FAIL lines with worker prefix for progress
            declare -a _PIDS=()
            for i in $(seq 0 $((_ACTUAL_WORKERS-1))); do
                _c="${_CONTAINERS[$i]}"
                _files="${_WORKER_FILE_ARGS[$i]}"
                _log="/tmp/rawes-parallel-${_RUN_ID}-w${i}.log"
                _WORKER_LOGS+=("$_log")
                echo "[w${i}] starting:$(echo "$_files" | xargs -n1 basename | tr '\n' ' ')"
                (
                    docker exec "$_c" \
                        /rawes/.venv/bin/python -m pytest $_files -s -v \
                        ${_PASS_ARGS[@]+"${_PASS_ARGS[@]}"} 2>&1 \
                    | tee "$_log" \
                    | grep --line-buffered -E "PASSED|FAILED|ERROR|passed|failed|error" \
                    | sed "s/^/[w${i}] /"
                ) &
                _PIDS+=($!)
            done

            # Wait for all workers
            _RC=0
            for _pid in "${_PIDS[@]}"; do
                wait "$_pid" || _RC=1
            done

            # Retrieve logs from all containers
            for _c in "${_CONTAINERS[@]}"; do
                _retrieve_logs "$_c"
            done

            _warn_new_procs "$_PROCS_BEFORE"

            # Print per-worker summary lines
            echo ""
            echo "----------------------------------------------------------------------"
            for i in $(seq 0 $((_ACTUAL_WORKERS-1))); do
                _log="${_WORKER_LOGS[$i]}"
                _summary=$(grep -E "^=+ .* in [0-9]" "$_log" 2>/dev/null | tail -1 || echo "(no summary)")
                echo "[w${i}] $_summary"
            done
            echo "----------------------------------------------------------------------"

        fi

        _WIN_LOGS=$(cygpath -w "$SCRIPT_DIR/logs" 2>/dev/null || echo "simulation\\logs")
        echo "[LOGS] ${_WIN_LOGS}"
        exit $_RC
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
        bash "$SCRIPT_DIR/dev.sh" test-stack "$@"
        ;;
    "")
        echo "Usage: $0 start|stop|sync|shell|exec <cmd>|test-stack [...]|test-stack-parallel [-n N] [--fresh] [...]|test-unit [...]|test-simtest [...]"
        exit 0
        ;;
    *)
        echo "[ERROR] Unknown command: $CMD"
        exit 1
        ;;
esac
