#!/usr/bin/env bash
#
# test.sh -- single entry point for RAWES tests and Docker dev container.
#
# Test suites:
#   bash test.sh unit     [pytest args...]   # Windows venv, no Docker
#   bash test.sh simtest  [pytest args...]   # Windows venv, no Docker
#   bash test.sh stack    [-n N] [pytest args...]   # Docker (one container per test file)
#   bash test.sh hil      [pytest args...]   # HIL pytest against a real Pixhawk on USB
#                                              requires RAWES_HIL_PORT=COMx
#
# Docker dev container (rawes-dev) lifecycle:
#   bash test.sh start                       # start + sync code
#   bash test.sh stop                        # stop and remove
#   bash test.sh sync                        # re-sync code into running container
#   bash test.sh shell                       # interactive bash
#   bash test.sh exec <cmd...>               # run command in container
#
# Examples:
#   bash test.sh unit                        # all unit tests
#   bash test.sh unit -k test_foo -s         # one unit test, verbose
#   bash test.sh simtest -k pumping          # one simtest
#   bash test.sh stack -n 1 -k test_foo      # one stack test
#   bash test.sh stack -n 8                  # full stack suite, 8 workers
#   RAWES_HIL_PORT=COM4 bash test.sh hil -v  # HIL smoke tests
#
set -euo pipefail
export MSYS_NO_PATHCONV=1

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$REPO_DIR/simulation"
VENV_PY="$SIM_DIR/.venv/Scripts/python.exe"

IMAGE=rawes-sim
CONTAINER=rawes-dev

_winpath() { cygpath -w "$1"; }
_log() { echo "$(date +%H:%M:%S) $*"; }

_require_venv() {
    if [ ! -x "$VENV_PY" ]; then
        echo "[ERROR] $VENV_PY not found.  Run setup.cmd first." >&2
        exit 1
    fi
}

# ---------------------------------------------------------------------------
# Code sync / log retrieval
# ---------------------------------------------------------------------------

_sync_code() {
    local _c="${1:-$CONTAINER}"
    echo "[INFO] Syncing code to container $_c..."
    docker exec "$_c" mkdir -p /rawes/simulation/logs
    tar -C "$SIM_DIR" \
        --exclude="./logs" \
        --exclude="./__pycache__" \
        --exclude="*/__pycache__" \
        --exclude="./eeprom*.bin" \
        --exclude="./tests/unit" \
        --exclude="./tests/hil" \
        --exclude="./.venv" \
        -cf - . \
    | docker exec -i "$_c" tar -xf - -C /rawes/simulation/
    # Sync the sibling aero package (editable-installed on host from ../aero
    # relative to the repo root).
    local _AERO_DIR="$REPO_DIR/../aero"
    if [ -d "$_AERO_DIR" ]; then
        docker exec "$_c" mkdir -p /rawes
        tar -C "$_AERO_DIR" \
            --exclude="./__pycache__" \
            --exclude="*/__pycache__" \
            --exclude="./.venv" \
            --exclude="./tests" \
            --exclude="./out" \
            --exclude="./Research" \
            --exclude="./envelope" \
            -cf - aero \
        | docker exec -i "$_c" tar -xf - -C /rawes/
    fi
    echo "[INFO] Code sync complete."
}

_retrieve_logs() {
    local _c="${1:-$CONTAINER}"
    mkdir -p "$SIM_DIR/logs"
    local _host_logs
    _host_logs=$(cygpath -w "$SIM_DIR/logs" 2>/dev/null || echo "$SIM_DIR/logs")
    docker cp "$_c:/rawes/simulation/logs/." "$_host_logs/" 2>/dev/null || true
}

# ---------------------------------------------------------------------------
# Orphan / stale process helpers
# ---------------------------------------------------------------------------

_snap_procs() {
    local _out=""
    local _cs
    _cs=$(docker ps --filter "name=rawes-" --format "{{.Names}}" 2>/dev/null || true)
    for _ct in $_cs; do
        local _hits
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
    local _before="$1"
    local _after
    _after=$(_snap_procs)
    [ -z "$_after" ] && return

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
    docker run -d --cap-add=SYS_PTRACE --name "$CONTAINER" "$IMAGE" sleep infinity >/dev/null
    echo "[INFO] Container '$CONTAINER' is ready."
    _sync_code "$CONTAINER"
}

# ---------------------------------------------------------------------------
# Stack-test parallel runner
# ---------------------------------------------------------------------------

_run_stack() {
    local _N_WORKERS=4
    local _PASS_ARGS=()
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -n) shift; _N_WORKERS="$1" ;;
            -n[0-9]*) _N_WORKERS="${1#-n}" ;;
            *) _PASS_ARGS+=("$1") ;;
        esac
        shift
    done

    local _RUN_ID
    _RUN_ID=$(date +%s)
    declare -a _CONTAINERS=()
    declare -a _WORKER_LOGS=()

    _parallel_cleanup() {
        echo ""
        _log "[INFO] Cleaning up parallel containers..."
        for _c in "${_CONTAINERS[@]+"${_CONTAINERS[@]}"}"; do
            docker rm -f "$_c" 2>/dev/null || true
        done
    }
    trap _parallel_cleanup EXIT INT TERM

    mapfile -t _ALL_FILES < <(find "$SIM_DIR/tests/sitl" -name "test_*.py" | sort)

    local _K_EXPR=""
    local _i _next
    for _i in "${!_PASS_ARGS[@]}"; do
        if [ "${_PASS_ARGS[$_i]}" = "-k" ]; then
            _next=$(( _i + 1 ))
            _K_EXPR="${_PASS_ARGS[$_next]:-}"
        fi
    done
    if [ -n "$_K_EXPR" ]; then
        declare -a _MATCHED=()
        local _tf
        for _tf in "${_ALL_FILES[@]}"; do
            if grep -qE "def (test_[a-zA-Z0-9_]*${_K_EXPR}[a-zA-Z0-9_]*|${_K_EXPR}[a-zA-Z0-9_]*)" "$_tf" 2>/dev/null \
               || grep -qF "def ${_K_EXPR}" "$_tf" 2>/dev/null \
               || [[ "$(basename "$_tf" .py)" == *"${_K_EXPR}"* ]]; then
                _MATCHED+=("$_tf")
            fi
        done
        if [ "${#_MATCHED[@]}" -gt 0 ]; then
            _ALL_FILES=("${_MATCHED[@]}")
            _log "[INFO] -k '${_K_EXPR}': pre-filtered to ${#_MATCHED[@]} file(s)"
        fi
    fi

    local _N_FILES=${#_ALL_FILES[@]}

    local _PROCS_BEFORE
    _PROCS_BEFORE=$(_snap_procs)

    _log "[INFO] $_N_FILES tests, max $_N_WORKERS concurrent (run=$_RUN_ID)"

    declare -a _ACTIVE_PIDS=()
    declare -a _ACTIVE_CTRS=()
    local _RC=0

    _reap_finished() {
        local _still_pids=() _still_ctrs=() _p _pc i _wrc
        for i in "${!_ACTIVE_PIDS[@]}"; do
            _p="${_ACTIVE_PIDS[$i]}"
            _pc="${_ACTIVE_CTRS[$i]}"
            if kill -0 "$_p" 2>/dev/null; then
                _still_pids+=("$_p")
                _still_ctrs+=("$_pc")
            else
                _wrc=0
                wait "$_p" || _wrc=$?
                if [ "$_wrc" -ne 0 ] && [ "$_wrc" -ne 5 ]; then
                    _RC=1
                fi
                docker rm -f "$_pc" >/dev/null 2>&1 || true
            fi
        done
        _ACTIVE_PIDS=("${_still_pids[@]+"${_still_pids[@]}"}")
        _ACTIVE_CTRS=("${_still_ctrs[@]+"${_still_ctrs[@]}"}")
    }

    local j _c _f _wlog _label
    for j in $(seq 0 $((_N_FILES-1))); do
        while [ "${#_ACTIVE_PIDS[@]}" -ge "$_N_WORKERS" ]; do
            _reap_finished
            [ "${#_ACTIVE_PIDS[@]}" -ge "$_N_WORKERS" ] && sleep 0.5
        done

        _c="rawes-parallel-${_RUN_ID}-${j}"
        _CONTAINERS+=("$_c")
        _f="/rawes/simulation/${_ALL_FILES[$j]#${SIM_DIR}/}"
        _wlog="/tmp/rawes-parallel-${_RUN_ID}-t${j}.log"
        _WORKER_LOGS+=("$_wlog")
        _label="$(basename "${_ALL_FILES[$j]}" .py)"

        _log "[t${j}] starting: $_label"
        (
            docker run -d --cap-add=SYS_PTRACE --name "$_c" "$IMAGE" sleep infinity >/dev/null 2>&1
            _sync_code "$_c" >/dev/null 2>&1
            docker exec "$_c" bash -c "rm -rf /rawes/simulation/logs && mkdir -p /rawes/simulation/logs"
            _test_rc=0
            docker exec \
                -e RAWES_RUN_STACK_INTEGRATION=1 \
                -e RAWES_SIM_VEHICLE=/ardupilot/Tools/autotest/sim_vehicle.py \
                -e PYTHONPATH=/rawes \
                "$_c" \
                /rawes/.venv/bin/python -m pytest "$_f" -s -v \
                ${_PASS_ARGS[@]+"${_PASS_ARGS[@]}"} 2>&1 \
            | tee "$_wlog" \
            | sed -n -E '/PASSED|FAILED|XFAIL|XPASS|ERROR|passed|failed|xfailed|xpassed|error/p' \
            | awk -v lbl="[${_label}]" '{print strftime("%H:%M:%S") " " lbl " " $0; fflush()}' \
            || _test_rc=$?
            rm -rf "$SIM_DIR/logs/${_label}"
            _retrieve_logs "$_c"
            docker rm -f "$_c" >/dev/null 2>&1 || true
            exit $_test_rc
        ) &
        _ACTIVE_PIDS+=($!)
        _ACTIVE_CTRS+=("$_c")
    done

    while [ "${#_ACTIVE_PIDS[@]}" -gt 0 ]; do
        _reap_finished
        [ "${#_ACTIVE_PIDS[@]}" -gt 0 ] && sleep 0.5
    done

    _warn_new_procs "$_PROCS_BEFORE"

    echo ""
    echo "----------------------------------------------------------------------"
    declare -a _FAILED_LABELS=()
    declare -a _FAILED_WLOGS=()
    local _summary
    for j in $(seq 0 $((_N_FILES-1))); do
        _wlog="${_WORKER_LOGS[$j]}"
        _label="$(basename "${_ALL_FILES[$j]}" .py)"
        _summary=$(grep -E "^=+ .* in [0-9]" "$_wlog" 2>/dev/null | tail -1 || echo "(no summary)")
        printf "[%-42s] %s\n" "$_label" "$_summary"
        if echo "$_summary" | grep -qiE "failed|error"; then
            _FAILED_LABELS+=("$_label")
            _FAILED_WLOGS+=("$_wlog")
        fi
    done
    echo "----------------------------------------------------------------------"

    if [ "${#_FAILED_LABELS[@]}" -gt 0 ]; then
        echo ""
        echo "============================== FAILURE DETAILS =============================="
        local _fi _fl _fw
        for _fi in "${!_FAILED_LABELS[@]}"; do
            _fl="${_FAILED_LABELS[$_fi]}"
            _fw="${_FAILED_WLOGS[$_fi]}"
            echo ""
            echo "--- $_fl ---"
            awk '
                /^=+[ ]+(FAILURES|ERRORS)[ ]=+/ { in_s=1; print; next }
                /^=+[ ]+short test summary/ { in_s=0 }
                in_s { print }
            ' "$_fw" 2>/dev/null | head -100 || true
            grep -E "^(FAILED|ERROR) " "$_fw" 2>/dev/null || true
            echo "---"
        done
        echo ""
        echo "============================================================================="
    fi

    local _WIN_LOGS
    _WIN_LOGS=$(cygpath -w "$SIM_DIR/logs" 2>/dev/null || echo "simulation\\logs")
    _log "[LOGS] ${_WIN_LOGS}"
    return $_RC
}

# ---------------------------------------------------------------------------
# Top-level dispatch
# ---------------------------------------------------------------------------

CMD="${1:-}"
shift || true

case "$CMD" in
    unit)
        _require_venv
        "$VENV_PY" -m pytest "$(_winpath "$SIM_DIR/tests/unit")" -m "not simtest" "$@"
        ;;
    simtest)
        _require_venv
        "$VENV_PY" "$(_winpath "$SIM_DIR/run_tests.py")" \
            "$(_winpath "$SIM_DIR/tests/simtests")" -m simtest "$@"
        ;;
    stack)
        _run_stack "$@"
        ;;
    hil)
        _require_venv
        "$VENV_PY" -m pytest "$(_winpath "$SIM_DIR/tests/hil")" "$@"
        ;;
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
    ""|-h|--help)
        sed -n '3,28p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
        [ -z "$CMD" ] && exit 1 || exit 0
        ;;
    *)
        echo "[ERROR] Unknown command: $CMD" >&2
        echo "Expected: unit | simtest | stack | hil | start | stop | sync | shell | exec" >&2
        exit 1
        ;;
esac
