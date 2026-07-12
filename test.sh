#!/usr/bin/env bash
#
# test.sh -- RAWES SITL Docker stack-test runner.
#
# Runs ONLY the ArduPilot SITL integration tests, in Docker, one container per
# test file, up to N in parallel.  The SITL stack is the only suite that needs
# Docker.  Every other suite runs with plain pytest in the Windows venv:
#
#   .venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest"
#   .venv/Scripts/python.exe simulation/run_tests.py simulation/tests/simtests -m simtest
#   .venv/Scripts/python.exe -m pytest simulation/tests/hil   # needs RAWES_HIL_PORT=COMx
#
# Usage:
#   bash test.sh [-n N] [pytest args...]         # run the SITL stack suite
#   bash test.sh stack [-n N] [pytest args...]   # same, explicit subcommand
#
# Examples:
#   bash test.sh -n 8                # full stack suite, 8 workers
#   bash test.sh -n 1 -k test_foo    # a single stack test
#
# Suppress path mangling in MSYS/Git-for-Windows; harmless elsewhere.
[[ -n "${MSYSTEM:-}" ]] && export MSYS_NO_PATHCONV=1

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$REPO_DIR/simulation"

IMAGE=rawes-sim

_log() { echo "$(date +%H:%M:%S) $*"; }

# ---------------------------------------------------------------------------
# Code sync / log retrieval
# ---------------------------------------------------------------------------

_sync_code() {
    local _c="$1"
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
    # Sync the sibling aero workspace (editable-installed on host from ../aero
    # relative to the repo root) for local development convenience.
    local _AERO_DIR="$REPO_DIR/../aero"
    if [ -d "$_AERO_DIR" ]; then
        docker exec "$_c" mkdir -p /rawes
        tar -C "$_AERO_DIR" \
            --exclude="./__pycache__" \
            --exclude="*/__pycache__" \
            --exclude="./.venv" \
            --exclude="./tests" \
            --exclude="./target" \
            --exclude="./out" \
            --exclude="./Research" \
            --exclude="./envelope" \
            -cf - Cargo.toml Cargo.lock pyproject.toml dynbem dynbem_rs aero \
        | docker exec -i "$_c" tar -xf - -C /rawes/
        docker exec "$_c" bash -lc 'python - <<"PY"
import importlib.metadata as m
import pathlib
import re
import subprocess
import sys

req_path = pathlib.Path("/rawes/simulation/requirements.txt")
if not req_path.exists():
    print(f"[ERROR] requirements file not found: {req_path}", file=sys.stderr)
    raise SystemExit(2)

required = None
for raw in req_path.read_text(encoding="utf-8").splitlines():
    line = raw.split("#", 1)[0].strip()
    if not line:
        continue
    m_req = re.match(r"^dynbem\s*==\s*([A-Za-z0-9_.+-]+)$", line)
    if m_req:
        required = m_req.group(1)
        break

if required is None:
    print("[ERROR] dynbem exact pin (dynbem==<version>) missing in requirements.txt; aborting sync.", file=sys.stderr)
    raise SystemExit(2)

try:
    version = m.version("dynbem")
except m.PackageNotFoundError:
    version = None

if version != required:
    try:
        # Install dynbem from PyPI (wheel-only) to avoid local Rust builds in container.
        subprocess.check_call([
            sys.executable,
            "-m",
            "pip",
            "install",
            "-q",
            "--upgrade",
            "--only-binary=:all:",
            f"dynbem=={required}",
        ])
    except subprocess.CalledProcessError as exc:
        print(f"[ERROR] dynbem=={required} PyPI wheel install failed; aborting sync.", file=sys.stderr)
        raise SystemExit(exc.returncode or 2)

# Hard guard: abort if dynbem is still missing or wrong version.
try:
    installed = m.version("dynbem")
except m.PackageNotFoundError:
    installed = None

if installed != required:
    print(f"[ERROR] dynbem version check failed after install (required={required!r}, found={installed!r}); aborting sync.", file=sys.stderr)
    raise SystemExit(2)
PY'
    fi
    echo "[INFO] Code sync complete."
}

_retrieve_logs() {
    local _c="$1"
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

    # Remove any leftover per-test containers from a previously aborted run.
    _cleanup_orphan_containers

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

    echo ""
    echo "=== STACK TEST RUN START run=$_RUN_ID files=$_N_FILES workers=$_N_WORKERS date=$(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
    echo ""

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

    local j _c _f _wlog _label _short
    for j in $(seq 0 $((_N_FILES-1))); do
        while [ "${#_ACTIVE_PIDS[@]}" -ge "$_N_WORKERS" ]; do
            _reap_finished
            [ "${#_ACTIVE_PIDS[@]}" -ge "$_N_WORKERS" ] && sleep 0.5
        done

        _label="$(basename "${_ALL_FILES[$j]}" .py)"
        _short="$(echo "${_label#test_}" | tr -cs '[:alnum:]' '-' | tr '[:upper:]' '[:lower:]' | sed 's/^-*//; s/-*$//')"
        _short="${_short:0:20}"
        [ -z "$_short" ] && _short="t${j}"

        _c="rawes-parallel-${_RUN_ID}-${_short}-${j}"
        _CONTAINERS+=("$_c")
        _f="/rawes/simulation/${_ALL_FILES[$j]#${SIM_DIR}/}"
        _wlog="/tmp/rawes-parallel-${_RUN_ID}-t${j}.log"
        _WORKER_LOGS+=("$_wlog")

        _log "[t${j}] starting: $_label ($_c)"
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
            | awk -v lbl="[${_label}]" '{print strftime("%H:%M:%S") " " lbl " " $0; fflush()}'
            _test_rc=${PIPESTATUS[0]}
            rm -rf "$SIM_DIR/logs/${_label}"
            _retrieve_logs "$_c"
            mkdir -p "$SIM_DIR/logs/${_label}"
            cp -f "$_wlog" "$SIM_DIR/logs/${_label}/worker.log" 2>/dev/null || true
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
    echo "=== SUMMARY ==="
    declare -a _FAILED_LABELS=()
    declare -a _FAILED_WLOGS=()
    local _summary _status _n_pass=0 _n_fail=0
    for j in $(seq 0 $((_N_FILES-1))); do
        _wlog="${_WORKER_LOGS[$j]}"
        _label="$(basename "${_ALL_FILES[$j]}" .py)"
        _summary=$(grep -E "^=+ .* in [0-9]" "$_wlog" 2>/dev/null | tail -1 || echo "(no output)")
        if [ -z "$_summary" ] || [ "$_summary" = "(no output)" ] || echo "$_summary" | grep -qiE "failed|error"; then
            _status="FAIL"
            _FAILED_LABELS+=("$_label")
            _FAILED_WLOGS+=("$_wlog")
            (( _n_fail++ )) || true
        else
            _status="PASS"
            (( _n_pass++ )) || true
        fi
        printf "%-6s | %-42s | %s\n" "$_status" "$_label" "$_summary"
    done
    echo "=== END SUMMARY ==="

    if [ "${#_FAILED_LABELS[@]}" -gt 0 ]; then
        echo ""
        echo "=== FAILURES ==="
        local _fi _fl _fw
        for _fi in "${!_FAILED_LABELS[@]}"; do
            _fl="${_FAILED_LABELS[$_fi]}"
            _fw="${_FAILED_WLOGS[$_fi]}"
            echo ""
            echo "### FAIL: $_fl ###"
            awk '
                /^=+[ ]+(FAILURES|ERRORS)[ ]=+/ { in_s=1; print; next }
                /^=+[ ]+short test summary/ { in_s=0 }
                in_s { print }
            ' "$_fw" 2>/dev/null | head -120 || true
            grep -E "^(FAILED|ERROR) " "$_fw" 2>/dev/null || true
            echo "### END FAIL: $_fl ###"
        done
        echo ""
        echo "=== END FAILURES ==="
    fi

    local _WIN_LOGS
    _WIN_LOGS=$(cygpath -w "$SIM_DIR/logs" 2>/dev/null || echo "$SIM_DIR/logs")
    echo ""
    echo "=== RESULT: $_n_pass passed, $_n_fail failed out of $((_n_pass+_n_fail)) ==="
    _log "[LOGS] ${_WIN_LOGS}"
    return $_RC
}

# ---------------------------------------------------------------------------
# Top-level dispatch -- SITL Docker stack tests only.
# ---------------------------------------------------------------------------

case "${1:-}" in
    -h|--help)
        sed -n '3,19p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
        exit 0
        ;;
    stack)
        shift
        _run_stack "$@"
        ;;
    *)
        # No subcommand needed: all args (e.g. -n 8, -k test_foo) are stack args.
        _run_stack "$@"
        ;;
esac
