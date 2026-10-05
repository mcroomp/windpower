#!/usr/bin/env bash
#
# test.sh -- RAWES SITL Docker stack-test runner.
#
# Runs ONLY the ArduPilot SITL integration tests, in Docker, one container per
# test file, up to N in parallel.  The SITL stack is the only suite that needs
# Docker.  Every other suite runs with plain pytest in the Windows venv:
#
#   .venv/Scripts/python.exe -m pytest tests/unit -m "not simtest"
#   .venv/Scripts/python.exe simulation/run_tests.py tests/simtests -m simtest
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

# Docker access on this Windows workstation is WSL-only. Never use or probe
# Docker Desktop's native Windows named pipe.
if [[ -n "${MSYSTEM:-}" && -z "${WSL_DISTRO_NAME:-}" ]]; then
    if ! command -v wsl.exe >/dev/null 2>&1; then
        echo "[ERROR] WSL is required for RAWES Docker stack tests." >&2
        exit 1
    fi
    _script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    _drive="${_script_dir:1:1}"
    _wsl_dir="/mnt/${_drive,,}${_script_dir:2}"
    _profile_lockstep="$(printf '%q' "${RAWES_PROFILE_LOCKSTEP:-0}")"
    _hard_timeout="$(printf '%q' "${RAWES_STACK_TEST_TIMEOUT_S:-600}")"
    exec wsl.exe -e bash -lc \
        "cd '$_wsl_dir' && RAWES_PROFILE_LOCKSTEP=$_profile_lockstep RAWES_STACK_TEST_TIMEOUT_S=$_hard_timeout bash test.sh $(printf '%q ' "$@")"
fi

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
    tar -C "$REPO_DIR" \
        --exclude="simulation/logs" \
        --exclude="__pycache__" \
        --exclude="*/__pycache__" \
        --exclude="simulation/eeprom*.bin" \
        --exclude="tests/unit" \
        --exclude=".venv" \
        --exclude="*.egg-info" \
        -cf - pyproject.toml simulation groundstation arduloop envelope analysis viz3d scripts tests calibrate linkhub_client \
    | docker exec -i "$_c" tar -xf - -C /rawes/
    # dynbem (the Rust-backed aero core) is installed from a pinned PyPI wheel,
    # not synced from the sibling ../aero source workspace -- that source tree
    # is not needed at runtime (all rawes code imports only `dynbem`, never
    # bare `aero`/`dynbem_rs`) and its pyproject.toml previously clobbered
    # rawes's own /rawes/pyproject.toml (pytest timeout/marker config) when
    # both were synced to the same container path.
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

try:
    import dynbem
    api_ok = hasattr(dynbem, "step_omega")
except ImportError:
    api_ok = False

if version != required or not api_ok:
    try:
        # Install dynbem from PyPI (wheel-only) to avoid local Rust builds in container.
        subprocess.check_call([
            sys.executable,
            "-m",
            "pip",
            "install",
            "-q",
            "--upgrade",
            "--force-reinstall",
            "--only-binary=:all:",
            "--index-url",
            "https://packagefeedproxy.microsoft.io/pypi/simple/",
            f"dynbem=={required}",
        ])
    except subprocess.CalledProcessError as exc:
        print(f"[ERROR] dynbem=={required} PyPI wheel install failed; aborting sync.", file=sys.stderr)
        raise SystemExit(exc.returncode or 2)

# Hard guard: abort if dynbem is still missing or wrong version.
try:
    installed = m.version("dynbem")
    import dynbem
    installed_api_ok = hasattr(dynbem, "step_omega")
except m.PackageNotFoundError:
    installed = None
    installed_api_ok = False

if installed != required or not installed_api_ok:
    print(f"[ERROR] dynbem validation failed after install (required={required!r}, found={installed!r}, step_omega={installed_api_ok}); aborting sync.", file=sys.stderr)
    raise SystemExit(2)
PY'
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
            "pgrep -a -f 'arducopter|sim_vehicle|mediator\.py|linkhub serve' 2>/dev/null | grep -v 'pgrep' || true" \
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
    local _HARD_TIMEOUT_S="${RAWES_STACK_TEST_TIMEOUT_S:-600}"
    local _PROFILE_LOCKSTEP="${RAWES_PROFILE_LOCKSTEP:-0}"
    local _PASS_ARGS=()
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -n) shift; _N_WORKERS="$1" ;;
            -n[0-9]*) _N_WORKERS="${1#-n}" ;;
            --profile-lockstep) _PROFILE_LOCKSTEP=1 ;;
            *) _PASS_ARGS+=("$1") ;;
        esac
        shift
    done

    if ! [[ "$_HARD_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]]; then
        echo "[ERROR] RAWES_STACK_TEST_TIMEOUT_S must be a positive integer" >&2
        return 2
    fi

    # Each worker runs ArduPilot, LinkHub, and a 400 Hz lockstep mediator.
    # More than two concurrent stacks on the supported Windows workstation
    # causes UDP loss rather than useful throughput.  Keep -n as the requested
    # upper bound, but cap actual lockstep concurrency at the measured-safe
    # value.  The LinkHub rate benchmark runs exclusively below.
    local _EFFECTIVE_WORKERS="$_N_WORKERS"
    if [ "$_EFFECTIVE_WORKERS" -gt 2 ]; then
        _EFFECTIVE_WORKERS=2
        _log "[INFO] Requested $_N_WORKERS workers; limiting lockstep concurrency to 2"
    fi

    # A cached Docker build is intentionally run before collection. Docker
    # reuses the expensive ArduPilot stage, but still notices changed Python
    # requirements, runtime layers, or LinkHub sources. The probe gives a short,
    # actionable failure before parallel workers are launched.
    bash "$REPO_DIR/setup.sh" build
    docker run --rm --entrypoint /bin/bash "$IMAGE" -lc \
        '/rawes/.venv/bin/python -c "import sys; assert sys.version_info >= (3, 12), sys.version" \
         && test -x /ardupilot/build/sitl/bin/arducopter-heli \
         && command -v linkhub >/dev/null'

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

    mapfile -t _ALL_FILES < <(find "$REPO_DIR/tests/sitl" -name "test_*.py" | sort)

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
    echo "=== STACK TEST RUN START run=$_RUN_ID files=$_N_FILES workers=$_EFFECTIVE_WORKERS requested_workers=$_N_WORKERS hard_timeout_s=$_HARD_TIMEOUT_S date=$(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
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
        _label="$(basename "${_ALL_FILES[$j]}" .py)"

        # Message-rate assertions measure LinkHub itself, so do not run that
        # benchmark while another CPU-intensive lockstep stack is active.
        if [ "$_label" = "test_linkhub_stress_sitl" ]; then
            while [ "${#_ACTIVE_PIDS[@]}" -gt 0 ]; do
                _reap_finished
                [ "${#_ACTIVE_PIDS[@]}" -gt 0 ] && sleep 0.5
            done
        fi

        while [ "${#_ACTIVE_PIDS[@]}" -ge "$_EFFECTIVE_WORKERS" ]; do
            _reap_finished
            [ "${#_ACTIVE_PIDS[@]}" -ge "$_EFFECTIVE_WORKERS" ] && sleep 0.5
        done

        _short="$(echo "${_label#test_}" | tr -cs '[:alnum:]' '-' | tr '[:upper:]' '[:lower:]' | sed 's/^-*//; s/-*$//')"
        _short="${_short:0:20}"
        [ -z "$_short" ] && _short="t${j}"

        _c="rawes-parallel-${_RUN_ID}-${_short}-${j}"
        _CONTAINERS+=("$_c")
        _f="/rawes/${_ALL_FILES[$j]#${REPO_DIR}/}"
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
                -e RAWES_PROFILE_LOCKSTEP="$_PROFILE_LOCKSTEP" \
                -e PYTHONPATH=/rawes:/rawes/linkhub_client/src \
                "$_c" \
                timeout --signal=TERM --kill-after=30s "${_HARD_TIMEOUT_S}s" \
                /rawes/.venv/bin/python -m pytest "$_f" -s -v \
                ${_PASS_ARGS[@]+"${_PASS_ARGS[@]}"} 2>&1 \
            | tee "$_wlog" \
            | sed -n -E '/PASSED|FAILED|XFAIL|XPASS|ERROR|Error|error|Traceback|ImportError|passed|failed|xfailed|xpassed/p' \
            | awk -v lbl="[${_label}]" '{print strftime("%H:%M:%S") " " lbl " " $0; fflush()}'
            _test_rc=${PIPESTATUS[0]}
            if [ "$_test_rc" -eq 124 ]; then
                echo "[ERROR] Hard timeout: ${_label} exceeded ${_HARD_TIMEOUT_S}s" \
                    | tee -a "$_wlog"
            fi
            rm -rf "$SIM_DIR/logs/${_label}"
            _retrieve_logs "$_c"
            mkdir -p "$SIM_DIR/logs/${_label}"
            cp -f "$_wlog" "$SIM_DIR/logs/${_label}/worker.log" 2>/dev/null || true
            docker rm -f "$_c" >/dev/null 2>&1 || true
            exit $_test_rc
        ) &
        _ACTIVE_PIDS+=($!)
        _ACTIVE_CTRS+=("$_c")

        if [ "$_label" = "test_linkhub_stress_sitl" ]; then
            while [ "${#_ACTIVE_PIDS[@]}" -gt 0 ]; do
                _reap_finished
                [ "${#_ACTIVE_PIDS[@]}" -gt 0 ] && sleep 0.5
            done
        fi
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
    local _summary _status _n_pass=0 _n_fail=0 _n_skip=0
    for j in $(seq 0 $((_N_FILES-1))); do
        _wlog="${_WORKER_LOGS[$j]}"
        _label="$(basename "${_ALL_FILES[$j]}" .py)"
        _summary=$(grep -E "^=+ .* in [0-9]" "$_wlog" 2>/dev/null | tail -1 || echo "(no output)")
        if [ -z "$_summary" ] || [ "$_summary" = "(no output)" ] || echo "$_summary" | grep -qiE "failed|error"; then
            _status="FAIL"
            _FAILED_LABELS+=("$_label")
            _FAILED_WLOGS+=("$_wlog")
            (( _n_fail++ )) || true
        elif echo "$_summary" | grep -qi "deselected" \
                && ! echo "$_summary" | grep -qiE "[0-9]+ passed"; then
            _status="SKIP"
            (( _n_skip++ )) || true
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
        local _fi _fl _fw _failure_detail
        for _fi in "${!_FAILED_LABELS[@]}"; do
            _fl="${_FAILED_LABELS[$_fi]}"
            _fw="${_FAILED_WLOGS[$_fi]}"
            echo ""
            echo "### FAIL: $_fl ###"
            _failure_detail=$(
              awk '
                /^=+[ ]+(FAILURES|ERRORS)[ ]=+/ { in_s=1; print; next }
                /^=+[ ]+short test summary/ { in_s=0 }
                in_s { print }
              ' "$_fw" 2>/dev/null
              grep -E "^(FAILED|ERROR) " "$_fw" 2>/dev/null || true
            )
            if [ -n "$_failure_detail" ]; then
                echo "$_failure_detail" | tail -60
            else
                echo "(pytest did not emit a standard failure section; worker log tail follows)"
                tail -60 "$_fw" 2>/dev/null || true
            fi
            echo "### END FAIL: $_fl ###"
        done
        echo ""
        echo "=== END FAILURES ==="
    fi

    if [ "$_n_pass" -eq 0 ] && [ "$_n_fail" -eq 0 ]; then
        echo ""
        echo "[ERROR] No tests matched the supplied pytest selection."
        _RC=1
    fi

    local _WIN_LOGS
    _WIN_LOGS=$(cygpath -w "$SIM_DIR/logs" 2>/dev/null || echo "$SIM_DIR/logs")
    echo ""
    echo "=== RESULT: $_n_pass passed, $_n_fail failed, $_n_skip deselected out of $((_n_pass+_n_fail+_n_skip)) files ==="
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
