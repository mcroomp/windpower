#!/bin/bash
# =============================================================================
# run_sim.sh — RAWES end-to-end simulation launcher
#
# Starts: MBDyn (physics), mediator.py (bridge), and prints SITL + MP commands.
#
# Prerequisites:
#   - MBDyn installed and on PATH (see install.sh)
#   - Python 3.10+ with numpy, matplotlib, scipy
#   - ArduPilot SITL built (see "SITL command" printed below)
#   - WSL2 / Linux environment
#
# Usage:
#   ./run_sim.sh [ardupilot_path]
#
# Arguments:
#   ardupilot_path   Path to ArduPilot repo root (default: ~/ardupilot)
#
# Ports:
#   9002  UDP  Mediator ← SITL  (servo outputs from ArduPilot)
#   9003  UDP  Mediator → SITL  (physics state to ArduPilot)
#   5760  TCP  Mission Planner  (default ArduPilot MAVLink ground port)
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_PATH="${1:-$HOME/ardupilot}"
LOG_DIR="/tmp/rawes_logs"
MBDYN_OUT="/tmp/rawes_out"

# Colour helpers
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Colour

log_info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $*"; }

# =============================================================================
# Cleanup function — removes sockets and kills background processes
# =============================================================================
PIDS=()

cleanup() {
    log_info "Cleaning up ..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    rm -f /tmp/rawes_forces.sock /tmp/rawes_state.sock
    log_info "Sockets removed."
}

trap cleanup EXIT

# =============================================================================
# 0. Pre-flight checks
# =============================================================================
log_info "=== RAWES Simulation Launcher ==="
echo ""

# Remove stale sockets from previous runs
rm -f /tmp/rawes_forces.sock /tmp/rawes_state.sock
log_info "Cleared stale sockets."

# Check MBDyn
if ! command -v mbdyn &>/dev/null; then
    log_error "MBDyn not found on PATH. Run install.sh first."
    exit 1
fi
MBDYN_VER=$(mbdyn --version 2>&1 | head -1 || echo "unknown")
log_info "MBDyn: $MBDYN_VER"

# Activate venv if present, otherwise check system python
VENV_DIR="$SCRIPT_DIR/.venv"
if [ -f "$VENV_DIR/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"
    log_info "Using venv: $VIRTUAL_ENV"
else
    log_warn "No venv found at $VENV_DIR — run install.sh first."
fi

# Check Python + numpy
if ! python3 -c "import numpy" 2>/dev/null; then
    log_error "Python3 numpy not found."
    log_error "Run:  bash install.sh   (sets up .venv with numpy)"
    exit 1
fi
log_info "Python3 + numpy: OK"

# Create log directory
mkdir -p "$LOG_DIR"

# =============================================================================
# 1. Generate SG6042 airfoil table
# =============================================================================
log_info "Generating SG6042.c81 airfoil table ..."
PYTHON="${VENV_DIR}/bin/python3"
[ -f "$PYTHON" ] || PYTHON=python3
"$PYTHON" "$SCRIPT_DIR/airfoil_gen.py" "$SCRIPT_DIR/mbdyn/SG6042.c81"
log_info "Airfoil table written to $SCRIPT_DIR/mbdyn/SG6042.c81"

# =============================================================================
# 2. Start MBDyn
# =============================================================================
log_info "Starting MBDyn ..."
mbdyn \
    -f "$SCRIPT_DIR/mbdyn/rotor.mbd" \
    -o "$MBDYN_OUT" \
    -s \
    > "$LOG_DIR/mbdyn.log" 2>&1 &
MBDYN_PID=$!
PIDS+=($MBDYN_PID)
log_info "MBDyn PID: $MBDYN_PID  (log: $LOG_DIR/mbdyn.log)"

# =============================================================================
# 3. Wait for MBDyn to create socket files
# =============================================================================
log_info "Waiting for MBDyn sockets ..."
SOCKET_TIMEOUT=10
elapsed=0
while true; do
    forces_ok=0
    state_ok=0
    [ -S /tmp/rawes_forces.sock ] && forces_ok=1
    [ -S /tmp/rawes_state.sock  ] && state_ok=1

    if [ $forces_ok -eq 1 ] && [ $state_ok -eq 1 ]; then
        log_info "Both MBDyn sockets are ready."
        break
    fi

    if [ $elapsed -ge $SOCKET_TIMEOUT ]; then
        log_error "MBDyn sockets did not appear after ${SOCKET_TIMEOUT}s."
        log_error "Check MBDyn log: $LOG_DIR/mbdyn.log"
        cat "$LOG_DIR/mbdyn.log" | tail -20
        exit 1
    fi

    sleep 1
    elapsed=$((elapsed + 1))
    echo -n "  ${elapsed}s  forces_sock=${forces_ok}  state_sock=${state_ok} ... "
    echo ""
done

# Check MBDyn is still running
if ! kill -0 "$MBDYN_PID" 2>/dev/null; then
    log_error "MBDyn exited early. Check: $LOG_DIR/mbdyn.log"
    cat "$LOG_DIR/mbdyn.log" | tail -30
    exit 1
fi

# =============================================================================
# 4. Start mediator.py
# =============================================================================
log_info "Starting mediator.py ..."
# Use venv python if available
PYTHON="${VENV_DIR}/bin/python3"
[ -f "$PYTHON" ] || PYTHON=python3

"$PYTHON" "$SCRIPT_DIR/mediator.py" \
    --mbdyn-force-sock /tmp/rawes_forces.sock \
    --mbdyn-state-sock /tmp/rawes_state.sock  \
    --sitl-recv-port 9002 \
    --sitl-send-port 9003 \
    --wind-x 10.0 \
    --wind-y 0.0  \
    --wind-z 0.0  \
    --log-level INFO \
    > "$LOG_DIR/mediator.log" 2>&1 &
MEDIATOR_PID=$!
PIDS+=($MEDIATOR_PID)
log_info "Mediator PID: $MEDIATOR_PID  (log: $LOG_DIR/mediator.log)"

sleep 1

if ! kill -0 "$MEDIATOR_PID" 2>/dev/null; then
    log_error "Mediator exited early. Check: $LOG_DIR/mediator.log"
    cat "$LOG_DIR/mediator.log" | tail -20
    exit 1
fi

# =============================================================================
# 5. Print manual SITL command
# =============================================================================
echo ""
echo "=========================================================="
echo "  SITL not started automatically (needs interactive terminal)"
echo "=========================================================="
echo ""
echo "  In a SEPARATE terminal, run ArduPilot SITL:"
echo ""

if [ -d "$ARDUPILOT_PATH" ]; then
    echo "    cd $ARDUPILOT_PATH"
    echo "    ./Tools/autotest/sim_vehicle.py \\"
    echo "        --vehicle ArduCopter \\"
    echo "        --frame heli \\"
    echo "        --custom-location=51.5074,-0.1278,50,0 \\"
    echo "        --sitl-instance-args='-f json:127.0.0.1' \\"
    echo "        --no-rebuild"
else
    log_warn "ArduPilot not found at $ARDUPILOT_PATH"
    echo "    cd ~/ardupilot"
    echo "    ./Tools/autotest/sim_vehicle.py \\"
    echo "        --vehicle ArduCopter \\"
    echo "        --frame heli \\"
    echo "        --custom-location=51.5074,-0.1278,50,0 \\"
    echo "        --sitl-instance-args='-f json:127.0.0.1' \\"
    echo "        --no-rebuild"
fi

echo ""
echo "  Or start ArduPilot SITL directly:"
echo ""
echo "    ./build/sitl/bin/arducopter \\"
echo "        --model json:127.0.0.1 \\"
echo "        --home 51.5074,-0.1278,50,0 \\"
echo "        --defaults Tools/autotest/default_params/copter.parm"
echo ""

# =============================================================================
# 6. Mission Planner connection instructions
# =============================================================================
echo "=========================================================="
echo "  Mission Planner connection"
echo "=========================================================="
echo ""
echo "  In Mission Planner:"
echo "    1. Top-right dropdown: 'UDP'"
echo "    2. Port: 14550  (or 14551 if 14550 is taken)"
echo "    3. Click 'Connect'"
echo ""
echo "  Or use MAVProxy:"
echo "    mavproxy.py --master=tcp:127.0.0.1:5760 --console --map"
echo ""

# =============================================================================
# 7. Monitor — wait for MBDyn to finish (or Ctrl+C)
# =============================================================================
echo "=========================================================="
echo "  Simulation running. Press Ctrl+C to stop."
echo "  Logs: $LOG_DIR/"
echo "=========================================================="
echo ""

# Tail mediator log in background
tail -f "$LOG_DIR/mediator.log" &
TAIL_PID=$!
PIDS+=($TAIL_PID)

# Wait for MBDyn process
wait "$MBDYN_PID" || true
MBDYN_EXIT=$?

kill "$TAIL_PID" 2>/dev/null || true

if [ $MBDYN_EXIT -ne 0 ]; then
    log_warn "MBDyn exited with code $MBDYN_EXIT"
    log_warn "Check: $LOG_DIR/mbdyn.log"
else
    log_info "MBDyn finished normally (final time reached)."
fi

log_info "Simulation complete."
