#!/bin/bash
# =============================================================================
# install.sh — WSL2 Ubuntu setup script for RAWES simulation
#
# Safe to run multiple times — each step is idempotent.
# All output is tee'd to simulation/install.log for failure analysis.
#
# This script NEVER runs sudo automatically.
# It prints the exact sudo commands you need to run manually.
#
# Usage:
#   bash simulation/install.sh [ardupilot_path]
#
# Log file:  simulation/install.log  (appended each run)
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="$SCRIPT_DIR/install.log"
ARDUPILOT_PATH="${1:-$HOME/ardupilot}"
# Python environment for the RAWES simulation stack only.
# Keep this local to simulation/ so it stays isolated from any other repo venvs.
VENV_DIR="$SCRIPT_DIR/.venv"

# =============================================================================
# Logging: tee all output to install.log, stripping colour codes for the file
# =============================================================================
# Colour codes for terminal output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m'

# Redirect everything through tee — colours go to terminal, plain text to log
exec > >(tee -a "$LOG_FILE") 2>&1

# Separator in log so multiple runs are distinguishable
echo ""
echo "========================================================================"
echo "install.sh run: $(date '+%Y-%m-%d %H:%M:%S')"
echo "========================================================================"

log_info()    { echo -e "${GREEN}[INFO]${NC}    $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC}    $*"; }
log_error()   { echo -e "${RED}[ERROR]${NC}   $*"; }
log_section() { echo -e "\n${BOLD}=== $* ===${NC}"; }
log_skip()    { echo -e "${GREEN}[SKIP]${NC}    $* (already done)"; }
log_sudo()    {
    echo -e "${YELLOW}[MANUAL STEP]${NC}  Please run in your terminal:"
    echo ""
    echo "    $*"
    echo ""
}

log_info "Log file: $LOG_FILE"

# =============================================================================
# 1. System packages — check and print missing ones
# =============================================================================
log_section "System packages"

REQUIRED_PKGS=(
    build-essential cmake git
    pkg-config python3 python3-pip python3-venv python3-dev
    libssl-dev wget curl
)

MISSING_PKGS=()
for pkg in "${REQUIRED_PKGS[@]}"; do
    if ! dpkg -s "$pkg" &>/dev/null 2>&1; then
        MISSING_PKGS+=("$pkg")
    fi
done

if [ ${#MISSING_PKGS[@]} -eq 0 ]; then
    log_skip "All required system packages already installed"
else
    log_warn "Missing packages: ${MISSING_PKGS[*]}"
    log_sudo "sudo apt-get update && sudo apt-get install -y ${MISSING_PKGS[*]}"

    # Hard stop only for truly critical tools
    CRITICAL_MISSING=()
    for pkg in python3 build-essential; do
        if ! command -v python3 &>/dev/null && [[ "$pkg" == python3 ]]; then
            CRITICAL_MISSING+=("$pkg")
        fi
        if ! command -v gcc &>/dev/null && [[ "$pkg" == build-essential ]]; then
            CRITICAL_MISSING+=("$pkg")
        fi
    done

    if [ ${#CRITICAL_MISSING[@]} -gt 0 ]; then
        log_error "Critical tools missing: ${CRITICAL_MISSING[*]}"
        log_error "Install system packages above, then re-run install.sh"
        log_error "See $LOG_FILE for details."
        exit 1
    fi
fi

# =============================================================================
# 2. Python virtual environment
# =============================================================================
log_section "Python virtual environment"

if [ -d "$VENV_DIR/bin" ] && [ -f "$VENV_DIR/bin/python3" ] && [ -f "$VENV_DIR/bin/pip" ]; then
    log_skip "venv exists at $VENV_DIR"
else
    if [ -d "$VENV_DIR" ]; then
        log_warn "Existing venv is broken (missing pip) — recreating ..."
        rm -rf "$VENV_DIR"
    fi
    log_info "Creating simulation-local venv at $VENV_DIR ..."
    python3 -m venv "$VENV_DIR"
    log_info "venv created."
fi

# Use explicit venv paths — never rely on activation propagating correctly
VENV_PYTHON="$VENV_DIR/bin/python3"
VENV_PIP="$VENV_DIR/bin/pip"

# Upgrade pip inside venv (no sudo, no system pip)
log_info "pip upgrade ..."
"$VENV_PIP" install --upgrade pip --quiet

# Install/upgrade packages (idempotent — pip skips if already current)
log_info "Installing simulation Python packages (numpy, matplotlib, scipy) ..."
"$VENV_PIP" install numpy matplotlib scipy --quiet

# Verify using venv python explicitly
"$VENV_PYTHON" -c "import numpy;      print(f'  numpy      {numpy.__version__}')"
"$VENV_PYTHON" -c "import scipy;      print(f'  scipy      {scipy.__version__}')"
"$VENV_PYTHON" -c "import matplotlib; print(f'  matplotlib {matplotlib.__version__}')"
log_info "Python packages: OK"

# =============================================================================
# 3. ArduPilot SITL check
# =============================================================================
log_section "ArduPilot SITL"

if [ -f "$ARDUPILOT_PATH/build/sitl/bin/arducopter" ]; then
    log_skip "ArduCopter SITL binary already built ($ARDUPILOT_PATH)"
elif [ -f "$ARDUPILOT_PATH/Tools/autotest/sim_vehicle.py" ]; then
    log_warn "ArduPilot found at $ARDUPILOT_PATH but SITL binary not yet built."
    echo "  Run:"
    echo "    cd $ARDUPILOT_PATH"
    echo "    Tools/environment_install/install-prereqs-ubuntu.sh -y"
    echo "    . ~/.profile"
    echo "    ./waf configure --board sitl && ./waf copter"
else
    log_warn "ArduPilot not found at $ARDUPILOT_PATH"
    echo "  Run:"
    echo "    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot"
    echo "    cd ~/ardupilot"
    echo "    Tools/environment_install/install-prereqs-ubuntu.sh -y"
    echo "    . ~/.profile"
    echo "    ./waf configure --board sitl && ./waf copter"
fi

# =============================================================================
# 4. Write / refresh venv activation helper
# =============================================================================
ACTIVATE_HELPER="$SCRIPT_DIR/activate_venv.sh"
cat > "$ACTIVATE_HELPER" << HELPER
#!/bin/bash
# Source this to activate the RAWES simulation venv:
#   source simulation/activate_venv.sh
_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
source "\$_DIR/.venv/bin/activate"
echo "RAWES venv active: \$VIRTUAL_ENV"
HELPER
chmod +x "$ACTIVATE_HELPER"
log_info "activate_venv.sh written."

# =============================================================================
# 5. Summary
# =============================================================================
log_section "Summary"
echo ""

_ok()   { echo -e "  ${GREEN}✓${NC}  $1"; }
_warn() { echo -e "  ${YELLOW}!${NC}  $1"; }
_fail() { echo -e "  ${RED}✗${NC}  $1"; }

# Python venv
if [ -d "$VENV_DIR" ] && [ -f "$VENV_DIR/bin/pip" ] && "$VENV_DIR/bin/python3" -c "import numpy" 2>/dev/null; then
    _ok "Python venv ($VENV_DIR)"
else
    _fail "Python venv"
fi

# ArduPilot
if [ -f "$ARDUPILOT_PATH/build/sitl/bin/arducopter" ]; then
    _ok "ArduPilot SITL"
else
    _warn "ArduPilot SITL — not yet built (simulation will wait for it)"
fi

echo ""
log_info "Log saved to: $LOG_FILE"
echo ""
echo "  Venv:  source simulation/activate_venv.sh"
echo ""
