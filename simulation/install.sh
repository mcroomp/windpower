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
# Stamps:    ~/mbdyn_build/mbdyn/.stamps/  (track MBDyn build progress)
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="$SCRIPT_DIR/install.log"
ARDUPILOT_PATH="${1:-$HOME/ardupilot}"
# Python environment for the MBDyn/SITL simulation stack only.
# Keep this local to simulation/ so it stays isolated from any other repo venvs.
VENV_DIR="$SCRIPT_DIR/.venv"
MBDYN_BUILD_DIR="$HOME/mbdyn_build"
MBDYN_PREFIX="$HOME/.local"
MBDYN_SRC="$MBDYN_BUILD_DIR/mbdyn"
STAMPS_DIR="$MBDYN_SRC/.stamps"
GITLAB_URL="https://public.gitlab.polimi.it/DAER/mbdyn.git"

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

# Stamp helpers — track build progress so re-runs skip completed steps
stamp_exists() { [ -f "$STAMPS_DIR/$1" ]; }
stamp_set()    { mkdir -p "$STAMPS_DIR"; touch "$STAMPS_DIR/$1"; }
stamp_clear()  { rm -f "$STAMPS_DIR/$1"; }

# =============================================================================
# 1. System packages — check and print missing ones
# =============================================================================
log_section "System packages"

REQUIRED_PKGS=(
    build-essential cmake git gfortran
    automake autoconf libtool autotools-dev
    liblapack-dev libblas-dev libsuitesparse-dev
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

    log_warn "Non-critical packages missing — proceeding; MBDyn build may fail."
    log_warn "Install the packages above if MBDyn configure fails."
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
# 3. MBDyn — clone → bootstrap → configure → build → install
#    Each sub-step is stamp-gated: skip if the stamp file exists AND the
#    expected output also exists (binary / Makefile / etc.).
# =============================================================================
log_section "MBDyn"

# -- Already installed? -------------------------------------------------------
MBDYN_BIN=""
if command -v mbdyn &>/dev/null; then
    MBDYN_BIN=$(command -v mbdyn)
elif [ -f "$MBDYN_PREFIX/bin/mbdyn" ]; then
    MBDYN_BIN="$MBDYN_PREFIX/bin/mbdyn"
    export PATH="$MBDYN_PREFIX/bin:$PATH"
fi

if [ -n "$MBDYN_BIN" ]; then
    log_skip "MBDyn already installed: $($MBDYN_BIN --version 2>&1 | head -1)"
    MBDYN_ALREADY_INSTALLED=1
else
    MBDYN_ALREADY_INSTALLED=0
fi

if [ "$MBDYN_ALREADY_INSTALLED" -eq 0 ]; then

    # Check lapack before attempting configure
    if ! (pkg-config --exists lapack 2>/dev/null || ldconfig -p 2>/dev/null | grep -q liblapack); then
        log_warn "liblapack-dev may not be installed. MBDyn configure will likely fail."
        log_sudo "sudo apt-get install -y liblapack-dev libblas-dev libsuitesparse-dev"
    fi

    # ---- 3a. Clone ----------------------------------------------------------
    if stamp_exists "clone" && [ -d "$MBDYN_SRC/.git" ]; then
        log_skip "MBDyn source already cloned ($MBDYN_SRC)"
    else
        log_info "Cloning MBDyn from $GITLAB_URL ..."
        mkdir -p "$MBDYN_BUILD_DIR"
        if git clone --depth 1 "$GITLAB_URL" "$MBDYN_SRC"; then
            stamp_set "clone"
            log_info "Clone complete."
        else
            log_error "Git clone failed."
            log_error "Manual options:"
            echo "  A) Download tarball: https://www.mbdyn.org/Download.html"
            echo "     tar xzf mbdyn-*.tar.gz -C $MBDYN_BUILD_DIR"
            echo "     mv $MBDYN_BUILD_DIR/mbdyn-* $MBDYN_SRC"
            echo "  B) Ubuntu package (needs sudo):"
            log_sudo "sudo apt-get install mbdyn"
            log_error "See $LOG_FILE for full git output."
            exit 1
        fi
    fi

    cd "$MBDYN_SRC"

    # ---- 3b. Bootstrap -------------------------------------------------------
    if stamp_exists "bootstrap" && [ -f "$MBDYN_SRC/configure" ]; then
        log_skip "MBDyn bootstrap already done"
    else
        log_info "Running bootstrap ..."
        BOOTSTRAP_OK=0
        if [ -f bootstrap.sh ]; then
            bash bootstrap.sh && BOOTSTRAP_OK=1
        elif [ -f autogen.sh ]; then
            bash autogen.sh && BOOTSTRAP_OK=1
        else
            autoconf && BOOTSTRAP_OK=1
        fi

        if [ "$BOOTSTRAP_OK" -ne 1 ] || [ ! -f "$MBDYN_SRC/configure" ]; then
            log_error "MBDyn bootstrap failed — autotools not available."
            log_error "Common cause: automake/autoconf/libtool not installed."
            log_sudo "sudo apt-get install -y automake autoconf libtool autotools-dev"
            log_error "Install the above, then re-run install.sh"
            log_error "Full output in $LOG_FILE"
            exit 1
        fi
        stamp_set "bootstrap"
        log_info "Bootstrap complete."
    fi

    # ---- 3c. Configure -------------------------------------------------------
    # Re-run configure if: no stamp, or Makefile missing, or prefix changed
    CONFIGURED_PREFIX=""
    [ -f "$MBDYN_SRC/Makefile" ] && \
        CONFIGURED_PREFIX=$(grep -m1 '^prefix' "$MBDYN_SRC/Makefile" 2>/dev/null | awk '{print $3}' || true)

    if stamp_exists "configure" && [ -f "$MBDYN_SRC/Makefile" ] && [ "$CONFIGURED_PREFIX" = "$MBDYN_PREFIX" ]; then
        log_skip "MBDyn configure already done (prefix=$MBDYN_PREFIX)"
    else
        [ "$CONFIGURED_PREFIX" != "$MBDYN_PREFIX" ] && [ -n "$CONFIGURED_PREFIX" ] && \
            log_warn "Prefix changed ($CONFIGURED_PREFIX → $MBDYN_PREFIX). Re-configuring."
        log_info "Configuring MBDyn (prefix=$MBDYN_PREFIX) ..."
        stamp_clear "build"    # force rebuild after reconfigure
        stamp_clear "install"
        if ./configure \
            --prefix="$MBDYN_PREFIX" \
            --with-lapack \
            --enable-python=no \
            --disable-documentation \
            CXXFLAGS="-O2 -pipe"; then
            stamp_set "configure"
            log_info "Configure complete."
        else
            log_error "MBDyn configure failed. Common causes:"
            echo "  - Missing liblapack-dev / libblas-dev"
            echo "  - Missing libsuitesparse-dev"
            log_error "Full configure output is in $LOG_FILE"
            exit 1
        fi
    fi

    # ---- 3d. Build -----------------------------------------------------------
    if stamp_exists "build" && [ -f "$MBDYN_SRC/mbdyn/mbdyn" ]; then
        log_skip "MBDyn already built"
    else
        log_info "Building MBDyn with $(nproc) jobs (this takes 5–15 minutes) ..."
        stamp_clear "install"   # force reinstall after rebuild
        if make -j"$(nproc)"; then
            stamp_set "build"
            log_info "Build complete."
        else
            log_error "MBDyn build failed."
            log_error "Full build output is in $LOG_FILE"
            log_warn "Tip: try 'make -j1' to get a clearer error:"
            echo "    cd $MBDYN_SRC && make -j1 2>&1 | tail -30"
            exit 1
        fi
    fi

    # ---- 3e. Install ---------------------------------------------------------
    if stamp_exists "install" && [ -f "$MBDYN_PREFIX/bin/mbdyn" ]; then
        log_skip "MBDyn already installed to $MBDYN_PREFIX"
    else
        log_info "Installing MBDyn to $MBDYN_PREFIX ..."
        if make install; then
            stamp_set "install"
            export PATH="$MBDYN_PREFIX/bin:$PATH"
            log_info "Install complete: $(mbdyn --version 2>&1 | head -1)"
        else
            log_error "MBDyn install failed. See $LOG_FILE"
            exit 1
        fi
    fi

    log_warn "Add MBDyn to your PATH permanently — add to ~/.bashrc:"
    echo "    export PATH=\"$MBDYN_PREFIX/bin:\$PATH\""

fi  # end MBDYN_ALREADY_INSTALLED=0

# =============================================================================
# 4. MBDyn smoke test — run a trivial model to confirm it works
# =============================================================================
log_section "MBDyn smoke test"

TEST_MBD=$(mktemp /tmp/mbdyn_smoke_XXXX.mbd)
TEST_OUT=$(mktemp /tmp/mbdyn_smoke_XXXX)
cat > "$TEST_MBD" << 'EOF'
begin: data;
    problem: initial value;
end: data;
begin: initial value;
    initial time: 0.;
    final time:   0.001;
    time step:    0.001;
    tolerance:    1.e-6;
    max iterations: 5;
end: initial value;
begin: control data;
    structural nodes: 1;
    rigid bodies:     1;
    gravity;
end: control data;
begin: nodes;
    structural: 1, dynamic,
        0., 0., 0., eye,
        0., 0., 0., 0., 0., 0.;
end: nodes;
begin: elements;
    body: 1, 1, 1.0, null, eye;
    gravity: uniform, 0., 0., -1., const, 9.81;
end: elements;
EOF

SMOKE_OK=0
if mbdyn -f "$TEST_MBD" -o "$TEST_OUT" -s 2>&1; then
    log_info "MBDyn smoke test: PASSED"
    SMOKE_OK=1
else
    log_warn "MBDyn smoke test: FAILED (see $LOG_FILE for output)"
    log_warn "The simulation will likely not work until this is resolved."
fi
rm -f "$TEST_MBD" "${TEST_OUT}".*

# =============================================================================
# 5. ArduPilot SITL check
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
# 6. Write / refresh venv activation helper
# =============================================================================
ACTIVATE_HELPER="$SCRIPT_DIR/activate_venv.sh"
cat > "$ACTIVATE_HELPER" << HELPER
#!/bin/bash
# Source this to activate the RAWES simulation venv:
#   source simulation/activate_venv.sh
_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
source "\$_DIR/.venv/bin/activate"
export PATH="$MBDYN_PREFIX/bin:\$PATH"
echo "RAWES venv active: \$VIRTUAL_ENV"
HELPER
chmod +x "$ACTIVATE_HELPER"
log_info "activate_venv.sh written."

# =============================================================================
# 7. Summary
# =============================================================================
log_section "Summary"
echo ""

_ok()   { echo -e "  ${GREEN}✓${NC}  $1"; }
_warn() { echo -e "  ${YELLOW}!${NC}  $1"; }
_fail() { echo -e "  ${RED}✗${NC}  $1"; }

# MBDyn
if command -v mbdyn &>/dev/null || [ -f "$MBDYN_PREFIX/bin/mbdyn" ]; then
    if [ "$SMOKE_OK" -eq 1 ]; then
        _ok "MBDyn — installed and smoke test passed"
    else
        _warn "MBDyn — installed but smoke test failed (check $LOG_FILE)"
    fi
else
    _fail "MBDyn — not installed"
fi

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
echo "  Next:  ./simulation/run_sim.sh"
echo "  Venv:  source simulation/activate_venv.sh"
echo ""
