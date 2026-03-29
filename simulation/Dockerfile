FROM ubuntu:22.04

# =============================================================================
# RAWES simulation image
#
# Installs: system deps, Python venv (numpy/scipy/matplotlib/pytest/pymavlink), MBDyn
# Optional: ArduPilot SITL (set INSTALL_ARDUPILOT=true at build time)
#
# Build (Windows cmd):
#   simulation\build.cmd
#   simulation\build.cmd ardupilot
#   simulation\build.cmd run
#
# Build (Linux/macOS):
#   docker build simulation/ -t rawes-sim
#   docker build simulation/ -t rawes-sim --build-arg INSTALL_ARDUPILOT=true
#
# Run (Linux/macOS):
#   docker run --rm -it -v "$(pwd)/simulation":/rawes/simulation rawes-sim
# =============================================================================

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# =============================================================================
# 1. System packages + non-root user for ArduPilot install
#    ArduPilot's install-prereqs-ubuntu.sh refuses to run as root, so we
#    create a 'sim' user with passwordless sudo for that step.
#    TZ and DEBIAN_FRONTEND are kept across sudo so tzdata never prompts.
# =============================================================================
RUN ln -snf /usr/share/zoneinfo/UTC /etc/localtime && echo UTC > /etc/timezone \
    && apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git gfortran \
        automake autoconf libtool autotools-dev \
        liblapack-dev libblas-dev libsuitesparse-dev \
        pkg-config python3 python3-pip python3-venv python3-dev \
        libssl-dev wget curl sudo tzdata \
    && rm -rf /var/lib/apt/lists/* \
    && useradd -m -s /bin/bash sim \
    && echo "sim ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/sim \
    && echo 'Defaults env_keep += "DEBIAN_FRONTEND TZ"' >> /etc/sudoers.d/sim

# =============================================================================
# 2. Python virtual environment
# =============================================================================
ENV VENV_DIR=/rawes/.venv

RUN python3 -m venv "$VENV_DIR" \
    && "$VENV_DIR/bin/pip" install --upgrade pip --quiet \
    && "$VENV_DIR/bin/pip" install numpy matplotlib scipy pytest pymavlink pexpect "empy==3.3.4" pyyaml --quiet

# Activate venv for all subsequent RUN commands and the final shell
ENV PATH="$VENV_DIR/bin:/usr/local/bin:$PATH"
ENV VIRTUAL_ENV="$VENV_DIR"

# =============================================================================
# 3. MBDyn — clone → bootstrap → configure → build → install
# =============================================================================
ENV MBDYN_PREFIX=/usr/local
ENV GITLAB_URL=https://public.gitlab.polimi.it/DAER/mbdyn.git

RUN git clone --depth 1 "$GITLAB_URL" /tmp/mbdyn \
    && cd /tmp/mbdyn \
    # bootstrap — try each script in order of preference
    && (bash bootstrap.sh 2>/dev/null || bash autogen.sh 2>/dev/null || autoconf) \
    && ./configure \
        --prefix="$MBDYN_PREFIX" \
        --with-lapack \
        --enable-python=no \
        --disable-documentation \
        CXXFLAGS="-O2 -pipe" \
    && make -j"$(nproc)" \
    && make install \
    && rm -rf /tmp/mbdyn

# =============================================================================
# 4. MBDyn smoke test
# =============================================================================
RUN printf '%s\n' \
        'begin: data;'                                    \
        '    problem: initial value;'                     \
        'end: data;'                                      \
        'begin: initial value;'                           \
        '    initial time: 0.;'                           \
        '    final time:   0.001;'                        \
        '    time step:    0.001;'                        \
        '    tolerance:    1.e-6;'                        \
        '    max iterations: 5;'                          \
        'end: initial value;'                             \
        'begin: control data;'                            \
        '    structural nodes: 1;'                        \
        '    rigid bodies:     1;'                        \
        '    gravity;'                                    \
        'end: control data;'                              \
        'begin: nodes;'                                   \
        '    structural: 1, dynamic,'                     \
        '        0., 0., 0., eye,'                        \
        '        0., 0., 0., 0., 0., 0.;'                \
        'end: nodes;'                                     \
        'begin: elements;'                                \
        '    body: 1, 1, 1.0, null, eye;'                \
        '    gravity: uniform, 0., 0., -1., const, 9.81;'\
        'end: elements;'                                  \
    > /tmp/smoke.mbd \
    && mbdyn -f /tmp/smoke.mbd -o /tmp/smoke_out -s \
    && rm -f /tmp/smoke.mbd /tmp/smoke_out.* \
    && echo "MBDyn smoke test: PASSED"

# =============================================================================
# 5. ArduPilot SITL (optional — large download, ~30 min build)
#    Must run as non-root; switch to 'sim' user for this step.
# =============================================================================
ARG INSTALL_ARDUPILOT=false
ARG ARDUPILOT_PATH=/ardupilot

RUN if [ "$INSTALL_ARDUPILOT" = "true" ]; then \
        mkdir -p "$ARDUPILOT_PATH" && chown sim:sim "$ARDUPILOT_PATH"; \
    fi

USER sim
ENV USER=sim
RUN if [ "$INSTALL_ARDUPILOT" = "true" ]; then \
        git clone --recurse-submodules \
            https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_PATH" \
        && cd "$ARDUPILOT_PATH" \
        && unset VIRTUAL_ENV \
        && export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
        && Tools/environment_install/install-prereqs-ubuntu.sh -y \
        && . ~/.profile \
        && git config --global --add safe.directory "$ARDUPILOT_PATH" \
        && ./waf configure --board sitl \
        && ./waf --target bin/arducopter-heli; \
    fi
USER root

# =============================================================================
# 6. Simulation files — mounted at runtime, not baked into the image
#
#   docker run --rm -it -v "$(pwd)/simulation":/rawes/simulation rawes-sim
#
# The VOLUME declaration documents the mount point; the actual files come
# from the host so changes are reflected immediately without rebuilding.
# =============================================================================
WORKDIR /rawes/simulation
VOLUME ["/rawes/simulation"]

# ArduPilot SITL MAVLink ports
EXPOSE 5760/tcp
EXPOSE 5761/tcp
EXPOSE 5762/tcp

# =============================================================================
# 7. Default shell with venv + MBDyn on PATH
# =============================================================================
CMD ["/bin/bash"]
