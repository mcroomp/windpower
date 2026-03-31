FROM ubuntu:22.04

# =============================================================================
# RAWES simulation image
#
# Installs: system deps, Python venv (numpy/scipy/matplotlib/pytest/pymavlink)
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
        build-essential cmake git \
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
# 3. ArduPilot SITL (optional — large download, ~30 min build)
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
# 3b. ArduPilot post-build configuration
#     Enable Lua scripting by default so it is available for all RAWES tests
#     and any future simulation work without requiring a separate enable step.
#     These are appended to the SITL defaults file; they apply on the first
#     SITL boot (before any EEPROM is created) and can be overridden via
#     MAVLink parameters as usual.
# =============================================================================
RUN if [ "$INSTALL_ARDUPILOT" = "true" ]; then \
        printf '\n# RAWES: Lua scripting + RPM sensor on by default\nSCR_ENABLE 1\nRPM1_TYPE 10\nRPM1_MIN 0\n' \
            >> "$ARDUPILOT_PATH/Tools/autotest/default_params/copter-heli.parm" \
        && mkdir -p "$ARDUPILOT_PATH/scripts"; \
    fi

# =============================================================================
# 3c. Patch ArduPilot JSON backend to expose 'rpm' field to Lua scripting.
#     The JSON keytable in SIM_JSON.h doesn't include rpm by default, so
#     RPM:get_rpm(0) always returns 0 even when the physics backend sends rpm.
#     We patch the keytable to add the rpm scalar field → state.rpm[0], then
#     do an incremental waf rebuild (only recompiles the affected .cpp file).
#     Battery.voltage is also added for completeness (already in keytable but
#     we confirm the state field name).
# =============================================================================
COPY patch_sim_json.py /tmp/patch_sim_json.py
USER sim
RUN if [ "$INSTALL_ARDUPILOT" = "true" ]; then \
        python3 /tmp/patch_sim_json.py \
        && cd "$ARDUPILOT_PATH" \
        && unset VIRTUAL_ENV \
        && export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
        && . ~/.profile \
        && ./waf --target bin/arducopter-heli; \
    fi
USER root

# =============================================================================
# 4. Simulation files — mounted at runtime, not baked into the image
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
# 5. Default shell with venv on PATH
# =============================================================================
CMD ["/bin/bash"]
