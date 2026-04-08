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

COPY requirements.txt requirements-docker.txt /tmp/
RUN python3 -m venv "$VENV_DIR" \
    && "$VENV_DIR/bin/pip" install --upgrade pip --quiet \
    && "$VENV_DIR/bin/pip" install -r /tmp/requirements-docker.txt --quiet

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
        git clone --recurse-submodules --branch sitl-json-rpm-feedback \
            https://github.com/mcroomp/ardupilot.git "$ARDUPILOT_PATH" \
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
#     Append SCR_ENABLE=1 and RPM sensor defaults to copter-heli.parm, then
#     boot ArduPilot once to flush those defaults into eeprom.bin.
#
#     WHY the boot is needed:
#       ArduPilot's scripting.init() runs before load_defaults_file() in the
#       startup sequence.  So SCR_ENABLE=1 in a --defaults file only takes
#       effect on the SECOND boot (first boot writes it to EEPROM; second boot
#       reads it from EEPROM before scripting.init() runs).  By running
#       arducopter-heli once here during image build we flush the defaults into
#       eeprom.bin, which is then baked into the image.  Every container created
#       from this image starts with SCR_ENABLE=1 already in EEPROM so Lua
#       scripting starts from the very first test boot.
#
#     --model heli uses the built-in helicopter physics (no external JSON
#     backend needed) so this runs cleanly in a Docker build layer.
#     --speedup 100 makes the 8 s wall-clock window cover ~800 s sim-time,
#     more than enough for AP_Param to complete its initialisation loop.
# =============================================================================
RUN if [ "$INSTALL_ARDUPILOT" = "true" ]; then \
        printf '\n# RAWES: Lua scripting + RPM sensor on by default\nSCR_ENABLE 1\nRPM1_TYPE 10\nRPM1_MIN 0\n' \
            >> "$ARDUPILOT_PATH/Tools/autotest/default_params/copter-heli.parm" \
        && mkdir -p "$ARDUPILOT_PATH/scripts" \
        && cd "$ARDUPILOT_PATH" \
        && timeout 8 build/sitl/bin/arducopter-heli \
               --model heli --speedup 100 --slave 0 \
               --defaults "Tools/autotest/default_params/copter-heli.parm" \
               -I0 --home 51.5074,-0.1278,50,0 \
               2>/dev/null; true; \
    fi

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
