# syntax=docker/dockerfile:1.6

FROM ubuntu:22.04 AS base

# =============================================================================
# RAWES simulation image
#
# Installs: system deps, Python venv (numpy/scipy/matplotlib/pytest/pymavlink)
# Optional: ArduPilot SITL via dedicated runtime target
#
# Build (any platform, via Git Bash):
#   bash setup.sh build                                    (with ArduPilot)
#
# Or directly:
#   docker build simulation/ -t rawes-sim --target runtime
#   docker build simulation/ -t rawes-sim --target runtime-ardupilot
#
# Stack tests spin up their own ephemeral containers:
#   bash test.sh [-n N] [pytest args]
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
        build-essential cmake git rustc cargo \
        pkg-config python3 python3-pip python3-venv python3-dev \
        libssl-dev wget curl sudo tzdata \
    && rm -rf /var/lib/apt/lists/* \
    && useradd -m -s /bin/bash sim \
    && echo "sim ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/sim \
    && echo 'Defaults env_keep += "DEBIAN_FRONTEND TZ"' >> /etc/sudoers.d/sim

# =============================================================================
# 2. Python virtual environment
# =============================================================================
FROM base AS pyenv

ENV VENV_DIR=/rawes/.venv

COPY requirements.txt requirements-docker.txt /tmp/
RUN python3 -m venv "$VENV_DIR" \
    && "$VENV_DIR/bin/pip" install --upgrade pip --quiet \
    && "$VENV_DIR/bin/pip" install -r /tmp/requirements-docker.txt --quiet

# Activate venv for all subsequent RUN commands and the final shell
ENV PATH="$VENV_DIR/bin:/usr/local/bin:$PATH"
ENV VIRTUAL_ENV="$VENV_DIR"

# =============================================================================
# 3. ArduPilot SITL build (expensive; isolated reusable stage)
# =============================================================================
FROM base AS ardupilot-build

ARG ARDUPILOT_PATH=/ardupilot

RUN mkdir -p "$ARDUPILOT_PATH" && chown sim:sim "$ARDUPILOT_PATH"

USER sim
ENV USER=sim
# Pinned to a specific commit on the ArduPilot-4.7 branch (V4.7.1) rather than
# the moving branch head, so image rebuilds are reproducible and don't drift
# out from under EXPECTED_ARDUPILOT_VERSION in tests/sitl/stack_utils.py.
# Required for the vehicle:set_target_angle_and_rate_and_throttle scripting
# binding used by rawes.lua, which does not exist in 4.6.x.
# To move to a newer ArduPilot-4.7 commit: update ARDUPILOT_COMMIT below and
# EXPECTED_ARDUPILOT_VERSION in tests/sitl/stack_utils.py together.
ARG ARDUPILOT_COMMIT=331c42a50c1f68b0065d4944e55eb688b62fe9c4
RUN git clone --branch ArduPilot-4.7 \
        https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_PATH" \
    && cd "$ARDUPILOT_PATH" \
    && git checkout "$ARDUPILOT_COMMIT" \
    && git submodule update --init --recursive \
    && export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && . ~/.profile \
    && git config --global --add safe.directory "$ARDUPILOT_PATH" \
    && ./waf configure --board sitl \
    && ./waf --target bin/arducopter-heli
USER root

# =============================================================================
# 3b. ArduPilot post-build configuration
#     Append SCR_ENABLE=1 to copter-heli.parm, then
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
# =============================================================================
RUN printf '\n# RAWES: Lua scripting on by default\nSCR_ENABLE 1\n' \
        >> "$ARDUPILOT_PATH/Tools/autotest/default_params/copter-heli.parm" \
    && mkdir -p "$ARDUPILOT_PATH/scripts" \
    && cd "$ARDUPILOT_PATH" \
    && timeout 8 build/sitl/bin/arducopter-heli \
           --model heli --slave 0 \
           --defaults "Tools/autotest/default_params/copter-heli.parm" \
           -I0 --home 51.5074,-0.1278,50,0 \
           2>/dev/null || true

# =============================================================================
# 4. Runtime image (no ArduPilot)
# =============================================================================
FROM pyenv AS runtime

# =============================================================================
# Simulation files — mounted at runtime, not baked into the image
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

# Default shell with venv on PATH
CMD ["/bin/bash"]

# =============================================================================
# 5. Runtime image with ArduPilot copied from reusable build stage
# =============================================================================
FROM runtime AS runtime-ardupilot

ARG ARDUPILOT_PATH=/ardupilot
COPY --from=ardupilot-build ${ARDUPILOT_PATH} ${ARDUPILOT_PATH}
