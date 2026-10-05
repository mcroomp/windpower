# syntax=docker/dockerfile:1.6

ARG ARDUPILOT_RUNTIME_IMAGE=rawes-sim-ardupilot-base:Copter-4.7.1-v1

FROM ubuntu:24.04 AS base

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
#   docker build -f simulation/Dockerfile . -t rawes-sim --target runtime
#   docker build -f simulation/Dockerfile . -t rawes-sim-ardupilot-base:Copter-4.7.1-v1 --target runtime-ardupilot-base
#   docker build -f simulation/Dockerfile . -t rawes-sim --target runtime-ardupilot
#
# Stack tests spin up their own ephemeral containers:
#   bash test.sh [-n N] [pytest args]
# =============================================================================

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC
ENV PIP_INDEX_URL=https://packagefeedproxy.microsoft.io/pypi/simple/

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

COPY simulation/requirements.txt simulation/requirements-docker.txt /tmp/
RUN python3 -m venv "$VENV_DIR" \
    && "$VENV_DIR/bin/pip" install --index-url "$PIP_INDEX_URL" --upgrade pip --quiet \
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
# Pinned to the stable Copter-4.7.1 release tag (not the moving ArduPilot-4.7
# branch head) so image rebuilds are reproducible. Required for the
# vehicle:set_target_angle_and_rate_and_throttle scripting binding used by
# rawes.lua, which does not exist in 4.6.x.
# To move to a newer stable release: update ARDUPILOT_TAG below and
# EXPECTED_ARDUPILOT_VERSION in tests/sitl/stack_utils.py together.
ARG ARDUPILOT_TAG=Copter-4.7.1
ARG ARDUPILOT_COMMIT=dbe792162d06cab66c3475fd5556bf7a120f119e
RUN git clone --depth 1 --recurse-submodules --shallow-submodules \
        --branch "$ARDUPILOT_TAG" \
        https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_PATH" \
    && test "$(git -C "$ARDUPILOT_PATH" rev-parse HEAD)" = "$ARDUPILOT_COMMIT"

WORKDIR "$ARDUPILOT_PATH"
RUN SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_EXT_ENV=1 SKIP_AP_COV_ENV=1 \
        DO_AP_STM_ENV=0 Tools/environment_install/install-prereqs-ubuntu.sh -y

ENV PATH="/home/sim/venv-ardupilot/bin:${PATH}"
RUN git config --global --add safe.directory "$ARDUPILOT_PATH" \
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
# 4. Runtime image (no ArduPilot or LinkHub)
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
# 5. Reusable runtime image with ArduPilot
# =============================================================================
FROM runtime AS runtime-ardupilot-base

ARG ARDUPILOT_PATH=/ardupilot
COPY --from=ardupilot-build ${ARDUPILOT_PATH} ${ARDUPILOT_PATH}

# =============================================================================
# 6. LinkHub and final stack image
#
# LinkHub is deliberately built and copied onto a separately tagged ArduPilot
# runtime image. setup.sh creates that image once and skips its build while the
# versioned tag exists, so LinkHub source edits cannot invalidate ArduPilot.
# =============================================================================
FROM rust:1.95-bookworm AS linkhub-build

WORKDIR /build
COPY linkhub/Cargo.toml linkhub/Cargo.lock ./
COPY linkhub/src ./src
RUN --mount=type=cache,id=linkhub-cargo-registry,target=/usr/local/cargo/registry \
    --mount=type=cache,id=linkhub-target,target=/build/target \
    cargo build --locked --release \
    && cp /build/target/release/linkhub /tmp/linkhub

FROM ${ARDUPILOT_RUNTIME_IMAGE} AS runtime-ardupilot
ARG RAWES_IMAGE_INPUT_HASH
LABEL org.rawes.image-input-hash="${RAWES_IMAGE_INPUT_HASH}"
# gdb lets SITL diagnostics snapshot thread stacks and mutex owners
# (tests/sitl/thread_trace.py); containers run with SYS_PTRACE.
RUN apt-get update && apt-get install -y --no-install-recommends gdb \
    && rm -rf /var/lib/apt/lists/*
COPY --from=linkhub-build /tmp/linkhub /usr/local/bin/linkhub
