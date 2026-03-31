"""
torque/conftest.py — pytest fixtures for counter-torque motor stack tests.

Provides the ``torque_armed`` fixture, which:
  1. Launches ArduPilot SITL (same helicopter frame used by all stack tests)
  2. Launches mediator_torque.py (stationary hub + bearing drag + GB4008 model)
  3. Connects GCS, sets minimal parameters, arms, switches to ACRO mode
  4. Yields a TorqueStackContext for the test to observe

Teardown terminates both processes and copies logs to simulation/logs/.

Environment variables (shared with main stack tests via stack_utils)
---------------------------------------------------------------------
  RAWES_RUN_STACK_INTEGRATION=1   required to enable (skips otherwise)
  RAWES_ARDUPILOT_PATH or RAWES_SIM_VEHICLE   locate sim_vehicle.py

Running
-------
  wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack -v -k yaw_regulation'
"""
from __future__ import annotations

import dataclasses
import logging
import os
import sys
import time
from pathlib import Path

import pytest

# ── Path setup ──────────────────────────────────────────────────────────────
_TORQUE_DIR = Path(__file__).resolve().parent          # simulation/torque/
_SIM_DIR    = _TORQUE_DIR.parent                       # simulation/
_REPO_ROOT  = _SIM_DIR.parent                          # repo root

sys.path.insert(0, str(_SIM_DIR))    # for stack_utils, gcs, etc.
sys.path.insert(0, str(_TORQUE_DIR)) # for model

# Shared SITL/process utilities — single source of truth
from stack_utils import (
    STACK_ENV_FLAG,
    ARDUPILOT_ENV,
    SIM_VEHICLE_ENV,
    SITL_GCS_PORT,
    GCS_ADDRESS,
    _resolve_sim_vehicle,
    _launch_sitl,
    _terminate_process,
    _kill_by_port,
    _configure_logging,
    copy_logs_to_dir,
    check_ports_free,
)

import subprocess


# ---------------------------------------------------------------------------
# StackContext
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class TorqueStackContext:
    """Everything a torque stack test needs after setup."""
    gcs:           object          # RawesGCS (imported lazily inside fixture)
    mediator_proc: subprocess.Popen
    sitl_proc:     subprocess.Popen
    mediator_log:  Path
    sitl_log:      Path
    gcs_log:       Path
    omega_axle:    float           # [rad/s] axle spin rate used for this run
    log:           logging.Logger


# ---------------------------------------------------------------------------
# Mediator launch (torque-specific — different script from main mediator.py)
# ---------------------------------------------------------------------------

#: Startup-hold duration passed to mediator_torque.py.
#: During this period the mediator sends a slow 5°/s yaw spin so the EKF can
#: align gyro bias and compass yaw before real hub dynamics begin.
#: Must match (or be less than) the EKF_ALIGN_TIMEOUT in the fixture below.
STARTUP_HOLD_S: float = 10.0  # 5°/s spin for EKF alignment; trim throttle handles equilibrium


def _launch_mediator_torque(
    log_path: Path,
    omega_axle: float,
    profile: str = "constant",
    tail_channel: int = 3,
    lua_mode: bool = False,
) -> subprocess.Popen:
    """Launch mediator_torque.py as a subprocess."""
    script = str(_TORQUE_DIR / "mediator_torque.py")
    cmd = [
        sys.executable, script,
        "--omega-axle",    str(omega_axle),
        "--startup-hold",  str(STARTUP_HOLD_S),
        "--profile",       profile,
        "--tail-channel",  str(tail_channel),
        "--log-level",     "INFO",
    ]
    if lua_mode:
        cmd.append("--lua-mode")
    return subprocess.Popen(
        cmd,
        cwd=str(_REPO_ROOT),
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


# ---------------------------------------------------------------------------
# Parameter helper
# ---------------------------------------------------------------------------

def _set_param(gcs, name: str, value: float, log: logging.Logger) -> bool:
    ok = gcs.set_param(name, value, timeout=5.0)
    log.info("  %-28s = %-10g  ACK=%s", name, value, ok)
    return ok


# ---------------------------------------------------------------------------
# Fixture
# ---------------------------------------------------------------------------

@pytest.fixture
def torque_armed(tmp_path):
    """
    Full torque-test stack fixture.

    Launches SITL + mediator_torque.py, connects GCS, sets minimal helicopter
    parameters for a yaw-only test, arms in ACRO mode, and yields a
    TorqueStackContext.

    Teardown terminates all processes and copies logs to simulation/logs/.
    """
    import math

    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run counter-torque stack tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} to locate sim_vehicle.py")

    pytest.importorskip("pymavlink")
    from gcs import RawesGCS, ACRO  # deferred: pymavlink must be importable first

    check_ports_free()

    import model as _m
    omega_axle = _m.OMEGA_AXLE_NOMINAL

    # ── Paths ──────────────────────────────────────────────────────────────
    mediator_log = tmp_path / "mediator_torque.log"
    sitl_log     = tmp_path / "sitl.log"
    gcs_log      = tmp_path / "gcs.log"

    _configure_logging(gcs_log)
    log = logging.getLogger("torque_armed")
    log.info(
        "torque_armed: launching  ω_axle=%.1f rad/s (%.0f RPM)",
        omega_axle, omega_axle * 60.0 / (2.0 * math.pi),
    )

    # ── Launch processes ───────────────────────────────────────────────────
    mediator_proc = _launch_mediator_torque(mediator_log, omega_axle)
    sitl_proc     = _launch_sitl(sim_vehicle, sitl_log)

    def _assert_alive():
        for name, proc, lp in [
            ("mediator_torque", mediator_proc, mediator_log),
            ("SITL",            sitl_proc,     sitl_log),
        ]:
            if proc.poll() is not None:
                txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{txt[-3000:]}")

    gcs = RawesGCS(address=GCS_ADDRESS)
    ctx = TorqueStackContext(
        gcs=gcs,
        mediator_proc=mediator_proc,
        sitl_proc=sitl_proc,
        mediator_log=mediator_log,
        sitl_log=sitl_log,
        gcs_log=gcs_log,
        omega_axle=omega_axle,
        log=log,
    )

    try:
        # ── Connect ────────────────────────────────────────────────────────
        log.info("Connecting GCS …")
        gcs.connect(timeout=30.0)
        gcs.start_heartbeat(rate_hz=1.0)
        _assert_alive()
        log.info("GCS connected")

        # Request ATTITUDE stream (needed for EKF wait and yaw observation)
        from pymavlink import mavutil as _mavu
        gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_EXTRA1, 10)   # ATTITUDE @ 10 Hz
        gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_EXTRA3, 2)    # EKF_STATUS_REPORT @ 2 Hz

        # ── Wait for param subsystem ───────────────────────────────────────
        log.info("Waiting for param subsystem …")
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            _assert_alive()
            gcs._mav.mav.param_request_read_send(
                gcs._target_system, gcs._target_component, b"SYSID_THISMAV", -1,
            )
            msg = gcs._mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is not None:
                log.info("Param subsystem ready (SYSID_THISMAV=%g)", msg.param_value)
                break
        else:
            pytest.fail("Param subsystem never responded within 20 s")

        # ── Set parameters ─────────────────────────────────────────────────
        log.info("Setting parameters …")
        params_to_set = [
            # Safety / arming
            ("ARMING_SKIPCHK",    0xFFFF),  # skip all pre-arm checks
            ("FS_EKF_ACTION",     0),        # no EKF failsafe
            ("FS_THR_ENABLE",     0),        # no throttle failsafe
            # EKF — compass yaw (suitable for stationary test)
            ("COMPASS_USE",       1),
            ("COMPASS_ENABLE",    1),
            ("EK3_SRC1_YAW",      1),        # 1 = compass
            ("EK3_MAG_CAL",       0),        # 0 = Never (use raw compass)
            ("EK3_GPS_CHECK",     0),        # no GPS health check
            # RSC — CH8 passthrough: runup_complete is set immediately when CH8 is HIGH.
            # H_RSC_MODE=0 is INVALID and causes auto-disarm ("H_RSC_MODE invalid").
            # H_RSC_MODE=1 (CH8 passthrough) gives instant runup_complete when CH8=2000.
            ("H_RSC_MODE",        1),        # 1 = CH8 passthrough (instant runup)
            ("H_RSC_RUNUP_TIME",  1),        # minimum (1 s)
            # Tail type — servo (Type 0): output centered at 1500 µs (neutral).
            # DDFP types (2/3) start from 0% idle, never reaching the 1500 µs
            # neutral needed by the biased throttle mapping in the mediator.
            # With Type 0 (servo): neutral sticks → 1500 µs → trim throttle (75%)
            # → exact equilibrium torque, PID only needs small corrections.
            ("H_TAIL_TYPE",       0),
            # Zero collective-to-yaw feedforward (no collective in this test)
            ("H_COL2YAW",         0.0),
            # Yaw PID — tuned for GB4008 + hub system.
            # The mediator's biased throttle mapping puts neutral PWM at the
            # equilibrium throttle (~75%), so the I-term is NOT needed for
            # steady-state (it would wind up during the startup spin and slam
            # the motor when dynamics start).  Pure-P with a tiny gain:
            # P=0.001 gives <1% throttle change per 1 deg/s yaw error.
            # The motor's back-EMF provides inherent speed regulation that
            # keeps the hub stable — the trim does the heavy lifting.
            ("ATC_RAT_YAW_P",     0.001),
            ("ATC_RAT_YAW_I",     0.0),
            ("ATC_RAT_YAW_D",     0.0),
            ("ATC_RAT_YAW_IMAX",  0.0),
            # Disable roll/pitch IMAX to prevent swashplate wind-up on neutral sticks
            ("ATC_RAT_RLL_IMAX",  0.0),
            ("ATC_RAT_PIT_IMAX",  0.0),
        ]
        for pname, pvalue in params_to_set:
            _set_param(gcs, pname, pvalue, log)
            _assert_alive()

        # ── Motor interlock HIGH (CH8=2000) + EKF alignment wait ─────────
        # CH8=2000 must be kept alive continuously — ArduPilot expires RC
        # override after ~1 s.  We re-send it every 0.5 s throughout the
        # EKF wait so the interlock stays HIGH when we arm.
        #
        # The mediator sends a slow 5°/s yaw spin for STARTUP_HOLD_S seconds
        # (mirrors the kinematic startup in the full stack tests).  With
        # compass yaw (EK3_SRC1_YAW=1), tilt + yaw alignment takes 4–8 s.
        # We wait up to 45 s (matching main conftest) to be safe.
        log.info("Sending motor interlock HIGH (CH8=2000) and waiting for EKF (up to 45 s) …")
        gcs.send_rc_override({8: 2000})

        ekf_ok      = False
        yaw_ok      = False   # saw "yaw alignment complete" STATUSTEXT
        deadline    = time.monotonic() + 45.0
        t_last_rc   = time.monotonic()
        t_ekf_start = time.monotonic()
        _MIN_WAIT   = 3.0   # ignore stale messages in the recv buffer

        while time.monotonic() < deadline:
            _assert_alive()
            # Keep CH8 alive
            if time.monotonic() - t_last_rc >= 0.5:
                gcs.send_rc_override({8: 2000})
                t_last_rc = time.monotonic()

            msg = gcs._mav.recv_match(
                type=["ATTITUDE", "STATUSTEXT"],
                blocking=True, timeout=0.5,
            )
            if msg is None:
                continue

            now = time.monotonic()

            if msg.get_type() == "STATUSTEXT":
                text = msg.text.rstrip("\x00").strip()
                log.info("SITL: %s", text)
                # Re-request the ATTITUDE stream whenever EKF activates,
                # since ArduPilot resets data streams on EKF initialisation.
                if "EKF3 active" in text or "EKF3 IMU" in text:
                    from pymavlink import mavutil as _mavu2
                    gcs.request_stream(_mavu2.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
                # Accept yaw alignment via STATUSTEXT (flat state may not produce
                # ATTITUDE until EKF is fully ready, so use this as a fallback).
                if "yaw alignment complete" in text.lower():
                    yaw_ok = True
                    log.info("EKF yaw alignment confirmed via STATUSTEXT")
                    if now - t_ekf_start >= _MIN_WAIT:
                        ekf_ok = True
                        break

            elif msg.get_type() == "ATTITUDE":
                if (all(math.isfinite(v) for v in (msg.roll, msg.pitch, msg.yaw))
                        and now - t_ekf_start >= _MIN_WAIT):
                    log.info(
                        "EKF attitude ready  rpy=(%.1f°, %.1f°, %.1f°)",
                        math.degrees(msg.roll),
                        math.degrees(msg.pitch),
                        math.degrees(msg.yaw),
                    )
                    ekf_ok = True
                    break

        if not ekf_ok:
            log.warning(
                "EKF alignment timed out (yaw_ok=%s) — proceeding anyway (SKIPCHK set)",
                yaw_ok,
            )

        # ── Arm (force=True, keep CH8 alive) ──────────────────────────────
        # gcs.arm() returns None on success and raises TimeoutError on failure.
        log.info("Arming (force=True) …")
        gcs.arm(timeout=15.0, force=True, rc_override={8: 2000})
        log.info("Armed")

        # ── Switch to ACRO ─────────────────────────────────────────────────
        # set_mode() returns None on success and raises TimeoutError on failure.
        log.info("Switching to ACRO …")
        gcs.set_mode(ACRO, timeout=10.0, rc_override={8: 2000})
        log.info("ACRO mode active — yielding context to test")

        yield ctx

    finally:
        log.info("Teardown: terminating processes …")
        try:
            gcs.close()
        except Exception:
            pass
        _terminate_process(mediator_proc)
        _terminate_process(sitl_proc)
        _kill_by_port(SITL_GCS_PORT)

        copy_logs_to_dir(_SIM_DIR / "logs", {
            "mediator_torque_last_run.log": mediator_log,
            "sitl_torque_last_run.log":     sitl_log,
            "gcs_torque_last_run.log":      gcs_log,
        })
        log.info("Logs copied to %s", _SIM_DIR / "logs")


@pytest.fixture
def torque_armed_profile(request, tmp_path):
    """
    Like ``torque_armed`` but accepts a profile name via ``request.param``.

    Use with pytest indirect parametrization:

        @pytest.mark.parametrize("torque_armed_profile", ["slow_vary"],
                                 indirect=True)
        def test_foo(torque_armed_profile):
            ...

    The profile name is forwarded to ``--profile`` in mediator_torque.py.
    """
    import math as _math

    profile = getattr(request, "param", "constant")

    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run counter-torque stack tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} to locate sim_vehicle.py")

    pytest.importorskip("pymavlink")
    from gcs import RawesGCS, ACRO  # noqa: F811

    check_ports_free()

    import model as _m
    omega_axle = _m.OMEGA_AXLE_NOMINAL

    mediator_log = tmp_path / "mediator_torque.log"
    sitl_log     = tmp_path / "sitl.log"
    gcs_log      = tmp_path / "gcs.log"

    _configure_logging(gcs_log)
    log = logging.getLogger(f"torque_armed[{profile}]")
    log.info("launching  profile=%s  ω_axle=%.1f rad/s (%.0f RPM)",
             profile, omega_axle, omega_axle * 60.0 / (2.0 * _math.pi))

    mediator_proc = _launch_mediator_torque(mediator_log, omega_axle, profile=profile)
    sitl_proc     = _launch_sitl(sim_vehicle, sitl_log)

    def _assert_alive():
        for name, proc, lp in [
            ("mediator_torque", mediator_proc, mediator_log),
            ("SITL",            sitl_proc,     sitl_log),
        ]:
            if proc.poll() is not None:
                txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else ""
                pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{txt[-2000:]}")

    gcs = RawesGCS(address=GCS_ADDRESS)
    ctx = TorqueStackContext(
        gcs=gcs,
        mediator_proc=mediator_proc,
        sitl_proc=sitl_proc,
        mediator_log=mediator_log,
        sitl_log=sitl_log,
        gcs_log=gcs_log,
        omega_axle=omega_axle,
        log=log,
    )

    try:
        log.info("Connecting GCS …")
        gcs.connect(timeout=30.0)
        gcs.start_heartbeat(rate_hz=1.0)
        _assert_alive()

        # Param subsystem
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            _assert_alive()
            gcs._mav.mav.param_request_read_send(
                gcs._target_system, gcs._target_component, b"SYSID_THISMAV", -1,
            )
            msg = gcs._mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is not None:
                log.info("Param subsystem ready"); break
        else:
            pytest.fail("Param subsystem never responded")

        # Parameters
        from pymavlink import mavutil as _mavu
        gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
        gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_EXTRA3, 2)
        for pname, pvalue in [
            ("ARMING_SKIPCHK",   0xFFFF), ("FS_EKF_ACTION",   0),
            ("FS_THR_ENABLE",    0),      ("COMPASS_USE",      1),
            ("COMPASS_ENABLE",   1),      ("EK3_SRC1_YAW",     1),
            ("EK3_MAG_CAL",      0),      ("EK3_GPS_CHECK",    0),
            ("H_RSC_MODE",       1),      ("H_RSC_RUNUP_TIME", 1),
            ("H_TAIL_TYPE",      0),      ("H_COL2YAW",        0.0),
            ("ATC_RAT_YAW_P",    0.001),  ("ATC_RAT_YAW_I",    0.0),
            ("ATC_RAT_YAW_D",    0.0),    ("ATC_RAT_YAW_IMAX", 0.0),
            ("ATC_RAT_RLL_IMAX", 0.0),   ("ATC_RAT_PIT_IMAX", 0.0),
        ]:
            _set_param(gcs, pname, pvalue, log)
            _assert_alive()

        # EKF alignment
        log.info("Waiting for EKF yaw alignment …")
        ekf_ok = False
        t_start = time.monotonic()
        t_rc    = time.monotonic()
        deadline = time.monotonic() + 45.0
        gcs.send_rc_override({8: 2000})
        while time.monotonic() < deadline:
            _assert_alive()
            if time.monotonic() - t_rc >= 0.5:
                gcs.send_rc_override({8: 2000})
                t_rc = time.monotonic()
            msg = gcs._mav.recv_match(type=["ATTITUDE","STATUSTEXT"],
                                       blocking=True, timeout=0.5)
            if msg is None: continue
            if msg.get_type() == "STATUSTEXT":
                text = msg.text.rstrip("\x00").strip()
                log.info("SITL: %s", text)
                if "EKF3 active" in text or "EKF3 IMU" in text:
                    gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
                if "yaw alignment complete" in text.lower():
                    if time.monotonic() - t_start >= 3.0:
                        ekf_ok = True; break
            elif msg.get_type() == "ATTITUDE":
                if (all(_math.isfinite(v) for v in (msg.roll, msg.pitch, msg.yaw))
                        and time.monotonic() - t_start >= 3.0):
                    ekf_ok = True; break
        if not ekf_ok:
            log.warning("EKF alignment timed out — proceeding (SKIPCHK set)")

        gcs.arm(timeout=15.0, force=True, rc_override={8: 2000})
        log.info("Armed")
        gcs.set_mode(ACRO, timeout=10.0, rc_override={8: 2000})
        log.info("ACRO active — profile=%s", profile)

        yield ctx

    finally:
        try: gcs.close()
        except Exception: pass
        _terminate_process(mediator_proc)
        _terminate_process(sitl_proc)
        _kill_by_port(SITL_GCS_PORT)
        copy_logs_to_dir(_SIM_DIR / "logs", {
            f"mediator_{profile}_last.log": mediator_log,
            f"sitl_{profile}_last.log":     sitl_log,
            f"gcs_{profile}_last.log":      gcs_log,
        })
        log.info("Logs copied")


# ---------------------------------------------------------------------------
# Lua feedforward fixture
# ---------------------------------------------------------------------------

def _launch_sitl_lua(sim_vehicle: Path, log_path: Path,
                     defaults_path: str) -> subprocess.Popen:
    """Launch SITL with wipe + extra defaults file for SCR_ENABLE=1 at boot.

    --wipe clears the EEPROM so the defaults file values take precedence.
    Without wipe, a previously stored SCR_ENABLE=0 in EEPROM would prevent
    the scripting engine from starting even with SCR_ENABLE=1 in the defaults.
    """
    return subprocess.Popen(
        [
            sys.executable, str(sim_vehicle),
            "--vehicle", "ArduCopter",
            "--frame",   "heli",
            "--custom-location=51.5074,-0.1278,50,0",
            "--model",   "JSON",
            "--sim-address", "127.0.0.1",
            "--no-rebuild",
            "--no-mavproxy",
            # SCR_ENABLE=1 is baked into copter-heli.parm via the Dockerfile,
            # so no --add-param-file needed. Extra param files cause SIGFPE in
            # this ArduPilot build when combined with scripting init.
        ],
        cwd=str(sim_vehicle.parent.parent.parent),
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _install_lua_scripts() -> None:
    """Copy all Lua scripts to SITL's scripts directory (/ardupilot/scripts/)."""
    import shutil as _shutil
    dst_dir = Path("/ardupilot/scripts")
    dst_dir.mkdir(exist_ok=True)
    for script in (_TORQUE_DIR / "scripts").glob("*.lua"):
        _shutil.copy2(script, dst_dir / script.name)


@pytest.fixture
def torque_armed_lua(tmp_path):
    """
    Like ``torque_armed`` but with the Lua feedforward script active.

    Key differences from ``torque_armed``:
      • Lua script rawes_yaw_trim.lua is installed to SITL's scripts directory
      • SCR_ENABLE=1, RPM1_TYPE=10 (reads motor RPM from JSON sensor packet)
      • SERVO9_FUNCTION=94 (Lua writes exclusively to Ch9 — Script 1)
      • mediator_torque.py --lua-mode --tail-channel 8 (reads Ch9, linear mapping)
      • No adaptive trim in mediator — Lua is the sole feedforward provider

    The mediator sends motor RPM in the JSON "rpm" field (axle × gear_ratio),
    matching what the AM32 ESC would report via DSHOT telemetry on hardware.
    """
    import math as _math
    import shutil as _shutil

    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run Lua feedforward stack tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} to locate sim_vehicle.py")

    pytest.importorskip("pymavlink")
    from gcs import RawesGCS, ACRO  # noqa: F811

    check_ports_free()

    # Delete EEPROM so copter-heli.parm defaults take effect on fresh boot.
    # This ensures SCR_ENABLE=1 and RPM1_TYPE=10 are active from the start.
    _eeprom = Path("/ardupilot/eeprom.bin")
    if _eeprom.exists():
        _eeprom.unlink()

    # Install Lua script before SITL starts so it's loaded at boot
    _install_lua_scripts()

    import model as _m
    omega_axle = _m.OMEGA_AXLE_NOMINAL

    mediator_log = tmp_path / "mediator_lua.log"
    sitl_log     = tmp_path / "sitl.log"
    gcs_log      = tmp_path / "gcs.log"

    _configure_logging(gcs_log)
    log = logging.getLogger("torque_armed_lua")
    log.info("Lua fixture: omega_axle=%.1f rad/s  tail_channel=Ch9", omega_axle)

    mediator_proc = _launch_mediator_torque(
        mediator_log, omega_axle,
        profile="constant",
        tail_channel=8,   # Ch9 — Lua writes here
        lua_mode=True,    # linear mapping, no adaptive trim
    )

    # Launch SITL with lua_defaults.parm pre-loaded so SCR_ENABLE=1 is active
    # from boot (setting it via MAVLink after start has no effect until reboot).
    lua_defaults = str(_TORQUE_DIR / "scripts" / "lua_defaults.parm")
    sitl_proc = _launch_sitl_lua(sim_vehicle, sitl_log, lua_defaults)

    def _assert_alive():
        for name, proc, lp in [
            ("mediator_lua", mediator_proc, mediator_log),
            ("SITL",         sitl_proc,     sitl_log),
        ]:
            if proc.poll() is not None:
                txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else ""
                pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{txt[-2000:]}")

    gcs = RawesGCS(address=GCS_ADDRESS)
    ctx = TorqueStackContext(
        gcs=gcs, mediator_proc=mediator_proc, sitl_proc=sitl_proc,
        mediator_log=mediator_log, sitl_log=sitl_log, gcs_log=gcs_log,
        omega_axle=omega_axle, log=log,
    )

    try:
        log.info("Connecting GCS …")
        gcs.connect(timeout=30.0)
        gcs.start_heartbeat(rate_hz=1.0)
        _assert_alive()

        # Param subsystem
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            _assert_alive()
            gcs._mav.mav.param_request_read_send(
                gcs._target_system, gcs._target_component, b"SYSID_THISMAV", -1,
            )
            msg = gcs._mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is not None:
                log.info("Param subsystem ready"); break
        else:
            pytest.fail("Param subsystem never responded")

        from pymavlink import mavutil as _mavu
        gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
        gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_EXTRA3, 2)

        for pname, pvalue in [
            # Safety
            ("ARMING_SKIPCHK",   0xFFFF), ("FS_EKF_ACTION",   0),
            ("FS_THR_ENABLE",    0),
            # EKF
            ("COMPASS_USE",      1),      ("COMPASS_ENABLE",   1),
            ("EK3_SRC1_YAW",     1),      ("EK3_MAG_CAL",      0),
            ("EK3_GPS_CHECK",    0),
            # RSC
            ("H_RSC_MODE",       1),      ("H_RSC_RUNUP_TIME", 1),
            ("H_TAIL_TYPE",      0),      ("H_COL2YAW",        0.0),
            # Disable helicopter yaw PID — Lua controls tail directly
            ("ATC_RAT_YAW_P",    0.0),   ("ATC_RAT_YAW_I",    0.0),
            ("ATC_RAT_YAW_D",    0.0),   ("ATC_RAT_YAW_IMAX", 0.0),
            ("ATC_RAT_RLL_IMAX", 0.0),   ("ATC_RAT_PIT_IMAX", 0.0),
            # Lua scripting — enable and set RPM source
            ("SCR_ENABLE",       1),      # enable Lua scripting
            ("RPM1_TYPE",        10),     # SITL: read rpm from JSON packet
            ("RPM1_MIN",         0),
            # SERVO9 → Script 1 (Lua controls Ch9 exclusively)
            ("SERVO9_FUNCTION",  94),
        ]:
            _set_param(gcs, pname, pvalue, log)
            _assert_alive()

        # EKF alignment
        log.info("Waiting for EKF alignment …")
        ekf_ok   = False
        t_start  = time.monotonic()
        t_rc     = time.monotonic()
        deadline = time.monotonic() + 45.0
        gcs.send_rc_override({8: 2000})
        while time.monotonic() < deadline:
            _assert_alive()
            if time.monotonic() - t_rc >= 0.5:
                gcs.send_rc_override({8: 2000})
                t_rc = time.monotonic()
            msg = gcs._mav.recv_match(
                type=["ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.5,
            )
            if msg is None: continue
            if msg.get_type() == "STATUSTEXT":
                text = msg.text.rstrip("\x00").strip()
                log.info("SITL: %s", text)
                if "EKF3 active" in text or "EKF3 IMU" in text:
                    gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
                if "yaw alignment complete" in text.lower():
                    if time.monotonic() - t_start >= 3.0:
                        ekf_ok = True; break
                if "rawes yaw trim" in text.lower():
                    log.info("Lua script confirmed loaded: %s", text)
            elif msg.get_type() == "ATTITUDE":
                if (all(_math.isfinite(v) for v in (msg.roll, msg.pitch, msg.yaw))
                        and time.monotonic() - t_start >= 3.0):
                    ekf_ok = True; break
        if not ekf_ok:
            log.warning("EKF alignment timed out — proceeding (SKIPCHK set)")

        gcs.arm(timeout=15.0, force=True, rc_override={8: 2000})
        log.info("Armed")
        gcs.set_mode(ACRO, timeout=10.0, rc_override={8: 2000})
        log.info("ACRO active — Lua yaw trim running on Ch9")

        yield ctx

    finally:
        try: gcs.close()
        except Exception: pass
        _terminate_process(mediator_proc)
        _terminate_process(sitl_proc)
        _kill_by_port(SITL_GCS_PORT)
        copy_logs_to_dir(_SIM_DIR / "logs", {
            "mediator_lua_last.log": mediator_log,
            "sitl_lua_last.log":     sitl_log,
            "gcs_lua_last.log":      gcs_log,
        })
        log.info("Logs copied")
