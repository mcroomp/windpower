"""
conftest.py — shared pytest fixtures and helpers for RAWES stack integration tests.

The ``acro_armed`` fixture handles the full stack lifecycle:
    launch mediator + SITL → connect GCS → set ACRO params →
    wait for EKF tilt alignment → arm (force) → confirm ACRO mode → yield

Tests receive a ``StackContext`` and can immediately start their observation phase.
Process teardown and log copying happen automatically after each test.

Diagnostic helpers
------------------
``analyze_startup_logs(ctx)``  — parse mediator + GCS logs, return structured dict
``dump_startup_diagnostics(ctx)`` — print full human-readable diagnostic block
``wait_for_acro_stability(gcs, log)`` — wait for clean ATTITUDE messages
``drain_statustext(gcs, log)``  — drain buffered STATUSTEXT, return list
"""
import dataclasses
import json
import logging
import math
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest

_SIM_DIR   = Path(__file__).resolve().parents[2]
_STACK_DIR = Path(__file__).resolve().parent
_TORQUE_DIR = _SIM_DIR / "torque"

sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_STACK_DIR))
sys.path.insert(0, str(_TORQUE_DIR))

import rotor_definition as _rd

from stack_utils import (
    ARDUPILOT_ENV,
    STACK_ENV_FLAG,
    SIM_VEHICLE_ENV,
    _launch_mediator,
    _launch_mediator_torque,
    _launch_sitl,
    _prime_sitl_eeprom,
    _resolve_sim_vehicle,
    _terminate_process,
    _kill_by_port,
    _configure_logging,
    copy_logs_to_dir,
    make_test_log_dir,
    check_ports_free,
)
import socket as _socket

from pymavlink import mavutil as _mavutil
from gcs import ACRO, STABILIZE, RawesGCS
from controller import make_hold_controller

_STARTING_STATE       = _SIM_DIR / "steady_state_starting.json"
_RAWES_DEFAULTS_PARM  = _STACK_DIR / "rawes_sitl_defaults.parm"





def pytest_addoption(parser):
    pass

# ---------------------------------------------------------------------------
# StackConfig — central configuration and pre-flight verification
# ---------------------------------------------------------------------------

class StackConfig:
    """
    Central configuration for the RAWES simulation stack.

    Holds all port assignments, addresses, and timeout values used by the
    stack.  Call ``verify()`` before launching any stack processes to catch
    port conflicts and missing prerequisites with a clear error message
    instead of a confusing 30-second timeout.

    Usage
    -----
    All stack tests should call ``StackConfig().verify()`` (or use the
    module-level ``assert_stack_ports_free()`` shorthand) before launching
    SITL or the mediator.

    Constants (class-level, use directly without instantiation)
    -----------------------------------------------------------
    SITL_GCS_PORT  : TCP port SITL listens on for GCS MAVLink connections
    SITL_JSON_PORT : UDP port mediator/sensor worker binds to for SITL servo data
    GCS_ADDRESS    : pymavlink connection string for the GCS
    """

    # Ports
    SITL_GCS_PORT  : int = 5760   # TCP — ArduPilot SITL MAVLink
    SITL_JSON_PORT : int = 9002   # UDP — SITL sends servo outputs here; mediator receives + sends back

    # Derived addresses
    GCS_ADDRESS : str = f"tcp:127.0.0.1:{SITL_GCS_PORT}"

    # Timeouts (seconds)
    CONNECT_TIMEOUT       : float = 30.0
    ARM_TIMEOUT           : float = 120.0
    MODE_TIMEOUT          : float = 60.0
    EKF_ALIGN_TIMEOUT     : float = 45.0
    EKF_STABILISE_TIMEOUT : float = 30.0
    # 65 s kinematic startup: hub moves at constant vel0 ≈ 0.96 m/s from
    # launch_pos to equilibrium pos0.  Kinematic must last past the arm attempt
    # so the hub is still frozen (constant velocity) when we arm — hub motion
    # after kinematic-end causes GPS innovations that corrupt EKF3.
    #
    # Timeline with EK3_SRC1_YAW=1 (compass yaw):
    #   ~4 s  tiltAlignComplete + yawAlignComplete
    #   ~10 s GPS detected
    #   ~21 s EKF3 GPS origin set (GPS fusion begins)
    #   ~54 s delAngBiasLearned → GPS position fuses → LOCAL_POSITION_NED
    #          (54 s = 21 s origin + ~33 s bias learning from compass-yaw start)
    #
    # Arm attempt fires at ~54 s from mediator start (50 s from EKF non-zero at ~4 s).
    # 65 s gives 11 s margin: hub is still kinematic at arm time, GPS is healthy.
    # After arm (~54 s), kinematic runs 11 more seconds before physics starts.
    STARTUP_DAMP_S        : float = 65.0
    LOCK_ORIENTATION      : bool  = True   # lock R_hub=R0 during kinematic so compass heading is constant (required for EK3_SRC1_YAW=1)
    # Permanent angular damping after startup ramp ends.
    # 50 N·m·s/rad (mediator default) is insufficient for the 10 Hz MAVLink
    # controller in physical-sensor mode — aerodynamic cyclic moments after
    # kinematic end overcome it before the controller can respond.
    # 300 N·m·s/rad gives τ=I/k=5/300≈17 ms — fast enough for 10 Hz hold.
    BASE_K_ANG            : float = 300.0
    # Phase 2: truth-state controller at 400 Hz inside the mediator.
    # When True, ArduPilot servo outputs are ignored in free flight;
    # the hold loop sends neutral RC sticks (motor-interlock keepalive only).
    # With the 400 Hz controller providing rate damping, BASE_K_ANG can be
    # reduced back to 50 (the mediator default); 300 was only needed for the
    # 10 Hz MAVLink controller which could not respond fast enough.
    INTERNAL_CONTROLLER   : bool  = True
    BASE_K_ANG_INTERNAL   : float = 50.0   # used when INTERNAL_CONTROLLER is True

    # Port descriptions for diagnostics
    _PORT_CHECKS = [
        ("127.0.0.1", SITL_GCS_PORT,  "tcp",
         "SITL GCS port — a previous SITL process may still be running"),
        ("0.0.0.0",   SITL_JSON_PORT, "udp",
         "mediator/sensor JSON port — a previous mediator or sensor worker may still be running"),
    ]

    @classmethod
    def verify(cls) -> None:
        """
        Check all stack ports and required resources before launching processes.

        Raises RuntimeError immediately with a clear, actionable message if
        anything is wrong.  Call this at the very start of any stack test or
        fixture, before launching SITL or the mediator.

        Checks performed
        ----------------
        - All stack UDP/TCP ports are free (no lingering processes)
        """
        cls._check_ports()

    @classmethod
    def _check_ports(cls) -> None:
        """Ensure all required ports are free before launching SITL.

        Strategy:
        1. Kill any process currently holding the port immediately (no waiting).
        2. Also kill lingering arducopter/sim_vehicle processes by name.
        3. Wait 2 s for the OS to release sockets.
        4. Verify the port is free.
        5. If still busy, restart the container (last resort).
        """
        import time as _time
        import subprocess as _sp
        import logging as _log

        _clog = _log.getLogger("conftest")

        # Step 1: kill any process holding the ports by port number (targeted, not by name).
        # Using fuser -k rather than pkill-by-name avoids killing processes mid-EEPROM-write
        # which corrupts eeprom.bin and causes a reload loop on the next SITL boot.
        for host, port, proto, hint in cls._PORT_CHECKS:
            if proto == "tcp":
                _kill_by_port(port)

        # Step 2: brief wait for sockets to be released
        _time.sleep(1.0)

        # Step 4: verify ports are free
        for host, port, proto, hint in cls._PORT_CHECKS:
            kind = _socket.SOCK_STREAM if proto == "tcp" else _socket.SOCK_DGRAM
            s = _socket.socket(_socket.AF_INET, kind)
            s.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
            try:
                s.bind((host, port))
                s.close()
            except OSError as exc:
                s.close()
                # Step 5: last resort — restart the container via dev.sh
                _clog.warning(
                    "Port %s:%d still busy after kill — restarting container", host, port
                )
                import os as _os
                _script = _os.path.join(_os.path.dirname(__file__), "..", "..", "dev.sh")
                _sp.run(["bash", _script, "stop"], capture_output=True, check=False)
                _sp.run(["bash", _script, "start"], capture_output=True, check=False)
                _time.sleep(3.0)
                # Final check after restart
                s2 = _socket.socket(_socket.AF_INET, kind)
                s2.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
                try:
                    s2.bind((host, port))
                    s2.close()
                except OSError as exc2:
                    s2.close()
                    raise RuntimeError(
                        f"{proto.upper()} port {host}:{port} still busy after container restart.\n"
                        f"  Hint: {hint}\n  Error: {exc2}"
                    ) from exc2


# Convenience aliases — use StackConfig directly for new code
SITL_GCS_PORT   = StackConfig.SITL_GCS_PORT
SITL_JSON_PORT  = StackConfig.SITL_JSON_PORT


def assert_stack_ports_free() -> None:
    """Convenience wrapper — calls StackConfig.verify()."""
    StackConfig.verify()


# Module-level aliases — single source of truth via StackConfig.
_STARTUP_TIMEOUT  = StackConfig.CONNECT_TIMEOUT
_ARM_TIMEOUT      = StackConfig.ARM_TIMEOUT
_MODE_TIMEOUT     = StackConfig.MODE_TIMEOUT
_STARTUP_DAMP_S = StackConfig.STARTUP_DAMP_S


# ---------------------------------------------------------------------------
# StackContext
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class StackContext:
    """
    Everything a stack test needs after the shared setup has run.

    Attributes
    ----------
    gcs            : connected, heartbeating RawesGCS (armed, in ACRO mode)
    mediator_proc  : running mediator subprocess
    sitl_proc      : running SITL subprocess
    mediator_log   : path to mediator stdout/stderr log
    sitl_log       : path to SITL stdout/stderr log
    gcs_log        : path to GCS/test structured log
    telemetry_log  : path to mediator telemetry CSV
    initial_state  : dict from steady_state_starting.json (vel overridden to 0)
    home_alt_m     : hub altitude above anchor [m] at launch (positive = above ground)
    flight_events  : timing checkpoints; setup populates, tests add their own
    all_statustext : all STATUSTEXT messages seen during setup
    setup_samples  : list of dicts — EKF/ATTITUDE samples captured during setup
    log            : logger scoped to the running fixture/test
    sim_dir        : simulation/ directory (for writing outputs)
    """
    gcs:            RawesGCS
    mediator_proc:  object
    sitl_proc:      object
    mediator_log:   Path
    sitl_log:       Path
    gcs_log:        Path
    telemetry_log:  Path
    initial_state:  dict | None
    home_alt_m:     float
    flight_events:  dict
    all_statustext: list
    setup_samples:  list      # EKF/ATTITUDE samples collected during setup
    log:            logging.Logger
    sim_dir:        Path
    controller:          object    # PhysicalHoldController
    internal_controller: bool      # True = mediator runs truth-state controller at 400 Hz


# ---------------------------------------------------------------------------
# _acro_stack — single context manager behind all ACRO fixtures
# ---------------------------------------------------------------------------

import contextlib
import numpy as _np

@contextlib.contextmanager
def _acro_stack(tmp_path, *, extra_config=None, log_name="acro_armed", log_prefix="",
                arm: bool = True, with_mediator: bool = True, test_name: str = "",
                prime_eeprom: bool = False):
    """
    Core ACRO stack lifecycle: pre-checks → launch → [arm] → yield ctx → teardown.

    All fixtures (acro_armed, acro_armed_pumping, acro_armed_lua) call this.
    Differences between them are handled outside this function:
      - extra_config   : pumping cycle passes trajectory config here
      - pre-launch work: Lua fixture installs scripts before calling this
      - post-arm work  : Lua fixture sets SCR_USER params inside the with-block

    Parameters
    ----------
    tmp_path       : pytest tmp_path fixture value
    extra_config   : optional dict merged into mediator config (pumping cycle)
    log_name       : logger name (shown in log output)
    log_prefix     : prefix component for persistent log file names
    arm            : if False, skip _run_acro_setup — yield with procs running
                     but GCS not connected and vehicle not armed.  Callers may
                     call ctx.gcs.connect() manually (e.g. smoke tests).
    with_mediator  : if False, skip mediator launch (pure SITL connectivity tests)
    test_name      : pytest test node name; included in persistent log file names
                     so each test run produces uniquely-named files in logs/
    """
    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run stack integration tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} to locate sim_vehicle.py")

    pytest.importorskip("pymavlink")
    assert_stack_ports_free()

    repo_root = Path(__file__).resolve().parents[3]
    sim_dir   = repo_root / "simulation"

    # ── Initial state ──────────────────────────────────────────────────────────
    # pos[2] is NED Z; altitude above ground = -pos[2].
    # vel0 is intentionally NOT loaded from steady_state_starting.json — the
    # kinematic startup ramp needs vel0 from config.py defaults (~0.96 m/s) so
    # the EKF gets a velocity-derived yaw heading from frame 0.
    initial_state = None
    home_alt_m    = 12.530
    if _STARTING_STATE.exists():
        initial_state = json.loads(_STARTING_STATE.read_text())
        home_alt_m    = -float(initial_state["pos"][2])
        initial_state = dict(initial_state)

    # ── Paths ──────────────────────────────────────────────────────────────────
    mediator_log  = tmp_path / "mediator.log"
    sitl_log      = tmp_path / "sitl.log"
    gcs_log       = tmp_path / "gcs.log"
    telemetry_log = tmp_path / "telemetry.csv"

    _configure_logging(gcs_log)
    log = logging.getLogger(log_name)
    logging.getLogger("gcs").setLevel(logging.DEBUG)

    # ── Hold controller ────────────────────────────────────────────────────────
    # Anchor in LOCAL_POSITION_NED frame:
    #   sensor.py sets home_ned_z = -home_alt_m; pos_ned_rel[2] = pos_ned[2] + home_alt_m
    #   Physics anchor NED [0,0,0] maps to LOCAL [0, 0, home_alt_m].
    _anchor_ned = _np.array([0.0, 0.0, float(home_alt_m)])
    controller   = make_hold_controller(anchor_ned=_anchor_ned)
    log.info("Hold controller: %s", type(controller).__name__)

    # ── Lua scripts ───────────────────────────────────────────────────────────
    # Always install rawes.lua so it is available if scripting is enabled.
    _install_lua_scripts("rawes.lua")

    # ── EEPROM / Lua: SCR_ENABLE=1 is set via MAVLink in step 3 and persists.
    # For the Lua fixture specifically: prime_eeprom=True runs a short SITL boot
    # to pre-seed SCR_ENABLE=1 into eeprom.bin on a fresh container where no
    # prior test has set it yet.  Subsequent runs skip the prime (eeprom.bin
    # already has SCR_ENABLE=1 from the previous Lua test's step 3 write).
    if prime_eeprom:
        ardupilot_root = sim_vehicle.parent.parent.parent
        eeprom_exists = (ardupilot_root / "eeprom.bin").exists()
        if eeprom_exists:
            log.info("%s: EEPROM has SCR_ENABLE=1 from prior run — skipping prime.", log_name)
        else:
            log.info("%s: Fresh container — priming EEPROM for Lua scripting ...", log_name)
            _prime_sitl_eeprom(sim_vehicle)
            log.info("%s: EEPROM primed.", log_name)

    # ── Launch ────────────────────────────────────────────────────────────────
    _run_id = int(time.time())
    log.info("RUN_ID=%d", _run_id)
    log.info("%s: launching%s SITL ...", log_name, " mediator +" if with_mediator else "")
    if with_mediator:
        mediator_proc = _launch_mediator(
            sim_dir, repo_root, mediator_log,
            telemetry_log_path   = str(telemetry_log),
            initial_state        = initial_state,
            startup_damp_seconds = _STARTUP_DAMP_S,
            lock_orientation     = StackConfig.LOCK_ORIENTATION,
            run_id               = _run_id,
            base_k_ang           = StackConfig.BASE_K_ANG_INTERNAL,
            internal_controller  = True,
            extra_config         = extra_config,
        )
    else:
        mediator_proc = None
    sitl_proc = _launch_sitl(sim_vehicle, sitl_log,
                             add_param_file=_RAWES_DEFAULTS_PARM)

    def _procs_alive():
        _checks = [("SITL", sitl_proc, sitl_log)]
        if with_mediator:
            _checks.insert(0, ("mediator", mediator_proc, mediator_log))
        for name, proc, lp in _checks:
            if proc.poll() is not None:
                txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{txt[-3000:]}")

    gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)

    ctx = StackContext(
        gcs=gcs, mediator_proc=mediator_proc, sitl_proc=sitl_proc,
        mediator_log=mediator_log, sitl_log=sitl_log, gcs_log=gcs_log,
        telemetry_log=telemetry_log, initial_state=initial_state,
        home_alt_m=home_alt_m, flight_events={}, all_statustext=[],
        setup_samples=[], log=log, sim_dir=sim_dir,
        controller=controller,
        internal_controller=True,
    )

    # Per-test log directory: simulation/logs/{label}/
    label = "_".join(filter(None, [log_prefix, test_name]))
    test_log_dir = make_test_log_dir(sim_dir, label) if label else sim_dir / "logs"

    try:
        if arm:
            _run_acro_setup(ctx, _procs_alive)
        yield ctx
    finally:
        gcs.close()
        _terminate_process(sitl_proc)
        if mediator_proc is not None:
            _terminate_process(mediator_proc)
        _kill_by_port(StackConfig.SITL_GCS_PORT)
        _logs = {
            "sitl.log": sitl_log,
            "gcs.log":  gcs_log,
        }
        if with_mediator:
            _logs["mediator.log"]  = mediator_log
            _logs["telemetry.csv"] = telemetry_log
        copy_logs_to_dir(test_log_dir, _logs)
        # Copy the ArduCopter terminal log (truncated per-test in _launch_sitl)
        _ardupilot_log = Path("/tmp/ArduCopter.log")
        if _ardupilot_log.exists():
            import shutil as _shutil2
            _shutil2.copy2(_ardupilot_log, test_log_dir / "arducopter.log")


# ---------------------------------------------------------------------------
# Fixtures — thin wrappers around _acro_stack
# ---------------------------------------------------------------------------

@pytest.fixture
def acro_armed(tmp_path, request):
    """Full ACRO stack fixture. Yields StackContext armed in ACRO mode."""
    with _acro_stack(tmp_path, test_name=request.node.name) as ctx:
        yield ctx


@pytest.fixture
def acro_armed_pumping(tmp_path, request):
    """
    Pumping-cycle variant of acro_armed.

    Mediator is launched with trajectory=deschutter so the pumping-cycle
    state machine runs (internal_controller=True required).
    """
    import config as _mcfg
    _dcfg = _mcfg.DEFAULTS["trajectory"]["deschutter"]
    extra = {
        "trajectory": {
            "type": "deschutter",
            "hold": {},
            "deschutter": {k: _dcfg[k] for k in (
                "t_reel_out", "t_reel_in", "t_transition",
                "v_reel_out", "v_reel_in", "tension_out", "tension_in",
            )},
        },
    }
    with _acro_stack(tmp_path, extra_config=extra,
                     log_name="acro_armed_pumping", log_prefix="pumping",
                     test_name=request.node.name) as ctx:
        yield ctx


# ---------------------------------------------------------------------------
# Lua flight fixture helpers
# ---------------------------------------------------------------------------

_SCRIPTS_DIR = _SIM_DIR / "scripts"  # simulation/scripts/ — all Lua scripts


def _install_lua_scripts(*names: str) -> None:
    """
    Copy the named Lua scripts from simulation/scripts/ to SITL's /ardupilot/scripts/.

    Must be called before SITL starts.  In normal use, pass only "rawes.lua"
    (the unified script); SCR_USER7 selects the active mode at runtime.

    Examples::

        _install_lua_scripts("rawes.lua")
    """
    dst_dir = Path("/ardupilot/scripts")
    dst_dir.mkdir(exist_ok=True)
    for name in names:
        src = _SCRIPTS_DIR / name
        shutil.copy2(src, dst_dir / name)


@pytest.fixture
def acro_armed_lua(tmp_path, request):
    """
    ACRO stack fixture with rawes.lua active in flight mode (SCR_USER7=1).

    Extends acro_armed with:
      1. rawes.lua installed to /ardupilot/scripts/ before SITL starts.
      2. SCR_USER1..5 (kp, slew rate, anchor NED) and SCR_USER7=1 set via MAVLink after arm.

    Lua's RC overrides are observable from SERVO_OUTPUT_RAW while the
    mediator's internal controller stabilises physics (internal_controller=True).
    """
    with _acro_stack(tmp_path, log_name="acro_armed_lua", log_prefix="lua",
                     test_name=request.node.name, prime_eeprom=True) as ctx:
        # Post-arm: configure rawes.lua via SCR_USER params.
        # SCR_USER5 = home_alt_m: anchor is home_alt_m metres below EKF HOME
        # (sensor.py sets pos_ned_rel[2] = pos_ned[2] + home_alt_m, so physics
        # anchor NED [0,0,0] appears at LOCAL [0, 0, home_alt_m]).
        ctx.log.info("Setting SCR_USER params for rawes.lua (flight mode) ...")
        lua_params = {
            "SCR_ENABLE": 1,              # persist scripting in EEPROM for future boots
            "SCR_USER1": 1.0,             # RAWES_KP_CYC   [rad/s / rad]
            "SCR_USER2": 0.40,            # RAWES_BZ_SLEW  [rad/s]
            "SCR_USER3": 0.0,             # anchor North   [m]
            "SCR_USER4": 0.0,             # anchor East    [m]
            "SCR_USER5": ctx.home_alt_m,  # anchor Down from EKF HOME [m]
            "SCR_USER7": 1,               # RAWES_MODE = 1 (flight only)
        }
        for pname, pvalue in lua_params.items():
            ok = ctx.gcs.set_param(pname, pvalue, timeout=5.0)
            ctx.log.info("  %-12s = %g  ACK=%s", pname, pvalue, ok)

        # Drain any STATUSTEXT that arrived during param setting (including
        # "RAWES flight: captured" which Lua may have sent while the GCS was
        # busy with PARAM_VALUE exchanges and couldn't process STATUSTEXT).
        import time as _time
        _t_drain = _time.monotonic() + 1.0
        while _time.monotonic() < _t_drain:
            _msg = ctx.gcs._mav.recv_match(type=["STATUSTEXT"], blocking=True, timeout=0.1)
            if _msg is not None:
                _text = _msg.text.rstrip("\x00").strip()
                ctx.all_statustext.append(_text)
                ctx.log.info("  [drain] STATUSTEXT: %s", _text)

        yield ctx


# ---------------------------------------------------------------------------
# Setup sequence
# ---------------------------------------------------------------------------

def _run_acro_setup(ctx: StackContext, _procs_alive) -> None:
    """
    Shared ACRO setup sequence.

    Timing contract: the mediator startup damping ramp is 30 s.  All steps
    here must complete inside that window so the hub is barely moving when
    we arm — strong damping keeps it near the initial position during GPS init.

    Steps:
        1. Connect GCS; request telemetry streams; clear motor interlock
        2. Wait for param subsystem
        3. Set ACRO parameters  (H_RUNUP_TIME intentionally omitted — force
           arm bypasses that check, and attempting to set it wastes 23 s)
        4. Wait for EKF tilt alignment — FAIL HARD if it doesn't arrive
        5. Arm with force=True (motor interlock low → arm → raise interlock)
        6. Confirm ACRO mode

    Populates ctx.flight_events with timing checkpoints.
    Populates ctx.setup_samples with every EKF/ATTITUDE/position sample.
    Raises RuntimeError on any unrecoverable failure so the fixture
    finally-block still runs cleanup.
    """
    gcs            = ctx.gcs
    log            = ctx.log
    all_statustext = ctx.all_statustext
    setup_samples  = ctx.setup_samples

    # ── 1. Connect ────────────────────────────────────────────────────────────
    log.info("[setup 1/6] Connecting GCS (timeout=%.0fs) ...", _STARTUP_TIMEOUT)
    gcs.connect(timeout=_STARTUP_TIMEOUT)
    gcs.start_heartbeat(rate_hz=1.0)
    _procs_alive()

    # Request telemetry streams so ArduPilot sends ATTITUDE, EKF, and position.
    # Without this, SITL often sends no messages on a plain TCP connection.
    log.info("[setup 1/6] Requesting telemetry streams ...")
    gcs._mav.mav.request_data_stream_send(
        gcs._target_system, gcs._target_component,
        _mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1,   # all streams at 10 Hz
    )
    # Also send a targeted ATTITUDE request to be sure
    gcs._mav.mav.request_data_stream_send(
        gcs._target_system, gcs._target_component,
        _mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1,  # ATTITUDE
    )
    gcs._mav.mav.request_data_stream_send(
        gcs._target_system, gcs._target_component,
        _mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5, 1,  # EKF_STATUS
    )
    gcs._mav.mav.request_data_stream_send(
        gcs._target_system, gcs._target_component,
        _mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1,  # LOCAL_POSITION_NED
    )

    # Motor interlock must be LOW (CH8=1000) during arm.
    # In H_RSC_MODE=1 (passthrough), motor_interlock_switch = (pilot_rotor_speed > 0.01).
    # CH8=2000 → motor_interlock_switch=True → "Arm: Motor Interlock Enabled" blocks arm
    # even with force=True.  CH8=1000 → motor_interlock_switch=False → arm succeeds.
    # After arm confirmation, CH8 is raised to 2000 so the rotor interlock engages.
    log.info("[setup 1/6] Sending motor interlock LOW (CH8=1000) ...")
    gcs.send_rc_override({8: 1000})

    # ── 2. Param subsystem ────────────────────────────────────────────────────
    log.info("[setup 2/6] Waiting for param subsystem ...")
    _wait_params_ready(gcs, log)
    _procs_alive()

    # ── 3. ACRO parameters ────────────────────────────────────────────────────
    # H_RSC_MODE=1 (CH8 Passthrough): RSC output = CH8 input directly.
    #   No ramp / runup sequence — runup_complete fires immediately when CH8
    #   is high.  SITL default (0) is INVALID ("H_RSC_MODE invalid" auto-
    #   disarm).  Modes 2–4 all require a runup wait AND motor interlock ON.
    #   Mode 1 with CH8=2000 gives instant runup_complete so HEARTBEAT shows
    #   armed=True immediately after the force arm ACK.
    #   Note: AP_MotorsHeli::output() clears _flags.armed if runup_complete
    #   is not set, regardless of force arm — this is why CH8 must be HIGH.
    log.info("[setup 3/6] Setting ACRO parameters ...")
    # All swashplate params are set explicitly so test results never depend on
    # ArduPilot defaults (which may change between builds).
    _base_params = {
        "ARMING_SKIPCHK":   0xFFFF, # skip ALL pre-arm checks (bitmask: each bit = one check to skip)
        "SCR_ENABLE":       1,      # enable Lua scripting; persisted to EEPROM for acro_armed_lua
        "INITIAL_MODE":     1,      # boot into ACRO
        "FS_THR_ENABLE":    0,      # no RC throttle failsafe
        "FS_GCS_ENABLE":    0,      # no GCS heartbeat failsafe
        "FS_EKF_ACTION":    0,      # disable EKF failsafe
        "H_RSC_MODE":       1,      # CH8 passthrough — instant runup_complete
        # Disable ACRO auto-leveling trainer: physical sensor mode reports
        # roll≈58°/pitch≈-35° at tether equilibrium.  ACRO_TRAINER=1 (default)
        # activates above 45° and commands large leveling corrections → rapid tumble.
        "ACRO_TRAINER":     0,
        # Zero ACRO I-term limits: orbital angular velocity accumulates as a
        # persistent rate error, causing ever-growing cyclic and hub crash (~4 s).
        "ATC_RAT_RLL_IMAX": 0.0,
        "ATC_RAT_PIT_IMAX": 0.0,
        "ATC_RAT_YAW_IMAX": 0.0,
        # Swashplate geometry — set explicitly to avoid build-to-build default drift.
        # H3_120: CH_1 at -60 deg, CH_2 at +60 deg, CH_3 at 180 deg.
        # H_SW_PHANG=0: no phase correction; ArduPilot H3_120 +90 deg roll advance
        # aligns with RAWES servo layout (S1=0 deg/East, S2=120 deg, S3=240 deg).
        "H_SW_TYPE":        3,      # 3 = H3_120  (subgroup H_SW_ + TYPE)
        "H_SW_PHANG":       0,      # deg, reset to known state before each test
        # Servo speed — derived from RotorDefinition to stay in sync with SwashplateServoModel
        # in simtests.  SIM_SERVO_SPEED units: full output range (0..1) per second.
        # DS113MG at 6V: 545 deg/s / 100 deg travel = 5.45
        "SIM_SERVO_SPEED":  _rd.default().sim_servo_speed,
    }
    _acro_params = {**_base_params, **ctx.controller.extra_params}
    for name, value in _acro_params.items():
        ok = gcs.set_param(name, value, timeout=5.0)
        if not ok:
            log.warning("  %s=%s had no ACK — continuing", name, value)

    # EK3 GPS/compass params are set as boot defaults in rawes_sitl_defaults.parm
    # (EK3_SRC1_YAW=1, EK3_MAG_CAL=0, EK3_GPS_CHECK=0, COMPASS_USE=1,
    # COMPASS_ENABLE=1).  We do NOT set them via MAVLink here — resetting the same
    # value via MAVLink after boot can trigger a subtle EKF3 yaw-state reset that
    # prevents GPS fusion (flags stay at 0x0400 GPS glitch instead of reaching
    # 0x0010 horiz_pos_abs).

    # ── 4. EKF attitude alignment ─────────────────────────────────────────────
    # Wait for a clean ATTITUDE (EKF attitude ready) before arming.
    # We also watch for LOCAL_POSITION_NED (EKF position ready) for diagnostics
    # but do NOT require it — GPS position lock may still be initialising.
    # force=True arm bypasses any EKF position pre-arm check.
    #
    # FAIL HARD if ATTITUDE doesn't arrive within 45 s.
    log.info("[setup 4/6] Waiting for EKF attitude alignment (timeout=45s) ...")
    t_ekf     = None
    ekf_att   = False   # seen a finite ATTITUDE
    ekf_pos   = False   # seen a LOCAL_POSITION_NED (optional, diagnostic only)
    ekf_ok    = False   # ATTITUDE ready
    deadline  = time.monotonic() + 45.0
    while time.monotonic() < deadline and not ekf_ok:
        _procs_alive()
        msg = gcs._mav.recv_match(
            type=["ATTITUDE", "EKF_STATUS_REPORT", "STATUSTEXT",
                  "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"],
            blocking=True, timeout=1.0,
        )
        if msg is None:
            continue
        t_now = time.monotonic()
        mt    = msg.get_type()

        if mt == "STATUSTEXT":
            text = msg.text.rstrip("\x00").strip()
            sev  = getattr(msg, "severity", "?")
            log.info("[setup 4/6] STATUSTEXT [sev=%s]: %s", sev, text)
            all_statustext.append(text)
            setup_samples.append({"t": t_now, "type": "STATUSTEXT",
                                   "severity": sev, "text": text})
            if "tilt alignment" in text.lower():
                log.info("[setup 4/6] EKF tilt alignment confirmed via STATUSTEXT.")
            # GPS Glitch during the freeze means something is wrong with our
            # sensor packet — fail immediately with diagnostics.
            if "gps glitch" in text.lower():
                _diag = analyze_startup_logs(ctx)
                dump_startup_diagnostics(ctx)
                raise RuntimeError(
                    "GPS Glitch detected during EKF wait (hub should be frozen).\n"
                    "Likely cause: mediator GPS position or velocity inconsistent "
                    "with IMU data in sensor packet.\n"
                    f"Known issues: {_diag['known_issues']}"
                )

        elif mt == "ATTITUDE":
            r, p, y = math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw)
            rr, pr, yr = (math.degrees(msg.rollspeed),
                          math.degrees(msg.pitchspeed),
                          math.degrees(msg.yawspeed))
            log.info("[setup 4/6] ATTITUDE  rpy=(%.2f°,%.2f°,%.2f°)  "
                     "rates=(%.2f,%.2f,%.2f)°/s", r, p, y, rr, pr, yr)
            setup_samples.append({"t": t_now, "type": "ATTITUDE",
                                   "roll": r, "pitch": p, "yaw": y,
                                   "rollspeed": rr, "pitchspeed": pr, "yawspeed": yr})
            if all(math.isfinite(v) for v in (r, p, y)) and not ekf_att:
                log.info("[setup 4/6] Clean ATTITUDE — EKF attitude ready.")
                ekf_att = True
                t_ekf   = t_now
                # Capture equilibrium orientation for PhysicalHoldController.
                # This is the tether equilibrium the hub holds during the 45 s
                # kinematic startup (R locked to R0).  The controller uses
                # (roll - roll_eq, pitch - pitch_eq) as a yaw-independent error
                # so corrections stay valid even when velocity-derived yaw jumps
                # ~150° at kinematic end when the tether activates.
                if hasattr(ctx.controller, "set_equilibrium"):
                    ctx.controller.set_equilibrium(msg.roll, msg.pitch)
                    log.info("[setup 4/6] Equilibrium set: roll_eq=%.2f° pitch_eq=%.2f°",
                             r, p)
            ekf_ok = ekf_att

        elif mt == "EKF_STATUS_REPORT":
            flags = msg.flags
            log.info("[setup 4/6] EKF_STATUS  flags=0x%04x  vel_var=%.3f  "
                     "pos_var=%.3f  hgt_var=%.3f",
                     flags,
                     getattr(msg, "velocity_variance", float("nan")),
                     getattr(msg, "pos_horiz_variance", float("nan")),
                     getattr(msg, "pos_vert_variance", float("nan")))
            setup_samples.append({"t": t_now, "type": "EKF_STATUS", "flags": flags})
            if flags & _mavutil.mavlink.EKF_ATTITUDE and not ekf_att:
                log.info("[setup 4/6] EKF_ATTITUDE flag set (flags=0x%04x).", flags)
                ekf_att = True
                t_ekf   = t_now
            ekf_ok = ekf_att

        elif mt == "LOCAL_POSITION_NED":
            log.info("[setup 4/6] LOCAL_POSITION_NED  N=%.2f  E=%.2f  D=%.2f  "
                     "vN=%.3f  vE=%.3f  vD=%.3f",
                     msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
            setup_samples.append({"t": t_now, "type": "LOCAL_POSITION_NED",
                                   "N": msg.x, "E": msg.y, "D": msg.z,
                                   "vN": msg.vx, "vE": msg.vy, "vD": msg.vz})
            if not ekf_pos:
                log.info("[setup 4/6] LOCAL_POSITION_NED received — EKF has position.")
                ekf_pos = True
            # ekf_ok does not depend on ekf_pos — ATTITUDE alone is sufficient

        elif mt == "GLOBAL_POSITION_INT":
            log.info("[setup 4/6] GLOBAL_POSITION_INT  lat=%d  lon=%d  alt_mm=%d",
                     msg.lat, msg.lon, msg.alt)
            setup_samples.append({"t": t_now, "type": "GLOBAL_POSITION_INT",
                                   "lat": msg.lat, "lon": msg.lon, "alt_mm": msg.alt})

    # FAIL HARD — do not proceed with a broken EKF
    if not ekf_ok:
        dump_startup_diagnostics(ctx)
        raise RuntimeError(
            "EKF attitude not confirmed within 45 s.\n"
            "The hub is frozen during this window so GPS should be perfectly "
            "stable — if ATTITUDE never arrives, mediator is not sending valid "
            "IMU data (accel_body or gyro_body).\n"
            f"Samples received: {[s['type'] for s in setup_samples]}\n"
            f"STATUSTEXT: {all_statustext}"
        )
    if not ekf_pos:
        log.warning("[setup 4/6] WARNING: no LOCAL_POSITION_NED during EKF wait "
                    "(GPS+compass enabled — EKF may still be initialising; "
                    "force arm will proceed)")

    all_statustext += drain_statustext(gcs, log)
    _procs_alive()

    # Stabilisation wait: arm immediately when EKF3 first reports attitude
    # confidence (flags & EKF_ATTITUDE = 0x0001).
    #
    # WHY early arm (not waiting for GPS fusion):
    #   With physical sensor mode (hub at roll~124 deg, pitch~-46 deg), EKF3
    #   periodically reinitialises (~41 s after start when GPS fusion is
    #   attempted with large innovations).  After each reinit, EKF3 and DCM
    #   diverge by ~30-140 deg, failing the mandatory attitudes_consistent()
    #   arm check.  The safe window is the ~8 s immediately after EKF3
    #   completes tilt+yaw alignment: both DCM (fast accel correction) and
    #   EKF3 (freshly aligned) agree on the orbital attitude within ~5 deg.
    #
    # TIMING (physical sensor mode, EK3_SRC1_YAW=1, vel0~0.96 m/s):
    #   ~2 s  : ATTITUDE message arrives (DCM converged) - step 4/6 exits
    #   ~5 s  : EKF3 IMU0 initialised, AHRS: EKF3 active
    #   ~8 s  : tiltAlignComplete + yawAlignComplete -> flags=0x00a7 (0x0001 set)
    #   ~41 s : GPS fusion attempt -> large innovations -> EKF3 reinit -> DCM diverges
    #
    # WHAT we wait for:
    #   - EKF_STATUS flags & 0x0001 (EKF_ATTITUDE): EKF3 attitude confidence.
    #     This is the earliest moment both DCM and EKF3 agree on the attitude.
    #   - Still capture LOCAL_POSITION_NED if it arrives (for test_gps_fuses_*).
    #   - Fallback: 20 s timeout (EKF3 should align well within this).
    _EKF_ATT_FLAG  = 0x0001   # bit 0: EKF3 attitude estimate good
    _EKF_POS_FLAG  = 0x0010   # bit 4: horiz_pos_abs (GPS position fused)
    log.info("[setup] Waiting for EKF3 attitude confidence before arming (timeout=20s) ...")
    ekf_att_ready  = False
    last_flags     = 0
    t_stab = time.monotonic() + 20.0
    while time.monotonic() < t_stab:
        _procs_alive()
        msg = gcs._mav.recv_match(
            type=["STATUSTEXT", "LOCAL_POSITION_NED", "EKF_STATUS_REPORT"],
            blocking=True, timeout=0.2,
        )
        if msg is not None:
            mt = msg.get_type()
            if mt == "STATUSTEXT":
                text = msg.text.rstrip("\x00").strip()
                sev  = getattr(msg, "severity", "?")
                log.info("[stabilise] STATUSTEXT [sev=%s]: %s", sev, text)
                all_statustext.append(text)
                # "tilt alignment complete" STATUSTEXT is the definitive signal
                # that EKF3 attitude is ready.  EKF_STATUS_REPORT polling at
                # 5 Hz often misses the brief 0x0001 window before EKF
                # transitions to 0x0400 (EKF_ACCEL_ERROR) when GPS fusion
                # begins.  Acting on the STATUSTEXT avoids this race.
                if "tilt alignment" in text.lower() and not ekf_att_ready:
                    ekf_att_ready = True
                    log.info("[stabilise] EKF tilt alignment confirmed via STATUSTEXT "
                             "-- proceeding to arm immediately.")
            elif mt == "LOCAL_POSITION_NED":
                log.info("[stabilise] LOCAL_POSITION_NED  N=%.2f  E=%.2f  D=%.2f",
                         msg.x, msg.y, msg.z)
                setup_samples.append({"t": time.monotonic(), "type": "LOCAL_POSITION_NED",
                                       "N": msg.x, "E": msg.y, "D": msg.z,
                                       "vN": msg.vx, "vE": msg.vy, "vD": msg.vz})
                if not ekf_pos:
                    log.info("[stabilise] EKF GPS position fused (LOCAL_POSITION_NED).")
                    ekf_pos = True
            elif mt == "EKF_STATUS_REPORT":
                flags      = msg.flags
                last_flags = flags
                log.info("[stabilise] EKF_STATUS  flags=0x%04x", flags)
                if flags & _EKF_POS_FLAG:
                    log.info("[stabilise] horiz_pos_abs set (flags=0x%04x) — GPS position fused.",
                             flags)
                if (flags & _EKF_ATT_FLAG) and not ekf_att_ready:
                    ekf_att_ready = True
                    log.info("[stabilise] EKF3 attitude confidence (flags=0x%04x). "
                             "Proceeding to arm.", flags)
        gcs.send_rc_override({8: 1000})   # keep interlock LOW until arm
        if ekf_att_ready:
            break
    if not ekf_att_ready:
        log.warning("[stabilise] EKF3 attitude confidence not seen within 20 s; "
                    "proceeding with flags=0x%04x", last_flags)

    # ── 5. Arm ────────────────────────────────────────────────────────────────
    # Arm with CH8=1000 (motor interlock LOW).  In H_RSC_MODE=1 passthrough,
    # motor_interlock_switch = (pilot_rotor_speed > 0.01): CH8=1000 → false →
    # "Motor Interlock Enabled" check passes → arm command accepted.
    # After HEARTBEAT confirms armed, raise CH8=2000 so the rotor interlock
    # engages and the RSC output matches the passthrough value.
    log.info("[setup 5/6] Arming with CH8=1000 (motor interlock LOW) ...")
    gcs.send_rc_override({8: 1000})
    time.sleep(0.3)
    gcs.send_rc_override({8: 1000})
    try:
        gcs.arm(timeout=_ARM_TIMEOUT, force=True, rc_override={8: 1000})
        log.info("[setup 5/6] Armed — raising motor interlock (CH8=2000) ...")
        gcs.send_rc_override({8: 2000})
        time.sleep(0.3)
        gcs.send_rc_override({8: 2000})
    except Exception as exc:
        all_statustext += drain_statustext(gcs, log)
        dump_startup_diagnostics(ctx)
        raise RuntimeError(f"Arm failed: {exc}") from exc
    all_statustext += drain_statustext(gcs, log)
    _procs_alive()

    # ── 6. ACRO mode ──────────────────────────────────────────────────────────
    log.info("[setup 6/6] Setting ACRO mode (timeout=%.0fs) ...", _MODE_TIMEOUT)
    try:
        gcs.set_mode(ACRO, timeout=_MODE_TIMEOUT, rc_override={8: 2000})
        log.info("[setup 6/6] ACRO mode confirmed.")
    except Exception as exc:
        all_statustext += drain_statustext(gcs, log)
        dump_startup_diagnostics(ctx)
        raise RuntimeError(f"Mode set failed: {exc}") from exc
    all_statustext += drain_statustext(gcs, log)
    _procs_alive()

    t0 = time.monotonic()
    ctx.flight_events["Setup complete"] = 0.0
    if t_ekf is not None:
        ctx.flight_events["EKF lock"] = t_ekf - t0
    log.info("acro_armed setup complete — vehicle is armed in ACRO mode.")


# ---------------------------------------------------------------------------
# Diagnostic analysis
# ---------------------------------------------------------------------------

def analyze_startup_logs(ctx: StackContext) -> dict:
    """
    Parse mediator and GCS logs to produce a structured startup diagnostic.

    Returns a dict with keys:
      mediator_errors   : list of ERROR/CRITICAL lines from mediator log
      mediator_warnings : list of WARNING lines
      mediator_tail     : last 30 lines of mediator log
      statustext        : all STATUSTEXT messages seen during setup
      ekf_samples       : EKF_STATUS samples from setup_samples
      attitude_samples  : ATTITUDE samples from setup_samples
      position_samples  : LOCAL_POSITION_NED samples from setup_samples
      ekf_aligned       : True if tilt-alignment STATUSTEXT was seen
      arm_attempted     : True if arm command was issued
      acro_confirmed    : True if ACRO mode STATUSTEXT or heartbeat was seen
      known_issues      : list of str describing detected known failure patterns
    """
    result: dict = {
        "mediator_errors":   [],
        "mediator_warnings": [],
        "mediator_tail":     [],
        "statustext":        list(ctx.all_statustext),
        "ekf_samples":       [],
        "attitude_samples":  [],
        "position_samples":  [],
        "ekf_aligned":       False,
        "arm_attempted":     False,
        "acro_confirmed":    False,
        "known_issues":      [],
    }

    # ── Mediator log ──────────────────────────────────────────────────────────
    if ctx.mediator_log.exists():
        lines = ctx.mediator_log.read_text(encoding="utf-8", errors="replace").splitlines()
        result["mediator_tail"] = lines[-30:]
        for line in lines:
            if " ERROR " in line or " CRITICAL " in line:
                result["mediator_errors"].append(line)
            elif " WARNING " in line:
                result["mediator_warnings"].append(line)

    # ── Setup samples ─────────────────────────────────────────────────────────
    for s in ctx.setup_samples:
        t = s.get("type", "")
        if t == "EKF_STATUS":
            result["ekf_samples"].append(s)
        elif t == "ATTITUDE":
            result["attitude_samples"].append(s)
        elif t == "LOCAL_POSITION_NED":
            result["position_samples"].append(s)

    # ── High-level flags ──────────────────────────────────────────────────────
    all_text = " ".join(ctx.all_statustext).lower()
    result["ekf_aligned"]    = "tilt alignment" in all_text
    result["arm_attempted"]  = "setup complete" in " ".join(str(v) for v in ctx.flight_events)
    result["acro_confirmed"] = any("acro" in t.lower() for t in ctx.all_statustext)

    # ── Known-issue pattern matching ──────────────────────────────────────────
    issues = result["known_issues"]

    if any("motor interlock" in t.lower() for t in ctx.all_statustext):
        issues.append(
            "PreArm: Motor Interlock Enabled — interlock was HIGH before arm. "
            "Fix: send CH8=1000 before arming, raise to 2000 after."
        )
    if any("runup" in t.lower() for t in ctx.all_statustext):
        issues.append(
            "PreArm: H_RUNUP_TIME too small — set H_RUNUP_TIME=0 to skip this check."
        )
    if any("gps glitch" in t.lower() for t in ctx.all_statustext):
        issues.append(
            "GPS Glitch — EKF sees position inconsistent with IMU. "
            "GPS/compass/IMU must all agree — check that "
            "accel_body and gyro_body are correctly expressed in the reported body frame."
        )
    if any("ekf variance" in t.lower() for t in ctx.all_statustext):
        issues.append(
            "EKF variance over threshold — EKF is uncertain. "
            "Common causes: IMU accel not aligned with reported attitude, "
            "or GPS position jumping while IMU says stationary."
        )
    if not result["ekf_aligned"] and not result["attitude_samples"]:
        issues.append(
            "EKF never produced ATTITUDE — mediator may not be sending IMU data, "
            "or SITL JSON backend is not receiving packets. "
            "Check mediator log for UDP send errors."
        )
    if result["mediator_errors"]:
        issues.append(
            f"{len(result['mediator_errors'])} ERROR/CRITICAL lines in mediator log."
        )

    return result


def dump_startup_diagnostics(ctx: StackContext) -> None:
    """
    Print a comprehensive human-readable diagnostic block to stdout.

    Call this on test failure so Claude or a human can diagnose the issue
    directly from the test output.
    """
    diag = analyze_startup_logs(ctx)
    sep  = "─" * 70

    print(f"\n{sep}")
    print("STARTUP DIAGNOSTICS")
    print(sep)

    # Known issues first — most actionable
    if diag["known_issues"]:
        print("\n▶ KNOWN ISSUES DETECTED:")
        for i, issue in enumerate(diag["known_issues"], 1):
            print(f"  {i}. {issue}")
    else:
        print("\n▶ No known issue patterns detected.")

    # EKF / attitude summary
    print(f"\n▶ EKF tilt aligned : {diag['ekf_aligned']}")
    print(f"  ATTITUDE samples : {len(diag['attitude_samples'])}")
    if diag["attitude_samples"]:
        first = diag["attitude_samples"][0]
        last  = diag["attitude_samples"][-1]
        print(f"  First ATTITUDE  rpy=({first['roll']:.2f}°, {first['pitch']:.2f}°, {first['yaw']:.2f}°)")
        print(f"  Last  ATTITUDE  rpy=({last['roll']:.2f}°,  {last['pitch']:.2f}°,  {last['yaw']:.2f}°)")
    print(f"  EKF_STATUS samples: {len(diag['ekf_samples'])}")
    if diag["ekf_samples"]:
        flags_seen = sorted({s["flags"] for s in diag["ekf_samples"]})
        print(f"  EKF flags seen : {[hex(f) for f in flags_seen]}")

    # Position
    print(f"\n▶ LOCAL_POSITION_NED samples: {len(diag['position_samples'])}")
    if diag["position_samples"]:
        p = diag["position_samples"][0]
        print(f"  First pos NED=({p['N']:.2f}, {p['E']:.2f}, {p['D']:.2f})")

    # STATUSTEXT
    print(f"\n▶ STATUSTEXT messages ({len(diag['statustext'])} total):")
    for t in diag["statustext"]:
        print(f"  • {t}")

    # Mediator errors/warnings
    print(f"\n▶ Mediator ERROR/CRITICAL ({len(diag['mediator_errors'])}):")
    for line in diag["mediator_errors"][:10]:
        print(f"  {line}")
    print(f"\n▶ Mediator tail (last 30 lines):")
    for line in diag["mediator_tail"]:
        print(f"  {line}")

    # Flight events
    print(f"\n▶ Flight events: {ctx.flight_events}")
    print(sep)


# ---------------------------------------------------------------------------
# Shared helpers (importable by tests)
# ---------------------------------------------------------------------------

def wait_for_acro_stability(gcs, log, timeout: float = 5.0) -> bool:
    """
    Drain MAVLink until a finite ATTITUDE message arrives or timeout.
    Returns True if a clean ATTITUDE was received.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = gcs._mav.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        r = math.degrees(msg.roll)
        p = math.degrees(msg.pitch)
        y = math.degrees(msg.yaw)
        if all(math.isfinite(v) for v in (r, p, y)):
            log.info("ACRO stable: rpy=(%.2f°, %.2f°, %.2f°)", r, p, y)
            return True
    log.warning("wait_for_acro_stability: no clean ATTITUDE within %.0fs", timeout)
    return False


def drain_statustext(gcs, log) -> list[str]:
    """Drain all buffered STATUSTEXT messages; return them as a list."""
    texts = []
    while True:
        msg = gcs._mav.recv_match(type="STATUSTEXT", blocking=True, timeout=0.05)
        if msg is None:
            break
        text = msg.text.rstrip("\x00").strip()
        log.warning("STATUSTEXT [sev=%s] %s", getattr(msg, "severity", "?"), text)
        texts.append(text)
    return texts


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _wait_params_ready(gcs, log, timeout: float = 15.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        gcs._mav.mav.param_request_read_send(
            gcs._target_system, gcs._target_component, b"SYSID_THISMAV", -1,
        )
        msg = gcs._mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is not None:
            log.info("Param subsystem ready (%s = %g)",
                     msg.param_id.rstrip("\x00"), msg.param_value)
            return
        log.debug("Waiting for param subsystem ...")
    raise TimeoutError(f"Param subsystem not ready after {timeout:.0f}s")


# ---------------------------------------------------------------------------
# Counter-torque motor stack fixtures
# ---------------------------------------------------------------------------
# These fixtures test the GB4008 anti-rotation motor (yaw stabilisation).
# They use mediator_torque.py instead of mediator.py — stationary hub model
# with bearing drag + motor physics — and a different EKF alignment sequence
# (short compass-only alignment instead of the long kinematic ramp).

_TORQUE_STARTUP_HOLD_S: float = 10.0   # mediator yaw-spin duration for EKF bias alignment

_BASE_TORQUE_PARAMS: list[tuple[str, float]] = [
    # Safety / arming
    ("ARMING_SKIPCHK",    0xFFFF),
    ("FS_EKF_ACTION",     0),
    ("FS_THR_ENABLE",     0),
    # EKF -- compass yaw (suitable for stationary test)
    ("COMPASS_USE",       1),
    ("COMPASS_ENABLE",    1),
    ("EK3_SRC1_YAW",      1),
    ("EK3_MAG_CAL",       0),
    ("EK3_GPS_CHECK",     0),
    # RSC -- CH8 passthrough: instant runup_complete when CH8=2000
    ("H_RSC_MODE",        1),
    ("H_RSC_RUNUP_TIME",  1),
    # Tail type -- servo (Type 0): neutral at 1500 us (bias mapping works correctly)
    ("H_TAIL_TYPE",       0),
    ("H_COL2YAW",         0.0),
    # Yaw PID -- pure-P; motor back-EMF provides inherent speed regulation
    ("ATC_RAT_YAW_P",     0.001),
    ("ATC_RAT_YAW_I",     0.0),
    ("ATC_RAT_YAW_D",     0.0),
    ("ATC_RAT_YAW_IMAX",  0.0),
    # Disable roll/pitch IMAX -- prevent swashplate wind-up on neutral sticks
    ("ATC_RAT_RLL_IMAX",  0.0),
    ("ATC_RAT_PIT_IMAX",  0.0),
]

# Applied after _BASE_TORQUE_PARAMS for the Lua feedforward fixture.
_LUA_TORQUE_EXTRA_PARAMS: list[tuple[str, float]] = [
    ("ATC_RAT_YAW_P",    0.0),   # override: Lua is the sole feedforward provider
    ("SCR_ENABLE",       1),
    ("RPM1_TYPE",        10),    # SITL: read rpm from JSON sensor packet
    ("RPM1_MIN",         0),
    ("SERVO9_FUNCTION",  94),    # Script 1 -- Lua writes exclusively to Ch9
    ("SCR_USER7",        2),     # RAWES_MODE = 2 (yaw trim only)
]


@dataclasses.dataclass
class TorqueStackContext:
    """Everything a torque stack test needs after setup."""
    gcs:           object
    mediator_proc: subprocess.Popen
    sitl_proc:     subprocess.Popen
    mediator_log:  Path
    sitl_log:      Path
    gcs_log:       Path
    omega_rotor:   float
    log:           logging.Logger


@contextlib.contextmanager
def _torque_stack(
    tmp_path: Path,
    *,
    omega_rotor: float,
    profile: str = "constant",
    lua_mode: bool = False,
    tail_channel: int = 3,
    extra_params=(),
    log_name: str = "torque_armed",
    test_name: str = "",
    eeprom_wipe: bool = False,
    install_scripts: tuple = (),
):
    """
    Full torque-test stack lifecycle: pre-checks -> launch -> arm -> yield -> teardown.

    Logs written to simulation/logs/{test_name}/ (per-test directory,
    matching the flight stack convention) and also to top-level logs/ with
    _last suffixes for quick access.

    Parameters
    ----------
    omega_rotor     : rotor hub angular velocity [rad/s]
    profile         : mediator_torque.py --profile value
    lua_mode        : if True, pass --lua-mode to mediator (linear mapping, no trim)
    tail_channel    : ArduPilot tail servo channel read by mediator
    extra_params    : additional (name, value) tuples applied after _BASE_TORQUE_PARAMS
    log_name        : logger name shown in log output
    test_name       : pytest test node name; used for per-test log directory
    eeprom_wipe     : if True, delete eeprom.bin before SITL launch
    install_scripts : tuple of Lua script names to install from simulation/scripts/
    """
    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run counter-torque stack tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} to locate sim_vehicle.py")

    pytest.importorskip("pymavlink")
    assert_stack_ports_free()

    repo_root = Path(__file__).resolve().parents[3]
    sim_dir   = repo_root / "simulation"

    # Per-test log directory (mirrors flight stack convention)
    test_log_dir = make_test_log_dir(sim_dir, test_name) if test_name else sim_dir / "logs"

    mediator_log = tmp_path / "mediator.log"
    sitl_log     = tmp_path / "sitl.log"
    gcs_log      = tmp_path / "gcs.log"

    _configure_logging(gcs_log)
    log = logging.getLogger(log_name)
    log.info(
        "launching  profile=%s  omega_rotor=%.1f rad/s (%.0f RPM)",
        profile, omega_rotor, omega_rotor * 60.0 / (2.0 * math.pi),
    )

    # Pre-launch: install Lua scripts before SITL starts
    if install_scripts:
        _install_lua_scripts(*install_scripts)

    mediator_proc = _launch_mediator_torque(
        _TORQUE_DIR, repo_root, mediator_log, omega_rotor,
        profile=profile, tail_channel=tail_channel, lua_mode=lua_mode,
        startup_hold_s=_TORQUE_STARTUP_HOLD_S,
    )
    sitl_proc = _launch_sitl(sim_vehicle, sitl_log, wipe_eeprom=eeprom_wipe)

    def _assert_alive() -> None:
        for name, proc, lp in [
            ("mediator_torque", mediator_proc, mediator_log),
            ("SITL",            sitl_proc,     sitl_log),
        ]:
            if proc.poll() is not None:
                txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{txt[-3000:]}")

    from gcs import RawesGCS, ACRO as _ACRO
    gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
    ctx = TorqueStackContext(
        gcs=gcs, mediator_proc=mediator_proc, sitl_proc=sitl_proc,
        mediator_log=mediator_log, sitl_log=sitl_log, gcs_log=gcs_log,
        omega_rotor=omega_rotor, log=log,
    )

    try:
        log.info("Connecting GCS ...")
        gcs.connect(timeout=30.0)
        gcs.start_heartbeat(rate_hz=1.0)
        _assert_alive()
        log.info("GCS connected")

        from pymavlink import mavutil as _mavu
        gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
        gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_EXTRA3, 2)
        gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)

        log.info("Waiting for param subsystem ...")
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

        log.info("Setting parameters ...")
        for pname, pvalue in list(_BASE_TORQUE_PARAMS) + list(extra_params):
            ok = gcs.set_param(pname, pvalue, timeout=5.0)
            log.info("  %-28s = %-10g  ACK=%s", pname, pvalue, ok)
            _assert_alive()

        # Motor interlock HIGH + EKF alignment
        # Keep CH8 alive every 0.5 s (ArduPilot expires RC override after ~1 s).
        log.info("Waiting for EKF yaw alignment (CH8=2000, up to 45 s) ...")
        gcs.send_rc_override({8: 2000})
        ekf_ok  = False
        t_start = time.monotonic()
        t_rc    = time.monotonic()
        deadline = time.monotonic() + 45.0
        _MIN_WAIT = 3.0

        while time.monotonic() < deadline:
            _assert_alive()
            if time.monotonic() - t_rc >= 0.5:
                gcs.send_rc_override({8: 2000})
                t_rc = time.monotonic()
            msg = gcs._mav.recv_match(
                type=["ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.5,
            )
            if msg is None:
                continue
            now = time.monotonic()
            if msg.get_type() == "STATUSTEXT":
                text = msg.text.rstrip("\x00").strip()
                log.info("SITL: %s", text)
                if "EKF3 active" in text or "EKF3 IMU" in text:
                    gcs.request_stream(_mavu.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
                if lua_mode and "rawes yaw trim" in text.lower():
                    log.info("Lua script confirmed loaded: %s", text)
                if "yaw alignment complete" in text.lower() and now - t_start >= _MIN_WAIT:
                    ekf_ok = True
                    break
            elif msg.get_type() == "ATTITUDE":
                if (all(math.isfinite(v) for v in (msg.roll, msg.pitch, msg.yaw))
                        and now - t_start >= _MIN_WAIT):
                    log.info(
                        "EKF attitude ready  rpy=(%.1f, %.1f, %.1f) deg",
                        math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw),
                    )
                    ekf_ok = True
                    break

        if not ekf_ok:
            log.warning("EKF alignment timed out -- proceeding (SKIPCHK set)")

        gcs.arm(timeout=15.0, force=True, rc_override={8: 2000})
        log.info("Armed")
        gcs.set_mode(_ACRO, timeout=10.0, rc_override={8: 2000})
        log.info("ACRO active -- profile=%s", profile)

        yield ctx

    finally:
        log.info("Teardown: terminating processes ...")
        try:
            gcs.close()
        except Exception:
            pass
        _terminate_process(sitl_proc)
        _terminate_process(mediator_proc)
        _kill_by_port(StackConfig.SITL_GCS_PORT)

        copy_logs_to_dir(test_log_dir, {
            "mediator.log": mediator_log,
            "sitl.log":     sitl_log,
            "gcs.log":      gcs_log,
        })
        _ardupilot_log = Path("/tmp/ArduCopter.log")
        if _ardupilot_log.exists():
            import shutil as _shutil2
            _shutil2.copy2(_ardupilot_log, test_log_dir / "arducopter.log")
        log.info("Logs copied to %s", test_log_dir)


@pytest.fixture
def torque_armed(tmp_path, request):
    """Counter-torque stack fixture (constant RPM). Yields TorqueStackContext."""
    import model as _m
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        test_name=request.node.name,
    ) as ctx:
        yield ctx


@pytest.fixture
def torque_armed_profile(request, tmp_path):
    """
    Like torque_armed but accepts a profile name via request.param.

    Usage::

        @pytest.mark.parametrize("torque_armed_profile", ["slow_vary"], indirect=True)
        def test_foo(torque_armed_profile):
            ...
    """
    import model as _m
    profile = getattr(request, "param", "constant")
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        profile=profile,
        log_name=f"torque_armed[{profile}]",
        test_name=request.node.name,
    ) as ctx:
        yield ctx


@pytest.fixture
def torque_armed_lua(tmp_path, request):
    """
    Like torque_armed but with rawes.lua active in yaw mode (SCR_USER7=2).

    Key differences from torque_armed:
      - rawes.lua installed to /ardupilot/scripts/ before boot
      - EEPROM wiped so copter-heli.parm defaults apply cleanly
      - SCR_ENABLE=1, SCR_USER7=2, RPM1_TYPE=10, SERVO9_FUNCTION=94
      - mediator_torque.py --lua-mode --tail-channel 8 (reads Ch9, linear mapping)
      - ATC_RAT_YAW_P=0 (Lua is sole feedforward provider, no ArduPilot yaw PID)
    """
    import model as _m
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        lua_mode=True,
        tail_channel=8,
        extra_params=_LUA_TORQUE_EXTRA_PARAMS,
        eeprom_wipe=True,
        install_scripts=("rawes.lua",),
        log_name="torque_armed_lua",
        test_name=request.node.name,
    ) as ctx:
        yield ctx
