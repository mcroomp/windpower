"""
stack_infra.py — infrastructure for RAWES stack integration tests.

Contains all non-pytest code: imports, constants, classes, context managers,
and helper functions.  Pytest fixtures and hooks live in conftest.py.

Exported names (imported by conftest.py via ``from stack_infra import *``):
    StackConfig, SitlContext, StackContext
    _sitl_stack, _acro_stack, _torque_stack
    _BASE_ACRO_PARAMS, _BASE_TORQUE_BOOT_PARAMS
    _LUA_TORQUE_EXTRA_PARAMS
    assert_stack_ports_free, dump_startup_diagnostics
    analyze_startup_logs, wait_for_acro_stability, drain_statustext
    observe, assert_procs_alive, assert_no_mediator_criticals, get_arducopter_crash_info
    _run_acro_setup, _wait_params_ready, _install_lua_scripts
    SITL_GCS_PORT, SITL_JSON_PORT
    _STARTING_STATE, _RAWES_DEFAULTS_PARM
    _TORQUE_STARTUP_HOLD_S
    _STARTUP_TIMEOUT, _ARM_TIMEOUT, _MODE_TIMEOUT, _STARTUP_DAMP_S
    _static_stack
"""
import contextlib
import dataclasses
import json
import logging
import math
import os
import re
import shutil
import socket as _socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

_SIM_DIR    = Path(__file__).resolve().parents[2]
_SITL_DIR   = Path(__file__).resolve().parent
_TORQUE_DIR = _SIM_DIR   # mediator_torque.py now lives in simulation/

sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

import numpy as _np
import rotor_definition as _rd

from stack_utils import (
    ARDUPILOT_ENV,
    STACK_ENV_FLAG,
    SIM_VEHICLE_ENV,
    ParamSetup,
    _launch_mediator,
    _launch_mediator_torque,
    _launch_mediator_static,
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

from pymavlink import mavutil as _mavutil
from gcs import ACRO, STABILIZE, RawesGCS
from mediator_events import MediatorEventLog
from controller import make_hold_controller

_STARTING_STATE       = _SIM_DIR / "steady_state_starting.json"
_RAWES_DEFAULTS_PARM  = _SITL_DIR / "rawes_sitl_defaults.parm"

# ── Flight fixture boot params ────────────────────────────────────────────────
# All ACRO/flight fixtures boot SITL with this complete parameter set.
# Combined with always-wipe EEPROM in _launch_sitl, this is the single source
# of truth — no EEPROM contamination from sequential tests is possible.
#
# rawes_sitl_defaults.parm provides EKF/compass/collective params.
# _BASE_ACRO_PARAMS adds the MAVLink-side ACRO mode params.
# Per-fixture extras (e.g. SCR_USER6, COL_CRUISE_FLIGHT_RAD) are merged on top.
#
# EKF params (EK3_*, COMPASS_*) are ONLY in the boot file — never set via
# MAVLink — because setting them post-boot triggers EKF3 yaw-state reset.
_BASE_ACRO_PARAMS = ParamSetup({
    "SCR_ENABLE":       1,      # Lua scripting — always enabled; boot file avoids
                                # the prime_eeprom bootstrap that was needed before
    "INITIAL_MODE":     1,      # boot directly into ACRO (mode 1)
    "FS_THR_ENABLE":    0,      # no RC throttle failsafe
    "FS_GCS_ENABLE":    0,      # no GCS heartbeat failsafe
    "FS_EKF_ACTION":    0,      # disable EKF failsafe
    "H_RSC_MODE":       1,      # CH8 passthrough — instant runup_complete
    "ACRO_TRAINER":     0,      # disable auto-leveling (physical tilt ~65 deg)
    "ATC_RAT_RLL_IMAX": 0.0,   # zero I-term limits — orbital rate accumulates
    "ATC_RAT_PIT_IMAX": 0.0,
    "ATC_RAT_YAW_IMAX": 0.0,
    "H_SW_TYPE":        3,      # H3_120 swashplate
    "H_SW_H3_PHANG":    0,      # no phase correction (RAWES geometry)
})


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
    CONNECT_TIMEOUT       : float = 90.0
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
    LOCK_ORIENTATION      : bool  = False  # do not force body_z to tether direction each frame
    # Permanent angular damping after startup ramp ends.
    # 50 N·m·s/rad (mediator default) — physical bearing + air drag estimate.
    # ArduPilot's 400 Hz ACRO rate PIDs provide the real angular control;
    # k_ang=50 is a background stabiliser, not a workaround.
    BASE_K_ANG            : float = 50.0
    # ArduPilot servo outputs drive the physics (full stack).
    # internal_controller=True is reserved for the Lua fixture where Lua only
    # controls cyclic and the mediator provides collective.
    INTERNAL_CONTROLLER   : bool  = False
    BASE_K_ANG_INTERNAL   : float = 50.0   # same physical value; used when internal_controller=True

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
            _kill_by_port(port, proto)

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

    Used by both flight tests (full mediator + physics + ArduPilot) and torque
    tests (mediator_torque + hub yaw ODE + ArduPilot).  Flight-only fields
    default to None/empty so torque fixtures can construct without them.

    Required for all tests
    ----------------------
    gcs            : connected, heartbeating RawesGCS (armed, in ACRO mode)
    mediator_proc  : running mediator subprocess
    sitl_proc      : running SITL subprocess
    mediator_log   : path to mediator stdout/stderr log
    sitl_log       : path to SITL stdout/stderr log
    gcs_log        : path to GCS/test structured log
    events_log     : MediatorEventLog wrapping the events.jsonl path
    log            : logger scoped to the running fixture/test

    Flight tests only (default None/empty for torque tests)
    --------------------------------------------------------
    telemetry_log  : path to mediator telemetry CSV
    mavlink_log    : path to MAVLink JSON log
    initial_state  : dict from steady_state_starting.json (vel overridden to 0)
    home_alt_m     : hub altitude above anchor [m] at launch
    flight_events  : timing checkpoints; setup populates, tests add their own
    all_statustext : all STATUSTEXT messages seen during setup
    setup_samples  : list of dicts — EKF/ATTITUDE samples captured during setup
    sim_dir        : simulation/ directory (for writing outputs)
    controller     : PhysicalHoldController instance
    internal_controller : True = mediator runs truth-state controller at 400 Hz

    Torque tests only (default 0.0 for flight tests)
    -------------------------------------------------
    omega_rotor    : rotor hub angular velocity [rad/s] used by the profile
    """
    # ── required for all stack tests ──────────────────────────────────────────
    gcs:           RawesGCS
    mediator_proc: object
    sitl_proc:     object
    mediator_log:  Path
    sitl_log:      Path
    gcs_log:       Path
    events_log:    MediatorEventLog
    log:           logging.Logger
    # ── flight tests (default empty for torque) ───────────────────────────────
    telemetry_log:       Path | None  = None
    mavlink_log:         Path | None  = None
    initial_state:       dict | None  = None
    home_alt_m:          float        = 0.0
    flight_events:       dict         = dataclasses.field(default_factory=dict)
    all_statustext:      list         = dataclasses.field(default_factory=list)
    setup_samples:       list         = dataclasses.field(default_factory=list)
    sim_dir:             Path | None  = None
    controller:          object       = None
    internal_controller: bool         = False
    # ── torque tests (default 0.0 for flight) ─────────────────────────────────
    omega_rotor:         float        = 0.0

    def wait_drain(
        self,
        *,
        until=None,
        timeout: float = 3.0,
        drain_s: float = 0.0,
        check_procs: bool = False,
        label: str = "drain",
    ) -> bool:
        """Drain STATUSTEXT into self.all_statustext, optionally waiting for a condition.

        Parameters
        ----------
        until       : zero-argument callable -> bool.  If given, poll until it
                      returns True (or timeout expires).  If None, drain for the
                      full timeout duration.
        timeout     : seconds to wait for ``until`` (or total drain time).
        drain_s     : extra seconds to keep draining after ``until`` fires —
                      useful to flush messages buffered in the MAVLink socket.
        check_procs : if True, call pytest.fail() immediately if mediator or
                      SITL process exits (crash detection).
        label       : prefix used in log lines: "STATUSTEXT [<label>]: ...".

        Returns True if ``until`` fired (or ``until`` is None), False on timeout.

        Examples
        --------
        # Timed flush only (no condition):
            ctx.wait_drain(timeout=1.0, label="post-param")

        # Wait for a log marker, then flush 3 s of buffered messages:
            ctx.wait_drain(
                until=lambda _: "TRANSITION kinematic" in med_log.read_text(),
                timeout=90.0, drain_s=3.0, check_procs=True, label="kin-wait",
            )
        """
        last_text: list[str | None] = [None]

        def _recv_one(recv_timeout: float) -> None:
            msg = self.gcs._recv(
                type=["STATUSTEXT", "ATTITUDE", "LOCAL_POSITION_NED",
                      "EKF_STATUS_REPORT", "SERVO_OUTPUT_RAW"],
                blocking=True, timeout=recv_timeout)
            if msg is not None and msg.get_type() == "STATUSTEXT":
                text = msg.text.rstrip("\x00").strip()
                self.all_statustext.append(text)
                self.log.info("STATUSTEXT [%s]: %s", label, text)
                last_text[0] = text
            else:
                last_text[0] = None

        def _check_liveness() -> None:
            if not check_procs:
                return
            for name, proc, lp in [
                ("mediator", self.mediator_proc, self.mediator_log),
                ("SITL",     self.sitl_proc,     self.sitl_log),
            ]:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    pytest.fail(
                        f"{name} exited during {label} "
                        f"(rc={proc.returncode}):\n{txt[-3000:]}"
                    )

        deadline = self.gcs.sim_now() + timeout
        if until is None:
            while self.gcs.sim_now() < deadline:
                _check_liveness()
                _recv_one(min(0.1, deadline - self.gcs.sim_now()))
            return True

        while self.gcs.sim_now() < deadline:
            _check_liveness()
            _recv_one(0.5)
            try:
                if until(last_text[0]):
                    t_end = self.gcs.sim_now() + drain_s
                    while self.gcs.sim_now() < t_end:
                        _check_liveness()
                        _recv_one(min(0.1, t_end - self.gcs.sim_now()))
                    return True
            except OSError:
                pass
        return False

    def wait_kinematic_done(self, timeout: float = 90.0, drain_s: float = 3.0) -> bool:
        """Block until the kinematic->free-flight transition, draining STATUSTEXT throughout.

        Polls events.jsonl for the "kinematic_exit" event written by the mediator.
        Returns True if seen, False on timeout.  Raises pytest.fail if mediator
        or SITL exits during the wait.
        """
        ev = self.events_log

        def _transition_seen(_text: str | None) -> bool:
            return ev.has_event("kinematic_exit")

        return self.wait_drain(
            until=_transition_seen,
            timeout=timeout,
            drain_s=drain_s,
            check_procs=True,
            label="kin-wait",
        )


# ---------------------------------------------------------------------------
# _sitl_stack — minimal SITL lifecycle (base for _acro_stack and bare tests)
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class SitlContext:
    """
    What _sitl_stack yields: SITL process + log paths + derived config.

    Use _static_stack (wraps this) when you need a static-sensor subprocess
    instead of full physics — no threading in test files.

    Use this directly when you need SITL + your own sensor worker without
    the full mediator + arm sequence (e.g. GPS fusion layer tests).
    """
    sitl_proc:     subprocess.Popen  # type: ignore[type-arg]
    sitl_log:      Path
    gcs_log:       Path
    mavlink_log:   Path              # pass as mavlog_path= to RawesGCS to enable logging
    telemetry_log: Path              # write telemetry CSV here; copied to test_log_dir on teardown
    sim_dir:       Path
    repo_root:     Path
    log:           logging.Logger
    test_log_dir:  Path
    boot_setup:    object            # ParamSetup
    # Set by _static_stack; None when using _sitl_stack directly
    mediator_proc: object = None     # subprocess.Popen | None
    mediator_log:  "Path | None" = None


@contextlib.contextmanager
def _sitl_stack(
    tmp_path, *,
    test_name:          str = "",
    extra_boot_params:  "dict[str, float] | None" = None,
    base_params:        "ParamSetup | None" = None,
):
    """
    Minimal SITL lifecycle: pre-checks → boot params → launch SITL → yield → teardown.

    Handles everything that doesn't require the mediator or physics:
      - environment / port pre-checks
      - boot param file (rawes_defaults + BASE_ACRO + servo_speed + extra_boot_params
        when base_params is None; or base_params + extra_boot_params when provided)
      - SITL process launch
      - logging setup (writes to gcs_log)
      - teardown: kill SITL + stray procs, kill ports, copy sitl/gcs/arducopter logs

    Yields SitlContext.  Caller is responsible for:
      - starting any sensor / mediator worker that feeds SITL
      - connecting and driving RawesGCS

    Parameters
    ----------
    base_params : ParamSetup | None
        When provided, replaces the default rawes_defaults + BASE_ACRO + SIM_SERVO_SPEED
        chain as the boot param set.  Use this for non-ACRO stacks (e.g. torque tests)
        that need a completely different parameter base.  extra_boot_params are merged
        on top regardless.
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

    # ── Paths ──────────────────────────────────────────────────────────────────
    sitl_log      = tmp_path / "sitl.log"
    gcs_log       = tmp_path / "gcs.log"
    mavlink_log   = tmp_path / "mavlink.jsonl"
    telemetry_log = tmp_path / "telemetry.csv"

    _configure_logging(gcs_log)
    log = logging.getLogger(test_name or "sitl")
    logging.getLogger("gcs").setLevel(logging.DEBUG)

    # ── Boot param file ────────────────────────────────────────────────────────
    if base_params is not None:
        # Caller supplied a complete param base (e.g. torque stack) — use as-is.
        _boot_setup = base_params.merge(ParamSetup(extra_boot_params or {}))
    else:
        _servo_speed = _rd.default().sim_servo_speed
        _boot_setup = (
            ParamSetup.from_parm_file(_RAWES_DEFAULTS_PARM)
            .merge(_BASE_ACRO_PARAMS)
            .merge(ParamSetup({"SIM_SERVO_SPEED": _servo_speed}))
            .merge(ParamSetup(extra_boot_params or {}))
        )
    boot_parm_file = tmp_path / "boot_params.parm"
    _boot_setup.write_parm_file(boot_parm_file)
    log.info("Boot params: %d entries", len(_boot_setup))

    # ── Per-test log directory ─────────────────────────────────────────────────
    test_log_dir = make_test_log_dir(sim_dir, test_name) if test_name else sim_dir / "logs"

    # ── Launch SITL ───────────────────────────────────────────────────────────
    sitl_proc = _launch_sitl(sim_vehicle, sitl_log,
                             add_param_file=boot_parm_file)

    ctx = SitlContext(
        sitl_proc     = sitl_proc,
        sitl_log      = sitl_log,
        gcs_log       = gcs_log,
        mavlink_log   = mavlink_log,
        telemetry_log = telemetry_log,
        sim_dir       = sim_dir,
        repo_root     = repo_root,
        log           = log,
        test_log_dir  = test_log_dir,
        boot_setup    = _boot_setup,
    )

    try:
        yield ctx
    finally:
        _terminate_process(sitl_proc)
        _kill_by_port(StackConfig.SITL_GCS_PORT, "tcp")
        _kill_by_port(StackConfig.SITL_JSON_PORT, "udp")
        import subprocess as _subprocess
        _subprocess.run(
            ["bash", "-c", "pgrep arducopter-heli | xargs -r kill -9"],
            capture_output=True,
        )
        _subprocess.run(
            ["bash", "-c", "pgrep -f /sim_vehicle.py | xargs -r kill -9"],
            capture_output=True,
        )
        _copy_map: dict[str, Path] = {"sitl.log": sitl_log, "gcs.log": gcs_log}
        if mavlink_log.exists():
            _copy_map["mavlink.jsonl"] = mavlink_log
        if telemetry_log.exists():
            _copy_map["telemetry.csv"] = telemetry_log
        copy_logs_to_dir(test_log_dir, _copy_map)
        _ardupilot_log = Path("/tmp/ArduCopter.log")
        if _ardupilot_log.exists():
            shutil.copy2(_ardupilot_log, test_log_dir / "arducopter.log")
        # DataFlash binary log — ArduPilot writes to /ardupilot/logs/ (cwd at launch)
        _df_log = Path("/ardupilot/logs/00000001.BIN")
        if _df_log.exists():
            shutil.copy2(_df_log, test_log_dir / "dataflash.BIN")


# ---------------------------------------------------------------------------
# _static_stack — SITL + static-sensor mediator subprocess (no threading)
# ---------------------------------------------------------------------------

@contextlib.contextmanager
def _static_stack(
    tmp_path,
    *,
    pos:        "_np.ndarray",
    vel:        "_np.ndarray",
    rpy:        "_np.ndarray",
    accel_body: "_np.ndarray",
    gyro:       "_np.ndarray",
    test_name:          str = "",
    extra_boot_params:  "dict[str, float] | None" = None,
):
    """
    SITL + static-sensor mediator: pre-checks -> boot params -> SITL ->
    mediator_static.py subprocess -> yield -> teardown.

    Drop-in replacement for manually managing a background sensor thread.
    The mediator_static.py subprocess handles the lockstep loop
    (recv_servos -> send_state with fixed values) exactly like the full
    mediator but with no physics.

    Built on top of _sitl_stack.  Populates ctx.mediator_proc and
    ctx.mediator_log so tests can check liveness with::

        if ctx.mediator_proc.poll() is not None:
            pytest.fail(...)

    Caller is responsible for connecting and driving RawesGCS.

    Parameters
    ----------
    pos        : NED position [m]
    vel        : NED velocity [m/s]
    rpy        : roll/pitch/yaw [rad]
    accel_body : body-frame specific force [m/s^2]  (include gravity: [0,0,-g] for level)
    gyro       : body-frame angular rate [rad/s]
    """
    with _sitl_stack(
        tmp_path,
        test_name         = test_name,
        extra_boot_params = extra_boot_params,
    ) as ctx:
        mediator_log    = tmp_path / "mediator.log"
        mediator_events = tmp_path / "mediator_events.jsonl"
        mediator_proc = _launch_mediator_static(
            ctx.sim_dir, ctx.repo_root, mediator_log,
            pos=pos, vel=vel, rpy=rpy, accel_body=accel_body, gyro=gyro,
            events_log=mediator_events,
        )
        ctx.mediator_proc = mediator_proc
        ctx.mediator_log  = mediator_log
        try:
            yield ctx
        finally:
            _terminate_process(mediator_proc)
            logs_to_copy = {"mediator.log": mediator_log}
            if mediator_events.exists():
                logs_to_copy["mediator_events.jsonl"] = mediator_events
            copy_logs_to_dir(ctx.test_log_dir, logs_to_copy)


# ---------------------------------------------------------------------------
# _acro_stack — full ACRO stack (mediator + arm) built on top of _sitl_stack
# ---------------------------------------------------------------------------

@contextlib.contextmanager
def _acro_stack(tmp_path, *, extra_config=None,
                arm: bool = True, with_mediator: bool = True, test_name: str = "",
                extra_boot_params: "dict[str, float] | None" = None,
                internal_controller: bool | None = None):
    """
    Core ACRO stack lifecycle: pre-checks → launch → [arm] → yield ctx → teardown.

    All fixtures call this.  Built on top of _sitl_stack which handles
    pre-checks, boot params, SITL launch, logging, and teardown.

    Differences between fixtures are handled outside this function:
      - extra_config   : pumping cycle passes trajectory config here
      - pre-launch work: Lua fixture installs scripts before calling this
      - post-arm work  : Lua fixture sets SCR_USER params inside the with-block

    Parameters
    ----------
    tmp_path       : pytest tmp_path fixture value
    extra_config   : optional dict merged into mediator config (pumping cycle)
    arm            : if False, skip _run_acro_setup — yield with procs running
                     but GCS not connected and vehicle not armed.  Callers may
                     call ctx.gcs.connect() manually (e.g. smoke tests).
    with_mediator  : if False, skip mediator launch (pure SITL connectivity tests)
    test_name      : pytest test node name; used as logger name and for persistent
                     log file names so each test run produces uniquely-named files.
    """
    # ── Initial state ──────────────────────────────────────────────────────────
    initial_state = None
    home_alt_m    = 12.530
    if _STARTING_STATE.exists():
        initial_state = json.loads(_STARTING_STATE.read_text())
        home_alt_m    = -float(initial_state["pos"][2])
        initial_state = dict(initial_state)

    # ── Hold controller ────────────────────────────────────────────────────────
    _anchor_ned = _np.array([0.0, 0.0, float(home_alt_m)])
    controller   = make_hold_controller(anchor_ned=_anchor_ned)

    # ── Lua scripts ───────────────────────────────────────────────────────────
    _install_lua_scripts("rawes.lua")

    # ── Merge controller params + test extras into boot params ─────────────────
    _extra: dict = dict(controller.extra_params)
    if extra_boot_params:
        _extra.update(extra_boot_params)

    with _sitl_stack(
        tmp_path,
        test_name         = test_name,
        extra_boot_params = _extra,
    ) as sitl_ctx:
        log = sitl_ctx.log
        log.info("Hold controller: %s", type(controller).__name__)

        # ── Extra log paths (mediator-specific) ────────────────────────────────
        mediator_log  = tmp_path / "mediator.log"
        telemetry_log = tmp_path / "telemetry.csv"
        mavlink_log   = tmp_path / "mavlink.jsonl"
        events_path   = tmp_path / "events.jsonl"

        # ── Launch mediator ────────────────────────────────────────────────────
        _run_id = int(time.time())
        log.info("RUN_ID=%d", _run_id)
        log.info("launching%s SITL ...", " mediator +" if with_mediator else "")
        _use_internal = internal_controller if internal_controller is not None else StackConfig.INTERNAL_CONTROLLER
        if with_mediator:
            _k_ang = StackConfig.BASE_K_ANG_INTERNAL if _use_internal else StackConfig.BASE_K_ANG
            mediator_proc = _launch_mediator(
                sitl_ctx.sim_dir, sitl_ctx.repo_root, mediator_log,
                telemetry_log_path   = str(telemetry_log),
                events_log_path      = str(events_path),
                initial_state        = initial_state,
                startup_damp_seconds = _STARTUP_DAMP_S,
                lock_orientation     = StackConfig.LOCK_ORIENTATION,
                run_id               = _run_id,
                base_k_ang           = _k_ang,
                internal_controller  = _use_internal,
                extra_config         = extra_config,
            )
        else:
            mediator_proc = None

        def _procs_alive():
            _checks = [("SITL", sitl_ctx.sitl_proc, sitl_ctx.sitl_log)]
            if with_mediator:
                _checks.insert(0, ("mediator", mediator_proc, mediator_log))
            for name, proc, lp in _checks:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{txt[-3000:]}")

        gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=mavlink_log)

        ctx = StackContext(
            gcs=gcs, mediator_proc=mediator_proc, sitl_proc=sitl_ctx.sitl_proc,
            mediator_log=mediator_log, sitl_log=sitl_ctx.sitl_log,
            gcs_log=sitl_ctx.gcs_log, telemetry_log=telemetry_log,
            mavlink_log=mavlink_log,
            events_log=MediatorEventLog(events_path),
            initial_state=initial_state, home_alt_m=home_alt_m,
            flight_events={}, all_statustext=[], setup_samples=[],
            log=log, sim_dir=sitl_ctx.sim_dir,
            controller=controller,
            internal_controller=_use_internal,
        )

        try:
            if arm:
                _run_acro_setup(ctx, _procs_alive, boot_setup=sitl_ctx.boot_setup)
            yield ctx
        finally:
            gcs.close()
            if mediator_proc is not None:
                _terminate_process(mediator_proc)
            # Copy mediator-specific logs on top of what _sitl_stack already copies
            _logs = {}
            if with_mediator:
                _logs["mediator.log"]  = mediator_log
                _logs["telemetry.csv"] = telemetry_log
            if mavlink_log.exists():
                _logs["mavlink.jsonl"] = mavlink_log
            if events_path.exists():
                _logs["events.jsonl"] = events_path
            if _logs:
                copy_logs_to_dir(sitl_ctx.test_log_dir, _logs)


# ---------------------------------------------------------------------------
# Lua scripts helper
# ---------------------------------------------------------------------------

_SCRIPTS_DIR = _SIM_DIR / "scripts"  # simulation/scripts/ — all Lua scripts


def _install_lua_scripts(*names: str) -> None:
    """
    Copy the named Lua scripts from simulation/scripts/ to SITL's /ardupilot/scripts/.

    Must be called before SITL starts.  In normal use, pass only "rawes.lua"
    (the unified script); SCR_USER6 selects the active mode at runtime.

    Examples::

        _install_lua_scripts("rawes.lua")
    """
    dst_dir = Path("/ardupilot/scripts")
    dst_dir.mkdir(exist_ok=True)
    for name in names:
        src = _SCRIPTS_DIR / name
        shutil.copy2(src, dst_dir / name)


# ---------------------------------------------------------------------------
# Setup sequence
# ---------------------------------------------------------------------------

def _run_acro_setup(ctx: StackContext, _procs_alive, boot_setup: "ParamSetup | None" = None) -> None:
    """
    Shared ACRO setup sequence.

    Timing contract: the mediator startup damping ramp is 30 s.  All steps
    here must complete inside that window so the hub is barely moving when
    we arm — strong damping keeps it near the initial position during GPS init.

    Steps:
        1. Connect GCS; request telemetry streams; clear motor interlock
        2. Wait for param subsystem
        3. Verify all boot params via MAVLink read-back (pytest.fail on mismatch)
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
    try:
        gcs.connect(timeout=_STARTUP_TIMEOUT)
    except TimeoutError as exc:
        raise TimeoutError(
            f"[setup 1/6] GCS connect timeout after {_STARTUP_TIMEOUT:.0f}s "
            f"(SITL may not have started yet — resource contention under -n 8?): {exc}"
        ) from exc
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

    # ── 3 / 4. EKF alignment then boot-param verify ───────────────────────────
    # EKF alignment is time-critical (must complete within kinematic startup).
    # Param verify is a sanity-check only — it runs AFTER EKF ready so it
    # doesn't delay the kinematic window.  All params are already in EEPROM
    # from the boot file; verify just reads them back (no SET calls).

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
    deadline  = gcs.sim_now() + 45.0
    while gcs.sim_now() < deadline and not ekf_ok:
        _procs_alive()
        msg = gcs._recv(
            type=["ATTITUDE", "EKF_STATUS_REPORT", "STATUSTEXT",
                  "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"],
            blocking=True, timeout=1.0,
        )
        if msg is None:
            continue
        t_now = gcs.sim_now()
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

    # ── 3. Verify boot parameters (after EKF ready — non-critical path) ───────
    # EKF alignment is done, so verifying params here doesn't delay kinematic.
    # Use a short per-param timeout (1 s): params respond instantly once SITL
    # is running; anything that doesn't respond in 1 s is a genuine problem.
    if boot_setup is not None:
        log.info("[setup 3/6] Verifying boot parameters ...")
        boot_setup.verify(gcs, log=log, read_timeout=1.0)
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
    t_stab = gcs.sim_now() + 20.0
    while gcs.sim_now() < t_stab:
        _procs_alive()
        msg = gcs._recv(
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
                setup_samples.append({"t": gcs.sim_now(), "type": "LOCAL_POSITION_NED",
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
    gcs.sim_sleep(0.3)
    gcs.send_rc_override({8: 1000})
    try:
        gcs.arm(timeout=_ARM_TIMEOUT, force=True, rc_override={8: 1000})
        ctx.flight_events["arm_t"] = gcs.sim_now()
        log.info("[setup 5/6] Armed — raising motor interlock (CH8=2000) ...")
        gcs.send_rc_override({8: 2000})
        gcs.sim_sleep(0.3)
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

    t0 = gcs.sim_now()
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
    deadline = gcs.sim_now() + timeout
    while gcs.sim_now() < deadline:
        msg = gcs._recv(type="ATTITUDE", blocking=True, timeout=1.0)
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
        msg = gcs._recv(type="STATUSTEXT", blocking=True, timeout=0.05)
        if msg is None:
            break
        text = msg.text.rstrip("\x00").strip()
        log.warning("STATUSTEXT [sev=%s] %s", getattr(msg, "severity", "?"), text)
        texts.append(text)
    return texts


def observe(
    ctx: "StackContext",
    duration_s: float,
    on_message,
    *,
    msg_types: "list[str]",
    label: str = "observation",
    recv_timeout: float = 0.2,
    keepalive: "dict | None" = None,
    keepalive_interval_s: float = 0.5,
) -> None:
    """
    Run a SITL observation loop for ``duration_s`` sim-seconds.

    On every iteration:
      1. ``assert_procs_alive(ctx, label)`` — pytest.fail if a process exited.
      2. Optional RC keepalive: if ``keepalive`` is set, ``send_rc_override``
         is called once every ``keepalive_interval_s`` sim-seconds.
      3. ``gcs._recv(type=msg_types, blocking=True, timeout=recv_timeout)``
      4. ``on_message(msg, t_rel)`` — ``msg`` may be ``None`` on recv timeout.
         Return ``True`` from the callback to exit the loop early.

    Parameters
    ----------
    ctx                  : StackContext (flight or torque)
    duration_s           : how many sim-seconds to run
    on_message           : callable(msg, t_rel) -> bool | None
    msg_types            : MAVLink message types to pass to _recv
    label                : label forwarded to assert_procs_alive / logs
    recv_timeout         : wall-clock seconds _recv waits for a message
    keepalive            : RC channel dict for send_rc_override, or None
    keepalive_interval_s : minimum sim-seconds between keepalive sends;
                           0.0 sends on every iteration
    """
    gcs      = ctx.gcs
    t_start  = gcs.sim_now()
    deadline = t_start + duration_s
    t_rc     = gcs.sim_now()

    while gcs.sim_now() < deadline:
        assert_procs_alive(ctx, label)

        if keepalive is not None and gcs.sim_now() - t_rc >= keepalive_interval_s:
            gcs.send_rc_override(keepalive)
            t_rc = gcs.sim_now()

        msg   = gcs._recv(type=msg_types, blocking=True, timeout=recv_timeout)
        t_rel = gcs.sim_now() - t_start
        if on_message(msg, t_rel):
            break


def get_arducopter_crash_info(ctx) -> str:
    """
    Extract ArduPilot crash details from arducopter.log if present.

    Returns a formatted multi-line string, or '' if no crash was found or the
    log is absent.  Returns '' when telemetry_log is None (torque tests).
    """
    try:
        log_path = getattr(ctx, "telemetry_log", None)
        if log_path is None:
            return ""
        acp = Path(log_path).parent / "arducopter.log"
        if not acp.exists():
            return ""
        _analysis = Path(__file__).resolve().parents[2] / "analysis"
        if str(_analysis) not in sys.path:
            sys.path.insert(0, str(_analysis))
        from analyse_run import parse_arducopter, RunReport  # type: ignore[import]
        rpt = RunReport()
        parse_arducopter(acp, rpt)
        if rpt.sitl_crash is None:
            return ""
        lines = ["", "--- ArduPilot crash ---", f"  {rpt.sitl_crash.error_line}"]
        for frame in rpt.sitl_crash.stack:
            lines.append(f"  {frame}")
        return "\n".join(lines)
    except Exception:
        return ""


def assert_procs_alive(ctx, label: str = "observation") -> None:
    """
    pytest.fail() immediately if either mediator or SITL process has exited.

    Appends the last 3000 chars of the relevant process log to the fail message.
    For flight-test StackContext, also appends ArduPilot crash info if present.

    Works with both flight and torque StackContext instances.

    Call once per receive loop iteration, before gcs._recv().
    """
    for name, proc, lp in [
        ("mediator", ctx.mediator_proc, ctx.mediator_log),
        ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
    ]:
        if proc.poll() is not None:
            txt   = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
            crash = get_arducopter_crash_info(ctx)
            pytest.fail(
                f"{name} exited during {label} (rc={proc.returncode}):\n{txt[-3000:]}"
                + crash
            )


def assert_no_mediator_criticals(mediator_log: Path) -> None:
    """Assert there are no CRITICAL lines in the mediator log.

    Call at the end of a test to catch any unhandled exceptions or
    misconfigured logging levels in mediator.py / mediator_torque.py.
    """
    if not mediator_log.exists():
        return
    lines    = mediator_log.read_text(encoding="utf-8", errors="replace").splitlines()
    critical = [ln for ln in lines if "CRITICAL" in ln]
    assert not critical, "CRITICAL errors in mediator:\n" + "\n".join(critical[:10])


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _wait_params_ready(gcs, log, timeout: float = 15.0) -> None:
    deadline = gcs.sim_now() + timeout
    while gcs.sim_now() < deadline:
        gcs._mav.mav.param_request_read_send(
            gcs._target_system, gcs._target_component, b"SYSID_THISMAV", -1,
        )
        msg = gcs._recv(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is not None:
            log.info("Param subsystem ready (%s = %g)",
                     msg.param_id.rstrip("\x00"), msg.param_value)
            return
        log.debug("Waiting for param subsystem ...")
    raise TimeoutError(
        f"[setup 3/6] Param subsystem (SYSID_THISMAV) not ready after {timeout:.0f}s "
        f"— SITL may be overloaded or crashed"
    )


# ---------------------------------------------------------------------------
# Counter-torque motor stack fixtures
# ---------------------------------------------------------------------------
# These fixtures test the GB4008 anti-rotation motor (yaw stabilisation).
# They use mediator_torque.py instead of mediator.py — stationary hub yaw ODE
# + motor physics — and a different EKF alignment sequence
# (short compass-only alignment instead of the long kinematic ramp).

_TORQUE_STARTUP_HOLD_S: float = 15.0   # SITL-seconds: enough for EKF + arming before DYNAMIC starts

# All torque test parameters in one boot-file set.
#
# _launch_sitl always wipes EEPROM and uses --add-param-file (which overrides EEPROM),
# so every param here is applied at first boot — no MAVLink setting phase is needed.
#
# Why PID/frame params must be at boot (not via MAVLink post-boot):
# mediator_torque physics become active at t=10 s (TORQUE_STARTUP_HOLD_S).  Any
# post-boot MAVLink write only completes at ~25 s.  The default ATC_RAT_YAW_P ≈ 0.18
# (180× larger than our 0.001 target) causes ±300 deg/s yaw oscillations in that
# window, corrupting the EKF gyro-bias estimate so badly that ATTITUDE.yawspeed reads
# ~9 deg/s even after the hub settles.
#
# Why EK3_SRC1_POSXY/VELXY=0 must be at boot:
# Hub is stationary but tilted; g·sin(θ) lateral accel projection triggers GPS Glitch,
# making EKF unhealthy → arm fails.  Writing EK3_SRC* via MAVLink post-boot triggers
# "EKF3 IMU0 forced reset" even if the value is unchanged, corrupting the gyro-bias
# estimate in the same way as the PID oscillation above.
#
# fixture boot_params are merged on top and may override individual values (e.g.
# ATC_RAT_YAW_P=0 for the Lua fixture where Lua is the sole feedforward provider).
_BASE_TORQUE_BOOT_PARAMS = ParamSetup({
    # Failsafe — disable EKF failsafe
    # (ARMING_SKIPCHK was removed in ArduPilot 4.6; force-arm bypasses checks)
    "FS_EKF_ACTION":    0,
    "FS_THR_ENABLE":    0,
    # Compass — enable and do not auto-calibrate.  Default EK3_MAG_CAL=3 (auto) causes
    # the EKF to distrust compass during calibration; when the wobble profile starts at
    # t=10 s the EKF has no yaw reference → diverges badly.  Set =0 (no calibration) so
    # compass headings are used from the first EKF loop.
    "COMPASS_USE":      1,
    "COMPASS_ENABLE":   1,
    "EK3_SRC1_YAW":    1,     # compass yaw (default, but be explicit so boot is consistent)
    "EK3_MAG_CAL":     0,     # no magnetometer calibration
    "EK3_GPS_CHECK":   0,     # no GPS pre-arm check (GPS not used in torque tests)
    # EKF source config: disable GPS position/velocity.
    "EK3_SRC1_POSXY":  0,
    "EK3_SRC1_VELXY":  0,
    # Yaw PID — P=0.015 for transient response.
    # Equilibrium: throttle_eq = omega_rotor * GEAR_RATIO / RPM_SCALE = 0.485.
    # With H_YAW_TRIM=0.02: I_integral_eq = throttle_eq + H_YAW_TRIM = 0.505.
    # slow_vary profile: omega up to 33 rad/s → I_eq = 33*1.818/105 + 0.02 = 0.592.
    # IMAX=0.7 > 0.592 so the integrator can reach zero error across all RPM profiles.
    # I=0.01 winds up to 0.5 in ~14 s at 3-4 rad/s residual error (within 40 s settle).
    "ATC_RAT_YAW_P":   0.015,
    "ATC_RAT_YAW_I":   0.01,
    "ATC_RAT_YAW_D":   0.0,
    "ATC_RAT_YAW_IMAX": 0.7,
    # Roll/pitch I-term — disable to prevent swashplate wind-up on neutral sticks
    "ATC_RAT_RLL_IMAX": 0.0,
    "ATC_RAT_PIT_IMAX": 0.0,
    # DDFP tail type: H_TAIL_TYPE=4 (DDFP CCW) matches the GB4008 hardware.
    # H_TAIL_TYPE=0 (servo tail) is NOT used — it requires a mediator-side
    # adaptive-trim hack to map 1500 µs to the equilibrium throttle, which has
    # no hardware equivalent.  DDFP + H_YAW_TRIM is the correct model.
    "H_TAIL_TYPE":     4,
    "H_COL2YAW":      0.0,
    # SERVO4 range matches GB4008 hardware: 800 µs = off, 2000 µs = full throttle.
    "SERVO4_MIN":       800,
    "SERVO4_MAX":       2000,
    "SERVO4_TRIM":      800,   # DDFP: trim = off (motor off at neutral stick)
    # H_YAW_TRIM ≈ 0: near-zero feedforward so the motor stays off during the STARTUP
    # phase (no overshoot when the rotor spins up from rest).  The integrator carries
    # the full equilibrium throttle (~0.42) once DYNAMIC begins — requires IMAX >= 0.44.
    "H_YAW_TRIM":      0.02,
    # RSC — enable CH8 passthrough (instant runup when CH8=2000)
    "H_RSC_MODE":      1,
    "H_RSC_RUNUP_TIME": 1,
})

# Extra params for the Lua feedforward fixture, merged on top of _BASE_TORQUE_BOOT_PARAMS.
# Lua controls Ch9 (SERVO9) exclusively; the ArduPilot DDFP controller on Ch4 is
# disabled by setting H_TAIL_TYPE=0 so it does not fight the Lua output.
_LUA_TORQUE_EXTRA_PARAMS = ParamSetup({
    "H_TAIL_TYPE":      0,     # disable DDFP on Ch4; Lua owns Ch9 entirely
    "ATC_RAT_YAW_P":    0.0,   # override: Lua is the sole feedforward provider
    "SCR_ENABLE":       1,
    "SERVO9_FUNCTION":  94,    # Script 1 -- Lua writes exclusively to Ch9
    "SERVO9_MIN":       800,   # PWM off = 800 us (matches rawes.lua _set_throttle_pct range)
    "SERVO9_MAX":       2000,  # PWM full = 2000 us
    "SERVO9_TRIM":      800,   # trim = off
    "SCR_USER6":        2,     # RAWES_MODE = 2 (yaw)
})

# Extra params for the ArduPilot DDFP (H_TAIL_TYPE=2) yaw PI fixture.
# ArduPilot's built-in ATC_RAT_YAW controller drives SERVO4 (Ch4) directly
# as a unidirectional motor (0% = off, 100% = full throttle).
# Motor range: 800 us = off, 2000 us = max (GB4008 66KV on REVVitRC ESC).
_DDFP_TORQUE_EXTRA_PARAMS = ParamSetup({
    # DDFP tail type: ArduPilot outputs 0-100 % motor throttle on SERVO4.
    # H_TAIL_TYPE=4 (DDFP CCW, enum value 4): hub under-speed causes CW drift
    # (positive psi_dot) → yaw error = 0 - positive = negative → PID output negative.
    # CCW mode applies _servo4_out *= -1 before thrust_to_actuator, mapping the
    # negative PID output to positive motor throttle → CCW reaction counters CW drift.
    # H_TAIL_TYPE=3 (DDFP CW, enum value 3) is wrong: no sign flip → negative output
    # → thrust_to_actuator clamps to 0 → motor stays off at 800 us.
    "H_TAIL_TYPE":          4,
    # SERVO4 range matches GB4008 hardware (800 us = off, 2000 us = max)
    "SERVO4_MIN":           800,
    "SERVO4_MAX":           2000,
    "SERVO4_TRIM":          800,   # DDFP: trim = off (motor off at neutral)
    # H_YAW_TRIM ≈ 0: near-zero feedforward; motor stays off during STARTUP (no overshoot).
    # Prescribed-yaw tests bypass the ODE so equilibrium isn't needed from the integrator.
    # P=0.5 provides the torque response; I=0 / IMAX=0 for open-loop prescribed tests.
    "H_YAW_TRIM":           0.02,
    # Yaw rate P-only for prescribed-yaw tests.  I=0 keeps the integrator inactive so
    # motor output is determined solely by P * psi_dot_error from the prescribed yaw signal.
    "ATC_RAT_YAW_P":        0.5,
    "ATC_RAT_YAW_I":        0.0,
    "ATC_RAT_YAW_D":        0.0,
    "ATC_RAT_YAW_IMAX":     0.0,
    # No Lua script; ArduPilot's built-in controller is the sole yaw actuator
    "SCR_ENABLE":           0,
})


# NOTE: _DDFP_RAMP_TORQUE_EXTRA_PARAMS is retained for reference but currently unused.
# torque_armed_ddfp_ramp uses _DDFP_TORQUE_EXTRA_PARAMS + yaw_slow_ramp profile
# (prescribed yaw) rather than the kinematic model driven by ArduPilot throttle.
#
# If kinematic DDFP ramp is needed in future: eq_throttle = 0.485 (RPM_SCALE=105, GEAR_RATIO=1.818).
# Pre-loading at H_YAW_TRIM=-0.35 gives throttle ≈ 0.44, below the equilibrium point.
# The I term accumulates from t=0 (psi_dot > 0 immediately due to motor under-speed).
_DDFP_RAMP_TORQUE_EXTRA_PARAMS = ParamSetup({
    "H_TAIL_TYPE":          4,
    "SERVO4_MIN":           800,
    "SERVO4_MAX":           2000,
    "SERVO4_TRIM":          800,
    "H_YAW_TRIM":           -0.419,  # eq_throttle = 0.485
    "ATC_RAT_YAW_P":        0.5,
    "ATC_RAT_YAW_I":        0.1,
    "ATC_RAT_YAW_D":        0.0,
    "ATC_RAT_YAW_IMAX":     1.0,
    "SCR_ENABLE":           0,
})


@contextlib.contextmanager
def _torque_stack(
    tmp_path: Path,
    *,
    omega_rotor: float,
    profile: str = "constant",
    tail_channel: int = 3,
    extra_params=(),
    test_name: str = "",
    install_scripts: tuple = (),
    boot_params: "dict | None" = None,
    startup_hold_s: float = _TORQUE_STARTUP_HOLD_S,
    startup_yaw_rate_deg_s: float = 0.0,
):
    """
    Full torque-test stack lifecycle: pre-checks -> launch -> arm -> yield -> teardown.

    Built on top of _sitl_stack which handles pre-checks, EEPROM wipe, boot params,
    SITL launch, logging, and teardown.  This context manager adds:
      - mediator_torque.py launch (stationary hub yaw ODE + motor physics)
      - EKF compass-yaw alignment (short, ~3-10 s)
      - arm + ACRO mode entry

    Logs written to simulation/logs/{test_name}/ (per-test directory,
    matching the flight stack convention).

    Parameters
    ----------
    omega_rotor            : rotor hub angular velocity [rad/s]
    profile                : mediator_torque.py --profile value
    tail_channel           : ArduPilot tail servo channel read by mediator
    extra_params           : ParamSetup merged into torque params before boot-file write
    test_name              : pytest test node name; used as logger name and for per-test log directory
    install_scripts        : tuple of Lua script names to install from simulation/scripts/
    boot_params            : dict of {param_name: value} merged on top of torque_setup
                             (for params only known at fixture call time, e.g. RPM1_TYPE).
    startup_yaw_rate_deg_s : yaw rate [deg/s] sent during startup hold for EKF init (0=stationary).
    """
    # Pre-launch: install Lua scripts before SITL starts
    if install_scripts:
        _install_lua_scripts(*install_scripts)

    # Build one complete param setup for this test.
    # Always-wipe EEPROM + per-test boot file = single source of truth.
    # Order (each layer overrides the previous):
    #   1. _BASE_TORQUE_BOOT_PARAMS  — all EKF/PID/RSC/arming/GPS-source values
    #   2. extra_params              — fixture-specific overrides (e.g. Lua)
    #   3. boot_params               — caller-supplied boot-time extras (e.g. RPM1_TYPE)
    torque_setup = (
        _BASE_TORQUE_BOOT_PARAMS
        .merge(extra_params or ParamSetup({}))
        .merge(ParamSetup(boot_params) if boot_params else ParamSetup({}))
    )

    with _sitl_stack(
        tmp_path,
        test_name   = test_name,
        base_params = torque_setup,
    ) as sitl_ctx:
        log = sitl_ctx.log
        log.info(
            "torque stack: profile=%s  omega_rotor=%.1f rad/s (%.0f RPM)  %d boot params",
            profile, omega_rotor, omega_rotor * 60.0 / (2.0 * math.pi), len(torque_setup),
        )

        mediator_log = tmp_path / "mediator.log"
        mavlink_log  = tmp_path / "mavlink.jsonl"
        events_path  = tmp_path / "events.jsonl"

        mediator_proc = _launch_mediator_torque(
            _TORQUE_DIR, sitl_ctx.repo_root, mediator_log, omega_rotor,
            profile=profile, tail_channel=tail_channel,
            startup_hold_s=startup_hold_s,
            events_log_path=str(events_path),
            startup_yaw_rate_deg_s=startup_yaw_rate_deg_s,
        )

        def _assert_alive() -> None:
            for name, proc, lp in [
                ("mediator_torque", mediator_proc,           mediator_log),
                ("SITL",           sitl_ctx.sitl_proc, sitl_ctx.sitl_log),
            ]:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{txt[-3000:]}")

        gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=mavlink_log)
        ctx = StackContext(
            gcs=gcs, mediator_proc=mediator_proc, sitl_proc=sitl_ctx.sitl_proc,
            mediator_log=mediator_log, sitl_log=sitl_ctx.sitl_log,
            gcs_log=sitl_ctx.gcs_log,
            events_log=MediatorEventLog(events_path),
            omega_rotor=omega_rotor, log=log,
        )

        try:
            log.info("Connecting GCS ...")
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            _assert_alive()
            log.info("GCS connected")

            gcs.request_stream(_mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
            gcs.request_stream(_mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2)
            gcs.request_stream(_mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)

            log.info("Waiting for param subsystem ...")
            deadline = gcs.sim_now() + 20.0
            while gcs.sim_now() < deadline:
                _assert_alive()
                gcs._mav.mav.param_request_read_send(
                    gcs._target_system, gcs._target_component, b"SYSID_THISMAV", -1,
                )
                msg = gcs._recv(type="PARAM_VALUE", blocking=True, timeout=1.0)
                if msg is not None:
                    log.info("Param subsystem ready (SYSID_THISMAV=%g)", msg.param_value)
                    break
            else:
                pytest.fail("Param subsystem never responded within 20 s")

            # Motor interlock HIGH + EKF alignment
            # Keep CH8 alive every 0.5 s (ArduPilot expires RC override after ~1 s).
            log.info("Waiting for EKF yaw alignment (CH8=2000, up to 45 s) ...")
            gcs.send_rc_override({8: 2000})
            ekf_ok  = False
            t_start = gcs.sim_now()
            t_rc    = gcs.sim_now()
            deadline = gcs.sim_now() + 45.0
            _MIN_WAIT = 3.0

            while gcs.sim_now() < deadline:
                _assert_alive()
                if gcs.sim_now() - t_rc >= 0.5:
                    gcs.send_rc_override({8: 2000})
                    t_rc = gcs.sim_now()
                msg = gcs._recv(
                    type=["ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.5,
                )
                if msg is None:
                    continue
                now = gcs.sim_now()
                if msg.get_type() == "STATUSTEXT":
                    text = msg.text.rstrip("\x00").strip()
                    log.info("SITL: %s", text)
                    if "EKF3 active" in text or "EKF3 IMU" in text:
                        gcs.request_stream(_mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
                    if "rawes" in text.lower() and "mode=2" in text.lower():
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

            # Verify all params after EKF ready — non-critical path.
            log.info("Verifying boot parameters via MAVLink ...")
            torque_setup.verify(gcs, log=log, read_timeout=1.0)
            _assert_alive()

            gcs.arm(timeout=15.0, force=True, rc_override={8: 2000})
            log.info("Armed")
            gcs.set_mode(ACRO, timeout=10.0, rc_override={8: 2000})
            log.info("ACRO active -- profile=%s", profile)

            yield ctx

        finally:
            log.info("Teardown: closing GCS and terminating mediator ...")
            try:
                gcs.close()
            except Exception:
                pass
            _terminate_process(mediator_proc)
            # sitl/gcs/arducopter/dataflash logs are handled by _sitl_stack teardown.
            # Copy mediator log and mavlink log here.
            _logs = {"mediator.log": mediator_log}
            if mavlink_log.exists():
                _logs["mavlink.jsonl"] = mavlink_log
            if events_path.exists():
                _logs["events.jsonl"] = events_path
            copy_logs_to_dir(sitl_ctx.test_log_dir, _logs)
            log.info("Mediator log copied to %s", sitl_ctx.test_log_dir)
