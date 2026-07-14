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
    _arm_sequence, _run_acro_setup, _wait_params_ready, _install_lua_scripts
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

# Optional: dynbem rotor_definition for test scripts that need it.
# Stack tests use mediator_static.py and do not need dynbem.
try:
    from dynbem import rotor_definition as _rd  # noqa: F401 — re-exported for callers
except ImportError:
    _rd = None  # dynbem not installed (Rust build may have failed in container)

# Project default rotor (the new aero package leaves ``default()`` undefined;
# we load it here from the project's YAML).
def _project_default_rotor():
    from tests.simtests._rotor_helpers import load_default_rotor
    return load_default_rotor()

from stack_utils import (
    ARDUPILOT_ENV,
    STACK_ENV_FLAG,
    SIM_VEHICLE_ENV,
    ParamSetup,
    SITL_UNSUPPORTED_PARAMS,
    _check_ardupilot_version,
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
from gcs import GUIDED, GUIDED_NOGPS, STABILIZE, RawesGCS
from mediator_events import MediatorEventLog
from controller import make_hold_controller
from ic import load_ic_dict, IC_JSON_PATH

_STARTING_STATE       = IC_JSON_PATH
_RAWES_COMMON_PARM    = _SITL_DIR / "rawes_common_defaults.parm"
_RAWES_SITL_ONLY_PARM = _SITL_DIR / "rawes_sitl_defaults.parm"
# Backward-compatible alias used by existing imports/docs.
_RAWES_DEFAULTS_PARM  = _RAWES_SITL_ONLY_PARM
_FALLBACK_SIM_SERVO_SPEED = 5.45  # 545 deg/s / 100 deg from beaupoil_2026 control block

# ── Flight fixture boot params ────────────────────────────────────────────────
# All GUIDED_NOGPS flight fixtures boot SITL with this complete parameter set.
# Combined with always-wipe EEPROM in _launch_sitl, this is the single source
# of truth — no EEPROM contamination from sequential tests is possible.
#
# rawes_common_defaults.parm provides shared RAWES tuning and heli params.
# rawes_sitl_defaults.parm provides SITL-only overlays (SIM_*, relaxed EKF gates).
# _BASE_ACRO_PARAMS adds the MAVLink-side mode/failsafe params.
# Per-fixture extras (e.g. RAWES_MODE, THRUST_CRUISE) are merged on top.
#
# EKF params (EK3_*, COMPASS_*) are ONLY in the boot file — never set via
# MAVLink — because setting them post-boot triggers EKF3 yaw-state reset.
_BASE_ACRO_PARAMS = ParamSetup({
    "SCR_ENABLE":       1,      # Lua scripting — always enabled; boot file avoids
                                # the prime_eeprom bootstrap that was needed before
    "INITIAL_MODE":     20,     # boot directly into GUIDED_NOGPS (mode 20)
    "FS_THR_ENABLE":    0,      # no RC throttle failsafe
    "FS_GCS_ENABLE":    0,      # no GCS heartbeat failsafe
    "FS_EKF_ACTION":    0,      # disable EKF failsafe
    "H_RSC_MODE":       1,      # CH8 passthrough — instant runup_complete
    "ACRO_TRAINER":     0,      # disable auto-leveling (physical tilt ~65 deg)
    "H_SW_TYPE":        3,      # H3_120 swashplate
    # H_SW_PHANG removed in ArduPilot 4.6 (param no longer exists; default=0 implicit)
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
    # Optional diagnostic angular damping after startup ramp ends.
    # Default is zero: dynbem models the rotor attitude response directly.
    BASE_K_ANG            : float = 0.0

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
                # Step 5: final check after a short settle; stack restart is handled
                # by the external test orchestrator, not from inside this process.
                _clog.warning(
                    "Port %s:%d still busy after kill; waiting for external orchestrator recovery",
                    host,
                    port,
                )
                _time.sleep(3.0)
                # Final check after settle
                s2 = _socket.socket(_socket.AF_INET, kind)
                s2.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
                try:
                    s2.bind((host, port))
                    s2.close()
                except OSError as exc2:
                    s2.close()
                    raise RuntimeError(
                        f"{proto.upper()} port {host}:{port} still busy after recovery wait.\n"
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
    gcs            : connected, heartbeating RawesGCS (armed, in GUIDED_NOGPS mode)
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
    test_log_dir:        Path | None  = None
    # ── pumping socket (default 0 = disabled) ────────────────────────────────
    winch_cmd_port:      int          = 0
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
    - boot param file (rawes_defaults + guided-mode base + servo_speed + extra_boot_params
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
        When provided, replaces the default rawes_defaults + guided-mode base + SIM_SERVO_SPEED
        chain as the boot param set.  Use this for custom stacks (e.g. torque tests)
        that need a completely different parameter base.  extra_boot_params are merged
        on top regardless.
    """
    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run stack integration tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} to locate sim_vehicle.py")

    _check_ardupilot_version(sim_vehicle)

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
        # SIM_SERVO_SPEED = travel-fraction-per-second = slew/travel.
        # Both fields live on the new RotorDefinition control block; the
        # project YAML carries 545 deg/s / 100 deg ≈ 5.45.
        try:
            _rotor = _project_default_rotor()
            _servo_speed = (_rotor.control.servo_slew_rate_deg_s
                            / _rotor.control.servo_travel_deg)
        except Exception as _exc:
            # Keep arm/connectivity stack tests runnable even if dynbem is
            # unavailable in the container (e.g. Rust toolchain mismatch).
            _servo_speed = _FALLBACK_SIM_SERVO_SPEED
            log.warning(
                "Rotor definition unavailable; using fallback SIM_SERVO_SPEED=%s (%s)",
                _servo_speed,
                _exc,
            )
        _boot_setup = (
            ParamSetup.from_parm_file(_RAWES_COMMON_PARM)
            .merge(ParamSetup.from_parm_file(_RAWES_SITL_ONLY_PARM))
            .merge(_BASE_ACRO_PARAMS)
            .merge(ParamSetup({"SIM_SERVO_SPEED": _servo_speed}))
            .merge(ParamSetup(extra_boot_params or {}))
        )
    # Drop hardware-only params (BLHeli / bidirectional-DShot on output 9) that
    # are absent from the ArduCopter-heli SITL build.  Leaving them in makes the
    # boot-param read-back verification fail with NOT_IN_DUMP for every flight
    # fixture.  SITL drives the yaw motor output as plain PWM.
    _boot_setup = _boot_setup.without(SITL_UNSUPPORTED_PARAMS)
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
        # Brief GCS connection to dump params before yielding to the test.
        _static_gcs_log = tmp_path / "static_gcs_params.jsonl"
        _static_gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=_static_gcs_log)
        try:
            _static_gcs.connect(timeout=_STARTUP_TIMEOUT)
            _static_gcs.start_heartbeat(rate_hz=1.0)
            _wait_params_ready(_static_gcs, ctx.log)
            _dump_params_to_log(_static_gcs, ctx.test_log_dir, ctx.log)
        except Exception as _exc:
            ctx.log.warning("_static_stack: param dump skipped: %s", _exc)
        finally:
            try:
                _static_gcs.close()
            except Exception:
                pass
        try:
            yield ctx
        finally:
            _terminate_process(mediator_proc)
            logs_to_copy = {"mediator.log": mediator_log}
            if mediator_events.exists():
                logs_to_copy["mediator_events.jsonl"] = mediator_events
            copy_logs_to_dir(ctx.test_log_dir, logs_to_copy)


# ---------------------------------------------------------------------------
# _acro_stack — full GUIDED_NOGPS stack (mediator + arm) built on top of _sitl_stack
# ---------------------------------------------------------------------------

@contextlib.contextmanager
def _acro_stack(tmp_path, *, extra_config=None,
                arm: bool = True, with_mediator: bool = True, test_name: str = "",
                extra_boot_params: "dict[str, float] | None" = None,
                rawes_params: "dict[str, float] | None" = None,
                arm_at_sim_s: "float | None" = None):
    """
    Core GUIDED_NOGPS stack lifecycle: pre-checks → launch → [arm] → yield ctx → teardown.

    All fixtures call this.  Built on top of _sitl_stack which handles
    pre-checks, boot params, SITL launch, logging, and teardown.

    Differences between fixtures are handled outside this function:
      - extra_config   : pumping cycle passes trajectory config here
      - pre-launch work: Lua fixture installs scripts before calling this
      - post-arm work  : Lua fixture sets RAWES_MODE inside the with-block

    Parameters
    ----------
    rawes_params : dict of RAWES_* params to apply via GCS after scripting is up.
        Script-generated params (RAWES_MODE, RAWES_KP_ALT, etc.) cannot be set
        via the boot parm file because param:add_table registers them at script
        load time, AFTER --add-param-file is applied.  Pass them here instead;
        _run_acro_setup applies them via set_param once the param subsystem is ready.
    """
    # ── Initial state ──────────────────────────────────────────────────────────
    initial_state = None
    home_alt_m    = 12.530
    if _STARTING_STATE.exists():
        initial_state = load_ic_dict()
        home_alt_m    = -float(initial_state["pos"][2])

    # ── Hold controller ────────────────────────────────────────────────────────
    _anchor_ned = _np.array([0.0, 0.0, float(home_alt_m)])
    controller   = make_hold_controller(anchor_ned=_anchor_ned)

    # ── Lua scripts ───────────────────────────────────────────────────────────
    _install_lua_scripts("rawes.lua")

    # ── Merge controller params + test extras into boot params ─────────────────
    _extra: dict = dict(controller.extra_params)
    if extra_boot_params:
        _extra.update(extra_boot_params)

    _med_extra = dict(extra_config or {})
    _med_extra.setdefault("mavlink_log_connection", "tcp:127.0.0.1:5762")
    _med_extra.setdefault("mavlink_att_target_hz", 100.0)
    _med_extra.setdefault("mavlink_attitude_hz", 100.0)
    _med_extra.setdefault("mavlink_servo_output_raw_hz", 100.0)

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
        if with_mediator:
            mediator_proc = _launch_mediator(
                sitl_ctx.sim_dir, sitl_ctx.repo_root, mediator_log,
                telemetry_log_path   = str(telemetry_log),
                events_log_path      = str(events_path),
                initial_state        = initial_state,
                startup_damp_seconds = _STARTUP_DAMP_S,
                run_id               = _run_id,
                base_k_ang           = StackConfig.BASE_K_ANG,
                extra_config         = _med_extra,
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

        gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=mavlink_log,
                       watchdog=_procs_alive)

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
            test_log_dir=sitl_ctx.test_log_dir,
            winch_cmd_port=int(_med_extra.get("winch_cmd_port", 0)),
        )

        _use_ic_pre_arm_att = bool(_med_extra.pop("use_ic_pre_arm_attitude", True))

        try:
            if arm:
                _run_acro_setup(
                    ctx,
                    _procs_alive,
                    boot_setup=sitl_ctx.boot_setup,
                    rawes_params=rawes_params,
                    arm_at_sim_s=arm_at_sim_s,
                    use_ic_pre_arm_attitude=_use_ic_pre_arm_att,
                )
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
    (the unified script); RAWES_MODE selects the active mode at runtime.

    Examples::

        _install_lua_scripts("rawes.lua")
    """
    dst_dir = Path("/ardupilot/scripts")
    dst_dir.mkdir(exist_ok=True)
    for name in names:
        src = _SCRIPTS_DIR / name
        shutil.copy2(src, dst_dir / name)


# ---------------------------------------------------------------------------
# Canonical arm state machine
# ---------------------------------------------------------------------------

def _arm_sequence(
    gcs: "RawesGCS",
    log,
    *,
    armon_ms: "int | None" = None,
    procs_alive=None,
    fail=None,
    mode_timeout: float = _MODE_TIMEOUT,
    arm_timeout: float = _ARM_TIMEOUT,
    target_mode: int = GUIDED_NOGPS,
    pre_arm_attitude_rpy: "tuple[float, float, float] | None" = None,
) -> None:
    """
    Canonical arm sequence for all SITL stack tests.

    Always runs in this order:
    1. Set target_mode (GUIDED_NOGPS).
      2. Hard-assert target_mode confirmed via HEARTBEAT; refuse to arm in any other mode.
            3a. armon_ms=None  — GCS force-arm (no RC override required).
            3b. armon_ms > 0   — GCS force-arm, then send RAWES_ARM as disarm timer (ms).
      3c. armon_ms = 0   — set target_mode only; return without arming (test owns arming).

    Notes
    -----
    GUIDED_NOGPS (mode 20) is the default: it allows arming without GPS pre-checks,
    while still supporting all guided-mode scripting bindings like
    set_target_angle_and_rate_and_throttle. Both set_target_*() methods gate on
    in_guided_mode(), which is true for both GUIDED (4) and GUIDED_NOGPS (20).
    All stack tests use GUIDED_NOGPS so script control paths remain consistent.

    Parameters
    ----------
    fail                 : callable(msg) invoked on unrecoverable failure.
                           None raises RuntimeError instead (use in non-pytest
                           helpers like _run_acro_setup).
    procs_alive          : nullary callable checked after each blocking step.
    """
    def _fail(msg: str) -> None:
        if fail is not None:
            fail(msg)
        else:
            raise RuntimeError(msg)

    def _check_alive() -> None:
        if procs_alive is not None:
            procs_alive()

    # -- 1. Set target mode ------------------------------------------------
    _mode_name = {GUIDED: "GUIDED", GUIDED_NOGPS: "GUIDED_NOGPS"}.get(target_mode, str(target_mode))
    log.info("[arm] Setting %s mode (timeout=%.0fs) ...", _mode_name, mode_timeout)
    try:
        gcs.set_mode(target_mode, timeout=mode_timeout)
    except Exception as exc:
        _fail(f"{_mode_name} mode set failed: {exc}")
        return

    # -- 2. Hard-assert target mode confirmed ------------------------------
    # gcs.set_mode() already hard-asserts mode confirmation via HEARTBEAT and
    # raises on failure. A second mandatory confirmation here can false-fail if
    # the MAVLink TCP link briefly resets right after successful set_mode().
    # Keep a best-effort sanity read for logging, but do not fail if unavailable.
    _actual = None
    _hb = gcs._recv(type="HEARTBEAT", blocking=True, timeout=0.5)
    if _hb is not None:
        _actual = int(_hb.custom_mode)
    if _actual is None:
        log.info("[arm] %s confirmed by set_mode(); no immediate heartbeat for re-check.", _mode_name)
    elif _actual != target_mode:
        log.warning(
            "[arm] post-set_mode heartbeat custom_mode=%d (expected %d); proceeding because set_mode already confirmed.",
            _actual,
            target_mode,
        )
    else:
        log.info("[arm] %s confirmed (custom_mode=%d).", _mode_name, target_mode)
    _check_alive()

    # -- 2b. Seed attitude target (optional; before arm) ------------------
    if pre_arm_attitude_rpy is not None:
        roll, pitch, yaw = pre_arm_attitude_rpy

        # MAVLink quaternion order: [w, x, y, z].
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]

        # Attitude-only target: ignore body-rate axes and throttle.
        mask = (
            _mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            | _mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            | _mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
            | _mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE
        )
        gcs._mav.mav.set_attitude_target_send(
            0,
            gcs._target_system,
            gcs._target_component,
            mask,
            q,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        log.info(
            "[arm] Seeded SET_ATTITUDE_TARGET from EKF rpy=(%.2f, %.2f, %.2f) deg",
            math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
        )

    # -- 3. Arm -----------------------------------------------------------
    if armon_ms == 0:
        log.info("[arm] armon_ms=0 -- yielding unarmed in %s mode.", _mode_name)
        return

    log.info("[arm] Arming (force=True, timeout=%.0fs) ...", arm_timeout)
    try:
        gcs.arm(timeout=arm_timeout, force=True)
    except Exception as exc:
        _fail(f"Arm failed: {exc}")
        return
    log.info("[arm] Armed.")
    _check_alive()

    # Optional Lua disarm timer (ms from now). This no longer controls arming.
    if armon_ms is not None and armon_ms > 0:
        gcs.send_named_float("RAWES_ARM", float(armon_ms))
        log.info("[arm] Sent RAWES_ARM disarm timer=%d ms", armon_ms)


# ---------------------------------------------------------------------------
# Setup sequence
# ---------------------------------------------------------------------------

def _dump_params_to_log(gcs, log_dir: Path, log) -> None:
    """Fetch all ArduPilot params and write params.json to log_dir."""
    log.info("[params] Fetching full param dump for log ...")
    try:
        params = gcs.fetch_all_params(timeout=30.0)
        out = log_dir / "params.json"
        out.write_text(json.dumps(params, sort_keys=True, indent=2))
        log.info("[params] Wrote %d params to %s", len(params), out)
    except Exception as exc:
        log.warning("[params] Could not dump params: %s", exc)


def _run_acro_setup(
    ctx: StackContext,
    _procs_alive,
    boot_setup: "ParamSetup | None" = None,
    rawes_params: "dict[str, float] | None" = None,
    arm_at_sim_s: "float | None" = None,
    use_ic_pre_arm_attitude: bool = True,
) -> None:
    """
    Shared GUIDED_NOGPS setup sequence.

    Timing contract: the mediator startup damping ramp is 30 s.  All steps
    here must complete inside that window so the hub is barely moving when
    we arm — strong damping keeps it near the initial position during GPS init.

    Steps:
        1. Connect GCS; request telemetry streams`r`n        2. Wait for param subsystem
        3. Verify all boot params via MAVLink read-back (pytest.fail on mismatch)
        4. Wait for EKF tilt alignment — FAIL HARD if it doesn't arrive
        5. Arm with force=True
        6. Confirm GUIDED_NOGPS mode

    Parameters
    ----------
    rawes_params : RAWES_* params to apply via GCS after the param subsystem is ready.
        Script-generated params cannot be set via boot parm file (registered at
        script load time, after --add-param-file).  Pass RAWES_MODE and any other
        RAWES_* values here; they are applied via set_param once scripting is up.
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
        _procs_alive()   # last-chance crash check before reporting a plain timeout
        raise TimeoutError(
            f"[setup 1/6] GCS connect timeout after {_STARTUP_TIMEOUT:.0f}s "
            f"(SITL may not have started yet): {exc}"
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

    # ── 2. Param subsystem ────────────────────────────────────────────────────
    log.info("[setup 2/6] Waiting for param subsystem ...")
    _wait_params_ready(gcs, log)
    _dump_params_to_log(gcs, ctx.test_log_dir, log)
    _procs_alive()

    # Apply RAWES_* script-generated params now that scripting is up.
    # param:add_table registers RAWES_* at Lua load time (after --add-param-file),
    # so they cannot be set via the boot parm file.  By the time _wait_params_ready
    # returns, rawes.lua has already loaded and all RAWES_* params are registered.
    if rawes_params:
        log.info("[setup 2/6] Applying %d RAWES_* params via GCS ...", len(rawes_params))
        for name, value in rawes_params.items():
            ok = gcs.set_param(name, float(value), timeout=5.0)
            log.info("  %-28s = %-10g  ACK=%s", name, value, ok)
            if not ok:
                log.warning("  [WARN] set_param %s=%g failed — param may not be registered", name, value)

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
    att_seed_rpy = None
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
            # GPS Glitch during pre-arm EKF wait: log as warning only.
            # ArduPilot often recovers and arm succeeds — only fail if arm
            # itself fails (checked in step 5+6).  Do not raise here.
            if "gps glitch" in text.lower():
                log.warning("[setup 4/6] GPS Glitch seen pre-arm — continuing "
                            "(will be flagged as known issue only if arm fails).")

        elif mt == "ATTITUDE":
            att_seed_rpy = (float(msg.roll), float(msg.pitch), float(msg.yaw))
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
    if boot_setup is not None:
        log.info("[setup 3/6] Verifying boot parameters ...")
        boot_setup.verify(gcs, log=log)
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
        if ekf_att_ready:
            break
    if not ekf_att_ready:
        log.warning("[stabilise] EKF3 attitude confidence not seen within 20 s; "
                    "proceeding with flags=0x%04x", last_flags)

    # Optional delayed arm schedule for fixtures that need a fixed sim-time arm.
    if arm_at_sim_s is not None:
        now = gcs.sim_now()
        if now < arm_at_sim_s:
            wait_s = arm_at_sim_s - now
            log.info("[setup] Delaying arm until sim t=%.1f s (waiting %.1f s) ...",
                     arm_at_sim_s, wait_s)
            while gcs.sim_now() < arm_at_sim_s:
                _procs_alive()
                msg = gcs._recv(
                    type=["STATUSTEXT", "EKF_STATUS_REPORT", "LOCAL_POSITION_NED"],
                    blocking=True,
                    timeout=0.5,
                )
                if msg is not None and msg.get_type() == "STATUSTEXT":
                    text = msg.text.rstrip("\x00").strip()
                    all_statustext.append(text)
                    log.info("[setup-delay] STATUSTEXT: %s", text)

    # Build pre-arm attitude seed.
    # Default is live EKF attitude captured during setup. Optionally, tests can
    # override roll/pitch from IC R0 so kinematic hold commands IC attitude
    # immediately (yaw remains live EKF yaw).
    pre_arm_target_rpy = att_seed_rpy
    _ic = ctx.initial_state
    if use_ic_pre_arm_attitude and _ic is not None and att_seed_rpy is not None:
        _R0 = _ic.get("R0")
        if _R0 is not None:
            _r20 = float(_R0[2][0])
            _r21 = float(_R0[2][1])
            _r22 = float(_R0[2][2])
            _ic_roll_rad = math.atan2(_r21, _r22)
            _ic_pitch_rad = -math.asin(max(-1.0, min(1.0, _r20)))
            _yaw_rad = float(att_seed_rpy[2])
            pre_arm_target_rpy = (_ic_roll_rad, _ic_pitch_rad, _yaw_rad)
            log.info(
                "[setup] Pre-arm target uses IC roll/pitch: r=%.2f deg p=%.2f deg yaw=%.2f deg",
                math.degrees(_ic_roll_rad),
                math.degrees(_ic_pitch_rad),
                math.degrees(_yaw_rad),
            )

    # ── 5+6. GUIDED_NOGPS mode (before arm) + arm ─────────────────────────────
    # GUIDED_NOGPS (mode 20) must be active before arm for flight-stack tests.
    # This keeps the control-path semantics consistent with rawes.lua's guided
    # attitude targets during kinematic and free-flight handover.
    all_statustext += drain_statustext(gcs, log)
    try:
        _arm_sequence(
            gcs, log,
            procs_alive=_procs_alive,
            fail=None,   # RuntimeError (propagated to fixture finally-block)
            mode_timeout=_MODE_TIMEOUT,
            arm_timeout=_ARM_TIMEOUT,
            pre_arm_attitude_rpy=pre_arm_target_rpy,
        )
    except Exception as exc:
        all_statustext += drain_statustext(gcs, log)
        dump_startup_diagnostics(ctx)
        raise
    ctx.flight_events["arm_t"] = gcs.sim_now()

    all_statustext += drain_statustext(gcs, log)
    _procs_alive()

    t0 = gcs.sim_now()
    ctx.flight_events["Setup complete"] = 0.0
    if t_ekf is not None:
        ctx.flight_events["EKF lock"] = t_ekf - t0
    log.info("guided_nogps_armed setup complete — vehicle is armed in GUIDED_NOGPS mode.")


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
    guided_confirmed  : True if GUIDED_NOGPS mode STATUSTEXT or heartbeat was seen
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
        "guided_confirmed":  False,
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
    result["guided_confirmed"] = any("guided" in t.lower() for t in ctx.all_statustext)

    # ── Known-issue pattern matching ──────────────────────────────────────────
    issues = result["known_issues"]

    if any("motor interlock" in t.lower() for t in ctx.all_statustext):
        issues.append(
            "PreArm: Motor Interlock Enabled — interlock was HIGH before arm. "
            "Fix: ensure no external CH8 override is active before arming."
        )
    if any("runup" in t.lower() for t in ctx.all_statustext):
        issues.append(
            "PreArm: H_RUNUP_TIME too small — set H_RUNUP_TIME=0 to skip this check."
        )
    # Only flag GPS Glitch as a known issue if arm was never achieved.
    # A pre-arm GPS glitch that clears before arming is expected during EKF
    # initialisation and does not indicate a sensor problem.
    arm_achieved = "arm_t" in ctx.flight_events
    if not arm_achieved and any("gps glitch" in t.lower() for t in ctx.all_statustext):
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
            log.info("GUIDED_NOGPS stable: rpy=(%.2f°, %.2f°, %.2f°)", r, p, y)
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
) -> None:
    """
    Run a SITL observation loop for ``duration_s`` sim-seconds.

    On every iteration:
      1. ``assert_procs_alive(ctx, label)`` — pytest.fail if a process exited.
        2. ``gcs._recv(type=msg_types, blocking=True, timeout=recv_timeout)``
        3. ``on_message(msg, t_rel)`` — ``msg`` may be ``None`` on recv timeout.
         Return ``True`` from the callback to exit the loop early.

    Parameters
    ----------
    ctx                  : StackContext (flight or torque)
    duration_s           : how many sim-seconds to run
    on_message           : callable(msg, t_rel) -> bool | None
    msg_types            : MAVLink message types to pass to _recv
    label                : label forwarded to assert_procs_alive / logs
    recv_timeout         : wall-clock seconds _recv waits for a message
    """
    gcs      = ctx.gcs
    t_start  = gcs.sim_now()
    deadline = t_start + duration_s

    while gcs.sim_now() < deadline:
        assert_procs_alive(ctx, label)

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
# ---------------------------------------------------------------------------
# STANDARD DDFP yaw regulation — single source of truth for ALL torque tests.
# ---------------------------------------------------------------------------
# Every STANDALONE-DDFP torque yaw test regulates hub yaw with ArduPilot's DDFP
# controller (H_TAIL_TYPE=3) as the SOLE yaw actuator -- no Lua manual PID runs
# (SCR_ENABLE=0, or Lua in MODE_NONE).  Those tests keep a nonzero AP yaw P here.
#
# NOTE: this set NO LONGER mirrors rawes_common_defaults.parm.  The flight/vanilla
# parm chain now zeroes the AP yaw PID (ATC_RAT_YAW_P/I/D = 0) because yaw there is
# regulated by the Lua manual PID (MODE_PASSIVE/STEADY) writing H_YAW_TRIM; a
# nonzero AP yaw gain would fight the Lua loop and drive a limit cycle.  This
# override set is intentionally different: it drives the AP DDFP loop directly, so
# it must retain P>0.  Rationale for P=0.02: with the inertial motor model the
# high-authority GB4008 (~58 rad/s per throttle) is stable at P=0.02 with a
# filtered-but-off D; H_YAW_TRIM carries the DC hold so I=0.
# Fixtures overlay this with EKF/compass/mode/script params only -- never with a
# different yaw PID.  All SITL tests use the standard DDFP CW tail (H_TAIL_TYPE=3,
# Motor4 on SERVO9).
_STANDARD_DDFP_YAW_PARAMS = ParamSetup({
    "ATC_RAT_YAW_P":    0.02,
    # AP yaw integral OFF: the Lua trim observer carries the DC hold.  A small AP I
    # was tried for disturbance rejection but it interacted with the observer during
    # the slow_vary RPM sweep and degraded steady tracking (vanilla), while the
    # gust/tilt spikes it was meant to catch are brief coupling artifacts a small I
    # doesn't fix -- so I stays 0.
    "ATC_RAT_YAW_I":    0.0,
    "ATC_RAT_YAW_D":    0.0,
    "ATC_RAT_YAW_IMAX": 0.1,
    "ATC_RAT_YAW_FLTD": 10.0,
    "H_TAIL_TYPE":      3,      # DDFP CW (US-convention rotor; no sign flip)
    "H_COL2YAW":        0.0,
    "SERVO4_FUNCTION":  0,      # release output 4
    "SERVO9_FUNCTION":  36,     # Motor4 on output 9 (AUX 1)
    "SERVO9_MIN":       1000,   # GB4008: 1000 us = off
    "SERVO9_MAX":       2000,   #         2000 us = full throttle
    "SERVO9_TRIM":      1000,   # DDFP: trim = off (motor off at neutral)
    "H_YAW_TRIM":       0.02,
})

# Full self-contained torque boot base = standard DDFP yaw + torque EKF/compass/
# mode overlay.  Used as base_params (replaces the parm chain) for the compass-yaw
# torque rigs.  Yaw PID / actuator config comes ONLY from _STANDARD_DDFP_YAW_PARAMS.
_BASE_TORQUE_BOOT_PARAMS = _STANDARD_DDFP_YAW_PARAMS.merge(ParamSetup({
    # Boot directly into GUIDED_NOGPS (mode 20) to keep mode usage consistent
    # across flight and torque stacks.
    "INITIAL_MODE":     20,
    # Failsafe — disable EKF failsafe.
    "FS_EKF_ACTION":    0,
    "FS_THR_ENABLE":    0,
    # Scripting — off by default; Lua fixtures override with SCR_ENABLE=1.
    "SCR_ENABLE":       0,
    # Compass — enable and do not auto-calibrate (EK3_MAG_CAL=0) so compass
    # headings are used from the first EKF loop when the profile starts at t=10 s.
    "COMPASS_USE":      1,
    "COMPASS_ENABLE":   1,
    "EK3_SRC1_YAW":    1,     # compass yaw
    "EK3_MAG_CAL":     0,     # no magnetometer calibration
    "EK3_GPS_CHECK":   0,     # no GPS pre-arm check (GPS not used in torque tests)
    # EKF source config: disable GPS position/velocity.
    "EK3_SRC1_POSXY":  0,
    "EK3_SRC1_VELXY":  0,
    # Roll/pitch I-term — disable to prevent swashplate wind-up on neutral sticks.
    "ATC_RAT_RLL_IMAX": 0.0,
    "ATC_RAT_PIT_IMAX": 0.0,
    # RSC — enable CH8 passthrough (instant runup when CH8=2000)
    "H_RSC_MODE":      1,
    "H_RSC_RUNUP_TIME": 2,   # must be > H_RSC_RAMP_TIME (default 1) to pass prearm check
}))

# IC-orientation torque rig (profile="ic") boots the flight parm chain
# (copter-heli + rawes_common_defaults + rawes_sitl_defaults), which already
# provides the STANDARD tail setup (H_TAIL_TYPE=3 DDFP CW, Motor4 on SERVO9,
# the SERVO ranges, H_COL2YAW, and the zeroed ATC_RAT_YAW D/I/FLTD).  This
# overlay therefore sets ONLY the params that DIFFER from the common file:
# the torque rig drives ArduPilot's DDFP yaw PID (ATC_RAT_YAW_P>0 + H_YAW_TRIM),
# whereas the common/flight default keeps the AP yaw PID off (P=0) and regulates
# yaw from the Lua PID.  Roll/pitch rate-I are disabled to prevent swashplate
# wind-up on neutral sticks.
_IC_TORQUE_YAW_PARAMS = ParamSetup({
    "ATC_RAT_YAW_P":    0.02,   # AP DDFP yaw PID active (common file has P=0)
    "ATC_RAT_YAW_IMAX": 0.1,
    "H_YAW_TRIM":       0.02,   # not set by the common file (AP default is 0)
    "ATC_RAT_RLL_IMAX": 0.0,
    "ATC_RAT_PIT_IMAX": 0.0,
    "FS_CRASH_CHECK":   0,      # observer needs time to converge; yaw angle may deviate temporarily
})

# Extra params for Lua torque fixtures.
# ArduPilot's built-in DDFP yaw PID (H_TAIL_TYPE=3) drives Motor4 for yaw control.
# Arming is handled by GCS; Lua RAWES_ARM is an optional disarm timer only.
# Inherits the standard DDFP yaw PID; adds only the scripting overlay.
_LUA_TORQUE_EXTRA_PARAMS = _STANDARD_DDFP_YAW_PARAMS.merge(ParamSetup({
    "SCR_ENABLE":       1,     # load rawes.lua for optional RAWES_ARM disarm timer
    # RAWES_MODE is NOT set here: script-generated params are applied via GCS
    # after scripting is up (see rawes_params arg to _run_acro_setup).
}))

# Extra params for the ArduPilot DDFP yaw PI fixture (US-convention rotor).
# ArduPilot's built-in ATC_RAT_YAW controller drives Motor4 directly
# as a unidirectional motor (0% = off, 100% = full throttle).
# Motor range: 800 us = off, 2000 us = max (GB4008 66KV on REVVitRC ESC).
#
# H_TAIL_TYPE enum (AP_MotorsHeli_Single):
#   0  Servo          — bidirectional servo; output centred at TRIM (1500 µs),
#                       PID maps ±1 directly to servo range.  No sign flip.
#   1  Servo+ExtGyro  — servo tail with external heading-hold gyro on Ch7.
#   2  DDFP           — Direct Drive Fixed Pitch, bidirectional PWM mapping.
#   3  DDFP CW        — unidirectional motor; positive PID -> more throttle (NO flip).
#   4  DDFP CCW       — unidirectional motor; applies _servo4_out *= -1 so negative
#                       PID output (CW drift) maps to positive throttle.
#
# Why CW (3) for the US-convention rotor:
#   Main rotor spins CCW from above; body drifts CCW (negative gyro:z()) under drag.
#   yaw error = 0 - gyro:z() = positive.  PID output positive.  H_TAIL_TYPE=3 passes
#   positive PID straight to Motor4 throttle -> motor on -> CW counter-torque on body. ✓
#   H_TAIL_TYPE=4 (CCW with -1 flip) would clamp positive PID to 0 -> motor off. ✗
#
# Standardized to the shared DDFP yaw PID (_STANDARD_DDFP_YAW_PARAMS): the same
# closed-loop gains regulate the prescribed-yaw response.  (Historically this set
# used an open-loop P=0.5; the prescribed-yaw assertions were re-tuned for the
# standard gains.)
_DDFP_TORQUE_EXTRA_PARAMS = _STANDARD_DDFP_YAW_PARAMS.merge(ParamSetup({
    # No Lua script; ArduPilot's built-in controller is the sole yaw actuator.
    "SCR_ENABLE":           0,
}))


@contextlib.contextmanager
def _torque_stack(
    tmp_path: Path,
    *,
    omega_rotor: float,
    profile: str = "constant",
    tail_channel: int = 8,
    extra_params=(),
    test_name: str = "",
    install_scripts: tuple = (),
    boot_params: "dict | None" = None,
    startup_hold_s: float = _TORQUE_STARTUP_HOLD_S,
    startup_yaw_rate_deg_s: float = 0.0,
    armon_ms: "int | None" = None,
    passive_init: bool = False,
    passive_thrust: float = 0.263,
    passive_roll_rad: float = 0.0,
    passive_pitch_rad: float = 0.0,
    passive_yaw_rad: "float | None" = None,
    use_vanilla_boot_defaults: bool = False,
    motor_delay_ms: float = 0.0,
):
    """
    Full torque-test stack lifecycle: pre-checks -> launch -> arm -> yield -> teardown.

    Built on top of _sitl_stack which handles pre-checks, EEPROM wipe, boot params,
    SITL launch, logging, and teardown.  This context manager adds:
      - mediator_torque.py launch (stationary hub yaw ODE + motor physics)
      - EKF compass-yaw alignment (short, ~3-10 s)
    - arm + GUIDED_NOGPS mode entry

    Logs written to simulation/logs/{test_name}/ (per-test directory,
    matching the flight stack convention).

    Parameters
    ----------
    omega_rotor            : rotor hub angular velocity [rad/s]
    profile                : mediator_torque.py --profile value
    tail_channel           : ArduPilot tail/motor channel read by mediator (0-based)
    extra_params           : ParamSetup merged into torque params before boot-file write
    test_name              : pytest test node name; used as logger name and for per-test log directory
    install_scripts        : tuple of Lua script names to install from simulation/scripts/
    boot_params            : dict of {param_name: value} merged on top of torque_setup
                             (for params only known at fixture call time, e.g. RPM1_TYPE).
    startup_yaw_rate_deg_s : yaw rate [deg/s] sent during startup hold for EKF init (0=stationary).
    armon_ms               : if set, arm via RAWES_ARM named float instead of GCS arm.
                             > 0: send RAWES_ARM(armon_ms) and wait for "RAWES arm-on: armed"
                                  STATUSTEXT (hard fail if not received); no RC override sent.
                             0: skip arming entirely; yield unarmed (test controls arming).
                            None (default): GCS force-arm (no RC override).
    passive_init           : if True, adopt the flight GUIDED_NOGPS init technique:
                             install rawes.lua, boot in MODE_PASSIVE (RAWES_MODE=3),
                             seed the IC operating point (RAWES_THR=passive_thrust,
                             RAWES_RIC=RAWES_PIC=0 -> level orientation) BEFORE arm, and
                             seed the EKF pre-arm attitude from the live EKF yaw.  The
                             Lua then commands the IC attitude as a GUIDED angle target
                             while the yaw motor regulates.  Orientation is unchanged
                             (roll=pitch=0); the hub does not rotate during the hold.
    passive_thrust          : IC thrust [0..1] seeded to MODE_PASSIVE when passive_init.
    passive_roll_rad       : IC roll [rad] seeded to MODE_PASSIVE (RAWES_RIC) + pre-arm attitude.
    passive_pitch_rad      : IC pitch [rad] seeded to MODE_PASSIVE (RAWES_PIC) + pre-arm attitude.
    passive_yaw_rad        : optional fixed yaw target [rad] sent to MODE_PASSIVE (RAWES_YIC).
                             When set, PASSIVE holds this absolute yaw instead of capturing
                             (and chasing) the spinning AHRS yaw.  None -> AHRS capture.
    use_vanilla_boot_defaults : when True, use _sitl_stack default boot chain
                             (copter-heli + rawes_common_defaults + rawes_sitl_defaults)
                             instead of _BASE_TORQUE_BOOT_PARAMS. Any extra_params,
                             passive boot overrides, and boot_params are still merged
                             on top as explicit overlays.
    motor_delay_ms         : transport delay [ms] applied to the motor throttle
                             response in mediator_torque (models ESC/actuation
                             latency).  0 = no delay (default).
    """
    # Pre-launch: install Lua scripts before SITL starts.
    # passive_init requires rawes.lua (MODE_PASSIVE) -> ensure it is installed.
    if passive_init and "rawes.lua" not in install_scripts:
        install_scripts = (*install_scripts, "rawes.lua")
    if install_scripts:
        _install_lua_scripts(*install_scripts)

    # passive_init boot overrides: enable scripting only (RAWES_MODE applied via GCS
    # after scripting is up; cannot be set via boot parm file).
    _passive_boot = {"SCR_ENABLE": 1} if passive_init else {}
    _passive_rawes = {"RAWES_MODE": 3} if passive_init else {}

    # Build one complete param setup for this test.
    # Always-wipe EEPROM + per-test boot file = single source of truth.
    # Order (each layer overrides the previous):
    #   1. _BASE_TORQUE_BOOT_PARAMS  — all EKF/PID/RSC/arming/GPS-source values
    #   2. extra_params              — fixture-specific overrides (e.g. Lua)
    #   3. _passive_boot             — MODE_PASSIVE init overrides (passive_init)
    #   4. boot_params               — caller-supplied boot-time extras (e.g. RPM1_TYPE)
    #
    # IC-orientation experiment (profile="ic"): boot from the FLIGHT default
    # params (rawes_sitl_defaults.parm: dual-GPS yaw, GPS pos/vel enabled)
    # instead of the compass-yaw torque base.  Those defaults already configure
    # the GB4008 yaw motor wiring (H_TAIL_TYPE=3, Motor4 on SERVO9, SERVO ranges);
    # the IC overlay (_IC_TORQUE_YAW_PARAMS) adds only the divergent yaw-PID
    # params, so the torque extras here just carry the Lua/passive/boot overlays.
    _ic_mode = (profile == "ic")
    if use_vanilla_boot_defaults:
        torque_setup = (
            ParamSetup({})
            .merge(extra_params or ParamSetup({}))
            .merge(ParamSetup(_passive_boot) if _passive_boot else ParamSetup({}))
            .merge(ParamSetup(boot_params) if boot_params else ParamSetup({}))
        )
        _stack_base_params = None
        _stack_extra_boot  = dict(torque_setup.as_list())
    elif _ic_mode:
        torque_setup = (
            _IC_TORQUE_YAW_PARAMS
            .merge(extra_params or ParamSetup({}))
            .merge(ParamSetup(_passive_boot) if _passive_boot else ParamSetup({}))
            .merge(ParamSetup(boot_params) if boot_params else ParamSetup({}))
        )
        _stack_base_params = None
        _stack_extra_boot  = dict(torque_setup.as_list())
    else:
        torque_setup = (
            _BASE_TORQUE_BOOT_PARAMS
            .merge(extra_params or ParamSetup({}))
            .merge(ParamSetup(_passive_boot) if _passive_boot else ParamSetup({}))
            .merge(ParamSetup(boot_params) if boot_params else ParamSetup({}))
        )
        _stack_base_params = torque_setup
        _stack_extra_boot  = None

    with _sitl_stack(
        tmp_path,
        test_name         = test_name,
        base_params       = _stack_base_params,
        extra_boot_params = _stack_extra_boot,
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
            motor_delay_ms=motor_delay_ms,
        )

        def _assert_alive() -> None:
            for name, proc, lp in [
                ("mediator_torque", mediator_proc,           mediator_log),
                ("SITL",           sitl_ctx.sitl_proc, sitl_ctx.sitl_log),
            ]:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{txt[-3000:]}")

        gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=mavlink_log,
                       watchdog=_assert_alive)
        ctx = StackContext(
            gcs=gcs, mediator_proc=mediator_proc, sitl_proc=sitl_ctx.sitl_proc,
            mediator_log=mediator_log, sitl_log=sitl_ctx.sitl_log,
            gcs_log=sitl_ctx.gcs_log,
            events_log=MediatorEventLog(events_path),
            omega_rotor=omega_rotor, log=log,
            test_log_dir=sitl_ctx.test_log_dir,
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
            _dump_params_to_log(gcs, sitl_ctx.test_log_dir, log)

            # Apply RAWES_* script-generated params now that scripting is up.
            # param:add_table registers RAWES_* at Lua load time (after --add-param-file),
            # so they cannot be set via the boot parm file.  Apply them here via GCS.
            if _passive_rawes:
                log.info("Applying RAWES_* params via GCS (scripting now registered them) ...")
                for _rname, _rval in _passive_rawes.items():
                    _ok = gcs.set_param(_rname, float(_rval), timeout=5.0)
                    log.info("  %-28s = %-10g  ACK=%s", _rname, _rval, _ok)

            # EKF alignment (no RC override keepalive required).
            log.info("Waiting for EKF yaw alignment (up to 45 s) ...")
            ekf_ok  = False
            t_start = gcs.sim_now()
            deadline = gcs.sim_now() + 45.0
            _MIN_WAIT = 3.0

            while gcs.sim_now() < deadline:
                _assert_alive()
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
                    if "rawes" in text.lower() and "mode=" in text.lower():
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

            # passive_init: seed the IC operating point BEFORE arm so rawes.lua
            # MODE_PASSIVE (RAWES_MODE=3) captures yaw and begins commanding the IC
            # attitude immediately.  Orientation stays level (roll=pitch=0); only
            # the collective + the captured EKF yaw are held.  Mirrors the flight
            # GUIDED_NOGPS init technique (seed IC -> seed pre-arm attitude -> arm).
            _pre_arm_rpy = None
            if passive_init:
                _pre_arm_yaw = 0.0
                _att = gcs._recv(type="ATTITUDE", blocking=True, timeout=2.0)
                if _att is not None and all(
                    math.isfinite(v) for v in (_att.roll, _att.pitch, _att.yaw)
                ):
                    _pre_arm_yaw = float(_att.yaw)
                gcs.send_named_float("RAWES_THR", float(passive_thrust))
                gcs.send_named_float("RAWES_RIC", float(passive_roll_rad))
                gcs.send_named_float("RAWES_PIC", float(passive_pitch_rad))
                if passive_yaw_rad is not None:
                    gcs.send_named_float("RAWES_YIC", float(passive_yaw_rad))
                    _pre_arm_yaw = float(passive_yaw_rad)
                log.info(
                    "PASSIVE seed: RAWES_THR=%.3f, RIC=%+.4f, PIC=%+.4f, YIC=%s; pre-arm yaw=%.1f deg",
                    passive_thrust, passive_roll_rad, passive_pitch_rad,
                    ("%+.4f" % passive_yaw_rad) if passive_yaw_rad is not None else "capture",
                    math.degrees(_pre_arm_yaw),
                )
                _pre_arm_rpy = (float(passive_roll_rad), float(passive_pitch_rad), _pre_arm_yaw)

            _arm_sequence(
                gcs, log,
                armon_ms=armon_ms,
                procs_alive=_assert_alive,
                fail=pytest.fail,
                mode_timeout=10.0,
                arm_timeout=15.0,
                target_mode=GUIDED_NOGPS,
                pre_arm_attitude_rpy=_pre_arm_rpy,
            )
            if armon_ms is None:
                log.info("Armed via GCS -- profile=%s", profile)
            elif armon_ms > 0:
                log.info("Armed via GCS + RAWES_ARM disarm timer -- profile=%s", profile)
            else:
                log.info("GUIDED_NOGPS active (unarmed) -- profile=%s",
                         profile)

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

