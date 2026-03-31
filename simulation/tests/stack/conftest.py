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
import sys
import time
from pathlib import Path

import pytest

_SIM_DIR   = Path(__file__).resolve().parents[2]
_STACK_DIR = Path(__file__).resolve().parent

sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_STACK_DIR))

from test_stack_integration import (
    ARDUPILOT_ENV,
    STACK_ENV_FLAG,
    SIM_VEHICLE_ENV,
    _launch_mediator,
    _launch_sitl,
    _resolve_sim_vehicle,
    _terminate_process,
    _kill_by_port,
)
from stack_utils import (
    _configure_logging,
    copy_logs_to_dir,
    check_ports_free,
)
import socket as _socket

from pymavlink import mavutil as _mavutil
from gcs import ACRO, STABILIZE, RawesGCS
from controller import make_hold_controller

_STARTING_STATE   = _SIM_DIR / "steady_state_starting.json"





def pytest_addoption(parser):
    parser.addoption(
        "--sensor-mode",
        default=None,
        choices=["tether_relative", "physical"],
        help="Override sensor mode for all stack tests (default: tether_relative).",
    )

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
    ARM_TIMEOUT           : float = 30.0
    MODE_TIMEOUT          : float = 60.0
    EKF_ALIGN_TIMEOUT     : float = 45.0
    EKF_STABILISE_TIMEOUT : float = 30.0
    # 45 s kinematic startup: hub moves at constant vel0 ≈ 0.96 m/s from
    # launch_pos to equilibrium pos0, providing a non-zero velocity heading
    # so EKF derives yaw via EK3_SRC1_YAW=2 (GPS velocity) from frame 0.
    # The kinematic phase must cover the full GPS fusion timeline:
    #   ~4 s  tiltAlignComplete + yawAlignComplete
    #   ~19 s gpsGoodToAlign (hardcoded 10 s delay from GPS detect at ~9 s)
    #   ~41 s delAngBiasLearned → GPS position fuses → LOCAL_POSITION_NED
    # 45 s gives 4 s margin after GPS fusion before free flight starts.
    # The hub is in stable constant-velocity kinematic throughout; EKF stays
    # healthy (no tether force, no acceleration) and GPS fuses cleanly.
    STARTUP_DAMP_S        : float = 45.0
    LOCK_ORIENTATION      : bool  = False  # kinematic startup sets correct R0; let physics run free
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
    def _check_ports(cls, retry_s: float = 15.0, poll_interval: float = 0.5) -> None:
        """Check that all required ports are free, retrying for up to retry_s seconds.

        A brief retry window handles the case where the previous test's SITL process
        has been sent SIGKILL but the OS hasn't yet released the port.
        """
        import time as _time
        for host, port, proto, hint in cls._PORT_CHECKS:
            kind = _socket.SOCK_STREAM if proto == "tcp" else _socket.SOCK_DGRAM
            deadline = _time.monotonic() + retry_s
            while True:
                s = _socket.socket(_socket.AF_INET, kind)
                s.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
                try:
                    s.bind((host, port))
                    s.close()
                    break   # port is free
                except OSError as exc:
                    s.close()
                    if _time.monotonic() >= deadline:
                        raise RuntimeError(
                            f"{proto.upper()} port {host}:{port} is still in use after "
                            f"{retry_s:.0f}s.\n"
                            f"  Hint: {hint}\n"
                            f"  Original error: {exc}"
                        ) from exc
                    _time.sleep(poll_interval)


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
    home_z_enu     : ENU Z of the home (starting) position [m]
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
    home_z_enu:     float
    flight_events:  dict
    all_statustext: list
    setup_samples:  list      # EKF/ATTITUDE samples collected during setup
    log:            logging.Logger
    sim_dir:        Path
    controller:          object    # TetherRelativeHoldController or PhysicalHoldController
    sensor_mode:         str       # "tether_relative" or "physical"
    internal_controller: bool      # True = mediator runs truth-state controller at 400 Hz


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def sensor_mode(request):
    """
    Sensor model used by the mediator.

    Resolved in priority order:
    1. ``--sensor-mode`` CLI option (``pytest --sensor-mode physical``)
    2. Override in a test module's own ``sensor_mode`` fixture
    3. Default: ``"tether_relative"``

    Choices
    -------
    ``"tether_relative"`` (default) — reports roll=pitch=0 at tether
    equilibrium; compass must be disabled.

    ``"physical"`` — reports actual orbital-frame orientation; GPS + compass
    can be fused normally.
    """
    cli = request.config.getoption("--sensor-mode", default=None)
    return cli if cli is not None else "physical"


@pytest.fixture
def acro_armed(tmp_path, sensor_mode):
    """
    Full ACRO stack fixture: launch → connect → configure → arm → ACRO.

    Yields a StackContext with a live GCS armed in ACRO mode.
    Teardown terminates processes and copies logs to simulation/.
    """
    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run stack integration tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(
            f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} to locate sim_vehicle.py"
        )

    pytest.importorskip("pymavlink")

    # Check for port conflicts before launching — surfaces lingering processes early.
    assert_stack_ports_free()

    repo_root = Path(__file__).resolve().parents[3]
    sim_dir   = repo_root / "simulation"

    # ── Initial state ──────────────────────────────────────────────────────────
    initial_state = None
    home_z_enu    = 12.530
    if _STARTING_STATE.exists():
        initial_state = json.loads(_STARTING_STATE.read_text())
        home_z_enu    = float(initial_state["pos"][2])
        initial_state = dict(initial_state)

    # ── Paths ──────────────────────────────────────────────────────────────────
    mediator_log  = tmp_path / "mediator.log"
    sitl_log      = tmp_path / "sitl.log"
    gcs_log       = tmp_path / "gcs.log"
    telemetry_log = tmp_path / "telemetry.csv"

    # ── Logging ───────────────────────────────────────────────────────────────
    _configure_logging(gcs_log)
    log = logging.getLogger("acro_armed")
    logging.getLogger("gcs").setLevel(logging.DEBUG)

    # ── Hold controller ────────────────────────────────────────────────────────
    # Compute anchor NED in LOCAL_POSITION_NED frame.
    #
    # SITL is launched with --custom-location=51.5074,-0.1278,50,0 which sets
    # HOME = GPS_ORIGIN = NED [0,0,0] relative to the anchor (ENU [0,0,0]).
    # LOCAL_POSITION_NED = pos_ned_absolute - HOME_NED = pos_ned_absolute - 0.
    #
    # sensor.py sends: pos_ned = [pos_enu[1], pos_enu[0], -(pos_enu[2] - home_z)]
    # At anchor ENU=[0,0,0]: pos_ned = [0, 0, home_z_enu]
    # So anchor_ned_LOCAL = [0, 0, home_z_enu]  (NOT [-pos0_y, -pos0_x, pos0_z])
    import numpy as _np
    _anchor_ned = _np.array([0.0, 0.0, float(home_z_enu)])
    controller = make_hold_controller(sensor_mode, anchor_ned=_anchor_ned)
    log.info("Hold controller: %s  (sensor_mode=%s)", type(controller).__name__, sensor_mode)

    # ── Launch ────────────────────────────────────────────────────────────────
    # Emit the same RUN_ID that mediator.py will write as its first log line.
    # analyse_run.py uses these to validate both logs are from the same run.
    import time as _t
    _run_id = int(_t.time())
    log.info("RUN_ID=%d", _run_id)
    log.info("acro_armed fixture: launching mediator + SITL ...")
    mediator_proc = _launch_mediator(
        sim_dir, repo_root, mediator_log,
        telemetry_log_path=str(telemetry_log),
        initial_state=initial_state,
        startup_damp_seconds=_STARTUP_DAMP_S,
        lock_orientation=StackConfig.LOCK_ORIENTATION,
        sensor_mode=sensor_mode,
        run_id=_run_id,
        base_k_ang=(StackConfig.BASE_K_ANG_INTERNAL
                    if StackConfig.INTERNAL_CONTROLLER
                    else StackConfig.BASE_K_ANG),
        internal_controller=StackConfig.INTERNAL_CONTROLLER,
    )
    sitl_proc = _launch_sitl(sim_vehicle, sitl_log)

    def _procs_alive():
        for name, proc, lp in [
            ("mediator", mediator_proc, mediator_log),
            ("SITL",     sitl_proc,     sitl_log),
        ]:
            if proc.poll() is not None:
                txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{txt[-3000:]}")

    gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)

    ctx = StackContext(
        gcs=gcs, mediator_proc=mediator_proc, sitl_proc=sitl_proc,
        mediator_log=mediator_log, sitl_log=sitl_log, gcs_log=gcs_log,
        telemetry_log=telemetry_log, initial_state=initial_state,
        home_z_enu=home_z_enu, flight_events={}, all_statustext=[],
        setup_samples=[], log=log, sim_dir=sim_dir,
        controller=controller, sensor_mode=sensor_mode,
        internal_controller=StackConfig.INTERNAL_CONTROLLER,
    )

    try:
        _run_acro_setup(ctx, _procs_alive)
        yield ctx
    finally:
        gcs.close()
        _terminate_process(sitl_proc)
        _terminate_process(mediator_proc)
        _kill_by_port(StackConfig.SITL_GCS_PORT)
        copy_logs_to_dir(sim_dir / "logs", {
            "telemetry.csv":         telemetry_log,
            "mediator_last_run.log": mediator_log,
            "sitl_last_run.log":     sitl_log,
            "gcs_last_run.log":      gcs_log,
        })


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

    # Send motor interlock HIGH immediately — matches the original GUIDED approach
    # that confirmed arm works.  The "Motor Interlock Enabled" warning is suppressed
    # by force arm; keeping it high avoids any RSC-ready state complications.
    log.info("[setup 1/6] Sending motor interlock HIGH (CH8=2000) ...")
    gcs.send_rc_override({8: 2000})

    # ── 2. Param subsystem ────────────────────────────────────────────────────
    log.info("[setup 2/6] Waiting for param subsystem ...")
    _wait_params_ready(gcs, log)
    _procs_alive()

    # ── 3. ACRO parameters ────────────────────────────────────────────────────
    # ARMING_CHECK=0:  disable ALL pre-arm checks (including "Motor Interlock
    #   Enabled") so force arm succeeds unconditionally.
    #
    # H_RSC_MODE=1 (CH8 Passthrough): RSC output = CH8 input directly.
    #   No ramp / runup sequence — runup_complete fires immediately when CH8
    #   is high.  SITL default (0) is INVALID ("H_RSC_MODE invalid" auto-
    #   disarm).  Modes 2–4 all require a runup wait AND motor interlock ON.
    #   Mode 1 with CH8=2000 gives instant runup_complete so HEARTBEAT shows
    #   armed=True immediately after the force arm ACK.
    #   Note: AP_MotorsHeli::output() clears _flags.armed if runup_complete
    #   is not set, regardless of force arm — this is why CH8 must be HIGH.
    log.info("[setup 3/6] Setting ACRO parameters (sensor_mode=%s) ...", ctx.sensor_mode)
    # Base params: always required regardless of sensor mode.
    _base_params = {
        "ARMING_SKIPCHK":   0xFFFF, # skip ALL pre-arm checks
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
    }
    # Mode-specific params from the controller (e.g. COMPASS_USE=0 for
    # tether_relative; no compass override for physical).
    _acro_params = {**_base_params, **ctx.controller.extra_params}
    for name, value in _acro_params.items():
        ok = gcs.set_param(name, value, timeout=5.0)
        if not ok:
            log.warning("  %s=%s had no ACK — continuing", name, value)

    # GPS position fusion params — applied AFTER ACRO params so they override
    # any conflicting value from the controller's extra_params.
    #
    # COMPASS_USE/ENABLE=1: magnetometer provides yaw for EKF GPS fusion.
    #   Required — EK3_SRC1_YAW=1 needs the compass sensor enabled.
    #
    # EK3_SRC1_YAW=1 (compass yaw):  hub is stationary during the 45 s kinematic
    #   startup (body_z locked to _R0 → constant compass heading).  Compass gives
    #   a stable, direct yaw measurement at 10 Hz, which converges P[12] in ~1 s
    #   after tilt alignment.  P[10,11] then converge passively through EKF
    #   cross-covariance in ~33 s → delAngBiasLearned at ~37 s.
    #   Using EK3_SRC1_YAW=2 (GPS velocity heading) with a non-zero kinematic
    #   velocity (vel0 ≈ 0.96 m/s) caused GPS position fusion to fail: when
    #   delAngBiasLearned fired (~43 s) the hub had moved ~24 m from the GPS
    #   origin, the position+velocity innovations were rejected, and the EKF
    #   went unhealthy (flags → 0x0400) before LOCAL_POSITION_NED was received.
    #
    # EK3_MAG_CAL=0 (Never): use raw compass readings immediately without
    #   in-flight calibration.  Same as stationary GPS test that passed.
    #
    # EK3_GPS_CHECK=0: skip HDOP/speed-accuracy/min-sat checks (SITL JSON GPS
    #   has no real quality fields; default checks block fusion indefinitely).
    _gps_params = {
        "COMPASS_USE":    1,
        "COMPASS_ENABLE": 1,
        "EK3_GPS_CHECK":  0,
        "EK3_SRC1_YAW":  1,    # 1=compass — overrides PhysicalHoldController=2
        "EK3_MAG_CAL":   0,    # 0=Never — use raw compass; no calibration needed
    }
    for name, value in _gps_params.items():
        ok = gcs.set_param(name, value, timeout=5.0)
        if not ok:
            log.warning("  GPS param %s=%s had no ACK — continuing", name, value)

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
        if ctx.sensor_mode == "tether_relative":
            log.warning("[setup 4/6] WARNING: no LOCAL_POSITION_NED during EKF wait "
                        "(EKF horizontal position not yet fused; force arm will proceed)")
        else:
            log.warning("[setup 4/6] WARNING: no LOCAL_POSITION_NED during EKF wait "
                        "(physical sensor with GPS+compass enabled — EKF may still be "
                        "initialising; force arm will proceed)")

    all_statustext += drain_statustext(gcs, log)
    _procs_alive()

    # Stabilisation wait: drain queued STATUSTEXT, watch for tilt/yaw alignment
    # completion, and wait for EKF_STATUS to leave the brief 0x0000 flash
    # (all-zeros means EKF reinitialising).  With ARMING_CHECK=0 + H_RSC_MODE=1
    # + CH8=2000 the arm will succeed as soon as 0x0000 clears.
    # Wait for EKF GPS position fusion (LOCAL_POSITION_NED) before arming.
    #
    # WHY position, not velocity:
    #   In AID_NONE mode the EKF has valid horiz/vert velocity estimates (from
    #   IMU integration + GPS-velocity yaw), so bits 0x0006 (horiz_vel + vert_vel)
    #   appear within ~5 s of tilt alignment — long before GPS position fuses.
    #   Exiting on 0x0006 arms while delAngBiasLearned is still false (~37–42 s
    #   from start), so GPS position never fuses before test_gps_fuses_during_startup
    #   times out.
    #
    # WHAT we wait for:
    #   - LOCAL_POSITION_NED received: EKF is in AID_ABSOLUTE (GPS position fused).
    #     This is the correct signal that readyToUseGPS() returned true.
    #   - Fallback: 50 s after first non-zero EKF flags.  This is longer than the
    #     ~40 s needed for delAngBiasLearned (binding gate); it ensures GPS has time
    #     to fuse before we give up.  For tether_relative mode (no GPS position
    #     expected), this fallback still fires and proceeds to arm.
    #
    # TIMING (physical sensor mode, EK3_SRC1_YAW=2, vel0≈0.96 m/s):
    #   ~4 s  : tiltAlignComplete, yawAlignComplete → EKF flags non-zero
    #   ~19 s : gpsGoodToAlign (hardcoded 10 s delay from GPS detect at ~9 s)
    #   ~41 s : delAngBiasLearned → GPS position fuses → LOCAL_POSITION_NED
    _EKF_POS_FLAG = 0x0010   # horiz_pos_abs: GPS horizontal position fused
    log.info("[setup] Waiting for EKF GPS position fusion before arming (timeout=50s from EKF ready) ...")
    ekf_non_zero_seen = False
    last_flags        = 0
    t_stab = time.monotonic() + 60.0   # hard outer limit (EKF non-zero not seen yet)
    t_stable_since    = None   # first non-zero EKF flags
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
            elif mt == "LOCAL_POSITION_NED":
                log.info("[stabilise] LOCAL_POSITION_NED  N=%.2f  E=%.2f  D=%.2f",
                         msg.x, msg.y, msg.z)
                setup_samples.append({"t": time.monotonic(), "type": "LOCAL_POSITION_NED",
                                       "N": msg.x, "E": msg.y, "D": msg.z,
                                       "vN": msg.vx, "vE": msg.vy, "vD": msg.vz})
                if not ekf_pos:
                    log.info("[stabilise] EKF GPS position fused (LOCAL_POSITION_NED). Proceeding to arm.")
                    ekf_pos = True
            elif mt == "EKF_STATUS_REPORT":
                flags     = msg.flags
                last_flags = flags
                log.info("[stabilise] EKF_STATUS  flags=0x%04x", flags)
                if flags != 0:
                    if t_stable_since is None:
                        t_stable_since = time.monotonic()
                    ekf_non_zero_seen = True
                else:
                    t_stable_since = None  # reset on 0x0000 flash
                if flags & _EKF_POS_FLAG:
                    log.info("[stabilise] horiz_pos_abs set (flags=0x%04x) — GPS position fused.",
                             flags)
        gcs.send_rc_override({8: 2000})
        # Preferred exit: GPS position fused (LOCAL_POSITION_NED received).
        if ekf_pos:
            break
        # Fallback: 50 s after EKF first went non-zero — proceed even if GPS never fused.
        if t_stable_since is not None and (time.monotonic() - t_stable_since) >= 50.0:
            log.warning("[stabilise] GPS position fusion did not occur within 50 s; "
                        "proceeding with flags=0x%04x", last_flags)
            break
    if not ekf_non_zero_seen:
        log.warning("[stabilise] EKF never left 0x0000 state — arming anyway")

    # ── 5. Arm ────────────────────────────────────────────────────────────────
    # H_RSC_MODE=1 (CH8 passthrough): RSC output = CH8 directly.
    # CH8=2000 (motor interlock enabled): RSC immediately at setpoint →
    # runup_complete fires instantly → AP_MotorsHeli keeps _flags.armed=True.
    # ARMING_CHECK=0 bypasses "Motor Interlock Enabled" pre-arm check.
    log.info("[setup 5/6] Arming with motor interlock enabled (CH8=2000) ...")
    gcs.send_rc_override({8: 2000})
    time.sleep(0.3)
    gcs.send_rc_override({8: 2000})
    try:
        gcs.arm(timeout=_ARM_TIMEOUT, force=True, rc_override={8: 2000})
        log.info("[setup 5/6] Armed.")
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
        if ctx.sensor_mode == "tether_relative":
            issues.append(
                "GPS Glitch — EKF sees position inconsistent with IMU. "
                "With tether_relative sensor (artificial attitude) this is usually harmless in ACRO; "
                "if arm fails, try setting EKF2_GPS_TYPE=3 to disable GPS in EKF."
            )
        else:
            issues.append(
                "GPS Glitch — EKF sees position inconsistent with IMU. "
                "With physical sensor, GPS/compass/IMU must all agree — check that "
                "accel_body and gyro_body are correctly expressed in the reported body frame."
            )
    if any("ekf variance" in t.lower() for t in ctx.all_statustext):
        issues.append(
            "EKF variance over threshold — EKF is uncertain. "
            "Common causes: IMU accel not aligned with reported attitude, "
            "or GPS position jumping while IMU says stationary."
        )
    if (ctx.sensor_mode == "tether_relative"
            and any("compass" in t.lower() for t in ctx.all_statustext)):
        issues.append(
            "Compass-related STATUSTEXT — ensure COMPASS_USE=0 is set before EKF init "
            "(tether_relative sensor requires compass disabled)."
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
