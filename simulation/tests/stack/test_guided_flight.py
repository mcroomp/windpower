"""
test_guided_flight.py — GUIDED mode pumping-cycle test using SITL + mediator.

Launches the two-process stack:
    ArduPilot SITL ↔ mediator.py  (internal RK4 dynamics)

Drives ArduPilot through the three RAWES pumping-cycle phases:

  MAINTAIN → REEL-IN → REEL-OUT

Power-optimal geometry
----------------------
  Tether anchor:  ENU (0, 0, 0)   — ground-level attachment point
  Hub home:       ENU (0, 0, 50)  — launch position, directly above anchor
  Wind:           10 m/s East (mediator default)
  Tether length:  50 m (test shortens it so the tether is taut from launch)

  For maximum power during reel-out the elevation angle β is chosen to
  maximise P = T_tether × v_reel × cos(β):

      dP/dβ = 0  →  tan²β = 1/2  →  β_opt = arctan(1/√2) ≈ 35°

  At β = 35° with L = 50 m:
      East of anchor  = L·cos(35°) ≈  41.0 m
      Altitude        = L·sin(35°) ≈  28.7 m  (21.3 m below home altitude)

  This requires the rotor disk to be tilted (cyclic pitch) so the thrust
  vector has a significant horizontal component opposing the tether pull.
  ArduPilot's position controller generates that cyclic automatically when
  commanded to hold this position against the tether.

Three modes (LOCAL_NED relative to home at 50 m above anchor)
--------------------------------------------------------------
  MAINTAIN  (N=0, E=41.0, D=+21.3)  — hold power-optimal tether angle
  REEL-IN   (N=0, E=0,    D=0  )    — fly back over anchor; tether goes slack
  REEL-OUT  (N=0, E=45.1, D=+18.4)  — L×1.1 in same direction; tether stretches

Assertions
----------
  - Both processes stay alive throughout
  - GCS connects, EKF goes healthy, vehicle arms
  - GUIDED mode confirmed by heartbeat
  - Vehicle moves east (tether direction) during maintain phase
  - No CRITICAL errors in mediator log
"""
import json
import logging
import math
import os
import subprocess
import sys
import time
from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

_SIM_DIR        = Path(__file__).resolve().parents[2]
_STACK_DIR      = Path(__file__).resolve().parent
_STARTING_STATE = _SIM_DIR / "steady_state_starting.json"
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_STACK_DIR))

# Reuse helpers from the main stack test module
from test_stack_integration import (
    ARDUPILOT_ENV,
    STACK_ENV_FLAG,
    SIM_VEHICLE_ENV,
    _launch_mediator,
    _launch_sitl,
    _resolve_sim_vehicle,
    _terminate_process,
)

from pymavlink import mavutil as _mavutil

from gcs import GUIDED, RawesGCS
from flight_report import plot_flight_report as _plot_flight_report_base

# ── Pumping-cycle geometry ────────────────────────────────────────────────────
#
# Tether rest length used in the test (mediator --tether-rest-length).
# Hub home is at ENU (0, 0, 50) — directly above the anchor.
# With rest_length = HOME_Z the tether is at its boundary length from launch.
_TETHER_LENGTH_M = 50.0     # m
_HOME_Z_ENU      = 50.0     # m — hub launch altitude above anchor
_WIND_SPEED_MS   = 10.0     # m/s — matches mediator default (--wind-x)

# Power-optimal elevation angle from horizontal.
# Derivation: maximise P = T_t · v_reel · cos(β)
#   dP/dβ = 0  →  tan²β = 1/2  →  β = arctan(1/√2) ≈ 35.26°
_BETA_OPT_RAD = math.atan(1.0 / math.sqrt(2.0))   # ≈ 0.6155 rad ≈ 35.26°

# East of anchor and altitude for the three modes (all in ENU metres from anchor)
_MAINTAIN_EAST_ENU = _TETHER_LENGTH_M * math.cos(_BETA_OPT_RAD)   # ≈ 40.8 m
_MAINTAIN_ALT_ENU  = _TETHER_LENGTH_M * math.sin(_BETA_OPT_RAD)   # ≈ 28.9 m

_REEL_OUT_FACTOR    = 1.1   # stretch tether 10% beyond rest length
_REEL_OUT_EAST_ENU  = _TETHER_LENGTH_M * _REEL_OUT_FACTOR * math.cos(_BETA_OPT_RAD)
_REEL_OUT_ALT_ENU   = _TETHER_LENGTH_M * _REEL_OUT_FACTOR * math.sin(_BETA_OPT_RAD)

# LOCAL_NED relative to home (origin = hub launch position = ENU (0,0,HOME_Z)):
#   East  = same as ENU East
#   Down  = -(alt_above_anchor - HOME_Z)   positive = below home altitude
_MAINTAIN_N  =  0.0
_MAINTAIN_E  = _MAINTAIN_EAST_ENU
_MAINTAIN_D  = -(_MAINTAIN_ALT_ENU - _HOME_Z_ENU)     # positive → hub is below home

_REEL_OUT_N  =  0.0
_REEL_OUT_E  = _REEL_OUT_EAST_ENU
_REEL_OUT_D  = -(_REEL_OUT_ALT_ENU - _HOME_Z_ENU)

_REEL_IN_N   =  0.0          # return to directly above anchor
_REEL_IN_E   =  0.0
_REEL_IN_D   =  0.0          # back to home altitude

# ── Timing ───────────────────────────────────────────────────────────────────
_STARTUP_TIMEOUT     = 30.0   # s — processes start + MAVLink heartbeat
_ARM_TIMEOUT         = 30.0   # s — arm confirmation
_MODE_TIMEOUT        = 60.0   # s — GUIDED mode confirmation
_MAINTAIN_SECONDS    = 40.0   # s — hold power-optimal position
_REEL_IN_SECONDS     = 20.0   # s — fly toward anchor (slack tether)
_REEL_OUT_SECONDS    = 40.0   # s — fly away from anchor (stretch tether)

# ── Tolerances ───────────────────────────────────────────────────────────────
# During maintain phase the hub must move at least this far East toward the
# power-optimal position (tether pulling hub East and down to 35° angle).
_MIN_EAST_DISPLACEMENT = 2.0   # m

# ── Position logging interval ─────────────────────────────────────────────────
_POS_LOG_INTERVAL    =  5.0   # s — print position summary this often


def _configure_logging(log_file: Path) -> None:
    """
    Direct Python logging to both the test log file and pytest's captured stdout.

    - gcs module: DEBUG (captures EKF flag progression, param ACKs, etc.)
    - everything else: INFO
    """
    fmt = "%(asctime)s %(name)-20s %(levelname)-8s %(message)s"
    datefmt = "%H:%M:%S"

    # File handler — full DEBUG output for gcs, INFO for rest
    fh = logging.FileHandler(str(log_file), encoding="utf-8")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(fmt, datefmt))

    # Stream handler — INFO for everything (shows in pytest -s / on failure)
    sh = logging.StreamHandler(sys.stdout)
    sh.setLevel(logging.INFO)
    sh.setFormatter(logging.Formatter(fmt, datefmt))

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    # Avoid duplicate handlers if pytest re-uses the process
    root.handlers.clear()
    root.addHandler(fh)
    root.addHandler(sh)

    # gcs module gets DEBUG so EKF flag progression is visible in the log file
    logging.getLogger("gcs").setLevel(logging.DEBUG)


def _wait_params_ready(gcs: RawesGCS, log: logging.Logger, timeout: float = 15.0) -> None:
    """
    Block until ArduPilot's parameter subsystem is accepting requests.

    Sends a PARAM_REQUEST_READ for "SYSID_THISMAV" (always present) and
    waits for any PARAM_VALUE reply.  This is more reliable than a fixed
    sleep because it confirms the flight controller has finished its boot
    initialisation and is processing MAVLink param commands.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        gcs._mav.mav.param_request_read_send(
            gcs._target_system,
            gcs._target_component,
            b"SYSID_THISMAV",
            -1,   # param_index = -1 → look up by name
        )
        msg = gcs._mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is not None:
            log.info("Param subsystem ready (first response: %s = %g)",
                     msg.param_id.rstrip("\x00"), msg.param_value)
            return
        log.debug("Waiting for param subsystem ...")
    raise TimeoutError(f"ArduPilot param subsystem not ready after {timeout:.0f}s")


def _drain_statustext(gcs: RawesGCS, log: logging.Logger) -> list[str]:
    """
    Non-blocking drain of queued STATUSTEXT messages.

    pymavlink buffers messages that don't match the type filter, so calling
    recv_match for a different type does not discard already-queued messages.
    Returns a list of drained text strings (also logs each one at WARNING).
    """
    texts = []
    while True:
        msg = gcs._mav.recv_match(type="STATUSTEXT", blocking=True, timeout=0.05)
        if msg is None:
            break
        text = msg.text.rstrip("\x00").strip()
        severity = getattr(msg, "severity", "?")
        log.warning("STATUSTEXT [sev=%s] %s", severity, text)
        texts.append(text)
    return texts


def _plot_flight_report(
    pos_history:      list,
    attitude_history: list,
    servo_history:    list,
    events:           dict,
    target:           tuple,
    out_path:         Path,
    telemetry_path:   Path | None = None,
) -> None:
    _plot_flight_report_base(
        pos_history      = pos_history,
        attitude_history = attitude_history,
        servo_history    = servo_history,
        events           = events,
        target           = target,
        out_path         = out_path,
        telemetry_path   = telemetry_path,
        tether_length_m  = _TETHER_LENGTH_M,
    )



def test_guided_flight_to_tether_position():
    """
    End-to-end GUIDED pumping-cycle test: arm → GUIDED → maintain → reel-in → reel-out.

    Phases:
      1. MAINTAIN  — hold power-optimal 35° tether position (E≈41 m, below home)
      2. REEL-IN   — fly back over anchor; tether goes slack for winch reel-in
      3. REEL-OUT  — fly to 1.1× tether length at 35°; tether under tension

    Asserts the hub moves East toward the power-optimal position during
    the maintain phase, proving ArduPilot is driving cyclic to hold the
    correct kite angle against tether tension.
    """
    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run stack integration tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(
            f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} to locate sim_vehicle.py"
        )

    pytest.importorskip("pymavlink")

    repo_root = Path(__file__).resolve().parents[3]
    sim_dir   = repo_root / "simulation"

    with TemporaryDirectory(prefix="rawes_guided_") as tmpdir:
        temp_path    = Path(tmpdir)
        mediator_log  = temp_path / "mediator.log"
        sitl_log      = temp_path / "sitl.log"
        gcs_log       = temp_path / "gcs.log"
        telemetry_log = temp_path / "telemetry.csv"

        _configure_logging(gcs_log)
        log = logging.getLogger("test_guided_flight")

        log.info("─── test_guided_flight_to_tether_position starting ───")
        log.info(
            "Pumping cycle geometry: tether=%.0fm  β_opt=%.1f°  wind=%.1f m/s East",
            _TETHER_LENGTH_M, math.degrees(_BETA_OPT_RAD), _WIND_SPEED_MS,
        )
        log.info(
            "  MAINTAIN  NED=(%.1f, %.1f, %.1f)   ENU East=%.1fm  Alt=%.1fm",
            _MAINTAIN_N, _MAINTAIN_E, _MAINTAIN_D,
            _MAINTAIN_EAST_ENU, _MAINTAIN_ALT_ENU,
        )
        log.info(
            "  REEL-OUT  NED=(%.1f, %.1f, %.1f)   ENU East=%.1fm  Alt=%.1fm",
            _REEL_OUT_N, _REEL_OUT_E, _REEL_OUT_D,
            _REEL_OUT_EAST_ENU, _REEL_OUT_ALT_ENU,
        )
        log.info("  REEL-IN   NED=(0.0, 0.0, 0.0)   return to home above anchor")

        # ── Load steady-state starting conditions ─────────────────────────────
        _initial_state = None
        if _STARTING_STATE.exists():
            _initial_state = json.loads(_STARTING_STATE.read_text())
            log.info("Loaded initial state from %s  pos=%s  omega_spin=%.2f rad/s",
                     _STARTING_STATE.name,
                     [round(v, 3) for v in _initial_state["pos"]],
                     _initial_state["omega_spin"])
        else:
            log.warning("steady_state_starting.json not found — using mediator defaults. "
                        "Run test_steady_flight.py first to generate it.")

        # ── Launch order: mediator → SITL ─────────────────────────────────────
        log.info("[1/9] Launching mediator ...")
        mediator_proc = _launch_mediator(
            sim_dir, repo_root,
            mediator_log,
            telemetry_log_path=str(telemetry_log),
            tether_rest_length=_TETHER_LENGTH_M,
            initial_state=_initial_state,
        )
        log.info("[1/9] Launching SITL ...")
        sitl_proc = _launch_sitl(sim_vehicle, sitl_log)

        def _procs_alive():
            for name, proc, log_path in [
                ("mediator", mediator_proc, mediator_log),
                ("SITL",     sitl_proc,     sitl_log),
            ]:
                if proc.poll() is not None:
                    log_text = (
                        log_path.read_text(encoding="utf-8", errors="replace")
                        if log_path.exists() else "(no log)"
                    )
                    pytest.fail(
                        f"{name} exited early (rc={proc.returncode}):\n{log_text[-4000:]}"
                    )

        def _dump_logs(label: str = "post-test") -> None:
            """Print all logs to stdout (captured by pytest, shown on failure)."""
            print(f"\n{'='*70}")
            print(f"=== LOG DUMP ({label}) ===")
            print(f"{'='*70}")

            # GCS / test Python logging
            if gcs_log.exists():
                text = gcs_log.read_text(encoding="utf-8", errors="replace")
                print(f"\n--- gcs.log ({len(text)} chars) ---\n{text}")

            # Mediator — full log, it's the most important for physics debugging
            if mediator_log.exists():
                text = mediator_log.read_text(encoding="utf-8", errors="replace")
                print(f"\n--- mediator.log ({len(text)} chars, last 5000) ---\n{text[-5000:]}")

            # SITL — usually large; tail only
            if sitl_log.exists():
                text = sitl_log.read_text(encoding="utf-8", errors="replace")
                print(f"\n--- sitl.log ({len(text)} chars, last 3000) ---\n{text[-3000:]}")

            # ArduCopter.log — only the portion written by this test
            if _ap_log.exists():
                with _ap_log.open("rb") as f:
                    f.seek(_ap_log_start)
                    ap_text = f.read().decode("utf-8", errors="replace")
                print(f"\n--- ArduCopter.log (this test: {len(ap_text)} chars) ---\n{ap_text}")

        # ── Telemetry history ─────────────────────────────────────────────────
        # All times are seconds since position target was sent (t_obs_start).
        pos_history:      list[tuple[float, float, float, float]] = []  # (t, N, E, D)
        attitude_history: list[tuple[float, float, float, float]] = []  # (t, roll°, pitch°, yaw°)
        servo_history:    list[tuple[float, int, int, int, int]]  = []  # (t, s1, s2, s3, s4 µs)

        # ── Key event wall-clock times (converted to obs-relative later) ───
        _t_ekf_lock:  float | None = None
        _t_armed:     float | None = None
        _t_guided:    float | None = None
        flight_events: dict[str, float] = {}

        # Record how many bytes were already in the ArduCopter log before this
        # test started so we only dump what *this* test generated.
        _ap_log = Path("/tmp/ArduCopter.log")
        _ap_log_start = _ap_log.stat().st_size if _ap_log.exists() else 0

        gcs = RawesGCS(address="tcp:127.0.0.1:5760")
        all_statustext: list[str] = []

        try:
            # ── 1. Connect and start GCS heartbeat ───────────────────────────
            log.info("[2/9] Connecting GCS (timeout=%.0fs) ...", _STARTUP_TIMEOUT)
            gcs.connect(timeout=_STARTUP_TIMEOUT)
            gcs.start_heartbeat(rate_hz=1.0)
            log.info("[2/9] GCS connected and heartbeat started.")

            # ── 2. Drain any early STATUSTEXT ─────────────────────────────────
            all_statustext += _drain_statustext(gcs, log)

            # ── 3. Wait for param subsystem, then set arming params ───────────
            # ArduPilot SITL sends a heartbeat before its param subsystem is
            # ready.  Probe with PARAM_REQUEST_READ until we get a PARAM_VALUE
            # back — that confirms the flight controller has finished booting.
            log.info("[3/9] Waiting for param subsystem (timeout=15s) ...")
            try:
                _wait_params_ready(gcs, log, timeout=15.0)
            except TimeoutError as exc:
                log.warning("[3/9] %s — attempting param set anyway", exc)

            # ARMING_CHECK=0   — disable all pre-arm safety checks
            # H_RSC_MODE=2     — PassThrough (valid value; RAWES rotor is wind-driven)
            # RC8_OPTION=32    — keep Motor Interlock on CH8 (required to be configured);
            #                    we release it via RC_CHANNELS_OVERRIDE below rather than
            #                    removing the assignment (RC8_OPTION=0 causes "not configured"
            #                    error that blocks even force arm)
            for param, value in [
                ("ARMING_CHECK",     0),  # disable pre-arm safety checks (float send; may not ACK but helps)
                ("H_RSC_MODE",       2),  # PassThrough (already default; confirm it)
                ("H_RSC_RUNUP_TIME", 0),  # 0 → RSC treats runup as always complete; needed for GUIDED
                ("RC8_OPTION",      32),  # keep Motor Interlock on CH8 (released via override below)
            ]:
                log.info("[3/9] Setting %s=%s ...", param, value)
                ok = gcs.set_param(param, value, timeout=5.0)
                if not ok:
                    log.warning("  %s set had no ACK — continuing anyway", param)
            all_statustext += _drain_statustext(gcs, log)

            # ── 4. Request data streams + wait for valid position estimate ────
            # LOCAL_POSITION_NED is not forwarded by default on TCP 5760.
            # Request all standard streams so ArduPilot sends position,
            # EKF status, and other telemetry.
            log.info("[4/9] Requesting data streams ...")
            gcs.request_stream(_mavutil.mavlink.MAV_DATA_STREAM_POSITION,       rate_hz=4)
            gcs.request_stream(_mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,         rate_hz=4)   # ATTITUDE
            gcs.request_stream(_mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, rate_hz=4)   # SERVO_OUTPUT_RAW
            gcs.request_stream(_mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,         rate_hz=4)
            gcs.request_stream(_mavutil.mavlink.MAV_DATA_STREAM_ALL,            rate_hz=2)

            # Fast EKF GPS lock: disable compass temporarily so the EKF initialises
            # on GPS alone (< 1 s) rather than waiting for compass yaw alignment
            # (~40 s).  Once the EKF has position we re-enable compass so it fuses
            # in during flight for improved yaw accuracy.
            log.info("[4/9] Disabling compass for fast GPS lock ...")
            ok = gcs.set_param("COMPASS_USE", 0, timeout=5.0)
            if not ok:
                log.warning("  COMPASS_USE=0 had no ACK — continuing anyway")

            log.info("[4/9] Waiting for EKF position lock (timeout=60s) ...")
            deadline_ekf = time.monotonic() + 60.0
            _last_ekf_log = time.monotonic()
            while time.monotonic() < deadline_ekf:
                _procs_alive()
                # Accept either LOCAL_POSITION_NED or EKF_STATUS_REPORT with pos flags
                msg = gcs._mav.recv_match(
                    type=["LOCAL_POSITION_NED", "EKF_STATUS_REPORT"],
                    blocking=True, timeout=1.0,
                )
                if msg is not None:
                    mt = msg.get_type()
                    if mt == "LOCAL_POSITION_NED":
                        log.info("[4/9] Got LOCAL_POSITION_NED: N=%.2f E=%.2f D=%.2f — EKF ready.",
                                 msg.x, msg.y, msg.z)
                        break
                    elif mt == "EKF_STATUS_REPORT":
                        # Log EKF flags periodically
                        if time.monotonic() - _last_ekf_log > 5.0:
                            log.info("[4/9] EKF_STATUS_REPORT flags=0x%04x "
                                     "(need POS_HORIZ_ABS=0x%02x)", msg.flags,
                                     _mavutil.mavlink.EKF_POS_HORIZ_ABS)
                            _last_ekf_log = time.monotonic()
                        _EKF_POS = (
                            _mavutil.mavlink.EKF_POS_HORIZ_REL
                            | _mavutil.mavlink.EKF_POS_HORIZ_ABS
                        )
                        if msg.flags & _EKF_POS:
                            log.info("[4/9] EKF has position (flags=0x%04x) — EKF ready.",
                                     msg.flags)
                            _t_ekf_lock = time.monotonic()
                            break
                all_statustext += _drain_statustext(gcs, log)
            else:
                log.warning("[4/9] EKF position not seen after 60s — attempting GUIDED anyway")
            _procs_alive()

            # Re-enable compass now that EKF has position — compass fuses in
            # during flight to provide yaw redundancy alongside GPS.
            log.info("[4/9] Re-enabling compass (COMPASS_USE=1) ...")
            ok = gcs.set_param("COMPASS_USE", 1, timeout=5.0)
            if not ok:
                log.warning("  COMPASS_USE=1 had no ACK — continuing anyway")

            # ── 4b. Release motor interlock via RC override ───────────────────
            # ArduPilot helicopter requires CH8 > 1800 to release the motor interlock.
            # ARMING_CHECK=0 disables the pre-arm check but the interlock must still
            # be in a "released" state.  Send override so ArduPilot sees CH8=2000.
            log.info("[4/9] Releasing motor interlock: RC CH8 = 2000 ...")
            gcs.send_rc_override({8: 2000})
            time.sleep(0.3)
            gcs.send_rc_override({8: 2000})

            # ── 5. Arm ────────────────────────────────────────────────────────
            log.info("[5/9] Arming (force=True, timeout=%.0fs) ...", _ARM_TIMEOUT)
            try:
                # Pass rc_override so arm() keeps CH8 high while waiting for confirmation.
                # ArduPilot expires RC overrides after ~1s if not refreshed.
                gcs.arm(timeout=_ARM_TIMEOUT, force=True, rc_override={8: 2000})
                _t_armed = time.monotonic()
                log.info("[5/9] Vehicle armed.")
            except Exception as exc:
                all_statustext += _drain_statustext(gcs, log)
                log.error("[5/9] Arm failed: %s", exc)
                log.error("STATUSTEXT messages captured so far:\n%s",
                          "\n".join(all_statustext) if all_statustext else "(none)")
                raise
            all_statustext += _drain_statustext(gcs, log)
            _procs_alive()

            # ── 5b. Brief pause for RSC to process armed state ────────────────
            # With H_RSC_RUNUP_TIME=0 the RSC declares runup complete within
            # one scheduler tick (~2.5 ms) but EKF alignment and position lock
            # can take several more seconds.  Wait 5 s and keep refreshing the
            # RC override so ArduPilot does not expire the motor interlock.
            log.info("[5/9] Waiting 5s for RSC runup and EKF position lock ...")
            t_wait_end = time.monotonic() + 5.0
            while time.monotonic() < t_wait_end:
                gcs.send_rc_override({8: 2000})
                time.sleep(0.4)
            all_statustext += _drain_statustext(gcs, log)
            _procs_alive()

            # ── 6. Set GUIDED mode ────────────────────────────────────────────
            log.info("[6/9] Setting GUIDED mode (mode_id=%d, timeout=%.0fs) ...",
                     GUIDED, _MODE_TIMEOUT)
            try:
                gcs.set_mode(GUIDED, timeout=_MODE_TIMEOUT, rc_override={8: 2000})
                _t_guided = time.monotonic()
                log.info("[6/9] GUIDED mode confirmed.")
            except Exception as exc:
                all_statustext += _drain_statustext(gcs, log)
                log.error("[6/9] Mode set failed: %s", exc)
                raise
            all_statustext += _drain_statustext(gcs, log)
            _procs_alive()

            # ── 7. Pumping-cycle phases ───────────────────────────────────────
            #
            # Helper: run one phase by holding a position target for duration_s,
            # collecting MAVLink messages into the shared history lists.
            def _run_phase(phase_name, north, east, down, duration_s, t_phase_start):
                deadline_phase = time.monotonic() + duration_s
                t_last_target  = time.monotonic() - 3.0   # force immediate send
                t_last_log     = time.monotonic()
                log.info(
                    "[8/9] Phase %-10s  target NED=(%.1f, %.1f, %.1f)  duration=%.0fs",
                    phase_name, north, east, down, duration_s,
                )
                while time.monotonic() < deadline_phase:
                    _procs_alive()
                    msg = gcs._mav.recv_match(
                        type=["LOCAL_POSITION_NED", "ATTITUDE",
                              "SERVO_OUTPUT_RAW", "STATUSTEXT"],
                        blocking=True, timeout=0.2,
                    )
                    t_rel = time.monotonic() - t_phase_start
                    if msg is not None:
                        mt = msg.get_type()
                        if mt == "LOCAL_POSITION_NED":
                            pos_history.append((t_rel, msg.x, msg.y, msg.z))
                        elif mt == "ATTITUDE":
                            attitude_history.append((
                                t_rel,
                                math.degrees(msg.roll),
                                math.degrees(msg.pitch),
                                math.degrees(msg.yaw),
                            ))
                        elif mt == "SERVO_OUTPUT_RAW":
                            servo_history.append((
                                t_rel,
                                msg.servo1_raw, msg.servo2_raw,
                                msg.servo3_raw, msg.servo4_raw,
                            ))
                        elif mt == "STATUSTEXT":
                            text = msg.text.rstrip("\x00").strip()
                            log.warning("STATUSTEXT [%s] %s", phase_name, text)
                            all_statustext.append(text)
                    now = time.monotonic()
                    # Re-send target every 2 s to prevent GUIDED timeout
                    if now - t_last_target >= 2.0:
                        gcs.send_position_target_ned(north=north, east=east, down=down)
                        t_last_target = now
                    # Periodic summary
                    if now - t_last_log >= _POS_LOG_INTERVAL and pos_history:
                        _, n, e, d = pos_history[-1]
                        log.info(
                            "  %-10s  t=%.0fs  NED=(%.2f, %.2f, %.2f)  "
                            "remaining=%.0fs",
                            phase_name, now - t_phase_start, n, e, d,
                            deadline_phase - now,
                        )
                        t_last_log = now

            t_obs_start = time.monotonic()

            # Convert pre-obs wall-clock events to obs-relative seconds
            flight_events["Position target"] = 0.0
            if _t_ekf_lock is not None:
                flight_events["EKF lock"] = _t_ekf_lock - t_obs_start
            if _t_armed is not None:
                flight_events["Armed"] = _t_armed - t_obs_start
            if _t_guided is not None:
                flight_events["GUIDED"] = _t_guided - t_obs_start

            # Phase 1: MAINTAIN — hold power-optimal 35° tether angle
            _run_phase("MAINTAIN", _MAINTAIN_N, _MAINTAIN_E, _MAINTAIN_D,
                       _MAINTAIN_SECONDS, t_obs_start)
            flight_events["Reel-in"] = time.monotonic() - t_obs_start

            # Phase 2: REEL-IN — fly back over anchor, tether goes slack
            _run_phase("REEL-IN", _REEL_IN_N, _REEL_IN_E, _REEL_IN_D,
                       _REEL_IN_SECONDS, t_obs_start)
            flight_events["Reel-out"] = time.monotonic() - t_obs_start

            # Phase 3: REEL-OUT — fly to 1.1× tether length, stretch for tension
            _run_phase("REEL-OUT", _REEL_OUT_N, _REEL_OUT_E, _REEL_OUT_D,
                       _REEL_OUT_SECONDS, t_obs_start)

            # Summary
            log.info("[8/9] All phases complete.  %d position readings collected.",
                     len(pos_history))
            if pos_history:
                east_vals  = [r[2] for r in pos_history]
                down_vals  = [r[3] for r in pos_history]
                log.info(
                    "  East: min=%.2f  max=%.2f  final=%.2f m  (maintain target=%.1f m)",
                    min(east_vals), max(east_vals), east_vals[-1], _MAINTAIN_E,
                )
                log.info(
                    "  Down: min=%.2f  max=%.2f  final=%.2f m  (maintain target=%.1f m)",
                    min(down_vals), max(down_vals), down_vals[-1], _MAINTAIN_D,
                )

            # ── 9. Assertions ─────────────────────────────────────────────────
            log.info("[9/9] Running assertions ...")

            assert pos_history, (
                "No LOCAL_POSITION_NED messages received during observation window.\n"
                "Possible causes:\n"
                "  - mediator is not running or crashed\n"
                "  - mediator is not sending JSON state back to SITL (port 9003)\n"
                "  - ArduPilot is not forwarding LOCAL_POSITION_NED on MAVLink\n"
                f"STATUSTEXT messages seen: {all_statustext}"
            )

            east_vals = [r[2] for r in pos_history]
            max_east  = max(east_vals)
            assert max_east >= _MIN_EAST_DISPLACEMENT, (
                f"Hub did not move East toward the power-optimal kite position:\n"
                f"  max East reached     : {max_east:.2f} m\n"
                f"  minimum required     : {_MIN_EAST_DISPLACEMENT:.1f} m\n"
                f"  maintain target East : {_MAINTAIN_E:.1f} m  (tether at 35°)\n"
                f"  readings collected   : {len(pos_history)}\n"
                f"  East trajectory      : {[round(v,2) for v in east_vals[::max(1,len(east_vals)//20)]]}\n"
                f"STATUSTEXT messages seen:\n"
                + ("\n".join(f"  {t}" for t in all_statustext) if all_statustext else "  (none)")
            )

            if mediator_log.exists():
                med_text = mediator_log.read_text(encoding="utf-8", errors="replace")
                critical_lines = [l for l in med_text.splitlines() if "CRITICAL" in l]
                assert not critical_lines, (
                    "CRITICAL-level errors in mediator log:\n"
                    + "\n".join(critical_lines[:10])
                )

            # Copy telemetry CSV to sim_dir for post-run inspection
            if telemetry_log.exists():
                import shutil as _shutil
                _shutil.copy2(telemetry_log, sim_dir / "telemetry.csv")
                log.info("Telemetry CSV → %s", sim_dir / "telemetry.csv")

            # Save raw flight data so the graph can be redrawn without re-running
            import json as _json
            flight_data = {
                "pos_history":      [list(r) for r in pos_history],
                "attitude_history": [list(r) for r in attitude_history],
                "servo_history":    [list(r) for r in servo_history],
                "events":           flight_events,
                "target":           [_MAINTAIN_N, _MAINTAIN_E, _MAINTAIN_D],
            }
            flight_data_path = sim_dir / "flight_data.json"
            with flight_data_path.open("w", encoding="utf-8") as _f:
                _json.dump(flight_data, _f)
            log.info("Flight data → %s", flight_data_path)

            _plot_flight_report(
                pos_history      = pos_history,
                attitude_history = attitude_history,
                servo_history    = servo_history,
                events           = flight_events,
                target           = (_MAINTAIN_N, _MAINTAIN_E, _MAINTAIN_D),
                out_path         = sim_dir / "flight_report.png",
                telemetry_path   = telemetry_log,
            )

            log.info("─── test_guided_flight_to_tether_position PASSED (maintain→reel-in→reel-out) ───")

        except Exception:
            log.exception("Test failed — see log dump below")
            _dump_logs(label="FAILURE")
            raise
        finally:
            gcs.close()
            _terminate_process(sitl_proc)
            _terminate_process(mediator_proc)
            # Always dump logs so a passing run is also inspectable
            _dump_logs(label="post-test")
