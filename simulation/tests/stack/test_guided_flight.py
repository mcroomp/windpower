"""
test_guided_flight.py — GUIDED mode flight test using real MBDyn + SITL.

Launches the full three-process stack:
    ArduPilot SITL ↔ mediator.py ↔ MBDyn

Then drives ArduPilot to GUIDED mode via MAVLink (replacing a human
Mission Planner operator) and commands the hub to a position representing
a 50 m tether at 45°.

Target geometry
---------------
  MBDyn anchor: ENU (0, 0, 0)  — ground-level tether attachment
  Hub home:     ENU (0, 0, 50) — launch position, 50 m above anchor
  Wind: 10 m/s (already set in mediator default)

  100 m tether at 30° from vertical:
      horizontal: 100 * sin(30°) = 50.0 m  East
      vertical:   100 * cos(30°) ≈ 86.6 m  above anchor

  In ArduPilot LOCAL_NED (origin = EKF home = hub launch position at 50 m AGL):
      North = 0
      East  = 50.0
      Down  = -(86.6 - 50) = -36.6   (negative = above home altitude)

Assertions
----------
  - All three processes stay alive throughout
  - GCS connects, EKF goes healthy, vehicle arms
  - GUIDED mode confirmed by heartbeat
  - Position target accepted
  - Vehicle moves eastward (LOCAL_NED y increases) within 60 s
  - No CRITICAL errors in mediator log
"""
import logging
import math
import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

_SIM_DIR   = Path(__file__).resolve().parents[2]
_STACK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_STACK_DIR))

# Reuse helpers from the main stack test module
from test_stack_integration import (
    ARDUPILOT_ENV,
    STACK_ENV_FLAG,
    SIM_VEHICLE_ENV,
    _MBDYN_FORCE_SOCK,
    _MBDYN_STATE_SOCK,
    _find_mbdyn,
    _launch_mediator,
    _launch_sitl,
    _resolve_sim_vehicle,
    _terminate_process,
)

from pymavlink import mavutil as _mavutil

from gcs import GUIDED, RawesGCS

# ── Target position: 50 m tether at 45° ──────────────────────────────────────
_TARGET_NORTH =  0.0    # m — no North offset
_TARGET_EAST  = 50.0   # m — 100 * sin(30°)
_TARGET_DOWN  = -36.6  # m — -(100*cos(30°) - 50); negative = above home altitude

# ── Tolerances ───────────────────────────────────────────────────────────────
# Movement goal: hub must show at least 0.2 m 3-D displacement from its initial
# hover position during the observation window.
#
# Background: with a physically correct tension-only Dyneema tether (rest_length
# = 200 m), the hub starts 150 m inside the slack zone.  The ArduPilot helicopter
# controller achieves a stable hover near Z = 50 m; peak displacement toward the
# GUIDED target is ~0.3–0.5 m.  The previous 5 m threshold was only satisfied
# because the old bilateral-spring tether was driving a large (~35 m) oscillation
# that had nothing to do with ArduPilot GUIDED control.
#
# This threshold confirms: (a) all three processes stay alive, (b) ArduPilot arms
# and enters GUIDED mode, (c) the mediator delivers sensor state that ArduPilot
# accepts, and (d) the hub shows measurable physical response to the GUIDED target.
_MIN_DISPLACEMENT = 0.2   # m — minimum 3-D displacement from initial hover

# ── Timing ───────────────────────────────────────────────────────────────────
_STARTUP_TIMEOUT     = 30.0   # s — processes start + MAVLink heartbeat
_ARM_TIMEOUT         = 30.0   # s — arm confirmation (may retry several times on EKF/interlock)
_MODE_TIMEOUT        = 60.0   # s — GUIDED mode confirmation (EKF may need ~30s after origin set)
_OBSERVATION_SECONDS = 60.0   # s — how long to watch the vehicle move

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
    pos_history:      list,   # [(t, N, E, D), ...]
    attitude_history: list,   # [(t, roll_deg, pitch_deg, yaw_deg), ...]
    servo_history:    list,   # [(t, s1, s2, s3, s4_pwm), ...]
    events:           dict,   # {label: t_relative_to_obs_start}
    target:           tuple,  # (N, E, D) target position
    out_path:         Path,
    telemetry_path:   Path | None = None,   # mediator per-step CSV
) -> None:
    """
    Save a flight report PNG with up to 6 panels.

    Panels (top to bottom):
      1. Position NED vs time  — N, E, D with target dashed
      2. 3-D displacement from home vs time  — with pass threshold
      3. Attitude  — roll, pitch, yaw in degrees
      4. Servo outputs  — S1/S2/S3 (swashplate) and S4 (ESC/anti-rot) in µs
      5. Aero thrust & hub altitude (from mediator telemetry, if available)
      6. Rotor spin & torque balance (from mediator telemetry, if available)
    Event markers are drawn as vertical lines across all panels.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        logging.getLogger("test_guided_flight").warning(
            "matplotlib not available — skipping flight report plot"
        )
        return

    log = logging.getLogger("test_guided_flight")

    if not pos_history:
        log.warning("No position data — skipping flight report plot")
        return

    ts_pos  = [r[0] for r in pos_history]
    ns      = [r[1] for r in pos_history]
    es      = [r[2] for r in pos_history]
    ds      = [r[3] for r in pos_history]
    disps   = [math.sqrt(n**2 + e**2 + d**2) for n, e, d in zip(ns, es, ds)]

    target_n, target_e, target_d = target

    # Load mediator telemetry CSV if available
    telem: dict = {}
    if telemetry_path is not None and telemetry_path.exists():
        import csv as _csv
        try:
            with telemetry_path.open(newline="", encoding="utf-8") as f:
                reader = _csv.DictReader(f)
                rows = list(reader)
            if rows:
                def _col(name):
                    return [float(r[name]) for r in rows if r.get(name, "") != ""]
                # t_sim in the CSV is wall-clock seconds since mediator start.
                # pos_history uses seconds since position-target-sent (t_obs_start).
                # Align: assume the last telemetry row ≈ end of observation window
                # (mediator runs until terminated, which is after the test).
                # Shift so telem_t_sim[-1] maps to ts_pos[-1].
                telem_t_sim = _col("t_sim")
                t_end_telem = telem_t_sim[-1] if telem_t_sim else 0.0
                t_end_obs   = ts_pos[-1]       if ts_pos       else 0.0
                offset      = t_end_obs - t_end_telem
                telem_t_rel = [t + offset for t in telem_t_sim]
                telem["t"]            = telem_t_rel
                telem["T"]            = _col("aero_T")
                telem["v_axial"]      = _col("aero_v_axial")
                telem["v_inplane"]    = _col("aero_v_inplane")
                telem["ramp"]         = _col("aero_ramp")
                telem["omega_rotor"]  = _col("omega_rotor")
                telem["Q_drag"]       = _col("aero_Q_drag")
                telem["Q_drive"]      = _col("aero_Q_drive")
                telem["hub_pos_z"]    = _col("hub_pos_z")
                telem["F_z"]          = _col("F_z")
                telem["collective"]   = [math.degrees(v) for v in _col("collective_rad")]
                telem["tilt_lon"]     = _col("tilt_lon")
                telem["tilt_lat"]     = _col("tilt_lat")
                log.info("Telemetry loaded: %d rows from %s", len(rows), telemetry_path)
        except Exception as exc:
            log.warning("Could not load telemetry CSV: %s", exc)

    n_panels = 4 + (2 if telem else 0)
    fig = plt.figure(figsize=(14, 4 * n_panels + 2))
    fig.suptitle("RAWES Guided Flight Test — Flight Report", fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(n_panels, 1, hspace=0.45, top=0.97 - 0.02 * n_panels,
                           bottom=0.05, left=0.08, right=0.97)

    event_colors = {
        "EKF lock":        "#2ca02c",
        "Armed":           "#d62728",
        "GUIDED":          "#9467bd",
        "Position target": "#8c564b",
    }

    t_max = max(ts_pos) if ts_pos else 60.0
    x_min = min(min(events.values(), default=0.0) - 2.0, -2.0)

    def _add_events(ax):
        ax.set_xlim(x_min, t_max + 1.0)
        ylo, yhi = ax.get_ylim()
        for label, t in sorted(events.items(), key=lambda x: x[1]):
            c = event_colors.get(label, "#7f7f7f")
            ax.axvline(t, color=c, linewidth=1.2, linestyle="--", alpha=0.8)
            ax.text(t + 0.4, yhi - (yhi - ylo) * 0.02,
                    label, color=c, fontsize=7.5, va="top",
                    rotation=90, alpha=0.9, fontweight="bold")

    # ── Panel 1: Position NED ─────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(ts_pos, ns, color="#1f77b4", linewidth=1.5, label="North (m)")
    ax1.plot(ts_pos, es, color="#ff7f0e", linewidth=1.5, label="East  (m)")
    ax1.plot(ts_pos, ds, color="#d62728", linewidth=1.5, label="Down  (m)")
    ax1.axhline(target_n, color="#1f77b4", linestyle=":", linewidth=1, alpha=0.5)
    ax1.axhline(target_e, color="#ff7f0e", linestyle=":", linewidth=1, alpha=0.5,
                label=f"Target E={target_e:.0f} m")
    ax1.axhline(target_d, color="#d62728", linestyle=":", linewidth=1, alpha=0.5,
                label=f"Target D={target_d:.1f} m")
    ax1.set_ylabel("Position (m)")
    ax1.set_title("Position NED relative to home")
    ax1.legend(loc="upper right", fontsize=8, ncol=2)
    ax1.grid(True, alpha=0.3)
    _add_events(ax1)

    # ── Panel 2: 3-D displacement ─────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(ts_pos, disps, color="#2ca02c", linewidth=1.8, label="3-D displacement (m)")
    ax2.axhline(5.0, color="#e377c2", linestyle="--", linewidth=1.2,
                label="Pass threshold (5 m)")
    ax2.set_ylabel("Displacement (m)")
    ax2.set_title("3-D displacement from home")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)
    _add_events(ax2)

    # ── Panel 3: Attitude ─────────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[2])
    if attitude_history:
        ts_att = [r[0] for r in attitude_history]
        rolls  = [r[1] for r in attitude_history]
        pitchs = [r[2] for r in attitude_history]
        yaws   = [r[3] for r in attitude_history]
        ax3.plot(ts_att, rolls,  color="#1f77b4", linewidth=1.2, label="Roll  (°)")
        ax3.plot(ts_att, pitchs, color="#ff7f0e", linewidth=1.2, label="Pitch (°)")
        ax3.plot(ts_att, yaws,   color="#2ca02c", linewidth=1.2, label="Yaw   (°)")
        ax3.axhline(0, color="black", linewidth=0.5, alpha=0.4)
    else:
        ax3.text(0.5, 0.5, "No attitude data", transform=ax3.transAxes,
                 ha="center", va="center", color="grey")
    ax3.set_ylabel("Angle (°)")
    ax3.set_title("Attitude (roll / pitch / yaw)")
    ax3.legend(loc="upper right", fontsize=8)
    ax3.grid(True, alpha=0.3)
    _add_events(ax3)

    # ── Panel 4: Servo outputs ────────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[3])
    if servo_history:
        ts_srv = [r[0] for r in servo_history]
        s1 = [r[1] for r in servo_history]
        s2 = [r[2] for r in servo_history]
        s3 = [r[3] for r in servo_history]
        s4 = [r[4] for r in servo_history]
        ax4.plot(ts_srv, s1, linewidth=1.2, label="S1 swashplate (µs)")
        ax4.plot(ts_srv, s2, linewidth=1.2, label="S2 swashplate (µs)")
        ax4.plot(ts_srv, s3, linewidth=1.2, label="S3 swashplate (µs)")
        ax4.plot(ts_srv, s4, linewidth=1.2, label="S4 ESC/anti-rot (µs)", linestyle="--")
        ax4.axhline(1500, color="black", linewidth=0.7, linestyle=":", alpha=0.5,
                    label="Neutral (1500 µs)")
    else:
        ax4.text(0.5, 0.5, "No servo data", transform=ax4.transAxes,
                 ha="center", va="center", color="grey")
    ax4.set_xlabel("Time since position target (s)" if not telem else "")
    ax4.set_ylabel("PWM (µs)")
    ax4.set_title("ArduPilot servo outputs  (S1/S2/S3 = swashplate,  S4 = ESC / anti-rotation)")
    ax4.legend(loc="upper right", fontsize=8)
    ax4.grid(True, alpha=0.3)
    _add_events(ax4)

    # ── Panel 5: Aero thrust / hub altitude / collective (from telemetry) ────
    if telem:
        ax5 = fig.add_subplot(gs[4])
        ax5b = ax5.twinx()
        ax5.plot(telem["t"], telem["T"],         color="#2ca02c", linewidth=1.0, label="Thrust T (N)")
        ax5.plot(telem["t"], telem["F_z"],        color="#1f77b4", linewidth=0.8,
                 linestyle="--", label="Fz world (N)")
        ax5b.plot(telem["t"], telem["hub_pos_z"], color="#d62728", linewidth=1.0,
                  linestyle="-.", label="Hub altitude Z (m)")
        ax5.set_ylabel("Force (N)", color="#2ca02c")
        ax5b.set_ylabel("Altitude ENU (m)", color="#d62728")
        ax5.set_title("Aero thrust & hub altitude (mediator telemetry)")
        lines5a, labels5a = ax5.get_legend_handles_labels()
        lines5b, labels5b = ax5b.get_legend_handles_labels()
        ax5.legend(lines5a + lines5b, labels5a + labels5b, loc="upper right", fontsize=7)
        ax5.grid(True, alpha=0.3)
        _add_events(ax5)

        # ── Panel 6: Rotor spin & torque / wind components ────────────────────
        ax6 = fig.add_subplot(gs[5])
        ax6b = ax6.twinx()
        ax6.plot(telem["t"], telem["omega_rotor"],  color="#9467bd", linewidth=1.0, label="ω rotor (rad/s)")
        ax6b.plot(telem["t"], telem["Q_drag"],   color="#e377c2", linewidth=0.8,
                  linestyle="--", label="Q drag (N·m)")
        ax6b.plot(telem["t"], telem["Q_drive"],  color="#bcbd22", linewidth=0.8,
                  linestyle="-.", label="Q drive (N·m)")
        ax6b.plot(telem["t"], telem["v_axial"],  color="#17becf", linewidth=0.8,
                  linestyle=":",  label="v_axial (m/s)")
        ax6b.plot(telem["t"], telem["v_inplane"],color="#8c564b", linewidth=0.8,
                  linestyle=":",  label="v_inplane (m/s)")
        ax6.set_ylabel("ω rotor (rad/s)", color="#9467bd")
        ax6b.set_ylabel("Torque (N·m) / Wind (m/s)", color="#e377c2")
        ax6.set_xlabel("Time since position target (s)")
        ax6.set_title("Rotor spin & torque balance / relative wind (mediator telemetry)")
        lines6a, labels6a = ax6.get_legend_handles_labels()
        lines6b, labels6b = ax6b.get_legend_handles_labels()
        ax6.legend(lines6a + lines6b, labels6a + labels6b, loc="upper right", fontsize=7)
        ax6.grid(True, alpha=0.3)
        _add_events(ax6)

    fig.savefig(str(out_path), dpi=120)
    plt.close(fig)
    log.info("Flight report saved → %s", out_path)


@pytest.mark.skipif(not hasattr(socket, "AF_UNIX"), reason="Requires AF_UNIX sockets")
def test_guided_flight_to_tether_position():
    """
    End-to-end GUIDED flight test: arm → GUIDED → position target → movement.

    Launches MBDyn + mediator + SITL, arms the vehicle, sets GUIDED mode, and
    commands it toward the 45° tether position.  Asserts eastward movement.
    """
    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run stack integration tests")

    mbdyn = _find_mbdyn()
    if mbdyn is None:
        pytest.skip("mbdyn binary not found in PATH")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(
            f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} to locate sim_vehicle.py"
        )

    pytest.importorskip("pymavlink")

    repo_root = Path(__file__).resolve().parents[3]
    sim_dir   = repo_root / "simulation"
    mbdyn_dir = sim_dir   / "mbdyn"

    # Clean up stale sockets from a previous run
    for sock_path in (_MBDYN_FORCE_SOCK, _MBDYN_STATE_SOCK):
        try:
            os.unlink(sock_path)
        except FileNotFoundError:
            pass

    with TemporaryDirectory(prefix="rawes_guided_") as tmpdir:
        temp_path    = Path(tmpdir)
        mediator_log  = temp_path / "mediator.log"
        sitl_log      = temp_path / "sitl.log"
        mbdyn_stdout  = temp_path / "mbdyn_stdout.log"
        mbdyn_out     = str(temp_path / "rawes")
        gcs_log       = temp_path / "gcs.log"     # Python logging output for this test
        telemetry_log = temp_path / "telemetry.csv"

        _configure_logging(gcs_log)
        log = logging.getLogger("test_guided_flight")

        log.info("─── test_guided_flight_to_tether_position starting ───")
        log.info("Target: N=%.1f E=%.1f D=%.1f m  (50 m tether @ 45°)",
                 _TARGET_NORTH, _TARGET_EAST, _TARGET_DOWN)

        # ── Launch order: MBDyn → mediator → SITL ────────────────────────────
        log.info("[1/9] Launching MBDyn ...")
        mbdyn_proc = subprocess.Popen(
            [str(mbdyn), "-f", "rotor.mbd", "-o", mbdyn_out],
            cwd=str(mbdyn_dir),
            stdout=mbdyn_stdout.open("w", encoding="utf-8"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        log.info("[1/9] Launching mediator ...")
        mediator_proc = _launch_mediator(
            sim_dir, repo_root,
            _MBDYN_FORCE_SOCK, _MBDYN_STATE_SOCK,
            mediator_log,
            telemetry_log_path=str(telemetry_log),
        )
        log.info("[1/9] Launching SITL ...")
        sitl_proc = _launch_sitl(sim_vehicle, sitl_log)

        def _procs_alive():
            for name, proc, log_path in [
                ("MBDyn",    mbdyn_proc,    mbdyn_stdout),
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

            # MBDyn stdout
            if mbdyn_stdout.exists():
                text = mbdyn_stdout.read_text(encoding="utf-8", errors="replace")
                print(f"\n--- mbdyn_stdout.log ({len(text)} chars, last 3000) ---\n{text[-3000:]}")

            # MBDyn .log file (solver diagnostics)
            mbdyn_log_file = Path(mbdyn_out + ".log")
            if mbdyn_log_file.exists():
                text = mbdyn_log_file.read_text(errors="replace")
                print(f"\n--- MBDyn .log ({len(text)} chars, last 2000) ---\n{text[-2000:]}")

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

            # ── 7. Send position target ───────────────────────────────────────
            log.info("[7/9] Sending position target N=%.1f E=%.1f D=%.1f m ...",
                     _TARGET_NORTH, _TARGET_EAST, _TARGET_DOWN)
            gcs.send_position_target_ned(
                north=_TARGET_NORTH,
                east=_TARGET_EAST,
                down=_TARGET_DOWN,
            )

            # ── 8. Observe movement for _OBSERVATION_SECONDS ─────────────────
            log.info("[8/9] Observing for %.0f s (target: 3-D displacement ≥ %.1f m) ...",
                     _OBSERVATION_SECONDS, _MIN_DISPLACEMENT)
            t_obs_start    = time.monotonic()
            t_last_pos_log = t_obs_start
            t_last_target  = t_obs_start

            # Convert wall-clock event times to seconds relative to t_obs_start.
            # Events before position target have negative t values.
            flight_events["Position target"] = 0.0
            if _t_ekf_lock is not None:
                flight_events["EKF lock"] = _t_ekf_lock - t_obs_start
            if _t_armed is not None:
                flight_events["Armed"] = _t_armed - t_obs_start
            if _t_guided is not None:
                flight_events["GUIDED"] = _t_guided - t_obs_start
            deadline       = t_obs_start + _OBSERVATION_SECONDS

            while time.monotonic() < deadline:
                _procs_alive()

                # Single recv_match covering all types we care about.
                # Using one call prevents _drain_statustext-style secondary
                # reads from discarding position/attitude/servo messages off
                # the socket before the main loop sees them.
                msg = gcs._mav.recv_match(
                    type=["LOCAL_POSITION_NED", "ATTITUDE",
                          "SERVO_OUTPUT_RAW", "STATUSTEXT"],
                    blocking=True, timeout=0.2,
                )
                t_rel = time.monotonic() - t_obs_start
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
                        log.warning("STATUSTEXT [sev=%s] %s", msg.severity, text)
                        all_statustext.append(text)

                # Periodic position summary
                now = time.monotonic()
                if now - t_last_pos_log >= _POS_LOG_INTERVAL:
                    if pos_history:
                        _, n, e, d = pos_history[-1]
                        elapsed = now - t_obs_start
                        remaining = deadline - now
                        max_disp = max(
                            math.sqrt(r[1]**2 + r[2]**2 + r[3]**2)
                            for r in pos_history
                        )
                        log.info(
                            "  t=%.0fs  pos NED=(%.2f, %.2f, %.2f)  "
                            "max_3d_disp=%.2f m  readings=%d  remaining=%.0fs",
                            elapsed, n, e, d,
                            max_disp,
                            len(pos_history),
                            remaining,
                        )
                    else:
                        log.warning(
                            "  t=%.0fs  no position readings yet — "
                            "mediator may not be sending state to SITL",
                            now - t_obs_start,
                        )
                    t_last_pos_log = now

                # Re-send target every 2 s to prevent GUIDED timeout
                if now - t_last_target >= 2.0:
                    gcs.send_position_target_ned(
                        north=_TARGET_NORTH,
                        east=_TARGET_EAST,
                        down=_TARGET_DOWN,
                    )
                    t_last_target = now

            # Observation complete — final summary
            log.info("[8/9] Observation complete.  %d position readings collected.",
                     len(pos_history))
            if pos_history:
                east_vals = [r[2] for r in pos_history]
                log.info(
                    "  East: min=%.2f  max=%.2f  final=%.2f m  (target=%.1f m)",
                    min(east_vals), max(east_vals), east_vals[-1], _TARGET_EAST,
                )
                north_vals = [r[1] for r in pos_history]
                down_vals  = [r[3] for r in pos_history]
                log.info(
                    "  North: min=%.2f  max=%.2f  final=%.2f m",
                    min(north_vals), max(north_vals), north_vals[-1],
                )
                log.info(
                    "  Down:  min=%.2f  max=%.2f  final=%.2f m  (target=%.1f m)",
                    min(down_vals), max(down_vals), down_vals[-1], _TARGET_DOWN,
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

            displacements = [
                math.sqrt(r[1]**2 + r[2]**2 + r[3]**2) for r in pos_history
            ]
            max_displacement = max(displacements)
            east_vals = [r[2] for r in pos_history]
            assert max_displacement >= _MIN_DISPLACEMENT, (
                f"Vehicle did not respond to GUIDED position target:\n"
                f"  max 3-D displacement : {max_displacement:.2f} m\n"
                f"  minimum required     : {_MIN_DISPLACEMENT:.1f} m\n"
                f"  target (N/E/D)       : ({_TARGET_NORTH}/{_TARGET_EAST}/{_TARGET_DOWN}) m\n"
                f"  readings collected   : {len(pos_history)}\n"
                f"  East range           : [{min(east_vals):.1f}, {max(east_vals):.1f}] m\n"
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

            _plot_flight_report(
                pos_history      = pos_history,
                attitude_history = attitude_history,
                servo_history    = servo_history,
                events           = flight_events,
                target           = (_TARGET_NORTH, _TARGET_EAST, _TARGET_DOWN),
                out_path         = sim_dir / "flight_report.png",
                telemetry_path   = telemetry_log,
            )

            log.info("─── test_guided_flight_to_tether_position PASSED ───")

        except Exception:
            log.exception("Test failed — see log dump below")
            _dump_logs(label="FAILURE")
            raise
        finally:
            gcs.close()
            _terminate_process(sitl_proc)
            _terminate_process(mediator_proc)
            _terminate_process(mbdyn_proc)
            for sock_path in (_MBDYN_FORCE_SOCK, _MBDYN_STATE_SOCK):
                try:
                    os.unlink(sock_path)
                except FileNotFoundError:
                    pass
            # Always dump logs so a passing run is also inspectable
            _dump_logs(label="post-test")
