"""
test_pumping_cycle.py — De Schutter pumping cycle stack test (SITL + ArduPilot).

Mirrors test_deschutter_cycle.py (unit, no ArduPilot) but runs the full stack:
  mediator (internal_controller=True, pumping_cycle=True) + ArduPilot SITL.

Pumping cycle strategy (De Schutter 2018 Fig. 4):
  Reel-out: body_z tether-aligned, TensionController drives collective to 200 N
            → high tether tension → winch generates power.
  Reel-in:  body_z blends to vertical over 5 s → thrust acts upward, not along
            tether → winch encounters near-zero aerodynamic resistance → low tension.

The mediator's internal pumping-cycle state machine handles all of this.
This test only observes and asserts from the outside, via mediator telemetry CSV.

Telemetry columns used:
  t, tension, tether_rest_length, phase, collective_rad, pos_enu_z

Pass criteria (matching unit test thresholds):
  1. Hub stays above MIN_ALT_M throughout — no crash.
  2. Reel-in steady-state mean tension < reel-out mean tension.
  3. Net energy (reel-out − reel-in) > 0.
  4. Peak tension < 80% of Dyneema SK75 break load (620 N).
  5. No CRITICAL errors in mediator log.
"""

import csv
import json
import logging
import math
import os
import sys
import time
from pathlib import Path

import pytest

_SIM_DIR   = Path(__file__).resolve().parents[2]
_STACK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_STACK_DIR))

from conftest import StackContext, dump_startup_diagnostics, assert_stack_ports_free

# ---------------------------------------------------------------------------
# Pumping cycle parameters — match test_deschutter_cycle.py defaults
# ---------------------------------------------------------------------------
_T_REEL_OUT       = 30.0   # s — reel-out phase duration
_T_REEL_IN        = 30.0   # s — reel-in phase duration
_T_TRANSITION     =  5.0   # s — body_z blend window at reel-in start
_V_REEL_OUT       =  0.4   # m/s — winch reel-out speed
_V_REEL_IN        =  0.4   # m/s — winch reel-in speed
_TENSION_OUT      = 200.0  # N — reel-out tension setpoint
_TENSION_IN       =  20.0  # N — reel-in tension setpoint

_CYCLE_DURATION   = _T_REEL_OUT + _T_REEL_IN   # 60 s per cycle

# How long after ACRO arm to wait for physics to stabilise before the first
# reel-out starts.  The mediator's startup_damp_seconds (45 s) has already
# elapsed by the time the fixture yields — we just need the aero ramp (5 s).
_SETTLE_SECONDS   = 6.0

# Total observation window: one full pumping cycle + settle time + margin
_OBS_SECONDS      = _SETTLE_SECONDS + _CYCLE_DURATION + 10.0

# ---------------------------------------------------------------------------
# Pass/fail thresholds
# ---------------------------------------------------------------------------
_MIN_ALT_M        =   2.0    # ENU Z floor — hub must stay above this
_MAX_DRIFT_M      = 200.0    # runaway-only guard
_BREAK_LOAD_N     = 620.0    # Dyneema SK75 1.9 mm break load [N]
_TENSION_LIMIT_N  = 0.8 * _BREAK_LOAD_N   # 496 N


# ---------------------------------------------------------------------------
# Telemetry parsing
# ---------------------------------------------------------------------------

def _parse_telemetry(path: Path) -> list[dict]:
    """Read mediator telemetry CSV into list of dicts with float values.

    Mediator column names used here:
      t_sim           — simulation time [s]
      tether_tension  — tether tension [N]
      pumping_phase   — 1.0 = reel-out, 2.0 = reel-in (empty when cycle inactive)
      tether_rest_length — current rest length [m]
      hub_pos_z       — hub ENU altitude [m]
    """
    rows = []
    if not path.exists():
        return rows
    with path.open(encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            parsed = {}
            for k, v in row.items():
                if v not in ("", "None", "nan"):
                    try:
                        parsed[k] = float(v)
                    except ValueError:
                        pass
            if parsed:
                rows.append(parsed)
    return rows


def _split_phases(rows: list[dict], t_start: float, t_reel_out: float, t_reel_in: float):
    """
    Split telemetry rows into reel-out and reel-in samples for the first cycle
    that starts at approximately t_start (in simulation time).

    Uses 'pumping_phase' column (1.0 = reel-out, 2.0 = reel-in) if present,
    otherwise falls back to time-based splitting relative to t_start.
    """
    cycle_end = t_start + t_reel_out + t_reel_in
    in_cycle  = [r for r in rows if t_start <= r.get("t_sim", 0.0) <= cycle_end]

    if in_cycle and "pumping_phase" in in_cycle[0]:
        out = [r for r in in_cycle if r.get("pumping_phase", 0.0) == 1.0]
        inn = [r for r in in_cycle if r.get("pumping_phase", 0.0) == 2.0]
    else:
        out = [r for r in in_cycle if r.get("t_sim", 0.0) < t_start + t_reel_out]
        inn = [r for r in in_cycle if r.get("t_sim", 0.0) >= t_start + t_reel_out]

    return out, inn


# ---------------------------------------------------------------------------
# Fixture override — use physical sensor mode for GPS fusion
# ---------------------------------------------------------------------------

@pytest.fixture
def sensor_mode():
    """Pumping cycle test uses physical sensor mode (GPS + compass consistent)."""
    return "physical"


# ---------------------------------------------------------------------------
# Main test
# ---------------------------------------------------------------------------

def test_pumping_cycle(acro_armed: StackContext):
    """
    Full pumping cycle stack test: reel-out (30 s) then reel-in (30 s).

    Asserts:
      1. No crash (hub above MIN_ALT_M throughout).
      2. Reel-in steady tension < reel-out mean tension.
      3. Net energy positive (energy_out > energy_in).
      4. Peak tension below 80% break load (496 N).
      5. No CRITICAL errors in mediator log.
    """
    ctx = acro_armed
    gcs = ctx.gcs
    log = logging.getLogger("test_pumping_cycle")

    pos_history:      list[tuple[float, float, float, float]] = []
    all_statustext    = ctx.all_statustext

    t_cycle_start = 0.0   # relative time (from obs start) when the first reel-out begins
    t_obs_start   = time.monotonic()
    deadline      = t_obs_start + _OBS_SECONDS

    log.info("─── test_pumping_cycle: observing %.0f s ───", _OBS_SECONDS)

    try:
        while time.monotonic() < deadline:
            # Check processes haven't died
            for name, proc, lp in [
                ("mediator", ctx.mediator_proc, ctx.mediator_log),
                ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
            ]:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    pytest.fail(f"{name} exited during pumping cycle (rc={proc.returncode}):\n{txt[-3000:]}")

            msg = gcs._mav.recv_match(
                type=["LOCAL_POSITION_NED", "STATUSTEXT"],
                blocking=True, timeout=0.2,
            )
            t_rel = time.monotonic() - t_obs_start
            if msg is not None:
                mt = msg.get_type()
                if mt == "LOCAL_POSITION_NED":
                    pos_history.append((t_rel, msg.x, msg.y, msg.z))
                elif mt == "STATUSTEXT":
                    text = msg.text.rstrip("\x00").strip()
                    log.info("STATUSTEXT: %s", text)
                    all_statustext.append(text)

            # Send neutral RC keepalive (motor interlock + neutral attitude)
            now = time.monotonic()
            if int(now * 10) % 10 == 0:   # approx every 0.1 s
                gcs.send_rc_override({1: 1500, 2: 1500, 3: 1500, 4: 1500, 8: 2000})

        log.info("Observation complete: %d pos samples", len(pos_history))

        # ── Crash check from MAVLink position ─────────────────────────────────
        assert pos_history, (
            "No LOCAL_POSITION_NED received during pumping cycle.\n"
            f"STATUSTEXT: {all_statustext}"
        )

        # LOCAL_POSITION_NED: d = NED-down positive, so ENU_z = home_z - d
        max_down  = max(d for _, _, _, d in pos_history)
        min_enu_z = ctx.home_z_enu - max_down
        log.info("Min ENU alt (MAVLink): %.2f m  (limit=%.1f m)", min_enu_z, _MIN_ALT_M)
        assert min_enu_z >= _MIN_ALT_M, (
            f"Hub crashed: min ENU Z {min_enu_z:.2f} m < {_MIN_ALT_M:.1f} m\n"
            f"STATUSTEXT: {all_statustext}"
        )

        drifts    = [math.sqrt(n**2 + e**2 + d**2) for _, n, e, d in pos_history]
        max_drift = max(drifts)
        log.info("Max drift: %.2f m  (limit=%.0f m)", max_drift, _MAX_DRIFT_M)
        assert max_drift <= _MAX_DRIFT_M, (
            f"Hub runaway: max drift {max_drift:.2f} m > {_MAX_DRIFT_M:.0f} m"
        )

        # ── Parse telemetry CSV for pumping-cycle assertions ─────────────────
        tel = _parse_telemetry(ctx.telemetry_log)
        log.info("Telemetry: %d rows", len(tel))

        if not tel:
            pytest.skip("No mediator telemetry CSV — cannot assert pumping cycle behaviour. "
                        "Ensure --telemetry-log is passed to mediator.")

        # Find rows that contain tension data (pumping cycle active rows)
        tension_rows = [r for r in tel
                        if "tether_tension" in r and r["tether_tension"] > 0
                        and "t_sim" in r]
        if not tension_rows:
            pytest.skip("Telemetry has no tether_tension data — pumping cycle assertions skipped.")

        tensions = [r["tether_tension"] for r in tension_rows]
        ts_sim   = [r["t_sim"]          for r in tension_rows]

        # ── Peak tension / tether safety ──────────────────────────────────────
        peak_tension = max(tensions)
        log.info("Peak tension: %.1f N  (limit=%.1f N)", peak_tension, _TENSION_LIMIT_N)
        assert peak_tension < _TENSION_LIMIT_N, (
            f"Peak tension {peak_tension:.1f} N ≥ 80% break load ({_TENSION_LIMIT_N:.1f} N)"
        )

        # ── Phase split: reel-out vs reel-in ──────────────────────────────────
        # The mediator starts the pumping cycle after startup_damp_seconds.
        # Use the first t_sim with a non-empty pumping_phase as the cycle anchor.
        phase_rows = [r for r in tension_rows if r.get("pumping_phase", 0.0) > 0]
        if phase_rows:
            t_cycle_start_abs = phase_rows[0]["t_sim"]
        else:
            # Fallback: skip settle + aero ramp
            t_cycle_start_abs = min(ts_sim) + _SETTLE_SECONDS

        out_rows, in_rows = _split_phases(tension_rows, t_cycle_start_abs,
                                          _T_REEL_OUT, _T_REEL_IN)

        log.info("Phase split: reel-out=%d rows  reel-in=%d rows", len(out_rows), len(in_rows))

        if len(out_rows) < 5 or len(in_rows) < 5:
            pytest.skip(
                f"Insufficient phase data: reel-out={len(out_rows)} reel-in={len(in_rows)} rows. "
                "Check telemetry and cycle timing."
            )

        mean_tension_out = sum(r["tether_tension"] for r in out_rows) / len(out_rows)

        # Skip transition period at start of reel-in (body_z blending)
        skip_in   = int(_T_TRANSITION * len(in_rows) / _T_REEL_IN)
        steady_in = in_rows[skip_in:] or in_rows

        mean_tension_in_steady = sum(r["tether_tension"] for r in steady_in) / len(steady_in)

        log.info(
            "Reel-out mean tension: %.1f N  |  Reel-in steady mean: %.1f N",
            mean_tension_out, mean_tension_in_steady,
        )

        # ── Energy accounting from telemetry ──────────────────────────────────
        # E = Σ T × v_reel × dt   (telemetry written every step at 400 Hz → dt ≈ 0.0025 s)
        dt_tel  = (ts_sim[-1] - ts_sim[0]) / max(len(ts_sim) - 1, 1)
        energy_out = sum(r["tether_tension"] * _V_REEL_OUT * dt_tel for r in out_rows)
        energy_in  = sum(r["tether_tension"] * _V_REEL_IN  * dt_tel for r in in_rows)
        net_energy = energy_out - energy_in

        log.info(
            "Energy: out=%.1f J  in=%.1f J  net=%.1f J",
            energy_out, energy_in, net_energy,
        )

        # ── Assertions ────────────────────────────────────────────────────────
        assert mean_tension_in_steady < mean_tension_out, (
            f"Reel-in steady tension ({mean_tension_in_steady:.1f} N) not less than "
            f"reel-out ({mean_tension_out:.1f} N) — De Schutter tilt mechanism not working"
        )

        assert net_energy > 0, (
            f"Net energy {net_energy:.1f} J ≤ 0 — pumping cycle does not produce net power"
        )

        # ── CRITICAL log check ─────────────────────────────────────────────────
        if ctx.mediator_log.exists():
            med_text = ctx.mediator_log.read_text(encoding="utf-8", errors="replace")
            critical = [l for l in med_text.splitlines() if "CRITICAL" in l]
            assert not critical, (
                "CRITICAL errors in mediator log:\n" + "\n".join(critical[:10])
            )

        log.info(
            "─── test_pumping_cycle PASSED  (net_energy=%.1f J  peak_tension=%.1f N) ───",
            net_energy, peak_tension,
        )

    except Exception:
        dump_startup_diagnostics(ctx)
        raise


# ---------------------------------------------------------------------------
# Fixture: launch mediator with pumping_cycle=True
# ---------------------------------------------------------------------------

# Override the acro_armed fixture's mediator launch to include pumping cycle config.
# We do this by extending conftest's acro_armed with a module-level fixture that
# injects the pumping cycle config via extra_config.

@pytest.fixture
def acro_armed(tmp_path, sensor_mode):
    """
    Pumping cycle variant of acro_armed.

    Extends conftest.acro_armed by launching the mediator with:
      - pumping_cycle = True
      - internal_controller = True  (required — pumping cycle uses truth-state controller)
      - Pumping cycle timing/tension parameters
    """
    import dataclasses
    import shutil
    import subprocess
    import os as _os

    from test_stack_integration import (
        ARDUPILOT_ENV, STACK_ENV_FLAG, SIM_VEHICLE_ENV,
        _launch_mediator, _launch_sitl, _resolve_sim_vehicle, _terminate_process,
    )
    from conftest import (
        StackConfig, _configure_logging, _run_acro_setup,
        assert_stack_ports_free,
    )
    from gcs import RawesGCS
    from controller import make_hold_controller
    import config as _mcfg

    if _os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run stack integration tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} to locate sim_vehicle.py")

    pytest.importorskip("pymavlink")
    assert_stack_ports_free()

    repo_root = Path(__file__).resolve().parents[3]
    sim_dir   = repo_root / "simulation"

    _STARTING_STATE = sim_dir / "steady_state_starting.json"
    initial_state   = None
    home_z_enu      = 12.530
    if _STARTING_STATE.exists():
        initial_state = json.loads(_STARTING_STATE.read_text())
        home_z_enu    = float(initial_state["pos"][2])

    mediator_log  = tmp_path / "mediator.log"
    sitl_log      = tmp_path / "sitl.log"
    gcs_log       = tmp_path / "gcs.log"
    telemetry_log = tmp_path / "telemetry.csv"

    _configure_logging(gcs_log)
    log = logging.getLogger("acro_armed_pumping")
    logging.getLogger("gcs").setLevel(logging.DEBUG)

    import numpy as _np
    _anchor_ned = _np.array([0.0, 0.0, float(home_z_enu)])
    controller = make_hold_controller(sensor_mode, anchor_ned=_anchor_ned)

    import time as _t
    _run_id = int(_t.time())
    log.info("RUN_ID=%d", _run_id)
    log.info("acro_armed (pumping cycle): launching mediator + SITL ...")

    trajectory_extra = {
        "trajectory": {
            "type": "deschutter",
            "hold": {},
            "deschutter": {
                "t_reel_out":     _T_REEL_OUT,
                "t_reel_in":      _T_REEL_IN,
                "t_transition":   _T_TRANSITION,
                "v_reel_out":     _V_REEL_OUT,
                "v_reel_in":      _V_REEL_IN,
                "tension_out":    _TENSION_OUT,
                "tension_in":     _TENSION_IN,
                "xi_reel_in_deg": 55.0,
            },
        },
    }

    mediator_proc = _launch_mediator(
        sim_dir, repo_root, mediator_log,
        telemetry_log_path    = str(telemetry_log),
        initial_state         = initial_state,
        startup_damp_seconds  = StackConfig.STARTUP_DAMP_S,
        lock_orientation      = StackConfig.LOCK_ORIENTATION,
        sensor_mode           = sensor_mode,
        run_id                = _run_id,
        base_k_ang            = StackConfig.BASE_K_ANG_INTERNAL,
        internal_controller   = True,   # required for pumping cycle
        extra_config          = trajectory_extra,
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
        internal_controller=True,
    )

    try:
        _run_acro_setup(ctx, _procs_alive)
        yield ctx
    finally:
        gcs.close()
        _terminate_process(sitl_proc)
        _terminate_process(mediator_proc)
        if telemetry_log.exists():
            shutil.copy2(telemetry_log, sim_dir / "telemetry_pumping.csv")
        if mediator_log.exists():
            shutil.copy(mediator_log, sim_dir / "mediator_last_run.log")
        if sitl_log.exists():
            shutil.copy(sitl_log, sim_dir / "sitl_last_run.log")
        if gcs_log.exists():
            shutil.copy(gcs_log, sim_dir / "gcs_last_run.log")
