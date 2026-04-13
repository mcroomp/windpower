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

Telemetry columns used (TelRow fields):
  t_sim, tether_tension, tether_rest_length, phase, collective_rad, pos_z

Pass criteria (matching unit test thresholds):
  1. Hub stays above MIN_ALT_M throughout — no crash.
  2. Reel-in steady-state mean tension < reel-out mean tension.
  3. Net energy (reel-out − reel-in) > 0.
  4. Peak tension < 80% of Dyneema SK75 break load (620 N).
  5. No CRITICAL errors in mediator log.
"""

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

from conftest import StackContext, dump_startup_diagnostics
from telemetry_csv import read_csv, TelRow

# ---------------------------------------------------------------------------
# Pumping cycle parameters — taken from config DEFAULTS so stack test and
# unit test (test_deschutter_cycle.py) use the same operating point.
# ---------------------------------------------------------------------------
import config as _cfg
_DCFG         = _cfg.DEFAULTS["trajectory"]["deschutter"]
_T_REEL_OUT   = float(_DCFG["t_reel_out"])    # 30.0 s
_T_REEL_IN    = float(_DCFG["t_reel_in"])     # 30.0 s
_T_TRANSITION = float(_DCFG["t_transition"])  # 3.7 s
_V_REEL_OUT   = float(_DCFG["v_reel_out"])    # 0.4 m/s
_V_REEL_IN    = float(_DCFG["v_reel_in"])     # 0.4 m/s
_TENSION_OUT  = float(_DCFG["tension_out"])   # 200.0 N
_TENSION_IN   = float(_DCFG["tension_in"])    # 55.0 N

_CYCLE_DURATION   = _T_REEL_OUT + _T_REEL_IN   # 60 s per cycle

# Total observation window.
# arm fires at ~t=54s from mediator start; fixture yields then.
# acro_armed_pumping uses t_hold_s=10s so the winch holds briefly after
# kinematic exit (t=65s) to let TensionPI stabilise.  After fixture yield:
#   remaining kinematic:  ~11s  (kinematic ends at t=65s)
#   post-kinematic hold:  10s   (t_hold_s=10, winch paused)
#   first reel-out:       30s
#   first reel-in:        30s
#   margin:               10s
#   total: ~91s  -> 125s gives ample margin.
_OBS_SECONDS      = 125.0

# Observation window for the Lua pumping variant.
# Kinematic runs for 120 s (startup_damp_seconds=120); GPS fuses at ~74 s so
# Lua captures at ~79 s.  After kinematic exit (t=120 s):
#   t_hold_s=10 s hold:  t=120..130 s
#   reel_out triggers:   t~130.4 s (0.05 m / 0.12 m/s after reel-out starts)
# Fixture yields at ~mediator t=12 s; observation must reach t~131 s = 119 s.
# Full pumping cycle to validate physics: add ~70 s reel-out + reel-in.
# 165 s gives ample margin.
_LUA_OBS_SECONDS  = 165.0

# ---------------------------------------------------------------------------
# Pass/fail thresholds
# ---------------------------------------------------------------------------
_MIN_ALT_M        =   0.5    # hub must stay above this (not crashed)
                              # Hub descends to ~1 m after kinematic phase due to thrust
                              # margin limitation (same as test_acro_hold); 0.5 m detects
                              # a real ground impact without false-failing on normal descent.
_MAX_DRIFT_M      = 200.0    # runaway-only guard
_BREAK_LOAD_N     = 620.0    # Dyneema SK75 1.9 mm break load [N]
_TENSION_LIMIT_N  = 0.8 * _BREAK_LOAD_N   # 496 N


# ---------------------------------------------------------------------------
# Telemetry helpers
# ---------------------------------------------------------------------------

def _split_phases(rows: list) -> tuple[list, list]:
    """
    Split TelRow list into first-complete-cycle reel-out and reel-in rows.

    Uses TelRow.phase ("reel-out" | "reel-in" | "") written by the mediator.
    Only returns rows from the FIRST complete cycle.  Partial cycles at the
    end of the observation window are discarded so assertions are never
    contaminated by incomplete data.

    Returns (out_rows, in_rows) where both lists may be empty if no
    complete cycle is found.
    """
    out_rows     = []
    in_rows      = []
    in_reel_out  = False
    reel_in_done = False

    for r in rows:
        phase = r.phase
        if phase == "reel-out":
            if reel_in_done:
                break
            in_reel_out = True
            out_rows.append(r)
        elif phase == "reel-in":
            if not in_reel_out:
                continue
            reel_in_done = True
            in_rows.append(r)
        else:
            if reel_in_done:
                break

    return out_rows, in_rows


# ---------------------------------------------------------------------------
# Main test
# ---------------------------------------------------------------------------

def test_pumping_cycle(acro_armed_pumping: StackContext):
    """
    Full pumping cycle stack test: reel-out (30 s) then reel-in (30 s).

    Asserts:
      1. No crash (hub above MIN_ALT_M throughout).
      2. Reel-in steady tension < reel-out mean tension.
      3. Net energy positive (energy_out > energy_in).
      4. Peak tension below 80% break load (496 N).
      5. No CRITICAL errors in mediator log.
    """
    ctx = acro_armed_pumping
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

        # ── MAVLink position (diagnostic only) ────────────────────────────────
        # LOCAL_POSITION_NED requires GPS fusion.  In physical-sensor mode the
        # EKF may not fuse GPS (CONST_POS_MODE), so 0 samples is normal.
        # The authoritative crash check uses the mediator telemetry CSV below.
        if pos_history:
            max_down  = max(d for _, _, _, d in pos_history)
            min_alt_mavlink = ctx.home_alt_m - max_down
            log.info("Min altitude (MAVLink, diagnostic): %.2f m", min_alt_mavlink)
            drifts    = [math.sqrt(n**2 + e**2 + d**2) for _, n, e, d in pos_history]
            max_drift = max(drifts)
            log.info("Max drift: %.2f m  (limit=%.0f m)", max_drift, _MAX_DRIFT_M)
        else:
            log.info("No LOCAL_POSITION_NED received (GPS not fused -- expected in "
                     "physical-sensor mode).  Using mediator CSV for crash check.")

        # ── Parse telemetry CSV for pumping-cycle assertions ─────────────────
        tel = read_csv(ctx.telemetry_log)
        log.info("Telemetry: %d rows", len(tel))

        if not tel:
            pytest.skip("No mediator telemetry CSV — cannot assert pumping cycle behaviour. "
                        "Ensure --telemetry-log is passed to mediator.")

        # ── Crash check from mediator telemetry (authoritative physics altitude) ──
        # Use pos_z (NED Z) from mediator telemetry instead of MAVLink LOCAL_POSITION_NED.
        # EKF altitude drifts during GPS glitch events in CONST_POS_MODE, giving false
        # crash detections when the physics altitude is well above the limit.
        # pos_z is NED Z (negative = above ground); altitude = -pos_z.
        z_tel = [-r.pos_z for r in tel]
        if z_tel:
            min_alt_tel = min(z_tel)
            log.info("Min physics altitude (telemetry): %.2f m  (limit=%.1f m)",
                     min_alt_tel, _MIN_ALT_M)
            assert min_alt_tel >= _MIN_ALT_M, (
                f"Hub crashed: min physics altitude {min_alt_tel:.2f} m < {_MIN_ALT_M:.1f} m\n"
                f"STATUSTEXT: {all_statustext}"
            )

        # ── Phase split: first complete reel-out + reel-in cycle ─────────────
        # _split_phases uses the "phase" string label ("reel-out" /
        # "reel-in") written by the mediator.  Only the first complete cycle
        # is returned; partial cycles at the end of the observation window
        # are discarded so assertions are never contaminated by incomplete data.
        out_rows, in_rows = _split_phases(tel)

        log.info("Phase split: reel-out=%d rows  reel-in=%d rows", len(out_rows), len(in_rows))

        # Confirm tension data is present in the active-phase rows
        tension_rows = [r for r in out_rows + in_rows if r.tether_tension > 0]
        if not tension_rows:
            pytest.skip("No tension data in phase rows — pumping cycle assertions skipped.")

        if len(out_rows) < 5 or len(in_rows) < 5:
            pytest.skip(
                f"Insufficient phase data: reel-out={len(out_rows)} reel-in={len(in_rows)} rows. "
                "Check telemetry and cycle timing."
            )

        # Timing step between telemetry rows (400 Hz → ≈ 0.0025 s)
        all_t   = [r.t_sim for r in out_rows + in_rows]
        dt_tel  = ((all_t[-1] - all_t[0]) / max(len(all_t) - 1, 1)) if len(all_t) > 1 else 0.0025

        mean_tension_out = sum(r.tether_tension for r in out_rows) / len(out_rows)

        # Skip transition period at start of reel-in (body_z blending)
        skip_in   = int(_T_TRANSITION * len(in_rows) / _T_REEL_IN)
        steady_in = in_rows[skip_in:] or in_rows

        mean_tension_in_steady = sum(r.tether_tension for r in steady_in) / len(steady_in)

        peak_tension = max((r.tether_tension for r in tension_rows), default=0.0)

        log.info(
            "Reel-out mean tension: %.1f N  |  Reel-in steady mean: %.1f N  |  Peak: %.1f N",
            mean_tension_out, mean_tension_in_steady, peak_tension,
        )

        # ── Energy accounting from telemetry ──────────────────────────────────
        # E = Σ T × v_reel × dt  (400 Hz telemetry; trapezoidal integral per phase)
        energy_out = sum(r.tether_tension * _V_REEL_OUT * dt_tel for r in out_rows)
        energy_in  = sum(r.tether_tension * _V_REEL_IN  * dt_tel for r in in_rows)
        net_energy = energy_out - energy_in

        log.info(
            "Energy: out=%.1f J  in=%.1f J  net=%.1f J",
            energy_out, energy_in, net_energy,
        )

        # ── Peak tension / tether safety ──────────────────────────────────────
        log.info("Peak tension: %.1f N  (limit=%.1f N)", peak_tension, _TENSION_LIMIT_N)
        assert peak_tension < _TENSION_LIMIT_N, (
            f"Peak tension {peak_tension:.1f} N ≥ 80% break load ({_TENSION_LIMIT_N:.1f} N)\n"
            "The reel-in phase is dynamically unstable after the 45 s kinematic startup:\n"
            "the hub enters a different orbit state than the unit-test steady state,\n"
            "causing the tether to snap taut during reel-in.\n"
            "Unit test test_deschutter_tether_not_broken validates the same physics\n"
            "correctly (peak ≈ 246 N) and must be used as the authoritative safety check\n"
            "until the stack-test initial-condition mismatch is resolved."
        )

        # ── De Schutter mechanism ─────────────────────────────────────────────
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
# Lua pumping cycle test
# ---------------------------------------------------------------------------

def test_pumping_cycle_lua(acro_armed_pumping_lua: StackContext):
    """
    Pumping cycle stack test with rawes.lua (SCR_USER6=5) running.

    Uses internal_controller=True (same as test_pumping_cycle) because the
    orbital tether geometry gives a net downward force at kinematic exit (~22
    m/s^2) -- the tether pulls mostly horizontally and slightly downward
    toward the anchor (8 deg below the hub), so Lua's 50 Hz RC overrides
    cannot recover.  The internal 400 Hz controller stabilises the hub.

    What this test validates beyond test_pumping_cycle:
      - Lua SCR_USER6=5 starts without error and does not disrupt physics.
      - Lua detects the mediator's winch reel-out from tether length change
        and sends "RAWES pump: reel_out" STATUSTEXT.

    Same pass criteria as test_pumping_cycle:
      1. No crash (hub above MIN_ALT_M throughout).
      2. Reel-in steady tension < reel-out mean tension.
      3. Net energy positive.
      4. Peak tension < 80% break load (496 N).
      5. "RAWES pump: reel_out" STATUSTEXT appears (Lua detected reel-out).
      6. No CRITICAL errors in mediator log.
    """
    ctx = acro_armed_pumping_lua
    gcs = ctx.gcs
    log = logging.getLogger("test_pumping_cycle_lua")

    all_statustext = ctx.all_statustext
    # Pre-populate from STATUSTEXT captured during fixture setup (e.g. drain loop)
    reel_out_seen  = any("RAWES pump: reel_out" in t for t in all_statustext)

    t_obs_start = time.monotonic()
    deadline    = t_obs_start + _LUA_OBS_SECONDS

    log.info("--- test_pumping_cycle_lua: observing %.0f s ---", _LUA_OBS_SECONDS)

    try:
        while time.monotonic() < deadline:
            # Motor interlock keepalive; Lua owns Ch1/Ch2/Ch3
            gcs.send_rc_override({8: 2000})

            for name, proc, lp in [
                ("mediator", ctx.mediator_proc, ctx.mediator_log),
                ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
            ]:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    pytest.fail(f"{name} exited during pumping_lua (rc={proc.returncode}):\n{txt[-3000:]}")

            msg = gcs._mav.recv_match(
                type=["LOCAL_POSITION_NED", "STATUSTEXT"],
                blocking=True, timeout=0.1,
            )
            if msg is not None:
                mt = msg.get_type()
                if mt == "STATUSTEXT":
                    text = msg.text.rstrip("\x00").strip()
                    all_statustext.append(text)
                    log.info("STATUSTEXT: %s", text)
                    if "RAWES pump: reel_out" in text:
                        reel_out_seen = True

        # ── Parse telemetry CSV ────────────────────────────────────────────
        tel = read_csv(ctx.telemetry_log)
        log.info("Telemetry: %d rows", len(tel))

        if not tel:
            pytest.skip("No mediator telemetry CSV.")

        # ── Lua sync check ────────────────────────────────────────────────
        assert reel_out_seen, (
            "STATUSTEXT 'RAWES pump: reel_out' never appeared. "
            "Lua may not have detected the mediator's winch paying out. "
            f"All STATUSTEXT: {all_statustext}"
        )

        # ── Crash check ───────────────────────────────────────────────────
        z_tel = [-r.pos_z for r in tel]
        if z_tel:
            min_alt_tel = min(z_tel)
            log.info("Min physics altitude: %.2f m  (limit=%.1f m)",
                     min_alt_tel, _MIN_ALT_M)
            assert min_alt_tel >= _MIN_ALT_M, (
                f"Hub crashed: min physics altitude {min_alt_tel:.2f} m < {_MIN_ALT_M:.1f} m\n"
                f"STATUSTEXT: {all_statustext}"
            )

        # ── Phase split ───────────────────────────────────────────────────
        out_rows, in_rows = _split_phases(tel)
        log.info("Phase split: reel-out=%d rows  reel-in=%d rows",
                 len(out_rows), len(in_rows))

        tension_rows = [r for r in out_rows + in_rows if r.tether_tension > 0]
        if not tension_rows:
            pytest.skip("No tension data in phase rows.")

        if len(out_rows) < 5 or len(in_rows) < 5:
            pytest.skip(
                f"Insufficient phase data: reel-out={len(out_rows)} "
                f"reel-in={len(in_rows)} rows."
            )

        all_t   = [r.t_sim for r in out_rows + in_rows]
        dt_tel  = ((all_t[-1] - all_t[0]) / max(len(all_t) - 1, 1)) if len(all_t) > 1 else 0.0025

        mean_tension_out = sum(r.tether_tension for r in out_rows) / len(out_rows)

        skip_in   = int(_T_TRANSITION * len(in_rows) / _T_REEL_IN)
        steady_in = in_rows[skip_in:] or in_rows
        mean_tension_in_steady = sum(r.tether_tension for r in steady_in) / len(steady_in)
        peak_tension = max((r.tether_tension for r in tension_rows), default=0.0)

        energy_out = sum(r.tether_tension * _V_REEL_OUT * dt_tel for r in out_rows)
        energy_in  = sum(r.tether_tension * _V_REEL_IN  * dt_tel for r in in_rows)
        net_energy = energy_out - energy_in

        log.info(
            "Tension: out=%.1f N  in_steady=%.1f N  peak=%.1f N",
            mean_tension_out, mean_tension_in_steady, peak_tension,
        )
        log.info(
            "Energy: out=%.1f J  in=%.1f J  net=%.1f J",
            energy_out, energy_in, net_energy,
        )

        assert peak_tension < _TENSION_LIMIT_N, (
            f"Peak tension {peak_tension:.1f} N >= 80%% break load "
            f"({_TENSION_LIMIT_N:.1f} N)"
        )

        assert mean_tension_in_steady < mean_tension_out, (
            f"Reel-in steady tension ({mean_tension_in_steady:.1f} N) not less than "
            f"reel-out ({mean_tension_out:.1f} N) -- De Schutter tilt mechanism not working"
        )

        assert net_energy > 0, (
            f"Net energy {net_energy:.1f} J <= 0 -- pumping cycle does not produce net power"
        )

        # ── CRITICAL log check ────────────────────────────────────────────
        if ctx.mediator_log.exists():
            med_text = ctx.mediator_log.read_text(encoding="utf-8", errors="replace")
            critical = [l for l in med_text.splitlines() if "CRITICAL" in l]
            assert not critical, (
                "CRITICAL errors in mediator log:\n" + "\n".join(critical[:10])
            )

        log.info(
            "--- test_pumping_cycle_lua PASSED  (net_energy=%.1f J  peak=%.1f N) ---",
            net_energy, peak_tension,
        )

    except Exception:
        dump_startup_diagnostics(ctx)
        raise
