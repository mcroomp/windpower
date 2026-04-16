"""
test_pumping_cycle.py — De Schutter pumping cycle stack test with rawes.lua.

test_pumping_cycle_lua validates the full Lua pumping mode (SCR_USER6=5):
  - Lua detects mediator winch reel-out from tether length change.
  - De Schutter mechanism (reel-out high tension / reel-in low tension) works.
  - Net energy > 0.
  - Peak tension < 80% Dyneema SK75 break load (496 N).
  - No crash, no CRITICAL mediator errors.

Telemetry columns used (TelRow fields):
  t_sim, tether_tension, tether_rest_length, phase, collective_rad, pos_z
"""

import logging
import sys
from pathlib import Path

import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import StackContext, dump_startup_diagnostics
from telemetry_csv import read_csv

# ---------------------------------------------------------------------------
# Pumping cycle parameters — taken from config DEFAULTS so stack test and
# unit test (test_deschutter_cycle.py) use the same operating point.
# ---------------------------------------------------------------------------
import config as _cfg
_DCFG         = _cfg.DEFAULTS["trajectory"]["deschutter"]
_T_REEL_IN    = float(_DCFG["t_reel_in"])     # 30.0 s
_T_TRANSITION = float(_DCFG["t_transition"])  # 3.7 s
_V_REEL_OUT   = float(_DCFG["v_reel_out"])    # 0.4 m/s
_V_REEL_IN    = float(_DCFG["v_reel_in"])     # 0.4 m/s

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

    t_obs_start = gcs.sim_now()
    deadline    = t_obs_start + _LUA_OBS_SECONDS

    log.info("--- test_pumping_cycle_lua: observing %.0f s ---", _LUA_OBS_SECONDS)

    try:
        while gcs.sim_now() < deadline:
            # Motor interlock keepalive; Lua owns Ch1/Ch2/Ch3
            gcs.send_rc_override({8: 2000})

            for name, proc, lp in [
                ("mediator", ctx.mediator_proc, ctx.mediator_log),
                ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
            ]:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    pytest.fail(f"{name} exited during pumping_lua (rc={proc.returncode}):\n{txt[-3000:]}")

            msg = gcs._recv(
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

        # -- Parse telemetry CSV -----------------------------------------------
        tel = read_csv(ctx.telemetry_log)
        log.info("Telemetry: %d rows", len(tel))

        if not tel:
            pytest.skip("No mediator telemetry CSV.")

        # -- Lua sync check ---------------------------------------------------
        assert reel_out_seen, (
            "STATUSTEXT 'RAWES pump: reel_out' never appeared. "
            "Lua may not have detected the mediator's winch paying out. "
            f"All STATUSTEXT: {all_statustext}"
        )

        # -- Crash check ------------------------------------------------------
        z_tel = [-r.pos_z for r in tel]
        if z_tel:
            min_alt_tel = min(z_tel)
            log.info("Min physics altitude: %.2f m  (limit=%.1f m)",
                     min_alt_tel, _MIN_ALT_M)
            assert min_alt_tel >= _MIN_ALT_M, (
                f"Hub crashed: min physics altitude {min_alt_tel:.2f} m < {_MIN_ALT_M:.1f} m\n"
                f"STATUSTEXT: {all_statustext}"
            )

        # -- Phase split ------------------------------------------------------
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

        # -- CRITICAL log check -----------------------------------------------
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
