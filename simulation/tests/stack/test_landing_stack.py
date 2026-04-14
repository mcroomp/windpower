"""
test_landing_stack.py -- Lua landing mode full-stack test (SITL + ArduPilot).

rawes.lua SCR_USER6=4 landing mode: Lua controls cyclic (orbit tracking) and
collective (VZ descent-rate controller) entirely via ArduPilot RC overrides.
The mediator runs the winch only (LandingPlanner.step for winch_speed_ms).

Landing sequence:
  descent   -- body_z fixed at capture orientation (xi~80 deg); winch reels in
               while TensionPI maintains ~80 N; Lua VZ controller holds
               v_land=0.5 m/s (encoded on Ch7).  Ends when tether <= 2 m.
  final_drop -- Lua sets collective=0; hub free-falls onto catch pad.

Timing (from mediator start):
  t=0..65 s  kinematic phase (hub locked to pos0=[0,3.47,-19.70] NED, xi=80 deg)
  t~15 s     arm fires; fixture yields; SCR_USER6=4 set; Lua enters settle wait
  t~62 s     KINEMATIC_SETTLE_MS expires; Lua captures body_z with converged EKF
  t~67..102s Lua VZ descent (17.7 m at 0.5 m/s = ~35 s)
  t~102 s    final_drop and floor contact

Telemetry columns used (TelRow fields):
  t_sim, pos_z, tether_rest_length, tether_tension, phase
"""

import logging
import sys
import time
from pathlib import Path

_STACK_DIR = Path(__file__).resolve().parent
_SIM_DIR   = _STACK_DIR.parent.parent
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_STACK_DIR))

from conftest import StackContext, dump_startup_diagnostics
from telemetry_csv import read_csv

# ---------------------------------------------------------------------------
# Test parameters
# ---------------------------------------------------------------------------

# Observation window after fixture yield.
# startup_damp_seconds=65: kinematic runs until t=65 s.
# GPS origin: t~20 s.  GPS fusion: t~37-54 s.
# Fixture yields at t~15 s (arm complete); test observes from there.
# Lua KINEMATIC_SETTLE_MS=62000: capture at t~62 s (3 s before kinematic exit).
# Descent: 17.7 m at 0.5 m/s = 35 s; final_drop at t~102 s (87 s into obs window).
# 165 s window ends at mediator t~180 s -> 78 s margin after final_drop.
_OBS_SECONDS      = 165.0

# Hub altitude limit above which it must stay during descent (physics safety).
# Landing starts at altitude=20 m; should descend, not rise.
_MAX_ALT_M        = 25.0

# Hub altitude below which "floor reached" is declared.
# final_drop phase ends with hub hitting the catch pad; use 1.5 m above anchor.
_FLOOR_ALT_M      = 2.5

# Tether tension limit during landing descent [N].
# 80% of Dyneema SK75 1.9 mm break load (620 N).
_TENSION_LIMIT_N  = 496.0

# ---------------------------------------------------------------------------
# Lua landing test
# ---------------------------------------------------------------------------

def test_landing_lua(acro_armed_landing_lua: StackContext):
    """
    Lua landing mode (SCR_USER6=4) stack test.

    rawes.lua controls cyclic (orbit tracking) and collective (VZ descent-rate
    controller) entirely via ArduPilot RC overrides.  The mediator only runs
    the winch (LandingPlanner.step for winch_speed_ms); collective+attitude
    come from Lua.

    Asserts (same as test_landing but via the Lua path):
      1. "RAWES land: captured" STATUSTEXT appears (Lua captured body_z).
      2. "RAWES land: final_drop" STATUSTEXT appears (tether reached min length).
      3. Hub reaches floor (altitude <= _FLOOR_ALT_M).
      4. Peak tension during descent < _TENSION_LIMIT_N.
      5. Hub does not rise above _MAX_ALT_M.
      6. No CRITICAL errors in mediator log.
    """
    ctx = acro_armed_landing_lua
    gcs = ctx.gcs
    log = logging.getLogger("test_landing_lua")

    # Pre-populate from STATUSTEXT captured during fixture setup (e.g. drain loop)
    captured_seen   = any("RAWES land: captured"    in t for t in ctx.all_statustext)
    final_drop_seen = any("RAWES land: final_drop"  in t for t in ctx.all_statustext)

    t_obs_start = time.monotonic()
    deadline    = t_obs_start + _OBS_SECONDS

    log.info("--- test_landing_lua: observing %.0f s ---", _OBS_SECONDS)

    try:
        while time.monotonic() < deadline:
            # Check processes haven't died
            for name, proc, lp in [
                ("mediator", ctx.mediator_proc, ctx.mediator_log),
                ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
            ]:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    raise RuntimeError(
                        f"{name} exited during landing_lua test (rc={proc.returncode}):\n{txt[-3000:]}"
                    )

            msg = gcs._mav.recv_match(
                type=["STATUSTEXT", "LOCAL_POSITION_NED"],
                blocking=True, timeout=0.1,
            )
            if msg is None:
                continue

            mt = msg.get_type()
            if mt == "STATUSTEXT":
                text = msg.text.rstrip("\x00").strip()
                ctx.all_statustext.append(text)
                log.info("STATUSTEXT: %s", text)
                if "RAWES land: captured" in text:
                    captured_seen = True
                    log.info("  -> body_z captured by Lua landing mode")
                if "RAWES land: final_drop" in text:
                    final_drop_seen = True
                    log.info("  -> Lua final_drop triggered")

        # ── Parse telemetry CSV ────────────────────────────────────────────
        tel = read_csv(ctx.telemetry_log)
        log.info("Telemetry: %d rows", len(tel))

        if not tel:
            raise RuntimeError(
                "No mediator telemetry CSV -- cannot assert landing behaviour. "
                "Ensure mediator is writing telemetry."
            )

        # ── STATUSTEXT assertions ─────────────────────────────────────────
        assert captured_seen, (
            "STATUSTEXT 'RAWES land: captured' never appeared. "
            "Lua landing mode may not have activated (SCR_USER6=4) or "
            "the kinematic phase did not exit before the observation window. "
            f"All STATUSTEXT seen: {ctx.all_statustext}"
        )

        assert final_drop_seen, (
            "STATUSTEXT 'RAWES land: final_drop' never appeared. "
            "Tether may not have reached min_tether_m=2.0 m before observation ended. "
            "Try increasing _OBS_SECONDS or check Lua tether length geometry. "
            f"All STATUSTEXT seen: {ctx.all_statustext}"
        )

        # ── Altitude: descent direction + floor reached ───────────────────
        altitudes = [-r.pos_z for r in tel]
        max_alt   = max(altitudes) if altitudes else 0.0
        min_alt   = min(altitudes) if altitudes else 0.0

        log.info("Altitude range: %.1f m .. %.1f m  (floor=%.1f m)",
                 min_alt, max_alt, _FLOOR_ALT_M)

        assert max_alt <= _MAX_ALT_M, (
            f"Hub rose to {max_alt:.1f} m (limit {_MAX_ALT_M:.1f} m) during Lua landing. "
            "Lua landing mode may be orbiting instead of descending."
        )

        assert min_alt <= _FLOOR_ALT_M, (
            f"Hub never reached floor: min altitude {min_alt:.1f} m > {_FLOOR_ALT_M:.1f} m. "
            "Lua landing descent did not complete within the observation window."
        )

        # ── Tension: no spike during descent ─────────────────────────────
        descent_rows = [r for r in tel if r.phase == "descent"]
        if descent_rows:
            peak_tension = max(r.tether_tension for r in descent_rows)
            log.info("Peak tension during descent: %.1f N  (limit=%.1f N)",
                     peak_tension, _TENSION_LIMIT_N)
            assert peak_tension < _TENSION_LIMIT_N, (
                f"Peak tension {peak_tension:.1f} N >= 80%% break load "
                f"({_TENSION_LIMIT_N:.1f} N) during Lua descent. "
                "Winch tension control is not keeping tension in range."
            )
        else:
            log.warning("No descent rows in telemetry -- tension check skipped.")

        # ── CRITICAL log check ────────────────────────────────────────────
        if ctx.mediator_log.exists():
            lines = ctx.mediator_log.read_text(encoding="utf-8", errors="replace").splitlines()
            critical = [l for l in lines if "CRITICAL" in l]
            assert not critical, "CRITICAL errors in mediator log:\n" + "\n".join(critical[:10])

        log.info(
            "--- test_landing_lua PASSED  (min_alt=%.1f m  peak_descent_tension=%.0f N) ---",
            min_alt,
            max(r.tether_tension for r in descent_rows) if descent_rows else 0.0,
        )

    except Exception:
        dump_startup_diagnostics(ctx)
        raise
