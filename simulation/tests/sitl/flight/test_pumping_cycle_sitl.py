"""
test_pumping_cycle_sitl.py — Pumping cycle stack test with rawes.lua (SCR_USER6=1, steady mode).

Architecture (mirrors test_pump_cycle_lua.py unit test):
  Test process (10 Hz):
    - PumpingGroundController: phase state machine (identical to the simtest).
    - UDP socket = the winch cable: sends WinchCommand {cruise_v, tension_target}
      to the mediator's GovernedWinchNode; receives WinchTelemetry
      {tension_n, rest_length, speed_ms, net_energy_j, wind_ned} back at ~10 Hz.
      No hub data crosses this boundary.
        - GCS (MAVLink): sends RAWES_TEN / RAWES_ALT / RAWES_SUB NVF
            to Lua so measured-tension feed-forward + altitude hold run in-flight.
  Mediator (400 Hz):
    - GovernedWinchNode hosts the fast tension-governing loop (same control law
      as the simtest), owns core.tether.rest_length.
  rawes.lua (50 Hz):
        - Altitude PID collective controlled by RAWES_ALT NVF.
        - Rate-only bz_altitude_hold cyclic uses RAWES_TEN feed-forward.

Pass criteria:
  1. No crash (hub above MIN_ALT_M throughout).
  2. "RAWES steady: captured" STATUSTEXT appears (Lua GPS fix + altitude hold init).
  3. Reel-in steady tension < reel-out mean tension (De Schutter mechanism works).
  4. Net energy positive.
  5. Peak tension < 80% break load (496 N).
  6. No CRITICAL errors in mediator log.
"""

import json
import logging
import math
import socket
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.sitl

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import (
    StackContext, dump_startup_diagnostics,
    assert_no_mediator_criticals, assert_procs_alive,
)
from telemetry_csv import read_csv
from pumping_planner import PumpingGroundController
from unified_ground import _cmd_to_nv
from tests.simtests._rotor_helpers import load_default_rotor

_ROTOR = load_default_rotor()

# ---------------------------------------------------------------------------
# Pumping parameters — match test_pump_cycle_lua.py exactly
# ---------------------------------------------------------------------------
N_CYCLES         = 3
DELTA_L          = 12.0
TENSION_OUT      = 300.0    # reel-out / IC tension target [N] (AP FF + winch)
TENSION_IN       = 100.0    # reel-in tension target [N]       (AP FF + winch)
TENSION_IC       = 300.0
T_REEL_OUT_MAX   = 120.0
T_REEL_IN_MAX    = 120.0
V_CRUISE_OUT     =   0.5    # reel-out cruise velocity [m/s] (GovernedWinch)
V_CRUISE_IN      =   0.5    # reel-in cruise speed [m/s]
CAPTURE_SETTLE_S =   2.0    # hold after steady capture before pumping begins

# Observation: kinematic (120 s) + hold (10 s) + 3 full cycles
_OBS_SECONDS = 120.0 + 10.0 + N_CYCLES * (T_REEL_OUT_MAX + T_REEL_IN_MAX) * 1.3

# ---------------------------------------------------------------------------
# Thresholds
# ---------------------------------------------------------------------------
_MIN_ALT_M       =   0.5
_BREAK_LOAD_N    = 620.0
_TENSION_LIMIT_N = 0.8 * _BREAK_LOAD_N   # 496 N

# Winch socket I/O timeout [s] — mediator sends state at ~10 Hz
_SOCK_TIMEOUT = 0.5


def test_pumping_cycle_lua_sitl(guided_nogps_armed_pumping_lua: StackContext):
    """
    Pumping cycle stack test: PumpingGroundController in test process,
    WinchController in mediator, altitude PID + rate-only hold in rawes.lua.
    """
    ctx = guided_nogps_armed_pumping_lua
    gcs = ctx.gcs
    log = logging.getLogger("test_pumping_cycle_lua_sitl")

    if not ctx.winch_cmd_port:
        pytest.skip("winch_cmd_port not set — fixture did not configure socket")

    all_statustext = ctx.all_statustext
    captured_seen  = any("RAWES steady: captured" in t for t in all_statustext)

    # ── Start at IC: wait for kinematic exit, then promote MODE_PASSIVE -> STEADY ──
    # The shared trapezoid fixture (guided_nogps_armed_pumping_lua) brings the hub
    # to the IC at rest and leaves the Lua in MODE_PASSIVE (SCR_USER6=3) with the
    # IC operating point seeded.  Mirror test_lua_flight_steady_sitl: re-seed the
    # IC NVFs, let the passive hold settle, then promote to MODE_STEADY before the
    # pumping schedule begins.
    log.info("Waiting for kinematic phase to end before pumping ...")
    if not ctx.wait_kinematic_done(timeout=60.0):
        pytest.fail(
            "Kinematic phase did not end within 60 s.\n"
            "Check mediator log for 'TRANSITION kinematic->free-flight'."
        )

    ic = ctx.initial_state
    if ic is None:
        pytest.fail("initial_state is required for pumping test")

    eq_phys = ic.get("eq_physics")
    if isinstance(eq_phys, dict) and "collective_rad" in eq_phys:
        coll_seed = float(eq_phys["collective_rad"])
    elif "stack_coll_eq" in ic:
        coll_seed = float(ic["stack_coll_eq"])
    elif "coll_eq_rad" in ic:
        coll_seed = float(ic["coll_eq_rad"])
    else:
        raise KeyError(
            "initial_state missing collective seed; expected one of "
            "eq_physics.collective_rad, stack_coll_eq, coll_eq_rad"
        )
    ten_seed = float(ic["tension_eq_n"])
    R0 = ic.get("R0")
    if R0 is None:
        pytest.fail("initial_state missing R0 for IC passive attitude seed")
    ic_roll_rad  = math.atan2(float(R0[2][1]), float(R0[2][2]))
    ic_pitch_rad = -math.asin(max(-1.0, min(1.0, float(R0[2][0]))))

    gcs.send_named_float("RAWES_COL", coll_seed)
    gcs.send_named_float("RAWES_TEN", ten_seed)
    gcs.send_named_float("RAWES_RIC", ic_roll_rad)
    gcs.send_named_float("RAWES_PIC", ic_pitch_rad)
    gcs.set_param("SCR_USER6", 3, timeout=5.0)
    log.info("  Holding MODE_PASSIVE 10 s to settle before MODE_STEADY ...")
    gcs.sim_sleep(10.0)
    ok = gcs.set_param("SCR_USER6", 1, timeout=5.0)
    log.info("Promoted Lua to MODE_STEADY (SCR_USER6 -> 1)  ACK=%s", ok)

    # ── Ground controller (shared PumpingGroundController -- same as simtest) ──
    target_alt_m = ctx.home_alt_m
    planner = PumpingGroundController(
        target_alt_m     = target_alt_m,
        delta_l          = DELTA_L,
        n_cycles         = N_CYCLES,
        tension_out      = TENSION_OUT,
        tension_in       = TENSION_IN,
        tension_ic       = TENSION_IC,
        t_reel_out_max   = T_REEL_OUT_MAX,
        t_reel_in_max    = T_REEL_IN_MAX,
        v_cruise_out     = V_CRUISE_OUT,
        v_cruise_in      = V_CRUISE_IN,
        capture_settle_s = CAPTURE_SETTLE_S,
    )
    # The MODE_PASSIVE->STEADY promotion above already established steady capture;
    # arm the planner's capture gate so the settle countdown begins immediately.
    if captured_seen:
        planner.notify_captured(gcs.sim_now())

    # ── Winch command socket = the winch cable ─────────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))          # OS picks test-side port
    sock.settimeout(_SOCK_TIMEOUT)
    mediator_addr = ("127.0.0.1", ctx.winch_cmd_port)

    # Prime the cable: an initial hold command (cruise_v=0 at the IC tension)
    # so the mediator learns our address and starts streaming telemetry back.
    if ctx.initial_state:
        _init_len = float(ctx.initial_state.get("rest_length", 50.0))
    else:
        _init_len = 50.0
    _init_cmd = json.dumps({
        "cruise_v":       0.0,
        "tension_target": TENSION_IC,
    }).encode()
    sock.sendto(_init_cmd, mediator_addr)

    t_sim        = 0.0
    dt_plan      = 0.1           # 10 Hz ground planner
    t_plan_next  = 0.0
    tension_now  = TENSION_IC
    rest_length  = _init_len
    net_energy_j = 0.0

    deadline = gcs.sim_now() + _OBS_SECONDS
    log.info("--- test_pumping_cycle_lua_sitl: observing %.0f s ---", _OBS_SECONDS)

    try:
        while gcs.sim_now() < deadline:
            assert_procs_alive(ctx, "pumping_lua")

            # ── Receive winch telemetry from mediator (the only winch data
            #    the planner is allowed to see) ─────────────────────────────
            try:
                data, _ = sock.recvfrom(256)
                state   = json.loads(data)
                tension_now  = float(state["tension_n"])
                rest_length  = float(state["rest_length"])
                net_energy_j = float(state["net_energy_j"])
            except (TimeoutError, socket.timeout):
                pass  # keep last known values; mediator may still be in kinematic

            # ── Ground planner step (10 Hz) ───────────────────────────────
            t_sim = gcs.sim_now()
            if t_sim >= t_plan_next:
                t_plan_next = t_sim + dt_plan

                cmd = planner.step(t_sim, tension_now, rest_length)

                # Send WinchCommand down the cable (cruise velocity + tension
                # target -- the planner's winch_target_velocity drives the
                # governed winch, exactly as in the simtest).
                winch_msg = json.dumps({
                    "cruise_v":       planner.winch_target_velocity,
                    "tension_target": planner.winch_target_tension,
                }).encode()
                sock.sendto(winch_msg, mediator_addr)

                # Send NVF to Lua via GCS
                for name, value in _cmd_to_nv(cmd):
                    gcs.send_named_float(name, value)

                # All cycles complete -> planner returns to hold (mirrors simtest).
                if planner.cycle_count >= N_CYCLES:
                    log.info("Planner completed all %d cycles", N_CYCLES)
                    break

            # ── Drain MAVLink for STATUSTEXT ──────────────────────────────
            msg = gcs._recv(
                type=["STATUSTEXT"],
                blocking=False, timeout=0.01,
            )
            if msg is not None and msg.get_type() == "STATUSTEXT":
                text = msg.text.rstrip("\x00").strip()
                all_statustext.append(text)
                log.info("STATUSTEXT: %s", text)
                if "RAWES steady: captured" in text:
                    captured_seen = True
                    planner.notify_captured(gcs.sim_now())

        sock.close()

        # ── Parse telemetry CSV ───────────────────────────────────────────
        if not ctx.telemetry_log:
            pytest.skip("No telemetry log path.")
        tel = read_csv(ctx.telemetry_log)
        log.info("Telemetry: %d rows", len(tel))

        if not tel:
            pytest.skip("No mediator telemetry CSV.")

        # ── Lua capture check ─────────────────────────────────────────────
        assert captured_seen, (
            "STATUSTEXT 'RAWES steady: captured' never appeared. "
            f"All STATUSTEXT: {all_statustext}"
        )

        # ── Crash check ───────────────────────────────────────────────────
        z_tel = [-r.pos_z for r in tel]
        if z_tel:
            min_alt = min(z_tel)
            log.info("Min physics altitude: %.2f m  (limit=%.1f m)", min_alt, _MIN_ALT_M)
            assert min_alt >= _MIN_ALT_M, (
                f"Hub crashed: min altitude {min_alt:.2f} m < {_MIN_ALT_M:.1f} m\n"
                f"STATUSTEXT: {all_statustext}"
            )

        # ── Phase split (first complete cycle) ────────────────────────────
        out_rows, in_rows = _split_phases(tel)
        log.info("Phase split: reel-out=%d  reel-in=%d rows",
                 len(out_rows), len(in_rows))

        if len(out_rows) < 5 or len(in_rows) < 5:
            pytest.skip(
                f"Insufficient phase data: reel-out={len(out_rows)} "
                f"reel-in={len(in_rows)} rows."
            )

        all_t  = [r.t_sim for r in out_rows + in_rows]
        dt_tel = (all_t[-1] - all_t[0]) / max(len(all_t) - 1, 1) if len(all_t) > 1 else 0.0025

        # Governed winch cruises at the planner's reel speeds (same for out/in).
        winch_out = V_CRUISE_OUT    # m/s pay-out speed
        winch_in  = V_CRUISE_IN     # m/s reel-in speed
        mean_tension_out    = sum(r.tether_tension for r in out_rows) / len(out_rows)
        # Skip the initial reel-in transient (first ~10% of reel-in rows) so the
        # steady reel-in tension is not biased by the reversal spike.
        skip_in             = int(0.1 * len(in_rows))
        steady_in           = in_rows[skip_in:] or in_rows
        mean_tension_in     = sum(r.tether_tension for r in steady_in) / len(steady_in)
        peak_tension        = max(r.tether_tension for r in out_rows + in_rows)
        energy_out = sum(r.tether_tension * winch_out * dt_tel for r in out_rows)
        energy_in  = sum(r.tether_tension * winch_in  * dt_tel for r in in_rows)
        net_energy = energy_out - energy_in

        log.info(
            "Tension: out=%.1f N  in_steady=%.1f N  peak=%.1f N",
            mean_tension_out, mean_tension_in, peak_tension,
        )
        log.info(
            "Energy (CSV): out=%.1f J  in=%.1f J  net=%.1f J",
            energy_out, energy_in, net_energy,
        )
        # Authoritative net energy from the winch node's own accounting.
        log.info("Energy (winch node net_energy_j): %.1f J", net_energy_j)

        assert peak_tension < _TENSION_LIMIT_N, (
            f"Peak tension {peak_tension:.1f} N >= limit ({_TENSION_LIMIT_N:.1f} N)"
        )
        assert mean_tension_in < mean_tension_out, (
            f"Reel-in tension ({mean_tension_in:.1f} N) not < reel-out "
            f"({mean_tension_out:.1f} N) -- tilt mechanism not working"
        )
        assert net_energy > 0, (
            f"Net energy {net_energy:.1f} J <= 0"
        )

        assert_no_mediator_criticals(ctx.mediator_log)

        log.info(
            "--- test_pumping_cycle_lua_sitl PASSED  (net=%.1f J  peak=%.1f N) ---",
            net_energy, peak_tension,
        )

    except Exception:
        sock.close()
        dump_startup_diagnostics(ctx)
        raise


def _split_phases(rows: list) -> tuple[list, list]:
    """Split telemetry into first complete reel-out and reel-in rows."""
    out_rows, in_rows = [], []
    in_reel_out = reel_in_done = False
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
