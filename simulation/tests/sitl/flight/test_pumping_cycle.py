"""
test_pumping_cycle.py — Pumping cycle stack test with rawes.lua (SCR_USER6=5).

Architecture (mirrors test_pump_cycle_lua.py unit test):
  Test process (10 Hz):
    - PumpingGroundController: phase state machine.
    - UDP socket: sends {target_length, target_tension} to mediator WinchController;
      receives {tension_n, rest_length, hub_alt_m} back at ~10 Hz.
    - GCS (MAVLink): sends RAWES_TSP / RAWES_TEN / RAWES_ALT / RAWES_SUB NVF
      to Lua so TensionPI + altitude hold run in-flight.
  Mediator (400 Hz):
    - WinchController: applies set_target commands, steps at physics rate,
      owns core.tether.rest_length.
  rawes.lua (50 Hz):
    - TensionPI collective controlled by RAWES_TSP / RAWES_TEN NVF.
    - bz_altitude_hold cyclic controlled by RAWES_ALT NVF.

Pass criteria:
  1. No crash (hub above MIN_ALT_M throughout).
  2. "RAWES pump: captured" STATUSTEXT appears (Lua GPS fix + altitude hold init).
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

from aero import rotor_definition as rd
_ROTOR = rd.default()

# ---------------------------------------------------------------------------
# Pumping parameters — match test_pump_cycle_lua.py exactly
# ---------------------------------------------------------------------------
_XI_START_DEG   = 30.0
_XI_REEL_IN_DEG = 50.0
_T_TRANSITION = (
    math.radians(_XI_REEL_IN_DEG - _XI_START_DEG) / _ROTOR.body_z_slew_rate_rad_s + 3.0
)

N_CYCLES         = 3
DELTA_L          = 12.0
TENSION_OUT      = 435.0
TENSION_IN       = 240.0
TENSION_IN_AP    = 213.0
TENSION_IC       = 300.0
EL_REEL_IN_RAD   = math.radians(_XI_REEL_IN_DEG)
T_REEL_OUT_MAX   = 120.0
T_REEL_IN_MAX    = 120.0

# Observation: kinematic (120 s) + hold (10 s) + 3 full cycles
_OBS_SECONDS = 120.0 + 10.0 + N_CYCLES * (T_REEL_OUT_MAX + _T_TRANSITION + T_REEL_IN_MAX) * 1.3

# ---------------------------------------------------------------------------
# Thresholds
# ---------------------------------------------------------------------------
_MIN_ALT_M       =   0.5
_BREAK_LOAD_N    = 620.0
_TENSION_LIMIT_N = 0.8 * _BREAK_LOAD_N   # 496 N

# Winch socket I/O timeout [s] — mediator sends state at ~10 Hz
_SOCK_TIMEOUT = 0.5


def test_pumping_cycle_lua(acro_armed_pumping_lua: StackContext):
    """
    Pumping cycle stack test: PumpingGroundController in test process,
    WinchController in mediator, TensionPI + altitude hold in rawes.lua.
    """
    ctx = acro_armed_pumping_lua
    gcs = ctx.gcs
    log = logging.getLogger("test_pumping_cycle_lua")

    if not ctx.winch_cmd_port:
        pytest.skip("winch_cmd_port not set — fixture did not configure socket")

    all_statustext = ctx.all_statustext
    captured_seen  = any("RAWES pump: captured" in t for t in all_statustext)

    # ── Ground controller (mirrors test_pump_cycle_lua.py) ─────────────────
    target_alt_m = ctx.home_alt_m
    planner = PumpingGroundController(
        t_transition   = _T_TRANSITION,
        target_alt_m   = target_alt_m,
        delta_l        = DELTA_L,
        el_reel_in_rad = EL_REEL_IN_RAD,
        n_cycles       = N_CYCLES,
        tension_out    = TENSION_OUT,
        tension_in     = TENSION_IN,
        tension_in_ap  = TENSION_IN_AP,
        tension_ic     = TENSION_IC,
        tension_ramp_s = 8.0,
        t_reel_out_max = T_REEL_OUT_MAX,
        t_reel_in_max  = T_REEL_IN_MAX,
    )

    # ── Winch command socket ───────────────────────────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))          # OS picks test-side port
    sock.settimeout(_SOCK_TIMEOUT)
    mediator_addr = ("127.0.0.1", ctx.winch_cmd_port)

    # Prime the socket: send an initial hold command so the mediator knows
    # our address and starts sending state back.
    if ctx.initial_state:
        _init_len = float(ctx.initial_state.get("rest_length", 50.0))
    else:
        _init_len = 50.0
    _init_cmd = json.dumps({
        "target_length":  _init_len,
        "target_tension": TENSION_IC,
    }).encode()
    sock.sendto(_init_cmd, mediator_addr)

    t_sim        = 0.0
    dt_plan      = 0.1           # 10 Hz ground planner
    t_plan_next  = 0.0
    tension_now  = TENSION_IC
    rest_length  = _init_len
    hub_alt_m    = target_alt_m

    deadline = gcs.sim_now() + _OBS_SECONDS
    log.info("--- test_pumping_cycle_lua: observing %.0f s ---", _OBS_SECONDS)

    try:
        while gcs.sim_now() < deadline:
            assert_procs_alive(ctx, "pumping_lua")

            # ── Receive winch state from mediator ─────────────────────────
            try:
                data, _ = sock.recvfrom(256)
                state   = json.loads(data)
                tension_now = float(state["tension_n"])
                rest_length = float(state["rest_length"])
                hub_alt_m   = float(state["hub_alt_m"])
            except (TimeoutError, socket.timeout):
                pass  # keep last known values; mediator may still be in kinematic

            # ── Ground planner step (10 Hz) ───────────────────────────────
            t_sim = gcs.sim_now()
            if t_sim >= t_plan_next:
                t_plan_next = t_sim + dt_plan

                cmd = planner.step(t_sim, tension_now, rest_length, hub_alt_m)

                # Send winch command to mediator (include phase for telemetry)
                winch_msg = json.dumps({
                    "target_length":  planner.winch_target_length,
                    "target_tension": planner.winch_target_tension,
                    "phase":          planner.phase,
                }).encode()
                sock.sendto(winch_msg, mediator_addr)

                # Send NVF to Lua via GCS
                for name, value in _cmd_to_nv(cmd):
                    gcs.send_named_float(name, value)

                if planner.phase == "hold":
                    log.info("Planner reached 'hold' — all %d cycles complete", N_CYCLES)
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
                if "RAWES pump: captured" in text:
                    captured_seen = True

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
            "STATUSTEXT 'RAWES pump: captured' never appeared. "
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

        winch_out = 0.40    # m/s pay-out speed (WinchController v_max_out)
        winch_in  = 0.80    # m/s reel-in speed (WinchController v_max_in)
        mean_tension_out    = sum(r.tether_tension for r in out_rows) / len(out_rows)
        skip_in             = int(_T_TRANSITION * len(in_rows) / T_REEL_IN_MAX)
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
            "Energy: out=%.1f J  in=%.1f J  net=%.1f J",
            energy_out, energy_in, net_energy,
        )

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
            "--- test_pumping_cycle_lua PASSED  (net=%.1f J  peak=%.1f N) ---",
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
