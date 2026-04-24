"""
test_pump_cycle.py -- De Schutter pumping cycle repeated N times, no landing.

Sequence
--------
  N_CYCLES repeats of the De Schutter reel-out / reel-in cycle.

  Each cycle (~60 s):
    Reel-out: tether 50 m -> 62 m, body_z tether-aligned, high tension (~200 N).
    Reel-in:  body_z tilts to xi=80 deg (nearly vertical), tether 62 m -> 50 m,
              low tension (~55 N).

  DeschutterPlanner loops automatically via modulo on its internal clock.
  No landing phase.  Run ends after N_CYCLES * (T_REEL_OUT + T_REEL_IN) seconds.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(600)]

import rotor_definition as rd
from controller      import (
    AltitudeHoldController,
    TensionPI,
)
from planner         import DeschutterPlanner, WindEstimator
from winch           import WinchController
from simtest_log     import SimtestLog, BadEventLog
from simtest_ic      import load_ic
from simtest_runner  import PhysicsRunner
from tel             import make_tel
from telemetry_csv   import TelRow, write_csv

_log   = SimtestLog(__file__)
_IC    = load_ic()
_ROTOR = rd.default()
# ── Simulation constants ───────────────────────────────────────────────────────
DT         = 1.0 / 400.0
DT_PLANNER = 1.0 / 10.0   # planner runs at 10 Hz (matches MAVLink RC override rate)
WIND       = np.array([0.0, 10.0, 0.0])   # NED: East wind
WIND.flags.writeable = False
BREAK_LOAD_N = 620.0

# ── Pumping cycle parameters (De Schutter) ────────────────────────────────────
N_CYCLES    = 3

T_REEL_OUT   = 30.0    # s
T_REEL_IN    = 30.0    # s
V_REEL_OUT   =  0.4    # m/s
V_REEL_IN    =  0.4    # m/s

XI_REEL_IN_DEG   = 80.0
BODY_Z_SLEW_RATE = _ROTOR.body_z_slew_rate_rad_s

_xi_start_deg = 30.0
T_TRANSITION  = math.radians(XI_REEL_IN_DEG - _xi_start_deg) / BODY_Z_SLEW_RATE + 1.5

TENSION_OUT    = 435.0   # N  (near equilibrium at IC elevation ~16 deg)
TENSION_IN     = 226.0   # N  (achievable minimum at ~16 deg elevation with AltHoldCtrl)

COL_MIN_RAD         = -0.28
COL_MAX_RAD         =  0.10
TENSION_SAFETY_N    = 496.0   # N (~80% break load)

FLOOR_ALT_M  = 1.0
T_CYCLE      = T_REEL_OUT + T_REEL_IN


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def _run_pumping_repeated() -> dict:
    runner = PhysicsRunner(_ROTOR, _IC, WIND)
    winch  = WinchController(rest_length=_IC.rest_length,
                              tension_safety_n=TENSION_SAFETY_N,
                              min_length=2.0)
    trajectory = DeschutterPlanner(
        t_reel_out     = T_REEL_OUT,
        t_reel_in      = T_REEL_IN,
        t_transition   = T_TRANSITION,
        v_reel_out     = V_REEL_OUT,
        v_reel_in      = V_REEL_IN,
        tension_out    = TENSION_OUT,
        tension_in     = TENSION_IN,
        wind_estimator = WindEstimator(seed_wind_ned=WIND),
        col_min_rad    = COL_MIN_RAD,
        col_max_rad    = COL_MAX_RAD,
        xi_reel_in_deg = None,
    )
    alt_ctrl   = AltitudeHoldController.from_pos(_IC.pos, BODY_Z_SLEW_RATE)
    # TensionPI runs at full physics rate (400 Hz) — planner supplies setpoint at 10 Hz.
    tension_pi = TensionPI(setpoint_n=TENSION_OUT, warm_coll_rad=_IC.coll_eq_rad)

    t_end_sim = N_CYCLES * T_CYCLE
    max_steps = int(t_end_sim / DT) + 1

    # Per-cycle energy tracking
    cycle_energy_out = [0.0] * N_CYCLES
    cycle_energy_in  = [0.0] * N_CYCLES
    events    = BadEventLog()
    telemetry = []
    tel_every     = max(1, int(0.05 / DT))
    planner_every = max(1, round(DT_PLANNER / DT))   # steps between planner calls
    pump_cmd: "dict | None" = None
    collective_rad = _IC.coll_eq_rad

    t_sim = 0.0
    for i in range(max_steps):
        t_sim = i * DT
        if t_sim >= t_end_sim:
            break

        hub_state   = runner.hub_state
        tension_now = runner.tension_now
        altitude    = -hub_state["pos"][2]
        cycle_idx   = min(int(t_sim / T_CYCLE), N_CYCLES - 1)

        # ── DeschutterPlanner at 10 Hz: provides setpoint + winch + altitude target ─
        if pump_cmd is None or i % planner_every == 0:
            obs = runner.observe()
            state_pkt = {
                "pos_ned":         obs.pos,
                "vel_ned":         obs.vel,
                "omega_spin":      obs.omega_spin,
                "body_z":          obs.body_z,
                "tension_n":       tension_now,
                "tether_length_m": winch.tether_length_m,
            }
            pump_cmd = trajectory.step(state_pkt, DT_PLANNER)
            # Propagate tension setpoint and collective floor to TensionPI
            tension_pi.setpoint = pump_cmd["tension_setpoint"]
            tension_pi.coll_min = pump_cmd["col_min_rad"]

        winch.step(pump_cmd["winch_speed_ms"], tension_now, DT)

        # TensionPI at 400 Hz: reads actual tension, outputs collective target
        collective_rad = tension_pi.update(tension_now, DT)

        pump_phase = pump_cmd["phase"]   # "reel-out" | "reel-in" | "hold"
        body_z_eq  = alt_ctrl.update(hub_state["pos"], pump_cmd["target_alt_m"],
                                     tension_now, _ROTOR.mass_kg, DT)
        if pump_phase == "reel-out":
            cycle_energy_out[cycle_idx] += tension_now * V_REEL_OUT * DT
        elif pump_phase == "reel-in":
            cycle_energy_in[cycle_idx]  += tension_now * V_REEL_IN  * DT

        # ── 400 Hz physics step (tether + attitude + aero + spin + dynamics) ──
        sr = runner.step(DT, collective_rad, body_z_eq,
                         rest_length=winch.rest_length)

        # ── Bad-event tracking ────────────────────────────────────────────────
        tel_phase_label = f"cycle{cycle_idx+1}_{pump_phase}"
        if runner.tether._last_info.get("slack", False):
            events.record("slack", t_sim, tel_phase_label, altitude,
                          tension=runner.tension_now)
        if runner.tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, tel_phase_label, altitude,
                          tension=runner.tension_now)
        if runner.hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t_sim, tel_phase_label, altitude)

        # ── Telemetry (20 Hz) ─────────────────────────────────────────────────
        if i % tel_every == 0:
            telemetry.append(make_tel(
                t_sim, runner.hub_state, runner.omega_spin,
                runner.tether, runner.tension_now,
                collective_rad, sr["tilt_lon"], sr["tilt_lat"], WIND,
                body_z_eq=body_z_eq, phase=tel_phase_label,
                aero_result=sr["aero_result"], aero_obj=runner.aero,
                tether_force=sr["tether_force"], tether_moment=sr["tether_moment"],
                tension_setpoint=pump_cmd.get("tension_setpoint", 0.0),
                collective_from_tension_ctrl=collective_rad,
                target_alt_m=pump_cmd.get("target_alt_m", 0.0),
            ))

    # ── Results ───────────────────────────────────────────────────────────────
    net_per_cycle = [cycle_energy_out[k] - cycle_energy_in[k] for k in range(N_CYCLES)]
    total_net     = sum(net_per_cycle)

    cycle_summary = "  ".join(
        f"c{k+1}={net_per_cycle[k]:.0f}J" for k in range(N_CYCLES)
    )
    parts = [
        f"total_net={total_net:.0f}J",
        cycle_summary,
        f"t_end={t_sim:.1f}s",
        events.summary(),
    ]

    if telemetry:
        write_csv([TelRow.from_tel(d) for d in telemetry],
                  _log.log_dir / "telemetry.csv")
    _log.write(["(telemetry: telemetry.csv)"],
               "  ".join(p for p in parts if p))

    return dict(
        t_end            = t_sim,
        cycle_energy_out = cycle_energy_out,
        cycle_energy_in  = cycle_energy_in,
        net_per_cycle    = net_per_cycle,
        total_net        = total_net,
        events           = events,
        floor_hits       = events.count("floor_hit"),
        slack_events     = events.count("slack"),
        tension_spikes   = events.count("tension_spike"),
        telemetry        = telemetry,
    )


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_pumping_repeated():
    """N De Schutter cycles: no bad events, every cycle positive, total net > 0."""
    r = _run_pumping_repeated()
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    for k, net in enumerate(r["net_per_cycle"]):
        if net <= 0:
            failures.append(f"cycle {k+1} net={net:.1f}J <= 0")
    if r["total_net"] <= 0:
        failures.append(f"total_net={r['total_net']:.1f}J <= 0")
    assert not failures, "\n  ".join(failures)
