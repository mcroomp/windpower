"""
test_pump_cycle_unified.py -- De Schutter pumping cycle using PumpingGroundController
and ThrustApController (clean ground/AP boundary).

Ground (10 Hz):  PumpingGroundController  -> ThrustCommand(thrust 0..1)
Winch  (400 Hz): WinchController          -> tether length
AP     (400 Hz): ThrustApController       -> collective + body_z_eq
                 thrust=0: tether-tilt (minimum tension, coll_min)
                 thrust=1: altitude-hold (maximum tension, coll_max)

Communication boundary:
  - Ground sends ThrustCommand to AP at 10 Hz
  - ThrustCommand carries (thrust 0..1, alt_m, phase)
  - AP slerps body_z between tether_dir and alt_hold, slews collective

Reel-out: ground ramps thrust 0->target_thrust over thrust_ramp_s
Transition: ground sends thrust=0; AP drives to minimum tension
Reel-in: ground sends thrust=0; winch waits for T < T_reel_in_start, then reels in
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(600)]

import rotor_definition as rd
from winch            import WinchController
from simtest_log      import SimtestLog, BadEventLog
from simtest_ic       import load_ic
from simtest_runner   import PhysicsRunner
from tel              import make_tel
from telemetry_csv    import TelRow, write_csv
from pumping_planner  import PumpingGroundController
from ap_controller    import ThrustApController

_log   = SimtestLog(__file__)
_IC    = load_ic()
_ROTOR = rd.default()

# ── Simulation constants ──────────────────────────────────────────────────────
DT         = 1.0 / 400.0
DT_PLANNER = 1.0 / 10.0
WIND       = np.array([0.0, 10.0, 0.0])
WIND.flags.writeable = False
BREAK_LOAD_N = 620.0

# ── Pumping cycle parameters ──────────────────────────────────────────────────
N_CYCLES     = 3
T_REEL_OUT   = 30.0
T_REEL_IN    = 30.0

# T_TRANSITION: trapezoid ramp at phase boundaries (matches DeschutterPlanner)
_XI_START_DEG    = 30.0
_XI_REEL_IN_DEG  = 80.0
T_TRANSITION = (
    math.radians(_XI_REEL_IN_DEG - _XI_START_DEG) / _ROTOR.body_z_slew_rate_rad_s + 1.5
)

COL_MIN_RAD   = -0.28
COL_MAX_RAD   =  0.10   # collective upper bound (test-level)

TENSION_SAFETY_N = 496.0
FLOOR_ALT_M      = 1.0
T_CYCLE          = T_REEL_OUT + T_REEL_IN


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def _run_pumping() -> dict:
    runner = PhysicsRunner(_ROTOR, _IC, WIND)

    T_INIT = 300.0   # IC targets 300 N tension (midway between min and max)

    # ── Ground: PumpingGroundController (10 Hz outer loop) ────────────────
    ground = PumpingGroundController(
        t_reel_out   = T_REEL_OUT,
        t_reel_in    = T_REEL_IN,
        t_transition = T_TRANSITION,
        target_alt_m = float(-_IC.pos[2]),
        n_cycles     = N_CYCLES,
        target_thrust= 0.9,
        thrust_ramp_s= 8.0,
    )

    # ── Ground: WinchController (program mode, phase-driven) ─────────────
    winch = WinchController(
        rest_length     = _IC.rest_length,
        delta_l         = 12.0,    # reel out 12 m, reel back in 12 m (symmetric)
        v_reel_out      = 0.40,    # generator optimal speed [m/s]
        v_reel_in       = 0.80,    # fast reel-in to maximise cycle rate [m/s]
        T_soft_max      = 470.0,   # start slowing on reel-out [N]
        T_hard_max      = 496.0,   # stop on reel-out (~80% break load) [N]
        T_soft_min      =  30.0,   # start slowing on reel-in (slack warning) [N]
        T_hard_min      =  10.0,   # stop on reel-in (slack) [N]
        T_reel_in_start = 200.0,   # wait for AP to reach low-tension before reeling in
        min_length      =   2.0,
    )

    # ── AP: ThrustApController (400 Hz, maps thrust -> collective + body_z) ──
    ap = ThrustApController(
        ic_pos          = _IC.pos,
        mass_kg         = _ROTOR.mass_kg,
        slew_rate_rad_s = _ROTOR.body_z_slew_rate_rad_s,
        warm_coll_rad   = _IC.coll_eq_rad,
        T_tension_est   = T_INIT,
        coll_min        = COL_MIN_RAD,
        coll_max        = COL_MAX_RAD,
    )

    t_end_sim = N_CYCLES * T_CYCLE
    max_steps = int(t_end_sim / DT) + 1

    cycle_energy_out = [0.0] * N_CYCLES
    cycle_energy_in  = [0.0] * N_CYCLES
    events    = BadEventLog()
    telemetry = []
    tel_every     = max(1, int(0.05 / DT))
    planner_every = max(1, round(DT_PLANNER / DT))

    collective_rad = _IC.coll_eq_rad
    body_z_eq      = _IC.R0[:, 2].copy()

    t_sim = 0.0
    for i in range(max_steps):
        t_sim = i * DT
        if t_sim >= t_end_sim:
            break

        hub_state   = runner.hub_state
        tension_now = runner.tension_now
        altitude    = -hub_state["pos"][2]
        cycle_idx   = min(int(t_sim / T_CYCLE), N_CYCLES - 1)

        # ── Ground 10 Hz outer: compute phase + thrust, send to AP ──────────
        if i % planner_every == 0:
            cmd = ground.step(t_sim)
            ap.receive_command(cmd)
            winch.set_phase(ground.phase)

        # ── Winch 400 Hz: execute program + safety layer ──────────────────
        len_before = winch.rest_length
        winch.step(tension_now, DT)
        speed_now = (winch.rest_length - len_before) / DT

        # ── AP 400 Hz: collective + body_z from thrust ────────────────────
        obs = runner.observe()
        collective_rad, body_z_eq = ap.step(obs.pos, obs.body_z, DT)

        # ── Physics 400 Hz ───────────────────────────────────────────────
        sr = runner.step(DT, collective_rad, body_z_eq, rest_length=winch.rest_length)

        # ── Per-cycle energy accounting ───────────────────────────────────
        if speed_now > 0.0:
            cycle_energy_out[cycle_idx] += runner.tension_now * speed_now * DT
        elif speed_now < 0.0:
            cycle_energy_in[cycle_idx]  += runner.tension_now * abs(speed_now) * DT

        # ── Bad-event tracking ────────────────────────────────────────────
        phase_label = f"cycle{cycle_idx+1}_{ground.phase}"
        if runner.tether._last_info.get("slack", False):
            events.record("slack", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t_sim, phase_label, altitude)

        # ── Telemetry 20 Hz ───────────────────────────────────────────────
        if i % tel_every == 0:
            telemetry.append(make_tel(
                t_sim, runner.hub_state, runner.omega_spin,
                runner.tether, runner.tension_now,
                collective_rad, sr["tilt_lon"], sr["tilt_lat"], WIND,
                body_z_eq         = body_z_eq,
                phase             = phase_label,
                aero_result       = sr["aero_result"],
                aero_obj          = runner.aero,
                tether_force      = sr["tether_force"],
                tether_moment     = sr["tether_moment"],
                tension_setpoint  = ap.drive,
                collective_from_tension_ctrl = collective_rad,
                target_alt_m      = ap._target_alt,
                thrust_cmd        = ap.thrust,
                winch_speed_ms    = speed_now,
                raw_coll          = ap.raw_coll,
                tilt_frac         = ap.tilt_frac,
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

def test_pumping_unified():
    """3 pumping cycles via PumpingGroundController/ThrustApController: no bad events, net > 0."""
    r = _run_pumping()
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    for k, net in enumerate(r["net_per_cycle"]):
        if net <= 0:
            failures.append(f"cycle {k+1} net={net:.1f}J <= 0")
    if r["total_net"] <= 0:
        failures.append(f"total_net={r['total_net']:.1f}J <= 0")
    assert not failures, "\n  ".join(failures)
