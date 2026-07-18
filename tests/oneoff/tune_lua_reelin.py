"""tune_lua_reelin.py -- fast inner-loop gain sweep for the Lua reel-in divergence.

One-off diagnostic (NOT a unit test). Runs a single reel_out->reel_in transition
of the Lua pumping harness (rawes.lua steady mode + GovernedWinchController) and reports
peak body-rate magnitude, slack samples, min altitude and peak tension for a given
set of ATC rate-PID gains. Lets us sweep gains in ~15 s instead of the ~150 s full
3-cycle test.

Usage:
    python tests/oneoff/tune_lua_reelin.py
    (edit the SWEEP list below)
"""
import sys
from pathlib import Path

import numpy as np

_SIM = Path(__file__).resolve().parents[2]

from simulation.winch import GovernedWinchController
from tests.simtests.simtest_ic import load_ic
from tests.simtests.simtest_runner import PhysicsRunner
from tests.common.mock_ardupilot import MockArdupilot
from arduloop import HeliParams, RateAxisParams
from simulation.pumping_planner import TensionCommand
from simulation.rawes_lua_harness import RawesLua
from simulation.rawes_modes import MODE_STEADY
from simulation.unified_ground import LuaComms
from tests.simtests._rotor_helpers import load_default_rotor

_IC = load_ic()
_ROTOR = load_default_rotor()

DT = 1.0 / 400.0
DT_PLANNER = 1.0 / 10.0
LUA_EVERY = round(0.020 / DT)
WIND = np.array([0.0, 10.0, 0.0])
WIND.flags.writeable = False

DELTA_L = 12.0
V_CRUISE = 0.5
TENSION_REEL_OUT = 300.0
TENSION_REEL_IN = 100.0
T_REEL_OUT_MAX = 120.0
BREAK_LOAD_N = 620.0
FLOOR_ALT_M = 1.0


def _heli_params(P, I, D, FF, FLTD, SMAX, hz):
    rate = RateAxisParams(P=P, I=I, D=D, FF=FF, IMAX=0.0,
                          FLTT=20.0, FLTE=0.0, FLTD=FLTD, SMAX=SMAX)
    hp = HeliParams(loop_rate_hz=hz)
    hp.roll = rate
    hp.pitch = rate
    return hp


def run(P=0.02, I=0.10, D=0.0, FF=0.0, FLTD=10.0, SMAX=0.0,
        t_end=58.0, label=""):
    sim = RawesLua(mode=MODE_STEADY)
    sim.armed = True
    sim.healthy = True
    sim.vehicle_mode = 4
    sim.pos_ned = _IC.pos.tolist()
    sim.vel_ned = _IC.vel.tolist()
    sim.R = _IC.R0
    sim.gyro = [0.0, 0.0, 0.0]

    runner = PhysicsRunner(_ROTOR, _IC, WIND, aero_model="quasi_static")
    ic_alt = float(-_IC.pos[2])

    winch = GovernedWinchController(
        rest_length=_IC.rest_length, v_max_out=1.5, v_max_in=1.5,
        kp_tension=4.0e-4, accel_limit_ms2=2.0, jerk_limit_ms3=10.0,
        tension_tau_s=0.08, min_length=2.0,
    )
    comms = LuaComms(sim.send_named_float)
    lua = MockArdupilot.for_lua(sim, initial_col_rad=_IC.coll_eq_rad, wind=WIND, dt=DT)
    # Override rate-PID gains for this sweep point.
    hz = round(1.0 / DT)
    lua.enable_guided(_heli_params(P, I, D, FF, FLTD, SMAX, hz))
    lua._ctrl = lua._guided_ctrl

    phase = "reel_out"
    phase_start_t = 0.0
    start_length = _IC.rest_length

    def ten_target():
        return TENSION_REEL_IN if phase == "reel_in" else TENSION_REEL_OUT

    def cmd_phase():
        return "reel-in" if phase == "reel_in" else "reel-out"

    comms.send(TensionCommand(tension_target_n=ten_target(), alt_m=ic_alt,
                              phase=cmd_phase()), DT_PLANNER)
    runner._core.warm_inflow(collective_rad=float(_IC.coll_eq_rad), n_steps=500)

    planner_every = max(1, round(DT_PLANNER / DT))
    prev_accel = None
    peak_omega = 0.0
    peak_omega_out = 0.0  # peak during reel_out only
    slack = 0
    min_alt = 1e9
    max_ten = 0.0
    max_crosswind = 0.0   # |North| component; wind is +Y so crosswind = pos_x
    reel_in_t = None
    floor = False
    start_length = _IC.rest_length
    cycle = 0

    n = int(t_end / DT)
    for i in range(n):
        t = i * DT
        tension_now = runner.tension_now
        t_in_phase = t - phase_start_t

        prev_phase = phase
        if phase == "reel_out" and (winch.rest_length >= start_length + DELTA_L - 0.05
                                    or t_in_phase >= T_REEL_OUT_MAX):
            phase = "reel_in"
            phase_start_t = t
            if reel_in_t is None:
                reel_in_t = t
        elif phase == "reel_in" and (winch.rest_length <= start_length + 0.05
                                     or t_in_phase >= 300.0):
            phase = "reel_out"
            phase_start_t = t
            cycle += 1

        if i % planner_every == 0 or phase != prev_phase:
            cruise = V_CRUISE if phase == "reel_out" else -V_CRUISE
            winch.set_command(cruise, ten_target())
            comms.send(TensionCommand(tension_target_n=ten_target(), alt_m=ic_alt,
                                      phase=cmd_phase()), DT_PLANNER)

        if i % LUA_EVERY == 0:
            lua.tick(t, runner, accel_ned=prev_accel)
        winch.step(tension_now, DT)
        sr = lua.step(runner, DT, rest_length=winch.rest_length)
        prev_accel = sr.get("accel_specific_world")

        om = float(np.linalg.norm(runner.hub_state["omega"]))
        peak_omega = max(peak_omega, om)
        if phase == "reel_out":
            peak_omega_out = max(peak_omega_out, om)
        if runner.tether._last_info.get("slack", False):
            slack += 1
        alt = -runner.hub_state["pos"][2]
        min_alt = min(min_alt, alt)
        max_ten = max(max_ten, runner.tension_now)
        max_crosswind = max(max_crosswind, abs(float(runner.hub_state["pos"][0])))
        if runner.hub_state["pos"][2] >= -FLOOR_ALT_M:
            floor = True
            break

    print(f"[{label}] P={P:.3f} D={D:.3f} FLTD={FLTD:.0f} SMAX={SMAX:.1f} "
          f"| peak_om={peak_omega:.2f} (out={peak_omega_out:.2f}) "
          f"slack={slack} max_cw={max_crosswind:.1f} min_alt={min_alt:.1f} "
          f"max_ten={max_ten:.0f} cyc={cycle} floor={floor}")
    return dict(peak_omega=peak_omega, slack=slack, min_alt=min_alt,
               max_ten=max_ten, max_cw=max_crosswind, floor=floor)


if __name__ == "__main__":
    # Multi-cycle stability sweep (P>=0.10 keeps crosswind drift bounded;
    # P=0.06 blows up by cycle 2).  P=0.15 is the chosen gain.  The rate-gated
    # collective vz-damping keeps the flight-start tension transient < 620 N.
    SWEEP = [
        dict(P=0.10, D=0.01, FLTD=10.0, SMAX=0.0, label="P10"),
        dict(P=0.15, D=0.02, FLTD=10.0, SMAX=0.0, label="P15"),
        dict(P=0.20, D=0.03, FLTD=15.0, SMAX=0.0, label="P20"),
    ]
    for s in SWEEP:
        run(t_end=95.0, **s)
