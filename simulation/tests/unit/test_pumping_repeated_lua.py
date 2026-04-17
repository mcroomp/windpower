"""
test_pumping_repeated_lua.py -- Lua-controlled De Schutter pumping cycle, N repeats.

Mirrors test_pumping_repeated.py, replacing the Python DeschutterPlanner +
AcroController with rawes.lua (mode=5) running in-process via lupa (Lua 5.4).

Division of labour (mirrors real stack):
  Python: winch speed on a timer (trapezoid profile); writes SCR_USER6 substate
  Lua:    phase detection (tlen threshold), collective per phase,
          body_z orbit tracking (reel-out) + slerp to xi=80 deg (reel-in)

Non-Lua reference: test_pumping_repeated.py
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

import rotor_definition as rd
from dynamics      import RigidBodyDynamics
from aero          import create_aero
from tether        import TetherModel
from controller    import RatePID
from swashplate    import SwashplateServoModel
from winch         import WinchController
from frames        import build_orb_frame
from simtest_ic    import load_ic
from simtest_log   import SimtestLog, BadEventLog
from tel           import make_tel
from telemetry_csv import TelRow, write_csv
from rawes_lua_harness import RawesLua
from rawes_modes   import (
    MODE_PUMPING,
    PUMP_HOLD, PUMP_REEL_OUT, PUMP_TRANSITION, PUMP_REEL_IN, PUMP_TRANSITION_BACK,
)

_IC    = load_ic()
_ROTOR = rd.default()
_log   = SimtestLog(__file__)

DT            = 1.0 / 400.0
LUA_PERIOD    = 0.010
LUA_EVERY     = round(LUA_PERIOD / DT)
T_AERO_OFFSET = 45.0

ANCHOR = np.zeros(3)
WIND   = np.array([0.0, 10.0, 0.0])

I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

_COL_MIN    = -0.28
_COL_MAX    =  0.10
_ACRO_SCALE = 500.0 / (360.0 * math.pi / 180.0)

KP_INNER = RatePID.DEFAULT_KP

BREAK_LOAD_N = 620.0
FLOOR_ALT_M  = 1.0

T_PUMP_HOLD       = 2.0    # s: hold before first cycle (Lua capture + GPS)
T_PUMP_OUT        = 30.0   # s: reel-out duration per cycle
T_PUMP_IN         = 30.0   # s: reel-in duration per cycle
T_PUMP_TRANSITION = 3.7    # s: winch ramp at phase boundaries (matches DeschutterPlanner)
V_PUMP_OUT        =  0.4   # m/s: reel-out speed
V_PUMP_IN         =  0.4   # m/s: reel-in speed
N_PUMP_CYCLES     = 3
PUMP_SAFETY_N     = 496.0  # 80% break load

_PUMP_T_CYCLE = T_PUMP_OUT + T_PUMP_IN
_PUMP_T_END   = T_PUMP_HOLD + N_PUMP_CYCLES * _PUMP_T_CYCLE


def _pwm_to_rate(pwm: int) -> float:
    return (pwm - 1500) / _ACRO_SCALE


def _pwm_to_col(pwm: int) -> float:
    return _COL_MIN + (pwm - 1000) / 1000.0 * (_COL_MAX - _COL_MIN)


def _pump_substate(t: float) -> int:
    """Ground planner phase substate written to SCR_USER6 each step."""
    if t < T_PUMP_HOLD:
        return PUMP_HOLD
    t_into = t - T_PUMP_HOLD
    cycle_idx = int(t_into / _PUMP_T_CYCLE)
    if cycle_idx >= N_PUMP_CYCLES:
        return PUMP_HOLD
    t_cyc = t_into - cycle_idx * _PUMP_T_CYCLE
    if cycle_idx > 0 and t_cyc < T_PUMP_TRANSITION:
        return PUMP_TRANSITION_BACK
    if t_cyc < T_PUMP_OUT:
        return PUMP_REEL_OUT
    elif t_cyc < T_PUMP_OUT + T_PUMP_TRANSITION:
        return PUMP_TRANSITION
    else:
        return PUMP_REEL_IN


def _winch_speed_at(t: float) -> float:
    """Trapezoid winch schedule matching DeschutterPlanner."""
    if t < T_PUMP_HOLD:
        return 0.0
    t_into = t - T_PUMP_HOLD
    cycle_idx = int(t_into / _PUMP_T_CYCLE)
    if cycle_idx >= N_PUMP_CYCLES:
        return 0.0
    t_cyc = t_into - cycle_idx * _PUMP_T_CYCLE
    if t_cyc < T_PUMP_OUT:
        alpha_start = min(1.0, t_cyc / T_PUMP_TRANSITION)
        alpha_end   = min(1.0, (T_PUMP_OUT - t_cyc) / T_PUMP_TRANSITION)
        return V_PUMP_OUT * min(alpha_start, alpha_end)
    else:
        t_in        = t_cyc - T_PUMP_OUT
        alpha_start = min(1.0, t_in / T_PUMP_TRANSITION)
        alpha_end   = min(1.0, (T_PUMP_IN - t_in) / T_PUMP_TRANSITION)
        return -V_PUMP_IN * min(alpha_start, alpha_end)


def _run_pumping() -> dict:
    """rawes.lua mode=5 (pumping_noyaw): N_PUMP_CYCLES De Schutter reel-out/reel-in."""
    sim = RawesLua(mode=MODE_PUMPING + PUMP_HOLD)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 1
    sim.pos_ned      = _IC.pos.tolist()
    sim.vel_ned      = _IC.vel.tolist()
    sim.R            = build_orb_frame(_IC.body_z)
    sim.gyro         = [0.0, 0.0, 0.0]

    dyn    = RigidBodyDynamics(
        mass=_ROTOR.mass_kg, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=_IC.pos.tolist(), vel0=_IC.vel.tolist(),
        R0=build_orb_frame(_IC.body_z), omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(_ROTOR)
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=_IC.rest_length,
                         axle_attachment_length=_ROTOR.axle_attachment_length_m)
    winch  = WinchController(rest_length=_IC.rest_length,
                              tension_safety_n=PUMP_SAFETY_N,
                              min_length=2.0)
    pid_lon = RatePID(kp=KP_INNER)
    pid_lat = RatePID(kp=KP_INNER)
    servo   = SwashplateServoModel.from_rotor(_ROTOR)

    omega_spin = _IC.omega_spin
    col_rad    = _IC.coll_eq_rad
    roll_sp    = 0.0
    pitch_sp   = 0.0

    hub_state = dyn.state
    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now = tether._last_info.get("tension", 0.0)

    cycle_energy_out = [0.0] * N_PUMP_CYCLES
    cycle_energy_in  = [0.0] * N_PUMP_CYCLES
    events = BadEventLog()

    tel_every = max(1, int(0.05 / DT))
    telemetry = []

    max_steps = int((_PUMP_T_END + 5.0) / DT)

    t_sim = 0.0
    for i in range(max_steps):
        t_sim = i * DT
        if t_sim >= _PUMP_T_END:
            break

        hub_state = dyn.state
        altitude  = -hub_state["pos"][2]

        speed_now = _winch_speed_at(t_sim)
        sub_now   = _pump_substate(t_sim)
        sim.set_param("mode", MODE_PUMPING + sub_now)
        if t_sim >= T_PUMP_HOLD:
            t_into = t_sim - T_PUMP_HOLD
            cycle_idx = min(int(t_into / _PUMP_T_CYCLE), N_PUMP_CYCLES - 1)
        else:
            cycle_idx = 0

        if i % LUA_EVERY == 0:
            omega_body = hub_state["R"].T @ hub_state["omega"]
            sim._mock.millis_val = int(t_sim * 1000)
            sim.R       = hub_state["R"]
            sim.pos_ned = hub_state["pos"].tolist()
            sim.vel_ned = hub_state["vel"].tolist()
            sim.gyro    = omega_body.tolist()
            sim._update_fn()

            ch1 = sim.ch_out[1]
            ch2 = sim.ch_out[2]
            ch3 = sim.ch_out[3]
            if ch1 is not None: roll_sp  = _pwm_to_rate(ch1)
            if ch2 is not None: pitch_sp = _pwm_to_rate(ch2)
            if ch3 is not None: col_rad  = _pwm_to_col(ch3)

        winch.step(speed_now, tension_now, DT)
        tether.rest_length = winch.rest_length

        omega_body    = hub_state["R"].T @ hub_state["omega"]
        omega_body[2] = 0.0
        tilt_lon_cmd  =  pid_lon.update(roll_sp,  omega_body[0], DT)
        tilt_lat_cmd  = -pid_lat.update(pitch_sp, omega_body[1], DT)
        tilt_lon, tilt_lat = servo.step(tilt_lon_cmd, tilt_lat_cmd, DT)

        result = aero.compute_forces(
            collective_rad=col_rad,
            tilt_lon=tilt_lon, tilt_lat=tilt_lat,
            R_hub=hub_state["R"], v_hub_world=hub_state["vel"],
            omega_rotor=omega_spin, wind_world=WIND,
            t=T_AERO_OFFSET + t_sim,
        )

        tf, tm      = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        tension_now = tether._last_info.get("tension", 0.0)
        F_net       = result.F_world + tf
        M_orbital   = result.M_orbital + tm

        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)
        hub_state  = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)

        if speed_now > 0.0:
            cycle_energy_out[cycle_idx] += tension_now * speed_now * DT
        elif speed_now < 0.0:
            cycle_energy_in[cycle_idx]  += tension_now * abs(speed_now) * DT

        dir_str = "reel-out" if speed_now > 0.0 else ("reel-in" if speed_now < 0.0 else "hold")
        phase_label = f"cycle{cycle_idx + 1}_{dir_str}"
        if tether._last_info.get("slack", False):
            events.record("slack", t_sim, phase_label, altitude, tension=tension_now)
        if tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase_label, altitude,
                          tension=tension_now)
        if hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t_sim, phase_label, altitude)

        if i % tel_every == 0:
            telemetry.append(make_tel(
                t_sim, hub_state, omega_spin, tether, tension_now,
                col_rad, tilt_lon, tilt_lat, WIND,
                body_z_eq=None, phase=phase_label,
            ))

    net_per_cycle = [cycle_energy_out[k] - cycle_energy_in[k]
                     for k in range(N_PUMP_CYCLES)]
    total_net = sum(net_per_cycle)

    lua_msgs  = sim.messages
    lua_bz_ri = sum(1 for _, m in lua_msgs if "bz_ri" in m)

    cycle_summary = "  ".join(
        f"c{k+1}={net_per_cycle[k]:.0f}J" for k in range(N_PUMP_CYCLES)
    )
    parts = [
        f"total_net={total_net:.0f}J",
        cycle_summary,
        f"t_end={t_sim:.1f}s",
        f"lua_bz_ri={lua_bz_ri}",
        events.summary(),
    ]
    for level, msg in lua_msgs[:15]:
        parts.append(f"  [GCS {level}] {msg}")

    if telemetry:
        write_csv([TelRow.from_tel(d) for d in telemetry],
                  _log.log_dir / "telemetry.csv")
    _log.write(["  ".join(p for p in parts if p)], "lua_pumping")

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
        lua_bz_ri        = lua_bz_ri,
        messages         = lua_msgs,
        telemetry        = telemetry,
    )


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_lua_pumping():
    """rawes.lua mode=5: no bad events, bz_ri emitted, every cycle positive, total net > 0."""
    r = _run_pumping()
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    if r["lua_bz_ri"] < 1:
        failures.append(
            f"lua_bz_ri={r['lua_bz_ri']} -- GPS may not have fused or pumping mode not entered"
        )
    for k, net in enumerate(r["net_per_cycle"]):
        if net <= 0:
            failures.append(f"cycle {k+1} net={net:.1f}J <= 0")
    if r["total_net"] <= 0:
        failures.append(f"total_net={r['total_net']:.1f}J <= 0")
    assert not failures, "\n  ".join(failures)
