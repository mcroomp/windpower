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
from dynamics        import RigidBodyDynamics
from aero            import create_aero
from tether          import TetherModel
from controller      import (
    OrbitTracker,
    col_min_for_altitude_rad,
    AcroController,
)
from planner         import DeschutterPlanner, WindEstimator, quat_apply, quat_is_identity
from winch           import WinchController
from simtest_log     import SimtestLog, BadEventLog
from simtest_ic      import load_ic
from tel             import make_tel
from telemetry_csv   import TelRow, write_csv

_log   = SimtestLog(__file__)
_IC    = load_ic()
_ROTOR = rd.default()
_AERO  = create_aero(_ROTOR)

# ── Simulation constants ───────────────────────────────────────────────────────
DT            = 1.0 / 400.0
ANCHOR        = np.zeros(3)
T_AERO_OFFSET = 45.0

I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5
WIND           = np.array([0.0, 10.0, 0.0])   # NED: East wind
BREAK_LOAD_N   = 620.0

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

TENSION_OUT    = 200.0   # N
TENSION_IN     =  55.0   # N

COL_MIN_RAD         = -0.28
COL_MAX_RAD         =  0.10
COL_MIN_REEL_IN_RAD = col_min_for_altitude_rad(_AERO, XI_REEL_IN_DEG, _ROTOR.mass_kg)
TENSION_SAFETY_N    = 496.0   # N (~80% break load)

FLOOR_ALT_M  = 1.0
T_CYCLE      = T_REEL_OUT + T_REEL_IN


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def _run_pumping_repeated() -> dict:
    dyn    = RigidBodyDynamics(
        mass=_ROTOR.mass_kg, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=_IC.pos.tolist(), vel0=_IC.vel.tolist(),
        R0=_IC.R0, omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(rd.default())
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=_IC.rest_length,
                         axle_attachment_length=_ROTOR.axle_attachment_length_m)
    winch  = WinchController(rest_length=_IC.rest_length,
                              tension_safety_n=TENSION_SAFETY_N,
                              min_length=2.0)

    trajectory = DeschutterPlanner(
        t_reel_out          = T_REEL_OUT,
        t_reel_in           = T_REEL_IN,
        t_transition        = T_TRANSITION,
        v_reel_out          = V_REEL_OUT,
        v_reel_in           = V_REEL_IN,
        tension_out         = TENSION_OUT,
        tension_in          = TENSION_IN,
        wind_estimator      = WindEstimator(seed_wind_ned=WIND),
        col_min_rad              = COL_MIN_RAD,
        col_max_rad              = COL_MAX_RAD,
        xi_reel_in_deg           = XI_REEL_IN_DEG,
        col_min_reel_in_rad      = COL_MIN_REEL_IN_RAD,
    )

    orbit_tracker = OrbitTracker(_IC.R0[:, 2], _IC.pos / np.linalg.norm(_IC.pos), BODY_Z_SLEW_RATE)
    acro          = AcroController.from_rotor(rd.default(), use_servo=True)

    hub_state   = dyn.state
    omega_spin  = _IC.omega_spin
    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now    = tether._last_info.get("tension", 0.0)
    collective_rad = _IC.coll_eq_rad
    body_z_eq      = orbit_tracker.bz_slerp

    t_end_sim = N_CYCLES * T_CYCLE
    max_steps = int(t_end_sim / DT) + 1

    # Per-cycle energy tracking
    cycle_energy_out = [0.0] * N_CYCLES
    cycle_energy_in  = [0.0] * N_CYCLES
    events = BadEventLog()

    telemetry = []
    tel_every = max(1, int(0.05 / DT))

    t_sim = 0.0
    for i in range(max_steps):
        t_sim  = i * DT
        if t_sim >= t_end_sim:
            break

        altitude = -hub_state["pos"][2]
        cycle_idx = min(int(t_sim / T_CYCLE), N_CYCLES - 1)

        # ── DeschutterPlanner drives all N cycles ─────────────────────────────
        state_pkt = {
            "pos_ned":         hub_state["pos"],
            "vel_ned":         hub_state["vel"],
            "omega_spin":      omega_spin,
            "body_z":          hub_state["R"][:, 2],
            "tension_n":       tension_now,
            "tether_length_m": winch.tether_length_m,
        }
        pump_cmd = trajectory.step(state_pkt, DT)

        winch.step(pump_cmd["winch_speed_ms"], tension_now, DT)
        tether.rest_length = winch.rest_length

        collective_rad = acro.slew_collective(
            COL_MIN_RAD + pump_cmd["thrust"] * (COL_MAX_RAD - COL_MIN_RAD), DT
        )

        _aq = pump_cmd["attitude_q"]
        _bz_target = (None if quat_is_identity(_aq)
                      else quat_apply(_aq, np.array([0.0, 0.0, -1.0])))
        body_z_eq = orbit_tracker.update(hub_state["pos"], DT, _bz_target)

        pump_phase = pump_cmd["phase"]   # "reel-out" | "reel-in" | "hold"
        if pump_phase == "reel-out":
            cycle_energy_out[cycle_idx] += tension_now * V_REEL_OUT * DT
        elif pump_phase == "reel-in":
            cycle_energy_in[cycle_idx]  += tension_now * V_REEL_IN  * DT

        # ── Attitude controller + servo ───────────────────────────────────────
        tilt_lon, tilt_lat = acro.update(hub_state, body_z_eq, DT)
        result = aero.compute_forces(
            collective_rad = collective_rad,
            tilt_lon       = tilt_lon,
            tilt_lat       = tilt_lat,
            R_hub          = hub_state["R"],
            v_hub_world    = hub_state["vel"],
            omega_rotor    = omega_spin,
            wind_world     = WIND,
            t              = T_AERO_OFFSET + t_sim,
        )

        # ── Tether ────────────────────────────────────────────────────────────
        tf, tm      = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        tension_now = tether._last_info.get("tension", 0.0)
        F_net       = result.F_world + tf
        M_orbital   = result.M_orbital + tm

        # ── Spin & dynamics ───────────────────────────────────────────────────
        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)
        hub_state  = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)

        # ── Bad-event tracking ────────────────────────────────────────────────
        tel_phase_label = f"cycle{cycle_idx+1}_{pump_phase}"
        if tether._last_info.get("slack", False):
            events.record("slack", t_sim, tel_phase_label, altitude,
                          tension=tension_now)
        if tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, tel_phase_label, altitude,
                          tension=tension_now)
        if hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t_sim, tel_phase_label, altitude)

        # ── Telemetry (20 Hz) ─────────────────────────────────────────────────
        if i % tel_every == 0:
            telemetry.append(make_tel(
                t_sim, hub_state, omega_spin, tether, tension_now,
                collective_rad, tilt_lon, tilt_lat, WIND,
                body_z_eq=body_z_eq, phase=tel_phase_label,
                aero_result=result, aero_obj=aero, tether_force=tf, tether_moment=tm,
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
