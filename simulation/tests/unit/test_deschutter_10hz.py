"""
test_deschutter_10hz.py -- De Schutter pumping cycle at 10 Hz RC-override rate.

In the stack (internal_controller=True), the DeschutterPlanner and WinchController
run at 400 Hz inside the mediator.  However, ArduPilot ACRO mode receives tilt
rate commands via MAVLink RC overrides at 10 Hz.  ArduPilot holds the last RC
override between updates, so from the hub's perspective the tilt setpoint
changes at most 10 times per second even though the inner rate loop runs faster.

This test reproduces the 10 Hz tilt-rate lag with 400 Hz inner physics.

Controller split (matches internal_controller=True stack path):
  OUTER (10 Hz) -- AcroController: computes tilt rate setpoint from body_z error
                   Rate setpoint held for the next 40 inner steps (ZOH)
  INNER (400 Hz) -- DeschutterPlanner + WinchController + OrbitTracker +
                    RatePID (held rate setpoint) + aero + dynamics

Pass criteria (same as test_deschutter_cycle.py):
  1. Hub stays above z_floor throughout.
  2. Reel-in mean tension < reel-out mean tension (De Schutter mechanism).
  3. Net energy positive.
  4. Peak tension stays below 80% of tether break load.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

import rotor_definition as rd
from dynamics    import RigidBodyDynamics
from aero        import create_aero
from tether      import TetherModel
from controller  import (OrbitTracker, col_min_for_altitude_rad,
                         AcroController, RatePID, compute_rate_cmd)
from planner     import DeschutterPlanner, WindEstimator, quat_apply, quat_is_identity
from winch       import WinchController
from frames      import build_orb_frame
from simtest_log import SimtestLog
from simtest_ic  import load_ic

_log   = SimtestLog(__file__)
_IC    = load_ic()
_ROTOR = rd.default()
_AERO  = create_aero(_ROTOR)

# ── Timing ─────────────────────────────────────────────────────────────────────
DT_INNER      = 1.0 / 400.0   # physics timestep [s]
DT_OUTER      = 1.0 / 10.0    # planner update rate [s]  -- matches MAVLink rate in stack
INNER_PER_OUTER = round(DT_OUTER / DT_INNER)   # 40 inner steps per outer step

# ── Simulation state ───────────────────────────────────────────────────────────
ANCHOR        = np.zeros(3)
POS0          = _IC.pos
VEL0          = _IC.vel
BODY_Z0       = _IC.body_z
OMEGA_SPIN0   = _IC.omega_spin
REST_LENGTH0  = _IC.rest_length

T_AERO_OFFSET = 45.0
I_SPIN_KGMS2  = 10.0
OMEGA_SPIN_MIN = 0.5

WIND          = np.array([0.0, 10.0, 0.0])   # NED: East wind = Y axis
BREAK_LOAD_N  = 620.0

# ── De Schutter cycle parameters ──────────────────────────────────────────────
T_REEL_OUT    = 30.0
T_REEL_IN     = 30.0
V_REEL_OUT    =  0.4
V_REEL_IN     =  0.4
XI_REEL_IN_DEG  = 80.0
BODY_Z_SLEW_RATE = _ROTOR.body_z_slew_rate_rad_s
T_TRANSITION    = math.radians(XI_REEL_IN_DEG - 30.0) / BODY_Z_SLEW_RATE + 1.5

TENSION_OUT   = 200.0
TENSION_IN    =  55.0
COL_MIN_RAD   = -0.28
COL_MAX_RAD   =  0.10
COL_MIN_REEL_IN_RAD = col_min_for_altitude_rad(_AERO, XI_REEL_IN_DEG, _ROTOR.mass_kg)
TENSION_SAFETY_N = 496.0


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def _run_10hz() -> dict:
    """
    Run one De Schutter pumping cycle (30 s reel-out + 30 s reel-in).

    In the stack (internal_controller=True), DeschutterPlanner and WinchController
    run at 400 Hz inside the mediator.  The bottleneck is the RC override rate:
    the attitude rate setpoint (from compute_rate_cmd) is sent to ArduPilot at
    10 Hz via MAVLink; ArduPilot holds the last rate command between updates.

    Rate computation (outer, 10 Hz):
      - compute_rate_cmd(bz_now, body_z_eq, R, kp) -> rate_sp_held
        (Lua equivalent; runs in mediator or Lua script, RC override at 10 Hz)

    Physics + inner control (inner, 400 Hz):
      - DeschutterPlanner.step()    -> collective, winch_speed, bz_target
      - WinchController.step()
      - OrbitTracker.update()       -> body_z_eq (slerp at 400 Hz)
      - RatePID.update(rate_sp_held, omega_body)  -> tilt  (held ZOH)
      - aero.compute_forces()
      - tether.compute()
      - dynamics.step()
    """
    dyn = RigidBodyDynamics(
        mass=_ROTOR.mass_kg, I_body=_ROTOR.I_body_kgm2, I_spin=0.0,
        pos0=POS0.tolist(), vel0=VEL0.tolist(),
        R0=build_orb_frame(BODY_Z0), omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(_ROTOR)
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=REST_LENGTH0,
                         axle_attachment_length=0.0)
    winch  = WinchController(rest_length=REST_LENGTH0,
                             tension_safety_n=TENSION_SAFETY_N)
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
    # Two-loop attitude controller split: outer computes rate setpoint, inner
    # applies it.  Only the outer (rate setpoint) runs at 10 Hz, mirroring the
    # RC override rate.  The inner RatePID and all other 400 Hz loops run every step.
    KP_OUTER   = AcroController.DEFAULT_KP_OUTER
    KP_INNER   = RatePID.DEFAULT_KP
    pid_lon    = RatePID(kp=KP_INNER)
    pid_lat    = RatePID(kp=KP_INNER)
    orbit_tracker = OrbitTracker(BODY_Z0, POS0 / np.linalg.norm(POS0), BODY_Z_SLEW_RATE)

    hub_state   = dyn.state
    omega_spin  = OMEGA_SPIN0
    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now = tether._last_info.get("tension", 0.0)

    from planner import Q_IDENTITY
    last_cmd = {
        "attitude_q":    Q_IDENTITY.copy(),
        "thrust":        1.0,
        "winch_speed_ms": 0.0,
        "phase":         "reel-out",
    }
    collective_rad = COL_MIN_RAD
    body_z_eq      = BODY_Z0.copy()

    # Held rate setpoint (zero-order hold between 10 Hz outer updates)
    rate_sp_held = np.zeros(3)

    t_total    = T_REEL_OUT + T_REEL_IN
    n_steps    = int(t_total / DT_INNER)
    tel_every  = max(1, int(0.05 / DT_INNER))   # 20 Hz telemetry

    tensions_out = []
    tensions_in  = []
    energy_out   = 0.0
    energy_in    = 0.0
    floor_hits   = 0
    telemetry    = []

    for i in range(n_steps):
        t = i * DT_INNER

        # ── Rate setpoint update (10 Hz) ─────────────────────────────────────
        # Mirrors MAVLink RC override rate: compute_rate_cmd runs on the ground
        # station (or Lua script) at 10 Hz; ArduPilot holds the last rate
        # setpoint between updates.
        if i % INNER_PER_OUTER == 0:
            bz_now   = hub_state["R"][:, 2]
            R_hub    = np.asarray(hub_state["R"], dtype=float)
            rate_sp_held = compute_rate_cmd(bz_now, body_z_eq, R_hub, kp=KP_OUTER)

        # ── Full 400 Hz inner loop ─────────────────────────────────────────────
        # DeschutterPlanner, WinchController, OrbitTracker, and physics all run
        # at 400 Hz — only the rate setpoint (above) is throttled to 10 Hz.

        # 1. DeschutterPlanner + WinchController (400 Hz, mirrors mediator)
        state_pkt = {
            "pos_ned":         hub_state["pos"],
            "vel_ned":         hub_state["vel"],
            "omega_spin":      omega_spin,
            "body_z":          hub_state["R"][:, 2],
            "tension_n":       tension_now,
            "tether_length_m": winch.tether_length_m,
        }
        last_cmd = trajectory.step(state_pkt, DT_INNER)
        winch.step(last_cmd["winch_speed_ms"], tension_now, DT_INNER)
        tether.rest_length = winch.rest_length
        collective_rad = float(last_cmd.get("collective_rad",
            COL_MIN_RAD + last_cmd["thrust"] * (COL_MAX_RAD - COL_MIN_RAD)))

        # 2. OrbitTracker slerp (400 Hz)
        _aq = last_cmd["attitude_q"]
        _bz_target = (None if quat_is_identity(_aq)
                      else quat_apply(_aq, np.array([0.0, 0.0, -1.0])))
        body_z_eq = orbit_tracker.update(hub_state["pos"], DT_INNER, _bz_target)

        # 3. Inner RatePID with held rate setpoint (400 Hz)
        omega_body = np.asarray(hub_state["R"], dtype=float).T @ np.asarray(hub_state["omega"], dtype=float)
        omega_body[2] = 0.0
        tilt_lon =  pid_lon.update(rate_sp_held[0], omega_body[0], DT_INNER)
        tilt_lat = -pid_lat.update(rate_sp_held[1], omega_body[1], DT_INNER)
        result = aero.compute_forces(
            collective_rad = collective_rad,
            tilt_lon       = tilt_lon,
            tilt_lat       = tilt_lat,
            R_hub          = hub_state["R"],
            v_hub_world    = hub_state["vel"],
            omega_rotor    = omega_spin,
            wind_world     = WIND,
            t              = T_AERO_OFFSET + t,
        )
        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        F_net     = result.F_world + tf
        M_orbital = result.M_orbital + tm
        tension_now = tether._last_info.get("tension", 0.0)

        phase = last_cmd.get("phase", "reel-out")
        if phase == "reel-out":
            energy_out += tension_now * V_REEL_OUT * DT_INNER
            tensions_out.append(tension_now)
        else:
            energy_in  += tension_now * V_REEL_IN  * DT_INNER
            tensions_in.append(tension_now)

        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT_INNER)
        M_orbital += -50.0 * hub_state["omega"]
        hub_state  = dyn.step(F_net, M_orbital, DT_INNER, omega_spin=omega_spin)

        if hub_state["pos"][2] >= -1.05:
            floor_hits += 1
            if floor_hits > 200:
                break

        if i % tel_every == 0:
            _ti = tether._last_info
            telemetry.append({
                "t_sim":          t,
                "phase":          phase,
                "altitude_m":     float(-hub_state["pos"][2]),
                "tether_tension": float(tension_now),
                "collective_rad": float(collective_rad),
                "omega_rotor":    float(omega_spin),
            })

    skip = int(T_TRANSITION / DT_OUTER)   # skip reel-in transition in mean
    n_skip_inner = skip * INNER_PER_OUTER
    mean_out = float(np.mean(tensions_out)) if tensions_out else 0.0
    mean_in  = float(np.mean(tensions_in[n_skip_inner:])) if tensions_in[n_skip_inner:] else 0.0
    peak     = max(tensions_out + tensions_in) if tensions_out or tensions_in else 0.0

    return dict(
        floor_hits  = floor_hits,
        energy_out  = energy_out,
        energy_in   = energy_in,
        net_energy  = energy_out - energy_in,
        mean_out    = mean_out,
        mean_in     = mean_in,
        peak        = peak,
        telemetry   = telemetry,
    )


import functools

@functools.lru_cache(maxsize=None)
def _results():
    return _run_10hz()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _print_results(r: dict) -> None:
    print(f"\n  Planner rate: {1.0/DT_OUTER:.0f} Hz (inner physics: {1.0/DT_INNER:.0f} Hz)")
    print(f"  Reel-out mean tension: {r['mean_out']:.1f} N")
    print(f"  Reel-in  mean tension: {r['mean_in']:.1f} N  (steady, skip={int(T_TRANSITION/DT_OUTER)} steps)")
    print(f"  Peak tension:          {r['peak']:.1f} N  (break load {BREAK_LOAD_N:.0f} N)")
    print(f"  Net energy:            {r['net_energy']:.0f} J")
    print(f"  Floor hits:            {r['floor_hits']}")


def test_10hz_no_crash():
    """Hub stays above z_floor at 10 Hz planner rate."""
    r = _results()
    _print_results(r)
    assert r["floor_hits"] == 0, \
        f"Hub hit z_floor {r['floor_hits']} times at 10 Hz planner rate"


def test_10hz_deschutter_mechanism():
    """Reel-in steady tension < reel-out tension at 10 Hz (De Schutter mechanism works)."""
    r = _results()
    assert r["mean_in"] < r["mean_out"], (
        f"De Schutter mechanism failed at 10 Hz: "
        f"reel-in {r['mean_in']:.1f} N >= reel-out {r['mean_out']:.1f} N"
    )


def test_10hz_net_energy_positive():
    """Net energy positive at 10 Hz (pumping cycle generates power)."""
    r = _results()
    assert r["net_energy"] > 0, \
        f"Net energy negative at 10 Hz: {r['net_energy']:.0f} J"


def test_10hz_peak_tension_safe():
    """Peak tension stays below 80% tether break load at 10 Hz."""
    r = _results()
    assert r["peak"] < TENSION_SAFETY_N, (
        f"Peak tension {r['peak']:.0f} N exceeds safety limit {TENSION_SAFETY_N:.0f} N "
        f"(80% break load) at 10 Hz planner rate"
    )
