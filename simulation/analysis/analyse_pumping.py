"""
analyse_pumping.py -- Steady-state collective sweep for pumping cycle reel-out.

Sweeps collective values.  For each, runs a simple point-mass ODE with:
  - Constant tether force vector (idealized, no elasticity)
  - 3-axis velocity PI -> cyclic (lon/lat) to null horizontal drift
  - Fixed disk orientation (body_z set from force-balance geometry at start)
  - Peters-He aero (valid at all skew angles)

Goal: find which collective produces zero net acceleration (vel stays near 0).

Convention
----------
  Tether force : East  (+Y in NED) + downward (+Z) component from elevation
  Wind         : West  (-Y in NED)
  NED          : X=North, Y=East, Z=Down;  gravity = [0, 0, +m*g]

Usage
-----
  .venv/Scripts/python.exe simulation/analysis/analyse_pumping.py
  .venv/Scripts/python.exe simulation/analysis/analyse_pumping.py --wind 8
  .venv/Scripts/python.exe simulation/analysis/analyse_pumping.py --el 30 --tension 400 --wind 10
  .venv/Scripts/python.exe simulation/analysis/analyse_pumping.py --col -0.21  # single run, verbose
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aero import rotor_definition as rd
from frames      import build_orb_frame
from physics_core import PhysicsCore


_G = 9.81


# ── Geometry helpers ───────────────────────────────────────────────────────────

def _tether_force_ned(elevation_deg: float, tension_n: float) -> np.ndarray:
    """
    Constant tether force on hub in NED.

    Hub is west of and above anchor.
    Tether pulls hub eastward and downward: [0, +cos(el), +sin(el)] * T.
    """
    el = math.radians(elevation_deg)
    return tension_n * np.array([0.0, math.cos(el), math.sin(el)])


def _balance_bz(F_teth: np.ndarray, mass: float) -> np.ndarray:
    """
    Disk normal (body_z) that balances tether + gravity.

    The thrust must be equal and opposite to (tether + gravity) at equilibrium.
    Thrust direction = body_z, so body_z = -normalize(F_teth + F_grav).
    """
    F_grav = np.array([0.0, 0.0, mass * _G])
    F_load = F_teth + F_grav
    return -F_load / np.linalg.norm(F_load)


# ── Single simulation run (PhysicsCore, evolving R) ───────────────────────────

def simulate_one(
    col:           float,
    elevation_deg: float = 30.0,
    tension_n:     float = 400.0,
    wind_speed:    float = 10.0,
    v_reel_in:     float = 0.0,
    omega_init:    float | None = None,
    dt:            float = 1.0 / 400.0,
    t_end:         float = 40.0,
    verbose:       bool  = False,
) -> dict:
    """
    Run PhysicsCore at fixed collective with evolving disk orientation.

    v_reel_in > 0: hub starts moving along the tether toward the anchor
    (models reel-in descent).  Cyclic nulls velocity TRANSVERSE to the
    tether axis only — along-tether velocity evolves freely, driven by
    the force balance.  Its equilibrium value is the steady reel-in speed.

    v_reel_in = 0: pure hover equilibrium sweep (reel-out).
    """
    import json
    from types import SimpleNamespace

    rotor = rd.default()
    mass  = rotor.dynamics_kwargs()["mass"]

    _d = json.loads(
        (Path(__file__).resolve().parents[1] / "steady_state_starting.json").read_text())
    if omega_init is None:
        omega_init = float(_d["omega_spin"])
    col_warm = float(_d["coll_eq_rad"])

    el     = math.radians(elevation_deg)
    L      = 100.0
    # Hub west of anchor, elevated: pos = [0, -L*cos(el), -L*sin(el)]
    pos0   = np.array([0.0, -L * math.cos(el), -L * math.sin(el)])

    # Tether unit vector hub -> anchor (East + Down in NED)
    t_hat  = np.array([0.0, math.cos(el), math.sin(el)])

    F_teth = _tether_force_ned(elevation_deg, tension_n)   # = tension_n * t_hat
    bz0    = _balance_bz(F_teth, mass)
    R0     = build_orb_frame(bz0)
    wind   = np.array([0.0, -wind_speed, 0.0])  # West in NED

    # Initial velocity: along tether toward anchor (reel-in descent)
    vel0 = float(v_reel_in) * t_hat

    ic = SimpleNamespace(
        pos         = pos0,
        vel         = vel0.copy(),
        R0          = R0,
        rest_length = L,
        coll_eq_rad = col_warm,
        omega_spin  = omega_init,
    )

    # lock_orientation=True pins the disk to R0 so it can't flip.
    # We still get correct aero forces from the hub's translational velocity
    # (descent inflow drives autorotation via the spin ODE inside PhysicsCore).
    core = PhysicsCore(rotor, ic, wind, aero_model="peters_he",
                       z_floor=-500.0, lock_orientation=True)

    M_zero = np.zeros(3)
    core._tether.compute = lambda _p, _v, _R=None: (F_teth, M_zero)  # type: ignore[assignment]
    core._tension_now    = float(tension_n)

    omega_floor = rotor.omega_min_rad_s

    history   = []
    log_every = max(1, int(1.0 / dt))

    n_steps = int(t_end / dt)
    for i in range(n_steps):
        t = i * dt

        hub  = core.hub_state
        vel  = hub["vel"]
        bz_now = hub["R"][:, 2]

        # Clamp spin before step to keep aero model valid
        if core._omega_spin < omega_floor:
            core._omega_spin = omega_floor

        # lock_orientation=True: disk stays fixed; no cyclic needed
        sr = core.step(dt, col, 0.0, 0.0)
        core._tension_now = float(tension_n)

        if i % log_every == 0:
            ar      = sr.get("aero_result")
            thrust  = float(np.dot(ar.F_world, bz_now)) if ar is not None else float("nan")
            v_along = float(np.dot(vel, t_hat))
            v_trans = float(np.linalg.norm(vel - v_along * t_hat))
            history.append(dict(
                t       = round(t, 3),
                omega   = round(core.omega_spin, 3),
                v_along = round(v_along, 4),
                v_trans = round(v_trans, 4),
                vel_z   = round(float(vel[2]), 4),
                thrust  = round(thrust, 2),
            ))

    final = [h for h in history if h["t"] >= t_end - 8.0]
    eq = ({k: round(float(np.mean([h[k] for h in final])), 5)
           for k in ("omega", "v_along", "v_trans", "vel_z", "thrust")}
          if final else (history[-1] if history else {}))

    if verbose:
        print(f"\n  col={math.degrees(col):.2f} deg  v_reel_in_init={v_reel_in:.2f} m/s")
        print(f"  {'t':>5}  {'omega':>7}  {'v_along':>8}  {'v_trans':>8}  {'thrust':>8}")
        print("  " + "-" * 55)
        for h in history:
            print(f"  {h['t']:>5.1f}  {h['omega']:>7.2f}  {h['v_along']:>8.3f}  "
                  f"{h['v_trans']:>8.3f}  {h['thrust']:>8.1f}")

    return dict(history=history, eq=eq)


# ── Sweep ──────────────────────────────────────────────────────────────────────

def sweep(
    elevation_deg: float = 30.0,
    tension_n:     float = 400.0,
    wind_speed:    float = 10.0,
    v_reel_in:     float = 0.0,
    col_list:      list[float] | None = None,
    t_end:         float = 40.0,
) -> None:
    rotor  = rd.default()
    mass   = rotor.dynamics_kwargs()["mass"]
    weight = mass * _G

    F_teth = _tether_force_ned(elevation_deg, tension_n)
    bz0    = _balance_bz(F_teth, mass)
    F_load = F_teth + np.array([0.0, 0.0, weight])

    print()
    print(f"Reel-out equilibrium sweep")
    print(f"  el={elevation_deg} deg  T={tension_n:.0f} N  wind={wind_speed} m/s West")
    print(f"  Tether force NED : [{F_teth[0]:+.1f}, {F_teth[1]:+.1f}, {F_teth[2]:+.1f}] N")
    print(f"  Total load NED   : [{F_load[0]:+.1f}, {F_load[1]:+.1f}, {F_load[2]:+.1f}] N")
    print(f"  Balance body_z   : [{bz0[0]:+.4f}, {bz0[1]:+.4f}, {bz0[2]:+.4f}]")
    print(f"  Disk tilt from vertical: "
          f"{math.degrees(math.acos(max(-1.0, min(1.0, float(-bz0[2]))))):.1f} deg")
    print()

    if col_list is None:
        col_list = list(np.linspace(-0.28, 0.02, 16))

    mode = "reel-in" if v_reel_in > 0 else "hover"
    print(f"  mode={mode}  v_reel_in_init={v_reel_in:.2f} m/s")
    print()
    print(f"  {'col_deg':>8}  {'omega':>7}  {'v_along':>8}  {'v_trans':>8}  {'thrust':>8}  note")
    print("  " + "-" * 65)

    best_col    = None
    best_metric = 1e9

    for col in col_list:
        r  = simulate_one(col, elevation_deg, tension_n, wind_speed,
                          v_reel_in=v_reel_in, t_end=t_end)
        eq = r["eq"]
        # For reel-in: look for stable v_along (not exploding) and small v_trans
        # For hover: look for small v_along
        v_along = eq.get("v_along", float("nan"))
        v_trans = eq.get("v_trans", float("nan"))
        metric  = abs(v_trans) + (abs(v_along - v_reel_in) if v_reel_in > 0 else abs(v_along))

        note = ""
        if metric < best_metric and math.isfinite(metric):
            best_metric = metric
            best_col    = col
        if v_trans < 0.5 and abs(v_along - v_reel_in) < 0.5:
            note = "<-- stable"
        elif v_along > 5:
            note = "(accelerating in)"
        elif v_along < -2:
            note = "(climbing out)"

        print(f"  {math.degrees(col):>8.2f}  "
              f"{eq.get('omega', 0):>7.2f}  "
              f"{v_along:>8.3f}  "
              f"{v_trans:>8.3f}  "
              f"{eq.get('thrust', 0):>8.1f}  "
              f"{note}")

    if best_col is not None:
        print()
        print(f"  Best collective: {math.degrees(best_col):.2f} deg ({best_col:.4f} rad)")


# ── CLI ────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Reel-out collective sweep")
    p.add_argument("--el",      type=float, default=30.0,  help="elevation angle [deg]")
    p.add_argument("--tension", type=float, default=400.0, help="tether tension [N]")
    p.add_argument("--wind",    type=float, default=10.0,  help="wind speed [m/s]")
    p.add_argument("--t-end",    type=float, default=40.0, help="sim time [s]")
    p.add_argument("--v-reel-in",type=float, default=0.0,
                   help="reel-in speed along tether [m/s] (0=hover/reel-out)")
    p.add_argument("--col",      type=float, default=None,
                   help="single collective [rad] — verbose time history")
    args = p.parse_args()

    if args.col is not None:
        r = simulate_one(args.col, args.el, args.tension, args.wind,
                         v_reel_in=args.v_reel_in, t_end=args.t_end, verbose=True)
        eq = r["eq"]
        print(f"\n  Equilibrium (last 8 s avg):")
        for k, v in eq.items():
            print(f"    {k:<14} = {v}")
    else:
        sweep(args.el, args.tension, args.wind,
              v_reel_in=args.v_reel_in, t_end=args.t_end)
