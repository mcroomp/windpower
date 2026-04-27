"""
analyse_descent.py -- Steady autorotation equilibrium for vertical powered descent.

Simulates the hub as a point mass descending vertically with a horizontal disk.
Cyclic is controlled by a horizontal-velocity PID to maintain zero horizontal
drift.  Collective and tether tension are fixed inputs.  The simulation runs
until omega and descent rate converge (or timeout).

Physics
-------
- Disk always horizontal: body_z = [0, 0, -1] (NED)
- Tether tension T_tether [N] acts purely downward (el_deg=90) -- extra gravity.
- Spin ODE:  d(omega)/dt = Q_aero / I_spin
- Vertical:  d(vel_z)/dt = (thrust - weight - T_tether) / mass
- Horizontal: d(vel_xy)/dt = F_horiz/mass,  cyclic PID drives F_horiz -> 0
- Peters-He inflow carried forward each step.

Thrust sign
-----------
Upward thrust = dot(F_world, [0,0,-1]) = -F_world[2].
NEVER negate disk_normal: dot(F_world, -disk_normal) gives DOWNWARD component.
Verified by test_hover_sign.py.

Usage
-----
  python simulation/analysis/analyse_descent.py
  python simulation/analysis/analyse_descent.py --wind 0 --col 0.02 --tension 176
  python simulation/analysis/analyse_descent.py --wind 5 --col 0.02 --tension 200
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rotor_definition as rd
from aero   import create_aero
from frames import build_orb_frame


_BZ_VERT = np.array([0.0, 0.0, -1.0])
_G       = 9.81


def simulate_descent(
    col: float,
    T_tether: float,
    wind_speed: float = 0.0,
    omega_init: float = 5.0,
    vel_z_init: float = 0.5,
    dt: float = 0.01,
    t_end: float = 30.0,
    kp_cyc: float = 0.5,
    ki_cyc: float = 0.1,
) -> dict:
    """
    Simulate descent at fixed collective and tether tension.

    A horizontal-velocity PID drives cyclic to keep vel_xy near zero.
    Returns dict with time histories and final equilibrium values.
    """
    rotor = rd.default()
    aero  = create_aero(rotor, model="peters_he")
    dk    = rotor.dynamics_kwargs()
    mass  = dk["mass"]
    I_spin = dk["I_spin"]
    weight = mass * _G

    R    = build_orb_frame(_BZ_VERT)
    wind = np.array([0.0, float(wind_speed), 0.0])

    # State
    omega  = float(omega_init)
    vel    = np.array([0.0, 0.0, float(vel_z_init)])
    c_lon  = 0.0
    c_lat  = 0.0
    int_vx = 0.0
    int_vy = 0.0

    history = []
    n_steps = int(t_end / dt)

    for i in range(n_steps):
        t = i * dt

        # Aero forces with current state
        hub_vel = vel.copy()
        f = aero.compute_forces(col, c_lon, c_lat, R, hub_vel, omega, wind, t=45.0)

        thrust  = float(np.dot(f.F_world, _BZ_VERT))   # upward = -F_world[2]
        Q_net   = float(np.dot(aero.last_M_spin, _BZ_VERT))
        F_horiz = f.F_world[:2].copy()                  # NED North, East

        # Spin ODE
        d_omega = Q_net / I_spin
        omega   = max(0.0, omega + d_omega * dt)

        # Vertical dynamics: positive vel_z = downward (NED)
        F_vert_net = thrust - weight - T_tether
        vel[2] += (-F_vert_net / mass) * dt   # NED: downward accel if F_vert_net < 0

        # Horizontal dynamics
        vel[:2] += (F_horiz / mass) * dt

        # Cyclic PID: drive horizontal velocity to zero
        int_vx += vel[0] * dt
        int_vy += vel[1] * dt
        c_lon = -(kp_cyc * vel[0] + ki_cyc * int_vx)
        c_lat = -(kp_cyc * vel[1] + ki_cyc * int_vy)
        # Clamp to physically reasonable range
        c_lon = float(np.clip(c_lon, -0.15, 0.15))
        c_lat = float(np.clip(c_lat, -0.15, 0.15))

        if i % int(1.0 / dt) == 0:
            history.append(dict(
                t=t, omega=omega, vel_z=vel[2], vel_horiz=float(np.linalg.norm(vel[:2])),
                thrust=thrust, Q_net=Q_net, F_vert_net=F_vert_net,
                c_lon=math.degrees(c_lon), c_lat=math.degrees(c_lat),
            ))

    # Final 5 s average as equilibrium estimate
    final = [h for h in history if h["t"] >= t_end - 5.0]
    if final:
        eq = {k: float(np.mean([h[k] for h in final]))
              for k in ("omega", "vel_z", "thrust", "Q_net", "F_vert_net", "c_lon", "c_lat")}
    else:
        eq = history[-1] if history else {}

    return dict(history=history, eq=eq, converged=len(final) > 0)


def analyse(
    wind_speed: float = 0.0,
    col: float = 0.02,
    T_tether: float = 176.0,
    omega_init: float = 5.0,
) -> None:
    print(f"\nDescent simulation -- wind={wind_speed} m/s  col={math.degrees(col):.1f} deg  "
          f"T_tether={T_tether:.0f} N")

    result = simulate_descent(col, T_tether, wind_speed=wind_speed, omega_init=omega_init)
    history = result["history"]
    eq      = result["eq"]

    print(f"\n  {'t':>5}  {'omega':>7}  {'vel_z':>7}  {'thrust':>8}  "
          f"{'Q_net':>8}  {'F_vert':>8}  {'c_lon_deg':>9}  {'c_lat_deg':>9}")
    print("  " + "-" * 75)
    for h in history:
        print(f"  {h['t']:>5.1f}  {h['omega']:>7.2f}  {h['vel_z']:>7.3f}  "
              f"{h['thrust']:>8.1f}  {h['Q_net']:>8.3f}  {h['F_vert_net']:>8.1f}  "
              f"{h['c_lon']:>9.2f}  {h['c_lat']:>9.2f}")

    rotor = rd.default()
    mass  = rotor.dynamics_kwargs()["mass"]
    weight = mass * _G
    print(f"\n  Equilibrium (last 5 s avg):")
    print(f"    omega    = {eq.get('omega',0):.2f} rad/s")
    print(f"    vel_z    = {eq.get('vel_z',0):.3f} m/s (NED, positive=down)")
    print(f"    thrust   = {eq.get('thrust',0):.1f} N  (weight={weight:.1f} N + tether={T_tether:.0f} N = {weight+T_tether:.1f} N)")
    print(f"    F_vert   = {eq.get('F_vert_net',0):.2f} N  (0 = perfect balance)")
    print(f"    c_lon    = {eq.get('c_lon',0):.2f} deg")
    print(f"    c_lat    = {eq.get('c_lat',0):.2f} deg")


def simulate_descent_bem_ode(
    col: float,
    T_tether: float,
    wind_speed: float = 0.0,
    omega_init: float = 5.0,
    vel_z_init: float = 0.5,
    dt: float         = 0.01,
    t_end: float      = 30.0,
    kp_cyc: float     = 0.5,
    ki_cyc: float     = 0.1,
) -> dict:
    """
    Same point-mass ODE as simulate_descent() but uses q_spin_from_aero +
    step_spin_ode explicitly, with I_ode_kgm2 (not I_spin) — matching PhysicsCore.
    """
    from physics_core import q_spin_from_aero, step_spin_ode

    rotor  = rd.default()
    dk     = rotor.dynamics_kwargs()
    mass   = dk["mass"]
    weight = mass * _G
    I_ode  = rotor.I_ode_kgm2
    omega_min = rotor.omega_min_rad_s

    aero = create_aero(rotor, model="peters_he")
    R    = build_orb_frame(_BZ_VERT)
    wind = np.array([0.0, float(wind_speed), 0.0])

    omega  = float(omega_init)
    vel    = np.array([0.0, 0.0, float(vel_z_init)])
    c_lon  = 0.0
    c_lat  = 0.0
    int_vx = 0.0
    int_vy = 0.0

    history = []
    n_steps = int(t_end / dt)

    for i in range(n_steps):
        t = i * dt

        f = aero.compute_forces(col, c_lon, c_lat, R, vel.copy(), omega, wind, t=45.0)

        thrust     = float(np.dot(f.F_world, _BZ_VERT))
        Q_spin     = q_spin_from_aero(aero, R)
        F_horiz    = f.F_world[:2].copy()
        F_vert_net = thrust - weight - T_tether

        omega  = step_spin_ode(omega, Q_spin, I_ode, omega_min, dt)
        vel[2] += (-F_vert_net / mass) * dt
        vel[:2] += (F_horiz / mass) * dt

        int_vx += vel[0] * dt
        int_vy += vel[1] * dt
        c_lon = float(np.clip(-(kp_cyc * vel[0] + ki_cyc * int_vx), -0.15, 0.15))
        c_lat = float(np.clip(-(kp_cyc * vel[1] + ki_cyc * int_vy), -0.15, 0.15))

        if i % int(1.0 / dt) == 0:
            history.append(dict(
                t=t, omega=omega, vel_z=vel[2],
                thrust=thrust, Q_spin=Q_spin, F_vert_net=F_vert_net,
                c_lon=math.degrees(c_lon),
            ))

    final = [h for h in history if h["t"] >= t_end - 5.0]
    eq = ({k: float(np.mean([h[k] for h in final]))
           for k in ("omega", "vel_z", "thrust", "F_vert_net", "c_lon")}
          if final else (history[-1] if history else {}))
    return dict(history=history, eq=eq)


def simulate_descent_physics(
    col: float,
    T_tether: float,
    wind_speed: float = 0.0,
    omega_init: float = 5.0,
    vel_z_init: float = 0.5,
    altitude: float   = 500.0,
    dt: float         = 1.0 / 400.0,
    t_end: float      = 30.0,
    kp_cyc: float     = 0.5,
    ki_cyc: float     = 0.1,
) -> dict:
    """
    Same scenario as simulate_descent() but using PhysicsCore directly + Peters-He.

    PhysicsRunner is intentionally avoided — it bakes in AcroControllerSitl (rate PID
    + servo lag) which would process c_lon/c_lat as rate setpoints rather than tilt
    angles, making comparison with the other two functions meaningless.

    PhysicsCore is used with lock_orientation=True so the disk stays horizontal
    (body_z tracks tether direction = [0,0,-1] for vertical descent from altitude).
    Constant tether force T_tether [N downward in NED] replaces the elastic model.
    """
    from types import SimpleNamespace
    from physics_core import PhysicsCore, q_spin_from_aero

    rotor   = rd.default()
    bz_vert = np.array([0.0, 0.0, -1.0])
    R0      = build_orb_frame(bz_vert)
    wind    = np.array([0.0, float(wind_speed), 0.0])

    ic = SimpleNamespace(
        pos         = np.array([0.0, 0.0, -float(altitude)]),
        vel         = np.array([0.0, 0.0, float(vel_z_init)]),
        R0          = R0,
        rest_length = float(altitude),
        coll_eq_rad = float(col),
        omega_spin  = float(omega_init),
    )

    core = PhysicsCore(rotor, ic, wind,
                       aero_model="peters_he",
                       z_floor=1.0)

    # Replace tether with constant downward force (NED +Z = downward).
    _F_tether = np.array([0.0, 0.0, float(T_tether)])
    _M_zero   = np.zeros(3)
    core._tether.compute = lambda pos, vel, R=None: (_F_tether, _M_zero)
    core._tension_now    = float(T_tether)

    c_lon  = 0.0
    c_lat  = 0.0
    int_vx = 0.0
    int_vy = 0.0

    history   = []
    log_every = max(1, int(1.0 / dt))

    n_steps = int(t_end / dt)
    for i in range(n_steps):
        t = i * dt

        hub   = core.hub_state
        vel   = hub["vel"]
        omega = core.omega_spin

        # Cyclic PID on horizontal velocity (same logic as other two functions)
        int_vx += vel[0] * dt
        int_vy += vel[1] * dt
        c_lon = float(np.clip(-(kp_cyc * vel[0] + ki_cyc * int_vx), -0.15, 0.15))
        c_lat = float(np.clip(-(kp_cyc * vel[1] + ki_cyc * int_vy), -0.15, 0.15))

        sr = core.step(dt, col, c_lon, c_lat)
        # tension_now is overwritten from _last_info after each step; restore it
        core._tension_now = float(T_tether)
        # Keep disk horizontal: reset R and zero orbital angular velocity after each step.
        # This matches what simulate_descent and simulate_descent_bem_ode do (fixed R=R0).
        core._dyn._R[:]     = R0
        core._dyn._omega[:] = 0.0

        if i % log_every == 0:
            aero_res  = sr.get("aero_result")
            thrust_up = float(np.dot(aero_res.F_world, bz_vert)) if aero_res is not None else float(np.nan)
            history.append(dict(
                t=t, omega=omega, vel_z=float(vel[2]),
                vel_horiz=float(np.linalg.norm(vel[:2])),
                tension=float(T_tether),
                thrust=thrust_up,
                c_lon=math.degrees(c_lon), c_lat=math.degrees(c_lat),
            ))

    final = [h for h in history if h["t"] >= t_end - 5.0]
    if final:
        eq = {k: float(np.mean([h[k] for h in final]))
              for k in ("omega", "vel_z", "tension", "thrust", "c_lon", "c_lat")}
    else:
        eq = history[-1] if history else {}

    return dict(history=history, eq=eq)


def compare(
    col: float        = 0.02,
    T_tether: float   = 176.0,
    wind_speed: float = 0.0,
    omega_init: float = 5.0,
) -> None:
    """Run both models side-by-side and compare equilibrium."""
    rotor  = rd.default()
    mass   = rotor.dynamics_kwargs()["mass"]
    weight = mass * _G

    print(f"\nComparison -- wind={wind_speed} m/s  col={math.degrees(col):.1f} deg  "
          f"T_tether={T_tether:.0f} N  (weight={weight:.1f} N, total={weight+T_tether:.1f} N)")

    r_simple  = simulate_descent(col, T_tether, wind_speed=wind_speed, omega_init=omega_init)
    r_bem     = simulate_descent_bem_ode(col, T_tether, wind_speed=wind_speed, omega_init=omega_init)
    r_physics = simulate_descent_physics(col, T_tether, wind_speed=wind_speed, omega_init=omega_init)

    eq_s = r_simple["eq"]
    eq_b = r_bem["eq"]
    eq_p = r_physics["eq"]

    print(f"\n  {'':20}  {'simple(I_spin)':>14}  {'bem_ode(I_ode)':>14}  {'PhysicsRunner':>14}")
    print(f"  {'':20}  {'--------------':>14}  {'--------------':>14}  {'-------------':>14}")
    print(f"  {'omega [rad/s]':20}  {eq_s.get('omega',0):>14.2f}  {eq_b.get('omega',0):>14.2f}  {eq_p.get('omega',0):>14.2f}")
    print(f"  {'vel_z [m/s]':20}  {eq_s.get('vel_z',0):>14.3f}  {eq_b.get('vel_z',0):>14.3f}  {eq_p.get('vel_z',0):>14.3f}")
    print(f"  {'thrust [N]':20}  {eq_s.get('thrust',0):>14.1f}  {eq_b.get('thrust',0):>14.1f}  {eq_p.get('thrust',0):>14.1f}")
    print(f"  {'c_lon [deg]':20}  {eq_s.get('c_lon',0):>14.2f}  {eq_b.get('c_lon',0):>14.2f}  {eq_p.get('c_lon',0):>14.2f}")

    print(f"\n  Time history (1 Hz, all three):")
    print(f"  {'t':>5}  {'om_s':>6}  {'vz_s':>6}  |  {'om_b':>6}  {'vz_b':>6}  |  {'om_p':>6}  {'vz_p':>6}  {'ten_p':>7}")
    print("  " + "-" * 70)
    hs = r_simple["history"]
    hb = r_bem["history"]
    hp = r_physics["history"]
    for i in range(min(len(hs), len(hb), len(hp))):
        print(f"  {hs[i]['t']:>5.1f}"
              f"  {hs[i]['omega']:>6.2f}  {hs[i]['vel_z']:>6.3f}"
              f"  |  {hb[i]['omega']:>6.2f}  {hb[i]['vel_z']:>6.3f}"
              f"  |  {hp[i]['omega']:>6.2f}  {hp[i]['vel_z']:>6.3f}"
              f"  {hp[i]['tension']:>7.1f}")


def sweep(
    wind_speed: float = 0.0,
    v_desc_targets: "list[float]" = (0.5, 1.0, 1.5),
    tension_list: "list[float]" = (0.0, 50.0, 100.0, 150.0, 200.0),
    col_list: "list[float] | None" = None,
) -> None:
    """Sweep (col, T_tether) and find best operating points."""
    if col_list is None:
        col_list = list(np.linspace(-0.10, 0.20, 7))

    rotor  = rd.default()
    mass   = rotor.dynamics_kwargs()["mass"]
    weight = mass * _G

    print(f"\nSweep -- wind={wind_speed} m/s  weight={weight:.1f} N")
    print(f"  {'col_deg':>8}  {'T_tether':>9}  {'omega_eq':>8}  {'vel_z_eq':>9}  "
          f"{'thrust_eq':>10}  {'F_vert':>8}  {'c_lon':>7}  converged?")
    print("  " + "-" * 85)

    for col in col_list:
        for T_tether in tension_list:
            r  = simulate_descent(col, T_tether, wind_speed=wind_speed)
            eq = r["eq"]
            ok = "YES" if r["converged"] and abs(eq.get("F_vert_net", 999)) < 10.0 else "---"
            print(f"  {math.degrees(col):>8.1f}  {T_tether:>9.0f}  "
                  f"{eq.get('omega',0):>8.2f}  {eq.get('vel_z',0):>9.3f}  "
                  f"{eq.get('thrust',0):>10.1f}  {eq.get('F_vert_net',0):>8.1f}  "
                  f"{eq.get('c_lon',0):>7.2f}  {ok}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Descent equilibrium simulation")
    parser.add_argument("--wind",    type=float, default=0.0,
                        help="crosswind speed [m/s] (default 0)")
    parser.add_argument("--col",     type=float, default=None,
                        help="fixed collective [rad] — if given, run single simulation")
    parser.add_argument("--tension", type=float, default=None,
                        help="tether tension [N] — if given with --col, run single sim")
    parser.add_argument("--sweep",   action="store_true",
                        help="sweep (col, tension) grid and print table")
    parser.add_argument("--compare", action="store_true",
                        help="run simple ODE and PhysicsRunner side-by-side")
    parser.add_argument("--omega0",  type=float, default=5.0,
                        help="initial omega [rad/s] (default 5)")
    args = parser.parse_args()

    if args.compare:
        col     = args.col     if args.col     is not None else 0.02
        tension = args.tension if args.tension is not None else 176.0
        compare(col=col, T_tether=tension, wind_speed=args.wind, omega_init=args.omega0)
    elif args.col is not None and args.tension is not None:
        analyse(wind_speed=args.wind, col=args.col,
                T_tether=args.tension, omega_init=args.omega0)
    elif args.sweep:
        sweep(wind_speed=args.wind)
    else:
        # Default: show zero-wind equilibrium for the known good operating point
        analyse(wind_speed=args.wind, col=0.02, T_tether=176.0, omega_init=5.0)
