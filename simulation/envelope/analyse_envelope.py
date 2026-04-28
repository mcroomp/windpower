"""
analyse_envelope.py -- Flight envelope analysis for the RAWES pumping cycle.

For each (elevation, tension, wind, collective) combination, finds the
steady-state along-tether velocity (v_along_eq).

Model assumptions
-----------------
- Tether direction is fixed (constant elevation) -- hub travels freely along it.
- Horizontal wind force is balanced by the tether geometry; we only care
  about the along-tether component of the force balance.
- "Stable" means v_along converges to a finite steady value.
- The desired operating point is a specific v_along target:
    reel_out : v_along < 0  (hub moving away from anchor)
    reel_in  : v_along > 0  (hub moving toward anchor)
    hover    : v_along = 0

Positive v_along = toward anchor (reel-in direction).
Negative v_along = away from anchor (reel-out direction).

Usage
-----
  python simulation/envelope/analyse_envelope.py --phase reel_out
  python simulation/envelope/analyse_envelope.py --phase reel_in
  python simulation/envelope/analyse_envelope.py --el 80 --tension 200 --wind 10
  python simulation/envelope/analyse_envelope.py --el 80 --tension 200 --wind 10 --col 0.08
  python simulation/envelope/analyse_envelope.py --envelope --wind 10
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from envelope.point_mass import simulate_point, balance_bz, tether_hat

_G = 9.81

# ── Pre-defined flight phases ──────────────────────────────────────────────────

PHASES: dict[str, dict] = {
    "reel_out": dict(elevation_deg=30.0, tension_n=400.0, wind_speed=10.0,
                     col_range=(-0.28, 0.05), v_target=-0.4,  note="power generation"),
    "reel_in":  dict(elevation_deg=80.0, tension_n=200.0, wind_speed=10.0,
                     col_range=(-0.20, 0.15), v_target=+0.8,  note="return stroke"),
    "descent":  dict(elevation_deg=90.0, tension_n=176.0, wind_speed=0.0,
                     col_range=(-0.15, 0.20), v_target=+1.6,  note="vertical landing"),
}


# ── Geometry summary ───────────────────────────────────────────────────────────

def _print_geometry(elevation_deg: float, tension_n: float, wind_speed: float) -> None:
    import rotor_definition as rd
    mass   = rd.default().dynamics_kwargs()["mass"]
    weight = mass * _G
    t_hat  = tether_hat(elevation_deg)
    F_teth = tension_n * t_hat
    F_grav = np.array([0.0, 0.0, weight])
    F_load = F_teth + F_grav
    bz     = balance_bz(elevation_deg, tension_n, mass)
    tilt   = math.degrees(math.acos(max(-1.0, min(1.0, float(-bz[2])))))
    t_horiz_pct = 100.0 * math.cos(math.radians(elevation_deg))

    print(f"  el={elevation_deg:.0f} deg  T={tension_n:.0f} N  "
          f"wind={wind_speed:.1f} m/s  weight={weight:.1f} N")
    print(f"  Tether horizontal component: {t_horiz_pct:.0f}%  "
          f"({tension_n * math.cos(math.radians(elevation_deg)):.1f} N East)")
    print(f"  Total load NED   : [{F_load[0]:+.1f}, {F_load[1]:+.1f}, {F_load[2]:+.1f}] N")
    print(f"  Balance body_z   : [{bz[0]:+.4f}, {bz[1]:+.4f}, {bz[2]:+.4f}]")
    print(f"  Disk tilt from vertical: {tilt:.1f} deg")
    print()


# ── Convergence check ─────────────────────────────────────────────────────────

def _is_converged(history: list[dict], key: str = "v_along", window: float = 10.0,
                  tol: float = 0.05) -> bool:
    """True if key varies by less than tol over the last window seconds."""
    final = [h[key] for h in history if h["t"] >= history[-1]["t"] - window]
    if len(final) < 3:
        return False
    return (max(final) - min(final)) < tol


# ── Collective sweep ───────────────────────────────────────────────────────────

def sweep_collective(
    elevation_deg: float = 90.0,
    tension_n:     float = 176.0,
    wind_speed:    float = 0.0,
    col_range:     tuple[float, float] = (-0.20, 0.20),
    n_col:         int   = 17,
    v_target:      float = 0.0,
    t_end:         float = 40.0,
    label:         str   = "",
) -> list[dict]:
    """
    Sweep collective at fixed (elevation, tension, wind).

    Stability criterion: v_along converges (variance < 0.05 m/s over last 10 s).
    Reports equilibrium v_along for each collective.
    Marks the collective closest to v_target.
    """
    col_list = list(np.linspace(col_range[0], col_range[1], n_col))

    print()
    if label:
        print(f"  [{label}]  v_target={v_target:+.2f} m/s along tether")
    _print_geometry(elevation_deg, tension_n, wind_speed)

    print(f"  {'col_deg':>8}  {'omega':>6}  {'v_along':>8}  {'thrust':>8}  "
          f"{'F_net_along':>11}  status")
    print("  " + "-" * 64)

    best_col  = None
    best_diff = 1e9
    results   = []

    for col in col_list:
        r  = simulate_point(col, elevation_deg, tension_n, wind_speed, t_end=t_end)
        eq = r["eq"]
        converged = _is_converged(r["history"])
        v_along   = eq.get("v_along", float("nan"))
        diff      = abs(v_along - v_target)

        if converged and math.isfinite(v_along) and diff < best_diff:
            best_diff = diff
            best_col  = col

        marker = " <-- target" if (converged and diff == best_diff) else ""
        status = "[OK]  " if converged else "[DIVE]"

        print(f"  {math.degrees(col):>8.2f}  "
              f"{eq.get('omega', 0):>6.2f}  "
              f"{v_along:>8.3f}  "
              f"{eq.get('thrust', 0):>8.1f}  "
              f"{eq.get('F_net_along', 0):>11.3f}  "
              f"{status}{marker}")

        results.append(dict(col_rad=col, col_deg=math.degrees(col),
                            **eq, converged=converged))

    if best_col is not None:
        print()
        print(f"  Closest to v_target={v_target:+.2f}: "
              f"col={math.degrees(best_col):.2f} deg  "
              f"v_along={results[[r['col_rad'] for r in results].index(best_col)]['v_along']:+.3f} m/s")
    else:
        print()
        print(f"  No converged solution found.")

    return results


# ── Full envelope: (elevation x tension) grid ─────────────────────────────────

def flight_envelope(
    elevation_list: list[float] | None = None,
    tension_list:   list[float] | None = None,
    wind_speed:     float = 10.0,
    v_target:       float = 0.0,
    n_col:          int   = 13,
    t_end:          float = 30.0,
) -> None:
    """
    Grid sweep over (elevation, tension) at fixed wind and v_target.

    For each cell, finds the best collective and reports the equilibrium
    v_along. Marks cells where a converged solution exists.
    """
    if elevation_list is None:
        elevation_list = [20.0, 30.0, 45.0, 60.0, 70.0, 80.0, 90.0]
    if tension_list is None:
        tension_list = [100.0, 200.0, 300.0, 400.0, 500.0, 600.0]

    col_list = list(np.linspace(-0.25, 0.20, n_col))

    print()
    print(f"Flight envelope  wind={wind_speed:.1f} m/s  v_target={v_target:+.2f} m/s")
    print(f"  (showing best v_along [m/s]; * = converged within 0.2 m/s of target)")
    print()

    t_header = "".join(f"  {f'T={int(t)}':>8}" for t in tension_list)
    print(f"  {'el':>5}  {t_header}")
    print(f"  {'deg':>5}  " + "  --------" * len(tension_list))

    for el in elevation_list:
        row = f"  {el:>5.0f}  "
        for T in tension_list:
            best_col    = None
            best_diff   = 1e9
            best_valng  = float("nan")

            for col in col_list:
                r  = simulate_point(col, el, T, wind_speed, t_end=t_end)
                eq = r["eq"]
                v_along = eq.get("v_along", float("nan"))
                if _is_converged(r["history"]) and math.isfinite(v_along):
                    diff = abs(v_along - v_target)
                    if diff < best_diff:
                        best_diff  = diff
                        best_col   = col
                        best_valng = v_along

            if best_col is not None:
                flag = "*" if best_diff < 0.2 else " "
                row += f"  {best_valng:>+6.2f}{flag} "
            else:
                row += f"  {'---':>8} "
        print(row)

    print()
    print(f"  * = best collective achieves v_along within 0.2 m/s of target ({v_target:+.2f} m/s)")
    print(f"  --- = no converged solution found.")


# ── Verbose single run ─────────────────────────────────────────────────────────

def single_verbose(
    col:           float,
    elevation_deg: float,
    tension_n:     float,
    wind_speed:    float,
    v_along_init:  float = 0.0,
    t_end:         float = 40.0,
) -> None:
    print()
    _print_geometry(elevation_deg, tension_n, wind_speed)
    r  = simulate_point(col, elevation_deg, tension_n, wind_speed,
                        v_along_init=v_along_init, t_end=t_end)
    eq = r["eq"]

    print(f"  col={math.degrees(col):.2f} deg  v_along_init={v_along_init:.2f} m/s")
    print(f"  {'t':>5}  {'omega':>6}  {'v_along':>8}  {'thrust':>8}  {'F_net_along':>11}")
    print("  " + "-" * 48)
    for h in r["history"]:
        print(f"  {h['t']:>5.1f}  {h['omega']:>6.2f}  {h['v_along']:>8.3f}  "
              f"{h['thrust']:>8.1f}  {h['F_net_along']:>11.3f}")

    converged = _is_converged(r["history"])
    print()
    print(f"  Equilibrium (last 10 s avg):  {'[CONVERGED]' if converged else '[DIVERGING]'}")
    for k, v in eq.items():
        print(f"    {k:<14} = {v}")


# ── CLI ────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    p = argparse.ArgumentParser(description="RAWES flight envelope analysis")
    p.add_argument("--phase",    choices=list(PHASES), default=None,
                   help="pre-defined flight phase")
    p.add_argument("--el",       type=float, default=None,
                   help="tether elevation [deg]")
    p.add_argument("--tension",  type=float, default=None,
                   help="tether tension [N]")
    p.add_argument("--wind",     type=float, default=10.0,
                   help="wind speed [m/s]")
    p.add_argument("--col",      type=float, default=None,
                   help="single collective [rad] -- verbose time history")
    p.add_argument("--v-along",  type=float, default=0.0,
                   help="initial along-tether velocity [m/s]")
    p.add_argument("--v-target", type=float, default=None,
                   help="target v_along for envelope best-match [m/s]")
    p.add_argument("--t-end",    type=float, default=40.0,
                   help="simulation duration [s]")
    p.add_argument("--envelope", action="store_true",
                   help="full (elevation x tension) grid at given wind")
    p.add_argument("--n-col",    type=int,   default=17,
                   help="number of collective points in sweep")
    args = p.parse_args()

    if args.envelope:
        v_target = args.v_target if args.v_target is not None else 0.0
        flight_envelope(wind_speed=args.wind, v_target=v_target)

    elif args.phase is not None:
        cfg = PHASES[args.phase]
        v_target = args.v_target if args.v_target is not None else cfg["v_target"]
        if args.col is not None:
            single_verbose(args.col, cfg["elevation_deg"], cfg["tension_n"],
                           cfg["wind_speed"], v_along_init=args.v_along, t_end=args.t_end)
        else:
            sweep_collective(
                elevation_deg = cfg["elevation_deg"],
                tension_n     = cfg["tension_n"],
                wind_speed    = cfg["wind_speed"],
                col_range     = cfg["col_range"],
                n_col         = args.n_col,
                v_target      = v_target,
                t_end         = args.t_end,
                label         = f"{args.phase} -- {cfg['note']}",
            )

    elif args.el is not None and args.tension is not None:
        v_target = args.v_target if args.v_target is not None else 0.0
        if args.col is not None:
            single_verbose(args.col, args.el, args.tension, args.wind,
                           v_along_init=args.v_along, t_end=args.t_end)
        else:
            sweep_collective(
                elevation_deg = args.el,
                tension_n     = args.tension,
                wind_speed    = args.wind,
                n_col         = args.n_col,
                v_target      = v_target,
                t_end         = args.t_end,
            )

    else:
        for name, cfg in PHASES.items():
            sweep_collective(
                elevation_deg = cfg["elevation_deg"],
                tension_n     = cfg["tension_n"],
                wind_speed    = cfg["wind_speed"],
                col_range     = cfg["col_range"],
                n_col         = 9,
                v_target      = cfg["v_target"],
                t_end         = 30.0,
                label         = f"{name} -- {cfg['note']}",
            )
