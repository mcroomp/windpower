"""
compute_map.py -- Compute and visualize the RAWES flight envelope map.

Each cell (wind, tether_angle, tension) is evaluated with a collective PI:
one simulation per (cell, v_target) lets the controller settle the rotor to
the target along-tether speed and records the equilibrium collective.

Usage
-----
  python simulation/envelope/compute_map.py --quick        # fast preview
  python simulation/envelope/compute_map.py --full         # fine grid, save map
  python simulation/envelope/compute_map.py --quick --wind 10
  python simulation/envelope/compute_map.py --load map.npz  # reload saved grid
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from envelope.point_mass import simulate_point


# ── Tension-band sweep worker ─────────────────────────────────────────────────


def _ramp_full_column(args: tuple) -> list:
    """
    Ramp one column from T_min to T_max, sampling at each grid tension.

    args: (vi, wi, ai, v_target, wind, angle, v_tension_list, dt, tension_list_unified)

    Returns list of (ti_unified, col_rad, v_along, col_saturated) for each sampled point.
    ti_unified is the index into tension_list_unified (the merged list of all tensions).
    """
    vi, wi, ai, v_target, wind, angle, v_tension_list, dt, tension_list_unified = args
    import rotor_definition as rd
    from aero import create_aero
    from frames import build_orb_frame
    from envelope.point_mass import tether_hat, balance_bz

    rotor   = rd.default()
    aero    = create_aero(rotor, model="peters_he")
    dk      = rotor.dynamics_kwargs()
    mass    = dk["mass"]
    I_spin  = dk["I_spin"]
    weight  = mass * 9.81
    clamp   = math.radians(8.6)
    wind_v  = np.array([0.0, -float(wind), 0.0])
    t_hat   = tether_hat(angle)

    # Fresh start: omega=28 (operational), vel=0, col=0
    omega    = 28.0
    vel      = np.zeros(3)
    col_now  = 0.0
    c_lon    = c_lat = 0.0
    int_vx   = int_vy = int_vcol = 0.0

    # Ramp parameters
    t_start_tension = float(v_tension_list[0])
    t_end_tension   = float(v_tension_list[-1])
    ramp_rate       = 5.0   # N/s
    t_settle        = 10.0  # settle at start before ramping
    col_min         = -0.25
    col_max         = 0.20
    kp_col          = 0.02
    ki_col          = 0.005
    kp_cyc          = 0.5
    ki_cyc          = 0.1

    total_s     = t_settle + abs(t_end_tension - t_start_tension) / ramp_rate + 5.0
    total_steps = int(total_s / dt)

    results = []
    t_ramp_start = None
    tension = t_start_tension
    next_sample_idx = 0

    # Build a mapping from v_tension_list values to indices in tension_list_unified
    v_tension_to_unified_idx = {}
    for v_t in v_tension_list:
        for u_idx, u_t in enumerate(tension_list_unified):
            if abs(u_t - v_t) < 0.01:
                v_tension_to_unified_idx[float(v_t)] = u_idx
                break

    for i in range(total_steps):
        t_sim = i * dt

        # Settle phase, then ramp
        if t_sim < t_settle:
            tension = t_start_tension
        else:
            if t_ramp_start is None:
                t_ramp_start = t_sim
            elapsed_ramp = t_sim - t_ramp_start
            tension = t_start_tension + ramp_rate * elapsed_ramp
            tension = min(tension, t_end_tension)

        bz     = balance_bz(angle, tension, mass)
        R      = build_orb_frame(bz)
        F_teth = tension * t_hat

        try:
            f = aero.compute_forces(col_now, c_lon, c_lat, R, vel.copy(),
                                    omega, wind_v, t=45.0)
        except (OverflowError, ValueError, FloatingPointError):
            break
        if not aero.is_valid():
            break

        thrust = float(np.dot(f.F_world, bz))
        Q_net  = float(np.dot(aero.last_M_spin, bz))
        F_net  = f.F_world + F_teth + np.array([0.0, 0.0, weight])

        omega = max(0.0, omega + (Q_net / I_spin) * dt)
        vel   = vel + (F_net / mass) * dt

        if not np.all(np.isfinite(vel)):
            break

        v_perp  = vel - float(np.dot(vel, t_hat)) * t_hat
        t_norm  = max(abs(thrust), 1.0)
        c_lon_r = -(kp_cyc / t_norm * v_perp[0] + ki_cyc / t_norm * int_vx)
        c_lat_r = -(kp_cyc / t_norm * v_perp[1] + ki_cyc / t_norm * int_vy)
        c_lon   = float(np.clip(c_lon_r, -clamp, clamp))
        c_lat   = float(np.clip(c_lat_r, -clamp, clamp))
        if abs(c_lon) < clamp:
            int_vx += v_perp[0] * dt
        if abs(c_lat) < clamp:
            int_vy += v_perp[1] * dt

        v_along_now = float(np.dot(vel, t_hat))
        err = v_along_now - v_target
        int_vcol += err * dt
        col_now = float(np.clip(col_now + kp_col * err + ki_col * int_vcol,
                                col_min, col_max))

        # Sample at grid tensions for this v_target
        if next_sample_idx < len(v_tension_list) and t_sim >= t_settle:
            t_grid = float(v_tension_list[next_sample_idx])
            # Check if we've crossed this grid tension
            crossed = tension >= t_grid - 0.5 * ramp_rate * dt
            if crossed:
                v_along = float(np.dot(vel, t_hat))
                col_rad = col_now
                sat     = (col_now <= col_min * 0.99 or col_now >= col_max * 0.99)
                # Only record if converged (v_along within tolerance)
                if math.isfinite(v_along) and abs(v_along - v_target) < 0.15:
                    ti_unified = v_tension_to_unified_idx.get(float(t_grid), -1)
                    if ti_unified >= 0:
                        results.append((ti_unified, col_rad, v_along, sat))
                next_sample_idx += 1

        if next_sample_idx >= len(v_tension_list):
            break

    return results



# ── Grid definitions ───────────────────────────────────────────────────────────

QUICK = dict(
    wind_list    = [10.0],
    angle_list   = list(range(20, 91, 5)),
    tension_list = {
        -0.8: list(range(25, 2025, 50)),     # REEL-OUT: 25–2000 N (by 50)
        0.8:  list(range(25, 1025, 25)),     # REEL-IN:  25–1000 N (by 25)
    },
    t_end        = 35.0,
    dt           = 0.02,
)

FULL = dict(
    wind_list    = [4, 6, 8, 10, 12, 14],
    angle_list   = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90],
    tension_list = {
        -0.8: [50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 750, 1000, 1250, 1500, 1750, 2000],  # REEL-OUT: up to 2000 N
        0.8:  [50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 750, 1000],                          # REEL-IN:  up to 1000 N
    },
    t_end        = 40.0,
    dt           = 0.02,
)

V_REEL_OUT = -0.8
V_REEL_IN  =  0.8


# ── PID-based sweep ────────────────────────────────────────────────────────────

def _run_pass(tasks: list, tag: str, n_workers: int,
              settled_col: np.ndarray, settled_v: np.ndarray,
              col_sat: np.ndarray, settled_ic: np.ndarray) -> int:
    """Submit tasks, collect results into arrays. Returns number of newly converged cells."""
    newly = 0
    total = len(tasks)
    done  = 0
    t0    = time.time()
    with ProcessPoolExecutor(max_workers=n_workers) as pool:
        futures = {pool.submit(_pid_cell, t[1:]): t[:4] for t in tasks}
        for fut in as_completed(futures):
            vi, wi, ai, ti = futures[fut][:4]
            _, _, _, col, va, sat, ic_out = fut.result()
            if np.isfinite(col):
                settled_col[vi, wi, ai, ti] = col
                settled_v  [vi, wi, ai, ti] = va
                col_sat    [vi, wi, ai, ti] = sat
                settled_ic [vi, wi, ai, ti] = ic_out
                newly += 1
            done += 1
            elapsed = time.time() - t0
            eta = elapsed / done * (total - done) if done > 0 else 0
            print(f"  [{tag}] {done:>4}/{total}  eta={eta:.0f}s    ", end="\r")
    print()
    return newly


def compute_grid(
    wind_list:    list[float],
    angle_list:   list[float],
    tension_list: list[float] | dict | None = None,
    v_targets:    list[float] | None = None,
    t_end:        float = 35.0,
    dt:           float = 0.02,
    n_workers:    int   = 8,
) -> dict:
    """
    Tension-band sweep: for each (v_target, wind, angle) column, run one continuous
    ramp from T_min to T_max, sampling at each grid tension point.

    Parallelizes across columns (independent ramps).

    tension_list can be:
      - list: single range for all v_targets
      - dict: maps v_target (as float) to tension list, e.g. {-0.8: [25, ..., 500], 0.8: [25, ..., 2000]}

    Returns a dict with arrays shaped (n_targets, nw, na, nt):
      settled_col, settled_v, col_sat, failed
    """
    if v_targets is None:
        v_targets = [V_REEL_OUT, V_REEL_IN]

    if tension_list is None:
        tension_list = list(range(25, 1025, 25))

    # Handle per-v_target tension ranges
    v_tension_lists = {}
    if isinstance(tension_list, dict):
        v_tension_lists = {float(k): list(map(float, v)) for k, v in tension_list.items()}
        tension_list_unified = sorted(set(t for tlist in v_tension_lists.values() for t in tlist))
    else:
        tension_list = list(map(float, tension_list))
        tension_list_unified = tension_list
        for v in v_targets:
            v_tension_lists[float(v)] = tension_list

    nw = len(wind_list)
    na = len(angle_list)
    nt = len(tension_list_unified)
    nv = len(v_targets)

    settled_col = np.full((nv, nw, na, nt), np.nan)
    settled_v   = np.full((nv, nw, na, nt), np.nan)
    col_sat     = np.zeros((nv, nw, na, nt), dtype=bool)

    # Build tasks: one ramp per (v_target, wind, angle) column
    tasks = [
        (vi, wi, ai,
         float(v_targets[vi]), float(wind_list[wi]), float(angle_list[ai]),
         v_tension_lists[float(v_targets[vi])], dt, tension_list_unified)
        for vi in range(nv)
        for wi in range(nw)
        for ai in range(na)
    ]

    total = len(tasks)
    done  = 0
    t0    = time.time()

    print(f"  Tension-band sweep: {total} columns (parallel ramps)")
    with ProcessPoolExecutor(max_workers=n_workers) as pool:
        futures = {pool.submit(_ramp_full_column, t): (t[0], t[1], t[2]) for t in tasks}
        for fut in as_completed(futures):
            vi, wi, ai = futures[fut]
            ramp_results = fut.result()

            # ramp_results: list of (ti_in_unified, col_rad, v_along, col_saturated)
            for ti_unified, col, va, sat in ramp_results:
                settled_col[vi, wi, ai, ti_unified] = col
                settled_v  [vi, wi, ai, ti_unified] = va
                col_sat    [vi, wi, ai, ti_unified] = sat

            done += 1
            elapsed = time.time() - t0
            eta = elapsed / done * (total - done) if done > 0 else 0
            print(f"  [{done:>4}/{total}] +{len(ramp_results)} points  "
                  f"eta={eta:.0f}s    ", end="\r")
    print()

    failed = ~np.isfinite(settled_col)

    # Store which tensions are valid for each v_target
    v_target_tensions = {}
    for vi, v in enumerate(v_targets):
        v_target_tensions[float(v)] = v_tension_lists[float(v)]

    return dict(
        wind_list    = np.array(wind_list),
        angle_list   = np.array(angle_list),
        tension_list = np.array(tension_list_unified),
        v_targets    = np.array(v_targets),
        v_target_tensions = v_target_tensions,  # Maps v_target → tension list for that mode
        t_end        = float(t_end),
        dt           = float(dt),
        settled_col  = settled_col,
        settled_v    = settled_v,
        col_sat      = col_sat,
        failed       = failed,
    )


# ── CLI ────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    p = argparse.ArgumentParser(description="RAWES flight envelope map (PID sweep)")
    p.add_argument("--quick",   action="store_true", help="coarse grid for preview")
    p.add_argument("--full",    action="store_true", help="fine grid, save map")
    p.add_argument("--wind",    type=float, default=None,
                   help="override wind speed (single value)")
    p.add_argument("--load",    type=str,   default=None,
                   help="load grid from .npz file (skips computation)")
    p.add_argument("--save",    type=str,   default=None,
                   help="save grid to .npz file")
    p.add_argument("-n", "--workers", type=int, default=8,
                   help="number of parallel workers (default 8)")
    args = p.parse_args()

    if args.load:
        print(f"\nLoading grid from {args.load} ...")
        raw  = np.load(args.load, allow_pickle=True)
        grid = {k: raw[k].item() if k == 'v_target_tensions' and isinstance(raw[k], np.ndarray) else raw[k]
                for k in raw.files}
        for key in ("t_end", "dt"):
            if key in grid:
                grid[key] = float(grid[key])
        print(f"  wind={grid['wind_list']}  "
              f"angles={grid['angle_list'][0]:.0f}..{grid['angle_list'][-1]:.0f}  "
              f"tensions={grid['tension_list'][0]:.0f}..{grid['tension_list'][-1]:.0f}  "
              f"v_targets={grid['v_targets']}")
    else:
        cfg = FULL if args.full else QUICK
        if args.wind is not None:
            cfg = dict(cfg, wind_list=[args.wind])

        v_targets    = [V_REEL_OUT, V_REEL_IN]
        wind_list    = list(map(float, cfg["wind_list"]))    # type: ignore[arg-type]
        angle_list   = list(map(float, cfg["angle_list"]))   # type: ignore[arg-type]

        # Handle both dict and list tension_list
        tension_list_cfg = cfg["tension_list"]
        if isinstance(tension_list_cfg, dict):
            tension_list = {float(k): list(map(float, v)) for k, v in tension_list_cfg.items()}  # type: ignore[union-attr]
            tension_list_min = min(min(v) for v in tension_list.values())
            tension_list_max = max(max(v) for v in tension_list.values())
        else:
            tension_list = list(map(float, tension_list_cfg))
            tension_list_min = tension_list[0]
            tension_list_max = tension_list[-1]

        t_end        = float(cfg["t_end"])                   # type: ignore[arg-type]
        dt           = float(cfg["dt"])                      # type: ignore[arg-type]

        print(f"\nComputing envelope map  ({'full' if args.full else 'quick'}, PID sweep)")
        print(f"  wind={wind_list}  "
              f"angles={angle_list[0]:.0f}..{angle_list[-1]:.0f}  "
              f"tensions={tension_list_min:.0f}..{tension_list_max:.0f}")

        # Count total simulations
        if isinstance(tension_list, dict):
            total_tension_pts = len(set(t for tlist in tension_list.values() for t in tlist))
        else:
            total_tension_pts = len(tension_list)
        total_sims = len(wind_list) * len(angle_list) * total_tension_pts * len(v_targets)
        print(f"  Simulations: ~{total_sims}  "
              f"(vs {total_sims * 41} for 41-collective sweep)")
        print()

        grid = compute_grid(
            wind_list    = wind_list,
            angle_list   = angle_list,
            tension_list = tension_list,
            v_targets    = v_targets,
            t_end        = t_end,
            dt           = dt,
            n_workers    = args.workers,
        )

        if args.save:
            path = Path(args.save)
            path.parent.mkdir(parents=True, exist_ok=True)
            grid_arrays = {k: np.array(v) if not isinstance(v, np.ndarray) else v
                          for k, v in grid.items()}
            np.savez(path, **grid_arrays, allow_pickle=True)
            print(f"  Saved to {path}")

            # Also render PNGs for each wind speed
            try:
                import matplotlib.pyplot as plt
                log_dir = path.parent / "plots"
                log_dir.mkdir(parents=True, exist_ok=True)
                for wi, wind in enumerate(grid["wind_list"]):
                    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
                    for v_idx, (v_target, label) in enumerate(zip(grid["v_targets"],
                                                                    ["REEL-OUT", "REEL-IN"])):
                        ax = axes[v_idx]
                        all_tensions = np.array(grid["tension_list"], dtype=float)
                        angles = np.array(grid["angle_list"], dtype=float)

                        # Determine the valid tension range for this v_target
                        if "v_target_tensions" in grid:
                            valid_tensions = np.array(
                                grid["v_target_tensions"].get(float(v_target), all_tensions),
                                dtype=float)
                        else:
                            valid_tensions = all_tensions

                        # Filter to only the rows that belong to this mode
                        t_mask = np.array(
                            [np.any(np.abs(valid_tensions - t) < 0.1) for t in all_tensions])
                        plot_tensions = all_tensions[t_mask]

                        col_rad = grid["settled_col"][v_idx, wi, :, :][:, t_mask].copy()
                        col_deg = np.degrees(col_rad)
                        has_data = np.isfinite(col_rad)

                        ax.pcolormesh(angles, plot_tensions,
                                    np.ones((len(angles), len(plot_tensions))).T,
                                    cmap="Greys", vmin=0, vmax=2,
                                    shading="nearest", alpha=0.4)

                        im = ax.pcolormesh(angles, plot_tensions, col_deg.T,
                                         cmap="RdBu_r", vmin=-15, vmax=15,
                                         shading="nearest")

                        for ai, angle in enumerate(angles):
                            for ti, tension in enumerate(plot_tensions):
                                if has_data[ai, ti]:
                                    ax.text(angle, tension, f"{col_deg[ai, ti]:.1f}",
                                          ha="center", va="center",
                                          fontsize=7, color="black", fontweight="bold")

                        ax.set_xlabel("Tether angle [deg]")
                        if v_idx == 0:
                            ax.set_ylabel("Tension [N]")
                        ax.set_title(f"{label} (v={v_target:+.1f} m/s)", fontsize=11)
                        plt.colorbar(im, ax=ax, label="Collective [deg]")

                    fig.suptitle(f"Wind = {wind:.0f} m/s", fontsize=13, fontweight="bold")
                    plt.tight_layout()

                    png_path = log_dir / f"envelope_wind_{wind:.0f}ms.png"
                    fig.savefig(png_path, dpi=100, bbox_inches="tight")
                    plt.close(fig)
                    print(f"  Rendered {png_path}")
            except ImportError:
                pass
