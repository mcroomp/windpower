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


# ── Legacy sweep worker (used by tests/bisection) ─────────────────────────────

def _sim_cell(args: tuple) -> tuple:
    """Simulate one cell across all fixed collectives (legacy sweep)."""
    wi, ai, ti, wind, angle, tension, col_list, t_end, dt = args
    row = [float("nan")] * len(col_list)
    for ci, col in enumerate(col_list):
        try:
            r  = simulate_point(col, angle, tension, wind, t_end=t_end, dt=dt)
            eq = r["eq"]
            h  = r["history"]
            va = eq.get("v_along", float("nan"))
            last = [x["v_along"] for x in h if x["t"] >= h[-1]["t"] - 10.0]
            conv = len(last) >= 3 and (max(last) - min(last)) < 0.05
            if conv and math.isfinite(va):
                row[ci] = va
        except (OverflowError, ValueError, FloatingPointError):
            pass
    return wi, ai, ti, row


def _bisect_cell(args: tuple) -> tuple:
    """Bisect collective to find v_target for one cell."""
    wi, ai, ti, wind, angle, tension, col_list, all_v_cell, v_target, t_end, dt, n_steps = args

    row   = all_v_cell
    valid = np.isfinite(row)
    if not np.any(valid):
        return wi, ai, ti, float("nan"), float("nan")

    col_lo = col_hi = float("nan")
    v_lo   = v_hi   = float("nan")
    valid_idx = [i for i in range(len(col_list)) if valid[i]]
    for k in range(len(valid_idx) - 1):
        i, j = valid_idx[k], valid_idx[k + 1]
        vi, vj = row[i], row[j]
        if (vi - v_target) * (vj - v_target) <= 0:
            col_lo, col_hi = col_list[i], col_list[j]
            v_lo,   v_hi   = vi, vj
            break

    if not math.isfinite(col_lo):
        diffs = np.where(valid, np.abs(row - v_target), np.inf)
        ci = int(np.argmin(diffs))
        return wi, ai, ti, float(col_list[ci]), float(row[ci])

    for _ in range(n_steps):
        col_mid = (col_lo + col_hi) / 2.0
        try:
            r  = simulate_point(col_mid, angle, tension, wind, t_end=t_end, dt=dt)
            h  = r["history"]
            va = r["eq"].get("v_along", float("nan"))
            last = [x["v_along"] for x in h if x["t"] >= h[-1]["t"] - 10.0]
            conv = len(last) >= 3 and (max(last) - min(last)) < 0.05
        except (OverflowError, ValueError, FloatingPointError):
            conv, va = False, float("nan")

        if not conv or not math.isfinite(va):
            break

        if (va - v_target) * (v_lo - v_target) <= 0:
            col_hi, v_hi = col_mid, va
        else:
            col_lo, v_lo = col_mid, va

    col_mid = (col_lo + col_hi) / 2.0
    try:
        r  = simulate_point(col_mid, angle, tension, wind, t_end=t_end, dt=dt)
        va = r["eq"].get("v_along", float("nan"))
    except (OverflowError, ValueError, FloatingPointError):
        va = float("nan")

    return wi, ai, ti, col_mid, va


# ── PID sweep workers ─────────────────────────────────────────────────────────

def _pid_cell(args: tuple) -> tuple:
    """
    Run one collective-PI simulation for a single (wind, angle, tension, v_target).

    args: (wi, ai, ti, wind, angle, tension, v_target, t_end, dt, ic)
    ic: dict | None — full IC to warm-start from, or None for cold start (col=0).

    Returns (wi, ai, ti, settled_col_rad, settled_v_along, col_saturated, ic_out).
    col/va = NaN, ic_out = None on non-convergence or error.
    """
    wi, ai, ti, wind, angle, tension, v_target, t_end, dt, ic = args
    col_init = ic["col"] if ic is not None else 0.0
    try:
        r   = simulate_point(
            col           = col_init,
            elevation_deg = angle,
            tension_n     = tension,
            wind_speed    = wind,
            v_target      = v_target,
            t_end         = t_end,
            dt            = dt,
            ic            = ic,
        )
        eq  = r["eq"]
        h   = r["history"]
        va  = eq.get("v_along",        float("nan"))
        col = eq.get("collective_rad", float("nan"))
        last = [x["v_along"] for x in h if x["t"] >= h[-1]["t"] - 10.0]
        conv = len(last) >= 3 and (max(last) - min(last)) < 0.1
        sat  = r["col_saturated"]
        if conv and math.isfinite(va) and math.isfinite(col):
            return wi, ai, ti, col, va, sat, r["ic"]
    except (OverflowError, ValueError, FloatingPointError):
        pass
    return wi, ai, ti, float("nan"), float("nan"), False, None


def _ramp_column(wind: float, angle: float, v_target: float,
                 tension_list: list, dt: float,
                 ti_start: int, ic_start: dict | None,
                 direction: int,
                 ramp_rate: float = 5.0,
                 t_settle: float  = 10.0,
                 col_min: float   = -0.25,
                 col_max: float   = 0.20,
                 kp_col: float    = 0.02,
                 ki_col: float    = 0.005,
                 kp_cyc: float    = 0.5,
                 ki_cyc: float    = 0.1,
                 cyc_clamp_deg: float = 8.6) -> list:
    """
    Continuously ramp tension through failed cells in one direction.

    Starts settled at tension_list[ti_start] using ic_start, settles for
    t_settle seconds, then ramps at ramp_rate N/s toward the end of the list
    (direction=+1) or toward index 0 (direction=-1).

    Samples the state at each grid tension crossing.  Returns list of
    (ti, col_rad, v_along, col_saturated, ic_dict) for each sampled grid point.
    """
    import rotor_definition as rd
    from aero import create_aero
    from frames import build_orb_frame
    from envelope.point_mass import tether_hat, balance_bz

    rotor   = rd.default()
    aero    = create_aero(rotor, model="peters_he",
                          state_dict=ic_start.get("aero_state") if ic_start else None)
    dk      = rotor.dynamics_kwargs()
    mass    = dk["mass"]
    I_spin  = dk["I_spin"]
    weight  = mass * 9.81
    clamp   = math.radians(cyc_clamp_deg)
    wind_v  = np.array([0.0, -float(wind), 0.0])
    t_hat   = tether_hat(angle)

    # restore IC (or use zero-velocity fresh start)
    if ic_start is not None:
        omega    = float(ic_start["omega"])
        vel      = np.array(ic_start["vel"], dtype=float)
        col_now  = float(ic_start["col"])
        c_lon    = float(ic_start["c_lon"])
        c_lat    = float(ic_start["c_lat"])
        int_vx   = float(ic_start["int_vx"])
        int_vy   = float(ic_start["int_vy"])
        int_vcol = float(ic_start["int_vcol"])
    else:
        omega    = 28.0
        vel      = np.zeros(3)
        col_now  = 0.0
        c_lon    = c_lat = 0.0
        int_vx   = int_vy = int_vcol = 0.0

    t_start_tension = float(tension_list[ti_start])
    t_end_tension   = float(tension_list[-1] if direction > 0 else tension_list[0])

    # grid tensions to sample (excluding start, which is already converged)
    if direction > 0:
        sample_tis = [ti for ti in range(ti_start + 1, len(tension_list))]
    else:
        sample_tis = [ti for ti in range(ti_start - 1, -1, -1)]
    next_sample_idx = 0

    results = []
    t_ramp_start = None
    tension = t_start_tension

    total_s     = t_settle + abs(t_end_tension - t_start_tension) / ramp_rate + 5.0
    total_steps = int(total_s / dt)

    for i in range(total_steps):
        t_sim = i * dt

        if t_sim < t_settle:
            tension = t_start_tension
        else:
            if t_ramp_start is None:
                t_ramp_start = t_sim
            elapsed_ramp = t_sim - t_ramp_start
            tension = t_start_tension + direction * ramp_rate * elapsed_ramp
            if direction > 0:
                tension = min(tension, t_end_tension)
            else:
                tension = max(tension, t_end_tension)

        bz     = balance_bz(angle, tension, mass)
        R      = build_orb_frame(bz)
        F_teth = tension * t_hat

        try:
            f = aero.compute_forces(col_now, c_lon, c_lat, R, vel.copy(),
                                    omega, wind_v, t=45.0)
        except (OverflowError, ValueError, FloatingPointError):
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

        # check if we've crossed the next grid tension
        if next_sample_idx >= len(sample_tis):
            break
        ti_next    = sample_tis[next_sample_idx]
        t_grid     = float(tension_list[ti_next])
        crossed    = (direction > 0 and tension >= t_grid - 0.5 * ramp_rate * dt) or \
                     (direction < 0 and tension <= t_grid + 0.5 * ramp_rate * dt)

        if crossed and t_sim >= t_settle:
            v_along = float(np.dot(vel, t_hat))
            col_rad = col_now
            sat     = (col_now <= col_min * 0.99 or col_now >= col_max * 0.99)
            ic_out  = dict(col=col_now, vel=vel.copy(), omega=omega,
                           c_lon=c_lon, c_lat=c_lat,
                           int_vx=int_vx, int_vy=int_vy, int_vcol=int_vcol,
                           aero_state=aero.to_dict())
            if math.isfinite(v_along) and abs(v_along - v_target) < 0.15:
                results.append((ti_next, col_rad, v_along, sat, ic_out))
            next_sample_idx += 1

    return results


# ── Grid definitions ───────────────────────────────────────────────────────────

QUICK = dict(
    wind_list    = [10.0],
    angle_list   = list(range(20, 91, 5)),
    tension_list = list(range(25, 1025, 25)),
    t_end        = 35.0,
    dt           = 0.02,
)

FULL = dict(
    wind_list    = [4, 6, 8, 10, 12, 14],
    angle_list   = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90],
    tension_list = [50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600],
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
    tension_list: list[float],
    v_targets:    list[float] | None = None,
    t_end:        float = 35.0,
    dt:           float = 0.02,
    n_workers:    int   = 8,
) -> dict:
    """
    Two-phase sweep:

    Pass 1 — all cells cold-start (ic=None, col=0), parallelized.
    Pass 2 — for each (vi, wi, ai) column, sweep low→high then high→low along
      the tension axis, warm-starting each failed cell from the settled IC of its
      nearest converged neighbor in that column. Columns are independent and run
      in parallel.

    Returns a dict with arrays shaped (n_targets, nw, na, nt):
      settled_col, settled_v, col_sat, failed
    """
    if v_targets is None:
        v_targets = [V_REEL_OUT, V_REEL_IN]

    nw = len(wind_list)
    na = len(angle_list)
    nt = len(tension_list)
    nv = len(v_targets)

    settled_col = np.full((nv, nw, na, nt), np.nan)
    settled_v   = np.full((nv, nw, na, nt), np.nan)
    col_sat     = np.zeros((nv, nw, na, nt), dtype=bool)
    settled_ic  = np.empty((nv, nw, na, nt), dtype=object)

    # ── Pass 1: cold start for every cell ────────────────────────────────────
    tasks = [
        (vi, wi, ai, ti,
         float(wind), float(angle), float(tension), float(v_target),
         t_end, dt, None)
        for vi, v_target in enumerate(v_targets)
        for wi, wind    in enumerate(wind_list)
        for ai, angle   in enumerate(angle_list)
        for ti, tension in enumerate(tension_list)
    ]
    print(f"  Pass 1: {len(tasks)} cells (cold start)")
    _run_pass(tasks, "pass1", n_workers, settled_col, settled_v, col_sat, settled_ic)

    # ── Pass 2: serial tension ramp per column ────────────────────────────────
    # For each column with gaps, ramp up from the lowest converged IC and
    # ramp down from the highest converged IC.  No multiprocessing here —
    # each ramp is a single continuous sim that must stay sequential.
    t_list = list(map(float, tension_list))
    newly_total = 0
    n_cols = nv * nw * na
    done   = 0
    t0     = time.time()
    print(f"  Pass 2: tension ramp  ({n_cols} columns, serial)")

    for vi in range(nv):
        for wi in range(nw):
            for ai in range(na):
                col_arr = settled_col[vi, wi, ai]
                ic_arr  = settled_ic [vi, wi, ai]

                conv_tis = [ti for ti in range(nt) if np.isfinite(col_arr[ti])
                            and ic_arr[ti] is not None]

                if conv_tis:
                    ti_low  = min(conv_tis)
                    ti_high = max(conv_tis)

                    for ti_start, direction in [(ti_low, +1), (ti_high, -1)]:
                        for ti, col, va, sat, ic_out in _ramp_column(
                                float(wind_list[wi]), float(angle_list[ai]),
                                float(v_targets[vi]), t_list, dt,
                                ti_start, ic_arr[ti_start], direction):
                            if not np.isfinite(settled_col[vi, wi, ai, ti]):
                                settled_col[vi, wi, ai, ti] = col
                                settled_v  [vi, wi, ai, ti] = va
                                col_sat    [vi, wi, ai, ti] = sat
                                settled_ic [vi, wi, ai, ti] = ic_out
                                newly_total += 1

                done += 1
                elapsed = time.time() - t0
                eta = elapsed / done * (n_cols - done) if done > 0 else 0
                print(f"  [pass2] {done:>4}/{n_cols}  "
                      f"+{newly_total} new  eta={eta:.0f}s    ", end="\r")
    print()
    print(f"  Pass 2: {newly_total} newly converged")

    failed = ~np.isfinite(settled_col)

    return dict(
        wind_list    = np.array(wind_list),
        angle_list   = np.array(angle_list),
        tension_list = np.array(tension_list),
        v_targets    = np.array(v_targets),
        t_end        = float(t_end),
        dt           = float(dt),
        settled_col  = settled_col,
        settled_v    = settled_v,
        col_sat      = col_sat,
        failed       = failed,
    )


# ── Visualisation ──────────────────────────────────────────────────────────────

def visualize(grid: dict, v_reel_out: float = V_REEL_OUT, v_reel_in: float = V_REEL_IN) -> None:
    import matplotlib.pyplot as plt

    angles   = np.asarray(grid["angle_list"],   dtype=float)
    tensions = np.asarray(grid["tension_list"], dtype=float)
    winds    = np.asarray(grid["wind_list"],    dtype=float)
    v_tgts   = np.asarray(grid["v_targets"],   dtype=float)
    nw = len(winds)
    na = len(angles)
    nt = len(tensions)

    settled_col = grid["settled_col"]   # (nv, nw, na, nt)
    settled_v   = grid["settled_v"]
    col_sat     = grid["col_sat"]
    failed      = grid.get("failed", ~np.isfinite(settled_col))

    tol = 0.2   # m/s — cell is "on target"

    fig, axes = plt.subplots(2, nw, figsize=(max(6, 4 * nw), 9), squeeze=False)
    fig.suptitle(
        f"RAWES Collective Map (PID-settled)\n"
        f"Green = on target [deg]  |  Grey = off-target/sat  |  "
        f"X = hard failure (no neighbors)  |  . = soft failure\n"
        f"reel-out={v_reel_out:+.1f} m/s    reel-in={v_reel_in:+.1f} m/s",
        fontsize=10, fontweight="bold")

    cmap_col = plt.colormaps["RdBu_r"]
    col_lim  = 15.0   # degrees

    def _vtgt_index(v: float) -> int:
        return int(np.argmin(np.abs(v_tgts - v)))

    for wi, wind in enumerate(winds):
        for row, (v_tgt, label) in enumerate([
            (v_reel_out, f"Reel-out  target={v_reel_out:+.1f} m/s"),
            (v_reel_in,  f"Reel-in   target={v_reel_in:+.1f} m/s"),
        ]):
            vi  = _vtgt_index(v_tgt)
            ax  = axes[row, wi]

            col_rad  = settled_col[vi, wi]   # (na, nt)
            v_eq     = settled_v  [vi, wi]
            sat      = col_sat    [vi, wi]
            fail     = failed     [vi, wi]

            col_deg  = np.degrees(col_rad)
            has_data = np.isfinite(v_eq)
            on_tgt   = has_data & ~sat & (np.abs(v_eq - v_tgt) <= tol)

            # hard failure: failed AND no converged neighbor along tension axis
            hard_fail = np.zeros((na, nt), dtype=bool)
            for ai in range(na):
                for ti in range(nt):
                    if fail[ai, ti]:
                        has_nbr = (
                            (ti > 0      and np.isfinite(col_rad[ai, ti - 1])) or
                            (ti < nt - 1 and np.isfinite(col_rad[ai, ti + 1]))
                        )
                        hard_fail[ai, ti] = not has_nbr

            col_plot = np.where(on_tgt, col_deg, np.nan)

            ax.pcolormesh(angles, tensions,
                          np.ones((len(angles), len(tensions))).T,
                          cmap="Greys", vmin=0, vmax=2,
                          shading="nearest", alpha=0.4)

            im = ax.pcolormesh(angles, tensions, col_plot.T,
                               cmap=cmap_col, vmin=-col_lim, vmax=col_lim,
                               shading="nearest")

            for ai, angle in enumerate(angles):
                for ti, tension in enumerate(tensions):
                    if on_tgt[ai, ti]:
                        ax.text(angle, tension, f"{col_deg[ai, ti]:.1f}",
                                ha="center", va="center",
                                fontsize=6, color="darkgreen", fontweight="bold")
                    elif has_data[ai, ti]:
                        color = "#884400" if sat[ai, ti] else "#444444"
                        ax.text(angle, tension, f"{v_eq[ai, ti]:+.2f}",
                                ha="center", va="center",
                                fontsize=6, color=color)
                    elif hard_fail[ai, ti]:
                        ax.text(angle, tension, "X",
                                ha="center", va="center",
                                fontsize=6, color="#cc0000", fontweight="bold")
                    else:
                        ax.text(angle, tension, ".",
                                ha="center", va="center",
                                fontsize=7, color="#aaaaaa")

            ax.set_xlabel("Tether angle [deg]")
            if wi == 0:
                ax.set_ylabel("Tension [N]")
            ax.set_title(f"wind={wind:.0f} m/s  |  {label}", fontsize=9)
            plt.colorbar(im, ax=ax, label="Collective [deg]")

    plt.tight_layout()
    plt.show()


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
    p.add_argument("--no-plot", action="store_true", help="skip visualisation")
    p.add_argument("--v-out",   type=float, default=V_REEL_OUT,
                   help=f"reel-out v_target (default {V_REEL_OUT})")
    p.add_argument("--v-in",    type=float, default=V_REEL_IN,
                   help=f"reel-in  v_target (default {V_REEL_IN})")
    args = p.parse_args()

    if args.load:
        print(f"\nLoading grid from {args.load} ...")
        raw  = np.load(args.load, allow_pickle=False)
        grid = {k: raw[k] for k in raw.files}
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

        v_targets    = [args.v_out, args.v_in]
        wind_list    = list(map(float, cfg["wind_list"]))    # type: ignore[arg-type]
        angle_list   = list(map(float, cfg["angle_list"]))   # type: ignore[arg-type]
        tension_list = list(map(float, cfg["tension_list"])) # type: ignore[arg-type]
        t_end        = float(cfg["t_end"])                   # type: ignore[arg-type]
        dt           = float(cfg["dt"])                      # type: ignore[arg-type]

        print(f"\nComputing envelope map  ({'full' if args.full else 'quick'}, PID sweep)")
        print(f"  wind={wind_list}  "
              f"angles={angle_list[0]:.0f}..{angle_list[-1]:.0f}  "
              f"tensions={tension_list[0]:.0f}..{tension_list[-1]:.0f}")
        total_sims = len(wind_list) * len(angle_list) * len(tension_list) * len(v_targets)
        print(f"  Simulations: {total_sims}  "
              f"(vs {total_sims * 41} for 41-collective sweep)")
        print()

        grid = compute_grid(
            wind_list    = wind_list,
            angle_list   = angle_list,
            tension_list = tension_list,
            v_targets    = v_targets,
            t_end        = t_end,
            dt           = dt,
        )

        if args.save:
            path = Path(args.save)
            path.parent.mkdir(parents=True, exist_ok=True)
            np.savez(path, **{k: np.array(v) if not isinstance(v, np.ndarray) else v
                              for k, v in grid.items()})
            print(f"  Saved to {path}")

    if not args.no_plot:
        visualize(grid, v_reel_out=args.v_out, v_reel_in=args.v_in)
