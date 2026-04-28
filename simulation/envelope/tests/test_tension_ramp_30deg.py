"""
test_tension_ramp_30deg.py -- Homotopy ramp at el=30 deg, reel-out v_target=-0.8 m/s.

Step 1: run the standalone ramp (own ODE loop, omega=28, vel=0, 40 s settle at T=25 N,
        then ramp at 5 N/s).  This is known to track all the way to 1000 N.
Step 2: capture the full IC dict (vel, omega, col, cyclic, integrators, aero_state)
        at each 25 N grid crossing.
Step 3: verify each IC is reproducible — restart simulate_point from that IC and
        check it immediately stays within tolerance.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rotor_definition as rd
from aero import create_aero
from frames import build_orb_frame
from envelope.point_mass import tether_hat, balance_bz, simulate_point

EL           = 30.0
WIND         = 10.0
V_TARGET     = -0.8
OMEGA_INIT   = 28.0
T_SETTLE     = 40.0
RAMP_RATE    = 5.0      # N/s
DT           = 0.02

TENSION_LIST = list(range(25, 1025, 25))   # 25 … 1000 N in 25 N steps


# ── standalone ramp (identical physics to the working _run_ramp test) ──────────

def _run_standalone_ramp():
    """Own ODE loop — omega=28, vel=0, 40 s settle at T=25 N, then ramp to 1000 N."""
    rotor   = rd.default()
    aero    = create_aero(rotor, model="peters_he")
    dk      = rotor.dynamics_kwargs()
    mass    = dk["mass"]
    I_spin  = dk["I_spin"]
    weight  = mass * 9.81
    clamp   = math.radians(8.6)
    wind_v  = np.array([0.0, -float(WIND), 0.0])
    t_hat   = tether_hat(EL)

    omega    = float(OMEGA_INIT)
    vel      = np.zeros(3)
    col_now  = 0.0
    c_lon    = c_lat = 0.0
    int_vx   = int_vy = int_vcol = 0.0

    T_START      = float(TENSION_LIST[0])
    T_MAX        = float(TENSION_LIST[-1])
    tension      = T_START
    t_ramp_start = None
    next_grid    = T_START       # next grid tension to sample

    samples      = []            # (ti, v_along, col_rad, ic_dict)
    diverged_at  = None

    total_s     = T_SETTLE + (T_MAX - T_START) / RAMP_RATE + 20.0
    total_steps = int(total_s / DT)

    for i in range(total_steps):
        t_sim = i * DT

        if t_sim < T_SETTLE:
            tension = T_START
        else:
            if t_ramp_start is None:
                t_ramp_start = t_sim
            tension = min(T_START + RAMP_RATE * (t_sim - t_ramp_start), T_MAX)

        bz     = balance_bz(EL, tension, mass)
        R      = build_orb_frame(bz)
        F_teth = tension * t_hat

        try:
            f = aero.compute_forces(col_now, c_lon, c_lat, R, vel.copy(),
                                    omega, wind_v, t=45.0)
        except (OverflowError, ValueError, FloatingPointError):
            diverged_at = (t_sim, tension)
            break

        thrust = float(np.dot(f.F_world, bz))
        Q_net  = float(np.dot(aero.last_M_spin, bz))
        F_net  = f.F_world + F_teth + np.array([0.0, 0.0, weight])

        omega = max(0.0, omega + (Q_net / I_spin) * DT)
        vel   = vel + (F_net / mass) * DT

        if not np.all(np.isfinite(vel)):
            diverged_at = (t_sim, tension)
            break

        v_perp  = vel - float(np.dot(vel, t_hat)) * t_hat
        t_norm  = max(abs(thrust), 1.0)
        c_lon_r = -(0.5 / t_norm * v_perp[0] + 0.1 / t_norm * int_vx)
        c_lat_r = -(0.5 / t_norm * v_perp[1] + 0.1 / t_norm * int_vy)
        c_lon   = float(np.clip(c_lon_r, -clamp, clamp))
        c_lat   = float(np.clip(c_lat_r, -clamp, clamp))
        if abs(c_lon) < clamp:
            int_vx += v_perp[0] * DT
        if abs(c_lat) < clamp:
            int_vy += v_perp[1] * DT

        v_along_now = float(np.dot(vel, t_hat))
        err = v_along_now - V_TARGET
        int_vcol += err * DT
        col_now = float(np.clip(col_now + 0.02 * err + 0.005 * int_vcol, -0.25, 0.20))

        # sample when tension crosses the next grid point
        if t_sim >= T_SETTLE and tension >= next_grid - 0.5 * RAMP_RATE * DT:
            ti      = TENSION_LIST.index(int(next_grid))
            v_along = float(np.dot(vel, t_hat))
            ic_out  = dict(
                col      = col_now,
                vel      = vel.copy(),
                omega    = omega,
                c_lon    = c_lon,    c_lat    = c_lat,
                int_vx   = int_vx,   int_vy   = int_vy,
                int_vcol = int_vcol,
                aero_state = aero.to_dict(),
            )
            samples.append((ti, v_along, col_now, ic_out))
            next_grid += float(TENSION_LIST[1] - TENSION_LIST[0])
            if next_grid > T_MAX + 0.1:
                break

    return samples, diverged_at


_RAMP_RESULT = None

def _get_ramp():
    global _RAMP_RESULT
    if _RAMP_RESULT is None:
        _RAMP_RESULT = _run_standalone_ramp()
    return _RAMP_RESULT


# ── tests ──────────────────────────────────────────────────────────────────────

def test_ramp_tracks_to_1000n(capsys):
    """Standalone ramp must not diverge and must track v_target throughout."""
    samples, diverged_at = _get_ramp()

    print(f"\n  el={EL:.0f}  wind={WIND}  v_target={V_TARGET:+.2f}  "
          f"omega_init={OMEGA_INIT}  ramp={RAMP_RATE} N/s")
    if diverged_at:
        print(f"  Diverged at T={diverged_at[1]:.0f} N  t={diverged_at[0]:.1f} s")
    else:
        print(f"  Tracked to T={TENSION_LIST[samples[-1][0]]:.0f} N  "
              f"({len(samples)} grid points)")

    print(f"\n  {'T':>6}  {'v_along':>8}  {'col_deg':>8}  {'omega':>7}")
    for ti, v_along, col_rad, ic in samples:
        marker = " <--" if abs(v_along - V_TARGET) > 0.15 else ""
        print(f"  {TENSION_LIST[ti]:6.0f}  {v_along:+8.3f}  "
              f"{math.degrees(col_rad):+8.2f}  {ic['omega']:.3f}{marker}")

    assert diverged_at is None, f"diverged at T={diverged_at[1]:.0f} N"
    bad = [(TENSION_LIST[ti], v_along) for ti, v_along, *_ in samples
           if abs(v_along - V_TARGET) > 0.15]
    assert not bad, f"lost tracking at {bad}"


def test_ics_are_reproducible(capsys):
    """Each captured IC must restart cleanly: 5 s sim stays within tolerance."""
    samples, _ = _get_ramp()
    failures = []

    print(f"\n  Verifying {len(samples)} ICs with 5 s restart sims")
    print(f"  {'T':>6}  {'v_along_0':>10}  {'v_along_5s':>10}  {'ok':>4}")

    for ti, v_along_ramp, col_rad, ic in samples:
        tension = float(TENSION_LIST[ti])
        r = simulate_point(
            col           = col_rad,
            elevation_deg = EL,
            tension_n     = tension,
            wind_speed    = WIND,
            v_target      = V_TARGET,
            t_end         = 5.0,
            dt            = DT,
            ic            = ic,
        )
        eq = r["eq"]
        va5 = eq.get("v_along", float("nan"))
        ok  = math.isfinite(va5) and abs(va5 - V_TARGET) < 0.15
        print(f"  {tension:6.0f}  {v_along_ramp:+10.3f}  {va5:+10.3f}  {'ok' if ok else 'FAIL'}")
        if not ok:
            failures.append((tension, v_along_ramp, va5))

    assert not failures, (
        f"{len(failures)} ICs not reproducible: "
        f"{[(t, f'{v0:+.3f}', f'{v5:+.3f}') for t, v0, v5 in failures]}"
    )
