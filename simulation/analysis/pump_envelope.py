"""
pump_envelope.py — Performance envelope for the combined tension+altitude controller.

Physics summary
---------------
Two actuators: collective (thrust magnitude) and body_z tilt (cyclic direction).
Two outputs:   tether tension T  and  hub altitude h.

At static equilibrium, the altitude constraint fixes a relationship between
body_z tilt angle δ (measured from tether direction toward zenith) and thrust:

    sin(δ) = mg·cos(el) / T_thrust        [altitude maintenance]
    T       = T_thrust·cos(δ) − mg·sin(el) [tether tension]

Eliminating T_thrust:

    T = mg · cos(el + δ) / sin(δ)         [combined formula]

So tension is controlled by δ alone (for altitude-maintaining flight).
Given a target tension T*:

    δ* = arctan( mg·cos(el) / (T* + mg·sin(el)) )
    T_thrust* = mg·cos(el) / sin(δ*)

The "impossible" regime is where T_thrust* is outside the aero model's
achievable range [T_thrust_min, T_thrust_max] at the given conditions.

Usage
-----
    simulation/.venv/Scripts/python.exe simulation/analysis/pump_envelope.py
"""

import sys, math, json
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aero import create_aero
import rotor_definition as rd
from controller import col_min_for_altitude_rad

# ── Constants ──────────────────────────────────────────────────────────────────

rotor  = rd.default()
aero   = create_aero(rotor)
MASS   = rotor.mass_kg
G      = 9.81
MG     = MASS * G
WIND   = np.array([0.0, 10.0, 0.0])   # 10 m/s East (NED)

d       = json.loads((Path(__file__).resolve().parents[1] / "steady_state_starting.json").read_text())
IC_POS  = np.array(d["pos"])
IC_R0   = np.array(d["R0"]).reshape(3, 3)
OMEGA   = float(d["omega_spin"])
COL_MIN = -0.28
COL_MAX =  0.10

BREAK_LOAD_N   = 620.0
SAFETY_LOAD_N  = 496.0   # 80% break load


# ── Core formulas ──────────────────────────────────────────────────────────────

def delta_for_tension(el_rad: float, target_t: float) -> float:
    """Body_z tilt angle δ from tdir toward zenith for target tension (rad)."""
    return math.atan2(MG * math.cos(el_rad), target_t + MG * math.sin(el_rad))


def thrust_for_delta(el_rad: float, delta_rad: float) -> float:
    """Required rotor thrust magnitude for altitude maintenance at tilt δ."""
    s = math.sin(delta_rad)
    if s < 1e-9:
        return float("inf")
    return MG * math.cos(el_rad) / s


def tension_from_delta(el_rad: float, delta_rad: float, t_thrust: float) -> float:
    """Tether tension given thrust and tilt angle."""
    return t_thrust * math.cos(delta_rad) - MG * math.sin(el_rad)


def body_z_eq(pos: np.ndarray, delta_rad: float) -> np.ndarray:
    """Body_z unit vector: tdir rotated δ toward elevation-up (zenith direction)."""
    tlen  = float(np.linalg.norm(pos))
    tdir  = pos / tlen
    el    = float(math.asin(max(-1.0, min(1.0, float(-pos[2]) / max(tlen, 1e-6)))))
    az    = float(math.atan2(pos[1], pos[0]))
    ce, se = math.cos(el), math.sin(el)
    ca, sa = math.cos(az), math.sin(az)
    e_up  = np.array([-se * ca, -se * sa, -ce])   # elevation-up unit vector
    raw   = math.cos(delta_rad) * tdir + math.sin(delta_rad) * e_up
    return raw / np.linalg.norm(raw)


# ── Aero model: thrust vs collective at IC orientation ─────────────────────────

def aero_thrust_along_bz(col: float, R_hub: np.ndarray = None) -> float:
    """Rotor thrust magnitude (along body_z) from aero model."""
    R = IC_R0 if R_hub is None else R_hub
    res = aero.compute_forces(col, 0.0, 0.0, R, np.zeros(3), OMEGA, WIND, t=45.0)
    return float(np.linalg.norm(res.F_world))


def col_for_thrust(target_thrust: float, R_hub: np.ndarray = None,
                   n_steps: int = 40) -> "float | None":
    """Binary-search collective that gives target_thrust. Returns None if infeasible."""
    t_lo = aero_thrust_along_bz(COL_MIN, R_hub)
    t_hi = aero_thrust_along_bz(COL_MAX, R_hub)
    if target_thrust < t_lo or target_thrust > t_hi:
        return None
    lo, hi = COL_MIN, COL_MAX
    for _ in range(n_steps):
        mid = (lo + hi) / 2
        t_mid = aero_thrust_along_bz(mid, R_hub)
        if t_mid < target_thrust:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


# ── Validation: verify combined formula at IC conditions ───────────────────────

def validate_formula() -> None:
    print("=" * 70)
    print("FORMULA VALIDATION at IC conditions")
    print(f"  mass={MASS:.1f} kg, mg={MG:.1f} N, wind=10 m/s East")
    tlen_ic = float(np.linalg.norm(IC_POS))
    el_ic   = math.degrees(math.asin(-IC_POS[2] / tlen_ic))
    print(f"  IC: elevation={el_ic:.1f} deg, altitude={-IC_POS[2]:.1f} m")
    print()

    el = math.radians(el_ic)
    print(f"  {'T_target_N':>12}  {'delta_deg':>10}  {'T_thrust_N':>12}  {'T_check_N':>11}  {'alt_check':>10}")
    print(f"  {'-'*12}  {'-'*10}  {'-'*12}  {'-'*11}  {'-'*10}")
    for t_star in [55, 100, 150, 200, 250, 300, 350, 400, 435, 496, 550]:
        d_rad  = delta_for_tension(el, float(t_star))
        t_thr  = thrust_for_delta(el, d_rad)
        t_chk  = tension_from_delta(el, d_rad, t_thr)
        # Altitude check: sin(δ) * T_thrust == mg * cos(el)
        alt_ok = abs(math.sin(d_rad) * t_thr - MG * math.cos(el)) < 0.01
        print(f"  {t_star:>12.0f}  {math.degrees(d_rad):>10.2f}  {t_thr:>12.1f}  "
              f"{t_chk:>11.1f}  {'OK' if alt_ok else 'FAIL':>10}")
    print()


# ── Performance envelope: achievable tensions at each elevation ─────────────────

def elevation_envelope() -> None:
    print("=" * 70)
    print("PERFORMANCE ENVELOPE: achievable tension range per elevation")
    print(f"  (using IC aero model: omega={OMEGA:.1f} rad/s, wind=10 m/s East)")
    print(f"  COL range: [{COL_MIN:.2f}, {COL_MAX:.2f}] rad")
    print(f"  Break load: {BREAK_LOAD_N:.0f} N, Safety: {SAFETY_LOAD_N:.0f} N")
    print()

    t_min_aero = aero_thrust_along_bz(COL_MIN)
    t_max_aero = aero_thrust_along_bz(COL_MAX)
    print(f"  Aero thrust at COL_MIN ({COL_MIN:.2f} rad): {t_min_aero:.1f} N")
    print(f"  Aero thrust at COL_MAX ({COL_MAX:.2f} rad): {t_max_aero:.1f} N")
    print()

    print(f"  {'el_deg':>7}  {'T_min_N':>9}  {'T_max_N':>9}  {'col_for_200N':>13}  "
          f"{'col_for_55N':>12}  {'status':>20}")
    print(f"  {'-'*7}  {'-'*9}  {'-'*9}  {'-'*13}  {'-'*12}  {'-'*20}")

    for el_deg in [5, 10, 15, 16.4, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80]:
        el = math.radians(el_deg)

        # Min tension: at COL_MIN, with optimal body_z tilt
        d_min  = delta_for_tension(el, 0.0)   # δ for T*=0 (maximum tilt)
        d_at_colmin = delta_for_tension(el, 0.0)
        t_thr_min = t_min_aero

        if t_thr_min < MG * math.cos(el):
            # Can't even maintain altitude at COL_MIN
            t_min = float("nan")
            t_max = float("nan")
            col_200 = col_55 = "infeasible"
            status = "ALTITUDE FAIL"
        else:
            d_opt   = math.asin(MG * math.cos(el) / t_thr_min)
            t_min   = tension_from_delta(el, d_opt, t_thr_min)
            d_opt_m = math.asin(min(1.0, MG * math.cos(el) / t_max_aero)) if t_max_aero >= MG * math.cos(el) else 0.0
            t_max   = tension_from_delta(el, d_opt_m, t_max_aero)

            # Collective needed for T*=200N
            d200    = delta_for_tension(el, 200.0)
            thr200  = thrust_for_delta(el, d200)
            c200    = col_for_thrust(thr200)
            col_200 = f"{c200:+.3f}" if c200 is not None else "infeasible"

            # Collective needed for T*=55N
            d55     = delta_for_tension(el, 55.0)
            thr55   = thrust_for_delta(el, d55)
            c55     = col_for_thrust(thr55)
            col_55  = f"{c55:+.3f}" if c55 is not None else "infeasible"

            if t_min > BREAK_LOAD_N:
                status = "BREAKS TETHER"
            elif t_min > SAFETY_LOAD_N:
                status = "ABOVE SAFETY"
            elif t_min > 200.0:
                status = "T>200 always"
            elif t_min > 55.0:
                status = "T>55 always"
            else:
                status = "OK"

        t_min_s = f"{t_min:8.1f}" if not math.isnan(t_min) else "     nan"
        t_max_s = f"{t_max:8.1f}" if not math.isnan(t_max) else "     nan"
        print(f"  {el_deg:>7.1f}  {t_min_s}  {t_max_s}  {col_200:>13}  {col_55:>12}  {status:>20}")
    print()


# ── Reel-out feasibility check ─────────────────────────────────────────────────

def reel_out_check() -> None:
    print("=" * 70)
    print("REEL-OUT CHECK: what tension is achievable during reel-out?")
    el_deg = float(math.degrees(math.asin(-IC_POS[2] / np.linalg.norm(IC_POS))))
    el     = math.radians(el_deg)
    print(f"  Reel-out elevation ~{el_deg:.1f} deg (IC position, tether pays out)")
    print()

    print(f"  {'collective':>12}  {'T_thrust_N':>12}  {'d_opt_deg':>10}  "
          f"{'T_tether_N':>12}  {'note':>15}")
    print(f"  {'-'*12}  {'-'*12}  {'-'*10}  {'-'*12}  {'-'*15}")
    for col in [COL_MIN, -0.26, -0.24, -0.22, -0.20, -0.18, -0.15, -0.10, -0.05, 0.0, 0.05, COL_MAX]:
        t_thr  = aero_thrust_along_bz(col)
        if t_thr < MG * math.cos(el):
            print(f"  {col:>+12.3f}  {t_thr:>12.1f}  {'---':>10}  {'---':>12}  altitude fail")
            continue
        d_opt  = math.asin(MG * math.cos(el) / t_thr)
        t_tet  = tension_from_delta(el, d_opt, t_thr)
        note   = ""
        if t_tet > BREAK_LOAD_N:
            note = "BREAKS TETHER"
        elif t_tet > SAFETY_LOAD_N:
            note = "above safety"
        elif col == -0.18:
            note = "<- IC equilibrium"
        print(f"  {col:>+12.3f}  {t_thr:>12.1f}  {math.degrees(d_opt):>10.2f}  "
              f"{t_tet:>12.1f}  {note:>15}")
    print()


# ── Reel-in: find best target elevation ────────────────────────────────────────

def reel_in_check() -> None:
    print("=" * 70)
    print("REEL-IN CHECK: maximum elevation where T_min < 200N (reel-out tension)")
    print("  (want T_reel_in < T_reel_out for net positive energy)")
    print()

    t_min_aero = aero_thrust_along_bz(COL_MIN)

    print(f"  {'el_deg':>7}  {'T_min_N':>9}  {'T_min < 200?':>13}  {'col_55N':>10}  "
          f"{'viable reel-in?':>17}")
    print(f"  {'-'*7}  {'-'*9}  {'-'*13}  {'-'*10}  {'-'*17}")

    best_el = None
    for el_deg in [5, 10, 15, 16.4, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80]:
        el = math.radians(el_deg)
        if t_min_aero < MG * math.cos(el):
            print(f"  {el_deg:>7.1f}  {'altitude fail':>9}")
            continue
        d_opt   = math.asin(MG * math.cos(el) / t_min_aero)
        t_min   = tension_from_delta(el, d_opt, t_min_aero)
        low_ok  = "YES" if t_min < 200.0 else "no"

        d55     = delta_for_tension(el, 55.0)
        thr55   = thrust_for_delta(el, d55)
        c55     = col_for_thrust(thr55)
        col_55s = f"{c55:+.3f}" if c55 is not None else "infeasible"

        viable  = (t_min < 200.0 and t_min < BREAK_LOAD_N)
        if viable and best_el is None:
            best_el = el_deg
        print(f"  {el_deg:>7.1f}  {t_min:>9.1f}  {low_ok:>13}  {col_55s:>10}  "
              f"{'<-- viable!' if viable else '':>17}")

    print()
    if best_el is not None:
        print(f"  Best reel-in elevation for minimum tension: {best_el:.1f} deg")
    print()


# ── Main ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print()
    print("pump_envelope.py — Combined tension+altitude controller feasibility")
    print(f"IC: pos={IC_POS.round(1)}, omega_spin={OMEGA:.1f} rad/s")
    print()
    validate_formula()
    elevation_envelope()
    reel_out_check()
    reel_in_check()
