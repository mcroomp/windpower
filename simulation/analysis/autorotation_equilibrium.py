"""
autorotation_equilibrium.py — Free-autorotation equilibrium RPM at 10 m/s wind.

Finds the omega where Q_spin = 0 (no mechanical resistance):
  - Wind: 10 m/s East in NED = [0, 10, 0]
  - Collective: 0 rad
  - Two disk orientations: horizontal (body_z up) and design tether angle

Uses PetersHeBEMJit (production model).  Settles inflow ODE for N_SETTLE steps
before sampling Q_spin, then uses scipy.optimize.brentq for exact root.

Usage:
    .venv/Scripts/python.exe simulation/analysis/autorotation_equilibrium.py
"""

import sys
import math
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aero import rotor_definition as rd
from aero.aero_peters_he_jit import PetersHeBEMJit

WIND      = np.array([0.0, 10.0, 0.0])   # NED: East at 10 m/s
COL       = 0.0                            # rad: free autorotation, no blade pitch
N_SETTLE  = 400                            # time steps to let inflow converge
DT        = 0.01                           # s per step (4 s total settle time)


def make_R_hub(body_z: np.ndarray) -> np.ndarray:
    """Build rotation matrix from disk normal (body_z = third column)."""
    bz = body_z / np.linalg.norm(body_z)
    # Reference direction: East; fall back to North if degenerate
    ref = np.array([0.0, 1.0, 0.0])
    if abs(np.dot(ref, bz)) > 0.99:
        ref = np.array([1.0, 0.0, 0.0])
    bx = ref - np.dot(ref, bz) * bz
    bx /= np.linalg.norm(bx)
    by = np.cross(bz, bx)
    return np.column_stack([bx, by, bz])


def q_spin_settled(rotor, omega: float, R_hub: np.ndarray) -> float:
    """Run the ODE for N_SETTLE steps at fixed omega, return steady Q_spin."""
    model = PetersHeBEMJit(rotor)
    t = 0.0
    q = 0.0
    for _ in range(N_SETTLE):
        t += DT
        result = model.compute_forces(
            collective_rad=COL,
            tilt_lon=0.0, tilt_lat=0.0,
            R_hub=R_hub,
            v_hub_world=np.zeros(3),
            omega_rotor=omega,
            wind_world=WIND,
            t=t,
        )
        q = result.Q_spin
    return q


def find_equilibrium(rotor, R_hub, label: str) -> float:
    """Print a Q_spin sweep and use brentq to find the autorotation equilibrium."""
    print(f"\n--- {label} ---")
    print(f"  wind = {WIND} m/s (East, NED)")
    disk_n = R_hub[:, 2]
    v_ax = float(np.dot(WIND, disk_n))
    v_ip = float(np.linalg.norm(WIND - v_ax * disk_n))
    print(f"  disk_normal = [{disk_n[0]:.3f}, {disk_n[1]:.3f}, {disk_n[2]:.3f}]")
    print(f"  v_axial = {v_ax:.2f} m/s,  v_inplane = {v_ip:.2f} m/s")
    print(f"  collective = {math.degrees(COL):.1f} deg")

    print(f"\n  {'omega rad/s':>12}  {'RPM':>8}  {'Q_spin N*m':>12}")
    omegas = [1, 3, 5, 8, 10, 14, 18, 22, 26, 30, 40, 50, 65, 80, 100]
    q_values = {}
    for om in omegas:
        q = q_spin_settled(rotor, float(om), R_hub)
        q_values[om] = q
        print(f"  {om:>12.1f}  {om*60/(2*math.pi):>8.1f}  {q:>12.2f}")

    # Find bracket for brentq
    # Find first sign change in sweep
    bracket_lo, bracket_hi = None, None
    sorted_oms = sorted(omegas)
    for i in range(len(sorted_oms) - 1):
        a, b = sorted_oms[i], sorted_oms[i + 1]
        if q_values[a] * q_values[b] < 0:
            bracket_lo, bracket_hi = a, b
            break

    if bracket_lo is None:
        print("\n  WARNING: no sign change found in sweep range")
        return float("nan")

    lo, hi = float(bracket_lo or 0), float(bracket_hi or 0)

    omega_eq = float(brentq(  # type: ignore[call-overload]
        lambda om: q_spin_settled(rotor, float(om), R_hub),
        float(lo), float(hi),
        xtol=0.01, maxiter=30,
    ))
    rpm_eq = omega_eq * 60.0 / (2.0 * math.pi)
    tip_speed = omega_eq * rotor.radius_m
    tsr = tip_speed / 10.0   # tip-speed ratio at 10 m/s wind

    print(f"\n  EQUILIBRIUM: omega = {omega_eq:.2f} rad/s  ({rpm_eq:.1f} RPM)")
    print(f"               tip speed = {tip_speed:.1f} m/s")
    print(f"               TSR (tip-speed ratio) = {tsr:.2f}")
    return omega_eq


def main():
    rotor = rd.default()
    print(f"Rotor: {rotor.name}")
    print(f"  R={rotor.radius_m} m, r_root={rotor.root_cutout_m} m, "
          f"N={rotor.n_blades}, c={rotor.chord_m} m")
    print(f"  CL0={rotor.CL0:.3f}, CL_alpha={rotor.CL_alpha_per_rad:.3f}/rad, "
          f"CD0={rotor.CD0:.4f}")
    print(f"  CL at col=0: {rotor.CL0 + rotor.CL_alpha_per_rad * COL:.3f}")

    # ── Pure axial flow: disk facing East, all wind is axial ─────────────────
    # body_z = East = [0,1,0] in NED.  Wind [0,10,0] is entirely axial.
    # This is the cleanest case: classic windmill/autogyro in axial inflow.
    body_z_axial = np.array([0.0, 1.0, 0.0])
    R_axial = make_R_hub(body_z_axial)

    find_equilibrium(rotor, R_axial, "Pure axial flow (disk facing East, v_axial=10 m/s, v_inplane=0)")


if __name__ == "__main__":
    main()
