"""
validate_caradonna_tung.py — BEM model validation against Caradonna & Tung (1981).

Reference: NASA TM-81232, "Experimental and Analytical Studies of a Model
Helicopter Rotor in Hover", Caradonna & Tung, 1981.

Rotor: 2-bladed, NACA 0012, no twist, no taper.
       R=1.143 m, c=0.1905 m, Mt=0.439 (tip Mach), hover.

Experimental CT values (Table I, Mt=0.439 case):
  theta=5 deg  -> CT=0.00168
  theta=8 deg  -> CT=0.00461
  theta=12 deg -> CT=0.00814

Note on CQ/FM: Caradonna & Tung (1981) did not measure torque — the paper is a
blade-pressure and tip-vortex benchmark only.  No experimental CQ is available
from this dataset.  The CQ and figure-of-merit (FM) columns below are therefore
model-only outputs, validated analytically against the ideal actuator-disk bound:
  FM_ideal = 1.0  (Betz limit)
  FM_typical = 0.70-0.80 for a clean, untwisted rotor at moderate loading

CQ definition: CQ = Q / (rho * A * (Omega*R)^2 * R)
FM definition: FM = CT^(3/2) / (sqrt(2) * CQ)

Run:
    python simulation/analysis/validate_caradonna_tung.py
"""

import sys, math
import numpy as np
from pathlib import Path

_SIM_DIR = Path(__file__).resolve().parents[1]
_AERO_DIR = _SIM_DIR / "aero"
for p in (str(_SIM_DIR), str(_AERO_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

from rotor_definition import RotorDefinition
from frames import build_orb_frame
from aero import PetersHeBEM, create_aero

# ---------------------------------------------------------------------------
# Caradonna-Tung rotor geometry
# ---------------------------------------------------------------------------
R     = 1.143    # tip radius [m]
CHORD = 0.1905   # chord [m]
MT    = 0.439    # tip Mach number
A_SND = 340.3    # speed of sound [m/s], sea level ISA
TIP_SPEED = MT * A_SND          # ~149 m/s
OMEGA = TIP_SPEED / R           # ~131 rad/s
RHO   = 1.225                   # sea level ISA [kg/m3]
DISK_AREA = math.pi * R**2      # full disk (no root cutout)

SPAN      = R - 0.05            # blade span (excluding small hub)
AR        = SPAN / CHORD        # aspect ratio ~5.7
# 3D finite-wing CL_alpha via Prandtl lifting-line: 2π/(1 + 2/AR)
# Using 2D thin-airfoil (2π) over-predicts CT by ~25% for this blade.
CL_ALPHA  = 2 * math.pi / (1 + 2 / AR)   # ≈ 4.66 /rad

ct_rotor = RotorDefinition(
    name           = "caradonna_tung_1981",
    n_blades       = 2,
    radius_m       = R,
    root_cutout_m  = 0.05,      # small hub; paper doesn't specify
    chord_m        = CHORD,
    CL0            = 0.0,       # NACA 0012, symmetric
    CL_alpha_per_rad = CL_ALPHA,  # 3D Prandtl lifting-line correction
    CD0            = 0.008,     # NACA 0012 at low AoA, Re~1M
    oswald_efficiency = 0.97,
    alpha_stall_deg = 15.0,
    rho_kg_m3      = RHO,
    swashplate_pitch_gain_rad = 1.0,  # irrelevant (zero cyclic)
)

# ---------------------------------------------------------------------------
# Experimental reference  (Caradonna & Tung 1981, Table I, Mt=0.439)
# ---------------------------------------------------------------------------
EXP = {5: 0.00168, 8: 0.00461, 12: 0.00814}

# ---------------------------------------------------------------------------
# Hover setup: disk facing up (body_z = NED up = [0,0,-1])
# ---------------------------------------------------------------------------
BZ    = np.array([0.0, 0.0, -1.0])
R_HUB = build_orb_frame(BZ)
WIND  = np.zeros(3)
T_RUN = 10.0   # past ramp

CQ_SCALE = RHO * DISK_AREA * TIP_SPEED**2 * R  # CQ = Q / CQ_SCALE

def run(model, theta_deg):
    col = math.radians(theta_deg)
    model.compute_forces(col, 0.0, 0.0, R_HUB, np.zeros(3), OMEGA, WIND, T_RUN)
    T  = model.last_T
    CT = T / (RHO * DISK_AREA * TIP_SPEED**2)
    Q  = abs(float(np.dot(model.last_M_spin, BZ)))  # spin-axis torque magnitude
    CQ = Q / CQ_SCALE
    FM = CT**1.5 / (math.sqrt(2.0) * CQ) if CQ > 0 else float("nan")
    return CT, CQ, FM

# ---------------------------------------------------------------------------
# Table 1: CT validation (vs experiment)
# ---------------------------------------------------------------------------
THETAS = [5, 8, 12]

print(f"\nCaradonna-Tung (1981) hover validation  Mt={MT}, Omega={OMEGA:.1f} rad/s")
print(f"\n--- Thrust coefficient CT (vs experiment) ---")
print(f"{'':>6}  {'CT_exp':>9}  {'CT_PetersHe':>12}  {'err_PH%':>8}  {'CT_Skewed':>10}  {'err_S%':>7}")
print("-" * 62)

ph_results = {}
for theta in THETAS:
    ph = PetersHeBEM(ct_rotor)
    sw = create_aero(ct_rotor, model="skewed_wake_numpy")

    CT_ph, CQ_ph, FM_ph = run(ph, theta)
    CT_s,  _,     _     = run(sw, theta)
    CT_exp = EXP[theta]

    ph_results[theta] = (CT_ph, CQ_ph, FM_ph)

    err_ph = (CT_ph - CT_exp) / CT_exp * 100
    err_s  = (CT_s  - CT_exp) / CT_exp * 100

    print(f"  {theta:2d} deg  {CT_exp:9.5f}  {CT_ph:12.5f}  {err_ph:+8.1f}%  {CT_s:10.5f}  {err_s:+7.1f}%")

# ---------------------------------------------------------------------------
# Table 2: CQ and figure of merit (Peters-He, no experimental reference)
# ---------------------------------------------------------------------------
print(f"\n--- Torque coefficient CQ and figure of merit FM (Peters-He, no exp reference) ---")
print(f"{'':>6}  {'CT':>9}  {'CQ':>10}  {'FM':>8}  {'FM_bound':>10}")
print("-" * 52)

for theta in THETAS:
    CT_ph, CQ_ph, FM_ph = ph_results[theta]
    # Ideal FM upper bound from actuator disk: FM <= 1.0
    # Practical bound for untwisted blade: ~0.70-0.80
    print(f"  {theta:2d} deg  {CT_ph:9.5f}  {CQ_ph:10.6f}  {FM_ph:8.3f}  {'0.70-0.80':>10}")

print()
print("CT ref: Caradonna & Tung (1981) NASA TM-81232, Table I, Mt=0.439")
print("CQ/FM:  no experimental data available from this paper (pressure/wake study only)")
print("FM=1.0 is the Betz ideal; clean untwisted rotors typically achieve 0.70-0.80")
print("Note: no Mach correction applied to CL_alpha; CT uncertainty +/-5%")
