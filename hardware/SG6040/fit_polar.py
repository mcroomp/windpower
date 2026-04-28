"""
fit_polar.py -- Fit BEM airfoil parameters from an XFoil polar CSV.

Reads xf-sg6040-il-500000.csv and outputs CL0, CL_alpha_per_rad, CD0,
oswald_efficiency, and alpha_stall_deg ready to paste into beaupoil_2026.yaml.

Usage:
    python hardware/SG6040/fit_polar.py
"""

import sys
import math
import csv
import numpy as np
from pathlib import Path

CSV_PATH = Path(__file__).parent / "xf-sg6040-il-500000.csv"

# Blade geometry (from beaupoil_2026.yaml) — needed for Oswald fit
AR       = 10.0   # aspect ratio = (R - r_root) / chord = 2.0 / 0.20
RE       = 500_000

# ---------------------------------------------------------------------------
# Load polar
# ---------------------------------------------------------------------------
alphas, CLs, CDs = [], [], []
with CSV_PATH.open() as fh:
    for line in fh:
        line = line.strip()
        if line.startswith("Alpha"):
            break  # found data header, stop skipping
    for line in fh:
        parts = line.strip().split(",")
        if len(parts) < 3:
            continue
        try:
            alphas.append(float(parts[0]))
            CLs.append(float(parts[1]))
            CDs.append(float(parts[2]))
        except ValueError:
            continue

alpha = np.array(alphas)
CL    = np.array(CLs)
CD    = np.array(CDs)

# ---------------------------------------------------------------------------
# 1. Identify linear CL region
#    Use alpha range where d²CL/dalpha² is small (curvature < threshold).
#    Conservative: fit only -4 to +8 deg.
# ---------------------------------------------------------------------------
LINEAR_LO = -4.0
LINEAR_HI =  8.0
mask_lin  = (alpha >= LINEAR_LO) & (alpha <= LINEAR_HI)

alpha_lin = alpha[mask_lin]
CL_lin    = CL[mask_lin]

# Linear regression: CL = CL0 + CL_alpha * alpha_rad
alpha_lin_rad = np.radians(alpha_lin)
A = np.column_stack([np.ones_like(alpha_lin_rad), alpha_lin_rad])
coeffs, res, _, _ = np.linalg.lstsq(A, CL_lin, rcond=None)
CL0_fit      = float(coeffs[0])
CL_alpha_fit = float(coeffs[1])

# R² of the linear fit
CL_pred  = A @ coeffs
ss_res   = float(np.sum((CL_lin - CL_pred)**2))
ss_tot   = float(np.sum((CL_lin - CL_lin.mean())**2))
r2_CL    = 1.0 - ss_res / ss_tot

# ---------------------------------------------------------------------------
# 2. Stall angle: where CL peaks
# ---------------------------------------------------------------------------
i_stall     = int(np.argmax(CL))
alpha_stall = float(alpha[i_stall])
CL_max      = float(CL[i_stall])

# Also compute where the linear fit error exceeds 0.05 (practical stall onset)
CL_linear_extrap = CL0_fit + CL_alpha_fit * np.radians(alpha)
deviation        = np.abs(CL - CL_linear_extrap)
# Find first alpha > 0 where deviation > 0.05
onset_mask = (alpha > 0) & (deviation > 0.05)
if onset_mask.any():
    alpha_stall_onset = float(alpha[onset_mask][0])
else:
    alpha_stall_onset = alpha_stall

# ---------------------------------------------------------------------------
# 3. CD0: drag at zero lift (interpolate)
# ---------------------------------------------------------------------------
# Find alpha where CL ~ 0 (zero-lift angle)
i_zero = int(np.argmin(np.abs(CL)))
# Interpolate to exact zero crossing
if i_zero > 0 and CL[i_zero - 1] * CL[i_zero] < 0:
    t = -CL[i_zero - 1] / (CL[i_zero] - CL[i_zero - 1])
    CD0_fit = float(CD[i_zero - 1] + t * (CD[i_zero] - CD[i_zero - 1]))
else:
    CD0_fit = float(CD[i_zero])

alpha_zero_lift = float(alpha[i_zero])

# ---------------------------------------------------------------------------
# 4. Oswald efficiency: fit CD = CD0 + CL²/(pi*AR*e) over operating CL range.
#
#    Note: for BEM, CD is the 2D section drag.  The parabolic drag polar
#    CD = CD0 + CL²/(pi*AR*e) is a simplification.  SG6040 has a flat drag
#    bucket so fit over the operating range (CL 0.4–1.1, alpha ~0–7 deg)
#    rather than the full linear range to get the best match at actual
#    operating conditions.
# ---------------------------------------------------------------------------
CL_OP_LO, CL_OP_HI = 0.4, 1.1
mask_oswald = (CL >= CL_OP_LO) & (CL <= CL_OP_HI)
CL_os  = CL[mask_oswald]
CD_os  = CD[mask_oswald]
dCD    = CD_os - CD0_fit
dCD    = np.maximum(dCD, 1e-9)
e_vals = CL_os**2 / (dCD * math.pi * AR)
e_fit  = float(np.median(e_vals))
e_fit  = min(max(e_fit, 0.5), 1.0)  # physical upper bound is 1.0

# Also compute residual: mean CD error over operating range with fitted e
CD_model_os = CD0_fit + CL_os**2 / (math.pi * AR * e_fit)
cd_rmse = float(np.sqrt(np.mean((CD_os - CD_model_os)**2)))

# ---------------------------------------------------------------------------
# 5. CL/CD max (best glide)
# ---------------------------------------------------------------------------
LoD     = CL / np.maximum(CD, 1e-6)
i_best  = int(np.argmax(LoD))
CL_best = float(CL[i_best])
CD_best = float(CD[i_best])
a_best  = float(alpha[i_best])

# ---------------------------------------------------------------------------
# Print results
# ---------------------------------------------------------------------------
print(f"\nSG6040 polar fit  Re={RE:,}  (XFoil Ncrit=9)")
print(f"Linear fit range: {LINEAR_LO} to {LINEAR_HI} deg  R²={r2_CL:.4f}")
print()
print(f"  CL0              = {CL0_fit:.4f}    (CL at alpha=0)")
print(f"  CL_alpha_per_rad = {CL_alpha_fit:.4f}    (/rad, linear fit)")
print(f"  CD0              = {CD0_fit:.5f}   (drag at zero lift, alpha~{alpha_zero_lift:.2f} deg)")
print(f"  oswald_eff       = {e_fit:.3f}    (fit over CL {CL_OP_LO}-{CL_OP_HI}, CD RMSE={cd_rmse:.5f})")
print(f"  alpha_stall_deg  = {alpha_stall:.1f}    (CL peak)")
print(f"  alpha_stall_onset= {alpha_stall_onset:.1f}    (first >0.05 dev from linear — use this)")
print(f"  NOTE: SG6040 stalls gradually; CL still rises past onset. Model clips here.")
print(f"  CL_max           = {CL_max:.4f}")
print()
print(f"Best L/D: {CL_best/CD_best:.1f} at alpha={a_best:.2f} deg  (CL={CL_best:.4f}, CD={CD_best:.5f})")
print()
print("--- YAML snippet (paste into beaupoil_2026.yaml airfoil section) ---")
print(f"""
airfoil:
  designation:       SG6040
  source:            "XFoil polar xf-sg6040-il-500000, Re=500k, Ncrit=9"

  CL0:               {CL0_fit:.4f}
  CL_alpha_per_rad:  {CL_alpha_fit:.4f}
  CD0:               {CD0_fit:.5f}
  oswald_efficiency: {e_fit:.3f}
  alpha_stall_deg:   {alpha_stall_onset:.1f}
""")

# ---------------------------------------------------------------------------
# Diagnostic: print per-alpha deviation from linear fit
# ---------------------------------------------------------------------------
print("--- CL linear fit deviation (alpha -6 to +16 deg) ---")
print(f"  {'alpha':>6}  {'CL_data':>8}  {'CL_fit':>8}  {'dev':>8}  {'CD':>8}")
mask_diag = (alpha >= -6) & (alpha <= 16)
for a, cl, cd in zip(alpha[mask_diag], CL[mask_diag], CD[mask_diag]):
    cl_fit = CL0_fit + CL_alpha_fit * math.radians(a)
    dev    = cl - cl_fit
    flag   = " <-- stall onset" if abs(dev) > 0.05 and a > 0 and abs(dev - 0.05) < 0.03 else ""
    print(f"  {a:6.2f}  {cl:8.4f}  {cl_fit:8.4f}  {dev:+8.4f}  {cd:8.5f}{flag}")
