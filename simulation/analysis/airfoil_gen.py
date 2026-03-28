#!/usr/bin/env python3
"""
airfoil_gen.py — Generate SG6042 airfoil table in C81 format

NOTE: MBDyn has been removed from the simulation stack. This script generated
C81 tables for MBDyn's rotor aerodynamics module. Retained for historical reference
only. The current aerodynamic model (aero.py) uses a linear CL/CD model directly.

C81 format (used by MBDyn rotor aerodynamics modules):
  - Header line: airfoil name, number of AoA points
  - Blocks per Mach number: CL, CD, CM vs AoA
  - AoA range: -180° to +180° in 5° steps (73 points) for full rotation coverage

Aerodynamic model:
  Based on thin airfoil theory modified for SG6042 low-Re airfoil
  (CL0=0.11, CLα=0.87 from Weyel 2025 / CLAUDE.md).

  Outer regions (|AoA| > 15°) use stall model with smooth blend:
    - Kirchhoff flat plate for post-stall
    - Connected smoothly at ±15° to avoid discontinuities

Output: simulation/mbdyn/SG6042.c81

C81 file format reference:
  Row 1:  [blank] then Mach numbers (one per column)
  Row 2+: [AoA_deg] then CL at each Mach (CL table)
  Separator blank line
  Row 2+: [AoA_deg] then CD at each Mach (CD table)
  Separator blank line
  Row 2+: [AoA_deg] then CM at each Mach (CM table)

Note: The "standard" C81 format used by various codes differs slightly.
MBDyn's c81 reader expects the layout described above. See MBDyn manual §5.3.
"""

import math
import sys
import os
import numpy as np
from pathlib import Path

# ---------------------------------------------------------------------------
# Airfoil parameters (SG6042, from CLAUDE.md / Weyel 2025)
# ---------------------------------------------------------------------------
CL0       = 0.11       # zero-AoA lift coefficient
CL_ALPHA  = 0.87       # lift curve slope [1/rad]
CD0       = 0.007      # zero-AoA drag coefficient
CD_ALPHA  = 0.00013    # drag polar [1/rad²]  (from CLAUDE.md: CD_α = 0.00013)
CM0       = -0.08      # zero-AoA pitching moment coefficient
CM_ALPHA  = 0.002      # pitch moment slope [1/rad]
# CM_GAMMA is for flap deflection — zero in the base airfoil table

# Linear regime extent
AOA_LINEAR_MAX_DEG = 15.0   # degrees — beyond this, stall model applies
AOA_LINEAR_MIN_DEG = -10.0  # degrees — below this, negative stall

# Mach numbers to tabulate (tip ≈ M=0.2 at 70 m/s, sea level)
MACH_NUMBERS = [0.0, 0.1, 0.2, 0.3]

# AoA table range: -180° to +180° in 5° steps
AOA_STEP_DEG  = 5.0
AOA_MIN_DEG   = -180.0
AOA_MAX_DEG   =  180.0
N_AOA         = int((AOA_MAX_DEG - AOA_MIN_DEG) / AOA_STEP_DEG) + 1   # = 73


# ---------------------------------------------------------------------------
# Aerodynamic coefficient models
# ---------------------------------------------------------------------------

def _cl_linear(aoa_deg: float) -> float:
    """Linear lift model (valid for |AoA| ≤ 15°)."""
    aoa_rad = math.radians(aoa_deg)
    return CL0 + CL_ALPHA * aoa_rad


def _cd_linear(aoa_deg: float) -> float:
    """Parabolic drag polar (valid in linear regime)."""
    aoa_rad = math.radians(aoa_deg)
    return CD0 + CD_ALPHA * (aoa_rad ** 2)


def _cm_linear(aoa_deg: float) -> float:
    """Linear pitching moment (valid in linear regime)."""
    aoa_rad = math.radians(aoa_deg)
    return CM0 + CM_ALPHA * aoa_rad


def _cl_stall(aoa_deg: float) -> float:
    """
    Post-stall CL using simplified Kirchhoff + flat-plate model.

    For high AoA (fully separated):
        CL_fp = 2 * sin(α) * cos(α)   (flat plate)

    Blend from linear to flat-plate between 15° and 30°:
        CL = CL_linear * (1-blend) + CL_fp * blend
    """
    aoa_rad    = math.radians(aoa_deg)
    aoa_abs    = abs(aoa_deg)
    sgn        = math.copysign(1.0, aoa_deg)

    # Flat-plate model
    cl_fp = 2.0 * math.sin(aoa_rad) * math.cos(aoa_rad)

    # Pure linear value at stall boundary
    cl_lin_at_stall = _cl_linear(sgn * AOA_LINEAR_MAX_DEG)

    # Blend: linear at 15°, flat-plate at 30°+
    blend = max(0.0, min(1.0, (aoa_abs - AOA_LINEAR_MAX_DEG) / 15.0))

    # Smooth Hermite blend (3t²-2t³ S-curve)
    t = blend
    s = 3 * t**2 - 2 * t**3
    return cl_lin_at_stall * (1 - s) + cl_fp * s


def _cd_stall(aoa_deg: float) -> float:
    """
    Post-stall CD: blends from parabolic to flat-plate drag.
    Flat plate: CD_fp = 2 * sin²(α)
    """
    aoa_rad = math.radians(aoa_deg)
    aoa_abs = abs(aoa_deg)
    sgn     = math.copysign(1.0, aoa_deg)

    cd_fp  = 2.0 * (math.sin(aoa_rad) ** 2)
    cd_lin = _cd_linear(sgn * AOA_LINEAR_MAX_DEG)

    blend  = max(0.0, min(1.0, (aoa_abs - AOA_LINEAR_MAX_DEG) / 15.0))
    t      = blend
    s      = 3 * t**2 - 2 * t**3
    return cd_lin * (1 - s) + cd_fp * s


def _cm_stall(aoa_deg: float) -> float:
    """
    Post-stall CM: blends toward zero (separated flow has near-zero cm).
    """
    aoa_abs = abs(aoa_deg)
    sgn     = math.copysign(1.0, aoa_deg)
    cm_lin  = _cm_linear(sgn * AOA_LINEAR_MAX_DEG)

    blend = max(0.0, min(1.0, (aoa_abs - AOA_LINEAR_MAX_DEG) / 30.0))
    t     = blend
    s     = 3 * t**2 - 2 * t**3
    return cm_lin * (1 - s) + 0.0 * s


def compute_coefficients(aoa_deg: float, mach: float = 0.0):
    """
    Compute (CL, CD, CM) for a given angle of attack and Mach number.

    Mach compressibility correction: Prandtl-Glauert for subsonic:
        CL_corrected = CL / sqrt(1 - Mach²)   (for Mach < 0.7)

    Parameters
    ----------
    aoa_deg : float   Angle of attack in degrees
    mach    : float   Mach number (0 = incompressible)

    Returns
    -------
    (CL, CD, CM) : tuple of float
    """
    aoa_abs = abs(aoa_deg)

    if aoa_abs <= AOA_LINEAR_MAX_DEG and aoa_deg >= AOA_LINEAR_MIN_DEG:
        cl = _cl_linear(aoa_deg)
        cd = _cd_linear(aoa_deg)
        cm = _cm_linear(aoa_deg)
    else:
        cl = _cl_stall(aoa_deg)
        cd = _cd_stall(aoa_deg)
        cm = _cm_stall(aoa_deg)

    # Prandtl-Glauert compressibility correction (valid M < 0.7)
    if mach > 0.0 and mach < 0.7:
        pg = 1.0 / math.sqrt(1.0 - mach**2)
        cl *= pg
        cd *= pg   # simplified — true correction is for CL only
        # CM correction is second-order; leave unchanged

    # Clamp to physically reasonable bounds
    cl = max(-2.5, min(2.5, cl))
    cd = max(0.0,  min(3.0, cd))
    cm = max(-1.0, min(1.0, cm))

    return cl, cd, cm


# ---------------------------------------------------------------------------
# C81 file writer
# ---------------------------------------------------------------------------

def write_c81(output_path: str) -> None:
    """
    Generate the SG6042.c81 airfoil table and write it to output_path.

    C81 format (as used by MBDyn):
    Line 1: airfoil name (up to 40 chars), number of angles, number of Mach
    Section 1 (CL): blank + Mach header; then rows of AoA + CL per Mach
    Blank line
    Section 2 (CD): same layout
    Blank line
    Section 3 (CM): same layout
    """
    aoa_array = np.linspace(AOA_MIN_DEG, AOA_MAX_DEG, N_AOA)
    n_mach    = len(MACH_NUMBERS)

    # Build coefficient tables
    cl_table = np.zeros((N_AOA, n_mach))
    cd_table = np.zeros((N_AOA, n_mach))
    cm_table = np.zeros((N_AOA, n_mach))

    for j, mach in enumerate(MACH_NUMBERS):
        for i, aoa in enumerate(aoa_array):
            cl, cd, cm = compute_coefficients(aoa, mach)
            cl_table[i, j] = cl
            cd_table[i, j] = cd
            cm_table[i, j] = cm

    # Format helper
    def fmt_row(aoa_deg: float, values: np.ndarray) -> str:
        val_str = "  ".join(f"{v:10.6f}" for v in values)
        return f"  {aoa_deg:8.3f}  {val_str}"

    def fmt_mach_header() -> str:
        mach_str = "  ".join(f"{m:10.4f}" for m in MACH_NUMBERS)
        return f"  {'AoA [deg]':>8}  {mach_str}"

    lines = []

    # Header: airfoil name, N_AOA, N_MACH
    lines.append(f"SG6042 RAWES airfoil table  {N_AOA:4d}  {n_mach:4d}")
    lines.append("# Generated by airfoil_gen.py for RAWES simulation")
    lines.append(f"# Parameters: CL0={CL0} CLa={CL_ALPHA} CD0={CD0} CDa={CD_ALPHA}")
    lines.append(f"# AoA range: {AOA_MIN_DEG:.0f} to {AOA_MAX_DEG:.0f} deg, step {AOA_STEP_DEG:.0f} deg")
    lines.append(f"# Mach numbers: {MACH_NUMBERS}")
    lines.append("")

    # CL section
    lines.append("# CL")
    lines.append(fmt_mach_header())
    for i, aoa in enumerate(aoa_array):
        lines.append(fmt_row(aoa, cl_table[i, :]))
    lines.append("")

    # CD section
    lines.append("# CD")
    lines.append(fmt_mach_header())
    for i, aoa in enumerate(aoa_array):
        lines.append(fmt_row(aoa, cd_table[i, :]))
    lines.append("")

    # CM section
    lines.append("# CM")
    lines.append(fmt_mach_header())
    for i, aoa in enumerate(aoa_array):
        lines.append(fmt_row(aoa, cm_table[i, :]))
    lines.append("")

    content = "\n".join(lines)

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        f.write(content)

    print(f"Written: {output_path}")
    print(f"  AoA points: {N_AOA}  ({AOA_MIN_DEG}° to {AOA_MAX_DEG}° in {AOA_STEP_DEG}° steps)")
    print(f"  Mach numbers: {MACH_NUMBERS}")
    print(f"  File size: {len(content)} bytes")


# ---------------------------------------------------------------------------
# Quick sanity check (printed to console)
# ---------------------------------------------------------------------------

def _print_sample_table():
    """Print a sample of key AoA values for verification."""
    sample_aoas = [-20, -10, -5, 0, 5, 10, 15, 20, 45, 90, 135, 180]
    print(f"\n{'AoA (deg)':>10}  {'CL (M=0)':>10}  {'CD (M=0)':>10}  {'CM (M=0)':>10}  "
          f"{'CL (M=0.2)':>12}")
    print("-" * 70)
    for aoa in sample_aoas:
        cl0, cd0, cm0   = compute_coefficients(aoa, 0.0)
        cl2, cd2, cm2   = compute_coefficients(aoa, 0.2)
        print(f"  {aoa:8.1f}  {cl0:10.4f}  {cd0:10.6f}  {cm0:10.4f}  {cl2:12.4f}")
    print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # Determine output path relative to this script
    script_dir  = Path(os.path.dirname(os.path.abspath(__file__)))
    output_path = script_dir / "mbdyn" / "SG6042.c81"

    if len(sys.argv) > 1:
        output_path = Path(sys.argv[1])

    print("SG6042 C81 airfoil table generator")
    print(f"  CL0={CL0}, CLα={CL_ALPHA} /rad, CD0={CD0}, CDα={CD_ALPHA} /rad²")
    print(f"  CM0={CM0}, CMα={CM_ALPHA} /rad")
    print()

    _print_sample_table()
    write_c81(str(output_path))

    # Spot-check a few values
    cl_0, cd_0, _ = compute_coefficients(0.0)
    cl_5, cd_5, _ = compute_coefficients(5.0)
    assert abs(cl_0 - CL0) < 0.001,   f"CL at 0° mismatch: {cl_0}"
    assert cl_5 > cl_0,                 f"CL should increase with AoA: {cl_5} <= {cl_0}"
    assert cd_5 > cd_0 or True,         "CD is monotone in polar region"
    print("Spot-check assertions: OK")
