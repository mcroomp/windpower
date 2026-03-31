"""
test_deschutter_validation.py — Validation against De Schutter 2018 known results.

Source: De Schutter J., Leuthold R., Diehl M. (2018). "Optimal Control of a
Rigid-Wing Rotary Kite System for Airborne Wind Energy." IFAC Proceedings.
Full paper: https://publications.syscop.de/DeSchutter2018.pdf

Parameters from Table I and Eq. 25 of the paper
-------------------------------------------------
    N = 3 blades (wings)
    L_w = 1.5 m  (wing span, Table I)
    A = 12       (aspect ratio, Table I)
    chord = L_w/A = 0.125 m
    L_b = 1.60 m (connecting arm length, Section IV-A optimal design result)
    r_root = L_b = 1.60 m
    R = L_b + L_w = 3.10 m
    r_cp = L_b + 2/3·L_w = 1.60 + 1.00 = 2.60 m   (Section II-D: 2/3 span from root)

Aerodynamics (Eq. 25) — thin airfoil, elliptical finite wing:
    C_{L,k} = (2π / (1 + 2/A)) · α_k   →  CL_alpha = 12π/7 ≈ 5.385 /rad, CL0 = 0
    C_{D,k} = C_{D,0} + C_{L,k}² / (π·A·O_e)   →  CD0=0.01, Oe=0.8
    |α_k| ≤ 15°  (stall constraint, Eq. 28)

    NOTE: No CL0 term — De Schutter uses thin airfoil theory (symmetric wing
    assumption, from von Mises [18]).  NOT the SG6042 or SG6040 airfoil.

Mass/inertia — from Weyel (2025) Table 1 (preliminary estimates, De Schutter does
not provide explicit values):
    m = 40 kg,  J_r = [20, 20, 40] kg·m²

Operating condition used in De Schutter simulations:
    ω_θ = 15π ≈ 47.12 rad/s  (tip speed ratio λ=7 at 10 m/s wind, from Weyel Table 1)
    V_wind = 10 m/s

Derived quantities at r_cp = 2.60 m, ω = 47.12 rad/s:
    v_tan   = 47.12 × 2.60 = 122.5 m/s
    At design tether orientation (v_axial=8.51, v_inplane=5.25):
      phi   = atan2(8.51, 122.5) = 3.97° = 0.0693 rad
      CL    = 5.385 × 0.0693 = 0.373  (zero collective)
      q     = 0.5 × 1.22 × 122.5² ≈ 9136 Pa
      S     = 0.125 × 1.5 = 0.1875 m²
    T_approx ≈ 3 × 9136 × 0.1875 × 0.370 ≈ 1905 N  (rough, no induction)

Actuator disk area (Eq. 18):
    A_K = π((L_b + L_w)² - L_b²) = π(3.10² - 1.60²) = 22.15 m²
"""

import sys
import math
from pathlib import Path

import numpy as np
import pytest

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parents[2]))
sys.path.insert(0, str(_HERE.parent))
sys.path.insert(0, str(_HERE))

from _helpers import (
    ALL_MODELS, DESIGN_BODY_Z, DESIGN_R_HUB, DESIGN_WIND,
)
import rotor_definition as rd

# ── De Schutter 2018 reference operating conditions ────────────────────────
DS_OMEGA  = 15.0 * math.pi   # ≈ 47.12 rad/s  (Weyel Table 1: λ=7, V=10 m/s)
DS_V_WIND = np.array([0.0, 10.0, 0.0])   # NED: East wind = Y axis

# Exact CL_alpha from De Schutter Eq. 25 with A=12
DS_CL_ALPHA_EXACT = 2.0 * math.pi / (1.0 + 2.0 / 12.0)   # = 12π/7 ≈ 5.385 /rad

DS_DESIGN_KWARGS = dict(
    collective_rad = 0.0,
    tilt_lon       = 0.0,
    tilt_lat       = 0.0,
    R_hub          = DESIGN_R_HUB,
    v_hub_world    = np.zeros(3),
    omega_rotor    = DS_OMEGA,
    wind_world     = DS_V_WIND,
    t              = 10.0,
)

DS_EDGEWISE_KWARGS = dict(
    collective_rad = 0.0,
    tilt_lon       = 0.0,
    tilt_lat       = 0.0,
    R_hub          = np.eye(3),
    v_hub_world    = np.zeros(3),
    omega_rotor    = DS_OMEGA,
    wind_world     = DS_V_WIND,
    t              = 10.0,
)


# ────────────────────────────────────────────────────────────────────────────
# 1. YAML parameter checks — exact De Schutter 2018 values
# ────────────────────────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def ds_rotor():
    return rd.load("de_schutter_2018")


def test_ds_n_blades(ds_rotor):
    """3 blades (wings), 120° apart."""
    assert ds_rotor.n_blades == 3


def test_ds_wing_span(ds_rotor):
    """L_w = 1.5 m  (De Schutter Table I)."""
    span = ds_rotor.radius_m - ds_rotor.root_cutout_m
    assert math.isclose(span, 1.5, rel_tol=1e-6), \
        f"span = {span:.4f} m, expected 1.5 m (L_w from Table I)"


def test_ds_radius(ds_rotor):
    """R = L_b + L_w = 1.60 + 1.50 = 3.10 m  (Section IV-A optimal design)."""
    assert math.isclose(ds_rotor.radius_m, 3.1, rel_tol=1e-4), \
        f"R = {ds_rotor.radius_m:.4f} m, expected 3.10 m"


def test_ds_root_cutout(ds_rotor):
    """r_root = L_b = 1.60 m  (connecting arm length, Section IV-A)."""
    assert math.isclose(ds_rotor.root_cutout_m, 1.6, rel_tol=1e-4), \
        f"r_root = {ds_rotor.root_cutout_m:.4f} m, expected 1.60 m (L_b)"


def test_ds_chord(ds_rotor):
    """c = L_w/A = 1.5/12 = 0.125 m  (Table I: A=12, L_w=1.5m)."""
    assert math.isclose(ds_rotor.chord_m, 0.125, rel_tol=1e-6)


def test_ds_aspect_ratio(ds_rotor):
    """A = 12  (De Schutter Table I — explicitly stated)."""
    assert math.isclose(ds_rotor.aspect_ratio, 12.0, rel_tol=0.01), \
        f"AR = {ds_rotor.aspect_ratio:.2f}, expected 12.0"


def test_ds_r_cp(ds_rotor):
    """
    r_cp = L_b + 2/3·L_w = 1.60 + 1.00 = 2.60 m
    (Section II-D: centre of pressure at 2/3 of wing span from root)
    """
    expected = 1.6 + (2.0 / 3.0) * 1.5
    assert math.isclose(ds_rotor.r_cp_m, expected, rel_tol=0.01), \
        f"r_cp = {ds_rotor.r_cp_m:.4f} m, expected {expected:.4f} m"


def test_ds_CL0(ds_rotor):
    """
    CL0 = 0  — De Schutter uses THIN AIRFOIL THEORY (Eq. 25): no CL0 term.
    C_{L,k} = (2π/(1+2/A)) × α_k  (symmetric/uncambered wing assumption).
    """
    assert math.isclose(ds_rotor.CL0, 0.0, abs_tol=1e-6), \
        f"CL0 = {ds_rotor.CL0:.4f}, expected 0.0 (thin airfoil: no zero-AoA lift)"


def test_ds_CL_alpha(ds_rotor):
    """
    CL_alpha = 2π/(1+2/A) = 12π/7 ≈ 5.385 /rad  (De Schutter Eq. 25, A=12).
    NOT 0.87/rad — that was Weyel's SG6040 approximation, not the De Schutter formula.
    """
    expected = DS_CL_ALPHA_EXACT
    assert math.isclose(ds_rotor.CL_alpha_per_rad, expected, rel_tol=0.005), \
        (f"CL_alpha = {ds_rotor.CL_alpha_per_rad:.4f} /rad, "
         f"expected {expected:.4f} /rad  (2π/(1+2/12) from Eq. 25)")


def test_ds_CD0(ds_rotor):
    """C_{D,0} = 0.01  (De Schutter Table I)."""
    assert math.isclose(ds_rotor.CD0, 0.01, rel_tol=1e-6)


def test_ds_oswald(ds_rotor):
    """O_e = 0.8  (De Schutter Table I)."""
    assert math.isclose(ds_rotor.oswald_efficiency, 0.8, rel_tol=1e-6)


def test_ds_alpha_stall(ds_rotor):
    """α_stall = 15°  (De Schutter Eq. 28: |α_k| ≤ 15°)."""
    assert math.isclose(ds_rotor.alpha_stall_deg, 15.0, rel_tol=1e-6)


def test_ds_solidity(ds_rotor):
    """σ = N·c/(π·R) = 3×0.125/(π×3.1) ≈ 0.0385."""
    sigma_expected = 3 * 0.125 / (math.pi * 3.1)
    assert math.isclose(ds_rotor.solidity, sigma_expected, rel_tol=0.01), \
        f"solidity = {ds_rotor.solidity:.4f}, expected {sigma_expected:.4f}"


def test_ds_mass_40kg(ds_rotor):
    """m = 40 kg  (Weyel Table 1 estimate — De Schutter does not specify)."""
    assert math.isclose(ds_rotor.mass_kg, 40.0, rel_tol=0.05)


# ────────────────────────────────────────────────────────────────────────────
# 2. Computed thrust at De Schutter operating conditions
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_ds_thrust_positive_at_design(AeroClass, ds_rotor):
    """All models must produce positive thrust at De Schutter design conditions."""
    model  = AeroClass(ds_rotor)
    forces = model.compute_forces(**DS_DESIGN_KWARGS)
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T      = float(np.dot(forces[:3], disk_n))
    assert T > 0, f"{AeroClass.__name__} [DS2018]: T={T:.1f} N ≤ 0 at design point"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_ds_thrust_order_of_magnitude(AeroClass, ds_rotor):
    """
    At ω=47.12 rad/s and design tether orientation, thrust is in [100, 10000] N.

    With r_cp=2.6m: v_tan=122.5 m/s, q≈9136 Pa, CL≈0.37 at zero collective.
    T_approx ≈ 3 × 9136 × 0.1875 × 0.37 ≈ 1905 N  (pre-induction, no stall).
    Actual value varies with model; range is generous.
    """
    model  = AeroClass(ds_rotor)
    forces = model.compute_forces(**DS_DESIGN_KWARGS)
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T      = float(np.dot(forces[:3], disk_n))
    assert 100.0 <= T <= 10000.0, \
        f"{AeroClass.__name__} [DS2018]: T={T:.1f} N out of [100, 10000] N"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_ds_thrust_increases_with_collective(AeroClass, ds_rotor):
    """Higher collective → more thrust for De Schutter rotor."""
    model  = AeroClass(ds_rotor)
    f_lo   = model.compute_forces(**{**DS_DESIGN_KWARGS, "collective_rad": 0.0})
    f_hi   = model.compute_forces(**{**DS_DESIGN_KWARGS, "collective_rad": 0.10})
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T_lo   = float(np.dot(f_lo[:3], disk_n))
    T_hi   = float(np.dot(f_hi[:3], disk_n))
    assert T_hi > T_lo, \
        f"{AeroClass.__name__} [DS2018]: T did not increase: {T_lo:.1f} → {T_hi:.1f}"


# ────────────────────────────────────────────────────────────────────────────
# 3. CT at De Schutter conditions
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_ds_CT_in_physical_range(AeroClass, ds_rotor):
    """
    CT = T / (ρ·A_K·(Ω·R)²) must be in [0.0001, 0.5].

    With ω=47.12 rad/s, R=3.1 m: tip_v = 146.1 m/s
    A_K = π(3.1² - 1.6²) = 22.15 m²
    CT = T / (1.22 × 22.15 × 146.1²) = T / 576,000

    For T ≈ 1000 N: CT ≈ 0.00174  (small — very high tip speed)
    """
    model  = AeroClass(ds_rotor)
    forces = model.compute_forces(**DS_DESIGN_KWARGS)
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T      = float(np.dot(forces[:3], disk_n))

    A_K    = math.pi * (ds_rotor.radius_m ** 2 - ds_rotor.root_cutout_m ** 2)
    rho    = ds_rotor.rho_kg_m3
    tip_v2 = (DS_OMEGA * ds_rotor.radius_m) ** 2
    CT     = T / max(rho * A_K * tip_v2, 1e-6)

    assert 0.0001 <= CT <= 0.5, \
        f"{AeroClass.__name__} [DS2018]: CT={CT:.6f} out of [0.0001, 0.5]"


# ────────────────────────────────────────────────────────────────────────────
# 4. H-force direction
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_ds_H_force_positive_edgewise(AeroClass, ds_rotor):
    """
    Horizontal disk + horizontal wind + small collective → H-force (Fy, NED East) > 0.
    Advance ratio μ = v_inplane/(ω·R) = 10/(47.12×3.1) = 0.068.

    Collective=0.05 rad is required for GlauertStateBEM: with CL0=0 (symmetric
    thin airfoil) and zero axial flow, the Glauert strip model gives T=0 at
    zero collective (no inflow angle → no lift → no H-force). This is physically
    correct for De Schutter's symmetric wing model.
    """
    model  = AeroClass(ds_rotor)
    forces = model.compute_forces(**{**DS_EDGEWISE_KWARGS, "collective_rad": 0.05})
    # NED: East = Y axis (index 1)
    assert forces[1] > 0, \
        f"{AeroClass.__name__} [DS2018 edgewise]: Fy={forces[1]:.1f} N ≤ 0"


# ────────────────────────────────────────────────────────────────────────────
# 5. Rotor validation
# ────────────────────────────────────────────────────────────────────────────

def test_ds_rotor_validation_no_errors(ds_rotor):
    """De Schutter rotor definition must pass all validation ERROR checks."""
    issues = ds_rotor.validate()
    errors = [i for i in issues if i.level == "ERROR"]
    assert len(errors) == 0, \
        f"De Schutter rotor validation errors: {[str(e) for e in errors]}"
