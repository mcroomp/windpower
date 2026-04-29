"""
test_aero_models.py — Model-specific correctness tests.

Tests that cover behaviour specific to a particular rotor definition or model
family (not universal invariants — those live in test_aero_physics.py).

  § De Schutter 2018 rotor — parameter spot-checks + physics at DS conditions
  § Cross-model spread      — thrust and H-force within one order of magnitude

Source for De Schutter parameters: De Schutter J., Leuthold R., Diehl M. (2018).
"Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy."
Table I and Eq. 25.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parents[2]))   # simulation/
sys.path.insert(0, str(_HERE.parent))       # simulation/aero/
sys.path.insert(0, str(_HERE))              # simulation/aero/tests/

from _helpers import ALL_MODELS, HOVER_KWARGS, DESIGN_KWARGS, DESIGN_BODY_Z, DESIGN_R_HUB
from aero import rotor_definition as rd


def _thrust(forces, disk_n=None):
    if disk_n is None:
        disk_n = np.array([0.0, 0.0, 1.0])
    return float(np.dot(forces[:3], disk_n))


# ── De Schutter 2018 operating conditions ────────────────────────────────────

DS_OMEGA  = 15.0 * math.pi   # ~47.12 rad/s  (Weyel Table 1: lambda=7, V=10 m/s)
DS_V_WIND = np.array([0.0, 10.0, 0.0])   # NED: East wind = Y axis

DS_CL_ALPHA_EXACT = 2.0 * math.pi / (1.0 + 2.0 / 12.0)   # 12*pi/7 ~ 5.385 /rad

DS_DESIGN_KWARGS = dict(
    collective_rad=0.0, tilt_lon=0.0, tilt_lat=0.0,
    R_hub=DESIGN_R_HUB, v_hub_world=np.zeros(3),
    omega_rotor=DS_OMEGA, wind_world=DS_V_WIND, t=10.0,
)

DS_EDGEWISE_KWARGS = dict(
    collective_rad=0.0, tilt_lon=0.0, tilt_lat=0.0,
    R_hub=np.eye(3), v_hub_world=np.zeros(3),
    omega_rotor=DS_OMEGA, wind_world=DS_V_WIND, t=10.0,
)


# ═══════════════════════════════════════════════════════════════════════════════
# § De Schutter 2018 rotor
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.fixture(scope="module")
def ds_rotor():
    return rd.load("de_schutter_2018")


# ── Rotor parameter spot-checks ───────────────────────────────────────────────

def test_ds_n_blades(ds_rotor):
    """3 blades (wings), 120 deg apart."""
    assert ds_rotor.n_blades == 3


def test_ds_wing_span(ds_rotor):
    """L_w = 1.5 m  (De Schutter Table I)."""
    span = ds_rotor.radius_m - ds_rotor.root_cutout_m
    assert math.isclose(span, 1.5, rel_tol=1e-6), \
        f"span = {span:.4f} m, expected 1.5 m"


def test_ds_radius(ds_rotor):
    """R = L_b + L_w = 1.60 + 1.50 = 3.10 m  (Section IV-A optimal design)."""
    assert math.isclose(ds_rotor.radius_m, 3.1, rel_tol=1e-4), \
        f"R = {ds_rotor.radius_m:.4f} m, expected 3.10 m"


def test_ds_root_cutout(ds_rotor):
    """r_root = L_b = 1.60 m  (connecting arm length, Section IV-A)."""
    assert math.isclose(ds_rotor.root_cutout_m, 1.6, rel_tol=1e-4), \
        f"r_root = {ds_rotor.root_cutout_m:.4f} m, expected 1.60 m"


def test_ds_chord(ds_rotor):
    """c = L_w/A = 1.5/12 = 0.125 m  (Table I)."""
    assert math.isclose(ds_rotor.chord_m, 0.125, rel_tol=1e-6)


def test_ds_aspect_ratio(ds_rotor):
    """A = 12  (De Schutter Table I)."""
    assert math.isclose(ds_rotor.aspect_ratio, 12.0, rel_tol=0.01), \
        f"AR = {ds_rotor.aspect_ratio:.2f}, expected 12.0"


def test_ds_r_cp(ds_rotor):
    """r_cp = L_b + 2/3*L_w = 1.60 + 1.00 = 2.60 m  (Section II-D)."""
    expected = 1.6 + (2.0 / 3.0) * 1.5
    assert math.isclose(ds_rotor.r_cp_m, expected, rel_tol=0.01), \
        f"r_cp = {ds_rotor.r_cp_m:.4f} m, expected {expected:.4f} m"


def test_ds_CL0(ds_rotor):
    """CL0 = 0 — thin airfoil theory, no zero-AoA lift (Eq. 25)."""
    assert math.isclose(ds_rotor.CL0, 0.0, abs_tol=1e-6), \
        f"CL0 = {ds_rotor.CL0:.4f}, expected 0.0"


def test_ds_CL_alpha(ds_rotor):
    """CL_alpha = 2*pi/(1+2/A) = 12*pi/7 ~ 5.385 /rad  (Eq. 25, A=12)."""
    assert math.isclose(ds_rotor.CL_alpha_3D_per_rad, DS_CL_ALPHA_EXACT, rel_tol=0.005), \
        f"CL_alpha = {ds_rotor.CL_alpha_3D_per_rad:.4f} /rad, expected {DS_CL_ALPHA_EXACT:.4f}"


def test_ds_CD0(ds_rotor):
    """C_D0 = 0.01  (Table I)."""
    assert math.isclose(ds_rotor.CD0, 0.01, rel_tol=1e-6)


def test_ds_oswald(ds_rotor):
    """O_e = 0.8  (Table I)."""
    assert math.isclose(ds_rotor.oswald_efficiency, 0.8, rel_tol=1e-6)


def test_ds_alpha_stall(ds_rotor):
    """alpha_stall = 15 deg  (Eq. 28: |alpha_k| <= 15 deg)."""
    assert math.isclose(ds_rotor.alpha_stall_deg, 15.0, rel_tol=1e-6)


def test_ds_solidity(ds_rotor):
    """sigma = N*c/(pi*R) = 3*0.125/(pi*3.1) ~ 0.0385."""
    expected = 3 * 0.125 / (math.pi * 3.1)
    assert math.isclose(ds_rotor.solidity, expected, rel_tol=0.01), \
        f"solidity = {ds_rotor.solidity:.4f}, expected {expected:.4f}"


def test_ds_mass_40kg(ds_rotor):
    """m = 40 kg  (Weyel Table 1 estimate)."""
    assert math.isclose(ds_rotor.mass_kg, 40.0, rel_tol=0.05)


def test_ds_rotor_validation_no_errors(ds_rotor):
    """De Schutter rotor definition must pass all validation ERROR checks."""
    errors = [i for i in ds_rotor.validate() if i.level == "ERROR"]
    assert len(errors) == 0, \
        f"De Schutter rotor validation errors: {[str(e) for e in errors]}"


# ── De Schutter physics at DS conditions ─────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_ds_thrust_positive_at_design(AeroClass, ds_rotor):
    """All models must produce positive thrust at De Schutter design conditions."""
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T = _thrust(AeroClass(ds_rotor).compute_forces(**DS_DESIGN_KWARGS), disk_n)
    assert T > 0, f"{AeroClass.__name__} [DS2018]: T={T:.1f} N <= 0 at design point"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_ds_thrust_order_of_magnitude(AeroClass, ds_rotor):
    """
    At omega=47.12 rad/s and design orientation, thrust must be in [100, 10000] N.
    T_approx ~ 3 * 9136 Pa * 0.1875 m^2 * 0.37 ~ 1905 N (pre-induction).
    """
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T = _thrust(AeroClass(ds_rotor).compute_forces(**DS_DESIGN_KWARGS), disk_n)
    assert 100.0 <= T <= 10000.0, \
        f"{AeroClass.__name__} [DS2018]: T={T:.1f} N out of [100, 10000] N"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_ds_thrust_increases_with_collective(AeroClass, ds_rotor):
    """Higher collective must produce more thrust on the De Schutter rotor."""
    model  = AeroClass(ds_rotor)
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T_lo   = _thrust(model.compute_forces(**{**DS_DESIGN_KWARGS, "collective_rad": 0.00}), disk_n)
    T_hi   = _thrust(model.compute_forces(**{**DS_DESIGN_KWARGS, "collective_rad": 0.10}), disk_n)
    assert T_hi > T_lo, \
        f"{AeroClass.__name__} [DS2018]: T did not increase: {T_lo:.1f} -> {T_hi:.1f}"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_ds_CT_in_physical_range(AeroClass, ds_rotor):
    """CT = T / (rho * A_K * (Omega*R)^2) must be in [0.0001, 0.5] at DS conditions."""
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T      = _thrust(AeroClass(ds_rotor).compute_forces(**DS_DESIGN_KWARGS), disk_n)

    A_K    = math.pi * (ds_rotor.radius_m ** 2 - ds_rotor.root_cutout_m ** 2)
    tip_v2 = (DS_OMEGA * ds_rotor.radius_m) ** 2
    CT     = T / max(ds_rotor.rho_kg_m3 * A_K * tip_v2, 1e-6)

    assert 0.0001 <= CT <= 0.5, \
        f"{AeroClass.__name__} [DS2018]: CT={CT:.6f} out of [0.0001, 0.5]"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_ds_H_force_positive_edgewise(AeroClass, ds_rotor):
    """
    Horizontal disk + East wind + small collective -> H-force (Fy, NED East) > 0.
    collective=0.05 rad avoids the zero-thrust degenerate case at zero axial flow
    and zero collective for models with CL0=0 (thin airfoil De Schutter rotor).
    """
    forces = AeroClass(ds_rotor).compute_forces(
        **{**DS_EDGEWISE_KWARGS, "collective_rad": 0.05})
    assert forces[1] > 0, \
        f"{AeroClass.__name__} [DS2018 edgewise]: Fy={forces[1]:.1f} N <= 0"


# ═══════════════════════════════════════════════════════════════════════════════
# § Cross-model spread
# ═══════════════════════════════════════════════════════════════════════════════

def test_thrust_spread_within_decade():
    """
    All models must produce thrust within one order of magnitude of each other.
    A 10x spread indicates a fundamental modelling error.
    """
    rotor  = rd.default()
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    thrusts = {cls.__name__: _thrust(cls(rotor).compute_forces(**DESIGN_KWARGS), disk_n)
               for cls in ALL_MODELS}

    T_max = max(thrusts.values())
    T_min = min(thrusts.values())
    assert T_max < 10 * T_min, \
        (f"Thrust spread exceeds 10x: min={T_min:.1f} N, max={T_max:.1f} N, "
         f"values={thrusts}")


def test_h_force_spread_within_decade():
    """In-plane H-force magnitude at the design point must be within one order of magnitude across all models."""
    rotor  = rd.default()
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)

    def h_mag(cls):
        f = cls(rotor).compute_forces(**DESIGN_KWARGS)
        F = np.asarray(f[:3])
        return float(np.linalg.norm(F - np.dot(F, disk_n) * disk_n))

    h_forces = {cls.__name__: h_mag(cls) for cls in ALL_MODELS}

    H_max = max(h_forces.values())
    H_min = min(h_forces.values())
    assert H_max < 10 * max(H_min, 1.0), \
        f"H-force spread exceeds 10x: min={H_min:.1f} N, max={H_max:.1f} N, values={h_forces}"
