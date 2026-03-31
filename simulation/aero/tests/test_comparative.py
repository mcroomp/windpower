"""
test_comparative.py — Cross-model comparison tests.

These tests compare all four models against each other to verify:
  1. Agreement on fundamental outputs (signs, directions, order of magnitude)
  2. Documented differences between model families
  3. Model-specific features that should differ by design

Model taxonomy
--------------
Lumped-BEM (no azimuth integration):
  GlauertStateBEM — strip BEM + Glauert inflow-state corrections

Per-blade azimuth-integrated (natural cyclic + H-force):
  DeSchutterAero  — vectorised per-blade strip theory
  PrandtlBEM      — DeSchutterAero + Prandtl tip/root loss
  SkewedWakeBEM   — DeSchutterAero + Prandtl + Coleman skewed wake (production)

Expected ordering (at design point, positive collective):
  Tip-loss models produce less thrust: PrandtlBEM ≤ DeSchutterAero  (Prandtl lowers tip)
  Skewed wake varies thrust distribution but not necessarily total
  Glauert correction can raise or lower thrust depending on regime
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
    ALL_MODELS, HOVER_KWARGS, DESIGN_KWARGS, EDGEWISE_KWARGS,
    DESIGN_BODY_Z, DESIGN_OMEGA_EQ,
)
import rotor_definition as rd
from aero import DeSchutterAero
from aero_prandtl_bem    import PrandtlBEM
from aero_skewed_wake    import SkewedWakeBEM
from aero_glauert_states import GlauertStateBEM


def _thrust(forces, disk_n=None):
    if disk_n is None:
        disk_n = np.array([0.0, 0.0, 1.0])
    return float(np.dot(forces[:3], disk_n))


# ────────────────────────────────────────────────────────────────────────────
# 1. All models agree on thrust SIGN
# ────────────────────────────────────────────────────────────────────────────

def test_all_models_agree_thrust_positive():
    """All five models must produce positive thrust at the design operating point."""
    rotor  = rd.default()
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    for cls in ALL_MODELS:
        m = cls(rotor)
        f = m.compute_forces(**DESIGN_KWARGS)
        T = _thrust(f, disk_n)
        assert T > 0, f"{cls.__name__}: T={T:.2f} N ≤ 0 (sign disagreement)"


def test_all_models_agree_thrust_sign_hover():
    """At hover-like conditions (R_hub=I, v_axial=5 m/s), all give T > 0."""
    rotor = rd.default()
    for cls in ALL_MODELS:
        m = cls(rotor)
        f = m.compute_forces(**HOVER_KWARGS)
        T = _thrust(f, disk_n=np.array([0, 0, 1]))
        assert T > 0, f"{cls.__name__} [hover]: T={T:.2f} ≤ 0"


def test_all_models_agree_H_force_positive():
    """All five models must agree that H-force (Fx) > 0 for +X wind."""
    rotor = rd.default()
    for cls in ALL_MODELS:
        m = cls(rotor)
        f = m.compute_forces(**HOVER_KWARGS)
        assert f[0] > 0, f"{cls.__name__}: Fx={f[0]:.2f} N — expected positive H-force"


# ────────────────────────────────────────────────────────────────────────────
# 2. All models agree: tilt changes moments
# ────────────────────────────────────────────────────────────────────────────

def test_all_models_tilt_lon_changes_moment():
    """All models: tilt_lon ≠ 0 changes at least one moment component."""
    rotor = rd.default()
    for cls in ALL_MODELS:
        m  = cls(rotor)
        f0 = m.compute_forces(**{**HOVER_KWARGS, "tilt_lon": 0.0})
        f1 = m.compute_forces(**{**HOVER_KWARGS, "tilt_lon": 0.3})
        # At least one moment component must change
        moment_changed = not np.allclose(f0[3:], f1[3:], rtol=1e-3, atol=0.1)
        assert moment_changed, \
            f"{cls.__name__}: moments unchanged by tilt_lon=0.3 ({f0[3:]!r} vs {f1[3:]!r})"


def test_all_models_tilt_lat_changes_moment():
    """All models: tilt_lat ≠ 0 changes at least one moment component."""
    rotor = rd.default()
    for cls in ALL_MODELS:
        m  = cls(rotor)
        f0 = m.compute_forces(**{**HOVER_KWARGS, "tilt_lat": 0.0})
        f1 = m.compute_forces(**{**HOVER_KWARGS, "tilt_lat": 0.3})
        moment_changed = not np.allclose(f0[3:], f1[3:], rtol=1e-3, atol=0.1)
        assert moment_changed, \
            f"{cls.__name__}: moments unchanged by tilt_lat=0.3 ({f0[3:]!r} vs {f1[3:]!r})"


# ────────────────────────────────────────────────────────────────────────────
# 3. Prandtl vs DeSchutter: tip-loss should reduce thrust
# ────────────────────────────────────────────────────────────────────────────

def test_prandtl_thrust_leq_deschutter():
    """
    Prandtl tip-loss reduces thrust vs no-tip-loss BEM.
    PrandtlBEM adds F_tip·F_root ≤ 1 to DeSchutterAero's strip forces.
    At the same operating point, T_prandtl ≤ T_deschutter.

    This is a fundamental property: tip-loss always reduces integrated thrust.
    Allow a tolerance of ≤ 5% for numerical differences in strip integration.
    """
    rotor   = rd.default()
    m_ds    = DeSchutterAero(rotor)
    m_pr    = PrandtlBEM(rotor)

    disk_n  = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    f_ds    = m_ds.compute_forces(**DESIGN_KWARGS)
    f_pr    = m_pr.compute_forces(**DESIGN_KWARGS)

    T_ds = _thrust(f_ds, disk_n)
    T_pr = _thrust(f_pr, disk_n)

    # Prandtl should give ≤ DS thrust (with some tolerance for different azimuth count)
    assert T_pr <= T_ds * 1.05, \
        (f"PrandtlBEM T={T_pr:.1f} N > DeSchutterAero T={T_ds:.1f} N × 1.05 = {T_ds*1.05:.1f} N — "
         "tip-loss should not increase thrust significantly")


def test_prandtl_F_mean_below_one():
    """PrandtlBEM: mean F factor must be < 1.0 at any reasonable operating point."""
    rotor = rd.default()
    m_pr  = PrandtlBEM(rotor)
    m_pr.compute_forces(**DESIGN_KWARGS)
    assert m_pr.last_F_prandtl < 1.0, \
        f"PrandtlBEM: F_prandtl = {m_pr.last_F_prandtl:.4f} — tip-loss factor should be < 1"


# ────────────────────────────────────────────────────────────────────────────
# 4. Skewed wake: skew angle increases with advance ratio
# ────────────────────────────────────────────────────────────────────────────

def test_skewed_wake_angle_increases_with_tilt():
    """
    For a larger v_inplane/v_axial ratio, the wake skew angle χ must increase.

    Comparison:
      DESIGN_KWARGS:  v_axial=8.51, v_inplane=5.25  → χ ≈ 22°
      HOVER_KWARGS:   R_hub=I, wind=[8.66,0,5]  → v_axial=5, v_inplane=8.66 → χ ≈ 53°
    """
    rotor = rd.default()
    m     = SkewedWakeBEM(rotor)

    # Design orientation: v_inplane/v_axial ≈ 0.62 → moderate χ
    m.compute_forces(**DESIGN_KWARGS)
    chi_mild = m.last_skew_angle_deg

    # Hover-like: R_hub=I, wind=[8.66,0,5] → v_axial=5, v_inplane=8.66 → larger ratio
    m.compute_forces(**HOVER_KWARGS)
    chi_high = m.last_skew_angle_deg

    assert chi_high > chi_mild, \
        (f"SkewedWakeBEM: χ did not increase: "
         f"design {chi_mild:.1f}° vs hover {chi_high:.1f}° — "
         "hover has larger v_inplane/v_axial → should have larger skew")


def test_skewed_wake_K_matches_chi():
    """
    Coleman K = tan(χ/2) must be consistent with the computed skew angle.
    """
    rotor = rd.default()
    m     = SkewedWakeBEM(rotor)
    m.compute_forces(**DESIGN_KWARGS)
    chi_rad = math.radians(m.last_skew_angle_deg)
    K_expected = math.tan(chi_rad / 2.0)
    assert math.isclose(m.last_K_skew, K_expected, rel_tol=0.01), \
        f"K={m.last_K_skew:.4f} vs tan(χ/2)={K_expected:.4f} — inconsistent"


# ────────────────────────────────────────────────────────────────────────────
# 5. Glauert state: windmill regime at RAWES design point
# ────────────────────────────────────────────────────────────────────────────

def test_glauert_windmill_regime_at_design():
    """
    GlauertStateBEM must detect windmill or turbulent state (not vortex ring)
    at the RAWES design point (v_axial > 0, extracting energy from wind).
    """
    rotor = rd.default()
    m     = GlauertStateBEM(rotor)
    m.compute_forces(**DESIGN_KWARGS)
    assert m.last_state in ("windmill", "turbulent"), \
        f"GlauertStateBEM: state={m.last_state!r} at design point — expected windmill/turbulent"


def test_glauert_vortex_ring_at_zero_axial():
    """
    With zero axial flow (horizontal disk, horizontal wind → v_axial=0),
    GlauertStateBEM should detect a degenerate case (near-zero induction).
    """
    rotor = rd.default()
    m     = GlauertStateBEM(rotor)
    # Purely in-plane wind (v_axial=0)
    m.compute_forces(
        collective_rad=0.1, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=np.eye(3), v_hub_world=np.zeros(3),
        omega_rotor=20.0, wind_world=np.array([0.0, 10.0, 0.0]),   # NED: East wind
        t=10.0,
    )
    # At v_axial=0 the induction should be near zero (can't drive momentum)
    assert m.last_a_mean < 0.3, \
        f"GlauertStateBEM: a_mean={m.last_a_mean:.3f} at zero axial flow — expected small"


def test_glauert_induction_factor_positive():
    """
    At the RAWES design point, induction must be positive
    (wind is being slowed by the rotor — energy extracted).
    """
    rotor = rd.default()
    m     = GlauertStateBEM(rotor)
    m.compute_forces(**DESIGN_KWARGS)
    assert m.last_a_mean >= 0, \
        f"GlauertStateBEM: a_mean={m.last_a_mean:.4f} < 0 at design point"


# ────────────────────────────────────────────────────────────────────────────
# 6. All models within 10× of each other on thrust
# ────────────────────────────────────────────────────────────────────────────

def test_thrust_spread_within_decade():
    """
    All five models should produce thrust within one order of magnitude of
    each other at the same operating point.

    A factor-of-10 spread would indicate a fundamental modelling error rather
    than a reasonable difference between approximation levels.
    """
    rotor  = rd.default()
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    thrusts = {}
    for cls in ALL_MODELS:
        m = cls(rotor)
        f = m.compute_forces(**DESIGN_KWARGS)
        thrusts[cls.__name__] = _thrust(f, disk_n)

    T_max = max(thrusts.values())
    T_min = min(thrusts.values())

    assert T_max < 10 * T_min, \
        (f"Thrust spread exceeds 10×: min={T_min:.1f} N ({min(thrusts, key=thrusts.get)}), "
         f"max={T_max:.1f} N ({max(thrusts, key=thrusts.get)}) — "
         f"all values: {thrusts}")


# ────────────────────────────────────────────────────────────────────────────
# 7. H-force comparison
# ────────────────────────────────────────────────────────────────────────────

def test_h_force_spread_within_decade():
    """
    H-force (in-plane drag) should also be within one order of magnitude
    across all models.  Edgewise flight is the H-force-dominated case.
    """
    rotor   = rd.default()
    h_forces = {}
    for cls in ALL_MODELS:
        m = cls(rotor)
        f = m.compute_forces(**HOVER_KWARGS)
        h_forces[cls.__name__] = float(f[0])  # Fx, direction of in-plane wind

    H_max = max(h_forces.values())
    H_min = min(v for v in h_forces.values() if v > 0)  # only positive values

    assert H_max < 10 * H_min, \
        (f"H-force spread exceeds 10×: min={H_min:.1f} N, max={H_max:.1f} N — "
         f"values: {h_forces}")


# ────────────────────────────────────────────────────────────────────────────
# 8. Spin torque polarity consistent across models
# ────────────────────────────────────────────────────────────────────────────

def test_all_models_Q_spin_positive_at_low_omega():
    """
    Below equilibrium omega (ω=5 rad/s), Q_spin should be positive for ALL models.
    This is the autorotation drive: wind energy flows into the spinning rotor.
    """
    rotor  = rd.default()
    kwargs = {**DESIGN_KWARGS, "omega_rotor": 5.0}
    for cls in ALL_MODELS:
        m = cls(rotor)
        m.compute_forces(**kwargs)
        assert m.last_Q_spin > 0, \
            f"{cls.__name__}: Q_spin={m.last_Q_spin:.2f} N·m at ω=5 (expected > 0)"


def test_all_models_Q_spin_negative_at_high_omega():
    """
    Above equilibrium omega (ω=40 rad/s), Q_spin should be negative for ALL models.
    This is the braking regime: aerodynamic drag exceeds drive torque.
    """
    rotor  = rd.default()
    kwargs = {**DESIGN_KWARGS, "omega_rotor": 40.0}
    for cls in ALL_MODELS:
        m = cls(rotor)
        m.compute_forces(**kwargs)
        assert m.last_Q_spin < 0, \
            f"{cls.__name__}: Q_spin={m.last_Q_spin:.2f} N·m at ω=40 (expected < 0)"


# ────────────────────────────────────────────────────────────────────────────
# 9. Numerical stability: extreme inputs
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_stability_high_omega(AeroClass):
    """All models return finite forces at ω=100 rad/s (5× normal)."""
    rotor  = rd.default()
    model  = AeroClass(rotor)
    forces = model.compute_forces(**{**DESIGN_KWARGS, "omega_rotor": 100.0})
    assert np.all(np.isfinite(forces)), \
        f"{AeroClass.__name__}: non-finite forces at ω=100: {forces}"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_stability_very_high_wind(AeroClass):
    """All models return finite forces at V=50 m/s wind."""
    rotor  = rd.default()
    model  = AeroClass(rotor)
    forces = model.compute_forces(**{**DESIGN_KWARGS,
                                     "wind_world": np.array([40.0, 0.0, 20.0])})
    assert np.all(np.isfinite(forces)), \
        f"{AeroClass.__name__}: non-finite forces at V=50 m/s: {forces}"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_stability_hub_moving(AeroClass):
    """All models return finite forces when hub is moving at 10 m/s."""
    rotor  = rd.default()
    model  = AeroClass(rotor)
    forces = model.compute_forces(**{**DESIGN_KWARGS,
                                     "v_hub_world": np.array([5.0, 0.0, 3.0])})
    assert np.all(np.isfinite(forces)), \
        f"{AeroClass.__name__}: non-finite forces with hub motion: {forces}"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_stability_large_collective(AeroClass):
    """All models handle max collective (±0.35 rad = ±20°) without NaN."""
    rotor  = rd.default()
    model  = AeroClass(rotor)
    for coll in [-0.35, 0.35]:
        forces = model.compute_forces(**{**DESIGN_KWARGS, "collective_rad": coll})
        assert np.all(np.isfinite(forces)), \
            f"{AeroClass.__name__}: non-finite forces at collective={coll:.2f}: {forces}"
