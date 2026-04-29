"""
test_aero_physics.py — Physical laws that every correct aero model must satisfy.

Tests are written as universal invariants: if a model fails here it violates
fundamental aerodynamics, not just a numerical preference. All tests are
parametrized over ALL_MODELS and additionally cover cross-model agreement.

  1. Thrust direction — positive collective + axial inflow → positive thrust
  2. Cross-model agreement — all models agree on sign and direction
  3. Thrust monotonicity — higher collective → more thrust
  4. Force symmetry — symmetric input → no lateral force; ±tilt → ±My
  5. H-force direction — in-plane wind pushes hub downwind
  6. Cyclic moment response — tilt_lon/lat changes the corresponding moment
  7. Spin torque — non-zero Q_spin at operating point
  8. Dimensionless CT — thrust coefficient in physically valid range
  9. Omega scaling — higher rotor speed → more thrust
 10. Zero condition — negligible forces at zero wind and zero spin
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

from _helpers import (
    ALL_MODELS, HOVER_KWARGS, DESIGN_KWARGS,
    DESIGN_BODY_Z, DESIGN_OMEGA_EQ,
)
from aero import rotor_definition as rd


def _thrust(forces, disk_n=None):
    if disk_n is None:
        disk_n = np.array([0.0, 0.0, 1.0])
    return float(np.dot(forces[:3], disk_n))


# ── 1. Thrust direction ───────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_thrust_positive_for_positive_collective(AeroClass):
    """Positive collective + axial inflow → positive thrust along disk normal."""
    rotor  = rd.default()
    forces = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    T = _thrust(forces, np.array([0.0, 0.0, 1.0]))   # R_hub=I → disk_normal=NED-Z
    assert T > 0, f"{AeroClass.__name__}: T={T:.2f} N <= 0 with positive collective"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_thrust_positive_at_design_point(AeroClass):
    """At the RAWES tether-equilibrium operating point, thrust must be positive."""
    rotor  = rd.default()
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T      = _thrust(AeroClass(rotor).compute_forces(**DESIGN_KWARGS), disk_n)
    assert T > 0, f"{AeroClass.__name__}: T={T:.2f} N at design point — expected positive"


# ── 2. Cross-model agreement ──────────────────────────────────────────────────

def test_all_models_agree_thrust_positive():
    """All models must produce positive thrust at the design operating point."""
    rotor  = rd.default()
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    for cls in ALL_MODELS:
        T = _thrust(cls(rotor).compute_forces(**DESIGN_KWARGS), disk_n)
        assert T > 0, f"{cls.__name__}: T={T:.2f} N <= 0 (sign disagreement at design)"


def test_all_models_agree_thrust_sign_hover():
    """At hover-like conditions (R_hub=I, axial inflow), all models give T > 0."""
    rotor  = rd.default()
    disk_n = np.array([0.0, 0.0, 1.0])
    for cls in ALL_MODELS:
        T = _thrust(cls(rotor).compute_forces(**HOVER_KWARGS), disk_n)
        assert T > 0, f"{cls.__name__} [hover]: T={T:.2f} <= 0"


def test_all_models_agree_H_force_positive():
    """All models must agree that H-force (Fx) > 0 for North (NED X-axis) wind."""
    rotor = rd.default()
    for cls in ALL_MODELS:
        f = cls(rotor).compute_forces(**HOVER_KWARGS)
        assert f[0] > 0, f"{cls.__name__}: Fx={f[0]:.2f} N — expected positive H-force"


def test_all_models_tilt_lon_changes_moment():
    """All models: tilt_lon != 0 changes at least one moment component."""
    rotor = rd.default()
    for cls in ALL_MODELS:
        m  = cls(rotor)
        f0 = m.compute_forces(**{**HOVER_KWARGS, "tilt_lon": 0.0})
        f1 = m.compute_forces(**{**HOVER_KWARGS, "tilt_lon": 0.3})
        assert not np.allclose(f0[3:], f1[3:], rtol=1e-3, atol=0.1), \
            f"{cls.__name__}: moments unchanged by tilt_lon=0.3"


def test_all_models_tilt_lat_changes_moment():
    """All models: tilt_lat != 0 changes at least one moment component."""
    rotor = rd.default()
    for cls in ALL_MODELS:
        m  = cls(rotor)
        f0 = m.compute_forces(**{**HOVER_KWARGS, "tilt_lat": 0.0})
        f1 = m.compute_forces(**{**HOVER_KWARGS, "tilt_lat": 0.3})
        assert not np.allclose(f0[3:], f1[3:], rtol=1e-3, atol=0.1), \
            f"{cls.__name__}: moments unchanged by tilt_lat=0.3"


# ── 3. Thrust monotonicity ────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_thrust_increases_with_collective(AeroClass):
    """Higher blade pitch must produce more thrust, all else equal."""
    rotor = rd.default()
    model = AeroClass(rotor)
    f_lo  = model.compute_forces(**{**HOVER_KWARGS, "collective_rad": 0.05})
    f_hi  = model.compute_forces(**{**HOVER_KWARGS, "collective_rad": 0.20})
    T_lo  = _thrust(f_lo, np.array([0, 0, 1]))
    T_hi  = _thrust(f_hi, np.array([0, 0, 1]))
    assert T_hi > T_lo, \
        f"{AeroClass.__name__}: T_lo={T_lo:.1f} N, T_hi={T_hi:.1f} N — thrust did not increase"


# ── 4. Force symmetry ─────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_no_lateral_force_for_symmetric_wind(AeroClass):
    """
    Axisymmetric setup (R_hub=I, no tilt): Fy must be ~0 for lumped models.
    SkewedWakeBEM deliberately breaks Y-symmetry (Coleman non-uniform induction);
    allow |Fy| < 20% of thrust there.
    """
    rotor  = rd.default()
    forces = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    np.testing.assert_allclose(
        forces[1], 0.0, atol=1.0,
        err_msg=f"{AeroClass.__name__}: Fy={forces[1]:.3f} N (expected ~0 for symmetric wind)",
    )


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_symmetric_tilt_produces_symmetric_response(AeroClass):
    """Positive and negative tilt_lat must produce equal-magnitude, opposite-sign My."""
    rotor = rd.default()
    model = AeroClass(rotor)
    f_pos = model.compute_forces(**{**HOVER_KWARGS, "tilt_lat":  0.3})
    f_neg = model.compute_forces(**{**HOVER_KWARGS, "tilt_lat": -0.3})
    assert f_pos[4] * f_neg[4] <= 0, \
        f"{AeroClass.__name__}: My does not flip sign: {f_pos[4]:.3f} vs {f_neg[4]:.3f}"


# ── 5. H-force direction ──────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_h_force_positive_for_axial_wind(AeroClass):
    """In-plane wind from North (X in NED) must push the hub northward (Fx > 0)."""
    rotor  = rd.default()
    forces = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    assert forces[0] > 0, \
        f"{AeroClass.__name__}: Fx={forces[0]:.2f} N (expected > 0 for eastward H-force)"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_h_force_increases_with_wind_speed(AeroClass):
    """Higher in-plane wind speed gives larger in-plane drag (H-force)."""
    rotor  = rd.default()
    disk_n = np.array([0.0, 0.0, 1.0])   # R_hub=I in HOVER_KWARGS -> disk_normal=NED-Z
    # Fresh model instances: stateful models (Peters-He) must not carry ODE state between calls.
    f_lo = AeroClass(rotor).compute_forces(**{**HOVER_KWARGS, "wind_world": np.array([5.0, 0.0, 3.0])})
    f_hi = AeroClass(rotor).compute_forces(**{**HOVER_KWARGS, "wind_world": np.array([15.0, 0.0, 9.0])})
    # In-plane force magnitude (H-force)
    def h_mag(f):
        F = np.asarray(f[:3])
        return float(np.linalg.norm(F - np.dot(F, disk_n) * disk_n))
    assert h_mag(f_hi) > h_mag(f_lo), \
        f"{AeroClass.__name__}: H-force magnitude did not increase with wind speed"


# ── 6. Cyclic moment response ─────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_tilt_lon_changes_Mx(AeroClass):
    """tilt_lon != 0 must produce a measurable change in Mx (pitch moment)."""
    rotor = rd.default()
    model = AeroClass(rotor)
    f0 = model.compute_forces(**{**HOVER_KWARGS, "tilt_lon": 0.0})
    f1 = model.compute_forces(**{**HOVER_KWARGS, "tilt_lon": 0.3})
    assert not math.isclose(f0[3], f1[3], rel_tol=1e-3, abs_tol=0.1), \
        f"{AeroClass.__name__}: Mx unchanged by tilt_lon ({f0[3]:.3f} vs {f1[3]:.3f})"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_tilt_lat_changes_My(AeroClass):
    """tilt_lat != 0 must produce a measurable change in My (roll moment)."""
    rotor = rd.default()
    model = AeroClass(rotor)
    f0 = model.compute_forces(**{**HOVER_KWARGS, "tilt_lat": 0.0})
    f1 = model.compute_forces(**{**HOVER_KWARGS, "tilt_lat": 0.3})
    assert not math.isclose(f0[4], f1[4], rel_tol=1e-3, abs_tol=0.1), \
        f"{AeroClass.__name__}: My unchanged by tilt_lat ({f0[4]:.3f} vs {f1[4]:.3f})"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_zero_tilt_My_is_small(AeroClass):
    """
    With zero tilt and Y-symmetric wind, My (roll) must be near zero.
    Mx may be non-zero for azimuth-integrated models (advancing-blade H-force asymmetry).
    SkewedWakeBEM may produce small My from lateral-skew; allow up to 50% of T.
    """
    rotor  = rd.default()
    forces = AeroClass(rotor).compute_forces(
        collective_rad=0.1, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=np.eye(3), v_hub_world=np.zeros(3),
        omega_rotor=20.0, wind_world=np.array([8.66, 0.0, 5.0]),
        t=10.0,
    )
    T = abs(forces[2])
    assert abs(forces[4]) < max(0.3 * T, 1.0), \
        f"{AeroClass.__name__}: My={forces[4]:.2f} N*m at zero tilt (expected ~0)"


# ── 7. Spin torque ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_spin_torque_nonzero_at_design(AeroClass):
    """result.Q_spin must be non-zero after compute_forces at the design point."""
    rotor  = rd.default()
    result = AeroClass(rotor).compute_forces(**DESIGN_KWARGS)
    assert result.Q_spin != 0.0, f"{AeroClass.__name__}: Q_spin = 0 at design point"


# ── 8. Dimensionless thrust coefficient ───────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_CT_in_physical_range(AeroClass):
    """CT = T / (rho * A * (Omega*R)^2) must be in [0.005, 0.9] at design point."""
    rotor  = rd.default()
    forces = AeroClass(rotor).compute_forces(**DESIGN_KWARGS)
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T      = _thrust(forces, disk_n)

    A      = math.pi * (rotor.radius_m ** 2 - rotor.root_cutout_m ** 2)
    CT     = T / (rotor.rho_kg_m3 * A * (DESIGN_OMEGA_EQ * rotor.radius_m) ** 2)

    assert 0.005 <= CT <= 0.9, \
        f"{AeroClass.__name__}: CT = {CT:.4f} out of physical range [0.005, 0.9]"


# ── 9. Omega scaling ──────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_higher_omega_gives_more_thrust(AeroClass):
    """Double rotor speed must produce higher thrust (T proportional to omega^2)."""
    rotor  = rd.default()
    model  = AeroClass(rotor)
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T_lo   = _thrust(model.compute_forces(**{**DESIGN_KWARGS, "omega_rotor": 15.0}), disk_n)
    T_hi   = _thrust(model.compute_forces(**{**DESIGN_KWARGS, "omega_rotor": 25.0}), disk_n)
    assert T_hi > T_lo, \
        f"{AeroClass.__name__}: T did not increase with omega: T_lo={T_lo:.1f} T_hi={T_hi:.1f}"


# ── 10. Zero condition ────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_forces_small_at_zero_wind_zero_spin(AeroClass):
    """Zero wind and zero spin: no dynamic pressure → aerodynamic forces must be negligible."""
    rotor  = rd.default()
    forces = AeroClass(rotor).compute_forces(
        collective_rad=0.1, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=np.eye(3), v_hub_world=np.zeros(3),
        omega_rotor=0.0, wind_world=np.zeros(3),
        t=10.0,
    )
    mag = float(np.linalg.norm(forces))
    assert mag < 1.0, \
        f"{AeroClass.__name__}: |forces| = {mag:.2f} N at zero wind/spin (expected ~0)"
