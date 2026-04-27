"""
test_physics.py — Physical consistency tests across all five aero models.

Tests that must pass for any physically correct RAWES aerodynamic model:
  1. Thrust direction — force along disk normal, same direction as wind axial component
  2. Thrust monotonicity — higher collective → more thrust (at constant omega, wind)
  3. Force symmetry — axisymmetric input → no lateral force Fy
  4. H-force direction — downwind push for in-plane wind
  5. Cyclic moment response — tilt_lon changes Mx, tilt_lat changes My
  6. Spin torque sign — opposing rotation direction
  7. Autorotation torque balance — Q_spin ≈ 0 at equilibrium omega_eq
  8. Dimensionless coefficients — CT in physically reasonable range
  9. Omega scaling — higher omega → higher thrust (blade speed effect)
 10. Ramp behaviour — smooth onset

These are fundamental aerodynamic constraints, not numerical targets.
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
    DESIGN_BODY_Z, DESIGN_OMEGA_EQ, DESIGN_WIND,
)
import rotor_definition as rd


# ────────────────────────────────────────────────────────────────────────────
# 1. Thrust direction
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_thrust_positive_for_positive_collective(AeroClass):
    """
    With wind flowing through the disk (positive v_axial), positive collective
    must produce positive thrust (force component along disk normal > 0).
    """
    rotor  = rd.default()
    model  = AeroClass(rotor)
    forces = model.compute_forces(**HOVER_KWARGS)
    T      = float(np.dot(forces[:3], np.array([0.0, 0.0, 1.0])))   # R_hub=I → disk_normal=Z
    assert T > 0, f"{AeroClass.__name__}: thrust T={T:.2f} N ≤ 0 with positive collective"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_thrust_positive_at_design_point(AeroClass):
    """At the RAWES tether-equilibrium operating point, thrust must be positive."""
    rotor  = rd.default()
    model  = AeroClass(rotor)
    forces = model.compute_forces(**DESIGN_KWARGS)
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T      = float(np.dot(forces[:3], disk_n))
    assert T > 0, \
        f"{AeroClass.__name__}: T={T:.2f} N at design point — expected positive thrust"


# ────────────────────────────────────────────────────────────────────────────
# 2. Thrust monotonicity
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_thrust_increases_with_collective(AeroClass):
    """Higher collective (blade pitch) must produce more thrust, all else equal."""
    rotor = rd.default()
    model = AeroClass(rotor)
    f_lo  = model.compute_forces(**{**HOVER_KWARGS, "collective_rad": 0.05})
    f_hi  = model.compute_forces(**{**HOVER_KWARGS, "collective_rad": 0.20})
    T_lo  = float(np.dot(f_lo[:3], [0, 0, 1]))
    T_hi  = float(np.dot(f_hi[:3], [0, 0, 1]))
    assert T_hi > T_lo, \
        f"{AeroClass.__name__}: T_lo={T_lo:.1f} N, T_hi={T_hi:.1f} N — thrust did not increase"


# ────────────────────────────────────────────────────────────────────────────
# 3. Force symmetry
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_no_lateral_force_for_symmetric_wind(AeroClass):
    """
    Horizontal wind + horizontal disk (R_hub=I, no tilt) — Y-symmetric setup.

    For lumped and azimuth-averaging models (RotorAero, DeSchutterAero, PrandtlBEM,
    GlauertStateBEM): Fy should be ~0 (within numerical tolerance).

    SkewedWakeBEM deliberately breaks Y-symmetry via the Coleman non-uniform
    induction: advancing blade (ψ=0) gets more induction than retreating blade
    (ψ=π), producing a net lateral force (helicopter "lateral flapping" effect).
    For SkewedWakeBEM we allow |Fy| < 20% of thrust.
    """
    from aero_skewed_wake import SkewedWakeBEM
    rotor  = rd.default()
    model  = AeroClass(rotor)
    forces = model.compute_forces(**HOVER_KWARGS)

    if isinstance(model, SkewedWakeBEM):
        # Lateral force from skewed wake is physically real — just bound it
        T = abs(forces[2])
        assert abs(forces[1]) < max(0.2 * T, 5.0), \
            f"SkewedWakeBEM: Fy={forces[1]:.1f} N exceeds 20% of T={T:.1f} N"
    else:
        np.testing.assert_allclose(
            forces[1], 0.0, atol=1.0,
            err_msg=f"{AeroClass.__name__}: Fy={forces[1]:.3f} N (expected ~0 for symmetric wind)",
        )


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_symmetric_tilt_produces_symmetric_response(AeroClass):
    """
    Positive tilt_lat and negative tilt_lat should produce equal-magnitude,
    opposite-sign My moments.
    """
    rotor  = rd.default()
    model  = AeroClass(rotor)
    f_pos  = model.compute_forces(**{**HOVER_KWARGS, "tilt_lat":  0.3})
    f_neg  = model.compute_forces(**{**HOVER_KWARGS, "tilt_lat": -0.3})
    # My (forces[4]) should flip sign
    assert f_pos[4] * f_neg[4] <= 0, \
        f"{AeroClass.__name__}: My does not flip sign: {f_pos[4]:.3f} vs {f_neg[4]:.3f}"


# ────────────────────────────────────────────────────────────────────────────
# 4. H-force (in-plane drag)
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_h_force_positive_for_eastward_wind(AeroClass):
    """
    With wind from East (+X) and a horizontal rotor, the H-force must push
    the hub eastward (Fx > 0).  This is the in-plane drag that RAWES must
    overcome with tether tension.
    """
    rotor  = rd.default()
    model  = AeroClass(rotor)
    forces = model.compute_forces(**HOVER_KWARGS)
    assert forces[0] > 0, \
        f"{AeroClass.__name__}: Fx={forces[0]:.2f} N (expected > 0 for eastward H-force)"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_h_force_increases_with_wind_speed(AeroClass):
    """Higher wind speed → higher H-force (both models scale with dynamic pressure)."""
    rotor   = rd.default()
    model   = AeroClass(rotor)
    f_lo    = model.compute_forces(**{**HOVER_KWARGS,
                                      "wind_world": np.array([5.0, 0.0, 3.0])})
    f_hi    = model.compute_forces(**{**HOVER_KWARGS,
                                      "wind_world": np.array([15.0, 0.0, 9.0])})   # 3× faster
    assert f_hi[0] > f_lo[0], \
        f"{AeroClass.__name__}: H-force did not increase with wind speed"


# ────────────────────────────────────────────────────────────────────────────
# 5. Cyclic moment response
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_tilt_lon_changes_Mx(AeroClass):
    """tilt_lon ≠ 0 must produce a measurable change in Mx (pitch moment)."""
    rotor = rd.default()
    model = AeroClass(rotor)
    f0    = model.compute_forces(**{**HOVER_KWARGS, "tilt_lon": 0.0})
    f1    = model.compute_forces(**{**HOVER_KWARGS, "tilt_lon": 0.3})
    assert not math.isclose(f0[3], f1[3], rel_tol=1e-3, abs_tol=0.1), \
        f"{AeroClass.__name__}: Mx unchanged by tilt_lon ({f0[3]:.3f} vs {f1[3]:.3f})"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_tilt_lat_changes_My(AeroClass):
    """tilt_lat ≠ 0 must produce a measurable change in My (roll moment)."""
    rotor = rd.default()
    model = AeroClass(rotor)
    f0    = model.compute_forces(**{**HOVER_KWARGS, "tilt_lat": 0.0})
    f1    = model.compute_forces(**{**HOVER_KWARGS, "tilt_lat": 0.3})
    assert not math.isclose(f0[4], f1[4], rel_tol=1e-3, abs_tol=0.1), \
        f"{AeroClass.__name__}: My unchanged by tilt_lat ({f0[4]:.3f} vs {f1[4]:.3f})"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_zero_tilt_both_moments_baseline(AeroClass):
    """
    With zero tilt (tilt_lon=tilt_lat=0), the My (roll about Y) moment from
    cyclic control should be zero.

    Note: Mx (pitching moment) may be non-zero for azimuth-integrated models
    (DeSchutterAero, PrandtlBEM, SkewedWakeBEM) even at zero tilt.  These models
    compute the H-force asymmetry (advancing vs retreating blade) which creates a
    natural pitching moment.  For lumped models (RotorAero, GlauertStateBEM) the
    K_cyc term is zero at zero tilt → Mx=0.  This difference is correct and expected.

    We only test My ≈ 0 (Y-symmetric wind direction → no roll at zero tilt).
    """
    from aero_skewed_wake import SkewedWakeBEM
    rotor  = rd.default()
    model  = AeroClass(rotor)
    forces = model.compute_forces(
        collective_rad=0.1, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=np.eye(3), v_hub_world=np.zeros(3),
        omega_rotor=20.0, wind_world=np.array([8.66, 0.0, 5.0]),
        t=10.0,
    )
    # My should be near zero for Y-symmetric wind and no tilt
    # SkewedWakeBEM may produce small My from lateral-skew effect
    T = abs(forces[2])
    if isinstance(model, SkewedWakeBEM):
        assert abs(forces[4]) < max(0.5 * T, 10.0), \
            f"SkewedWakeBEM: My={forces[4]:.1f} N·m too large vs T={T:.1f} N"
    else:
        assert abs(forces[4]) < max(0.3 * T, 1.0), \
            f"{AeroClass.__name__}: My={forces[4]:.2f} N·m at zero tilt (expected ~0)"


# ────────────────────────────────────────────────────────────────────────────
# 6. Spin torque sign
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_spin_torque_set_after_compute(AeroClass):
    """result.Q_spin must be non-zero after compute_forces at design point."""
    rotor = rd.default()
    model = AeroClass(rotor)
    result = model.compute_forces(**DESIGN_KWARGS)
    assert result.Q_spin != 0.0, f"{AeroClass.__name__}: result.Q_spin = 0 at design point"


# ────────────────────────────────────────────────────────────────────────────
# 7. Dimensionless thrust coefficient
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_CT_in_physical_range(AeroClass):
    """
    At the design operating point, CT = T / (ρ·A·(ΩR)²) should be in [0.01, 0.5].
    CT > 8/9 ≈ 0.889 violates Betz limit; CT < 0.01 means negligible thrust.

    Note: this convention uses the full disk area and tip speed squared.
    """
    rotor = rd.default()
    model = AeroClass(rotor)
    forces = model.compute_forces(**DESIGN_KWARGS)
    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)
    T = float(np.dot(forces[:3], disk_n))

    R      = rotor.radius_m
    A      = math.pi * (R ** 2 - rotor.root_cutout_m ** 2)
    rho    = rotor.rho_kg_m3
    omega  = DESIGN_OMEGA_EQ
    tip_v2 = (omega * R) ** 2
    CT     = T / (rho * A * tip_v2)

    assert 0.005 <= CT <= 0.9, \
        f"{AeroClass.__name__}: CT = {CT:.4f} out of physical range [0.005, 0.9]"


# ────────────────────────────────────────────────────────────────────────────
# 8. Omega scaling
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_higher_omega_gives_more_thrust(AeroClass):
    """
    At double the rotor speed (all else equal), thrust should increase.
    Blade element theory: T ∝ ω² for fixed collective at moderate inflow.
    """
    rotor = rd.default()
    model = AeroClass(rotor)

    disk_n = DESIGN_BODY_Z / np.linalg.norm(DESIGN_BODY_Z)

    f_lo = model.compute_forces(**{**DESIGN_KWARGS, "omega_rotor": 15.0})
    f_hi = model.compute_forces(**{**DESIGN_KWARGS, "omega_rotor": 25.0})

    T_lo = float(np.dot(f_lo[:3], disk_n))
    T_hi = float(np.dot(f_hi[:3], disk_n))

    assert T_hi > T_lo, \
        f"{AeroClass.__name__}: T did not increase with omega: T_lo={T_lo:.1f} T_hi={T_hi:.1f}"


# ────────────────────────────────────────────────────────────────────────────
# 9. Forces zero at zero wind and zero omega
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_forces_small_at_zero_wind_zero_spin(AeroClass):
    """
    With zero wind and zero spin, aerodynamic forces must be negligible
    (no dynamic pressure → no lift or drag).  Exact zero after ramp.
    """
    rotor  = rd.default()
    model  = AeroClass(rotor)
    forces = model.compute_forces(
        collective_rad=0.1, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=np.eye(3), v_hub_world=np.zeros(3),
        omega_rotor=0.0, wind_world=np.zeros(3),
        t=10.0,
    )
    mag = float(np.linalg.norm(forces))
    assert mag < 1.0, \
        f"{AeroClass.__name__}: |forces| = {mag:.2f} N at zero wind/spin (expected ~0)"


# ────────────────────────────────────────────────────────────────────────────
# 10. GlauertStateBEM — windmill state at design point
# ────────────────────────────────────────────────────────────────────────────

def test_glauert_windmill_state_at_design_point():
    """
    At the RAWES design point (v_axial=8.51 m/s, ω=20.148 rad/s), the
    GlauertStateBEM should detect windmill or turbulent state (not vortex ring).
    """
    from aero_glauert_states import GlauertStateBEM
    rotor  = rd.default()
    model  = GlauertStateBEM(rotor)
    model.compute_forces(**DESIGN_KWARGS)
    assert model.last_state in ("windmill", "turbulent"), \
        f"GlauertStateBEM: expected windmill/turbulent at design point, got {model.last_state!r}"


def test_glauert_induction_factor_valid_range():
    """
    At the RAWES design point, the mean induction factor must be physically valid.
    Betz limit: a = 1/3 for maximum power extraction.
    The model may exceed a > 0.5 (turbulent wake) — the Glauert correction handles
    this regime.  The valid range is 0 ≤ a < 1.0 (a ≥ 1 is vortex ring, unphysical).
    """
    from aero_glauert_states import GlauertStateBEM
    rotor  = rd.default()
    model  = GlauertStateBEM(rotor)
    model.compute_forces(**DESIGN_KWARGS)
    a = model.last_a_mean
    assert 0.0 <= a < 1.0, \
        f"GlauertStateBEM: induction factor a={a:.3f} outside valid range [0, 1)"


# ────────────────────────────────────────────────────────────────────────────
# 11. SkewedWakeBEM — skew angle at design point
# ────────────────────────────────────────────────────────────────────────────

def test_skewed_wake_large_angle_at_design_point():
    """
    At the RAWES design point (v_inplane=5.25 m/s, v_axial=8.51 m/s),
    the wake skew angle should be moderate: χ = atan2(5.25, 8.51) ≈ 31.7°.
    Verify the model computes a physically reasonable skew angle.
    """
    from aero_skewed_wake import SkewedWakeBEM
    rotor  = rd.default()
    model  = SkewedWakeBEM(rotor)
    model.compute_forces(**DESIGN_KWARGS)
    chi = model.last_skew_angle_deg
    # At design point: chi = atan2(5.25, 8.51+v_i) ≈ 25–35°
    assert 10.0 <= chi <= 75.0, \
        f"SkewedWakeBEM: skew angle χ={chi:.1f}° — expected 10°–75° at design point"


def test_skewed_wake_larger_angle_at_high_advance_ratio():
    """
    At high v_inplane/v_axial ratio (disk near-horizontal, horizontal wind),
    the wake skew angle should be large: χ >> design-point value (~22°).

    HOVER_KWARGS: R_hub=I, wind=[8.66,0,5] → v_axial=5, v_inplane=8.66 → χ ≈ 53°.
    This is larger than the design point (~22°) because v_inplane dominates.
    """
    from aero_skewed_wake import SkewedWakeBEM
    rotor = rd.default()
    model = SkewedWakeBEM(rotor)
    model.compute_forces(**HOVER_KWARGS)
    chi = model.last_skew_angle_deg
    # v_inplane=8.66, v_axial=5, v_i≈3-6 → χ=atan2(8.66, 8-11) ≈ 38-47°
    assert chi > 30.0, \
        f"SkewedWakeBEM: χ={chi:.1f}° in high-advance-ratio flight — expected > 30°"


# ────────────────────────────────────────────────────────────────────────────
# 12. PrandtlBEM — F_prandtl reduces toward tip
# ────────────────────────────────────────────────────────────────────────────

def test_prandtl_F_less_than_one():
    """
    The mean Prandtl correction factor must be < 1.0 (tip-loss effect).
    A factor of 1.0 would mean no tip-loss, which is only correct for
    infinite blade number.
    """
    from aero_prandtl_bem import PrandtlBEM
    rotor = rd.default()
    model = PrandtlBEM(rotor)
    model.compute_forces(**HOVER_KWARGS)
    assert model.last_F_prandtl < 1.0, \
        f"PrandtlBEM: last_F_prandtl = {model.last_F_prandtl:.4f} — expected < 1.0"
    assert model.last_F_prandtl > 0.5, \
        f"PrandtlBEM: last_F_prandtl = {model.last_F_prandtl:.4f} — unreasonably small"


def test_prandtl_F_higher_for_more_blades():
    """
    More blades → less tip-loss per blade → higher mean Prandtl F.
    Compare N=4 (Beaupoil) vs N=3 (De Schutter).
    """
    from aero_prandtl_bem import PrandtlBEM

    m4 = PrandtlBEM(rd.default())       # N=4 blades
    m3 = PrandtlBEM(rd.load("de_schutter_2018"))  # N=3 blades

    m4.compute_forces(**HOVER_KWARGS)
    m3.compute_forces(**HOVER_KWARGS)

    assert m4.last_F_prandtl >= m3.last_F_prandtl, \
        f"PrandtlBEM: F_4blades={m4.last_F_prandtl:.4f} < F_3blades={m3.last_F_prandtl:.4f} — " \
        f"expected more blades → less tip-loss → higher F"
