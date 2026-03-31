"""
test_interface.py — Interface compliance tests for all five aero models.

Every model must expose an identical compute_forces() signature and the
required post-call diagnostic attributes.  These tests ensure new models
are drop-in replacements for RotorAero/DeSchutterAero in the mediator loop.
"""

import sys
import math
from pathlib import Path

import numpy as np
import pytest

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parents[2]))   # simulation/
sys.path.insert(0, str(_HERE.parent))       # simulation/aero/
sys.path.insert(0, str(_HERE))              # simulation/aero/tests/

from _helpers import ALL_MODELS, HOVER_KWARGS, DESIGN_KWARGS
import rotor_definition as rd


# ────────────────────────────────────────────────────────────────────────────
# 1. Construction via from_definition()
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_from_definition_constructs(AeroClass):
    """All models must construct via from_definition(rotor) without error."""
    rotor = rd.default()
    model = AeroClass.from_definition(rotor)
    assert model is not None
    assert hasattr(model, "compute_forces")


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_direct_constructor_matches_from_definition(AeroClass):
    """AeroClass(rotor) and AeroClass.from_definition(rotor) must give same forces."""
    rotor = rd.default()
    m1 = AeroClass(rotor)
    m2 = AeroClass.from_definition(rotor)
    f1 = m1.compute_forces(**HOVER_KWARGS)
    f2 = m2.compute_forces(**HOVER_KWARGS)
    np.testing.assert_allclose(f1, f2, rtol=1e-9,
                                err_msg=f"{AeroClass.__name__}: constructor vs from_definition differ")


# ────────────────────────────────────────────────────────────────────────────
# 2. Return type and shape
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_compute_forces_returns_6_element_array(AeroClass):
    """compute_forces must return an AeroResult with len 6 and wrench shape (6,)."""
    from aero import AeroResult
    rotor  = rd.default()
    model  = AeroClass(rotor)
    result = model.compute_forces(**HOVER_KWARGS)
    assert isinstance(result, AeroResult), \
        f"{AeroClass.__name__}: return type is not AeroResult, got {type(result)}"
    assert len(result) == 6, \
        f"{AeroClass.__name__}: expected len 6, got {len(result)}"
    assert result.wrench.shape == (6,), \
        f"{AeroClass.__name__}: expected wrench shape (6,), got {result.wrench.shape}"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_compute_forces_returns_finite_values(AeroClass):
    """All force/moment components must be finite (no NaN or Inf)."""
    rotor  = rd.default()
    model  = AeroClass(rotor)
    forces = model.compute_forces(**HOVER_KWARGS)
    assert np.all(np.isfinite(forces)), \
        f"{AeroClass.__name__}: forces contain NaN/Inf: {forces}"


# ────────────────────────────────────────────────────────────────────────────
# 2b. AeroResult field tests
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_compute_forces_returns_AeroResult(AeroClass):
    """compute_forces must return an AeroResult instance."""
    from aero import AeroResult
    rotor  = rd.default()
    model  = AeroClass(rotor)
    result = model.compute_forces(**HOVER_KWARGS)
    assert isinstance(result, AeroResult), \
        f"{AeroClass.__name__}: expected AeroResult, got {type(result)}"

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_F_world_shape(AeroClass):
    rotor = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    assert result.F_world.shape == (3,)
    assert np.all(np.isfinite(result.F_world))

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_M_orbital_shape(AeroClass):
    rotor = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    assert result.M_orbital.shape == (3,)
    assert np.all(np.isfinite(result.M_orbital))

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_Q_spin_is_float(AeroClass):
    rotor = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    assert isinstance(result.Q_spin, float)
    assert math.isfinite(result.Q_spin)

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_M_spin_shape(AeroClass):
    rotor = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    assert result.M_spin.shape == (3,)
    assert np.all(np.isfinite(result.M_spin))

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_wrench_reconstructs_6dof(AeroClass):
    """wrench == concat(F_world, M_orbital + M_spin) and has shape (6,)."""
    rotor = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    expected = np.concatenate([result.F_world, result.M_orbital + result.M_spin])
    np.testing.assert_array_equal(result.wrench, expected)

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_backward_compat_indexing(AeroClass):
    """result[i] must match result.wrench[i] for all i."""
    rotor = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    for i in range(6):
        assert result[i] == result.wrench[i], \
            f"{AeroClass.__name__}: result[{i}]={result[i]} != wrench[{i}]={result.wrench[i]}"

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_M_orbital_plus_M_spin_equals_total_moment(AeroClass):
    """M_orbital + M_spin must equal the total moment (wrench[3:])."""
    rotor = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    np.testing.assert_allclose(
        result.M_orbital + result.M_spin, result.wrench[3:], rtol=1e-9,
        err_msg=f"{AeroClass.__name__}: M_orbital + M_spin != total moment"
    )


# ────────────────────────────────────────────────────────────────────────────
# 3. Required diagnostic attributes
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_last_Q_spin_attribute_exists_and_is_float(AeroClass):
    """last_Q_spin must exist and be a Python float after compute_forces."""
    rotor = rd.default()
    model = AeroClass(rotor)
    model.compute_forces(**HOVER_KWARGS)
    assert hasattr(model, "last_Q_spin"), f"{AeroClass.__name__}: missing last_Q_spin"
    assert isinstance(model.last_Q_spin, float), \
        f"{AeroClass.__name__}: last_Q_spin is {type(model.last_Q_spin)}, expected float"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_last_M_spin_attribute_exists_and_is_ndarray3(AeroClass):
    """last_M_spin must exist and be a length-3 ndarray after compute_forces."""
    rotor = rd.default()
    model = AeroClass(rotor)
    model.compute_forces(**HOVER_KWARGS)
    assert hasattr(model, "last_M_spin"), f"{AeroClass.__name__}: missing last_M_spin"
    assert isinstance(model.last_M_spin, np.ndarray), \
        f"{AeroClass.__name__}: last_M_spin is not ndarray"
    assert model.last_M_spin.shape == (3,), \
        f"{AeroClass.__name__}: last_M_spin shape {model.last_M_spin.shape} != (3,)"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_last_v_inplane_attribute_nonnegative(AeroClass):
    """last_v_inplane must be non-negative (it's a magnitude)."""
    rotor = rd.default()
    model = AeroClass(rotor)
    model.compute_forces(**HOVER_KWARGS)
    assert hasattr(model, "last_v_inplane"), f"{AeroClass.__name__}: missing last_v_inplane"
    assert model.last_v_inplane >= 0.0, \
        f"{AeroClass.__name__}: last_v_inplane = {model.last_v_inplane} < 0"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_last_ramp_at_t10(AeroClass):
    """At t=10s (past 5s ramp), last_ramp must equal 1.0."""
    rotor = rd.default()
    model = AeroClass(rotor)
    model.compute_forces(**HOVER_KWARGS)   # t=10.0 in HOVER_KWARGS
    assert hasattr(model, "last_ramp"), f"{AeroClass.__name__}: missing last_ramp"
    assert math.isclose(model.last_ramp, 1.0, rel_tol=1e-9), \
        f"{AeroClass.__name__}: last_ramp = {model.last_ramp} at t=10s (expected 1.0)"


# ────────────────────────────────────────────────────────────────────────────
# 4. Ramp-to-zero at t=0
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_forces_zero_at_t0(AeroClass):
    """At t=0 all models must return exactly zero (ramp initialisation)."""
    rotor  = rd.default()
    model  = AeroClass(rotor)
    forces = model.compute_forces(
        collective_rad=0.1, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=np.eye(3), v_hub_world=np.zeros(3),
        omega_rotor=20.0, wind_world=np.array([0.0, 10.0, 0.0]),   # NED: East wind
        t=0.0,
    )
    np.testing.assert_allclose(forces, np.zeros(6), atol=1e-12,
                                err_msg=f"{AeroClass.__name__}: forces != 0 at t=0")


# ────────────────────────────────────────────────────────────────────────────
# 5. Linearity of ramp factor
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_forces_scale_with_ramp(AeroClass):
    """
    Forces at t=2.5 s (half-ramp) should be ≈ half of forces at t=10 s.
    Checks that ramp factor linearly scales the output.
    """
    rotor   = rd.default()
    model   = AeroClass(rotor)
    base_kw = dict(
        collective_rad=0.1, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=np.eye(3), v_hub_world=np.zeros(3),
        omega_rotor=20.0, wind_world=np.array([8.66, 0.0, 5.0]),
    )
    f_full = model.compute_forces(**base_kw, t=10.0)
    f_half = model.compute_forces(**base_kw, t=2.5)    # ramp = 2.5/5.0 = 0.5

    # Should be approximately half (allow some tolerance for nonlinear induction)
    ratio = np.linalg.norm(f_half) / max(np.linalg.norm(f_full), 1e-6)
    assert 0.4 < ratio < 0.6, \
        f"{AeroClass.__name__}: at half-ramp, force ratio = {ratio:.3f} (expected ~0.5)"


# ────────────────────────────────────────────────────────────────────────────
# 6. Determinism: repeated calls return same result
# ────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_deterministic_output(AeroClass):
    """Calling compute_forces twice with the same input must return the same result."""
    rotor  = rd.default()
    model  = AeroClass(rotor)
    f1     = model.compute_forces(**HOVER_KWARGS)
    f2     = model.compute_forces(**HOVER_KWARGS)
    np.testing.assert_allclose(f1, f2, rtol=1e-12,
                                err_msg=f"{AeroClass.__name__}: non-deterministic output")


# ────────────────────────────────────────────────────────────────────────────
# 7. Model-specific extras
# ────────────────────────────────────────────────────────────────────────────

def test_prandtl_bem_F_attribute():
    """PrandtlBEM must expose last_F_prandtl in (0, 1] after compute_forces."""
    from aero_prandtl_bem import PrandtlBEM
    rotor = rd.default()
    model = PrandtlBEM(rotor)
    model.compute_forces(**HOVER_KWARGS)
    assert 0.0 < model.last_F_prandtl <= 1.0, \
        f"last_F_prandtl = {model.last_F_prandtl} out of (0, 1]"


def test_skewed_wake_skew_angle_attribute():
    """SkewedWakeBEM must expose last_skew_angle_deg after compute_forces."""
    from aero_skewed_wake import SkewedWakeBEM
    rotor = rd.default()
    model = SkewedWakeBEM(rotor)
    model.compute_forces(**DESIGN_KWARGS)
    assert hasattr(model, "last_skew_angle_deg")
    assert 0.0 <= model.last_skew_angle_deg <= 90.0, \
        f"last_skew_angle_deg = {model.last_skew_angle_deg:.1f}° out of [0°, 90°]"


def test_glauert_states_state_attribute():
    """GlauertStateBEM must expose last_state string after compute_forces."""
    from aero_glauert_states import GlauertStateBEM
    rotor  = rd.default()
    model  = GlauertStateBEM(rotor)
    model.compute_forces(**DESIGN_KWARGS)
    assert hasattr(model, "last_state")
    assert model.last_state in ("windmill", "turbulent", "vortex_ring", "climb", "unknown"), \
        f"Unexpected last_state = {model.last_state!r}"


def test_glauert_states_a_mean_attribute():
    """GlauertStateBEM must expose last_a_mean (induction factor) after compute_forces."""
    from aero_glauert_states import GlauertStateBEM
    rotor = rd.default()
    model = GlauertStateBEM(rotor)
    model.compute_forces(**DESIGN_KWARGS)
    assert hasattr(model, "last_a_mean")
    assert math.isfinite(model.last_a_mean), f"last_a_mean is not finite: {model.last_a_mean}"
    assert model.last_a_mean >= 0.0, f"last_a_mean = {model.last_a_mean} < 0"
