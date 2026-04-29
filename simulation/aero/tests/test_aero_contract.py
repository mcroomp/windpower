"""
test_aero_contract.py — AeroBase API contract tests.

Every model that implements AeroBase must pass all tests in this file:
  1. Construction — from_definition() and direct constructor
  2. Return type and shape — AeroResult fields, wrench, indexing
  3. Diagnostic attributes — last_M_spin, last_v_inplane, last_ramp
  4. Ramp behaviour — zero at t=0, linear scaling, saturated at t>ramp_time
  5. Determinism — repeated calls with same inputs return same result
  6. Numerical robustness — finite output at extreme omega, wind, motion, collective
  7. Serialization — to_dict / state_dict roundtrip (Peters-He)
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

from _helpers import ALL_MODELS, HOVER_KWARGS, DESIGN_KWARGS
from aero import rotor_definition as rd


# ── 1. Construction ───────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_from_definition_constructs(AeroClass):
    """All models must construct via from_definition(rotor) without error."""
    rotor = rd.default()
    model = AeroClass.from_definition(rotor)
    assert model is not None
    assert hasattr(model, "compute_forces")


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_direct_constructor_matches_from_definition(AeroClass):
    """AeroClass(rotor) and AeroClass.from_definition(rotor) must give the same forces."""
    rotor = rd.default()
    m1 = AeroClass(rotor)
    m2 = AeroClass.from_definition(rotor)
    f1 = m1.compute_forces(**HOVER_KWARGS)
    f2 = m2.compute_forces(**HOVER_KWARGS)
    np.testing.assert_allclose(f1, f2, rtol=1e-9,
                                err_msg=f"{AeroClass.__name__}: constructor vs from_definition differ")


# ── 2. Return type and shape ──────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_compute_forces_returns_AeroResult(AeroClass):
    """compute_forces must return an AeroResult with wrench shape (6,) and len 6."""
    from aero import AeroResult
    rotor  = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    assert isinstance(result, AeroResult), \
        f"{AeroClass.__name__}: expected AeroResult, got {type(result)}"
    assert len(result) == 6
    assert result.wrench.shape == (6,)


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_compute_forces_returns_finite_values(AeroClass):
    """All force/moment components must be finite (no NaN or Inf)."""
    rotor  = rd.default()
    forces = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    assert np.all(np.isfinite(forces)), \
        f"{AeroClass.__name__}: forces contain NaN/Inf: {forces}"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_F_world_shape(AeroClass):
    rotor  = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    assert result.F_world.shape == (3,)
    assert np.all(np.isfinite(result.F_world))


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_M_orbital_shape(AeroClass):
    rotor  = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    assert result.M_orbital.shape == (3,)
    assert np.all(np.isfinite(result.M_orbital))


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_Q_spin_is_float(AeroClass):
    rotor  = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    assert isinstance(result.Q_spin, float)
    assert math.isfinite(result.Q_spin)


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_M_spin_shape(AeroClass):
    rotor  = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    assert result.M_spin.shape == (3,)
    assert np.all(np.isfinite(result.M_spin))


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_wrench_reconstructs_6dof(AeroClass):
    """wrench == concat(F_world, M_orbital + M_spin)."""
    rotor  = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    expected = np.concatenate([result.F_world, result.M_orbital + result.M_spin])
    np.testing.assert_array_equal(result.wrench, expected)


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_backward_compat_indexing(AeroClass):
    """result[i] must match result.wrench[i] for all i."""
    rotor  = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    for i in range(6):
        assert result[i] == result.wrench[i], \
            f"{AeroClass.__name__}: result[{i}]={result[i]} != wrench[{i}]={result.wrench[i]}"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_AeroResult_M_orbital_plus_M_spin_equals_total_moment(AeroClass):
    """M_orbital + M_spin must equal wrench[3:]."""
    rotor  = rd.default()
    result = AeroClass(rotor).compute_forces(**HOVER_KWARGS)
    np.testing.assert_allclose(
        result.M_orbital + result.M_spin, result.wrench[3:], rtol=1e-9,
        err_msg=f"{AeroClass.__name__}: M_orbital + M_spin != total moment",
    )


# ── 3. Diagnostic attributes ──────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_last_M_spin_attribute_exists_and_is_ndarray3(AeroClass):
    """last_M_spin must exist and be a length-3 ndarray after compute_forces."""
    rotor = rd.default()
    model = AeroClass(rotor)
    model.compute_forces(**HOVER_KWARGS)
    assert hasattr(model, "last_M_spin"), f"{AeroClass.__name__}: missing last_M_spin"
    assert isinstance(model.last_M_spin, np.ndarray)
    assert model.last_M_spin.shape == (3,)


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_last_v_inplane_attribute_nonnegative(AeroClass):
    """last_v_inplane must be non-negative (it is a magnitude)."""
    rotor = rd.default()
    model = AeroClass(rotor)
    model.compute_forces(**HOVER_KWARGS)
    assert hasattr(model, "last_v_inplane"), f"{AeroClass.__name__}: missing last_v_inplane"
    assert model.last_v_inplane >= 0.0, \
        f"{AeroClass.__name__}: last_v_inplane = {model.last_v_inplane} < 0"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_last_ramp_at_t10(AeroClass):
    """At t=10 s (past 5 s ramp), last_ramp must equal 1.0."""
    rotor = rd.default()
    model = AeroClass(rotor)
    model.compute_forces(**HOVER_KWARGS)   # t=10.0 in HOVER_KWARGS
    assert hasattr(model, "last_ramp"), f"{AeroClass.__name__}: missing last_ramp"
    assert math.isclose(model.last_ramp, 1.0, rel_tol=1e-9), \
        f"{AeroClass.__name__}: last_ramp = {model.last_ramp} at t=10 s (expected 1.0)"


# ── 4. Ramp behaviour ─────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_forces_zero_at_t0(AeroClass):
    """At t=0 all models must return exactly zero (ramp initialisation)."""
    rotor  = rd.default()
    forces = AeroClass(rotor).compute_forces(
        collective_rad=0.1, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=np.eye(3), v_hub_world=np.zeros(3),
        omega_rotor=20.0, wind_world=np.array([0.0, 10.0, 0.0]),
        t=0.0,
    )
    np.testing.assert_allclose(forces, np.zeros(6), atol=1e-12,
                                err_msg=f"{AeroClass.__name__}: forces != 0 at t=0")


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_forces_scale_with_ramp(AeroClass):
    """Forces at t=2.5 s (half-ramp) must be ~50% of forces at t=10 s."""
    rotor   = rd.default()
    model   = AeroClass(rotor)
    base_kw = dict(
        collective_rad=0.1, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=np.eye(3), v_hub_world=np.zeros(3),
        omega_rotor=20.0, wind_world=np.array([8.66, 0.0, 5.0]),
    )
    f_full = model.compute_forces(**base_kw, t=10.0)
    f_half = model.compute_forces(**base_kw, t=2.5)
    ratio = np.linalg.norm(f_half) / max(np.linalg.norm(f_full), 1e-6)
    assert 0.4 < ratio < 0.6, \
        f"{AeroClass.__name__}: at half-ramp, force ratio = {ratio:.3f} (expected ~0.5)"


# ── 5. Determinism ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_deterministic_output(AeroClass):
    """Two calls with identical inputs must return identical results."""
    rotor = rd.default()
    model = AeroClass(rotor)
    f1    = model.compute_forces(**HOVER_KWARGS)
    f2    = model.compute_forces(**HOVER_KWARGS)
    np.testing.assert_allclose(f1, f2, rtol=1e-12,
                                err_msg=f"{AeroClass.__name__}: non-deterministic output")


# ── 6. Numerical robustness ───────────────────────────────────────────────────

@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_stability_high_omega(AeroClass):
    """All models return finite forces at ω=100 rad/s (5× normal)."""
    rotor  = rd.default()
    forces = AeroClass(rotor).compute_forces(**{**DESIGN_KWARGS, "omega_rotor": 100.0})
    assert np.all(np.isfinite(forces)), \
        f"{AeroClass.__name__}: non-finite forces at omega=100: {forces}"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_stability_very_high_wind(AeroClass):
    """All models return finite forces at V=50 m/s wind."""
    rotor  = rd.default()
    forces = AeroClass(rotor).compute_forces(
        **{**DESIGN_KWARGS, "wind_world": np.array([40.0, 0.0, 20.0])})
    assert np.all(np.isfinite(forces)), \
        f"{AeroClass.__name__}: non-finite forces at V=50 m/s: {forces}"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_stability_hub_moving(AeroClass):
    """All models return finite forces when the hub is moving at 10 m/s."""
    rotor  = rd.default()
    forces = AeroClass(rotor).compute_forces(
        **{**DESIGN_KWARGS, "v_hub_world": np.array([5.0, 0.0, 3.0])})
    assert np.all(np.isfinite(forces)), \
        f"{AeroClass.__name__}: non-finite forces with hub motion: {forces}"


@pytest.mark.parametrize("AeroClass", ALL_MODELS, ids=[c.__name__ for c in ALL_MODELS])
def test_stability_large_collective(AeroClass):
    """All models handle ±0.35 rad collective (±20°) without NaN."""
    rotor = rd.default()
    model = AeroClass(rotor)
    for coll in [-0.35, 0.35]:
        forces = model.compute_forces(**{**DESIGN_KWARGS, "collective_rad": coll})
        assert np.all(np.isfinite(forces)), \
            f"{AeroClass.__name__}: non-finite forces at collective={coll:.2f}: {forces}"


# ── 7. Serialization (Peters-He) ──────────────────────────────────────────────

class TestPetersHeSerialization:
    """to_dict / state_dict roundtrip for PetersHeBEM."""

    @pytest.fixture(scope="class")
    def rotor(self):
        return rd.load("beaupoil_2026")

    def test_to_dict_restores_all_five_states(self, rotor):
        from aero import PetersHeBEM
        b = PetersHeBEM(rotor)
        b._v0, b._v1c, b._v1s, b._v2c, b._v2s = 1.1, -0.3, 0.7, 0.05, -0.02
        b._last_t = 12.5

        b2 = PetersHeBEM(rotor, state_dict=b.to_dict())

        assert b2._v0    == pytest.approx(1.1)
        assert b2._v1c   == pytest.approx(-0.3)
        assert b2._v1s   == pytest.approx(0.7)
        assert b2._v2c   == pytest.approx(0.05)
        assert b2._v2s   == pytest.approx(-0.02)
        assert b2._last_t == pytest.approx(12.5)

    def test_restored_object_skips_cold_start(self, rotor):
        """Restoring from state_dict (_last_t >= 0) skips cold-start initialisation."""
        from aero import PetersHeBEM
        b = PetersHeBEM(rotor)
        b._v0, b._last_t = 3.0, 5.0

        b2 = PetersHeBEM(rotor, state_dict=b.to_dict())
        assert b2._v0    == pytest.approx(3.0)
        assert b2._last_t >= 0.0

    def test_dict_contains_type_key(self, rotor):
        from aero import PetersHeBEM
        b = PetersHeBEM(rotor)
        assert b.to_dict()["type"] == "PetersHeBEM"
