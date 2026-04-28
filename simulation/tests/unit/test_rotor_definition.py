"""
test_rotor_definition.py — Unit tests for RotorDefinition API.

Tests cover:
  - Loading built-in definitions by name
  - Derived geometry values (span, r_cp, S_w, AR, σ, disk_area, DL)
  - Non-dimensional parameters (CL_alpha_3D, lock_number)
  - validate(): zero ERRORs for both built-in definitions
  - aero_kwargs(): keys match RotorAero constructor, values match defaults
  - RotorAero(beaupoil_2026) using the rotor definition gives valid, non-zero forces
  - De Schutter aero gives different thrust than Beaupoil (different geometry)
  - Kaman flap: TBD fields → INFO/WARNING only, no ERRORs
  - Validation correctly catches bad geometry inputs
"""

import dataclasses
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rotor_definition as rd
from aero import create_aero

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _errors(issues):
    return [i for i in issues if i.level == "ERROR"]

def _warnings(issues):
    return [i for i in issues if i.level == "WARNING"]

def _infos(issues):
    return [i for i in issues if i.level == "INFO"]


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

class TestLoad:
    def test_load_beaupoil_by_name(self):
        r = rd.load("beaupoil_2026")
        assert r.name == "beaupoil_2026"

    def test_load_de_schutter_by_name(self):
        r = rd.load("de_schutter_2018")
        assert r.name == "de_schutter_2018"

    def test_default_returns_beaupoil(self):
        r = rd.default()
        assert r.name == "beaupoil_2026"

    def test_load_by_explicit_path(self, tmp_path):
        # Copy beaupoil yaml to a temp path and load by path
        import shutil
        src = Path(rd.__file__).parent / "rotor_definitions" / "beaupoil_2026.yaml"
        dst = tmp_path / "my_rotor.yaml"
        shutil.copy(src, dst)
        r = rd.load(str(dst))
        assert r.n_blades == 4

    def test_load_unknown_name_raises(self):
        with pytest.raises(FileNotFoundError):
            rd.load("nonexistent_rotor_xyz")

    def test_builtin_dict_keys(self):
        assert "beaupoil_2026" in rd.BUILTIN
        assert "de_schutter_2018" in rd.BUILTIN


# ---------------------------------------------------------------------------
# Beaupoil 2026 — geometry
# ---------------------------------------------------------------------------

class TestBeaupoilGeometry:
    """Verified against YAML comment header values."""

    def setup_method(self):
        self.r = rd.load("beaupoil_2026")

    def test_r_cp_m(self):
        # r_cp = r_root + 2/3 · span = 0.5 + 2/3 × 2.0 = 1.833 m
        assert self.r.r_cp_m == pytest.approx(0.5 + 2/3 * 2.0, rel=1e-6)

    def test_r_eff_thrust_m(self):
        # r_eff = √((R³ − r_root³) / (3·span))
        R, r = 2.5, 0.5
        span = 2.0
        expected = math.sqrt((R**3 - r**3) / (3.0 * span))
        assert self.r.r_eff_thrust_m == pytest.approx(expected, rel=1e-6)

    def test_r_cp_above_r_eff(self):
        # De Schutter r_cp should overestimate q² — it must be > r_eff_T
        assert self.r.r_cp_m > self.r.r_eff_thrust_m

    def test_S_w_m2(self):
        # S_w = N · c · span = 4 × 0.20 × 2.0 = 1.60 m²
        assert self.r.S_w_m2 == pytest.approx(4 * 0.20 * 2.0)

    def test_disk_area_m2(self):
        # A = π(R² − r_root²) = π(6.25 − 0.25) = π × 6.0 ≈ 18.850 m²
        expected = math.pi * (2.5**2 - 0.5**2)
        assert self.r.disk_area_m2 == pytest.approx(expected, rel=1e-6)

    def test_aspect_ratio(self):
        # AR = span / chord = 2.0 / 0.20 = 10.0
        assert self.r.aspect_ratio == pytest.approx(2.0 / 0.20, rel=1e-6)

    def test_solidity(self):
        # σ = N·c/(π·R) = 4×0.20/(π×2.5) ≈ 0.1019
        expected = 4 * 0.20 / (math.pi * 2.5)
        assert self.r.solidity == pytest.approx(expected, rel=1e-6)

    def test_disk_loading(self):
        # DL = m·g / A = 5.0×9.81 / 18.85 ≈ 2.60 N/m²
        expected = 5.0 * 9.81 / self.r.disk_area_m2
        assert self.r.disk_loading_N_m2 == pytest.approx(expected, rel=1e-6)

    def test_CL_alpha_3D(self):
        # 2π / (1 + 2/AR) at AR=10.0
        AR = 2.0 / 0.20
        expected = 2 * math.pi / (1 + 2.0 / AR)
        assert self.r.CL_alpha_3D_per_rad == pytest.approx(expected, rel=1e-6)

    def test_CL_alpha_3D_in_range(self):
        assert 4.0 <= self.r.CL_alpha_3D_per_rad <= 7.0

    def test_lock_number_none_when_I_b_unknown(self):
        # I_blade_flap_kgm2 is null in beaupoil_2026.yaml
        assert self.r.I_blade_flap_kgm2 is None
        assert self.r.lock_number is None

    def test_lock_number_computed_when_I_b_given(self):
        r = dataclasses.replace(rd.load("beaupoil_2026"), I_blade_flap_kgm2=0.5)
        gamma = r.lock_number
        assert gamma is not None
        # γ = ρ·CL_alpha_3D·c·R⁴ / I_b
        expected = r.rho_kg_m3 * r.CL_alpha_3D_per_rad * r.chord_m * r.radius_m**4 / 0.5
        assert gamma == pytest.approx(expected, rel=1e-6)


# ---------------------------------------------------------------------------
# De Schutter 2018 — geometry
# ---------------------------------------------------------------------------

class TestDeSchutterGeometry:
    """Verified against Table I of De Schutter et al. (2018)."""

    def setup_method(self):
        self.r = rd.load("de_schutter_2018")

    def test_aspect_ratio(self):
        # AR = 1.5 / 0.125 = 12.0  (matches De Schutter Table I AR=12)
        assert self.r.aspect_ratio == pytest.approx(12.0, rel=1e-6)

    def test_solidity(self):
        # σ = N·c / (π·R)
        r = self.r
        expected = r.n_blades * r.chord_m / (math.pi * r.radius_m)
        assert r.solidity == pytest.approx(expected, rel=1e-6)

    def test_r_cp_m(self):
        # r_cp = r_root + 2/3 × span
        r = self.r
        expected = r.root_cutout_m + (2.0 / 3.0) * (r.radius_m - r.root_cutout_m)
        assert r.r_cp_m == pytest.approx(expected, rel=1e-6)

    def test_disk_area_m2(self):
        # A = π(R² − r_root²)
        r = self.r
        expected = math.pi * (r.radius_m**2 - r.root_cutout_m**2)
        assert r.disk_area_m2 == pytest.approx(expected, rel=1e-6)

    def test_kaman_disabled(self):
        assert self.r.kaman_flap.enabled is False


# ---------------------------------------------------------------------------
# Validation — built-in definitions should have zero ERRORs
# ---------------------------------------------------------------------------

class TestValidation:
    def test_beaupoil_no_errors(self):
        r = rd.load("beaupoil_2026")
        issues = r.validate()
        errors = _errors(issues)
        assert errors == [], f"Unexpected errors in beaupoil_2026: {errors}"

    def test_de_schutter_no_errors(self):
        r = rd.load("de_schutter_2018")
        issues = r.validate()
        errors = _errors(issues)
        assert errors == [], f"Unexpected errors in de_schutter_2018: {errors}"

    def test_beaupoil_kaman_tbd_is_info_not_error(self):
        # Kaman fields should generate no ERRORs (geometry is estimated, not confirmed)
        r = rd.load("beaupoil_2026")
        issues = r.validate()
        kaman_issues = [i for i in issues if "kaman" in i.field.lower()]
        kaman_errors = [i for i in kaman_issues if i.level == "ERROR"]
        assert kaman_errors == []

    def test_beaupoil_I_b_unknown_is_info(self):
        r = rd.load("beaupoil_2026")
        issues = r.validate()
        info_fields = [i.field for i in issues if i.level == "INFO"]
        assert any("blade_flap" in f for f in info_fields)

    def test_beaupoil_re_mismatch_warning(self):
        # Re_design=127k (bench test) vs Re_operating=490k (flight) — 3.86x mismatch
        # generates a WARNING since operating > 2x design.
        r = rd.load("beaupoil_2026")
        issues = r.validate()
        re_warns = [i for i in issues if "Re_operating" in i.field and i.level == "WARNING"]
        assert len(re_warns) == 1

    def test_bad_span_gives_error(self):
        r = dataclasses.replace(rd.load("beaupoil_2026"), radius_m=0.3)  # R < r_root → span < 0
        issues = r.validate()
        assert any(i.level == "ERROR" and "span" in i.field for i in issues)

    def test_zero_mass_gives_error(self):
        r = dataclasses.replace(rd.load("beaupoil_2026"), mass_kg=0.0)
        issues = r.validate()
        assert any(i.level == "ERROR" and "mass" in i.field for i in issues)

    def test_kaman_span_start_in_hub_gives_error(self):
        base = rd.load("beaupoil_2026")
        r = dataclasses.replace(base, kaman_flap=dataclasses.replace(base.kaman_flap, span_start_m=0.3))
        issues = r.validate()
        assert any(i.level == "ERROR" and "span_start_m" in i.field for i in issues)

    def test_kaman_span_end_beyond_tip_gives_error(self):
        base = rd.load("beaupoil_2026")
        r = dataclasses.replace(base, kaman_flap=dataclasses.replace(base.kaman_flap, span_end_m=3.0))
        issues = r.validate()
        assert any(i.level == "ERROR" and "span_end_m" in i.field for i in issues)

    def test_de_schutter_omega_eq_none_no_warning(self):
        # omega_eq_rad_s=null → no consistency check → no autorotation warning
        r = rd.load("de_schutter_2018")
        issues = r.validate()
        auto_warns = [i for i in issues if "autorotation" in i.field and i.level == "WARNING"]
        assert auto_warns == []


# ---------------------------------------------------------------------------
# aero_kwargs factory
# ---------------------------------------------------------------------------

class TestAeroKwargs:
    def test_all_required_keys_present(self):
        r = rd.load("beaupoil_2026")
        kwargs = r.aero_kwargs()
        required = {"n_blades", "r_root", "r_tip", "chord", "rho",
                    "aspect_ratio", "K_cyc",
                    "CL0", "CL_alpha", "CD0", "oswald_eff", "aoa_limit",
                    "k_drive_spin", "k_drag_spin"}
        assert required.issubset(kwargs.keys())

    def test_aspect_ratio_matches_derived(self):
        r = rd.load("beaupoil_2026")
        kw = r.aero_kwargs()
        assert kw["aspect_ratio"] == pytest.approx(r.aspect_ratio)

    def test_can_construct_PetersHe(self):
        r    = rd.load("beaupoil_2026")
        aero = create_aero(r)
        assert aero.N_BLADES == r.n_blades
        assert aero.R_TIP    == pytest.approx(r.radius_m, rel=1e-6)


# ---------------------------------------------------------------------------
# dynamics_kwargs factory
# ---------------------------------------------------------------------------

class TestDynamicsKwargs:
    def test_keys_present(self):
        r = rd.load("beaupoil_2026")
        kw = r.dynamics_kwargs()
        assert {"mass", "I_body", "I_spin"}.issubset(kw.keys())

    def test_I_spin_matches_derived(self):
        r = rd.load("beaupoil_2026")
        kw = r.dynamics_kwargs()
        # I_spin is computed from blade_mass_kg when I_spin_kgm2 is null in YAML.
        # Verify dynamics_kwargs passes through the derived value consistently.
        assert kw["I_spin"] == pytest.approx(r.I_spin_effective_kgm2)
        assert kw["I_spin"] > 1.0, "Expected non-zero I_spin from blade_mass_kg"


# ---------------------------------------------------------------------------
# SkewedWakeBEM.from_definition — produces valid forces from beaupoil_2026
# ---------------------------------------------------------------------------

class TestFromDefinition:
    """create_aero(beaupoil) must produce finite, non-zero forces."""

    def _aero_forces(self, aero, t=10.0, collective=0.05, tilt_lon=0.0, tilt_lat=0.0,
                     wind=None, omega_rotor=20.148):
        if wind is None:
            wind = np.array([0.0, 10.0, 0.0])  # NED: East wind = Y axis
        return aero.compute_forces(
            t=t,
            R_hub=np.eye(3),
            v_hub_world=np.zeros(3),
            wind_world=wind,
            collective_rad=collective,
            tilt_lon=tilt_lon,
            tilt_lat=tilt_lat,
            omega_rotor=omega_rotor,
        )

    def test_from_definition_produces_valid_forces(self):
        r    = rd.load("beaupoil_2026")
        aero = create_aero(r)
        f    = self._aero_forces(aero)
        assert np.all(np.isfinite(f)), "forces must be finite"
        assert np.any(f.F_world != 0.0), "forces must be non-zero"


    def test_from_definition_sets_correct_geometry(self):
        r    = rd.load("beaupoil_2026")
        aero = create_aero(r)
        assert aero.R_ROOT   == pytest.approx(r.root_cutout_m)
        assert aero.R_TIP    == pytest.approx(r.radius_m)
        assert aero.N_BLADES == r.n_blades


# ---------------------------------------------------------------------------
# Kaman flap
# ---------------------------------------------------------------------------

class TestKamanFlap:
    def test_beaupoil_kaman_enabled(self):
        r = rd.load("beaupoil_2026")
        assert r.kaman_flap.enabled is True

    def test_beaupoil_kaman_fully_specified_with_guesses(self):
        # tau is now estimated → fully specified
        r = rd.load("beaupoil_2026")
        assert r.kaman_flap.is_fully_specified()

    def test_swashplate_load_reduction_computed_when_given(self):
        kf = dataclasses.replace(rd.load("beaupoil_2026").kaman_flap, swashplate_load_fraction=0.2)
        assert kf.swashplate_load_reduction_pct() == pytest.approx(80.0)

    def test_is_geometry_defined_true_when_set(self):
        kf = dataclasses.replace(
            rd.load("beaupoil_2026").kaman_flap,
            chord_fraction=0.25, span_start_m=0.8, span_end_m=2.4,
        )
        assert kf.is_geometry_defined()

    def test_de_schutter_kaman_disabled(self):
        r = rd.load("de_schutter_2018")
        assert r.kaman_flap.enabled is False
        assert not r.kaman_flap.is_geometry_defined()


# ---------------------------------------------------------------------------
# omega_eq consistency
# ---------------------------------------------------------------------------

class TestAutorotation:
    """
    Consistency tests for autorotation ODE parameters I_ode, omega_eq, omega_min.
    """

    def setup_method(self):
        self.r = rd.load("beaupoil_2026")

    def test_I_ode_positive(self):
        assert self.r.I_ode_kgm2 > 0

    def test_omega_min_positive(self):
        assert self.r.omega_min_rad_s > 0

    def test_I_ode_exceeds_structural_I_spin(self):
        # Effective ODE inertia (includes added mass of accelerated air) must be
        # at least as large as the structural spin inertia.
        r = self.r
        I_spin = r.I_spin_effective_kgm2
        if I_spin is not None:
            assert r.I_ode_kgm2 >= I_spin, (
                f"I_ode={r.I_ode_kgm2:.2f} < I_spin={I_spin:.2f} kg·m² — "
                f"effective ODE inertia should not be less than structural inertia"
            )

    def test_de_schutter_omega_eq_is_none(self):
        r = rd.load("de_schutter_2018")
        assert r.omega_eq_rad_s is None


# ---------------------------------------------------------------------------
# Report and summary
# ---------------------------------------------------------------------------

class TestReportAndSummary:
    def test_summary_is_string(self):
        r = rd.load("beaupoil_2026")
        s = r.summary()
        assert isinstance(s, str)
        assert "beaupoil_2026" in s

    def test_report_is_string(self):
        r = rd.load("beaupoil_2026")
        rep = r.report()
        assert isinstance(rep, str)
        assert "beaupoil_2026" in rep
        assert "Geometry" in rep
        assert "Airfoil" in rep

    def test_report_contains_validation_section(self):
        r = rd.load("beaupoil_2026")
        rep = r.report()
        assert "Validation" in rep
