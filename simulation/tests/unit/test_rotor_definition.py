"""
test_rotor_definition.py — Unit tests for RotorDefinition API.

Tests cover:
  - Loading built-in definitions by name
  - Derived geometry values (span, r_cp, S_w, AR, σ, disk_area, DL)
  - Non-dimensional parameters (CL_alpha_3D, lock_number)
  - validate(): zero ERRORs for both built-in definitions
  - aero_kwargs(): keys match RotorAero constructor, values match defaults
  - RotorAero constructed from beaupoil_2026 gives same forces as default RotorAero()
  - De Schutter aero gives different thrust than Beaupoil (different geometry)
  - Kaman flap: TBD fields → INFO/WARNING only, no ERRORs
  - Validation correctly catches bad geometry inputs
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rotor_definition as rd
from aero import RotorAero

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

    def test_n_blades(self):
        assert self.r.n_blades == 4

    def test_radius_m(self):
        assert self.r.radius_m == pytest.approx(2.5)

    def test_root_cutout_m(self):
        assert self.r.root_cutout_m == pytest.approx(0.5)

    def test_chord_m(self):
        assert self.r.chord_m == pytest.approx(0.15)

    def test_span_m(self):
        # span = R − r_root = 2.5 − 0.5 = 2.0 m
        assert self.r.span_m == pytest.approx(2.0)

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
        # S_w = N · c · span = 4 × 0.15 × 2.0 = 1.20 m²
        assert self.r.S_w_m2 == pytest.approx(4 * 0.15 * 2.0)

    def test_disk_area_m2(self):
        # A = π(R² − r_root²) = π(6.25 − 0.25) = π × 6.0 ≈ 18.850 m²
        expected = math.pi * (2.5**2 - 0.5**2)
        assert self.r.disk_area_m2 == pytest.approx(expected, rel=1e-6)

    def test_aspect_ratio(self):
        # AR = span / chord = 2.0 / 0.15 ≈ 13.333
        assert self.r.aspect_ratio == pytest.approx(2.0 / 0.15, rel=1e-6)

    def test_solidity(self):
        # σ = N·c/(π·R) = 4×0.15/(π×2.5) ≈ 0.0764
        expected = 4 * 0.15 / (math.pi * 2.5)
        assert self.r.solidity == pytest.approx(expected, rel=1e-6)

    def test_disk_loading(self):
        # DL = m·g / A = 5.0×9.81 / 18.85 ≈ 2.60 N/m²
        expected = 5.0 * 9.81 / self.r.disk_area_m2
        assert self.r.disk_loading_N_m2 == pytest.approx(expected, rel=1e-6)

    def test_CL_alpha_3D(self):
        # 2π / (1 + 2/AR) at AR=13.333
        AR = 2.0 / 0.15
        expected = 2 * math.pi / (1 + 2.0 / AR)
        assert self.r.CL_alpha_3D_per_rad == pytest.approx(expected, rel=1e-6)

    def test_CL_alpha_3D_above_empirical(self):
        # Thin-airfoil theory must exceed Weyel empirical value
        assert self.r.CL_alpha_3D_per_rad > self.r.CL_alpha_per_rad

    def test_lock_number_none_when_I_b_unknown(self):
        # I_blade_flap_kgm2 is null in beaupoil_2026.yaml
        assert self.r.I_blade_flap_kgm2 is None
        assert self.r.lock_number is None

    def test_lock_number_computed_when_I_b_given(self):
        r = rd.load("beaupoil_2026")
        r.I_blade_flap_kgm2 = 0.5
        gamma = r.lock_number
        assert gamma is not None
        # γ = ρ·a·c·R⁴ / I_b = 1.22 × 0.87 × 0.15 × 2.5⁴ / 0.5
        expected = 1.22 * 0.87 * 0.15 * (2.5**4) / 0.5
        assert gamma == pytest.approx(expected, rel=1e-6)


# ---------------------------------------------------------------------------
# De Schutter 2018 — geometry
# ---------------------------------------------------------------------------

class TestDeSchutterGeometry:
    """Verified against Table I of De Schutter et al. (2018)."""

    def setup_method(self):
        self.r = rd.load("de_schutter_2018")

    def test_n_blades(self):
        assert self.r.n_blades == 3

    def test_radius_m(self):
        assert self.r.radius_m == pytest.approx(2.0)

    def test_chord_m(self):
        assert self.r.chord_m == pytest.approx(0.125)

    def test_span_m(self):
        # span = 2.0 − 0.5 = 1.5 m
        assert self.r.span_m == pytest.approx(1.5)

    def test_aspect_ratio(self):
        # AR = 1.5 / 0.125 = 12.0  (matches De Schutter Table I AR=12)
        assert self.r.aspect_ratio == pytest.approx(12.0, rel=1e-6)

    def test_solidity(self):
        # σ = 3×0.125/(π×2.0) ≈ 0.0597
        expected = 3 * 0.125 / (math.pi * 2.0)
        assert self.r.solidity == pytest.approx(expected, rel=1e-6)

    def test_r_cp_m(self):
        # r_cp = 0.5 + 2/3 × 1.5 = 1.5 m
        assert self.r.r_cp_m == pytest.approx(1.5, rel=1e-6)

    def test_disk_area_m2(self):
        # A = π(4.0 − 0.25) = π×3.75 ≈ 11.781 m²
        expected = math.pi * (2.0**2 - 0.5**2)
        assert self.r.disk_area_m2 == pytest.approx(expected, rel=1e-6)

    def test_mass_kg(self):
        assert self.r.mass_kg == pytest.approx(40.0)

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

    def test_beaupoil_re_mismatch_is_warning(self):
        # Operating Re = 370000 > 2× design Re 127000 → WARNING
        r = rd.load("beaupoil_2026")
        issues = r.validate()
        re_warns = [i for i in issues if "Re_operating" in i.field]
        assert len(re_warns) == 1
        assert re_warns[0].level == "WARNING"

    def test_bad_span_gives_error(self):
        r = rd.load("beaupoil_2026")
        r.radius_m = 0.3   # R < r_root → span < 0
        issues = r.validate()
        assert any(i.level == "ERROR" and "span" in i.field for i in issues)

    def test_zero_mass_gives_error(self):
        r = rd.load("beaupoil_2026")
        r.mass_kg = 0.0
        issues = r.validate()
        assert any(i.level == "ERROR" and "mass" in i.field for i in issues)

    def test_negative_CL_alpha_gives_error(self):
        r = rd.load("beaupoil_2026")
        r.CL_alpha_per_rad = -0.1
        issues = r.validate()
        assert any(i.level == "ERROR" and "CL_alpha" in i.field for i in issues)

    def test_kaman_span_start_in_hub_gives_error(self):
        r = rd.load("beaupoil_2026")
        r.kaman_flap.span_start_m = 0.3   # < root_cutout_m=0.5 → inside hub
        issues = r.validate()
        assert any(i.level == "ERROR" and "span_start_m" in i.field for i in issues)

    def test_kaman_span_end_beyond_tip_gives_error(self):
        r = rd.load("beaupoil_2026")
        r.kaman_flap.span_end_m = 3.0   # > R=2.5 → beyond tip
        issues = r.validate()
        assert any(i.level == "ERROR" and "span_end_m" in i.field for i in issues)

    def test_omega_eq_consistency_warning(self):
        # If omega_eq_rad_s differs by >15% from K-derived value, warn
        r = rd.load("beaupoil_2026")
        r.omega_eq_rad_s = 50.0   # far from theory ~21 rad/s → >15% difference
        issues = r.validate()
        auto_warns = [i for i in issues if "autorotation" in i.field and i.level == "WARNING"]
        assert len(auto_warns) >= 1

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
                    "aspect_ratio", "oswald_eff", "CD0", "CL0", "CL_alpha",
                    "K_cyc", "aoa_limit", "ramp_time"}
        assert required.issubset(kwargs.keys())

    def test_values_match_yaml(self):
        r = rd.load("beaupoil_2026")
        kw = r.aero_kwargs()
        assert kw["n_blades"] == 4
        assert kw["r_root"] == pytest.approx(0.5)
        assert kw["r_tip"]  == pytest.approx(2.5)
        assert kw["chord"]  == pytest.approx(0.15)
        assert kw["CL0"]    == pytest.approx(0.11)
        assert kw["CL_alpha"] == pytest.approx(0.87)
        assert kw["CD0"]    == pytest.approx(0.01)
        assert kw["oswald_eff"] == pytest.approx(0.8)
        assert kw["K_cyc"]  == pytest.approx(0.4)
        assert kw["aoa_limit"] == pytest.approx(math.radians(15.0))

    def test_aspect_ratio_matches_derived(self):
        r = rd.load("beaupoil_2026")
        kw = r.aero_kwargs()
        assert kw["aspect_ratio"] == pytest.approx(r.aspect_ratio)

    def test_can_construct_RotorAero(self):
        r = rd.load("beaupoil_2026")
        aero = RotorAero(**r.aero_kwargs())
        assert aero.n_blades == 4
        assert aero.r_cp == pytest.approx(r.r_cp_m, rel=1e-6)
        assert aero.S_w  == pytest.approx(r.S_w_m2,  rel=1e-6)


# ---------------------------------------------------------------------------
# dynamics_kwargs factory
# ---------------------------------------------------------------------------

class TestDynamicsKwargs:
    def test_keys_present(self):
        r = rd.load("beaupoil_2026")
        kw = r.dynamics_kwargs()
        assert {"mass", "I_body", "I_spin"}.issubset(kw.keys())

    def test_values_match_yaml(self):
        r = rd.load("beaupoil_2026")
        kw = r.dynamics_kwargs()
        assert kw["mass"] == pytest.approx(5.0)
        assert kw["I_body"] == pytest.approx([5.0, 5.0, 10.0])
        assert kw["I_spin"] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# RotorAero.from_definition — produces same forces as default RotorAero()
# ---------------------------------------------------------------------------

class TestFromDefinition:
    """RotorAero.from_definition(beaupoil) must produce identical forces to
    RotorAero() with default parameters, since beaupoil_2026.yaml IS the
    reference configuration that _DEFAULTS was extracted from."""

    def _aero_forces(self, aero, t=10.0, collective=0.05, tilt_lon=0.0, tilt_lat=0.0,
                     wind=None, omega_rotor=20.148):
        if wind is None:
            wind = np.array([10.0, 0.0, 0.0])
        R = np.eye(3)
        return aero.compute_forces(
            t=t,
            R_hub=R,
            v_hub_world=np.zeros(3),
            wind_world=wind,
            collective_rad=collective,
            tilt_lon=tilt_lon,
            tilt_lat=tilt_lat,
            omega_rotor=omega_rotor,
        )

    def test_from_definition_matches_default(self):
        r = rd.load("beaupoil_2026")
        aero_from_def = RotorAero.from_definition(r)
        aero_default  = RotorAero()

        forces_from_def = self._aero_forces(aero_from_def)
        forces_default  = self._aero_forces(aero_default)

        # Small difference expected: _DEFAULTS uses rounded aspect_ratio=13.3,
        # beaupoil_2026 derives exact 2.0/0.15=13.333... — allow 2% tolerance.
        np.testing.assert_allclose(forces_from_def, forces_default, rtol=0.02,
                                   err_msg="RotorAero.from_definition(beaupoil) must be close to RotorAero() defaults")

    def test_de_schutter_gives_different_thrust_than_beaupoil(self):
        beaupoil  = rd.load("beaupoil_2026")
        deschutter = rd.load("de_schutter_2018")

        aero_b = RotorAero.from_definition(beaupoil)
        aero_d = RotorAero.from_definition(deschutter)

        forces_b = self._aero_forces(aero_b)
        forces_d = self._aero_forces(aero_d)

        # Thrust is F[2] (z-component = along disk normal in identity-R frame)
        T_beaupoil    = forces_b[2]
        T_de_schutter = forces_d[2]

        # De Schutter: 3 blades, shorter chord → different thrust than 4-blade Beaupoil
        # They should NOT be equal
        assert abs(T_beaupoil - T_de_schutter) > 1.0, (
            f"Thrust should differ between rotors: beaupoil={T_beaupoil:.1f}N, "
            f"de_schutter={T_de_schutter:.1f}N"
        )

    def test_from_definition_sets_correct_geometry(self):
        r = rd.load("beaupoil_2026")
        aero = RotorAero.from_definition(r)
        assert aero.r_cp    == pytest.approx(r.r_cp_m,  rel=1e-6)
        assert aero.S_w     == pytest.approx(r.S_w_m2,  rel=1e-6)
        assert aero.r_root  == pytest.approx(r.root_cutout_m)
        assert aero.r_tip   == pytest.approx(r.radius_m)
        assert aero.n_blades == r.n_blades


# ---------------------------------------------------------------------------
# Kaman flap
# ---------------------------------------------------------------------------

class TestKamanFlap:
    def test_beaupoil_kaman_enabled(self):
        r = rd.load("beaupoil_2026")
        assert r.kaman_flap.enabled is True

    def test_beaupoil_kaman_geometry_estimated(self):
        # Geometry is now filled with GUESS values, not TBD null
        r = rd.load("beaupoil_2026")
        kf = r.kaman_flap
        assert kf.chord_fraction == pytest.approx(0.25)
        assert kf.span_start_m   == pytest.approx(1.2)
        assert kf.span_end_m     == pytest.approx(2.5)
        assert kf.is_geometry_defined()

    def test_beaupoil_kaman_fully_specified_with_guesses(self):
        # tau is now estimated → fully specified
        r = rd.load("beaupoil_2026")
        assert r.kaman_flap.is_fully_specified()

    def test_swashplate_load_reduction_estimated(self):
        # swashplate_load_fraction = 0.1 (GUESS) → 90% reduction
        r = rd.load("beaupoil_2026")
        assert r.kaman_flap.swashplate_load_fraction == pytest.approx(0.1)
        assert r.kaman_flap.swashplate_load_reduction_pct() == pytest.approx(90.0)

    def test_swashplate_load_reduction_computed_when_given(self):
        r = rd.load("beaupoil_2026")
        r.kaman_flap.swashplate_load_fraction = 0.2   # 80% load reduction
        pct = r.kaman_flap.swashplate_load_reduction_pct()
        assert pct == pytest.approx(80.0)

    def test_is_geometry_defined_true_when_set(self):
        r = rd.load("beaupoil_2026")
        r.kaman_flap.chord_fraction = 0.25
        r.kaman_flap.span_start_m   = 0.8
        r.kaman_flap.span_end_m     = 2.4
        assert r.kaman_flap.is_geometry_defined()

    def test_de_schutter_kaman_disabled(self):
        r = rd.load("de_schutter_2018")
        assert r.kaman_flap.enabled is False
        assert not r.kaman_flap.is_geometry_defined()


# ---------------------------------------------------------------------------
# omega_eq consistency
# ---------------------------------------------------------------------------

class TestOmegaEq:
    def test_beaupoil_omega_eq_within_tolerance(self):
        # beaupoil omega_eq_rad_s = 20.148; K-derived ≈ 21.4 rad/s at v_inplane=5.83 m/s
        # Difference should be < 15% (the K constants were empirically tuned)
        r = rd.load("beaupoil_2026")
        omega_theory = r.omega_eq_from_K_rad_s
        omega_nominal = r.omega_eq_rad_s
        assert omega_nominal is not None
        diff_pct = abs(omega_theory - omega_nominal) / omega_nominal
        assert diff_pct < 0.15, (
            f"omega_eq theory={omega_theory:.3f} vs nominal={omega_nominal:.3f} "
            f"differs by {diff_pct:.0%}, should be <15%"
        )

    def test_omega_eq_from_K_is_positive(self):
        r = rd.load("beaupoil_2026")
        assert r.omega_eq_from_K_rad_s > 0

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
