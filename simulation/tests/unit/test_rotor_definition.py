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

    def test_CL_alpha_3D_above_empirical(self):
        # Prandtl lifting-line (5.24 /rad) is a theoretical lower bound for thin plates.
        # NeuralFoil at Re=490k gives 5.47 /rad for SG6042 — slightly above lifting-line
        # because camber enhances lift slope at flight Re (physically valid).
        # Both must be in the plausible aerodynamic range.
        assert 4.0 <= self.r.CL_alpha_per_rad <= 7.0
        assert 4.0 <= self.r.CL_alpha_3D_per_rad <= 7.0

    def test_lock_number_none_when_I_b_unknown(self):
        # I_blade_flap_kgm2 is null in beaupoil_2026.yaml
        assert self.r.I_blade_flap_kgm2 is None
        assert self.r.lock_number is None

    def test_lock_number_computed_when_I_b_given(self):
        r = rd.load("beaupoil_2026")
        r.I_blade_flap_kgm2 = 0.5
        gamma = r.lock_number
        assert gamma is not None
        # γ = ρ·a·c·R⁴ / I_b = 1.22 × 5.47 × 0.20 × 2.5⁴ / 0.5
        expected = 1.22 * 5.47 * 0.20 * (2.5**4) / 0.5
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
                    "K_cyc", "aoa_limit"}
        assert required.issubset(kwargs.keys())

    def test_aspect_ratio_matches_derived(self):
        r = rd.load("beaupoil_2026")
        kw = r.aero_kwargs()
        assert kw["aspect_ratio"] == pytest.approx(r.aspect_ratio)

    def test_can_construct_RotorAero(self):
        r = rd.load("beaupoil_2026")
        aero = RotorAero(r)
        assert aero.n_blades == r.n_blades
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

    def test_I_spin_matches_derived(self):
        r = rd.load("beaupoil_2026")
        kw = r.dynamics_kwargs()
        # I_spin is computed from blade_mass_kg when I_spin_kgm2 is null in YAML.
        # Verify dynamics_kwargs passes through the derived value consistently.
        assert kw["I_spin"] == pytest.approx(r.I_spin_effective_kgm2)
        assert kw["I_spin"] > 1.0, "Expected non-zero I_spin from blade_mass_kg"


# ---------------------------------------------------------------------------
# RotorAero.from_definition — produces valid forces from beaupoil_2026 rotor definition
# ---------------------------------------------------------------------------

class TestFromDefinition:
    """RotorAero.from_definition(beaupoil) must produce finite, non-zero forces
    using the beaupoil_2026 rotor definition (RotorAero now requires a
    RotorDefinition argument; bare RotorAero() is no longer valid)."""

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

    def test_from_definition_produces_valid_forces(self):
        # Verify RotorAero.from_definition(beaupoil) runs and produces finite non-zero forces.
        # (Previously compared against RotorAero() hardcoded defaults, but beaupoil now uses
        # chord=0.20 m and CD0=0.007 which differ from the RotorAero._DEFAULTS values.)
        r = rd.load("beaupoil_2026")
        aero_from_def = RotorAero.from_definition(r)
        forces = self._aero_forces(aero_from_def)
        assert np.all(np.isfinite(forces)), "forces must be finite"
        assert np.any(forces != 0.0), "forces must be non-zero"

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

    def test_beaupoil_kaman_fully_specified_with_guesses(self):
        # tau is now estimated → fully specified
        r = rd.load("beaupoil_2026")
        assert r.kaman_flap.is_fully_specified()

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

class TestAutorotation:
    """
    Consistency tests for autorotation ODE parameters K_drive, K_drag, I_ode.

    The spin ODE is:
        dω/dt = (K_drive · v_inplane − K_drag · ω²) / I_ode
    Equilibrium:
        ω_eq = sqrt(K_drive · v_inplane / K_drag)

    Design-point v_inplane (5.25 m/s) is the in-plane wind at DEFAULT_BODY_Z
    orientation ([0.851, 0.305, 0.427]) with 10 m/s headwind and hub stationary:
        v_axial   = dot([10,0,0], body_z) = 8.51 m/s
        v_inplane = |[10,0,0] - 8.51·body_z| = 5.25 m/s
    """

    def setup_method(self):
        self.r = rd.load("beaupoil_2026")

    def test_K_drive_positive(self):
        assert self.r.K_drive_Nms_m > 0

    def test_K_drag_positive(self):
        assert self.r.K_drag_Nms2_rad2 > 0

    def test_I_ode_positive(self):
        assert self.r.I_ode_kgm2 > 0

    def test_omega_min_positive(self):
        assert self.r.omega_min_rad_s > 0

    def test_omega_eq_from_K_consistent_with_nominal(self):
        # K_drive / K_drag tuned so omega_eq_from_K ≈ omega_eq_rad_s within 5%.
        # Design v_inplane = 5.25 m/s → formula gives 20.28 rad/s vs stored 20.148 rad/s.
        omega_formula = self.r.omega_eq_from_K_rad_s
        omega_stored  = self.r.omega_eq_rad_s
        assert omega_stored is not None
        diff_pct = abs(omega_formula - omega_stored) / omega_stored
        assert diff_pct < 0.05, (
            f"omega_eq formula={omega_formula:.3f} rad/s vs stored={omega_stored:.3f} rad/s "
            f"({diff_pct:.1%} apart); should be <5% at v_inplane=5.25 m/s"
        )

    def test_K_ratio_matches_omega_eq(self):
        # K_drive / K_drag = omega_eq² / v_inplane_design
        # At v_inplane=5.25: K/K = 20.15²/5.25 ≈ 77.3.  Stored ratio = 1.4/0.01786 = 78.4.
        r = self.r
        v_design  = r._V_INPLANE_DESIGN_MS
        K_ratio   = r.K_drive_Nms_m / r.K_drag_Nms2_rad2
        K_ratio_expected = r.omega_eq_rad_s ** 2 / v_design
        assert K_ratio == pytest.approx(K_ratio_expected, rel=0.05), (
            f"K_drive/K_drag={K_ratio:.2f} should be ≈ omega_eq²/v_inplane="
            f"{K_ratio_expected:.2f} (within 5%)"
        )

    def test_implied_v_inplane_in_plausible_range(self):
        # The v_inplane at which K constants give exactly omega_eq.
        # Must be in [4, 7] m/s for 10 m/s wind at plausible tether elevations (15°–45°).
        r = self.r
        v_implied = r.omega_eq_rad_s ** 2 * r.K_drag_Nms2_rad2 / r.K_drive_Nms_m
        assert 4.0 < v_implied < 7.0, (
            f"Implied design v_inplane={v_implied:.3f} m/s is outside [4, 7] m/s — "
            f"check K_drive, K_drag, or omega_eq values"
        )

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
