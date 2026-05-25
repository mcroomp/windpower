"""test_rotor_definition.py — RotorDefinition (new aero package).

The new RotorDefinition is nested (blade / airfoil / inertia / control /
autorotation sub-records).  These tests verify YAML loading, derived
geometry properties, and validation for the rotors used by this project.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from dynbem import rotor_definition as rd
from tests.unit._aero_probe import load_rotor


def _errors(issues):
    return [i for i in issues if i.level == "ERROR"]


# ── Loading ─────────────────────────────────────────────────────────────────


class TestLoad:
    def test_load_beaupoil_by_path(self):
        r = load_rotor("beaupoil_2026")
        assert r.name == "beaupoil_2026"
        assert r.blade.n_blades == 4

    def test_load_de_schutter_by_path(self):
        r = load_rotor("de_schutter_2018")
        assert r.name == "de_schutter_2018"

    def test_load_unknown_path_raises(self):
        with pytest.raises(FileNotFoundError):
            rd.load("e:/does/not/exist.yaml")


# ── Beaupoil geometry ───────────────────────────────────────────────────────


class TestBeaupoilGeometry:
    def setup_method(self):
        self.r = load_rotor("beaupoil_2026")

    def test_r_cp_m(self):
        # r_cp = r_root + 2/3 * span = 0.5 + 2/3 * 2.0 = 1.833 m
        assert self.r.blade.r_cp_m == pytest.approx(0.5 + 2/3 * 2.0, rel=1e-6)

    def test_S_w_m2(self):
        # S_w = N * c * span = 4 * 0.20 * 2.0 = 1.60 m^2
        b = self.r.blade
        assert b.n_blades * b.chord_m * b.span_m == pytest.approx(4 * 0.20 * 2.0)

    def test_disk_area_m2(self):
        # A = pi(R^2 - r_root^2) = pi * 6.0 ~ 18.85 m^2
        expected = math.pi * (2.5**2 - 0.5**2)
        assert self.r.blade.disk_area_m2 == pytest.approx(expected, rel=1e-6)

    def test_aspect_ratio(self):
        # AR = span / chord = 2.0 / 0.20 = 10.0
        b = self.r.blade
        assert b.span_m / b.chord_m == pytest.approx(10.0, rel=1e-6)

    def test_solidity(self):
        # sigma = N * c / (pi * R) = 4 * 0.20 / (pi * 2.5)
        expected = 4 * 0.20 / (math.pi * 2.5)
        assert self.r.blade.solidity == pytest.approx(expected, rel=1e-6)


# ── De Schutter geometry ─────────────────────────────────────────────────────


class TestDeSchutterGeometry:
    """Verified against Table I of De Schutter et al. (2018)."""

    def setup_method(self):
        self.r = load_rotor("de_schutter_2018")

    def test_aspect_ratio(self):
        # AR = 1.5 / 0.125 = 12
        b = self.r.blade
        assert b.span_m / b.chord_m == pytest.approx(12.0, rel=1e-6)

    def test_kaman_disabled(self):
        if self.r.control is not None:
            # In dynbem 0.2.0, KamanFlap.enabled was removed; disabled is
            # indicated by all key fields being None (chord_fraction, etc.)
            kf = self.r.control.kaman_flap
            if kf is not None:
                assert kf.chord_fraction is None, (
                    "Expected kaman_flap to be disabled (chord_fraction=None)"
                )


# ── Validation ──────────────────────────────────────────────────────────────


class TestValidation:
    def test_beaupoil_no_errors(self):
        issues = load_rotor("beaupoil_2026").validate()
        assert _errors(issues) == [], f"Unexpected errors: {_errors(issues)}"

    def test_de_schutter_no_errors(self):
        issues = load_rotor("de_schutter_2018").validate()
        assert _errors(issues) == [], f"Unexpected errors: {_errors(issues)}"


# ── Inertia / control wiring ───────────────────────────────────────────────


class TestInertiaControl:
    def test_beaupoil_mass_resolved_from_components(self):
        """mass_kg must be auto-resolved from blade+stationary+shell sums."""
        r = load_rotor("beaupoil_2026")
        assert r.inertia.mass_kg is not None and r.inertia.mass_kg > 0

    def test_beaupoil_I_body_present(self):
        r = load_rotor("beaupoil_2026")
        assert len(r.inertia.I_body_kgm2) == 3
        assert all(v > 0 for v in r.inertia.I_body_kgm2)

    def test_beaupoil_control_has_swashplate_gain(self):
        r = load_rotor("beaupoil_2026")
        assert r.control is not None
        assert r.control.swashplate_pitch_gain_rad > 0

    def test_beaupoil_autorotation_I_ode(self):
        r = load_rotor("beaupoil_2026")
        assert r.autorotation.I_ode_kgm2 is not None
        assert r.autorotation.I_ode_kgm2 > 0


# ── Kaman flap sub-record ──────────────────────────────────────────────────


class TestKamanFlap:
    def test_beaupoil_kaman_enabled(self):
        r = load_rotor("beaupoil_2026")
        assert r.control is not None
        # dynbem 0.2.0: no .enabled field; enabled = chord_fraction is not None
        assert r.control.kaman_flap is not None
        assert r.control.kaman_flap.chord_fraction is not None

    def test_de_schutter_kaman_disabled(self):
        r = load_rotor("de_schutter_2018")
        if r.control is not None:
            # disabled = kaman_flap absent or chord_fraction is None
            kf = r.control.kaman_flap
            if kf is not None:
                assert kf.chord_fraction is None
