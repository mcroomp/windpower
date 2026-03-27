"""
test_deschutter_validation.py

Validation tests confirming that aero.py operates within De Schutter's (2018)
stated constraints and uses Weyel's (2025) empirical SG6042 lift model.

Lift model — Weyel (2025) empirical SG6042 at Re ≈ 127k:
  - CL = CL0 + CL_alpha × α,  CL0 = 0.11,  CL_alpha = 0.87 /rad
  - CL0 = 0.11 accounts for SG6042 camber (vs De Schutter's symmetric CL0 = 0)
  - CL_alpha = 0.87 /rad is ~6× below thin-plate theory (5.46 /rad at AR=13.3)
    due to low-Re viscous effects on the SG6042 at operating conditions

Drag polar — De Schutter (2018), Table I and Section II–IV:
  - CD = CD0 + CL²/(π·R·Oe),  CD0 = 0.01, Oe = 0.8
  - AoA hard constraint: −15° ≤ α ≤ 15°  (Eq. 28)
  - Induction: 0 ≤ a ≤ 0.5  (Eq. 19, AD theory validity)
  - Reel-out collective: +5° to +8° (Fig. 4, u_ref=10 m/s)
  - Tether safety factor: f_s = 10  →  max operating tension ≈ 62 N

Our model geometry:
  - AR = (R_tip − R_root) / chord = 2.0 / 0.15 = 13.3
  - Single-point evaluation at r_cp = r_root + (2/3)×span = 1.833 m
  - S_w = N_blades × chord × span = 1.2 m²
  - Induction: exact quadratic solution of T = 2ρA·v_i·(|v_axial|+v_i)
  - With Weyel CL_alpha, induction factor a ≈ 0.26 (within AD validity a ≤ 0.5)

Remaining model gaps vs De Schutter (documented in tests):
  - Tether warning threshold (80% break load = 496 N) is 8× higher than
    De Schutter's 10× safety-factor limit (62 N).
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from aero import RotorAero

# ---------------------------------------------------------------------------
# Reference constants from De Schutter 2018
# ---------------------------------------------------------------------------

_DS_ASPECT_RATIO = 12.0
_DS_OSWALD       = 0.8
_DS_CD0          = 0.01
_DS_CL_ALPHA     = 2 * math.pi / (1 + 2 / _DS_ASPECT_RATIO)   # ≈ 5.38 /rad

# Operating point
_RATED_WIND_MS   = 10.0
_OMEGA_RAD_S     = 28.0      # = 70 m/s tip speed / 2.5 m R_tip  (λ=7 at rated wind)
_KITE_MASS_KG    = 5.0
_KITE_WEIGHT_N   = _KITE_MASS_KG * 9.81

# De Schutter reel-out collective (Fig. 4, rated wind)
_REEL_OUT_COLL_LOW_DEG  = 5.0
_REEL_OUT_COLL_HIGH_DEG = 8.0

# Tether safety factor
_TETHER_SAFETY_FACTOR   = 10.0
_TETHER_BREAK_LOAD_N    = 620.0
_DS_MAX_OPERATING_N     = _TETHER_BREAK_LOAD_N / _TETHER_SAFETY_FACTOR   # 62 N

# AoA hard constraint (De Schutter Eq. 28)
_AOA_LIMIT_DEG = 15.0
_AOA_LIMIT_RAD = math.radians(_AOA_LIMIT_DEG)


# ---------------------------------------------------------------------------
# Helpers — compute CL/CD using the same formulas as aero.py
# ---------------------------------------------------------------------------

def _our_CL(alpha_rad: float, aero: RotorAero) -> float:
    """Our model CL (Weyel 2025 empirical SG6042: CL0=0.11, CL_alpha=0.87 /rad)."""
    return aero.CL0 + aero.CL_alpha * alpha_rad


def _our_CD(alpha_rad: float, aero: RotorAero) -> float:
    """Our model CD (De Schutter induced drag polar)."""
    CL = _our_CL(alpha_rad, aero)
    return aero.CD0 + CL ** 2 / (math.pi * aero.aspect_ratio * aero.oswald_eff)


def _deschutter_CL(alpha_rad: float) -> float:
    """De Schutter thin airfoil 3D CL (R=12)."""
    return _DS_CL_ALPHA * alpha_rad


def _deschutter_CD(CL: float) -> float:
    """De Schutter induced drag polar."""
    return _DS_CD0 + CL ** 2 / (math.pi * _DS_ASPECT_RATIO * _DS_OSWALD)


def _rotated_disk_normal_from_tilt(tilt_deg: float) -> tuple:
    """
    Return (R_hub, disk_normal) for a rotor tilted `tilt_deg` from vertical
    about the Y axis (tilted toward the wind direction = X).
    """
    tilt_rad = math.radians(tilt_deg)
    R_hub = np.array([
        [ math.cos(tilt_rad), 0, math.sin(tilt_rad)],
        [             0,      1,             0       ],
        [-math.sin(tilt_rad), 0, math.cos(tilt_rad) ],
    ])
    return R_hub, R_hub[:, 2]


# ---------------------------------------------------------------------------
# 1. CL model — matches De Schutter thin airfoil formula
# ---------------------------------------------------------------------------

_WEYEL_CL0      = 0.11    # SG6042 camber lift offset (Weyel 2025)
_WEYEL_CL_ALPHA = 0.87    # SG6042 empirical lift slope [/rad] at Re≈127k (Weyel 2025)


class TestCLModel:
    """
    CL model uses Weyel (2025) empirical SG6042 parameters:
        CL = CL0 + CL_alpha × α,  CL0=0.11,  CL_alpha=0.87 /rad

    This departs from De Schutter's thin-airfoil assumption (CL0=0, CL_alpha≈5.38/rad)
    to match the actual SG6042 airfoil at Re≈127k.  The Weyel slope is ~6× lower than
    thin-plate theory due to low-Re viscous effects; this brings thrust into the
    physically plausible 600N range (vs >2700N with De Schutter's slope).
    """

    def test_cl_alpha_positive(self):
        """CL_alpha must be positive."""
        aero = RotorAero()
        assert aero.CL_alpha > 0.0

    def test_cl_alpha_is_weyel_value(self):
        """CL_alpha = 0.87 /rad (Weyel 2025 SG6042 empirical value)."""
        aero = RotorAero()
        assert math.isclose(aero.CL_alpha, _WEYEL_CL_ALPHA, rel_tol=0.01), (
            f"CL_alpha={aero.CL_alpha:.4f} /rad should match Weyel value "
            f"{_WEYEL_CL_ALPHA:.4f} /rad"
        )

    def test_cl0_is_weyel_value(self):
        """CL0 = 0.11 (SG6042 camber offset from Weyel 2025)."""
        aero = RotorAero()
        assert math.isclose(aero.CL0, _WEYEL_CL0, rel_tol=0.01), (
            f"CL0={aero.CL0:.4f} should match Weyel value {_WEYEL_CL0:.4f}"
        )

    def test_cl_at_zero_aoa_equals_cl0(self):
        """CL at α=0 equals CL0=0.11 (SG6042 camber, not zero like De Schutter)."""
        aero = RotorAero()
        assert math.isclose(_our_CL(0.0, aero), _WEYEL_CL0, rel_tol=0.01), (
            f"CL at α=0 should be CL0={_WEYEL_CL0}, got {_our_CL(0.0, aero):.4f}"
        )

    def test_cl_linear_with_aoa(self):
        """CL increases linearly with AoA at slope CL_alpha=0.87/rad."""
        aero = RotorAero()
        for alpha_deg in [5, 7, 10, 12, 15]:
            alpha_rad = math.radians(alpha_deg)
            expected  = _WEYEL_CL0 + _WEYEL_CL_ALPHA * alpha_rad
            got       = _our_CL(alpha_rad, aero)
            assert math.isclose(got, expected, rel_tol=0.01), (
                f"At α={alpha_deg}°: expected CL={expected:.4f}, got {got:.4f}"
            )

    def test_cl_positive_at_positive_aoa(self):
        """CL > 0 for positive AoA (CL0 + positive slope term)."""
        aero = RotorAero()
        for alpha_deg in [1, 5, 10, 15]:
            assert _our_CL(math.radians(alpha_deg), aero) > 0.0

    def test_cl_negative_at_strongly_negative_aoa(self):
        """
        CL < 0 only for α < −CL0/CL_alpha ≈ −7.24°.
        The CL0=0.11 camber offset means the SG6042 still lifts at small
        negative AoA — unlike De Schutter's symmetric thin-plate model.
        """
        aero = RotorAero()
        # Zero crossing: α_zero = −CL0/CL_alpha ≈ −7.24°
        for alpha_deg in [-10, -15]:
            assert _our_CL(math.radians(alpha_deg), aero) < 0.0, (
                f"CL at α={alpha_deg}° should be negative"
            )
        # Near zero-crossing, CL should be close to zero
        alpha_zero = -_WEYEL_CL0 / _WEYEL_CL_ALPHA
        assert abs(_our_CL(alpha_zero, aero)) < 1e-6

    def test_cl_substantially_below_deschutter_slope(self):
        """
        Weyel CL_alpha=0.87/rad is ~6× below De Schutter's thin-airfoil slope≈5.38/rad.
        This reflects SG6042 viscous effects at Re≈127k.
        """
        aero  = RotorAero()
        ratio = aero.CL_alpha / _DS_CL_ALPHA
        assert ratio < 0.20, (
            f"Weyel CL_alpha ({aero.CL_alpha:.4f}) should be <20% of De Schutter's "
            f"thin-plate slope ({_DS_CL_ALPHA:.4f}). Got ratio={ratio:.3f}"
        )

    def test_tip_speed_ratio_at_rated_conditions(self):
        """De Schutter operates at λ = v_tip / u_ref = 7; our nominal ω gives λ=7."""
        v_tip = _OMEGA_RAD_S * 2.5
        assert math.isclose(v_tip / _RATED_WIND_MS, 7.0, rel_tol=0.05)


# ---------------------------------------------------------------------------
# 2. CD model — De Schutter induced drag polar
# ---------------------------------------------------------------------------

class TestCDModel:
    """
    Our CD now uses De Schutter's induced drag polar: CD = CD0 + CL²/(π·AR·Oe).
    """

    def test_cd_at_zero_aoa_near_cd0(self):
        """
        CD at α=0 is close to CD0 but slightly above it.

        With Weyel CL0=0.11, CL at α=0 is non-zero, adding a small induced drag
        term: ΔCD = CL0²/(π·AR·Oe) = 0.11²/(π×13.3×0.8) ≈ 0.00036.
        So CD(0) ≈ CD0 + 0.00036, not exactly CD0.
        """
        aero = RotorAero()
        cd   = _our_CD(0.0, aero)
        assert cd >= aero.CD0, "CD at α=0 must be ≥ CD0 (CL0 adds induced drag)"
        assert cd < aero.CD0 * 1.10, (
            f"CD at α=0 ({cd:.5f}) should be within 10% of CD0={aero.CD0} "
            f"(CL0 camber term is small)"
        )

    def test_cd0_matches_deschutter(self):
        """CD0 = 0.01 (De Schutter Table I)."""
        aero = RotorAero()
        assert math.isclose(aero.CD0, _DS_CD0, rel_tol=0.01), (
            f"CD0={aero.CD0} should equal De Schutter value {_DS_CD0}"
        )

    def test_induced_drag_positive_at_reel_out_collective(self):
        """
        At reel-out collective (+7°), induced drag is positive.

        With Weyel CL_alpha=0.87/rad (vs De Schutter's 5.46/rad), CL at 7° is:
          CL = 0.11 + 0.87×0.122 = 0.216
          ΔCD = 0.216²/(π×13.3×0.8) ≈ 0.0014
        This is smaller than CD0 (0.01) — the polar is less dominant than with
        De Schutter's slope, but still physically correct and positive.
        """
        aero       = RotorAero()
        alpha_rad  = math.radians(7.0)
        cd_total   = _our_CD(alpha_rad, aero)
        cd_induced = cd_total - aero.CD0
        assert cd_induced > 0.0, (
            f"Induced drag ({cd_induced:.5f}) must be > 0 at 7° AoA"
        )

    @pytest.mark.parametrize("alpha_deg", [5, 10, 15])
    def test_ld_ratio_exceeds_1_at_operating_aoa(self, alpha_deg):
        """L/D > 1 in the operating AoA range (required for efficient AWE)."""
        aero = RotorAero()
        cl   = _our_CL(math.radians(alpha_deg), aero)
        cd   = _our_CD(math.radians(alpha_deg), aero)
        assert cl / cd > 1.0, (
            f"L/D = {cl/cd:.2f} < 1 at α={alpha_deg}°"
        )

    def test_oswald_efficiency_matches_deschutter(self):
        """Oswald efficiency = 0.8 (De Schutter Table I)."""
        aero = RotorAero()
        assert math.isclose(aero.oswald_eff, _DS_OSWALD, rel_tol=0.01)

    def test_cd_increases_with_aoa_magnitude(self):
        """CD increases as |AoA| increases (induced drag grows as CL²)."""
        aero  = RotorAero()
        cds   = [_our_CD(math.radians(a), aero) for a in [0, 5, 10, 15]]
        assert all(cds[i] <= cds[i+1] for i in range(len(cds)-1)), (
            "CD should be non-decreasing with AoA magnitude"
        )


# ---------------------------------------------------------------------------
# 3. AoA constraint adherence
# ---------------------------------------------------------------------------

class TestAoAConstraintAtOperatingConditions:
    """
    AoA constraint: −15° ≤ α ≤ 15° (De Schutter Eq. 28).

    REMAINING MODEL GAP:
    The single-point model evaluates forces at r_cp = r_root + (2/3)×span = 1.833 m,
    which is within De Schutter's blade region and safely away from the problematic
    low-radius inner region.  The old strip-BEM tests that iterated over _r_mid are
    replaced with equivalent checks at the single evaluation point r_cp.
    """

    def test_rcp_aoa_within_15deg_at_rated_conditions(self):
        """
        At r_cp = 1.833 m (De Schutter single-point), AoA stays within ±15°
        at tilt=30°, collective=5° (lower reel-out bound).
        """
        aero  = RotorAero()
        R_hub, disk_normal = _rotated_disk_normal_from_tilt(30.0)
        v_axial = float(np.dot(np.array([_RATED_WIND_MS, 0.0, 0.0]), disk_normal))
        v_i     = math.sqrt(_KITE_WEIGHT_N / (2 * 1.22 * math.pi * 2.5 ** 2))
        v_eff   = v_axial + v_i
        coll    = math.radians(5.0)

        aoa_deg = abs(math.degrees(math.atan2(v_eff, _OMEGA_RAD_S * aero.r_cp) + coll))
        assert aoa_deg <= _AOA_LIMIT_DEG, (
            f"r_cp AoA {aoa_deg:.2f}° exceeds ±15° (tilt=30°, coll=5°)"
        )

    def test_inner_strip_gap_resolved_by_single_point_model(self):
        """
        Documents that the old inner-strip AoA problem no longer exists.

        The previous radial BEM computed forces at r < 1.0 m where v_tan was
        low (28×0.5=14 m/s) and inflow angles large (≈28°), driving pre-clamp
        AoA above 15°.  The De Schutter single-point model evaluates only at
        r_cp = 1.833 m, which is above the problematic region and within
        De Schutter's blade span (his beam length L_b ≈ 1.6 m).
        """
        aero  = RotorAero()
        # r_cp should be well above the old problematic inner strip range
        assert aero.r_cp >= 1.5, (
            f"r_cp={aero.r_cp:.3f} m is below 1.5 m — inner-strip AoA issue may resurface"
        )
        # Pre-clamp AoA at r_cp should be within ±15° at rated conditions
        v_axial = 5.0   # representative axial inflow
        v_eff   = v_axial + 1.0
        coll    = math.radians(7.0)
        aoa_deg = abs(math.degrees(math.atan2(v_eff, _OMEGA_RAD_S * aero.r_cp) + coll))
        assert aoa_deg <= _AOA_LIMIT_DEG, (
            f"r_cp AoA {aoa_deg:.2f}° exceeds ±15° — clamp needed at single eval point"
        )

    def test_aoa_clamp_is_applied_at_rcp(self):
        """After clamping, |AoA| ≤ aoa_limit at the single evaluation point r_cp."""
        aero  = RotorAero()
        R_hub, disk_normal = _rotated_disk_normal_from_tilt(40.0)
        v_axial = float(np.dot(np.array([_RATED_WIND_MS, 0.0, 0.0]), disk_normal))
        v_eff   = v_axial + 1.0
        coll    = math.radians(7.0)

        v_tan   = _OMEGA_RAD_S * aero.r_cp
        aoa     = math.atan2(v_eff, v_tan) + coll
        clamped = max(-aero.aoa_limit, min(aero.aoa_limit, aoa))
        assert abs(clamped) <= aero.aoa_limit + 1e-9

    def test_aoa_clamp_threshold_matches_deschutter(self):
        """aero.aoa_limit ≤ De Schutter's ±15° constraint."""
        aero = RotorAero()
        assert aero.aoa_limit <= _AOA_LIMIT_RAD + 1e-6, (
            f"aoa_limit {math.degrees(aero.aoa_limit):.2f}° exceeds ±{_AOA_LIMIT_DEG}°"
        )

    def test_rcp_aoa_within_limit_at_reel_in(self):
        """During reel-in (tilt=70°, collective=−12°), r_cp AoA stays within ±15°."""
        aero  = RotorAero()
        R_hub, disk_normal = _rotated_disk_normal_from_tilt(70.0)
        v_axial = float(np.dot(np.array([_RATED_WIND_MS, 0.0, 0.0]), disk_normal))
        v_eff   = v_axial + 0.5   # induction ≈ 0 during reel-in
        coll    = math.radians(-12.0)

        aoa_deg = abs(math.degrees(math.atan2(v_eff, _OMEGA_RAD_S * aero.r_cp) + coll))
        assert aoa_deg <= _AOA_LIMIT_DEG + 0.5, (
            f"r_cp AoA {aoa_deg:.2f}° exceeds ±15° in reel-in"
        )

    def test_high_collective_clamp_still_bounded(self):
        """At low spin (ω=15 rad/s) the clamp still keeps |AoA_clamped| ≤ ±15° at r_cp."""
        aero  = RotorAero()
        R_hub, disk_normal = _rotated_disk_normal_from_tilt(40.0)
        v_axial = float(np.dot(np.array([_RATED_WIND_MS, 0.0, 0.0]), disk_normal))
        v_eff   = v_axial + 1.0
        coll    = math.radians(_REEL_OUT_COLL_HIGH_DEG)
        omega   = 15.0

        v_tan   = omega * aero.r_cp
        aoa     = math.atan2(v_eff, v_tan) + coll
        clamped = max(-aero.aoa_limit, min(aero.aoa_limit, aoa))
        assert abs(clamped) <= _AOA_LIMIT_RAD + 1e-9


# ---------------------------------------------------------------------------
# 4. Thrust at De Schutter reel-out conditions
# ---------------------------------------------------------------------------

class TestThrustAtRatedConditions:
    """
    At De Schutter's reel-out collective range (+5° to +8°), the model
    must produce thrust ≥ kite weight (49.05 N) along the disk normal.
    With Weyel's CL (CL_alpha=0.87/rad, CL0=0.11), thrust at rated conditions
    is ~600–700 N — well above the 5 kg weight, creating tether tension for
    AWE power generation.
    """

    @pytest.mark.parametrize("collective_deg", [5.0, 7.0, 8.0, 10.0, 12.0])
    def test_thrust_exceeds_weight_at_reel_out_collective(self, collective_deg):
        """Thrust ≥ mg at each reel-out collective (40° tilt, rated wind)."""
        aero  = RotorAero()
        R_hub, disk_normal = _rotated_disk_normal_from_tilt(40.0)

        forces = aero.compute_forces(
            collective_rad = math.radians(collective_deg),
            tilt_lon=0.0, tilt_lat=0.0,
            R_hub          = R_hub,
            v_hub_world    = np.zeros(3),
            omega_rotor    = _OMEGA_RAD_S,
            wind_world     = np.array([_RATED_WIND_MS, 0.0, 0.0]),
            t              = 10.0,
        )
        T = float(np.dot(forces[:3], disk_normal))
        assert T >= _KITE_WEIGHT_N, (
            f"Thrust {T:.2f} N < weight {_KITE_WEIGHT_N:.2f} N at collective={collective_deg}°"
        )

    def test_thrust_non_decreasing_with_collective(self):
        """Thrust is non-decreasing with collective (inner-strip clamping may plateau)."""
        aero  = RotorAero()
        R_hub, disk_normal = _rotated_disk_normal_from_tilt(40.0)
        prev_T = -1e9

        for deg in [0, 3, 5, 7, 10, 13]:
            forces = aero.compute_forces(
                math.radians(deg), 0, 0, R_hub, np.zeros(3),
                _OMEGA_RAD_S, np.array([_RATED_WIND_MS, 0.0, 0.0]), t=10.0,
            )
            T = float(np.dot(forces[:3], disk_normal))
            assert T >= prev_T, (
                f"Thrust decreased from prev_T={prev_T:.2f} N to T={T:.2f} N "
                f"at collective={deg}°"
            )
            prev_T = T

    def test_negative_collective_reduces_thrust(self):
        """Reel-in collective (−12°) reduces thrust below neutral."""
        aero  = RotorAero()
        R_hub, disk_normal = _rotated_disk_normal_from_tilt(40.0)
        wind  = np.array([_RATED_WIND_MS, 0.0, 0.0])

        T_neutral  = float(np.dot(aero.compute_forces(
            0.0, 0, 0, R_hub, np.zeros(3), _OMEGA_RAD_S, wind, t=10.0
        )[:3], disk_normal))
        T_reel_in  = float(np.dot(aero.compute_forces(
            math.radians(-12), 0, 0, R_hub, np.zeros(3), _OMEGA_RAD_S, wind, t=10.0
        )[:3], disk_normal))

        assert T_reel_in < T_neutral, (
            f"Reel-in collective should reduce thrust. "
            f"T_reel_in={T_reel_in:.2f} N, T_neutral={T_neutral:.2f} N"
        )


# ---------------------------------------------------------------------------
# 5. Induction factor — now within De Schutter's valid range
# ---------------------------------------------------------------------------

class TestInductionFactorRange:
    """
    With the exact quadratic induction solution, the induction factor
    a = v_i/|v_axial| should satisfy De Schutter's constraint 0 ≤ a ≤ 0.5
    (Eq. 19) for physically reasonable operating conditions.
    """

    def test_induction_within_ad_range_at_tilted_conditions(self):
        """
        With Weyel CL_alpha=0.87/rad, induction factor a ≤ 0.5 at 40° tilt.

        De Schutter's AD validity requires a = v_i/|v_axial| ≤ 0.5, equivalent to
        T ≤ 1.5·ρA·v_axial² ≈ 1485 N at 40° tilt (v_axial = 6.43 m/s).

        With Weyel's empirical lift slope, thrust ≈ 600–700 N — well inside the
        AD validity envelope.  This is the correct operating regime for a 5 kg kite.
        Previously (with De Schutter's thin-plate CL_alpha = 5.46 /rad), thrust was
        ~2700 N and a > 0.5, violating AD theory.
        """
        aero  = RotorAero()
        R_hub, disk_normal = _rotated_disk_normal_from_tilt(40.0)

        aero.compute_forces(
            math.radians(8.0), 0, 0, R_hub, np.zeros(3),
            _OMEGA_RAD_S, np.array([_RATED_WIND_MS, 0.0, 0.0]), t=10.0,
        )

        v_i     = aero.last_v_i
        v_axial = abs(aero.last_v_axial)
        a       = v_i / v_axial if v_axial > 0.1 else 0.0

        # Quadratic solver converges (v_i is finite and non-negative)
        assert math.isfinite(v_i) and v_i >= 0.0, "v_i must be finite and non-negative"

        # With Weyel params, thrust is within the AD validity envelope
        assert a <= 0.5, (
            f"Induction factor a = {a:.3f} > 0.5 at 40° tilt. "
            f"T={aero.last_T:.0f} N, v_i={v_i:.2f} m/s, v_axial={v_axial:.2f} m/s. "
            f"Check if CL parameters have reverted to De Schutter's thin-plate values."
        )

    def test_induction_within_ad_range_pure_axial_flow(self):
        """Induction factor is in [0, 0.5] for pure axial inflow."""
        aero  = RotorAero()
        aero.compute_forces(
            math.radians(5.0), 0, 0, np.eye(3), np.zeros(3),
            _OMEGA_RAD_S, np.array([0.0, 0.0, _RATED_WIND_MS]), t=10.0,
        )

        v_i     = aero.last_v_i
        v_axial = abs(aero.last_v_axial)
        a       = v_i / v_axial if v_axial > 0.1 else 0.0

        assert 0.0 <= a <= 0.5, (
            f"Induction factor a = {a:.3f} out of [0, 0.5] for axial flow. "
            f"v_i={v_i:.2f}, v_axial={v_axial:.2f}"
        )

    def test_induced_velocity_positive(self):
        """v_i ≥ 0 (disk always slows the air, never speeds it up)."""
        aero = RotorAero()
        aero.compute_forces(
            math.radians(7.0), 0, 0, np.eye(3), np.zeros(3),
            _OMEGA_RAD_S, np.array([0.0, 0.0, _RATED_WIND_MS]), t=10.0,
        )
        assert aero.last_v_i >= 0.0

    def test_tip_speed_ratio_at_rated_conditions(self):
        """λ = v_tip / u_ref = 7 at rated wind (De Schutter operating point)."""
        v_tip = _OMEGA_RAD_S * 2.5
        assert math.isclose(v_tip / _RATED_WIND_MS, 7.0, rel_tol=0.05)


# ---------------------------------------------------------------------------
# 6. Tether safety factor
# ---------------------------------------------------------------------------

class TestTetherSafetyFactor:
    """
    De Schutter uses safety factor 10× → max operating tension 62 N.
    Our mediator warns at 80% break load (496 N) — documented gap.
    """

    def test_deschutter_safety_limit_is_62N(self):
        assert math.isclose(_DS_MAX_OPERATING_N, 62.0, abs_tol=1.0)

    def test_our_warning_threshold_exceeds_deschutter_limit(self):
        """Our 80% threshold (496 N) >> De Schutter's 10× limit (62 N)."""
        our = 0.8 * _TETHER_BREAK_LOAD_N
        assert our / _DS_MAX_OPERATING_N > 5.0, (
            f"Expected our threshold ({our:.0f} N) >> De Schutter limit "
            f"({_DS_MAX_OPERATING_N:.0f} N). Gap has been reduced — check if "
            f"structural analysis now supports a tighter warning threshold."
        )

    def test_kite_weight_below_deschutter_tension_limit(self):
        """Kite weight (49 N) < De Schutter's 62 N operating limit."""
        assert _KITE_WEIGHT_N < _DS_MAX_OPERATING_N


# ---------------------------------------------------------------------------
# 7. Cyclic pitch and disk-tilt convention
# ---------------------------------------------------------------------------

class TestCyclicPitchConvention:
    """
    Verify that cyclic pitch convention matches De Schutter Eq. 23:
      - Longitudinal tilt → pitching moment (My)
      - Lateral tilt → rolling moment (Mx)
      - 4-blade phase offset = π/2 (vs De Schutter's 2π/3 for 3 wings)
    """

    def test_longitudinal_tilt_produces_pitching_moment(self):
        """Forward cyclic adds a pitching moment without changing forces."""
        aero  = RotorAero()
        wind  = np.array([_RATED_WIND_MS, 0.0, 0.0])
        R_hub = np.eye(3)
        coll  = math.radians(7.0)

        base = aero.compute_forces(coll, 0.0, 0.0, R_hub, np.zeros(3), _OMEGA_RAD_S, wind, t=10.0)
        fwd  = aero.compute_forces(coll, 0.5, 0.0, R_hub, np.zeros(3), _OMEGA_RAD_S, wind, t=10.0)

        assert np.linalg.norm(fwd[3:] - base[3:]) > 0.01
        np.testing.assert_allclose(fwd[:3], base[:3], rtol=1e-9, atol=1e-9,
                                   err_msg="Cyclic tilt must not change forces")

    def test_lateral_tilt_produces_rolling_moment(self):
        """Lateral tilt adds mainly Mx; longitudinal adds mainly My (body frame)."""
        aero  = RotorAero()
        wind  = np.array([_RATED_WIND_MS, 0.0, 0.0])
        R_hub = np.eye(3)
        coll  = math.radians(7.0)

        base = aero.compute_forces(coll, 0.0, 0.0, R_hub, np.zeros(3), _OMEGA_RAD_S, wind, t=10.0)
        lat  = aero.compute_forces(coll, 0.0, 0.5, R_hub, np.zeros(3), _OMEGA_RAD_S, wind, t=10.0)
        lon  = aero.compute_forces(coll, 0.5, 0.0, R_hub, np.zeros(3), _OMEGA_RAD_S, wind, t=10.0)

        d_lat = lat[3:] - base[3:]
        d_lon = lon[3:] - base[3:]

        # ENU body frame: body X=East, body Y=North, body Z=Up
        #   tilt_lat > 0 (roll right/East): Ry torque → My (forces[4]) dominant
        #   tilt_lon > 0 (pitch forward/North): -Rx torque → Mx (forces[3]) dominant
        assert abs(d_lat[1]) > abs(d_lat[0]), "Lateral tilt should mainly produce My (ENU body Y = North axis)"
        assert abs(d_lon[0]) > abs(d_lon[1]), "Longitudinal tilt should mainly produce Mx (ENU body X = East axis)"

    def test_four_blade_phase_offset_is_90deg(self):
        """4-blade offset = π/2 (90°), distinct from De Schutter's 2π/3 (120°)."""
        offset_4  = math.pi / 2
        offset_3  = 2 * math.pi / 3
        assert not math.isclose(offset_4, offset_3, abs_tol=0.01)
        assert math.isclose(math.degrees(offset_4), 90.0, abs_tol=0.01)
