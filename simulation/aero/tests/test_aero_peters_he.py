"""
test_aero_peters_he.py — Unit tests for PetersHeBEM internal components.

Tests the Peters-He dynamic inflow ODE and helper primitives directly,
bypassing compute_forces() so each component can be verified in isolation.

  TestRampFactor  — linear ramp 0 -> 1 over ramp_time seconds
  TestPrandtlF    — combined tip + root Prandtl loss factor (used inside PetersHeBEM)
  TestStripForces — full blade strip integration with explicit inflow states
  TestOdeStep     — implicit Euler step on the 5-state inflow ODE
  TestIsValid     — inflow state bounds check
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parents[2]))   # simulation/

from aero import rotor_definition as rd
from aero import PetersHeBEM
from frames import build_orb_frame


# ── Shared fixtures ───────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def rotor():
    return rd.load("beaupoil_2026")


@pytest.fixture(scope="module")
def bem(rotor):
    return PetersHeBEM(rotor)


@pytest.fixture(scope="module")
def R_horiz():
    """Rotation matrix for a horizontal disk (disk normal = NED up = [0,0,-1])."""
    return build_orb_frame(np.array([0.0, 0.0, -1.0]))


@pytest.fixture(scope="module")
def alpha_zero_rad(rotor):
    """Zero-lift AoA from the linear model: alpha_zero = -CL0 / CL_alpha."""
    return -rotor.CL0 / rotor.CL_alpha_per_rad


# ── TestRampFactor ────────────────────────────────────────────────────────────

class TestRampFactor:
    def test_zero_at_t0(self, bem):
        assert bem._ramp_factor(0.0) == pytest.approx(0.0)

    def test_one_at_ramp_time(self, bem):
        assert bem._ramp_factor(bem.ramp_time) == pytest.approx(1.0)

    def test_half_at_half_ramp(self, bem):
        assert bem._ramp_factor(bem.ramp_time / 2.0) == pytest.approx(0.5)

    def test_clamped_above_ramp_time(self, bem):
        assert bem._ramp_factor(bem.ramp_time * 10.0) == pytest.approx(1.0)

    def test_negative_t_clamps_to_zero(self, bem):
        assert bem._ramp_factor(-1.0) == pytest.approx(0.0)

    def test_monotone_over_ramp(self, bem):
        ts   = np.linspace(0.0, bem.ramp_time, 20)
        vals = [bem._ramp_factor(t) for t in ts]
        assert all(b >= a for a, b in zip(vals, vals[1:]))


# ── TestPrandtlF ──────────────────────────────────────────────────────────────

class TestPrandtlF:
    def test_near_zero_at_tip(self, bem):
        """Loss factor approaches zero near the blade tip."""
        F = bem._prandtl_F(bem.R_TIP - 0.001, math.radians(5.0))
        assert F < 0.20, f"F={F:.4f} at r~R_tip; expected < 0.20"

    def test_near_zero_at_root(self, bem):
        """Loss factor approaches zero near the blade root."""
        F = bem._prandtl_F(bem.R_ROOT + 0.001, math.radians(5.0))
        assert F < 0.30, f"F={F:.4f} at r~R_root; expected < 0.30"

    def test_near_one_at_midspan(self, bem):
        """Loss factor is close to 1 at midspan for moderate inflow angle."""
        F = bem._prandtl_F(bem.R_CP, math.radians(10.0))
        assert F > 0.80, f"F={F:.4f} at r=R_cp; expected > 0.80"

    def test_symmetric_in_phi(self, bem):
        """F(r, phi) == F(r, -phi) — uses abs(sin(phi))."""
        assert bem._prandtl_F(bem.R_CP, math.radians(8.0)) == pytest.approx(
            bem._prandtl_F(bem.R_CP, math.radians(-8.0))
        )

    def test_zero_phi_does_not_crash(self, bem):
        """Guard sin_phi >= 1e-4 prevents division by zero at phi=0."""
        F = bem._prandtl_F(bem.R_CP, 0.0)
        assert math.isfinite(F) and F >= 0.0

    def test_bounded_in_0_1(self, bem):
        """F is always in [0, 1]."""
        for r in np.linspace(bem.R_ROOT + 0.01, bem.R_TIP - 0.01, 10):
            for phi_deg in [1, 5, 10, 20, 45]:
                F = bem._prandtl_F(r, math.radians(phi_deg))
                assert 0.0 <= F <= 1.0, f"F={F} out of [0,1] at r={r:.2f} phi={phi_deg}"


# ── TestStripForces ───────────────────────────────────────────────────────────

class TestStripForces:
    """
    _strip_forces called with explicit inflow states (bypasses cold_start).
    All tests use a horizontal disk (R_hub = R_horiz) and stationary hub.
    """

    def _thrust(self, bem, R_hub, collective, omega=28.0,
                v0=0.0, v1c=0.0, v1s=0.0, v2c=0.0, v2s=0.0, v_rel=None):
        if v_rel is None:
            v_rel = np.zeros(3)
        disk_normal = R_hub[:, 2]
        F, M, _, _ = bem._strip_forces(
            v0, v1c, v1s, v2c, v2s,
            v_rel, disk_normal, R_hub, omega,
            collective, 0.0, 0.0, spin_angle=0.0,
        )
        return float(np.dot(F / bem.N_AZ, disk_normal))

    def test_zero_lift_at_alpha_zero_no_induction(self, bem, rotor, R_horiz, alpha_zero_rad):
        """At zero-lift collective (= -CL0/CL_alpha) with zero induction, T ~ 0."""
        T = self._thrust(bem, R_horiz, alpha_zero_rad, omega=28.0, v0=0.0)
        assert abs(T) < 5.0, f"T={T:.2f} N at zero-lift collective; expected |T| < 5 N"

    def test_positive_collective_gives_positive_thrust(self, bem, rotor, R_horiz):
        T = self._thrust(bem, R_horiz, math.radians(3.0), omega=28.0, v0=0.0)
        assert T > 0.0, f"T={T:.2f} N at col=+3 deg; expected positive"

    def test_negative_collective_gives_negative_thrust(self, bem, rotor, R_horiz):
        T = self._thrust(bem, R_horiz, math.radians(-8.0), omega=28.0, v0=0.0)
        assert T < 0.0, f"T={T:.2f} N at col=-8 deg; expected negative"

    def test_thrust_monotone_with_collective(self, bem, rotor, R_horiz):
        cols    = np.linspace(math.radians(-5.0), math.radians(5.0), 8)
        thrusts = [self._thrust(bem, R_horiz, c, v0=0.0) for c in cols]
        assert all(b > a for a, b in zip(thrusts, thrusts[1:])), \
            f"Thrust not monotone vs collective: {[round(t, 1) for t in thrusts]}"

    def test_thrust_scales_with_omega_squared(self, bem, rotor, R_horiz):
        """T proportional to omega^2 (Glauert momentum limit at low induction)."""
        col  = math.radians(2.0)
        T1   = self._thrust(bem, R_horiz, col, omega=20.0, v0=0.0)
        T2   = self._thrust(bem, R_horiz, col, omega=40.0, v0=0.0)
        ratio = T2 / T1
        assert 3.5 < ratio < 4.5, f"T(40)/T(20) = {ratio:.2f}; expected ~4"

    def test_induction_reduces_thrust(self, bem, rotor, R_horiz):
        """Positive v0 (upward induction) reduces AoA and therefore thrust."""
        col        = math.radians(5.0)
        T_no_ind   = self._thrust(bem, R_horiz, col, omega=28.0, v0=0.0)
        T_with_ind = self._thrust(bem, R_horiz, col, omega=28.0, v0=3.0)
        assert T_with_ind < T_no_ind, \
            f"Induction should reduce thrust: T(v0=0)={T_no_ind:.1f} T(v0=3)={T_with_ind:.1f}"

    def test_tilt_lon_changes_in_plane_force(self, bem, rotor, R_horiz):
        """Longitudinal cyclic tilts the force vector, changing the in-plane component."""
        disk_normal = R_horiz[:, 2]
        col  = math.radians(2.0)
        tilt = math.radians(2.0) * bem.pitch_gain_rad

        F_tilt, _, _, _ = bem._strip_forces(
            0.0, 0.0, 0.0, 0.0, 0.0,
            np.zeros(3), disk_normal, R_horiz, 28.0,
            col, tilt, 0.0, spin_angle=0.0,
        )
        F_zero, _, _, _ = bem._strip_forces(
            0.0, 0.0, 0.0, 0.0, 0.0,
            np.zeros(3), disk_normal, R_horiz, 28.0,
            col, 0.0, 0.0, spin_angle=0.0,
        )

        def in_plane_mag(F):
            axial = float(np.dot(F / bem.N_AZ, disk_normal)) * disk_normal
            return np.linalg.norm(F / bem.N_AZ - axial)

        assert abs(in_plane_mag(F_tilt) - in_plane_mag(F_zero)) > 0.01, \
            "Longitudinal cyclic should change in-plane force component"


# ── TestOdeStep ───────────────────────────────────────────────────────────────

class TestOdeStep:
    def test_zero_dt_leaves_states_unchanged(self, bem):
        b = PetersHeBEM(rd.load("beaupoil_2026"))
        b._v0, b._v1c, b._v1s = 1.0, 0.2, -0.3
        b._ode_step(T=50.0, Mx_disk=0.0, My_disk=0.0,
                    M2c_disk=0.0, M2s_disk=0.0,
                    v_axial=0.0, v_inplane=5.0, dt=0.0)
        assert b._v0  == pytest.approx(1.0)
        assert b._v1c == pytest.approx(0.2)
        assert b._v1s == pytest.approx(-0.3)

    def test_positive_thrust_drives_v0_positive(self, bem):
        b = PetersHeBEM(rd.load("beaupoil_2026"))
        b._v0 = 0.0
        b._ode_step(T=200.0, Mx_disk=0.0, My_disk=0.0,
                    M2c_disk=0.0, M2s_disk=0.0,
                    v_axial=0.0, v_inplane=10.0, dt=0.01)
        assert b._v0 > 0.0, f"v0={b._v0:.4f}; positive T should drive v0 > 0"

    def test_v0_converges_to_rankine_froude(self, bem):
        """In pure axial flow, v0 converges to Rankine-Froude momentum theory."""
        b = PetersHeBEM(rd.load("beaupoil_2026"))
        T = 250.0
        for _ in range(3000):
            b._ode_step(T=T, Mx_disk=0.0, My_disk=0.0,
                        M2c_disk=0.0, M2s_disk=0.0,
                        v_axial=0.0, v_inplane=0.0, dt=0.0025)
        v0_rf = T / (2.0 * b.RHO * b.disk_area * b._v0)
        assert b._v0 == pytest.approx(v0_rf, rel=0.01), \
            f"v0={b._v0:.4f} m/s, Rankine-Froude target={v0_rf:.4f} m/s"

    def test_pitching_moment_drives_v1c(self, bem):
        b = PetersHeBEM(rd.load("beaupoil_2026"))
        b._v0, b._v1c = 2.0, 0.0
        b._ode_step(T=0.0, Mx_disk=0.0, My_disk=10.0,
                    M2c_disk=0.0, M2s_disk=0.0,
                    v_axial=0.0, v_inplane=5.0, dt=0.1)
        assert b._v1c > 0.0, f"v1c={b._v1c:.5f}; My>0 should drive v1c > 0"

    def test_rolling_moment_drives_v1s_negative(self, bem):
        b = PetersHeBEM(rd.load("beaupoil_2026"))
        b._v0, b._v1s = 2.0, 0.0
        b._ode_step(T=0.0, Mx_disk=10.0, My_disk=0.0,
                    M2c_disk=0.0, M2s_disk=0.0,
                    v_axial=0.0, v_inplane=5.0, dt=0.1)
        assert b._v1s < 0.0, f"v1s={b._v1s:.5f}; Mx>0 should drive v1s < 0"

    def test_large_dt_does_not_diverge(self, bem):
        b = PetersHeBEM(rd.load("beaupoil_2026"))
        b._v0 = 1.0
        for _ in range(10):
            b._ode_step(T=200.0, Mx_disk=0.0, My_disk=0.0,
                        M2c_disk=0.0, M2s_disk=0.0,
                        v_axial=0.0, v_inplane=10.0, dt=1.0)
        assert math.isfinite(b._v0), "v0 diverged with large dt"
        assert abs(b._v0) < 50.0, f"v0={b._v0:.2f} m/s implausibly large after large dt"


# ── TestIsValid ───────────────────────────────────────────────────────────────

class TestIsValid:
    def test_fresh_object_is_valid(self, bem):
        assert PetersHeBEM(rd.load("beaupoil_2026")).is_valid()

    def test_large_v0_is_invalid(self, bem):
        b = PetersHeBEM(rd.load("beaupoil_2026"))
        b._v0 = PetersHeBEM.INFLOW_MAX_MS + 1.0
        assert not b.is_valid()

    def test_large_v1c_is_invalid(self, bem):
        b = PetersHeBEM(rd.load("beaupoil_2026"))
        b._v1c = PetersHeBEM.INFLOW_MAX_MS + 1.0
        assert not b.is_valid()

    def test_large_v1s_is_invalid(self, bem):
        b = PetersHeBEM(rd.load("beaupoil_2026"))
        b._v1s = -(PetersHeBEM.INFLOW_MAX_MS + 1.0)
        assert not b.is_valid()

    def test_nan_thrust_is_invalid(self, bem):
        b = PetersHeBEM(rd.load("beaupoil_2026"))
        b.last_T = float("nan")
        assert not b.is_valid()

    def test_borderline_state_is_valid(self, bem):
        b = PetersHeBEM(rd.load("beaupoil_2026"))
        b._v0 = PetersHeBEM.INFLOW_MAX_MS - 0.1
        assert b.is_valid()
