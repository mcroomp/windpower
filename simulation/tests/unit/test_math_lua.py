"""
test_lua_math.py  --  Unit tests for rawes.lua math via the actual Lua runtime.

Runs rawes.lua under lupa (Lua 5.4) using mock ArduPilot bindings.
Tests call into the real Lua functions via the _rawes_fns test surface,
so any change to rawes.lua is immediately reflected here -- no Python
transcription to drift out of sync.

Cross-checks compare Lua output against controller.py equivalents to catch
divergence when either side is updated.

No SITL, no Docker.  Runs with the existing unit-test venv.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from controller import (
    compute_rate_cmd,
    compute_bz_altitude_hold,
)
from rawes_lua_harness import RawesLua

# ── Module-level harness ──────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def sim():
    s = RawesLua()
    s.R = np.eye(3)
    return s


# ── Helpers ───────────────────────────────────────────────────────────────────

def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def _rot_z(angle):
    """3x3 rotation matrix around NED Z (yaw)."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def _build_R(body_z):
    """Rotation matrix body→NED whose third column is body_z."""
    bz = _unit(body_z)
    ref = np.array([0., 1., 0.]) if abs(bz[1]) < 0.9 else np.array([1., 0., 0.])
    bx = np.cross(ref, bz);  bx /= np.linalg.norm(bx)
    by = np.cross(bz, bx);   by /= np.linalg.norm(by)
    return np.column_stack([bx, by, bz])


def _py_pwm(rate_rads, rate_max_deg=360.0):
    """Python controller PWM formula (reference for cross-check)."""
    max_rate = math.radians(rate_max_deg)
    return int(round(1500.0 + 500.0 * max(-1.0, min(1.0, rate_rads / max_rate))))


# ── Constants sanity ─────────────────────────────────────────────────────────

def test_constants_have_expected_values(sim):
    """Key rawes.lua constants match the values documented in CLAUDE.md."""
    f = sim.fns
    assert float(f.ACRO_RP_RATE_DEG)      == pytest.approx(360.0)
    assert float(f.COL_CRUISE_FLIGHT_RAD) == pytest.approx(-0.18)
    assert float(f.COL_MIN_RAD)           == pytest.approx(-0.28)
    assert float(f.COL_MAX_RAD)           == pytest.approx(0.10)
    assert float(f.COL_REEL_OUT)          == pytest.approx(-0.20)
    assert float(f.T_PUMP_TRANSITION)     == pytest.approx(3.7)
    assert float(f.KP_VZ)                 == pytest.approx(0.05)
    assert float(f.MASS_KG)               == pytest.approx(5.0)
    assert float(f.G_ACCEL)               == pytest.approx(9.81)
    assert float(f.KP_TEN)               == pytest.approx(2e-4)
    assert float(f.KI_TEN)               == pytest.approx(1e-3)
    assert float(f.COL_MAX_TEN)          == pytest.approx(0.10)


# ── bz_altitude_hold ─────────────────────────────────────────────────────────

class TestBzAltitudeHold:
    """
    Tests for bz_altitude_hold(pos, el_rad, tension_n).

    Lua equivalent of Python compute_bz_altitude_hold:
      - points the disk at (el_rad, azimuth-from-pos) in NED
      - adds a gravity-compensation tilt: raw = tdir + (mass*g*cos(el)/tension) * e_up
      - normalises to a unit vector
    """

    def test_output_is_unit_vector(self, sim):
        """Result is always a unit vector."""
        pos = sim.lua_vec(10.0, 5.0, -8.0)
        r   = sim.fns.bz_altitude_hold(pos, 0.3, 200.0)
        rv  = sim.vec_to_list(r)
        assert math.sqrt(sum(x**2 for x in rv)) == pytest.approx(1.0, abs=1e-12)

    def test_vertical_tether_points_up(self, sim):
        """At el=pi/2 (vertical), body_z should point nearly straight up (NED z ~ -1)."""
        pos = sim.lua_vec(0.0, 0.0, -50.0)   # directly above anchor
        r   = sim.fns.bz_altitude_hold(pos, math.pi / 2, 300.0)
        rv  = sim.vec_to_list(r)
        # At el=90 deg, tdir = [0,0,-1] (up in NED), gravity comp = 0 → result = [0,0,-1]
        assert rv[0] == pytest.approx(0.0, abs=1e-6)
        assert rv[1] == pytest.approx(0.0, abs=1e-6)
        assert rv[2] == pytest.approx(-1.0, abs=1e-6)

    def test_azimuth_from_position(self, sim):
        """Changing azimuth of pos rotates the output body_z horizontally."""
        el = 0.3
        r  = 30.0
        alt = r * math.sin(el)
        # North position → body_z should have X-dominant horizontal component
        pos_n = sim.lua_vec(r * math.cos(el), 0.0, -alt)
        rv_n  = sim.vec_to_list(sim.fns.bz_altitude_hold(pos_n, el, 200.0))
        # East position → body_z should have Y-dominant horizontal component
        pos_e = sim.lua_vec(0.0, r * math.cos(el), -alt)
        rv_e  = sim.vec_to_list(sim.fns.bz_altitude_hold(pos_e, el, 200.0))
        # North case: X component dominant, Y small
        assert abs(rv_n[0]) > abs(rv_n[1])
        # East case: Y component dominant, X small
        assert abs(rv_e[1]) > abs(rv_e[0])

    def test_gravity_comp_increases_with_lower_elevation(self, sim):
        """
        The angular deviation of body_z from the tether direction is larger
        at low elevation: k = mass*g*cos(el)/tension is larger when el is small.
        """
        r = 30.0
        tension = 200.0

        def deviation_angle(el_deg):
            el  = math.radians(el_deg)
            pos = sim.lua_vec(r * math.cos(el), 0.0, -r * math.sin(el))
            bz  = sim.vec_to_list(sim.fns.bz_altitude_hold(pos, el, tension))
            tdir = [math.cos(el), 0.0, -math.sin(el)]
            dot  = sum(bz[i] * tdir[i] for i in range(3))
            return math.acos(max(-1.0, min(1.0, dot)))

        assert deviation_angle(10.0) > deviation_angle(70.0)

    @pytest.mark.parametrize("el_deg,az_deg,tension", [
        (10.0, 0.0,   150.0),
        (25.0, 45.0,  200.0),
        (40.0, 90.0,  300.0),
        (70.0, 180.0, 250.0),
        (80.0, 270.0, 400.0),
        (15.0, 30.0,  180.0),
    ])
    def test_matches_python_compute_bz_altitude_hold(self, sim, el_deg, az_deg, tension):
        """
        Cross-check: Lua bz_altitude_hold matches controller.py
        compute_bz_altitude_hold across a sweep of elevations and azimuths.
        """
        MASS_KG = float(sim.fns.MASS_KG)
        el_rad  = math.radians(el_deg)
        az_rad  = math.radians(az_deg)
        r       = 40.0
        pos_np  = np.array([
            r * math.cos(el_rad) * math.cos(az_rad),
            r * math.cos(el_rad) * math.sin(az_rad),
            -r * math.sin(el_rad),
        ])
        expected = compute_bz_altitude_hold(pos_np, el_rad, tension, MASS_KG).tolist()

        pos_lua = sim.lua_vec(*pos_np)
        result  = sim.vec_to_list(sim.fns.bz_altitude_hold(pos_lua, el_rad, tension))

        assert result == pytest.approx(expected, abs=1e-10), \
            f"Mismatch at el={el_deg} az={az_deg} T={tension}"


# ── cyclic_error_body ─────────────────────────────────────────────────────────

class TestCyclicErrorBody:
    """
    Tests for the body-frame cyclic error: err_ned = bz_now × bz_target,
    projected into body frame via ahrs:earth_to_body().
    """

    def test_equilibrium_is_zero(self, sim):
        """When bz_now == bz_target the error is identically zero."""
        sim.R = np.eye(3)
        bz = sim.lua_vec(0, 0, 1)
        result = sim.fns.cyclic_error_body(bz, bz)
        assert float(result[1]) == pytest.approx(0.0, abs=1e-12)
        assert float(result[2]) == pytest.approx(0.0, abs=1e-12)

    def test_north_tilt_drives_pitch(self, sim):
        """
        bz_now tilted North from bz_target (NED down): err_ned has Y component
        → body-Y (pitch) error is nonzero; body-X (roll) is near zero.
        """
        sim.R = np.eye(3)
        bz_target = sim.lua_vec(0, 0, 1)
        bz_now    = sim.lua_vec(*_unit([0.15, 0, 1]))
        result    = sim.fns.cyclic_error_body(bz_now, bz_target)
        err_bx, err_by = float(result[1]), float(result[2])
        assert abs(err_bx) < 1e-10
        assert err_by < -1e-6

    def test_east_tilt_drives_roll(self, sim):
        """bz_now tilted East: body-X (roll) error nonzero; pitch near zero."""
        sim.R = np.eye(3)
        bz_target = sim.lua_vec(0, 0, 1)
        bz_now    = sim.lua_vec(*_unit([0, 0.15, 1]))
        result    = sim.fns.cyclic_error_body(bz_now, bz_target)
        err_bx, err_by = float(result[1]), float(result[2])
        assert err_bx > 1e-6
        assert abs(err_by) < 1e-10

    def test_error_magnitude_invariant_under_yaw(self, sim):
        """
        Rotating R around Z (yaw) must not change |err_bx|^2 + |err_by|^2.
        The total cyclic correction magnitude depends only on the tilt angle.
        """
        bz_target = sim.lua_vec(0, 0, 1)
        bz_now    = sim.lua_vec(*_unit([0.1, 0.2, 0.9]))
        sim.R = np.eye(3)
        r0    = sim.fns.cyclic_error_body(bz_now, bz_target)
        ref_mag = math.sqrt(float(r0[1])**2 + float(r0[2])**2)

        for yaw in [0.4, 1.2, -0.7, math.pi / 4]:
            sim.R = _rot_z(yaw)
            r   = sim.fns.cyclic_error_body(bz_now, bz_target)
            mag = math.sqrt(float(r[1])**2 + float(r[2])**2)
            assert mag == pytest.approx(ref_mag, abs=1e-10), \
                f"Magnitude changed at yaw={math.degrees(yaw):.1f} deg"

    @pytest.mark.parametrize("yaw", [0.0, 0.5, 1.2, -0.7, math.pi / 4])
    def test_matches_compute_rate_cmd(self, sim, yaw):
        """
        Cross-check: Lua cyclic_error_body (kp=1) matches controller.py
        compute_rate_cmd(kp=1, kd=0) roll and pitch outputs.
        """
        bz_target_np = _unit([0.0, 0.0, 1.0])
        bz_now_np    = _unit([0.1, 0.2, 0.9])
        R            = _rot_z(yaw)
        sim.R = R

        result   = sim.fns.cyclic_error_body(
            sim.lua_vec(*bz_now_np), sim.lua_vec(*bz_target_np))
        py_rates = compute_rate_cmd(
            bz_now_np, bz_target_np, R, kp=1.0, kd=0.0)

        assert float(result[1]) == pytest.approx(py_rates[0], abs=1e-12)
        assert float(result[2]) == pytest.approx(py_rates[1], abs=1e-12)


# ── output_rate_limit ─────────────────────────────────────────────────────────

class TestOutputRateLimit:

    def test_no_change_needed(self, sim):
        assert int(sim.fns.output_rate_limit(1600, 1600, 30)) == 1600

    def test_small_step_passes_through(self, sim):
        assert int(sim.fns.output_rate_limit(1520, 1500, 30)) == 1520

    def test_large_positive_step_clamped(self, sim):
        assert int(sim.fns.output_rate_limit(1700, 1500, 30)) == 1530

    def test_large_negative_step_clamped(self, sim):
        assert int(sim.fns.output_rate_limit(1300, 1500, 30)) == 1470

    def test_max_delta_zero_disables(self, sim):
        assert int(sim.fns.output_rate_limit(1000, 1500, 0)) == 1000
        assert int(sim.fns.output_rate_limit(2000, 1500, 0)) == 2000

    def test_convergence_steps(self, sim):
        """Steps to reach target == ceil(|target - start| / max_delta)."""
        start, target, max_delta = 1500, 1800, 30
        expected = math.ceil(abs(target - start) / max_delta)
        prev, steps = start, 0
        for _ in range(200):
            new = int(sim.fns.output_rate_limit(target, prev, max_delta))
            steps += 1
            if new == target:
                break
            prev = new
        assert steps == expected


# ── rate_to_pwm ───────────────────────────────────────────────────────────────

class TestRateToPwm:

    def test_zero_rate_is_neutral(self, sim):
        assert int(sim.fns.rate_to_pwm(0.0)) == 1500

    def test_full_positive_is_2000(self, sim):
        max_rate = math.radians(360.0)
        assert int(sim.fns.rate_to_pwm(max_rate)) == 2000

    def test_full_negative_is_1000(self, sim):
        max_rate = math.radians(360.0)
        assert int(sim.fns.rate_to_pwm(-max_rate)) == 1000

    def test_clamped_above_full_stick(self, sim):
        assert int(sim.fns.rate_to_pwm(100.0)) == 2000

    def test_clamped_below_full_stick(self, sim):
        assert int(sim.fns.rate_to_pwm(-100.0)) == 1000

    @pytest.mark.parametrize("rate_rads", [
        0.0, 0.5, -0.5, 1.0, -1.0, math.pi, -math.pi
    ])
    def test_matches_controller_py_pwm(self, sim, rate_rads):
        """
        Cross-check: Lua rate_to_pwm matches the Python controller PWM formula.
        Both use ACRO_RP_RATE = 360 deg/s.
        """
        lua_pwm = int(sim.fns.rate_to_pwm(rate_rads))
        py_pwm  = _py_pwm(rate_rads, rate_max_deg=360.0)
        assert lua_pwm == py_pwm, \
            f"PWM mismatch at rate={rate_rads:.4f}: lua={lua_pwm} py={py_pwm}"

    @pytest.mark.parametrize("acro_rate_deg", [200.0, 360.0, 720.0])
    def test_matches_py_pwm_various_acro_rates(self, sim, acro_rate_deg):
        """Both formulas agree across a sweep of rates for various ACRO_RP_RATE settings."""
        for rate_rads in np.linspace(-math.radians(acro_rate_deg),
                                      math.radians(acro_rate_deg), 21):
            lua_pwm = int(sim.fns.rate_to_pwm(rate_rads, acro_rate_deg))
            py_pwm  = _py_pwm(rate_rads, rate_max_deg=acro_rate_deg)
            assert lua_pwm == py_pwm
