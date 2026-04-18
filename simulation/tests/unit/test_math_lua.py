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
    orbit_tracked_body_z_eq,
    slerp_body_z,
)
from rawes_lua_harness import RawesLua

# ── Module-level harness ──────────────────────────────────────────────────────
# One Lua runtime for the whole module.  Math functions are stateless;
# tests that need a specific R matrix set it explicitly.

@pytest.fixture(scope="module")
def sim():
    s = RawesLua()
    s.R = np.eye(3)   # identity default: body = NED
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
    assert float(f.KP_VZ)                 == pytest.approx(0.05)
    assert float(f.VZ_LAND_SP)            == pytest.approx(0.5)
    assert float(f.XI_REEL_IN_DEG)        == pytest.approx(80.0)


# ── rodrigues ─────────────────────────────────────────────────────────────────

class TestRodrigues:

    def test_zero_angle_identity(self, sim):
        v    = sim.lua_vec(1, 0, 0)
        axis = sim.lua_vec(0, 0, 1)
        r    = sim.fns.rodrigues(v, axis, 0.0)
        assert sim.vec_to_list(r) == pytest.approx([1, 0, 0], abs=1e-12)

    def test_90deg_around_z(self, sim):
        v    = sim.lua_vec(1, 0, 0)
        axis = sim.lua_vec(0, 0, 1)
        r    = sim.fns.rodrigues(v, axis, math.pi / 2)
        assert sim.vec_to_list(r) == pytest.approx([0, 1, 0], abs=1e-12)

    def test_180deg_around_z(self, sim):
        v    = sim.lua_vec(1, 0, 0)
        axis = sim.lua_vec(0, 0, 1)
        r    = sim.fns.rodrigues(v, axis, math.pi)
        assert sim.vec_to_list(r) == pytest.approx([-1, 0, 0], abs=1e-12)

    def test_preserves_vector_length(self, sim):
        v    = sim.lua_vec(0.5, 0.3, 0.8)
        axis = sim.lua_vec(*_unit([1, 1, 1]))
        r    = sim.fns.rodrigues(v, axis, 0.7)
        orig_len = math.sqrt(0.5**2 + 0.3**2 + 0.8**2)
        result   = sim.vec_to_list(r)
        assert math.sqrt(sum(x**2 for x in result)) == pytest.approx(orig_len, abs=1e-12)

    def test_parallel_vector_unchanged(self, sim):
        """Rotating a vector parallel to the axis leaves it unchanged."""
        axis = sim.lua_vec(0, 0, 1)
        v    = sim.lua_vec(0, 0, 2.5)
        r    = sim.fns.rodrigues(v, axis, 1.5)
        assert sim.vec_to_list(r) == pytest.approx([0, 0, 2.5], abs=1e-12)

    def test_inverse_is_negative_angle(self, sim):
        """rodrigues(rodrigues(v, axis, a), axis, -a) == v."""
        v    = sim.lua_vec(*_unit([0.4, -0.3, 0.8]))
        axis = sim.lua_vec(*_unit([1, 2, -1]))
        for angle in [0.1, 0.5, 1.0, -0.8, -math.pi / 3]:
            fwd = sim.fns.rodrigues(v,   axis,  angle)
            bwd = sim.fns.rodrigues(fwd, axis, -angle)
            assert sim.vec_to_list(bwd) == pytest.approx(sim.vec_to_list(v), abs=1e-11)

    def test_matches_rotation_matrix(self, sim):
        """rodrigues(v, axis, a) == scipy-style R @ v."""
        angle = 0.4
        axis  = _unit([1, 2, 3])
        v_np  = np.array([0.6, -0.2, 0.7])
        # Rodrigues rotation matrix formula
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        R = np.eye(3) + math.sin(angle)*K + (1 - math.cos(angle))*(K @ K)
        expected = R @ v_np
        v_lua   = sim.lua_vec(*v_np)
        axis_lua = sim.lua_vec(*axis)
        result  = sim.vec_to_list(sim.fns.rodrigues(v_lua, axis_lua, angle))
        assert result == pytest.approx(expected.tolist(), abs=1e-12)


# ── orbit_track_azimuthal ─────────────────────────────────────────────────────

class TestOrbitTrackAzimuthal:
    """
    Tests for the azimuthal orbit tracking function used in rawes.lua.

    orbit_track_azimuthal(bz_eq0, tdir0, diff):
      - diff is the raw pos-anchor vector (not normalised)
      - applies a 2D horizontal rotation from tdir0 to diff/|diff|
      - preserves the Z component of bz_eq0 before normalising
      - output is always a unit vector
    """

    def test_no_movement_returns_bz_eq0(self, sim):
        """When diff points the same direction as tdir0, no rotation is applied."""
        bz_eq0 = sim.lua_vec(*_unit([0.5, 0.2, 0.8]))
        tdir0  = sim.lua_vec(1, 0, 0)
        diff   = sim.lua_vec(5, 0, 0)      # same direction, arbitrary length
        r = sim.fns.orbit_track_azimuthal(bz_eq0, tdir0, diff)
        assert sim.vec_to_list(r) == pytest.approx(sim.vec_to_list(bz_eq0), abs=1e-10)

    def test_short_diff_guard(self, sim):
        """dh < 0.01 m: horizontal component too small → returns bz_eq0."""
        bz_eq0 = sim.lua_vec(*_unit([0.5, 0.2, 0.8]))
        tdir0  = sim.lua_vec(1, 0, 0)
        diff   = sim.lua_vec(0, 0.005, 0)  # dh = 0.005 < 0.01
        r = sim.fns.orbit_track_azimuthal(bz_eq0, tdir0, diff)
        assert sim.vec_to_list(r) == pytest.approx(sim.vec_to_list(bz_eq0), abs=1e-10)

    def test_vertical_tdir0_guard(self, sim):
        """n0 (horizontal norm of tdir0) < 0.01 → returns bz_eq0."""
        bz_eq0 = sim.lua_vec(*_unit([0.5, 0.2, 0.8]))
        tdir0  = sim.lua_vec(0, 0, 1)      # vertical: horizontal norm = 0
        diff   = sim.lua_vec(5, 0, 0)
        r = sim.fns.orbit_track_azimuthal(bz_eq0, tdir0, diff)
        assert sim.vec_to_list(r) == pytest.approx(sim.vec_to_list(bz_eq0), abs=1e-10)

    def test_output_is_unit_vector(self, sim):
        """Result is normalised regardless of input magnitudes."""
        bz_eq0 = sim.lua_vec(*_unit([0.3, 0.4, 0.8]))
        tdir0  = sim.lua_vec(1, 0, 0)
        diff   = sim.lua_vec(0, 10, -3)   # arbitrary
        r = sim.fns.orbit_track_azimuthal(bz_eq0, tdir0, diff)
        rv = sim.vec_to_list(r)
        assert math.sqrt(sum(x**2 for x in rv)) == pytest.approx(1.0, abs=1e-12)

    def test_90deg_rotation_z_preserved(self, sim):
        """
        Tether moves 90° in horizontal plane.  The z component of bz_eq0
        is preserved before normalisation, so it should only change due to
        re-normalisation of the rotated horizontal components.
        """
        bz_eq0_np = _unit([0.5, 0.0, 0.8])   # tilted North + slightly down
        bz_eq0 = sim.lua_vec(*bz_eq0_np)
        tdir0  = sim.lua_vec(1, 0, 0)          # initial tether: North
        diff   = sim.lua_vec(0, 5, 0)           # tether now: East (90° rotation)
        r = sim.fns.orbit_track_azimuthal(bz_eq0, tdir0, diff)
        rv = sim.vec_to_list(r)

        # After 90° CW rotation around Z: bx→by, by→-bx
        bx0, by0, bz0 = bz_eq0_np
        expected_raw = np.array([-by0, bx0, bz0])    # sin(90)*bx + cos(90)*by...
        # Actually: cos_phi=0, sin_phi=1 for 90deg: r:x = -by, r:y = bx
        # Wait: cos_phi = t0x*thx + t0y*thy = 1*0 + 0*1 = 0
        #       sin_phi = t0x*thy - t0y*thx = 1*1 - 0*0 = 1
        # r:x = cos*bx - sin*by = 0*0.5 - 1*0 = 0
        # r:y = sin*bx + cos*by = 1*0.5 + 0*0 = 0.5
        # r:z = bz0 = 0.8
        expected_raw = np.array([0.0, 0.5, 0.8])
        expected = expected_raw / np.linalg.norm(expected_raw)
        assert rv == pytest.approx(expected.tolist(), abs=1e-10)

    def test_matches_controller_orbit_tracked_body_z_eq(self, sim):
        """
        Cross-check: Lua azimuthal track agrees with controller.py
        orbit_tracked_body_z_eq() across a full 360° orbit.
        """
        bz_eq0_np = _unit([0.6, 0.3, 0.7])
        tdir0_np  = _unit([1, 0, 0])
        bz_eq0    = sim.lua_vec(*bz_eq0_np)
        tdir0     = sim.lua_vec(*tdir0_np)

        for az_deg in range(0, 360, 30):
            az = math.radians(az_deg)
            pos = np.array([50 * math.cos(az), 50 * math.sin(az), -14.0])
            diff_np = pos   # anchor = [0,0,0]
            diff = sim.lua_vec(*diff_np)

            lua_r = sim.vec_to_list(
                sim.fns.orbit_track_azimuthal(bz_eq0, tdir0, diff))
            py_r  = orbit_tracked_body_z_eq(
                pos, tdir0_np, bz_eq0_np).tolist()

            assert lua_r == pytest.approx(py_r, abs=1e-10), \
                f"Mismatch at az={az_deg} deg"


# ── slerp_step ────────────────────────────────────────────────────────────────

class TestSlerpStep:
    """
    Tests for the rate-limited slerp step used in rawes.lua.
    This is the inline code in run_flight() that advances _bz_slerp toward goal.
    """

    def test_already_at_goal_unchanged(self, sim):
        """When bz_slerp == goal (remain < 1e-4), output equals input."""
        bz = sim.lua_vec(*_unit([0.3, 0.4, 0.8]))
        r  = sim.fns.slerp_step(bz, bz, 0.4, 0.02)
        assert sim.vec_to_list(r) == pytest.approx(sim.vec_to_list(bz), abs=1e-12)

    def test_step_limited_to_slew_rate(self, sim):
        """Angular advance per step <= slew_rate * dt."""
        bz   = sim.lua_vec(0, 0, 1)
        goal = sim.lua_vec(1, 0, 0)   # 90° away
        slew, dt = 0.4, 0.02
        r    = sim.fns.slerp_step(bz, goal, slew, dt)
        bv, rv = np.array([0, 0, 1]), np.array(sim.vec_to_list(r))
        step_taken = math.acos(float(np.clip(np.dot(bv, rv), -1, 1)))
        assert step_taken == pytest.approx(slew * dt, abs=1e-10)

    def test_converges_to_goal(self, sim):
        """Repeated steps reach goal within expected number of steps."""
        bz_np   = np.array([0., 0., 1.])
        goal_np = _unit([0.5, 0.5, 0.7])
        bz      = sim.lua_vec(*bz_np)
        goal    = sim.lua_vec(*goal_np)
        slew, dt = 0.4, 0.02
        angle0   = math.acos(float(np.dot(bz_np, goal_np)))
        max_steps = int(angle0 / (slew * dt)) + 5

        for _ in range(max_steps):
            bz = sim.fns.slerp_step(bz, goal, slew, dt)
            remaining = math.acos(float(np.clip(
                np.dot(sim.vec_to_list(bz), goal_np), -1, 1)))
            if remaining < 1e-4:
                break
        assert remaining < 1e-4

    def test_preserves_unit_length(self, sim):
        """slerp_step output is always a unit vector."""
        bz   = sim.lua_vec(*_unit([0.3, -0.5, 0.8]))
        goal = sim.lua_vec(*_unit([-0.2, 0.7, 0.6]))
        for _ in range(30):
            bz = sim.fns.slerp_step(bz, goal, 0.5, 0.02)
            rv = sim.vec_to_list(bz)
            assert math.sqrt(sum(x**2 for x in rv)) == pytest.approx(1.0, abs=1e-12)

    @pytest.mark.parametrize("slew", [0.1, 0.4, 1.0, 2.0])
    def test_matches_controller_slerp_body_z(self, sim, slew):
        """
        Cross-check: Lua slerp_step matches controller.py slerp_body_z
        for all slew rates across a multi-step trajectory.
        """
        bz_lua_np = np.array([0., 0., 1.])
        bz_py     = bz_lua_np.copy()
        goal_np   = _unit([0.5, 0.5, 0.7])
        bz_lua    = sim.lua_vec(*bz_lua_np)
        goal_lua  = sim.lua_vec(*goal_np)
        dt = 0.02

        for step in range(50):
            bz_lua = sim.fns.slerp_step(bz_lua, goal_lua, slew, dt)
            bz_py  = slerp_body_z(bz_py, goal_np, slew_rate_rad_s=slew, dt=dt)
            assert sim.vec_to_list(bz_lua) == pytest.approx(bz_py.tolist(), abs=1e-10), \
                f"Diverged at step {step}, slew={slew}"


# ── cyclic_error_body ─────────────────────────────────────────────────────────

class TestCyclicErrorBody:
    """
    Tests for the body-frame cyclic error: err_ned = bz_now × bz_orbit,
    projected into body frame via ahrs:earth_to_body().
    """

    def test_equilibrium_is_zero(self, sim):
        """When bz_now == bz_orbit the error is identically zero."""
        sim.R = np.eye(3)
        bz = sim.lua_vec(0, 0, 1)
        result = sim.fns.cyclic_error_body(bz, bz)
        assert float(result[1]) == pytest.approx(0.0, abs=1e-12)
        assert float(result[2]) == pytest.approx(0.0, abs=1e-12)

    def test_north_tilt_drives_pitch(self, sim):
        """
        bz_now tilted North from bz_orbit (NED down): err_ned has Y component
        → body-Y (pitch) error is nonzero; body-X (roll) is near zero.
        """
        sim.R = np.eye(3)   # body = NED
        bz_orbit = sim.lua_vec(0, 0, 1)
        bz_now   = sim.lua_vec(*_unit([0.15, 0, 1]))  # tilted N
        result   = sim.fns.cyclic_error_body(bz_now, bz_orbit)
        err_bx, err_by = float(result[1]), float(result[2])
        assert abs(err_bx) < 1e-10
        assert err_by < -1e-6

    def test_east_tilt_drives_roll(self, sim):
        """bz_now tilted East: body-X (roll) error nonzero; pitch near zero."""
        sim.R = np.eye(3)
        bz_orbit = sim.lua_vec(0, 0, 1)
        bz_now   = sim.lua_vec(*_unit([0, 0.15, 1]))  # tilted E
        result   = sim.fns.cyclic_error_body(bz_now, bz_orbit)
        err_bx, err_by = float(result[1]), float(result[2])
        assert err_bx > 1e-6
        assert abs(err_by) < 1e-10

    def test_error_magnitude_invariant_under_yaw(self, sim):
        """
        Rotating R around Z (yaw) must not change |err_bx|^2 + |err_by|^2.
        The total cyclic correction magnitude depends only on the tilt angle.
        """
        bz_orbit = sim.lua_vec(0, 0, 1)
        bz_now   = sim.lua_vec(*_unit([0.1, 0.2, 0.9]))
        # Reference magnitude with identity R
        sim.R = np.eye(3)
        r0 = sim.fns.cyclic_error_body(bz_now, bz_orbit)
        ref_mag = math.sqrt(float(r0[1])**2 + float(r0[2])**2)

        for yaw in [0.4, 1.2, -0.7, math.pi / 4]:
            sim.R = _rot_z(yaw)
            r = sim.fns.cyclic_error_body(bz_now, bz_orbit)
            mag = math.sqrt(float(r[1])**2 + float(r[2])**2)
            assert mag == pytest.approx(ref_mag, abs=1e-10), \
                f"Magnitude changed at yaw={math.degrees(yaw):.1f} deg"

    @pytest.mark.parametrize("yaw", [0.0, 0.5, 1.2, -0.7, math.pi / 4])
    def test_matches_compute_rate_cmd(self, sim, yaw):
        """
        Cross-check: Lua cyclic_error_body (kp=1) matches controller.py
        compute_rate_cmd(kp=1, kd=0) roll and pitch outputs.
        """
        bz_orbit_np = _unit([0.0, 0.0, 1.0])
        bz_now_np   = _unit([0.1, 0.2, 0.9])
        R           = _rot_z(yaw)
        sim.R = R

        result = sim.fns.cyclic_error_body(
            sim.lua_vec(*bz_now_np), sim.lua_vec(*bz_orbit_np))
        py_rates = compute_rate_cmd(
            bz_now_np, bz_orbit_np, R, kp=1.0, kd=0.0)

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
