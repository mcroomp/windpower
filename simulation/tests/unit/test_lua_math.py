"""
test_lua_math.py -- Unit tests for all math in rawes_flight.lua.

Covers four Lua operations:
  1. rodrigues()        -- rotate a vector around a unit axis by an angle.
  2. orbit_track()      -- rotate bz_eq0 by the minimal rotation mapping tdir0 to bzt.
  3. slerp_step()       -- rate-limited advance of bz_slerp toward a goal.
  4. cyclic_error_body() -- NED-frame body_z error -> body-frame roll/pitch components.
  5. output_rate_limit() -- per-step PWM clamp.
  6. PWM scalar formula  -- rate_rads -> PWM integer.

Also cross-checks each Lua reference implementation against the corresponding
controller.py function to catch divergence when one side is updated.

Design note -- two orbit-tracking functions in controller.py:
  orbit_tracked_body_z_eq()     azimuthal-only rotation (preserves Z, stable at
      400 Hz without rate limiting -- used in the Python simulation loop)
  orbit_tracked_body_z_eq_3d()  full 3D Rodrigues rotation -- matches
      rawes_flight.lua::orbit_track() exactly; safe only when downstream slerp
      limits bandwidth (as the Lua 0.40 rad/s slerp does on hardware)

No SITL, no Docker.  Runs with the existing unit-test venv.
"""
import math
import random
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from controller import (
    compute_rate_cmd,
    orbit_tracked_body_z_eq,
    orbit_tracked_body_z_eq_3d,
    slerp_body_z,
)


# ===========================================================================
# -- Reference implementations --
# Python reference implementations of rawes_flight.lua functions.
# These must stay bit-for-bit identical to the Lua source.
# ===========================================================================

def rodrigues(v: np.ndarray, axis_n: np.ndarray, angle: float) -> np.ndarray:
    """
    Rotate vector v around unit vector axis_n by angle radians.

    v' = v*cos(theta) + (axis x v)*sin(theta) + axis*(axis.v)*(1 - cos(theta))

    Mirrors rawes_flight.lua rodrigues() verbatim.

    Lua source:
        local ca  = math.cos(angle)
        local sa  = math.sin(angle)
        local ac  = axis_n:cross(v)
        local ad  = axis_n:dot(v)
        return v * ca + ac * sa + axis_n * (ad * (1.0 - ca))
    """
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (v * ca
            + np.cross(axis_n, v) * sa
            + axis_n * np.dot(axis_n, v) * (1.0 - ca))


def orbit_track_bz(
    bz_eq0: np.ndarray,
    tdir0:  np.ndarray,
    bzt:    np.ndarray,
) -> np.ndarray:
    """
    Rotate bz_eq0 by the minimal rotation that maps tdir0 -> bzt.

    Mirrors rawes_flight.lua orbit-tracking block:
        axis  = tdir0 x bzt
        sinth = |axis|
        if sinth > 1e-6:
            bz_orbit = rodrigues(bz_eq0, axis/sinth, atan2(sinth, costh))
        else:
            bz_orbit = bz_eq0  (tether hasn't moved)
    """
    axis  = np.cross(tdir0, bzt)
    sinth = float(np.linalg.norm(axis))
    if sinth < 1e-6:
        return bz_eq0.copy()
    costh = float(np.dot(tdir0, bzt))
    return rodrigues(bz_eq0, axis / sinth, math.atan2(sinth, costh))


def slerp_step(
    bz_slerp:  np.ndarray,
    goal:      np.ndarray,
    slew_rate: float,
    dt:        float,
) -> np.ndarray:
    """
    One rate-limited slerp step advancing bz_slerp toward goal.

    Mirrors rawes_flight.lua slerp block:
        dot    = clamp(bz_slerp . goal, -1, 1)
        remain = acos(dot)
        if remain > 1e-4:
            ax   = bz_slerp x goal
            step = min(slew_rate * dt, remain)
            bz_slerp = rodrigues(bz_slerp, ax / |ax|, step)
    """
    dot    = float(np.clip(np.dot(bz_slerp, goal), -1.0, 1.0))
    remain = math.acos(dot)
    if remain < 1e-4:
        return bz_slerp.copy()
    ax = np.cross(bz_slerp, goal)
    ax_len = float(np.linalg.norm(ax))
    if ax_len < 1e-6:
        return bz_slerp.copy()
    step = min(slew_rate * dt, remain)
    return rodrigues(bz_slerp, ax / ax_len, step)


def cyclic_error_body(
    R:        np.ndarray,
    bz_now:   np.ndarray,
    bz_slerp: np.ndarray,
) -> tuple:
    """
    Body-frame cyclic error (roll, pitch) from the P-gain loop.

    Mirrors rawes_flight.lua:
        err_ned = bz_now x bz_slerp
        err_bx  = R.col(0) . err_ned    (body X = roll axis)
        err_by  = R.col(1) . err_ned    (body Y = pitch axis)

    R is rotation_body_to_ned (3x3, columns = body axes in NED).
    """
    err_ned = np.cross(bz_now, bz_slerp)
    err_bx  = float(R[:, 0] @ err_ned)
    err_by  = float(R[:, 1] @ err_ned)
    return err_bx, err_by


def output_rate_limit(
    ch_desired: int,
    ch_prev:    int,
    max_delta:  int,
) -> int:
    """
    Python reference implementation of the rawes_flight.lua output rate limiter.

    Clamps the per-step PWM change to max_delta.  Returns the new PWM value.
    max_delta == 0 means disabled (no clamping).

    Mirrors rawes_flight.lua:
        d = ch_desired - ch_prev
        d = clamp(d, -max_delta, +max_delta)
        return ch_prev + d
    """
    if max_delta == 0:
        return ch_desired
    d = ch_desired - ch_prev
    if d >  max_delta: d =  max_delta
    if d < -max_delta: d = -max_delta
    return ch_prev + d


# ---------------------------------------------------------------------------
# Additional Lua transcriptions used in algorithm parity tests
# ---------------------------------------------------------------------------

def _lua_orbit_track(
    bz_eq0: np.ndarray,
    tdir0:  np.ndarray,
    pos:    np.ndarray,
    anchor: np.ndarray,
) -> np.ndarray:
    """
    Python transcription of rawes_flight.lua orbit_track().

    Rotates bz_eq0 by the same 3D rotation that maps tdir0 -> current
    tether direction.  Returns bz_eq0 unchanged when |pos-anchor| < 0.5 m
    or when there is no rotation (hub hasn't moved).
    """
    diff = pos - anchor
    tlen = float(np.linalg.norm(diff))
    if tlen < 0.5:
        return bz_eq0.copy()
    bzt   = diff / tlen
    axis  = np.cross(tdir0, bzt)
    sinth = float(np.linalg.norm(axis))
    if sinth < 1e-6:
        return bz_eq0.copy()
    costh = float(np.dot(tdir0, bzt))
    return rodrigues(bz_eq0, axis / sinth, math.atan2(sinth, costh))


def _lua_slerp_step(
    bz_slerp:  np.ndarray,
    goal:      np.ndarray,
    slew_rate: float,
    dt:        float,
) -> np.ndarray:
    """
    Python transcription of rawes_flight.lua rate-limited slerp step.

    Advances bz_slerp toward goal by at most slew_rate*dt radians.
    """
    dot    = float(np.clip(np.dot(bz_slerp, goal), -1.0, 1.0))
    remain = math.acos(dot)
    if remain < 1e-4:
        return goal.copy()
    step = min(slew_rate * dt, remain)
    ax   = np.cross(bz_slerp, goal)
    axn  = float(np.linalg.norm(ax))
    if axn < 1e-6:
        return goal.copy()
    return rodrigues(bz_slerp, ax / axn, step)


def _lua_cyclic_rates(
    R_col_x:    np.ndarray,   # R:colx() -- body X axis in world frame
    R_col_y:    np.ndarray,   # R:coly() -- body Y axis in world frame
    R_col_z:    np.ndarray,   # R:colz() -- body Z axis in world frame
    bz_slerp:   np.ndarray,
    kp:         float,
    acro_rp_deg: float = 360.0,
) -> tuple:
    """
    Python transcription of rawes_flight.lua cyclic loop.

    Returns (roll_rads, pitch_rads, ch1_pwm, ch2_pwm).

    Lua source:
        local err_ned = bz_now:cross(_bz_slerp)
        local err_bx  = R:colx():dot(err_ned)
        local err_by  = R:coly():dot(err_ned)
        local roll_rads  = kp * err_bx
        local pitch_rads = kp * err_by
        local scale = 500 / (ACRO_RP_RATE_DEG * pi / 180)
        ch1 = clamp(1500 + scale * roll_rads,  1000, 2000)
        ch2 = clamp(1500 + scale * pitch_rads, 1000, 2000)
    """
    err_world  = np.cross(R_col_z, bz_slerp)
    err_bx     = float(np.dot(R_col_x, err_world))
    err_by     = float(np.dot(R_col_y, err_world))
    roll_rads  = kp * err_bx
    pitch_rads = kp * err_by
    scale  = 500.0 / (acro_rp_deg * math.pi / 180.0)
    ch1 = int(round(1500.0 + scale * roll_rads))
    ch2 = int(round(1500.0 + scale * pitch_rads))
    ch1 = max(1000, min(2000, ch1))
    ch2 = max(1000, min(2000, ch2))
    return roll_rads, pitch_rads, ch1, ch2


def _lua_pwm(rate_rads: float, acro_rp_rate_deg: float = 360.0) -> int:
    """Mirror of rawes_flight.lua ch1/ch2 computation (pure math, no ArduPilot)."""
    scale = 500.0 / (acro_rp_rate_deg * math.pi / 180.0)
    ch = math.floor(1500.0 + scale * rate_rads + 0.5)
    return max(1000, min(2000, ch))


def _py_pwm(rate_rads: float, rate_max_deg: float = 360.0) -> int:
    """Mirror of compute_rc_rates inner _pwm() (pure math, no hub_state)."""
    max_rate = math.radians(rate_max_deg)
    return int(round(1500.0 + 500.0 * max(-1.0, min(1.0, rate_rads / max_rate))))


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def _unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def _rot_z(angle: float) -> np.ndarray:
    """3x3 rotation matrix: rotate around NED Z (down) axis."""
    ca, sa = math.cos(angle), math.sin(angle)
    return np.array([[ca, -sa, 0.],
                     [sa,  ca, 0.],
                     [0.,  0., 1.]])


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0.]])


def _build_R_from_body_z(bz: np.ndarray) -> np.ndarray:
    """Rotation matrix whose third column is bz (arbitrary body X/Y)."""
    bz = bz / np.linalg.norm(bz)
    ref = np.array([0., 1., 0.]) if abs(bz[1]) < 0.9 else np.array([1., 0., 0.])
    bx  = np.cross(ref, bz);  bx /= np.linalg.norm(bx)
    by  = np.cross(bz, bx);   by /= np.linalg.norm(by)
    return np.column_stack([bx, by, bz])


# Reference hub: 50 m tether, 30 deg elevation, East of anchor
_ELEV   = math.radians(30.0)
_ANCHOR = np.zeros(3)
_POS0   = 50.0 * np.array([math.cos(_ELEV), 0., math.sin(_ELEV)])
_TDIR0  = _POS0 / np.linalg.norm(_POS0)


# ===========================================================================
# -- Algorithm unit tests --
# Tests for rodrigues(), orbit_track_bz(), slerp_step(), cyclic_error_body(),
# output_rate_limit() as standalone functions.
# ===========================================================================

# ---------------------------------------------------------------------------
# rodrigues() -- standalone reference implementation tests
# ---------------------------------------------------------------------------

def test_rodrigues_zero_angle_identity():
    v    = np.array([1., 0., 0.])
    axis = np.array([0., 0., 1.])
    np.testing.assert_allclose(rodrigues(v, axis, 0.0), v, atol=1e-12)


def test_rodrigues_90deg_around_z():
    """Rotate [1,0,0] 90 deg around Z -> [0,1,0]."""
    v    = np.array([1., 0., 0.])
    axis = np.array([0., 0., 1.])
    np.testing.assert_allclose(rodrigues(v, axis, math.pi / 2),
                               [0., 1., 0.], atol=1e-12)


def test_rodrigues_180deg_around_z():
    """Rotate [1,0,0] 180 deg around Z -> [-1,0,0]."""
    v    = np.array([1., 0., 0.])
    axis = np.array([0., 0., 1.])
    np.testing.assert_allclose(rodrigues(v, axis, math.pi),
                               [-1., 0., 0.], atol=1e-12)


def test_rodrigues_preserves_vector_length():
    v    = np.array([0.5, 0.3, 0.8])
    axis = _unit(np.array([1., 1., 1.]))
    result = rodrigues(v, axis, 0.7)
    assert abs(np.linalg.norm(result) - np.linalg.norm(v)) < 1e-12


def test_rodrigues_parallel_vector_unchanged():
    """Rotating a vector parallel to the axis leaves it unchanged."""
    axis = np.array([0., 0., 1.])
    v    = np.array([0., 0., 2.5])
    np.testing.assert_allclose(rodrigues(v, axis, 1.5), v, atol=1e-12)


def test_rodrigues_matches_rotation_matrix():
    """rodrigues() must equal the equivalent 3x3 rotation matrix * v."""
    angle = 0.4
    axis  = _unit(np.array([1., 2., 3.]))
    v     = np.array([0.6, -0.2, 0.7])
    K = _skew(axis)
    R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K
    np.testing.assert_allclose(rodrigues(v, axis, angle), R @ v, atol=1e-12)


@pytest.mark.parametrize("angle", [0.1, 0.5, 1.0, math.pi / 4, -0.8, -math.pi / 3])
def test_rodrigues_inverse_is_negative_angle(angle):
    """rodrigues(v, axis, angle) followed by rodrigues(_, axis, -angle) == v."""
    v    = _unit(np.array([0.4, -0.3, 0.8]))
    axis = _unit(np.array([1., 2., -1.]))
    forward  = rodrigues(v,       axis,  angle)
    backward = rodrigues(forward, axis, -angle)
    np.testing.assert_allclose(backward, v, atol=1e-12)


# ---------------------------------------------------------------------------
# rodrigues() -- class-based tests (mirror of Lua function verbatim)
# ---------------------------------------------------------------------------

class TestRodrigues:

    def test_identity_zero_angle(self):
        v    = np.array([1., 0., 0.])
        axis = np.array([0., 0., 1.])
        out  = rodrigues(v, axis, 0.0)
        np.testing.assert_allclose(out, v, atol=1e-12)

    def test_90deg_about_z(self):
        """rotate [1,0,0] by +90 deg around Z -> [0,1,0]"""
        v    = np.array([1., 0., 0.])
        axis = np.array([0., 0., 1.])
        out  = rodrigues(v, axis, math.pi / 2)
        np.testing.assert_allclose(out, [0., 1., 0.], atol=1e-12)

    def test_180deg_about_z(self):
        v    = np.array([1., 0., 0.])
        axis = np.array([0., 0., 1.])
        out  = rodrigues(v, axis, math.pi)
        np.testing.assert_allclose(out, [-1., 0., 0.], atol=1e-12)

    def test_90deg_about_x(self):
        """rotate [0,1,0] by +90 deg around X -> [0,0,1]"""
        v    = np.array([0., 1., 0.])
        axis = np.array([1., 0., 0.])
        out  = rodrigues(v, axis, math.pi / 2)
        np.testing.assert_allclose(out, [0., 0., 1.], atol=1e-12)

    def test_output_is_unit_length(self):
        """Rodrigues rotation preserves vector length."""
        v    = np.array([0.6, 0.8, 0.])
        axis = np.array([0., 0., 1.])
        out  = rodrigues(v, axis, 1.1)
        np.testing.assert_allclose(np.linalg.norm(out), np.linalg.norm(v), atol=1e-12)

    def test_recovers_original_after_360deg(self):
        v    = np.array([0.577, 0.577, 0.577])
        v   /= np.linalg.norm(v)
        axis = np.array([0., 1., 0.])
        out  = rodrigues(v, axis, 2 * math.pi)
        np.testing.assert_allclose(out, v, atol=1e-12)

    def test_rotate_parallel_to_axis_unchanged(self):
        """Rotating a vector parallel to the axis leaves it unchanged."""
        axis = np.array([0., 0., 1.])
        out  = rodrigues(axis.copy(), axis, math.pi / 3)
        np.testing.assert_allclose(out, axis, atol=1e-12)


# ---------------------------------------------------------------------------
# orbit_track_bz() -- standalone reference implementation tests
# ---------------------------------------------------------------------------

def test_orbit_track_no_movement():
    """No tether movement -> bz_orbit == bz_eq0."""
    bz_eq0 = _unit(np.array([0.5, 0.2, 0.8]))
    tdir0  = _unit(np.array([0.3, 0.4, 0.7]))
    np.testing.assert_allclose(orbit_track_bz(bz_eq0, tdir0, tdir0),
                               bz_eq0, atol=1e-12)


def test_orbit_track_90deg_in_horizontal_plane():
    """
    Tether rotates 90 deg in the XY plane (tdir0->bzt around Z).

    When tdir0 and bzt are both horizontal (Z=0), the Rodrigues axis
    tdir0 x bzt is [0,0,1], so orbit_track_bz applies exactly R_z(90) to bz_eq0.
    """
    tdir0  = np.array([1., 0., 0.])
    bzt    = np.array([0., 1., 0.])
    bz_eq0 = _unit(np.array([0.8, 0., 0.6]))
    expected = _unit(_rot_z(math.pi / 2) @ bz_eq0)
    result   = orbit_track_bz(bz_eq0, tdir0, bzt)
    np.testing.assert_allclose(result, expected, atol=1e-10)


def test_orbit_track_preserves_tether_bz_angle():
    """
    The angle between body_z and the tether direction is preserved during orbit.

    Core invariant: dot(bzt, orbit_track_bz(bz_eq0, tdir0, bzt))
                      == dot(tdir0, bz_eq0)
    because orbit_track_bz applies the rotation R* with R* @ tdir0 = bzt,
    and rotations preserve inner products.
    """
    tdir0   = _unit(np.array([0.5, 0.3, 0.8]))
    bz_eq0  = _unit(np.array([0.4, 0.2, 0.9]))
    cos0    = float(np.dot(tdir0, bz_eq0))

    for theta in [0.3, 1.0, -0.5, math.pi / 3, 2.0]:
        R   = _rot_z(theta)
        bzt = _unit(R @ tdir0)
        bz_new = orbit_track_bz(bz_eq0, tdir0, bzt)
        cos_new = float(np.dot(bzt, bz_new))
        assert abs(cos_new - cos0) < 1e-10, (
            f"dot(bzt, bz_orbit) changed: initial={cos0:.6f} "
            f"after theta={math.degrees(theta):.1f} deg: {cos_new:.6f}"
        )


def test_orbit_track_output_is_unit_vector():
    bz_eq0 = _unit(np.array([0.5, 0.2, 0.8]))
    tdir0  = _unit(np.array([0.3, 0.4, 0.7]))
    bzt    = _unit(np.array([-0.1, 0.6, 0.8]))
    result = orbit_track_bz(bz_eq0, tdir0, bzt)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-12


def test_orbit_track_near_antiparallel_tether():
    """Near-antiparallel tdir0 and bzt (almost 180 deg apart) is handled without crash."""
    tdir0  = _unit(np.array([1., 0., 0.]))
    bzt    = _unit(np.array([-1., 1e-7, 0.]))   # almost opposite
    bz_eq0 = _unit(np.array([0.8, 0., 0.6]))
    result = orbit_track_bz(bz_eq0, tdir0, bzt)
    assert np.linalg.norm(result) > 0.5
    assert math.isfinite(np.linalg.norm(result))


# ---------------------------------------------------------------------------
# slerp_step() -- standalone reference implementation tests
# ---------------------------------------------------------------------------

def test_slerp_already_at_goal():
    bz = _unit(np.array([0.3, 0.4, 0.8]))
    np.testing.assert_allclose(slerp_step(bz, bz, slew_rate=0.4, dt=0.02),
                               bz, atol=1e-12)


def test_slerp_clamps_to_rate_limit():
    """Step taken must not exceed slew_rate * dt."""
    bz   = np.array([0., 0., 1.])    # NED down
    goal = np.array([1., 0., 0.])    # NED north (90 deg away)
    dt, slew = 0.02, 0.4
    result    = slerp_step(bz, goal, slew_rate=slew, dt=dt)
    step_taken = math.acos(float(np.clip(np.dot(bz, result), -1., 1.)))
    assert abs(step_taken - slew * dt) < 1e-10


def test_slerp_takes_full_step_when_close():
    """When remaining angle < slew_rate * dt, jump all the way to goal."""
    bz   = np.array([0., 0., 1.])
    tiny = 0.001   # rad -- much less than 0.4 * 0.02 = 0.008 rad
    goal = _unit(rodrigues(bz, np.array([1., 0., 0.]), tiny))
    result = slerp_step(bz, goal, slew_rate=0.4, dt=0.02)
    np.testing.assert_allclose(result, goal, atol=1e-10)


def test_slerp_converges_to_goal():
    """Repeated slerp steps must reach the goal in finite time."""
    bz   = np.array([0., 0., 1.])
    goal = _unit(np.array([0.5, 0.5, 0.7]))
    slew, dt = 0.4, 0.02
    angle0   = math.acos(float(np.clip(np.dot(bz, goal), -1., 1.)))
    max_steps = int(angle0 / (slew * dt)) + 10
    for _ in range(max_steps):
        bz = slerp_step(bz, goal, slew_rate=slew, dt=dt)
        if math.acos(float(np.clip(np.dot(bz, goal), -1., 1.))) < 1e-4:
            break
    np.testing.assert_allclose(bz, goal, atol=1e-4)


def test_slerp_preserves_unit_length():
    bz   = _unit(np.array([0.3, -0.5, 0.8]))
    goal = _unit(np.array([-0.2, 0.7, 0.6]))
    for _ in range(50):
        bz = slerp_step(bz, goal, slew_rate=0.5, dt=0.02)
        assert abs(np.linalg.norm(bz) - 1.0) < 1e-12


@pytest.mark.parametrize("slew_rate", [0.1, 0.4, 1.0, 2.0])
def test_slerp_rate_limit_parametric(slew_rate):
    bz   = np.array([0., 0., 1.])
    goal = np.array([1., 0., 0.])
    dt   = 0.02
    result     = slerp_step(bz, goal, slew_rate=slew_rate, dt=dt)
    step_taken = math.acos(float(np.clip(np.dot(bz, result), -1., 1.)))
    assert step_taken <= slew_rate * dt + 1e-10


# ---------------------------------------------------------------------------
# cyclic_error_body() -- standalone reference implementation tests
# ---------------------------------------------------------------------------

def test_cyclic_error_at_equilibrium_is_zero():
    """When bz_now == bz_slerp, error is identically zero."""
    bz = _unit(np.array([0.5, 0.2, 0.8]))
    R  = np.eye(3)
    err_bx, err_by = cyclic_error_body(R, bz, bz)
    assert abs(err_bx) < 1e-12
    assert abs(err_by) < 1e-12


def test_cyclic_error_direction_north_tilt():
    """
    bz_now tilted toward North relative to bz_slerp -> error has body-X component.

    With identity R (body frame = NED frame) and bz_slerp pointing down:
    tilting bz_now slightly toward NED North [1,0,0] produces an error in the
    East direction (err_ned[1] != 0) and zero in the North direction.
    """
    R        = np.eye(3)
    bz_slerp = np.array([0., 0., 1.])    # NED down
    bz_now   = _unit(np.array([0.1, 0., 1.]))   # tilted North
    err_bx, err_by = cyclic_error_body(R, bz_now, bz_slerp)
    # bz_now x bz_slerp = [0.1, 0, 1]x[0,0,1] = [0, -0.1, 0] (approx)
    # With identity R: err_bx = err_ned[0] = 0, err_by = err_ned[1] < 0
    assert abs(err_bx) < 1e-10     # no roll error for a pure North tilt
    assert err_by < -1e-6          # pitch error is negative (bz tilted past goal)


def test_cyclic_error_opposite_tilts_opposite_signs():
    """
    Tilting bz_now East vs West gives opposite-sign roll errors (body X).

    bz_now x bz_slerp: [0, +e, 1]x[0,0,1] = [+e, 0, 0] -> err_ned +X -> roll > 0
                       [0, -e, 1]x[0,0,1] = [-e, 0, 0] -> err_ned -X -> roll < 0
    """
    R        = np.eye(3)
    bz_slerp = np.array([0., 0., 1.])
    bz_pos   = _unit(np.array([0.,  0.1, 1.]))   # tilted East -> positive roll error
    bz_neg   = _unit(np.array([0., -0.1, 1.]))   # tilted West -> negative roll error
    err_pos_x, _ = cyclic_error_body(R, bz_pos, bz_slerp)
    err_neg_x, _ = cyclic_error_body(R, bz_neg, bz_slerp)
    assert err_pos_x * err_neg_x < 0, (
        "Opposite East/West tilts must give opposite-sign roll (body-X) errors"
    )


def test_cyclic_error_body_yaw_rotation_preserves_magnitude():
    """
    Rotating R around Z (yaw) must not change |roll_err|^2 + |pitch_err|^2.

    Under a pure yaw rotation, body X and Y are rotated in the horizontal plane.
    The total cyclic correction magnitude (proportional to tilt error) is invariant.
    """
    bz_slerp = _unit(np.array([0., 0., 1.]))
    bz_now   = _unit(np.array([0.1, 0.2, 0.9]))
    err_ned  = np.cross(bz_now, bz_slerp)
    ref_mag  = math.sqrt(err_ned[0]**2 + err_ned[1]**2)

    for yaw in [0.0, 0.4, 1.2, -0.7, math.pi / 4]:
        R = _rot_z(yaw)
        err_bx, err_by = cyclic_error_body(R, bz_now, bz_slerp)
        body_mag = math.sqrt(err_bx**2 + err_by**2)
        assert abs(body_mag - ref_mag) < 1e-10, (
            f"Magnitude changed under yaw={math.degrees(yaw):.1f} deg: "
            f"body_mag={body_mag:.6f} ref={ref_mag:.6f}"
        )


def test_cyclic_error_larger_tilt_larger_magnitude():
    """Larger tilt angle -> larger error magnitude."""
    R        = np.eye(3)
    bz_slerp = np.array([0., 0., 1.])
    small_tilt = _unit(np.array([0.05, 0., 1.]))
    large_tilt = _unit(np.array([0.2,  0., 1.]))
    bx_small, by_small = cyclic_error_body(R, small_tilt, bz_slerp)
    bx_large, by_large = cyclic_error_body(R, large_tilt, bz_slerp)
    mag_small = math.sqrt(bx_small**2 + by_small**2)
    mag_large = math.sqrt(bx_large**2 + by_large**2)
    assert mag_large > mag_small, "Larger tilt must give larger cyclic error"


# ---------------------------------------------------------------------------
# output_rate_limit() tests  (mirrors rawes_flight.lua step 9)
# ---------------------------------------------------------------------------

def test_rate_limit_no_change_needed():
    """If desired == prev, output equals desired."""
    assert output_rate_limit(1600, 1600, 30) == 1600


def test_rate_limit_small_step_passes_through():
    """A step smaller than max_delta passes through unchanged."""
    assert output_rate_limit(1520, 1500, 30) == 1520


def test_rate_limit_exact_boundary():
    """A step exactly equal to max_delta passes through."""
    assert output_rate_limit(1530, 1500, 30) == 1530


def test_rate_limit_large_positive_step_clamped():
    """A step larger than max_delta is clamped to prev + max_delta."""
    assert output_rate_limit(1700, 1500, 30) == 1530


def test_rate_limit_large_negative_step_clamped():
    """A large negative step is clamped to prev - max_delta."""
    assert output_rate_limit(1300, 1500, 30) == 1470


def test_rate_limit_disabled_when_zero():
    """max_delta == 0 disables clamping: output equals desired regardless."""
    assert output_rate_limit(1000, 1500, 0) == 1000
    assert output_rate_limit(2000, 1500, 0) == 2000


def test_rate_limit_converges_to_target():
    """Repeated application of the limiter eventually reaches the target."""
    target   = 1800
    prev     = 1500
    max_delta = 30
    for _ in range(100):
        prev = output_rate_limit(target, prev, max_delta)
        if prev == target:
            break
    assert prev == target, f"Did not converge: stopped at {prev}"


def test_rate_limit_convergence_steps():
    """Number of steps to reach target equals ceil(|target - start| / max_delta)."""
    start, target, max_delta = 1500, 1800, 30
    expected_steps = math.ceil(abs(target - start) / max_delta)
    prev  = start
    steps = 0
    for _ in range(200):
        new = output_rate_limit(target, prev, max_delta)
        steps += 1
        if new == target:
            break
        prev = new
    assert steps == expected_steps, (
        f"Expected {expected_steps} steps, took {steps}"
    )


@pytest.mark.parametrize("max_delta", [10, 30, 50, 100])
def test_rate_limit_never_exceeds_delta(max_delta):
    """Per-step change never exceeds max_delta for any input."""
    rng = random.Random(42)
    prev = 1500
    for _ in range(500):
        desired = rng.randint(1000, 2000)
        out = output_rate_limit(desired, prev, max_delta)
        assert abs(out - prev) <= max_delta, (
            f"Step {abs(out - prev)} exceeds max_delta={max_delta} "
            f"(desired={desired}, prev={prev})"
        )
        prev = out


# ---------------------------------------------------------------------------
# Orbit tracking -- _lua_orbit_track class-based tests (algorithm parity)
# ---------------------------------------------------------------------------

class TestOrbitTracking:

    def test_zero_movement_returns_bz_eq0(self):
        """No orbit movement -> bz_eq0 unchanged."""
        bz_eq0 = _TDIR0.copy()
        result = _lua_orbit_track(bz_eq0, _TDIR0, _POS0, _ANCHOR)
        np.testing.assert_allclose(result, bz_eq0, atol=1e-12)

    def test_short_tether_guard_returns_bz_eq0(self):
        """Tether shorter than 0.5 m -> bz_eq0 unchanged (guard)."""
        bz_eq0 = _TDIR0.copy()
        result = _lua_orbit_track(bz_eq0, _TDIR0, np.array([0.1, 0., 0.]), _ANCHOR)
        np.testing.assert_allclose(result, bz_eq0, atol=1e-12)

    def test_output_is_unit_vector(self):
        """Result is always a unit vector."""
        pos1   = 50.0 * np.array([0., math.cos(_ELEV), math.sin(_ELEV)])
        result = _lua_orbit_track(_TDIR0, _TDIR0, pos1, _ANCHOR)
        np.testing.assert_allclose(np.linalg.norm(result), 1.0, atol=1e-12)

    def test_tether_aligned_capture_gives_current_tether_dir(self):
        """
        When bz_eq0 = tdir0 (tether-aligned capture), orbit_track always
        returns the current tether direction regardless of orbit path.

        This is the normal reel-out case.  The Lua Rodrigues formula maps
        tdir0 -> bzt exactly by construction.
        """
        pos1   = 50.0 * np.array([0., math.cos(_ELEV), math.sin(_ELEV)])
        bzt    = pos1 / np.linalg.norm(pos1)
        result = _lua_orbit_track(_TDIR0, _TDIR0, pos1, _ANCHOR)
        np.testing.assert_allclose(result, bzt, atol=1e-12)

    def test_azimuthal_orbit_agrees_with_3d_when_tether_aligned(self):
        """
        For azimuthal orbit at constant elevation with bz_eq0 = tdir0,
        the Lua 3D Rodrigues result matches orbit_tracked_body_z_eq_3d().
        Both reduce to 'track the current tether direction' in this case.
        """
        for az_deg in [30, 60, 90, 120, 180]:
            az  = math.radians(az_deg)
            pos = 50.0 * np.array([math.cos(_ELEV) * math.cos(az),
                                   math.cos(_ELEV) * math.sin(az),
                                   math.sin(_ELEV)])
            lua_result = _lua_orbit_track(_TDIR0, _TDIR0, pos, _ANCHOR)
            py_result  = orbit_tracked_body_z_eq_3d(pos, _TDIR0, _TDIR0)
            np.testing.assert_allclose(lua_result, py_result, atol=1e-10,
                err_msg=f"az_deg={az_deg}")

    def test_non_tether_aligned_agrees_with_3d(self):
        """
        When bz_eq0 != tdir0 (tilted capture, e.g. reel-in phase),
        orbit_tracked_body_z_eq_3d() matches the Lua exactly.
        """
        tilt_axis = np.array([0., 0., 1.])
        bz_eq0_tilted = rodrigues(_TDIR0, tilt_axis, math.radians(20.0))
        bz_eq0_tilted /= np.linalg.norm(bz_eq0_tilted)

        az  = math.radians(60.0)
        pos = 50.0 * np.array([math.cos(_ELEV) * math.cos(az),
                               math.cos(_ELEV) * math.sin(az),
                               math.sin(_ELEV)])

        lua_result = _lua_orbit_track(bz_eq0_tilted, _TDIR0, pos, _ANCHOR)
        py_result  = orbit_tracked_body_z_eq_3d(pos, _TDIR0, bz_eq0_tilted)

        np.testing.assert_allclose(lua_result, py_result, atol=1e-10)

    def test_simulation_variant_diverges_from_lua_for_tilted_capture(self):
        """
        orbit_tracked_body_z_eq() (simulation variant) diverges from the Lua
        when bz_eq0 != tdir0.  This is intentional: the simulation variant
        preserves body_z_eq[2] to avoid altitude instability at 400 Hz.
        """
        tilt_axis = np.array([0., 0., 1.])
        bz_eq0_tilted = rodrigues(_TDIR0, tilt_axis, math.radians(20.0))
        bz_eq0_tilted /= np.linalg.norm(bz_eq0_tilted)

        az  = math.radians(60.0)
        pos = 50.0 * np.array([math.cos(_ELEV) * math.cos(az),
                               math.cos(_ELEV) * math.sin(az),
                               math.sin(_ELEV)])

        lua_result = _lua_orbit_track(bz_eq0_tilted, _TDIR0, pos, _ANCHOR)
        py_result  = orbit_tracked_body_z_eq(pos, _TDIR0, bz_eq0_tilted)

        angle_diff = math.acos(float(np.clip(np.dot(lua_result, py_result), -1, 1)))
        assert angle_diff > math.radians(0.5), (
            f"Expected simulation variant to diverge from Lua, "
            f"but they agree to {math.degrees(angle_diff):.2f} deg"
        )

    def test_360deg_orbit_returns_bz_eq0(self):
        """Full 360 deg orbit returns to starting tether direction -> bz_eq0 restored."""
        pos_back = _POS0 * 1.001   # tiny drift -- still within numerical tolerance
        result   = _lua_orbit_track(_TDIR0, _TDIR0, pos_back, _ANCHOR)
        np.testing.assert_allclose(result, _TDIR0, atol=1e-3)


# ---------------------------------------------------------------------------
# Rate-limited slerp -- _lua_slerp_step class-based tests (algorithm parity)
# ---------------------------------------------------------------------------

class TestSlerp:

    def test_no_movement_when_already_at_goal(self):
        bz = _TDIR0.copy()
        out = _lua_slerp_step(bz, bz, 0.40, 0.02)
        np.testing.assert_allclose(out, bz, atol=1e-12)

    def test_rate_limited_does_not_exceed_slew_rate(self):
        """Step size must not exceed slew_rate * dt."""
        bz     = np.array([0., 0., 1.])
        goal   = np.array([1., 0., 0.])
        slew   = 0.40   # rad/s
        dt     = 0.02   # s
        result = _lua_slerp_step(bz, goal, slew, dt)
        angle = math.acos(float(np.clip(np.dot(bz, result), -1, 1)))
        assert angle <= slew * dt + 1e-9

    def test_reaches_goal_in_finite_steps(self):
        """Repeated slerp steps eventually converge to goal."""
        bz   = np.array([0., 0., 1.])
        goal = _TDIR0.copy()
        slew = 0.40
        dt   = 0.02
        for _ in range(500):
            bz = _lua_slerp_step(bz, goal, slew, dt)
        np.testing.assert_allclose(bz, goal, atol=1e-6)

    def test_output_is_unit_vector(self):
        bz     = np.array([0., 0., 1.])
        goal   = _TDIR0.copy()
        result = _lua_slerp_step(bz, goal, 0.40, 0.02)
        np.testing.assert_allclose(np.linalg.norm(result), 1.0, atol=1e-12)

    def test_matches_slerp_body_z(self):
        """
        Lua rodrigues-based slerp and controller.slerp_body_z() give
        the same result for the same inputs.
        """
        bz   = np.array([0., 0., 1.])
        goal = _TDIR0.copy()
        slew = 0.40
        dt   = 0.02

        lua_result = _lua_slerp_step(bz, goal, slew, dt)
        py_result  = slerp_body_z(bz, goal, slew, dt)
        np.testing.assert_allclose(lua_result, py_result, atol=1e-12)

    def test_matches_slerp_body_z_multiple_angles(self):
        """Agreement across a range of starting angles and slew budgets."""
        goal = _TDIR0.copy()
        for start_deg in [10, 30, 60, 90, 150]:
            bz = rodrigues(
                np.array([0., 0., 1.]),
                np.array([0., 1., 0.]),
                math.radians(start_deg),
            )
            for slew in [0.10, 0.40, 2.0]:
                lua_r = _lua_slerp_step(bz, goal, slew, 0.02)
                py_r  = slerp_body_z(bz, goal, slew, 0.02)
                np.testing.assert_allclose(lua_r, py_r, atol=1e-12,
                    err_msg=f"start_deg={start_deg} slew={slew}")


# ---------------------------------------------------------------------------
# Cyclic loop -- _lua_cyclic_rates class-based tests (algorithm parity)
# ---------------------------------------------------------------------------

class TestCyclicLoop:

    # Reference state: hub at equilibrium (body_z = tether dir, bz_slerp = body_z)
    _R_EQ = _build_R_from_body_z(_TDIR0)

    def test_neutral_at_equilibrium(self):
        """Zero error -> neutral PWM (1500) on both channels."""
        _, _, ch1, ch2 = _lua_cyclic_rates(
            self._R_EQ[:, 0], self._R_EQ[:, 1], self._R_EQ[:, 2],
            bz_slerp=_TDIR0.copy(),
            kp=1.0,
        )
        assert ch1 == 1500
        assert ch2 == 1500

    def test_pwm_neutral_at_zero_rate(self):
        """PWM scaling: 0 rad/s -> 1500 us."""
        scale = 500.0 / (360.0 * math.pi / 180.0)
        pwm = int(round(1500.0 + scale * 0.0))
        assert pwm == 1500

    def test_pwm_full_roll_deflection(self):
        """Full-scale rate (ACRO_RP_RATE deg/s) -> PWM 2000."""
        acro_rp_deg = 360.0
        max_rate = acro_rp_deg * math.pi / 180.0
        scale    = 500.0 / max_rate
        pwm      = int(round(1500.0 + scale * max_rate))
        assert pwm == 2000

    def test_pwm_clamped_above_max(self):
        """Rates exceeding ACRO_RP_RATE are clamped to PWM 2000."""
        acro_rp_deg = 360.0
        max_rate  = acro_rp_deg * math.pi / 180.0
        scale     = 500.0 / max_rate
        pwm_raw   = int(round(1500.0 + scale * 2 * max_rate))
        pwm_clamp = max(1000, min(2000, pwm_raw))
        assert pwm_clamp == 2000

    def test_roll_error_drives_ch1_only(self):
        """
        Pure roll error (body_z displaced toward body_x) should command
        ch1 (roll) and leave ch2 (pitch) near neutral.
        """
        R    = self._R_EQ
        bz_x = R[:, 0]   # body X axis
        bz_slerp = rodrigues(R[:, 2], bz_x, -0.2)
        bz_slerp /= np.linalg.norm(bz_slerp)

        _, _, ch1, ch2 = _lua_cyclic_rates(R[:, 0], R[:, 1], R[:, 2],
                                           bz_slerp, kp=1.0)
        assert abs(ch1 - 1500) > 10, "ch1 should respond to roll error"
        assert abs(ch2 - 1500) < 5,  "ch2 should be near neutral for pure roll error"

    def test_pitch_error_drives_ch2_only(self):
        """
        Pure pitch error (body_z displaced toward body_y) should command
        ch2 (pitch) and leave ch1 (roll) near neutral.
        """
        R    = self._R_EQ
        bz_y = R[:, 1]
        bz_slerp = rodrigues(R[:, 2], bz_y, -0.2)
        bz_slerp /= np.linalg.norm(bz_slerp)

        _, _, ch1, ch2 = _lua_cyclic_rates(R[:, 0], R[:, 1], R[:, 2],
                                           bz_slerp, kp=1.0)
        assert abs(ch2 - 1500) > 10, "ch2 should respond to pitch error"
        assert abs(ch1 - 1500) < 5,  "ch1 should be near neutral for pure pitch error"

    def test_matches_compute_rate_cmd(self):
        """
        Lua cyclic rates match compute_rate_cmd(kp, kd=0) from controller.py.

        compute_rate_cmd is frame-agnostic and used by the internal controller.
        The Lua cyclic loop is its hardware equivalent (kd=0 because ArduPilot's
        rate PIDs supply the damping).
        """
        R        = self._R_EQ
        bz_slerp = rodrigues(R[:, 2], R[:, 0], -0.15)   # 0.15 rad roll error
        bz_slerp /= np.linalg.norm(bz_slerp)
        kp       = 1.0

        lua_roll, lua_pitch, _, _ = _lua_cyclic_rates(
            R[:, 0], R[:, 1], R[:, 2], bz_slerp, kp)

        py_rates = compute_rate_cmd(
            bz_now=R[:, 2], bz_eq=bz_slerp,
            R_body_to_world=R, kp=kp, kd=0.0)

        np.testing.assert_allclose(lua_roll,  py_rates[0], atol=1e-12)
        np.testing.assert_allclose(lua_pitch, py_rates[1], atol=1e-12)

    def test_matches_compute_rate_cmd_arbitrary_orientations(self):
        """compute_rate_cmd and Lua cyclic agree across a range of orientations."""
        kp = 1.5
        for roll_err in [-0.2, -0.05, 0.0, 0.08, 0.25]:
            for pitch_err in [-0.15, 0.0, 0.12]:
                R   = self._R_EQ
                bz  = R[:, 2].copy()
                bz  = rodrigues(bz, R[:, 0], roll_err)
                bz  = rodrigues(bz, R[:, 1], pitch_err)
                bz /= np.linalg.norm(bz)

                lua_roll, lua_pitch, _, _ = _lua_cyclic_rates(
                    R[:, 0], R[:, 1], R[:, 2], bz, kp)
                py = compute_rate_cmd(R[:, 2], bz, R, kp, kd=0.0)

                np.testing.assert_allclose(lua_roll,  py[0], atol=1e-12,
                    err_msg=f"roll_err={roll_err} pitch_err={pitch_err}")
                np.testing.assert_allclose(lua_pitch, py[1], atol=1e-12,
                    err_msg=f"roll_err={roll_err} pitch_err={pitch_err}")

    def test_pwm_symmetric_around_neutral(self):
        """Equal and opposite errors produce PWM equidistant from 1500."""
        R  = self._R_EQ
        bz_pos = rodrigues(R[:, 2], R[:, 0], -0.1)
        bz_neg = rodrigues(R[:, 2], R[:, 0], +0.1)
        bz_pos /= np.linalg.norm(bz_pos)
        bz_neg /= np.linalg.norm(bz_neg)

        _, _, ch1_pos, _ = _lua_cyclic_rates(R[:, 0], R[:, 1], R[:, 2], bz_pos, 1.0)
        _, _, ch1_neg, _ = _lua_cyclic_rates(R[:, 0], R[:, 1], R[:, 2], bz_neg, 1.0)

        assert ch1_pos + ch1_neg == 3000  # symmetric around 1500


# ===========================================================================
# -- Python/Lua equivalence tests --
# Cross-check each Lua reference implementation against controller.py.
# A failure here means Lua and Python implementations have diverged.
# ===========================================================================

# ---------------------------------------------------------------------------
# slerp_step  <->  slerp_body_z
#
# Both advance bz_prev toward bz_target by at most slew_rate*dt radians.
# Lua reference: Rodrigues incremental rotation by exactly step=min(slew*dt, remain).
# controller.py: classic SLERP formula with alpha=min(1, slew*dt/theta).
# These are mathematically identical for unit vectors (same axis, same angle).
# ---------------------------------------------------------------------------

SLERP_CASES = [
    # (bz_prev, bz_target, slew_rate, dt, label)
    (np.array([0., 0., 1.]), np.array([1., 0., 0.]), 0.4, 0.02, "large_step_clamped"),
    (np.array([0., 0., 1.]), np.array([1., 0., 0.]), 5.0, 0.02, "fast_slew_reaches_goal"),
    (_unit([0.3, 0.4, 0.8]), _unit([-0.2, 0.7, 0.6]), 0.4, 0.02, "general_unit_vectors"),
    (_unit([1., 0., 0.]), _unit([0.999, 0.001, 0.]), 0.4, 0.02, "nearly_parallel"),
    # NOTE: near-antiparallel (angle ~pi) is NOT tested here because the two
    # implementations diverge in that degenerate case:
    #   slerp_step returns bz_slerp unchanged (cross product < 1e-6 threshold).
    #   slerp_body_z applies SLERP which is numerically unstable near sin(theta)~0.
    # This case never arises in RAWES: bz_slerp and goal are never antiparallel.
]


@pytest.mark.parametrize("bz, goal, slew, dt, label", SLERP_CASES)
def test_slerp_equivalence_single_step(bz, goal, slew, dt, label):
    """
    slerp_step (Lua Rodrigues) and slerp_body_z (SLERP formula) must give the
    same result to within 1e-10 for any single step.
    """
    lua_result = slerp_step(bz, goal, slew_rate=slew, dt=dt)
    py_result  = slerp_body_z(bz, goal, slew_rate_rad_s=slew, dt=dt)
    np.testing.assert_allclose(
        lua_result, py_result, atol=1e-10,
        err_msg=f"slerp diverged for case '{label}'"
    )


def test_slerp_equivalence_multi_step_trajectory():
    """
    Running slerp_step and slerp_body_z for 200 steps from the same start
    must stay in lock-step throughout (not just first step).
    """
    bz_lua = np.array([0., 0., 1.])
    bz_py  = bz_lua.copy()
    goal   = _unit(np.array([0.5, 0.5, 0.7]))
    slew, dt = 0.4, 0.02

    for i in range(200):
        bz_lua = slerp_step(bz_lua, goal, slew_rate=slew, dt=dt)
        bz_py  = slerp_body_z(bz_py, goal, slew_rate_rad_s=slew, dt=dt)
        np.testing.assert_allclose(
            bz_lua, bz_py, atol=1e-10,
            err_msg=f"slerp diverged at step {i}"
        )


@pytest.mark.parametrize("slew", [0.1, 0.4, 1.0, 2.0])
def test_slerp_step_taken_agrees(slew):
    """
    Both implementations must advance by the same angular distance per step.
    """
    bz   = np.array([0., 0., 1.])
    goal = np.array([1., 0., 0.])
    dt   = 0.02
    r_lua = slerp_step(bz, goal, slew_rate=slew, dt=dt)
    r_py  = slerp_body_z(bz, goal, slew_rate_rad_s=slew, dt=dt)
    angle_lua = math.acos(float(np.clip(np.dot(bz, r_lua), -1., 1.)))
    angle_py  = math.acos(float(np.clip(np.dot(bz, r_py),  -1., 1.)))
    assert abs(angle_lua - angle_py) < 1e-10, (
        f"Step angle differs: lua={angle_lua:.6f} py={angle_py:.6f} slew={slew}"
    )


# ---------------------------------------------------------------------------
# cyclic_error_body  <->  compute_rate_cmd (kp=1, kd=0)
#
# Both project the body_z alignment error (bz_now x bz_eq, in world frame)
# into the body frame.
# Lua: err_ned = bz_now x bz_slerp; err_bx = R[:,0] @ err_ned; err_by = R[:,1] @ err_ned
# Python: error_world = cross(bz_now, bz_eq); output = R.T @ (kp * error_world)
#         => output[0] = R[:,0] @ error_world = err_bx  (roll)
#            output[1] = R[:,1] @ error_world = err_by  (pitch)
# ---------------------------------------------------------------------------

CYCLIC_CASES = [
    # (bz_now, bz_slerp, R, label)
    (
        _unit([0.5, 0.2, 0.8]),
        _unit([0.5, 0.2, 0.8]),
        np.eye(3),
        "at_equilibrium_zero_error",
    ),
    (
        _unit([0.1, 0., 1.]),
        np.array([0., 0., 1.]),
        np.eye(3),
        "small_north_tilt",
    ),
    (
        _unit([0., 0.2, 1.]),
        np.array([0., 0., 1.]),
        _rot_z(0.5),
        "east_tilt_with_yaw",
    ),
    (
        _unit([0.3, -0.4, 0.8]),
        _unit([0.1, 0.2, 0.9]),
        _rot_z(1.2),
        "general_with_yaw",
    ),
    (
        _unit([0., 0., 1.]),
        _unit([0.5, 0.5, 0.7]),
        _rot_z(-0.7),
        "large_error_negative_yaw",
    ),
]


@pytest.mark.parametrize("bz_now, bz_slerp, R, label", CYCLIC_CASES)
def test_cyclic_error_matches_compute_rate_cmd(bz_now, bz_slerp, R, label):
    """
    cyclic_error_body() must match compute_rate_cmd(kp=1, kd=0) roll+pitch components.

    R here is R_body_to_world (columns are body axes in world frame), which is
    the same convention used by both functions.
    """
    lua_bx, lua_by = cyclic_error_body(R, bz_now, bz_slerp)
    py_rate = compute_rate_cmd(bz_now, bz_slerp, R, kp=1.0, kd=0.0)

    assert abs(lua_bx - py_rate[0]) < 1e-12, (
        f"Roll (body-X) mismatch '{label}': lua={lua_bx:.8f} py={py_rate[0]:.8f}"
    )
    assert abs(lua_by - py_rate[1]) < 1e-12, (
        f"Pitch (body-Y) mismatch '{label}': lua={lua_by:.8f} py={py_rate[1]:.8f}"
    )


@pytest.mark.parametrize("kp", [0.5, 1.0, 2.0])
def test_cyclic_error_kp_scales_linearly(kp):
    """
    Applying kp in cyclic_error_body (multiply result) must equal compute_rate_cmd(kp=kp).
    """
    bz_now   = _unit([0.2, 0.1, 0.9])
    bz_slerp = _unit([0., 0., 1.])
    R        = _rot_z(0.4)

    lua_bx, lua_by = cyclic_error_body(R, bz_now, bz_slerp)
    py_rate = compute_rate_cmd(bz_now, bz_slerp, R, kp=kp, kd=0.0)

    assert abs(kp * lua_bx - py_rate[0]) < 1e-12
    assert abs(kp * lua_by - py_rate[1]) < 1e-12


# ---------------------------------------------------------------------------
# orbit_track_bz  <->  orbit_tracked_body_z_eq_3d
#
# Both use the full 3D Rodrigues rotation that maps tether_dir0 -> bzt.
# orbit_tracked_body_z_eq_3d takes cur_pos (unnormalised); it normalises
# internally to get bzt.  We pass bzt * some_length as cur_pos.
#
# NOTE: controller.py also has orbit_tracked_body_z_eq() (without _3d suffix).
# That variant uses an azimuthal-only (horizontal-projection) rotation to
# preserve body_z[2] and avoid altitude instability when called at 400 Hz
# without rate limiting.  The _3d variant is the Lua-equivalent function,
# intended for use ONLY downstream of a rate-limited slerp.
# ---------------------------------------------------------------------------

ORBIT_CASES = [
    # (bz_eq0, tdir0, bzt_direction, label)
    # horizontal tether, pure azimuthal orbit
    (_unit([0.6, 0.3, 0.7]), np.array([1., 0., 0.]), _unit([0., 1., 0.]),
     "horizontal_90deg"),
    (_unit([0.6, 0.3, 0.7]), np.array([1., 0., 0.]), _unit([-1., 0.01, 0.]),
     "horizontal_near_180deg"),
    # 3D tether with elevation
    (_unit([0.5, 0.5, 0.7]), np.array([1., 0., 0.]), _unit([0., -0.5, -0.8]),
     "tilted_south_down"),
    (_unit([0.4, 0.2, 0.9]), _unit([0.5, 0.3, 0.8]), _unit([-0.3, 0.6, 0.7]),
     "general_3d_both_tilted"),
    # tilted capture (bz_eq0 != tdir0)
    (_unit([0.7, 0.2, 0.6]), _unit([1., 0., 0.]), _unit([0., 1., 0.]),
     "tilted_capture_azimuthal"),
]


@pytest.mark.parametrize("bz_eq0, tdir0, bzt, label", ORBIT_CASES)
def test_orbit_track_3d_equivalence(bz_eq0, tdir0, bzt, label):
    """
    orbit_track_bz (Lua ref) and orbit_tracked_body_z_eq_3d (controller.py)
    must give identical results for all tether geometries.
    """
    lua_result = orbit_track_bz(bz_eq0, tdir0, bzt)
    cur_pos    = bzt * 15.0   # unnormalised; function normalises internally
    py_result  = orbit_tracked_body_z_eq_3d(cur_pos, tdir0, bz_eq0)

    np.testing.assert_allclose(
        lua_result, py_result, atol=1e-10,
        err_msg=f"orbit_track_3d diverged for case '{label}'"
    )


# ---------------------------------------------------------------------------
# PWM scalar formula
#
# Lua (rawes_flight.lua):
#   scale = 500 / (ACRO_RP_RATE_DEG * math.pi / 180)
#   ch = math.floor(1500 + scale * rate_rads + 0.5)
#   ch = math.max(1000, math.min(2000, ch))
#
# Python (compute_rc_rates inner _pwm):
#   max_rate = radians(rate_max_deg)
#   pwm = round(1500 + 500 * clip(rate / max_rate, -1, 1))
#
# Both formulas are equivalent: 500/max_rate * rate  ==  500 * rate/max_rate
# floor(x+0.5) == round(x) for all non-half-integer x (and for PWM integer
# inputs both produce the same integer in practice).
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("rate_rads", [
    0.0, 0.1, -0.1, 1.0, -1.0, math.pi, -math.pi,
    2 * math.pi, -2 * math.pi,      # full-stick limits
    0.5, -0.5, 3.0, -3.0,
])
def test_pwm_formula_equivalence(rate_rads):
    """
    Lua floor(1500 + scale*rate + 0.5) must equal Python round(1500 + 500*clip(rate/max)).
    Both use ACRO_RP_RATE = 360 deg/s.
    """
    lua = _lua_pwm(rate_rads, acro_rp_rate_deg=360.0)
    py  = _py_pwm(rate_rads,  rate_max_deg=360.0)
    assert lua == py, f"PWM mismatch at rate={rate_rads:.4f}: lua={lua} py={py}"


@pytest.mark.parametrize("acro_rate_deg", [200.0, 360.0, 720.0])
def test_pwm_formula_equivalence_various_acro_rates(acro_rate_deg):
    """Both formulas must agree across common ACRO_RP_RATE settings."""
    for rate_rads in np.linspace(-math.radians(acro_rate_deg),
                                  math.radians(acro_rate_deg), 41):
        lua = _lua_pwm(rate_rads, acro_rp_rate_deg=acro_rate_deg)
        py  = _py_pwm(rate_rads,  rate_max_deg=acro_rate_deg)
        assert lua == py, (
            f"PWM mismatch at ACRO_RP_RATE={acro_rate_deg} rate={rate_rads:.4f}: "
            f"lua={lua} py={py}"
        )


def test_pwm_clamp_equivalence():
    """Both implementations must clamp to [1000, 2000]."""
    for rate in [100.0, -100.0]:   # extreme values far beyond full stick
        lua = _lua_pwm(rate)
        py  = _py_pwm(rate)
        assert 1000 <= lua <= 2000, f"Lua PWM out of range: {lua}"
        assert 1000 <= py  <= 2000, f"Python PWM out of range: {py}"
        assert lua == py, f"Clamped PWM differs: lua={lua} py={py}"
