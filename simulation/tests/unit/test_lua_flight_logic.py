"""
test_lua_flight_logic.py — unit tests for the math in rawes_flight.lua.

rawes_flight.lua implements four operations:
  1. rodrigues(): rotate a vector around a unit axis by an angle.
  2. orbit_track_bz(): rotate bz_eq0 by the minimal rotation that maps
     the captured tether direction (tdir0) to the current one (bzt),
     keeping body_z aligned with the tether as the hub orbits.
  3. slerp_step(): advance bz_slerp toward a goal at most slew_rate rad/s
     per 50 Hz step.
  4. cyclic_error_body(): transform NED-frame body_z error (bz_now × bz_slerp)
     into body-frame roll/pitch components for the P-gain cyclic loop.

These are Python reference implementations of the Lua functions.
Tests run on Windows with no Docker or ArduPilot dependency.
"""
import math

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Python reference implementations (mirrors rawes_flight.lua exactly)
# ---------------------------------------------------------------------------

def rodrigues(v: np.ndarray, axis_n: np.ndarray, angle: float) -> np.ndarray:
    """
    Rotate vector v around unit vector axis_n by angle radians.

    v' = v·cos(θ) + (axis × v)·sin(θ) + axis·(axis·v)·(1 − cos(θ))

    Mirrors rawes_flight.lua rodrigues() verbatim.
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
    Rotate bz_eq0 by the minimal rotation that maps tdir0 → bzt.

    Mirrors rawes_flight.lua orbit-tracking block:
        axis  = tdir0 × bzt
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
        dot    = clamp(bz_slerp · goal, -1, 1)
        remain = acos(dot)
        if remain > 1e-4:
            ax   = bz_slerp × goal
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
) -> tuple[float, float]:
    """
    Body-frame cyclic error (roll, pitch) from the P-gain loop.

    Mirrors rawes_flight.lua:
        err_ned = bz_now × bz_slerp
        err_bx  = R.col(0) · err_ned    (body X = roll axis)
        err_by  = R.col(1) · err_ned    (body Y = pitch axis)

    R is rotation_body_to_ned (3×3, columns = body axes in NED).
    """
    err_ned = np.cross(bz_now, bz_slerp)
    err_bx  = float(R[:, 0] @ err_ned)
    err_by  = float(R[:, 1] @ err_ned)
    return err_bx, err_by


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def _rot_z(angle: float) -> np.ndarray:
    """3×3 rotation matrix: rotate around NED Z (down) axis."""
    ca, sa = math.cos(angle), math.sin(angle)
    return np.array([[ca, -sa, 0.],
                     [sa,  ca, 0.],
                     [0.,  0., 1.]])


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0.]])


# ---------------------------------------------------------------------------
# rodrigues() tests
# ---------------------------------------------------------------------------

def test_rodrigues_zero_angle_identity():
    v    = np.array([1., 0., 0.])
    axis = np.array([0., 0., 1.])
    np.testing.assert_allclose(rodrigues(v, axis, 0.0), v, atol=1e-12)


def test_rodrigues_90deg_around_z():
    """Rotate [1,0,0] 90° around Z → [0,1,0]."""
    v    = np.array([1., 0., 0.])
    axis = np.array([0., 0., 1.])
    np.testing.assert_allclose(rodrigues(v, axis, math.pi / 2),
                               [0., 1., 0.], atol=1e-12)


def test_rodrigues_180deg_around_z():
    """Rotate [1,0,0] 180° around Z → [-1,0,0]."""
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
    """rodrigues() must equal the equivalent 3×3 rotation matrix × v."""
    angle = 0.4
    axis  = _unit(np.array([1., 2., 3.]))
    v     = np.array([0.6, -0.2, 0.7])
    # Build 3×3 via Rodrigues formula in matrix form
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
# orbit_track_bz() tests
# ---------------------------------------------------------------------------

def test_orbit_track_no_movement():
    """No tether movement → bz_orbit == bz_eq0."""
    bz_eq0 = _unit(np.array([0.5, 0.2, 0.8]))
    tdir0  = _unit(np.array([0.3, 0.4, 0.7]))
    np.testing.assert_allclose(orbit_track_bz(bz_eq0, tdir0, tdir0),
                               bz_eq0, atol=1e-12)


def test_orbit_track_90deg_in_horizontal_plane():
    """
    Tether rotates 90° in the XY plane (tdir0→bzt around Z).

    When tdir0 and bzt are both horizontal (Z=0), the Rodrigues axis
    tdir0 × bzt is [0,0,1], so orbit_track_bz applies exactly R_z(90°) to bz_eq0.
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
            f"after theta={math.degrees(theta):.1f}°: {cos_new:.6f}"
        )


def test_orbit_track_output_is_unit_vector():
    bz_eq0 = _unit(np.array([0.5, 0.2, 0.8]))
    tdir0  = _unit(np.array([0.3, 0.4, 0.7]))
    bzt    = _unit(np.array([-0.1, 0.6, 0.8]))
    result = orbit_track_bz(bz_eq0, tdir0, bzt)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-12


def test_orbit_track_near_antiparallel_tether():
    """Near-antiparallel tdir0 and bzt (almost 180° apart) is handled without crash."""
    tdir0  = _unit(np.array([1., 0., 0.]))
    bzt    = _unit(np.array([-1., 1e-7, 0.]))   # almost opposite
    bz_eq0 = _unit(np.array([0.8, 0., 0.6]))
    result = orbit_track_bz(bz_eq0, tdir0, bzt)
    assert np.linalg.norm(result) > 0.5    # some valid output
    assert math.isfinite(np.linalg.norm(result))


# ---------------------------------------------------------------------------
# slerp_step() tests
# ---------------------------------------------------------------------------

def test_slerp_already_at_goal():
    bz = _unit(np.array([0.3, 0.4, 0.8]))
    np.testing.assert_allclose(slerp_step(bz, bz, slew_rate=0.4, dt=0.02),
                               bz, atol=1e-12)


def test_slerp_clamps_to_rate_limit():
    """Step taken must not exceed slew_rate * dt."""
    bz   = np.array([0., 0., 1.])    # NED down
    goal = np.array([1., 0., 0.])    # NED north (90° away)
    dt, slew = 0.02, 0.4
    result    = slerp_step(bz, goal, slew_rate=slew, dt=dt)
    step_taken = math.acos(float(np.clip(np.dot(bz, result), -1., 1.)))
    assert abs(step_taken - slew * dt) < 1e-10


def test_slerp_takes_full_step_when_close():
    """When remaining angle < slew_rate * dt, jump all the way to goal."""
    bz   = np.array([0., 0., 1.])
    tiny = 0.001   # rad — much less than 0.4 * 0.02 = 0.008 rad
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
# cyclic_error_body() tests
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
    bz_now tilted toward North relative to bz_slerp → error has body-X component.

    With identity R (body frame = NED frame) and bz_slerp pointing down:
    tilting bz_now slightly toward NED North [1,0,0] produces an error in the
    East direction (err_ned[1] ≠ 0) and zero in the North direction.
    """
    R        = np.eye(3)
    bz_slerp = np.array([0., 0., 1.])    # NED down
    bz_now   = _unit(np.array([0.1, 0., 1.]))   # tilted North
    err_bx, err_by = cyclic_error_body(R, bz_now, bz_slerp)
    # bz_now × bz_slerp = [0.1, 0, 1]×[0,0,1] = [0*1-1*0, 1*0-0.1*1, 0.1*0-0*0]
    # = [0, -0.1, 0] (normalized bz_now × bz_slerp points in -Y = -East)
    # With identity R: err_bx = err_ned[0] = 0, err_by = err_ned[1] < 0
    assert abs(err_bx) < 1e-10     # no roll error for a pure North tilt
    assert err_by < -1e-6          # pitch error is negative (bz tilted past goal)


def test_cyclic_error_opposite_tilts_opposite_signs():
    """
    Tilting bz_now East vs West gives opposite-sign roll errors (body X).

    bz_now × bz_slerp: [0, +ε, 1]×[0,0,1] = [+ε, 0, 0] → err_ned points +X → roll > 0
                       [0, -ε, 1]×[0,0,1] = [-ε, 0, 0] → err_ned points -X → roll < 0
    """
    R        = np.eye(3)
    bz_slerp = np.array([0., 0., 1.])
    bz_pos   = _unit(np.array([0.,  0.1, 1.]))   # tilted East → positive roll error
    bz_neg   = _unit(np.array([0., -0.1, 1.]))   # tilted West → negative roll error
    err_pos_x, _ = cyclic_error_body(R, bz_pos, bz_slerp)
    err_neg_x, _ = cyclic_error_body(R, bz_neg, bz_slerp)
    assert err_pos_x * err_neg_x < 0, (
        "Opposite East/West tilts must give opposite-sign roll (body-X) errors"
    )


def test_cyclic_error_body_yaw_rotation_preserves_magnitude():
    """
    Rotating R around Z (yaw) must not change |roll_err|² + |pitch_err|².

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
            f"Magnitude changed under yaw={math.degrees(yaw):.1f}°: "
            f"body_mag={body_mag:.6f} ref={ref_mag:.6f}"
        )


def test_cyclic_error_larger_tilt_larger_magnitude():
    """Larger tilt angle → larger error magnitude."""
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
    import math as _math
    expected_steps = _math.ceil(abs(target - start) / max_delta)
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
    import random
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
