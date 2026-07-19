"""
test_controller.py — Unit tests for simulation/controller.py.

Covers compute_bz_tether, slerp_body_z, compute_rate_cmd, and related
geometry helpers.
"""
import sys
from pathlib import Path

import numpy as np
import pytest


from simulation.controller import (
    compute_bz_tether,
    slerp_body_z,
    compute_rate_cmd,
    compute_rate_cmd_sqrt,
)


# ── Helpers ──────────────────────────────────────────────────────────────────

def _Rz(angle_rad):
    """Rotation matrix about Z axis."""
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


# ---------------------------------------------------------------------------
# compute_bz_tether tests
# ---------------------------------------------------------------------------

def test_bz_tether_unit_vector():
    """Result is always a unit vector."""
    bz = compute_bz_tether([3., 4., 0.], [0., 0., 0.])
    assert bz is not None
    assert abs(np.linalg.norm(bz) - 1.0) < 1e-12


def test_bz_tether_direction():
    """FRD body_z points hub→anchor.  Hub along +X from anchor → [-1,0,0]."""
    bz = compute_bz_tether([10., 0., 0.], [0., 0., 0.])
    np.testing.assert_allclose(bz, [-1., 0., 0.], atol=1e-12)


def test_bz_tether_with_nonzero_anchor():
    """Hub 5 m above anchor at [10,10,0] → FRD body_z points DOWN (anchor side)."""
    bz = compute_bz_tether([10., 10., 5.], [10., 10., 0.])
    np.testing.assert_allclose(bz, [0., 0., -1.], atol=1e-12)


def test_bz_tether_degenerate_returns_none():
    """Hub at anchor (within 0.1 m) → None."""
    assert compute_bz_tether([0., 0., 0.], [0., 0., 0.]) is None
    assert compute_bz_tether([0.05, 0., 0.], [0., 0., 0.]) is None


def test_bz_tether_ned_frame():
    """FRD body_z points hub→anchor: hub 30 m north of anchor → [-1,0,0]."""
    bz = compute_bz_tether([30., 0., 0.], [0., 0., 0.])
    np.testing.assert_allclose(bz, [-1., 0., 0.], atol=1e-12)


# ---------------------------------------------------------------------------
# slerp_body_z tests
# ---------------------------------------------------------------------------

def test_slerp_identical_vectors():
    """Identical vectors → returns the target unchanged."""
    bz = np.array([0., 0., 1.])
    out = slerp_body_z(bz, bz, slew_rate_rad_s=1.0, dt=0.01)
    np.testing.assert_allclose(out, bz, atol=1e-12)


def test_slerp_result_is_unit_vector():
    """Output is always a unit vector regardless of angle."""
    bz_from = np.array([1., 0., 0.])
    bz_to   = np.array([0., 1., 0.])
    for dt in (0.001, 0.01, 0.1, 1.0, 100.0):
        out = slerp_body_z(bz_from, bz_to, slew_rate_rad_s=0.5, dt=dt)
        assert abs(np.linalg.norm(out) - 1.0) < 1e-12


def test_slerp_advances_at_most_one_step():
    """With a 10° target and 1 deg/s slew rate, 1 s step → exactly 1° advance."""
    bz_from = np.array([0., 0., 1.])
    # 10° off in the XZ plane
    angle_target = np.radians(10.0)
    bz_to = np.array([np.sin(angle_target), 0., np.cos(angle_target)])
    slew  = np.radians(1.0)   # 1 deg/s
    out   = slerp_body_z(bz_from, bz_to, slew_rate_rad_s=slew, dt=1.0)
    advanced = float(np.arccos(np.clip(np.dot(bz_from, out), -1, 1)))
    np.testing.assert_allclose(advanced, slew * 1.0, atol=1e-10)


def test_slerp_large_dt_reaches_target():
    """Large dt → result equals bz_target."""
    bz_from = np.array([1., 0., 0.])
    bz_to   = np.array([0., 0., 1.])
    out = slerp_body_z(bz_from, bz_to, slew_rate_rad_s=10.0, dt=1000.0)
    np.testing.assert_allclose(out, bz_to, atol=1e-12)


def test_slerp_monotonically_approaches_target():
    """Successive steps reduce the angle to bz_target."""
    bz      = np.array([1., 0., 0.])
    bz_to   = np.array([0., 0., 1.])
    slew    = np.radians(5.0)
    prev_angle = float(np.arccos(np.dot(bz, bz_to)))
    for _ in range(20):
        bz    = slerp_body_z(bz, bz_to, slew, dt=0.02)
        angle = float(np.arccos(np.clip(np.dot(bz, bz_to), -1, 1)))
        assert angle <= prev_angle + 1e-10
        prev_angle = angle


# ---------------------------------------------------------------------------
# compute_rate_cmd tests
# ---------------------------------------------------------------------------

# Identity rotation: body frame = world frame
_I3 = np.eye(3)


def test_rate_cmd_aligned_gives_zero():
    """bz_now == bz_eq → zero rate command."""
    bz = np.array([0., 0., 1.])
    rates = compute_rate_cmd(bz, bz, _I3, kp=1.0)
    np.testing.assert_allclose(rates, [0., 0., 0.], atol=1e-12)


def test_rate_cmd_opposite_tilts_give_opposite_rates():
    """Tilting +5° vs −5° off bz_eq produces mirrored rate commands."""
    bz_eq  = np.array([0., 0., 1.])
    bz_pos = np.array([np.sin(np.radians( 5.)), 0., np.cos(np.radians( 5.))])
    bz_neg = np.array([np.sin(np.radians(-5.)), 0., np.cos(np.radians(-5.))])
    r_pos  = compute_rate_cmd(bz_pos, bz_eq, _I3, kp=1.0, kd=0.0)
    r_neg  = compute_rate_cmd(bz_neg, bz_eq, _I3, kp=1.0, kd=0.0)
    np.testing.assert_allclose(r_pos + r_neg, [0., 0., 0.], atol=1e-12)


def test_rate_cmd_larger_error_larger_rate():
    """A larger tilt angle produces a proportionally larger rate command."""
    bz_eq    = np.array([0., 0., 1.])
    bz_small = np.array([np.sin(np.radians( 5.)), 0., np.cos(np.radians( 5.))])
    bz_large = np.array([np.sin(np.radians(20.)), 0., np.cos(np.radians(20.))])
    r_small  = np.linalg.norm(compute_rate_cmd(bz_small, bz_eq, _I3, kp=1.0))
    r_large  = np.linalg.norm(compute_rate_cmd(bz_large, bz_eq, _I3, kp=1.0))
    assert r_large > r_small


def test_rate_cmd_spin_not_damped():
    """Pure spin along bz_now is stripped; kd has no effect on it."""
    bz = np.array([0., 0., 1.])
    omega_spin = np.array([0., 0., 25.])   # 25 rad/s along bz_now
    r_no_spin  = compute_rate_cmd(bz, bz, _I3, kp=1.0, kd=1.0, omega_world=omega_spin)
    np.testing.assert_allclose(r_no_spin, [0., 0., 0.], atol=1e-12)


def test_rate_cmd_orbital_rate_damped():
    """Orbital omega (perpendicular to bz_now) contributes damping when kd > 0."""
    bz          = np.array([0., 0., 1.])
    omega_orb   = np.array([0.3, 0., 0.])  # orbital, not spin
    rates_no_kd = compute_rate_cmd(bz, bz, _I3, kp=0.0, kd=0.0, omega_world=omega_orb)
    rates_kd    = compute_rate_cmd(bz, bz, _I3, kp=0.0, kd=1.0, omega_world=omega_orb)
    np.testing.assert_allclose(rates_no_kd, [0., 0., 0.], atol=1e-12)
    assert np.linalg.norm(rates_kd) > 0


def test_rate_cmd_transforms_into_body_frame():
    """Result is in body frame: rotating R by 90° about Z flips roll/pitch."""
    bz_now = np.array([0., 0., 1.])
    bz_eq  = np.array([np.sin(np.radians(5.)), 0., np.cos(np.radians(5.))])
    R_world = _I3
    R_rot90 = _Rz(np.radians(90.0))   # body rotated 90° about Z in world
    r_world = compute_rate_cmd(bz_now, bz_eq, R_world, kp=1.0)
    r_rot90 = compute_rate_cmd(bz_now, bz_eq, R_rot90, kp=1.0)
    # Norms must be equal (same physical correction magnitude)
    np.testing.assert_allclose(np.linalg.norm(r_world), np.linalg.norm(r_rot90), atol=1e-10)
    # But directions differ (frame has rotated)
    assert not np.allclose(r_world, r_rot90, atol=1e-6)


def test_rate_cmd_sqrt_matches_linear_in_small_error_region():
    """Inside sqrt controller's linear region, shaped rate is angle-P."""
    bz_now = np.array([0., 0., 1.])
    angle = np.radians(5.0)
    bz_eq = np.array([np.sin(angle), 0., np.cos(angle)])
    rates = compute_rate_cmd_sqrt(bz_now, bz_eq, _I3, kp=1.0, accel_max=10.0, dt=0.02)
    assert np.linalg.norm(rates) == pytest.approx(angle, rel=1e-6)


def test_rate_cmd_sqrt_limits_large_error_below_linear_rate():
    """Large errors are sqrt-shaped below the raw kp*angle demand."""
    bz_now = np.array([0., 0., 1.])
    angle = np.radians(60.0)
    bz_eq = np.array([np.sin(angle), 0., np.cos(angle)])
    kp = 2.5
    rates = compute_rate_cmd_sqrt(bz_now, bz_eq, _I3, kp=kp, accel_max=1.0, dt=0.02)
    assert np.linalg.norm(rates) < kp * angle


def test_rate_cmd_sqrt_transforms_into_body_frame():
    """Sqrt-shaped helper still returns body-frame rates."""
    bz_now = np.array([0., 0., 1.])
    bz_eq = np.array([np.sin(np.radians(10.0)), 0., np.cos(np.radians(10.0))])
    r_world = compute_rate_cmd_sqrt(bz_now, bz_eq, _I3, kp=1.0, accel_max=10.0, dt=0.02)
    r_rot90 = compute_rate_cmd_sqrt(bz_now, bz_eq, _Rz(np.radians(90.0)), kp=1.0, accel_max=10.0, dt=0.02)
    np.testing.assert_allclose(np.linalg.norm(r_world), np.linalg.norm(r_rot90), atol=1e-10)
    assert not np.allclose(r_world, r_rot90, atol=1e-6)
