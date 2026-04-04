"""
test_lua_python_equivalence.py -- Cross-check rawes_flight.lua math against controller.py

Each test compares the Python reference implementation of a Lua function
(from test_lua_flight_logic.py) against the corresponding controller.py function.

Purpose: catch divergence when one side is updated without updating the other.
If a test here fails it means the Lua and Python implementations no longer agree.

Functions tested:
  slerp_step      (Lua ref) <-> slerp_body_z          (controller.py)
  cyclic_error_body (Lua ref) <-> compute_rate_cmd kp=1 kd=0 (controller.py)
  orbit_track_bz  (Lua ref) <-> orbit_tracked_body_z_eq (controller.py)
      NOTE: these use different algorithms; they only agree when the tether is
      horizontal.  The divergence is intentional and documented here.
  PWM scalar formula (Lua) <-> _pwm formula (controller.py compute_rc_rates)
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

# Add simulation/ to path for controller imports
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
# Add tests/unit/ to path so we can import the Lua reference module
sys.path.insert(0, str(Path(__file__).resolve().parent))

from controller import (
    compute_rate_cmd,
    orbit_tracked_body_z_eq_3d,
    slerp_body_z,
)
from test_lua_flight_logic import (
    cyclic_error_body,
    orbit_track_bz,
    rodrigues,
    slerp_step,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def _rot_z(angle):
    ca, sa = math.cos(angle), math.sin(angle)
    return np.array([[ca, -sa, 0.], [sa, ca, 0.], [0., 0., 1.]])


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

def _lua_pwm(rate_rads: float, acro_rp_rate_deg: float = 360.0) -> int:
    """Mirror of rawes_flight.lua ch1/ch2 computation (pure math, no ArduPilot)."""
    scale = 500.0 / (acro_rp_rate_deg * math.pi / 180.0)
    ch = math.floor(1500.0 + scale * rate_rads + 0.5)
    return max(1000, min(2000, ch))


def _py_pwm(rate_rads: float, rate_max_deg: float = 360.0) -> int:
    """Mirror of compute_rc_rates inner _pwm() (pure math, no hub_state)."""
    max_rate = math.radians(rate_max_deg)
    return int(round(1500.0 + 500.0 * max(-1.0, min(1.0, rate_rads / max_rate))))


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
