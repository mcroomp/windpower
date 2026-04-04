"""
test_lua_algorithm.py — Algorithm parity: rawes_flight.lua vs controller.py

Validates three algorithms transplanted from Python into rawes_flight.lua:

  A. Rodrigues rotation  — pure math; used for orbit tracking and slerp
  B. orbit_track()       — 3D tether rotation applied to body_z
  C. slerp step          — rate-limited body_z convergence
  D. cyclic loop         — body_z error → body-frame rates → RC PWM

Python transcriptions of the Lua functions live in this file as module-level
helpers so the comparison is exact (same arithmetic, same algorithm).

Design note — two orbit-tracking functions in controller.py:
  orbit_tracked_body_z_eq()     azimuthal-only rotation (preserves Z, stable at
      400 Hz without rate limiting — used in the Python simulation loop)
  orbit_tracked_body_z_eq_3d()  full 3D Rodrigues rotation — matches
      rawes_flight.lua::orbit_track() exactly; safe only when downstream slerp
      limits bandwidth (as the Lua's 0.40 rad/s slerp does on hardware)

Tests B-3/B-4 verify orbit_tracked_body_z_eq_3d against the Lua reference;
B-5 documents the intentional divergence of the simulation variant.

No SITL, no Docker.  Runs with the existing unit-test venv.
"""
import math
import sys
from pathlib import Path

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from controller import (
    orbit_tracked_body_z_eq,
    orbit_tracked_body_z_eq_3d,
    slerp_body_z,
    compute_rate_cmd,
)

# ---------------------------------------------------------------------------
# Python transcriptions of rawes_flight.lua functions
# These must stay bit-for-bit identical to the Lua source.
# ---------------------------------------------------------------------------

def _rodrigues(v: np.ndarray, axis_n: np.ndarray, angle: float) -> np.ndarray:
    """
    Rodrigues rotation: rotate v around unit vector axis_n by angle radians.

    Lua source (rawes_flight.lua):
        local ca  = math.cos(angle)
        local sa  = math.sin(angle)
        local ac  = axis_n:cross(v)
        local ad  = axis_n:dot(v)
        return v * ca + ac * sa + axis_n * (ad * (1.0 - ca))
    """
    ca = math.cos(angle)
    sa = math.sin(angle)
    ac = np.cross(axis_n, v)
    ad = float(np.dot(axis_n, v))
    return v * ca + ac * sa + axis_n * (ad * (1.0 - ca))


def _lua_orbit_track(
    bz_eq0: np.ndarray,
    tdir0:  np.ndarray,
    pos:    np.ndarray,
    anchor: np.ndarray,
) -> np.ndarray:
    """
    Python transcription of rawes_flight.lua orbit_track().

    Rotates bz_eq0 by the same 3D rotation that maps tdir0 → current
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
    return _rodrigues(bz_eq0, axis / sinth, math.atan2(sinth, costh))


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
    return _rodrigues(bz_slerp, ax / axn, step)


def _lua_cyclic_rates(
    R_col_x:    np.ndarray,   # R:colx() — body X axis in world frame
    R_col_y:    np.ndarray,   # R:coly() — body Y axis in world frame
    R_col_z:    np.ndarray,   # R:colz() — body Z axis in world frame
    bz_slerp:   np.ndarray,
    kp:         float,
    acro_rp_deg: float = 360.0,
) -> tuple[float, float, int, int]:
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


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def _build_R_from_body_z(bz: np.ndarray) -> np.ndarray:
    """Rotation matrix whose third column is bz (arbitrary body X/Y)."""
    bz = bz / np.linalg.norm(bz)
    # Pick an arbitrary X that is not parallel to bz
    ref = np.array([0., 1., 0.]) if abs(bz[1]) < 0.9 else np.array([1., 0., 0.])
    bx  = np.cross(ref, bz);  bx /= np.linalg.norm(bx)
    by  = np.cross(bz, bx);   by /= np.linalg.norm(by)
    return np.column_stack([bx, by, bz])


# Reference hub: 50 m tether, 30° elevation, East of anchor
_ELEV   = math.radians(30.0)
_ANCHOR = np.zeros(3)
_POS0   = 50.0 * np.array([math.cos(_ELEV), 0., math.sin(_ELEV)])   # [E, N, U] or generic world
_TDIR0  = _POS0 / np.linalg.norm(_POS0)


# ===========================================================================
# A — Rodrigues rotation
# ===========================================================================

class TestRodrigues:

    def test_identity_zero_angle(self):
        v    = np.array([1., 0., 0.])
        axis = np.array([0., 0., 1.])
        out  = _rodrigues(v, axis, 0.0)
        np.testing.assert_allclose(out, v, atol=1e-12)

    def test_90deg_about_z(self):
        """rotate [1,0,0] by +90° around Z → [0,1,0]"""
        v    = np.array([1., 0., 0.])
        axis = np.array([0., 0., 1.])
        out  = _rodrigues(v, axis, math.pi / 2)
        np.testing.assert_allclose(out, [0., 1., 0.], atol=1e-12)

    def test_180deg_about_z(self):
        v    = np.array([1., 0., 0.])
        axis = np.array([0., 0., 1.])
        out  = _rodrigues(v, axis, math.pi)
        np.testing.assert_allclose(out, [-1., 0., 0.], atol=1e-12)

    def test_90deg_about_x(self):
        """rotate [0,1,0] by +90° around X → [0,0,1]"""
        v    = np.array([0., 1., 0.])
        axis = np.array([1., 0., 0.])
        out  = _rodrigues(v, axis, math.pi / 2)
        np.testing.assert_allclose(out, [0., 0., 1.], atol=1e-12)

    def test_output_is_unit_length(self):
        """Rodrigues rotation preserves vector length."""
        v    = np.array([0.6, 0.8, 0.])
        axis = np.array([0., 0., 1.])
        out  = _rodrigues(v, axis, 1.1)
        np.testing.assert_allclose(np.linalg.norm(out), np.linalg.norm(v), atol=1e-12)

    def test_recovers_original_after_360deg(self):
        v    = np.array([0.577, 0.577, 0.577])
        v   /= np.linalg.norm(v)
        axis = np.array([0., 1., 0.])
        out  = _rodrigues(v, axis, 2 * math.pi)
        np.testing.assert_allclose(out, v, atol=1e-12)

    def test_rotate_parallel_to_axis_unchanged(self):
        """Rotating a vector parallel to the axis leaves it unchanged."""
        axis = np.array([0., 0., 1.])
        out  = _rodrigues(axis.copy(), axis, math.pi / 3)
        np.testing.assert_allclose(out, axis, atol=1e-12)


# ===========================================================================
# B — Orbit tracking
# ===========================================================================

class TestOrbitTracking:

    def test_zero_movement_returns_bz_eq0(self):
        """No orbit movement → bz_eq0 unchanged."""
        bz_eq0 = _TDIR0.copy()
        result = _lua_orbit_track(bz_eq0, _TDIR0, _POS0, _ANCHOR)
        np.testing.assert_allclose(result, bz_eq0, atol=1e-12)

    def test_short_tether_guard_returns_bz_eq0(self):
        """Tether shorter than 0.5 m → bz_eq0 unchanged (guard)."""
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
        tdir0 → bzt exactly by construction.
        """
        # Hub has orbited 90° azimuthally at the same elevation
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
        bz_eq0_tilted = _rodrigues(_TDIR0, tilt_axis, math.radians(20.0))
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
        bz_eq0_tilted = _rodrigues(_TDIR0, tilt_axis, math.radians(20.0))
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
        """Full 360° orbit returns to starting tether direction → bz_eq0 restored."""
        # If bz_eq0 = tdir0, after a full orbit pos returns to ~_POS0
        # and the result should be back to tdir0.
        pos_back = _POS0 * 1.001   # tiny drift — still within numerical tolerance
        result   = _lua_orbit_track(_TDIR0, _TDIR0, pos_back, _ANCHOR)
        np.testing.assert_allclose(result, _TDIR0, atol=1e-3)


# ===========================================================================
# C — Rate-limited slerp
# ===========================================================================

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
        # Angle advanced
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
            bz = _rodrigues(
                np.array([0., 0., 1.]),
                np.array([0., 1., 0.]),
                math.radians(start_deg),
            )
            for slew in [0.10, 0.40, 2.0]:
                lua_r = _lua_slerp_step(bz, goal, slew, 0.02)
                py_r  = slerp_body_z(bz, goal, slew, 0.02)
                np.testing.assert_allclose(lua_r, py_r, atol=1e-12,
                    err_msg=f"start_deg={start_deg} slew={slew}")


# ===========================================================================
# D — Cyclic loop: body_z error → body rates → RC PWM
# ===========================================================================

class TestCyclicLoop:

    # Reference state: hub at equilibrium (body_z = tether dir, bz_slerp = body_z)
    _R_EQ = _build_R_from_body_z(_TDIR0)

    def test_neutral_at_equilibrium(self):
        """Zero error → neutral PWM (1500) on both channels."""
        _, _, ch1, ch2 = _lua_cyclic_rates(
            self._R_EQ[:, 0], self._R_EQ[:, 1], self._R_EQ[:, 2],
            bz_slerp=_TDIR0.copy(),
            kp=1.0,
        )
        assert ch1 == 1500
        assert ch2 == 1500

    def test_pwm_neutral_at_zero_rate(self):
        """PWM scaling: 0 rad/s → 1500 µs."""
        scale = 500.0 / (360.0 * math.pi / 180.0)
        pwm = int(round(1500.0 + scale * 0.0))
        assert pwm == 1500

    def test_pwm_full_roll_deflection(self):
        """Full-scale rate (ACRO_RP_RATE deg/s) → PWM 2000."""
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
        # Tilt bz_slerp toward body_x by 0.2 rad (gives ~16 µs deflection)
        bz_slerp = _rodrigues(R[:, 2], bz_x, -0.2)
        bz_slerp /= np.linalg.norm(bz_slerp)

        _, _, ch1, ch2 = _lua_cyclic_rates(R[:, 0], R[:, 1], R[:, 2],
                                           bz_slerp, kp=1.0)
        # ch1 should deflect from neutral, ch2 should stay near neutral
        assert abs(ch1 - 1500) > 10, "ch1 should respond to roll error"
        assert abs(ch2 - 1500) < 5,  "ch2 should be near neutral for pure roll error"

    def test_pitch_error_drives_ch2_only(self):
        """
        Pure pitch error (body_z displaced toward body_y) should command
        ch2 (pitch) and leave ch1 (roll) near neutral.
        """
        R    = self._R_EQ
        bz_y = R[:, 1]
        bz_slerp = _rodrigues(R[:, 2], bz_y, -0.2)
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
        bz_slerp = _rodrigues(R[:, 2], R[:, 0], -0.15)   # 0.15 rad roll error
        bz_slerp /= np.linalg.norm(bz_slerp)
        kp       = 1.0

        # Lua cyclic (returns body rates)
        lua_roll, lua_pitch, _, _ = _lua_cyclic_rates(
            R[:, 0], R[:, 1], R[:, 2], bz_slerp, kp)

        # Python compute_rate_cmd with kd=0 (no damping — matches hardware)
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
                # Apply combined roll+pitch error
                bz  = _rodrigues(bz, R[:, 0], roll_err)
                bz  = _rodrigues(bz, R[:, 1], pitch_err)
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
        bz_pos = _rodrigues(R[:, 2], R[:, 0], -0.1)
        bz_neg = _rodrigues(R[:, 2], R[:, 0], +0.1)
        bz_pos /= np.linalg.norm(bz_pos)
        bz_neg /= np.linalg.norm(bz_neg)

        _, _, ch1_pos, _ = _lua_cyclic_rates(R[:, 0], R[:, 1], R[:, 2], bz_pos, 1.0)
        _, _, ch1_neg, _ = _lua_cyclic_rates(R[:, 0], R[:, 1], R[:, 2], bz_neg, 1.0)

        assert ch1_pos + ch1_neg == 3000  # symmetric around 1500
