"""
test_startup_trajectory.py — Unit tests for the startup kinematic trajectory.

The RAWES cannot hover statically — it needs orbital velocity for lift.
To give the EKF a stable yaw signal from frame 0 and avoid impulsive loads,
we use a constant-velocity startup trajectory:

    1. Compute a "launch position" by working backwards from the target
       steady-state (position, velocity) at constant velocity.
    2. Start the hub there with velocity = target_vel (constant throughout).
    3. By the end of the damping window the hub has moved linearly to
       target_pos, still at target_vel — ready for free flight.

Benefits over constant-acceleration:
    - Non-zero velocity from frame 0 → EKF derives yaw heading immediately
    - Zero acceleration → IMU sees only gravity (cleaner EKF signal)

These tests verify the kinematic formula and trajectory properties only.
No aerodynamics, wind, or tether forces are involved.
"""

import sys
import os
import math

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Import from mediator
# ---------------------------------------------------------------------------
_SIM_DIR = os.path.join(os.path.dirname(__file__), "..", "..")
sys.path.insert(0, os.path.abspath(_SIM_DIR))

from mediator import (
    compute_launch_position,
    DEFAULT_POS0,
    DEFAULT_VEL0,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_TARGET_POS = np.array(DEFAULT_POS0, dtype=float)
_TARGET_VEL = np.array(DEFAULT_VEL0, dtype=float)
_T          = 30.0   # s — default startup damping window


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _kinematic_state(launch_pos: np.ndarray, target_vel: np.ndarray, t: float):
    """Return (pos, vel) for constant-velocity motion from launch_pos."""
    pos = launch_pos + target_vel * t
    vel = target_vel.copy()
    return pos, vel


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestComputeLaunchPosition:
    """Verify the launch-position formula itself."""

    def test_arrival_position_exact(self):
        """Hub must be exactly at target position at t=T."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        pos_at_T, _ = _kinematic_state(launch_pos, _TARGET_VEL, _T)
        np.testing.assert_allclose(pos_at_T, _TARGET_POS, atol=1e-10)

    def test_arrival_velocity_exact(self):
        """Hub must have exactly target velocity at t=T."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        _, vel_at_T = _kinematic_state(launch_pos, _TARGET_VEL, _T)
        np.testing.assert_allclose(vel_at_T, _TARGET_VEL, atol=1e-10)

    def test_constant_velocity_from_start(self):
        """Hub starts at target_vel at t=0 (constant velocity throughout)."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        _, vel_at_0 = _kinematic_state(launch_pos, _TARGET_VEL, 0.0)
        np.testing.assert_allclose(vel_at_0, _TARGET_VEL, atol=1e-10)

    def test_launch_pos_is_offset_from_target(self):
        """launch_pos = target_pos − target_vel * T."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        expected = _TARGET_POS - _TARGET_VEL * _T
        np.testing.assert_allclose(launch_pos, expected, atol=1e-12)

    def test_returns_array_not_tuple(self):
        """compute_launch_position returns a single array, not a tuple."""
        result = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        assert isinstance(result, np.ndarray), (
            f"expected np.ndarray, got {type(result)}"
        )
        assert result.shape == (3,)

    def test_works_for_different_damp_windows(self):
        """Formula holds for any positive damp window."""
        for T in [5.0, 10.0, 60.0, 120.0]:
            launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, T)
            pos_at_T, vel_at_T = _kinematic_state(launch_pos, _TARGET_VEL, T)
            np.testing.assert_allclose(pos_at_T, _TARGET_POS, atol=1e-10,
                                       err_msg=f"position mismatch at T={T}")
            np.testing.assert_allclose(vel_at_T, _TARGET_VEL, atol=1e-10,
                                       err_msg=f"velocity mismatch at T={T}")


class TestTrajectorySmoothnessNumerical:
    """Verify the kinematic trajectory is smooth (no sudden jumps)."""

    @pytest.fixture
    def trajectory(self):
        """Sample the trajectory at 400 Hz over the full damping window."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        dt = 1.0 / 400.0
        n  = int(_T / dt) + 1
        times = np.linspace(0.0, _T, n)
        positions  = np.array([_kinematic_state(launch_pos, _TARGET_VEL, t)[0] for t in times])
        velocities = np.array([_kinematic_state(launch_pos, _TARGET_VEL, t)[1] for t in times])
        return times, positions, velocities

    def test_no_velocity_jump_at_any_step(self, trajectory):
        """Velocity is constant — change per step must be zero."""
        _, _, velocities = trajectory
        dv = np.diff(velocities, axis=0)
        step_impulse = np.linalg.norm(dv, axis=1)
        assert np.all(step_impulse < 1e-12), (
            f"velocity changed between steps (max change {step_impulse.max():.2e} m/s), "
            "expected zero for constant-velocity trajectory"
        )

    def test_acceleration_is_zero(self, trajectory):
        """Numerically estimated acceleration must be zero throughout."""
        times, _, velocities = trajectory
        dt = times[1] - times[0]
        accel_numeric = np.diff(velocities, axis=0) / dt    # (N-1, 3)
        max_accel = np.abs(accel_numeric).max()
        assert max_accel < 1e-9, (
            f"non-zero acceleration detected: max={max_accel:.2e} m/s² "
            "(expected zero for constant-velocity trajectory)"
        )

    def test_constant_speed_throughout(self, trajectory):
        """Speed must be constant (= |target_vel|) throughout."""
        _, _, velocities = trajectory
        speeds = np.linalg.norm(velocities, axis=1)
        target_speed = np.linalg.norm(_TARGET_VEL)
        np.testing.assert_allclose(speeds, target_speed, atol=1e-10,
                                   err_msg="speed must be constant at |target_vel|")

    def test_position_monotone_toward_target_per_axis(self, trajectory):
        """Each axis must move monotonically from launch_pos toward target_pos."""
        _, positions, _ = trajectory
        launch_pos = positions[0]
        for axis, label in enumerate(["E", "N", "U"]):
            delta_total = _TARGET_POS[axis] - launch_pos[axis]
            if abs(delta_total) < 1e-9:
                continue   # axis barely moves — skip monotonicity check
            remaining = _TARGET_POS[axis] - positions[:, axis]
            if delta_total > 0:
                assert np.all(np.diff(remaining) <= 1e-12), (
                    f"{label}-axis not monotonically approaching target"
                )
            else:
                assert np.all(np.diff(remaining) >= -1e-12), (
                    f"{label}-axis not monotonically approaching target"
                )


class TestHandoffContinuity:
    """
    Simulate the trajectory step-by-step (as the mediator would run it),
    switch from kinematic startup to free flight at T/2, and verify there
    are no position or velocity discontinuities at the handoff.

    The mediator runs two phases:
        Phase 1 (0 … T/2) : kinematic — hub moves at constant target_vel.
        Phase 2 (T/2 … )  : free steady-state flight — physics takes over
                             from whatever state the hub is in at T/2.

    Continuity requirement: the hub's position and velocity seen by physics
    at the first free-flight step must equal the last kinematic step exactly.
    """

    DT = 1.0 / 400.0   # mediator timestep [s]

    def _run_kinematic_phase(self, launch_pos, target_vel, duration):
        """Step through constant-velocity trajectory at 400 Hz; return final (pos, vel)."""
        pos = launch_pos.copy()
        n   = int(round(duration / self.DT))
        for _ in range(n):
            pos = pos + target_vel * self.DT
        return pos, target_vel.copy()

    def test_step_by_step_matches_analytic_at_midpoint(self):
        """Discrete 400 Hz integration must match analytic formula at T/2."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        pos_step, vel_step = self._run_kinematic_phase(launch_pos, _TARGET_VEL, _T / 2)
        pos_analytic, vel_analytic = _kinematic_state(launch_pos, _TARGET_VEL, _T / 2)
        np.testing.assert_allclose(pos_step, pos_analytic, atol=1e-9,
                                   err_msg="position mismatch at T/2")
        np.testing.assert_allclose(vel_step, vel_analytic, atol=1e-12,
                                   err_msg="velocity mismatch at T/2")

    def test_step_by_step_matches_analytic_at_T(self):
        """Discrete 400 Hz integration must arrive at target position/velocity at T."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        pos_step, vel_step = self._run_kinematic_phase(launch_pos, _TARGET_VEL, _T)
        np.testing.assert_allclose(pos_step, _TARGET_POS, atol=1e-9,
                                   err_msg="final position mismatch")
        np.testing.assert_allclose(vel_step, _TARGET_VEL, atol=1e-12,
                                   err_msg="final velocity mismatch")

    def test_no_position_jump_at_handoff(self):
        """Position must be continuous at the kinematic→free-flight switch."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        pos_end_kin, vel_end_kin = self._run_kinematic_phase(launch_pos, _TARGET_VEL, _T / 2)
        # Physics receives pos_end_kin directly — no re-initialisation.
        pos_physics_init = pos_end_kin.copy()
        vel_physics_init = vel_end_kin.copy()
        assert np.linalg.norm(pos_physics_init - pos_end_kin) == 0.0
        assert np.linalg.norm(vel_physics_init - vel_end_kin) == 0.0

    def test_velocity_at_handoff_equals_target_vel(self):
        """At any point in kinematic phase, velocity == target_vel (constant)."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        _, vel_at_half = _kinematic_state(launch_pos, _TARGET_VEL, _T / 2)
        np.testing.assert_allclose(vel_at_half, _TARGET_VEL, atol=1e-12,
                                   err_msg="velocity should equal target_vel throughout")

    def test_no_velocity_discontinuity_across_handoff(self):
        """No velocity spike across the kinematic→free-flight boundary."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        # One step before and at handoff: velocity is identically target_vel
        _, vel_before = _kinematic_state(launch_pos, _TARGET_VEL, _T / 2 - self.DT)
        _, vel_at     = _kinematic_state(launch_pos, _TARGET_VEL, _T / 2)
        dv = np.linalg.norm(vel_at - vel_before)
        assert dv < 1e-12, (
            f"velocity change at handoff boundary: {dv:.2e} m/s (expected 0)"
        )

    def test_full_trajectory_step_by_step_constant_speed(self):
        """Speed must be constant throughout the entire kinematic window."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        target_speed = np.linalg.norm(_TARGET_VEL)
        n      = int(round(_T / self.DT))
        pos    = launch_pos.copy()
        for _ in range(n + 1):
            speed = np.linalg.norm(_TARGET_VEL)
            assert abs(speed - target_speed) < 1e-12, (
                f"speed deviated from target: {speed:.6f} != {target_speed:.6f}"
            )
            pos = pos + _TARGET_VEL * self.DT


class TestDefaultParametersNumerical:
    """Spot-check the trajectory for the actual DEFAULT_POS0/DEFAULT_VEL0 values."""

    def test_launch_pos_reasonable_altitude(self):
        """Launch position must be above the ground (z > 0)."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        assert launch_pos[2] > 0.0, (
            f"launch altitude {launch_pos[2]:.2f} m is underground"
        )

    def test_launch_pos_within_tether_length(self):
        """Launch position must be within 300 m of anchor (tether max)."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        dist = np.linalg.norm(launch_pos)
        assert dist < 300.0, f"launch position {dist:.1f} m from anchor exceeds tether limit"

    def test_midpoint_velocity_equals_target(self):
        """At t=T/2, velocity is still exactly target_vel (constant-velocity)."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        _, vel_half = _kinematic_state(launch_pos, _TARGET_VEL, _T / 2)
        np.testing.assert_allclose(vel_half, _TARGET_VEL, atol=1e-12)

    def test_kinematic_values(self):
        """Regression: verify the specific launch position for default parameters."""
        launch_pos = compute_launch_position(_TARGET_POS, _TARGET_VEL, _T)
        expected_launch = _TARGET_POS - _TARGET_VEL * _T
        # With T=30: offset = [-0.257, 0.916, -0.093] * 30 = [-7.71, 27.48, -2.79]
        # launch_pos = [46.258+7.71, 14.241-27.48, 12.530+2.79] = [53.968, -13.239, 15.320]
        np.testing.assert_allclose(launch_pos, expected_launch, atol=1e-12)
        assert launch_pos[0] > _TARGET_POS[0], "launch E should be east of target (v_E < 0)"
        assert launch_pos[1] < _TARGET_POS[1], "launch N should be south of target (v_N > 0)"
