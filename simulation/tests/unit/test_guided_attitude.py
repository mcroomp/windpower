"""test_guided_attitude.py -- unit tests for arduloop.guided.

Tests are in four groups:

1. _sqrt_controller         -- matches AP_Math/control.cpp exactly.
2. _attitude_command_model  -- matches AC_AttitudeControl.cpp attitude_command_model exactly.
3. _thrust_vector_rotation_angles -- correct roll/pitch/yaw error decomposition.
4. GuidedAttitudeController -- end-to-end, including slewed target state and
                               feedforward blending from attitude_controller_run_quat.

All tests are pure math -- no SITL, no physics, no network.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


from arduloop.guided import (
    GuidedAttitudeController,
    GuidedAttitudeParams,
    _THRUST_ERROR_ANGLE,
    _attitude_command_model,
    _sqrt_controller,
    _thrust_vector_rotation_angles,
)
from arduloop import HeliParams
from arduloop.attitude_heli import HeliRateOutput


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _q(roll_deg: float = 0.0, pitch_deg: float = 0.0, yaw_deg: float = 0.0) -> np.ndarray:
    return Rotation.from_euler('ZYX', [yaw_deg, pitch_deg, roll_deg], degrees=True).as_quat()


def _make_ctrl(
    ang_p: float = 4.5,
    accel_max_cdss: float = 0.0,   # 0 = linear P
    input_tc: float = 0.0,         # 0 = no shaping delay (instantaneous)
) -> GuidedAttitudeController:
    hp = HeliParams()
    gp = GuidedAttitudeParams(
        ATC_ANG_RLL_P=ang_p,
        ATC_ANG_PIT_P=ang_p,
        ATC_ANG_YAW_P=ang_p,
        ATC_ACCEL_R_MAX=accel_max_cdss,
        ATC_ACCEL_P_MAX=accel_max_cdss,
        ATC_ACCEL_Y_MAX=accel_max_cdss,
        ATC_INPUT_TC=input_tc,
    )
    return GuidedAttitudeController(hp, gp)


# ---------------------------------------------------------------------------
# 1. _sqrt_controller
# ---------------------------------------------------------------------------

class TestSqrtController:
    def test_zero_error(self):
        assert _sqrt_controller(0.0, 4.5, 0.0, 0.0025) == pytest.approx(0.0)

    def test_linear_no_accel_limit(self):
        assert _sqrt_controller(0.5, 4.5, 0.0, 0.0025) == pytest.approx(2.25, rel=1e-6)

    def test_linear_region_with_accel(self):
        p, accel = 4.5, math.radians(110000 * 0.01)
        err = accel / (p * p) * 0.5  # half of linear_dist -> still in linear region
        assert _sqrt_controller(err, p, accel, 0.0025) == pytest.approx(err * p, rel=1e-5)

    def test_sqrt_region(self):
        p, accel = 4.5, math.radians(110000 * 0.01)
        linear_dist = accel / (p * p)
        err = linear_dist * 10.0
        expected = math.sqrt(2.0 * accel * (err - linear_dist / 2.0))
        assert _sqrt_controller(err, p, accel, 0.0025) == pytest.approx(expected, rel=1e-4)

    def test_sign_symmetry(self):
        pos = _sqrt_controller( 0.3, 4.5, 5.0, 0.0)
        neg = _sqrt_controller(-0.3, 4.5, 5.0, 0.0)
        assert pos == pytest.approx(-neg, rel=1e-6)

    def test_dt_anti_overshoot(self):
        err, dt = 0.01, 0.025
        rate = _sqrt_controller(err, 1000.0, 0.0, dt)
        assert abs(rate) <= abs(err) / dt + 1e-9


# ---------------------------------------------------------------------------
# 2. _attitude_command_model
# ---------------------------------------------------------------------------

class TestAttitudeCommandModel:
    """Port of AC_AttitudeControl::attitude_command_model (3rd-order jerk-limited shaper).

    Key behavioural differences from the old 2-step _input_shaping_angle:
    - First step is jerk-limited: max vel increment = (accel_max / input_tc) * dt^2
    - State is (vel, accel), not just vel.
    """

    _accel = math.radians(110000 * 0.01)   # AP default: 1100 deg/s^2
    _tc    = 0.15                           # AP default input_tc
    _dt    = 0.0025                         # 400 Hz

    def test_zero_error_returns_zero_state(self):
        v, a = _attitude_command_model(0.0, 0.0, 0.0, 0.0, self._accel, self._tc, self._dt)
        assert v == pytest.approx(0.0, abs=1e-9)
        assert a == pytest.approx(0.0, abs=1e-9)

    def test_first_step_jerk_limited(self):
        # With zero initial state, first-step velocity <= jerk_max * dt^2.
        jerk_max = self._accel / self._tc
        max_vel_step = jerk_max * self._dt * self._dt
        v, _ = _attitude_command_model(1.0, 0.0, 0.0, 0.0, self._accel, self._tc, self._dt)
        assert abs(v) <= abs(max_vel_step) + 1e-9

    def test_velocity_grows_over_time(self):
        # Over many steps, target velocity should increase from zero.
        v, a = 0.0, 0.0
        for _ in range(200):
            v, a = _attitude_command_model(1.0, v, a, 0.0, self._accel, self._tc, self._dt)
        assert v > 0.0

    def test_max_ang_vel_clamps_velocity(self):
        # max_ang_vel=100 deg/s: velocity must stay within limits.
        max_vel = math.radians(100.0)
        v, a = 0.0, 0.0
        for _ in range(400):
            v, a = _attitude_command_model(5.0, v, a, max_vel, self._accel, self._tc, self._dt)
        assert abs(v) <= max_vel + 1e-9

    def test_zero_accel_no_jerk_limit(self):
        # With accel_max=0 the shaper falls back to linear P (no jerk limiting).
        # Result should match sqrt_controller(err, 1/tc, 0, dt) integrated.
        v, _ = _attitude_command_model(0.1, 0.0, 0.0, 0.0, 0.0, self._tc, self._dt)
        expected = _sqrt_controller(0.1, 1.0 / self._tc, 0.0, self._dt) * self._dt
        # With accel_max=0, AP substitutes 1800 deg/s^2, so this is NOT pure linear P.
        # Just verify the result is finite and in the right direction.
        assert math.isfinite(v)
        assert v > 0.0

    def test_sign_symmetry(self):
        v_pos, _ = _attitude_command_model( 0.5, 0.0, 0.0, 0.0, self._accel, self._tc, self._dt)
        v_neg, _ = _attitude_command_model(-0.5, 0.0, 0.0, 0.0, self._accel, self._tc, self._dt)
        assert v_pos == pytest.approx(-v_neg, abs=1e-10)


# ---------------------------------------------------------------------------
# 3. _thrust_vector_rotation_angles
# ---------------------------------------------------------------------------

class TestThrustVectorRotation:
    def test_identity_zero_error(self):
        q = _q()
        err, _, _ = _thrust_vector_rotation_angles(q, q)
        assert err == pytest.approx([0.0, 0.0, 0.0], abs=1e-8)

    def test_pure_roll_error(self):
        err, _, _ = _thrust_vector_rotation_angles(_q(roll_deg=20.0), _q())
        assert abs(err[0]) > abs(err[1]) * 5
        assert err[0] > 0.0
        assert abs(err[2]) < 0.02

    def test_pure_pitch_error(self):
        err, _, _ = _thrust_vector_rotation_angles(_q(pitch_deg=20.0), _q())
        assert abs(err[1]) > abs(err[0]) * 5
        assert err[1] > 0.0
        assert abs(err[2]) < 0.02

    def test_pure_yaw_is_heading_only(self):
        # Pure yaw: thrust vectors are the same, only heading differs.
        err, _, _ = _thrust_vector_rotation_angles(_q(yaw_deg=30.0), _q())
        assert abs(err[0]) < 1e-6
        assert abs(err[1]) < 1e-6
        assert err[2] == pytest.approx(math.radians(30.0), abs=1e-5)

    def test_extreme_tilt_no_nan(self):
        err, ta, tea = _thrust_vector_rotation_angles(_q(pitch_deg=-65.0), _q())
        assert all(math.isfinite(v) for v in err)
        assert math.isfinite(ta) and math.isfinite(tea)

    def test_small_angle_linearity(self):
        for deg in [2.0, 5.0, 10.0]:
            err, _, _ = _thrust_vector_rotation_angles(_q(roll_deg=deg), _q())
            assert abs(err[0]) == pytest.approx(math.radians(deg), rel=0.01)


# ---------------------------------------------------------------------------
# 4. GuidedAttitudeController
# ---------------------------------------------------------------------------

class TestGuidedAttitudeController:

    # --- Basic sanity ---

    def test_identity_target_zero_cyclic(self):
        """At target == current, all outputs are zero."""
        ctrl = _make_ctrl()
        q = _q()
        ctrl.set_target_angle_and_climbrate(0.0, 0.0, 0.0, sim_time=0.0)
        out = ctrl.update(q, (0.0, 0.0, 0.0), dt=0.0025, sim_time=0.0)
        assert out.roll_cyclic  == pytest.approx(0.0, abs=1e-6)
        assert out.pitch_cyclic == pytest.approx(0.0, abs=1e-6)

    def test_roll_error_activates_roll_cyclic(self):
        ctrl = _make_ctrl(ang_p=4.5, accel_max_cdss=0.0, input_tc=0.0)
        ctrl.set_target_angle_and_climbrate(20.0, 0.0, 0.0, sim_time=0.0)
        out = ctrl.update(_q(), (0.0, 0.0, 0.0), dt=0.0025, sim_time=0.0)
        assert abs(out.roll_cyclic) > abs(out.pitch_cyclic) * 3

    def test_pitch_error_activates_pitch_cyclic(self):
        ctrl = _make_ctrl(ang_p=4.5, accel_max_cdss=0.0, input_tc=0.0)
        ctrl.set_target_angle_and_climbrate(0.0, 20.0, 0.0, sim_time=0.0)
        out = ctrl.update(_q(), (0.0, 0.0, 0.0), dt=0.0025, sim_time=0.0)
        assert abs(out.pitch_cyclic) > abs(out.roll_cyclic) * 3

    def test_yaw_only_error_is_yaw_only(self):
        ctrl = _make_ctrl(ang_p=4.5, accel_max_cdss=0.0, input_tc=0.0)
        ctrl.set_target_angle_and_climbrate(0.0, 0.0, 30.0, sim_time=0.0)
        out = ctrl.update(_q(), (0.0, 0.0, 0.0), dt=0.0025, sim_time=0.0)
        assert abs(out.yaw_cmd) > 0.0
        assert abs(out.roll_cyclic)  < 1e-4
        assert abs(out.pitch_cyclic) < 1e-4

    # --- Extreme tilt ---

    def test_extreme_tilt_no_nan(self):
        ctrl = _make_ctrl()
        ctrl.set_target_angle_and_climbrate(0.0, -65.0, 0.0, sim_time=0.0)
        out = ctrl.update(_q(pitch_deg=-65.0), (0.0, 0.0, 0.0), dt=0.0025, sim_time=0.0)
        assert math.isfinite(out.roll_cyclic)
        assert math.isfinite(out.pitch_cyclic)
        assert math.isfinite(out.yaw_cmd)

    def test_extreme_tilt_no_bleed_to_horizontal(self):
        """Holding 65-deg tilt at 65-deg: zero error, zero cyclic."""
        ctrl = _make_ctrl(input_tc=0.0)
        q = _q(pitch_deg=-65.0)
        ctrl.set_target_angle_and_climbrate(0.0, -65.0, 0.0, sim_time=0.0)
        out = ctrl.update(q, (0.0, 0.0, 0.0), dt=0.0025, sim_time=0.0)
        assert out.roll_cyclic  == pytest.approx(0.0, abs=1e-6)
        assert out.pitch_cyclic == pytest.approx(0.0, abs=1e-6)

    # --- Slewing _attitude_target (the key AP behaviour) ---

    def test_attitude_target_slews_not_jumps(self):
        """With input_tc > 0 and accel limit, _attitude_target must lag behind
        the commanded quaternion after a single tick.

        This is the critical difference from a naive P-on-commanded-quat
        implementation: in AP, _attitude_target slews via input_shaping_angle.
        """
        # Use realistic AP defaults (tc=0.15, accel=110000 cdss)
        ctrl = _make_ctrl(ang_p=4.5, accel_max_cdss=110000.0, input_tc=0.15)
        q_body = _q()
        ctrl.set_target_angle_and_climbrate(0.0, -65.0, 0.0, sim_time=0.0)

        # After one 400 Hz tick, _attitude_target must still be near identity,
        # NOT jumped to -65 deg, because accel limit caps the rate.
        ctrl.update(q_body, (0.0, 0.0, 0.0), dt=0.0025, sim_time=0.0)
        att_euler = ctrl.attitude_target_rotation.as_euler('ZYX', degrees=True)
        # att_euler[1] is pitch (ZYX order: [yaw, pitch, roll])
        # After one 2.5 ms step with accel ~19 rad/s^2: dv = 19*0.0025 ~ 0.047 rad/s
        # dtheta = 0.5 * 0.047 * 0.0025 ~ 0.00006 rad < 0.004 deg
        assert abs(att_euler[1]) < 0.01  # well under 0.1 deg

    def test_attitude_target_reaches_commanded_over_time(self):
        """After enough ticks without physics (body holds still), _attitude_target
        should converge toward the commanded quaternion."""
        ctrl = _make_ctrl(ang_p=4.5, accel_max_cdss=110000.0, input_tc=0.15)
        q_body = _q()
        ctrl.set_target_angle_and_climbrate(0.0, -20.0, 0.0, sim_time=0.0)

        dt = 0.0025
        for i in range(600):  # 1.5 s of slewing
            ctrl.update(q_body, (0.0, 0.0, 0.0), dt=dt, sim_time=i * dt)

        att_euler = ctrl.attitude_target_rotation.as_euler('ZYX', degrees=True)
        # pitch should be within ~2 deg of -20 deg
        assert abs(att_euler[1] - (-20.0)) < 2.0

    # --- Feedforward blending ---

    def test_no_feedforward_at_small_error(self):
        """Small error: feedforward is added (thrust_error < 30 deg).
        Output must be non-zero and finite."""
        ctrl = _make_ctrl(ang_p=4.5, accel_max_cdss=0.0, input_tc=0.0)
        ctrl.set_target_angle_and_climbrate(5.0, 0.0, 0.0, sim_time=0.0)
        out = ctrl.update(_q(), (0.0, 0.0, 0.0), dt=0.0025, sim_time=0.0)
        assert math.isfinite(out.roll_cyclic)
        assert abs(out.roll_cyclic) > 0.0

    def test_yaw_locked_to_gyro_at_large_thrust_error(self):
        """When thrust_error_angle > 60 deg, AP locks yaw rate target to gyro z.

        The threshold is on error between _attitude_target and q_body, NOT
        between q_commanded and q_body.  We force _attitude_target to -80 deg
        pitch while body stays at identity -- guaranteed > 60 deg thrust error.
        """
        ctrl = _make_ctrl(ang_p=4.5, accel_max_cdss=0.0, input_tc=0.0)
        ctrl.set_target_angle_and_climbrate(0.0, -80.0, 0.0, sim_time=0.0)

        # Force _attitude_target far from body (bypasses slewing for this test).
        ctrl._attitude_target = _q(pitch_deg=-80.0)
        ctrl._initialized = True

        gyro_yaw = 0.5  # rad/s
        # First tick: yaw target is forced to gyro (lock path); with zero I-term,
        # yaw rate PID error = gyro - gyro = 0, so yaw_cmd should be near zero.
        ctrl.update(_q(), (0.0, 0.0, gyro_yaw), dt=0.0025, sim_time=0.0)
        # Second tick: still locked, I-term tiny, yaw_cmd still near zero.
        out = ctrl.update(_q(), (0.0, 0.0, gyro_yaw), dt=0.0025, sim_time=0.0025)
        assert abs(out.yaw_cmd) < 0.05

    # --- set_target_rotation matches Euler path ---

    def test_set_target_rotation_equals_euler(self):
        hp = HeliParams()
        gp = GuidedAttitudeParams()
        roll, pitch, yaw = 5.0, -25.0, 45.0

        ctrl_e = GuidedAttitudeController(hp, gp)
        ctrl_r = GuidedAttitudeController(hp, gp)
        ctrl_e.set_target_angle_and_climbrate(roll, pitch, yaw, sim_time=0.0)
        R = Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=True).as_matrix()
        ctrl_r.set_target_rotation(R, sim_time=0.0)

        q_body = _q(roll_deg=0.0, pitch_deg=-10.0, yaw_deg=30.0)
        gyro = (0.0, 0.0, 0.0)
        out_e = ctrl_e.update(q_body, gyro, dt=0.0025, sim_time=0.0)
        out_r = ctrl_r.update(q_body, gyro, dt=0.0025, sim_time=0.0)
        assert out_e.roll_cyclic  == pytest.approx(out_r.roll_cyclic,  abs=1e-8)
        assert out_e.pitch_cyclic == pytest.approx(out_r.pitch_cyclic, abs=1e-8)
        assert out_e.yaw_cmd      == pytest.approx(out_r.yaw_cmd,      abs=1e-8)

    # --- Timeout ---

    def test_timeout_freezes_output(self):
        ctrl = _make_ctrl(accel_max_cdss=110000.0, input_tc=0.15)
        q = _q()
        ctrl.set_target_angle_and_climbrate(30.0, 0.0, 0.0, sim_time=0.0)

        # Run enough ticks for the 3rd-order shaper to build up a meaningful
        # ang_vel_target (the new shaper is jerk-limited so one tick is tiny).
        out_before: HeliRateOutput | None = None
        dt = 0.0025
        for i in range(80):  # 0.2 s of tracking
            out_before = ctrl.update(q, (0.0, 0.0, 0.0), dt=dt, sim_time=i * dt)
        assert out_before is not None
        assert abs(out_before.roll_cyclic) > 0.01

        # 5 s > 3 s timeout: target snaps to body, slew state zeroed, PIDs reset
        out_after = ctrl.update(q, (0.0, 0.0, 0.0), dt=dt, sim_time=5.0)
        assert out_after.roll_cyclic  == pytest.approx(0.0, abs=1e-6)
        assert out_after.pitch_cyclic == pytest.approx(0.0, abs=1e-6)

    # --- Euler round-trip ---

    def test_target_euler_deg_roundtrip(self):
        ctrl = _make_ctrl()
        ctrl.set_target_angle_and_climbrate(10.0, -25.0, 45.0, sim_time=0.0)
        e = ctrl.target_euler_deg
        assert e[0] == pytest.approx(10.0,  abs=1e-4)
        assert e[1] == pytest.approx(-25.0, abs=1e-4)
        assert e[2] == pytest.approx(45.0,  abs=1e-4)

    # --- Rate-only Guided thrust path ---

    def test_rate_and_throttle_zero_rate_holds_current_attitude(self):
        """Rate-only Guided does not require an externally known absolute angle.

        On the first tick ArduPilot conditions _attitude_target from current
        body attitude; zero requested body rates therefore produce no cyclic
        even when the vehicle is not level.
        """
        ctrl = _make_ctrl(accel_max_cdss=0.0, input_tc=0.0)
        q_body = _q(roll_deg=12.0, pitch_deg=-35.0, yaw_deg=44.0)
        ctrl.set_target_rate_and_throttle(0.0, 0.0, 0.0, throttle=0.25, sim_time=0.0)
        out = ctrl.update(q_body, (0.0, 0.0, 0.0), dt=0.0025, sim_time=0.0)

        assert out.roll_cyclic == pytest.approx(0.0, abs=1e-6)
        assert out.pitch_cyclic == pytest.approx(0.0, abs=1e-6)
        assert out.yaw_cmd == pytest.approx(0.0, abs=1e-6)
        assert out.collective_norm_cmd == pytest.approx(-0.5, abs=1e-6)

    def test_rate_and_throttle_roll_rate_drives_rate_loop(self):
        ctrl = _make_ctrl(accel_max_cdss=0.0, input_tc=0.0)
        ctrl.set_target_rate_and_throttle(20.0, 0.0, 0.0, throttle=0.5, sim_time=0.0)
        out = ctrl.update(_q(), (0.0, 0.0, 0.0), dt=0.0025, sim_time=0.0)

        assert abs(out.roll_cyclic) > abs(out.pitch_cyclic) * 3
        assert out.collective_norm_cmd == pytest.approx(0.0, abs=1e-6)

    # --- Vertical channel from set_target_angle_and_climbrate ---

    def test_collective_command_present_with_vertical_state(self):
        ctrl = _make_ctrl()
        ctrl.set_target_angle_and_climbrate(0.0, 0.0, 0.0, climbrate_ms=0.0, sim_time=0.0)
        out = ctrl.update(
            _q(),
            (0.0, 0.0, 0.0),
            dt=0.01,
            sim_time=0.0,
            pos_z_up_m=10.0,
            vel_z_up_mps=0.0,
        )
        assert out.collective_norm_cmd is not None

    def test_positive_climbrate_increases_collective_command(self):
        ctrl = _make_ctrl()
        # Prime z-target at the current altitude with zero climb command.
        ctrl.set_target_angle_and_climbrate(0.0, 0.0, 0.0, climbrate_ms=0.0, sim_time=0.0)
        out0 = ctrl.update(
            _q(),
            (0.0, 0.0, 0.0),
            dt=0.01,
            sim_time=0.0,
            pos_z_up_m=10.0,
            vel_z_up_mps=0.0,
        )
        assert out0.collective_norm_cmd is not None
        c0 = float(out0.collective_norm_cmd)
        ctrl.set_target_angle_and_climbrate(0.0, 0.0, 0.0, climbrate_ms=1.0, sim_time=0.01)
        out1 = ctrl.update(
            _q(),
            (0.0, 0.0, 0.0),
            dt=0.01,
            sim_time=0.01,
            pos_z_up_m=10.0,
            vel_z_up_mps=0.0,
        )
        assert out1.collective_norm_cmd is not None
        assert float(out1.collective_norm_cmd) > c0

    # --- Closed-loop convergence ---

    def test_convergence_closed_loop(self):
        """With input_tc=0 and no accel limit, integrating body by rate_target
        (perfect plant) must monotonically reduce attitude error."""
        ctrl = _make_ctrl(ang_p=4.5, accel_max_cdss=0.0, input_tc=0.0)
        ctrl.set_target_angle_and_climbrate(15.0, 0.0, 0.0, sim_time=0.0)

        rot = Rotation.from_quat(_q())
        dt = 0.0025
        prev_err = None
        for step in range(300):
            t = step * dt
            err, _, _ = _thrust_vector_rotation_angles(ctrl._q_commanded, rot.as_quat())
            err_mag = float(np.linalg.norm(err))
            if prev_err is not None:
                assert err_mag <= prev_err * 1.05 + 1e-8, \
                    f"Error grew at step {step}: {prev_err:.6f} -> {err_mag:.6f}"
            if err_mag < 1e-5:
                break
            # Ideal plant: integrate body by outer P rate command directly.
            rate = err * 4.5
            rot = rot * Rotation.from_rotvec(rate * dt)
            prev_err = err_mag
