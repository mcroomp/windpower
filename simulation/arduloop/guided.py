"""
guided.py -- Faithful Python port of ArduPilot GUIDED-mode attitude control.

The control path mirrors ArduPilot exactly, including the internal slewed
``_attitude_target`` state and ``_ang_vel_target`` feedforward that the
simple "P-directly-on-commanded-quat" version was missing.

Full ArduPilot call chain (per 400 Hz tick)
-------------------------------------------
``set_target_angle_and_climbrate(roll, pitch, yaw, climbrate, ...)``
    -> ``mode_guided.set_angle(q_desired, ang_vel_body={}, ...)``
       stores ``guided_angle_state.attitude_quat = q_desired``

400 Hz ``mode_guided::run_angle_control()``
    -> ``attitude_control->input_quaternion(q_desired, ang_vel_body={})``
         1. ``update_attitude_target()``
               ``_attitude_target *= from_rotvec(_ang_vel_target * dt)``
         2. ``attitude_error_quat = _attitude_target.inv * q_desired``
            ``attitude_error_angle = axis_angle(attitude_error_quat)``
         3. Per-axis ``input_shaping_angle(wrap_pi(error), input_tc, accel_max,
                                           _ang_vel_target[i], ff=0, max_vel, dt)``
               updates ``_ang_vel_target[i]``
         4. ``attitude_controller_run_quat()``
               a. ``thrust_vector_rotation_angles(_attitude_target, q_body)``
                  -> att_error (body frame roll/pitch/yaw)
               b. ``update_ang_vel_target_from_att_error(att_error)``
                  -> P-controlled rate correction  [sqrt controller]
               c. ``rotation_target_to_body = q_body.inv * _attitude_target``
                  ``ang_vel_ff = rotation_target_to_body * _ang_vel_target``
               d. blend ff based on thrust_error_angle threshold:
                  - error < 30 deg:  add full ff
                  - error < 60 deg:  partial ff, blend yaw toward gyro
                  - error >= 60 deg: hold yaw = gyro, no roll/pitch ff
               e. result -> rate targets for HeliRateController
    -> HeliRateController (rate PIDs + swash)

Key parameters
--------------
``ATC_ANG_RLL/PIT/YAW_P``   outer attitude P-gain (default 4.5)
``ATC_ACCEL_R/P/Y_MAX``     sqrt-controller accel limit centi-deg/s^2
``ATC_RATE_R/P/Y_MAX``      max angular velocity (deg/s, 0 = unlimited)
``ATC_INPUT_TC``             input shaping time constant (s, default 0.15)

References
----------
``libraries/AC_AttitudeControl/AC_AttitudeControl.cpp``
  ``input_quaternion``, ``update_attitude_target``,
  ``attitude_controller_run_quat``, ``thrust_vector_rotation_angles``,
  ``update_ang_vel_target_from_att_error``,
  ``input_shaping_angle``, ``input_shaping_ang_vel``
``AP_Math/control.cpp``
  ``sqrt_controller``
``ArduCopter/Copter.cpp``
  ``set_target_angle_and_climbrate``
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .params import HeliParams
from .attitude_heli import HeliRateController, HeliRateOutput


# Thrust angle error above which yaw feedforward is blended/disabled.
# AC_AttitudeControl.h: AC_ATTITUDE_THRUST_ERROR_ANGLE = radians(30)
_THRUST_ERROR_ANGLE = math.radians(30.0)


# ---------------------------------------------------------------------------
# sqrt_controller  (AP_Math/control.cpp)
# ---------------------------------------------------------------------------

def _sqrt_controller(error: float, p: float, accel_max: float, dt: float) -> float:
    """Piecewise proportional/sqrt controller.

    Matches ``sqrt_controller(error, p, second_ord_lim, dt)`` exactly.
    ``accel_max`` is the second-order limit in rad/s^2 (0 = pure linear P).
    """
    if accel_max <= 0.0:
        rate = error * p
    elif p == 0.0:
        rate = math.copysign(math.sqrt(2.0 * accel_max * abs(error)), error) if error != 0.0 else 0.0
    else:
        linear_dist = accel_max / (p * p)
        if error > linear_dist:
            rate = math.sqrt(2.0 * accel_max * (error - linear_dist / 2.0))
        elif error < -linear_dist:
            rate = -math.sqrt(2.0 * accel_max * (-error - linear_dist / 2.0))
        else:
            rate = error * p

    if dt > 0.0:
        rate = max(-abs(error) / dt, min(abs(error) / dt, rate))
    return rate


# ---------------------------------------------------------------------------
# input_shaping_ang_vel  (AC_AttitudeControl.cpp)
# ---------------------------------------------------------------------------

def _input_shaping_ang_vel(
    target_ang_vel: float,
    desired_ang_vel: float,
    accel_max: float,
    dt: float,
) -> float:
    """Acceleration-limited rate shaper.

    Matches ``input_shaping_ang_vel(target, desired, accel_max, dt, input_tc=0)``.
    ``input_tc=0`` skips the jerk-limit block (AP passes 0.0 from
    ``input_shaping_angle``).
    """
    if accel_max > 0.0:
        delta = accel_max * dt
        return max(target_ang_vel - delta, min(target_ang_vel + delta, desired_ang_vel))
    return desired_ang_vel


# ---------------------------------------------------------------------------
# input_shaping_angle  (AC_AttitudeControl.cpp)
# ---------------------------------------------------------------------------

def _input_shaping_angle(
    error_angle: float,
    input_tc: float,
    accel_max: float,
    target_ang_vel: float,
    max_ang_vel: float,
    dt: float,
) -> float:
    """Convert angle error to a shaped rate command.

    Matches ``input_shaping_angle(error, input_tc, accel_max,
                                   target_ang_vel, desired_ang_vel=0,
                                   max_ang_vel, dt)``.
    ``desired_ang_vel`` feedforward is zero because
    ``set_target_angle_and_climbrate`` passes ``ang_vel_body={}``
    (Copter.cpp:392 -- ``mode_guided.set_angle(q, Vector3f{}, ...)``)
    """
    tc = max(input_tc, 0.01)
    desired = _sqrt_controller(error_angle, 1.0 / tc, accel_max, dt)
    if max_ang_vel > 0.0:
        desired = max(-max_ang_vel, min(max_ang_vel, desired))
    return _input_shaping_ang_vel(target_ang_vel, desired, accel_max, dt)


# ---------------------------------------------------------------------------
# thrust_vector_rotation_angles  (AC_AttitudeControl.cpp)
# ---------------------------------------------------------------------------

def _thrust_vector_rotation_angles(
    q_target: np.ndarray,
    q_body: np.ndarray,
) -> tuple[np.ndarray, float, float]:
    """Decompose attitude error into thrust-vector error (roll/pitch) + yaw error.

    Port of ``AC_AttitudeControl::thrust_vector_rotation_angles()``.

    Parameters
    ----------
    q_target, q_body : (4,) [x,y,z,w] passive body-to-NED quaternions.

    Returns
    -------
    attitude_error     : (3,) body-frame axis-angle [roll_err, pitch_err, yaw_err]
    thrust_angle       : current lean angle from vertical (rad)
    thrust_error_angle : angle between body and target thrust vectors (rad)
    """
    rot_body   = Rotation.from_quat(q_body)
    rot_target = Rotation.from_quat(q_target)

    thrust_up = np.array([0.0, 0.0, -1.0])

    body_thrust_vec   = rot_body.apply(thrust_up)
    target_thrust_vec = rot_target.apply(thrust_up)

    # Current lean angle (from vertical [0,0,-1] in NED)
    thrust_angle = math.acos(float(np.clip(np.dot(thrust_up, body_thrust_vec), -1.0, 1.0)))

    # Thrust-vector error angle
    cos_err = float(np.clip(np.dot(body_thrust_vec, target_thrust_vec), -1.0, 1.0))
    thrust_error_angle = math.acos(cos_err)

    # Rotation axis in NED = cross(body_thrust, target_thrust), then into body frame
    cross_ned = np.cross(body_thrust_vec, target_thrust_vec)
    cross_len = float(np.linalg.norm(cross_ned))
    if cross_len < 1e-10 or thrust_error_angle < 1e-10:
        cross_ned = thrust_up  # arbitrary; error angle is zero
    else:
        cross_ned = cross_ned / cross_len

    cross_body = rot_body.inv().apply(cross_ned)
    rot_tvc = Rotation.from_rotvec(cross_body * thrust_error_angle)

    # Roll/pitch error from thrust-vector correction
    tvc_rotvec = rot_tvc.as_rotvec()
    attitude_error = np.zeros(3)
    attitude_error[0] = tvc_rotvec[0]
    attitude_error[1] = tvc_rotvec[1]

    # Yaw error: remaining rotation after thrust-vector correction
    rot_heading = rot_tvc.inv() * rot_body.inv() * rot_target
    attitude_error[2] = rot_heading.as_rotvec()[2]

    return attitude_error, thrust_angle, thrust_error_angle


# ---------------------------------------------------------------------------
# GuidedAttitudeParams
# ---------------------------------------------------------------------------

@dataclass
class GuidedAttitudeParams:
    """Parameters for the GUIDED attitude outer loop.

    All field names match ArduPilot parameter names 1:1 so gains tuned in
    SITL transfer directly.
    """
    # Outer attitude P gains [rad/s per rad].  ATC_ANG_RLL/PIT/YAW_P
    ATC_ANG_RLL_P: float = 4.5
    ATC_ANG_PIT_P: float = 4.5
    ATC_ANG_YAW_P: float = 4.5

    # Accel limits for sqrt/shaping (centi-deg/s^2). 0 = linear P only.
    # ATC_ACCEL_R_MAX, ATC_ACCEL_P_MAX, ATC_ACCEL_Y_MAX
    ATC_ACCEL_R_MAX: float = 110000.0
    ATC_ACCEL_P_MAX: float = 110000.0
    ATC_ACCEL_Y_MAX: float = 27000.0

    # Max body-frame angular velocity (deg/s). 0 = unlimited.
    # ATC_RATE_R_MAX, ATC_RATE_P_MAX, ATC_RATE_Y_MAX
    ATC_RATE_R_MAX: float = 0.0
    ATC_RATE_P_MAX: float = 0.0
    ATC_RATE_Y_MAX: float = 0.0

    # Input shaping time constant (s).  ATC_INPUT_TC
    # AP default: 0.15 (Medium) from AC_AttitudeControl.cpp
    ATC_INPUT_TC: float = 0.15

    # Vertical GUIDED angle-mode parameters (ArduPilot-style climb-rate path).
    # WPNAV climb-rate limits [cm/s] used to constrain set_target_angle_and_climbrate.
    WPNAV_SPEED_UP_CMS: float = 250.0
    WPNAV_SPEED_DN_CMS: float = 150.0
    # Simplified z-loop gains for the Python emulation.
    # z error [m] -> velocity correction [m/s]
    POS_Z_P: float = 1.0
    # velocity error [m/s] -> acceleration command [m/s^2]
    VEL_Z_P: float = 2.0
    # Hover throttle/collective midpoint (0..1) for accel->throttle feedforward.
    THR_HOVER: float = 0.5

    @classmethod
    def from_heli_params(cls, p: HeliParams) -> "GuidedAttitudeParams":
        return cls(
            ATC_ANG_RLL_P=p.ATC_ANG_RLL_P,
            ATC_ANG_PIT_P=p.ATC_ANG_PIT_P,
            ATC_ANG_YAW_P=p.ATC_ANG_YAW_P,
            ATC_ACCEL_R_MAX=p.ATC_ACCEL_R_MAX,
            ATC_ACCEL_P_MAX=p.ATC_ACCEL_P_MAX,
            ATC_ACCEL_Y_MAX=p.ATC_ACCEL_Y_MAX,
            ATC_RATE_R_MAX=p.ATC_RATE_R_MAX,
            ATC_RATE_P_MAX=p.ATC_RATE_P_MAX,
            ATC_RATE_Y_MAX=p.ATC_RATE_Y_MAX,
        )


# ---------------------------------------------------------------------------
# GuidedAttitudeController
# ---------------------------------------------------------------------------

class GuidedAttitudeController:
    """Full GUIDED-mode attitude controller: slewed target + outer P + rate PIDs.

    Faithfully mirrors ``AC_AttitudeControl::input_quaternion`` including:

    * Internal ``_attitude_target`` quaternion that slews toward the commanded
      target via ``input_shaping_angle`` (``ATC_INPUT_TC``, default 0.15 s).
    * ``_ang_vel_target`` feedforward rotated into body frame and added to the
      P-loop output, blended based on thrust-vector error angle (30/60 deg
      thresholds from ``attitude_controller_run_quat``).

    Without these the sim would converge at a different rate and with different
    transient dynamics than SITL, making pre-SITL tuning unreliable.

    Usage
    -----
    ::

        ctrl = GuidedAttitudeController(heli_params)

        # 50 Hz (Lua-equivalent):
        ctrl.set_target_angle_and_climbrate(roll_deg, pitch_deg, yaw_deg,
                                            climbrate_ms=0.0, sim_time=t)
        # or from a rotation matrix (skips Euler round-trip):
        ctrl.set_target_rotation(R_body_ned, sim_time=t)

        # 400 Hz (physics tick):
        out = ctrl.update(q_body_ned, gyro_rads, dt, collective_norm, sim_time=t)
        # out.roll_cyclic, out.pitch_cyclic -> swashplate inputs

    Quaternion convention
    ---------------------
    ``q_body_ned`` is ``[x, y, z, w]`` (scipy convention) -- passive
    body-to-NED rotation, i.e. columns of ``R_body`` expressed in NED.
    Matches ``ahrs:get_quaternion()`` in ArduPilot.
    """

    def __init__(
        self,
        heli_params: HeliParams,
        guided_params: GuidedAttitudeParams | None = None,
    ):
        self._hp = heli_params
        self._gp = guided_params or GuidedAttitudeParams.from_heli_params(heli_params)
        self._rate_ctrl = HeliRateController(heli_params)

        # --- AP internal state ---
        # _attitude_target: slewed internal target (AP: _attitude_target).
        # Initialised to identity; reset to body attitude on first update().
        self._attitude_target: np.ndarray = np.array([0.0, 0.0, 0.0, 1.0])
        # _ang_vel_target: rate of change of _attitude_target, expressed in
        # the _attitude_target body frame (AP: _ang_vel_target).
        self._ang_vel_target: np.ndarray = np.zeros(3)
        self._initialized: bool = False

        # Commanded target (from Lua call)
        self._q_commanded: np.ndarray = np.array([0.0, 0.0, 0.0, 1.0])
        self._target_set: bool = False

        # Timeout
        self._last_target_time: float = -1.0
        self.timeout_s: float = 3.0
        self.climbrate_ms: float = 0.0

        # Vertical state for set_target_angle_and_climbrate (use_thrust=false path).
        self._z_target_up_m: float | None = None

    # ------------------------------------------------------------------
    # Lua-facing API  (50 Hz)
    # ------------------------------------------------------------------

    def set_target_angle_and_climbrate(
        self,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        climbrate_ms: float = 0.0,
        use_yaw_rate: bool = False,
        yaw_rate_degs: float = 0.0,
        *,
        sim_time: float = 0.0,
    ) -> None:
        """Match ``vehicle:set_target_angle_and_climbrate`` exactly.

        ArduPilot converts via ``q.from_euler(radians(roll), radians(pitch),
        radians(yaw))`` which is intrinsic XYZ = extrinsic ZYX.
        """
        r = Rotation.from_euler('ZYX', [yaw_deg, pitch_deg, roll_deg], degrees=True)
        self._q_commanded = r.as_quat()
        self._target_set = True
        self._last_target_time = sim_time
        self.climbrate_ms = climbrate_ms

    def set_target_rotation(
        self,
        R_body_ned: np.ndarray,
        *,
        sim_time: float = 0.0,
        climbrate_ms: float = 0.0,
    ) -> None:
        """Set target from a 3x3 rotation matrix (columns = body axes in NED).

        ``R_body_ned[:,2]`` = body_z pointing toward anchor for tethered hover.
        """
        r = Rotation.from_matrix(R_body_ned)  # body-to-NED, same convention as q_body in update()
        self._q_commanded = r.as_quat()
        self._target_set = True
        self._last_target_time = sim_time
        self.climbrate_ms = climbrate_ms

    # ------------------------------------------------------------------
    # 400 Hz physics tick
    # ------------------------------------------------------------------

    def update(
        self,
        q_body_ned: np.ndarray,
        gyro_body_rads: tuple[float, float, float] | np.ndarray,
        dt: float,
        collective_norm: float = 0.0,
        sim_time: float = 0.0,
        saturated: tuple[bool, bool, bool] = (False, False, False),
        pos_z_up_m: float | None = None,
        vel_z_up_mps: float | None = None,
    ) -> HeliRateOutput:
        """Run one 400 Hz attitude+rate tick.

        Parameters
        ----------
        q_body_ned     : (4,) [x,y,z,w] passive body-to-NED quaternion.
        gyro_body_rads : body-frame angular velocity (rad/s) [roll, pitch, yaw].
        dt             : tick duration (s).
        collective_norm: current collective in [-1,1] for hover-roll-trim.
        sim_time       : current sim time (s) for timeout detection.
        saturated      : per-axis PID saturation flags for anti-windup.
        """
        q_body = np.asarray(q_body_ned, dtype=float)
        gyro   = np.asarray(gyro_body_rads, dtype=float)
        gp     = self._gp

        # Initialise _attitude_target to body attitude on first tick (avoids
        # a step transient from identity to actual attitude at t=0).
        # Also reset _ang_vel_target so Step 2 (error computation) is done with
        # the correct rot_att frame; without this, rot_att would still hold the
        # pre-init identity and produce a huge spurious feedforward.
        if not self._initialized:
            self._attitude_target = q_body.copy()
            self._ang_vel_target[:] = 0.0
            self._initialized = True

        # Timeout: snap everything to current body and wipe slew state.
        if self._last_target_time >= 0.0 and sim_time - self._last_target_time > self.timeout_s:
            self._q_commanded = q_body.copy()
            self._attitude_target = q_body.copy()
            self._ang_vel_target[:] = 0.0
            self._rate_ctrl.reset()
            self._z_target_up_m = pos_z_up_m

        # === Step 1: update_attitude_target() ===
        # Integrate _attitude_target by _ang_vel_target * dt.
        # Right-multiply: _ang_vel_target is in the _attitude_target body frame.
        # AP: attitude_target_update.from_axis_angle(_ang_vel_target * dt)
        #     _attitude_target *= attitude_target_update
        rot_att = Rotation.from_quat(self._attitude_target)
        rot_att = rot_att * Rotation.from_rotvec(self._ang_vel_target * dt)
        self._attitude_target = rot_att.as_quat()

        # === Step 2: attitude error _attitude_target.inv * q_commanded ===
        # AP: attitude_error_quat = _attitude_target.inverse() * attitude_desired_quat
        rot_cmd = Rotation.from_quat(self._q_commanded)
        err_rot = rot_att.inv() * rot_cmd
        err_rotvec = err_rot.as_rotvec()

        # === Step 3: input_shaping_angle per axis => update _ang_vel_target ===
        accel_r = math.radians(gp.ATC_ACCEL_R_MAX * 0.01)
        accel_p = math.radians(gp.ATC_ACCEL_P_MAX * 0.01)
        accel_y = math.radians(gp.ATC_ACCEL_Y_MAX * 0.01)
        max_r = math.radians(gp.ATC_RATE_R_MAX) if gp.ATC_RATE_R_MAX > 0.0 else 0.0
        max_p = math.radians(gp.ATC_RATE_P_MAX) if gp.ATC_RATE_P_MAX > 0.0 else 0.0
        max_y = math.radians(gp.ATC_RATE_Y_MAX) if gp.ATC_RATE_Y_MAX > 0.0 else 0.0

        def _wrap_pi(x: float) -> float:
            while x > math.pi:  x -= 2.0 * math.pi
            while x < -math.pi: x += 2.0 * math.pi
            return x

        self._ang_vel_target[0] = _input_shaping_angle(
            _wrap_pi(err_rotvec[0]), gp.ATC_INPUT_TC, accel_r,
            self._ang_vel_target[0], max_r, dt)
        self._ang_vel_target[1] = _input_shaping_angle(
            _wrap_pi(err_rotvec[1]), gp.ATC_INPUT_TC, accel_p,
            self._ang_vel_target[1], max_p, dt)
        self._ang_vel_target[2] = _input_shaping_angle(
            _wrap_pi(err_rotvec[2]), gp.ATC_INPUT_TC, accel_y,
            self._ang_vel_target[2], max_y, dt)

        # === Step 4: attitude_controller_run_quat() ===
        # Error between slewed _attitude_target and actual body attitude.
        att_error, _thrust_angle, thrust_error_angle = _thrust_vector_rotation_angles(
            self._attitude_target, q_body)

        # Outer P-loop: attitude error -> body-frame rate correction.
        # AP: update_ang_vel_target_from_att_error(attitude_error)
        rate_roll  = _sqrt_controller(att_error[0], gp.ATC_ANG_RLL_P, accel_r, dt)
        rate_pitch = _sqrt_controller(att_error[1], gp.ATC_ANG_PIT_P, accel_p, dt)
        rate_yaw   = _sqrt_controller(att_error[2], gp.ATC_ANG_YAW_P, accel_y, dt)
        ang_vel_body = np.array([rate_roll, rate_pitch, rate_yaw])

        # Feedforward: rotate _ang_vel_target from target frame into body frame.
        # AP: rotation_target_to_body = attitude_body.inverse() * _attitude_target
        #     ang_vel_body_feedforward = rotation_target_to_body * _ang_vel_target
        rot_body = Rotation.from_quat(q_body)
        rot_t2b = rot_body.inv() * rot_att
        ang_vel_ff = rot_t2b.apply(self._ang_vel_target)

        # Blend feedforward based on thrust error angle.
        # AC_ATTITUDE_THRUST_ERROR_ANGLE = 30 deg; thresholds at 30 and 60 deg.
        if thrust_error_angle > _THRUST_ERROR_ANGLE * 2.0:
            # Too much tilt error: disable roll/pitch ff entirely; lock yaw to gyro.
            ang_vel_body[2] = gyro[2]
        elif thrust_error_angle > _THRUST_ERROR_ANGLE:
            ff_scalar = 1.0 - (thrust_error_angle - _THRUST_ERROR_ANGLE) / _THRUST_ERROR_ANGLE
            ang_vel_body[0] += ang_vel_ff[0] * ff_scalar
            ang_vel_body[1] += ang_vel_ff[1] * ff_scalar
            ang_vel_body[2] += ang_vel_ff[2]
            ang_vel_body[2] = gyro[2] * (1.0 - ff_scalar) + ang_vel_body[2] * ff_scalar
        else:
            ang_vel_body += ang_vel_ff

        # Final rate limits (ang_vel_limit)
        if max_r > 0.0: ang_vel_body[0] = max(-max_r, min(max_r, ang_vel_body[0]))
        if max_p > 0.0: ang_vel_body[1] = max(-max_p, min(max_p, ang_vel_body[1]))
        if max_y > 0.0: ang_vel_body[2] = max(-max_y, min(max_y, ang_vel_body[2]))

        # === Step 5: HeliRateController (rate PIDs + swash) ===
        out = self._rate_ctrl.update(
            rate_target_rads=tuple(ang_vel_body),
            gyro_rate_rads=tuple(gyro),
            dt=dt,
            collective_norm=collective_norm,
            saturated=saturated,
        )

        # Vertical collective command for set_target_angle_and_climbrate.
        # Mirrors ArduPilot semantics at a high level: climbrate target is
        # constrained, integrated into a moving altitude target, then closed
        # through vertical velocity/acceleration to a collective around hover.
        out.collective_norm_cmd = self._compute_collective_norm(
            q_body=q_body,
            dt=dt,
            pos_z_up_m=pos_z_up_m,
            vel_z_up_mps=vel_z_up_mps,
        )
        return out

    def _compute_collective_norm(
        self,
        *,
        q_body: np.ndarray,
        dt: float,
        pos_z_up_m: float | None,
        vel_z_up_mps: float | None,
    ) -> float | None:
        if pos_z_up_m is None or vel_z_up_mps is None:
            return None

        gp = self._gp
        if self._z_target_up_m is None:
            self._z_target_up_m = float(pos_z_up_m)

        climb_rate_cms = float(self.climbrate_ms * 100.0)
        climb_rate_cms = max(-gp.WPNAV_SPEED_DN_CMS, min(gp.WPNAV_SPEED_UP_CMS, climb_rate_cms))
        climb_rate_mps = climb_rate_cms * 0.01

        # Integrate commanded climb-rate into a moving altitude target.
        self._z_target_up_m += climb_rate_mps * dt

        z_err = self._z_target_up_m - float(pos_z_up_m)
        vel_target_mps = climb_rate_mps + gp.POS_Z_P * z_err
        accel_target_mps2 = gp.VEL_Z_P * (vel_target_mps - float(vel_z_up_mps))

        # Map target vertical acceleration to collective around hover.
        thr_hover = max(0.05, min(0.95, gp.THR_HOVER))
        thr = thr_hover + (thr_hover / 9.81) * accel_target_mps2

        # Approximate angle-boost so high tilt preserves vertical authority.
        rot_body = Rotation.from_quat(q_body)
        thrust_up_ned = rot_body.apply(np.array([0.0, 0.0, -1.0]))
        cos_tilt = max(0.1, float(np.dot(thrust_up_ned, np.array([0.0, 0.0, -1.0]))))
        thr = thr / cos_tilt

        thr = max(0.0, min(1.0, thr))
        return thr * 2.0 - 1.0

    def reset(self) -> None:
        """Reset all internal state (rate PIDs + slew state).

        Call when resuming after a pause or mode switch.
        """
        self._rate_ctrl.reset()
        self._ang_vel_target[:] = 0.0
        self._initialized = False
        self._z_target_up_m = None

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------

    @property
    def target_rotation(self) -> Rotation:
        """Commanded target attitude (not the slewed _attitude_target)."""
        return Rotation.from_quat(self._q_commanded)

    @property
    def attitude_target_rotation(self) -> Rotation:
        """Internal slewed _attitude_target (what the P-loop tracks against)."""
        return Rotation.from_quat(self._attitude_target)

    @property
    def target_euler_deg(self) -> np.ndarray:
        """Commanded target as ZYX Euler [roll, pitch, yaw] in degrees."""
        zyx = self.target_rotation.as_euler('ZYX', degrees=True)
        return np.array([zyx[2], zyx[1], zyx[0]])
