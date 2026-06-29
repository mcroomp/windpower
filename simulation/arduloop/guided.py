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
``ATC_ANG_RLL/PIT/YAW_P``   outer attitude P-gain (loaded from .parm)
``ATC_ACC_R/P/Y_MAX``       AP 4.7 accel limit in deg/s^2
``ATC_ACCEL_R/P/Y_MAX``     legacy fallback accel naming (centi-deg/s^2)
``ATC_RATE_R/P/Y_MAX``      max angular velocity (deg/s, 0 = unlimited)
``ATC_INPUT_TC``             input shaping time constant (s, loaded from .parm)

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


def _guided_ap_defaults() -> dict[str, float]:
    from param_defaults import load_attitude_params

    return load_attitude_params()


def _accel_param_to_radss(value: float) -> float:
    """Convert AP accel parameter to rad/s^2.

    ArduPilot 4.7 uses deg/s^2 (`ATC_ACC_*_MAX`), while older paths used
    centi-deg/s^2 naming (`ATC_ACCEL_*_MAX`). Support both by magnitude.
    """
    if abs(value) > 5000.0:
        return math.radians(value * 0.01)
    return math.radians(value)


# Thrust angle error above which yaw feedforward is blended/disabled.
# AC_AttitudeControl.h: AC_ATTITUDE_THRUST_ERROR_ANGLE = radians(30)
_THRUST_ERROR_ANGLE = math.radians(30.0)

# Accel limits for sqrt controller (AC_AttitudeControl.h)
_ACCEL_RP_MIN = math.radians(40.0)    # AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS
_ACCEL_RP_MAX = math.radians(720.0)   # AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS
_ACCEL_Y_MIN  = math.radians(10.0)    # AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS
_ACCEL_Y_MAX  = math.radians(120.0)   # AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS
_YAW_MAX_ERROR_ANGLE_RAD = math.radians(45.0)  # AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD


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
# shape_accel  (AP_Math/control.cpp)
# ---------------------------------------------------------------------------

def _shape_accel(accel_desired: float, accel: float, jerk_max: float, dt: float) -> float:
    """Jerk-limited acceleration update.  Port of shape_accel()."""
    if jerk_max <= 0.0 or dt <= 0.0:
        return accel
    delta = max(-jerk_max * dt, min(jerk_max * dt, accel_desired - accel))
    return accel + delta


# ---------------------------------------------------------------------------
# sqrt_controller_accel  (AP_Math/control.cpp)
# ---------------------------------------------------------------------------

def _sqrt_controller_accel(
    error: float,
    rate_cmd: float,
    rate_state: float,
    p: float,
    second_ord_lim: float,
) -> float:
    """Bias correction for velocity error in sqrt shaping.  Port of sqrt_controller_accel()."""
    if rate_cmd * rate_state <= 0.0:
        return 0.0
    if second_ord_lim <= 0.0:
        return -p * rate_state if p > 0.0 else 0.0
    if p <= 0.0:
        return 0.0 if rate_cmd == 0.0 else -(second_ord_lim / abs(rate_cmd)) * rate_state
    linear_dist = second_ord_lim / (p * p)
    if abs(error) <= linear_dist:
        return -p * rate_state
    return 0.0 if rate_cmd == 0.0 else -(second_ord_lim / abs(rate_cmd)) * rate_state


# ---------------------------------------------------------------------------
# attitude_command_model  (AC_AttitudeControl.cpp)
# ---------------------------------------------------------------------------

def _attitude_command_model(
    error_angle: float,
    target_ang_vel: float,
    target_ang_accel: float,
    max_ang_vel: float,
    accel_max: float,
    input_tc: float,
    dt: float,
) -> tuple[float, float]:
    """3rd-order (jerk-limited) angle shaper.  Port of attitude_command_model().

    Replaces the old 2-step _input_shaping_angle/_input_shaping_ang_vel pair
    with the exact jerk-limited shape_pos_vel_accel chain from AP_Math/control.cpp.

    Returns (new_target_ang_vel, new_target_ang_accel).
    """
    if dt <= 0.0:
        return target_ang_vel, target_ang_accel
    if accel_max <= 0.0:
        accel_max = math.radians(1800.0)
    if input_tc <= 0.0:
        input_tc = dt * 10.0

    jerk_max = accel_max / input_tc
    k_v = jerk_max / accel_max  # = 1/input_tc

    # pos_desired=error_angle, pos=0, vel_desired=0, accel_desired=0
    err = error_angle
    vel = target_ang_vel

    # Velocity correction from angle error (shape_pos_vel_accel core).
    vel_corr_cmd = _sqrt_controller(err, k_v, accel_max, dt)
    # Accel bias to account for closing rate.
    accel_corr = _sqrt_controller_accel(err, vel_corr_cmd, vel, k_v, accel_max)
    vel_corr_cmd += accel_corr / k_v

    # Velocity limits: vel_min=-max_ang_vel, vel_max=+max_ang_vel.
    # Only applied when max_ang_vel > 0 (0 = unlimited in AP).
    if max_ang_vel > 0.0:
        vel_corr_cmd = max(-max_ang_vel, min(max_ang_vel, vel_corr_cmd))
    vel_target = vel_corr_cmd
    if max_ang_vel > 0.0:  # limit_total
        vel_target = max(-max_ang_vel, min(max_ang_vel, vel_target))

    # Acceleration from velocity error.
    accel_target = (vel_target - vel) * k_v
    accel_target = max(-accel_max, min(accel_max, accel_target))
    accel_target = max(-accel_max, min(accel_max, accel_target))  # limit_total

    # Jerk-limit new acceleration, integrate velocity.
    new_accel = _shape_accel(accel_target, target_ang_accel, jerk_max, dt)
    new_vel = target_ang_vel + new_accel * dt
    return new_vel, new_accel


# ---------------------------------------------------------------------------
# input_shaping_ang_vel  (AC_AttitudeControl.cpp)
# ---------------------------------------------------------------------------

def _input_shaping_ang_vel(
    target_ang_vel: float,
    desired_ang_vel: float,
    accel_max: float,
    dt: float,
    input_tc: float,
) -> float:
    """Shape a body-frame angular-velocity request.

    Port of ``AC_AttitudeControl::input_shaping_ang_vel`` used by
    ``input_rate_bf_roll_pitch_yaw``.  ``accel_max`` is rad/s^2.
    """
    if input_tc > 0.0:
        error_rate = desired_ang_vel - target_ang_vel
        desired_ang_accel = _sqrt_controller(error_rate, 1.0 / max(input_tc, 0.01), 0.0, dt)
        desired_ang_vel = target_ang_vel + desired_ang_accel * dt
    if accel_max > 0.0:
        delta_ang_vel = accel_max * dt
        return max(target_ang_vel - delta_ang_vel, min(target_ang_vel + delta_ang_vel, desired_ang_vel))
    return desired_ang_vel


# ---------------------------------------------------------------------------
# inv_sqrt_controller  (AP_Math/control.cpp)
# ---------------------------------------------------------------------------

def _inv_sqrt_controller(output: float, p: float, D_max: float) -> float:
    """Inverse of sqrt_controller: distance -> stopping distance.  Port of inv_sqrt_controller()."""
    if D_max > 0.0 and p == 0.0:
        return (output * output) / (2.0 * D_max)
    if D_max <= 0.0 and p != 0.0:
        return output / p
    if D_max <= 0.0 and p == 0.0:
        return 0.0
    linear_velocity = D_max / p
    if abs(output) < linear_velocity:
        return output / p
    linear_dist = D_max / (p * p)
    stopping_dist = linear_dist * 0.5 + (output * output) / (2.0 * D_max)
    return stopping_dist if output > 0.0 else -stopping_dist


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
# thrust_heading_rotation_angles  (AC_AttitudeControl.cpp)
# ---------------------------------------------------------------------------

def _thrust_heading_rotation_angles(
    q_target: np.ndarray,
    q_body: np.ndarray,
    rate_yaw_kP: float,
    angle_yaw_kP: float,
    accel_yaw_max: float,
) -> tuple[np.ndarray, float, float, np.ndarray]:
    """Thrust-heading decomposition with yaw error clamp.

    Port of ``AC_AttitudeControl_Heli::thrust_heading_rotation_angles()``.
    Extends ``_thrust_vector_rotation_angles`` by clamping yaw error to the
    maximum that can be resolved without saturating the yaw rate output, and
    re-derives ``_attitude_target`` when the clamp is active so the slewed
    target does not accumulate unlimited yaw error.

    Returns
    -------
    att_error          : (3,) [roll_err, pitch_err, yaw_err] (yaw possibly clamped)
    thrust_angle       : current lean angle from vertical (rad)
    thrust_error_angle : angle between body and target thrust vectors (rad)
    new_q_target       : updated q_target (== q_target unless yaw was clamped)
    """
    att_error, thrust_angle, thrust_error_angle = _thrust_vector_rotation_angles(q_target, q_body)

    # heading_accel_max: half of yaw accel limit, clamped to physical bounds.
    # AP: constrain(accel_max * 0.5, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS,
    #               AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS)
    heading_accel_max = max(_ACCEL_Y_MIN, min(_ACCEL_Y_MAX, accel_yaw_max * 0.5))

    if rate_yaw_kP > 0.0 and angle_yaw_kP > 0.0:
        # Maximum yaw error = stopping distance from 1/rate_kP at angle_kP gain.
        heading_error_max = min(
            _inv_sqrt_controller(1.0 / rate_yaw_kP, angle_yaw_kP, heading_accel_max),
            _YAW_MAX_ERROR_ANGLE_RAD,
        )
        if abs(att_error[2]) > heading_error_max:
            # Clamp yaw error and re-derive attitude_target so the shaper
            # does not integrate unbounded yaw offset.
            yaw_clamped = max(-heading_error_max, min(heading_error_max,
                              _wrap_pi_scalar(att_error[2])))
            att_error[2] = yaw_clamped

            rot_body = Rotation.from_quat(q_body)
            rot_target = Rotation.from_quat(q_target)
            thrust_up = np.array([0.0, 0.0, -1.0])
            body_tv = rot_body.apply(thrust_up)
            target_tv = rot_target.apply(thrust_up)
            cross_ned = np.cross(body_tv, target_tv)
            tvc_angle = math.acos(float(np.clip(np.dot(body_tv, target_tv), -1.0, 1.0)))
            cross_len = float(np.linalg.norm(cross_ned))
            if cross_len < 1e-10 or tvc_angle < 1e-10:
                cross_ned = thrust_up
            else:
                cross_ned = cross_ned / cross_len
            cross_body = rot_body.inv().apply(cross_ned)
            rot_tvc = Rotation.from_rotvec(cross_body * tvc_angle)
            heading_correction = Rotation.from_rotvec(np.array([0.0, 0.0, yaw_clamped]))
            q_target = (rot_body * rot_tvc * heading_correction).as_quat()

    return att_error, thrust_angle, thrust_error_angle, q_target


def _wrap_pi_scalar(x: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


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
    ATC_ANG_RLL_P: float | None = None
    ATC_ANG_PIT_P: float | None = None
    ATC_ANG_YAW_P: float | None = None

    # Accel limits for sqrt/shaping (centi-deg/s^2). 0 = linear P only.
    # ATC_ACCEL_R_MAX, ATC_ACCEL_P_MAX, ATC_ACCEL_Y_MAX
    ATC_ACCEL_R_MAX: float | None = None
    ATC_ACCEL_P_MAX: float | None = None
    ATC_ACCEL_Y_MAX: float | None = None

    # Max body-frame angular velocity (deg/s). 0 = unlimited.
    # ATC_RATE_R_MAX, ATC_RATE_P_MAX, ATC_RATE_Y_MAX
    ATC_RATE_R_MAX: float | None = None
    ATC_RATE_P_MAX: float | None = None
    ATC_RATE_Y_MAX: float | None = None

    # Input shaping time constant (s).  ATC_INPUT_TC
    # AP default: 0.15 (Medium) from AC_AttitudeControl.cpp
    ATC_INPUT_TC: float | None = None
    # Yaw input shaping time constant (s).  AP uses a separate _rate_y_tc value
    # (AC_AttitudeControl._rate_y_tc) which is set to ATC_INPUT_TC by default.
    # Provide separately so yaw can be slowed independently if needed.
    ATC_INPUT_TC_YAW: float | None = None

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

    # Direct-thrust path (set_target_angle_and_rate_and_throttle).
    # ATC_ANGLE_BOOST: scale collective by 1/cos(tilt) to maintain vertical thrust.
    # AP parameter: AC_AttitudeControl ANGLE_BOOST (default 1 = enabled).
    # For RAWES set False: collective controls blade pitch for tether tension, not hover thrust.
    ATC_ANG_BOOST: bool = False
    # Normalised [0..1] collective at which the angle-boost pivot is applied.
    # Matches AP_MotorsHeli::get_coll_mid() scaled to [0,1].  Default 0.5.
    H_COL_MID_norm: float = 0.5

    def __post_init__(self) -> None:
        ap = _guided_ap_defaults()
        if self.ATC_ANG_RLL_P is None:
            self.ATC_ANG_RLL_P = ap["ATC_ANG_RLL_P"]
        if self.ATC_ANG_PIT_P is None:
            self.ATC_ANG_PIT_P = ap["ATC_ANG_PIT_P"]
        if self.ATC_ANG_YAW_P is None:
            self.ATC_ANG_YAW_P = ap["ATC_ANG_YAW_P"]
        if self.ATC_ACCEL_R_MAX is None:
            self.ATC_ACCEL_R_MAX = ap["ATC_ACCEL_R_MAX"]
        if self.ATC_ACCEL_P_MAX is None:
            self.ATC_ACCEL_P_MAX = ap["ATC_ACCEL_P_MAX"]
        if self.ATC_ACCEL_Y_MAX is None:
            self.ATC_ACCEL_Y_MAX = ap["ATC_ACCEL_Y_MAX"]
        if self.ATC_RATE_R_MAX is None:
            self.ATC_RATE_R_MAX = ap["ATC_RATE_R_MAX"]
        if self.ATC_RATE_P_MAX is None:
            self.ATC_RATE_P_MAX = ap["ATC_RATE_P_MAX"]
        if self.ATC_RATE_Y_MAX is None:
            self.ATC_RATE_Y_MAX = ap["ATC_RATE_Y_MAX"]
        if self.ATC_INPUT_TC is None:
            self.ATC_INPUT_TC = ap["ATC_INPUT_TC"]
        if self.ATC_INPUT_TC_YAW is None:
            self.ATC_INPUT_TC_YAW = self.ATC_INPUT_TC

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
            ATC_INPUT_TC=p.ATC_INPUT_TC,
            ATC_INPUT_TC_YAW=p.ATC_INPUT_TC,
        )


# ---------------------------------------------------------------------------
# GuidedAttitudeController
# ---------------------------------------------------------------------------

class GuidedAttitudeController:
    """Full GUIDED-mode attitude controller: slewed target + outer P + rate PIDs.

    Faithfully mirrors ``AC_AttitudeControl::input_quaternion`` including:

    * Internal ``_attitude_target`` quaternion that slews toward the commanded
      target via ``attitude_command_model`` (3rd-order jerk-limited shaper,
      ``ATC_INPUT_TC`` / ``ATC_INPUT_TC_YAW``, default 0.15 s).
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
        # _ang_accel_target: jerk-limited acceleration state for attitude_command_model.
        # (AP: _ang_accel_target, persistent across ticks)
        self._ang_accel_target: np.ndarray = np.zeros(3)
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

        # State for set_target_angle_and_rate_and_throttle (use_thrust=true path).
        # _ang_vel_body_rads: body-frame angular velocity feedforward (rad/s).  Stored
        # from the Lua call and used to advance _q_commanded at 400 Hz, exactly as
        # ArduPilot advances attitude_desired_quat inside input_quaternion.
        self._ang_vel_body_rads: np.ndarray = np.zeros(3)
        # _thrust_direct: raw thrust [0..1] from Lua, or None when using climbrate path.
        self._thrust_direct: float | None = None
        # Rate-only GUIDED target from vehicle:set_target_rate_and_throttle.
        # ArduPilot represents this as a zero quaternion in ModeGuided::set_angle.
        self._rate_only: bool = False
        self._rate_command_rads: np.ndarray = np.zeros(3)

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
        ang_vel_body is zero (Copter.cpp: mode_guided.set_angle(q, Vector3f{}, ...)).
        """
        r = Rotation.from_euler('ZYX', [yaw_deg, pitch_deg, roll_deg], degrees=True)
        self._q_commanded = r.as_quat()
        self._ang_vel_body_rads[:] = 0.0
        self._rate_command_rads[:] = 0.0
        self._last_rate_target_rads: np.ndarray = np.zeros(3)  # Initialize last rate target
        self._thrust_direct = None
        self._target_set = True
        self._last_target_time = sim_time
        self.climbrate_ms = climbrate_ms

    def set_target_angle_and_rate_and_throttle(
        self,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        roll_rate_degs: float,
        pitch_rate_degs: float,
        yaw_rate_degs: float,
        throttle: float,
        *,
        sim_time: float = 0.0,
    ) -> None:
        """Match ``vehicle:set_target_angle_and_rate_and_throttle`` exactly.

        ArduPilot call chain (Copter.cpp:set_target_angle_and_rate_and_throttle):

            q.from_euler(roll_rad, pitch_rad, yaw_rad)
            ang_vel_body = {roll_rate_degs, pitch_rate_degs, yaw_rate_degs} * DEG_TO_RAD
            mode_guided.set_angle(q, ang_vel_body, throttle, use_thrust=True)

        400 Hz angle_control_run:
            attitude_control->input_quaternion(attitude_quat, ang_vel_body)
                -- ang_vel_body advances attitude_desired_quat each tick:
                   ang_vel_target = attitude_desired_quat * ang_vel_body   (to world frame)
                   attitude_desired_quat *= from_axis_angle(ang_vel_target * dt)
            attitude_control->set_throttle_out(thrust_norm, apply_angle_boost=true, filt)
                -- direct to collective; Z PID chain completely bypassed.
        """
        r = Rotation.from_euler('ZYX', [yaw_deg, pitch_deg, roll_deg], degrees=True)
        self._q_commanded = r.as_quat()
        self._ang_vel_body_rads = np.array([
            math.radians(roll_rate_degs),
            math.radians(pitch_rate_degs),
            math.radians(yaw_rate_degs),
        ])
        self._rate_command_rads[:] = 0.0
        self._rate_only = False
        self._thrust_direct = float(throttle)
        self._target_set = True
        self._last_target_time = sim_time
        # Clear climbrate path state so there is no cross-contamination if the
        # caller switches back to set_target_angle_and_climbrate later.
        self.climbrate_ms = 0.0

    def set_target_rate_and_throttle(
        self,
        roll_rate_degs: float,
        pitch_rate_degs: float,
        yaw_rate_degs: float,
        throttle: float,
        *,
        sim_time: float = 0.0,
    ) -> None:
        """Match ``vehicle:set_target_rate_and_throttle``.

        ArduPilot call chain:

            Copter::set_target_rate_and_throttle
                q.zero()  -- marks rate-only Guided Angle control
                ang_vel_body = {roll,pitch,yaw}_deg_s * DEG_TO_RAD
                mode_guided.set_angle(q, ang_vel_body, throttle, use_thrust=True)

            ModeGuided::run_angle_control
                if attitude_quat.is_zero():
                    attitude_control->input_rate_bf_roll_pitch_yaw(...)
                attitude_control->set_throttle_out(thrust, apply_angle_boost=true, filt)

        This does not require the caller to know an absolute target attitude.
        The internal ``_attitude_target`` is conditioned from the current body
        attitude and the commanded body-rate vector is shaped by the same
        acceleration/time-constant path ArduPilot uses.
        """
        self._rate_command_rads = np.array([
            math.radians(roll_rate_degs),
            math.radians(pitch_rate_degs),
            math.radians(yaw_rate_degs),
        ])
        self._ang_vel_body_rads[:] = 0.0
        self._rate_only = True
        self._thrust_direct = float(throttle)
        self._target_set = True
        self._last_target_time = sim_time
        self.climbrate_ms = 0.0

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
        self._rate_command_rads[:] = 0.0
        self._rate_only = False
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
            self._ang_accel_target[:] = 0.0
            self._ang_vel_body_rads[:] = 0.0
            self._rate_command_rads[:] = 0.0
            self._rate_only = False
            self._thrust_direct = None
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

        if self._rate_only:
            accel_r = _accel_param_to_radss(gp.ATC_ACCEL_R_MAX)
            accel_p = _accel_param_to_radss(gp.ATC_ACCEL_P_MAX)
            accel_y = _accel_param_to_radss(gp.ATC_ACCEL_Y_MAX)
            max_r = math.radians(gp.ATC_RATE_R_MAX) if gp.ATC_RATE_R_MAX > 0.0 else 0.0
            max_p = math.radians(gp.ATC_RATE_P_MAX) if gp.ATC_RATE_P_MAX > 0.0 else 0.0
            max_y = math.radians(gp.ATC_RATE_Y_MAX) if gp.ATC_RATE_Y_MAX > 0.0 else 0.0

            # AC_AttitudeControl::input_rate_bf_roll_pitch_yaw: shape the
            # requested body rates into _ang_vel_target, then run the same
            # quaternion attitude controller against the internally integrated
            # _attitude_target.  RATE_FF_ENAB=false is not modeled separately;
            # ArduCopter defaults it on and this is the path used by Copter.
            self._ang_vel_target[0] = _input_shaping_ang_vel(
                self._ang_vel_target[0], self._rate_command_rads[0], accel_r, dt, gp.ATC_INPUT_TC)
            self._ang_vel_target[1] = _input_shaping_ang_vel(
                self._ang_vel_target[1], self._rate_command_rads[1], accel_p, dt, gp.ATC_INPUT_TC)
            self._ang_vel_target[2] = _input_shaping_ang_vel(
                self._ang_vel_target[2], self._rate_command_rads[2], accel_y, dt, gp.ATC_INPUT_TC_YAW)

            att_error, _thrust_angle, thrust_error_angle, self._attitude_target = \
                _thrust_heading_rotation_angles(
                    self._attitude_target, q_body,
                    self._hp.yaw.P, gp.ATC_ANG_YAW_P, accel_y)

            rate_roll = _sqrt_controller(att_error[0], gp.ATC_ANG_RLL_P,
                                         max(_ACCEL_RP_MIN, min(_ACCEL_RP_MAX, accel_r * 0.5)), dt)
            rate_pitch = _sqrt_controller(att_error[1], gp.ATC_ANG_PIT_P,
                                          max(_ACCEL_RP_MIN, min(_ACCEL_RP_MAX, accel_p * 0.5)), dt)
            rate_yaw = _sqrt_controller(att_error[2], gp.ATC_ANG_YAW_P,
                                        max(_ACCEL_Y_MIN, min(_ACCEL_Y_MAX, accel_y * 0.5)), dt)
            ang_vel_body = np.array([rate_roll, rate_pitch, rate_yaw])

            rot_body = Rotation.from_quat(q_body)
            rot_t2b = rot_body.inv() * Rotation.from_quat(self._attitude_target)
            ang_vel_ff = rot_t2b.apply(self._ang_vel_target)
            if thrust_error_angle > _THRUST_ERROR_ANGLE * 2.0:
                ang_vel_body[2] = gyro[2]
            elif thrust_error_angle > _THRUST_ERROR_ANGLE:
                ff_scalar = 1.0 - (thrust_error_angle - _THRUST_ERROR_ANGLE) / _THRUST_ERROR_ANGLE
                ang_vel_body[0] += ang_vel_ff[0] * ff_scalar
                ang_vel_body[1] += ang_vel_ff[1] * ff_scalar
                ang_vel_body[2] += ang_vel_ff[2]
                ang_vel_body[2] = gyro[2] * (1.0 - ff_scalar) + ang_vel_body[2] * ff_scalar
            else:
                ang_vel_body += ang_vel_ff

            if max_r > 0.0: ang_vel_body[0] = max(-max_r, min(max_r, ang_vel_body[0]))
            if max_p > 0.0: ang_vel_body[1] = max(-max_p, min(max_p, ang_vel_body[1]))
            if max_y > 0.0: ang_vel_body[2] = max(-max_y, min(max_y, ang_vel_body[2]))
            self._last_rate_target_rads = ang_vel_body.copy()

            out = self._rate_ctrl.update(
                rate_target_rads=tuple(ang_vel_body),
                gyro_rate_rads=tuple(gyro),
                dt=dt,
                collective_norm=collective_norm,
                saturated=saturated,
                sim_time_s=sim_time,
            )
            out.collective_norm_cmd = self._apply_throttle_out(
                float(self._thrust_direct if self._thrust_direct is not None else 0.0),
                q_body,
                _thrust_angle,
            )
            return out

        # === Step 2: attitude error _attitude_target.inv * q_commanded ===
        # AP: attitude_error_quat = _attitude_target.inverse() * attitude_desired_quat
        rot_cmd = Rotation.from_quat(self._q_commanded)
        err_rot = rot_att.inv() * rot_cmd
        err_rotvec = err_rot.as_rotvec()

        # === Step 3: attitude_command_model per axis => update _ang_vel_target/_ang_accel_target ===
        # AP: attitude_command_model(wrap_PI(error), 0.0, _ang_vel_target[i], _ang_accel_target[i],
        #       max_ang_vel[i], accel_max[i], input_tc[i], dt)
        # Roll/pitch use ATC_INPUT_TC; yaw uses ATC_INPUT_TC_YAW.
        accel_r = _accel_param_to_radss(gp.ATC_ACCEL_R_MAX)
        accel_p = _accel_param_to_radss(gp.ATC_ACCEL_P_MAX)
        accel_y = _accel_param_to_radss(gp.ATC_ACCEL_Y_MAX)
        max_r = math.radians(gp.ATC_RATE_R_MAX) if gp.ATC_RATE_R_MAX > 0.0 else 0.0
        max_p = math.radians(gp.ATC_RATE_P_MAX) if gp.ATC_RATE_P_MAX > 0.0 else 0.0
        max_y = math.radians(gp.ATC_RATE_Y_MAX) if gp.ATC_RATE_Y_MAX > 0.0 else 0.0

        self._ang_vel_target[0], self._ang_accel_target[0] = _attitude_command_model(
            _wrap_pi_scalar(err_rotvec[0]), self._ang_vel_target[0], self._ang_accel_target[0],
            max_r, accel_r, gp.ATC_INPUT_TC, dt)
        self._ang_vel_target[1], self._ang_accel_target[1] = _attitude_command_model(
            _wrap_pi_scalar(err_rotvec[1]), self._ang_vel_target[1], self._ang_accel_target[1],
            max_p, accel_p, gp.ATC_INPUT_TC, dt)
        self._ang_vel_target[2], self._ang_accel_target[2] = _attitude_command_model(
            _wrap_pi_scalar(err_rotvec[2]), self._ang_vel_target[2], self._ang_accel_target[2],
            max_y, accel_y, gp.ATC_INPUT_TC_YAW, dt)

        # === Step 4: attitude_controller_run_quat() (thrust_heading_rotation_angles) ===
        # Error between slewed _attitude_target and actual body attitude.
        # Also clamps yaw error to prevent saturation and re-derives _attitude_target.
        att_error, _thrust_angle, thrust_error_angle, self._attitude_target = \
            _thrust_heading_rotation_angles(
                self._attitude_target, q_body,
                self._hp.yaw.P, gp.ATC_ANG_YAW_P, accel_y)

        # Outer P-loop: attitude error -> body-frame rate correction.
        # AP: update_ang_vel_target_from_att_error(attitude_error)
        # Accel limit is halved (as in AP) and clamped to physical bounds.
        rate_roll  = _sqrt_controller(att_error[0], gp.ATC_ANG_RLL_P,
                                      max(_ACCEL_RP_MIN, min(_ACCEL_RP_MAX, accel_r * 0.5)), dt)
        rate_pitch = _sqrt_controller(att_error[1], gp.ATC_ANG_PIT_P,
                                      max(_ACCEL_RP_MIN, min(_ACCEL_RP_MAX, accel_p * 0.5)), dt)
        rate_yaw   = _sqrt_controller(att_error[2], gp.ATC_ANG_YAW_P,
                                      max(_ACCEL_Y_MIN,  min(_ACCEL_Y_MAX,  accel_y * 0.5)), dt)
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
        self._last_rate_target_rads = ang_vel_body.copy()

        # === Step 5: Advance _q_commanded by ang_vel_body feedforward ===
        # Mirrors AP input_quaternion step 4: advance attitude_desired_quat by
        #   ang_vel_target = attitude_desired_quat * ang_vel_body_rads   (body->world)
        #   attitude_desired_quat *= from_axis_angle(ang_vel_target * dt)
        # This extrapolates the target at 400 Hz between 50 Hz Lua calls.
        # Only active when ang_vel is non-zero (set_target_angle_and_rate_and_throttle).
        if np.any(self._ang_vel_body_rads != 0.0):
            rot_cmd_now = Rotation.from_quat(self._q_commanded)
            ang_vel_world = rot_cmd_now.apply(self._ang_vel_body_rads)
            self._q_commanded = (rot_cmd_now * Rotation.from_rotvec(ang_vel_world * dt)).as_quat()

        # === Step 6: HeliRateController (rate PIDs + swash) ===
        out = self._rate_ctrl.update(
            rate_target_rads=tuple(ang_vel_body),
            gyro_rate_rads=tuple(gyro),
            dt=dt,
            collective_norm=collective_norm,
            saturated=saturated,
            sim_time_s=sim_time,
        )

        # === Step 7: Collective command ===
        if self._thrust_direct is not None:
            # set_target_angle_and_rate_and_throttle path: direct thrust.
            # Mirrors AP: set_throttle_out(thrust_norm, apply_angle_boost=true, filt)
            # _thrust_angle_rad is the current lean angle computed in Step 4.
            out.collective_norm_cmd = self._apply_throttle_out(
                self._thrust_direct, q_body, _thrust_angle,
            )
        else:
            # set_target_angle_and_climbrate path: closed-loop altitude PID.
            out.collective_norm_cmd = self._compute_collective_norm(
                q_body=q_body,
                dt=dt,
                pos_z_up_m=pos_z_up_m,
                vel_z_up_mps=vel_z_up_mps,
            )
        return out

    def _apply_throttle_out(
        self,
        throttle_in: float,
        q_body: np.ndarray,
        thrust_angle_rad: float,
    ) -> float:
        """Port of AC_AttitudeControl_Heli::set_throttle_out + get_throttle_boosted.

        ``throttle_in`` is the raw Lua thrust [0..1].
        Returns collective_norm_cmd in [-1..1] (arduloop convention).

        AP reference: AC_AttitudeControl_Heli.cpp:450-485
            set_throttle_out(throttle_in, apply_angle_boost=true, filt)
            get_throttle_boosted(throttle_in):
                cos_tilt        = ahrs.cos_pitch() * ahrs.cos_roll()     <- actual body attitude
                inverted_factor = constrain(2*cos_tilt, -1, 1)
                cos_tilt_target = |cos(_thrust_angle_rad)|               <- from attitude_controller_run_quat
                boost_factor    = 1 / constrain(cos_tilt_target, 0.1, 1)
                coll_mid        = H_COL_MID normalised
                throttle_out    = (throttle_in - coll_mid) * inverted_factor * boost_factor + coll_mid
        """
        gp = self._gp
        if gp.ATC_ANG_BOOST:
            # cos_tilt from actual body attitude: cos_pitch * cos_roll = R_body[2,2] in NED.
            # body-z in NED for a level vehicle is [0,0,+1], so R[2,2]=1 -> cos_tilt=1.
            body_z_ned = Rotation.from_quat(q_body).apply(np.array([0.0, 0.0, 1.0]))
            cos_tilt = float(body_z_ned[2])
            inverted_factor = max(-1.0, min(1.0, 2.0 * cos_tilt))
            cos_tilt_target = abs(math.cos(thrust_angle_rad))
            boost_factor = 1.0 / max(0.1, cos_tilt_target)
            coll_mid = gp.H_COL_MID_norm
            throttle_in = (throttle_in - coll_mid) * inverted_factor * boost_factor + coll_mid

        throttle_in = max(0.0, min(1.0, throttle_in))
        # Convert [0,1] throttle to arduloop [-1,1] collective_norm_cmd.
        return throttle_in * 2.0 - 1.0

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
        self._ang_accel_target[:] = 0.0
        self._ang_vel_body_rads[:] = 0.0
        self._rate_command_rads[:] = 0.0
        self._rate_only = False
        self._thrust_direct = None
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

    @property
    def attitude_target_euler_deg(self) -> np.ndarray:
        """Internal slewed _attitude_target as ZYX Euler [roll, pitch, yaw] in degrees."""
        zyx = self.attitude_target_rotation.as_euler('ZYX', degrees=True)
        return np.array([zyx[2], zyx[1], zyx[0]])

    @property
    def last_rate_target_rads(self) -> np.ndarray:
        """Last body-rate target sent into HeliRateController (rad/s)."""
        return self._last_rate_target_rads.copy()
