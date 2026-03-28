"""
controller.py — RAWES attitude rate controller for ArduPilot ACRO mode.

Keeps body_z (rotor axle) aligned with the tether direction by computing
angular rate corrections from hub truth state and expressing them in the
yaw-rotated NED body frame that ArduPilot expects in ACRO mode.

Usage
-----
rc = compute_rc_rates(hub_state, anchor_pos, vel_ned)
gcs.send_rc_override(rc)   # replaces the neutral-stick send in test_acro_hold
"""

import numpy as np
from frames import T_ENU_NED, build_orb_frame


def compute_rc_rates(
    hub_state:    dict,
    anchor_pos:   np.ndarray,
    vel_ned:      np.ndarray,
    kp:           float = 0.5,
    kd:           float = 0.2,
    rate_max_deg: float = 200.0,
) -> dict:
    """
    Compute RC override channels to keep body_z aligned with the tether.

    Parameters
    ----------
    hub_state : dict
        Current rigid-body state with keys:
          ``pos``   — ENU position (3,)
          ``R``     — rotation matrix body→world (3, 3)
          ``omega`` — angular velocity in world ENU frame (3,)
    anchor_pos : array-like (3,)
        Tether anchor position in ENU world frame.
    vel_ned : array-like (3,)
        Hub velocity in NED frame (used to derive heading yaw for body-frame
        alignment with ArduPilot's ACRO controller).
    kp : float
        Proportional gain on attitude error (rad/s per rad).
    kd : float
        Derivative gain on orbital angular rate (dimensionless).
    rate_max_deg : float
        Angular rate corresponding to full stick deflection (degrees/s).

    Returns
    -------
    dict
        RC channel overrides: {1: pwm, 2: pwm, 4: pwm, 8: 2000}.
        Channel 1 = roll rate, 2 = pitch rate, 4 = yaw rate, 8 = motor interlock.
        Neutral = 1500, min = 1000, max = 2000.
    """
    pos   = np.asarray(hub_state["pos"],   dtype=float)
    R     = np.asarray(hub_state["R"],     dtype=float)
    omega = np.asarray(hub_state["omega"], dtype=float)
    anch  = np.asarray(anchor_pos,         dtype=float)

    # Safety: if hub is at/inside anchor, return neutral
    tether = pos - anch
    t_len  = float(np.linalg.norm(tether))
    if t_len < 0.1:
        return {1: 1500, 2: 1500, 4: 1500, 8: 2000}

    # Desired body_z = tether direction (pointing away from anchor)
    body_z_eq  = tether / t_len

    # Actual body_z = third column of rotation matrix
    body_z_cur = R[:, 2]

    # Attitude error in world ENU frame (cross product → rotation axis toward target)
    error_world = np.cross(body_z_cur, body_z_eq)

    # Strip spin component from omega (spin is along body_z, not an orbital rate)
    omega_spin    = np.dot(omega, body_z_cur) * body_z_cur
    omega_orbital = omega - omega_spin

    # Rate correction in ENU world frame: P term + D term
    omega_corr_enu = kp * error_world - kd * omega_orbital

    # Transform to NED
    omega_corr_ned = T_ENU_NED @ omega_corr_enu

    # Rotate into ArduPilot's yaw-aligned body frame
    vel_ned  = np.asarray(vel_ned, dtype=float)
    v_horiz  = float(np.hypot(vel_ned[0], vel_ned[1]))
    yaw      = float(np.arctan2(vel_ned[1], vel_ned[0])) if v_horiz > 0.1 else 0.0
    cy, sy   = np.cos(yaw), np.sin(yaw)
    Rz_inv   = np.array([[cy, sy, 0.], [-sy, cy, 0.], [0., 0., 1.]])
    omega_body = Rz_inv @ omega_corr_ned

    # Map rad/s to PWM [1000, 2000], 1500 = zero
    max_rate = np.radians(rate_max_deg)

    def _pwm(w: float) -> int:
        return int(round(1500.0 + 500.0 * float(np.clip(w / max_rate, -1.0, 1.0))))

    return {
        1: _pwm(omega_body[0]),   # roll rate
        2: _pwm(omega_body[1]),   # pitch rate
        4: _pwm(omega_body[2]),   # yaw rate
        8: 2000,                  # motor interlock always on
    }


def compute_swashplate_from_state(
    hub_state:    dict,
    anchor_pos:   np.ndarray,
    kp:           float = 0.5,
    kd:           float = 0.2,
    tilt_max_rad: float = 0.3,
) -> dict:
    """
    Compute swashplate tilt commands directly from hub truth state.

    This bypasses ArduPilot entirely — the mediator can call this directly
    and feed tilt_lon/tilt_lat into aero.compute_forces().

    Parameters
    ----------
    hub_state   : dict with pos (ENU), R (body→world), omega (world ENU)
    anchor_pos  : tether anchor in ENU [m]
    kp          : proportional gain on attitude error [rad/rad]
    kd          : derivative gain on orbital rate [rad·s/rad]
    tilt_max_rad: maximum swashplate tilt [rad] (saturation limit)

    Returns
    -------
    dict: {collective_rad: float, tilt_lon: float, tilt_lat: float}
          tilt_lon/tilt_lat in [-1, 1] (normalised swashplate coordinates)
    """
    pos   = np.asarray(hub_state["pos"],   dtype=float)
    R     = np.asarray(hub_state["R"],     dtype=float)
    omega = np.asarray(hub_state["omega"], dtype=float)
    anch  = np.asarray(anchor_pos,         dtype=float)

    # Safety: if hub is at/inside anchor, return neutral
    tether = pos - anch
    t_len  = float(np.linalg.norm(tether))
    if t_len < 0.1:
        return {"collective_rad": 0.0, "tilt_lon": 0.0, "tilt_lat": 0.0}

    body_z_eq  = tether / t_len
    body_z_cur = R[:, 2]

    error_world = np.cross(body_z_cur, body_z_eq)

    omega_spin    = np.dot(omega, body_z_cur) * body_z_cur
    omega_orbital = omega - omega_spin

    # Correction in world ENU frame
    corr_enu = kp * error_world - kd * omega_orbital

    # Project onto disk-plane axes (tilt_lon = along disk X, tilt_lat = along disk Y)
    disk_x = R[:, 0]
    disk_y = R[:, 1]

    tilt_lon = float(np.clip(np.dot(corr_enu, disk_x) / tilt_max_rad, -1.0, 1.0))
    tilt_lat = float(np.clip(np.dot(corr_enu, disk_y) / tilt_max_rad, -1.0, 1.0))

    return {
        "collective_rad": 0.0,   # collective held at zero; attitude via tilt only
        "tilt_lon":       tilt_lon,
        "tilt_lat":       tilt_lat,
    }


def compute_rc_from_attitude(
    roll:         float,
    pitch:        float,
    rollspeed:    float,
    pitchspeed:   float,
    yawspeed:     float,
    kp:           float = 1.0,
    kd:           float = 0.3,
    rate_max_deg: float = 200.0,
) -> dict:
    """
    Compute RC override channels from ArduPilot ATTITUDE message fields.

    Because the mediator reports **tether-relative** attitude (roll=0, pitch=0
    when body_z is on the tether), the ATTITUDE message roll/pitch are directly
    the attitude error from tether equilibrium.  rollspeed/pitchspeed/yawspeed
    are the body-frame angular rates that ArduPilot's ACRO loop is controlling.

    The controller commands:
        cmd_roll  = -kp * roll  - kd * rollspeed
        cmd_pitch = -kp * pitch - kd * pitchspeed
        cmd_yaw   = -kd * yawspeed

    This keeps body_z aligned with the tether and damps all body-frame rates.

    Parameters
    ----------
    roll, pitch           : tether-relative attitude angles [rad]
    rollspeed, pitchspeed, yawspeed : body-frame angular rates [rad/s]
    kp                    : attitude error gain [rad/s per rad]
    kd                    : rate damping gain [dimensionless]
    rate_max_deg          : full-stick rate [deg/s] → maps to PWM 1000/2000

    Returns
    -------
    dict : {1: pwm, 2: pwm, 4: pwm, 8: 2000}
    """
    max_rate = np.radians(rate_max_deg)

    def _pwm(w: float) -> int:
        return int(round(1500.0 + 500.0 * float(np.clip(w / max_rate, -1.0, 1.0))))

    cmd_roll  = -kp * float(roll)  - kd * float(rollspeed)
    cmd_pitch = -kp * float(pitch) - kd * float(pitchspeed)
    cmd_yaw   =                    - kd * float(yawspeed)

    return {
        1: _pwm(cmd_roll),
        2: _pwm(cmd_pitch),
        4: _pwm(cmd_yaw),
        8: 2000,
    }
