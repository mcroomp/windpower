"""
tel.py -- Canonical telemetry-frame helper for unit / simtests.

All simulation tests that append telemetry should use make_tel() so
every telemetry frame has the same schema.  Rows are written to CSV via
TelRow.from_tel() + write_csv(), and read by the visualizer (CSVSource)
and analysis scripts.

Canonical keys
--------------
t_sim               float      simulation time [s]
phase               str        pumping phase label ('' if not applicable)
pos_ned             [3]        hub position NED [m]
vel_ned             [3]        hub velocity NED [m/s]
R                   [3][3]     hub rotation matrix (body-to-NED)
omega_body          [3]        body angular velocity NED [rad/s]
accel_world         [3]        world-frame acceleration NED [m/s^2]
rpy                 [3]        ZYX Euler angles [roll, pitch, yaw] [rad]
altitude_m          float      = -pos_ned[2]
omega_rotor         float      rotor spin rate [rad/s]
collective_rad      float      collective blade pitch [rad]
tilt_lon            float      longitudinal swashplate tilt, normalised [-1,1]
tilt_lat            float      lateral swashplate tilt, normalised [-1,1]
tether_tension      float      tether tension [N]
tether_rest_length  float      paid-out tether length [m]
tether_slack        int        1 if tether is slack, 0 if taut
tether_force        [3]        tether force NED [N] (zeros if slack)
body_z_eq           [3]|null   orbit body-z setpoint (NED unit vec), or null
wind_ned            [3]        ambient wind NED [m/s]
tension_setpoint    float      tension PI setpoint [N] (0 if not applicable)
thrust_cmd          float      ThrustCommand.thrust [0..1] (0 if N/A)
winch_speed_ms      float      winch reel speed [m/s] +ve=out, -ve=in (0 if N/A)
raw_coll            float      AP unclamped collective integrator state [rad] (0 if N/A)
tilt_frac           float      AP tilt fraction 0..1 (0=alt-hold, 1=full tether tilt)
aero_F              [3]        aero force NED [N]
aero_T              float      aero thrust magnitude [N]
aero_v_axial        float      axial inflow velocity [m/s]
aero_v_inplane      float      in-plane wind speed [m/s]
aero_v_i            float      induced velocity [m/s]
aero_Q_drag         float      profile drag torque [N·m]
aero_Q_drive        float      autorotation drive torque [N·m]
net_F               [3]        net force NED [N] (aero + tether)
net_M               [3]        net orbital moment NED [N·m]

Tests may append extra keys via **extra for test-specific diagnostics
(e.g. xi_level_deg, wind_est_ned).  These are ignored by the visualizer.
"""
from __future__ import annotations

import math
import numpy as np
from typing import Optional, Any


def _rpy_from_R(R: np.ndarray) -> tuple[float, float, float]:
    """ZYX Euler angles from rotation matrix (body-to-NED)."""
    roll  = math.atan2(float(R[2, 1]), float(R[2, 2]))
    pitch = -math.asin(float(np.clip(R[2, 0], -1.0, 1.0)))
    yaw   = math.atan2(float(R[1, 0]), float(R[0, 0]))
    return roll, pitch, yaw


def make_tel(
    t_sim: float,
    hub_state: dict,
    omega_spin: float,
    tether,
    tension_now: float,
    collective_rad: float,
    tilt_lon: float,
    tilt_lat: float,
    wind_ned,
    *,
    body_z_eq: Optional[np.ndarray] = None,
    phase: str = "",
    tension_setpoint: float = 0.0,
    thrust_cmd: float = 0.0,
    winch_speed_ms: float = 0.0,
    raw_coll: float = 0.0,
    tilt_frac: float = 0.0,
    aero_result=None,
    aero_obj=None,
    tether_force: Optional[np.ndarray] = None,
    tether_moment: Optional[np.ndarray] = None,
    omega_body: Optional[np.ndarray] = None,
    accel_world: Optional[np.ndarray] = None,
    net_moment: Optional[np.ndarray] = None,
    **extra: Any,
) -> dict:
    """
    Build the canonical telemetry dict for one frame.

    Parameters
    ----------
    hub_state       dict with keys 'pos', 'vel', 'R', optionally 'omega' (NED).
    tether          TetherModel instance; provides rest_length + _last_info.
    tension_now     current tether tension [N].
    wind_ned        3-element array-like [vN, vE, vD] m/s.
    body_z_eq       optional NED unit vector (orbit setpoint); None if unknown.
    aero_result     AeroResult from aero.compute_forces() — adds F_world, M_orbital.
    aero_obj        Aero model instance — adds last_T, last_v_axial, etc.
    tether_force    NED tether force vector [N] — adds tether_fx/fy/fz.
    omega_body      body angular velocity NED [rad/s] — adds omega_x/y/z.
                    Falls back to hub_state["omega"] if present.
    accel_world     world-frame acceleration NED [m/s^2] — adds accel_x/y/z.
    net_moment      net orbital moment NED [N·m] — adds M_x/y/z.
    extra           test-specific extras passed through unchanged.
    """
    pos = np.asarray(hub_state["pos"], dtype=float)
    vel = np.asarray(hub_state["vel"], dtype=float)
    R   = np.asarray(hub_state["R"],   dtype=float)
    ti  = tether._last_info

    roll, pitch, yaw = _rpy_from_R(R)

    # omega_body: explicit arg > hub_state["omega"] > zeros
    if omega_body is not None:
        om = np.asarray(omega_body, dtype=float)
    elif "omega" in hub_state:
        om = np.asarray(hub_state["omega"], dtype=float)
    else:
        om = np.zeros(3)

    d: dict = {
        "t_sim":              float(t_sim),
        "phase":              str(phase),
        "pos_ned":            pos.tolist(),
        "vel_ned":            vel.tolist(),
        "R":                  R.tolist(),
        "altitude_m":         float(-pos[2]),
        "omega_rotor":        float(omega_spin),
        "collective_rad":     float(collective_rad),
        "tilt_lon":           float(tilt_lon),
        "tilt_lat":           float(tilt_lat),
        "tether_tension":     float(tension_now),
        "tether_rest_length": float(tether.rest_length),
        "tether_slack":       int(bool(ti.get("slack", False))),
        "body_z_eq":          (np.asarray(body_z_eq, dtype=float).tolist()
                               if body_z_eq is not None else None),
        "wind_ned":           [float(v) for v in wind_ned],
        "tension_setpoint":   float(tension_setpoint),
        "thrust_cmd":         float(thrust_cmd),
        "winch_speed_ms":     float(winch_speed_ms),
        "raw_coll":           float(raw_coll),
        "tilt_frac":          float(tilt_frac),
        # rotation and attitude
        "rpy":                [roll, pitch, yaw],
        "omega_body":         om.tolist(),
    }

    if accel_world is not None:
        d["accel_world"] = [float(v) for v in accel_world]

    if tether_force is not None:
        tf = np.asarray(tether_force, dtype=float)
        d["tether_force"] = tf.tolist()
    else:
        tf_raw = ti.get("force")
        if tf_raw is not None:
            d["tether_force"] = [float(v) for v in tf_raw]

    if aero_result is not None:
        F = np.asarray(aero_result.F_world, dtype=float)
        M = np.asarray(aero_result.M_orbital, dtype=float)
        d["aero_F"]  = F.tolist()
        d["aero_mx"] = float(M[0])
        d["aero_my"] = float(M[1])
        d["aero_mz"] = float(M[2])
        if aero_obj is not None:
            d["aero_T"]         = float(aero_obj.last_T)
            d["aero_v_axial"]   = float(aero_obj.last_v_axial)
            d["aero_v_inplane"] = float(aero_obj.last_v_inplane)
            d["aero_v_i"]       = float(aero_obj.last_v_i)
            d["aero_Q_drag"]    = float(aero_obj.last_Q_drag)
            d["aero_Q_drive"]   = float(aero_obj.last_Q_drive)
        if tether_force is not None:
            net = F + np.asarray(tether_force, dtype=float)
            d["net_F"] = net.tolist()
        if net_moment is not None:
            d["net_M"] = [float(v) for v in net_moment]

    if tether_moment is not None:
        tm = np.asarray(tether_moment, dtype=float)
        d["tether_mx"] = float(tm[0])
        d["tether_my"] = float(tm[1])
        d["tether_mz"] = float(tm[2])

    if extra:
        d.update(extra)
    return d
