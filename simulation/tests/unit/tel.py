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
altitude_m          float      = -pos_ned[2]
omega_rotor         float      rotor spin rate [rad/s]
collective_rad      float      collective blade pitch [rad]
tilt_lon            float      longitudinal swashplate tilt, normalised [-1,1]
tilt_lat            float      lateral swashplate tilt, normalised [-1,1]
tether_tension      float      tether tension [N]
tether_rest_length  float      paid-out tether length [m]
tether_slack        int        1 if tether is slack, 0 if taut
body_z_eq           [3]|null   orbit body-z setpoint (NED unit vec), or null
wind_ned            [3]        ambient wind NED [m/s]
tension_setpoint    float      tension PI setpoint [N] (0 if not applicable)

Tests may append extra keys via **extra for test-specific diagnostics
(e.g. xi_level_deg, wind_est_ned).  These are ignored by the visualizer.
"""
from __future__ import annotations

import numpy as np
from typing import Optional, Any


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
    **extra: Any,
) -> dict:
    """
    Build the canonical telemetry dict for one frame.

    Parameters
    ----------
    hub_state       dict with keys 'pos', 'vel', 'R' (NED numpy arrays).
    tether          TetherModel instance; provides rest_length + _last_info.
    tension_now     current tether tension [N].
    wind_ned        3-element array-like [vN, vE, vD] m/s.
    body_z_eq       optional NED unit vector (orbit setpoint); None if unknown.
    extra           test-specific extras passed through unchanged.
    """
    pos = np.asarray(hub_state["pos"], dtype=float)
    vel = np.asarray(hub_state["vel"], dtype=float)
    R   = np.asarray(hub_state["R"],   dtype=float)
    ti  = tether._last_info

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
    }
    if extra:
        d.update(extra)
    return d
