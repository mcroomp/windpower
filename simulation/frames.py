"""
frames.py — Shared coordinate-frame constants and utilities for RAWES simulation.

All simulation modules import from here to avoid duplicated frame definitions.

Frames used:
  NED  — world frame: X=North, Y=East, Z=Down  (dynamics, aero, sensor output, SITL)
  body — hub body frame: columns of R_hub are body axes in world NED frame

Legacy constant T_ENU_NED is kept for converting ENU initial conditions / test data.
"""

import numpy as np


def cross3(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Cross product a × b for 1D length-3 arrays. Avoids np.cross overhead."""
    return np.array([
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ])


# ENU ↔ NED coordinate transform (symmetric: T @ T = I)
# Maps [E, N, U] ↔ [N, E, D]:
#   NED_x = ENU_y   (North = Y)
#   NED_y = ENU_x   (East  = X)
#   NED_z = -ENU_z  (Down  = -Up)
T_ENU_NED = np.array([
    [0., 1., 0.],
    [1., 0., 0.],
    [0., 0., -1.],
], dtype=float)


def build_gps_yaw_frame(body_z: np.ndarray) -> np.ndarray:
    """
    Build a rotation matrix with body_x purely horizontal (NED z-component = 0).

    body_x = normalize(body_z x NED_down) = normalize([body_z_y, -body_z_x, 0])

    This gives a RELPOSNED baseline vector that is completely horizontal regardless
    of tether tilt, so ArduPilot accepts the heading at any tether angle.
    Use this (not build_orb_frame) wherever GPS fusion is required.

    Falls back to build_orb_frame when body_z is vertical (tether pointing straight
    up or down — pathological case that never occurs in normal flight).
    """
    body_z = np.asarray(body_z, dtype=float)
    body_z = body_z / float(np.linalg.norm(body_z))
    _DOWN  = np.array([0.0, 0.0, 1.0])   # NED down
    x_orb  = cross3(body_z, _DOWN)
    norm   = float(np.linalg.norm(x_orb))
    if norm < 1e-6:
        return build_orb_frame(body_z)   # degenerate: body_z is vertical
    x_orb /= norm
    y_orb  = cross3(body_z, x_orb)
    return np.column_stack([x_orb, y_orb, body_z])


def build_vel_aligned_frame(body_z: np.ndarray, vel_ned: np.ndarray) -> np.ndarray:
    """
    Build an orbital rotation matrix whose ZYX yaw matches the GPS velocity heading.

    Ensures atan2(R[1,0], R[0,0]) == atan2(vel_ned[1], vel_ned[0]), so the
    compass heading ArduPilot reads matches the GPS velocity direction exactly.
    This gives zero heading gap from t=0, allowing GPS fusion during kinematic.

    Falls back to build_orb_frame when horizontal speed < 0.1 m/s.
    """
    body_z  = np.asarray(body_z,  dtype=float)
    vel_ned = np.asarray(vel_ned, dtype=float)
    v_horiz = float(np.sqrt(vel_ned[0]**2 + vel_ned[1]**2))
    if v_horiz < 0.1 or abs(body_z[2]) < 1e-6:
        return build_orb_frame(body_z)

    # Need x_orb in disk plane with atan2(x_orb[1], x_orb[0]) = psi = vel_heading.
    # x_orb = [cos(psi)*k, sin(psi)*k, c] with body_z · x_orb = 0:
    #   c = -(body_z[0]*cos(psi) + body_z[1]*sin(psi)) / body_z[2]  (per unit k)
    psi = float(np.arctan2(vel_ned[1], vel_ned[0]))
    cp  = float(np.cos(psi))
    sp  = float(np.sin(psi))
    c_per_k = -(body_z[0]*cp + body_z[1]*sp) / body_z[2]
    k       = 1.0 / float(np.sqrt(1.0 + c_per_k**2))
    x_orb   = np.array([cp*k, sp*k, c_per_k*k])
    y_orb   = cross3(body_z, x_orb)
    y_orb   = y_orb / float(np.linalg.norm(y_orb))
    return np.column_stack([x_orb, y_orb, body_z])


def build_orb_frame(body_z: np.ndarray) -> np.ndarray:
    """
    Build an orbital rotation matrix whose column 2 = body_z (rotor axle direction).

    The orbital frame removes rotor spin: body X is aligned with world East
    projected onto the disk plane, falling back to world North if the disk faces East.

    Used by:
      - mediator.py  — to build the initial R0 from body_z
      - sensor.py    — to extract physical orbital-frame Euler angles
      - aero.py      — (if needed) to express forces in disk frame

    Parameters
    ----------
    body_z : (3,) array  — unit vector pointing along rotor axle in world NED frame

    Returns
    -------
    R : (3, 3) rotation matrix whose columns are [x_orb, y_orb, body_z]
    """
    body_z = np.asarray(body_z, dtype=float)
    _EAST  = np.array([0.0, 1.0, 0.0])   # NED: East = Y axis
    _NORTH = np.array([1.0, 0.0, 0.0])   # NED: North = X axis
    east_proj = _EAST - np.dot(_EAST, body_z) * body_z
    if np.linalg.norm(east_proj) > 1e-6:
        x_orb = east_proj / np.linalg.norm(east_proj)
    else:
        north_proj = _NORTH - np.dot(_NORTH, body_z) * body_z
        x_orb = north_proj / np.linalg.norm(north_proj)
    y_orb = cross3(body_z, x_orb)
    return np.column_stack([x_orb, y_orb, body_z])
