"""
frames.py — Shared coordinate-frame constants and utilities for RAWES simulation.

All simulation modules import from here to avoid duplicated frame definitions.

Frames used:
  ENU  — world frame: X=East, Y=North, Z=Up  (dynamics, aero)
  NED  — ArduPilot: X=North, Y=East, Z=Down  (sensor output, SITL)
  body — hub body frame: columns of R_hub are body axes in world frame
"""

import numpy as np

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


def build_orb_frame(body_z: np.ndarray) -> np.ndarray:
    """
    Build an orbital rotation matrix whose column 2 = body_z (rotor axle direction).

    The orbital frame removes rotor spin: body X is aligned with world East
    projected onto the disk plane, falling back to world North if the disk faces East.

    Used by:
      - mediator.py  — to build the initial R0 from body_z
      - sensor.py    — to compute tether-relative attitude deviation
      - aero.py      — (if needed) to express forces in disk frame

    Parameters
    ----------
    body_z : (3,) array  — unit vector pointing along rotor axle in world ENU frame

    Returns
    -------
    R : (3, 3) rotation matrix whose columns are [x_orb, y_orb, body_z]
    """
    body_z = np.asarray(body_z, dtype=float)
    _EAST  = np.array([1.0, 0.0, 0.0])
    _NORTH = np.array([0.0, 1.0, 0.0])
    east_proj = _EAST - np.dot(_EAST, body_z) * body_z
    if np.linalg.norm(east_proj) > 1e-6:
        x_orb = east_proj / np.linalg.norm(east_proj)
    else:
        north_proj = _NORTH - np.dot(_NORTH, body_z) * body_z
        x_orb = north_proj / np.linalg.norm(north_proj)
    y_orb = np.cross(body_z, x_orb)
    return np.column_stack([x_orb, y_orb, body_z])
