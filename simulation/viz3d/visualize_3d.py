"""
visualize_3d.py — 3D playback of RAWES simulation telemetry.

Geometry matches aero.py exactly:
  n_blades=4, r_root=0.5 m, r_tip=2.5 m, chord=0.15 m, 90° blade spacing.

Scene elements:
  - Rotor disk (semi-transparent annulus, r_root→r_tip, oriented by R)
  - 4 blades (rectangles, chord=0.15 m, spinning at omega_spin)
  - Hub sphere
  - Rotor axle arrow (body_z, green)
  - Equilibrium target arrow (body_z_eq, yellow — when available)
  - Tether line (orange, anchor → hub)
  - Anchor marker
  - Wind arrow (at hub)
  - Trajectory trail (last N frames)
  - Ground plane
  - HUD: t, alt, spin, tension, collective, tilt

Usage:
    # From unit test JSON (run test first with --save-telemetry):
    python visualize_3d.py telemetry_deschutter.json
    python visualize_3d.py telemetry_closed_loop.json

    # From mediator CSV (stack test):
    python visualize_3d.py /path/to/telemetry.csv --source csv

    # Export to GIF:
    python visualize_3d.py telemetry.json --export playback.gif --fps 10

Controls (interactive mode):
    Space       — play / pause
    Left/Right  — step one frame (while paused)
    +/-         — 2× / 0.5× playback speed
    Slider      — scrub to any frame
    Mouse       — orbit / pan / zoom (standard PyVista)
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import List, Optional

import numpy as np

try:
    from vtkmodules.vtkRenderingCore import (
        vtkRenderer, vtkPolyDataMapper, vtkActor,
    )
    _HAS_VTK = True
except ImportError:
    _HAS_VTK = False

try:
    import pyvista as pv
except ImportError:
    print("PyVista not found.  Install with:  pip install pyvista")
    sys.exit(1)

# Ensure viz3d package is importable when run as a script
_HERE = Path(__file__).resolve().parent
_SIM  = _HERE.parent
if str(_SIM) not in sys.path:
    sys.path.insert(0, str(_SIM))

from viz3d.telemetry import TelemetryFrame, JSONSource, CSVSource

# ---------------------------------------------------------------------------
# Rotor geometry — sourced from beaupoil_2026.yaml via RotorDefinition
# ---------------------------------------------------------------------------
import sys as _sys; _sys.path.insert(0, str(__import__('pathlib').Path(__file__).parents[1]))
import rotor_definition as _rd
_ROTOR           = _rd.default()
N_BLADES         = _ROTOR.n_blades
R_ROOT           = _ROTOR.root_cutout_m
R_TIP            = _ROTOR.radius_m
CHORD            = _ROTOR.chord_m
BLADE_THK        = 0.025  # m — slight thickness so blades are visible from the side
TETHER_AXLE_OFFSET = 0.4  # m — tether attaches this far below CM along -body_z
                            #    (bottom of axle, toward anchor side)
ANCHOR    = np.zeros(3)

# ---------------------------------------------------------------------------
# Visual style
# ---------------------------------------------------------------------------
COL_HUB      = "#d0d0d0"   # light grey
COL_DISK     = "#4488ff"   # blue (semi-transparent)
COL_BLADES   = ["#FF7043", "#4DD0E1", "#FFD54F", "#CE93D8"]  # deep-orange, cyan, amber, lavender
COL_TETHER   = "#ff8800"   # orange
COL_ANCHOR   = "#ff3300"   # red
COL_BODY_Z   = "#00ff44"   # green  — current axle
COL_BODY_ZEQ = "#ffee00"   # yellow — equilibrium target
COL_WIND     = "#88ccff"   # light blue
COL_TRAIL    = "#888888"   # grey
COL_GROUND   = "#2a4a2a"   # dark green

TRAIL_LEN = 200  # frames to keep in trajectory trail

# ---------------------------------------------------------------------------
# NED → PyVista display (ENU, Z=up) conversion
# ---------------------------------------------------------------------------
# The simulation uses NED (North=X, East=Y, Down=Z).
# PyVista renders with Z=up, so we display in ENU (East=X, North=Y, Up=Z).
#   E = NED_Y,  N = NED_X,  U = -NED_Z
_T_NED_ENU = np.array([
    [0., 1., 0.],   # ENU_X = NED_Y (East)
    [1., 0., 0.],   # ENU_Y = NED_X (North)
    [0., 0., -1.],  # ENU_Z = -NED_Z (Up)
])

def _to_viz(v: np.ndarray) -> np.ndarray:
    """Convert a NED vector/position to the PyVista ENU display frame."""
    return _T_NED_ENU @ np.asarray(v, dtype=float)

# Tether physical properties (matches tether.py / CLAUDE.md)
TETHER_LINEAR_MASS  = 0.0021   # kg/m  (Dyneema SK75 1.9 mm)
TETHER_TUBE_RADIUS  = 0.05     # m display radius (~26× actual; visible at scene scale)
TETHER_TENSION_MAX  = 300.0    # N — maps to red end of colour scale


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _tether_colour(tension: float) -> str:
    """
    Map tether tension to a colour.
        slack / 0 N  →  dim grey
        low   ~50 N  →  steel blue
        mid  ~150 N  →  orange
        high ~300 N  →  red
    """
    if tension < 1.0:
        return "#555566"
    t = float(np.clip(tension / TETHER_TENSION_MAX, 0.0, 1.0))
    if t < 0.5:
        s = t * 2.0                          # 0→1 across low half
        r = int(0x22 + s * (0xff - 0x22))   # 0x22 → 0xff
        g = int(0x88 + s * (0x88 - 0x88))   # 0x88 constant
        b = int(0xdd + s * (0x00 - 0xdd))   # 0xdd → 0x00
    else:
        s = (t - 0.5) * 2.0                  # 0→1 across high half
        r = 0xff
        g = int(0x88 + s * (0x22 - 0x88))   # 0x88 → 0x22
        b = 0x00
    return f"#{r:02x}{g:02x}{b:02x}"


def _blade_mesh_local(k: int, spin_angle: float) -> pv.PolyData:
    """
    One blade as a thin box in the disk's local frame (Z = rotor axle).

    The blade runs radially at angle (spin_angle + k*90°) in the XY plane.
    Chord is perpendicular to the radial direction; thickness is along Z.
    """
    angle = spin_angle + k * (np.pi / 2)
    ca, sa  = np.cos(angle), np.sin(angle)
    radial  = np.array([ ca,  sa, 0.0])
    perp    = np.array([-sa,  ca, 0.0])   # chord direction
    normal  = np.array([ 0.0, 0.0, 1.0])  # disk normal (local)

    hc = CHORD  / 2
    ht = BLADE_THK / 2

    # 8 corners of the blade box
    pts = np.array([
        R_ROOT * radial - hc*perp - ht*normal,
        R_ROOT * radial + hc*perp - ht*normal,
        R_TIP  * radial + hc*perp - ht*normal,
        R_TIP  * radial - hc*perp - ht*normal,
        R_ROOT * radial - hc*perp + ht*normal,
        R_ROOT * radial + hc*perp + ht*normal,
        R_TIP  * radial + hc*perp + ht*normal,
        R_TIP  * radial - hc*perp + ht*normal,
    ])

    # 6 faces as quads
    faces = np.array([
        4, 0, 1, 2, 3,   # bottom
        4, 4, 5, 6, 7,   # top
        4, 0, 1, 5, 4,   # root side
        4, 3, 2, 6, 7,   # tip side
        4, 0, 3, 7, 4,   # trailing edge
        4, 1, 2, 6, 5,   # leading edge
    ])
    return pv.PolyData(pts, faces)


def _rotor_local(spin_angle: float) -> pv.PolyData:
    """Disk + all blades assembled in local frame (pre-transform)."""
    disk = pv.Disc(inner=R_ROOT, outer=R_TIP,
                   normal=[0, 0, 1], r_res=1, c_res=64)
    parts: List[pv.PolyData] = [disk]
    for k in range(N_BLADES):
        parts.append(_blade_mesh_local(k, spin_angle))
    return pv.merge(parts)


def _hub_to_world(R: np.ndarray, pos: np.ndarray) -> np.ndarray:
    """4×4 homogeneous transform: local disk frame → world ENU."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = pos
    return T


def _arrow(start: np.ndarray, direction: np.ndarray,
           scale: float = 1.0) -> pv.PolyData:
    n = np.linalg.norm(direction)
    if n < 1e-9:
        return pv.PolyData()
    return pv.Arrow(start=start, direction=direction / n,
                    scale=n * scale,
                    tip_length=0.25, tip_radius=0.08, shaft_radius=0.03)


def _human_figure(x: float = 3.0, y: float = 2.0) -> pv.PolyData:
    """
    Simple stick figure ~1.75 m tall standing at (x, y, 0).
    Head + torso + arms + legs built from spheres and cylinders.
    """
    def cyl(cx, cy, cz, dx, dy, dz, r, h):
        return pv.Cylinder(center=(cx, cy, cz), direction=(dx, dy, dz),
                           radius=r, height=h, resolution=12)

    parts = [
        # Head
        pv.Sphere(radius=0.115, center=(x, y, 1.635), theta_resolution=12,
                  phi_resolution=12),
        # Torso
        cyl(x, y, 1.15,  0, 0, 1, 0.065, 0.48),
        # Left upper arm (angled slightly down)
        cyl(x - 0.22, y, 1.28,  1, 0, -0.3, 0.03, 0.38),
        # Right upper arm
        cyl(x + 0.22, y, 1.28, -1, 0, -0.3, 0.03, 0.38),
        # Left leg
        cyl(x - 0.09, y, 0.45,  0, 0, 1, 0.045, 0.9),
        # Right leg
        cyl(x + 0.09, y, 0.45,  0, 0, 1, 0.045, 0.9),
    ]
    return pv.merge(parts)


def _tree(x: float, y: float, height: float = 5.0, seed: int = 0) -> pv.PolyData:
    """
    Simple conifer: brown trunk cylinder + two stacked green cones.
    ``seed`` offsets the proportions slightly for variety.
    """
    rng    = (seed * 7 + 13) % 10
    h      = height * (0.85 + rng * 0.03)
    r_base = h * 0.18
    trunk_h = h * 0.30
    parts  = [
        pv.Cylinder(center=(x, y, trunk_h / 2), direction=(0, 0, 1),
                    radius=h * 0.04, height=trunk_h, resolution=8),
        pv.Cone(center=(x, y, trunk_h + h * 0.30), direction=(0, 0, 1),
                height=h * 0.55, radius=r_base, resolution=12),
        pv.Cone(center=(x, y, trunk_h + h * 0.52), direction=(0, 0, 1),
                height=h * 0.42, radius=r_base * 0.75, resolution=12),
    ]
    return pv.merge(parts)


def _mountain(cx: float, cy: float, radius: float, height: float,
              n_faces: int = 6) -> pv.PolyData:
    """
    Irregular mountain silhouette: a jagged cone built from a polygon base.
    ``n_faces`` sets the number of flanks (5-8 gives a good rocky look).
    """
    import math as _math
    pts   = [[cx, cy, 0.0]]   # apex-ish at top; build from base up
    faces_list = []

    # Base ring with slight radial jitter
    base = []
    for i in range(n_faces):
        a   = 2.0 * _math.pi * i / n_faces
        r   = radius * (0.8 + 0.4 * ((i * 31 + 7) % 10) / 10.0)
        base.append([cx + r * _math.cos(a), cy + r * _math.sin(a), 0.0])

    # Summit jitter
    summit = [cx + radius * 0.08 * (((n_faces * 3) % 5) - 2),
              cy + radius * 0.05 * (((n_faces * 7) % 5) - 2),
              height * (0.88 + 0.12 * ((n_faces * 13) % 10) / 10.0)]

    all_pts = base + [summit]
    n_base  = len(base)
    summit_i = n_base

    triangles = []
    for i in range(n_base):
        j = (i + 1) % n_base
        triangles += [3, i, j, summit_i]   # side triangle
    # Base cap
    for i in range(1, n_base - 1):
        triangles += [3, 0, i, i + 1]

    mesh = pv.PolyData()
    mesh.points = np.array(all_pts, dtype=float)
    mesh.faces  = np.array(triangles, dtype=int)
    return mesh


def _ground_grid(extent: float = 200, spacing: float = 10) -> pv.PolyData:
    """
    Line grid on the z=0 plane, centred on the anchor.
    Returns a PolyData with one line segment per grid line.
    """
    lines_x = np.arange(-extent, extent + spacing, spacing)
    lines_y = np.arange(-extent, extent + spacing, spacing)

    pts: list[list[float]] = []
    segments: list[int] = []

    def add_line(p0, p1):
        i = len(pts)
        pts.append(p0)
        pts.append(p1)
        segments.extend([2, i, i + 1])

    for x in lines_x:
        add_line([x, -extent, 0.0], [x,  extent, 0.0])
    for y in lines_y:
        add_line([-extent, y, 0.0], [ extent, y, 0.0])

    mesh = pv.PolyData()
    mesh.points = np.array(pts, dtype=float)
    mesh.lines  = np.array(segments, dtype=int)
    return mesh


def _tether_catenary(
    anchor: np.ndarray,
    hub: np.ndarray,
    tension: float,
    n_pts: int = 40,
    linear_mass: float = TETHER_LINEAR_MASS,
) -> np.ndarray:
    """
    Parabolic catenary approximation for tether shape under gravity.

    The tether sags in the direction of the gravity component perpendicular
    to the chord, by:
        sag_max = μ · g_perp · L² / (8 · T)

    Valid for sag << span (holds here: sag ~0.3 m at 250 N over 50 m;
    nearly straight at reel-in since tether is near-vertical → g_perp → 0).

    Returns (n_pts, 3) array of world ENU positions from anchor to hub.
    """
    chord = hub - anchor
    L = np.linalg.norm(chord)
    if L < 0.1:
        return np.linspace(anchor, hub, n_pts)

    e_chord = chord / L
    g_vec = np.array([0.0, 0.0, -9.81])

    # Gravity component perpendicular to chord — drives sag
    g_perp_vec = g_vec - np.dot(g_vec, e_chord) * e_chord
    g_perp_mag = np.linalg.norm(g_perp_vec)

    s = np.linspace(0.0, 1.0, n_pts)
    straight = anchor[None, :] + s[:, None] * chord[None, :]

    if g_perp_mag < 0.01 or tension < 0.5:
        # Near-vertical tether or slack: gravity acts along cord, no lateral sag
        return straight

    sag_max = linear_mass * g_perp_mag * L ** 2 / (8.0 * tension)
    sag_dir = g_perp_vec / g_perp_mag          # direction tether sags toward
    sag     = sag_max * 4.0 * s * (1.0 - s)   # parabolic profile, zero at both ends
    return straight + sag[:, None] * sag_dir[None, :]


def _drop_line(pos_viz: np.ndarray, dot_spacing: float = 1.5) -> pv.PolyData:
    """Dotted vertical plumb line from hub straight down to z=0 (viz ENU frame)."""
    x, y, z = pos_viz
    if z <= 0:
        return pv.PolyData()
    n = max(1, int(z / dot_spacing) + 1)
    pts = np.array([
        [x, y, z - i * dot_spacing]
        for i in range(n)
        if z - i * dot_spacing >= 0
    ])
    return pv.PolyData(pts)


def _lerp_frame(f1, f2, alpha: float):
    """
    Linearly interpolate between two TelemetryFrames.

    alpha=0 → f1,  alpha=1 → f2.  Rotation matrices are lerp-then-renormalised
    (Gram-Schmidt on the first two columns), which is accurate for the small
    inter-frame angles at 20 Hz telemetry.
    """
    from viz3d.telemetry import TelemetryFrame
    a = float(np.clip(alpha, 0.0, 1.0))
    b = 1.0 - a

    # Rotation: linear interpolation of columns, re-orthogonalise
    R = b * f1.R + a * f2.R
    c0 = R[:, 0] / max(np.linalg.norm(R[:, 0]), 1e-12)
    c1 = R[:, 1] - np.dot(R[:, 1], c0) * c0
    c1 /= max(np.linalg.norm(c1), 1e-12)
    c2 = np.cross(c0, c1)
    R_out = np.column_stack([c0, c1, c2])

    bzeq = None
    if f1.body_z_eq is not None and f2.body_z_eq is not None:
        v = b * f1.body_z_eq + a * f2.body_z_eq
        n = np.linalg.norm(v)
        bzeq = v / n if n > 1e-9 else f1.body_z_eq.copy()

    return TelemetryFrame(
        t                  = b * f1.t + a * f2.t,
        pos_ned            = b * f1.pos_ned + a * f2.pos_ned,
        R                  = R_out,
        omega_spin         = b * f1.omega_spin + a * f2.omega_spin,
        tether_tension     = b * f1.tether_tension + a * f2.tether_tension,
        tether_rest_length = b * f1.tether_rest_length + a * f2.tether_rest_length,
        swash_collective   = b * f1.swash_collective + a * f2.swash_collective,
        swash_tilt_lon     = b * f1.swash_tilt_lon + a * f2.swash_tilt_lon,
        swash_tilt_lat     = b * f1.swash_tilt_lat + a * f2.swash_tilt_lat,
        body_z_eq          = bzeq,
        wind_ned           = b * f1.wind_ned + a * f2.wind_ned,
    )


def _dir_mat4(start: np.ndarray, direction: np.ndarray) -> np.ndarray:
    """
    4×4 transform that maps a canonical unit +Z arrow to an arrow at ``start``
    pointing in ``direction`` with length ``|direction|``.
    Used to reorient pre-created arrow actors via user_matrix.
    """
    d      = np.asarray(direction, dtype=float)
    length = float(np.linalg.norm(d))
    if length < 1e-9:
        M = np.eye(4, dtype=float); M[:3, 3] = start; return M
    d_hat = d / length

    z     = np.array([0., 0., 1.])
    axis  = np.cross(z, d_hat)
    sin_a = float(np.linalg.norm(axis))
    cos_a = float(np.dot(z, d_hat))

    if sin_a < 1e-9:
        R3 = np.eye(3) if cos_a > 0 else np.diag([1., -1., -1.]).astype(float)
    else:
        axis /= sin_a
        K  = np.array([[0, -axis[2], axis[1]],
                        [axis[2], 0, -axis[0]],
                        [-axis[1], axis[0], 0]], dtype=float)
        R3 = np.eye(3) + sin_a * K + (1.0 - cos_a) * (K @ K)

    M = np.eye(4, dtype=float)
    M[:3, :3] = R3 * length
    M[:3,  3] = start
    return M


def _rz4(angle: float) -> np.ndarray:
    """4×4 rotation matrix about Z by angle [rad]."""
    c, s = math.cos(angle), math.sin(angle)
    m = np.eye(4, dtype=float)
    m[0, 0] =  c;  m[0, 1] = -s
    m[1, 0] =  s;  m[1, 1] =  c
    return m


def _trail_mesh(positions: np.ndarray) -> Optional[pv.PolyData]:
    if len(positions) < 2:
        return None
    pts = positions
    lines = np.zeros((len(pts)-1, 3), dtype=int)
    for i in range(len(pts)-1):
        lines[i] = [2, i, i+1]
    mesh = pv.PolyData()
    mesh.points = pts
    mesh.lines  = lines.ravel()
    return mesh


# ---------------------------------------------------------------------------
# Swashplate inset helpers
# ---------------------------------------------------------------------------

_SWASH_R_INNER         = 0.12   # inner radius of display ring  [m]
_SWASH_R_OUTER         = 0.58   # outer radius of display ring  [m]
_SWASH_R_SERVO         = 0.50   # servo attachment radius       [m]
_SWASH_COLL_SCALE      = 4.0    # vertical travel per radian collective
                                 # (±0.1 rad → ±0.4 m, comparable to tilt excursion)
_SWASH_TRACK_HALF      = 0.85   # ± height of the servo travel track [m]
_SWASH_COLL_EQ         = -0.1   # equilibrium collective [rad] — reference ring shown here
# H3-120: three servo attachment points equally spaced at 90°, 210°, 330°
_SWASH_SERVO_ANGLES    = (90.0, 210.0, 330.0)
_SWASH_SERVO_COLORS    = ("#ff5555", "#55ee88", "#5599ff")   # S1 red, S2 green, S3 blue


def _swash_normal(tilt_lon: float, tilt_lat: float) -> tuple:
    """
    Swashplate disk normal from cyclic tilt inputs.
      tilt_lat  — tilts the plate about Y (east side rises/falls)
      tilt_lon  — tilts the plate about X (north side rises/falls)
    Returns a normalised (nx, ny, nz) tuple.
    """
    n = np.array([tilt_lat, tilt_lon, 1.0])
    n /= np.linalg.norm(n)
    return tuple(n.tolist())


def _servo_z(collective_rad: float, tilt_lon: float, tilt_lat: float,
             angle_deg: float) -> float:
    """Z height of a servo attachment point on the tilted swashplate."""
    theta       = np.radians(angle_deg)
    z_coll      = collective_rad * _SWASH_COLL_SCALE
    nx, ny, nz  = _swash_normal(tilt_lon, tilt_lat)
    return float(z_coll - _SWASH_R_SERVO * (nx * np.cos(theta) + ny * np.sin(theta)) / nz)


def _swash_plate_mat4(tilt_lon: float, tilt_lat: float,
                      collective_rad: float) -> np.ndarray:
    """
    4×4 transform for the active swashplate ring.

    Created flat (normal=[0,0,1]) at z=0; this matrix tilts it to the
    requested normal and lifts it to the collective height.
    """
    normal = np.array(_swash_normal(tilt_lon, tilt_lat))
    z      = np.array([0., 0., 1.])
    axis   = np.cross(z, normal)
    sin_a  = float(np.linalg.norm(axis))
    cos_a  = float(np.dot(z, normal))
    if sin_a < 1e-9:
        R3 = np.eye(3) if cos_a > 0 else np.diag([1., -1., -1.]).astype(float)
    else:
        axis /= sin_a
        K  = np.array([[0, -axis[2], axis[1]],
                       [axis[2], 0, -axis[0]],
                       [-axis[1], axis[0], 0]], dtype=float)
        R3 = np.eye(3) + sin_a * K + (1.0 - cos_a) * (K @ K)
    M          = np.eye(4, dtype=float)
    M[:3, :3]  = R3
    M[2,   3]  = collective_rad * _SWASH_COLL_SCALE
    return M


def _swashplate_meshes(collective_rad: float, tilt_lon: float,
                       tilt_lat: float) -> list:
    """
    Return list of (PolyData, color_hex, opacity) representing the swashplate.

    Scene elements
    --------------
    - Neutral reference ring  (flat grey, z=0)
    - Tilted swashplate plate (teal, offset vertically by collective)
    - Central axle bar
    - Three servo tracks (dark cylinders, full travel range)
    - Three servo attachment spheres at current heights (S1=red, S2=green, S3=blue)
    """
    z_coll = collective_rad * _SWASH_COLL_SCALE
    normal = _swash_normal(tilt_lon, tilt_lat)
    parts  = []

    # Equilibrium reference ring — flat, centred on the typical operating collective
    z_eq = _SWASH_COLL_EQ * _SWASH_COLL_SCALE
    ref  = pv.Disc(inner=_SWASH_R_INNER, outer=_SWASH_R_OUTER,
                   normal=(0, 0, 1), c_res=64, r_res=1)
    ref.translate((0.0, 0.0, z_eq), inplace=True)
    parts.append((ref, "#4444aa", 0.40))

    # Tilted active plate
    plate = pv.Disc(inner=_SWASH_R_INNER, outer=_SWASH_R_OUTER,
                    normal=normal, c_res=64, r_res=1)
    plate.translate((0.0, 0.0, z_coll), inplace=True)
    parts.append((plate, "#33ccdd", 0.80))

    # Central axle
    axle = pv.Cylinder(center=(0, 0, 0), direction=(0, 0, 1),
                       radius=0.035, height=_SWASH_TRACK_HALF * 2.5,
                       resolution=10)
    parts.append((axle, "#666677", 0.50))

    for angle_deg, col in zip(_SWASH_SERVO_ANGLES, _SWASH_SERVO_COLORS):
        theta = np.radians(angle_deg)
        sx    = _SWASH_R_SERVO * np.cos(theta)
        sy    = _SWASH_R_SERVO * np.sin(theta)
        zh    = _servo_z(collective_rad, tilt_lon, tilt_lat, angle_deg)

        # Travel track — centred on equilibrium z so operating range sits mid-track
        track = pv.Cylinder(center=(sx, sy, z_eq), direction=(0, 0, 1),
                             radius=0.018, height=_SWASH_TRACK_HALF * 2,
                             resolution=8)
        parts.append((track, "#2a2a44", 0.70))

        # Current attachment sphere
        sph = pv.Sphere(radius=0.068, center=(sx, sy, zh),
                        theta_resolution=14, phi_resolution=14)
        parts.append((sph, col, 1.00))

    return parts


# ---------------------------------------------------------------------------
# Camera helpers
# ---------------------------------------------------------------------------

def fit_camera(
    plotter: pv.Plotter,
    positions: np.ndarray,
    anchor: np.ndarray = ANCHOR,
    margin: float = 0.30,
) -> None:
    """
    Position the camera so the entire trajectory + anchor are in frame.

    Strategy: compute the bounding box of all hub positions plus the anchor,
    find the bounding-box centre and diagonal, then place the camera at a
    south-west elevated angle (shows East extent and altitude) at a distance
    proportional to the diagonal.

    Parameters
    ----------
    positions : (N, 3) array of hub ENU positions across all frames.
    anchor    : anchor ENU position (default [0, 0, 0]).
    margin    : fractional extra distance beyond the bounding diagonal (0.30 = 30 %).
    """
    all_pts = np.vstack([positions, anchor[None, :]])
    mn  = all_pts.min(axis=0)
    mx  = all_pts.max(axis=0)
    ctr = (mn + mx) * 0.5
    diag = float(np.linalg.norm(mx - mn))

    # Camera offset direction: south-west and elevated
    #   x = +1.0 (east), y = -1.2 (south), z = +0.7 (up)
    #   → views from behind/below the typical east-of-anchor trajectory
    cam_dir = np.array([0.6, -1.0, 0.55])
    cam_dir /= np.linalg.norm(cam_dir)

    dist = diag * (1.0 + margin)
    plotter.camera.focal_point = tuple(ctr)
    plotter.camera.position    = tuple(ctr + cam_dir * dist)
    plotter.camera.up          = (0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# Visualizer
# ---------------------------------------------------------------------------

class RAWESVisualizer:
    """PyVista-based 3D renderer for RAWES simulation telemetry."""

    def __init__(
        self,
        frames: List[TelemetryFrame],
        trail_len: int = TRAIL_LEN,
        playback_fps: float = 10.0,
    ) -> None:
        self._frames      = frames
        self._trail_len   = trail_len
        self._fps         = playback_fps
        self._n           = len(frames)
        self._pos_history = np.array([_to_viz(f.pos_ned) for f in frames])
        self._spin_angles = self._integrate_spin()
        self._energy      = self._integrate_energy()
        self._inset_ren: Optional[object] = None   # vtkRenderer, set up lazily

    # ------------------------------------------------------------------
    # Public API

    def play(self) -> None:
        """
        Interactive window driven by a wall-clock while loop.

        Uses ``show(interactive_update=True)`` so PyVista does not block,
        then advances frames based on elapsed wall time.  This avoids the
        unreliable PyVista timer and delivers consistent frame rates.
        """
        plotter = pv.Plotter(title="RAWES 3D — interactive playback")
        plotter.set_background("#1a1a2e")
        self._add_static_scene(plotter)
        self._add_dynamic_actors(plotter, 0)
        self._add_hud(plotter)

        idx     = [0]
        playing = [False]       # start paused so user can orient first
        speed   = [1.0]
        wall_t0 = [time.monotonic()]
        sim_t0  = [self._frames[0].t]

        def _reanchor() -> None:
            """Re-anchor wall↔sim clocks at the current position."""
            wall_t0[0] = time.monotonic()
            sim_t0[0]  = self._frames[idx[0]].t

        def _go(i: int) -> None:
            idx[0] = max(0, min(self._n - 1, i))
            self._update_dynamic(plotter, idx[0])
            _reanchor()
            _update_status()

        def _update_status() -> None:
            state = "PLAYING" if playing[0] else "PAUSED "
            spd   = f" {speed[0]:.3g}x" if speed[0] != 1.0 else ""
            _stat_actor.SetInput(f"[Space] {state}{spd}")
            col = pv.Color("#00ff88" if playing[0] else "#ff8800").float_rgb
            _stat_actor.GetTextProperty().SetColor(*col)

        def toggle_play() -> None:
            playing[0] = not playing[0]
            _reanchor()
            _update_status()

        def faster() -> None:
            speed[0] = min(speed[0] * 2, 16.0)
            _reanchor()
            _update_status()

        def slower() -> None:
            speed[0] = max(speed[0] / 2, 0.0625)
            _reanchor()
            _update_status()

        plotter.add_key_event("space", toggle_play)
        plotter.add_key_event("Right", lambda: _go(idx[0] + 1))
        plotter.add_key_event("Left",  lambda: _go(idx[0] - 1))
        plotter.add_key_event("equal", faster)
        plotter.add_key_event("minus", slower)

        _fps_actor  = plotter.add_text("FPS --",  position=(10, 40),
                                       font_size=8, color="#aaaaaa")
        _stat_actor = plotter.add_text("PAUSED",  position=(10, 60),
                                       font_size=8, color="#ff8800")
        # Slider registered after _stat_actor exists (PyVista fires cb at init)
        self._add_slider(plotter, on_scrub=_go)
        self._update_hud(plotter, 0)
        _update_status()
        self._set_camera(plotter)

        _FRAME_DT  = 1.0 / 30.0
        _fps_times: list = []

        plotter.show(interactive_update=True, auto_close=False)

        while (plotter.render_window is not None
               and plotter.render_window.GetGenericContext()):
            t_loop = time.monotonic()
            _fps_times.append(t_loop)
            if len(_fps_times) > 30:
                _fps_times.pop(0)
            if len(_fps_times) >= 2:
                actual_fps = (len(_fps_times) - 1) / (_fps_times[-1] - _fps_times[0])
                _fps_actor.SetInput(f"FPS {actual_fps:4.1f}")

            if playing[0]:
                sim_target = sim_t0[0] + (t_loop - wall_t0[0]) * speed[0]

                if sim_target >= self._frames[-1].t:
                    idx[0]    = 0
                    _reanchor()
                    sim_target = sim_t0[0]

                # Find the bracket [i, i+1] containing sim_target
                i = idx[0]
                while i < self._n - 1 and self._frames[i + 1].t <= sim_target:
                    i += 1
                idx[0] = i

                # Interpolate within the bracket for smooth sub-frame motion
                if i < self._n - 1:
                    f1, f2 = self._frames[i], self._frames[i + 1]
                    dt_ = f2.t - f1.t
                    alpha = (sim_target - f1.t) / dt_ if dt_ > 1e-9 else 0.0
                    frame = _lerp_frame(f1, f2, alpha)
                    sa    = self._spin_angles[i] + f1.omega_spin * (sim_target - f1.t)
                else:
                    frame = self._frames[-1]
                    sa    = self._spin_angles[-1]

                self._hud_frame_idx = i
                self._render_frame_obj(frame, sa)
                # Trail update for nearest recorded frame
                self._last_trail_idx = i
                start = max(0, i - self._trail_len)
                pts   = self._pos_history[start : i + 1]
                if len(pts) >= 2:
                    seg = np.array([[2, j, j + 1] for j in range(len(pts) - 1)],
                                   dtype=int).ravel()
                    self._mesh_trail.points = pts
                    self._mesh_trail.lines  = seg

            budget_ms = max(1, int((_FRAME_DT - (time.monotonic() - t_loop)) * 1000))
            plotter.update(budget_ms)

        plotter.close()

    def scrub(self) -> None:
        """
        Interactive scrubber: opens a window with a large slider.
        Drag the slider to step through frames live — no pre-rendering.

        Controls
        --------
        Slider     — drag to any frame
        ← / →      — step one frame
        Space      — toggle auto-play
        + / -      — 2× / 0.5× auto-play speed
        Mouse      — orbit / pan / zoom (standard PyVista)
        """
        pl = pv.Plotter(title="RAWES scrubber")
        pl.set_background("#1a1a2e")
        self._add_static_scene(pl)
        self._add_dynamic_actors(pl, 0)
        self._update_trail(pl, 0)
        self._add_hud(pl)
        self._update_hud(pl, 0)
        self._update_inset(0)

        idx     = [0]
        playing = [False]
        speed   = [1.0]

        def _go(i: int) -> None:
            i = max(0, min(self._n - 1, i))
            idx[0] = i
            self._add_dynamic_actors(pl, i)
            self._update_trail(pl, i)
            self._update_hud(pl, i)
            self._update_inset(i)

        def _slider_cb(value: float) -> None:
            _go(int(round(value)))

        # Wide slider spanning almost the full window width
        pl.add_slider_widget(
            _slider_cb,
            rng=[0, self._n - 1],
            value=0,
            title="Frame",
            pointa=(0.04, 0.06),
            pointb=(0.96, 0.06),
            style="modern",
        )

        pl.add_key_event("space", lambda: playing.__setitem__(0, not playing[0]))
        pl.add_key_event("Right", lambda: _go(idx[0] + 1))
        pl.add_key_event("Left",  lambda: _go(idx[0] - 1))
        pl.add_key_event("equal", lambda: speed.__setitem__(0, min(speed[0] * 2, 16.0)))
        pl.add_key_event("minus", lambda: speed.__setitem__(0, max(speed[0] / 2, 0.0625)))

        base_ms = int(1000 / self._fps)

        def _timer_cb(_step: int) -> None:
            if playing[0]:
                _go((idx[0] + 1) % self._n)

        pl.add_timer_event(max_steps=10**9, duration=base_ms, callback=_timer_cb)

        fit_camera(pl, self._pos_history)
        pl.show()

    def export(self, path: str, fps: float = 10.0,
               spin_substeps: int = 0) -> None:
        """
        Render all frames to a GIF or MP4 (requires ffmpeg for mp4).

        spin_substeps
            Number of video frames rendered per telemetry frame.  Each sub-frame
            advances the spin angle by omega * dt/substeps, keeping the rotor
            below the 4-blade Nyquist limit (45°/frame).
            0 = auto-compute the minimum to avoid backwards-aliasing (default).
        """
        # ── Auto-compute substeps to avoid stroboscopic aliasing ──────────────
        # With n_blades=4, the blade pattern repeats every 90°.  To see forward
        # rotation, each video frame must advance < 45° (Nyquist = period/2).
        if spin_substeps == 0:
            omega_max  = max((f.omega_spin for f in self._frames), default=30.0)
            dt_tel     = ((self._frames[-1].t - self._frames[0].t)
                          / max(1, len(self._frames) - 1))
            nyquist    = math.pi / N_BLADES          # 45° for 4 blades
            spin_substeps = max(1, math.ceil(omega_max * dt_tel / nyquist))

        export_fps = fps * spin_substeps

        ext = Path(path).suffix.lower()
        plotter = pv.Plotter(off_screen=True, title="RAWES 3D export")
        plotter.set_background("#1a1a2e")
        self._add_static_scene(plotter)
        self._add_dynamic_actors(plotter, 0)
        self._add_hud(plotter)
        self._set_camera(plotter)

        if ext == ".gif":
            plotter.open_gif(path, fps=export_fps)
        elif ext in (".mp4", ".avi"):
            plotter.open_movie(path, framerate=export_fps)
        else:
            raise ValueError(f"Unsupported export format: {ext}")

        n_video = 0
        for i in range(self._n - 1):
            f1 = self._frames[i]
            f2 = self._frames[i + 1]
            sa1   = self._spin_angles[i]
            dt_tel = f2.t - f1.t
            for k in range(spin_substeps):
                alpha    = k / spin_substeps
                f_interp = _lerp_frame(f1, f2, alpha)
                sa_interp = sa1 + f1.omega_spin * alpha * dt_tel
                self._hud_frame_idx = i
                self._render_frame_obj(f_interp, sa_interp)
                plotter.render()
                plotter.write_frame()
                n_video += 1

        # Final telemetry frame
        self._hud_frame_idx = self._n - 1
        self._render_frame_obj(self._frames[-1], self._spin_angles[-1])
        plotter.render()
        plotter.write_frame()
        n_video += 1

        plotter.close()
        print(f"Exported: {path}  "
              f"({n_video} frames @ {export_fps:.0f} fps, "
              f"{spin_substeps} spin substep{'s' if spin_substeps != 1 else ''})")

    # ------------------------------------------------------------------
    # Scene construction

    def _add_static_scene(self, plotter: pv.Plotter) -> None:
        # Solid ground fill
        ground = pv.Plane(center=[0, 0, -0.1],
                          direction=[0, 0, 1],
                          i_size=400, j_size=400)
        plotter.add_mesh(ground, color=COL_GROUND, opacity=0.35)

        # 10 m grid lines on the ground plane
        grid_mesh = _ground_grid(extent=200, spacing=10)
        plotter.add_mesh(grid_mesh, color="#3a5c3a", opacity=0.25,
                         line_width=1)

        anchor = pv.Sphere(radius=0.6, center=ANCHOR)
        plotter.add_mesh(anchor, color=COL_ANCHOR)

        human = _human_figure(x=3.0, y=2.0)
        plotter.add_mesh(human, color="#f5c99a", smooth_shading=True)

        # ── Trees: random positions, outside 60 m of anchor ─────────────────
        # All trunks merged → 1 actor; lower foliage merged → 1 actor;
        # upper foliage merged → 1 actor.  Total: 3 actors regardless of count.
        rng = np.random.default_rng(42)
        trunks, lower_cones, upper_cones = [], [], []
        n_trees, placed = 40, 0
        while placed < n_trees:
            tx = rng.uniform(-180, 180)
            ty = rng.uniform(-180, 180)
            if tx*tx + ty*ty < 60.0**2:
                continue
            th      = rng.uniform(4.5, 8.0)
            trunk_h = th * 0.30
            trunks.append(
                pv.Cylinder(center=(tx, ty, trunk_h / 2), direction=(0,0,1),
                            radius=th * 0.04, height=trunk_h, resolution=8))
            lower_cones.append(
                pv.Cone(center=(tx, ty, trunk_h + th*0.28), direction=(0,0,1),
                        height=th*0.55, radius=th*0.18, resolution=12))
            upper_cones.append(
                pv.Cone(center=(tx, ty, trunk_h + th*0.50), direction=(0,0,1),
                        height=th*0.42, radius=th*0.135, resolution=12))
            placed += 1
        plotter.add_mesh(pv.merge(trunks),      color="#5a3a1a", smooth_shading=True)
        plotter.add_mesh(pv.merge(lower_cones), color="#2d6e2d", smooth_shading=True, opacity=0.92)
        plotter.add_mesh(pv.merge(upper_cones), color="#3a8a3a", smooth_shading=True, opacity=0.92)

        self._setup_inset(plotter)

    # ------------------------------------------------------------------
    # Dynamic actor setup: ALL actors created ONCE; updated in-place per frame.
    # No new meshes, no new shaders after initial setup.

    _N_TETHER_PTS = 40   # fixed catenary sample count

    def _add_dynamic_actors(self, plotter: pv.Plotter, idx: int) -> None:
        """
        Create all dynamic actors exactly ONCE.  Subsequent calls just call
        _fast_update (guard on _actors_created).
        """
        if getattr(self, "_actors_created", False):
            self._fast_update(idx)
            return

        f       = self._frames[idx]
        pos_viz = _to_viz(f.pos_ned)
        R_viz   = _T_NED_ENU @ f.R
        bz_viz  = R_viz[:, 2]
        T       = _hub_to_world(R_viz, pos_viz)
        sa      = self._spin_angles[idx]

        # ── Rotor disk — local frame; spin via user_matrix ────────────────────
        self._mesh_disk = pv.Disc(inner=R_ROOT, outer=R_TIP,
                                  normal=[0, 0, 1], r_res=1, c_res=64)
        self._actor_disk = plotter.add_mesh(self._mesh_disk, color=COL_DISK,
                                            opacity=0.45, smooth_shading=True)
        self._actor_disk.user_matrix = T @ _rz4(sa)

        # ── Blades — single actor, per-cell colour per blade ─────────────────
        # One merged mesh keeps the actor/shader count identical to before.
        # Cell scalars (blade index 0-3) are mapped to COL_BLADES via a
        # ListedColormap so each blade gets a distinct colour.
        blade_meshes = [_blade_mesh_local(k, 0.0) for k in range(N_BLADES)]
        self._mesh_blades = pv.merge(blade_meshes)
        _N_FACES_PER_BLADE = blade_meshes[0].n_cells          # 6 quads per blade
        self._mesh_blades.cell_data["blade_id"] = np.repeat(
            np.arange(N_BLADES, dtype=float), _N_FACES_PER_BLADE)
        from matplotlib.colors import ListedColormap as _LCM
        _blade_cmap = _LCM(COL_BLADES[:N_BLADES])
        self._actor_blades = plotter.add_mesh(
            self._mesh_blades, scalars="blade_id", cmap=_blade_cmap,
            clim=[0.0, float(N_BLADES - 1)], show_scalar_bar=False,
            smooth_shading=True)
        self._actor_blades.user_matrix = T @ _rz4(sa)

        # ── Hub sphere ────────────────────────────────────────────────────────
        self._mesh_hub = pv.Sphere(radius=0.2, center=[0, 0, 0])
        self._actor_hub = plotter.add_mesh(self._mesh_hub, color=COL_HUB)
        Mh = np.eye(4, dtype=float); Mh[:3, 3] = pos_viz
        self._actor_hub.user_matrix = Mh

        # ── Tether — PolyData line with fixed N points; update .points ────────
        tether_attach = pos_viz - TETHER_AXLE_OFFSET * bz_viz
        tpts = _tether_catenary(ANCHOR, tether_attach, f.tether_tension,
                                n_pts=self._N_TETHER_PTS)
        self._mesh_tether = pv.PolyData()
        self._mesh_tether.points = tpts
        seg = np.array([[2, i, i + 1] for i in range(self._N_TETHER_PTS - 1)],
                       dtype=int).ravel()
        self._mesh_tether.lines = seg
        self._actor_tether = plotter.add_mesh(self._mesh_tether,
                                              color=_tether_colour(f.tether_tension),
                                              line_width=3, opacity=0.9)

        # ── Drop line — PolyData points; update .points ───────────────────────
        self._mesh_drop = pv.PolyData(np.zeros((1, 3), dtype=float))
        self._actor_drop = plotter.add_mesh(self._mesh_drop,
                                            color="#ffffff", opacity=0.5,
                                            render_points_as_spheres=True,
                                            point_size=6)

        # ── Arrows — canonical unit +Z mesh; direction via user_matrix ────────
        _arrow_kw = dict(tip_length=0.25, tip_radius=0.08, shaft_radius=0.03)
        self._mesh_body_z   = pv.Arrow(start=[0,0,0], direction=[0,0,1],
                                       scale=1.0, **_arrow_kw)
        self._actor_body_z  = plotter.add_mesh(self._mesh_body_z, color=COL_BODY_Z)
        self._actor_body_z.user_matrix = _dir_mat4(pos_viz, bz_viz * 4.0)

        self._mesh_body_zeq  = pv.Arrow(start=[0,0,0], direction=[0,0,1],
                                        scale=1.0, **_arrow_kw)
        self._actor_body_zeq = plotter.add_mesh(self._mesh_body_zeq, color=COL_BODY_ZEQ)
        bzeq_viz = (_to_viz(f.body_z_eq) if f.body_z_eq is not None else bz_viz)
        self._actor_body_zeq.user_matrix = _dir_mat4(pos_viz, bzeq_viz * 4.0)

        self._mesh_wind  = pv.Arrow(start=[0,0,0], direction=[0,0,1],
                                    scale=1.0, **_arrow_kw)
        self._actor_wind = plotter.add_mesh(self._mesh_wind, color=COL_WIND)
        wind_viz   = _to_viz(f.wind_ned)
        wind_start = pos_viz - wind_viz * 0.3
        self._actor_wind.user_matrix = _dir_mat4(wind_start, wind_viz * 0.4)

        # ── Trail — PolyData; points updated in-place ─────────────────────────
        self._mesh_trail = pv.PolyData(self._pos_history[:1].copy())
        self._actor_trail = plotter.add_mesh(self._mesh_trail,
                                             color=COL_TRAIL, line_width=1,
                                             opacity=0.6)

        self._actors_created = True
        self._last_trail_idx = idx

    def _fast_update(self, idx: int) -> None:
        """Update all dynamic actors in-place. No new actors or shaders."""
        f = self._frames[idx]
        self._render_frame_obj(f, self._spin_angles[idx])
        self._last_trail_idx = idx
        self._last_trail_idx = idx
        # Trail: update points in-place
        start = max(0, idx - self._trail_len)
        pts   = self._pos_history[start : idx + 1]
        if len(pts) >= 2:
            seg = np.array([[2, i, i + 1] for i in range(len(pts) - 1)],
                           dtype=int).ravel()
            self._mesh_trail.points = pts
            self._mesh_trail.lines  = seg

    def _render_frame_obj(self, f, spin_angle: float) -> None:
        """
        Update ALL dynamic actors in-place for frame object ``f``.
        No new actors, no new shaders — only user_matrix and .points updates.
        """
        pos_viz = _to_viz(f.pos_ned)
        R_viz   = _T_NED_ENU @ f.R
        bz_viz  = R_viz[:, 2]
        T       = _hub_to_world(R_viz, pos_viz)
        sa      = spin_angle

        # Rotor + blades: spin × hub transform
        Tsa = T @ _rz4(sa)
        self._actor_disk.user_matrix   = Tsa
        self._actor_blades.user_matrix = Tsa

        # Hub sphere: translate only
        Mh = np.eye(4, dtype=float); Mh[:3, 3] = pos_viz
        self._actor_hub.user_matrix = Mh

        # Tether: update points in-place (fixed N, topology unchanged)
        tether_attach = pos_viz - TETHER_AXLE_OFFSET * bz_viz
        tpts = _tether_catenary(ANCHOR, tether_attach, f.tether_tension,
                                n_pts=self._N_TETHER_PTS)
        self._mesh_tether.points = tpts
        # Update tether colour via actor property
        col = pv.Color(_tether_colour(f.tether_tension)).float_rgb
        self._actor_tether.GetProperty().SetColor(*col)

        # Drop line: update points in-place
        drop = _drop_line(tether_attach)
        self._mesh_drop.points = drop.points if drop.n_points else np.zeros((1, 3))

        # Arrows: direction via user_matrix
        self._actor_body_z.user_matrix = _dir_mat4(pos_viz, bz_viz * 4.0)
        bzeq_viz = (_to_viz(f.body_z_eq) if f.body_z_eq is not None else bz_viz)
        self._actor_body_zeq.user_matrix = _dir_mat4(pos_viz, bzeq_viz * 4.0)
        wind_viz   = _to_viz(f.wind_ned)
        wind_start = pos_viz - wind_viz * 0.3
        self._actor_wind.user_matrix = _dir_mat4(wind_start, wind_viz * 0.4)

        # HUD text: SetInput on the underlying vtkTextActor (no new actor)
        self._update_inset_frame(f)
        if hasattr(self, "_hud_actor") and hasattr(self, "_hud_frame_idx"):
            fi = self._hud_frame_idx
            e_net = self._energy[fi]
            lines = [
                f"t          {f.t:7.2f} s",
                f"altitude   {f.altitude:7.2f} m",
                f"spin       {f.omega_spin:7.2f} rad/s",
                f"tension    {f.tether_tension:7.1f} N",
                f"rest_len   {f.tether_rest_length:7.2f} m",
                f"coll       {np.degrees(f.swash_collective):7.2f} deg",
                f"tilt_lon   {f.swash_tilt_lon:7.4f}",
                f"tilt_lat   {f.swash_tilt_lat:7.4f}",
                f"",
                f"E_net    {e_net:+8.1f} J",
            ]
            self._hud_actor.SetInput("\n".join(lines))

    def _update_dynamic(self, plotter: pv.Plotter, idx: int) -> None:
        self._last_trail_idx = idx
        self._add_dynamic_actors(plotter, idx)
        self._update_trail(plotter, idx)
        self._update_hud(plotter, idx)
        self._update_inset(idx)

    def _update_dynamic_inplace(self, idx: int) -> None:
        """Fast in-place update used by the play loop — no plotter arg needed."""
        self._fast_update(idx)
        self._update_inset(idx)

    def _update_trail(self, plotter: pv.Plotter, idx: int) -> None:
        if not getattr(self, "_actors_created", False):
            return
        start = max(0, idx - self._trail_len)
        pts   = self._pos_history[start : idx + 1]
        if len(pts) >= 2:
            seg = np.array([[2, i, i + 1] for i in range(len(pts) - 1)],
                           dtype=int).ravel()
            self._mesh_trail.points = pts
            self._mesh_trail.lines  = seg

    def _add_hud(self, plotter: pv.Plotter) -> None:
        # Use pixel coordinates → returns vtkTextActor (has SetInput).
        # "upper_left" returns CornerAnnotation which lacks SetInput.
        w, h = plotter.window_size
        self._hud_actor = plotter.add_text(
            " ", position=(10, h - 170),
            font_size=9, color="white", shadow=True,
        )
        plotter.add_text(
            "[Space] play/pause   [</>] step   [+/-] speed   slider: scrub",
            position=(10, 10), font_size=7, color="#888888",
        )

    def _update_hud(self, plotter: pv.Plotter, idx: int) -> None:
        self._hud_frame_idx = idx
        f = self._frames[idx]
        e_net = self._energy[idx]
        lines = [
            f"t          {f.t:7.2f} s",
            f"altitude   {f.altitude:7.2f} m",
            f"spin       {f.omega_spin:7.2f} rad/s",
            f"tension    {f.tether_tension:7.1f} N",
            f"rest_len   {f.tether_rest_length:7.2f} m",
            f"coll       {np.degrees(f.swash_collective):7.2f} deg",
            f"tilt_lon   {f.swash_tilt_lon:7.4f}",
            f"tilt_lat   {f.swash_tilt_lat:7.4f}",
            f"frame      {idx+1}/{self._n}",
            f"",
            f"E_net    {e_net:+8.1f} J",
        ]
        if hasattr(self, "_hud_actor"):
            self._hud_actor.SetInput("\n".join(lines))

    def _add_slider(self, plotter: pv.Plotter, on_scrub=None) -> None:
        """
        Add a frame scrub slider.  ``on_scrub``, if provided, is called with
        the integer frame index whenever the slider moves (allows play() to
        sync its idx[0] closure).
        """
        def cb(value: float) -> None:
            i = int(round(value))
            if on_scrub is not None:
                on_scrub(i)
            else:
                self._update_dynamic(plotter, i)

        plotter.add_slider_widget(
            cb,
            rng=[0, self._n - 1],
            value=0,
            title="Frame",
            pointa=(0.10, 0.05),
            pointb=(0.90, 0.05),
            style="modern",
        )

    # ------------------------------------------------------------------
    # Swashplate inset (bottom-right corner — collective + cyclic diagram)

    def _setup_inset(self, plotter: pv.Plotter) -> None:
        """
        Create all swashplate inset actors ONCE.  Per-frame updates use
        user_matrix only — no new actors or shaders after this call.

        Static actors  : reference ring, axle, 3 servo tracks
        Dynamic actors : active plate (user_matrix), 3 servo spheres (user_matrix)
        Total          : 9 actors created once at startup.
        """
        if not _HAS_VTK:
            return
        ren = vtkRenderer()
        ren.SetViewport(0.72, 0.02, 0.98, 0.30)
        ren.SetBackground(0.05, 0.05, 0.12)
        ren.SetLayer(1)
        plotter.ren_win.SetNumberOfLayers(2)
        plotter.ren_win.AddRenderer(ren)

        cam = ren.GetActiveCamera()
        cam.SetPosition(1.6, -2.0, 2.2)
        cam.SetFocalPoint(0.0, 0.0, 0.0)
        cam.SetViewUp(0.0, 0.0, 1.0)
        cam.SetParallelProjection(True)
        cam.SetParallelScale(1.05)

        self._inset_ren = ren

        def _add(mesh, color, opacity=1.0):
            return self._inset_add(mesh, color, opacity)

        # ── Static: all merged into ONE actor to minimise shader count ──────────
        z_eq         = _SWASH_COLL_EQ * _SWASH_COLL_SCALE
        ref          = pv.Disc(inner=_SWASH_R_INNER, outer=_SWASH_R_OUTER,
                               normal=(0,0,1), c_res=48, r_res=1)
        ref.translate((0.0, 0.0, z_eq), inplace=True)
        static_parts = [ref,
                        pv.Cylinder(center=(0,0,0), direction=(0,0,1),
                                    radius=0.035, height=_SWASH_TRACK_HALF*2.5,
                                    resolution=8)]
        for angle_deg in _SWASH_SERVO_ANGLES:
            theta = np.radians(angle_deg)
            sx, sy = _SWASH_R_SERVO*np.cos(theta), _SWASH_R_SERVO*np.sin(theta)
            static_parts.append(
                pv.Cylinder(center=(sx, sy, z_eq), direction=(0,0,1),
                            radius=0.018, height=_SWASH_TRACK_HALF*2,
                            resolution=6))
        _add(pv.merge(static_parts), "#4a4a77", 0.55)   # 1 actor for all static parts

        # ── Dynamic: active plate — user_matrix updated per frame ──────────────
        plate_mesh = pv.Disc(inner=_SWASH_R_INNER, outer=_SWASH_R_OUTER,
                             normal=(0,0,1), c_res=64, r_res=1)
        self._inset_plate = _add(plate_mesh, "#33ccdd", 0.80)

        # ── Dynamic: servo spheres — Z translated per frame ────────────────────
        self._inset_spheres = []
        for angle_deg, col in zip(_SWASH_SERVO_ANGLES, _SWASH_SERVO_COLORS):
            theta = np.radians(angle_deg)
            sx, sy = _SWASH_R_SERVO*np.cos(theta), _SWASH_R_SERVO*np.sin(theta)
            sph = pv.Sphere(radius=0.068, center=(sx, sy, 0.0),
                            theta_resolution=10, phi_resolution=10)
            self._inset_spheres.append(_add(sph, col, 1.00))

    def _update_inset(self, idx: int) -> None:
        """Update swashplate inset via user_matrix — no new actors created."""
        if self._inset_ren is None or not hasattr(self, "_inset_plate"):
            return
        f = self._frames[idx]
        self._update_inset_frame(f)

    @staticmethod
    def _np_to_vtk4x4(m: np.ndarray):
        """Convert 4×4 numpy array to vtkMatrix4x4 for raw vtkActor.SetUserMatrix."""
        from vtkmodules.vtkCommonMath import vtkMatrix4x4
        vtk_m = vtkMatrix4x4()
        for i in range(4):
            for j in range(4):
                vtk_m.SetElement(i, j, float(m[i, j]))
        return vtk_m

    def _update_inset_frame(self, f) -> None:
        """Update inset actors in-place — no new actors or shaders."""
        if self._inset_ren is None or not hasattr(self, "_inset_plate"):
            return
        # Active plate: rotation + Z lift via vtkMatrix4x4
        self._inset_plate.SetUserMatrix(
            self._np_to_vtk4x4(_swash_plate_mat4(
                f.swash_tilt_lon, f.swash_tilt_lat, f.swash_collective)))
        # Servo spheres: Z translation via SetPosition (sphere local origin = sx,sy,0)
        for actor, angle_deg in zip(self._inset_spheres, _SWASH_SERVO_ANGLES):
            zh = _servo_z(f.swash_collective, f.swash_tilt_lon,
                          f.swash_tilt_lat, angle_deg)
            actor.SetPosition(0.0, 0.0, zh)
        self._inset_ren.ResetCameraClippingRange()

    def _inset_add(self, mesh: pv.PolyData, color: str,
                   opacity: float = 1.0) -> "vtkActor":
        """Add a mesh to the inset renderer; return the actor."""
        mapper = vtkPolyDataMapper()
        mapper.SetInputData(mesh)
        actor = vtkActor()
        actor.SetMapper(mapper)
        c = pv.Color(color).float_rgb
        actor.GetProperty().SetColor(*c)
        actor.GetProperty().SetOpacity(opacity)
        self._inset_ren.AddActor(actor)
        return actor

    def _set_camera(self, plotter: pv.Plotter) -> None:
        fit_camera(plotter, self._pos_history)

    # ------------------------------------------------------------------
    # Spin integration

    def _integrate_spin(self) -> np.ndarray:
        """Cumulative spin angle [rad] for each frame."""
        angles = np.zeros(self._n)
        for i in range(1, self._n):
            dt = self._frames[i].t - self._frames[i-1].t
            angles[i] = angles[i-1] + self._frames[i].omega_spin * dt
        return angles

    def _integrate_energy(self) -> np.ndarray:
        """
        Running net winch energy [J] at each frame.

        dE = tension × d(rest_length)
          positive (rest_length grows) → reel-out, energy generated
          negative (rest_length shrinks) → reel-in, energy consumed
        Net = cumulative sum; positive = net power generated so far.
        """
        energy = np.zeros(self._n)
        for i in range(1, self._n):
            dl = (self._frames[i].tether_rest_length
                  - self._frames[i-1].tether_rest_length)
            t_avg = (self._frames[i].tether_tension
                     + self._frames[i-1].tether_tension) * 0.5
            energy[i] = energy[i-1] + t_avg * dl
        return energy


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="RAWES 3D telemetry visualizer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("path", help="Telemetry file (.json or .csv)")
    p.add_argument("--source", choices=["json", "csv"], default="json",
                   help="File format (default: json)")
    p.add_argument("--export", metavar="FILE",
                   help="Export to GIF or MP4 instead of interactive window")
    p.add_argument("--fps", type=float, default=10.0,
                   help="Playback / export frame rate (default: 10)")
    p.add_argument("--spin-substeps", type=int, default=0, metavar="N",
                   help="Video frames per telemetry frame for smooth rotor spin "
                        "(0 = auto, default)")
    p.add_argument("--trail", type=int, default=TRAIL_LEN,
                   help=f"Trajectory trail length in frames (default: {TRAIL_LEN})")
    return p


def main(argv: Optional[list] = None) -> None:
    args = _build_parser().parse_args(argv)

    path = Path(args.path)
    if not path.exists():
        print(f"File not found: {path}")
        sys.exit(1)

    if args.source == "csv":
        source = CSVSource(path)
    else:
        source = JSONSource(path)

    frames = list(source.frames())
    if not frames:
        print("No frames loaded — check file format.")
        sys.exit(1)

    print(f"Loaded {len(frames)} frames from {path}")
    print(f"  t_start={frames[0].t:.2f}s  t_end={frames[-1].t:.2f}s  "
          f"fps={len(frames)/(frames[-1].t - frames[0].t + 1e-9):.1f}")

    viz = RAWESVisualizer(frames, trail_len=args.trail, playback_fps=args.fps)

    if args.export:
        viz.export(args.export, fps=args.fps,
                   spin_substeps=args.spin_substeps)
    else:
        viz.play()


if __name__ == "__main__":
    main()
