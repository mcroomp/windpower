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
COL_BLADE    = "#ffffff"   # white
COL_TETHER   = "#ff8800"   # orange
COL_ANCHOR   = "#ff3300"   # red
COL_BODY_Z   = "#00ff44"   # green  — current axle
COL_BODY_ZEQ = "#ffee00"   # yellow — equilibrium target
COL_WIND     = "#88ccff"   # light blue
COL_TRAIL    = "#888888"   # grey
COL_GROUND   = "#2a4a2a"   # dark green

TRAIL_LEN = 200  # frames to keep in trajectory trail

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


def _drop_line(pos_enu: np.ndarray, dot_spacing: float = 1.5) -> pv.PolyData:
    """Dotted vertical plumb line from hub straight down to z=0."""
    x, y, z = pos_enu
    if z <= 0:
        return pv.PolyData()
    n = max(1, int(z / dot_spacing) + 1)
    pts = np.array([
        [x, y, z - i * dot_spacing]
        for i in range(n)
        if z - i * dot_spacing >= 0
    ])
    return pv.PolyData(pts)


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
        self._pos_history = np.array([f.pos_enu for f in frames])
        self._spin_angles = self._integrate_spin()
        self._inset_ren: Optional[object]  = None   # vtkRenderer, set up lazily
        self._inset_actors: list           = []

    # ------------------------------------------------------------------
    # Public API

    def play(self) -> None:
        """Interactive window: slider, play/pause, keyboard step."""
        plotter = pv.Plotter(title="RAWES 3D — interactive playback")
        plotter.set_background("#1a1a2e")
        self._add_static_scene(plotter)
        self._add_dynamic_actors(plotter, 0)
        self._add_slider(plotter)
        self._add_hud(plotter)

        idx    = [0]
        playing = [True]
        speed   = [1.0]

        def step(delta: int) -> None:
            idx[0] = max(0, min(self._n - 1, idx[0] + delta))
            self._update_dynamic(plotter, idx[0])

        def toggle_play() -> None:
            playing[0] = not playing[0]

        def faster() -> None:
            speed[0] = min(speed[0] * 2, 16.0)

        def slower() -> None:
            speed[0] = max(speed[0] / 2, 0.0625)

        plotter.add_key_event("space",  toggle_play)
        plotter.add_key_event("Right",  lambda: step(1))
        plotter.add_key_event("Left",   lambda: step(-1))
        plotter.add_key_event("equal",  faster)
        plotter.add_key_event("minus",  slower)

        interval_ms = int(1000 / self._fps)

        def timer_cb(_step: int) -> None:
            if playing[0]:
                idx[0] = (idx[0] + 1) % self._n
                self._update_dynamic(plotter, idx[0])

        plotter.add_timer_event(
            max_steps=10**9,
            duration=interval_ms,
            callback=timer_cb,
        )
        self._set_camera(plotter)
        plotter.show()

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

    def export(self, path: str, fps: float = 10.0) -> None:
        """Render all frames to a GIF or MP4 (requires ffmpeg for mp4)."""
        ext = Path(path).suffix.lower()
        plotter = pv.Plotter(off_screen=True, title="RAWES 3D export")
        plotter.set_background("#1a1a2e")
        self._add_static_scene(plotter)
        self._add_dynamic_actors(plotter, 0)
        self._add_hud(plotter)
        self._set_camera(plotter)

        if ext == ".gif":
            plotter.open_gif(path, fps=fps)
        elif ext in (".mp4", ".avi"):
            plotter.open_movie(path, framerate=fps)
        else:
            raise ValueError(f"Unsupported export format: {ext}")

        for i in range(self._n):
            self._update_dynamic(plotter, i)
            plotter.render()
            plotter.write_frame()

        plotter.close()
        print(f"Exported: {path}  ({self._n} frames @ {fps} fps)")

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

        plotter.add_axes(
            line_width=2,
            x_color="red", y_color="green", z_color="blue",
            xlabel="East", ylabel="North", zlabel="Up",
        )

        self._setup_inset(plotter)

    def _add_dynamic_actors(self, plotter: pv.Plotter, idx: int) -> None:
        f = self._frames[idx]
        T = _hub_to_world(f.R, f.pos_enu)

        rotor = _rotor_local(self._spin_angles[idx])
        rotor.transform(T, inplace=True)
        plotter.add_mesh(rotor, color=COL_DISK, opacity=0.45,
                         smooth_shading=True, name="rotor")

        blades_only = pv.merge([
            _blade_mesh_local(k, self._spin_angles[idx]).transform(T, inplace=False)
            for k in range(N_BLADES)
        ])
        plotter.add_mesh(blades_only, color=COL_BLADE,
                         smooth_shading=True, name="blades")

        hub = pv.Sphere(radius=0.2, center=f.pos_enu)
        plotter.add_mesh(hub, color=COL_HUB, name="hub")

        # Tether attachment: bottom of axle, TETHER_AXLE_OFFSET below CM along -body_z
        tether_attach = f.pos_enu - TETHER_AXLE_OFFSET * f.body_z

        drop = _drop_line(tether_attach)
        if drop.n_points:
            plotter.add_mesh(drop, color="#ffffff", opacity=0.5,
                             render_points_as_spheres=True, point_size=6,
                             name="drop_line")

        # Tether attaches at the bottom of the axle (-body_z from CM)
        tether_attach = f.pos_enu - TETHER_AXLE_OFFSET * f.body_z
        tether_pts    = _tether_catenary(ANCHOR, tether_attach, f.tether_tension)
        tether_spline = pv.Spline(tether_pts, n_points=len(tether_pts))
        tether_tube   = tether_spline.tube(radius=TETHER_TUBE_RADIUS)
        plotter.add_mesh(tether_tube, color=_tether_colour(f.tether_tension),
                         smooth_shading=True, name="tether")

        axle = _arrow(f.pos_enu, f.body_z * 4.0)
        if axle.n_points:
            plotter.add_mesh(axle, color=COL_BODY_Z, name="body_z")

        if f.body_z_eq is not None:
            eq = _arrow(f.pos_enu, f.body_z_eq * 4.0)
            if eq.n_points:
                plotter.add_mesh(eq, color=COL_BODY_ZEQ, name="body_z_eq")

        wind = _arrow(f.pos_enu - f.wind_enu * 0.3, f.wind_enu, scale=0.4)
        if wind.n_points:
            plotter.add_mesh(wind, color=COL_WIND, name="wind")

    def _update_dynamic(self, plotter: pv.Plotter, idx: int) -> None:
        self._add_dynamic_actors(plotter, idx)
        self._update_trail(plotter, idx)
        self._update_hud(plotter, idx)
        self._update_inset(idx)

    def _update_trail(self, plotter: pv.Plotter, idx: int) -> None:
        start = max(0, idx - self._trail_len)
        trail = _trail_mesh(self._pos_history[start : idx + 1])
        if trail is not None:
            plotter.add_mesh(trail, color=COL_TRAIL, line_width=1,
                             opacity=0.6, name="trail")

    def _add_hud(self, plotter: pv.Plotter) -> None:
        plotter.add_text("", position="upper_left",
                         font_size=9, color="white",
                         name="hud", shadow=True)
        plotter.add_text(
            "[Space] play/pause   [←/→] step   [+/-] speed   slider: scrub",
            position="lower_right", font_size=7, color="#888888",
        )

    def _update_hud(self, plotter: pv.Plotter, idx: int) -> None:
        f = self._frames[idx]
        lines = [
            f"t          {f.t:7.2f} s",
            f"altitude   {f.altitude:7.2f} m",
            f"spin       {f.omega_spin:7.2f} rad/s",
            f"tension    {f.tether_tension:7.1f} N",
            f"rest_len   {f.tether_rest_length:7.2f} m",
            f"coll       {np.degrees(f.swash_collective):7.2f} °",
            f"tilt_lon   {f.swash_tilt_lon:7.4f}",
            f"tilt_lat   {f.swash_tilt_lat:7.4f}",
            f"frame      {idx+1}/{self._n}",
        ]
        plotter.add_text("\n".join(lines), position="upper_left",
                         font_size=9, color="white",
                         name="hud", shadow=True)

    def _add_slider(self, plotter: pv.Plotter) -> None:
        def cb(value: float) -> None:
            idx = int(round(value))
            self._update_dynamic(plotter, idx)

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
        Add a second VTK renderer in the bottom-right corner showing a
        swashplate diagram: collective (plate height), tilt_lon and tilt_lat
        (plate tilt), and three servo attachment spheres on travel tracks.

        S1=red  S2=green  S3=blue  (H3-120 at 90°/210°/330°)
        """
        if not _HAS_VTK:
            return
        ren = vtkRenderer()
        ren.SetViewport(0.72, 0.02, 0.98, 0.30)
        ren.SetBackground(0.05, 0.05, 0.12)
        ren.SetLayer(1)
        plotter.ren_win.SetNumberOfLayers(2)
        plotter.ren_win.AddRenderer(ren)

        # 3/4 elevated camera so tilt and collective are both clearly visible
        cam = ren.GetActiveCamera()
        cam.SetPosition(1.6, -2.0, 2.2)
        cam.SetFocalPoint(0.0, 0.0, 0.0)
        cam.SetViewUp(0.0, 0.0, 1.0)
        cam.SetParallelProjection(True)
        cam.SetParallelScale(1.05)   # fits ~2.1 m vertically (track ±0.85 m)

        self._inset_ren = ren

    def _update_inset(self, idx: int) -> None:
        """Rebuild swashplate inset actors for the current frame."""
        if self._inset_ren is None:
            return

        ren = self._inset_ren
        for actor in self._inset_actors:
            ren.RemoveActor(actor)
        self._inset_actors.clear()

        f = self._frames[idx]
        for mesh, color, opacity in _swashplate_meshes(
                f.swash_collective, f.swash_tilt_lon, f.swash_tilt_lat):
            self._inset_actors.append(self._inset_add(mesh, color, opacity))

        ren.ResetCameraClippingRange()

    def _inset_add(self, mesh: pv.PolyData, color: str,
                   opacity: float = 1.0) -> "vtkActor":
        """Add a pyvista mesh to the inset vtkRenderer; return the actor."""
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
        viz.export(args.export, fps=args.fps)
    else:
        viz.play()


if __name__ == "__main__":
    main()
