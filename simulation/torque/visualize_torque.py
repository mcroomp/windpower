"""
torque/visualize_torque.py — 3D playback of counter-torque motor telemetry.

Scene (ENU, Z up)
------------------
  Hub cylinder        — squat disc that rotates in yaw (ψ)
  Hub top quadrants   — alternating black/white quarters on top face; rotate
                        with hub so yaw rotation is clearly visible
  Axle                — thin dark rod through hub centre, spins at ω_axle
  Rotor indicator     — 4 small blade stubs at top of axle; spin fast to show
                        that the axle is driven at autorotation speed
  Yaw pointer         — amber arrow showing current hub heading direction
  Torque arcs         — orange = bearing drag (CCW), blue = motor (CW)
  Throttle bar        — vertical green bar at left (with equilibrium line)
  Ground disc         — dark translucent ground plane
  Event log panel     — scrolling highlighted events (upper-left)
  HUD                 — right-side state readout

Animation is real-time: playback advances at actual simulation speed
(adjusted by ×speed multiplier).

Controls
--------
  Space       — play / pause
  Left/Right  — step one frame (while paused)
  +/-         — 2× / 0.5× playback speed
  Mouse       — orbit / pan / zoom

Usage
-----
  python simulation/torque/visualize_torque.py simulation/logs/torque_telemetry.json
  python simulation/torque/visualize_torque.py telemetry.json --export out.gif --fps 15
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import List, Optional

import numpy as np

_HERE = Path(__file__).resolve().parent
_SIM  = _HERE.parent
for _p in [str(_HERE), str(_SIM)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import dataclasses as _dc
from torque_telemetry import TorqueTelemetryFrame, TorqueJSONSource
from mediator_torque import PROFILES as _MEDIATOR_PROFILES

try:
    import pyvista as pv
except ImportError:
    print("PyVista not found.  Install with:  pip install pyvista")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Geometry (metres)
# ---------------------------------------------------------------------------

HUB_RADIUS      = 0.150   # hub outer radius
HUB_HEIGHT      = 0.060   # hub axial height

AXLE_RADIUS     = 0.010   # thin axle rod radius
AXLE_TOTAL_H    = 0.50    # total visible axle height (below + above hub)

ROTOR_BLADE_LEN = 0.22    # indicator blade length from centre
ROTOR_BLADE_W   = 0.026   # blade width
ROTOR_Z         = HUB_HEIGHT + AXLE_TOTAL_H * 0.55  # blade height (~mid-axle)

_EQ_THROTTLE    = 0.747   # equilibrium throttle line position
_THRESHOLD      = 1.0     # ψ_dot warning threshold [deg/s]

# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------
C_HUB_SIDE  = (0.40, 0.45, 0.55)   # dark steel for hub side
C_AXLE      = (0.15, 0.15, 0.18)   # near-black axle rod
C_ROTOR     = (0.85, 0.55, 0.10)   # amber blades
C_POINTER   = (0.95, 0.80, 0.20)   # amber yaw arrow
C_DRAG      = (0.90, 0.40, 0.20)   # orange bearing drag arc
C_MOTOR_T   = (0.30, 0.65, 0.90)   # blue motor torque arc
C_GROUND    = (0.15, 0.15, 0.15)   # dark ground
C_THROTTLE  = (0.25, 0.80, 0.45)   # green throttle bar
C_EQ_LINE   = (0.95, 0.95, 0.40)   # yellow equilibrium mark


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _sector_mesh(start_deg: float, end_deg: float,
                 radius: float, z: float,
                 n_arc: int = 20) -> pv.PolyData:
    """Filled pie-slice (triangle fan) in the XY plane at height z."""
    angles = np.linspace(np.radians(start_deg), np.radians(end_deg), n_arc + 1)
    # vertices: centre + arc points
    verts = np.zeros((n_arc + 2, 3), dtype=float)
    verts[0] = [0.0, 0.0, z]
    verts[1:] = np.column_stack([radius * np.cos(angles),
                                  radius * np.sin(angles),
                                  np.full(n_arc + 1, z)])
    # triangle faces: [3, 0, i, i+1]
    faces = np.empty((n_arc, 4), dtype=int)
    faces[:, 0] = 3
    faces[:, 1] = 0
    faces[:, 2] = np.arange(1, n_arc + 1)
    faces[:, 3] = np.arange(2, n_arc + 2)
    return pv.PolyData(verts, faces.ravel())


def _blade_mesh(angle_rad: float,
                length: float = ROTOR_BLADE_LEN,
                width: float  = ROTOR_BLADE_W,
                z: float      = ROTOR_Z) -> pv.PolyData:
    """Thin flat rectangular blade centred on the axle, pointing at angle_rad."""
    ca, sa  = math.cos(angle_rad), math.sin(angle_rad)
    cp, sp  = -sa, ca           # perpendicular direction
    hw      = width / 2
    # 4 corners
    verts = np.array([
        [ hw*cp,           hw*sp,           z],   # root left
        [-hw*cp,          -hw*sp,           z],   # root right
        [length*ca + hw*cp, length*sa + hw*sp, z], # tip left
        [length*ca - hw*cp, length*sa - hw*sp, z], # tip right
    ], dtype=float)
    faces = np.array([4, 0, 1, 3, 2], dtype=int)
    return pv.PolyData(verts, faces)


# ---------------------------------------------------------------------------
# Frame interpolation
# ---------------------------------------------------------------------------

def _lerp_angle(a1: float, a2: float, alpha: float) -> float:
    """Interpolate angle [deg] taking the shortest arc (handles wrap-around)."""
    diff = ((a2 - a1 + 180.0) % 360.0) - 180.0
    return a1 + alpha * diff


def _lerp_frame(f1: TorqueTelemetryFrame,
                f2: TorqueTelemetryFrame,
                alpha: float) -> TorqueTelemetryFrame:
    """Return a frame linearly interpolated between f1 (alpha=0) and f2 (alpha=1)."""
    def L(a, b): return a + alpha * (b - a)
    return TorqueTelemetryFrame(
        t               = L(f1.t, f2.t),
        psi_deg         = _lerp_angle(f1.psi_deg, f2.psi_deg, alpha),
        psi_dot_degs    = L(f1.psi_dot_degs, f2.psi_dot_degs),
        throttle        = L(f1.throttle, f2.throttle),
        omega_axle_rads = f1.omega_axle_rads,          # constant
        q_bearing_nm    = L(f1.q_bearing_nm, f2.q_bearing_nm),
        q_motor_nm      = L(f1.q_motor_nm, f2.q_motor_nm),
        phase           = f1.phase,
    )


def _find_dynamics_start(frames: List[TorqueTelemetryFrame],
                         startup_rate: float = 3.0) -> float:
    """
    Return the simulation time when the hub is first left to its own physics.

    The startup spin shows as |psi_dot| > startup_rate deg/s in early frames.
    We detect the first sustained drop below that level.
    """
    saw_high = False
    for f in frames:
        if abs(f.psi_dot_degs) > startup_rate:
            saw_high = True
        elif saw_high:
            return f.t      # first frame after the startup spin ends
    return frames[0].t      # fallback: start of recording


# ---------------------------------------------------------------------------
# Event log panel
# ---------------------------------------------------------------------------

class EventLog:
    """Scrolling event panel in the upper-left of the viewport."""

    MAX_LINES = 12

    _COLOURS = {"info": "white", "warn": "yellow", "pass": "lime", "fail": "red"}

    def __init__(self, pl: pv.Plotter,
                 settle_s: float, observe_s: float, threshold: float) -> None:
        self.pl          = pl
        self._settle_s   = settle_s
        self._observe_s  = observe_s
        self._threshold  = threshold
        self._events: list[tuple[str, float, str]] = []
        self._prev_phase   = ""
        self._prev_above   = False
        self._settle_done  = False
        self._observe_done = False

    def _push(self, sev: str, t: float, msg: str) -> None:
        self._events.append((sev, t, msg))

    def check(self, f: TorqueTelemetryFrame) -> None:
        pd = abs(f.psi_dot_degs)
        if f.phase != self._prev_phase and f.phase:
            self._push("info", f.t, f"-> {f.phase}")
            self._prev_phase = f.phase
        above = pd > self._threshold
        if above and not self._prev_above:
            self._push("warn", f.t, f"|psi_dot|={pd:.2f} deg/s  EXCEEDS limit")
        elif not above and self._prev_above:
            self._push("pass", f.t, f"|psi_dot|={pd:.2f} deg/s  within limit")
        self._prev_above = above
        if not self._settle_done and f.t >= self._settle_s:
            self._push("info", f.t, f"settle t>={self._settle_s:.0f}s")
            self._settle_done = True
        if not self._observe_done and f.t >= self._settle_s + self._observe_s:
            self._push("info", f.t, "observe window closed")
            self._observe_done = True

    def init_actors(self, win_w: int = 1024, win_h: int = 768) -> None:
        """Create text actors once (pixel positions → vtkTextActor → SetInput)."""
        self._hist_actor = self.pl.add_text(
            " ", position=(10, win_h - 220), font_size=7, font="courier",
            color=(0.50, 0.50, 0.50),
        )
        self._latest_actor = self.pl.add_text(
            " ", position=(10, win_h - 230), font_size=8, font="courier",
            color="white",
        )

    def reset(self, settle_s: float, observe_s: float, threshold: float) -> None:
        """Clear events and state for a new file — keeps existing actors."""
        self._events.clear()
        self._settle_s    = settle_s
        self._observe_s   = observe_s
        self._threshold   = threshold
        self._prev_phase  = ""
        self._prev_above  = False
        self._settle_done = False
        self._observe_done = False
        if hasattr(self, "_hist_actor"):
            self._hist_actor.SetInput(" ")
        if hasattr(self, "_latest_actor"):
            self._latest_actor.SetInput(" ")

    def render(self) -> None:
        if not self._events:
            return
        window = self._events[-self.MAX_LINES:]
        hist   = window[:-1]
        latest = window[-1]

        hist_text = "\n".join(f"[{t:5.1f}] {msg}" for _, t, msg in hist) if hist else " "
        self._hist_actor.SetInput(hist_text)

        sev, t_ev, msg = latest
        self._latest_actor.SetInput(f"[{t_ev:5.1f}] {msg}")
        col = self._COLOURS.get(sev, "white")
        # map colour name → RGB
        _rgb = {"white":(1,1,1), "yellow":(1,1,0), "lime":(0,1,0), "red":(1,0,0)}
        r, g, b = _rgb.get(col, (1,1,1))
        self._latest_actor.GetTextProperty().SetColor(r, g, b)


# ---------------------------------------------------------------------------
# Scene  —  meshes created ONCE; only transforms updated each frame
# ---------------------------------------------------------------------------

def _rz4(angle: float) -> np.ndarray:
    """4×4 rotation matrix around world Z by angle [rad]."""
    c, s = math.cos(angle), math.sin(angle)
    m = np.eye(4, dtype=float)
    m[0, 0] =  c;  m[0, 1] = -s
    m[1, 0] =  s;  m[1, 1] =  c
    return m


def _r_hub4(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    4×4 rotation matrix for the hub: Rz(yaw) @ Ry(pitch) @ Rx(roll).
    All angles in radians.  Matches the ArduPilot ZYX Euler convention.
    """
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    m = np.eye(4, dtype=float)
    m[0, 0] = cy*cp;   m[0, 1] = cy*sp*sr - sy*cr;  m[0, 2] = cy*sp*cr + sy*sr
    m[1, 0] = sy*cp;   m[1, 1] = sy*sp*sr + cy*cr;  m[1, 2] = sy*sp*cr - cy*sr
    m[2, 0] = -sp;     m[2, 1] = cp*sr;              m[2, 2] = cp*cr
    return m


class TorqueScene:
    """
    All scene actors are created once in __init__.
    update() only sets actor.user_matrix — no mesh objects created per frame.
    """

    def __init__(self, pl: pv.Plotter,
                 settle_s: float = 40.0,
                 observe_s: float = 20.0,
                 threshold_degs: float = 1.0) -> None:
        self.pl          = pl
        self._axle_angle = 0.0
        self._prev_t     = 0.0
        self.event_log   = EventLog(pl, settle_s, observe_s, threshold_degs)
        self._build()

    # ── One-time build ────────────────────────────────────────────────────

    def _build(self) -> None:
        pl = self.pl
        z_top = HUB_HEIGHT + 0.001   # hub top face height

        # Ground
        pl.add_mesh(pv.Disc(center=(0,0,0), inner=0, outer=1.2,
                            normal=(0,0,1), r_res=1, c_res=48),
                    color=C_GROUND, opacity=0.55)
        pl.add_mesh(pv.Line([0,0,0.001], [0,0.6,0.001]),
                    color=(0.5,0.5,0.9), line_width=1, opacity=0.4)
        pl.add_axes(line_width=2, x_color="red", y_color="green", z_color="blue")

        # Hub body — cylinder centred at (0,0,HUB_HEIGHT/2); rotates with psi
        self._hub_a = pl.add_mesh(
            pv.Cylinder(center=(0,0,HUB_HEIGHT/2), direction=(0,0,1),
                        radius=HUB_RADIUS, height=HUB_HEIGHT, resolution=48),
            color=C_HUB_SIDE, smooth_shading=True,
        )

        # Hub top quadrants: 4 sectors created at psi=0, same transform as hub
        self._quad_a = []
        for i in range(4):
            color = "white" if i % 2 == 0 else "black"
            self._quad_a.append(pl.add_mesh(
                _sector_mesh(i*90.0, i*90.0+90.0, HUB_RADIUS*0.98, z_top, n_arc=20),
                color=color,
            ))
        # Dividing spokes — 4 radii at 0°,90°,180°,270°
        self._spoke_a = []
        for i in range(4):
            a = math.radians(i * 90.0)
            self._spoke_a.append(pl.add_mesh(
                pv.Line([0,0,z_top+0.0005],
                        [HUB_RADIUS*0.98*math.cos(a),
                         HUB_RADIUS*0.98*math.sin(a),
                         z_top+0.0005]),
                color="black", line_width=2,
            ))

        # Axle — static, no transform
        pl.add_mesh(
            pv.Cylinder(center=(0,0,AXLE_TOTAL_H/2), direction=(0,0,1),
                        radius=AXLE_RADIUS, height=AXLE_TOTAL_H, resolution=12),
            color=C_AXLE,
        )

        # Rotor indicator cap (static)
        pl.add_mesh(
            pv.Disc(center=(0,0,ROTOR_Z), inner=0, outer=AXLE_RADIUS*3,
                    normal=(0,0,1), r_res=1, c_res=16),
            color=(0.35, 0.35, 0.35),
        )
        # 4 blades — created pointing in +X, rotated via user_matrix
        self._blade_a = []
        for i in range(4):
            color = (0.95,0.95,0.95) if i % 2 == 0 else (0.28,0.28,0.28)
            self._blade_a.append(pl.add_mesh(
                _blade_mesh(0.0),   # all start at angle=0; transform sets real angle
                color=color,
            ))

        # Yaw pointer — arrow along +X from hub centre; rotated by psi
        ln = HUB_RADIUS * 1.35
        self._ptr_a = pl.add_mesh(
            pv.Arrow(start=[0,0,HUB_HEIGHT+0.015], direction=[1,0,0],
                     scale=ln, tip_length=0.22, tip_radius=0.07, shaft_radius=0.025),
            color=C_POINTER,
        )

        # Throttle bar — full-height box, Z-scaled by throttle each frame
        _x, _y, self._max_h = -0.50, -0.50, 0.38
        self._thr_a = pl.add_mesh(
            pv.Box(bounds=(_x-.025, _x+.025, _y-.025, _y+.025, 0.0, self._max_h)),
            color=C_THROTTLE, opacity=0.90,
        )
        # Frame (static)
        pl.add_mesh(
            pv.Box(bounds=(_x-.027, _x+.027, _y-.027, _y+.027, 0.0, self._max_h)),
            color=(0.35,0.35,0.35), opacity=0.45, style="wireframe",
        )
        # Equilibrium mark (static)
        eq_z = _EQ_THROTTLE * self._max_h
        pl.add_mesh(
            pv.Box(bounds=(_x-.032, _x+.032, _y-.002, _y+.002,
                           eq_z-.003, eq_z+.003)),
            color=C_EQ_LINE, opacity=0.95,
        )

        # Pre-create ALL text actors — updated in-place each frame (no recreation).
        # Use pixel positions so all actors are vtkTextActor (supports SetInput).
        w, h = pl.window_size  # default window size for positioning
        self._hud_actor = pl.add_text(
            " ", position=(w - 240, h - 160), font_size=8, font="courier", color="white",
        )
        self._psi_dot_actor = pl.add_text(
            " ", position=(w - 240, 30), font_size=13, font="courier", color="white",
        )
        self.event_log.init_actors(w, h)

    # ── Per-frame update — transforms only, no mesh creation ─────────────

    def update(self, frame: TorqueTelemetryFrame) -> None:
        psi   = math.radians(frame.psi_deg)
        roll  = math.radians(frame.roll_deg)
        pitch = math.radians(frame.pitch_deg)
        omega = frame.omega_axle_rads
        dt    = max(0.0, frame.t - self._prev_t)
        self._axle_angle = (self._axle_angle + omega * dt) % (2 * math.pi)
        self._prev_t     = frame.t

        # Full hub orientation: yaw from PID + any tilt from profile
        Rhub = _r_hub4(roll, pitch, psi)

        # Hub body and quadrants rotate together with full orientation
        self._hub_a.user_matrix = Rhub
        for a in self._quad_a:
            a.user_matrix = Rhub
        for a in self._spoke_a:
            a.user_matrix = Rhub

        # Rotor blades each get their own axle rotation
        for i, a in enumerate(self._blade_a):
            a.user_matrix = _rz4(self._axle_angle + i * math.pi / 2)

        # Yaw pointer — follows hub heading (full tilt orientation)
        self._ptr_a.user_matrix = Rhub

        # Throttle bar: scale Z by throttle (box goes 0→max_h, scale → 0→thr*max_h)
        M = np.eye(4, dtype=float)
        M[2, 2] = max(0.002, frame.throttle)
        self._thr_a.user_matrix = M

        # Text overlays (fast — just string replacement)
        self._draw_hud(frame)
        self.event_log.check(frame)
        self.event_log.render()

    # ── HUD text — updates in-place, no actor recreation ─────────────────

    def _draw_hud(self, frame: TorqueTelemetryFrame) -> None:
        self._hud_actor.SetInput("\n".join([
            f"t      = {frame.t:7.2f} s",
            f"psi    = {frame.psi_deg:+7.2f} deg",
            f"psi_dt = {frame.psi_dot_degs:+7.2f} deg/s",
            f"thr    = {frame.throttle:7.3f}",
            f"omega  = {frame.omega_axle_rads:6.1f} rad/s",
            f"phase  = {frame.phase}",
        ]))
        self._psi_dot_actor.SetInput(
            f"psi_dot = {frame.psi_dot_degs:+.2f} deg/s"
        )
        r, g, b = (1, 0, 0) if abs(frame.psi_dot_degs) > _THRESHOLD else (1, 1, 1)
        self._psi_dot_actor.GetTextProperty().SetColor(r, g, b)


# ---------------------------------------------------------------------------
# Player
# ---------------------------------------------------------------------------

def play(frames: List[TorqueTelemetryFrame],
         meta: dict,
         fps: float = 30.0,
         export: Optional[str] = None,
         all_paths: Optional[list] = None,
         file_idx: Optional[list] = None,
         load_fn=None) -> None:
    if not frames:
        print("No frames.")
        return

    title = (
        f"Counter-torque test  omega={meta.get('omega_axle_rads','?'):.1f} rad/s  "
        f"result={meta.get('result','?')}"
    )

    pl = pv.Plotter(title=title, off_screen=(export is not None))
    pl.set_background((0.05, 0.05, 0.08))   # very dark blue-black
    pl.camera_position = [
        (0.55, -0.75, 0.80),
        (0.0,   0.0,  0.20),
        (0.0,   0.0,  1.0),
    ]
    pl.enable_anti_aliasing()

    scene = TorqueScene(
        pl,
        settle_s       = float(meta.get("settle_s",       40.0)),
        observe_s      = float(meta.get("observe_s",       20.0)),
        threshold_degs = float(meta.get("threshold_degs",  1.0)),
    )

    # ── Auto-seek: start 5 s before hub dynamics kick in ──────────────────
    dyn_t   = _find_dynamics_start(frames)
    loop_t0 = max(frames[0].t, dyn_t - 5.0)   # loop start time (sim seconds)
    loop_i0 = next((i for i, f in enumerate(frames) if f.t >= loop_t0), 0)
    print(f"Dynamics start: t={dyn_t:.2f}s  "
          f"Loop start: t={frames[loop_i0].t:.2f}s  (frame {loop_i0})")

    idx     = [loop_i0]
    playing = [True]
    speed   = [1.0]

    def _interp_at(sim_target: float) -> TorqueTelemetryFrame:
        """Return a frame interpolated to exactly sim_target time."""
        # Find the last frame at or before sim_target
        i = idx[0]
        while i < len(frames) - 1 and frames[i + 1].t <= sim_target:
            i += 1
        idx[0] = i
        if i >= len(frames) - 1:
            return frames[-1]
        f1, f2 = frames[i], frames[i + 1]
        dt = f2.t - f1.t
        alpha = (sim_target - f1.t) / dt if dt > 1e-9 else 0.0
        alpha = max(0.0, min(1.0, alpha))
        return _lerp_frame(f1, f2, alpha)

    def _draw():
        scene.update(frames[max(0, min(idx[0], len(frames)-1))])

    # Mutable references for file cycling
    _frames   = [frames]
    _meta     = [meta]
    _file_n   = [0 if file_idx is None else file_idx[0]]
    _n_files  = len(all_paths) if all_paths else 1

    def _make_omega_fn(m: dict):
        """Return omega(frame_t) using the profile stored in metadata."""
        profile  = m.get("profile", m.get("test", "constant"))
        omega_nom = float(m.get("omega_axle_rads", 28.0))
        dyn_start = _find_dynamics_start(_frames[0])
        omega_fn, _ = _MEDIATOR_PROFILES.get(profile, _MEDIATOR_PROFILES["constant"])
        startup_hold = 10.0   # matches STARTUP_HOLD_S in conftest

        def _omega(frame_t: float) -> float:
            # dynamics_t mirrors mediator: t - startup_hold_s (clamped ≥ 0)
            dynamics_t = max(0.0, frame_t - dyn_start)
            return max(1.0, omega_fn(dynamics_t, omega_nom))

        return _omega

    _omega_fn = [_make_omega_fn(meta)]

    def _reload(delta: int) -> None:
        if all_paths is None or load_fn is None:
            return
        _file_n[0] = (_file_n[0] + delta) % _n_files
        new_frames, new_meta = load_fn(_file_n[0])
        _frames[0]   = new_frames
        _meta[0]     = new_meta
        _omega_fn[0] = _make_omega_fn(new_meta)
        # Reset animation state
        idx[0]     = 0
        wall_t0[0] = time.monotonic()
        sim_t0[0]  = new_frames[0].t
        scene._axle_angle = 0.0
        scene._prev_t     = new_frames[0].t
        scene.event_log.reset(
            float(new_meta.get("settle_s", 40.0)),
            float(new_meta.get("observe_s", 20.0)),
            float(new_meta.get("threshold_degs", 1.0)),
        )
        # Update window title
        profile = new_meta.get("profile", new_meta.get("test", "?"))
        result  = new_meta.get("result", "?")
        pl.title = f"[{_file_n[0]+1}/{_n_files}] {profile}  result={result}"

    def _cb_space():  playing[0] = not playing[0]
    def _cb_tab():    _reload(+1)
    def _cb_prev():   _reload(-1)

    def _cb_right():
        playing[0] = False
        idx[0] = min(idx[0] + 1, len(frames) - 1)
        _draw()

    def _cb_left():
        playing[0] = False
        idx[0] = max(idx[0] - 1, 0)
        _draw()

    def _cb_plus():   speed[0] = min(speed[0] * 2, 16.0)
    def _cb_minus():  speed[0] = max(speed[0] / 2, 0.125)

    if export is None:
        pl.add_key_event("space", _cb_space)
        pl.add_key_event("Right", _cb_right)
        pl.add_key_event("Left",  _cb_left)
        pl.add_key_event("plus",  _cb_plus)
        pl.add_key_event("minus", _cb_minus)
        pl.add_key_event("n",     _cb_tab)    # n = next file
        pl.add_key_event("b",     _cb_prev)   # b = back (previous file)

    # FPS counter + controls — pre-created, FPS updated in-place via SetInput
    pl.add_text("Space=play/pause  </>=step  +/-=speed  N/B=next/prev file  drag=orbit",
                position=(10, 10), font_size=7, color=(0.45, 0.45, 0.45))
    _fps_actor = pl.add_text("FPS --.-", position=(10, 28),
                              font_size=9, font="courier", color=(0.5, 0.9, 0.5))

    # File counter (top-centre)
    _w, _h = pl.window_size
    _file_actor = pl.add_text(
        f"[1/{_n_files}] {meta.get('profile', meta.get('test','?'))}  {meta.get('result','?')}",
        position=(_w // 2 - 140, _h - 28), font_size=9, font="courier",
        color=(0.85, 0.85, 0.40),
    )

    # Sensor / motor panel — what is being sent to ArduPilot and motor state
    _sensor_actor = pl.add_text(" ", position=(10, _h - 270),
                                  font_size=7, font="courier",
                                  color=(0.55, 0.85, 0.55))

    _draw()

    # ── Export mode ────────────────────────────────────────────────────────
    if export is not None:
        pl.open_gif(export)
        for i, frame in enumerate(frames):
            scene.update(frame)
            pl.render()
            pl.write_frame()
            if i % 50 == 0:
                print(f"  frame {i}/{len(frames)}  t={frame.t:.1f}s")
        pl.close()
        print(f"Saved → {export}")
        return

    # ── Interactive: real-time interpolated playback ──────────────────────
    wall_t0   = [time.monotonic()]
    sim_t0    = [frames[loop_i0].t]
    _FRAME_DT = 1.0 / 30.0
    _fps_times: list[float] = []

    pl.show(interactive_update=True, auto_close=False)

    while pl.render_window is not None and pl.render_window.GetGenericContext():
        t_frame_start = time.monotonic()
        cur_frames    = _frames[0]

        # FPS
        _fps_times.append(t_frame_start)
        if len(_fps_times) > 30: _fps_times.pop(0)
        actual_fps = ((len(_fps_times)-1) / (_fps_times[-1] - _fps_times[0])
                      if len(_fps_times) >= 2 else 0.0)
        _fps_actor.SetInput(f"FPS {actual_fps:4.1f}  x{speed[0]:.2g}")

        # File counter
        cm = _meta[0]
        _file_actor.SetInput(
            f"[{_file_n[0]+1}/{_n_files}] "
            f"{cm.get('profile', cm.get('test','?'))}"
            f"  {cm.get('result','?')}"
        )

        if playing[0]:
            wall_elapsed = t_frame_start - wall_t0[0]
            sim_target   = sim_t0[0] + wall_elapsed * speed[0]

            if sim_target >= cur_frames[-1].t:
                idx[0]     = loop_i0
                wall_t0[0] = t_frame_start
                sim_t0[0]  = cur_frames[loop_i0].t
                scene._axle_angle = 0.0
                scene._prev_t     = cur_frames[loop_i0].t
                scene.event_log.reset(
                    scene.event_log._settle_s,
                    scene.event_log._observe_s,
                    scene.event_log._threshold,
                )
                sim_target = sim_t0[0]

            # Interpolate
            i = idx[0]
            while i < len(cur_frames) - 1 and cur_frames[i + 1].t <= sim_target:
                i += 1
            idx[0] = i
            if i < len(cur_frames) - 1:
                f1, f2 = cur_frames[i], cur_frames[i + 1]
                dt_ = f2.t - f1.t
                alpha = max(0.0, min(1.0, (sim_target - f1.t) / dt_)) if dt_ > 1e-9 else 0.0
                interp = _lerp_frame(f1, f2, alpha)
            else:
                interp = cur_frames[-1]

            # Inject profile-reconstructed omega so the rotor indicator and
            # HUD reflect the actual (varying) axle speed, not the nominal constant.
            real_omega = _omega_fn[0](interp.t)
            if abs(real_omega - interp.omega_axle_rads) > 0.01:
                interp = _dc.replace(interp, omega_axle_rads=real_omega)

            scene.update(interp)

            # Sensor / motor panel
            psi_rad     = math.radians(interp.psi_deg)
            pd_rad      = math.radians(interp.psi_dot_degs)
            om          = interp.omega_axle_rads
            thr         = interp.throttle
            pwm_ch4     = int(1000 + thr * 1000)
            om_motor    = om * (80 / 44)
            tau_motor   = 0.293 * max(0.0, thr - om_motor / 105.1)
            q_motor     = 1.818 * tau_motor
            q_bearing   = 0.005 * om
            _sensor_actor.SetInput("\n".join([
                "sent to ArduPilot",
                f" gyro z   = {-pd_rad:+.3f} rad/s",
                f" accel z  = {-9.81:.2f} m/s²",
                f" yaw (NED)= {-psi_rad:+.3f} rad",
                "motor state",
                f" Ch4 PWM  = {pwm_ch4:4d} us",
                f" throttle = {thr:.3f}",
                f" omega_m  = {om_motor:.1f} rad/s",
                f" tau_m    = {tau_motor:.4f} Nm",
                f" Q_motor  = {-q_motor:+.4f} Nm",
                f" Q_bear   = {+q_bearing:+.4f} Nm",
                f" Q_net    = {q_bearing-q_motor:+.4f} Nm",
            ]))
        else:
            wall_t0[0] = t_frame_start
            sim_t0[0]  = cur_frames[idx[0]].t
            _draw()

        budget_ms = max(1, int((_FRAME_DT - (time.monotonic() - t_frame_start)) * 1000))
        pl.update(budget_ms)

    pl.close()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description="3D real-time playback of counter-torque motor telemetry"
    )
    ap.add_argument("telemetry", nargs="?", default=None,
                    help="Path to a specific torque_telemetry*.json  "
                         "(default: auto-discover all in simulation/logs/)")
    ap.add_argument("--fps",    type=float, default=30.0,
                    help="Playback rate for export (default: 30)")
    ap.add_argument("--export", default=None,
                    help="Export to GIF path instead of interactive display")
    args = ap.parse_args()

    # ── Discover all telemetry files ──────────────────────────────────────
    if args.telemetry:
        all_paths = [Path(args.telemetry)]
    else:
        log_dir   = Path(__file__).resolve().parents[1] / "logs"
        all_paths = sorted(log_dir.glob("torque_telemetry*.json"))
        if not all_paths:
            print(f"No torque_telemetry*.json found in {log_dir}")
            return

    print(f"Found {len(all_paths)} telemetry file(s):")
    for p in all_paths:
        print(f"  {p.name}")

    # ── Load current file and play ────────────────────────────────────────
    file_idx = [0]

    def _load(idx: int):
        path = all_paths[idx % len(all_paths)]
        print(f"\nLoading [{idx % len(all_paths) + 1}/{len(all_paths)}]: {path.name}")
        src  = TorqueJSONSource(path)
        frms = list(src.frames())
        meta = src.meta
        print(f"  {len(frms)} frames  "
              f"t=[{frms[0].t:.1f}, {frms[-1].t:.1f}]s  "
              f"result={meta.get('result','?')}  "
              f"profile={meta.get('profile', meta.get('test','?'))}")
        return frms, meta

    if args.export:
        frames, meta = _load(0)
        play(frames, meta, fps=args.fps, export=args.export)
        return

    # Multi-file interactive: reload on Tab / N / P
    frames, meta = _load(0)
    play(frames, meta, fps=args.fps,
         all_paths=all_paths, file_idx=file_idx, load_fn=_load)


if __name__ == "__main__":
    main()
