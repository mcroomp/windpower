"""
render_cycle.py — Render a RAWES telemetry file to MP4 or GIF.

Usage:
    python render_cycle.py telemetry_deschutter.json
    python render_cycle.py telemetry_deschutter.json --out cycle.mp4
    python render_cycle.py telemetry_deschutter.json --out cycle.gif --fps 15
    python render_cycle.py telemetry.csv --source csv --speed 4

Options:
    --out FILE       Output file (.mp4 or .gif).  Default: <input stem>.mp4
    --source         json (default) or csv
    --fps FPS        Playback frame rate.  Default: 20
    --speed N        Real-time multiplier: every Nth frame is rendered.
                     speed=2 → 2× faster, half the frames.  Default: 2
    --width W        Frame width in pixels.  Default: 1280
    --height H       Frame height in pixels.  Default: 720
    --cam-pos        Camera position  "x,y,z".  Default: auto
    --cam-focal      Camera focal point "x,y,z".  Default: auto
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Optional

# Ensure simulation/ is on sys.path when run from anywhere
_HERE = Path(__file__).resolve().parent
_SIM  = _HERE.parent
if str(_SIM) not in sys.path:
    sys.path.insert(0, str(_SIM))

# Add ffmpeg to PATH if it lives in the standard Windows location
_FFMPEG = Path(r"C:\ffmpeg\bin")
if _FFMPEG.exists():
    os.environ["PATH"] = str(_FFMPEG) + os.pathsep + os.environ.get("PATH", "")

try:
    import pyvista as pv
except ImportError:
    sys.exit("PyVista not found.  pip install pyvista")

from viz3d.telemetry import JSONSource, CSVSource
from viz3d.visualize_3d import RAWESVisualizer, fit_camera


# ---------------------------------------------------------------------------
# Default camera: 3/4 view that fits the full deschutter trajectory
#   Reel-out: hub ~46–62 m East, ~14 m North, ~12 m alt
#   Reel-in:  hub climbs to ~54 m alt, returns toward anchor
# ---------------------------------------------------------------------------
DEFAULT_CAM_POS   = (110.0, -60.0, 70.0)
DEFAULT_CAM_FOCAL = ( 25.0,  15.0, 20.0)


def _parse_vec3(s: str) -> tuple[float, float, float]:
    parts = [float(v) for v in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("Expected x,y,z")
    return tuple(parts)  # type: ignore


def render(
    source,
    out_path: Path,
    fps: int = 20,
    speed: int = 2,
    width: int = 1280,
    height: int = 720,
    cam_pos: Optional[tuple] = None,
    cam_focal: Optional[tuple] = None,
) -> None:
    frames = list(source.frames())
    if not frames:
        sys.exit("No frames loaded — check source file.")

    t_total = frames[-1].t - frames[0].t
    export_indices = list(range(0, len(frames), speed))
    n = len(export_indices)
    duration = n / fps

    print(f"Source:   {len(frames)} frames  "
          f"({t_total:.1f} s real-time, {speed}× speed)")
    print(f"Output:   {out_path}  "
          f"({n} frames @ {fps} fps = {duration:.1f} s video)")
    print(f"Res:      {width}×{height}")

    viz = RAWESVisualizer(frames)

    pl = pv.Plotter(off_screen=True, window_size=[width, height])
    pl.set_background("#1a1a2e")

    ext = out_path.suffix.lower()
    if ext == ".mp4":
        pl.open_movie(str(out_path), framerate=fps, quality=8)
    elif ext == ".gif":
        pl.open_gif(str(out_path), fps=fps)
    else:
        sys.exit(f"Unsupported format: {ext}  (use .mp4 or .gif)")

    viz._add_static_scene(pl)
    if cam_pos is not None:
        pl.camera.position    = cam_pos
        pl.camera.focal_point = cam_focal or DEFAULT_CAM_FOCAL
        pl.camera.up          = (0, 0, 1)
    else:
        fit_camera(pl, viz._pos_history)
        if cam_focal is not None:
            pl.camera.focal_point = cam_focal

    for i, fi in enumerate(export_indices):
        viz._add_dynamic_actors(pl, fi)
        viz._update_trail(pl, fi)
        viz._update_hud(pl, fi)
        viz._update_inset(fi)
        pl.render()
        pl.write_frame()
        if i % 100 == 0 or i == n - 1:
            pct = 100 * i // n
            print(f"  [{pct:3d}%]  frame {i+1}/{n}  t={frames[fi].t:.1f}s",
                  flush=True)

    pl.close()
    size_mb = out_path.stat().st_size / 1e6
    print(f"Saved:  {out_path}  ({size_mb:.1f} MB)")


def main() -> None:
    p = argparse.ArgumentParser(
        description="Render RAWES telemetry to MP4 or GIF",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("input",          help="Telemetry file (.json or .csv)")
    p.add_argument("--out",          help="Output file (default: <input>.mp4)")
    p.add_argument("--source",       choices=["json", "csv"], default="json")
    p.add_argument("--fps",          type=int,   default=20)
    p.add_argument("--speed",        type=int,   default=2,
                   help="Render every Nth frame (default: 2 = 2× realtime)")
    p.add_argument("--width",        type=int,   default=1280)
    p.add_argument("--height",       type=int,   default=720)
    p.add_argument("--cam-pos",      type=_parse_vec3, default=None,
                   metavar="x,y,z")
    p.add_argument("--cam-focal",    type=_parse_vec3, default=None,
                   metavar="x,y,z")
    args = p.parse_args()

    in_path  = Path(args.input)
    out_path = Path(args.out) if args.out else in_path.with_suffix(".mp4")

    if not in_path.exists():
        sys.exit(f"Not found: {in_path}")

    source = CSVSource(in_path) if args.source == "csv" else JSONSource(in_path)

    render(
        source    = source,
        out_path  = out_path,
        fps       = args.fps,
        speed     = args.speed,
        width     = args.width,
        height    = args.height,
        cam_pos   = args.cam_pos,
        cam_focal = args.cam_focal,
    )


if __name__ == "__main__":
    main()
