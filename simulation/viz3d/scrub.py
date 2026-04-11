"""
scrub.py — Interactive frame scrubber for RAWES telemetry.

Opens a 3D window with a large slider.  Drag to any frame and the scene
renders live — no pre-rendered movie.

Usage:
    python scrub.py telemetry_deschutter.csv
    python scrub.py telemetry_closed_loop.csv
    python scrub.py logs/telemetry.csv

Controls:
    Slider     — drag to any frame
    Left/Right — step one frame (while paused)
    Space      — toggle auto-play
    + / -      — 2× / 0.5× auto-play speed
    Mouse      — orbit / pan / zoom
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Ensure simulation/ is on sys.path when run from anywhere
_HERE = Path(__file__).resolve().parent
_SIM  = _HERE.parent
if str(_SIM) not in sys.path:
    sys.path.insert(0, str(_SIM))

try:
    import pyvista as pv  # noqa: F401 — fail fast with a clear message
except ImportError:
    sys.exit("PyVista not found.  pip install pyvista")

from viz3d.telemetry import CSVSource
from viz3d.visualize_3d import RAWESVisualizer


def main() -> None:
    p = argparse.ArgumentParser(
        description="RAWES live scrubber — drag slider to step through frames",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("input", help="Telemetry CSV file")
    p.add_argument("--fps", type=float, default=20.0,
                   help="Auto-play frame rate when Space is pressed (default: 20)")
    p.add_argument("--trail", type=int, default=200,
                   help="Trajectory trail length in frames (default: 200)")
    args = p.parse_args()

    path = Path(args.input)
    if not path.exists():
        sys.exit(f"Not found: {path}")

    source = CSVSource(path)
    frames = list(source.frames())
    if not frames:
        sys.exit("No frames loaded — check file format.")

    t_span = frames[-1].t - frames[0].t
    print(f"Loaded {len(frames)} frames  "
          f"({t_span:.1f} s  @  {len(frames) / max(t_span, 1e-9):.0f} Hz)")
    print("Controls: drag slider | Left/Right step | Space play/pause | +/- speed")

    viz = RAWESVisualizer(frames, trail_len=args.trail, playback_fps=args.fps)
    viz.scrub()


if __name__ == "__main__":
    main()
