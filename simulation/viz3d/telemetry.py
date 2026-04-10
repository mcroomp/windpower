"""
telemetry.py — RAWES simulation telemetry data stream abstraction.

Defines TelemetryFrame (unit of data) and TelemetrySource protocol.
The same renderer works with any source:

  InMemorySource   — unit test history list (no file I/O)
  JSONSource       — JSON file written by unit tests via save_telemetry()
  CSVSource        — mediator telemetry.csv from stack tests (approx R)
  LiveQueueSource  — queue.Queue for future live rendering at 400 Hz

Usage from a unit test:
    import sys; sys.path.insert(0, ...)
    from viz3d.telemetry import save_telemetry
    result = _run_deschutter_cycle()
    save_telemetry("telemetry.json", result["telemetry"])

Usage from the visualizer:
    source = JSONSource("telemetry.json")
    for frame in source.frames():
        renderer.update(frame)
"""
from __future__ import annotations

import csv
import json
import math
import queue
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator, List, Optional, Protocol, runtime_checkable

import numpy as np


# ---------------------------------------------------------------------------
# TelemetryFrame
# ---------------------------------------------------------------------------

@dataclass
class TelemetryFrame:
    """One snapshot of RAWES simulation state for 3D rendering."""

    t: float                            # simulation time [s]
    pos_ned: np.ndarray                 # hub position NED [m], shape (3,)
    R: np.ndarray                       # body→NED rotation matrix, shape (3,3)
    omega_spin: float                   # rotor spin rate [rad/s]
    tether_tension: float = 0.0        # tether tension [N]
    tether_rest_length: float = 0.0    # paid-out tether length [m]
    swash_collective: float = 0.0      # collective blade pitch [rad]
    swash_tilt_lon: float = 0.0        # longitudinal swashplate tilt, normalised [-1,1]
    swash_tilt_lat: float = 0.0        # lateral swashplate tilt, normalised [-1,1]
    body_z_eq: Optional[np.ndarray] = None  # equilibrium body_z setpoint (unit vec, NED)
    wind_ned: np.ndarray = field(
        default_factory=lambda: np.array([10.0, 0.0, 0.0])
    )

    @property
    def body_z(self) -> np.ndarray:
        """Rotor axle direction (world NED), i.e. R[:,2]."""
        return self.R[:, 2]

    @property
    def altitude(self) -> float:
        return float(-self.pos_ned[2])

    @property
    def tether_length(self) -> float:
        return float(np.linalg.norm(self.pos_ned))


# ---------------------------------------------------------------------------
# TelemetrySource protocol
# ---------------------------------------------------------------------------

@runtime_checkable
class TelemetrySource(Protocol):
    def frames(self) -> Iterator[TelemetryFrame]: ...


# ---------------------------------------------------------------------------
# InMemorySource
# ---------------------------------------------------------------------------

class InMemorySource:
    """
    Wraps a list of dicts from a unit test telemetry recording.

    Expected keys per dict (mediator-compatible names):
        t_sim, pos_ned, R, omega_rotor
    Optional:
        tether_tension, tether_rest_length, swash_collective,
        swash_tilt_lon, swash_tilt_lat, body_z_eq, wind_ned
    """
    def __init__(self, history: List[dict]) -> None:
        self._history = history

    def frames(self) -> Iterator[TelemetryFrame]:
        for h in self._history:
            pos = np.asarray(h.get("pos_ned", [0.0, 0.0, 0.0]), dtype=float)
            R_raw = h.get("R", np.eye(3))
            R = np.asarray(R_raw, dtype=float).reshape(3, 3)
            bzeq = h.get("body_z_eq")
            yield TelemetryFrame(
                t                  = float(h["t_sim"]),
                pos_ned            = pos,
                R                  = R,
                omega_spin         = float(h.get("omega_rotor", 0.0)),
                tether_tension     = float(h.get("tether_tension", 0.0)),
                tether_rest_length = float(h.get("tether_rest_length", 0.0)),
                swash_collective   = float(h.get("collective_rad", 0.0)),
                swash_tilt_lon     = float(h.get("tilt_lon", 0.0)),
                swash_tilt_lat     = float(h.get("tilt_lat", 0.0)),
                body_z_eq          = (np.asarray(bzeq, dtype=float)
                                      if bzeq is not None else None),
                wind_ned           = np.asarray(
                    h.get("wind_ned", [10.0, 0.0, 0.0]), dtype=float
                ),
            )


# ---------------------------------------------------------------------------
# JSONSource
# ---------------------------------------------------------------------------

class JSONSource:
    """
    Reads a JSON telemetry file written by save_telemetry().

    File format: JSON array of frame dicts (same keys as InMemorySource).
    """
    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)

    def frames(self) -> Iterator[TelemetryFrame]:
        with open(self._path) as fh:
            data = json.load(fh)
        yield from InMemorySource(data).frames()


# ---------------------------------------------------------------------------
# CSVSource  (mediator telemetry.csv from stack tests)
# ---------------------------------------------------------------------------

class CSVSource:
    """
    Reads the mediator telemetry CSV (telemetry.csv from dev.sh test-stack).

    R is approximated from hub position: body_z ≈ tether direction, disk
    plane built by _orb_frame_from_pos().  For exact orientation, use
    JSONSource from unit tests which log R directly.
    """
    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)

    def frames(self) -> Iterator[TelemetryFrame]:
        with open(self._path, newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                try:
                    pos = np.array([
                        float(row["hub_pos_x"]),
                        float(row["hub_pos_y"]),
                        float(row["hub_pos_z"]),
                    ])
                    R = _orb_frame_from_pos(pos)
                    yield TelemetryFrame(
                        t                  = float(row.get("t_sim", 0)),
                        pos_ned            = pos,
                        R                  = R,
                        omega_spin         = float(row.get("omega_rotor", 0)),
                        tether_tension     = float(row.get("tether_tension", 0)),
                        tether_rest_length = float(row.get("tether_rest_length", 0)),
                        swash_collective   = float(row.get("collective_rad", 0)),
                        swash_tilt_lon     = float(row.get("tilt_lon", 0)),
                        swash_tilt_lat     = float(row.get("tilt_lat", 0)),
                    )
                except (KeyError, ValueError):
                    continue


# ---------------------------------------------------------------------------
# LiveQueueSource  (future: live rendering fed by mediator callback)
# ---------------------------------------------------------------------------

class LiveQueueSource:
    """
    Reads TelemetryFrames from a queue.Queue for live rendering.

    Wire-up in mediator.py:
        from viz3d.telemetry import TelemetryFrame, LiveQueueSource
        q = queue.Queue()
        source = LiveQueueSource(q)
        # in step loop: q.put(TelemetryFrame(...))
        # when done:    q.put(None)
    """
    def __init__(self, q: "queue.Queue", timeout: float = 5.0) -> None:
        self._q = q
        self._timeout = timeout

    def frames(self) -> Iterator[TelemetryFrame]:
        while True:
            try:
                frame = self._q.get(timeout=self._timeout)
            except queue.Empty:
                return
            if frame is None:
                return
            yield frame


# ---------------------------------------------------------------------------
# save_telemetry — helper for unit tests
# ---------------------------------------------------------------------------

def save_telemetry(path: str | Path, frames: List[dict]) -> None:
    """
    Serialise a unit-test telemetry list to a JSON file.

    Example:
        result = _run_deschutter_cycle()
        save_telemetry("telemetry_deschutter.json", result["telemetry"])
    """
    path = Path(path)
    with open(path, "w") as fh:
        json.dump(frames, fh)
    print(f"Telemetry saved: {path}  ({len(frames)} frames)")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _orb_frame_from_pos(pos_ned: np.ndarray) -> np.ndarray:
    """
    Approximate rotation matrix from hub NED position alone.
    body_z = tether direction (NED), body_x = East (NED [0,1,0]) projected onto disk plane.
    Matches build_orb_frame() in frames.py.
    """
    body_z = pos_ned / (np.linalg.norm(pos_ned) + 1e-12)
    east   = np.array([0.0, 1.0, 0.0])   # East in NED
    body_x = east - np.dot(east, body_z) * body_z
    norm_x = np.linalg.norm(body_x)
    if norm_x < 1e-6:
        east   = np.array([1.0, 0.0, 0.0])   # North in NED (fallback)
        body_x = east - np.dot(east, body_z) * body_z
        norm_x = np.linalg.norm(body_x)
    body_x /= norm_x
    body_y  = np.cross(body_z, body_x)
    return np.column_stack([body_x, body_y, body_z])
