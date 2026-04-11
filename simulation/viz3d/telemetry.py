"""
telemetry.py — RAWES simulation telemetry data stream abstraction.

Defines TelemetryFrame (unit of data) and TelemetrySource protocol.
The same renderer works with any source:

  CSVSource        — mediator telemetry.csv or unit-test telemetry CSV
  LiveQueueSource  — queue.Queue for future live rendering at 400 Hz

All telemetry is written as CSV via write_csv() from telemetry_csv.py and
read back as TelRow objects which carry the full R matrix, so blade
orientation in the 3D renderer is exact.

Usage:
    from telemetry_csv import TelRow, write_csv
    write_csv([TelRow.from_tel(d) for d in telemetry], "telemetry.csv")

    source = CSVSource("telemetry.csv")
    for frame in source.frames():
        renderer.update(frame)
"""
from __future__ import annotations

import queue
import sys
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
# CSVSource
# ---------------------------------------------------------------------------

class CSVSource:
    """
    Reads a telemetry CSV written by write_csv() (telemetry_csv.COLUMNS schema).

    R is read directly from the r00..r22 columns — exact, not approximated.
    Auto-detected from .csv extension in all visualizer CLIs.
    """
    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)

    def frames(self) -> Iterator[TelemetryFrame]:
        _sim = str(Path(__file__).resolve().parents[1])
        if _sim not in sys.path:
            sys.path.insert(0, _sim)
        from telemetry_csv import read_csv as _read_csv  # noqa: PLC0415

        for r in _read_csv(self._path):
            bzeq = r.body_z_eq
            yield TelemetryFrame(
                t                  = r.t_sim,
                pos_ned            = r.pos_ned,
                R                  = r.R,
                omega_spin         = r.omega_rotor,
                tether_tension     = r.tether_tension,
                tether_rest_length = r.tether_rest_length,
                swash_collective   = r.collective_rad,
                swash_tilt_lon     = r.tilt_lon,
                swash_tilt_lat     = r.tilt_lat,
                body_z_eq          = (bzeq if np.linalg.norm(bzeq) > 0.5 else None),
                wind_ned           = r.wind_ned,
            )


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
