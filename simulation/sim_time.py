"""
sim_time.py — Speed-adjusted timing utilities for the RAWES simulation harness.

Why this exists
---------------
The GCS (gcs.py) and stack fixture (conftest.py) contain wall-clock sleeps and
poll intervals that must shorten when ArduPilot SITL runs faster than real-time
(via --speedup N passed to sim_vehicle.py).  This module provides two functions:

    wall_s(nominal_s)    — convert a 1x duration to a wall-clock duration
    sim_sleep(nominal_s) — time.sleep(wall_s(nominal_s))

Speedup estimation
------------------
Speedup is estimated adaptively from observed timing events fed via
record_sim_step().  Two sources update the estimator:

    Mediator process  — records wall-clock time per 400 Hz physics step
                        (driven by recv_servos() round-trips to ArduPilot SITL)

    Test process      — records wall-clock deltas between MAVLink messages that
    (GCS / conftest)    carry time_boot_ms (e.g. ATTITUDE at 10 Hz sim-time)

Each process runs its own estimator instance; they are independent.  When the
estimator has no samples yet it falls back to the RAWES_SPEEDUP environment
variable, so static configuration always works too.

Environment variables (fallback / override)
-------------------------------------------
    RAWES_SPEEDUP=2.0     2x faster  (wall_s(1.0) → 0.5 s)

Read lazily (each call) so it takes effect even when set after import
(e.g. in conftest.py's pytest_configure hook).
"""

import math
import os
import threading
import time
from collections import deque


# ---------------------------------------------------------------------------
# Adaptive speedup estimator
# ---------------------------------------------------------------------------

class _SpeedEstimator:
    """Rolling-window estimate of simulation speedup from observed step timing.

    Feed it timing pairs via record(): how many wall-seconds elapsed for each
    sim-second of simulated time.  The speedup estimate is the reciprocal of
    the rolling mean of those ratios.

    Thread-safe: the mediator loop and GCS heartbeat thread may call record()
    concurrently.
    """

    def __init__(self, window: int = 50) -> None:
        self._lock    = threading.Lock()
        self._samples: deque = deque(maxlen=window)

    def record(self, wall_dt: float, sim_dt: float) -> None:
        """Record one observation: wall_dt wall-seconds elapsed for sim_dt sim-seconds."""
        if wall_dt <= 0 or sim_dt <= 0:
            return
        with self._lock:
            self._samples.append(wall_dt / sim_dt)   # wall-s per sim-s

    def speedup(self) -> float:
        """Return estimated speedup (sim-s per wall-s).

        Falls back to environment variables when no samples are available.
        """
        with self._lock:
            if not self._samples:
                return _env_speedup()
            mean_wall_per_sim = sum(self._samples) / len(self._samples)
        if mean_wall_per_sim <= 0:
            return _env_speedup()
        return 1.0 / mean_wall_per_sim   # sim-s per wall-s = speedup

    def reset(self) -> None:
        """Clear all samples (useful between test phases)."""
        with self._lock:
            self._samples.clear()


# Module-level estimator instance shared within this process.
_estimator = _SpeedEstimator()


# ---------------------------------------------------------------------------
# Environment-variable fallback
# ---------------------------------------------------------------------------

def _env_speedup() -> float:
    """Read speedup from environment variables (lazy, re-evaluated each call)."""
    raw = os.environ.get("RAWES_SPEEDUP", "1.0")
    try:
        return float(raw)
    except ValueError:
        return 1.0


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def get_speedup() -> float:
    """Return the current speedup estimate (adaptive if steps have been recorded,
    otherwise falls back to the RAWES_SPEEDUP environment variable)."""
    return _estimator.speedup()


def record_sim_step(wall_dt: float, sim_dt: float) -> None:
    """Feed a timing observation into the adaptive speedup estimator.

    Call this once per simulation step with:
        wall_dt  — wall-clock seconds the step actually took
        sim_dt   — sim-seconds that step advanced (e.g. DT_TARGET = 1/400)

    In the mediator, wall_dt is the total loop time (including any sleep).
    In the GCS, wall_dt is the wall-clock interval between MAVLink messages
    that carry time_boot_ms, and sim_dt is the corresponding sim-time delta
    (e.g. (msg.time_boot_ms - prev_boot_ms) / 1000.0).
    """
    _estimator.record(wall_dt, sim_dt)


def wall_s(nominal_s: float, *, floor: float = 0.005) -> float:
    """Convert a nominal (1x) duration to a wall-clock duration.

    At 1x speedup:  wall_s(0.5) == 0.5
    At 4x speedup:  wall_s(0.5) == 0.125

    Parameters
    ----------
    nominal_s : float
        Duration at 1x simulation speed [seconds].
    floor : float
        Minimum wall-clock value returned when speedup > 1 (default 5 ms).
        Prevents recv_match(timeout=0) busy-spin edge cases.
        In fast mode (speedup=inf) the floor is NOT applied; 0.0 is returned.

    Returns
    -------
    float
        Wall-clock seconds to use for timeouts / poll intervals.
    """
    sp = _estimator.speedup()
    if nominal_s <= 0:
        return 0.0
    return max(nominal_s / sp, floor)


def sim_sleep(nominal_s: float) -> None:
    """Sleep for nominal_s of simulation time.

    At 1x:    time.sleep(nominal_s)
    At 4x:    time.sleep(nominal_s / 4)
    """
    w = wall_s(nominal_s, floor=0.0)
    if w > 0:
        time.sleep(w)
