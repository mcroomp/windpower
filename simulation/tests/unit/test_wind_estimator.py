"""
test_wind_estimator.py — Unit tests for WindEstimator in trajectory.py.

Tests cover:
  - Not ready until min_samples reached
  - wind_dir_ned tracks mean horizontal hub position (→ wind direction)
  - v_inplane_ms tracks omega_spin via autorotation torque balance
  - Rolling window evicts old samples

All positions use NED frame (X=North, Y=East, Z=Down).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from planner import WindEstimator


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_SEED_EAST = np.array([0.0, 1.0, 0.0])   # NED: East — default seed for tests that don't care about direction
_SEED_EAST.flags.writeable = False


def _state(pos_ned, omega_spin=0.0, vel_ned=None):
    return {
        "pos_ned":    np.asarray(pos_ned, dtype=float),
        "vel_ned":    np.zeros(3) if vel_ned is None else np.asarray(vel_ned, dtype=float),
        "omega_spin": float(omega_spin),
    }


def _feed(est, states, dt=1.0):
    for s in states:
        est.update(s, dt)


# ---------------------------------------------------------------------------
# Readiness
# ---------------------------------------------------------------------------

def test_not_ready_initially():
    est = WindEstimator(seed_wind_ned=_SEED_EAST, min_samples=5)
    assert not est.ready
    # Before ready, wind_dir_ned returns the seed (never None)
    assert np.allclose(est.wind_dir_ned, _SEED_EAST)
    assert est.v_inplane_ms is None


def test_ready_after_min_samples():
    est = WindEstimator(seed_wind_ned=_SEED_EAST, min_samples=5)
    for i in range(4):
        est.update(_state([0.0, 10.0, -5.0]))  # NED: East
    assert not est.ready
    est.update(_state([0.0, 10.0, -5.0]))
    assert est.ready


# ---------------------------------------------------------------------------
# Wind direction from mean position
# ---------------------------------------------------------------------------

def test_wind_dir_eastward():
    """Hub consistently east of anchor → wind_dir_ned points east (NED Y)."""
    est = WindEstimator(seed_wind_ned=_SEED_EAST, min_samples=10)
    for i in range(20):
        angle = i * 2 * np.pi / 20
        # Hub orbits at radius ~48 m, east of anchor (mean NED Y ≈ 46)
        pos = np.array([10.0 * np.sin(angle),
                        46.0 + 10.0 * np.cos(angle),
                        -12.0])
        est.update(_state(pos))
    d = est.wind_dir_ned
    assert d is not None
    assert d[1] > 0.95, f"expected mostly east (NED Y), got {d}"
    assert abs(d[0]) < 0.1
    assert abs(d[2]) < 1e-9   # horizontal: NED Z = 0


def test_wind_dir_northward():
    """Hub consistently north of anchor → wind_dir_ned points north (NED X)."""
    est = WindEstimator(seed_wind_ned=_SEED_EAST, min_samples=10)
    for i in range(20):
        angle = i * 2 * np.pi / 20
        pos = np.array([46.0 + 10.0 * np.sin(angle),
                        10.0 * np.cos(angle),
                        -12.0])
        est.update(_state(pos))
    d = est.wind_dir_ned
    assert d is not None
    assert d[0] > 0.95, f"expected mostly north (NED X), got {d}"
    assert abs(d[1]) < 0.1


def test_wind_dir_is_unit_vector():
    est = WindEstimator(seed_wind_ned=_SEED_EAST, min_samples=5)
    for i in range(10):
        est.update(_state([5.0, 30.0 + i * 0.1, -10.0]))
    d = est.wind_dir_ned
    assert d is not None
    assert abs(np.linalg.norm(d) - 1.0) < 1e-9


def test_wind_dir_z_always_zero():
    """wind_dir_ned must be horizontal (NED Z = 0)."""
    est = WindEstimator(seed_wind_ned=_SEED_EAST, min_samples=5)
    for i in range(10):
        est.update(_state([20.0, 40.0, -15.0 - i]))
    d = est.wind_dir_ned
    assert d is not None
    assert abs(d[2]) < 1e-9


# ---------------------------------------------------------------------------
# In-plane wind speed from omega_spin
# ---------------------------------------------------------------------------

def test_v_inplane_none_when_no_spin():
    est = WindEstimator(seed_wind_ned=_SEED_EAST, min_samples=5)
    for i in range(10):
        est.update(_state([0.0, 40.0, -12.0], omega_spin=0.0))
    assert est.v_inplane_ms is None


def test_v_inplane_formula():
    """v_inplane = omega_spin² × K_drag / K_drive."""
    K_drive = 1.4
    K_drag  = 0.01786
    omega   = 20.0
    expected = omega ** 2 * K_drag / K_drive

    est = WindEstimator(seed_wind_ned=_SEED_EAST, min_samples=5, K_drive=K_drive, K_drag=K_drag)
    for i in range(10):
        est.update(_state([0.0, 40.0, -12.0], omega_spin=omega))
    v = est.v_inplane_ms
    assert v is not None
    assert abs(v - expected) < 1e-6


# ---------------------------------------------------------------------------
# Rolling window
# ---------------------------------------------------------------------------

def test_rolling_window_evicts_old_samples():
    est = WindEstimator(seed_wind_ned=_SEED_EAST, window_s=5.0, min_samples=3)
    # Feed 10 samples with dt=1.0; internal clock goes 1..10; window=5 keeps last 5
    for i in range(10):
        est.update(_state([0.0, float(i), -5.0]), dt=1.0)
    # Internal clock = 10; cutoff = 10 - 5 = 5; keep entries with t >= 5
    remaining_ts = [e[0] for e in est._buf]
    assert all(t >= 5.0 for t in remaining_ts), f"stale samples remain: {remaining_ts}"

