"""
test_wind_estimator.py — Unit tests for WindEstimator in trajectory.py.

Tests cover:
  - Not ready until min_samples reached
  - wind_dir_ned tracks mean horizontal hub position (→ wind direction)
  - v_inplane_ms tracks omega_spin via autorotation torque balance
  - Rolling window evicts old samples
  - DeschutterPlanner updates reel-in quaternion from live wind estimate

All positions use NED frame (X=North, Y=East, Z=Down).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from planner import WindEstimator, DeschutterPlanner, quat_apply, Q_IDENTITY


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

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
    est = WindEstimator(min_samples=5)
    assert not est.ready
    assert est.wind_dir_ned is None
    assert est.v_inplane_ms is None


def test_ready_after_min_samples():
    est = WindEstimator(min_samples=5)
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
    est = WindEstimator(min_samples=10)
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
    est = WindEstimator(min_samples=10)
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
    est = WindEstimator(min_samples=5)
    for i in range(10):
        est.update(_state([5.0, 30.0 + i * 0.1, -10.0]))
    d = est.wind_dir_ned
    assert d is not None
    assert abs(np.linalg.norm(d) - 1.0) < 1e-9


def test_wind_dir_z_always_zero():
    """wind_dir_ned must be horizontal (NED Z = 0)."""
    est = WindEstimator(min_samples=5)
    for i in range(10):
        est.update(_state([20.0, 40.0, -15.0 - i]))
    d = est.wind_dir_ned
    assert d is not None
    assert abs(d[2]) < 1e-9


# ---------------------------------------------------------------------------
# In-plane wind speed from omega_spin
# ---------------------------------------------------------------------------

def test_v_inplane_none_when_no_spin():
    est = WindEstimator(min_samples=5)
    for i in range(10):
        est.update(_state([0.0, 40.0, -12.0], omega_spin=0.0))
    assert est.v_inplane_ms is None


def test_v_inplane_formula():
    """v_inplane = omega_spin² × K_drag / K_drive."""
    K_drive = 1.4
    K_drag  = 0.01786
    omega   = 20.0
    expected = omega ** 2 * K_drag / K_drive

    est = WindEstimator(min_samples=5, K_drive=K_drive, K_drag=K_drag)
    for i in range(10):
        est.update(_state([0.0, 40.0, -12.0], omega_spin=omega))
    v = est.v_inplane_ms
    assert v is not None
    assert abs(v - expected) < 1e-6


# ---------------------------------------------------------------------------
# Rolling window
# ---------------------------------------------------------------------------

def test_rolling_window_evicts_old_samples():
    est = WindEstimator(window_s=5.0, min_samples=3)
    # Feed 10 samples with dt=1.0; internal clock goes 1..10; window=5 keeps last 5
    for i in range(10):
        est.update(_state([0.0, float(i), -5.0]), dt=1.0)
    # Internal clock = 10; cutoff = 10 - 5 = 5; keep entries with t >= 5
    remaining_ts = [t for t, _, _ in est._buf]
    assert all(t >= 5.0 for t in remaining_ts), f"stale samples remain: {remaining_ts}"


# ---------------------------------------------------------------------------
# DeschutterPlanner with wind_estimator
# ---------------------------------------------------------------------------

def test_deschutter_uses_initial_wind_before_estimator_ready():
    """Before estimator is ready, reel-in quaternion comes from fixed wind_ned."""
    wind_ned = np.array([0.0, 10.0, 0.0])  # NED: East wind
    est = WindEstimator(min_samples=100)  # high threshold — never ready in this test
    traj = DeschutterPlanner(
        t_reel_out=30, t_reel_in=30, t_transition=5,
        v_reel_out=0.4, v_reel_in=0.4,
        tension_out=200, tension_in=20,
        wind_ned=wind_ned,
        xi_reel_in_deg=55,
        wind_estimator=est,
    )
    # Step into reel-in phase (advance internal clock past t_reel_out=30 s)
    for _ in range(310):   # 310 × 0.1 s = 31 s
        cmd = traj.step(_state([14.0, 46.0, -12.0]), 0.1)  # NED: T @ ENU [46,14,12]
    q = cmd["attitude_q"]
    # Must not be identity — reel-in q should be applied
    assert not (abs(q[0] - 1.0) < 1e-6 and np.linalg.norm(q[1:]) < 1e-6)


def test_deschutter_updates_reel_in_q_when_estimator_ready():
    """After estimator converges on a different wind direction, reel-in q updates."""
    wind_initial = np.array([0.0, 10.0, 0.0])  # NED: East wind
    est = WindEstimator(min_samples=5)
    traj = DeschutterPlanner(
        t_reel_out=30, t_reel_in=30, t_transition=5,
        v_reel_out=0.4, v_reel_in=0.4,
        tension_out=200, tension_in=20,
        wind_ned=wind_initial,
        xi_reel_in_deg=55,
        wind_estimator=est,
    )
    # Record initial reel-in quaternion (from wind_initial = East)
    q_before = traj._attitude_q_reel_in.copy()

    # Feed estimator with hub positions indicating wind from north (NED X)
    # mean NED pos should be north of anchor → wind_dir = north
    for i in range(10):
        angle = i * 2 * np.pi / 10
        pos = np.array([46.0 + 10.0 * np.sin(angle), 10.0 * np.cos(angle), -12.0])
        traj.step(_state(pos, omega_spin=20.0), 0.1)

    q_after = traj._attitude_q_reel_in
    assert q_after is not None
    # The quaternion should have changed — wind direction has changed
    assert not np.allclose(q_before, q_after, atol=1e-3), \
        "reel-in quaternion did not update after wind estimator converged"

    # The new body_z from q_after should point roughly north+up in NED
    # NED up = [0,0,-1]; q_after rotates from [0,0,-1] toward the new wind direction
    body_z = quat_apply(q_after, np.array([0.0, 0.0, -1.0]))  # NED up
    assert body_z[0] > 0.3, f"expected northward tilt in body_z (NED X), got {body_z}"
