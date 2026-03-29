"""
test_startup_damping.py — Verify startup damping behaviour.

The mediator applies a linearly-decaying drag force during the first
``startup_damp_T`` seconds so the hub stays near its equilibrium position
while the EKF initialises.  These tests replicate the mediator physics loop
(aero + tether + damping → dynamics step) WITHOUT running the full mediator
process, allowing fast, deterministic validation.

Key assertions:
  1. During full damping (alpha=1), position stays within a tight radius of
     the initial equilibrium — no sudden movement.
  2. Gravity compensation prevents the hub from sinking while velocity is
     damped.
  3. Position and velocity are continuous at the moment damping ends —
     no sudden jump when alpha crosses zero.
  4. After damping ends, the hub resumes physics-driven motion gradually
     (no instantaneous velocity step).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

# Add simulation/ to path so we can import modules directly.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from dynamics import RigidBodyDynamics
from aero import RotorAero
from tether import TetherModel
from frames import build_orb_frame

# ── Mediator constants (mirrored from mediator.py) ─────────────────────────

DT_TARGET        = 1.0 / 400.0   # 400 Hz
K_DRIVE_SPIN     = 1.4
K_DRAG_SPIN      = 0.01786
OMEGA_SPIN_MIN   = 0.5
I_SPIN_KGMS2     = 10.0

DEFAULT_POS0        = [46.258, 14.241, 12.530]
DEFAULT_VEL0        = [-0.257,  0.916, -0.093]
DEFAULT_BODY_Z      = [0.851018, 0.305391, 0.427206]
DEFAULT_OMEGA_SPIN  = 20.148

STARTUP_DAMP_K_VEL = 200.0    # N·s/m   — velocity drag
STARTUP_DAMP_K_ANG = 500.0    # N·m·s/rad
STARTUP_DAMP_K_POS = 2000.0   # N/m     — position spring (= k_vel²/(4m), critical damping)
MASS               = 5.0      # kg
G                  = 9.81     # m/s²

WIND_WORLD = np.array([10.0, 0.0, 0.0], dtype=float)   # 10 m/s East


# ── Helpers ─────────────────────────────────────────────────────────────────

def _make_dynamics(pos0=None, vel0=None):
    bz   = np.array(DEFAULT_BODY_Z, dtype=float)
    bz  /= np.linalg.norm(bz)
    R0   = build_orb_frame(bz)
    return RigidBodyDynamics(
        mass   = MASS,
        I_body = [5.0, 5.0, 10.0],
        I_spin = 0.0,
        pos0   = pos0 if pos0 is not None else DEFAULT_POS0,
        vel0   = vel0 if vel0 is not None else DEFAULT_VEL0,
        R0     = R0,
        omega0 = [0.0, 0.0, 0.0],
        z_floor = 1.0,
    )


def _make_tether():
    return TetherModel(
        anchor_enu             = np.zeros(3),
        rest_length            = 49.949,
        hub_mass               = MASS,
        axle_attachment_length = 0.0,
    )


def _physics_step(dynamics, aero, tether, omega_spin, alpha,
                  k_vel=STARTUP_DAMP_K_VEL, k_ang=STARTUP_DAMP_K_ANG,
                  k_pos=STARTUP_DAMP_K_POS,
                  pos0=None, t_sim=0.0):
    """Run one 400 Hz mediator physics step with the given damping alpha.

    Mirrors the mediator loop exactly:
      forces += aero + tether
      if alpha > 0:
          forces -= k_vel * alpha * vel          # velocity drag
          forces -= k_pos * alpha * (pos - pos0) # position spring → v→0 at p≈pos0
          forces[Z] += m*g*alpha                 # gravity compensation
      M_orbital -= (50 + k_ang*alpha) * omega
    """
    if pos0 is None:
        pos0 = np.array(DEFAULT_POS0, dtype=float)
    hub = dynamics.state
    forces = aero.compute_forces(
        collective_rad = 0.0,
        tilt_lon       = 0.0,
        tilt_lat       = 0.0,
        R_hub          = hub["R"],
        v_hub_world    = hub["vel"],
        omega_rotor    = omega_spin,
        wind_world     = WIND_WORLD,
        t              = t_sim,
    )
    tether_force, tether_moment = tether.compute(hub["pos"], hub["vel"], hub["R"])
    forces[0:3] += tether_force
    forces[3:6] += tether_moment

    if alpha > 0.0:
        forces[0:3] -= k_vel * alpha * hub["vel"]
        forces[0:3] -= k_pos * alpha * (hub["pos"] - pos0)
        forces[2]   += MASS * G * alpha      # gravity compensation

    M_orbital = forces[3:6] - aero.last_M_spin
    k_ang_total = 50.0 + k_ang * alpha
    M_orbital  -= k_ang_total * hub["omega"]

    # Update omega_spin
    Q_spin     = K_DRIVE_SPIN * aero.last_v_inplane - K_DRAG_SPIN * omega_spin**2
    omega_spin = max(OMEGA_SPIN_MIN, omega_spin + Q_spin / I_SPIN_KGMS2 * DT_TARGET)

    hub_state = dynamics.step(forces[0:3], M_orbital, DT_TARGET, omega_spin)
    return hub_state, omega_spin


# ── Tests ───────────────────────────────────────────────────────────────────

def test_full_damping_limits_position_drift():
    """With alpha=1.0 (full damping) for 5 s, the position spring must keep
    the hub within 0.1 m of the initial equilibrium in all three axes."""
    dynamics   = _make_dynamics()
    aero       = RotorAero()
    tether     = _make_tether()
    omega_spin = DEFAULT_OMEGA_SPIN

    pos0 = np.array(DEFAULT_POS0, dtype=float)
    max_drift  = 0.0
    n_steps    = int(5.0 / DT_TARGET)      # 5 seconds at 400 Hz

    for i in range(n_steps):
        hub, omega_spin = _physics_step(dynamics, aero, tether, omega_spin,
                                        alpha=1.0, t_sim=i * DT_TARGET)
        drift = np.linalg.norm(hub["pos"] - pos0)
        max_drift = max(max_drift, drift)

    assert max_drift < 0.1, (
        f"Hub drifted {max_drift:.3f} m from equilibrium during full damping "
        f"(limit: 0.1 m).  Position spring is not anchoring the hub."
    )


def test_full_damping_limits_velocity():
    """After 1 s of full damping, hub velocity must be near-zero (< 0.05 m/s).
    The velocity time-constant at k_vel=200 N·s/m, m=5 kg is 25 ms."""
    dynamics   = _make_dynamics()
    aero       = RotorAero()
    tether     = _make_tether()
    omega_spin = DEFAULT_OMEGA_SPIN

    n_steps = int(1.0 / DT_TARGET)   # 1 second
    for i in range(n_steps):
        hub, omega_spin = _physics_step(dynamics, aero, tether, omega_spin,
                                        alpha=1.0, t_sim=i * DT_TARGET)

    speed = np.linalg.norm(hub["vel"])
    # With the position spring (k_pos=2000 N/m) + velocity drag (k_vel=200 N·s/m)
    # the hub settles to v=0 at p≈p0.  Residual = spring_offset × k_vel / m ≈ 0.
    # Allow a small tolerance for RK4 integration and transient settling.
    assert speed < 0.02, (
        f"Hub speed after 1 s full damping: {speed:.5f} m/s (limit: 0.02 m/s).  "
        f"Position spring is not driving velocity to zero."
    )


def test_gravity_compensation_prevents_sinking():
    """With damping active and initial velocity = 0, the hub must not drop
    more than 0.5 m over 10 s.  Without gravity compensation the hub would
    fall at ½g t² ≈ 490 m/s² over 10 s."""
    dynamics   = _make_dynamics(vel0=[0.0, 0.0, 0.0])
    aero       = RotorAero()
    tether     = _make_tether()
    omega_spin = DEFAULT_OMEGA_SPIN

    z0      = DEFAULT_POS0[2]
    min_z   = z0
    n_steps = int(10.0 / DT_TARGET)

    for i in range(n_steps):
        hub, omega_spin = _physics_step(dynamics, aero, tether, omega_spin,
                                        alpha=1.0, t_sim=i * DT_TARGET)
        min_z = min(min_z, float(hub["pos"][2]))

    drop = z0 - min_z
    assert drop < 0.5, (
        f"Hub dropped {drop:.2f} m during full damping with zero initial vel "
        f"(limit: 0.5 m).  Gravity compensation is not working."
    )


def test_no_position_jump_at_damp_end():
    """The position must be continuous when damping ends.  Run the system to
    one step before damp-end (alpha≈ε) then one step after (alpha=0) and
    verify position changes by at most one step's worth of drift."""
    dynamics   = _make_dynamics()
    aero       = RotorAero()
    tether     = _make_tether()
    omega_spin = DEFAULT_OMEGA_SPIN

    # Run to near-end of damping (alpha close to zero but still positive)
    T_damp   = 30.0
    t_pre    = T_damp - DT_TARGET              # one step before ramp end
    n_warmup = int(t_pre / DT_TARGET)

    for i in range(n_warmup):
        t        = i * DT_TARGET
        alpha    = max(0.0, 1.0 - t / T_damp)
        hub, omega_spin = _physics_step(dynamics, aero, tether, omega_spin,
                                        alpha=alpha, t_sim=t)

    # Record state one step before damping ends
    pos_pre = dynamics.state["pos"].copy()
    vel_pre = dynamics.state["vel"].copy()

    # One step with alpha≈ε (last damped step)
    alpha_last = 1.0 - t_pre / T_damp
    hub, omega_spin = _physics_step(dynamics, aero, tether, omega_spin,
                                    alpha=alpha_last, t_sim=t_pre)
    pos_at_end = dynamics.state["pos"].copy()
    vel_at_end = dynamics.state["vel"].copy()

    # One step with alpha=0 (first undamped step)
    hub, omega_spin = _physics_step(dynamics, aero, tether, omega_spin,
                                    alpha=0.0, t_sim=T_damp)
    pos_after = dynamics.state["pos"].copy()
    vel_after = dynamics.state["vel"].copy()

    # Position step across the transition must be < 2× a single free-flight step
    # (free-flight step ≈ vel × dt ≈ max_vel_at_end × 0.0025 s)
    max_free_step = max(np.linalg.norm(vel_at_end), 0.1) * DT_TARGET * 2
    pos_jump = np.linalg.norm(pos_after - pos_at_end)
    assert pos_jump < max_free_step + 0.01, (
        f"Position jumped {pos_jump:.4f} m at damp-end (limit: {max_free_step:.4f} m).  "
        f"There is a discontinuity at the damping boundary."
    )

    # Velocity step must be < 0.5 m/s  (no sudden impulse when damping is removed)
    vel_jump = np.linalg.norm(vel_after - vel_at_end)
    assert vel_jump < 0.5, (
        f"Velocity jumped {vel_jump:.4f} m/s at damp-end (limit: 0.5 m/s).  "
        f"There is a velocity discontinuity at the damping boundary."
    )


def test_physics_resumes_gradually_after_damping():
    """After damping ends the hub should accelerate smoothly — speed must
    increase gradually, not jump suddenly.  Check that speed 0.5 s after
    damp-end is greater than immediately after (physics is active) but the
    rate of change is bounded."""
    T_damp   = 5.0   # short damp for speed
    dynamics   = _make_dynamics()
    aero       = RotorAero()
    tether     = _make_tether()
    omega_spin = DEFAULT_OMEGA_SPIN

    # Run through the damping phase
    n_damp = int(T_damp / DT_TARGET)
    for i in range(n_damp):
        t     = i * DT_TARGET
        alpha = max(0.0, 1.0 - t / T_damp)
        hub, omega_spin = _physics_step(dynamics, aero, tether, omega_spin,
                                        alpha=alpha, t_sim=t)

    speed_at_end = np.linalg.norm(dynamics.state["vel"])

    # Run 0.5 s of undamped physics
    n_post = int(0.5 / DT_TARGET)
    speed_samples = []
    for i in range(n_post):
        hub, omega_spin = _physics_step(dynamics, aero, tether, omega_spin,
                                        alpha=0.0, t_sim=T_damp + i * DT_TARGET)
        speed_samples.append(np.linalg.norm(hub["vel"]))

    # Speed should increase after damping ends (physics forces > zero)
    speed_0_5s = speed_samples[-1]
    assert speed_0_5s > speed_at_end, (
        f"Hub speed did not increase after damping ended "
        f"({speed_0_5s:.4f} ≤ {speed_at_end:.4f} m/s).  Physics may not be active."
    )

    # Speed increase must be gradual: average acceleration < 10 m/s²
    avg_accel = (speed_0_5s - speed_at_end) / 0.5
    assert avg_accel < 10.0, (
        f"Hub accelerated too fast after damp-end: {avg_accel:.2f} m/s² "
        f"(limit: 10 m/s²).  Transition is not gradual."
    )
