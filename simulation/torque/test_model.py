"""
torque/test_model.py — Unit tests for the counter-torque motor model.

Runs natively on Windows with the existing unit-test venv (pure Python, no
Docker, no ArduPilot SITL required).

Run with:
    simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/torque/test_model.py -v

Tests
-----
1. test_bearing_drag_alone          — zero throttle → hub spins in axle direction
2. test_equilibrium_throttle_holds  — computed feedforward keeps ψ = ψ_dot = 0
3. test_p_controller_convergence    — P-controller on yaw rate converges within 15 s
4. test_rpm_step_recovery           — step in ω_axle is rejected within 10 s
5. test_zero_axle_no_drift          — stopped axle → hub stays at rest
6. test_motor_reaction_direction    — motor torque must oppose bearing drag on hub
7. test_motor_saturation            — throttle clamped; torque always ≥ 0
8. test_gear_ratio_authority        — motor at full throttle must dominate bearing drag
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

# Make model importable regardless of working directory
sys.path.insert(0, str(Path(__file__).parent))
import model as m


# ── Simulation helpers ────────────────────────────────────────────────────────

DT   = 1.0 / 400.0   # 400 Hz — same as production mediator
TMAX = 30.0           # [s] used for steady-state checks


def _simulate(
    omega_axle_fn,    # callable(t: float) -> float
    throttle_fn,      # callable(state: m.HubState, t: float) -> float
    params: m.HubParams | None = None,
    t_end: float = TMAX,
    dt: float = DT,
) -> list[tuple[float, m.HubState]]:
    """
    Run the hub yaw dynamics and collect (time, state) samples.

    Returns a list of (t, HubState) tuples from t=0 to t=t_end inclusive.
    """
    if params is None:
        params = m.HubParams()
    state   = m.HubState()
    history = [(0.0, m.HubState(state.psi, state.psi_dot))]
    t = 0.0
    while t < t_end:
        omega    = omega_axle_fn(t)
        throttle = throttle_fn(state, t)
        state    = m.step(state, omega, throttle, params, dt)
        t       += dt
        history.append((t, m.HubState(state.psi, state.psi_dot)))
    return history


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_bearing_drag_alone():
    """
    With no motor (throttle=0), bearing drag alone accelerates the hub in the
    same direction as the axle rotation.  After 5 s, ψ_dot and ψ must both be
    clearly positive.
    """
    hist = _simulate(
        omega_axle_fn=lambda t: m.OMEGA_AXLE_NOMINAL,
        throttle_fn=lambda s, t: 0.0,
        t_end=5.0,
    )
    final = hist[-1][1]
    assert final.psi_dot > 0.05, (
        f"Hub should spin up from bearing drag; got ψ_dot={final.psi_dot:.4f} rad/s"
    )
    assert final.psi > 0.001, (
        f"Hub should accumulate yaw; got ψ={math.degrees(final.psi):.3f}°"
    )


def test_equilibrium_throttle_holds():
    """
    The feedforward throttle from equilibrium_throttle() must hold ψ = ψ_dot = 0
    for all time when started from rest.  Any deviation is a modelling error.
    """
    params      = m.HubParams()
    eq_throttle = m.equilibrium_throttle(m.OMEGA_AXLE_NOMINAL, params)

    # Throttle must be physically realizable
    assert 0.0 < eq_throttle < 1.0, (
        f"Equilibrium throttle out of range: {eq_throttle:.4f}"
    )

    hist = _simulate(
        omega_axle_fn=lambda t: m.OMEGA_AXLE_NOMINAL,
        throttle_fn=lambda s, t: eq_throttle,
        t_end=10.0,
    )
    max_psi_dot = max(abs(s.psi_dot) for _, s in hist)
    max_psi_deg = math.degrees(max(abs(s.psi) for _, s in hist))

    assert max_psi_dot < 1e-6, (
        f"ψ_dot not held to zero by equilibrium throttle; max={max_psi_dot:.2e} rad/s"
    )
    assert max_psi_deg < 1e-4, (
        f"ψ not held to zero by equilibrium throttle; max={max_psi_deg:.4f}°"
    )


def test_p_controller_convergence():
    """
    A proportional yaw-rate controller (feedforward + P-gain on ψ_dot) must
    drive ψ_dot to near-zero and keep accumulated yaw below 5° after settling.

    This models the ideal ArduPilot behaviour: neutral sticks → rate command = 0
    → tail motor corrects yaw drift.
    """
    params = m.HubParams()
    # Kp stability limit at 400 Hz: Kp_max ≈ 2.7 (eigenvalue × dt < 2.8 for RK4).
    # Use 1.0 for a comfortable safety margin while still being a meaningful P gain.
    Kp     = 1.0

    def throttle_fn(state: m.HubState, t: float) -> float:
        eq  = m.equilibrium_throttle(m.OMEGA_AXLE_NOMINAL, params)
        cmd = eq + Kp * state.psi_dot
        return max(0.0, min(1.0, cmd))

    hist = _simulate(
        omega_axle_fn=lambda t: m.OMEGA_AXLE_NOMINAL,
        throttle_fn=throttle_fn,
        t_end=TMAX,
    )

    # Check only the settled region (t > 5 s — controller has had time to act)
    late = [(t, s) for t, s in hist if t > 5.0]
    max_psi_dot      = max(abs(s.psi_dot) for _, s in late)
    max_psi_deg      = math.degrees(max(abs(s.psi) for _, s in late))

    assert max_psi_dot < 0.01, (
        f"P-controller should null yaw rate; settled max ψ_dot={max_psi_dot:.4f} rad/s"
    )
    assert max_psi_deg < 5.0, (
        f"Accumulated yaw should stay small; settled max ψ={max_psi_deg:.2f}°"
    )


def test_rpm_step_recovery():
    """
    Axle speed steps from 80% to 120% of nominal at t=5 s.  A feedforward +
    proportional controller should recover ψ_dot < 0.02 rad/s within 10 s of
    the step.
    """
    params     = m.HubParams()
    omega_low  = m.OMEGA_AXLE_NOMINAL * 0.8
    omega_high = m.OMEGA_AXLE_NOMINAL * 1.2
    Kp         = 1.0   # numerically stable at 400 Hz (see test_p_controller_convergence)

    def omega_fn(t: float) -> float:
        return omega_high if t >= 5.0 else omega_low

    def throttle_fn(state: m.HubState, t: float) -> float:
        eq  = m.equilibrium_throttle(omega_fn(t), params)
        cmd = eq + Kp * state.psi_dot
        return max(0.0, min(1.0, cmd))

    hist = _simulate(omega_axle_fn=omega_fn, throttle_fn=throttle_fn, t_end=20.0)

    # Check 10 s after the step (t > 15 s) — should be re-settled
    late = [(t, s) for t, s in hist if t > 15.0]
    max_psi_dot = max(abs(s.psi_dot) for _, s in late)
    assert max_psi_dot < 0.02, (
        f"Should recover after RPM step; late max ψ_dot={max_psi_dot:.4f} rad/s"
    )


def test_zero_axle_no_drift():
    """
    With axle stopped (ω_axle = 0), there is no bearing drag.  The hub must
    remain perfectly at rest regardless of throttle.
    """
    hist = _simulate(
        omega_axle_fn=lambda t: 0.0,
        throttle_fn=lambda s, t: 0.0,
        t_end=10.0,
    )
    max_psi_dot = max(abs(s.psi_dot) for _, s in hist)
    max_psi     = max(abs(s.psi)     for _, s in hist)
    assert max_psi_dot < 1e-12, f"Hub should be at rest; ψ_dot={max_psi_dot}"
    assert max_psi     < 1e-12, f"Hub should be at rest; ψ={max_psi}"


def test_motor_reaction_direction():
    """
    Increasing throttle must produce a torque on the hub that OPPOSES the
    direction of bearing drag (not adds to it).

    If this fails it means the gear-ratio sign is wrong — the motor would make
    things worse instead of better, and yaw regulation would be impossible.
    """
    params      = m.HubParams()
    omega_axle  = m.OMEGA_AXLE_NOMINAL
    omega_motor = omega_axle * params.gear_ratio

    tau_shaft       = m.motor_torque(0.5, omega_motor, params)
    Q_motor_on_hub  = -params.gear_ratio * tau_shaft   # same calc as in _derivatives

    # Bearing drag is positive (CCW); motor reaction must be negative (CW)
    assert Q_motor_on_hub < 0.0, (
        f"Motor reaction on hub must be negative (opposing bearing drag); "
        f"got {Q_motor_on_hub:.4f} N·m"
    )


def test_gear_ratio_authority():
    """
    At full throttle the motor reaction must exceed the bearing drag at nominal
    axle speed — otherwise yaw is uncontrollable at design-point RPM.
    """
    params      = m.HubParams()
    omega_axle  = m.OMEGA_AXLE_NOMINAL
    omega_motor = omega_axle * params.gear_ratio

    Q_bearing_magnitude = params.k_bearing * omega_axle        # N·m, magnitude
    tau_full            = m.motor_torque(1.0, omega_motor, params)
    Q_motor_magnitude   = params.gear_ratio * tau_full         # N·m, magnitude

    assert Q_motor_magnitude > Q_bearing_magnitude, (
        f"Motor at full throttle ({Q_motor_magnitude:.3f} N·m) must exceed bearing "
        f"drag ({Q_bearing_magnitude:.3f} N·m) — otherwise yaw is uncontrollable"
    )


def test_motor_saturation():
    """
    Motor torque must always be ≥ 0 and must never increase beyond the valid
    throttle range.  Clamping must be graceful.
    """
    params      = m.HubParams()
    omega_motor = m.OMEGA_AXLE_NOMINAL * params.gear_ratio

    for throttle in [-2.0, -1.0, -0.001, 0.0, 0.5, 1.0, 1.5, 2.0]:
        tau = m.motor_torque(throttle, omega_motor, params)
        assert tau >= 0.0, (
            f"Motor torque must be ≥ 0 for any input; got τ={tau:.4f} at throttle={throttle}"
        )

    # Monotonic: higher valid throttle → higher (or equal) torque
    tau_half = m.motor_torque(0.5, omega_motor, params)
    tau_full = m.motor_torque(1.0, omega_motor, params)
    assert tau_full >= tau_half, "Full throttle must give at least as much torque as half"


def test_equilibrium_throttle_scales_with_rpm():
    """
    Higher axle speed needs proportionally more throttle to maintain ψ_dot = 0.
    """
    params  = m.HubParams()
    thr_low = m.equilibrium_throttle(m.OMEGA_AXLE_NOMINAL * 0.5, params)
    thr_nom = m.equilibrium_throttle(m.OMEGA_AXLE_NOMINAL,       params)
    thr_hi  = m.equilibrium_throttle(m.OMEGA_AXLE_NOMINAL * 1.5, params)

    assert thr_low < thr_nom < thr_hi, (
        f"Equilibrium throttle should increase with RPM: "
        f"low={thr_low:.4f}, nom={thr_nom:.4f}, hi={thr_hi:.4f}"
    )
