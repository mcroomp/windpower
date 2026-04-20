"""
tests/unit/test_torque_model.py — Unit tests for the counter-torque motor model.

Runs natively on Windows with the existing unit-test venv (pure Python, no
Docker, no ArduPilot SITL required).

Run with:
    simulation/.venv/Scripts/python.exe -m pytest simulation/tests/unit/test_torque_model.py -v

Tests
-----
1. test_low_throttle_causes_cw_drift    — throttle=0 -> hub spins CW at omega_rotor
2. test_high_throttle_causes_ccw_drift  — throttle=1 -> hub counter-rotates CCW after spin-up
3. test_equilibrium_throttle_holds      — computed feedforward keeps psi_dot = 0 at steady state
4. test_p_controller_convergence        — P-controller on yaw rate nulls drift
5. test_rpm_step_recovery               — step in omega_rotor recovered by feedforward + P
6. test_zero_axle_no_drift              — stopped axle, throttle=0 -> hub stays at rest
7. test_full_throttle_ccw_capability    — full throttle has significant CCW authority at steady state
8. test_throttle_clamping               — throttle outside [0,1] clamped gracefully
9. test_equilibrium_throttle_scales_with_rpm — higher RPM needs higher throttle
10. test_lua_p_controller               — mirrors rawes.lua run_yaw_trim() P gains
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

# simulation/ is two levels up from tests/unit/
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import torque_model as m


# ── Simulation helpers ────────────────────────────────────────────────────────

DT   = 1.0 / 400.0   # 400 Hz — same as production mediator
TMAX = 30.0           # [s] used for steady-state checks


def _simulate(
    omega_rotor_fn,   # callable(t: float) -> float
    throttle_fn,      # callable(state: m.HubState, t: float) -> float
    params: m.HubParams | None = None,
    t_end: float = TMAX,
    dt: float = DT,
    initial_state: m.HubState | None = None,
) -> list[tuple[float, m.HubState]]:
    """Run the hub yaw dynamics and collect (time, state) samples."""
    if params is None:
        params = m.HubParams()
    state   = initial_state if initial_state is not None else m.HubState()
    history = [(0.0, m.HubState(state.psi, state.psi_dot, state.omega_motor))]
    t = 0.0
    while t < t_end:
        omega    = omega_rotor_fn(t)
        throttle = throttle_fn(state, t)
        state    = m.step(state, omega, throttle, params, dt)
        t       += dt
        history.append((t, m.HubState(state.psi, state.psi_dot, state.omega_motor)))
    return history


def _warm_up(
    omega_rotor: float,
    throttle: float,
    params: m.HubParams | None = None,
    t_warmup: float = 0.3,
    dt: float = DT,
) -> m.HubState:
    """
    Run the motor at a fixed throttle until omega_motor reaches steady state
    (~15 x MOTOR_TAU).  Returns the final HubState with omega_motor settled.
    """
    if params is None:
        params = m.HubParams()
    state = m.HubState()
    t = 0.0
    while t < t_warmup:
        state = m.step(state, omega_rotor, throttle, params, dt)
        t += dt
    return state


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_low_throttle_causes_cw_drift():
    """
    With throttle=0 the motor is commanded to zero speed.  The outer hub still
    spins at omega_rotor, so the inner assembly is dragged CW with it.

    omega_motor starts at 0 and stays at 0 (commanded=0), so psi_dot = omega_rotor > 0.
    """
    params = m.HubParams()
    state  = m.step(m.HubState(), m.OMEGA_ROTOR_NOMINAL, 0.0, params, DT)
    assert state.psi_dot > 0.1, (
        f"Throttle=0 should cause CW drift (psi_dot > 0); got {state.psi_dot:.4f} rad/s"
    )


def test_high_throttle_causes_ccw_drift():
    """
    With throttle=1 the motor is commanded above equilibrium speed.
    After the motor spins up (>> MOTOR_TAU), the inner assembly counter-rotates CCW.

    psi_dot = omega_rotor - omega_motor / GEAR_RATIO < 0 once omega_motor -> RPM_SCALE.
    """
    params = m.HubParams()
    # warm up to steady state at throttle=1
    state = _warm_up(m.OMEGA_ROTOR_NOMINAL, 1.0, params)
    assert state.psi_dot < -0.1, (
        f"Throttle=1 should cause CCW drift (psi_dot < 0) at steady state; "
        f"got {state.psi_dot:.4f} rad/s (omega_motor={state.omega_motor:.2f})"
    )


def test_equilibrium_throttle_holds():
    """
    The feedforward throttle from equilibrium_throttle() must hold psi_dot = 0
    once the motor has reached steady state.

    During the startup transient (t < ~5 x MOTOR_TAU = 0.1 s), omega_motor is
    still spinning up and psi_dot will be nonzero.  Check only late-time values.
    """
    params      = m.HubParams()
    eq_throttle = m.equilibrium_throttle(m.OMEGA_ROTOR_NOMINAL, params)

    assert 0.0 < eq_throttle < 1.0, f"Equilibrium throttle out of range: {eq_throttle:.4f}"

    hist = _simulate(
        omega_rotor_fn=lambda t: m.OMEGA_ROTOR_NOMINAL,
        throttle_fn=lambda s, t: eq_throttle,
        t_end=10.0,
    )
    # skip the motor spin-up transient (first 0.5 s >> 25 x MOTOR_TAU)
    late = [(t, s) for t, s in hist if t > 0.5]
    max_psi_dot = max(abs(s.psi_dot) for _, s in late)
    assert max_psi_dot < 1e-9, (
        f"Equilibrium throttle should hold psi_dot=0 at steady state; "
        f"max={max_psi_dot:.2e} rad/s"
    )


def test_p_controller_convergence():
    """
    A proportional controller (feedforward + P-gain on psi_dot) must null yaw rate.

    Kp must be small enough to avoid throttle saturation during the motor-lag
    transient (< (1-eq) / omega_rotor_nominal ~= 0.018).  Kp=0.01 is used here.
    """
    params = m.HubParams()
    Kp     = 0.01

    def throttle_fn(state: m.HubState, t: float) -> float:
        eq  = m.equilibrium_throttle(m.OMEGA_ROTOR_NOMINAL, params)
        cmd = eq + Kp * state.psi_dot
        return max(0.0, min(1.0, cmd))

    hist = _simulate(
        omega_rotor_fn=lambda t: m.OMEGA_ROTOR_NOMINAL,
        throttle_fn=throttle_fn,
        t_end=TMAX,
    )

    late = [(t, s) for t, s in hist if t > 5.0]
    max_psi_dot = max(abs(s.psi_dot) for _, s in late)
    assert max_psi_dot < 0.01, (
        f"P-controller should null yaw rate; settled max psi_dot={max_psi_dot:.4f} rad/s"
    )


def test_rpm_step_recovery():
    """
    Axle speed steps from 80% to 120% of nominal at t=5 s.  A feedforward +
    proportional controller should recover psi_dot < 0.02 rad/s after the step.
    """
    params     = m.HubParams()
    omega_low  = m.OMEGA_ROTOR_NOMINAL * 0.8
    omega_high = m.OMEGA_ROTOR_NOMINAL * 1.2
    Kp         = 0.01

    def omega_fn(t: float) -> float:
        return omega_high if t >= 5.0 else omega_low

    def throttle_fn(state: m.HubState, t: float) -> float:
        eq  = m.equilibrium_throttle(omega_fn(t), params)
        cmd = eq + Kp * state.psi_dot
        return max(0.0, min(1.0, cmd))

    hist = _simulate(omega_rotor_fn=omega_fn, throttle_fn=throttle_fn, t_end=20.0)

    late = [(t, s) for t, s in hist if t > 15.0]
    max_psi_dot = max(abs(s.psi_dot) for _, s in late)
    assert max_psi_dot < 0.02, (
        f"Should recover after RPM step; late max psi_dot={max_psi_dot:.4f} rad/s"
    )


def test_zero_axle_no_drift():
    """
    With axle stopped (omega_rotor=0) and throttle=0, psi_dot = 0 - 0 = 0.
    The hub must remain perfectly at rest.
    """
    hist = _simulate(
        omega_rotor_fn=lambda t: 0.0,
        throttle_fn=lambda s, t: 0.0,
        t_end=10.0,
    )
    max_psi_dot = max(abs(s.psi_dot) for _, s in hist)
    max_psi     = max(abs(s.psi)     for _, s in hist)
    assert max_psi_dot < 1e-12, f"Hub should be at rest; psi_dot={max_psi_dot}"
    assert max_psi     < 1e-12, f"Hub should be at rest; psi={max_psi}"


def test_full_throttle_ccw_capability():
    """
    At steady state, full throttle must produce significant CCW authority above
    the equilibrium point — otherwise disturbance rejection range is too narrow.

    Both states are warmed up to their respective steady-state omega_motor before
    comparing, so the lag transient does not obscure the authority comparison.

    Check: |psi_dot(throttle=1)| > |psi_dot(throttle=eq)| + 10 deg/s
    """
    params = m.HubParams()
    eq     = m.equilibrium_throttle(m.OMEGA_ROTOR_NOMINAL, params)

    state_full = _warm_up(m.OMEGA_ROTOR_NOMINAL, 1.0,  params)
    state_eq   = _warm_up(m.OMEGA_ROTOR_NOMINAL, eq,   params)

    ccw_range = abs(state_full.psi_dot) - abs(state_eq.psi_dot)
    assert ccw_range > math.radians(10.0), (
        f"Full-throttle CCW range = {math.degrees(ccw_range):.1f} deg/s, need > 10 deg/s"
    )


def test_throttle_clamping():
    """
    Throttle values outside [0, 1] must be clamped gracefully.
    psi_dot must equal the clamped result.
    """
    params = m.HubParams()
    for throttle_raw, throttle_clamped in [(-1.0, 0.0), (2.0, 1.0)]:
        s_raw     = m.step(m.HubState(), m.OMEGA_ROTOR_NOMINAL, throttle_raw,     params, DT)
        s_clamped = m.step(m.HubState(), m.OMEGA_ROTOR_NOMINAL, throttle_clamped, params, DT)
        assert abs(s_raw.psi_dot - s_clamped.psi_dot) < 1e-12, (
            f"throttle={throttle_raw} should clamp to {throttle_clamped}; "
            f"psi_dot={s_raw.psi_dot:.6f} vs {s_clamped.psi_dot:.6f}"
        )


def test_equilibrium_throttle_scales_with_rpm():
    """
    Higher axle speed needs proportionally more throttle to maintain psi_dot = 0.
    """
    params  = m.HubParams()
    thr_low = m.equilibrium_throttle(m.OMEGA_ROTOR_NOMINAL * 0.5, params)
    thr_nom = m.equilibrium_throttle(m.OMEGA_ROTOR_NOMINAL,       params)
    thr_hi  = m.equilibrium_throttle(m.OMEGA_ROTOR_NOMINAL * 1.5, params)

    assert thr_low < thr_nom < thr_hi, (
        f"Equilibrium throttle should increase with RPM: "
        f"low={thr_low:.4f}, nom={thr_nom:.4f}, hi={thr_hi:.4f}"
    )


def test_lua_p_controller():
    """
    Mirror of rawes.lua run_yaw_trim() with the current P-only gains.

    Constants must match rawes.lua exactly:
      KP_YAW = 0.001
      feedforward = equilibrium_throttle (mirrors compute_trim)
    """
    KP_YAW = 0.001
    params  = m.HubParams()

    def throttle_fn(state: m.HubState, t: float) -> float:
        trim = m.equilibrium_throttle(m.OMEGA_ROTOR_NOMINAL, params)
        return max(0.0, min(1.0, trim - KP_YAW * state.psi_dot))

    hist = _simulate(
        omega_rotor_fn=lambda t: m.OMEGA_ROTOR_NOMINAL,
        throttle_fn=throttle_fn,
        t_end=60.0,
    )

    late = [(t, s) for t, s in hist if t > 10.0]
    max_psi_dot     = max(abs(s.psi_dot) for _, s in late)
    max_psi_dot_dgs = math.degrees(max_psi_dot)
    max_psi_deg     = math.degrees(max(abs(s.psi) for _, s in late))

    print(f"\n[lua_p] max psi_dot={max_psi_dot_dgs:.3f} deg/s  max psi={max_psi_deg:.2f} deg  (60 s)")
    assert max_psi_dot < math.radians(2.0), (
        f"Lua P controller: settled psi_dot={max_psi_dot_dgs:.3f} deg/s > 2 deg/s"
    )
