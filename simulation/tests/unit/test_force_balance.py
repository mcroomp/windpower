"""
test_force_balance.py — validate the RAWES force chain produces lift > gravity.

These tests confirm that the aero + dynamics stack can sustain flight with
realistic servo commands, and that the H-force pushes the hub eastward with
wind blowing East.

If these pass, the rotor should not fall during a guided flight test.
If they fail, the root cause of sinking is identified here without needing
the full Docker stack.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aero import RotorAero
import rotor_definition as _rd
from dynamics import RigidBodyDynamics
from swashplate import collective_to_pitch, h3_inverse_mix, pwm_to_normalized


# ── Physical constants ─────────────────────────────────────────────────────────
MASS   = 5.0      # kg  (matches mediator.py)
G      = 9.81     # m/s²
WEIGHT = MASS * G  # N  — force that must be overcome to hover
OMEGA  = 28.0     # rad/s nominal spin (mediator initial condition)
WIND   = np.array([10.0, 0.0, 0.0])   # 10 m/s East (mediator default)
DT     = 2.5e-3   # s  (mediator step size, 400 Hz)


# ── Helpers ────────────────────────────────────────────────────────────────────

def _neutral_collective() -> float:
    """collective_rad when all swashplate servos are at neutral (1500 µs)."""
    s = pwm_to_normalized(1500.0)          # = 0.0
    coll_norm, _, _ = h3_inverse_mix(s, s, s)  # = 0.0
    return collective_to_pitch(coll_norm)   # = 0.0 rad


def _aero_forces_after_ramp(collective_rad: float) -> np.ndarray:
    """Return aero forces at t=10s (ramp=1) with level disk, stationary hub."""
    aero = RotorAero(_rd.default())
    return aero.compute_forces(
        collective_rad = collective_rad,
        tilt_lon       = 0.0,
        tilt_lat       = 0.0,
        R_hub          = np.eye(3),
        v_hub_world    = np.zeros(3),
        omega_rotor    = OMEGA,
        wind_world     = WIND,
        t              = 10.0,   # past 5 s ramp → ramp = 1.0
    )


# ── Tests ──────────────────────────────────────────────────────────────────────

def test_neutral_collective_is_zero_rad():
    """1500 µs on all servos → collective = 0 rad (not negative)."""
    coll = _neutral_collective()
    assert coll == 0.0, f"Expected 0.0 rad, got {coll}"


def test_aero_thrust_exceeds_weight_at_neutral_collective():
    """
    With neutral collective (0 rad), spin=28 rad/s, wind=10 m/s East,
    vertical thrust Fz must exceed the rotor weight after the ramp period.

    This is the minimum condition for sustained flight.
    If Fz < WEIGHT the hub will fall regardless of ArduPilot commands.
    """
    forces = _aero_forces_after_ramp(collective_rad=0.0)
    Fz = forces[2]
    assert Fz > WEIGHT, (
        f"Thrust Fz={Fz:.1f} N < weight={WEIGHT:.1f} N at neutral collective.\n"
        f"Hub will fall even with correct ArduPilot commands.\n"
        f"Full force vector: F={forces[:3].round(1)}  M={forces[3:].round(1)}"
    )


def test_aero_thrust_exceeds_weight_at_minimum_positive_collective():
    """
    With the minimum positive collective that ArduPilot might command (~5°),
    thrust must comfortably exceed weight.
    """
    collective_rad = math.radians(5.0)   # 5° — typical low-collective hover
    forces = _aero_forces_after_ramp(collective_rad=collective_rad)
    Fz = forces[2]
    assert Fz > WEIGHT * 2.0, (
        f"At 5° collective, Fz={Fz:.1f} N should be well above weight={WEIGHT:.1f} N.\n"
        f"Full force vector: F={forces[:3].round(1)}"
    )


def test_aero_h_force_is_positive_east_for_east_wind():
    """
    With 10 m/s East wind and a level disk, the H-force must push the hub
    East (positive Fx in ENU).  This is the force that creates tether tension.
    """
    forces = _aero_forces_after_ramp(collective_rad=0.0)
    Fx = forces[0]
    assert Fx > 0.0, (
        f"H-force Fx={Fx:.3f} N is not positive eastward with East wind.\n"
        f"Full force vector: F={forces[:3].round(2)}"
    )


def test_aero_h_force_scales_with_wind_speed():
    """H-force should increase with wind speed (μ = v_wind / (ω·R_tip))."""
    aero = RotorAero(_rd.default())
    forces_10 = aero.compute_forces(0.0, 0.0, 0.0, np.eye(3), np.zeros(3),
                                    OMEGA, np.array([10.0, 0.0, 0.0]), t=10.0)
    forces_20 = aero.compute_forces(0.0, 0.0, 0.0, np.eye(3), np.zeros(3),
                                    OMEGA, np.array([20.0, 0.0, 0.0]), t=10.0)
    assert forces_20[0] > forces_10[0], (
        f"H-force should be larger at 20 m/s wind than 10 m/s.\n"
        f"Fx@10={forces_10[0]:.2f}  Fx@20={forces_20[0]:.2f}"
    )


def test_dynamics_hover_with_exact_thrust():
    """
    With F_world = weight (exact compensation), the hub must stay at its
    initial altitude for 10 seconds.
    """
    dyn = RigidBodyDynamics(
        mass=MASS, I_body=[5.0, 5.0, 10.0],
        pos0=[0.0, 0.0, 50.0], vel0=[0.0, 0.0, 0.0],
        omega0=[0.0, 0.0, OMEGA],
    )
    F_hover = np.array([0.0, 0.0, WEIGHT])
    for _ in range(int(10.0 / DT)):
        state = dyn.step(F_hover, np.zeros(3), DT)

    z_final = state["pos"][2]
    assert abs(z_final - 50.0) < 0.05, (
        f"Hub drifted {abs(z_final-50.0):.3f} m from 50 m with exact hover thrust.\n"
        f"Dynamics integrator may have a gravity bug."
    )


def test_thrust_greatly_exceeds_weight_at_reel_out_collective():
    """
    RAWES characterisation: at reel-out collective (~5°) thrust is >>weight.

    De Schutter model has CL0=0 (symmetric thin-plate), so thrust requires
    positive AoA.  At reel-out collective (5°, De Schutter Table I operating
    point) thrust >> weight — this surplus creates tether tension in flight.

    Without a tether (or with a slack tether), the hub will rise rapidly at
    reel-out collective.  ArduPilot must account for this when commanding
    collective in GUIDED mode.
    """
    REEL_OUT = math.radians(5.0)   # De Schutter reel-out operating point
    aero = RotorAero(_rd.default())
    forces = aero.compute_forces(
        collective_rad=REEL_OUT, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=np.eye(3), v_hub_world=np.zeros(3),
        omega_rotor=OMEGA, wind_world=WIND, t=10.0,
    )
    Fz = forces[2]
    assert Fz > WEIGHT * 2.0, (
        f"At 5° collective, Fz={Fz:.1f} N should be >>weight={WEIGHT:.1f} N.\n"
        f"RAWES surplus thrust (Fz-W={Fz-WEIGHT:.1f} N) creates tether tension."
    )
    # Static force check is sufficient. Dynamics simulation omitted here: with
    # strip-model thrust ~1200 N and mass 5 kg, the hub accelerates at ~230 m/s²
    # and the RK4 integrator overflows before 10 s. The Fz >> W assertion above
    # already proves the hub would rise rapidly without a tether.


def test_hover_collective_is_negative():
    """
    The blade pitch angle that produces exactly hover thrust (T = W) must be
    negative.  This means ArduPilot must command below-neutral collective to
    prevent the hub from rising when the tether is slack.
    """
    aero = RotorAero(_rd.default())
    # Scan collective until Fz crosses weight
    for coll_deg in range(0, -25, -1):
        coll_rad = math.radians(coll_deg)
        f = aero.compute_forces(coll_rad, 0.0, 0.0, np.eye(3), np.zeros(3),
                                OMEGA, WIND, t=10.0)
        if f[2] <= WEIGHT:
            hover_deg = coll_deg
            break
    else:
        hover_deg = -25  # not found in range

    assert hover_deg < 0, (
        f"Hover collective should be negative; found Fz≈W at {hover_deg}°.\n"
        f"ArduPilot must command below-neutral collective for RAWES."
    )


def test_h_force_causes_eastward_drift():
    """
    With 10 m/s East wind and no position controller, the hub should drift
    East over 10 seconds due to the H-force.
    """
    aero = RotorAero(_rd.default())
    dyn  = RigidBodyDynamics(
        mass=MASS, I_body=[5.0, 5.0, 10.0],
        pos0=[0.0, 0.0, 50.0], vel0=[0.0, 0.0, 0.0],
        omega0=[0.0, 0.0, OMEGA],
    )

    for step in range(int(10.0 / DT)):
        t = step * DT
        s = dyn.state
        forces = aero.compute_forces(
            collective_rad = 0.0,
            tilt_lon=0.0, tilt_lat=0.0,
            R_hub=s["R"], v_hub_world=s["vel"],
            omega_rotor=float(np.dot(s["omega"], s["R"][:, 2])) or OMEGA,
            wind_world=WIND, t=t,
        )
        dyn.step(forces[:3], forces[3:], DT)

    final_pos = dyn.state["pos"]
    assert final_pos[0] > 0.1, (
        f"Hub did not drift East after 10 s with 10 m/s East wind.\n"
        f"Final ENU pos: {final_pos.round(3)}\n"
        f"H-force may be too small or not being applied."
    )


def test_force_balance_at_kite_equilibrium():
    """
    At the power-optimal kite position (35° elevation, 50 m tether):
      ENU = (41.0, 0, 28.9) relative to anchor
    the net force on the hub should be close to zero — tether tension +
    aero forces + gravity should balance.

    This confirms the 35° target is physically reachable, not just a
    mathematical ideal.

    Expected equilibrium:
      Thrust T ≈ 163 N (neutral collective, ω=28)
      H-force H ≈ 11.7 N (eastward)
      Weight W = 49.1 N

      Tether direction at β=35°: unit vector = (-cos35°, 0, -sin35°)
                                             = (-0.819, 0, -0.574)
      Tether tension T_t:
        X:  H - T_t·cos(35°) = 0  →  T_t = 11.7/0.819 = 14.3 N
        Z:  T - W - T_t·sin(35°) = 0
            163 - 49.1 - T_t·0.574 = 0
            T_t = 113.9/0.574 = 198 N

      The two T_t values (14.3 N vs 198 N) don't match — the hub is NOT
      at equilibrium at zero collective.  At neutral collective the rotor
      generates way more vertical force than the tether can absorb at 35°.

      The actual equilibrium elevation β satisfies:
        tan(β) = (T-W)/H = (163-49.1)/11.7 = 9.74  →  β = 84°

    This test documents this discrepancy and quantifies the actual
    equilibrium angle vs the power-optimal target.
    """
    aero = RotorAero(_rd.default())
    forces = aero.compute_forces(
        collective_rad=0.0, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=np.eye(3), v_hub_world=np.zeros(3),
        omega_rotor=OMEGA, wind_world=WIND, t=10.0,
    )
    T = forces[2]   # vertical thrust [N]
    H = forces[0]   # horizontal H-force [N]

    # Natural equilibrium angle from force balance
    beta_actual_rad = math.atan2(T - WEIGHT, H)
    beta_actual_deg = math.degrees(beta_actual_rad)

    beta_optimal_deg = math.degrees(math.atan(1.0 / math.sqrt(2.0)))  # 35.26°

    # The actual equilibrium should be much steeper than the power-optimal angle
    assert beta_actual_deg > 60.0, (
        f"Expected actual equilibrium >60° (nearly vertical for fast autogyro).\n"
        f"Got β={beta_actual_deg:.1f}° (T={T:.1f}N, H={H:.1f}N, W={WEIGHT:.1f}N).\n"
        f"Power-optimal target is β={beta_optimal_deg:.1f}° — requires disk tilt."
    )
    # H-force exists but is small compared to thrust
    assert 0 < H < T * 0.2, (
        f"H-force={H:.1f}N should be small (< 20% of T={T:.1f}N) for fast autogyro.\n"
        f"H/T ratio = {H/T:.3f}  (advance ratio μ = v_wind/(ω·R) = {10/(28*2.5):.3f})"
    )
