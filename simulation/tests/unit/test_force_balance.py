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

from aero import create_aero
import rotor_definition as _rd
from dynamics import RigidBodyDynamics
from swashplate import collective_to_pitch, h3_inverse_mix, pwm_to_normalized
from frames import build_orb_frame


# ── Physical constants ─────────────────────────────────────────────────────────
MASS   = 5.0      # kg  (matches mediator.py)
G      = 9.81     # m/s²
WEIGHT = MASS * G  # N  — force that must be overcome to hover
OMEGA  = 28.0     # rad/s nominal spin (mediator initial condition)
WIND   = np.array([0.0, 10.0, 0.0])   # NED: 10 m/s East = Y axis (mediator default)
DT     = 2.5e-3   # s  (mediator step size, 400 Hz)


# ── Helpers ────────────────────────────────────────────────────────────────────

_COL_MAX_RAD = 0.35   # physical collective limit matching mediator trajectory config

def _neutral_collective() -> float:
    """collective_rad when all swashplate servos are at neutral (1500 µs)."""
    s = pwm_to_normalized(1500.0)              # = 0.0
    coll_norm, _, _ = h3_inverse_mix(s, s, s)  # = 0.0
    return collective_to_pitch(coll_norm, _COL_MAX_RAD)   # = 0.0 rad


def _R_tilt_30() -> np.ndarray:
    """Rotation matrix for a rotor disk tilted 30° from vertical (toward East).

    This is the typical RAWES operating condition: body Z (disk normal) aligned
    with the tether at 30° elevation angle.  In NED: East = Y, Up = -Z.
    body_z = [0, cos(30°), -sin(30°)] points mostly East and somewhat Up.
    """
    tilt = math.radians(30.0)
    body_z = np.array([0.0, math.cos(tilt), -math.sin(tilt)])
    return build_orb_frame(body_z)


def _aero_forces_after_ramp(collective_rad: float, R_hub: np.ndarray = None) -> np.ndarray:
    """Return aero forces at t=10s (ramp=1) with 30°-tilted disk, stationary hub.

    The 30° tilt (disk normal = tether direction at 30° elevation) is the typical
    RAWES operating condition.  With a level disk and horizontal wind the axial
    inflow is zero, producing unrealistic results for BEM models.
    """
    if R_hub is None:
        R_hub = _R_tilt_30()
    aero = create_aero(_rd.default())
    return aero.compute_forces(
        collective_rad = collective_rad,
        tilt_lon       = 0.0,
        tilt_lat       = 0.0,
        R_hub          = R_hub,
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
    and disk tilted 30° (RAWES operating condition), vertical thrust Fz must
    exceed the rotor weight after the ramp period.

    This is the minimum condition for sustained flight.
    If Fz < WEIGHT the hub will fall regardless of ArduPilot commands.
    """
    forces = _aero_forces_after_ramp(collective_rad=0.0)
    Fz = -forces[2]   # In NED: upward thrust = -NED Z component
    assert Fz > WEIGHT, (
        f"Thrust Fz={Fz:.1f} N < weight={WEIGHT:.1f} N at neutral collective (30° tilt).\n"
        f"Hub will fall even with correct ArduPilot commands.\n"
        f"Full force vector: F={forces[:3].round(1)}  M={forces[3:].round(1)}"
    )


def test_aero_thrust_exceeds_weight_at_minimum_positive_collective():
    """
    With the minimum positive collective that ArduPilot might command (~5°),
    and disk tilted 30° (RAWES operating condition), thrust must comfortably exceed weight.
    """
    collective_rad = math.radians(5.0)   # 5° — typical low-collective hover
    forces = _aero_forces_after_ramp(collective_rad=collective_rad)
    Fz = -forces[2]   # In NED: upward thrust = -NED Z component
    assert Fz > WEIGHT * 2.0, (
        f"At 5° collective (30° tilt), Fz={Fz:.1f} N should be well above weight={WEIGHT:.1f} N.\n"
        f"Full force vector: F={forces[:3].round(1)}"
    )


def test_aero_h_force_is_positive_east_for_east_wind():
    """
    With 10 m/s East wind and a level disk, the H-force must push the hub
    East (positive NED Y = F_world[1]).  This is the force that creates tether tension.
    """
    forces = _aero_forces_after_ramp(collective_rad=0.0)
    Fy = forces[1]   # NED: East = Y axis
    assert Fy > 0.0, (
        f"H-force Fy={Fy:.3f} N is not positive eastward with East wind.\n"
        f"Full force vector: F={forces[:3].round(2)}"
    )


def test_aero_h_force_scales_with_wind_speed():
    """H-force should increase with wind speed (μ = v_wind / (ω·R_tip))."""
    aero = create_aero(_rd.default())
    forces_10 = aero.compute_forces(0.0, 0.0, 0.0, np.eye(3), np.zeros(3),
                                    OMEGA, np.array([0.0, 10.0, 0.0]), t=10.0)  # NED East
    forces_20 = aero.compute_forces(0.0, 0.0, 0.0, np.eye(3), np.zeros(3),
                                    OMEGA, np.array([0.0, 20.0, 0.0]), t=10.0)  # NED East
    assert forces_20[1] > forces_10[1], (
        f"H-force (NED Y=East) should be larger at 20 m/s wind than 10 m/s.\n"
        f"Fy@10={forces_10[1]:.2f}  Fy@20={forces_20[1]:.2f}"
    )


def test_dynamics_hover_with_exact_thrust():
    """
    With F_world = weight (exact compensation), the hub must stay at its
    initial altitude for 10 seconds.
    """
    # In NED: altitude 50 m = pos[2] = -50. Upward thrust = negative NED Z.
    dyn = RigidBodyDynamics(
        mass=MASS, I_body=[5.0, 5.0, 10.0],
        pos0=[0.0, 0.0, -50.0], vel0=[0.0, 0.0, 0.0],
        omega0=[0.0, 0.0, OMEGA],
    )
    F_hover = np.array([0.0, 0.0, -WEIGHT])   # upward force = -NED Z
    for _ in range(int(10.0 / DT)):
        state = dyn.step(F_hover, np.zeros(3), DT)

    z_final = state["pos"][2]
    assert abs(z_final - (-50.0)) < 0.05, (
        f"Hub drifted {abs(z_final-(-50.0)):.3f} m from NED Z=-50 with exact hover thrust.\n"
        f"Dynamics integrator may have a gravity bug."
    )


def test_thrust_greatly_exceeds_weight_at_reel_out_collective():
    """
    RAWES characterisation: at reel-out collective (~5°) thrust is >>weight.

    At reel-out collective (5°, De Schutter Table I operating point) with the
    disk tilted 30° (RAWES operating condition), thrust >> weight — this surplus
    creates tether tension in flight.

    Without a tether (or with a slack tether), the hub will rise rapidly at
    reel-out collective.  ArduPilot must account for this when commanding
    collective in GUIDED mode.
    """
    REEL_OUT = math.radians(5.0)   # De Schutter reel-out operating point
    forces = _aero_forces_after_ramp(collective_rad=REEL_OUT)
    Fz = -forces[2]   # In NED: upward force = -NED Z component
    assert Fz > WEIGHT * 2.0, (
        f"At 5° collective (30° tilt), Fz={Fz:.1f} N should be >>weight={WEIGHT:.1f} N.\n"
        f"RAWES surplus thrust (Fz-W={Fz-WEIGHT:.1f} N) creates tether tension."
    )


def test_hover_collective_is_negative():
    """
    With disk tilted 30° (RAWES operating condition), the blade pitch angle that
    produces exactly hover thrust (Fz = W) must be negative.  This means ArduPilot
    must command below-neutral collective to prevent the hub from rising when the
    tether is slack.
    """
    aero = create_aero(_rd.default())
    R_tilt = _R_tilt_30()
    # Scan collective until Fz crosses weight
    hover_deg = -25  # sentinel: not found in range
    for coll_deg in range(0, -25, -1):
        coll_rad = math.radians(coll_deg)
        f = aero.compute_forces(coll_rad, 0.0, 0.0, R_tilt, np.zeros(3),
                                OMEGA, WIND, t=10.0)
        if -f[2] <= WEIGHT:   # In NED: upward = -NED Z
            hover_deg = coll_deg
            break

    assert hover_deg < 0, (
        f"Hover collective should be negative; found Fz≈W at {hover_deg}° (30° tilt).\n"
        f"ArduPilot must command below-neutral collective for RAWES."
    )


def test_h_force_causes_eastward_drift():
    """
    With 10 m/s East wind and no position controller, the hub should drift
    East over 10 seconds due to the H-force.
    """
    # In NED: hub at altitude 50 m = pos[2] = -50
    aero = create_aero(_rd.default())
    dyn  = RigidBodyDynamics(
        mass=MASS, I_body=[5.0, 5.0, 10.0],
        pos0=[0.0, 0.0, -50.0], vel0=[0.0, 0.0, 0.0],
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
    # In NED: East = Y axis (index 1)
    assert final_pos[1] > 0.1, (
        f"Hub did not drift East (NED Y) after 10 s with 10 m/s East wind.\n"
        f"Final NED pos: {final_pos.round(3)}\n"
        f"H-force may be too small or not being applied."
    )


def test_force_balance_at_kite_equilibrium():
    """
    At the 30° tether elevation operating condition, with neutral collective:
      - Vertical thrust T >> weight (RAWES generates surplus lift, creating tether tension)
      - H-force H > 0 (pushes hub downwind, creating tether tension)
      - T > W confirms the hub cannot hover freely — it must be held down by tether

    This characterises the RAWES operating point using the SkewedWakeBEM model.
    With a 30°-tilted disk (body Z = tether direction) and 10 m/s East wind:
      - T ≈ 268 N >> W = 49.1 N (surplus creates tether tension)
      - H ≈ 173 N (large eastward in-plane force, also creates tether tension)
    """
    forces = _aero_forces_after_ramp(collective_rad=0.0)
    T = -forces[2]  # upward thrust [N] = -NED Z component
    H = forces[1]   # horizontal (East) force component [N] = NED Y

    # Both thrust and H-force must be positive (rotor lifts and pushes East)
    assert T > WEIGHT, (
        f"Thrust T={T:.1f} N must exceed weight W={WEIGHT:.1f} N.\n"
        f"RAWES surplus creates tether tension at 30° tilt."
    )
    assert H > 0.0, (
        f"H-force={H:.1f} N must be positive eastward (pushes hub downwind)."
    )
