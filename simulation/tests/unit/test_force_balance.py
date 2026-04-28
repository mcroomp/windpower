"""
test_force_balance.py — RAWES force chain across rotor/aero-model combinations.

Parametrized by (rotor_name, aero_model):
  beaupoil_2026 / jit        — SkewedWakeBEMJit (production, xi < 85°)
  beaupoil_2026 / peters_he  — Peters-He 5-state (all xi, required for descent)

Physical setup for all tests: 30° tether elevation, 10 m/s East wind, stationary hub.

Tests
-----
Force balance
  thrust > weight at neutral collective
  thrust > 2×weight at +5° collective
  H-force positive eastward with East wind
  H-force scales with wind speed
  hover collective is negative

Omega equilibrium
  Q_spin > 0 at omega << equilibrium  ]  bracket proves autorotation
  Q_spin < 0 at omega >> equilibrium  ]  equilibrium exists in [lo, hi]
  equilibrium omega converges into [10, 60] rad/s after integrating ODE
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aero        import create_aero
import rotor_definition as rd
from dynamics    import RigidBodyDynamics
from frames      import build_orb_frame
from physics_core import q_spin_from_aero

# ── Parametrization ───────────────────────────────────────────────────────────

CASES = [
    pytest.param("beaupoil_2026", "jit",       id="beaupoil/jit"),
    pytest.param("beaupoil_2026", "peters_he", id="beaupoil/peters_he"),
]

# ── Shared constants ──────────────────────────────────────────────────────────

G         = 9.81
WIND_EAST = np.array([0.0, 10.0, 0.0])   # NED: 10 m/s East
T_STEADY  = 45.0                           # s — past ramp
DT        = 2.5e-3                         # s — 400 Hz
ELEV_DEG  = 30.0                          # tether elevation for force-balance tests


def _R_tilt(elevation_deg: float) -> np.ndarray:
    """Orbital frame for disk normal aligned with tether at given elevation."""
    el     = math.radians(elevation_deg)
    body_z = np.array([0.0, math.cos(el), -math.sin(el)])
    return build_orb_frame(body_z)


# ── Force-balance tests ───────────────────────────────────────────────────────


@pytest.mark.parametrize("rotor_name,model", CASES)
def test_thrust_exceeds_weight_at_neutral_collective(rotor_name, model):
    """Thrust > weight at col=0, 30° tilt, 10 m/s East wind, omega=28 rad/s."""
    rotor  = rd.load(rotor_name)
    weight = rotor.dynamics_kwargs()["mass"] * G
    aero   = create_aero(rotor, model=model)
    R30    = _R_tilt(ELEV_DEG)

    f = aero.compute_forces(0.0, 0.0, 0.0, R30, np.zeros(3), 28.0, WIND_EAST, t=T_STEADY)
    thrust = float(np.dot(f.F_world, R30[:, 2]))   # along disk normal = upward

    assert thrust > weight, (
        f"[{rotor_name}/{model}] thrust={thrust:.1f} N < weight={weight:.1f} N at col=0"
    )


@pytest.mark.parametrize("rotor_name,model", CASES)
def test_thrust_exceeds_2x_weight_at_positive_5deg(rotor_name, model):
    """At +5° collective the surplus thrust >> weight (creates tether tension)."""
    rotor  = rd.load(rotor_name)
    weight = rotor.dynamics_kwargs()["mass"] * G
    aero   = create_aero(rotor, model=model)
    R30    = _R_tilt(ELEV_DEG)

    col = math.radians(5.0)
    f   = aero.compute_forces(col, 0.0, 0.0, R30, np.zeros(3), 28.0, WIND_EAST, t=T_STEADY)
    thrust = float(np.dot(f.F_world, R30[:, 2]))

    assert thrust > weight * 2.0, (
        f"[{rotor_name}/{model}] thrust={thrust:.1f} N < 2×weight={2*weight:.1f} N at col=+5°"
    )


@pytest.mark.parametrize("rotor_name,model", CASES)
def test_h_force_positive_east_with_east_wind(rotor_name, model):
    """H-force pushes hub East (NED Y > 0) when wind blows East."""
    rotor = rd.load(rotor_name)
    aero  = create_aero(rotor, model=model)
    R30   = _R_tilt(ELEV_DEG)

    f  = aero.compute_forces(0.0, 0.0, 0.0, R30, np.zeros(3), 28.0, WIND_EAST, t=T_STEADY)
    Fy = float(f.F_world[1])   # NED Y = East

    assert Fy > 0.0, (
        f"[{rotor_name}/{model}] H-force Fy={Fy:.3f} N not eastward with East wind"
    )


@pytest.mark.parametrize("rotor_name,model", CASES)
def test_h_force_scales_with_wind_speed(rotor_name, model):
    """H-force at 20 m/s wind > H-force at 10 m/s wind."""
    rotor = rd.load(rotor_name)
    aero  = create_aero(rotor, model=model)
    R30   = _R_tilt(ELEV_DEG)

    f10 = aero.compute_forces(0.0, 0.0, 0.0, R30, np.zeros(3), 28.0,
                               np.array([0., 10., 0.]), t=T_STEADY)
    f20 = aero.compute_forces(0.0, 0.0, 0.0, R30, np.zeros(3), 28.0,
                               np.array([0., 20., 0.]), t=T_STEADY)

    assert float(f20.F_world[1]) > float(f10.F_world[1]), (
        f"[{rotor_name}/{model}] H-force did not increase with wind speed"
    )


@pytest.mark.parametrize("rotor_name,model", CASES)
def test_hover_collective_is_negative(rotor_name, model):
    """Collective that makes thrust = weight must be negative (RAWES tether must pull down)."""
    rotor  = rd.load(rotor_name)
    weight = rotor.dynamics_kwargs()["mass"] * G
    aero   = create_aero(rotor, model=model)
    R30    = _R_tilt(ELEV_DEG)

    hover_deg = None
    for deg in range(0, -26, -1):
        col = math.radians(deg)
        f   = aero.compute_forces(col, 0.0, 0.0, R30, np.zeros(3), 28.0, WIND_EAST, t=T_STEADY)
        if float(np.dot(f.F_world, R30[:, 2])) <= weight:
            hover_deg = deg
            break

    assert hover_deg is not None and hover_deg < 0, (
        f"[{rotor_name}/{model}] hover collective should be < 0°; crossed at {hover_deg}°"
    )


# ── Omega equilibrium tests ───────────────────────────────────────────────────


@pytest.mark.parametrize("rotor_name,model", CASES)
def test_autorotation_q_brackets_zero(rotor_name, model):
    """
    At 30° tilt, 10 m/s East wind, equilibrium collective (~-0.15 rad):
      Q_spin > 0 at omega=2 rad/s   (wind drives spin-up)
      Q_spin < 0 at omega=60 rad/s  (drag dominates)
    → autorotation equilibrium must exist in [2, 60] rad/s.
    """
    rotor = rd.load(rotor_name)
    aero  = create_aero(rotor, model=model)
    R30   = _R_tilt(ELEV_DEG)
    col   = math.radians(-10.0)   # typical tethered flight collective

    def q_at(omega: float) -> float:
        aero.compute_forces(col, 0.0, 0.0, R30, np.zeros(3), omega, WIND_EAST, t=T_STEADY)
        return q_spin_from_aero(aero, R30)

    q_lo = q_at(2.0)
    q_hi = q_at(60.0)

    assert q_lo > 0.0, (
        f"[{rotor_name}/{model}] Q_spin={q_lo:.4f} N·m at omega=2 rad/s — expected > 0 (spin-up)"
    )
    assert q_hi < 0.0, (
        f"[{rotor_name}/{model}] Q_spin={q_hi:.4f} N·m at omega=60 rad/s — expected < 0 (drag)"
    )


@pytest.mark.parametrize("rotor_name,model", CASES)
def test_autorotation_omega_equilibrium_in_range(rotor_name, model):
    """
    Integrate the spin ODE from omega=5 rad/s until omega converges.
    The settled value must lie in [8, 60] rad/s and |Q| < 0.01 N*m.

    Catches models that predict no autorotation (omega -> 0) or runaway
    (omega -> inf). Equilibrium is reached monotonically (Q asymptotes to 0).
    """
    rotor  = rd.load(rotor_name)
    dk     = rotor.dynamics_kwargs()
    I_spin = dk["I_spin"]
    aero   = create_aero(rotor, model=model)
    R30    = _R_tilt(ELEV_DEG)
    col    = math.radians(-10.0)

    omega     = 5.0
    omega_min = 0.5
    dt        = 0.01    # 100 Hz — stable, fast
    t_max     = 30.0    # s — ample (this rotor converges in ~3 s)
    steps     = int(t_max / dt)
    Q         = 0.0

    for _ in range(steps):
        aero.compute_forces(col, 0.0, 0.0, R30, np.zeros(3), omega, WIND_EAST, t=T_STEADY)
        if not aero.is_valid():
            break
        Q     = q_spin_from_aero(aero, R30)
        omega = max(omega_min, omega + Q / I_spin * dt)

    assert 8.0 <= omega <= 60.0, (
        f"[{rotor_name}/{model}] equilibrium omega={omega:.1f} rad/s outside [8, 60] rad/s"
    )
    assert abs(Q) < 0.01, (
        f"[{rotor_name}/{model}] Q={Q:.4f} N*m after {t_max} s -- ODE did not converge"
    )


# ── Dynamics integrator sanity (not aero-model-specific) ─────────────────────


def test_dynamics_hover_with_exact_thrust():
    """With F_world = weight (exact), hub stays at initial altitude for 10 s."""
    mass  = rd.load("beaupoil_2026").dynamics_kwargs()["mass"]
    dyn   = RigidBodyDynamics(
        mass=mass, I_body=[5.0, 5.0, 10.0],
        pos0=[0.0, 0.0, -50.0], vel0=[0.0, 0.0, 0.0],
        omega0=[0.0, 0.0, 28.0],
    )
    F_hover = np.array([0.0, 0.0, -(mass * G)])   # NED: upward = -Z
    for _ in range(int(10.0 / DT)):
        state = dyn.step(F_hover, np.zeros(3), DT)

    assert abs(state["pos"][2] - (-50.0)) < 0.05, (
        f"Hub drifted {abs(state['pos'][2]+50.0):.3f} m from NED Z=-50 with exact hover thrust"
    )
