"""
test_spin_ode.py -- Spin ODE correctness for axial inflow autorotation.

Tests the two public functions extracted from the spin ODE:

  q_spin_from_aero(aero, R_hub)          -> Q_spin [N*m]
  step_spin_ode(omega, Q, I, omega_min, dt) -> omega_new

Two regimes:

1. Axial descent (zero crosswind)
   - Hub descending creates upward axial inflow through horizontal disk
   - q_spin_from_aero must return positive Q at low omega (spin-up torque)
   - The old K_drive*v_inplane formula would give Q~0 here (wrong; v_inplane=0)
   - Note: pure crosswind (horizontal wind, horizontal disk) does NOT drive
     autorotation — flow is in-plane with zero axial component.

2. step_spin_ode integration
   - Positive Q increases omega
   - Negative Q decreases omega
   - omega_min clamp respected

Physical setup:
   disk horizontal: R_hub[:,2] = [0,0,-1] (NED pointing up)
   Peters-He aero only (SkewedWakeBEM invalid at xi=90 deg)
   omega_rotor=1.5 rad/s chosen well below equilibrium for all v_desc values,
   so Q>0 holds even from a cold-start Peters-He inflow state.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rotor_definition as rd
from aero        import create_aero
from frames      import build_orb_frame
from physics_core import q_spin_from_aero, step_spin_ode

R_HORIZONTAL = build_orb_frame(np.array([0., 0., -1.]))
T_STEADY     = 45.0   # s — past Peters-He ramp


def _aero():
    return create_aero(rd.default(), model="peters_he")


# ---------------------------------------------------------------------------
# 1a. Axial descent — Q_spin positive
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("v_desc", [0.5, 1.0, 2.0])
def test_axial_descent_q_spin_positive(v_desc):
    """Downward hub velocity -> upward axial inflow -> positive Q_spin."""
    aero = _aero()
    hub_vel = np.array([0., 0., v_desc])
    aero.compute_forces(
        collective_rad = 0.02,
        tilt_lon = 0.0, tilt_lat = 0.0,
        R_hub          = R_HORIZONTAL,
        v_hub_world    = hub_vel,
        omega_rotor    = 1.5,
        wind_world     = np.zeros(3),
        t              = T_STEADY,
    )
    Q = q_spin_from_aero(aero, R_HORIZONTAL)
    assert Q > 0.0, (
        f"v_desc={v_desc} m/s: Q_spin={Q:.4f} N*m -- "
        f"axial inflow must drive spin-up"
    )


def test_axial_descent_v_inplane_near_zero():
    """With zero crosswind, v_inplane ~ 0 — old K_drive*v_inplane formula would give Q~0."""
    aero = _aero()
    aero.compute_forces(
        collective_rad = 0.02,
        tilt_lon = 0.0, tilt_lat = 0.0,
        R_hub          = R_HORIZONTAL,
        v_hub_world    = np.array([0., 0., 1.0]),
        omega_rotor    = 1.5,
        wind_world     = np.zeros(3),
        t              = T_STEADY,
    )
    assert aero.last_v_inplane < 0.1, (
        f"v_inplane={aero.last_v_inplane:.4f} m/s — expected ~0 for pure axial descent"
    )
    Q = q_spin_from_aero(aero, R_HORIZONTAL)
    assert Q > 0.0, (
        f"Q_spin={Q:.4f} N*m — BEM must give positive torque even when v_inplane~0"
    )


# ---------------------------------------------------------------------------
# 1b. Axial descent — Q_spin increases with descent rate
# ---------------------------------------------------------------------------

def test_axial_descent_q_increases_with_v_desc():
    """Faster descent -> more axial inflow -> larger Q_spin."""
    aero = _aero()
    qs = []
    for v_desc in [0.3, 0.6, 1.0, 1.5]:
        aero.compute_forces(
            collective_rad = 0.02,
            tilt_lon = 0.0, tilt_lat = 0.0,
            R_hub          = R_HORIZONTAL,
            v_hub_world    = np.array([0., 0., v_desc]),
            omega_rotor    = 5.0,
            wind_world     = np.zeros(3),
            t              = T_STEADY,
        )
        qs.append(q_spin_from_aero(aero, R_HORIZONTAL))
    for i in range(len(qs) - 1):
        assert qs[i] < qs[i + 1], (
            f"Q_spin not monotone: v_desc sweep gave {qs}"
        )


# ---------------------------------------------------------------------------
# 2. Crosswind — no autorotation with horizontal disk
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("omega", [2.0, 5.0, 10.0, 15.0])
def test_crosswind_horizontal_disk_no_spinup(omega):
    """Pure crosswind on a horizontal disk never drives spin-up.

    Crosswind is in-plane (zero axial component) so there is no mechanism for
    autorotation — Q_spin must be negative (drag only) at every omega.
    """
    aero = _aero()
    aero.compute_forces(
        collective_rad = 0.0,
        tilt_lon = 0.0, tilt_lat = 0.0,
        R_hub          = R_HORIZONTAL,
        v_hub_world    = np.zeros(3),
        omega_rotor    = omega,
        wind_world     = np.array([0., 10., 0.]),
        t              = T_STEADY,
    )
    Q = q_spin_from_aero(aero, R_HORIZONTAL)
    assert Q < 0.0, (
        f"crosswind, horizontal disk: omega={omega} rad/s "
        f"Q_spin={Q:.4f} N*m — expected negative (drag only, no axial inflow)"
    )


# ---------------------------------------------------------------------------
# 3. step_spin_ode integration
# ---------------------------------------------------------------------------

def test_step_spin_ode_positive_q_increases_omega():
    omega_new = step_spin_ode(omega=10.0, Q_spin=5.0, I_ode_kgm2=10.0,
                               omega_min_rad_s=0.5, dt=0.01)
    assert omega_new > 10.0, f"omega should increase with positive Q, got {omega_new}"


def test_step_spin_ode_negative_q_decreases_omega():
    omega_new = step_spin_ode(omega=10.0, Q_spin=-5.0, I_ode_kgm2=10.0,
                               omega_min_rad_s=0.5, dt=0.01)
    assert omega_new < 10.0, f"omega should decrease with negative Q, got {omega_new}"


def test_step_spin_ode_clamps_to_omega_min():
    omega_new = step_spin_ode(omega=1.0, Q_spin=-1000.0, I_ode_kgm2=10.0,
                               omega_min_rad_s=0.5, dt=1.0)
    assert omega_new == pytest.approx(0.5), (
        f"omega should clamp to omega_min=0.5, got {omega_new}"
    )


def test_step_spin_ode_zero_q_unchanged():
    omega_new = step_spin_ode(omega=10.0, Q_spin=0.0, I_ode_kgm2=10.0,
                               omega_min_rad_s=0.5, dt=0.01)
    assert omega_new == pytest.approx(10.0), (
        f"omega should be unchanged with Q=0, got {omega_new}"
    )


def test_step_spin_ode_correct_euler_step():
    """omega_new = omega + Q/I * dt."""
    omega_new = step_spin_ode(omega=5.0, Q_spin=20.0, I_ode_kgm2=10.0,
                               omega_min_rad_s=0.5, dt=0.5)
    assert omega_new == pytest.approx(5.0 + 20.0 / 10.0 * 0.5)
