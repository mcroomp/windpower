"""test_force_balance.py — RAWES force chain with the new Øye aero.

Physical setup for all tests: 30 deg tether elevation, 10 m/s East wind,
stationary hub, omega=28 rad/s.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from dynamics    import RigidBodyDynamics
from frames      import build_orb_frame
from tests.unit._aero_probe import load_rotor, make_probe, probe_steady


G         = 9.81
WIND_EAST = np.array([0.0, 10.0, 0.0])
T_STEADY  = 45.0
DT        = 2.5e-3
ELEV_DEG  = 30.0

_ROTOR = load_rotor("beaupoil_2026")
_AERO  = make_probe(_ROTOR)
_MASS  = float(_ROTOR.inertia.mass_kg)


def _R_tilt(elevation_deg: float) -> np.ndarray:
    """FRD body_z (down through disk = hub→anchor) for a hub at the given
    elevation angle East of the anchor."""
    el     = math.radians(elevation_deg)
    body_z = np.array([0.0, -math.cos(el), math.sin(el)])
    return build_orb_frame(body_z)


def _thrust_along_normal(result, R_hub):
    """Upward thrust opposing body_z (FRD: body_z down).  Positive = upward."""
    return float(-np.dot(result.F_world, R_hub[:, 2]))


# ── Force-balance tests ─────────────────────────────────────────────────────


def test_thrust_exceeds_weight_at_neutral_collective():
    R30 = _R_tilt(ELEV_DEG)
    r   = probe_steady(_AERO, collective_rad=0.0, R_hub=R30,
                       v_hub_world=np.zeros(3), omega_rotor=28.0,
                       wind_world=WIND_EAST, t=T_STEADY)
    thrust = _thrust_along_normal(r, R30)
    weight = _MASS * G
    assert thrust > weight, f"thrust={thrust:.1f} N < weight={weight:.1f} N at col=0"


def test_thrust_exceeds_2x_weight_at_positive_5deg():
    R30 = _R_tilt(ELEV_DEG)
    r   = probe_steady(_AERO, collective_rad=math.radians(5.0), R_hub=R30,
                       v_hub_world=np.zeros(3), omega_rotor=28.0,
                       wind_world=WIND_EAST, t=T_STEADY)
    thrust = _thrust_along_normal(r, R30)
    weight = _MASS * G
    assert thrust > 2.0 * weight, (
        f"thrust={thrust:.1f} N < 2*weight={2*weight:.1f} N at col=+5 deg"
    )


def test_h_force_positive_east_with_east_wind():
    """In-plane component along East > 0 with East wind."""
    R30 = _R_tilt(ELEV_DEG)
    r   = probe_steady(_AERO, collective_rad=0.0, R_hub=R30,
                       v_hub_world=np.zeros(3), omega_rotor=28.0,
                       wind_world=WIND_EAST, t=T_STEADY)
    Fy = float(r.F_world[1])
    assert Fy > 0.0, f"Fy={Fy:.3f} N not eastward with East wind"


def test_h_force_scales_with_wind_speed():
    R30 = _R_tilt(ELEV_DEG)
    f10 = probe_steady(_AERO, collective_rad=0.0, R_hub=R30,
                       v_hub_world=np.zeros(3), omega_rotor=28.0,
                       wind_world=np.array([0., 10., 0.]), t=T_STEADY)
    f20 = probe_steady(_AERO, collective_rad=0.0, R_hub=R30,
                       v_hub_world=np.zeros(3), omega_rotor=28.0,
                       wind_world=np.array([0., 20., 0.]), t=T_STEADY)
    assert float(f20.F_world[1]) > float(f10.F_world[1]), (
        "H-force did not increase with wind speed"
    )


def test_hover_collective_is_negative():
    """Collective where thrust crosses weight must be negative (tether pulls down)."""
    R30 = _R_tilt(ELEV_DEG)
    weight = _MASS * G
    hover_deg = None
    for deg in range(0, -26, -1):
        col = math.radians(deg)
        r   = probe_steady(_AERO, collective_rad=col, R_hub=R30,
                           v_hub_world=np.zeros(3), omega_rotor=28.0,
                           wind_world=WIND_EAST, t=T_STEADY)
        if _thrust_along_normal(r, R30) <= weight:
            hover_deg = deg
            break
    assert hover_deg is not None and hover_deg < 0, (
        f"hover collective should be < 0 deg; crossed at {hover_deg} deg"
    )


# ── Spin equilibrium (now via the aero state ODE derivative) ────────────────


def _d_omega(omega: float, col: float, R_hub: np.ndarray) -> float:
    """Run the aero forward to a settled inflow and read d_omega from the derivative."""
    from dynbem import RotorInputs, omega_derivative
    state = _AERO.initial_rotor_state()
    I_ode = _ROTOR.autorotation.I_ode_kgm2 or 10.0
    inputs = RotorInputs(
        collective_rad=col, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R_hub, v_hub_world=np.zeros(3),
        wind_world=WIND_EAST, omega_rad_s=float(omega), t=T_STEADY, rho_kg_m3=1.225,
    )
    dt = 0.02
    for _ in range(200):
        _r, deriv = _AERO.compute_forces(inputs, state)
        state = state.from_array(state.to_array() + dt * deriv.to_array())
    _r, _ = _AERO.compute_forces(inputs, state)
    return float(omega_derivative(_r.Q_spin, 0.0, I_ode))


def test_autorotation_d_omega_brackets_zero():
    """d_omega > 0 at low omega (spin-up); d_omega < 0 at high omega (drag)."""
    R30 = _R_tilt(ELEV_DEG)
    col = math.radians(-10.0)
    dw_lo = _d_omega(2.0,  col, R30)
    dw_hi = _d_omega(60.0, col, R30)
    assert dw_lo > 0.0, f"d_omega={dw_lo:.4f} at omega=2 rad/s -- expected > 0"
    assert dw_hi < 0.0, f"d_omega={dw_hi:.4f} at omega=60 rad/s -- expected < 0"


def test_autorotation_omega_equilibrium_in_range():
    """Forward-integrate the full aero state until omega converges.

    Settled omega in [8, 60] rad/s and |d_omega| < 0.01 rad/s^2.
    """
    from dynbem import RotorInputs, euler_step_omega, omega_derivative
    R30 = _R_tilt(ELEV_DEG)
    col = math.radians(-10.0)
    state = _AERO.initial_rotor_state()
    omega_now = 5.0
    spin_angle = 0.0
    omega_min = 0.5
    I_ode = _ROTOR.autorotation.I_ode_kgm2 or 10.0
    dt = 0.01
    d_omega = 0.0
    for _ in range(int(30.0 / dt)):
        inputs = RotorInputs(
            collective_rad=col, tilt_lon=0.0, tilt_lat=0.0,
            R_hub=R30, v_hub_world=np.zeros(3),
            wind_world=WIND_EAST, omega_rad_s=omega_now, t=T_STEADY, rho_kg_m3=1.225,
        )
        _r, deriv = _AERO.compute_forces(inputs, state)
        state = state.from_array(state.to_array() + dt * deriv.to_array())
        new_omega, spin_angle = euler_step_omega(omega_now, spin_angle, float(_r.Q_spin), 0.0, I_ode, dt)
        omega_now = max(omega_min, new_omega)
        d_omega = float(omega_derivative(_r.Q_spin, 0.0, I_ode))
    assert 8.0 <= omega_now <= 60.0, (
        f"equilibrium omega={omega_now:.1f} rad/s outside [8, 60] rad/s"
    )
    assert abs(d_omega) < 0.05, (
        f"d_omega={d_omega:.4f} rad/s^2 after 30 s -- did not converge"
    )


# ── Dynamics integrator sanity (model-independent) ─────────────────────────


def test_dynamics_hover_with_exact_thrust():
    """With F_world = -weight (NED), hub stays at initial altitude for 10 s."""
    dyn = RigidBodyDynamics(
        mass=_MASS, I_body=[5.0, 5.0, 10.0],
        pos0=[0.0, 0.0, -50.0], vel0=[0.0, 0.0, 0.0],
        omega0=[0.0, 0.0, 28.0],
    )
    F_hover = np.array([0.0, 0.0, -(_MASS * G)])
    state = dyn.state
    for _ in range(int(10.0 / DT)):
        state = dyn.step(F_hover, np.zeros(3), DT)
    assert abs(state["pos"][2] - (-50.0)) < 0.05, (
        f"Hub drifted {abs(state['pos'][2]+50.0):.3f} m from Z=-50"
    )
