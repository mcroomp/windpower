"""test_dynamics_physics.py — invariants of the 6-DOF rigid-body integrator.

These tests lock down the physical contract of ``RigidBodyDynamics``
independent of aero or controller code:

  * free fall under gravity
  * angular momentum conservation (no torque, no spin)
  * gyroscopic precession (spinning rotor under a transverse torque)
  * gyroscopic precession sign with reversed spin
  * no spin ⇒ no precession (control case)
  * orientation kinematics (constant ω about a body axis rotates R correctly)

Frames: NED world + FRD body throughout.  body_z = R[:,2] points DOWN
through the rotor disk for a level hover.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from dynamics import RigidBodyDynamics


# ── shared constants ────────────────────────────────────────────────────────

MASS    = 5.0                       # kg
I_BODY  = [5.0, 5.0, 10.0]          # kg·m²  Ixx, Iyy, Izz
I_SPIN  = 4.0                       # kg·m²  rotor spin-axis inertia
G       = 9.81

# Small dt for low integration error; many steps for short physical time.
DT      = 1.0e-3
N_STEPS_1S = int(round(1.0 / DT))


def _fresh(*, I_spin: float = 0.0,
           R0: np.ndarray = None,
           omega0=None,
           pos0=(0.0, 0.0, -50.0),
           z_floor: float = None) -> RigidBodyDynamics:
    """Build a clean dynamics instance with no floor by default."""
    return RigidBodyDynamics(
        mass=MASS,
        I_body=I_BODY,
        I_spin=I_spin,
        pos0=list(pos0),
        vel0=[0.0, 0.0, 0.0],
        R0=np.eye(3) if R0 is None else R0,
        omega0=list(omega0) if omega0 is not None else [0.0, 0.0, 0.0],
        z_floor=z_floor,
    )


# ── 1. Free fall ────────────────────────────────────────────────────────────


def test_free_fall_matches_g():
    """No forces, no spin ⇒ hub falls at g in +Z (NED Down)."""
    dyn = _fresh()
    F0  = np.zeros(3)
    M0  = np.zeros(3)
    t   = 1.0
    for _ in range(int(round(t / DT))):
        s = dyn.step(F0, M0, DT)
    # z = z0 + 0.5*g*t^2 ; v_z = g*t
    expected_z  = -50.0 + 0.5 * G * t**2
    expected_vz = G * t
    assert s["pos"][2] == pytest.approx(expected_z,  abs=1e-3)
    assert s["vel"][2] == pytest.approx(expected_vz, abs=1e-3)
    # No horizontal motion / rotation
    np.testing.assert_allclose(s["pos"][:2], [0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(s["omega"],   [0.0, 0.0, 0.0], atol=1e-9)


def test_force_exactly_cancels_gravity_holds_altitude():
    """F = -m·g·ẑ_world (i.e. F[2] = -m·g) holds the hub for arbitrarily long."""
    dyn   = _fresh()
    F_up  = np.array([0.0, 0.0, -MASS * G])   # NED: upward in world frame
    for _ in range(N_STEPS_1S * 5):           # 5 s
        s = dyn.step(F_up, np.zeros(3), DT)
    assert s["pos"][2] == pytest.approx(-50.0, abs=1e-6)
    assert s["vel"][2] == pytest.approx(0.0,   abs=1e-6)


# ── 2. Angular momentum conservation (no spin, no torque) ──────────────────


def test_no_spin_no_torque_holds_attitude():
    """ω₀ = 0 and τ = 0 ⇒ R stays at identity forever."""
    dyn = _fresh()
    F_up = np.array([0.0, 0.0, -MASS * G])
    for _ in range(N_STEPS_1S * 2):
        s = dyn.step(F_up, np.zeros(3), DT)
    np.testing.assert_allclose(s["R"], np.eye(3), atol=1e-9)
    np.testing.assert_allclose(s["omega"], [0.0, 0.0, 0.0], atol=1e-9)


def test_constant_body_rate_about_z_rotates_R_correctly():
    """ω = [0,0,Ω_z] world ⇒ R(t) = Rz(Ω_z·t)."""
    Ω_z = 1.0   # rad/s about world Z (down)
    dyn = _fresh(omega0=[0.0, 0.0, Ω_z])
    F_up = np.array([0.0, 0.0, -MASS * G])
    t = 0.5
    for _ in range(int(round(t / DT))):
        s = dyn.step(F_up, np.zeros(3), DT)
    # body_x should have rotated by Ω_z·t about +Z (i.e. yaw by +0.5 rad)
    psi = Ω_z * t
    R_expected = np.array([[ math.cos(psi), -math.sin(psi), 0.0],
                           [ math.sin(psi),  math.cos(psi), 0.0],
                           [ 0.0,            0.0,           1.0]])
    np.testing.assert_allclose(s["R"], R_expected, atol=5e-4)


# ── 3. Gyroscopic precession ────────────────────────────────────────────────
#
# For a spinning rotor with spin angular momentum H_spin and an applied
# transverse torque τ, the spin axis precesses at rate Ω_p satisfying
#
#     τ = Ω_p × H_spin       (steady-state precession, ignoring nutation)
#
# In FRD body coordinates the aero's omega_rad_s > 0 means "CCW from above";
# with body_z pointing DOWN through the disk that is rotation about −body_z,
# so the rotor's spin angular momentum vector in body frame is
#
#     H_spin_b = −I_spin · ω_spin · ẑ_b           (i.e. pointing UP for level hover)
#
# (See dynamics.py:_derivs and CLAUDE.md "Coordinates & signs".)


def _integrate(omega_spin: float, M: np.ndarray, t: float,
               I_spin: float = I_SPIN) -> RigidBodyDynamics:
    """Run a fresh body forward by t seconds with constant M and omega_spin."""
    dyn = _fresh(I_spin=I_spin)
    F_up = np.array([0.0, 0.0, -MASS * G])
    for _ in range(int(round(t / DT))):
        dyn.step(F_up, M, DT, omega_spin=omega_spin)
    return dyn


def _time_average_omega(omega_spin: float, M: np.ndarray, t_total: float,
                        I_spin: float = I_SPIN) -> np.ndarray:
    """Mean angular velocity over t_total — averages out nutation transients."""
    dyn = _fresh(I_spin=I_spin)
    F_up = np.array([0.0, 0.0, -MASS * G])
    n = int(round(t_total / DT))
    acc = np.zeros(3)
    for _ in range(n):
        s = dyn.step(F_up, M, DT, omega_spin=omega_spin)
        acc += s["omega"]
    return acc / n


def test_gyroscopic_precession_axis_aligns_with_H_cross_M():
    """Spinning rotor + transverse torque ⇒ ω precesses along (H × M)/|H|².

    Level hover, R = I, body_z = +ẑ_world.  Aero convention: ω_spin > 0
    means CCW from above ⇒ H_spin_world = −I·ω·ẑ (points UP in NED).
    Apply M = +ŷ (East).  Expected Ω_p = (H × M)/|H|² along +x̂ (North),
    magnitude M/(I_spin·ω_spin).

    Time-averaging over multiple nutation periods removes the initial
    nutation transient (nutation period ≈ 2π·I_b/|H|).
    """
    omega_spin = 30.0
    M_amp      = 1.0
    M          = np.array([0.0, M_amp, 0.0])

    nutation_period = 2.0 * math.pi * I_BODY[0] / (I_SPIN * omega_spin)
    t_avg = 4.0 * nutation_period   # average over 4 cycles

    omega_avg = _time_average_omega(omega_spin, M, t_avg)
    expected_mag = M_amp / (I_SPIN * omega_spin)

    # +North-axis precession; ω_y and ω_z near zero on average.
    assert omega_avg[0] == pytest.approx(expected_mag, rel=0.1)
    assert abs(omega_avg[1]) < 0.05 * expected_mag
    assert abs(omega_avg[2]) < 0.05 * expected_mag


def test_gyroscopic_precession_reverses_with_reverse_spin():
    """Reversing ω_spin reverses the precession direction."""
    M_amp = 1.0
    M = np.array([0.0, M_amp, 0.0])
    nutation_period = 2.0 * math.pi * I_BODY[0] / (I_SPIN * 30.0)
    t_avg = 4.0 * nutation_period
    omega_pos = _time_average_omega(+30.0, M, t_avg)
    omega_neg = _time_average_omega(-30.0, M, t_avg)
    expected_mag = M_amp / (I_SPIN * 30.0)
    assert omega_pos[0] == pytest.approx(+expected_mag, rel=0.1)
    assert omega_neg[0] == pytest.approx(-expected_mag, rel=0.1)


def test_no_spin_yields_no_precession_only_roll_about_torque_axis():
    """Control: with I_spin·ω_spin = 0 the body just rotates about the
    applied-torque axis (Euler with no gyro coupling)."""
    M_amp = 1.0
    dyn = _fresh(I_spin=0.0)
    F_up = np.array([0.0, 0.0, -MASS * G])
    M    = np.array([0.0, M_amp, 0.0])
    t = 0.05
    for _ in range(int(round(t / DT))):
        s = dyn.step(F_up, M, DT, omega_spin=0.0)
    # Without gyroscopic coupling and Ixx = Iyy = 5, ω about +y grows at M/Iyy.
    expected_omega_y = M_amp / I_BODY[1] * t
    np.testing.assert_allclose(
        s["omega"], [0.0, expected_omega_y, 0.0], atol=5e-3,
    )


def test_gyroscopic_precession_rate_magnitude_scales_with_torque():
    """Time-averaged precession rate magnitude scales linearly with τ (|Ω_p| = M/|H|)."""
    omega_spin = 30.0
    H_mag = I_SPIN * omega_spin
    nutation_period = 2.0 * math.pi * I_BODY[0] / H_mag
    t_avg = 4.0 * nutation_period

    omega_x = []
    for M_amp in (0.5, 1.0, 2.0):
        M = np.array([0.0, M_amp, 0.0])
        omega_avg = _time_average_omega(omega_spin, M, t_avg)
        omega_x.append(omega_avg[0])

    expected = [0.5 / H_mag, 1.0 / H_mag, 2.0 / H_mag]
    np.testing.assert_allclose(omega_x, expected, rtol=0.10)


def test_gyroscopic_precession_tilts_disk_perpendicular_to_torque():
    """Disk tilts perpendicular to the applied torque, not aligned with it.

    With CCW spin (H along −ẑ_world = UP) and M along +ŷ (East), the
    precession is about +x̂ (North).  Rotation about +North takes +Down
    toward −East, so body_z gains a NEGATIVE Y component (tilts West) —
    classic helicopter "apply right-stick, disk tilts forward" gyro response,
    rotated for our axis convention.
    """
    omega_spin = 30.0
    M = np.array([0.0, 0.5, 0.0])   # mild East torque
    dyn = _integrate(omega_spin, M, t=0.5)
    body_z_world = dyn.state["R"][:, 2]

    # body_z[1] < 0 : tilted toward WEST (perpendicular to East torque, per precession)
    assert body_z_world[1] < -1e-3, (
        f"body_z did not precess West under East-torque + CCW spin: bz={body_z_world}"
    )
    # body_z[2] still mostly +1 (FRD: down through disk preserved)
    assert body_z_world[2] > 0.99
    # Negligible North-axis lean (rotation about North → no rotation along ±X)
    assert abs(body_z_world[0]) < 1e-3
