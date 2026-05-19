"""test_attitude_convergence.py — closed-loop attitude convergence.

These tests close the loop at the unit-test level: take a hub with a known
mis-alignment, apply ``compute_rate_cmd`` → swashplate cyclic → aero hub
moment → ``RigidBodyDynamics``, and check that body_z actually converges
toward the target.

This catches sign errors in the full attitude-control chain that the
isolated component tests can miss — for example, the gyroscopic precession
phase that the controller must compensate for.

If these tests fail, the loop has a sign issue somewhere between
``compute_rate_cmd``, ``HeliCyclicController``, the new aero's cyclic
mapping, and ``RigidBodyDynamics.H_spin_b`` — i.e. exactly the chain that
makes ``test_create_ic`` settle (or not).
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aero        import RotorInputs
from controller  import HeliCyclicController, compute_rate_cmd
from dynamics    import RigidBodyDynamics
from frames      import build_orb_frame
from tests.unit._aero_probe import load_rotor, make_probe


# ── Setup ───────────────────────────────────────────────────────────────────

_ROTOR  = load_rotor("beaupoil_2026")
_AERO   = make_probe(_ROTOR)
_MASS   = float(_ROTOR.inertia.mass_kg)
_I_BODY = list(_ROTOR.inertia.I_body_kgm2)
_I_SPIN = float(_ROTOR.inertia.I_spin_kgm2
                if _ROTOR.inertia.I_spin_kgm2 is not None else 0.0)

DT          = 1.0 / 400.0
# Most attitude-convergence tests isolate the control loop from aero
# disturbances by using zero wind.  See `test_wind_creates_baseline_hub_moment`
# below for the disturbance fact this isolation hides.
WIND_NONE   = np.zeros(3)
OMEGA_SPIN  = 28.0
KP_OUTER    = 2.5


def _alignment_angle(bz_now: np.ndarray, bz_target: np.ndarray) -> float:
    """Angle between two unit vectors [rad]."""
    cos_a = float(np.clip(np.dot(bz_now, bz_target), -1.0, 1.0))
    return math.acos(cos_a)


def _run_attitude_loop(
    *,
    R0:           np.ndarray,
    body_z_eq:    np.ndarray,
    omega_spin:   float,
    wind:         np.ndarray = WIND_NONE,
    t_total:      float = 5.0,
) -> list:
    """Inner attitude loop with no F_world and no tether, only cyclic moment.

    Cancels gravity so the hub is stationary in space.  Holds omega_spin
    fixed.  Returns list of (t, angle_to_target).
    """
    dyn = RigidBodyDynamics(
        mass=_MASS, I_body=_I_BODY, I_spin=_I_SPIN,
        pos0=[0.0, 0.0, -50.0], vel0=[0.0, 0.0, 0.0],
        R0=R0.copy(), omega0=[0.0, 0.0, 0.0],
    )
    acro  = HeliCyclicController(_ROTOR, col_min_rad=-0.28, col_max_rad=0.10)
    state = _AERO.initial_rotor_state()
    state.omega_rad_s = float(omega_spin)
    F_grav_cancel = np.array([0.0, 0.0, -_MASS * 9.81])

    history = []
    n = int(round(t_total / DT))
    for i in range(n):
        s         = dyn.state
        R         = s["R"]
        omega_w   = s["omega"]
        omega_b   = R.T @ omega_w
        bz_now    = R[:, 2]
        rate_body = compute_rate_cmd(bz_now, body_z_eq, R, kp=KP_OUTER, kd=0.0)
        tlon, tlat, _col = acro.step(
            collective_cmd=0.0,
            rate_roll_sp =rate_body[0],
            rate_pitch_sp=rate_body[1],
            omega_body   =omega_b,
            dt           =DT,
        )
        inputs = RotorInputs(
            collective_rad=0.0, tilt_lon=tlon, tilt_lat=tlat,
            R_hub=R, v_hub_world=s["vel"], wind_world=wind, t=10.0,
        )
        result, _deriv = _AERO.compute_forces(inputs, state)
        state.omega_rad_s = float(omega_spin)

        dyn.step(F_grav_cancel, result.M_orbital, DT, omega_spin=omega_spin)

        if i % 10 == 0:
            history.append((i * DT, _alignment_angle(bz_now, body_z_eq)))

    s = dyn.state
    history.append((n * DT, _alignment_angle(s["R"][:, 2], body_z_eq)))
    return history


# ── Tests ────────────────────────────────────────────────────────────────────


def _R_tilt_in_direction(tilt_angle_rad: float, azimuth_rad: float) -> np.ndarray:
    """Build R0 with body_z tilted by `tilt_angle` toward world azimuth `az`.

    Azimuth measured in the world XY plane (NED): 0 = +X (North),
    π/2 = +Y (East), π = -X (South), 3π/2 = -Y (West).
    """
    bx = np.array([math.cos(azimuth_rad), math.sin(azimuth_rad), 0.0])  # in-plane direction
    bz_target = (math.sin(tilt_angle_rad) * bx
                 + math.cos(tilt_angle_rad) * np.array([0.0, 0.0, 1.0]))
    return build_orb_frame(bz_target)


@pytest.mark.parametrize("azimuth_deg", [0, 45, 90, 135, 180, 225, 270, 315])
def test_attitude_loop_converges_from_all_directions(azimuth_deg):
    """Sweep initial body_z error around all 8 cardinal+diagonal directions.

    With gyroscopic precession (CCW spin, FRD body_z DOWN) the hub-moment ↔
    body-rate relationship is rotated ~90° from the no-spin case.  A control
    law that's sign-correct on cardinal axes but not phase-compensated will
    converge for some azimuths and oscillate / diverge for others.  This
    test catches phase mismatches that a single-direction test would miss.
    """
    body_z_eq = np.array([0.0, 0.0, 1.0])    # FRD level
    R0   = _R_tilt_in_direction(math.radians(5.0), math.radians(azimuth_deg))
    hist = _run_attitude_loop(R0=R0, body_z_eq=body_z_eq,
                              omega_spin=OMEGA_SPIN, t_total=5.0)
    angle_initial = hist[0][1]
    angle_final   = hist[-1][1]
    assert angle_initial == pytest.approx(math.radians(5.0), abs=1e-3)
    assert angle_final < math.radians(2.0), (
        f"az={azimuth_deg}deg: did not converge to <2deg in 5s: "
        f"initial={math.degrees(angle_initial):.2f}deg, "
        f"final={math.degrees(angle_final):.2f}deg"
    )


def test_attitude_loop_holds_aligned_state():
    """Starting aligned with body_z_eq, the loop produces no drift."""
    body_z_eq = np.array([0.0, 0.0, 1.0])
    R0 = build_orb_frame(body_z_eq)
    hist = _run_attitude_loop(R0=R0, body_z_eq=body_z_eq, omega_spin=OMEGA_SPIN,
                              t_total=2.0)
    angle_final = hist[-1][1]
    assert angle_final < math.radians(0.5), (
        f"Aligned state drifted to {math.degrees(angle_final):.3f}deg"
    )


def test_zero_spin_produces_no_cyclic_authority():
    """With ω_spin = 0 the rotor has no aerodynamic forces — cyclic pitch
    does not produce a hub moment, and the body cannot be commanded.

    Documented here so the no-spin attitude failure is understood as a
    physics fact (stationary blades have no lift) rather than a sign bug.
    Re-tuning the closed loop must always assume a nonzero, regulated ω_spin.
    """
    body_z_eq = np.array([0.0, 0.0, 1.0])
    angle = math.radians(5.0)
    R0 = np.array([
        [ 1.0,  0.0,             0.0           ],
        [ 0.0,  math.cos(angle), -math.sin(angle)],
        [ 0.0,  math.sin(angle),  math.cos(angle)],
    ])
    hist = _run_attitude_loop(R0=R0, body_z_eq=body_z_eq, omega_spin=0.0,
                              t_total=2.0)
    # No control authority with stationary blades — angle does not decay.
    assert hist[-1][1] == pytest.approx(math.radians(5.0), abs=math.radians(0.5))


# ── Wind disturbance (separate fact — explains why test_create_ic is hard) ──


def test_acro_trim_feedforward_cancels_baseline_disturbance():
    """``HeliCyclicController.set_trim`` applied with the value from
    ``aero.solve_trim_cyclic`` cancels the wind-driven baseline hub moment.

    Without trim: at zero rate command the cyclic output is zero, so the
    full wind-driven baseline moment (~230 N*m) is applied to the body.
    With trim: the cyclic output equals the trim cyclic, which the aero
    package guarantees nulls the hub moment to within tolerance.
    """
    from aero import solve_trim_cyclic
    from controller import HeliCyclicController

    R = build_orb_frame(np.array([0.0, 0.0, 1.0]))
    wind = np.array([0.0, 10.0, 0.0])

    # Solve trim at the operating point.
    state0 = _AERO.initial_rotor_state()
    state0.omega_rad_s = OMEGA_SPIN
    trim = solve_trim_cyclic(
        _AERO, state0,
        collective_rad=-0.05, R_hub=R, v_hub_world=np.zeros(3),
        wind_world=wind, tolerance_Nm=0.5,
    )
    assert trim.converged

    # HeliCyclicController with zero rate command must reproduce the trim
    # cyclic at its output.
    acro = HeliCyclicController(_AERO.defn, col_min_rad=-0.28, col_max_rad=0.10)
    acro.set_trim(trim.tilt_lon, trim.tilt_lat)
    # Run a few steps with zero rate command so the servo settles to trim.
    omega_body_zero = np.zeros(3)
    for _ in range(200):
        tlon, tlat, _ = acro.step(
            collective_cmd=-0.05,
            rate_roll_sp =0.0,
            rate_pitch_sp=0.0,
            omega_body   =omega_body_zero,
            dt           =DT,
        )

    # Plug the AcroController's output back into the aero — the resulting
    # hub moment must equal the trim residual (small).
    inputs = RotorInputs(
        collective_rad=-0.05, tilt_lon=tlon, tilt_lat=tlat,
        R_hub=R, v_hub_world=np.zeros(3), wind_world=wind, t=10.0,
    )
    result, _ = _AERO.compute_forces(inputs, trim.final_state)
    M_mag = float(np.linalg.norm(result.M_orbital))
    assert M_mag < 5.0, (
        f"Trim feedforward did not cancel disturbance: |M|={M_mag:.2f} N*m"
    )


def test_wind_creates_baseline_hub_moment():
    """A horizontal wind across a level FRD disk produces a baseline hub
    moment even at zero cyclic input — comparable to the cyclic authority.

    This is the disturbance that the closed-loop attitude controller has to
    reject in the full IC-generation simtest.  Documented here as a
    standalone invariant so it's not lost when tuning the controller.
    """
    state = _AERO.initial_rotor_state()
    state.omega_rad_s = OMEGA_SPIN
    R = build_orb_frame(np.array([0.0, 0.0, 1.0]))   # level FRD

    # Settle dynamic inflow
    inp = RotorInputs(
        collective_rad=0.0, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R, v_hub_world=np.zeros(3),
        wind_world=np.array([0.0, 10.0, 0.0]), t=10.0,
    )
    for _ in range(200):
        r, d = _AERO.compute_forces(inp, state)
        state = state.from_array(state.to_array() + 0.02 * d.to_array())
        state.omega_rad_s = OMEGA_SPIN
    r, _ = _AERO.compute_forces(inp, state)

    M_mag = float(np.linalg.norm(r.M_orbital))
    assert M_mag > 50.0, (
        f"Expected a large baseline hub moment from cross-disk wind; got "
        f"|M|={M_mag:.1f} N*m"
    )
