"""Python-only outer-loop isolation tests.

These tests sit between controller math and full 6-DOF physics.  They avoid
Lua, ArduPilot SITL, aero attitude lag, and elastic tether dynamics so failures
identify the Python guidance/collective layer before the full-loop tests.
"""
import math

import numpy as np
import pytest

from controller import TensionPI, compute_bz_tether, position_feedback_bz_eq
from tests.unit._aero_probe import load_rotor


_ROTOR = load_rotor("beaupoil_2026")
_MASS_KG = _ROTOR.inertia.mass_kg
if _MASS_KG is None:
    raise ValueError("beaupoil_2026 inertia.mass_kg is required")
_MASS = float(_MASS_KG)

DT = 1.0 / 400.0
TENSION_N = 300.0
KP_POS = 20.0
KD_POS = 45.0
COL_MIN = -0.28
COL_MAX = 0.10
COL_TRIM = -0.18


def _design_pos(elevation_deg: float = 30.0, tether_m: float = 100.0) -> np.ndarray:
    el = math.radians(elevation_deg)
    return tether_m * np.array([0.0, math.cos(el), -math.sin(el)])


def _restoring_force_from_bz(base_bz: np.ndarray, corrected_bz: np.ndarray) -> np.ndarray:
    """Force change from a body_z correction under the ideal thrust model."""
    return -TENSION_N * (corrected_bz - base_bz)


def test_tensionpi_increases_collective_when_tension_is_low():
    """The scalar TensionPI command has the expected sign."""
    ctrl = TensionPI(
        setpoint_n=300.0, kp=2.0e-4, ki=1.0e-4,
        coll_min=COL_MIN, coll_max=COL_MAX, warm_coll_rad=COL_TRIM,
    )
    low = ctrl.update(tension_actual=250.0, dt=0.1)
    assert low > COL_TRIM


def test_tensionpi_decreases_collective_when_tension_is_high():
    """The scalar TensionPI command backs off for excess tension."""
    ctrl = TensionPI(
        setpoint_n=300.0, kp=2.0e-4, ki=1.0e-4,
        coll_min=COL_MIN, coll_max=COL_MAX, warm_coll_rad=COL_TRIM,
    )
    high = ctrl.update(tension_actual=350.0, dt=0.1)
    assert high < COL_TRIM


def test_tensionpi_converges_simple_positive_tension_plant():
    """TensionPI regulates a scalar plant with positive dT/dcollective.

    This is the collective/tension controller layer by itself.  Elastic tether
    spring/slack dynamics are intentionally absent here; those belong in a
    higher-level integration test once the plant model has its own damping.
    """
    ctrl = TensionPI(
        setpoint_n=300.0, kp=2.5e-4, ki=8.0e-5, kd=2.0e-5,
        coll_min=COL_MIN, coll_max=COL_MAX, warm_coll_rad=COL_TRIM,
    )
    tension = 240.0
    dt = 0.02
    plant_tau = 0.8
    slope_n_per_rad = 500.0
    trim_tension = 300.0
    for _ in range(int(20.0 / dt)):
        col = ctrl.update(tension, dt)
        tension_target = trim_tension + slope_n_per_rad * (col - COL_TRIM)
        tension += (tension_target - tension) / plant_tau * dt

    assert abs(tension - 300.0) < 2.0
    assert COL_MIN < col < COL_MAX


def test_position_feedback_tilts_thrust_against_position_error():
    """A +North off-plane displacement must produce a South thrust correction."""
    anchor = np.zeros(3)
    target = _design_pos()
    pos = target + np.array([5.0, 0.0, 0.0])
    vel = np.zeros(3)
    base_bz = compute_bz_tether(pos, anchor)
    assert base_bz is not None

    corrected_bz = position_feedback_bz_eq(
        base_bz, pos, vel, target, TENSION_N, kp_pos=KP_POS, kd_pos=0.0,
    )
    force_correction = _restoring_force_from_bz(base_bz, corrected_bz)
    displacement = pos - target

    assert float(np.dot(force_correction, displacement)) < 0.0
    assert force_correction[0] < 0.0


def test_position_feedback_tilts_thrust_against_velocity():
    """A +North off-plane velocity must produce a South damping thrust."""
    anchor = np.zeros(3)
    target = _design_pos()
    pos = target.copy()
    vel = np.array([1.0, 0.0, 0.0])
    base_bz = compute_bz_tether(pos, anchor)
    assert base_bz is not None

    corrected_bz = position_feedback_bz_eq(
        base_bz, pos, vel, target, TENSION_N, kp_pos=0.0, kd_pos=KD_POS,
    )
    force_correction = _restoring_force_from_bz(base_bz, corrected_bz)

    assert float(np.dot(force_correction, vel)) < 0.0
    assert force_correction[0] < 0.0


def _run_ideal_constant_tether(
    *,
    pos_perturb: np.ndarray | None = None,
    vel_perturb: np.ndarray | None = None,
    force_pulse: tuple[float, float, np.ndarray] | None = None,
    kp_pos: float = KP_POS,
    kd_pos: float = KD_POS,
    t_total: float = 20.0,
) -> dict:
    """Translation-only plant with perfect attitude and constant tension.

    Tether force is ``T * body_z_tether``.  Ideal rotor thrust is
    ``-T * body_z_cmd``.  With no position feedback these cancel at every
    position, so any recovery comes only from the Python body_z correction.
    """
    anchor = np.zeros(3)
    target = _design_pos()
    pos = target + (np.asarray(pos_perturb, dtype=float)
                    if pos_perturb is not None else np.zeros(3))
    vel = (np.asarray(vel_perturb, dtype=float).copy()
           if vel_perturb is not None else np.zeros(3))

    dist_hist = []
    north_hist = []
    speed_hist = []
    for i in range(int(round(t_total / DT))):
        t_now = i * DT
        base_bz = compute_bz_tether(pos, anchor)
        if base_bz is None:
            return {
                "final_dist": math.inf,
                "max_dist": math.inf,
                "final_north": math.inf,
                "max_abs_north": math.inf,
                "final_speed": math.inf,
                "max_speed": math.inf,
                "final_pos": pos,
                "failure": "anchor_singularity",
            }
        bz_cmd = position_feedback_bz_eq(
            base_bz, pos, vel, target, TENSION_N, kp_pos=kp_pos, kd_pos=kd_pos,
            max_tilt_rad=math.radians(30.0),
        )
        force = TENSION_N * base_bz - TENSION_N * bz_cmd
        if force_pulse is not None:
            t0, t1, pulse = force_pulse
            if t0 <= t_now < t1:
                force = force + np.asarray(pulse, dtype=float)
        acc = force / _MASS
        vel = vel + acc * DT
        pos = pos + vel * DT
        dist_hist.append(float(np.linalg.norm(pos - target)))
        north_hist.append(float(pos[0] - target[0]))
        speed_hist.append(float(np.linalg.norm(vel)))

    return {
        "final_dist": dist_hist[-1],
        "max_dist": float(np.max(dist_hist)),
        "final_north": north_hist[-1],
        "max_abs_north": float(np.max(np.abs(north_hist))),
        "final_speed": speed_hist[-1],
        "max_speed": float(np.max(speed_hist)),
        "final_pos": pos,
    }


def test_ideal_constant_tether_recovers_position_offset():
    """With perfect attitude, the Python guidance recovers off-plane error."""
    r = _run_ideal_constant_tether(pos_perturb=np.array([5.0, 0.0, 0.0]))
    assert r["max_abs_north"] < 5.5
    assert abs(r["final_north"]) < 0.5
    assert r["final_speed"] < 0.5


def test_ideal_constant_tether_damps_velocity_kick():
    """With perfect attitude, the Python guidance damps off-plane speed."""
    r = _run_ideal_constant_tether(vel_perturb=np.array([1.0, 0.0, 0.0]))
    assert r["max_abs_north"] < 1.5
    assert abs(r["final_north"]) < 0.5
    assert r["final_speed"] < 0.5


def test_ideal_constant_tether_recovers_force_impulse():
    """With perfect attitude, recovery after a brief lateral force is bounded."""
    r = _run_ideal_constant_tether(
        force_pulse=(2.0, 4.0, np.array([20.0, 0.0, 0.0])),
        t_total=25.0,
    )
    assert r["max_abs_north"] < 6.0
    assert abs(r["final_north"]) < 0.8
    assert r["final_speed"] < 0.5
