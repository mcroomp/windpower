import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aero import RotorAero


def test_compute_forces_is_zero_during_ramp_start():
    aero = RotorAero()

    forces = aero.compute_forces(
        collective_rad=0.1,
        tilt_lon=0.0,
        tilt_lat=0.0,
        R_hub=np.eye(3),
        v_hub_world=np.zeros(3),
        omega_rotor=28.0,
        wind_world=np.array([10.0, 0.0, 0.0]),
        t=0.0,
    )

    np.testing.assert_allclose(forces, np.zeros(6), atol=1e-12)


def test_compute_forces_produces_upward_thrust_and_opposing_drag_torque():
    aero = RotorAero()

    forces = aero.compute_forces(
        collective_rad=0.1,
        tilt_lon=0.0,
        tilt_lat=0.0,
        R_hub=np.eye(3),
        v_hub_world=np.zeros(3),
        omega_rotor=28.0,
        wind_world=np.array([10.0, 0.0, 0.0]),
        t=10.0,
    )

    assert forces[2] > 0.0
    np.testing.assert_allclose(forces[:2], np.zeros(2), atol=1e-9)
    np.testing.assert_allclose(forces[3:5], np.zeros(2), atol=1e-9)
    assert forces[5] < 0.0


def test_compute_forces_cyclic_tilt_changes_lateral_moment_component():
    aero = RotorAero()

    baseline = aero.compute_forces(
        collective_rad=0.1,
        tilt_lon=0.0,
        tilt_lat=0.0,
        R_hub=np.eye(3),
        v_hub_world=np.zeros(3),
        omega_rotor=28.0,
        wind_world=np.array([10.0, 0.0, 0.0]),
        t=10.0,
    )
    tilted = aero.compute_forces(
        collective_rad=0.1,
        tilt_lon=0.0,
        tilt_lat=0.5,
        R_hub=np.eye(3),
        v_hub_world=np.zeros(3),
        omega_rotor=28.0,
        wind_world=np.array([10.0, 0.0, 0.0]),
        t=10.0,
    )

    assert tilted[3] > baseline[3]
    np.testing.assert_allclose(tilted[:3], baseline[:3], rtol=1e-9, atol=1e-9)


def test_compute_anti_rotation_moment_clamps_negative_esc_and_follows_spin_sign():
    aero = RotorAero()

    assert aero.compute_anti_rotation_moment(-1.0, 28.0, 50.0) == 0.0

    positive = aero.compute_anti_rotation_moment(0.5, 28.0, 50.0)
    negative = aero.compute_anti_rotation_moment(0.5, -28.0, 50.0)

    assert positive > 0.0
    assert negative < 0.0
    assert math.isclose(abs(positive), abs(negative), rel_tol=1e-12)