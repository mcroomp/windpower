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
    # H-force from in-plane wind (blowing in +X) pushes hub downwind → Fx > 0
    assert forces[0] > 0.0, f"Expected positive Fx from in-plane wind drag (H-force), got {forces[0]}"
    np.testing.assert_allclose(forces[1], 0.0, atol=1e-9)  # no Y force for X-aligned wind
    np.testing.assert_allclose(forces[3:5], np.zeros(2), atol=1e-9)
    # Near-equilibrium spin: |Mz| should be small relative to thrust (not a large drag spike)
    assert abs(forces[5]) < abs(forces[2]), \
        f"|Mz|={abs(forces[5]):.2f} should be < |Fz|={abs(forces[2]):.2f} at near-equilibrium spin"


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

    # tilt_lat > 0 (roll right/East) → My_body > 0 → forces[4] (My) increases
    # tilt_lon = 0 so Mx (forces[3]) is unchanged
    assert tilted[4] > baseline[4], (
        f"tilt_lat=0.5 should increase My (forces[4]); got baseline={baseline[4]:.4f} tilted={tilted[4]:.4f}"
    )
    np.testing.assert_allclose(tilted[3], baseline[3], rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(tilted[:3], baseline[:3], rtol=1e-9, atol=1e-9)


def test_compute_anti_rotation_moment_clamps_negative_esc_and_follows_spin_sign():
    aero = RotorAero()

    assert aero.compute_anti_rotation_moment(-1.0, 28.0, 50.0) == 0.0

    positive = aero.compute_anti_rotation_moment(0.5, 28.0, 50.0)
    negative = aero.compute_anti_rotation_moment(0.5, -28.0, 50.0)

    assert positive > 0.0
    assert negative < 0.0
    assert math.isclose(abs(positive), abs(negative), rel_tol=1e-12)