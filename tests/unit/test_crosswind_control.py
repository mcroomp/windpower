import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from simulation.controller import compute_crosswind_rate_cmd, apply_crosswind_rate_to_body_rates
from simulation.rawes_lua_harness import RawesLua


@pytest.mark.parametrize(("north", "velocity"), [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)])
def test_crosswind_command_produces_restoring_thrust(north: float, velocity: float):
    sim = RawesLua()
    rate = compute_crosswind_rate_cmd(
        np.array([north, 0.0, 0.0]), np.array([velocity, 0.0, 0.0]),
        np.zeros(3), kp=0.2, kd=0.1, rate_max=0.6,
    )
    assert sim.fns.crosswind_rate_cmd(north, velocity, 0.2, 0.1, 0.6) == pytest.approx(rate)
    body_z = Rotation.from_rotvec([0.0, rate * 0.01, 0.0]).as_matrix()[:, 2]
    thrust_north = -body_z[0]
    assert thrust_north * (north + velocity) < 0.0


@pytest.mark.parametrize("angles", [(0, 0, 0), (0, 0, 90), (25, 40, 70), (-35, 60, -120)])
def test_lua_crosswind_projection_matches_full_body_rotation(angles: tuple[float, float, float]):
    sim = RawesLua()
    R = Rotation.from_euler("xyz", angles, degrees=True).as_matrix()
    sim.R = R
    roll, pitch = sim.fns.crosswind_body_rates(0.6)
    expected = apply_crosswind_rate_to_body_rates(0.6, R)
    np.testing.assert_allclose([roll, pitch], expected[:2], atol=1e-12)


def test_guided_api_receives_degrees_per_second_not_radians():
    sim = RawesLua()
    sim.fns.send_guided_angle_rate_throttle(10.0, 20.0, 30.0, 0.1, -0.2, 0.3, 0.6, "test")
    target = sim.guided_target
    assert target is not None
    assert target["roll_rate"] == pytest.approx(math.degrees(0.1))
    assert target["pitch_rate"] == pytest.approx(math.degrees(-0.2))
    assert target["yaw_rate"] == pytest.approx(math.degrees(0.3))
