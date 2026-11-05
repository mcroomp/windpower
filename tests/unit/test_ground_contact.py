"""Ground impact must end physics and preserve the final telemetry frame."""
from types import SimpleNamespace

import numpy as np
import pytest

from tests.common.mock_ardupilot import _MockArdupilotBase
from tests.simtests._rotor_helpers import load_default_rotor
from tests.simtests.simtest_runner import PhysicsRunner


class _NoAero:
    def initial_rotor_state(self):
        return None

    def step(self, inputs, state, dt):
        return SimpleNamespace(
            F_world=np.zeros(3), m_hub_world=np.zeros(3), Q_spin=0.0,
        ), state


def _runner(floor=0.0, *, starting_z=None, vertical_speed=1.0):
    ic = SimpleNamespace(
        pos=[0.0, 0.0, floor - 0.01 if starting_z is None else starting_z],
        vel=[0.0, 0.0, vertical_speed], R0=np.eye(3),
        rest_length=100.0, eq_thrust=0.6, omega_spin=40.0,
    )
    return PhysicsRunner(
        load_default_rotor(), ic, np.zeros(3), z_floor=floor, aero_override=_NoAero(),
        tether_model="spring_damper",
    )


@pytest.mark.parametrize("floor", [0.0, -1.0])
def test_impact_stops_substeps_and_prevents_restart(floor):
    runner = _runner(floor)
    core = runner._core
    result = core.step(0.02, 0.0, 0.0, 0.0)
    assert result["ground_contact"]
    assert core.terminated
    assert 0.0 < core.t_sim < 0.02
    assert result["hub_state"]["pos"][2] == floor
    stopped_time = core.t_sim
    stopped_spin = core.omega_spin
    stopped_state = core.hub_state
    with pytest.raises(RuntimeError, match="terminated on ground contact"):
        core.step(0.02, 1.0, 1.0, 1.0, rest_length=50.0)
    assert core.t_sim == stopped_time
    assert core.omega_spin == stopped_spin
    assert core.rest_length == 100.0
    for key in stopped_state:
        np.testing.assert_array_equal(core.hub_state[key], stopped_state[key])


def test_liftoff_from_floor_is_not_treated_as_impact():
    runner = _runner(starting_z=0.0, vertical_speed=-1.0)
    result = runner._core.step(0.02, 0.0, 0.0, 0.0)
    assert result["hub_state"]["pos"][2] < 0.0
    assert not runner._core.terminated
    assert not result["ground_contact"]


def test_ground_contact_is_logged_even_between_telemetry_ticks():
    runner = _runner()
    result = runner._core.step(0.02, 0.0, 0.0, 0.0)
    ap = _MockArdupilotBase(wind=np.zeros(3), dt=0.0025)
    ap.tel_fn = lambda r, sr: {}
    ap._log_step = 1
    ap.log(runner, result)
    assert len(ap.telemetry) == 1
    assert ap.telemetry[0].t_sim == runner.t_sim
    assert ap.telemetry[0].pos_z == 0.0
    assert ap.telemetry[0].note == "ground_contact"


def test_terminal_scenario_frame_can_be_forced_between_logging_ticks():
    runner = _runner()
    result = runner._core.step(0.001, 0.0, 0.0, 0.0)
    assert not result["ground_contact"]
    ap = _MockArdupilotBase(wind=np.zeros(3), dt=0.0025)
    ap.tel_fn = lambda r, sr: {}
    ap._log_step = 1
    ap.log(runner, result, force=True)
    assert len(ap.telemetry) == 1
    assert ap.telemetry[0].t_sim == runner.t_sim
