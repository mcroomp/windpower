"""Torque-driven drum, moving-tether damping, and coupled-step regressions."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from dynbem import RotorInputs

from simulation.physics_core import PhysicsCore, PrescribedAeroLoads
from simulation.param_defaults import thrust_to_coll_rad
from simulation.tether import SpringDamperTether, TetherInputs, TetherProperties
from simulation.winch import TorqueWinchController
from tests.simtests._rotor_helpers import load_default_rotor
from tests.simtests.simtest_ic import load_ic


def _winch(rest_length: float = 1.0, *, reflected_mass_kg: float = 2.0):
    return TorqueWinchController(
        rest_length=rest_length, reflected_mass_kg=reflected_mass_kg, speed_damping_ns_m=20.0,
        min_length=0.5, max_length=100.0,
    )


def test_balanced_drum_does_not_move():
    winch = _winch()
    winch.set_command(0.0, 100.0)
    winch.step(100.0, 1.0)
    assert winch.rest_length == 1.0
    assert winch.speed_ms == 0.0


@pytest.mark.parametrize(("load", "direction"), [(120.0, 1), (80.0, -1)])
def test_drum_accelerates_under_unbalanced_load(load: float, direction: int):
    winch = _winch(rest_length=10.0)
    winch.set_command(0.0, 100.0)
    winch.step(load, 0.001)
    assert winch.speed_ms * direction > 0.0
    assert abs(winch.speed_ms) < 0.01
    for _ in range(999):
        winch.step(load, 0.001)
    assert winch.speed_ms == pytest.approx(direction, abs=1e-4)
    assert winch.net_energy_j == pytest.approx(load * (winch.rest_length - 10.0))


def test_drum_end_stop_uses_actual_displacement_for_work():
    winch = _winch(rest_length=99.99)
    winch.set_command(1.0, 0.0)
    winch.step(100.0, 1.0)
    assert winch.rest_length == 100.0
    assert winch.speed_ms == 0.0
    assert winch.energy_out_j == pytest.approx(1.0)


@pytest.mark.parametrize(("speed", "tension"), [(float("nan"), 100.0), (0.0, -1.0)])
def test_invalid_command_is_rejected(speed: float, tension: float):
    with pytest.raises(ValueError):
        _winch().set_command(speed, tension)


@pytest.mark.parametrize("dt", [0.0, -0.1, float("nan")])
def test_invalid_drum_timestep_is_rejected(dt: float):
    with pytest.raises(ValueError):
        _winch().step(100.0, dt)


def test_damping_uses_extension_rate_not_absolute_hub_speed():
    tether = SpringDamperTether()
    position = np.array([0.0, 0.0, -100.0])
    velocity = np.array([0.0, 0.0, -1.0])
    def evaluate(vel: np.ndarray, payout_rate_ms: float = 0.0):
        return tether.evaluate(TetherInputs(
            anchor_pos_ned=np.zeros(3), anchor_vel_ned=np.zeros(3),
            attachment_pos_ned=position, attachment_vel_ned=vel,
            rest_length_m=99.99, payout_rate_ms=payout_rate_ms,
        ))

    matched = evaluate(velocity, payout_rate_ms=1.0)
    assert matched.damping_power_w == 0.0
    assert evaluate(velocity).hub_tension_n > matched.hub_tension_n
    assert evaluate(np.zeros(3), payout_rate_ms=-1.0).damping_power_w > 0.0


class _ConstantAero:
    def __init__(self, lift: float):
        self._lift = lift

    def initial_rotor_state(self):
        return None

    def step(self, inputs: RotorInputs, state: object, dt: float):
        return PrescribedAeroLoads(
            F_world=np.array([0.0, 0.0, -self._lift]),
            m_hub_world=np.zeros(3), M_spin=np.zeros(3), Q_spin=0.0,
        ), state


class _VelocityAero(_ConstantAero):
    def __init__(self, lift: float):
        super().__init__(lift)
        self.timesteps: list[float] = []

    def step(self, inputs: RotorInputs, state: object, dt: float):
        self.timesteps.append(dt)
        result, state = super().step(inputs, state, dt)
        result.F_world -= 10.0 * inputs.v_hub_world
        return result, state


def _coupled_run(
    dt: float, lift: float, duration: float = 0.5, *, reflected_mass_kg: float = 2.0,
    aero: _ConstantAero | None = None,
):
    rotor = load_default_rotor()
    assert rotor.control is not None
    assert rotor.control.axle_attachment_length_m is not None
    attachment_length = 1.0 - rotor.control.axle_attachment_length_m
    length = attachment_length * (1.0 - 100.0 / TetherProperties().axial_stiffness_n)
    winch = _winch(rest_length=length, reflected_mass_kg=reflected_mass_kg)
    winch.set_command(0.0, 100.0)
    ic = SimpleNamespace(
        pos=[0.0, 0.0, -1.0], vel=[0.0, 0.0, 0.0], R0=np.eye(3),
        rest_length=length, eq_thrust=0.6, omega_spin=40.0,
    )
    core = PhysicsCore(
        rotor, ic, np.zeros(3), winch=winch, z_floor=0.0,
        aero_override=_ConstantAero(lift) if aero is None else aero, tether_model="spring_damper",
    )
    tensions = []
    for _ in range(round(duration / dt)):
        result = core.step(dt, 0.0, 0.0, 0.0)
        assert result["tension_min"] <= core.tension_now <= result["tension_max"]
        tensions.extend((result["tension_min"], result["tension_max"]))
    return core, winch, np.array(tensions)


@pytest.mark.expensive
@pytest.mark.parametrize("reflected_mass_kg", [0.2, 2.0])
def test_preloaded_short_tether_stays_in_equilibrium(reflected_mass_kg: float):
    core, winch, tensions = _coupled_run(1 / 400, 149.05, reflected_mass_kg=reflected_mass_kg)
    assert core.t_sim == pytest.approx(0.5)
    np.testing.assert_allclose(core.hub_state["pos"], [0.0, 0.0, -1.0], atol=1e-8)
    np.testing.assert_allclose(tensions, 100.0, atol=1e-5)
    assert winch.speed_ms == pytest.approx(0.0, abs=1e-8)


@pytest.mark.expensive
@pytest.mark.parametrize("reflected_mass_kg", [0.2, 2.0])
def test_accelerating_short_tether_converges_with_timestep(reflected_mass_kg: float):
    coarse, drum_coarse, tensions = _coupled_run(1 / 400, 185.0, reflected_mass_kg=reflected_mass_kg)
    fine, drum_fine, _ = _coupled_run(1 / 800, 185.0, reflected_mass_kg=reflected_mass_kg)
    assert tensions.min() > 0.0
    assert tensions.max() < TetherProperties().break_load_n
    np.testing.assert_allclose(coarse.hub_state["pos"], fine.hub_state["pos"], atol=0.003)
    assert drum_coarse.speed_ms == pytest.approx(drum_fine.speed_ms, abs=0.01)


@pytest.mark.expensive
def test_stiff_substeps_keep_aero_at_frame_rate_and_converge():
    aero = _VelocityAero(185.0)
    coarse, drum, tensions = _coupled_run(
        1 / 400, 185.0, duration=0.1, reflected_mass_kg=0.2, aero=aero,
    )
    fine, fine_drum, _ = _coupled_run(
        1 / 1600, 185.0, duration=0.1, reflected_mass_kg=0.2, aero=_VelocityAero(185.0),
    )
    assert len(aero.timesteps) == 40
    assert sum(aero.timesteps) == pytest.approx(coarse.t_sim)
    assert tensions.min() > 0.0
    assert tensions.max() < TetherProperties().break_load_n
    np.testing.assert_allclose(coarse.hub_state["pos"], fine.hub_state["pos"], atol=0.003)
    np.testing.assert_allclose(coarse.hub_state["vel"], fine.hub_state["vel"], atol=0.01)
    assert drum.speed_ms == pytest.approx(fine_drum.speed_ms, abs=0.01)


@pytest.mark.expensive
@pytest.mark.parametrize("aero_model", ["quasi_static", "pitt_peters", "oye"])
def test_coupled_flight_aero_converges_with_frame_timestep(aero_model: str):
    rotor = load_default_rotor()
    ic = load_ic()
    collective = thrust_to_coll_rad(ic.eq_thrust)

    def run(dt: float):
        winch = TorqueWinchController(
            rest_length=ic.rest_length, reflected_mass_kg=0.2,
            speed_damping_ns_m=20.0, min_length=2.0, max_length=200.0,
        )
        winch.set_command(0.5, 300.0)
        core = PhysicsCore(rotor, ic, [0.0, 10.0, 0.0], winch=winch, aero_model=aero_model)
        core.warm_inflow(collective)
        for _ in range(round(0.1 / dt)):
            core.step(dt, collective, 0.0, 0.0)
        return core, winch

    coarse, drum = run(1 / 400)
    fine, fine_drum = run(1 / 1600)
    np.testing.assert_allclose(coarse.hub_state["pos"], fine.hub_state["pos"], atol=0.001)
    np.testing.assert_allclose(coarse.hub_state["vel"], fine.hub_state["vel"], atol=0.01)
    np.testing.assert_allclose(coarse.hub_state["R"], fine.hub_state["R"], atol=0.001)
    assert coarse.omega_spin == pytest.approx(fine.omega_spin, rel=0.001)
    assert coarse.tension_now == pytest.approx(fine.tension_now, rel=0.01)
    assert drum.speed_ms == pytest.approx(fine_drum.speed_ms, abs=0.01)


def test_coupled_drum_uses_anchor_load_not_hub_tension(monkeypatch: pytest.MonkeyPatch):
    rotor = load_default_rotor()
    assert rotor.control is not None
    assert rotor.control.axle_attachment_length_m is not None
    winch = _winch(rest_length=99.95)
    winch.set_command(0.0, 100.0)
    ic = SimpleNamespace(
        pos=[0.0, 0.0, -100.0 - rotor.control.axle_attachment_length_m],
        vel=[0.0, 0.0, 0.0], R0=np.eye(3),
        rest_length=winch.rest_length, eq_thrust=0.6, omega_spin=40.0,
    )
    core = PhysicsCore(
        rotor, ic, np.zeros(3), winch=winch, aero_override=_ConstantAero(149.05),
    )
    anchor_load = core.anchor_tension
    assert abs(anchor_load - core.tension_now) > 1.0
    step = Mock(wraps=winch.step)
    monkeypatch.setattr(winch, "step", step)

    core.step(1 / 400, 0.0, 0.0, 0.0)

    assert step.call_args_list[0].args[0] == pytest.approx(anchor_load)


@pytest.mark.parametrize("prescribed", [{"rest_length": 1.0}, {"payout_rate_ms": 0.0}])
def test_coupled_drum_rejects_prescribed_motion(prescribed: dict[str, float]):
    core, winch, _ = _coupled_run(1 / 400, 149.05, duration=0.0)
    length = winch.rest_length
    with pytest.raises(ValueError, match="coupled drum"):
        core.step(1 / 400, 0.0, 0.0, 0.0, **prescribed)
    assert core.t_sim == 0.0
    assert winch.rest_length == length
