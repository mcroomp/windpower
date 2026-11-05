"""Torque-driven drum, moving-tether damping, and coupled-step regressions."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from simulation.physics_core import PhysicsCore
from simulation.tether import SpringDamperTether, TetherInputs, TetherProperties
from simulation.winch import TorqueWinchController
from tests.simtests._rotor_helpers import load_default_rotor


def _winch(rest_length=1.0, *, reflected_mass_kg=2.0):
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
def test_drum_accelerates_under_unbalanced_load(load, direction):
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
def test_invalid_command_is_rejected(speed, tension):
    with pytest.raises(ValueError):
        _winch().set_command(speed, tension)


@pytest.mark.parametrize("dt", [0.0, -0.1, float("nan")])
def test_invalid_drum_timestep_is_rejected(dt):
    with pytest.raises(ValueError):
        _winch().step(100.0, dt)


def test_damping_uses_extension_rate_not_absolute_hub_speed():
    tether = SpringDamperTether()
    position = np.array([0.0, 0.0, -100.0])
    velocity = np.array([0.0, 0.0, -1.0])
    def evaluate(vel, payout_rate_ms=0.0):
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
    def __init__(self, lift):
        self._lift = lift

    def initial_rotor_state(self):
        return None

    def step(self, inputs, state, dt):
        return SimpleNamespace(
            F_world=np.array([0.0, 0.0, -self._lift]),
            m_hub_world=np.zeros(3), Q_spin=0.0,
        ), state


def _coupled_run(dt, lift, duration=0.5, *, reflected_mass_kg=2.0):
    rotor = load_default_rotor()
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
        aero_override=_ConstantAero(lift), tether_model="spring_damper",
    )
    tensions = []
    for _ in range(round(duration / dt)):
        result = core.step(dt, 0.0, 0.0, 0.0)
        assert result["tension_min"] <= core.tension_now <= result["tension_max"]
        tensions.extend((result["tension_min"], result["tension_max"]))
    return core, winch, np.array(tensions)


@pytest.mark.expensive
@pytest.mark.parametrize("reflected_mass_kg", [0.2, 2.0])
def test_preloaded_short_tether_stays_in_equilibrium(reflected_mass_kg):
    core, winch, tensions = _coupled_run(1 / 400, 149.05, reflected_mass_kg=reflected_mass_kg)
    assert core.t_sim == pytest.approx(0.5)
    np.testing.assert_allclose(core.hub_state["pos"], [0.0, 0.0, -1.0], atol=1e-8)
    np.testing.assert_allclose(tensions, 100.0, atol=1e-5)
    assert winch.speed_ms == pytest.approx(0.0, abs=1e-8)


@pytest.mark.expensive
@pytest.mark.parametrize("reflected_mass_kg", [0.2, 2.0])
def test_accelerating_short_tether_converges_with_timestep(reflected_mass_kg):
    coarse, drum_coarse, tensions = _coupled_run(1 / 400, 185.0, reflected_mass_kg=reflected_mass_kg)
    fine, drum_fine, _ = _coupled_run(1 / 800, 185.0, reflected_mass_kg=reflected_mass_kg)
    assert tensions.min() > 0.0
    assert tensions.max() < TetherProperties().break_load_n
    np.testing.assert_allclose(coarse.hub_state["pos"], fine.hub_state["pos"], atol=0.003)
    assert drum_coarse.speed_ms == pytest.approx(drum_fine.speed_ms, abs=0.01)


def test_coupled_drum_uses_anchor_load_not_hub_tension(monkeypatch):
    rotor = load_default_rotor()
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
def test_coupled_drum_rejects_prescribed_motion(prescribed):
    core, winch, _ = _coupled_run(1 / 400, 149.05, duration=0.0)
    length = winch.rest_length
    with pytest.raises(ValueError, match="coupled drum"):
        core.step(1 / 400, 0.0, 0.0, 0.0, **prescribed)
    assert core.t_sim == 0.0
    assert winch.rest_length == length
