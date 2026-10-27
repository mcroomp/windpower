from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.optimize import brentq

from simulation.config import defaults
from simulation.physics_core import PhysicsCore
from simulation.telemetry_csv import TelRow
from simulation.tether import (
    DEFAULT_TETHER_MODEL,
    ConstantTensionTether,
    SpringDamperTether,
    TallakQuasiStaticTether,
    TetherConvergenceError,
    TetherInputs,
    TetherProperties,
    TetherResult,
    make_tether,
)
from simulation.winch import GovernedWinchController
from simulation.winch_node import GovernedWinchNode
from tests.simtests._rotor_helpers import load_default_rotor
from tests.simtests.simtest_ic import load_ic
from tests.simtests.simtest_runner import PhysicsRunner


def _inputs(**changes) -> TetherInputs:
    return replace(TetherInputs(
        anchor_pos_ned=np.zeros(3),
        anchor_vel_ned=np.zeros(3),
        attachment_pos_ned=np.array([100.1, 0.0, 0.0]),
        attachment_vel_ned=np.zeros(3),
        rest_length_m=100.0,
    ), **changes)


@pytest.mark.parametrize("length,velocity", [(50.1, 0.0), (100.1, 0.3), (200.1, -0.3)])
def test_spring_matches_original_constitutive_law(length: float, velocity: float) -> None:
    model = SpringDamperTether()
    inputs = _inputs(
        attachment_pos_ned=np.array([length, 0.0, 0.0]),
        attachment_vel_ned=np.array([velocity, 0.0, 0.0]),
        rest_length_m=length - 0.1,
    )
    result = model.evaluate(inputs)
    stiffness = model.properties.axial_stiffness_n / length
    expected = stiffness * 0.1 + 0.05 * 2.0 * np.sqrt(stiffness * 5.0) * max(0.0, velocity)
    assert result.hub_tension_n == pytest.approx(expected)
    np.testing.assert_array_equal(result.force_on_hub_ned, -result.force_on_anchor_ned)
    assert result.damping_power_w >= 0.0


@pytest.mark.parametrize("position", [np.zeros(3), np.array([99.0, 0.0, 0.0])])
def test_unloaded_spring_never_pushes(position: np.ndarray) -> None:
    result = SpringDamperTether().evaluate(_inputs(attachment_pos_ned=position))
    assert result.slack
    assert result.hub_tension_n == result.anchor_tension_n == 0.0


def test_payout_and_anchor_motion_are_removed_from_extension_rate() -> None:
    model = SpringDamperTether()
    baseline = model.evaluate(_inputs())
    matched_payout = model.evaluate(_inputs(
        anchor_vel_ned=np.array([1.0, 0.0, 0.0]),
        attachment_vel_ned=np.array([3.0, 0.0, 0.0]),
        payout_rate_ms=2.0,
    ))
    assert matched_payout.hub_tension_n == pytest.approx(baseline.hub_tension_n)
    assert matched_payout.damping_power_w == pytest.approx(0.0)
    reeling_in = model.evaluate(_inputs(payout_rate_ms=-0.5))
    assert reeling_in.hub_tension_n > baseline.hub_tension_n


def test_spring_elastic_energy_derivative_matches_force() -> None:
    model = SpringDamperTether(zeta=0.0)
    step = 1e-4
    before = model.evaluate(_inputs(attachment_pos_ned=np.array([100.1 - step, 0.0, 0.0])))
    after = model.evaluate(_inputs(attachment_pos_ned=np.array([100.1 + step, 0.0, 0.0])))
    force_from_energy = (after.elastic_energy_j - before.elastic_energy_j) / (2 * step)
    assert force_from_energy == pytest.approx(model.evaluate(_inputs()).hub_tension_n, rel=1e-6)


@pytest.mark.parametrize("model", [
    SpringDamperTether(),
    TallakQuasiStaticTether(properties=TetherProperties(linear_mass_kg_m=0.0)),
    ConstantTensionTether(100.0),
])
def test_no_distributed_load_has_equal_opposite_endpoint_reactions(model) -> None:
    result = model.evaluate(_inputs())
    np.testing.assert_allclose(result.force_on_hub_ned + result.force_on_anchor_ned, 0.0, atol=1e-10)
    np.testing.assert_allclose(result.positions_ned[[0, -1]], [[0, 0, 0], [100.1, 0, 0]], atol=1e-6)


def test_quasi_static_unloaded_elastic_limit_stays_close_to_spring() -> None:
    inputs = _inputs()
    static = TallakQuasiStaticTether(properties=TetherProperties(linear_mass_kg_m=0.0)).evaluate(inputs)
    spring = SpringDamperTether().evaluate(inputs)
    assert static.hub_tension_n == pytest.approx(spring.hub_tension_n, rel=0.002)


@pytest.mark.parametrize("segments", [8, 16, 32])
def test_gravity_only_matches_analytical_catenary(segments: int) -> None:
    span, length = 80.0, 100.0
    properties = TetherProperties(axial_stiffness_n=1e12)
    inputs = _inputs(
        attachment_pos_ned=np.array([span, 0.0, 0.0]),
        rest_length_m=length,
    )
    result = TallakQuasiStaticTether(properties=properties, segments=segments).evaluate(inputs)
    a = brentq(lambda value: 2 * value * np.sinh(span / (2 * value)) - length, 1.0, 1000.0)
    assert isinstance(a, float)
    weight_per_m = properties.linear_mass_kg_m * 9.81
    sag = a * (np.cosh(span / (2 * a)) - 1)
    assert result.force_on_anchor_ned[0] == pytest.approx(weight_per_m * a, rel=0.015)
    assert result.force_on_anchor_ned[2] == pytest.approx(weight_per_m * length / 2, rel=1e-6)
    assert result.positions_ned[segments // 2, 2] == pytest.approx(sag, rel=0.015)
    np.testing.assert_allclose(result.positions_ned[-1], inputs.attachment_pos_ned, atol=1e-6)
    np.testing.assert_allclose(
        result.force_on_hub_ned + result.force_on_anchor_ned,
        properties.linear_mass_kg_m * length * inputs.gravity_ned, atol=1e-8,
    )
    assert not result.slack


def test_transverse_wind_drag_is_quadratic_and_shared_between_endpoints() -> None:
    properties = TetherProperties(linear_mass_kg_m=0.0)
    model = TallakQuasiStaticTether(properties=properties)
    low = model.evaluate(_inputs(wind_ned=np.array([0.0, 2.0, 0.0])))
    high = model.evaluate(_inputs(wind_ned=np.array([0.0, 4.0, 0.0])))
    assert high.distributed_force_ned[1] / low.distributed_force_ned[1] == pytest.approx(4.0, rel=0.002)
    assert high.force_on_hub_ned[1] > 0.0
    assert high.force_on_anchor_ned[1] > 0.0
    assert high.force_on_hub_ned[1] == pytest.approx(high.distributed_force_ned[1] / 2, rel=0.002)
    np.testing.assert_allclose(
        high.force_on_hub_ned + high.force_on_anchor_ned,
        high.distributed_force_ned, atol=1e-9,
    )
    expected_full_drag = 0.5 * 1.225 * properties.diameter_m * properties.drag_coefficient * 4**2 * high.length_m
    assert high.distributed_force_ned[1] == pytest.approx(expected_full_drag, rel=0.002)


def test_wind_parallel_to_straight_line_has_no_transverse_drag() -> None:
    model = TallakQuasiStaticTether(properties=TetherProperties(linear_mass_kg_m=0.0))
    result = model.evaluate(_inputs(wind_ned=np.array([10.0, 0.0, 0.0])))
    np.testing.assert_array_equal(result.distributed_force_ned, np.zeros(3))


@pytest.mark.parametrize("vertical_sign", [-1.0, 1.0])
def test_endpoint_tension_order_follows_weight_and_geometry(vertical_sign: float) -> None:
    inputs = _inputs(attachment_pos_ned=np.array([0.0, 0.0, vertical_sign * 100.1]))
    model = TallakQuasiStaticTether()
    result = model.evaluate(inputs)
    weight = model.properties.linear_mass_kg_m * inputs.rest_length_m * 9.81
    assert result.hub_tension_n - result.anchor_tension_n == pytest.approx(-vertical_sign * weight)


def test_quasi_static_mesh_converges_and_warm_start_does_not_change_solution() -> None:
    inputs = _inputs(
        attachment_pos_ned=np.array([80.0, 0.0, -40.0]),
        rest_length_m=100.0, wind_ned=np.array([0.0, 10.0, 0.0]),
    )
    results = [TallakQuasiStaticTether(segments=n).evaluate(inputs) for n in (8, 16, 32)]
    coarse_error = np.linalg.norm(results[0].force_on_hub_ned - results[2].force_on_hub_ned)
    fine_error = np.linalg.norm(results[1].force_on_hub_ned - results[2].force_on_hub_ned)
    assert fine_error < coarse_error
    assert fine_error / results[2].hub_tension_n < 0.01
    model = TallakQuasiStaticTether()
    cold = model.evaluate(inputs)
    warm = model.evaluate(inputs)
    np.testing.assert_allclose(warm.force_on_hub_ned, cold.force_on_hub_ned, rtol=1e-6, atol=1e-6)


def test_quasi_static_is_galilean_invariant() -> None:
    inputs = _inputs(wind_ned=np.array([0.0, 10.0, 0.0]))
    velocity = np.array([3.0, -2.0, 1.0])
    shifted = replace(
        inputs, wind_ned=inputs.wind_ned + velocity,
        anchor_vel_ned=inputs.anchor_vel_ned + velocity,
        attachment_vel_ned=inputs.attachment_vel_ned + velocity,
    )
    original = TallakQuasiStaticTether().evaluate(inputs)
    moved = TallakQuasiStaticTether().evaluate(shifted)
    np.testing.assert_allclose(moved.force_on_hub_ned, original.force_on_hub_ned, atol=1e-8)


@pytest.mark.parametrize("length", [50.0, 100.0, 200.0])
@pytest.mark.parametrize("nominal_tension", [30.0, 300.0])
@pytest.mark.parametrize("wind_speed", [0.0, 10.0, 15.0])
def test_operating_envelope_is_finite_balanced_and_mesh_resolved(
    length: float, nominal_tension: float, wind_speed: float,
) -> None:
    properties = TetherProperties()
    attachment = length * np.array([np.sqrt(3) / 2, 0.0, -0.5])
    inputs = _inputs(
        attachment_pos_ned=attachment,
        rest_length_m=length * (1 - nominal_tension / properties.axial_stiffness_n),
        wind_ned=np.array([0.0, wind_speed, 0.0]),
    )
    result = TallakQuasiStaticTether().evaluate(inputs)
    reference = TallakQuasiStaticTether(segments=32).evaluate(inputs)
    assert np.all(np.isfinite(result.positions_ned))
    np.testing.assert_allclose(result.positions_ned[-1], attachment, atol=1e-6)
    np.testing.assert_allclose(
        result.force_on_hub_ned + result.force_on_anchor_ned,
        result.distributed_force_ned, atol=1e-8,
    )
    assert np.dot(result.force_on_hub_ned, attachment) < 0.0
    assert result.length_m >= length - 1e-6
    assert result.extension_m >= 0.0
    np.testing.assert_allclose(
        result.force_on_hub_ned, reference.force_on_hub_ned,
        rtol=0.01, atol=0.02,
    )


def test_wind_step_and_payout_warm_start_match_independent_equilibrium() -> None:
    model = TallakQuasiStaticTether()
    model.evaluate(_inputs(wind_ned=np.array([0.0, 5.0, 0.0])))
    changed = _inputs(
        wind_ned=np.array([0.0, 15.0, 0.0]), rest_length_m=100.05,
        payout_rate_ms=0.5,
    )
    warm = model.evaluate(changed)
    cold = TallakQuasiStaticTether().evaluate(changed)
    np.testing.assert_allclose(warm.force_on_hub_ned, cold.force_on_hub_ned, rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(warm.force_on_anchor_ned, cold.force_on_anchor_ned, rtol=1e-5, atol=1e-5)


def test_solver_failure_is_explicit(monkeypatch: pytest.MonkeyPatch) -> None:
    def failed_solve(fun, initial, **kwargs):
        return SimpleNamespace(x=initial, success=False, message="test failure")

    monkeypatch.setattr("simulation.tether.least_squares", failed_solve)
    with pytest.raises(TetherConvergenceError, match="test failure"):
        TallakQuasiStaticTether().evaluate(_inputs())


@pytest.mark.parametrize("changes", [
    {"rest_length_m": 0.0}, {"rest_length_m": float("nan")},
    {"wind_ned": np.array([0.0, float("nan"), 0.0])},
    {"attachment_pos_ned": np.zeros(2)}, {"payout_rate_ms": float("inf")},
    {"air_density_kg_m3": -1.0},
])
def test_invalid_physics_inputs_are_rejected(changes) -> None:
    with pytest.raises(ValueError):
        _inputs(**changes)


def test_unknown_model_is_not_silently_replaced_by_spring() -> None:
    with pytest.raises(ValueError, match="Unknown tether model"):
        make_tether("misspelled", hub_mass=5.0)


@pytest.mark.parametrize("name,model_type", [
    ("spring_damper", SpringDamperTether),
    ("tallak_quasi_static", TallakQuasiStaticTether),
    ("constant_tension", ConstantTensionTether),
])
def test_model_selection_is_explicit(name: str, model_type: type) -> None:
    assert isinstance(make_tether(name, hub_mass=5.0), model_type)


def test_tallak_is_the_shared_default() -> None:
    config = defaults()
    assert DEFAULT_TETHER_MODEL == config["tether_model"] == "tallak_quasi_static"
    rotor = load_default_rotor()
    ic = load_ic()
    wind = np.asarray(config["wind"], dtype=float)
    core = PhysicsCore(rotor, ic, wind)
    runner = PhysicsRunner(rotor, ic, wind)
    assert isinstance(core.tether, TallakQuasiStaticTether)
    assert isinstance(runner.tether, TallakQuasiStaticTether)
    assert not core.tether_result.slack
    assert not runner.tether_result.slack


def test_winch_energy_uses_local_load_cell_force() -> None:
    winch = GovernedWinchController(
        rest_length=100.0, v_max_out=0.4, v_max_in=0.8,
        kp_tension=0.005, accel_limit_ms2=0.5, jerk_limit_ms3=1.0,
    )
    winch.set_command(0.2, 40.0)
    node = GovernedWinchNode(winch)
    node.update_sensors(40.0)
    node.step(0.1)
    telemetry = node.telemetry()
    assert telemetry.speed_ms > 0.0
    assert telemetry.net_energy_j == pytest.approx(40.0 * telemetry.speed_ms * 0.1)


class _RecordingTether:
    properties = TetherProperties()

    def __init__(self) -> None:
        self.inputs: TetherInputs | None = None

    def evaluate(self, inputs: TetherInputs) -> TetherResult:
        self.inputs = inputs
        return TetherResult(
            force_on_hub_ned=np.array([0.0, -50.0, 0.0]),
            force_on_anchor_ned=np.array([0.0, 40.0, 0.0]),
            positions_ned=np.array([inputs.anchor_pos_ned, inputs.attachment_pos_ned]),
            length_m=100.0, extension_m=0.1, slack=False,
            distributed_force_ned=np.array([0.0, -10.0, 0.0]),
        )


def test_physics_uses_attachment_kinematics_and_separate_winch_load() -> None:
    rotor = load_default_rotor()
    assert rotor.control is not None
    assert rotor.control.axle_attachment_length_m is not None
    ic = SimpleNamespace(
        pos=np.array([0.0, 100.0, -50.0]), vel=np.array([1.0, 0.0, 0.0]),
        R0=np.eye(3), rest_length=100.0, eq_thrust=0.5, omega_spin=40.0,
    )
    tether = _RecordingTether()
    core = PhysicsCore(rotor, ic, np.zeros(3), tether_override=tether)
    core._dyn._omega[:] = [1.0, 0.0, 0.0]
    offset = np.array([0.0, 0.0, rotor.control.axle_attachment_length_m])
    expected_position = core.hub_state["pos"].copy() + offset
    expected_velocity = core.hub_state["vel"].copy() + np.cross([1.0, 0.0, 0.0], offset)
    result = core.step(1 / 1200, 0.0, 0.0, 0.0, rest_length=100.001)
    assert tether.inputs is not None
    np.testing.assert_allclose(tether.inputs.attachment_pos_ned, expected_position)
    np.testing.assert_allclose(tether.inputs.attachment_vel_ned, expected_velocity)
    assert tether.inputs.payout_rate_ms == pytest.approx(1.2)
    np.testing.assert_allclose(result["tether_moment"], np.cross(offset, [0.0, -50.0, 0.0]))
    assert core.tension_now == 50.0
    assert core.anchor_tension == 40.0

    node = GovernedWinchNode(GovernedWinchController(
        rest_length=100.0, v_max_out=0.4, v_max_in=0.8,
        kp_tension=0.005, accel_limit_ms2=0.5, jerk_limit_ms3=1.0,
    ))
    node.update_sensors(core.anchor_tension)
    node.step(1 / 1200)
    assert node.telemetry().tension_n == 40.0
    row = TelRow.from_physics(
        core, result, collective_rad=0.0, wind_ned=np.zeros(3),
    )
    assert row.tether_tension == 50.0
    assert row.tether_anchor_tension == 40.0
    assert row.to_dict()["tether_anchor_tension"] == 40.0
