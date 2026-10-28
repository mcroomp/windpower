import numpy as np

from simulation.telemetry_csv import TelRow
from tests.simtests._rotor_helpers import load_default_rotor
from tests.simtests.simtest_ic import load_ic
from tests.simtests.simtest_runner import PhysicsRunner


WIND_NED = np.array([0.0, 10.0, 0.0])


def _build_runner(*, aero_model: str) -> PhysicsRunner:
	rotor = load_default_rotor()
	ic = load_ic()
	return PhysicsRunner(rotor, ic, WIND_NED, aero_model=aero_model, z_floor=0.0)


def test_aero_columns_populated_from_physics_step() -> None:
	runner = _build_runner(aero_model="quasi_static")
	assert runner.altitude > 1.0
	assert not runner.tether_result.slack
	assert np.all(runner.tether_result.positions_ned[:, 2] <= 0.0)
	step_result = runner._core.step(0.02, collective_rad=0.08, tilt_lon=0.01, tilt_lat=-0.02)
	row = TelRow.from_physics(
		runner,
		step_result,
		collective_rad=0.08,
		wind_ned=WIND_NED,
		phase="unit_audit",
	)

	aero_result = step_result["aero_result"]
	F = np.asarray(aero_result.F_world, dtype=float)
	M = np.asarray(aero_result.m_hub_world, dtype=float)

	assert np.isclose(row.aero_fx, float(F[0]))
	assert np.isclose(row.aero_fy, float(F[1]))
	assert np.isclose(row.aero_fz, float(F[2]))
	assert np.isclose(row.aero_mx, float(M[0]))
	assert np.isclose(row.aero_my, float(M[1]))
	assert np.isclose(row.aero_mz, float(M[2]))
	assert np.isclose(row.aero_T, float(np.linalg.norm(F)))
	assert np.isclose(row.aero_Q_spin, float(aero_result.Q_spin))

	# AeroResult from aero only exposes F_world, m_hub_world, Q_spin, M_spin.
	# Non-source decomposition diagnostics must remain zero.
	assert np.isclose(row.aero_v_axial, 0.0)
	assert np.isclose(row.aero_v_inplane, 0.0)
	assert np.isclose(row.aero_v_i, 0.0)
