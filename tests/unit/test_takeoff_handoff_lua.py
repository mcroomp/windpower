import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from groundstation.rawes_modes import MODE_STEADY, MODE_TAKEOFF, send_anchor_ned
from linkhub_client.messages import NamedValueFloat
from simulation.rawes_lua_harness import RawesLua
from simulation.controller import compute_takeoff_body_z


def _sim(mode: int):
    sim = RawesLua(mode=mode)
    sim.armed = True
    sim.healthy = True
    sim.vehicle_mode = 4
    sim.pos_ned = [0.0, 2.0, -10.0]
    sim.vel_ned = [0.0, 0.0, -1.0]
    send_anchor_ned(sim, 0.0, 0.0, 0.0)
    for name, value in (("RAWES_ALT", 11.0), ("RAWES_THR", 0.8), ("RAWES_TEN", 100.0)):
        sim.send_message(NamedValueFloat(name=name, value=value))
    return sim


def test_takeoff_handoff_preserves_live_altitude_controller():
    sim = _sim(MODE_TAKEOFF)
    sim.run(0.1)
    thrust = sim.guided_throttle
    assert thrust is not None
    integral = sim.fns.takeoff_alt_i()
    trim = sim.fns.takeoff_thrust_trim()
    sim.send_message(NamedValueFloat(name="RAWES_THR", value=thrust))
    sim.set_param("RAWES_MODE", MODE_STEADY)
    sim.run(0.02)
    assert sim.fns.el_initialized()
    assert sim.fns.thrust_trim() == pytest.approx(trim)
    assert sim.fns.alt_i() == pytest.approx(integral)
    assert sim.guided_throttle == pytest.approx(thrust)
    sim.run(0.02)
    expected = trim + sim.fns.diag_nvf("OL_AP") + sim.fns.alt_i() + sim.fns.diag_nvf("OL_AD")
    assert sim.guided_throttle == pytest.approx(expected)


def test_cold_steady_capture_keeps_release_recovery():
    sim = _sim(MODE_STEADY)
    sim.run(0.02)
    assert sim.fns.thrust_trim() == pytest.approx(0.8)
    assert sim.fns.alt_i() == 0.0
    sim.run(0.02)
    assert sim.guided_throttle == pytest.approx(0.8, abs=0.005)


@pytest.mark.parametrize("yaw_rate", [0.0, 3.0])
def test_vertical_damping_is_not_disabled_by_rotor_axis_yaw(yaw_rate: float):
    sim = _sim(MODE_STEADY)
    sim.gyro = [0.0, 0.0, yaw_rate]
    sim.run(0.04)
    assert sim.fns.diag_nvf("OL_AD") < 0.0
    still = _sim(MODE_STEADY)
    still.run(0.04)
    assert sim.fns.diag_nvf("OL_AD") == pytest.approx(still.fns.diag_nvf("OL_AD"))


@pytest.mark.parametrize("azimuth", [0.0, math.pi / 2, math.pi, -math.pi / 2])
@pytest.mark.parametrize("yaw", [0.0, 1.3])
def test_takeoff_lean_pulls_downwind_independently_of_body_yaw(azimuth: float, yaw: float):
    sim = _sim(MODE_TAKEOFF)
    sim.R = Rotation.from_euler("z", yaw).as_matrix()
    lean = math.radians(50.0)
    for name, value in (("RAWES_LEAN", lean), ("RAWES_AZ", azimuth)):
        sim.send_message(NamedValueFloat(name=name, value=value))
    sim.run(4.0)
    target = sim.guided_target
    assert target is not None
    R = Rotation.from_euler(
        "xyz", [target["roll_deg"], target["pitch_deg"], target["yaw_deg"]], degrees=True,
    ).as_matrix()
    expected = compute_takeoff_body_z(float(np.float32(lean)), float(np.float32(azimuth)))
    np.testing.assert_allclose(R[:, 2], expected, atol=1e-8)
    assert float(np.dot(-R[:2, 2], [math.cos(azimuth), math.sin(azimuth)])) > 0.0
    lua_bz = sim.fns.takeoff_body_z(lean, azimuth)
    np.testing.assert_allclose(
        [lua_bz.x(lua_bz), lua_bz.y(lua_bz), lua_bz.z(lua_bz)],
        compute_takeoff_body_z(lean, azimuth),
    )


def test_takeoff_lean_stays_level_below_clearance_and_slews_after_liftoff():
    sim = _sim(MODE_TAKEOFF)
    sim.pos_ned = [0.0, 0.0, -1.0]
    for name, value in (("RAWES_LEAN", math.radians(50.0)), ("RAWES_AZ", math.pi / 2)):
        sim.send_message(NamedValueFloat(name=name, value=value))
    sim.run(0.1)
    assert sim.fns.diag_nvf("BZG_D") == pytest.approx(1.0)
    sim.pos_ned = [0.0, 0.0, -2.0]
    sim.run(0.02)
    first_lean = math.acos(sim.fns.diag_nvf("BZG_D"))
    assert 0.0 < first_lean <= sim.fns.bz_slew() * 0.02


@pytest.mark.parametrize(
    ("lean", "azimuth"),
    [(math.radians(61), 0.0), (-0.1, 0.0), (float("nan"), 0.0),
     (0.5, None), (0.5, float("inf"))],
)
def test_takeoff_rejects_invalid_wind_capture_commands(lean: float, azimuth: float | None):
    sim = _sim(MODE_TAKEOFF)
    sim.send_message(NamedValueFloat(name="RAWES_LEAN", value=lean))
    if azimuth is not None:
        sim.send_message(NamedValueFloat(name="RAWES_AZ", value=azimuth))
    sim.run(0.04)
    assert any("invalid lean" in message for _, message in sim.messages)
    assert sim.guided_target is None


def test_takeoff_reentry_does_not_reuse_a_stale_lean_command():
    sim = _sim(MODE_TAKEOFF)
    for name, value in (("RAWES_LEAN", 0.5), ("RAWES_AZ", math.pi / 2)):
        sim.send_message(NamedValueFloat(name=name, value=value))
    sim.run(0.1)
    sim.set_param("RAWES_MODE", MODE_STEADY)
    sim.run(0.02)
    sim.set_param("RAWES_MODE", MODE_TAKEOFF)
    sim.run(0.04)
    assert sim.fns.diag_nvf("BZG_D") == pytest.approx(1.0)
