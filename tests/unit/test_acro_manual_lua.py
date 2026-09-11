from groundstation.gcs import NamedValueFloat
from simulation.rawes_lua_harness import RawesLua


def _seed_manual(sim: RawesLua, roll: float, pitch: float, collective: float) -> None:
    sim.send_message(NamedValueFloat("RAWES_RLL", roll))
    sim.send_message(NamedValueFloat("RAWES_PIT", pitch))
    sim.send_message(NamedValueFloat("RAWES_COL", collective))


def test_acro_manual_latches_normalized_controls_and_reasserts_overrides():
    sim = RawesLua(mode=2)
    sim.vehicle_mode = 1
    sim.armed = True
    _seed_manual(sim, 0.4, -0.2, 0.6)

    sim.tick()
    first = (sim.ch_out[1], sim.ch_out[2], sim.ch_out[3])
    sim.run(2.0)

    assert first == (1700, 1400, 1600)
    assert (sim.ch_out[1], sim.ch_out[2], sim.ch_out[3]) == first
    assert sim.armed
    assert sim.guided_target is None
    assert sim.guided_rate_target is None
    assert sim.guided_throttle is None


def test_acro_manual_uses_asymmetric_rc_calibration_and_reversal():
    sim = RawesLua(
        mode=2,
        RC1_MIN=1100,
        RC1_TRIM=1450,
        RC1_MAX=1900,
        RC1_REVERSED=1,
        RC3_MIN=1200,
        RC3_MAX=1800,
        RC3_REVERSED=1,
    )
    sim.vehicle_mode = 1
    _seed_manual(sim, 0.5, 0.0, 0.25)

    sim.tick()

    assert sim.ch_out[1] == 1275
    assert sim.ch_out[2] == 1500
    assert sim.ch_out[3] == 1650


def test_acro_manual_disarms_outside_acro():
    sim = RawesLua(mode=2)
    sim.vehicle_mode = 4
    sim.armed = True
    _seed_manual(sim, 0.0, 0.0, 0.5)

    sim.tick()

    assert not sim.armed


def test_acro_manual_disarms_without_complete_seed_or_flybar_mode():
    missing_seed = RawesLua(mode=2)
    missing_seed.vehicle_mode = 1
    missing_seed.armed = True
    missing_seed.tick()
    assert not missing_seed.armed

    no_flybar = RawesLua(mode=2, H_FLYBAR_MODE=0)
    no_flybar.vehicle_mode = 1
    no_flybar.armed = True
    _seed_manual(no_flybar, 0.0, 0.0, 0.5)
    no_flybar.tick()
    assert not no_flybar.armed


def test_acro_manual_disarms_with_collective_expo():
    sim = RawesLua(mode=2, IM_ACRO_COL_EXP=0.3)
    sim.vehicle_mode = 1
    sim.armed = True
    _seed_manual(sim, 0.0, 0.0, 0.5)

    sim.tick()

    assert not sim.armed


def test_leaving_acro_manual_releases_rc_overrides():
    sim = RawesLua(mode=2)
    sim.vehicle_mode = 1
    _seed_manual(sim, 0.4, -0.2, 0.6)
    sim.tick()

    sim.set_param("mode", 0)
    sim.tick()

    assert sim.ch_out[1] == 0
    assert sim.ch_out[2] == 0
    assert sim.ch_out[3] == 0


def test_acro_manual_runs_yaw_trim_observer():
    sim = RawesLua(mode=2)
    sim.vehicle_mode = 1
    sim.armed = True
    sim.gyro = [0.0, 0.0, -0.5]
    sim.set_srv_out(36, 1500)
    _seed_manual(sim, 0.0, 0.0, 0.5)

    sim.run(0.1)

    assert sim.get_param("H_YAW_TRIM") > 0.0
