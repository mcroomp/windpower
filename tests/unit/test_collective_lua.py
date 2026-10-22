import pytest

from groundstation.rawes_modes import MODE_COLLECTIVE
from linkhub_client.messages import NamedValueFloat
from simulation.rawes_lua_harness import RawesLua


def collective_sim(**params):
    sim = RawesLua(
        mode=MODE_COLLECTIVE,
        **{"H_TAIL_TYPE": 3, "SERVO9_FUNCTION": 36, "H_SV_MAN": 0, **params},
    )
    sim.vehicle_mode = 1
    return sim


def send_collective(sim, value):
    sim.send_message(NamedValueFloat(name="RAWES_COL", value=value))


def test_collective_only_moves_collective_without_arming_or_guided_control():
    sim = collective_sim()
    sim.send_message(NamedValueFloat(name="RAWES_RLL", value=1))
    sim.send_message(NamedValueFloat(name="RAWES_PIT", value=-1))
    sim.send_message(NamedValueFloat(name="RAWES_THR", value=1))
    send_collective(sim, 0.7)
    sim.tick()

    assert (sim.ch_out[1], sim.ch_out[2], sim.ch_out[3]) == (1500, 1500, 1700)
    assert (sim.ch_out[4], sim.ch_out[8]) == (1500, 1000)
    assert not sim.armed
    assert sim.arm_call_count == 0
    assert sim.guided_target is None
    assert sim.guided_rate_target is None
    assert sim.guided_throttle is None


@pytest.mark.parametrize(("value", "expected"), [(-1, 1000), (0, 1000), (1, 2000), (2, 2000)])
def test_collective_clamps_to_normalized_limits(value, expected):
    sim = collective_sim()
    send_collective(sim, value)
    sim.tick()
    assert sim.ch_out[3] == expected


def test_collective_uses_rc_calibration_and_reversal():
    sim = collective_sim(RC3_MIN=1200, RC3_MAX=1800, RC3_REVERSED=1)
    send_collective(sim, 0.25)
    sim.tick()
    assert sim.ch_out[3] == 1650


def test_collective_waits_at_neutral_and_times_out_exactly_after_three_seconds():
    sim = collective_sim()
    sim.tick()
    assert sim.ch_out[3] == 1505
    assert sim.get_param("mode") == MODE_COLLECTIVE
    sim.run(2.98)
    assert sim.get_param("mode") == MODE_COLLECTIVE
    sim.run(0.02)
    assert sim.get_param("mode") == 0
    assert sim.has_message("RAWES_COL timeout")


def test_collective_keepalive_refreshes_even_for_unchanged_values():
    sim = collective_sim()
    for _ in range(5):
        send_collective(sim, 0.7)
        sim.run(1.0)
        assert sim.get_param("mode") == MODE_COLLECTIVE
        assert sim.ch_out[3] == 1700
    sim.run(2.01)
    assert sim.get_param("mode") == 0
    assert sim.ch_out[3] == 1505
    assert sim.ch_out[8] == 1000


def test_collective_discards_previous_values_on_reentry_and_returns_to_neutral():
    sim = collective_sim()
    send_collective(sim, 0.8)
    sim.tick()
    sim.set_param("mode", 0)
    sim.tick()
    assert sim.ch_out[3] == 1505
    sim.set_param("mode", MODE_COLLECTIVE)
    sim.tick()
    assert sim.ch_out[3] == 1505


def test_collective_releases_override_when_leaving_for_another_mode():
    sim = collective_sim()
    send_collective(sim, 0.8)
    sim.tick()
    sim.set_param("mode", 1)
    sim.tick()
    assert sim.ch_out[3] == 0


def test_collective_disarms_and_neutralizes_if_armed():
    sim = collective_sim()
    send_collective(sim, 0.8)
    sim.tick()
    sim.armed = True
    sim.tick()
    assert not sim.armed
    assert sim.get_param("mode") == 0
    assert sim.ch_out[3] == 1505
    assert sim.ch_out[8] == 1000
    assert sim.get_param("H_YAW_TRIM") == 0
    assert sim.has_message("vehicle armed")


@pytest.mark.parametrize(("name", "value"), [
    ("H_FLYBAR_MODE", 0), ("H_SV_MAN", 2), ("IM_ACRO_COL_EXP", 0.3),
    ("H_TAIL_TYPE", 0), ("SERVO9_FUNCTION", 0),
])
def test_collective_rejects_invalid_configuration(name, value):
    sim = collective_sim(**{name: value})
    send_collective(sim, 0.8)
    sim.tick()
    assert sim.get_param("mode") == 0
    assert sim.ch_out[3] == 1505
    assert sim.ch_out[8] == 1000
    assert sim.has_message("invalid setup configuration")


def test_collective_rejects_non_acro_mode():
    sim = collective_sim()
    sim.vehicle_mode = 20
    sim.tick()
    assert sim.get_param("mode") == 0
    assert sim.vehicle_mode == 1


def test_collective_ignores_stale_manual_seed():
    sim = collective_sim()
    sim.set_param("mode", 0)
    send_collective(sim, 0.9)
    sim.tick()
    sim.set_param("mode", MODE_COLLECTIVE)
    sim.tick()
    assert sim.ch_out[3] == 1505


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_collective_rejects_nonfinite_commands_without_refreshing_watchdog(value):
    sim = collective_sim()
    send_collective(sim, value)
    sim.tick()
    assert sim.ch_out[3] == 1505
    assert sim.has_message("rejected non-finite RAWES_COL")
    sim.run(3.0)
    assert sim.get_param("mode") == 0
