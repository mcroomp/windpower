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


def record_cyclic_overrides(sim):
    writes = {1: [], 2: []}
    rc = sim._lua.globals().rc
    for number in writes:
        channel = rc.get_channel(rc, number)
        original = channel.set_override

        def record(channel, pwm, number=number, original=original):
            writes[number].append(int(pwm))
            original(channel, pwm)

        channel.set_override = record
    return writes


@pytest.mark.parametrize(("roll", "pitch", "expected"), [
    (1, 0, (2000, 1500)),
    (0, -1, (1500, 1000)),
    (0.6, -0.8, (1800, 1100)),
])
def test_setup_does_not_pulse_neutral_between_cyclic_commands(roll, pitch, expected):
    sim = collective_sim()
    send_collective(sim, 0.5)
    sim.tick()
    # Observe all writes; final ch_out alone hides sub-tick neutral pulses.
    writes = record_cyclic_overrides(sim)
    for _ in range(4):
        sim.send_message(NamedValueFloat(name="RAWES_RLL", value=roll))
        sim.send_message(NamedValueFloat(name="RAWES_PIT", value=pitch))
        send_collective(sim, 0.5)
        sim.run(0.5)
    assert writes[1] == [expected[0]] * 200
    assert writes[2] == [expected[1]] * 200
    assert sim.get_param("mode") == MODE_COLLECTIVE
    assert not sim.armed


@pytest.mark.parametrize("failure", ["armed", "configuration", "timeout"])
def test_setup_failure_writes_only_neutral_cyclic(failure):
    sim = collective_sim()
    sim.send_message(NamedValueFloat(name="RAWES_RLL", value=0.6))
    sim.send_message(NamedValueFloat(name="RAWES_PIT", value=-0.8))
    send_collective(sim, 0.7)
    sim.tick()
    if failure == "armed":
        sim.armed = True
    elif failure == "configuration":
        sim.set_param("H_SV_MAN", 2)
    else:
        sim.run(2.99)
    writes = record_cyclic_overrides(sim)
    sim.tick()
    assert writes == {1: [1500], 2: [1500]}
    assert sim.ch_out[3] == 1505
    assert sim.ch_out[8] == 1000
    assert sim.get_param("mode") == 0
    assert not sim.armed


def test_collective_supports_bounded_cyclic_without_arming_or_guided_control():
    sim = collective_sim()
    sim.send_message(NamedValueFloat(name="RAWES_RLL", value=1))
    sim.send_message(NamedValueFloat(name="RAWES_PIT", value=-1))
    sim.send_message(NamedValueFloat(name="RAWES_THR", value=1))
    send_collective(sim, 0.7)
    sim.tick()

    assert sim.ch_out[1] == pytest.approx(1854, abs=1)
    assert sim.ch_out[2] == pytest.approx(1146, abs=1)
    assert sim.ch_out[3] == 1700
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


def test_cyclic_expires_independently_while_collective_is_refreshed():
    sim = collective_sim()
    sim.send_message(NamedValueFloat(name="RAWES_RLL", value=1))
    sim.send_message(NamedValueFloat(name="RAWES_PIT", value=0.5))
    send_collective(sim, 0.7)
    sim.tick()
    assert sim.ch_out[1] > 1500
    assert sim.ch_out[2] > 1500
    for _ in range(3):
        send_collective(sim, 0.7)
        sim.run(1)
    assert sim.get_param("mode") == MODE_COLLECTIVE
    assert (sim.ch_out[1], sim.ch_out[2], sim.ch_out[3]) == (1500, 1500, 1700)


def test_setup_discards_cyclic_seeds_on_exit_and_reentry():
    sim = collective_sim()
    sim.send_message(NamedValueFloat(name="RAWES_RLL", value=1))
    send_collective(sim, 0.5)
    sim.tick()
    assert sim.ch_out[1] == 2000
    sim.set_param("mode", 0)
    sim.tick()
    sim.set_param("mode", MODE_COLLECTIVE)
    send_collective(sim, 0.5)
    sim.tick()
    assert (sim.ch_out[1], sim.ch_out[2]) == (1500, 1500)


@pytest.mark.parametrize("name", ["RAWES_RLL", "RAWES_PIT"])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_setup_rejects_nonfinite_cyclic(name, value):
    sim = collective_sim()
    sim.send_message(NamedValueFloat(name=name, value=value))
    send_collective(sim, 0.5)
    sim.tick()
    assert (sim.ch_out[1], sim.ch_out[2]) == (1500, 1500)
    assert sim.has_message("rejected non-finite cyclic")


def test_armed_failure_neutralizes_cyclic_and_collective():
    sim = collective_sim()
    sim.send_message(NamedValueFloat(name="RAWES_RLL", value=1))
    send_collective(sim, 0.8)
    sim.tick()
    sim.armed = True
    sim.tick()
    assert not sim.armed
    assert (sim.ch_out[1], sim.ch_out[2], sim.ch_out[3], sim.ch_out[8]) == (1500, 1500, 1505, 1000)
