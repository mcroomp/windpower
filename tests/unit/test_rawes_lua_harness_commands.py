from groundstation.rawes_modes import CMD_ENTER_PASSIVE, enter_passive_params
from linkhub_client.messages import MavCmd, MavResult
from simulation.rawes_lua_harness import RawesLua


def test_command_ack_is_decoded_to_typed_command_and_result() -> None:
    sim = RawesLua(mode=2)
    sim.healthy = True
    sim.armed = True

    sim.send_command(CMD_ENTER_PASSIVE, enter_passive_params())
    sim.tick()

    # ENTER_PASSIVE is only accepted in RAWES_MODE=3.
    assert sim.command_acks == [{
        "command": MavCmd.USER_2,
        "result": MavResult.DENIED,
        "target_system": 255,
        "target_component": 190,
    }]
