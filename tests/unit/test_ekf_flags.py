import pytest

from groundstation.ekf_flags import EKF_FLAG_BITS, EKF_FLAGS, decode_flags, flags_to_mask
from groundstation.rawes_modes import (
    CMD_ENTER_GUIDED,
    CMD_ENTER_PASSIVE,
    send_rawes_command,
)
from linkhub_client.messages import EkfStatusFlags, MavCmd, MavResult


def test_flag_bits_cover_exactly_the_named_ekf_flag_masks() -> None:
    assert sorted(EKF_FLAG_BITS.values()) == sorted(EKF_FLAGS)


def test_flags_to_mask_matches_integer_decoder() -> None:
    flags = frozenset({
        EkfStatusFlags.ATTITUDE,
        EkfStatusFlags.POS_HORIZ_ABS,
        EkfStatusFlags("EKF_CONST_POS_MODE"),
    })

    mask = flags_to_mask(flags)

    assert mask == 0x0001 | 0x0010 | 0x0080
    assert decode_flags(mask) == "attitude, horiz_pos_abs, const_pos_mode"
    assert flags_to_mask(frozenset()) == 0


def test_flags_to_mask_rejects_unknown_flag() -> None:
    with pytest.raises(KeyError):
        flags_to_mask(frozenset({EkfStatusFlags("EKF_SOMETHING_NEW")}))


class _Gcs:
    def __init__(self, result: MavResult) -> None:
        self.result = result
        self.sent: list[tuple[MavCmd, list[float]]] = []

    def command(self, command, params, *, timeout):
        self.sent.append((command, params))
        return {"command": command, "result": self.result}


def test_rawes_commands_are_typed_mav_cmds() -> None:
    assert CMD_ENTER_GUIDED is MavCmd.USER_1
    assert CMD_ENTER_PASSIVE is MavCmd.USER_2


def test_send_rawes_command_accepts_only_accepted_result() -> None:
    gcs = _Gcs(MavResult.ACCEPTED)
    send_rawes_command(gcs, CMD_ENTER_PASSIVE, [0.3])
    assert gcs.sent == [(MavCmd.USER_2, [0.3])]

    with pytest.raises(RuntimeError, match="MAV_RESULT_DENIED"):
        send_rawes_command(_Gcs(MavResult.DENIED), CMD_ENTER_GUIDED)
