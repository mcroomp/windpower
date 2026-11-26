import struct
from types import SimpleNamespace
from unittest.mock import Mock

from pymavlink.dialects.v20 import ardupilotmega
import pytest

from tests.oneoff.probe_pymavlink_ftp import encode_raw_ftp_request, read_parameter


@pytest.mark.parametrize(
    ("framing", "frame_length"),
    [("mavlink2-trimmed", 39), ("mavlink2-full", 266), ("mavlink1", 262)],
)
def test_raw_ftp_frames_decode_with_upstream_parser(framing: str, frame_length: int) -> None:
    frame = encode_raw_ftp_request(framing, 7, 42, 190, 1, 1, 3)
    decoded = ardupilotmega.MAVLink(None).parse_char(frame)
    assert len(frame) == frame_length
    assert isinstance(decoded, ardupilotmega.MAVLink_file_transfer_protocol_message)
    assert (decoded.get_srcSystem(), decoded.get_srcComponent()) == (255, 190)
    assert (decoded.target_network, decoded.target_system, decoded.target_component) == (0, 1, 1)
    assert struct.unpack("<HBBBBBBI", bytes(decoded.payload[:12])) == (42, 0, 3, 12, 0, 0, 0, 0)
    assert bytes(decoded.payload[12:24]) == b"/APM/scripts"
    assert not any(decoded.payload[24:])


def test_parameter_control_ignores_other_names_and_sources() -> None:
    def reply(name: str, system: int = 1, component: int = 1) -> SimpleNamespace:
        return SimpleNamespace(
            param_id=name, param_value=1.0,
            get_srcSystem=lambda: system, get_srcComponent=lambda: component,
        )

    master = SimpleNamespace(
        mav=SimpleNamespace(param_request_read_send=Mock()),
        recv_match=Mock(side_effect=[
            reply("BRD_OPTIONS"),
            reply("MAV_SYSID", system=2),
            reply("MAV_SYSID", component=190),
            reply("MAV_SYSID"),
        ]),
    )
    assert read_parameter(master, 1, 1, "MAV_SYSID") == 1.0
    master.mav.param_request_read_send.assert_called_once_with(1, 1, b"MAV_SYSID", -1)
