from __future__ import annotations

import io
import json

from groundstation.mavlink_log import MavlinkLogWriter


def test_writer_serializes_mavlink_binary_payloads() -> None:
    class Message:
        def to_dict(self) -> dict:
            return {
                "mavpackettype": "FILE_TRANSFER_PROTOCOL",
                "data": bytearray((1, 2, 255)),
            }

    output = io.StringIO()
    MavlinkLogWriter(output).write(Message(), last_time_boot_ms=0)

    record = json.loads(output.getvalue())
    assert record["data"] == [1, 2, 255]
