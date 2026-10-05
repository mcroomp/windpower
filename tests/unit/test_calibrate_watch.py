"""Regression tests for calibration watch stream handlers."""

import math

from calibrate.watch import _watch_attitude
from linkhub_client import MessageBatch, SimClock
from linkhub_client.messages import Attitude

_CLOCK = SimClock(epoch=1, time_boot_ms=1, quality=None)


class _Log:
    def __init__(self):
        self.rows = []
        self.n_rows = 0

    def write_header(self, columns):
        self.columns = columns

    def row(self, values):
        self.rows.append(values)
        self.n_rows += 1


class _Session:
    _target_system = 1
    _target_component = 1

    def send_message(self, _message):
        pass

    def current_cursor(self):
        return "v1:0"

    def read_messages(self, _after, message_types, **_kwargs):
        if message_types == ["ATTITUDE", "HEARTBEAT", "STATUSTEXT"]:
            return MessageBatch((Attitude(
                roll=math.radians(10),
                pitch=math.radians(-5),
                yaw=math.radians(20),
                rollspeed=0.1,
                pitchspeed=-0.2,
                yawspeed=0.3,
            ),), "v1:1", _CLOCK)
        return MessageBatch((), "v1:1", _CLOCK)


def test_watch_attitude_handles_already_decoded_attitude(monkeypatch):
    clock = iter([0.0, 0.1, 0.2, 16.0])
    monkeypatch.setattr("calibrate.watch.time.monotonic", clock.__next__)
    monkeypatch.setattr("calibrate.run.decode_message", lambda message: message)
    log = _Log()

    _watch_attitude(_Session(), 10.0, log)

    assert log.rows
    assert log.rows[0][1:4] == ["10.000", "-5.000", "20.000"]
