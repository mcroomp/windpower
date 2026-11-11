"""
unified_ground.py -- Ground-side pumping comms adapters (simtest/test-only).

Marshals a PumpingGroundController's TensionCommand to the AP via a test-only
comms adapter:

  DirectComms(ap)      Python simtest: calls ap.receive_command() directly

The real production adapter (GcsComms) and the _cmd_to_nv wire
marshalling live in groundstation/unified_ground.py. Lua unit tests use GcsComms
with the Lua harness as the `gcs`, since it implements send_message() too.
"""

from __future__ import annotations

from groundstation.pumping_planner import TensionCommand
from typing import Protocol


class _CommandReceiver(Protocol):
    def receive_command(self, cmd: TensionCommand, dt_cmd: float) -> None: ...


class DirectComms:
    """Delivers TensionCommand directly to a local MockArdupilot Python equivalent."""

    def __init__(self, ap: _CommandReceiver) -> None:
        self._ap = ap

    def send(self, cmd: TensionCommand, dt: float) -> None:
        self._ap.receive_command(cmd, dt)
