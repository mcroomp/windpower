"""
unified_ground.py -- Ground-side pumping comms adapters (simtest/test-only).

Marshals a PumpingGroundController's TensionCommand to the AP via one of two
test-only comms adapters:

  DirectComms(ap)      Python simtest: calls ap.receive_command() directly
  LuaComms(inject)     Lua unit test: injects NV floats via send_named_float

The real production adapter (GcsComms) and the NvComms base + _cmd_to_nv wire
marshalling live in groundstation/unified_ground.py.
"""

from __future__ import annotations

from groundstation.pumping_planner import TensionCommand
from groundstation.unified_ground import NvComms


class DirectComms:
    """Delivers TensionCommand directly to a local MockArdupilot Python equivalent."""

    def __init__(self, ap) -> None:
        self._ap = ap

    def send(self, cmd: TensionCommand, dt: float) -> None:
        self._ap.receive_command(cmd, dt)


class LuaComms(NvComms):
    """Injects TensionCommand into Lua's named-value inbox via send_named_float.

    inject: callable(name: str, value: float) — e.g. RawesLua.send_named_float.
    """

    def __init__(self, inject) -> None:
        self._inject = inject

    def send_nv(self, name: str, value: float) -> None:
        self._inject(name, value)

