"""
unified_ground.py -- Ground-side pumping comms adapters.

Marshals a PumpingGroundController's TensionCommand to the AP via one of three
comms adapters:

  DirectComms(ap)      Python simtest: calls ap.receive_command() directly
  LuaComms(inject)     Lua unit test: injects NV floats via send_named_float
  GcsComms(gcs)        SITL stack test: sends NAMED_VALUE_FLOAT via MAVLink

LuaComms and GcsComms both marshal TensionCommand to the same three NV pairs:
    RAWES_TEN     target/feed-forward tension [N] for gravity compensation
  RAWES_ALT     altitude target [m]
  RAWES_SUB     phase as integer (0=hold 1=reel-out 2=transition 3=reel-in)
"""

from __future__ import annotations

from simulation.pumping_planner import TensionCommand

_PHASE_TO_SUB: dict[str, int] = {
    "hold":       0,
    "reel-out":   1,
    "transition": 2,
    "reel-in":    3,
}


def _cmd_to_nv(cmd: TensionCommand) -> list[tuple[str, float]]:
    """Convert a TensionCommand to a list of (name, value) NV float pairs."""
    return [
        ("RAWES_TEN",  cmd.tension_target_n),   # target tension for gravity comp
        ("RAWES_ALT",  cmd.alt_m),
        ("RAWES_SUB",  float(_PHASE_TO_SUB.get(cmd.phase, 0))),
    ]


# ---------------------------------------------------------------------------
# Comms adapters
# ---------------------------------------------------------------------------

class DirectComms:
    """Delivers TensionCommand directly to a local MockArdupilot Python equivalent."""

    def __init__(self, ap) -> None:
        self._ap = ap

    def send(self, cmd: TensionCommand, dt: float) -> None:
        self._ap.receive_command(cmd, dt)


class NvComms:
    """Base for comms that marshal TensionCommand to NAMED_VALUE_FLOAT pairs."""

    def send_nv(self, name: str, value: float) -> None:
        raise NotImplementedError

    def send(self, cmd: TensionCommand, dt: float) -> None:
        for name, value in _cmd_to_nv(cmd):
            self.send_nv(name, value)


class GcsComms(NvComms):
    """Sends TensionCommand via MAVLink NAMED_VALUE_FLOAT (SITL stack tests).

    gcs: object with send_named_float(name: str, value: float) — e.g. RawesGCS.
    """

    def __init__(self, gcs) -> None:
        self._gcs = gcs

    def send_nv(self, name: str, value: float) -> None:
        self._gcs.send_named_float(name, value)


class LuaComms(NvComms):
    """Injects TensionCommand into Lua's named-value inbox via send_named_float.

    inject: callable(name: str, value: float) — e.g. RawesLua.send_named_float.
    """

    def __init__(self, inject) -> None:
        self._inject = inject

    def send_nv(self, name: str, value: float) -> None:
        self._inject(name, value)
