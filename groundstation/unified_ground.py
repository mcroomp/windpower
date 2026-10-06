"""
unified_ground.py -- Ground-side pumping comms wire protocol + production adapter.

Marshals a PumpingGroundController's TensionCommand to the AP as
NAMED_VALUE_FLOAT pairs:

    RAWES_TEN     target/feed-forward tension [N] for gravity compensation
    RAWES_ALT     altitude target [m]
    RAWES_SUB     phase as integer (0=hold 1=reel-out 2=transition 3=reel-in)

  GcsComms(gcs)   SITL stack test / real hardware: sends NAMED_VALUE_FLOAT via MAVLink

The _cmd_to_nv marshalling here is the single source of truth for this wire
format. Lua unit tests reuse GcsComms with the Lua harness as the `gcs`;
simulation.unified_ground only holds the Python-simtest DirectComms.
"""

from __future__ import annotations

from linkhub_client.messages import NamedValueFloat
from groundstation.pumping_planner import TensionCommand

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


class GcsComms:
    """Sends TensionCommand via MAVLink NAMED_VALUE_FLOAT (SITL stack tests).

    gcs: object with send_message(msg) — e.g. LinkHubClient.
    """

    def __init__(self, gcs) -> None:
        self._gcs = gcs

    def send(self, cmd: TensionCommand, dt: float) -> None:
        for name, value in _cmd_to_nv(cmd):
            self._gcs.send_message(NamedValueFloat(name=name, value=value))
