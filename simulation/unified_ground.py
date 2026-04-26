"""
unified_ground.py -- Ground-side pumping controller with pluggable comms.

UnifiedGroundController wraps PumpingGroundController + WinchController and
delivers TensionCommand to the AP via one of three comms adapters:

  DirectComms(ap)      Python simtest: calls ap.receive_command() directly
  LuaComms(inject)     Lua unit test: injects NV floats via send_named_float
  GcsComms(gcs)        SITL stack test: sends NAMED_VALUE_FLOAT via MAVLink

LuaComms and GcsComms both marshal TensionCommand to the same four NV pairs:
  RAWES_TSP   tension setpoint [N]
  RAWES_TEN     tension measured [N]  (also used by Lua for gravity compensation)
  RAWES_ALT     altitude target [m]
  RAWES_SUB     phase as integer (0=hold 1=reel-out 2=transition 3=reel-in)
"""

from __future__ import annotations

from pumping_planner import PumpingGroundController, TensionCommand
from winch import WinchController

_PHASE_TO_SUB: dict[str, int] = {
    "hold":       0,
    "reel-out":   1,
    "transition": 2,
    "reel-in":    3,
}


def _cmd_to_nv(cmd: TensionCommand) -> list[tuple[str, float]]:
    """Convert a TensionCommand to a list of (name, value) NV float pairs."""
    return [
        ("RAWES_TSP",  cmd.tension_setpoint_n),
        ("RAWES_TEN",    cmd.tension_measured_n),
        ("RAWES_ALT",    cmd.alt_m),
        ("RAWES_SUB",    float(_PHASE_TO_SUB.get(cmd.phase, 0))),
    ]


# ---------------------------------------------------------------------------
# Comms adapters
# ---------------------------------------------------------------------------

class DirectComms:
    """Delivers TensionCommand directly to a Python TensionApController."""

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


# ---------------------------------------------------------------------------
# Ground controller
# ---------------------------------------------------------------------------

class UnifiedGroundController:
    """
    Ground-side pumping controller combining PumpingGroundController + WinchController.

    Manages 10 Hz planner updates internally; caller steps at physics rate.

    Parameters
    ----------
    ground  : PumpingGroundController
    winch   : WinchController
    comms   : DirectComms | LuaComms | GcsComms
    dt_plan : float  planner step interval [s]  (default 1/10 = 0.1)
    """

    def __init__(
        self,
        ground:  PumpingGroundController,
        winch:   WinchController,
        comms,
        dt_plan: float = 0.1,
    ) -> None:
        self._ground  = ground
        self._winch   = winch
        self._comms   = comms
        self._dt_plan = float(dt_plan)
        self._t_next  = 0.0
        self._last_cmd: TensionCommand | None = None

    # ── properties ────────────────────────────────────────────────────────────

    @property
    def phase(self) -> str:
        return self._ground.phase

    @property
    def cycle_count(self) -> int:
        return self._ground.cycle_count

    @property
    def rest_length(self) -> float:
        return self._winch.rest_length

    @property
    def winch_speed_ms(self) -> float:
        return self._winch.speed_ms

    @property
    def last_cmd(self) -> TensionCommand | None:
        return self._last_cmd

    # ── main step (call at physics rate) ──────────────────────────────────────

    def step(
        self,
        t_sim:     float,
        tension_n: float,
        hub_alt_m: float,
        dt:        float,
    ) -> None:
        """
        Call at physics rate.  Manages 10 Hz planner and 400 Hz winch internally.

        t_sim     : elapsed simulation time [s]
        tension_n : current tether tension [N]
        hub_alt_m : current hub altitude above anchor [m]
        dt        : physics step interval [s]
        """
        if t_sim >= self._t_next:
            cmd = self._ground.step(
                t_sim, tension_n,
                rest_length = self._winch.rest_length,
                hub_alt_m   = hub_alt_m,
            )
            self._last_cmd = cmd
            self._t_next   = t_sim + self._dt_plan

            if self._ground.phase != "hold":
                self._comms.send(cmd, self._dt_plan)

            self._winch.set_target(
                self._ground.winch_target_length,
                self._ground.winch_target_tension,
            )

        self._winch.step(tension_n, dt)
