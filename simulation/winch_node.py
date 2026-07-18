"""
winch_node -- simulation of the ground-based winch controller node.

On hardware this is a separate MAVLink node at the anchor that:
  - runs the fast tension-governing loop (GovernedWinchController)
  - measures tether tension via load cell
  - measures tether length via drum encoder
  - measures wind via a co-located anemometer

Protocol boundary
-----------------
The ground station planner communicates with this node ONLY through the cable
boundary:

  winch_node.exchange(WinchCommand) -> WinchTelemetry

WinchCommand/WinchTelemetry are the genuine wire protocol and live in
groundstation.winch_protocol (importable by both the real ground station and
this simulated node). GovernedWinchNode itself stands in for winch-node
firmware that does not exist yet, so it stays here in simulation/.

The mediator (physics side) additionally calls:

  winch_node.update_sensors(tension, wind)     (feed physics outputs in)
  winch_node.step(dt)                          (advance the governing loop)
  winch_node.rest_length                       (sync tether.rest_length)
  winch_node.telemetry()                       (host-side telemetry snapshot)

No hub altitude / position / attitude ever crosses the cable boundary.
"""

import numpy as np

from simulation.winch import GovernedWinchController
from groundstation.winch_protocol import WinchCommand, WinchTelemetry


class Anemometer:
    """Ground-level anemometer mounted on the winch node.

    Samples the ambient wind vector at a fixed height above the anchor.
    In simulation the wind is spatially uniform so height only matters if
    a shear model is added.  Gaussian measurement noise can be enabled.

    On hardware this is replaced by a live MAVLink stream from the anemometer.
    """

    def __init__(self, height_m: float = 3.0, noise_std_ms: float = 0.0):
        """
        height_m      -- anemometer mounting height above anchor [m]
        noise_std_ms  -- 1-sigma horizontal wind noise [m/s], default 0 (ideal)
        """
        self.height_m     = float(height_m)
        self.noise_std_ms = float(noise_std_ms)

    def measure(self, wind_world_ned: np.ndarray) -> np.ndarray:
        """Return anemometer reading [NED, m/s].

        In simulation wind_world_ned is the true ambient wind.  The anemometer
        reading is the same value plus optional horizontal Gaussian noise.
        """
        reading = np.array(wind_world_ned, dtype=float)
        if self.noise_std_ms > 0.0:
            reading[0] += np.random.normal(0.0, self.noise_std_ms)
            reading[1] += np.random.normal(0.0, self.noise_std_ms)
        return reading


# ===========================================================================
# Split-comms winch node (GovernedWinchController on the winch node)
# ===========================================================================
#
# Two logical parts coupled by a single boundary:
#
#   GCS / planner node  --(WinchCommand: targets only)-->  Winch node
#   Winch node          --(WinchTelemetry: sensed only)->  GCS / planner node
#
# The fast tension-governing control loop (GovernedWinchController) lives on
# the winch node, co-located with the load cell, and runs at the physics rate.
# The channel carries ONLY slow target commands down and winch-measurable
# telemetry up -- it is never in the fast feedback path.
#
# No-leak guarantee: the planner can only call exchange() and only ever sees
# a WinchTelemetry, whose fields are all quantities the drum/anchor hardware
# physically senses (load cell, drum encoder, co-located anemometer).  Hub
# altitude / position / attitude never cross this boundary.


class GovernedWinchNode:
    """Winch + load cell + anemometer node hosting the fast governing loop.

    On hardware this is a self-contained node at the anchor: a winch drum with
    a load cell and a co-located anemometer, running the tension-governing
    control loop locally.  The ground-station planner talks to it ONLY through
    the cable boundary ``exchange(WinchCommand) -> WinchTelemetry``.

    Planner side (across the cable)
    -------------------------------
    exchange(cmd)        apply target command, return winch-measurable telemetry

    Physics side (mediator / simulation only -- NOT the protocol)
    -------------------------------------------------------------
    update_sensors(T, w) feed load-cell tension and ambient wind into the node
    step(dt)             advance the fast governing loop one physics tick
    rest_length          drum encoder, used to sync tether.rest_length
    """

    def __init__(self, winch: GovernedWinchController,
                 anemometer: "Anemometer | None" = None):
        self._winch      = winch
        self._anemometer = anemometer
        self._tension_n: float      = 0.0
        self._wind_ned:  np.ndarray = np.zeros(3)

    # ---- physics side (mediator / sim only) ---------------------------------

    def update_sensors(self, tension_n: float, wind_world_ned=None) -> None:
        """Feed the load-cell reading and ambient wind into the node.

        Called at the physics rate before step().  tension_n is the tether
        load-cell reading [N]; wind_world_ned is the true ambient wind [NED]
        sampled by the co-located anemometer (optional).
        """
        self._tension_n = float(tension_n)
        if self._anemometer is not None and wind_world_ned is not None:
            self._wind_ned = self._anemometer.measure(wind_world_ned)

    def step(self, dt: float) -> None:
        """Advance the fast governing loop one tick using the local load cell."""
        self._winch.step(self._tension_n, float(dt))

    @property
    def rest_length(self) -> float:
        """Drum encoder reading [m] (mediator syncs tether.rest_length)."""
        return self._winch.rest_length

    def telemetry(self) -> WinchTelemetry:
        """Read-only winch telemetry snapshot (host/transport side).

        Used by the mediator to forward periodic telemetry up the cable
        without re-applying a command.  Same payload exchange() returns.
        """
        return self._telemetry()

    # ---- planner side (the cable boundary) ----------------------------------

    def exchange(self, cmd: WinchCommand) -> WinchTelemetry:
        """Apply a target command and return current winch telemetry.

        This is the ONLY planner-facing method -- pure marshalling, so no hub
        or physics state can cross the boundary.
        """
        self._winch.set_command(cmd.cruise_v, cmd.tension_target)
        return self._telemetry()

    def _telemetry(self) -> WinchTelemetry:
        w = self._wind_ned
        return WinchTelemetry(
            tension_n    = float(self._tension_n),
            rest_length  = float(self._winch.rest_length),
            speed_ms     = float(self._winch.speed_ms),
            net_energy_j = float(self._winch.net_energy_j),
            wind_ned     = (float(w[0]), float(w[1]), float(w[2])),
        )
