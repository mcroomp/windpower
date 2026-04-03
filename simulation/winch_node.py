"""
WinchNode -- simulation of the ground-based winch controller node.

On hardware this is a separate MAVLink node that:
  - runs the winch reel speed PID (WinchController)
  - measures tether tension via load cell
  - measures tether length via encoder
  - measures wind via co-located anemometer

Protocol boundary
-----------------
The ground station planner communicates with this node ONLY through two methods:

  winch_node.get_telemetry()          -> dict  (planner reads)
  winch_node.receive_command(speed)            (planner writes)

The mediator (physics side) calls:

  winch_node.update_sensors(tension, wind)     (feed physics outputs in)

No other cross-boundary access is allowed.  This mirrors the hardware
architecture where all data crosses as MAVLink messages and the planner
has no direct access to tether.py or the true wind vector.
"""

import numpy as np
from winch import WinchController


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


class WinchNode:
    """Simulated winch controller node.

    Encapsulates WinchController + Anemometer and enforces the protocol
    boundary between the physics simulation and the ground station planner.

    Mediator (physics side)
    -----------------------
    Call update_sensors() after tether.compute() each physics step to feed
    the load cell and anemometer readings.  Then use receive_command() to
    execute the winch speed command from the planner, and rest_length to
    synchronise tether.rest_length.

    Planner (ground station side)
    ------------------------------
    Call get_telemetry() to read tension, tether length, and wind.  Call
    receive_command() to send a winch speed setpoint.  No other access to
    physics state is available.
    """

    def __init__(self, winch: WinchController, anemometer: Anemometer):
        self._winch      = winch
        self._anemometer = anemometer
        self._tension_n: float       = 0.0
        self._wind_ned:  np.ndarray  = np.zeros(3)

    # ---- physics side (mediator only) ---------------------------------------

    def update_sensors(self, tension_n: float, wind_world_ned: np.ndarray) -> None:
        """Feed load cell reading and ambient wind into the node.

        Called by the mediator after tether.compute() each step.
        tension_n       -- tether tension from tether._last_info ["tension"] [N]
        wind_world_ned  -- true ambient wind NED [m/s] (NOT passed to planner)
        """
        self._tension_n = float(tension_n)
        self._wind_ned  = self._anemometer.measure(wind_world_ned)

    def receive_command(self, winch_speed_ms: float, dt: float) -> None:
        """Execute a winch speed command from the planner.

        Delegates to WinchController which enforces tension safety limits.
        winch_speed_ms -- reel speed [m/s]: +ve = pay out, -ve = reel in
        dt             -- physics timestep [s]
        """
        self._winch.step(winch_speed_ms, self._tension_n, dt)

    @property
    def rest_length(self) -> float:
        """Current tether rest length [m].

        Used by mediator to keep tether.rest_length in sync after each
        winch step.  Not part of the planner-facing protocol.
        """
        return self._winch.rest_length

    # ---- planner side (ground station only) ---------------------------------

    def get_telemetry(self) -> dict:
        """Return the node's MAVLink telemetry packet.

        Keys
        ----
        tension_n       -- load cell reading [N]
        tether_length_m -- encoder reading [m]
        wind_ned        -- anemometer reading [NED, m/s]
        """
        return {
            "tension_n":       self._tension_n,
            "tether_length_m": self._winch.tether_length_m,
            "wind_ned":        self._wind_ned.copy(),
        }
