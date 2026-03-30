"""
winch.py — Ground-station Winch Controller.

Models the winch controller that sits on the ground alongside the trajectory
planner.  It translates planner reel-speed commands into tether rest_length
changes, enforces a tension safety cutoff, and provides tether_length_m and
tension_n back to the planner over the local ground link.

Architecture (raws_mode.md §2):
    Trajectory Planner  →  winch_speed_ms  →  WinchController
    WinchController     →  tension_n, tether_length_m  →  Trajectory Planner

The Pixhawk is never involved in winch control.
"""


class WinchController:
    """
    Ground-station winch controller.

    Translates planner reel-speed commands into tether rest_length changes,
    enforcing a tension safety cutoff and a minimum-length floor.

    Parameters
    ----------
    rest_length      : float — initial unstretched tether length [m]
    tension_safety_n : float — max tension before paying-out stops [N].
                       Protects tether from breakage during load spikes.
    min_length       : float — minimum rest_length during reel-in [m].  Default 10.0.
    """

    def __init__(
        self,
        rest_length:      float,
        tension_safety_n: float,
        min_length:       float = 10.0,
    ):
        self.rest_length       = float(rest_length)
        self._tension_safety_n = float(tension_safety_n)
        self._min_length       = float(min_length)

    # ------------------------------------------------------------------
    def step(self, winch_speed_ms: float, tension_n: float, dt: float) -> None:
        """
        Advance the winch by one timestep.

        Parameters
        ----------
        winch_speed_ms : float — commanded reel rate [m/s].
                         +ve = pay out, −ve = reel in, 0 = hold.
        tension_n      : float — current tether tension [N] (from load cell).
        dt             : float — timestep [s].
        """
        speed = float(winch_speed_ms)

        # Safety: stop paying out if tension exceeds limit
        if speed > 0.0 and tension_n > self._tension_safety_n:
            speed = 0.0

        new_length = self.rest_length + speed * float(dt)

        # Floor: never reel in below minimum length
        if speed < 0.0:
            new_length = max(self._min_length, new_length)

        self.rest_length = new_length

    # ------------------------------------------------------------------
    @property
    def tether_length_m(self) -> float:
        """Current tether rest length [m] — encoder reading."""
        return self.rest_length
