"""
pumping_planner.py -- Pumping planner: ground controller + communication protocol.

Design
------
Ground (10 Hz):
    PumpingGroundController.step(t_sim)
        -> ThrustCommand (forwarded to AP)

    ThrustCommand carries a thrust fraction [0..1]:
        0 = minimum tension (AP tilts body_z toward tether, collective at coll_min)
        1 = maximum tension (AP uses altitude-hold body_z, collective at coll_max)

    Reel-out: thrust ramps from 0 to target_thrust over thrust_ramp_s seconds.
    Transition / reel-in / hold: thrust = 0.
    Winch waits for tension < T_reel_in_start before reeling in.

AP (400 Hz in Python sim, ~50 Hz in Lua):
    ThrustApController.receive_command(cmd: ThrustCommand)  -- called at ~10 Hz
    ThrustApController.step(pos_ned, body_z, dt)
        -> (collective_rad, body_z_eq)

    body_z  = slerp(tether_dir, alt_hold, thrust)
    coll    = coll_min + thrust * (coll_max - coll_min),  slewed at coll_slew_rate

    AP owns all slew limiting -- no jerky movements, gives winch time to react.

Communication protocol (ground -> AP, 10 Hz):
    ThrustCommand.thrust  -- [0..1] tension target fraction
    ThrustCommand.alt_m   -- target altitude above anchor [m]
    ThrustCommand.phase   -- phase label for telemetry

Comms-loss failsafe:
    On dropout, thrust falls to 0 (minimum tension, tether-tilt mode).
    Winch applies its own safety independently.
"""

from __future__ import annotations

from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Command packet (crosses the MAVLink boundary)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ThrustCommand:
    """
    Command sent from ground to AP at ~10 Hz.

    thrust : float  0..1  0=min tension (tether-tilt, coll_min)
                          1=max tension (alt-hold, coll_max)
    alt_m  : float  target altitude above anchor [m]
    phase  : str    "hold" | "reel-out" | "transition" | "reel-in"
    """
    thrust : float
    alt_m  : float
    phase  : str


# ---------------------------------------------------------------------------
# Ground controller
# ---------------------------------------------------------------------------

class PumpingGroundController:
    """
    Ground-side pumping controller.

    Owns the phase schedule and emits ThrustCommand to the AP at ~10 Hz.

    Phase schedule (cyclic, repeating n_cycles times):
        0 .. t_reel_out                        : "reel-out"
        t_reel_out .. t_reel_out + t_transition: "transition"
        t_reel_out + t_transition .. t_cycle   : "reel-in"
        after n_cycles * t_cycle               : "hold"

    Thrust profile:
        reel-out   : ramp from 0 to target_thrust over thrust_ramp_s seconds
        transition : 0 (AP drives to minimum tension, winch holds)
        reel-in    : 0 (AP at minimum tension, winch reels in)
        hold       : 0

    Parameters
    ----------
    t_reel_out      : float  reel-out phase duration [s]
    t_reel_in       : float  reel-in + transition total duration [s]
    t_transition    : float  transition window at reel-out -> reel-in boundary [s]
    target_alt_m    : float  altitude to hold above anchor [m]
    n_cycles        : int    number of pumping cycles to run (0 = infinite)
    target_thrust   : float  peak thrust during reel-out [0..1]  (default 0.9)
    thrust_ramp_s   : float  ramp-up duration from 0 to target_thrust [s]
    """

    def __init__(
        self,
        t_reel_out   : float,
        t_reel_in    : float,
        t_transition : float,
        target_alt_m : float,
        n_cycles     : int   = 0,
        target_thrust: float = 0.9,
        thrust_ramp_s: float = 8.0,
    ) -> None:
        self._t_out       = float(t_reel_out)
        self._t_in        = float(t_reel_in)
        self._t_tr        = float(t_transition)
        self._alt_m       = float(target_alt_m)
        self._ncyc        = int(n_cycles)
        self._t_cyc       = self._t_out + self._t_in
        self._target      = float(target_thrust)
        self._ramp_s      = max(float(thrust_ramp_s), 1e-9)
        self._phase       : str   = "hold"
        self._phase_start : float = 0.0
        self._last_phase  : str   = ""

    @property
    def phase(self) -> str:
        """Current phase label."""
        return self._phase

    def step(self, t_sim: float) -> ThrustCommand:
        """
        10 Hz outer step.  Returns ThrustCommand for AP.

        Parameters
        ----------
        t_sim : elapsed simulation time [s]
        """
        phase = self._phase_at(t_sim)
        if phase != self._last_phase:
            self._phase_start = t_sim
            self._last_phase  = phase
        self._phase = phase

        if phase == "reel-out":
            t_in_phase = t_sim - self._phase_start
            thrust = min(self._target, t_in_phase / self._ramp_s)
        else:
            thrust = 0.0

        return ThrustCommand(thrust=thrust, alt_m=self._alt_m, phase=phase)

    # ── private helpers ────────────────────────────────────────────────────

    def _phase_at(self, t: float) -> str:
        if self._ncyc > 0 and t >= self._ncyc * self._t_cyc:
            return "hold"
        t_cyc = t % self._t_cyc
        if t_cyc < self._t_out:
            return "reel-out"
        elif t_cyc < self._t_out + self._t_tr:
            return "transition"
        else:
            return "reel-in"
