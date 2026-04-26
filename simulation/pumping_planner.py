"""
pumping_planner.py -- Pumping planner: ground controller + communication protocol.

Design
------
Ground (10 Hz):
    PumpingGroundController.step(t_sim, tension_measured_n)
        -> TensionCommand (forwarded to AP)

    TensionCommand carries:
        tension_setpoint_n : requested tether tension [N]
        tension_measured_n : current load-cell reading [N]  (AP uses as PI feedback)
        alt_m              : target altitude above anchor [m]
        phase              : phase label

    The AP has no tension sensor of its own.  The ground reads the load cell and
    packs both the setpoint and the measurement into each TensionCommand so the
    AP's TensionPI can close the loop using ground-transmitted feedback at 10 Hz.

Phase state machine (length-driven):
    reel-out   : exit when rest_length >= start_length + delta_l  (or safety timeout)
    transition : exit after t_transition seconds  (AP attitude change; winch already reeling)
    reel-in    : exit when rest_length <= start_length  (or safety timeout)
    hold       : entered after n_cycles complete

Winch strategy:
    reel-out   : target = start_length + delta_l,  tension = tension_ic
                 (generator pays out as AP raises tension above tension_ic)
    transition : target = start_length,  tension = tension_in
                 (winch starts reeling in immediately; AP drops tension, so winch
                  moves naturally once T < tension_in; no forced slack)
    reel-in    : same as transition (winch continues reeling in)
    hold       : target = rest_length (hold position)

Altitude ramp:
    Transition: ramp from hub_alt_m → reel-in target over t_transition.
    Reel-out start: ramp from hub_alt_m → IC altitude over t_transition.
    Reel-in: hold at rest_length * sin(el_reel_in_rad).
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Command packet (crosses the MAVLink boundary)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TensionCommand:
    """
    Command sent from ground to AP at ~10 Hz.

    tension_setpoint_n : float  requested tether tension [N]
    tension_measured_n : float  current load-cell reading [N]  (AP PI feedback)
    alt_m              : float  target altitude above anchor [m]
    phase              : str    "hold" | "reel-out" | "transition" | "reel-in"
    """
    tension_setpoint_n : float
    tension_measured_n : float
    alt_m              : float
    phase              : str


# ---------------------------------------------------------------------------
# Ground controller
# ---------------------------------------------------------------------------

class PumpingGroundController:
    """
    Ground-side pumping controller.

    Phase exits are driven by tether length (reel-out, reel-in) or time
    (transition).  t_reel_out and t_reel_in are safety timeouts only.

    Parameters
    ----------
    t_transition    : float  time for AP attitude change [s]  (primary)
    target_alt_m    : float  altitude to hold during reel-out [m]
    delta_l         : float  tether length paid out per cycle [m]
    n_cycles        : int    number of pumping cycles (0 = infinite)
    tension_out     : float  reel-out tension setpoint [N]
    tension_in      : float  reel-in / transition tension setpoint [N]
    tension_ic      : float  IC equilibrium tension [N]
    tension_ramp_s  : float  ramp from tension_ic to tension_out [s]
    el_reel_in_rad  : float  hub elevation target during reel-in [rad]
    t_reel_out_max  : float  safety timeout for reel-out phase [s]
    t_reel_in_max   : float  safety timeout for reel-in phase [s]
    """

    def __init__(
        self,
        t_transition     : float,
        target_alt_m     : float,
        delta_l          : float = 12.0,
        n_cycles         : int   = 0,
        tension_out      : float = 435.0,
        tension_in       : float = 226.0,
        tension_in_ap    : float = 150.0,
        tension_ic       : float = 300.0,
        tension_ramp_s   : float = 8.0,
        el_reel_in_rad   : float = 0.0,
        t_reel_out_max   : float = 300.0,
        t_reel_in_max    : float = 300.0,
    ) -> None:
        self._t_tr         = float(t_transition)
        self._alt_m        = float(target_alt_m)
        self._delta_l      = float(delta_l)
        self._el_reel_in   = float(el_reel_in_rad)
        self._ncyc         = int(n_cycles)
        self._tension_out  = float(tension_out)
        self._tension_in   = float(tension_in)        # winch threshold during reel-in
        self._tension_in_ap = float(tension_in_ap)   # AP setpoint during reel-in (< tension_in)
        self._tension_ic   = float(tension_ic)
        self._ramp_s       = max(float(tension_ramp_s), 1e-9)
        self._t_out_max    = float(t_reel_out_max)
        self._t_in_max     = float(t_reel_in_max)

        # State machine
        self._phase        : str   = "reel-out"
        self._phase_start  : float = 0.0
        self._cycle_count  : int   = 0       # completed cycles
        self._start_length : float = 0.0     # rest_length at reel-out start
        self._reel_out_tgt : float = 0.0     # start_length + delta_l
        self._initialized  : bool  = False   # True after first step()

        # Winch outputs
        self._winch_target_length  : float = 0.0
        self._winch_target_tension : float = float(tension_ic)

        # Altitude ramp state
        self._alt_ramp_start  : float = float(target_alt_m)
        self._alt_ramp_end    : float = float(target_alt_m)
        self._reel_out_ramp_s : float = float(t_transition)  # may extend to limit descent rate

    # ── public properties ──────────────────────────────────────────────────

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def cycle_count(self) -> int:
        """Number of completed pumping cycles."""
        return self._cycle_count

    @property
    def winch_target_length(self) -> float:
        return self._winch_target_length

    @property
    def winch_target_tension(self) -> float:
        return self._winch_target_tension

    # ── main step (10 Hz) ─────────────────────────────────────────────────

    def step(self, t_sim: float, tension_measured_n: float,
             rest_length: float = 0.0,
             hub_alt_m: float = 0.0) -> TensionCommand:
        """
        10 Hz outer step.  Returns TensionCommand for AP.

        Also updates winch_target_length and winch_target_tension.

        Parameters
        ----------
        t_sim              : elapsed simulation time [s]
        tension_measured_n : current load-cell tension reading [N]
        rest_length        : current tether rest length [m]
        hub_alt_m          : current hub altitude above anchor [m]
        """
        prev_phase = self._phase
        self._advance_phase(t_sim, rest_length)
        phase      = self._phase

        # On phase entry (or first call): initialise phase state
        if phase != prev_phase or not self._initialized:
            self._initialized = True
            self._phase_start = t_sim
            if phase == "reel-out":
                self._start_length   = rest_length
                self._reel_out_tgt   = rest_length + self._delta_l
                self._alt_ramp_start = hub_alt_m if hub_alt_m > 0.0 else self._alt_m
                self._alt_ramp_end   = self._alt_m
                # Limit commanded descent to 3 m/s to avoid tension spikes.
                descent = max(0.0, self._alt_ramp_start - self._alt_m)
                self._reel_out_ramp_s = max(self._t_tr, descent / 3.0)
            elif phase == "transition":
                self._alt_ramp_start = hub_alt_m if hub_alt_m > 0.0 else self._alt_m
                reel_in_alt          = (rest_length * math.sin(self._el_reel_in)
                                        if self._el_reel_in > 0.0 and rest_length > 0.0
                                        else self._alt_m)
                self._alt_ramp_end   = reel_in_alt

        t_in_phase = t_sim - self._phase_start

        # ── tension setpoint & winch targets ──────────────────────────────
        if phase == "reel-out":
            frac     = min(1.0, t_in_phase / self._ramp_s)
            setpoint = self._tension_ic + frac * (self._tension_out - self._tension_ic)
            self._winch_target_length  = self._reel_out_tgt
            # Generator pays out when tension rises above tension_ic (not peak tension_out).
            self._winch_target_tension = self._tension_ic

        elif phase in ("transition", "reel-in"):
            # AP drives tension to tension_in_ap (< tension_in) so the winch has
            # consistent headroom: v_cruise = kp*(tension_in - tension_in_ap) > 0.
            # Winch reels in from transition start; tension is high at transition
            # entry so winch is stopped until AP drives tension below tension_in.
            setpoint = self._tension_in_ap
            self._winch_target_length  = self._start_length
            self._winch_target_tension = self._tension_in

        else:  # hold
            setpoint = self._tension_ic
            self._winch_target_length  = rest_length
            self._winch_target_tension = self._tension_ic

        # ── altitude command ──────────────────────────────────────────────
        if self._el_reel_in > 0.0:
            if phase == "transition":
                frac  = min(1.0, t_in_phase / max(self._t_tr, 1e-9))
                alt_m = self._alt_ramp_start + frac * (self._alt_ramp_end - self._alt_ramp_start)
            elif phase == "reel-in":
                alt_m = (rest_length * math.sin(self._el_reel_in)
                         if rest_length > 0.0 else self._alt_m)
            elif phase == "reel-out":
                if t_in_phase < self._reel_out_ramp_s and self._alt_ramp_start > self._alt_m + 1.0:
                    frac  = min(1.0, t_in_phase / max(self._reel_out_ramp_s, 1e-9))
                    alt_m = self._alt_ramp_start + frac * (self._alt_ramp_end - self._alt_ramp_start)
                else:
                    alt_m = self._alt_m
            else:
                alt_m = self._alt_m
        else:
            alt_m = self._alt_m

        return TensionCommand(
            tension_setpoint_n = setpoint,
            tension_measured_n = float(tension_measured_n),
            alt_m              = alt_m,
            phase              = phase,
        )

    # ── private helpers ────────────────────────────────────────────────────

    def _advance_phase(self, t_sim: float, rest_length: float) -> None:
        """
        Update self._phase based on tether length (reel-out, reel-in) or
        elapsed time (transition).  Increments cycle count on reel-in exit.
        """
        t_in_phase = t_sim - self._phase_start

        if self._phase == "reel-out":
            # Primary exit: tether reached target length
            at_target = (self._reel_out_tgt > 0.0 and
                         rest_length >= self._reel_out_tgt - 0.05)
            timed_out = t_in_phase >= self._t_out_max
            if at_target or timed_out:
                self._phase = "transition"

        elif self._phase == "transition":
            # Exit after attitude-change window
            if t_in_phase >= self._t_tr:
                self._phase = "reel-in"

        elif self._phase == "reel-in":
            # Primary exit: tether back to start length
            at_target = (self._start_length > 0.0 and
                         rest_length <= self._start_length + 0.05)
            timed_out = t_in_phase >= self._t_in_max
            if at_target or timed_out:
                self._cycle_count += 1
                if self._ncyc > 0 and self._cycle_count >= self._ncyc:
                    self._phase = "hold"
                else:
                    self._phase = "reel-out"
