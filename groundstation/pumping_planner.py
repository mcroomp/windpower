"""
pumping_planner.py -- Pumping planner: ground controller + communication protocol.

Design
------
Ground (10 Hz):
    PumpingGroundController.step(t_sim, tension_measured_n, rest_length, hub_alt_m)
        -> TensionCommand (forwarded to AP)

    TensionCommand carries:
        tension_target_n   : target/feed-forward tension [N] for AP body-z
        alt_m              : target altitude above anchor [m] (CONSTANT = IC alt)
        phase              : phase label

    The AP has no tension sensor of its own.  The ground reads the load cell for
    the winch/phase state machine, but sends the desired phase tension to the AP
    so gravity-compensated body-z does not chase transient tether dynamics.
    Collective is local AP altitude PID; ground does not command thrust.

This is the proven logic from the test_pump_cycle_lua simtest: a two-phase,
length-driven cycle with a CONSTANT commanded altitude (the IC altitude above
the anchor).  The kite holds altitude while the winch pays out and reels in the
tether; the per-phase tension target is both the AP body-z feed-forward and the
winch tension governor target.

Phase state machine (length-driven, with safety timeouts):
    hold     : pre-capture, and after all cycles complete.  Begins the first
               reel-out once notify_captured() has fired and capture_settle_s
               has elapsed.
    reel-out : exit when rest_length >= start_length + delta_l  (or t_reel_out_max)
    reel-in  : exit when rest_length <= start_length            (or t_reel_in_max)

Winch strategy (the planner serves two winch backends):
    GovernedWinchController (velocity cruise) -> winch_target_velocity
    WinchController          (length target)  -> winch_target_length
    Both share                                -> winch_target_tension

    reel-out : length = start_length + delta_l, velocity = +v_cruise_out,
               tension = tension_out
    reel-in  : length = start_length,           velocity = -v_cruise_in,
               tension = tension_in
    hold     : length = rest_length,            velocity = 0,
               tension = tension_ic
"""

from __future__ import annotations

from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Command packet (crosses the MAVLink boundary)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TensionCommand:
    """
    Command sent from ground to AP at ~10 Hz.

    tension_target_n   : float  target/feed-forward tension [N] for AP gravity comp
    alt_m              : float  target altitude above anchor [m] (CONSTANT = IC alt)
    phase              : str    "hold" | "reel-out" | "reel-in"
    """
    tension_target_n   : float
    alt_m              : float
    phase              : str


# ---------------------------------------------------------------------------
# Ground controller
# ---------------------------------------------------------------------------

class PumpingGroundController:
    """
    Ground-side pumping controller (the proven test_pump_cycle_lua logic).

    A two-phase, length-driven cycle (hold / reel-out / reel-in) gated on Lua
    steady capture.  Commands a CONSTANT target altitude (the IC altitude above
    the anchor) in every phase -- the kite holds altitude while the winch pays
    out and reels in the tether.  Per-phase tension is both the AP body-z
    feed-forward AND the winch tension governor target.

    The planner serves two winch backends:
      - GovernedWinchController (velocity cruise): use winch_target_velocity.
      - WinchController          (length target):  use winch_target_length.
    Both share winch_target_tension.

    Phase exits (length-driven, with safety timeouts):
      reel-out : rest_length >= start_length + delta_l   (or t_reel_out_max)
      reel-in  : rest_length <= start_length             (or t_reel_in_max)
      hold     : pre-capture, and after n_cycles complete

    Capture gating:
      Stays in 'hold' until notify_captured(t) is called AND capture_settle_s
      has elapsed, then begins the first reel-out.  start_length (the cycle
      baseline) is latched once, when pumping begins.

    Parameters
    ----------
    target_alt_m     : float  constant commanded altitude above anchor [m] (IC alt)
    delta_l          : float  tether length paid out per cycle [m]
    n_cycles         : int    number of pumping cycles (0 = infinite)
    tension_out      : float  reel-out tension target [N]  (AP FF + winch)
    tension_in       : float  reel-in tension target [N]   (AP FF + winch)
    tension_ic       : float  IC / hold tension target [N]
    v_cruise_out     : float  reel-out cruise velocity [m/s] (GovernedWinch)
    v_cruise_in      : float  reel-in cruise speed [m/s] (applied as -v_cruise_in)
    t_reel_out_max   : float  safety timeout for reel-out phase [s]
    t_reel_in_max    : float  safety timeout for reel-in phase [s]
    capture_settle_s : float  hold time after capture before pumping begins [s]
    tension_ramp_s   : float  ramp duration for tension target changes [s]
                              (0 = step change, default 2.0 s)
    """

    def __init__(
        self,
        target_alt_m     : float,
        delta_l          : float = 12.0,
        n_cycles         : int   = 0,
        tension_out      : float = 300.0,
        tension_in       : float = 100.0,
        tension_ic       : float = 300.0,
        v_cruise_out     : float = 0.5,
        v_cruise_in      : float = 0.5,
        t_reel_out_max   : float = 120.0,
        t_reel_in_max    : float = 300.0,
        capture_settle_s : float = 2.0,
        tension_ramp_s   : float = 2.0,
    ) -> None:
        self._alt_m        = float(target_alt_m)
        self._delta_l      = float(delta_l)
        self._ncyc         = int(n_cycles)
        self._tension_out  = float(tension_out)
        self._tension_in   = float(tension_in)
        self._tension_ic   = float(tension_ic)
        self._v_cruise_out = float(v_cruise_out)
        self._v_cruise_in  = float(v_cruise_in)
        self._t_out_max    = float(t_reel_out_max)
        self._t_in_max     = float(t_reel_in_max)
        self._settle_s     = float(capture_settle_s)
        self._tension_ramp_s = float(tension_ramp_s)

        # State machine.  Starts in 'hold' (pre-capture); pumping begins only
        # after notify_captured() + capture_settle_s.
        self._phase        : str   = "hold"
        self._phase_start  : float = 0.0
        self._cycle_count  : int   = 0       # completed cycles
        self._start_length : float = 0.0     # cycle baseline (latched at pump start)
        self._initialized  : bool  = False   # True after first step()
        self._cycles_done  : bool  = False   # True once all n_cycles complete
        self._capture_t    : float | None = None  # first steady-capture time [s]

        # Winch outputs
        self._winch_target_length   : float = 0.0
        self._winch_target_tension  : float = float(tension_ic)
        self._winch_target_velocity : float = 0.0
        # Ramped tension state (slews toward the phase target)
        self._tension_current       : float = float(tension_ic)
        self._prev_step_t           : float | None = None

    # ── capture gate ─────────────────────────────────────────────────────────

    def notify_captured(self, t_sim: float) -> None:
        """Record the first Lua steady-capture time; starts the settle countdown."""
        if self._capture_t is None:
            self._capture_t = float(t_sim)

    @property
    def captured(self) -> bool:
        return self._capture_t is not None

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

    @property
    def winch_target_velocity(self) -> float:
        """Cruise velocity [m/s] for a GovernedWinchController (+out/-in/0=hold)."""
        return self._winch_target_velocity

    # ── main step (10 Hz) ─────────────────────────────────────────────────

    def step(self, t_sim: float, tension_measured_n: float,
             rest_length: float = 0.0,
             hub_alt_m: float = 0.0) -> TensionCommand:
        """
        10 Hz outer step.  Returns TensionCommand for AP.

        Also updates winch_target_length, winch_target_tension and
        winch_target_velocity.

        Parameters
        ----------
        t_sim              : elapsed simulation time [s]
        tension_measured_n : current load-cell tension reading [N]
        rest_length        : current tether rest length [m]
        hub_alt_m          : current hub altitude above anchor [m]
        """
        if not self._initialized:
            self._initialized = True
            self._phase_start = t_sim
            self._start_length = rest_length

        self._advance_phase(t_sim, rest_length)
        phase = self._phase

        # ── per-phase tension target (step change — sent to Lua as RAWES_TEN) ──
        if phase == "reel-out":
            t_target = self._tension_out
        elif phase == "reel-in":
            t_target = self._tension_in
        else:  # hold
            t_target = self._tension_ic

        # ── ramp winch_target_tension toward phase target ─────────────────
        # winch governor sees the smoothed value; Lua receives t_target
        # (step change) and applies its own ramp via RAWES_TRP.
        dt = (t_sim - self._prev_step_t) if self._prev_step_t is not None else 0.0
        self._prev_step_t = t_sim
        if self._tension_ramp_s > 0.0 and dt > 0.0:
            alpha = dt / self._tension_ramp_s
            self._tension_current += alpha * (t_target - self._tension_current)
        else:
            self._tension_current = float(t_target)

        # ── winch length target (length-driven WinchController) ───────────
        if phase == "reel-out":
            self._winch_target_length = self._start_length + self._delta_l
        elif phase == "reel-in":
            self._winch_target_length = self._start_length
        else:  # hold
            self._winch_target_length = rest_length

        # ── winch velocity command (GovernedWinchController) ──────────────
        if phase == "reel-out":
            self._winch_target_velocity = +self._v_cruise_out
        elif phase == "reel-in":
            self._winch_target_velocity = -self._v_cruise_in
        else:  # hold
            self._winch_target_velocity = 0.0

        self._winch_target_tension = self._tension_current

        # Altitude is CONSTANT in every phase: the kite holds the IC altitude
        # while the winch pumps the tether.
        # tension_target_n carries the step-change target (unramped) so Lua
        # can apply its own RAWES_TRP ramp independently.
        return TensionCommand(
            tension_target_n = float(t_target),
            alt_m            = self._alt_m,
            phase            = phase,
        )

    # ── private helpers ────────────────────────────────────────────────────

    def _advance_phase(self, t_sim: float, rest_length: float) -> None:
        """
        Update self._phase from tether length (length-driven exits) with
        safety timeouts.  Increments cycle count on reel-in exit.
        """
        t_in_phase = t_sim - self._phase_start

        if self._phase == "hold":
            # Pre-capture, or all cycles complete.  Begin the first reel-out
            # once capture has fired and the settle time has elapsed.
            if (not self._cycles_done
                    and self._capture_t is not None
                    and (t_sim - self._capture_t) >= self._settle_s):
                self._start_length = rest_length   # latch cycle baseline once
                self._phase        = "reel-out"
                self._phase_start  = t_sim

        elif self._phase == "reel-out":
            at_target = rest_length >= self._start_length + self._delta_l - 0.05
            timed_out = t_in_phase >= self._t_out_max
            if at_target or timed_out:
                self._phase       = "reel-in"
                self._phase_start = t_sim

        elif self._phase == "reel-in":
            at_target = rest_length <= self._start_length + 0.05
            timed_out = t_in_phase >= self._t_in_max
            if at_target or timed_out:
                self._cycle_count += 1
                if self._ncyc > 0 and self._cycle_count >= self._ncyc:
                    self._phase       = "hold"
                    self._cycles_done = True
                else:
                    self._phase = "reel-out"
                self._phase_start = t_sim
