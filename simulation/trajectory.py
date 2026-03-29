"""
trajectory.py — Pluggable trajectory controllers for the RAWES pumping cycle.

Models the offboard MAVLink trajectory planner that runs on a ground station
or companion computer and sends commands to the custom ArduPilot Mode_RAWES.

Protocol
--------
Two packet types cross the MAVLink boundary:

    STATE packet  (Pixhawk → planner, ~10 Hz):
        "pos_enu"    np.ndarray [3]  — hub position ENU [m]        (LOCAL_POSITION_NED)
        "vel_enu"    np.ndarray [3]  — hub velocity ENU [m/s]      (LOCAL_POSITION_NED)
        "tension_n"  float           — tether tension [N]           (NAMED_VALUE_FLOAT)
        "t_free"     float           — free-flight elapsed time [s] (onboard timer)

    COMMAND packet  (planner → Pixhawk, ~10 Hz):
        "body_z_target"      np.ndarray [3] | None
            Desired disk axis in ENU.
            None  → natural tether-aligned equilibrium (no correction needed).
            Mode_RAWES always orbit-tracks the tether as its baseline at 400 Hz;
            body_z_target is only non-None when the planner wants a different
            orientation (e.g. reel-in tilt).  The planner does NOT manage orbit
            tracking — that runs entirely inside Mode_RAWES.
        "blend_alpha"        float  [0..1]
            0 = fully tether-aligned (natural orbit, no planner correction).
            1 = fully at body_z_target.
            The planner ramps this over t_transition seconds so Mode_RAWES
            transitions smoothly.
        "tension_setpoint_n" float  [N]
            Tension setpoint for the Mode_RAWES inner PI controller.
        "winch_speed_ms"     float  [m/s]
            +ve = pay out, −ve = reel in, 0 = hold.
        "phase"              str    — telemetry label ("hold"|"reel-out"|"reel-in")

Mode_RAWES responsibilities (NOT in this file):
    - Orbit tracking: compute tether-aligned body_z_eq at 400 Hz from current pos
    - Blend toward body_z_target by blend_alpha
    - Attitude error → cyclic tilt  (compute_swashplate_from_state)
    - Tension PI → collective        (TensionController)
    - Winch actuation                (aux PWM, with rest_length_min clamp)

Available controllers
---------------------
    HoldTrajectory()
        Returns natural equilibrium command every step — zero blend, zero winch.
        Mode_RAWES stays tether-aligned through its own orbit tracking.
        Use for: test_closed_loop_60s.py

    DeschutterTrajectory(t_reel_out, t_reel_in, t_transition,
                         v_reel_out, v_reel_in, tension_out, tension_in,
                         wind_enu, rest_length_min, xi_reel_in_deg=55.0)
        De Schutter (2018) pumping cycle.
        Reel-out: blend_alpha=0 (natural tether-aligned), tension=tension_out.
        Reel-in:  ramps blend_alpha 0→1 over t_transition s toward a body_z
                  at xi_reel_in_deg from wind, tension=tension_in.
        xi_reel_in_deg=55  — constrained from 90° for BEM validity (default).
        xi_reel_in_deg=None — no tilt change; blend_alpha stays 0 all cycle.
        Use for: test_deschutter_cycle.py (xi=55), test_pumping_cycle.py (xi=None)
"""

import math
import numpy as np


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class TrajectoryController:
    """Abstract base class — subclass and implement step()."""

    def step(self, state: dict, dt: float) -> dict:
        """
        Advance one planner step.

        Parameters
        ----------
        state : dict — STATE packet from Pixhawk (pos_enu, vel_enu, tension_n, t_free)
        dt    : float — timestep [s]

        Returns
        -------
        COMMAND packet dict with keys:
            body_z_target, blend_alpha, tension_setpoint_n, winch_speed_ms, phase
        """
        raise NotImplementedError(
            f"{type(self).__name__} must implement step()")


# ---------------------------------------------------------------------------
# HoldTrajectory
# ---------------------------------------------------------------------------

class HoldTrajectory(TrajectoryController):
    """
    Natural equilibrium — no correction, no winch.

    Sends blend_alpha=0 and body_z_target=None every step.  Mode_RAWES stays
    tether-aligned through its own orbit-tracking (no planner involvement).

    Use for: closed-loop stability tests where the pumping cycle is inactive.
    """

    def step(self, state: dict, dt: float) -> dict:
        return {
            "body_z_target":      None,   # Mode_RAWES uses tether-aligned baseline
            "blend_alpha":        0.0,
            "tension_setpoint_n": 0.0,
            "winch_speed_ms":     0.0,
            "phase":              "hold",
        }


# ---------------------------------------------------------------------------
# DeschutterTrajectory
# ---------------------------------------------------------------------------

class DeschutterTrajectory(TrajectoryController):
    """
    De Schutter (2018) reel-out / reel-in pumping cycle.

    The planner manages only mission-level decisions: phase timing, tension
    setpoints, winch speed, and the reel-in tilt target.  Orbit tracking runs
    entirely inside Mode_RAWES — the planner never touches it.

    Reel-out phase:
        blend_alpha = 0  →  Mode_RAWES stays tether-aligned naturally.
        tension_setpoint_n = tension_out.
        winch_speed_ms = +v_reel_out.

    Reel-in phase:
        blend_alpha ramps 0 → 1 over t_transition seconds.
        body_z_target = fixed ENU vector at xi_reel_in_deg from wind.
        tension_setpoint_n = tension_in.
        winch_speed_ms = −v_reel_in.

    Parameters
    ----------
    t_reel_out      : float — reel-out phase duration [s]
    t_reel_in       : float — reel-in phase duration [s]
    t_transition    : float — blend ramp duration at phase boundaries [s]
    v_reel_out      : float — winch pay-out speed [m/s]
    v_reel_in       : float — winch reel-in speed [m/s]
    tension_out     : float — reel-out tension setpoint [N]
    tension_in      : float — reel-in tension setpoint [N]
    wind_enu        : array — wind vector ENU [m/s]
    rest_length_min : float — informational floor; enforced by Mode_RAWES [m]
    xi_reel_in_deg  : float or None
        Angle between body_z and wind during reel-in [degrees].
        55.0 (default) — constrained from 90° to keep BEM valid (v_axial > 0).
        None — no tilt change; blend_alpha stays 0 (simple tension-only cycle).
    """

    def __init__(
        self,
        t_reel_out:      float,
        t_reel_in:       float,
        t_transition:    float,
        v_reel_out:      float,
        v_reel_in:       float,
        tension_out:     float,
        tension_in:      float,
        wind_enu:        np.ndarray,
        rest_length_min: float = 10.0,
        xi_reel_in_deg:  "float | None" = 55.0,
    ):
        self._t_reel_out   = float(t_reel_out)
        self._t_reel_in    = float(t_reel_in)
        self._t_transition = float(max(t_transition, 1e-6))
        self._v_reel_out   = float(v_reel_out)
        self._v_reel_in    = float(v_reel_in)
        self._tension_out  = float(tension_out)
        self._tension_in   = float(tension_in)
        self._t_cycle      = self._t_reel_out + self._t_reel_in

        # Reel-in body_z target: fixed ENU vector, computed once from wind.
        # None = no tilt change (xi_reel_in_deg=None).
        if xi_reel_in_deg is not None:
            xi_rad     = math.radians(float(xi_reel_in_deg))
            wind_horiz = np.array([float(wind_enu[0]), float(wind_enu[1]), 0.0])
            wh_norm    = np.linalg.norm(wind_horiz)
            wind_horiz = wind_horiz / wh_norm if wh_norm > 1e-6 else np.array([1.0, 0.0, 0.0])
            self._body_z_target: "np.ndarray | None" = (
                math.cos(xi_rad) * wind_horiz
                + math.sin(xi_rad) * np.array([0.0, 0.0, 1.0])
            )
        else:
            self._body_z_target = None   # no tilt — blend_alpha stays 0

    # ------------------------------------------------------------------
    def step(self, state: dict, dt: float) -> dict:
        t_free = float(state["t_free"])

        t_cyc     = t_free % self._t_cycle
        cyc       = int(t_free / self._t_cycle)
        phase_out = t_cyc < self._t_reel_out

        # Winch command
        winch_speed = self._v_reel_out if phase_out else -self._v_reel_in

        # Blend alpha: ramps during transitions, 0 elsewhere
        if self._body_z_target is None:
            # No tilt change requested — stay tether-aligned always
            blend_alpha   = 0.0
            body_z_target = None
        elif phase_out:
            if cyc > 0 and t_cyc < self._t_transition:
                # Blending back from reel-in toward tether-aligned (natural)
                blend_alpha = 1.0 - t_cyc / self._t_transition
            else:
                blend_alpha = 0.0
            body_z_target = self._body_z_target
        else:
            # Blending from tether-aligned toward reel-in target
            t_since     = t_cyc - self._t_reel_out
            blend_alpha = min(1.0, t_since / self._t_transition)
            body_z_target = self._body_z_target

        return {
            "body_z_target":      body_z_target,
            "blend_alpha":        blend_alpha,
            "tension_setpoint_n": self._tension_out if phase_out else self._tension_in,
            "winch_speed_ms":     winch_speed,
            "phase":              "reel-out" if phase_out else "reel-in",
        }
