"""
landing_planner.py -- Ground-station landing planner for RAWES vertical descent.

Manages the two-phase landing sequence that follows a De Schutter reel-in.
Designed to be called at each physics step after the pumping cycle completes,
with the same step(state_pkt, dt) interface as DeschutterPlanner.

Design rationale
----------------
The previous design leveled the disk to body_z=[0,0,-1] (xi=90 deg from wind)
before descending.  This is physically problematic:

  1. xi=90 deg is outside the valid range of SkewedWakeBEM (~85 deg limit).
  2. On a horizontal disk in horizontal wind, the tether restoring torque
     M = r_attach x F_tether is zero (body_z parallel to tether at touchdown),
     so there is no passive stabilisation against wind tilt.
  3. BEM predicts rolling moments of ~300 N*m at xi=90 deg -- well above the
     ~160 N*m cyclic authority -- so simulation stability is an artefact of
     the out-of-range model.

A tether-tracking approach (body_z = pos/|pos|) also fails: the large upward
aero lift (186 N vs 49 N weight at xi=80 deg) causes the hub to swing toward
being directly above the anchor, driving xi toward 90 deg.

Current design: fixed-orientation descent
-----------------------------------------
body_z_eq is held at its value when landing begins (the end-of-reel-in
orientation, typically xi=80 deg from wind) throughout the entire descent.
This achieves:

  - xi stays at the natural orbit-equilibrium angle (~80 deg) where BEM is
    valid and the aero forces are well-modelled throughout.

  - The AcroController maintains the orientation against the tether restoring
    torque (which grows as the hub descends toward the anchor).  The cyclic
    authority (~160 N*m) exceeds the restoring torque at landing tensions.

  - The wind H-force is balanced by the tether's lateral component (pendulum
    effect).  As the tether shortens, the hub naturally tracks toward directly
    above the anchor: touchdown East offset ~ tether_length * sin(eq_angle).

  - Body_z does NOT approach [0,0,-1], keeping BEM in its valid range and
    the restoring torque non-zero and stabilising throughout.

Energy balance during descent
------------------------------
The rotor generates more lift than weight at xi=80 deg (186 N vs 49 N).
The winch tension provides the controlled downward force to overcome the
excess lift and drive the descent.  This is the "tether provides autorotation
energy" concept: wind autorotates the rotor; the winch controls descent.

Phase sequence
--------------
  descent    -- body_z_eq fixed at initial_body_z (xi~80 deg).
                Descent rate controller: col = col_cruise + kp_vz * vz_error.
                Winch reels in at v_land.  Ends when tether_length_m <= min_tether_m.

  final_drop -- collective=0, winch holds.  Hub drops last ~1 m onto catch pad.
                Phase is terminal.

State packet fields consumed
-----------------------------
    body_z          np.ndarray [3]   current disk axis NED unit vector
    vel_ned         np.ndarray [3]   hub velocity NED [m/s]
    tether_length_m float            winch encoder reading [m]
    (pos_ned, tension_n not required for this design)

Command dict returned
---------------------
    collective_rad  float            direct collective [rad]
    winch_speed_ms  float            [m/s]: negative = reel in, 0 = hold
    body_z_eq       np.ndarray [3]   fixed attitude setpoint NED (initial_body_z)
    phase           str              "descent" | "final_drop"
"""
from __future__ import annotations

import numpy as np
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class LandingPlanner:
    """
    Ground-station landing planner: fixed-orientation vertical descent + final drop.

    Parameters
    ----------
    initial_body_z       : np.ndarray [3]   body_z when landing begins (unit vec, NED)
                                            Held fixed throughout descent as body_z_eq.
    v_land               : float    target descent rate [m/s], positive (downward)
    col_cruise           : float    base collective at orbit-equilibrium hover [rad]
                                    should equal col_min_reel_in (~0.079 rad at xi=80 deg)
    kp_vz                : float    descent rate proportional gain [rad/(m/s)]
    col_min_rad          : float    collective floor [rad]
    col_max_rad          : float    collective ceiling [rad]
    min_tether_m         : float    tether length that triggers final_drop [m]
    """

    def __init__(
        self,
        initial_body_z:  np.ndarray,
        v_land:          float,
        col_cruise:      float,
        kp_vz:           float,
        col_min_rad:     float,
        col_max_rad:     float,
        min_tether_m:    float,
        # Ignored (kept for API compatibility with old callers)
        body_z_slew_rate: float = 0.4,
        anchor_ned:       "np.ndarray | None" = None,
        tension_target_n: float = 150.0,
        k_winch:          float = 0.005,
        v_winch_max:      float = 1.5,
        **_kwargs,
    ):
        self._body_z_eq  = np.array(initial_body_z, dtype=float)
        self._body_z_eq /= np.linalg.norm(self._body_z_eq)

        self._v_land     = float(v_land)
        self._col_cruise = float(col_cruise)
        self._kp_vz      = float(kp_vz)
        self._col_min    = float(col_min_rad)
        self._col_max    = float(col_max_rad)
        self._min_tether = float(min_tether_m)

        self._phase      = "descent"

    # ------------------------------------------------------------------
    @property
    def phase(self) -> str:
        """Current phase name: "descent" | "final_drop"."""
        return self._phase

    # ------------------------------------------------------------------
    def step(self, state_pkt: dict, dt: float) -> dict:
        """
        Advance the planner by one timestep.

        Parameters
        ----------
        state_pkt : dict  -- must contain vel_ned, tether_length_m
                            (body_z, pos_ned, tension_n accepted but not used)
        dt        : float -- timestep [s]

        Returns
        -------
        dict with collective_rad, winch_speed_ms, body_z_eq, phase
        """
        vel_ned    = np.asarray(state_pkt["vel_ned"], dtype=float)
        tether_len = float(state_pkt["tether_length_m"])

        # ── Phase transition ──────────────────────────────────────────
        if self._phase == "descent" and tether_len <= self._min_tether:
            self._phase = "final_drop"

        # ── Body-z setpoint: fixed at initial orientation ─────────────
        # body_z_eq does not change — the AcroController maintains the
        # xi=80 deg disk orientation against the tether restoring torque.

        # ── Collective and winch ──────────────────────────────────────
        if self._phase == "descent":
            vz_error       = float(vel_ned[2]) - self._v_land   # +ve = too fast
            collective_rad = float(np.clip(
                self._col_cruise + self._kp_vz * vz_error,
                self._col_min, self._col_max))
            winch_speed_ms = -self._v_land   # constant reel-in speed
        else:   # final_drop
            collective_rad = 0.0
            winch_speed_ms = 0.0

        return {
            "collective_rad": collective_rad,
            "winch_speed_ms": winch_speed_ms,
            "body_z_eq":      self._body_z_eq.copy(),
            "phase":          self._phase,
        }
