"""
landing_planner.py -- Ground-station landing planner for RAWES vertical descent.

Manages the three-phase landing sequence that follows a De Schutter reel-in.
Designed to be called by the ground station at each physics step after the
pumping cycle completes, with the same step(state_pkt, dt) interface as
DeschutterPlanner.

Landing physics
---------------
At the end of the De Schutter reel-in body_z is at xi=80 deg from the
(horizontal) wind vector.  Since the wind is horizontal that puts the disk
only ~10 deg from horizontal -- leveling is essentially instant.  The nearly-
vertical tether then supports >95% of hub weight throughout descent, so the
tether acts as a passive altitude-hold mechanism rather than a source of
tension spikes.

A spiral descent is deliberately avoided: as the tether shortens during a
tethered orbit the hub speeds up (angular momentum conservation).  At short
tether lengths the orbital speed exceeds the reel-in rate, the tether goes
slack, and tension spikes when it snaps taut.  A vertical drop directly above
the anchor avoids this entirely.

Descent rate controller
-----------------------
TensionPI reacts to tension error.  If the hub descends faster than the
reel-in rate the tether goes slack, TensionPI sees near-zero tension and
commands max collective; the tether snaps taut and the cycle repeats.  The
descent rate controller reacts to hub vz directly (LOCAL_POSITION_NED) and
holds descent at v_land regardless of tether state:

    vz_error       = hub_vel_z - v_land      (NED; +ve = descending too fast)
    collective_rad = clip(col_cruise + kp_vz * vz_error, col_min, col_max)

col_cruise is the collective required to support the hub's weight with a
horizontal disk (hover equilibrium = col_min_reel_in at xi=80 deg = 0.079 rad).

Phase sequence
--------------
  leveling  -- body_z slerps from initial_body_z toward BZ_LEVEL at
               body_z_slew_rate_rad_s.  Ends when disk is within
               level_thresh_deg of horizontal, or after t_level_max seconds.

  descent   -- body_z_eq fixed at [0,0,-1].  Descent rate controller holds
               hub vz at v_land.  Winch reels in at v_land.  Ends when
               tether_length_m <= min_tether_m.

  final_drop -- collective = 0, winch holds.  Hub drops onto catch device.
               Phase is terminal; planner continues returning this phase.

Protocol
--------
State packet consumed (same fields as DeschutterPlanner):

    body_z          np.ndarray [3]  current rotor axis NED unit vector
                                    (from ATTITUDE_QUATERNION col 2)
    vel_ned         np.ndarray [3]  hub velocity NED [m/s]
                                    (from LOCAL_POSITION_NED)
    tether_length_m float           winch encoder reading [m]
                                    (from WinchNode.get_telemetry())

Command dict returned:

    collective_rad  float           direct collective [rad]
    winch_speed_ms  float           [m/s]: -ve = reel in, 0 = hold
    body_z_eq       np.ndarray [3]  rate-limited attitude setpoint NED
    phase           str             "leveling" | "descent" | "final_drop"
"""
from __future__ import annotations

import numpy as np

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller import slerp_body_z

_BZ_LEVEL = np.array([0.0, 0.0, -1.0])   # NED up = horizontal disk


class LandingPlanner:
    """
    Ground-station landing planner: leveling + vertical descent + final drop.

    Parameters
    ----------
    initial_body_z       : np.ndarray [3]  body_z when landing begins (unit vec, NED)
    v_land               : float   target descent rate [m/s], positive
    col_cruise           : float   base collective at horizontal-disk hover [rad]
                                   should equal col_min_reel_in at xi=80 deg (~0.079 rad)
    kp_vz                : float   descent rate proportional gain [rad/(m/s)]
    col_min_rad          : float   collective floor [rad]
    col_max_rad          : float   collective ceiling [rad]
    body_z_slew_rate     : float   max body_z rotation rate [rad/s]
    level_thresh_deg     : float   xi < this -> leveling complete [deg]
    t_level_max          : float   leveling phase timeout [s]
    min_tether_m         : float   tether length that triggers final_drop [m]
    """

    def __init__(
        self,
        initial_body_z:    np.ndarray,
        v_land:            float,
        col_cruise:        float,
        kp_vz:             float,
        col_min_rad:       float,
        col_max_rad:       float,
        body_z_slew_rate:  float,
        level_thresh_deg:  float,
        t_level_max:       float,
        min_tether_m:      float,
    ):
        self._body_z_eq   = np.array(initial_body_z, dtype=float)
        self._body_z_eq  /= np.linalg.norm(self._body_z_eq)

        self._v_land      = float(v_land)
        self._col_cruise  = float(col_cruise)
        self._kp_vz       = float(kp_vz)
        self._col_min     = float(col_min_rad)
        self._col_max     = float(col_max_rad)
        self._slew_rate   = float(body_z_slew_rate)
        self._level_cos   = float(np.cos(np.radians(level_thresh_deg)))
        self._t_level_max = float(t_level_max)
        self._min_tether  = float(min_tether_m)

        self._phase       = "leveling"
        self._t_phase     = 0.0   # time elapsed in current phase

    # ------------------------------------------------------------------
    @property
    def phase(self) -> str:
        """Current phase name: "leveling" | "descent" | "final_drop"."""
        return self._phase

    # ------------------------------------------------------------------
    def step(self, state_pkt: dict, dt: float) -> dict:
        """
        Advance the planner by one timestep.

        Parameters
        ----------
        state_pkt : dict  -- must contain body_z, vel_ned, tether_length_m
        dt        : float -- timestep [s]

        Returns
        -------
        dict with collective_rad, winch_speed_ms, body_z_eq, phase
        """
        body_z_cur = np.asarray(state_pkt["body_z"],  dtype=float)
        vel_ned    = np.asarray(state_pkt["vel_ned"], dtype=float)
        tether_len = float(state_pkt["tether_length_m"])

        # ── Phase transitions ─────────────────────────────────────────
        if self._phase == "leveling":
            self._t_phase += dt
            xi_cos = float(np.dot(body_z_cur, _BZ_LEVEL))   # cos(xi from horiz)
            leveled = xi_cos >= self._level_cos
            timed_out = self._t_phase >= self._t_level_max
            if leveled or timed_out:
                self._phase   = "descent"
                self._t_phase = 0.0

        elif self._phase == "descent":
            if tether_len <= self._min_tether:
                self._phase   = "final_drop"
                self._t_phase = 0.0

        # ── Body-z setpoint ───────────────────────────────────────────
        if self._phase in ("leveling", "descent"):
            self._body_z_eq = slerp_body_z(
                self._body_z_eq, _BZ_LEVEL, self._slew_rate, dt)
        else:
            self._body_z_eq = _BZ_LEVEL.copy()

        # ── Collective and winch ──────────────────────────────────────
        if self._phase in ("leveling", "descent"):
            vz_error       = float(vel_ned[2]) - self._v_land   # +ve = too fast
            collective_rad = float(np.clip(
                self._col_cruise + self._kp_vz * vz_error,
                self._col_min, self._col_max))
            winch_speed_ms = -self._v_land
        else:   # final_drop
            collective_rad = 0.0
            winch_speed_ms = 0.0

        return {
            "collective_rad": collective_rad,
            "winch_speed_ms": winch_speed_ms,
            "body_z_eq":      self._body_z_eq.copy(),
            "phase":          self._phase,
        }
