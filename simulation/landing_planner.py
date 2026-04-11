"""
landing_planner.py -- Ground-station landing planner for RAWES vertical descent.

Phase sequence
--------------
  descent    -- body_z_eq fixed at initial_body_z (xi~80 deg).
                Collective: col_cruise + kp_vz * vz_error  (descent rate controller).
                Winch:      tension-PI + geometry feed-forward  (see below).
                Ends when tether_length_m <= min_tether_m.

  final_drop -- collective=0, winch holds.  Hub drops last ~1 m onto catch pad.

Winch control design
--------------------
Open-loop reel-in at constant v_land causes slack at the pumping->landing
transition: the hub arrives in orbit with significant horizontal velocity that
briefly drives the hub toward the anchor faster than v_land, collapsing the
small tether extension left by the De Schutter reel-in.

Two layered fixes:

1. Tension-PI
   winch_speed = -v_land + k_winch * (tension_n - tension_target)

   When tension < target (tether loose): reel in faster.
   When tension > target (tether tight): pay out to reduce tension.
   This replaces the open-loop -v_land command with active tension control.

2. Geometry feed-forward (anti-slack guard)
   dL/dt = vel_ned . unit_tether  (rate hub-anchor distance changes)
   If dL/dt < PI_speed (hub approaching faster than PI commands):
       winch_speed = max(dL/dt, -v_land_max)   -- track hub; keep extension >= 0

   This ensures rest_length never decreases slower than L, preventing slack
   during fast orbital passes regardless of the PI state.

State packet fields consumed
-----------------------------
    vel_ned         np.ndarray [3]   hub velocity NED [m/s]   (required)
    tether_length_m float            winch encoder reading [m] (required)
    tension_n       float            load-cell tension [N]     (for PI; defaults to tension_target)
    pos_ned         np.ndarray [3]   hub position NED [m]      (for feed-forward; falls back to open-loop)
    body_z, omega_spin               accepted but not used

Command dict returned
---------------------
    collective_rad  float
    winch_speed_ms  float   negative = reel in, positive = pay out, 0 = hold
    body_z_eq       np.ndarray [3]
    phase           str     "descent" | "final_drop"
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
    initial_body_z   : np.ndarray [3]
    v_land           : float   target descent rate [m/s], positive = downward
    col_cruise       : float   base collective [rad] = col_min_reel_in at xi=80 deg
    kp_vz            : float   descent rate gain [rad/(m/s)]
    col_min_rad      : float
    col_max_rad      : float
    min_tether_m     : float   tether length at which final_drop begins [m]
    anchor_ned       : np.ndarray [3] | None
                               Anchor position NED.  Required for geometry
                               feed-forward; falls back to tension-PI only if None.
    tension_target_n : float   winch PI tension setpoint [N].  Should be close
                               to the natural tether tension during descent (~80 N).
    k_winch          : float   winch PI gain [m/s per N].
    v_pay_out        : float   maximum pay-out speed [m/s].  Limits how fast the
                               winch can lengthen the tether to shed tension.
    """

    def __init__(
        self,
        initial_body_z:   np.ndarray,
        v_land:           float,
        col_cruise:       float,
        kp_vz:            float,
        col_min_rad:      float,
        col_max_rad:      float,
        min_tether_m:     float,
        anchor_ned:       "np.ndarray | None" = None,
        tension_target_n: float = 80.0,
        k_winch:          float = 0.005,
        v_pay_out:        float = 0.5,
        # Ignored (kept for API compatibility with old callers)
        body_z_slew_rate: float = 0.4,
        **_kwargs,
    ):
        self._body_z_eq = np.array(initial_body_z, dtype=float)
        self._body_z_eq /= np.linalg.norm(self._body_z_eq)

        self._v_land        = float(v_land)
        self._col_cruise    = float(col_cruise)
        self._kp_vz         = float(kp_vz)
        self._col_min       = float(col_min_rad)
        self._col_max       = float(col_max_rad)
        self._min_tether    = float(min_tether_m)
        self._anchor        = (np.asarray(anchor_ned, dtype=float)
                               if anchor_ned is not None else None)
        self._T_target      = float(tension_target_n)
        self._k_winch       = float(k_winch)
        self._v_pay_out     = float(v_pay_out)
        self._v_land_max    = 3.0 * self._v_land   # geometry feed-forward cap

        self._phase         = "descent"

    # ------------------------------------------------------------------
    @property
    def phase(self) -> str:
        return self._phase

    # ------------------------------------------------------------------
    def step(self, state_pkt: dict, dt: float) -> dict:
        vel_ned    = np.asarray(state_pkt["vel_ned"], dtype=float)
        tether_len = float(state_pkt["tether_length_m"])
        tension_n  = float(state_pkt.get("tension_n", self._T_target))

        # ── Phase transition ──────────────────────────────────────────
        if self._phase == "descent" and tether_len <= self._min_tether:
            self._phase = "final_drop"

        # ── Collective and winch ──────────────────────────────────────
        if self._phase == "descent":
            # Descent rate controller (collective)
            vz_error       = float(vel_ned[2]) - self._v_land
            collective_rad = float(np.clip(
                self._col_cruise + self._kp_vz * vz_error,
                self._col_min, self._col_max))

            # Tension-PI winch:
            #   tension < target -> reel in faster (more negative)
            #   tension > target -> pay out (less negative or positive)
            pi_speed = -self._v_land + self._k_winch * (tension_n - self._T_target)
            pi_speed = float(np.clip(pi_speed, -self._v_land_max, self._v_pay_out))

            # Geometry feed-forward: three cases where geometry overrides PI
            # (only when PI is reeling in, i.e. pi_speed < 0).
            #   1. Hub moving AWAY (dL_dt > 0): pay out at dL_dt to stop
            #      extension building — prevents tension spikes from wind gusts
            #      or wind-driven swinging.
            #   2. Hub APPROACHING faster than PI rate (dL_dt < pi_speed < 0):
            #      match hub speed (anti-slack guard for orbital transitions).
            #   3. Hub moving tangentially or approaching slowly
            #      (pi_speed < dL_dt <= 0) AND tension above target: track hub
            #      radial rate.  This prevents tension spikes at short tether
            #      (k_eff = EA/L is very large) including the case dL_dt = 0
            #      (hub purely tangential) where the else branch would reel in
            #      at pi_speed and rapidly build extension.  Not engaged below
            #      T_target so PI can freely reel in to recover from slack.
            # When pi_speed >= 0 (PI already paying out), do NOT override.
            pos_ned = state_pkt.get("pos_ned")
            if pos_ned is not None and self._anchor is not None:
                r = np.asarray(pos_ned, dtype=float) - self._anchor
                L = float(np.linalg.norm(r))
                if L > 1e-6:
                    dL_dt = float(np.dot(vel_ned, r / L))
                    if pi_speed < 0 and dL_dt > 0:
                        # Hub moving away while reeling in: pay out to prevent spike
                        winch_speed_ms = float(min(dL_dt, self._v_pay_out))
                    elif pi_speed < 0 and dL_dt < pi_speed:
                        # Hub approaching faster than PI reel-in: match hub
                        winch_speed_ms = float(np.clip(
                            dL_dt, -self._v_land_max, -self._v_land))
                    elif pi_speed < 0 and 0.0 >= dL_dt > pi_speed and tension_n > 3.0 * self._T_target:
                        # Hub approaching slowly or stationary radially (dL_dt <= 0)
                        # while tension is well above target (>3x).  Track hub
                        # radial rate to keep extension constant.  This prevents
                        # rapid spike growth at short tether (k_eff = EA/L large).
                        # Threshold 3x T_target ensures the condition can only fire
                        # when pi_speed is effectively zero-or-pay-out for any
                        # k_winch != 0 tuning (pi_speed crosses 0 at T = T_target +
                        # v_land/k_winch), so PI slack-recovery is never overridden.
                        winch_speed_ms = float(np.clip(dL_dt, -self._v_land_max, 0.0))
                    else:
                        winch_speed_ms = pi_speed
                else:
                    winch_speed_ms = pi_speed
            else:
                winch_speed_ms = pi_speed

            # Tension reel-in limiter: when case-2 geometry tracking drives
            # the winch faster than pi_speed AND tension is critically high
            # (> 5x target), lerp the winch speed from the tracked rate back
            # toward pi_speed.  This reduces the case-2 override gradually,
            # causing the hub to approach faster than the winch reels in, so
            # extension (and hence tension) decreases.  Unlike scaling to zero,
            # pi_speed is always a non-zero reel-in floor, so descent never
            # stalls.  The equilibrium naturally stabilises near 5x T_target
            # because the formula zero-crosses there.
            # Only fires when case-2 is active (winch_speed < pi_speed < 0);
            # case-3, normal PI, and pay-out are unaffected.
            if winch_speed_ms < pi_speed and tension_n > 5.0 * self._T_target:
                T_excess_frac = min(1.0, (tension_n - 5.0 * self._T_target) /
                                    (2.0 * self._T_target))
                winch_speed_ms = (winch_speed_ms
                                  + T_excess_frac * (pi_speed - winch_speed_ms))

        else:   # final_drop
            collective_rad = 0.0
            winch_speed_ms = 0.0

        return {
            "collective_rad": collective_rad,
            "winch_speed_ms": winch_speed_ms,
            "body_z_eq":      self._body_z_eq.copy(),
            "phase":          self._phase,
        }
