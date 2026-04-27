"""
landing_planner.py -- Ground-station landing controller and AP-side command protocol.

Unified architecture (mirrors pumping_planner.py):
  LandingGroundController (10 Hz)  -> LandingCommand -> LandingApController (400 Hz)
  WinchController (400 Hz) set_target at 10 Hz from LandingGroundController.

Phase sequence (LandingGroundController):
  reel_in      -- winch reels in from IC length to target_length_m.  body_z slerps
                  to [0,0,-1] simultaneously, shedding tension so the winch can pull.
                  Hub may rise in altitude — VZ PI vz_sp=0 is a soft hold.
                  Exits when rest_length <= target_length_m.

  get_vertical -- winch holds at target_length_m.  body_z slerps to [0,0,-1]
                  (horizontal disk, xi=90 deg).  AP holds altitude (vz_sp=0).
                  Requires Peters-He aero (SkewedWakeBEM invalid above xi~85 deg).
                  Exits after slew time + settle.

  descent      -- horizontal disk fixed.  Winch tension-PI reels in to min_tether_m,
                  pulling hub down.  AP VZ PI descends at vz_sp=v_land.
                  Exits when rest_length <= min_tether_m.

  flare        -- winch holds.  AP VZ PI holds vz_sp=0, seeded at col_descent +
                  col_flare_delta to arrest descent.  Terminal phase.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


# ---------------------------------------------------------------------------
# LandingCommand — ground → AP at 10 Hz
# ---------------------------------------------------------------------------

@dataclass
class LandingCommand:
    """
    Command sent from LandingGroundController to LandingApController at 10 Hz.

    phase           : "reel_in" | "get_vertical" | "descent" | "flare"
    body_z_target   : target disk orientation (NED unit vector); AP slerps toward it
    vz_setpoint_ms  : NED descent rate setpoint [m/s], positive = downward
    col_cruise_rad  : VZ PI integrator seed [rad] — applied on phase entry
    """
    phase:          str
    body_z_target:  np.ndarray
    vz_setpoint_ms: float
    col_cruise_rad: float


# ---------------------------------------------------------------------------
# LandingGroundController — 10 Hz ground side
# ---------------------------------------------------------------------------

class LandingGroundController:
    """
    Ground-side landing controller (10 Hz).

    Phase sequence: reel_in → get_vertical → descent → flare.

    Parameters
    ----------
    initial_body_z   : IC body_z NED unit vector (~30 deg elevation at steady flight)
    target_length_m  : tether rest length at end of reel_in [m]
    slew_rate_rad_s  : body_z slew rate [rad/s] — used to compute get_vertical duration
    col_reel_in_rad  : VZ PI seed during reel_in [rad]
    col_vertical_rad : VZ PI seed during get_vertical [rad]
    col_descent_rad  : VZ PI seed during descent [rad]
    col_flare_delta  : collective bump above col_descent_rad at flare entry [rad]
    tension_reel_in  : winch tension target during reel_in [N]
    tension_descent  : winch tension target during descent [N]
                       kp*(tension_descent - natural_tension) ≈ v_land
    v_land           : NED descent rate setpoint during descent [m/s]
    min_tether_m     : rest_length threshold for descent → flare [m]
    t_reel_in_max    : safety timeout for reel_in phase [s]
    t_vertical_max   : safety timeout for get_vertical phase [s]
    t_descent_max    : safety timeout for descent phase [s]
    """

    _BZ_VERTICAL = np.array([0.0, 0.0, -1.0])   # disk horizontal, pointing up in NED

    def __init__(
        self,
        initial_body_z:   np.ndarray,
        target_length_m:  float = 50.0,
        slew_rate_rad_s:  float = 0.40,
        col_reel_in_rad:  float = -0.23,
        col_vertical_rad: float = -0.23,
        col_descent_rad:  float = -0.23,
        col_flare_delta:  float = 0.05,
        tension_reel_in:  float = 300.0,
        tension_descent:  float = 100.0,
        v_land:           float = 1.0,
        min_tether_m:     float = 5.0,
        t_reel_in_max:    float = 120.0,
        t_vertical_max:   float = 30.0,
        t_descent_max:    float = 300.0,
    ) -> None:
        bz_ic = np.asarray(initial_body_z, dtype=float)
        bz_ic = bz_ic / np.linalg.norm(bz_ic)

        self._bz_ic          = bz_ic.copy()
        self._target_len     = float(target_length_m)
        self._col_reel_in    = float(col_reel_in_rad)
        self._col_vertical   = float(col_vertical_rad)
        self._col_descent    = float(col_descent_rad)
        self._col_flare      = float(col_descent_rad) + float(col_flare_delta)
        self._tension_ri     = float(tension_reel_in)
        self._tension_desc   = float(tension_descent)
        self._v_land         = float(v_land)
        self._min_tether     = float(min_tether_m)
        self._t_reel_max     = float(t_reel_in_max)
        self._t_vertical_max = float(t_vertical_max)
        self._t_desc_max     = float(t_descent_max)

        # get_vertical duration: slew from IC xi to 90 deg + 2 s settle
        xi_ic_rad         = float(np.arcsin(np.clip(-bz_ic[2], -1.0, 1.0)))
        angle_to_vertical = abs(math.radians(90.0) - xi_ic_rad)
        self._t_vertical  = angle_to_vertical / max(slew_rate_rad_s, 1e-6) + 2.0

        self._phase        = "reel_in"
        self._phase_start  = 0.0
        self._initialized  = False

        self._winch_tgt_len = 0.0
        self._winch_tgt_ten = 0.0

    # ── public properties ──────────────────────────────────────────────────

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def winch_target_length(self) -> float:
        return self._winch_tgt_len

    @property
    def winch_target_tension(self) -> float:
        return self._winch_tgt_ten

    # ── main step (10 Hz) ─────────────────────────────────────────────────

    def step(
        self,
        t_sim:              float,
        tension_measured_n: float,
        rest_length:        float = 0.0,
        hub_alt_m:          float = 0.0,
    ) -> LandingCommand:
        """10 Hz step. Returns LandingCommand for LandingApController."""
        if not self._initialized:
            self._initialized = True
            self._phase_start = t_sim

        prev_phase = self._phase
        self._advance_phase(t_sim, rest_length)
        if self._phase != prev_phase:
            self._phase_start = t_sim

        if self._phase == "reel_in":
            self._winch_tgt_len = self._target_len
            self._winch_tgt_ten = self._tension_ri
            bz_tgt              = self._BZ_VERTICAL   # tilt toward horizontal to shed tension
            col_cruise          = self._col_reel_in
            vz_sp               = 0.0
        elif self._phase == "get_vertical":
            self._winch_tgt_len = self._target_len
            self._winch_tgt_ten = self._tension_ri      # irrelevant (no motion)
            bz_tgt              = self._BZ_VERTICAL
            col_cruise          = self._col_vertical
            vz_sp               = 0.0
        elif self._phase == "descent":
            self._winch_tgt_len = self._min_tether
            self._winch_tgt_ten = self._tension_desc
            bz_tgt              = self._BZ_VERTICAL
            col_cruise          = self._col_descent
            vz_sp               = self._v_land
        else:  # flare
            self._winch_tgt_len = rest_length           # hold wherever we are
            self._winch_tgt_ten = self._tension_desc
            bz_tgt              = self._BZ_VERTICAL
            col_cruise          = self._col_flare
            vz_sp               = 0.0

        return LandingCommand(
            phase          = self._phase,
            body_z_target  = bz_tgt.copy(),
            vz_setpoint_ms = vz_sp,
            col_cruise_rad = col_cruise,
        )

    # ── private helpers ────────────────────────────────────────────────────

    def _advance_phase(self, t_sim: float, rest_length: float) -> None:
        t_in = t_sim - self._phase_start
        if self._phase == "reel_in":
            if rest_length <= self._target_len + 0.5 or t_in >= self._t_reel_max:
                self._phase = "get_vertical"
        elif self._phase == "get_vertical":
            if t_in >= self._t_vertical or t_in >= self._t_vertical_max:
                self._phase = "descent"
        elif self._phase == "descent":
            if rest_length <= self._min_tether + 0.1 or t_in >= self._t_desc_max:
                self._phase = "flare"
        # flare: terminal phase
