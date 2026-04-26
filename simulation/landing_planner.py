"""
landing_planner.py -- Ground-station landing controller and AP-side command protocol.

Unified architecture (mirrors pumping_planner.py):
  LandingGroundController (10 Hz)  -> LandingCommand -> LandingApController (400 Hz)
  WinchController (400 Hz) set_target at 10 Hz from LandingGroundController.

Phase sequence (LandingGroundController):
  reel_in     -- body_z slerps from IC orientation to xi_reel_in_deg.
                 Winch holds at IC rest_length.  AP VZ PI holds altitude (vz_sp=0).
                 Exits after body_z slew time + settle margin.

  descent     -- body_z fixed at xi_reel_in; AP VZ PI descends at vz_sp=v_land;
                 winch tension-PI reels in to min_tether_m.

  final_drop  -- collective=0; winch holds; hub drops onto catch pad.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


# ---------------------------------------------------------------------------
# LandingCommand — ground → AP at 10 Hz (new unified protocol)
# ---------------------------------------------------------------------------

@dataclass
class LandingCommand:
    """
    Command sent from LandingGroundController to LandingApController at 10 Hz.

    phase           : "reel_in" | "descent" | "final_drop"
    body_z_target   : target disk orientation (NED unit vector); AP slerps toward it
    vz_setpoint_ms  : NED descent rate setpoint [m/s], positive = downward
                       0 during reel_in (hold altitude), v_land during descent
    col_cruise_rad  : VZ PI integrator seed [rad]
                       IC collective during reel_in; col_min_reel_in during descent
    """
    phase:          str
    body_z_target:  np.ndarray
    vz_setpoint_ms: float
    col_cruise_rad: float


# ---------------------------------------------------------------------------
# LandingGroundController — 10 Hz ground side (new unified architecture)
# ---------------------------------------------------------------------------

class LandingGroundController:
    """
    Ground-side landing controller (10 Hz).

    Manages the reel_in → descent → final_drop phase sequence and emits
    LandingCommand for LandingApController.  Also exposes winch_target_length
    and winch_target_tension for WinchController.set_target().

    Parameters
    ----------
    initial_body_z   : IC body_z NED unit vector (xi ~ 30 deg at steady flight)
    xi_reel_in_deg   : target disk tilt; fixed body_z orientation during descent [deg]
    slew_rate_rad_s  : body_z slew rate [rad/s] — used to compute reel_in duration
    col_reel_in_rad  : VZ PI seed collective during reel_in [rad]
    col_cruise_rad   : VZ PI seed collective during descent [rad]
    tension_descent  : winch tension target during descent [N]
                       set so that kp*(tension_descent - natural_tension) ≈ v_land
    v_land           : VZ descent rate setpoint [m/s]
    min_tether_m     : rest_length threshold for descent → final_drop [m]
    t_reel_in_max    : safety timeout for reel_in phase [s]
    t_descent_max    : safety timeout for descent phase [s]
    """

    def __init__(
        self,
        initial_body_z:  np.ndarray,
        xi_reel_in_deg:  float = 80.0,
        slew_rate_rad_s: float = 0.40,
        col_reel_in_rad: float = 0.079,
        col_cruise_rad:  float = 0.079,
        tension_descent: float = 180.0,
        v_land:          float = 0.5,
        min_tether_m:    float = 2.0,
        t_reel_in_max:   float = 120.0,
        t_descent_max:   float = 600.0,
    ) -> None:
        bz_ic = np.asarray(initial_body_z, dtype=float)
        bz_ic = bz_ic / np.linalg.norm(bz_ic)

        self._col_reel_in   = float(col_reel_in_rad)
        self._col_cruise    = float(col_cruise_rad)
        self._tension_desc  = float(tension_descent)
        self._v_land        = float(v_land)
        self._min_tether    = float(min_tether_m)
        self._t_reel_max    = float(t_reel_in_max)
        self._t_desc_max    = float(t_descent_max)

        # Body_z target at xi_reel_in, same azimuth as IC
        xi_rad    = math.radians(xi_reel_in_deg)
        horiz     = np.array([bz_ic[0], bz_ic[1], 0.0])
        horiz_len = float(np.linalg.norm(horiz))
        az_unit   = horiz / horiz_len if horiz_len > 1e-6 else np.array([0.0, 1.0, 0.0])
        bz_tgt    = az_unit * math.cos(xi_rad) + np.array([0.0, 0.0, -math.sin(xi_rad)])
        self._bz_target = bz_tgt / np.linalg.norm(bz_tgt)

        # Reel-in duration = time to slew from IC xi to xi_reel_in + 2 s settle
        xi_ic = float(np.degrees(np.arcsin(max(-1.0, min(1.0, float(-bz_ic[2]))))))
        self._t_reel_in = abs(math.radians(xi_reel_in_deg - xi_ic)) / slew_rate_rad_s + 2.0

        self._phase         = "reel_in"
        self._phase_start   = 0.0
        self._start_length  = 0.0
        self._initialized   = False

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
            self._initialized  = True
            self._start_length = rest_length
            self._phase_start  = t_sim

        prev_phase = self._phase
        self._advance_phase(t_sim, rest_length)
        if self._phase != prev_phase:
            self._phase_start = t_sim

        if self._phase == "reel_in":
            self._winch_tgt_len = self._start_length   # hold
            self._winch_tgt_ten = self._tension_desc   # irrelevant (remaining≈0)
            col_cruise          = self._col_reel_in
            vz_sp               = 0.0
        elif self._phase == "descent":
            self._winch_tgt_len = self._min_tether
            self._winch_tgt_ten = self._tension_desc
            col_cruise          = self._col_cruise
            vz_sp               = self._v_land
        else:  # final_drop
            self._winch_tgt_len = rest_length          # hold wherever we are
            self._winch_tgt_ten = self._tension_desc
            col_cruise          = self._col_cruise
            vz_sp               = 0.0

        return LandingCommand(
            phase          = self._phase,
            body_z_target  = self._bz_target.copy(),
            vz_setpoint_ms = vz_sp,
            col_cruise_rad = col_cruise,
        )

    # ── private helpers ────────────────────────────────────────────────────

    def _advance_phase(self, t_sim: float, rest_length: float) -> None:
        t_in = t_sim - self._phase_start
        if self._phase == "reel_in":
            if t_in >= self._t_reel_in or t_in >= self._t_reel_max:
                self._phase = "descent"
        elif self._phase == "descent":
            if rest_length <= self._min_tether + 0.05 or t_in >= self._t_desc_max:
                self._phase = "final_drop"
        # final_drop: terminal phase
