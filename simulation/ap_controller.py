"""
ap_controller.py -- AP-side pumping controller.

Runs on the ArduPilot side (400 Hz in Python sim, ~50 Hz in Lua on hardware).

Communication boundary:
    Ground -> AP: TensionCommand at ~10 Hz via MAVLink NAMED_VALUE_FLOAT.
    AP has no tension sensor of its own; the ground transmits both the setpoint
    and the current load-cell reading inside each TensionCommand.

Architecture:
    Ground (10 Hz): reads load cell, computes setpoint, packs TensionCommand.
    AP (10 Hz):     one TensionPI step per received TensionCommand.
                    Collective is held constant between commands.
    AP (400 Hz):    elevation rate-limited toward altitude target.

Daisy-chain tension control:
    Primary actuator:   collective (TensionPI, 10 Hz update)
    Secondary actuator: disk elevation (activates when collective saturates)

    Low tension (reel-in): collective pins at floor; residual tension error
    bleeds into an elevation correction that tilts the disk toward horizontal,
    reducing tension below what collective alone can achieve.  Exploits wind
    shear — lower elevation = lower altitude = weaker wind = less tension.

    High tension (reel-out): collective has full range; elevation correction
    is zero and decays to zero if previously active.

10 Hz gain tuning:
    For a static plant T = K*(C - C_ref), the PI integral converges with
    time-constant τ = (1 + K*kp) / (dt * K * ki).  With K≈2000 N/rad,
    dt=0.1 s, kp=2e-4, ki=1e-3: τ ≈ 7 steps (0.7 s).

Comms dropout:
    On timeout, setpoint falls to tension_ic (safe equilibrium mode).
"""

from __future__ import annotations

import numpy as np

from controller import TensionPI, compute_bz_altitude_hold, compute_rate_cmd, slerp_body_z
from pumping_planner import TensionCommand
from landing_planner import LandingCommand


class TensionApController:
    """
    AP-side pumping controller driven by TensionCommand (10 Hz from ground).

    Integrates TensionPI (collective) and elevation hold (body_z) into a
    single controller with daisy-chain allocation: collective is the primary
    tension actuator; when it saturates, residual tension error bleeds into
    an elevation correction that tilts the disk to further reduce/increase
    tension.

    Parameters
    ----------
    ic_pos          : np.ndarray (3,)  initial hub NED position [m]
    mass_kg         : float            rotor mass [kg]
    slew_rate_rad_s : float            elevation slew rate [rad/s]
    warm_coll_rad   : float            collective at startup (warm-starts TensionPI)
    tension_ic      : float            IC equilibrium tension [N] (comms-loss fallback)
    cmd_timeout_s   : float            comms dropout timeout [s]
    kp              : float            TensionPI proportional gain
    ki              : float            TensionPI integral gain (tuned for 10 Hz)
    el_corr_ki      : float            elevation correction integrator gain [rad/(N·s)]
    el_corr_max     : float            max elevation correction magnitude [rad]
    el_corr_tau     : float            correction decay time constant when not saturated [s]
    """

    CMD_TIMEOUT_S: float = 0.5

    # Commanded elevation is considered unreachable if the current hub elevation
    # differs from the commanded elevation by more than the AP can slew in this
    # many seconds.  Exceeding this flags "ap_unreachable_alt" in the event log.
    FEASIBILITY_WINDOW_S: float = 1.0

    def __init__(
        self,
        ic_pos         : np.ndarray,
        mass_kg        : float,
        slew_rate_rad_s: float,
        warm_coll_rad  : float,
        tension_ic     : float = 300.0,
        cmd_timeout_s  : float = CMD_TIMEOUT_S,
        kp             : float = 2e-4,
        ki             : float = 1e-3,
        el_corr_ki     : float = 5e-4,   # rad/(N·s)
        el_corr_max    : float = 0.35,   # rad (~20°)
        el_corr_tau    : float = 5.0,    # s
        kp_outer       : float = 2.5,    # body_z error → rate cmd [rad/s per rad]
        events         = None,           # optional BadEventLog
    ) -> None:
        self._mass_kg    = float(mass_kg)
        self._timeout    = float(cmd_timeout_s)
        self._tension_ic = float(tension_ic)
        self._slew       = float(slew_rate_rad_s)
        self._el_corr_ki  = float(el_corr_ki)
        self._el_corr_max = float(el_corr_max)
        self._el_corr_tau = float(el_corr_tau)
        self._kp_outer    = float(kp_outer)
        self._events      = events   # BadEventLog | None

        # Elevation state (inlined from AltitudeHoldController)
        tlen     = float(np.linalg.norm(ic_pos))
        self._el = float(np.arcsin(max(-1.0, min(1.0, float(-ic_pos[2]) / max(tlen, 0.1)))))
        self._el_correction  = 0.0
        self._coll_saturated = False

        self._tension_pi = TensionPI(
            setpoint_n    = tension_ic,
            warm_coll_rad = warm_coll_rad,
            kp            = kp,
            ki            = ki,
        )

        self._C_held      = float(warm_coll_rad)
        self._target_alt  = float(-ic_pos[2])
        self._cmd_age     = 0.0
        self._comms_ok    = True
        self._t_sim       = 0.0                           # accumulated sim time
        self._pos_ned     = np.asarray(ic_pos, dtype=float)  # updated each 400 Hz step

    # ── command reception (10 Hz from ground) ──────────────────────────────

    def receive_command(self, cmd: TensionCommand, dt_cmd: float) -> None:
        """
        Receive a TensionCommand from the ground and advance TensionPI one step.

        Uses the hub position cached from the most recent step() call to check
        whether the commanded altitude is within the AP's physical envelope.

        Checks performed (logged as bad events, command still accepted):
          ap_impossible_alt  — commanded alt_m > tether_length (arcsin undefined)
          ap_unreachable_alt — commanded elevation differs from current hub
                               elevation by more than the AP can slew in
                               FEASIBILITY_WINDOW_S seconds
        """
        new_alt = float(cmd.alt_m)

        if self._events is not None:
            pos_ned = self._pos_ned
            tlen    = float(np.linalg.norm(pos_ned))
            hub_alt = float(-pos_ned[2])
            phase   = cmd.phase

            # Geometric impossibility: alt_m > tether length → arcsin undefined.
            if new_alt > tlen - 1e-3:
                self._events.record(
                    "ap_impossible_alt", self._t_sim, phase, hub_alt,
                    target_alt=new_alt, tlen=tlen,
                )
            else:
                # Rate-feasibility: can the AP's slew rate bridge the elevation
                # gap from the AP's current internal _el within FEASIBILITY_WINDOW_S?
                # We compare against self._el (the AP's rate-limited internal state),
                # not against the hub's actual position.  Hub-to-_el lag is normal
                # physics transient; what we want to catch is a sudden jump in the
                # ground command relative to what the AP is already tracking.
                cmd_el   = float(np.arcsin(max(-1.0, min(1.0, new_alt / max(tlen, 0.1)))))
                el_gap   = abs(cmd_el - self._el)
                max_slew = self._slew * self.FEASIBILITY_WINDOW_S
                if el_gap > max_slew:
                    hub_el = float(np.arcsin(max(-1.0, min(1.0, hub_alt / max(tlen, 0.1)))))
                    self._events.record(
                        "ap_unreachable_alt", self._t_sim, phase, hub_alt,
                        target_alt=new_alt, ap_el_deg=float(np.degrees(self._el)),
                        cmd_el_deg=float(np.degrees(cmd_el)),
                        el_gap_deg=float(np.degrees(el_gap)),
                        max_slew_deg=float(np.degrees(max_slew)),
                        hub_el_deg=float(np.degrees(hub_el)),
                    )

        self._tension_pi.setpoint = float(cmd.tension_setpoint_n)
        self._target_alt          = new_alt
        self._cmd_age             = 0.0
        self._comms_ok            = True

        self._C_held = self._tension_pi.update(
            float(cmd.tension_measured_n), float(dt_cmd)
        )

        # Daisy-chain: accumulate elevation correction when collective is pinned
        coll_at_floor   = self._C_held <= self._tension_pi.coll_min + 1e-5
        coll_at_ceiling = self._C_held >= self._tension_pi.coll_max - 1e-5
        tension_error   = self._tension_pi.setpoint - float(cmd.tension_measured_n)

        self._coll_saturated = (
            (coll_at_floor   and tension_error < 0) or
            (coll_at_ceiling and tension_error > 0)
        )

        if self._coll_saturated:
            self._el_correction = float(np.clip(
                self._el_correction + self._el_corr_ki * tension_error * float(dt_cmd),
                -self._el_corr_max, self._el_corr_max,
            ))

    # ── 400 Hz step ────────────────────────────────────────────────────────

    def step(
        self,
        pos_ned: np.ndarray,
        R_hub  : np.ndarray,
        dt     : float,
    ) -> "tuple[float, float, float]":
        """
        400 Hz step.  Returns (collective_rad, rate_roll_sp, rate_pitch_sp).

        collective_rad  : held from last receive_command()
        rate_roll_sp    : desired roll  rate [rad/s] — feed into AcroControllerSitl
        rate_pitch_sp   : desired pitch rate [rad/s] — feed into AcroControllerSitl

        Elevation is rate-limited toward the altitude target plus any active
        daisy-chain correction.  The correction decays when collective is not
        saturated.  Rate commands are computed with kd=0 — damping is supplied
        by AcroControllerSitl's RatePID, mirroring the hardware architecture.
        """
        self._pos_ned  = np.asarray(pos_ned, dtype=float)
        self._cmd_age += dt
        self._t_sim   += dt
        if self._comms_ok and self._cmd_age > self._timeout:
            self._comms_ok = False
            self._tension_pi.setpoint = self._tension_ic

        # Decay correction when collective is not saturated
        if not self._coll_saturated:
            self._el_correction *= (1.0 - dt / self._el_corr_tau)

        # Rate-limit elevation toward altitude target + daisy-chain correction
        tlen      = float(np.linalg.norm(pos_ned))
        target_el = float(np.arcsin(max(-1.0, min(1.0, self._target_alt / max(tlen, 0.1)))))
        el_cmd    = float(np.clip(target_el + self._el_correction, -np.pi / 2, np.pi / 2))
        delta     = float(np.clip(el_cmd - self._el, -self._slew * dt, self._slew * dt))
        self._el += delta

        bz_goal = compute_bz_altitude_hold(
            pos_ned, self._el, self._tension_pi.setpoint, self._mass_kg
        )
        R       = np.asarray(R_hub, dtype=float)
        bz_now  = R[:, 2]
        rate_sp = compute_rate_cmd(bz_now, bz_goal, R, kp=self._kp_outer, kd=0.0)

        return self._C_held, float(rate_sp[0]), float(rate_sp[1])

    # ── diagnostics ────────────────────────────────────────────────────────

    @property
    def comms_ok(self) -> bool:
        return self._comms_ok

    @property
    def tension_setpoint(self) -> float:
        return self._tension_pi.setpoint

    @property
    def elevation_rad(self) -> float:
        """Current rate-limited elevation angle [rad]."""
        return self._el

    @property
    def el_correction_rad(self) -> float:
        """Active daisy-chain elevation correction [rad]."""
        return self._el_correction

    @property
    def coll_saturated(self) -> bool:
        """True when collective is pinned at floor or ceiling."""
        return self._coll_saturated


# ---------------------------------------------------------------------------
# LandingApController — AP-side landing controller (400 Hz)
# ---------------------------------------------------------------------------

class LandingApController:
    """
    AP-side landing controller driven by LandingCommand (10 Hz from ground).

    Control laws per phase:
      reel_in     -- body_z slerps toward body_z_target;
                     VZ PI with vz_sp=0 holds altitude while disk tilts.
      descent     -- body_z held fixed; VZ PI descends at vz_sp=v_land.
      final_drop  -- collective=0; body_z held fixed.

    Parameters
    ----------
    ic_body_z       : IC body_z NED unit vector (starting orientation)
    slew_rate_rad_s : body_z slew rate [rad/s]
    warm_coll_rad   : starting collective [rad] (VZ PI integrator seed)
    kp_vz           : VZ PI proportional gain [rad/(m/s)]
    ki_vz           : VZ PI integral gain [rad/(m/s)/s]
    col_min_rad     : collective floor [rad]
    col_max_rad     : collective ceiling [rad]
    kp_outer        : body_z error → rate command gain [rad/s per rad]
    """

    def __init__(
        self,
        ic_body_z:       np.ndarray,
        slew_rate_rad_s: float,
        warm_coll_rad:   float,
        kp_vz:           float = 0.05,
        ki_vz:           float = 0.005,
        col_min_rad:     float = -0.28,
        col_max_rad:     float =  0.10,
        kp_outer:        float = 2.5,
    ) -> None:
        bz = np.asarray(ic_body_z, dtype=float)
        self._bz_current = bz / np.linalg.norm(bz)
        self._bz_target  = self._bz_current.copy()
        self._slew       = float(slew_rate_rad_s)
        self._C_held     = float(warm_coll_rad)
        self._col_i      = float(warm_coll_rad)   # VZ PI integrator
        self._kp_vz      = float(kp_vz)
        self._ki_vz      = float(ki_vz)
        self._col_min    = float(col_min_rad)
        self._col_max    = float(col_max_rad)
        self._kp_outer   = float(kp_outer)
        self._vz_sp      = 0.0
        self._phase      = "reel_in"

    # ── command reception (10 Hz from ground) ─────────────────────────────

    def receive_command(self, cmd: LandingCommand, dt_cmd: float) -> None:  # noqa: ARG002
        """Receive a LandingCommand from LandingGroundController at 10 Hz."""
        prev_phase = self._phase
        self._phase      = cmd.phase
        self._bz_target  = np.asarray(cmd.body_z_target, dtype=float).copy()
        self._vz_sp      = float(cmd.vz_setpoint_ms)

        # Seed the VZ PI integrator at the feedforward collective on phase entry.
        if cmd.phase != prev_phase:
            self._col_i = float(np.clip(cmd.col_cruise_rad, self._col_min, self._col_max))

        if cmd.phase == "final_drop":
            self._C_held = 0.0

    # ── 400 Hz step ────────────────────────────────────────────────────────

    def step(
        self,
        pos_ned: np.ndarray,  # noqa: ARG002
        R_hub:   np.ndarray,
        vel_ned: np.ndarray,
        dt:      float,
    ) -> "tuple[float, float, float]":
        """
        400 Hz step.  Returns (collective_rad, rate_roll_sp, rate_pitch_sp).

        pos_ned  : hub NED position (accepted for API symmetry; not used directly)
        R_hub    : body-to-NED rotation matrix
        vel_ned  : hub NED velocity [m/s]
        dt       : timestep [s]
        """
        R      = np.asarray(R_hub, dtype=float)
        bz_now = R[:, 2]

        # Slerp body_z toward target at slew_rate
        self._bz_current = slerp_body_z(self._bz_current, self._bz_target, self._slew, dt)

        # Body_z rate command (kd=0; damping supplied by AcroControllerSitl RatePID)
        rate_sp = compute_rate_cmd(bz_now, self._bz_current, R, kp=self._kp_outer, kd=0.0)

        # Collective
        if self._phase == "final_drop":
            self._C_held = 0.0
        else:
            vz_err       = float(vel_ned[2]) - self._vz_sp
            self._col_i  = float(np.clip(
                self._col_i + self._ki_vz * vz_err * dt,
                self._col_min, self._col_max,
            ))
            self._C_held = float(np.clip(
                self._col_i + self._kp_vz * vz_err,
                self._col_min, self._col_max,
            ))

        return self._C_held, float(rate_sp[0]), float(rate_sp[1])

    # ── diagnostics ────────────────────────────────────────────────────────

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def bz_current(self) -> np.ndarray:
        """Current rate-limited body_z direction (NED unit vector)."""
        return self._bz_current.copy()

    @property
    def elevation_rad(self) -> float:
        """Elevation angle of current body_z [rad] — for telemetry."""
        return float(np.arcsin(np.clip(-self._bz_current[2], -1.0, 1.0)))
