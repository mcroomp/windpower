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

10 Hz gain tuning:
    For a static plant T = K*(C - C_ref), the PI integral converges with
    time-constant τ = (1 + K*kp) / (dt * K * ki).  With K≈2000 N/rad,
    dt=0.1 s, kp=2e-4, ki=1e-3: τ ≈ 7 steps (0.7 s).

Comms dropout:
    On timeout, setpoint falls to tension_ic (safe equilibrium mode).
"""

from __future__ import annotations

import numpy as np

from controller import (AccelVibrationDamper, TensionPI,
                        compute_bz_altitude_hold, compute_rate_cmd, slerp_body_z)
from physics_core   import HubObservation
from pumping_planner import TensionCommand
from landing_planner import LandingCommand


class TensionApController:
    """
    AP-side pumping controller driven by TensionCommand (10 Hz from ground).

    TensionPI adjusts collective to track tether tension; elevation is
    rate-limited toward the altitude target from the ground command.

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
    kd              : float            TensionPI derivative gain (0 = pure PI)
    coll_min_rad    : float            collective floor [rad]
    coll_max_rad    : float            collective ceiling [rad]
    kp_outer        : float            body_z error → rate command gain [rad/s per rad]
    """

    CMD_TIMEOUT_S: float = 0.5

    # Commanded elevation is considered unreachable if the current hub elevation
    # differs from the commanded elevation by more than the AP can slew in this
    # many seconds.  Exceeding this flags "ap_unreachable_alt" in the event log.
    FEASIBILITY_WINDOW_S: float = 1.0

    # TensionPI gains — mirrors rawes.lua KP_TEN / KI_TEN / KD_TEN
    KP_TEN: float = 2e-4    # rad/N        proportional gain (10 Hz-tuned)
    KI_TEN: float = 1e-3    # rad/(N·s)    integral gain (10 Hz-tuned)
    KD_TEN: float = 0.0     # rad·s/N      derivative (0 = pure PI)

    # Collective limits — mirrors rawes.lua COL_MIN_RAD / COL_MAX_TEN
    COL_MIN_RAD: float = -0.28   # rad  collective floor (autorotation lower bound)
    COL_MAX_TEN: float =  0.10   # rad  TensionPI ceiling

    # Vibration damper — mirrors rawes.lua K_VIB / VIB_HP_TAU / VIB_VEL_TAU / VIB_COL_MAX
    K_VIB:       float = 0.008   # rad/(m/s)  velocity feedback gain
    VIB_HP_HZ:   float = 1.5     # Hz         high-pass cutoff (Lua stores as VIB_HP_TAU = 1/(2π×Hz))
    VIB_VEL_TAU: float = 0.5     # s          leaky integrator time constant
    VIB_COL_MAX: float = 0.04    # rad        max correction magnitude

    def __init__(
        self,
        ic_pos         : np.ndarray,
        mass_kg        : float,
        slew_rate_rad_s: float,
        warm_coll_rad  : float,
        tension_ic     : float,
        cmd_timeout_s  : float = CMD_TIMEOUT_S,
        kp             : float = KP_TEN,
        ki             : float = KI_TEN,
        kd             : float = KD_TEN,
        coll_min_rad   : float = COL_MIN_RAD,
        coll_max_rad   : float = COL_MAX_TEN,
        kp_outer       : float = 2.5,
        events         = None,           # optional BadEventLog
        k_vib          : float = K_VIB,
        vib_hp_hz      : float = VIB_HP_HZ,
        vib_vel_tau_s  : float = VIB_VEL_TAU,
        vib_col_max    : float = VIB_COL_MAX,
    ) -> None:
        self._mass_kg    = float(mass_kg)
        self._timeout    = float(cmd_timeout_s)
        self._tension_ic = float(tension_ic)
        self._slew       = float(slew_rate_rad_s)
        self._kp_outer   = float(kp_outer)
        self._events     = events   # BadEventLog | None

        # Elevation state
        tlen     = float(np.linalg.norm(ic_pos))
        self._el = float(np.arcsin(max(-1.0, min(1.0, float(-ic_pos[2]) / max(tlen, 0.1)))))

        self._tension_pi = TensionPI(
            setpoint_n    = tension_ic,
            kp            = kp,
            ki            = ki,
            coll_min      = coll_min_rad,
            coll_max      = coll_max_rad,
            warm_coll_rad = warm_coll_rad,
            kd            = kd,
        )

        self._C_held      = float(warm_coll_rad)
        self._target_alt  = float(-ic_pos[2])
        self._cmd_age     = 0.0
        self._comms_ok    = True
        self._t_sim       = 0.0                           # accumulated sim time
        self._pos_ned     = np.asarray(ic_pos, dtype=float)  # updated each 400 Hz step

        self._vib_damper  = (
            AccelVibrationDamper(
                k_vib        = float(k_vib),
                hp_freq_hz   = float(vib_hp_hz),
                vel_tau_s    = float(vib_vel_tau_s),
                col_damp_max = float(vib_col_max),
            ) if k_vib != 0.0 else None
        )
        self._last_vib_corr = 0.0

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

    # ── 400 Hz step ────────────────────────────────────────────────────────

    def step(
        self,
        obs      : HubObservation,
        dt       : float,
        *,
        accel_ned: "np.ndarray | None" = None,
    ) -> "tuple[float, float, float]":
        """
        AP step.  Returns (collective_rad, rate_roll_sp, rate_pitch_sp).

        obs       : current hub observation from runner.observe() or physics_core.hub_observe()
        accel_ned : NED specific force [m/s²] from previous physics step (IMU measurement,
                    gravity excluded).  When provided and k_vib != 0, the AccelVibrationDamper
                    adds a collective correction opposing the estimated 3–8 Hz tether resonance
                    velocity.  Uses the previous step's accel (one-step lag) because accel is
                    computed by physics after collective is applied; this matches the hardware
                    latency between IMU measurement and actuator output.

        Rate commands use kd=0 — damping is supplied by AcroControllerSitl's
        RatePID, mirroring the hardware architecture.
        """
        self._pos_ned  = obs.pos
        self._cmd_age += dt
        self._t_sim   += dt
        if self._comms_ok and self._cmd_age > self._timeout:
            self._comms_ok = False
            self._tension_pi.setpoint = self._tension_ic

        # Rate-limit elevation toward altitude target
        tlen      = float(np.linalg.norm(obs.pos))
        target_el = float(np.arcsin(max(-1.0, min(1.0, self._target_alt / max(tlen, 0.1)))))
        delta     = float(np.clip(target_el - self._el, -self._slew * dt, self._slew * dt))
        self._el += delta

        bz_goal = compute_bz_altitude_hold(
            obs.pos, self._el, self._tension_pi.setpoint, self._mass_kg
        )
        R       = obs.R
        bz_now  = R[:, 2]
        rate_sp = compute_rate_cmd(bz_now, bz_goal, R, kp=self._kp_outer, kd=0.0)

        # Vibration damper: HP-filter body-Z accel → estimate resonance velocity → oppose it
        col_out = self._C_held
        self._last_vib_corr = 0.0
        if accel_ned is not None and self._vib_damper is not None:
            accel_body_z = float((R.T @ np.asarray(accel_ned, dtype=float))[2])
            self._last_vib_corr = self._vib_damper.step(accel_body_z, dt)
            col_out  = float(np.clip(
                col_out + self._last_vib_corr,
                self._tension_pi.coll_min,
                self._tension_pi.coll_max,
            ))

        return col_out, float(rate_sp[0]), float(rate_sp[1])

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

    def log_fields(self) -> dict:
        """Standard AP state fields for telemetry kwargs."""
        return dict(
            tension_setpoint             = self.tension_setpoint,
            elevation_rad                = self.elevation_rad,
            comms_ok                     = self.comms_ok,
            collective_from_tension_ctrl = self._C_held,
            vib_corr                     = self._last_vib_corr,
            ten_pi_integral              = self._tension_pi._integral,
        )


# ---------------------------------------------------------------------------
# LandingApController — AP-side landing controller (400 Hz)
# ---------------------------------------------------------------------------

class LandingApController:
    """
    AP-side landing controller driven by LandingCommand (10 Hz from ground).

    Class constants mirror rawes.lua module-level constants for landing mode.

    Control laws per phase:
      reel_in      -- body_z slerps toward body_z_target (held at IC);
                      VZ PI with vz_sp=0 holds altitude while winch reels in.
      get_vertical -- body_z slerps toward [0,0,-1] (horizontal disk, xi=90°);
                      VZ PI with vz_sp=0 holds altitude. Requires Peters-He aero.
      descent      -- body_z fixed at [0,0,-1]; VZ PI descends at vz_sp=v_land.
      flare        -- body_z fixed at [0,0,-1]; VZ PI at vz_sp=0, integrator
                      seeded at col_descent + col_flare_delta on phase entry.

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

    # Collective limits — mirrors rawes.lua COL_MIN_RAD / COL_MAX_RAD
    COL_MIN_RAD: float = -0.28   # rad
    COL_MAX_RAD: float =  0.10   # rad

    # VZ PI gains — mirrors rawes.lua KP_VZ / KI_VZ
    KP_VZ: float = 0.05    # rad/(m/s)
    KI_VZ: float = 0.005   # rad/(m/s)/s

    def __init__(
        self,
        ic_body_z:       np.ndarray,
        slew_rate_rad_s: float,
        warm_coll_rad:   float,
        kp_vz:           float,
        ki_vz:           float,
        col_min_rad:     float,
        col_max_rad:     float,
        kp_outer:        float,
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

    # ── 400 Hz step ────────────────────────────────────────────────────────

    def step(
        self,
        obs: HubObservation,
        dt:  float,
        *,
        accel_ned: "np.ndarray | None" = None,  # noqa: ARG002 — unused; accepted for API symmetry
    ) -> "tuple[float, float, float]":
        """
        AP step.  Returns (collective_rad, rate_roll_sp, rate_pitch_sp).

        obs : current hub observation from runner.observe() or physics_core.hub_observe()
        dt  : timestep [s]
        """
        R      = obs.R
        bz_now = R[:, 2]

        # Slerp body_z toward target at slew_rate
        self._bz_current = slerp_body_z(self._bz_current, self._bz_target, self._slew, dt)

        # Body_z rate command (kd=0; damping supplied by AcroControllerSitl RatePID)
        rate_sp = compute_rate_cmd(bz_now, self._bz_current, R, kp=self._kp_outer, kd=0.0)

        # VZ PI: all phases use the PI; phase entry seeds the integrator via receive_command
        vz_err       = float(obs.vel[2]) - self._vz_sp
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

    def log_fields(self) -> dict:
        """Standard AP state fields for telemetry kwargs."""
        return dict(
            elevation_rad = self.elevation_rad,
            body_z_eq     = self.bz_current,
        )
