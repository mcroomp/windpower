"""
physics_core.py — Shared 400 Hz physics integration core.

Used by both the SITL mediator and simtests to ensure identical physics
constants and integration logic across all simulation paths.

PhysicsCore owns
----------------
- RigidBodyDynamics (6-DOF RK4)
- Aero model (SkewedWakeBEMJit)
- TetherModel (elastic, tension-only)
- Spin ODE (autorotation equilibrium)
- Yaw damping: k_yaw (GB4008 yaw axis), with optional hub-motor ODE
  (torque_model.HubState) driven by caller-supplied yaw_throttle
- KinematicStartup: state override + tether gating + extra startup damping
- Time tracking (_t_sim)

Callers own
-----------
- Trajectory planners (HoldPlanner, etc.)
- Altitude-PID collective management (commanded tension is feedforward only)
- WinchController / tether rest-length updates
- SITL interface and sensor building (mediator only)
- Telemetry logging

Step method
-----------
step(dt, collective_rad, tilt_lon, tilt_lat)   raw tilts  — mediator / Lua-test path

Returns a result dict.
"""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from types import SimpleNamespace

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dynamics   import RigidBodyDynamics
from dynbem       import create_aero, RotorInputs, euler_step_omega
from tether     import TetherModel
from rotor_physics import resolve_i_spin_kgm2
from param_defaults import thrust_to_coll_rad as _t2c
from torque_model import (
    HubState as _HubState, HubParams as _HubParams, step as _hub_step,
    equilibrium_throttle as _hub_equilibrium_throttle,
)


@dataclass
class HubObservation:
    """
    Observable state of the hub — only what real hardware sensors expose.

    R : ndarray (3, 3)
        Rotation matrix body→world (NED).  Source: AHRS (EKF3 attitude estimate).

    pos : ndarray (3,)
        Hub position in NED [m] relative to anchor/home.
        Source: GPS/EKF3 LOCAL_POSITION_NED.

    vel : ndarray (3,)
        Hub velocity in NED [m/s].
        Source: GPS/EKF3 LOCAL_POSITION_NED.

    body_z : ndarray (3,)
        Rotor axis (disk normal) as a unit vector in NED.  Equal to R[:, 2].
        Derived from R — not an independent sensor; convenience accessor.

    gyro : ndarray (3,)
        Body-frame angular rate [rad/s]: R.T @ omega_world.
        What the Pixhawk IMU gyroscope measures — orbital angular velocity of
        the hub electronics projected into body frame.  Does NOT include rotor
        spin (omega_spin is a separate scalar).
        Same formula as sensor.py PhysicalSensor.

    omega_spin : float
        Rotor blade spin rate [rad/s] (autorotation).  In hardware derived
        from GB4008 ESC telemetry RPM divided by the 10:1 gear ratio.
        Not available via AHRS — a distinct sensor channel.
    """
    R:          np.ndarray
    pos:        np.ndarray
    vel:        np.ndarray
    body_z:     np.ndarray
    gyro:       np.ndarray
    omega_spin: float


class PhysicsCore:
    """
    400 Hz physics integration shared between mediator and simtests.

    Parameters
    ----------
    rotor               : RotorDefinition — single source of all physical constants
    ic                  : object with .pos, .vel, .R0, .rest_length,
                          .eq_thrust, .omega_spin
    wind                : NED wind vector [m/s]
    base_k_ang          : optional diagnostic angular damping [N·m·s/rad]
    k_yaw               : GB4008 yaw damper around disk_normal [N·m·s/rad]
    kinematic           : KinematicStartup | None
    startup_damp_k_ang  : extra angular damping applied during kinematic ramp
    z_floor             : NED Z floor for dynamics (default -1.0 m = 1 m altitude floor)
    """

    BASE_K_ANG    = 0.0     # N·m·s/rad — no permanent artificial damping
    K_YAW_DEFAULT = 100.0   # N·m·s/rad — matches mediator _K_YAW_DEFAULT
    T_AERO_OFFSET = 45.0    # s — aero ramp already complete at simulation start
    KINEMATIC_SPINUP_S = 5.0  # s — final kinematic window to blend omega to IC target

    def __init__(
        self,
        rotor,
        ic,
        wind,
        *,
        base_k_ang:         float = BASE_K_ANG,
        k_yaw:              float = K_YAW_DEFAULT,
        kinematic                 = None,
        startup_damp_k_ang: float = 0.0,
        z_floor:            float = -1.0,
        aero_model:         str   = "quasi_static",
        kinematic_aero_mode: str  = "locked",
        kinematic_nul_rate_gain_rads_per_rad: float = 0.0,
        aero_override             = None,
        tether_override           = None,
    ):
        self._rotor              = rotor
        self._wind               = np.asarray(wind, dtype=float).copy()
        self._wind.flags.writeable = False
        self._base_k_ang         = float(base_k_ang)
        self._k_yaw              = float(k_yaw)
        self._kinematic          = kinematic
        self._startup_damp_k_ang = float(startup_damp_k_ang)
        self._kinematic_aero_mode = str(kinematic_aero_mode)
        self._kinematic_nul_rate_gain_rads_per_rad = float(
            kinematic_nul_rate_gain_rads_per_rad
        )
        # GB4008 yaw-motor hub model (torque_model.py) -- same ESC-governor +
        # finite-torque ODE used by the standalone counter-torque tests.
        # Only advanced when the caller supplies yaw_throttle (see step()).
        # Start the motor at the trim speed that holds psi_dot=0 for the IC's
        # rotor spin rate (matches physical reality: the ESC is already
        # holding heading before free flight starts, not spinning up from a
        # dead stop -- avoids a spurious startup yaw-drift transient).
        self._hub_params = _HubParams()
        _eq_throttle = _hub_equilibrium_throttle(float(ic.omega_spin), self._hub_params)
        self._hub_state  = _HubState(omega_motor=_eq_throttle * self._hub_params.rpm_scale)

        I_spin = resolve_i_spin_kgm2(rotor)
        self._dyn = RigidBodyDynamics(
            mass   = float(rotor.inertia.mass_kg),
            I_body = list(rotor.inertia.I_body_kgm2),
            I_spin = float(I_spin),
            pos0   = list(ic.pos),
            vel0   = list(ic.vel),
            R0     = ic.R0,
            omega0 = [0.0, 0.0, 0.0],
            z_floor= z_floor,
        )
        self._aero = (aero_override if aero_override is not None
                      else create_aero(rotor, model=aero_model))
        # Inflow state owned by the aero model.  omega is tracked separately
        # (moved out of RotorState in dynbem 0.2.0 — now an input).
        self._rotor_state    = self._aero.initial_rotor_state()
        self._omega_rad_s    = float(ic.omega_spin)
        self._omega_hold_start_rad_s = float(self._omega_rad_s)
        self._omega_release_target_rad_s = float(ic.omega_spin)
        self._spin_angle_rad = 0.0
        self._was_kinematic = bool(self._kinematic is not None and self._kinematic.is_active(0.0))

        if rotor.control is None or rotor.control.axle_attachment_length_m is None:
            raise ValueError("rotor.control.axle_attachment_length_m must be set")
        if tether_override is not None:
            self._tether = tether_override
        else:
            self._tether = TetherModel(
                anchor_ned             = np.zeros(3),
                rest_length            = float(ic.rest_length),
                axle_attachment_length = float(rotor.control.axle_attachment_length_m),
            )
        self._t_sim      = 0.0
        self._damp_alpha = 0.0

        # Bootstrap tether tension from IC position
        s = self._dyn.state
        self._tether.compute(s["pos"], s["vel"], s["R"])
        self._tension_now = float(self._tether._last_info.get("tension", 0.0))

    # ── Convenience constructor ───────────────────────────────────────────────

    @classmethod
    def from_state(cls, rotor, pos, vel, R0, rest_length, eq_thrust,
                   omega_spin, wind, **kwargs):
        ic = SimpleNamespace(
            pos        = np.asarray(pos, dtype=float),
            vel        = np.asarray(vel, dtype=float),
            R0         = R0,
            rest_length= float(rest_length),
            eq_thrust  = float(eq_thrust),
            omega_spin = float(omega_spin),
        )
        return cls(rotor, ic, wind, **kwargs)

    # ── Read-only state properties ────────────────────────────────────────────

    @property
    def hub_state(self) -> dict:
        return self._dyn.state

    @property
    def tension_now(self) -> float:
        return self._tension_now

    @property
    def omega_spin(self) -> float:
        return self._omega_rad_s

    @property
    def t_sim(self) -> float:
        return self._t_sim

    @property
    def aero(self):
        return self._aero

    @property
    def tether(self):
        return self._tether

    def hub_observe(self) -> HubObservation:
        """Return current observable hub state as a HubObservation."""
        hub = self._dyn.state
        return HubObservation(
            R          = hub["R"],
            pos        = hub["pos"],
            vel        = hub["vel"],
            body_z     = hub["R"][:, 2],
            gyro       = hub["R"].T @ hub["omega"],
            omega_spin = self._omega_rad_s,
        )

    @property
    def is_kinematic(self) -> bool:
        return self._kinematic is not None and self._kinematic.is_active(self._t_sim)

    @property
    def damp_alpha(self) -> float:
        if self._kinematic is None:
            return 0.0
        return float(self._kinematic.damp_alpha(self._t_sim))

    def _kinematic_omega_target(self, t_sim: float) -> float:
        """Return commanded omega during kinematic hold.

        Holds the startup omega, then blends to the IC release omega over the
        final KINEMATIC_SPINUP_S seconds so free-flight starts without a jump.
        """
        if self._kinematic is None:
            return self._omega_rad_s
        duration = float(self._kinematic.duration)
        if duration <= 0.0:
            return self._omega_release_target_rad_s

        spinup_s = min(self.KINEMATIC_SPINUP_S, duration)
        if spinup_s <= 0.0:
            return self._omega_release_target_rad_s

        t_ramp_start = duration - spinup_s
        if t_sim <= t_ramp_start:
            return self._omega_hold_start_rad_s

        u = np.clip((t_sim - t_ramp_start) / spinup_s, 0.0, 1.0)
        # Smoothstep blend avoids a slope discontinuity at ramp boundaries.
        u = u * u * (3.0 - 2.0 * u)
        return float(
            self._omega_hold_start_rad_s
            + (self._omega_release_target_rad_s - self._omega_hold_start_rad_s) * u
        )

    # ── Physics steps ─────────────────────────────────────────────────────────

    def step(self, dt: float, collective_rad: float,
             tilt_lon: float, tilt_lat: float,
             rest_length: "float | None" = None,
             yaw_throttle: "float | None" = None) -> dict:
        """
        400 Hz step with caller-supplied tilts.

        Use for mediator (ArduPilot servo decode) and Lua-controlled simtests
        (HeliCyclicController produces tilt_lon/tilt_lat).

        yaw_throttle : GB4008 tail-motor throttle command [0..1] (Motor4/SERVO9
            for the real SITL mediator; arduloop HeliRateOutput.yaw_cmd clamped
            to [0,1] for simtests).  Advances the same torque_model.HubState
            ESC-governor ODE used by the standalone counter-torque tests, so
            rotor/counter-rotation determines hub spin identically in both
            paths (see _integrate).  Default None reproduces the previous
            pure damping-to-zero behavior exactly (no hub-motor ODE advance).
        """
        return self._integrate(dt, collective_rad, tilt_lon, tilt_lat,
                                rest_length, yaw_throttle)

    def warm_inflow(self, collective_rad: float, n_steps: int = 500,
                    dt: float = 1e-3) -> None:
        """
        Integrate the aero inflow state in-place with the hub FROZEN at its
        current position/orientation/RPM.

        Call once after construction (before the first real physics step) to
        converge dynamic-inflow models from the zero initial condition to the
        trim equilibrium.  Quasi-static aero has no meaningful inflow transient,
        so this is effectively a no-op for the default flight model.

        Parameters
        ----------
        collective_rad : equilibrium collective [rad]
        n_steps        : number of ODE steps (default 500 @ 1 ms = 0.5 s)
        dt             : inflow ODE time step [s] (default 1e-3)
        """
        hub = self._dyn.state
        inputs = RotorInputs(
            collective_rad = collective_rad,
            tilt_lon       = 0.0,
            tilt_lat       = 0.0,
            R_hub          = hub["R"],
            v_hub_world    = hub["vel"],
            wind_world     = self._wind,
            omega_rad_s    = self._omega_rad_s,
            rho_kg_m3      = 1.225,
        )
        for _ in range(n_steps):
            _, self._rotor_state = self._aero.step(inputs, self._rotor_state, dt)

    def warm_inflow_from_thrust(self, thrust_cmd: float, n_steps: int = 500,
                                dt: float = 1e-3) -> None:
        """Thrust-domain wrapper around warm_inflow()."""
        thrust = float(np.clip(thrust_cmd, 0.0, 1.0))
        self.warm_inflow(_t2c(thrust), n_steps=n_steps, dt=dt)

    # ── Internal integration ──────────────────────────────────────────────────

    def _integrate(self, dt: float, collective_rad: float,
                   tilt_lon: float, tilt_lat: float,
                   rest_length: "float | None",
                   yaw_throttle: "float | None" = None) -> dict:

        if rest_length is not None:
            self._tether.rest_length = float(rest_length)

        hub = self._dyn.state
        r   = self._rotor

        kin_active = self.is_kinematic
        just_released = bool(self._was_kinematic and not kin_active)
        self._damp_alpha = self.damp_alpha if kin_active else 0.0

        if kin_active:
            tf = np.zeros(3)
            tm = np.zeros(3)
            self._tension_now = 0.0
            if self._kinematic_aero_mode == "nul":
                # Kinematic "nul" aero (rotation-only): lock translation to the
                # kinematic trajectory and apply cyclic-proportional body rates.
                # tilt_lat > 0 -> roll right (+roll), tilt_lon > 0 -> nose-down (-pitch).
                kin = self._kinematic.state_at(self._t_sim) if self._kinematic is not None else None
                if kin is not None:
                    pos_kin, vel_kin = kin
                else:
                    pos_kin = hub["pos"]
                    vel_kin = hub["vel"]

                gain = self._kinematic_nul_rate_gain_rads_per_rad
                omega_body = np.array([
                    gain * tilt_lat,
                    -gain * tilt_lon,
                    0.0,
                ])
                omega_world = hub["R"] @ omega_body

                # First-order orientation integration with re-orthonormalization.
                wx, wy, wz = omega_world
                skew = np.array([
                    [0.0, -wz,  wy],
                    [wz,   0.0, -wx],
                    [-wy,  wx,  0.0],
                ])
                R_next = hub["R"] + dt * (skew @ hub["R"])
                U, _, Vt = np.linalg.svd(R_next)
                R_next = U @ Vt

                new_hub = {
                    "pos": pos_kin,
                    "vel": vel_kin,
                    "R": R_next,
                    "omega": omega_world,
                }
                self._dyn._pos[:] = pos_kin
                self._dyn._vel[:] = vel_kin
                self._dyn._R[:] = R_next
                self._dyn._omega[:] = omega_world

                F_net = np.zeros(3)
                result = SimpleNamespace(
                    F_world=np.zeros(3),
                    m_hub_world=np.zeros(3),
                    M_spin=np.zeros(3),
                    Q_spin=0.0,
                )
            else:
                # Kinematic hold: bypass all physics contributions.
                # No tether, no aero, no gravity-driven dynamics step, and no rotor spin update.
                self._omega_rad_s = self._kinematic_omega_target(self._t_sim)
                result = SimpleNamespace(
                    F_world=np.zeros(3),
                    m_hub_world=np.zeros(3),
                    M_spin=np.zeros(3),
                    Q_spin=0.0,
                )
                F_net = np.zeros(3)
                new_hub = hub
        else:
            tf, tm            = self._tether.compute(hub["pos"], hub["vel"], hub["R"])
            self._tension_now = float(self._tether._last_info.get("tension", 0.0))

            # Aerodynamic forces — new state-based API.  Aero model integrates
            # omega, spin angle, and inflow states through the returned derivative.
            rotor_inputs = RotorInputs(
                collective_rad = collective_rad,
                tilt_lon       = tilt_lon,
                tilt_lat       = tilt_lat,
                R_hub          = hub["R"],
                v_hub_world    = hub["vel"],
                wind_world     = self._wind,
                omega_rad_s    = self._omega_rad_s,
                rho_kg_m3      = 1.225,
            )
            # Aero integrates its inflow state internally via step(); omega is a
            # RotorInputs field (not part of RotorState) so it stays external.
            result, self._rotor_state = self._aero.step(rotor_inputs, self._rotor_state, dt)

            # Integrate omega externally using euler_step_omega
            omega_min = (r.autorotation.omega_min_rad_s
                         if r.autorotation.omega_min_rad_s is not None else 0.5)
            I_ode = (r.autorotation.I_ode_kgm2
                     if r.autorotation.I_ode_kgm2 is not None else 10.0)
            if just_released:
                # Preserve IC release RPM exactly on the first free-flight step.
                # This avoids a transition artifact where immediate aero torque
                # can pull omega away from the kinematic target before the
                # mediator logs kinematic_exit.
                self._omega_rad_s = max(omega_min, float(self._omega_release_target_rad_s))
            else:
                new_omega, new_spin = euler_step_omega(
                    self._omega_rad_s, self._spin_angle_rad,
                    float(result.Q_spin), 0.0, I_ode, dt,
                )
                self._omega_rad_s    = max(omega_min, new_omega)
                self._spin_angle_rad = new_spin

            disk_normal = hub["R"][:, 2]

            # Angular damping
            # base term  : optional diagnostic damping; default 0 in free flight
            # startup extra: additional damping during kinematic ramp
            # k_yaw term : GB4008 counter-torque around disk_normal (rotor axle).
            # Yaw authority: the hub yaw rate is damped toward yaw_rate_target,
            # which comes from advancing torque_model.HubState (same ESC-governor
            # + finite-torque ODE as the standalone counter-torque tests) using
            # the caller-supplied yaw_throttle.  Rotor spin (self._omega_rad_s)
            # and the gear-reflected motor speed together determine hub spin,
            # exactly as in the torque tests -- this replaces the earlier
            # instantaneous/algebraic target with a slew-rate-limited one, so
            # throttle changes can no longer inject torque-impulse jumps.
            # yaw_throttle is None for callers that don't drive a yaw motor
            # (analysis scripts, legacy tests): reproduces the previous pure
            # damping-to-zero behavior exactly, with no hub ODE advance.
            if yaw_throttle is None:
                yaw_rate_target = 0.0
            else:
                self._hub_state = _hub_step(
                    self._hub_state, self._omega_rad_s, float(yaw_throttle),
                    self._hub_params, dt,
                )
                yaw_rate_target = self._hub_state.psi_dot
            k_total   = self._base_k_ang + self._startup_damp_k_ang * self._damp_alpha
            omega_yaw = float(np.dot(hub["omega"], disk_normal))
            M_net = (result.m_hub_world + tm
                     - k_total * hub["omega"]
                     - self._k_yaw * (omega_yaw - yaw_rate_target) * disk_normal)

            # 6-DOF rigid-body integration (gravity applied internally)
            F_net   = result.F_world + tf
            new_hub = self._dyn.step(F_net, M_net, dt,
                                     omega_spin=self._omega_rad_s)

        # Kinematic state override — applied post-dynamics so t_sim is pre-advance
        if self._kinematic is not None and not (kin_active and self._kinematic_aero_mode == "nul"):
            self._kinematic.apply(new_hub, self._dyn, self._t_sim)

        self._was_kinematic = kin_active
        self._t_sim += dt

        return {
            "hub_state":    new_hub,
            "tension_now":  self._tension_now,
            "omega_spin":   self._omega_rad_s,
            "tether_force": tf,
            "tether_moment": tm,
            "aero_result":  result,
            "tilt_lon":     tilt_lon,
            "tilt_lat":     tilt_lat,
            "damp_alpha":   self._damp_alpha,
            "is_kinematic": kin_active,
            # Specific force in NED world frame: (F_aero + F_tether) / mass.
            # Gravity is excluded — this is what the IMU accelerometer measures.
            # Body frame: accel_body = R.T @ accel_specific_world
            "accel_specific_world": F_net / self._dyn.mass,
        }
