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
- Angular damping: base_k_ang (omnidirectional) + k_yaw (GB4008 yaw axis)
- KinematicStartup: state override + tether gating + extra startup damping
- Time tracking (_t_sim)

Callers own
-----------
- Trajectory planners (HoldPlanner, etc.)
- TensionPI / collective management
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
from aero       import create_aero
from tether     import TetherModel
from frames     import build_orb_frame


def q_spin_from_aero(aero, R_hub: np.ndarray) -> float:
    """
    Extract spin torque [N*m] from the most recent aero.compute_forces() call.

    Returns dot(last_M_spin, disk_normal) — positive = spin-up.
    Must be called immediately after compute_forces(); uses cached last_M_spin.
    Handles both in-plane (crosswind) and axial-flow (descent) autorotation.
    """
    return float(np.dot(aero.last_M_spin, R_hub[:, 2]))


def step_spin_ode(omega: float, Q_spin: float,
                  I_ode_kgm2: float, omega_min_rad_s: float, dt: float) -> float:
    """
    Single Euler step of the rotor spin ODE.

    omega_new = max(omega_min, omega + Q_spin / I_ode * dt)
    """
    return max(omega_min_rad_s, omega + Q_spin / I_ode_kgm2 * dt)


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
        from GB4008 ESC telemetry RPM divided by the 80:44 gear ratio.
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
                          .coll_eq_rad, .omega_spin
    wind                : NED wind vector [m/s]
    base_k_ang          : omnidirectional angular damping [N·m·s/rad]
    k_yaw               : GB4008 yaw damper around disk_normal [N·m·s/rad]
    kinematic           : KinematicStartup | None
    startup_damp_k_ang  : extra angular damping applied during kinematic ramp
    z_floor             : NED Z floor for dynamics (default -1.0 m = 1 m altitude floor)
    """

    BASE_K_ANG    = 50.0    # N·m·s/rad — matches mediator default base_k_ang
    K_YAW_DEFAULT = 100.0   # N·m·s/rad — matches mediator _K_YAW_DEFAULT
    T_AERO_OFFSET = 45.0    # s — aero ramp already complete at simulation start

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
        lock_orientation:   bool  = False,
        z_floor:            float = -1.0,
        aero_model:         str   = "skewed_wake",
        aero_override             = None,
    ):
        self._rotor              = rotor
        self._wind               = np.asarray(wind, dtype=float).copy()
        self._wind.flags.writeable = False
        self._base_k_ang         = float(base_k_ang)
        self._k_yaw              = float(k_yaw)
        self._kinematic          = kinematic
        self._startup_damp_k_ang = float(startup_damp_k_ang)
        self._lock_orientation   = bool(lock_orientation)

        self._dyn = RigidBodyDynamics(
            **rotor.dynamics_kwargs(),
            pos0   = list(ic.pos),
            vel0   = list(ic.vel),
            R0     = ic.R0,
            omega0 = [0.0, 0.0, 0.0],
            z_floor= z_floor,
        )
        self._aero   = aero_override if aero_override is not None else create_aero(rotor, model=aero_model)
        self._tether = TetherModel(
            anchor_ned             = np.zeros(3),
            rest_length            = float(ic.rest_length),
            axle_attachment_length = rotor.axle_attachment_length_m,
        )
        self._omega_spin = float(ic.omega_spin)
        self._t_sim      = 0.0
        self._damp_alpha = 0.0

        # Bootstrap tether tension from IC position
        s = self._dyn.state
        self._tether.compute(s["pos"], s["vel"], s["R"])
        self._tension_now = float(self._tether._last_info.get("tension", 0.0))

    # ── Convenience constructor ───────────────────────────────────────────────

    @classmethod
    def from_state(cls, rotor, pos, vel, R0, rest_length, coll_eq_rad,
                   omega_spin, wind, **kwargs):
        """Construct from explicit state arrays — used when no IC object exists."""
        ic = SimpleNamespace(
            pos        = np.asarray(pos, dtype=float),
            vel        = np.asarray(vel, dtype=float),
            R0         = R0,
            rest_length= float(rest_length),
            coll_eq_rad= float(coll_eq_rad),
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
        return self._omega_spin

    @property
    def t_sim(self) -> float:
        return self._t_sim

    @property
    def aero(self):
        return self._aero

    @property
    def tether(self):
        return self._tether

    def hub_observe(self) -> dict:
        """
        Raw observable state — only what real hardware sensors can measure.

        Returns a plain dict so physics_core.py stays free of dataclass
        dependencies.  PhysicsRunner.observe() wraps this in SensorReading.

        Keys
        ----
        R           ndarray (3,3) — body→world rotation matrix (AHRS)
        pos         ndarray (3,)  — NED position [m] (GPS/EKF)
        vel         ndarray (3,)  — NED velocity [m/s] (GPS/EKF)
        omega_world ndarray (3,)  — world-frame angular velocity [rad/s] (IMU input;
                                    sensor computes gyro_body = R.T @ omega_world)
        omega_spin  float         — rotor spin rate [rad/s] (ESC telemetry)
        """
        hub = self._dyn.state
        return {
            "R":           hub["R"],
            "pos":         hub["pos"],
            "vel":         hub["vel"],
            "omega_world": hub["omega"],
            "omega_spin":  self._omega_spin,
        }

    def hub_observe(self) -> HubObservation:
        """Return current observable hub state as a HubObservation."""
        hub = self._dyn.state
        return HubObservation(
            R          = hub["R"],
            pos        = hub["pos"],
            vel        = hub["vel"],
            body_z     = hub["R"][:, 2],
            gyro       = hub["R"].T @ hub["omega"],
            omega_spin = self._omega_spin,
        )

    @property
    def is_kinematic(self) -> bool:
        return self._damp_alpha > 0.0

    @property
    def damp_alpha(self) -> float:
        return self._damp_alpha

    # ── Physics steps ─────────────────────────────────────────────────────────

    def step(self, dt: float, collective_rad: float,
             tilt_lon: float, tilt_lat: float,
             rest_length: "float | None" = None) -> dict:
        """
        400 Hz step with caller-supplied tilts.

        Use for mediator (ArduPilot servo decode) and Lua-controlled simtests
        (RatePID + SwashplateServoModel produce tilt_lon/tilt_lat).
        """
        return self._integrate(dt, collective_rad, tilt_lon, tilt_lat, rest_length)

    # ── Internal integration ──────────────────────────────────────────────────

    def _integrate(self, dt: float, collective_rad: float,
                   tilt_lon: float, tilt_lat: float,
                   rest_length: "float | None") -> dict:

        if rest_length is not None:
            self._tether.rest_length = float(rest_length)

        hub = self._dyn.state
        r   = self._rotor

        # Kinematic blend factor: 1 at startup, ramps to 0, then 0 in free flight
        self._damp_alpha = (self._kinematic.damp_alpha(self._t_sim)
                            if self._kinematic is not None else 0.0)

        # Tether — zeroed during kinematic (hub may be outside tether envelope)
        if self._damp_alpha > 0.0:
            tf, tm            = np.zeros(3), np.zeros(3)
            self._tension_now = 0.0
        else:
            tf, tm            = self._tether.compute(hub["pos"], hub["vel"], hub["R"])
            self._tension_now = float(self._tether._last_info.get("tension", 0.0))

        # Aerodynamic forces
        result = self._aero.compute_forces(
            collective_rad = collective_rad,
            tilt_lon       = tilt_lon,
            tilt_lat       = tilt_lat,
            R_hub          = hub["R"],
            v_hub_world    = hub["vel"],
            omega_rotor    = self._omega_spin,
            wind_world     = self._wind,
            t              = self.T_AERO_OFFSET + self._t_sim,
        )

        # Spin ODE: dω/dt = Q_aero / I_ode
        disk_normal      = hub["R"][:, 2]
        Q_spin           = q_spin_from_aero(self._aero, hub["R"])
        self._omega_spin = step_spin_ode(self._omega_spin, Q_spin,
                                         r.I_ode_kgm2, r.omega_min_rad_s, dt)

        # Angular damping / lock_orientation
        if self._lock_orientation:
            # Magic tether: zero moments so integrator never accelerates omega.
            M_net = np.zeros(3)
        else:
            # base term  : opposes all orbital angular velocity (prevents tumbling)
            # startup extra: additional damping during kinematic ramp
            # k_yaw term : GB4008 counter-torque around disk_normal (rotor axle)
            k_total   = self._base_k_ang + self._startup_damp_k_ang * self._damp_alpha
            omega_yaw = float(np.dot(hub["omega"], disk_normal))
            M_net = (result.M_orbital + tm
                     - k_total * hub["omega"]
                     - self._k_yaw * omega_yaw * disk_normal)

        # 6-DOF rigid-body integration (gravity applied internally)
        F_net   = result.F_world + tf
        new_hub = self._dyn.step(F_net, M_net, dt, omega_spin=self._omega_spin)

        # lock_orientation post-step: track tether direction, zero rotation
        if self._lock_orientation and self._damp_alpha == 0.0:
            cur_bz         = new_hub["pos"] / np.linalg.norm(new_hub["pos"])
            R_lock         = build_orb_frame(cur_bz)
            new_hub["R"]   = R_lock.copy()
            self._dyn._R[:]     = R_lock
            new_hub["omega"][:] = 0.0
            self._dyn._omega[:] = 0.0

        # Kinematic state override — applied post-dynamics so t_sim is pre-advance
        if self._kinematic is not None:
            self._kinematic.apply(new_hub, self._dyn, self._t_sim)

        self._t_sim += dt

        return {
            "hub_state":    new_hub,
            "tension_now":  self._tension_now,
            "omega_spin":   self._omega_spin,
            "tether_force": tf,
            "tether_moment": tm,
            "aero_result":  result,
            "tilt_lon":     tilt_lon,
            "tilt_lat":     tilt_lat,
            "damp_alpha":   self._damp_alpha,
            "is_kinematic": self._damp_alpha > 0.0,
            # Specific force in NED world frame: (F_aero + F_tether) / mass.
            # Gravity is excluded — this is what the IMU accelerometer measures.
            # Body frame: accel_body = R.T @ accel_specific_world
            "accel_specific_world": F_net / self._dyn.mass,
        }
