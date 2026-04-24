"""
simtest_runner.py — Shared 400 Hz physics core for simtests.

PhysicsRunner is a thin wrapper around PhysicsCore (simulation/physics_core.py),
translating the simtest API into the core's two step methods:

    step(dt, collective, body_z_eq)                → core.step_acro()  [Python-controlled]
    step_raw(dt, collective, tilt_lon, tilt_lat)   → core.step()       [Lua-controlled]

PhysicsCore owns all physics constants (base_k_ang, k_yaw, T_AERO_OFFSET) and
the integration loop (dynamics, aero, tether, spin ODE, angular damping).
Callers own planners, TensionPI, WinchController — at their own rates.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from physics_core import PhysicsCore, HubObservation


class PhysicsRunner:
    """
    Simtest wrapper around PhysicsCore.

    Exposes the same API used by all existing simtests while delegating all
    physics integration to PhysicsCore (shared with the mediator).

    Usage
    -----
    runner = PhysicsRunner(rotor, ic, wind)
    for i in range(steps):
        sr = runner.step(DT, collective_rad, body_z_eq, rest_length=winch.rest_length)
        tension_now = runner.tension_now
        hub = runner.hub_state
    """

    # Expose constants so simtests that reference PhysicsRunner.BASE_K_ANG still work
    BASE_K_ANG    = PhysicsCore.BASE_K_ANG
    T_AERO_OFFSET = PhysicsCore.T_AERO_OFFSET

    def __init__(self, rotor, ic, wind, *, z_floor: float = -1.0):
        """
        Parameters
        ----------
        rotor   : RotorDefinition
        ic      : object with .pos, .vel, .R0, .rest_length, .coll_eq_rad, .omega_spin
        wind    : NED wind vector [m/s]
        z_floor : NED Z floor for dynamics (default -1.0 m = 1 m altitude floor)
        """
        self._core = PhysicsCore(rotor, ic, wind, z_floor=z_floor)

    # ── Convenience constructor for warmup / IC generation runs ───────────────

    @classmethod
    def for_warmup(cls, rotor, pos, R0, rest_length, coll_eq_rad, omega_spin, wind):
        """Construct from explicit state — used when no SimtestIC exists yet."""
        ic = SimpleNamespace(
            pos        = np.asarray(pos, dtype=float),
            vel        = np.zeros(3),
            R0         = R0,
            rest_length= float(rest_length),
            coll_eq_rad= float(coll_eq_rad),
            omega_spin = float(omega_spin),
        )
        return cls(rotor, ic, wind)

    # ── Read-only state properties ────────────────────────────────────────────

    @property
    def hub_state(self) -> dict:
        return self._core.hub_state

    @property
    def tension_now(self) -> float:
        return self._core.tension_now

    @property
    def omega_spin(self) -> float:
        return self._core.omega_spin

    @property
    def t_sim(self) -> float:
        return self._core.t_sim

    @property
    def aero(self):
        return self._core.aero

    @property
    def tether(self):
        return self._core.tether

    # ── Physics steps ─────────────────────────────────────────────────────────

    def step(self, dt: float, collective_rad: float, body_z_eq: np.ndarray,
             rest_length: "float | None" = None) -> dict:
        """
        400 Hz step with AcroController-driven tilts (Python-controlled tests).

        AcroController converts body_z_eq → tilt_lon/tilt_lat internally.
        Returns dict with hub_state, tension_now, omega_spin, tether_force,
        tether_moment, aero_result, tilt_lon, tilt_lat.
        """
        return self._core.step_acro(dt, collective_rad, body_z_eq, rest_length)

    def step_raw(self, dt: float, collective_rad: float,
                 tilt_lon: float, tilt_lat: float,
                 rest_length: "float | None" = None) -> dict:
        """
        400 Hz step with caller-supplied tilts (Lua-controlled tests).

        Use when Lua PWM → RatePID + SwashplateServoModel produces tilt_lon/tilt_lat.
        """
        return self._core.step(dt, collective_rad, tilt_lon, tilt_lat, rest_length)

    def observe(self) -> HubObservation:
        """Return current observable hub state (sensor boundary enforcement)."""
        return self._core.hub_observe()


def feed_obs(sim, obs: HubObservation) -> None:
    """Write a HubObservation into a RawesLua mock (AHRS fields only)."""
    sim.R       = obs.R
    sim.pos_ned = obs.pos.tolist()
    sim.vel_ned = obs.vel.tolist()
    sim.gyro    = obs.gyro.tolist()
