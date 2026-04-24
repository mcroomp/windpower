"""
simtest_runner.py — Shared 400 Hz physics core for simtests.

PhysicsRunner owns dynamics, aero, tether, AcroController, and spin state.
Callers own planners and TensionPI (at whatever rate they run) and supply
collective_rad + body_z_eq to each step() call.

Eliminates copy-pasted boilerplate and ensures identical physics constants
across all simtests that use it.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from dynamics   import RigidBodyDynamics
from aero       import create_aero
from tether     import TetherModel
from controller import AcroController


class PhysicsRunner:
    """
    400 Hz physics core shared across simtests.

    Owns: RigidBodyDynamics, aero, TetherModel, AcroController, spin ODE.
    Caller owns: planners, TensionPI, WinchController — all at their own rates.

    Usage
    -----
    runner = PhysicsRunner(rotor, ic, wind)
    for i in range(steps):
        # caller computes collective_rad and body_z_eq (from TensionPI / planner)
        step = runner.step(DT, collective_rad, body_z_eq, rest_length=winch.rest_length)
        tension_now = runner.tension_now
        hub = runner.hub_state
    """

    BASE_K_ANG    = 50.0   # N·m·s/rad — angular damping (matches mediator default)
    T_AERO_OFFSET = 45.0   # s — aero ramp already complete at simulation start

    def __init__(self, rotor, ic, wind, *, z_floor: float = -1.0):
        """
        Parameters
        ----------
        rotor   : RotorDefinition — single source of all physical constants
        ic      : object with .pos, .vel, .R0, .rest_length, .coll_eq_rad, .omega_spin
                  (SimtestIC from simtest_ic.py, or a SimpleNamespace for warmup runs)
        wind    : NED wind vector [m/s]
        z_floor : NED Z floor for dynamics (default -1.0 m = 1 m altitude floor)
        """
        self._rotor = rotor
        self._wind  = np.asarray(wind, dtype=float).copy()
        self._wind.flags.writeable = False

        self._dyn = RigidBodyDynamics(
            **rotor.dynamics_kwargs(),
            pos0=list(ic.pos),
            vel0=list(ic.vel),
            R0=ic.R0,
            omega0=[0.0, 0.0, 0.0],
            z_floor=z_floor,
        )
        self._aero   = create_aero(rotor)
        self._tether = TetherModel(
            anchor_ned=np.zeros(3),
            rest_length=float(ic.rest_length),
            axle_attachment_length=rotor.axle_attachment_length_m,
        )
        self._acro = AcroController.from_rotor(
            rotor, use_servo=True, collective_rad=float(ic.coll_eq_rad),
        )
        self._omega_spin = float(ic.omega_spin)
        self._t_sim      = 0.0

        # Bootstrap tether tension from IC position
        s = self._dyn.state
        self._tether.compute(s["pos"], s["vel"], s["R"])
        self._tension_now = float(self._tether._last_info.get("tension", 0.0))

    # ── Convenience constructor for warmup / IC generation runs ───────────────

    @classmethod
    def for_warmup(cls, rotor, pos, R0, rest_length, coll_eq_rad, omega_spin, wind):
        """Construct from explicit state — used when no SimtestIC exists yet."""
        ic = SimpleNamespace(
            pos=np.asarray(pos, dtype=float),
            vel=np.zeros(3),
            R0=R0,
            rest_length=float(rest_length),
            coll_eq_rad=float(coll_eq_rad),
            omega_spin=float(omega_spin),
        )
        return cls(rotor, ic, wind)

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
        """Direct tether access — for inspecting _last_info or slack state."""
        return self._tether

    # ── Physics steps ─────────────────────────────────────────────────────────

    def step_raw(self, dt: float, collective_rad: float,
                 tilt_lon: float, tilt_lat: float,
                 rest_length: "float | None" = None) -> dict:
        """
        400 Hz physics step with caller-supplied tilts (no AcroController).

        Use for Lua-controlled tests where Lua PWM → RatePID + SwashplateServoModel
        produces tilt_lon/tilt_lat.  The caller owns all attitude control; this
        method owns tether + aero + spin ODE + dynamics.
        """
        if rest_length is not None:
            self._tether.rest_length = float(rest_length)

        hub = self._dyn.state
        r   = self._rotor

        tf, tm            = self._tether.compute(hub["pos"], hub["vel"], hub["R"])
        self._tension_now = float(self._tether._last_info.get("tension", 0.0))

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

        Q_spin = (r.K_drive_Nms_m * self._aero.last_v_inplane
                  - r.K_drag_Nms2_rad2 * self._omega_spin ** 2)
        self._omega_spin = max(
            r.omega_min_rad_s,
            self._omega_spin + Q_spin / r.I_ode_kgm2 * dt,
        )

        F_net   = result.F_world   + tf
        M_net   = result.M_orbital + tm - self.BASE_K_ANG * hub["omega"]
        new_hub = self._dyn.step(F_net, M_net, dt, omega_spin=self._omega_spin)

        self._t_sim += dt

        return {
            "hub_state":     new_hub,
            "tension_now":   self._tension_now,
            "omega_spin":    self._omega_spin,
            "tether_force":  tf,
            "tether_moment": tm,
            "aero_result":   result,
            "tilt_lon":      tilt_lon,
            "tilt_lat":      tilt_lat,
        }

    def step(self, dt: float, collective_rad: float, body_z_eq: np.ndarray,
             rest_length: "float | None" = None) -> dict:
        """
        Run one 400 Hz physics step.

        Parameters
        ----------
        dt            : timestep [s] (typically 1/400)
        collective_rad: blade pitch [rad] — from TensionPI or fixed
        body_z_eq     : target rotor-axis NED unit vector
        rest_length   : if provided, updates tether rest length before computing

        Returns a dict with all outputs needed for telemetry and event logging.
        """
        if rest_length is not None:
            self._tether.rest_length = float(rest_length)

        hub = self._dyn.state
        r   = self._rotor

        # Tether forces
        tf, tm            = self._tether.compute(hub["pos"], hub["vel"], hub["R"])
        self._tension_now = float(self._tether._last_info.get("tension", 0.0))

        # Attitude control + servo slew
        tilt_lon, tilt_lat = self._acro.update(hub, body_z_eq, dt)

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

        # Rotor spin ODE:  dω/dt = (K_drive·v_inplane − K_drag·ω²) / I_spin
        Q_spin = (r.K_drive_Nms_m * self._aero.last_v_inplane
                  - r.K_drag_Nms2_rad2 * self._omega_spin ** 2)
        self._omega_spin = max(
            r.omega_min_rad_s,
            self._omega_spin + Q_spin / r.I_ode_kgm2 * dt,
        )

        # 6-DOF rigid-body integration
        F_net   = result.F_world   + tf
        M_net   = result.M_orbital + tm - self.BASE_K_ANG * hub["omega"]
        new_hub = self._dyn.step(F_net, M_net, dt, omega_spin=self._omega_spin)

        self._t_sim += dt

        return {
            "hub_state":     new_hub,
            "tension_now":   self._tension_now,
            "omega_spin":    self._omega_spin,
            "tether_force":  tf,
            "tether_moment": tm,
            "aero_result":   result,
            "tilt_lon":      tilt_lon,
            "tilt_lat":      tilt_lat,
        }
