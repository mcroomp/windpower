"""
simtest_runner.py — Shared 400 Hz physics core for simtests.

PhysicsRunner is a thin wrapper around PhysicsCore (simulation/physics_core.py).

    step(dt, collective, rate_roll, rate_pitch, omega_body)
        → runs AcroControllerSitl (baked in) then core.step()

PhysicsCore owns all physics constants (base_k_ang, k_yaw, T_AERO_OFFSET) and
the integration loop (dynamics, aero, tether, spin ODE, angular damping).
Callers own planners, TensionPI, WinchController — at their own rates.

LuaAP
-----
Helper for Lua simtests.  Wraps the 50-100 Hz tick pattern that every Lua test
repeats: millis update → feed_obs → update_fn → PWM decode.  After tick() the
decoded (col_rad, roll_sp, pitch_sp) are held on the instance and forwarded to
runner.step() at 400 Hz.
"""
import math
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from physics_core import PhysicsCore, HubObservation
from controller   import AcroControllerSitl


class PhysicsRunner:
    """
    Simtest wrapper around PhysicsCore.

    Exposes the same API used by all existing simtests while delegating all
    physics integration to PhysicsCore (shared with the mediator).

    Usage
    -----
    runner = PhysicsRunner(rotor, ic, wind)
    for i in range(steps):
        sr = runner.step(DT, collective_rad, rate_roll, rate_pitch, omega_body, rest_length=winch.rest_length)
        tension_now = runner.tension_now
        hub = runner.hub_state
    """

    # Expose constants so simtests that reference PhysicsRunner.BASE_K_ANG still work
    BASE_K_ANG    = PhysicsCore.BASE_K_ANG
    T_AERO_OFFSET = PhysicsCore.T_AERO_OFFSET

    def __init__(self, rotor, ic, wind, *, z_floor: float = -1.0,
                 aero_model: str = "skewed_wake", aero_override=None,
                 col_min_rad: float = -0.28, col_max_rad: float = 0.10):
        """
        Parameters
        ----------
        rotor         : RotorDefinition
        ic            : object with .pos, .vel, .R0, .rest_length, .coll_eq_rad, .omega_spin
        wind          : NED wind vector [m/s]
        z_floor       : NED Z floor for dynamics (default -1.0 m = 1 m altitude floor)
        aero_model    : aero model key passed to create_aero() (default "skewed_wake")
        aero_override : if provided, used directly instead of create_aero()
        col_min_rad   : collective floor for AcroControllerSitl servo model
        col_max_rad   : collective ceiling for AcroControllerSitl servo model
        """
        self._core = PhysicsCore(rotor, ic, wind, z_floor=z_floor,
                                 aero_model=aero_model, aero_override=aero_override)
        self._acro = AcroControllerSitl(rotor, col_min_rad=col_min_rad,
                                        col_max_rad=col_max_rad)
        self._acro._servo.reset(ic.coll_eq_rad)

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

    @property
    def omega_body(self) -> np.ndarray:
        """Body-frame angular velocity (mutable copy). Caller may zero yaw channel."""
        hs = self._core.hub_state
        return np.asarray(hs["R"], dtype=float).T @ np.asarray(hs["omega"], dtype=float)

    @property
    def altitude(self) -> float:
        """Hub altitude above anchor [m] = -pos_z (NED)."""
        return -float(self._core.hub_state["pos"][2])

    # ── Physics steps ─────────────────────────────────────────────────────────

    def step(self, dt: float, collective_rad: float,
             rate_roll: float, rate_pitch: float,
             omega_body: np.ndarray,
             *, rest_length: "float | None" = None) -> dict:
        """
        400 Hz step for Python-AP tests.

        Runs AcroControllerSitl (baked-in RatePID + servo model) then physics.
        Use when a Python AP controller produces (collective, rate_roll, rate_pitch).
        """
        tlon, tlat, col_act = self._acro.step(
            collective_rad, rate_roll, rate_pitch, omega_body, dt)
        return self._core.step(dt, col_act, tlon, tlat, rest_length)

    def observe(self) -> HubObservation:
        """Return current observable hub state (sensor boundary enforcement)."""
        return self._core.hub_observe()


def tel_every_from_env(dt: float, default_hz: float = 20.0) -> int:
    """
    Return the telemetry decimation factor from RAWES_TEL_HZ env var.

    Usage in simtests::

        tel_every = tel_every_from_env(DT)   # default 20 Hz
        ...
        if i % tel_every == 0:
            telemetry.append(TelRow.from_physics(...))

    Override::

        RAWES_TEL_HZ=400 pytest simulation/tests/simtests/test_pump_cycle_unified.py -s -q
    """
    hz = float(os.environ.get("RAWES_TEL_HZ", default_hz))
    return max(1, round(1.0 / (hz * dt)))


def feed_obs(sim, obs: HubObservation) -> None:
    """Write a HubObservation into a RawesLua mock (AHRS fields only)."""
    sim.R       = obs.R
    sim.pos_ned = obs.pos.tolist()
    sim.vel_ned = obs.vel.tolist()
    sim.gyro    = obs.gyro.tolist()


class LuaAP:
    """
    Lua AP tick helper for simtests.

    Encapsulates the 50-100 Hz tick pattern common to all Lua simtests:
    millis update → feed_obs → update_fn → PWM decode.

    After tick(), col_rad/roll_sp/pitch_sp hold the latest decoded channel
    values and can be forwarded to runner.step() at 400 Hz between ticks.

    Usage
    -----
    lua = LuaAP(sim, initial_col_rad=ic.coll_eq_rad)
    for i in range(max_steps):
        if i % lua_every == 0:
            lua.tick(t_sim, runner)
        omega_body    = runner.omega_body
        omega_body[2] = 0.0   # yaw not controlled by Lua AP
        sr = runner.step(DT, lua.col_rad, lua.roll_sp, lua.pitch_sp, omega_body)
    """

    COL_MIN    = -0.28
    COL_MAX    =  0.10
    ACRO_SCALE = 500.0 / (360.0 * math.pi / 180.0)

    def __init__(self, sim, *, initial_col_rad: float = 0.0) -> None:
        self._sim     = sim
        self.col_rad  = float(initial_col_rad)
        self.roll_sp  = 0.0
        self.pitch_sp = 0.0

    def tick(self, t_sim: float, runner: PhysicsRunner, *,
             inject=None) -> None:
        """
        Feed physics obs into Lua, run update_fn, decode PWM channels.

        inject : optional callable(sim, runner) called between feed_obs and
                 update_fn — used to push NV floats (e.g. RAWES_TEN) into Lua
                 before the update runs.
        """
        self._sim._mock.millis_val = int(t_sim * 1000)
        feed_obs(self._sim, runner.observe())
        if inject is not None:
            inject(self._sim, runner)
        self._sim._update_fn()
        ch1 = self._sim.ch_out[1]
        ch2 = self._sim.ch_out[2]
        ch3 = self._sim.ch_out[3]
        if ch1 is not None:
            self.roll_sp = (ch1 - 1500) / self.ACRO_SCALE
        if ch2 is not None:
            self.pitch_sp = (ch2 - 1500) / self.ACRO_SCALE
        if ch3 is not None:
            self.col_rad = self.COL_MIN + (ch3 - 1000) / 1000.0 * (self.COL_MAX - self.COL_MIN)
