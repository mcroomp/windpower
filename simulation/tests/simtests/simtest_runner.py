"""
simtest_runner.py — Shared 400 Hz physics core for simtests.

PhysicsRunner is a thin wrapper around PhysicsCore (simulation/physics_core.py).

    step(dt, collective, rate_roll, rate_pitch, omega_body)
        → runs HeliCyclicController (baked in) then core.step()

PhysicsCore owns all physics constants (k_yaw, T_AERO_OFFSET) and
the integration loop (dynamics, aero, tether, spin ODE, angular damping).
Callers own planners, TensionPI, WinchController — at their own rates.

MockArdupilot
-------------
Public adapter entrypoint for simtests. Use:

    MockArdupilot.for_lua(...)
    MockArdupilot.for_python(...)

Implementation lives in tests/common/mock_ardupilot.py.

Lua backend wraps the 50-100 Hz tick pattern that every Lua test
repeats: millis update → feed_obs → update_fn → PWM decode.

In GUIDED mode rawes.lua drives cyclic via vehicle:set_target_angle_and_climbrate
(stored in _mock.guided_target) rather than ch1/ch2 RC overrides. The Lua backend
feeds guided_target into a GuidedAttitudeController (400 Hz inner loop) and
calls runner.step_guided() so physics sees the correct swashplate tilt.
Collective is sourced from the guided vertical channel when available,
with ch3 kept as a legacy fallback.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from physics_core  import PhysicsCore, HubObservation
from controller    import HeliCyclicController
from param_defaults import thrust_to_coll_rad as _t2c, load_collective_phys_range as _load_col_range


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

    # Expose constants so simtests that reference PhysicsRunner.BASE_K_ANG still work.
    # Default is zero; nonzero values are diagnostic-only.
    BASE_K_ANG    = PhysicsCore.BASE_K_ANG
    T_AERO_OFFSET = PhysicsCore.T_AERO_OFFSET

    def __init__(self, rotor, ic, wind, *, z_floor: float = -1.0,
                 aero_model: str = "quasi_static", aero_override=None):
        """
        Parameters
        ----------
        rotor         : RotorDefinition
        ic            : object with .pos, .vel, .R0, .rest_length, .eq_thrust, .omega_spin
                and optional .trim_tilt_lon/.trim_tilt_lat cyclic trim
        wind          : NED wind vector [m/s]
        z_floor       : NED Z floor for dynamics (default -1.0 m = 1 m altitude floor)
        aero_model    : aero model key passed to create_aero() (default "quasi_static" = no inflow state)
        aero_override : if provided, used directly instead of create_aero()
        """
        self._core = PhysicsCore(rotor, ic, wind, z_floor=z_floor,
                                 aero_model=aero_model, aero_override=aero_override)
        self._tilt_lon_trim = float(getattr(ic, "trim_tilt_lon", 0.0))
        self._tilt_lat_trim = float(getattr(ic, "trim_tilt_lat", 0.0))
        self._acro = HeliCyclicController(rotor)
        self._acro.set_trim(self._tilt_lon_trim, self._tilt_lat_trim)
        # coll_eq_rad is the physics equilibrium collective; eq_thrust is its
        # normalized form. Use coll_eq_rad if present, else convert eq_thrust.
        servo_col = getattr(ic, 'coll_eq_rad', None) or _t2c(ic.eq_thrust)
        self._acro._servo.reset(
            servo_col,
            tilt_lon=self._tilt_lon_trim,
            tilt_lat=self._tilt_lat_trim,
        )

    # ── Convenience constructor for warmup / IC generation runs ───────────────

    @classmethod
    def for_warmup(cls, rotor, pos, R0, rest_length, eq_thrust, omega_spin, wind):
        col_min, col_max = _load_col_range()
        ic = SimpleNamespace(
            pos        = np.asarray(pos, dtype=float),
            vel        = np.zeros(3),
            R0         = R0,
            rest_length= float(rest_length),
            eq_thrust  = float(eq_thrust),
            omega_spin = float(omega_spin),
        )
        return cls(rotor, ic, wind, col_min_rad=col_min, col_max_rad=col_max)

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

        Runs HeliCyclicController (arduloop rate PID + servo model) then physics.
        Use when a Python AP controller produces (collective, rate_roll, rate_pitch).
        """
        tlon, tlat, col_act = self._acro.step(
            collective_rad, rate_roll, rate_pitch, omega_body, dt)
        return self._core.step(dt, col_act, tlon, tlat, rest_length)

    def observe(self) -> HubObservation:
        """Return current observable hub state (sensor boundary enforcement)."""
        return self._core.hub_observe()

    def step_guided(
        self,
        dt: float,
        collective_cmd: float,
        heli_out,
        *,
        rest_length: "float | None" = None,
    ) -> dict:
        """400 Hz step for GUIDED tests.

        Takes a HeliRateOutput from GuidedAttitudeController.update() directly,
        applies the SwashplateServoModel for servo lag, then calls physics.
        Bypasses the rate PIDs in _acro (those are inside GuidedAttitudeController).
        """
        # Sign mapping matches HeliCyclicController.step():
        #   roll_cyclic  ->  tilt_lat  (no sign flip)
        #   pitch_cyclic -> -tilt_lon  (sign flip)
        tilt_lat_cmd =  heli_out.roll_cyclic
        tilt_lon_cmd = -heli_out.pitch_cyclic
        tilt_lat_cmd += self._tilt_lat_trim
        tilt_lon_cmd += self._tilt_lon_trim
        col_act, tlon, tlat = self._acro._servo.step(
            collective_cmd, tilt_lon_cmd, tilt_lat_cmd, dt)
        return self._core.step(dt, col_act, tlon, tlat, rest_length)


from tests.common.mock_ardupilot import MockArdupilot
