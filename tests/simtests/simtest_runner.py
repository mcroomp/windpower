"""
simtest_runner.py — Shared 400 Hz physics core for simtests.

PhysicsRunner is a thin wrapper around PhysicsCore (simulation/physics_core.py).

    step_from_thrust(dt, thrust, rate_roll, rate_pitch, omega_body)
        → runs HeliCyclicController.step_from_thrust(), then core.step()

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

In GUIDED mode rawes.lua drives cyclic/collective via
vehicle:set_target_angle_and_rate_and_throttle and
vehicle:set_target_rate_and_throttle. The Lua backend feeds those targets into
GuidedAttitudeController (400 Hz inner loop) and calls runner.step_guided() so
physics sees the correct swashplate tilt.
Collective is sourced from the guided vertical channel when available,
with ch3 kept as a legacy fallback.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np


from simulation.physics_core import PhysicsCore, HubObservation
from simulation.controller import HeliCyclicController
from simulation.param_defaults import load_collective_phys_range as _load_col_range


class PhysicsRunner:
    """
    Simtest wrapper around PhysicsCore.

    Exposes the same API used by all existing simtests while delegating all
    physics integration to PhysicsCore (shared with the mediator).

    Usage
    -----
    runner = PhysicsRunner(rotor, ic, wind)
    for i in range(steps):
        sr = runner.step_from_thrust(DT, thrust_cmd, rate_roll, rate_pitch, omega_body, rest_length=winch.rest_length)
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
        self._col_min, self._col_max = _load_col_range()
        self._tilt_lon_trim = float(getattr(ic, "trim_tilt_lon", 0.0))
        self._tilt_lat_trim = float(getattr(ic, "trim_tilt_lat", 0.0))
        self._acro = HeliCyclicController(rotor)
        self._acro.set_trim(self._tilt_lon_trim, self._tilt_lat_trim)
        self._acro.reset_from_thrust(
            float(ic.eq_thrust),
            tilt_lon=self._tilt_lon_trim,
            tilt_lat=self._tilt_lat_trim,
        )

    # ── Convenience constructor for warmup / IC generation runs ───────────────

    @classmethod
    def for_warmup(cls, rotor, pos, R0, rest_length, eq_thrust, omega_spin, wind):
        ic = SimpleNamespace(
            pos        = np.asarray(pos, dtype=float),
            vel        = np.zeros(3),
            R0         = R0,
            rest_length= float(rest_length),
            eq_thrust  = float(eq_thrust),
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

    def step_from_thrust(self, dt: float, thrust_cmd: float,
                         rate_roll: float, rate_pitch: float,
                         omega_body: np.ndarray,
                         *, rest_length: "float | None" = None) -> dict:
        """400 Hz step for Python-AP tests using thrust [0..1].

        Does not drive the yaw-motor hub ODE (yaw_throttle stays None -> pure
        damp-to-zero): HeliCyclicController's yaw rate PID has no anti-rotation
        trim observer behind it (unlike the GUIDED path -- see
        mock_ardupilot._MockArdupilotBase.step_physics()), and this API is
        shared by many simtests that never intended to exercise yaw-motor
        coupling (e.g. static force-balance checks). Real yaw authority is
        exercised via step_guided() instead.
        """
        tlon, tlat, col_act = self._acro.step_from_thrust(
            thrust_cmd, rate_roll, rate_pitch, omega_body, dt)
        return self._core.step(dt, col_act, tlon, tlat, rest_length)

    def observe(self) -> HubObservation:
        """Return current observable hub state (sensor boundary enforcement)."""
        return self._core.hub_observe()

    def step_guided(
        self,
        dt: float,
        thrust_cmd: float,
        heli_out,
        *,
        rest_length: "float | None" = None,
        yaw_throttle: "float | None" = None,
    ) -> dict:
        """400 Hz step for GUIDED tests.

        Takes a HeliRateOutput from GuidedAttitudeController.update() directly,
        converts thrust [0..1] to collective radians at the boundary,
        applies the SwashplateServoModel for servo lag, then calls physics.
        Bypasses the rate PIDs in _acro (those are inside GuidedAttitudeController).

        yaw_throttle : caller-supplied Motor4 throttle override [0..1] (e.g.
            after adding the anti-rotation trim -- see
            mock_ardupilot._MockArdupilotBase.step_physics()). Defaults to
            heli_out.yaw_cmd clamped to [0, 1] when not supplied.
        """
        # Sign mapping matches HeliCyclicController._step_collective():
        #   roll_cyclic  ->  tilt_lat  (no sign flip)
        #   pitch_cyclic -> -tilt_lon  (sign flip)
        tilt_lat_cmd =  heli_out.roll_cyclic
        tilt_lon_cmd = -heli_out.pitch_cyclic
        tilt_lat_cmd += self._tilt_lat_trim
        tilt_lon_cmd += self._tilt_lon_trim
        thrust = float(np.clip(thrust_cmd, 0.0, 1.0))
        collective_cmd = self._col_min + thrust * (self._col_max - self._col_min)
        col_act, tlon, tlat = self._acro._servo.step(
            collective_cmd, tilt_lon_cmd, tilt_lat_cmd, dt)
        if yaw_throttle is None:
            yaw_throttle = float(np.clip(heli_out.yaw_cmd, 0.0, 1.0))
        return self._core.step(dt, col_act, tlon, tlat, rest_length,
                                yaw_throttle=float(yaw_throttle))


from tests.common.mock_ardupilot import MockArdupilot
