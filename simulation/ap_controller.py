"""
ap_controller.py -- AP-side pumping controller.

Runs on the ArduPilot side (400 Hz in Python sim, ~50 Hz in Lua on hardware).

Communication boundary:
    Ground -> AP: ThrustCommand at ~10 Hz via MAVLink NAMED_VALUE_FLOAT.
    AP has no direct tension measurement.

Architecture:
    Ground (10 Hz): computes thrust [0..1] from phase schedule
                    thrust=0 -> minimum tension (tether-tilt, coll_min)
                    thrust=1 -> maximum tension (alt-hold, coll_max)
                    Reel-out: ramps 0 -> target_thrust slowly (safe build-up)
                    Transition/reel-in: thrust=0 (tension drops)
                    -> sends ThrustCommand to AP

    AP (400 Hz): maps thrust to body_z and collective
                    body_z = slerp(tether_dir, alt_hold, thrust)
                    coll   = coll_min + thrust * (coll_max - coll_min)
                 Both are slewed at hardware-limited rates to avoid jerky
                 movements and give the winch time to react.

    Winch (400 Hz, ground): waits for tension < T_reel_in_start before
                             reeling in; self-contained safety layer.

Comms dropout:
    On timeout, thrust falls to 0 -- AP drives to minimum tension / tether-tilt.
"""

from __future__ import annotations

import math

import numpy as np

from controller import AltitudeHoldController, slerp_body_z
from pumping_planner import ThrustCommand


def _slerp_frac(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    """Spherical interpolation from unit vector a to b by fraction t in [0,1]."""
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if abs(dot) > 0.9999:
        r = a + t * (b - a)
        n = np.linalg.norm(r)
        return r / n if n > 1e-9 else b
    theta = math.acos(dot)
    s = math.sin(theta)
    return (math.sin((1.0 - t) * theta) / s * a
            + math.sin(t * theta)        / s * b)


class ThrustApController:
    """
    AP-side pumping controller driven by ThrustCommand (0..1).

    thrust=0 -> tether-tilt body_z (minimum tension), collective at coll_min
    thrust=1 -> altitude-hold body_z (maximum tension), collective at coll_max

    Both body_z and collective are slewed at hardware-limited rates.
    On comms dropout, thrust falls to 0 (safe minimum-tension mode).

    Parameters
    ----------
    ic_pos          : np.ndarray (3,)  initial hub NED position [m]
    mass_kg         : float            rotor mass [kg]
    slew_rate_rad_s : float            body_z slew rate [rad/s]
    warm_coll_rad   : float            collective at startup (warm start) [rad]
    coll_min        : float            minimum collective [rad]
    coll_max        : float            maximum collective [rad]
    coll_slew_rate  : float            collective slew rate [rad/s]
    cmd_timeout_s   : float            comms dropout timeout [s]
    T_tension_est   : float            tension estimate for gravity compensation [N]
    """

    CMD_TIMEOUT_S: float = 0.5

    def __init__(
        self,
        ic_pos         : np.ndarray,
        mass_kg        : float,
        slew_rate_rad_s: float,
        warm_coll_rad  : float,
        coll_min       : float = -0.28,
        coll_max       : float =  0.10,
        coll_slew_rate : float =  0.15,
        cmd_timeout_s  : float = CMD_TIMEOUT_S,
        T_tension_est  : float = 300.0,
    ) -> None:
        self._coll_min    = float(coll_min)
        self._coll_max    = float(coll_max)
        self._coll_slew   = float(coll_slew_rate)
        self._bz_slew     = float(slew_rate_rad_s)
        self._timeout     = float(cmd_timeout_s)
        self._T_est       = float(T_tension_est)
        self._mass_kg     = float(mass_kg)

        self._alt_ctrl    = AltitudeHoldController.from_pos(ic_pos, slew_rate_rad_s)
        self._target_alt  = float(-ic_pos[2])

        self._thrust      = 0.0
        self._current_coll = float(warm_coll_rad)
        self._current_bz  : np.ndarray | None = None

        self._cmd_age     = 0.0
        self._comms_ok    = True

    # ── command reception (10 Hz from ground) ──────────────────────────────

    def receive_command(self, cmd: ThrustCommand) -> None:
        """Update thrust target from an incoming ThrustCommand (~10 Hz)."""
        self._thrust     = float(np.clip(cmd.thrust, 0.0, 1.0))
        self._target_alt = float(cmd.alt_m)
        self._cmd_age    = 0.0
        self._comms_ok   = True

    # ── 400 Hz step ────────────────────────────────────────────────────────

    def step(
        self,
        pos_ned : np.ndarray,
        body_z  : np.ndarray,
        dt      : float,
    ) -> tuple[float, np.ndarray]:
        """
        400 Hz step.  Returns (collective_rad, body_z_eq).

        body_z = slerp(tether_dir, alt_hold, thrust)
        coll   = coll_min + thrust * (coll_max - coll_min),  slewed
        """
        self._cmd_age += dt
        if self._comms_ok and self._cmd_age > self._timeout:
            self._comms_ok = False

        thrust = self._thrust if self._comms_ok else 0.0

        # ── body_z ──────────────────────────────────────────────────────────
        bz_alt = self._alt_ctrl.update(
            pos_ned, self._target_alt, self._T_est, self._mass_kg, dt
        )
        pos = np.asarray(pos_ned, dtype=float)
        pn  = float(np.linalg.norm(pos))
        tether_dir = -pos / pn if pn > 0.1 else np.array([0.0, 0.0, -1.0])
        bz_target  = _slerp_frac(tether_dir, bz_alt, thrust)

        if self._current_bz is None:
            self._current_bz = bz_alt.copy()

        self._current_bz = slerp_body_z(
            self._current_bz, bz_target, self._bz_slew, dt
        )

        # ── collective ──────────────────────────────────────────────────────
        target_coll = self._coll_min + thrust * (self._coll_max - self._coll_min)
        max_d = self._coll_slew * dt
        self._current_coll = float(np.clip(
            target_coll,
            self._current_coll - max_d,
            self._current_coll + max_d,
        ))
        self._current_coll = float(np.clip(
            self._current_coll, self._coll_min, self._coll_max
        ))

        return self._current_coll, self._current_bz

    # ── diagnostics ────────────────────────────────────────────────────────

    @property
    def comms_ok(self) -> bool:
        return self._comms_ok

    @property
    def thrust(self) -> float:
        """Current thrust target [0..1] (for telemetry)."""
        return self._thrust if self._comms_ok else 0.0

    @property
    def drive(self) -> float:
        """Collective as fraction of [coll_min, coll_max] (for telemetry)."""
        span = self._coll_max - self._coll_min
        return (self._current_coll - self._coll_min) / span if span > 1e-9 else 0.5

    @property
    def raw_coll(self) -> float:
        """Current collective [rad] (for telemetry)."""
        return self._current_coll

    @property
    def tilt_frac(self) -> float:
        """Tilt fraction: 1 - thrust (0=alt-hold, 1=full tether tilt)."""
        return 1.0 - (self._thrust if self._comms_ok else 0.0)

    @property
    def _target_alt(self) -> float:
        return self.__target_alt

    @_target_alt.setter
    def _target_alt(self, v: float) -> None:
        self.__target_alt = float(v)
