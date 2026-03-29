"""
trajectory.py — Pluggable trajectory controllers for the RAWES pumping cycle.

A TrajectoryController is called every physics step.  It receives a rich state
dict containing everything needed to make control decisions (position, velocity,
attitude, spin, tether tension, wind, elapsed time) and a TetherModel it may
mutate directly for winch control.  It returns a command dict used by the caller
to drive the aerodynamic model and attitude controller.

Interface
---------
    cmd = trajectory.step(state, tether, dt)

    state keys:
        pos        np.ndarray [3]  — ENU hub position [m]
        vel        np.ndarray [3]  — ENU hub velocity [m/s]
        R          np.ndarray [3×3]— hub rotation matrix (body frame)
        omega      np.ndarray [3]  — hub angular velocity [rad/s]
        omega_spin float           — rotor spin speed [rad/s]
        tension    float           — current tether tension [N]
        wind       np.ndarray [3]  — wind vector ENU [m/s]
        t_free     float           — free-flight elapsed time [s]

    tether : TetherModel — may be updated in-place (winch: rest_length changed)

    cmd keys (returned):
        collective_rad  float          — blade collective pitch [rad]
        body_z_eq       np.ndarray [3] — attitude setpoint unit vector (ENU)
        phase           str            — label for telemetry ("hold","reel-out","reel-in")
        tension_setpoint float         — current tension setpoint [N] (for telemetry)

Available controllers
---------------------
    HoldTrajectory()
        Orbit-tracks tether direction, commands zero collective, no winch.
        Use for: test_closed_loop.py, test_closed_loop_60s.py

    DeschutterTrajectory(t_reel_out, t_reel_in, t_transition,
                         v_reel_out, v_reel_in, tension_out, tension_in,
                         wind_enu, rest_length_min, xi_reel_in_deg=55.0)
        De Schutter (2018) reel-out / reel-in pumping cycle.
        Reel-out: body_z tracks tether, TensionController drives collective.
        Reel-in:  body_z blends to angle xi_reel_in_deg from wind direction.
        xi_reel_in_deg=55  — constrained from 90° for BEM validity (default).
        xi_reel_in_deg=None — no tilt change; simple tension-only cycle.
        Use for: test_deschutter_cycle.py (xi=55), test_pumping_cycle.py (xi=None)
"""

import math
import numpy as np

from controller import (
    TensionController,
    orbit_tracked_body_z_eq,
    blend_body_z,
)


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class TrajectoryController:
    """Abstract base class — subclass and implement step()."""

    def step(self, state: dict, tether, dt: float) -> dict:
        """
        Advance one physics step.

        Parameters
        ----------
        state  : dict — see module docstring for keys
        tether : TetherModel — updated in-place for winch control
        dt     : float — timestep [s]

        Returns
        -------
        dict with keys: collective_rad, body_z_eq, phase, tension_setpoint
        """
        raise NotImplementedError(
            f"{type(self).__name__} must implement step()")


# ---------------------------------------------------------------------------
# HoldTrajectory
# ---------------------------------------------------------------------------

class HoldTrajectory(TrajectoryController):
    """
    Orbit-track tether direction, zero collective, no winch.

    Captures the tether direction and body_z at the first step and from then
    on rotates the equilibrium body_z azimuthally as the hub orbits the anchor,
    keeping the attitude error at zero throughout the orbit.

    Use for: closed-loop stability tests where the pumping cycle is inactive.
    """

    def __init__(self):
        self._tether_dir0: "np.ndarray | None" = None
        self._body_z_eq0:  "np.ndarray | None" = None

    def step(self, state: dict, tether, dt: float) -> dict:
        pos = state["pos"]

        if self._tether_dir0 is None:
            self._tether_dir0 = pos / np.linalg.norm(pos)
            self._body_z_eq0  = state["R"][:, 2].copy()

        body_z_eq = orbit_tracked_body_z_eq(
            pos, self._tether_dir0, self._body_z_eq0)

        return {
            "collective_rad":  0.0,
            "body_z_eq":       body_z_eq,
            "phase":           "hold",
            "tension_setpoint": 0.0,
        }


# ---------------------------------------------------------------------------
# DeschutterTrajectory
# ---------------------------------------------------------------------------

class DeschutterTrajectory(TrajectoryController):
    """
    De Schutter (2018) reel-out / reel-in pumping cycle.

    Reel-out phase:
        body_z tracks tether direction (orbit-tracked).
        TensionController drives collective to reach tension_out [N].
        Winch pays out tether at v_reel_out [m/s].

    Reel-in phase:
        body_z blends from tether-aligned to angle xi_reel_in_deg from
        horizontal wind direction over t_transition seconds.
        TensionController drives collective to tension_in [N].
        Winch reels in at v_reel_in [m/s], stopping at rest_length_min.

    Parameters
    ----------
    t_reel_out      : float — reel-out phase duration [s]
    t_reel_in       : float — reel-in phase duration [s]
    t_transition    : float — body_z blend window at reel-in start [s]
    v_reel_out      : float — winch pay-out speed [m/s]
    v_reel_in       : float — winch reel-in speed [m/s]
    tension_out     : float — reel-out tension setpoint [N]
    tension_in      : float — reel-in tension setpoint [N]
    wind_enu        : array — wind vector ENU [m/s] (used to compute reel-in orientation)
    rest_length_min : float — minimum tether rest length (reel-in floor) [m]
    xi_reel_in_deg  : float or None
        Angle between body_z and wind during reel-in [degrees].
        55.0 (default) — constrained from 90° to keep BEM valid (v_axial > 0).
        None — no body_z change; body_z stays tether-aligned (simple tension cycle).

    Cycles repeat automatically: after reel-in ends, the next reel-out begins
    and body_z blends back to tether-aligned over t_transition seconds.
    """

    def __init__(
        self,
        t_reel_out:      float,
        t_reel_in:       float,
        t_transition:    float,
        v_reel_out:      float,
        v_reel_in:       float,
        tension_out:     float,
        tension_in:      float,
        wind_enu:        np.ndarray,
        rest_length_min: float = 10.0,
        xi_reel_in_deg:  "float | None" = 55.0,
    ):
        self._t_reel_out      = float(t_reel_out)
        self._t_reel_in       = float(t_reel_in)
        self._t_transition    = float(max(t_transition, 1e-6))
        self._v_reel_out      = float(v_reel_out)
        self._v_reel_in       = float(v_reel_in)
        self._tension_out     = float(tension_out)
        self._tension_in      = float(tension_in)
        self._rest_length_min = float(rest_length_min)
        self._t_cycle         = self._t_reel_out + self._t_reel_in

        self._ctrl_out = TensionController(setpoint_n=tension_out)
        self._ctrl_in  = TensionController(setpoint_n=tension_in)

        # Reel-in body_z target: in the vertical plane containing wind_enu,
        # at xi_reel_in_deg from the horizontal wind direction.
        # None means "stay tether-aligned" (no tilt change during reel-in).
        if xi_reel_in_deg is not None:
            xi_rad      = math.radians(float(xi_reel_in_deg))
            wind_horiz  = np.array([float(wind_enu[0]), float(wind_enu[1]), 0.0])
            wh_norm     = np.linalg.norm(wind_horiz)
            wind_horiz  = wind_horiz / wh_norm if wh_norm > 1e-6 else np.array([1.0, 0.0, 0.0])
            self._body_z_reel_in: "np.ndarray | None" = (
                math.cos(xi_rad) * wind_horiz
                + math.sin(xi_rad) * np.array([0.0, 0.0, 1.0])
            )
        else:
            self._body_z_reel_in = None   # use tether-aligned (no tilt change)

        # Orbit-tracking initial conditions — captured at first step
        self._tether_dir0: "np.ndarray | None" = None
        self._body_z_eq0:  "np.ndarray | None" = None

    # ------------------------------------------------------------------
    def step(self, state: dict, tether, dt: float) -> dict:
        pos     = state["pos"]
        t_free  = float(state["t_free"])
        tension = float(state["tension"])

        # Capture orbit-tracking IC on the very first call
        if self._tether_dir0 is None:
            self._tether_dir0 = pos / np.linalg.norm(pos)
            self._body_z_eq0  = state["R"][:, 2].copy()

        # Phase logic
        t_cyc     = t_free % self._t_cycle
        cyc       = int(t_free / self._t_cycle)
        phase_out = t_cyc < self._t_reel_out

        # Winch
        if phase_out:
            tether.rest_length += self._v_reel_out * dt
        else:
            tether.rest_length = max(
                self._rest_length_min,
                tether.rest_length - self._v_reel_in * dt,
            )

        # Orbit-tracked tether-aligned equilibrium
        bz_tether = orbit_tracked_body_z_eq(
            pos, self._tether_dir0, self._body_z_eq0)

        # Body_z_eq setpoint
        if self._body_z_reel_in is None:
            # Simple tension cycle: always tether-aligned, no tilt
            body_z_eq = bz_tether
        elif phase_out:
            if cyc > 0 and t_cyc < self._t_transition:
                # Blend back from reel-in orientation to tether-aligned
                alpha     = 1.0 - t_cyc / self._t_transition
                body_z_eq = blend_body_z(alpha, bz_tether, self._body_z_reel_in)
            else:
                body_z_eq = bz_tether
        else:
            # Reel-in: blend tether-aligned → reel-in orientation
            t_since   = t_cyc - self._t_reel_out
            alpha     = min(1.0, t_since / self._t_transition)
            body_z_eq = blend_body_z(alpha, bz_tether, self._body_z_reel_in)

        # Collective from TensionController
        ctrl       = self._ctrl_out if phase_out else self._ctrl_in
        collective = ctrl.update(tension, dt)

        return {
            "collective_rad":   collective,
            "body_z_eq":        body_z_eq,
            "phase":            "reel-out" if phase_out else "reel-in",
            "tension_setpoint": self._tension_out if phase_out else self._tension_in,
        }
