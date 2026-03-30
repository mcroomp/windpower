"""
trajectory.py — Pluggable trajectory controllers for the RAWES pumping cycle.

Models the offboard MAVLink trajectory planner that runs on a ground station
or companion computer and sends commands to the custom ArduPilot Mode_RAWES.

Protocol
--------
Two packet types cross the MAVLink boundary:

    STATE packet  (Pixhawk → planner, standard ArduPilot streams):
        "pos_enu"    np.ndarray [3]  — hub position ENU [m]        (LOCAL_POSITION_NED converted)
        "vel_enu"    np.ndarray [3]  — hub velocity ENU [m/s]      (LOCAL_POSITION_NED converted)
        "omega_spin" float           — rotor spin rate [rad/s]      (ESC_STATUS[RAWES_CTR_ESC].rpm
                                                                      × 2π/60 / 11 × 44/80)

    Planner-local quantities (NOT from Pixhawk — read from WinchController local link):
        tension_n       float  — tether tension [N]       — winch load cell
        tether_length_m float  — tether rest length [m]   — winch encoder
        t_free          float  — elapsed free-flight [s]  — planner internal clock

    COMMAND packet  (planner → Pixhawk, ~10 Hz):
        "attitude_q"      np.ndarray [4]  (w,x,y,z)                (SET_ATTITUDE_TARGET quaternion)
            Desired disk orientation in ENU.
            [1,0,0,0] (identity) = tether-aligned natural orbit — Mode_RAWES tracks
            the tether direction at 400 Hz without any planner correction.
            Non-identity = desired body_z is quat_apply(attitude_q, [0,0,1]).
            Mode_RAWES rate-limits the slew internally (body_z_slew_rate_rad_s).
        "thrust"          float [0..1]                              (SET_ATTITUDE_TARGET thrust)
            Normalised collective, direct output of the tension PI:
              collective_rad = kP * err + kI * integral
              thrust = clamp((collective_rad - col_min) / (col_max - col_min), 0, 1)
            Mode_RAWES calls set_throttle_out(thrust) — direct passthrough, no conversion.
            Simulation mediator denormalises: collective_rad = col_min + thrust*(col_max-col_min).
        "phase"           str — telemetry label ("hold"|"reel-out"|"reel-in"). Not on wire.

    Winch command  (planner → WinchController, ground-local link):
        "winch_speed_ms"  float [m/s]                               (MAV_CMD_DO_WINCH RATE_CONTROL)
            +ve = pay out, −ve = reel in, 0 = hold.
            Passed to WinchController.step() — the Pixhawk is not involved.

Mode_RAWES responsibilities (NOT in this file):
    - Orbit tracking: compute tether-aligned body_z_eq at 400 Hz from current pos
    - Slew body_z_eq toward attitude_q target at body_z_slew_rate_rad_s
    - Attitude error → cyclic tilt  (compute_swashplate_from_state)
    - Collective: set_throttle_out(thrust) — direct passthrough, no PI on Pixhawk
    - Counter-torque motor (inner loop, not commanded by planner)

WinchController responsibilities (NOT in this file — see winch.py):
    - Execute winch_speed_ms commands
    - Enforce tension safety limit (stop paying out if tension too high)
    - Provide tension_n and tether_length_m to the planner

Available controllers
---------------------
    HoldPlanner()
        Returns natural equilibrium command every step.
        attitude_q = identity → Mode_RAWES stays tether-aligned through orbit tracking.
        Use for: test_closed_loop_60s.py

    DeschutterPlanner(t_reel_out, t_reel_in, t_transition,
                         v_reel_out, v_reel_in, tension_out, tension_in,
                         wind_enu, xi_reel_in_deg=55.0,
                         tension_kp=5e-4, tension_ki=1e-4,
                         col_min_rad=-0.436, col_max_rad=0.0,
                         tension_max_n=None, wind_estimator=None)
        De Schutter (2018) pumping cycle.
        Owns the tension PI internally; outputs normalised thrust [0..1].
        Reel-out: identity attitude_q (natural tether-aligned orbit).
        Reel-in:  reel-in attitude_q at xi_reel_in_deg from estimated (or fixed) wind direction.
        If wind_estimator is provided, the reel-in quaternion updates each step as the
        wind direction estimate converges.  Falls back to fixed wind_enu until ready.
        xi_reel_in_deg=55  — constrained from 90° for BEM validity (default).
        xi_reel_in_deg=None — no tilt change; identity attitude_q all cycle.
        Use for: test_deschutter_cycle.py (xi=55), test_pumping_cycle.py (xi=None)

Wind estimation
---------------
    WindEstimator(window_s=60.0, K_drive=1.4, K_drag=0.01786)
        Estimates wind direction and in-plane speed from STATE packets.

        wind_dir_enu  — unit vector in wind direction (ENU, horizontal).
            Derived from the rolling mean horizontal hub position.  Over one
            orbit the hub's mean horizontal position points downwind from the
            anchor.  Requires no extra sensors beyond pos_enu.

        v_inplane_ms  — in-plane wind speed component [m/s].
            Derived from rotor spin rate via the autorotation torque balance:
            v_inplane = omega_spin² × K_drag / K_drive.
            This is the wind component perpendicular to the rotor axle.
            Requires omega_spin in the STATE packet.
"""

import math
import numpy as np
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller import TensionPI


# ---------------------------------------------------------------------------
# Quaternion utilities (module-level, importable)
# ---------------------------------------------------------------------------

#: Quaternion identity [w, x, y, z] — represents no rotation.
Q_IDENTITY: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])


def quat_from_vectors(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
    """
    Quaternion [w, x, y, z] that rotates unit vector *v1* to unit vector *v2*.

    The quaternion satisfies:  quat_apply(q, v1) ≈ v2.
    Both vectors are normalised internally.
    """
    v1 = np.asarray(v1, dtype=float)
    v2 = np.asarray(v2, dtype=float)
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    c = float(np.dot(v1, v2))
    if c >= 1.0 - 1e-9:
        return Q_IDENTITY.copy()
    if c <= -1.0 + 1e-9:
        # Anti-parallel: 180° rotation around any perpendicular axis
        perp = np.array([1.0, 0.0, 0.0]) if abs(v1[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(v1, perp)
        axis = axis / np.linalg.norm(axis)
        return np.array([0.0, axis[0], axis[1], axis[2]])
    axis = np.cross(v1, v2)
    axis = axis / np.linalg.norm(axis)
    angle = math.acos(c)
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), s * axis[0], s * axis[1], s * axis[2]])


def quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Rotate vector *v* by quaternion *q* = [w, x, y, z].

    Equivalent to:  q ⊗ [0, v] ⊗ q*
    """
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    v = np.asarray(v, dtype=float)
    xyz = np.array([x, y, z])
    t = 2.0 * np.cross(xyz, v)
    return v + w * t + np.cross(xyz, t)


def quat_is_identity(q: np.ndarray, atol: float = 1e-6) -> bool:
    """Return True if *q* is the identity quaternion within *atol*."""
    return abs(float(q[0]) - 1.0) < atol and float(np.linalg.norm(q[1:])) < atol


# ---------------------------------------------------------------------------
# WindEstimator
# ---------------------------------------------------------------------------

class WindEstimator:
    """
    Estimates wind direction and in-plane speed from STATE packets.

    Uses two methods that require no hardware beyond what is already present:

    **Direction — orbital mean position (Method 2):**
    Over one full orbit the hub's mean horizontal position points downwind from
    the anchor.  This is because the rotor lives on the downwind side of the
    anchor at all times; the orbit is symmetric about the wind direction, so
    the mean horizontal position is the mean tether azimuth, which equals the
    wind direction.  Only pos_enu is needed.

    **In-plane speed — rotor spin rate (Method 1):**
    The rotor is in autorotation equilibrium when aerodynamic drive torque equals
    drag torque:
        Q_drive(v_inplane) = Q_drag(omega_spin)
        → v_inplane = omega_spin² × K_drag / K_drive
    v_inplane is the wind component perpendicular to the rotor axle.  To convert
    to full wind speed: v_wind ≈ v_inplane / sin(xi), where xi is the disk tilt
    from wind (available from body_z and wind_dir_enu).
    Requires omega_spin in the STATE packet.

    Parameters
    ----------
    window_s    : float — rolling average window [s].  Should be ≥ one orbit
                  period (~60 s at default equilibrium).  Default: 60.0.
    min_samples : int   — minimum samples required before returning estimates.
    K_drive     : float — autorotation drive constant [N·m·s/m].
                  Default matches beaupoil_2026 rotor (K_DRIVE_SPIN in mediator).
    K_drag      : float — autorotation drag constant [N·m·s²/rad²].
                  Default matches beaupoil_2026 rotor (K_DRAG_SPIN in mediator).
    """

    def __init__(
        self,
        window_s:    float = 60.0,
        min_samples: int   = 20,
        K_drive:     float = 1.4,
        K_drag:      float = 0.01786,
    ):
        self._window_s    = float(window_s)
        self._min_samples = int(min_samples)
        self._K_drive     = float(K_drive)
        self._K_drag      = float(K_drag)
        self._t           = 0.0   # internal monotonic clock [s]
        # Buffer entries: (t, pos_enu [3], omega_spin)
        self._buf: "list[tuple[float, np.ndarray, float]]" = []

    def update(self, state: dict, dt: float = 1.0) -> None:
        """
        Ingest one STATE packet.  Call every planner step.

        dt advances the internal clock used for window eviction.
        """
        self._t += float(dt)
        pos        = np.asarray(state["pos_enu"], dtype=float).copy()
        omega_spin = float(state.get("omega_spin", 0.0))
        self._buf.append((self._t, pos, omega_spin))
        # Trim entries older than window
        cutoff = self._t - self._window_s
        self._buf = [(ti, pi, oi) for ti, pi, oi in self._buf if ti >= cutoff]

    @property
    def ready(self) -> bool:
        """True once enough samples have accumulated to return estimates."""
        return len(self._buf) >= self._min_samples

    @property
    def wind_dir_enu(self) -> "np.ndarray | None":
        """
        Estimated wind direction as a horizontal ENU unit vector, or None if
        not yet ready.

        Computed as the normalised mean horizontal hub position.  A minimum
        magnitude of 1.0 m is required; returns None if the hub appears to be
        at the anchor (degenerate case).
        """
        if not self.ready:
            return None
        positions  = np.array([p for _, p, _ in self._buf])
        mean_pos   = np.mean(positions, axis=0)
        mean_horiz = np.array([mean_pos[0], mean_pos[1], 0.0])
        norm       = float(np.linalg.norm(mean_horiz))
        if norm < 1.0:
            return None
        return mean_horiz / norm

    @property
    def v_inplane_ms(self) -> "float | None":
        """
        Estimated in-plane wind speed [m/s] from rotor spin rate, or None if
        omega_spin data is unavailable or estimator is not yet ready.

        v_inplane is the wind component perpendicular to the rotor axle.
        Convert to full wind speed: v_wind ≈ v_inplane / sin(xi).
        Returns None if no valid omega_spin samples are in the buffer.
        """
        if not self.ready:
            return None
        spins = [o for _, _, o in self._buf if o > 1.0]
        if not spins:
            return None
        omega_mean = float(np.mean(spins))
        return omega_mean ** 2 * self._K_drag / self._K_drive


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class TrajectoryPlanner:
    """Abstract base class — subclass and implement step()."""

    def step(self, state: dict, dt: float,
             tension_n: float = 0.0,
             tether_length_m: "float | None" = None) -> dict:
        """
        Advance one planner step.

        Parameters
        ----------
        state           : dict  — STATE packet (pos_enu, vel_enu, omega_spin)
        dt              : float — timestep [s]
        tension_n       : float — tether tension [N] from WinchController load cell
        tether_length_m : float | None — tether rest length [m] from WinchController encoder

        Returns
        -------
        COMMAND packet dict with keys:
            attitude_q, thrust, winch_speed_ms, phase
        """
        raise NotImplementedError(
            f"{type(self).__name__} must implement step()")


# ---------------------------------------------------------------------------
# HoldPlanner
# ---------------------------------------------------------------------------

class HoldPlanner(TrajectoryPlanner):
    """
    Natural equilibrium — no correction, no winch.

    Sends identity attitude_q and zero thrust every step.  Mode_RAWES stays
    tether-aligned through its own orbit-tracking (no planner involvement).

    Use for: closed-loop stability tests where the pumping cycle is inactive.
    """

    def step(self, state: dict, dt: float,
             tension_n: float = 0.0,
             tether_length_m: "float | None" = None) -> dict:
        return {
            "attitude_q":     Q_IDENTITY.copy(),
            "thrust":         0.0,
            "winch_speed_ms": 0.0,
            "phase":          "hold",
        }


# ---------------------------------------------------------------------------
# DeschutterPlanner
# ---------------------------------------------------------------------------

class DeschutterPlanner(TrajectoryPlanner):
    """
    De Schutter (2018) reel-out / reel-in pumping cycle.

    The planner manages only mission-level decisions: phase timing, tension
    setpoints, winch speed, and the reel-in tilt target.  Orbit tracking and
    body_z slewing run entirely inside Mode_RAWES — the planner only sends the
    target attitude quaternion and thrust setpoint.

    If *wind_estimator* is provided, the reel-in quaternion is recomputed each
    step from the live wind direction estimate as it converges.  Until the
    estimator is ready, the fixed *wind_enu* constructor argument is used.

    Reel-out phase:
        attitude_q = identity  →  Mode_RAWES stays tether-aligned naturally.
        thrust = tension_out / tension_max_n.
        winch_speed_ms = +v_reel_out.

    Reel-in phase:
        attitude_q = reel-in quaternion (body_z at xi_reel_in_deg from wind).
        thrust = tension_in / tension_max_n.
        winch_speed_ms = −v_reel_in.

    Parameters
    ----------
    t_reel_out      : float — reel-out phase duration [s]
    t_reel_in       : float — reel-in phase duration [s]
    t_transition    : float — stored for reference; slewing handled by Mode_RAWES
    v_reel_out      : float — winch pay-out speed [m/s]
    v_reel_in       : float — winch reel-in speed [m/s]
    tension_out     : float — reel-out tension setpoint [N]
    tension_in      : float — reel-in tension setpoint [N]
    wind_enu        : array — initial/fallback wind vector ENU [m/s]
    rest_length_min : float — informational floor; enforced by Mode_RAWES [m]
    xi_reel_in_deg  : float or None
        Angle between body_z and wind during reel-in [degrees].
        55.0 (default) — constrained from 90° for BEM validity.
        None — no tilt change; identity attitude_q all cycle.
    tension_max_n   : float or None
        Max tension for thrust normalisation.  None → use tension_out.
    wind_estimator  : WindEstimator or None
        If provided, the reel-in quaternion tracks the live wind estimate.
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
        xi_reel_in_deg:  "float | None" = 55.0,
        tension_kp:      float = 5e-4,
        tension_ki:      float = 1e-4,
        col_min_rad:     float = TensionPI.COLL_MIN_RAD,
        col_max_rad:     float = TensionPI.COLL_MAX_RAD,
        wind_estimator:  "WindEstimator | None" = None,
    ):
        self._t_reel_out    = float(t_reel_out)
        self._t_reel_in     = float(t_reel_in)
        self._t_transition  = float(max(t_transition, 1e-6))
        self._v_reel_out    = float(v_reel_out)
        self._v_reel_in     = float(v_reel_in)
        self._tension_out   = float(tension_out)
        self._tension_in    = float(tension_in)
        self._t_cycle       = self._t_reel_out + self._t_reel_in
        self._col_min_rad   = float(col_min_rad)
        self._col_max_rad   = float(col_max_rad)
        self._xi_reel_in_deg: "float | None" = float(xi_reel_in_deg) if xi_reel_in_deg is not None else None
        self._wind_estimator = wind_estimator
        self._t_free         = 0.0   # internal elapsed free-flight time [s]

        # Tension PI — owned by the planner (raws_mode.md §3.2)
        # Single controller whose setpoint changes at the phase boundary so the
        # integral state carries across smoothly (no warm-start discontinuity).
        self._tension_ctrl = TensionPI(
            setpoint_n=float(tension_out),
            kp=float(tension_kp),
            ki=float(tension_ki),
            coll_min=float(col_min_rad),
            coll_max=float(col_max_rad),
        )

        # Initial/fallback reel-in attitude from fixed wind_enu
        self._attitude_q_reel_in: "np.ndarray | None" = self._q_from_wind(
            np.asarray(wind_enu, dtype=float))

    # ------------------------------------------------------------------
    def _q_from_wind(self, wind_enu: np.ndarray) -> "np.ndarray | None":
        """Compute reel-in quaternion from a wind direction vector."""
        if self._xi_reel_in_deg is None:
            return None
        xi_rad     = math.radians(self._xi_reel_in_deg)
        wind_horiz = np.array([float(wind_enu[0]), float(wind_enu[1]), 0.0])
        wh_norm    = np.linalg.norm(wind_horiz)
        if wh_norm < 1e-6:
            return None
        wind_horiz = wind_horiz / wh_norm
        body_z_target = (
            math.cos(xi_rad) * wind_horiz
            + math.sin(xi_rad) * np.array([0.0, 0.0, 1.0])
        )
        return quat_from_vectors(np.array([0.0, 0.0, 1.0]), body_z_target)

    # ------------------------------------------------------------------
    def step(self, state: dict, dt: float,
             tension_n: float = 0.0,
             tether_length_m: "float | None" = None) -> dict:
        self._t_free += dt

        # Update wind estimator and refresh reel-in quaternion if direction changed
        if self._wind_estimator is not None:
            self._wind_estimator.update(state, dt)
            wind_dir = self._wind_estimator.wind_dir_enu
            if wind_dir is not None:
                v_inplane = self._wind_estimator.v_inplane_ms
                wind_vec  = wind_dir * v_inplane if (v_inplane is not None and v_inplane > 0.5) else wind_dir
                q_new = self._q_from_wind(wind_vec)
                if q_new is not None:
                    self._attitude_q_reel_in = q_new

        t_cyc     = self._t_free % self._t_cycle
        phase_out = t_cyc < self._t_reel_out

        # Tension PI (planner-owned) — updates setpoint at phase boundary
        tension_setpoint = self._tension_out if phase_out else self._tension_in
        self._tension_ctrl.setpoint = tension_setpoint
        collective_rad = self._tension_ctrl.update(float(tension_n), dt)

        # Normalise collective → thrust [0..1] for SET_ATTITUDE_TARGET
        col_range = self._col_max_rad - self._col_min_rad
        thrust = float(max(0.0, min(1.0,
            (collective_rad - self._col_min_rad) / col_range if col_range > 1e-9 else 0.5
        )))

        # Winch speed — stop reel-out if tether has reached max length (if provided)
        winch_speed = self._v_reel_out if phase_out else -self._v_reel_in

        if phase_out and self._attitude_q_reel_in is None:
            attitude_q = Q_IDENTITY.copy()
        elif phase_out:
            attitude_q = Q_IDENTITY.copy()
        else:
            attitude_q = self._attitude_q_reel_in.copy() if self._attitude_q_reel_in is not None else Q_IDENTITY.copy()

        return {
            "attitude_q":     attitude_q,
            "thrust":         thrust,
            "winch_speed_ms": winch_speed,
            "phase":          "reel-out" if phase_out else "reel-in",
        }
