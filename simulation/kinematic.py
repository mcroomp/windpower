"""
kinematic.py — Kinematic startup phase for RAWES simulation.

During the first ``duration`` seconds of simulation the hub position, velocity,
and orientation are overridden directly (kinematic control) rather than computed
from physics.  This gives the EKF time to initialise on GPS before free flight.

Default (linear+ramp) trajectory
---------------------------------
    t in [0, t_ramp_start)        : vel = target_vel  (constant)
    t in [t_ramp_start, duration] : vel = target_vel * (duration - t) / ramp_s
                                     (linear taper to zero)

Position is the analytic integral of the velocity profile, adjusted so the hub
arrives exactly at ``target_pos`` with ``target_vel`` at t = ``duration``.

Minimum-jerk trajectory
-----------------------
Pass ``traj_fn = make_min_jerk_traj(p0, v0, pf, vf, T)`` to fly a 5th-order
polynomial from ``(p0, v0)`` to ``(pf, vf)`` in ``T`` seconds with zero initial
and final acceleration.  After ``T`` the callable returns ``(pf, vf)`` (clamped).

Set ``KinematicStartup.duration > T`` to hold at ``(pf, vf)`` for a margin
before free flight begins — useful to ensure GPS fuses before hand-off.

Custom trajectories
-------------------
Pass ``traj_fn(t_sim) -> (pos, vel)`` to fly any path during kinematic startup.
Optionally pass ``R_fn(t_sim) -> R`` to vary the locked orientation over time
(e.g. so body_z tracks the tether direction on a curved trajectory).

Use ``make_linear_traj()`` or ``make_min_jerk_traj()`` to build callables.

Shared between mediator.py (production loop) and unit/simtests (no Docker).
No imports beyond numpy — zero Docker / SITL dependencies.
"""
from __future__ import annotations

import numpy as np
from typing import Any, Callable, Tuple


# ---------------------------------------------------------------------------
# Standalone helper (backward-compatible with mediator.compute_launch_position)
# ---------------------------------------------------------------------------

def compute_launch_position(
    target_pos,
    target_vel,
    damp_seconds: float,
    ramp_s: float = 0.0,
) -> np.ndarray:
    """
    Compute the hub starting position for a kinematic startup trajectory.

    The hub travels at constant ``target_vel`` from ``launch_pos``, arriving at
    ``target_pos`` at t = ``damp_seconds``.  If ``ramp_s > 0`` the velocity is
    linearly tapered to zero over the last ``ramp_s`` seconds so the hub enters
    free flight at rest; ``launch_pos`` is adjusted so the hub still reaches
    ``target_pos`` at t = ``damp_seconds`` despite the reduced average velocity.

    Velocity profile:
        t in [0,  T - ramp_s]:  v(t) = target_vel        (constant)
        t in [T - ramp_s, T]:   v(t) = target_vel * (T - t) / ramp_s  (linear → 0)

    Position at t = T with ramp:
        launch_pos + target_vel * (T - ramp_s) + target_vel * ramp_s / 2 = target_pos
        → launch_pos = target_pos - target_vel * (T - ramp_s / 2)

    Without ramp (ramp_s = 0):
        launch_pos = target_pos - target_vel * T   (original formula)

    Returns
    -------
    np.ndarray shape (3,)
    """
    target_pos = np.asarray(target_pos, dtype=float)
    target_vel = np.asarray(target_vel, dtype=float)
    ramp_s = float(np.clip(ramp_s, 0.0, damp_seconds))
    return target_pos - target_vel * (damp_seconds - ramp_s / 2.0)


# ---------------------------------------------------------------------------
# Trajectory factory — linear + optional ramp
# ---------------------------------------------------------------------------

def make_linear_traj(
    target_pos,
    target_vel,
    duration: float,
    ramp_s: float = 0.0,
) -> "Callable[[float], Tuple[np.ndarray, np.ndarray]]":
    """
    Return a trajectory callable for the classic linear+ramp kinematic path.

    The returned ``fn(t_sim)`` produces:
        t < t_ramp_start           : constant velocity toward target_pos
        t_ramp_start <= t < duration : velocity linearly tapered to zero

    This is the default trajectory used by ``KinematicStartup`` when no
    ``traj_fn`` is provided.  Use it directly when you need to compose the
    linear trajectory with other logic.

    Parameters
    ----------
    target_pos : array-like (3,)
        Hub position in NED [m] at t = duration.
    target_vel : array-like (3,)
        Hub velocity in NED [m/s] during the constant phase.
    duration : float
        Total kinematic duration [s].
    ramp_s : float, optional
        Velocity ramp-to-zero window at end [s].  Clamped to [0, duration].

    Returns
    -------
    fn : Callable[[float], tuple[ndarray, ndarray]]
        ``fn(t_sim)`` → ``(pos, vel)`` in NED.  Caller is responsible for
        bounding ``t_sim`` to ``[0, duration)``.
    """
    target_pos   = np.asarray(target_pos, dtype=float).copy()
    target_vel   = np.asarray(target_vel, dtype=float).copy()
    ramp_s       = float(np.clip(ramp_s, 0.0, duration))
    launch_pos   = compute_launch_position(target_pos, target_vel, duration, ramp_s)
    t_ramp_start = duration - ramp_s
    ramp_start_pos = launch_pos + target_vel * t_ramp_start

    def _fn(t_sim: float) -> "Tuple[np.ndarray, np.ndarray]":
        if ramp_s > 0.0 and t_sim >= t_ramp_start:
            u         = t_sim - t_ramp_start
            ramp_frac = max(0.0, (ramp_s - u) / ramp_s)
            pos = ramp_start_pos + target_vel * u * (1.0 - u / (2.0 * ramp_s))
            vel = target_vel * ramp_frac
        else:
            pos = launch_pos + target_vel * t_sim
            vel = target_vel.copy()
        return pos, vel

    def _accel(t_sim: float) -> "np.ndarray":
        if ramp_s > 0.0 and t_sim >= t_ramp_start:
            return -target_vel / ramp_s
        return np.zeros(3)

    _fn.accel = _accel  # type: ignore[attr-defined]
    return _fn


# ---------------------------------------------------------------------------
# Trajectory factory — minimum-jerk (5th-order polynomial)
# ---------------------------------------------------------------------------

def make_min_jerk_traj(
    p0,
    v0,
    pf,
    vf,
    T: float,
) -> "Callable[[float], Tuple[np.ndarray, np.ndarray]]":
    """
    Return a trajectory callable for a 5th-order minimum-jerk trajectory.

    Boundary conditions:
        pos(0) = p0,  vel(0) = v0,  accel(0) = 0
        pos(T) = pf,  vel(T) = vf,  accel(T) = 0

    Coefficients in normalised time tau = t/T:
        p(tau) = c0 + c1*tau + c2*tau^2 + c3*tau^3 + c4*tau^4 + c5*tau^5

    Zero initial/final acceleration constraint sets c2 = 0:
        D = pf - p0 - v0*T
        E = (vf - v0)*T
        c0=p0,  c1=v0*T,  c2=0,  c3=10D-4E,  c4=-15D+7E,  c5=6D-3E

    After t = T the callable returns ``(pf, vf)`` (clamped), so
    ``KinematicStartup.duration`` may exceed T to hold at the target
    position before free flight begins — e.g. to wait for GPS fusion.

    Parameters
    ----------
    p0 : array-like (3,)   Starting NED position [m].
    v0 : array-like (3,)   Starting NED velocity [m/s].
    pf : array-like (3,)   Final NED position [m].
    vf : array-like (3,)   Final NED velocity [m/s].
    T  : float             Trajectory duration [s].

    Returns
    -------
    fn : Callable[[float], tuple[ndarray, ndarray]]
        ``fn(t_sim)`` → ``(pos, vel)`` in NED.
    """
    p0 = np.asarray(p0, dtype=float).copy()
    v0 = np.asarray(v0, dtype=float).copy()
    pf = np.asarray(pf, dtype=float).copy()
    vf = np.asarray(vf, dtype=float).copy()
    T  = float(T)

    D  = pf - p0 - v0 * T
    E  = (vf - v0) * T
    c0 = p0
    c1 = v0 * T
    c3 = 10.0*D - 4.0*E
    c4 = -15.0*D + 7.0*E
    c5 = 6.0*D - 3.0*E

    def _fn(t_sim: float) -> "Tuple[np.ndarray, np.ndarray]":
        if t_sim >= T:
            # Extrapolate at constant vf — hub keeps moving after polynomial ends.
            # This avoids teleporting the hub back to pf each step when vf != 0.
            pos = pf + vf * (t_sim - T)
            return pos, vf.copy()
        tau = t_sim / T
        pos = c0 + c1*tau + c3*tau**3 + c4*tau**4 + c5*tau**5
        vel = (c1 + 3.0*c3*tau**2 + 4.0*c4*tau**3 + 5.0*c5*tau**4) / T
        return pos, vel

    return _fn


# ---------------------------------------------------------------------------
# KinematicStartup
# ---------------------------------------------------------------------------

class KinematicStartup:
    """
    Encapsulates all state and computation for the kinematic startup phase.

    Two usage modes
    ---------------
    **Linear trajectory (default)** — pass ``target_pos`` and ``target_vel``:
        The hub moves at constant velocity from a pre-computed ``launch_pos``,
        arriving at ``target_pos`` at t = ``duration``.  An optional velocity
        ramp (``ramp_s``) tapers the speed to zero at the end.

    **Custom trajectory** — pass ``traj_fn``:
        ``traj_fn(t_sim) -> (pos, vel)`` is called each step instead.  Any
        path is supported: circular orbits, waypoint splines, etc.  The
        ``target_pos``, ``target_vel``, and ``ramp_s`` arguments are ignored.
        Use ``make_linear_traj()`` to build a linear trajectory callable if
        you need to compose it with other logic.

    Orientation locking
    -------------------
    By default the orientation is locked to the fixed matrix ``R0`` for the
    entire kinematic window.  Pass ``R_fn(t_sim) -> R`` to vary the locked
    orientation over time (e.g. to keep body_z aligned with the tether
    direction on a curved trajectory).

    Parameters
    ----------
    target_pos : array-like (3,), optional
        Hub equilibrium position NED [m].  Required when ``traj_fn`` is None.
    target_vel : array-like (3,), optional
        Hub equilibrium velocity NED [m/s].  Required when ``traj_fn`` is None.
    duration : float
        Total kinematic phase duration [s].  0.0 → no kinematic phase (all
        methods return their free-flight sentinels immediately).
    ramp_s : float, optional
        Velocity ramp-to-zero window at end of kinematic [s].  Clamped to
        [0, duration].  Ignored when ``traj_fn`` is provided.  Default 0.0.
    R0 : ndarray (3, 3), optional
        Equilibrium rotation matrix body→world locked during kinematic
        (when ``R_fn`` is None).  Defaults to identity.
        Pass ``build_orb_frame(body_z)`` from the caller.
    traj_fn : Callable[[float], tuple[ndarray, ndarray]], optional
        Custom trajectory: ``t_sim → (pos, vel)`` in NED.  Overrides the
        linear+ramp path computed from ``target_pos``/``target_vel``/``ramp_s``.
    R_fn : Callable[[float], ndarray], optional
        Custom orientation: ``t_sim → R`` (3×3 body→world rotation matrix).
        Overrides the fixed ``R0`` during ``apply()``.

    Attributes (read-only after __init__)
    --------------------------------------
    launch_pos   starting position at t = 0 (NED [m])
    ramp_s       ramp duration [s] (0.0 for custom trajectories)
    """

    def __init__(
        self,
        target_pos = None,
        target_vel = None,
        duration:  float = 0.0,
        ramp_s:    float = 0.0,
        R0:        "np.ndarray | None" = None,
        traj_fn:   "Callable[[float], Tuple[np.ndarray, np.ndarray]] | None" = None,
        R_fn:      "Callable[[float], np.ndarray] | None" = None,
    ) -> None:
        self.duration = float(duration)
        self.R0       = (np.asarray(R0, dtype=float).copy()
                         if R0 is not None else np.eye(3))
        self._R_fn    = R_fn

        if traj_fn is not None:
            # ── Custom trajectory path ─────────────────────────────────────
            self._traj_fn = traj_fn
            self.ramp_s   = 0.0
            if self.duration > 0.0:
                _pos0, _ = traj_fn(0.0)
                self.launch_pos = np.asarray(_pos0, dtype=float).copy()
            else:
                self.launch_pos = np.zeros(3)
            # Keep target_pos/target_vel as None to signal custom mode.
            self.target_pos = None
            self.target_vel = None
        else:
            # ── Linear+ramp trajectory path (original behaviour) ──────────
            if target_pos is None or target_vel is None:
                raise ValueError(
                    "KinematicStartup: target_pos and target_vel are required "
                    "when traj_fn is not provided."
                )
            self.target_pos = np.asarray(target_pos, dtype=float).copy()
            self.target_vel = np.asarray(target_vel, dtype=float).copy()
            self.ramp_s     = float(np.clip(ramp_s, 0.0, self.duration))
            self._traj_fn   = make_linear_traj(
                self.target_pos, self.target_vel, self.duration, self.ramp_s
            )
            if self.duration > 0.0:
                self.launch_pos = compute_launch_position(
                    self.target_pos, self.target_vel, self.duration, self.ramp_s
                )
            else:
                self.launch_pos = self.target_pos.copy()

    # ------------------------------------------------------------------
    def accel_at(self, t_sim: float) -> np.ndarray:
        """Return desired acceleration [m/s²] at t_sim (zero outside kinematic)."""
        if not self.is_active(t_sim):
            return np.zeros(3)
        accel_fn = getattr(self._traj_fn, "accel", None)
        if accel_fn is not None:
            return accel_fn(t_sim)
        return np.zeros(3)

    # ------------------------------------------------------------------
    def is_active(self, t_sim: float) -> bool:
        """Return True while the kinematic phase is running (t_sim < duration)."""
        return self.duration > 0.0 and t_sim < self.duration

    # ------------------------------------------------------------------
    def damp_alpha(self, t_sim: float) -> float:
        """
        Kinematic blend factor: 1.0 at t = 0, 0.0 at t = duration, 0.0 after.

        Used by the mediator for angular damping:
            k_ang_total = base_k_ang + startup_damp_k_ang * startup.damp_alpha(t)
        """
        if self.duration <= 0.0 or t_sim >= self.duration:
            return 0.0
        return max(0.0, 1.0 - t_sim / self.duration)

    # ------------------------------------------------------------------
    def state_at(self, t_sim: float) -> "tuple[np.ndarray, np.ndarray] | None":
        """
        Return (pos, vel) for the kinematic trajectory at t_sim, or None if the
        kinematic phase has ended (caller should run free-flight physics).

        For the default linear+ramp mode this reproduces the original formula.
        For a custom traj_fn the result is ``traj_fn(t_sim)``.
        """
        if not self.is_active(t_sim):
            return None
        return self._traj_fn(t_sim)

    # ------------------------------------------------------------------
    def apply(self, hub_state: dict, dynamics: Any, t_sim: float) -> bool:
        """
        If kinematic phase is active, override hub_state in-place and sync the
        dynamics integrator's internal arrays so the first free-flight step
        integrates from the correct state.

        Returns True if the override was applied, False if free flight.

        Translation override
        --------------------
        Position and velocity are set to ``state_at(t_sim)`` every step.

        Orientation override
        --------------------
        If ``R_fn`` was provided: ``R = R_fn(t_sim)`` (time-varying, e.g. tracks
        the tether direction on a curved trajectory).
        Otherwise: ``R = R0`` (fixed equilibrium orientation, original behaviour).

        In both cases omega is held at zero so ACRO commands cannot misalign the
        disk before free-flight physics takes over.

        Parameters
        ----------
        hub_state : dict with keys pos, vel, R, omega (mutated in-place)
        dynamics  : RigidBodyDynamics — internal arrays _pos, _vel, _R, _omega
                    are updated to stay in sync with the override.
        t_sim     : current simulation time [s]
        """
        kin = self.state_at(t_sim)
        if kin is None:
            return False

        pos, vel = kin
        hub_state["pos"] = pos
        hub_state["vel"] = vel
        dynamics._pos[:] = pos
        dynamics._vel[:] = vel

        # Orientation: fixed R0 (default) or time-varying R_fn.
        R = self._R_fn(t_sim) if self._R_fn is not None else self.R0
        hub_state["R"]     = R.copy()
        hub_state["omega"] = np.zeros(3)
        dynamics._R[:]     = R
        dynamics._omega[:] = np.zeros(3)

        return True
