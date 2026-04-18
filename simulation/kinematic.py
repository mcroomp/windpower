"""
kinematic.py — Kinematic startup phase for RAWES simulation.

During the first ``duration`` seconds of simulation the hub position, velocity,
and orientation are overridden directly (kinematic control) rather than computed
from physics.  This gives the EKF time to initialise on GPS before free flight.

Linear+ramp trajectory (the only trajectory type)
--------------------------------------------------
    t in [0, t_ramp_start)        : vel = target_vel  (constant)
    t in [t_ramp_start, duration] : vel = target_vel * (duration - t) / ramp_s
                                     (linear taper to zero)

Position is the analytic integral of the velocity profile, adjusted so the hub
arrives exactly at ``target_pos`` with ``target_vel`` at t = ``duration``.

With dual GPS (EK3_SRC1_YAW=2, RELPOSNED heading from F9P moving-baseline),
GPS fusion does not require vehicle motion.  Yaw is known from the first GPS fix;
delAngBiasLearned converges with constant-zero gyro input (~21 s after arm).
A stationary hold (vel0=[0,0,0], ramp_s=0) is the standard kinematic pattern.

Custom trajectories
-------------------
Pass ``traj_fn(t_sim) -> (pos, vel)`` to fly any path during kinematic startup.
Optionally pass ``R_fn(t_sim) -> R`` to vary the locked orientation over time.
Use ``make_linear_traj()`` to build a linear trajectory callable.

Shared between mediator.py (production loop) and unit/simtests (no Docker).
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
        ``traj_fn(t_sim) -> (pos, vel)`` is called each step instead.  The
        ``target_pos``, ``target_vel``, and ``ramp_s`` arguments are ignored.
        Use ``make_linear_traj()`` to build a linear trajectory callable if
        you need to compose it with other logic.

    Orientation locking
    -------------------
    By default the orientation is locked to the fixed matrix ``R0`` for the
    entire kinematic window.  Pass ``R_fn(t_sim) -> R`` to vary the locked
    orientation over time.

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
    def _omega_body_at(self, t_sim: float) -> np.ndarray:
        """
        Compute world-frame (NED) angular velocity [rad/s] from the time derivative
        of R_fn at t_sim, or return zeros if R_fn is not set.

        Uses 1 ms numerical differentiation of R_fn:
            S        = R(t)^T @ (R(t+dt) - R(t)) / dt   (body-frame skew-symmetric)
            omega_b  = vee(S)                             (body-frame angular velocity)
            omega_w  = R(t) @ omega_b                    (world-frame, NED)

        Returns world-frame omega because hub_state["omega"] and mediator.py
        yaw damping both use the world (NED) frame convention.

        At fast circles (~1 rad/s) the body-frame and world-frame values differ
        significantly (~6x at typical RAWES tilt).  World-frame is required so
        that apply() can correctly strip GPS-tracking spin before setting
        dynamics._omega as the free-flight initial condition.
        """
        if self._R_fn is None:
            return np.zeros(3)
        dt_fd = 1e-3                           # 1 ms: small enough for accuracy, large enough for stability
        R0    = self._R_fn(t_sim)
        R1    = self._R_fn(t_sim + dt_fd)
        # S = R^T @ dR/dt  (should be skew-symmetric: S = -S^T)
        S = R0.T @ ((R1 - R0) / dt_fd)
        # vee operator extracts body-frame angular velocity from skew-symmetric matrix:
        #   S = [[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]]
        #   omega_body = [S[2,1], S[0,2], S[1,0]]
        omega_body_frame = np.array([S[2, 1], S[0, 2], S[1, 0]])
        # sensor.compute() and hub_state["omega"] convention: world (NED) frame.
        # Convert: omega_world = R @ omega_body
        return R0 @ omega_body_frame

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

        Angular velocity
        ----------------
        ``hub_state["omega"]`` is set to the full world-frame omega of R_fn so
        that the gyro reported to SITL is consistent with the rotating R trajectory.
        This prevents the EKF from estimating a spurious gyro bias.

        ``dynamics._omega`` is zeroed: the physical hub has no angular velocity in
        steady orbit (GB4008 damps it), so zero is the correct free-flight IC at
        kinematic exit.

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
        hub_state["R"] = R.copy()
        dynamics._R[:] = R

        # Angular velocity.
        #
        # hub_state["omega"] → sensor.py → gyro_body reported to SITL.
        # Must be the FULL world-frame omega of R_fn so that gyro is consistent
        # with the R trajectory.  If we strip the GPS-heading spin here, the EKF
        # sees R rotating at ~1 rad/s but gyro reporting a smaller rate → it
        # estimates a large gyro bias → delAngBiasLearned stays false → GPS
        # fusion blocked or arrives with a large innovation → GPS glitch.
        #
        # dynamics._omega → free-flight initial condition at kinematic exit.
        # R_fn includes an artificial GPS-heading spin (~1 rad/s near fast-circle
        # closure) that does not represent real hub rotation.  Strip this
        # component (along disk_normal) from the free-flight IC only, so physics
        # starts with the true orbital angular velocity (~0.01–0.19 rad/s).
        omega_world = self._omega_body_at(t_sim)
        hub_state["omega"] = omega_world   # full omega → sensors, consistent with R_fn
        dynamics._omega[:] = 0.0          # free-flight IC: hub has no physical angular velocity

        return True
