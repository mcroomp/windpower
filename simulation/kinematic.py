"""
kinematic.py — Kinematic startup phase for RAWES simulation.

During the first ``duration`` seconds of simulation the hub position, velocity,
and orientation are overridden directly (kinematic control) rather than computed
from physics.  This gives the EKF time to initialise on GPS before free flight.

Velocity profile
----------------
    t in [0, t_ramp_start)        : vel = target_vel  (constant)
    t in [t_ramp_start, duration] : vel = target_vel * (duration - t) / ramp_s
                                     (linear taper to zero)

Position is the analytic integral of the velocity profile, adjusted so the hub
arrives exactly at ``target_pos`` with ``target_vel`` at t = ``duration``.

Shared between mediator.py (production loop) and unit/simtests (no Docker).
No imports beyond numpy — zero Docker / SITL dependencies.
"""
from __future__ import annotations

import numpy as np
from typing import Any, Optional


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
# KinematicStartup
# ---------------------------------------------------------------------------

class KinematicStartup:
    """
    Encapsulates all state and computation for the kinematic startup phase.

    Parameters
    ----------
    target_pos : array-like (3,)
        Hub equilibrium position NED [m].  The hub arrives here at t = duration.
    target_vel : array-like (3,)
        Hub equilibrium velocity NED [m/s].  Held constant until the ramp window.
    duration : float
        Total kinematic phase duration [s].  0.0 → no kinematic phase (all
        methods return their free-flight sentinels immediately).
    ramp_s : float, optional
        Velocity ramp-to-zero window at end of kinematic [s].  Clamped to
        [0, duration].  Default 0.0 (no ramp).
    R0 : ndarray (3, 3), optional
        Equilibrium rotation matrix body→world locked during kinematic.
        Defaults to identity.  Pass ``build_orb_frame(body_z)`` from the caller.

    Attributes (read-only after __init__)
    --------------------------------------
    launch_pos      starting position at t = 0 (NED [m])
    t_ramp_start    simulation time when velocity ramp begins [s]
    ramp_start_pos  hub position at t_ramp_start (NED [m])
    """

    def __init__(
        self,
        target_pos,
        target_vel,
        duration:    float,
        ramp_s:      float = 0.0,
        R0:          "np.ndarray | None" = None,
    ) -> None:
        self.target_pos = np.asarray(target_pos, dtype=float).copy()
        self.target_vel = np.asarray(target_vel, dtype=float).copy()
        self.duration   = float(duration)
        self.ramp_s     = float(np.clip(ramp_s, 0.0, self.duration))
        self.R0         = (np.asarray(R0, dtype=float).copy()
                           if R0 is not None else np.eye(3))

        # Pre-compute derived quantities used in every loop iteration.
        self.t_ramp_start = self.duration - self.ramp_s
        if self.duration > 0.0:
            self.launch_pos = compute_launch_position(
                self.target_pos, self.target_vel, self.duration, self.ramp_s)
        else:
            self.launch_pos = self.target_pos.copy()
        self.ramp_start_pos = self.launch_pos + self.target_vel * self.t_ramp_start

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

        Velocity profile:
            t in [0, t_ramp_start)        → vel = target_vel
            t in [t_ramp_start, duration] → vel = target_vel * (duration - t) / ramp_s
        Position is the analytic integral of the above.
        """
        if not self.is_active(t_sim):
            return None

        if self.ramp_s > 0.0 and t_sim >= self.t_ramp_start:
            u         = t_sim - self.t_ramp_start        # time into ramp [0, ramp_s]
            ramp_frac = max(0.0, (self.ramp_s - u) / self.ramp_s)
            pos = (self.ramp_start_pos
                   + self.target_vel * u * (1.0 - u / (2.0 * self.ramp_s)))
            vel = self.target_vel * ramp_frac
        else:
            pos = self.launch_pos + self.target_vel * t_sim
            vel = self.target_vel.copy()

        return pos, vel

    # ------------------------------------------------------------------
    def apply(self, hub_state: dict, dynamics: Any, t_sim: float) -> bool:
        """
        If kinematic phase is active, override hub_state in-place and sync the
        dynamics integrator's internal arrays so the first free-flight step
        integrates from the correct state.

        Returns True if the override was applied, False if free flight.

        Two-phase override
        ------------------
        Constant-velocity phase (t < t_ramp_start):
            pos, vel, R, omega all locked.  ACRO servo commands during this
            window would misalign the disk; locking R = R0 prevents that.

        Velocity-ramp phase (t_ramp_start <= t < duration, only when ramp_s > 0):
            pos and vel are still kinematically controlled (hub coasts smoothly
            to rest at target_pos), but R and omega are released.  The cyclic
            controller and tether restoring torque have ramp_s seconds to reach
            the orbit-angular equilibrium while the translational state is still
            under control.  At kinematic exit body_z is already settled, so the
            tether restoring torque produces no transient oscillation.

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

        # Lock orientation throughout the entire kinematic phase (including the
        # velocity-ramp window).  The mediator's internal controller only runs
        # when damp_alpha == 0.0 (free flight), so releasing R during the ramp
        # just lets ArduPilot ACRO drive body_z to a wrong orientation in the
        # 15 s before kinematic exit — causing insufficient lift on entry to
        # free flight.  Keeping R = R0 until the very last kinematic frame
        # ensures the hub arrives at free-flight start in the correct equilibrium
        # orientation, consistent with the mediator CRITICAL comment above.
        hub_state["R"]     = self.R0.copy()
        hub_state["omega"] = np.zeros(3)
        dynamics._R[:]     = self.R0
        dynamics._omega[:] = np.zeros(3)

        return True
