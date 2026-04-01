"""
dynamics.py — Python RK4 6-DOF rigid-body integrator for RAWES hub

Integrates Newton-Euler equations of motion for a single rigid body in the
NED world frame (X=North, Y=East, Z=Down).

State vector:
    pos     (3,)  hub position in NED world frame [m]
    vel     (3,)  hub velocity in NED world frame [m/s]
    R       (3,3) rotation matrix body → world (orthonormal)
    omega   (3,)  angular velocity in WORLD NED frame [rad/s]

Gravity is applied internally as F_grav = [0, 0, +m·g] in the NED world frame
(gravity acts in the +Z/Down direction).
The caller supplies only aerodynamic + tether forces/moments — no gravity.
"""

import numpy as np


class RigidBodyDynamics:
    """
    RK4 6-DOF rigid-body integrator.

    Parameters
    ----------
    mass : float
        Total rotor mass [kg].
    I_body : array-like, shape (3,)
        Principal moments of inertia [Ixx, Iyy, Izz] in the body frame [kg·m²].
        The inertia tensor is assumed diagonal (principal axes aligned with body axes).
    pos0 : array-like, shape (3,)
        Initial hub position in NED world frame [m].
    vel0 : array-like, shape (3,)
        Initial hub velocity in NED world frame [m/s].
    R0 : ndarray, shape (3,3)
        Initial rotation matrix body→world.  Defaults to identity (upright hub).
    omega0 : array-like, shape (3,)
        Initial angular velocity in the WORLD NED frame [rad/s].
    g : float
        Gravitational acceleration [m/s²].  Applied as +g in the NED Z (Down) direction.
    reorth_interval : int
        Number of integration steps between SVD re-orthogonalisations of R.
        Re-orthogonalisation corrects floating-point drift without altering the
        represented rotation significantly.
    """

    def __init__(
        self,
        mass:             float,
        I_body,                       # (3,) principal moments [kg·m²]
        pos0,                         # (3,) initial position ENU [m]
        vel0,                         # (3,) initial velocity [m/s]
        R0:               np.ndarray  = None,
        omega0                        = None,  # (3,) initial orbital omega world [rad/s]
        g:                float       = 9.81,
        reorth_interval:  int         = 200,
        I_spin:           float       = 0.0,   # spin-axis inertia for gyroscopic coupling [kg·m²]
        z_floor:          float       = None,  # maximum NED Z (altitude floor) [m]; None = no floor
    ):
        self.mass    = float(mass)
        self.g       = float(g)
        self._reorth = int(reorth_interval)
        self._I_spin = float(I_spin)  # rotor spin inertia (separate DOF)
        self._z_floor = float(z_floor) if z_floor is not None else None

        I = np.asarray(I_body, dtype=float)
        self._I_b     = np.diag(I)
        self._I_b_inv = np.diag(1.0 / I)

        self._pos   = np.asarray(pos0,   dtype=float).copy()
        self._vel   = np.asarray(vel0,   dtype=float).copy()
        self._R     = np.eye(3, dtype=float) if R0 is None else np.asarray(R0, dtype=float).copy()
        self._omega = np.zeros(3, dtype=float) if omega0 is None else np.asarray(omega0, dtype=float).copy()

        self._step_count = 0

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _skew(w: np.ndarray) -> np.ndarray:
        """3×3 skew-symmetric cross-product matrix of vector w."""
        return np.array([
            [ 0.0,   -w[2],  w[1]],
            [ w[2],   0.0,  -w[0]],
            [-w[1],   w[0],  0.0 ],
        ])

    def _derivs(
        self,
        pos:      np.ndarray,
        vel:      np.ndarray,
        R:        np.ndarray,
        omega_w:  np.ndarray,
        F_ext:    np.ndarray,   # external force in world frame (no gravity)
        M_ext:    np.ndarray,   # external moment in world frame (orbital moments only)
        omega_spin: float = 0.0,  # rotor spin rate [rad/s] for gyroscopic coupling
    ):
        """
        State derivatives for [pos, vel, R, omega_orbital_world].

        omega_w is the ORBITAL angular velocity (tilting/precessing) only — it does
        not include the rotor spin.  The spin is kept as a separate scalar state
        (omega_spin) maintained by the mediator and fed in here solely to compute the
        gyroscopic coupling term that the spinning rotor exerts on hub attitude.

        Gravity is added here so the caller never needs to include it.
        Euler's rotational equations are solved in the body frame and
        converted back to the world frame.
        """
        # --- linear ---
        F_total = F_ext + np.array([0.0, 0.0, +self.g * self.mass])
        dpos    = vel
        dvel    = F_total / self.mass

        # --- angular (Euler's equation in body frame) ---
        omega_b  = R.T @ omega_w
        tau_b    = R.T @ M_ext

        # Gyroscopic coupling from the spinning rotor.
        # The rotor carries angular momentum H_spin = I_spin * omega_spin along body Z.
        # Euler's equation for the orbital DOF:
        #   I_b * domega_b = tau_b − omega_b × (I_b * omega_b + H_spin_b)
        H_spin_b = np.array([0.0, 0.0, self._I_spin * omega_spin])
        Ih = self._I_b @ omega_b + H_spin_b
        gyro = np.array([
            omega_b[1]*Ih[2] - omega_b[2]*Ih[1],
            omega_b[2]*Ih[0] - omega_b[0]*Ih[2],
            omega_b[0]*Ih[1] - omega_b[1]*Ih[0],
        ])
        domega_b = self._I_b_inv @ (tau_b - gyro)

        # Convert back to world frame
        domega_w = R @ domega_b

        # --- rotation matrix kinematics ---
        # dR/dt = skew(ω_orbital) · R  (spin does NOT rotate R — it's a separate DOF)
        dR = self._skew(omega_w) @ R

        return dpos, dvel, dR, domega_w

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def step(
        self,
        F_world:    np.ndarray,   # (3,) external force in NED world frame [N]  — no gravity
        M_world:    np.ndarray,   # (3,) orbital moments in NED world frame [N·m] (no M_spin)
        dt:         float,
        omega_spin: float = 0.0,  # rotor spin rate [rad/s] — for gyroscopic coupling only
    ) -> dict:
        """
        Advance the rigid-body state by dt seconds using RK4.

        Parameters
        ----------
        F_world : (3,) ndarray
            External force in NED world frame [N].  Do NOT include gravity;
            it is applied internally.
        M_world : (3,) ndarray
            Orbital (cyclic) moment in NED world frame [N·m].
            Do NOT include the spin-axis drag/drive torque (M_spin) — that is
            integrated separately by the mediator as the scalar omega_spin state.
        dt : float
            Integration timestep [s].
        omega_spin : float
            Current rotor spin rate [rad/s].  Used only to compute the gyroscopic
            coupling term in Euler's equations; does not modify omega_spin itself.

        Returns
        -------
        dict with keys "pos", "vel", "R", "omega".
        """
        p, v, R, ow = self._pos, self._vel, self._R, self._omega

        k1 = self._derivs(p,              v,              R,              ow,              F_world, M_world, omega_spin)
        k2 = self._derivs(p+.5*dt*k1[0], v+.5*dt*k1[1], R+.5*dt*k1[2], ow+.5*dt*k1[3], F_world, M_world, omega_spin)
        k3 = self._derivs(p+.5*dt*k2[0], v+.5*dt*k2[1], R+.5*dt*k2[2], ow+.5*dt*k2[3], F_world, M_world, omega_spin)
        k4 = self._derivs(p+   dt*k3[0], v+   dt*k3[1], R+   dt*k3[2], ow+   dt*k3[3], F_world, M_world, omega_spin)

        c = dt / 6.0
        self._pos   = p  + c * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0])
        self._vel   = v  + c * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
        self._R     = R  + c * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2])
        self._omega = ow + c * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3])

        # Ground floor: prevent hub from falling through altitude floor.
        # In NED, Z increases downward, so the altitude floor is a maximum NED Z value.
        if self._z_floor is not None and self._pos[2] > self._z_floor:
            self._pos[2] = self._z_floor
            if self._vel[2] > 0.0:   # downward velocity → zero
                self._vel[2] = 0.0

        # Periodic SVD re-orthogonalisation to suppress R drift
        self._step_count += 1
        if self._step_count >= self._reorth:
            U, _, Vt = np.linalg.svd(self._R)
            self._R = U @ Vt
            self._step_count = 0

        return {
            "pos":   self._pos.copy(),
            "vel":   self._vel.copy(),
            "R":     self._R.copy(),
            "omega": self._omega.copy(),
        }

    @property
    def state(self) -> dict:
        """Current state without advancing (read-only snapshot)."""
        return {
            "pos":   self._pos.copy(),
            "vel":   self._vel.copy(),
            "R":     self._R.copy(),
            "omega": self._omega.copy(),
        }


# ---------------------------------------------------------------------------
# Smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    print("RigidBodyDynamics smoke test")

    # Hover test: upward thrust = weight; in NED upward = -Z direction.
    # Should stay at initial altitude (NED Z = -50 m = 50 m above ground).
    dyn = RigidBodyDynamics(
        mass   = 5.0,
        I_body = [5.0, 5.0, 10.0],
        pos0   = [0.0, 0.0, -50.0],  # NED Z = -50 m (50 m altitude)
        vel0   = [0.0, 0.0,   0.0],
        omega0 = [0.0, 0.0,  28.0],
    )

    W = 5.0 * 9.81  # weight [N]
    dt = 2.5e-3

    for _ in range(400):   # 1 second
        s = dyn.step(
            F_world = np.array([0.0, 0.0, -W]),  # upward thrust in NED = -Z
            M_world = np.zeros(3),
            dt      = dt,
        )

    z = s["pos"][2]
    assert abs(z - (-50.0)) < 0.01, f"Hover drift too large: z={z:.4f} m"
    print(f"  Hover 1s: z={z:.4f} m (expect -50.0) — OK")

    # Free-fall test: no external force, should descend (NED Z increases).
    dyn2 = RigidBodyDynamics(
        mass   = 5.0,
        I_body = [5.0, 5.0, 10.0],
        pos0   = [0.0, 0.0, -50.0],  # 50 m altitude
        vel0   = [0.0, 0.0,   0.0],
        omega0 = [0.0, 0.0,   0.0],
    )
    for _ in range(400):
        s2 = dyn2.step(F_world=np.zeros(3), M_world=np.zeros(3), dt=dt)

    z2 = s2["pos"][2]
    z_expected = -50.0 + 0.5 * 9.81 * 1.0**2  # NED Z increases as hub descends
    assert abs(z2 - z_expected) < 0.1, f"Free-fall z={z2:.3f} expected {z_expected:.3f}"
    print(f"  Free-fall 1s: z={z2:.3f} m (expect {z_expected:.3f}) — OK")

    print("All smoke tests passed.")
    sys.exit(0)
