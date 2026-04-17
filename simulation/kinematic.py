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

Orbital trajectory
------------------
Use ``make_orbital_kinematics(anchor_pos, p0, v_orb)`` to fly a circular orbit
around ``anchor_pos`` at constant altitude.  Returns ``(traj_fn, R_fn)`` where
``R_fn`` keeps body_z aligned with the tether direction and compass yaw matches
the GPS velocity heading at every step.

Why orbital is required for EKFGSF convergence:
  The EK3_SRC1_YAW=8 GPS-velocity-yaw source uses the EKFGSF (Gaussian Sum
  Filter) to estimate heading from GPS velocity.  EKFGSF maintains N=5 heading
  hypotheses and down-weights the ones whose predicted velocity diverges from
  GPS.  With a constant straight-line velocity, all hypotheses predict the same
  NED velocity — no divergence, no discrimination, YCS stays at maximum, and
  yawAlignComplete is never set.  An orbital trajectory gives a continuously
  rotating GPS velocity heading (omega = v_orb / r_h rad/s) that makes the N
  hypotheses produce distinguishably different velocity predictions, allowing
  the filter to converge.

Custom trajectories
-------------------
Pass ``traj_fn(t_sim) -> (pos, vel)`` to fly any path during kinematic startup.
Optionally pass ``R_fn(t_sim) -> R`` to vary the locked orientation over time
(e.g. so body_z tracks the tether direction on a curved trajectory).

Use ``make_linear_traj()``, ``make_min_jerk_traj()``, or
``make_orbital_kinematics()`` to build callables.

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
# Trajectory factory — circular orbital
# ---------------------------------------------------------------------------

def make_orbital_kinematics(
    anchor_pos,
    p0,
    v_orb: float,
    orbit_dir: int = +1,
) -> "tuple[Callable[[float], Tuple[np.ndarray, np.ndarray]], Callable[[float], np.ndarray]]":
    """
    Return ``(traj_fn, R_fn)`` for a circular orbital kinematic trajectory.

    The hub orbits ``anchor_pos`` at constant altitude and constant radius, giving
    a continuously rotating GPS velocity heading.  This is required for EKFGSF yaw
    convergence: a constant straight-line velocity makes all EKFGSF hypotheses
    predict identical NED velocities, so the filter cannot discriminate between
    them and yawAlignComplete is never set.

    Both returned callables share the same orbital geometry so they stay
    consistent with each other.

    Parameters
    ----------
    anchor_pos : array-like (3,)
        Anchor position in NED [m].  Usually ``[0, 0, 0]``.
    p0 : array-like (3,)
        Hub equilibrium position in NED [m].  Defines the orbital radius,
        altitude, and starting angle.  The hub starts here at t=0.
    v_orb : float
        Orbital speed [m/s].
    orbit_dir : int
        ``+1`` = counter-clockwise when viewed from above (N→E→S→W).
        ``-1`` = clockwise.

    Returns
    -------
    traj_fn : Callable[[float], tuple[ndarray, ndarray]]
        ``traj_fn(t_sim)`` → ``(pos, vel)`` in NED.
        Attaches ``traj_fn.accel(t_sim)`` → centripetal acceleration [m/s²].
    R_fn : Callable[[float], ndarray]
        ``R_fn(t_sim)`` → 3×3 body-to-NED rotation matrix.
        body_z tracks the tether direction (anchor→hub); x_orb is aligned so
        ZYX yaw equals the GPS velocity heading at every step.
    """
    # Import here to avoid making frames a module-level dependency; frames.py
    # is pure-numpy and Docker-free so this is safe in all test contexts.
    from frames import build_vel_aligned_frame

    anchor = np.asarray(anchor_pos, dtype=float).copy()
    p0_arr = np.asarray(p0,         dtype=float).copy()

    r_vec   = p0_arr - anchor
    r_h     = float(np.hypot(r_vec[0], r_vec[1]))
    if r_h < 0.1:
        raise ValueError(
            f"make_orbital_kinematics: hub is directly above/below anchor "
            f"(r_h={r_h:.3f} m).  Check anchor_pos and p0."
        )

    alt_d    = float(p0_arr[2])              # NED Z of hub (negative = above ground)
    theta0   = float(np.arctan2(r_vec[1], r_vec[0]))
    omega    = v_orb / r_h                   # angular velocity [rad/s]
    teth_len = float(np.sqrt(r_h**2 + alt_d**2))  # tether length [m]

    def _traj_fn(t_sim: float) -> "Tuple[np.ndarray, np.ndarray]":
        theta = theta0 + orbit_dir * omega * t_sim
        pos = np.array([
            anchor[0] + r_h * np.cos(theta),
            anchor[1] + r_h * np.sin(theta),
            alt_d,
        ])
        vel = np.array([
            -orbit_dir * v_orb * np.sin(theta),
             orbit_dir * v_orb * np.cos(theta),
             0.0,
        ])
        return pos, vel

    def _accel(t_sim: float) -> "np.ndarray":
        # Centripetal acceleration: a = v²/r inward = v_orb*omega inward
        theta = theta0 + orbit_dir * omega * t_sim
        return v_orb * omega * np.array([-np.cos(theta), -np.sin(theta), 0.0])

    _traj_fn.accel = _accel  # type: ignore[attr-defined]

    def _R_fn(t_sim: float) -> "np.ndarray":
        theta  = theta0 + orbit_dir * omega * t_sim
        # body_z: tether direction from anchor to hub
        body_z = np.array([r_h * np.cos(theta), r_h * np.sin(theta), alt_d]) / teth_len
        vel    = np.array([
            -orbit_dir * v_orb * np.sin(theta),
             orbit_dir * v_orb * np.cos(theta),
             0.0,
        ])
        # Align x_orb so ZYX yaw == GPS velocity heading — zero heading gap prevents
        # EKF GPS-glitch at every step of the orbital kinematic.
        return build_vel_aligned_frame(body_z, vel)

    return _traj_fn, _R_fn


# ---------------------------------------------------------------------------
# Trajectory factory — fast circles → slow orbit  (EKFGSF convergence)
# ---------------------------------------------------------------------------

def make_fast_circle_orbit_kinematics(
    anchor_pos,
    p_eq,
    v_orb_eq:   float,
    v_fast:     float  = 5.0,
    n_fast:     int    = 0,
    T_lead:     float  = 10.0,
    r_circle:   float  = 10.0,
    orbit_dir:  int    = +1,
    t_hold:     float  = 15.0,
) -> "tuple[Callable[[float], Tuple[np.ndarray, np.ndarray]], Callable[[float], np.ndarray]]":
    """
    Five-phase kinematic trajectory: hold → accelerate → [constant] → decelerate → large orbit.

    Designed to converge EKFGSF (GPS-velocity-yaw, EK3_SRC1_YAW=8) quickly.
    EKFGSF needs a rotating GPS velocity heading to discriminate its 5 yaw
    hypotheses (spaced 72° apart).  The large equilibrium orbit gives only
    ~0.56 deg/s — too slow for practical convergence.  This trajectory briefly
    circles a tight loop at v_fast (default 5 m/s → 57 deg/s at r=5 m), then
    decelerates back to equilibrium orbital speed before handing off to free flight.

    Phase 0 — stationary hold (t ∈ [0, t_hold]):
        Hub sits at p_start with zero velocity.  EKF tilt aligns, GPS detects,
        and the vehicle is armed during this window.  GPS heading is undefined
        (vel=0) so EKFGSF is idle — that is expected and fine.

    Phase 1 — acceleration circle (t ∈ [t_hold, t_hold + T_accel]):
        Hub accelerates from rest to v_fast over exactly one full circle.
        Speed ramps linearly: v(u) = v_fast * u / T_accel, u = t − t_hold.
        T_accel = 4π r_circle / v_fast  (derived so angle traversed = 2π).
        GPS heading immediately starts rotating — EKFGSF sees the signal from
        the first metre of movement.  omega_heading at v_fast/2 is already
        v_fast / (2 r_circle) rad/s.

    Phase 2 — constant-speed circles (t ∈ [t_hold+T_accel, t_hold+T_accel + n_fast·T_per]):
        Hub circles at v_fast for n_fast additional circles.  Optional: n_fast=0
        skips this phase entirely.  The acceleration circle (Phase 1) alone
        provides sufficient EKFGSF signal at v_fast ≥ 3 m/s.

    Phase 3 — deceleration circle:
        One circle, speed linearly ramped from v_fast down to v_orb_eq.
        Hub returns to p_start at orbital speed and heading.

    Phase 4 — large orbit lead-in (T_lead seconds):
        Hub follows the equilibrium orbit from p_start to p_eq.  Lua captures
        orbit tracking here (GPS already fused by Phase 1).  Kinematic exits
        at p_eq with the correct orbital velocity.

    Geometry:
        p_eq      — equilibrium position on large orbit (kinematic exit)
        p_start   — large orbit position T_lead s before p_eq (hub starts here)
        c         — circle centre = p_start shifted r_circle m toward anchor
        |p_start - c| = r_circle  → hub returns to p_start after each full circle

    Phase durations:
        T_accel  = 4π r_circle / v_fast                (accel circle)
        T_per    = 2π r_circle / v_fast                (one constant circle)
        T_decel  = 2π r_circle / mean(v_fast, v_orb_eq) (decel circle)
        Total    = t_hold + T_accel + n_fast·T_per + T_decel + T_lead

    Parameters
    ----------
    anchor_pos : array-like (3,)  Anchor NED position [m].
    p_eq       : array-like (3,)  Equilibrium (exit) position NED [m].
    v_orb_eq   : float            Equilibrium orbital speed [m/s].
    v_fast     : float            Peak circle speed [m/s].  Default 5.0.
                                  Must be > v_orb_eq.  Use ≥ 3 m/s for reliable
                                  EKFGSF convergence.
    n_fast     : int              Additional constant-speed circles after the
                                  acceleration circle.  Default 0.
    T_lead     : float            Large-orbit lead-in before exit [s].  Default 10.
    r_circle   : float            Circle radius [m].  Default 10.
    orbit_dir  : int              +1=CCW, -1=CW (viewed from above).
    t_hold     : float            Stationary hold at p_start before Phase 1 [s].
                                  Set to ≥ the expected arm time (typically 12–15 s).
                                  Default 15.0.

    Returns
    -------
    traj_fn : Callable  traj_fn(t) → (pos, vel) NED.  Has .accel(t) attribute.
    R_fn    : Callable  R_fn(t) → 3×3 body-to-NED rotation matrix.
    """
    anchor  = np.asarray(anchor_pos, dtype=float).copy()
    p_eq    = np.asarray(p_eq,       dtype=float).copy()

    # ── Large orbit geometry ──────────────────────────────────────────────────
    r_vec     = p_eq[:2] - anchor[:2]
    r_h       = float(np.hypot(r_vec[0], r_vec[1]))
    alt_d     = float(p_eq[2])
    theta_eq  = float(np.arctan2(r_vec[1], r_vec[0]))
    omega_lrg = v_orb_eq / r_h          # large-orbit angular velocity [rad/s]

    # p_start: large-orbit position T_lead s before p_eq (hub holds and exits here)
    theta_start = theta_eq - orbit_dir * omega_lrg * T_lead
    p_start = np.array([
        anchor[0] + r_h * np.cos(theta_start),
        anchor[1] + r_h * np.sin(theta_start),
        alt_d,
    ])

    # ── Fast-circle geometry ──────────────────────────────────────────────────
    # Centre is r_circle m toward anchor (horizontal) from p_start.
    c = np.array([
        anchor[0] + (r_h - r_circle) * np.cos(theta_start),
        anchor[1] + (r_h - r_circle) * np.sin(theta_start),
        alt_d,
    ])
    circle_theta0 = theta_start          # angle from c to p_start = theta_start

    omega_fast   = v_fast / r_circle
    T_per_fast   = 2.0 * np.pi / omega_fast                        # period of one constant circle
    T_accel      = 4.0 * np.pi * r_circle / v_fast                 # accel circle: 0 → v_fast in 2π
    v_mean_decel = 0.5 * (v_fast + v_orb_eq)
    T_decel      = 2.0 * np.pi * r_circle / v_mean_decel           # decel circle: v_fast → v_orb_eq

    # Phase time boundaries (relative to t=0)
    t_a = float(t_hold)                             # end of hold / start of accel
    t_b = t_a + T_accel                             # end of accel / start of constant
    t_c = t_b + n_fast * T_per_fast                 # end of constant / start of decel
    t_d = t_c + T_decel                             # end of decel / start of large orbit
    t_total = t_d + T_lead                          # kinematic exit

    # After accel circle (1×2π) + n_fast constant circles (n_fast×2π), the hub
    # is back at circle_theta0.  The decel circle starts from the same angle.
    theta_decel_start = circle_theta0 + orbit_dir * (1.0 + n_fast) * 2.0 * np.pi

    # ── Trajectory callable ───────────────────────────────────────────────────
    def _traj_fn(t_sim: float) -> "Tuple[np.ndarray, np.ndarray]":
        if t_sim < t_a:
            # Phase 0: stationary hold
            return p_start.copy(), np.zeros(3)

        elif t_sim < t_b:
            # Phase 1: acceleration circle — speed ramps 0 → v_fast over 2π
            u     = t_sim - t_a
            v     = v_fast * u / T_accel
            theta = circle_theta0 + orbit_dir * v_fast * u * u / (2.0 * r_circle * T_accel)
            pos   = np.array([c[0] + r_circle * np.cos(theta),
                               c[1] + r_circle * np.sin(theta), alt_d])
            vel   = np.array([-orbit_dir * v * np.sin(theta),
                               orbit_dir * v * np.cos(theta), 0.0])

        elif t_sim < t_c:
            # Phase 2: constant-speed circles at v_fast
            u     = t_sim - t_b
            theta = circle_theta0 + orbit_dir * (2.0 * np.pi + omega_fast * u)
            pos   = np.array([c[0] + r_circle * np.cos(theta),
                               c[1] + r_circle * np.sin(theta), alt_d])
            vel   = np.array([-orbit_dir * v_fast * np.sin(theta),
                               orbit_dir * v_fast * np.cos(theta), 0.0])

        elif t_sim < t_d:
            # Phase 3: decel circle — speed ramps v_fast → v_orb_eq over 2π
            u     = t_sim - t_c
            dv    = v_orb_eq - v_fast
            v     = v_fast + dv * u / T_decel
            delta = orbit_dir * (v_fast * u + dv * u * u / (2.0 * T_decel)) / r_circle
            theta = theta_decel_start + delta
            pos   = np.array([c[0] + r_circle * np.cos(theta),
                               c[1] + r_circle * np.sin(theta), alt_d])
            vel   = np.array([-orbit_dir * v * np.sin(theta),
                               orbit_dir * v * np.cos(theta), 0.0])

        else:
            # Phase 4: large equilibrium orbit from p_start → p_eq
            u     = t_sim - t_d
            theta = theta_start + orbit_dir * omega_lrg * u
            pos   = np.array([anchor[0] + r_h * np.cos(theta),
                               anchor[1] + r_h * np.sin(theta), alt_d])
            vel   = np.array([-orbit_dir * v_orb_eq * np.sin(theta),
                               orbit_dir * v_orb_eq * np.cos(theta), 0.0])

        return pos, vel

    def _accel(t_sim: float) -> "np.ndarray":
        if t_sim < t_a:
            # Phase 0: stationary
            return np.zeros(3)
        elif t_sim < t_b:
            # Phase 1: centripetal + tangential (constant tangential accel)
            u      = t_sim - t_a
            v      = v_fast * u / T_accel
            theta  = circle_theta0 + orbit_dir * v_fast * u * u / (2.0 * r_circle * T_accel)
            inward = np.array([-np.cos(theta), -np.sin(theta), 0.0])
            tangent = np.array([-orbit_dir * np.sin(theta), orbit_dir * np.cos(theta), 0.0])
            return (v * v / r_circle) * inward + (v_fast / T_accel) * tangent
        elif t_sim < t_c:
            # Phase 2: centripetal only (constant speed)
            u      = t_sim - t_b
            theta  = circle_theta0 + orbit_dir * (2.0 * np.pi + omega_fast * u)
            inward = np.array([-np.cos(theta), -np.sin(theta), 0.0])
            return (v_fast * v_fast / r_circle) * inward
        elif t_sim < t_d:
            # Phase 3: centripetal + tangential (constant tangential decel)
            u      = t_sim - t_c
            dv     = v_orb_eq - v_fast
            v      = v_fast + dv * u / T_decel
            delta  = orbit_dir * (v_fast * u + dv * u * u / (2.0 * T_decel)) / r_circle
            theta  = theta_decel_start + delta
            inward = np.array([-np.cos(theta), -np.sin(theta), 0.0])
            tangent = np.array([-orbit_dir * np.sin(theta), orbit_dir * np.cos(theta), 0.0])
            return (v * v / r_circle) * inward + (dv / T_decel) * tangent
        else:
            # Phase 4: centripetal toward anchor on large orbit
            u      = t_sim - t_d
            theta  = theta_start + orbit_dir * omega_lrg * u
            inward = np.array([-np.cos(theta), -np.sin(theta), 0.0])
            return (v_orb_eq * v_orb_eq / r_h) * inward

    _traj_fn.accel = _accel  # type: ignore[attr-defined]

    # ── Velocity reference for hold phase ────────────────────────────────────
    # During hold (vel=0), build_vel_aligned_frame would fall back to
    # build_orb_frame, giving yaw = East-reference orientation.  When Phase 1
    # starts and vel builds past 0.1 m/s, build_vel_aligned_frame switches to
    # tangential direction (~NW at p_start) → sudden 80+ deg yaw jump → 600+
    # deg/s gyro spike → EKF corruption.
    # Fix: pre-compute the tangential vel direction at p_start and use it as
    # the vel reference whenever actual speed is below 0.1 m/s.  This keeps
    # yaw aligned with the upcoming circle direction throughout the hold phase
    # so the transition is smooth.
    _vel_start_ref = np.array([
        -float(orbit_dir) * np.sin(theta_start),
         float(orbit_dir) * np.cos(theta_start),
         0.0,
    ])

    from frames import build_vel_aligned_frame as _bvaf

    def _R_fn(t_sim: float) -> "np.ndarray":
        pos, vel = _traj_fn(t_sim)
        raw    = pos - anchor
        body_z = raw / max(float(np.linalg.norm(raw)), 1e-6)
        # Use actual vel when moving; use tangential reference during hold so
        # there is no yaw discontinuity when circles start.
        v_horiz = float(np.sqrt(vel[0]**2 + vel[1]**2))
        vel_ref = vel if v_horiz >= 0.1 else _vel_start_ref
        return _bvaf(body_z, vel_ref)

    # ── Expose timing metadata used by the mediator ──────────────────────────
    _traj_fn.total_duration   = t_total  # type: ignore[attr-defined]
    _traj_fn.phase3_start_t   = t_d      # type: ignore[attr-defined]  # start of large-orbit lead-in
    _traj_fn.phase3_start_pos = p_start.copy()  # type: ignore[attr-defined]

    def _phase_at(t_sim: float) -> str:
        if t_sim < t_a:  return "hold"
        if t_sim < t_b:  return "accel"
        if t_sim < t_c:  return "const"
        if t_sim < t_d:  return "decel"
        return "orbit"

    _traj_fn.phase_at = _phase_at  # type: ignore[attr-defined]

    return _traj_fn, _R_fn


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
