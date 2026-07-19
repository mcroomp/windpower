"""
controller.py — RAWES guidance/control helpers.

Legacy RC-override helpers are retained for historical tests only; runtime
flight stack control is GUIDED-only.
"""

import math

import numpy as np
from simulation.frames import build_orb_frame, cross3  # noqa: F401 — build_orb_frame re-exported for callers
from simulation.swashplate import SwashplateServoModel
from dynbem     import RotorInputs
from simulation.param_defaults import load_ap_params as _load_ap_params
from simulation.param_defaults import load_collective_phys_range as _load_col_range
from arduloop import HeliRateController, HeliParams, RateAxisParams


class TensionPI:
    """
    PID controller: adjusts collective_rad to maintain requested tether tension.

    Owned by the Trajectory Planner (ground station) — raws_mode.md §3.2.
    Runs on fresh load cell data at planner rate (~10 Hz or 400 Hz in simulation).
    Anti-windup clamps the integrator so output stays within [coll_min, coll_max].

    kd = 0.0 by default (pure PI); try ~2e-5 to damp tension oscillations.
    warm_coll_rad = None cold-starts the integrator at zero; pass the equilibrium
    collective to pre-seed the integrator and avoid a tension spike at t=0.
    """

    def __init__(self, setpoint_n: float, kp: float, ki: float,
                 coll_min: float, coll_max: float,
                 warm_coll_rad: "float | None",
                 kd: float = 0.0):
        self.setpoint  = float(setpoint_n)
        self.kp        = float(kp)
        self.ki        = float(ki)
        self.kd        = float(kd)
        self.coll_min  = float(coll_min)
        self.coll_max  = float(coll_max)
        if warm_coll_rad is not None:
            self._integral  = float(warm_coll_rad) / max(self.ki, 1e-12)
        else:
            self._integral  = 0.0
        self._prev_error = 0.0

    def update(self, tension_actual: float, dt: float) -> float:
        error = self.setpoint - tension_actual
        d_term = self.kd * (error - self._prev_error) / max(dt, 1e-6)
        raw_before = self.kp * error + self.ki * self._integral + d_term

        # Conditional anti-windup: only integrate when output is not already
        # saturated in the same direction as the error.  This prevents the
        # integrator from winding deep into the clamp, which would cause the
        # output to stay at the clamp limit even after the error reverses.
        if not (raw_before <= self.coll_min and error < 0):
            if not (raw_before >= self.coll_max and error > 0):
                self._integral += error * dt

        self._prev_error = error
        raw = self.kp * error + self.ki * self._integral + d_term
        return float(np.clip(raw, self.coll_min, self.coll_max))


def position_feedback_bz_eq(
    bz_eq:      np.ndarray,
    pos:        np.ndarray,
    vel:        np.ndarray,
    target_pos: np.ndarray,
    tension_n:  float,
    kp_pos:     float,
    kd_pos:     float,
    max_tilt_rad: float = math.radians(80.0),
) -> np.ndarray:
    """PD position-feedback correction added to a base body_z target.

    The base ``bz_eq`` (e.g. from ``compute_bz_tether`` or
    ``compute_bz_altitude_hold``) is an *open-loop* setpoint based on the
    target position.  When the hub is displaced from the target, this
    function tilts ``bz_eq`` in the direction of the displacement —
    because thrust is along ``−body_z`` in FRD, that tilt produces a
    thrust component **opposing** the displacement, restoring the hub.

    Physics
    -------
    With FRD body_z pointing hub→anchor, thrust = −T·body_z.  Tilting
    body_z toward Δp = pos − target_pos rotates thrust by −Δp/(T/|F|),
    producing a force ≈ −T·(Δp/T) = −Δp.  Stiffness gain ``kp_pos`` is
    the desired N per metre of position error; ``kd_pos`` is the N per
    m/s of velocity (viscous damping on lateral motion).

    Parameters
    ----------
    bz_eq       Base body_z target (FRD unit vector, world frame).
    pos, vel    Current hub position and velocity (world frame).
    target_pos  Desired hub position (world frame).
    tension_n   Current tether tension [N], for force→tilt conversion.
    kp_pos      Position gain [N/m].  Typical 5–30 for the beaupoil
                hub on a 100 m tether (pendulum period ~22 s, critical
                stiffness ~mω²·L ≈ 1 N/m; use 5–30× critical for fast
                settling without over-correction).
    kd_pos      Velocity gain [N·s/m].  Pick ~2·√(kp_pos·m) for
                critical damping (mass m ≈ 5 kg).

    Returns
    -------
    Corrected (normalised) body_z unit vector.
    """
    pos        = np.asarray(pos,        dtype=float)
    vel        = np.asarray(vel,        dtype=float)
    target_pos = np.asarray(target_pos, dtype=float)
    delta_pos  = pos - target_pos
    correction = (kp_pos * delta_pos + kd_pos * vel) / max(tension_n, 1.0)
    # Saturate the correction magnitude so a large transient excursion
    # doesn't request a disk tilt that flips body_z past the equator
    # (which destabilises the attitude loop) or overwhelms the gravity-
    # comp baseline.  ``max_tilt_rad`` is the cap on the additional tilt
    # beyond ``bz_eq``; the correction vector's magnitude is the small-
    # angle approximation of that tilt for unit ``bz_eq``.
    mag = float(np.linalg.norm(correction))
    if mag > max_tilt_rad:
        correction = correction * (max_tilt_rad / mag)
    bz_new     = np.asarray(bz_eq, dtype=float) + correction
    return bz_new / float(np.linalg.norm(bz_new))


def damp_bz_eq_lateral(
    bz_eq:     np.ndarray,
    pos:       np.ndarray,
    vel:       np.ndarray,
    anchor:    np.ndarray,
    tension_n: float,
    kd_lat:    float,
) -> np.ndarray:
    """Add a lateral-velocity damping correction to ``body_z_eq``.

    The open-loop gravity-comp tilt in ``compute_bz_altitude_hold`` only
    knows the *target* operating point — once the hub drifts perpendicular
    to the tether, the disk tilt doesn't adapt and the pendulum mode is
    undamped.  This function adds a small disk-tilt proportional to the
    hub's lateral velocity (the component of velocity perpendicular to
    the anchor→hub direction), so the rotor thrust gains a component that
    opposes that velocity.

    Physics
    -------
    FRD body_z points hub→anchor; thrust = −T·body_z (toward anchor).
    Tilting body_z **toward** v_lat produces a thrust component **against**
    v_lat with magnitude ≈ T·|Δb|.  Setting Δb = kd_lat·v_lat/T yields a
    damping force F_damp = −kd_lat·v_lat — viscous damping on lateral
    motion at gain ``kd_lat`` [N·s/m].

    Parameters
    ----------
    bz_eq      : current body_z target (FRD unit vector, world frame)
    pos        : hub position [m]
    vel        : hub velocity [m/s]
    anchor     : tether anchor position [m] (same NED origin as ``pos``)
    tension_n  : current tether tension [N], used to convert force →
                 disk-tilt magnitude.  Floored at 1 N for safety.
    kd_lat     : lateral velocity damping gain [N·s/m].  Typical values
                 ~5–30 for a 5 kg hub on a 100 m tether (pendulum mode
                 period ~22 s ⇒ critical damping coefficient ~3 N·s/m;
                 use 2–10× critical for fast settling).

    Returns
    -------
    Corrected (normalised) body_z unit vector.
    """
    pos     = np.asarray(pos,    dtype=float)
    vel     = np.asarray(vel,    dtype=float)
    anchor  = np.asarray(anchor, dtype=float)
    tether  = pos - anchor
    t_len   = float(np.linalg.norm(tether))
    if t_len < 0.1:
        return bz_eq
    tether_hat = tether / t_len

    # Lateral velocity: drop the along-tether component.
    v_along = float(np.dot(vel, tether_hat))
    v_lat   = vel - v_along * tether_hat

    # Tilt body_z toward v_lat by kd_lat · v_lat / T_safe.  Thrust = −T·body_z
    # therefore gains a component −kd_lat·v_lat ⇒ viscous damping.
    correction = kd_lat * v_lat / max(tension_n, 1.0)
    bz_new = np.asarray(bz_eq, dtype=float) + correction
    return bz_new / float(np.linalg.norm(bz_new))


def compute_crosswind_rate_cmd(
    pos:               np.ndarray,
    vel:               np.ndarray,
    target_pos:        np.ndarray,
    kp:                float,
    kd:                float,
    rate_max:          float = math.pi,
) -> float:
    """Compute world-frame East rotation rate from crosswind (North) error.

    A simple rate-based damper for lateral/crosswind excursion without
    tether-axis feedback.  Commands a rotation about the East (NED +Y)
    axis proportional to North position error and North velocity.

    Physics: A world-frame rotation about East is like a swing; tilting
    the body ``toward`` the North error produces a corrective Northward
    force from gravity (like pendulum restoring force).  The rate limit
    prevents aggressive saturated corrections.

    Parameters
    ----------
    pos        : hub NED position [m]
    vel        : hub velocity [m/s]
    target_pos : desired hub position [m]
    kp         : position gain [rad/s per m]
    kd         : velocity gain [rad/s per (m/s)]
    rate_max   : saturation limit on returned rate [rad/s]

    Returns
    -------
    float : world-frame East (NED +Y) rotation rate command [rad/s].
            Positive = body tilts North; negative = body tilts South.
    """
    pos        = np.asarray(pos,        dtype=float)
    vel        = np.asarray(vel,        dtype=float)
    target_pos = np.asarray(target_pos, dtype=float)

    crosswind_err = float(pos[0] - target_pos[0])  # North error (NED +X)
    crosswind_vel = float(vel[0])                  # North velocity (NED +X)

    omega_east_cmd = float(np.clip(
        -(kp * crosswind_err + kd * crosswind_vel),
        -rate_max,
        rate_max,
    ))
    return omega_east_cmd


def apply_crosswind_rate_to_body_rates(
    omega_east_world: float,
    R_body_to_world:  np.ndarray,
) -> np.ndarray:
    """Convert world-frame East rotation rate to body-frame roll/pitch rates.

    The crosswind rate loop commands a rotation about East (NED +Y).
    This rotation has roll and pitch components when expressed in the body
    frame, which are added to the rate command sent to the acro controller.

    Parameters
    ----------
    omega_east_world : world-frame East (NED +Y) rotation rate [rad/s]
    R_body_to_world  : rotation matrix; columns are body axes in world frame

    Returns
    -------
    np.ndarray (3,) : (roll_rate, pitch_rate, yaw_rate) correction [rad/s] to
                      be added to the main rate command.  Only roll and pitch
                      are typically nonzero; yaw is zero for a pure East
                      rotation.
    """
    R = np.asarray(R_body_to_world, dtype=float)
    # East in NED world frame
    east_world = np.array([0.0, 1.0, 0.0])
    omega_vec_world = omega_east_world * east_world
    # Map to body frame
    omega_body_corr = R.T @ omega_vec_world
    return np.asarray(omega_body_corr, dtype=float)


# Time constant [s] for the plane-keeping azimuth low-pass.  The body-z azimuth
# reference is slowly slewed toward the kite's instantaneous position azimuth so
# fast lateral excursions (e.g. low-tension reel-in) do not chase their own
# position — that positive feedback drives the kite off the downwind plane.
# This is a plane-keeping ESTIMATE derived from the kite's own position only; it
# uses no truth-wind oracle (AGENTS.md no-truth-wind invariant).
AZ_REF_TAU_S: float = 15.0


def _wrap_pi(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return float(math.atan2(math.sin(a), math.cos(a)))


def update_plane_azimuth(az_ref: float, pos: np.ndarray,
                         tau_s: float, dt: float) -> float:
    """Low-pass the downwind-plane azimuth toward the position azimuth.

    Portable plane-keeping estimator mirrored 1:1 in rawes.lua
    (update_plane_azimuth).  az_ref and the return are in (-pi, pi].
    """
    az_meas = float(math.atan2(pos[1], pos[0]))
    alpha   = dt / tau_s
    return _wrap_pi(az_ref + alpha * _wrap_pi(az_meas - az_ref))


def compute_bz_altitude_hold(
    pos:           np.ndarray,
    target_el_rad: float,
    tension_n:     float,
    mass_kg:       float,
    G:             float = 9.81,
    az_ref_rad:    float | None = None,
) -> np.ndarray:
    """
    Compute body_z_eq for altitude-holding flight at a target elevation angle.

    Points the disk at (target_elevation, current_azimuth) and adds a gravity-
    compensation tilt so the thrust has an elevation-upward component equal to
    mass_kg * G, exactly counteracting gravity's pull to lower elevation.

    Stateless — no orbit reference, no history.  Works at any azimuth or
    disturbance: a gust that changes azimuth is handled because azimuth is
    read from pos each call; a gust that drops elevation generates an immediate
    corrective tilt.

    Parameters
    ----------
    pos           : hub NED position [m]
    target_el_rad : target elevation angle above horizontal [rad]
    tension_n     : current tether tension [N] — scales the gravity compensation
    mass_kg       : hub mass [kg]
    G             : gravitational acceleration [m/s^2]

    Returns
    -------
    body_z_eq : NED unit vector — desired disk normal
    """
    az        = float(az_ref_rad) if az_ref_rad is not None else float(np.arctan2(pos[1], pos[0]))
    cos_el    = float(np.cos(target_el_rad))
    sin_el    = float(np.sin(target_el_rad))
    cos_az    = float(np.cos(az))
    sin_az    = float(np.sin(az))

    # FRD: body_z points DOWN through the disk, i.e. from hub toward anchor.
    # tdir = hub→anchor direction at the target elevation (toward the ground).
    # e_dn = elevation-downward direction (perpendicular to tdir, toward Earth).
    tdir = np.array([-cos_el * cos_az, -cos_el * sin_az,  sin_el])
    e_dn = np.array([ sin_el * cos_az,  sin_el * sin_az,  cos_el])

    # Gravity's tangential component in the elevation direction = mg·cos(el).
    # At low elevation this is nearly mg; at 90° (vertical) it is zero.
    g_tangential = mass_kg * G * float(np.cos(target_el_rad))
    raw = tdir + (g_tangential / max(tension_n, 1.0)) * e_dn
    return raw / np.linalg.norm(raw)


class YawTrimObserver:
    """Python port of rawes.lua's yaw trim observer (anti-rotation feedforward).

    Mirrors ``yaw_trim_step(dt, u, psi_dot)`` in ``scripts/rawes.lua``
    exactly (see ``test_yaw_trim_lua.py`` for the Lua-side unit tests and
    ``test_yaw_trim_parity.py`` for the cross-check). Keep these two in sync —
    per AGENTS.md, any change to the Lua formula must be mirrored here in the
    same commit.

    Reads back the total applied Motor4/SERVO9 throttle ``u`` [0, 1] and the
    measured body yaw rate ``psi_dot`` [rad/s], computes the equilibrium trim
    that would hold psi_dot=0 (``u - psi_dot / YFF_A``), and low-passes it
    into a running trim estimate (clamped to [0, YFF_MAX]).
    """

    #: Motor throttle→yaw-rate slope, bench-calibrated: 0.504 RPM/µs over the
    #: SERVO9 PWM span (1000 µs) → rad/s per unit throttle. Matches rawes.lua's
    #: default (RAWES_YAW_SLP=0 → this bench default).
    YFF_A_DEFAULT: float = 0.504 * 1000.0 * (2.0 * math.pi / 60.0)
    YFF_MAX_DEFAULT: float = 0.7
    YFF_TAU_DEFAULT: float = 0.3

    def __init__(self, yaw_slope_rpm_per_us: float = 0.0,
                 yff_max: float = YFF_MAX_DEFAULT,
                 yff_tau: float = YFF_TAU_DEFAULT):
        slope = float(yaw_slope_rpm_per_us) if yaw_slope_rpm_per_us > 0 else 0.504
        self.yff_a = slope * 1000.0 * (2.0 * math.pi / 60.0)
        self.yff_max = float(yff_max)
        self.yff_tau = float(yff_tau)
        self.trim = 0.0

    def reset(self) -> None:
        self.trim = 0.0

    def step(self, dt: float, u: float, psi_dot: float) -> float:
        trim_target = u - psi_dot / self.yff_a
        trim_target = max(0.0, min(self.yff_max, trim_target))
        alpha = dt / (self.yff_tau + dt)
        self.trim += alpha * (trim_target - self.trim)
        self.trim = max(0.0, min(self.yff_max, self.trim))
        return self.trim


class AltitudeHoldController:
    """
    Elevation-holding cyclic controller.

    Converts a target altitude setpoint into a body_z_eq for the swashplate.
    The elevation angle is rate-limited so disk transitions are smooth.

    Stateless geometry — no orbit reference, no quaternion slerp.  The planner
    sets target_alt_m; the controller handles how fast to get there.

    Usage
    -----
        ctrl = AltitudeHoldController.from_pos(ic.pos, rotor.body_z_slew_rate_rad_s)
        # each physics step:
        body_z_eq = ctrl.update(pos, target_alt_m, tension_n, mass_kg, dt)
        # then feed body_z_eq into HeliCyclicController via compute_rate_cmd
    """

    def __init__(self, initial_el_rad: float,
                 slew_rate_rad_s: float,
                 az_ref_tau_s: float = AZ_REF_TAU_S) -> None:
        self._el       = float(initial_el_rad)
        self._slew     = float(slew_rate_rad_s)
        self._az_tau   = float(az_ref_tau_s)
        self._az_ref   = None   # lazily initialised from the first pos

    @classmethod
    def from_pos(cls, pos: np.ndarray,
                 slew_rate_rad_s: float) -> "AltitudeHoldController":
        """Initialise elevation from the hub's starting NED position."""
        tlen = float(np.linalg.norm(pos))
        el   = float(np.arcsin(max(-1.0, min(1.0, float(-pos[2]) / max(tlen, 0.1)))))
        return cls(el, slew_rate_rad_s)

    @property
    def elevation_rad(self) -> float:
        return self._el

    def update(self, pos: np.ndarray, target_alt_m: float,
               tension_n: float, mass_kg: float, dt: float,
               G: float = 9.81) -> np.ndarray:
        """
        Step the controller.  Returns body_z_eq (NED unit vector).

        Parameters
        ----------
        pos          : current hub NED position [m]
        target_alt_m : desired altitude above anchor [m]  (positive = up)
        tension_n    : current tether tension [N]
        mass_kg      : hub mass [kg]
        dt           : timestep [s]
        """
        tlen       = float(np.linalg.norm(pos))
        target_el  = float(np.arcsin(max(-1.0, min(1.0,
                          target_alt_m / max(tlen, 0.1)))))
        max_step   = self._slew * dt
        self._el  += max(-max_step, min(max_step, target_el - self._el))
        if self._az_ref is None:
            self._az_ref = float(np.arctan2(pos[1], pos[0]))
        else:
            self._az_ref = update_plane_azimuth(self._az_ref, pos, self._az_tau, dt)
        return compute_bz_altitude_hold(pos, self._el, tension_n, mass_kg, G,
                                        az_ref_rad=self._az_ref)


class ElevationHoldController:
    """
    Outer cyclic loop: altitude target → body-frame rate commands.

    Combines AltitudeHoldController (elevation rate-limiting + gravity-compensated
    body_z_eq) with compute_rate_cmd (body_z error → rate setpoint).  The output
    feeds directly into HeliCyclicController (or the ArduPilot ACRO rate PIDs on
    hardware), giving a clean two-level split:

        ElevationHoldController  →  (rate_roll_sp, rate_pitch_sp)
        HeliCyclicController       →  (tilt_lon, tilt_lat)

    This mirrors the rawes.lua outer-loop structure, but runtime stack control
    now uses GUIDED setpoints rather than RC override transport.

    Usage
    -----
        ctrl = ElevationHoldController.from_pos(ic.pos, slew_rate_rad_s, mass_kg)
        # each 400 Hz step:
        rate_roll, rate_pitch = ctrl.update(pos, R, target_alt_m, tension_n, dt)
    """

    DEFAULT_KP_OUTER: float = 2.5

    def __init__(
        self,
        initial_el_rad : float,
        slew_rate_rad_s: float,
        mass_kg        : float,
        kp_outer       : float,
        az_ref_tau_s   : float = AZ_REF_TAU_S,
    ) -> None:
        self._el       = float(initial_el_rad)
        self._slew     = float(slew_rate_rad_s)
        self._mass_kg  = float(mass_kg)
        self._kp_outer = float(kp_outer)
        self._az_tau   = float(az_ref_tau_s)
        self._az_ref   = None   # lazily initialised from the first pos

    @classmethod
    def from_pos(
        cls,
        pos            : np.ndarray,
        slew_rate_rad_s: float,
        mass_kg        : float,
        kp_outer       : float,
    ) -> "ElevationHoldController":
        tlen = float(np.linalg.norm(pos))
        el   = float(np.arcsin(max(-1.0, min(1.0, float(-pos[2]) / max(tlen, 0.1)))))
        return cls(el, slew_rate_rad_s, mass_kg, kp_outer)

    @property
    def elevation_rad(self) -> float:
        return self._el

    def update(
        self,
        pos         : np.ndarray,
        R           : np.ndarray,
        target_alt_m: float,
        tension_n   : float,
        dt          : float,
        G           : float = 9.81,
    ) -> "tuple[float, float]":
        """
        Step the controller.  Returns (rate_roll_sp, rate_pitch_sp) in [rad/s].

        Parameters
        ----------
        pos          : current hub NED position [m]
        R            : current hub rotation matrix (3×3, body→world)
        target_alt_m : desired altitude above anchor [m]
        tension_n    : current tether tension [N]
        dt           : timestep [s]
        """
        tlen      = float(np.linalg.norm(pos))
        target_el = float(np.arcsin(max(-1.0, min(1.0,
                         target_alt_m / max(tlen, 0.1)))))
        max_step  = self._slew * dt
        self._el += max(-max_step, min(max_step, target_el - self._el))

        if self._az_ref is None:
            self._az_ref = float(np.arctan2(pos[1], pos[0]))
        else:
            self._az_ref = update_plane_azimuth(self._az_ref, pos, self._az_tau, dt)

        R        = np.asarray(R, dtype=float)
        bz_goal  = compute_bz_altitude_hold(pos, self._el, tension_n, self._mass_kg, G,
                                            az_ref_rad=self._az_ref)
        rate_sp  = compute_rate_cmd(R[:, 2], bz_goal, R, kp=self._kp_outer, kd=0.0)
        return float(rate_sp[0]), float(rate_sp[1])


# ---------------------------------------------------------------------------
# Heli cyclic controller — ArduPilot-compatible rate loop + servo model
# ---------------------------------------------------------------------------


class HeliCyclicController:
    """ArduPilot-compatible rate / cyclic controller wrapping the
    ``simulation.arduloop`` package.

    The rate PID, target/error/derivative low-pass filters, optional
    notch slots, FF and D_FF terms, and the swashplate phase rotation
    all use ArduPilot's traditional-heli structure 1:1, so any gain
    tuned in simulation transfers directly to ATC_RAT_RLL_* /
    ATC_RAT_PIT_* / H_SW_H3_PHANG on hardware.

    The ``SwashplateServoModel`` (DS113MG slew + 3-servo H3-120 coupling)
    is applied to the controller output so the closed-loop bandwidth in
    sim matches the bandwidth limited by the actual servos.

    This class is thrust-first. Callers provide thrust in ``[0..1]`` via
    ``step_from_thrust()`` and conversion to physical collective radians is
    applied once at the swashplate/physics boundary.

    Usage::

        acro = HeliCyclicController(rotor)
        tilt_lon, tilt_lat, col_actual = acro.step_from_thrust(
            thrust_cmd,
            rate_roll_sp, rate_pitch_sp, omega_body, dt,
        )

    Parameters mapped to ArduPilot:
        Default (no explicit gains): roll loads ATC_RAT_RLL_* and pitch loads
        ATC_RAT_PIT_* from the same copter-heli.parm + rawes_sitl_defaults.parm
        chain as SITL. Passing P/I/D/... explicitly overrides both axes with
        those gains.
        P, I, D, FF, IMAX, FLTT, FLTE, FLTD  ↔  ATC_RAT_xxx_P/I/D/FF/IMAX/FLTT/FLTE/FLTD
        h_sw_h3_phang                        ↔  H_SW_H3_PHANG
    """

    def __init__(
        self,
        rotor,
        P:              float = 0.67,
        I:              float = 0.15,
        D:              float = 0.02,
        FF:             float = 0.00,
        IMAX:           float = 0.30,
        FLTT:           float = 40.0,
        FLTE:           float = 0.0,
        FLTD:           float = 40.0,
        h_sw_h3_phang:  float = 0.0,
        loop_rate_hz:   float = 400.0,
    ) -> None:
        _ap = _load_ap_params()
        roll_cfg  = RateAxisParams.from_ap_dict(_ap, "RLL")
        pitch_cfg = RateAxisParams.from_ap_dict(_ap, "PIT")
        # Override both axes with caller's params if provided (any deviation from
        # the default signature builds an explicit RateAxisParams applied to roll
        # and pitch alike). When no override is given, roll loads ATC_RAT_RLL_*
        # and pitch loads ATC_RAT_PIT_* from the same .parm chain as SITL.
        if P != 0.67 or I != 0.15 or D != 0.02 or FF != 0.00 or IMAX != 0.30 or FLTT != 40.0 or FLTE != 0.0 or FLTD != 40.0:
            override = RateAxisParams(
                P=P, I=I, D=D, FF=FF, IMAX=IMAX,
                FLTT=FLTT, FLTE=FLTE, FLTD=FLTD,
            )
            roll_cfg = override
            pitch_cfg = override
        yaw_cfg = RateAxisParams.from_ap_dict(_ap, "YAW")
        params = HeliParams(
            roll=roll_cfg, pitch=pitch_cfg, yaw=yaw_cfg,
            loop_rate_hz=loop_rate_hz, H_SW_H3_PHANG=h_sw_h3_phang
        )
        self._ctrl  = HeliRateController(params)
        self._servo = SwashplateServoModel.from_rotor(rotor)
        self._col_min, self._col_max = _load_col_range()
        self._tilt_lon_trim = 0.0
        self._tilt_lat_trim = 0.0

    def set_trim(self, tilt_lon: float, tilt_lat: float) -> None:
        """Static cyclic feedforward added to the controller output.

        Equivalent to pre-loading the PID integrators with the
        steady-state cyclic from ``aero.solve_trim_cyclic`` — the inner
        loop then only handles small perturbations around equilibrium.
        Optional; arduloop's integral term reaches the same trim through
        feedback, but the FF makes the warmup transient cleaner.
        """
        self._tilt_lon_trim = float(tilt_lon)
        self._tilt_lat_trim = float(tilt_lat)

    def reset_from_thrust(
        self,
        thrust_cmd: float,
        *,
        tilt_lon: float = 0.0,
        tilt_lat: float = 0.0,
    ) -> None:
        """Reset servo state using thrust [0..1] at the physics boundary."""
        thrust = float(np.clip(thrust_cmd, 0.0, 1.0))
        col_rad = self._col_min + thrust * (self._col_max - self._col_min)
        self._servo.reset(col_rad, tilt_lon=float(tilt_lon), tilt_lat=float(tilt_lat))

    @property
    def yaw_cmd(self) -> float:
        """Last tail-rotor/yaw-axis command from the rate PID (arduloop
        HeliRateOutput.yaw_cmd, [-1, 1]).  Callers wanting a GB4008 motor
        throttle should clamp to [0, 1] (the physical motor cannot reverse;
        this mirrors ArduPilot's SERVO_TRIM=SERVO_MIN convention for the
        one-directional Motor4 output)."""
        return float(self._ctrl.out.yaw_cmd)

    def _step_collective(
        self,
        collective_cmd: float,
        rate_roll_sp  : float,
        rate_pitch_sp : float,
        omega_body    : np.ndarray,
        dt            : float,
        collective_norm: float = 0.0,
    ) -> "tuple[float, float, float]":
        """Advance one timestep from physical collective [rad].

        Returns ``(tilt_lon, tilt_lat, col_actual)``.  All three channels
        pass through the SwashplateServoModel so collective and cyclic
        share the same physical servos and slew limits.

        Sign mapping between arduloop and windpower's FRD aero (empirically
        verified — see tests/oneoff/arduloop_vs_acro.py):

          * arduloop ``roll_cyclic``  → ``tilt_lat``   (positive roll-right
            command → positive +M_body_x in the aero)
          * arduloop ``pitch_cyclic`` → ``−tilt_lon``  (positive pitch-up
            command negates because ``tilt_lon>0`` produces ``M_body[1]<0``
            in the new aero's convention, i.e. nose-down)
        """
        omega_body = np.asarray(omega_body, dtype=float)
        out = self._ctrl.update(
            rate_target_rads=(float(rate_roll_sp), float(rate_pitch_sp), 0.0),
            gyro_rate_rads  =(float(omega_body[0]), float(omega_body[1]),
                              float(omega_body[2])),
            dt              =float(dt),
            collective_norm =float(collective_norm),
        )
        tilt_lat_cmd =  out.roll_cyclic  + self._tilt_lat_trim
        tilt_lon_cmd = -out.pitch_cyclic + self._tilt_lon_trim
        col_act, tlon, tlat = self._servo.step(
            float(collective_cmd), tilt_lon_cmd, tilt_lat_cmd, dt)
        return float(tlon), float(tlat), float(col_act)

    def step_from_thrust(
        self,
        thrust_cmd    : float,
        rate_roll_sp  : float,
        rate_pitch_sp : float,
        omega_body    : np.ndarray,
        dt            : float,
        collective_norm: float | None = None,
    ) -> "tuple[float, float, float]":
        """Advance one timestep using thrust [0..1] as input.

        This is the preferred API for guided-stack integration because it matches
        ArduPilot's throttle/thrust-facing interface. Conversion to physical
        collective radians is done exactly once at this boundary.
        """
        thrust = float(np.clip(thrust_cmd, 0.0, 1.0))
        col_rad = self._col_min + thrust * (self._col_max - self._col_min)
        c_norm = float(2.0 * thrust - 1.0) if collective_norm is None else float(collective_norm)
        tlon, tlat, col_act = self._step_collective(
            collective_cmd=col_rad,
            rate_roll_sp=rate_roll_sp,
            rate_pitch_sp=rate_pitch_sp,
            omega_body=omega_body,
            dt=dt,
            collective_norm=c_norm,
        )
        return float(tlon), float(tlat), float(col_act)

# ---------------------------------------------------------------------------
# Aero-model utilities
# ---------------------------------------------------------------------------

def col_min_for_altitude_rad(
    aero,
    xi_deg:        float,
    mass_kg:       float,
    omega:         float,
    wind_m_s:      float = 10.0,
    safety_rad:    float = 0.01,
) -> float:
    """
    Minimum collective [rad] that keeps Fz ≥ mass·g at tilt angle xi from wind.

    Binary searches for the collective where the vertical aerodynamic force
    equals the hub weight, then adds a small safety margin.  Used to set
    ``col_min_reel_in_rad`` in the De Schutter planner so the hub stays aloft
    during high-tilt reel-in without explicit altitude feedback.

    Parameters
    ----------
    aero      : SkewedWakeBEM (or any aero model with compute_forces)
    xi_deg    : disk tilt from wind direction [°]  (0°=into wind, 90°=vertical)
    mass_kg   : hub mass [kg]
    wind_m_s  : wind speed [m/s]
    omega     : rotor spin rate [rad/s]
    safety_rad: margin added above the exact floor [rad]
    """
    xi_r = math.radians(xi_deg)
    # bz in NED: East = Y axis.  xi from East direction toward Down (negative NED Z = Up).
    bz   = np.array([0.0, math.cos(xi_r), -math.sin(xi_r)])
    R    = build_orb_frame(bz)
    wind = np.array([0.0, wind_m_s, 0.0])   # NED: East wind = Y axis
    W    = mass_kg * 9.81

    def _thrust_at(col: float) -> float:
        inputs = RotorInputs(
            collective_rad=col, tilt_lon=0.0, tilt_lat=0.0,
            R_hub=R, v_hub_world=np.zeros(3), wind_world=wind,
            omega_rad_s=float(omega), rho_kg_m3=1.225,
        )
        # Fresh state per probe: the aero step() API settles its inflow/wake
        # state on the first call, so one step from a clean state yields the
        # steady-state thrust at this collective.  Threading a state carried
        # over from a different collective would defeat the auto-settle.
        result, _ = aero.step(inputs, aero.initial_rotor_state(), 0.0025)
        # In NED, upward force is negative Z.
        return float(-result.F_world[2])

    lo, hi = -0.35, 0.20
    for _ in range(50):
        mid = (lo + hi) / 2.0
        if _thrust_at(mid) > W:
            hi = mid
        else:
            lo = mid

    return (lo + hi) / 2.0 + safety_rad


# ---------------------------------------------------------------------------
# Portable core — frame-agnostic functions that map 1:1 to Lua/C++ Mode_RAWES.
#
# Rules:
#   • No ArduPilot API calls, no side effects, no global state.
#   • Frame-agnostic: callers pass NED or ENU; the functions work identically
#     in either frame.  The simulation uses NED; firmware uses NED.
#   • These three functions are the entire on-board algorithm; everything
#     else (reading sensors, sending outputs) is platform glue.
# ---------------------------------------------------------------------------

def compute_bz_tether(
    pos:    np.ndarray,
    anchor: np.ndarray,
) -> "np.ndarray | None":
    """
    Equilibrium body_z (FRD: hub axis pointing DOWN through the disk).

    In tethered flight, the tether pulls the bottom of the axle toward the
    anchor, so the rotor's down-axis points from hub toward anchor.  This
    returns the unit vector (anchor − hub), matching the FRD convention
    used end-to-end with the aero package and ArduPilot's EKF.

    Returns None when the hub is at or inside the anchor (degenerate).
    Frame-agnostic: pass ENU or NED; the returned vector is in the same frame.
    """
    delta = np.asarray(anchor, dtype=float) - np.asarray(pos, dtype=float)
    d_len = float(np.linalg.norm(delta))
    if d_len < 0.1:
        return None
    return delta / d_len


def slerp_body_z(
    bz_prev:         np.ndarray,
    bz_target:       np.ndarray,
    slew_rate_rad_s: float,
    dt:              float,
) -> np.ndarray:
    """
    Rate-limited spherical interpolation between two unit vectors.

    Advances bz_prev toward bz_target by at most ``slew_rate_rad_s * dt``
    radians per call.  Returns a copy of bz_target when already within 1 µrad.
    Frame-agnostic.
    """
    bz_prev   = np.asarray(bz_prev,   dtype=float)
    bz_target = np.asarray(bz_target, dtype=float)
    cos_theta = float(np.clip(np.dot(bz_prev, bz_target), -1.0, 1.0))
    theta     = float(np.arccos(cos_theta))
    if theta < 1e-6:
        return bz_target.copy()
    alpha     = min(1.0, float(slew_rate_rad_s) * float(dt) / theta)
    sin_theta = np.sin(theta)
    result    = (np.sin((1.0 - alpha) * theta) * bz_prev
                 + np.sin(alpha * theta) * bz_target) / sin_theta
    return result / np.linalg.norm(result)


def compute_rate_cmd(
    bz_now:          np.ndarray,
    bz_eq:           np.ndarray,
    R_body_to_world: np.ndarray,
    kp:              float,
    kd:              float = 0.0,
    omega_world:     "np.ndarray | None" = None,
) -> np.ndarray:
    """
    Body_z alignment error → body-frame angular rate command.

    Computes the rotation needed to align bz_now with bz_eq, then projects it
    into the body frame as a rate command for an ACRO-style rate controller.

    Parameters
    ----------
    bz_now          : current rotor axle unit vector (world frame)
    bz_eq           : desired rotor axle unit vector (world frame)
    R_body_to_world : rotation matrix; columns are body axes in world frame
    kp              : proportional gain [rad/s per rad]
    kd              : derivative damping gain on orbital rate [dimensionless];
                      ignored when omega_world is None
    omega_world     : angular velocity in world frame [rad/s]; needed for kd > 0

    Returns
    -------
    np.ndarray (3,) — (roll_rate, pitch_rate, yaw_rate) in body frame [rad/s]

    Frame-agnostic: pass ENU or NED consistently across all arguments.
    On hardware (Lua/C++) kd=0 because ArduPilot's rate PIDs supply damping.
    In simulation kd > 0 supplements the absent firmware rate loop.
    """
    bz_now  = np.asarray(bz_now,          dtype=float)
    bz_eq   = np.asarray(bz_eq,           dtype=float)
    R       = np.asarray(R_body_to_world, dtype=float)

    error_world   = cross3(bz_now, bz_eq)

    damping_world = np.zeros(3)
    if kd != 0.0 and omega_world is not None:
        omega         = np.asarray(omega_world, dtype=float)
        omega_spin    = np.dot(omega, bz_now) * bz_now
        omega_orbital = omega - omega_spin
        damping_world = kd * omega_orbital

    return R.T @ (kp * error_world - damping_world)


def _sqrt_rate_from_error(error: float, kp: float, accel_max: float, dt: float) -> float:
    """ArduPilot-style sqrt controller: angle error -> rate command."""
    error = float(error)
    kp = float(kp)
    accel_max = float(accel_max)
    dt = float(dt)

    if accel_max <= 0.0:
        rate = error * kp
    elif kp == 0.0:
        rate = math.copysign(math.sqrt(2.0 * accel_max * abs(error)), error) if error != 0.0 else 0.0
    else:
        linear_dist = accel_max / (kp * kp)
        if error > linear_dist:
            rate = math.sqrt(2.0 * accel_max * (error - linear_dist / 2.0))
        elif error < -linear_dist:
            rate = -math.sqrt(2.0 * accel_max * (-error - linear_dist / 2.0))
        else:
            rate = error * kp

    if dt > 0.0:
        rate = max(-abs(error) / dt, min(abs(error) / dt, rate))
    return rate


def compute_rate_cmd_sqrt(
    bz_now:          np.ndarray,
    bz_eq:           np.ndarray,
    R_body_to_world: np.ndarray,
    kp:              float,
    accel_max:       float,
    dt:              float,
    kd:              float = 0.0,
    omega_world:     "np.ndarray | None" = None,
) -> np.ndarray:
    """
    Body_z alignment error -> sqrt-shaped body-frame angular rate command.

    This is the rate-only equivalent of ArduPilot's angle-error shaping: the
    error axis comes from ``bz_now x bz_eq`` and the error magnitude is shaped
    by a sqrt controller so large target changes respect an angular acceleration
    limit.  Small errors remain linear P.
    """
    bz_now = np.asarray(bz_now, dtype=float)
    bz_eq = np.asarray(bz_eq, dtype=float)
    R = np.asarray(R_body_to_world, dtype=float)

    cross_world = cross3(bz_now, bz_eq)
    cross_norm = float(np.linalg.norm(cross_world))
    dot = float(np.clip(np.dot(bz_now, bz_eq), -1.0, 1.0))
    angle = float(math.atan2(cross_norm, dot))

    if cross_norm > 1e-12 and angle > 1e-12:
        axis_world = cross_world / cross_norm
        proportional_world = axis_world * _sqrt_rate_from_error(angle, kp, accel_max, dt)
    else:
        proportional_world = np.zeros(3)

    damping_world = np.zeros(3)
    if kd != 0.0 and omega_world is not None:
        omega = np.asarray(omega_world, dtype=float)
        omega_spin = np.dot(omega, bz_now) * bz_now
        omega_orbital = omega - omega_spin
        damping_world = kd * omega_orbital

    return R.T @ (proportional_world - damping_world)

