"""
controller.py — RAWES attitude rate controllers for ArduPilot ACRO mode.

The mediator reports actual orbital-frame orientation (~65° from NED vertical
at tether equilibrium).  PhysicalHoldController derives the tether-alignment
error from the equilibrium captured during kinematic startup and uses
compute_rc_from_attitude to send corrective ACRO RC overrides.
GPS + compass are fused normally.

Usage
-----
controller = make_hold_controller(anchor_ned=anchor_ned)
controller.send_correction(att, pos_ned, gcs)   # in the hold loop
"""

import math

import numpy as np
from frames     import build_orb_frame, cross3  # noqa: F401 — build_orb_frame re-exported for callers
from servo_pwm  import SWASH_PWM_NEUTRAL, SWASH_PWM_RANGE, INTERLOCK_PWM_HIGH
from swashplate import SwashplateServoModel


def compute_rc_rates(
    hub_state:    dict,
    anchor_pos:   np.ndarray,
    vel_ned:      np.ndarray,
    kp:           float = 0.5,
    kd:           float = 0.2,
    rate_max_deg: float = 200.0,
) -> dict:
    """
    Compute RC override channels to keep body_z aligned with the tether.

    Parameters
    ----------
    hub_state : dict
        Current rigid-body state with keys:
          ``pos``   — NED position (3,)
          ``R``     — rotation matrix body→world (3, 3)
          ``omega`` — angular velocity in world NED frame (3,)
    anchor_pos : array-like (3,)
        Tether anchor position in NED world frame.
    vel_ned : array-like (3,)
        Hub velocity in NED frame (used to derive heading yaw for body-frame
        alignment with ArduPilot's ACRO controller).
    kp : float
        Proportional gain on attitude error (rad/s per rad).
    kd : float
        Derivative gain on orbital angular rate (dimensionless).
    rate_max_deg : float
        Angular rate corresponding to full stick deflection (degrees/s).

    Returns
    -------
    dict
        RC channel overrides: {1: pwm, 2: pwm, 4: pwm, 8: 2000}.
        Channel 1 = roll rate, 2 = pitch rate, 4 = yaw rate, 8 = motor interlock.
        Neutral = 1500, min = 1000, max = 2000.
    """
    pos   = np.asarray(hub_state["pos"],   dtype=float)
    R     = np.asarray(hub_state["R"],     dtype=float)
    omega = np.asarray(hub_state["omega"], dtype=float)
    anch  = np.asarray(anchor_pos,         dtype=float)

    # Safety: if hub is at/inside anchor, return neutral
    tether = pos - anch
    t_len  = float(np.linalg.norm(tether))
    if t_len < 0.1:
        return {1: SWASH_PWM_NEUTRAL, 2: SWASH_PWM_NEUTRAL, 4: SWASH_PWM_NEUTRAL, 8: INTERLOCK_PWM_HIGH}

    # Desired body_z = tether direction (pointing away from anchor)
    body_z_eq  = tether / t_len

    # Actual body_z = third column of rotation matrix
    body_z_cur = R[:, 2]

    # Attitude error in world NED frame (cross product → rotation axis toward target)
    error_world = cross3(body_z_cur, body_z_eq)

    # Strip spin component from omega (spin is along body_z, not an orbital rate)
    omega_spin    = np.dot(omega, body_z_cur) * body_z_cur
    omega_orbital = omega - omega_spin

    # Rate correction in NED world frame: P term + D term
    omega_corr = kp * error_world - kd * omega_orbital

    # Rotate into ArduPilot's yaw-aligned body frame
    vel_ned  = np.asarray(vel_ned, dtype=float)
    v_horiz  = float(np.hypot(vel_ned[0], vel_ned[1]))
    yaw      = float(np.arctan2(vel_ned[1], vel_ned[0])) if v_horiz > 0.1 else 0.0
    cy, sy   = np.cos(yaw), np.sin(yaw)
    Rz_inv   = np.array([[cy, sy, 0.], [-sy, cy, 0.], [0., 0., 1.]])
    omega_body = Rz_inv @ omega_corr

    # Map rad/s to PWM [1000, 2000], 1500 = zero
    max_rate = np.radians(rate_max_deg)

    def _pwm(w: float) -> int:
        return int(round(SWASH_PWM_NEUTRAL + SWASH_PWM_RANGE * float(np.clip(w / max_rate, -1.0, 1.0))))

    return {
        1: _pwm(omega_body[0]),   # roll rate
        2: _pwm(omega_body[1]),   # pitch rate
        4: _pwm(omega_body[2]),   # yaw rate
        8: INTERLOCK_PWM_HIGH,    # motor interlock always on
    }


def compute_swashplate_from_state(
    hub_state:            dict,
    anchor_pos:           np.ndarray,
    kp:                   float = 0.5,
    kd:                   float = 0.2,
    tilt_max_rad:         float = 0.3,
    body_z_eq:            "np.ndarray | None" = None,
    swashplate_phase_deg: float = 0.0,
) -> dict:
    """
    Compute swashplate tilt commands directly from hub truth state.

    This bypasses ArduPilot entirely — the mediator can call this directly
    and feed tilt_lon/tilt_lat into aero.compute_forces().

    Parameters
    ----------
    hub_state   : dict with pos (NED), R (body→world), omega (world NED)
    anchor_pos  : tether anchor in NED [m]
    kp          : proportional gain on attitude error [rad/rad]
    kd          : derivative gain on orbital rate [rad·s/rad]
    tilt_max_rad: maximum swashplate tilt [rad] (saturation limit)
    body_z_eq   : optional equilibrium body-z override (unit vector, NED).
                  When None (default), body_z_eq is computed as the tether
                  direction (pos - anchor) / |pos - anchor|.  Pass the hub's
                  current R[:,2] to hold the present orientation with zero
                  error, or a blended vector to ramp in tether-alignment
                  gradually after a mode transition.

    Returns
    -------
    dict: {collective_rad: float, tilt_lon: float, tilt_lat: float}
          tilt_lon/tilt_lat in [-1, 1] (normalised swashplate coordinates)
    """
    pos   = np.asarray(hub_state["pos"],   dtype=float)
    R     = np.asarray(hub_state["R"],     dtype=float)
    omega = np.asarray(hub_state["omega"], dtype=float)
    anch  = np.asarray(anchor_pos,         dtype=float)

    # Safety: if hub is at/inside anchor, return neutral
    tether = pos - anch
    t_len  = float(np.linalg.norm(tether))
    if t_len < 0.1:
        return {"collective_rad": 0.0, "tilt_lon": 0.0, "tilt_lat": 0.0}

    if body_z_eq is not None:
        body_z_eq = np.asarray(body_z_eq, dtype=float)
        n = float(np.linalg.norm(body_z_eq))
        body_z_eq = body_z_eq / n if n > 1e-6 else tether / t_len
    else:
        body_z_eq = tether / t_len
    body_z_cur = R[:, 2]

    error_world = cross3(body_z_cur, body_z_eq)

    omega_spin    = np.dot(omega, body_z_cur) * body_z_cur
    omega_orbital = omega - omega_spin

    # Correction in world NED frame
    corr_ned = kp * error_world - kd * omega_orbital

    # Project onto disk-plane axes (tilt_lon = along disk X, tilt_lat = along disk Y)
    disk_x = R[:, 0]
    disk_y = R[:, 1]

    # Sign convention — per-blade physics (SkewedWakeBEM / DeSchutterAero):
    #
    #   tilt_lat_rad > 0 → max pitch at ψ=0 (East/disk_x blade position)
    #     moment arm = r·disk_x,  lift = F·disk_z
    #     M = cross(r·disk_x, F·disk_z) = r·F·(disk_x × disk_z) = −r·F·disk_y
    #     → moment in −disk_y  (NEGATIVE My for positive tilt_lat)
    #
    #   tilt_lon_rad > 0 → max pitch at ψ=90° (North/disk_y blade position)
    #     moment arm = r·disk_y,  lift = F·disk_z
    #     M = cross(r·disk_y, F·disk_z) = r·F·(disk_y × disk_z) = +r·F·disk_x
    #     → moment in +disk_x  (POSITIVE Mx for positive tilt_lon)
    #
    # corr_ned is the required rotation axis (= direction to apply moment).
    # To get M in +disk_x: need tilt_lon > 0 → tilt_lon = +dot(corr_ned, disk_x)
    # To get M in +disk_y: need tilt_lat < 0 → tilt_lat = −dot(corr_ned, disk_y)
    tilt_lon = float(np.clip( np.dot(corr_ned, disk_x) / tilt_max_rad, -1.0, 1.0))
    tilt_lat = float(np.clip(-np.dot(corr_ned, disk_y) / tilt_max_rad, -1.0, 1.0))

    # Gyroscopic phase compensation.
    # A spinning rotor precesses 90° off-axis from an applied cyclic torque.
    # Rotating the command by swashplate_phase_deg advances it so the aerodynamic
    # response lands in the intended direction.  0° = no compensation (I_spin = 0).
    if swashplate_phase_deg != 0.0:
        phi = math.radians(swashplate_phase_deg)
        c, s = math.cos(phi), math.sin(phi)
        tilt_lon, tilt_lat = (
            c * tilt_lon - s * tilt_lat,
            s * tilt_lon + c * tilt_lat,
        )

    return {
        "collective_rad": 0.0,   # collective held at zero; attitude via tilt only
        "tilt_lon":       tilt_lon,
        "tilt_lat":       tilt_lat,
    }


def compute_rc_from_attitude(
    roll:         float,
    pitch:        float,
    rollspeed:    float,
    pitchspeed:   float,
    yawspeed:     float,
    kp:           float = 1.0,
    kd:           float = 0.3,
    rate_max_deg: float = 200.0,
) -> dict:
    """
    Map attitude error + body rates to ACRO RC PWM overrides.

    Generic error-to-PWM converter used by PhysicalHoldController, which
    pre-computes roll/pitch as deviations from the captured equilibrium
    (att["roll"] − roll_eq, att["pitch"] − pitch_eq) before calling here.

    Commands:
        cmd_roll  = -kp * roll  - kd * rollspeed
        cmd_pitch = -kp * pitch - kd * pitchspeed
        cmd_yaw   =             - kd * yawspeed

    Parameters
    ----------
    roll, pitch           : attitude error from equilibrium [rad]
    rollspeed, pitchspeed, yawspeed : body-frame angular rates [rad/s]
    kp                    : attitude error gain [rad/s per rad]
    kd                    : rate damping gain [dimensionless]
    rate_max_deg          : full-stick rate [deg/s] → maps to PWM 1000/2000

    Returns
    -------
    dict : {1: pwm, 2: pwm, 4: pwm, 8: 2000}
    """
    max_rate = np.radians(rate_max_deg)

    def _pwm(w: float) -> int:
        return int(round(SWASH_PWM_NEUTRAL + SWASH_PWM_RANGE * float(np.clip(w / max_rate, -1.0, 1.0))))

    cmd_roll  = -kp * float(roll)  - kd * float(rollspeed)
    cmd_pitch = -kp * float(pitch) - kd * float(pitchspeed)
    cmd_yaw   =                    - kd * float(yawspeed)

    return {
        1: _pwm(cmd_roll),
        2: _pwm(cmd_pitch),
        4: _pwm(cmd_yaw),
        8: INTERLOCK_PWM_HIGH,
    }


def compute_rc_from_physical_attitude(
    roll:         float,
    pitch:        float,
    yaw:          float,
    rollspeed:    float,
    pitchspeed:   float,
    yawspeed:     float,
    pos_ned:      np.ndarray,   # hub NED position [m] from LOCAL_POSITION_NED
    anchor_ned:   np.ndarray,   # tether anchor NED position [m] (relative to same origin)
    kp:           float = 1.0,
    kd:           float = 0.3,
    rate_max_deg: float = 200.0,
) -> dict:
    """
    Compute RC override channels from physical attitude + position.

    Used with ``PhysicalSensor`` where ATTITUDE.roll/pitch are the actual NED
    Euler angles of the orbital frame (~65° from vertical at tether
    equilibrium), NOT tether-relative deviations.

    Derives the tether-alignment error directly from:
      - The physical body_z direction (column 2 of R_body built from rpy)
      - The tether equilibrium direction (pos_ned − anchor_ned, normalised)

    The PD correction is computed in the physical NED body frame and mapped
    to PWM the same way as ``compute_rc_from_attitude``.

    Parameters
    ----------
    roll, pitch, yaw      : actual NED Euler angles [rad] from ATTITUDE message
    rollspeed, pitchspeed, yawspeed : NED body-frame angular rates [rad/s]
    pos_ned               : hub position in NED [m]
    anchor_ned            : tether anchor in NED [m] (same LOCAL frame origin)
    kp, kd, rate_max_deg  : same meaning as compute_rc_from_attitude

    Returns
    -------
    dict : {1: pwm, 2: pwm, 4: pwm, 8: 2000}
    """
    max_rate = np.radians(rate_max_deg)

    def _pwm(w: float) -> int:
        return int(round(SWASH_PWM_NEUTRAL + SWASH_PWM_RANGE * float(np.clip(w / max_rate, -1.0, 1.0))))

    # Body→NED rotation matrix from ZYX Euler angles.
    # Column j = j-th body axis expressed in NED.
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    R_body = np.array([
        [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,     cp*sr,             cp*cr            ],
    ])

    # Actual rotor axle (body Z) in NED
    body_z_ned = R_body[:, 2]

    # Tether equilibrium direction: anchor → hub, normalised, in NED
    tether_ned = np.asarray(pos_ned, dtype=float) - np.asarray(anchor_ned, dtype=float)
    t_len = float(np.linalg.norm(tether_ned))
    if t_len < 0.1:
        return {1: SWASH_PWM_NEUTRAL, 2: SWASH_PWM_NEUTRAL, 4: SWASH_PWM_NEUTRAL, 8: INTERLOCK_PWM_HIGH}
    body_z_eq = tether_ned / t_len

    # Attitude error: rotation axis needed to align body_z with body_z_eq.
    # cross(a, b) = |a||b|sin(θ) n̂ — small when already aligned.
    error_ned = cross3(body_z_ned, body_z_eq)

    # Express error in NED body frame (NED→body = R_body.T)
    error_body = R_body.T @ error_ned

    # Angular velocity in body frame from ATTITUDE message
    omega_body = np.array([rollspeed, pitchspeed, yawspeed])

    # Strip spin component along body_z (electronics don't spin)
    omega_spin    = np.dot(omega_body, body_z_ned) * body_z_ned
    omega_orbital = omega_body - omega_spin

    # PD rate correction in body frame
    omega_corr = kp * error_body - kd * omega_orbital

    return {
        1: _pwm(omega_corr[0]),   # roll rate
        2: _pwm(omega_corr[1]),   # pitch rate
        4: _pwm(omega_corr[2]),   # yaw rate
        8: INTERLOCK_PWM_HIGH,
    }


# ---------------------------------------------------------------------------
# Hold controller abstraction — used by test_guided_flight.py
# ---------------------------------------------------------------------------



class PhysicalHoldController:
    """
    ACRO hold controller for physical sensor mode.

    ATTITUDE.roll/pitch are actual NED Euler angles.  Error is computed as
    deviation from the tether equilibrium orientation (roll_eq, pitch_eq)
    captured during the kinematic startup.  Uses compute_rc_from_attitude so
    the correction is yaw-independent — the velocity-derived yaw jumps ~150°
    when the tether activates at kinematic end, making any yaw-dependent error
    computation destabilising.

    Parameters
    ----------
    anchor_ned : array (3,)
        Tether anchor position in NED [m] (kept for divergence guard).

    Call ``set_equilibrium(roll_eq, pitch_eq)`` with values from the first
    clean ATTITUDE message before the controller is used.
    """

    extra_params: dict = {
        "ATC_RAT_RLL_IMAX": 0.0,
        "ATC_RAT_PIT_IMAX": 0.0,
        "ATC_RAT_YAW_IMAX": 0.0,
    }

    def __init__(self, anchor_ned: np.ndarray):
        self._anchor_ned = np.asarray(anchor_ned, dtype=float)
        self._roll_eq:  float | None = None
        self._pitch_eq: float | None = None

    def set_equilibrium(self, roll_eq: float, pitch_eq: float) -> None:
        """
        Record the tether equilibrium roll and pitch [rad] from the first
        clean ATTITUDE message during kinematic startup.  Must be called
        before send_correction is used.
        """
        self._roll_eq  = float(roll_eq)
        self._pitch_eq = float(pitch_eq)

    def send_correction(
        self,
        att:     dict,
        pos_ned: tuple | None,
        gcs,
    ) -> dict:
        """
        Compute and send RC override.  Returns the rc dict sent (for logging).

        Error = (roll - roll_eq, pitch - pitch_eq) — yaw-independent deviation
        from the tether equilibrium captured during kinematic startup.

        Sends neutral sticks when equilibrium has not been set yet.
        """
        _neutral = {1: SWASH_PWM_NEUTRAL, 2: SWASH_PWM_NEUTRAL,
                    3: SWASH_PWM_NEUTRAL, 4: SWASH_PWM_NEUTRAL, 8: INTERLOCK_PWM_HIGH}

        if self._roll_eq is None or self._pitch_eq is None:
            gcs.send_rc_override(_neutral)
            return _neutral

        roll_dev  = att["roll"]  - self._roll_eq
        pitch_dev = att["pitch"] - self._pitch_eq

        rc = compute_rc_from_attitude(
            roll       = roll_dev,
            pitch      = pitch_dev,
            rollspeed  = att["rollspeed"],
            pitchspeed = att["pitchspeed"],
            yawspeed   = att["yawspeed"],
        )
        rc[3] = SWASH_PWM_NEUTRAL   # neutral collective
        rc[8] = INTERLOCK_PWM_HIGH  # motor interlock on
        gcs.send_rc_override(rc)
        return rc


# ---------------------------------------------------------------------------
# Pumping cycle helpers — used by planner (planner.py) and unit tests
# ---------------------------------------------------------------------------

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


def orbit_tracked_body_z_eq(
    cur_pos:     np.ndarray,
    tether_dir0: np.ndarray,
    body_z_eq0:  np.ndarray,
) -> np.ndarray:
    """
    Rotate body_z_eq0 azimuthally to track the hub's orbital position.

    Projects both tether directions onto the horizontal plane and applies the
    resulting azimuthal (vertical-axis only) rotation to body_z_eq0.  The
    vertical (Z) component of body_z_eq0 is preserved exactly.

    Used by the Python simulation loop, which calls this at 400 Hz without
    rate limiting.  The altitude-insensitive Z-preservation prevents the
    positive-feedback altitude instability that would arise if the setpoint
    fully tracked tether elevation changes at high bandwidth.

    For the Lua-equivalent 3D algorithm (used on hardware, where a rate-limited
    slerp acts as the bandwidth limiter) see orbit_tracked_body_z_eq_3d().

    At t=0 (cur_pos = initial pos) returns body_z_eq0 unchanged → zero error
    and zero tilt at the natural equilibrium.
    """
    cur_tdir = cur_pos / np.linalg.norm(cur_pos)
    th0h = np.array([tether_dir0[0], tether_dir0[1], 0.0])
    thh  = np.array([cur_tdir[0],    cur_tdir[1],    0.0])
    n0h = np.linalg.norm(th0h); nhh = np.linalg.norm(thh)
    if n0h < 0.01 or nhh < 0.01:
        return body_z_eq0.copy()
    th0h /= n0h; thh /= nhh
    cos_phi = float(np.clip(np.dot(th0h, thh), -1.0, 1.0))
    sin_phi = float(th0h[0] * thh[1] - th0h[1] * thh[0])
    bz0    = body_z_eq0
    result = np.array([
        cos_phi * bz0[0] - sin_phi * bz0[1],
        sin_phi * bz0[0] + cos_phi * bz0[1],
        bz0[2],
    ])
    return result / np.linalg.norm(result)


def compute_bz_altitude_hold(
    pos:           np.ndarray,
    target_el_rad: float,
    tension_n:     float,
    mass_kg:       float,
    G:             float = 9.81,
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
    az        = float(np.arctan2(pos[1], pos[0]))
    cos_el    = float(np.cos(target_el_rad))
    sin_el    = float(np.sin(target_el_rad))
    cos_az    = float(np.cos(az))
    sin_az    = float(np.sin(az))

    tdir = np.array([cos_el * cos_az, cos_el * sin_az, -sin_el])
    e_up = np.array([-sin_el * cos_az, -sin_el * sin_az, -cos_el])

    # Gravity's tangential component in the elevation direction = mg·cos(el).
    # At low elevation this is nearly mg; at 90° (vertical) it is zero.
    g_tangential = mass_kg * G * float(np.cos(target_el_rad))
    raw = tdir + (g_tangential / max(tension_n, 1.0)) * e_up
    return raw / np.linalg.norm(raw)


class AltitudeHoldController:
    """
    Elevation-holding cyclic controller.

    Converts a target altitude setpoint into a body_z_eq for the swashplate.
    The elevation angle is rate-limited so disk transitions are smooth.

    Replaces OrbitTracker for altitude-holding flight — stateless geometry,
    no orbit reference, no quaternion slerp.  The planner sets target_alt_m;
    the controller handles how fast to get there.

    Usage
    -----
        ctrl = AltitudeHoldController.from_pos(ic.pos, rotor.body_z_slew_rate_rad_s)
        # each physics step:
        body_z_eq = ctrl.update(pos, target_alt_m, tension_n, mass_kg, dt)
        # then: compute_swashplate_from_state(state, omega, body_z_eq)
    """

    def __init__(self, initial_el_rad: float,
                 slew_rate_rad_s: float) -> None:
        self._el       = float(initial_el_rad)
        self._slew     = float(slew_rate_rad_s)

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
        return compute_bz_altitude_hold(pos, self._el, tension_n, mass_kg, G)


class ElevationHoldController:
    """
    Outer cyclic loop: altitude target → body-frame rate commands.

    Combines AltitudeHoldController (elevation rate-limiting + gravity-compensated
    body_z_eq) with compute_rate_cmd (body_z error → rate setpoint).  The output
    feeds directly into AcroControllerSitl (or the ArduPilot ACRO rate PIDs on
    hardware), giving a clean two-level split:

        ElevationHoldController  →  (rate_roll_sp, rate_pitch_sp)
        AcroControllerSitl       →  (tilt_lon, tilt_lat)

    This mirrors the rawes.lua architecture where the outer loop sends rate
    commands via RC override and the firmware ACRO PIDs close the inner loop.

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
    ) -> None:
        self._el       = float(initial_el_rad)
        self._slew     = float(slew_rate_rad_s)
        self._mass_kg  = float(mass_kg)
        self._kp_outer = float(kp_outer)

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

        R        = np.asarray(R, dtype=float)
        bz_goal  = compute_bz_altitude_hold(pos, self._el, tension_n, self._mass_kg, G)
        rate_sp  = compute_rate_cmd(R[:, 2], bz_goal, R, kp=self._kp_outer, kd=0.0)
        return float(rate_sp[0]), float(rate_sp[1])


def orbit_tracked_body_z_eq_3d(
    cur_pos:     np.ndarray,
    tether_dir0: np.ndarray,
    body_z_eq0:  np.ndarray,
) -> np.ndarray:
    """
    Rotate body_z_eq0 to track the hub's orbital position via full 3D Rodrigues.

    Applies the minimal 3D rotation that maps tether_dir0 to the current
    tether direction (cur_pos normalised).  Matches rawes.lua
    orbit_track() exactly.

    Use this function only when the output is fed through a rate-limited slerp
    before reaching the attitude controller — otherwise the altitude-sensitive
    Z component creates a positive-feedback instability (see orbit_tracked_body_z_eq
    docstring).  On hardware the Lua slerp (0.40 rad/s) provides this limiting.

    Frame-agnostic: pass NED or ENU consistently across all arguments.
    """
    cur_pos     = np.asarray(cur_pos,     dtype=float)
    tether_dir0 = np.asarray(tether_dir0, dtype=float)
    body_z_eq0  = np.asarray(body_z_eq0,  dtype=float)

    t_len = float(np.linalg.norm(cur_pos))
    if t_len < 0.01:
        return body_z_eq0.copy()
    bzt = cur_pos / t_len

    axis  = np.cross(tether_dir0, bzt)
    sinth = float(np.linalg.norm(axis))
    if sinth < 1e-6:
        return body_z_eq0.copy()
    costh  = float(np.dot(tether_dir0, bzt))
    angle  = float(np.arctan2(sinth, costh))
    axis_n = axis / sinth

    ca = float(np.cos(angle))
    sa = float(np.sin(angle))
    ad = float(np.dot(axis_n, body_z_eq0))
    return body_z_eq0 * ca + np.cross(axis_n, body_z_eq0) * sa + axis_n * (ad * (1.0 - ca))


class OrbitTracker:
    """
    Rate-limited orbit-tracking body_z setpoint.

    Mirrors the rawes.lua state machine exactly:
        _bz_orbit = orbit_track_3d(bz_eq0, tdir0, pos/|pos|)   each step
        _bz_slerp = slerp(_bz_slerp, bz_target or _bz_orbit, slew_rate, dt)

    Parameters
    ----------
    body_z_eq0      : body_z at equilibrium capture (NED unit vector)
    tether_dir0     : tether direction at capture (NED unit vector)
    slew_rate_rad_s : maximum setpoint rotation rate [rad/s]
                      Use rotor_definition.body_z_slew_rate_rad_s.

    Usage
    -----
        tracker = OrbitTracker(ic_body_z_eq0, ic_tether_dir0, rd.body_z_slew_rate_rad_s)
        # in loop — natural orbit:
        body_z_eq = tracker.update(pos, dt)
        # in loop — planner override (e.g. reel-in tilt):
        body_z_eq = tracker.update(pos, dt, bz_target=quat_apply(aq, [0,0,-1]))
    """

    def __init__(
        self,
        body_z_eq0:      np.ndarray,
        tether_dir0:     np.ndarray,
        slew_rate_rad_s: float,
    ) -> None:
        self._bz_slerp    = np.asarray(body_z_eq0,  dtype=float).copy()
        self._tether_dir0 = np.asarray(tether_dir0, dtype=float).copy()
        self._body_z_eq0  = np.asarray(body_z_eq0,  dtype=float).copy()
        self._slew_rate   = float(slew_rate_rad_s)

    def update(
        self,
        pos:       np.ndarray,
        dt:        float,
        bz_target: "np.ndarray | None" = None,
    ) -> np.ndarray:
        """
        Advance and return the rate-limited body_z setpoint.

        Parameters
        ----------
        pos       : hub position NED [m]; anchor assumed at origin
        dt        : timestep [s]
        bz_target : planner override unit vector NED; None = follow natural orbit

        Returns
        -------
        np.ndarray (3,) — rate-limited body_z setpoint (copy)
        """
        bz_orbit = orbit_tracked_body_z_eq_3d(pos, self._tether_dir0, self._body_z_eq0)
        goal     = np.asarray(bz_target, dtype=float) if bz_target is not None else bz_orbit
        self._bz_slerp = slerp_body_z(self._bz_slerp, goal, self._slew_rate, dt)
        return self._bz_slerp.copy()

    @property
    def bz_slerp(self) -> np.ndarray:
        """Current rate-limited setpoint (read-only copy)."""
        return self._bz_slerp.copy()


def blend_body_z(
    alpha:    float,
    bz_start: np.ndarray,
    bz_end:   np.ndarray,
) -> np.ndarray:
    """
    Linearly blend two unit vectors and renormalise.

    alpha=0 → bz_start, alpha=1 → bz_end.  Not true SLERP but accurate
    enough for smooth attitude transitions over several seconds.
    """
    alpha   = float(np.clip(alpha, 0.0, 1.0))
    blended = (1.0 - alpha) * np.asarray(bz_start, dtype=float) \
            + alpha          * np.asarray(bz_end,   dtype=float)
    n = np.linalg.norm(blended)
    return blended / n if n > 1e-6 else np.asarray(bz_end, dtype=float).copy()


# ---------------------------------------------------------------------------
# Simulated ACRO rate PID — mirrors ArduPilot AC_AttitudeControl inner loop.
# ---------------------------------------------------------------------------

class RatePID:
    """
    Single-axis rate PID: rate error (rad/s) → normalised tilt output [-1, 1].

    Mirrors ArduPilot's AC_AttitudeControl inner rate loop so the simulation
    uses the same two-loop architecture as the hardware:

        outer: compute_rate_cmd(kp_outer, kd=0) → rate setpoint
        inner: RatePID.update(setpoint, actual, dt) → normalised tilt

    On hardware, ArduPilot's rate PIDs supply all damping, so compute_rate_cmd
    is called with kd=0.  In simulation, RatePID provides the equivalent
    damping via its kp term (setpoint=0 at equilibrium → tilt = -kp * omega).

    Default gains are calibrated to match the legacy single-loop behaviour
    (kp=0.5, kd=0.2, tilt_max_rad=0.3) so existing tests pass unchanged:
        kp_inner * kp_outer = kp_old / tilt_max_rad  →  0.67 * 2.5 = 1.67
        kp_inner             = kd_old / tilt_max_rad  →  0.2  / 0.3 = 0.67

    ArduPilot parameter mapping (helicopter ACRO):
        kp    ↔  ATC_RAT_RLL_P  /  ATC_RAT_PIT_P
        ki    ↔  ATC_RAT_RLL_I  /  ATC_RAT_PIT_I   (set 0 in our config)
        kd    ↔  ATC_RAT_RLL_D  /  ATC_RAT_PIT_D
        imax  ↔  ATC_RAT_RLL_IMAX / ATC_RAT_PIT_IMAX (set 0 in our config)
    """

    # Gain calibrated to match legacy kd=0.2, tilt_max_rad=0.3 behaviour:
    #   kd_old / tilt_max_rad = 0.2 / 0.3 = 2/3
    DEFAULT_KP: float = 2.0 / 3.0

    def __init__(
        self,
        kp:         float,
        ki:         float,
        kd:         float,
        imax:       float,
        output_max: float,
    ):
        self.kp         = float(kp)
        self.ki         = float(ki)
        self.kd         = float(kd)
        self.imax       = float(imax)
        self.output_max = float(output_max)
        self._integral  = 0.0
        self._last_err  = 0.0

    def update(self, setpoint: float, actual: float, dt: float) -> float:
        """
        Advance the PID by one step.

        Parameters
        ----------
        setpoint : desired angular rate [rad/s]
        actual   : measured angular rate [rad/s]
        dt       : time step [s]

        Returns
        -------
        float — normalised tilt command in [-output_max, +output_max]
        """
        error  = float(setpoint) - float(actual)
        output = self.kp * error

        if self.ki != 0.0:
            self._integral += error * float(dt)
            if self.imax > 0.0:
                limit = self.imax / self.ki
                self._integral = float(np.clip(self._integral, -limit, limit))
            output += self.ki * self._integral

        if self.kd != 0.0 and dt > 0.0:
            output += self.kd * (error - self._last_err) / float(dt)

        self._last_err = error
        return float(np.clip(output, -self.output_max, self.output_max))

    def reset(self) -> None:
        """Reset integrator and derivative state (call on mode entry)."""
        self._integral = 0.0
        self._last_err = 0.0


# ---------------------------------------------------------------------------
# ACRO attitude controller emulator
# ---------------------------------------------------------------------------


class AcroControllerSitl:
    """
    Inner rate-loop emulator for ArduPilot ACRO mode.

    Accepts body-frame rate commands (as rawes.lua sends via RC override) and
    applies RatePID + SwashplateServoModel to produce slew-limited swashplate
    outputs, mirroring what ArduPilot's AC_AttitudeControl + DS113MG servos do.

    All three channels (collective + cyclic) pass through the shared servo model
    so that coupling and slew limits are correct.

    Usage::

        acro = AcroControllerSitl(rotor)
        tilt_lon, tilt_lat, col_actual = acro.step(
            collective_rad, rate_roll_sp, rate_pitch_sp, omega_body, dt)
    """

    def __init__(self, rotor,
                 col_min_rad: float,
                 col_max_rad: float,
                 kp: float = RatePID.DEFAULT_KP) -> None:
        self._pid_lon = RatePID(kp=kp, ki=0.0, kd=0.0, imax=0.0, output_max=1.0)
        self._pid_lat = RatePID(kp=kp, ki=0.0, kd=0.0, imax=0.0, output_max=1.0)
        self._servo   = SwashplateServoModel.from_rotor(
            rotor, col_min_rad=col_min_rad, col_max_rad=col_max_rad)

    def step(
        self,
        collective_cmd: float,
        rate_roll_sp  : float,
        rate_pitch_sp : float,
        omega_body    : np.ndarray,
        dt            : float,
    ) -> "tuple[float, float, float]":
        """
        Advance one timestep.

        All three channels pass through the SwashplateServoModel so collective
        and cyclic share the same 3 physical servos with coupled slew limits.

        Returns (tilt_lon, tilt_lat, col_actual) — use col_actual (not the
        commanded value) as the collective input to PhysicsCore.
        """
        omega_body   = np.asarray(omega_body, dtype=float)
        tilt_lon_cmd =  self._pid_lon.update(rate_roll_sp,  omega_body[0], dt)
        tilt_lat_cmd = -self._pid_lat.update(rate_pitch_sp, omega_body[1], dt)
        col_act, tlon, tlat = self._servo.step(
            float(collective_cmd), tilt_lon_cmd, tilt_lat_cmd, dt)
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
    import math as _math
    import numpy as _np
    from frames import build_orb_frame as _build_orb_frame

    xi_r = _math.radians(xi_deg)
    # bz in NED: East = Y axis.  xi from East direction toward Down (negative NED Z = Up).
    bz   = _np.array([0.0, _math.cos(xi_r), -_math.sin(xi_r)])
    R    = _build_orb_frame(bz)
    wind = _np.array([0.0, wind_m_s, 0.0])   # NED: East wind = Y axis
    W    = mass_kg * 9.81

    lo, hi = -0.35, 0.20
    for _ in range(50):
        mid = (lo + hi) / 2.0
        r   = aero.compute_forces(mid, 0.0, 0.0, R, _np.zeros(3), omega, wind, 50.0)
        # In NED, upward force is negative Z; hub stays aloft when -F_world[2] >= W
        if float(-r.F_world[2]) > W:
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
    Tether direction unit vector (anchor → hub).

    Returns None when the hub is at or inside the anchor (degenerate).
    Frame-agnostic: pass ENU or NED; the returned vector is in the same frame.
    """
    tether = np.asarray(pos, dtype=float) - np.asarray(anchor, dtype=float)
    t_len  = float(np.linalg.norm(tether))
    if t_len < 0.1:
        return None
    return tether / t_len


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


def make_hold_controller(
    anchor_ned: "np.ndarray",
) -> "PhysicalHoldController":
    """
    Return a PhysicalHoldController for the given anchor position.

    Parameters
    ----------
    anchor_ned : tether anchor in NED [m] relative to LOCAL_POSITION_NED origin.
    """
    return PhysicalHoldController(anchor_ned)


class AccelVibrationDamper:
    """
    Accelerometer-based tether spring-mode vibration damper.

    Uses body-Z specific force (the dominant axis of tether spring oscillation)
    to estimate oscillatory hub velocity along the disk-normal / tether direction,
    then feeds back a collective correction opposing that velocity.

    Works independently of ground comms — only requires IMU data, which is
    available at AP sample rate regardless of MAVLink state.

    Design
    ------
    1. High-pass filter body-Z acceleration to remove DC (gravity projection
       on body-Z changes slowly with attitude; the HP removes it cleanly).
    2. Leaky-integrate the HP-filtered signal to estimate oscillatory velocity.
       The leak term prevents drift from accumulating over long segments.
    3. Return -k_vib * vel_est, clamped to ±col_damp_max.

    The sign: body-Z points along disk normal.  When the tether stretches
    (tension spike), the hub accelerates toward the anchor → body-Z accel
    becomes more negative.  The HP+integrate gives negative velocity →
    correction is positive (more collective) → pushes hub back = damping.

    Frequency range
    ---------------
    Tether spring-mass resonance: f = sqrt(EA / (m * L)) / (2π)
    With EA=281 kN, m=5 kg, L=50–200 m → f ≈ 3–8 Hz.

    Default hp_freq_hz=1.5 passes this band while blocking attitude-change
    contributions (bandwidth ~0.1–0.5 Hz) and high-frequency noise above ~25 Hz
    (where the leaky integrator provides natural roll-off).

    Parameters
    ----------
    k_vib        : collective correction per m/s of estimated velocity [rad/(m/s)]
    hp_freq_hz   : high-pass filter cutoff [Hz]
    vel_tau_s    : leaky integrator decay time constant [s]
    col_damp_max : clamp on correction magnitude [rad]
    """

    def __init__(
        self,
        k_vib:        float = 0.008,
        hp_freq_hz:   float = 1.5,
        vel_tau_s:    float = 0.5,
        col_damp_max: float = 0.04,
    ) -> None:
        self._k_vib        = float(k_vib)
        self._hp_freq      = float(hp_freq_hz)
        self._vel_tau      = float(vel_tau_s)
        self._col_damp_max = float(col_damp_max)
        self._acc_hp   = 0.0
        self._acc_prev = 0.0
        self._vel_est  = 0.0

    def reset(self) -> None:
        self._acc_hp   = 0.0
        self._acc_prev = 0.0
        self._vel_est  = 0.0

    def step(self, accel_body_z: float, dt: float) -> float:
        """
        Update with one body-Z specific force sample.

        Parameters
        ----------
        accel_body_z : body-Z specific force [m/s²]  (R.T @ accel_specific_world)[2]
        dt           : timestep [s]

        Returns
        -------
        Collective correction [rad], clamped to ±col_damp_max.
        """
        # First-order high-pass: y[n] = alpha*(y[n-1] + x[n] - x[n-1])
        tau_hp   = 1.0 / (2.0 * math.pi * self._hp_freq)
        alpha_hp = tau_hp / (tau_hp + dt)
        self._acc_hp   = alpha_hp * (self._acc_hp + accel_body_z - self._acc_prev)
        self._acc_prev = accel_body_z

        # Leaky integrator: v[n] = exp(-dt/tau) * v[n-1] + dt * a_hp[n]
        leak          = math.exp(-dt / max(self._vel_tau, 1e-6))
        self._vel_est = leak * self._vel_est + dt * self._acc_hp

        correction = -self._k_vib * self._vel_est
        return float(max(-self._col_damp_max, min(self._col_damp_max, correction)))
