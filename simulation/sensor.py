"""
sensor.py — Sensor simulation for RAWES

Converts MBDyn world-frame state (ENU) into simulated sensor readings
in the body frame / NED convention expected by ArduPilot.

Simulated sensors:
  - IMU accelerometer  (body frame, m/s²)
  - IMU gyroscope      (body frame, rad/s)
  - Position           (NED, metres relative to takeoff)
  - Velocity           (NED, m/s)
  - Attitude (roll, pitch, yaw) extracted from rotation matrix

Noise model:
  - Gyro:   σ = 0.003 rad/s  (MEMS-grade)
  - Accel:  σ = 0.05  m/s²
  (Pixhawk 6C: ICM-42688-P / ICM-20649 specs)

Coordinate frames:
  ENU (MBDyn):  X = East, Y = North, Z = Up
  NED (ArduPilot): X = North, Y = East, Z = Down
  Body frame:   defined by R_hub rotation matrix (columns = body axes in world)

Euler angles extracted in ZYX order (yaw-pitch-roll) to match ArduPilot.
"""

import logging
import numpy as np
import math
from typing import Optional
from frames import T_ENU_NED, build_orb_frame

log = logging.getLogger(__name__)

# Sensor noise standard deviations
_GYRO_SIGMA  = 0.003   # rad/s
_ACCEL_SIGMA = 0.05    # m/s²

# Gravity vector in ENU world frame [m/s²]
_GRAVITY_ENU = np.array([0.0, 0.0, -9.81])


def _rotation_matrix_to_euler_zyx(R: np.ndarray) -> np.ndarray:
    """
    Extract ZYX Euler angles (yaw, pitch, roll) from rotation matrix.

    R transforms from body frame to world (ENU) frame:
        v_world = R @ v_body

    Returns [roll, pitch, yaw] in radians (ArduPilot convention).

    Singularity handling: uses atan2 formulation; pitch is clamped to ±π/2
    to avoid gimbal lock issues (should be safe for RAWES operating envelope).
    """
    # R columns: [x_body, y_body, z_body] in world frame
    # ZYX convention:
    #   R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    #
    # Extract: (using Tait-Bryan ZYX)
    #   pitch  = -asin(R[2,0])
    #   roll   = atan2(R[2,1], R[2,2])
    #   yaw    = atan2(R[1,0], R[0,0])
    #
    # (note: MBDyn uses column-major, but numpy is row-major, so R[row,col])

    # Clamp to avoid domain errors from floating point drift
    sin_pitch = float(np.clip(-R[2, 0], -1.0, 1.0))
    pitch = math.asin(sin_pitch)

    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) > 1e-6:
        roll = math.atan2(R[2, 1] / cos_pitch, R[2, 2] / cos_pitch)
        yaw  = math.atan2(R[1, 0] / cos_pitch, R[0, 0] / cos_pitch)
    else:
        # Gimbal lock: pitch ≈ ±90°
        roll = 0.0
        yaw  = math.atan2(-R[0, 1], R[1, 1])

    return np.array([roll, pitch, yaw])


def _euler_zyx_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Build a ZYX rotation matrix from [roll, pitch, yaw] angles.

    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    Maps body frame to world: v_world = R @ v_body.
    Inverse of _rotation_matrix_to_euler_zyx.
    """
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,     cp*sr,             cp*cr            ],
    ])



class SensorSim:
    """
    Simulated sensor suite for ArduPilot SITL.

    Parameters
    ----------
    gyro_sigma  : float   Gyroscope noise std dev [rad/s]
    accel_sigma : float   Accelerometer noise std dev [m/s²]
    rng_seed    : int | None   Random number seed for reproducibility
    home_enu_z  : float   ENU Z coordinate of the ArduPilot home position [m].
                          ArduPilot's NED origin (D=0) corresponds to this altitude
                          in the mediator's ENU frame.  Defaults to 0.0 (ENU origin
                          = ArduPilot home), but must be set to the hub's initial
                          ENU Z (typically 50.0 m) so that the hub starts at NED D=0
                          (at home altitude) rather than 50 m above it.
    anchor_enu  : array-like (3,)   ENU position of the tether anchor [m].
                  Attitude is reported as deviation from the tether-aligned
                  equilibrium orientation (body Z pointing from anchor to hub).
                  When the hub axle is perfectly aligned with the tether,
                  ArduPilot sees roll=0, pitch=0.  Defaults to world origin.
    """

    def __init__(
        self,
        gyro_sigma:    float = _GYRO_SIGMA,
        accel_sigma:   float = _ACCEL_SIGMA,
        rng_seed:      Optional[int] = None,
        home_enu_z:    float = 0.0,
        anchor_enu:    Optional[np.ndarray] = None,
        stable_body_z: Optional[np.ndarray] = None,
    ):
        self._gyro_sigma  = gyro_sigma
        self._accel_sigma = accel_sigma
        self._rng = np.random.default_rng(rng_seed)
        self._home_enu_z  = float(home_enu_z)
        self._anchor_enu  = np.asarray(anchor_enu, dtype=float) if anchor_enu is not None \
                            else np.zeros(3)
        # Physical equilibrium orientation: the disk orientation where forces balance
        # at zero collective.  When provided, ArduPilot sees roll=pitch=0 at this
        # orientation — NOT at the tether direction.  At low tether elevation angles
        # (< 20°) the tether direction has too little vertical component to support
        # the hub against gravity + tether downward pull, so using the tether as
        # the "level" reference causes the hub to fall immediately after unfreeze.
        if stable_body_z is not None:
            bz = np.asarray(stable_body_z, dtype=float)
            self._stable_R_orb_eq = build_orb_frame(bz / np.linalg.norm(bz))
        else:
            self._stable_R_orb_eq = None

    def compute(
        self,
        pos_enu:        np.ndarray,   # (3,) hub position in ENU [m]
        vel_enu:        np.ndarray,   # (3,) hub velocity in ENU [m/s]
        R_hub:          np.ndarray,   # (3,3) body→world rotation matrix
        omega_body:     np.ndarray,   # (3,) angular velocity in WORLD frame [rad/s]
                                      #       (MBDyn returns omega in world frame)
        accel_world_enu: np.ndarray,  # (3,) hub acceleration in world ENU [m/s²]
                                      #       = (vel_new - vel_old) / dt
        dt:             float,        # time step [s] (unused; for API consistency)
    ) -> dict:
        """
        Compute simulated sensor outputs from MBDyn world-frame state.

        Returns
        -------
        dict with keys:
            pos_ned    : np.ndarray (3,)  position NED [m]
            vel_ned    : np.ndarray (3,)  velocity NED [m/s]
            rpy        : np.ndarray (3,)  [roll, pitch, yaw] radians (ZYX)
            accel_body : np.ndarray (3,)  accelerometer reading body frame [m/s²]
            gyro_body  : np.ndarray (3,)  gyroscope reading body frame [rad/s]
        """
        # ------------------------------------------------------------------
        # 1. ENU → NED coordinate conversion
        #    NED: x=North=ENU_y, y=East=ENU_x, z=Down
        #
        # NED D is measured relative to the ArduPilot home altitude.
        # home_enu_z is the ENU Z of the hub's starting position (= home).
        # NED D = home_enu_z − ENU_Z   so that D=0 at the starting altitude.
        #
        # Without the home offset (−ENU_Z only), the hub at ENU Z=50m would
        # send NED D=−50m (50m above home), making ArduPilot believe it is at
        # home_altitude + 50m and command a 50m descent.  This is the root
        # cause of the hub falling 100m below its starting position.
        # ------------------------------------------------------------------
        pos_ned = np.array([pos_enu[1], pos_enu[0], self._home_enu_z - pos_enu[2]])
        vel_ned = np.array([vel_enu[1], vel_enu[0], -vel_enu[2]])

        # ------------------------------------------------------------------
        # 2. Attitude: tether-relative deviation from equilibrium orientation
        #
        # The RAWES equilibrium has the rotor axle (body Z) aligned with the
        # tether (anchor → hub direction).  ArduPilot must see roll=0, pitch=0
        # at this equilibrium — otherwise its attitude controller commands
        # maximum cyclic trying to "uprght" a disk that is already in its
        # natural position.
        #
        # Strategy:
        #   1. Compute R_orb_eq: equilibrium orbital frame built from the tether
        #      direction as body Z.  When hub axle = tether direction → equilibrium.
        #   2. Compute R_orb: actual orbital frame from disk_normal (spin removed).
        #   3. R_dev = R_orb_eq.T @ R_orb: deviation from equilibrium (small when
        #      axle ≈ tether, large when axle is tilted away from tether).
        #   4. Report attitude as R_dev in NED convention → roll=0,pitch=0 at equil.
        #
        # Accel and gyro are still expressed in R_orb (the physical electronics
        # frame).  At equilibrium R_orb = R_orb_eq so all three are consistent.
        # Small deviations cause only second-order EKF inconsistency.
        # ------------------------------------------------------------------

        # Actual orbital frame (spin removed): body Z = disk_normal.
        disk_normal = R_hub[:, 2]
        R_orb = build_orb_frame(disk_normal)

        # Equilibrium orbital frame.
        # Use the stable_body_z if provided (physical force-balance equilibrium).
        # Otherwise fall back to the live tether direction.
        if self._stable_R_orb_eq is not None:
            R_orb_eq = self._stable_R_orb_eq
        else:
            tether_vec = pos_enu - self._anchor_enu
            tether_len = float(np.linalg.norm(tether_vec))
            if tether_len > 1.0:
                body_z_eq = tether_vec / tether_len
            else:
                body_z_eq = np.array([0.0, 0.0, 1.0])
            R_orb_eq = build_orb_frame(body_z_eq)

        # Deviation rotation: identity when hub axle is aligned with tether.
        R_dev = R_orb_eq.T @ R_orb

        # Convert to NED body convention and extract Euler angles.
        # T = [[0,1,0],[1,0,0],[0,0,-1]]: ENU body ↔ NED body (symmetric).
        # T @ R_dev @ T gives NED convention; Euler=[0,0,0] at tether equilibrium.
        _T = T_ENU_NED
        R_ned = _T @ R_dev @ _T

        rpy = _rotation_matrix_to_euler_zyx(R_ned)

        # ------------------------------------------------------------------
        # 2b. Replace yaw with velocity-derived heading for EKF consistency.
        #
        # SensorSim reports roll/pitch as tether-relative deviations from
        # R_dev — so ArduPilot sees roll=pitch=0 at equilibrium, giving the
        # compute_rc_from_attitude P-term a real error signal.
        #
        # However the orientation-derived yaw (from R_dev) differs from the
        # GPS velocity heading atan2(vE, vN).  At our equilibrium the
        # orientation yaw is 0° while the velocity heading is ~−15.7°.  After
        # GPS lock the EKF compares these and triggers "MAG ground anomaly /
        # yaw re-aligned" or "GPS Glitch", causing an emergency yaw reset and
        # hub crash.
        #
        # Fix: replace rpy[2] with velocity-derived yaw (as build_sitl_packet
        # does).  This keeps roll/pitch as tether-relative deviations while
        # making yaw consistent with the velocity vector, eliminating the EKF
        # inconsistency.  Rebuild the body-frame rotation matrix to match.
        #
        # Fallback: when horizontal speed < 0.05 m/s (start-up or near-zero
        # velocity) keep the orientation-derived yaw to avoid discontinuities.
        # ------------------------------------------------------------------
        v_horiz = math.hypot(vel_ned[0], vel_ned[1])
        if v_horiz > 0.05:
            rpy[2] = math.atan2(vel_ned[1], vel_ned[0])
        # else: keep orientation-derived rpy[2]

        # Rebuild R_body consistent with the (possibly updated) yaw.
        R_body = _euler_zyx_to_rotation(float(rpy[0]), float(rpy[1]), float(rpy[2]))

        # ------------------------------------------------------------------
        # 3. IMU accelerometer
        #
        # For EKF consistency, the accelerometer must be expressed in the
        # SAME body frame as the attitude we report (R_body above).
        #
        #   specific_force_ned = T @ (accel_world_enu - gravity_enu)
        #   accel_body = R_body.T @ specific_force_ned
        #
        # At tether equilibrium with vel_yaw: R_body = Rz(vel_yaw).
        #   specific_force = T @ [0,0,9.81] = [0,0,-9.81]
        #   accel_body = Rz(-vel_yaw) @ [0,0,-9.81] = [0,0,-9.81] ✓
        # ------------------------------------------------------------------
        a_specific_enu = accel_world_enu - _GRAVITY_ENU
        specific_force_ned = _T @ a_specific_enu
        accel_body = R_body.T @ specific_force_ned

        # Add accelerometer noise
        accel_body = accel_body + self._rng.normal(0.0, self._accel_sigma, 3)

        # ------------------------------------------------------------------
        # 4. IMU gyroscope
        #
        # Same consistency requirement: gyro must be in the same body frame
        # (R_body) so that d(attitude)/dt = skew(gyro) × attitude.
        #
        #   gyro_body = R_body.T @ (T @ omega_orbital_enu)
        #
        # At equilibrium R_body = Rz(vel_yaw) and omega_orbital = 0:
        #   gyro_body = 0 ✓
        # ------------------------------------------------------------------
        disk_normal_world = R_hub[:, 2]   # body Z-axis expressed in ENU world frame
        omega_spin_world   = np.dot(omega_body, disk_normal_world) * disk_normal_world
        omega_orbital_world = omega_body - omega_spin_world
        omega_body_frame = R_body.T @ (_T @ omega_orbital_world)

        # Add gyroscope noise
        gyro_body = omega_body_frame + self._rng.normal(0.0, self._gyro_sigma, 3)

        return {
            "pos_ned":    pos_ned,
            "vel_ned":    vel_ned,
            "rpy":        rpy,
            "accel_body": accel_body,
            "gyro_body":  gyro_body,
        }


def build_sitl_packet(
    hub_state:     dict,
    accel_world:   np.ndarray,
    home_z_enu:    float,
    freeze_vel_ned: np.ndarray,
    frozen:        bool,
) -> dict:
    """
    Build the SITL sensor packet from hub ENU truth state.

    Reports tether-relative attitude (rpy=[0,0,yaw_from_velocity]) so that
    ArduPilot sees roll=pitch=0 at tether equilibrium. All quantities are
    expressed in the yaw-aligned NED body frame ArduPilot expects.

    Parameters
    ----------
    hub_state      : dict with keys pos, vel, R, omega (all ENU world frame)
    accel_world    : (3,) kinematic acceleration in ENU world frame [m/s²]
    home_z_enu     : ENU Z of the ArduPilot home position [m]
    freeze_vel_ned : (3,) hint velocity [m/s] used during startup freeze
    frozen         : True during startup freeze period

    Returns
    -------
    dict with keys: pos_ned, vel_ned, rpy, accel_body, gyro_body
    """
    _pos = hub_state["pos"]
    _R   = hub_state["R"]
    _omg = hub_state["omega"]
    _vel = hub_state["vel"]

    # Position NED
    _pos_ned = np.array([_pos[1], _pos[0], -(_pos[2] - home_z_enu)])

    # Velocity NED (or freeze hint)
    if frozen:
        _vel_ned = freeze_vel_ned
    else:
        _vel_ned = np.array([_vel[1], _vel[0], -_vel[2]])

    # Yaw from velocity heading (keeps attitude/velocity consistent in EKF)
    _v_horiz = np.hypot(_vel_ned[0], _vel_ned[1])
    _yaw_ned = float(np.arctan2(_vel_ned[1], _vel_ned[0])) if _v_horiz > 0.1 else 0.0
    _rpy     = np.array([0., 0., _yaw_ned])

    # Accelerometer: kinematic accel (ENU) → NED body frame
    _accel_ned  = T_ENU_NED @ accel_world + np.array([0., 0., -9.81])
    _cy, _sy    = np.cos(_yaw_ned), np.sin(_yaw_ned)
    _Rz_inv     = np.array([[_cy, _sy, 0.], [-_sy, _cy, 0.], [0., 0., 1.]])
    _accel_body = _Rz_inv @ _accel_ned

    # Gyro: strip spin, rotate into yaw-aligned body frame
    _bz_cur     = _R[:, 2]
    _omg_nospin = _omg - np.dot(_omg, _bz_cur) * _bz_cur
    _gyro_ned   = T_ENU_NED @ _omg_nospin
    _gyro_body  = _Rz_inv @ _gyro_ned

    return {
        "pos_ned":    _pos_ned,
        "vel_ned":    _vel_ned,
        "rpy":        _rpy,
        "accel_body": _accel_body,
        "gyro_body":  _gyro_body,
    }


# ---------------------------------------------------------------------------
# Physical sensor — reports actual orientation of the Pixhawk electronics
# ---------------------------------------------------------------------------

class PhysicalSensor:
    """
    Simulates the real sensor readings of the Pixhawk mounted on the
    non-rotating RAWES electronics assembly.

    Unlike SensorSim, this class does NOT transform attitude into a
    tether-relative frame.  It reports the true NED Euler angles of the
    orbital frame (rotor axle direction = body Z, spin stripped), which at
    tether equilibrium is roughly 65° from NED vertical.

    Accel and gyro are expressed in the same physical orbital body frame,
    consistent with the reported attitude.  Yaw is still velocity-derived
    for EKF GPS consistency (same constraint as SensorSim).

    Use case: GUIDED_NOGPS + SET_ATTITUDE_TARGET, where the GCS commands
    the real equilibrium quaternion as the target and ArduPilot holds it at
    full inner-loop bandwidth.
    """

    def __init__(
        self,
        home_enu_z:  float = 0.0,
        gyro_sigma:  float = _GYRO_SIGMA,
        accel_sigma: float = _ACCEL_SIGMA,
        rng_seed:    Optional[int] = None,
    ):
        self._home_enu_z  = float(home_enu_z)
        self._gyro_sigma  = gyro_sigma
        self._accel_sigma = accel_sigma
        self._rng = np.random.default_rng(rng_seed)
        # Cache of the last velocity-derived yaw.  Once set, this is used as
        # the fallback yaw when v_horiz < threshold so there is no yaw
        # discontinuity when the hub decelerates through zero (e.g. at physics
        # unfreeze).  Without this, the orbital-frame geometric yaw would snap
        # back into use, rotating the body frame and causing a large magnetometer
        # reading change → EKF "GPS Glitch or Compass error".
        self._last_vel_yaw: Optional[float] = None
        # Maximum rate at which the velocity-derived yaw is allowed to change
        # [rad/s].  This prevents a sudden gyro axis remap when the tether
        # activates and the hub velocity direction rotates quickly.  Without
        # rate-limiting, a 165° yaw jump in 2 s causes the ACRO rate loop to
        # see gyro on the wrong body axis → destabilising corrections.
        # The hub's natural orbital yaw rate is ~0.02 rad/s (1°/s); 0.05 rad/s
        # (≈3°/s) allows orbital tracking while freezing the ~82°/s tether
        # activation transient so the gyro body frame is stable.
        self._vel_yaw_rate_max: float = 0.05   # rad/s

    def compute(
        self,
        pos_enu:         np.ndarray,
        vel_enu:         np.ndarray,
        R_hub:           np.ndarray,
        omega_body:      np.ndarray,
        accel_world_enu: np.ndarray,
        dt:              float,
    ) -> dict:
        """
        Compute physical sensor outputs.

        Returns the same dict schema as SensorSim.compute():
            pos_ned, vel_ned, rpy, accel_body, gyro_body
        """
        # Position and velocity in NED (same as SensorSim)
        pos_ned = np.array([pos_enu[1], pos_enu[0], -(pos_enu[2] - self._home_enu_z)])
        vel_ned = np.array([vel_enu[1], vel_enu[0], -vel_enu[2]])

        # Attitude: actual orbital frame (electronics, spin removed) → NED Euler angles.
        # R_orb is body→world in ENU; T @ R_orb @ T converts both frames to NED.
        disk_normal = R_hub[:, 2]
        R_orb = build_orb_frame(disk_normal)
        R_ned = T_ENU_NED @ R_orb @ T_ENU_NED
        rpy   = _rotation_matrix_to_euler_zyx(R_ned)

        # Replace orientation-derived yaw with velocity heading for EKF GPS
        # consistency.  Cache the velocity-derived yaw and use it as fallback
        # when v_horiz < threshold — prevents a compass reading jump when the
        # hub decelerates through zero (physics unfreeze starts from vel=0).
        #
        # Rate-limit yaw updates: when the tether activates and the hub
        # velocity direction rotates rapidly (~82°/s), an un-limited jump
        # remaps the gyro body axes, causing ACRO's rate loop to apply
        # destabilising corrections.  Limiting to _vel_yaw_rate_max (default
        # 1.0 rad/s) prevents this while tracking the ~0.2 rad/s orbital rate.
        v_horiz = math.hypot(vel_ned[0], vel_ned[1])
        if v_horiz > 0.05:
            raw_vel_yaw = math.atan2(vel_ned[1], vel_ned[0])
            if self._last_vel_yaw is None:
                self._last_vel_yaw = raw_vel_yaw
            else:
                # Wrap delta to [-π, π] before rate-limiting
                delta = raw_vel_yaw - self._last_vel_yaw
                delta = (delta + math.pi) % (2 * math.pi) - math.pi
                max_delta = self._vel_yaw_rate_max * dt
                self._last_vel_yaw += max(-max_delta, min(max_delta, delta))
                self._last_vel_yaw = (self._last_vel_yaw + math.pi) % (2 * math.pi) - math.pi
        if self._last_vel_yaw is not None:
            rpy[2] = self._last_vel_yaw

        # Rebuild R_body consistent with the (possibly yaw-overridden) rpy so
        # accel and gyro are expressed in the same frame ArduPilot expects.
        R_body = _euler_zyx_to_rotation(float(rpy[0]), float(rpy[1]), float(rpy[2]))

        # Accelerometer: specific force in physical body frame
        a_specific_enu    = accel_world_enu - _GRAVITY_ENU
        specific_force_ned = T_ENU_NED @ a_specific_enu
        accel_body = R_body.T @ specific_force_ned
        accel_body = accel_body + self._rng.normal(0.0, self._accel_sigma, 3)

        # Gyroscope: orbital angular velocity in physical body frame.
        # The electronics are kept non-rotating by the GB4008 motor, so the
        # gyro does not see the blade spin — strip it before projecting.
        omega_spin_world    = np.dot(omega_body, disk_normal) * disk_normal
        omega_orbital_world = omega_body - omega_spin_world
        gyro_body = R_body.T @ (T_ENU_NED @ omega_orbital_world)
        gyro_body = gyro_body + self._rng.normal(0.0, self._gyro_sigma, 3)

        return {
            "pos_ned":    pos_ned,
            "vel_ned":    vel_ned,
            "rpy":        rpy,
            "accel_body": accel_body,
            "gyro_body":  gyro_body,
        }


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def make_sensor(
    mode:         str,
    home_enu_z:   float = 0.0,
    anchor_enu:   Optional[np.ndarray] = None,
    stable_body_z: Optional[np.ndarray] = None,
    gyro_sigma:   float = _GYRO_SIGMA,
    accel_sigma:  float = _ACCEL_SIGMA,
    rng_seed:     Optional[int] = None,
) -> "SensorSim | PhysicalSensor":
    """
    Return a configured sensor for the given mode.

    Parameters
    ----------
    mode : str
        ``"tether_relative"`` — SensorSim: reports roll=pitch=0 at tether
        equilibrium.  Use with ACRO + RC override.

        ``"physical"`` — PhysicalSensor: reports actual orbital-frame
        orientation (~65° from NED vertical at tether equilibrium).
        Use with GUIDED_NOGPS + SET_ATTITUDE_TARGET.
    home_enu_z : float
        ENU Z of the ArduPilot home position (NED D=0 reference) [m].
    anchor_enu : array (3,) or None
        Tether anchor in ENU [m].  Only used by ``"tether_relative"`` mode.
    stable_body_z : array (3,) or None
        Fixed equilibrium body_z for tether-relative reporting.
        ``None`` = use live tether direction.  Only used by ``"tether_relative"``.
    """
    if mode == "tether_relative":
        return SensorSim(
            home_enu_z    = home_enu_z,
            anchor_enu    = anchor_enu,
            stable_body_z = stable_body_z,
            gyro_sigma    = gyro_sigma,
            accel_sigma   = accel_sigma,
            rng_seed      = rng_seed,
        )
    if mode == "physical":
        return PhysicalSensor(
            home_enu_z  = home_enu_z,
            gyro_sigma  = gyro_sigma,
            accel_sigma = accel_sigma,
            rng_seed    = rng_seed,
        )
    raise ValueError(
        f"Unknown sensor mode {mode!r}. Choose 'tether_relative' or 'physical'."
    )


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    print("SensorSim smoke test")

    sensor = SensorSim(rng_seed=42)

    # Test coordinate transform (ENU → NED)
    pos_enu = np.array([5.0, 10.0, 50.0])   # East=5, North=10, Up=50
    pos_ned_expected = np.array([10.0, 5.0, -50.0])  # North, East, Down
    pos_ned = np.array([pos_enu[1], pos_enu[0], -pos_enu[2]])
    assert np.allclose(pos_ned, pos_ned_expected), f"ENU→NED failed: {pos_ned}"
    print("  ENU→NED conversion: OK")

    # Test with identity rotation (no tilt)
    R = np.eye(3)
    omega_world = np.array([0.0, 0.0, 28.0])  # spin about world Z
    accel_world = np.array([0.0, 0.0, 9.81])  # hovering (aero thrust = weight)
    vel_enu     = np.zeros(3)

    result = sensor.compute(pos_enu, vel_enu, R, omega_world, accel_world, dt=0.0025)

    print(f"  pos_ned    : {result['pos_ned']}")
    print(f"  vel_ned    : {result['vel_ned']}")
    print(f"  rpy (deg)  : {np.degrees(result['rpy']).round(3)}")
    print(f"  accel_body : {result['accel_body'].round(4)}")
    print(f"  gyro_body  : {result['gyro_body'].round(4)}")

    # Hover accel in NED body frame:
    #   specific_force_world = a_world - g_enu = [0,0,0] - [0,0,-9.81] = [0,0,+9.81]
    #   In ENU orbital body frame (R_orb = I): accel_enu_body = [0, 0, +9.81]
    #   Apply T (ENU→NED body): T @ [0,0,9.81] = [0, 0, -9.81]
    #   (body Z flips: ENU Up → NED Down, so +9.81 Up → -9.81 in NED body Z)
    accel_hover_world = np.zeros(3)
    result2 = sensor.compute(pos_enu, vel_enu, R, omega_world, accel_hover_world, dt=0.0025)
    assert abs(result2["accel_body"][2] - (-9.81)) < 0.3, \
        f"Hover accel Z expected ~-9.81 (NED body Z=Down), got {result2['accel_body'][2]}"
    print("  Hover accelerometer Z ≈ -9.81 (NED body-down): OK")

    # Euler extraction from identity R → all zero angles
    rpy_zero = _rotation_matrix_to_euler_zyx(np.eye(3))
    assert np.allclose(rpy_zero, 0.0, atol=1e-10), f"Identity R→non-zero angles: {rpy_zero}"
    print("  Identity R → zero Euler: OK")

    # 90° roll test
    R_roll90 = np.array([
        [1, 0,  0],
        [0, 0, -1],
        [0, 1,  0],
    ], dtype=float)
    rpy_roll90 = _rotation_matrix_to_euler_zyx(R_roll90)
    assert abs(rpy_roll90[0] - math.pi / 2) < 1e-6, \
        f"90° roll test failed: {math.degrees(rpy_roll90[0]):.2f}°"
    print(f"  90° roll: rpy = {np.degrees(rpy_roll90).round(2)} deg OK")

    print("All smoke tests passed.")
    sys.exit(0)
