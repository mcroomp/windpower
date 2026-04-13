"""
sensor.py — Sensor simulation for RAWES

Converts physics world-frame state (NED) into simulated sensor readings
in the body frame / NED convention expected by ArduPilot.

The physics layer (dynamics, aero, tether, controller) all operate in NED,
so no ENU↔NED conversion is required here.

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
  NED: X = North, Y = East, Z = Down  (world frame + ArduPilot convention)
  Body frame: defined by R_hub rotation matrix (columns = body axes in world NED)

Euler angles extracted in ZYX order (yaw-pitch-roll) to match ArduPilot.
"""

import logging
import numpy as np
import math
from typing import Optional
from frames import build_orb_frame

log = logging.getLogger(__name__)

# Sensor noise standard deviations
_GYRO_SIGMA  = 0.003   # rad/s
_ACCEL_SIGMA = 0.05    # m/s²

# Gravity vector in NED world frame [m/s²]  (Z=Down so gravity is +Z)
_GRAVITY_NED = np.array([0.0, 0.0, 9.81])


def _rotation_matrix_to_euler_zyx(R: np.ndarray) -> np.ndarray:
    """
    Extract ZYX Euler angles (yaw, pitch, roll) from rotation matrix.

    R transforms from body frame to world (NED) frame:
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
    # (note: numpy is row-major, so R[row,col])

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


# ---------------------------------------------------------------------------
# Physical sensor — reports actual orientation of the Pixhawk electronics
# ---------------------------------------------------------------------------

class PhysicalSensor:
    """
    Simulates the real sensor readings of the Pixhawk mounted on the
    non-rotating RAWES electronics assembly.

    Reports the true NED Euler angles of the orbital frame (rotor axle
    direction = body Z, spin stripped).  At tether equilibrium the attitude
    is roughly 65° from NED vertical.

    Accel and gyro are expressed in the physical orbital body frame,
    consistent with the reported attitude.  Yaw is velocity-derived for
    EKF GPS consistency.
    """

    def __init__(
        self,
        home_ned_z:  float = 0.0,
        gyro_sigma:  float = _GYRO_SIGMA,
        accel_sigma: float = _ACCEL_SIGMA,
        rng_seed:    Optional[int] = None,
        initial_vel: Optional[np.ndarray] = None,
        initial_R:   Optional[np.ndarray] = None,
    ):
        self._home_ned_z  = float(home_ned_z)
        self._gyro_sigma  = gyro_sigma
        self._accel_sigma = accel_sigma
        self._rng = np.random.default_rng(rng_seed)
        # Cache of the last velocity-derived yaw.  Once set, this is used as
        # the fallback yaw when v_horiz < threshold so there is no yaw
        # discontinuity when the hub decelerates through zero (e.g. at physics
        # unfreeze).  Without this, the orbital-frame geometric yaw would snap
        # back into use, rotating the body frame and causing a large magnetometer
        # reading change → EKF "GPS Glitch or Compass error".
        #
        # Pre-initialized from initial_vel/initial_R when provided (production
        # path via make_sensor).  Tests that construct PhysicalSensor directly
        # and control every compute() call can omit these.
        if initial_vel is not None:
            v = np.asarray(initial_vel, dtype=float)
            v_horiz = math.hypot(v[0], v[1])
            if v_horiz > 0.05:
                self._last_vel_yaw: Optional[float] = math.atan2(v[1], v[0])
            elif initial_R is not None:
                disk_normal = np.asarray(initial_R, dtype=float)[:, 2]
                R_orb = build_orb_frame(disk_normal)
                rpy = _rotation_matrix_to_euler_zyx(R_orb)
                self._last_vel_yaw = float(rpy[2])
            else:
                self._last_vel_yaw = None
        else:
            self._last_vel_yaw = None
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
        pos_ned:         np.ndarray,
        vel_ned:         np.ndarray,
        R_hub:           np.ndarray,
        omega_body:      np.ndarray,
        accel_world_ned: np.ndarray,
        dt:              float,
    ) -> dict:
        """
        Compute physical sensor outputs.

        Returns:
            pos_ned, vel_ned, rpy, accel_body, gyro_body
        """
        # Position and velocity relative to home (already in NED)
        pos_ned_rel = np.array([pos_ned[0], pos_ned[1], pos_ned[2] - self._home_ned_z])
        vel_ned_out = vel_ned.copy()

        # Attitude: actual orbital frame (electronics, spin removed) → NED Euler angles.
        # R_orb is body→world in NED; extract Euler angles directly.
        disk_normal = R_hub[:, 2]
        R_orb = build_orb_frame(disk_normal)
        rpy   = _rotation_matrix_to_euler_zyx(R_orb)

        # Replace orientation-derived yaw with a coherent velocity-or-attitude
        # heading for EKF GPS consistency.  Two regimes, both rate-limited:
        #
        #   v_horiz > 0.05 m/s  ->  GPS velocity heading (atan2 vel_ned)
        #   v_horiz <= 0.05 m/s ->  orbital frame yaw from R_orb
        #
        # Tracking the orbital frame yaw at low speed (rather than freezing the
        # last cached value) prevents EKF yaw divergence during long kinematic
        # hold phases where ACRO/Lua corrections rotate the hub: a frozen
        # reported yaw causes the gyro-integrated yaw to drift away from the
        # attitude yaw → EKF detects compass/GPS inconsistency → emergency yaw
        # reset.  The orbital frame yaw is already coherent with the gyroscope
        # because it is derived from R_hub (the same orientation the gyro tracks).
        #
        # Rate-limit in both cases: when the tether activates and the hub
        # velocity direction rotates rapidly (~82°/s), an un-limited jump
        # remaps the gyro body axes, causing ACRO's rate loop to apply
        # destabilising corrections.  0.05 rad/s (~3°/s) covers the normal
        # orbital yaw rate (~0.02 rad/s) with margin, while damping the
        # tether-activation transient.
        v_horiz  = math.hypot(vel_ned_out[0], vel_ned_out[1])
        orb_yaw  = float(rpy[2])          # R_orb yaw before any override

        if v_horiz > 0.05:
            raw_yaw = math.atan2(vel_ned_out[1], vel_ned_out[0])
        else:
            raw_yaw = orb_yaw

        if self._last_vel_yaw is None:
            self._last_vel_yaw = raw_yaw
        else:
            # Wrap delta to [-π, π] before rate-limiting
            delta = raw_yaw - self._last_vel_yaw
            delta = (delta + math.pi) % (2 * math.pi) - math.pi
            max_delta = self._vel_yaw_rate_max * dt
            self._last_vel_yaw += max(-max_delta, min(max_delta, delta))
            self._last_vel_yaw = (self._last_vel_yaw + math.pi) % (2 * math.pi) - math.pi
        rpy[2] = self._last_vel_yaw

        # Rebuild R_body consistent with the (possibly yaw-overridden) rpy so
        # accel and gyro are expressed in the same frame ArduPilot expects.
        R_body = _euler_zyx_to_rotation(float(rpy[0]), float(rpy[1]), float(rpy[2]))

        # Accelerometer: specific force in physical body frame
        a_specific_ned = accel_world_ned - _GRAVITY_NED
        accel_body = R_body.T @ a_specific_ned
        accel_body = accel_body + self._rng.normal(0.0, self._accel_sigma, 3)

        # Gyroscope: orbital angular velocity in physical body frame.
        # The electronics are kept non-rotating by the GB4008 motor, so the
        # gyro does not see the blade spin — strip it before projecting.
        omega_spin_world    = np.dot(omega_body, disk_normal) * disk_normal
        omega_orbital_world = omega_body - omega_spin_world
        gyro_body = R_body.T @ omega_orbital_world
        gyro_body = gyro_body + self._rng.normal(0.0, self._gyro_sigma, 3)

        return {
            "pos_ned":     pos_ned_rel,
            "vel_ned":     vel_ned_out,
            "rpy":         rpy,
            "accel_body":  accel_body,
            "gyro_body":   gyro_body,
            # Diagnostics for EKF yaw tracking: logged to telemetry CSV so
            # GPS/EKF glitches can be correlated with the physics inputs.
            "orb_yaw_rad": orb_yaw,   # R_orb yaw (actual hub orientation)
            "v_horiz_ms":  v_horiz,   # horizontal speed [m/s]
        }


# ---------------------------------------------------------------------------
# SpinSensor
# ---------------------------------------------------------------------------

class SpinSensor:
    """
    Simulated rotor spin rate sensor.

    In the simulation, rotor spin speed (omega_spin) is an internal ODE state.
    This class adds optional Gaussian noise to simulate measurement uncertainty.
    On hardware, omega_spin is derived from the GB4008 counter-torque motor eRPM
    reported by the AM32 ESC via telemetry, converted using the 80:44 gear ratio
    and 11 pole pairs: ``omega_spin = eRPM × 2π/60 / 11 × 44/80``.

    Physical model validation (optional)
    -------------------------------------
    ``validate(omega_measured, v_inplane)`` checks the measured spin rate
    against the autorotation torque-balance prediction:

        omega_expected = sqrt(K_drive × v_inplane / K_drag)

    Parameters
    ----------
    sigma    : float — Gaussian noise std dev [rad/s].  0 = ideal (default).
    K_drive  : float — autorotation drive constant [N·m·s/m].
    K_drag   : float — autorotation drag constant [N·m·s²/rad²].
    rng_seed : int or None — random seed for reproducibility.
    """

    def __init__(
        self,
        sigma:    float = 0.0,
        K_drive:  float = 1.4,
        K_drag:   float = 0.01786,
        rng_seed: "int | None" = None,
    ):
        self._sigma   = float(sigma)
        self._K_drive = float(K_drive)
        self._K_drag  = float(K_drag)
        self._rng     = np.random.default_rng(rng_seed)

    def measure(self, omega_true: float) -> float:
        """
        Return omega_true with optional Gaussian noise added [rad/s].
        Always returns a non-negative value.
        """
        if self._sigma > 0.0:
            return float(max(0.0, omega_true + self._rng.normal(0.0, self._sigma)))
        return float(max(0.0, omega_true))

    def validate(
        self,
        omega_measured: float,
        v_inplane:      float,
        warn_threshold: float = 0.15,
    ) -> dict:
        """
        Cross-check *omega_measured* against the autorotation torque-balance model.

        Uses: omega_expected = sqrt(K_drive × v_inplane / K_drag)

        Parameters
        ----------
        omega_measured  : float — measured spin rate [rad/s]
        v_inplane       : float — estimated in-plane wind speed [m/s]
        warn_threshold  : float — fractional deviation that triggers a warning (default 15 %)

        Returns
        -------
        dict with keys:
            "omega_expected"  float — model prediction [rad/s]
            "deviation_frac"  float — |measured - expected| / expected
            "ok"              bool  — True if deviation < warn_threshold
        """
        if v_inplane <= 0.0:
            return {"omega_expected": 0.0, "deviation_frac": float("inf"), "ok": False}
        omega_expected  = math.sqrt(self._K_drive * v_inplane / self._K_drag)
        deviation_frac  = abs(omega_measured - omega_expected) / max(omega_expected, 1e-9)
        return {
            "omega_expected": omega_expected,
            "deviation_frac": deviation_frac,
            "ok":             deviation_frac < warn_threshold,
        }


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def make_sensor(
    home_ned_z:  float,
    initial_vel: np.ndarray,
    initial_R:   np.ndarray,
    gyro_sigma:  float = _GYRO_SIGMA,
    accel_sigma: float = _ACCEL_SIGMA,
    rng_seed:    Optional[int] = None,
) -> "PhysicalSensor":
    """
    Return a PhysicalSensor with yaw cache pre-seeded from the initial state.

    Parameters
    ----------
    home_ned_z  : NED Z of ArduPilot home (LOCAL_POSITION_NED D=0 reference) [m]
    initial_vel : initial hub NED velocity [m/s] — seeds the yaw cache
    initial_R   : initial hub rotation matrix — fallback yaw when vel is near zero
    """
    return PhysicalSensor(
        home_ned_z  = home_ned_z,
        gyro_sigma  = gyro_sigma,
        accel_sigma = accel_sigma,
        rng_seed    = rng_seed,
        initial_vel = initial_vel,
        initial_R   = initial_R,
    )


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    print("PhysicalSensor smoke test")

    sensor = make_sensor(
        home_ned_z  = -50.0,
        initial_vel = np.array([0.9, 0.0, 0.0]),
        initial_R   = np.eye(3),
        rng_seed    = 42,
    )

    # Test with identity rotation (no tilt)
    # In NED: hub at North=10, East=5, altitude=50 → pos_ned=[10, 5, -50]
    pos_ned = np.array([10.0, 5.0, -50.0])   # North=10, East=5, Down=-50
    R = np.eye(3)
    omega_world = np.array([0.0, 0.0, 28.0])  # spin about NED Z (down)
    accel_world = np.array([0.0, 0.0, 9.81])  # hovering (aero thrust = weight)
    vel_ned_in  = np.zeros(3)

    result = sensor.compute(pos_ned, vel_ned_in, R, omega_world, accel_world, dt=0.0025)

    print(f"  pos_ned    : {result['pos_ned']}")
    print(f"  vel_ned    : {result['vel_ned']}")
    print(f"  rpy (deg)  : {np.degrees(result['rpy']).round(3)}")
    print(f"  accel_body : {result['accel_body'].round(4)}")
    print(f"  gyro_body  : {result['gyro_body'].round(4)}")
    accel_hover_world = np.zeros(3)
    result2 = sensor.compute(pos_ned, vel_ned_in, R, omega_world, accel_hover_world, dt=0.0025)
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
