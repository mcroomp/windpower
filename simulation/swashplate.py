"""
swashplate.py — H3-120 swashplate mixing and blade pitch computation

Implements the mechanical chain from servo PWM commands through the
H3-120 swashplate geometry to individual blade cyclic pitch angles.

Physical layout (RAWES):
  S1 at 0°   (East reference)
  S2 at 120° (South-East)
  S3 at 240° (South-West)

Sign conventions:
  tilt_lon > 0  →  rotor disk tilts forward (nose down, pitch positive)
  tilt_lat > 0  →  rotor disk tilts right (roll right, starboard up)
  collective > 0 → positive blade pitch angle
"""

import math
import numpy as np

# H3-120 geometry: servo angular positions [rad]
_S1_ANG = 0.0               # 0°
_S2_ANG = 2.0 * math.pi / 3   # 120°
_S3_ANG = 4.0 * math.pi / 3   # 240°

# H3-120 inverse mix precomputed constants
_SIN_S2 = math.sin(_S2_ANG)   #  sin(120°) =  0.866
_SIN_S3 = math.sin(_S3_ANG)   #  sin(240°) = -0.866
_COS_S2 = math.cos(_S2_ANG)   #  cos(120°) = -0.5
_COS_S3 = math.cos(_S3_ANG)   #  cos(240°) = -0.5


# ---------------------------------------------------------------------------
# PWM conversion
# ---------------------------------------------------------------------------

def pwm_to_normalized(pwm_us: float) -> float:
    """
    Convert PWM microseconds to a normalised [-1, 1] value.

    1000 µs → -1.0   (full negative)
    1500 µs →  0.0   (neutral / centre)
    2000 µs → +1.0   (full positive)

    Parameters
    ----------
    pwm_us : float   PWM pulse width in microseconds

    Returns
    -------
    float  Normalised value clamped to [-1, 1]
    """
    return float(np.clip((pwm_us - 1500.0) / 500.0, -1.0, 1.0))


def pwm_array_to_normalized(pwm_array: np.ndarray) -> np.ndarray:
    """Vectorised version of pwm_to_normalized for an array of PWM values."""
    arr = np.asarray(pwm_array, dtype=np.float64)
    return np.clip((arr - 1500.0) / 500.0, -1.0, 1.0)


# ---------------------------------------------------------------------------
# H3-120 inverse mixer
# ---------------------------------------------------------------------------

def h3_inverse_mix(s1: float, s2: float, s3: float):
    """
    Decode H3-120 swashplate servo positions into collective and cyclic terms.

    The H3-120 layout places servos at 0°, 120°, 240° around the swashplate.
    Given normalised servo positions, compute:
      collective — average swashplate displacement (up/down)
      tilt_lon   — longitudinal tilt (pitch axis, forward positive)
      tilt_lat   — lateral tilt (roll axis, right positive)

    All outputs are normalised ([-1, 1] before gain scaling). Apply
    _PITCH_GAIN_RAD to convert tilt to radians.

    Parameters
    ----------
    s1, s2, s3 : float   Normalised servo values in [-1, 1]

    Returns
    -------
    (collective, tilt_lon, tilt_lat) : tuple of float, all normalised [-1, 1]
    """
    # Collective: equal contribution from all three servos
    collective = (s1 + s2 + s3) / 3.0

    # Lateral tilt (roll): projection on X axis (servo at 0° = s1)
    # tilt_lat = (2/3) * (s1 * cos(0°) + s2 * cos(120°) + s3 * cos(240°))
    tilt_lat = (2.0 / 3.0) * (s1 * 1.0 + s2 * _COS_S2 + s3 * _COS_S3)

    # Longitudinal tilt (pitch): projection on Y axis
    # tilt_lon = (2/3) * (s1 * sin(0°) + s2 * sin(120°) + s3 * sin(240°))
    # Note: s1 * sin(0°) = 0, so:
    tilt_lon = (2.0 / 3.0) * (s2 * _SIN_S2 + s3 * _SIN_S3)

    return collective, tilt_lon, tilt_lat


# ---------------------------------------------------------------------------
# Collective → blade pitch
# ---------------------------------------------------------------------------

def collective_to_pitch(collective_norm: float, col_max_rad: float) -> float:
    """
    Convert normalised collective [-1, 1] to blade collective pitch angle [rad].

    Maps linearly and symmetrically:
        -1.0 → -col_max_rad
         0.0 →  0.0 rad
        +1.0 → +col_max_rad

    Parameters
    ----------
    collective_norm : float   Normalised collective in [-1, 1]
    col_max_rad     : float   Maximum collective blade pitch angle [rad]

    Returns
    -------
    float   Collective blade pitch angle in radians
    """
    return float(np.clip(collective_norm * col_max_rad, -col_max_rad, col_max_rad))


# ---------------------------------------------------------------------------
# Cyclic pitch computation
# ---------------------------------------------------------------------------

def cyclic_to_blade_pitches(
    tilt_lon:       float,
    tilt_lat:       float,
    omega:          float,
    t:              float,
    pitch_gain_rad: float,
    collective:     float = 0.0,
) -> np.ndarray:
    """
    Compute individual blade pitch angles for a 4-blade rotor using cyclic
    pitch modulation.

    For 4 blades at 90° spacing, the cyclic pitch signal is:
        βi(t) = β̄ + U0 * sin(ω·t + (i-1)·π/2 + Ψ)
    where:
        β̄  = collective blade pitch [rad]
        U0  = cyclic amplitude = sqrt(tilt_lon² + tilt_lat²) * pitch_gain_rad [rad]
        Ψ   = phase of rotor disk tilt = atan2(tilt_lon, tilt_lat) [rad]
        i   = blade index 0..3 (blade 0 starts at East reference)

    Parameters
    ----------
    tilt_lon       : float   Normalised longitudinal tilt [-1, 1]
    tilt_lat       : float   Normalised lateral tilt [-1, 1]
    omega          : float   Rotor angular velocity [rad/s]
    t              : float   Simulation time [s]
    pitch_gain_rad : float   Max blade pitch per unit normalised tilt [rad]
    collective     : float   Collective blade pitch [rad] (default 0)

    Returns
    -------
    np.ndarray, shape (4,)   Blade pitch angles in radians [β0, β1, β2, β3]
    """
    tilt_lon_rad = tilt_lon * pitch_gain_rad
    tilt_lat_rad = tilt_lat * pitch_gain_rad

    # Cyclic amplitude and phase
    U0  = math.sqrt(tilt_lon_rad ** 2 + tilt_lat_rad ** 2)
    Psi = math.atan2(tilt_lon_rad, tilt_lat_rad)

    # Phase offsets for 4-blade 90° rotor: 0, π/2, π, 3π/2
    phase_offsets = np.array([0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi])

    blade_pitches = (collective
                     + U0 * np.sin(omega * t + phase_offsets + Psi))
    return blade_pitches


# ---------------------------------------------------------------------------
# Convenience: full servo → blade pitches pipeline
# ---------------------------------------------------------------------------

def servos_to_blade_pitches(
    servo_pwm:      np.ndarray,
    omega:          float,
    t:              float,
    pitch_gain_rad: float,
    col_max_rad:    float,
) -> tuple:
    """
    Full pipeline: servo PWM values → blade pitch angles.

    Expects servo_pwm[0..2] = S1, S2, S3 (swashplate).

    Parameters
    ----------
    servo_pwm      : array_like, length ≥ 3   PWM values in microseconds
    omega          : float   Rotor spin rate [rad/s]
    t              : float   Simulation time [s]
    pitch_gain_rad : float   Max blade pitch per unit normalised tilt [rad]
    col_max_rad    : float   Max collective blade pitch [rad]

    Returns
    -------
    (blade_pitches, collective_rad, tilt_lon, tilt_lat) : tuple
        blade_pitches : np.ndarray (4,)  — individual blade pitch [rad]
        collective_rad : float           — collective angle [rad]
        tilt_lon       : float           — normalised longitudinal tilt
        tilt_lat       : float           — normalised lateral tilt
    """
    pwm = np.asarray(servo_pwm, dtype=np.float64)
    s1 = pwm_to_normalized(pwm[0])
    s2 = pwm_to_normalized(pwm[1])
    s3 = pwm_to_normalized(pwm[2])

    collective_norm, tilt_lon, tilt_lat = h3_inverse_mix(s1, s2, s3)
    collective_rad = collective_to_pitch(collective_norm, col_max_rad)
    blade_pitches  = cyclic_to_blade_pitches(tilt_lon, tilt_lat, omega, t,
                                              pitch_gain_rad, collective_rad)
    return blade_pitches, collective_rad, tilt_lon, tilt_lat


# ---------------------------------------------------------------------------
# Servo slew rate limiter
# ---------------------------------------------------------------------------

def _slew_limit(current: float, target: float, max_step: float) -> float:
    """Move current toward target by at most max_step."""
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)


class SwashplateServoModel:
    """
    Stateful swashplate servo model with physical slew rate limiting.

    Models the DS113MG V6.0 servo speed through the H3-120 swashplate
    geometry to limit how fast tilt_lon and tilt_lat can change each step.

    Mirrors ArduPilot's SIM_SERVO_SPEED constraint so simtests and SITL
    apply identical servo bandwidth.  Both are derived from the same
    RotorDefinition fields (servo_slew_rate_deg_s, servo_travel_deg):

        SIM_SERVO_SPEED         = slew_rate_deg_s / travel_deg     [full-range/s]
        max_rate_norm [norm/s]  = 2 * SIM_SERVO_SPEED

    The factor of 2 converts ArduPilot's [0, 1] output range to our
    normalised tilt range [-1, +1].

    Example — DS113MG at 6 V (60 deg in 0.05 s = 1200 deg/s):
        SIM_SERVO_SPEED = 1200 / 60 = 20
        max_rate_norm   = 40 normalised units/s
        At 400 Hz (dt=0.0025 s): max step = 0.10 per step (10% of full range)
    """

    def __init__(self, slew_rate_deg_s: float, travel_deg: float):
        # normalised range [-1,1] = 2 units; ArduPilot range [0,1] = 1 unit
        self._max_rate = 2.0 * slew_rate_deg_s / travel_deg
        self._tilt_lon = 0.0
        self._tilt_lat = 0.0

    @classmethod
    def from_rotor(cls, rotor) -> "SwashplateServoModel":
        """Construct from a RotorDefinition."""
        return cls(
            slew_rate_deg_s=rotor.servo_slew_rate_deg_s,
            travel_deg=rotor.servo_travel_deg,
        )

    def step(self, tilt_lon_cmd: float, tilt_lat_cmd: float, dt: float):
        """
        Advance servo state by dt seconds toward commanded tilt.

        Parameters
        ----------
        tilt_lon_cmd, tilt_lat_cmd : float   Target normalised tilt [-1, 1]
        dt                         : float   Time step [s]

        Returns
        -------
        (tilt_lon, tilt_lat) : tuple of float — slew-rate-limited output
        """
        max_step = self._max_rate * dt
        self._tilt_lon = _slew_limit(self._tilt_lon, float(tilt_lon_cmd), max_step)
        self._tilt_lat = _slew_limit(self._tilt_lat, float(tilt_lat_cmd), max_step)
        return self._tilt_lon, self._tilt_lat

    def reset(self, tilt_lon: float = 0.0, tilt_lat: float = 0.0) -> None:
        """Reset servo state (call on mode entry or between simulation runs)."""
        self._tilt_lon = float(tilt_lon)
        self._tilt_lat = float(tilt_lat)

    @property
    def tilt_lon(self) -> float:
        return self._tilt_lon

    @property
    def tilt_lat(self) -> float:
        return self._tilt_lat


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    print("Swashplate smoke test")

    # PWM conversion
    assert pwm_to_normalized(1000) == -1.0, "1000µs should → -1.0"
    assert pwm_to_normalized(1500) ==  0.0, "1500µs should → 0.0"
    assert pwm_to_normalized(2000) ==  1.0, "2000µs should → +1.0"
    assert pwm_to_normalized(1250) == -0.5, "1250µs should → -0.5"
    print("  pwm_to_normalized: OK")

    # H3 inverse mix — all neutral
    coll, tlon, tlat = h3_inverse_mix(0.0, 0.0, 0.0)
    assert abs(coll) < 1e-10 and abs(tlon) < 1e-10 and abs(tlat) < 1e-10
    print("  h3_inverse_mix neutral: OK")

    # H3 inverse mix — pure collective
    coll, tlon, tlat = h3_inverse_mix(1.0, 1.0, 1.0)
    assert abs(coll - 1.0) < 1e-10
    assert abs(tlon) < 1e-10
    assert abs(tlat) < 1e-10
    print(f"  h3_inverse_mix collective=1.0: coll={coll:.4f} tlon={tlon:.4f} tlat={tlat:.4f} OK")

    # Collective to pitch
    _COL_MAX = 0.35
    assert abs(collective_to_pitch(0.0, _COL_MAX))  < 1e-10
    assert abs(collective_to_pitch(1.0, _COL_MAX) - _COL_MAX) < 1e-10
    assert abs(collective_to_pitch(-1.0, _COL_MAX) - (-_COL_MAX)) < 1e-10
    print("  collective_to_pitch: OK")

    _PITCH_GAIN = 0.3
    # Cyclic: zero tilt → all blades at collective only
    pitches = cyclic_to_blade_pitches(0.0, 0.0, 28.0, 0.0, _PITCH_GAIN, 0.1)
    assert np.allclose(pitches, 0.1), f"Zero tilt cyclic failed: {pitches}"
    print("  cyclic_to_blade_pitches (zero tilt): OK")

    # Cyclic: non-zero tilt
    pitches2 = cyclic_to_blade_pitches(0.3, 0.0, 28.0, 0.0, _PITCH_GAIN, 0.0)
    print(f"  cyclic with tilt_lon=0.3: {pitches2.round(4)}")
    assert not np.allclose(pitches2, 0.0), "Non-zero tilt should produce non-zero pitches"

    # Full pipeline
    pwm = np.array([1500, 1700, 1300, 1500] + [1500]*12, dtype=float)
    bp, coll_r, tl, tla = servos_to_blade_pitches(pwm, 28.0, 0.5, _PITCH_GAIN, _COL_MAX)
    print(f"  Full pipeline: coll={coll_r:.4f} tlon={tl:.4f} tlat={tla:.4f}")
    print(f"  Blade pitches: {bp.round(4)}")

    print("All smoke tests passed.")
    sys.exit(0)
