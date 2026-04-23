"""
swashplate.py — ArduPilot H3-120 swashplate mixing (faithful Python clone)

Mirrors AP_MotorsHeli_Swash.cpp and AP_MotorsHeli_Single.cpp exactly so that
simtests and the SITL mediator use identical swashplate arithmetic.

Reference: hardware/ardupilot_swashplate.md

ArduPilot H3-120 servo layout (AP_MotorsHeli_Swash.cpp:151-153):
    SERVO1 (CH_1) at -60 deg
    SERVO2 (CH_2) at +60 deg
    SERVO3 (CH_3) at 180 deg

Cyclic gain 0.45 (AP_MotorsHeli_Swash.cpp:196-198):
    rollFactor[i]  = cos(azimuth + 90 deg) * 0.45
    pitchFactor[i] = cos(azimuth)          * 0.45
    collectiveFactor = 1.0  (all servos)

H_COL_MIN / H_COL_MAX scaling (AP_MotorsHeli_Single.cpp:415-416):
    collective_scalar     = (H_COL_MAX - H_COL_MIN) * 0.001
    collective_out_scaled = collective_out * collective_scalar
                          + (H_COL_MIN - 1000) * 0.001

Normalized domain used throughout this module:
    s[i]  = output_rescaled[i]  in [-1..+1]
           = (raw_pwm - 1500) / 500  (with default SERVO#_TRIM = 1500)
           = SITL servo output value directly

This is the single source of truth for swashplate arithmetic.
"""

import math
import numpy as np


# ---------------------------------------------------------------------------
# ArduPilot H3-120 mixing constants
# ---------------------------------------------------------------------------

_CYCLIC_GAIN = 0.45   # AP_MotorsHeli_Swash.cpp:196

# Servo azimuths: CH1=-60°, CH2=+60°, CH3=180°
_AZIMUTHS_DEG = (-60.0, 60.0, 180.0)

# rollFactor[i]  = cos(azimuth + 90°) * 0.45
_ROLL_F  = tuple(math.cos(math.radians(a + 90.0)) * _CYCLIC_GAIN for a in _AZIMUTHS_DEG)
# = (+0.3897, -0.3897, 0.0)

# pitchFactor[i] = cos(azimuth) * 0.45
_PITCH_F = tuple(math.cos(math.radians(a)) * _CYCLIC_GAIN for a in _AZIMUTHS_DEG)
# = (+0.225, +0.225, -0.45)


# ---------------------------------------------------------------------------
# Forward mix: (collective_out, roll, pitch) → normalized servo outputs
# ---------------------------------------------------------------------------

def ardupilot_h3_120_forward(
    collective_out: float,
    roll_norm:      float,
    pitch_norm:     float,
    h_col_min:      float = 1000.0,
    h_col_max:      float = 2000.0,
) -> tuple:
    """
    ArduPilot H3-120 forward swashplate mix.

    Mirrors move_actuators() in AP_MotorsHeli_Single.cpp and
    calculate() in AP_MotorsHeli_Swash.cpp.

    Parameters
    ----------
    collective_out : float  Collective from RC3 in ACRO mode, linear [0..1].
                            0 = RC3 at 1000 µs (H_COL_MIN),
                            1 = RC3 at 2000 µs (H_COL_MAX).
    roll_norm      : float  Roll cyclic command [-1..+1].
    pitch_norm     : float  Pitch cyclic command [-1..+1].
    h_col_min      : float  H_COL_MIN parameter [µs], default 1000.
    h_col_max      : float  H_COL_MAX parameter [µs], default 2000.

    Returns
    -------
    (s1, s2, s3) : tuple of float
        Normalized servo outputs in [-1..+1].
        s[i] = output_rescaled[i] = (raw_pwm - 1500) / 500 for TRIM=1500.
        This is also the value ArduPilot sends over the SITL UDP socket.
    """
    collective_scalar = (h_col_max - h_col_min) * 0.001
    collective_offset = (h_col_min - 1000.0) * 0.001
    col_scaled = collective_out * collective_scalar + collective_offset

    out = []
    for i in range(3):
        output = _ROLL_F[i] * roll_norm + _PITCH_F[i] * pitch_norm + col_scaled
        out.append(2.0 * output - 1.0)   # rescale [0..1] → [-1..+1]

    return (out[0], out[1], out[2])


# ---------------------------------------------------------------------------
# Inverse mix: normalized servo outputs → (collective_out, roll, pitch)
# ---------------------------------------------------------------------------

def ardupilot_h3_120_inverse(
    s1:        float,
    s2:        float,
    s3:        float,
    h_col_min: float = 1000.0,
    h_col_max: float = 2000.0,
) -> tuple:
    """
    ArduPilot H3-120 inverse swashplate mix.

    Recovers the collective_out, roll_norm, pitch_norm that produced
    the given servo outputs.  Exact inverse of ardupilot_h3_120_forward.

    Parameters
    ----------
    s1, s2, s3 : float
        Normalized servo outputs in [-1..+1].
        Pass SITL values directly, or apply pwm_to_normalized() to MAVLink
        SERVO_OUTPUT_RAW values first (assumes SERVO#_TRIM = 1500).
    h_col_min  : float  H_COL_MIN parameter [µs], default 1000.
    h_col_max  : float  H_COL_MAX parameter [µs], default 2000.

    Returns
    -------
    (collective_out, roll_norm, pitch_norm) : tuple of float
        collective_out in [0..1], roll_norm and pitch_norm in [-1..+1].
    """
    # Undo the 2x-1 rescale: output[i] = (s[i] + 1) / 2  ∈ [0..1]
    o1 = (s1 + 1.0) * 0.5
    o2 = (s2 + 1.0) * 0.5
    o3 = (s3 + 1.0) * 0.5

    # Collective: sum of roll/pitch factors is zero, so average = collective
    col_scaled = (o1 + o2 + o3) / 3.0

    # Roll: CH1 and CH2 are symmetric about the roll axis
    #   o1 - o2 = 2 * ROLL_F[0] * roll  (ROLL_F[1] = -ROLL_F[0])
    roll_norm = (o1 - o2) / (2.0 * _ROLL_F[0])   # 2 * 0.3897

    # Pitch: CH3 has no roll factor, pitch factor = PITCH_F[2] = -0.45
    #   o3 = PITCH_F[2]*pitch + col_scaled  →  pitch = (col_scaled - o3) / 0.45
    pitch_norm = (col_scaled - o3) / (-_PITCH_F[2])   # / 0.45

    # Undo H_COL_MIN/H_COL_MAX scaling to recover collective_out [0..1]
    collective_scalar = (h_col_max - h_col_min) * 0.001
    collective_offset = (h_col_min - 1000.0) * 0.001
    if collective_scalar != 0.0:
        collective_out = (col_scaled - collective_offset) / collective_scalar
    else:
        collective_out = 0.5

    return (collective_out, roll_norm, pitch_norm)


# ---------------------------------------------------------------------------
# Collective angle conversion
# ---------------------------------------------------------------------------

def collective_out_to_rad(
    collective_out: float,
    col_min_rad:    float,
    col_max_rad:    float,
) -> float:
    """
    Map ArduPilot collective_out [0..1] to physical blade pitch [rad].

    Mirrors Lua's encoding:
        thrust = (col_rad - col_min) / (col_max - col_min)
        ch3    = 1000 + thrust * 1000
    → collective_out = thrust ∈ [0..1].

    Parameters
    ----------
    collective_out : float  ArduPilot collective [0..1].
    col_min_rad    : float  Physical collective floor (e.g. -0.28 rad).
    col_max_rad    : float  Physical collective ceiling (e.g. +0.10 rad).

    Returns
    -------
    float  Blade collective pitch [rad], clamped to [col_min_rad, col_max_rad].
    """
    rad = col_min_rad + float(np.clip(collective_out, 0.0, 1.0)) * (col_max_rad - col_min_rad)
    return float(np.clip(rad, col_min_rad, col_max_rad))


def collective_rad_to_out(
    collective_rad: float,
    col_min_rad:    float,
    col_max_rad:    float,
) -> float:
    """
    Map physical blade pitch [rad] to ArduPilot collective_out [0..1].

    Inverse of collective_out_to_rad.

    Parameters
    ----------
    collective_rad : float  Blade collective pitch [rad].
    col_min_rad    : float  Physical collective floor [rad].
    col_max_rad    : float  Physical collective ceiling [rad].

    Returns
    -------
    float  collective_out in [0..1], clamped.
    """
    span = col_max_rad - col_min_rad
    if span == 0.0:
        return 0.5
    return float(np.clip((collective_rad - col_min_rad) / span, 0.0, 1.0))


# ---------------------------------------------------------------------------
# PWM utility
# ---------------------------------------------------------------------------

def pwm_to_normalized(pwm_us: float) -> float:
    """
    Convert PWM microseconds to the ArduPilot output_rescaled value.

    output_rescaled = (raw_pwm - 1500) / 500, which equals s[i] in the
    ArduPilot forward mix (assuming SERVO#_TRIM = 1500).

    1000 µs → -1.0
    1500 µs →  0.0
    2000 µs → +1.0
    """
    return float(np.clip((pwm_us - 1500.0) / 500.0, -1.0, 1.0))


def normalized_to_pwm(s: float) -> float:
    """Inverse of pwm_to_normalized: output_rescaled → raw PWM µs (TRIM=1500)."""
    return float(np.clip(1500.0 + 500.0 * s, 1000.0, 2000.0))


# ---------------------------------------------------------------------------
# Cyclic blade pitch computation (aero model interface)
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

    For 4 blades at 90° spacing:
        βi(t) = β̄ + U0 * sin(ω·t + (i-1)·π/2 + Ψ)
    where:
        β̄  = collective blade pitch [rad]
        U0  = cyclic amplitude = sqrt(tilt_lon² + tilt_lat²) * pitch_gain_rad
        Ψ   = phase = atan2(tilt_lon, tilt_lat)

    tilt_lon and tilt_lat are ArduPilot pitch_norm / roll_norm [-1..+1] passed
    directly from ardupilot_h3_120_inverse.

    Parameters
    ----------
    tilt_lon       : float  Pitch cyclic (= ArduPilot pitch_norm) [-1..+1]
    tilt_lat       : float  Roll cyclic  (= ArduPilot roll_norm)  [-1..+1]
    omega          : float  Rotor spin rate [rad/s]
    t              : float  Simulation time [s]
    pitch_gain_rad : float  Physical cyclic gain [rad / normalised unit]
    collective     : float  Collective blade pitch [rad]

    Returns
    -------
    np.ndarray, shape (4,)  Blade pitch angles [rad].
    """
    tilt_lon_rad = tilt_lon * pitch_gain_rad
    tilt_lat_rad = tilt_lat * pitch_gain_rad

    U0  = math.sqrt(tilt_lon_rad ** 2 + tilt_lat_rad ** 2)
    Psi = math.atan2(tilt_lon_rad, tilt_lat_rad)

    phase_offsets = np.array([0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi])
    return collective + U0 * np.sin(omega * t + phase_offsets + Psi)


# ---------------------------------------------------------------------------
# Servo slew rate limiter (unchanged from original)
# ---------------------------------------------------------------------------

def _slew_limit(current: float, target: float, max_step: float) -> float:
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)


class SwashplateServoModel:
    """
    Stateful swashplate servo model with physical slew rate limiting.

    Models the DS113MG V6.0 servo speed through the H3-120 swashplate
    geometry to limit how fast tilt_lon, tilt_lat, and collective can change
    each step.

    Mirrors ArduPilot's SIM_SERVO_SPEED constraint so simtests and SITL
    apply identical servo bandwidth.  Both are derived from the same
    RotorDefinition fields (servo_slew_rate_deg_s, servo_travel_deg):

        SIM_SERVO_SPEED         = slew_rate_deg_s / travel_deg     [full-range/s]
        max_rate_norm [norm/s]  = 2 * SIM_SERVO_SPEED

    The factor of 2 converts ArduPilot's [0, 1] output range to our
    normalised tilt range [-1, +1].
    """

    def __init__(
        self,
        slew_rate_deg_s:    float,
        travel_deg:         float,
        col_pitch_gain_rad: float,
    ):
        self._max_rate     = 2.0 * slew_rate_deg_s / travel_deg
        self._max_col_rate = self._max_rate * col_pitch_gain_rad
        self._tilt_lon     = 0.0
        self._tilt_lat     = 0.0
        self._collective_rad = 0.0

    @classmethod
    def from_rotor(cls, rotor) -> "SwashplateServoModel":
        return cls(
            slew_rate_deg_s=rotor.servo_slew_rate_deg_s,
            travel_deg=rotor.servo_travel_deg,
            col_pitch_gain_rad=rotor.swashplate_pitch_gain_rad,
        )

    def step(self, tilt_lon_cmd: float, tilt_lat_cmd: float, dt: float):
        max_step = self._max_rate * dt
        self._tilt_lon = _slew_limit(self._tilt_lon, float(tilt_lon_cmd), max_step)
        self._tilt_lat = _slew_limit(self._tilt_lat, float(tilt_lat_cmd), max_step)
        return self._tilt_lon, self._tilt_lat

    def step_collective(self, collective_rad_cmd: float, dt: float) -> float:
        max_step = self._max_col_rate * dt
        self._collective_rad = _slew_limit(
            self._collective_rad, float(collective_rad_cmd), max_step
        )
        return self._collective_rad

    def reset(self, tilt_lon=0.0, tilt_lat=0.0, collective_rad=0.0):
        self._tilt_lon       = float(tilt_lon)
        self._tilt_lat       = float(tilt_lat)
        self._collective_rad = float(collective_rad)

    @property
    def tilt_lon(self): return self._tilt_lon

    @property
    def tilt_lat(self): return self._tilt_lat

    @property
    def collective_rad(self): return self._collective_rad


# ---------------------------------------------------------------------------
# Smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    COL_MIN = -0.28
    COL_MAX =  0.10

    print("ArduPilot H3-120 swashplate smoke test")

    # pwm_to_normalized
    assert pwm_to_normalized(1000) == -1.0
    assert pwm_to_normalized(1500) ==  0.0
    assert pwm_to_normalized(2000) ==  1.0
    print("  pwm_to_normalized: OK")

    # Round-trip: pure collective
    for col_out in (0.0, 0.263, 0.5, 1.0):
        s1, s2, s3 = ardupilot_h3_120_forward(col_out, 0.0, 0.0)
        col_rt, r, p = ardupilot_h3_120_inverse(s1, s2, s3)
        assert abs(col_rt - col_out) < 1e-10, f"col round-trip failed: {col_out} -> {col_rt}"
        assert abs(r) < 1e-10, f"spurious roll from pure collective: {r}"
        assert abs(p) < 1e-10, f"spurious pitch from pure collective: {p}"
    print("  pure collective round-trip: OK")

    # Round-trip: pure roll
    for roll in (-1.0, -0.4, 0.0, 0.4, 1.0):
        s1, s2, s3 = ardupilot_h3_120_forward(0.5, roll, 0.0)
        col_rt, r, p = ardupilot_h3_120_inverse(s1, s2, s3)
        assert abs(col_rt - 0.5) < 1e-10, f"collective disturbed by roll: {col_rt}"
        assert abs(r - roll) < 1e-10,     f"roll round-trip failed: {roll} -> {r}"
        assert abs(p) < 1e-10,            f"spurious pitch from pure roll: {p}"
    print("  pure roll round-trip: OK")

    # Round-trip: pure pitch
    for pitch in (-1.0, -0.4, 0.0, 0.4, 1.0):
        s1, s2, s3 = ardupilot_h3_120_forward(0.5, 0.0, pitch)
        col_rt, r, p = ardupilot_h3_120_inverse(s1, s2, s3)
        assert abs(col_rt - 0.5) < 1e-10, f"collective disturbed by pitch: {col_rt}"
        assert abs(r) < 1e-10,            f"spurious roll from pure pitch: {r}"
        assert abs(p - pitch) < 1e-10,    f"pitch round-trip failed: {pitch} -> {p}"
    print("  pure pitch round-trip: OK")

    # Known value: Lua COL_CRUISE = -0.18 rad -> col_out = 0.263 -> s = -0.474
    col_out = collective_rad_to_out(-0.18, COL_MIN, COL_MAX)
    assert abs(col_out - 0.263158) < 1e-5, f"col_out wrong: {col_out}"
    s1, s2, s3 = ardupilot_h3_120_forward(col_out, 0.0, 0.0)
    assert abs(s1 - (-0.473684)) < 1e-5, f"s1 wrong: {s1}"
    assert abs(s1 - s2) < 1e-10 and abs(s1 - s3) < 1e-10, "pure collective: all servos equal"
    pwm = normalized_to_pwm(s1)
    assert abs(pwm - 1263.158) < 0.1, f"PWM wrong: {pwm}"
    print(f"  col=-0.18 rad -> col_out={col_out:.4f} -> s={s1:.4f} -> PWM={pwm:.1f} us: OK")

    # collective_out_to_rad / collective_rad_to_out round-trip
    for rad in (-0.28, -0.18, 0.0, 0.10):
        out = collective_rad_to_out(rad, COL_MIN, COL_MAX)
        back = collective_out_to_rad(out, COL_MIN, COL_MAX)
        assert abs(back - rad) < 1e-12, f"rad round-trip failed: {rad} -> {out} -> {back}"
    print("  collective rad/out round-trip: OK")

    print("All smoke tests passed.")
    sys.exit(0)
