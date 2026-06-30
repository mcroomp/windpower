"""
Parameter dataclasses with field names mirroring ArduPilot parameters.

A `RateAxisParams` corresponds to one rate-PID group (`ATC_RAT_RLL_*`,
`ATC_RAT_PIT_*`, `ATC_RAT_YAW_*`).

`HeliParams` wraps the three axes plus the heli-specific extras
(`ATC_HOVR_ROL_TRM`, `ATC_PIRO_COMP`, `H_SW_H3_PHANG`).
"""

from __future__ import annotations

from dataclasses import dataclass, field


def _attitude_defaults() -> dict[str, float]:
    from param_defaults import load_attitude_params

    return load_attitude_params()


@dataclass
class RateAxisParams:
    # PID gains — `ATC_RAT_xxx_P / I / D / FF / IMAX`
    # All gains are required — use ArduPilot parameter file as single source of truth.
    P:    float
    I:    float
    D:    float
    FF:   float
    IMAX: float
    # Target / error / derivative low-pass cutoffs (Hz)
    # `ATC_RAT_xxx_FLTT / FLTE / FLTD`
    FLTT: float
    FLTE: float
    FLTD: float

    # Optional fields with defaults (must come after required fields)
    D_FF: float = 0.0
    PDMX: float = 0.0   # 0 disables the PD sum limit

    # Slew-rate limiter (SMAX).  0 = disabled.  `ATC_RAT_xxx_SMAX`
    # When non-zero, limits the rate of change of P+D output to SMAX/s,
    # reducing oscillation.  SlewLimiter port from Filter/SlewLimiter.cpp.
    SMAX: float = 0.0
    # Slew-rate filter time constant (s).  Matches AP default initial_srtau=1.0.
    SRTAU: float = 1.0

    # Integrator leak minimum (helicopter only).  `ATC_RAT_xxx_ILMI`
    # The integrator leaks toward +-ILMI when there is no error.
    # 0 = no leak (standard behaviour).
    ILMI: float = 0.0

    # Target / error notch — `ATC_RAT_xxx_NTF / NEF`
    # In ArduPilot these are indices into the `FILT*` parameter bank.
    # Here we inline the notch configuration to keep the sim self-contained.
    NTF_center_hz:    float = 0.0   # 0 disables
    NTF_bandwidth_hz: float = 0.0
    NTF_attn_db:      float = 40.0

    NEF_center_hz:    float = 0.0
    NEF_bandwidth_hz: float = 0.0
    NEF_attn_db:      float = 40.0


def make_roll_params() -> RateAxisParams:
    """Factory for the roll rate PID (``ATC_RAT_RLL_*``).

    Every consumer -- unit tests, simtests, and SITL stack tests -- loads the
    same merged ``copter-heli.parm`` + ``rawes_sitl_defaults.parm`` chain, so
    the gains here are identical to what ArduPilot loads at boot. Tune the rate
    PID by editing ``rawes_sitl_defaults.parm``; do not hardcode gains here.

    See Also
    --------
    make_pitch_params : Pitch rate PID (``ATC_RAT_PIT_*``)
    make_yaw_params : Yaw rate PID (``ATC_RAT_YAW_*``)
    """
    from param_defaults import make_roll_params_from_file

    return make_roll_params_from_file()


# Backwards-compatible alias (historically returned the roll axis and was reused
# for pitch). Pitch now has its own factory; prefer make_roll_params /
# make_pitch_params in new code.
make_roll_pitch_params = make_roll_params


def make_pitch_params() -> RateAxisParams:
    """Factory for the pitch rate PID (``ATC_RAT_PIT_*``).

    Loaded from the same merged ``copter-heli.parm`` + ``rawes_sitl_defaults.parm``
    chain as roll/yaw. ArduPilot's pitch gains differ from roll, so simtests now
    use the pitch axis for pitch (previously roll was reused for both).
    """
    from param_defaults import make_pitch_params_from_file

    return make_pitch_params_from_file()


def make_yaw_params() -> RateAxisParams:
    """Factory for the yaw rate PID (``ATC_RAT_YAW_*``).

    Loaded from the same merged ``copter-heli.parm`` + ``rawes_sitl_defaults.parm``
    chain as roll/pitch. Tune via the .parm file, not in Python.

    See Also
    --------
    make_roll_params : Roll rate PID (``ATC_RAT_RLL_*``)
    make_pitch_params : Pitch rate PID (``ATC_RAT_PIT_*``)
    """
    from param_defaults import make_yaw_params_from_file

    return make_yaw_params_from_file()


@dataclass
class HeliParams:
    # Per-axis rate PIDs — loaded from ArduPilot parameter file by default.
    roll: RateAxisParams = field(default_factory=make_roll_params)
    pitch: RateAxisParams = field(default_factory=make_pitch_params)
    yaw: RateAxisParams = field(default_factory=make_yaw_params)

    # Heli-specific — `ATC_HOVR_ROL_TRM`, `ATC_PIRO_COMP`
    HOVR_ROL_TRM_cd: float | None = None    # centi-degrees, like ArduPilot
    PIRO_COMP_enabled: bool | None = None

    # Swashplate — `H_SW_H3_PHANG` (degrees)
    H_SW_H3_PHANG: float | None = None

    # Loop rate (Hz). ArduPilot heli typical: 400.
    loop_rate_hz: float = 400.0

    # Output limit per axis [-1, 1] like motor mixer normalised cyclic.
    output_limit: float = 1.0

    # -----------------------------------------------------------------------
    # Attitude (outer) P-loop gains — `ATC_ANG_RLL_P / PIT_P / YAW_P`
    # Loaded from ArduPilot .parm files when unset.
    # -----------------------------------------------------------------------
    ATC_ANG_RLL_P: float | None = None
    ATC_ANG_PIT_P: float | None = None
    ATC_ANG_YAW_P: float | None = None

    # Angular acceleration limits for the sqrt-controller.
    # AP 4.7 names: `ATC_ACC_*_MAX` in deg/s^2.
    # Legacy fallback names: `ATC_ACCEL_*_MAX` in centi-deg/s^2.
    # Values are loaded from .parm files when unset.
    ATC_ACCEL_R_MAX: float | None = None
    ATC_ACCEL_P_MAX: float | None = None
    ATC_ACCEL_Y_MAX: float | None = None

    # Maximum body-frame angular velocity (deg/s). 0 = unlimited.
    # `ATC_RATE_R_MAX`, `ATC_RATE_P_MAX`, `ATC_RATE_Y_MAX`
    ATC_RATE_R_MAX: float | None = None
    ATC_RATE_P_MAX: float | None = None
    ATC_RATE_Y_MAX: float | None = None

    # Input shaping time constant (s).  `ATC_INPUT_TC`  AP default: 0.15
    ATC_INPUT_TC: float | None = None

    def __post_init__(self) -> None:
        ap = _attitude_defaults()

        if self.HOVR_ROL_TRM_cd is None:
            self.HOVR_ROL_TRM_cd = ap["ATC_HOVR_ROL_TRM"]
        if self.PIRO_COMP_enabled is None:
            self.PIRO_COMP_enabled = bool(ap["ATC_PIRO_COMP"])
        if self.H_SW_H3_PHANG is None:
            self.H_SW_H3_PHANG = ap["H_SW_H3_PHANG"]

        if self.ATC_ANG_RLL_P is None:
            self.ATC_ANG_RLL_P = ap["ATC_ANG_RLL_P"]
        if self.ATC_ANG_PIT_P is None:
            self.ATC_ANG_PIT_P = ap["ATC_ANG_PIT_P"]
        if self.ATC_ANG_YAW_P is None:
            self.ATC_ANG_YAW_P = ap["ATC_ANG_YAW_P"]

        if self.ATC_ACCEL_R_MAX is None:
            self.ATC_ACCEL_R_MAX = ap["ATC_ACCEL_R_MAX"]
        if self.ATC_ACCEL_P_MAX is None:
            self.ATC_ACCEL_P_MAX = ap["ATC_ACCEL_P_MAX"]
        if self.ATC_ACCEL_Y_MAX is None:
            self.ATC_ACCEL_Y_MAX = ap["ATC_ACCEL_Y_MAX"]

        if self.ATC_RATE_R_MAX is None:
            self.ATC_RATE_R_MAX = ap["ATC_RATE_R_MAX"]
        if self.ATC_RATE_P_MAX is None:
            self.ATC_RATE_P_MAX = ap["ATC_RATE_P_MAX"]
        if self.ATC_RATE_Y_MAX is None:
            self.ATC_RATE_Y_MAX = ap["ATC_RATE_Y_MAX"]

        if self.ATC_INPUT_TC is None:
            self.ATC_INPUT_TC = ap["ATC_INPUT_TC"]
