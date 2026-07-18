"""
Parameter dataclasses with field names mirroring ArduPilot parameters.

A `RateAxisParams` corresponds to one rate-PID group (`ATC_RAT_RLL_*`,
`ATC_RAT_PIT_*`, `ATC_RAT_YAW_*`).

`HeliParams` wraps the three axes plus the heli-specific extras
(`ATC_HOVR_ROL_TRM`, `ATC_PIRO_COMP`, `H_SW_H3_PHANG`).

Construction
------------
Pass a flat AP params dict to build from .parm files::

    from simulation.param_defaults import load_ap_params
    hp = HeliParams.from_ap_dict(load_ap_params())
    rp = RateAxisParams.from_ap_dict(load_ap_params(), "RLL")

Or construct explicitly with all rate axes required::

    hp = HeliParams(roll=rp, pitch=rp, yaw=yp)
"""

from __future__ import annotations

from dataclasses import dataclass, field


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

    @classmethod
    def from_ap_dict(cls, params: dict[str, float], axis: str) -> "RateAxisParams":
        """Build from a flat AP params dict. axis is 'RLL', 'PIT', or 'YAW'."""
        p = axis
        return cls(
            P    = float(params[f"ATC_RAT_{p}_P"]),
            I    = float(params[f"ATC_RAT_{p}_I"]),
            D    = float(params[f"ATC_RAT_{p}_D"]),
            FF   = float(params[f"ATC_RAT_{p}_FF"]),
            IMAX = float(params[f"ATC_RAT_{p}_IMAX"]),
            FLTT = float(params[f"ATC_RAT_{p}_FLTT"]),
            FLTE = float(params[f"ATC_RAT_{p}_FLTE"]),
            FLTD = float(params[f"ATC_RAT_{p}_FLTD"]),
            D_FF = float(params.get(f"ATC_RAT_{p}_D_FF", 0.0)),
            PDMX = float(params.get(f"ATC_RAT_{p}_PDMX", 0.0)),
            SMAX = float(params.get(f"ATC_RAT_{p}_SMAX", 0.0)),
            ILMI = float(params.get(f"ATC_RAT_{p}_ILMI", 0.0)),
        )


@dataclass
class HeliParams:
    # Per-axis rate PIDs.  Use HeliParams.from_ap_dict(load_ap_params()) to
    # construct from ArduPilot .parm files; or provide all three axes explicitly.
    roll:  RateAxisParams
    pitch: RateAxisParams
    yaw:   RateAxisParams

    # Heli-specific — `ATC_HOVR_ROL_TRM`, `ATC_PIRO_COMP`
    HOVR_ROL_TRM_cd:  float = 0.0   # centi-degrees, like ArduPilot
    PIRO_COMP_enabled: bool = False

    # Swashplate — `H_SW_H3_PHANG` (degrees)
    H_SW_H3_PHANG: float = 0.0

    # Loop rate (Hz). ArduPilot heli typical: 400.
    loop_rate_hz: float = 400.0

    # Output limit per axis [-1, 1] like motor mixer normalised cyclic.
    output_limit: float = 1.0

    # `H_CYC_MAX` (centi-degrees) — combined roll/pitch cyclic magnitude above
    # which AP_MotorsHeli_Single::move_actuators rescales roll/pitch down and
    # sets `_motors.limit.roll/pitch` (anti-windup feedback for next tick).
    # AP default AP_MOTORS_HELI_SWASH_CYCLIC_MAX = 2500 centi-degrees.
    CYC_MAX_cd: float = 2500.0

    # Attitude (outer) P-loop gains — `ATC_ANG_RLL_P / PIT_P / YAW_P`.
    # 0.0 = no outer-loop correction (safe default for ACRO-only use).
    ATC_ANG_RLL_P: float = 0.0
    ATC_ANG_PIT_P: float = 0.0
    ATC_ANG_YAW_P: float = 0.0

    # Angular acceleration limits for the sqrt-controller.
    # AP 4.7 names: `ATC_ACC_*_MAX` in deg/s^2.  0 = linear P only.
    ATC_ACCEL_R_MAX: float = 0.0
    ATC_ACCEL_P_MAX: float = 0.0
    ATC_ACCEL_Y_MAX: float = 0.0

    # Maximum body-frame angular velocity (deg/s). 0 = unlimited.
    # `ATC_RATE_R_MAX`, `ATC_RATE_P_MAX`, `ATC_RATE_Y_MAX`
    ATC_RATE_R_MAX: float = 0.0
    ATC_RATE_P_MAX: float = 0.0
    ATC_RATE_Y_MAX: float = 0.0

    # Input shaping time constant (s).  `ATC_INPUT_TC`  0 = no shaping.
    ATC_INPUT_TC: float = 0.0

    @classmethod
    def from_ap_dict(cls, params: dict[str, float], **overrides) -> "HeliParams":
        """Build from a flat AP params dict (e.g. from param_defaults.load_ap_params()).

        Pass ``**overrides`` to set non-AP fields like ``loop_rate_hz``.
        """
        def _f(key: str, default: float = 0.0) -> float:
            return float(params.get(key, default))
        return cls(
            roll  = RateAxisParams.from_ap_dict(params, "RLL"),
            pitch = RateAxisParams.from_ap_dict(params, "PIT"),
            yaw   = RateAxisParams.from_ap_dict(params, "YAW"),
            HOVR_ROL_TRM_cd   = _f("ATC_HOVR_ROL_TRM"),
            PIRO_COMP_enabled = bool(_f("ATC_PIRO_COMP")),
            H_SW_H3_PHANG     = _f("H_SW_H3_PHANG") or _f("H3_PHANG"),
            ATC_ANG_RLL_P     = _f("ATC_ANG_RLL_P"),
            ATC_ANG_PIT_P     = _f("ATC_ANG_PIT_P"),
            ATC_ANG_YAW_P     = _f("ATC_ANG_YAW_P"),
            ATC_ACCEL_R_MAX   = _f("ATC_ACC_R_MAX") or _f("ATC_ACCEL_R_MAX"),
            ATC_ACCEL_P_MAX   = _f("ATC_ACC_P_MAX") or _f("ATC_ACCEL_P_MAX"),
            ATC_ACCEL_Y_MAX   = _f("ATC_ACC_Y_MAX") or _f("ATC_ACCEL_Y_MAX"),
            ATC_RATE_R_MAX    = _f("ATC_RATE_R_MAX"),
            ATC_RATE_P_MAX    = _f("ATC_RATE_P_MAX"),
            ATC_RATE_Y_MAX    = _f("ATC_RATE_Y_MAX"),
            ATC_INPUT_TC      = _f("ATC_INPUT_TC"),
            CYC_MAX_cd        = _f("H_CYC_MAX", 2500.0),
            **overrides,
        )
