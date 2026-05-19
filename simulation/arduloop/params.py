"""
Parameter dataclasses with field names mirroring ArduPilot parameters.

A `RateAxisParams` corresponds to one rate-PID group (`ATC_RAT_RLL_*`,
`ATC_RAT_PIT_*`, `ATC_RAT_YAW_*`).

`HeliParams` wraps the three axes plus the heli-specific extras
(`ATC_HOVR_ROL_TRM`, `ATC_PIRO_COMP`, `H_SW_H3_PHANG`).
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class RateAxisParams:
    # PID gains — `ATC_RAT_xxx_P / I / D / FF / IMAX`
    P:    float = 0.10
    I:    float = 0.10
    D:    float = 0.005
    FF:   float = 0.05
    IMAX: float = 0.3
    D_FF: float = 0.0
    PDMX: float = 0.0   # 0 disables the PD sum limit

    # Target / error / derivative low-pass cutoffs (Hz)
    # `ATC_RAT_xxx_FLTT / FLTE / FLTD`
    FLTT: float = 20.0
    FLTE: float = 0.0
    FLTD: float = 20.0

    # Target / error notch — `ATC_RAT_xxx_NTF / NEF`
    # In ArduPilot these are indices into the `FILT*` parameter bank.
    # Here we inline the notch configuration to keep the sim self-contained.
    NTF_center_hz:    float = 0.0   # 0 disables
    NTF_bandwidth_hz: float = 0.0
    NTF_attn_db:      float = 40.0

    NEF_center_hz:    float = 0.0
    NEF_bandwidth_hz: float = 0.0
    NEF_attn_db:      float = 40.0


@dataclass
class HeliParams:
    # Per-axis rate PIDs
    roll:  RateAxisParams = field(default_factory=RateAxisParams)
    pitch: RateAxisParams = field(default_factory=RateAxisParams)
    yaw:   RateAxisParams = field(default_factory=lambda: RateAxisParams(
        P=0.18, I=0.12, D=0.003, FF=0.024, IMAX=0.4, FLTT=20.0, FLTD=10.0))

    # Heli-specific — `ATC_HOVR_ROL_TRM`, `ATC_PIRO_COMP`
    HOVR_ROL_TRM_cd: float = 0.0    # centi-degrees, like ArduPilot
    PIRO_COMP_enabled: bool = False

    # Swashplate — `H_SW_H3_PHANG` (degrees)
    H_SW_H3_PHANG: float = 0.0

    # Loop rate (Hz). ArduPilot heli typical: 400.
    loop_rate_hz: float = 400.0

    # Output limit per axis [-1, 1] like motor mixer normalised cyclic.
    output_limit: float = 1.0
