"""
arduloop — Python port of ArduPilot's traditional-heli attitude + rate stack.

Two layers
----------
GuidedAttitudeController  (guided.py)
    Outer attitude loop.  Faithfully ports AC_AttitudeControl::input_quaternion
    including the slewed _attitude_target state, input_shaping_angle, and the
    30/60-degree feedforward blending from attitude_controller_run_quat.
    Use whenever rawes.lua calls vehicle:set_target_angle_and_climbrate().

HeliRateController  (attitude_heli.py)
    Inner rate loop.  Ports AC_AttitudeControl_Heli: per-axis AC_PID with
    target/error notches + 3 LPFs, PIRO_COMP, hover-roll-trim, H_SW_H3_PHANG
    phase rotation.  Driven by GuidedAttitudeController or directly.

All parameter field names are 1:1 with ArduPilot so gains transfer directly
to a .parm file.  See README.md for the full parameter table and usage guide.

Public modules
--------------
- :mod:`arduloop.guided`         — GuidedAttitudeController, GuidedAttitudeParams
- :mod:`arduloop.attitude_heli`  — HeliRateController, HeliRateOutput
- :mod:`arduloop.params`         — RateAxisParams, HeliParams (all AP param names)
- :mod:`arduloop.pid`            — AC_PID port with target/error notches + FLTT/FLTE/FLTD
- :mod:`arduloop.swash`          — SwashH3 H_SW_H3_PHANG phase rotation
- :mod:`arduloop.filters`        — LowPassFilter1p, NotchFilter (AP biquad)
- :mod:`arduloop.plant`          — HeliPlant — coupled rotational + pendulum + spring
- :mod:`arduloop.signals`        — step, chirp, multisine, doublet generators
- :mod:`arduloop.analysis`       — empirical FRF, stability margins, step-response score
"""

from .params import RateAxisParams, HeliParams, make_roll_pitch_params, make_yaw_params
from .filters import LowPassFilter1p, NotchFilter
from .pid import AC_PID
from .swash import SwashH3
from .attitude_heli import HeliRateController
from .guided import GuidedAttitudeController, GuidedAttitudeParams
from .plant import HeliPlant
from .log_df_equiv import DataflashEquivRow, make_df_equiv_row, write_df_equiv_csv
from . import signals, analysis

__all__ = [
    "RateAxisParams",
    "HeliParams",
    "make_roll_pitch_params",
    "make_yaw_params",
    "LowPassFilter1p",
    "NotchFilter",
    "AC_PID",
    "SwashH3",
    "HeliRateController",
    "GuidedAttitudeController",
    "GuidedAttitudeParams",
    "HeliPlant",
    "DataflashEquivRow",
    "make_df_equiv_row",
    "write_df_equiv_csv",
    "signals",
    "analysis",
]
