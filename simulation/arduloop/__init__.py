"""
arduloop — A Python port of ArduPilot's traditional-heli rate / attitude loop.

The block diagram, parameter names, and signal flow mirror
`libraries/AC_PID/AC_PID.cpp` and `libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp`
so that gains tuned here transfer directly to ArduPilot parameters
(`ATC_RAT_RLL_*`, `ATC_RAT_PIT_*`, `H_SW_H3_PHANG`, ...).

Public modules
--------------
- :mod:`arduloop.filters`        — first-order LPF, biquad notch (AP-style).
- :mod:`arduloop.pid`            — AC_PID port with target / error / derivative
                                   filters and target / error notches.
- :mod:`arduloop.swash`          — H3 swashplate phase rotation.
- :mod:`arduloop.attitude_heli`  — per-axis rate controller plus PIRO_COMP and
                                   hover-roll-trim, matching the heli wrapper.
- :mod:`arduloop.plant`          — coupled 2-axis rotational plant with optional
                                   pendulum and tether-spring modes.
- :mod:`arduloop.signals`        — chirp / multisine / step input generators.
- :mod:`arduloop.analysis`       — Bode, stability margins, step-response score.
- :mod:`arduloop.params`         — dataclass holding the full parameter set,
                                   with one-to-one ArduPilot names.
"""

from .params import RateAxisParams, HeliParams
from .filters import LowPassFilter1p, NotchFilter
from .pid import AC_PID
from .swash import SwashH3
from .attitude_heli import HeliRateController
from .plant import HeliPlant
from . import signals, analysis

__all__ = [
    "RateAxisParams",
    "HeliParams",
    "LowPassFilter1p",
    "NotchFilter",
    "AC_PID",
    "SwashH3",
    "HeliRateController",
    "HeliPlant",
    "signals",
    "analysis",
]
