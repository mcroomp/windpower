"""
simtest_ic.py — Load steady-state initial conditions for simtests.

All simtests that need a hub starting at the aerodynamic equilibrium
should call `load_ic()` rather than hardcoding position/velocity/spin values.

The JSON is generated (and regenerated) by test_generate_ic.py.  When the
aero model or rotor geometry changes, run test_generate_ic.py once to update it:

    .venv/Scripts/python.exe -m pytest
        simulation/tests/unit/test_generate_ic.py::test_create_ic -s

Usage
-----
    from simtest_ic import load_ic
    ic = load_ic()
    POS0         = ic.pos
    VEL0         = ic.vel
    R0           = ic.R0          # body-to-NED rotation
    BODY_Z0      = ic.R0[:, 2]   # disk normal at design orientation
    ORBIT_BZ0    = ic.orbit_bz   # pre-tilted disk normal for orbit tracking start
    OMEGA_SPIN0  = ic.omega_spin
    REST_LENGTH0 = ic.rest_length
    EQ_THRUST    = ic.eq_thrust   # equilibrium thrust [0..1] (~0.263)
    TRIM_TLAT    = ic.trim_tilt_lat
"""

from __future__ import annotations

from ic import IC, load_ic, load_ic_dict, IC_JSON_PATH

# Backward-compatible alias for the JSON path (old name used by some tests).
_JSON_PATH = IC_JSON_PATH

__all__ = ["IC", "load_ic", "load_ic_dict", "IC_JSON_PATH", "_JSON_PATH"]

