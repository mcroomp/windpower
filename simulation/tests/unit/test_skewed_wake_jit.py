"""
test_skewed_wake_jit.py — Verify SkewedWakeBEMJit matches SkewedWakeBEM.

These tests guard against numerical drift between the JIT-compiled kernel
and the numpy reference implementation.  The two versions must agree to
within floating-point rounding (~1e-12 absolute) on forces, moments, and
all diagnostic attributes, across a representative sweep of operating
conditions (collective, tilt, RPM, inflow angle, ramp factor).
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aero import SkewedWakeBEM, SkewedWakeBEMJit
import rotor_definition as rd

# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def rotor():
    return rd.default()


@pytest.fixture(scope="module")
def ref(rotor):
    return SkewedWakeBEM(rotor)


@pytest.fixture(scope="module")
def jit(rotor):
    return SkewedWakeBEMJit(rotor)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _R_tilted(tilt_deg: float) -> np.ndarray:
    """Hub rotation matrix tilted tilt_deg from vertical (NED Z-down)."""
    a  = math.radians(tilt_deg)
    ca, sa = math.cos(a), math.sin(a)
    return np.array([[-ca, 0.0, sa],
                     [0.0, 1.0, 0.0],
                     [-sa, 0.0, -ca]])


def _call_both(ref, jit, collective, tilt_lon, tilt_lat,
               R_hub, v_hub, omega, wind, t, spin_angle=0.0):
    r = ref.compute_forces(collective, tilt_lon, tilt_lat,
                           R_hub, v_hub, omega, wind, t, spin_angle)
    g = jit.compute_forces(collective, tilt_lon, tilt_lat,
                           R_hub, v_hub, omega, wind, t, spin_angle)
    return r, g


def _assert_close(ref_result, jit_result, *, atol=1e-10, label=""):
    prefix = f"[{label}] " if label else ""
    assert np.allclose(ref_result.F_world,   jit_result.F_world,   atol=atol), \
        f"{prefix}F_world mismatch: ref={ref_result.F_world} jit={jit_result.F_world}"
    assert np.allclose(ref_result.M_orbital, jit_result.M_orbital, atol=atol), \
        f"{prefix}M_orbital mismatch: ref={ref_result.M_orbital} jit={jit_result.M_orbital}"
    assert np.allclose(ref_result.M_spin,    jit_result.M_spin,    atol=atol), \
        f"{prefix}M_spin mismatch: ref={ref_result.M_spin} jit={jit_result.M_spin}"
    assert abs(ref_result.Q_spin - jit_result.Q_spin) < atol, \
        f"{prefix}Q_spin mismatch: ref={ref_result.Q_spin} jit={jit_result.Q_spin}"


# ---------------------------------------------------------------------------
# Tests — operating-point sweep
# ---------------------------------------------------------------------------

# (collective_rad, tilt_lon, tilt_lat, omega_rad_s, wind_ned, t, label)
_CASES = [
    (0.05,  0.10,  0.05,  3.5, [0.0, 8.0,  0.0], 10.0, "nominal"),
    (0.08,  0.00,  0.00,  4.0, [0.0, 10.0, 0.5], 10.0, "axial inflow"),
    (0.10,  0.20,  0.10,  2.0, [1.0,  6.0, -0.5], 10.0, "high tilt"),
    (0.02,  0.05,  0.05,  0.5, [0.0,  3.0,  0.0], 10.0, "low rpm"),
    (0.00,  0.00,  0.00,  5.0, [0.0, 12.0,  0.0], 10.0, "zero collective"),
    (-0.05, 0.15, -0.10,  3.0, [0.5,  9.0,  0.3], 10.0, "negative collective"),
    (0.10,  0.00,  0.00,  6.0, [0.0, 15.0,  0.0], 10.0, "high wind"),
    (0.05,  0.10,  0.05,  3.5, [0.0,  8.0,  0.0],  2.0, "mid ramp"),
    (0.05,  0.10,  0.05,  3.5, [0.0,  8.0,  0.0],  0.5, "early ramp"),
]


@pytest.mark.parametrize("col,tlon,tlat,om,wind,t,label", _CASES)
def test_forces_match(ref, jit, col, tlon, tlat, om, wind, t, label):
    R_hub = _R_tilted(55.0)
    v_hub = np.array([0.0, 2.0, 0.5])
    r, g  = _call_both(ref, jit, col, tlon, tlat,
                       R_hub, v_hub, om, np.array(wind), t)
    _assert_close(r, g, label=label)


def test_spin_angle_variation(ref, jit):
    """Results must be consistent as spin_angle varies over a full revolution."""
    R_hub = _R_tilted(45.0)
    v_hub = np.array([0.0, 1.5, 0.0])
    wind  = np.array([0.0, 8.0, 0.0])
    for spin_angle in np.linspace(0, 2 * math.pi, 8, endpoint=False):
        r, g = _call_both(ref, jit, 0.05, 0.08, 0.03,
                          R_hub, v_hub, 3.5, wind, 10.0, spin_angle)
        _assert_close(r, g, label=f"spin={math.degrees(spin_angle):.0f}deg")


def test_zero_wind(ref, jit):
    """Both models must return zero forces at zero wind and zero rpm."""
    R_hub = np.eye(3)
    r, g  = _call_both(ref, jit, 0.0, 0.0, 0.0,
                       R_hub, np.zeros(3), 0.0, np.zeros(3), 10.0)
    _assert_close(r, g, label="zero wind")
    assert np.allclose(r.F_world, 0.0, atol=1e-10), "expected zero force at zero wind/rpm"


def test_ramp_zero(ref, jit):
    """Both models must return zero forces during the t=0 ramp phase."""
    R_hub = _R_tilted(45.0)
    wind  = np.array([0.0, 10.0, 0.0])
    r, g  = _call_both(ref, jit, 0.05, 0.1, 0.0,
                       R_hub, np.zeros(3), 4.0, wind, 0.0)
    _assert_close(r, g, label="t=0 ramp")
    assert np.allclose(r.F_world, 0.0, atol=1e-10), "expected zero force at t=0"


def test_negative_omega(ref, jit):
    """Sign of omega_rotor must be handled identically."""
    R_hub = _R_tilted(50.0)
    v_hub = np.zeros(3)
    wind  = np.array([0.0, 8.0, 0.0])
    r_pos, g_pos = _call_both(ref, jit, 0.05, 0.0, 0.0, R_hub, v_hub,  3.5, wind, 10.0)
    r_neg, g_neg = _call_both(ref, jit, 0.05, 0.0, 0.0, R_hub, v_hub, -3.5, wind, 10.0)
    _assert_close(r_pos, g_pos, label="+omega")
    _assert_close(r_neg, g_neg, label="-omega")


def test_hub_velocity(ref, jit):
    """Hub translational velocity must subtract from apparent wind identically."""
    R_hub = _R_tilted(55.0)
    wind  = np.array([0.0, 10.0, 0.0])
    for v_hub in [np.zeros(3), np.array([0.5, 2.0, -0.3]), np.array([-1.0, 0.0, 1.0])]:
        r, g = _call_both(ref, jit, 0.06, 0.08, 0.04,
                          R_hub, v_hub, 3.5, wind, 10.0)
        _assert_close(r, g, label=f"v_hub={v_hub}")


def test_tilt_sweep(ref, jit):
    """Sweep rotor tilt from near-axial to near-edgewise."""
    v_hub = np.zeros(3)
    wind  = np.array([0.0, 10.0, 0.0])
    for xi_deg in [10, 25, 40, 55, 70, 80]:
        R_hub = _R_tilted(xi_deg)
        r, g  = _call_both(ref, jit, 0.05, 0.0, 0.0, R_hub, v_hub, 3.5, wind, 10.0)
        _assert_close(r, g, label=f"xi={xi_deg}")


# ---------------------------------------------------------------------------
# Diagnostic attributes
# ---------------------------------------------------------------------------

def test_diagnostics_match(ref, jit):
    """All last_* diagnostic attributes must agree after a call."""
    R_hub = _R_tilted(55.0)
    v_hub = np.array([0.0, 2.0, 0.5])
    wind  = np.array([0.0, 8.0, 0.0])
    ref.compute_forces(0.05, 0.1, 0.05, R_hub, v_hub, 3.5, wind, 10.0, 0.3)
    jit.compute_forces(0.05, 0.1, 0.05, R_hub, v_hub, 3.5, wind, 10.0, 0.3)

    scalar_attrs = [
        "last_T", "last_v_axial", "last_v_i", "last_v_inplane",
        "last_ramp", "last_Q_spin", "last_H_force",
        "last_skew_angle_deg", "last_K_skew", "last_F_prandtl",
    ]
    for attr in scalar_attrs:
        rv, gv = getattr(ref, attr), getattr(jit, attr)
        assert abs(rv - gv) < 1e-10, f"{attr}: ref={rv} jit={gv}"

    for attr in ("last_M_spin", "last_M_cyc"):
        rv, gv = getattr(ref, attr), getattr(jit, attr)
        assert np.allclose(rv, gv, atol=1e-10), f"{attr}: ref={rv} jit={gv}"


# ---------------------------------------------------------------------------
# create_aero default
# ---------------------------------------------------------------------------

def test_create_aero_default_is_jit(rotor):
    """create_aero() with no model arg must return SkewedWakeBEMJit."""
    from aero import create_aero
    aero = create_aero(rotor)
    assert isinstance(aero, SkewedWakeBEMJit), \
        f"Expected SkewedWakeBEMJit, got {type(aero).__name__}"


def test_create_aero_numpy_fallback(rotor):
    """create_aero(model='skewed_wake_numpy') must return the pure-numpy version."""
    from aero import create_aero
    aero = create_aero(rotor, model="skewed_wake_numpy")
    assert type(aero) is SkewedWakeBEM, \
        f"Expected SkewedWakeBEM, got {type(aero).__name__}"
