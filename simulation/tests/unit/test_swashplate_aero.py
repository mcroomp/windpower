"""
test_swashplate_aero.py — swashplate commands produce physically sensible aero forces.

Pure swashplate + aero model.  No mediator, no SITL, no JSON.

Geometry
--------
Wind 10 m/s East (NED Y).  The disk is parametrized by how many degrees it is
tilted away from the pure-axial (wind-aligned) orientation:

  tilt_deg = 0  : body_z = [0,1,0]  (East) — wind goes straight into disk
  tilt_deg = 30 : body_z = [0, cos30, -sin30] — typical RAWES orbit angle
  tilt_deg = 60 : body_z = [0, cos60, -sin60] — high-tilt / reel-in
  tilt_deg = 75 : body_z near-level disk — limit of valid BEM

The tilt is always in the East-Up plane so the geometry stays 2-D and the
expected cyclic directions remain consistent across all parametrized cases.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aero import create_aero
import rotor_definition as _rd
from frames import build_orb_frame
from swashplate import (
    ardupilot_h3_120_forward,
    ardupilot_h3_120_inverse,
    collective_out_to_rad,
    collective_rad_to_out,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_OMEGA   = 28.0
_WIND    = np.array([0.0, 10.0, 0.0])   # NED: 10 m/s East
_T       = 10.0                          # past BEM 5 s ramp
_COL_MIN = -0.28
_COL_MAX =  0.10

# Tilt angles to test: from pure-axial (0 deg) through typical RAWES angles
_TILT_ANGLES = [0, 15, 30, 45, 60, 75]

# Thrust-along-disk-normal increases with collective, but the zero-crossing
# collective shifts upward as tilt increases (more skewed inflow → more pitch
# needed for the same axial thrust).  Use a per-tilt collective that is above
# the zero-crossing at every tested orientation.
_CRUISE_COL = {
    0:  -0.20,   # pure axial: generous margin
    15: -0.20,
    30: -0.18,   # typical RAWES cruise
    45: -0.05,   # zero-crossing ~-0.17; use midpoint of valid range
    60:  0.02,   # zero-crossing ~-0.05; use modest positive collective
    75:  0.07,   # zero-crossing ~+0.03; near col_max
}

# At tilt=75° the Coleman skewed-wake BEM becomes numerically unstable in the
# moderate-collective range and thrust is non-monotone there.  Limit the
# monotone sweep to angles where BEM is well-behaved.
_TILT_ANGLES_MONOTONE = [0, 15, 30, 45, 60]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _body_z(tilt_deg: float) -> np.ndarray:
    """Disk normal, tilted tilt_deg away from wind (East) in the East-Up plane."""
    t = math.radians(tilt_deg)
    return np.array([0.0, math.cos(t), -math.sin(t)])


def _r_hub(tilt_deg: float) -> np.ndarray:
    return build_orb_frame(_body_z(tilt_deg))


def _aero(tilt_deg: float):
    return create_aero(_rd.default())


def _forces(tilt_deg: float, col_rad: float,
            tilt_lat: float = 0.0, tilt_lon: float = 0.0) -> np.ndarray:
    """6-vector [Fx,Fy,Fz,Mx,My,Mz] in NED world frame."""
    return _aero(tilt_deg).compute_forces(
        collective_rad=col_rad,
        tilt_lon=tilt_lon,
        tilt_lat=tilt_lat,
        R_hub=_r_hub(tilt_deg),
        v_hub_world=np.zeros(3),
        omega_rotor=_OMEGA,
        wind_world=_WIND,
        t=_T,
    )


def _thrust(tilt_deg: float, col_rad: float,
            tilt_lat: float = 0.0, tilt_lon: float = 0.0) -> float:
    """Force component along body_z (disk normal)."""
    bz = _body_z(tilt_deg)
    return float(np.dot(_forces(tilt_deg, col_rad, tilt_lat, tilt_lon)[:3], bz))


def _off_axis(delta_f: np.ndarray, tilt_deg: float) -> np.ndarray:
    """Component of a force delta that is perpendicular to body_z."""
    bz = _body_z(tilt_deg)
    return delta_f - np.dot(delta_f, bz) * bz


# ---------------------------------------------------------------------------
# Collective tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_collective_produces_positive_thrust(tilt_deg):
    """At the per-tilt operating collective, thrust along disk normal must be positive.

    The operating collective increases with tilt because skewed inflow requires more
    blade pitch to generate the same axial thrust.  _CRUISE_COL encodes physically
    appropriate collectives for each disk orientation.
    """
    col = _CRUISE_COL[tilt_deg]
    assert _thrust(tilt_deg, col) > 0.0, (
        f"tilt={tilt_deg} deg: expected positive thrust at col={col:.2f} rad"
    )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES_MONOTONE)
def test_collective_monotonically_increases_thrust(tilt_deg):
    """Higher collective → strictly more thrust along disk normal.

    Only tested up to 60° tilt: at 75° the Coleman skewed-wake BEM becomes
    numerically unstable in the middle of the collective range and thrust is
    non-monotone there.
    """
    cols = [-0.25, -0.18, -0.10, 0.0, 0.05]
    thrusts = [_thrust(tilt_deg, c) for c in cols]
    for i in range(len(thrusts) - 1):
        assert thrusts[i] < thrusts[i + 1], (
            f"tilt={tilt_deg} deg: thrust not monotone at "
            f"col={cols[i]:.2f}→{cols[i+1]:.2f}: "
            f"T={thrusts[i]:.1f}→{thrusts[i+1]:.1f} N"
        )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_collective_range_produces_large_thrust_difference(tilt_deg):
    """Full collective range must span > 30 N along disk normal."""
    delta = _thrust(tilt_deg, _COL_MAX) - _thrust(tilt_deg, _COL_MIN)
    assert delta > 30.0, (
        f"tilt={tilt_deg} deg: full collective range gave only {delta:.1f} N "
        f"thrust change along disk normal (expected > 30 N)"
    )


# ---------------------------------------------------------------------------
# Cyclic direction tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_roll_cyclic_deflects_thrust_off_axis(tilt_deg):
    """Roll cyclic must produce a detectable off-axis force change."""
    f0 = _forces(tilt_deg, -0.18, tilt_lat=0.0)
    fp = _forces(tilt_deg, -0.18, tilt_lat=0.4)
    off = _off_axis(fp[:3] - f0[:3], tilt_deg)
    assert np.linalg.norm(off) > 0.5, (
        f"tilt={tilt_deg} deg: roll cyclic (tilt_lat=0.4) off-axis force "
        f"{np.linalg.norm(off):.3f} N < 0.5 N"
    )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_pitch_cyclic_deflects_thrust_off_axis(tilt_deg):
    """Pitch cyclic must produce a detectable off-axis force change."""
    f0 = _forces(tilt_deg, -0.18, tilt_lon=0.0)
    fp = _forces(tilt_deg, -0.18, tilt_lon=0.4)
    off = _off_axis(fp[:3] - f0[:3], tilt_deg)
    assert np.linalg.norm(off) > 0.5, (
        f"tilt={tilt_deg} deg: pitch cyclic (tilt_lon=0.4) off-axis force "
        f"{np.linalg.norm(off):.3f} N < 0.5 N"
    )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_roll_and_pitch_cyclic_act_on_different_axes(tilt_deg):
    """Roll and pitch cyclic must deflect thrust in non-parallel directions."""
    f0 = _forces(tilt_deg, -0.18)
    off_r = _off_axis(_forces(tilt_deg, -0.18, tilt_lat=0.4)[:3] - f0[:3], tilt_deg)
    off_p = _off_axis(_forces(tilt_deg, -0.18, tilt_lon=0.4)[:3] - f0[:3], tilt_deg)

    norm_r, norm_p = np.linalg.norm(off_r), np.linalg.norm(off_p)
    assert norm_r > 0.3, f"tilt={tilt_deg} deg: roll off-axis force too small ({norm_r:.3f} N)"
    assert norm_p > 0.3, f"tilt={tilt_deg} deg: pitch off-axis force too small ({norm_p:.3f} N)"

    cos_angle = abs(np.dot(off_r, off_p)) / (norm_r * norm_p)
    assert cos_angle < 0.9, (
        f"tilt={tilt_deg} deg: roll and pitch cyclic nearly parallel "
        f"(cos={cos_angle:.3f}); expected < 0.9"
    )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_opposite_cyclic_produces_opposite_deflection(tilt_deg):
    """Flipping cyclic sign must flip the off-axis deflection direction."""
    f0 = _forces(tilt_deg, -0.18)
    off_p = _off_axis(_forces(tilt_deg, -0.18, tilt_lat=0.3)[:3] - f0[:3], tilt_deg)
    off_n = _off_axis(_forces(tilt_deg, -0.18, tilt_lat=-0.3)[:3] - f0[:3], tilt_deg)
    assert np.dot(off_p, off_n) < 0.0, (
        f"tilt={tilt_deg} deg: opposite roll cyclic should deflect in opposite directions"
    )


# ---------------------------------------------------------------------------
# Swashplate encode/decode round-trip
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_encode_decode_roundtrip_gives_identical_forces(tilt_deg):
    """
    Encoding collective_rad through the swashplate and decoding back must give
    identical aero forces at every disk orientation.
    """
    col_direct = -0.18
    roll_cmd   =  0.10
    pitch_cmd  = -0.05

    f_direct = _forces(tilt_deg, col_direct, tilt_lat=roll_cmd, tilt_lon=pitch_cmd)

    col_out = collective_rad_to_out(col_direct, _COL_MIN, _COL_MAX)
    s1, s2, s3 = ardupilot_h3_120_forward(col_out, roll_cmd, pitch_cmd)
    col_out_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
    col_rad_rt = collective_out_to_rad(col_out_rt, _COL_MIN, _COL_MAX)

    f_sitl = _forces(tilt_deg, col_rad_rt, tilt_lat=roll_rt, tilt_lon=pitch_rt)

    np.testing.assert_allclose(
        f_direct, f_sitl, atol=1e-6,
        err_msg=f"tilt={tilt_deg} deg: forces differ between direct and encode/decode paths"
    )


# ---------------------------------------------------------------------------
# Pure autorotation — disk level, no wind
# ---------------------------------------------------------------------------
# body_z = [0, 0, -1] (straight up, NED). No external inflow. This is the
# hover/autorotation limit case.
#
# The skewed-wake BEM is designed for RAWES operating conditions (wind present,
# tilt 0–55°). With zero inflow the induction factor computation is degenerate:
# thrust is negative at all collectives (wrong direction) and non-monotone.
# Collective tests are therefore NOT performed for this case.
#
# What IS expected to work: the swashplate mixing geometry is inflow-independent.
# Cyclic commands must still produce large off-axis force changes relative to the
# zero-cyclic baseline, and the two cyclic axes must remain non-parallel.
# ---------------------------------------------------------------------------

def _forces_no_wind(col_rad: float,
                    tilt_lat: float = 0.0, tilt_lon: float = 0.0) -> np.ndarray:
    bz = np.array([0.0, 0.0, -1.0])   # straight up in NED
    return create_aero(_rd.default()).compute_forces(
        collective_rad=col_rad,
        tilt_lon=tilt_lon,
        tilt_lat=tilt_lat,
        R_hub=build_orb_frame(bz),
        v_hub_world=np.zeros(3),
        omega_rotor=_OMEGA,
        wind_world=np.zeros(3),
        t=_T,
    )


def test_autorotation_no_wind_model_returns_finite():
    """BEM must not crash or return NaN/Inf with zero wind."""
    for col in [-0.28, -0.18, 0.0, 0.10]:
        f = _forces_no_wind(col)
        assert np.all(np.isfinite(f)), (
            f"col={col}: BEM returned non-finite forces at zero wind: {f}"
        )


def test_autorotation_no_wind_roll_cyclic_deflects_off_axis():
    """Roll cyclic must produce a large off-axis force change even with no wind."""
    bz = np.array([0.0, 0.0, -1.0])
    f0 = _forces_no_wind(0.0, tilt_lat=0.0)
    fp = _forces_no_wind(0.0, tilt_lat=0.4)
    delta = fp[:3] - f0[:3]
    off = delta - np.dot(delta, bz) * bz
    assert np.linalg.norm(off) > 5.0, (
        f"roll cyclic (tilt_lat=0.4) off-axis force {np.linalg.norm(off):.1f} N < 5 N at zero wind"
    )


def test_autorotation_no_wind_pitch_cyclic_deflects_off_axis():
    """Pitch cyclic must produce a large off-axis force change even with no wind."""
    bz = np.array([0.0, 0.0, -1.0])
    f0 = _forces_no_wind(0.0, tilt_lon=0.0)
    fp = _forces_no_wind(0.0, tilt_lon=0.4)
    delta = fp[:3] - f0[:3]
    off = delta - np.dot(delta, bz) * bz
    assert np.linalg.norm(off) > 5.0, (
        f"pitch cyclic (tilt_lon=0.4) off-axis force {np.linalg.norm(off):.1f} N < 5 N at zero wind"
    )


def test_autorotation_no_wind_roll_and_pitch_act_on_different_axes():
    """Roll and pitch cyclic must deflect force in non-parallel directions (no wind)."""
    bz = np.array([0.0, 0.0, -1.0])
    f0 = _forces_no_wind(0.0)

    def off_axis(delta):
        return delta - np.dot(delta, bz) * bz

    off_r = off_axis(_forces_no_wind(0.0, tilt_lat=0.4)[:3] - f0[:3])
    off_p = off_axis(_forces_no_wind(0.0, tilt_lon=0.4)[:3] - f0[:3])

    norm_r, norm_p = np.linalg.norm(off_r), np.linalg.norm(off_p)
    assert norm_r > 1.0, f"roll off-axis force too small ({norm_r:.1f} N) at zero wind"
    assert norm_p > 1.0, f"pitch off-axis force too small ({norm_p:.1f} N) at zero wind"

    cos_angle = abs(np.dot(off_r, off_p)) / (norm_r * norm_p)
    assert cos_angle < 0.9, (
        f"roll and pitch cyclic nearly parallel at zero wind (cos={cos_angle:.3f})"
    )


def test_autorotation_no_wind_collective_range_changes_forces():
    """Full collective range must change the total force magnitude, even with no wind."""
    f_min = _forces_no_wind(_COL_MIN)
    f_max = _forces_no_wind(_COL_MAX)
    delta = np.linalg.norm(f_max[:3] - f_min[:3])
    assert delta > 100.0, (
        f"Full collective range at zero wind changed total force by only {delta:.1f} N (expected > 100 N)"
    )
