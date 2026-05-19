"""
test_swashplate_aero.py — swashplate commands produce physically sensible aero forces.

Pure swashplate + aero model (BEMModel, Level-1 quasi-static, new aero package).

Geometry
--------
Wind 10 m/s East (NED Y).  The disk is parametrised by how many degrees it
is tilted away from pure-axial (wind-aligned) orientation:

  tilt_deg = 0  : body_z = [0,1,0]  (East) — wind goes straight into disk
  tilt_deg = 30 : body_z = [0, cos30, -sin30] — typical RAWES orbit angle
  tilt_deg = 60 : body_z = [0, cos60, -sin60] — high-tilt / reel-in
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from frames import build_orb_frame
from swashplate import (
    ardupilot_h3_120_forward,
    ardupilot_h3_120_inverse,
    collective_out_to_rad,
    collective_rad_to_out,
)
from tests.unit._aero_probe import make_probe, probe_steady

_OMEGA   = 28.0
_WIND    = np.array([0.0, 10.0, 0.0])   # NED: 10 m/s East
_T       = 10.0
_COL_MIN = -0.28
_COL_MAX =  0.10

_TILT_ANGLES = [0, 15, 30, 45, 60, 75]
_TILT_ANGLES_MONOTONE = [0, 15, 30, 45, 60]

# Single shared probe model — quasi-static, no carried state between calls.
_AERO = make_probe()


def _body_z(tilt_deg: float) -> np.ndarray:
    """FRD: body_z = hub axis DOWN through disk.  For tilt_deg=0 the disk is
    wind-facing (axial wind blows in +body_z direction = autorotation up
    through disk in NED)."""
    t = math.radians(tilt_deg)
    return np.array([0.0, -math.cos(t), math.sin(t)])


def _r_hub(tilt_deg: float) -> np.ndarray:
    return build_orb_frame(_body_z(tilt_deg))


def _forces6(tilt_deg: float, col_rad: float,
             tilt_lat: float = 0.0, tilt_lon: float = 0.0) -> np.ndarray:
    """6-vector [Fx,Fy,Fz,Mx,My,Mz] (M = M_orbital + M_spin) in NED world frame."""
    r = probe_steady(
        _AERO,
        collective_rad=col_rad,
        tilt_lon=tilt_lon,
        tilt_lat=tilt_lat,
        R_hub=_r_hub(tilt_deg),
        v_hub_world=np.zeros(3),
        omega_rotor=_OMEGA,
        wind_world=_WIND,
        t=_T,
    )
    return np.concatenate([r.F_world, r.M_orbital + r.M_spin])


def _thrust(tilt_deg: float, col_rad: float,
            tilt_lat: float = 0.0, tilt_lon: float = 0.0) -> float:
    """Thrust opposing body_z (FRD: body_z points DOWN through disk, so lift
    is along -body_z).  Positive = lifting away from anchor."""
    bz = _body_z(tilt_deg)
    return float(-np.dot(_forces6(tilt_deg, col_rad, tilt_lat, tilt_lon)[:3], bz))


# ── Collective tests ─────────────────────────────────────────────────────────


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES_MONOTONE)
def test_collective_monotonically_increases_thrust(tilt_deg):
    """Higher collective → strictly more thrust along disk normal."""
    cols = [-0.25, -0.18, -0.10, 0.0, 0.05]
    thrusts = [_thrust(tilt_deg, c) for c in cols]
    for i in range(len(thrusts) - 1):
        assert thrusts[i] < thrusts[i + 1], (
            f"tilt={tilt_deg} deg: thrust not monotone at "
            f"col={cols[i]:.2f}->{cols[i+1]:.2f}: "
            f"T={thrusts[i]:.2f}->{thrusts[i+1]:.2f} N"
        )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_collective_range_produces_thrust_difference(tilt_deg):
    """Full collective range must produce a positive thrust change."""
    delta = _thrust(tilt_deg, _COL_MAX) - _thrust(tilt_deg, _COL_MIN)
    assert delta > 5.0, (
        f"tilt={tilt_deg} deg: full collective range produced only "
        f"{delta:.2f} N along disk normal (expected > 5 N)"
    )


# ── Cyclic direction tests ──────────────────────────────────────────────────
# In the new aero (Øye / Pitt-Peters), cyclic input produces a HUB MOMENT
# (M_orbital), not a direct off-axis force deflection.  Rigid-body dynamics
# then integrate that moment to tilt the disk, which redirects thrust.
# So the right thing to test at the aero boundary is the moment response.


def _moment(tilt_deg, col, tilt_lat=0.0, tilt_lon=0.0):
    return _forces6(tilt_deg, col, tilt_lat, tilt_lon)[3:]


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_roll_cyclic_produces_hub_moment(tilt_deg):
    """Lateral swashplate tilt must produce a non-trivial hub moment."""
    M0 = _moment(tilt_deg, -0.18, tilt_lat=0.0)
    Mp = _moment(tilt_deg, -0.18, tilt_lat=0.4)
    delta = np.linalg.norm(Mp - M0)
    assert delta > 1.0, (
        f"tilt={tilt_deg} deg: roll cyclic produced only {delta:.3f} N*m hub "
        f"moment change (expected > 1 N*m)"
    )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_pitch_cyclic_produces_hub_moment(tilt_deg):
    """Longitudinal swashplate tilt must produce a non-trivial hub moment."""
    M0 = _moment(tilt_deg, -0.18, tilt_lon=0.0)
    Mp = _moment(tilt_deg, -0.18, tilt_lon=0.4)
    delta = np.linalg.norm(Mp - M0)
    assert delta > 1.0, (
        f"tilt={tilt_deg} deg: pitch cyclic produced only {delta:.3f} N*m hub "
        f"moment change (expected > 1 N*m)"
    )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_roll_and_pitch_cyclic_produce_non_parallel_moments(tilt_deg):
    """Roll and pitch cyclic must yield non-parallel hub moments."""
    M0 = _moment(tilt_deg, -0.18)
    dMr = _moment(tilt_deg, -0.18, tilt_lat=0.4) - M0
    dMp = _moment(tilt_deg, -0.18, tilt_lon=0.4) - M0
    nr, np_ = np.linalg.norm(dMr), np.linalg.norm(dMp)
    if nr < 1e-3 or np_ < 1e-3:
        pytest.skip(f"tilt={tilt_deg}: cyclic moment below sensitivity")
    cos_angle = abs(np.dot(dMr, dMp)) / (nr * np_)
    assert cos_angle < 0.98, (
        f"tilt={tilt_deg} deg: roll and pitch cyclic moments nearly parallel "
        f"(cos={cos_angle:.3f})"
    )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_opposite_cyclic_produces_opposite_moment(tilt_deg):
    """Flipping the cyclic sign must flip the hub-moment direction."""
    M0 = _moment(tilt_deg, -0.18)
    dM_p = _moment(tilt_deg, -0.18, tilt_lat=0.3) - M0
    dM_n = _moment(tilt_deg, -0.18, tilt_lat=-0.3) - M0
    if np.linalg.norm(dM_p) < 1e-3 or np.linalg.norm(dM_n) < 1e-3:
        pytest.skip(f"tilt={tilt_deg}: cyclic moment below sensitivity")
    assert np.dot(dM_p, dM_n) < 0.0, (
        f"tilt={tilt_deg} deg: opposite roll cyclic should yield opposite "
        f"hub-moment direction"
    )


# ── Swashplate encode/decode round-trip ─────────────────────────────────────


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_encode_decode_roundtrip_gives_identical_forces(tilt_deg):
    """Encoding through the H3-120 swashplate and decoding back must give
    identical aero forces at every disk orientation."""
    col_direct = -0.18
    roll_cmd   =  0.10
    pitch_cmd  = -0.05

    f_direct = _forces6(tilt_deg, col_direct, tilt_lat=roll_cmd, tilt_lon=pitch_cmd)

    col_out = collective_rad_to_out(col_direct, _COL_MIN, _COL_MAX)
    s1, s2, s3 = ardupilot_h3_120_forward(col_out, roll_cmd, pitch_cmd)
    col_out_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
    col_rad_rt = collective_out_to_rad(col_out_rt, _COL_MIN, _COL_MAX)

    f_sitl = _forces6(tilt_deg, col_rad_rt, tilt_lat=roll_rt, tilt_lon=pitch_rt)

    np.testing.assert_allclose(
        f_direct, f_sitl, atol=1e-6,
        err_msg=f"tilt={tilt_deg} deg: forces differ between direct and encode/decode paths"
    )


# ── Pure autorotation — disk level, no wind ─────────────────────────────────


def _forces6_no_wind(col_rad: float,
                     tilt_lat: float = 0.0, tilt_lon: float = 0.0) -> np.ndarray:
    bz = np.array([0.0, 0.0, 1.0])   # FRD: down through disk for level hover
    r = probe_steady(
        _AERO,
        collective_rad=col_rad,
        tilt_lon=tilt_lon,
        tilt_lat=tilt_lat,
        R_hub=build_orb_frame(bz),
        v_hub_world=np.zeros(3),
        omega_rotor=_OMEGA,
        wind_world=np.zeros(3),
        t=_T,
    )
    return np.concatenate([r.F_world, r.M_orbital + r.M_spin])


def test_autorotation_no_wind_model_returns_finite():
    for col in [-0.28, -0.18, 0.0, 0.10]:
        f = _forces6_no_wind(col)
        assert np.all(np.isfinite(f)), (
            f"col={col}: BEM returned non-finite forces at zero wind: {f}"
        )


def test_autorotation_no_wind_collective_changes_thrust():
    """Different collective settings must change the axial force at zero wind."""
    f_min = _forces6_no_wind(_COL_MIN)
    f_max = _forces6_no_wind(_COL_MAX)
    delta = abs(f_max[2] - f_min[2])
    assert delta > 1.0, (
        f"Collective range at zero wind only changed Fz by {delta:.3f} N "
        f"(expected > 1 N)"
    )
