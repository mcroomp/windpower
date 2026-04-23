"""
test_swashplate_aero_glauert.py — same swashplate+aero tests run against PetersHeBEM.

Mirrors test_swashplate_aero.py exactly, but uses PetersHeBEM (Glauert uniform inflow)
instead of SkewedWakeBEM (Coleman skewed-wake).

PetersHeBEM handles the full flight envelope including hover (tilt=90°, no wind),
where SkewedWakeBEM degenerates.  All collective tests are run at all tilt angles
including 90° with no wind.

Also runs a side-by-side comparison at RAWES operating tilts (0–60°) to show
where the two models agree.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aero import PetersHeBEMJit, create_aero
import rotor_definition as _rd
from frames import build_orb_frame
from swashplate import (
    ardupilot_h3_120_forward,
    ardupilot_h3_120_inverse,
    collective_out_to_rad,
    collective_rad_to_out,
)

# ---------------------------------------------------------------------------
# Constants — identical to test_swashplate_aero.py
# ---------------------------------------------------------------------------
_OMEGA   = 28.0
_WIND    = np.array([0.0, 10.0, 0.0])   # NED: 10 m/s East
_WIND.flags.writeable = False
_T       = 10.0
_COL_MIN = -0.28
_COL_MAX =  0.10

_TILT_ANGLES          = [0, 15, 30, 45, 60, 75, 90]   # includes hover (90°)
_TILT_ANGLES_MONOTONE = [0, 15, 30, 45, 60, 75, 90]   # Glauert is monotone in CT>0 domain

# Collective sweep used for the monotone test.
# At tilt=90° (hover, no wind), negative collective produces negative thrust — an invalid
# operating condition (CT=0 means free-fall; RAWES hover uses positive collective).
# Restrict the sweep to the physically meaningful positive-CT range for hover.
_MONOTONE_COLS = {
    t: [-0.25, -0.18, -0.10, 0.0, 0.05]  for t in [0, 15, 30, 45, 60, 75]
}
_MONOTONE_COLS[90] = [0.0, 0.02, 0.05, 0.08, 0.10]  # hover: stay in CT>0 region

# Per-tilt operating collective (same reasoning as test_swashplate_aero.py, extended to 90°)
_CRUISE_COL = {
    0:   -0.20,
    15:  -0.20,
    30:  -0.18,
    45:  -0.05,
    60:   0.02,
    75:   0.07,
    90:   0.05,   # hover: zero-thrust collective ≈ -0.09 rad from simple BEM; use positive for +T
}


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def aero():
    """Module-scoped PetersHeBEMJit instance — created once, warm-started across tests."""
    return PetersHeBEMJit(_rd.default())


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _body_z(tilt_deg: float) -> np.ndarray:
    t = math.radians(tilt_deg)
    return np.array([0.0, math.cos(t), -math.sin(t)])


def _wind_for(tilt_deg: float) -> np.ndarray:
    """No wind at 90° (pure hover); 10 m/s East for all other tilts."""
    return np.zeros(3) if tilt_deg == 90 else _WIND


def _forces(aero, tilt_deg: float, col_rad: float,
            tilt_lat: float = 0.0, tilt_lon: float = 0.0) -> np.ndarray:
    return aero.compute_forces(
        collective_rad=col_rad,
        tilt_lon=tilt_lon,
        tilt_lat=tilt_lat,
        R_hub=build_orb_frame(_body_z(tilt_deg)),
        v_hub_world=np.zeros(3),
        omega_rotor=_OMEGA,
        wind_world=_wind_for(tilt_deg),
        t=_T,
    )


def _thrust(aero, tilt_deg: float, col_rad: float,
            tilt_lat: float = 0.0, tilt_lon: float = 0.0) -> float:
    bz = _body_z(tilt_deg)
    return float(np.dot(_forces(aero, tilt_deg, col_rad, tilt_lat, tilt_lon)[:3], bz))


def _off_axis(delta_f: np.ndarray, tilt_deg: float) -> np.ndarray:
    bz = _body_z(tilt_deg)
    return delta_f - np.dot(delta_f, bz) * bz


def _forces_hover_wind(aero, col: float, tilt_lon: float = 0.0, tilt_lat: float = 0.0) -> np.ndarray:
    bz = np.array([0.0, 0.0, -1.0])
    return aero.compute_forces(
        collective_rad=col, tilt_lon=tilt_lon, tilt_lat=tilt_lat,
        R_hub=build_orb_frame(bz),
        v_hub_world=np.zeros(3),
        omega_rotor=_OMEGA,
        wind_world=_WIND,
        t=_T,
    )


def _q_empirical(aero, tilt_deg: float, col: float, omega: float,
                 wind: np.ndarray) -> float:
    bz = _body_z(tilt_deg)
    r  = build_orb_frame(bz)
    f  = aero.compute_forces(col, 0.0, 0.0, r, np.zeros(3), omega, wind, _T)
    return float(f.Q_spin)


def _q_bem(aero, tilt_deg: float, col: float, omega: float,
           wind: np.ndarray) -> float:
    """BEM aerodynamic spin torque (dot of M_spin with disk_normal)."""
    bz = _body_z(tilt_deg)
    r  = build_orb_frame(bz)
    f  = aero.compute_forces(col, 0.0, 0.0, r, np.zeros(3), omega, wind, _T)
    return float(np.dot(f.M_spin, bz))


# ---------------------------------------------------------------------------
# Collective tests (all tilt angles including 90°)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_glauert_collective_produces_positive_thrust(aero, tilt_deg):
    """PetersHeBEM must produce positive thrust at the per-tilt operating collective."""
    col = _CRUISE_COL[tilt_deg]
    T = _thrust(aero, tilt_deg, col)
    assert T > 0.0, (
        f"tilt={tilt_deg} deg: expected positive thrust at col={col:.2f} rad, got {T:.1f} N"
    )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES_MONOTONE)
def test_glauert_collective_monotonically_increases_thrust(aero, tilt_deg):
    """Higher collective → strictly more thrust along disk normal (CT>0 domain).

    At tilt=90° (hover, no wind) only positive collective is tested — negative
    collective produces CT<0 (downward thrust) which is not a valid RAWES operating
    condition.  CT≈0 is just the neutral/idle point, not a regime we operate in.
    """
    cols = _MONOTONE_COLS[tilt_deg]
    thrusts = [_thrust(aero, tilt_deg, c) for c in cols]
    for i in range(len(thrusts) - 1):
        assert thrusts[i] < thrusts[i + 1], (
            f"tilt={tilt_deg} deg: thrust not monotone at "
            f"col={cols[i]:.2f}→{cols[i+1]:.2f}: "
            f"T={thrusts[i]:.1f}→{thrusts[i+1]:.1f} N"
        )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_glauert_collective_range_produces_large_thrust_difference(aero, tilt_deg):
    """Full collective range must span > 30 N along disk normal."""
    delta = _thrust(aero, tilt_deg, _COL_MAX) - _thrust(aero, tilt_deg, _COL_MIN)
    assert delta > 30.0, (
        f"tilt={tilt_deg} deg: full collective range gave only {delta:.1f} N "
        f"(expected > 30 N)"
    )


# ---------------------------------------------------------------------------
# Cyclic tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_glauert_roll_cyclic_deflects_thrust_off_axis(aero, tilt_deg):
    col = _CRUISE_COL[tilt_deg]
    f0  = _forces(aero, tilt_deg, col, tilt_lat=0.0)
    fp  = _forces(aero, tilt_deg, col, tilt_lat=0.4)
    off = _off_axis(fp[:3] - f0[:3], tilt_deg)
    assert np.linalg.norm(off) > 0.5, (
        f"tilt={tilt_deg} deg: roll cyclic off-axis force {np.linalg.norm(off):.3f} N < 0.5 N"
    )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_glauert_pitch_cyclic_deflects_thrust_off_axis(aero, tilt_deg):
    col = _CRUISE_COL[tilt_deg]
    f0  = _forces(aero, tilt_deg, col, tilt_lon=0.0)
    fp  = _forces(aero, tilt_deg, col, tilt_lon=0.4)
    off = _off_axis(fp[:3] - f0[:3], tilt_deg)
    assert np.linalg.norm(off) > 0.5, (
        f"tilt={tilt_deg} deg: pitch cyclic off-axis force {np.linalg.norm(off):.3f} N < 0.5 N"
    )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_glauert_roll_and_pitch_cyclic_act_on_different_axes(aero, tilt_deg):
    col = _CRUISE_COL[tilt_deg]
    f0  = _forces(aero, tilt_deg, col)
    off_r = _off_axis(_forces(aero, tilt_deg, col, tilt_lat=0.4)[:3] - f0[:3], tilt_deg)
    off_p = _off_axis(_forces(aero, tilt_deg, col, tilt_lon=0.4)[:3] - f0[:3], tilt_deg)

    norm_r, norm_p = np.linalg.norm(off_r), np.linalg.norm(off_p)
    assert norm_r > 0.3, f"tilt={tilt_deg} deg: roll off-axis too small ({norm_r:.3f} N)"
    assert norm_p > 0.3, f"tilt={tilt_deg} deg: pitch off-axis too small ({norm_p:.3f} N)"

    cos_angle = abs(np.dot(off_r, off_p)) / (norm_r * norm_p)
    assert cos_angle < 0.9, (
        f"tilt={tilt_deg} deg: roll/pitch nearly parallel (cos={cos_angle:.3f})"
    )


@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_glauert_opposite_cyclic_produces_opposite_deflection(aero, tilt_deg):
    col  = _CRUISE_COL[tilt_deg]
    f0   = _forces(aero, tilt_deg, col)
    off_p = _off_axis(_forces(aero, tilt_deg, col, tilt_lat=0.3)[:3] - f0[:3], tilt_deg)
    off_n = _off_axis(_forces(aero, tilt_deg, col, tilt_lat=-0.3)[:3] - f0[:3], tilt_deg)
    assert np.dot(off_p, off_n) < 0.0, (
        f"tilt={tilt_deg} deg: opposite roll cyclic should flip deflection direction"
    )


# ---------------------------------------------------------------------------
# Encode/decode round-trip
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("tilt_deg", _TILT_ANGLES)
def test_glauert_encode_decode_roundtrip(aero, tilt_deg):
    col_direct = _CRUISE_COL[tilt_deg]
    roll_cmd   =  0.10
    pitch_cmd  = -0.05

    f_direct = _forces(aero, tilt_deg, col_direct, tilt_lat=roll_cmd, tilt_lon=pitch_cmd)

    col_out = collective_rad_to_out(col_direct, _COL_MIN, _COL_MAX)
    s1, s2, s3 = ardupilot_h3_120_forward(col_out, roll_cmd, pitch_cmd)
    col_out_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
    col_rad_rt = collective_out_to_rad(col_out_rt, _COL_MIN, _COL_MAX)

    f_sitl = _forces(aero, tilt_deg, col_rad_rt, tilt_lat=roll_rt, tilt_lon=pitch_rt)

    np.testing.assert_allclose(
        f_direct, f_sitl, atol=1e-6,
        err_msg=f"tilt={tilt_deg} deg: encode/decode changed forces"
    )


# ---------------------------------------------------------------------------
# Model comparison — where Glauert and SkewedWake agree
# ---------------------------------------------------------------------------

@pytest.mark.xfail(
    reason=(
        "PetersHeBEM assumes uniform (minimum-power) induction across the disk. "
        "SkewedWakeBEM uses Coleman non-uniform induction, which is less efficient "
        "and therefore requires a higher v_i for the same CT — producing 2–5x less "
        "thrust at the same collective in the autorotation regime. "
        "The two models are complementary, not interchangeable: SkewedWakeBEM is "
        "correct for tilt 0–55°; PetersHeBEM is correct for hover (tilt>=80°). "
        "This test documents the known divergence; it is expected to fail."
    ),
    strict=True,
)
@pytest.mark.parametrize("tilt_deg", [0, 15, 30, 45, 60])
def test_glauert_vs_skewedwake_thrust_within_30pct(aero, tilt_deg):
    """
    At RAWES operating tilts (0–60°, wind present), PetersHeBEM and SkewedWakeBEM
    must agree on axial thrust to within 30%.

    KNOWN FAILURE: PetersHeBEM over-estimates thrust 2–5x in the autorotation
    regime because it assumes uniform (most efficient) disk induction, while
    SkewedWakeBEM uses Coleman non-uniform induction.  Marked xfail to document
    the known limitation rather than silently skip.
    """
    col = _CRUISE_COL[tilt_deg]

    T_glauert = _thrust(aero, tilt_deg, col)

    skewed = create_aero(_rd.default())
    bz = _body_z(tilt_deg)
    f_sk = skewed.compute_forces(
        collective_rad=col, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=build_orb_frame(bz),
        v_hub_world=np.zeros(3),
        omega_rotor=_OMEGA,
        wind_world=_WIND,
        t=_T,
    )
    T_skewed = float(np.dot(f_sk[:3], bz))

    assert T_glauert > 0 and T_skewed > 0, (
        f"tilt={tilt_deg}: both models must produce positive thrust; "
        f"Glauert={T_glauert:.1f} N  Skewed={T_skewed:.1f} N"
    )
    ratio = T_glauert / T_skewed
    assert 0.7 < ratio < 1.4, (
        f"tilt={tilt_deg}: Glauert/Skewed thrust ratio {ratio:.2f} outside [0.70, 1.40]; "
        f"Glauert={T_glauert:.1f} N  Skewed={T_skewed:.1f} N"
    )


# ---------------------------------------------------------------------------
# Hover in wind — H-force and cyclic trim (tilt=90°, horizontal wind)
# ---------------------------------------------------------------------------
# At tilt=90° the disk faces up and wind is entirely in-plane (advance ratio μ>0),
# equivalent to a helicopter in forward flight.  The wind creates an eastward H-force
# that would accelerate the hub downwind.  Negative tilt_lon tilts the thrust vector
# westward and can exactly cancel the H-force — the standard hover-in-wind trim.
#
# body_x = [0,1,0] (East) for disk-up orientation, so tilt_lon acts on the east axis.
# ---------------------------------------------------------------------------

def test_hover_wind_creates_eastward_h_force(aero):
    """Horizontal wind on a level disk must produce a net eastward force (H-force)."""
    f_wind   = _forces_hover_wind(aero, 0.05)
    f_nowind = aero.compute_forces(
        0.05, 0.0, 0.0,
        build_orb_frame(np.array([0.0, 0.0, -1.0])),
        np.zeros(3), _OMEGA, np.zeros(3), _T,
    )
    h_force = float(f_wind[1]) - float(f_nowind[1])   # NED East component delta
    assert h_force > 5.0, (
        f"10 m/s East wind on level disk should create >5 N eastward H-force; got {h_force:.2f} N"
    )


def test_hover_wind_h_force_both_models_agree(aero):
    """PetersHeBEM and SkewedWakeBEM must agree on H-force magnitude to within 50%."""
    bz   = np.array([0.0, 0.0, -1.0])
    R    = build_orb_frame(bz)
    col  = 0.05

    fg = aero.compute_forces(col, 0.0, 0.0, R, np.zeros(3), _OMEGA, _WIND, _T)
    fs = create_aero(_rd.default()).compute_forces(
        col, 0.0, 0.0, R, np.zeros(3), _OMEGA, _WIND, _T)

    h_glauert = float(fg[1])
    h_skewed  = float(fs[1])

    assert h_glauert > 0, f"PetersHeBEM H-force should be eastward, got {h_glauert:.1f} N"
    assert h_skewed  > 0, f"SkewedWakeBEM H-force should be eastward, got {h_skewed:.1f} N"
    ratio = h_glauert / h_skewed
    assert 0.5 < ratio < 2.0, (
        f"H-force ratio Glauert/Skewed={ratio:.2f} — models disagree by more than 2x; "
        f"Glauert={h_glauert:.1f} N  Skewed={h_skewed:.1f} N"
    )


def test_negative_tilt_lon_reduces_eastward_force_in_hover_wind(aero):
    """Negative tilt_lon (tilting thrust westward) must reduce the eastward net force."""
    f_neutral  = _forces_hover_wind(aero, 0.05, tilt_lon=0.0)
    f_tilted   = _forces_hover_wind(aero, 0.05, tilt_lon=-0.3)
    assert float(f_tilted[1]) < float(f_neutral[1]), (
        "tilt_lon=-0.3 should reduce eastward force vs neutral; "
        f"neutral={float(f_neutral[1]):.2f} N  tilted={float(f_tilted[1]):.2f} N"
    )


def test_cyclic_can_trim_hover_against_wind(aero):
    """
    There must exist a tilt_lon in [-0.5, 0] that reduces the net eastward force
    to near zero — the standard hover-in-wind cyclic trim.

    At tilt=90° with 10 m/s East wind and col=+0.05, the wind creates ~29 N
    eastward H-force.  tilt_lon ≈ -0.197 zeros it (body_x = East so longitudinal
    cyclic tilts thrust east/west).
    """
    from scipy.optimize import brentq

    def east_force(tilt_lon):
        return float(_forces_hover_wind(aero, 0.05, tilt_lon=tilt_lon)[1])

    # Confirm the sign change exists across the search bracket
    f_lo = east_force(-0.5)
    f_hi = east_force(0.0)
    assert f_lo < 0.0 and f_hi > 0.0, (
        f"Expected sign change in east_force across [-0.5, 0]; "
        f"f(-0.5)={f_lo:.2f}  f(0)={f_hi:.2f}"
    )

    trim_lon = brentq(east_force, -0.5, 0.0, xtol=1e-4)
    net_east = east_force(trim_lon)
    assert abs(net_east) < 1.0, (
        f"Trim cyclic tilt_lon={trim_lon:.4f} left residual east force {net_east:.3f} N"
    )
    assert -0.5 < trim_lon < 0.0, (
        f"Trim tilt_lon={trim_lon:.4f} outside expected range [-0.5, 0]"
    )


# ---------------------------------------------------------------------------
# Spin torque — autorotation speeds rotor up, hover slows it down
# ---------------------------------------------------------------------------
# Two distinct spin torques are available in each result:
#
#   result.Q_spin  — empirical ODE torque: k_drive*v_inplane - k_drag*omega²
#                    This drives the rotor speed ODE in the mediator.
#                    Sign: positive = speeds rotor up; negative = slows it.
#
#   dot(result.M_spin, disk_normal)  — aerodynamic spin torque from BEM strip forces.
#                    This is the raw moment about the spin axis computed from
#                    blade element lift/drag.  Not used in the ODE but diagnostic.
#
# Physical expectations:
#   Autorotation (wind through disk, tilt=0–30°): in-plane wind drives spin → Q_spin > 0
#                                                  at omega below equilibrium
#   Hover (no wind, positive collective):          rotor does work against air → Q_spin < 0
#   More in-plane wind → larger Q_spin (more spin energy from wind)
# ---------------------------------------------------------------------------

def test_hover_empirical_spin_torque_is_negative(aero):
    """With no wind (pure hover) the empirical Q_spin must be negative — rotor decelerates.

    Empirical formula: Q_spin = k_drive*v_inplane - k_drag*omega².
    With v_inplane=0 this reduces to -k_drag*omega² < 0 always.
    """
    Q = _q_empirical(aero, 90, 0.05, _OMEGA, np.zeros(3))
    assert Q < 0.0, f"Hover Q_spin should be negative (rotor decelerates); got {Q:.2f} N*m"


def test_autorotation_empirical_spin_torque_increases_with_wind(aero):
    """More in-plane wind → larger empirical Q_spin (more spin energy delivered to rotor)."""
    # tilt=30°: v_inplane = wind * sin(30°).  Double wind doubles v_inplane.
    Q_10 = _q_empirical(aero, 30, -0.18, _OMEGA, _WIND)           # 10 m/s
    Q_20 = _q_empirical(aero, 30, -0.18, _OMEGA, _WIND * 2.0)     # 20 m/s
    assert Q_20 > Q_10, (
        f"Q_spin should increase with wind speed; Q@10m/s={Q_10:.2f}  Q@20m/s={Q_20:.2f} N*m"
    )


def test_autorotation_empirical_spin_torque_positive_below_equilibrium(aero):
    """Below equilibrium RPM, Q_spin must be positive (wind accelerates the rotor).

    Equilibrium omega ≈ 20 rad/s for v_inplane ≈ 5 m/s (tilt=30°, wind=10 m/s).
    At omega=10 rad/s (below equilibrium) Q_spin must be positive.
    """
    Q = _q_empirical(aero, 30, -0.18, 10.0, _WIND)   # omega=10, well below equilibrium
    assert Q > 0.0, (
        f"Below-equilibrium autorotation Q_spin should be positive; got {Q:.2f} N*m"
    )


def test_hover_bem_spin_torque_is_negative(aero):
    """BEM aerodynamic spin torque must be negative in hover — blades do work against air.

    In hover with positive collective the blade generates upward thrust by
    accelerating air downward.  The reaction torque on the blade opposes
    rotation → negative spin-axis moment from strip forces.
    """
    Q_bem = _q_bem(aero, 90, 0.05, _OMEGA, np.zeros(3))
    assert Q_bem < 0.0, (
        f"Hover BEM spin torque should be negative; got {Q_bem:.2f} N*m"
    )


@pytest.mark.parametrize("tilt_deg", [0, 15, 30])
def test_autorotation_bem_spin_torque_is_positive(aero, tilt_deg):
    """BEM aerodynamic spin torque must be positive in pure axial autorotation.

    With wind flowing directly through the disk (tilt=0–30°) the axial inflow
    angle drives positive blade lift in the rotational direction — the wind
    is doing work on the rotor.

    Only tested at low tilts (0–30°) where PetersHeBEM's uniform inflow is a
    reasonable approximation for the spin torque direction.
    """
    col = _CRUISE_COL[tilt_deg]
    Q_bem = _q_bem(aero, tilt_deg, col, _OMEGA, _WIND)
    assert Q_bem > 0.0, (
        f"tilt={tilt_deg} deg: autorotation BEM spin torque should be positive; "
        f"got {Q_bem:.2f} N*m"
    )
