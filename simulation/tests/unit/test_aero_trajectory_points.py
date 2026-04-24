"""
test_aero_trajectory_points.py — SkewedWakeBEM at De Schutter trajectory operating points.

Tests the aero model at specific (xi, collective, omega) triples that represent
real operating conditions along the De Schutter pumping cycle.

ξ (xi) is the angle between body_z (disk normal) and the wind direction:
  ξ=0°   disk normal parallel to wind  (max v_axial = v_wind)
  ξ=90°  disk normal perpendicular to wind  (v_axial = 0, edgewise flow)

For a 10 m/s eastward wind:  v_axial = 10·cos(ξ),  v_inplane = 10·sin(ξ)
Equilibrium spin rate:  omega_eq = sqrt(k_drive·v_inplane / k_drag)

Key findings from model characterisation
-----------------------------------------
  ξ=35°  (reel-out):  col=-0.20 gives Fz=142 N >> W=49 N, omega_eq=21 rad/s
  ξ=55°  (reel-in):   col=-0.10 gives Fz=72  N >  W,      omega_eq=25 rad/s
  ξ=70°  (high):      col= 0.00 gives Fz=-12 N <  W — borderline infeasible
  ξ=80°  (high):      col= 0.10 gives Fz=338 N >  W — but 0.10 rad > col_max
  ξ≥80°  altitude support requires collective above the normal range [−0.28, 0.00]

Normal collective range (from planner): COL_MIN=-0.28 rad, COL_MAX=0.00 rad.
High-tilt flight (ξ>~65°) requires positive collective — outside normal range.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rotor_definition as _rd
from aero import create_aero
from frames import build_orb_frame

# ── Common conditions ────────────────────────────────────────────────────────
WIND        = np.array([0.0, 10.0, 0.0])   # NED: 10 m/s East = Y axis
T_PAST_RAMP = 50.0                          # s — well past the 5 s ramp
V_HUB       = np.zeros(3)                  # hub stationary
W_HUB_N     = 5.0 * 9.81                   # hub weight [N]


# ── Helpers ──────────────────────────────────────────────────────────────────

def _bz_from_xi(xi_deg: float) -> np.ndarray:
    """bz = cos(ξ)·wind_hat + sin(ξ)·up  (in NED: East=Y, Up=-Z)"""
    xi_r = math.radians(xi_deg)
    bz   = math.cos(xi_r) * np.array([0., 1., 0.]) + math.sin(xi_r) * np.array([0., 0., -1.])
    return bz / np.linalg.norm(bz)


def _omega_eq(aero, xi_deg: float) -> float:
    """Equilibrium spin rate at which Q_spin = 0 for the given tilt."""
    v_ip = 10.0 * math.sin(math.radians(xi_deg))
    return math.sqrt(aero.k_drive_spin * max(v_ip, 0.01) / aero.k_drag_spin)


def _call(xi_deg, collective_rad, omega=None):
    """Run SkewedWakeBEM. omega defaults to equilibrium spin for the given xi."""
    aero = create_aero(_rd.default())
    if omega is None:
        omega = _omega_eq(aero, xi_deg)
    bz = _bz_from_xi(xi_deg)
    R  = build_orb_frame(bz)
    r  = aero.compute_forces(
        collective_rad = collective_rad,
        tilt_lon       = 0.0,
        tilt_lat       = 0.0,
        R_hub          = R,
        v_hub_world    = V_HUB,
        omega_rotor    = omega,
        wind_world     = WIND,
        t              = T_PAST_RAMP,
    )
    return r, aero


# ── 1. Finite outputs at every grid point ────────────────────────────────────

_XI_GRID  = [0.0, 20.0, 35.0, 45.0, 55.0, 65.0, 70.0, 80.0, 85.0, 90.0]
_COL_GRID = [-0.28, -0.20, -0.10, 0.0, 0.10]

@pytest.mark.parametrize("xi_deg",        _XI_GRID,  ids=[f"xi{x:.0f}" for x in _XI_GRID])
@pytest.mark.parametrize("collective_rad", _COL_GRID, ids=[f"col{c:.2f}" for c in _COL_GRID])
def test_finite_at_all_grid_points(xi_deg, collective_rad):
    """compute_forces must never return NaN or inf anywhere in the operating space."""
    r, aero = _call(xi_deg, collective_rad)
    assert np.all(np.isfinite(r.F_world)),   "F_world contains NaN/inf"
    assert np.all(np.isfinite(r.M_orbital)), "M_orbital contains NaN/inf"
    assert math.isfinite(aero.last_T),             "last_T is NaN/inf"
    assert math.isfinite(aero.last_Q_spin),        "last_Q_spin is NaN/inf"
    assert math.isfinite(aero.last_skew_angle_deg),"skew_angle is NaN/inf"


# ── 2. Reel-out operating point (A): xi=35°, col=-0.20 ───────────────────────

def test_reel_out_vertical_force():
    """
    Reel-out (xi=35°, col=-0.20): vertical force must exceed hub weight.

    At equilibrium spin (omega_eq≈21 rad/s) and tether-aligned body_z, the disk
    must produce Fz > W = 49 N so the hub can fly against gravity.
    """
    r, aero = _call(xi_deg=35.0, collective_rad=-0.20)
    Fz = float(-r.F_world[2])   # NED: upward = -Z
    assert Fz > W_HUB_N, (
        f"Reel-out Fz={Fz:.1f} N < weight {W_HUB_N:.1f} N — "
        f"disk cannot support hub (v_axial={aero.last_v_axial:.2f} m/s)"
    )


def test_reel_out_thrust_along_tether():
    """Reel-out: significant thrust along tether direction (generates tension)."""
    r, _ = _call(xi_deg=35.0, collective_rad=-0.20)
    bz   = _bz_from_xi(35.0)
    T_bz = float(np.dot(r.F_world, bz))
    assert T_bz > 50.0, (
        f"Reel-out thrust along tether T.bz={T_bz:.1f} N — too low to generate tension"
    )


def test_reel_out_collective_increases_tension():
    """Higher collective at reel-out gives higher thrust along tether (PI has authority)."""
    r_lo, _ = _call(35.0, -0.25)
    r_hi, _ = _call(35.0, -0.15)
    bz = _bz_from_xi(35.0)
    assert np.dot(r_hi.F_world, bz) > np.dot(r_lo.F_world, bz), (
        "Collective authority lost at reel-out: higher collective does not increase T.bz"
    )


# ── 3. Reel-in nominal (B): xi=55°, col=-0.10 ────────────────────────────────

def test_reel_in_55_altitude_supported():
    """
    Reel-in nominal (xi=55°, col=-0.10): Fz must exceed hub weight.

    Also checks that collective authority is positive: col=-0.10 must produce
    more upward force than col=-0.20 (reducing negative collective increases lift).
    """
    r_flight, _  = _call(xi_deg=55.0, collective_rad=-0.10)
    r_too_low, _ = _call(xi_deg=55.0, collective_rad=-0.20)
    Fz_flight   = float(-r_flight.F_world[2])  # NED: upward = -Z
    Fz_too_low  = float(-r_too_low.F_world[2])
    assert Fz_flight > W_HUB_N, (
        f"xi=55° col=-0.10: Fz={Fz_flight:.1f} N < weight {W_HUB_N:.1f} N"
    )
    assert Fz_flight > Fz_too_low, (
        f"xi=55°: collective authority wrong — col=-0.10 gives {Fz_flight:.1f} N "
        f"but col=-0.20 gives {Fz_too_low:.1f} N (should be less)"
    )


def test_reel_in_55_lower_tether_tension_than_reel_out():
    """
    Tilting the disk away from the wind (xi=35° → xi=55°) reduces the thrust
    component along the tether (bz) at fixed collective and spin rate.

    This is the core De Schutter mechanism: tilting body_z away from the wind
    direction reduces outward tether tension, enabling net energy production
    over a reel-out/reel-in cycle.
    """
    OMEGA_FIXED = 20.0   # rad/s — fixed spin for a clean comparison
    COL_FIXED   = -0.20
    r_out, _ = _call(35.0, COL_FIXED, omega=OMEGA_FIXED)
    r_in, _  = _call(55.0, COL_FIXED, omega=OMEGA_FIXED)
    T_out    = float(np.dot(r_out.F_world, _bz_from_xi(35.0)))
    T_in     = float(np.dot(r_in.F_world,  _bz_from_xi(55.0)))
    assert T_in < T_out, (
        f"De Schutter mechanism not working: T.bz at xi=55° ({T_in:.1f} N) >= "
        f"T.bz at xi=35° ({T_out:.1f} N) at fixed col={COL_FIXED} omega={OMEGA_FIXED}"
    )


# ── 4. High-tilt boundary: xi≥70° needs positive collective ──────────────────

@pytest.mark.parametrize("xi_deg", [70.0, 80.0, 85.0],
                         ids=["xi70", "xi80", "xi85"])
def test_high_tilt_collective_authority(xi_deg):
    """
    At xi≥70°, collective must still have positive authority: more collective
    (less negative pitch) must produce more upward force.

    This guards against stall or sign-reversal of collective effectiveness
    at extreme tilt angles.
    """
    r_lo, _ = _call(xi_deg, collective_rad=-0.20)
    r_hi, _ = _call(xi_deg, collective_rad= 0.00)
    Fz_lo   = float(-r_lo.F_world[2])
    Fz_hi   = float(-r_hi.F_world[2])
    assert Fz_hi > Fz_lo, (
        f"xi={xi_deg}: collective authority lost — "
        f"col=0.0 gives {Fz_hi:.1f} N, col=-0.20 gives {Fz_lo:.1f} N (should be less)"
    )


@pytest.mark.parametrize("xi_deg", [70.0, 80.0, 85.0],
                         ids=["xi70", "xi80", "xi85"])
def test_high_tilt_altitude_recoverable_with_extended_collective(xi_deg):
    """
    At xi≥70°, positive collective (col=+0.10) does restore altitude support.

    This confirms the model is not broken — altitude is physically possible at
    high tilt, but only with collective extended beyond the normal range.
    """
    r, _ = _call(xi_deg, collective_rad=0.10)
    Fz   = float(-r.F_world[2])   # NED: upward = -Z
    assert Fz > W_HUB_N, (
        f"xi={xi_deg}°: Fz={Fz:.1f} N at col=+0.10 still below weight {W_HUB_N:.1f} N — "
        f"altitude not recoverable with extended collective"
    )


# ── 5. Spin torque sign ───────────────────────────────────────────────────────

@pytest.mark.parametrize("xi_deg", [35.0, 55.0, 70.0],
                         ids=["xi35", "xi55", "xi70"])
def test_q_spin_positive_below_equilibrium(xi_deg):
    """
    At omega < omega_eq, Q_spin > 0: rotor accelerates toward equilibrium.
    """
    aero_tmp = create_aero(_rd.default())
    omega_eq = _omega_eq(aero_tmp, xi_deg)
    omega_low = omega_eq * 0.75   # well below equilibrium
    _, aero = _call(xi_deg, collective_rad=-0.20, omega=omega_low)
    assert aero.last_Q_spin > 0.0, (
        f"xi={xi_deg}°: Q_spin={aero.last_Q_spin:.2f} N·m at omega={omega_low:.1f} rad/s "
        f"(below eq {omega_eq:.1f}) — expected positive (rotor should accelerate)"
    )


@pytest.mark.parametrize("xi_deg", [35.0, 55.0, 70.0],
                         ids=["xi35", "xi55", "xi70"])
def test_q_spin_negative_above_equilibrium(xi_deg):
    """
    At omega > omega_eq, Q_spin < 0: rotor decelerates toward equilibrium.
    """
    aero_tmp = create_aero(_rd.default())
    omega_eq = _omega_eq(aero_tmp, xi_deg)
    omega_high = omega_eq * 1.5   # well above equilibrium
    _, aero = _call(xi_deg, collective_rad=-0.20, omega=omega_high)
    assert aero.last_Q_spin < 0.0, (
        f"xi={xi_deg}°: Q_spin={aero.last_Q_spin:.2f} N·m at omega={omega_high:.1f} rad/s "
        f"(above eq {omega_eq:.1f}) — expected negative (rotor should decelerate)"
    )


# ── 6. Skew angle increases monotonically with xi ────────────────────────────

def test_skew_angle_increases_with_tilt():
    """
    Wake skew angle χ must increase monotonically as xi increases.
    χ = atan2(v_inplane, v_axial+v_i) → 90° at xi=90°.
    """
    xis  = [20.0, 35.0, 55.0, 70.0, 80.0, 85.0, 90.0]
    chis = []
    for xi in xis:
        _, aero = _call(xi, collective_rad=-0.10)
        chis.append(aero.last_skew_angle_deg)
    for i in range(len(chis) - 1):
        assert chis[i] < chis[i + 1], (
            f"Skew angle not monotonic: chi({xis[i]}°)={chis[i]:.1f}° >= "
            f"chi({xis[i+1]}°)={chis[i+1]:.1f}°"
        )


# ── 7. Diagnostic table ───────────────────────────────────────────────────────

def test_trajectory_characterisation_table(capsys):
    """
    Print characterisation table across the full trajectory. Not a pass/fail
    test — run with -s to see the output.

    Columns: xi  omega_eq  v_ax  v_ip  chi  Fz@col=-0.20  Fz@col=0.00  Q_sign
    """
    rows = [
        f"{'xi':>4}  {'omega_eq':>8}  {'v_ax':>5}  {'v_ip':>5}  "
        f"{'chi':>5}  {'Fz@-0.20':>9}  {'Fz@0.00':>8}  {'Fz@+0.10':>9}",
        "-" * 68,
    ]
    for xi in [0, 20, 35, 45, 55, 65, 70, 75, 80, 85, 90]:
        aero_tmp = create_aero(_rd.default())
        omega_eq = _omega_eq(aero_tmp, float(xi))
        r_lo, a_lo = _call(float(xi), -0.20)
        r_mid, _   = _call(float(xi),  0.00)
        r_hi, _    = _call(float(xi),  0.10)
        rows.append(
            f"{xi:4d}  {omega_eq:8.1f}  {a_lo.last_v_axial:5.2f}  {a_lo.last_v_inplane:5.2f}  "
            f"{a_lo.last_skew_angle_deg:5.1f}  "
            f"{-r_lo.F_world[2]:9.1f}  {-r_mid.F_world[2]:8.1f}  {-r_hi.F_world[2]:9.1f}"
        )
    print("\n" + "\n".join(rows))
    print(f"\n  Hub weight = {W_HUB_N:.1f} N  |  COL_MAX (normal) = 0.00 rad")
    print("  Rows with Fz@0.00 < weight are infeasible within normal collective range.")
