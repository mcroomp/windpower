"""
test_vertical_landing_phases.py

Force-balance feasibility tests for 3-phase vertical landing
(hub directly above anchor, disk horizontal, 10 m/s crosswind East).

Uses Peters-He aero (no xi validity limit; SkewedWakeBEM degenerate at xi > 85 deg).

Key insight about autorotation:
    omega_eq = 28 rad/s at 10 m/s in-plane crosswind (from BEM equilibrium).
    The rotor self-sustains at IC spin rate with 10 m/s in-plane crosswind at ANY disk
    orientation — including horizontal disk (xi=90 deg) directly above the anchor.

Phases tested (all instantaneous force balance, no dynamics loop):

  Phase 1 -- Vertical hover:
    Hub stationary above anchor.  Can the rotor support its own weight with crosswind?

  Phase 2 -- Powered descent (winch-driven):
    Winch reels in tether, pulling hub down (adds to load).
    Descent speed adds upward axial flow through the disk (helps autorotation).
    Is autorotation self-sustaining?  Is combined load within thrust range?

  Phase 3 -- Landing flare:
    Hub in fast descent (3 m/s); tether goes slack; increase collective +0.05 rad.
    Does the flare produce sufficient net upward force to arrest the descent?
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aero import rotor_definition as rd
from aero import create_aero

_ROTOR = rd.default()

MASS_KG  = 5.0
G        = 9.81
WEIGHT_N = MASS_KG * G           # 49.05 N

WIND_NED = np.array([0.0, 10.0, 0.0])   # 10 m/s East crosswind
OMEGA_IC = 28.0                          # rad/s  (autorotation equilibrium at v_ip=10 m/s)

COL_MIN = -0.28   # collective floor  [rad]
COL_MAX =  0.10   # collective ceiling [rad]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _R_disk_horizontal() -> np.ndarray:
    """
    Rotation matrix for a horizontal disk (body_z = [0,0,-1] NED = pointing UP).

    Construction (det=1, verified):
      bz = [0,0,-1],  ref=[1,0,0]
      bx = cross(ref, bz) = [0, 1, 0]  (East)
      by = cross(bz,  bx) = [1, 0, 0]  (North)
      R  = [bx | by | bz]
    """
    return np.array([
        [0.0, 1.0,  0.0],
        [1.0, 0.0,  0.0],
        [0.0, 0.0, -1.0],
    ], dtype=float)


def _aero(col: float, v_hub_ned: np.ndarray,
          omega: float = OMEGA_IC):
    """
    Single force evaluation with Peters-He at t=10 s (past 5 s ramp → ramp=1).
    Cold-starts to quasi-static fixed point on every call (fresh instance).

    Returns
    -------
    thrust_up_N : float  upward force in NED [N]; positive = resisting gravity
    Q_spin_Nm   : float  net spin torque [N·m]; positive = rotor accelerating
    """
    aero  = create_aero(_ROTOR, model="peters_he")
    R_hub = _R_disk_horizontal()
    res   = aero.compute_forces(
        collective_rad=col, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R_hub, v_hub_world=v_hub_ned,
        omega_rotor=omega, wind_world=WIND_NED, t=10.0,
    )
    thrust_up = float(-res.F_world[2])   # NED: up = -z component
    return thrust_up, float(res.Q_spin)


def _bisect_col(target_N: float, v_hub_ned: np.ndarray,
                omega: float = OMEGA_IC) -> float:
    """
    Bisect collective in [COL_MIN, COL_MAX] to achieve target_N thrust.
    Returns NaN if target is outside the achievable range.
    """
    T_lo, _ = _aero(COL_MIN, v_hub_ned, omega)
    T_hi, _ = _aero(COL_MAX, v_hub_ned, omega)
    if (T_lo - target_N) * (T_hi - target_N) > 0:
        return float("nan")
    lo, hi = COL_MIN, COL_MAX
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        T_mid, _ = _aero(mid, v_hub_ned, omega)
        if (T_mid - target_N) * (T_lo - target_N) < 0:
            hi = mid
        else:
            lo, T_lo = mid, T_mid
        if hi - lo < 1e-5:
            break
    return 0.5 * (lo + hi)


# ---------------------------------------------------------------------------
# Phase 1 -- Vertical hover above anchor
# ---------------------------------------------------------------------------

def test_vertical_hover_feasibility():
    """
    Disk horizontal (xi=90 deg), hub stationary above anchor, 10 m/s crosswind.

    Autorotation at omega=28 rad/s is self-sustaining: the crosswind provides
    v_inplane=10 m/s regardless of disk angle, and BEM equilibrium gives omega_eq = 28 rad/s.

    Asserts:
      (a) T(COL_MIN) < weight < T(COL_MAX)  -- equilibrium collective exists.
      (b) col_eq is within [COL_MIN, COL_MAX].
      (c) Q_spin >= 0 at col_eq  -- autorotation self-sustaining at hover.
    """
    v_hub = np.zeros(3)

    T_lo, Q_lo = _aero(COL_MIN, v_hub)
    T_hi, Q_hi = _aero(COL_MAX, v_hub)
    col_eq     = _bisect_col(WEIGHT_N, v_hub)
    _, Q_eq    = _aero(col_eq, v_hub)

    print(f"\nPhase 1 -- vertical hover  (v_hub=0, v_ip=10 m/s, omega={OMEGA_IC} rad/s):")
    print(f"  T(COL_MIN={COL_MIN})  = {T_lo:.1f} N   Q_spin={Q_lo:.2f} Nm")
    print(f"  T(COL_MAX={COL_MAX}) = {T_hi:.1f} N   Q_spin={Q_hi:.2f} Nm")
    print(f"  weight               = {WEIGHT_N:.1f} N")
    print(f"  col_eq               = {col_eq:.4f} rad   Q_spin={Q_eq:.2f} Nm")

    assert T_lo < WEIGHT_N, (
        f"T(COL_MIN)={T_lo:.1f}N >= weight {WEIGHT_N:.1f}N: "
        "can't reduce thrust enough to descend from hover"
    )
    assert T_hi > WEIGHT_N, (
        f"T(COL_MAX)={T_hi:.1f}N <= weight {WEIGHT_N:.1f}N: "
        "rotor cannot support hub weight above anchor with 10 m/s crosswind"
    )
    assert not np.isnan(col_eq), "No equilibrium collective found (bisection failed)"
    assert COL_MIN <= col_eq <= COL_MAX, f"col_eq={col_eq:.4f} outside [{COL_MIN}, {COL_MAX}]"


# ---------------------------------------------------------------------------
# Phase 2 -- Powered descent (winch pulls hub down)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("v_desc_ms,tether_n", [
    (1.0,  50.0),
    (2.0, 100.0),
    (3.0, 150.0),
])
def test_powered_descent_force_balance(v_desc_ms, tether_n):
    """
    Hub descending at v_desc_ms m/s (NED vel_z = +v_desc_ms).
    Winch tether pulls down at tether_n N.

    With disk horizontal and 10 m/s crosswind:
      v_axial   = v_desc_ms   (upward airflow through disk = descent contributes to autorotation)
      v_inplane = 10 m/s      (crosswind, independent of descent speed)

    Total downward load = weight + tether_n.  Rotor must produce this much upward thrust.

    Asserts:
      (a) v_axial > 0: downward descent produces upward airflow.
      (b) Combined load is bracketed by [T(COL_MIN), T(COL_MAX)].
      (c) Equilibrium collective exists in range.
      (d) Autorotation Q_spin >= 0 at equilibrium.
    """
    v_hub  = np.array([0.0, 0.0, v_desc_ms])   # NED: downward
    load_n = WEIGHT_N + tether_n

    disk_up = np.array([0.0, 0.0, -1.0])        # disk normal = up in NED
    v_axial = float(np.dot(WIND_NED - v_hub, disk_up))

    T_lo, _ = _aero(COL_MIN, v_hub)
    T_hi, _ = _aero(COL_MAX, v_hub)
    col_eq  = _bisect_col(load_n, v_hub)
    _, Q_eq = _aero(col_eq, v_hub)

    print(f"\nPhase 2 -- powered descent  v={v_desc_ms} m/s  T_tether={tether_n} N:")
    print(f"  v_axial (up through disk) = {v_axial:.2f} m/s")
    print(f"  total load (wt+tether)    = {load_n:.1f} N")
    print(f"  thrust range              = [{T_lo:.1f}, {T_hi:.1f}] N")
    print(f"  col_eq                    = {col_eq:.4f} rad   Q_spin={Q_eq:.2f} Nm")

    assert v_axial > 0, (
        f"v_axial={v_axial:.2f} m/s at v_desc={v_desc_ms} m/s: "
        "no upward airflow through disk -- autorotation not driven by descent"
    )
    assert T_lo < load_n, (
        f"T(COL_MIN)={T_lo:.1f}N >= load {load_n:.1f}N: "
        "can't reduce thrust enough to maintain powered descent"
    )
    assert T_hi > load_n, (
        f"T(COL_MAX)={T_hi:.1f}N <= load {load_n:.1f}N: "
        f"rotor can't balance weight+tether at v_desc={v_desc_ms}m/s"
    )
    assert not np.isnan(col_eq), (
        f"No equilibrium collective for load={load_n:.0f}N at v_desc={v_desc_ms}m/s. "
        f"Thrust range [{T_lo:.1f}, {T_hi:.1f}]N."
    )


# ---------------------------------------------------------------------------
# Phase 3 -- Landing flare
# ---------------------------------------------------------------------------

def test_landing_flare_decelerates():
    """
    Hub in fast final descent (v_desc = 3 m/s), tether released (T->0 at flare).
    Increase collective by 0.05 rad to arrest descent.

    At flare: only load is hub weight (tether slack).
    Flare collective = descent_collective + 0.05 rad.

    Asserts:
      (a) col_descent is feasible (within range).
      (b) At col_flare, thrust > weight by at least 20 N (upward net force).
      (c) col_flare remains within collective ceiling.
    """
    V_FINAL  = 3.0   # m/s approach speed
    v_hub    = np.array([0.0, 0.0, V_FINAL])

    col_descent = _bisect_col(WEIGHT_N, v_hub)   # hold altitude at 3 m/s (tether slack)
    col_flare   = min(COL_MAX, col_descent + 0.05)

    T_descent, Q_desc = _aero(col_descent, v_hub)
    T_flare,   Q_flar = _aero(col_flare,   v_hub)
    delta_T           = T_flare - T_descent

    print(f"\nPhase 3 -- landing flare  (v_desc={V_FINAL} m/s, tether released):")
    print(f"  col_descent = {col_descent:.4f} rad   T={T_descent:.1f}N  Q={Q_desc:.2f}Nm")
    print(f"  col_flare   = {col_flare:.4f} rad   T={T_flare:.1f}N  Q={Q_flar:.2f}Nm")
    print(f"  delta_T     = {delta_T:.1f} N  (net upward deceleration force)")
    print(f"  weight      = {WEIGHT_N:.1f} N")

    assert not np.isnan(col_descent), (
        "Could not find descent equilibrium collective: "
        "hub cannot fly level at 3 m/s with horizontal disk and crosswind"
    )
    assert col_flare <= COL_MAX, (
        f"col_flare={col_flare:.4f} exceeds COL_MAX={COL_MAX}: "
        "flare requires more collective than available"
    )
    assert T_flare > WEIGHT_N, (
        f"Flare thrust {T_flare:.1f}N <= weight {WEIGHT_N:.1f}N: "
        "cannot arrest descent -- more collective needed or v_final too high"
    )
    assert delta_T >= 20.0, (
        f"Flare delta-thrust {delta_T:.1f}N < 20N: "
        "insufficient deceleration margin for safe landing"
    )
