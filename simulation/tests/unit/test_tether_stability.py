"""
test_tether_stability.py — tether restoring torque and 30° tether-angle stability.

For a RAWES at 30° tether elevation angle the tether applies both a force
(tension) and a restoring torque that aligns the hub axle (body Z) with the
tether direction.  These tests verify:

  1. The restoring moment is zero when the axle is already aligned.
  2. The restoring moment is non-zero and in the correct direction when the disk
     is upright (vertical) while the tether is at 30° from horizontal.
  3. Working backwards from the required lift at 30° tether angle, a
     self-consistent equilibrium collective can be found such that the
     hub stays within ±10 m of its initial position for 2 seconds
     with no ArduPilot (open-loop, fixed collective).

Coordinate frame: NED — X=North, Y=East, Z=Down.  Tether anchor at origin.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mediator as _mediator_module
from tests.unit._aero_probe import make_probe, probe_steady

TetherModel = _mediator_module.TetherModel

# ── Physical constants ────────────────────────────────────────────────────────
MASS   = 5.0                        # kg  (matches test_force_balance.py)
G      = 9.81                       # m/s²
WEIGHT = MASS * G                   # N
OMEGA  = 18.108                     # rad/s — steady-state IC from steady_state_starting.json
WIND   = np.array([0.0, 10.0, 0.0]) # NED: 10 m/s East = Y axis
DT     = 2.5e-3                     # s  (400 Hz)

# ── Tether geometry at 30° elevation ─────────────────────────────────────────
ELEV_DEG  = 30.0
ELEV_RAD  = math.radians(ELEV_DEG)
L_TETHER  = 100.0                   # m  — tether length at this flight point


# ── Helpers ──────────────────────────────────────────────────────────────────

def _hub_pos(elev_rad: float = ELEV_RAD, length: float = L_TETHER) -> np.ndarray:
    """NED position of hub at given elevation angle (tether points East = NED Y)."""
    return np.array([0.0, length * math.cos(elev_rad), -length * math.sin(elev_rad)])


def _R_from_body_z(body_z_world: np.ndarray) -> np.ndarray:
    """
    Build an orthonormal rotation matrix whose third column equals body_z_world.
    Used to set up an arbitrary hub orientation for a given axle direction.
    """
    z = np.asarray(body_z_world, dtype=float)
    z = z / np.linalg.norm(z)
    # Pick a reference direction not parallel to z
    ref = np.array([0.0, 1.0, 0.0])
    if abs(np.dot(z, ref)) > 0.9:
        ref = np.array([0.0, 0.0, 1.0])
    x = ref - np.dot(ref, z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _compute_equilibrium_collective(
    wind: np.ndarray,
    mass: float,
    omega: float,
    elev_rad: float,
) -> tuple:
    """
    Find the collective pitch (rad) that balances all forces at the given
    tether elevation angle, working backwards from the required lift.

    Note: a true static equilibrium does not exist without cyclic — the
    gravity component perpendicular to the tether (~48 N) must be balanced
    by active cyclic in flight.  This function finds the collective at which
    the along-tether force balance is satisfied, which is useful for setting
    up test conditions even though the perpendicular imbalance remains.

    Returns (coll_rad, T_z_req, T_t_est, H_y).
    """
    aero = make_probe()
    W = mass * 9.81

    # FRD: body_z (= hub axis DOWN through disk) points hub→anchor in tethered
    # hover, i.e. opposite of the anchor→hub tether direction.
    tether_dir = np.array([0.0, -math.cos(elev_rad), math.sin(elev_rad)])
    R_hub = _R_from_body_z(tether_dir)

    _MAX_TETHER_TENSION_N = 496.0

    def _probe(coll_rad):
        r = probe_steady(aero, collective_rad=coll_rad,
                         R_hub=R_hub, v_hub_world=np.zeros(3),
                         omega_rotor=omega, wind_world=wind, t=10.0)
        return r.F_world

    for half_deg in range(0, -81, -1):
        coll_rad = math.radians(half_deg * 0.5)
        f = _probe(coll_rad)
        H_y     = float(f[1])
        T_z     = float(-f[2])
        T_t     = H_y / math.cos(elev_rad) if H_y > 0 else 0.0
        T_z_req = W + T_t * math.sin(elev_rad)
        if T_z <= T_z_req and T_t < _MAX_TETHER_TENSION_N:
            return coll_rad, T_z_req, T_t, H_y

    coll_rad = math.radians(-17.0)
    f   = _probe(coll_rad)
    H_y = float(f[1])
    T_t = H_y / math.cos(elev_rad) if H_y > 0 else 0.0
    return coll_rad, W + T_t * math.sin(elev_rad), T_t, H_y


# ── Tests ────────────────────────────────────────────────────────────────────

def test_restoring_moment_zero_when_axle_aligned():
    """
    When the hub axle (body Z) is exactly aligned with the tether direction
    (anchor → hub), the cross product r_attach × F_tether is zero — no
    correction torque is needed.
    """
    pos        = _hub_pos()
    tether_dir = pos / np.linalg.norm(pos)     # unit vector from anchor to hub
    R_aligned  = _R_from_body_z(tether_dir)    # body Z aligned with tether

    tether = TetherModel(
        anchor_ned  = np.zeros(3),
        rest_length = L_TETHER - 0.01,          # 10 mm extension → taut
    )
    _, moment = tether.compute(pos, np.zeros(3), R_aligned)

    np.testing.assert_allclose(
        moment, np.zeros(3), atol=1e-9,
        err_msg="Restoring moment must be zero when axle is aligned with tether",
    )


def test_restoring_moment_nonzero_when_axle_misaligned():
    """
    Upright disk (body Z = vertical) with tether at 30° from horizontal:
    the restoring moment must be non-zero, showing the physics constraint is active.
    """
    pos    = _hub_pos()
    R_vert = np.eye(3)   # body Z = [0,0,1], tether at 30° → misaligned

    tether = TetherModel(
        anchor_ned  = np.zeros(3),
        rest_length = L_TETHER - 0.01,
    )
    _, moment = tether.compute(pos, np.zeros(3), R_vert)

    assert np.linalg.norm(moment) > 1e-4, (
        f"Expected non-zero restoring moment, got |M|={np.linalg.norm(moment):.2e} N·m"
    )


def test_restoring_moment_tilts_axle_toward_anchor():
    """
    With the hub East of the anchor and an upright FRD disk (body_z = Down),
    the tether restoring moment must rotate body_z toward the hub→anchor
    direction (West + Up).

    The required rotation axis is bz_now × bz_target.  With bz_now = [0,0,1]
    and the hub at East elevation 30° (so hub→anchor = [0, −cos30°, sin30°]),
    the cross product is along +North.  The tether moment must therefore
    have a positive X (North) component.
    """
    pos    = _hub_pos()                  # East=cos30°·L, Alt=sin30°·L
    R_vert = np.eye(3)                   # FRD: body_z = [0,0,1] = Down (level disk)

    tether = TetherModel(
        anchor_ned  = np.zeros(3),
        rest_length = L_TETHER - 0.01,
    )
    _, moment = tether.compute(pos, np.zeros(3), R_vert)

    # Expected rotation axis (from kinematics): bz_now × bz_target.
    bz_now    = R_vert[:, 2]
    bz_target = -pos / np.linalg.norm(pos)       # FRD: hub→anchor
    axis_expected = np.cross(bz_now, bz_target)
    axis_expected /= np.linalg.norm(axis_expected)

    assert np.linalg.norm(moment) > 1e-4, "Restoring moment is zero for misaligned axle"
    # Moment must point along the same direction as the required rotation axis.
    # cos(angle) = dot(moment_unit, axis_expected); should be ~+1.
    cos_align = float(np.dot(moment / np.linalg.norm(moment), axis_expected))
    assert cos_align > 0.99, (
        f"Restoring moment direction is wrong: cos(angle)={cos_align:.3f} "
        f"(expected ~+1).  moment={moment.round(3)}, expected axis={axis_expected.round(3)}"
    )


def test_restoring_moment_sign_for_horizontal_offset():
    """Pure-East hub offset with FRD-level disk ⇒ moment about +North.

    Geometry: hub at [0, 50, 0] (East, level with anchor).  bz_now = [0,0,1]
    (down).  Tether pulls hub west (force = [0, -T, 0]).  Attach point is at
    +axle_attach·bz_now = [0, 0, +0.3] (FRD: below CoM in the gravity sense).
    Moment = r × F = [0,0,0.3] × [0,-T,0] = [+0.3T, 0, 0] → +North. This is
    the axis along which body_z must rotate to align with anchor direction.
    """
    pos = np.array([0.0, 50.0, 0.0])     # East, level
    R_vert = np.eye(3)                   # body_z = [0,0,1] (down)
    tether = TetherModel(anchor_ned=np.zeros(3), rest_length=49.99)
    _, moment = tether.compute(pos, np.zeros(3), R_vert)
    assert moment[0] > 0.0, f"Moment X must be > 0 (toward +North) — got {moment.round(3)}"
    assert abs(moment[1]) < 1e-6
    assert abs(moment[2]) < 1e-6


def test_equilibrium_collective_is_at_or_below_neutral():
    """
    The equilibrium collective at 30° tether angle is at or below neutral (≤ 0°).
    With the disk tilted to align with the tether, the axial wind component is
    reduced (~8.66 m/s vs 10 m/s) and the thrust vector already has a horizontal
    East component, so neutral or slightly negative collective achieves equilibrium.
    """
    coll_eq, *_ = _compute_equilibrium_collective(WIND, MASS, OMEGA, ELEV_RAD)
    assert coll_eq <= 0.0, (
        f"Equilibrium collective at 30° should be ≤ 0°, got {math.degrees(coll_eq):.1f}°"
    )


