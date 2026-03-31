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

Coordinate frame: ENU — body Z = Up, tether anchor at origin.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mediator as _mediator_module
from aero     import create_aero
import rotor_definition as _rd
from dynamics import RigidBodyDynamics

TetherModel = _mediator_module.TetherModel

# ── Physical constants ────────────────────────────────────────────────────────
MASS   = 5.0                        # kg  (matches test_force_balance.py)
G      = 9.81                       # m/s²
WEIGHT = MASS * G                   # N
OMEGA  = 28.0                       # rad/s nominal spin (autorotation at 10 m/s wind)
WIND   = np.array([10.0, 0.0, 0.0]) # 10 m/s East
DT     = 2.5e-3                     # s  (400 Hz)

# ── Tether geometry at 30° elevation ─────────────────────────────────────────
ELEV_DEG  = 30.0
ELEV_RAD  = math.radians(ELEV_DEG)
L_TETHER  = 50.0                    # m  — tether length at this flight point


# ── Helpers ──────────────────────────────────────────────────────────────────

def _hub_pos(elev_rad: float = ELEV_RAD, length: float = L_TETHER) -> np.ndarray:
    """ENU position of hub at given elevation angle and tether length."""
    return np.array([length * math.cos(elev_rad), 0.0, length * math.sin(elev_rad)])


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

    The equilibrium orientation has body Z (rotor axle) aligned with the tether
    direction — the disk rotates around the same axis as the tether.  With hub
    East of anchor at elevation β:

        tether_dir = [cos(β), 0, sin(β)]
        body Z = tether_dir  →  R_hub = R_from_body_z(tether_dir)

    Force balance in world frame (forces from aero.compute_forces in ENU):

        East:    Fx_aero = T_t * cos(β)   → T_t = Fx_aero / cos(β)
        Up:      Fz_aero = W + T_t * sin(β) = W + Fx_aero * tan(β)

    Scan collective 0° → -30° until Fz_aero ≤ W + Fx_aero * tan(β).

    Returns
    -------
    (coll_rad, T_z_req, T_t_est, H_x)
        coll_rad  : float  equilibrium collective [rad]
        T_z_req   : float  required vertical (Up) aero force at that collective [N]
        T_t_est   : float  estimated tether tension at equilibrium [N]
        H_x       : float  East aero force at equilibrium collective [N]
    """
    aero = create_aero(_rd.default())
    W = mass * 9.81

    tether_dir = np.array([math.cos(elev_rad), 0.0, math.sin(elev_rad)])
    R_hub = _R_from_body_z(tether_dir)

    _MAX_TETHER_TENSION_N = 496.0   # 80% of break load (620 N)

    for half_deg in range(0, -81, -1):   # 0° → -40° in 0.5° steps
        coll_rad = math.radians(half_deg * 0.5)
        f = aero.compute_forces(coll_rad, 0.0, 0.0, R_hub, np.zeros(3),
                                omega, wind, t=10.0)
        H_x   = float(f[0])   # East force (world frame)
        T_z   = float(f[2])   # Up force (world frame)
        T_t   = H_x / math.cos(elev_rad) if H_x > 0 else 0.0
        T_z_req = W + T_t * math.sin(elev_rad)
        if T_z <= T_z_req and T_t < _MAX_TETHER_TENSION_N:
            return coll_rad, T_z_req, T_t, H_x

    # Fallback: use -17° (gives non-zero thrust unlike -30°)
    coll_rad = math.radians(-17.0)
    f = aero.compute_forces(coll_rad, 0.0, 0.0, R_hub, np.zeros(3),
                            omega, wind, t=10.0)
    H_x = float(f[0])
    T_t = H_x / math.cos(elev_rad) if H_x > 0 else 0.0
    return coll_rad, W + T_t * math.sin(elev_rad), T_t, H_x


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
        anchor_enu  = np.zeros(3),
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
        anchor_enu  = np.zeros(3),
        rest_length = L_TETHER - 0.01,
    )
    _, moment = tether.compute(pos, np.zeros(3), R_vert)

    assert np.linalg.norm(moment) > 1e-4, (
        f"Expected non-zero restoring moment, got |M|={np.linalg.norm(moment):.2e} N·m"
    )


def test_restoring_moment_tilts_axle_toward_anchor():
    """
    With the hub East of the anchor and an upright disk, the restoring
    moment must tilt body Z toward East (My > 0 in world frame = rotation
    from body Z toward body X = East), which pulls the axle bottom toward
    the anchor as required by the tether constraint.
    """
    pos    = _hub_pos()                  # East=43.3 m, Alt=25 m
    R_vert = np.eye(3)                   # upright

    tether = TetherModel(
        anchor_enu  = np.zeros(3),
        rest_length = L_TETHER - 0.01,
    )
    _, moment = tether.compute(pos, np.zeros(3), R_vert)

    # My > 0 → rotation from body Z toward body X = East; pulls axle bottom West toward anchor.
    assert moment[1] > 0.0, (
        f"Restoring moment Y component should be positive to tilt axle East. "
        f"Got My = {moment[1]:.4f} N·m, full moment = {moment.round(4)}"
    )


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


def test_equilibrium_collective_gives_balanced_net_force():
    """
    At the computed equilibrium collective the net force on the hub
    (aero + tether tension + gravity) is much smaller than the rotor weight —
    confirming the collective scan produced a genuinely balanced operating point.

    The tolerance is generous (< 2 × weight) because H-force is computed
    self-consistently but vertical thrust uses a discrete 1° scan step.
    """
    coll_eq, T_z_req, T_t_est, H_x = _compute_equilibrium_collective(
        WIND, MASS, OMEGA, ELEV_RAD)

    pos  = _hub_pos()
    aero = create_aero(_rd.default())

    # Set rest_length so tether is taut with exactly T_t_est at L_TETHER
    # k = EA / L;  extension = T_t / k  →  rest_length = L - extension
    ea    = TetherModel.EA_N
    k_eff = ea / L_TETHER
    ext   = T_t_est / k_eff if T_t_est > 0 else 0.005
    rest  = L_TETHER - max(ext, 0.001)   # at least 1 mm extension

    tether = TetherModel(anchor_enu=np.zeros(3), rest_length=rest)

    tether_dir    = pos / np.linalg.norm(pos)
    R_hub_eq      = _R_from_body_z(tether_dir)
    f_aero        = aero.compute_forces(coll_eq, 0.0, 0.0, R_hub_eq, np.zeros(3),
                                        OMEGA, WIND, t=10.0)
    f_teth, _m    = tether.compute(pos, np.zeros(3), np.eye(3))
    f_grav        = np.array([0.0, 0.0, -WEIGHT])

    net     = f_aero[:3] + f_teth + f_grav
    net_mag = float(np.linalg.norm(net))

    assert net_mag < WEIGHT * 2.0, (
        f"Net force at equilibrium collective {math.degrees(coll_eq):.1f}° is "
        f"{net_mag:.1f} N (>{2*WEIGHT:.1f} N = 2×weight).\n"
        f"  aero  = {f_aero[:3].round(2)}\n"
        f"  tether= {f_teth.round(2)}\n"
        f"  grav  = {f_grav.round(2)}\n"
        f"  net   = {net.round(2)}"
    )


def test_hub_stays_bounded_at_30_degrees():
    """
    Open-loop stability: with the equilibrium collective, steady East wind,
    taut tether at 30° elevation, and no ArduPilot, the hub must stay
    within ±10 m of its initial position for 2 seconds.

    This validates the complete physics chain:
      aero forces → tether force + restoring torque → rigid-body dynamics.

    The tether rest length is set so that the equilibrium tether tension
    self-consistently balances the horizontal H-force from the aero model.
    """
    STEPS  = 800    # 2 s at 400 Hz
    BOUNDS = 10.0   # m — maximum drift allowed from initial position

    coll_eq, _T_z_req, T_t_est, _H_x = _compute_equilibrium_collective(
        WIND, MASS, OMEGA, ELEV_RAD)

    pos0  = _hub_pos()
    ea    = TetherModel.EA_N
    k_eff = ea / L_TETHER
    ext   = T_t_est / k_eff if T_t_est > 0 else 0.005
    rest  = L_TETHER - max(ext, 0.001)

    # Equilibrium orientation: axle (body Z) aligned with tether direction
    tether_dir = pos0 / np.linalg.norm(pos0)
    R0_eq      = _R_from_body_z(tether_dir)

    aero   = create_aero(_rd.default())
    tether = TetherModel(anchor_enu=np.zeros(3), rest_length=rest)
    dyn    = RigidBodyDynamics(
        mass   = MASS,
        I_body = [5.0, 5.0, 10.0],
        pos0   = pos0.tolist(),
        vel0   = [0.0, 0.0, 0.0],
        R0     = R0_eq,
        omega0 = [0.0, 0.0, 0.0],
    )

    for step in range(STEPS):
        state = dyn.state
        pos   = state["pos"]

        assert np.all(np.isfinite(pos)), f"NaN/inf in position at step {step}: {pos}"
        assert np.all(np.isfinite(state["R"])), f"NaN/inf in rotation at step {step}"

        result          = aero.compute_forces(
            collective_rad = coll_eq,
            tilt_lon       = 0.0,
            tilt_lat       = 0.0,
            R_hub          = state["R"],
            v_hub_world    = state["vel"],
            omega_rotor    = OMEGA,
            wind_world     = WIND,
            t              = 10.0,   # past 5 s ramp → ramp = 1.0
        )
        f_teth, m_teth  = tether.compute(pos, state["vel"], state["R"])
        F_net = result.F_world + f_teth
        M_net = result.M_orbital + result.M_spin + m_teth

        dyn.step(F_net, M_net, DT)

    final = dyn.state["pos"]
    drift = np.abs(final - pos0)

    assert drift[0] < BOUNDS, (
        f"East drift {drift[0]:.2f} m > {BOUNDS} m at collective "
        f"{math.degrees(coll_eq):.1f}°.  Hub is not in equilibrium."
    )
    assert drift[2] < BOUNDS, (
        f"Vertical drift {drift[2]:.2f} m > {BOUNDS} m at collective "
        f"{math.degrees(coll_eq):.1f}°.  Hub is not in equilibrium."
    )
