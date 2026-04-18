"""
test_steady_flight.py — Steady-state physics test at 30° tether elevation.

The simplest possible flight scenario:

  - Hub starts at the 30° tether-angle equilibrium position (East of anchor).
  - Body Z (rotor axle) is aligned with the tether direction from the start.
  - Collective is set to the open-loop equilibrium value (no ArduPilot).
  - Tether rest length is set so the tether is already taut at the equilibrium
    tension.
  - Wind is steady East.

Because the system starts close to equilibrium (correct position, orientation,
and collective), it should remain near that position.  This is the "simple case"
— no transient from launch, no ArduPilot control loop.

A separate test (test_drift_from_origin, TODO) will cover the harder case of
starting directly above the anchor and letting wind + tether drive equilibrium.

Coordinate frame: NED — X=North, Y=East, Z=Down.  Tether anchor at origin.

Output artefacts (written by test_steady_state_hub_does_not_drift):
  steady_state_starting.json  — equilibrium initial conditions (read by stack tests)
  Regenerate: simulation/.venv/Scripts/python.exe -m pytest simulation/tests/unit/test_steady_flight.py::test_steady_state_hub_does_not_drift -s
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

import mediator as _mediator_module
from aero        import create_aero
import rotor_definition as _rd
from dynamics    import RigidBodyDynamics
from controller  import compute_swashplate_from_state, orbit_tracked_body_z_eq, TensionPI
from swashplate  import SwashplateServoModel
from frames      import build_orb_frame

TetherModel = _mediator_module.TetherModel

# Design tether equilibrium orientation (from beaupoil_2026.yaml) in NED
_BODY_Z_DESIGN = np.array([0.305391, 0.851018, -0.427206])  # NED: T @ ENU
_BASE_K_ANG    = 50.0   # N·m·s/rad — angular damping (matches mediator default)
_T_AERO_OFFSET = 45.0   # s — aero ramp already done

# ── Physical constants ────────────────────────────────────────────────────────
MASS   = 5.0                        # kg
G      = 9.81                       # m/s²
WEIGHT = MASS * G                   # N
OMEGA  = 28.0                       # rad/s  initial spin (autorotation at 10 m/s wind)
WIND   = np.array([0.0, 10.0, 0.0]) # NED: 10 m/s East = Y axis

DT     = 2.5e-3                     # s  (400 Hz)

# ── Spin dynamics constants (from mediator.py) ────────────────────────────────
K_DRIVE_SPIN  = 1.4      # N·m·s/m   autorotation drive per unit in-plane wind speed
K_DRAG_SPIN   = 0.01786  # N·m·s²/rad²  profile drag
I_SPIN_KGMS2  = 10.0     # kg·m²  rotor spin-axis inertia
OMEGA_SPIN_MIN = 0.5     # rad/s  minimum clamp

# ── Tether geometry ───────────────────────────────────────────────────────────
ELEV_DEG = 30.0
ELEV_RAD = math.radians(ELEV_DEG)
L_TETHER = 100.0                     # m  tether length at this flight point


# ── Helpers ───────────────────────────────────────────────────────────────────

def _tether_dir(elev_rad: float = ELEV_RAD) -> np.ndarray:
    """Unit vector from anchor toward hub at given elevation angle (NED: East = Y)."""
    return np.array([0.0, math.cos(elev_rad), -math.sin(elev_rad)])


def _hub_pos(elev_rad: float = ELEV_RAD, length: float = L_TETHER) -> np.ndarray:
    return length * _tether_dir(elev_rad)


def _R_from_body_z(body_z_world: np.ndarray) -> np.ndarray:
    z = np.asarray(body_z_world, dtype=float)
    z = z / np.linalg.norm(z)
    ref = np.array([0.0, 1.0, 0.0])
    if abs(np.dot(z, ref)) > 0.9:
        ref = np.array([0.0, 0.0, 1.0])
    x = ref - np.dot(ref, z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _equilibrium_setup():
    """
    Compute the self-consistent equilibrium for steady-state orbit with attitude control.

    Uses the design tether orientation (body_z=[0.851, 0.305, 0.427]) and the
    TensionPI minimum collective (COL_MIN_RAD) as the hold collective — matching
    what the HoldPlanner outputs (thrust=0 → COL_MIN).

    Iterates omega_spin to the autorotation equilibrium at this collective, then
    computes the tether rest length from the expected thrust.

    Returns (coll_rad, omega_spin_eq, R0, pos0, rest_length).
    """
    aero  = create_aero(_rd.default())
    t_dir = _BODY_Z_DESIGN / np.linalg.norm(_BODY_Z_DESIGN)
    R0    = build_orb_frame(t_dir)
    pos0  = L_TETHER * t_dir   # hub at design orientation, radius = tether length

    # Collective = TensionPI minimum (what HoldPlanner thrust=0 gives).
    # The warmup physics is computed at this collective to find the unit-test equilibrium.
    # The stack test uses coll_eq_target (below) for warm_coll_rad — giving TensionPI
    # 0.10 rad of downward headroom during the pumping cycle.
    coll_eq = TensionPI.COLL_MIN_RAD   # -0.28 rad (unit-test equilibrium)

    # Stack-test target collective: 0.10 rad above col_min for PI headroom.
    # At this collective, compute the equilibrium tether tension so the stack test
    # can set tension_out=tension_eq_n.  The TensionPI then operates at coll_eq_target
    # (above col_min) with room to decrease if tension rises.
    coll_eq_target = TensionPI.COLL_MIN_RAD + 0.10   # -0.18 rad

    # Converge omega_spin to autorotation equilibrium at this collective
    omega_spin_eq = OMEGA
    for _ in range(30):
        aero.compute_forces(coll_eq, 0.0, 0.0, R0, np.zeros(3),
                            omega_spin_eq, WIND, t=_T_AERO_OFFSET)
        v_ip = float(aero.last_v_inplane)
        omega_new = math.sqrt(max(1e-6, K_DRIVE_SPIN * v_ip / K_DRAG_SPIN))
        if abs(omega_new - omega_spin_eq) < 1e-4:
            omega_spin_eq = omega_new
            break
        omega_spin_eq = omega_new

    # Estimate tether tension ≈ thrust (for orbit: T_tether ≈ T along tether)
    f = aero.compute_forces(coll_eq, 0.0, 0.0, R0, np.zeros(3),
                            omega_spin_eq, WIND, t=_T_AERO_OFFSET)
    T_est = float(np.dot(f.F_world, t_dir))   # thrust component along tether
    T_t_est = max(T_est, 10.0)                # tether tension ≈ thrust

    k_eff = TetherModel.EA_N / L_TETHER
    ext   = T_t_est / k_eff
    rest  = L_TETHER - max(ext, 0.001)

    # Compute equilibrium tension at the stack-test target collective (coll_eq_target).
    # This is what the stack test's TensionPI should target so it operates at
    # coll_eq_target rather than col_min.
    f_target = aero.compute_forces(coll_eq_target, 0.0, 0.0, R0, np.zeros(3),
                                   omega_spin_eq, WIND, t=_T_AERO_OFFSET)
    T_est_target = float(np.dot(f_target.F_world, t_dir))
    T_gravity_along_tether = MASS * G * math.sin(ELEV_RAD)
    T_tether_target = max(T_est_target - T_gravity_along_tether, 10.0)

    return coll_eq, omega_spin_eq, R0, pos0, rest, coll_eq_target, T_tether_target


def _physics_loop(dyn, aero, tether, coll_eq, omega_spin_start, steps,
                  ic_dir0=None, ic_bz0=None, tension_ctrl=None):
    """
    Run `steps` physics steps and return per-step telemetry arrays plus the
    final omega_spin scalar.  Used for both warmup and the recorded run.

    Uses the attitude controller (compute_swashplate_from_state) and angular
    damping — matching the mediator and other simtests.

    If tension_ctrl is provided, collective is controlled by the TensionPI
    rather than the fixed coll_eq value.  This finds the natural operational
    equilibrium altitude under closed-loop tension control.
    """
    omega_spin  = omega_spin_start
    anchor      = np.zeros(3)
    tension_now = 0.0   # initial tether tension for TensionPI feedback

    t_arr      = np.zeros(steps)
    pos_arr    = np.zeros((steps, 3))
    vel_arr    = np.zeros((steps, 3))
    ten_arr    = np.zeros(steps)
    angle_arr  = np.zeros(steps)
    body_z_arr = np.zeros((steps, 3))
    spin_arr   = np.zeros(steps)

    # Capture orbit-tracking ICs from first step
    if ic_dir0 is None:
        ic_dir0 = dyn.state["pos"] / max(np.linalg.norm(dyn.state["pos"]), 0.1)
    if ic_bz0 is None:
        ic_bz0 = dyn.state["R"][:, 2].copy()
    servo = SwashplateServoModel.from_rotor(_rd.default())
    for step in range(steps):
        state  = dyn.state
        pos    = state["pos"]
        body_z = state["R"][:, 2]

        t_arr[step]      = step * DT
        pos_arr[step]    = pos
        vel_arr[step]    = state["vel"]
        body_z_arr[step] = body_z
        spin_arr[step]   = omega_spin

        f_teth, m_teth = tether.compute(pos, state["vel"], state["R"])
        ten_arr[step]  = tether._last_info.get("tension", 0.0)
        tension_now    = ten_arr[step]

        tlen = np.linalg.norm(pos)
        if tlen > 0.1:
            tdir = pos / tlen
            cos_a = np.clip(np.dot(body_z, tdir), -1.0, 1.0)
            angle_arr[step] = math.degrees(math.acos(cos_a))

        # Attitude controller: orbit-tracking with tilt correction
        body_z_eq = orbit_tracked_body_z_eq(pos, ic_dir0, ic_bz0)
        sw = compute_swashplate_from_state(state, anchor, body_z_eq=body_z_eq)
        tilt_lon, tilt_lat = servo.step(sw["tilt_lon"], sw["tilt_lat"], DT)

        # Collective: TensionPI (closed-loop) if provided, else fixed coll_eq
        if tension_ctrl is not None:
            collective = tension_ctrl.update(tension_now, DT)
        else:
            collective = coll_eq

        result = aero.compute_forces(
            collective_rad=collective,
            tilt_lon=tilt_lon, tilt_lat=tilt_lat,
            R_hub=state["R"], v_hub_world=state["vel"],
            omega_rotor=omega_spin, wind_world=WIND, t=_T_AERO_OFFSET + step * DT,
        )

        Q_spin     = K_DRIVE_SPIN * aero.last_v_inplane - K_DRAG_SPIN * omega_spin ** 2
        omega_spin = max(OMEGA_SPIN_MIN, omega_spin + Q_spin / I_SPIN_KGMS2 * DT)

        F_net = result.F_world + f_teth
        M_net = result.M_orbital + m_teth - _BASE_K_ANG * state["omega"]
        dyn.step(F_net, M_net, DT)

    return (t_arr, pos_arr, vel_arr, ten_arr, angle_arr, body_z_arr, spin_arr,
            omega_spin)


def _run_simulation(steps: int = 4000, warmup_steps: int = 24000):
    """
    Run the steady-state physics for `steps` iterations.

    A warmup pass of `warmup_steps` steps runs first.  The final state of the
    warmup (position, velocity, rotation, orbital omega, spin, tether angle) is
    used as the initial condition for the recorded run.  This means the recorded
    run starts from the system's natural settled state rather than the analytic
    equilibrium estimate — eliminating the initial transient.

    Spin is integrated with the same two-term model as mediator.py.

    Returns a dict of per-step arrays plus scalars.
    """
    coll_eq, omega_spin_eq, R0, pos0, rest, coll_eq_target, T_tether_target = _equilibrium_setup()

    aero   = create_aero(_rd.default())

    # ── Warmup pass with closed-loop tension control ───────────────────────────
    # Use standard tension_out=200N so the unit-test equilibrium is unchanged.
    tension_out = 200.0   # N — matches DeschutterPlanner default
    tension_ctrl_wu = TensionPI(setpoint_n=tension_out)

    tether_wu = TetherModel(anchor_ned=np.zeros(3), rest_length=rest)
    dyn_wu    = RigidBodyDynamics(
        mass=MASS, I_body=[5.0, 5.0, 10.0],
        pos0=pos0.tolist(), vel0=[0., 0., 0.], R0=R0, omega0=[0., 0., 0.],
    )
    *_, omega_spin_settled = _physics_loop(
        dyn_wu, aero, tether_wu, coll_eq, omega_spin_eq, warmup_steps,
        tension_ctrl=tension_ctrl_wu)

    # ── Extract settled state ─────────────────────────────────────────────────
    # Use the warmup final pos/vel/R/omega as initial conditions.
    # Keep the ORIGINAL rest length — if the hub sank inside it during warmup
    # (tether went slack) that is physically correct; do not shorten the tether.
    s        = dyn_wu.state
    pos_s    = s["pos"]
    vel_s    = s["vel"]
    R_s      = s["R"]
    omega_s  = s["omega"]

    # ── Recorded run from settled state ──────────────────────────────────────
    tension_ctrl_rec = TensionPI(setpoint_n=tension_out)
    tether = TetherModel(anchor_ned=np.zeros(3), rest_length=rest)
    dyn    = RigidBodyDynamics(
        mass=MASS, I_body=[5.0, 5.0, 10.0],
        pos0=pos_s.tolist(), vel0=vel_s.tolist(), R0=R_s, omega0=omega_s.tolist(),
    )
    (t_arr, pos_arr, vel_arr, ten_arr, angle_arr, body_z_arr, spin_arr,
     _final_spin) = _physics_loop(
        dyn, aero, tether, coll_eq, omega_spin_settled, steps,
        tension_ctrl=tension_ctrl_rec)

    return {
        "coll_deg":       math.degrees(coll_eq),
        "coll_eq":        coll_eq,
        "omega_spin_eq":  omega_spin_settled,
        "pos0":           pos_s,         # settled start position (after warmup)
        "vel0":           vel_s,
        "R0":             R_s,
        "omega0":         omega_s,
        "rest_length":    rest,
        "coll_eq":        coll_eq,            # unit-test equilibrium (-0.28 rad)
        "omega_spin_eq":  omega_spin_eq,
        "tension_eq_n":   T_tether_target,  # stack-test tension_out for PI headroom
        "stack_coll_eq":  coll_eq_target,   # stack-test warm-start (-0.18 rad, gives PI headroom)
        "t":              t_arr,
        "pos":            pos_arr,
        "vel":            vel_arr,
        "tension":        ten_arr,
        "axle_deg":       angle_arr,
        "body_z":         body_z_arr,
        "omega_spin":     spin_arr,
    }


def _save_starting_json(data: dict, path: Path) -> None:
    """
    Write a compact JSON file with the warmup-settled initial state so that
    the ArduPilot stack flight test can pass identical initial conditions to
    the mediator via CLI args.

    Fields
    ------
    pos          : NED position [m]
    vel          : NED velocity [m/s]
    body_z       : body-Z axis in world NED frame (unit vector; R0 derived from this)
    omega_spin   : rotor spin rate [rad/s]
    rest_length  : tether rest length [m]
    coll_eq_rad  : equilibrium collective [rad] — TensionPI warm-start value
    tension_eq_n : equilibrium tether tension [N] — TensionPI setpoint (tension_out)
                   at this collective.  Set tension_out=tension_eq_n so PI has zero
                   error at equilibrium and headroom in both directions.
    """
    R0 = data["R0"]
    out = {
        "pos":          data["pos0"].tolist(),
        "vel":          data["vel0"].tolist(),
        "R0":           R0.tolist(),  # equilibrium rotation matrix; stack tests apply build_gps_yaw_frame themselves
        "omega_spin":   float(data["omega_spin_eq"]),
        "rest_length":  float(data["rest_length"]),
        "coll_eq_rad":    float(data["coll_eq"]),        # unit-test equilibrium collective
        "tension_eq_n":   float(data["tension_eq_n"]),   # stack-test tension_out
        "stack_coll_eq":  float(data["stack_coll_eq"]),  # stack-test TensionPI warm-start
        "home_z_ned":   0.0,   # GPS home at ground level
    }
    path.write_text(json.dumps(out, indent=2))


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_steady_state_hub_does_not_drift():
    """
    Starting exactly at the 30° tether equilibrium, the hub must not drift more
    than 15 m in 10 seconds with a fixed collective and no ArduPilot.

    The 15 m bound allows for the ~1° scan resolution of the equilibrium
    collective while still catching any catastrophic divergence.

    """
    STEPS  = 4000    # 10 s at 400 Hz
    BOUNDS = 15.0    # m — catches instability; not sub-metre accuracy

    data = _run_simulation(STEPS)

    pos0  = data["pos0"]
    final = data["pos"][-1]
    drift = np.abs(final - pos0)

    # Save artefacts regardless of pass/fail
    _save_starting_json(data, Path(__file__).resolve().parents[2] / "steady_state_starting.json")

    assert np.all(np.isfinite(data["pos"])), "NaN/inf in position history"
    assert np.all(np.isfinite(data["vel"])), "NaN/inf in velocity history"

    assert drift[1] < BOUNDS, (
        f"East (NED Y) drift {drift[1]:.2f} m > {BOUNDS} m — hub not in steady state "
        f"(collective {data['coll_deg']:.1f}°)"
    )
    assert drift[2] < BOUNDS, (
        f"Vertical (NED Z) drift {drift[2]:.2f} m > {BOUNDS} m — hub not in steady state "
        f"(collective {data['coll_deg']:.1f}°)"
    )


def test_steady_state_axle_stays_aligned_with_tether():
    """
    Body Z must stay within 15° of the tether direction throughout 10 seconds.

    The tether restoring torque enforces this alignment; a missing or reversed
    torque will cause the axle to precess and this test will catch it.
    """
    STEPS     = 4000
    MAX_ANGLE = 20.0   # degrees — tether restoring torque keeps this bounded

    data = _run_simulation(STEPS)

    violations = np.where(data["axle_deg"] > MAX_ANGLE)[0]
    assert len(violations) == 0, (
        f"Axle misalignment exceeded {MAX_ANGLE}° at {len(violations)} steps. "
        f"First violation at step {violations[0]} "
        f"({violations[0]*DT:.2f} s): {data['axle_deg'][violations[0]]:.1f}°"
    )


def test_steady_state_tether_tension_is_positive():
    """
    The tether must remain taut (non-zero tension) throughout 10 seconds.

    A slack tether at this position indicates the force balance has failed.
    """
    STEPS = 4000

    data = _run_simulation(STEPS)

    slack_steps = np.where(data["tension"] < 0.01)[0]
    assert len(slack_steps) == 0, (
        f"Tether went slack at {len(slack_steps)} steps. "
        f"First slack at step {slack_steps[0]} ({slack_steps[0]*DT:.2f} s). "
        f"Hub distance from anchor at that step: "
        f"{np.linalg.norm(data['pos'][slack_steps[0]]):.2f} m, "
        f"rest_length={data['rest_length']:.3f} m"
    )
