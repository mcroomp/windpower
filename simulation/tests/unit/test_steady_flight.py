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

Output artefacts (written alongside this file on every run):
  steady_flight_report.png  — 5-panel diagnostic plot
  steady_flight_telemetry.json — full per-step telemetry for offline analysis
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

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
L_TETHER = 50.0                     # m  tether length at this flight point

# ── Output directory (simulation/logs/) ──────────────────────────────────────
_OUT_DIR = Path(__file__).resolve().parents[2] / "logs"
_OUT_DIR.mkdir(exist_ok=True)


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

    # Collective = TensionPI minimum (what HoldPlanner thrust=0 gives)
    coll_eq = TensionPI.COLL_MIN_RAD   # -0.28 rad

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

    return coll_eq, omega_spin_eq, R0, pos0, rest


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
    coll_eq, omega_spin_eq, R0, pos0, rest = _equilibrium_setup()

    aero   = create_aero(_rd.default())

    # ── Warmup pass with closed-loop tension control ───────────────────────────
    # Use TensionPI to find the natural operational equilibrium altitude.
    # A fixed coll_eq (COL_MIN) gives z≈5.8m which is too low for the pumping
    # cycle.  TensionPI with tension_out=200N finds z≈10-12m (same as RotorAero).
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
    pos       : NED position [m]
    vel       : NED velocity [m/s]
    body_z    : body-Z axis in world NED frame (unit vector; R0 derived from this)
    omega_spin: rotor spin rate [rad/s]
    rest_length: tether rest length [m]
    """
    R0 = data["R0"]
    body_z = R0[:, 2].tolist()   # third column = body Z in world frame
    out = {
        "pos":          data["pos0"].tolist(),
        "vel":          data["vel0"].tolist(),
        "body_z":       body_z,
        "omega_spin":   float(data["omega_spin_eq"]),
        "rest_length":  float(data["rest_length"]),
        "coll_eq_rad":  float(data["coll_eq"]),
        "home_z_ned":   0.0,   # GPS home at ground level
    }
    path.write_text(json.dumps(out, indent=2))


def _save_json(data: dict, path: Path) -> None:
    out = {
        "coll_deg":    data["coll_deg"],
        "rest_length": data["rest_length"],
        "pos0":        data["pos0"].tolist(),
        "omega_eq_rad_s": float(data["omega_spin_eq"]),
        "steps": [
            {
                "t":            float(data["t"][i]),
                "pos":          data["pos"][i].tolist(),
                "vel":          data["vel"][i].tolist(),
                "tension_N":    float(data["tension"][i]),
                "axle_deg":     float(data["axle_deg"][i]),
                "body_z":       data["body_z"][i].tolist(),
                "omega_spin":   float(data["omega_spin"][i]),
            }
            for i in range(0, len(data["t"]), 40)   # every 40 steps = every 0.1 s
        ],
    }
    path.write_text(json.dumps(out, indent=2))


def _save_plot(data: dict, path: Path) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        return   # matplotlib optional; skip silently

    t    = data["t"]
    pos  = data["pos"]
    vel  = data["vel"]
    ten  = data["tension"]
    ang  = data["axle_deg"]
    spin = data["omega_spin"]
    pos0 = data["pos0"]
    coll = data["coll_deg"]

    omega_eq = data["omega_spin_eq"]

    fig = plt.figure(figsize=(14, 16))
    fig.suptitle(
        f"Steady-state flight at 30° tether elevation  |  collective = {coll:.1f}°  |  10 m/s East wind",
        fontsize=13, fontweight="bold",
    )
    gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.50, wspace=0.35,
                           height_ratios=[1.0, 1.0, 1.0, 1.4])

    # ── Hub position ─────────────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, pos[:, 0], label="East (X)", color="steelblue")
    ax1.plot(t, pos[:, 2], label="Altitude (Z)", color="tomato")
    ax1.axhline(pos0[0], color="steelblue", ls="--", lw=0.8, label=f"Start East={pos0[0]:.1f} m")
    ax1.axhline(pos0[2], color="tomato",    ls="--", lw=0.8, label=f"Start Alt={pos0[2]:.1f} m")
    ax1.set_xlabel("Time (s)"); ax1.set_ylabel("Position (m)")
    ax1.set_title("Hub position — ENU"); ax1.legend(fontsize=8, ncol=2); ax1.grid(True, alpha=0.3)

    # ── Hub velocity ──────────────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t, vel[:, 0], label="East vel", color="steelblue")
    ax2.plot(t, vel[:, 2], label="Up vel",   color="tomato")
    ax2.axhline(0, color="k", lw=0.5)
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Velocity (m/s)")
    ax2.set_title("Hub velocity"); ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

    # ── Rotor spin ────────────────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(t, spin, color="darkgreen", label="ω_spin")
    ax3.axhline(omega_eq, color="green", ls="--", lw=0.8,
                label=f"ω_eq = {omega_eq:.1f} rad/s (start)")
    ax3_rpm = ax3.twinx()
    ax3_rpm.plot(t, spin * 60 / (2 * math.pi), alpha=0.0)
    ax3_rpm.set_ylabel("RPM", color="darkgreen", fontsize=8)
    ax3_rpm.tick_params(axis="y", labelcolor="darkgreen", labelsize=7)
    ax3.set_xlabel("Time (s)"); ax3.set_ylabel("ω (rad/s)")
    ax3.set_title("Rotor spin rate"); ax3.legend(fontsize=8); ax3.grid(True, alpha=0.3)

    # ── Tether tension ────────────────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[2, 0])
    ax4.plot(t, ten, color="darkorange")
    ax4.axhline(TetherModel.BREAK_LOAD_N * 0.8, color="red", ls="--", lw=0.8, label="80% break load")
    ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Tension (N)")
    ax4.set_title("Tether tension"); ax4.legend(fontsize=8); ax4.grid(True, alpha=0.3)

    # ── Axle misalignment ─────────────────────────────────────────────────────
    ax5 = fig.add_subplot(gs[2, 1])
    ax5.plot(t, ang, color="purple")
    ax5.axhline(20.0, color="red", ls="--", lw=0.8, label="20° limit")
    ax5.set_xlabel("Time (s)"); ax5.set_ylabel("Misalignment (°)")
    ax5.set_title("Axle vs tether direction"); ax5.legend(fontsize=8); ax5.grid(True, alpha=0.3)

    # ── Side-view geometry: anchor, tether, hub ──────────────────────────────
    # Use slant range (distance along ground projected horizontally) so that
    # North drift is represented correctly: horiz = sqrt(E²+N²), vert = Z.
    ax6 = fig.add_subplot(gs[3, :])

    pos_start = pos0
    pos_final = pos[-1]
    anchor    = np.zeros(3)

    def _horiz(p):
        return math.sqrt(p[0]**2 + p[1]**2)   # horizontal distance from anchor

    horiz_traj = np.sqrt(pos[:, 0]**2 + pos[:, 1]**2)

    h_start = _horiz(pos_start)
    h_final = _horiz(pos_final)
    tlen_start = np.linalg.norm(pos_start)
    tlen_final = np.linalg.norm(pos_final)

    elev_start = math.degrees(math.atan2(pos_start[2], h_start))
    elev_final = math.degrees(math.atan2(pos_final[2], h_final))
    north_start = pos_start[1]
    north_final = pos_final[1]

    # Tether trajectory (slant view)
    ax6.plot(horiz_traj, pos[:, 2], color="teal", lw=1.5, label="Hub trajectory", zorder=3)

    # Tether lines anchor → hub
    ax6.plot([0, h_start], [0, pos_start[2]],
             color="green", lw=2.5, ls="-", zorder=2,
             label=f"Tether start  L={tlen_start:.1f} m  elev={elev_start:.1f}°  N={north_start:.1f} m")
    ax6.plot([0, h_final], [0, pos_final[2]],
             color="firebrick", lw=2.5, ls="--", zorder=2,
             label=f"Tether end    L={tlen_final:.1f} m  elev={elev_final:.1f}°  N={north_final:.1f} m")

    # Hub markers
    ax6.plot(h_start, pos_start[2], "o", color="green",    ms=10, zorder=5, label="Hub start")
    ax6.plot(h_final, pos_final[2], "s", color="firebrick", ms=10, zorder=5, label="Hub end")
    ax6.plot(0, 0, "k^", ms=12, zorder=5, label="Anchor")

    # Elevation arcs
    for h, z, colour in [(h_start, pos_start[2], "green"), (h_final, pos_final[2], "firebrick")]:
        tl = math.sqrt(h**2 + z**2)
        elev = math.degrees(math.atan2(z, h))
        arc_r = tl * 0.22
        thetas = np.linspace(0, math.radians(elev), 40)
        ax6.plot(arc_r * np.cos(thetas), arc_r * np.sin(thetas), color=colour, lw=1.0, ls=":")
        mid = math.radians(elev / 2)
        ax6.text(arc_r * 1.08 * math.cos(mid), arc_r * 1.08 * math.sin(mid),
                 f"{elev:.1f}°", fontsize=8, color=colour, va="center")

    # Ground
    x_max = max(h_start, h_final) * 1.15
    ax6.axhline(0, color="saddlebrown", lw=1.0, alpha=0.5)
    ax6.fill_between([0, x_max], [-x_max * 0.03, -x_max * 0.03], [0, 0],
                     color="saddlebrown", alpha=0.15)

    # Wind arrow
    ax6.annotate("", xy=(x_max * 0.15, pos_start[2] * 0.45),
                 xytext=(0.0, pos_start[2] * 0.45),
                 arrowprops=dict(arrowstyle="->", color="royalblue", lw=1.5))
    ax6.text(x_max * 0.075, pos_start[2] * 0.52,
             "10 m/s wind", fontsize=8, color="royalblue", ha="center")

    ax6.set_xlim(-x_max * 0.03, x_max)
    ax6.set_ylim(-x_max * 0.04, max(pos_start[2], pos_final[2]) * 1.35)
    ax6.set_xlabel("Horizontal distance from anchor  √(E²+N²)  (m)")
    ax6.set_ylabel("Altitude (m)")
    ax6.set_title("Side view — tether geometry  (slant: horizontal distance vs altitude; N drift shown in legend)")
    ax6.set_aspect("equal")
    ax6.legend(fontsize=8, ncol=2, loc="upper left")
    ax6.grid(True, alpha=0.25)

    fig.savefig(str(path), dpi=130, bbox_inches="tight")
    plt.close(fig)


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_steady_state_hub_does_not_drift():
    """
    Starting exactly at the 30° tether equilibrium, the hub must not drift more
    than 15 m in 10 seconds with a fixed collective and no ArduPilot.

    The 15 m bound allows for the ~1° scan resolution of the equilibrium
    collective while still catching any catastrophic divergence.

    Artefacts written to the test directory:
      steady_flight_report.png   — diagnostic plots
      steady_flight_telemetry.json — per-step telemetry (every 0.1 s)
    """
    STEPS  = 4000    # 10 s at 400 Hz
    BOUNDS = 15.0    # m — catches instability; not sub-metre accuracy

    data = _run_simulation(STEPS)

    pos0  = data["pos0"]
    final = data["pos"][-1]
    drift = np.abs(final - pos0)

    # Save artefacts regardless of pass/fail
    _save_json(data, _OUT_DIR / "steady_flight_telemetry.json")
    _save_plot(data, _OUT_DIR / "steady_flight_report.png")
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
