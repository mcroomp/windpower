"""
test_closed_loop_60s.py — 60-second closed-loop simulation, no ArduPilot.

Mirrors what the stack test requires: 60 seconds of free flight from the
steady-state equilibrium initial conditions, using the orbit-tracking
internal controller at 400 Hz.  Fails if the hub crashes (z <= 1 m) or
spin collapses.

Key design decisions replicated from mediator.py:
  - axle_attachment_length=0.0: tether restoring torque disabled (body_z
    stability comes from aerodynamics, not tether moment)
  - aero time offset by 45 s: matches the 45 s kinematic phase so the
    5 s aero startup ramp is already done at free-flight t=0
  - Orbit-tracking body_z_eq: the controller references the aerodynamic
    equilibrium body_z (BODY_Z0) rotated azimuthally to track the orbit,
    not the geometric tether direction.  At t=0 this gives zero error and
    zero tilt, which is the correct output at equilibrium.

This is the unit-level equivalent of test_acro_hold without SITL/ArduPilot.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(120)]

import rotor_definition as rd
from dynamics   import RigidBodyDynamics
from aero       import create_aero
from tether     import TetherModel
from controller import orbit_tracked_body_z_eq, compute_rate_cmd, RatePID
from planner import HoldPlanner
from frames     import build_orb_frame
from simtest_log import SimtestLog
from simtest_ic  import load_ic

_log = SimtestLog(__file__)
_IC  = load_ic()

DT            = 1.0 / 400.0
T_SIM         = 60.0
T_AERO_OFFSET = 45.0   # 45 s kinematic phase → 5 s aero ramp already done
ANCHOR        = np.zeros(3)
POS0          = _IC.pos
VEL0          = _IC.vel
BODY_Z0       = _IC.body_z
OMEGA_SPIN0   = _IC.omega_spin

I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

# Two-loop controller gains (see controller.py RatePID docstring for derivation).
# Outer loop: compute_rate_cmd converts body_z error (rad) → rate setpoint (rad/s).
# Inner loop: RatePID converts rate error (rad/s) → normalised swashplate tilt [-1,1].
KP_OUTER = 2.5   # outer P gain [rad/s per rad]  — matches hardware compute_rate_cmd call
KP_INNER = RatePID.DEFAULT_KP  # inner P gain — calibrated to legacy kd=0.2 behaviour

WIND = np.array([10.0, 0.0, 0.0])



def _run(t_sim: float = T_SIM):
    dyn = RigidBodyDynamics(
        mass=5.0, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS0.tolist(), vel0=VEL0.tolist(),
        R0=build_orb_frame(BODY_Z0), omega0=[0.0, 0.0, 0.0], z_floor=1.0,
    )
    aero   = create_aero(rd.default())
    tether = TetherModel(anchor_enu=ANCHOR,
                         rest_length=_IC.rest_length,
                         axle_attachment_length=0.0)   # match mediator

    hub_state  = dyn.state
    omega_spin = OMEGA_SPIN0
    history    = []
    floor_hits = 0
    telemetry  = []   # 20 Hz rich frames for 3D visualizer
    tel_every  = max(1, int(0.05 / DT))   # 20 Hz

    # Trajectory planner (offboard) — HoldTrajectory, no winch, no tilt correction
    trajectory      = HoldPlanner()
    # Mode_RAWES orbit tracking ICs — captured at free-flight start
    ic_tether_dir0  = POS0 / np.linalg.norm(POS0)
    ic_body_z_eq0   = BODY_Z0.copy()

    # Simulated ACRO rate PIDs (one per swashplate axis).
    # kp_inner calibrated to match legacy kd=0.2 damping (see RatePID docstring).
    pid_lon = RatePID(kp=KP_INNER)   # lon axis: body_x error → tilt_lon
    pid_lat = RatePID(kp=KP_INNER)   # lat axis: body_y error → tilt_lat

    for i in range(int(t_sim / DT)):
        t = i * DT

        # STATE packet (Pixhawk → planner)
        state_pkt = {
            "pos_enu":    hub_state["pos"],
            "vel_enu":    hub_state["vel"],
            "tension_n":  0.0,
            "omega_spin": omega_spin,
            "t_free":     t,
        }
        trajectory.step(state_pkt, DT)
        # cmd["attitude_q"] == identity → Mode_RAWES stays at tether-aligned natural equilibrium

        # Mode_RAWES outer loop: orbit tracking → body_z_eq → rate setpoint
        body_z_eq = orbit_tracked_body_z_eq(hub_state["pos"], ic_tether_dir0, ic_body_z_eq0)
        bz_now    = hub_state["R"][:, 2]
        rate_sp   = compute_rate_cmd(bz_now, body_z_eq, hub_state["R"],
                                     kp=KP_OUTER, kd=0.0)

        # Mode_RAWES inner loop: rate PID → normalised swashplate tilt
        # Strip spin (body_z in body frame = [0,0,1]) before feeding to PIDs.
        omega_body          = hub_state["R"].T @ hub_state["omega"]
        omega_body_orbital  = omega_body.copy()
        omega_body_orbital[2] = 0.0
        tilt_lon =  pid_lon.update(rate_sp[0],  omega_body_orbital[0], DT)
        tilt_lat = -pid_lat.update(rate_sp[1],  omega_body_orbital[1], DT)

        result = aero.compute_forces(
            collective_rad=_IC.coll_eq_rad,
            tilt_lon=tilt_lon,
            tilt_lat=tilt_lat,
            R_hub=hub_state["R"],
            v_hub_world=hub_state["vel"],
            omega_rotor=omega_spin,
            wind_world=WIND,
            t=T_AERO_OFFSET + t,
        )

        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        F_net     = result.F_world + tf
        M_orbital = result.M_orbital + tm
        tension_now = tether._last_info.get("tension", 0.0)

        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)

        hub_state = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)

        if hub_state["pos"][2] <= 1.05:
            floor_hits += 1

        if i % (int(10.0 / DT)) == 0:
            history.append({
                "t":          t,
                "pos":        hub_state["pos"].copy(),
                "omega_spin": omega_spin,
                "floor_hits": floor_hits,
            })

        if i % tel_every == 0:
            telemetry.append({
                "t":                   t,
                "pos_enu":             hub_state["pos"].tolist(),
                "R":                   hub_state["R"].tolist(),
                "omega_spin":          omega_spin,
                "tether_tension":      tension_now,
                "tether_rest_length":  tether.rest_length,
                "swash_collective":    _IC.coll_eq_rad,
                "swash_tilt_lon":      tilt_lon,
                "swash_tilt_lat":      tilt_lat,
                "body_z_eq":           body_z_eq.tolist(),
                "wind_enu":            WIND.tolist(),
            })

    history.append({
        "t":          t_sim,
        "pos":        hub_state["pos"].copy(),
        "omega_spin": omega_spin,
        "floor_hits": floor_hits,
    })
    return history, telemetry


def test_60s_altitude_maintained():
    """Hub must stay above z_floor=1 m for the full 60 s."""
    history, _ = _run()
    lines = ["t(s)      z(m)  spin(rad/s)  floor_hits"]
    for snap in history:
        lines.append(f"  {snap['t']:5.1f}  {snap['pos'][2]:7.2f}  "
                     f"{snap['omega_spin']:10.1f}  {snap['floor_hits']}")
    floor_hits = history[-1]["floor_hits"]
    min_z = min(s["pos"][2] for s in history)
    _log.write(lines, f"floor_hits={floor_hits}  min_z={min_z:.2f}m")
    assert floor_hits == 0, f"Hub hit z_floor {floor_hits} times in 60 s"


def test_60s_no_runaway():
    """Hub must not drift more than 200 m from anchor over 60 s."""
    history, _ = _run()
    drifts = [float(np.linalg.norm(s["pos"])) for s in history]
    assert max(drifts) <= 200.0, f"Hub runaway: max drift={max(drifts):.1f} m"


def test_60s_spin_maintained():
    """Rotor must stay above 5 rad/s for full 60 s (autorotation sustained)."""
    history, _ = _run()
    spins = [s["omega_spin"] for s in history]
    assert min(spins) >= 5.0, f"Spin collapsed: min={min(spins):.2f} rad/s"


def test_zero_tilt_at_equilibrium():
    """
    At the aerodynamic equilibrium (t=0, hub at POS0 with BODY_Z0),
    the orbit-tracking controller must output zero tilt.
    Verifies the controller does not destabilize the equilibrium it's
    supposed to maintain.
    """
    R0 = build_orb_frame(BODY_Z0)
    hub_state = {
        "pos":   POS0.copy(),
        "vel":   VEL0.copy(),
        "R":     R0,
        "omega": np.zeros(3),
    }
    # Mode_RAWES orbit tracking at t=0: body_z_eq = BODY_Z0 (no azimuthal offset)
    ic_tether_dir0 = POS0 / np.linalg.norm(POS0)
    body_z_eq      = orbit_tracked_body_z_eq(POS0, ic_tether_dir0, BODY_Z0)
    bz_now         = R0[:, 2]
    rate_sp        = compute_rate_cmd(bz_now, body_z_eq, R0, kp=KP_OUTER, kd=0.0)
    # At equilibrium bz_now == body_z_eq → rate_sp == 0 → PIDs output 0 tilt
    pid_lon = RatePID(kp=KP_INNER)
    pid_lat = RatePID(kp=KP_INNER)
    tilt_lon =  pid_lon.update(rate_sp[0], 0.0, DT)
    tilt_lat = -pid_lat.update(rate_sp[1], 0.0, DT)
    assert abs(tilt_lon) < 1e-6, f"tilt_lon={tilt_lon:.6f} at equilibrium (expected 0)"
    assert abs(tilt_lat) < 1e-6, f"tilt_lat={tilt_lat:.6f} at equilibrium (expected 0)"


# ── CLI: generate telemetry JSON for 3D visualizer ────────────────────────────
if __name__ == "__main__":
    import argparse as _ap
    p = _ap.ArgumentParser(description="Run 60 s closed-loop sim and save telemetry")
    p.add_argument("--save-telemetry", metavar="PATH",
                   default="telemetry_closed_loop.json",
                   help="Output JSON path (default: telemetry_closed_loop.json)")
    args = p.parse_args()

    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from simulation.viz3d.telemetry import save_telemetry  # type: ignore

    _, tel = _run()
    save_telemetry(args.save_telemetry, tel)
    print(f"\nRun visualizer:")
    print(f"  python simulation/viz3d/visualize_3d.py {args.save_telemetry}")
