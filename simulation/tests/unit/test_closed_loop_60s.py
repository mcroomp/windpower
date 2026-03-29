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

pytestmark = pytest.mark.simtest

import rotor_definition as rd
from dynamics   import RigidBodyDynamics
from aero       import create_aero
from tether     import TetherModel
from controller import compute_swashplate_from_state, orbit_tracked_body_z_eq
from frames     import build_orb_frame

DT            = 1.0 / 400.0
T_SIM         = 60.0
T_AERO_OFFSET = 45.0   # 45 s kinematic phase → 5 s aero ramp already done
ANCHOR        = np.zeros(3)
POS0          = np.array([46.258, 14.241, 12.530])
VEL0          = np.array([-0.257,  0.916, -0.093])
BODY_Z0       = np.array([0.851018, 0.305391, 0.427206])
OMEGA_SPIN0   = 20.148

I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

WIND = np.array([10.0, 0.0, 0.0])



def _run(t_sim: float = T_SIM):
    dyn = RigidBodyDynamics(
        mass=5.0, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS0.tolist(), vel0=VEL0.tolist(),
        R0=build_orb_frame(BODY_Z0), omega0=[0.0, 0.0, 0.0], z_floor=1.0,
    )
    aero   = create_aero(rd.default())
    tether = TetherModel(anchor_enu=ANCHOR, rest_length=49.949,
                         axle_attachment_length=0.0)   # match mediator

    hub_state  = dyn.state
    omega_spin = OMEGA_SPIN0
    history    = []
    floor_hits = 0
    telemetry  = []   # 20 Hz rich frames for 3D visualizer
    tel_every  = max(1, int(0.05 / DT))   # 20 Hz

    # Orbit-tracking reference (captured from initial state = BODY_Z0 at POS0)
    tether_dir0 = POS0 / np.linalg.norm(POS0)
    body_z_eq0  = BODY_Z0.copy()

    for i in range(int(t_sim / DT)):
        t = i * DT

        # Orbit-tracked equilibrium body_z: zero error at t=0
        body_z_eq_cur = orbit_tracked_body_z_eq(
            hub_state["pos"], tether_dir0, body_z_eq0)

        sw = compute_swashplate_from_state(
            hub_state  = hub_state,
            anchor_pos = ANCHOR,
            body_z_eq  = body_z_eq_cur,
        )

        forces = aero.compute_forces(
            collective_rad=sw["collective_rad"],
            tilt_lon=sw["tilt_lon"],
            tilt_lat=sw["tilt_lat"],
            R_hub=hub_state["R"],
            v_hub_world=hub_state["vel"],
            omega_rotor=omega_spin,
            wind_world=WIND,
            t=T_AERO_OFFSET + t,
        )

        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        forces[0:3] += tf
        forces[3:6] += tm
        tension_now = tether._last_info.get("tension", 0.0)

        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)

        M_orbital = forces[3:6] - aero.last_M_spin
        M_orbital += -50.0 * hub_state["omega"]

        hub_state = dyn.step(forces[0:3], M_orbital, DT, omega_spin=omega_spin)

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
                "swash_collective":    sw["collective_rad"],
                "swash_tilt_lon":      sw["tilt_lon"],
                "swash_tilt_lat":      sw["tilt_lat"],
                "body_z_eq":           body_z_eq_cur.tolist(),
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
    for snap in history:
        print(f"  t={snap['t']:5.1f}s  z={snap['pos'][2]:.2f}m  "
              f"spin={snap['omega_spin']:.1f}  floor_hits={snap['floor_hits']}")
    floor_hits = history[-1]["floor_hits"]
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
    tether_dir0 = POS0 / np.linalg.norm(POS0)
    body_z_eq0  = BODY_Z0.copy()
    body_z_eq   = orbit_tracked_body_z_eq(hub_state["pos"], tether_dir0, body_z_eq0)
    sw = compute_swashplate_from_state(hub_state, ANCHOR, body_z_eq=body_z_eq)
    assert abs(sw["tilt_lon"]) < 1e-6, f"tilt_lon={sw['tilt_lon']:.6f} at equilibrium (expected 0)"
    assert abs(sw["tilt_lat"]) < 1e-6, f"tilt_lat={sw['tilt_lat']:.6f} at equilibrium (expected 0)"


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
