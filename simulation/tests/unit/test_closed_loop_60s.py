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

from dynamics   import RigidBodyDynamics
from aero       import RotorAero
from tether     import TetherModel
from controller import compute_swashplate_from_state
from frames     import build_orb_frame

DT            = 1.0 / 400.0
T_SIM         = 60.0
T_AERO_OFFSET = 45.0   # 45 s kinematic phase → 5 s aero ramp already done
ANCHOR        = np.zeros(3)
POS0          = np.array([46.258, 14.241, 12.530])
VEL0          = np.array([-0.257,  0.916, -0.093])
BODY_Z0       = np.array([0.851018, 0.305391, 0.427206])
OMEGA_SPIN0   = 20.148

K_DRIVE_SPIN   = 1.4
K_DRAG_SPIN    = 0.01786
I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

WIND = np.array([10.0, 0.0, 0.0])


def _orbit_tracked_body_z_eq(cur_pos, tether_dir0, body_z_eq0):
    """
    Rotate body_z_eq0 by the azimuthal angle the tether direction has rotated
    from its initial value.  Gives body_z_eq = body_z_eq0 at t=0 (zero error
    at equilibrium) and smoothly tracks the orbit thereafter.
    """
    cur_tdir = cur_pos / np.linalg.norm(cur_pos)
    th0h = np.array([tether_dir0[0], tether_dir0[1], 0.0])
    thh  = np.array([cur_tdir[0],    cur_tdir[1],    0.0])
    n0h = np.linalg.norm(th0h); nhh = np.linalg.norm(thh)
    if n0h < 0.01 or nhh < 0.01:
        return body_z_eq0
    th0h /= n0h; thh /= nhh
    cos_phi = float(np.clip(np.dot(th0h, thh), -1.0, 1.0))
    sin_phi = float(th0h[0] * thh[1] - th0h[1] * thh[0])
    bz0 = body_z_eq0
    result = np.array([
        cos_phi * bz0[0] - sin_phi * bz0[1],
        sin_phi * bz0[0] + cos_phi * bz0[1],
        bz0[2],
    ])
    return result / np.linalg.norm(result)


def _run(t_sim: float = T_SIM):
    dyn = RigidBodyDynamics(
        mass=5.0, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS0.tolist(), vel0=VEL0.tolist(),
        R0=build_orb_frame(BODY_Z0), omega0=[0.0, 0.0, 0.0], z_floor=1.0,
    )
    aero   = RotorAero()
    tether = TetherModel(anchor_enu=ANCHOR, rest_length=49.949,
                         axle_attachment_length=0.0)   # match mediator

    hub_state  = dyn.state
    omega_spin = OMEGA_SPIN0
    history    = []
    floor_hits = 0

    # Orbit-tracking reference (captured from initial state = BODY_Z0 at POS0)
    tether_dir0 = POS0 / np.linalg.norm(POS0)
    body_z_eq0  = BODY_Z0.copy()

    for i in range(int(t_sim / DT)):
        t = i * DT

        # Orbit-tracked equilibrium body_z: zero error at t=0
        body_z_eq_cur = _orbit_tracked_body_z_eq(
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

        Q_spin = K_DRIVE_SPIN * aero.last_v_inplane - K_DRAG_SPIN * omega_spin ** 2
        omega_spin = max(OMEGA_SPIN_MIN, omega_spin + Q_spin / I_SPIN_KGMS2 * DT)

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

    history.append({
        "t":          t_sim,
        "pos":        hub_state["pos"].copy(),
        "omega_spin": omega_spin,
        "floor_hits": floor_hits,
    })
    return history


def test_60s_altitude_maintained():
    """Hub must stay above z_floor=1 m for the full 60 s."""
    history = _run()
    for snap in history:
        print(f"  t={snap['t']:5.1f}s  z={snap['pos'][2]:.2f}m  "
              f"spin={snap['omega_spin']:.1f}  floor_hits={snap['floor_hits']}")
    floor_hits = history[-1]["floor_hits"]
    assert floor_hits == 0, f"Hub hit z_floor {floor_hits} times in 60 s"


def test_60s_no_runaway():
    """Hub must not drift more than 200 m from anchor over 60 s."""
    history = _run()
    drifts = [float(np.linalg.norm(s["pos"])) for s in history]
    assert max(drifts) <= 200.0, f"Hub runaway: max drift={max(drifts):.1f} m"


def test_60s_spin_maintained():
    """Rotor must stay above 5 rad/s for full 60 s (autorotation sustained)."""
    history = _run()
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
    body_z_eq   = _orbit_tracked_body_z_eq(hub_state["pos"], tether_dir0, body_z_eq0)
    sw = compute_swashplate_from_state(hub_state, ANCHOR, body_z_eq=body_z_eq)
    assert abs(sw["tilt_lon"]) < 1e-6, f"tilt_lon={sw['tilt_lon']:.6f} at equilibrium (expected 0)"
    assert abs(sw["tilt_lat"]) < 1e-6, f"tilt_lat={sw['tilt_lat']:.6f} at equilibrium (expected 0)"
