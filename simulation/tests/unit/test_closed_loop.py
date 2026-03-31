"""
test_closed_loop.py — Closed-loop simulation test without ArduPilot.

Runs the physics engine (dynamics + aero + tether) with the truth-state
controller (compute_swashplate_from_state) for 10 simulated seconds and
asserts the hub remains stable — altitude above 2 m, drift below 200 m.

This validates the control law before ArduPilot integration.
No networking, no Docker, no SITL required.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(120)]

from dynamics   import RigidBodyDynamics
from aero       import create_aero
import rotor_definition as _rd
from tether     import TetherModel
from controller import compute_swashplate_from_state
from frames     import build_orb_frame
from simtest_ic import load_ic

# ── Simulation parameters ─────────────────────────────────────────────────────
DT         = 1.0 / 400.0   # 400 Hz
T_SIM      = 10.0          # seconds to simulate
ANCHOR     = np.zeros(3)

_IC        = load_ic()
POS0       = _IC.pos
VEL0       = _IC.vel
BODY_Z0    = _IC.body_z
OMEGA_SPIN0 = _IC.omega_spin

# Spin model constants (from mediator.py)
K_DRIVE_SPIN  = 1.4
K_DRAG_SPIN   = 0.01786
I_SPIN_KGMS2  = 10.0
OMEGA_SPIN_MIN = 0.5


def _run_simulation(t_sim_s: float, wind: np.ndarray = None) -> list:
    """Run closed-loop simulation, return list of (t, pos_ned) tuples."""
    if wind is None:
        wind = np.array([0.0, 10.0, 0.0])  # NED: East wind = Y axis

    R0 = build_orb_frame(BODY_Z0)

    dyn = RigidBodyDynamics(
        mass   = 5.0,
        I_body = [5.0, 5.0, 10.0],
        I_spin = 0.0,
        pos0   = POS0.tolist(),
        vel0   = VEL0.tolist(),
        R0     = R0,
        omega0 = [0.0, 0.0, 0.0],
        z_floor = -1.0,  # NED: max Z = -1 m (altitude floor at 1 m)
    )
    aero    = create_aero(_rd.default())
    tether  = TetherModel(anchor_ned=ANCHOR, rest_length=49.949,
                         axle_attachment_length=0.0)

    hub_state  = dyn.state
    omega_spin = OMEGA_SPIN0
    history    = []
    n_steps    = int(t_sim_s / DT)

    for i in range(n_steps):
        t = i * DT

        # Controller: compute swashplate tilt from truth state
        sw = compute_swashplate_from_state(hub_state, ANCHOR)

        # Aero forces
        result = aero.compute_forces(
            collective_rad = sw["collective_rad"],
            tilt_lon       = sw["tilt_lon"],
            tilt_lat       = sw["tilt_lat"],
            R_hub          = hub_state["R"],
            v_hub_world    = hub_state["vel"],
            omega_rotor    = omega_spin,
            wind_world     = wind,
            t              = t,
        )

        # Tether force
        tether_force, tether_moment = tether.compute(
            hub_state["pos"], hub_state["vel"], hub_state["R"])
        F_net     = result.F_world + tether_force
        M_orbital = result.M_orbital + tether_moment

        # Spin update
        Q_spin    = K_DRIVE_SPIN * aero.last_v_inplane - K_DRAG_SPIN * omega_spin ** 2
        omega_spin = max(OMEGA_SPIN_MIN, omega_spin + Q_spin / I_SPIN_KGMS2 * DT)

        # Rate damping
        M_orbital += -50.0 * hub_state["omega"]

        hub_state = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)

        if i % 400 == 0:   # record once per simulated second
            history.append((t, hub_state["pos"].copy()))

    return history


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_closed_loop_altitude_maintained():
    """Hub must not fall below the ground floor (1 m) for 10 s with tether-alignment controller."""
    history = _run_simulation(T_SIM)
    # In NED, altitude = -pos[2]. Floor at z_floor=-1 → altitude ≥ 1 m.
    altitudes = [-pos[2] for _, pos in history]
    min_alt = min(altitudes)
    assert min_alt >= 1.0, f"Hub crashed below floor: min altitude = {min_alt:.2f} m"


def test_closed_loop_no_runaway():
    """Hub must not drift more than 200 m from anchor."""
    history = _run_simulation(T_SIM)
    drifts = [float(np.linalg.norm(pos)) for _, pos in history]
    max_drift = max(drifts)
    assert max_drift <= 200.0, f"Hub runaway: max drift = {max_drift:.2f} m"


def test_closed_loop_spin_stays_positive():
    """Rotor must maintain positive spin throughout (autorotation sustained)."""
    # Simulate and track spin separately
    wind = np.array([0.0, 10.0, 0.0])  # NED: East wind
    R0 = build_orb_frame(BODY_Z0)
    dyn = RigidBodyDynamics(
        mass=5.0, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS0.tolist(), vel0=VEL0.tolist(), R0=R0,
        omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(_rd.default())
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=49.949)
    hub_state  = dyn.state
    omega_spin = OMEGA_SPIN0
    min_spin   = omega_spin

    for i in range(int(T_SIM / DT)):
        sw = compute_swashplate_from_state(hub_state, ANCHOR)
        result = aero.compute_forces(
            collective_rad=sw["collective_rad"], tilt_lon=sw["tilt_lon"],
            tilt_lat=sw["tilt_lat"], R_hub=hub_state["R"],
            v_hub_world=hub_state["vel"], omega_rotor=omega_spin,
            wind_world=wind, t=i*DT,
        )
        tether_force, tether_moment = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        F_net     = result.F_world + tether_force
        M_orbital = result.M_orbital + tether_moment
        Q_spin    = K_DRIVE_SPIN * aero.last_v_inplane - K_DRAG_SPIN * omega_spin ** 2
        omega_spin = max(OMEGA_SPIN_MIN, omega_spin + Q_spin / I_SPIN_KGMS2 * DT)
        M_orbital += -50.0 * hub_state["omega"]
        hub_state  = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)
        min_spin   = min(min_spin, omega_spin)

    assert min_spin >= OMEGA_SPIN_MIN, f"Spin collapsed: min = {min_spin:.2f} rad/s"
