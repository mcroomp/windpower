"""phase_sweep.py — sweep swashplate_phase_deg, log open-loop body-rate response.

One-off diagnostic, not a unit test.  Used while investigating the
gyroscopic-precession phase calibration for the new Øye aero.  Each row
prints the body-rate vector after 0.5 s of a constant +0.1 cyclic input
at the given swashplate_phase_deg.

Run:
    .venv/Scripts/python.exe simulation/tests/oneoff/phase_sweep.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from dataclasses import replace

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from dynbem import OyeBEMModel, RotorInputs, rotor_definition as rd
from dynamics import RigidBodyDynamics


def _settle_state(aero, omega_spin, R_hub, n_steps=50, dt=0.02):
    state = aero.initial_rotor_state()
    state.omega_rad_s = omega_spin
    inp = RotorInputs(
        collective_rad=-0.05, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R_hub, v_hub_world=np.zeros(3), wind_world=np.zeros(3), t=10.0,
    )
    for _ in range(n_steps):
        _, deriv = aero.compute_forces(inp, state)
        state = state.from_array(state.to_array() + dt * deriv.to_array())
        state.omega_rad_s = omega_spin
    return state


def probe(phase_deg: float, tlon: float, tlat: float, *,
          omega_spin: float = 28.0, t_total: float = 0.5, dt: float = 2.5e-3,
          mass_kg: float = 5.0, I_body=(5.0, 5.0, 10.0), I_spin: float = 4.0,
          rotor_yaml: str = "simulation/rotor_definitions/beaupoil_2026.yaml"):
    defn = rd.load(rotor_yaml)
    ctrl = replace(defn.control, swashplate_phase_deg=phase_deg)
    defn = replace(defn, control=ctrl)
    aero = OyeBEMModel(defn=defn)
    R_hub = np.eye(3)
    state = _settle_state(aero, omega_spin, R_hub)
    dyn = RigidBodyDynamics(
        mass=mass_kg, I_body=list(I_body), I_spin=I_spin,
        pos0=[0, 0, -50], vel0=[0, 0, 0], R0=R_hub.copy(),
        omega0=[0, 0, 0],
    )
    F_g = np.array([0.0, 0.0, -mass_kg * 9.81])
    for _ in range(int(t_total / dt)):
        s = dyn.state
        inp = RotorInputs(
            collective_rad=-0.05, tilt_lon=tlon, tilt_lat=tlat,
            R_hub=s["R"], v_hub_world=s["vel"], wind_world=np.zeros(3), t=10.0,
        )
        res, deriv = aero.compute_forces(inp, state)
        state = state.from_array(state.to_array() + dt * deriv.to_array())
        state.omega_rad_s = omega_spin
        dyn.step(F_g + res.F_world, res.M_orbital, dt, omega_spin=omega_spin)
    s = dyn.state
    return s["R"].T @ s["omega"]


if __name__ == "__main__":
    print("phase | tilt_lat=+0.1 (ang)        | tilt_lon=+0.1 (ang)")
    for phase in range(0, 360, 30):
        o_lat = probe(phase, 0.0, 0.1)
        o_lon = probe(phase, 0.1, 0.0)
        ang_lat = math.degrees(math.atan2(o_lat[1], o_lat[0]))
        ang_lon = math.degrees(math.atan2(o_lon[1], o_lon[0]))
        print(f" {phase:3d}  | ({o_lat[0]:+5.2f},{o_lat[1]:+5.2f}) ang={ang_lat:+6.1f} | "
              f"({o_lon[0]:+5.2f},{o_lon[1]:+5.2f}) ang={ang_lon:+6.1f}")
