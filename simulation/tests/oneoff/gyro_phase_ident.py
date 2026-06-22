"""gyro phase identification - one-off diagnostic, not a unit test."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SIM = ROOT / "simulation"
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(SIM / "tests" / "simtests"))

from arduloop.swash import SwashH3
from dynbem import RotorInputs, create_aero
from dynamics import RigidBodyDynamics
from frames import build_orb_frame
from rotor_physics import resolve_i_spin_kgm2
from tests.simtests._rotor_helpers import load_default_rotor


DT = 1.0 / 400.0
WIND = np.array([0.0, 10.0, 0.0])
COLLECTIVE = -0.18
OMEGA_SPIN = 28.0


def _initial_R() -> np.ndarray:
    el = math.radians(30.0)
    pos = 100.0 * np.array([0.0, math.cos(el), -math.sin(el)])
    return build_orb_frame(-pos / np.linalg.norm(pos))


def _run_pulse(phase_deg: float, roll_cmd: float, pitch_cmd: float,
               duration_s: float = 0.5) -> np.ndarray:
    rotor = load_default_rotor()
    aero = create_aero(rotor, model="quasi_static")
    state = aero.initial_rotor_state()
    R0 = _initial_R()
    dyn = RigidBodyDynamics(
        mass=float(rotor.inertia.mass_kg),
        I_body=list(rotor.inertia.I_body_kgm2),
        I_spin=resolve_i_spin_kgm2(rotor),
        pos0=[0.0, 86.6025403784, -50.0],
        vel0=[0.0, 0.0, 0.0],
        R0=R0,
        omega0=[0.0, 0.0, 0.0],
    )
    swash = SwashH3(phase_deg)
    roll_out, pitch_out = swash.mix(roll_cmd, pitch_cmd)
    tilt_lat = roll_out
    tilt_lon = -pitch_out
    gravity_cancel = np.array([0.0, 0.0, -float(rotor.inertia.mass_kg) * 9.81])
    omega_samples = []
    for step in range(int(round(duration_s / DT))):
        hub = dyn.state
        inputs = RotorInputs(
            collective_rad=COLLECTIVE,
            tilt_lon=tilt_lon,
            tilt_lat=tilt_lat,
            R_hub=hub["R"],
            v_hub_world=np.zeros(3),
            wind_world=WIND,
            omega_rad_s=OMEGA_SPIN,
            t=45.0 + step * DT,
            rho_kg_m3=1.225,
        )
        result, deriv = aero.compute_forces(inputs, state)
        state = state.from_array(state.to_array() + DT * deriv.to_array())
        dyn.step(gravity_cancel + result.F_world, result.M_orbital, DT,
                 omega_spin=OMEGA_SPIN)
        s = dyn.state
        omega_samples.append(s["R"].T @ s["omega"])
    return np.mean(np.asarray(omega_samples[-40:]), axis=0)


def _angle_deg(v: np.ndarray) -> float:
    return math.degrees(math.atan2(float(v[1]), float(v[0])))


def _angle_err_deg(actual: float, target: float) -> float:
    return abs(((actual - target + 180.0) % 360.0) - 180.0)


def main() -> None:
    print("phase roll_dwx roll_dwy roll_ang pitch_dwx pitch_dwy pitch_ang score")
    rows = []
    for phase in range(0, 360, 15):
        base = _run_pulse(phase, roll_cmd=0.0, pitch_cmd=0.0)
        roll = _run_pulse(phase, roll_cmd=0.10, pitch_cmd=0.0) - base
        pitch = _run_pulse(phase, roll_cmd=0.0, pitch_cmd=0.10) - base
        roll_ang = _angle_deg(roll)
        pitch_ang = _angle_deg(pitch)
        # Desired body-rate directions from ArduPilot setpoints:
        # +roll command -> +omega_x; +pitch command -> +omega_y.
        score = _angle_err_deg(roll_ang, 0.0) + _angle_err_deg(pitch_ang, 90.0)
        rows.append((score, phase, roll, roll_ang, pitch, pitch_ang))
    rows.sort()
    for score, phase, roll, roll_ang, pitch, pitch_ang in rows:
        print(f"{phase:5.1f} {roll[0]:+7.3f} {roll[1]:+7.3f} {roll_ang:+8.1f} "
              f"{pitch[0]:+8.3f} {pitch[1]:+8.3f} {pitch_ang:+9.1f} {score:7.1f}")


if __name__ == "__main__":
    main()