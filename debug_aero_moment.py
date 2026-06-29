import json
from pathlib import Path
import numpy as np
from dynbem import RotorInputs, create_aero, rotor_definition

SIM = Path("simulation")
IC_PATH = SIM / "steady_state_starting.json"
ROTOR_PATH = SIM / "rotor_definitions" / "beaupoil_2026.yaml"

data = json.loads(IC_PATH.read_text())
ic = type('IC', (), {
    'R0': np.array(data["R0"], dtype=float).reshape(3, 3),
    'vel': np.array(data["vel"], dtype=float),
    'omega_spin': float(data["omega_spin"]),
})()

rotor = rotor_definition.load(str(ROTOR_PATH))
aero = create_aero(rotor, model="quasi_static")
state = aero.initial_rotor_state()

def test_moment(tilt_lat):
    inputs = RotorInputs(
        collective_rad=-0.18,
        tilt_lon=0.0,
        tilt_lat=tilt_lat,
        R_hub=ic.R0,
        v_hub_world=ic.vel,
        wind_world=np.array([0.0, 10.0, 0.0], dtype=float),
        omega_rad_s=ic.omega_spin,
        t=0.0,
        rho_kg_m3=1.225,
    )
    result, _ = aero.compute_forces(inputs, state)
    M_ned = result.M_orbital
    M_body = ic.R0.T @ M_ned
    return M_ned, M_body

print("Testing tilt_lat = -0.05:")
M_ned, M_body = test_moment(-0.05)
print(f"  M_orbital (NED): [{M_ned[0]:8.2f}, {M_ned[1]:8.2f}, {M_ned[2]:8.2f}]")
print(f"  M_body:         [{M_body[0]:8.2f}, {M_body[1]:8.2f}, {M_body[2]:8.2f}]")

print("Testing tilt_lat = +0.05:")
M_ned, M_body = test_moment(+0.05)
print(f"  M_orbital (NED): [{M_ned[0]:8.2f}, {M_ned[1]:8.2f}, {M_ned[2]:8.2f}]")
print(f"  M_body:         [{M_body[0]:8.2f}, {M_body[1]:8.2f}, {M_body[2]:8.2f}]")

print("\nDerivative dm_dlat[0] (body roll moment vs tilt_lat):")
eps = 0.04
m_pos = test_moment(+eps)[1]
m_neg = test_moment(-eps)[1]
dm_dlat_0 = (m_pos[0] - m_neg[0]) / (2 * eps)
print(f"  dm_dlat[0] = {dm_dlat_0:.2f} (should be > 100 for correct sign)")
