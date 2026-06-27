"""
Find the collective that achieves force balance at IC position.
"""
import json
import sys
import math
import numpy as np
sys.path.insert(0, 'simulation')

from dynbem import create_aero, RotorInputs, relax_inflow
from tests.simtests._rotor_helpers import load_default_rotor
from tether import TetherModel

# Constants from test_generate_ic
L_TETHER = 100.0
MASS = 5.0
G = 9.81
IC_TARGET_TENSION_N = 300
WIND = np.array([0.0, 10.0, 0.0])
STACK_COLL = -0.18

# Load rotor
rotor = load_default_rotor()

# Load current IC
with open('simulation/steady_state_starting.json') as f:
    ic = json.load(f)

pos = np.array(ic['pos'], dtype=float)
R0 = np.array(ic['R0'], dtype=float).reshape(3, 3)
omega_spin = ic['omega_spin']
rest_length = ic['rest_length']

print("="*80)
print("FINDING COLLECTIVE FOR FORCE BALANCE")
print("="*80)
print(f"\nIC Parameters:")
print(f"  Position: {pos}")
print(f"  Body_z: {R0[:, 2]}")
print(f"  Omega spin: {omega_spin:.2f} rad/s")
print(f"  Current collective: {STACK_COLL:.4f} rad")

# Compute tether force (static)
tether_extension = np.linalg.norm(pos) - rest_length
tether_force_mag = TetherModel.EA_N * tether_extension / rest_length if tether_extension > 0 else 0.0
tether_direction = -pos / np.linalg.norm(pos)  # toward anchor
tether_force_ned = tether_force_mag * tether_direction

gravity_force = np.array([0, 0, MASS * G])

print(f"\nFixed forces:")
print(f"  Gravity: {gravity_force}")
print(f"  Tether: {tether_force_ned}")

# Create aero model
aero = create_aero(rotor, model="quasi_static")
state = aero.initial_rotor_state()

# Settle inflow
input0 = RotorInputs(
    collective_rad=STACK_COLL, tilt_lon=0.0, tilt_lat=0.0,
    R_hub=R0, v_hub_world=np.zeros(3), wind_world=WIND,
    omega_rad_s=omega_spin, t=0.0, rho_kg_m3=1.225,
)
state = relax_inflow(aero, state, input0, n_steps=400, dt=1.0/400.0)

print(f"\nSearching for balanced collective...")

# Binary search for collective that balances vertical forces
# For force balance, we need: aero_fz + gravity_z + tether_fz ≈ 0

def get_net_z_force(coll_rad):
    """Compute net vertical force at given collective."""
    input_aero = RotorInputs(
        collective_rad=coll_rad, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R0, v_hub_world=np.zeros(3), wind_world=WIND,
        omega_rad_s=omega_spin, t=0.0, rho_kg_m3=1.225,
    )
    result, _ = aero.compute_forces(input_aero, state)
    aero_force = result.F_world
    # Thrust magnitude is the component along the body_z direction
    thrust_mag = float(np.dot(aero_force, R0[:, 2]))
    net_z = aero_force[2] + gravity_force[2] + tether_force_ned[2]
    return net_z, thrust_mag

# Search range
coll_min = -0.28
coll_max = 0.10

# Test endpoints
fz_min, t_min = get_net_z_force(coll_min)
fz_max, t_max = get_net_z_force(coll_max)

print(f"\nForce range:")
print(f"  At coll={coll_min:.4f}: Fz={fz_min:+.1f} N, Thrust={t_min:.1f} N")
print(f"  At coll={coll_max:.4f}: Fz={fz_max:+.1f} N, Thrust={t_max:.1f} N")

# Binary search
for iteration in range(20):
    coll_mid = (coll_min + coll_max) / 2
    fz_mid, t_mid = get_net_z_force(coll_mid)
    
    if abs(fz_mid) < 0.1:
        print(f"\n✓ Found balanced collective!")
        coll_balanced = coll_mid
        break
    elif fz_mid > 0:  # Z force is upward, need less collective
        coll_max = coll_mid
        fz_max = fz_mid
    else:  # Z force is downward, need more collective
        coll_min = coll_mid
        fz_min = fz_mid
        
    if iteration % 3 == 0:
        print(f"  Iteration {iteration}: coll={coll_mid:.4f} → Fz={fz_mid:+.1f} N")

print(f"\nResult:")
print(f"  Balanced collective: {coll_balanced:.4f} rad")
fz_check, t_check = get_net_z_force(coll_balanced)
print(f"  Net Z force: {fz_check:+.1f} N (target ≈ 0)")
print(f"  Aero thrust: {t_check:.1f} N")

# Check if this is feasible
if coll_min <= coll_balanced <= coll_max:
    print(f"  ✓ Within control range [{coll_min:.4f}, {coll_max:.4f}]")
else:
    print(f"  ✗ Outside control range!")

# Compute required change
delta_coll = coll_balanced - STACK_COLL
print(f"\nRequired change from current IC:")
print(f"  Current: {STACK_COLL:.4f} rad")
print(f"  Needed:  {coll_balanced:.4f} rad")
print(f"  Delta:   {delta_coll:+.4f} rad ({delta_coll*180/np.pi:+.2f}°)")

print(f"\nConclusion:")
if abs(fz_check) < 1.0:
    print(f"  The IC collective is TOO NEGATIVE (not enough collective)")
    print(f"  Increase collective by {-delta_coll:.4f} rad to achieve balance")
    print(f"  This will allow the body to hover stably at kinematic exit")
else:
    print(f"  Force balance cannot be achieved - system may be inherently unstable")
