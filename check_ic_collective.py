"""
Compute what the equilibrium collective should be for hover.
Then compare to the IC collective to see if there's a mismatch.
"""
import json
import sys
sys.path.insert(0, 'simulation')

from dynbem import create_aero, RotorInputs, RotorState
import numpy as np

# Load IC
with open('simulation/steady_state_starting.json') as f:
    ic = json.load(f)

# Load config to get rotor and aero model
cfg_file = 'simulation/rotor_definitions/beaupoil_2026.yaml'
import yaml
with open(cfg_file) as f:
    rotor_cfg = yaml.safe_load(f)

print("="*80)
print("EQUILIBRIUM COLLECTIVE COMPUTATION")
print("="*80)

ic_coll = ic.get('coll_eq_rad', ic.get('stack_coll_eq'))
ic_omega = ic['omega_spin']
mass_kg = ic['eq_physics']['mass_kg']
g = 9.81
hover_thrust_needed = mass_kg * g

print(f"\nIC parameters:")
print(f"  Collective: {ic_coll:.4f} rad")
print(f"  Rotor RPM: {ic_omega:.2f} rad/s")
print(f"  Mass: {mass_kg} kg")
print(f"  Thrust needed for hover: {hover_thrust_needed:.1f} N")

# Create aero model (same as mediator uses)
print(f"\nCreating aero model from {cfg_file}...")
try:
    aero = create_aero(
        rotor_rpm_min=0.1,
        rotor_rpm_max=150.0,
        dyn_inflow_model="quasi_static",  # default in mediator
        rotor_config=rotor_cfg,
    )
    print("Aero model created successfully")
except Exception as e:
    print(f"ERROR creating aero model: {e}")
    exit(1)

# Compute thrust at IC conditions
# Use level flight (R = identity, v_hub = 0)
R_level = np.eye(3)
v_hub = np.array([0., 0., 0.])
wind = np.array([0., 10., 0.])  # 10 m/s wind as in IC

rotor_state = RotorState()  # Default inflow state
rotor_input = RotorInputs(
    collective_rad=ic_coll,
    tilt_lon=0.0,
    tilt_lat=0.0,
    R_hub=R_level,
    v_hub_world=v_hub,
    wind_world=wind,
    omega_rad_s=ic_omega,
    t=0.0,
    rho_kg_m3=1.225,
)

result, deriv = aero.compute_forces(rotor_input, rotor_state)
print(f"\nAero result at IC conditions (level hover):")
print(f"  Thrust: {result.T:.1f} N")
print(f"  Needed: {hover_thrust_needed:.1f} N")
print(f"  Deficit: {hover_thrust_needed - result.T:.1f} N")

if result.T < hover_thrust_needed - 10:
    print("\n*** WARNING: IC collective produces INSUFFICIENT thrust for hover! ***")
    print("The system will descend even if collective is held at IC value.")
    
    # Try to find what collective is needed
    print("\nSearching for correct collective for hover...")
    for test_coll in np.linspace(-0.3, 0.15, 50):
        rotor_input = RotorInputs(
            collective_rad=test_coll,
            tilt_lon=0.0,
            tilt_lat=0.0,
            R_hub=R_level,
            v_hub_world=v_hub,
            wind_world=wind,
            omega_rad_s=ic_omega,
            t=0.0,
            rho_kg_m3=1.225,
        )
        result, _ = aero.compute_forces(rotor_input, rotor_state)
        if abs(result.T - hover_thrust_needed) < 5:
            print(f"  Collective {test_coll:.4f} rad → Thrust {result.T:.1f} N (target {hover_thrust_needed:.1f})")

elif result.T > hover_thrust_needed + 10:
    print("\n*** WARNING: IC collective produces EXCESS thrust! ***")
    print("The system will climb even if collective is held at IC value.")
else:
    print("\n*** IC collective seems CORRECT for hover! ***")
    print("If the system descends, the problem is elsewhere (rotor state, orientation, etc).")
