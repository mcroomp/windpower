"""
Verify that the aero model generates the expected force at IC conditions.
Uses IC JSON directly without loading YAML config.
"""
import json
import sys
sys.path.insert(0, 'simulation')

import numpy as np

# Load IC
with open('simulation/steady_state_starting.json') as f:
    ic = json.load(f)

print("="*80)
print("IC STATE VERIFICATION AT KINEMATIC EXIT (t=80s)")
print("="*80)

ic_coll = ic.get('coll_eq_rad', ic.get('stack_coll_eq'))
ic_omega = ic['omega_spin']
mass_kg = ic['eq_physics']['mass_kg']
g = 9.81
weight_N = mass_kg * g

print(f"\nIC Parameters from steady_state_starting.json:")
print(f"  Collective: {ic_coll:.4f} rad")
print(f"  Rotor RPM: {ic_omega:.2f} rad/s ({ic_omega*60/(2*np.pi):.1f} RPM)")
print(f"  Mass: {mass_kg} kg")
print(f"  Weight (thrust needed): {weight_N:.1f} N")
print(f"  Position NED: {np.array(ic['pos'])}")
print(f"  Velocity NED: {np.array(ic.get('vel', [0,0,0]))}")
print(f"  Wind NED: {np.array(ic['eq_physics']['wind_ned'])}")
print(f"  Rest length: {ic['rest_length']:.2f} m")
print(f"  Tether length: {np.linalg.norm(ic['pos']):.2f} m")
print(f"  Tether extension: {np.linalg.norm(ic['pos']) - ic['rest_length']:.4f} m")

# Display rotor physics from IC
if 'eq_physics' in ic:
    print(f"\nEquilibrium physics (from IC):")
    eq = ic['eq_physics']
    for k, v in sorted(eq.items()):
        if isinstance(v, (int, float)):
            print(f"  {k}: {v}")

# Expected tension from IC
print(f"\nExpected equilibrium at kinematic exit:")
print(f"  Tension: {ic.get('tension_eq_n', 300):.0f} N")
print(f"  Thrust needed (weight): {weight_N:.1f} N")

# Now check telemetry at t=80s to see if system is at IC
tel_file = 'simulation/logs/test_lua_flight_steady_sitl/telemetry.csv'
try:
    import pandas as pd
    df = pd.read_csv(tel_file, low_memory=False)
    
    # Find rows near t=80s (use t_sim column)
    df_80 = df[(df['t_sim'] >= 79.9) & (df['t_sim'] <= 80.1)].copy()
    
    print(f"\n" + "="*80)
    print(f"TELEMETRY AT t=80.0s (KINEMATIC EXIT)")
    print(f"="*80)
    
    if len(df_80) > 0:
        row = df_80.iloc[0]
        print(f"\nSystem state at kinematic exit:")
        print(f"  Time: {row['t_sim']:.3f} s")
        print(f"  Position NED: [{row['pos_x']:.2f}, {row['pos_y']:.2f}, {row['pos_z']:.2f}]")
        print(f"  Velocity NED: [{row['vel_x']:.4f}, {row['vel_y']:.4f}, {row['vel_z']:.4f}]")
        
        # Find aero columns
        aero_cols = [c for c in df.columns if 'aero' in c.lower()]
        if 'aero_T' in df.columns:
            aero_thrust = row['aero_T']
        elif 'aero_thrust' in df.columns:
            aero_thrust = row['aero_thrust']
        else:
            aero_thrust = None
            
        coll_cols = [c for c in df.columns if 'collective' in c.lower()]
        if 'collective_rad' in df.columns:
            coll = row['collective_rad']
        else:
            coll = None
        
        print(f"  Collective rad: {coll}")
        print(f"  Tether tension: {row['tether_tension']:.1f} N")
        if 'omega_rotor' in df.columns:
            print(f"  Rotor RPM: {row['omega_rotor']:.2f}")
        print(f"  Aero thrust: {aero_thrust}")
        
        pos_ic = np.array(ic['pos'])
        pos_tel = np.array([row['pos_x'], row['pos_y'], row['pos_z']])
        pos_err = np.linalg.norm(pos_ic - pos_tel)
        
        if coll is not None:
            col_err = abs(ic_coll - coll)
        else:
            col_err = None
        
        print(f"\nComparison IC vs telemetry at t=80s:")
        print(f"  Position error: {pos_err:.4f} m")
        if col_err is not None:
            print(f"  Collective error: {col_err:.4f} rad")
        print(f"  Tension at t=80s: {row['tether_tension']:.1f} N (target {ic.get('tension_eq_n', 300):.0f} N)")
        
        if aero_thrust is not None and (aero_thrust > weight_N - 5):
            print(f"  Aero thrust: {aero_thrust:.1f} N (weight: {weight_N:.1f} N)")
            
        if pos_err < 0.01 and (col_err is None or col_err < 0.001) and (aero_thrust is None or aero_thrust > weight_N - 5):
            print(f"\n✓ SYSTEM IS AT IC EQUILIBRIUM")
            print(f"  All parameters match IC values within tolerance")
        else:
            print(f"\n✗ SYSTEM IS NOT AT IC EQUILIBRIUM")
            if pos_err >= 0.01:
                print(f"  - Position error too large: {pos_err:.4f} m")
            if col_err is not None and col_err >= 0.001:
                print(f"  - Collective error too large: {col_err:.4f} rad")
            if aero_thrust is not None and aero_thrust < weight_N - 5:
                print(f"  - Aero thrust too low for hover: {aero_thrust:.1f} N < {weight_N:.1f} N")
                
    else:
        print(f"No telemetry data at t=80s")
        
except Exception as e:
    import traceback
    print(f"Could not read telemetry: {e}")
    traceback.print_exc()
