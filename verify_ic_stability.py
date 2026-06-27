"""
Verify that the IC (Initial Condition) is actually stable at kinematic exit (t=80s).
Check that position, velocity, collective, and aero forces match expectations.
"""
import json
import numpy as np
import pandas as pd
from pathlib import Path

# Load IC from steady_state_starting.json
ic_file = Path("simulation/steady_state_starting.json")
if not ic_file.exists():
    print("ERROR: steady_state_starting.json not found")
    exit(1)

with open(ic_file) as f:
    ic = json.load(f)

# Load telemetry at t=80s
log_dir = Path("simulation/logs/test_lua_flight_steady_sitl")
if not log_dir.exists():
    print(f"ERROR: {log_dir} not found. Run test first.")
    exit(1)

tel_file = None
for root, dirs, files in log_dir.walk():
    if "telemetry.csv" in files:
        tel_file = root / "telemetry.csv"
        break

if tel_file is None:
    print("ERROR: telemetry.csv not found")
    exit(1)

print(f"Loading telemetry from: {tel_file}")
df = pd.read_csv(tel_file)

# Find rows around t=80s (kinematic exit) and t=81,82,83s (free flight)
t_init = 80.0
t_end = 84.0
mask = (df['t_sim'] >= t_init) & (df['t_sim'] <= t_end)
subset = df[mask].copy()

if subset.empty:
    print(f"ERROR: No data between t={t_init}s and t={t_end}s")
    exit(1)

print("\n" + "="*80)
print("IC VERIFICATION AT KINEMATIC EXIT (t=80s+)")
print("="*80)

print("\n[IC from steady_state_starting.json]")
print(f"  Position NED:        {np.array(ic['pos'])}")
print(f"  Velocity:            {np.array(ic['vel'])}")
print(f"  Collective eq:       {ic.get('coll_eq_rad', ic.get('stack_coll_eq')):.4f} rad")
print(f"  Tension target:      {ic.get('tension_eq_n', 300):.0f} N")
print(f"  Rotor omega:         {ic['omega_spin']:.2f} rad/s")
print(f"  Rest length:         {ic['rest_length']:.2f} m")

print("\n[Telemetry at t=80.0s (end of kinematic)]")
t80_rows = subset[subset['t_sim'] <= 80.1]
if not t80_rows.empty:
    row = t80_rows.iloc[0]
    print(f"  Position NED:        [{row['pos_x']:.2f}, {row['pos_y']:.2f}, {row['pos_z']:.2f}]")
    print(f"  Velocity:            [{row['vel_x']:.4f}, {row['vel_y']:.4f}, {row['vel_z']:.4f}]")
    print(f"  Collective:          {row['collective_rad']:.4f} rad")
    print(f"  Tether tension:      {row['tether_tension']:.0f} N")
    print(f"  Rotor RPM:           {row['omega_rotor']:.2f} rad/s")
    print(f"  Aero thrust:         {row['aero_T']:.1f} N")
    print(f"  Tether length:       {row['tether_length']:.2f} m (extension: {row['tether_extension']:.3f}m)")
    
    # Check if we're at IC
    pos_error = np.linalg.norm(np.array([row['pos_x'], row['pos_y'], row['pos_z']]) - np.array(ic['pos']))
    coll_ic = ic.get('coll_eq_rad', ic.get('stack_coll_eq'))
    coll_error = abs(row['collective_rad'] - coll_ic)
    
    print(f"\n  Position error from IC:      {pos_error:.4f} m")
    print(f"  Collective error from IC:    {coll_error:.6f} rad")

print("\n[Telemetry at t=83.8s (peak tension spike)]")
t83_rows = subset[(subset['t_sim'] >= 83.7) & (subset['t_sim'] <= 83.9)]
if not t83_rows.empty:
    row = t83_rows.iloc[0]
    print(f"  Position NED:        [{row['pos_x']:.2f}, {row['pos_y']:.2f}, {row['pos_z']:.2f}]")
    print(f"  Velocity:            [{row['vel_x']:.2f}, {row['vel_y']:.2f}, {row['vel_z']:.2f}]")
    print(f"  Collective:          {row['collective_rad']:.4f} rad")
    print(f"  Tether tension:      {row['tether_tension']:.0f} N  <-- SPIKE!")
    print(f"  Rotor RPM:           {row['omega_rotor']:.2f} rad/s")
    print(f"  Aero thrust:         {row['aero_T']:.1f} N")
    print(f"  Tether extension:    {row['tether_extension']:.2f} m")

print("\n[Analysis]")
print("If the IC is stable, the system at t=80.0s should:")
print("  1. Be at or very near the IC position")
print("  2. Have zero or near-zero velocity")
print("  3. Command the IC collective")
print("  4. Experience tether tension ≈ 300N")
print("  5. Not descend (tether extension should not increase)")
print()

# Check tension trajectory
t80_t84 = subset[(subset['t_sim'] >= 80.0) & (subset['t_sim'] <= 84.0)]
print(f"Tension evolution from t=80s to peak:")
for idx, (_, row) in enumerate(t80_t84.iterrows()):
    if idx % 20 == 0:  # Sample every 20th row
        print(f"  t={row['t_sim']:.1f}s: tension={row['tether_tension']:.0f}N, pos_z={row['pos_z']:.2f}m, aero_T={row['aero_T']:.0f}N, vel_z={row['vel_z']:.2f}m/s")

print()


