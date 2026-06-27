"""
Verify IC force balance and augment with tether direction/force for assertions.
"""
import json
import numpy as np
import sys
sys.path.insert(0, 'simulation')

from tether import TetherModel

# Load IC
with open('simulation/steady_state_starting.json') as f:
    ic = json.load(f)

print("="*80)
print("IC FORCE BALANCE VERIFICATION")
print("="*80)

ic_pos = np.array(ic['pos'], dtype=float)
ic_vel = np.array(ic.get('vel', [0, 0, 0]), dtype=float)
ic_R0 = np.array(ic['R0'], dtype=float).reshape(3, 3)
ic_coll = ic.get('coll_eq_rad', ic.get('stack_coll_eq'))
ic_omega = ic['omega_spin']
mass_kg = ic['eq_physics']['mass_kg']
g = 9.81

print(f"\nIC State:")
print(f"  Position NED: {ic_pos}")
print(f"  Velocity NED: {ic_vel}")
print(f"  Collective: {ic_coll:.4f} rad")
print(f"  Rotor RPM: {ic_omega:.2f} rad/s")
print(f"  Mass: {mass_kg} kg")
print(f"  R0 body_z: {ic_R0[:, 2]}")

# Compute tether direction and force
rest_length = ic['rest_length']
tether_model = TetherModel(
    anchor_ned=np.zeros(3),
    rest_length=rest_length,
    axle_attachment_length=0.0,
)

# Tether force: pulls hub toward anchor
tether_length = np.linalg.norm(ic_pos)
tether_direction = ic_pos / tether_length  # unit vector from anchor to hub
tether_extension = tether_length - rest_length
tether_force_mag = TetherModel.EA_N * tether_extension / rest_length if tether_extension > 0 else 0.0
tether_force_ned = tether_force_mag * tether_direction  # force on hub pulls toward anchor

print(f"\nTether State:")
print(f"  Rest length: {rest_length:.2f} m")
print(f"  Actual length: {tether_length:.2f} m")
print(f"  Extension: {tether_extension:.4f} m")
print(f"  Force magnitude: {tether_force_mag:.1f} N")
print(f"  Direction (toward anchor): {tether_direction}")
print(f"  Force vector NED: {tether_force_ned}")
print(f"  Body_z direction: {ic_R0[:, 2]}")

# Dot product: should be close to 1 if body_z aligns with tether
alignment = np.dot(ic_R0[:, 2], tether_direction)
print(f"  Alignment (body_z · tether_dir): {alignment:.4f}")
if abs(alignment) > 0.9:
    print(f"    [OK] Body_z is well-aligned with tether")
elif abs(alignment) > 0.5:
    print(f"    [WARN] Body_z has partial alignment with tether (angle = {np.degrees(np.arccos(abs(alignment))):.1f}°)")
else:
    print(f"    [FAIL] Body_z is NOT aligned with tether (angle = {np.degrees(np.arccos(abs(alignment))):.1f}°)")

# Check forces from IC eq_physics
eq = ic['eq_physics']
aero_force = np.array([eq['aero_fx'], eq['aero_fy'], eq['aero_fz']], dtype=float)
gravity_force = np.array([0, 0, mass_kg * g], dtype=float)  # NED: Z is down, so gravity is positive

print(f"\nForces at IC:")
print(f"  Gravity NED: {gravity_force}")
print(f"  Aero NED: {aero_force}")
print(f"  Tether NED: {tether_force_ned}")

# Net force
F_net = aero_force + gravity_force + tether_force_ned
F_net_mag = np.linalg.norm(F_net)

print(f"\nNet Force:")
print(f"  F_net NED: {F_net}")
print(f"  |F_net|: {F_net_mag:.3f} N")

if F_net_mag < 1.0:
    print(f"  [OK] BALANCED (residual < 1 N)")
elif F_net_mag < 5.0:
    print(f"  [WARN] NEARLY BALANCED (residual {F_net_mag:.1f} N, expect < 1 N)")
else:
    print(f"  [FAIL] NOT BALANCED (residual {F_net_mag:.1f} N, expect < 1 N)")

# Compare with IC's reported residual
if 'F_residual' in eq:
    ic_residual = np.array(eq['F_residual'], dtype=float)
    ic_residual_mag = np.linalg.norm(ic_residual)
    print(f"\nIC reported F_residual: {ic_residual_mag:.3f} N")
    print(f"  Recalculated F_net:   {F_net_mag:.3f} N")
    if abs(ic_residual_mag - F_net_mag) < 0.1:
        print(f"  ✓ Match!")
    else:
        print(f"  ⚠ Discrepancy: {abs(ic_residual_mag - F_net_mag):.3f} N")

# Update IC with tether info for future assertions
ic['tether_state'] = {
    'rest_length_m': float(rest_length),
    'actual_length_m': float(tether_length),
    'extension_m': float(tether_extension),
    'force_magnitude_n': float(tether_force_mag),
    'direction_ned': tether_direction.tolist(),
    'force_vector_ned': tether_force_ned.tolist(),
}

ic['force_balance'] = {
    'aero_ned': aero_force.tolist(),
    'gravity_ned': gravity_force.tolist(),
    'tether_ned': tether_force_ned.tolist(),
    'net_ned': F_net.tolist(),
    'net_magnitude_n': float(F_net_mag),
    'body_z_tether_alignment': float(alignment),
    'is_balanced': float(F_net_mag) < 1.0,
}

# Write updated IC back to file
print(f"\n" + "="*80)
print("UPDATING steady_state_starting.json with tether/force balance info")
print("="*80)

with open('simulation/steady_state_starting.json', 'w') as f:
    json.dump(ic, f, indent=2)

print(f"[OK] Updated IC file with tether_state and force_balance fields")
print(f"\nYou can now assert:")
print(f"  assert ic['force_balance']['is_balanced'] == True")
print(f"  assert ic['force_balance']['net_magnitude_n'] < 1.0")
print(f"  assert ic['force_balance']['body_z_tether_alignment'] > 0.9")
