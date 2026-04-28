"""
check_landing_forces.py -- Force balance sanity check for landing descent at xi=80 deg.

At xi=80 deg, disk is nearly horizontal. For equilibrium:
  - Thrust T along disk_normal must support weight + tether tension component
  - Print thrust vs collective sweep to find the actual col_min for hover
"""
import sys, math
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "unit"))

import rotor_definition as rd
from aero import create_aero
from controller import col_min_for_altitude_rad
from simtest_ic import load_ic

WIND       = np.array([0.0, 10.0, 0.0])
XI_DEG     = 80.0
OMEGA_SPIN = 20.3   # rad/s (from IC)
T_AERO     = 50.0   # well past ramp
MASS_KG    = rd.default().mass_kg
G          = 9.81

rotor = rd.default()
aero  = create_aero(rotor)
IC    = load_ic()

# Build R_hub at xi=80 deg with equilibrium yaw (body_x = East projected)
xi_rad     = math.radians(XI_DEG)
# disk_normal at xi=80 from vertical: tilted 80 deg from [0,0,-1]
# pointing mostly South + slightly up: [sin(xi), 0, -cos(xi)] if tilt toward North
# Use same direction as IC but scaled to xi=80:
bz_unit = IC.R0[:, 2] / np.linalg.norm(IC.R0[:, 2])
# Build disk_normal at xi=80 in the same azimuthal direction as IC
bz_horiz = bz_unit.copy(); bz_horiz[2] = 0.0
bz_horiz_norm = np.linalg.norm(bz_horiz)
if bz_horiz_norm > 1e-6:
    bz_horiz /= bz_horiz_norm
else:
    bz_horiz = np.array([1.0, 0.0, 0.0])
disk_normal = math.cos(xi_rad) * bz_horiz + math.sin(xi_rad) * np.array([0.0, 0.0, -1.0])
disk_normal /= np.linalg.norm(disk_normal)

# Build orbital frame around disk_normal
east = np.array([0.0, 1.0, 0.0])
ep   = east - np.dot(east, disk_normal) * disk_normal
bx   = ep / np.linalg.norm(ep)
by   = np.cross(disk_normal, bx)
R_hub = np.column_stack([bx, by, disk_normal])

v_hub = np.zeros(3)

weight = MASS_KG * G
print(f"Hub mass: {MASS_KG:.2f} kg   weight: {weight:.1f} N")
print(f"xi = {XI_DEG} deg   disk_normal = {disk_normal.round(3)}")
print(f"Axial wind component: {np.dot(WIND, disk_normal):.2f} m/s")
print(f"In-plane wind:        {np.linalg.norm(WIND - np.dot(WIND, disk_normal)*disk_normal):.2f} m/s")
print()

# At xi=80, the tether pulls the hub toward anchor.
# For a hub at (pos_x, pos_y, alt), tether direction ~ -pos/|pos|
# Approximate tether force: along -disk_normal direction (for nearly-tangential orbit)
# Actually at xi=80 the tether is roughly along -disk_normal (tangent tethered kite config)
# Weight component along disk_normal:
w_axial = weight * abs(np.dot(np.array([0.0, 0.0, 1.0]), disk_normal))
print(f"Weight component along disk_normal (axial load): {w_axial:.1f} N")
print()

# Sweep collective
print(f"{'coll_rad':>10}  {'T_axial':>9}  {'F_z':>8}  {'net_axial':>10}  {'K_skew':>7}  {'v_i':>6}")
print("-" * 60)
for coll in np.arange(-0.30, 0.15, 0.01):
    r = aero.compute_forces(
        collective_rad=float(coll),
        tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R_hub, v_hub_world=v_hub,
        omega_rotor=OMEGA_SPIN, wind_world=WIND,
        t=T_AERO,
    )
    T_axial  = float(np.dot(r.F_world, disk_normal))   # thrust along disk axis
    net      = T_axial - w_axial                        # net axial force
    print(f"{coll:10.3f}  {T_axial:9.1f}  {float(r.F_world[2]):8.1f}  {net:10.1f}  "
          f"{aero.last_K_skew:7.4f}  {aero.last_v_i:6.3f}")

print()
print(f"col_min_for_altitude_rad(xi=80): {col_min_for_altitude_rad(aero, XI_DEG, MASS_KG, omega=OMEGA_SPIN):.4f} rad")
print(f"COL_CRUISE hardcoded:             0.0790 rad")
