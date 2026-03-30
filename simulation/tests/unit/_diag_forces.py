import sys
sys.path.insert(0, "/rawes/simulation")
import numpy as np
from aero import RotorAero
import rotor_definition as _rd
from tether import TetherModel
from dynamics import RigidBodyDynamics
from frames import build_orb_frame
from controller import compute_swashplate_from_state

POS0 = np.array([46.258, 14.241, 12.530])
VEL0 = np.array([-0.257, 0.916, -0.093])
BODY_Z0 = np.array([0.851018, 0.305391, 0.427206])
OMEGA_SPIN0 = 20.148
ANCHOR = np.zeros(3)
WIND = np.array([10.0, 0.0, 0.0])
DT = 1.0/400.0
K_DRIVE_SPIN=1.4; K_DRAG_SPIN=0.01786; I_SPIN=10.0; OMEGA_MIN=0.5
R0 = build_orb_frame(BODY_Z0)

# Scan collective to find one that gives net_z >= 0 at equilibrium
aero = RotorAero(_rd.default())
tether = TetherModel(anchor_enu=ANCHOR, rest_length=49.949)
tf, _ = tether.compute(POS0, VEL0, R0)
print("Scanning collective for altitude equilibrium (net_z ~ 0):")
for coll_deg in [0, -5, -10, -15, -20, -25, -30, -35, -40]:
    coll = np.radians(coll_deg)
    f = aero.compute_forces(collective_rad=coll, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R0, v_hub_world=VEL0, omega_rotor=OMEGA_SPIN0, wind_world=WIND, t=50.0)
    net_z = f[2] + tf[2] - 5*9.81
    print(f"  coll={coll_deg:4d}d  F_aero_z={f[2]:.1f}  net_z={net_z:.1f} N")

# Find equilibrium collective
print("\nFinding equilibrium collective (net_z = 0):")
lo, hi = np0 = np.radians(0.0), np.radians(-50.0)
for _ in range(50):
    mid = (lo + hi) / 2
    f = aero.compute_forces(collective_rad=mid, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R0, v_hub_world=VEL0, omega_rotor=OMEGA_SPIN0, wind_world=WIND, t=50.0)
    nz = f[2] + tf[2] - 5*9.81
    if nz > 0:
        lo = mid
    else:
        hi = mid
coll_eq = (lo + hi) / 2
print(f"  coll_eq = {np.degrees(coll_eq):.2f} deg = {coll_eq:.4f} rad  (net_z={nz:.3f} N)")

# Now test 60s stability with equilibrium collective and M_spin included
print("\n--- 60s stability test with coll_eq and full M (like steady-state test) ---")
aero2 = RotorAero(_rd.default())
tether2 = TetherModel(anchor_enu=ANCHOR, rest_length=49.949)
dyn = RigidBodyDynamics(mass=5.0, I_body=[5.0,5.0,10.0], I_spin=0.0,
    pos0=POS0.tolist(), vel0=VEL0.tolist(), R0=R0, omega0=[0.0,0.0,0.0], z_floor=1.0)
hs = dyn.state
omega_spin = OMEGA_SPIN0
floor_hits = 0
for i in range(int(60.0/DT)+1):
    t = 50.0 + i*DT
    f = aero2.compute_forces(collective_rad=coll_eq, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=hs["R"], v_hub_world=hs["vel"], omega_rotor=omega_spin, wind_world=WIND, t=t)
    tf2, tm2 = tether2.compute(hs["pos"], hs["vel"], hs["R"])
    f[0:3] += tf2; f[3:6] += tm2
    Q = K_DRIVE_SPIN * aero2.last_v_inplane - K_DRAG_SPIN*omega_spin**2
    omega_spin = max(OMEGA_MIN, omega_spin + Q/I_SPIN*DT)
    M = f[3:6]  # include M_spin (full moment like steady-state test)
    M += -50.0 * hs["omega"]
    hs = dyn.step(f[0:3], M, DT, omega_spin=omega_spin)
    if hs["pos"][2] <= 1.05:
        floor_hits += 1
    if i % int(10.0/DT) == 0:
        bz = hs["R"][:,2]
        print(f"  t={i*DT:.0f}s z={hs['pos'][2]:.2f}m body_z_z={bz[2]:.3f} spin={omega_spin:.1f} floor_hits={floor_hits}")
