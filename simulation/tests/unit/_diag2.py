import sys; sys.path.insert(0, "/rawes/simulation")
import numpy as np
from aero import RotorAero
import rotor_definition as _rd
from tether import TetherModel
from dynamics import RigidBodyDynamics
from frames import build_orb_frame

POS0=np.array([46.258,14.241,12.530]); VEL0=np.array([-0.257,0.916,-0.093])
BODY_Z0=np.array([0.851018,0.305391,0.427206]); ANCHOR=np.zeros(3)
WIND=np.array([10.,0.,0.]); DT=1./400.
K_DRIVE=1.4; K_DRAG=0.01786; I_SPIN=10.; OMEGA_MIN=0.5
R0=build_orb_frame(BODY_Z0)

aero=RotorAero(_rd.default())
tether=TetherModel(anchor_ned=ANCHOR,rest_length=49.949,axle_attachment_length=0.0)
dyn=RigidBodyDynamics(mass=5.,I_body=[5.,5.,10.],I_spin=0.,
    pos0=POS0.tolist(),vel0=VEL0.tolist(),R0=R0,omega0=[0.,0.,0.],z_floor=1.)
hs=dyn.state; omega_spin=20.148; floor_hits=0

print("Zero tilt, axle_attach=0, k_ang=50, sub M_spin (exact mediator match for Phase 2):")
for i in range(int(60./DT)+1):
    t=50.+i*DT
    result=aero.compute_forces(0.,0.,0.,hs["R"],hs["vel"],omega_spin,WIND,t=t)
    tf,tm=tether.compute(hs["pos"],hs["vel"],hs["R"])
    F_net=result.F_world+tf
    M_orbital=result.M_orbital+tm
    Q=K_DRIVE*aero.last_v_inplane-K_DRAG*omega_spin**2
    omega_spin=max(OMEGA_MIN,omega_spin+Q/I_SPIN*DT)
    M_orbital+=-50.*hs["omega"]
    hs=dyn.step(F_net,M_orbital,DT,omega_spin=omega_spin)
    if hs["pos"][2]<=1.05: floor_hits+=1
    if i%int(10./DT)==0:
        bz=hs["R"][:,2]
        pos=hs["pos"]
        print(f"  t={i*DT:.0f}s z={pos[2]:.2f}m bz_z={bz[2]:.3f} spin={omega_spin:.1f} fhits={floor_hits} pos=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})")
