import sys, math
import numpy as np
sys.path.insert(0, "e:/repos/windpower/simulation")
from envelope.point_mass import tether_hat, balance_bz
import rotor_definition as rd

mass = rd.default().dynamics_kwargs()["mass"]

print(f"{'el':>4}  {'T':>5}  {'bz·t_hat':>9}  {'xi_disk_tether_deg':>19}  {'bz':>30}")
for el in [20, 30, 35, 45, 60, 70, 80, 90]:
    for T in [200, 400, 600]:
        th = tether_hat(el)
        bz = balance_bz(el, T, mass)
        dot = float(np.dot(bz, th))
        xi  = math.degrees(math.acos(max(-1.0, min(1.0, -dot))))  # angle between thrust and tether
        print(f"{el:4.0f}  {T:5.0f}  {dot:+9.4f}  {xi:19.2f}  {np.round(bz,3)}")
