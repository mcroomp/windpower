"""
aero_basics_test.py -- Verify airfoil direction, hover thrust, and autorotation.

Two tests:
  A) Hover (no wind, disk horizontal, rotor spinning): collective sweep.
     Expect positive upward thrust (F_up > 0) for positive collective.
     Expect autorotation-style torque direction.

  B) Autorotation (disk horizontal, 10 m/s wind blowing horizontally):
     At collective=0, wind should drive rotor (Q_spin > 0).
     Check spin-up torque vs collective.

Conventions (NED):
  R_hub[:,2] = body_z = disk normal (NED).
  For a disk facing up: body_z = [0, 0, -1] (NED Z is down, so -1 = up).
  F_world[2] < 0  =>  upward force (NED).
  Q_spin > 0      =>  spin drives rotor faster (autorotation driving torque).

Usage
-----
    .venv/Scripts/python.exe simulation/analysis/aero_basics_test.py
"""
import sys, math, json
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rotor_definition as rd
from aero import create_aero

# ── Setup ──────────────────────────────────────────────────────────────────────
import argparse
_p = argparse.ArgumentParser()
_p.add_argument("--model", default="skewed_wake_numpy",
    choices=["skewed_wake","skewed_wake_numpy","skewed_wake2","skewed_wake2_jit",
             "peters_he","peters_he_numpy","deschutter","prandtl","glauert"])
_args = _p.parse_args()

rotor  = rd.default()
aero   = create_aero(rotor, model=_args.model)
MASS   = rotor.mass_kg
G      = 9.81
MG     = MASS * G

d      = json.loads((Path(__file__).resolve().parents[1] / "steady_state_starting.json").read_text())
OMEGA_IC = float(d["omega_spin"])   # equilibrium spin from IC

COL_MIN  = -0.28
COL_MAX  =  0.10

# ── R_hub for horizontal disk (body_z pointing up in NED = [0,0,-1]) ──────────
# body_z = [0, 0, -1]  (up in NED)
# body_x = [1, 0,  0]  (North)
# body_y = [0, 1,  0]  (East)   -- right-hand: x cross y = z => [1,0,0]x[0,1,0]=[0,0,1] but we need [0,0,-1]
#   => use body_y = [-1, 0, 0] x [0, 0, -1] ... let's just set it directly
# R_hub columns = [body_x, body_y, body_z] in NED
R_HOVER = np.array([
    [ 1.0,  0.0,  0.0],   # body_x = North
    [ 0.0, -1.0,  0.0],   # body_y = -East  (right-hand with body_z=up)
    [ 0.0,  0.0, -1.0],   # body_z = Up (NED: -Z)
], dtype=float)
# Verify: R[:,2] should be [0,0,-1]
assert abs(R_HOVER[2, 2] - (-1.0)) < 1e-9, "body_z not pointing up"

print()
print(f"aero_basics_test -- airfoil direction, hover thrust, and autorotation  [model={_args.model}]")
print(f"  mass = {MASS:.2f} kg   mg = {MG:.2f} N")
print(f"  IC omega_spin = {OMEGA_IC:.2f} rad/s")
print(f"  Disk facing UP: body_z (NED) = [0, 0, -1]")
print()

# ── Test A: Hover (no wind) ────────────────────────────────────────────────────
print("=" * 70)
print("A: HOVER -- no wind, disk facing up, spinning at IC omega")
print("   Expect: positive collective => upward thrust (F_up > 0)")
print()
print(f"  {'col_rad':>8}  {'F_up_N':>8}  {'F_east_N':>9}  {'F_north_N':>10}  "
      f"{'Q_spin':>9}  {'note':}")
print(f"  {'-'*8}  {'-'*8}  {'-'*9}  {'-'*10}  {'-'*9}  {'-'*25}")

WIND_NONE = np.array([0.0, 0.0, 0.0])
for col in [COL_MIN, -0.20, -0.18, -0.15, -0.10, -0.05, 0.0, 0.05, COL_MAX]:
    res = aero.compute_forces(col, 0.0, 0.0, R_HOVER, np.zeros(3),
                              OMEGA_IC, WIND_NONE, t=45.0)
    F_up    = float(-res.F_world[2])   # NED Z negated = upward
    F_east  = float(res.F_world[1])
    F_north = float(res.F_world[0])
    Q_spin  = float(res.Q_spin)
    note = ""
    if col == COL_MIN:  note = "<-- col_min"
    if col == COL_MAX:  note = "<-- col_max"
    if abs(col) < 1e-4: note = "<-- col=0"
    lift_ok = "LIFT" if F_up > MG else ("partial" if F_up > 0 else "DOWN")
    print(f"  {col:>8.3f}  {F_up:>8.1f}  {F_east:>9.1f}  {F_north:>10.1f}  "
          f"{Q_spin:>9.3f}  {lift_ok}  {note}")
print()

# ── Test B: Autorotation (10 m/s East wind, disk facing up) ───────────────────
print("=" * 70)
print("B: AUTOROTATION -- 10 m/s East wind, disk facing up")
print("   Expect: Q_spin > 0 (wind drives rotor) at low/zero collective")
print("   (Positive Q_spin = rotor spins faster = wind energy captured)")
print()
print(f"  {'col_rad':>8}  {'omega':>8}  {'F_up_N':>8}  {'Q_spin':>9}  "
      f"{'driving?':>10}  note")
print(f"  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*9}  {'-'*10}  {'-'*20}")

WIND_10 = np.array([0.0, 10.0, 0.0])
for col in [COL_MIN, -0.20, -0.18, -0.15, -0.10, -0.05, 0.0, 0.05, COL_MAX]:
    res = aero.compute_forces(col, 0.0, 0.0, R_HOVER, np.zeros(3),
                              OMEGA_IC, WIND_10, t=45.0)
    F_up   = float(-res.F_world[2])
    Q_spin = float(res.Q_spin)
    driving = "DRIVING" if Q_spin > 0 else "BRAKING"
    note = ""
    if col == COL_MIN:  note = "<-- col_min"
    if col == COL_MAX:  note = "<-- col_max"
    if abs(col) < 1e-4: note = "<-- col=0"
    print(f"  {col:>8.3f}  {OMEGA_IC:>8.2f}  {F_up:>8.1f}  {Q_spin:>9.3f}  "
          f"{driving:>10}  {note}")
print()

# ── Test C: Spin rate sweep at col=0 with wind, to find equilibrium ────────────
print("=" * 70)
print("C: Spin sweep at col=0, 10 m/s wind -- find autorotation equilibrium")
print("   Q_spin=0 crossover = autorotation equilibrium RPM")
print()
print(f"  {'omega':>8}  {'RPM':>8}  {'Q_spin':>9}  {'F_up_N':>8}  status")
print(f"  {'-'*8}  {'-'*8}  {'-'*9}  {'-'*8}  {'-'*15}")

for omega in [2.0, 5.0, 8.0, 10.0, 12.0, 15.0, 18.0, 20.0, 25.0, 30.0]:
    res = aero.compute_forces(0.0, 0.0, 0.0, R_HOVER, np.zeros(3),
                              omega, WIND_10, t=45.0)
    F_up   = float(-res.F_world[2])
    Q_spin = float(res.Q_spin)
    rpm    = omega * 60.0 / (2 * math.pi)
    status = "driving" if Q_spin > 0 else "braking"
    print(f"  {omega:>8.1f}  {rpm:>8.1f}  {Q_spin:>9.3f}  {F_up:>8.1f}  {status}")

print()
print(f"  (IC omega = {OMEGA_IC:.2f} rad/s = {OMEGA_IC*60/(2*math.pi):.1f} RPM)")
print()
