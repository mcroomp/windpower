"""
Diagnostic: scan BEM spin torque vs collective for PCA-2 test points.
Run to check whether autorotation zero-crossing exists in the BEM model.
"""
import sys, math
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
from aero import rotor_definition as rd
from aero import create_aero

rotor = rd.load("pca2_1934")
aero  = create_aero(rotor)

TEST_POINTS = [
    (0.210, 9.0, 99.6, 0.006251),
    (0.307, 4.8, 99.6, 0.005892),
    (0.341, 4.0, 98.0, 0.005793),
    (0.512, 2.1, 98.8, 0.005140),
]

def make_state(alpha_s_deg, rpm, mu):
    a = math.radians(alpha_s_deg)
    omega = rpm * 2 * math.pi / 60
    V = mu * omega * 6.858 / math.cos(a)
    bz = np.array([math.sin(a), 0.0, math.cos(a)])
    bx = np.array([math.cos(a), 0.0, -math.sin(a)])
    by = np.array([0.0, 1.0, 0.0])
    return np.column_stack([bx, by, bz]), omega, np.array([V, 0.0, 0.0])

colls = [-0.10, -0.05, 0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35]

print("BEM spin torque dot(M, disk_normal) vs collective [N·m]")
print(f"{'mu':>6} {'as':>5} | " + " ".join(f"{c:>9.2f}" for c in colls))
print("-" * (14 + 10 * len(colls)))

for mu, as_deg, rpm, ct_exp in TEST_POINTS:
    R_hub, omega, wind = make_state(as_deg, rpm, mu)
    bz = R_hub[:, 2]
    vals = []
    for c in colls:
        f = aero.compute_forces(c, 0.0, 0.0, R_hub, np.zeros(3), omega, wind, t=10.0)
        vals.append(np.dot(f[3:], bz))
    print(f"{mu:>6.3f} {as_deg:>5.1f} | " + " ".join(f"{v:>9.1f}" for v in vals))

R_TIP = 6.858
RHO   = 1.225
A     = math.pi * R_TIP**2

def find_autorotation_coll(aero, R_hub, omega, wind):
    bz = R_hub[:, 2]
    def q(c):
        f = aero.compute_forces(c, 0.0, 0.0, R_hub, np.zeros(3), omega, wind, t=10.0)
        return float(np.dot(f[3:], bz))
    lo, hi = -0.08, 0.08
    if q(lo) * q(hi) > 0:
        return None
    for _ in range(60):
        mid = 0.5*(lo+hi)
        if q(lo)*q(mid) <= 0: hi = mid
        else: lo = mid
    return 0.5*(lo+hi)

def ct_at(aero, R_hub, omega, wind, coll):
    bz = R_hub[:, 2]
    f  = aero.compute_forces(coll, 0.0, 0.0, R_hub, np.zeros(3), omega, wind, t=10.0)
    T  = float(np.dot(f[:3], bz))
    return T / (RHO * A * (omega * R_TIP)**2)

print()
print("CT at autorotation collective (BEM Q_spin=0):")
print(f"{'mu':>6} {'as':>5} | {'coll_deg':>9} {'CT_sim':>10} {'CT_exp':>10} {'ratio':>7}  note")
print("-" * 75)
for mu, as_deg, rpm, ct_exp in TEST_POINTS:
    R_hub, omega, wind = make_state(as_deg, rpm, mu)
    coll = find_autorotation_coll(aero, R_hub, omega, wind)
    if coll is None:
        print(f"{mu:>6.3f} {as_deg:>5.1f} | {'n/a':>9} {'n/a':>10} {ct_exp:>10.6f} {'n/a':>7}  no zero crossing")
        continue
    ct_sim = ct_at(aero, R_hub, omega, wind, coll)
    ratio  = ct_sim / ct_exp if ct_exp > 0 else float("inf")
    note   = "retreating reversal dominates" if ct_sim < 0 else "ok"
    print(f"{mu:>6.3f} {as_deg:>5.1f} | {math.degrees(coll):>9.2f} {ct_sim:>10.6f} {ct_exp:>10.6f} {ratio:>7.3f}  {note}")

print()
print("CT at fixed collective=0.025 rad (1.4 deg) — sensitivity check:")
print(f"{'mu':>6} {'as':>5} | {'CT_sim':>10} {'CT_exp':>10} {'ratio':>7}")
print("-" * 50)
for mu, as_deg, rpm, ct_exp in TEST_POINTS:
    R_hub, omega, wind = make_state(as_deg, rpm, mu)
    ct_sim = ct_at(aero, R_hub, omega, wind, 0.025)
    ratio  = ct_sim / ct_exp if ct_exp > 0 else float("inf")
    print(f"{mu:>6.3f} {as_deg:>5.1f} | {ct_sim:>10.6f} {ct_exp:>10.6f} {ratio:>7.3f}")
