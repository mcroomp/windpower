"""
Domain table: where does the BEM agree vs disagree with PCA-2 empirical data?

Scans advance ratio mu from 0 to 0.55 at fixed shaft angle and RPM.
Key threshold: mu * R = r_root → reversed-flow region enters the blade strip range.
Below this mu, the BEM is physically valid. Above it, retreating-blade reversal
dominates and the model breaks down without blade flapping.
"""
import sys, math
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
import rotor_definition as rd
from aero import create_aero

rotor = rd.load("pca2_1934")
aero  = create_aero(rotor)

R_TIP   = 6.858   # m
R_ROOT  = 0.500   # m  (root cutout)
RHO     = 1.225
A       = math.pi * R_TIP**2
RPM     = 99.0
OMEGA   = RPM * 2 * math.pi / 60   # ~10.37 rad/s
AS_DEG  = 5.0                       # fixed shaft angle for scan

# Experimental data (mu, CT_exp) from Harris App 11.8
EXP = {0.210: 0.006251, 0.307: 0.005892, 0.341: 0.005793, 0.512: 0.005140}

def make_state(mu, as_deg):
    a  = math.radians(as_deg)
    V  = mu * OMEGA * R_TIP / math.cos(a)
    bz = np.array([math.sin(a), 0.0, math.cos(a)])
    bx = np.array([math.cos(a), 0.0, -math.sin(a)])
    by = np.array([0.0, 1.0, 0.0])
    return np.column_stack([bx, by, bz]), np.array([V, 0.0, 0.0])

def find_autorotation_coll(R_hub, omega, wind):
    bz = R_hub[:, 2]
    def q(c):
        f = aero.compute_forces(c, 0.0, 0.0, R_hub, np.zeros(3), omega, wind, t=10.0)
        return float(np.dot(f[3:], bz))
    # Try two brackets: near-zero and wider
    for lo, hi in [(-0.08, 0.08), (-0.15, 0.15)]:
        if q(lo) * q(hi) < 0:
            for _ in range(60):
                mid = 0.5*(lo+hi)
                if q(lo)*q(mid) <= 0: hi = mid
                else: lo = mid
            return 0.5*(lo+hi)
    return None

def ct_at(R_hub, omega, wind, coll):
    bz = R_hub[:, 2]
    f  = aero.compute_forces(coll, 0.0, 0.0, R_hub, np.zeros(3), omega, wind, t=10.0)
    T  = float(np.dot(f[:3], bz))
    return T / (RHO * A * (omega * R_TIP)**2)

# mu at which reversed-flow boundary hits root cutout
mu_crit = R_ROOT / R_TIP   # = 0.073

MU_SCAN = [0.00, 0.05, 0.073, 0.10, 0.15, 0.20, 0.21, 0.25, 0.307, 0.341, 0.40, 0.512, 0.55]

print(f"PCA-2 BEM domain table  (shaft angle={AS_DEG}deg, ~{RPM} RPM, R={R_TIP}m, r_root={R_ROOT}m)")
print(f"Reversed-flow enters blade strip range at mu = r_root/R = {mu_crit:.3f}")
print()
print(f"{'mu':>6}  {'rev_flow':>10}  {'coll_deg':>9}  {'CT_sim':>9}  {'CT_exp':>9}  {'ratio':>7}  {'sign':>5}  domain")
print("-" * 85)

for mu in MU_SCAN:
    R_hub, wind = make_state(mu, AS_DEG)
    coll = find_autorotation_coll(R_hub, OMEGA, wind)

    rev_r   = max(0.0, mu * R_TIP - R_ROOT)   # reversed-flow depth into blade [m]
    blade_span = R_TIP - R_ROOT
    rev_frac = rev_r / blade_span              # fraction of active blade in reversal

    if coll is None:
        ct_s = float("nan")
        coll_str = "n/a"
    else:
        ct_s = ct_at(R_hub, OMEGA, wind, coll)
        coll_str = f"{math.degrees(coll):>9.2f}"

    # Nearest experimental CT (if within 0.05 of a known point)
    ct_e = None
    for mu_exp, ct_exp in EXP.items():
        if abs(mu - mu_exp) < 0.005:
            ct_e = ct_exp

    if math.isnan(ct_s):
        ratio_str = "n/a"
        sign_str  = "n/a"
    elif ct_e is not None:
        ratio = ct_s / ct_e
        ratio_str = f"{ratio:>7.3f}"
        sign_str  = "OK" if ct_s > 0 else "WRONG"
    else:
        ratio_str = "---"
        sign_str  = "OK" if ct_s > 0 else "WRONG"

    if rev_frac == 0:
        domain = "VALID  — no reversed flow in blade"
    elif rev_frac < 0.10:
        domain = f"MARGINAL — {rev_frac*100:.0f}% blade in reversal"
    else:
        domain = f"INVALID — {rev_frac*100:.0f}% blade in reversal"

    ct_s_str = f"{ct_s:>9.6f}" if not math.isnan(ct_s) else "      n/a"
    ct_e_str = f"{ct_e:>9.6f}" if ct_e is not None else "      ---"

    print(f"{mu:>6.3f}  {rev_r:>6.2f}m/{rev_frac*100:>3.0f}%  {coll_str}  {ct_s_str}  {ct_e_str}  {ratio_str:>7}  {sign_str:>5}  {domain}")

print()
print("Beaupoil operating envelope (R=2.5m, r_root=0.5m, mu_crit=0.200):")
print(f"{'mu':>6}  {'rev_flow':>10}  {'domain'}")
print("-" * 55)
rotor_b = rd.load("beaupoil_2026")
aero_b  = create_aero(rotor_b)
R_B   = 2.5
R_RB  = 0.5
OM_B  = 20.0
mu_crit_b = R_RB / R_B
for mu in [0.05, 0.10, 0.15, 0.20, 0.25, 0.30]:
    rev_r  = max(0.0, mu * R_B - R_RB)
    rev_frac = rev_r / (R_B - R_RB)
    if rev_frac == 0:
        domain = "VALID  — no reversed flow in blade"
    elif rev_frac < 0.10:
        domain = f"MARGINAL — {rev_frac*100:.0f}% blade in reversal"
    else:
        domain = f"INVALID — {rev_frac*100:.0f}% blade in reversal"
    print(f"{mu:>6.3f}  {rev_r:>6.2f}m/{rev_frac*100:>3.0f}%  {domain}")
