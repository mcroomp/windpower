"""
plot_bem_rotor_c.py — BEM model predictions for NACA TR-552 Rotor C vs experimental data.

Runs our RotorAero BEM model for each (mu, pitch) combination using the shaft angles
from the NACA experimental data, converts to the NACA CL convention, and overlays
against the digitized measurements.

NACA CL convention (Wheatley & Bioletti 1936):
    CL_NACA = T / (rho * n^2 * D^4)
    where n = rev/s, D = rotor diameter [m]

Relationship to standard CT:
    CL_NACA = CT * pi^3/4  ≈  CT * 7.75
    (derived from CT = T / (rho * pi * R^2 * (Omega*R)^2))

Shaft angle convention (alpha_s from horizontal):
    body_z = [sin(alpha_s), 0, cos(alpha_s)]  in ENU (X=downwind, Z=up)
    alpha_s = 0: disk normal horizontal, axial propeller-mode flow
    alpha_s = 90: disk normal vertical, edgewise helicopter-like flow
    At mu=0.10, alpha_s≈58 (steeply tilted); at mu=0.90, alpha_s≈13 (nearly edge-on).
"""

import sys, math, csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "simulation"))

from aero import rotor_definition as rd
from aero import create_aero

# ---------------------------------------------------------------------------
# Rotor and model setup
# ---------------------------------------------------------------------------
rotor = rd.load("naca_tr_552_c")
aero  = create_aero(rotor, model="rotor")

RHO   = rotor.rho_kg_m3
R     = rotor.radius_m
D     = 2.0 * R                      # 3.05 m diameter
RPM   = 99.6
OMEGA = RPM * 2.0 * math.pi / 60.0  # 10.43 rad/s
N_RPS = RPM / 60.0                   # 1.66 rev/s

# NACA CL normalisation: CL_NACA = T / (rho * n^2 * D^4)
NORM_NACA = RHO * N_RPS**2 * D**4
print(f"Rotor C: R={R:.3f}m, D={D:.3f}m, RPM={RPM}, OmegaR={OMEGA*R:.2f} m/s")
print(f"NACA CL normalisation factor: {NORM_NACA:.1f} N  (CL = T / {NORM_NACA:.1f})")

# ---------------------------------------------------------------------------
# Shaft angle (alpha_s from horizontal) interpolated from NACA experimental data
# Columns: pitch=4.5° data from NACA TR-552 (represents the equilibrium tilt trajectory)
# ---------------------------------------------------------------------------
_MU_REF = np.array([0.06, 0.10, 0.15, 0.20, 0.30, 0.42, 0.50, 0.60, 0.75, 0.90])
_AS_REF = np.array([65,   58,   52,   46,   38,   30,   27,   22,   17,   13  ])  # degrees

def alpha_s_from_mu(mu: float) -> float:
    """Shaft angle [deg] interpolated from NACA TR-552 pitch=4.5° equilibrium data."""
    return float(np.interp(mu, _MU_REF, _AS_REF))

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_R_hub(alpha_s_deg: float) -> np.ndarray:
    """
    Build rotation matrix R_hub from shaft angle.
    body_z = [sin(a), 0, cos(a)]  (a = alpha_s from horizontal)
    body_y = [0, 1, 0]
    body_x = body_y × body_z
    """
    a  = math.radians(alpha_s_deg)
    bz = np.array([math.sin(a), 0.0, math.cos(a)])
    by = np.array([0.0, 1.0, 0.0])
    bx = np.cross(by, bz)
    return np.column_stack([bx, by, bz])


def compute_cl_naca_bem(mu: float, alpha_s_deg: float, pitch_deg: float) -> float:
    """
    Run BEM and return CL in NACA convention.
    T = dot(F_world[:3], body_z) — thrust along disk normal.
    CL_NACA = T / NORM_NACA.
    """
    R_hub = make_R_hub(alpha_s_deg)
    V     = mu * OMEGA * R                   # wind tunnel speed [m/s]
    wind  = np.array([V, 0.0, 0.0])
    coll  = math.radians(pitch_deg)

    # t=100 bypasses the aero ramp
    f = aero.compute_forces(coll, 0.0, 0.0, R_hub, np.zeros(3), OMEGA, wind, t=100.0)
    T = float(np.dot(f[:3], R_hub[:, 2]))
    return T / NORM_NACA


# ---------------------------------------------------------------------------
# Load digitized NACA CSV for Rotor C
# ---------------------------------------------------------------------------
def load_naca_cl() -> dict:
    """Load CL values for Rotor C from naca_tr_552_data.csv.
    Returns dict: pitch_deg -> sorted list of (mu, CL).
    """
    data = {}
    csv_path = HERE / "naca_tr_552_data.csv"
    with open(csv_path) as fp:
        for raw_line in fp:
            line = raw_line.split('#')[0].strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) < 5:
                continue
            try:
                rotor_id = parts[0]
                if rotor_id != 'C':
                    continue
                pitch = float(parts[2])
                mu    = float(parts[3])
                cl    = float(parts[4]) if parts[4] else float('nan')
                if not math.isnan(cl):
                    data.setdefault(pitch, []).append((mu, cl))
            except (ValueError, IndexError):
                continue
    # Sort each pitch curve by mu
    for p in data:
        data[p].sort(key=lambda x: x[0])
    return data

naca_cl = load_naca_cl()
print(f"\nNACA data loaded: pitch settings = {sorted(naca_cl.keys())}")

# ---------------------------------------------------------------------------
# Compute BEM curves
# ---------------------------------------------------------------------------
PITCHES = [4.5, 3.5, 3.0, 1.5, 0.0, -1.5, -3.0, -4.5]
MU_SCAN = np.linspace(0.08, 0.90, 70)

colors = plt.cm.RdYlGn(np.linspace(0.12, 0.88, len(PITCHES)))[::-1]

print("\nComputing BEM curves...")
bem_curves = {}
for pitch in PITCHES:
    mus, cls = [], []
    for mu in MU_SCAN:
        a_s = alpha_s_from_mu(mu)
        cl  = compute_cl_naca_bem(mu, a_s, pitch)
        mus.append(mu)
        cls.append(cl)
    bem_curves[pitch] = (np.array(mus), np.array(cls))
    mu_mid = 0.30
    cl_mid = compute_cl_naca_bem(mu_mid, alpha_s_from_mu(mu_mid), pitch)
    print(f"  pitch={pitch:+.1f}°  CL(mu=0.10)={compute_cl_naca_bem(0.10, alpha_s_from_mu(0.10), pitch):.3f}"
          f"  CL(mu=0.30)={cl_mid:.3f}"
          f"  CL(mu=0.60)={compute_cl_naca_bem(0.60, alpha_s_from_mu(0.60), pitch):.3f}")

# Also store NACA values at matched conditions for table
print("\n--- Point comparison (BEM vs NACA at exact test conditions) ---")
print(f"{'pitch':>7} {'mu':>6} {'alpha_s':>8} {'CL_BEM':>9} {'CL_NACA':>9} {'ratio':>7}")
print("-" * 55)
for pitch in [4.5, 3.5, 1.5, 0.0]:
    if pitch not in naca_cl:
        continue
    for mu, cl_exp in naca_cl[pitch]:
        if mu < 0.09 or mu > 0.91:
            continue
        a_s    = alpha_s_from_mu(mu)
        cl_bem = compute_cl_naca_bem(mu, a_s, pitch)
        ratio  = cl_bem / cl_exp if abs(cl_exp) > 0.001 else float('nan')
        print(f"{pitch:>7.1f} {mu:>6.2f} {a_s:>8.1f} {cl_bem:>9.3f} {cl_exp:>9.3f} {ratio:>7.2f}")

# ---------------------------------------------------------------------------
# Plot — three panels
# ---------------------------------------------------------------------------
fig = plt.figure(figsize=(18, 6))
gs  = fig.add_gridspec(1, 3, wspace=0.32)
ax_bem  = fig.add_subplot(gs[0])
ax_naca = fig.add_subplot(gs[1])
ax_both = fig.add_subplot(gs[2])

fig.suptitle(
    "NACA TR-552 Rotor C (NACA 4412) — BEM prediction vs digitised data\n"
    r"$C_L = T\,/\,(\rho\,n^2\,D^4)$   |   shaft angle $\alpha_s(\mu)$ from NACA equilibrium table",
    fontsize=11
)

Y_MIN, Y_MAX = -0.08, 0.35
X_MIN, X_MAX =  0.0,  0.92

def setup_ax(ax, title):
    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Y_MIN, Y_MAX)
    ax.set_xlabel(r"Advance ratio  $\mu$", fontsize=10)
    ax.set_ylabel(r"Lift coefficient  $C_L$", fontsize=10)
    ax.set_xticks(np.arange(0.0, 1.0, 0.1))
    ax.axhline(0, color='black', lw=0.8, zorder=2)
    ax.axvspan(0.10, 0.18, color='#90EE90', alpha=0.35, zorder=1, label='RAWES μ range')
    ax.grid(True, alpha=0.30)
    ax.set_title(title, fontsize=9)

setup_ax(ax_bem,  "BEM prediction (RotorAero)")
setup_ax(ax_naca, "Digitised NACA TR-552 data")
setup_ax(ax_both, "BEM (dashed) vs NACA (solid markers)")

# ── Left: BEM ────────────────────────────────────────────────────────────────
for i, pitch in enumerate(PITCHES):
    mus, cls = bem_curves[pitch]
    lw = 2.2 if abs(pitch) < 0.1 else 1.6
    ls = '--' if pitch < 0 else '-'
    ax_bem.plot(mus, cls, color=colors[i], lw=lw, ls=ls,
                label=f"{pitch:+.1f}°")

ax_bem.legend(title="Pitch setting", fontsize=7, title_fontsize=8,
              loc='upper right', ncol=2)
ax_bem.annotate(f"CL_alpha={rotor.CL_alpha_per_rad:.1f}/rad\nCL0={rotor.CL0:.2f}\nCD0={rotor.CD0:.3f}",
                xy=(0.55, 0.26), fontsize=7.5, color='#333333',
                bbox=dict(boxstyle='round,pad=0.3', fc='lightyellow', ec='gray', alpha=0.85))

# ── Middle: NACA data ─────────────────────────────────────────────────────────
naca_pitches_plot = [4.5, 3.5, 3.0, 1.5, 0.0, -1.5, -3.0, -4.5]
naca_colors = plt.cm.RdYlGn(np.linspace(0.12, 0.88, len(naca_pitches_plot)))[::-1]
for i, pitch in enumerate(naca_pitches_plot):
    if pitch not in naca_cl:
        continue
    pts = naca_cl[pitch]
    ax_naca.plot([p[0] for p in pts], [p[1] for p in pts],
                 color=naca_colors[i], lw=2.0, marker='o', ms=5,
                 ls='--' if pitch < 0 else '-',
                 label=f"{pitch:+.1f}°")

ax_naca.legend(title="Pitch setting", fontsize=7, title_fontsize=8,
               loc='upper right', ncol=2)

# ── Right: overlay ────────────────────────────────────────────────────────────
for i, pitch in enumerate(PITCHES):
    c = colors[i]
    # BEM dashed
    mus_b, cls_b = bem_curves[pitch]
    ax_both.plot(mus_b, cls_b, color=c, lw=1.5, ls='--', zorder=3)
    # NACA solid + markers (only main pitch settings)
    if pitch in naca_cl:
        pts = naca_cl[pitch]
        ax_both.plot([p[0] for p in pts], [p[1] for p in pts],
                     color=c, lw=2.0, marker='o', ms=4, ls='-', zorder=4)

# Legend proxies
from matplotlib.lines import Line2D
handles = [Line2D([0],[0], color='gray', lw=2, ls='--', label='BEM prediction'),
           Line2D([0],[0], color='gray', lw=2, ls='-', marker='o', ms=4, label='NACA TR-552')]
ax_both.legend(handles=handles, fontsize=8, loc='upper right')
ax_both.annotate("BEM tends to\noverpredict at μ>0.3\n(retreating-blade\nreversal not modelled)",
                 xy=(0.55, 0.10), fontsize=7.5, color='#333333',
                 bbox=dict(boxstyle='round,pad=0.3', fc='#fff8dc', ec='gray', alpha=0.85))

# Mark RAWES operating range on middle panel too
for ax in [ax_bem, ax_naca, ax_both]:
    ax.annotate("RAWES\nrange", xy=(0.14, Y_MAX - 0.04),
                fontsize=6.5, color='darkgreen', ha='center')

plt.tight_layout()
out = HERE / "bem_vs_naca_rotor_c.png"
fig.savefig(out, dpi=150, bbox_inches='tight')
plt.close(fig)
print(f"\nSaved: {out}")
