"""
compare_aero_beaupoil.py -- PetersHeBEM vs SkewedWakeBEM on the beaupoil_2026 rotor.

Two figures:
  Fig 1 -- Tilt sweep 0-90deg (autorotation -> hover) at V=10 m/s, omega=20.15 rad/s
           4 panels: Thrust, v_i, Glauert diagram, CT vs mu
           RAWES regime bands: reel-out (xi=20-65deg), reel-in (xi=65-80deg),
           landing (xi=80-90deg)

  Fig 2 -- Wind speed sweep 5-20 m/s at two fixed tilts:
           xi=45deg (mid reel-out), xi=70deg (reel-in)
           2 panels per tilt: Thrust vs V, v_i vs V

Run:
    python simulation/analysis/compare_aero_beaupoil.py
"""

import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

_SIM_DIR  = Path(__file__).resolve().parents[1]
_AERO_DIR = _SIM_DIR / "aero"
for _p in [str(_SIM_DIR), str(_AERO_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import rotor_definition as rd
from frames import build_orb_frame
from aero import create_aero, PetersHeBEM

# ---------------------------------------------------------------------------
# Beaupoil rotor parameters
# ---------------------------------------------------------------------------
rotor   = rd.default()          # beaupoil_2026
kw      = rotor.aero_kwargs()
R_TIP   = float(kw["r_tip"])    # 2.5 m
R_ROOT  = float(kw["r_root"])   # 0.5 m
RHO     = float(kw["rho"])      # 1.22 kg/m3
A_DISK  = math.pi * (R_TIP**2 - R_ROOT**2)

OMEGA       = 20.15             # rad/s  (omega_eq at V=10 m/s)
TIP_SPEED   = OMEGA * R_TIP    # ~50.4 m/s
COL_RAD     = -0.10            # moderate collective (reel-out operating point)
T_RUN       = 10.0             # past ramp

# ---------------------------------------------------------------------------
# Regime definitions
# ---------------------------------------------------------------------------
REGIMES = [
    (0,  20,  "#f0f0f0", "free spin\n(no tether)"),
    (20, 65,  "#d4edda", "reel-out\n(xi=20-65deg)"),
    (65, 80,  "#fff3cd", "reel-in\n(xi=65-80deg)"),
    (80, 90,  "#f8d7da", "landing\n(xi=80-90deg)"),
]

def shade_regimes(ax, show_labels=True):
    for lo, hi, color, label in REGIMES:
        ax.axvspan(lo, hi, alpha=0.35, color=color)
        if show_labels:
            mid = (lo + hi) / 2
            ax.text(mid, 0.97, label, transform=ax.get_xaxis_transform(),
                    ha="center", va="top", fontsize=6.5, color="#444",
                    multialignment="center")


# ---------------------------------------------------------------------------
# Sweep helpers
# ---------------------------------------------------------------------------
def _disk_normal(tilt_deg):
    """Disk normal at tilt_deg from wind (East) toward Down."""
    t = math.radians(tilt_deg)
    return np.array([0.0, math.cos(t), -math.sin(t)])


def tilt_sweep(model_factory, tilt_arr, wind_speed, omega):
    """Return (N,9) array: tilt, T, v_i, v_ax, v_inp, CT, mu_z, mu, lambda_i."""
    rows = []
    wind = np.array([0.0, wind_speed, 0.0])
    for td in tilt_arr:
        bz    = _disk_normal(float(td))
        R_hub = build_orb_frame(bz)
        mdl   = model_factory()
        mdl.compute_forces(COL_RAD, 0.0, 0.0, R_hub, np.zeros(3), omega, wind, T_RUN)
        T   = mdl.last_T
        v_i = mdl.last_v_i
        v_ax  = mdl.last_v_axial
        v_inp = mdl.last_v_inplane
        CT      = T / (RHO * A_DISK * TIP_SPEED**2) if TIP_SPEED > 0 else 0.0
        mu_z    = v_ax  / TIP_SPEED
        mu      = v_inp / TIP_SPEED
        lam_i   = v_i   / TIP_SPEED
        rows.append((td, T, v_i, v_ax, v_inp, CT, mu_z, mu, lam_i))
    return np.array(rows)


def wind_sweep(model_factory, tilt_deg, wind_arr, omega):
    """Return (N,3) array: V, T, v_i."""
    bz    = _disk_normal(float(tilt_deg))
    R_hub = build_orb_frame(bz)
    rows  = []
    for V in wind_arr:
        wind = np.array([0.0, V, 0.0])
        mdl  = model_factory()
        mdl.compute_forces(COL_RAD, 0.0, 0.0, R_hub, np.zeros(3), omega, wind, T_RUN)
        rows.append((V, mdl.last_T, mdl.last_v_i))
    return np.array(rows)


# ---------------------------------------------------------------------------
# Run sweeps
# ---------------------------------------------------------------------------
TILT_DEG  = np.arange(0, 91, 5)
WIND_DEG  = np.linspace(5, 20, 16)
PH_fac    = lambda: PetersHeBEM(rotor)
SW_fac    = lambda: create_aero(rotor, model="skewed_wake_numpy")

print("Fig 1 -- tilt sweep (PetersHe)...")
G = tilt_sweep(PH_fac, TILT_DEG, 10.0, OMEGA)
print("Fig 1 -- tilt sweep (SkewedWake)...")
S = tilt_sweep(SW_fac, TILT_DEG, 10.0, OMEGA)

print("Fig 2 -- wind sweep xi=45deg (PetersHe)...")
G45 = wind_sweep(PH_fac, 45, WIND_DEG, OMEGA)
print("Fig 2 -- wind sweep xi=45deg (SkewedWake)...")
S45 = wind_sweep(SW_fac, 45, WIND_DEG, OMEGA)

print("Fig 2 -- wind sweep xi=70deg (PetersHe)...")
G70 = wind_sweep(PH_fac, 70, WIND_DEG, OMEGA)
print("Fig 2 -- wind sweep xi=70deg (SkewedWake)...")
S70 = wind_sweep(SW_fac, 70, WIND_DEG, OMEGA)

# Unpack tilt sweep
td    = G[:, 0]
T_g   = G[:, 1];  T_s   = S[:, 1]
vi_g  = G[:, 2];  vi_s  = S[:, 2]
CT_g  = G[:, 5];  CT_s  = S[:, 5]
mu_z_g = G[:, 6]; mu_z_s = S[:, 6]
mu_g   = G[:, 7]; mu_s   = S[:, 7]
li_g   = G[:, 8]; li_s   = S[:, 8]

ratio = np.where(T_s > 1.0, T_g / T_s, np.nan)

C_PH  = "#1f77b4"   # PetersHe -- blue
C_SW  = "#d62728"   # SkewedWake -- red
C_RAT = "#2ca02c"   # ratio -- green


# ===========================================================================
# Figure 1 -- Tilt sweep (4 panels)
# ===========================================================================
fig1, axes1 = plt.subplots(2, 2, figsize=(13, 10))
fig1.suptitle(
    f"PetersHeBEM vs SkewedWakeBEM -- beaupoil_2026\n"
    f"col={COL_RAD:.2f} rad, omega={OMEGA:.1f} rad/s, V_wind=10 m/s East",
    fontsize=12, fontweight="bold",
)

# --- Panel 1: Thrust vs tilt ---
ax = axes1[0, 0]
shade_regimes(ax)
ax.plot(td, T_g, color=C_PH, lw=2, label="PetersHe")
ax.plot(td, T_s, color=C_SW, lw=2, label="SkewedWake")
ax2 = ax.twinx()
ax2.plot(td, ratio, color=C_RAT, lw=1.5, ls="--", label="ratio PH/SW")
ax2.axhline(1.0, color=C_RAT, lw=0.7, ls=":")
ax2.set_ylabel("PetersHe / SkewedWake", color=C_RAT, fontsize=9)
ax2.tick_params(axis="y", labelcolor=C_RAT)
ax2.set_ylim(0, max(np.nanmax(ratio) * 1.2, 2.0))
ax.set_xlabel("Disk tilt from wind [deg]")
ax.set_ylabel("Thrust T [N]")
ax.set_title("1. Thrust vs tilt angle", fontweight="bold")
ax.set_xlim(0, 90)
ax.axhline(0, color="k", lw=0.5)
lines = [Line2D([0],[0], color=C_PH, lw=2),
         Line2D([0],[0], color=C_SW, lw=2),
         Line2D([0],[0], color=C_RAT, lw=1.5, ls="--")]
ax.legend(lines, ["PetersHe", "SkewedWake", "ratio PH/SW"], fontsize=8, loc="upper left")

# --- Panel 2: v_i vs tilt ---
ax = axes1[0, 1]
shade_regimes(ax)
ax.plot(td, vi_g, color=C_PH, lw=2, label="PetersHe v_i")
ax.plot(td, vi_s, color=C_SW, lw=2, label="SkewedWake v_i0")
ax.set_xlabel("Disk tilt from wind [deg]")
ax.set_ylabel("Induced velocity v_i [m/s]")
ax.set_title("2. Induced velocity", fontweight="bold")
ax.set_xlim(0, 90)
ax.legend(fontsize=8)
ax.text(0.97, 0.50,
        "Glauert: uniform (min power)\nColeman: non-uniform (higher v_i at tilt)",
        transform=ax.transAxes, ha="right", va="center", fontsize=7.5,
        bbox=dict(fc="white", ec="gray", alpha=0.7, pad=3))

# --- Panel 3: Glauert universal inflow diagram ---
ax = axes1[1, 0]
CT_ref = float(np.nanmean(CT_g[CT_g > 0]))
mu_z_line = np.linspace(-0.3, 0.5, 300)
lambda_theory = []
vi0 = 0.05
for mz in mu_z_line:
    li = vi0
    for _ in range(80):
        denom = math.sqrt(max(1e-9, (mz + li)**2))
        li = 0.4 * CT_ref / (2.0 * denom) + 0.6 * li
    lambda_theory.append(li)
lambda_theory = np.array(lambda_theory)
ax.plot(mu_z_line, lambda_theory, color="gray", lw=1.2, ls="--",
        label=f"Glauert curve (CT~{CT_ref:.4f})")
sc_g = ax.scatter(mu_z_g, li_g, c=td, cmap="Blues_r", s=45, zorder=5,
                  vmin=0, vmax=90, edgecolors=C_PH, lw=0.8, label="PetersHe")
sc_s = ax.scatter(mu_z_s, li_s, c=td, cmap="Reds_r",  s=45, zorder=5,
                  vmin=0, vmax=90, edgecolors=C_SW, lw=0.8, marker="s",
                  label="SkewedWake")
for idx, lbl in [(0, "0deg"), (-1, "90deg")]:
    ax.annotate(f"tilt={lbl}", xy=(mu_z_g[idx], li_g[idx]),
                xytext=(8, 4), textcoords="offset points", fontsize=7.5, color=C_PH)
ax.set_xlabel("Axial advance ratio  mu_z = v_axial / (Omega*R)")
ax.set_ylabel("Induced inflow ratio  lambda_i = v_i / (Omega*R)")
ax.set_title("3. Glauert universal inflow diagram", fontweight="bold")
ax.axvline(0, color="k", lw=0.5)
ax.axhline(0, color="k", lw=0.5)
cb = fig1.colorbar(sc_g, ax=ax, shrink=0.7, pad=0.01)
cb.set_label("Tilt angle [deg]", fontsize=8)
ax.legend(fontsize=7.5, loc="upper right")

# --- Panel 4: CT vs mu ---
ax = axes1[1, 1]
sc_g2 = ax.scatter(mu_g, CT_g, c=td, cmap="Blues_r", s=50, zorder=5,
                   vmin=0, vmax=90, edgecolors=C_PH, lw=0.8, label="PetersHe")
sc_s2 = ax.scatter(mu_s, CT_s, c=td, cmap="Reds_r",  s=50, zorder=5,
                   vmin=0, vmax=90, edgecolors=C_SW, lw=0.8, marker="s",
                   label="SkewedWake")
for i in range(0, len(td), 3):
    ax.annotate("", xy=(mu_g[i], CT_g[i]), xytext=(mu_s[i], CT_s[i]),
                arrowprops=dict(arrowstyle="-", color="gray", lw=0.6, alpha=0.5))
for idx, lbl in [(0, "tilt=0deg"), (-1, "tilt=90deg")]:
    ax.annotate(lbl, xy=(mu_g[idx], CT_g[idx]),
                xytext=(8,  4), textcoords="offset points", fontsize=7.5, color=C_PH)
    ax.annotate(lbl, xy=(mu_s[idx], CT_s[idx]),
                xytext=(8, -10), textcoords="offset points", fontsize=7.5, color=C_SW)
ax.set_xlabel("In-plane advance ratio  mu = v_inplane / (Omega*R)")
ax.set_ylabel("Thrust coefficient  CT")
ax.set_title("4. CT vs advance ratio", fontweight="bold")
ax.axhline(0, color="k", lw=0.5)
cb2 = fig1.colorbar(sc_g2, ax=ax, shrink=0.7, pad=0.01)
cb2.set_label("Tilt angle [deg]", fontsize=8)
ax.legend(fontsize=7.5, loc="upper left")

# RAWES operating envelope annotation
ax.text(0.97, 0.08,
        "RAWES operating envelope:\n"
        "reel-out: tilt=20-65deg\n"
        "reel-in:  tilt=65-80deg",
        transform=ax.transAxes, ha="right", va="bottom", fontsize=7,
        bbox=dict(fc="white", ec="gray", alpha=0.7, pad=3))

plt.tight_layout()
out1 = Path(__file__).parent / "compare_aero_beaupoil_tilt.png"
fig1.savefig(out1, dpi=150, bbox_inches="tight")
print(f"Saved: {out1}")


# ===========================================================================
# Figure 2 -- Wind speed sweep at xi=45 and xi=70
# ===========================================================================
fig2, axes2 = plt.subplots(2, 2, figsize=(12, 8))
fig2.suptitle(
    f"PetersHeBEM vs SkewedWakeBEM -- beaupoil_2026  Wind speed sweep\n"
    f"col={COL_RAD:.2f} rad, omega={OMEGA:.1f} rad/s",
    fontsize=12, fontweight="bold",
)

for row, (tilt_deg, G_ws, S_ws, regime_label) in enumerate([
    (45, G45, S45, "reel-out (xi=45deg)"),
    (70, G70, S70, "reel-in  (xi=70deg)"),
]):
    V   = G_ws[:, 0]
    Tg  = G_ws[:, 1];  Ts  = S_ws[:, 1]
    vig = G_ws[:, 2];  vis = S_ws[:, 2]

    # Thrust vs V
    ax = axes2[row, 0]
    ax.plot(V, Tg, color=C_PH, lw=2, label="PetersHe")
    ax.plot(V, Ts, color=C_SW, lw=2, label="SkewedWake")
    ax.axhline(0, color="k", lw=0.5)
    ax.axvline(10, color="gray", lw=0.8, ls=":", label="V=10 m/s (nominal)")
    ax.set_xlabel("Wind speed V [m/s]")
    ax.set_ylabel("Thrust T [N]")
    ax.set_title(f"Thrust vs wind -- {regime_label}", fontweight="bold")
    ax.legend(fontsize=8)
    ax.text(0.03, 0.97,
            f"tilt = {tilt_deg} deg from wind\nomega = {OMEGA:.1f} rad/s",
            transform=ax.transAxes, ha="left", va="top", fontsize=8,
            bbox=dict(fc="white", ec="gray", alpha=0.7, pad=3))

    # v_i vs V
    ax = axes2[row, 1]
    ax.plot(V, vig, color=C_PH, lw=2, label="PetersHe v_i")
    ax.plot(V, vis, color=C_SW, lw=2, label="SkewedWake v_i0")
    ax.axhline(0, color="k", lw=0.5)
    ax.axvline(10, color="gray", lw=0.8, ls=":", label="V=10 m/s (nominal)")
    ax.set_xlabel("Wind speed V [m/s]")
    ax.set_ylabel("Induced velocity v_i [m/s]")
    ax.set_title(f"Induced velocity vs wind -- {regime_label}", fontweight="bold")
    ax.legend(fontsize=8)

    # Difference annotation
    diff_T  = np.abs(Tg - Ts)
    diff_vi = np.abs(vig - vis)
    idx10 = np.argmin(np.abs(V - 10.0))
    ax.text(0.97, 0.97,
            f"At V=10: |PH-SW|={diff_T[idx10]:.1f} N T, {diff_vi[idx10]:.2f} m/s v_i",
            transform=axes2[row, 1].transAxes, ha="right", va="top", fontsize=7.5,
            bbox=dict(fc="lightyellow", ec="gray", alpha=0.9, pad=3))

plt.tight_layout()
out2 = Path(__file__).parent / "compare_aero_beaupoil_wind.png"
fig2.savefig(out2, dpi=150, bbox_inches="tight")
print(f"Saved: {out2}")
