"""
compare_aero_models.py — PetersHeBEM vs SkewedWakeBEM across the full tilt envelope.

Sweeps tilt angle from 0° (pure autorotation, axial inflow) to 90° (hover,
in-plane inflow) and plots four panels:

  1. Thrust vs tilt — primary divergence between models
  2. Induced velocity v_i vs tilt — the root cause of the divergence
  3. Glauert universal inflow diagram — operating envelope on (μ_z, μ, λ_i) axes
  4. CT vs in-plane advance ratio μ — standard helicopter performance chart

Run:
    python simulation/analysis/compare_aero_models.py
"""

import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")          # headless; swap to TkAgg/Qt5Agg for interactive
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
# Sweep parameters
# ---------------------------------------------------------------------------
TILT_DEG   = np.arange(0, 91, 5)       # 0° (autorotation) → 90° (hover)
COL_RAD    = -0.10                      # moderate collective, positive thrust for both
OMEGA      = 28.0                       # rad/s — nominal RAWES spin rate
WIND_SPEED = 10.0                       # m/s
WIND_NED   = np.array([0.0, WIND_SPEED, 0.0])   # East
T_PAST_RAMP = 10.0


def _body_z(tilt_deg: float) -> np.ndarray:
    """Disk normal: rotated tilt_deg from wind direction (East) toward Down."""
    t = math.radians(tilt_deg)
    return np.array([0.0, math.cos(t), -math.sin(t)])


def _run_sweep(model_factory):
    """Return arrays (tilt_deg, T, v_i, v_axial, v_inplane, CT, mu_z, mu, lambda_i)."""
    rotor = rd.default()
    R = rotor.aero_kwargs()["r_tip"]
    RHO = float(rotor.aero_kwargs()["rho"])
    A   = math.pi * R**2

    results = []
    for td in TILT_DEG:
        bz = _body_z(float(td))
        R_hub = build_orb_frame(bz)
        model = model_factory(rotor)
        model.compute_forces(
            collective_rad=COL_RAD,
            tilt_lon=0.0, tilt_lat=0.0,
            R_hub=R_hub,
            v_hub_world=np.zeros(3),
            omega_rotor=OMEGA,
            wind_world=WIND_NED,
            t=T_PAST_RAMP,
        )
        T   = model.last_T
        v_i = model.last_v_i
        v_ax   = model.last_v_axial
        v_inp  = model.last_v_inplane

        tip_speed = OMEGA * R
        CT      = T / (RHO * A * tip_speed**2) if T > 0 else 0.0
        mu_z    = v_ax   / tip_speed
        mu      = v_inp  / tip_speed
        lambda_i = v_i  / tip_speed

        results.append((td, T, v_i, v_ax, v_inp, CT, mu_z, mu, lambda_i))

    return np.array(results)


# ---------------------------------------------------------------------------
# Run both models
# ---------------------------------------------------------------------------
print("Running PetersHeBEM sweep...")
G = _run_sweep(lambda r: PetersHeBEM(r))

print("Running SkewedWakeBEM sweep...")
S = _run_sweep(lambda r: create_aero(r, model="skewed_wake_numpy"))

# Unpack columns
td   = G[:, 0]
T_g  = G[:, 1];  T_s  = S[:, 1]
vi_g = G[:, 2];  vi_s = S[:, 2]
CT_g = G[:, 5];  CT_s = S[:, 5]
mu_z_g = G[:, 6]; mu_z_s = S[:, 6]
mu_g   = G[:, 7]; mu_s   = S[:, 7]
li_g   = G[:, 8]; li_s   = S[:, 8]

ratio = np.where(T_s > 1.0, T_g / T_s, np.nan)

# ---------------------------------------------------------------------------
# Colours and regime shading helpers
# ---------------------------------------------------------------------------
C_GL  = "#1f77b4"    # Glauert  — blue
C_SW  = "#d62728"    # Skewed   — red
C_RAT = "#2ca02c"    # ratio    — green

REGIME_AUTOROT = dict(color="#e8f4f8", label="autorotation\n(Coleman valid)")
REGIME_TRANSIT = dict(color="#fef9e7", label="transition")
REGIME_HOVER   = dict(color="#f9ebea", label="hover\n(Glauert valid)")

def shade_regimes(ax):
    ax.axvspan( 0, 55, alpha=0.40, **{k: v for k, v in REGIME_AUTOROT.items() if k != "label"})
    ax.axvspan(55, 75, alpha=0.40, **{k: v for k, v in REGIME_TRANSIT.items() if k != "label"})
    ax.axvspan(75, 90, alpha=0.40, **{k: v for k, v in REGIME_HOVER.items()   if k != "label"})


# ---------------------------------------------------------------------------
# Figure — 2 × 2
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(2, 2, figsize=(13, 10))
fig.suptitle(
    f"PetersHeBEM vs SkewedWakeBEM  |  col={COL_RAD:.2f} rad, "
    f"ω={OMEGA:.0f} rad/s, wind={WIND_SPEED:.0f} m/s East",
    fontsize=13, fontweight="bold",
)

# ── Panel 1: Thrust vs tilt ─────────────────────────────────────────────────
ax = axes[0, 0]
shade_regimes(ax)
ax.plot(td, T_g, color=C_GL, lw=2,   label="PetersHeBEM")
ax.plot(td, T_s, color=C_SW, lw=2,   label="SkewedWakeBEM")
ax2 = ax.twinx()
ax2.plot(td, ratio, color=C_RAT, lw=1.5, ls="--", label="ratio G/S")
ax2.axhline(1.0, color=C_RAT, lw=0.7, ls=":")
ax2.set_ylabel("Glauert / Skewed ratio", color=C_RAT, fontsize=9)
ax2.tick_params(axis="y", labelcolor=C_RAT)
ax2.set_ylim(0, max(np.nanmax(ratio) * 1.1, 2.0))

ax.set_xlabel("Disk tilt from wind [°]")
ax.set_ylabel("Axial thrust T [N]")
ax.set_title("1. Thrust vs tilt angle", fontweight="bold")
ax.set_xlim(0, 90)
ax.axhline(0, color="k", lw=0.5)
ax.axvline(55, color="gray", lw=0.8, ls=":")
ax.axvline(75, color="gray", lw=0.8, ls=":")
ax.text( 27, 0.03, "autorotation\n(Coleman valid)",
         transform=ax.get_xaxis_transform(), ha="center", va="bottom", fontsize=7.5, color="#555")
ax.text( 65, 0.03, "transition",
         transform=ax.get_xaxis_transform(), ha="center", va="bottom", fontsize=7.5, color="#555")
ax.text( 83, 0.03, "hover\n(Glauert valid)",
         transform=ax.get_xaxis_transform(), ha="center", va="bottom", fontsize=7.5, color="#555")

lines = [
    Line2D([0], [0], color=C_GL, lw=2),
    Line2D([0], [0], color=C_SW, lw=2),
    Line2D([0], [0], color=C_RAT, lw=1.5, ls="--"),
]
ax.legend(lines, ["PetersHeBEM", "SkewedWakeBEM", "ratio G/S"], fontsize=8, loc="lower left")

# ── Panel 2: Induced velocity vs tilt ───────────────────────────────────────
ax = axes[0, 1]
shade_regimes(ax)
ax.plot(td, vi_g, color=C_GL, lw=2, label="PetersHeBEM  v_i")
ax.plot(td, vi_s, color=C_SW, lw=2, label="SkewedWakeBEM  v_i₀")
ax.axvline(55, color="gray", lw=0.8, ls=":")
ax.axvline(75, color="gray", lw=0.8, ls=":")
ax.set_xlabel("Disk tilt from wind [°]")
ax.set_ylabel("Induced velocity v_i  [m/s]")
ax.set_title("2. Induced velocity — root cause of divergence", fontweight="bold")
ax.set_xlim(0, 90)
ax.legend(fontsize=8)
ax.text(0.97, 0.95,
        "Glauert: uniform (min power)\nColeman: non-uniform (higher v_i → less T)",
        transform=ax.transAxes, ha="right", va="top", fontsize=7.5,
        bbox=dict(fc="white", ec="gray", alpha=0.7, pad=3))

# ── Panel 3: Glauert universal inflow diagram ────────────────────────────────
# x-axis: μ_z (axial advance ratio), y-axis: λ_i (induced inflow ratio)
# Overlay the theoretical Glauert curve and both models' operating points.
ax = axes[1, 0]

# Theoretical Glauert curve at fixed CT (use mean of measured CT values)
CT_ref = float(np.nanmean(CT_g[CT_g > 0]))
mu_z_line = np.linspace(-0.3, 0.5, 300)
lambda_theory = []
vi0 = 0.05
for mz in mu_z_line:
    li = vi0
    for _ in range(80):
        denom = math.sqrt(max(1e-9, (mz + li)**2))
        li_new = CT_ref / (2.0 * denom)
        li = 0.4 * li_new + 0.6 * li
    lambda_theory.append(li)
lambda_theory = np.array(lambda_theory)

ax.plot(mu_z_line, lambda_theory, color="gray", lw=1.2, ls="--",
        label=f"Glauert curve (CT≈{CT_ref:.4f})")

# Operating points coloured by tilt angle
sc_g = ax.scatter(mu_z_g, li_g, c=td, cmap="Blues_r", s=40, zorder=5,
                  vmin=0, vmax=90, edgecolors=C_GL, lw=0.8, label="PetersHeBEM pts")
sc_s = ax.scatter(mu_z_s, li_s, c=td, cmap="Reds_r",  s=40, zorder=5,
                  vmin=0, vmax=90, edgecolors=C_SW, lw=0.8, marker="s",
                  label="SkewedWakeBEM pts")

# Annotate tilt=0 and tilt=90
for idx, label in [(0, "0°"), (-1, "90°")]:
    ax.annotate(f"tilt={label}", xy=(mu_z_g[idx], li_g[idx]),
                xytext=(8, 4), textcoords="offset points", fontsize=7.5, color=C_GL)

ax.set_xlabel("Axial advance ratio  μ_z = v_axial / (Ω·R)")
ax.set_ylabel("Induced inflow ratio  λ_i = v_i / (Ω·R)")
ax.set_title("3. Glauert universal inflow diagram", fontweight="bold")
ax.axvline(0, color="k", lw=0.5)
ax.axhline(0, color="k", lw=0.5)
cb = fig.colorbar(sc_g, ax=ax, shrink=0.7, pad=0.01)
cb.set_label("Tilt angle [°]", fontsize=8)
ax.legend(fontsize=7.5, loc="upper right")
ax.text(0.02, 0.05, "← axial descent        axial climb →",
        transform=ax.transAxes, fontsize=7, color="gray", ha="left")

# ── Panel 4: CT vs in-plane advance ratio μ ─────────────────────────────────
ax = axes[1, 1]

sc_g2 = ax.scatter(mu_g, CT_g, c=td, cmap="Blues_r", s=50, zorder=5,
                   vmin=0, vmax=90, edgecolors=C_GL, lw=0.8, label="PetersHeBEM")
sc_s2 = ax.scatter(mu_s, CT_s, c=td, cmap="Reds_r",  s=50, zorder=5,
                   vmin=0, vmax=90, edgecolors=C_SW, lw=0.8, marker="s",
                   label="SkewedWakeBEM")

# Connect corresponding tilt angles with light arrows
for i in range(0, len(td), 3):
    ax.annotate("", xy=(mu_g[i], CT_g[i]), xytext=(mu_s[i], CT_s[i]),
                arrowprops=dict(arrowstyle="-", color="gray", lw=0.6, alpha=0.5))

# Annotate tilt=0 and tilt=90
for idx, label in [(0, "tilt=0°"), (-1, "tilt=90°")]:
    ax.annotate(label, xy=(mu_g[idx], CT_g[idx]),
                xytext=(8, 4), textcoords="offset points", fontsize=7.5, color=C_GL)
    ax.annotate(label, xy=(mu_s[idx], CT_s[idx]),
                xytext=(8, -10), textcoords="offset points", fontsize=7.5, color=C_SW)

ax.set_xlabel("In-plane advance ratio  μ = v_inplane / (Ω·R)")
ax.set_ylabel("Thrust coefficient  CT = T / (ρ·A·(Ω·R)²)")
ax.set_title("4. CT vs advance ratio  (standard helicopter chart)", fontweight="bold")
ax.axhline(0, color="k", lw=0.5)

cb2 = fig.colorbar(sc_g2, ax=ax, shrink=0.7, pad=0.01)
cb2.set_label("Tilt angle [°]", fontsize=8)
ax.legend(fontsize=7.5, loc="upper left")
ax.text(0.97, 0.05,
        "Each point is one tilt angle.\n"
        "Arrows connect same tilt between models.\n"
        "Tilt 0° = axial (μ=0); tilt 90° = in-plane (μ max).",
        transform=ax.transAxes, ha="right", va="bottom", fontsize=7,
        bbox=dict(fc="white", ec="gray", alpha=0.7, pad=3))

# ---------------------------------------------------------------------------
# Save
# ---------------------------------------------------------------------------
plt.tight_layout()
out = Path(__file__).parent / "compare_aero_models.png"
fig.savefig(out, dpi=150, bbox_inches="tight")
print(f"Saved: {out}")
