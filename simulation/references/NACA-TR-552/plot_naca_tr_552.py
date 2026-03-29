"""
plot_naca_tr_552.py — Recreate NACA-TR-552 performance charts from digitized data
and compare against original scanned page images.

Run from anywhere:
    python plot_naca_tr_552.py

Outputs (saved alongside this script):
    naca_4412_performance.png   — Rotor C (NACA 4412) CL / CT / L/D / alpha
    naca_comparison.png         — All four rotors compared at optimum pitch
    naca_comparison_page9.png   — Recreated chart next to original scan
"""

import sys
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import csv

HERE = Path(__file__).parent

# ---------------------------------------------------------------------------
# Load CSV (skip comment lines)
# ---------------------------------------------------------------------------

def load_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(
            (line for line in f if not line.lstrip().startswith("#")),
            fieldnames=["rotor","airfoil","pitch_deg","mu","CL","CT","LD","alpha_deg"]
        )
        for row in reader:
            try:
                rows.append({
                    "rotor":     row["rotor"].strip(),
                    "airfoil":   row["airfoil"].strip(),
                    "pitch":     float(row["pitch_deg"]),
                    "mu":        float(row["mu"]),
                    "CL":        float(row["CL"])   if row["CL"].strip()        else float("nan"),
                    "CT":        float(row["CT"])   if row["CT"].strip()        else float("nan"),
                    "LD":        float(row["LD"])   if row["LD"].strip()        else float("nan"),
                    "alpha":     float(row["alpha_deg"]) if row["alpha_deg"].strip() else float("nan"),
                })
            except (ValueError, TypeError):
                continue
    return rows

def select(rows, rotor=None, airfoil=None, pitch=None, mu_range=None):
    out = rows
    if rotor:   out = [r for r in out if r["rotor"]   == rotor]
    if airfoil: out = [r for r in out if r["airfoil"] == airfoil]
    if pitch is not None:
        out = [r for r in out if abs(r["pitch"] - pitch) < 0.01]
    if mu_range:
        out = [r for r in out if mu_range[0] <= r["mu"] <= mu_range[1]]
    return sorted(out, key=lambda r: r["mu"])

rows = load_csv(HERE / "naca_tr_552_data.csv")

# Beaupoil operating range
RAWES_MU_LO = 0.10
RAWES_MU_HI = 0.18
RAWES_PITCH = -5.7   # deg (Beaupoil equilibrium collective in degrees)

# Pitch settings in the dataset for Rotor C
C_PITCHES = [4.5, 3.5, 3.0, 1.5, 0.0, -1.5, -3.0, -4.5]
COLORS    = plt.cm.RdYlGn(np.linspace(0.15, 0.85, len(C_PITCHES)))[::-1]

# ---------------------------------------------------------------------------
# Figure 1 — Rotor C (NACA 4412) four-panel chart
# ---------------------------------------------------------------------------

fig, axes = plt.subplots(2, 2, figsize=(13, 10))
fig.suptitle(
    "NACA-TR-552  —  Rotor C (NACA 4412, cambered)  —  digitized from wind-tunnel data\n"
    "Wheatley & Bioletti 1936    |    shaded band = RAWES Beaupoil operating range (μ = 0.10–0.18)",
    fontsize=11
)

panels = [
    ("CL",    "Lift coefficient  $C_L$",       (0, 0.30),  axes[0, 0]),
    ("CT",    "Thrust coefficient  $C_T$",      (0, 0.012), axes[0, 1]),
    ("LD",    "Lift / Drag  $L/D$",             (0, 6.0),   axes[1, 0]),
    ("alpha", "Shaft angle of attack  α  [°]",  (0, 90),    axes[1, 1]),
]

for key, ylabel, ylim, ax in panels:
    ax.set_xlabel("Tip-speed ratio  μ  =  V / (ΩR)", fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.set_xlim(0, 0.95)
    ax.set_ylim(*ylim)
    ax.axvspan(RAWES_MU_LO, RAWES_MU_HI, color="#90EE90", alpha=0.35, label="RAWES range")
    ax.axvline(RAWES_MU_LO, color="green", lw=0.8, ls="--")
    ax.axvline(RAWES_MU_HI, color="green", lw=0.8, ls="--")
    ax.set_xticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
    ax.grid(True, alpha=0.35)

    for i, pitch in enumerate(C_PITCHES):
        data = select(rows, rotor="C", pitch=pitch)
        mus  = [d["mu"]     for d in data if not math.isnan(d[key])]
        vals = [d[key]      for d in data if not math.isnan(d[key])]
        if len(mus) < 2:
            continue
        label = f"{pitch:+.1f}°"
        lw    = 2.0 if abs(pitch) < 0.5 else 1.4
        ls    = "--" if pitch < 0 else "-"
        ax.plot(mus, vals, color=COLORS[i], lw=lw, ls=ls,
                marker="o", ms=3.5, label=label)

    # Mark RAWES pitch line on CL and CT panels
    if key in ("CL", "CT"):
        ax.axhline(0, color="black", lw=0.5)

    ax.legend(title="Pitch setting", fontsize=7.5, title_fontsize=8,
              loc="upper right", ncol=2)

# Add Beaupoil annotation to CL panel
axes[0, 0].annotate(
    f"Beaupoil\nμ ≈ 0.10–0.18\npitch ≈ {RAWES_PITCH}°",
    xy=(0.14, 0.04), xytext=(0.5, 0.08),
    arrowprops=dict(arrowstyle="->", color="darkgreen"),
    fontsize=8, color="darkgreen",
    bbox=dict(boxstyle="round,pad=0.3", fc="lightyellow", ec="green", alpha=0.8),
)

plt.tight_layout()
out1 = HERE / "naca_4412_performance.png"
fig.savefig(out1, dpi=150, bbox_inches="tight")
plt.close(fig)
print(f"Saved: {out1}")

# ---------------------------------------------------------------------------
# Figure 2 — All four rotors compared at optimum pitch (μ full range)
# ---------------------------------------------------------------------------

ROTOR_STYLES = {
    "A": dict(color="#1f77b4", ls="-",  lw=2.0, label="A — NACA 0012 (symmetric)"),
    "B": dict(color="#ff7f0e", ls="--", lw=2.0, label="B — NACA 0018 (symmetric, thick)"),
    "C": dict(color="#2ca02c", ls="-",  lw=2.5, label="C — NACA 4412 (cambered) ← closest to SG6042"),
    "D": dict(color="#d62728", ls="-.", lw=2.0, label="D — NACA 4418 (cambered, thick)"),
}

fig, axes = plt.subplots(1, 3, figsize=(15, 5))
fig.suptitle(
    "NACA-TR-552  —  All four rotors at pitch = 3.5°  (near-optimum for cambered sections)\n"
    "Shaded band = RAWES Beaupoil operating range (μ = 0.10–0.18)",
    fontsize=11
)

for ax, key, ylabel, ylim in [
    (axes[0], "CL", "Lift coefficient  $C_L$",  (0, 0.30)),
    (axes[1], "CT", "Thrust coefficient  $C_T$", (0, 0.012)),
    (axes[2], "LD", "Lift / Drag  $L/D$",        (0, 6.5)),
]:
    ax.set_xlabel("Tip-speed ratio  μ", fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.set_xlim(0, 0.95)
    ax.set_ylim(*ylim)
    ax.axvspan(RAWES_MU_LO, RAWES_MU_HI, color="#90EE90", alpha=0.35)
    ax.set_xticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
    ax.grid(True, alpha=0.35)

    for rotor, style in ROTOR_STYLES.items():
        data = select(rows, rotor=rotor, pitch=3.5, mu_range=(0, 3.5))
        if len(data) < 2:
            # fall back to 4.5° if 3.5 not enough points
            data = select(rows, rotor=rotor, pitch=4.5, mu_range=(0, 3.5))
        mus  = [d["mu"]  for d in data if not math.isnan(d[key])]
        vals = [d[key]   for d in data if not math.isnan(d[key])]
        if len(mus) < 2:
            continue
        ax.plot(mus, vals, marker="o", ms=4, **style)

    ax.legend(fontsize=8, loc="upper right")

plt.tight_layout()
out2 = HERE / "naca_comparison_all_rotors.png"
fig.savefig(out2, dpi=150, bbox_inches="tight")
plt.close(fig)
print(f"Saved: {out2}")

# ---------------------------------------------------------------------------
# Figure 3 — Side-by-side: recreated chart vs original scan (page 9)
# ---------------------------------------------------------------------------

orig_img = HERE / "page-09.png"
if orig_img.exists():
    fig, axes = plt.subplots(1, 2, figsize=(16, 10))
    fig.suptitle(
        "Left: recreated from digitized data (Rotor C, NACA 4412, CT and CL)\n"
        "Right: original NACA-TR-552 page 207 scan  —  verify digitization accuracy",
        fontsize=10
    )

    ax = axes[0]
    ax.set_title("Recreated — CL (top) and CT (bottom) vs μ", fontsize=9)

    # Twin-axis CL on top half, CT on bottom half using inset axes
    ax.set_xlim(0, 0.95)
    ax.set_ylim(-0.04, 0.30)
    ax.set_xlabel("Tip-speed ratio  μ", fontsize=9)
    ax.set_ylabel("Lift coefficient  $C_L$", fontsize=9)
    ax.set_xticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
    ax.grid(True, alpha=0.3)
    ax.axvspan(RAWES_MU_LO, RAWES_MU_HI, color="#90EE90", alpha=0.4, label="RAWES μ range")

    for i, pitch in enumerate(C_PITCHES):
        data = select(rows, rotor="C", pitch=pitch)
        mus  = [d["mu"] for d in data if not math.isnan(d["CL"])]
        vals = [d["CL"] for d in data if not math.isnan(d["CL"])]
        if len(mus) < 2:
            continue
        ls = "--" if pitch < 0 else "-"
        ax.plot(mus, vals, color=COLORS[i], lw=1.6, ls=ls, marker="o", ms=3,
                label=f"{pitch:+.1f}°")

    ax.axhline(0, color="black", lw=0.5)
    ax.legend(title="Pitch", fontsize=7, title_fontsize=8,
              loc="upper right", ncol=2)

    # CT inset axis (bottom-right of left panel)
    ax_ct = ax.inset_axes([0.0, -0.60, 1.0, 0.55])
    ax_ct.set_xlim(0, 0.95)
    ax_ct.set_ylim(0, 0.013)
    ax_ct.set_xlabel("Tip-speed ratio  μ", fontsize=8)
    ax_ct.set_ylabel("Thrust coefficient  $C_T$", fontsize=8)
    ax_ct.set_xticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
    ax_ct.grid(True, alpha=0.3)
    ax_ct.axvspan(RAWES_MU_LO, RAWES_MU_HI, color="#90EE90", alpha=0.4)

    for i, pitch in enumerate([4.5, 3.5, 3.0, 1.5, 0.0]):
        data = select(rows, rotor="C", pitch=pitch)
        mus  = [d["mu"] for d in data if not math.isnan(d["CT"])]
        vals = [d["CT"] for d in data if not math.isnan(d["CT"])]
        if len(mus) < 2:
            continue
        ax_ct.plot(mus, vals, color=COLORS[C_PITCHES.index(pitch)],
                   lw=1.6, marker="o", ms=3, label=f"{pitch:+.1f}°")
    ax_ct.legend(title="Pitch", fontsize=7, title_fontsize=8, loc="upper right")

    # Original scan on right
    img = mpimg.imread(str(orig_img))
    axes[1].imshow(img)
    axes[1].set_title("Original NACA-TR-552 page 207 scan (Rotor C, NACA 4412)", fontsize=9)
    axes[1].axis("off")

    plt.subplots_adjust(left=0.05, right=0.98, top=0.88, bottom=0.05, wspace=0.12)
    out3 = HERE / "naca_comparison_vs_original.png"
    fig.savefig(out3, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out3}")

# ---------------------------------------------------------------------------
# Figure 4 — Beaupoil context: where our rotor sits on the NACA curves
# ---------------------------------------------------------------------------

fig, axes = plt.subplots(1, 2, figsize=(13, 5))
fig.suptitle(
    "NACA-TR-552 Rotor C (NACA 4412) — zoomed to RAWES Beaupoil operating range\n"
    "Beaupoil op. point: μ ≈ 0.10–0.18,  estimated pitch ≈ 0° to −5.7°",
    fontsize=10
)

for ax, key, ylabel, ylim in [
    (axes[0], "CL", "Lift coefficient  $C_L$",  (-0.05, 0.20)),
    (axes[1], "CT", "Thrust coefficient  $C_T$", (0, 0.008)),
]:
    ax.set_xlabel("Tip-speed ratio  μ", fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.set_xlim(0, 0.60)
    ax.set_ylim(*ylim)
    ax.axvspan(RAWES_MU_LO, RAWES_MU_HI, color="#90EE90", alpha=0.45, label="Beaupoil op. range")
    ax.axvline(RAWES_MU_LO, color="green", lw=1.0, ls="--")
    ax.axvline(RAWES_MU_HI, color="green", lw=1.0, ls="--")
    ax.axhline(0, color="black", lw=0.6)
    ax.grid(True, alpha=0.35)

    for i, pitch in enumerate(C_PITCHES):
        data = select(rows, rotor="C", pitch=pitch, mu_range=(0, 0.65))
        mus  = [d["mu"]  for d in data if not math.isnan(d[key])]
        vals = [d[key]   for d in data if not math.isnan(d[key])]
        if len(mus) < 2:
            continue
        ls = "--" if pitch < 0 else "-"
        lw = 2.0 if -1.0 <= pitch <= 1.0 else 1.4
        ax.plot(mus, vals, color=COLORS[i], lw=lw, ls=ls, marker="o", ms=4,
                label=f"{pitch:+.1f}°")

    ax.legend(title="Pitch", fontsize=8, title_fontsize=9, loc="upper right", ncol=2)

# Annotate Beaupoil pitch region
for ax in axes:
    ax.annotate(
        "Beaupoil range\nμ=0.10–0.18\npitch≈0° to −5.7°",
        xy=(0.14, axes[0].get_ylim()[0] * 0.3 + axes[0].get_ylim()[1] * 0.7),
        xytext=(0.28, (ax.get_ylim()[0] + ax.get_ylim()[1]) * 0.7),
        arrowprops=dict(arrowstyle="->", color="darkgreen"),
        fontsize=8, color="darkgreen",
        bbox=dict(boxstyle="round,pad=0.3", fc="lightyellow", ec="green", alpha=0.8),
    )

plt.tight_layout()
out4 = HERE / "naca_beaupoil_range.png"
fig.savefig(out4, dpi=150, bbox_inches="tight")
plt.close(fig)
print(f"Saved: {out4}")

print("\nAll plots saved to:", HERE)
