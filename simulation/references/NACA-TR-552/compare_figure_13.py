"""
compare_figure_13.py — Overlay digitized CSV data on the user-traced Figure 13.

Finds graph boundaries automatically by detecting the axis lines in the image,
then transforms data coordinates to image pixel coordinates for accurate overlay.
"""

import sys, math, csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.transforms import Affine2D

HERE = Path(__file__).parent

# ---------------------------------------------------------------------------
# Load CSV
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
                    "rotor":  row["rotor"].strip(),
                    "pitch":  float(row["pitch_deg"]),
                    "mu":     float(row["mu"]),
                    "CL":     float(row["CL"])  if row["CL"].strip()  else float("nan"),
                    "CT":     float(row["CT"])  if row["CT"].strip()  else float("nan"),
                    "LD":     float(row["LD"])  if row["LD"].strip()  else float("nan"),
                })
            except (ValueError, TypeError):
                continue
    return rows

def get_curve(rows, rotor, pitch, tol=0.26):
    data = sorted(
        [r for r in rows if r["rotor"] == rotor and abs(r["pitch"] - pitch) < tol],
        key=lambda r: r["mu"]
    )
    mus  = [r["mu"] for r in data if not math.isnan(r["CL"])]
    vals = [r["CL"] for r in data if not math.isnan(r["CL"])]
    return mus, vals

rows = load_csv(HERE / "naca_tr_552_data.csv")

# ---------------------------------------------------------------------------
# Detect graph boundaries from traced image
# ---------------------------------------------------------------------------

img = mpimg.imread(str(HERE / "figure_13_traced.png"))
H, W = img.shape[:2]

# Convert to grayscale
gray = np.mean(img[:, :, :3], axis=2) if img.shape[2] >= 3 else img[:, :, 0]

# Find leftmost dark vertical column (y-axis line) — left 30% of image
left_region = gray[:, :int(W * 0.35)]
col_darkness = np.mean(left_region < 0.3, axis=0)   # fraction of dark pixels per column
x_axis_col = int(np.argmax(col_darkness)) + 1        # pixel x of y-axis

# Find bottommost dark horizontal row (x-axis line) — bottom 40% of image
bot_region = gray[int(H * 0.60):, :]
row_darkness = np.mean(bot_region < 0.3, axis=1)
y_axis_row = int(np.argmax(row_darkness)) + int(H * 0.60)  # pixel y of x-axis

# Right edge: last major vertical grid line — search right 15% of image
right_region = gray[:, int(W * 0.75):]
col_d_r = np.mean(right_region < 0.3, axis=0)
x_right_col = int(W * 0.75) + int(np.argmax(col_d_r[::-1]))  # last dark col from right

# Top edge: first major horizontal grid line — search top 15%
top_region = gray[:int(H * 0.20), :]
row_d_t = np.mean(top_region < 0.3, axis=1)
y_top_row = int(np.argmax(row_d_t))

print(f"Image size: {W}x{H}")
print(f"Detected graph boundaries (pixels):")
print(f"  x-axis col (left):  {x_axis_col}")
print(f"  x-axis row (bottom): {y_axis_row}")
print(f"  right col:          {x_right_col}")
print(f"  top row:            {y_top_row}")

# Data axis limits (confirmed from traced image)
X_DATA_MIN, X_DATA_MAX = 0.0,   0.90
Y_DATA_MIN, Y_DATA_MAX = -0.10, 0.30

# Transform: data → pixel
def data_to_px(mu, CL):
    px = x_axis_col + (mu  - X_DATA_MIN) / (X_DATA_MAX - X_DATA_MIN) * (x_right_col - x_axis_col)
    py = y_axis_row  - (CL  - Y_DATA_MIN) / (Y_DATA_MAX - Y_DATA_MIN) * (y_axis_row  - y_top_row)
    return px, py

# ---------------------------------------------------------------------------
# Figure: left = traced + overlay, right = CSV only
# ---------------------------------------------------------------------------

fig, axes = plt.subplots(1, 2, figsize=(14, 6))
fig.suptitle(
    "Figure 13 — Lift coefficient  $C_L$  vs  μ  (Rotor C, NACA 4412)\n"
    "Left: user-traced original with CSV data overlaid    Right: CSV data alone",
    fontsize=11
)

# ── Left panel: traced image in pixel coords ─────────────────────────────────
ax = axes[0]
ax.imshow(img, origin="upper", zorder=0)
ax.set_xlim(0, W)
ax.set_ylim(H, 0)   # image origin top-left
ax.axis("off")

# RAWES range band
mu_lo_px, _  = data_to_px(0.10, Y_DATA_MIN)
mu_hi_px, _  = data_to_px(0.18, Y_DATA_MIN)
ax.axvspan(mu_lo_px, mu_hi_px, color="#00CC00", alpha=0.25, zorder=1)

# Zero-CL horizontal line
_, zero_py = data_to_px(0, 0.0)
ax.axhline(zero_py, color="black", lw=0.8, ls=":", zorder=1)

# Overlay CSV curves in pixel coordinates
overlay = [
    (0.0, "red",    (3, 6), "0° (CSV)"),
    (1.5, "#CC7700",(4, 8), "1.5° (CSV ≈ 2°)"),
    (3.5, "blue",   (5,10), "3.5° (CSV ≈ 3.6°)"),
]
for pitch, color, (ms, lw), label in overlay:
    mus, vals = get_curve(rows, "C", pitch)
    if len(mus) < 2:
        continue
    pxs = [data_to_px(m, v) for m, v in zip(mus, vals)]
    xs  = [p[0] for p in pxs]
    ys  = [p[1] for p in pxs]
    ax.plot(xs, ys, color=color, lw=lw, ls="--", marker="s", ms=ms, zorder=3,
            label=label)

# Annotate RAWES range
mu_mid_px, _ = data_to_px(0.14, Y_DATA_MAX)
ax.annotate("RAWES\nrange", xy=(mu_mid_px, y_top_row + 15),
            xytext=(mu_mid_px + 25, y_top_row + 40),
            arrowprops=dict(arrowstyle="->", color="darkgreen"),
            fontsize=7.5, color="darkgreen",
            bbox=dict(boxstyle="round,pad=0.2", fc="lightyellow", ec="green", alpha=0.9))

ax.legend(fontsize=8, loc="lower left", framealpha=0.85)
ax.set_title(f"Traced image + CSV overlay  (graph area: x={x_axis_col}–{x_right_col}px, y={y_top_row}–{y_axis_row}px)",
             fontsize=8)

# ── Right panel: CSV data alone ──────────────────────────────────────────────
ax2 = axes[1]
ax2.set_xlim(X_DATA_MIN, X_DATA_MAX)
ax2.set_ylim(Y_DATA_MIN, Y_DATA_MAX)
ax2.set_xlabel("Tip-speed ratio  μ", fontsize=10)
ax2.set_ylabel("Lift coefficient  $C_L$", fontsize=10)
ax2.set_xticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
ax2.axhline(0, color="black", lw=0.8)
ax2.axvspan(0.10, 0.18, color="#90EE90", alpha=0.45, label="RAWES μ range")
ax2.grid(True, alpha=0.35)

# All pitch settings
pitches  = [4.5, 3.5, 3.0, 1.5,  0.0, -1.5, -3.0, -4.5]
colors_p = plt.cm.RdYlGn(np.linspace(0.15, 0.85, len(pitches)))[::-1]

for i, pitch in enumerate(pitches):
    mus, vals = get_curve(rows, "C", pitch)
    if len(mus) < 2:
        continue
    ls = "--" if pitch < 0 else "-"
    lw = 2.2 if abs(pitch) < 0.1 else 1.6
    ax2.plot(mus, vals, color=colors_p[i], lw=lw, ls=ls, marker="o", ms=4,
             label=f"{pitch:+.1f}°")

# Highlight the three traced curves
for pitch, color, label in [(0.0,"red","0°"), (1.5,"#CC7700","1.5° (≈2°)"), (3.5,"blue","3.5° (≈3.6°)")]:
    mus, vals = get_curve(rows, "C", pitch)
    if len(mus) >= 2:
        ax2.plot(mus, vals, color=color, lw=3.0, ls="-", alpha=0.5, zorder=0)

ax2.annotate("Beaupoil\nop. point\n(μ≈0.14)", xy=(0.14, 0.05),
             xytext=(0.28, 0.12),
             arrowprops=dict(arrowstyle="->", color="darkgreen"),
             fontsize=8, color="darkgreen",
             bbox=dict(boxstyle="round,pad=0.2", fc="lightyellow", ec="green", alpha=0.8))

ax2.legend(title="Pitch setting", fontsize=8, title_fontsize=9,
           loc="upper right", ncol=2)
ax2.set_title("CSV data — Rotor C (NACA 4412)", fontsize=9)

plt.tight_layout()
out = HERE / "figure_13_comparison.png"
fig.savefig(out, dpi=150, bbox_inches="tight")
plt.close(fig)
print(f"Saved: {out}")
