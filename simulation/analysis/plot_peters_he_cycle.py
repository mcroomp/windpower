"""
plot_peters_he_cycle.py — tether tension, motor response, aero response
for the Peters-He pumping cycle telemetry.

Usage:
  .venv/Scripts/python.exe simulation/analysis/plot_peters_he_cycle.py
  .venv/Scripts/python.exe simulation/analysis/plot_peters_he_cycle.py --csv path/to/telemetry.csv
"""
import sys
import argparse
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

_SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM))

DEFAULT_CSV = _SIM / "logs" / "test_pump_cycle_unified" / "telemetry.csv"

PHASE_COLORS = {
    "reel-out":        "#2196F3",   # blue
    "transition":      "#FF9800",   # orange
    "reel-in":         "#4CAF50",   # green
    "transition-back": "#9C27B0",   # purple
    "hold":            "#607D8B",   # grey
}


def load(csv: Path) -> dict:
    """Load telemetry CSV into numpy arrays keyed by column name."""
    with open(csv, newline="") as f:
        header = f.readline().strip().split(",")
    data = np.genfromtxt(csv, delimiter=",", skip_header=1,
                         filling_values=0.0)
    d = {h: data[:, i] for i, h in enumerate(header)}
    # phase is a string column — re-read it
    phases = []
    with open(csv, newline="") as f:
        f.readline()
        for line in f:
            parts = line.split(",", 3)
            phases.append(parts[1].strip() if len(parts) > 1 else "")
    d["phase"] = np.array(phases)
    return d


def shade_phases(ax, t, phases):
    """Draw a thin colored background stripe for each phase."""
    if len(t) == 0:
        return
    cur = phases[0]
    t0  = t[0]
    for i in range(1, len(t)):
        if phases[i] != cur or i == len(t) - 1:
            col = PHASE_COLORS.get(cur, "#EEEEEE")
            ax.axvspan(t0, t[i], alpha=0.12, color=col, linewidth=0)
            cur = phases[i]
            t0  = t[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(DEFAULT_CSV))
    ap.add_argument("--t0",  type=float, default=None, help="zoom start [s]")
    ap.add_argument("--t1",  type=float, default=None, help="zoom end [s]")
    args = ap.parse_args()

    csv = Path(args.csv)
    if not csv.exists():
        print(f"CSV not found: {csv}")
        sys.exit(1)

    d = load(csv)
    t = d["t_sim"]

    mask = np.ones(len(t), dtype=bool)
    if args.t0 is not None:
        mask &= t >= args.t0
    if args.t1 is not None:
        mask &= t <= args.t1
    t  = t[mask]
    ph = d["phase"][mask]

    def col(name):
        return d[name][mask]

    fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True)
    fig.suptitle("Peters-He pumping cycle — aero / tether / motor response", fontsize=13)

    # ── Row 1: Tether tension ─────────────────────────────────────────────────
    ax = axes[0]
    shade_phases(ax, t, ph)
    ax.plot(t, col("tether_tension"), color="#E53935", lw=0.9, label="tension")
    ax.axhline(435, color="#E53935", ls="--", lw=0.7, alpha=0.6, label="T_out setpoint")
    ax.axhline(226, color="#1E88E5", ls="--", lw=0.7, alpha=0.6, label="T_in setpoint")
    ax.axhline(0,   color="k",       ls=":",  lw=0.5)
    ax.set_ylabel("Tether tension [N]")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_ylim(-20, 600)

    # ── Row 2: Aero thrust (axial) ────────────────────────────────────────────
    ax = axes[1]
    shade_phases(ax, t, ph)
    ax.plot(t, col("aero_T"), color="#7B1FA2", lw=0.9, label="aero thrust T")
    ax.set_ylabel("Aero thrust [N]")
    ax.legend(loc="upper right", fontsize=8)
    ax.axhline(0, color="k", ls=":", lw=0.5)

    # ── Row 3: Inflow velocities (Peters-He states) ───────────────────────────
    ax = axes[2]
    shade_phases(ax, t, ph)
    ax.plot(t, col("aero_v_i"),      color="#00897B", lw=0.9,  label="v0 (uniform inflow)")
    ax.plot(t, col("aero_v_axial"),  color="#43A047", lw=0.7,  ls="--", label="v_axial (wind-on-disk)")
    ax.plot(t, col("aero_v_inplane"),color="#FB8C00", lw=0.7,  ls="--", label="v_inplane")
    ax.set_ylabel("Velocity [m/s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.axhline(0, color="k", ls=":", lw=0.5)

    # ── Row 4: Rotor spin + collective ────────────────────────────────────────
    ax = axes[3]
    shade_phases(ax, t, ph)
    ax2 = ax.twinx()
    ax.plot(t,  col("omega_rotor"),    color="#1565C0", lw=0.9, label="omega_rotor [rad/s]")
    ax2.plot(t, col("collective_rad"), color="#C62828", lw=0.7, ls="--", label="collective [rad]")
    ax.set_ylabel("omega_rotor [rad/s]", color="#1565C0")
    ax2.set_ylabel("collective [rad]",   color="#C62828")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    # ── Row 5: Hub altitude + winch speed ─────────────────────────────────────
    ax = axes[4]
    shade_phases(ax, t, ph)
    ax2 = ax.twinx()
    ax.plot(t,  -col("pos_z"),          color="#37474F", lw=0.9, label="altitude [m]")
    ax2.plot(t,  col("winch_speed_ms"), color="#558B2F", lw=0.7, ls="--", label="winch speed [m/s]")
    ax.set_ylabel("Altitude [m]",        color="#37474F")
    ax2.set_ylabel("Winch speed [m/s]",  color="#558B2F")
    ax.set_xlabel("t [s]")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    # Phase legend
    patches = [mpatches.Patch(color=c, alpha=0.4, label=p)
               for p, c in PHASE_COLORS.items()]
    fig.legend(handles=patches, loc="lower center", ncol=len(PHASE_COLORS),
               fontsize=8, framealpha=0.8)

    plt.tight_layout(rect=[0, 0.04, 1, 0.97])
    plt.show()


if __name__ == "__main__":
    main()
