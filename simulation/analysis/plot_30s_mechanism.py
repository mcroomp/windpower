#!/usr/bin/env python3
"""
plot_30s_mechanism.py — Annotated diagram explaining the 30-second dip.

Reads telemetry.csv and produces a single self-contained figure showing
the coupled tether-spring / rotor-spin oscillation mechanism.

Usage:  python plot_30s_mechanism.py [telemetry.csv] [--out mechanism.png]
"""

import argparse, csv, math, sys
from pathlib import Path


def load(path):
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    def col(k): return [float(r[k]) for r in rows]
    return {k: col(k) for k in rows[0]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="telemetry.csv", type=Path)
    ap.add_argument("--out", "-o", type=Path, default=None)
    args = ap.parse_args()
    out = args.out or args.csv.with_name("mechanism_explained.png")

    d = load(args.csv)
    t        = d["t_sim"]
    pos_z    = d["hub_pos_z"]
    vel_z    = d["hub_vel_z"]
    omega    = d["omega_rotor"]
    T        = d["aero_T"]
    F_z      = d["F_z"]
    v_axial  = d["aero_v_axial"]
    Q_drag   = d["aero_Q_drag"]
    Q_drive  = d["aero_Q_drive"]
    Q_net    = [a + b for a, b in zip(Q_drag, Q_drive)]
    coll_deg = [math.degrees(v) for v in d["collective_rad"]]

    TETHER_K, TETHER_REST, WEIGHT = 300.0, 50.0, 49.05
    tether_ext   = [max(0.0, z - TETHER_REST) for z in pos_z]
    tether_force = [TETHER_K * e for e in tether_ext]
    net_Fz = [F_z[i] + tether_force[i] - WEIGHT for i in range(len(t))]

    # ── clip to 0–70 s for clarity ────────────────────────────────────────────
    T_MAX = 70.0
    idx   = [i for i, ti in enumerate(t) if ti <= T_MAX]
    sl    = slice(idx[0], idx[-1] + 1)
    tt    = t[sl]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gs
    import matplotlib.patches as mpatches
    from matplotlib.lines import Line2D

    # ── colour palette ────────────────────────────────────────────────────────
    C_ALT    = "#1f77b4"   # blue   — altitude
    C_VZ     = "#aec7e8"   # light blue — Vz
    C_OMEGA  = "#9467bd"   # purple — rotor spin
    C_THRUST = "#2ca02c"   # green  — thrust
    C_TETHER = "#ff7f0e"   # orange — tether
    C_WEIGHT = "#7f7f7f"   # grey   — weight line
    C_NET    = "#000000"   # black  — net force
    C_QDRAG  = "#e377c2"   # pink   — Q_drag
    C_QDRIVE = "#d62728"   # red    — Q_drive
    C_QNET   = "#8c564b"   # brown  — Q_net
    C_DIP    = "#d62728"

    fig = plt.figure(figsize=(18, 20))
    fig.patch.set_facecolor("#f9f9f9")

    fig.suptitle(
        "RAWES Guided-Flight Simulation — What Causes the 30-Second Dip?\n"
        "Coupled tether-spring overshoot  ×  rotor-spin decay/recovery",
        fontsize=15, fontweight="bold", y=0.99,
    )

    GS = gs.GridSpec(
        5, 2,
        figure=fig,
        hspace=0.60, wspace=0.38,
        top=0.94, bottom=0.06,
        left=0.07, right=0.97,
    )

    # ── helpers ───────────────────────────────────────────────────────────────
    DIP_T  = 34.0   # approximate altitude minimum
    PHASE  = [
        (0,  5,  "#fff9c4", "Ramp-up\n(aero loads\nzero→full)"),
        (5,  14, "#c8f0c8", "Launch:\ntether launches\nhub skyward"),
        (14, 34, "#fde0dc", "Spin decay +\ndescent:\nQ_net < 0 → ω↓ → T↓"),
        (34, 50, "#dce8fd", "Recovery:\nQ_net > 0 → ω↑ → T↑"),
    ]

    def shade(ax):
        for t0, t1, color, label in PHASE:
            ax.axvspan(t0, t1, alpha=0.25, color=color, lw=0)
        ax.axvline(DIP_T, color=C_DIP, lw=1.0, ls="--", alpha=0.6)

    def fmt(ax, ylabel, title):
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_title(title, fontsize=9, fontweight="bold", loc="left", pad=3)
        ax.grid(True, alpha=0.25, lw=0.6)
        ax.tick_params(labelsize=8)
        ax.set_xlim(0, T_MAX)
        shade(ax)

    def xlabel(ax):
        ax.set_xlabel("Simulation time  t_sim (s)", fontsize=8)

    # ═════════════════════════════════════════════════════════════════════════
    # Panel A  (top-left, spans 2 rows) — Altitude + Vz
    # ═════════════════════════════════════════════════════════════════════════
    axA = fig.add_subplot(GS[0:2, 0])  # spans rows 0–1, left column
    axA.plot(tt, pos_z[sl],  color=C_ALT,   lw=1.6, label="Hub altitude Z (m)")
    axA.axhline(TETHER_REST, color=C_TETHER, lw=0.9, ls=":", alpha=0.7,
                label="Tether rest length 50 m")
    axA.axhline(TETHER_REST + 50, color=C_TETHER, lw=0.6, ls=":", alpha=0.4,
                label="50 m above rest (tether=15 kN)")
    axA2 = axA.twinx()
    axA2.plot(tt, vel_z[sl], color=C_VZ, lw=1.0, ls="--", label="Vz (m/s)")
    axA2.axhline(0, color="black", lw=0.5, alpha=0.3)
    axA2.set_ylabel("Vz (m·s⁻¹)", fontsize=9, color=C_VZ)
    axA2.tick_params(labelsize=8)
    fmt(axA, "Altitude Z ENU (m)", "A  Hub altitude & vertical velocity")

    # Annotations
    axA.annotate("Hub overshoots\nto 85 m\n(tether +35 m)", xy=(13, 85.3),
                 xytext=(20, 83), fontsize=8, color=C_ALT,
                 arrowprops=dict(arrowstyle="->", color=C_ALT, lw=0.8))
    axA.annotate(f"Altitude minimum\n≈61 m  (t≈{DIP_T:.0f}s)", xy=(DIP_T, 60.9),
                 xytext=(40, 57), fontsize=8, color=C_DIP,
                 arrowprops=dict(arrowstyle="->", color=C_DIP, lw=0.8))

    lines1, labs1 = axA.get_legend_handles_labels()
    lines2, labs2 = axA2.get_legend_handles_labels()
    axA.legend(lines1 + lines2, labs1 + labs2, fontsize=7, loc="upper right")

    # ═════════════════════════════════════════════════════════════════════════
    # Panel B  (top-right, spans 2 rows) — Rotor spin + torque balance
    # Split into two sub-panels stacked in the same GridSpec slot via axes
    # ═════════════════════════════════════════════════════════════════════════
    axB = fig.add_subplot(GS[0, 1])
    axB.plot(tt, omega[sl], color=C_OMEGA, lw=1.6, label="ω rotor (rad/s)")
    axB.axhline(28.0, color=C_OMEGA, lw=0.7, ls=":", alpha=0.5,
                label="Nominal ω = 28 rad/s")
    axB.set_ylim(0, 40)
    fmt(axB, "ω rotor (rad/s)", "B  Rotor spin rate")
    axB.legend(fontsize=7, loc="upper right")

    # Find Q_net sign flip time
    qn_sl = Q_net[sl]
    flip_t = None
    for i in range(1, len(qn_sl)):
        if qn_sl[i-1] < 0 and qn_sl[i] >= 0:
            flip_t = tt[i]; break

    closest_i = lambda tv: min(range(len(tt)), key=lambda i: abs(tt[i] - tv))
    axB.annotate("ω decay:\n|Q_drag| > Q_drive",
                 xy=(18, omega[sl][closest_i(18)]),
                 xytext=(25, 30), fontsize=7.5, color=C_QDRAG,
                 arrowprops=dict(arrowstyle="->", color=C_QDRAG, lw=0.8))
    if flip_t:
        axB.annotate(f"Q_net → +\nω recovers\n(t≈{flip_t:.0f}s)",
                     xy=(flip_t, omega[sl][closest_i(flip_t)]),
                     xytext=(flip_t + 6, 10), fontsize=7.5, color=C_QNET,
                     arrowprops=dict(arrowstyle="->", color=C_QNET, lw=0.8))

    axB2 = fig.add_subplot(GS[1, 1])
    axB2.plot(tt, Q_drag[sl],  color=C_QDRAG,  lw=1.0, label="Q_drag (N·m)  opposes spin")
    axB2.plot(tt, Q_drive[sl], color=C_QDRIVE, lw=1.0, ls="--", label="Q_drive (N·m)  wind-driven, ≈const")
    axB2.plot(tt, qn_sl,       color=C_QNET,   lw=1.4, label="Q_net = Q_drag + Q_drive")
    axB2.axhline(0, color="black", lw=0.9, alpha=0.5, ls="-.")
    if flip_t:
        axB2.axvline(flip_t, color=C_QNET, lw=0.8, ls=":", alpha=0.7)
    axB2.set_ylim(-80, 80)
    fmt(axB2, "Torque (N·m)", "B2  Torque balance")
    axB2.legend(fontsize=7, loc="upper right")
    axB2.text(2, 44, "← drag wins (ω decays)", fontsize=7.5, color=C_QDRAG)
    axB2.text(2, -60, "← drive wins (ω recovers)", fontsize=7.5, color=C_QDRIVE)

    # ═════════════════════════════════════════════════════════════════════════
    # Panel C  (row 2, left) — Force budget
    # ═════════════════════════════════════════════════════════════════════════
    axC = fig.add_subplot(GS[2, 0])   # row 2, left
    axC.plot(tt, T[sl],            color=C_THRUST, lw=1.2, label="Aero thrust T (N)")
    axC.plot(tt, tether_force[sl], color=C_TETHER, lw=1.2, label="Tether spring force (N)")
    axC.axhline(WEIGHT,            color=C_WEIGHT, lw=0.9, ls=":", label="Weight 49 N")
    fmt(axC, "Force (N)", "C  Force budget: what holds the hub up?")
    axC.set_yscale("log")
    axC.set_ylim(1, 15000)
    axC.legend(fontsize=7, loc="upper right")
    axC.annotate("Tether dominates:\n8000–10000 N\nvs thrust ~250 N",
                 xy=(10, tether_force[sl][tt.index(min(tt, key=lambda x: abs(x-10)))]),
                 xytext=(25, 5000), fontsize=7.5, color=C_TETHER,
                 arrowprops=dict(arrowstyle="->", color=C_TETHER, lw=0.8))
    axC.annotate("Thrust collapses\nas ω² falls",
                 xy=(25, T[sl][tt.index(min(tt, key=lambda x: abs(x-25)))]),
                 xytext=(38, 80), fontsize=7.5, color=C_THRUST,
                 arrowprops=dict(arrowstyle="->", color=C_THRUST, lw=0.8))
    xlabel(axC)

    # ═════════════════════════════════════════════════════════════════════════
    # Panel D  (row 2, right) — Tether extension
    # ═════════════════════════════════════════════════════════════════════════
    axD = fig.add_subplot(GS[2, 1])   # row 2, right
    axD.plot(tt, tether_ext[sl], color=C_TETHER, lw=1.3, label="Tether extension (m)")
    axD2 = axD.twinx()
    axD2.plot(tt, tether_force[sl], color="#d62728", lw=1.0, ls="--",
              label="Spring force (N) = 300 × ext")
    axD2.set_ylabel("Spring force (N)", fontsize=9, color="#d62728")
    axD2.tick_params(labelsize=8)
    fmt(axD, "Extension beyond 50 m (m)", "D  Tether extension & spring force")
    lines1, labs1 = axD.get_legend_handles_labels()
    lines2, labs2 = axD2.get_legend_handles_labels()
    axD.legend(lines1 + lines2, labs1 + labs2, fontsize=7, loc="upper right")
    xlabel(axD)

    # ═════════════════════════════════════════════════════════════════════════
    # Panel E  (row 3, full width) — Feedback loop diagram as text+arrows
    #          drawn in data coords of a blank axes
    # ═════════════════════════════════════════════════════════════════════════
    axE = fig.add_subplot(GS[3:5, :])  # rows 3–4, full width
    axE.set_xlim(0, 10)
    axE.set_ylim(0, 10)
    axE.axis("off")
    axE.set_title("E  Mechanism: two coupled feedback loops",
                  fontsize=9, fontweight="bold", loc="left", pad=3)

    # ── Causal chain boxes ────────────────────────────────────────────────────
    BOXES = [
        # (cx, cy, text, facecolor, phase)
        (0.7,  7.5, "Large\ninitial\ntether\nextension\n(hub at 50m,\nrest=50m)",
                     "#fff9c4", "INIT"),
        (2.8,  7.5, "Huge\nupward\nspring force\n(300 N/m × ext)\n≫ weight (49 N)",
                     "#fff9c4", "→"),
        (5.0,  7.5, "Hub\naccelerated\nupward\nZ → 85 m\n(t≈13 s)",
                     "#c8f0c8", "→"),
        (7.2,  7.5, "v_axial ↑\n(hub rising,\nwind less\neffective\nthrough disk)",
                     "#fde0dc", "→"),
        (9.0,  7.5, "But ω = 28\n→ T ≈ 250 N\n(initial\novershoot)",
                     "#fde0dc", ""),

        (0.7,  3.5, "Hub\ndescends\nback to\nrest Z≈61m\n(t≈34 s)",
                     "#dce8fd", "BOTTOM"),
        (2.8,  3.5, "Tether\nextension\nshrinks\n→ spring force\nfalls",
                     "#dce8fd", "→"),
        (5.0,  3.5, "T ↓  →\nQ_drag ↓\n(Q_drag ∝ T)\nQ_net flips +\nω starts ↑",
                     "#dce8fd", "→"),
        (7.2,  3.5, "T = K_auto·\nρ·A·v_ip²·r\n(const ≈41 Nm)\nbeats Q_drag\nonce T<~120N",
                     "#dce8fd", "→"),
        (9.0,  3.5, "ω ↑ → T ↑\n→ hub climbs\nagain\n(bounce)",
                     "#c8f0c8", ""),
    ]

    box_w, box_h = 1.55, 3.5

    def draw_box(cx, cy, text, fc):
        rect = mpatches.FancyBboxPatch(
            (cx - box_w/2, cy - box_h/2), box_w, box_h,
            boxstyle="round,pad=0.05", linewidth=0.8,
            edgecolor="#555", facecolor=fc, zorder=3,
        )
        axE.add_patch(rect)
        axE.text(cx, cy, text, ha="center", va="center",
                 fontsize=7, zorder=4, multialignment="center")

    for cx, cy, text, fc, _ in BOXES:
        draw_box(cx, cy, text, fc)

    # Arrows between boxes on same row
    def arrow(x0, y0, x1, y1, color="#444", label=""):
        axE.annotate("", xy=(x1, y1), xytext=(x0, y0),
                     arrowprops=dict(arrowstyle="-|>", color=color, lw=1.1), zorder=5)
        if label:
            mx, my = (x0+x1)/2, (y0+y1)/2
            axE.text(mx, my + 0.3, label, ha="center", fontsize=6.5, color=color)

    # Top row arrows
    gap = 0.78
    for i in range(len(BOXES)-1):
        if BOXES[i][4] == "→" or (i < 4 and BOXES[i+1][4] in ("→","")):
            x0 = BOXES[i][0] + box_w/2
            x1 = BOXES[i+1][0] - box_w/2
            y  = BOXES[i][1]
            if abs(BOXES[i][1] - BOXES[i+1][1]) < 0.5:
                arrow(x0, y, x1, y, color="#2c7")

    # Bottom row arrows
    for i in range(5, len(BOXES)-1):
        x0 = BOXES[i][0] + box_w/2
        x1 = BOXES[i+1][0] - box_w/2
        y  = BOXES[i][1]
        if abs(BOXES[i][1] - BOXES[i+1][1]) < 0.5:
            arrow(x0, y, x1, y, color="#1166cc")

    # Downward connector: top-row "hub at 85m" → bottom-row "hub descends"
    axE.annotate("", xy=(BOXES[5][0], BOXES[5][1] + box_h/2),
                 xytext=(BOXES[2][0], BOXES[2][1] - box_h/2),
                 arrowprops=dict(arrowstyle="-|>", color="#d62728", lw=1.3,
                                 connectionstyle="arc3,rad=0.0"), zorder=5)
    axE.text(4.1, 5.5, "Gravity + tether pulls\nhub back down as\nextension shrinks",
             fontsize=7, color="#d62728", ha="center", style="italic")

    # Spin-decay connector: top-row omega-high → drag-wins
    axE.annotate("", xy=(BOXES[7][0], BOXES[7][1] + box_h/2),
                 xytext=(BOXES[3][0], BOXES[3][1] - box_h/2),
                 arrowprops=dict(arrowstyle="-|>", color="#9467bd", lw=1.1,
                                 connectionstyle="arc3,rad=0.25"), zorder=5)
    axE.text(7.5, 5.5, "Q_drag=−K·T·r\ngrows with T\n→ ω decays",
             fontsize=7, color="#9467bd", ha="center", style="italic")

    # Phase labels
    for label_x, label_y, txt, col in [
        (0.7,  9.5, "① LAUNCH  (t = 0–13 s)", "#2c7"),
        (5.0,  9.5, "② FALL + SPIN DECAY  (t = 14–34 s)", "#d62728"),
        (0.7,  1.5, "③ MINIMUM  (t ≈ 34 s) → BOUNCE", "#1166cc"),
        (7.2,  1.5, "④ RECOVERY  (t = 34–50 s)", "#2c7"),
    ]:
        axE.text(label_x, label_y, txt, fontsize=8, color=col, fontweight="bold")

    # ── Key numbers callout ───────────────────────────────────────────────────
    axE.text(5.0, 0.35,
             "Key numbers:  K_tether = 300 N/m   |   Weight = 49 N   |   "
             "K_drag·r_tip = 0.25 N·m/N   |   K_auto·ρ·A·r = 0.00684 → Q_drive ≈ 41 N·m (const at v_ip=10 m/s)   |   "
             "ω_natural (spring/mass) = √(300/5)/(2π) ≈ 1.23 Hz  (sub-second — not the ≈20 s envelope seen here)",
             ha="center", va="center", fontsize=7, color="#333",
             bbox=dict(boxstyle="round,pad=0.3", fc="#eeeeee", ec="#aaa", lw=0.6))

    # ── Phase legend ──────────────────────────────────────────────────────────
    legend_patches = [
        mpatches.Patch(fc="#fff9c4", ec="#888", label="Ramp / launch phase"),
        mpatches.Patch(fc="#c8f0c8", ec="#888", label="Hub rising / recovering"),
        mpatches.Patch(fc="#fde0dc", ec="#888", label="Spin decay / descent"),
        mpatches.Patch(fc="#dce8fd", ec="#888", label="Near-minimum / bounce"),
    ]
    axE.legend(handles=legend_patches, fontsize=7, loc="upper right",
               framealpha=0.9, ncol=2)

    fig.savefig(str(out), dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved -> {out}")


if __name__ == "__main__":
    main()
