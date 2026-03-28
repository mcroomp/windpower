#!/usr/bin/env python3
"""
analyse_30s_dip.py — Investigate the displacement dip seen at ~30 s in guided flight.

Reads telemetry.csv and produces a focused diagnostic figure plus a printed
summary of what is happening in the 20–40 s window.

Usage
-----
    python analyse_30s_dip.py [telemetry.csv] [--out dip_analysis.png]

The telemetry CSV is written by mediator.py --telemetry-log.
"""

import argparse
import csv
import math
import sys
from pathlib import Path


# ---------------------------------------------------------------------------
# Load
# ---------------------------------------------------------------------------

def load(path: Path) -> dict:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"[ERROR] No data in {path}")

    def col(name):
        return [float(r[name]) for r in rows]

    data = {k: col(k) for k in rows[0]}
    n = len(rows)
    print(f"Loaded {n} rows  ({rows[0]['t_sim']}s – {rows[-1]['t_sim']}s)")
    return data


# ---------------------------------------------------------------------------
# Derived quantities
# ---------------------------------------------------------------------------

def derive(d: dict) -> dict:
    """Add computed columns to the data dict."""
    t       = d["t_sim"]
    pos_z   = d["hub_pos_z"]
    vel_z   = d["hub_vel_z"]
    F_z     = d["F_z"]
    aero_T  = d["aero_T"]
    omega   = d["omega_rotor"]
    v_ax    = d["aero_v_axial"]
    v_ip    = d["aero_v_inplane"]
    Q_drag  = d["aero_Q_drag"]
    Q_drive = d["aero_Q_drive"]
    ramp    = d["aero_ramp"]
    coll    = [math.degrees(v) for v in d["collective_rad"]]

    # Tether spring force: K=300 N/m, rest_length=50m (node-to-node)
    # Ground node at Z=0, hub at pos_z; rod rest len = 50 m.
    # Extension = (hub_height - 50); force = K * extension (positive = tension)
    TETHER_K = 300.0
    TETHER_REST = 50.0
    WEIGHT = 49.05
    tether_ext   = [max(0.0, pz - TETHER_REST) for pz in pos_z]  # extension [m]
    tether_force = [TETHER_K * e for e in tether_ext]            # upward spring [N]

    # Net vertical force = Fz + tether - weight
    net_Fz = [F_z[i] + tether_force[i] - WEIGHT for i in range(len(t))]

    # Torque net = Q_drag + Q_drive
    Q_net = [Q_drag[i] + Q_drive[i] for i in range(len(t))]

    d["collective_deg"] = coll
    d["tether_ext"]     = tether_ext
    d["tether_force"]   = tether_force
    d["net_Fz"]         = net_Fz
    d["Q_net"]          = Q_net
    return d


# ---------------------------------------------------------------------------
# Windowed statistics
# ---------------------------------------------------------------------------

def stats_window(values: list, t: list, t_lo: float, t_hi: float) -> dict:
    sub = [v for v, ti in zip(values, t) if t_lo <= ti <= t_hi]
    if not sub:
        return {"mean": float("nan"), "min": float("nan"), "max": float("nan"), "std": float("nan")}
    mean = sum(sub) / len(sub)
    variance = sum((v - mean) ** 2 for v in sub) / len(sub)
    return {
        "mean": mean,
        "min":  min(sub),
        "max":  max(sub),
        "std":  variance ** 0.5,
        "n":    len(sub),
    }


# ---------------------------------------------------------------------------
# Printed summary
# ---------------------------------------------------------------------------

def print_summary(d: dict) -> None:
    t = d["t_sim"]
    windows = [
        ("Pre-dip   (15–25 s)", 15.0, 25.0),
        ("Dip       (25–35 s)", 25.0, 35.0),
        ("Post-dip  (35–45 s)", 35.0, 45.0),
    ]

    cols = {
        "hub_pos_z        (m)":     d["hub_pos_z"],
        "hub_vel_z       (m/s)":    d["hub_vel_z"],
        "aero_T           (N)":     d["aero_T"],
        "F_z (→MBDyn)     (N)":    d["F_z"],
        "tether_force     (N)":     d["tether_force"],
        "net_Fz (F+T-W)   (N)":    d["net_Fz"],
        "aero_v_axial    (m/s)":    d["aero_v_axial"],
        "aero_v_inplane  (m/s)":    d["aero_v_inplane"],
        "aero_Q_drag    (N·m)":     d["aero_Q_drag"],
        "aero_Q_drive   (N·m)":     d["aero_Q_drive"],
        "Q_net          (N·m)":     d["Q_net"],
        "omega_rotor   (rad/s)":    d["omega_rotor"],
        "collective_deg   (°)":     d["collective_deg"],
        "tilt_lon        (norm)":   d["tilt_lon"],
        "tilt_lat        (norm)":   d["tilt_lat"],
        "tether_ext       (m)":     d["tether_ext"],
    }

    print()
    print("=" * 90)
    print("WINDOW STATISTICS  (mean ± std  [min … max])")
    print("=" * 90)

    header = f"{'Variable':<30}" + "".join(f"  {lbl:<26}" for lbl, _, _ in windows)
    print(header)
    print("-" * 90)

    for var_name, values in cols.items():
        row = f"{var_name:<30}"
        for _, t_lo, t_hi in windows:
            s = stats_window(values, t, t_lo, t_hi)
            row += f"  {s['mean']:+8.3f} ±{s['std']:6.3f}  [{s['min']:+8.3f}…{s['max']:+8.3f}]"
        print(row)

    print()
    print("=" * 90)
    print("HYPOTHESIS TESTS")
    print("=" * 90)

    # H1: Does thrust drop around t=30?
    T_pre = stats_window(d["aero_T"], t, 20.0, 28.0)
    T_dip = stats_window(d["aero_T"], t, 28.0, 33.0)
    T_post = stats_window(d["aero_T"], t, 33.0, 40.0)
    print(f"\nH1 — Thrust drop at dip?")
    print(f"     T pre-dip  : {T_pre['mean']:+.2f} N")
    print(f"     T at dip   : {T_dip['mean']:+.2f} N   (Δ = {T_dip['mean']-T_pre['mean']:+.2f} N)")
    print(f"     T post-dip : {T_post['mean']:+.2f} N")
    verdict_T = "YES" if abs(T_dip['mean'] - T_pre['mean']) > 5.0 else "NO"
    print(f"     → {verdict_T}: {'significant drop' if verdict_T=='YES' else 'no significant change'}")

    # H2: Does tether spring dominate?
    tf_pre = stats_window(d["tether_force"], t, 20.0, 28.0)
    tf_dip = stats_window(d["tether_force"], t, 28.0, 33.0)
    print(f"\nH2 — Tether spring force changes?")
    print(f"     Tether force pre-dip : {tf_pre['mean']:+.2f} N")
    print(f"     Tether force at dip  : {tf_dip['mean']:+.2f} N   (Δ = {tf_dip['mean']-tf_pre['mean']:+.2f} N)")
    print(f"     Tether ext pre-dip   : {stats_window(d['tether_ext'], t, 20.0, 28.0)['mean']:+.4f} m")
    print(f"     Tether ext at dip    : {stats_window(d['tether_ext'], t, 28.0, 33.0)['mean']:+.4f} m")
    verdict_tf = "YES" if abs(tf_dip['mean'] - tf_pre['mean']) > 10.0 else "NO"
    print(f"     → {verdict_tf}: tether spring {'changes significantly' if verdict_tf=='YES' else 'roughly constant'}")

    # H3: Collective pitch changes?
    c_pre = stats_window(d["collective_deg"], t, 20.0, 28.0)
    c_dip = stats_window(d["collective_deg"], t, 28.0, 33.0)
    print(f"\nH3 — Collective pitch changes at dip?")
    print(f"     Collective pre-dip : {c_pre['mean']:+.3f}°")
    print(f"     Collective at dip  : {c_dip['mean']:+.3f}°   (Δ = {c_dip['mean']-c_pre['mean']:+.3f}°)")
    verdict_c = "YES" if abs(c_dip['mean'] - c_pre['mean']) > 1.0 else "NO"
    print(f"     → {verdict_c}: collective {'steps' if verdict_c=='YES' else 'unchanged'}")

    # H4: Rotor spin changes?
    om_pre = stats_window(d["omega_rotor"], t, 20.0, 28.0)
    om_dip = stats_window(d["omega_rotor"], t, 28.0, 33.0)
    print(f"\nH4 — Rotor spin changes?")
    print(f"     ω pre-dip : {om_pre['mean']:+.3f} rad/s")
    print(f"     ω at dip  : {om_dip['mean']:+.3f} rad/s   (Δ = {om_dip['mean']-om_pre['mean']:+.3f} rad/s)")
    verdict_om = "YES" if abs(om_dip['mean'] - om_pre['mean']) > 1.0 else "NO"
    print(f"     → {verdict_om}: spin {'changes' if verdict_om=='YES' else 'stable'}")

    # H5: Net vertical force oscillation (spring-mass resonance)?
    nFz_pre = stats_window(d["net_Fz"], t, 20.0, 28.0)
    nFz_dip = stats_window(d["net_Fz"], t, 28.0, 33.0)
    print(f"\nH5 — Net vertical force sign flip (spring-mass overshoot)?")
    print(f"     net_Fz pre-dip : mean={nFz_pre['mean']:+.2f} N  std={nFz_pre['std']:.2f} N")
    print(f"     net_Fz at dip  : mean={nFz_dip['mean']:+.2f} N  std={nFz_dip['std']:.2f} N")

    # Find zero-crossings of net_Fz in window 15-45s to detect oscillation
    nFz = d["net_Fz"]
    crossings = []
    for i in range(1, len(t)):
        if 15.0 <= t[i] <= 45.0:
            if nFz[i-1] * nFz[i] < 0:   # sign change
                crossings.append(t[i])
    if crossings:
        periods = [crossings[i+1] - crossings[i-1]
                   for i in range(1, len(crossings)-1, 2)]
        avg_period = sum(periods) / len(periods) if periods else float("nan")
        print(f"     → Net Fz zero-crossings at: {[f'{tt:.2f}s' for tt in crossings]}")
        if not math.isnan(avg_period):
            freq = 1.0 / avg_period
            print(f"     → Estimated oscillation: period={avg_period:.2f}s  freq={freq:.3f}Hz")
            # Compare to spring-mass natural frequency: f = 1/(2π) * sqrt(K/M)
            K = 300.0; M = 5.0
            f_n = (1.0 / (2.0 * math.pi)) * math.sqrt(K / M)
            print(f"     → Spring-mass natural freq: sqrt(K/M)/(2π) = sqrt({K}/{M})/(2π) = {f_n:.3f} Hz")
            print(f"     → Match: {'YES (spring-mass resonance)' if abs(freq - f_n) < 0.1 * f_n else 'NO'}")
    else:
        print(f"     → No zero-crossings found in 15–45s window")

    # H6: v_axial sign change (autorotation stall)?
    vax_pre = stats_window(d["aero_v_axial"], t, 20.0, 28.0)
    vax_dip = stats_window(d["aero_v_axial"], t, 28.0, 33.0)
    print(f"\nH6 — Axial wind reversal at dip?")
    print(f"     v_axial pre-dip : {vax_pre['mean']:+.3f} m/s  (positive = through disk)")
    print(f"     v_axial at dip  : {vax_dip['mean']:+.3f} m/s")
    verdict_vax = "YES" if vax_pre['mean'] * vax_dip['mean'] < 0 else "NO"
    print(f"     → {verdict_vax}: {'sign reversal — hub moving into wind' if verdict_vax=='YES' else 'no reversal'}")

    print()


# ---------------------------------------------------------------------------
# Figure
# ---------------------------------------------------------------------------

def plot_analysis(d: dict, out_path: Path) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
        import matplotlib.patches as mpatches
    except ImportError:
        print("[ERROR] matplotlib not installed.", file=sys.stderr)
        return

    t = d["t_sim"]

    # Focus window: 5 s before / 20 s after the dip
    T_DIP = 30.0
    FOCUS_LO = T_DIP - 15.0
    FOCUS_HI = T_DIP + 20.0

    def idx_range():
        return [i for i, ti in enumerate(t) if FOCUS_LO <= ti <= FOCUS_HI]

    idxs = idx_range()
    if not idxs:
        print("[WARNING] No data in focus window — plotting full range")
        idxs = list(range(len(t)))

    def sub(col):
        return [col[i] for i in idxs]

    tt = sub(t)

    fig = plt.figure(figsize=(16, 22))
    fig.suptitle(
        f"RAWES 30-second Dip — Diagnostic Analysis\n"
        f"Focus window: {FOCUS_LO:.0f} – {FOCUS_HI:.0f} s  |  dip reference: t = {T_DIP:.0f} s",
        fontsize=13, fontweight="bold",
    )
    gs = gridspec.GridSpec(7, 1, hspace=0.55, top=0.93, bottom=0.04,
                           left=0.09, right=0.97)

    def _ax(idx, title, ylabel):
        ax = fig.add_subplot(gs[idx])
        ax.set_title(title, fontsize=9, loc="left", fontweight="bold")
        ax.set_ylabel(ylabel, fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=7)
        # Shade dip zone
        ax.axvspan(T_DIP - 2, T_DIP + 3, alpha=0.12, color="red", label="_nolegend_")
        # Dip marker
        ax.axvline(T_DIP, color="red", linewidth=0.8, linestyle="--", alpha=0.5)
        return ax

    # ── 1. Hub vertical position & velocity ───────────────────────────────────
    ax = _ax(0, "Hub altitude Z (ENU) and vertical velocity", "m / m·s⁻¹")
    ax.plot(tt, sub(d["hub_pos_z"]),  color="#1f77b4", linewidth=1.2, label="Z altitude (m)")
    ax2 = ax.twinx()
    ax2.plot(tt, sub(d["hub_vel_z"]), color="#d62728", linewidth=0.9, linestyle="--",
             label="Vz (m/s)")
    ax2.axhline(0, color="black", linewidth=0.5, alpha=0.3)
    ax2.set_ylabel("Vz (m/s)", fontsize=8, color="#d62728")
    ax2.tick_params(labelsize=7)
    lines1, labs1 = ax.get_legend_handles_labels()
    lines2, labs2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labs1 + labs2, fontsize=7, loc="upper right")

    # ── 2. Force budget: thrust, tether, weight, net ─────────────────────────
    ax = _ax(1, "Vertical force budget  (positive = upward)", "N")
    ax.plot(tt, sub(d["F_z"]),           color="#2ca02c", linewidth=1.0, label="Fz thrust (N)")
    ax.plot(tt, sub(d["tether_force"]),  color="#ff7f0e", linewidth=1.0, linestyle="-.",
            label="Tether spring force (N)")
    ax.axhline(49.05, color="grey", linewidth=0.8, linestyle=":", label="Weight 49.05 N")
    ax.plot(tt, sub(d["net_Fz"]),        color="black",  linewidth=1.0, linestyle="--",
            label="Net Fz (thrust+tether−weight)")
    ax.axhline(0, color="black", linewidth=0.5, alpha=0.3)
    ax.legend(fontsize=7, loc="upper right", ncol=2)

    # ── 3. Relative wind components ───────────────────────────────────────────
    ax = _ax(2, "Relative wind at rotor disk", "m/s")
    ax.plot(tt, sub(d["aero_v_axial"]),   color="#17becf", linewidth=1.0,
            label="v_axial (through disk, +up)")
    ax.plot(tt, sub(d["aero_v_inplane"]), color="#8c564b", linewidth=1.0,
            label="v_inplane (tangential to disk)")
    ax.plot(tt, sub(d["aero_v_i"]),       color="#bcbd22", linewidth=0.8, linestyle="--",
            label="v_i induced (actuator disk)")
    ax.axhline(0, color="black", linewidth=0.5, alpha=0.3)
    ax.legend(fontsize=7, loc="upper right")

    # ── 4. Torque balance ─────────────────────────────────────────────────────
    ax = _ax(3, "Rotor torque balance  (Q_drag + Q_drive = net spin acceleration)", "N·m")
    ax.plot(tt, sub(d["aero_Q_drag"]),  color="#e377c2", linewidth=1.0, label="Q_drag")
    ax.plot(tt, sub(d["aero_Q_drive"]), color="#ff7f0e", linewidth=1.0, label="Q_drive")
    ax.plot(tt, sub(d["Q_net"]),        color="black",   linewidth=1.2, linestyle="--",
            label="Q_net = Q_drag + Q_drive")
    ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)
    ax2 = ax.twinx()
    ax2.plot(tt, sub(d["omega_rotor"]), color="#9467bd", linewidth=0.8, linestyle="-.",
             label="ω rotor (rad/s)")
    ax2.set_ylabel("ω rotor (rad/s)", fontsize=8, color="#9467bd")
    ax2.tick_params(labelsize=7)
    lines1, labs1 = ax.get_legend_handles_labels()
    lines2, labs2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labs1 + labs2, fontsize=7, loc="upper right")

    # ── 5. Collective / cyclic pitch ──────────────────────────────────────────
    ax = _ax(4, "Swashplate commands (ArduPilot outputs decoded)", "")
    ax.plot(tt, sub(d["collective_deg"]), color="#1f77b4", linewidth=1.0, label="Collective (°)")
    ax.set_ylabel("Collective (°)", fontsize=8)
    ax2 = ax.twinx()
    ax2.plot(tt, sub(d["tilt_lon"]), color="#ff7f0e", linewidth=0.9, linestyle="--",
             label="tilt_lon (norm)")
    ax2.plot(tt, sub(d["tilt_lat"]), color="#2ca02c", linewidth=0.9, linestyle="--",
             label="tilt_lat (norm)")
    ax2.axhline(0, color="black", linewidth=0.5, alpha=0.3)
    ax2.set_ylabel("Tilt (−1…+1)", fontsize=8)
    ax2.tick_params(labelsize=7)
    lines1, labs1 = ax.get_legend_handles_labels()
    lines2, labs2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labs1 + labs2, fontsize=7, loc="upper right")

    # ── 6. Tether extension ───────────────────────────────────────────────────
    ax = _ax(5, "Tether extension (hub_pos_z − 50 m rest length)", "m")
    ax.plot(tt, sub(d["tether_ext"]),   color="#ff7f0e", linewidth=1.0,
            label="Tether extension (m)")
    ax2 = ax.twinx()
    ax2.plot(tt, sub(d["tether_force"]), color="#d62728", linewidth=0.9, linestyle="--",
             label="Tether spring force (N) = K × ext")
    ax2.set_ylabel("Tether force (N)", fontsize=8, color="#d62728")
    ax2.tick_params(labelsize=7)
    lines1, labs1 = ax.get_legend_handles_labels()
    lines2, labs2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labs1 + labs2, fontsize=7, loc="upper right")

    # ── 7. Net vertical force — zoom to show oscillation ─────────────────────
    ax = _ax(6, "Net vertical force  (Fz + tether − weight)  — oscillation check", "N")
    ax.plot(tt, sub(d["net_Fz"]), color="black", linewidth=0.9,
            label="Net Fz (N)")
    ax.axhline(0, color="red", linewidth=0.8, linestyle="--", alpha=0.6,
               label="Zero (hover equilibrium)")
    # Mark zero-crossings
    nFz = d["net_Fz"]
    for i in range(1, len(t)):
        if FOCUS_LO <= t[i] <= FOCUS_HI and nFz[i-1] * nFz[i] < 0:
            ax.axvline(t[i], color="blue", linewidth=0.7, alpha=0.5, linestyle=":")
    ax.set_xlabel("Simulation time  t_sim (s)", fontsize=8)
    ax.legend(fontsize=7, loc="upper right")

    fig.savefig(str(out_path), dpi=130)
    plt.close(fig)
    print(f"Saved figure → {out_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(
        description="Diagnose the 30-second dip in RAWES guided flight telemetry",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("telemetry_csv", nargs="?",
                   default="telemetry.csv",
                   type=Path,
                   help="Path to telemetry.csv from mediator --telemetry-log")
    p.add_argument("--out", "-o", type=Path, default=None,
                   help="Output PNG (default: <csv_stem>_dip_analysis.png)")
    args = p.parse_args()

    csv_path = args.telemetry_csv
    out_path = args.out or csv_path.with_name(csv_path.stem + "_dip_analysis.png")

    data = load(csv_path)
    data = derive(data)
    print_summary(data)
    plot_analysis(data, out_path)


if __name__ == "__main__":
    main()
