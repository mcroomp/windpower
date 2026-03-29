#!/usr/bin/env python3
"""
generate_flight_report.py — Offline flight report generator for RAWES simulation.

Reads the CSV telemetry file written by mediator.py (--telemetry-log) and
produces a multi-panel PNG figure with:

  Panel 1  — Hub altitude & position (ENU, from MBDyn state)
  Panel 2  — Hub velocity (ENU)
  Panel 3  — Rotor spin rate (omega_rotor)
  Panel 4  — Aerodynamic thrust & vertical force (Fz)
  Panel 5  — Relative wind: axial and in-plane components
  Panel 6  — Torque balance: drag torque vs driving torque
  Panel 7  — Swashplate inputs: collective (deg), tilt_lon (→ pitch), tilt_lat (→ roll)
  Panel 8  — Attitude reported to ArduPilot: roll, pitch, yaw (deg)
  Panel 9  — Raw servo inputs (normalised, -1..+1)

Usage
-----
    python generate_flight_report.py telemetry.csv [--out flight_report.png]

The telemetry CSV is written by mediator.py when launched with:
    python mediator.py --telemetry-log telemetry.csv
"""

import argparse
import csv
import math
import sys
from pathlib import Path


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_telemetry(csv_path: Path) -> dict:
    """
    Load mediator telemetry CSV into a dict of column-name → list[float].

    Returns an empty dict if the file is missing or has no data rows.
    """
    if not csv_path.exists():
        print(f"[ERROR] File not found: {csv_path}", file=sys.stderr)
        return {}

    with csv_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print(f"[WARNING] No data rows in {csv_path}", file=sys.stderr)
        return {}

    def _col(name):
        return [float(r[name]) for r in rows if r.get(name, "").strip() != ""]

    data = {}
    for col in rows[0].keys():
        try:
            data[col] = _col(col)
        except (ValueError, KeyError):
            pass   # skip columns that can't be parsed as float

    print(f"[INFO] Loaded {len(rows)} rows, {len(data)} columns from {csv_path}")
    return data


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_report(data: dict, out_path: Path) -> None:
    """Render and save the multi-panel flight report."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("[ERROR] matplotlib not installed.  pip install matplotlib", file=sys.stderr)
        sys.exit(1)

    if not data:
        print("[ERROR] No data to plot.", file=sys.stderr)
        sys.exit(1)

    t = data.get("t_sim", [])
    if not t:
        print("[ERROR] 't_sim' column missing.", file=sys.stderr)
        sys.exit(1)

    n_panels = 9
    fig = plt.figure(figsize=(16, 4.0 * n_panels))
    fig.suptitle(
        f"RAWES Simulation — Mediator Telemetry\n{out_path.stem}",
        fontsize=14, fontweight="bold",
    )
    gs = gridspec.GridSpec(n_panels, 1, hspace=0.55, top=0.96, bottom=0.03,
                           left=0.08, right=0.97)

    def _ax(idx, title, ylabel):
        ax = fig.add_subplot(gs[idx])
        ax.set_title(title, fontsize=9, loc="left")
        ax.set_ylabel(ylabel, fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=7)
        return ax

    def _get(name, default=None):
        col = data.get(name, default or [])
        return col if len(col) == len(t) else (default or [None] * len(t))

    # ── Panel 1: Hub position (ENU) ──────────────────────────────────────────
    ax = _ax(0, "Hub position — MBDyn world ENU frame", "m")
    ax.plot(t, _get("hub_pos_x"), linewidth=0.8, label="X (East)")
    ax.plot(t, _get("hub_pos_y"), linewidth=0.8, label="Y (North)")
    ax.plot(t, _get("hub_pos_z"), linewidth=0.8, label="Z (Up / altitude)")
    ax.legend(fontsize=7, loc="upper right")

    # ── Panel 2: Hub velocity (ENU) ──────────────────────────────────────────
    ax = _ax(1, "Hub velocity — MBDyn world ENU frame", "m/s")
    ax.plot(t, _get("hub_vel_x"), linewidth=0.8, label="Vx (East)")
    ax.plot(t, _get("hub_vel_y"), linewidth=0.8, label="Vy (North)")
    ax.plot(t, _get("hub_vel_z"), linewidth=0.8, label="Vz (Up)")
    ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)
    ax.legend(fontsize=7, loc="upper right")

    # ── Panel 3: Rotor spin ───────────────────────────────────────────────────
    ax = _ax(2, "Rotor spin (estimated from hub angular velocity)", "rad/s")
    ax.plot(t, _get("omega_rotor"), color="#9467bd", linewidth=0.8)
    ax.axhline(28.0, color="grey", linewidth=0.6, linestyle="--", alpha=0.6,
               label="Nominal 28 rad/s")
    ax.legend(fontsize=7, loc="upper right")

    # ── Panel 4: Thrust & Fz ─────────────────────────────────────────────────
    ax = _ax(3, "Aerodynamic thrust (BEM) & vertical force sent to MBDyn", "N")
    ax.plot(t, _get("aero_T"),  color="#2ca02c", linewidth=0.8, label="Thrust T (aero model)")
    ax.plot(t, _get("F_z"),     color="#1f77b4", linewidth=0.8, linestyle="--",
            label="Fz world (sent to MBDyn)")
    ax.axhline(49.05, color="red", linewidth=0.6, linestyle=":", alpha=0.7,
               label="Weight 49.05 N")
    ax.legend(fontsize=7, loc="upper right")

    # ── Panel 5: Relative wind ────────────────────────────────────────────────
    ax = _ax(4, "Relative wind at rotor disk", "m/s")
    ax.plot(t, _get("aero_v_axial"),   color="#17becf", linewidth=0.8, label="v_axial (through disk)")
    ax.plot(t, _get("aero_v_inplane"), color="#8c564b", linewidth=0.8, label="v_inplane (in-disk-plane)")
    ax.plot(t, _get("aero_v_i"),       color="#bcbd22", linewidth=0.8, linestyle="--",
            label="v_i (induced, actuator disk)")
    ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)
    ax.legend(fontsize=7, loc="upper right")

    # ── Panel 6: Torque balance ───────────────────────────────────────────────
    ax = _ax(5, "Spin torque balance  (Q_drag + Q_drive = net spin acceleration)", "N·m")
    ax.plot(t, _get("aero_Q_drag"),  color="#e377c2", linewidth=0.8, label="Q_drag (opposes spin)")
    ax.plot(t, _get("aero_Q_drive"), color="#ff7f0e", linewidth=0.8, label="Q_drive (wind-driven)")
    q_drag  = _get("aero_Q_drag",  [0.0] * len(t))
    q_drive = _get("aero_Q_drive", [0.0] * len(t))
    q_net   = [a + b for a, b in zip(q_drag, q_drive)]
    ax.plot(t, q_net, color="black", linewidth=1.0, linestyle="-.", label="Q_net")
    ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)
    ax.legend(fontsize=7, loc="upper right")

    # ── Panel 7: Swashplate commands ──────────────────────────────────────────
    # tilt_lon drives longitudinal disk tilt → ArduPilot interprets as pitch correction
    # tilt_lat drives lateral disk tilt      → ArduPilot interprets as roll correction
    ax = _ax(6, "Swashplate commands (decoded from servo channels)", "")
    collective_deg = [math.degrees(v) for v in _get("collective_rad", [0.0] * len(t))]
    ax.plot(t, collective_deg,         color="#1f77b4", linewidth=0.8, label="Collective (°)")
    ax2 = ax.twinx()
    ax2.plot(t, _get("tilt_lon"), color="#ff7f0e", linewidth=0.8, linestyle="--",
             label="tilt_lon → pitch (norm)")
    ax2.plot(t, _get("tilt_lat"), color="#2ca02c", linewidth=0.8, linestyle="--",
             label="tilt_lat → roll (norm)")
    ax2.set_ylabel("Tilt (−1…+1)", fontsize=8)
    ax2.tick_params(labelsize=7)
    lines1, labs1 = ax.get_legend_handles_labels()
    lines2, labs2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labs1 + labs2, fontsize=7, loc="upper right")
    ax.set_ylabel("Collective (°)", fontsize=8)

    # ── Panel 8: Attitude reported to ArduPilot ───────────────────────────────
    # roll/pitch are tether-relative (0° at equilibrium); yaw = velocity heading.
    # tilt_lat (panel 7) drives roll correction; tilt_lon drives pitch correction.
    ax = _ax(7, "Attitude sent to ArduPilot EKF  (tether-relative: 0° = equilibrium)", "deg")
    roll_deg  = [math.degrees(v) for v in _get("rpy_roll",  [0.0] * len(t))]
    pitch_deg = [math.degrees(v) for v in _get("rpy_pitch", [0.0] * len(t))]
    yaw_deg   = [math.degrees(v) for v in _get("rpy_yaw",   [0.0] * len(t))]
    ax.plot(t, roll_deg,  color="#2ca02c", linewidth=0.8, label="roll  (← tilt_lat)")
    ax.plot(t, pitch_deg, color="#ff7f0e", linewidth=0.8, label="pitch (← tilt_lon)")
    ax.plot(t, yaw_deg,   color="#9467bd", linewidth=0.8, linestyle=":", label="yaw  (velocity heading)")
    ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)
    ax.legend(fontsize=7, loc="upper right")

    # ── Panel 9: Raw servo inputs ─────────────────────────────────────────────
    ax = _ax(8, "Raw servo inputs from ArduPilot SITL (normalised −1…+1)", "")
    ax.plot(t, _get("servo_s1"),  linewidth=0.8, label="S1")
    ax.plot(t, _get("servo_s2"),  linewidth=0.8, label="S2")
    ax.plot(t, _get("servo_s3"),  linewidth=0.8, label="S3")
    ax.plot(t, _get("servo_esc"), linewidth=0.8, linestyle="--", label="ESC (anti-rotation motor)")
    ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)
    ax.set_xlabel("Simulation time (s)", fontsize=8)
    ax.legend(fontsize=7, loc="upper right")

    # Add ramp shading to all axes (first ramp_time=5s)
    for axes in fig.get_axes():
        axes.axvspan(0, 5.0, alpha=0.07, color="orange", label="_nolegend_")

    fig.savefig(str(out_path), dpi=120)
    plt.close(fig)
    print(f"[INFO] Saved -> {out_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args():
    p = argparse.ArgumentParser(
        description="Generate RAWES flight report from mediator telemetry CSV",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("telemetry_csv", type=Path,
                   help="Path to telemetry.csv written by mediator.py --telemetry-log")
    p.add_argument("--out", "-o", type=Path, default=None,
                   help="Output PNG path (default: <telemetry_csv_stem>_report.png)")
    return p.parse_args()


def main():
    args = _parse_args()
    csv_path = args.telemetry_csv
    out_path = args.out or csv_path.with_name(csv_path.stem + "_report.png")

    data = load_telemetry(csv_path)
    plot_report(data, out_path)


if __name__ == "__main__":
    main()
