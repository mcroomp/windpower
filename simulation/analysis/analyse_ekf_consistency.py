#!/usr/bin/env python3
"""
analyse_ekf_consistency.py — Post-flight EKF vs. physics divergence analysis.

Loads two data sources from the same test run:

    simulation/telemetry.csv      400 Hz physics ground truth (mediator)
    simulation/flight_data.json   MAVLink captures (EKF output during hold loop)

For each EKF sample (LOCAL_POSITION_NED / ATTITUDE), the nearest-in-time
mediator sample is looked up and residuals are computed:

    pos_residual   = |EKF pos_NED − mediator pos_NED|         [m]
    att_residual   = |EKF roll/pitch − mediator rpy_roll/pitch| [deg]

Divergence events are detected as contiguous windows where pos_residual
exceeds POS_DIVERGE_M.  For each event the mediator state in the
preceding look-back window is printed as a trigger report.

Produces a multi-panel PNG (--out) with:
    • EKF vs. mediator position (N, E, D)
    • Position residual with event markers
    • EKF vs. mediator attitude (roll, pitch)
    • Physics health: thrust, tether tension, omega_spin, altitude
    • Force balance: net vertical force (should ≈ 0 in steady orbit)

Usage
-----
    python3 simulation/analysis/analyse_ekf_consistency.py
    python3 simulation/analysis/analyse_ekf_consistency.py \\
        --telemetry  /path/to/telemetry.csv \\
        --flight     /path/to/flight_data.json \\
        --out        ekf_consistency.png
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
POS_DIVERGE_M   = 5.0    # m — threshold for a "divergence event"
ATT_DIVERGE_DEG = 10.0   # deg — threshold for attitude divergence flag
LOOKBACK_S      = 5.0    # s — window before event onset to report triggers
GRAVITY_MS2     = 9.81   # m/s²
ROTOR_MASS_KG   = 5.0    # kg (from mediator.py)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_telemetry(path: Path) -> dict:
    """Load mediator telemetry CSV → dict of np.ndarray, one per column."""
    import csv
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    if not rows:
        raise ValueError(f"Empty telemetry file: {path}")
    keys = list(rows[0].keys())
    data = {k: np.array([float(r[k]) for r in rows]) for k in keys}
    print(f"  Telemetry: {len(rows)} rows × {len(keys)} columns  "
          f"(t={data['t_sim'][0]:.1f}..{data['t_sim'][-1]:.1f} s)")
    return data


def _load_flight_data(path: Path) -> dict:
    """Load flight_data.json → parsed arrays."""
    raw = json.loads(path.read_text(encoding="utf-8"))
    out = {}
    for key in ("pos_history", "attitude_history", "servo_history", "rc_history"):
        if key in raw:
            out[key] = np.array(raw[key], dtype=float)
    out["events"]    = raw.get("events", {})
    out["anchor_ned"] = np.array(raw.get("anchor_ned", [0.0, 0.0, 0.0]))
    print(f"  Flight data: "
          f"pos={len(out.get('pos_history', []))}  "
          f"att={len(out.get('attitude_history', []))}  "
          f"rc={len(out.get('rc_history', []))}  "
          f"events={list(out['events'].keys())}")
    return out


def _mediator_to_pos_ned(tel: dict, home_ned_z: float) -> np.ndarray:
    """
    Return mediator hub NED position offset to EKF origin [N, E, D] (m).

    Mediator hub_pos_x/y/z are NED relative to the simulation anchor origin.
    home_ned_z is hub_pos_z at t=0 (the EKF origin NED Z, which is negative
    for an airborne hub).  We subtract it from D so that the origin matches
    ArduPilot LOCAL_POSITION_NED (EKF home = hub's initial position).
    N and E are compared raw; origin_offset in main() handles any residual XY bias.
    """
    N = tel["hub_pos_x"]              # NED X = North
    E = tel["hub_pos_y"]              # NED Y = East
    D = tel["hub_pos_z"] - home_ned_z  # NED Z offset to EKF home (home_ned_z < 0)
    return np.column_stack([N, E, D])   # (T, 3)


def _find_time_offset(tel: dict, fd: dict, home_ned_z: float) -> float:
    """
    Find the time offset δ such that  tel["t_sim"] + δ ≈ flight_data t_rel.

    Strategy: mediator t_sim is wall-clock seconds since mediator start.
    flight_data t_rel is seconds since hold-loop start (inside the test).
    The offset is: t_sim_at_hold_start − 0.

    We align by finding the mediator sample whose pos_NED is closest to the
    first flight_data pos sample (t_rel ≈ 0).  This is valid because the
    hold loop starts immediately after arming — the position at t_rel=0 is
    a deterministic function of hub state, not affected by GPS Glitch.

    Returns δ = t_rel − t_sim  (add this to t_sim to get t_rel).
    """
    if "pos_history" not in fd or len(fd["pos_history"]) == 0:
        return 0.0

    # EKF position at t_rel = 0
    pos_fd0 = fd["pos_history"][0, 1:4]   # (N, E, D)

    # Mediator pos_NED at each step
    med_ned = _mediator_to_pos_ned(tel, home_ned_z)   # (T, 3)
    dists   = np.linalg.norm(med_ned - pos_fd0, axis=1)
    best_idx = int(np.argmin(dists))
    best_dist = dists[best_idx]

    t_sim_at_hold = tel["t_sim"][best_idx]
    delta = 0.0 - t_sim_at_hold   # t_rel = t_sim + delta

    print(f"  Time alignment: hold start ≈ t_sim={t_sim_at_hold:.1f} s  "
          f"(nearest-pos match, residual={best_dist:.2f} m)")
    return delta


def _nearest_lookup(ref_times: np.ndarray, query_times: np.ndarray,
                    values: np.ndarray) -> np.ndarray:
    """For each query time, return the value at the nearest ref time."""
    idx = np.searchsorted(ref_times, query_times, side="left")
    idx = np.clip(idx, 0, len(ref_times) - 1)
    left  = np.clip(idx - 1, 0, len(ref_times) - 1)
    right = idx
    closer_left = np.abs(ref_times[left] - query_times) < np.abs(ref_times[right] - query_times)
    idx  = np.where(closer_left, left, right)
    return values[idx]


def _detect_events(times: np.ndarray, signal: np.ndarray,
                   threshold: float, min_gap_s: float = 2.0
                   ) -> list[dict]:
    """
    Detect contiguous windows where signal > threshold.

    Returns list of dicts: {onset_t, peak_t, peak_val, duration_s}.
    """
    above = signal > threshold
    events = []
    in_event = False
    onset_i  = 0
    for i, a in enumerate(above):
        if a and not in_event:
            in_event = True
            onset_i  = i
        elif (not a or i == len(above) - 1) and in_event:
            in_event = False
            seg = signal[onset_i:i]
            peak_i = onset_i + int(np.argmax(seg))
            ev = {
                "onset_t":    float(times[onset_i]),
                "peak_t":     float(times[peak_i]),
                "peak_val":   float(signal[peak_i]),
                "duration_s": float(times[min(i, len(times)-1)] - times[onset_i]),
            }
            # Merge with previous if gap is small
            if events and ev["onset_t"] - (events[-1]["onset_t"] + events[-1]["duration_s"]) < min_gap_s:
                prev = events[-1]
                prev["duration_s"] = ev["onset_t"] + ev["duration_s"] - prev["onset_t"]
                prev["peak_val"]   = max(prev["peak_val"], ev["peak_val"])
                prev["peak_t"]     = ev["peak_t"] if ev["peak_val"] > prev["peak_val"] else prev["peak_t"]
            else:
                events.append(ev)
    return events


def _trigger_report(onset_t_rel: float, tel: dict, delta: float) -> dict:
    """
    Summarise mediator state in [onset_t_rel − LOOKBACK_S, onset_t_rel].

    delta: offset to convert t_sim → t_rel  (t_rel = t_sim + delta).
    """
    t_sim_onset = onset_t_rel - delta
    t_sim_start = t_sim_onset - LOOKBACK_S

    mask = (tel["t_sim"] >= t_sim_start) & (tel["t_sim"] <= t_sim_onset)
    if not np.any(mask):
        return {}

    report = {}

    # Thrust
    T = tel["aero_T"][mask]
    report["thrust_mean_N"]  = float(np.mean(T))
    report["thrust_min_N"]   = float(np.min(T))
    report["thrust_drop_N"]  = float(np.max(T) - np.min(T))

    # Tether
    slack     = tel["tether_slack"][mask].astype(bool)
    tension   = tel["tether_tension"][mask]
    report["tether_slack_fraction"] = float(np.mean(slack))
    report["tether_tension_mean_N"] = float(np.mean(tension[~slack])) if np.any(~slack) else 0.0
    report["tether_tension_max_N"]  = float(np.max(tension))

    # Omega spin
    omega = tel["omega_rotor"][mask]
    report["omega_mean_rads"]  = float(np.mean(omega))
    report["omega_drop_rads"]  = float(np.max(omega) - np.min(omega))

    # Net upward force (NED: negate Z forces to get upward; subtract gravity)
    weight = ROTOR_MASS_KG * GRAVITY_MS2
    F_vert_up = -(tel["F_z"][mask] + tel["tether_fz"][mask])
    report["net_vert_force_mean_N"] = float(np.mean(F_vert_up) - weight)
    report["net_vert_force_std_N"]  = float(np.std(F_vert_up))

    # Hub altitude (NED Z is negative when airborne; altitude = -hub_pos_z)
    alt = -tel["hub_pos_z"][mask]
    report["altitude_mean_m"] = float(np.mean(alt))
    report["altitude_min_m"]  = float(np.min(alt))

    # Attitude
    roll_deg  = np.degrees(tel["rpy_roll"][mask])
    pitch_deg = np.degrees(tel["rpy_pitch"][mask])
    report["roll_max_deg"]  = float(np.max(np.abs(roll_deg)))
    report["pitch_max_deg"] = float(np.max(np.abs(pitch_deg)))

    return report


def _print_summary(pos_events: list, att_events: list,
                   pos_res: np.ndarray, att_res_roll: np.ndarray,
                   att_res_pitch: np.ndarray) -> None:
    print("\n" + "═" * 70)
    print("  EKF CONSISTENCY SUMMARY")
    print("═" * 70)

    print(f"\n  Position residual  (|EKF − physics|):")
    print(f"    median  {np.median(pos_res):6.2f} m")
    print(f"    mean    {np.mean(pos_res):6.2f} m")
    print(f"    max     {np.max(pos_res):6.2f} m")
    print(f"    > {POS_DIVERGE_M:.0f} m    {100*np.mean(pos_res > POS_DIVERGE_M):.1f}% of samples")

    print(f"\n  Attitude residual  (|EKF − physics| in body frame):")
    print(f"    roll  : median {np.median(att_res_roll):5.1f}°   max {np.max(att_res_roll):5.1f}°")
    print(f"    pitch : median {np.median(att_res_pitch):5.1f}°   max {np.max(att_res_pitch):5.1f}°")

    print(f"\n  Divergence events  (pos_residual > {POS_DIVERGE_M:.0f} m):")
    if not pos_events:
        print("    none detected ✓")
    else:
        for i, ev in enumerate(pos_events, 1):
            print(f"    #{i}  onset={ev['onset_t']:+.1f} s  "
                  f"peak={ev['peak_val']:.1f} m  duration={ev['duration_s']:.1f} s")

    print(f"\n  Attitude divergence events  (roll or pitch > {ATT_DIVERGE_DEG:.0f}°):")
    if not att_events:
        print("    none detected ✓")
    else:
        for i, ev in enumerate(att_events, 1):
            print(f"    #{i}  onset={ev['onset_t']:+.1f} s  "
                  f"peak={ev['peak_val']:.1f}°  duration={ev['duration_s']:.1f} s")

    print()


def _print_trigger_reports(pos_events: list, tel: dict, delta: float) -> None:
    if not pos_events:
        return
    print("═" * 70)
    print("  TRIGGER REPORTS (mediator state in window before each event)")
    print("═" * 70)
    for i, ev in enumerate(pos_events, 1):
        rpt = _trigger_report(ev["onset_t"], tel, delta)
        if not rpt:
            continue
        print(f"\n  Event #{i}  onset t_rel={ev['onset_t']:+.1f} s  "
              f"(mediator t_sim≈{ev['onset_t'] - delta:.1f} s)")
        print(f"    Thrust       : mean={rpt['thrust_mean_N']:.1f} N   "
              f"min={rpt['thrust_min_N']:.1f} N   "
              f"drop={rpt['thrust_drop_N']:.1f} N")
        print(f"    Tether       : slack={100*rpt['tether_slack_fraction']:.0f}%   "
              f"tension_mean={rpt['tether_tension_mean_N']:.1f} N   "
              f"tension_max={rpt['tether_tension_max_N']:.1f} N")
        print(f"    Spin         : mean={rpt['omega_mean_rads']:.2f} rad/s   "
              f"drop={rpt['omega_drop_rads']:.2f} rad/s")
        print(f"    Net vert.F   : mean={rpt['net_vert_force_mean_N']:+.1f} N   "
              f"std={rpt['net_vert_force_std_N']:.1f} N")
        print(f"    Altitude     : mean={rpt['altitude_mean_m']:.1f} m   "
              f"min={rpt['altitude_min_m']:.1f} m")
        print(f"    Attitude     : |roll|_max={rpt['roll_max_deg']:.1f}°   "
              f"|pitch|_max={rpt['pitch_max_deg']:.1f}°")

        # Diagnosis
        flags = []
        if rpt["thrust_drop_N"] > 20:
            flags.append(f"thrust spike ΔT={rpt['thrust_drop_N']:.0f} N")
        if rpt["tether_slack_fraction"] > 0.1:
            flags.append(f"tether slack {100*rpt['tether_slack_fraction']:.0f}% of window")
        if rpt["omega_drop_rads"] > 2.0:
            flags.append(f"spin drop Δω={rpt['omega_drop_rads']:.1f} rad/s")
        if abs(rpt["net_vert_force_mean_N"]) > 15:
            flags.append(f"mean vert imbalance {rpt['net_vert_force_mean_N']:+.0f} N")
        if rpt["altitude_min_m"] < 5.0:
            flags.append(f"low altitude min={rpt['altitude_min_m']:.1f} m")
        if rpt["roll_max_deg"] > 30 or rpt["pitch_max_deg"] > 30:
            flags.append(f"large attitude error "
                         f"(roll={rpt['roll_max_deg']:.0f}° pitch={rpt['pitch_max_deg']:.0f}°)")

        if flags:
            print(f"    ⚠ Likely triggers: {', '.join(flags)}")
        else:
            print(f"    ✓ Physics appeared nominal — likely pure GPS Glitch / EKF yaw reset")
    print()


def _plot(tel: dict, fd: dict, pos_res: np.ndarray,
          att_res_roll: np.ndarray, att_res_pitch: np.ndarray,
          t_ekf: np.ndarray, t_med_aligned: np.ndarray,
          pos_events: list, att_events: list,
          med_ned: np.ndarray,
          out_path: Path,
          freeze_end_t_rel: float = 0.0) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig, axes = plt.subplots(6, 1, figsize=(14, 26), sharex=False)
    fig.suptitle("EKF vs. Physics Consistency Analysis", fontsize=14, fontweight="bold")

    # Helper: mark events on an axis
    def _mark_events(ax, events, color, label_prefix):
        for ev in events:
            ax.axvspan(ev["onset_t"], ev["onset_t"] + ev["duration_s"],
                       color=color, alpha=0.15)
            ax.axvline(ev["onset_t"], color=color, lw=1.2, ls="--", alpha=0.6)

    # Helper: draw freeze-end marker, extending x-axis left if needed
    def _mark_freeze(ax):
        # Ensure the freeze line is visible even if x-axis starts later
        xl = ax.get_xlim()
        if freeze_end_t_rel < xl[0]:
            ax.set_xlim(left=freeze_end_t_rel - 1.0)
        ax.axvline(freeze_end_t_rel, color="purple", lw=1.5, ls=":",
                   label="Freeze ends / physics starts", zorder=5)
        ax.text(freeze_end_t_rel + 0.3, ax.get_ylim()[1] * 0.97,
                "freeze\nends", color="purple", fontsize=6.5,
                va="top", ha="left")

    # ── Panel 1: Position comparison ─────────────────────────────────────
    ax = axes[0]
    ax.set_title("Position: EKF (solid) vs. Mediator (dashed)")
    ekf_N = fd["pos_history"][:, 1]
    ekf_E = fd["pos_history"][:, 2]
    ekf_D = fd["pos_history"][:, 3]

    # Downsample mediator (400 Hz → every 40th sample ≈ 10 Hz)
    ds = 40
    t_med_ds  = t_med_aligned[::ds]
    med_N_ds  = med_ned[::ds, 0]
    med_E_ds  = med_ned[::ds, 1]
    med_D_ds  = med_ned[::ds, 2]

    ax.plot(t_ekf, ekf_N, "C0",   lw=1.5, label="EKF N")
    ax.plot(t_ekf, ekf_E, "C1",   lw=1.5, label="EKF E")
    ax.plot(t_ekf, ekf_D, "C2",   lw=1.5, label="EKF D")
    ax.plot(t_med_ds, med_N_ds, "C0--", lw=0.8, alpha=0.7, label="med N")
    ax.plot(t_med_ds, med_E_ds, "C1--", lw=0.8, alpha=0.7, label="med E")
    ax.plot(t_med_ds, med_D_ds, "C2--", lw=0.8, alpha=0.7, label="med D")
    _mark_events(ax, pos_events, "red", "GPS")
    _mark_freeze(ax)
    ax.set_ylabel("Position NED [m]")
    ax.legend(ncol=6, fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.3)

    # ── Panel 2: Position residual ────────────────────────────────────────
    ax = axes[1]
    ax.set_title(f"Position Residual  (threshold={POS_DIVERGE_M:.0f} m, "
                 f"events={len(pos_events)})")
    ax.plot(t_ekf, pos_res, "C3", lw=1.2, label="|pos_res|")
    ax.axhline(POS_DIVERGE_M, color="red", ls=":", lw=1.2, label=f"{POS_DIVERGE_M} m threshold")
    _mark_events(ax, pos_events, "red", "GPS")
    for ev in pos_events:
        ax.annotate(f"{ev['peak_val']:.1f} m",
                    xy=(ev["peak_t"], ev["peak_val"]),
                    xytext=(ev["peak_t"] + 1, ev["peak_val"] + 1),
                    fontsize=7, color="red")
    _mark_freeze(ax)
    ax.set_ylabel("Residual [m]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Panel 3: Attitude comparison ──────────────────────────────────────
    ax = axes[2]
    ax.set_title("Attitude: EKF (solid) vs. Mediator reported (dashed)  [deg]")
    ekf_roll  = fd["attitude_history"][:, 1]
    ekf_pitch = fd["attitude_history"][:, 2]
    t_att     = fd["attitude_history"][:, 0]

    # Mediator rpy (nearest lookup at MAVLink times)
    t_sim_att = t_att - (t_ekf[0] - t_att[0]) * 0   # already t_rel
    # Re-compute at attitude times
    t_rel_med = t_med_aligned
    med_roll_deg  = np.degrees(tel["rpy_roll"])
    med_pitch_deg = np.degrees(tel["rpy_pitch"])
    # Nearest lookup
    med_roll_at_att  = _nearest_lookup(t_rel_med, t_att, med_roll_deg)
    med_pitch_at_att = _nearest_lookup(t_rel_med, t_att, med_pitch_deg)

    ax.plot(t_att, ekf_roll,  "C0",   lw=1.5, label="EKF roll")
    ax.plot(t_att, ekf_pitch, "C1",   lw=1.5, label="EKF pitch")
    ax.plot(t_att, med_roll_at_att,  "C0--", lw=0.8, alpha=0.7, label="med roll (sent)")
    ax.plot(t_att, med_pitch_at_att, "C1--", lw=0.8, alpha=0.7, label="med pitch (sent)")
    _mark_events(ax, pos_events, "red", "GPS")
    _mark_events(ax, att_events, "orange", "ATT")
    _mark_freeze(ax)
    ax.set_ylabel("Angle [deg]")
    ax.legend(ncol=4, fontsize=7)
    ax.grid(True, alpha=0.3)

    # ── Panel 4: Physics health ───────────────────────────────────────────
    ax = axes[3]
    ax.set_title("Physics Health  (mediator ground truth, 10 Hz downsample)")

    t_ds  = t_med_aligned[::ds]
    T_ds  = tel["aero_T"][::ds]
    ten_ds = tel["tether_tension"][::ds]
    om_ds  = tel["omega_rotor"][::ds]
    alt_ds = -tel["hub_pos_z"][::ds]   # NED Z → altitude (positive up)

    ax2 = ax.twinx()
    l1, = ax.plot(t_ds, T_ds,  "C0", lw=1.2, label="Thrust [N]")
    l2, = ax.plot(t_ds, ten_ds,"C3", lw=1.0, label="Tether tension [N]")
    l3, = ax2.plot(t_ds, om_ds, "C2", lw=1.0, ls="--", label="ω_spin [rad/s]")
    l4, = ax2.plot(t_ds, alt_ds,"C4", lw=1.0, ls="-.", label="Altitude [m]")
    _mark_events(ax, pos_events, "red", "GPS")
    _mark_freeze(ax)
    ax.set_ylabel("Force [N]")
    ax2.set_ylabel("ω [rad/s]  or  alt [m]")
    ax.legend(handles=[l1, l2, l3, l4], fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.3)

    # ── Panel 5: Vertical force balance ──────────────────────────────────
    ax = axes[4]
    ax.set_title("Net Vertical Force Balance  "
                 "(−F_z_aero − F_z_tether − mg,  NED sign;  should ≈ 0 in orbit)")
    weight = ROTOR_MASS_KG * GRAVITY_MS2
    # In NED, F_z is positive-down. Negate all NED-Z forces to get net upward force.
    F_vert_net = -(tel["F_z"][::ds] + tel["tether_fz"][::ds]) - weight
    ax.plot(t_ds, F_vert_net, "C5", lw=1.0, label="net F_z [N]")
    ax.axhline(0.0, color="gray", lw=0.8, ls=":")
    ax.fill_between(t_ds, F_vert_net, 0.0,
                    where=F_vert_net > 0, alpha=0.25, color="green", label="net up")
    ax.fill_between(t_ds, F_vert_net, 0.0,
                    where=F_vert_net < 0, alpha=0.25, color="red", label="net down")
    _mark_events(ax, pos_events, "red", "GPS")
    _mark_freeze(ax)
    ax.set_ylabel("Net F_z [N]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Panel 6: RC override commands ─────────────────────────────────────
    ax = axes[5]
    ax.set_title("RC Override Commands sent to ArduPilot  (1500 = neutral)")
    if "rc_history" in fd and len(fd["rc_history"]) > 0:
        t_rc   = fd["rc_history"][:, 0]
        ch1    = fd["rc_history"][:, 1]   # roll rate
        ch2    = fd["rc_history"][:, 2]   # pitch rate
        ch4    = fd["rc_history"][:, 4]   # yaw rate (index 4 = column 4 of [t,ch1,ch2,ch3,ch4])
        ax.plot(t_rc, ch1, "C0", lw=1.0, label="CH1 roll rate")
        ax.plot(t_rc, ch2, "C1", lw=1.0, label="CH2 pitch rate")
        ax.plot(t_rc, ch4, "C4", lw=1.0, label="CH4 yaw rate", alpha=0.7)
        ax.axhline(1500, color="gray", lw=0.8, ls=":", label="neutral 1500")
        ax.axhline(2000, color="gray", lw=0.5, ls="--", alpha=0.4)
        ax.axhline(1000, color="gray", lw=0.5, ls="--", alpha=0.4)
        ax.set_ylim(950, 2050)
    else:
        ax.text(0.5, 0.5, "No rc_history (re-run test to capture)",
                ha="center", va="center", transform=ax.transAxes, color="gray")
    _mark_events(ax, pos_events, "red", "GPS")
    _mark_events(ax, att_events, "orange", "ATT")
    _mark_freeze(ax)
    ax.set_ylabel("PWM [µs]")
    ax.set_xlabel("t_rel  [s]  (relative to hold-loop start)")
    ax.legend(ncol=4, fontsize=7)
    ax.grid(True, alpha=0.3)

    # Event legend patch
    red_patch    = mpatches.Patch(color="red",    alpha=0.3, label="GPS divergence event")
    orange_patch = mpatches.Patch(color="orange", alpha=0.3, label="Attitude divergence event")
    purple_patch = mpatches.Patch(color="purple", alpha=0.3, label="Freeze ends / physics starts")
    fig.legend(handles=[red_patch, orange_patch, purple_patch],
               loc="lower center", ncol=3, fontsize=9, frameon=True)

    plt.tight_layout(rect=[0, 0.02, 1, 1])
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    print(f"  Plot saved → {out_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    sim_dir = Path(__file__).resolve().parents[1]

    parser = argparse.ArgumentParser(
        description="Analyse EKF vs. physics consistency after a test_acro_hold run.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    log_dir = sim_dir / "logs"
    parser.add_argument("--telemetry",  type=Path,
                        default=log_dir / "telemetry.csv",
                        help="Mediator telemetry CSV (400 Hz physics ground truth)")
    parser.add_argument("--flight",     type=Path,
                        default=log_dir / "flight_data.json",
                        help="MAVLink capture JSON (EKF output from hold loop)")
    parser.add_argument("--out",        type=Path,
                        default=log_dir / "ekf_consistency.png",
                        help="Output plot path")
    parser.add_argument("--home-ned-z", type=float, default=None,
                        help="Hub home NED Z [m] (negative = above ground; "
                             "default: read from telemetry first sample)")
    args = parser.parse_args()

    # -- Load ----------------------------------------------------------------
    print("Loading data...")
    if not args.telemetry.exists():
        print(f"ERROR: telemetry file not found: {args.telemetry}", file=sys.stderr)
        sys.exit(1)
    if not args.flight.exists():
        print(f"ERROR: flight data not found: {args.flight}", file=sys.stderr)
        sys.exit(1)

    tel = _load_telemetry(args.telemetry)
    fd  = _load_flight_data(args.flight)

    # Home NED Z: hub_pos_z at t=0 (negative for airborne hub).
    # Used as the EKF-origin reference for the D axis.
    home_ned_z = args.home_ned_z
    if home_ned_z is None:
        home_ned_z = float(tel["hub_pos_z"][0])
        print(f"  home_ned_z = {home_ned_z:.3f} m  (first mediator sample)")

    # -- Time alignment ------------------------------------------------------
    print("\nAligning time bases...")
    delta   = _find_time_offset(tel, fd, home_ned_z)
    # t_rel = t_sim + delta;  t_rel axis is relative to hold-loop start
    t_med_aligned = tel["t_sim"] + delta

    # -- Compute residuals ---------------------------------------------------
    print("\nComputing residuals...")
    med_ned       = _mediator_to_pos_ned(tel, home_ned_z)   # (T, 3)
    origin_offset = np.zeros(3)   # filled in below once EKF home is known

    if "pos_history" in fd and len(fd["pos_history"]) > 0:
        t_ekf   = fd["pos_history"][:, 0]   # t_rel
        ekf_ned = fd["pos_history"][:, 1:4]

        # Mediator NED at EKF sample times (nearest lookup)
        med_N_at_ekf = _nearest_lookup(t_med_aligned, t_ekf, med_ned[:, 0])
        med_E_at_ekf = _nearest_lookup(t_med_aligned, t_ekf, med_ned[:, 1])
        med_D_at_ekf = _nearest_lookup(t_med_aligned, t_ekf, med_ned[:, 2])
        med_ned_at_ekf = np.column_stack([med_N_at_ekf, med_E_at_ekf, med_D_at_ekf])

        # ArduPilot LOCAL_POSITION_NED is relative to EKF home (initial GPS fix).
        # Mediator pos_ned is absolute from world origin.
        # Subtract the offset at t_rel=0 so both traces share the same reference.
        origin_offset = med_ned_at_ekf[0] - ekf_ned[0]
        print(f"  Frame offset at t_rel=0: N={origin_offset[0]:.2f} m  "
              f"E={origin_offset[1]:.2f} m  D={origin_offset[2]:.2f} m  "
              f"(mediator absolute NED − EKF home-relative NED)")
        med_ned_at_ekf -= origin_offset   # now both relative to EKF home

        pos_res = np.linalg.norm(ekf_ned - med_ned_at_ekf, axis=1)
    else:
        print("  WARNING: no pos_history in flight_data — skipping position residual")
        t_ekf   = np.array([0.0, 1.0])
        pos_res = np.zeros(2)

    if "attitude_history" in fd and len(fd["attitude_history"]) > 0:
        t_att     = fd["attitude_history"][:, 0]
        ekf_roll  = fd["attitude_history"][:, 1]
        ekf_pitch = fd["attitude_history"][:, 2]

        med_roll_deg  = np.degrees(tel["rpy_roll"])
        med_pitch_deg = np.degrees(tel["rpy_pitch"])
        med_roll_at_att  = _nearest_lookup(t_med_aligned, t_att, med_roll_deg)
        med_pitch_at_att = _nearest_lookup(t_med_aligned, t_att, med_pitch_deg)

        att_res_roll  = np.abs(ekf_roll  - med_roll_at_att)
        att_res_pitch = np.abs(ekf_pitch - med_pitch_at_att)
        att_res_max   = np.maximum(att_res_roll, att_res_pitch)
    else:
        print("  WARNING: no attitude_history in flight_data — skipping attitude residual")
        t_att  = t_ekf
        att_res_roll  = np.zeros_like(pos_res)
        att_res_pitch = np.zeros_like(pos_res)
        att_res_max   = np.zeros_like(pos_res)

    # -- Detect events -------------------------------------------------------
    pos_events = _detect_events(t_ekf, pos_res, POS_DIVERGE_M)
    att_events = _detect_events(t_att, att_res_max, ATT_DIVERGE_DEG)

    # -- Report --------------------------------------------------------------
    _print_summary(pos_events, att_events, pos_res, att_res_roll, att_res_pitch)
    _print_trigger_reports(pos_events, tel, delta)

    # -- Plot ----------------------------------------------------------------
    # Freeze ends when telemetry begins; convert to t_rel for the marker.
    freeze_end_t_rel = float(tel["t_sim"].min()) + delta

    print("Generating plot...")
    _plot(
        tel              = tel,
        fd               = fd,
        pos_res          = pos_res,
        att_res_roll     = att_res_roll,
        att_res_pitch    = att_res_pitch,
        t_ekf            = t_ekf,
        t_med_aligned    = t_med_aligned,
        pos_events       = pos_events,
        att_events       = att_events,
        med_ned          = med_ned - origin_offset,   # same reference as EKF
        out_path         = args.out,
        freeze_end_t_rel = freeze_end_t_rel,
    )

    print("\nDone.")


if __name__ == "__main__":
    main()
