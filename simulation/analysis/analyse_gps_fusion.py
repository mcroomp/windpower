#!/usr/bin/env python3
"""
analyse_gps_fusion.py — Parse gcs_last_run.log and diagnose EKF3 GPS fusion.

Reads the GCS log written by test_stationary_gps or test_guided_flight and
prints a structured timeline covering:

  • STATUSTEXT events (EKF alignment, GPS detection, origin set, …)
  • EKF flag transitions (when each flag appears or disappears)
  • hp / hv trend (position / velocity variance over time)
  • Whether LOCAL_POSITION_NED was ever received (GPS fused)
  • Diagnosis: which readyToUseGPS() condition is still blocking

Usage
-----
    python3 simulation/analysis/analyse_gps_fusion.py
    python3 simulation/analysis/analyse_gps_fusion.py --log /path/to/gcs.log
    python3 simulation/analysis/analyse_gps_fusion.py --plot
"""

import argparse
import re
import sys
from pathlib import Path

_SIM_DIR = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------------------
# EKF flag definitions (EKF_STATUS_REPORT.flags)
# ---------------------------------------------------------------------------
_EKF_FLAGS = {
    0x0001: "att",
    0x0002: "horiz_vel",
    0x0004: "vert_vel",
    0x0008: "horiz_pos_rel",
    0x0010: "horiz_pos_abs",
    0x0020: "vert_pos_abs",
    0x0040: "terrain_alt",
    0x0080: "CONST_POS_MODE",
    0x0100: "pred_horiz_pos_rel",
    0x0200: "pred_horiz_pos_abs",
    0x0400: "pred_horiz_ok",
}

_FLAG_NAMES_FOR_GPS = {
    "att":            "tiltAlignComplete",
    "horiz_pos_abs":  "GPS position fusing (AID_ABSOLUTE)",
    "CONST_POS_MODE": "AID_NONE (not yet fusing position)",
}

# ---------------------------------------------------------------------------
# Regex patterns for the GCS log
# ---------------------------------------------------------------------------
# Lines look like:
#   HH:MM:SS logger  LEVEL   STATUSTEXT t=4.2s: ...
#   HH:MM:SS logger  LEVEL   EKF t=4.3s  0x00a7  [att horiz_vel …]  hv=0.000 hp=0.000
#   HH:MM:SS logger  LEVEL   ATTITUDE t=4.3s  rpy=(0.1° -0.0° 39.3°)
#   HH:MM:SS logger  LEVEL   GPS FUSED at t=21.0 s: LOCAL_POSITION_NED  N=…

_RE_STATUSTEXT = re.compile(r"STATUSTEXT t=([\d.]+)s:\s*(.+)")
_RE_EKF        = re.compile(
    r"EKF t=([\d.]+)s\s+0x([0-9a-fA-F]+)\s+\[([^\]]*)\]\s+hv=([\d.nan]+)\s+hp=([\d.nan]+)"
)
_RE_ATTITUDE   = re.compile(
    r"ATTITUDE t=([\d.]+)s\s+rpy=\(([-\d.]+)°\s+([-\d.]+)°\s+([-\d.]+)°\)"
)
_RE_GPS_FUSED  = re.compile(r"GPS FUSED at t=([\d.]+)")
_RE_PARAM_SET  = re.compile(r"(EK3_\S+|COMPASS_\S+|ARMING_\S+|FS_EKF\S+)\s+=\s+(\S+)\s+\[critical")


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

class EkfSample:
    __slots__ = ("t", "flags_raw", "flags_set", "hv", "hp")
    def __init__(self, t, flags_raw, flags_set, hv, hp):
        self.t         = t
        self.flags_raw = flags_raw
        self.flags_set = flags_set   # frozenset of flag-name strings
        self.hv        = hv
        self.hp        = hp


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

def parse_log(path: Path):
    statustext = []   # [(t, text)]
    ekf        = []   # [EkfSample]
    attitudes  = []   # [(t, roll, pitch, yaw)]
    gps_fused  = None # t float or None
    params_set = []   # [(name, value)]

    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            # STATUSTEXT
            m = _RE_STATUSTEXT.search(line)
            if m:
                statustext.append((float(m.group(1)), m.group(2).strip()))
                continue

            # EKF_STATUS_REPORT
            m = _RE_EKF.search(line)
            if m:
                t         = float(m.group(1))
                flags_raw = int(m.group(2), 16)
                label_str = m.group(3)
                hv        = float(m.group(4)) if m.group(4) != "nan" else float("nan")
                hp        = float(m.group(5)) if m.group(5) != "nan" else float("nan")
                flags_set = frozenset(label_str.split()) if label_str.strip() else frozenset()
                ekf.append(EkfSample(t, flags_raw, flags_set, hv, hp))
                continue

            # ATTITUDE
            m = _RE_ATTITUDE.search(line)
            if m:
                attitudes.append((
                    float(m.group(1)),
                    float(m.group(2)), float(m.group(3)), float(m.group(4)),
                ))
                continue

            # GPS FUSED
            m = _RE_GPS_FUSED.search(line)
            if m:
                gps_fused = float(m.group(1))
                continue

            # Param set lines (from the setting block)
            m = _RE_PARAM_SET.search(line)
            if m:
                params_set.append((m.group(1), m.group(2)))

    return statustext, ekf, attitudes, gps_fused, params_set


# ---------------------------------------------------------------------------
# Analysis helpers
# ---------------------------------------------------------------------------

def _flag_transitions(ekf_samples):
    """Return list of (t, flag_name, appeared/disappeared)."""
    transitions = []
    prev = frozenset()
    for s in ekf_samples:
        appeared   = s.flags_set - prev
        disappeared = prev - s.flags_set
        for f in sorted(appeared):
            transitions.append((s.t, f, "appeared"))
        for f in sorted(disappeared):
            transitions.append((s.t, f, "cleared"))
        prev = s.flags_set
    return transitions


def _hp_trend(ekf_samples, after_t=None):
    """Return (times, hp_values) optionally filtered to t >= after_t."""
    ts  = [s.t  for s in ekf_samples if (after_t is None or s.t >= after_t) and not _isnan(s.hp)]
    hps = [s.hp for s in ekf_samples if (after_t is None or s.t >= after_t) and not _isnan(s.hp)]
    return ts, hps


def _isnan(x):
    try:
        return x != x
    except Exception:
        return False


def _hp_rate(ts, hps):
    """Linear fit slope (hp/s) over the series."""
    if len(ts) < 2:
        return float("nan")
    n  = len(ts)
    sx = sum(ts); sy = sum(hps)
    sxx = sum(t*t for t in ts); sxy = sum(t*h for t,h in zip(ts,hps))
    denom = n*sxx - sx*sx
    if abs(denom) < 1e-12:
        return 0.0
    return (n*sxy - sx*sy) / denom


# ---------------------------------------------------------------------------
# Printer
# ---------------------------------------------------------------------------

def report(statustext, ekf, attitudes, gps_fused, params_set, log_path):
    W = 72
    print("=" * W)
    print(f"  GPS FUSION ANALYSIS — {log_path.name}")
    print("=" * W)

    # ── Params actually set ───────────────────────────────────────────────
    if params_set:
        print("\n── Params set ──")
        for name, val in params_set:
            print(f"  {name:<28} = {val}")

    # ── STATUSTEXT timeline ───────────────────────────────────────────────
    print("\n── STATUSTEXT timeline ──")
    for t, txt in statustext:
        print(f"  t={t:6.1f}s  {txt}")

    # ── EKF flag transitions ──────────────────────────────────────────────
    print("\n── EKF flag transitions ──")
    transitions = _flag_transitions(ekf)
    if transitions:
        for t, flag, action in transitions:
            marker = "+" if action == "appeared" else "-"
            note   = ""
            if flag == "att" and action == "appeared":
                note = "  ← tiltAlignComplete"
            elif flag == "CONST_POS_MODE" and action == "cleared":
                note = "  ← GPS position fusion started!"
            elif flag == "horiz_pos_abs" and action == "appeared":
                note = "  ← AID_ABSOLUTE (GPS fusing)"
            print(f"  t={t:6.1f}s  {marker} {flag}{note}")
    else:
        print("  (no transitions detected)")

    # ── EKF flags at end of run ───────────────────────────────────────────
    if ekf:
        last = ekf[-1]
        print(f"\n── EKF flags at end (t={last.t:.1f}s) ──")
        print(f"  raw=0x{last.flags_raw:04x}  [{' '.join(sorted(last.flags_set)) or 'none'}]")
        print(f"  hp={last.hp:.4f}  hv={last.hv:.4f}")

    # ── GPS fusion result ─────────────────────────────────────────────────
    print("\n── GPS fusion result ──")
    if gps_fused is not None:
        print(f"  PASS  LOCAL_POSITION_NED received at t={gps_fused:.1f}s")
    else:
        print("  FAIL  LOCAL_POSITION_NED never received")

    # ── hp trend after origin set ─────────────────────────────────────────
    origin_t = None
    for t, txt in statustext:
        if "origin set" in txt:
            if origin_t is None or t < origin_t:
                origin_t = t

    print("\n── hp (horizontal position variance) trend ──")
    if origin_t is not None:
        ts, hps = _hp_trend(ekf, after_t=origin_t)
        if ts:
            rate = _hp_rate(ts, hps)
            print(f"  Origin set at t={origin_t:.1f}s")
            print(f"  hp at origin:   {hps[0]:.4f}")
            print(f"  hp at end:      {hps[-1]:.4f}  (t={ts[-1]:.1f}s)")
            print(f"  hp growth rate: {rate:+.4f} /s")
            if rate > 0.002:
                print("  ⚠  hp growing → GPS NOT fusing position (dead-reckoning drift)")
            elif rate < -0.001:
                print("  ✓  hp shrinking → GPS IS fusing position")
            else:
                print("  ≈  hp roughly flat")
        else:
            print("  (no EKF samples after origin)")
    else:
        print("  (EKF3 origin never set)")

    # ── EKF health resets ─────────────────────────────────────────────────
    healthy_flag_set = {"att", "horiz_vel", "vert_vel", "vert_pos_abs"}
    resets = []
    for i, tr in enumerate(transitions):
        t_tr, flag, action = tr
        if action == "cleared" and flag == "att":
            # Check if all health flags disappeared at the same time
            same_t = [f for tt, f, a in transitions if tt == t_tr and a == "cleared"]
            if healthy_flag_set.issubset(set(same_t)):
                resets.append(t_tr)
    if resets:
        print(f"\n── EKF health resets (all health flags cleared simultaneously) ──")
        for t_r in resets:
            print(f"  t={t_r:6.1f}s  EKF became unhealthy — large GPS innovation or filter divergence")
        print("  This means GPS fusion started but was immediately rejected (position drift)")
        print("  Fix: prevent EKF position drift during AID_NONE phase")

    # ── Diagnosis ─────────────────────────────────────────────────────────
    print("\n── Diagnosis: readyToUseGPS() conditions ──")

    # Determine which conditions are known
    tilt_t = next((t for t,txt in statustext if "tilt alignment complete" in txt), None)
    yaw_t  = next((t for t,txt in statustext if "yaw alignment complete" in txt), None)
    gps_det_t = next((t for t,txt in statustext if "detected" in txt.lower() and "gps" in txt.lower()), None)
    origin_t2 = next((t for t,txt in statustext if "origin set" in txt), None)

    conds = [
        ("getPosXYSource==GPS (EK3_SRC1_POSXY=3)",
         True, "param default — always true"),
        ("tiltAlignComplete",
         tilt_t is not None,
         f"t={tilt_t:.1f}s" if tilt_t else "never seen"),
        ("yawAlignComplete",
         yaw_t is not None,
         f"t={yaw_t:.1f}s" if yaw_t else "never seen"),
        ("gpsGoodToAlign (10s hardcoded + GPS detect)",
         origin_t2 is not None,
         f"inferred true (origin set t={origin_t2:.1f}s)" if origin_t2 else
         f"GPS detected t={gps_det_t:.1f}s but origin not set — check 10s delay" if gps_det_t else
         "GPS never detected"),
        ("validOrigin",
         origin_t2 is not None,
         f"set at t={origin_t2:.1f}s" if origin_t2 else "never set"),
        ("gpsDataToFuse",
         origin_t2 is not None,
         "should be true after validOrigin" if origin_t2 else "blocked by validOrigin"),
        ("delAngBiasLearned",
         gps_fused is not None,   # only known true if GPS actually fused
         "true (GPS fused)" if gps_fused else
         "LIKELY FALSE — most probable blocker if all others are true"),
    ]

    all_ok = True
    for name, ok, note in conds:
        sym = "✓" if ok else "✗"
        if not ok:
            all_ok = False
        print(f"  {sym}  {name}")
        print(f"       {note}")

    if all_ok and gps_fused is None:
        print("\n  All conditions appear true but GPS never fused — check")
        print("  gpsDataToFuse more carefully (buffer timing / GPS rate).")
    elif not all_ok:
        first_fail = next(name for name, ok, note in conds if not ok)
        print(f"\n  First failing condition: {first_fail}")

    # ── Attitude wobble summary ───────────────────────────────────────────
    if attitudes:
        rolls  = [r for _,r,_,_ in attitudes]
        pitchs = [p for _,_,p,_ in attitudes]
        yaws   = [y for _,_,_,y in attitudes]
        yaw_range = max(yaws) - min(yaws)
        print(f"\n── Attitude wobble summary ──")
        print(f"  roll:  min={min(rolls):.1f}°  max={max(rolls):.1f}°  range={max(rolls)-min(rolls):.1f}°")
        print(f"  pitch: min={min(pitchs):.1f}°  max={max(pitchs):.1f}°  range={max(pitchs)-min(pitchs):.1f}°")
        print(f"  yaw:   range={yaw_range:.1f}°  (>30° needed for gyro Z-axis observability)")
        if yaw_range < 30.0 and max(rolls) - min(rolls) < 1.0 and max(pitchs) - min(pitchs) < 1.0:
            print("  ⚠  No significant motion on any axis.")
            print("     delAngBiasLearned may never converge (pure zero-motion case).")
        elif max(rolls) - min(rolls) > 1.0 or max(pitchs) - min(pitchs) > 1.0:
            print("  Roll/pitch present — all three bias axes observable (fast convergence)")
        else:
            print("  Yaw rotation present — bias converges passively via cross-covariance (~35 s)")

    print("\n" + "=" * W)


# ---------------------------------------------------------------------------
# Optional: plot hp and EKF flags over time
# ---------------------------------------------------------------------------

def plot_ekf_timeline(ekf, statustext, gps_fused, out_path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("matplotlib not available; skipping plot")
        return

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("EKF3 GPS Fusion Timeline", fontsize=13)

    ts  = [s.t  for s in ekf]
    hps = [s.hp for s in ekf]
    hvs = [s.hv for s in ekf]

    ax1 = axes[0]
    ax1.plot(ts, hps, label="hp (horiz pos variance)", color="tab:blue")
    ax1.plot(ts, hvs, label="hv (horiz vel variance)", color="tab:orange", alpha=0.7)
    ax1.set_ylabel("Variance")
    ax1.legend(loc="upper left")
    ax1.set_title("EKF position/velocity variance")

    # Mark key STATUSTEXT events
    colors = {"tilt": "green", "yaw": "purple", "GPS": "red", "origin": "blue"}
    for t, txt in statustext:
        color = "gray"
        if "tilt" in txt:       color = "green"
        elif "yaw" in txt:      color = "purple"
        elif "GPS" in txt.upper() or "gps" in txt: color = "red"
        elif "origin" in txt:   color = "blue"
        ax1.axvline(t, color=color, linewidth=1, alpha=0.5)
        ax1.text(t, ax1.get_ylim()[1]*0.9, txt[:20], rotation=90, fontsize=6, color=color)

    if gps_fused is not None:
        ax1.axvline(gps_fused, color="lime", linewidth=2, label=f"GPS fused t={gps_fused:.1f}s")
        ax1.legend(loc="upper left")

    # ── Flags bitmask over time ──
    ax2 = axes[1]
    flag_names = sorted(_EKF_FLAGS.values())
    flag_y     = {f: i for i, f in enumerate(flag_names)}

    for s in ekf:
        for fname in s.flags_set:
            y = flag_y.get(fname, -1)
            if y >= 0:
                ax2.scatter(s.t, y, c="tab:blue", s=2, marker="|")

    ax2.set_yticks(list(flag_y.values()))
    ax2.set_yticklabels(list(flag_y.keys()), fontsize=8)
    ax2.set_xlabel("Test time (s)")
    ax2.set_title("EKF flags active over time")

    plt.tight_layout()
    fig.savefig(str(out_path), dpi=120)
    print(f"  Plot saved → {out_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log",  default=str(_SIM_DIR / "logs" / "gcs_last_run.log"),
                    help="Path to gcs log file (default: simulation/logs/gcs_last_run.log)")
    ap.add_argument("--plot", action="store_true",
                    help="Save EKF timeline plot alongside the log")
    args = ap.parse_args()

    log_path = Path(args.log)
    if not log_path.exists():
        print(f"ERROR: log file not found: {log_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Parsing {log_path} …")
    statustext, ekf, attitudes, gps_fused, params_set = parse_log(log_path)
    print(f"  {len(statustext)} STATUSTEXT  {len(ekf)} EKF samples  "
          f"{len(attitudes)} ATTITUDE  gps_fused={'yes' if gps_fused else 'no'}\n")

    report(statustext, ekf, attitudes, gps_fused, params_set, log_path)

    if args.plot:
        out = log_path.with_suffix(".png")
        plot_ekf_timeline(ekf, statustext, gps_fused, out)


if __name__ == "__main__":
    main()
