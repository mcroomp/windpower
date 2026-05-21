"""
analyze_yaw_tuning.py -- post-run analysis for `run yaw` sessions.

Reads a calibrate.py run-yaw CSV (under simulation/logs/calibrate/) and:
  1. Parses the metadata header (PID gains, timestamps, key params).
  2. Derives yaw rate from the yaw_deg column.
  3. Computes summary metrics (settling, oscillation, steady-state error,
     integrator / output saturation).
  4. Saves a 3-panel PNG plot next to the CSV.
  5. Prints PID tuning hints based on observed behavior.

Usage:
    python simulation/analysis/analyze_yaw_tuning.py <path/to/run_yaw_*.csv>
    python simulation/analysis/analyze_yaw_tuning.py <csv> --window 5
    python simulation/analysis/analyze_yaw_tuning.py <csv> --no-plot
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------------------
# Header / CSV parsing
# ---------------------------------------------------------------------------

def parse_metadata(path: Path) -> dict[str, str]:
    """Pull '# key: value' lines from the top of the CSV into a dict."""
    meta: dict[str, str] = {}
    with open(path, "r") as f:
        for line in f:
            if not line.startswith("#"):
                break
            line = line[1:].strip()
            if ":" in line:
                k, _, v = line.partition(":")
                meta[k.strip()] = v.strip()
    return meta


def load_data(path: Path) -> dict[str, np.ndarray]:
    """Load the CSV body (skipping '#' lines) into a dict of column-name -> ndarray.
    Blank cells become NaN."""
    with open(path, "r") as f:
        # Skip metadata
        for line in f:
            if not line.startswith("#"):
                header = line
                break
        rdr = csv.reader(f)
        cols = [c.strip() for c in header.strip().split(",")]
        rows: list[list[str]] = list(rdr)

    data: dict[str, list[float]] = {c: [] for c in cols}
    for row in rows:
        if len(row) != len(cols):
            continue
        for c, v in zip(cols, row):
            if v == "":
                data[c].append(float("nan"))
            else:
                try:
                    data[c].append(float(v))
                except ValueError:
                    data[c].append(float("nan"))
    return {c: np.array(vals, dtype=float) for c, vals in data.items()}


# ---------------------------------------------------------------------------
# Derived signals
# ---------------------------------------------------------------------------

def unwrap_yaw_deg(yaw_deg: np.ndarray) -> np.ndarray:
    """Unwrap yaw angle so derivative across the -180/+180 seam doesn't
    explode.  Input can have any branch; output is monotonic-friendly."""
    rad = np.deg2rad(yaw_deg.astype(float))
    return np.rad2deg(np.unwrap(rad))


def derive_yaw_rate(t_s: np.ndarray, yaw_deg: np.ndarray,
                     window_s: float = 0.2) -> np.ndarray:
    """Look-back yaw rate (deg/s).  For each sample i, finds the earliest
    sample j such that t[i] - t[j] >= window_s, then returns
    (yaw_u[i] - yaw_u[j]) / (t[i] - t[j]).  Robust to irregular sample
    spacing -- avoids the noise amplification you get from naive central
    differences when consecutive samples are 4 ms apart in a 2 Hz stream."""
    yaw_u = unwrap_yaw_deg(yaw_deg)
    n = len(t_s)
    rate = np.full(n, np.nan)
    j = 0
    for i in range(n):
        # Slide j forward until t[i] - t[j] >= window_s
        while j < i and (t_s[i] - t_s[j]) > window_s:
            j += 1
        # j is now the earliest index inside the window; back off one to
        # include the first sample that's at-or-just-outside the window
        jj = max(0, j - 1)
        dt = t_s[i] - t_s[jj]
        if dt > 1e-6:
            rate[i] = (yaw_u[i] - yaw_u[jj]) / dt
    return rate


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def compute_metrics(df: dict[str, np.ndarray], meta: dict, window_s: float,
                     settle_band_dps: float = 5.0) -> dict:
    """Summarize the yaw-rate response.  `window_s` defines the tail
    window used for steady-state stats; `settle_band_dps` is the +/- band
    used to compute settling time."""
    t = df["t_s"]
    yaw = df["yaw_deg"]
    yaw_rate = derive_yaw_rate(t, yaw)
    t_end = float(t[-1])

    # Steady-state stats: last `window_s` seconds
    tail = t >= max(0.0, t_end - window_s)
    yr_tail = yaw_rate[tail]
    yr_tail = yr_tail[~np.isnan(yr_tail)]

    # Settling time: first time after which |yaw_rate| stays inside band
    # for at least 1 s continuously.
    in_band = np.abs(yaw_rate) < settle_band_dps
    settle_t = float("nan")
    if in_band.any():
        # find the earliest index where every subsequent sample within 1 s
        # is also in band
        dt_target = 1.0
        for i in range(len(t)):
            if not in_band[i]:
                continue
            j = i
            while j < len(t) and t[j] - t[i] < dt_target:
                if not in_band[j]:
                    break
                j += 1
            else:
                # didn't break -> all in band for >= dt_target
                if j < len(t) or (j == len(t) and t[-1] - t[i] >= dt_target):
                    settle_t = float(t[i])
                    break
            if not math.isnan(settle_t):
                break

    # Integrator / output saturation
    imax = float(meta.get("ATC_RAT_YAW_IMAX", 0.7) or 0.7)
    yaw_i = df.get("yaw_i_lua", np.array([]))
    yaw_i = yaw_i[~np.isnan(yaw_i)]
    yaw_out = df.get("yaw_out_lua", np.array([]))
    yaw_out = yaw_out[~np.isnan(yaw_out)]
    i_satur_frac = float(np.mean(np.abs(yaw_i) > 0.98 * imax)) if len(yaw_i) else 0.0
    o_satur_frac = float(np.mean((yaw_out > 1.0) | (yaw_out < 0.0))) if len(yaw_out) else 0.0

    # SERVO4 PWM utilisation
    s4 = df.get("s4_us", np.array([]))
    s4 = s4[~np.isnan(s4)]

    return {
        "duration_s":         t_end - float(t[0]),
        "n_samples":          int(len(t)),
        "yaw_rate_mean_dps":  float(np.nanmean(yaw_rate)),
        "yaw_rate_rms_dps":   float(np.sqrt(np.nanmean(yaw_rate**2))),
        "yaw_rate_max_dps":   float(np.nanmax(np.abs(yaw_rate))),
        "tail_mean_dps":      float(yr_tail.mean()) if len(yr_tail) else float("nan"),
        "tail_std_dps":       float(yr_tail.std())  if len(yr_tail) else float("nan"),
        "tail_max_abs_dps":   float(np.max(np.abs(yr_tail))) if len(yr_tail) else float("nan"),
        "settle_t_s":         settle_t,
        "settle_band_dps":    settle_band_dps,
        "tail_window_s":      window_s,
        "i_min":              float(np.nanmin(yaw_i)) if len(yaw_i) else float("nan"),
        "i_max":              float(np.nanmax(yaw_i)) if len(yaw_i) else float("nan"),
        "i_saturated_frac":   i_satur_frac,
        "out_min":            float(np.nanmin(yaw_out)) if len(yaw_out) else float("nan"),
        "out_max":            float(np.nanmax(yaw_out)) if len(yaw_out) else float("nan"),
        "out_clamped_frac":   o_satur_frac,
        "s4_min_us":          float(np.min(s4)) if len(s4) else float("nan"),
        "s4_max_us":          float(np.max(s4)) if len(s4) else float("nan"),
    }


# ---------------------------------------------------------------------------
# Tuning hints
# ---------------------------------------------------------------------------

def tuning_hints(metrics: dict, meta: dict) -> list[str]:
    """Heuristic suggestions based on observed behavior."""
    hints: list[str] = []
    kp = float(meta.get("ATC_RAT_YAW_P",    0.0) or 0.0)
    ki = float(meta.get("ATC_RAT_YAW_I",    0.0) or 0.0)
    kd = float(meta.get("ATC_RAT_YAW_D",    0.0) or 0.0)
    imax = float(meta.get("ATC_RAT_YAW_IMAX", 0.7) or 0.7)

    tail_max = metrics["tail_max_abs_dps"]
    tail_std = metrics["tail_std_dps"]
    tail_mean = metrics["tail_mean_dps"]
    settled  = not math.isnan(metrics["settle_t_s"])
    i_sat    = metrics["i_saturated_frac"]
    out_clamp = metrics["out_clamped_frac"]

    if not settled:
        hints.append(
            f"[NOT SETTLED] |yaw_rate| never stayed under {metrics['settle_band_dps']:.1f} deg/s "
            f"for >=1 s.  Either the gains are too low (try higher P) or "
            f"the disturbance is too strong for the current motor authority."
        )
    else:
        hints.append(f"[OK] Settled at t={metrics['settle_t_s']:.2f} s "
                     f"(|rate| < {metrics['settle_band_dps']:.1f} deg/s).")

    if tail_std > 5.0 and not math.isnan(tail_std):
        hints.append(
            f"[OSCILLATING] Tail-window std={tail_std:.2f} deg/s -- significant "
            f"residual oscillation.  Reduce P (current {kp:.4f}) by ~30%, or "
            f"add D (current {kd:.4f}; try kp/10 ~ {kp/10:.4f})."
        )
    elif tail_std > 2.0 and tail_std > 0:
        hints.append(
            f"[WIGGLY] Tail std={tail_std:.2f} deg/s.  Minor residual motion -- "
            f"could be fine, but D=0 means no rate damping.  Consider a small D."
        )

    if abs(tail_mean) > 1.0:
        hints.append(
            f"[STEADY-STATE BIAS] Tail mean={tail_mean:+.2f} deg/s.  "
            f"Increase I (current {ki:.4f}); try {max(ki, kp/10):.4f}.  "
            f"Or set H_YAW_TRIM to compensate for the constant disturbance."
        )

    if i_sat > 0.10:
        hints.append(
            f"[INTEGRATOR SATURATING] |yaw_i| > 0.98*IMAX for {i_sat*100:.0f}% of samples.  "
            f"Either IMAX={imax:.2f} is too small (motor still has headroom) or "
            f"the disturbance is overwhelming the actuator.  Check H_YAW_TRIM."
        )

    if out_clamp > 0.10:
        hints.append(
            f"[OUTPUT CLAMPING] PID output outside [0,1] for {out_clamp*100:.0f}% of samples.  "
            f"Motor is at its limit -- either raise SERVO4_MAX (current limit "
            f"may be cutting authority), reduce P, or accept slower response."
        )

    if kd == 0 and tail_std > 3.0:
        hints.append(
            f"[TRY D] D=0 with residual oscillation -- a small D (e.g. {kp/8:.4f}) "
            f"often kills the wiggle.  Watch out for noise amplification: set "
            f"ATC_RAT_YAW_FLTD ~10-20 Hz."
        )

    return hints


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def make_plot(df: dict[str, np.ndarray], metrics: dict, meta: dict,
              csv_path: Path, out_path: Path) -> None:
    """3-panel plot: yaw angle + rate / PID internals / SERVO4 + current."""
    try:
        import matplotlib
        matplotlib.use("Agg")   # no display
        import matplotlib.pyplot as plt
    except ImportError:
        print("  [WARN] matplotlib not installed -- skipping plot")
        return

    t = df["t_s"]
    yaw_deg = unwrap_yaw_deg(df["yaw_deg"])
    yaw_rate = derive_yaw_rate(t, df["yaw_deg"])

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    # Panel 1: yaw angle (unwrapped) + yaw rate
    ax = axes[0]
    ax.set_title(
        f"Yaw PID run -- {csv_path.name}\n"
        f"P={meta.get('ATC_RAT_YAW_P','?')}  I={meta.get('ATC_RAT_YAW_I','?')}  "
        f"D={meta.get('ATC_RAT_YAW_D','?')}  IMAX={meta.get('ATC_RAT_YAW_IMAX','?')}  "
        f"TRIM={meta.get('H_YAW_TRIM','?')}"
    )
    ax.plot(t, yaw_deg, label="yaw (deg, unwrapped)", color="C0")
    ax.set_ylabel("yaw [deg]", color="C0")
    ax.tick_params(axis="y", labelcolor="C0")
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(t, yaw_rate, label="yaw_rate (deg/s)", color="C3", alpha=0.7)
    ax2.axhline(0, color="black", linewidth=0.5)
    band = metrics["settle_band_dps"]
    ax2.axhline(+band, color="grey", linestyle="--", linewidth=0.5)
    ax2.axhline(-band, color="grey", linestyle="--", linewidth=0.5)
    ax2.set_ylabel("yaw rate [deg/s]", color="C3")
    ax2.tick_params(axis="y", labelcolor="C3")
    if not math.isnan(metrics["settle_t_s"]):
        ax.axvline(metrics["settle_t_s"], color="green",
                    linestyle=":", linewidth=1,
                    label=f"settle@{metrics['settle_t_s']:.2f}s")
        ax.legend(loc="upper left")

    # Panel 2: PID internals
    ax = axes[1]
    if "yaw_i_lua" in df:
        ax.plot(t, df["yaw_i_lua"], label="yaw_i (integrator)", color="C1")
    if "yaw_out_lua" in df:
        ax.plot(t, df["yaw_out_lua"], label="yaw_out (pre-clamp)", color="C2")
    imax = float(meta.get("ATC_RAT_YAW_IMAX", 0.7) or 0.7)
    ax.axhline(imax, color="C1", linestyle="--", linewidth=0.5, alpha=0.5,
                label=f"IMAX={imax:.2f}")
    ax.axhline(0.0, color="grey", linestyle="--", linewidth=0.5)
    ax.axhline(1.0, color="C2", linestyle="--", linewidth=0.5, alpha=0.5,
                label="output limit 1.0")
    ax.set_ylabel("PID state")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 3: SERVO4 PWM + current
    ax = axes[2]
    if "s4_us" in df:
        ax.plot(t, df["s4_us"], label="SERVO4 PWM (us)", color="C4")
        ax.set_ylabel("PWM [us]", color="C4")
        ax.tick_params(axis="y", labelcolor="C4")
        ax.grid(True, alpha=0.3)
    ax3 = ax.twinx()
    if "current_a" in df:
        cur = df["current_a"]
        if np.any(~np.isnan(cur)):
            ax3.plot(t, cur, label="current (A)", color="C5", alpha=0.6)
            ax3.set_ylabel("current [A]", color="C5")
            ax3.tick_params(axis="y", labelcolor="C5")
    ax.set_xlabel("t [s]")

    plt.tight_layout()
    plt.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f"  Plot saved: {out_path}")


# ---------------------------------------------------------------------------
# Console output
# ---------------------------------------------------------------------------

def print_report(meta: dict, metrics: dict, hints: list[str]) -> None:
    print()
    print("=" * 70)
    print(f"Yaw PID tuning analysis")
    print("=" * 70)
    print()
    print("Run metadata")
    print("-" * 70)
    keys = ["run_start_local", "duration_s",
            "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D",
            "ATC_RAT_YAW_FF", "ATC_RAT_YAW_IMAX", "H_YAW_TRIM",
            "ATC_RAT_YAW_FLTE", "ATC_RAT_YAW_FLTT", "ATC_RAT_YAW_FLTD",
            "SERVO4_MIN", "SERVO4_MAX"]
    for k in keys:
        if k in meta:
            print(f"  {k:<22} {meta[k]}")
    print()
    print("Yaw rate response")
    print("-" * 70)
    band = metrics["settle_band_dps"]
    win  = metrics["tail_window_s"]
    settle = metrics["settle_t_s"]
    settle_s = f"{settle:.2f} s" if not math.isnan(settle) else f"NEVER (|rate| >= {band} deg/s throughout)"
    print(f"  Duration                {metrics['duration_s']:.2f} s  ({metrics['n_samples']} samples)")
    print(f"  RMS yaw_rate            {metrics['yaw_rate_rms_dps']:+.3f} deg/s  (over the whole run)")
    print(f"  Max |yaw_rate|          {metrics['yaw_rate_max_dps']:.3f} deg/s")
    print(f"  Settle time             {settle_s}  (band +/- {band:.1f} deg/s for >=1 s)")
    print(f"  Tail mean (last {win:.0f} s)   {metrics['tail_mean_dps']:+.3f} deg/s  (steady-state bias)")
    print(f"  Tail std  (last {win:.0f} s)   {metrics['tail_std_dps']:+.3f} deg/s  (oscillation amplitude)")
    print(f"  Tail max |rate|         {metrics['tail_max_abs_dps']:.3f} deg/s")
    print()
    print("PID internals")
    print("-" * 70)
    print(f"  yaw_i range             [{metrics['i_min']:+.4f}, {metrics['i_max']:+.4f}]")
    print(f"  yaw_i saturated         {metrics['i_saturated_frac']*100:.1f}%  (samples at >=0.98 IMAX)")
    print(f"  yaw_out range           [{metrics['out_min']:+.4f}, {metrics['out_max']:+.4f}]")
    print(f"  yaw_out clamped         {metrics['out_clamped_frac']*100:.1f}%  (outside [0, 1])")
    print(f"  SERVO4 PWM range        [{metrics['s4_min_us']:.0f}, {metrics['s4_max_us']:.0f}] us")
    print()
    print("Tuning hints")
    print("-" * 70)
    if not hints:
        print("  (no specific suggestions -- looks reasonable)")
    else:
        for h in hints:
            print(f"  - {h}")
    print()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="run_yaw_*.csv produced by calibrate.py run yaw")
    ap.add_argument("--window", type=float, default=5.0,
                     help="tail-window length in seconds for steady-state stats (default 5)")
    ap.add_argument("--band", type=float, default=5.0,
                     help="settle-band in deg/s -- |yaw_rate| must stay below this for >=1 s "
                          "to count as settled (default 5)")
    ap.add_argument("--no-plot", action="store_true",
                     help="skip the PNG plot (text-only report)")
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f"Error: {path} not found", file=sys.stderr)
        sys.exit(1)

    meta = parse_metadata(path)
    df   = load_data(path)
    metrics = compute_metrics(df, meta, window_s=args.window,
                                settle_band_dps=args.band)
    hints = tuning_hints(metrics, meta)

    print_report(meta, metrics, hints)

    if not args.no_plot:
        out_path = path.with_suffix(".png")
        make_plot(df, metrics, meta, path, out_path)


if __name__ == "__main__":
    main()
