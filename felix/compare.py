"""
Compare digitized original thesis figures with simulation output.

Loads CSVs from csv_original/ (digitized from PNGs) and csv_simulated/
(from simulate.py), computes deviations, and prints a summary report.
"""

import numpy as np
import os
import sys


def load_csv(path):
    """Load CSV with header row, return dict of column_name -> array."""
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    return {name: data[name] for name in data.dtype.names}


def interpolate_to_common(t_orig, y_orig, t_sim, y_sim, n_pts=300):
    """Interpolate both signals to a common time grid and return them."""
    t_min = max(t_orig[0], t_sim[0])
    t_max = min(t_orig[-1], t_sim[-1])
    if t_max <= t_min:
        return None, None, None
    t_common = np.linspace(t_min, t_max, n_pts)
    y_o = np.interp(t_common, t_orig, y_orig)
    y_s = np.interp(t_common, t_sim, y_sim)
    return t_common, y_o, y_s


def compute_metrics(y_orig, y_sim):
    """Compute comparison metrics between two signals."""
    diff = y_sim - y_orig

    # Filter out NaN
    valid = ~(np.isnan(y_orig) | np.isnan(y_sim))
    if valid.sum() < 5:
        return {"status": "insufficient_data"}

    y_o = y_orig[valid]
    y_s = y_sim[valid]
    d = diff[valid]

    # Amplitude comparison
    amp_orig = (np.max(y_o) - np.min(y_o)) / 2.0
    amp_sim  = (np.max(y_s) - np.min(y_s)) / 2.0
    amp_ratio = amp_sim / amp_orig if amp_orig > 1e-6 else float("inf")

    # Mean offset
    mean_orig = np.mean(y_o)
    mean_sim  = np.mean(y_s)

    # RMSE
    rmse = np.sqrt(np.mean(d**2))

    # Normalized RMSE (relative to signal range)
    sig_range = np.max(y_o) - np.min(y_o)
    nrmse = rmse / sig_range if sig_range > 1e-6 else float("inf")

    # Cross-correlation (shape similarity, 1.0 = perfect)
    y_o_z = y_o - np.mean(y_o)
    y_s_z = y_s - np.mean(y_s)
    denom = np.sqrt(np.sum(y_o_z**2) * np.sum(y_s_z**2))
    xcorr = np.sum(y_o_z * y_s_z) / denom if denom > 1e-12 else 0.0

    # Best cross-correlation over all phase shifts (phase-invariant)
    # Handles cyclic signals where digitizer phase may be shifted
    n = len(y_o_z)
    if n > 10 and denom > 1e-12:
        cc_full = np.correlate(y_o_z, y_s_z, mode="full")
        best_xcorr = np.max(cc_full) / denom
    else:
        best_xcorr = max(xcorr, 0.0)

    # Spectral amplitude comparison (phase-invariant)
    # Compare FFT magnitude spectra
    fft_o = np.abs(np.fft.rfft(y_o_z))
    fft_s = np.abs(np.fft.rfft(y_s_z))
    # Normalize both to unit energy
    norm_o = np.sqrt(np.sum(fft_o**2))
    norm_s = np.sqrt(np.sum(fft_s**2))
    if norm_o > 1e-12 and norm_s > 1e-12:
        spectral_corr = np.sum(fft_o / norm_o * fft_s / norm_s)
    else:
        spectral_corr = 0.0

    # Max absolute error
    max_err = np.max(np.abs(d))

    return {
        "status": "ok",
        "amp_orig": amp_orig,
        "amp_sim": amp_sim,
        "amp_ratio": amp_ratio,
        "mean_orig": mean_orig,
        "mean_sim": mean_sim,
        "rmse": rmse,
        "nrmse": nrmse,
        "xcorr": xcorr,
        "best_xcorr": best_xcorr,
        "spectral_corr": spectral_corr,
        "max_err": max_err,
    }


def fmt_metric(m):
    """Format a metrics dict as a one-line summary."""
    if m.get("status") != "ok":
        return "  ** %s" % m.get("status", "unknown")
    return (
        "  amp=%.3f  spec=%.3f  best_xc=%.3f  xc=%.3f  NRMSE=%.0f%%"
        % (m["amp_ratio"], m["spectral_corr"], m["best_xcorr"],
           m["xcorr"], m["nrmse"] * 100))


def grade(m):
    """Return a letter grade based on metrics (uses best available metric)."""
    if m.get("status") != "ok":
        return "?"
    # Use spectral correlation and best xcorr (phase-invariant) for grading
    sc = m.get("spectral_corr", 0)
    bxc = m.get("best_xcorr", 0)
    amp = m.get("amp_ratio", 0)
    best = max(sc, bxc)
    amp_ok = 0.5 < amp < 2.0
    if best > 0.90 and amp_ok:
        return "A"
    elif best > 0.75 and amp_ok:
        return "B"
    elif best > 0.50 and amp_ok:
        return "C"
    elif best > 0.30:
        return "D"
    else:
        return "F"


# ---------------------------------------------------------------------------
# Figure-specific comparisons
# ---------------------------------------------------------------------------

def compare_fig10(orig_dir, sim_dir):
    """Compare Fig 10 blade force data."""
    print("=" * 70)
    print("FIGURE 10 -- Blade forces during one pitching cycle")
    print("=" * 70)

    quantities = ["v_ap", "alpha", "beta", "F_ex", "F_ey", "F_ez"]
    all_grades = []

    for qty in quantities:
        orig_path = os.path.join(orig_dir, "orig_fig10_%s.csv" % qty)
        sim_path  = os.path.join(sim_dir, "sim_fig10_%s.csv" % qty)

        if not os.path.exists(orig_path):
            print("\n  [%s] original CSV not found" % qty)
            continue
        if not os.path.exists(sim_path):
            print("\n  [%s] simulated CSV not found" % qty)
            continue

        orig = load_csv(orig_path)
        sim  = load_csv(sim_path)

        print("\n  [%s]" % qty)
        for blade in ["blade_1", "blade_2", "blade_3"]:
            if blade not in orig or blade not in sim:
                print("    %-8s -- missing data" % blade)
                continue

            t_c, y_o, y_s = interpolate_to_common(
                orig["time"], orig[blade],
                sim["time"], sim[blade])

            if t_c is None:
                print("    %-8s -- no time overlap" % blade)
                continue

            m = compute_metrics(y_o, y_s)
            g = grade(m)
            all_grades.append(g)
            print("    %-8s [%s] %s" % (blade, g, fmt_metric(m)))

    return all_grades


def compare_fig14(orig_dir, sim_dir):
    """Compare Fig 14 controller tracking data."""
    print("\n" + "=" * 70)
    print("FIGURE 14 -- Controller performance")
    print("=" * 70)

    panels = ["a", "b", "c", "d"]
    all_grades = []

    for panel in panels:
        orig_path = os.path.join(orig_dir, "orig_fig14_%s.csv" % panel)
        sim_path  = os.path.join(sim_dir, "sim_fig14_%s.csv" % panel)

        if not os.path.exists(orig_path):
            print("\n  [panel %s] original CSV not found" % panel)
            continue
        if not os.path.exists(sim_path):
            print("\n  [panel %s] simulated CSV not found" % panel)
            continue

        orig = load_csv(orig_path)
        sim  = load_csv(sim_path)

        print("\n  [panel %s]" % panel)
        for col in ["beta", "error"]:
            if col not in orig or col not in sim:
                print("    %-10s -- missing data" % col)
                continue

            t_c, y_o, y_s = interpolate_to_common(
                orig["time"], orig[col],
                sim["time"], sim[col])

            if t_c is None:
                print("    %-10s -- no time overlap" % col)
                continue

            m = compute_metrics(y_o, y_s)
            g = grade(m)
            all_grades.append(g)
            print("    %-10s [%s] %s" % (col, g, fmt_metric(m)))

    return all_grades


def compare_fig15(orig_dir, sim_dir):
    """Compare Fig 15 modelling error data."""
    print("\n" + "=" * 70)
    print("FIGURE 15 -- Modelling error: open-loop vs closed-loop")
    print("=" * 70)

    panels = ["open_loop", "closed_loop"]
    all_grades = []

    for panel in panels:
        orig_path = os.path.join(orig_dir, "orig_fig15_%s.csv" % panel)
        sim_path  = os.path.join(sim_dir, "sim_fig15_%s.csv" % panel)

        if not os.path.exists(orig_path):
            print("\n  [%s] original CSV not found" % panel)
            continue
        if not os.path.exists(sim_path):
            print("\n  [%s] simulated CSV not found" % panel)
            continue

        orig = load_csv(orig_path)
        sim  = load_csv(sim_path)

        print("\n  [%s]" % panel)
        for col in ["beta", "error"]:
            if col not in orig or col not in sim:
                print("    %-10s -- missing data" % col)
                continue

            t_c, y_o, y_s = interpolate_to_common(
                orig["time"], orig[col],
                sim["time"], sim[col])

            if t_c is None:
                print("    %-10s -- no time overlap" % col)
                continue

            m = compute_metrics(y_o, y_s)
            g = grade(m)
            all_grades.append(g)
            print("    %-10s [%s] %s" % (col, g, fmt_metric(m)))

    return all_grades


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    orig_dir = "csv_original"
    sim_dir  = "csv_simulated"

    if not os.path.isdir(orig_dir):
        print("Run digitize.py first to create %s/" % orig_dir)
        sys.exit(1)
    if not os.path.isdir(sim_dir):
        print("Run simulate.py first to create %s/" % sim_dir)
        sys.exit(1)

    all_grades = []
    all_grades += compare_fig10(orig_dir, sim_dir)
    all_grades += compare_fig14(orig_dir, sim_dir)
    all_grades += compare_fig15(orig_dir, sim_dir)

    # Summary
    print("\n" + "=" * 70)
    print("OVERALL SUMMARY")
    print("=" * 70)

    grade_map = {"A": 4, "B": 3, "C": 2, "D": 1, "F": 0, "?": -1}
    valid = [g for g in all_grades if g != "?"]
    if valid:
        counts = {}
        for g in valid:
            counts[g] = counts.get(g, 0) + 1
        print("  Grade distribution: %s" %
              "  ".join("%s=%d" % (g, counts.get(g, 0))
                        for g in ["A", "B", "C", "D", "F"] if g in counts))
        avg = np.mean([grade_map[g] for g in valid])
        print("  Average GPA: %.2f / 4.00" % avg)

    print("\nMetric key:")
    print("  amp:      amplitude ratio (sim/orig, 1.0 = perfect)")
    print("  spec:     spectral correlation (FFT magnitude, phase-invariant)")
    print("  best_xc:  best cross-correlation over all phase shifts")
    print("  xc:       zero-lag cross-correlation")
    print("  NRMSE:    RMSE normalized by signal range")
    print("\nGrade thresholds (uses best of spec/best_xc):")
    print("  A: best > 0.90 and 0.5 < amp < 2.0")
    print("  B: best > 0.75 and 0.5 < amp < 2.0")
    print("  C: best > 0.50 and 0.5 < amp < 2.0")
    print("  D: best > 0.30")
    print("  F: best <= 0.30")


if __name__ == "__main__":
    main()
