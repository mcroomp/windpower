"""
sg6042_polar.py — SG6042 airfoil polar analysis via NeuralFoil (XFOIL surrogate).

Sweeps:
  - AoA −8° to +16° at 3 Reynolds numbers:
      Re=127,000  (bench-test condition from FlapRotor Blade Design doc)
      Re=490,000  (flight condition at r_cp = 1.833 m, omega=20 rad/s, c=0.20 m)
      Re=900,000  (blade tip at r_tip = 2.5 m in flight)

Extracts for each Re:
  - CL0         (CL at AoA = 0°)
  - CL_alpha    (slope of linear CL-AoA fit over the linear range)
  - CD0         (CD at CL = 0, i.e. at AoA ~ zero-lift angle)
  - zero-lift AoA
  - stall AoA   (AoA at CL peak)
  - L/D_max

Compares against beaupoil_2026.yaml values and de_schutter_2018.yaml values.
"""

import math
import sys
from pathlib import Path

import numpy as np
import neuralfoil as nf

# SG6042 .dat file from aerosandbox database
import site as _site
_ASB_DB = None
for _sp in _site.getsitepackages():
    _candidate = Path(_sp) / "aerosandbox/geometry/airfoil/airfoil_database/sg6042.dat"
    if _candidate.exists():
        _ASB_DB = _candidate.parent
        break
if _ASB_DB is None:
    raise FileNotFoundError("sg6042.dat not found in any site-packages path")
SG6042_DAT = str(_ASB_DB / "sg6042.dat")

# Analysis conditions
REYNOLDS_NUMBERS = {
    "Re=127k  (bench test, FlapRotor doc)":   127_000,
    "Re=490k  (flight, r_cp=1.833m, ω=20)":  490_000,
    "Re=900k  (flight, r_tip=2.5m, ω=20)":   900_000,
}

AOA_SWEEP = np.linspace(-8.0, 16.0, 121)   # 0.2° resolution

# Current YAML values
YAML_CL0          =  0.43
YAML_CL_ALPHA     =  4.71   # /rad
YAML_CD0          =  0.010
YAML_ALPHA_STALL  = 12.0    # degrees
DS_CL0            =  0.10
DS_CL_ALPHA       =  0.87   # /rad


def _fit_linear_range(aoa_deg: np.ndarray, CL: np.ndarray,
                      aoa_min: float = -4.0, aoa_max: float = 9.0):
    """Least-squares linear fit over the linear CL-AoA range."""
    mask = (aoa_deg >= aoa_min) & (aoa_deg <= aoa_max)
    aoa_r = np.radians(aoa_deg[mask])
    cl    = CL[mask]
    # CL = CL0 + CL_alpha * aoa_rad
    A     = np.column_stack([np.ones_like(aoa_r), aoa_r])
    coef, _, _, _ = np.linalg.lstsq(A, cl, rcond=None)
    return float(coef[0]), float(coef[1])   # CL0, CL_alpha


def _zero_lift_aoa(CL0: float, CL_alpha: float) -> float:
    """AoA (degrees) at CL = 0."""
    if abs(CL_alpha) < 1e-6:
        return 0.0
    return float(np.degrees(-CL0 / CL_alpha))


def run_polar(re: float, label: str) -> dict:
    """Run NeuralFoil polar at given Re, return extracted coefficients."""
    result = nf.get_aero_from_dat_file(
        filename  = SG6042_DAT,
        alpha     = AOA_SWEEP,
        Re        = re,
        n_crit    = 9.0,
        model_size= "xlarge",
    )
    CL = np.array(result["CL"])
    CD = np.array(result["CD"])
    CM = np.array(result["CM"])

    # Linear-range fit
    CL0_fit, CL_alpha_fit = _fit_linear_range(AOA_SWEEP, CL)

    # Zero-lift AoA
    alpha_L0 = _zero_lift_aoa(CL0_fit, CL_alpha_fit)

    # Stall AoA (peak CL, only on positive side)
    pos_mask     = AOA_SWEEP > 0
    peak_idx     = np.argmax(CL[pos_mask])
    alpha_stall  = float(AOA_SWEEP[pos_mask][peak_idx])
    CL_max       = float(CL[pos_mask][peak_idx])

    # CD0 (at CL = 0, i.e. at zero-lift AoA)
    zla_idx = np.argmin(np.abs(AOA_SWEEP - alpha_L0))
    CD0_fit = float(CD[zla_idx])

    # L/D max
    with np.errstate(divide="ignore", invalid="ignore"):
        LD    = np.where(CD > 0.001, CL / CD, 0.0)
    LD_max    = float(np.max(LD))
    LD_max_aoa= float(AOA_SWEEP[np.argmax(LD)])

    return {
        "label":       label,
        "Re":          re,
        "CL0":         CL0_fit,
        "CL_alpha":    CL_alpha_fit,
        "alpha_L0":    alpha_L0,
        "alpha_stall": alpha_stall,
        "CL_max":      CL_max,
        "CD0":         CD0_fit,
        "LD_max":      LD_max,
        "LD_max_aoa":  LD_max_aoa,
        "CL":          CL,
        "CD":          CD,
        "CM":          CM,
    }


def print_results(polars: list[dict]) -> None:
    print("\n── SG6042 polar summary (NeuralFoil XFOIL surrogate) ──")
    print(f"  {'Condition':<42}  {'CL0':>6}  {'CL_α /rad':>9}  {'CD0':>6}  "
          f"{'α_L0 °':>7}  {'α_stall °':>9}  {'CL_max':>6}  {'L/D_max':>7}")
    print("  " + "─" * 105)

    for p in polars:
        print(f"  {p['label']:<42}  {p['CL0']:6.3f}  {p['CL_alpha']:9.3f}  "
              f"{p['CD0']:6.4f}  {p['alpha_L0']:7.2f}  {p['alpha_stall']:9.1f}  "
              f"{p['CL_max']:6.3f}  {p['LD_max']:7.1f}  (at {p['LD_max_aoa']:.1f}°)")

    print()
    print("  Reference values in beaupoil_2026.yaml:")
    print(f"    CL0       = {YAML_CL0:.3f}  (from diyrcwings.com polar at Re≈127k)")
    print(f"    CL_alpha  = {YAML_CL_ALPHA:.3f} /rad  (thin-airfoil 3D correction, 10% reduction)")
    print(f"    CD0       = {YAML_CD0:.4f}")
    print(f"    α_stall   = {YAML_ALPHA_STALL:.1f}°")
    print()
    print("  Reference values in de_schutter_2018.yaml:")
    print(f"    CL0       = {DS_CL0:.3f}")
    print(f"    CL_alpha  = {DS_CL_ALPHA:.3f} /rad")
    print()

    # Show the key comparison
    flight_polar = next(p for p in polars if "490k" in p["label"])
    bench_polar  = next(p for p in polars if "127k" in p["label"])

    print("── Key comparison: NeuralFoil vs YAML values ──")
    params = [
        ("CL0",        "CL0",       ""),
        ("CL_alpha",   "CL_alpha",  " /rad"),
        ("CD0",        "CD0",       ""),
        ("alpha_stall","alpha_stall"," deg"),
    ]

    print(f"  {'Parameter':<12}  {'YAML (127k)':>11}  {'NF Re=127k':>10}  "
          f"{'NF Re=490k':>10}  {'NF Re=900k':>10}")
    print("  " + "─" * 60)

    yaml_vals = {
        "CL0": YAML_CL0, "CL_alpha": YAML_CL_ALPHA,
        "CD0": YAML_CD0, "alpha_stall": YAML_ALPHA_STALL,
    }
    tip_polar = next(p for p in polars if "900k" in p["label"])

    for name, key, unit in params:
        y  = yaml_vals[key]
        b  = bench_polar[key]
        f  = flight_polar[key]
        t  = tip_polar[key]
        print(f"  {name+unit:<12}  {y:>11.4g}  {b:>10.4g}  {f:>10.4g}  {t:>10.4g}")

    print()
    print("── Operating point analysis (flight at r_cp, ω=20 rad/s) ──")
    print(f"  Conditions: chord=0.20m, Re=490k, v_tan=36.67m/s")
    for col_deg in [-10.0, -5.7, 0.0, 5.0, 8.0]:
        # At r_cp=1.833m, omega=20 rad/s: v_tan=36.67, v_axial~8.66 (xi=30°)
        v_tan   = 36.67
        v_axial = 8.66
        phi_rad = math.atan2(v_axial, v_tan)
        phi_deg = math.degrees(phi_rad)
        aoa_deg = phi_deg + col_deg

        # Interpolate NF polar at that AoA
        aoa_idx = np.argmin(np.abs(AOA_SWEEP - aoa_deg))
        cl = float(flight_polar["CL"][aoa_idx])
        cd = float(flight_polar["CD"][aoa_idx])

        v_loc = math.sqrt(v_tan**2 + v_axial**2)
        q     = 0.5 * 1.22 * v_loc**2
        S_w   = 4 * 0.20 * (2.5 - 0.5)     # 4 blades × chord × span
        T_strip = q * S_w * (cl * math.cos(phi_rad) - cd * math.sin(phi_rad))

        print(f"  col={col_deg:+5.1f}°  AoA={aoa_deg:5.1f}°  CL={cl:.3f}  CD={cd:.4f}  "
              f"T(lumped)={max(0, T_strip):.0f} N")

    print()
    print("Note: T(lumped) uses full S_w at single r_cp — the overcounting issue.")
    print(f"  Betz limit at 10m/s, A=18.85m²: {0.5*1.22*100*0.89*18.85:.0f} N")


def plot_polars(polars: list[dict]) -> None:
    try:
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("matplotlib not available")
        return

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("SG6042 Polar — NeuralFoil (XFOIL surrogate)\n"
                 "Compares bench-test Re vs flight Re vs YAML model values",
                 fontsize=12)
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.38)

    colors = ["steelblue", "tomato", "forestgreen"]

    # CL vs AoA
    ax1 = fig.add_subplot(gs[0, 0])
    for p, c in zip(polars, colors):
        ax1.plot(AOA_SWEEP, p["CL"], color=c, label=p["label"].split("(")[0].strip(), lw=1.5)
    # YAML linear model
    aoa_r = np.radians(AOA_SWEEP)
    CL_yaml = np.clip(YAML_CL0 + YAML_CL_ALPHA * aoa_r,
                      -YAML_CL_ALPHA * math.radians(12) + YAML_CL0,
                      YAML_CL_ALPHA * math.radians(12) + YAML_CL0)
    ax1.plot(AOA_SWEEP, YAML_CL0 + YAML_CL_ALPHA * aoa_r,
             "k--", lw=1, label="YAML linear model", alpha=0.6)
    CL_ds = DS_CL0 + DS_CL_ALPHA * aoa_r
    ax1.plot(AOA_SWEEP, CL_ds, "k:", lw=1, label="de_schutter model", alpha=0.6)
    ax1.axhline(0, color="gray", lw=0.5)
    ax1.axvline(0, color="gray", lw=0.5)
    ax1.set_xlabel("AoA [°]"); ax1.set_ylabel("CL"); ax1.set_title("Lift Curve")
    ax1.legend(fontsize=7); ax1.grid(True, alpha=0.3)
    ax1.set_xlim(-8, 16)

    # CD vs AoA
    ax2 = fig.add_subplot(gs[0, 1])
    for p, c in zip(polars, colors):
        ax2.plot(AOA_SWEEP, p["CD"], color=c, lw=1.5)
    ax2.set_xlabel("AoA [°]"); ax2.set_ylabel("CD"); ax2.set_title("Drag Polar (AoA)")
    ax2.grid(True, alpha=0.3); ax2.set_xlim(-8, 16)

    # Drag polar (CL vs CD)
    ax3 = fig.add_subplot(gs[0, 2])
    for p, c in zip(polars, colors):
        ax3.plot(p["CD"], p["CL"], color=c, lw=1.5)
    ax3.set_xlabel("CD"); ax3.set_ylabel("CL"); ax3.set_title("Drag Polar (CL vs CD)")
    ax3.grid(True, alpha=0.3)

    # L/D vs AoA
    ax4 = fig.add_subplot(gs[1, 0])
    for p, c in zip(polars, colors):
        with np.errstate(divide="ignore", invalid="ignore"):
            LD = np.where(p["CD"] > 0.002, p["CL"] / p["CD"], 0.0)
        ax4.plot(AOA_SWEEP, LD, color=c, lw=1.5)
    ax4.axhline(0, color="gray", lw=0.5)
    ax4.set_xlabel("AoA [°]"); ax4.set_ylabel("L/D"); ax4.set_title("Lift-to-Drag Ratio")
    ax4.grid(True, alpha=0.3); ax4.set_xlim(-8, 16)

    # CL_alpha comparison bar chart
    ax5 = fig.add_subplot(gs[1, 1])
    labels = [p["label"].split("(")[0].strip() for p in polars] + ["YAML", "de_Schutter"]
    vals   = [p["CL_alpha"] for p in polars] + [YAML_CL_ALPHA, DS_CL_ALPHA]
    bar_c  = colors + ["gray", "lightgray"]
    x = range(len(labels))
    bars = ax5.bar(x, vals, color=bar_c, alpha=0.8)
    ax5.set_xticks(list(x)); ax5.set_xticklabels(labels, rotation=20, ha="right", fontsize=7)
    ax5.set_ylabel("CL_alpha [/rad]"); ax5.set_title("Lift Slope Comparison")
    ax5.grid(True, alpha=0.3, axis="y")
    for bar, val in zip(bars, vals):
        ax5.text(bar.get_x() + bar.get_width()/2, val + 0.05, f"{val:.2f}",
                 ha="center", va="bottom", fontsize=7)

    # CL0 comparison
    ax6 = fig.add_subplot(gs[1, 2])
    vals0 = [p["CL0"] for p in polars] + [YAML_CL0, DS_CL0]
    bars6 = ax6.bar(x, vals0, color=bar_c, alpha=0.8)
    ax6.set_xticks(list(x)); ax6.set_xticklabels(labels, rotation=20, ha="right", fontsize=7)
    ax6.set_ylabel("CL0"); ax6.set_title("Zero-AoA Lift Coefficient")
    ax6.grid(True, alpha=0.3, axis="y")
    for bar, val in zip(bars6, vals0):
        ax6.text(bar.get_x() + bar.get_width()/2, val + 0.005, f"{val:.3f}",
                 ha="center", va="bottom", fontsize=7)

    out = Path(__file__).parent.parent / "logs" / "sg6042_polar.png"
    fig.savefig(str(out), dpi=130, bbox_inches="tight")
    print(f"Plot saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    print(f"SG6042 .dat file: {SG6042_DAT}")
    polars = []
    for label, re in REYNOLDS_NUMBERS.items():
        print(f"  Running Re={re:.0e}...", end=" ", flush=True)
        polars.append(run_polar(re, label))
        print("done")

    print_results(polars)

    if args.plot:
        plot_polars(polars)
