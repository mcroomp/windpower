#!/usr/bin/env python3
"""
analyze_tuning.py -- Diagnose a tuning.csv from `calibrate.py yawmanual`.

Reports:
  * Summary stats (rate, PWM, power, gains)
  * Sign-check  : does PWM correlate with yaw rate the right way?
  * Saturation  : how much time was the motor at SERVO4_MAX?
  * Gyro clip   : was the IMU pegged at +-2000 dps?
  * Runaway     : did rate diverge while PWM saturated?
  * Recommendations for P, I, D, IMAX, SERVO4_MAX

Usage
-----
    python analyze_tuning.py                        # reads ./tuning.csv
    python analyze_tuning.py path/to/x.csv
    python analyze_tuning.py x.csv --servo-min 800 --servo-max 1500
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Diagnostic thresholds
GYRO_SATURATION_DPS    = 1990.0   # Pixhawk 6C IMU saturates near +-2000 dps
DISTURBANCE_DPS        = 5.0      # |yaw_rate| above this is a real disturbance
PWM_SAT_FRAC_HOT       = 0.30     # >30% time at SERVO4_MAX => "gains too high"
RATE_BIAS_DPS          = 5.0      # persistent bias above this => "gains too low"
DESIGN_KP              = 0.015
DESIGN_KI              = 0.010
DESIGN_KD              = 0.0


@dataclass
class Trace:
    t: np.ndarray
    armed: np.ndarray
    yaw_rate: np.ndarray   # deg/s
    target_rate: np.ndarray
    pwm: np.ndarray
    vbat: np.ndarray
    current: np.ndarray
    power: np.ndarray
    yaw_i: np.ndarray | None        # integrator state from Lua (NaN if absent)
    yaw_out_unclamp: np.ndarray | None  # pre-clamp output from Lua
    kp: float
    ki: float
    kd: float
    imax: float
    trim: float
    meta: dict                       # parsed "# key: value" header lines


# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------

def _to_float(s: str) -> float:
    if s is None or s == "":
        return float("nan")
    return float(s)


def _parse_metadata(meta_lines: list[str]) -> dict:
    """Parse '# key: value' lines into a dict.  Values stay as strings here;
    callers cast to float / int as needed.  Lines that don't match the form
    are skipped silently."""
    out = {}
    for line in meta_lines:
        line = line.lstrip("#").strip()
        if not line or ":" not in line:
            continue
        k, _, v = line.partition(":")
        out[k.strip()] = v.strip()
    return out


def load_csv(path: Path) -> Trace:
    # Read once, partition into "# ..." metadata lines and CSV data lines.
    # csv.reader can't handle "#" comments natively, so we filter first.
    with open(path) as fh:
        all_lines = fh.read().splitlines()
    meta_lines = [ln for ln in all_lines if ln.startswith("#")]
    data_lines = [ln for ln in all_lines if not ln.startswith("#") and ln.strip() != ""]
    if not data_lines:
        raise ValueError(f"no data rows in {path}")
    meta = _parse_metadata(meta_lines)

    reader = csv.DictReader(data_lines)
    rows = list(reader)
    if not rows:
        raise ValueError(f"no data rows in {path}")

    t       = np.array([float(r["t_s"])           for r in rows])
    armed   = np.array([int(r["armed"])           for r in rows])
    yaw     = np.array([float(r["yaw_rate_dps"])  for r in rows])
    target  = np.array([float(r["target_dps"])    for r in rows])
    pwm     = np.array([_to_float(r["servo4_pwm"]) for r in rows])
    vbat    = np.array([_to_float(r["vbat_v"])    for r in rows])
    cur     = np.array([_to_float(r["current_a"]) for r in rows])
    power   = np.array([_to_float(r["power_w"])   for r in rows])
    # Optional columns -- only present in newer captures with Lua NAMED_VALUE_FLOAT.
    has_yi = "yaw_i_lua" in reader.fieldnames
    has_yo = "yaw_out_unclamp_lua" in reader.fieldnames
    yaw_i_arr = np.array([_to_float(r["yaw_i_lua"])         for r in rows]) if has_yi else None
    yaw_o_arr = np.array([_to_float(r["yaw_out_unclamp_lua"]) for r in rows]) if has_yo else None

    # Gains: prefer metadata header; fall back to per-row columns for backward compat.
    def _meta_float(key: str, fallback: float) -> float:
        v = meta.get(key)
        if v is None or v == "" or v == "None":
            return fallback
        try:
            return float(v)
        except ValueError:
            return fallback
    r0 = rows[0]
    kp_fb   = float(r0["kp"])         if "kp"         in r0 else 0.0
    ki_fb   = float(r0["ki"])         if "ki"         in r0 else 0.0
    kd_fb   = float(r0["kd"])         if "kd"         in r0 else 0.0
    imax_fb = float(r0["imax"])       if "imax"       in r0 else 0.0
    trim_fb = float(r0["h_yaw_trim"]) if "h_yaw_trim" in r0 else 0.0

    return Trace(
        t=t, armed=armed, yaw_rate=yaw, target_rate=target,
        pwm=pwm, vbat=vbat, current=cur, power=power,
        yaw_i=yaw_i_arr, yaw_out_unclamp=yaw_o_arr,
        kp   = _meta_float("ATC_RAT_YAW_P",    kp_fb),
        ki   = _meta_float("ATC_RAT_YAW_I",    ki_fb),
        kd   = _meta_float("ATC_RAT_YAW_D",    kd_fb),
        imax = _meta_float("ATC_RAT_YAW_IMAX", imax_fb),
        trim = _meta_float("H_YAW_TRIM",       trim_fb),
        meta = meta,
    )


# ---------------------------------------------------------------------------
# Per-section analyses
# ---------------------------------------------------------------------------

def _armed_slice(tr: Trace):
    """Return mask + slices for armed-only samples."""
    mask = tr.armed == 1
    return mask, tr.t[mask], tr.yaw_rate[mask], tr.pwm[mask]


def section_summary(tr: Trace) -> list[str]:
    out = ["=" * 60, "TUNING ANALYSIS", "=" * 60]
    if tr.meta:
        ts = tr.meta.get("run_start_local") or tr.meta.get("run_start_utc")
        if ts:
            out.append(f"  recorded      : {ts}")
    out.append(f"  samples       : {len(tr.t)}")
    duration = tr.t[-1] - tr.t[0]
    mask, t_a, _, _ = _armed_slice(tr)
    armed_dt = (t_a.max() - t_a.min()) if t_a.size >= 2 else 0.0
    out.append(f"  duration      : {duration:.1f} s  (armed: {armed_dt:.1f} s)")
    out.append(f"  gains         : P={tr.kp:.5f}  I={tr.ki:.5f}  D={tr.kd:.5f}")
    out.append(f"                  IMAX={tr.imax:.3f}   H_YAW_TRIM={tr.trim:.3f}")
    out.append(f"  design target : P={DESIGN_KP}   I={DESIGN_KI}   D={DESIGN_KD}")
    if tr.meta:
        smin = tr.meta.get("SERVO4_MIN", "?")
        smax = tr.meta.get("SERVO4_MAX", "?")
        sfn  = tr.meta.get("SERVO4_FUNCTION", "?")
        mode = tr.meta.get("SCR_USER6", "?")
        out.append(f"  servo4_min/max: {smin} / {smax} us   SERVO4_FUNCTION={sfn}   SCR_USER6={mode}")
    return out


def section_yaw_rate(tr: Trace) -> tuple[list[str], dict]:
    mask, _, yaw, _ = _armed_slice(tr)
    if yaw.size < 10:
        return ["", "YAW RATE", "  [SKIP] too few armed samples"], {}
    stats = {
        "mean":     float(np.nanmean(yaw)),
        "std":      float(np.nanstd(yaw)),
        "peak_abs": float(np.nanmax(np.abs(yaw))),
        "sat_frac": float((np.abs(yaw) > GYRO_SATURATION_DPS).mean()),
    }
    out = ["", "YAW RATE (armed)"]
    out.append(f"  mean          : {stats['mean']:+8.2f} deg/s")
    out.append(f"  std (1 sigma) : {stats['std']:8.2f} deg/s")
    out.append(f"  peak |rate|   : {stats['peak_abs']:8.1f} deg/s")
    if stats["sat_frac"] > 0.005:
        out.append(f"  [WARN] gyro saturation: {stats['sat_frac']*100:.1f}% "
                   f"of armed samples at |rate| > {GYRO_SATURATION_DPS:.0f} dps")
    return out, stats


def section_pwm(tr: Trace, servo_min: int, servo_max: int) -> tuple[list[str], dict]:
    mask, _, _, pwm = _armed_slice(tr)
    valid = ~np.isnan(pwm)
    pwm_v = pwm[valid]
    if pwm_v.size < 10:
        return ["", "SERVO4 PWM", "  [SKIP] too few armed PWM samples"], {}
    stats = {
        "mean":     float(pwm_v.mean()),
        "max_frac": float((pwm_v >= servo_max - 1).mean()),
        "min_frac": float((pwm_v <= servo_min + 1).mean()),
    }
    out = ["", "SERVO4 PWM (armed)"]
    out.append(f"  mean          : {stats['mean']:8.1f} us")
    out.append(f"  at SERVO4_MAX : {stats['max_frac']*100:5.1f}% of armed samples "
               f"(cap = {servo_max} us)")
    out.append(f"  at SERVO4_MIN : {stats['min_frac']*100:5.1f}% of armed samples "
               f"(cap = {servo_min} us)")
    return out, stats


def section_integrator(tr: Trace) -> tuple[list[str], dict]:
    """Diagnose the asymmetric integrator from Lua's YAW_I NAMED_VALUE_FLOAT."""
    if tr.yaw_i is None:
        return [], {}
    mask, _, _, _ = _armed_slice(tr)
    yi = tr.yaw_i[mask]
    yi = yi[~np.isnan(yi)]
    if yi.size < 10:
        return ["", "INTEGRATOR (YAW_I from rawes.lua)", "  [SKIP] no Lua YAW_I samples"], {}
    stats = {
        "mean":       float(yi.mean()),
        "peak":       float(yi.max()),
        "frac_zero":  float((yi < 1e-4).mean()),
        "frac_imax":  float((yi > 0.95 * tr.imax).mean()) if tr.imax > 0 else 0.0,
    }
    out = ["", "INTEGRATOR (YAW_I from rawes.lua)"]
    out.append(f"  mean          : {stats['mean']:.4f}   peak: {stats['peak']:.4f}   imax: {tr.imax:.3f}")
    out.append(f"  near zero (<1e-4)        : {stats['frac_zero']*100:5.1f}% of armed samples")
    out.append(f"  near imax (>{0.95*tr.imax:.3f}) : {stats['frac_imax']*100:5.1f}% of armed samples")
    if stats["frac_imax"] > 0.5:
        out.append("  [WARN] integrator pinned at IMAX >50% of run -- I-term is dominating.  "
                   "Reduce I or IMAX (or both).")
    if stats["frac_zero"] > 0.5 and stats["peak"] < 0.01:
        out.append("  [INFO] integrator basically never engaged -- the run had no CCW drift "
                   "to integrate.  Inject a deliberate CCW disturbance to exercise it.")
    return out, stats


def section_unclamp_pressure(tr: Trace) -> list[str]:
    """How often does the pre-clamp output exceed [0, 1]?  This is the
    PID's 'desired but blocked' command authority."""
    if tr.yaw_out_unclamp is None:
        return []
    mask, _, _, _ = _armed_slice(tr)
    yo = tr.yaw_out_unclamp[mask]
    yo = yo[~np.isnan(yo)]
    if yo.size < 10:
        return []
    frac_over = float((yo > 1.0).mean())
    frac_under = float((yo < 0.0).mean())
    out = ["", "PRE-CLAMP OUTPUT (YAW_OUT from rawes.lua)"]
    out.append(f"  mean (raw)    : {yo.mean():.4f}   peak: {yo.max():.4f}   trough: {yo.min():.4f}")
    out.append(f"  > 1.0 (would saturate high): {frac_over*100:5.1f}% of armed samples")
    out.append(f"  < 0.0 (would saturate low) : {frac_under*100:5.1f}% of armed samples")
    if frac_over > 0.10:
        out.append("  [INFO] PID is consistently asking for more output than 1.0.  "
                   "Either gains are too aggressive, or SERVO4_MAX is set as a tuning cap.")
    return out


def section_power(tr: Trace) -> list[str]:
    mask, _, _, _ = _armed_slice(tr)
    pw = tr.power[mask]
    pw = pw[~np.isnan(pw)]
    if pw.size < 5:
        return []
    out = ["", "POWER (whole-pack)"]
    out.append(f"  mean          : {pw.mean():8.2f} W   peak {pw.max():.2f} W")
    if pw.max() < 5.0:
        out.append("  [WARN] peak < 5 W -- battery monitor may not be on the motor circuit "
                   "(power column is electronics only, not motor)")
    return out


def section_sign_check(tr: Trace, servo_min: int, servo_max: int) -> tuple[list[str], dict | None]:
    """
    Correlate yaw_rate with PWM on non-saturated samples.

    Current rawes.lua convention:
        err = -yaw_rate           (rawes.lua:272)
        output >= 0 (clamped)
    So when the loop is healthy:
        hub drifts CCW (yaw_rate < 0) -> err > 0 -> PWM up   -> motor torque counters drift
        hub drifts CW  (yaw_rate > 0) -> err < 0 -> PWM=MIN  (no authority on this side)
    => Expected correlation between yaw_rate and PWM is NEGATIVE.
       Positive correlation = Lua sign or motor wiring is reversed.

    Note: a negative correlation alone is not sufficient to declare success.
    A runaway loop with the WRONG motor direction also produces negative correlation
    (PWM ramps to MAX while rate diverges to gyro saturation).  Check the RUNAWAY
    section as well -- bounded rate + negative correlation = working.
    """
    mask, _, yaw, pwm = _armed_slice(tr)
    valid = ~np.isnan(pwm)
    yaw_v = yaw[valid]
    pwm_v = pwm[valid]
    unsat = (pwm_v > servo_min + 5) & (pwm_v < servo_max - 5)
    out = ["", "SIGN CHECK"]
    if unsat.sum() < 30:
        out.append("  [SKIP] insufficient non-saturated samples "
                   f"(only {int(unsat.sum())}); PWM railed too much for a clean correlation")
        return out, None
    # also require some real rate activity, not just noise
    yaw_unsat = yaw_v[unsat]
    if np.nanstd(yaw_unsat) < DISTURBANCE_DPS / 2:
        out.append("  [SKIP] yaw rate too quiet on non-saturated samples "
                   f"(sigma={np.nanstd(yaw_unsat):.2f} dps); run with a deliberate disturbance")
        return out, None
    r = float(np.corrcoef(yaw_unsat, pwm_v[unsat])[0, 1])
    out.append(f"  corr(yaw_rate, pwm) on non-saturated samples: r = {r:+.3f}  "
               f"(n = {int(unsat.sum())})")
    if r <= -0.3:
        out.append("  [OK] NEGATIVE correlation -- PWM goes up when hub drifts CCW "
                   "(matches rawes.lua's err = -yaw_rate convention; loop is on the right side).")
        out.append("         Verify with RUNAWAY CHECK: bounded rate => actually working; "
                   "rate diverging to gyro clip => motor torque direction is still wrong.")
    elif r >= 0.3:
        out.append("  [FAIL] POSITIVE correlation -- PWM goes up when hub drifts CW.")
        out.append("         Lua sign or motor wiring is inverted vs. the current convention.  "
                   "Either reverse motor direction, OR flip err = -yaw_rate -> err = +yaw_rate "
                   "in rawes.lua (and the asymmetric integrator branch).")
    else:
        out.append(f"  [INCONCLUSIVE] |r| < 0.3 -- not enough signal to assess sign.  "
                   f"Run with a deliberate, larger disturbance.")
    return out, {"r": r, "n": int(unsat.sum())}


def section_runaway(tr: Trace, servo_max: int) -> list[str]:
    """
    Detect the canonical runaway: PWM saturates AND |rate| keeps growing in the
    same direction for >= 0.2 s.  Classic positive-feedback signature.
    """
    mask, t_a, yaw, pwm = _armed_slice(tr)
    valid = ~np.isnan(pwm)
    t_v = t_a[valid]; yaw_v = yaw[valid]; pwm_v = pwm[valid]
    sat = pwm_v >= servo_max - 1
    if sat.sum() < 5:
        return []
    # find first contiguous saturation run >= 0.2 s
    out = ["", "RUNAWAY CHECK"]
    edges = np.diff(sat.astype(int))
    starts = list(np.where(edges == 1)[0] + 1)
    ends = list(np.where(edges == -1)[0] + 1)
    if sat[0]:
        starts.insert(0, 0)
    if sat[-1]:
        ends.append(len(sat))
    runs = list(zip(starts, ends))
    if not runs:
        return []
    s, e = max(runs, key=lambda r: r[1] - r[0])
    dur = t_v[e - 1] - t_v[s]
    if dur < 0.2:
        return []
    rate_at_sat_start = yaw_v[s]
    rate_at_sat_end   = yaw_v[e - 1]
    drift = rate_at_sat_end - rate_at_sat_start
    out.append(f"  longest PWM-sat run : {dur:.2f} s  ({e - s} samples)")
    out.append(f"  rate at sat start   : {rate_at_sat_start:+.1f} dps")
    out.append(f"  rate at sat end     : {rate_at_sat_end:+.1f} dps  "
               f"(drift {drift:+.1f} dps over sat)")
    same_sign_growth = (
        (rate_at_sat_start < 0 and rate_at_sat_end < rate_at_sat_start) or
        (rate_at_sat_start > 0 and rate_at_sat_end > rate_at_sat_start)
    )
    converging = abs(rate_at_sat_end) < abs(rate_at_sat_start) - 50
    if same_sign_growth and abs(drift) > 50:
        out.append("  [FAIL] rate grew in the SAME direction while PWM was railed -- "
                   "actuator is driving the hub the wrong way (positive feedback).")
    elif converging:
        out.append("  [OK] |rate| shrank by "
                   f"{abs(rate_at_sat_start) - abs(rate_at_sat_end):.0f} dps while PWM was "
                   "railed -- actuator authority is real and on the correct side of the loop.")
    return out


# ---------------------------------------------------------------------------
# Recommendations
# ---------------------------------------------------------------------------

def recommendations(tr: Trace, rate_stats: dict, pwm_stats: dict,
                    sign: dict | None, servo_min: int, servo_max: int) -> list[str]:
    recs: list[str] = []

    # ── Sign ────────────────────────────────────────────────────────────────
    # rawes.lua uses err = -yaw_rate => healthy loop has NEGATIVE correlation.
    # Positive correlation = sign / wiring inverted.
    if sign is not None and sign["r"] >= 0.3:
        recs.append("Loop sign is inverted (positive corr) -- fix before touching gains.  "
                    "See SIGN CHECK above.")

    # ── Gyro saturation ─────────────────────────────────────────────────────
    if rate_stats.get("sat_frac", 0) > 0.01:
        target_max = max(servo_min + 200, int(servo_min + 0.4 * (servo_max - servo_min)))
        recs.append(f"Gyro saturated ({rate_stats['sat_frac']*100:.0f}%): "
                    f"set SERVO4_MAX = {target_max} so the motor cannot drive the hub past "
                    f"the IMU range.  Raise it again as you gain confidence in the gains.")

    # ── PWM saturation ──────────────────────────────────────────────────────
    if pwm_stats.get("max_frac", 0) > PWM_SAT_FRAC_HOT:
        new_kp = tr.kp / 2.0
        recs.append(f"PWM at SERVO4_MAX {pwm_stats['max_frac']*100:.0f}% of armed time: "
                    f"gains too aggressive.  Halve P: {tr.kp:.5f} -> {new_kp:.5f}.  "
                    f"Disable I (set ATC_RAT_YAW_I=0) while tuning P alone.")

    # ── Persistent rate bias (gains too low) ────────────────────────────────
    if (abs(rate_stats.get("mean", 0)) > RATE_BIAS_DPS
            and pwm_stats.get("max_frac", 0) < 0.01
            and pwm_stats.get("min_frac", 0) < 0.01):
        new_kp = tr.kp * 2.0
        recs.append(f"Persistent rate bias {rate_stats['mean']:+.1f} dps with PWM never "
                    f"saturating: gains too low.  Double P: {tr.kp:.5f} -> {new_kp:.5f}.")

    # ── Gains vs. design ────────────────────────────────────────────────────
    if tr.kp >= 4 * DESIGN_KP:
        recs.append(f"P={tr.kp:.4f} is {tr.kp/DESIGN_KP:.0f}x the design value ({DESIGN_KP}).  "
                    f"Start at design and raise only if response is sluggish.")
    if tr.ki >= 4 * DESIGN_KI:
        recs.append(f"I={tr.ki:.4f} is {tr.ki/DESIGN_KI:.0f}x the design value ({DESIGN_KI}).  "
                    f"With a one-sided actuator + asymmetric integrator, I=0.1 will pin the "
                    f"output at IMAX within a few hundred ms.  Disable while tuning P; "
                    f"add back at ~P/10 once steady-state error is measured.")
    if tr.kd > 0.0:
        recs.append(f"D={tr.kd:.6f} > 0: D-term is rarely useful on this airframe (gyro noise "
                    f"amplifies it).  Recommend D=0 until you have evidence it helps.")

    # ── IMAX vs. saturation ─────────────────────────────────────────────────
    if rate_stats.get("peak_abs", 0) > GYRO_SATURATION_DPS and tr.imax > 0.3:
        recs.append(f"Gyro hit saturation AND IMAX={tr.imax:.2f} is high: I-term may have "
                    f"wound past recovery during the saturation window.  Consider IMAX -> "
                    f"{tr.imax/2:.2f} so saturation events are recoverable.")

    return recs


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("csv", nargs="?", default=None,
                    help="path to tuning_*.csv (default: newest tuning_*.csv "
                         "in CWD, falling back to tuning.csv)")
    ap.add_argument("--servo-min", type=int, default=None,
                    help="override SERVO4_MIN PWM (us); default: read from metadata, else 800")
    ap.add_argument("--servo-max", type=int, default=None,
                    help="override SERVO4_MAX PWM (us); default: read from metadata, else 2000")
    args = ap.parse_args()

    # Find the CSV: explicit path > newest tuning_*.csv > tuning.csv
    if args.csv:
        path = Path(args.csv)
    else:
        candidates = sorted(Path(".").glob("tuning_*.csv"))
        path = candidates[-1] if candidates else Path("tuning.csv")
        print(f"  (auto-selected {path})")
    if not path.exists():
        print(f"  [ERROR] file not found: {path.resolve()}")
        sys.exit(1)

    tr = load_csv(path)

    # Resolve servo limits: explicit flag > metadata > 800/2000.
    def _meta_int(key: str, fallback: int) -> int:
        v = tr.meta.get(key)
        if v is None or v == "" or v == "None":
            return fallback
        try:
            return int(float(v))
        except ValueError:
            return fallback
    servo_min = args.servo_min if args.servo_min is not None else _meta_int("SERVO4_MIN", 800)
    servo_max = args.servo_max if args.servo_max is not None else _meta_int("SERVO4_MAX", 2000)

    lines: list[str] = []
    lines += section_summary(tr)
    rate_lines, rate_stats = section_yaw_rate(tr)
    lines += rate_lines
    pwm_lines, pwm_stats = section_pwm(tr, servo_min, servo_max)
    lines += pwm_lines
    int_lines, _int_stats = section_integrator(tr)
    lines += int_lines
    lines += section_unclamp_pressure(tr)
    lines += section_power(tr)
    sign_lines, sign = section_sign_check(tr, servo_min, servo_max)
    lines += sign_lines
    lines += section_runaway(tr, servo_max)

    recs = recommendations(tr, rate_stats, pwm_stats, sign,
                           servo_min, servo_max)
    lines.append("")
    lines.append("RECOMMENDATIONS")
    if recs:
        for i, r in enumerate(recs, 1):
            lines.append(f"  {i}. {r}")
    else:
        lines.append("  No issues flagged.  Trace looks clean.")

    print("\n".join(lines))


if __name__ == "__main__":
    main()
