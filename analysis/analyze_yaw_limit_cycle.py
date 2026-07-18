"""analyze_yaw_limit_cycle.py -- diagnose the Lua yaw-PID limit cycle from telemetry CSV.

Reads a torque telemetry CSV (written by tests/sitl/torque/torque_test_utils.py,
schema = simulation/telemetry_columns.py) and decides WHICH mechanism is
sustaining the yaw limit cycle so tuning effort is aimed at the real cause.

Three candidate mechanisms are tested from the data:

  (A) UNIDIRECTIONAL-MOTOR SATURATION.
      The GB4008 counter-torque motor is one-directional (throttle >= 0, it can
      only push the body CW to oppose the rotor's CCW drag).  When the body
      overshoots into the CW / motor-driven direction the controller WANTS
      negative throttle to brake, but the command clamps at idle (SERVO4_MIN),
      so the motor shuts off and the yaw rate merely COASTS back on rotor drag.
      Signature: SERVO4 pinned at the idle floor for a large duty, the
      reconstructed pre-clamp PID demand goes negative while pinned, and the
      "coast" half-cycle has a much gentler |d(psi_dot)/dt| than the "drive"
      half-cycle (asymmetry).

  (B) LINEAR UNDERDAMPING.
      P (rate damping) too low vs I (heading spring).  Signature: the SERVO4
      command oscillates in mid-range WITHOUT pinning at either rail, roughly
      symmetric drive/coast slopes.  Fix = raise KP or lower KI.

  (C) INTEGRAL WINDUP / OVER-AUTHORITY.
      Signature: SERVO4 (or YFF_T) pinned at the HIGH rail (YFF_MAX) a large
      duty, integral riding the clamp.  Fix = lower KI / IMAX.

Usage:
    python analysis/analyze_yaw_limit_cycle.py [csv] [--start SEC]
        [--idle-tol-us US] [--ceil-frac F]

    csv defaults to simulation/logs/test_yaw_regulation_sitl/telemetry.csv
    --start   analysis window start [sim s] (default 25 = end of spin-up)

All output is 7-bit ASCII (Windows cp1252 safe).
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

# SERVO4 (GB4008) PWM mapping and Lua trim clamp -- mirror rawes.lua / the parm chain.
SERVO4_MIN_US = 800.0
SERVO4_MAX_US = 2000.0
YFF_MAX = 0.7            # Lua H_YAW_TRIM clamp ceiling (rawes.lua YFF_MAX)

_DEFAULT_CSV = "simulation/logs/test_yaw_regulation_sitl/telemetry.csv"


def _f(row: dict, key: str) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return float("nan")


def load(path: str, start_s: float) -> dict:
    """Load the telemetry CSV columns needed for the diagnosis, windowed to t >= start_s."""
    t, omz, s4 = [], [], []
    trim, yi, gz, kp, ki, kd = [], [], [], [], [], []
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            ts = _f(row, "t_sim")
            if not math.isfinite(ts) or ts < start_s:
                continue
            t.append(ts)
            omz.append(_f(row, "omega_z"))
            s4.append(_f(row, "servo_mot_us"))
            trim.append(_f(row, "mav_nvf_yff_trim"))
            yi.append(_f(row, "mav_nvf_yff_u"))
            gz.append(_f(row, "mav_nvf_yff_gz"))
            kp.append(_f(row, "mav_nvf_yff_kp"))
            ki.append(_f(row, "mav_nvf_yff_ki"))
            kd.append(_f(row, "mav_nvf_yff_kd"))
    return {
        "t": np.asarray(t), "omz": np.asarray(omz), "s4": np.asarray(s4),
        "trim": np.asarray(trim), "yi": np.asarray(yi), "gz": np.asarray(gz),
        "kp": np.asarray(kp), "ki": np.asarray(ki), "kd": np.asarray(kd),
    }


def _zero_cross_period(t: np.ndarray, x: np.ndarray) -> float:
    """Mean full period [s] from sign changes of x about its mean (nan if <2 crossings)."""
    xc = x - np.nanmean(x)
    idx = np.where(np.sign(xc[:-1]) * np.sign(xc[1:]) < 0)[0]
    if len(idx) < 2:
        return float("nan")
    # Linear-interpolate crossing times for a better estimate.
    tc = []
    for i in idx:
        x0, x1 = xc[i], xc[i + 1]
        frac = x0 / (x0 - x1) if (x0 - x1) != 0 else 0.0
        tc.append(t[i] + frac * (t[i + 1] - t[i]))
    tc = np.asarray(tc)
    return float(2.0 * np.mean(np.diff(tc)))


def _fft_period(t: np.ndarray, x: np.ndarray) -> float:
    """Dominant period [s] via FFT on a uniformly resampled signal (nan if degenerate)."""
    if len(t) < 8:
        return float("nan")
    dt = np.median(np.diff(t))
    if not math.isfinite(dt) or dt <= 0:
        return float("nan")
    tu = np.arange(t[0], t[-1], dt)
    xu = np.interp(tu, t, x)
    xu = xu - np.mean(xu)
    if len(xu) < 8 or np.allclose(xu, 0):
        return float("nan")
    freqs = np.fft.rfftfreq(len(xu), d=dt)
    mag = np.abs(np.fft.rfft(xu))
    mag[0] = 0.0
    k = int(np.argmax(mag))
    if freqs[k] <= 0:
        return float("nan")
    return float(1.0 / freqs[k])


def analyze(d: dict, idle_tol_us: float, ceil_frac: float) -> None:
    t, omz, s4 = d["t"], d["omz"], d["s4"]
    if len(t) < 10:
        print("[FAIL] not enough samples in window (need >= 10, got %d)" % len(t))
        return

    omz_deg = np.degrees(omz)
    dur = t[-1] - t[0]
    n = len(t)

    # --- Limit-cycle characterisation ------------------------------------
    amp_pk = float(np.nanmax(np.abs(omz_deg)))
    rms = float(np.sqrt(np.nanmean(omz_deg ** 2)))
    p_zc = _zero_cross_period(t, omz_deg)
    p_fft = _fft_period(t, omz_deg)
    # Sustained vs decaying: RMS in first vs last third.
    third = n // 3
    rms_first = float(np.sqrt(np.nanmean(omz_deg[:third] ** 2)))
    rms_last = float(np.sqrt(np.nanmean(omz_deg[-third:] ** 2)))
    trend = "SUSTAINED" if rms_last > 0.6 * rms_first else "DECAYING"

    print("=" * 72)
    print("YAW LIMIT-CYCLE DIAGNOSIS   (window t=%.1f..%.1f s, %d samples)" % (t[0], t[-1], n))
    print("=" * 72)
    kp = float(np.nanmedian(d["kp"])); ki = float(np.nanmedian(d["ki"])); kd = float(np.nanmedian(d["kd"]))
    print("Gains (from telemetry): KP=%.4g  KI=%.4g  KD=%.4g" % (kp, ki, kd))
    print("")
    print("[1] Limit cycle")
    print("    peak |psi_dot|   = %7.1f deg/s   RMS = %6.1f deg/s" % (amp_pk, rms))
    print("    period (zero-x)  = %7.2f s        (FFT) = %6.2f s" % (p_zc, p_fft))
    print("    amplitude trend  = %s (RMS first third=%.1f, last third=%.1f)"
          % (trend, rms_first, rms_last))

    # --- Actuator saturation ---------------------------------------------
    have_s4 = np.isfinite(s4).any()
    idle_duty = ceil_duty = float("nan")
    thr = np.full_like(s4, np.nan)
    if have_s4:
        thr = (s4 - SERVO4_MIN_US) / (SERVO4_MAX_US - SERVO4_MIN_US)
        idle_mask = s4 <= (SERVO4_MIN_US + idle_tol_us)
        ceil_us = SERVO4_MIN_US + ceil_frac * (SERVO4_MAX_US - SERVO4_MIN_US)
        ceil_mask = s4 >= ceil_us
        idle_duty = float(np.mean(idle_mask))
        ceil_duty = float(np.mean(ceil_mask))
    print("")
    print("[2] Actuator (SERVO4) saturation")
    if have_s4:
        print("    throttle mean=%.3f  min=%.3f  max=%.3f" %
              (np.nanmean(thr), np.nanmin(thr), np.nanmax(thr)))
        print("    idle-floor duty  (<= %.0f us) = %5.1f %%" % (SERVO4_MIN_US + idle_tol_us, 100 * idle_duty))
        print("    high-rail duty   (>= %.0f%% ) = %5.1f %%" % (100 * ceil_frac, 100 * ceil_duty))
    else:
        print("    SERVO4 telemetry absent -- cannot assess actuator saturation")

    # --- Controller WANTS reverse (pre-clamp demand) ---------------------
    # Reconstruct the pre-clamp PID output the Lua computed:
    #   err = -gz ; out_preclamp = yaw_i + kp*err (+ kd*d_lp; kd=0 here).
    # yff_i and gz are captured; when out_preclamp < 0 the motor cannot comply
    # (one-directional) and the command clamps at idle.
    gz = d["gz"]; yi = d["yi"]
    have_demand = np.isfinite(gz).any() and np.isfinite(yi).any()
    want_rev_duty = float("nan")
    pinned_when_want_rev = float("nan")
    if have_demand:
        pre = yi + d["kp"] * (-gz)          # kd term omitted (kd=0 in this regime)
        want_rev = pre < 0.0
        want_rev_duty = float(np.nanmean(want_rev))
        if have_s4 and np.any(want_rev):
            idle_mask = s4 <= (SERVO4_MIN_US + idle_tol_us)
            pinned_when_want_rev = float(np.mean(idle_mask[want_rev]))
    print("")
    print("[3] Controller demand vs authority")
    if have_demand:
        print("    'wants negative throttle' duty (pre-clamp < 0) = %5.1f %%" % (100 * want_rev_duty))
        if math.isfinite(pinned_when_want_rev):
            print("    of that time, SERVO4 pinned at idle           = %5.1f %%" % (100 * pinned_when_want_rev))
    else:
        print("    YFF_I / YFF_GZ telemetry absent -- cannot reconstruct demand")

    # --- Coast vs drive asymmetry ----------------------------------------
    # domega/dt in motor-ON (driving) vs motor-OFF (coasting on drag) samples.
    drive_slope = coast_slope = float("nan")
    if have_s4 and n > 5:
        dodt = np.gradient(omz_deg, t)                 # deg/s^2
        idle_mask = s4 <= (SERVO4_MIN_US + idle_tol_us)
        on_mask = ~idle_mask
        if np.any(on_mask):
            drive_slope = float(np.nanmean(np.abs(dodt[on_mask])))
        if np.any(idle_mask):
            coast_slope = float(np.nanmean(np.abs(dodt[idle_mask])))
    print("")
    print("[4] Coast/drive asymmetry (|d psi_dot/dt|)")
    if math.isfinite(drive_slope) and math.isfinite(coast_slope):
        ratio = drive_slope / coast_slope if coast_slope > 1e-9 else float("inf")
        print("    motor-ON  (drive) = %8.1f deg/s^2" % drive_slope)
        print("    motor-OFF (coast) = %8.1f deg/s^2   drive/coast = %.2f" % (coast_slope, ratio))
    else:
        print("    insufficient data to split drive vs coast")

    # --- Verdict ----------------------------------------------------------
    print("")
    print("[VERDICT]")
    verdict = []
    # (A) unidirectional saturation
    a_hit = (
        math.isfinite(idle_duty) and idle_duty > 0.15
        and math.isfinite(want_rev_duty) and want_rev_duty > 0.15
        and math.isfinite(pinned_when_want_rev) and pinned_when_want_rev > 0.6
    )
    # (C) integral windup / over-authority
    c_hit = math.isfinite(ceil_duty) and ceil_duty > 0.15
    # (B) linear underdamping (no rail pinning to speak of)
    b_hit = (
        math.isfinite(idle_duty) and idle_duty < 0.10
        and (not math.isfinite(ceil_duty) or ceil_duty < 0.10)
    )
    if a_hit:
        verdict.append(
            "(A) UNIDIRECTIONAL-MOTOR SATURATION CONFIRMED: the motor is pinned at\n"
            "    idle %.0f%% of the run, and %.0f%% of the time the PID demands negative\n"
            "    throttle it is stuck at idle -- the rate then coasts (drive/coast slope\n"
            "    ratio above).  P/I tuning alone cannot fix this; needs an equilibrium\n"
            "    feed-forward or an idle bias that keeps the motor in its linear band."
            % (100 * idle_duty, 100 * pinned_when_want_rev)
        )
    if c_hit:
        verdict.append(
            "(C) INTEGRAL WINDUP / OVER-AUTHORITY: SERVO4 rides the HIGH rail %.0f%% of\n"
            "    the run -- lower KI / IMAX." % (100 * ceil_duty)
        )
    if b_hit and not a_hit:
        verdict.append(
            "(B) LINEAR UNDERDAMPING: the actuator never pins at a rail, so the cycle is\n"
            "    an underdamped P/I response -- raise KP (rate damping) or lower KI."
        )
    if not verdict:
        verdict.append(
            "INCONCLUSIVE: no single mechanism dominates by the thresholds.  Inspect the\n"
            "    per-section numbers above (idle duty, want-reverse duty, slope ratio)."
        )
    for v in verdict:
        print("  " + v)
    print("=" * 72)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", default=_DEFAULT_CSV,
                    help="telemetry CSV (default: %(default)s)")
    ap.add_argument("--start", type=float, default=25.0,
                    help="analysis window start [sim s] (default: %(default)s)")
    ap.add_argument("--idle-tol-us", type=float, default=20.0,
                    help="SERVO4 us above SERVO4_MIN still counted as 'idle' (default: %(default)s)")
    ap.add_argument("--ceil-frac", type=float, default=0.60,
                    help="throttle fraction counted as 'high rail' (default: %(default)s)")
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print("[FAIL] CSV not found: %s" % path)
        raise SystemExit(2)

    d = load(str(path), args.start)
    analyze(d, args.idle_tol_us, args.ceil_frac)


if __name__ == "__main__":
    main()
