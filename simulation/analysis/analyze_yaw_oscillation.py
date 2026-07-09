"""analyze_yaw_oscillation.py -- yaw limit-cycle / Lua-trim interaction analyzer.

Reads a calibrate ``.mavlink.jsonl`` traffic log (written by RawesGCS /
MavlinkLogWriter) and characterises the yaw oscillation and how it interacts
with the Lua yaw-trim feedforward.

Signals used (each on the FC boot-time clock, so they align):
  * ATTITUDE.yawspeed      yaw RATE [rad/s]  (== the Lua's regulated gyro:z())  24 Hz
  * ATTITUDE.yaw           heading [rad]                                        24 Hz
  * SERVO_OUTPUT_RAW.servo4_raw   yaw motor PWM [us]  (time_usec clock)         24 Hz
  * NAMED_VALUE_FLOAT YFF_T   Lua trim  = H_YAW_TRIM  [0..1]                    ~2 Hz
  * NAMED_VALUE_FLOAT YFF_U   applied SERVO4 throttle u [0..1]                  ~2 Hz
  * NAMED_VALUE_FLOAT YFF_GZ  Lua psi_dot (gyro:z()) [rad/s]                    ~2 Hz
  * NAMED_VALUE_FLOAT YFF_A   calibrated slope [rad/s per u]                    ~2 Hz
  * NAMED_VALUE_FLOAT YFF_KD  yaw-D gain (SCR_USER1)                            ~2 Hz

Reports oscillation frequency / amplitude (FFT + zero-crossings), the
actuator<->response phase, the ESC deadband duty, and the trim behaviour.

Usage:
    python simulation/analysis/analyze_yaw_oscillation.py <log.mavlink.jsonl> [--plot]
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

# Bench-calibrated SERVO4 deadband (see repo memory yaw-regulation-lua-ff-clamp.md).
YAW_DEADBAND_US = 1108.0
SERVO4_MIN_US = 800.0
SERVO4_MAX_US = 2000.0


# ---------------------------------------------------------------------------
# Load
# ---------------------------------------------------------------------------

def _boot_ms(d: dict) -> "float | None":
    """FC boot-time [ms] for a message, using the most reliable native field."""
    if d.get("mavpackettype") == "SERVO_OUTPUT_RAW" and "time_usec" in d:
        return float(d["time_usec"]) / 1000.0
    tb = d.get("time_boot_ms")
    return float(tb) if tb is not None else None


def load(path: str) -> dict:
    att_t, att_yaw, att_rate = [], [], []
    srv_t, srv_pwm = [], []
    nvf: dict[str, list] = {k: [] for k in
                            ("YFF_T", "YFF_U", "YFF_GZ", "YFF_A", "YFF_KD")}
    nvf_t: dict[str, list] = {k: [] for k in nvf}

    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except Exception:
                continue
            if d.get("_dir") != "rx":
                continue
            mt = d.get("mavpackettype")
            t = _boot_ms(d)
            if t is None:
                continue
            if mt == "ATTITUDE":
                att_t.append(t)
                att_yaw.append(float(d.get("yaw", math.nan)))
                att_rate.append(float(d.get("yawspeed", math.nan)))
            elif mt == "SERVO_OUTPUT_RAW":
                srv_t.append(t)
                srv_pwm.append(float(d.get("servo4_raw", math.nan)))
            elif mt == "NAMED_VALUE_FLOAT":
                nm = d.get("name")
                if nm in nvf:
                    nvf[nm].append(float(d.get("value", math.nan)))
                    nvf_t[nm].append(t)

    return {
        "att_t": np.array(att_t), "att_yaw": np.array(att_yaw),
        "att_rate": np.array(att_rate),
        "srv_t": np.array(srv_t), "srv_pwm": np.array(srv_pwm),
        "nvf": {k: np.array(v) for k, v in nvf.items()},
        "nvf_t": {k: np.array(v) for k, v in nvf_t.items()},
    }


# ---------------------------------------------------------------------------
# Signal analysis
# ---------------------------------------------------------------------------

def _resample(t_ms: np.ndarray, y: np.ndarray):
    """Uniformly resample (t in ms, y) onto the median dt grid. Returns (fs, yr)."""
    if len(t_ms) < 4:
        return None, None
    t = (t_ms - t_ms[0]) / 1000.0            # seconds
    dt = np.median(np.diff(t))
    if dt <= 0:
        return None, None
    grid = np.arange(t[0], t[-1], dt)
    yr = np.interp(grid, t, y)
    return 1.0 / dt, yr


def dominant_freq(t_ms: np.ndarray, y: np.ndarray, fmin=0.2):
    """FFT dominant frequency [Hz] of the AC part, ignoring freqs < fmin."""
    fs, yr = _resample(t_ms, y)
    if fs is None or len(yr) < 8:
        return None, None
    yr = yr - np.mean(yr)
    win = np.hanning(len(yr))
    spec = np.abs(np.fft.rfft(yr * win))
    freqs = np.fft.rfftfreq(len(yr), 1.0 / fs)
    mask = freqs >= fmin
    if not np.any(mask):
        return None, fs
    k = np.argmax(spec[mask])
    return float(freqs[mask][k]), fs


def zero_cross_freq(t_ms: np.ndarray, y: np.ndarray):
    """Oscillation frequency from mean-crossings [Hz]."""
    if len(y) < 4:
        return None
    dur = (t_ms[-1] - t_ms[0]) / 1000.0
    if dur <= 0:
        return None
    ac = y - np.mean(y)
    crossings = np.sum(np.diff(np.signbit(ac)) != 0)
    return crossings / (2.0 * dur)


def phase_lag_ms(t_a, a, t_b, b, period_s=None):
    """Cross-correlation lag of b relative to a [ms] on a common resampled grid.
    Positive => b lags a.  If *period_s* is given, the search is constrained to
    +/- period/2 so a periodic signal does not alias onto a distant cycle."""
    fs_a, ar = _resample(t_a, a)
    fs_b, br = _resample(t_b, b)
    if fs_a is None or fs_b is None:
        return None
    n = min(len(ar), len(br))
    ar = ar[:n] - np.mean(ar[:n])
    br = br[:n] - np.mean(br[:n])
    if np.std(ar) == 0 or np.std(br) == 0:
        return None
    corr = np.correlate(br, ar, mode="full")
    lags = np.arange(-(n - 1), n)          # samples
    if period_s is not None and period_s > 0:
        max_lag = 0.5 * period_s * fs_a
        keep = np.abs(lags) <= max_lag
        corr, lags = corr[keep], lags[keep]
        if len(corr) == 0:
            return None
    lag = lags[int(np.argmax(corr))]
    return lag * (1000.0 / fs_a)


def _fmt(x, nd=3):
    return "n/a" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{nd}f}"


def _common_grid(t_a_ms, a, t_b_ms, b):
    """Resample a(t_a) and b(t_b) onto one uniform grid over their overlap.
    Returns (dt, grid_s, a_on_grid, b_on_grid) or (None,...)."""
    if len(t_a_ms) < 4 or len(t_b_ms) < 4:
        return None, None, None, None
    ta = (t_a_ms - t_a_ms[0]) / 1000.0
    tb = (t_b_ms - t_b_ms[0]) / 1000.0
    # shared absolute time base (both already in FC boot ms; realign to min)
    t0 = min(t_a_ms[0], t_b_ms[0])
    ta = (t_a_ms - t0) / 1000.0
    tb = (t_b_ms - t0) / 1000.0
    dt = np.median(np.diff(ta))
    if dt <= 0:
        return None, None, None, None
    lo = max(ta[0], tb[0])
    hi = min(ta[-1], tb[-1])
    if hi - lo < 1.0:
        return None, None, None, None
    grid = np.arange(lo, hi, dt)
    return dt, grid, np.interp(grid, ta, a), np.interp(grid, tb, b)


def correction_analysis(t_srv_ms, pwm, t_att_ms, rate):
    """Decompose the SERVO4 AC command (the correction) against the yaw rate.

    With a CONSTANT rotor RPM the reaction torque is constant, so the
    equilibrium SERVO4 throttle is a flat line -- every bit of AC on SERVO4 is
    the controller's correction.  We regress that correction against the yaw
    rate and its integral and derivative to identify the effective control law
    (P: servo ~ -rate, I: servo ~ -integral(rate), D: servo ~ -d/dt(rate)), and
    report the phase, which shows whether the correction damps or drives the
    oscillation.
    """
    u = (pwm - SERVO4_MIN_US) / (SERVO4_MAX_US - SERVO4_MIN_US)
    dt, grid, u_g, r_g = _common_grid(t_srv_ms, u, t_att_ms, rate)
    if grid is None:
        print("  (not enough overlapping samples for correction analysis)")
        return

    u_ac = u_g - np.mean(u_g)          # servo correction (throttle units)
    psi = r_g - np.mean(r_g)           # yaw rate (rad/s), AC
    dpsi = np.gradient(psi, dt)        # yaw acceleration
    ipsi = np.cumsum(psi) * dt
    ipsi = ipsi - np.mean(ipsi)        # integral of yaw rate (heading-ish), AC

    # Regress u_ac on [rate, d/dt rate, integral rate].  Signs reveal the law.
    X = np.column_stack([psi, dpsi, ipsi])
    coef, *_ = np.linalg.lstsq(X, u_ac, rcond=None)
    pred = X @ coef
    ss_res = np.sum((u_ac - pred) ** 2)
    ss_tot = np.sum((u_ac - np.mean(u_ac)) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    # Relative influence of each term = |coef|*std(term)/std(u_ac).
    infl = [abs(coef[i]) * np.std(X[:, i]) / (np.std(u_ac) + 1e-12) for i in range(3)]
    labels = ["P  (u ~ rate)      ", "D  (u ~ d/dt rate) ", "I  (u ~ int rate)  "]

    print("\n--- Correction law (SERVO4 AC vs yaw rate) ---")
    print(f"  (constant rotor RPM => equilibrium SERVO4 is constant; all AC is correction)")
    print(f"  servo AC rms = {_fmt(np.std(u_ac)*1000,1)} m-throttle "
          f"({_fmt(np.std(u_ac)*(SERVO4_MAX_US-SERVO4_MIN_US),0)} us)")
    print(f"  fit u_ac = cP*rate + cD*(d rate) + cI*(int rate)   R^2 = {_fmt(r2,3)}")
    for lab, c, inf in zip(labels, coef, infl):
        print(f"    {lab} coef={_fmt(c,4)}   relative influence={_fmt(inf,2)}")

    # Correlation of servo with rate, and the "power" it does on the oscillation.
    corr = float(np.corrcoef(u_ac, psi)[0, 1]) if np.std(psi) > 0 else float("nan")
    # Motor counter-torque ~ +u; body yaw power ~ torque*rate.  AC average of
    # u_ac*psi > 0 => throttle rises when rate rises (in phase) => the motor is
    # NOT opposing the rate at zero lag.  The sign of the D-term coef tells the
    # damping: for damping we want u to oppose acceleration (cD same sign as ...).
    power = float(np.mean(u_ac * psi))
    print(f"  corr(servo AC, yaw rate) = {_fmt(corr,3)}   mean(servo_AC * rate) = {_fmt(power,4)}")

    dom = int(np.argmax(infl))
    dom_name = ["proportional (P) on yaw rate",
                "derivative (D) on yaw rate",
                "integral (I) on yaw rate"][dom]
    print(f"  -> dominant term: {dom_name}")
    if dom == 2:
        print("     An integral law through the laggy one-directional deadband motor")
        print("     limit-cycles: the correction lags the rate ~90 deg + motor lag,")
        print("     so at the cycle frequency it feeds the oscillation instead of")
        print("     damping it.  This matches the Lua trim degenerating to an")
        print("     integrator when the AP P is zero.")


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def analyze(path: str, do_plot: bool = False) -> None:
    D = load(path)
    att_t, yaw, rate = D["att_t"], D["att_yaw"], D["att_rate"]
    srv_t, pwm = D["srv_t"], D["srv_pwm"]
    nvf, nvf_t = D["nvf"], D["nvf_t"]

    print(f"=== Yaw oscillation analysis: {Path(path).name} ===")
    if len(att_t) < 4 or len(srv_t) < 4:
        print("  Not enough ATTITUDE/SERVO samples to analyze.")
        return

    t0 = min(att_t[0], srv_t[0])
    dur = (max(att_t[-1], srv_t[-1]) - t0) / 1000.0
    print(f"  duration = {dur:.1f} s   ATTITUDE N={len(att_t)} "
          f"({len(att_t)/dur:.1f} Hz)   SERVO N={len(srv_t)} ({len(srv_t)/dur:.1f} Hz)")

    # -- Yaw rate (the regulated variable) --------------------------------
    r_ac = rate - np.mean(rate)
    f_fft, fs = dominant_freq(att_t, rate)
    f_zc = zero_cross_freq(att_t, rate)
    print("\n--- Yaw RATE (ATTITUDE.yawspeed = gyro:z) ---")
    print(f"  mean = {_fmt(np.mean(rate))} rad/s ({_fmt(math.degrees(np.mean(rate)),1)} deg/s)")
    print(f"  rms(AC) = {_fmt(np.std(r_ac))} rad/s   "
          f"peak-to-peak = {_fmt(np.ptp(rate))} rad/s ({_fmt(math.degrees(np.ptp(rate)),1)} deg/s)")
    print(f"  dominant freq = {_fmt(f_fft,2)} Hz (period {_fmt(1/f_fft if f_fft else None,2)} s) [FFT]"
          f"   zero-cross = {_fmt(f_zc,2)} Hz")

    # -- Yaw heading ------------------------------------------------------
    print("\n--- Yaw HEADING (ATTITUDE.yaw) ---")
    print(f"  mean = {_fmt(math.degrees(np.mean(yaw)),1)} deg   "
          f"peak-to-peak = {_fmt(math.degrees(np.ptp(yaw)),1)} deg")

    # -- Servo4 (yaw motor PWM) -------------------------------------------
    pwm_ac = pwm - np.mean(pwm)
    fs_fft, _ = dominant_freq(srv_t, pwm)
    fs_zc = zero_cross_freq(srv_t, pwm)
    below = float(np.mean(pwm < YAW_DEADBAND_US)) * 100.0
    at_min = float(np.mean(pwm <= SERVO4_MIN_US + 1)) * 100.0
    print("\n--- SERVO4 (yaw motor PWM) ---")
    print(f"  mean = {_fmt(np.mean(pwm),0)} us   peak-to-peak = {_fmt(np.ptp(pwm),0)} us   "
          f"rms(AC) = {_fmt(np.std(pwm_ac),0)} us")
    print(f"  min={_fmt(np.min(pwm),0)}  max={_fmt(np.max(pwm),0)}   "
          f"below deadband({YAW_DEADBAND_US:.0f}us) = {below:.0f}%   at SERVO4_MIN = {at_min:.0f}%")
    print(f"  dominant freq = {_fmt(fs_fft,2)} Hz   zero-cross = {_fmt(fs_zc,2)} Hz")

    # -- Actuator <-> response --------------------------------------------
    period = (1.0 / f_fft) if f_fft else None
    lag = phase_lag_ms(srv_t, pwm, att_t, rate, period_s=period)
    print("\n--- Actuator <-> response ---")
    if f_fft and fs_fft:
        match = "MATCH" if abs(f_fft - fs_fft) < 0.15 * max(f_fft, fs_fft) else "differ"
        print(f"  servo freq {_fmt(fs_fft,2)} Hz vs yaw-rate freq {_fmt(f_fft,2)} Hz -> {match}")
    if lag is not None and f_fft:
        deg = (lag / 1000.0) * f_fft * 360.0
        print(f"  yaw-rate lags servo by {_fmt(lag,0)} ms ({_fmt(deg,0)} deg at {_fmt(f_fft,2)} Hz)")
    if below > 5.0 and np.ptp(pwm) > 100:
        print("  -> motor bangs across the deadband (one-directional actuator + dead zone):")
        print("     classic yaw LIMIT CYCLE, not a linear underdamped response.")

    # -- Correction law: is the oscillating PWM a correction for the yaw rate? --
    correction_analysis(srv_t, pwm, att_t, rate)

    # -- Lua yaw trim interaction -----------------------------------------
    print("\n--- Lua yaw-trim feedforward (NVF) ---")
    kd = nvf["YFF_KD"]
    a = nvf["YFF_A"]
    print(f"  YFF_KD (yaw-D gain, SCR_USER1) = "
          f"{_fmt(kd[-1],4) if len(kd) else 'not streamed'}"
          f"{'' if len(kd)<2 else f'  (range {_fmt(kd.min(),4)}..{_fmt(kd.max(),4)})'}")
    print(f"  YFF_A  (calibrated slope)      = {_fmt(a[-1],1) if len(a) else 'n/a'} rad/s per u")
    trim, trim_t = nvf["YFF_T"], nvf_t["YFF_T"]
    if len(trim) >= 2:
        ts = (trim_t - trim_t[0]) / 1000.0
        drift = np.polyfit(ts, trim, 1)[0]
        print(f"  YFF_T  (trim = H_YAW_TRIM)     : mean={_fmt(np.mean(trim),4)} "
              f"std={_fmt(np.std(trim),4)} drift={_fmt(drift,5)}/s  (N={len(trim)})")
    u = nvf["YFF_U"]
    if len(u):
        u_from_srv = (np.mean(pwm) - SERVO4_MIN_US) / (SERVO4_MAX_US - SERVO4_MIN_US)
        print(f"  YFF_U  (applied throttle u)    : mean={_fmt(np.mean(u),4)} "
              f"(servo-derived mean u={_fmt(u_from_srv,4)})")
    gz = nvf["YFF_GZ"]
    if len(gz):
        print(f"  YFF_GZ (Lua psi_dot)           : mean={_fmt(np.mean(gz),3)} "
              f"rms={_fmt(np.std(gz),3)} rad/s  (ATTITUDE rms={_fmt(np.std(r_ac),3)})")

    # -- Interpretation ---------------------------------------------------
    print("\n--- Interpretation ---")
    kd_now = kd[-1] if len(kd) else 0.0
    if len(trim) >= 2 and abs(np.polyfit((trim_t-trim_t[0])/1000.0, trim, 1)[0]) < 0.005:
        print("  * Trim (DC hold) is essentially steady -> the feedforward has converged;")
        print("    it is NOT the source of the fast oscillation (it is low-passed at 0.3 s).")
    else:
        print("  * Trim is still drifting -> DC hold not yet settled over this window.")
    if kd_now and kd_now > 0:
        print(f"  * Yaw-D active (kd={kd_now:.4f}); compare rms(AC) across runs at different kd")
        print("    to see whether it is damping the oscillation.")
    else:
        print("  * Yaw-D is OFF (kd=0); the oscillation is uncontrolled by the D term.")

    if do_plot:
        _plot(D, t0)


def _plot(D, t0):
    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"  [plot skipped: {e}]")
        return
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    ta = (D["att_t"] - t0) / 1000.0
    ts = (D["srv_t"] - t0) / 1000.0
    ax[0].plot(ta, np.degrees(D["att_rate"]), lw=0.8)
    ax[0].set_ylabel("yaw rate [deg/s]"); ax[0].grid(True, alpha=0.3)
    ax[1].plot(ts, D["srv_pwm"], lw=0.8, color="tab:red")
    ax[1].axhline(YAW_DEADBAND_US, ls="--", color="k", alpha=0.5, label="deadband")
    ax[1].set_ylabel("SERVO4 [us]"); ax[1].grid(True, alpha=0.3); ax[1].legend()
    tt = (D["nvf_t"]["YFF_T"] - t0) / 1000.0
    if len(tt):
        ax[2].plot(tt, D["nvf"]["YFF_T"], marker=".", label="YFF_T (trim)")
    tu = (D["nvf_t"]["YFF_U"] - t0) / 1000.0
    if len(tu):
        ax[2].plot(tu, D["nvf"]["YFF_U"], marker=".", label="YFF_U (u)")
    ax[2].set_ylabel("throttle [0..1]"); ax[2].set_xlabel("t [s]")
    ax[2].grid(True, alpha=0.3); ax[2].legend()
    plt.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description="Analyze yaw oscillation + Lua trim interaction")
    ap.add_argument("log", help="path to a .mavlink.jsonl calibrate log")
    ap.add_argument("--plot", action="store_true", help="show time-series plots")
    args = ap.parse_args()
    analyze(args.log, do_plot=args.plot)


if __name__ == "__main__":
    main()
