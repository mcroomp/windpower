"""
pump_diagnosis.py -- Pump cycle controller decision audit.

Default output (AI-friendly):
  - Compact summary printed to stdout
  - <log_dir>/pump_diagnosis_summary.json   -- key stats, flag counts, osc + correlation highlights
  - <log_dir>/pump_diagnosis_buckets.csv    -- per-bucket table (all controller metrics)
  - <log_dir>/pump_diagnosis_osc.csv        -- dominant-frequency per phase x signal
  - <log_dir>/pump_diagnosis_corr.csv       -- pairwise coherence + phase-lag at dominant tension freq
                                               and Pearson r in time domain, per phase segment

Usage
-----
    .venv/Scripts/python.exe simulation/analysis/pump_diagnosis.py
    .venv/Scripts/python.exe simulation/analysis/pump_diagnosis.py --test test_pump_cycle_unified --bucket 1
    .venv/Scripts/python.exe simulation/analysis/pump_diagnosis.py --verbose   # full tables to stdout

pump_diagnosis_corr.csv columns:
  phase          phase segment name
  t_start        segment start [s]
  dur_s          segment duration [s]
  f_ref_hz       dominant tension frequency used as reference [Hz]
  sig_a          reference signal name
  sig_b          comparison signal name
  pearson_r      Pearson correlation in time domain (detrended), range -1..1
  coherence      magnitude-squared coherence at f_ref (0=uncorrelated, 1=locked)
  phase_lag_s    time by which sig_b lags sig_a at f_ref [s]; negative = sig_b leads
  phase_lag_deg  same, in degrees
  pp_a           peak-to-peak amplitude of sig_a (detrended)
  pp_b           peak-to-peak amplitude of sig_b (detrended)

Interpretation:
  coherence > 0.7   -- signals are strongly coupled at that frequency
  phase_lag_s > 0   -- sig_b responds after sig_a  (sig_a drives, sig_b lags)
  phase_lag_s < 0   -- sig_b anticipates sig_a     (sig_b may be the driver)
"""

import sys, math, csv, json, argparse
from pathlib import Path
from collections import defaultdict

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aero import create_aero
import rotor_definition as rd

# ── Constants ──────────────────────────────────────────────────────────────────

_ROTOR  = rd.default()
_AERO   = create_aero(_ROTOR)
MASS    = _ROTOR.mass_kg
G       = 9.81
MG      = MASS * G
WIND    = np.array([0.0, 10.0, 0.0])

_d   = json.loads((Path(__file__).resolve().parents[1] / "steady_state_starting.json").read_text())
_R0  = np.array(_d["R0"]).reshape(3, 3)
OMEGA = float(_d["omega_spin"])

COL_MIN       = -0.28
COL_MAX       =  0.10
BREAK_LOAD_N  = 620.0
FLOOR_ALT_M   = 1.5


def _aero_thrust(col):
    res = _AERO.compute_forces(col, 0.0, 0.0, _R0, np.zeros(3), OMEGA, WIND, t=45.0)
    return float(np.linalg.norm(res.F_world))


_T_THRUST_MIN = _aero_thrust(COL_MIN)
_T_THRUST_MAX = _aero_thrust(COL_MAX)


# ── Physics helpers ────────────────────────────────────────────────────────────

def _elevation(pos):
    tlen = np.linalg.norm(pos)
    return math.degrees(math.asin(max(-1.0, min(1.0, float(-pos[2]) / max(tlen, 1e-6)))))


def _angle_deg(v1, v2):
    n1 = np.linalg.norm(v1); n2 = np.linalg.norm(v2)
    if n1 < 1e-9 or n2 < 1e-9:
        return float("nan")
    c = float(np.dot(v1, v2)) / (n1 * n2)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _t_thrust_needed(el_deg, delta_deg):
    el = math.radians(el_deg)
    d  = math.radians(delta_deg)
    sd = math.sin(d)
    if sd < 1e-6:
        return float("inf")
    if delta_deg >= 90.0:
        return float("nan")
    return MG * math.cos(el) / sd


def _t_min_achievable(el_deg):
    el = math.radians(el_deg)
    if _T_THRUST_MIN < MG * math.cos(el):
        return float("nan")
    delta = math.asin(MG * math.cos(el) / _T_THRUST_MIN)
    return _T_THRUST_MIN * math.cos(delta) - MG * math.sin(el)


# ── Load telemetry ─────────────────────────────────────────────────────────────

def load_telemetry(csv_path):
    rows = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rec = {}
            for k, v in row.items():
                if k in ("phase", "note"):
                    rec[k] = v
                else:
                    try:
                        rec[k] = float(v)
                    except (ValueError, TypeError):
                        rec[k] = float("nan")
            rows.append(rec)
    return rows


# ── Phase transitions ──────────────────────────────────────────────────────────

def _phase_transitions(rows):
    events = []
    if not rows:
        return events
    cur_phase = rows[0].get("phase", "")
    t0 = rows[0]["t_sim"]
    rl0 = rows[0].get("tether_rest_length", float("nan"))
    alt0 = float(-rows[0].get("pos_z", 0))
    for r in rows[1:]:
        ph = r.get("phase", "")
        if ph != cur_phase:
            events.append((t0, r["t_sim"], cur_phase, rl0, r.get("tether_rest_length", float("nan")),
                           alt0, float(-r.get("pos_z", 0))))
            cur_phase = ph
            t0   = r["t_sim"]
            rl0  = r.get("tether_rest_length", float("nan"))
            alt0 = float(-r.get("pos_z", 0))
    r = rows[-1]
    events.append((t0, r["t_sim"], cur_phase, rl0, r.get("tether_rest_length", float("nan")),
                   alt0, float(-r.get("pos_z", 0))))
    return events


# ── Slack events ───────────────────────────────────────────────────────────────

def _slack_events(rows, tension_window=5):
    events = []
    n = len(rows)
    i = 0
    while i < n:
        if rows[i].get("tether_slack", 0) > 0.5:
            j = i
            while j < n and rows[j].get("tether_slack", 0) > 0.5:
                j += 1
            seg    = rows[i:j]
            before = rows[max(0, i - tension_window):i]
            after  = rows[j:min(n, j + tension_window)]

            def mean_col(rlist, col):
                vals = [r[col] for r in rlist if col in r and not math.isnan(r.get(col, float("nan")))]
                return float(np.mean(vals)) if vals else float("nan")

            events.append(dict(
                phase          = seg[0].get("phase", ""),
                t_start        = seg[0]["t_sim"],
                t_end          = seg[-1]["t_sim"],
                duration_s     = seg[-1]["t_sim"] - seg[0]["t_sim"],
                n_rows         = len(seg),
                alt_start      = float(-seg[0].get("pos_z", 0)),
                alt_end        = float(-seg[-1].get("pos_z", 0)),
                rest_l_start   = seg[0].get("tether_rest_length", float("nan")),
                rest_l_end     = seg[-1].get("tether_rest_length", float("nan")),
                tension_before = mean_col(before, "tether_tension"),
                tension_after  = mean_col(after,  "tether_tension"),
                winch_before   = mean_col(before, "winch_speed_ms"),
            ))
            i = j
        else:
            i += 1
    return events


# ── Oscillation analysis ───────────────────────────────────────────────────────

_OSC_SIGNALS = [
    ("tether_tension",              "tension_N"),
    ("collective_rad",              "col_act"),
    ("collective_from_tension_ctrl","col_tpi"),
    ("aero_T",                      "aero_T_N"),
    ("aero_v_i",                    "v_inflow"),
    ("aero_v_axial",                "v_axial"),
    ("pos_z",                       "alt_m"),
    ("vel_z",                       "vel_z"),
    ("tether_length",               "t_len"),
]

_MIN_SEGMENT_ROWS = 40


def _dominant_freq(sig, dt):
    if len(sig) < _MIN_SEGMENT_ROWS:
        return float("nan"), float("nan")
    x = np.asarray(sig, dtype=float)
    x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))
    if np.std(x) < 1e-10:
        return 0.0, 0.0
    n     = len(x)
    freqs = np.fft.rfftfreq(n, d=dt)
    amps  = np.abs(np.fft.rfft(x)) * 2.0 / n
    idx   = int(np.argmax(amps[1:])) + 1
    return float(freqs[idx]), float(amps[idx] * 2.0)


def _oscillation_rows(rows, dt):
    segments = []
    cur_phase = rows[0].get("phase", "")
    seg_rows  = [rows[0]]
    for r in rows[1:]:
        ph = r.get("phase", "")
        if ph != cur_phase:
            segments.append((cur_phase, seg_rows))
            cur_phase = ph
            seg_rows  = []
        seg_rows.append(r)
    segments.append((cur_phase, seg_rows))

    osc_rows = []
    for phase, seg in segments:
        dur = seg[-1].get("t_sim", 0.0) - seg[0].get("t_sim", 0.0)
        if len(seg) < _MIN_SEGMENT_ROWS:
            continue
        t_start = seg[0].get("t_sim", 0.0)
        for col, label in _OSC_SIGNALS:
            vals = [r[col] for r in seg if col in r and not math.isnan(r.get(col, float("nan")))]
            if len(vals) < _MIN_SEGMENT_ROWS:
                continue
            if col == "pos_z":
                vals = [-v for v in vals]
            f_hz, pp = _dominant_freq(vals, dt)
            arr = np.asarray(vals, dtype=float)
            arr -= np.mean(arr)
            rms = float(np.sqrt(np.mean(arr ** 2)))
            osc_rows.append({
                "phase":    phase,
                "t_start":  t_start,
                "dur_s":    dur,
                "signal":   label,
                "f_dom_hz": round(f_hz, 4) if not math.isnan(f_hz) else None,
                "pp_amp":   round(pp, 5)   if not math.isnan(pp)   else None,
                "rms":      round(rms, 5),
            })
    return osc_rows


# ── Per-bucket computation ─────────────────────────────────────────────────────

def _compute_buckets(rows, bucket_s):
    buckets = defaultdict(list)
    for r in rows:
        bk = int(r["t_sim"] / bucket_s) * bucket_s
        buckets[bk].append(r)

    bucket_rows = []
    flag_counts = defaultdict(int)

    for bk in sorted(buckets.keys()):
        grp   = buckets[bk]
        phase = grp[len(grp) // 2].get("phase", "")

        def avg(key):
            vals = [r[key] for r in grp if key in r and not math.isnan(r[key])]
            return float(np.mean(vals)) if vals else float("nan")

        def std(key):
            vals = [r[key] for r in grp if key in r and not math.isnan(r[key])]
            return float(np.std(vals)) if len(vals) > 1 else float("nan")

        def peak_to_peak(key):
            vals = [r[key] for r in grp if key in r and not math.isnan(r[key])]
            return float(np.max(vals) - np.min(vals)) if len(vals) > 1 else float("nan")

        def frac(pred):
            n = sum(1 for r in grp if pred(r))
            return n / max(len(grp), 1)

        pos    = np.array([avg("pos_x"), avg("pos_y"), avg("pos_z")])
        bz_eq  = np.array([avg("bz_eq_x"), avg("bz_eq_y"), avg("bz_eq_z")])
        bz_act = np.array([avg("r02"), avg("r12"), avg("r22")])
        tdir   = pos / max(np.linalg.norm(pos), 1e-6)

        alt     = float(-pos[2])
        tgt     = avg("gnd_alt_cmd_m")
        alt_err = alt - tgt if not math.isnan(tgt) else float("nan")
        el      = _elevation(pos)
        d_eq    = _angle_deg(bz_eq, tdir)
        d_act   = _angle_deg(bz_act, tdir)

        t_need = _t_thrust_needed(el, d_eq)
        if math.isnan(t_need):
            feas = "PAST90"
        elif t_need < _T_THRUST_MIN:
            feas = "TOO_LOW"
        elif t_need > _T_THRUST_MAX:
            feas = "TOO_HIGH"
        else:
            feas = "OK"

        t_set   = avg("tension_setpoint")
        t_act   = avg("tether_tension")
        col_tpi = avg("collective_from_tension_ctrl")
        col_act = avg("collective_rad")
        col_std = std("collective_rad")
        col_pp  = peak_to_peak("collective_rad")

        slack_f = frac(lambda r: r.get("tether_slack", 0) > 0.5)
        floor_f = frac(lambda r: (-r["pos_z"]) < FLOOR_ALT_M if "pos_z" in r else False)
        spike_f = frac(lambda r: r.get("tether_tension", 0) > BREAK_LOAD_N)

        flags = []
        if slack_f  > 0.05: flags.append(f"SLACK={slack_f*100:.0f}%")
        if floor_f  > 0.05: flags.append(f"FLOOR={floor_f*100:.0f}%")
        if spike_f  > 0.02: flags.append(f"SPIKE={spike_f*100:.0f}%")
        if not math.isnan(d_eq) and d_eq > 90:
            flags.append(f"BZ_PAST_PERP({d_eq:.0f}deg)")
        if not math.isnan(alt_err) and alt_err < -5:
            flags.append(f"ALT_LOW({alt_err:+.0f}m)")
        if not math.isnan(alt_err) and alt_err > 10:
            flags.append(f"ALT_HIGH({alt_err:+.0f}m)")
        if not math.isnan(col_std) and col_std > 0.010:
            flags.append(f"COL_OSC(sd={col_std:.3f})")
        if feas not in ("OK",):
            flags.append(f"T_{feas}")
        t_min_ach = _t_min_achievable(el)
        if not math.isnan(t_set) and not math.isnan(t_min_ach) and t_set < t_min_ach:
            flags.append(f"SET_IMPOSSIBLE(min={t_min_ach:.0f}N)")

        for f in flags:
            key = f.split("(")[0].split("=")[0]
            flag_counts[key] += 1

        def r4(v): return round(v, 4) if not math.isnan(v) else None
        def r1(v): return round(v, 1) if not math.isnan(v) else None

        bucket_rows.append({
            "t":       bk,
            "phase":   phase,
            "alt":     r1(alt),
            "tgt":     r1(tgt),
            "alt_err": r1(alt_err),
            "el":      r1(el),
            "d_eq":    r1(d_eq),
            "d_act":   r1(d_act),
            "T_need":  r1(t_need) if not math.isinf(t_need) else None,
            "feasible":feas,
            "T_set":   r1(t_set),
            "T_act":   r1(t_act),
            "col_tpi": r4(col_tpi),
            "col_act": r4(col_act),
            "col_sd":  r4(col_std),
            "col_pp":  r4(col_pp),
            "flags":   "  ".join(flags),
        })

    return bucket_rows, dict(flag_counts)


# ── Writers ────────────────────────────────────────────────────────────────────

def _write_buckets_csv(bucket_rows, path):
    if not bucket_rows:
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(bucket_rows[0].keys()))
        w.writeheader()
        w.writerows(bucket_rows)


def _write_osc_csv(osc_rows, path):
    if not osc_rows:
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(osc_rows[0].keys()))
        w.writeheader()
        w.writerows(osc_rows)


# ── Correlation / coherence analysis ──────────────────────────────────────────
#
# Signals included in pairwise analysis.  pos_z is sign-flipped to altitude.
_CORR_SIGNALS = [
    ("tether_tension",               "tension_N"),
    ("collective_rad",               "col_act"),
    ("collective_from_tension_ctrl", "col_tpi"),
    ("aero_T",                       "aero_T"),
    ("aero_v_i",                     "v_inflow"),
    ("aero_v_axial",                 "v_axial"),
    ("aero_v_inplane",               "v_inplane"),
    ("pos_z",                        "alt_m"),      # sign flipped below
    ("vel_z",                        "vel_z"),
    ("tether_length",                "t_len"),
    ("tether_extension",             "t_ext"),
    ("omega_rotor",                  "rpm"),
    ("winch_speed_ms",               "winch_v"),
]

# Reference signal for phase-lag calculations (always computed relative to this)
_CORR_REF = "tension_N"


def _extract_signal(seg, col, label):
    """Extract a clean float array for one signal from a phase segment."""
    vals = []
    for r in seg:
        v = r.get(col, float("nan"))
        if not math.isnan(v):
            vals.append(float(v))
        else:
            vals.append(float("nan"))
    arr = np.asarray(vals, dtype=float)
    if label == "alt_m":           # flip pos_z → altitude
        arr = -arr
    return arr


def _detrend(arr):
    """Linear detrend, ignoring NaN."""
    good = np.isfinite(arr)
    if good.sum() < 4:
        return arr - np.nanmean(arr)
    x = np.arange(len(arr), dtype=float)
    p = np.polyfit(x[good], arr[good], 1)
    return arr - np.polyval(p, x)


def _cross_spectral_pair(a_raw, b_raw, dt, f_target):
    """
    Coherence and phase lag of b relative to a at f_target.

    Returns (coherence, phase_lag_s, phase_lag_deg).
    phase_lag_s > 0  means b lags a (a leads).
    phase_lag_s < 0  means b leads a.
    """
    n = len(a_raw)
    if n < _MIN_SEGMENT_ROWS or f_target <= 0 or math.isnan(f_target):
        return float("nan"), float("nan"), float("nan")

    a = _detrend(a_raw.copy())
    b = _detrend(b_raw.copy())

    # Replace NaN with interpolation; if >20% NaN skip
    for arr in (a, b):
        nan_mask = ~np.isfinite(arr)
        if nan_mask.sum() > 0.20 * n:
            return float("nan"), float("nan"), float("nan")
        if nan_mask.any():
            idx = np.arange(n)
            arr[nan_mask] = np.interp(idx[nan_mask], idx[~nan_mask], arr[~nan_mask])

    A = np.fft.rfft(a)
    B = np.fft.rfft(b)
    freqs = np.fft.rfftfreq(n, d=dt)

    k = int(np.argmin(np.abs(freqs - f_target)))
    k = max(1, k)

    Paa = float(np.abs(A[k]) ** 2)
    Pbb = float(np.abs(B[k]) ** 2)
    Pab = A[k] * np.conj(B[k])

    denom = Paa * Pbb
    coh   = float(np.abs(Pab) ** 2 / max(denom, 1e-30))
    coh   = min(coh, 1.0)   # clamp numerical noise

    # Phase of cross-spectrum: angle(A·conj(B)) = phase_A - phase_B
    # So positive = A leads B = B lags A
    phase_rad = float(np.angle(Pab))
    f_used    = float(freqs[k])
    lag_s     = phase_rad / (2.0 * math.pi * f_used) if f_used > 0 else float("nan")
    lag_deg   = math.degrees(phase_rad)

    return coh, lag_s, lag_deg


def _correlation_rows(rows, dt, osc_rows):
    """
    For each phase segment, compute pairwise coherence + phase lag at the dominant
    tension frequency, plus Pearson r in the time domain.

    Returns list of dicts (one per segment x signal-pair).
    """
    # Build a map: phase -> dominant tension frequency from osc_rows
    tension_freq = {}
    for r in osc_rows:
        if r["signal"] == "tension_N" and r["f_dom_hz"] is not None:
            tension_freq[r["phase"]] = r["f_dom_hz"]

    # Split rows into phase segments
    segments = []
    cur_phase = rows[0].get("phase", "")
    seg_rows  = [rows[0]]
    for r in rows[1:]:
        ph = r.get("phase", "")
        if ph != cur_phase:
            segments.append((cur_phase, seg_rows))
            cur_phase = ph
            seg_rows  = []
        seg_rows.append(r)
    segments.append((cur_phase, seg_rows))

    ref_label = _CORR_REF

    result = []
    for phase, seg in segments:
        if len(seg) < _MIN_SEGMENT_ROWS:
            continue
        t_start = seg[0].get("t_sim", 0.0)
        dur_s   = seg[-1].get("t_sim", 0.0) - t_start
        f_ref   = tension_freq.get(phase, float("nan"))

        # Extract all signals for this segment
        signals = {}
        for col, label in _CORR_SIGNALS:
            arr = _extract_signal(seg, col, label)
            if np.isfinite(arr).sum() >= _MIN_SEGMENT_ROWS:
                signals[label] = arr

        if ref_label not in signals:
            continue

        ref_arr = signals[ref_label]
        ref_dt  = _detrend(ref_arr.copy())
        ref_pp  = float(np.nanmax(ref_dt) - np.nanmin(ref_dt))

        for label, arr in signals.items():
            if label == ref_label:
                continue

            # Pearson r (time domain, detrended)
            a_dt = _detrend(ref_arr.copy())
            b_dt = _detrend(arr.copy())
            good = np.isfinite(a_dt) & np.isfinite(b_dt)
            if good.sum() >= _MIN_SEGMENT_ROWS:
                pearson_r = float(np.corrcoef(a_dt[good], b_dt[good])[0, 1])
            else:
                pearson_r = float("nan")

            # Coherence + phase lag at dominant tension frequency
            coh, lag_s, lag_deg = _cross_spectral_pair(ref_arr, arr, dt, f_ref)

            b_dt_pp = float(np.nanmax(b_dt) - np.nanmin(b_dt))

            def _r(v, n=4):
                return round(v, n) if math.isfinite(v) else None

            result.append({
                "phase":         phase,
                "t_start":       round(t_start, 2),
                "dur_s":         round(dur_s, 2),
                "f_ref_hz":      _r(f_ref, 4),
                "sig_a":         ref_label,
                "sig_b":         label,
                "pearson_r":     _r(pearson_r, 4),
                "coherence":     _r(coh, 4),
                "phase_lag_s":   _r(lag_s, 5),
                "phase_lag_deg": _r(lag_deg, 2),
                "pp_a":          _r(ref_pp, 3),
                "pp_b":          _r(b_dt_pp, 3),
            })

    return result


def _write_corr_csv(corr_rows, path):
    if not corr_rows:
        return
    with open(path, "w", newline="") as f:
        # NOTE: coherence is single-epoch (one DFT window) so is always 1.0 for
        # a clean sinusoid — treat it as a detection flag only.
        # pearson_r (time domain) and phase_lag_deg are the primary diagnostics.
        w = csv.DictWriter(f, fieldnames=list(corr_rows[0].keys()))
        w.writeheader()
        w.writerows(corr_rows)


# ── Verbose table (--verbose) ──────────────────────────────────────────────────

def _print_bucket_table(bucket_rows, dt, n_rows):
    print()
    print(f"pump_diagnosis -- per-bucket table  bucket={bucket_rows[1]['t'] - bucket_rows[0]['t']:.1f}s  "
          f"rows={n_rows}  dt={dt*1000:.2f}ms")
    print()
    hdr = (f"  {'t':>5}  {'phase':<20}  {'alt':>5}  {'tgt':>5}  {'err':>5}  "
           f"{'el':>5}  {'d_eq':>5}  {'d_act':>5}  "
           f"{'T_need':>6}  {'feas':>6}  "
           f"{'T_set':>6}  {'T_act':>6}  "
           f"{'col_tpi':>7}  {'col_act':>7}  {'col_sd':>6}  {'col_pp':>7}  flags")
    print(hdr)
    print("  " + "-" * (len(hdr) + 30))

    def f(v, fmt):
        if v is None: return "  nan"
        return format(v, fmt)

    for b in bucket_rows:
        print(f"  {b['t']:>5.0f}  {b['phase']:<20}  {f(b['alt'],'5.1f')}  {f(b['tgt'],'5.1f')}  "
              f"{f(b['alt_err'],'+5.1f')}  "
              f"{f(b['el'],'5.1f')}  {f(b['d_eq'],'5.1f')}  {f(b['d_act'],'5.1f')}  "
              f"{f(b['T_need'],'6.0f')}  {b['feasible']:>6}  "
              f"{f(b['T_set'],'6.0f')}  {f(b['T_act'],'6.0f')}  "
              f"{f(b['col_tpi'],'7.3f')}  {f(b['col_act'],'7.3f')}  "
              f"{f(b['col_sd'],'6.4f')}  {f(b['col_pp'],'7.4f')}  "
              + b["flags"])


def _print_osc_table(osc_rows, dt):
    if not osc_rows:
        return
    print()
    print(f"Oscillation analysis  dt={dt*1000:.2f}ms  fs={1/dt:.0f}Hz")
    print(f"  {'phase':<24}  {'dur_s':>5}  {'signal':<18}  "
          f"{'f_dom_Hz':>8}  {'pp_amp':>8}  {'rms':>8}")
    print("  " + "-" * 80)
    last_phase = None
    for r in osc_rows:
        ph    = r["phase"] if r["phase"] != last_phase else ""
        dur_s = f"{r['dur_s']:5.1f}" if r["phase"] != last_phase else "     "
        last_phase = r["phase"]
        fhz = f"{r['f_dom_hz']:8.3f}" if r["f_dom_hz"] is not None else "     nan"
        pp  = f"{r['pp_amp']:8.4f}"   if r["pp_amp"]   is not None else "     nan"
        print(f"  {ph:<24}  {dur_s}  {r['signal']:<18}  {fhz}  {pp}  {r['rms']:8.4f}")
        if r["phase"] != (osc_rows[osc_rows.index(r)+1]["phase"]
                          if osc_rows.index(r) + 1 < len(osc_rows) else None):
            print()


# ── Main ───────────────────────────────────────────────────────────────────────

def diagnose(csv_path, bucket_s=5.0, verbose=False, out_dir=None):
    rows = load_telemetry(csv_path)
    if not rows:
        print(f"No telemetry rows found at {csv_path}")
        return

    if out_dir is None:
        out_dir = Path(csv_path).parent
    out_dir = Path(out_dir)

    t0 = rows[0].get("t_sim", 0.0)
    t1 = rows[1].get("t_sim", 0.0) if len(rows) > 1 else t0 + 0.05
    dt = max(t1 - t0, 1e-6)

    bucket_rows, flag_counts = _compute_buckets(rows, bucket_s)
    osc_rows    = _oscillation_rows(rows, dt)
    corr_rows   = _correlation_rows(rows, dt, osc_rows)
    transitions = _phase_transitions(rows)
    slack_evs   = _slack_events(rows)

    # ── Write files ──────────────────────────────────────────────────────────
    buckets_csv  = out_dir / "pump_diagnosis_buckets.csv"
    osc_csv      = out_dir / "pump_diagnosis_osc.csv"
    corr_csv     = out_dir / "pump_diagnosis_corr.csv"
    summary_json = out_dir / "pump_diagnosis_summary.json"

    _write_buckets_csv(bucket_rows, buckets_csv)
    _write_osc_csv(osc_rows, osc_csv)
    _write_corr_csv(corr_rows, corr_csv)

    # Build osc summary: most-flagged signals per phase type
    osc_summary = []
    for r in osc_rows:
        if r["pp_amp"] is not None and r["pp_amp"] > 0.001:
            osc_summary.append(r)

    # Count n_cycles (count distinct "reel-out" phase names)
    n_cycles = len({ph for _, _, ph, *_ in transitions if "reel-out" in ph})

    t_total   = rows[-1]["t_sim"] - rows[0]["t_sim"]
    n_buckets = len(bucket_rows)
    flagged_buckets = [b for b in bucket_rows if b["flags"]]

    summary = {
        "test":       str(Path(csv_path).parent.name),
        "csv":        str(csv_path),
        "rows":       len(rows),
        "dt_ms":      round(dt * 1000, 4),
        "fs_hz":      round(1.0 / dt, 1),
        "t_total_s":  round(t_total, 2),
        "bucket_s":   bucket_s,
        "n_buckets":  n_buckets,
        "n_cycles":   n_cycles,
        "flag_counts":flag_counts,
        "n_flagged_buckets": len(flagged_buckets),
        "slack_events": len(slack_evs),
        "slack_total_s": round(sum(e["duration_s"] for e in slack_evs), 3),
        "phase_timeline": [
            {"phase": ph, "t_start": round(t0, 2), "t_end": round(t1, 2),
             "dur_s": round(t1 - t0, 2)}
            for t0, t1, ph, *_ in transitions
        ],
        "osc_significant": [
            r for r in osc_summary
            if (r["pp_amp"] or 0) > 0.01 or (r["rms"] or 0) > 0.005
        ],
        # Strongly coherent pairs (coherence > 0.5) sorted by coherence desc
        "corr_highlights": sorted(
            [r for r in corr_rows
             if r["coherence"] is not None and r["coherence"] > 0.5],
            key=lambda r: r["coherence"], reverse=True
        )[:20],
        "files": {
            "buckets_csv":  str(buckets_csv),
            "osc_csv":      str(osc_csv),
            "corr_csv":     str(corr_csv),
            "summary_json": str(summary_json),
        },
    }

    with open(summary_json, "w") as f:
        json.dump(summary, f, indent=2)

    # ── Compact stdout summary ────────────────────────────────────────────────
    print()
    print(f"pump_diagnosis  {summary['test']}  rows={len(rows)}  "
          f"dt={dt*1000:.2f}ms ({1/dt:.0f} Hz)  bucket={bucket_s}s  t={t_total:.0f}s")
    print(f"  cycles={n_cycles}  buckets={n_buckets}  flagged={len(flagged_buckets)}")

    if flag_counts:
        fc_str = "  ".join(f"{k}={v}" for k, v in sorted(flag_counts.items()))
        print(f"  Flag counts: {fc_str}")

    print(f"  Slack: {len(slack_evs)} event(s)  {summary['slack_total_s']:.3f}s total")

    # Oscillation highlights: top 4 by pp_amp, tension and collective only
    top_osc = sorted(
        [r for r in osc_rows if r["signal"] in ("tension_N", "col_act") and (r["pp_amp"] or 0) > 0],
        key=lambda r: r["pp_amp"] or 0, reverse=True
    )[:4]
    if top_osc:
        print("  Osc highlights (top pp):")
        for r in top_osc:
            print(f"    {r['phase']:<28}  {r['signal']:<12}  "
                  f"f={r['f_dom_hz']:.3f} Hz  pp={r['pp_amp']:.4f}  rms={r['rms']:.4f}")

    # Correlation highlights: strongly coherent pairs, sorted by coherence
    top_corr = sorted(
        [r for r in corr_rows if (r["coherence"] or 0) > 0.5],
        key=lambda r: r["coherence"] or 0, reverse=True
    )[:8]
    if top_corr:
        print("  Corr highlights (coherence > 0.5 at tension f_dom):")
        print(f"    {'phase':<24}  {'sig_b':<12}  {'f_ref':>7}  "
              f"{'pearson_r':>9}  {'coherence':>9}  {'lag_ms':>8}  {'lag_deg':>8}")
        for r in top_corr:
            lag_ms = f"{r['phase_lag_s']*1000:+8.1f}" if r["phase_lag_s"] is not None else "     nan"
            lag_dg = f"{r['phase_lag_deg']:+8.1f}"    if r["phase_lag_deg"] is not None else "     nan"
            fref   = f"{r['f_ref_hz']:.3f}" if r["f_ref_hz"] is not None else "  nan"
            pr     = f"{r['pearson_r']:+.3f}" if r["pearson_r"] is not None else "  nan"
            coh    = f"{r['coherence']:.3f}" if r["coherence"] is not None else " nan"
            print(f"    {r['phase']:<24}  {r['sig_b']:<12}  {fref:>7}  "
                  f"{pr:>9}  {coh:>9}  {lag_ms}  {lag_dg}")

    print()
    print(f"  Output files:")
    print(f"    {buckets_csv}")
    print(f"    {osc_csv}")
    print(f"    {corr_csv}")
    print(f"    {summary_json}")
    print()

    # ── Verbose: full tables ──────────────────────────────────────────────────
    if verbose:
        _print_bucket_table(bucket_rows, dt, len(rows))

        print()
        print("Phase transition timeline:")
        print(f"  {'phase':<24}  {'t_start':>7}  {'t_end':>7}  {'dur':>6}  "
              f"{'rest_l0':>7}  {'rest_l1':>7}  {'alt0':>6}  {'alt1':>6}")
        print("  " + "-" * 80)
        for t0, t1, ph, rl0, rl1, a0, a1 in transitions:
            def fv(v): return f"{v:7.2f}" if not math.isnan(v) else "    nan"
            print(f"  {ph:<24}  {t0:>7.2f}  {t1:>7.2f}  {t1-t0:>6.2f}  "
                  f"{fv(rl0)}  {fv(rl1)}  {a0:>6.1f}  {a1:>6.1f}")

        if slack_evs:
            print()
            print(f"Slack events: {len(slack_evs)} runs")
            print(f"  {'phase':<24}  {'t_start':>7}  {'dur_ms':>7}  "
                  f"{'alt0':>6}  {'alt1':>6}  {'rest_l':>7}  "
                  f"{'T_before':>8}  {'T_after':>8}  {'winch_v':>8}")
            print("  " + "-" * 90)
            for e in slack_evs:
                dur_ms = e["duration_s"] * 1000.0
                def fv(v): return f"{v:8.1f}" if not math.isnan(v) else "     nan"
                wv = f"{e['winch_before']:8.3f}" if not math.isnan(e["winch_before"]) else "     nan"
                print(f"  {e['phase']:<24}  {e['t_start']:>7.3f}  {dur_ms:>7.1f}  "
                      f"  {e['alt_start']:>5.1f}  {e['alt_end']:>5.1f}  "
                      f"{e['rest_l_start']:>7.2f}  "
                      f"{fv(e['tension_before'])}  {fv(e['tension_after'])}  {wv}")

        _print_osc_table(osc_rows, dt)
        print()


# ── CLI ────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--test",    default="test_pump_cycle_unified")
    p.add_argument("--bucket",  type=float, default=5.0)
    p.add_argument("--verbose", action="store_true", help="also print full tables to stdout")
    p.add_argument("--out-dir", default=None, help="override output directory for CSV/JSON files")
    args = p.parse_args()

    csv_path = Path(__file__).resolve().parents[1] / "logs" / args.test / "telemetry.csv"
    if not csv_path.exists():
        print(f"No telemetry at {csv_path}")
        sys.exit(1)

    diagnose(csv_path, args.bucket, verbose=args.verbose, out_dir=args.out_dir)
