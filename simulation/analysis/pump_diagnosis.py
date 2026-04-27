"""
pump_diagnosis.py -- Read pump cycle telemetry and diagnose controller decisions.

Columns per time bucket:
  t          simulation time [s]
  phase      pumping phase
  alt        actual altitude [m]
  tgt        target altitude from planner [m]
  alt_err    alt - tgt [m]  (+ = too high, - = too low)
  el         actual elevation angle [deg]
  d_eq       delta: angle(bz_eq, tdir) [deg]  -- what AltHoldCtrl commands
  d_act      delta: angle(body_z_act, tdir) [deg]  -- what hub is doing
  T_thr_need T_thrust required for altitude at current d_eq [N]
  feasible   is T_thr_need achievable by aero?
  T_set      tension PI setpoint [N]
  T_act      actual tether tension [N]
  col_tpi    collective from TensionPI (pre-slew) [rad]
  col_act    actual collective (post-slew) [rad]
  flags      SLACK/FLOOR/SPIKE/BZ_PAST_PERP/ALT_CRASH/T_INFEAS

Usage
-----
    .venv/Scripts/python.exe simulation/analysis/pump_diagnosis.py
    .venv/Scripts/python.exe simulation/analysis/pump_diagnosis.py --test test_pump_cycle --bucket 2
"""

import sys, math, csv, argparse
from pathlib import Path
from collections import defaultdict

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aero import create_aero
import rotor_definition as rd
import json

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
    """Angle between two vectors [degrees]. Returns nan if either is zero."""
    n1 = np.linalg.norm(v1); n2 = np.linalg.norm(v2)
    if n1 < 1e-9 or n2 < 1e-9:
        return float("nan")
    c = float(np.dot(v1, v2)) / (n1 * n2)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _t_thrust_needed(el_deg, delta_deg):
    """T_thrust for altitude maintenance at tilt delta [N]. >0 only if delta in (0,90)."""
    el = math.radians(el_deg)
    d  = math.radians(delta_deg)
    sd = math.sin(d)
    if sd < 1e-6:
        return float("inf")
    if delta_deg >= 90.0:
        return float("nan")   # body_z past perpendicular — altitude maintenance impossible
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


# ── Main diagnostic ────────────────────────────────────────────────────────────

def _phase_transitions(rows):
    """Return list of (t_start, t_end, phase, rest_l_start, rest_l_end, alt_start, alt_end)."""
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
    # last segment
    r = rows[-1]
    events.append((t0, r["t_sim"], cur_phase, rl0, r.get("tether_rest_length", float("nan")),
                   alt0, float(-r.get("pos_z", 0))))
    return events


def _slack_events(rows, tension_window=5):
    """
    Group consecutive slack rows into events.  Each event dict has:
      phase, t_start, t_end, duration_s, n_rows,
      alt_start, alt_end,
      rest_l_start, rest_l_end,
      tension_before  (mean over tension_window rows before),
      tension_after   (mean over tension_window rows after),
      winch_speed_before (mean over window rows before, if column present),
    """
    events = []
    n = len(rows)
    i = 0
    while i < n:
        if rows[i].get("tether_slack", 0) > 0.5:
            j = i
            while j < n and rows[j].get("tether_slack", 0) > 0.5:
                j += 1
            # event: rows[i:j]
            seg = rows[i:j]
            before = rows[max(0, i - tension_window):i]
            after  = rows[j:min(n, j + tension_window)]

            def mean_col(rlist, col):
                vals = [r[col] for r in rlist if col in r and not math.isnan(r.get(col, float("nan")))]
                return float(np.mean(vals)) if vals else float("nan")

            ev = dict(
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
            )
            events.append(ev)
            i = j
        else:
            i += 1
    return events


def diagnose(csv_path, bucket_s=5.0):
    rows = load_telemetry(csv_path)
    if not rows:
        print("No telemetry rows found.")
        return

    buckets = defaultdict(list)
    for r in rows:
        bk = int(r["t_sim"] / bucket_s) * bucket_s
        buckets[bk].append(r)

    print()
    print("pump_diagnosis -- controller decision audit")
    print(f"  Rows={len(rows)}  Bucket={bucket_s}s  "
          f"T_thrust_range=[{_T_THRUST_MIN:.0f},{_T_THRUST_MAX:.0f}]N  mg={MG:.1f}N")
    print()

    hdr = (f"  {'t':>5}  {'phase':<20}  {'alt':>5}  {'tgt':>5}  {'err':>5}  "
           f"{'el':>5}  {'d_eq':>5}  {'d_act':>5}  "
           f"{'T_need':>6}  {'feas':>6}  "
           f"{'T_set':>6}  {'T_act':>6}  "
           f"{'col_tpi':>7}  {'col_act':>7}  {'col_sd':>6}  {'col_pp':>7}  flags")
    print(hdr)
    print("  " + "-" * (len(hdr) + 30))

    for bk in sorted(buckets.keys()):
        grp   = buckets[bk]
        phase = grp[len(grp) // 2].get("phase", "")

        def avg(key):
            vals = [r[key] for r in grp
                    if key in r and not math.isnan(r[key])]
            return float(np.mean(vals)) if vals else float("nan")

        def std(key):
            vals = [r[key] for r in grp
                    if key in r and not math.isnan(r[key])]
            return float(np.std(vals)) if len(vals) > 1 else float("nan")

        def peak_to_peak(key):
            vals = [r[key] for r in grp
                    if key in r and not math.isnan(r[key])]
            return float(np.max(vals) - np.min(vals)) if len(vals) > 1 else float("nan")

        def frac(pred):
            n = sum(1 for r in grp if pred(r))
            return n / max(len(grp), 1)

        pos     = np.array([avg("pos_x"), avg("pos_y"), avg("pos_z")])
        bz_eq   = np.array([avg("bz_eq_x"), avg("bz_eq_y"), avg("bz_eq_z")])
        bz_act  = np.array([avg("r02"), avg("r12"), avg("r22")])
        tdir    = pos / max(np.linalg.norm(pos), 1e-6)

        alt     = float(-pos[2])
        tgt     = avg("gnd_alt_cmd_m")
        alt_err = alt - tgt if not math.isnan(tgt) else float("nan")
        el      = _elevation(pos)
        d_eq    = _angle_deg(bz_eq, tdir)
        d_act   = _angle_deg(bz_act, tdir)

        t_need  = _t_thrust_needed(el, d_eq)
        if math.isnan(t_need):
            feas = "PAST90"
        elif t_need < _T_THRUST_MIN:
            feas = "TOO_LOW"
        elif t_need > _T_THRUST_MAX:
            feas = "TOO_HIGH"
        else:
            feas = "OK"

        t_set    = avg("tension_setpoint")
        t_act    = avg("tether_tension")
        col_tpi  = avg("collective_from_tension_ctrl")
        col_act  = avg("collective_rad")
        col_std  = std("collective_rad")
        col_pp   = peak_to_peak("collective_rad")

        # Per-row event counts (as fraction of bucket)
        slack_f  = frac(lambda r: r.get("tether_slack", 0) > 0.5)
        floor_f  = frac(lambda r: (-r["pos_z"]) < FLOOR_ALT_M if "pos_z" in r else False)
        spike_f  = frac(lambda r: r.get("tether_tension", 0) > BREAK_LOAD_N)

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

        def f(v, fmt):
            return format(v, fmt) if not math.isnan(v) else "  nan"

        print(f"  {bk:>5.0f}  {phase:<20}  {alt:>5.1f}  {f(tgt,'5.1f')}  {f(alt_err,'+5.1f')}  "
              f"{el:>5.1f}  {f(d_eq,'5.1f')}  {f(d_act,'5.1f')}  "
              f"{f(t_need,'6.0f')}  {feas:>6}  "
              f"{f(t_set,'6.0f')}  {f(t_act,'6.0f')}  "
              f"{f(col_tpi,'7.3f')}  {f(col_act,'7.3f')}  {f(col_std,'6.4f')}  {f(col_pp,'7.4f')}  "
              + "  ".join(flags))

    # ── Phase transition timeline ─────────────────────────────────────────────
    transitions = _phase_transitions(rows)
    print()
    print("Phase transition timeline:")
    print(f"  {'phase':<24}  {'t_start':>7}  {'t_end':>7}  {'dur':>6}  "
          f"{'rest_l0':>7}  {'rest_l1':>7}  {'alt0':>6}  {'alt1':>6}")
    print("  " + "-" * 80)
    for t0, t1, ph, rl0, rl1, a0, a1 in transitions:
        def fv(v): return f"{v:7.2f}" if not math.isnan(v) else "    nan"
        print(f"  {ph:<24}  {t0:>7.2f}  {t1:>7.2f}  {t1-t0:>6.2f}  "
              f"{fv(rl0)}  {fv(rl1)}  {a0:>6.1f}  {a1:>6.1f}")

    # ── Slack event detail ────────────────────────────────────────────────────
    slack_events = _slack_events(rows)
    if slack_events:
        print()
        n_total = sum(e["n_rows"] for e in slack_events)
        t_total = sum(e["duration_s"] for e in slack_events)
        print(f"Slack events: {len(slack_events)} runs  "
              f"{n_total} rows  {t_total:.3f}s total")
        print(f"  {'phase':<24}  {'t_start':>7}  {'dur_ms':>7}  "
              f"{'alt0':>6}  {'alt1':>6}  {'rest_l':>7}  "
              f"{'T_before':>8}  {'T_after':>8}  {'winch_v':>8}")
        print("  " + "-" * 90)
        for e in slack_events:
            dur_ms = e["duration_s"] * 1000.0
            def fv(v): return f"{v:8.1f}" if not math.isnan(v) else "     nan"
            wv = f"{e['winch_before']:8.3f}" if not math.isnan(e["winch_before"]) else "     nan"
            print(f"  {e['phase']:<24}  {e['t_start']:>7.3f}  {dur_ms:>7.1f}  "
                  f"  {e['alt_start']:>5.1f}  {e['alt_end']:>5.1f}  "
                  f"{e['rest_l_start']:>7.2f}  "
                  f"{fv(e['tension_before'])}  {fv(e['tension_after'])}  {wv}")
    else:
        print()
        print("No slack events.")

    # ── Oscillation analysis ──────────────────────────────────────────────────
    _oscillation_analysis(rows)

    print()


# ── Oscillation analysis ───────────────────────────────────────────────────────

# Signals to analyse and their display labels
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

_MIN_SEGMENT_ROWS = 40   # ~2 s at 20 Hz — minimum for meaningful FFT


def _dominant_freq(sig, dt):
    """Return (freq_hz, amplitude_pp) of the dominant FFT component after detrending."""
    if len(sig) < _MIN_SEGMENT_ROWS:
        return float("nan"), float("nan")
    x = np.asarray(sig, dtype=float)
    x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))  # detrend
    if np.std(x) < 1e-10:
        return 0.0, 0.0
    n     = len(x)
    freqs = np.fft.rfftfreq(n, d=dt)
    amps  = np.abs(np.fft.rfft(x)) * 2.0 / n
    # ignore DC (index 0)
    idx = int(np.argmax(amps[1:])) + 1
    return float(freqs[idx]), float(amps[idx] * 2.0)   # amplitude in peak-to-peak


def _oscillation_analysis(rows):
    if not rows:
        return

    # Estimate telemetry dt from first two rows
    t0 = rows[0].get("t_sim", 0.0)
    t1 = rows[1].get("t_sim", 0.0) if len(rows) > 1 else t0 + 0.05
    dt = max(t1 - t0, 1e-6)

    # Split into contiguous phase segments
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

    print()
    print("Oscillation analysis  (dominant frequency per phase segment)")
    print(f"  dt={dt*1000:.1f}ms  fs={1/dt:.0f}Hz  min_seg={_MIN_SEGMENT_ROWS} rows")
    print(f"  {'phase':<24}  {'dur_s':>5}  {'signal':<18}  "
          f"{'f_dom_Hz':>8}  {'pp_amp':>8}  {'rms':>8}")
    print("  " + "-" * 80)

    for phase, seg in segments:
        dur = seg[-1].get("t_sim", 0.0) - seg[0].get("t_sim", 0.0)
        if len(seg) < _MIN_SEGMENT_ROWS:
            continue

        first = True
        for col, label in _OSC_SIGNALS:
            vals = [r[col] for r in seg if col in r and not math.isnan(r.get(col, float("nan")))]
            if len(vals) < _MIN_SEGMENT_ROWS:
                continue
            # flip pos_z so positive = altitude
            if col == "pos_z":
                vals = [-v for v in vals]
            f_hz, pp = _dominant_freq(vals, dt)
            arr  = np.asarray(vals, dtype=float)
            arr -= np.mean(arr)
            rms  = float(np.sqrt(np.mean(arr ** 2)))
            ph_label = phase if first else ""
            dur_s    = f"{dur:5.1f}" if first else "     "
            first    = False
            print(f"  {ph_label:<24}  {dur_s}  {label:<18}  "
                  f"{f_hz:>8.3f}  {pp:>8.4f}  {rms:>8.4f}")
        print()


# ── CLI ────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--test",   default="test_pump_cycle")
    p.add_argument("--bucket", type=float, default=5.0)
    args = p.parse_args()

    csv_path = Path(__file__).resolve().parents[1] / "logs" / args.test / "telemetry.csv"
    if not csv_path.exists():
        print(f"No telemetry at {csv_path}")
        sys.exit(1)

    diagnose(csv_path, args.bucket)
