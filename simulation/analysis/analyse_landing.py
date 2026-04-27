"""
analyse_landing.py -- Diagnose LandingGroundController / LandingApController runs.

Per-bucket table columns:
  t         simulation time [s]
  phase     reel_in | descent | final_drop
  alt       actual altitude [m]
  vz_act    actual NED descent rate [m/s]  (+ = down)
  vz_tgt    VZ setpoint [m/s]  (from phase; 0=reel_in, v_land=descent)
  rest_l    winch rest length [m]
  w_spd     winch speed [m/s]  (- = reel in)
  xi_act    actual tilt from horizontal [deg]  (xi=90 => horizontal disk)
  xi_eq     bz_eq tilt [deg]
  T_act     actual tether tension [N]
  col       collective [rad]
  flags     SLACK/SPIKE/FLOOR

Usage
-----
    .venv/Scripts/python.exe simulation/analysis/analyse_landing.py
    .venv/Scripts/python.exe simulation/analysis/analyse_landing.py --test test_landing_lua --bucket 2
"""

import sys, math, csv, argparse
from pathlib import Path
from collections import defaultdict

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

BREAK_LOAD_N  = 620.0
FLOOR_ALT_M   = 1.5


# ── Helpers ────────────────────────────────────────────────────────────────────

def _xi_deg(bz_ned):
    """Elevation angle of body_z from horizontal plane [deg]. xi=90 = vertical disk."""
    bz = np.asarray(bz_ned)
    if np.linalg.norm(bz) < 1e-9:
        return float("nan")
    return math.degrees(math.asin(max(-1.0, min(1.0, float(-bz[2]) / np.linalg.norm(bz)))))


def _f(v, fmt):
    return format(v, fmt) if not math.isnan(v) else " " * len(format(0, fmt))


# ── Load ───────────────────────────────────────────────────────────────────────

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


# ── Phase timeline ─────────────────────────────────────────────────────────────

def _phase_transitions(rows):
    events = []
    if not rows:
        return events
    cur = rows[0].get("phase", "")
    t0 = rows[0]["t_sim"]
    rl0 = rows[0].get("tether_rest_length", float("nan"))
    alt0 = float(-rows[0].get("pos_z", 0))
    for r in rows[1:]:
        ph = r.get("phase", "")
        if ph != cur:
            events.append((t0, r["t_sim"], cur, rl0,
                           r.get("tether_rest_length", float("nan")),
                           alt0, float(-r.get("pos_z", 0))))
            cur = ph
            t0 = r["t_sim"]
            rl0 = r.get("tether_rest_length", float("nan"))
            alt0 = float(-r.get("pos_z", 0))
    r = rows[-1]
    events.append((t0, r["t_sim"], cur, rl0,
                   r.get("tether_rest_length", float("nan")),
                   alt0, float(-r.get("pos_z", 0))))
    return events


# ── Slack events ───────────────────────────────────────────────────────────────

def _slack_events(rows, window=5):
    events = []
    n = len(rows)
    i = 0
    while i < n:
        if rows[i].get("tether_slack", 0) > 0.5:
            j = i
            while j < n and rows[j].get("tether_slack", 0) > 0.5:
                j += 1
            seg    = rows[i:j]
            before = rows[max(0, i - window):i]
            after  = rows[j:min(n, j + window)]

            def mc(rlist, col):
                vals = [r[col] for r in rlist
                        if col in r and not math.isnan(r.get(col, float("nan")))]
                return float(np.mean(vals)) if vals else float("nan")

            events.append(dict(
                phase        = seg[0].get("phase", ""),
                t_start      = seg[0]["t_sim"],
                t_end        = seg[-1]["t_sim"],
                duration_s   = seg[-1]["t_sim"] - seg[0]["t_sim"],
                n_rows       = len(seg),
                alt          = float(-seg[0].get("pos_z", 0)),
                rest_l       = seg[0].get("tether_rest_length", float("nan")),
                T_before     = mc(before, "tether_tension"),
                T_after      = mc(after,  "tether_tension"),
                winch_before = mc(before, "winch_speed_ms"),
                winch_during = mc(seg,    "winch_speed_ms"),
                vz_before    = mc(before, "vel_z"),
            ))
            i = j
        else:
            i += 1
    return events


# ── Tension spikes ─────────────────────────────────────────────────────────────

def _spike_events(rows, threshold=BREAK_LOAD_N, window=5):
    events = []
    n = len(rows)
    i = 0
    while i < n:
        if rows[i].get("tether_tension", 0) > threshold:
            j = i
            while j < n and rows[j].get("tether_tension", 0) > threshold:
                j += 1
            seg = rows[i:j]
            events.append(dict(
                phase    = seg[0].get("phase", ""),
                t_start  = seg[0]["t_sim"],
                t_end    = seg[-1]["t_sim"],
                n_rows   = len(seg),
                alt      = float(-seg[0].get("pos_z", 0)),
                rest_l   = seg[0].get("tether_rest_length", float("nan")),
                T_peak   = max(r.get("tether_tension", 0) for r in seg),
            ))
            i = j
        else:
            i += 1
    return events


# ── Main diagnostic ────────────────────────────────────────────────────────────

def diagnose(csv_path, bucket_s=5.0):
    rows = load_telemetry(csv_path)
    if not rows:
        print("No telemetry rows found.")
        return

    buckets = defaultdict(list)
    for r in rows:
        bk = int(r["t_sim"] / bucket_s) * bucket_s
        buckets[bk].append(r)

    t_end  = rows[-1]["t_sim"]
    n_rows = len(rows)
    n_slack  = sum(1 for r in rows if r.get("tether_slack", 0) > 0.5)
    n_spike  = sum(1 for r in rows if r.get("tether_tension", 0) > BREAK_LOAD_N)
    T_max    = max((r.get("tether_tension", 0) for r in rows), default=0)

    print()
    print("analyse_landing -- landing controller audit")
    print(f"  Rows={n_rows}  t_end={t_end:.1f}s  Bucket={bucket_s}s")
    print(f"  T_max={T_max:.0f}N  slack_rows={n_slack}  spike_rows(>{BREAK_LOAD_N:.0f}N)={n_spike}")
    print()

    hdr = (f"  {'t':>5}  {'phase':<11}  {'alt':>5}  "
           f"{'vz_act':>6}  {'vz_tgt':>6}  "
           f"{'rest_l':>7}  {'w_spd':>6}  "
           f"{'xi_act':>6}  {'xi_eq':>6}  "
           f"{'T_act':>6}  {'col':>7}  flags")
    print(hdr)
    print("  " + "-" * (len(hdr) + 20))

    for bk in sorted(buckets.keys()):
        grp = buckets[bk]
        phase = grp[len(grp) // 2].get("phase", "")

        def avg(key):
            vals = [r[key] for r in grp
                    if key in r and not math.isnan(r.get(key, float("nan")))]
            return float(np.mean(vals)) if vals else float("nan")

        def frac(pred):
            return sum(1 for r in grp if pred(r)) / max(len(grp), 1)

        alt     = float(-avg("pos_z"))
        vz_act  = avg("vel_z")
        rest_l  = avg("tether_rest_length")
        w_spd   = avg("winch_speed_ms")
        t_act   = avg("tether_tension")
        col     = avg("collective_rad")

        bz_act  = np.array([avg("r02"), avg("r12"), avg("r22")])
        bz_eq   = np.array([avg("bz_eq_x"), avg("bz_eq_y"), avg("bz_eq_z")])
        xi_act  = _xi_deg(bz_act)
        xi_eq   = _xi_deg(bz_eq)

        # vz target: 0 in reel_in, v_land (~0.5) in descent, 0 in final_drop
        if "descent" in phase:
            vz_tgt = 0.5
        else:
            vz_tgt = 0.0

        slack_f = frac(lambda r: r.get("tether_slack", 0) > 0.5)
        spike_f = frac(lambda r: r.get("tether_tension", 0) > BREAK_LOAD_N)
        floor_f = frac(lambda r: (-r.get("pos_z", 999)) < FLOOR_ALT_M)

        flags = []
        if slack_f > 0.05: flags.append(f"SLACK={slack_f*100:.0f}%")
        if spike_f > 0.01: flags.append(f"SPIKE={spike_f*100:.0f}%")
        if floor_f > 0.05: flags.append("FLOOR")
        if not math.isnan(t_act) and t_act > 400:
            flags.append(f"HIGH_T={t_act:.0f}N")
        if not math.isnan(w_spd) and w_spd < -0.6:
            flags.append(f"FAST_REEL={w_spd:.2f}")

        print(f"  {bk:>5.0f}  {phase:<11}  {alt:>5.1f}  "
              f"{_f(vz_act,'6.3f')}  {vz_tgt:>6.3f}  "
              f"{_f(rest_l,'7.2f')}  {_f(w_spd,'6.3f')}  "
              f"{_f(xi_act,'6.1f')}  {_f(xi_eq,'6.1f')}  "
              f"{_f(t_act,'6.0f')}  {_f(col,'7.3f')}  "
              + "  ".join(flags))

    # ── Phase timeline ────────────────────────────────────────────────────────
    transitions = _phase_transitions(rows)
    print()
    print("Phase transition timeline:")
    print(f"  {'phase':<12}  {'t_start':>7}  {'t_end':>7}  {'dur':>6}  "
          f"{'rest_l0':>7}  {'rest_l1':>7}  {'alt0':>6}  {'alt1':>6}")
    print("  " + "-" * 72)
    for t0, t1, ph, rl0, rl1, a0, a1 in transitions:
        fv = lambda v: f"{v:7.2f}" if not math.isnan(v) else "    nan"
        print(f"  {ph:<12}  {t0:>7.2f}  {t1:>7.2f}  {t1-t0:>6.2f}  "
              f"{fv(rl0)}  {fv(rl1)}  {a0:>6.1f}  {a1:>6.1f}")

    # ── Slack events ──────────────────────────────────────────────────────────
    slack_events = _slack_events(rows)
    if slack_events:
        print()
        n_evts  = len(slack_events)
        n_total = sum(e["n_rows"] for e in slack_events)
        t_total = sum(e["duration_s"] for e in slack_events)
        print(f"Slack events: {n_evts} runs  {n_total} rows  {t_total:.3f}s total")
        # Show first 20 and last 5 to avoid flooding
        show = slack_events[:20] + (slack_events[-5:] if len(slack_events) > 25 else [])
        print(f"  {'phase':<12}  {'t_start':>7}  {'dur_ms':>7}  "
              f"{'alt':>5}  {'rest_l':>7}  "
              f"{'T_bef':>7}  {'T_aft':>7}  "
              f"{'w_bef':>7}  {'w_dur':>7}  {'vz_bef':>7}")
        print("  " + "-" * 90)
        for e in show:
            fv = lambda v: f"{v:7.1f}" if not math.isnan(v) else "    nan"
            fw = lambda v: f"{v:7.3f}" if not math.isnan(v) else "    nan"
            print(f"  {e['phase']:<12}  {e['t_start']:>7.3f}  {e['duration_s']*1000:>7.1f}  "
                  f"  {e['alt']:>4.1f}  {fv(e['rest_l'])}  "
                  f"{fv(e['T_before'])}  {fv(e['T_after'])}  "
                  f"{fw(e['winch_before'])}  {fw(e['winch_during'])}  {fw(e['vz_before'])}")
        if len(slack_events) > 25:
            print(f"  ... ({len(slack_events) - 25} more events omitted) ...")
    else:
        print()
        print("No slack events.")

    # ── Tension spikes ────────────────────────────────────────────────────────
    spike_events = _spike_events(rows)
    if spike_events:
        print()
        print(f"Tension spike events (>{BREAK_LOAD_N:.0f}N):")
        print(f"  {'phase':<12}  {'t_start':>7}  {'t_end':>7}  "
              f"{'alt':>5}  {'rest_l':>7}  {'T_peak':>8}")
        print("  " + "-" * 60)
        for e in spike_events[:30]:
            fv = lambda v: f"{v:7.2f}" if not math.isnan(v) else "    nan"
            print(f"  {e['phase']:<12}  {e['t_start']:>7.3f}  {e['t_end']:>7.3f}  "
                  f"  {e['alt']:>4.1f}  {fv(e['rest_l'])}  {e['T_peak']:>8.0f}")
    else:
        print()
        print("No tension spikes.")

    # ── Winch summary ─────────────────────────────────────────────────────────
    desc_rows = [r for r in rows if r.get("phase", "") == "descent"]
    if desc_rows:
        w_speeds = [r["winch_speed_ms"] for r in desc_rows
                    if "winch_speed_ms" in r and not math.isnan(r.get("winch_speed_ms", float("nan")))]
        tensions = [r["tether_tension"] for r in desc_rows
                    if "tether_tension" in r and not math.isnan(r.get("tether_tension", float("nan")))]
        vz_vals  = [r["vel_z"] for r in desc_rows
                    if "vel_z" in r and not math.isnan(r.get("vel_z", float("nan")))]
        print()
        print("Descent phase summary:")
        if w_speeds:
            print(f"  winch_speed: mean={np.mean(w_speeds):.3f}  "
                  f"min={min(w_speeds):.3f}  max={max(w_speeds):.3f} m/s")
        if tensions:
            print(f"  tension:     mean={np.mean(tensions):.0f}  "
                  f"min={min(tensions):.0f}  max={max(tensions):.0f} N")
        if vz_vals:
            print(f"  vz_actual:   mean={np.mean(vz_vals):.3f}  "
                  f"min={min(vz_vals):.3f}  max={max(vz_vals):.3f} m/s  (target=0.500)")

    print()


# ── CLI ────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--test",   default="test_landing")
    p.add_argument("--bucket", type=float, default=5.0)
    p.add_argument("csv",      nargs="?", help="direct path to telemetry.csv")
    args = p.parse_args()

    if args.csv:
        csv_path = Path(args.csv)
    else:
        csv_path = Path(__file__).resolve().parents[1] / "logs" / args.test / "telemetry.csv"

    if not csv_path.exists():
        print(f"No telemetry at {csv_path}")
        sys.exit(1)

    diagnose(csv_path, args.bucket)
