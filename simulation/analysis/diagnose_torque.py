#!/usr/bin/env python3
"""
diagnose_torque.py — post-run diagnosis for torque stack tests.

Sources used
------------
  events.jsonl  — mediator physics log (psi_dot, throttle, omega_rotor)
  dataflash.BIN — ArduPilot DataFlash (PIDY: I-term + Limit flag; RATE: yaw
                  error; ATT: EKF attitude; IMU: raw gyro)

Failure patterns detected
--------------------------
  FROZEN     — psi_dot stuck at same value for > N seconds
               (SITL lockstep stall or mediator crash)
  SATURATED  — throttle clamped at max while psi_dot is large
               (IMAX too low for current RPM)
  SLOW_CONV  — psi_dot not converged by settle_s
  MOTOR_DEAD — throttle near 0 while psi_dot is large

Usage
-----
    python simulation/analysis/diagnose_torque.py <test_name_or_log_dir> [settle_s [observe_s]]

Examples
    python simulation/analysis/diagnose_torque.py test_wobble[wobble]
    python simulation/analysis/diagnose_torque.py test_wobble[wobble] 80 20
    python simulation/analysis/diagnose_torque.py simulation/logs/test_slow_rpm[slow_vary] 50 30
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

_SIM_DIR  = Path(__file__).resolve().parents[1]
_LOGS_DIR = _SIM_DIR / "logs"

# ── torque_model constants (mirror of torque_model.py) ───────────────────────
GEAR_RATIO  = 80.0 / 44.0   # 1.818
RPM_SCALE   = 105.0          # rad/s at 100 % throttle
OMEGA_NOM   = 28.0           # rad/s nominal rotor speed
THROTTLE_EQ = OMEGA_NOM * GEAR_RATIO / RPM_SCALE   # ~0.485

# ── diagnosis thresholds ─────────────────────────────────────────────────────
FREEZE_REPEAT_S     = 4      # identical value for this many 1 Hz samples → frozen
IMAX_MARGIN         = 0.01   # throttle within this of observed max → probably saturated
MOTOR_DEAD_THROTTLE = 0.05   # throttle below this while psi_dot > threshold → dead
SLOW_CONV_DEG_S     = 15.0   # psi_dot above this at window start → slow convergence


# ── events.jsonl helpers ──────────────────────────────────────────────────────

def _load_events(log_dir: Path) -> list[dict]:
    p = log_dir / "events.jsonl"
    if not p.exists():
        return []
    out = []
    for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return out


def _dynamics_start_tsim(events: list[dict]) -> float | None:
    """Return t_sim of first dynamics_start event."""
    for e in events:
        if e.get("event") == "dynamics_start":
            return float(e["t_sim"])
    return None


def _heartbeats(events: list[dict], t_dyn_start: float) -> list[dict]:
    """Return DYNAMIC heartbeat events with t_dyn = t_sim - t_dyn_start."""
    out = []
    for e in events:
        if e.get("event") == "heartbeat" and e.get("phase") == "DYNAMIC":
            e2 = dict(e)
            e2["t_dyn"] = round(float(e["t_sim"]) - t_dyn_start, 3)
            out.append(e2)
    out.sort(key=lambda x: x["t_dyn"])
    return out


# ── DataFlash helpers ─────────────────────────────────────────────────────────

def _load_dataflash(bin_path: Path, t_dyn_start_s: float) -> dict:
    """
    Parse DataFlash .BIN using pymavlink DFReader.

    Returns a dict of message-type -> list of dicts, each with t_dyn added.
    t_dyn = TimeUS/1e6 - t_dyn_start_s  (both share the SITL boot clock).

    Returns empty dict if DFReader not available or file unreadable.
    """
    try:
        from pymavlink import DFReader as _DFR
    except ImportError:
        return {}

    if not bin_path.exists():
        return {}

    wanted = {"PIDY", "RATE", "ATT", "IMU"}
    result: dict[str, list[dict]] = {k: [] for k in wanted}
    result["_last_t_dyn"] = []   # sentinel: last record timestamp

    try:
        log = _DFR.DFReader_binary(str(bin_path), zero_time_base=False)
    except Exception:
        return {}

    last_t_dyn = None
    while True:
        try:
            msg = log.recv_msg()
        except Exception:
            break
        if msg is None:
            break
        mtype = msg.get_type()
        try:
            d = msg.to_dict()
        except Exception:
            continue
        t_us = d.get("TimeUS", 0)
        t_dyn = round(t_us / 1e6 - t_dyn_start_s, 3)
        last_t_dyn = t_dyn
        if mtype not in wanted:
            continue
        d["t_dyn"] = t_dyn
        result[mtype].append(d)

    if last_t_dyn is not None:
        result["_last_t_dyn"] = [{"t_dyn": last_t_dyn}]

    return result


def _bucket_df(records: list[dict], t_lo: float, t_hi: float) -> list[dict]:
    return [r for r in records if t_lo <= r["t_dyn"] < t_hi]


# ── physics-side failure detectors ───────────────────────────────────────────

def _detect_frozen(hb: list[dict]) -> list[str]:
    issues = []
    if len(hb) < 2:
        return issues
    run_val   = hb[0]["psi_dot_deg_s"]
    run_start = hb[0]["t_dyn"]
    run_len   = 1
    for h in hb[1:]:
        v = h["psi_dot_deg_s"]
        if v == run_val:
            run_len += 1
        else:
            if run_len >= FREEZE_REPEAT_S:
                issues.append(
                    f"FROZEN  psi_dot={run_val:.2f} deg/s stuck for {run_len}s "
                    f"from t_dyn={run_start:.1f}s  (SITL stall or mediator crash)"
                )
            run_val, run_start, run_len = v, h["t_dyn"], 1
    if run_len >= FREEZE_REPEAT_S:
        issues.append(
            f"FROZEN  psi_dot={run_val:.2f} deg/s stuck for {run_len}s "
            f"from t_dyn={run_start:.1f}s  (SITL stall or mediator crash)"
        )
    return issues


def _detect_saturation(hb: list[dict]) -> list[str]:
    if not hb:
        return []
    max_thr = max(h["throttle"] for h in hb)
    ceiling_hits = [
        h for h in hb
        if abs(h["throttle"] - max_thr) < IMAX_MARGIN
        and abs(h["psi_dot_deg_s"]) > SLOW_CONV_DEG_S
    ]
    if len(ceiling_hits) < 3:
        return []
    avg_omega = sum(h.get("omega_rad_s", OMEGA_NOM) for h in ceiling_hits) / len(ceiling_hits)
    i_eq = avg_omega * GEAR_RATIO / RPM_SCALE
    return [
        f"SATURATED  throttle ceiling={max_thr:.4f} ({len(ceiling_hits)} samples) "
        f"while |psi_dot| > {SLOW_CONV_DEG_S:.0f} deg/s.  "
        f"I_eq(omega={avg_omega:.1f})={i_eq:.3f} -> raise IMAX above {i_eq:.3f}"
    ]


def _detect_motor_dead(hb: list[dict]) -> list[str]:
    dead = [
        h for h in hb
        if h["throttle"] < MOTOR_DEAD_THROTTLE
        and abs(h["psi_dot_deg_s"]) > SLOW_CONV_DEG_S
    ]
    if len(dead) < 3:
        return []
    return [
        f"MOTOR_DEAD  throttle < {MOTOR_DEAD_THROTTLE:.2f} for {len(dead)} samples "
        f"(t_dyn={dead[0]['t_dyn']:.1f}-{dead[-1]['t_dyn']:.1f}s) while |psi_dot| large  "
        f"(check H_YAW_TRIM, H_TAIL_TYPE, arming)"
    ]


def _convergence_time(hb: list[dict], target_deg_s: float = 10.0) -> float | None:
    for i, h in enumerate(hb):
        if abs(h["psi_dot_deg_s"]) <= target_deg_s:
            if all(abs(hb[j]["psi_dot_deg_s"]) <= target_deg_s
                   for j in range(i, min(i + 5, len(hb)))):
                return h["t_dyn"]
    return None


# ── DataFlash section ─────────────────────────────────────────────────────────

# ── top-level ─────────────────────────────────────────────────────────────────

def diagnose(log_dir: Path, settle_s: float = 80.0, observe_s: float = 20.0) -> None:
    events = _load_events(log_dir)
    if not events:
        print(f"[ERROR] No events.jsonl found in {log_dir}")
        return

    t_dyn_start = _dynamics_start_tsim(events)
    if t_dyn_start is None:
        print("[ERROR] No dynamics_start event — did DYNAMIC phase start?")
        return

    hb = _heartbeats(events, t_dyn_start)
    if not hb:
        print("[ERROR] No DYNAMIC heartbeat events found.")
        return

    t_dyn_end = hb[-1]["t_dyn"]

    print(f"\n{'='*70}")
    print(f"  TORQUE TEST DIAGNOSIS")
    print(f"  Log dir : {log_dir}")
    print(f"  Window  : settle={settle_s:.0f}s  observe={observe_s:.0f}s")
    print(f"  DYNAMIC : {len(hb)} heartbeats  t_dyn=[{hb[0]['t_dyn']:.1f}, {t_dyn_end:.1f}]")
    print(f"{'='*70}\n")

    # ── Physics stats ─────────────────────────────────────────────────────────
    window = [h for h in hb if settle_s <= h["t_dyn"] <= settle_s + observe_s]
    overall_max  = max(abs(h["psi_dot_deg_s"]) for h in hb)
    window_max   = max((abs(h["psi_dot_deg_s"]) for h in window), default=None)
    window_mean  = (sum(abs(h["psi_dot_deg_s"]) for h in window) / len(window)
                    if window else None)
    thr_end      = hb[-1]["throttle"]
    conv_t       = _convergence_time(hb, 10.0)

    print("[PHYSICS STATS]")
    print(f"  Overall max |psi_dot|  : {overall_max:.2f} deg/s")
    if window_max is not None:
        print(f"  Window  max |psi_dot|  : {window_max:.2f} deg/s  "
              f"(mean {window_mean:.2f})  n={len(window)}")
    else:
        print(f"  Window  [EMPTY] — observation window never reached!")
    print(f"  Throttle at end        : {thr_end:.4f}  (eq~{THROTTLE_EQ:.3f})")
    if conv_t is not None:
        print(f"  Converged <10 deg/s    : t_dyn={conv_t:.1f}s")
    else:
        print(f"  Converged <10 deg/s    : never")

    # ── Issue detection ───────────────────────────────────────────────────────
    issues: list[str] = []
    issues += _detect_frozen(hb)
    issues += _detect_saturation(hb)
    issues += _detect_motor_dead(hb)

    if window_max is None:
        issues.append("TIMEOUT  observation window not reached")
    elif window_max > SLOW_CONV_DEG_S and not issues:
        win = [h for h in hb if settle_s <= h["t_dyn"] <= settle_s + observe_s]
        if win:
            h1 = win[:max(1, len(win)//2)]
            h2 = win[max(1, len(win)//2):]
            m1 = sum(abs(h["psi_dot_deg_s"]) for h in h1) / len(h1)
            m2 = sum(abs(h["psi_dot_deg_s"]) for h in h2) / len(h2)
            trend = "still converging" if m2 < m1 * 0.7 else "not converging"
            issues.append(
                f"SLOW_CONV  {trend} in window: "
                f"first-half mean={m1:.1f} / second-half mean={m2:.1f} deg/s  "
                f"({'increase settle_s' if trend == 'still converging' else 'increase P or IMAX'})"
            )

    print()
    if issues:
        print("[ISSUES]")
        for iss in issues:
            print(f"  ! {iss}")
    else:
        print("[ISSUES]  none detected")

    # ── Unified timeline (physics + DataFlash) ────────────────────────────────
    print()
    bin_path = log_dir / "dataflash.BIN"
    df = _load_dataflash(bin_path, t_dyn_start)
    pidy = df.get("PIDY", [])
    rate = df.get("RATE", [])
    imu  = df.get("IMU",  [])
    has_df = bool(pidy or rate or imu)

    hdr = (f"  {'t_dyn':>11}  {'psi_dot':>9}  {'thr':>6}"
           + (f"  {'I_term':>7}  {'clamp':>5}  {'yaw_err':>7}  {'gyroZ_max':>9}"
              if has_df else "")
           + "")
    sep = "-" * len(hdr)
    label = "[TIMELINE]  10 s buckets" + (" — physics + dataflash (PIDY/RATE/IMU)" if has_df else "")
    print(label)
    print(hdr)
    print(f"  {sep}")

    t = 0.0
    while t < t_dyn_end + 10.0:
        phys = [h for h in hb if t <= h["t_dyn"] < t + 10.0]
        if not phys and not _bucket_df(pidy, t, t + 10.0):
            t += 10.0
            continue

        max_psi  = max((abs(h["psi_dot_deg_s"]) for h in phys), default=float("nan"))
        mean_thr = (sum(h["throttle"] for h in phys) / len(phys)) if phys else float("nan")

        marker = ""
        if t <= settle_s < t + 10.0:
            marker = "  <-- settle"
        elif t <= settle_s + observe_s < t + 10.0:
            marker = "  <-- obs end"

        row = f"  t={t:4.0f}-{t+10.0:<4.0f}  {max_psi:9.2f}  {mean_thr:6.4f}"

        if has_df:
            pb = _bucket_df(pidy, t, t + 10.0)
            rb = _bucket_df(rate, t, t + 10.0)
            ib = _bucket_df(imu,  t, t + 10.0)

            if pb:
                i_vals   = [r.get("I", r.get("Int", 0.0))               for r in pb]
                lim_vals = [r.get("Limit", r.get("Lim", 0))              for r in pb]
                mean_I   = sum(i_vals) / len(i_vals)
                clamp_pct = 100.0 * sum(1 for v in lim_vals if int(v) & 1) / len(pb)
            else:
                mean_I = clamp_pct = float("nan")

            if rb:
                y_des = [r.get("YDes", r.get("Ydes", 0.0)) for r in rb]
                y_act = [r.get("Y",    0.0)                 for r in rb]
                mean_yerr = sum(abs(d - a) for d, a in zip(y_des, y_act)) / len(rb)
            else:
                mean_yerr = float("nan")

            if ib:
                gz_deg = [r.get("GyrZ", 0.0) * (180.0 / 3.14159265) for r in ib]
                max_gz = max(abs(v) for v in gz_deg)
            else:
                max_gz = float("nan")

            def _fmt(v: float, w: int, d: int) -> str:
                return f"{v:{w}.{d}f}" if v == v else " " * (w - 1) + "-"

            row += (f"  {_fmt(mean_I, 7, 4)}"
                    f"  {_fmt(clamp_pct, 4, 0)}%"
                    f"  {_fmt(mean_yerr, 7, 3)}"
                    f"  {_fmt(max_gz, 9, 2)}")

        print(row + marker)
        t += 10.0

    if not has_df:
        print("  (dataflash not available — pymavlink missing or file absent)")
    print()


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        print("Usage: python diagnose_torque.py <test_name_or_log_dir> [settle_s [observe_s]]")
        sys.exit(1)

    arg = sys.argv[1]
    candidate = Path(arg)
    if not candidate.is_absolute():
        candidate = Path.cwd() / arg
    if not candidate.exists():
        candidate = _LOGS_DIR / arg
    if not candidate.exists():
        print(f"[ERROR] Log directory not found: {arg}")
        print(f"  Tried: {Path.cwd() / arg}")
        print(f"  Tried: {_LOGS_DIR / arg}")
        sys.exit(1)

    settle_s  = float(sys.argv[2]) if len(sys.argv) > 2 else 80.0
    observe_s = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

    diagnose(candidate, settle_s=settle_s, observe_s=observe_s)


if __name__ == "__main__":
    main()
