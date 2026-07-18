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
    python analysis/diagnose_torque.py <test_name_or_log_dir> [settle_s [observe_s]]

Examples
    python analysis/diagnose_torque.py test_yaw_regulation_sitl
    python analysis/diagnose_torque.py test_yaw_regulation_sitl 75 20
    python analysis/diagnose_torque.py simulation/logs/test_yaw_regulation_sitl 75 20
"""
from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

import simulation as _simulation_pkg
_SIM_DIR = Path(_simulation_pkg.__file__).resolve().parent  # simulation/
_LOGS_DIR = _SIM_DIR / "logs"

# ── torque_model constants (imported from torque_model.py — single source of truth) ─
import sys as _sys, os as _os
_sys.path.insert(0, str(_SIM_DIR))
from simulation.torque_model import GEAR_RATIO, RPM_SCALE, OMEGA_ROTOR_NOMINAL as OMEGA_NOM
THROTTLE_EQ = OMEGA_NOM * GEAR_RATIO / RPM_SCALE

# ── diagnosis thresholds ─────────────────────────────────────────────────────
FREEZE_REPEAT_S     = 4      # identical value for this many 1 Hz samples → frozen
FREEZE_MIN_DEG_S    = 5.0    # frozen psi_dot below this is converged station-keeping, not a stall
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


def _load_params(log_dir: Path) -> dict:
    p = log_dir / "params.json"
    if not p.exists():
        return {}
    try:
        return json.loads(p.read_text(encoding="utf-8", errors="replace"))
    except Exception:
        return {}


def _load_mavlink(log_dir: Path) -> list[dict]:
    p = log_dir / "mavlink.jsonl"
    if not p.exists():
        return []
    out: list[dict] = []
    for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return out


def _detect_trim_adaptive_mode(log_dir: Path, mav_msgs: list[dict], params: dict) -> tuple[bool, list[str]]:
    """Best-effort detection of trim-adaptive yaw control usage.

    Evidence sources:
      * Lua startup STATUSTEXT advertising mode=3 (MODE_PASSIVE)
      * H_YAW_TRIM present in params snapshot (always expected on heli)
      * RAWES_YFK override present in params snapshot when explicitly set
    """
    reasons: list[str] = []

    for m in mav_msgs:
        if m.get("mavpackettype") != "STATUSTEXT":
            continue
        txt = str(m.get("text", ""))
        if "RAWES: loaded" in txt and "mode=3" in txt:
            reasons.append("Lua MODE_PASSIVE (mode=3) detected")
            break

    if "H_YAW_TRIM" in params:
        reasons.append(f"H_YAW_TRIM snapshot={float(params.get('H_YAW_TRIM', 0.0)):.3f}")

    if "RAWES_YFK" in params:
        reasons.append(f"RAWES_YFK snapshot={float(params.get('RAWES_YFK', 0.0)):.3f}")

    # Conservative: require PASSIVE evidence before changing recommendations.
    trim_mode = any("MODE_PASSIVE" in r for r in reasons)
    return trim_mode, reasons


def _nearest_interp(x_src: list[float], y_src: list[float], x_q: list[float]) -> list[float]:
    if not x_src or not y_src or len(x_src) != len(y_src):
        return [float("nan")] * len(x_q)
    out: list[float] = []
    for x in x_q:
        j = min(range(len(x_src)), key=lambda k: abs(x_src[k] - x))
        out.append(float(y_src[j]))
    return out


def _pearson(a: list[float], b: list[float]) -> float:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 3:
        return float("nan")
    ax = [p[0] for p in pairs]
    bx = [p[1] for p in pairs]
    ma = sum(ax) / len(ax)
    mb = sum(bx) / len(bx)
    da = [x - ma for x in ax]
    db = [x - mb for x in bx]
    va = sum(x * x for x in da)
    vb = sum(x * x for x in db)
    if va <= 0.0 or vb <= 0.0:
        return float("nan")
    cov = sum(x * y for x, y in zip(da, db))
    return cov / math.sqrt(va * vb)


def _tracking_quality(hb: list[dict], settle_s: float, observe_s: float) -> dict:
    w = [h for h in hb if settle_s <= h["t_dyn"] <= settle_s + observe_s]
    if not w:
        return {"n": 0, "pct_12": 0.0, "pct_30": 0.0, "max_abs": float("nan"), "mean_abs": float("nan")}
    abs_rates = [abs(float(h["psi_dot_deg_s"])) for h in w]
    n = len(abs_rates)
    pct_12 = 100.0 * sum(1 for v in abs_rates if v <= 12.0) / n
    pct_30 = 100.0 * sum(1 for v in abs_rates if v <= 30.0) / n
    return {
        "n": n,
        "pct_12": pct_12,
        "pct_30": pct_30,
        "max_abs": max(abs_rates),
        "mean_abs": sum(abs_rates) / n,
    }


def _find_first_divergence(hb: list[dict], threshold_deg_s: float = 120.0, consecutive: int = 3) -> float | None:
    if len(hb) < consecutive:
        return None
    run = 0
    for h in hb:
        if abs(float(h["psi_dot_deg_s"])) >= threshold_deg_s:
            run += 1
            if run >= consecutive:
                return float(h["t_dyn"]) - float(consecutive - 1)
        else:
            run = 0
    return None


def _actuation_effectiveness(hb: list[dict], t_center: float, span_s: float = 8.0) -> dict:
    """Check whether throttle changes reduce |psi_dot| near divergence.

    If large throttle changes are rarely followed by reductions in |psi_dot|, it
    suggests wrong sign, severe delay, or insufficient authority.
    """
    seg = [h for h in hb if (t_center - span_s) <= h["t_dyn"] <= (t_center + span_s)]
    if len(seg) < 3:
        return {"available": False, "note": "insufficient heartbeat samples near divergence"}

    checks = 0
    improve = 0
    worsen = 0
    for i in range(len(seg) - 1):
        h0 = seg[i]
        h1 = seg[i + 1]
        dthr = float(h1["throttle"]) - float(h0["throttle"])
        a0 = abs(float(h0["psi_dot_deg_s"]))
        a1 = abs(float(h1["psi_dot_deg_s"]))
        if abs(dthr) < 0.2 or a0 < 80.0:
            continue
        checks += 1
        if a1 < a0 - 20.0:
            improve += 1
        elif a1 > a0 + 20.0:
            worsen += 1

    if checks == 0:
        return {"available": False, "note": "no strong actuation transitions near divergence"}

    improve_ratio = 100.0 * improve / checks
    worsen_ratio = 100.0 * worsen / checks
    verdict = "effective"
    if improve_ratio < 35.0 and worsen_ratio > 35.0:
        verdict = "likely wrong-sign or ineffective"
    elif improve_ratio < 35.0:
        verdict = "weak authority or delayed response"

    return {
        "available": True,
        "checks": checks,
        "improve_ratio": improve_ratio,
        "worsen_ratio": worsen_ratio,
        "verdict": verdict,
    }


def _rate_imu_sign_check(df: dict, t_center: float, span_s: float = 8.0) -> dict:
    rate = df.get("RATE", [])
    imu = df.get("IMU", [])
    if not rate or not imu:
        return {"available": False, "note": "RATE/IMU records unavailable"}

    t_lo = t_center - span_s
    t_hi = t_center + span_s
    rb = [r for r in rate if t_lo <= float(r.get("t_dyn", 0.0)) <= t_hi]
    ib = [r for r in imu if t_lo <= float(r.get("t_dyn", 0.0)) <= t_hi]
    if not rb or not ib:
        return {"available": False, "note": "no RATE/IMU samples in divergence window"}

    rt = [float(r["t_dyn"]) for r in rb]
    ry = [float(r.get("Y", 0.0)) for r in rb]
    it = [float(r["t_dyn"]) for r in ib]
    ig = [float(r.get("GyrZ", 0.0)) * (180.0 / math.pi) for r in ib]
    ig_at_r = _nearest_interp(it, ig, rt)
    corr = _pearson(ry, ig_at_r)

    verdict = "consistent"
    if math.isfinite(corr) and corr < -0.5:
        verdict = "possible sign inversion"
    elif math.isfinite(corr) and corr < 0.2:
        verdict = "weak consistency"

    return {
        "available": True,
        "corr": corr,
        "verdict": verdict,
    }


def _mode_handoff_context(mav_msgs: list[dict], t_dyn_start: float, t_center: float, span_s: float = 8.0) -> list[str]:
    out: list[str] = []
    t_abs = t_dyn_start + t_center
    t_lo = t_abs - span_s
    t_hi = t_abs + span_s
    for m in mav_msgs:
        if m.get("mavpackettype") != "STATUSTEXT":
            continue
        tb_ms = m.get("time_boot_ms")
        if tb_ms is None:
            continue
        t = float(tb_ms) / 1000.0
        if not (t_lo <= t <= t_hi):
            continue
        txt = str(m.get("text", "")).strip()
        if not txt:
            continue
        # Keep only ownership/mode-relevant lines.
        tags = ("RAWES", "Mode", "GUIDED", "ACRO", "Arming", "disarm", "EKF", "prearm", "Arm")
        if any(tag in txt for tag in tags):
            out.append(f"t={t - t_dyn_start:+.1f}s: {txt}")
    return out[:12]


def _estimator_check(hb: list[dict], df: dict) -> dict:
    """Check whether AP's observed gyro/rate behavior is consistent with physics.

    Returns a dictionary with summary stats and a plain-language verdict.
    """
    imu = df.get("IMU", [])
    rate = df.get("RATE", [])
    if not imu and not rate:
        return {
            "available": False,
            "verdict": "No IMU/RATE DataFlash records available for estimator cross-check.",
        }

    t_hb = [float(h["t_dyn"]) for h in hb]
    phys = [float(h["psi_dot_deg_s"]) for h in hb]

    imu_t = [float(r["t_dyn"]) for r in imu]
    imu_gz_deg = [float(r.get("GyrZ", 0.0)) * (180.0 / math.pi) for r in imu]
    rate_t = [float(r["t_dyn"]) for r in rate]
    rate_y = [float(r.get("Y", 0.0)) for r in rate]

    best = {
        "corr_imu": float("nan"),
        "corr_rate": float("nan"),
        "lag_s": 0.0,
        "sign": 1.0,
        "med_abs_err_imu": float("nan"),
    }

    # Allow small timing offset and sign convention differences when comparing
    # mediator physics vs DataFlash signals.
    lags = [x * 0.25 for x in range(-12, 13)]  # -3s .. +3s
    for lag in lags:
        t_q = [t + lag for t in t_hb]
        imu_interp = _nearest_interp(imu_t, imu_gz_deg, t_q) if imu else [float("nan")] * len(t_q)
        rate_interp = _nearest_interp(rate_t, rate_y, t_q) if rate else [float("nan")] * len(t_q)
        for sign in (1.0, -1.0):
            imu_signed = [sign * v if math.isfinite(v) else v for v in imu_interp]
            rate_signed = [sign * v if math.isfinite(v) else v for v in rate_interp]
            corr_imu = _pearson(phys, imu_signed)
            corr_rate = _pearson(phys, rate_signed)

            if not math.isfinite(corr_imu):
                continue
            if not math.isfinite(best["corr_imu"]) or abs(corr_imu) > abs(best["corr_imu"]):
                pairs_imu = [(p, i) for p, i in zip(phys, imu_signed) if math.isfinite(p) and math.isfinite(i)]
                med_abs_err_imu = float("nan")
                if pairs_imu:
                    errs = sorted(abs(p - i) for p, i in pairs_imu)
                    med_abs_err_imu = errs[len(errs) // 2]
                best = {
                    "corr_imu": corr_imu,
                    "corr_rate": corr_rate,
                    "lag_s": lag,
                    "sign": sign,
                    "med_abs_err_imu": med_abs_err_imu,
                }

    corr_imu = float(best["corr_imu"])
    corr_rate = float(best["corr_rate"])
    med_abs_err_imu = float(best["med_abs_err_imu"])

    # Heuristic verdicts for practical debugging.
    if math.isfinite(corr_imu) and abs(corr_imu) > 0.6:
        verdict = "Estimator/gyro follows physics trend (after lag/sign alignment); issue is likely real control/plant instability, not EKF confusion."
    elif math.isfinite(corr_imu) and abs(corr_imu) < 0.2:
        verdict = "Very low physics-vs-IMU correlation even after lag/sign alignment; possible estimator/sign/frame mismatch worth deeper EKF review."
    else:
        verdict = "Estimator check inconclusive; need focused ATT/NKF timeline or higher-rate logging."

    return {
        "available": True,
        "corr_imu": corr_imu,
        "corr_rate": corr_rate,
        "lag_s": float(best["lag_s"]),
        "sign": float(best["sign"]),
        "med_abs_err_imu": med_abs_err_imu,
        "verdict": verdict,
    }


# ── physics-side failure detectors ───────────────────────────────────────────

def _detect_frozen(hb: list[dict]) -> list[str]:
    """Detect a genuine SITL lockstep stall / mediator crash.

    A real stall stops the mediator producing new frames: sim time `t_sim`
    stops advancing and psi_dot freezes at whatever (typically non-converged)
    value it last held. Two signatures distinguish that from healthy flight:

      * t_sim does NOT advance across the frozen samples (the harness keeps
        emitting 1 Hz heartbeats stamped with the stale sim time); and
      * psi_dot is frozen ABOVE the converged station-keeping band.

    Healthy station-keeping parks psi_dot near zero, and 2-decimal rounding
    makes several consecutive samples read bit-identical (e.g. 0.50 deg/s) even
    though the raw signal is jittering and t_sim keeps advancing. That is not a
    stall, so we require BOTH a stalled clock and a non-converged magnitude.
    """
    issues = []
    if len(hb) < 2:
        return issues

    run_val   = hb[0]["psi_dot_deg_s"]
    run_start = hb[0]["t_dyn"]
    run_t0    = hb[0]["t_dyn"]
    run_len   = 1
    prev_t    = hb[0]["t_dyn"]

    def _flush(val: float, start: float, t0: float, t1: float, length: int) -> None:
        clock_stalled = (t1 - t0) < FREEZE_REPEAT_S * 0.5   # sim time barely moved
        non_converged = abs(val) > FREEZE_MIN_DEG_S
        if length >= FREEZE_REPEAT_S and clock_stalled and non_converged:
            issues.append(
                f"FROZEN  psi_dot={val:.2f} deg/s stuck with stalled sim clock "
                f"for {length} samples from t_dyn={start:.1f}s  (SITL stall or mediator crash)"
            )

    for h in hb[1:]:
        v = h["psi_dot_deg_s"]
        if v == run_val:
            run_len += 1
        else:
            _flush(run_val, run_start, run_t0, prev_t, run_len)
            run_val, run_start, run_t0, run_len = v, h["t_dyn"], h["t_dyn"], 1
        prev_t = h["t_dyn"]
    _flush(run_val, run_start, run_t0, prev_t, run_len)
    return issues


def _detect_saturation(hb: list[dict], trim_adaptive: bool) -> list[str]:
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
    if trim_adaptive:
        return [
            f"SATURATED  throttle ceiling={max_thr:.4f} ({len(ceiling_hits)} samples) "
            f"while |psi_dot| > {SLOW_CONV_DEG_S:.0f} deg/s.  "
            f"Trim-adaptive path active: inspect trim adaptation rate/limits and sign, "
            f"not just ATC_RAT_YAW_IMAX. (I_eq~{i_eq:.3f})"
        ]
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

    # ── mode/context detection ────────────────────────────────────────────────
    params = _load_params(log_dir)
    mav_msgs = _load_mavlink(log_dir)
    trim_adaptive, trim_reasons = _detect_trim_adaptive_mode(log_dir, mav_msgs, params)

    print("[CONTROL CONTEXT]")
    print(f"  Trim-adaptive mode       : {'yes' if trim_adaptive else 'no/unknown'}")
    if trim_reasons:
        for r in trim_reasons:
            print(f"    - {r}")
    else:
        print("    - no explicit PASSIVE/trim evidence found in logs")

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

    tq = _tracking_quality(hb, settle_s, observe_s)
    print("[TRACKING QUALITY]")
    print(f"  Window samples          : {tq['n']}")
    print(f"  In-band <=12 deg/s      : {tq['pct_12']:.1f}%")
    print(f"  In-band <=30 deg/s      : {tq['pct_30']:.1f}%")
    if tq["n"] > 0:
        print(f"  Window mean/max |psi|   : {tq['mean_abs']:.2f} / {tq['max_abs']:.2f} deg/s")

    # ── Issue detection ───────────────────────────────────────────────────────
    issues: list[str] = []
    issues += _detect_frozen(hb)
    issues += _detect_saturation(hb, trim_adaptive)
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

    est = _estimator_check(hb, df)
    print("[ESTIMATOR CHECK]")
    if not est.get("available", False):
        print(f"  {est['verdict']}")
    else:
        corr_imu = est.get("corr_imu", float("nan"))
        corr_rate = est.get("corr_rate", float("nan"))
        med_abs_err_imu = est.get("med_abs_err_imu", float("nan"))
        lag_s = est.get("lag_s", float("nan"))
        sign = est.get("sign", float("nan"))
        if math.isfinite(corr_imu):
            print(f"  Corr(physics psi_dot, IMU GyrZ): {corr_imu:.3f}")
        if math.isfinite(corr_rate):
            print(f"  Corr(physics psi_dot, RATE.Y)  : {corr_rate:.3f}")
        if math.isfinite(lag_s):
            sign_txt = "+" if sign >= 0 else "-"
            print(f"  Best align (lag/sign)          : {lag_s:+.2f}s / {sign_txt}")
        if math.isfinite(med_abs_err_imu):
            print(f"  Median |physics-IMU| error      : {med_abs_err_imu:.2f} deg/s")
        print(f"  Verdict                         : {est['verdict']}")
    print()

    # ── First divergence and immediate precursors ────────────────────────────
    div_t = _find_first_divergence(hb)
    print("[DIVERGENCE ROOT-CAUSE HINTS]")
    if div_t is None:
        print("  No persistent divergence found (|psi_dot| >= 120 deg/s for 3+ samples).")
    else:
        print(f"  First persistent divergence     : t_dyn={div_t:.1f}s")

        eff = _actuation_effectiveness(hb, div_t)
        if eff.get("available", False):
            print(
                "  Actuation effectiveness         : "
                f"{eff['improve_ratio']:.1f}% improve / {eff['worsen_ratio']:.1f}% worsen "
                f"across {eff['checks']} strong transitions ({eff['verdict']})"
            )
        else:
            print(f"  Actuation effectiveness         : {eff.get('note', 'n/a')}")

        rs = _rate_imu_sign_check(df, div_t)
        if rs.get("available", False):
            c = rs.get("corr", float("nan"))
            if math.isfinite(c):
                print(f"  RATE.Y vs IMU.GyrZ correlation  : {c:.3f} ({rs['verdict']})")
            else:
                print(f"  RATE.Y vs IMU.GyrZ correlation  : n/a ({rs['verdict']})")
        else:
            print(f"  RATE.Y vs IMU.GyrZ correlation  : {rs.get('note', 'n/a')}")

        handoff = _mode_handoff_context(mav_msgs, t_dyn_start, div_t)
        if handoff:
            print("  Mode/ownership context near divergence:")
            for line in handoff:
                print(f"    - {line}")
        else:
            print("  Mode/ownership context          : no relevant STATUSTEXT near divergence")
    print()

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
        # Log dirs are sanitized (pytest '[param]' brackets -> underscores) by
        # stack_utils.sanitize_log_name; try the sanitized form of the name.
        _safe = re.sub(r"[\[\]() /\\]+", "_", arg).strip("_")
        _sanitized = _LOGS_DIR / _safe
        if _sanitized.exists():
            candidate = _sanitized
    if not candidate.exists():
        print(f"[ERROR] Log directory not found: {arg}")
        print(f"  Tried: {Path.cwd() / arg}")
        print(f"  Tried: {_LOGS_DIR / arg}")
        print(f"  Tried: {_LOGS_DIR / re.sub(r'[\\[\\]() /\\\\]+', '_', arg).strip('_')}")
        sys.exit(1)

    settle_s  = float(sys.argv[2]) if len(sys.argv) > 2 else 80.0
    observe_s = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

    diagnose(candidate, settle_s=settle_s, observe_s=observe_s)


if __name__ == "__main__":
    main()
