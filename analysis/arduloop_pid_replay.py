"""arduloop_pid_replay.py -- replay real SITL PID_TUNING through arduloop's AC_PID port.

Purpose
-------
Isolate whether arduloop's Python port of ArduPilot's rate-PID (`arduloop.pid.AC_PID`)
computes the SAME P/I/D/FF contributions as the real firmware, GIVEN THE EXACT SAME
target/measurement stream the real SITL saw. This is different from
`compare_pumping_sitl_simtest.py`, which diffs two INDEPENDENT simulations (real SITL
physics vs mock-Python physics) that inevitably drift apart over time for reasons
unrelated to the controller math itself (different physics, EKF, timing).

Here there is no independent simulation and no time-alignment ambiguity: for each
real `PID_TUNING` sample (per axis) logged by the SITL run, we feed the REAL
`desired` (target) and `achieved` (measurement) straight into a same-gain
`arduloop.pid.AC_PID` instance and record what our port would have computed.
Any divergence from the real `P`/`I`/`D`/`FF` values is a genuine modeling gap in
arduloop's port (or a params/gain mismatch), not a physics/EKF difference.

Usage:
    python analysis/arduloop_pid_replay.py <log_dir> [--window START_S END_S]

Example:
    python analysis/arduloop_pid_replay.py simulation/logs/test_lua_flight_steady_sitl \\
        --window 60 75
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


from arduloop.params import RateAxisParams
from arduloop.pid import AC_PID

AXIS_NAMES = {1: "roll", 2: "pitch", 3: "yaw"}
AXIS_TO_AP = {1: "RLL", 2: "PIT", 3: "YAW"}


def _load_pid_tuning(mavlink_jsonl: Path) -> dict[int, list[dict]]:
    """Return {axis: [rows sorted by time_boot_ms]} for PID_TUNING messages."""
    rows: dict[int, list[dict]] = {1: [], 2: [], 3: []}
    with mavlink_jsonl.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except Exception:
                continue
            if d.get("_dir") != "rx" or d.get("mavpackettype") != "PID_TUNING":
                continue
            axis = int(d.get("axis", -1))
            if axis not in rows:
                continue
            if d.get("time_boot_ms") is None or d.get("_t_wall") is None:
                continue
            rows[axis].append(d)
    for axis in rows:
        rows[axis].sort(key=lambda d: d["time_boot_ms"])
    return rows


def _startup_run_id(log_dir: Path) -> float | None:
    """Wall-clock epoch at t_sim=0, from events.jsonl's `startup` event (if present)."""
    events = log_dir / "events.jsonl"
    if not events.exists():
        return None
    with events.open(encoding="utf-8") as f:
        for line in f:
            try:
                d = json.loads(line)
            except Exception:
                continue
            if d.get("event") == "startup" and "run_id" in d:
                return float(d["run_id"])
    return None


def replay_axis(axis: int, real_rows: list[dict], params: dict[str, float],
                 run_id: float | None) -> dict:
    """Feed real desired/achieved through arduloop's AC_PID; return arrays for diffing."""
    ap_axis = AXIS_TO_AP[axis]
    rp = RateAxisParams.from_ap_dict(params, ap_axis)
    pid = AC_PID(rp, sample_hz=400.0)

    n = len(real_rows)
    t_rel = np.zeros(n)   # t_sim [s] if run_id known, else boot_ms/1000 relative to first sample
    ard_P = np.zeros(n)
    ard_I = np.zeros(n)
    ard_D = np.zeros(n)
    ard_FF = np.zeros(n)
    real_P = np.zeros(n)
    real_I = np.zeros(n)
    real_D = np.zeros(n)
    real_FF = np.zeros(n)

    t0_ms = real_rows[0]["time_boot_ms"]
    t_prev = None
    for i, d in enumerate(real_rows):
        t_ms = d["time_boot_ms"]
        if run_id is not None:
            t_rel[i] = float(d["_t_wall"]) - run_id
        else:
            t_rel[i] = (t_ms - t0_ms) / 1000.0
        dt = 0.0 if t_prev is None else max(0.0, (t_ms - t_prev) / 1000.0)
        t_prev = t_ms

        target = float(d.get("desired", math.nan))
        measurement = float(d.get("achieved", math.nan))

        if i == 0:
            pid.reset(target=target, measurement=measurement)
        else:
            pid.update_all(target=target, measurement=measurement, dt=dt,
                            sim_time_s=t_ms / 1000.0)

        ard_P[i] = pid.debug.P
        ard_I[i] = pid.debug.I
        ard_D[i] = pid.debug.D
        ard_FF[i] = pid.debug.FF
        real_P[i] = float(d.get("P", math.nan))
        real_I[i] = float(d.get("I", math.nan))
        real_D[i] = float(d.get("D", math.nan))
        real_FF[i] = float(d.get("FF", math.nan))

    return dict(t_rel=t_rel, ard_P=ard_P, ard_I=ard_I, ard_D=ard_D, ard_FF=ard_FF,
                real_P=real_P, real_I=real_I, real_D=real_D, real_FF=real_FF)


def _window_mask(t_rel: np.ndarray, window: tuple[float, float] | None) -> np.ndarray:
    if window is None or len(t_rel) == 0:
        return np.ones(len(t_rel), dtype=bool)
    return (t_rel >= window[0]) & (t_rel <= window[1])


def report(axis: int, res: dict, window: tuple[float, float] | None) -> None:
    mask = _window_mask(res["t_rel"], window)
    n = int(mask.sum())
    name = AXIS_NAMES[axis]
    print(f"\n=== axis={axis} ({name}) : n={n} samples"
          + (f"  window=[{window[0]:.1f},{window[1]:.1f}]s" if window else "") + " ===")
    if n == 0:
        print("  (no samples in window)")
        return

    for term in ("P", "I", "D", "FF"):
        ard = res[f"ard_{term}"][mask]
        real = res[f"real_{term}"][mask]
        diff = ard - real
        mean_abs = float(np.nanmean(np.abs(diff)))
        max_abs = float(np.nanmax(np.abs(diff))) if n else float("nan")
        idx = int(np.nanargmax(np.abs(diff))) if n else -1
        t_at_max = float(res["t_rel"][mask][idx]) if idx >= 0 else float("nan")
        print(f"  {term:>2}  mean|diff|={mean_abs:.6f}  max|diff|={max_abs:.6f}"
              f"  (at t_rel={t_at_max:.2f}s: arduloop={ard[idx]:.6f} real={real[idx]:.6f})")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log_dir", help="SITL test log dir, e.g. simulation/logs/test_lua_flight_steady_sitl")
    ap.add_argument("--window", nargs=2, type=float, metavar=("START_S", "END_S"),
                     help="restrict to t_rel in [START_S, END_S] (relative to first PID_TUNING sample)")
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    mavlink_jsonl = log_dir / "mavlink.jsonl"
    params_json = log_dir / "params.json"
    if not mavlink_jsonl.exists():
        raise SystemExit(f"missing {mavlink_jsonl}")
    if not params_json.exists():
        raise SystemExit(f"missing {params_json}")

    params = json.loads(params_json.read_text())
    pid_rows = _load_pid_tuning(mavlink_jsonl)
    run_id = _startup_run_id(log_dir)

    window = tuple(args.window) if args.window else None

    print(f"arduloop PID replay vs real SITL PID_TUNING : {log_dir}")
    print(f"Time base: {'t_sim (run_id-aligned)' if run_id is not None else 'boot_ms relative to first PID_TUNING sample (no run_id found)'}")
    print(f"Gains: ATC_RAT_RLL_P={params.get('ATC_RAT_RLL_P')}  "
          f"ATC_RAT_PIT_P={params.get('ATC_RAT_PIT_P')}  "
          f"ATC_RAT_YAW_P={params.get('ATC_RAT_YAW_P')}")

    for axis in (1, 2, 3):
        rows = pid_rows[axis]
        if not rows:
            print(f"\n=== axis={axis} ({AXIS_NAMES[axis]}) : no PID_TUNING samples ===")
            continue
        res = replay_axis(axis, rows, params, run_id)
        report(axis, res, window)


if __name__ == "__main__":
    main()
