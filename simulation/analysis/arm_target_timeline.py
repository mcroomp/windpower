#!/usr/bin/env python3
"""
arm_target_timeline.py -- Consolidated arm/target-angle sequencing report.

Builds a simple 1-second timeline from all stack-test artifacts in
simulation/logs/{test_name}/ and answers the specific ordering question:

    "Did arming happen before/after target-angle commands?"

Sources used:
  - telemetry.csv
  - mavlink.jsonl (ATTITUDE, ATTITUDE_TARGET, HEARTBEAT, STATUSTEXT)
  - dataflash.BIN (ATT desired/actual)
  - gcs.log
  - mediator.log
  - arducopter.log

Output:
  1) Ordered key milestones (sim-time where possible)
  2) 1-second bucket table with arm/mode/target/attitude signals
  3) Optional CSV for offline inspection

Examples:
  simulation/.venv/Scripts/python.exe simulation/analysis/arm_target_timeline.py test_lua_flight_ic_passive_sitl
  simulation/.venv/Scripts/python.exe simulation/analysis/arm_target_timeline.py simulation/logs/test_lua_flight_ic_passive_sitl --csv out.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


_SIM_DIR = Path(__file__).resolve().parents[1]
_LOGS_DIR = _SIM_DIR / "logs"

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from flight_log import FlightLog  # noqa: E402


@dataclass
class TextEvent:
    source: str
    line_no: int
    wall_ts: str
    kind: str
    text: str


_RE_WALL = re.compile(r"^(\d{2}:\d{2}:\d{2})\s+")


def _wall_to_s(ts: str) -> float:
    if not ts:
        return float("inf")
    hh, mm, ss = ts.split(":")
    return int(hh) * 3600 + int(mm) * 60 + int(ss)


def _resolve_log_dir(target: str) -> Path:
    p = Path(target)
    if p.exists() and p.is_dir():
        return p
    return _LOGS_DIR / target


def _fmt_opt(v: Optional[float], prec: int = 2) -> str:
    if v is None:
        return "-"
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return "-"
    return f"{v:.{prec}f}"


def _rad_to_deg_opt(v: Optional[float]) -> Optional[float]:
    if v is None:
        return None
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return None
    return math.degrees(v)


def _is_nonzero(v: Optional[float], eps: float = 0.1) -> bool:
    if v is None:
        return False
    if isinstance(v, float) and math.isnan(v):
        return False
    return abs(v) > eps


def _parse_text_log(path: Path, source: str) -> list[TextEvent]:
    if not path.exists():
        return []

    events: list[TextEvent] = []
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    for i, line in enumerate(lines, start=1):
        m = _RE_WALL.match(line)
        wall = m.group(1) if m else ""
        l = line.lower()

        if "seeded set_attitude_target" in l:
            events.append(TextEvent(source, i, wall, "SET_ATT_TARGET_CMD", line.strip()))
        elif "set_target_angle" in l:
            events.append(TextEvent(source, i, wall, "SET_TARGET_ANGLE_CALL", line.strip()))
        elif "sending arm command" in l:
            events.append(TextEvent(source, i, wall, "ARM_CMD_SENT", line.strip()))
        elif "arm command accepted" in l:
            events.append(TextEvent(source, i, wall, "ARM_CMD_ACCEPTED", line.strip()))
        elif "vehicle is armed" in l or "armed=true" in l:
            events.append(TextEvent(source, i, wall, "ARMED_LOG", line.strip()))
        elif "disarming motors" in l or "armed=false" in l:
            events.append(TextEvent(source, i, wall, "DISARMED_LOG", line.strip()))
        elif "rawes ic seed set" in l:
            events.append(TextEvent(source, i, wall, "RAWES_IC_SEED_SET", line.strip()))

    return events


def _first_sim_event(fl: FlightLog, category: str, text_equals: Optional[str] = None) -> Optional[float]:
    for ev in fl.events:
        if ev.category != category:
            continue
        if text_equals is not None and ev.text.strip() != text_equals:
            continue
        return ev.t_sim
    return None


def _first_tgt_time(fl: FlightLog) -> Optional[float]:
    if not fl._tgt_rows:
        return None
    return fl._tgt_rows[0]["t_sim"]


def _first_nonzero_tgt_time(fl: FlightLog, eps_deg: float = 0.1) -> Optional[float]:
    for r in fl._tgt_rows:
        if _is_nonzero(r.get("roll"), eps_deg) or _is_nonzero(r.get("pitch"), eps_deg):
            return r["t_sim"]
    return None


def _first_df_des_nonzero_time(fl: FlightLog, eps_deg: float = 0.1) -> Optional[float]:
    for r in fl._df_att_rows:
        if _is_nonzero(r.get("des_roll"), eps_deg) or _is_nonzero(r.get("des_pitch"), eps_deg):
            return r["t_s"]
    return None


def _first_nonzero_tel_mav_rate_time(fl: FlightLog, eps_rads: float = 0.01) -> Optional[float]:
    for r in fl.tel_rows:
        if (
            _is_nonzero(r.mav_att_target_roll_rate_rads, eps_rads)
            or _is_nonzero(r.mav_att_target_pitch_rate_rads, eps_rads)
            or _is_nonzero(r.mav_att_target_yaw_rate_rads, eps_rads)
        ):
            return r.t_sim
    return None


def _first_nonzero_tel_ap_rate_sp_time(fl: FlightLog, eps_rads: float = 0.01) -> Optional[float]:
    for r in fl.tel_rows:
        if _is_nonzero(r.roll_sp_rads, eps_rads) or _is_nonzero(r.pitch_sp_rads, eps_rads):
            return r.t_sim
    return None


def _first_nonzero_df_pid_tar_time(fl: FlightLog, eps_deg_s: float = 0.1) -> Optional[float]:
    ts: list[float] = []
    for r in fl._df_pidr_rows:
        if _is_nonzero(r.get("tar"), eps_deg_s):
            ts.append(r["t_s"])
            break
    for r in fl._df_pidp_rows:
        if _is_nonzero(r.get("tar"), eps_deg_s):
            ts.append(r["t_s"])
            break
    return min(ts) if ts else None


def _bucket_events_short(bucket) -> str:
    if not bucket.events:
        return ""
    labels: list[str] = []
    for ev in bucket.events:
        if ev.category == "ARM":
            labels.append(ev.text)
        elif ev.category == "MODE":
            labels.append(ev.text)
        elif ev.category == "RAWES":
            labels.append(f"RAWES:{ev.text[:36]}")
        elif ev.category == "EKF" and ev.severity in {"WARN", "ERROR"}:
            labels.append(f"EKF:{ev.text[:30]}")
    # Keep rows readable.
    return " | ".join(labels[:3])


def _print_key_order(fl: FlightLog, text_events: list[TextEvent]) -> None:
    t_armed = _first_sim_event(fl, "ARM", "ARMED")
    t_first_tgt = _first_tgt_time(fl)
    t_first_tgt_nonzero = _first_nonzero_tgt_time(fl)
    t_df_des_nonzero = _first_df_des_nonzero_time(fl)
    t_tel_mav_rate_nonzero = _first_nonzero_tel_mav_rate_time(fl)
    t_tel_ap_rate_nonzero = _first_nonzero_tel_ap_rate_sp_time(fl)
    t_df_pid_tar_nonzero = _first_nonzero_df_pid_tar_time(fl)

    print("\n== Key Order (sim-time) ==")
    print(f"ARMED heartbeat/event         : {_fmt_opt(t_armed, 3)} s")
    print(f"First ATTITUDE_TARGET msg     : {_fmt_opt(t_first_tgt, 3)} s")
    print(f"First non-zero ATT_TARGET r/p : {_fmt_opt(t_first_tgt_nonzero, 3)} s")
    print(f"First DF desired non-zero r/p : {_fmt_opt(t_df_des_nonzero, 3)} s")
    print(f"First non-zero MAV target rate: {_fmt_opt(t_tel_mav_rate_nonzero, 3)} s")
    print(f"First non-zero AP rate setpoint: {_fmt_opt(t_tel_ap_rate_nonzero, 3)} s")
    print(f"First non-zero DF PID Tar r/p : {_fmt_opt(t_df_pid_tar_nonzero, 3)} s")

    if t_armed is not None and t_first_tgt is not None:
        rel = "before" if t_first_tgt < t_armed else "after"
        print(f"Ordering: first ATTITUDE_TARGET is {rel} arming.")
    if t_armed is not None and t_first_tgt_nonzero is not None:
        rel_nz = "before" if t_first_tgt_nonzero < t_armed else "after"
        print(f"Ordering: first non-zero target attitude is {rel_nz} arming.")
    if t_armed is not None and t_tel_mav_rate_nonzero is not None:
        rel_rt = "before" if t_tel_mav_rate_nonzero < t_armed else "after"
        print(f"Ordering: first non-zero MAV target rate is {rel_rt} arming.")
    if t_armed is not None and t_tel_ap_rate_nonzero is not None:
        rel_ap = "before" if t_tel_ap_rate_nonzero < t_armed else "after"
        print(f"Ordering: first non-zero AP rate setpoint is {rel_ap} arming.")

    gcs_cmds = [
        e
        for e in text_events
        if e.kind in {"SET_ATT_TARGET_CMD", "SET_TARGET_ANGLE_CALL", "RAWES_IC_SEED_SET"}
    ]
    arm_cmds = [e for e in text_events if e.kind in {"ARM_CMD_SENT", "ARM_CMD_ACCEPTED", "ARMED_LOG"}]

    print("\n== Text-Log Command Sequence (wall-time order) ==")
    if not (gcs_cmds or arm_cmds):
        print("(no matching text-log command markers found)")
    else:
        merged = sorted(
            gcs_cmds + arm_cmds,
            key=lambda e: (_wall_to_s(e.wall_ts), e.source, e.line_no),
        )
        for ev in merged:
            print(f"{ev.source}:{ev.line_no:5d} [{ev.wall_ts}] {ev.kind}: {ev.text}")


def _print_timeline_1s(fl: FlightLog, bucket_s: float, csv_path: Optional[Path]) -> None:
    buckets = fl.buckets(bucket_s=bucket_s)
    if not buckets:
        print("\nNo telemetry/dataflash buckets available.")
        return

    print("\n== 1s Timeline (consolidated) ==")
    header = (
        "t0..t1  kin  mode/arm    "
        "mav_tgt_r/p   mav_tgt_rr/pr   ap_sp_rr/pr   df_tar_rr/pr   df_act_rr/pr   "
        "mav_att_r/p   df_des_r/p   df_att_r/p   notes"
    )
    print(header)

    # Track latest mode/arm state over time from FlightEvents.
    events_sorted = sorted(fl.events, key=lambda e: e.t_sim)
    eidx = 0
    arm_state = "?"
    mode_state = "?"

    csv_rows = []
    for b in buckets:
        while eidx < len(events_sorted) and events_sorted[eidx].t_sim < b.t_end:
            ev = events_sorted[eidx]
            if ev.category == "ARM":
                arm_state = "ARMED" if ev.text.strip() == "ARMED" else "DISARM"
            elif ev.category == "MODE":
                mode_state = ev.text.replace("mode -> ", "")
            eidx += 1

        mode_arm = f"{mode_state}/{arm_state}"
        kin = "Y" if b.is_kinematic else "N"
        tgt = f"{_fmt_opt(b.tgt_roll_deg,1)}/{_fmt_opt(b.tgt_pitch_deg,1)}"

        # Telemetry-logged desired rate snapshots in this bucket [deg/s].
        tel_in = [r for r in fl.tel_rows if b.t_start <= r.t_sim < b.t_end]
        if tel_in:
            rr = [_rad_to_deg_opt(r.mav_att_target_roll_rate_rads) for r in tel_in]
            pr = [_rad_to_deg_opt(r.mav_att_target_pitch_rate_rads) for r in tel_in]
            ap_rr = [_rad_to_deg_opt(r.roll_sp_rads) for r in tel_in]
            ap_pr = [_rad_to_deg_opt(r.pitch_sp_rads) for r in tel_in]

            rr_v = [v for v in rr if v is not None]
            pr_v = [v for v in pr if v is not None]
            ap_rr_v = [v for v in ap_rr if v is not None]
            ap_pr_v = [v for v in ap_pr if v is not None]

            mav_rr = sum(rr_v) / len(rr_v) if rr_v else None
            mav_pr = sum(pr_v) / len(pr_v) if pr_v else None
            ap_rr_m = sum(ap_rr_v) / len(ap_rr_v) if ap_rr_v else None
            ap_pr_m = sum(ap_pr_v) / len(ap_pr_v) if ap_pr_v else None
        else:
            mav_rr = mav_pr = ap_rr_m = ap_pr_m = None

        mav_rr_pr = f"{_fmt_opt(mav_rr,1)}/{_fmt_opt(mav_pr,1)}"
        ap_rr_pr = f"{_fmt_opt(ap_rr_m,1)}/{_fmt_opt(ap_pr_m,1)}"

        df_tar_rr_pr = f"{_fmt_opt(b.df_pidr_tar,1)}/{_fmt_opt(b.df_pidp_tar,1)}"
        df_act_rr_pr = f"{_fmt_opt(b.df_pidr_act,1)}/{_fmt_opt(b.df_pidp_act,1)}"
        att = f"{_fmt_opt(b.att_roll_deg,1)}/{_fmt_opt(b.att_pitch_deg,1)}"
        dfd = f"{_fmt_opt(b.df_des_roll_deg,1)}/{_fmt_opt(b.df_des_pitch_deg,1)}"
        dfa = f"{_fmt_opt(b.df_roll_deg,1)}/{_fmt_opt(b.df_pitch_deg,1)}"
        notes = _bucket_events_short(b)

        print(
            f"{b.t_start:5.1f}-{b.t_end:5.1f}  {kin:1s}  "
            f"{mode_arm:10.10s}  "
            f"{tgt:11s}  {mav_rr_pr:13s}  {ap_rr_pr:11s}  {df_tar_rr_pr:13s}  {df_act_rr_pr:13s}  "
            f"{att:11s}  {dfd:11s}  {dfa:11s}  {notes}"
        )

        csv_rows.append({
            "t_start_s": round(b.t_start, 3),
            "t_end_s": round(b.t_end, 3),
            "is_kinematic": b.is_kinematic,
            "mode_state": mode_state,
            "arm_state": arm_state,
            "mav_tgt_roll_deg": b.tgt_roll_deg,
            "mav_tgt_pitch_deg": b.tgt_pitch_deg,
            "mav_tgt_roll_rate_deg_s": mav_rr,
            "mav_tgt_pitch_rate_deg_s": mav_pr,
            "ap_roll_sp_deg_s": ap_rr_m,
            "ap_pitch_sp_deg_s": ap_pr_m,
            "df_pidr_tar_deg_s": b.df_pidr_tar,
            "df_pidp_tar_deg_s": b.df_pidp_tar,
            "df_pidr_act_deg_s": b.df_pidr_act,
            "df_pidp_act_deg_s": b.df_pidp_act,
            "mav_att_roll_deg": b.att_roll_deg,
            "mav_att_pitch_deg": b.att_pitch_deg,
            "df_des_roll_deg": b.df_des_roll_deg,
            "df_des_pitch_deg": b.df_des_pitch_deg,
            "df_att_roll_deg": b.df_roll_deg,
            "df_att_pitch_deg": b.df_pitch_deg,
            "notes": notes,
        })

    if csv_path is not None:
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()))
            w.writeheader()
            for row in csv_rows:
                w.writerow(row)
        print(f"\nWrote CSV timeline: {csv_path}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Consolidate logs and report arm vs target-angle order in 1-second buckets."
    )
    ap.add_argument(
        "target",
        help="Test name under simulation/logs/ OR explicit log-dir path",
    )
    ap.add_argument(
        "--bucket",
        type=float,
        default=1.0,
        help="Bucket size in seconds (default: 1.0)",
    )
    ap.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Optional output CSV path for the 1-second timeline",
    )
    args = ap.parse_args()

    log_dir = _resolve_log_dir(args.target)
    if not log_dir.exists():
        print(f"Log directory not found: {log_dir}")
        return 2

    fl = FlightLog.load(log_dir)
    gcs_events = _parse_text_log(log_dir / "gcs.log", "gcs.log")
    med_events = _parse_text_log(log_dir / "mediator.log", "mediator.log")
    sitl_events = _parse_text_log(log_dir / "arducopter.log", "arducopter.log")
    text_events = gcs_events + med_events + sitl_events

    print(f"Log dir: {log_dir}")
    print(
        "Loaded sources: "
        f"telemetry={len(fl.tel_rows)} rows, "
        f"mav_att={len(fl._att_rows)}, mav_tgt={len(fl._tgt_rows)}, "
        f"df_att={len(fl._df_att_rows)}, df_swsh={len(fl._df_swsh_rows)}, "
        f"events={len(fl.events)}, text_events={len(text_events)}"
    )

    _print_key_order(fl, text_events)
    _print_timeline_1s(fl, bucket_s=float(args.bucket), csv_path=args.csv)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
