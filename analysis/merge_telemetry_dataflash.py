"""merge_telemetry_dataflash.py -- merge telemetry.csv with dataflash.BIN.

This is a reusable analysis utility, not a one-off diagnostic.

It aligns ArduPilot DataFlash onto telemetry.csv using lockstep frame count
reconstructed from DataFlash RCIN.C11/C12.
The merged CSV keeps telemetry rows as the primary timeline and adds
interpolated DataFlash channels, plus optional MAVLink ATTITUDE/ATTITUDE_TARGET
columns when mavlink.jsonl is available in the same log directory.

Usage
-----
  python analysis/merge_telemetry_dataflash.py \
      simulation/logs/test_lua_flight_steady_sitl/telemetry.csv \
      --out merged.csv

  # You may also pass the explicit dataflash path if it is elsewhere.
  python analysis/merge_telemetry_dataflash.py \
      simulation/logs/test_lua_flight_steady_sitl/telemetry.csv \
      simulation/logs/test_lua_flight_steady_sitl/dataflash.BIN

Frame encoding expected in RCIN:
    C11 = floor(frame_count / 1000) + 1000
    C12 = (frame_count % 1000) + 1000
    frame_count = (C11 - 1000) * 1000 + (C12 - 1000)
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np


import simulation as _simulation_pkg
_SIM_DIR = Path(_simulation_pkg.__file__).resolve().parent  # simulation/

from analysis.flight_log import FlightLog  # noqa: E402
from simulation.telemetry_csv import COLUMNS, read_csv  # noqa: E402


DEFAULT_OUT = Path("simulation/logs/merged_telemetry_dataflash.csv")

CHANNEL_GROUPS: dict[str, list[str]] = {
    "att": [
        "df_t_s",
        "df_att_roll_deg", "df_att_pitch_deg", "df_att_yaw_deg",
        "df_att_des_roll_deg", "df_att_des_pitch_deg", "df_att_des_yaw_deg",
    ],
    "swsh": [
        "df_t_s",
        "df_cyc_roll", "df_cyc_pitch", "df_collective_norm_cmd",
    ],
    "pidr": [
        "df_t_s",
        "df_rate_roll_deg_s", "df_rate_des_roll_deg_s",
        "df_pidr_p", "df_pidr_i", "df_pidr_d", "df_pidr_ff",
    ],
    "pidp": [
        "df_t_s",
        "df_rate_pitch_deg_s", "df_rate_des_pitch_deg_s",
        "df_pidp_p", "df_pidp_i", "df_pidp_d", "df_pidp_ff",
    ],
    "servo": [
        "df_t_s",
        "df_servo_s1_us", "df_servo_s2_us", "df_servo_s3_us", "df_servo4_us",
    ],
    "mavlink": [
        "mav_att_roll_deg", "mav_att_pitch_deg", "mav_att_yaw_deg",
        "mav_att_rollspeed_deg_s", "mav_att_pitchspeed_deg_s", "mav_att_yawspeed_deg_s",
        "mav_tgt_roll_deg", "mav_tgt_pitch_deg", "mav_tgt_yaw_deg", "mav_tgt_thrust",
    ],
    "frame": [
        "df_t_s",
        "df_rcin_c11",
        "df_rcin_c12",
        "tel_frame_count",
        "df_frame_count",
        "frame_err",
        "tel_frame_mod1000",
        "df_frame_mod1000",
        "frame_mod_err",
    ],
}


def _to_float(value: object) -> float:
    try:
        return float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return float("nan")


def _interp(src_t: np.ndarray, src_v: np.ndarray, tq: np.ndarray) -> np.ndarray:
    mask = np.isfinite(src_t) & np.isfinite(src_v)
    if np.sum(mask) < 2:
        return np.full_like(tq, np.nan, dtype=float)
    x = src_t[mask]
    y = src_v[mask]
    order = np.argsort(x)
    return np.interp(tq, x[order], y[order], left=np.nan, right=np.nan)


def _interp_nearest(src_t: np.ndarray, src_v: np.ndarray, tq: np.ndarray) -> np.ndarray:
    mask = np.isfinite(src_t) & np.isfinite(src_v)
    if np.sum(mask) < 1:
        return np.full_like(tq, np.nan, dtype=float)
    x = src_t[mask]
    y = src_v[mask]
    order = np.argsort(x)
    x = x[order]
    y = y[order]
    out = np.full_like(tq, np.nan, dtype=float)
    idx = np.searchsorted(x, tq)
    for i, j in enumerate(idx):
        if not np.isfinite(tq[i]):
            continue
        if j <= 0:
            out[i] = y[0]
        elif j >= len(x):
            out[i] = y[-1]
        else:
            left = j - 1
            right = j
            out[i] = y[left] if abs(tq[i] - x[left]) <= abs(tq[i] - x[right]) else y[right]
    return out


def _mod_err(actual: np.ndarray, expected: np.ndarray, mod: float) -> np.ndarray:
    return ((actual - expected + (0.5 * mod)) % mod) - (0.5 * mod)


def _load_df_rcin_frame_channels(df_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    try:
        from pymavlink import DFReader
    except ImportError:
        empty = np.asarray([], dtype=float)
        return empty, empty, empty, empty

    t_s: list[float] = []
    c11: list[float] = []
    c12: list[float] = []
    frame_abs: list[float] = []
    log = DFReader.DFReader_binary(str(df_path))
    msg = log.recv_msg()
    while msg is not None:
        if msg.get_type() == "RCIN":
            time_us = getattr(msg, "TimeUS", None)
            ch11 = getattr(msg, "C11", None)
            ch12 = getattr(msg, "C12", None)
            if time_us is not None and ch11 is not None and ch12 is not None:
                f11 = float(ch11)
                f12 = float(ch12)
                f_abs = (f11 - 1000.0) * 1000.0 + (f12 - 1000.0)
                t_s.append(float(time_us) / 1e6)
                c11.append(f11)
                c12.append(f12)
                frame_abs.append(f_abs)
        msg = log.recv_msg()
    return (
        np.asarray(t_s, dtype=float),
        np.asarray(c11, dtype=float),
        np.asarray(c12, dtype=float),
        np.asarray(frame_abs, dtype=float),
    )


def _load_df_rcou_channels(df_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    try:
        from pymavlink import DFReader
    except ImportError:
        empty = np.asarray([], dtype=float)
        return empty, empty, empty, empty, empty

    t_s: list[float] = []
    c1: list[float] = []
    c2: list[float] = []
    c3: list[float] = []
    c4: list[float] = []
    log = DFReader.DFReader_binary(str(df_path))
    msg = log.recv_msg()
    while msg is not None:
        if msg.get_type() == "RCOU":
            time_us = getattr(msg, "TimeUS", None)
            v1 = getattr(msg, "C1", None)
            v2 = getattr(msg, "C2", None)
            v3 = getattr(msg, "C3", None)
            v4 = getattr(msg, "C4", None)
            if time_us is not None and v1 is not None and v2 is not None and v3 is not None and v4 is not None:
                t_s.append(float(time_us) / 1e6)
                c1.append(float(v1))
                c2.append(float(v2))
                c3.append(float(v3))
                c4.append(float(v4))
        msg = log.recv_msg()
    return (
        np.asarray(t_s, dtype=float),
        np.asarray(c1, dtype=float),
        np.asarray(c2, dtype=float),
        np.asarray(c3, dtype=float),
        np.asarray(c4, dtype=float),
    )


def _servo_match_stats(
    tel_vals: np.ndarray,
    df_vals: np.ndarray,
    tol_us: float,
) -> tuple[int, int, float, float, float]:
    diff = df_vals - tel_vals
    valid = np.isfinite(diff)
    valid_count = int(np.sum(valid))
    if valid_count == 0:
        return 0, 0, 0.0, float("nan"), float("nan")
    abs_diff = np.abs(diff[valid])
    pass_count = int(np.sum(abs_diff <= tol_us))
    pass_ratio = float(pass_count) / float(valid_count)
    p95 = float(np.percentile(abs_diff, 95))
    max_err = float(np.max(abs_diff))
    return pass_count, valid_count, pass_ratio, p95, max_err


def _estimate_rate_hz(t: np.ndarray) -> float:
    """Estimate source sample rate from monotonic timestamps.

    Returns 0.0 when the rate cannot be estimated.
    """
    if t.size < 2:
        return 0.0
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0.0)]
    if dt.size == 0:
        return 0.0
    return float(1.0 / np.median(dt))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("telemetry_csv", type=Path, help="Path to telemetry.csv")
    parser.add_argument("dataflash_bin", type=Path, nargs="?", help="Path to dataflash.BIN (defaults to same log dir)")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="Output merged CSV path")
    parser.add_argument("--no-mavlink", action="store_true", help="Do not add MAVLink ATTITUDE/ATTITUDE_TARGET columns")
    parser.add_argument(
        "--groups",
        nargs="+",
        choices=sorted(CHANNEL_GROUPS.keys()),
        default=sorted(CHANNEL_GROUPS.keys()),
        help="Channel groups to include in the merged CSV",
    )
    parser.add_argument("--list-groups", action="store_true", help="Print available channel groups and exit")
    parser.add_argument("--window-start", type=float, default=None, help="Optional telemetry window start [s]")
    parser.add_argument("--window-end", type=float, default=None, help="Optional telemetry window end [s]")
    parser.add_argument("--frame-tol", type=float, default=3.0, help="Allowed frame-mod error for alignment checks")
    parser.add_argument("--fail-on-frame-misaligned", action="store_true",
                        help="Exit non-zero when frame alignment exceeds tolerance")
    parser.add_argument("--frame-pass-ratio", type=float, default=0.98,
                        help="Required pass ratio for --fail-on-frame-misaligned")
    parser.add_argument("--servo-tol-us", type=float, default=25.0,
                        help="Allowed absolute PWM difference [us] for servo match checks")
    parser.add_argument("--fail-on-servo-misaligned", action="store_true",
                        help="Exit non-zero when servo match pass ratio is below threshold")
    parser.add_argument("--servo-pass-ratio", type=float, default=0.98,
                        help="Required pass ratio for --fail-on-servo-misaligned")
    parser.add_argument("--interp-min-hz", type=float, default=350.0,
                        help="Use linear interpolation only for sources sampled at or above this rate; otherwise use nearest")
    args = parser.parse_args()

    if args.list_groups:
        for name in sorted(CHANNEL_GROUPS):
            print(f"{name}: {', '.join(CHANNEL_GROUPS[name])}")
        return 0

    selected_groups = [g for g in args.groups if g in CHANNEL_GROUPS]
    if not selected_groups:
        raise RuntimeError("no channel groups selected")

    tel_path = args.telemetry_csv
    if not tel_path.exists():
        raise FileNotFoundError(f"telemetry not found: {tel_path}")

    log_dir = tel_path.parent
    df_path = args.dataflash_bin or (log_dir / "dataflash.BIN")
    if not df_path.exists():
        raise FileNotFoundError(f"dataflash not found: {df_path}")

    tel_rows = read_csv(tel_path)
    if not tel_rows:
        raise RuntimeError(f"no telemetry rows in {tel_path}")

    if args.window_start is not None or args.window_end is not None:
        t_start = -float("inf") if args.window_start is None else float(args.window_start)
        t_end = float("inf") if args.window_end is None else float(args.window_end)
        tel_rows = [r for r in tel_rows if t_start <= float(r.t_sim) <= t_end]
        if not tel_rows:
            raise RuntimeError("telemetry window produced no rows")

    fl = FlightLog.load(log_dir)
    if not fl._df_att_rows and not fl._df_swsh_rows and not fl._df_pidr_rows and not fl._df_pidp_rows:
        raise RuntimeError(f"no usable dataflash rows found in {df_path}")

    df_att_t = np.asarray([float(r["t_s"]) for r in fl._df_att_rows], dtype=float)
    df_swsh_t = np.asarray([float(r["t_s"]) for r in fl._df_swsh_rows], dtype=float)
    df_pidr_t = np.asarray([float(r["t_s"]) for r in fl._df_pidr_rows], dtype=float)
    df_pidp_t = np.asarray([float(r["t_s"]) for r in fl._df_pidp_rows], dtype=float)
    df_rcou_t, df_rcou_c1, df_rcou_c2, df_rcou_c3, df_rcou_c4 = _load_df_rcou_channels(df_path)

    df_fields: dict[str, np.ndarray] = {}
    if fl._df_swsh_rows:
        df_fields["cyc_roll"] = np.asarray([float(r["rcyc"]) for r in fl._df_swsh_rows], dtype=float)
        df_fields["cyc_pitch"] = np.asarray([float(r["pcyc"]) for r in fl._df_swsh_rows], dtype=float)
        df_fields["collective_norm_cmd"] = np.asarray([float(r["col"]) for r in fl._df_swsh_rows], dtype=float)
    if fl._df_pidr_rows:
        df_fields["rate_des_roll_deg_s"] = np.asarray([float(r["tar"]) for r in fl._df_pidr_rows], dtype=float)
        df_fields["rate_roll_deg_s"] = np.asarray([float(r["act"]) for r in fl._df_pidr_rows], dtype=float)
        df_fields["pidr_p"] = np.asarray([float(r["p"]) for r in fl._df_pidr_rows], dtype=float)
        df_fields["pidr_i"] = np.asarray([float(r["i"]) for r in fl._df_pidr_rows], dtype=float)
        df_fields["pidr_d"] = np.asarray([float(r["d"]) for r in fl._df_pidr_rows], dtype=float)
        df_fields["pidr_ff"] = np.asarray([float(r["ff"]) for r in fl._df_pidr_rows], dtype=float)
    if fl._df_pidp_rows:
        df_fields["rate_des_pitch_deg_s"] = np.asarray([float(r["tar"]) for r in fl._df_pidp_rows], dtype=float)
        df_fields["rate_pitch_deg_s"] = np.asarray([float(r["act"]) for r in fl._df_pidp_rows], dtype=float)
        df_fields["pidp_p"] = np.asarray([float(r["p"]) for r in fl._df_pidp_rows], dtype=float)
        df_fields["pidp_i"] = np.asarray([float(r["i"]) for r in fl._df_pidp_rows], dtype=float)
        df_fields["pidp_d"] = np.asarray([float(r["d"]) for r in fl._df_pidp_rows], dtype=float)
        df_fields["pidp_ff"] = np.asarray([float(r["ff"]) for r in fl._df_pidp_rows], dtype=float)
    if df_rcou_t.size:
        df_fields["servo_s1_us"] = df_rcou_c1
        df_fields["servo_s2_us"] = df_rcou_c2
        df_fields["servo_s3_us"] = df_rcou_c3
        df_fields["servo4_us"] = df_rcou_c4

    families: dict[str, tuple[np.ndarray, dict[str, np.ndarray]]] = {}
    if fl._df_att_rows:
        families["att"] = (df_att_t, {
            "att_roll_deg": np.asarray([float(r["roll"]) for r in fl._df_att_rows], dtype=float),
            "att_pitch_deg": np.asarray([float(r["pitch"]) for r in fl._df_att_rows], dtype=float),
            "att_yaw_deg": np.asarray([float(r["yaw"]) for r in fl._df_att_rows], dtype=float),
            "att_des_roll_deg": np.asarray([float(r["des_roll"]) for r in fl._df_att_rows], dtype=float),
            "att_des_pitch_deg": np.asarray([float(r["des_pitch"]) for r in fl._df_att_rows], dtype=float),
            "att_des_yaw_deg": np.asarray([float(r["des_yaw"]) for r in fl._df_att_rows], dtype=float),
        })
    if fl._df_swsh_rows:
        families["swsh"] = (df_swsh_t, {
            "cyc_roll": np.asarray([float(r["rcyc"]) for r in fl._df_swsh_rows], dtype=float),
            "cyc_pitch": np.asarray([float(r["pcyc"]) for r in fl._df_swsh_rows], dtype=float),
            "collective_norm_cmd": np.asarray([float(r["col"]) for r in fl._df_swsh_rows], dtype=float),
        })
    if fl._df_pidr_rows:
        families["pidr"] = (df_pidr_t, {
            "rate_roll_deg_s": np.asarray([float(r["act"]) for r in fl._df_pidr_rows], dtype=float),
            "rate_des_roll_deg_s": np.asarray([float(r["tar"]) for r in fl._df_pidr_rows], dtype=float),
            "pidr_p": np.asarray([float(r["p"]) for r in fl._df_pidr_rows], dtype=float),
            "pidr_i": np.asarray([float(r["i"]) for r in fl._df_pidr_rows], dtype=float),
            "pidr_d": np.asarray([float(r["d"]) for r in fl._df_pidr_rows], dtype=float),
            "pidr_ff": np.asarray([float(r["ff"]) for r in fl._df_pidr_rows], dtype=float),
        })
    if fl._df_pidp_rows:
        families["pidp"] = (df_pidp_t, {
            "rate_pitch_deg_s": np.asarray([float(r["act"]) for r in fl._df_pidp_rows], dtype=float),
            "rate_des_pitch_deg_s": np.asarray([float(r["tar"]) for r in fl._df_pidp_rows], dtype=float),
            "pidp_p": np.asarray([float(r["p"]) for r in fl._df_pidp_rows], dtype=float),
            "pidp_i": np.asarray([float(r["i"]) for r in fl._df_pidp_rows], dtype=float),
            "pidp_d": np.asarray([float(r["d"]) for r in fl._df_pidp_rows], dtype=float),
            "pidp_ff": np.asarray([float(r["ff"]) for r in fl._df_pidp_rows], dtype=float),
        })
    if df_rcou_t.size:
        families["servo"] = (df_rcou_t, {
            "servo_s1_us": df_rcou_c1,
            "servo_s2_us": df_rcou_c2,
            "servo_s3_us": df_rcou_c3,
            "servo4_us": df_rcou_c4,
        })

    tel_t = np.asarray([float(r.t_sim) for r in tel_rows], dtype=float)
    if not (tel_rows and hasattr(tel_rows[0], "frame_count")):
        raise RuntimeError("telemetry.csv missing frame_count; frame-exclusive merge requires it")
    tel_frame_count = np.asarray([float(getattr(r, "frame_count", 0.0)) for r in tel_rows], dtype=float)

    rcin_t, rcin_c11, rcin_c12, rcin_frame = _load_df_rcin_frame_channels(df_path)
    if rcin_t.size == 0:
        raise RuntimeError("dataflash.BIN missing RCIN C11/C12 frame channels; frame-exclusive merge requires them")

    print("Using dynamic RC11/C12 frame matching (time-varying frame offset)")

    # Build a time-varying frame correction from RCIN samples.
    # frame_delta(t) maps telemetry frame space into raw RC frame space.
    tel_frame_at_rcin = _interp(tel_t, tel_frame_count, rcin_t)
    frame_delta_rcin = rcin_frame - tel_frame_at_rcin
    frame_delta_tel = _interp(rcin_t, frame_delta_rcin, tel_t)
    tel_frame_in_rc_space = tel_frame_count + frame_delta_tel
    d_med = float(np.nanmedian(frame_delta_rcin)) if frame_delta_rcin.size else 0.0
    d_p95 = float(np.nanpercentile(np.abs(frame_delta_rcin - d_med), 95)) if frame_delta_rcin.size else 0.0
    print(
        "Applying time-varying RC frame correction: "
        f"median_delta={d_med:.2f} frames, p95_dev={d_p95:.2f}"
    )

    # Derive DataFlash timestamp for each telemetry row via dynamic frame-space mapping.
    df_t_aligned = _interp(rcin_frame, rcin_t, tel_frame_in_rc_space)

    # Build per-family frame axes from DataFlash time -> raw RC frame mapping.
    family_frames: dict[str, np.ndarray] = {}
    family_rates_hz: dict[str, float] = {}
    for family_name, (src_t, _) in families.items():
        family_frames[family_name] = _interp(rcin_t, rcin_frame, src_t)
        family_rates_hz[family_name] = _estimate_rate_hz(src_t)

    if family_rates_hz:
        print("Family sample rates (Hz): " + ", ".join(
            f"{name}={rate:.2f}" for name, rate in sorted(family_rates_hz.items())
        ))

    # Frame diagnostics on telemetry timeline.
    df_frame_count = _interp(rcin_t, rcin_frame, df_t_aligned)
    df_frame_count_aligned = df_frame_count - frame_delta_tel
    c11_aligned = np.floor(df_frame_count_aligned / 1000.0) + 1000.0
    c12_aligned = np.mod(df_frame_count_aligned, 1000.0) + 1000.0
    tel_frame_mod = np.mod(tel_frame_count, 1000.0)
    df_frame_mod = np.mod(df_frame_count_aligned, 1000.0)
    frame_err = df_frame_count_aligned - tel_frame_count
    frame_mod_err = _mod_err(df_frame_mod, tel_frame_mod, 1000.0)

    valid = np.isfinite(frame_err)
    tol = float(args.frame_tol)
    pass_mask = valid & (np.abs(frame_err) <= tol)
    valid_count = int(np.sum(valid))
    pass_count = int(np.sum(pass_mask))
    pass_ratio = (float(pass_count) / float(valid_count)) if valid_count else 0.0
    p95 = float(np.percentile(np.abs(frame_err[valid]), 95)) if valid_count else float("nan")
    max_err = float(np.max(np.abs(frame_err[valid]))) if valid_count else float("nan")
    print(
        "Frame alignment (absolute frame count): "
        f"pass={pass_count}/{valid_count} ({pass_ratio * 100.0:.2f}%), "
        f"p95_abs_err={p95:.2f}, max_abs_err={max_err:.2f}, tol={tol:.2f}"
    )
    if args.fail_on_frame_misaligned and pass_ratio < float(args.frame_pass_ratio):
        print(
            f"ERROR: frame alignment ratio {pass_ratio:.4f} below required "
            f"{float(args.frame_pass_ratio):.4f}"
        )
        return 2

    # Servo verification: compare telemetry servo outputs against DataFlash RCOU
    # channels after frame-based alignment.
    servo_checks = [
        ("s1", "servo_s1_us", "df_servo_s1_us"),
        ("s2", "servo_s2_us", "df_servo_s2_us"),
        ("s3", "servo_s3_us", "df_servo_s3_us"),
        ("s4", "servo4_us", "df_servo4_us"),
    ]
    servo_stats: list[tuple[str, int, int, float, float, float]] = []
    for label, tel_field, df_field in servo_checks:
        tel_vals = np.asarray([float(getattr(r, tel_field, float("nan"))) for r in tel_rows], dtype=float)
        if "servo" in family_frames and "servo" in families:
            _servo_frame = family_frames["servo"]
            _servo_src = families["servo"][1].get(df_field.replace("df_", ""), np.asarray([], dtype=float))
            _servo_rate = family_rates_hz.get("servo", 0.0)
            if _servo_rate >= float(args.interp_min_hz):
                df_vals = _interp(_servo_frame, _servo_src, tel_frame_in_rc_space)
            else:
                df_vals = _interp_nearest(_servo_frame, _servo_src, tel_frame_in_rc_space)
        else:
            df_vals = np.full_like(tel_frame_in_rc_space, np.nan, dtype=float)
        pass_n, valid_n, ratio, p95_us, max_us = _servo_match_stats(
            tel_vals, df_vals, float(args.servo_tol_us)
        )
        servo_stats.append((label, pass_n, valid_n, ratio, p95_us, max_us))
        if valid_n:
            print(
                f"Servo match {label}: pass={pass_n}/{valid_n} ({ratio * 100.0:.2f}%), "
                f"p95_abs_err={p95_us:.2f}us, max_abs_err={max_us:.2f}us, "
                f"tol={float(args.servo_tol_us):.2f}us"
            )

    if args.fail_on_servo_misaligned:
        required = float(args.servo_pass_ratio)
        for label, _pass_n, valid_n, ratio, _p95_us, _max_us in servo_stats:
            if valid_n and ratio < required:
                print(
                    f"ERROR: servo {label} pass ratio {ratio:.4f} below required {required:.4f}"
                )
                return 3

    frame_diag: dict[str, np.ndarray] = {
        "df_rcin_c11": c11_aligned,
        "df_rcin_c12": c12_aligned,
        "tel_frame_count": tel_frame_count,
        "df_frame_count": df_frame_count_aligned,
        "frame_err": frame_err,
        "tel_frame_mod": tel_frame_mod,
        "df_frame_mod": df_frame_mod,
        "frame_mod_err": frame_mod_err,
        "df_t_s": df_t_aligned,
    }

    # Build output rows on telemetry time base.
    out_fields = list(COLUMNS)
    df_out_fields: list[str] = []
    for group in selected_groups:
        for field in CHANNEL_GROUPS[group]:
            if field not in df_out_fields:
                df_out_fields.append(field)
    mav_fields = [] if args.no_mavlink or "mavlink" not in selected_groups else CHANNEL_GROUPS["mavlink"]

    tel_dicts = [{name: getattr(r, name) for name in COLUMNS} for r in tel_rows]
    def _df_interp(group: str, field: str) -> np.ndarray:
        if group not in families:
            return np.full_like(tel_frame_in_rc_space, np.nan, dtype=float)
        _, src_fields = families[group]
        src_frame = family_frames.get(group)
        if src_frame is None:
            return np.full_like(tel_frame_in_rc_space, np.nan, dtype=float)
        if field not in src_fields:
            return np.full_like(tel_frame_in_rc_space, np.nan, dtype=float)
        if family_rates_hz.get(group, 0.0) >= float(args.interp_min_hz):
            return _interp(src_frame, src_fields[field], tel_frame_in_rc_space)
        return _interp_nearest(src_frame, src_fields[field], tel_frame_in_rc_space)

    df_interp_cache = {field: np.full_like(tel_frame_count, np.nan, dtype=float) for field in df_out_fields}
    for group in selected_groups:
        for field in CHANNEL_GROUPS[group]:
            if field == "df_t_s":
                continue
            if group == "mavlink":
                continue
            source_field = field.replace("df_", "")
            if field.startswith("df_att_") and "att" in families:
                if field == "df_att_roll_deg":
                    df_interp_cache[field] = _df_interp("att", "att_roll_deg")
                elif field == "df_att_pitch_deg":
                    df_interp_cache[field] = _df_interp("att", "att_pitch_deg")
                elif field == "df_att_yaw_deg":
                    df_interp_cache[field] = _df_interp("att", "att_yaw_deg")
                elif field == "df_att_des_roll_deg":
                    df_interp_cache[field] = _df_interp("att", "att_des_roll_deg")
                elif field == "df_att_des_pitch_deg":
                    df_interp_cache[field] = _df_interp("att", "att_des_pitch_deg")
                elif field == "df_att_des_yaw_deg":
                    df_interp_cache[field] = _df_interp("att", "att_des_yaw_deg")
            elif field in ("df_cyc_roll", "df_cyc_pitch", "df_collective_norm_cmd"):
                if field == "df_cyc_roll":
                    df_interp_cache[field] = _df_interp("swsh", "cyc_roll")
                elif field == "df_cyc_pitch":
                    df_interp_cache[field] = _df_interp("swsh", "cyc_pitch")
                else:
                    df_interp_cache[field] = _df_interp("swsh", "collective_norm_cmd")
            elif field.startswith("df_rate_roll") or field.startswith("df_pidr_"):
                if field == "df_rate_roll_deg_s":
                    df_interp_cache[field] = _df_interp("pidr", "rate_roll_deg_s")
                elif field == "df_rate_des_roll_deg_s":
                    df_interp_cache[field] = _df_interp("pidr", "rate_des_roll_deg_s")
                else:
                    df_interp_cache[field] = _df_interp("pidr", field.replace("df_", ""))
            elif field.startswith("df_rate_pitch") or field.startswith("df_pidp_"):
                if field == "df_rate_pitch_deg_s":
                    df_interp_cache[field] = _df_interp("pidp", "rate_pitch_deg_s")
                elif field == "df_rate_des_pitch_deg_s":
                    df_interp_cache[field] = _df_interp("pidp", "rate_des_pitch_deg_s")
                else:
                    df_interp_cache[field] = _df_interp("pidp", field.replace("df_", ""))
            elif field in ("df_servo_s1_us", "df_servo_s2_us", "df_servo_s3_us", "df_servo4_us"):
                df_interp_cache[field] = _df_interp("servo", field.replace("df_", ""))

    mav_cache: dict[str, np.ndarray] = {}
    if not args.no_mavlink and "mavlink" in selected_groups and fl._att_rows:
        mav_t = np.asarray([float(r["t_sim"]) for r in fl._att_rows], dtype=float)
        mav_cache["mav_att_roll_deg"] = _interp(mav_t, np.asarray([float(r["roll"]) for r in fl._att_rows], dtype=float), tel_t)
        mav_cache["mav_att_pitch_deg"] = _interp(mav_t, np.asarray([float(r["pitch"]) for r in fl._att_rows], dtype=float), tel_t)
        mav_cache["mav_att_yaw_deg"] = _interp(mav_t, np.asarray([float(r["yaw"]) for r in fl._att_rows], dtype=float), tel_t)
        mav_cache["mav_att_rollspeed_deg_s"] = _interp(mav_t, np.asarray([float(r["rollspeed"]) for r in fl._att_rows], dtype=float), tel_t)
        mav_cache["mav_att_pitchspeed_deg_s"] = _interp(mav_t, np.asarray([float(r["pitchspeed"]) for r in fl._att_rows], dtype=float), tel_t)
        mav_cache["mav_att_yawspeed_deg_s"] = _interp(mav_t, np.asarray([float(r["yawspeed"]) for r in fl._att_rows], dtype=float), tel_t)
    if not args.no_mavlink and "mavlink" in selected_groups and fl._tgt_rows:
        tgt_t = np.asarray([float(r["t_sim"]) for r in fl._tgt_rows], dtype=float)
        mav_cache["mav_tgt_roll_deg"] = _interp(tgt_t, np.asarray([float(r["roll"]) for r in fl._tgt_rows], dtype=float), tel_t)
        mav_cache["mav_tgt_pitch_deg"] = _interp(tgt_t, np.asarray([float(r["pitch"]) for r in fl._tgt_rows], dtype=float), tel_t)
        mav_cache["mav_tgt_yaw_deg"] = _interp(tgt_t, np.asarray([float(r["yaw"]) for r in fl._tgt_rows], dtype=float), tel_t)
        mav_cache["mav_tgt_thrust"] = _interp(tgt_t, np.asarray([float(r["thrust"]) for r in fl._tgt_rows], dtype=float), tel_t)

    merged_rows: list[dict[str, object]] = []
    for idx, tel_row in enumerate(tel_dicts):
        out_row: dict[str, object] = dict(tel_row)
        out_row["df_t_s"] = float(frame_diag["df_t_s"][idx])
        for field in df_out_fields:
            if field == "df_t_s":
                continue
            out_row[field] = float(df_interp_cache[field][idx]) if field in df_interp_cache else float("nan")
        if not args.no_mavlink:
            for field in mav_fields:
                out_row[field] = float(mav_cache[field][idx]) if field in mav_cache else float("nan")
        out_row["df_rcin_c11"] = float(frame_diag["df_rcin_c11"][idx])
        out_row["df_rcin_c12"] = float(frame_diag["df_rcin_c12"][idx])
        out_row["tel_frame_count"] = float(frame_diag["tel_frame_count"][idx])
        out_row["df_frame_count"] = float(frame_diag["df_frame_count"][idx])
        out_row["frame_err"] = float(frame_diag["frame_err"][idx])
        out_row["tel_frame_mod1000"] = float(frame_diag["tel_frame_mod"][idx])
        out_row["df_frame_mod1000"] = float(frame_diag["df_frame_mod"][idx])
        out_row["frame_mod_err"] = float(frame_diag["frame_mod_err"][idx])
        merged_rows.append(out_row)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(COLUMNS) + df_out_fields + mav_fields
    with args.out.open("w", newline="", encoding="ascii") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for row in merged_rows:
            writer.writerow({k: row.get(k, "") for k in fieldnames})

    print(f"Wrote merged CSV: {args.out}")
    print(f"Rows: {len(merged_rows)}")
    print("Alignment mode: RC11/C12 dynamic")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())