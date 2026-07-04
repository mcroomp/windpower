#!/usr/bin/env python3
"""
analyze_ang_error.py — reconstruct ArduPilot's attitude-error decomposition
from a stack-test DataFlash log so you can see WHY the yaw loop stops opposing
the hub spin.

Background
----------
ArduCopter's GUIDED angle controller holds a full quaternion target, but inside
AC_AttitudeControl::attitude_controller_run_quat() the error is split into:

  1. a THRUST-VECTOR error  (roll/pitch of the disk axis), and
  2. a HEADING error        (rotation about the disk axis -> the yaw motor).

Two gain-dependent clamps can make the commanded yaw rate FOLLOW the measured
spin instead of opposing it (PIDY Tar == Act, frozen integrator, idle motor):

  * HEADING CLAMP  (thrust_heading_rotation_angles):
        if |attitude_error.z| > heading_error_max:
            _attitude_target = attitude_body * thrust_corr * heading_corr
    -> the target is REWRITTEN onto the (spinning) body, so DesYaw tracks Yaw.
    heading_error_max = MIN(inv_sqrt_controller(1/ATC_RAT_YAW_P,
                                                ATC_ANG_YAW_P, accel/2), 45 deg)
    With a tiny ATC_RAT_YAW_P that window is small and the target chases the spin.

  * THRUST-ERROR BYPASS  (attitude_controller_run_quat):
        if thrust_error_angle > 2 * 30 deg = 60 deg:
            ang_vel_body.z = gyro.z          # command yaw rate == measured spin

This tool reconstructs both quantities per ATT sample (target & body quaternions
from the logged Euler angles, decomposed exactly as ArduPilot does), flags when
each clamp engages, and pairs them with PIDY (Tar/Act/I) so you can see the
"target follows spin" lock directly.

Sources
-------
  dataflash.BIN  — ATT (Des/actual Euler), PIDY (yaw rate PID), RATE (yaw rate),
                   PARM (ATC_RAT_YAW_P / ATC_ANG_YAW_P / ATC_ACC_Y_MAX)
  events.jsonl   — dynamics_start (to express time as t_dyn)

Usage
-----
    python simulation/analysis/analyze_ang_error.py <test_name_or_log_dir>
                 [settle_s [observe_s]]
                 [--samples]            # dump raw per-sample rows in the window
                 [--ratp P] [--angp P] [--accy DEGSS]   # override gains

Examples
    python simulation/analysis/analyze_ang_error.py test_yaw_regulation_sitl 75 20
    python simulation/analysis/analyze_ang_error.py test_yaw_regulation_sitl 75 20 --samples
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

_SIM_DIR = Path(__file__).resolve().parents[1]
_LOGS_DIR = _SIM_DIR / "logs"

# ── ArduPilot constants (mirror AC_AttitudeControl.h) ─────────────────────────
THRUST_ERR_ANGLE_RAD = math.radians(30.0)      # AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD
THRUST_BYPASS_RAD = THRUST_ERR_ANGLE_RAD * 2.0  # > this -> ang_vel.z = gyro.z (60 deg)
YAW_MAX_ERROR_RAD = math.radians(45.0)          # AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD
ACCEL_Y_MIN_RADSS = math.radians(10.0)          # AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS
ACCEL_Y_MAX_RADSS = math.radians(120.0)         # AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS

# Parameter defaults (used only if PARM is absent from the log)
DEFAULT_RAT_YAW_P = 0.18
DEFAULT_ANG_YAW_P = 4.5
DEFAULT_ACC_Y_DEGSS = 270.0


# ── quaternion helpers (match AP_Math/quaternion.cpp conventions) ─────────────
# Quaternion stored [w, x, y, z]; q rotates a body-frame vector into NED (q * v).

def _quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX (321) Euler -> quaternion, matching Quaternion::from_euler()."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a (x) b."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _quat_inverse(q: np.ndarray) -> np.ndarray:
    """Conjugate (unit quaternion inverse)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate body-frame vector v into NED via q (q * v)."""
    qv = np.array([0.0, v[0], v[1], v[2]])
    out = _quat_mul(_quat_mul(q, qv), _quat_inverse(q))
    return out[1:]


def _quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Unit axis + angle -> quaternion."""
    s = math.sin(angle * 0.5)
    return np.array([math.cos(angle * 0.5), axis[0] * s, axis[1] * s, axis[2] * s])


def _quat_to_axis_angle(q: np.ndarray) -> np.ndarray:
    """Return axis * angle (angle wrapped to [-pi, pi]), matching to_axis_angle()."""
    l = math.sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    v = np.array([q[1], q[2], q[3]])
    if l < 1e-12:
        return np.zeros(3)
    ang = _wrap_pi(2.0 * math.atan2(l, q[0]))
    return (v / l) * ang


def _disk_axis(q: np.ndarray) -> np.ndarray:
    """Disk axis (thrust vector, body [0,0,-1]) expressed in NED."""
    return _quat_rotate(q, np.array([0.0, 0.0, -1.0]))


def _disk_axis_err_deg(q_a: np.ndarray, q_b: np.ndarray) -> float:
    """Angle (deg) between the disk axes of two attitudes."""
    dot = float(np.clip(np.dot(_disk_axis(q_a), _disk_axis(q_b)), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _decompose(q_target: np.ndarray, q_body: np.ndarray) -> tuple[float, float]:
    """
    Replicate thrust_vector_rotation_angles(): returns
        (thrust_error_angle_rad, heading_error_rad_signed).
    """
    up = np.array([0.0, 0.0, -1.0])
    tvec_t = _quat_rotate(q_target, up)   # target disk axis in NED
    tvec_b = _quat_rotate(q_body, up)     # body   disk axis in NED

    dot = float(np.clip(np.dot(tvec_b, tvec_t), -1.0, 1.0))
    thrust_err = math.acos(dot)

    cross = np.cross(tvec_b, tvec_t)
    clen = float(np.linalg.norm(cross))
    if clen < 1e-9 or thrust_err < 1e-9:
        cross_b = up.copy()
    else:
        cross = cross / clen
        cross_b = _quat_rotate(_quat_inverse(q_body), cross)  # into body frame

    thrust_corr = _quat_from_axis_angle(cross_b, thrust_err)
    heading_corr = _quat_mul(
        _quat_mul(_quat_inverse(thrust_corr), _quat_inverse(q_body)), q_target
    )
    heading_err_z = _quat_to_axis_angle(heading_corr)[2]
    return thrust_err, heading_err_z


def _inv_sqrt_controller(output: float, p: float, d_max: float) -> float:
    """Mirror AP_Math/control.cpp inv_sqrt_controller()."""
    if d_max > 0.0 and p == 0.0:
        return (output * output) / (2.0 * d_max)
    if d_max <= 0.0 and p != 0.0:
        return output / p
    if d_max <= 0.0 and p == 0.0:
        return 0.0
    linear_velocity = d_max / p
    if abs(output) < linear_velocity:
        return output / p
    linear_dist = d_max / (p * p)
    stopping = (linear_dist * 0.5) + (output * output) / (2.0 * d_max)
    return stopping if output > 0.0 else -stopping


def _heading_error_max(rat_yaw_p: float, ang_yaw_p: float, acc_y_degss: float) -> float:
    """Mirror the heading_error_max in thrust_heading_rotation_angles()."""
    accel = math.radians(acc_y_degss)
    heading_accel = min(max(accel / 2.0, ACCEL_Y_MIN_RADSS), ACCEL_Y_MAX_RADSS)
    if rat_yaw_p == 0.0:
        return YAW_MAX_ERROR_RAD
    return min(_inv_sqrt_controller(1.0 / rat_yaw_p, ang_yaw_p, heading_accel),
               YAW_MAX_ERROR_RAD)


# ── log loading ───────────────────────────────────────────────────────────────

def _dynamics_start_tsim(log_dir: Path) -> float | None:
    p = log_dir / "events.jsonl"
    if not p.exists():
        return None
    for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            e = json.loads(line)
        except json.JSONDecodeError:
            continue
        if e.get("event") == "dynamics_start":
            return float(e["t_sim"])
    return None


def _load_dataflash(bin_path: Path, t_dyn_start_s: float) -> dict:
    """Return {ATT, PIDY, RATE: [records w/ t_dyn], PARM: {name: value}}."""
    try:
        from pymavlink import DFReader as _DFR
    except ImportError:
        return {}
    if not bin_path.exists():
        return {}

    wanted = {"ATT", "PIDY", "RATE", "SIM"}
    result: dict[str, list[dict]] = {k: [] for k in wanted}
    params: dict[str, float] = {}
    try:
        log = _DFR.DFReader_binary(str(bin_path), zero_time_base=False)
    except Exception:
        return {}

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
        if mtype == "PARM":
            name = d.get("Name", "")
            if name:
                try:
                    params[name] = float(d.get("Value", 0.0))
                except (TypeError, ValueError):
                    pass
            continue
        if mtype not in wanted:
            continue
        t_us = d.get("TimeUS", 0)
        d["t_dyn"] = round(t_us / 1e6 - t_dyn_start_s, 3)
        result[mtype].append(d)

    result["PARM"] = params  # type: ignore[assignment]
    return result


def _nearest(records: list[dict], t: float) -> dict | None:
    if not records:
        return None
    return min(records, key=lambda r: abs(r["t_dyn"] - t))


# ── analysis ──────────────────────────────────────────────────────────────────

def analyze(log_dir: Path, settle_s: float, observe_s: float,
            dump_samples: bool, gain_override: dict) -> None:
    t_dyn_start = _dynamics_start_tsim(log_dir)
    if t_dyn_start is None:
        print(f"[ERROR] No dynamics_start in {log_dir/'events.jsonl'} — "
              "cannot align t_dyn.")
        return

    df = _load_dataflash(log_dir / "dataflash.BIN", t_dyn_start)
    att = df.get("ATT", [])
    pidy = df.get("PIDY", [])
    rate = df.get("RATE", [])
    sim = df.get("SIM", [])
    parm = df.get("PARM", {})
    if not att:
        print("[ERROR] No ATT records (pymavlink missing or dataflash absent).")
        return

    rat_p = gain_override.get("ratp") or parm.get("ATC_RAT_YAW_P", DEFAULT_RAT_YAW_P)
    ang_p = gain_override.get("angp") or parm.get("ATC_ANG_YAW_P", DEFAULT_ANG_YAW_P)
    acc_y = gain_override.get("accy") or parm.get("ATC_ACC_Y_MAX", DEFAULT_ACC_Y_DEGSS)
    he_max = _heading_error_max(rat_p, ang_p, acc_y)

    t_end = att[-1]["t_dyn"]

    print(f"\n{'='*74}")
    print("  ATTITUDE-ERROR DECOMPOSITION (GUIDED quaternion controller)")
    print(f"  Log dir : {log_dir}")
    print(f"  Window  : settle={settle_s:.0f}s  observe={observe_s:.0f}s")
    print(f"  ATT     : {len(att)} samples  t_dyn=[{att[0]['t_dyn']:.1f}, {t_end:.1f}]")
    print(f"  Gains   : ATC_RAT_YAW_P={rat_p:.4f}  ATC_ANG_YAW_P={ang_p:.3f}  "
          f"ATC_ACC_Y_MAX={acc_y:.0f} deg/s2")
    print(f"  Clamps  : heading_error_max={math.degrees(he_max):6.2f} deg   "
          f"thrust-bypass={math.degrees(THRUST_BYPASS_RAD):.0f} deg")
    print(f"  Truth   : SIM records={len(sim)}  "
          f"({'EKF-vs-truth comparison ON' if sim else 'no SIM truth in log'})")
    print(f"{'='*74}\n")

    # Per-sample decomposition.
    rows = []
    for a in att:
        q_t = _quat_from_euler(math.radians(a.get("DesRoll", 0.0)),
                               math.radians(a.get("DesPitch", 0.0)),
                               math.radians(a.get("DesYaw", 0.0)))
        q_b = _quat_from_euler(math.radians(a.get("Roll", 0.0)),
                               math.radians(a.get("Pitch", 0.0)),
                               math.radians(a.get("Yaw", 0.0)))
        thr_e, head_e = _decompose(q_t, q_b)
        # EKF confusion: angle between TRUE disk axis (SIM) and EKF disk axis (ATT).
        ekf_tilt_err = float("nan")
        st = _nearest(sim, a["t_dyn"]) if sim else None
        if st is not None:
            q_truth = _quat_from_euler(math.radians(st.get("Roll", 0.0)),
                                       math.radians(st.get("Pitch", 0.0)),
                                       math.radians(st.get("Yaw", 0.0)))
            ekf_tilt_err = _disk_axis_err_deg(q_truth, q_b)
        rows.append({
            "t_dyn": a["t_dyn"],
            "thr_e_deg": math.degrees(thr_e),
            "head_e_deg": math.degrees(head_e),
            "bypass": thr_e > THRUST_BYPASS_RAD,
            "clamp": abs(head_e) > he_max,
            "ekf_tilt_err": ekf_tilt_err,
            "DesYaw": a.get("DesYaw", 0.0),
            "Yaw": a.get("Yaw", 0.0),
        })

    # ── 5 s bucket summary ────────────────────────────────────────────────────
    print("[TIMELINE]  5 s buckets   (ekfTilt = TRUE disk axis vs EKF disk axis)")
    hdr = (f"  {'t_dyn':>11}  {'thrErr':>7}  {'|hdErr|':>7}  {'byp%':>5}  "
           f"{'clmp%':>5}  {'ekfTilt':>7}  {'Tar':>8}  {'Act':>8}  {'I':>6}")
    print(hdr)
    print(f"  {'-'*(len(hdr)-2)}")

    t = 0.0
    bucket = 5.0
    while t < t_end + bucket:
        rb = [r for r in rows if t <= r["t_dyn"] < t + bucket]
        if not rb:
            t += bucket
            continue
        thr_mean = sum(r["thr_e_deg"] for r in rb) / len(rb)
        thr_max = max(r["thr_e_deg"] for r in rb)
        hd_mean = sum(abs(r["head_e_deg"]) for r in rb) / len(rb)
        byp_pct = 100.0 * sum(1 for r in rb if r["bypass"]) / len(rb)
        clmp_pct = 100.0 * sum(1 for r in rb if r["clamp"]) / len(rb)
        ekf_vals = [r["ekf_tilt_err"] for r in rb if r["ekf_tilt_err"] == r["ekf_tilt_err"]]
        ekf_mean = sum(ekf_vals) / len(ekf_vals) if ekf_vals else float("nan")

        pb = [p for p in pidy if t <= p["t_dyn"] < t + bucket]
        if pb:
            tar = sum(p.get("Tar", 0.0) for p in pb) / len(pb)
            act = sum(p.get("Act", 0.0) for p in pb) / len(pb)
            iterm = sum(p.get("I", 0.0) for p in pb) / len(pb)
            pid_str = f"  {tar:8.1f}  {act:8.1f}  {iterm:6.3f}"
        else:
            pid_str = f"  {'-':>8}  {'-':>8}  {'-':>6}"

        marker = ""
        if t <= settle_s < t + bucket:
            marker = "  <-- settle"
        elif t <= settle_s + observe_s < t + bucket:
            marker = "  <-- obs end"

        ekf_str = f"{ekf_mean:7.1f}" if ekf_mean == ekf_mean else f"{'-':>7}"
        print(f"  t={t:4.0f}-{t+bucket:<4.0f}  {thr_mean:7.1f}  {hd_mean:7.1f}  "
              f"{byp_pct:4.0f}%  {clmp_pct:4.0f}%  {ekf_str}{pid_str}"
              f"   (thrMax={thr_max:.0f}){marker}")
        t += bucket

    # ── observe-window verdict ────────────────────────────────────────────────
    win = [r for r in rows if settle_s <= r["t_dyn"] <= settle_s + observe_s]
    print()
    if not win:
        print("[VERDICT]  observe window contains no ATT samples.")
        return

    n = len(win)
    byp = sum(1 for r in win if r["bypass"])
    clmp = sum(1 for r in win if r["clamp"])
    thr_mean = sum(r["thr_e_deg"] for r in win) / n
    hd_mean = sum(abs(r["head_e_deg"]) for r in win) / n
    ekf_win = [r["ekf_tilt_err"] for r in win if r["ekf_tilt_err"] == r["ekf_tilt_err"]]
    ekf_mean = sum(ekf_win) / len(ekf_win) if ekf_win else float("nan")

    # Does the target yaw FOLLOW the body yaw? Compare unwrapped rates.
    des_rate = _yaw_rate(win, "DesYaw")
    act_rate = _yaw_rate(win, "Yaw")

    print("[VERDICT]  observation window")
    print(f"  samples              : {n}")
    print(f"  mean thrust error    : {thr_mean:6.2f} deg")
    print(f"  mean |heading error| : {hd_mean:6.2f} deg   "
          f"(clamp at {math.degrees(he_max):.2f} deg)")
    print(f"  thrust-bypass active : {100.0*byp/n:5.1f}% of window  (>60 deg)")
    print(f"  heading-clamp active : {100.0*clmp/n:5.1f}% of window")
    if ekf_mean == ekf_mean:
        print(f"  EKF disk-axis error  : {ekf_mean:6.2f} deg  (TRUE vs EKF estimate)")
    print(f"  DesYaw rate          : {des_rate:7.1f} deg/s")
    print(f"  Yaw    rate          : {act_rate:7.1f} deg/s")
    follows = (abs(act_rate) > 20.0
               and abs(des_rate - act_rate) < 0.30 * max(abs(act_rate), 1.0))
    ekf_confused = ekf_mean == ekf_mean and ekf_mean > 20.0
    if ekf_confused:
        print(f"  -> EKF IS CONFUSED: its estimated disk axis is {ekf_mean:.0f} deg off "
              "the TRUE\n     attitude. A body spinning about a tilted axis gives the "
              "SAME body-frame gyro\n     as a LEVEL spin; the EKF rejects the (sweeping) "
              "accel during the fast spin\n     and collapses the tilt to ~level. The "
              "controller is fed a wrong attitude, so\n     the 60 deg thrust-bypass / 45 deg "
              "heading-clamp fire and the yaw motor never\n     opposes the spin. Break the "
              "loop upstream: stop the body spinning up\n     (H_YAW_TRIM feedforward) so the "
              "EKF can hold the tilt.")
    elif follows:
        print("  -> TARGET FOLLOWS SPIN: DesYaw tracks Yaw; the heading clamp is "
              "dragging\n     the attitude target onto the spinning body. The yaw "
              "PID sees ~0 rate\n     error, so the motor never opposes the spin. "
              "Raise yaw authority\n     (ATC_RAT_YAW_P / ATC_ANG_YAW_P) or add yaw "
              "feedforward (H_YAW_TRIM).")
    elif clmp > n * 0.5 or byp > n * 0.5:
        print("  -> CLAMP-LIMITED: heading error is being clamped a majority of the "
              "window;\n     the yaw loop cannot command enough rate to null the spin.")
    else:
        print("  -> Clamps mostly inactive in window; look elsewhere "
              "(rate-loop gains, IMAX, motor output).")
    print()


def _yaw_rate(win: list[dict], key: str) -> float:
    """Average unwrapped yaw rate (deg/s) over the window for the given key."""
    if len(win) < 2:
        return float("nan")
    times = [r["t_dyn"] for r in win]
    vals = np.unwrap([math.radians(r[key]) for r in win])
    dt = times[-1] - times[0]
    if dt <= 0:
        return float("nan")
    return math.degrees(vals[-1] - vals[0]) / dt


# ── per-sample dump ───────────────────────────────────────────────────────────

def dump_window(log_dir: Path, settle_s: float, observe_s: float,
                gain_override: dict) -> None:
    t_dyn_start = _dynamics_start_tsim(log_dir)
    if t_dyn_start is None:
        return
    df = _load_dataflash(log_dir / "dataflash.BIN", t_dyn_start)
    att = df.get("ATT", [])
    pidy = df.get("PIDY", [])
    parm = df.get("PARM", {})
    rat_p = gain_override.get("ratp") or parm.get("ATC_RAT_YAW_P", DEFAULT_RAT_YAW_P)
    ang_p = gain_override.get("angp") or parm.get("ATC_ANG_YAW_P", DEFAULT_ANG_YAW_P)
    acc_y = gain_override.get("accy") or parm.get("ATC_ACC_Y_MAX", DEFAULT_ACC_Y_DEGSS)
    he_max = _heading_error_max(rat_p, ang_p, acc_y)

    print("[SAMPLES]  observe window (per ATT record)")
    print(f"  {'t_dyn':>8}  {'thrErr':>7}  {'hdErr':>7}  {'byp':>3}  {'clmp':>4}  "
          f"{'DesYaw':>7}  {'Yaw':>7}  {'Tar':>8}  {'Act':>8}")
    for a in att:
        if not (settle_s <= a["t_dyn"] <= settle_s + observe_s):
            continue
        q_t = _quat_from_euler(math.radians(a.get("DesRoll", 0.0)),
                               math.radians(a.get("DesPitch", 0.0)),
                               math.radians(a.get("DesYaw", 0.0)))
        q_b = _quat_from_euler(math.radians(a.get("Roll", 0.0)),
                               math.radians(a.get("Pitch", 0.0)),
                               math.radians(a.get("Yaw", 0.0)))
        thr_e, head_e = _decompose(q_t, q_b)
        p = _nearest(pidy, a["t_dyn"])
        tar = p.get("Tar", float("nan")) if p else float("nan")
        act = p.get("Act", float("nan")) if p else float("nan")
        byp = "Y" if thr_e > THRUST_BYPASS_RAD else "."
        clmp = "Y" if abs(head_e) > he_max else "."
        print(f"  {a['t_dyn']:8.2f}  {math.degrees(thr_e):7.1f}  "
              f"{math.degrees(head_e):7.1f}  {byp:>3}  {clmp:>4}  "
              f"{a.get('DesYaw', 0.0):7.1f}  {a.get('Yaw', 0.0):7.1f}  "
              f"{tar:8.1f}  {act:8.1f}")
    print()


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        sys.exit(1)

    gain_override: dict = {}
    dump = False
    positional: list[str] = []
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--samples":
            dump = True
        elif a == "--ratp":
            i += 1
            gain_override["ratp"] = float(args[i])
        elif a == "--angp":
            i += 1
            gain_override["angp"] = float(args[i])
        elif a == "--accy":
            i += 1
            gain_override["accy"] = float(args[i])
        else:
            positional.append(a)
        i += 1

    if not positional:
        print("[ERROR] No test name or log directory given.")
        sys.exit(1)

    arg = positional[0]
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

    settle_s = float(positional[1]) if len(positional) > 1 else 75.0
    observe_s = float(positional[2]) if len(positional) > 2 else 20.0

    analyze(candidate, settle_s, observe_s, dump, gain_override)
    if dump:
        dump_window(candidate, settle_s, observe_s, gain_override)


if __name__ == "__main__":
    main()
