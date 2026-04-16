#!/usr/bin/env python3
"""
validate_sitl_sensors.py -- Physical consistency check for SITL sensor data.

Loads telemetry CSV and verifies that every value sent to ArduPilot via SITL
is physically self-consistent with the logged trajectory state.

Checks
------
A  accel_world == diff(vel) / dt
     The world-frame acceleration stored in telemetry must equal the
     backward finite difference of velocity.  This is trivially true by
     construction; any error means the mediator is using a different formula.

B  sens_accel == R.T @ (accel_world - gravity)
     The body-frame specific force sent to ArduPilot must equal the
     world-frame acceleration minus gravity, rotated into body frame.

C  sens_gyro == R.T @ omega
     The body-frame gyro sent to ArduPilot must equal the world-frame angular
     velocity rotated into body frame (no spin stripping).

D  rpy from R
     The roll/pitch/yaw logged must equal the ZYX Euler decomposition of
     the rotation matrix R.

E  sens_vel == vel
     Velocity sent to SITL must equal the kinematic velocity.

F  Gravity magnitude at rest (hold phase)
     During stationary hold (vel=0), |accel_world|=0 so the specific force
     must be exactly -gravity.  |sens_accel| should be 9.81 m/s^2 and the
     direction should be consistent with level orientation.

G  Centripetal acceleration during fast circles
     During const phase, |accel_world_horiz| should equal v^2 / r_circle.

Usage
-----
  python simulation/analysis/validate_sitl_sensors.py <log_name>
  e.g.  python simulation/analysis/validate_sitl_sensors.py kin_gps_test_kinematic_gps
"""
from __future__ import annotations

import sys
import csv
import math
import numpy as np
from pathlib import Path

LOG_ROOT   = Path(__file__).resolve().parents[2] / "simulation" / "logs"
GRAVITY    = np.array([0.0, 0.0, 9.81])   # NED: +Z is down


# ---------------------------------------------------------------------------
def _load(path: Path) -> list[dict]:
    rows: list[dict] = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            parsed: dict = {}
            for k, v in row.items():
                if v in ("", "nan", "inf"):
                    parsed[k] = float("nan")
                else:
                    try:
                        parsed[k] = float(v)
                    except ValueError:
                        parsed[k] = v   # keep string columns as-is
            rows.append(parsed)
    return rows


def _R(row: dict) -> np.ndarray:
    return np.array([
        [row["r00"], row["r01"], row["r02"]],
        [row["r10"], row["r11"], row["r12"]],
        [row["r20"], row["r21"], row["r22"]],
    ])


def _euler_zyx(R: np.ndarray) -> np.ndarray:
    sin_p = float(np.clip(-R[2, 0], -1.0, 1.0))
    pitch = math.asin(sin_p)
    cp    = math.cos(pitch)
    if abs(cp) > 1e-6:
        roll = math.atan2(R[2, 1] / cp, R[2, 2] / cp)
        yaw  = math.atan2(R[1, 0] / cp, R[0, 0] / cp)
    else:
        roll = 0.0
        yaw  = math.atan2(-R[0, 1], R[1, 1])
    return np.array([roll, pitch, yaw])


def _wrap(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return float((a + math.pi) % (2 * math.pi) - math.pi)


def _fmt(label: str, errs: np.ndarray, unit: str, warn: float, bad: float) -> None:
    m  = float(np.nanmedian(errs))
    mx = float(np.nanmax(np.abs(errs)))
    flag = "[!!]" if mx > bad else ("[! ]" if mx > warn else "[ok]")
    print(f"  {flag}  {label:<38s}  median={m:+.4f}  max={mx:.4f}  {unit}")


# ---------------------------------------------------------------------------
def main() -> None:
    name = sys.argv[1] if len(sys.argv) > 1 else "kin_gps_test_kinematic_gps"
    log_dir = LOG_ROOT / name
    csv_path = log_dir / "telemetry.csv"
    if not csv_path.exists():
        # try treating as bare test name (no prefix)
        matches = sorted(LOG_ROOT.glob(f"*{name}*"))
        if matches:
            log_dir  = matches[-1]
            csv_path = log_dir / "telemetry.csv"
        if not csv_path.exists():
            print(f"ERROR: cannot find telemetry.csv for {name!r}")
            sys.exit(1)

    rows = _load(csv_path)
    print(f"Loaded {len(rows)} rows from {csv_path}")
    print()

    # ------------------------------------------------------------------ #
    # Pre-allocate result arrays                                          #
    # ------------------------------------------------------------------ #
    n = len(rows)
    t  = np.array([r["t_sim"] for r in rows])
    da = np.array([r["damp_alpha"] for r in rows])

    # kinematic mask (damp_alpha > 0) and per-phase masks
    kin_mask  = da > 0.0
    # phase from note/damp_alpha combination -- use damp_alpha as proxy
    hold_mask = np.zeros(n, bool)
    circ_mask = np.zeros(n, bool)

    # We need the events log to know phase boundaries; fall back to
    # velocity magnitude as proxy.
    vel = np.column_stack([
        [r["vel_x"] for r in rows],
        [r["vel_y"] for r in rows],
        [r["vel_z"] for r in rows],
    ])
    speed = np.linalg.norm(vel, axis=1)
    hold_mask = kin_mask & (speed < 0.05)
    circ_mask = kin_mask & (speed > 0.5)

    # Build arrays
    accel_world = np.column_stack([
        [r["accel_x"] for r in rows],
        [r["accel_y"] for r in rows],
        [r["accel_z"] for r in rows],
    ])
    omega = np.column_stack([
        [r["omega_x"] for r in rows],
        [r["omega_y"] for r in rows],
        [r["omega_z"] for r in rows],
    ])
    sens_accel = np.column_stack([
        [r["sens_accel_x"] for r in rows],
        [r["sens_accel_y"] for r in rows],
        [r["sens_accel_z"] for r in rows],
    ])
    sens_gyro = np.column_stack([
        [r["sens_gyro_x"] for r in rows],
        [r["sens_gyro_y"] for r in rows],
        [r["sens_gyro_z"] for r in rows],
    ])
    sens_vel = np.column_stack([
        [r["sens_vel_n"] for r in rows],
        [r["sens_vel_e"] for r in rows],
        [r["sens_vel_d"] for r in rows],
    ])
    rpy_log = np.column_stack([
        [r["rpy_roll"] for r in rows],
        [r["rpy_pitch"] for r in rows],
        [r["rpy_yaw"] for r in rows],
    ])
    pos = np.column_stack([
        [r["pos_x"] for r in rows],
        [r["pos_y"] for r in rows],
        [r["pos_z"] for r in rows],
    ])

    # ------------------------------------------------------------------ #
    # Recompute expected sensor values from R, accel, omega               #
    # ------------------------------------------------------------------ #
    exp_accel = np.full((n, 3), float("nan"))
    exp_gyro  = np.full((n, 3), float("nan"))
    exp_rpy   = np.full((n, 3), float("nan"))

    for i, row in enumerate(rows):
        R = _R(row)
        # Check R is orthogonal
        # Expected accel: R.T @ (accel_world - gravity)
        a_spec = accel_world[i] - GRAVITY
        exp_accel[i] = R.T @ a_spec
        # Expected gyro: R.T @ omega (no spin stripping)
        exp_gyro[i] = R.T @ omega[i]
        # Expected rpy from R
        exp_rpy[i]  = _euler_zyx(R)

    # ------------------------------------------------------------------ #
    # CHECK A -- accel_world == diff(vel) / dt                            #
    # ------------------------------------------------------------------ #
    print("=" * 70)
    print("CHECK A: accel_world == diff(vel) / dt")
    print("  (backward finite difference -- should be exact by construction)")
    print("=" * 70)
    dt_arr = np.diff(t, prepend=t[0])
    dt_arr[0] = dt_arr[1]  # avoid /0 on first row
    vel_diff = np.diff(vel, axis=0, prepend=vel[:1]) / dt_arr[:, None]

    err_A = accel_world - vel_diff
    # first row is always wrong (no previous vel), skip it
    for ax, name in enumerate("xyz"):
        e = err_A[1:, ax]
        _fmt(f"accel_{name} - diff(vel_{name})/dt", e, "m/s^2", 0.01, 0.1)
    print()

    # ------------------------------------------------------------------ #
    # CHECK B -- sens_accel == R.T @ (accel_world - gravity)              #
    # ------------------------------------------------------------------ #
    print("=" * 70)
    print("CHECK B: sens_accel == R.T @ (accel_world - gravity)")
    print("=" * 70)
    err_B = sens_accel - exp_accel
    for ax, name in enumerate("xyz"):
        _fmt(f"sens_accel_{name} - expected", err_B[:, ax], "m/s^2", 0.001, 0.01)
    print()

    # ------------------------------------------------------------------ #
    # CHECK C -- sens_gyro == R.T @ (omega - omega_spin)                  #
    # ------------------------------------------------------------------ #
    print("=" * 70)
    print("CHECK C: sens_gyro == R.T @ omega")
    print("=" * 70)
    err_C = sens_gyro - exp_gyro
    for ax, name in enumerate("xyz"):
        _fmt(f"sens_gyro_{name} - expected", err_C[:, ax], "rad/s", 0.001, 0.01)
    print()

    # ------------------------------------------------------------------ #
    # CHECK D -- rpy from R                                               #
    # ------------------------------------------------------------------ #
    print("=" * 70)
    print("CHECK D: logged rpy == ZYX Euler from R matrix")
    print("=" * 70)
    err_D = np.array([[_wrap(rpy_log[i, j] - exp_rpy[i, j]) for j in range(3)]
                      for i in range(n)])
    for ax, name in enumerate(["roll", "pitch", "yaw"]):
        _fmt(f"rpy_{name} - euler(R)", err_D[:, ax], "rad", 0.001, 0.01)
    print()

    # ------------------------------------------------------------------ #
    # CHECK E -- sens_vel == vel                                          #
    # ------------------------------------------------------------------ #
    print("=" * 70)
    print("CHECK E: sens_vel == vel (direct copy)")
    print("=" * 70)
    err_E = sens_vel - vel
    for ax, name in enumerate(["N", "E", "D"]):
        _fmt(f"sens_vel_{name} - vel", err_E[:, ax], "m/s", 0.0001, 0.001)
    print()

    # ------------------------------------------------------------------ #
    # CHECK F -- specific force magnitude at rest                         #
    # ------------------------------------------------------------------ #
    print("=" * 70)
    print("CHECK F: specific force magnitude (should be ~9.81 m/s^2 everywhere)")
    print("=" * 70)
    sf_mag = np.linalg.norm(sens_accel, axis=1)
    sf_err = sf_mag - 9.81

    # hold phase only
    if hold_mask.any():
        _fmt("hold: |sens_accel| - 9.81", sf_err[hold_mask], "m/s^2", 0.1, 1.0)
    if circ_mask.any():
        _fmt("circles: |sens_accel| - 9.81", sf_err[circ_mask], "m/s^2", 1.0, 10.0)
    _fmt("all rows: |sens_accel| - 9.81", sf_err, "m/s^2", 1.0, 10.0)
    print()

    # ------------------------------------------------------------------ #
    # CHECK G -- body-frame specific force in hold phase                  #
    # ------------------------------------------------------------------ #
    print("=" * 70)
    print("CHECK G: body-frame specific force direction at rest")
    print("  (hold phase: accel_world=0 so specific force = -gravity in body frame)")
    print("=" * 70)
    if hold_mask.any():
        sf_hold = sens_accel[hold_mask]
        _fmt("hold: sens_accel_x (should be 0)", sf_hold[:, 0], "m/s^2", 0.1, 0.5)
        _fmt("hold: sens_accel_y (should be 0)", sf_hold[:, 1], "m/s^2", 0.1, 0.5)
        _fmt("hold: sens_accel_z (should be ~-9.81)", sf_hold[:, 2] + 9.81, "m/s^2", 0.1, 0.5)
    else:
        print("  (no hold-phase rows found)")
    print()

    # ------------------------------------------------------------------ #
    # CHECK H -- centripetal acceleration during fast circles             #
    # ------------------------------------------------------------------ #
    print("=" * 70)
    print("CHECK H: horizontal accel_world magnitude during fast circles")
    print("  (at v=5 m/s, r=5 m: centripetal = v^2/r = 5.00 m/s^2)")
    print("=" * 70)
    accel_horiz = np.sqrt(accel_world[:, 0]**2 + accel_world[:, 1]**2)
    if circ_mask.any():
        # expected centripetal: v^2 / r; we don't know r directly, report magnitude
        # look for max-speed region (const circles, v~5 m/s)
        fast_mask = circ_mask & (speed > 3.0)
        if fast_mask.any():
            ac = accel_horiz[fast_mask]
            print(f"  fast circle rows ({fast_mask.sum()}):  "
                  f"median={np.median(ac):.3f}  min={ac.min():.3f}  max={ac.max():.3f}  m/s^2")
            exp_centripetal = 5.0**2 / 5.0   # v^2 / r
            _fmt("fast: |accel_horiz| - v^2/r (5.0 m/s^2)", ac - exp_centripetal,
                 "m/s^2", 0.5, 2.0)
    print()

    # ------------------------------------------------------------------ #
    # CHECK I -- R orthogonality                                          #
    # ------------------------------------------------------------------ #
    print("=" * 70)
    print("CHECK I: R matrix orthogonality |R.T @ R - I|")
    print("=" * 70)
    orth_errs = []
    for row in rows:
        R = _R(row)
        err = np.linalg.norm(R.T @ R - np.eye(3))
        orth_errs.append(err)
    orth_errs = np.array(orth_errs)
    _fmt("||R^T R - I||_F", orth_errs, "(unitless)", 1e-6, 1e-4)
    print()

    # ------------------------------------------------------------------ #
    # CHECK J -- velocity heading vs yaw angle                           #
    # ------------------------------------------------------------------ #
    print("=" * 70)
    print("CHECK J: heading gap (yaw angle vs GPS velocity heading)")
    print("  (should be small for EKF to not fight itself)")
    print("=" * 70)
    v_horiz = np.sqrt(vel[:, 0]**2 + vel[:, 1]**2)
    moving  = v_horiz > 0.1
    if moving.any():
        vel_heading = np.degrees(np.arctan2(vel[moving, 1], vel[moving, 0]))
        yaw_deg     = np.degrees(rpy_log[moving, 2])
        gap = np.array([_wrap(math.radians(y - v)) for y, v in zip(yaw_deg, vel_heading)])
        gap_deg = np.degrees(gap)
        m  = float(np.median(np.abs(gap_deg)))
        mx = float(np.max(np.abs(gap_deg)))
        flag = "[!!]" if mx > 30 else ("[! ]" if mx > 10 else "[ok]")
        print(f"  {flag}  yaw_deg - vel_heading_deg:  "
              f"median_abs={m:.1f} deg  max_abs={mx:.1f} deg")
    print()

    # ------------------------------------------------------------------ #
    # Phase summary                                                        #
    # ------------------------------------------------------------------ #
    print("=" * 70)
    print("PHASE SUMMARY")
    print("=" * 70)
    print(f"  Total rows          : {n}")
    print(f"  Kinematic (alpha>0) : {kin_mask.sum()} rows  "
          f"t=[{t[kin_mask].min():.1f}, {t[kin_mask].max():.1f}]s")
    print(f"  Hold (speed<0.05)   : {hold_mask.sum()} rows")
    print(f"  Moving circles      : {circ_mask.sum()} rows")
    print(f"  Free flight         : {(~kin_mask).sum()} rows  "
          f"t=[{t[~kin_mask].min():.1f}, {t[~kin_mask].max():.1f}]s" if (~kin_mask).any() else "")
    print()
    print("Legend: [ok]=ok  [! ]=warn  [!!]=problem")


if __name__ == "__main__":
    main()
