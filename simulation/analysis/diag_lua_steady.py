"""
diag_lua_steady.py -- Post-mortem analysis of test_lua_flight_steady.

Reads telemetry.csv + ekf_telemetry.csv from the lua_full_test_lua_flight_steady
log directory and prints a second-by-second table showing what happens around
and after kinematic exit.

Usage:
    python simulation/analysis/diag_lua_steady.py
    python simulation/analysis/diag_lua_steady.py --log <log_dir>
"""
import argparse
import csv
import math
import os
import re
import sys
from pathlib import Path

# ── CLI ──────────────────────────────────────────────────────────────────────
ap = argparse.ArgumentParser()
ap.add_argument("--log", default=None,
                help="Log directory (default: simulation/logs/lua_full_test_lua_flight_steady)")
ap.add_argument("--window", type=float, default=60.0,
                help="Seconds of free-flight to show after kinematic exit (default: 60)")
ap.add_argument("--pre", type=float, default=5.0,
                help="Seconds before kinematic exit to show (default: 5)")
args = ap.parse_args()

REPO = Path(__file__).resolve().parents[2]
LOG_DIR = Path(args.log) if args.log else \
          REPO / "simulation/logs/lua_full_test_lua_flight_steady"

TEL_CSV = LOG_DIR / "telemetry.csv"
EKF_CSV = LOG_DIR / "ekf_telemetry.csv"
MED_LOG = LOG_DIR / "mediator.log"

for f in [TEL_CSV]:
    if not f.exists():
        print(f"ERROR: {f} not found. Run test_lua_flight_steady first.")
        sys.exit(1)

# ── Load telemetry.csv ───────────────────────────────────────────────────────
print(f"Loading {TEL_CSV} ...")
tel_rows = []
with open(TEL_CSV, newline="", encoding="utf-8") as fh:
    reader = csv.DictReader(fh)
    for row in reader:
        t = float(row["t_sim"])
        tel_rows.append({
            "t":          t,
            "phase":      row.get("phase", ""),
            "pos_z":      float(row["pos_z"]),          # NED z; altitude = -pos_z
            "vel_z":      float(row["vel_z"]),           # NED z; positive = down
            "vel_mag":    math.sqrt(float(row["vel_x"])**2 + float(row["vel_y"])**2 + float(row["vel_z"])**2),
            "col_rad":    float(row["collective_rad"]),
            "tension":    float(row["tether_tension"]),
            "aero_T":     float(row["aero_T"]),
            "s1":         float(row["servo_s1"]) if row["servo_s1"] else 0.0,
            "s2":         float(row["servo_s2"]) if row["servo_s2"] else 0.0,
            "s3":         float(row["servo_s3"]) if row["servo_s3"] else 0.0,
            "bz_x":       float(row["bz_eq_x"]) if row["bz_eq_x"] else 0.0,
            "bz_y":       float(row["bz_eq_y"]) if row["bz_eq_y"] else 0.0,
            "bz_z":       float(row["bz_eq_z"]) if row["bz_eq_z"] else 0.0,
            "r00": float(row["r00"]), "r01": float(row["r01"]), "r02": float(row["r02"]),
            "r10": float(row["r10"]), "r11": float(row["r11"]), "r12": float(row["r12"]),
            "r20": float(row["r20"]), "r21": float(row["r21"]), "r22": float(row["r22"]),
        })

print(f"  {len(tel_rows)} telemetry rows  t=[{tel_rows[0]['t']:.1f}, {tel_rows[-1]['t']:.1f}] s")

# ── Load ekf_telemetry.csv ───────────────────────────────────────────────────
ekf_rows = []
if EKF_CSV.exists():
    with open(EKF_CSV, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            try:
                ekf_rows.append({
                    "t_ms":  int(row["time_boot_ms"]),
                    "roll":  float(row["att_roll"]),
                    "pitch": float(row["att_pitch"]),
                    "yaw":   float(row["att_yaw"]),
                    "pos_n": float(row["pos_n"]),
                    "pos_e": float(row["pos_e"]),
                    "pos_d": float(row["pos_d"]),
                    "vel_n": float(row["vel_n"]),
                    "vel_e": float(row["vel_e"]),
                    "vel_d": float(row["vel_d"]),
                })
            except (ValueError, KeyError):
                pass
    print(f"  {len(ekf_rows)} EKF rows")
else:
    print(f"  WARNING: {EKF_CSV} not found -- EKF comparison skipped")

# ── Find kinematic exit time ─────────────────────────────────────────────────
# phase column: non-empty string during kinematic ("kinematic" or blank).
# Telemetry written every 400 Hz step; phase column is set by mediator.
# Find the last row where phase != "" (kinematic still active) then first free.
t_kin_exit = None
for i, r in enumerate(tel_rows):
    if r["phase"] == "" and i > 0 and tel_rows[i-1]["phase"] != "":
        t_kin_exit = r["t"]
        break

if t_kin_exit is None:
    # Try mediator log
    if MED_LOG.exists():
        text = MED_LOG.read_text(encoding="utf-8", errors="replace")
        m = re.search(r"TRANSITION kinematic.*?at t=(\d+\.\d+)", text)
        if m:
            t_kin_exit = float(m.group(1))
            print(f"  Kinematic exit from mediator log: t={t_kin_exit:.2f} s")
    if t_kin_exit is None:
        # Fall back: look for damp_alpha going to zero
        for r in tel_rows:
            if float(r.get("phase", 0) or 0) == 0:
                break
        t_kin_exit = 65.0  # hardcoded fallback
        print(f"  WARNING: could not detect kinematic exit -- assuming t={t_kin_exit:.0f} s")

print(f"\nKinematic exit: t = {t_kin_exit:.2f} s")

# ── Find home altitude (altitude at first free-flight row) ──────────────────
# altitude = -pos_z; home_alt is roughly the altitude at the target position.
home_alt = None
for r in tel_rows:
    if r["t"] >= t_kin_exit:
        home_alt = -r["pos_z"]
        break
if home_alt is None:
    home_alt = 14.0  # default
print(f"Target altitude:  {home_alt:.2f} m")

# ── Build EKF lookup (by time_boot_ms) ──────────────────────────────────────
# Mediator starts SITL then arms; sim time ~ wall time.
# EKF t_ms is ArduPilot boot time; arm happens at roughly t~15 s after boot.
# We want EKF attitude vs physics body_z around kinematic exit.
# Match by nearest t_ms (convert sim t_sim -> t_ms: t_ms ~ (t_sim + arm_offset) * 1000).
# We don't know exact arm offset, so we use relative time from kinematic exit.

def ekf_at(t_ms_approx, window_ms=200):
    """Return the EKF row closest to t_ms_approx, within window_ms."""
    best = None
    best_dt = float("inf")
    for r in ekf_rows:
        dt = abs(r["t_ms"] - t_ms_approx)
        if dt < best_dt and dt < window_ms:
            best_dt = dt
            best = r
    return best

# Estimate arm offset: look for first row where EKF position ~= 0 (start of sim)
# This is unreliable; just note the EKF t range.
if ekf_rows:
    ekf_t_min = ekf_rows[0]["t_ms"]
    ekf_t_max = ekf_rows[-1]["t_ms"]
    print(f"EKF time range:  {ekf_t_min/1000:.1f} .. {ekf_t_max/1000:.1f} s (boot time)")

# ── Physics body_z from R matrix ─────────────────────────────────────────────
def body_z_from_R(r):
    """Extract body_z in NED from rotation matrix stored in telemetry row."""
    # R is body-to-world (NED). body_z = R @ [0,0,1] = third column.
    return (r["r02"], r["r12"], r["r22"])

def angle_deg(v1, v2):
    """Angle in degrees between two 3-vectors."""
    dot = sum(a*b for a,b in zip(v1,v2))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))

# ── Filter rows to display window ────────────────────────────────────────────
t_lo = t_kin_exit - args.pre
t_hi = t_kin_exit + args.window
display_rows = [r for r in tel_rows if t_lo <= r["t"] <= t_hi]

# Downsample to ~1 row per second for readability
SAMPLE_DT = 0.5
sampled = []
last_t = -999.0
for r in display_rows:
    if r["t"] - last_t >= SAMPLE_DT:
        sampled.append(r)
        last_t = r["t"]

# ── Print header ─────────────────────────────────────────────────────────────
print()
print("=" * 110)
print(f"{'t_sim':>7}  {'phase':>6}  {'alt_m':>6}  {'vel_z':>6}  {'vel_m':>6}  "
      f"{'col_rad':>8}  {'T_aero':>7}  {'tension':>8}  {'s1':>5}  {'s2':>5}  {'s3':>5}  "
      f"{'bz_phys_y':>10}  {'FLAG'}")
print("-" * 110)

ALERT_TENSION = 500.0   # N -- flag if tension exceeds this
ALERT_VEL     = 5.0     # m/s -- flag if speed exceeds this
ALERT_ALT_LO  = 3.0     # m -- flag if altitude drops below this

first_free = True
for r in sampled:
    t     = r["t"]
    phase = "kin" if r["phase"] else "free"
    alt   = -r["pos_z"]
    vel_z = r["vel_z"]
    vel_m = r["vel_mag"]
    col   = r["col_rad"]
    T     = r["aero_T"]
    ten   = r["tension"]
    s1    = r["s1"]
    s2    = r["s2"]
    s3    = r["s3"]
    bz_y  = r["bz_y"]  # East component of body_z (should be ~0.87 at equilibrium)

    # Kinematic exit marker
    marker = ""
    if phase == "free" and first_free:
        marker = "<-- KIN EXIT"
        first_free = False

    flags = []
    if ten > ALERT_TENSION:
        flags.append(f"TENSION={ten:.0f}N")
    if vel_m > ALERT_VEL:
        flags.append(f"SPEED={vel_m:.1f}m/s")
    if alt < ALERT_ALT_LO and phase == "free":
        flags.append(f"ALT={alt:.1f}m")
    if col < -0.25:
        flags.append(f"COL_NEG={col:.3f}")
    if abs(s3 - 1500) > 400 and phase == "free":
        if col < -0.20:
            flags.append(f"S3_LOW={s3:.0f}")

    flag_str = "  ".join(flags)
    if marker:
        flag_str = marker + ("  " + flag_str if flag_str else "")

    print(f"{t:7.1f}  {phase:>6}  {alt:6.2f}  {vel_z:6.3f}  {vel_m:6.3f}  "
          f"{col:8.4f}  {T:7.2f}  {ten:8.2f}  {s1:5.0f}  {s2:5.0f}  {s3:5.0f}  "
          f"{bz_y:10.4f}  {flag_str}")

print("=" * 110)

# ── Summary stats ─────────────────────────────────────────────────────────────
free = [r for r in tel_rows if r["t"] >= t_kin_exit and not r["phase"]]
if free:
    alts    = [-r["pos_z"] for r in free]
    tensions = [r["tension"] for r in free]
    cols    = [r["col_rad"] for r in free]
    vels    = [r["vel_mag"] for r in free]
    print(f"\nFree-flight summary ({len(free)} rows, t=[{free[0]['t']:.1f}, {free[-1]['t']:.1f}] s):")
    print(f"  Altitude: min={min(alts):.2f}  max={max(alts):.2f}  mean={sum(alts)/len(alts):.2f} m")
    print(f"  Tension:  min={min(tensions):.1f}  max={max(tensions):.1f}  mean={sum(tensions)/len(tensions):.1f} N")
    print(f"  Col rad:  min={min(cols):.4f}  max={max(cols):.4f}  mean={sum(cols)/len(cols):.4f} rad")
    print(f"  Speed:    min={min(vels):.3f}  max={max(vels):.3f}  mean={sum(vels)/len(vels):.3f} m/s")

    # Find first "bad" event
    for r in free:
        if r["tension"] > ALERT_TENSION:
            print(f"\n  FIRST high tension: t={r['t']:.2f} s  tension={r['tension']:.0f} N"
                  f"  alt={-r['pos_z']:.2f} m  col={r['col_rad']:.4f} rad"
                  f"  vel_mag={r['vel_mag']:.2f} m/s")
            break
    for r in free:
        if r["vel_mag"] > ALERT_VEL:
            print(f"  FIRST high speed:   t={r['t']:.2f} s  vel_mag={r['vel_mag']:.2f} m/s"
                  f"  alt={-r['pos_z']:.2f} m  col={r['col_rad']:.4f} rad"
                  f"  tension={r['tension']:.0f} N")
            break

# ── Mediator log: extract key events ─────────────────────────────────────────
if MED_LOG.exists():
    print("\n--- Key events from mediator.log ---")
    patterns = [
        "TRANSITION", "RAWES", "GPS", "EKF", "captured", "SCR_USER6",
        "col=", "body_z=", "vel_NED=", "tension",
    ]
    lines = MED_LOG.read_text(encoding="utf-8", errors="replace").splitlines()
    # Show lines matching key patterns that fall in the window
    for line in lines:
        tm = re.search(r"t=(\d+\.\d+)", line)
        if tm:
            lt = float(tm.group(1))
            if lt < t_lo or lt > t_hi:
                continue
        if any(p.lower() in line.lower() for p in patterns):
            # Trim long lines
            print("  " + line[:140])

print("\nDone.")
