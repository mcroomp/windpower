"""
_calibrate/repl.py -- All _cmd_* handlers, REPL loop, CLI parser, main().
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from datetime import datetime, timezone

from pymavlink import mavutil

from .constants import (
    RawesGCS, WallClock,
    SERVO_S1, SERVO_S2, SERVO_S3, SERVO_MOTOR,
    MOTOR_OFF_US, MOTOR_FULL_US, MOTOR_ESC_CHANNEL,
    SWASH_SERVOS,
    PWM_MIN, PWM_MAX, PWM_NEUTRAL,
    _COPTER_MODES, _FALLBACK_BAUDS,
    _AP_BASE_PARM_PATH, _RAWES_COMMON_PARM_PATH,
    _RUN_MODES, _WATCH_STREAMS,
)
from .hw import (
    _arm, _disarm, _sweep, _send_set_servo, _send_motor_test,
    _print_status, _ping_ports, _probe_port,
    _h3_forward_mix, _norm_to_pwm,
    _refresh_pole_pairs, _monitor_esc,
)
from .params import (
    _CONFIG_TARGET_PARAMS_ALL, _CONFIG_TARGET_PARAMS_COMMON,
    _list_scripts, _remove_script, _upload_script, _cmd_logs,
)
from .run import (
    _cmd_run, _observation_loop, _take_servo4, _safety_shutdown,
    _run_observation,
)
from .util import _RunLog, _parse_flags, _parse_kv_list, _log_path, _esc_check
from .watch import _cmd_watch


# ---------------------------------------------------------------------------
# Help text
# ---------------------------------------------------------------------------

_HELP = """
RAWES calibration tool.  Two long-running verbs (run, watch) handle all
time-bounded operations and log to simulation/logs/calibrate/*.csv.  The
rest are one-shot.

Long-running (always log; ESC or Ctrl-C aborts):
  run <name> [--duration N] [--trim K=V,...] [--gain K=V,...]
        Activate a Lua mode + arm via RAWES_ARM + stream observation rows.
        On exit (timer / ESC / Ctrl-C) every overridden param is restored.

        --duration N       run for N seconds; omit for unbounded (5-min ARM)
        --trim K=V,K=V     cyclic + collective held by the Lua; sent as
                           NAMED_VALUE_FLOAT to rawes.lua's MODE_PASSIVE /
                           MODE_YAW.  Repeatable.  **All values in degrees.**
                           The wire converts to radians automatically.
                             tlon  longitudinal cyclic [deg].  >0 = nose-down
                                   (forward-stick); <0 = nose-up.
                                   Range +/- H_CYC_MAX_cd/100 (passive: +/- 10).
                                   Typical bench: 0.5 .. 3 deg.
                             tlat  lateral cyclic [deg].  >0 = roll-right.
                                   Same range/limits as tlon.
                             col   IC collective [deg of blade pitch].
                                   COL_MIN_RAD (-0.28 rad = -16 deg)
                                   .. COL_MAX_RAD (+0.10 rad = +5.7 deg)
                                   linearly maps to RC3 PWM 1000..2000.
                                   Typical values:
                                     -5    zero-thrust / cruise neutral
                                     -8.6  modest negative (autorotation feed)
                                      0    neutral collective
                                     +3    light positive thrust
        --gain K=V,K=V     per-run AP param overrides.
                           Repeatable.  Run `run` (no args) for the per-mode
                           gain-key table.
        --exclude-saturate After the run, print an analysis report computed
                           ONLY from samples where the yaw loop was not
                           saturated (trim below YFF_MAX).
        --osc TARGET       Walk a sequence of trim setpoints, 5 s/step;
                           overrides --trim.  Targets:
                             all  full 13-step sweep through tlon/tlat/col
                                  extremes (~65 s)
                             s1   isolated S1 up/down (~25 s, 5 steps)
                             s2   isolated S2 up/down
                             s3   isolated S3 up/down (longitudinal axis)
                           The s1/s2/s3 sequences use mixer-isolated
                           combinations so the target servo dominates while
                           the other two stay near center.

                Modes (run with no args to see force_params per mode):
          passive   armed but quiet in GUIDED_NOGPS (matches the SITL passive
                    test).  Seeds the IC (RAWES_COL/RIC/PIC) and holds the IC
                    attitude via the GUIDED angle API; DDFP yaw motor stays
                    under AP + the Lua H_YAW_TRIM observer.
          steady    steady flight (alt hold + VZ PI collective)
          pumping   De Schutter pumping cycle
          landing   landing (reserved)

        Examples (IC angles in DEGREES):
          # Hold the level IC (roll=pitch=0, col=-8.6 deg) on the bench for 30 s
          run passive --duration 30 --trim col=-8.6

                    # Hold a fixed yaw IC as well
                    run passive --duration 20 --yaw 90 --trim col=-8.6

          # Hold a tilted IC: 3 deg roll, -25 deg pitch, IC collective
          run passive --duration 20 --roll 3 --pitch -25 --trim col=-8.6

          # Unbounded passive session (ESC to stop, 5-min RAWES_ARM fallback)
          run passive

          # Yaw tuning: gentler P, lower motor max, IC operating point loaded
          run yaw --duration 60 --gain p=0.015,i=0.005,imax=0.7,servo_max=1100 \\
                  --trim tlon=1.15,col=-8.6

          # Full oscillation sweep through every axis extreme (~65 s)
          run passive --osc all

          # Isolated S2 swashplate-servo test (~25 s, S2 dominant up/down)
          run passive --osc s2

  watch <stream> [--duration N]
        Read-only observation; no state change.  Default duration 10 s.
        streams: servos    SERVO_OUTPUT_RAW for ch1..8
                 esc       ESC_TELEMETRY (rpm/volt/current/temp)
                 text      STATUSTEXT only
                 attitude  ATTITUDE (roll/pitch/yaw + body rates)
                 power     BATTERY_STATUS / SYS_STATUS (vbat / current / W)

One-shot:
  status                          Vehicle / battery / EKF / servos / key params
  set <name> <value>              Write a parameter (read-back verified)
  get <name>                      Read a parameter
  swash <coll%> [lon%] [lat%]     H3-120 manual mixer (-100..+100 each)
  swash range <min> <max>         Set H_COL_MIN / H_COL_MAX (heli swash range)
  swash neutral [n]               Drive S1/S2/S3 (or n) to 1500 us
  swash info                      Print current swashplate geometry + factors
  servo <ch> <pwm>                Set channel ch to pwm directly
  servo sweep <ch> [--step-ms N]  Slowly sweep ch: 1500 -> 2000 -> 1000 -> 1500
  servo hold <ch> <pwm> [--duration N]  Arm, hold ch at pwm for N s
  motor <pwm_us> [--duration N]   Arm (RAWES_ARM) + drive the motor output at
                                  pwm_us for N s (default 5).  DShot ESC self-arms
                                  from idle -- no ESC pre-arm hold.
                                  pwm_us must be within [SERVO<motor>_MIN, _MAX];
                                  >5% of that range prompts unless --force.
                                  Logs to CSV like `run`.
  motor off                       motor -> idle (off) + disarm immediately
  arm [--duration N]              ACRO + RAWES_ARM (no Lua mode change)
  disarm                          Disarm vehicle
  script upload <file>            Upload .lua to /APM/scripts and restart engine
  script list                     List /APM/scripts
  script remove <name>            Remove from /APM/scripts
    config check [--all]            Diff params against defaults
                                                                    default: rawes_common_defaults.parm overrides only
                                                                    --all: copter-heli.parm + rawes_common_defaults.parm
    config fix [--all]              Write the DIFFs (same scope rules as check)
    config show/apply               Compatibility aliases for check/fix
  logs list                       List all dataflash logs on the FC (id / size)
  logs fetch [--id N] [--dir D]   Download a dataflash .BIN log (default: latest)
                                  --id N   specific log id; omit for latest
                                  --dir D  destination dir (default: simulation/logs/calibrate)
  reboot                          Reboot ArduPilot
  ping [baud]                     Scan COM ports for ArduPilot heartbeats
  help                            Show this list
  quit                            Exit (REPL only)
"""


# ---------------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------------

def _run_command(session: RawesGCS, tokens: list[str],
                 force: bool = False) -> bool:
    """
    Execute one calibration command.

    tokens : verb + arguments, e.g. ["run", "passive", "--duration", "30"]
    force  : skip interactive confirmation prompts (for CLI / scripted use)

    Returns True if the verb was recognised, False otherwise.
    """
    if not tokens:
        return True
    verb = tokens[0].lower()
    args = tokens[1:]

    if   verb == "status":   _print_status(session)
    elif verb == "ping":     _cmd_ping(args)
    elif verb == "reboot":   _cmd_reboot(session)
    elif verb == "disarm":   _disarm(session)
    elif verb == "arm":      _cmd_arm(session, args)
    elif verb == "set":      _cmd_set(session, args)
    elif verb == "get":      _cmd_get(session, args)
    elif verb == "swash":    _cmd_swash(session, args)
    elif verb == "servo":    _cmd_servo(session, args)
    elif verb == "motor":    _cmd_motor(session, args, force=force)
    elif verb == "run":      _cmd_run(session, args)
    elif verb == "logs":        _cmd_logs(session, args)
    elif verb == "watch":    _cmd_watch(session, args)
    elif verb == "script":   _cmd_script(session, args)
    elif verb == "config":   _cmd_config(session, args)
    elif verb == "help":     print(_HELP)
    else:
        return False
    return True


# ---------------------------------------------------------------------------
# One-shot verb implementations
# ---------------------------------------------------------------------------

def _cmd_ping(args: list[str]) -> None:
    """ping [baud]"""
    try:
        baud = int(args[0]) if args else 115200
    except ValueError:
        print("  Usage: ping [baud]"); return
    _ping_ports(baud=baud)


def _cmd_reboot(session: RawesGCS) -> None:
    print("  Sending reboot command ...")
    session._mav.mav.command_long_send(
        session._target_system, session._target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    print("  Pixhawk rebooting -- reconnect in ~5 s.")


def _cmd_set(session: RawesGCS, args: list[str]) -> None:
    """set <name> <value>"""
    if len(args) < 2:
        print("  Usage: set <name> <value>"); return
    name = args[0].upper()
    try:
        value = float(args[1])
    except ValueError:
        print("  Error: value must be a number"); return
    ok = session.set_param(name, value)
    if not ok:
        print(f"  [FAIL] {name}: no ACK within timeout"); return
    actual = session.get_param(name)
    if actual is None:
        print(f"  [WARN] {name}: set ACK'd but readback failed")
    elif abs(actual - value) > 1e-6:
        print(f"  [FAIL] {name}: wrote {value}, read back {actual}  (likely silently rejected)")
    else:
        print(f"  [OK]   {name} = {actual}")


def _cmd_get(session: RawesGCS, args: list[str]) -> None:
    """get <name>"""
    if not args:
        print("  Usage: get <name>"); return
    name = args[0].upper()
    v = session.get_param(name)
    if v is None:
        print(f"  [FAIL] {name}: not found")
    else:
        print(f"  {name} = {v}")


def _print_swash_layout(session: RawesGCS) -> None:
    """Print the current H3-120 swashplate geometry as read from the FC."""
    def g(name, default=None):
        v = session.get_param(name)
        return float(v) if v is not None else default

    sw_type   = g("H_SW_TYPE")
    sv1_pos   = g("H_SW_H3_SV1_POS")
    sv2_pos   = g("H_SW_H3_SV2_POS")
    sv3_pos   = g("H_SW_H3_SV3_POS")
    phang     = g("H_SW_H3_PHANG")
    ahrs_orn  = g("AHRS_ORIENTATION")
    col_min   = g("H_COL_MIN")
    col_max   = g("H_COL_MAX")
    col_mid   = g("H_COL_MID")
    cyc_max   = g("H_CYC_MAX")
    flybar    = g("H_FLYBAR_MODE")
    sv_man    = g("H_SV_MAN")

    def _quad(az):
        """Rough physical quadrant label from azimuth (deg, CCW from front)."""
        if az is None: return "?"
        az = ((az + 180.0) % 360.0) - 180.0   # -180..+180
        if   -30 <= az <=  30:  return "front"
        elif  30 <  az <  90:   return "front-left"
        elif  az >= 90 and az <= 150:  return "left-back"
        elif az > 150 or az < -150:    return "back"
        elif -90 <= az < -30:   return "front-right"
        else:                    return "right-back"

    def _factors(az):
        if az is None: return (None, None)
        a = math.radians(az)
        return (-math.sin(a), math.cos(a))   # (roll_factor, pitch_factor)

    print()
    print("RAWES H3-120 swashplate layout")
    print("==============================")
    print()
    print("ArduPilot params:")
    print(f"  H_SW_TYPE         = {sw_type!s:<8}  (3 = H3-120 generic)")
    print(f"  H_SW_H3_SV1_POS   = {sv1_pos!s:<8}  ({_quad(sv1_pos)})")
    print(f"  H_SW_H3_SV2_POS   = {sv2_pos!s:<8}  ({_quad(sv2_pos)})")
    print(f"  H_SW_H3_SV3_POS   = {sv3_pos!s:<8}  ({_quad(sv3_pos)})")
    print(f"  H_SW_H3_PHANG     = {phang!s:<8}  (deg of phase correction)")
    print(f"  AHRS_ORIENTATION  = {ahrs_orn!s:<8}  (0 = forward; 4 = YAW_180)")
    print()
    print(f"  H_COL_MIN = {col_min!s:<6}  H_COL_MAX = {col_max!s:<6}  "
          f"H_COL_MID = {col_mid!s:<6}")
    if cyc_max is not None:
        print(f"  H_CYC_MAX = {cyc_max:.0f}  cd  ({cyc_max/100:.1f} deg of swash tilt at full stick)")
    else:
        print(f"  H_CYC_MAX = ?")
    print(f"  H_FLYBAR_MODE = {flybar!s:<4}  (1 = ACRO passthrough, 0 = rate PID)")
    print(f"  H_SV_MAN      = {sv_man!s:<4}  (0 = AUTOMATED, !=0 = manual setup mode)")
    print()
    print("Servo factors  (AP mixer: roll = -sin(az), pitch = cos(az)):")
    print(f"  {'Servo':<6} {'Azimuth':>8}  {'Position':<14}  {'roll_f':>8}  {'pitch_f':>8}")
    for label, az in (("S1", sv1_pos), ("S2", sv2_pos), ("S3", sv3_pos)):
        rf, pf = _factors(az)
        rf_s = f"{rf:+.3f}" if rf is not None else "  n/a"
        pf_s = f"{pf:+.3f}" if pf is not None else "  n/a"
        az_s = f"{az:+.1f}" if az is not None else " n/a"
        print(f"  {label:<6} {az_s:>8}  {_quad(az):<14}  {rf_s:>8}  {pf_s:>8}")
    print()
    print("Layout (top view, looking down at the swashplate):")
    print()
    print("                FRONT (nose, +x)")
    print("                      ^")
    print("                      |")
    print("       SV2  *    [FC] *  SV1")
    print(f"     ({_fmt_az(sv2_pos)})         ({_fmt_az(sv1_pos)})")
    print(f"     {_quad(sv2_pos):<12}       {_quad(sv1_pos):<12}")
    print("                      |")
    print("                      *  SV3")
    print(f"                    ({_fmt_az(sv3_pos)})")
    print(f"                    {_quad(sv3_pos)}")
    print("                      v")
    print("                BACK (tail)")
    print()
    print("Sign convention (design/ardupilot_swashplate.md):")
    print("  tlat > 0 = roll right;  tlon > 0 = nose-DOWN disk;  col > 0 = positive thrust")
    print()
    # Live PWMs
    session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)
    srv = session._recv(type="SERVO_OUTPUT_RAW", blocking=True, timeout=2.0)
    if srv:
        s1 = getattr(srv, "servo1_raw", 0)
        s2 = getattr(srv, "servo2_raw", 0)
        s3 = getattr(srv, "servo3_raw", 0)
        smot = getattr(srv, f"servo{SERVO_MOTOR}_raw", 0)
        print(f"Current PWMs:  S1={s1} us  S2={s2} us  S3={s3} us  "
              f"S{SERVO_MOTOR}(motor)={smot} us")
    else:
        print("Current PWMs:  (no SERVO_OUTPUT_RAW received)")
    print()


def _fmt_az(az):
    if az is None:
        return "  ?  "
    return f"{az:+.0f} deg"


def _cmd_swash(session: RawesGCS, args: list[str]) -> None:
    """swash <coll%> [lon%] [lat%]
       swash range <min> <max>
       swash neutral [n]
       swash info"""
    if not args:
        print("  Usage: swash <coll%> [lon%] [lat%]")
        print("         swash range <min_us> <max_us>")
        print("         swash neutral [n]")
        print("         swash info")
        return
    sub = args[0].lower()
    if sub == "info":
        _print_swash_layout(session)
        return
    if sub == "range":
        if len(args) != 3:
            print("  Usage: swash range <min_us> <max_us>"); return
        try:
            lo = int(args[1]); hi = int(args[2])
        except ValueError:
            print("  Error: min and max must be integers"); return
        if not (800 <= lo < hi <= 2200):
            print(f"  Error: need 800 <= min < max <= 2200 (got {lo}..{hi})"); return
        for nm, val in (("H_COL_MIN", lo), ("H_COL_MAX", hi)):
            session.set_param(nm, float(val))
            actual = session.get_param(nm)
            tag = "[OK]  " if actual is not None and abs(actual - val) < 1.0 else "[FAIL]"
            print(f"  {tag} {nm} = {actual}")
        return
    if sub == "neutral":
        targets = list(SWASH_SERVOS)
        if len(args) >= 2:
            try:
                targets = [int(args[1])]
            except ValueError:
                print("  Usage: swash neutral [n]"); return
        for n in targets:
            _send_set_servo(session, n, PWM_NEUTRAL)
        print(f"  Output(s) {targets} -> {PWM_NEUTRAL} us")
        return
    # Positional: swash <coll%> [lon%] [lat%]
    try:
        coll = float(args[0]) / 100.0
        lon  = float(args[1]) / 100.0 if len(args) > 1 else 0.0
        lat  = float(args[2]) / 100.0 if len(args) > 2 else 0.0
    except ValueError:
        print("  Error: values must be numbers"); return
    s1n, s2n, s3n = _h3_forward_mix(coll, lon, lat)
    pwm1, pwm2, pwm3 = _norm_to_pwm(s1n), _norm_to_pwm(s2n), _norm_to_pwm(s3n)
    _send_set_servo(session, SERVO_S1, pwm1)
    _send_set_servo(session, SERVO_S2, pwm2)
    _send_set_servo(session, SERVO_S3, pwm3)
    print(f"  swash coll={coll*100:.0f}% lon={lon*100:.0f}% lat={lat*100:.0f}%")
    print(f"    S1={pwm1} us  S2={pwm2} us  S3={pwm3} us")


def _cmd_servo(session: RawesGCS, args: list[str]) -> None:
    """servo <ch> <pwm>
       servo sweep <ch> [--step-ms N]
       servo hold <ch> <pwm> [--duration N]"""
    if not args:
        print("  Usage: servo <ch> <pwm>")
        print("         servo sweep <ch> [--step-ms N]")
        print("         servo hold <ch> <pwm> [--duration N]")
        return
    sub = args[0].lower()
    if sub == "sweep":
        try:
            pos, flags = _parse_flags(args[1:], {"--step-ms": "int"})
        except ValueError as e:
            print(f"  Error: {e}"); return
        if len(pos) != 1:
            print("  Usage: servo sweep <ch> [--step-ms N]"); return
        try:
            ch = int(pos[0])
        except ValueError:
            print("  Error: ch must be an integer"); return
        step = flags.get("--step-ms", 5)
        _sweep(session, ch, step_ms=step)
        return
    if sub == "hold":
        try:
            pos, flags = _parse_flags(args[1:], {"--duration": "float"})
        except ValueError as e:
            print(f"  Error: {e}"); return
        if len(pos) != 2:
            print("  Usage: servo hold <ch> <pwm> [--duration N]"); return
        try:
            ch = int(pos[0]); pwm = int(pos[1])
        except ValueError:
            print("  Error: ch and pwm must be integers"); return
        if not (800 <= pwm <= PWM_MAX):
            print(f"  Error: pwm must be 800-{PWM_MAX}"); return
        duration = flags.get("--duration", 60.0)
        if not _arm(session, force=True):
            _safety_shutdown(session, skip_motor_off=(ch != SERVO_MOTOR))
            return
        print("  [OK] Armed.")
        deadline = time.monotonic() + duration
        try:
            while time.monotonic() < deadline:
                _send_set_servo(session, ch, pwm)
                time.sleep(0.1)
                if _esc_check():
                    print("\n  [ESC] abort"); break
        except KeyboardInterrupt:
            print()
        finally:
            _safety_shutdown(session, skip_motor_off=(ch != SERVO_MOTOR))
        return
    # Positional: servo <ch> <pwm>
    if len(args) < 2:
        print("  Usage: servo <ch> <pwm>"); return
    try:
        ch  = int(args[0]); pwm = int(args[1])
    except ValueError:
        print("  Error: ch and pwm must be integers"); return
    if not (1 <= ch <= 16):
        print("  Error: ch must be 1-16"); return
    if not (PWM_MIN <= pwm <= PWM_MAX):
        print(f"  Error: pwm must be {PWM_MIN}-{PWM_MAX}"); return
    _send_set_servo(session, ch, pwm)
    print(f"  Output {ch} -> {pwm} us")


def _cmd_motor(session: RawesGCS, args: list[str], *, force: bool) -> None:
    """motor <pwm_us> [--duration N]
       motor off

    Run-style lifecycle: arms via RAWES_ARM, releases the motor output from any
    AP mixer, then drives the motor output at the requested PWM for `duration`
    seconds while logging telemetry to
    simulation/logs/calibrate/motor_<pwm>_<ts>.csv.  On exit (timer / ESC /
    Ctrl-C / exception): motor -> idle (off), disarm, SERVO<motor>_FUNCTION restored."""
    if not args:
        print("  Usage: motor <pwm_us> [--duration N]  OR  motor off"); return
    if args[0].lower() in ("off", "stop"):
        # Immediate stop: force the motor to idle (off) and disarm.
        try:
            _send_set_servo(session, SERVO_MOTOR, MOTOR_OFF_US)
            print(f"  SERVO{SERVO_MOTOR} -> {MOTOR_OFF_US} us (motor off)")
        except Exception as e:
            print(f"  [WARN] failed to drive SERVO{SERVO_MOTOR} off: {e}")
        try:
            _disarm(session, timeout=5.0)
        except Exception as e:
            print(f"  [WARN] disarm failed: {e}")
        return
    try:
        pos, flags = _parse_flags(args, {"--duration": "float"})
    except ValueError as e:
        print(f"  Error: {e}"); return
    if not pos:
        print("  Usage: motor <pwm_us> [--duration N]"); return
    try:
        pwm = int(pos[0])
    except ValueError:
        print("  Error: pwm must be an integer (microseconds)"); return
    secs = flags.get("--duration", 5.0)

    # Read the live SERVO<motor>_MIN / SERVO<motor>_MAX caps so the prompt + clamp
    # warning reflect the per-bench safety cap (e.g. MAX=1100 during early tuning).
    mot_min_key = f"SERVO{SERVO_MOTOR}_MIN"
    mot_max_key = f"SERVO{SERVO_MOTOR}_MAX"
    s4_min = int(session.get_param(mot_min_key) or MOTOR_OFF_US)
    s4_max = int(session.get_param(mot_max_key) or MOTOR_FULL_US)
    if not (s4_min <= pwm <= s4_max):
        print(f"  Error: pwm must be in [{s4_min}, {s4_max}]  ({mot_min_key}/{mot_max_key}); got {pwm}")
        return
    if s4_max > s4_min:
        pct_for_prompt = (pwm - s4_min) / (s4_max - s4_min) * 100.0
    else:
        pct_for_prompt = 0.0
    if not force and pct_for_prompt > 5.0:
        confirm = input(
            f"  WARNING: SERVO{SERVO_MOTOR} = {pwm} us "
            f"({pct_for_prompt:.0f}% of [{s4_min},{s4_max}]) for {secs:.0f}s. "
            f"Confirm (y/N): ")
        if confirm.strip().lower() != "y":
            print("  Cancelled."); return

    # Same shuffle as `run yaw`: release the motor output from any AP mixer so our
    # DO_SET_SERVO commands win.
    saved_fn = _take_servo4(session)

    # MODE_PASSIVE / MODE_YAW would also drive SERVO4 -- force Lua to NONE.
    saved_overrides: dict[str, float] = {}
    saved_scr = session.get_param("SCR_USER6")
    if saved_scr is not None and int(saved_scr) != 0:
        saved_overrides["SCR_USER6"] = float(saved_scr)
        session.set_param("SCR_USER6", 0)
        print(f"  SCR_USER6 {int(saved_scr)} -> 0 (motor needs direct SERVO{SERVO_MOTOR} control)")

    if not _arm(session, force=True):
        _safety_shutdown(session, saved_servo4_fn=saved_fn, saved_overrides=saved_overrides)
        return
    print("  [OK] Armed.")

    # DShot self-arms from idle -- no ESC pre-arm hold needed.
    print(f"  Motor: SERVO{SERVO_MOTOR} = {pwm} us for {secs:.1f}s "
          f"(SERVO{SERVO_MOTOR} range [{s4_min}, {s4_max}]).")

    # Snapshot params for the log header
    meta = {
        "verb":             "motor",
        "pwm_us":           pwm,
        "duration_s":       secs,
        mot_min_key:        s4_min,
        mot_max_key:        s4_max,
        "run_start_local":  datetime.now().isoformat(timespec="seconds"),
        "run_start_utc":    datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    log = _RunLog.open("motor", f"{pwm}us", meta)
    print(f"  Logging to {log.path}")

    # On_tick refreshes the motor PWM ~twice per second while t_rel < secs.
    last_send = [-10.0]
    stopped   = [False]
    def on_tick(t_rel: float) -> None:
        target = pwm if t_rel < secs else MOTOR_OFF_US
        if t_rel >= secs and not stopped[0]:
            _send_set_servo(session, SERVO_MOTOR, MOTOR_OFF_US)
            stopped[0] = True
            last_send[0] = t_rel
            return
        if t_rel - last_send[0] >= 0.5:
            _send_set_servo(session, SERVO_MOTOR, target)
            last_send[0] = t_rel

    try:
        _run_observation(session, "motor", secs, log, on_tick=on_tick)
    finally:
        log.close()
        print(f"  Wrote {log.n_rows} rows to {log.path}")
        _safety_shutdown(session, saved_servo4_fn=saved_fn,
                         saved_overrides=saved_overrides)
    print("  Done.")


def _cmd_arm(session: RawesGCS, args: list[str]) -> None:
    """arm [--duration N]   -- ACRO + direct arm (no Lua mode change)"""
    try:
        pos, flags = _parse_flags(args, {"--duration": "float"})
    except ValueError as e:
        print(f"  Error: {e}"); return
    if pos:
        print("  Usage: arm [--duration N]  (use --duration, not positional)"); return
    print("  Setting ACRO mode ...")
    session.set_mode(1)
    if not _arm(session, force=True):
        print("  [WARN] Arm failed.")
    else:
        print("  [OK] Armed.")


def _cmd_script(session: RawesGCS, args: list[str]) -> None:
    """script upload <file>
       script list
       script remove <name>"""
    if not args:
        print("  Usage: script upload <file>")
        print("         script list")
        print("         script remove <name>")
        return
    sub = args[0].lower()
    if sub == "upload":
        if len(args) != 2:
            print("  Usage: script upload <file>"); return
        _upload_script(session, args[1])
        return
    if sub == "list":
        _list_scripts(session); return
    if sub == "remove":
        if len(args) != 2:
            print("  Usage: script remove <name>"); return
        _remove_script(session, args[1]); return
    print(f"  Unknown script subcommand {sub!r}  (valid: upload, list, remove)")


def _cmd_config(session: RawesGCS, args: list[str]) -> None:
    """config check|show
       config fix|apply"""
    if not args:
        print("  Usage: config check [--all]  OR  config fix [--all]"); return
    sub = args[0].lower()
    alias = {"show": "check", "apply": "fix"}
    sub = alias.get(sub, sub)
    if sub not in ("check", "fix"):
        print(f"  Unknown config subcommand {sub!r}  (valid: check, fix)"); return
    opt_tokens = args[1:]
    use_all = False
    for tok in opt_tokens:
        if tok == "--all":
            use_all = True
        else:
            print(f"  Unknown option {tok!r} (valid: --all)")
            return
    apply = (sub == "fix")
    target = _CONFIG_TARGET_PARAMS_ALL if use_all else _CONFIG_TARGET_PARAMS_COMMON
    scope = "all shared defaults" if use_all else "rawes_common overrides only"
    action = "Applying" if apply else "Preview -- 'config fix' to write"
    print(f"  RAWES config  [{action}]")
    print(f"  Scope: {scope}")
    print(f"  Source: {_AP_BASE_PARM_PATH}")
    print(f"          {_RAWES_COMMON_PARM_PATH}")
    print("          (SITL-only rawes_sitl_defaults.parm excluded)")
    print("          (hardware calibration params excluded)")
    print()
    print(f"  {'Parameter':<25} {'Expected':>8}  {'Actual':>10}  Status")
    print(f"  {'-'*25}  {'-'*8}  {'-'*10}  ------")
    any_diff = False
    any_fail = False
    for name in sorted(target):
        expected = target[name]
        actual = session.get_param(name)
        if actual is None:
            print(f"  {name:<25} {str(expected):>8}  {'N/A':>10}  [FAIL] not found")
            any_fail = True
        elif abs(actual - float(expected)) < 1e-4:
            print(f"  {name:<25} {str(expected):>8}  {actual:>10.4g}  OK")
        else:
            any_diff = True
            if apply:
                ok = session.set_param(name, float(expected))
                if ok:
                    print(f"  {name:<25} {str(expected):>8}  {actual:>10.4g}  -> SET")
                else:
                    print(f"  {name:<25} {str(expected):>8}  {actual:>10.4g}  [FAIL] no ACK")
                    any_fail = True
            else:
                print(f"  {name:<25} {str(expected):>8}  {actual:>10.4g}  DIFF")
    print()
    if apply and any_diff and not any_fail:
        print("  Done -- consider 'reboot' to apply any boot-time params.")
    elif any_fail:
        print("  Done with failures -- check above.")
    elif any_diff:
        print("  Done -- run 'config fix' to write the DIFFs.")
    else:
        print("  Done -- everything matches.")


# ---------------------------------------------------------------------------
# REPL
# ---------------------------------------------------------------------------

def _repl(session: RawesGCS) -> None:
    print("\nConnected. Type 'help' for commands, 'quit' to exit.\n")
    while True:
        try:
            raw = input("calibrate> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not raw:
            continue
        tokens = raw.split()
        cmd    = tokens[0].lower()
        if cmd in ("quit", "exit", "q"):
            break
        if cmd == "help":
            print(_HELP)
            continue
        if not _run_command(session, tokens, force=False):
            print(f"  Unknown command: {cmd!r}  (type 'help')")


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="RAWES calibration tool -- servo, motor, and Lua script management",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_HELP,
    )
    p.add_argument("--port", "-p", default=None,
                   help="Serial port (e.g. COM4), or 'sitl' / 'tcp:localhost:5760' for SITL")
    p.add_argument("--baud", "-b", "--rate", dest="baud", type=int, default=115200,
                   help="Baud rate (default: 115200)")
    p.add_argument("--force", "-f", action="store_true",
                   help="Skip confirmation prompts (for scripted/CI use)")
    p.add_argument("command", nargs="?", default=None,
                   help="Command to run non-interactively; omit for interactive REPL")
    p.add_argument("args", nargs=argparse.REMAINDER,
                   help="Arguments for the command")
    return p


# ---------------------------------------------------------------------------
# Connection helpers
# ---------------------------------------------------------------------------

def _resolve_port(port: "str | None", baud: int) -> tuple:
    """
    Return (port, baud) to use for the connection.

    TCP/UDP addresses and the 'sitl' shorthand bypass serial scanning entirely.
    If port is None, scans all COM ports and returns the first that gives a
    heartbeat (trying baud then _FALLBACK_BAUDS in order).
    If port is given, probes that port with baud fallbacks until a heartbeat
    is received, then returns the working (port, baud).
    Raises SystemExit if nothing responds.
    """
    # SITL shorthand and raw TCP/UDP strings go straight to pymavlink.
    if port is not None:
        if port == "sitl":
            print("Using SITL shorthand -> tcp:localhost:5760")
            return "tcp:localhost:5760", baud
        if port.startswith(("tcp:", "udp:", "udpin:", "tcpin:")):
            return port, baud
    try:
        import serial.tools.list_ports as _list_ports
    except ImportError:
        raise SystemExit("pyserial not installed -- cannot scan COM ports")

    if port is None:
        ports = sorted(_list_ports.comports(), key=lambda p: p.device)
        if not ports:
            raise SystemExit("No COM ports found.")
        candidates = [(info.device, (info.description or "").strip()) for info in ports]
        print(f"No port specified -- scanning {len(candidates)} COM port(s) ...")
    else:
        candidates = [(port, "")]

    all_bauds = [baud] + [b for b in _FALLBACK_BAUDS if b < baud]
    for dev, desc in candidates:
        label = f"{dev}  {desc}".strip()
        for try_baud in all_bauds:
            suffix = f" (fallback)" if try_baud != baud else ""
            print(f"  {label:<50} {try_baud} baud{suffix} ... ", end="", flush=True)
            ok, sysid = _probe_port(dev, try_baud, timeout=3.0)
            if ok:
                print(f"[OK] sysid={sysid}")
                return dev, try_baud
            print("--")

    tried = "/".join(str(b) for b in all_bauds)
    if port is None:
        raise SystemExit(f"No ArduPilot heartbeat on any COM port (tried {tried}).")
    raise SystemExit(f"No heartbeat from {port} at {tried}.")


def _connect(port: "str | None", baud: int) -> RawesGCS:
    port, baud = _resolve_port(port, baud)
    is_tcp = port.startswith(("tcp:", "udp:", "udpin:", "tcpin:"))
    if is_tcp:
        print(f"Connecting to {port} ...")
    else:
        print(f"Connecting to {port} at {baud} baud ...")
    session = RawesGCS(address=port, baud=baud, clock=WallClock())
    session.connect(timeout=15.0)
    print(f"Connected: sysid={session._target_system} compid={session._target_component}")
    session.start_heartbeat()
    session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 10)
    _refresh_pole_pairs(session)
    return session


def _extract_late_connection_flags(tokens: list[str],
                                   port: "str | None",
                                   baud: int) -> tuple[list[str], "str | None", int]:
    """Pull connection flags from command args so users can place them after verbs.

    Accepts: --port/-p <port>, --baud/-b/--rate <baud>
    Returns remaining tokens plus resolved (port, baud).
    """
    out: list[str] = []
    i = 0
    while i < len(tokens):
        t = tokens[i]
        if t in ("--port", "-p"):
            if i + 1 >= len(tokens):
                raise SystemExit("Missing value for --port")
            port = tokens[i + 1]
            i += 2
            continue
        if t in ("--baud", "-b", "--rate"):
            if i + 1 >= len(tokens):
                raise SystemExit(f"Missing value for {t}")
            try:
                baud = int(tokens[i + 1])
            except ValueError:
                raise SystemExit(f"Invalid baud value for {t}: {tokens[i + 1]!r}")
            i += 2
            continue
        out.append(t)
        i += 1
    return out, port, baud


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    args = _build_parser().parse_args()

    cmd_args = list(args.args)
    if args.command is not None:
        cmd_args, late_port, late_baud = _extract_late_connection_flags(
            cmd_args, args.port, args.baud)
        args.port = late_port
        args.baud = late_baud

    if args.command == "ping":
        _ping_ports(baud=args.baud)
        return

    session = _connect(args.port, args.baud)
    exit_code = 0
    try:
        if args.command:
            tokens = [args.command] + cmd_args
            ok = _run_command(session, tokens, force=args.force)
            if not ok:
                print(f"Unknown command: {args.command!r}")
                _build_parser().print_help()
                exit_code = 1
        else:
            _repl(session)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        session.close()
        print("Disconnected.")

    if exit_code:
        sys.exit(exit_code)
