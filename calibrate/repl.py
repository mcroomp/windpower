"""
calibrate/repl.py -- All _cmd_* handlers, REPL loop, CLI parser, main().
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from datetime import datetime, timezone
from typing import Protocol

from linkhub_client import mav_constants as mavlink
from linkhub_client.mav_constants import mavutil

from .constants import (
    LinkHubClient, WallClock, CommandLong, RequestDataStream,
    SERVO_S1, SERVO_S2, SERVO_S3, SERVO_MOTOR,
    MOTOR_OFF_US, MOTOR_FULL_US, MOTOR_ESC_CHANNEL,
    SWASH_SERVOS,
    PWM_MIN, PWM_MAX, PWM_NEUTRAL,
    _COPTER_MODES,
    _LOG_DIR,
    _AP_BASE_PARM_PATH, _RAWES_COMMON_PARM_PATH, _RAWES_HARDWARE_PARM_PATH,
    _AZ_S1, _AZ_S2, _AZ_S3,
    _RUN_MODES, _WATCH_STREAMS,
)
from .hw import (
    _arm, _disarm, _send_set_servo, _send_motor_test,
    _print_status,
    _h3_forward_mix, _norm_to_pwm,
    _release_servo_functions, _restore_servo_functions, _set_heli_servo_mode,
    _refresh_pole_pairs, _monitor_esc, _set_safe_off_state,
)
from .linkhub import ensure_linkhub
from .messages import read_one
from .params import (
    _config_target_params,
    _fetch_dataflash_log,
    _list_dataflash_logs,
    _list_scripts,
    _remove_script,
    _upload_script,
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
  run <name> [--duration N] [--trim K=V,...] [--force]
        Activate a Lua mode + arm via MAVLink + stream observation rows.
      On exit (timer / ESC / Ctrl-C) safety shutdown disarms and sets RAWES_MODE=0.

        --duration N       run for N seconds; omit for unbounded
        --force            bypass ArduPilot pre-arm checks on secured bench hardware
        --trim K=V,K=V     cyclic trim + IC thrust sent as NAMED_VALUE_FLOAT
                           to rawes.lua.  Repeatable.
                             tlon  longitudinal cyclic [deg].  >0 = nose-down
                                   (forward-stick); <0 = nose-up.
                                   Range +/- H_CYC_MAX_cd/100 (passive: +/- 10).
                                   Typical bench: 0.5 .. 3 deg.
                             tlat  lateral cyclic [deg].  >0 = roll-right.
                                   Same range/limits as tlon.
                             thr   IC thrust [0..1] (passive mode only; sent as
                                   RAWES_THR before arming).
                                   col_min=-0.28 rad (-16 deg) at thrust=0,
                                   col_max=+0.10 rad (+5.7 deg) at thrust=1.
                                   Typical values:
                                     0.342  autorotation feed (-8.6 deg equiv)
                                     0.507  zero-thrust neutral (-5 deg equiv)
                                     0.875  light positive thrust (+3 deg equiv)
        --exclude-saturate After the run, print an analysis report computed
                           ONLY from samples where the yaw loop was not
                           saturated (trim below YFF_MAX).
        --settle-rate-deg-s N
                           Maximum body rate allowed during passive capture
                           qualification (default 2.865 deg/s).
        --settle-time N    Continuous quiet time required before capture
                           (default 3 s).
        --settle-timeout N Maximum time to wait for passive qualification
                           before safe shutdown (default 15 s).
        --rotor-motor      Connect to the BLDC* Bluetooth rotor drive (passive only).
                           During the run: m starts/stops, [/] changes speed by 5,
                           and 0 emergency-stops. Exit always stops and disconnects.
        --rotor-speed N    Initial external rotor-drive speed, 0..100 (default 10).
        --rotor-direction D
                           External rotor-drive direction: cw (default) or ccw.
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

                Modes:
          acro-manual
                   ACRO flybar passthrough from normalized Lua controls.
                   Arrows set roll/pitch; -/= set collective; AP + Lua own yaw.
          passive   interactive GUIDED_NOGPS attitude hold. Ground qualifies a
                    quiet interval, then asks Lua to capture the quaternion;
                    arrows apply relative roll/pitch offsets by 5 deg, limited
                    to +/-30 deg; -/= change held thrust by 0.05; ,/. (or </>)
                    apply relative yaw by 5 deg.
          steady    steady flight (alt hold + VZ PI collective)
          pumping   De Schutter pumping cycle
          landing   landing (reserved)

        Examples:
          # Capture current attitude and interactively hold it for 30 s
          run passive --duration 30 --trim thr=0.342

          # Apply initial offsets relative to the captured quaternion
          run passive --duration 20 --roll 3 --pitch -25 --trim thr=0.342

          # Unbounded passive session (ESC or Ctrl-C to stop)
          run passive

          # Live yaw PID tuning during steady/pumping runs:
          #   q/a = P +/- 0.002,  w/s = I +/- 0.0005,  e/d = D +/- 0.001

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
  battery monitor off|on [type]   Disable monitoring; on defaults to type 4
  set <name> <value>              Write a parameter (read-back verified)
  get <name> [<name> ...]         Read one or more parameters
  swash <coll%> [lon%] [lat%]     HR3-120 physical mixer (-100..+100 each)
  swash range <min> <max>         Set H_COL_MIN / H_COL_MAX (heli swash range)
  swash neutral [n]               Drive S1/S2/S3 (or n) to 1500 us
  swash info                      Print current swashplate geometry + factors
    servo <ch> <pwm>                Set ch directly; disconnect/restore swash ch1-3
    servo mode <name|0..5> [--duration N]  Run a native H_SV_MAN mode
                                            oscillate requires --allow-full-range
    servo sweep [--duration N]      Safe raw-PWM sweep within H_COL_MIN/MAX
    swash fit-range <min> <max> --cyclic N --allow-full-range
                                    Fit/verify H_COL limits using native oscillation
    servo hold <ch> <pwm> [--duration N]  Hold ch; disconnect/restore swash ch1-3
  motor <pwm_us> [--duration N]   Arm via MAVLink + drive the motor output at
                                  pwm_us for N s (default 5).  DShot ESC self-arms
                                  from idle -- no ESC pre-arm hold.
                                  pwm_us must be within [SERVO<motor>_MIN, _MAX];
                                  >5% of that range prompts unless --force.
                                  Logs to CSV like `run`.
  motor off                       motor -> idle (off) + disarm immediately
  arm [--duration N] [--force]    Arm in ACRO, then disarm (default 5 s);
                                  --force bypasses ArduPilot pre-arm checks
  disarm                          Disarm vehicle
  script upload <file>            Upload .lua to /APM/scripts and restart engine
  script list                     List /APM/scripts
  script remove <name>            Remove from /APM/scripts
    config check [--all]            Diff params against hardware defaults
                                    default: common + physical-airframe overrides
                                    --all: also include copter-heli.parm baseline
    config fix [--all]              Write the DIFFs (same scope rules as check)
  logs list                       List all dataflash logs on the FC (id / size)
  logs fetch [--id N] [--dir D]   Download a dataflash .BIN log (default: latest)
                                  --id N   specific log id; omit for latest
                                  --dir D  destination dir (default: simulation/logs/calibrate)
  reboot                          Reboot ArduPilot
  help                            Show this list
  quit                            Exit (REPL only)
"""


class _BatteryParameterSession(Protocol):
    def set_param(self, name: str, value: float) -> bool: ...

    def get_param(self, name: str) -> float | None: ...


# ---------------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------------

def _run_command(session: LinkHubClient, tokens: list[str],
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
    if verb == "param" and args:
        verb, args = args[0].lower(), args[1:]

    if   verb == "status":   _print_status(session)
    elif verb == "reboot":   _cmd_reboot(session)
    elif verb == "disarm":   _cmd_disarm(session)
    elif verb == "arm":      _cmd_arm(session, args)
    elif verb == "battery":  _cmd_battery(session, args)
    elif verb == "set":      _cmd_set(session, args)
    elif verb == "get":      _cmd_get(session, args)
    elif verb == "swash":    _cmd_swash(session, args)
    elif verb == "servo":    _cmd_servo(session, args)
    elif verb == "motor":    _cmd_motor(session, args, force=force)
    elif verb == "run":      _cmd_run(session, args)
    elif verb == "watch":    _cmd_watch(session, args)
    elif verb == "script":   _cmd_script(session, args)
    elif verb == "logs":     _cmd_logs(session, args)
    elif verb == "config":   _cmd_config(session, args)
    elif verb == "help":     print(_HELP)
    else:
        return False
    return True


# ---------------------------------------------------------------------------
# One-shot verb implementations
# ---------------------------------------------------------------------------

def _cmd_reboot(session: LinkHubClient) -> None:
    print("  Sending reboot command ...")
    session.command(
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        [1.0],
    )


def _cmd_disarm(session: LinkHubClient) -> None:
    """Converge on canonical safe-off, forcing disarm only when necessary."""
    if _disarm(session):
        return
    print("  [WARN] Normal disarm failed; requesting forced disarm.")
    if not _disarm(session, force=True):
        print("  [FAIL] Could not confirm disarm; applying safe-off outputs anyway.")
        _set_safe_off_state(session)
    print("  Pixhawk rebooting -- reconnect in ~5 s.")
    print(
        "  [SAFETY] After reconnect, run disarm to restore "
        "ACRO / H_FLYBAR_MODE=1 / H_SV_MAN=0 safe-off."
    )


def _cmd_set(session: LinkHubClient, args: list[str]) -> None:
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


def _cmd_battery(session: _BatteryParameterSession, args: list[str]) -> None:
    """battery monitor off|on [type]; type 4 is analog voltage and current."""
    if not args or args[0].lower() != "monitor":
        print("  Usage: battery monitor off|on [type]")
        return
    if len(args) < 2 or len(args) > 3:
        print("  Usage: battery monitor off|on [type]")
        return

    action = args[1].lower()
    if action == "off" and len(args) == 2:
        monitor_type = 0
    elif action == "on":
        if len(args) == 2:
            monitor_type = 4
        else:
            try:
                monitor_type = int(args[2])
            except ValueError:
                print("  Error: battery monitor type must be an integer")
                return
            if not 1 <= monitor_type <= 32:
                print("  Error: enabled battery monitor type must be 1..32")
                return
    else:
        print("  Usage: battery monitor off|on [type]")
        return

    if not session.set_param("BATT_MONITOR", monitor_type):
        print("  [FAIL] BATT_MONITOR: no ACK within timeout")
        return
    actual = session.get_param("BATT_MONITOR")
    if actual is None:
        print("  [WARN] BATT_MONITOR set ACK'd but readback failed")
    elif round(actual) != monitor_type:
        print(
            f"  [FAIL] BATT_MONITOR: wrote {monitor_type}, "
            f"read back {actual} (likely silently rejected)"
        )
        return
    else:
        label = (
            "disabled"
            if monitor_type == 0
            else "analog voltage + current"
            if monitor_type == 4
            else "enabled"
        )
        print(f"  [OK] BATT_MONITOR = {monitor_type} ({label})")
    print("  [NOTE] ArduPilot marks BATT_MONITOR reboot-required; reboot to apply.")


def _cmd_get(session: LinkHubClient, args: list[str]) -> None:
    """get <name> [<name> ...]"""
    if not args:
        print("  Usage: get <name> [<name> ...]"); return
    for raw_name in args:
        name = raw_name.upper()
        v = session.get_param(name)
        if v is None:
            print(f"  [FAIL] {name}: not found")
        else:
            print(f"  {name} = {v}")


def _print_swash_layout(session: LinkHubClient) -> None:
    """Print the configured mixer and RAWES physical HR3-120 geometry."""
    def g(name, default=None):
        v = session.get_param(name)
        return float(v) if v is not None else default

    sw_type   = g("H_SW_TYPE")
    sv1_pos   = g("H_SW_H3_SV1_POS")
    sv2_pos   = g("H_SW_H3_SV2_POS")
    sv3_pos   = g("H_SW_H3_SV3_POS")
    phang     = g("H_SW_H3_PHANG")
    ahrs_orn  = g("AHRS_ORIENTATION")
    col_dir   = g("H_SW_COL_DIR")
    rev1      = g("SERVO1_REVERSED")
    rev2      = g("SERVO2_REVERSED")
    rev3      = g("SERVO3_REVERSED")
    col_min   = g("H_COL_MIN")
    col_max   = g("H_COL_MAX")
    col_zero  = g("H_COL_ZERO_THRST")
    col_hover = g("H_COL_HOVER")
    cyc_max   = g("H_CYC_MAX")
    flybar    = g("H_FLYBAR_MODE")
    sv_man    = g("H_SV_MAN")

    cursor = session.current_cursor()
    session.send_message(RequestDataStream(
        target_system=session._target_system,
        target_component=session._target_component,
        req_stream_id=mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        req_message_rate=10,
    ))
    srv, _ = read_one(session, cursor, "SERVO_OUTPUT_RAW", wait=2.0)
    pwm = {
        1: getattr(srv, "servo1_raw", 0) if srv else 0,
        2: getattr(srv, "servo2_raw", 0) if srv else 0,
        3: getattr(srv, "servo3_raw", 0) if srv else 0,
        SERVO_MOTOR: getattr(srv, f"servo{SERVO_MOTOR}_raw", 0) if srv else 0,
    }

    def _pwm_text(channel: int) -> str:
        value = pwm[channel]
        return f"{value} us" if value else "no data"

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
    physical_positions = (
        ("S1", math.degrees(_AZ_S1)),
        ("S2", math.degrees(_AZ_S2)),
        ("S3", math.degrees(_AZ_S3)),
    )

    print("RAWES HR3-120 swashplate layout")
    print("===============================")
    print()
    print("ArduPilot params:")
    print(f"  H_SW_TYPE         = {sw_type!s:<8}  (3 = H3-120; reversals implement HR3)")
    print(f"  H_SW_COL_DIR      = {col_dir!s:<8}  (1 required for RAWES HR3)")
    print(f"  SERVO1/2/3_REV    = {rev1!s}/{rev2!s}/{rev3!s}  (1/1/1 required)")
    print(f"  H_SW_H3_SV1_POS   = {sv1_pos!s:<8}  (unused when H_SW_TYPE=3)")
    print(f"  H_SW_H3_SV2_POS   = {sv2_pos!s:<8}  (unused when H_SW_TYPE=3)")
    print(f"  H_SW_H3_SV3_POS   = {sv3_pos!s:<8}  (unused when H_SW_TYPE=3)")
    print(f"  H_SW_H3_PHANG     = {phang!s:<8}  (deg of phase correction)")
    print(f"  AHRS_ORIENTATION  = {ahrs_orn!s:<8}  (0 = forward; 4 = YAW_180)")
    print()
    print(f"  H_COL_MIN         = {col_min!s:<8}  H_COL_MAX = {col_max!s}")
    print(f"  H_COL_ZERO_THRST  = {col_zero!s:<8}  deg")
    print(f"  H_COL_HOVER       = {col_hover!s:<8}  normalized collective")
    if cyc_max is not None:
        print(f"  H_CYC_MAX = {cyc_max:.0f}  cd  ({cyc_max/100:.1f} deg of swash tilt at full stick)")
    else:
        print(f"  H_CYC_MAX = ?")
    print(f"  H_FLYBAR_MODE = {flybar!s:<4}  (1 = ACRO passthrough, 0 = rate PID)")
    print(f"  H_SV_MAN      = {sv_man!s:<4}  (0 = AUTOMATED, !=0 = manual setup mode)")
    print()
    print("Servo factors  (AP mixer: roll = -sin(az), pitch = cos(az)):")
    print(f"  {'Servo':<6} {'Azimuth':>8}  {'Position':<14}  {'roll_f':>8}  {'pitch_f':>8}")
    for label, az in physical_positions:
        rf, pf = _factors(az)
        rf_s = f"{rf:+.3f}" if rf is not None else "  n/a"
        pf_s = f"{pf:+.3f}" if pf is not None else "  n/a"
        az_s = f"{az:+.1f}" if az is not None else " n/a"
        print(f"  {label:<6} {az_s:>8}  {_quad(az):<14}  {rf_s:>8}  {pf_s:>8}")
    print()
    print("Layout (top view, looking down at the swashplate):")
    print()
    print("                    FRONT / +X / TOWARD CG")
    print("                              ^")
    print("                              |")
    print("                         [ FC arrow ]")
    print("                              |")
    print("                    S3 - FRONT / ELEVATOR")
    print("                         MAIN OUT 3")
    print(f"                          [{_pwm_text(3)}]")
    print("                              o")
    print("                            /   \\")
    print("                           /  O  \\")
    print("                          /       \\")
    print("                         /         \\")
    print("                        o-----------o")
    print("              S2 - LEFT-REAR     S1 - RIGHT-REAR")
    print("                 MAIN OUT 2         MAIN OUT 1")
    print(f"                  [{_pwm_text(2)}]          [{_pwm_text(1)}]")
    print()
    print("                              |")
    print("                              v")
    print("                            REAR / -X")
    print()
    print("Sign convention (design/ardupilot_swashplate.md):")
    print("  tlat > 0 = roll right;  tlon > 0 = nose-DOWN disk;  col > 0 = positive thrust")
    print()
    if srv:
        print("Current outputs:")
        print(f"  S1 right-rear     OUT 1: {_pwm_text(1)}")
        print(f"  S2 left-rear      OUT 2: {_pwm_text(2)}")
        print(f"  S3 front/elevator OUT 3: {_pwm_text(3)}")
        print(f"  yaw motor         OUT {SERVO_MOTOR}: {_pwm_text(SERVO_MOTOR)}")
    else:
        print("Current PWMs:  (no SERVO_OUTPUT_RAW received)")
    print()


def _fmt_az(az):
    if az is None:
        return "  ?  "
    return f"{az:+.0f} deg"


def _cmd_swash(session: LinkHubClient, args: list[str]) -> None:
    """swash <coll%> [lon%] [lat%]
       swash range <min> <max>
       swash neutral [n]
       swash fit-range <min> <max> --cyclic N --allow-full-range
       swash info"""
    if not args:
        print("  Usage: swash <coll%> [lon%] [lat%]")
        print("         swash range <min_us> <max_us>")
        print("         swash neutral [n]")
        print("         swash fit-range <min_us> <max_us> --cyclic N --allow-full-range")
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
    if sub == "fit-range":
        try:
            pos, flags = _parse_flags(
                args[1:],
                {
                    "--cyclic": "int",
                    "--iterations": "int",
                    "--margin": "int",
                    "--allow-full-range": "bool",
                },
            )
        except ValueError as e:
            print(f"  Error: {e}"); return
        if len(pos) != 2 or "--cyclic" not in flags:
            print("  Usage: swash fit-range <min_us> <max_us> --cyclic N "
                  "--allow-full-range")
            return
        if not flags.get("--allow-full-range", False):
            print("  [REFUSED] fit-range runs native full-envelope oscillation.")
            print("  Disconnect the servos and add --allow-full-range.")
            return
        try:
            lo, hi = (int(value) for value in pos)
        except ValueError:
            print("  Error: min and max must be integers"); return
        cyclic = flags["--cyclic"]
        iterations = flags.get("--iterations", 3)
        margin = flags.get("--margin", 3)
        _fit_swash_range(session, lo, hi, cyclic, iterations, margin)
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


_SV_MAN_MODES = {
    "automated": 0,
    "passthrough": 1,
    "max": 2,
    "zero": 3,
    "min": 4,
    "oscillate": 5,
}


def _run_servo_mode(
    session: LinkHubClient,
    mode: int,
    duration: float,
) -> tuple[int, int] | None:
    cursor = session.current_cursor()
    heartbeat, cursor = read_one(
        session, cursor, "HEARTBEAT", wait=2.0,
    )
    if heartbeat is None:
        print("  [FAIL] no heartbeat; servo mode aborted")
        return None
    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("  [FAIL] vehicle is armed; native servo modes require disarmed")
        return None
    saved_mode = session.get_param("H_SV_MAN")
    if saved_mode is None:
        print("  [FAIL] H_SV_MAN unreadable; servo mode aborted")
        return None
    if not _set_heli_servo_mode(session, float(mode)):
        print(f"  [FAIL] could not set H_SV_MAN={mode}")
        return None
    mode_name = next(name for name, value in _SV_MAN_MODES.items() if value == mode)
    print(f"  H_SV_MAN={mode} ({mode_name}) running for {duration:.1f}s (ESC or Ctrl-C to stop)")
    session.send_message(RequestDataStream(
        target_system=session._target_system,
        target_component=session._target_component,
        req_stream_id=mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        req_message_rate=10,
    ))
    print("  t(s)    S1(us)  S2(us)  S3(us)")
    started = time.monotonic()
    deadline = time.monotonic() + duration
    next_print = started
    observed: list[int] = []
    completed = True
    try:
        while time.monotonic() < deadline:
            msg, cursor = read_one(
                session, cursor, "SERVO_OUTPUT_RAW", wait=0.2,
            )
            now = time.monotonic()
            if msg is not None:
                values = [
                    getattr(msg, "servo1_raw", 0),
                    getattr(msg, "servo2_raw", 0),
                    getattr(msg, "servo3_raw", 0),
                ]
                observed.extend(value for value in values if value > 0)
                if now >= next_print:
                    print(f"  {now - started:5.1f}  "
                          f"{values[0]:6d}  {values[1]:6d}  {values[2]:6d}")
                    next_print = now + 0.2
            if _esc_check():
                print("\n  [ESC] stopping servo mode")
                completed = False
                break
    except KeyboardInterrupt:
        print()
        completed = False
    finally:
        if _set_heli_servo_mode(session, saved_mode):
            print(f"  H_SV_MAN restored to {saved_mode:.0f}")
        else:
            print(f"  [FAIL] could not restore H_SV_MAN to {saved_mode:.0f}")
    if not completed or not observed:
        return None
    extrema = min(observed), max(observed)
    print(f"  Observed envelope: {extrema[0]}..{extrema[1]} us")
    return extrema


def _set_param_verified(session: LinkHubClient, name: str, value: int) -> bool:
    if not session.set_param(name, float(value)):
        print(f"  [FAIL] {name}: no ACK")
        return False
    actual = session.get_param(name)
    ok = actual is not None and abs(actual - value) < 1.0
    print(f"  {'[OK]  ' if ok else '[FAIL]'} {name} = {actual}")
    return ok


def _fit_swash_range(
    session: LinkHubClient,
    lo: int,
    hi: int,
    cyclic: int,
    iterations: int,
    margin: int,
) -> None:
    if not (800 <= lo < hi <= 2200):
        print(f"  Error: need 800 <= min < max <= 2200 (got {lo}..{hi})")
        return
    if not (0 <= cyclic <= 4500):
        print("  Error: cyclic must be in 0..4500")
        return
    if not (1 <= iterations <= 5):
        print("  Error: iterations must be in 1..5")
        return
    if margin < 0 or 2 * margin >= hi - lo:
        print("  Error: margin must be non-negative and less than half the range")
        return

    trim = round((lo + hi) / 2)
    initial = {
        "SERVO1_TRIM": trim,
        "SERVO2_TRIM": trim,
        "SERVO3_TRIM": trim,
        "H_CYC_MAX": cyclic,
    }
    if not all(_set_param_verified(session, name, value) for name, value in initial.items()):
        print("  [FAIL] setup write failed; fit aborted")
        return

    target_lo, target_hi = lo + margin, hi - margin
    print(f"  Target final envelope: {target_lo}..{target_hi} us "
          f"({margin} us margin)")
    for attempt in range(1, iterations + 1):
        print(f"\n  Fit iteration {attempt}/{iterations}")
        extrema = _run_servo_mode(session, _SV_MAN_MODES["oscillate"], 12.0)
        if extrema is None:
            print("  [FAIL] incomplete oscillation; fit aborted")
            return
        observed_lo, observed_hi = extrema
        if observed_lo >= target_lo and observed_hi <= target_hi:
            print(f"  [PASS] configured envelope is within {lo}..{hi} us")
            for name in ("SERVO1_TRIM", "H_COL_MIN", "H_COL_MAX", "H_CYC_MAX"):
                value = session.get_param(name)
                print(f"    {name} = {value}")
            return

        col_min = session.get_param("H_COL_MIN")
        col_max = session.get_param("H_COL_MAX")
        if col_min is None or col_max is None:
            print("  [FAIL] H_COL_MIN/MAX readback failed")
            return
        next_min = round(col_min + target_lo - observed_lo)
        next_max = round(col_max + target_hi - observed_hi)
        if next_min >= next_max:
            print("  [FAIL] requested cyclic leaves no usable collective range")
            return
        print(f"  Adjusting collective limits: {round(col_min)}..{round(col_max)} "
              f"-> {next_min}..{next_max}")
        if not (
            _set_param_verified(session, "H_COL_MIN", next_min)
            and _set_param_verified(session, "H_COL_MAX", next_max)
        ):
            print("  [FAIL] collective-limit write failed")
            return

    print("  [FAIL] fit did not converge within the requested iterations")


def _bounded_swash_waypoints(lo: int, neutral: int, hi: int) -> tuple[tuple[int, int, int], ...]:
    """Safe HR3 setup positions, all bounded by the measured servo envelope."""
    return (
        (neutral, neutral, neutral),
        (lo, lo, lo),
        (neutral, neutral, neutral),
        (hi, hi, hi),
        (neutral, neutral, neutral),
        (hi, hi, lo),
        (lo, lo, hi),
        (neutral, neutral, neutral),
        (hi, lo, neutral),
        (lo, hi, neutral),
        (neutral, neutral, neutral),
    )


def _run_bounded_swash_sweep(session: LinkHubClient, duration: float) -> None:
    cursor = session.current_cursor()
    heartbeat, cursor = read_one(
        session, cursor, "HEARTBEAT", wait=2.0,
    )
    if heartbeat is None:
        print("  [FAIL] no heartbeat; swash sweep aborted")
        return
    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("  [FAIL] vehicle is armed; swash sweep requires disarmed")
        return

    lo = session.get_param("H_COL_MIN")
    hi = session.get_param("H_COL_MAX")
    if lo is None or hi is None:
        print("  [FAIL] H_COL_MIN/MAX unreadable; swash sweep aborted")
        return
    lo, hi = round(lo), round(hi)
    if not lo <= PWM_NEUTRAL <= hi:
        print(f"  [FAIL] neutral {PWM_NEUTRAL} us is outside {lo}..{hi} us")
        return

    saved_functions = _release_servo_functions(session, SWASH_SERVOS)
    if saved_functions is None:
        return

    waypoints = _bounded_swash_waypoints(lo, PWM_NEUTRAL, hi)
    segment_s = duration / (len(waypoints) - 1)
    started = time.monotonic()
    deadline = started + duration
    print(f"  Safe HR3 sweep: {lo}..{hi} us for {duration:.1f}s (ESC or Ctrl-C to stop)")
    print("  t(s)    S1(us)  S2(us)  S3(us)")
    session.send_message(RequestDataStream(
        target_system=session._target_system,
        target_component=session._target_component,
        req_stream_id=mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        req_message_rate=10,
    ))
    next_print = started
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            progress = min((now - started) / segment_s, len(waypoints) - 1)
            index = min(int(progress), len(waypoints) - 2)
            blend = progress - index
            start = waypoints[index]
            end = waypoints[index + 1]
            targets = tuple(round(a + blend * (b - a)) for a, b in zip(start, end))
            for output, pwm in zip(SWASH_SERVOS, targets):
                _send_set_servo(session, output, pwm)

            msg, cursor = read_one(
                session, cursor, "SERVO_OUTPUT_RAW", wait=0.08,
            )
            if msg is not None and now >= next_print:
                values = tuple(getattr(msg, f"servo{output}_raw", 0) for output in SWASH_SERVOS)
                status = "OK" if all(lo <= value <= hi for value in values) else "OUT OF RANGE"
                print(f"  {now - started:5.1f}  {values[0]:6d}  {values[1]:6d}  {values[2]:6d}  {status}")
                next_print = now + 0.2
            if _esc_check():
                print("\n  [ESC] stopping swash sweep")
                break
    except KeyboardInterrupt:
        print()
    finally:
        try:
            for output in SWASH_SERVOS:
                _send_set_servo(session, output, PWM_NEUTRAL)
        finally:
            _restore_servo_functions(session, saved_functions)


def _cmd_servo(session: LinkHubClient, args: list[str]) -> None:
    """servo <ch> <pwm>
       servo mode <name|0..5> [--duration N]
       servo sweep [--duration N]
       servo hold <ch> <pwm> [--duration N]"""
    if not args:
        print("  Usage: servo <ch> <pwm>")
        print("         servo mode <automated|passthrough|max|zero|min|oscillate> [--duration N]")
        print("         servo sweep [--duration N]")
        print("         servo hold <ch> <pwm> [--duration N]")
        return
    sub = args[0].lower()
    if sub in ("mode", "sweep"):
        try:
            mode_args = args[1:]
            pos, flags = _parse_flags(
                mode_args,
                {"--duration": "float", "--allow-full-range": "bool"},
            )
        except ValueError as e:
            print(f"  Error: {e}"); return
        if sub == "sweep":
            if pos:
                print("  Usage: servo sweep [--duration N]"); return
            duration = flags.get("--duration", 12.0)
            if duration <= 0:
                print("  Error: duration must be greater than zero"); return
            _run_bounded_swash_sweep(session, duration)
            return
        else:
            if len(pos) != 1:
                print("  Usage: servo mode <automated|passthrough|max|zero|min|oscillate> [--duration N]")
                return
            mode_arg = pos[0].lower()
            if mode_arg in _SV_MAN_MODES:
                mode = _SV_MAN_MODES[mode_arg]
            else:
                try:
                    mode = int(mode_arg)
                except ValueError:
                    mode = -1
                if mode not in _SV_MAN_MODES.values():
                    print(f"  Error: unknown servo mode {pos[0]!r}; expected a name or 0..5")
                    return
            if mode == _SV_MAN_MODES["oscillate"]:
                if not flags.get("--allow-full-range", False):
                    print("  [REFUSED] native oscillate exercises the full configured")
                    print("  swash envelope. Disconnect the servos, then explicitly use")
                    print("  'servo mode oscillate --allow-full-range'.")
                    return
        duration = flags.get("--duration", 12.0 if mode == 5 else 10.0)
        if duration <= 0:
            print("  Error: duration must be greater than zero"); return
        _run_servo_mode(session, mode, duration)
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
        outputs_to_release = tuple(dict.fromkeys((*SWASH_SERVOS, ch)))
        saved_functions = _release_servo_functions(session, outputs_to_release)
        if saved_functions is None:
            return
        if ch not in SWASH_SERVOS:
            if not _arm(session, force=True):
                _restore_servo_functions(session, saved_functions)
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
            if ch in SWASH_SERVOS:
                try:
                    _send_set_servo(session, ch, PWM_NEUTRAL)
                finally:
                    _restore_servo_functions(session, saved_functions)
            else:
                try:
                    _safety_shutdown(session, skip_motor_off=(ch != SERVO_MOTOR))
                finally:
                    _restore_servo_functions(
                        session,
                        saved_functions,
                        exclude_outputs=(SERVO_MOTOR,),
                    )
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
    outputs_to_release = tuple(dict.fromkeys((*SWASH_SERVOS, ch)))
    saved_functions = _release_servo_functions(session, outputs_to_release)
    if saved_functions is None:
        return
    try:
        _send_set_servo(session, ch, pwm)
        print(f"  Output {ch} -> {pwm} us")
    finally:
        _restore_servo_functions(session, saved_functions)


def _cmd_motor(session: LinkHubClient, args: list[str], *, force: bool) -> None:
    """motor <pwm_us> [--duration N]
       motor off

    Run-style lifecycle: arms via MAVLink, releases the motor output from any
    AP mixer, then drives the motor output at the requested PWM for `duration`
    seconds while logging telemetry to
    simulation/logs/calibrate/motor_<pwm>_<ts>.csv.  On exit (timer / ESC /
    Ctrl-C / exception): motor -> idle, disarm, and leave the motor disconnected."""
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

    # Same shuffle as `run`: release the motor output from any AP mixer so our
    # DO_SET_SERVO commands win.
    _take_servo4(session)

    # MODE_PASSIVE / MODE_YAW would also drive SERVO4 -- force Lua to NONE.
    saved_scr = session.get_param("RAWES_MODE")
    if saved_scr is not None and int(saved_scr) != 0:
        session.set_param("RAWES_MODE", 0)
        print(f"  RAWES_MODE {int(saved_scr)} -> 0 (motor needs direct SERVO{SERVO_MOTOR} control)")

    if not _arm(session, force=True):
        _safety_shutdown(session)
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
        _safety_shutdown(session)
    print("  Done.")


def _cmd_arm(session: LinkHubClient, args: list[str]) -> None:
    """Arm normally for a bounded interval, then return to safe-off."""
    try:
        pos, flags = _parse_flags(
            args,
            {"--duration": "float", "--force": "bool"},
        )
    except ValueError as e:
        print(f"  Error: {e}"); return
    if pos:
        print(
            "  Usage: arm [--duration N] [--force]"
            "  (use --duration, not positional)"
        )
        return
    duration = float(flags.get("--duration", 5.0))
    force = bool(flags.get("--force", False))
    if duration <= 0:
        print("  Error: --duration must be positive")
        return
    if force:
        print("  [WARN] Force-arm enabled: ArduPilot pre-arm checks are bypassed.")
    print("  Setting ACRO mode ...")
    session.set_mode(1)
    if not _arm(session, force=force):
        print("  [WARN] Arm failed.")
        _set_safe_off_state(session)
        return
    print(f"  [OK] Armed for {duration:g} s.")
    try:
        time.sleep(duration)
    finally:
        if not _disarm(session):
            print("  [WARN] Normal disarm failed; requesting forced disarm.")
            _disarm(session, force=True)


def _cmd_script(session: LinkHubClient, args: list[str]) -> None:
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


def _cmd_logs(session: LinkHubClient, args: list[str]) -> None:
    if not args or args[0].lower() not in {"list", "fetch"}:
        print("  Usage: logs list")
        print("         logs fetch [--id N] [--dir D]")
        return
    if args[0].lower() == "list":
        if len(args) != 1:
            print("  Usage: logs list")
            return
        _list_dataflash_logs(session)
        return

    log_id: int | None = None
    directory = _LOG_DIR
    index = 1
    try:
        while index < len(args):
            option = args[index]
            if option == "--id" and index + 1 < len(args):
                log_id = int(args[index + 1])
                index += 2
            elif option == "--dir" and index + 1 < len(args):
                directory = args[index + 1]
                index += 2
            else:
                raise ValueError(f"unknown or incomplete option {option!r}")
    except ValueError as exc:
        print(f"  Error: {exc}")
        print("  Usage: logs fetch [--id N] [--dir D]")
        return
    _fetch_dataflash_log(session, log_id=log_id, directory=directory)


def _cmd_config(session: LinkHubClient, args: list[str]) -> None:
    """config check|fix"""
    if not args:
        print("  Usage: config check [--all]  OR  config fix [--all]"); return
    sub = args[0].lower()
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
    target = _config_target_params(use_all=use_all)
    scope = (
        "all shared and hardware defaults"
        if use_all
        else "RAWES common and hardware overrides"
    )
    action = "Applying" if apply else "Preview -- 'config fix' to write"
    print(f"  RAWES config  [{action}]")
    print(f"  Scope: {scope}")
    print(f"  Source: {_AP_BASE_PARM_PATH}")
    print(f"          {_RAWES_COMMON_PARM_PATH}")
    print(f"          {_RAWES_HARDWARE_PARM_PATH}")
    print("          (SITL-only rawes_sitl_defaults.parm excluded)")
    print()
    print(f"  {'Parameter':<25} {'Expected':>8}  {'Actual':>10}  Status")
    print(f"  {'-'*25}  {'-'*8}  {'-'*10}  ------")
    any_diff = False
    any_fail = False
    try:
        current = session.fetch_all_param_records(timeout=30.0)
    except RuntimeError as exc:
        print(f"  [FAIL] Could not fetch parameters: {exc}")
        return

    differences = [
        {
            "name": name,
            "value": float(target[name]),
            "type": int(current[name]["type"]),
        }
        for name in sorted(target)
        if name in current
        and abs(float(current[name]["value"]) - float(target[name])) >= 1e-4
    ]
    updated: dict[str, dict[str, object]] = {}
    if apply and differences:
        try:
            session.set_params(differences, timeout=15.0)
            updated = session.fetch_all_param_records(timeout=30.0)
        except RuntimeError as exc:
            print(f"  [FAIL] Batch update failed: {exc}")
            return

    for name in sorted(target):
        expected = target[name]
        record = current.get(name)
        if record is None:
            print(f"  {name:<25} {str(expected):>8}  {'N/A':>10}  [FAIL] not found")
            any_fail = True
            continue

        actual = float(record["value"])
        if abs(actual - float(expected)) < 1e-4:
            print(f"  {name:<25} {str(expected):>8}  {actual:>10.4g}  OK")
        else:
            any_diff = True
            if apply:
                verified = updated.get(name)
                if (
                    verified is not None
                    and abs(float(verified["value"]) - float(expected)) < 1e-4
                ):
                    print(f"  {name:<25} {str(expected):>8}  {actual:>10.4g}  -> SET")
                else:
                    print(
                        f"  {name:<25} {str(expected):>8}  {actual:>10.4g}"
                        "  [FAIL] verification mismatch"
                    )
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

def _repl(session: LinkHubClient) -> None:
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
    p.add_argument(
        "--server",
        default="http://127.0.0.1:8999",
        help="LinkHub base URL (default: http://127.0.0.1:8999)",
    )
    p.add_argument(
        "--connection",
        help=(
            "restrict LinkHub discovery to one serial port (for example COM4); "
            "omit to auto-scan every port"
        ),
    )
    p.add_argument(
        "--baud",
        type=int,
        help="restrict LinkHub discovery to one baud rate; omit to try standard rates",
    )
    p.add_argument("--motor-name-prefix", help="optional Bluetooth motor name prefix")
    p.add_argument("--force", "-f", action="store_true",
                   help="Skip confirmation prompts (for scripted/CI use)")
    p.add_argument("command", nargs="?", default=None,
                   help="Command to run non-interactively; omit for interactive REPL")
    p.add_argument("args", nargs=argparse.REMAINDER,
                   help="Arguments for the command")
    return p


# ---------------------------------------------------------------------------
# Connection helper
# ---------------------------------------------------------------------------

def _connect(server: str) -> LinkHubClient:
    print(f"Connecting to LinkHub at {server} ...")
    session = LinkHubClient(address=server)
    session.connect(timeout=15.0)
    print(f"Connected: sysid={session._target_system} compid={session._target_component}")
    session.send_message(RequestDataStream(
        target_system=session._target_system,
        target_component=session._target_component,
        req_stream_id=mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER,
        req_message_rate=10,
    ))
    _refresh_pole_pairs(session)
    return session


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    args = _build_parser().parse_args()

    cmd_args = list(args.args)
    exit_code = 0
    with ensure_linkhub(
        args.server,
        connection=args.connection,
        baud=args.baud,
        motor_name_prefix=args.motor_name_prefix,
    ):
        session = _connect(args.server)
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
