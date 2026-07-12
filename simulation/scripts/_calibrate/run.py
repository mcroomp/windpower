"""
_calibrate/run.py -- Observation loop engine, run command, oscillate.
"""
from __future__ import annotations

import csv
import math
import os
import time
from datetime import datetime, timezone

from pymavlink import mavutil

# msvcrt is Windows stdlib -- used for non-blocking ESC-key abort.
# Falls back to a stub on non-Windows so the rest of the module still imports.
try:
    import msvcrt
except ImportError:
    class _MsvcrtStub:
        def kbhit(self):    return False
        def getch(self):    return b""
    msvcrt = _MsvcrtStub()  # type: ignore[assignment]

from .constants import (
    RawesGCS,
    SERVO_MOTOR, MOTOR_OFF_US, MOTOR_ESC_CHANNEL,
    _ESC_TELEM_MSGS,
    _RUN_MODES, _TRIM_NVF, _PASSIVE_IC_COL_DEG,
    _AP_YAW_ZERO_PARAMS, _OSCILLATE_TARGETS, _OSCILLATE_STEP_S,
    _COPTER_MODES, _LOG_DIR,
)
from .hw import (
    _arm, _disarm, _send_set_servo,
    _esc_telem_msg_for_channel, _esc_erpm, _rpm_triplet,
)
from .util import (
    _fmt, _log_path, _parse_kv_list, _parse_flags,
    _RunLog, _esc_check, _poll_keys,
)


# ---------------------------------------------------------------------------
# Logging helpers
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Arm / pre-run helpers
# ---------------------------------------------------------------------------

def _wait_for_armed(session: RawesGCS, timeout_s: float = 15.0) -> bool:
    """Block until armed heartbeat or timeout.  Prints STATUSTEXT inline."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = session._recv(type=["HEARTBEAT", "STATUSTEXT"],
                            blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"  [FC] {msg.text.rstrip()}")
        elif msg.get_type() == "HEARTBEAT":
            if bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
    return False


def _take_servo4(session: RawesGCS) -> "float | None":
    """Set SERVO<motor>_FUNCTION=0 (release from DDFP).  Returns saved value for restore."""
    servo_key = f"SERVO{SERVO_MOTOR}_FUNCTION"
    saved = session.get_param(servo_key)
    if saved is not None and saved != 0:
        session.set_param(servo_key, 0)
        print(f"  {servo_key} {saved:.0f} -> 0 (released from DDFP)")
        return float(saved)
    return None


def _ensure_passive_tail_setup(session: RawesGCS) -> None:
    """Ensure passive mode uses AP-owned DDFP tail control on channel 4."""
    expected_tail = 3.0
    tail = session.get_param("H_TAIL_TYPE")
    if tail is None:
        print("  [WARN] H_TAIL_TYPE unreadable; cannot enforce passive tail setup")
    elif abs(float(tail) - expected_tail) > 1e-4:
        ok = session.set_param("H_TAIL_TYPE", expected_tail)
        tag = "[OK]" if ok else "[FAIL]"
        print(f"  {tag} H_TAIL_TYPE: {tail:.6g} -> {expected_tail:.0f} (passive expects DDFP CW)")

    s4f = session.get_param(f"SERVO{SERVO_MOTOR}_FUNCTION")
    if s4f is None:
        print(f"  [WARN] SERVO{SERVO_MOTOR}_FUNCTION unreadable; cannot verify motor-channel ownership")
    elif int(round(float(s4f))) == 0:
        print(f"  [WARN] SERVO{SERVO_MOTOR}_FUNCTION=0 (output {SERVO_MOTOR} released). "
              "Passive expects AP-owned tail output.")


def _restore_servo4(session: RawesGCS, saved: "float | None") -> None:
    if saved is None:
        return
    servo_key = f"SERVO{SERVO_MOTOR}_FUNCTION"
    try:
        session.set_param(servo_key, saved)
        print(f"  [SAFETY] {servo_key} restored to {saved:.0f}")
    except Exception as e:
        print(f"  [SAFETY] failed to restore {servo_key}: {e}")


def _safety_shutdown(session: RawesGCS, *,
                     saved_servo4_fn: "float | None" = None,
                     saved_overrides: "dict[str, float] | None" = None,
                     skip_motor_off: bool = False) -> None:
    """Unified post-run shutdown.  Order: stop Lua -> wait -> motor off ->
    disarm -> restore SERVO4_FUNCTION -> restore param overrides.  Every step
    is best-effort: one failure does not skip the next."""
    print("  [SAFETY] shutting down ...")
    try:
        session.set_param("RAWES_MODE", 0)
        print("  [SAFETY] RAWES_MODE -> 0 (Lua mode none)")
    except Exception as e:
        print(f"  [SAFETY] failed to set RAWES_MODE=0: {e}")
    time.sleep(0.30)   # let SRV_Channels override timeout expire
    if not skip_motor_off:
        try:
            _send_set_servo(session, SERVO_MOTOR, MOTOR_OFF_US)
            print(f"  [SAFETY] SERVO{SERVO_MOTOR} -> {MOTOR_OFF_US} us (motor off)")
        except Exception as e:
            print(f"  [SAFETY] failed to drive SERVO{SERVO_MOTOR} off: {e}")
    try:
        if not _disarm(session, timeout=5.0):
            print("  [SAFETY] disarm not confirmed within 5 s -- retrying force-disarm")
            if not _disarm(session, timeout=5.0, force=True):
                print("  [SAFETY] force-disarm not confirmed")
    except Exception as e:
        print(f"  [SAFETY] disarm command failed: {e}")
    _restore_servo4(session, saved_servo4_fn)
    for param, orig in (saved_overrides or {}).items():
        try:
            session.set_param(param, orig)
            print(f"  [SAFETY] {param} restored to {orig:.6g}")
        except Exception as e:
            print(f"  [SAFETY] failed to restore {param}: {e}")


# ---------------------------------------------------------------------------
# Key polling
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Shared observation loop engine
# ---------------------------------------------------------------------------

def _observation_loop(session: RawesGCS, *,
                      duration_s: "float | None",
                      msg_types: list[str],
                      streams: list[tuple],
                      handle_msg,
                      render_row,
                      header_cols: list[str],
                      log: "_RunLog | None" = None,
                      print_period_s: float = 1.0,
                      header_print_cols: "list[str] | None" = None,
                      on_tick=None,
                      suppress_status: bool = False,
                      key_handler=None,
                      setup_hook=None,
                      ) -> tuple[int, bool]:
    """Run the standard observation loop.

    handle_msg(state, msg, t_rel) -> updates state dict in place.
    render_row(state, t_rel)      -> list of CSV values (None entries -> blank).
    header_cols                   -> CSV column names.
    header_print_cols (optional)  -> if set, used for the live console table
                                     header.  Defaults to header_cols.
    on_tick(t_rel)    (optional)  -> called once per loop iteration; useful for
                                     scheduled side-effects (e.g. oscillating
                                     trim NVFs).

    Returns (n_rows, aborted).
    """
    for s_id, hz in streams:
        session.request_stream(s_id, hz)

    if setup_hook is not None:
        setup_hook()

    if log is not None:
        log.write_header(header_cols)

    print_hdr = header_print_cols if header_print_cols is not None else header_cols
    widths = [max(6, len(c) + 1) for c in print_hdr]
    print("  " + "  ".join(f"{c:>{w}}" for c, w in zip(print_hdr, widths)))
    print("  " + "  ".join("-" * w for w in widths))

    state = {"armed": True, "pending_text": []}
    t0 = time.monotonic()
    deadline = (t0 + duration_s + 5.0) if duration_s else None
    last_print = -1.0
    aborted = False
    print("  Press ESC (or Ctrl-C) to abort.")
    try:
        while True:
            if deadline and time.monotonic() >= deadline:
                break
            keys = _poll_keys()
            if b"\x1b" in keys:
                aborted = True
                print("\n  [ESC] abort -- running safety shutdown ...")
                break
            if key_handler is not None:
                for k in keys:
                    key_handler(k)
            msg = session._recv(type=msg_types, blocking=True, timeout=0.1)
            t_rel = time.monotonic() - t0
            if on_tick is not None:
                on_tick(t_rel)
            if msg is not None:
                if msg.get_type() == "HEARTBEAT":
                    state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif msg.get_type() == "STATUSTEXT":
                    if not suppress_status:
                        text = msg.text.rstrip("\x00").strip()
                        if text:
                            state["pending_text"].append(text)
                else:
                    row = handle_msg(state, msg, t_rel)
                    if row is not None and log is not None:
                        log.row(row)
            if t_rel - last_print >= print_period_s:
                last_print = t_rel
                cells = render_row(state, t_rel)
                txt = state["pending_text"].pop(0) if state["pending_text"] else ""
                cell_strs = [f"{v}" if v is not None else "n/a" for v in cells]
                line = "  ".join(f"{c:>{w}}" for c, w in zip(cell_strs, widths))
                print(f"  {line}  {txt}")
                while state["pending_text"]:
                    print(f"  {' '*sum(widths)}  {state['pending_text'].pop(0)}")
    except KeyboardInterrupt:
        print()
        aborted = True
    return (log.n_rows if log else 0, aborted)


# ---------------------------------------------------------------------------
# Oscillate callback
# ---------------------------------------------------------------------------

def _make_oscillate_tick(session: RawesGCS, steps: list):
    """Return an on_tick(t_rel) callback that advances through `steps`
    (a list of (tlon_deg, tlat_deg, col_deg, label) tuples) at
    _OSCILLATE_STEP_S sec/step.  Sends RAWES_TLN/TLT/COL NVFs on each
    step boundary.  Once the sequence ends the callback is a no-op (the
    duration timer in _observation_loop expires shortly after)."""
    last_step = [None]
    def _tick(t_rel: float) -> None:
        idx = int(t_rel / _OSCILLATE_STEP_S)
        if idx >= len(steps):
            return
        if idx == last_step[0]:
            return
        last_step[0] = idx
        tlon_d, tlat_d, col_d, label = steps[idx]
        session.send_named_float("RAWES_TLN", math.radians(tlon_d))
        session.send_named_float("RAWES_TLT", math.radians(tlat_d))
        session.send_named_float("RAWES_THR", float(col_d))
        print(f"  [{t_rel:6.1f}s] osc {idx+1}/{len(steps)}  "
              f"tlon={tlon_d:+5.1f}  tlat={tlat_d:+5.1f}  col={col_d:+5.1f}  "
              f"({label})")
    return _tick


# ---------------------------------------------------------------------------
# Generic run observation loop
# ---------------------------------------------------------------------------

def _run_observation(session: RawesGCS, mode_name: str,
                     duration: "float | None", log: _RunLog,
                     on_tick=None, keep_rc: bool = False) -> None:
    """Generic observation loop for `run` modes.  Stream + columns are the
    same across all modes; the row content is whatever the FC reports.  Per-
    mode NVFs (e.g. YAW_I, YAW_OUT) appear as columns when emitted.

    on_tick(t_rel) is called once per loop iteration (~10 Hz); used by
    oscillate mode to advance the trim sequence.

    Serial bandwidth is tight (57600 SiK ~= 5760 B/s, ~70% used by default),
    which drops NVF/telemetry.  So we trim streams we do not need for tuning:
    AHRS2 (EXTRA3) off, EXTENDED_STATUS down to 1 Hz, and RC_CHANNELS disabled
    (keeping SERVO_OUTPUT_RAW in the same stream) unless keep_rc is set."""
    from .util import _fmt   # pure utility, no cycle risk

    cols = ["t_s", "armed",
            "mav_att_roll_deg", "mav_att_pitch_deg", "mav_att_yaw_deg",
            "mav_att_yaw_rate_rads",
            "ch1_us", "ch2_us", "ch3_us", "ch4_us",
            "mav_servo1_us", "mav_servo2_us", "mav_servo3_us", "mav_servo9_us",
            "vbat_v", "current_a",
            "mav_nvf_yff_trim", "mav_nvf_yff_u", "mav_nvf_yff_gz",
            "erpm", "mech_rpm", "rotor_rpm"]

    # Live table: H_YAW_TRIM output (out=trim), motor throttle command (u=YFF_U),
    # swashplate PWMs (s1..s3), GB4008 motor PWM with 5 s rolling average, rotor RPM.
    print_cols = ["t(s)", "armed", "yaw(d)", "yrate_d", "out", "u",
                  "s1", "s2", "s3", "mot", "mot~5s", "V", "A", "mRPM"]

    mot_window_s = 5.0   # rolling average window for motor (SERVO_MOTOR) PWM

    if mode_name == "none":
        # In none mode (Lua idle) the yaw PID is inert, so the live keys tune the
        # static DDFP trim H_YAW_TRIM directly.  Step = 0.005 (~5 us over the
        # SERVO_MOTOR 1000-2000 us range); clamp [0, 1].
        _trim = {"param": "H_YAW_TRIM", "step": 0.005, "val": 0.0}
        _tv = session.get_param(_trim["param"])
        _trim["val"] = float(_tv) if _tv is not None else 0.0
        _pwm = 1000 + _trim["val"] * 1000.0
        print(f"  Yaw trim: H_YAW_TRIM={_trim['val']:.4f}  (~{_pwm:.0f} us)")
        print("  tune keys:  UP/DOWN arrows  or  '-'/'='   (H_YAW_TRIM +/- 0.005)")
        _arrow_pending = [False]

        def key_handler(k: bytes) -> None:
            # Windows arrow keys arrive as a two-byte sequence: a 0xe0/0x00
            # prefix followed by 'H' (up) / 'P' (down).  _poll_keys yields the
            # bytes separately, so latch the prefix and decode on the next byte.
            sign = 0
            if _arrow_pending[0]:
                _arrow_pending[0] = False
                if k == b"H":
                    sign = +1
                elif k == b"P":
                    sign = -1
                else:
                    return
            elif k in (b"\xe0", b"\x00"):
                _arrow_pending[0] = True
                return
            elif k == b"=":
                sign = +1
            elif k == b"-":
                sign = -1
            else:
                return
            old = _trim["val"]
            new = min(1.0, max(0.0, old + sign * _trim["step"]))
            ok = session.set_param(_trim["param"], new)
            _trim["val"] = new
            pwm = 1000 + new * 1000.0
            tag = "" if ok else "  [FAIL]"
            print(f"  TRIM {old:.4f} -> {new:.4f}  (~{pwm:.0f} us){tag}")
    else:
        def key_handler(k: bytes) -> None:  # no live key tuning for this mode
            pass

    state = {
        "roll": None, "pitch": None, "yaw": None, "yaw_rate": None,
        "ch1": None, "ch2": None, "ch3": None, "ch4": None,
        "s1": None, "s2": None, "s3": None, "smot": None,
        "smot_hist": [],
        "mrpm_hist": [],    # (t_rel, mech_rpm) for the 5 s rolling average on screen
        "vbat": None, "curr": None,
        "erpm": None,
        "yff_t": None, "yff_u": None, "yff_gz": None,
        "yff_t_ts": None, "yff_u_ts": None, "yff_gz_ts": None,
    }

    def handle_msg(st, msg, t_rel):
        mt = msg.get_type()
        if mt == "ATTITUDE":
            state["roll"]     = math.degrees(msg.roll)
            state["pitch"]    = math.degrees(msg.pitch)
            state["yaw"]      = math.degrees(msg.yaw)
            state["yaw_rate"] = msg.yawspeed          # rad/s (mav_att_yaw_rate_rads)
            # Emit one CSV row per ATTITUDE message (typically 10-50 Hz)
            _erpm, _mech, _rotor = _rpm_triplet(state["erpm"])
            return [
                f"{t_rel:.4f}", int(st["armed"]),
                _fmt(state["roll"]), _fmt(state["pitch"]), _fmt(state["yaw"]),
                _fmt(state["yaw_rate"]),
                state["ch1"], state["ch2"], state["ch3"], state["ch4"],
                state["s1"], state["s2"], state["s3"], state["smot"],
                _fmt(state["vbat"]), _fmt(state["curr"]),
                _fmt(state["yff_t"]), _fmt(state["yff_u"]), _fmt(state["yff_gz"]),
                _fmt(_erpm), _fmt(_mech), _fmt(_rotor),
            ]
        elif mt == "RC_CHANNELS":
            state["ch1"] = getattr(msg, "chan1_raw", None)
            state["ch2"] = getattr(msg, "chan2_raw", None)
            state["ch3"] = getattr(msg, "chan3_raw", None)
            state["ch4"] = getattr(msg, "chan4_raw", None)
        elif mt == "SERVO_OUTPUT_RAW":
            state["s1"] = getattr(msg, "servo1_raw", None)
            state["s2"] = getattr(msg, "servo2_raw", None)
            state["s3"] = getattr(msg, "servo3_raw", None)
            # GB4008 motor is on output SERVO_MOTOR (AUX 1); SERVO4 is unused now.
            state["smot"] = getattr(msg, f"servo{SERVO_MOTOR}_raw", None)
            if state["smot"] is not None:
                state["smot_hist"].append((t_rel, float(state["smot"])))
                cutoff = t_rel - mot_window_s
                while state["smot_hist"] and state["smot_hist"][0][0] < cutoff:
                    state["smot_hist"].pop(0)
        elif mt == "BATTERY_STATUS":
            cells = [v for v in msg.voltages if v != 65535]
            if cells:
                state["vbat"] = sum(cells) / 1000.0
            if msg.current_battery >= 0:
                state["curr"] = msg.current_battery / 100.0
        elif mt == "SYS_STATUS":
            if state["vbat"] is None and msg.voltage_battery != 65535:
                state["vbat"] = msg.voltage_battery / 1000.0
            if state["curr"] is None and msg.current_battery >= 0:
                state["curr"] = msg.current_battery / 100.0
        elif mt in _ESC_TELEM_MSGS:
            erpm = _esc_erpm(msg, MOTOR_ESC_CHANNEL)
            if erpm is not None:
                state["erpm"] = erpm
                _, _m, _ = _rpm_triplet(erpm)
                if _m is not None:
                    state["mrpm_hist"].append((t_rel, _m))
                    cutoff = t_rel - mot_window_s
                    while state["mrpm_hist"] and state["mrpm_hist"][0][0] < cutoff:
                        state["mrpm_hist"].pop(0)
        elif mt == "NAMED_VALUE_FLOAT":
            nm = msg.name.rstrip("\x00").strip() if isinstance(msg.name, str) else \
                 msg.name.decode("ascii", errors="replace").rstrip("\x00").strip()
            if nm == "YFF_T":
                state["yff_t"] = float(msg.value)
                state["yff_t_ts"] = t_rel
            elif nm == "YFF_U":
                state["yff_u"] = float(msg.value)
                state["yff_u_ts"] = t_rel
            elif nm == "YFF_GZ":
                state["yff_gz"] = float(msg.value)
                state["yff_gz_ts"] = t_rel
        return None

    def render_row(st, t_rel):
        def _fresh_or_stale(value, value_ts, fmt):
            if value is None:
                return None
            if value_ts is None or (t_rel - value_ts) > 2.0:
                return "stale"
            return fmt(value)

        yaw_s = f"{state['yaw']:+6.1f}" if state["yaw"] is not None else None
        yrate_s = f"{math.degrees(state['yaw_rate']):+6.1f}" if state["yaw_rate"] is not None else None
        out_s = _fresh_or_stale(state["yff_t"], state["yff_t_ts"], lambda v: f"{v:+.3f}")
        i_s = _fresh_or_stale(state["yff_u"], state["yff_u_ts"], lambda v: f"{v:+.3f}")
        mot_avg_s = None
        if state["smot_hist"]:
            mot_avg_s = f"{sum(v for _, v in state['smot_hist']) / len(state['smot_hist']):.0f}"
        _e, _m, _rotor = _rpm_triplet(state["erpm"])
        mrpm_avg_s = None
        if state["mrpm_hist"]:
            mrpm_avg_s = f"{sum(v for _, v in state['mrpm_hist']) / len(state['mrpm_hist']):.0f}"
        return [
            f"{t_rel:.1f}",
            "YES" if st["armed"] else "no",
            yaw_s,
            yrate_s,
            out_s,
            i_s,
            state["s1"], state["s2"], state["s3"], state["smot"], mot_avg_s,
            f"{state['vbat']:.2f}" if state["vbat"] is not None else None,
            f"{state['curr']:.2f}" if state["curr"] is not None else None,
            mrpm_avg_s,
        ]

    def _trim_streams() -> None:
        # Runs right after the stream requests so the RC_CHANNELS disable wins
        # over the RC_CHANNELS stream (which we keep for SERVO_OUTPUT_RAW).
        # Also request the motor's ESC telemetry (bidir DShot RPM) at 5 Hz.
        _esc_name, _esc_id = _esc_telem_msg_for_channel(MOTOR_ESC_CHANNEL)
        session.set_message_interval(_esc_id, 200000)   # 5 Hz
        if not keep_rc:
            session.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, -1)
            print("  Stream trim: RC_CHANNELS off, AHRS2 off, EXTENDED_STATUS 1 Hz "
                  f"(use --rc to keep RC_CHANNELS); {_esc_name} 5 Hz")
        else:
            print("  Stream trim: AHRS2 off, EXTENDED_STATUS 1 Hz (RC_CHANNELS kept); "
                  f"{_esc_name} 5 Hz")

    _observation_loop(
        session,
        duration_s=duration,
        msg_types=["ATTITUDE", "RC_CHANNELS", "SERVO_OUTPUT_RAW",
                   "HEARTBEAT", "STATUSTEXT", "BATTERY_STATUS", "SYS_STATUS",
                   "NAMED_VALUE_FLOAT",
                   _esc_telem_msg_for_channel(MOTOR_ESC_CHANNEL)[0]],
        streams=[
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,          25),
            (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,     25),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,          0),
        ],
        handle_msg=handle_msg,
        render_row=render_row,
        header_cols=cols,
        header_print_cols=print_cols,
        log=log,
        on_tick=on_tick,
        suppress_status=True,
        key_handler=key_handler,
        setup_hook=_trim_streams,
    )
    # Restore telemetry the trim disabled (best-effort; resets on FC reboot).
    if not keep_rc:
        session.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 0)
    session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2)
    session.request_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2)

    # Report final H_YAW_TRIM.
    _tv = session.get_param("H_YAW_TRIM")
    if _tv is not None:
        _pwm = 1000 + float(_tv) * 1000.0
        print(f"  Yaw trim final: H_YAW_TRIM={float(_tv):.4f}  (~{_pwm:.0f} us)")


# ---------------------------------------------------------------------------
# Passive yaw setup verification
# ---------------------------------------------------------------------------

def _verify_passive_yaw_setup(session: RawesGCS) -> bool:
    print("  Passive yaw setup check (AP yaw PID must be 0):")
    ok = True
    for n in _AP_YAW_ZERO_PARAMS:
        v = session.get_param(n)
        if v is None:
            print(f"    [WARN] {n}: unreadable")
            ok = False
            continue
        good = abs(float(v)) < 1e-9
        ok = ok and good
        tag = "[OK]  " if good else "[FAIL]"
        print(f"    {tag} {n} = {float(v):.6g}   (expect 0)")
    if not ok:
        print("    [WARN] AP yaw PID is NOT all zero -- H_YAW_TRIM trim will be contaminated.")
    return ok


# ---------------------------------------------------------------------------
# `run <mode>` command
# ---------------------------------------------------------------------------

def _cmd_run(session: RawesGCS, args: list[str]) -> None:
    """run <mode> [--duration N] [--trim K=V,...] [--gain K=V,...] [--osc TARGET]"""
    schema = {
        "--duration":         "float",
        "--trim":             "kv",
        "--gain":             "kv",
        "--osc":              "str",
        "--yaw":              "float",   # passive fixed yaw IC [deg] (RAWES_YIC)
        "--roll":             "float",   # passive IC roll  [deg] (RAWES_RIC)
        "--pitch":            "float",   # passive IC pitch [deg] (RAWES_PIC)
        "--rc":               "bool",    # keep RC_CHANNELS stream (mixer diagnosis)
    }
    if not args:
        print("  Usage: run <name> [--duration N] [--trim K=V,...] "
              "[--gain K=V,...] [--osc {all|s1|s2|s3}]")
        print("  Modes:")
        for name, cfg in _RUN_MODES.items():
            print(f"    {name:<8} -- {cfg['doc']}")
        return
    try:
        pos, flags = _parse_flags(args, schema)
    except ValueError as e:
        print(f"  Error: {e}"); return
    if len(pos) != 1:
        print("  Usage: run <name> [--duration N] [--trim K=V,...] "
              "[--gain K=V,...] [--osc {all|s1|s2|s3}]")
        return
    name = pos[0].lower()
    cfg = _RUN_MODES.get(name)
    if cfg is None:
        print(f"  Unknown mode {name!r}  (valid: {', '.join(_RUN_MODES)})")
        return
    duration         = flags.get("--duration")
    trim             = flags.get("--trim", {}) or {}
    gain             = flags.get("--gain", {}) or {}
    osc              = flags.get("--osc")

    osc_steps = None
    if osc is not None:
        osc = osc.lower()
        if osc not in _OSCILLATE_TARGETS:
            print(f"  Unknown --osc target {osc!r}  "
                  f"(valid: {', '.join(_OSCILLATE_TARGETS)})")
            return
        osc_steps = _OSCILLATE_TARGETS[osc]
        if duration is None:
            duration = len(osc_steps) * _OSCILLATE_STEP_S
        if trim:
            print(f"  [WARN] --trim {list(trim)} ignored because --osc is set")
            trim = {}

    # Validate trim keys
    bad = [k for k in trim if k not in _TRIM_NVF]
    if bad:
        print(f"  Unknown --trim keys: {bad}  (valid: {', '.join(_TRIM_NVF)})")
        return
    # Validate gain keys
    gain_map = cfg["gain_keys"]
    bad = [k for k in gain if k not in gain_map]
    if bad:
        if not gain_map:
            print(f"  Mode {name!r} does not accept --gain")
        else:
            print(f"  Unknown --gain keys for {name!r}: {bad}  (valid: {list(gain_map)})")
        return

    # Apply per-run param overrides (restored on exit)
    saved_overrides: dict[str, float] = {}
    if gain:
        print("  Per-run overrides:")
        for k, v in gain.items():
            ap_name = gain_map[k]
            orig = session.get_param(ap_name)
            if orig is None:
                print(f"    [WARN] {ap_name}: could not read -- skipping")
                continue
            saved_overrides[ap_name] = float(orig)
            ok = session.set_param(ap_name, float(v))
            tag = "[OK]  " if ok else "[FAIL]"
            print(f"    {tag} {ap_name}: {orig:.6g} -> {v:.6g}")

    # Mode-required param overrides (e.g. H_FLYBAR_MODE=1 for passthrough).
    force_params = cfg.get("force_params", {})
    if force_params:
        print("  Mode-required overrides:")
        for ap_name, target in force_params.items():
            orig = session.get_param(ap_name)
            if orig is None:
                print(f"    [WARN] {ap_name}: could not read -- skipping")
                continue
            if ap_name not in saved_overrides:
                saved_overrides[ap_name] = float(orig)
            ok = session.set_param(ap_name, float(target))
            tag = "[OK]  " if ok else "[FAIL]"
            print(f"    {tag} {ap_name}: {orig:.6g} -> {target}")

    # Passive mode requires AP-owned DDFP tail control on channel 4.
    if name == "passive":
        _ensure_passive_tail_setup(session)

    # SERVO4 ownership shuffle if the mode needs it
    saved_fn = _take_servo4(session) if cfg["take_servo4"] else None

    # Activate Lua mode
    session.set_param("RAWES_MODE", cfg["rawes_mode"])
    print(f"  RAWES_MODE -> {cfg['rawes_mode']} ({name} mode)")

    # Passive confirms the AP yaw PID was forced to zero.
    if name == "passive":
        _verify_passive_yaw_setup(session)

    # Flight mode required by this Lua mode (e.g. GUIDED_NOGPS for passive) --
    # matches the SITL passive arming flow.
    _fm = cfg.get("flight_mode")
    if _fm is not None:
        session.set_mode(_fm)
        print(f"  Flight mode -> {_COPTER_MODES.get(_fm, _fm)} ({_fm})")

    # Seed the IC (RAWES_THR/RIC/PIC) BEFORE arming so PASSIVE holds a defined
    # attitude (mirrors the SITL passive_init seed).
    if cfg.get("ic_seed"):
        col_deg = float(trim.get("col", _PASSIVE_IC_COL_DEG))
        print("  Seeding IC (deg -> rad on the wire):")
        session.send_named_float("RAWES_THR", float(col_deg))
        print(f"    RAWES_THR = {col_deg:.3f}  (thrust [0..1])")

        if "--yaw" in flags:
            yaw_deg = float(flags["--yaw"])
            session.send_named_float("RAWES_YIC", math.radians(yaw_deg))
            print(f"    RAWES_YIC = {yaw_deg:+7.3f} deg  ({math.radians(yaw_deg):+.4f} rad)")

        if "--roll" in flags:
            roll_deg = float(flags["--roll"])
            session.send_named_float("RAWES_RIC", math.radians(roll_deg))
            print(f"    RAWES_RIC = {roll_deg:+7.3f} deg  ({math.radians(roll_deg):+.4f} rad)")
        if "--pitch" in flags:
            pitch_deg = float(flags["--pitch"])
            session.send_named_float("RAWES_PIC", math.radians(pitch_deg))
            print(f"    RAWES_PIC = {pitch_deg:+7.3f} deg  ({math.radians(pitch_deg):+.4f} rad)")
        # col was consumed by the IC seed -- don't re-send it via the trim block.
        trim.pop("col", None)

    # Send NVF trims.  --trim values are user-facing DEGREES; convert to
    # radians for the wire (rawes.lua receives RAWES_TLN/TLT/COL in radians).
    if trim:
        print("  Sending trim NVFs (deg -> rad on the wire):")
        for k, v_deg in trim.items():
            v_rad = math.radians(float(v_deg))
            session.send_named_float(_TRIM_NVF[k], v_rad)
            print(f"    {_TRIM_NVF[k]} = {v_deg:+7.3f} deg  ({v_rad:+.4f} rad)")

    # Arm. Passive mode keeps channel 4 under AP/Lua tail ownership, so skip
    # direct DO_SET_SERVO pre-arm pulses on SERVO4 to avoid ownership conflicts.
    esc_arm = (name != "passive")
    if not _arm(session, force=True, esc_arm=esc_arm):
        _safety_shutdown(session, saved_servo4_fn=saved_fn,
                         saved_overrides=saved_overrides)
        return
    print("  [OK] Armed.")

    # Snapshot key params for the log header
    meta = {
        "verb":            "run",
        "name":            name,
        "duration_s":      duration if duration is not None else "",
        "trim_deg":        ", ".join(f"{k}={v}" for k, v in trim.items()),
        "gain":            ", ".join(f"{k}={v}" for k, v in gain.items()),
        "osc":             osc if osc is not None else "",
        "run_start_local": datetime.now().isoformat(timespec="seconds"),
        "run_start_utc":   datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "RAWES_MODE":      cfg["rawes_mode"],
    }
    # Add per-mode AP param snapshot
    for ap_name in gain_map.values():
        v = session.get_param(ap_name)
        meta[ap_name] = float(v) if v is not None else ""

    log = _RunLog.open("run", name, meta)
    print(f"  Logging to {log.path}")

    # Parallel MAVLink traffic log alongside the CSV.
    mavlog_path = log.path[:-4] + ".mavlink.jsonl" if log.path.endswith(".csv") \
        else log.path + ".mavlink.jsonl"
    session.start_mavlog(mavlog_path)
    print(f"  MAVLink log: {mavlog_path}")

    # Build the oscillate tick callback if --osc was set
    on_tick = None
    if osc_steps is not None:
        on_tick = _make_oscillate_tick(session, osc_steps)
        total_s = len(osc_steps) * _OSCILLATE_STEP_S
        print(f"  Oscillate target={osc!r}: walking {len(osc_steps)} steps "
              f"x {_OSCILLATE_STEP_S:.0f}s each (total {total_s:.0f}s).")

    try:
        _run_observation(session, name, duration, log, on_tick=on_tick,
                         keep_rc=bool(flags.get("--rc", False)))
    finally:
        session.stop_mavlog()
        log.close()
        print(f"  Wrote {log.n_rows} rows to {log.path}")
        _safety_shutdown(session, saved_servo4_fn=saved_fn,
                         saved_overrides=saved_overrides)
    print("  Done.")
