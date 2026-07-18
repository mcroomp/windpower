"""
calibrate/run.py -- Observation loop engine, run command, oscillate.
"""
from __future__ import annotations

import csv
import math
import os
import time
from datetime import datetime, timezone

from pymavlink import mavutil
from groundstation.gcs import decode_message

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
    Attitude,
    Heartbeat,
    EscTelemetry,
    PidTuning,
    RcChannels,
    RawesGCS,
    NamedValueFloat,
    CommandLong,
    RequestDataStream,
    SetAttitudeTarget,
    StatusText,
    SERVO_MOTOR, MOTOR_OFF_US, MOTOR_ESC_CHANNEL,
    _ESC_TELEM_MSGS,
    _RUN_MODES, _TRIM_NVF, _IC_TRIM_KEYS, _PASSIVE_IC_THRUST,
    _RAWES_YIC_CAPTURE_SENTINEL,
    _OSCILLATE_TARGETS, _OSCILLATE_STEP_S,
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
        decoded = decode_message(msg)
        if isinstance(decoded, StatusText):
            print(f"  [FC] {decoded.text}")
        elif isinstance(decoded, Heartbeat):
            if bool(decoded.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
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
            print("  [SAFETY] disarm rejected/not confirmed -- retrying force-disarm immediately")
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
        session.send_message(RequestDataStream(
            target_system=session._target_system,
            target_component=session._target_component,
            req_stream_id=s_id,
            req_message_rate=hz,
        ))

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
                decoded = decode_message(msg)
                if isinstance(decoded, Heartbeat):
                    state["armed"] = bool(decoded.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif isinstance(decoded, StatusText):
                    if not suppress_status:
                        text = decoded.text
                        if text:
                            state["pending_text"].append(text)
                else:
                    row = handle_msg(state, decoded, t_rel)
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
    (a list of (tlon_deg, tlat_deg, thr, label) tuples) at
    _OSCILLATE_STEP_S sec/step.  Sends RAWES_TLN/TLT NVFs (in radians) and
    RAWES_THR (thrust [0..1]) on each step boundary.  Once the sequence ends
    the callback is a no-op (the duration timer in _observation_loop expires
    shortly after)."""
    last_step: list[int | None] = [None]
    def _tick(t_rel: float) -> None:
        idx = int(t_rel / _OSCILLATE_STEP_S)
        if idx >= len(steps):
            return
        if idx == last_step[0]:
            return
        last_step[0] = idx
        tlon_d, tlat_d, thr_d, label = steps[idx]
        session.send_message(NamedValueFloat("RAWES_TLN", math.radians(tlon_d)))
        session.send_message(NamedValueFloat("RAWES_TLT", math.radians(tlat_d)))
        session.send_message(NamedValueFloat("RAWES_THR", float(thr_d)))
        print(f"  [{t_rel:6.1f}s] osc {idx+1}/{len(steps)}  "
              f"tlon={tlon_d:+5.1f}  tlat={tlat_d:+5.1f}  thr={thr_d:.3f}  "
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
            "mav_att_target_roll_deg", "mav_att_target_pitch_deg", "mav_att_target_yaw_deg",
            "mav_att_target_roll_rate_rads", "mav_att_target_pitch_rate_rads", "mav_att_target_yaw_rate_rads",
            "mav_att_target_thrust",
            "ch1_us", "ch2_us", "ch3_us", "ch4_us",
            "mav_servo1_us", "mav_servo2_us", "mav_servo3_us", "mav_servo9_us",
            "vbat_v", "current_a",
            "mav_nvf_yff_trim", "mav_nvf_yff_u", "mav_nvf_yff_gz",
            "mav_nvf_ol_rsp", "mav_nvf_ol_psp", "mav_nvf_ol_ysp",
            "mav_nvf_ol_rer", "mav_nvf_ol_per", "mav_nvf_ol_yer",
            "mav_nvf_ol_ap", "mav_nvf_ol_ai", "mav_nvf_ol_ad",
            "mav_nvf_ol_col", "mav_nvf_ol_ten",
            "mav_pid_roll_des", "mav_pid_roll_ach", "mav_pid_roll_ff", "mav_pid_roll_p", "mav_pid_roll_i", "mav_pid_roll_d",
            "mav_pid_pitch_des", "mav_pid_pitch_ach", "mav_pid_pitch_ff", "mav_pid_pitch_p", "mav_pid_pitch_i", "mav_pid_pitch_d",
            "mav_pid_yaw_des", "mav_pid_yaw_ach", "mav_pid_yaw_ff", "mav_pid_yaw_p", "mav_pid_yaw_i", "mav_pid_yaw_d",
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
    elif mode_name in ("passive", "steady", "pumping"):
        _yaw_pid = {
            "P": {"param": "ATC_RAT_YAW_P", "step": 0.002,  "val": 0.0},
            "I": {"param": "ATC_RAT_YAW_I", "step": 0.0005, "val": 0.0},
            "D": {"param": "ATC_RAT_YAW_D", "step": 0.001,  "val": 0.0},
        }
        for axis in _yaw_pid.values():
            _tv = session.get_param(axis["param"])
            axis["val"] = float(_tv) if _tv is not None else 0.0
        print("  Yaw PID:")
        print(f"    P={_yaw_pid['P']['val']:.4f}  I={_yaw_pid['I']['val']:.4f}  D={_yaw_pid['D']['val']:.4f}")
        print("  tune keys:  q/a = P +/- 0.002,  w/s = I +/- 0.0005,  e/d = D +/- 0.001")

        def _bump(axis_name: str, sign: int) -> None:
            axis = _yaw_pid[axis_name]
            old = axis["val"]
            new = max(0.0, old + sign * axis["step"])
            ok = session.set_param(axis["param"], new)
            axis["val"] = new
            tag = "" if ok else "  [FAIL]"
            print(f"  {axis['param']} {old:.4f} -> {new:.4f}{tag}")

        def key_handler(k: bytes) -> None:
            if k == b"q":
                _bump("P", +1)
            elif k == b"a":
                _bump("P", -1)
            elif k == b"w":
                _bump("I", +1)
            elif k == b"s":
                _bump("I", -1)
            elif k == b"e":
                _bump("D", +1)
            elif k == b"d":
                _bump("D", -1)
    else:
        def key_handler(k: bytes) -> None:  # no live key tuning for this mode
            pass

    state = {
        "roll": None, "pitch": None, "yaw": None, "yaw_rate": None,
        "att_target_roll": None, "att_target_pitch": None, "att_target_yaw": None,
        "att_target_roll_rate": None, "att_target_pitch_rate": None, "att_target_yaw_rate": None,
        "att_target_thrust": None,
        "ch1": None, "ch2": None, "ch3": None, "ch4": None,
        "s1": None, "s2": None, "s3": None, "smot": None,
        "smot_hist": [],
        "mrpm_hist": [],    # (t_rel, mech_rpm) for the 5 s rolling average on screen
        "vbat": None, "curr": None,
        "erpm": None,
        "yff_t": None, "yff_u": None, "yff_gz": None,
        "ol_rsp": None, "ol_psp": None, "ol_ysp": None,
        "ol_rer": None, "ol_per": None, "ol_yer": None,
        "ol_ap": None, "ol_ai": None, "ol_ad": None,
        "ol_col": None, "ol_ten": None,
        "yff_t_ts": None, "yff_u_ts": None, "yff_gz_ts": None,
        "pid_roll_des": None, "pid_roll_ach": None, "pid_roll_ff": None, "pid_roll_p": None, "pid_roll_i": None, "pid_roll_d": None,
        "pid_pitch_des": None, "pid_pitch_ach": None, "pid_pitch_ff": None, "pid_pitch_p": None, "pid_pitch_i": None, "pid_pitch_d": None,
        "pid_yaw_des": None, "pid_yaw_ach": None, "pid_yaw_ff": None, "pid_yaw_p": None, "pid_yaw_i": None, "pid_yaw_d": None,
    }

    def _quat_to_rpy_deg(q) -> tuple[float, float, float] | tuple[None, None, None]:
        if q is None or len(q) != 4:
            return None, None, None
        w, x, y, z = [float(v) for v in q]
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
        else:
            pitch = math.degrees(math.asin(sinp))

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        return roll, pitch, yaw

    def handle_msg(st, msg, t_rel):
        if isinstance(msg, Attitude):
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
                _fmt(state["att_target_roll"]), _fmt(state["att_target_pitch"]), _fmt(state["att_target_yaw"]),
                _fmt(state["att_target_roll_rate"]), _fmt(state["att_target_pitch_rate"]), _fmt(state["att_target_yaw_rate"]),
                _fmt(state["att_target_thrust"]),
                state["ch1"], state["ch2"], state["ch3"], state["ch4"],
                state["s1"], state["s2"], state["s3"], state["smot"],
                _fmt(state["vbat"]), _fmt(state["curr"]),
                _fmt(state["yff_t"]), _fmt(state["yff_u"]), _fmt(state["yff_gz"]),
                _fmt(state["ol_rsp"]), _fmt(state["ol_psp"]), _fmt(state["ol_ysp"]),
                _fmt(state["ol_rer"]), _fmt(state["ol_per"]), _fmt(state["ol_yer"]),
                _fmt(state["ol_ap"]), _fmt(state["ol_ai"]), _fmt(state["ol_ad"]),
                _fmt(state["ol_col"]), _fmt(state["ol_ten"]),
                _fmt(state["pid_roll_des"]), _fmt(state["pid_roll_ach"]), _fmt(state["pid_roll_ff"]), _fmt(state["pid_roll_p"]), _fmt(state["pid_roll_i"]), _fmt(state["pid_roll_d"]),
                _fmt(state["pid_pitch_des"]), _fmt(state["pid_pitch_ach"]), _fmt(state["pid_pitch_ff"]), _fmt(state["pid_pitch_p"]), _fmt(state["pid_pitch_i"]), _fmt(state["pid_pitch_d"]),
                _fmt(state["pid_yaw_des"]), _fmt(state["pid_yaw_ach"]), _fmt(state["pid_yaw_ff"]), _fmt(state["pid_yaw_p"]), _fmt(state["pid_yaw_i"]), _fmt(state["pid_yaw_d"]),
                _fmt(_erpm), _fmt(_mech), _fmt(_rotor),
            ]
        if isinstance(msg, SetAttitudeTarget):
            att_r, att_p, att_y = _quat_to_rpy_deg(msg.q)
            state["att_target_roll"] = att_r
            state["att_target_pitch"] = att_p
            state["att_target_yaw"] = att_y
            state["att_target_roll_rate"] = msg.body_roll_rate
            state["att_target_pitch_rate"] = msg.body_pitch_rate
            state["att_target_yaw_rate"] = msg.body_yaw_rate
            state["att_target_thrust"] = msg.thrust
        elif isinstance(msg, RcChannels):
            state["ch1"] = msg.chan1_raw
            state["ch2"] = msg.chan2_raw
            state["ch3"] = msg.chan3_raw
            state["ch4"] = msg.chan4_raw
        elif hasattr(msg, "servo1_raw"):
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
        elif hasattr(msg, "voltages"):
            cells = [v for v in msg.voltages if v != 65535]
            if cells:
                state["vbat"] = sum(cells) / 1000.0
            if msg.current_battery >= 0:
                state["curr"] = msg.current_battery / 100.0
        elif hasattr(msg, "voltage_battery"):
            if state["vbat"] is None and msg.voltage_battery != 65535:
                state["vbat"] = msg.voltage_battery / 1000.0
            if state["curr"] is None and msg.current_battery >= 0:
                state["curr"] = msg.current_battery / 100.0
        elif isinstance(msg, EscTelemetry):
            erpm = _esc_erpm(msg, MOTOR_ESC_CHANNEL)
            if erpm is not None:
                state["erpm"] = erpm
                _, _m, _ = _rpm_triplet(erpm)
                if _m is not None:
                    state["mrpm_hist"].append((t_rel, _m))
                    cutoff = t_rel - mot_window_s
                    while state["mrpm_hist"] and state["mrpm_hist"][0][0] < cutoff:
                        state["mrpm_hist"].pop(0)
        elif isinstance(msg, NamedValueFloat):
            nm = msg.name
            if nm == "YFF_T":
                state["yff_t"] = float(msg.value)
                state["yff_t_ts"] = t_rel
            elif nm == "YFF_U":
                state["yff_u"] = float(msg.value)
                state["yff_u_ts"] = t_rel
            elif nm == "YFF_GZ":
                state["yff_gz"] = float(msg.value)
                state["yff_gz_ts"] = t_rel
            elif nm == "OL_RSP":
                state["ol_rsp"] = float(msg.value)
            elif nm == "OL_PSP":
                state["ol_psp"] = float(msg.value)
            elif nm == "OL_YSP":
                state["ol_ysp"] = float(msg.value)
            elif nm == "OL_RER":
                state["ol_rer"] = float(msg.value)
            elif nm == "OL_PER":
                state["ol_per"] = float(msg.value)
            elif nm == "OL_YER":
                state["ol_yer"] = float(msg.value)
            elif nm == "OL_AP":
                state["ol_ap"] = float(msg.value)
            elif nm == "OL_AI":
                state["ol_ai"] = float(msg.value)
            elif nm == "OL_AD":
                state["ol_ad"] = float(msg.value)
            elif nm == "OL_COL":
                state["ol_col"] = float(msg.value)
            elif nm == "OL_TEN":
                state["ol_ten"] = float(msg.value)
        elif isinstance(msg, PidTuning):
            axis = msg.axis
            # ArduPilot emits PID_TUNING axis as 1=roll, 2=pitch, 3=yaw, 4=accelz.
            prefix = {1: "pid_roll", 2: "pid_pitch", 3: "pid_yaw"}.get(axis)
            if prefix is not None:
                state[f"{prefix}_des"] = msg.desired
                state[f"{prefix}_ach"] = msg.achieved
                state[f"{prefix}_ff"] = msg.FF
                state[f"{prefix}_p"] = msg.P
                state[f"{prefix}_i"] = msg.I
                state[f"{prefix}_d"] = msg.D
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
        session.send_message(CommandLong(
            target_system=session._target_system,
            target_component=session._target_component,
            command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            param1=float(_esc_id),
            param2=200000.0,
        ))  # 5 Hz

        if not keep_rc:
            session.send_message(CommandLong(
                target_system=session._target_system,
                target_component=session._target_component,
                command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                param1=float(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS),
                param2=-1.0,
            ))
            print("  Stream trim: RC_CHANNELS off, AHRS2 off, EXTENDED_STATUS 1 Hz "
                  f"(use --rc to keep RC_CHANNELS); {_esc_name} 5 Hz")
        else:
            print("  Stream trim: AHRS2 off, EXTENDED_STATUS 1 Hz (RC_CHANNELS kept); "
                  f"{_esc_name} 5 Hz")

    _observation_loop(
        session,
        duration_s=duration,
        msg_types=["ATTITUDE", "RC_CHANNELS", "SERVO_OUTPUT_RAW",
                   "ATTITUDE_TARGET", "PID_TUNING",
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
        session.send_message(CommandLong(
            target_system=session._target_system,
            target_component=session._target_component,
            command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            param1=float(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS),
            param2=0.0,
        ))
    session.send_message(RequestDataStream(
        target_system=session._target_system,
        target_component=session._target_component,
        req_stream_id=mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
        req_message_rate=2,
    ))
    session.send_message(RequestDataStream(
        target_system=session._target_system,
        target_component=session._target_component,
        req_stream_id=mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
        req_message_rate=2,
    ))

    # Report final H_YAW_TRIM.
    _tv = session.get_param("H_YAW_TRIM")
    if _tv is not None:
        _pwm = 1000 + float(_tv) * 1000.0
        print(f"  Yaw trim final: H_YAW_TRIM={float(_tv):.4f}  (~{_pwm:.0f} us)")


# ---------------------------------------------------------------------------
# `run <mode>` command
# ---------------------------------------------------------------------------

def _cmd_run(session: RawesGCS, args: list[str]) -> None:
    """run <mode> [--duration N] [--trim K=V,...] [--osc TARGET]"""
    schema = {
        "--duration":         "float",
        "--trim":             "kv",
        "--osc":              "str",
        "--yaw":              "float",   # passive fixed yaw IC [deg] (RAWES_YIC)
        "--roll":             "float",   # passive IC roll  [deg] (RAWES_RIC)
        "--pitch":            "float",   # passive IC pitch [deg] (RAWES_PIC)
        "--hold":             "bool",    # passive: capture current roll/pitch/yaw as IC (RAWES_YIC sentinel)
        "--rc":               "bool",    # keep RC_CHANNELS stream (mixer diagnosis)
    }
    if not args:
        print("  Usage: run <name> [--duration N] [--trim K=V,...] "
              "[--osc {all|s1|s2|s3}]")
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
              "[--osc {all|s1|s2|s3}]")
        return
    name = pos[0].lower()
    cfg = _RUN_MODES.get(name)
    if cfg is None:
        print(f"  Unknown mode {name!r}  (valid: {', '.join(_RUN_MODES)})")
        return
    duration         = flags.get("--duration")
    trim             = flags.get("--trim", {}) or {}
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

    # Validate trim keys: angle keys (tlon/tlat) + ic-seed thrust key (thr, passive only)
    allowed_trim = set(_TRIM_NVF)
    if cfg.get("ic_seed"):
        allowed_trim |= _IC_TRIM_KEYS
    bad = [k for k in trim if k not in allowed_trim]
    if bad:
        print(f"  Unknown --trim keys: {bad}  (valid: {', '.join(sorted(allowed_trim))})")
        return

    # Open CSV + MAVLink logs before any run side-effects so startup/arming
    # delays are visible in the packet trace.
    meta = {
        "verb":            "run",
        "name":            name,
        "duration_s":      duration if duration is not None else "",
        "trim_deg":        ", ".join(f"{k}={v}" for k, v in trim.items()),
        "osc":             osc if osc is not None else "",
        "run_start_local": datetime.now().isoformat(timespec="seconds"),
        "run_start_utc":   datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "RAWES_MODE":      cfg["rawes_mode"],
    }
    log = _RunLog.open("run", name, meta)
    print(f"  Logging to {log.path}")

    mavlog_path = log.path[:-4] + ".mavlink.jsonl" if log.path.endswith(".csv") \
        else log.path + ".mavlink.jsonl"
    session.start_mavlog(mavlog_path)
    print(f"  MAVLink log: {mavlog_path}")

    saved_overrides: dict[str, float] = {}
    armed = False
    saved_fn = None
    done_ok = False
    try:

        # Passive mode requires AP-owned DDFP tail control on channel 4.
        if name == "passive":
            _ensure_passive_tail_setup(session)

        # SERVO4 ownership shuffle if the mode needs it
        saved_fn = _take_servo4(session) if cfg["take_servo4"] else None

        # Activate Lua mode
        session.set_param("RAWES_MODE", cfg["rawes_mode"])
        print(f"  RAWES_MODE -> {cfg['rawes_mode']} ({name} mode)")

    # Flight mode required by this Lua mode (e.g. GUIDED_NOGPS for passive) --
    # matches the SITL passive arming flow.
        _fm = cfg.get("flight_mode")
        if _fm is not None:
            session.set_mode(_fm)
            print(f"  Flight mode -> {_COPTER_MODES.get(_fm, _fm)} ({_fm})")

    # Seed the IC (RAWES_THR/RIC/PIC) BEFORE arming so PASSIVE holds a defined
    # attitude (mirrors the SITL passive_init seed).
        if cfg.get("ic_seed"):
            thr = float(trim.get("thr", _PASSIVE_IC_THRUST))
            print("  Seeding IC:")
            session.send_message(NamedValueFloat("RAWES_THR", thr))
            print(f"    RAWES_THR = {thr:.3f}  (thrust [0..1])")

            if flags.get("--hold"):
                # --hold captures the CURRENT roll/pitch/yaw in Lua via the
                # RAWES_YIC sentinel, so RAWES_RIC/RAWES_PIC must NOT be sent
                # here (Lua only auto-fills them if they haven't already been
                # explicitly provided).
                if "--roll" in flags or "--pitch" in flags:
                    print("  [WARN] --hold ignores --roll/--pitch (captures current AHRS attitude instead)")
                if "--yaw" in flags:
                    print("  [WARN] --hold ignores --yaw (captures current AHRS attitude instead)")
                session.send_message(NamedValueFloat("RAWES_YIC", _RAWES_YIC_CAPTURE_SENTINEL))
                print(f"    RAWES_YIC = {_RAWES_YIC_CAPTURE_SENTINEL:.1f}  (capture current roll/pitch/yaw)")
            else:
                roll_deg = float(flags.get("--roll", 0.0))
                pitch_deg = float(flags.get("--pitch", 0.0))
                # RAWES IC seed commits atomically only after THR+RIC+PIC all arrive.
                # Always send roll/pitch (default 0 deg) so PASSIVE does not stall at
                # "ic=waiting" when only --trim thr is provided.
                session.send_message(NamedValueFloat("RAWES_RIC", math.radians(roll_deg)))
                print(f"    RAWES_RIC = {roll_deg:+7.3f} deg  ({math.radians(roll_deg):+.4f} rad)")
                session.send_message(NamedValueFloat("RAWES_PIC", math.radians(pitch_deg)))
                print(f"    RAWES_PIC = {pitch_deg:+7.3f} deg  ({math.radians(pitch_deg):+.4f} rad)")

                if "--yaw" in flags:
                    yaw_deg = float(flags["--yaw"])
                    session.send_message(NamedValueFloat("RAWES_YIC", math.radians(yaw_deg)))
                    print(f"    RAWES_YIC = {yaw_deg:+7.3f} deg  ({math.radians(yaw_deg):+.4f} rad)")
            # thr was consumed by the IC seed -- don't re-send it via the trim block.
            trim.pop("thr", None)

    # Send NVF trims.  --trim values are user-facing DEGREES; convert to
    # radians for the wire (rawes.lua receives RAWES_TLN/TLT/COL in radians).
        if trim:
            print("  Sending trim NVFs (deg -> rad on the wire):")
            for k, v_deg in trim.items():
                v_rad = math.radians(float(v_deg))
                session.send_message(NamedValueFloat(_TRIM_NVF[k], v_rad))
                print(f"    {_TRIM_NVF[k]} = {v_deg:+7.3f} deg  ({v_rad:+.4f} rad)")

    # Arm. Passive mode keeps channel 4 under AP/Lua tail ownership, so skip
    # direct DO_SET_SERVO pre-arm pulses on SERVO4 to avoid ownership conflicts.
        esc_arm = (name != "passive")
        if not _arm(session, force=True, esc_arm=esc_arm):
            return
        armed = True
        print("  [OK] Armed.")

    # Build the oscillate tick callback if --osc was set
        on_tick = None
        if osc_steps is not None:
            on_tick = _make_oscillate_tick(session, osc_steps)
            total_s = len(osc_steps) * _OSCILLATE_STEP_S
            print(f"  Oscillate target={osc!r}: walking {len(osc_steps)} steps "
                  f"x {_OSCILLATE_STEP_S:.0f}s each (total {total_s:.0f}s).")

        _run_observation(session, name, duration, log, on_tick=on_tick,
                         keep_rc=bool(flags.get("--rc", False)))
        done_ok = True
    finally:
        try:
            if armed or saved_fn is not None:
                _safety_shutdown(session, saved_servo4_fn=saved_fn,
                                 saved_overrides=saved_overrides)
        finally:
            session.stop_mavlog()
            log.close()
            print(f"  Wrote {log.n_rows} rows to {log.path}")
    if done_ok:
        print("  Done.")
