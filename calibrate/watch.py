"""
calibrate/watch.py -- watch stream command and helpers.
"""
from __future__ import annotations

import math
import time
from datetime import datetime, timezone

from pymavlink import mavutil

from .constants import (
    Attitude,
    BatteryStatus,
    EscTelemetry,
    RawesGCS,
    CommandLong,
    SysStatus,
    StatusText,
    decode_message,
    MOTOR_ESC_CHANNEL, SERVO_MOTOR,
    _WATCH_STREAMS,
)
from .hw import (
    _esc_telem_msg_for_channel, _esc_erpm, _rpm_triplet,
)
from .run import _RunLog, _observation_loop
from .util import _fmt, _parse_flags


# ---------------------------------------------------------------------------
# `watch <stream>` command
# ---------------------------------------------------------------------------

def _cmd_watch(session: RawesGCS, args: list[str]) -> None:
    """watch <stream> [--duration N]"""

    schema = {"--duration": "float"}
    if not args:
        print("  Usage: watch <stream> [--duration N]")
        print("  Streams:")
        for k, v in _WATCH_STREAMS.items():
            print(f"    {k:<10} -- {v}")
        return
    try:
        pos, flags = _parse_flags(args, schema)
    except ValueError as e:
        print(f"  Error: {e}"); return
    if len(pos) != 1:
        print("  Usage: watch <stream> [--duration N]"); return
    stream = pos[0].lower()
    if stream not in _WATCH_STREAMS:
        print(f"  Unknown stream {stream!r}  (valid: {', '.join(_WATCH_STREAMS)})")
        return
    duration = flags.get("--duration", 10.0)

    meta = {
        "verb":            "watch",
        "stream":          stream,
        "duration_s":      duration,
        "run_start_local": datetime.now().isoformat(timespec="seconds"),
        "run_start_utc":   datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    log = _RunLog.open("watch", stream, meta)
    print(f"  Logging to {log.path}")

    try:
        if stream == "servos":
            _watch_servos(session, duration, log)
        elif stream == "esc":
            _watch_esc(session, duration, log)
        elif stream == "text":
            _watch_text(session, duration, log)
        elif stream == "attitude":
            _watch_attitude(session, duration, log)
        elif stream == "power":
            _watch_power(session, duration, log)
    finally:
        log.close()
        print(f"  Wrote {log.n_rows} rows to {log.path}")
    print("  Done.")


# ---------------------------------------------------------------------------
# Individual stream handlers
# ---------------------------------------------------------------------------

def _watch_servos(session, duration, log):
    cols = ["t_s"] + [f"s{i}_us" for i in range(1, 9)]
    state = {f"s{i}": None for i in range(1, 9)}

    def handle(st, msg, t_rel):
        decoded = decode_message(msg)
        if hasattr(decoded, "servo1_raw"):
            for i in range(1, 9):
                state[f"s{i}"] = getattr(decoded, f"servo{i}_raw", None)
            return [f"{t_rel:.4f}"] + [state[f"s{i}"] for i in range(1, 9)]
        return None

    def render(st, t_rel):
        return [f"{t_rel:.1f}"] + [state[f"s{i}"] for i in range(1, 9)]

    _observation_loop(
        session, duration_s=duration,
        msg_types=["SERVO_OUTPUT_RAW", "HEARTBEAT", "STATUSTEXT"],
        streams=[(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)],
        handle_msg=handle, render_row=render,
        header_cols=cols,
        header_print_cols=["t(s)"] + [f"s{i}" for i in range(1, 9)],
        log=log,
    )


def _watch_esc(session, duration, log):
    # GB4008 bidir DShot telemetry (AUX 1 = SERVO9 -> ESC_TELEMETRY_9_TO_12[0]).
    cols = ["t_s", "erpm", "mech_rpm", "rotor_rpm", "voltage_v", "current_a", "temp_c"]
    esc_name, esc_id = _esc_telem_msg_for_channel(MOTOR_ESC_CHANNEL)
    idx = (MOTOR_ESC_CHANNEL - 1) % 4
    state = {"erpm": None, "volt": None, "curr": None, "temp": None}

    def handle(st, msg, t_rel):
        decoded = decode_message(msg)
        if not isinstance(decoded, EscTelemetry) or decoded.message_name != esc_name:
            return None
        erpm = _esc_erpm(decoded, MOTOR_ESC_CHANNEL)
        volt = decoded.voltage[idx] / 100.0 if idx < len(decoded.voltage) else None
        curr = decoded.current[idx] / 100.0 if idx < len(decoded.current) else None
        temp = decoded.temperature[idx] if idx < len(decoded.temperature) else None
        state["erpm"], state["volt"], state["curr"], state["temp"] = \
            erpm, volt, curr, temp
        _e, mech, rotor = _rpm_triplet(erpm)
        return [f"{t_rel:.4f}", _fmt(_e), _fmt(mech), _fmt(rotor),
                _fmt(volt), _fmt(curr), temp]

    def render(st, t_rel):
        _e, mech, rotor = _rpm_triplet(state["erpm"])
        return [
            f"{t_rel:.1f}",
            f"{_e:.0f}" if _e is not None else None,
            f"{mech:.0f}" if mech is not None else None,
            f"{rotor:.0f}" if rotor is not None else None,
            f"{state['volt']:.2f}" if state["volt"] is not None else None,
            f"{state['curr']:.2f}" if state["curr"] is not None else None,
            state["temp"],
        ]

    def _req_esc() -> None:
        session.send_message(CommandLong(
            target_system=session._target_system,
            target_component=session._target_component,
            command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            param1=float(esc_id),
            param2=100000.0,
        ))  # 10 Hz

    _observation_loop(
        session, duration_s=duration,
        msg_types=[esc_name, "HEARTBEAT", "STATUSTEXT"],
        streams=[(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)],
        handle_msg=handle, render_row=render,
        header_cols=cols,
        header_print_cols=["t(s)", "eRPM", "mRPM", "rotRPM", "V", "A", "T"],
        setup_hook=_req_esc,
        log=log,
    )


def _watch_text(session, duration, log):
    cols = ["t_s", "severity", "text"]

    def handle(st, msg, t_rel):
        # STATUSTEXTs are handled by the engine and surfaced via state["pending_text"]
        # We still log them here so the CSV captures everything.
        decoded = decode_message(msg)
        if isinstance(decoded, StatusText):
            text = decoded.text
            if text:
                return [f"{t_rel:.4f}", int(getattr(msg, "severity", 6)), text]
        return None

    def render(st, t_rel):
        return [f"{t_rel:.1f}"]

    _observation_loop(
        session, duration_s=duration,
        msg_types=["HEARTBEAT", "STATUSTEXT"],
        streams=[],
        handle_msg=handle, render_row=render,
        header_cols=cols,
        header_print_cols=["t(s)"],
        log=log,
    )


def _watch_attitude(session, duration, log):
    cols = ["t_s",
            "mav_att_roll_deg", "mav_att_pitch_deg", "mav_att_yaw_deg",
            "mav_att_roll_rate_rads", "mav_att_pitch_rate_rads", "mav_att_yaw_rate_rads"]
    state = {"roll": None, "pitch": None, "yaw": None,
             "wx": None, "wy": None, "wz": None}

    def handle(st, msg, t_rel):
        decoded = decode_message(msg)
        if isinstance(decoded, Attitude):
            state["roll"]  = math.degrees(decoded.roll)
            state["pitch"] = math.degrees(decoded.pitch)
            state["yaw"]   = math.degrees(decoded.yaw)
            state["wx"]    = decoded.rollspeed
            state["wy"]    = decoded.pitchspeed
            state["wz"]    = decoded.yawspeed
            return [f"{t_rel:.4f}",
                    f"{state['roll']:.3f}", f"{state['pitch']:.3f}", f"{state['yaw']:.3f}",
                    f"{state['wx']:.4f}", f"{state['wy']:.4f}", f"{state['wz']:.4f}"]
        return None

    def render(st, t_rel):
        return [
            f"{t_rel:.1f}",
            f"{state['roll']:+.1f}"  if state["roll"]  is not None else None,
            f"{state['pitch']:+.1f}" if state["pitch"] is not None else None,
            f"{state['yaw']:+.1f}"   if state["yaw"]   is not None else None,
            f"{math.degrees(state['wx']):+.1f}" if state["wx"] is not None else None,
            f"{math.degrees(state['wy']):+.1f}" if state["wy"] is not None else None,
            f"{math.degrees(state['wz']):+.1f}" if state["wz"] is not None else None,
        ]

    _observation_loop(
        session, duration_s=duration,
        msg_types=["ATTITUDE", "HEARTBEAT", "STATUSTEXT"],
        streams=[(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 25)],
        handle_msg=handle, render_row=render,
        header_cols=cols, log=log,
    )


def _watch_power(session, duration, log):
    cols = ["t_s", "vbat_v", "current_a", "power_w"]
    state = {"vbat": None, "curr": None}

    def handle(st, msg, t_rel):
        match decode_message(msg):
            case BatteryStatus(voltages=voltages, current_battery=current_battery):
                cells = [v for v in voltages if v != 65535]
                if cells:
                    state["vbat"] = sum(cells) / 1000.0
                if current_battery >= 0:
                    state["curr"] = current_battery / 100.0
            case SysStatus(voltage_battery=voltage_battery, current_battery=current_battery):
                if state["vbat"] is None and voltage_battery != 65535:
                    state["vbat"] = voltage_battery / 1000.0
                if state["curr"] is None and current_battery >= 0:
                    state["curr"] = current_battery / 100.0
            case _:
                return None
        power = state["vbat"] * state["curr"] if (state["vbat"] is not None and state["curr"] is not None) else None
        return [f"{t_rel:.4f}", _fmt(state["vbat"]), _fmt(state["curr"]), _fmt(power)]

    def render(st, t_rel):
        p = state["vbat"] * state["curr"] if (state["vbat"] is not None and state["curr"] is not None) else None
        return [
            f"{t_rel:.1f}",
            f"{state['vbat']:.2f}" if state["vbat"] is not None else None,
            f"{state['curr']:.2f}" if state["curr"] is not None else None,
            f"{p:.1f}" if p is not None else None,
        ]

    _observation_loop(
        session, duration_s=duration,
        msg_types=["BATTERY_STATUS", "SYS_STATUS", "HEARTBEAT", "STATUSTEXT"],
        streams=[(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5)],
        handle_msg=handle, render_row=render,
        header_cols=cols, log=log,
    )
