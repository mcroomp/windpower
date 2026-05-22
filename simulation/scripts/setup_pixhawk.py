#!/usr/bin/env python3
"""
setup_pixhawk.py -- RAWES Pixhawk 6C yaw-regulation bench setup.

Connects to the Pixhawk over USB, sets the parameters required to run the
GB4008 counter-torque motor under ArduPilot ATC_RAT_YAW DDFP control (SCR_USER6=0).
Prompts before each reboot.

Usage (Windows Git Bash):
    RAWES_HIL_PORT=COM4 .venv/Scripts/python.exe simulation/scripts/setup_pixhawk.py

Environment variables:
    RAWES_HIL_PORT   Serial port (required). Windows: COM3, COM4, ...
    RAWES_HIL_BAUD   Baud rate (default: 115200)
"""
from __future__ import annotations

import json
import os
import sys
import time

from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Parameter tables -- single source of truth is rawes_params.json
# ---------------------------------------------------------------------------
#
# rawes_params.json defines the canonical hardware configuration.  This script
# applies it; calibrate.py's `config show / config apply` validates against it.
#
# Phase split:
#   Phase 1 -- FRAME_CLASS only.  Must be written and the FC rebooted before
#              any H_* parameter is visible.
#   Phase 2 -- everything else, applied after the post-FRAME_CLASS reboot.
#
# Entries with "expected": null (e.g. SCR_USER6) are skipped -- the value is
# selected at run time, not at setup.

_PARAMS_JSON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "rawes_params.json")


def _load_params() -> tuple[list[tuple[str, float]], list[tuple[str, float]]]:
    """Return (phase1, phase2) parameter lists sourced from rawes_params.json.

    Phase 1 contains FRAME_CLASS only (it gates the rest behind a reboot).
    Phase 2 contains every other entry with a non-null expected value, in the
    order: setup -> key -> tail_pid (so H_* params are written after the heli
    frame class has unlocked them, and PID/tuning lands last).
    """
    with open(_PARAMS_JSON_PATH) as fh:
        groups = json.load(fh)

    phase1: list[tuple[str, float]] = []
    phase2: list[tuple[str, float]] = []
    for group_name in ("setup", "key", "tail_pid"):
        for entry in groups.get(group_name, []):
            name = entry["name"]
            expected = entry["expected"]
            if expected is None:
                continue   # runtime-selected (e.g. SCR_USER6)
            if name == "FRAME_CLASS":
                phase1.append((name, float(expected)))
            else:
                phase2.append((name, float(expected)))
    return phase1, phase2


_PHASE1, _PHASE2 = _load_params()

# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------

BAUD    = int(os.environ.get("RAWES_HIL_BAUD", "115200"))
TIMEOUT = 30.0   # seconds to wait for heartbeat / param ACK


def _connect(port: str) -> mavutil.mavfile:
    print(f"  Connecting to {port} at {BAUD} baud ...")
    mav = mavutil.mavlink_connection(port, baud=BAUD, source_system=255)
    mav.wait_heartbeat(timeout=TIMEOUT)
    print(f"  Connected: system={mav.target_system} component={mav.target_component}")
    return mav


def _recv_param(mav: mavutil.mavfile, name: str, timeout: float = 5.0) -> float | None:
    mav.mav.param_request_read_send(
        mav.target_system, mav.target_component,
        name.encode(), -1,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg and msg.param_id.rstrip("\x00") == name:
            return float(msg.param_value)
    return None


def _set_param(mav: mavutil.mavfile, name: str, value: float, timeout: float = 5.0) -> bool:
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        name.encode(), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg and msg.param_id.rstrip("\x00") == name:
            actual = float(msg.param_value)
            ok = abs(actual - float(value)) < 0.01
            return ok, actual
    return False, None


def _apply_params(mav: mavutil.mavfile, params: list[tuple[str, float]]) -> int:
    """Set each parameter and print status.  Returns number of failures."""
    failures = 0
    for name, value in params:
        ok, actual = _set_param(mav, name, value)
        if ok:
            print(f"  [OK]   {name} = {actual}")
        elif actual is not None:
            print(f"  [WARN] {name}: sent {value}, got {actual}")
            failures += 1
        else:
            print(f"  [FAIL] {name}: no ACK within timeout")
            failures += 1
    return failures


def _send_reboot(mav: mavutil.mavfile) -> None:
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,   # confirmation
        1,   # param1: 1 = reboot autopilot
        0, 0, 0, 0, 0, 0,
    )


def _wait_disconnect(mav: mavutil.mavfile, timeout: float = 15.0) -> None:
    """Wait until the Pixhawk stops sending heartbeats (reboot in progress)."""
    print("  Waiting for Pixhawk to go offline ...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            print("  Pixhawk offline.")
            return
    print("  (timeout waiting for disconnect -- continuing anyway)")


def _wait_reconnect(port: str, wait_s: float = 5.0) -> mavutil.mavfile:
    """Poll until the Pixhawk comes back up after a reboot."""
    print(f"  Waiting for Pixhawk to come back online ...")
    time.sleep(wait_s)
    for attempt in range(15):
        try:
            mav = _connect(port)
            print("  Reconnected.")
            return mav
        except Exception:
            print(f"  Reconnect attempt {attempt + 1}/15 ...")
            time.sleep(3.0)
    print("[ERROR] Could not reconnect after reboot.  Check USB cable.")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Main flow
# ---------------------------------------------------------------------------

def main() -> None:
    port = os.environ.get("RAWES_HIL_PORT")
    if not port:
        print("ERROR: set RAWES_HIL_PORT=COMx before running.")
        sys.exit(1)

    print()
    print("RAWES Pixhawk yaw-trim bench setup")
    print("=" * 40)

    # ── Connect ──────────────────────────────────────────────────────────────
    mav = _connect(port)

    # ── Phase 1: FRAME_CLASS ─────────────────────────────────────────────────
    print()
    print("[Phase 1] Frame type")
    if not _PHASE1:
        print("  [FAIL] FRAME_CLASS missing from rawes_params.json 'setup' group.")
        sys.exit(1)
    fc_name, fc_target = _PHASE1[0]
    current_fc = _recv_param(mav, fc_name)
    print(f"  Current {fc_name} = {current_fc}  (target {fc_target})")

    if current_fc != fc_target:
        print(f"  Setting {fc_name}={fc_target} (Traditional Helicopter) ...")
        ok, actual = _set_param(mav, fc_name, fc_target)
        if ok:
            print(f"  [OK]   {fc_name} = {actual}")
        else:
            print(f"  [FAIL] Could not set {fc_name}.  Aborting.")
            sys.exit(1)

        print()
        print("  Sending reboot command ...")
        _send_reboot(mav)
        _wait_disconnect(mav)
        mav.close()
        mav = _wait_reconnect(port)
    else:
        print(f"  {fc_name} already {fc_target} -- skipping reboot.")

    # ── Phase 2: Yaw-trim parameters ─────────────────────────────────────────
    print()
    print("[Phase 2] Yaw-trim parameters")
    failures = _apply_params(mav, _PHASE2)

    if failures:
        print(f"  {failures} parameter(s) failed to set -- check log above.")
    else:
        print("  All parameters set successfully.")

    # ── Phase 3: Lua script ───────────────────────────────────────────────────
    print()
    print("[Phase 3] Lua script deployment")
    scr = _recv_param(mav, "SCR_ENABLE")
    if scr != 1.0:
        print(f"  [WARN] SCR_ENABLE = {scr}  (expected 1 -- set via Mission Planner and reboot)")
    else:
        print(f"  [OK]   SCR_ENABLE = 1")

    mav.close()

    print()
    print("  Action required -- copy the Lua script to the Pixhawk SD card:")
    print("    Source : simulation\\scripts\\rawes.lua")
    print("    SD card: APM\\scripts\\rawes.lua")
    print()
    print("  Options:")
    print("    A) Remove the microSD card, copy the file, reinsert, power on.")
    print("    B) Mission Planner > Setup > Advanced > File Manager > /APM/scripts/")
    print()
    print("  After copying, reboot the Pixhawk and check GCS Messages for:")
    print("    RAWES hw: loaded  mode=2 (yaw)")
    print()
    print("  Then run:  RAWES_HIL_PORT=COM4 bash sim.sh test-hil -v")

    print("Parameters saved.  Setup complete.")


if __name__ == "__main__":
    main()
