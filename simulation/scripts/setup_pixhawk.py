#!/usr/bin/env python3
"""
setup_pixhawk.py -- RAWES Pixhawk 6C yaw-trim bench setup.

Connects to the Pixhawk over USB, sets the parameters required to run the
GB4008 counter-torque motor under rawes.lua yaw-trim control (SCR_USER6=2).
Prompts before each reboot.

Usage (Windows Git Bash):
    RAWES_HIL_PORT=COM4 simulation/.venv/Scripts/python.exe simulation/scripts/setup_pixhawk.py

Environment variables:
    RAWES_HIL_PORT   Serial port (required). Windows: COM3, COM4, ...
    RAWES_HIL_BAUD   Baud rate (default: 115200)
"""
from __future__ import annotations

import os
import sys
import time

from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Parameter tables
# ---------------------------------------------------------------------------

# Phase 1: set before first reboot.  FRAME_CLASS unlocks all H_* parameters.
_PHASE1 = [
    ("FRAME_CLASS", 6),   # Traditional Helicopter -- H_* params appear after reboot
]

# Phase 2: set after reboot (H_* parameters now exist).
_PHASE2 = [
    # RSC: CH8 passthrough -- arming does not wait for rotor spin-up
    ("H_RSC_MODE",        1),
    ("H_RSC_RUNUP_TIME",  1),
    # Yaw motor: servo output, linear 1000-2000 us, neutral at 1500 us
    ("H_TAIL_TYPE",       0),
    ("H_COL2YAW",         0.0),
    # Yaw PID: pure-P starting point; back-EMF provides inherent speed regulation
    ("ATC_RAT_YAW_P",     0.001),
    ("ATC_RAT_YAW_I",     0.0),
    ("ATC_RAT_YAW_D",     0.0),
    ("ATC_RAT_YAW_IMAX",  0.0),
    # Roll/pitch IMAX: zero to prevent integrator windup on neutral sticks
    ("ATC_RAT_RLL_IMAX",  0.0),
    ("ATC_RAT_PIT_IMAX",  0.0),
    # ACRO: disable auto-leveling (tether pulls hub to ~65 deg, trainer would fight it)
    ("ACRO_TRAINER",      0),
    # Boot into ACRO so rawes.lua yaw-trim runs immediately after arm
    ("INITIAL_MODE",      1),
    # Failsafes: disable for bench work (no RC transmitter, no GCS radio)
    ("FS_THR_ENABLE",     0),
    ("FS_GCS_ENABLE",     0),
    ("FS_EKF_ACTION",     0),
    # RPM sensor: disabled until AM32 EDT is enabled on the ESC.
    # After enabling EDT via BLHeli passthrough: set RPM1_TYPE=5 and SERVO_BLH_BDMASK=256.
    ("RPM1_TYPE",         0),
    ("RPM1_MIN",          0),
    # Output 9 (AUX OUT 1, FMU): GB4008 under Lua Script 1 control
    # SERVO9_MIN=800: motor off at 800 us, full throttle at 2000 us (matches rawes.lua range)
    # Default 1100 clamps the DShot signal and keeps motor silent.
    ("SERVO9_FUNCTION",   94),  # Script 1: Lua writes GB4008 PWM via SRV_Channels
    ("SERVO9_MIN",        800),
    ("SERVO9_MAX",        2000),
    ("SERVO9_TRIM",       800),   # trim = off
    # DShot300 on output 9 (AUX OUT 1, FMU processor -- BRD_IO_DSHOT not needed)
    ("SERVO_BLH_MASK",    256),  # bit 8 = output 9
    ("SERVO_BLH_OTYPE",   5),    # DShot300
    ("SERVO_BLH_POLES",   22),   # GB4008 24N22P; default 14 is wrong
    ("SERVO_BLH_TRATE",   10),   # telemetry request rate
    ("SERVO_BLH_AUTO",    0),    # manual mask config
    ("SERVO_BLH_BDMASK",  0),    # one-way DShot; set to 256 after AM32 EDT enabled
    ("SERVO_DSHOT_ESC",   3),    # AM32 (REVVitRC)
    ("SERVO_DSHOT_RATE",  0),    # 1 kHz (default)
    ("BRD_SAFETY_DEFLT",  0),    # safety switch disabled -- outputs live on boot
    # rawes.lua mode: 2 = yaw_lua (yaw-trim only)
    # SCR_USER6 is the mode selector.
    ("SCR_USER6",         2),
]

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
    current_fc = _recv_param(mav, "FRAME_CLASS")
    print(f"  Current FRAME_CLASS = {current_fc}")

    if current_fc != 6.0:
        print("  Setting FRAME_CLASS=6 (Traditional Helicopter) ...")
        ok, actual = _set_param(mav, "FRAME_CLASS", 6)
        if ok:
            print(f"  [OK]   FRAME_CLASS = {actual}")
        else:
            print(f"  [FAIL] Could not set FRAME_CLASS.  Aborting.")
            sys.exit(1)

        print()
        print("  Sending reboot command ...")
        _send_reboot(mav)
        _wait_disconnect(mav)
        mav.close()
        mav = _wait_reconnect(port)
    else:
        print("  FRAME_CLASS already 6 -- skipping reboot.")

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
