#!/usr/bin/env python3
"""
sitl_bench.py -- Interactive SITL bench session for calibrate.py.

Boots the same stack as the torque stack tests (SITL --model JSON +
mediator_torque.py) but with omega_rotor=0 (rotor stationary) and a very
long startup hold so the mediator never transitions to DYNAMIC.

This lets calibrate.py connect over tcp:localhost:5760 from the Windows host
and exercise rawes.lua / servo commands / arming exactly as on hardware.

Usage (run inside the rawes-sitl Docker container via 'bash test.sh sitl'):
    python3 /rawes/simulation/scripts/sitl_bench.py [--omega-rotor RAD_S]

Connect from Windows host (second terminal):
    calibrate.py --port sitl
"""
from __future__ import annotations

import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
from pathlib import Path

# ---------------------------------------------------------------------------
# Path setup — reach simulation/ and tests/sitl/ from any CWD
# ---------------------------------------------------------------------------
_SCRIPTS_DIR = Path(__file__).resolve().parent          # simulation/scripts/
_SIM_DIR     = _SCRIPTS_DIR.parent                      # simulation/
_SITL_DIR    = _SIM_DIR / "tests" / "sitl"

for _p in (str(_SIM_DIR), str(_SITL_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from stack_utils import (   # noqa: E402
    ParamSetup,
    _launch_sitl,
    _prime_sitl_eeprom,
    _resolve_sim_vehicle,
    _launch_mediator_torque,
    _terminate_process,
    _kill_by_port,
)

# ---------------------------------------------------------------------------
# Boot parameters (mirrors _BASE_TORQUE_BOOT_PARAMS + _LUA_TORQUE_EXTRA_PARAMS
# from stack_infra.py — single source of truth is stack_infra; keep in sync)
# ---------------------------------------------------------------------------
_BOOT_PARAMS: dict[str, float] = {
    # Mode / failsafes
    "INITIAL_MODE":       1,     # ACRO from first boot
    "FS_EKF_ACTION":      0,
    "FS_THR_ENABLE":      0,
    # EKF — compass yaw, no GPS position/velocity
    "COMPASS_USE":        1,
    "COMPASS_ENABLE":     1,
    "EK3_SRC1_YAW":      1,     # compass yaw
    "EK3_MAG_CAL":        0,     # no auto-calibration
    "EK3_GPS_CHECK":      0,
    "EK3_SRC1_POSXY":     0,
    "EK3_SRC1_VELXY":     0,
    # Yaw PID — torque-test tuning (matches _BASE_TORQUE_BOOT_PARAMS)
    "ATC_RAT_YAW_P":      0.015,
    "ATC_RAT_YAW_I":      0.01,
    "ATC_RAT_YAW_D":      0.0,
    "ATC_RAT_YAW_IMAX":   0.7,
    "ATC_RAT_RLL_IMAX":   0.0,
    "ATC_RAT_PIT_IMAX":   0.0,
    # DDFP CW tail — GB4008 anti-rotation motor (US-convention rotor)
    "H_TAIL_TYPE":        3,
    "H_COL2YAW":          0.0,
    "SERVO4_MIN":         800,
    "SERVO4_MAX":         2000,
    "SERVO4_TRIM":        800,
    "H_YAW_TRIM":         0.02,
    # RSC — CH8 passthrough (instant runup when CH8=2000)
    "H_RSC_MODE":         1,
    "H_RSC_RUNUP_TIME":   2,
    # Lua scripting
    "SCR_ENABLE":         1,
    "SCR_USER6":          0,     # MODE_NONE at boot; calibrate.py sets mode
    # Arming — disable prearm checks so force-arm from calibrate.py works
    "ARMING_CHECK":       0,
}

# Very long startup hold so mediator_torque never transitions from STARTUP to
# DYNAMIC.  The physics backend stays alive (answering JSON packets) without
# actually spinning up the rotor.
_STARTUP_HOLD_S = 86400.0   # 24 hours


def _write_parm_file(path: Path, params: dict) -> None:
    with path.open("w") as f:
        for k, v in params.items():
            f.write(f"{k} {v}\n")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--omega-rotor", type=float, default=0.0,
                    help="Rotor angular velocity [rad/s] (default: 0 = stationary)")
    ap.add_argument("--sim-vehicle", default="/ardupilot/Tools/autotest/sim_vehicle.py",
                    help="Path to sim_vehicle.py")
    args = ap.parse_args()

    os.environ.setdefault("RAWES_SIM_VEHICLE", args.sim_vehicle)

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        sys.exit("[sitl_bench] sim_vehicle.py not found. "
                 "Set RAWES_SIM_VEHICLE or RAWES_ARDUPILOT_PATH.")

    # Install rawes.lua into the ArduPilot scripts directory
    scripts_dst = Path("/ardupilot/scripts")
    scripts_dst.mkdir(exist_ok=True)
    shutil.copy2(_SCRIPTS_DIR / "rawes.lua", scripts_dst / "rawes.lua")
    print(f"[sitl_bench] rawes.lua -> {scripts_dst}/rawes.lua")

    # Write boot param file to a temp directory
    tmp = Path(tempfile.mkdtemp(prefix="sitl_bench_"))
    boot_parm = tmp / "boot_params.parm"
    _write_parm_file(boot_parm, _BOOT_PARAMS)
    print(f"[sitl_bench] boot params: {len(_BOOT_PARAMS)} entries -> {boot_parm}")

    # Prime EEPROM: boot once with heli model to flush SCR_ENABLE=1, then kill.
    # _prime_sitl_eeprom wipes eeprom.bin before the prime boot, then kills SITL.
    print("[sitl_bench] Priming EEPROM (wipe + prime boot) ...")
    _prime_sitl_eeprom(sim_vehicle, add_param_file=boot_parm)
    print("[sitl_bench] EEPROM primed.")

    # Launch SITL (--model JSON, no eeprom wipe — eeprom is already primed)
    sitl_log = tmp / "sitl.log"
    print("[sitl_bench] Launching SITL (--model JSON) ...")
    sitl_proc = _launch_sitl(sim_vehicle, sitl_log,
                             add_param_file=boot_parm, wipe_eeprom=False)

    # Launch mediator_torque (omega_rotor=0, startup hold = 24 h)
    mediator_log = tmp / "mediator.log"
    print(f"[sitl_bench] Launching mediator_torque "
          f"(omega_rotor={args.omega_rotor:.2f} rad/s = "
          f"{args.omega_rotor * 60 / (2 * math.pi):.0f} RPM) ...")
    mediator_proc = _launch_mediator_torque(
        _SIM_DIR, _SIM_DIR.parent,
        mediator_log, args.omega_rotor,
        profile="constant",
        tail_channel=3,
        startup_hold_s=_STARTUP_HOLD_S,
    )

    print()
    print("  =====================================================")
    print("  SITL running on  tcp:0.0.0.0:5760")
    print()
    print("  Connect from Windows host (second terminal):")
    print("    calibrate.py --port sitl")
    print()
    print(f"  Logs: {tmp}/")
    print("  Ctrl-C to stop.")
    print("  =====================================================")
    print()

    # Tail SITL log to terminal so Lua startup messages and STATUSTEXT are visible
    tail_proc = subprocess.Popen(
        ["tail", "-F", str(sitl_log)],
        stdout=sys.stdout, stderr=subprocess.DEVNULL,
    )

    procs = [sitl_proc, mediator_proc, tail_proc]

    def _cleanup(signum=None, frame=None):
        print("\n[sitl_bench] Shutting down ...")
        for p in procs:
            try:
                _terminate_process(p)
            except Exception:
                pass
        _kill_by_port(5760, "tcp")
        _kill_by_port(9002, "udp")
        sys.exit(0)

    signal.signal(signal.SIGINT,  _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    # Wait — exit if either key process dies
    import time
    while True:
        time.sleep(2.0)
        for name, proc in [("SITL", sitl_proc), ("mediator_torque", mediator_proc)]:
            if proc.poll() is not None:
                print(f"[sitl_bench] {name} exited (rc={proc.returncode}) — shutting down.")
                _cleanup()


if __name__ == "__main__":
    main()
