"""
mediator_static.py -- static-sensor SITL mediator for stack tests.

Runs the ArduPilot SITL lockstep loop with fixed sensor values.
Used by tests that need ArduPilot running (EKF alignment, arm sequence,
GPS fusion) but do not require full physics simulation.

Exits with code 1 if ArduPilot stops responding (servo watchdog expired).
Exits cleanly on SIGTERM.

Usage
-----
  python mediator_static.py \\
      --pos  N E D    \\
      --vel  vN vE vD \\
      --rpy  roll pitch yaw \\
      --accel ax ay az \\
      --gyro  gx gy gz \\
      [--port 9002]
      [--events-log path/to/events.jsonl]
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mediator_base import install_sigterm_handler, run_lockstep, setup_logging
from mediator_events import MediatorEventLog
from sitl_interface import SITLInterface

log = logging.getLogger("mediator_static")


def main() -> None:
    setup_logging()
    is_stopped = install_sigterm_handler()

    parser = argparse.ArgumentParser(description="Static-sensor SITL mediator")
    parser.add_argument("--port",       type=int,   default=9002)
    parser.add_argument("--pos",        type=float, nargs=3, required=True,
                        metavar=("N", "E", "D"))
    parser.add_argument("--vel",        type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("vN", "vE", "vD"))
    parser.add_argument("--rpy",        type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("roll", "pitch", "yaw"))
    parser.add_argument("--accel",      type=float, nargs=3, required=True,
                        metavar=("ax", "ay", "az"))
    parser.add_argument("--gyro",       type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("gx", "gy", "gz"))
    parser.add_argument("--events-log", default=None,
                        help="Path for structured JSONL event log")
    args = parser.parse_args()

    pos        = np.array(args.pos,   dtype=float)
    vel        = np.array(args.vel,   dtype=float)
    rpy        = np.array(args.rpy,   dtype=float)
    accel_body = np.array(args.accel, dtype=float)
    gyro       = np.array(args.gyro,  dtype=float)

    ev = MediatorEventLog(args.events_log)
    ev.open()

    # step_fn ignores servo values — sensor output is always fixed
    _state = dict(pos_ned=pos, vel_ned=vel, rpy_rad=rpy,
                  accel_body=accel_body, gyro_body=gyro)

    def step_fn(_servos, _t):
        return _state

    n = 0
    try:
        log.info("pos=%s  vel=%s  rpy=%s  accel=%s  gyro=%s",
                 pos, vel, rpy, accel_body, gyro)
        ev.write("startup", t_sim=0.0,
                 pos=pos.tolist(), vel=vel.tolist(), rpy=rpy.tolist())
        with SITLInterface(recv_port=args.port, watchdog_timeout=5.0) as sitl:
            n = run_lockstep(sitl, step_fn, is_stopped, log=log, ev=ev)
    except KeyboardInterrupt:
        log.info("Interrupted -- shutting down")
    finally:
        ev.write("shutdown", t_sim=0.0, frames=n)
        ev.close()

    log.info("exiting cleanly (frames=%d)", n)


if __name__ == "__main__":
    main()
