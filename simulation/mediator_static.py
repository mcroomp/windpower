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
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sitl_interface import SITLInterface
from mediator_base import install_sigterm_handler, setup_logging

log = logging.getLogger("mediator_static")


def main() -> None:
    setup_logging()
    is_stopped = install_sigterm_handler()

    parser = argparse.ArgumentParser(description="Static-sensor SITL mediator")
    parser.add_argument("--port",  type=int,   default=9002)
    parser.add_argument("--pos",   type=float, nargs=3, required=True,
                        metavar=("N", "E", "D"))
    parser.add_argument("--vel",   type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("vN", "vE", "vD"))
    parser.add_argument("--rpy",   type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("roll", "pitch", "yaw"))
    parser.add_argument("--accel", type=float, nargs=3, required=True,
                        metavar=("ax", "ay", "az"))
    parser.add_argument("--gyro",  type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("gx", "gy", "gz"))
    args = parser.parse_args()

    pos        = np.array(args.pos,   dtype=float)
    vel        = np.array(args.vel,   dtype=float)
    rpy        = np.array(args.rpy,   dtype=float)
    accel_body = np.array(args.accel, dtype=float)
    gyro       = np.array(args.gyro,  dtype=float)

    log.info("pos=%s  vel=%s  rpy=%s  accel=%s  gyro=%s", pos, vel, rpy, accel_body, gyro)
    log.info("Binding SITL JSON port %d ...", args.port)

    with SITLInterface(recv_port=args.port, watchdog_timeout=5.0) as sitl:
        log.info("Entering lockstep loop")
        frame = 0
        while not is_stopped():
            servos = sitl.recv_servos()
            if servos is None:
                if not is_stopped():
                    log.error("SITL servo watchdog expired -- ArduPilot stopped responding")
                    sys.exit(1)
                break
            sitl.send_state(
                pos_ned=pos, vel_ned=vel,
                rpy_rad=rpy, accel_body=accel_body, gyro_body=gyro,
            )
            frame += 1
            if frame % 4000 == 0:  # ~10 s at 400 Hz
                log.info("heartbeat: frame=%d  sim_t=%.1f s", frame, sitl.sim_now())

    log.info("exiting cleanly (frame=%d)", frame)


if __name__ == "__main__":
    main()
