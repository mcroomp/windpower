#!/usr/bin/env python3
"""
RAWES Simulation Mediator
Bridges ArduPilot SITL <-> MBDyn for end-to-end flight simulation.

Architecture:
    ArduPilot SITL  <--UDP 9002/9003-->  mediator.py  <--UNIX sock-->  MBDyn

Data flow (each 400 Hz step):
    1. recv_servos()     — non-blocking, from SITL UDP port 9002
    2. swashplate mix    — servo PWM -> collective + cyclic tilt
    3. aero.compute()    — BEM forces in world (ENU) frame
    4. add motor torque  — anti-rotation GB4008 moment (servo[3])
    5. send_forces()     — to MBDyn via UNIX socket
    6. recv_state()      — from MBDyn via UNIX socket (18 float64)
    7. sensor.compute()  — world state -> IMU + NED outputs
    8. send_state()      — to SITL UDP port 9003

Usage:
    python3 mediator.py [options]

Options:
    --mbdyn-force-sock PATH    MBDyn forces socket  (default: /tmp/rawes_forces.sock)
    --mbdyn-state-sock PATH    MBDyn state socket   (default: /tmp/rawes_state.sock)
    --sitl-recv-port  PORT     SITL servo recv port  (default: 9002)
    --sitl-send-port  PORT     SITL state send port  (default: 9003)
    --wind-x  FLOAT            Initial wind E [m/s]  (default: 10.0)
    --wind-y  FLOAT            Initial wind N [m/s]  (default: 0.0)
    --wind-z  FLOAT            Initial wind U [m/s]  (default: 0.0)
    --log-level LEVEL          Logging level         (default: INFO)
"""

import argparse
import logging
import sys
import time
import os

import numpy as np

# Local modules (same directory)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mbdyn_interface import MBDynInterface
from sitl_interface  import SITLInterface
from swashplate      import h3_inverse_mix, collective_to_pitch, cyclic_to_blade_pitches, pwm_to_normalized
from aero            import RotorAero
from sensor          import SensorSim

# ---------------------------------------------------------------------------
# Configuration defaults
# ---------------------------------------------------------------------------
DT_TARGET    = 1.0 / 400.0    # 400 Hz target loop rate [s]
LOG_INTERVAL = 1.0             # position/attitude log interval [s]
GRAVITY_COMP = np.array([0.0, 0.0, 49.05, 0.0, 0.0, 0.0])   # initial forces [N]
WEIGHT_N     = 49.05           # rotor weight [N] = 5 kg * 9.81 m/s²


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
def _parse_args():
    parser = argparse.ArgumentParser(
        description="RAWES Simulation Mediator — bridges ArduPilot SITL and MBDyn",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--mbdyn-force-sock", default="/tmp/rawes_forces.sock",
                        help="Path to MBDyn forces UNIX socket")
    parser.add_argument("--mbdyn-state-sock", default="/tmp/rawes_state.sock",
                        help="Path to MBDyn state UNIX socket")
    parser.add_argument("--sitl-recv-port", type=int, default=9002,
                        help="UDP port to listen on for SITL servo data")
    parser.add_argument("--sitl-send-port", type=int, default=9003,
                        help="UDP port to send physics state to SITL")
    parser.add_argument("--wind-x", type=float, default=10.0,
                        help="Ambient wind East component [m/s]")
    parser.add_argument("--wind-y", type=float, default=0.0,
                        help="Ambient wind North component [m/s]")
    parser.add_argument("--wind-z", type=float, default=0.0,
                        help="Ambient wind Up component [m/s]")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                        help="Logging verbosity level")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main mediator loop
# ---------------------------------------------------------------------------
def run_mediator(args):
    log = logging.getLogger("mediator")

    wind_world = np.array([args.wind_x, args.wind_y, args.wind_z])
    log.info("Wind vector (ENU): %s m/s", wind_world)

    # -- Instantiate subsystems -----------------------------------------------
    mbdyn  = MBDynInterface(
        force_sock_path=args.mbdyn_force_sock,
        state_sock_path=args.mbdyn_state_sock,
    )
    sitl   = SITLInterface(
        recv_port=args.sitl_recv_port,
        send_port=args.sitl_send_port,
    )
    aero   = RotorAero()
    sensor = SensorSim()

    # -- Connect ---------------------------------------------------------------
    log.info("Binding SITL UDP sockets...")
    sitl.bind()

    log.info("Connecting to MBDyn sockets (waiting up to 30s) ...")
    mbdyn.connect()
    log.info("All connections established. Starting main loop.")

    # -- State ----------------------------------------------------------------
    t_sim           = 0.0
    step            = 0
    last_log_time   = -LOG_INTERVAL       # ensure immediate first log
    last_vel        = np.zeros(3)
    hub_state       = {
        "pos":   np.array([0.0, 0.0, 50.0]),
        "vel":   np.zeros(3),
        "R":     np.eye(3),
        "omega": np.array([0.0, 0.0, 28.0]),
    }
    last_servos     = np.zeros(16)        # neutral PWM (mapped from 1500µs → 0)
    # Estimated rotor spin (maintained from initial condition)
    omega_rotor_est = 28.0                # rad/s

    # -- Warm-up: send gravity compensation to MBDyn before SITL connects -----
    log.info("Sending initial gravity-compensation forces ...")
    mbdyn.send_forces(GRAVITY_COMP)

    # Read initial state from MBDyn
    try:
        hub_state = mbdyn.recv_state()
        log.info("Initial MBDyn state: pos=%s vel=%s",
                 hub_state["pos"].round(2), hub_state["vel"].round(2))
    except Exception as exc:
        log.warning("Could not read initial state: %s", exc)

    # -- Main loop ------------------------------------------------------------
    log.info("Entering main loop at %.0f Hz target.", 1.0 / DT_TARGET)

    try:
        t_wall_start = time.monotonic()

        while True:
            t_wall_loop = time.monotonic()
            t_sim = t_wall_loop - t_wall_start

            # ----------------------------------------------------------------
            # Step 1: Receive servo values from SITL (non-blocking)
            # ----------------------------------------------------------------
            new_servos = sitl.recv_servos()
            if new_servos is not None:
                last_servos = new_servos
            # Use last known servo values if no new packet
            servos = last_servos

            # ----------------------------------------------------------------
            # Step 2: Swashplate mixing
            # Servo channels 0,1,2 = S1,S2,S3 (swashplate); channel 3 = ESC
            # Servos arrive normalised [-1,1] from SITLInterface
            # ----------------------------------------------------------------
            s1 = float(servos[0])
            s2 = float(servos[1])
            s3 = float(servos[2])
            esc_norm = float(np.clip(servos[3], -1.0, 1.0))

            collective_norm, tilt_lon, tilt_lat = h3_inverse_mix(s1, s2, s3)
            collective_rad = collective_to_pitch(collective_norm)

            # ----------------------------------------------------------------
            # Step 3: Compute aerodynamic forces
            # Use current hub state from previous MBDyn step
            # ----------------------------------------------------------------
            R_hub    = hub_state["R"]
            v_hub    = hub_state["vel"]
            omega_w  = hub_state["omega"]    # world-frame angular velocity

            # Estimate scalar rotor spin from omega about world Z projected on
            # the rotor disk normal (hub body Z-axis in world)
            disk_normal  = R_hub[:, 2]
            omega_rotor_est = float(np.dot(omega_w, disk_normal))
            if abs(omega_rotor_est) < 1.0:
                omega_rotor_est = 28.0   # fallback to nominal until MBDyn settles

            forces = aero.compute_forces(
                collective_rad = collective_rad,
                tilt_lon       = tilt_lon,
                tilt_lat       = tilt_lat,
                R_hub          = R_hub,
                v_hub_world    = v_hub,
                omega_rotor    = omega_rotor_est,
                wind_world     = wind_world,
                t              = t_sim,
            )

            # ----------------------------------------------------------------
            # Step 4: Add anti-rotation motor torque (Mz)
            # GB4008 counter-torque keeps electronics/swashplate stationary
            # ----------------------------------------------------------------
            # Estimate current thrust magnitude
            T_est = max(0.0, float(np.dot(forces[0:3], disk_normal)))
            mz_motor = aero.compute_anti_rotation_moment(
                esc_normalized = esc_norm,
                omega_rotor    = omega_rotor_est,
                T              = T_est,
            )
            forces[5] += mz_motor   # add to world Mz

            # ----------------------------------------------------------------
            # Step 5: Send forces to MBDyn
            # ----------------------------------------------------------------
            mbdyn.send_forces(forces)

            # ----------------------------------------------------------------
            # Step 6: Receive new hub state from MBDyn
            # ----------------------------------------------------------------
            prev_vel  = hub_state["vel"].copy()
            hub_state = mbdyn.recv_state()
            new_vel   = hub_state["vel"]

            # ----------------------------------------------------------------
            # Step 7: Compute hub acceleration (finite difference)
            # ----------------------------------------------------------------
            dt_actual = DT_TARGET   # use nominal dt; wall-clock varies
            accel_world = (new_vel - prev_vel) / dt_actual

            # ----------------------------------------------------------------
            # Step 8: Compute sensor outputs
            # ----------------------------------------------------------------
            sensor_data = sensor.compute(
                pos_enu         = hub_state["pos"],
                vel_enu         = hub_state["vel"],
                R_hub           = hub_state["R"],
                omega_body      = hub_state["omega"],
                accel_world_enu = accel_world,
                dt              = dt_actual,
            )

            # ----------------------------------------------------------------
            # Step 9: Send physics state to SITL
            # ----------------------------------------------------------------
            sitl.send_state(
                timestamp  = t_sim,
                pos_ned    = sensor_data["pos_ned"],
                vel_ned    = sensor_data["vel_ned"],
                rpy_rad    = sensor_data["rpy"],
                accel_body = sensor_data["accel_body"],
                gyro_body  = sensor_data["gyro_body"],
            )

            # ----------------------------------------------------------------
            # Step 10: Periodic status log (1 Hz)
            # ----------------------------------------------------------------
            if t_sim - last_log_time >= LOG_INTERVAL:
                last_log_time = t_sim
                pos   = hub_state["pos"]
                rpy_d = np.degrees(sensor_data["rpy"])
                alt   = pos[2]
                log.info(
                    "t=%6.1fs  pos_ENU=[%6.1f %6.1f %6.1f]m  "
                    "rpy=[%5.1f %5.1f %5.1f]deg  "
                    "omega=%.1f rad/s  T=%.1fN",
                    t_sim,
                    pos[0], pos[1], pos[2],
                    rpy_d[0], rpy_d[1], rpy_d[2],
                    omega_rotor_est,
                    T_est,
                )

            # ----------------------------------------------------------------
            # Timing: sleep remainder of target period
            # ----------------------------------------------------------------
            elapsed = time.monotonic() - t_wall_loop
            remaining = DT_TARGET - elapsed
            if remaining > 0:
                time.sleep(remaining)

            step += 1

    except KeyboardInterrupt:
        log.info("KeyboardInterrupt received — shutting down.")

    except Exception as exc:
        log.exception("Fatal error in main loop: %s", exc)
        raise

    finally:
        # -- Graceful shutdown ------------------------------------------------
        log.info("Final state at step %d (t=%.2f s):", step, t_sim)
        log.info("  pos_ENU : %s m",  hub_state["pos"].round(3))
        log.info("  vel_ENU : %s m/s", hub_state["vel"].round(3))
        log.info("  omega   : %.2f rad/s", omega_rotor_est)

        log.info("Closing connections ...")
        try:
            mbdyn.close()
        except Exception:
            pass
        try:
            sitl.close()
        except Exception:
            pass
        log.info("Mediator stopped.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    args = _parse_args()
    logging.basicConfig(
        level    = getattr(logging, args.log_level),
        format   = "%(asctime)s %(levelname)-8s %(name)s: %(message)s",
        datefmt  = "%H:%M:%S",
    )
    run_mediator(args)


if __name__ == "__main__":
    main()
