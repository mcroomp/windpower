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
import csv
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
# Tether model
# ---------------------------------------------------------------------------
class TetherModel:
    """
    Tension-only elastic tether.

    Models a Dyneema SK75 1.9 mm braided tether attached from a fixed ground
    anchor to the hub node.  Force is zero when the tether is slack (hub
    closer to anchor than rest_length) and elastic when taut.

    Physical basis — Dyneema SK75 1.9 mm braided
    -----------------------------------------------
    Fibre:      UHMWPE (Dyneema SK75)
    Diameter:   1.9 mm (nominal)
    Area:       A = π × (0.00095 m)² = 2.835 × 10⁻⁶ m²
    Fibre E:    109 GPa  (DSM Dyneema SK75 datasheet)
    Braid eff.: ~0.91 (typical braid efficiency for a tight 12-strand braid)
    EA:         109 × 10⁹ × 2.835 × 10⁻⁶ × 0.91 ≈ 281 kN → use 280 kN
    Break load: ≈ 620 N  (DSM SK75, 2 mm; conservative for 1.9 mm)
    Lin. mass:  ρ_fibre × A × braid_fill = 970 × 2.835e-6 / 0.91 ≈ 3.0 g/m
                (fill factor inverted because braid is looser than solid rod)
                Practical value: ~2.1 g/m (manufacturer data for 2 mm SK75)

    Stiffness at operating length L
    --------------------------------
    k(L) = EA / L          [N/m]  — nonlinear: stiffer for shorter tether

    Structural damping
    ------------------
    UHMWPE loss tangent: δ ≈ 0.04 → ζ ≈ 0.02 (2 % critical)
    At L=200 m, m=5 kg:  c_crit = 2√(k·m) = 2√(1400·5) = 167 N·s/m
    c = ζ · c_crit = 0.02 × 167 ≈ 3.3 N·s/m → use 5 N·s/m (conservative)
    Damping is applied only while tether is extending (no negative damping).

    Scenario
    ---------
    rest_length = 200 m : tether paid out to 200 m by the ground winch.
    Hub starts at Z = 50 m ≪ 200 m → slack zone; no tether force at launch.
    Tether activates only if hub flies farther than 200 m from anchor.
    This represents a realistic AWE flight test scenario where the winch has
    pre-deployed 200 m of tether before the hub launches.
    """

    # Dyneema SK75 1.9 mm braided — material constants
    DIAMETER_M       = 0.0019           # nominal diameter [m]
    AREA_M2          = np.pi * (DIAMETER_M / 2) ** 2   # = 2.835e-6 m²
    E_FIBRE_PA       = 109e9            # fibre axial modulus [Pa]
    BRAID_EFFICIENCY = 0.91             # braid efficiency (axial stiffness fraction)
    EA_N             = E_FIBRE_PA * AREA_M2 * BRAID_EFFICIENCY  # ≈ 281 kN [N]
    LINEAR_MASS_KG_M = 0.0021          # linear mass density [kg/m]  (2.1 g/m)
    BREAK_LOAD_N     = 620.0           # conservative breaking load [N]

    def __init__(
        self,
        anchor_enu:  np.ndarray = np.array([0.0, 0.0, 0.0]),
        rest_length: float      = 200.0,   # unstretched tether length [m]
        EA:          float      = None,    # override axial stiffness [N]
        damping:     float      = 5.0,     # structural damping [N·s/m]
    ):
        self.anchor      = np.asarray(anchor_enu, dtype=float)
        self.rest_length = float(rest_length)
        self.EA          = float(EA) if EA is not None else self.EA_N
        self.damping     = float(damping)
        self._last_info: dict = {}

    def compute(
        self,
        hub_pos: np.ndarray,   # hub position in ENU world frame [m]
        hub_vel: np.ndarray,   # hub velocity in ENU world frame [m/s]
    ) -> np.ndarray:
        """
        Return tether force on hub in ENU world frame [N].
        Stores diagnostic info in self._last_info for telemetry logging.
        """
        r = hub_pos - self.anchor           # vector: anchor → hub
        L = float(np.linalg.norm(r))

        if L < 1e-6:
            self._last_info = dict(slack=True, tension=0.0, extension=0.0,
                                   length=0.0, k_eff=0.0)
            return np.zeros(3)

        unit = r / L                        # unit vector anchor → hub
        extension = L - self.rest_length

        if extension <= 0.0:               # tether is slack
            self._last_info = dict(slack=True, tension=0.0, extension=extension,
                                   length=L, k_eff=0.0)
            return np.zeros(3)

        # Stiffness: k = EA / L  (nonlinear — stiffer when shorter)
        k_eff = self.EA / L

        # Elastic tension
        T_elastic = k_eff * extension

        # Structural damping — only resists further extension
        v_radial = float(np.dot(hub_vel, unit))   # rate of length increase
        T_damp   = self.damping * max(0.0, v_radial)

        T_total  = T_elastic + T_damp

        # Force on hub: directed from hub toward anchor (opposes extension)
        force = -T_total * unit

        self._last_info = dict(
            slack        = False,
            tension      = T_total,
            t_elastic    = T_elastic,
            t_damp       = T_damp,
            extension    = extension,
            length       = L,
            k_eff        = k_eff,
        )
        return force


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
    parser.add_argument("--telemetry-log", default=None,
                        help="Path for per-step CSV telemetry log (MBDyn + aero data)")
    parser.add_argument("--tether-rest-length", type=float, default=200.0,
                        help="Unstretched tether length [m] (ground-winch pay-out). "
                             "Hub starts at Z=50m so rest_length=200m means "
                             "150m of slack at launch.")
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
    tether = TetherModel(
        anchor_enu  = np.array([0.0, 0.0, 0.0]),
        rest_length = args.tether_rest_length,
    )
    log.info(
        "Tether: Dyneema SK75 1.9mm  EA=%.0f kN  rest_length=%.0f m  "
        "damping=%.1f N·s/m  break_load=%.0f N",
        tether.EA / 1000, tether.rest_length,
        tether.damping, tether.BREAK_LOAD_N,
    )

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

    # -- Telemetry CSV --------------------------------------------------------
    _telemetry_file   = None
    _telemetry_writer = None
    if args.telemetry_log:
        _telemetry_file = open(args.telemetry_log, "w", newline="", encoding="utf-8")
        _telemetry_writer = csv.writer(_telemetry_file)
        _telemetry_writer.writerow([
            # Time
            "t_sim",
            # Servo inputs (normalized, -1..+1)
            "servo_s1", "servo_s2", "servo_s3", "servo_esc",
            # Swashplate mixing outputs
            "collective_norm", "tilt_lon", "tilt_lat", "collective_rad",
            # Rotor spin estimate
            "omega_rotor",
            # Aerodynamic model internals
            "aero_T", "aero_v_axial", "aero_v_i", "aero_v_inplane", "aero_ramp",
            "aero_Q_drag", "aero_Q_drive",
            # Aerodynamic forces sent to MBDyn (ENU world frame)
            "F_x", "F_y", "F_z", "M_x", "M_y", "M_z",
            # Hub state received from MBDyn (ENU world frame)
            "hub_pos_x", "hub_pos_y", "hub_pos_z",
            "hub_vel_x", "hub_vel_y", "hub_vel_z",
            "hub_omega_x", "hub_omega_y", "hub_omega_z",
            # Hub acceleration (finite-difference)
            "hub_accel_x", "hub_accel_y", "hub_accel_z",
            # Tether state
            "tether_length", "tether_extension", "tether_tension",
            "tether_fx", "tether_fy", "tether_fz",
            "tether_slack",
        ])
        log.info("Telemetry logging → %s", args.telemetry_log)

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
            # Step 4: Tether force (tension-only, Dyneema SK75 1.9 mm)
            # Computed in Python so we can enforce the slack condition
            # (a real rope cannot push).  Added to aerodynamic force before
            # sending to MBDyn.
            # ----------------------------------------------------------------
            tether_force = tether.compute(hub_state["pos"], hub_state["vel"])
            forces[0:3] += tether_force   # add to Fx/Fy/Fz; moments unchanged

            # Warn if tether tension approaches break load
            if tether._last_info.get("tension", 0) > 0.8 * tether.BREAK_LOAD_N:
                log.warning(
                    "t=%.1f TETHER TENSION %.0f N is >80%% of break load (%.0f N)!",
                    t_sim, tether._last_info["tension"], tether.BREAK_LOAD_N,
                )

            # ----------------------------------------------------------------
            # Step 4b: Thrust magnitude (for logging)
            # ----------------------------------------------------------------
            # Motor counter-torque is NOT applied here. In the current single-body
            # MBDyn model the GB4008 motor and bearing drag are internal forces
            # between the spinning hub and the non-rotating electronics assembly.
            # Internal forces cancel in a lumped-mass model and produce no net
            # external torque on the hub body.
            T_est = max(0.0, float(np.dot(forces[0:3], disk_normal)))

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
            # Step 7b: Write telemetry row
            # ----------------------------------------------------------------
            if _telemetry_writer is not None:
                _telemetry_writer.writerow([
                    f"{t_sim:.6f}",
                    f"{s1:.6f}", f"{s2:.6f}", f"{s3:.6f}", f"{esc_norm:.6f}",
                    f"{collective_norm:.6f}", f"{tilt_lon:.6f}", f"{tilt_lat:.6f}",
                    f"{collective_rad:.6f}",
                    f"{omega_rotor_est:.4f}",
                    f"{aero.last_T:.4f}",
                    f"{aero.last_v_axial:.4f}",
                    f"{aero.last_v_i:.4f}",
                    f"{aero.last_v_inplane:.4f}",
                    f"{aero.last_ramp:.4f}",
                    f"{aero.last_Q_drag:.4f}",
                    f"{aero.last_Q_drive:.4f}",
                    f"{forces[0]:.4f}", f"{forces[1]:.4f}", f"{forces[2]:.4f}",
                    f"{forces[3]:.4f}", f"{forces[4]:.4f}", f"{forces[5]:.4f}",
                    f"{hub_state['pos'][0]:.4f}", f"{hub_state['pos'][1]:.4f}", f"{hub_state['pos'][2]:.4f}",
                    f"{hub_state['vel'][0]:.4f}", f"{hub_state['vel'][1]:.4f}", f"{hub_state['vel'][2]:.4f}",
                    f"{hub_state['omega'][0]:.4f}", f"{hub_state['omega'][1]:.4f}", f"{hub_state['omega'][2]:.4f}",
                    f"{accel_world[0]:.4f}", f"{accel_world[1]:.4f}", f"{accel_world[2]:.4f}",
                    f"{tether._last_info.get('length', 0.0):.4f}",
                    f"{tether._last_info.get('extension', 0.0):.4f}",
                    f"{tether._last_info.get('tension', 0.0):.4f}",
                    f"{tether_force[0]:.4f}", f"{tether_force[1]:.4f}", f"{tether_force[2]:.4f}",
                    "1" if tether._last_info.get("slack", True) else "0",
                ])

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
                rpm_rad_s  = abs(omega_rotor_est),
            )

            # ----------------------------------------------------------------
            # Step 10: Periodic status log (1 Hz)
            # ----------------------------------------------------------------
            if t_sim - last_log_time >= LOG_INTERVAL:
                last_log_time = t_sim
                pos   = hub_state["pos"]
                rpy_d = np.degrees(sensor_data["rpy"])
                alt   = pos[2]
                teth = tether._last_info
                teth_str = (
                    f"SLACK  L={teth.get('length',0):.1f}m"
                    if teth.get("slack", True) else
                    f"TAUT  T={teth.get('tension',0):.0f}N  ext={teth.get('extension',0):.3f}m"
                )
                log.info(
                    "t=%6.1fs  pos_ENU=[%6.1f %6.1f %6.1f]m  "
                    "rpy=[%5.1f %5.1f %5.1f]deg  "
                    "omega=%.1f rad/s  T=%.1fN  tether=%s",
                    t_sim,
                    pos[0], pos[1], pos[2],
                    rpy_d[0], rpy_d[1], rpy_d[2],
                    omega_rotor_est,
                    T_est,
                    teth_str,
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

        if _telemetry_file is not None:
            try:
                _telemetry_file.flush()
                _telemetry_file.close()
                log.info("Telemetry log closed.")
            except Exception:
                pass

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
