"""
telemetry_csv.py -- Unified telemetry row for CSV read/write.

Single source of truth for the flat CSV schema used by simtests and analysis.
COLUMNS is the canonical ordered column list.  TelRow is the typed row object.
write_csv / read_csv handle I/O.

Simtests build rows via TelRow.from_physics(...) and call write_csv().
analyse_pumping_cycle.py reads via read_csv() instead of its own TelRow + loader.

Mediator-only columns (aero_*, servo_*, forces, rpy) default to 0.0 in rows
built from simtest telemetry.  tether_length and tether_extension are derived
from pos_ned so analysis scripts get correct geometry without the mediator's
tether model internals.

Array-valued properties
-----------------------
TelRow exposes logical groups as numpy arrays via read-only properties:

    r.pos_ned     -> np.ndarray [3]   hub position NED [m]
    r.vel_ned     -> np.ndarray [3]   hub velocity NED [m/s]
    r.R           -> np.ndarray [3,3] body-to-NED rotation matrix
    r.body_z_eq   -> np.ndarray [3]   equilibrium body-z setpoint (NED)
    r.wind_ned    -> np.ndarray [3]   ambient wind NED [m/s]

These are computed from the flat scalar fields on every access; they are not
stored separately, so the CSV remains flat and human-readable.
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


# ---------------------------------------------------------------------------
# Column order -- single source of truth
# ---------------------------------------------------------------------------

COLUMNS: list[str] = [
    "t_sim",
    "phase",               # "reel-out" | "reel-in" | "descent" | "final_drop" | ""
    "note",                # free-text event annotation (e.g. "kinematic_exit", "reel_out_start")
    "damp_alpha",          # startup damping blend factor [0..1]; 0.0 when not damping
    "pos_x", "pos_y", "pos_z",          # NED [m]; altitude = -pos_z
    "vel_x", "vel_y", "vel_z",          # NED [m/s]
    "omega_x", "omega_y", "omega_z",    # body angular rate [rad/s]
    "accel_x", "accel_y", "accel_z",    # NED acceleration [m/s^2] (mediator finite-diff)
    "omega_rotor",                       # rotor spin [rad/s]
    "tether_length", "tether_extension", # [m]
    "tether_tension", "tether_rest_length", "tether_slack",
    "tether_fx", "tether_fy", "tether_fz",  # tether force NED [N]
    "aero_fx", "aero_fy", "aero_fz",        # aero force NED [N] (before tether)
    "aero_mx", "aero_my", "aero_mz",        # aero orbital moment NED [N·m]
    "tether_mx", "tether_my", "tether_mz",  # tether moment NED [N·m]
    "collective_rad", "collective_norm",
    "tilt_lon", "tilt_lat",
    # Ground-commanded altitude [m] (TensionCommand.alt_m, set by PumpingGroundController)
    "tension_setpoint", "collective_from_tension_ctrl", "gnd_alt_cmd_m",
    "winch_speed_ms",
    "aero_T", "aero_v_axial", "aero_v_inplane", "aero_v_i",
    "aero_Q_drag", "aero_Q_drive",
    "F_x", "F_y", "F_z",                # net aero force NED [N]
    "M_x", "M_y", "M_z",                # net aero moment NED [N·m]
    "rpy_roll", "rpy_pitch", "rpy_yaw",
    # EKF yaw tracking diagnostics: difference between vel-heading yaw
    # (sent to ArduPilot as rpy_yaw) and actual R_orb orientation yaw.
    # Large delta -> EKF may see attitude/GPS inconsistency.
    "orb_yaw_rad",   # R_orb yaw before velocity-heading override [rad]
    "v_horiz_ms",    # horizontal speed [m/s]; selects yaw tracking source
    # Sensor consistency: what the mediator actually sends to SITL.
    # Use these to verify compass/GPS velocity heading alignment.
    # heading_gap_deg = |compass_deg - vel_heading_deg| wrapped to [-180,180].
    # EKF blocks GPS when |heading_gap_deg| is large (typically > ~90 deg).
    "sens_vel_n", "sens_vel_e", "sens_vel_d",         # GPS vel sent to SITL (NED) [m/s]
    "sens_accel_x", "sens_accel_y", "sens_accel_z",   # IMU accel sent to SITL (body) [m/s^2]
    "sens_gyro_x", "sens_gyro_y", "sens_gyro_z",      # IMU gyro sent to SITL (body) [rad/s]
    "vel_heading_deg",   # atan2(vel_E, vel_N) in degrees — GPS velocity heading
    "heading_gap_deg",   # |compass_deg - vel_heading_deg| wrapped to (-180,180] deg
    "servo_s1_us", "servo_s2_us", "servo_s3_us", "servo4_us",  # raw PWM [µs]
    "q_bearing_nm", "q_motor_nm", "throttle",
    "wind_x", "wind_y", "wind_z",
    "bz_eq_x", "bz_eq_y", "bz_eq_z",
    # TensionApController diagnostics (0.0 when not using that architecture)
    "elevation_rad",      # rate-limited elevation angle tracked by AP [rad]
    "el_correction_rad",  # daisy-chain elevation correction [rad]
    "coll_saturated",     # collective pinned at floor or ceiling (0/1)
    "comms_ok",           # ground comms healthy (0=dropout, 1=ok)
    # body-to-NED rotation matrix (row-major: r00=R[0,0], r01=R[0,1], ...)
    "r00", "r01", "r02",
    "r10", "r11", "r12",
    "r20", "r21", "r22",
]


# ---------------------------------------------------------------------------
# TelRow
# ---------------------------------------------------------------------------

@dataclass
class TelRow:
    t_sim: float = 0.0
    phase: str   = ""
    note:  str   = ""          # free-text event marker stamped once per event frame
    damp_alpha: float = 0.0   # startup damping blend [0..1]; 0.0 when free-flying

    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0
    vel_x: float = 0.0
    vel_y: float = 0.0
    vel_z: float = 0.0
    omega_x: float = 0.0
    omega_y: float = 0.0
    omega_z: float = 0.0

    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0

    omega_rotor: float = 0.0

    tether_length:      float = 0.0
    tether_extension:   float = 0.0
    tether_tension:     float = 0.0
    tether_rest_length: float = 0.0
    tether_slack:       int   = 0

    tether_fx: float = 0.0
    tether_fy: float = 0.0
    tether_fz: float = 0.0

    aero_fx: float = 0.0
    aero_fy: float = 0.0
    aero_fz: float = 0.0

    aero_mx: float = 0.0
    aero_my: float = 0.0
    aero_mz: float = 0.0

    tether_mx: float = 0.0
    tether_my: float = 0.0
    tether_mz: float = 0.0

    collective_rad:  float = 0.0
    collective_norm: float = 0.0
    tilt_lon:        float = 0.0
    tilt_lat:        float = 0.0
    tension_setpoint:             float = 0.0
    collective_from_tension_ctrl: float = 0.0
    gnd_alt_cmd_m:                float = 0.0   # ground-commanded altitude [m] (TensionCommand.alt_m)
    winch_speed_ms:               float = 0.0   # winch reel speed [m/s] +ve=out -ve=in

    aero_T:         float = 0.0
    aero_v_axial:   float = 0.0
    aero_v_inplane: float = 0.0
    aero_v_i:       float = 0.0
    aero_Q_drag:    float = 0.0
    aero_Q_drive:   float = 0.0

    F_x: float = 0.0
    F_y: float = 0.0
    F_z: float = 0.0

    M_x: float = 0.0
    M_y: float = 0.0
    M_z: float = 0.0

    rpy_roll:  float = 0.0
    rpy_pitch: float = 0.0
    rpy_yaw:   float = 0.0

    # EKF yaw diagnostics
    orb_yaw_rad: float = 0.0   # R_orb yaw (actual hub orientation) [rad]
    v_horiz_ms:  float = 0.0   # horizontal speed [m/s]

    # Sensor consistency: actual values sent to SITL
    sens_vel_n:  float = 0.0   # GPS vel North sent to SITL [m/s]
    sens_vel_e:  float = 0.0   # GPS vel East  sent to SITL [m/s]
    sens_vel_d:  float = 0.0   # GPS vel Down  sent to SITL [m/s]
    sens_accel_x: float = 0.0  # IMU accel X (body) sent to SITL [m/s^2]
    sens_accel_y: float = 0.0  # IMU accel Y (body) sent to SITL [m/s^2]
    sens_accel_z: float = 0.0  # IMU accel Z (body) sent to SITL [m/s^2]
    sens_gyro_x: float = 0.0   # IMU gyro X (body) sent to SITL [rad/s]
    sens_gyro_y: float = 0.0   # IMU gyro Y (body) sent to SITL [rad/s]
    sens_gyro_z: float = 0.0   # IMU gyro Z (body) sent to SITL [rad/s]
    vel_heading_deg:  float = 0.0  # atan2(vel_E, vel_N) [deg] — GPS heading
    heading_gap_deg:  float = 0.0  # compass - vel_heading wrapped to (-180,180] [deg]

    servo_s1_us: float = 0.0   # S1 swashplate servo PWM [µs]
    servo_s2_us: float = 0.0   # S2 swashplate servo PWM [µs]
    servo_s3_us: float = 0.0   # S3 swashplate servo PWM [µs]
    servo4_us:   float = 0.0   # SERVO4 / GB4008 motor PWM [µs]

    # Counter-torque motor (torque tests; 0.0 in flight tests)
    q_bearing_nm: float = 0.0   # reserved; always 0 (bearing drag absorbed by ESC)
    q_motor_nm:   float = 0.0
    throttle:     float = 0.0

    wind_x: float = 0.0
    wind_y: float = 0.0
    wind_z: float = 0.0

    bz_eq_x: float = 0.0
    bz_eq_y: float = 0.0
    bz_eq_z: float = 0.0

    # TensionApController diagnostics
    elevation_rad:     float = 0.0
    el_correction_rad: float = 0.0
    coll_saturated:    int   = 0
    comms_ok:          int   = 1

    # Body-to-NED rotation matrix (row-major)
    r00: float = 1.0
    r01: float = 0.0
    r02: float = 0.0
    r10: float = 0.0
    r11: float = 1.0
    r12: float = 0.0
    r20: float = 0.0
    r21: float = 0.0
    r22: float = 1.0

    # ------------------------------------------------------------------
    # Scalar convenience
    # ------------------------------------------------------------------

    @property
    def altitude(self) -> float:
        return -self.pos_z

    @property
    def orbit_radius(self) -> float:
        return math.sqrt(self.pos_x ** 2 + self.pos_y ** 2)

    def heartbeat(self, remaining_s: float = 0.0) -> str:
        """One-line status string for live log monitoring (7-bit ASCII only).

        Parameters
        ----------
        remaining_s : float
            Seconds remaining in the startup damping phase.  Only shown when
            damp_alpha > 0; ignored (and not shown) during free flight.
        """
        if self.damp_alpha > 0.0:
            # Kinematic phase: hub on fake trajectory, tether force not computed.
            teth = "N/A (kinematic)"
        elif self.tether_slack:
            teth = f"SLACK  L={self.tether_length:.1f}m"
        else:
            teth = f"TAUT  T={self.tether_tension:.0f}N  ext={self.tether_extension:.3f}m"
        damp = (
            f"  [DAMP a={self.damp_alpha:.2f} remaining={remaining_s:.0f}s]"
            if self.damp_alpha > 0.0 else ""
        )
        return (
            f"t={self.t_sim:6.1f}s  "
            f"pos_NED=[{self.pos_x:6.1f} {self.pos_y:6.1f} {self.pos_z:6.1f}]m  "
            f"alt={-self.pos_z:.2f}m  "
            f"rpy=[{math.degrees(self.rpy_roll):5.1f} {math.degrees(self.rpy_pitch):5.1f}"
            f" {math.degrees(self.rpy_yaw):5.1f}]deg  "
            f"omega={self.omega_rotor:.1f} rad/s  "
            f"T={self.tether_tension:.1f}N  "
            f"tether={teth}{damp}"
        )

    # ------------------------------------------------------------------
    # Array views (computed from flat fields; not stored separately)
    # ------------------------------------------------------------------

    @property
    def pos_ned(self) -> np.ndarray:
        """Hub position NED [m], shape (3,)."""
        return np.array([self.pos_x, self.pos_y, self.pos_z])

    @property
    def vel_ned(self) -> np.ndarray:
        """Hub velocity NED [m/s], shape (3,)."""
        return np.array([self.vel_x, self.vel_y, self.vel_z])

    @property
    def R(self) -> np.ndarray:
        """Body-to-NED rotation matrix, shape (3, 3)."""
        return np.array([
            [self.r00, self.r01, self.r02],
            [self.r10, self.r11, self.r12],
            [self.r20, self.r21, self.r22],
        ])

    @property
    def body_z_eq(self) -> np.ndarray:
        """Equilibrium body-z setpoint (NED unit vector), shape (3,)."""
        return np.array([self.bz_eq_x, self.bz_eq_y, self.bz_eq_z])

    @property
    def wind_ned(self) -> np.ndarray:
        """Ambient wind NED [m/s], shape (3,)."""
        return np.array([self.wind_x, self.wind_y, self.wind_z])

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------

    @classmethod
    def from_tel(cls, d: dict) -> "TelRow":
        """Construct from a make_tel() canonical dict."""
        pos  = d.get("pos_ned",   [0.0, 0.0, 0.0])
        vel  = d.get("vel_ned",   [0.0, 0.0, 0.0])
        wind = d.get("wind_ned",  [0.0, 0.0, 0.0])
        bzeq = d.get("body_z_eq") or [0.0, 0.0, 0.0]
        trl  = float(d.get("tether_rest_length", 0.0))
        tl   = math.sqrt(float(pos[0])**2 + float(pos[1])**2 + float(pos[2])**2)
        R_raw = d.get("R")
        R_mat = np.asarray(R_raw, dtype=float).reshape(3, 3) if R_raw is not None else np.eye(3)

        rpy  = d.get("rpy",        [0.0, 0.0, 0.0])
        om   = d.get("omega_body", [0.0, 0.0, 0.0])
        acc  = d.get("accel_world",[0.0, 0.0, 0.0])
        tf   = d.get("tether_force",[0.0, 0.0, 0.0])
        af   = d.get("aero_F",     [0.0, 0.0, 0.0])
        nf   = d.get("net_F",      [0.0, 0.0, 0.0])
        nm   = d.get("net_M",      [0.0, 0.0, 0.0])

        return cls(
            t_sim               = float(d.get("t_sim", 0.0)),
            phase               = str(d.get("phase", "")),
            pos_x               = float(pos[0]),
            pos_y               = float(pos[1]),
            pos_z               = float(pos[2]),
            vel_x               = float(vel[0]),
            vel_y               = float(vel[1]),
            vel_z               = float(vel[2]),
            omega_x             = float(om[0]),
            omega_y             = float(om[1]),
            omega_z             = float(om[2]),
            accel_x             = float(acc[0]),
            accel_y             = float(acc[1]),
            accel_z             = float(acc[2]),
            omega_rotor         = float(d.get("omega_rotor",      0.0)),
            tether_length       = tl,
            tether_extension    = tl - trl,
            tether_tension      = float(d.get("tether_tension",   0.0)),
            tether_rest_length  = trl,
            tether_slack        = int(d.get("tether_slack",       0)),
            tether_fx           = float(tf[0]),
            tether_fy           = float(tf[1]),
            tether_fz           = float(tf[2]),
            aero_fx             = float(af[0]),
            aero_fy             = float(af[1]),
            aero_fz             = float(af[2]),
            aero_mx             = float(d.get("aero_mx",           0.0)),
            aero_my             = float(d.get("aero_my",           0.0)),
            aero_mz             = float(d.get("aero_mz",           0.0)),
            tether_mx           = float(d.get("tether_mx",         0.0)),
            tether_my           = float(d.get("tether_my",         0.0)),
            tether_mz           = float(d.get("tether_mz",         0.0)),
            collective_rad      = float(d.get("collective_rad",   0.0)),
            tilt_lon            = float(d.get("tilt_lon",         0.0)),
            tilt_lat            = float(d.get("tilt_lat",         0.0)),
            tension_setpoint             = float(d.get("tension_setpoint",             0.0)),
            collective_from_tension_ctrl = float(d.get("collective_from_tension_ctrl", 0.0)),
            gnd_alt_cmd_m                = float(d.get("gnd_alt_cmd_m",                0.0)),
            winch_speed_ms               = float(d.get("winch_speed_ms",               0.0)),
            aero_T              = float(d.get("aero_T",           0.0)),
            aero_v_axial        = float(d.get("aero_v_axial",     0.0)),
            aero_v_inplane      = float(d.get("aero_v_inplane",   0.0)),
            aero_v_i            = float(d.get("aero_v_i",         0.0)),
            aero_Q_drag         = float(d.get("aero_Q_drag",      0.0)),
            aero_Q_drive        = float(d.get("aero_Q_drive",     0.0)),
            F_x                 = float(nf[0]),
            F_y                 = float(nf[1]),
            F_z                 = float(nf[2]),
            M_x                 = float(nm[0]),
            M_y                 = float(nm[1]),
            M_z                 = float(nm[2]),
            rpy_roll            = float(rpy[0]),
            rpy_pitch           = float(rpy[1]),
            rpy_yaw             = float(rpy[2]),
            wind_x              = float(wind[0]),
            wind_y              = float(wind[1]),
            wind_z              = float(wind[2]),
            bz_eq_x             = float(bzeq[0]),
            bz_eq_y             = float(bzeq[1]),
            bz_eq_z             = float(bzeq[2]),
            elevation_rad       = float(d.get("elevation_rad",     0.0)),
            el_correction_rad   = float(d.get("el_correction_rad", 0.0)),
            coll_saturated      = int(bool(d.get("coll_saturated", False))),
            comms_ok            = int(bool(d.get("comms_ok",        True))),
            r00=float(R_mat[0, 0]), r01=float(R_mat[0, 1]), r02=float(R_mat[0, 2]),
            r10=float(R_mat[1, 0]), r11=float(R_mat[1, 1]), r12=float(R_mat[1, 2]),
            r20=float(R_mat[2, 0]), r21=float(R_mat[2, 1]), r22=float(R_mat[2, 2]),
        )

    @classmethod
    def from_physics(
        cls,
        runner,
        step_result: dict,
        collective_rad: float,
        wind_ned,
        *,
        body_z_eq=None,
        phase: str = "",
        tension_setpoint: float = 0.0,
        collective_from_tension_ctrl: float = 0.0,
        gnd_alt_cmd_m: float = 0.0,
        winch_speed_ms: float = 0.0,
        elevation_rad: float = 0.0,
        el_correction_rad: float = 0.0,
        coll_saturated: bool = False,
        comms_ok: bool = True,
        net_moment=None,
        **extra,
    ) -> "TelRow":
        """Build a TelRow from a PhysicsRunner and its step result (simtest use).

        Parameters
        ----------
        runner       PhysicsRunner — provides hub_state, omega_spin, tether,
                     tension_now, t_sim, aero.
        step_result  dict returned by runner.step() —
                     provides tilt_lon, tilt_lat, aero_result, tether_force,
                     tether_moment, omega_body, accel_world.
        collective_rad  collective blade pitch used this step [rad].
        wind_ned     3-element array-like [vN, vE, vD] m/s.
        body_z_eq    optional NED unit vector (body-z setpoint).
        extra        ignored (forward-compat for test-specific diagnostics).
        """
        hub_state   = runner.hub_state
        tether      = runner.tether
        tension_now = runner.tension_now
        omega_spin  = runner.omega_spin
        t_sim       = runner.t_sim
        aero_obj    = runner.aero

        tilt_lon    = float(step_result.get("tilt_lon",  0.0))
        tilt_lat    = float(step_result.get("tilt_lat",  0.0))
        aero_result = step_result.get("aero_result")
        tether_force  = step_result.get("tether_force")
        tether_moment = step_result.get("tether_moment")
        omega_body  = step_result.get("omega_body")
        accel_world = step_result.get("accel_world")

        pos = np.asarray(hub_state["pos"], dtype=float)
        vel = np.asarray(hub_state["vel"], dtype=float)
        R   = np.asarray(hub_state["R"],   dtype=float)
        ti  = tether._last_info

        roll  = math.atan2(float(R[2, 1]), float(R[2, 2]))
        pitch = -math.asin(float(np.clip(R[2, 0], -1.0, 1.0)))
        yaw   = math.atan2(float(R[1, 0]), float(R[0, 0]))

        if omega_body is not None:
            om = np.asarray(omega_body, dtype=float)
        elif "omega" in hub_state:
            om = np.asarray(hub_state["omega"], dtype=float)
        else:
            om = np.zeros(3)

        acc = np.asarray(accel_world, dtype=float) if accel_world is not None else np.zeros(3)
        bzeq = np.asarray(body_z_eq, dtype=float).tolist() if body_z_eq is not None else [0.0, 0.0, 0.0]

        tf_raw = ti.get("force")
        if tether_force is not None:
            tf = np.asarray(tether_force, dtype=float)
        elif tf_raw is not None:
            tf = np.asarray(tf_raw, dtype=float)
        else:
            tf = np.zeros(3)

        aero_fx = aero_fy = aero_fz = 0.0
        aero_mx = aero_my = aero_mz = 0.0
        aero_T = aero_v_axial = aero_v_inplane = aero_v_i = aero_Q_drag = aero_Q_drive = 0.0
        net_F = np.zeros(3)
        net_M = np.zeros(3)
        if aero_result is not None:
            F = np.asarray(aero_result.F_world, dtype=float)
            M = np.asarray(aero_result.M_orbital, dtype=float)
            aero_fx, aero_fy, aero_fz = float(F[0]), float(F[1]), float(F[2])
            aero_mx, aero_my, aero_mz = float(M[0]), float(M[1]), float(M[2])
            net_F = F + tf
            if net_moment is not None:
                net_M = np.asarray(net_moment, dtype=float)
            if aero_obj is not None:
                aero_T         = float(aero_obj.last_T)
                aero_v_axial   = float(aero_obj.last_v_axial)
                aero_v_inplane = float(aero_obj.last_v_inplane)
                aero_v_i       = float(aero_obj.last_v_i)
                aero_Q_drag    = float(aero_obj.last_Q_drag)
                aero_Q_drive   = float(aero_obj.last_Q_drive)

        tm = np.asarray(tether_moment, dtype=float) if tether_moment is not None else np.zeros(3)

        trl = float(tether.rest_length)
        tl  = float(np.linalg.norm(pos))
        wind = [float(v) for v in wind_ned]

        return cls(
            t_sim               = float(t_sim),
            phase               = str(phase),
            pos_x               = float(pos[0]),
            pos_y               = float(pos[1]),
            pos_z               = float(pos[2]),
            vel_x               = float(vel[0]),
            vel_y               = float(vel[1]),
            vel_z               = float(vel[2]),
            omega_x             = float(om[0]),
            omega_y             = float(om[1]),
            omega_z             = float(om[2]),
            accel_x             = float(acc[0]),
            accel_y             = float(acc[1]),
            accel_z             = float(acc[2]),
            omega_rotor         = float(omega_spin),
            tether_length       = tl,
            tether_extension    = tl - trl,
            tether_tension      = float(tension_now),
            tether_rest_length  = trl,
            tether_slack        = int(bool(ti.get("slack", False))),
            tether_fx           = float(tf[0]),
            tether_fy           = float(tf[1]),
            tether_fz           = float(tf[2]),
            aero_fx             = aero_fx,
            aero_fy             = aero_fy,
            aero_fz             = aero_fz,
            aero_mx             = aero_mx,
            aero_my             = aero_my,
            aero_mz             = aero_mz,
            tether_mx           = float(tm[0]),
            tether_my           = float(tm[1]),
            tether_mz           = float(tm[2]),
            collective_rad      = float(collective_rad),
            tilt_lon            = float(tilt_lon),
            tilt_lat            = float(tilt_lat),
            tension_setpoint             = float(tension_setpoint),
            collective_from_tension_ctrl = float(collective_from_tension_ctrl),
            gnd_alt_cmd_m                = float(gnd_alt_cmd_m),
            winch_speed_ms               = float(winch_speed_ms),
            aero_T              = aero_T,
            aero_v_axial        = aero_v_axial,
            aero_v_inplane      = aero_v_inplane,
            aero_v_i            = aero_v_i,
            aero_Q_drag         = aero_Q_drag,
            aero_Q_drive        = aero_Q_drive,
            F_x                 = float(net_F[0]),
            F_y                 = float(net_F[1]),
            F_z                 = float(net_F[2]),
            M_x                 = float(net_M[0]),
            M_y                 = float(net_M[1]),
            M_z                 = float(net_M[2]),
            rpy_roll            = roll,
            rpy_pitch           = pitch,
            rpy_yaw             = yaw,
            wind_x              = wind[0],
            wind_y              = wind[1],
            wind_z              = wind[2],
            bz_eq_x             = float(bzeq[0]),
            bz_eq_y             = float(bzeq[1]),
            bz_eq_z             = float(bzeq[2]),
            elevation_rad       = float(elevation_rad),
            el_correction_rad   = float(el_correction_rad),
            coll_saturated      = int(bool(coll_saturated)),
            comms_ok            = int(bool(comms_ok)),
            r00=float(R[0, 0]), r01=float(R[0, 1]), r02=float(R[0, 2]),
            r10=float(R[1, 0]), r11=float(R[1, 1]), r12=float(R[1, 2]),
            r20=float(R[2, 0]), r21=float(R[2, 1]), r22=float(R[2, 2]),
        )

    def to_dict(self) -> dict:
        return {col: getattr(self, col) for col in COLUMNS}


# ---------------------------------------------------------------------------
# CSV I/O
# ---------------------------------------------------------------------------

def write_csv(rows: "list[TelRow]", path: "Path | str") -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS)
        w.writeheader()
        for row in rows:
            w.writerow(row.to_dict())


def read_csv(path: "Path | str") -> "list[TelRow]":
    path = Path(path)
    if not path.exists():
        return []
    rows: list[TelRow] = []
    with path.open(encoding="utf-8", errors="replace") as fh:
        for raw in csv.DictReader(fh):
            row = _row_from_raw(raw)
            if row is not None:
                rows.append(row)
    return rows


# ---------------------------------------------------------------------------
# Internal
# ---------------------------------------------------------------------------

_STR_COLS   = frozenset({"phase", "note"})
_INT_COLS   = frozenset({"tether_slack", "coll_saturated", "comms_ok"})
_FLOAT_COLS = [c for c in COLUMNS if c not in _STR_COLS and c not in _INT_COLS and c != "t_sim"]


def _f(s: object) -> Optional[float]:
    if s in (None, "", "None", "nan", "inf", "-inf"):
        return None
    try:
        return float(s)  # type: ignore[arg-type]
    except (ValueError, TypeError):
        return None


def _row_from_raw(raw: dict) -> Optional[TelRow]:
    t = _f(raw.get("t_sim"))
    if t is None:
        return None
    r = TelRow(t_sim=t)
    r.phase = str(raw.get("phase") or "").strip()
    r.note  = str(raw.get("note")  or "").strip()
    v = _f(raw.get("tether_slack"))
    if v is not None:
        r.tether_slack = int(v)
    for col in _FLOAT_COLS:
        v = _f(raw.get(col))
        if v is not None:
            setattr(r, col, v)
    return r
