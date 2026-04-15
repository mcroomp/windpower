"""
test_gps_fusion_layers.py -- Layer-by-layer GPS fusion diagnostic.

Static sensor worker, no mediator, no dynamics.  Each layer changes one
variable from the baseline to isolate what blocks GPS fusion.

Layer progression:
  L0  Level vehicle, yaw=0, vel=[0.1,0,0] North  -- gap=0 deg  (expect PASS)
  L1  RAWES tilt (65 deg), yaw=0, vel North       -- gap=0 deg  (expect PASS)
  L2  RAWES tilt + RAWES yaw=136.6, vel aligned   -- gap=0 deg  (expect PASS)
  L3  RAWES tilt + yaw=136.6, vel=current cfg     -- gap=152 deg (may fail)
  L4  RAWES tilt + yaw=136.6, vel=East (90 deg)   -- gap=46.6 deg (may fail)
  L5  RAWES tilt + yaw=136.6, vel=North (0 deg)   -- gap=136.6 deg (may fail)

Also: test_gps_fusion_armed -- same L0 sensor, but arms vehicle first,
  then waits for GPS fusion.  Tests hypothesis that GPS fusion requires
  armed state.

Run:
  bash simulation/dev.sh test-stack -v -k test_gps_fusion_layers
  bash simulation/dev.sh test-stack -v -k "test_gps_fusion_layers[L0"
  bash simulation/dev.sh test-stack -v -k test_gps_fusion_armed
"""
from __future__ import annotations

import logging
import math
import sys
import threading
import time
from pathlib import Path

import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

import json

from stack_infra import StackConfig, _sitl_stack
from gcs import RawesGCS
from sitl_interface import SITLInterface
from telemetry_csv import TelRow, write_csv

_STEADY_STATE_JSON = _SIM_DIR / "steady_state_starting.json"

log = logging.getLogger("test_gps_layers")

_GRAVITY = 9.81

# ---------------------------------------------------------------------------
# Rotation matrix helpers
# ---------------------------------------------------------------------------

def _euler_to_R(roll: float, pitch: float, yaw: float):
    """Body-to-NED rotation matrix from ZYX Euler angles [rad]."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return [
        [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,     cp*sr,             cp*cr            ],
    ]


def _accel_body_from_rpy_deg(roll_deg, pitch_deg, yaw_deg):
    """
    IMU specific force at rest in body frame = R^T @ [0, 0, -9.81].
    (NED: gravity = +Z, specific force at rest = -Z in world frame.)
    """
    R = _euler_to_R(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
    g = [0.0, 0.0, -_GRAVITY]
    return [
        R[0][0]*g[0] + R[1][0]*g[1] + R[2][0]*g[2],
        R[0][1]*g[0] + R[1][1]*g[1] + R[2][1]*g[2],
        R[0][2]*g[0] + R[1][2]*g[1] + R[2][2]*g[2],
    ]


# ---------------------------------------------------------------------------
# Physics engine for sensor worker
# ---------------------------------------------------------------------------

class _Physics:
    """Minimal kinematic physics for the GPS-fusion sensor worker.

    Maintains position, velocity, and kinematic acceleration in NED.
    The sensor worker calls step() each tick; the main thread calls
    set_target_vel() to schedule smooth velocity transitions.

    accel_body returned is the IMU specific force:
        specific_force_NED = accel_world - gravity_NED
                           = accel_world - [0, 0, +g]
        accel_body         = R^T @ specific_force_NED

    This ensures the EKF's IMU integration matches the GPS velocity/position
    data — preventing large innovations that would block GPS fusion.
    """

    def __init__(
        self,
        pos_ned: list,
        vel_ned: list,
        attitude_deg: list,
        accel_limit: float = 0.3,   # m/s^2 per-axis ramp limit
    ) -> None:
        self._lock       = threading.Lock()
        self.pos         = list(pos_ned)
        self.vel         = list(vel_ned)
        self.target_vel  = list(vel_ned)
        self.accel_world = [0.0, 0.0, 0.0]   # kinematic accel in NED (m/s^2)
        self.accel_limit = accel_limit
        rpy = [math.radians(a) for a in attitude_deg]
        self._rpy_rad = rpy
        self._R       = _euler_to_R(*rpy)

    def set_target_vel(self, target: list) -> None:
        """Schedule a velocity transition (main-thread safe)."""
        with self._lock:
            self.target_vel = list(target)

    def step(self, dt: float) -> None:
        """Advance physics by dt seconds (sensor-thread only)."""
        if dt <= 0:
            return
        with self._lock:
            dv_max = self.accel_limit * dt
            for i in range(3):
                err   = self.target_vel[i] - self.vel[i]
                delta = max(-dv_max, min(dv_max, err))
                self.accel_world[i] = delta / dt
                self.vel[i] += delta
            for i in range(3):
                self.pos[i] += self.vel[i] * dt

    def snapshot(self):
        """Return (pos, vel, accel_world, accel_body, rpy_rad) atomically.

        accel_world : kinematic acceleration in NED [m/s^2]  (what we log as accel_x/y/z)
        accel_body  : IMU specific force in body frame        (what we send to SITL)
        """
        with self._lock:
            aw = list(self.accel_world)
            sf = [aw[0], aw[1], aw[2] - _GRAVITY]
            R  = self._R
            ab = [
                R[0][0]*sf[0] + R[1][0]*sf[1] + R[2][0]*sf[2],
                R[0][1]*sf[0] + R[1][1]*sf[1] + R[2][1]*sf[2],
                R[0][2]*sf[0] + R[1][2]*sf[1] + R[2][2]*sf[2],
            ]
            return list(self.pos), list(self.vel), aw, ab, list(self._rpy_rad)


# ---------------------------------------------------------------------------
# Minimum-jerk trajectory
# ---------------------------------------------------------------------------

class _MinJerkTraj:
    """5th-order polynomial (minimum-jerk) trajectory for each NED axis.

    Boundary conditions:
        pos(0) = p0,  vel(0) = v0,  accel(0) = 0
        pos(T) = pf,  vel(T) = vf,  accel(T) = 0

    Coefficients in normalised time tau = t/T:
        p(tau) = c0 + c1*tau + c2*tau^2 + c3*tau^3 + c4*tau^4 + c5*tau^5

    Derivation for zero initial/final acceleration (c2 = 0):
        D = pf - p0 - v0*T
        E = (vf - v0)*T
        c0=p0, c1=v0*T, c2=0, c3=10D-4E, c4=-15D+7E, c5=6D-3E
    """

    def __init__(
        self,
        p0: list,
        v0: list,
        pf: list,
        vf: list,
        T: float,
    ) -> None:
        self.T  = float(T)
        self._n = len(p0)
        self._c = []        # list of [c0..c5] per axis
        for i in range(self._n):
            D  = pf[i] - p0[i] - v0[i] * T
            E  = (vf[i] - v0[i]) * T
            self._c.append([
                p0[i],          # c0
                v0[i] * T,      # c1
                0.0,            # c2  (zero initial accel constraint)
                10*D - 4*E,     # c3
                -15*D + 7*E,    # c4
                6*D - 3*E,      # c5
            ])

    def at(self, t: float):
        """Return (pos, vel, accel_world) lists at time t (clamped to [0, T])."""
        tau = max(0.0, min(1.0, t / self.T))
        T   = self.T
        pos, vel, acc = [], [], []
        for c in self._c:
            p = (c[0] + c[1]*tau + c[2]*tau**2
                 + c[3]*tau**3 + c[4]*tau**4 + c[5]*tau**5)
            v = (c[1] + 2*c[2]*tau + 3*c[3]*tau**2
                 + 4*c[4]*tau**3 + 5*c[5]*tau**4) / T
            a = (2*c[2] + 6*c[3]*tau + 12*c[4]*tau**2
                 + 20*c[5]*tau**3) / T**2
            pos.append(p); vel.append(v); acc.append(a)
        return pos, vel, acc

    @property
    def peak_speed(self) -> float:
        """Approximate peak speed (m/s) — at tau=0.5 for zero-boundary case."""
        _, v05, _ = self.at(0.5 * self.T)
        return math.sqrt(sum(vi**2 for vi in v05))


class _TrajPhysics:
    """Kinematic physics that follows a _MinJerkTraj analytically.

    Unlike _Physics (which ramps toward a target velocity numerically),
    _TrajPhysics evaluates the polynomial closed-form at each tick so
    accel_body is exact — no numerical differentiation noise.

    omega_spin ramps linearly from 0 to omega_target over the trajectory
    duration and is returned from snapshot() for send_state(rpm_rad_s=...).
    """

    def __init__(
        self,
        traj: _MinJerkTraj,
        attitude_deg: list,
        omega_target: float = 0.0,
    ) -> None:
        self._lock         = threading.Lock()
        self._traj         = traj
        self._t            = 0.0
        self._started      = False
        self._omega_target = omega_target
        rpy                = [math.radians(a) for a in attitude_deg]
        self._rpy_rad      = rpy
        self._R            = _euler_to_R(*rpy)

    def start(self) -> None:
        """Begin the trajectory clock.  Call this after arm."""
        with self._lock:
            self._started = True

    def step(self, dt: float) -> None:
        """Advance trajectory clock (sensor-thread only)."""
        with self._lock:
            if self._started:
                self._t = min(self._t + dt, self._traj.T)

    def snapshot(self):
        """Return (pos, vel, accel_world, accel_body, rpy_rad, omega_rad_s)."""
        with self._lock:
            t   = self._t
            T   = self._traj.T
            pos, vel, aw = self._traj.at(t)
            # IMU specific force = accel_world - gravity_NED
            sf = [aw[0], aw[1], aw[2] - _GRAVITY]
            R  = self._R
            ab = [
                R[0][0]*sf[0] + R[1][0]*sf[1] + R[2][0]*sf[2],
                R[0][1]*sf[0] + R[1][1]*sf[1] + R[2][1]*sf[2],
                R[0][2]*sf[0] + R[1][2]*sf[1] + R[2][2]*sf[2],
            ]
            alpha = t / T if T > 0 else 1.0
            omega = self._omega_target * alpha
            return list(pos), list(vel), list(aw), ab, list(self._rpy_rad), omega

    @property
    def elapsed(self) -> float:
        with self._lock:
            return self._t

    @property
    def done(self) -> bool:
        with self._lock:
            return self._t >= self._traj.T


def _traj_sensor_worker(
    stop_event: threading.Event,
    traj_phys: _TrajPhysics,
    hz: int = 100,
    telemetry: list | None = None,
    tel_hz: int = 10,
) -> None:
    """Sensor worker driven by _TrajPhysics (minimum-jerk trajectory).

    Identical protocol to _physics_sensor_worker but also passes rpm_rad_s
    to send_state() so ArduPilot's RPM sensor sees the rotor spinning up.
    """
    gyro   = [0.0, 0.0, 0.0]
    sitl   = SITLInterface(recv_port=StackConfig.SITL_JSON_PORT, lockstep=False)
    sitl.bind()

    t0        = time.monotonic()
    t_next    = t0
    t_last    = t0
    t_tel     = t0
    connected = False
    tel_dt    = 1.0 / tel_hz

    try:
        while not stop_event.is_set():
            servos = sitl.recv_servos()
            if servos is not None and not connected:
                log.info("traj_worker: SITL connected")
                connected = True

            now = time.monotonic()
            if connected and now >= t_next:
                dt = now - t_last
                traj_phys.step(dt)
                t_last = now

                pos, vel, accel_w, accel_b, rpy, omega = traj_phys.snapshot()

                sitl.send_state(
                    timestamp  = now - t0,
                    pos_ned    = pos,
                    vel_ned    = vel,
                    rpy_rad    = rpy,
                    accel_body = accel_b,
                    gyro_body  = gyro,
                    rpm_rad_s  = omega,
                )
                t_next = now + 1.0 / hz

                if telemetry is not None and now >= t_tel:
                    t_tel = now + tel_dt
                    telemetry.append(TelRow(
                        t_sim        = now - t0,
                        pos_x        = pos[0],
                        pos_y        = pos[1],
                        pos_z        = pos[2],
                        vel_x        = vel[0],
                        vel_y        = vel[1],
                        vel_z        = vel[2],
                        accel_x      = accel_w[0],
                        accel_y      = accel_w[1],
                        accel_z      = accel_w[2],
                        sens_vel_n   = vel[0],
                        sens_vel_e   = vel[1],
                        sens_vel_d   = vel[2],
                        sens_accel_x = accel_b[0],
                        sens_accel_y = accel_b[1],
                        sens_accel_z = accel_b[2],
                        rpy_roll     = math.degrees(rpy[0]),
                        rpy_pitch    = math.degrees(rpy[1]),
                        rpy_yaw      = math.degrees(rpy[2]),
                        v_horiz_ms   = math.hypot(vel[0], vel[1]),
                        omega_rotor  = omega,
                    ))
    finally:
        sitl.close()


def _physics_sensor_worker(
    stop_event: threading.Event,
    phys: _Physics,
    hz: int = 100,
    telemetry: list | None = None,
    tel_hz: int = 10,
) -> None:
    """Send sensor packets to SITL driven by _Physics state.

    Each tick:
      1. phys.step(dt) — ramps vel toward target_vel, integrates pos
      2. phys.snapshot() — returns pos, vel, accel_world, accel_body, rpy_rad
      3. sitl.send_state() — sends JSON packet using the same format as mediator

    The accel_body is computed from the actual velocity change each tick, so the
    EKF's IMU integration predicts the same position/velocity as GPS — small
    innovations allow GPS fusion to succeed.

    If telemetry is not None, appends a TelRow every 1/tel_hz seconds.
    """
    gyro = [0.0, 0.0, 0.0]

    sitl = SITLInterface(recv_port=StackConfig.SITL_JSON_PORT, lockstep=False)
    sitl.bind()

    t0        = time.monotonic()
    t_next    = t0
    t_last    = t0
    t_tel     = t0
    connected = False
    tel_dt    = 1.0 / tel_hz

    try:
        while not stop_event.is_set():
            servos = sitl.recv_servos()   # non-blocking; discovers SITL addr
            if servos is not None and not connected:
                log.info("sensor_worker: SITL connected")
                connected = True

            now = time.monotonic()
            if connected and now >= t_next:
                dt = now - t_last
                phys.step(dt)
                t_last = now

                pos, vel, accel_w, accel_b, rpy = phys.snapshot()

                sitl.send_state(
                    timestamp  = now - t0,
                    pos_ned    = pos,
                    vel_ned    = vel,
                    rpy_rad    = rpy,
                    accel_body = accel_b,
                    gyro_body  = gyro,
                )
                t_next = now + 1.0 / hz

                # Telemetry at tel_hz
                if telemetry is not None and now >= t_tel:
                    t_tel = now + tel_dt
                    row = TelRow(
                        t_sim        = now - t0,
                        pos_x        = pos[0],
                        pos_y        = pos[1],
                        pos_z        = pos[2],
                        vel_x        = vel[0],
                        vel_y        = vel[1],
                        vel_z        = vel[2],
                        accel_x      = accel_w[0],
                        accel_y      = accel_w[1],
                        accel_z      = accel_w[2],
                        sens_vel_n   = vel[0],
                        sens_vel_e   = vel[1],
                        sens_vel_d   = vel[2],
                        sens_accel_x = accel_b[0],
                        sens_accel_y = accel_b[1],
                        sens_accel_z = accel_b[2],
                        rpy_roll     = math.degrees(rpy[0]),
                        rpy_pitch    = math.degrees(rpy[1]),
                        rpy_yaw      = math.degrees(rpy[2]),
                        v_horiz_ms   = math.hypot(vel[0], vel[1]),
                    )
                    telemetry.append(row)
    finally:
        sitl.close()


# ---------------------------------------------------------------------------
# Layer definitions
# ---------------------------------------------------------------------------
# RAWES equilibrium Euler angles (from build_orb_frame(body_z=[0.277,0.877,-0.392])):
#   roll ~124.74 deg, pitch ~-46.12 deg, yaw ~136.6 deg
#
# Current vel0 = [0.916, -0.257, 0.093]  -> heading = atan2(-0.257, 0.916) = -15.7 deg
# heading_gap = 136.6 - (-15.7) = 152.3 deg  <- EKF blocks GPS here

_RAWES_ROLL  =  124.74
_RAWES_PITCH =  -46.12
_RAWES_YAW   =  136.6
_RAWES_ALT   =  14.12    # m above SITL home
_V           =  0.96     # orbital speed m/s


def _vel_heading(heading_deg: float, speed: float = _V):
    h = math.radians(heading_deg)
    return [speed*math.cos(h), speed*math.sin(h), 0.0]


# (id, description, attitude_deg=[roll,pitch,yaw], vel_ned, position_ned, expected_gap_deg)
_LAYERS = [
    (
        "L0_baseline",
        "Level hover, yaw=0, vel North  (gap=0 deg)",
        [0.0, 0.0, 0.0],
        [0.1, 0.0, 0.0],
        [0.0, 0.0, -50.0],
        0.0,
    ),
    (
        "L1_rawes_tilt_yaw0",
        "RAWES tilt, yaw=0, vel North  (gap=0 deg)",
        [_RAWES_ROLL, _RAWES_PITCH, 0.0],
        [0.1, 0.0, 0.0],
        [0.0, 0.0, -_RAWES_ALT],
        0.0,
    ),
    (
        "L2_rawes_full_vel_aligned",
        "RAWES full attitude, vel aligned with compass  (gap=0 deg)",
        [_RAWES_ROLL, _RAWES_PITCH, _RAWES_YAW],
        _vel_heading(_RAWES_YAW),
        [27.9, 95.0, -_RAWES_ALT],
        0.0,
    ),
    (
        "L3_rawes_vel_current",
        "RAWES full attitude, vel=current cfg  (gap=152 deg)",
        [_RAWES_ROLL, _RAWES_PITCH, _RAWES_YAW],
        [0.916, -0.257, 0.093],
        [27.9, 95.0, -_RAWES_ALT],
        152.3,
    ),
    (
        "L4_rawes_vel_east",
        "RAWES full attitude, vel East  (gap=46.6 deg)",
        [_RAWES_ROLL, _RAWES_PITCH, _RAWES_YAW],
        _vel_heading(90.0),
        [27.9, 95.0, -_RAWES_ALT],
        46.6,
    ),
    (
        "L5_rawes_vel_north",
        "RAWES full attitude, vel North  (gap=136.6 deg)",
        [_RAWES_ROLL, _RAWES_PITCH, _RAWES_YAW],
        _vel_heading(0.0),
        [27.9, 95.0, -_RAWES_ALT],
        136.6,
    ),
]


# ---------------------------------------------------------------------------
# Static sensor worker
# ---------------------------------------------------------------------------

def _sensor_worker(
    stop_event: threading.Event,
    attitude_deg: list,
    vel_ned: list,
    position_ned: list,
    hz: int = 100,
) -> None:
    """Send sensor packets to SITL (no dynamics, no mediator).

    Attitude, velocity, and accel are constant.  Position is integrated from
    vel_ned each tick so GPS position is always consistent with velocity —
    prevents a growing EKF position innovation that would block GPS fusion.
    """
    accel   = _accel_body_from_rpy_deg(*attitude_deg)
    rpy_rad = [math.radians(a) for a in attitude_deg]
    gyro    = [0.0, 0.0, 0.0]
    pos     = list(position_ned)   # mutable; integrated each tick

    sitl = SITLInterface(recv_port=StackConfig.SITL_JSON_PORT, lockstep=False)
    sitl.bind()

    t0        = time.monotonic()
    t_next    = t0
    t_last    = t0
    connected = False

    try:
        while not stop_event.is_set():
            servos = sitl.recv_servos()   # non-blocking; discovers SITL addr
            if servos is not None and not connected:
                log.info("sensor_worker: SITL connected")
                connected = True

            now = time.monotonic()
            if connected and now >= t_next:
                dt = now - t_last
                pos[0] += vel_ned[0] * dt
                pos[1] += vel_ned[1] * dt
                pos[2] += vel_ned[2] * dt
                t_last  = now

                sitl.send_state(
                    timestamp  = now - t0,
                    pos_ned    = pos,
                    vel_ned    = vel_ned,
                    rpy_rad    = rpy_rad,
                    accel_body = accel,
                    gyro_body  = gyro,
                )
                t_next = now + 1.0 / hz
    finally:
        sitl.close()


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "layer_id,description,attitude_deg,vel_ned,position_ned,expected_gap",
    _LAYERS,
    ids=[l[0] for l in _LAYERS],
)
def test_gps_fusion_layers(
    tmp_path, request,
    layer_id, description, attitude_deg, vel_ned, position_ned, expected_gap,
):
    """
    Launch SITL with standard boot params, send static sensor packets for 90 s,
    report whether GPS fused.

    Layers with expected_gap < 30 deg assert GPS must fuse.
    Layers with expected_gap >= 30 deg are marked xfail.
    """
    compass_deg  = attitude_deg[2]
    vel_head_deg = math.degrees(math.atan2(vel_ned[1], vel_ned[0]))
    actual_gap   = abs(((compass_deg - vel_head_deg + 180) % 360) - 180)

    with _sitl_stack(
        tmp_path,
        log_name   = f"gps_layers_{layer_id}",
        log_prefix = "gps_layers",
        test_name  = request.node.name,
    ) as ctx:
        _log = ctx.log
        _log.info("")
        _log.info("=" * 60)
        _log.info("  Layer       : %s", layer_id)
        _log.info("  Description : %s", description)
        _log.info("  attitude    : roll=%.1f  pitch=%.1f  yaw=%.1f", *attitude_deg)
        _log.info("  accel_body  : %s", _accel_body_from_rpy_deg(*attitude_deg))
        _log.info("  vel_ned     : %s  heading=%.1f deg", vel_ned, vel_head_deg)
        _log.info("  heading_gap : %.1f deg  (compass=%.1f, vel=%.1f)",
                  actual_gap, compass_deg, vel_head_deg)
        _log.info("=" * 60)

        # Start static sensor worker
        stop_ev = threading.Event()
        worker  = threading.Thread(
            target=_sensor_worker,
            args=(stop_ev, attitude_deg, vel_ned, position_ned),
            daemon=True, name=f"sensor_{layer_id}",
        )
        worker.start()

        gps_origin = False
        gps_fused  = False
        gps_glitch = False

        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            _log.info("Connected. Waiting 90 s for GPS fusion ...")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 90.0:
                msg = gcs._recv(  # type: ignore[union-attr]
                    type=["STATUSTEXT"],
                    blocking=True, timeout=1.0,
                )
                if msg is None:
                    continue
                text = msg.text.strip()
                _log.info("  STATUSTEXT: %s", text)
                tl = text.lower()
                if "origin set" in tl:
                    gps_origin = True
                if "is using gps" in tl:
                    gps_fused = True
                if "gps glitch" in tl or "compass error" in tl:
                    gps_glitch = True

        finally:
            stop_ev.set()
            worker.join(timeout=2.0)
            try:
                gcs.close()
            except Exception:
                pass

        result = ("FUSED"  if gps_fused and not gps_glitch else
                  "GLITCH" if gps_glitch else
                  "NO_GPS")
        _log.info("")
        _log.info("  GPS origin : %s", gps_origin)
        _log.info("  GPS fused  : %s", gps_fused)
        _log.info("  GPS glitch : %s", gps_glitch)
        _log.info("  RESULT     : %s", result)

        _expect_pass = (expected_gap < 30.0)
        if _expect_pass:
            assert gps_origin, f"[{layer_id}] GPS origin never set"
            assert gps_fused,  f"[{layer_id}] GPS never fused (gap={actual_gap:.1f} deg)"
            assert not gps_glitch, f"[{layer_id}] GPS fused then glitched"
        else:
            if not gps_fused or gps_glitch:
                pytest.xfail(
                    f"[{layer_id}] GPS did not fuse as expected (gap={actual_gap:.1f} deg)"
                )


# ---------------------------------------------------------------------------
# "In-between" test: same L0 sensor, but arm vehicle before watching GPS
# ---------------------------------------------------------------------------

def _wait_statustext(gcs, keywords, timeout, log, label):
    """Poll MAVLink messages until any keyword matches a STATUSTEXT or timeout.

    Logs every second:
      EKF_STATUS_REPORT  — flags + variances
      GPS_RAW_INT        — fix type, satellites, HDOP, speed
      ATTITUDE           — roll/pitch/yaw from EKF
    Local position logged every 5 s.
    """
    t0         = time.monotonic()
    t_last_ekf = t0 - 1.0   # force immediate first log
    t_last_gps = t0 - 1.0
    t_last_att = t0 - 1.0
    t_last_pos = t0
    last_flags = None

    while time.monotonic() - t0 < timeout:
        msg = gcs._recv(  # type: ignore[union-attr]
            type=["STATUSTEXT", "EKF_STATUS_REPORT", "LOCAL_POSITION_NED",
                  "GPS_RAW_INT", "ATTITUDE"],
            blocking=True, timeout=0.2,
        )
        now     = time.monotonic()
        elapsed = now - t0

        if msg is None:
            continue

        mt = msg.get_type()

        if mt == "STATUSTEXT":
            text = msg.text.strip()
            log.info("  [%s t=%5.1f s] STATUSTEXT: %s", label, elapsed, text)
            tl = text.lower()
            for kw in keywords:
                if kw in tl:
                    return text

        elif mt == "EKF_STATUS_REPORT":
            flags = msg.flags
            if now - t_last_ekf >= 1.0 or flags != last_flags:
                log.info(
                    "  [%s t=%5.1f s] EKF flags=0x%04x vel=%.2f pos_h=%.2f pos_v=%.2f",
                    label, elapsed, flags,
                    msg.velocity_variance, msg.pos_horiz_variance, msg.pos_vert_variance,
                )
                t_last_ekf = now
                last_flags = flags

        elif mt == "GPS_RAW_INT" and now - t_last_gps >= 1.0:
            # fix_type: 0=no fix,1=no fix,2=2D,3=3D,4=DGPS,5=RTK float,6=RTK fixed
            log.info(
                "  [%s t=%5.1f s] GPS fix=%d sats=%d hdop=%.1f spd=%.2f m/s",
                label, elapsed,
                msg.fix_type, msg.satellites_visible,
                msg.eph / 100.0,          # eph is HDOP*100
                msg.vel / 100.0,          # vel is cm/s
            )
            t_last_gps = now

        elif mt == "ATTITUDE" and now - t_last_att >= 1.0:
            log.info(
                "  [%s t=%5.1f s] ATT roll=%.1f pitch=%.1f yaw=%.1f deg",
                label, elapsed,
                math.degrees(msg.roll),
                math.degrees(msg.pitch),
                math.degrees(msg.yaw),
            )
            t_last_att = now

        elif mt == "LOCAL_POSITION_NED" and now - t_last_pos >= 5.0:
            log.info(
                "  [%s t=%5.1f s] LOCAL_POS z=%.2f vz=%.3f (NED, -=up)",
                label, elapsed, msg.z, msg.vz,
            )
            t_last_pos = now

    pytest.fail(f"[{label}] timed out after {timeout:.0f} s waiting for: {keywords}")



def _rc_keepalive(
    gcs,
    channels: dict,
    stop_event: threading.Event,
    interval: float = 0.4,
) -> None:
    """Send RC_CHANNELS_OVERRIDE every `interval` seconds until stop_event is set.

    ArduPilot's RC-override expiry is ~1 sim-second, so 0.4 s keeps a safe margin.
    Runs as a daemon thread; stops automatically when the test exits.
    """
    while not stop_event.is_set():
        try:
            gcs.send_rc_override(channels)
        except Exception:
            pass
        stop_event.wait(interval)


def _ramp_vel(vel_ned: list, target: list, duration: float, _log) -> None:
    """Linearly ramp vel_ned in-place toward target over duration seconds.

    Updates vel_ned every 0.1 s so the sensor worker sees a smooth acceleration
    rather than a step change that would spike GPS velocity innovations.
    """
    steps = max(1, int(duration / 0.1))
    v0 = list(vel_ned)
    for i in range(1, steps + 1):
        alpha = i / steps
        for j in range(3):
            vel_ned[j] = v0[j] + alpha * (target[j] - v0[j])
        time.sleep(0.1)
    for j in range(3):
        vel_ned[j] = target[j]
    _log.info("  vel_ned ramped to [%.2f, %.2f, %.2f]", *vel_ned)


def test_gps_fusion_armed(tmp_path, request):
    """
    GPS fusion pre-arm + post-arm checklist (EK3_SRC1_YAW=8 GSF, no compass).

    Pre-arm the vehicle is stationary — moving velocity causes GPS position to drift
    away from EKF's AID_NONE constant-position estimate → horizErrSq > 1 → healthy()=False
    → all EKF flags stay 0x0000.  Stationary sensor keeps innovations near zero.

    From EKF3 source (AP_NavEKF3_VehicleStatus.cpp, AP_NavEKF3_Control.cpp):
      [1] Pre-arm: EKF tilt alignment  (~5 s)
      [2] Pre-arm: GPS origin set      (~10 s after GPS detected, EK3_GPS_CHECK=0 bypasses checks)
      [3] Arm vehicle (force=True)
      [4] Post-arm: ramp vel to 1.2 m/s North + 0.5 m/s climb over 4 s
             inFlight fires when pos.z - posDownAtTakeoff < -1.5 m  (~3 s at 0.5 m/s)
             EKFGSF_run_filterbank=True → GSF accumulates → yawAlignComplete
             "EKF3 IMU0 yaw aligned using GPS" STATUSTEXT
      [5] GPS fusion: readyToUseGPS() → AID_ABSOLUTE
             "EKF3 IMU0 is using GPS" STATUSTEXT

    Sensor: stationary pre-arm (vel=[0,0,0]), position=[0,0,-50] NED.
    After arm, velocity is ramped smoothly to [1.2, 0, -0.5] to avoid innovation spikes.
    """
    attitude_deg = [0.0, 0.0, 0.0]
    position_ned = [0.0, 0.0, 0.0]    # start at home altitude (ground level)
    # Physics engine: stationary until arm, then ramp to 1.5 m/s North + 1.5 m/s
    # climb (NED Z negative).  At 0.3 m/s² per axis the ramp completes in ~5 s.
    # After ~35 s of cruise the vehicle is ~50 m North and ~50 m above home.
    phys = _Physics(
        pos_ned      = position_ned,
        vel_ned      = [0.0, 0.0, 0.0],
        attitude_deg = attitude_deg,
        accel_limit  = 0.3,   # m/s^2 -> 1.5 m/s in ~5 s
    )

    with _sitl_stack(
        tmp_path,
        log_name   = "gps_fusion_armed",
        log_prefix = "gps_fusion",
        test_name  = request.node.name,
    ) as ctx:
        _log = ctx.log
        _log.info("Pre-arm GPS checks: EK3_SRC1_YAW=1 (compass), EK3_GPS_CHECK=0, COMPASS_USE=1")
        _log.info("Sensor: physics engine — stationary pre-arm, ramped post-arm, pos=[0,0,-50]")

        tel_rows: list[TelRow] = []

        stop_ev = threading.Event()
        worker  = threading.Thread(
            target=_physics_sensor_worker,
            args=(stop_ev, phys),
            kwargs={"telemetry": tel_rows},
            daemon=True, name="sensor_physics",
        )
        worker.start()

        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)   # ALL streams at 10 Hz

            # ------------------------------------------------------------------
            # [1] Pre-arm: EKF tilt alignment
            # ------------------------------------------------------------------
            _log.info("[1] Waiting for EKF tilt alignment (pre-arm) ...")
            if not gcs.wait_ekf_attitude(timeout=20.0):
                pytest.fail("[1] EKF tilt alignment timed out after 20 s")
            _log.info("[1] PASS — EKF tilt aligned")

            # ------------------------------------------------------------------
            # [2] Pre-arm: GPS origin set
            #     calcGpsGoodToAlign() needs 10 s of no GPS failures.
            #     With EK3_GPS_CHECK=0 all checks disabled → fires ~10 s after
            #     first GPS data.  Stationary sensor keeps EKF healthy (no drift).
            # ------------------------------------------------------------------
            _log.info("[2] Waiting for GPS origin (pre-arm, stationary, expect ~20 s) ...")
            _wait_statustext(
                gcs, ["origin set"],
                timeout=35.0, log=_log, label="2-origin",
            )
            _log.info("[2] PASS — GPS origin set pre-arm")

            time.sleep(0.5)
            t_check = time.monotonic()
            while time.monotonic() - t_check < 3.0:
                msg = gcs._recv(type=["EKF_STATUS_REPORT"], blocking=True, timeout=1.0)
                if msg is not None:
                    _log.info(
                        "Post-origin EKF flags=0x%04x vel=%.3f pos_h=%.3f pos_v=%.3f",
                        msg.flags, msg.velocity_variance,
                        msg.pos_horiz_variance, msg.pos_vert_variance,
                    )
                    break

            _log.info("Pre-arm checks complete. Ready to arm.")

            # ------------------------------------------------------------------
            # [3] Switch to ACRO, arm, engage motor interlock
            #   Mirrors _run_acro_setup: arm with CH8=1000 (interlock LOW) so
            #   H_RSC_MODE=1 passthrough doesn't block arm, then raise to 2000.
            # ------------------------------------------------------------------
            _log.info("[3] Setting ACRO mode (mode 1) ...")
            gcs.set_mode(1, timeout=10.0)
            gcs.send_rc_override({8: 1000})   # interlock LOW before arm

            _log.info("[3] Arming vehicle ...")
            gcs.arm(force=True, timeout=15.0, rc_override={8: 1000})
            _log.info("[3] PASS — armed")

            # Raise interlock HIGH and start keepalive (mirrors stack_infra behaviour)
            gcs.send_rc_override({3: 1700, 8: 2000})
            rc_stop = threading.Event()
            rc_thread = threading.Thread(
                target=_rc_keepalive,
                args=(gcs, {3: 1700, 8: 2000}, rc_stop),
                daemon=True, name="rc_keepalive",
            )
            rc_thread.start()
            _log.info("[3] RC keepalive started: CH3=1700, CH8=2000 (interlock HIGH)")

            # Ramp velocity post-arm: 1.2 m/s North + 0.5 m/s climb (NED Z negative).
            # - Horizontal speed > 0.5 m/s needed for compass yaw estimate
            # - Climb rate 0.5 m/s -> 1.5 m altitude gain in ~3 s -> inFlight=True
            # - Physics engine ramps at 0.3 m/s^2 (reach 1.2 m/s in ~4 s)
            # - accel_body is computed from actual dv/dt each tick -> EKF IMU integration
            #   predicts the same velocity as GPS -> small innovations -> fusion succeeds
            # H_RSC_MODE=1 (Passthrough): servo8 = RC8 immediately -> THROTTLE_UNLIMITED
            # on first RC override -> land_complete clears -> onGround=False -> posDownAtTakeoff
            # freezes at arm-time position.  No spool-up wait needed.
            # Fly 50 m North + 50 m up.  Physics ramps at 0.3 m/s^2; 1.5 m/s
            # reached in ~5 s.  After ~35 s cruise: pos ~ [50, 0, -50] NED.
            _log.info("[3] Climbing to 50 m North + 50 m altitude (1.5 m/s each axis) ...")
            phys.set_target_vel([1.5, 0.0, -1.5])
            time.sleep(5.0)   # wait for ramp to complete (0.3 m/s^2 -> ~5 s)
            pos_snap, vel_snap, _, _, _ = phys.snapshot()
            _log.info("vel ramped: vel=[%.2f, %.2f, %.2f] pos=[%.1f, %.1f, %.1f]",
                      *vel_snap, *pos_snap)

            # ------------------------------------------------------------------
            # [4] Wait for yaw alignment
            #     With EK3_SRC1_YAW=1 (compass), yaw alignment can complete
            #     pre-arm — in that case the EKF attitude bit (0x0001) is
            #     already set when we reach this step.  Check flags first;
            #     fall back to STATUSTEXT poll only if not yet aligned.
            # ------------------------------------------------------------------
            _log.info("[4] Waiting for yaw alignment (post-arm, expect ~30 s) ...")
            yaw_aligned = False
            t_check = time.monotonic()
            while time.monotonic() - t_check < 3.0 and not yaw_aligned:
                msg = gcs._recv(type=["EKF_STATUS_REPORT"], blocking=True, timeout=0.5)
                if msg is not None:
                    _log.info(
                        "[4-yaw t=  0.0 s] EKF flags=0x%04x vel=%.2f pos_h=%.2f pos_v=%.2f",
                        msg.flags, msg.velocity_variance,
                        msg.pos_horiz_variance, msg.pos_vert_variance,
                    )
                    if msg.flags & 0x0001:   # EKF_ATTITUDE — attitude + yaw aligned
                        yaw_aligned = True
                        _log.info("[4] EKF attitude bit set — yaw aligned (pre-arm or post-arm)")
            if not yaw_aligned:
                _wait_statustext(
                    gcs, ["yaw align"],
                    timeout=60.0, log=_log, label="4-yaw",
                )
            _log.info("[4] PASS — yaw aligned")

            # Log EKF flags at yaw-align — expect velocity + attitude bits set
            time.sleep(0.5)
            t_check = time.monotonic()
            while time.monotonic() - t_check < 3.0:
                msg = gcs._recv(type=["EKF_STATUS_REPORT"], blocking=True, timeout=1.0)
                if msg is not None:
                    _log.info(
                        "Post-yaw EKF flags=0x%04x vel=%.3f pos_h=%.3f pos_v=%.3f",
                        msg.flags, msg.velocity_variance,
                        msg.pos_horiz_variance, msg.pos_vert_variance,
                    )
                    break

            # ------------------------------------------------------------------
            # [5] Wait for GPS fusion
            #     yawAlignComplete → readyToUseGPS() → PV_AidingMode=AID_ABSOLUTE
            #     "EKF3 IMU0 is using GPS" STATUSTEXT
            # ------------------------------------------------------------------
            _log.info("[5] Waiting for GPS fusion (vehicle flying N+up, expect ~30 s) ...")
            _wait_statustext(
                gcs, ["is using gps"],
                timeout=60.0, log=_log, label="5-fused",
            )
            _log.info("[5] PASS — GPS fused")

        finally:
            try:
                rc_stop.set()
            except NameError:
                pass
            stop_ev.set()
            worker.join(timeout=2.0)
            try:
                gcs.close()
            except Exception:
                pass
            if tel_rows:
                tel_path = ctx.test_log_dir / "telemetry.csv"
                write_csv(tel_rows, tel_path)
                _log.info("Telemetry written: %d rows -> %s", len(tel_rows), tel_path)
            # Post-run: call  python simulation/analysis/merge_logs.py gps_fusion_test_gps_fusion_armed


# ---------------------------------------------------------------------------
# Trajectory test: smooth flight from ground to steady-state position
# ---------------------------------------------------------------------------

def test_gps_fusion_trajectory(tmp_path, request):
    """
    Fly a minimum-jerk trajectory from ground to the RAWES steady-state
    position (steady_state_starting.json) and verify GPS fuses en-route.

    Trajectory design
    -----------------
    Start : pos=[0,0,0] NED, vel=[0,0,0]
    End   : pos=steady_state pos, vel=[0,0,0]

    Yaw is fixed to the compass bearing from origin to the target:
        yaw = atan2(pos_E, pos_N)  [degrees]
    This guarantees heading_gap = 0 throughout -- GPS velocity heading and
    compass heading match -- so GPS fusion is never blocked by yaw mismatch.

    The 5th-order minimum-jerk polynomial has zero accel at both ends, so
    the IMU sees no step changes.  accel_body is computed analytically from
    the polynomial (not numerically), so EKF innovations are minimal.

    omega_spin ramps linearly from 0 to the steady-state value over T seconds
    and is reported via RPM1_TYPE=10 (SITL RPM sensor).

    GPS fusion is expected to complete well before the trajectory ends.
    The test asserts GPS fuses within the trajectory duration + a 30 s buffer.
    """
    ss = json.loads(_STEADY_STATE_JSON.read_text())
    pos_target   = ss["pos"]          # NED [m]
    vel_target   = [0.0, 0.0, 0.0]   # arrive at rest (steady-state vel ~ 0)
    omega_target = ss["omega_spin"]   # rad/s

    # Compass yaw = bearing from origin to target (North = 0, East = +90 deg)
    yaw_deg = math.degrees(math.atan2(pos_target[1], pos_target[0]))
    # Horizontal distance and altitude gain for logging
    h_dist  = math.hypot(pos_target[0], pos_target[1])
    alt_m   = -pos_target[2]    # NED Z negative = above ground

    # Trajectory duration = STARTUP_DAMP_S (65 s).
    # GPS fusion happens at ~63 s post-arm, so the vehicle arrives at the
    # target ~2 s after GPS fuses — the EKF is already tracking for the
    # final approach, giving the smallest possible position error at arrival.
    T_traj  = 65.0   # seconds

    traj = _MinJerkTraj(
        p0 = [0.0, 0.0, 0.0],
        v0 = [0.0, 0.0, 0.0],
        pf = pos_target,
        vf = vel_target,
        T  = T_traj,
    )

    with _sitl_stack(
        tmp_path,
        log_name   = "gps_fusion_trajectory",
        log_prefix = "gps_fusion",
        test_name  = request.node.name,
    ) as ctx:
        _log = ctx.log
        _log.info("Steady-state target (NED): pos=[%.2f, %.2f, %.2f] m",
                  *pos_target)
        _log.info("  h_dist=%.1f m  alt=%.1f m  yaw=%.1f deg  T=%.0f s",
                  h_dist, alt_m, yaw_deg, T_traj)
        _log.info("  omega_target=%.3f rad/s  peak_speed~=%.2f m/s",
                  omega_target, traj.peak_speed)

        traj_phys = _TrajPhysics(
            traj         = traj,
            attitude_deg = [0.0, 0.0, yaw_deg],   # level, heading toward target
            omega_target = omega_target,
        )
        tel_rows: list[TelRow] = []

        stop_ev = threading.Event()
        worker  = threading.Thread(
            target = _traj_sensor_worker,
            args   = (stop_ev, traj_phys),
            kwargs = {"telemetry": tel_rows},
            daemon = True,
            name   = "traj_worker",
        )
        worker.start()

        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            # [1] EKF tilt alignment
            _log.info("[1] Waiting for EKF tilt alignment ...")
            if not gcs.wait_ekf_attitude(timeout=20.0):
                pytest.fail("[1] EKF tilt alignment timed out")
            _log.info("[1] PASS — EKF tilt aligned")

            # [2] GPS origin (pre-arm, vehicle stationary at [0,0,0])
            _log.info("[2] Waiting for GPS origin (stationary pre-arm) ...")
            _wait_statustext(gcs, ["origin set"], timeout=35.0,
                             log=_log, label="2-origin")
            _log.info("[2] PASS — GPS origin set")

            # [3] Arm in ACRO; raise interlock; trajectory starts immediately
            _log.info("[3] Setting ACRO mode and arming ...")
            gcs.set_mode(1, timeout=10.0)
            gcs.send_rc_override({8: 1000})
            gcs.arm(force=True, timeout=15.0, rc_override={8: 1000})
            _log.info("[3] PASS — armed; trajectory now running")

            # Start trajectory clock now — vehicle was stationary at [0,0,0] pre-arm
            traj_phys.start()
            _log.info("[3] Trajectory clock started at arm")

            gcs.send_rc_override({3: 1700, 8: 2000})
            rc_stop = threading.Event()
            threading.Thread(
                target = _rc_keepalive,
                args   = (gcs, {3: 1700, 8: 2000}, rc_stop),
                daemon = True,
                name   = "rc_keepalive",
            ).start()

            # [4] Yaw alignment — compass fired pre-arm; just check the flag
            _log.info("[4] Checking yaw alignment ...")
            yaw_aligned = False
            t_chk = time.monotonic()
            while time.monotonic() - t_chk < 3.0 and not yaw_aligned:
                msg = gcs._recv(type=["EKF_STATUS_REPORT"],
                                blocking=True, timeout=0.5)
                if msg is not None and (msg.flags & 0x0001):
                    yaw_aligned = True
            if not yaw_aligned:
                _wait_statustext(gcs, ["yaw align"], timeout=60.0,
                                 log=_log, label="4-yaw")
            _log.info("[4] PASS — yaw aligned")

            # [5] GPS fusion — wait up to T_traj + 30 s
            _log.info("[5] Waiting for GPS fusion (trajectory T=%.0f s) ...", T_traj)
            _wait_statustext(gcs, ["is using gps"],
                             timeout=T_traj + 30.0, log=_log, label="5-fused")
            _log.info("[5] PASS — GPS fused")

            # Read EKF position immediately after fusion for error report
            ekf_pos = None
            t_ekf = time.monotonic()
            while time.monotonic() - t_ekf < 2.0:
                msg = gcs._recv(type=["LOCAL_POSITION_NED"],
                                blocking=True, timeout=0.5)
                if msg is not None:
                    ekf_pos = [msg.x, msg.y, msg.z]   # NED metres
                    break

            pos_now, _, _, _, _, _ = traj_phys.snapshot()
            traj_frac = 100.0 * traj_phys.elapsed / T_traj
            _log.info("Trajectory at GPS fusion: t=%.1f s / %.0f s (%.0f%%)",
                      traj_phys.elapsed, T_traj, traj_frac)
            _log.info("  physics pos = [%.3f, %.3f, %.3f] m", *pos_now)
            if ekf_pos is not None:
                ekf_err = [ekf_pos[i] - pos_now[i] for i in range(3)]
                ekf_dist = math.sqrt(sum(e**2 for e in ekf_err))
                _log.info("  EKF pos     = [%.3f, %.3f, %.3f] m", *ekf_pos)
                _log.info("  EKF error   = [%+.3f, %+.3f, %+.3f] m  |err|=%.3f m",
                          *ekf_err, ekf_dist)
            else:
                _log.warning("  LOCAL_POSITION_NED not received within 2 s of fusion")

        finally:
            try:
                rc_stop.set()
            except NameError:
                pass
            stop_ev.set()
            worker.join(timeout=2.0)
            try:
                gcs.close()
            except Exception:
                pass
            if tel_rows:
                tel_path = ctx.test_log_dir / "telemetry.csv"
                write_csv(tel_rows, tel_path)
                _log.info("Telemetry: %d rows -> %s", len(tel_rows), tel_path)
