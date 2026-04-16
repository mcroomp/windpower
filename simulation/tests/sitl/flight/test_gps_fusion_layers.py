"""
test_gps_fusion_layers.py -- GPS fusion diagnostic with orbital sensor.

Each test: hold stationary at orbit start position pre-arm, then after arm
accelerate in a 10 m radius CCW circle.  Speed ramps linearly 0 -> 5 m/s over
exactly one full circle (~25 s), then holds 5 m/s.  The rotating GPS velocity
heading gives EKFGSF (EK3_SRC1_YAW=8) enough signal to converge and set
yawAlignComplete, which unblocks GPS fusion.

Layer variants test different altitudes; the orbital pattern is identical.

test_gps_fusion_fast_circle: same fast-circle trajectory as test_kinematic_gps
  (hold->accel->const->decel->large orbit) but using the simpler level-body
  sensor model (BZ_LEVEL=[0,0,1], no tether-direction tilt).
  Used to isolate whether GPS glitch is caused by the trajectory itself or by
  the tilted body_z in the full mediator/sensor.py path.

Run:
  bash simulation/dev.sh test-stack-parallel --fresh -n 1 -k test_gps_fusion_layers
  bash simulation/dev.sh test-stack-parallel --fresh -n 1 -k test_gps_fusion_armed
  bash simulation/dev.sh test-stack-parallel --fresh -n 1 -k test_gps_fusion_trajectory
  bash simulation/dev.sh test-stack-parallel --fresh -n 1 -k test_gps_fusion_fast_circle
"""
from __future__ import annotations

import logging
import math
import sys
import threading
from pathlib import Path

import numpy as np
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
# Frame helpers
# ---------------------------------------------------------------------------

def _rpy_from_R(R: np.ndarray) -> np.ndarray:
    """ZYX Euler [roll, pitch, yaw] in radians from a 3x3 body-to-NED matrix."""
    pitch = math.asin(-float(np.clip(R[2, 0], -1.0, 1.0)))
    roll  = math.atan2(float(R[2, 1]), float(R[2, 2]))
    yaw   = math.atan2(float(R[1, 0]), float(R[0, 0]))
    return np.array([roll, pitch, yaw])


# ---------------------------------------------------------------------------
# Unified orbital sensor worker
# ---------------------------------------------------------------------------

_ORB_R     = 10.0   # orbit radius [m]
_ORB_VMAX  = 5.0    # peak speed [m/s]
# T_ramp: one full circle (2π r) at mean speed (v_max/2)  →  4π r / v_max
_ORB_TRAMP = 4.0 * math.pi * _ORB_R / _ORB_VMAX   # ≈ 25.13 s


def _orbital_sensor_worker(
    stop_event:  threading.Event,
    p_hold:      np.ndarray,          # NED [m] — hold position pre-arm (= orbit start)
    alt_d:       float,               # NED Z of orbit plane [m] (< 0 = above ground)
    start_event: threading.Event,    # set after arm → begin circular motion
    telemetry:   list | None = None,
    tel_hz:      int = 10,
) -> None:
    """Lockstep sensor worker.

    Hold phase  (until start_event):
        Stationary at p_hold, level attitude, zero velocity.
        Sends correct gravity-consistent accel_body so EKF tilt aligns.

    Orbit phase (after start_event):
        10 m radius CCW circle.  Speed ramps 0 → 5 m/s over one full circle
        (T_ramp ≈ 25 s), then holds 5 m/s.  Yaw tracks GPS velocity heading
        (heading gap = 0) via build_vel_aligned_frame.  Gyro computed from
        finite-diff of the orientation matrix.

    Replies to every SITL servo packet (lockstep — never skip a reply).
    """
    from frames import build_vel_aligned_frame, build_orb_frame

    GRAVITY_NED = np.array([0.0, 0.0, _GRAVITY])
    BZ_LEVEL    = np.array([0.0, 0.0, 1.0])

    # Pre-arm constants (level, stationary)
    R_HOLD  = build_orb_frame(BZ_LEVEL)
    RPY_HOLD = _rpy_from_R(R_HOLD)
    AB_HOLD  = R_HOLD.T @ (-GRAVITY_NED)   # = [0, 0, -g] for level body

    # Orbit start angle
    theta0 = math.atan2(float(p_hold[1]), float(p_hold[0]))   # = 0 for [r,0,alt]

    DT_FD  = 1e-3          # finite-diff step for gyro [s]
    tel_dt = 1.0 / tel_hz

    sitl      = SITLInterface(recv_port=StackConfig.SITL_JSON_PORT)
    sitl.bind()

    t_arm     = None       # sim time when orbit started
    t_tel     = 0.0
    connected = False

    def _orbit_state(t: float):
        """Return (pos, vel, accel_world) for time t since orbit start."""
        if t < _ORB_TRAMP:
            v  = _ORB_VMAX * t / _ORB_TRAMP
            at = _ORB_VMAX / _ORB_TRAMP          # tangential accel
            th = theta0 + (_ORB_VMAX * t * t) / (2.0 * _ORB_R * _ORB_TRAMP)
        else:
            v  = _ORB_VMAX
            at = 0.0
            th = theta0 + 2.0 * math.pi + (_ORB_VMAX / _ORB_R) * (t - _ORB_TRAMP)
        sn, cs = math.sin(th), math.cos(th)
        pos = np.array([_ORB_R * cs, _ORB_R * sn, alt_d])
        vel = np.array([-v * sn,      v * cs,      0.0  ])
        ac  = v * v / _ORB_R                    # centripetal magnitude
        aw  = np.array([
            -ac * cs - at * sn,                 # centripetal-inward + tangential
            -ac * sn + at * cs,
            0.0,
        ])
        return pos, vel, aw

    try:
        while not stop_event.is_set():
            servos = sitl.recv_servos()
            if servos is None:
                continue
            if not connected:
                log.info("orbital_worker: SITL connected")
                connected = True

            t_sim = sitl.sim_now()

            if t_arm is None and start_event.is_set():
                t_arm = t_sim
                log.info("orbital_worker: orbit started at t_sim=%.2f s", t_arm)

            if t_arm is None:
                # ── Hold phase ────────────────────────────────────────────
                sitl.send_state(
                    pos_ned    = p_hold,
                    vel_ned    = np.zeros(3),
                    rpy_rad    = RPY_HOLD,
                    accel_body = AB_HOLD,
                    gyro_body  = np.zeros(3),
                )
            else:
                # ── Orbit phase ───────────────────────────────────────────
                t       = t_sim - t_arm
                pos, vel, aw = _orbit_state(t)

                R       = build_vel_aligned_frame(BZ_LEVEL, vel)
                rpy     = _rpy_from_R(R)

                # Gyro from orientation finite-diff
                _, vel1, _ = _orbit_state(t + DT_FD)
                R1      = build_vel_aligned_frame(BZ_LEVEL, vel1)
                S       = R.T @ ((R1 - R) / DT_FD)
                gyro    = np.array([S[2, 1], S[0, 2], S[1, 0]])

                accel_body = R.T @ (aw - GRAVITY_NED)

                sitl.send_state(
                    pos_ned    = pos,
                    vel_ned    = vel,
                    rpy_rad    = rpy,
                    accel_body = accel_body,
                    gyro_body  = gyro,
                )

                if telemetry is not None and t_sim >= t_tel:
                    t_tel = t_sim + tel_dt
                    v_horiz = math.hypot(float(vel[0]), float(vel[1]))
                    telemetry.append(TelRow(
                        t_sim      = t_sim,
                        pos_x      = float(pos[0]),
                        pos_y      = float(pos[1]),
                        pos_z      = float(pos[2]),
                        vel_x      = float(vel[0]),
                        vel_y      = float(vel[1]),
                        vel_z      = float(vel[2]),
                        v_horiz_ms = v_horiz,
                    ))
    finally:
        sitl.close()


# ---------------------------------------------------------------------------
# Helpers shared by all tests
# ---------------------------------------------------------------------------

def _wait_statustext(gcs, keywords, timeout, log, label):
    """Poll MAVLink until any keyword matches a STATUSTEXT or timeout.

    Logs EKF_STATUS_REPORT, GPS_RAW_INT, ATTITUDE, and LOCAL_POSITION_NED.
    Returns the matching STATUSTEXT string, or calls pytest.fail on timeout.
    """
    t0         = gcs.sim_now()
    t_last_ekf = t0 - 1.0
    t_last_gps = t0 - 1.0
    t_last_att = t0 - 1.0
    t_last_pos = t0
    last_flags = None

    while gcs.sim_now() - t0 < timeout:
        msg = gcs._recv(  # type: ignore[union-attr]
            type=["STATUSTEXT", "EKF_STATUS_REPORT", "LOCAL_POSITION_NED",
                  "GPS_RAW_INT", "ATTITUDE"],
            blocking=True, timeout=0.2,
        )
        now     = gcs.sim_now()
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
            log.info(
                "  [%s t=%5.1f s] GPS fix=%d sats=%d hdop=%.1f spd=%.2f m/s",
                label, elapsed,
                msg.fix_type, msg.satellites_visible,
                msg.eph / 100.0,
                msg.vel / 100.0,
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
                "  [%s t=%5.1f s] LOCAL_POS z=%.2f vz=%.3f",
                label, elapsed, msg.z, msg.vz,
            )
            t_last_pos = now

    pytest.fail(f"[{label}] timed out after {timeout:.0f} s waiting for: {keywords}")


def _rc_keepalive(gcs, channels: dict, stop_event: threading.Event, interval: float = 0.4):
    """Send RC_CHANNELS_OVERRIDE every interval seconds until stop_event."""
    while not stop_event.is_set():
        try:
            gcs.send_rc_override(channels)
        except Exception:
            pass
        stop_event.wait(interval)


# ---------------------------------------------------------------------------
# Layer definitions
# ---------------------------------------------------------------------------
# Each layer tests a different altitude.  The orbital sensor pattern is the
# same for all: stationary pre-arm, then 10 m circle accelerating to 5 m/s.

_LAYERS = [
    ("L0_baseline",  "Level vehicle at 50 m altitude",          -50.0),
    ("L1_rawes_alt", "RAWES equilibrium altitude (14.12 m)",    -14.12),
]


# ---------------------------------------------------------------------------
# test_gps_fusion_layers
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "layer_id, description, alt_ned",
    _LAYERS,
    ids=[l[0] for l in _LAYERS],
)
def test_gps_fusion_layers(tmp_path, request, layer_id, description, alt_ned):
    """
    GPS fusion via 10 m orbital trajectory.

    Pattern:
      [1] Stationary at orbit start position, EKF tilt aligns.
      [2] Wait for GPS origin (~20 s).
      [3] Arm in ACRO.
      [4] Start 10 m circles — speed ramps 0→5 m/s over one circle.
      [5] Assert "EKF3 IMU0 is using GPS" within 90 s.
    """
    p_hold = np.array([_ORB_R, 0.0, alt_ned])

    with _sitl_stack(
        tmp_path,
        log_name   = f"gps_layers_{layer_id}",
        log_prefix = "gps_layers",
        test_name  = request.node.name,
    ) as ctx:
        _log = ctx.log
        _log.info("Layer: %s — %s  (alt_ned=%.2f m)", layer_id, description, alt_ned)
        _log.info("Orbit: r=%.0f m  v_max=%.0f m/s  T_ramp=%.1f s",
                  _ORB_R, _ORB_VMAX, _ORB_TRAMP)

        tel_rows: list[TelRow] = []
        stop_ev  = threading.Event()
        start_ev = threading.Event()

        worker = threading.Thread(
            target  = _orbital_sensor_worker,
            args    = (stop_ev, p_hold, alt_ned, start_ev),
            kwargs  = {"telemetry": tel_rows},
            daemon  = True,
            name    = f"orb_{layer_id}",
        )
        worker.start()

        rc_stop = threading.Event()
        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            # [1] EKF tilt alignment
            _log.info("[1] Waiting for EKF tilt alignment ...")
            if not gcs.wait_ekf_attitude(timeout=20.0):
                pytest.fail(f"[{layer_id}] EKF tilt alignment timed out")
            _log.info("[1] PASS")

            # [2] GPS origin
            _log.info("[2] Waiting for GPS origin ...")
            _wait_statustext(gcs, ["origin set"], timeout=35.0, log=_log, label="2-origin")
            _log.info("[2] PASS")

            # [3] Arm in ACRO
            _log.info("[3] Arming in ACRO ...")
            gcs.set_mode(1, timeout=10.0)
            gcs.send_rc_override({8: 1000})
            gcs.arm(force=True, timeout=15.0, rc_override={8: 1000})
            gcs.send_rc_override({3: 1700, 8: 2000})
            rc_thread = threading.Thread(
                target=_rc_keepalive,
                args=(gcs, {3: 1700, 8: 2000}, rc_stop),
                daemon=True, name="rc_keepalive",
            )
            rc_thread.start()
            _log.info("[3] PASS — armed")

            # [4] Start orbital motion
            start_ev.set()
            _log.info("[4] Orbital motion started (10 m circle, 0->5 m/s over one lap)")

            # [5] Wait for GPS fusion
            _log.info("[5] Waiting for GPS fusion ...")
            _wait_statustext(gcs, ["is using gps"], timeout=90.0, log=_log, label="5-gps")
            _log.info("[5] PASS — GPS fused")

        finally:
            rc_stop.set()
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


# ---------------------------------------------------------------------------
# test_gps_fusion_armed
# ---------------------------------------------------------------------------

def test_gps_fusion_armed(tmp_path, request):
    """
    GPS fusion step-by-step diagnostic at RAWES altitude.

    Same orbital sensor as test_gps_fusion_layers but logs each EKF milestone:
    tilt alignment, GPS origin, yaw alignment, GPS fusion.
    """
    alt_ned = -14.12
    p_hold  = np.array([_ORB_R, 0.0, alt_ned])

    with _sitl_stack(
        tmp_path,
        log_name   = "gps_fusion_armed",
        log_prefix = "gps_fusion",
        test_name  = request.node.name,
    ) as ctx:
        _log = ctx.log
        _log.info("Sensor: stationary at [%.1f, 0, %.1f], then 10 m circles 0->5 m/s",
                  _ORB_R, alt_ned)

        tel_rows: list[TelRow] = []
        stop_ev  = threading.Event()
        start_ev = threading.Event()

        worker = threading.Thread(
            target  = _orbital_sensor_worker,
            args    = (stop_ev, p_hold, alt_ned, start_ev),
            kwargs  = {"telemetry": tel_rows},
            daemon  = True,
            name    = "sensor_armed",
        )
        worker.start()

        rc_stop = threading.Event()
        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            # [1] EKF tilt alignment
            _log.info("[1] Waiting for EKF tilt alignment ...")
            if not gcs.wait_ekf_attitude(timeout=20.0):
                pytest.fail("[1] EKF tilt alignment timed out")
            _log.info("[1] PASS")

            # [2] GPS origin
            _log.info("[2] Waiting for GPS origin (stationary, expect ~20 s) ...")
            _wait_statustext(gcs, ["origin set"], timeout=35.0, log=_log, label="2-origin")
            _log.info("[2] PASS")

            # [3] Arm in ACRO
            _log.info("[3] Arming in ACRO ...")
            gcs.set_mode(1, timeout=10.0)
            gcs.send_rc_override({8: 1000})
            gcs.arm(force=True, timeout=15.0, rc_override={8: 1000})
            gcs.send_rc_override({3: 1700, 8: 2000})
            rc_thread = threading.Thread(
                target=_rc_keepalive,
                args=(gcs, {3: 1700, 8: 2000}, rc_stop),
                daemon=True, name="rc_keepalive",
            )
            rc_thread.start()
            _log.info("[3] PASS — armed")

            # [4] Start orbital motion
            start_ev.set()
            _log.info("[4] Orbital motion started")

            # [5] Yaw alignment
            _log.info("[5] Waiting for yaw alignment ...")
            _wait_statustext(gcs, ["yaw align", "yaw aligned"], timeout=60.0,
                             log=_log, label="5-yaw")
            _log.info("[5] PASS — yaw aligned")

            # [6] GPS fusion
            _log.info("[6] Waiting for GPS fusion ...")
            _wait_statustext(gcs, ["is using gps"], timeout=60.0, log=_log, label="6-gps")
            _log.info("[6] PASS — GPS fused")

        finally:
            rc_stop.set()
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


# ---------------------------------------------------------------------------
# test_gps_fusion_trajectory
# ---------------------------------------------------------------------------

def test_gps_fusion_trajectory(tmp_path, request):
    """
    GPS fusion at RAWES equilibrium altitude using the orbital sensor.

    Same pattern as test_gps_fusion_armed but uses the equilibrium altitude
    from steady_state_starting.json so this test exercises the real operating
    condition (alt ~14 m, r_h ~99 m from anchor).
    """
    ss      = json.loads(_STEADY_STATE_JSON.read_text())
    alt_ned = float(ss["pos"][2])   # NED Z (<0 = above ground)

    p_hold  = np.array([_ORB_R, 0.0, alt_ned])

    with _sitl_stack(
        tmp_path,
        log_name   = "gps_fusion_trajectory",
        log_prefix = "gps_fusion",
        test_name  = request.node.name,
    ) as ctx:
        _log = ctx.log
        _log.info("Steady-state alt_ned=%.2f m, orbit r=%.0f m v_max=%.0f m/s",
                  alt_ned, _ORB_R, _ORB_VMAX)

        tel_rows: list[TelRow] = []
        stop_ev  = threading.Event()
        start_ev = threading.Event()

        worker = threading.Thread(
            target  = _orbital_sensor_worker,
            args    = (stop_ev, p_hold, alt_ned, start_ev),
            kwargs  = {"telemetry": tel_rows},
            daemon  = True,
            name    = "traj_worker",
        )
        worker.start()

        rc_stop = threading.Event()
        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            _log.info("[1] Waiting for EKF tilt alignment ...")
            if not gcs.wait_ekf_attitude(timeout=20.0):
                pytest.fail("[1] EKF tilt alignment timed out")
            _log.info("[1] PASS")

            _log.info("[2] Waiting for GPS origin ...")
            _wait_statustext(gcs, ["origin set"], timeout=35.0, log=_log, label="2-origin")
            _log.info("[2] PASS")

            _log.info("[3] Arming in ACRO ...")
            gcs.set_mode(1, timeout=10.0)
            gcs.send_rc_override({8: 1000})
            gcs.arm(force=True, timeout=15.0, rc_override={8: 1000})
            gcs.send_rc_override({3: 1700, 8: 2000})
            rc_thread = threading.Thread(
                target=_rc_keepalive,
                args=(gcs, {3: 1700, 8: 2000}, rc_stop),
                daemon=True, name="rc_keepalive",
            )
            rc_thread.start()
            _log.info("[3] PASS — armed")

            start_ev.set()
            _log.info("[4] Orbital motion started — waiting for GPS fusion ...")
            _wait_statustext(gcs, ["is using gps"], timeout=90.0, log=_log, label="4-gps")
            _log.info("[4] PASS — GPS fused")

        finally:
            rc_stop.set()
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


# ---------------------------------------------------------------------------
# test_gps_fusion_fast_circle
# ---------------------------------------------------------------------------

def _fast_circle_level_worker(
    stop_event: threading.Event,
    anchor:     np.ndarray,
    p_eq:       np.ndarray,
    v_orb_eq:   float,
    orbit_dir:  int   = +1,
    v_fast:     float = 5.0,
    r_circle:   float = 5.0,
    n_fast:     int   = 1,
    T_lead:     float = 10.0,
    t_hold:     float = 15.0,
) -> None:
    """Sensor worker: fast-circle trajectory, level body (BZ_LEVEL=[0,0,1]).

    Uses the identical trajectory as test_kinematic_gps (via
    make_fast_circle_orbit_kinematics) but replaces the tilted
    tether-direction body_z with BZ_LEVEL=[0,0,1].  This means:
      - gravity always reads as [0, 0, -9.81] in body frame (EKF-friendly)
      - body Z is vertical so yaw IS the heading; no tilt-induced coupling
      - gyro is purely the heading rate, not mixed with tilt rotation

    If GPS fuses cleanly here but not in test_kinematic_gps, the problem lies
    in the tilted body_z path in sensor.py, not in the trajectory itself.
    """
    from frames import build_vel_aligned_frame
    from kinematic import make_fast_circle_orbit_kinematics

    GRAVITY_NED = np.array([0.0, 0.0, _GRAVITY])
    BZ_LEVEL    = np.array([0.0, 0.0, 1.0])
    DT_FD       = 1e-3

    traj_fn, _ = make_fast_circle_orbit_kinematics(
        anchor_pos = anchor,
        p_eq       = p_eq,
        v_orb_eq   = v_orb_eq,
        v_fast     = v_fast,
        n_fast     = n_fast,
        T_lead     = T_lead,
        r_circle   = r_circle,
        orbit_dir  = orbit_dir,
        t_hold     = t_hold,
    )

    # Pre-compute the tangential direction at t=t_hold so the hold-phase body
    # heading matches the first moving step -- prevents yaw jump at t_hold.
    _, vel_start_ref_raw = traj_fn(t_hold + 0.05)
    spd = float(np.hypot(vel_start_ref_raw[0], vel_start_ref_raw[1]))
    vel_start_ref = (vel_start_ref_raw / spd) if spd > 0.01 else np.array([0.0, 1.0, 0.0])

    def _R_for_vel(v: np.ndarray) -> np.ndarray:
        vh = float(np.hypot(v[0], v[1]))
        vref = v if vh >= 0.1 else vel_start_ref
        return build_vel_aligned_frame(BZ_LEVEL, vref)

    sitl = SITLInterface(recv_port=StackConfig.SITL_JSON_PORT)
    sitl.bind()
    try:
        while not stop_event.is_set():
            servos = sitl.recv_servos()
            if servos is None:
                continue

            t_sim       = sitl.sim_now()
            pos, vel    = traj_fn(t_sim)
            accel_world = traj_fn.accel(t_sim)

            R    = _R_for_vel(vel)
            rpy  = _rpy_from_R(R)

            # Gyro from finite-diff of R -- consistent with velocity-heading rotation
            _, vel1 = traj_fn(t_sim + DT_FD)
            R1      = _R_for_vel(vel1)
            S       = R.T @ ((R1 - R) / DT_FD)
            gyro    = np.array([S[2, 1], S[0, 2], S[1, 0]])

            accel_body = R.T @ (accel_world - GRAVITY_NED)

            sitl.send_state(
                pos_ned    = pos,
                vel_ned    = vel,
                rpy_rad    = rpy,
                accel_body = accel_body,
                gyro_body  = gyro,
            )
    finally:
        sitl.close()


def test_gps_fusion_fast_circle(tmp_path, request):
    """
    GPS fusion via fast-circle trajectory using level body (BZ_LEVEL=[0,0,1]).

    Identical trajectory to test_kinematic_gps (hold 15 s -> accel circle
    r=5 m v=5 m/s -> 1 const circle -> decel -> large orbit lead-in) but
    using a level body instead of the tether-direction body_z.

    Purpose: isolate whether GPS glitch in test_kinematic_gps is caused by
    the trajectory (too fast / too jerky for EKF) or by the tilted body frame
    in sensor.py.

    Pass condition: "EKF3 IMU0 is using GPS" within 70 s AND no GPS glitch
    flag (0x8000) in EKF_STATUS_REPORT for 10 s after fusion.
    """
    ss     = json.loads(_STEADY_STATE_JSON.read_text())
    p_eq   = np.array(ss["pos"], dtype=float)
    anchor = np.array([0.0, 0.0, 0.0])
    v_orb  = 0.96

    with _sitl_stack(
        tmp_path,
        log_name   = "gps_fc_level",
        log_prefix = "gps_fc",
        test_name  = request.node.name,
    ) as ctx:
        _log    = ctx.log
        stop_ev = threading.Event()

        worker = threading.Thread(
            target = _fast_circle_level_worker,
            args   = (stop_ev, anchor, p_eq, v_orb),
            kwargs = dict(orbit_dir=1, v_fast=5.0, r_circle=5.0,
                          n_fast=1, T_lead=10.0, t_hold=15.0),
            daemon = True,
            name   = "fc_level_worker",
        )
        worker.start()

        rc_stop = threading.Event()
        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            _log.info("[1] EKF tilt alignment ...")
            if not gcs.wait_ekf_attitude(timeout=20.0):
                pytest.fail("EKF tilt alignment timed out")
            _log.info("[1] PASS")

            _log.info("[2] GPS origin ...")
            _wait_statustext(gcs, ["origin set"], timeout=35.0, log=_log, label="2-origin")
            _log.info("[2] PASS")

            _log.info("[3] Arming in ACRO ...")
            gcs.set_mode(1, timeout=10.0)
            gcs.send_rc_override({8: 1000})
            gcs.arm(force=True, timeout=15.0, rc_override={8: 1000})
            gcs.send_rc_override({3: 1700, 8: 2000})
            rc_thread = threading.Thread(
                target = _rc_keepalive,
                args   = (gcs, {3: 1700, 8: 2000}, rc_stop),
                daemon = True, name = "rc_keepalive",
            )
            rc_thread.start()
            _log.info("[3] PASS -- armed")

            _log.info("[4] Waiting for GPS fusion (70 s) ...")
            _wait_statustext(gcs, ["is using gps"], timeout=70.0,
                             log=_log, label="4-gps")
            t_fused = gcs.sim_now()
            _log.info("[4] PASS -- GPS fused at t=%.1f s", t_fused)

            # [5] GPS must stay stable (no 0x8000 glitch) for 10 s after fusion
            _log.info("[5] Checking GPS stability for 10 s ...")
            t0_check     = gcs.sim_now()
            glitch_count = 0
            GPS_GLITCH   = 0x8000
            while gcs.sim_now() - t0_check < 10.0:
                msg = gcs._recv(type="EKF_STATUS_REPORT", blocking=True, timeout=0.2)
                if msg is not None and (msg.flags & GPS_GLITCH):
                    glitch_count += 1
                    _log.warning("[5] GPS glitch at t=%.1f s  flags=0x%04x",
                                 gcs.sim_now(), msg.flags)
            _log.info("[5] GPS glitch frames in 10 s window: %d", glitch_count)
            assert glitch_count == 0, (
                f"GPS glitch fired {glitch_count} times in 10 s after fusion "
                f"(t_fused={t_fused:.1f} s).  Level body, same trajectory as "
                f"test_kinematic_gps -- glitch is NOT trajectory-caused."
            )
            _log.info("[5] PASS -- GPS stable after fusion")

        finally:
            rc_stop.set()
            stop_ev.set()
            worker.join(timeout=2.0)
            try:
                gcs.close()
            except Exception:
                pass
