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
  bash simulation/dev.sh test-stack-parallel --fresh -n 1 -k test_gps_fusion_dual_gps
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
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=ctx.mavlink_log)
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
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=ctx.mavlink_log)
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
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=ctx.mavlink_log)
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
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=ctx.mavlink_log)
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


# ---------------------------------------------------------------------------
# test_gps_fusion_fast_circle_tilted
# ---------------------------------------------------------------------------

def _fast_circle_tilted_worker(
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
    """Sensor worker: fast-circle trajectory, TILTED body (body_z = tether direction).

    Identical to _fast_circle_level_worker except body_z = normalize(pos - anchor)
    at each step, matching what sensor.py does in the full mediator kinematic path.

    Purpose: confirm that tilted body_z (not the trajectory itself) causes GSF
    convergence failure.  If this test fails where the level variant passes,
    the root cause is in how the tilted body gyro/accel are seen by ArduPilot.
    """
    from frames import build_vel_aligned_frame
    from kinematic import make_fast_circle_orbit_kinematics

    GRAVITY_NED = np.array([0.0, 0.0, _GRAVITY])
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

    _, vel_start_ref_raw = traj_fn(t_hold + 0.05)
    spd = float(np.hypot(vel_start_ref_raw[0], vel_start_ref_raw[1]))
    vel_start_ref = (vel_start_ref_raw / spd) if spd > 0.01 else np.array([0.0, 1.0, 0.0])

    def _bz_for_pos(pos: np.ndarray) -> np.ndarray:
        """Tether direction: normalize(pos - anchor), fallback to [0,0,1]."""
        diff = pos - anchor
        n = float(np.linalg.norm(diff))
        return diff / n if n > 0.1 else np.array([0.0, 0.0, 1.0])

    def _R_for_state(pos: np.ndarray, vel: np.ndarray) -> np.ndarray:
        bz   = _bz_for_pos(pos)
        vh   = float(np.hypot(vel[0], vel[1]))
        vref = vel if vh >= 0.1 else vel_start_ref
        return build_vel_aligned_frame(bz, vref)

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

            R    = _R_for_state(pos, vel)
            rpy  = _rpy_from_R(R)

            _, vel1 = traj_fn(t_sim + DT_FD)
            R1      = _R_for_state(pos + vel * DT_FD, vel1)
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


def test_gps_fusion_fast_circle_tilted(tmp_path, request):
    """
    GPS fusion via fast-circle trajectory using TILTED body (tether-direction body_z).

    Identical trajectory to test_gps_fusion_fast_circle but body_z = normalize(pos - anchor)
    at each step, matching the full mediator/sensor.py kinematic path.

    Pass condition: "EKF3 IMU0 is using GPS" within 70 s AND no GPS glitch
    flag (0x8000) for 10 s after fusion.

    If this FAILS where test_gps_fusion_fast_circle (level body) PASSES, the
    tilted body_z is the root cause of GSF convergence failure in test_kinematic_gps.
    """
    ss     = json.loads(_STEADY_STATE_JSON.read_text())
    p_eq   = np.array(ss["pos"], dtype=float)
    anchor = np.array([0.0, 0.0, 0.0])
    v_orb  = 0.96

    with _sitl_stack(tmp_path, test_name=request.node.name) as ctx:
        _log    = ctx.log
        stop_ev = threading.Event()

        worker = threading.Thread(
            target = _fast_circle_tilted_worker,
            args   = (stop_ev, anchor, p_eq, v_orb),
            kwargs = dict(orbit_dir=1, v_fast=5.0, r_circle=5.0,
                          n_fast=1, T_lead=10.0, t_hold=15.0),
            daemon = True,
            name   = "fc_tilted_worker",
        )
        worker.start()

        rc_stop = threading.Event()
        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=ctx.mavlink_log)
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
                f"(t_fused={t_fused:.1f} s).  Tilted body_z, same trajectory as "
                f"test_gps_fusion_fast_circle -- glitch IS tilt-body-caused."
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


# ---------------------------------------------------------------------------
# test_gps_fusion_physics_driven  (level circle -- EKFGSF heading from motion)
# test_gps_fusion_tilted_hover    (tilted body -- compass yaw, no motion needed)
# ---------------------------------------------------------------------------
#
# Physics-driven workers replace kinematic state-override for sensor consistency.
# RigidBodyDynamics feeds gyro/accel from the integrator so they are always
# mutually consistent (no spurious gyro-bias estimate in EKF).
#
# test_gps_fusion_physics_driven:
#   Level body (BZ=[0,0,1]).  Stationary hold -> accel East -> CCW circle.
#   EKFGSF (EK3_SRC1_YAW=8, GPS velocity yaw) converges from heading rotation.
#   GPS fuses around t~75 s.
#
# test_gps_fusion_tilted_hover:
#   Tilted body (BZ = tether direction, ~82 deg from vertical).
#   Compass enabled (COMPASS_ENABLE=1, COMPASS_USE=1) -> yaw known from boot;
#   no circular motion needed.  GPS should fuse by t~20 s.
#   Validates that a tilted IMU produces consistent sensor readings ArduPilot
#   can accept without EKF divergence.

_T_ARM_S    = 12.0
_T_ACCEL_S  =  5.0    # straight-line accel duration [s]
_V_FAST_MS  =  5.0    # circle speed [m/s]
_R_CIRCLE_M = 10.0    # circle radius [m]
_T_END_S    = 100.0   # circle test run length [s] (GPS fuses ~75 s)

_T_HOVER_ARM_S = 12.0
_T_HOVER_END_S = 70.0   # delAngBiasLearned needs ~30s after EKF start (~t=4s) = ~t=34s; GPS fuses shortly after


def _simple_circle_physics_worker(
    stop_event: threading.Event,
    p_start:    np.ndarray,
    t_arm:      float = _T_ARM_S,
    t_accel:    float = _T_ACCEL_S,
    v_fast:     float = _V_FAST_MS,
    r_circle:   float = _R_CIRCLE_M,
) -> None:
    """Physics-driven sensor worker: stationary hold → straight-line accel → constant circle.

    Phase 1  (t < t_arm):
      Level body [0,0,1], hub stationary at p_start.  F_net exactly cancels
      gravity.  accel_body = [0,0,-g], gyro = 0.  Clean hover for arming.

    Phase 2  (t_arm <= t < t_arm + t_accel):
      Straight-line acceleration East: a = v_fast/t_accel.
      pos_d / vel_d / accel_d all analytic.  Level body, heading = East.

    Phase 3  (t >= t_arm + t_accel):
      Constant-speed CCW circle: r=r_circle, v=v_fast, omega=v_fast/r_circle.
      Center = p_accel_end + [-r_circle, 0, 0]  (South of accel endpoint).
      Level body; heading tracks velocity direction.

    All sensors from RigidBodyDynamics -- gyro/accel always mutually consistent.
    Gains critically damped: KD_POS = 2*sqrt(KP_POS), KD_ORI = 2*sqrt(KP_ORI*I_zz).
    """
    from dynamics import RigidBodyDynamics
    from frames import build_vel_aligned_frame

    GRAVITY_NED = np.array([0.0, 0.0, _GRAVITY])
    BZ_LEVEL    = np.array([0.0, 0.0, 1.0])
    VEL_EAST    = np.array([0.0, 1.0, 0.0])   # heading reference during hold
    DT          = 1.0 / 400.0

    MASS   = 5.0
    I_BODY = [5.0, 5.0, 10.0]   # [Ixx, Iyy, Izz]

    # Critically-damped gains:
    #   position: omega_n = sqrt(KP_POS) = 7.07 rad/s, KD = 2*omega_n = 14.1
    #   orientation (yaw axis I_zz=10): omega_n = sqrt(KP_ORI/I_zz) = 3.16 rad/s
    #                                   KD = 2*omega_n*I_zz = 63.2 (use 65)
    KP_POS = 50.0
    KD_POS = 14.2
    KP_ORI = 100.0
    KD_ORI = 65.0

    # Trajectory geometry
    p_start   = np.asarray(p_start, dtype=float).copy()
    a_accel   = v_fast / t_accel                          # [m/s^2]
    d_accel   = 0.5 * v_fast * t_accel                   # [m] distance East
    p_accel_end = p_start + np.array([0.0, d_accel, 0.0])
    c_circle    = p_accel_end - np.array([r_circle, 0.0, 0.0])  # center South of endpoint
    omega_c     = v_fast / r_circle                       # [rad/s] CCW

    def _traj(t: float):
        """Returns (pos_d, vel_d, accel_d) for sim time t."""
        if t < t_arm:
            return (p_start.copy(),
                    np.zeros(3),
                    np.zeros(3))
        u = t - t_arm
        if u < t_accel:
            return (p_start + np.array([0.0, 0.5 * a_accel * u * u, 0.0]),
                    np.array([0.0, a_accel * u, 0.0]),
                    np.array([0.0, a_accel, 0.0]))
        # Circle phase: theta=0 when hub is at p_accel_end (North of center)
        u2 = u - t_accel
        th  = omega_c * u2
        cs, sn = float(np.cos(th)), float(np.sin(th))
        pos_d   = c_circle + np.array([r_circle * cs, r_circle * sn, 0.0])
        vel_d   = np.array([-v_fast * sn, v_fast * cs, 0.0])
        accel_d = np.array([-(v_fast**2 / r_circle) * cs,
                             -(v_fast**2 / r_circle) * sn,
                             0.0])
        return pos_d, vel_d, accel_d

    def _R_desired(vel: np.ndarray) -> np.ndarray:
        vh   = float(np.hypot(vel[0], vel[1]))
        vref = vel if vh >= 0.1 else VEL_EAST
        return build_vel_aligned_frame(BZ_LEVEL, vref)

    R0  = _R_desired(VEL_EAST)
    dyn = RigidBodyDynamics(
        mass   = MASS,
        I_body = I_BODY,
        pos0   = p_start.copy(),
        vel0   = np.zeros(3),
        R0     = R0,
        omega0 = np.zeros(3),
    )

    sitl = SITLInterface(recv_port=StackConfig.SITL_JSON_PORT)
    sitl.bind()
    try:
        while not stop_event.is_set():
            servos = sitl.recv_servos()
            if servos is None:
                continue

            t_sim            = sitl.sim_now()
            pos_d, vel_d, accel_d = _traj(t_sim)
            R_d              = _R_desired(vel_d)

            pos   = dyn._pos
            vel   = dyn._vel
            R     = dyn._R
            omega = dyn._omega

            # Position PD + feedforward (dynamics adds gravity internally).
            a_ff  = accel_d - GRAVITY_NED
            F_net = MASS * (a_ff + KP_POS * (pos_d - pos) + KD_POS * (vel_d - vel))

            # Orientation PD via axis-angle error.
            R_err = R.T @ R_d
            tr    = float(np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0))
            theta = float(np.arccos(tr))
            if theta > 1e-6:
                axis_body  = np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ]) / (2.0 * float(np.sin(theta)))
                axis_world = R @ axis_body
            else:
                axis_world = np.zeros(3)
            M_net = KP_ORI * theta * axis_world - KD_ORI * omega

            hub = dyn.step(F_net, M_net, DT)

            R_new      = hub["R"]
            accel_body = R_new.T @ (F_net / MASS)
            gyro_body  = R_new.T @ hub["omega"]

            sitl.send_state(
                pos_ned    = hub["pos"],
                vel_ned    = hub["vel"],
                rpy_rad    = _rpy_from_R(R_new),
                accel_body = accel_body,
                gyro_body  = gyro_body,
            )
    finally:
        sitl.close()


def test_gps_fusion_physics_driven(tmp_path, request):
    """
    GPS fusion using a physics-driven (computed-torque) worker on a fixed schedule.

    Simple level-body trajectory -- no tilt, no complex kinematics:
      t=0-12 s    stationary hold at p_start  (EKF aligns, GPS acquires)
      t=12 s      arm command sent (test fails if rejected)
      t=12-17 s   straight-line accel East: 0 -> 5 m/s over 5 s
      t=17-80 s   constant CCW circle: r=10 m, v=5 m/s, omega=0.5 rad/s
                  heading rotates 28.6 deg/s -> full circle in 12.6 s
                  EKFGSF expects ~1-2 circles to converge -> GPS fused t~30-40 s

    No mid-run assertions.  Read gcs.log and run analyse_mavlink after.
    """
    ss    = json.loads(_STEADY_STATE_JSON.read_text())
    p_eq  = np.array(ss["pos"], dtype=float)

    with _sitl_stack(tmp_path, test_name=request.node.name) as ctx:
        _log    = ctx.log
        stop_ev = threading.Event()

        worker = threading.Thread(
            target = _simple_circle_physics_worker,
            args   = (stop_ev, p_eq),
            kwargs = dict(
                t_arm    = _T_ARM_S,
                t_accel  = _T_ACCEL_S,
                v_fast   = _V_FAST_MS,
                r_circle = _R_CIRCLE_M,
            ),
            daemon = True,
            name   = "sc_physics_worker",
        )
        worker.start()

        rc_stop = threading.Event()
        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=ctx.mavlink_log)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            # t ~ 0-12 s: level hover, let EKF and GPS settle.
            _log.info("Holding %.0f s for EKF/GPS init ...", _T_ARM_S)
            gcs.sim_sleep(_T_ARM_S)

            # t ~ 12 s: arm.  Fail immediately if rejected.
            _log.info("Sending arm at t=%.1f s ...", gcs.sim_now())
            gcs.set_mode(1, timeout=5.0)
            gcs.send_rc_override({8: 1000})
            gcs.arm(force=True, timeout=8.0, rc_override={8: 1000})
            gcs.send_rc_override({3: 1700, 8: 2000})
            rc_thread = threading.Thread(
                target = _rc_keepalive,
                args   = (gcs, {3: 1700, 8: 2000}, rc_stop),
                daemon = True,
                name   = "rc_keepalive",
            )
            rc_thread.start()
            _log.info("Armed at t=%.1f s -- accel+circle phase begins", gcs.sim_now())

            # Run to T_END on fixed schedule.
            t_remaining = _T_END_S - gcs.sim_now()
            _log.info("Running to T_END=%.0f s (%.0f s remaining) ...",
                      _T_END_S, t_remaining)
            gcs.sim_sleep(max(t_remaining, 0.0))
            _log.info("Fixed-schedule run complete at t=%.1f s", gcs.sim_now())

        finally:
            rc_stop.set()
            stop_ev.set()
            worker.join(timeout=2.0)
            try:
                gcs.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# test_gps_fusion_tilted_hover
# ---------------------------------------------------------------------------

def _tilted_hover_worker(
    stop_event: threading.Event,
    p_eq:       np.ndarray,
    body_z:     "np.ndarray | None" = None,
) -> None:
    """
    Physics-driven sensor worker: hover at p_eq with sinusoidal gyro excitation.

    body_z : body Z-axis in NED world frame.
      None (default) = tether direction (p_eq / |p_eq|) -- 82 deg tilt.
      [0, 0, 1]      = level body (rotor disk horizontal, roll=pitch=0).

    F_net exactly cancels gravity.  A sinusoidal torque rocks the body gently
    in roll (0.5 Hz) and pitch (0.7 Hz) to give the EKF gyro excitation,
    which speeds up delAngBiasLearned convergence (~30 s vs ~60 s static).
    An orientation PD spring keeps the amplitude to ~1 deg peak.

    accel_body = R.T @ (F_hover / MASS) -- specific force (up).
    gyro_body  = R.T @ omega            -- from integrator, naturally consistent.

    With compass enabled (COMPASS_USE=1), ArduPilot knows yaw from boot.
    """
    import math
    from dynamics import RigidBodyDynamics
    from frames import build_orb_frame

    MASS   = 5.0
    I_BODY = [5.0, 5.0, 10.0]   # [Ixx, Iyy, Izz]
    DT     = 1.0 / 400.0

    # Orientation PD -- critically damped for I_zz=10:
    #   omega_n = sqrt(KP/I) = sqrt(50/10) = 2.24 rad/s
    #   KD_crit = 2 * omega_n * I = 2 * 2.24 * 10 = 44.7
    KP_ORI = 50.0
    KD_ORI = 45.0

    # Sinusoidal excitation -- 1 N*m at 0.5/0.7 Hz gives ~4 deg/s peak gyro,
    # ~0.7 deg peak tilt (well within linear regime, negligible sensor error).
    TORQUE_AMP = 1.0    # N*m
    F_ROLL_HZ  = 0.5    # body-X rocking
    F_PITCH_HZ = 0.7    # body-Y rocking (different freq to decorrelate)

    p_eq   = np.asarray(p_eq, dtype=float).copy()
    if body_z is None:
        bz = p_eq / float(np.linalg.norm(p_eq))   # tether direction (~82 deg tilt)
    else:
        bz = np.asarray(body_z, dtype=float)
        bz = bz / float(np.linalg.norm(bz))
    R0 = build_orb_frame(bz)

    # F_net cancels gravity; dynamics adds gravity internally so net = 0.
    F_HOVER = np.array([0.0, 0.0, -MASS * _GRAVITY])

    dyn = RigidBodyDynamics(
        mass   = MASS,
        I_body = I_BODY,
        pos0   = p_eq.copy(),
        vel0   = np.zeros(3),
        R0     = R0,
        omega0 = np.zeros(3),
    )

    sitl = SITLInterface(recv_port=StackConfig.SITL_JSON_PORT)
    sitl.bind()
    try:
        step = 0
        while not stop_event.is_set():
            servos = sitl.recv_servos()
            if servos is None:
                continue

            t_local = step * DT
            step   += 1

            R_cur   = dyn._R
            omega_w = dyn._omega

            # Orientation PD -- keeps body near R0 despite excitation torque.
            R_err = R_cur.T @ R0
            tr    = float(np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0))
            theta = float(np.arccos(tr))
            if theta > 1e-6:
                axis_body  = np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ]) / (2.0 * float(np.sin(theta)))
                axis_world = R_cur @ axis_body
            else:
                axis_world = np.zeros(3)
            M_restore = KP_ORI * theta * axis_world - KD_ORI * omega_w

            # Sinusoidal excitation in body frame, rotated to world frame.
            M_exc_body  = np.array([
                TORQUE_AMP * math.sin(2.0 * math.pi * F_ROLL_HZ  * t_local),
                TORQUE_AMP * math.sin(2.0 * math.pi * F_PITCH_HZ * t_local),
                0.0,
            ])
            M_net = M_restore + R_cur @ M_exc_body

            hub        = dyn.step(F_HOVER, M_net, DT)
            R_new      = hub["R"]
            accel_body = R_new.T @ (F_HOVER / MASS)
            gyro_body  = R_new.T @ hub["omega"]

            sitl.send_state(
                pos_ned    = hub["pos"],
                vel_ned    = hub["vel"],
                rpy_rad    = _rpy_from_R(R_new),
                accel_body = accel_body,
                gyro_body  = gyro_body,
            )
    finally:
        sitl.close()


def test_gps_fusion_tilted_hover(tmp_path, request):
    """
    GPS fusion for a physics-driven tilted hover with compass enabled.

    Hub sits stationary at p_eq with body_z = tether direction (~82 deg tilt).
    F_net exactly cancels gravity; zero torque; hub stays at R0.

    Compass enabled (COMPASS_ENABLE=1, COMPASS_USE=1) so EKF has yaw from
    boot -- no circular motion or EKFGSF needed.

    Expected timeline:
      t=0-10 s   EKF tilt aligns, GPS acquires first fix
      t=10-20 s  GPS 10 s quality window passes
      t=12 s     arm command sent
      t=~20 s    EKF origin set; GPS fuses
      t=40 s     run ends -- analyse log with analyse_mavlink.py

    Validates that a tilted IMU (82 deg) produces consistent sensor readings
    ArduPilot accepts without EKF divergence or GPS glitch.
    """
    ss   = json.loads(_STEADY_STATE_JSON.read_text())
    p_eq = np.array(ss["pos"], dtype=float)

    extra_boot = {
        "COMPASS_ENABLE":  1.0,
        "COMPASS_USE":     1.0,
        "EK3_SRC1_YAW":   1.0,   # 1 = compass yaw (override default GPS-vel-yaw=8)
        # Keep copter-heli.parm offsets (different per compass instance --
        # that diversity is what lets ArduPilot's compass selector pick a
        # winner; zeroing all three makes them identical and selection stalls).
        #
        # Disable EKF3 online compass calibration: a stationary body has no
        # heading rotation so the online learner resets yawAlignComplete when
        # it switches to MAG_FUSION=2(learning) at t~29 s.
        "EK3_MAG_CAL":    0.0,
    }

    with _sitl_stack(tmp_path, test_name=request.node.name,
                     extra_boot_params=extra_boot) as ctx:
        _log    = ctx.log
        stop_ev = threading.Event()

        BZ_LEVEL = np.array([0.0, 0.0, 1.0])   # level body: roll=pitch=0, compass works
        worker = threading.Thread(
            target = _tilted_hover_worker,
            args   = (stop_ev, p_eq, BZ_LEVEL),
            daemon = True,
            name   = "tilted_hover_worker",
        )
        worker.start()

        rc_stop = threading.Event()
        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=ctx.mavlink_log)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            _log.info("Tilted hover -- holding %.0f s for EKF/GPS init ...", _T_HOVER_ARM_S)
            gcs.sim_sleep(_T_HOVER_ARM_S)

            _log.info("Sending arm at t=%.1f s ...", gcs.sim_now())
            gcs.set_mode(1, timeout=5.0)
            gcs.send_rc_override({8: 1000})
            gcs.arm(force=True, timeout=8.0, rc_override={8: 1000})
            gcs.send_rc_override({3: 1700, 8: 2000})
            rc_thread = threading.Thread(
                target = _rc_keepalive,
                args   = (gcs, {3: 1700, 8: 2000}, rc_stop),
                daemon = True,
                name   = "rc_keepalive_hover",
            )
            rc_thread.start()
            _log.info("Armed at t=%.1f s", gcs.sim_now())

            t_remaining = _T_HOVER_END_S - gcs.sim_now()
            _log.info("Running to T_HOVER_END=%.0f s (%.0f s remaining) ...",
                      _T_HOVER_END_S, t_remaining)
            gcs.sim_sleep(max(t_remaining, 0.0))
            _log.info("Tilted-hover run complete at t=%.1f s", gcs.sim_now())

        finally:
            rc_stop.set()
            stop_ev.set()
            worker.join(timeout=2.0)
            try:
                gcs.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# test_gps_fusion_dual_gps
# ---------------------------------------------------------------------------

def test_gps_fusion_dual_gps(tmp_path, request):
    """
    GPS fusion using dual-GPS moving-baseline heading (no compass).

    Two GPS antennas 50 cm apart on opposite sides of the rotor disk (±25 cm
    along body X from hub centre).  ArduPilot derives heading from the NED
    vector between them -- yaw is known immediately from first GPS fix,
    no EKFGSF rotation needed.

    Antenna placement (body frame, body X = disk-plane forward):
      GPS1: SIM_GPS_POS_X  = +0.25 m   (one side of disk)
      GPS2: SIM_GPS2_POS_X = -0.25 m   (opposite side)
      Total separation = 0.50 m

    With level body (body Z = down, body X = East in NED):
      GPS1_NED = p_eq + [0, +0.25, 0]   (25 cm East of hub)
      GPS2_NED = p_eq + [0, -0.25, 0]   (25 cm West of hub)
      Vector GPS1->GPS2 in body X = -0.50 m  (GPS2_MB_OFS_X = -0.5)
      Heading derived = 90 deg (East)

    ArduPilot params:
      GPS1_TYPE=17, GPS2_TYPE=18     UBLOX F9P: 17=base (sends RTCM), 18=rover (outputs RELPOSNED)
      SIM_GPS2_HDG=1                 generate RELPOSNED on GPS2 (the rover instance)
      GPS_AUTO_CONFIG=0              required in SITL -- prevents corruption of RELPOSNED stream
      EK3_SRC1_YAW=2                 GPS yaw (from RELPOSNED dual-antenna heading)
      COMPASS_USE/USE2/USE3=0        disabled -- not needed, avoids conflicts

    Expected: GPS fuses < 40 s after arm (compare with ~62 s for
    test_gps_fusion_tilted_hover which uses compass + EKFGSF alignment).
    """
    ss   = json.loads(_STEADY_STATE_JSON.read_text())
    p_eq = np.array(ss["pos"], dtype=float)

    extra_boot = {
        # GPS_AUTO_CONFIG=0: do not try to reconfigure the UBLOX chips over serial
        # (required in SITL -- without it ArduPilot corrupts the simulated RELPOSNED stream)
        "GPS_AUTO_CONFIG":   0.0,
        "SIM_GPS2_DISABLE":  0.0,
        "SIM_GPS2_TYPE":     1.0,    # UBLOX (1=UBLOX; type 2 invalid in this build)
        "SIM_GPS2_HDG":      1.0,    # enable RELPOSNED heading on GPS2 (the rover, type=18)
        "SIM_GPS_POS_X":     0.25,   # GPS1 (base): +25 cm along body X
        "SIM_GPS_POS_Y":     0.0,
        "SIM_GPS_POS_Z":     0.0,
        "SIM_GPS2_POS_X":   -0.25,   # GPS2 (rover): -25 cm along body X (opposite side)
        "SIM_GPS2_POS_Y":    0.0,
        "SIM_GPS2_POS_Z":    0.0,
        # GPS1_TYPE=17=RTK_BASE (sends corrections), GPS2_TYPE=18=RTK_ROVER (outputs RELPOSNED)
        "GPS1_TYPE":        17.0,
        "GPS2_TYPE":        18.0,
        # Lever-arm correction for EKF (4.6+ parameter names)
        "GPS1_POS_X":        0.25,
        "GPS2_POS_X":       -0.25,
        "EK3_SRC1_YAW":      2.0,   # GPS yaw (from RELPOSNED dual-antenna heading)
        "COMPASS_USE":       0.0,
        "COMPASS_USE2":      0.0,
        "COMPASS_USE3":      0.0,
    }

    with _sitl_stack(tmp_path, test_name=request.node.name,
                     extra_boot_params=extra_boot) as ctx:
        _log    = ctx.log
        stop_ev = threading.Event()

        # Tilted body: body_z = tether direction, body_x = East proj onto disk plane
        from frames import build_orb_frame as _bof
        bz = p_eq / float(np.linalg.norm(p_eq))
        body_x_ned = _bof(bz)[:, 0]
        gps1_ned   = p_eq + 0.25 * body_x_ned
        gps2_ned   = p_eq - 0.25 * body_x_ned
        _log.info("body_z (tether)=[%.3f, %.3f, %.3f]", *bz)
        _log.info("GPS1_NED=[%.2f, %.2f, %.2f]  GPS2_NED=[%.2f, %.2f, %.2f]",
                  *gps1_ned, *gps2_ned)

        # None = tether direction body_z = normalize(p_eq) (~82 deg tilt)
        worker = threading.Thread(
            target = _tilted_hover_worker,
            args   = (stop_ev, p_eq, None),
            daemon = True,
            name   = "dual_gps_hover_worker",
        )
        worker.start()

        rc_stop = threading.Event()
        try:
            gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=ctx.mavlink_log)
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            _log.info("Holding %.0f s for EKF/GPS init ...", _T_HOVER_ARM_S)
            gcs.sim_sleep(_T_HOVER_ARM_S)

            _log.info("Sending arm at t=%.1f s ...", gcs.sim_now())
            gcs.set_mode(1, timeout=5.0)
            gcs.send_rc_override({8: 1000})
            gcs.arm(force=True, timeout=8.0, rc_override={8: 1000})
            gcs.send_rc_override({3: 1700, 8: 2000})
            rc_thread = threading.Thread(
                target = _rc_keepalive,
                args   = (gcs, {3: 1700, 8: 2000}, rc_stop),
                daemon = True,
                name   = "rc_keepalive_dual_gps",
            )
            rc_thread.start()
            _log.info("Armed at t=%.1f s -- waiting for GPS fusion ...", gcs.sim_now())

            _wait_statustext(gcs, ["is using gps"], timeout=70.0,
                             log=_log, label="gps-fuse")
            _log.info("GPS fused at t=%.1f s", gcs.sim_now())

        finally:
            rc_stop.set()
            stop_ev.set()
            worker.join(timeout=2.0)
            try:
                gcs.close()
            except Exception:
                pass
