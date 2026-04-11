"""
test_stationary_gps.py — GPS position fusion smoke test.

Sends a fixed state (position=[0,0,0] NED, velocity=[0,0,0], attitude flat)
to ArduPilot SITL forever.  No physics, no dynamics — the hub just sits at
home position.

Purpose: isolate whether EKF3 GPS horizontal position fusion works at all in
this SITL setup, independent of trajectory complexity or sensor mode.

Asserts:
  - LOCAL_POSITION_NED received within 60 s (EKF fused GPS position)
"""
import json
import logging
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest

_SIM_DIR   = Path(__file__).resolve().parents[2]
_STACK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_STACK_DIR))

from test_stack_integration import (
    STACK_ENV_FLAG,
    SIM_VEHICLE_ENV,
    ARDUPILOT_ENV,
    _launch_sitl,
    _resolve_sim_vehicle,
    _terminate_process,
    _kill_by_port,
)
from conftest import StackConfig, _configure_logging, copy_logs_to_dir
from stack_utils import make_test_log_dir
from pymavlink import mavutil as _mavutil
from gcs import RawesGCS

_FUSE_TIMEOUT  = 45.0   # s — delAngBiasLearned converges ~37 s (passive cross-cov
                        #     accumulation) + GPS detect 10 s + 5 s margin → 42 s typical
_TOTAL_TIMEOUT = 60.0   # s — hard limit (param fetch ~2 s + fuse wait 45 s + margin)

# ---------------------------------------------------------------------------
# Stub mediator script (inline, run as a subprocess)
# ---------------------------------------------------------------------------

_STUB_MEDIATOR_SRC = textwrap.dedent("""\
    import json, socket, struct, time, sys, logging

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    log = logging.getLogger("stub_mediator")

    RECV_PORT   = 9002
    DT          = 1.0 / 400.0

    SERVO_FMT16 = "<HHI16H"
    SERVO_SZ16  = struct.calcsize(SERVO_FMT16)
    SERVO_MAG16 = 18458
    SERVO_FMT32 = "<HHI32H"
    SERVO_SZ32  = struct.calcsize(SERVO_FMT32)
    SERVO_MAG32 = 29569

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", RECV_PORT))
    sock.settimeout(DT)
    log.info("Stub mediator bound to UDP port %d", RECV_PORT)

    sitl_addr = None
    t0 = time.time()
    n_sent = 0

    import math as _math

    # Yaw-only rotation: constant yaw spin, attitude stays flat.
    #
    # Why not roll/pitch oscillation:
    #   Any tilt projects gravity horizontally in body frame.  In AID_NONE mode
    #   the EKF has no GPS to correct drift, so it dead-reckons the tilt as
    #   horizontal movement.  When GPS fusion finally starts the EKF position has
    #   drifted far from the GPS reading → huge innovation → EKF unhealthy.
    #
    # Why yaw works:
    #   Yaw-only with flat attitude keeps accel=[0,0,-9.81] in body frame.
    #   Zero horizontal gravity → no horizontal dead-reckoning drift during
    #   AID_NONE.  Position estimate stays near [0,0,0] the whole time.
    #
    # Why it still converges delAngBiasLearned:
    #   P[12][12] (yaw bias) converges from compass heading updates.
    #   P[10][10] and P[11][11] (roll/pitch bias) converge MORE slowly through
    #   accumulated cross-covariance from EKF prediction steps + tilt corrections
    #   from the accelerometer.  In practice this takes ~30-35 s with the default
    #   EK3_GBIAS_P_NSE=0.001.  Once all three converge, GPS fusion starts
    #   immediately with a near-zero position innovation → stable lock.
    YAW_RATE = _math.radians(5.0)   # rad/s — slow constant yaw spin

    while True:
        try:
            data, addr = sock.recvfrom(4096)
            sitl_addr = addr
        except socket.timeout:
            pass

        if sitl_addr is not None:
            ts  = time.time() - t0
            yaw = (YAW_RATE * ts) % (2.0 * _math.pi)

            msg = {
                "timestamp": ts,
                "imu": {
                    "gyro":       [0.0, 0.0, YAW_RATE],
                    "accel_body": [0.0, 0.0, -9.81],
                },
                "position": [0.0, 0.0, 0.0],
                "attitude": [0.0, 0.0, yaw],
                "velocity": [0.0, 0.0, 0.0],
                "rpm":      [0.0, 0.0],
            }
            payload = (json.dumps(msg) + "\\n").encode()
            try:
                sock.sendto(payload, sitl_addr)
                n_sent += 1
                if n_sent % 2000 == 0:
                    log.info("Sent %d state packets  t=%.1fs", n_sent, ts)
            except OSError as exc:
                log.warning("Send failed: %s", exc)
""")


def _launch_stub_mediator(log_path: Path) -> subprocess.Popen:
    """Launch the stub mediator as a subprocess."""
    return subprocess.Popen(
        [sys.executable, "-c", _STUB_MEDIATOR_SRC],
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_stationary_gps_fusion(tmp_path, request):
    """
    Hub sits at NED=[0,0,0] forever.  EKF3 must fuse GPS horizontal position
    within 60 s, demonstrated by receiving LOCAL_POSITION_NED.

    This is the minimal possible SITL test — no physics, no tether, no sensor
    mode complexity.  If this fails, the issue is in ArduPilot SITL/EKF3
    configuration, not in our sensor data.
    """
    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run stack integration tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} to locate sim_vehicle.py")

    pytest.importorskip("pymavlink")

    # Hard test timeout — raises SIGALRM after _TOTAL_TIMEOUT seconds.
    # Prevents the test hanging forever if param fetch or SITL startup stalls.
    def _on_timeout(signum, frame):
        raise TimeoutError(f"Test exceeded {_TOTAL_TIMEOUT:.0f} s hard limit")
    signal.signal(signal.SIGALRM, _on_timeout)
    signal.alarm(int(_TOTAL_TIMEOUT))

    # Check ports free before launch
    StackConfig.verify()

    sim_dir      = _SIM_DIR
    test_log_dir = make_test_log_dir(sim_dir, request.node.name)

    stub_log = tmp_path / "stub_mediator.log"
    sitl_log = tmp_path / "sitl.log"
    gcs_log  = tmp_path / "gcs.log"

    _configure_logging(gcs_log)
    log = logging.getLogger("test_stationary_gps")

    log.info("Launching stub mediator + SITL ...")
    stub_proc = _launch_stub_mediator(stub_log)
    sitl_proc = _launch_sitl(sim_vehicle, sitl_log)

    def _procs_alive():
        for name, proc, lp in [
            ("stub_mediator", stub_proc, stub_log),
            ("SITL",          sitl_proc, sitl_log),
        ]:
            if proc.poll() is not None:
                txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{txt[-2000:]}")

    gcs = RawesGCS(address=StackConfig.GCS_ADDRESS)

    try:
        # ── Connect ────────────────────────────────────────────────────────
        log.info("Connecting GCS ...")
        gcs.connect(timeout=30.0)
        gcs.start_heartbeat(rate_hz=1.0)
        _procs_alive()

        # Request all telemetry streams
        gcs._mav.mav.request_data_stream_send(
            gcs._target_system, gcs._target_component,
            _mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1,
        )
        gcs._mav.mav.request_data_stream_send(
            gcs._target_system, gcs._target_component,
            _mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1,
        )

        # ── Param subsystem + full EK3 dump ───────────────────────────────
        log.info("Fetching all params (needed to find EK3_* names) ...")
        gcs._mav.mav.param_request_list_send(
            gcs._target_system, gcs._target_component,
        )
        all_params: dict[str, float] = {}
        deadline = time.monotonic() + 30.0
        expected_count = None
        while time.monotonic() < deadline:
            _procs_alive()
            msg = gcs._mav.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=1.0,
            )
            if msg is None:
                continue
            pid   = msg.param_id.rstrip("\x00")
            pval  = msg.param_value
            all_params[pid] = pval
            if expected_count is None:
                expected_count = msg.param_count
            if len(all_params) >= expected_count:
                break

        if not all_params:
            pytest.fail("Param subsystem never responded within 30 s")

        log.info("Received %d params (expected %s)", len(all_params), expected_count)

        # Log vehicle identity
        fw_ver = all_params.get("SYSID_THISMAV", "?")
        log.info("Vehicle SYSID_THISMAV=%s", fw_ver)

        # Dump all EK3 params
        ek3_params = {k: v for k, v in sorted(all_params.items()) if k.startswith("EK3")}
        log.info("── All EK3 params (%d) ──", len(ek3_params))
        for k, v in ek3_params.items():
            log.info("  %-25s = %g", k, v)

        # Dump GPS params
        gps_keys = [k for k in sorted(all_params) if "GPS" in k or "COMPASS" in k]
        log.info("── GPS/COMPASS params (%d) ──", len(gps_keys))
        for k in gps_keys:
            log.info("  %-25s = %g", k, all_params[k])

        # ── Set params and verify ──────────────────────────────────────────
        # Each entry: (name, value, critical)
        # critical=True → abort immediately if param doesn't exist or can't be set.
        #
        # EK3_SRC1_YAW=2 means "GPS velocity heading" — with zero or low velocity
        # the EKF can never derive yaw and stays in CONST_POS_MODE forever.
        # Set to 1 (compass) so SITL's simulated magnetometer provides yaw.
        # EK3_MAG_CAL=3 ("Always") requires 3D movement for calibration; set to 0
        # ("Never") to use raw compass readings immediately in SITL.
        _params_to_set = [
            ("ARMING_SKIPCHK",   0xFFFF, True),
            ("FS_EKF_ACTION",    0,      True),
            ("COMPASS_USE",      1,      True),
            ("COMPASS_ENABLE",   1,      True),
            ("EK3_GPS_CHECK",    0,      True),
            ("EK3_SRC1_YAW",     1,      True),    # 1=compass (not GPS velocity heading)
            ("EK3_MAG_CAL",      0,      True),    # 0=Never (use raw compass, no cal needed)
            # These may not exist in all builds — log but don't abort:
            ("EK3_GPS_CTRL",     3,      False),   # fuse GPS position + velocity (newer builds)
            ("EK3_SRC1_POSXY",   3,      False),   # GPS horizontal position source
            ("EK3_SRC1_VELXY",   3,      False),   # GPS horizontal velocity source
        ]

        log.info("── Setting params ──")
        missing_critical = []
        for pname, pval, critical in _params_to_set:
            if pname not in all_params:
                status = "DOES NOT EXIST"
                if critical:
                    missing_critical.append(pname)
            else:
                ok = gcs.set_param(pname, pval, timeout=5.0)
                # Read back to confirm
                read_back = all_params.get(pname)
                status = f"ACK={ok}  (was {read_back})"
                if critical and not ok:
                    missing_critical.append(pname)
            log.info("  %-25s = %-8s  [critical=%s]  %s",
                     pname, pval, critical, status)

        if missing_critical:
            pytest.fail(
                f"Critical params could not be set: {missing_critical}\n"
                f"Available EK3 params: {list(ek3_params.keys())}"
            )

        # ── Wait for LOCAL_POSITION_NED ────────────────────────────────────
        log.info("Waiting up to %.0f s for LOCAL_POSITION_NED ...", _FUSE_TIMEOUT)
        t_start  = time.monotonic()
        deadline = t_start + _FUSE_TIMEOUT
        statustext_seen = []

        while time.monotonic() < deadline:
            _procs_alive()
            msg = gcs._mav.recv_match(
                type=["LOCAL_POSITION_NED", "EKF_STATUS_REPORT", "STATUSTEXT", "ATTITUDE"],
                blocking=True, timeout=0.5,
            )
            if msg is None:
                continue

            mt = msg.get_type()
            t_rel = time.monotonic() - t_start

            if mt == "LOCAL_POSITION_NED":
                log.info("GPS FUSED at t=%.1f s: LOCAL_POSITION_NED  N=%.3f E=%.3f D=%.3f",
                         t_rel, msg.x, msg.y, msg.z)
                return   # PASS

            elif mt == "EKF_STATUS_REPORT":
                flags = msg.flags
                labels = []
                if flags & 0x0001: labels.append("att")
                if flags & 0x0002: labels.append("horiz_vel")
                if flags & 0x0004: labels.append("vert_vel")
                if flags & 0x0008: labels.append("horiz_pos_rel")
                if flags & 0x0010: labels.append("horiz_pos_abs")
                if flags & 0x0020: labels.append("vert_pos_abs")
                if flags & 0x0080: labels.append("CONST_POS_MODE")
                if flags & 0x0400: labels.append("pred_horiz_ok")
                log.info("EKF t=%.1fs  0x%04x  [%s]  hv=%.3f hp=%.3f",
                         t_rel, flags, " ".join(labels) or "none",
                         getattr(msg, "velocity_variance", float("nan")),
                         getattr(msg, "pos_horiz_variance", float("nan")))

            elif mt == "STATUSTEXT":
                text = msg.text.rstrip("\x00").strip()
                log.info("STATUSTEXT t=%.1fs: %s", t_rel, text)
                statustext_seen.append(text)

            elif mt == "ATTITUDE":
                r = math.degrees(msg.roll)
                p = math.degrees(msg.pitch)
                y = math.degrees(msg.yaw)
                log.info("ATTITUDE t=%.1fs  rpy=(%.1f° %.1f° %.1f°)", t_rel, r, p, y)

        pytest.fail(
            f"LOCAL_POSITION_NED never received within {_FUSE_TIMEOUT:.0f} s.\n"
            f"GPS horizontal position did not fuse.\n"
            f"STATUSTEXT seen: {statustext_seen}"
        )

    finally:
        signal.alarm(0)   # cancel hard timeout
        gcs.close()
        _terminate_process(sitl_proc)
        _terminate_process(stub_proc)
        _kill_by_port(StackConfig.SITL_GCS_PORT)
        copy_logs_to_dir(test_log_dir, {
            "stub_mediator.log": stub_log,
            "sitl.log":          sitl_log,
            "gcs.log":           gcs_log,
        })
        _ardupilot_log = Path("/tmp/ArduCopter.log")
        if _ardupilot_log.exists():
            import shutil
            shutil.copy2(_ardupilot_log, test_log_dir / "arducopter.log")
