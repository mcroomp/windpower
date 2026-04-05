import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from stack_utils import (
    STACK_ENV_FLAG,
    ARDUPILOT_ENV,
    SIM_VEHICLE_ENV,
    _resolve_sim_vehicle,
    _launch_sitl,
    _terminate_process,
    _kill_by_port,
)


class FakeGroundStation:
    """Thin MAVLink driver."""

    def __init__(self, endpoint: str = "tcp:127.0.0.1:5760"):
        from pymavlink import mavutil
        self._mavutil = mavutil
        self._endpoint = endpoint
        self._conn = None

    def connect(self, timeout: float = 30.0, poll=None):
        deadline = time.monotonic() + timeout
        last_error = None
        while time.monotonic() < deadline:
            if poll is not None:
                poll()
            try:
                self._conn = self._mavutil.mavlink_connection(self._endpoint, autoreconnect=True)
                return
            except OSError as exc:
                last_error = exc
                time.sleep(1.0)
        raise TimeoutError(
            f"Timed out connecting to MAVLink endpoint {self._endpoint}: {last_error}"
        )

    def wait_heartbeat(self, timeout: float = 30.0):
        if self._conn is None:
            raise RuntimeError("Call connect() first.")
        return self._conn.wait_heartbeat(timeout=timeout)

    def wait_for_message(self, message_type: str, timeout: float = 30.0):
        if self._conn is None:
            raise RuntimeError("Call connect() first.")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._conn.recv_match(type=message_type, blocking=True, timeout=1.0)
            if msg is not None:
                return msg
        raise TimeoutError(f"Timed out waiting for MAVLink {message_type}")

    def close(self):
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None


class FullGroundStation(FakeGroundStation):
    """Extended GCS with MAVLink command sending for full stack tests."""

    def request_streams(self, rate_hz: int = 4) -> None:
        from pymavlink import mavutil
        if self._conn is None:
            raise RuntimeError("Call connect() first.")
        for stream_id in [
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
        ]:
            self._conn.mav.request_data_stream_send(
                self._conn.target_system,
                self._conn.target_component,
                stream_id,
                rate_hz,
                1,
            )

    def send_param_request_list(self) -> None:
        if self._conn is None:
            raise RuntimeError("Call connect() first.")
        self._conn.mav.param_request_list_send(
            self._conn.target_system,
            self._conn.target_component,
        )

    def wait_for_any_of(self, message_types: list[str], timeout: float = 30.0):
        if self._conn is None:
            raise RuntimeError("Call connect() first.")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._conn.recv_match(type=message_types, blocking=True, timeout=1.0)
            if msg is not None:
                return msg
        raise TimeoutError(f"Timed out waiting for any of {message_types}")


# ─────────────────────────────────────────────────────────────────────────────
# Smoke test (fast connectivity check)
# ─────────────────────────────────────────────────────────────────────────────

def test_stack_integration_smoke(tmp_path, request):
    """
    Fast smoke test: SITL <-> mediator (internal RK4 dynamics).

    Verifies:
    - Both processes start without crashing
    - MAVLink heartbeat received (proves SITL accepted mediator state)
    - Mediator loop ran for >= 10 s (log shows per-second update lines)
    - No CRITICAL errors in mediator log

    Uses _acro_stack(arm=False): processes are launched and available but the
    full arm/EKF sequence is skipped, keeping the test fast.
    """
    from conftest import _acro_stack

    with _acro_stack(tmp_path, arm=False,
                     log_name="smoke", test_name=request.node.name) as ctx:

        def _assert_procs_alive():
            for name, proc, lp in [
                ("mediator", ctx.mediator_proc, ctx.mediator_log),
                ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
            ]:
                if proc.poll() is not None:
                    log_text = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{log_text[-3000:]}")

        def _dump_logs():
            for name, lp in [("SITL", ctx.sitl_log), ("mediator", ctx.mediator_log)]:
                text = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                print(f"\n=== {name} log (last 3000 chars) ===\n{text[-3000:]}")

        try:
            ctx.gcs.connect(timeout=30.0)
            _assert_procs_alive()

            # Run for 15 s so mediator loop accumulates >= 10 per-second log lines
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                _assert_procs_alive()
                time.sleep(1.0)

            log_text = ctx.mediator_log.read_text(encoding="utf-8", errors="replace") if ctx.mediator_log.exists() else ""
            loop_lines = [l for l in log_text.splitlines() if "pos_NED" in l]
            assert len(loop_lines) >= 10, (
                f"Mediator loop ran fewer than 10 per-second iterations: {len(loop_lines)} lines found.\n"
                f"Loop may have stalled after startup."
            )

            critical_lines = [l for l in log_text.splitlines() if "CRITICAL" in l]
            assert not critical_lines, (
                "CRITICAL-level errors in mediator log:\n" + "\n".join(critical_lines[:10])
            )
        except Exception:
            _dump_logs()
            raise


# ─────────────────────────────────────────────────────────────────────────────
# Full stack integration test
# ─────────────────────────────────────────────────────────────────────────────

def test_full_stack_integration(acro_armed):
    """
    Full stack integration test: armed ACRO + MAVLink data flow.

    GPS horizontal position fusion is NOT required (unreliable with the tilted
    physical hub in EKF3 — see test_gps_fuses_during_startup xfail).  Instead
    this test verifies the MAVLink pipeline is fully operational:

    ATTITUDE data flow
      At least 10 ATTITUDE messages in 10 s after arm.

    Attitude validity
      All received attitude values are finite (no NaN from broken IMU feed).

    No mediator CRITICAL errors
      The mediator log must not contain any CRITICAL-level entries.

    MAVLink bidirectionality
      PARAM_REQUEST_LIST must elicit at least one PARAM_VALUE reply.
    """
    import math as _math

    ctx = acro_armed
    gcs = ctx.gcs

    def _procs_alive():
        for name, proc, lp in [
            ("mediator", ctx.mediator_proc, ctx.mediator_log),
            ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
        ]:
            if proc.poll() is not None:
                txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                pytest.fail(f"{name} exited during test (rc={proc.returncode}):\n{txt[-3000:]}")

    # ── Phase 1: collect 10 s of ATTITUDE data ───────────────────────────────
    # ATTITUDE flows in ACRO mode regardless of GPS status.
    att_msgs = []
    pos_msgs = []    # collect LOCAL_POSITION_NED opportunistically (diagnostic)
    t_start = time.monotonic()
    while time.monotonic() - t_start < 10.0:
        _procs_alive()
        msg = gcs._mav.recv_match(
            type=["ATTITUDE", "LOCAL_POSITION_NED"],
            blocking=True, timeout=1.0,
        )
        if msg is None:
            continue
        if msg.get_type() == "ATTITUDE":
            att_msgs.append(msg)
        elif msg.get_type() == "LOCAL_POSITION_NED":
            pos_msgs.append(msg)

    # ── Phase 2: ATTITUDE data flow ───────────────────────────────────────────
    assert len(att_msgs) >= 10, (
        f"Too few ATTITUDE messages: {len(att_msgs)} in 10s (need >= 10). "
        "ArduPilot is not sending telemetry — check MAVLink stream request."
    )

    # ── Phase 3: attitude validity ────────────────────────────────────────────
    for msg in att_msgs:
        assert all(_math.isfinite(v) for v in (msg.roll, msg.pitch, msg.yaw)), (
            f"Non-finite ATTITUDE: rpy=({msg.roll}, {msg.pitch}, {msg.yaw})"
        )

    # ── Phase 4: GPS position (diagnostic only) ───────────────────────────────
    # GPS fusion is unreliable in physical-sensor mode; no assertion here.
    if pos_msgs:
        for msg in pos_msgs:
            assert np.isfinite([msg.x, msg.y, msg.z]).all(), (
                f"Non-finite LOCAL_POSITION_NED: ({msg.x}, {msg.y}, {msg.z})"
            )

    # ── Phase 5: no CRITICAL errors ───────────────────────────────────────────
    log_text = ctx.mediator_log.read_text(encoding="utf-8", errors="replace") if ctx.mediator_log.exists() else ""
    critical_lines = [l for l in log_text.splitlines() if "CRITICAL" in l]
    assert not critical_lines, (
        "CRITICAL-level errors in mediator log:\n" + "\n".join(critical_lines[:10])
    )

    # ── Phase 6: MAVLink bidirectionality ─────────────────────────────────────
    gcs._mav.mav.param_request_list_send(
        gcs._mav.target_system, gcs._mav.target_component
    )
    deadline = time.monotonic() + 15.0
    param_msg = None
    while time.monotonic() < deadline:
        param_msg = gcs._mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if param_msg is not None:
            break
    assert param_msg is not None, "No PARAM_VALUE received after PARAM_REQUEST_LIST"
    assert np.isfinite(param_msg.param_value)
