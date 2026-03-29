import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


STACK_ENV_FLAG = "RAWES_RUN_STACK_INTEGRATION"
ARDUPILOT_ENV = "RAWES_ARDUPILOT_PATH"
SIM_VEHICLE_ENV = "RAWES_SIM_VEHICLE"


# ─────────────────────────────────────────────────────────────────────────────
# Discovery helpers
# ─────────────────────────────────────────────────────────────────────────────

def _resolve_sim_vehicle() -> Path | None:
    explicit = os.environ.get(SIM_VEHICLE_ENV)
    if explicit:
        candidate = Path(explicit)
        return candidate if candidate.is_file() else None

    ardupilot_root = os.environ.get(ARDUPILOT_ENV)
    if not ardupilot_root:
        return None

    candidate = Path(ardupilot_root) / "Tools" / "autotest" / "sim_vehicle.py"
    return candidate if candidate.is_file() else None


# ─────────────────────────────────────────────────────────────────────────────
# Process launch helpers (shared by all stack tests)
# ─────────────────────────────────────────────────────────────────────────────

def _launch_sitl(sim_vehicle: Path, log_path: Path) -> subprocess.Popen:
    return subprocess.Popen(
        [
            sys.executable,
            str(sim_vehicle),
            "--vehicle", "ArduCopter",
            "--frame", "heli",
            "--custom-location=51.5074,-0.1278,50,0",
            "--model", "JSON",
            "--sim-address", "127.0.0.1",
            "--no-rebuild",
            "--no-mavproxy",
        ],
        cwd=str(sim_vehicle.parent.parent.parent),
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _launch_mediator(
    sim_dir: Path,
    repo_root: Path,
    log_path: Path,
    telemetry_log_path: str | None = None,
    tether_rest_length: float | None = None,
    initial_state: dict | None = None,
    startup_damp_seconds: float | None = None,
    lock_orientation: bool = False,
    sensor_mode: str = "tether_relative",
    run_id: int | None = None,
    base_k_ang: float | None = None,
) -> subprocess.Popen:
    cmd = [
        sys.executable,
        str(sim_dir / "mediator.py"),
        "--sitl-recv-port", "9002",
        "--sitl-send-port", "9003",
        "--log-level", "INFO",
    ]
    if telemetry_log_path is not None:
        cmd += ["--telemetry-log", telemetry_log_path]
    if tether_rest_length is not None:
        cmd += ["--tether-rest-length", str(tether_rest_length)]
    if startup_damp_seconds is not None:
        cmd += ["--startup-damp-seconds", str(startup_damp_seconds)]
    if base_k_ang is not None:
        cmd += ["--base-k-ang", str(base_k_ang)]
    if lock_orientation:
        cmd += ["--lock-orientation"]
    if sensor_mode != "tether_relative":
        cmd += ["--sensor-mode", sensor_mode]
    if run_id is not None:
        cmd += ["--run-id", str(run_id)]
    if initial_state is not None:
        # Use = form to avoid argparse treating negative floats as flags
        cmd.append(f"--pos0={','.join(str(v) for v in initial_state['pos'])}")
        cmd.append(f"--vel0={','.join(str(v) for v in initial_state['vel'])}")
        cmd.append(f"--body-z={','.join(str(v) for v in initial_state['body_z'])}")
        cmd.append(f"--omega-spin={initial_state['omega_spin']}")
        if tether_rest_length is None and "rest_length" in initial_state:
            cmd.append(f"--tether-rest-length={initial_state['rest_length']}")
    return subprocess.Popen(
        cmd,
        cwd=str(repo_root),
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _terminate_process(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    import signal
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=10.0)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    proc.wait(timeout=5.0)


# ─────────────────────────────────────────────────────────────────────────────
# Ground station drivers
# ─────────────────────────────────────────────────────────────────────────────

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

def test_stack_integration_smoke():
    """
    Fast smoke test: SITL ↔ mediator (internal RK4 dynamics).

    Verifies:
    - Both processes start without crashing
    - MAVLink heartbeat received (proves SITL accepted mediator state)
    - Mediator loop ran for ≥ 10 s (log shows per-second update lines)
    - No CRITICAL errors in mediator log
    """
    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run stack integration tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(
            f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} so the stack integration test can launch SITL"
        )

    pytest.importorskip("pymavlink")

    repo_root = Path(__file__).resolve().parents[3]
    sim_dir = repo_root / "simulation"

    with TemporaryDirectory(prefix="rawes_smoke_") as tmpdir:
        temp_path = Path(tmpdir)
        mediator_log = temp_path / "mediator.log"
        sitl_log = temp_path / "sitl.log"

        mediator_proc = _launch_mediator(sim_dir, repo_root, mediator_log)
        sitl_proc = _launch_sitl(sim_vehicle, sitl_log)

        def _assert_procs_alive():
            for name, proc, log in [("SITL", sitl_proc, sitl_log), ("mediator", mediator_proc, mediator_log)]:
                if proc.poll() is not None:
                    log_text = log.read_text(encoding="utf-8", errors="replace") if log.exists() else "(no log)"
                    pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{log_text[-3000:]}")

        def _dump_logs():
            for name, log in [("SITL", sitl_log), ("mediator", mediator_log)]:
                text = log.read_text(encoding="utf-8", errors="replace") if log.exists() else "(no log)"
                print(f"\n=== {name} log (last 3000 chars) ===\n{text[-3000:]}")

        gcs = FakeGroundStation()
        try:
            gcs.connect(timeout=30.0, poll=_assert_procs_alive)
            _assert_procs_alive()
            heartbeat = gcs.wait_heartbeat(timeout=30.0)
            assert heartbeat is not None, "No MAVLink heartbeat received"

            # Run for 15 s so the mediator loop accumulates ≥ 10 per-second log lines
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                _assert_procs_alive()
                time.sleep(1.0)

            log_text = mediator_log.read_text(encoding="utf-8", errors="replace") if mediator_log.exists() else ""
            loop_lines = [l for l in log_text.splitlines() if "pos_ENU" in l]
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
        finally:
            gcs.close()
            _terminate_process(sitl_proc)
            _terminate_process(mediator_proc)


# ─────────────────────────────────────────────────────────────────────────────
# Full stack integration test
# ─────────────────────────────────────────────────────────────────────────────

def test_full_stack_integration():
    """
    Full stack integration test: SITL ↔ mediator (internal RK4 dynamics).

    Waits for EKF lock, then asserts:

    MAVLink data flow
      At least 10 LOCAL_POSITION_NED messages in 10 s (≥ 1 Hz average).

    Position validity
      All received position values are finite.

    No mediator CRITICAL errors
      The mediator log must not contain any CRITICAL-level entries.

    MAVLink bidirectionality
      PARAM_REQUEST_LIST must elicit at least one PARAM_VALUE reply.
    """
    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run stack integration tests")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(
            f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} so the stack integration test can launch SITL"
        )

    pytest.importorskip("pymavlink")

    repo_root = Path(__file__).resolve().parents[3]
    sim_dir = repo_root / "simulation"

    sys.path.insert(0, str(sim_dir))
    from gcs import RawesGCS

    with TemporaryDirectory(prefix="rawes_full_") as tmpdir:
        temp_path = Path(tmpdir)
        mediator_log = temp_path / "mediator.log"
        sitl_log = temp_path / "sitl.log"

        mediator_proc = _launch_mediator(sim_dir, repo_root, mediator_log)
        sitl_proc = _launch_sitl(sim_vehicle, sitl_log)

        def _procs_alive():
            for name, proc, log in [
                ("SITL",     sitl_proc,     sitl_log),
                ("mediator", mediator_proc, mediator_log),
            ]:
                if proc.poll() is not None:
                    log_text = log.read_text(encoding="utf-8", errors="replace") if log.exists() else "(no log)"
                    pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{log_text[-3000:]}")

        def _dump_logs():
            for name, log in [("mediator", mediator_log), ("SITL", sitl_log)]:
                text = log.read_text(encoding="utf-8", errors="replace") if log.exists() else "(no log)"
                print(f"\n=== {name} log ===\n{text[-2000:]}")

        from pymavlink import mavutil as _mavutil
        gcs = RawesGCS(address="tcp:127.0.0.1:5760")
        try:
            # ── Phase 1: connect ──────────────────────────────────────────────
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat()
            _procs_alive()

            # Wait for param subsystem (confirms SITL has finished booting)
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                gcs._mav.mav.param_request_read_send(
                    gcs._mav.target_system, gcs._mav.target_component,
                    b"SYSID_THISMAV", -1,
                )
                if gcs._mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0):
                    break

            # Disable compass so EKF can converge on GPS alone (< 5 s).
            # Without a real magnetometer the compass blocks yaw lock, which
            # blocks position — same technique as the guided flight test.
            gcs.set_param("COMPASS_USE", 0, timeout=5.0)

            # Request position stream
            gcs.request_stream(_mavutil.mavlink.MAV_DATA_STREAM_POSITION, rate_hz=4)
            gcs.request_stream(_mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz=2)

            # Wait for EKF position lock (POS_HORIZ_REL | POS_HORIZ_ABS)
            _EKF_POS = (
                _mavutil.mavlink.EKF_POS_HORIZ_REL
                | _mavutil.mavlink.EKF_POS_HORIZ_ABS
            )
            deadline = time.monotonic() + 60.0
            ekf_locked = False
            while time.monotonic() < deadline:
                _procs_alive()
                msg = gcs._mav.recv_match(
                    type=["LOCAL_POSITION_NED", "EKF_STATUS_REPORT"],
                    blocking=True, timeout=1.0,
                )
                if msg is None:
                    continue
                if msg.get_type() == "LOCAL_POSITION_NED":
                    ekf_locked = True
                    break
                if msg.get_type() == "EKF_STATUS_REPORT" and (msg.flags & _EKF_POS):
                    ekf_locked = True
                    break
            assert ekf_locked, "EKF position lock not achieved within 60 s"
            _procs_alive()

            # ── Phase 2: collect 10 s of position data ───────────────────────
            pos_msgs = []
            t_start = time.monotonic()
            while time.monotonic() - t_start < 10.0:
                _procs_alive()
                msg = gcs._mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
                if msg is not None:
                    pos_msgs.append(msg)

            # ── Phase 3: data flow ────────────────────────────────────────────
            assert len(pos_msgs) >= 10, (
                f"Too few LOCAL_POSITION_NED messages: {len(pos_msgs)} in 10 s "
                f"(need ≥ 10)"
            )

            # ── Phase 4: position validity ────────────────────────────────────
            for msg in pos_msgs:
                assert np.isfinite([msg.x, msg.y, msg.z]).all(), (
                    f"Non-finite position: ({msg.x}, {msg.y}, {msg.z})"
                )

            # ── Phase 5: no CRITICAL errors ───────────────────────────────────
            if mediator_log.exists():
                log_text = mediator_log.read_text(encoding="utf-8", errors="replace")
                critical_lines = [l for l in log_text.splitlines() if "CRITICAL" in l]
                assert not critical_lines, (
                    "CRITICAL-level errors in mediator log:\n" + "\n".join(critical_lines[:10])
                )

            # ── Phase 6: MAVLink bidirectionality ─────────────────────────────
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

        except Exception:
            _dump_logs()
            raise
        finally:
            gcs.stop_heartbeat()
            gcs.close()
            _terminate_process(sitl_proc)
            _terminate_process(mediator_proc)
