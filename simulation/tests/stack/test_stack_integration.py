import os
import re
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mbdyn_interface import _FORCE_FMT, _FORCE_SIZE, _STATE_FMT, _recv_exactly


STACK_ENV_FLAG = "RAWES_RUN_STACK_INTEGRATION"
ARDUPILOT_ENV = "RAWES_ARDUPILOT_PATH"
SIM_VEHICLE_ENV = "RAWES_SIM_VEHICLE"

# Default MBDyn socket paths (hardcoded in rawes.set)
_MBDYN_FORCE_SOCK = "/tmp/rawes_forces.sock"
_MBDYN_STATE_SOCK = "/tmp/rawes_state.sock"


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


def _find_mbdyn() -> Path | None:
    path = shutil.which("mbdyn")
    return Path(path) if path else None


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
    force_path: str,
    state_path: str,
    log_path: Path,
    telemetry_log_path: str | None = None,
) -> subprocess.Popen:
    cmd = [
        sys.executable,
        str(sim_dir / "mediator.py"),
        "--mbdyn-force-sock", force_path,
        "--mbdyn-state-sock", state_path,
        "--sitl-recv-port", "9002",
        "--sitl-send-port", "9003",
        "--log-level", "INFO",
    ]
    if telemetry_log_path is not None:
        cmd += ["--telemetry-log", telemetry_log_path]
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
# Physics plant simulator (in-process MBDyn substitute)
# ─────────────────────────────────────────────────────────────────────────────

def _pack_state(pos, vel, R, omega):
    # MBDyn output order: pos(3), R(9), vel(3), omega(3)
    values = np.concatenate(
        [
            np.asarray(pos, dtype=np.float64),
            np.asarray(R, dtype=np.float64).reshape(9),
            np.asarray(vel, dtype=np.float64),
            np.asarray(omega, dtype=np.float64),
        ]
    )
    return struct.pack(_STATE_FMT, *values)


class FakePlantServer:
    """Minimal MBDyn stand-in for smoke tests (unchanged interface)."""

    def __init__(self, force_path: str, state_path: str, dt: float = 1.0 / 400.0):
        self._force_path = force_path
        self._state_path = state_path
        self._dt = dt
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self.force_packets: list[np.ndarray] = []
        self.pos = np.array([0.0, 0.0, 50.0], dtype=np.float64)
        self.vel = np.zeros(3, dtype=np.float64)
        self.R = np.eye(3, dtype=np.float64)
        self.omega = np.array([0.0, 0.0, 28.0], dtype=np.float64)
        self.mass = 5.0

    def start(self):
        self._thread = threading.Thread(target=self._serve, name="fake-plant", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _serve(self):
        for path in (self._force_path, self._state_path):
            try:
                os.unlink(path)
            except FileNotFoundError:
                pass

        force_server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        state_server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            force_server.bind(self._force_path)
            state_server.bind(self._state_path)
            force_server.listen(1)
            state_server.listen(1)
            force_server.settimeout(0.5)
            state_server.settimeout(0.5)

            force_conn = state_conn = None
            while not self._stop_event.is_set() and (force_conn is None or state_conn is None):
                if force_conn is None:
                    try:
                        force_conn, _ = force_server.accept()
                    except socket.timeout:
                        pass
                if state_conn is None:
                    try:
                        state_conn, _ = state_server.accept()
                    except socket.timeout:
                        pass

            if force_conn is None or state_conn is None:
                return

            force_conn.settimeout(0.5)
            while not self._stop_event.is_set():
                try:
                    raw = _recv_exactly(force_conn, _FORCE_SIZE)
                except (ConnectionResetError, OSError, socket.timeout):
                    break

                forces = np.array(struct.unpack(_FORCE_FMT, raw), dtype=np.float64)
                self.force_packets.append(forces)

                accel = (forces[:3] / self.mass) + np.array([0.0, 0.0, -9.81])
                self.vel = self.vel + accel * self._dt
                self.pos = self.pos + self.vel * self._dt

                state_conn.sendall(_pack_state(self.pos, self.vel, self.R, self.omega))
        finally:
            for s in filter(None, [
                locals().get("force_conn"), locals().get("state_conn"),
                force_server, state_server,
            ]):
                try:
                    s.close()
                except OSError:
                    pass
            for path in (self._force_path, self._state_path):
                try:
                    os.unlink(path)
                except FileNotFoundError:
                    pass


class PlantSimulator:
    """
    Enhanced physics plant simulator with timestamped packet tracking.

    Identical mechanics to FakePlantServer but stores (wall_time, forces)
    tuples so tests can check packet rates and force values over time windows.
    """

    MASS: float = 5.0
    GRAVITY: float = 9.81
    DT: float = 1.0 / 400.0

    def __init__(self, force_path: str, state_path: str):
        self._force_path = force_path
        self._state_path = state_path
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        # (wall_time_s, forces_6dof)
        self._packets: list[tuple[float, np.ndarray]] = []
        self.pos = np.array([0.0, 0.0, 50.0], dtype=np.float64)
        self.vel = np.zeros(3, dtype=np.float64)
        self.R = np.eye(3, dtype=np.float64)
        self.omega = np.array([0.0, 0.0, 28.0], dtype=np.float64)

    def start(self):
        self._thread = threading.Thread(target=self._serve, name="plant-sim", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    @property
    def packets(self) -> list[tuple[float, np.ndarray]]:
        with self._lock:
            return list(self._packets)

    @property
    def force_packets(self) -> list[np.ndarray]:
        """Compatibility shim — returns forces without timestamps."""
        with self._lock:
            return [f for _, f in self._packets]

    def _serve(self):
        for path in (self._force_path, self._state_path):
            try:
                os.unlink(path)
            except FileNotFoundError:
                pass

        force_server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        state_server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            force_server.bind(self._force_path)
            state_server.bind(self._state_path)
            force_server.listen(1)
            state_server.listen(1)
            force_server.settimeout(0.5)
            state_server.settimeout(0.5)

            force_conn = state_conn = None
            while not self._stop_event.is_set() and (force_conn is None or state_conn is None):
                if force_conn is None:
                    try:
                        force_conn, _ = force_server.accept()
                    except socket.timeout:
                        pass
                if state_conn is None:
                    try:
                        state_conn, _ = state_server.accept()
                    except socket.timeout:
                        pass

            if force_conn is None or state_conn is None:
                return

            force_conn.settimeout(0.5)
            while not self._stop_event.is_set():
                try:
                    raw = _recv_exactly(force_conn, _FORCE_SIZE)
                except (ConnectionResetError, OSError, socket.timeout):
                    break

                forces = np.array(struct.unpack(_FORCE_FMT, raw), dtype=np.float64)
                with self._lock:
                    self._packets.append((time.monotonic(), forces))

                accel = forces[:3] / self.MASS + np.array([0.0, 0.0, -self.GRAVITY])
                self.vel += accel * self.DT
                self.pos += self.vel * self.DT
                state_conn.sendall(_pack_state(self.pos, self.vel, self.R, self.omega))
        finally:
            for s in filter(None, [
                locals().get("force_conn"), locals().get("state_conn"),
                force_server, state_server,
            ]):
                try:
                    s.close()
                except OSError:
                    pass
            for path in (self._force_path, self._state_path):
                try:
                    os.unlink(path)
                except FileNotFoundError:
                    pass


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

    def send_param_request_list(self) -> None:
        """Ask the vehicle to stream all parameter values."""
        if self._conn is None:
            raise RuntimeError("Call connect() first.")
        self._conn.mav.param_request_list_send(
            self._conn.target_system,
            self._conn.target_component,
        )

    def wait_for_any_of(self, message_types: list[str], timeout: float = 30.0):
        """Return the first message matching any of the given types."""
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

@pytest.mark.skipif(not hasattr(socket, "AF_UNIX"), reason="Requires AF_UNIX sockets")
def test_stack_integration_smoke():
    """
    Fast smoke test: SITL ↔ mediator ↔ FakePlantServer.

    Verifies:
    - All processes start without crashing
    - MAVLink heartbeat received
    - At least 2 force packets flow through the bridge (end-to-end loop works)
    - Latest force wrench is finite (no NaN/Inf)
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
        force_path = str(temp_path / "forces.sock")
        state_path = str(temp_path / "state.sock")
        mediator_log = temp_path / "mediator.log"
        sitl_log = temp_path / "sitl.log"

        fake_plant = FakePlantServer(force_path=force_path, state_path=state_path)
        fake_plant.start()
        mediator_proc = _launch_mediator(sim_dir, repo_root, force_path, state_path, mediator_log)
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
            ap_log = Path("/tmp/ArduCopter.log")
            ap_text = ap_log.read_text(encoding="utf-8", errors="replace") if ap_log.exists() else "(no log)"
            print(f"\n=== ArduCopter.log (last 8000 chars) ===\n{ap_text[-8000:]}")

        gcs = FakeGroundStation()
        try:
            gcs.connect(timeout=30.0, poll=_assert_procs_alive)
            _assert_procs_alive()
            heartbeat = gcs.wait_heartbeat(timeout=30.0)
            assert heartbeat is not None

            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                _assert_procs_alive()
                if len(fake_plant.force_packets) >= 2:
                    break
                time.sleep(0.5)
            else:
                pytest.fail(
                    f"Mediator bridge stalled: only {len(fake_plant.force_packets)} force "
                    "packet(s) received from fake plant after 30 s"
                )

            latest_force = fake_plant.force_packets[-1]
            assert latest_force.shape == (6,)
            assert np.isfinite(latest_force).all()
        except Exception:
            _dump_logs()
            raise
        finally:
            gcs.close()
            _terminate_process(sitl_proc)
            _terminate_process(mediator_proc)
            fake_plant.stop()


# ─────────────────────────────────────────────────────────────────────────────
# Full stack integration test (in-process physics, detailed assertions)
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not hasattr(socket, "AF_UNIX"), reason="Requires AF_UNIX sockets")
def test_full_stack_integration():
    """
    Full stack integration test: SITL ↔ mediator ↔ PlantSimulator.

    Runs the complete data pipeline for 10+ seconds and asserts:

    Bridge loop rate
      ≥ 2 000 force packets in the first 10 s after heartbeat (≥ 200 Hz avg),
      confirming the mediator loop is running at close to the target 400 Hz.

    Force wrench validity
      All 6-DOF force components are finite throughout the run.

    Post-ramp vertical thrust
      After the 5 s aero spin-up ramp, mean Fz (Up) must lie in [15, 80] N.
      Expected value ≈ 49 N (gravity compensation for a 5 kg rotor).

    Anti-torque presence
      At least one force packet must have |Mz| > 0.1 N·m, confirming the
      anti-rotation motor moment is being applied.

    No mediator CRITICAL errors
      The mediator log must not contain any CRITICAL-level log entries.

    MAVLink bidirectionality
      Sending PARAM_REQUEST_LIST must elicit at least one PARAM_VALUE reply
      within 15 s, confirming the full GCS ↔ SITL ↔ vehicle MAVLink loop.
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

    with TemporaryDirectory(prefix="rawes_full_") as tmpdir:
        temp_path = Path(tmpdir)
        force_path = str(temp_path / "forces.sock")
        state_path = str(temp_path / "state.sock")
        mediator_log = temp_path / "mediator.log"
        sitl_log = temp_path / "sitl.log"

        plant = PlantSimulator(force_path=force_path, state_path=state_path)
        plant.start()
        mediator_proc = _launch_mediator(sim_dir, repo_root, force_path, state_path, mediator_log)
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
            ap_log = Path("/tmp/ArduCopter.log")
            if ap_log.exists():
                print(f"\n=== ArduCopter.log ===\n{ap_log.read_text(errors='replace')[-3000:]}")

        gcs = FullGroundStation()
        try:
            # ── Phase 1: connectivity ────────────────────────────────────────
            gcs.connect(timeout=30.0, poll=_procs_alive)
            heartbeat = gcs.wait_heartbeat(timeout=30.0)
            assert heartbeat is not None, "No MAVLink heartbeat received"

            # ── Phase 2: collect 10 s of force data ─────────────────────────
            t_window_start = time.monotonic()
            while time.monotonic() - t_window_start < 10.0:
                _procs_alive()
                time.sleep(0.25)
            t_window_end = time.monotonic()

            all_packets = plant.packets
            assert all_packets, "No force packets received at all"

            # Packets within the 10 s window
            window = [
                (t, f) for t, f in all_packets
                if t_window_start <= t <= t_window_end
            ]
            assert len(window) >= 2000, (
                f"Force loop rate too low: {len(window)} packets in "
                f"{t_window_end - t_window_start:.1f} s "
                f"(need ≥ 2000 for ≥ 200 Hz average)"
            )

            # ── Phase 3: wrench validity ─────────────────────────────────────
            all_forces = np.array([f for _, f in all_packets])
            assert all_forces.shape[1] == 6, "Force wrench must be 6-DOF"
            assert np.isfinite(all_forces).all(), (
                f"NaN or Inf in force wrench — first bad index: "
                f"{np.where(~np.isfinite(all_forces))[0][0]}"
            )

            # ── Phase 4: post-ramp vertical thrust ───────────────────────────
            # The mediator's 5 s aero ramp starts when it enters its main loop.
            # The last 1 000 packets are guaranteed to be post-ramp once the
            # total packet count exceeds ~2 000 (5 s × 400 Hz).
            assert len(all_packets) >= 2000, (
                f"Not enough total packets ({len(all_packets)}) to sample post-ramp thrust"
            )
            post_ramp_forces = [f for _, f in all_packets[-1000:]]
            fz = np.array([f[2] for f in post_ramp_forces])   # index 2 = Fz (Up, ENU)
            mean_fz = float(np.mean(fz))
            assert 15.0 <= mean_fz <= 80.0, (
                f"Post-ramp vertical force out of expected range: "
                f"mean Fz = {mean_fz:.1f} N (expected 15–80 N, "
                f"nominal ≈ 49 N for 5 kg gravity compensation)"
            )

            # ── Phase 5: anti-torque moment present ──────────────────────────
            mz = np.array([f[5] for _, f in all_packets])   # index 5 = Mz
            assert np.any(np.abs(mz) > 0.1), (
                "Anti-rotation motor torque (Mz) is always near zero — "
                "the motor moment path may be broken"
            )

            # ── Phase 6: no CRITICAL errors in mediator log ──────────────────
            if mediator_log.exists():
                log_text = mediator_log.read_text(encoding="utf-8", errors="replace")
                critical_lines = [l for l in log_text.splitlines() if "CRITICAL" in l]
                assert not critical_lines, (
                    f"CRITICAL-level errors in mediator log:\n"
                    + "\n".join(critical_lines[:10])
                )

            # ── Phase 7: MAVLink bidirectionality ────────────────────────────
            gcs.send_param_request_list()
            param_msg = gcs.wait_for_any_of(["PARAM_VALUE"], timeout=15.0)
            assert param_msg is not None, (
                "No PARAM_VALUE received after PARAM_REQUEST_LIST — "
                "uplink MAVLink (GCS → vehicle) or param subsystem may be broken"
            )
            assert hasattr(param_msg, "param_id"), "PARAM_VALUE missing param_id field"
            assert hasattr(param_msg, "param_value"), "PARAM_VALUE missing param_value field"
            assert np.isfinite(param_msg.param_value), (
                f"PARAM_VALUE.param_value is not finite: {param_msg.param_value}"
            )

        except Exception:
            _dump_logs()
            raise
        finally:
            gcs.close()
            _terminate_process(sitl_proc)
            _terminate_process(mediator_proc)
            plant.stop()


# ─────────────────────────────────────────────────────────────────────────────
# MBDyn full stack test (real rigid-body physics)
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not hasattr(socket, "AF_UNIX"), reason="Requires AF_UNIX sockets")
def test_mbdyn_stack_integration():
    """
    Full stack test using real MBDyn rigid-body physics.

    Launches the complete three-process pipeline:
        ArduPilot SITL ↔ mediator.py ↔ MBDyn

    Asserts:
    - All three processes start and stay alive for 15 s
    - MAVLink heartbeat received (SITL ↔ mediator ↔ MBDyn handshake complete)
    - MBDyn runs at least one integration step (output log created and non-empty)
    - Mediator log shows non-zero thrust after the aero ramp (bridge is live)
    - No CRITICAL errors in mediator log
    """
    if os.environ.get(STACK_ENV_FLAG) != "1":
        pytest.skip(f"Set {STACK_ENV_FLAG}=1 to run stack integration tests")

    mbdyn = _find_mbdyn()
    if mbdyn is None:
        pytest.skip("mbdyn binary not found in PATH")

    sim_vehicle = _resolve_sim_vehicle()
    if sim_vehicle is None:
        pytest.skip(
            f"Set {SIM_VEHICLE_ENV} or {ARDUPILOT_ENV} so the stack integration test can launch SITL"
        )

    pytest.importorskip("pymavlink")

    repo_root = Path(__file__).resolve().parents[3]
    sim_dir = repo_root / "simulation"
    mbdyn_dir = sim_dir / "mbdyn"

    # Clean up any stale MBDyn sockets before starting
    for sock_path in (_MBDYN_FORCE_SOCK, _MBDYN_STATE_SOCK):
        try:
            os.unlink(sock_path)
        except FileNotFoundError:
            pass

    with TemporaryDirectory(prefix="rawes_mbdyn_") as tmpdir:
        temp_path = Path(tmpdir)
        mediator_log = temp_path / "mediator.log"
        sitl_log     = temp_path / "sitl.log"
        mbdyn_stdout = temp_path / "mbdyn_stdout.log"
        mbdyn_out    = str(temp_path / "rawes")   # MBDyn output prefix → rawes.log, rawes.out etc.

        # Launch order: MBDyn first (creates sockets), then mediator (connects),
        # then SITL (mediator is already looping when SITL starts).
        mbdyn_proc = subprocess.Popen(
            [str(mbdyn), "-f", "rotor.mbd", "-o", mbdyn_out],
            cwd=str(mbdyn_dir),
            stdout=mbdyn_stdout.open("w", encoding="utf-8"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        mediator_proc = _launch_mediator(
            sim_dir, repo_root,
            _MBDYN_FORCE_SOCK, _MBDYN_STATE_SOCK,
            mediator_log,
        )
        sitl_proc = _launch_sitl(sim_vehicle, sitl_log)

        def _procs_alive():
            for name, proc, log in [
                ("MBDyn",    mbdyn_proc,    mbdyn_stdout),
                ("mediator", mediator_proc, mediator_log),
                ("SITL",     sitl_proc,     sitl_log),
            ]:
                if proc.poll() is not None:
                    log_text = log.read_text(encoding="utf-8", errors="replace") if log.exists() else "(no log)"
                    pytest.fail(f"{name} exited early (rc={proc.returncode}):\n{log_text[-3000:]}")

        def _dump_logs():
            for name, log in [
                ("MBDyn stdout", mbdyn_stdout),
                ("mediator",     mediator_log),
                ("SITL",         sitl_log),
            ]:
                text = log.read_text(encoding="utf-8", errors="replace") if log.exists() else "(no log)"
                print(f"\n=== {name} ===\n{text[-2000:]}")
            mbdyn_log_file = Path(mbdyn_out + ".log")
            if mbdyn_log_file.exists():
                print(f"\n=== MBDyn .log ===\n{mbdyn_log_file.read_text(errors='replace')[-2000:]}")
            ap_log = Path("/tmp/ArduCopter.log")
            if ap_log.exists():
                print(f"\n=== ArduCopter.log ===\n{ap_log.read_text(errors='replace')[-2000:]}")

        gcs = FakeGroundStation()
        try:
            # ── 1. MAVLink heartbeat (proves all three are talking) ───────────
            gcs.connect(timeout=30.0, poll=_procs_alive)
            heartbeat = gcs.wait_heartbeat(timeout=30.0)
            assert heartbeat is not None, "No MAVLink heartbeat after MBDyn stack start"

            # ── 2. Sustained run for 15 s ────────────────────────────────────
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                _procs_alive()
                time.sleep(1.0)

            # ── 3. MBDyn produced output (ran at least one step) ─────────────
            mbdyn_log_file = Path(mbdyn_out + ".log")
            assert mbdyn_log_file.exists() and mbdyn_log_file.stat().st_size > 0, (
                f"MBDyn output log {mbdyn_log_file} is missing or empty — "
                "MBDyn may not have started the simulation"
            )

            # ── 4. Mediator shows non-zero thrust after ramp ─────────────────
            med_text = mediator_log.read_text(encoding="utf-8", errors="replace")
            thrust_values = [
                float(m.group(1))
                for m in re.finditer(r"T=(\d+\.\d+)N", med_text)
            ]
            post_ramp = [t for t in thrust_values if t > 1.0]
            assert len(post_ramp) >= 3, (
                f"Expected non-zero thrust in mediator log after aero ramp. "
                f"Thrust values found: {thrust_values[:20]}"
            )

            # ── 5. No CRITICAL errors in mediator log ────────────────────────
            critical_lines = [l for l in med_text.splitlines() if "CRITICAL" in l]
            assert not critical_lines, (
                "CRITICAL-level errors in mediator log:\n"
                + "\n".join(critical_lines[:10])
            )

        except Exception:
            _dump_logs()
            raise
        finally:
            gcs.close()
            _terminate_process(sitl_proc)
            _terminate_process(mediator_proc)
            _terminate_process(mbdyn_proc)
            for sock_path in (_MBDYN_FORCE_SOCK, _MBDYN_STATE_SOCK):
                try:
                    os.unlink(sock_path)
                except FileNotFoundError:
                    pass
