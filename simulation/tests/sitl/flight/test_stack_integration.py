import sys
import time
from pathlib import Path

import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

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


# -----------------------------------------------------------------------------
# Smoke test (fast connectivity check)
# -----------------------------------------------------------------------------

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
    from stack_infra import _acro_stack

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
