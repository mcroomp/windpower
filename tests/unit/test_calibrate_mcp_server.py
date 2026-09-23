from __future__ import annotations

import threading
import time

from calibrate import mcp_server


class _FakeSession:
    _target_system = 1
    _target_component = 2

    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


def test_bridge_reuses_persistent_connection(monkeypatch):
    session = _FakeSession()
    calls = []

    def fake_connect(port, baud):
        calls.append((port, baud))
        print("connected output")
        return session

    monkeypatch.setattr(mcp_server.repl, "_connect", fake_connect)
    bridge = mcp_server.CalibrationBridge()

    connected = bridge.connect()
    connected_again = bridge.connect()

    assert calls == [("COM6", 57600)]
    assert connected["connected"] is True
    assert connected["system_id"] == 1
    assert connected["component_id"] == 2
    assert connected["output"] == "connected output"
    assert connected_again["output"] == "Already connected."

    disconnected = bridge.disconnect()
    assert disconnected["connected"] is False
    assert session.closed is True


def test_bridge_calls_shared_run_library_and_captures_output(monkeypatch):
    session = _FakeSession()
    bridge = mcp_server.CalibrationBridge()
    monkeypatch.setattr(mcp_server.repl, "_connect", lambda _port, _baud: session)
    bridge.connect()
    received = []

    def fake_run(actual_session, args, *, stop_requested):
        received.append((actual_session, args, stop_requested()))
        print("command output")

    monkeypatch.setattr(mcp_server, "_cmd_run", fake_run)

    result = bridge.run(
        "passive",
        ["--protocol-debug", "--duration", "5"],
    )

    assert result.ok is True
    assert result.output == "command output"
    assert received == [(
        session,
        ["passive", "--protocol-debug", "--duration", "5"],
        False,
    )]


def test_bridge_rejects_commands_until_connected():
    bridge = mcp_server.CalibrationBridge()

    try:
        bridge.status_command()
    except RuntimeError as error:
        assert str(error) == "Not connected. Call connect first."
    else:
        raise AssertionError("Expected disconnected command to fail")


def test_stop_operation_interrupts_active_run(monkeypatch):
    session = _FakeSession()
    bridge = mcp_server.CalibrationBridge()
    monkeypatch.setattr(mcp_server.repl, "_connect", lambda _port, _baud: session)
    bridge.connect()
    run_started = threading.Event()
    run_stopped = threading.Event()

    def fake_run(_session, _args, *, stop_requested):
        run_started.set()
        while not stop_requested():
            time.sleep(0.01)
        run_stopped.set()

    monkeypatch.setattr(mcp_server, "_cmd_run", fake_run)
    thread = threading.Thread(target=bridge.run, args=("passive",))
    thread.start()
    assert run_started.wait(timeout=1.0)

    result = bridge.stop_operation()
    thread.join(timeout=1.0)

    assert result["stop_requested"] is True
    assert run_stopped.is_set()
    assert not thread.is_alive()


def test_watchdog_requests_stop_then_kills_stuck_process(monkeypatch):
    bridge = mcp_server.CalibrationBridge()
    stopped = threading.Event()
    exited = threading.Event()
    exit_codes = []
    monkeypatch_bridge_stop = bridge.request_stop

    def request_stop():
        monkeypatch_bridge_stop()
        stopped.set()

    monkeypatch.setattr(bridge, "request_stop", request_stop)
    watchdog = mcp_server.CommandWatchdog(
        bridge,
        grace_s=0.02,
        exit_process=lambda code: (exit_codes.append(code), exited.set()),
    )

    with watchdog.monitor("stuck", timeout_s=0.02):
        assert stopped.wait(timeout=0.5)
        assert exited.wait(timeout=0.5)

    assert exit_codes == [mcp_server.WATCHDOG_EXIT_CODE]


def test_watchdog_does_not_kill_operation_that_stops_during_grace(monkeypatch):
    bridge = mcp_server.CalibrationBridge()
    stopped = threading.Event()
    exit_codes = []
    original_request_stop = bridge.request_stop

    def request_stop():
        original_request_stop()
        stopped.set()

    monkeypatch.setattr(bridge, "request_stop", request_stop)
    watchdog = mcp_server.CommandWatchdog(
        bridge,
        grace_s=0.1,
        exit_process=exit_codes.append,
    )

    with watchdog.monitor("slow", timeout_s=0.02):
        assert stopped.wait(timeout=0.5)

    time.sleep(0.15)
    assert exit_codes == []


def test_shutdown_refuses_exit_when_disarm_is_unconfirmed(monkeypatch):
    session = _FakeSession()
    bridge = mcp_server.CalibrationBridge()
    monkeypatch.setattr(mcp_server.repl, "_connect", lambda _port, _baud: session)
    monkeypatch.setattr(mcp_server, "_disarm", lambda *_args, **_kwargs: False)
    bridge.connect()

    result = bridge.shutdown()

    assert result.ok is False
    assert result.server_shutdown is False
    assert bridge.connected is True
    assert session.closed is False


def test_forced_shutdown_closes_connection_after_failed_disarm(monkeypatch):
    session = _FakeSession()
    bridge = mcp_server.CalibrationBridge()
    monkeypatch.setattr(mcp_server.repl, "_connect", lambda _port, _baud: session)
    monkeypatch.setattr(mcp_server, "_disarm", lambda *_args, **_kwargs: False)
    bridge.connect()

    result = bridge.shutdown(force=True)

    assert result.ok is False
    assert result.server_shutdown is True
    assert bridge.connected is False
    assert session.closed is True
