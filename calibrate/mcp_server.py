"""Local MCP server exposing the complete RAWES calibration command surface."""
from __future__ import annotations

import argparse
import atexit
import contextlib
import io
import os
import threading
from dataclasses import dataclass
from typing import Any, Callable

import anyio
from mcp.server.mcpserver import MCPServer

from calibrate import repl
from calibrate.hw import _disarm, _ping_ports
from calibrate.params import _cmd_logs
from calibrate.run import _cmd_run
from calibrate.watch import _cmd_watch
from groundstation.gcs import RawesGCS


DEFAULT_COMMAND_TIMEOUT_S = 30.0
DEFAULT_LONG_COMMAND_TIMEOUT_S = 600.0
DEFAULT_TIMEOUT_GRACE_S = 10.0
WATCHDOG_EXIT_CODE = 124


def _write_watchdog_message(message: str) -> None:
    try:
        os.write(2, message.encode("ascii", errors="replace"))
    except OSError:
        pass


@dataclass(frozen=True)
class CommandResult:
    ok: bool
    output: str


@dataclass(frozen=True)
class ShutdownResult:
    ok: bool
    server_shutdown: bool
    output: str


class CommandWatchdog:
    """Request cooperative cancellation, then kill a wedged MCP process."""

    def __init__(
        self,
        bridge: CalibrationBridge,
        *,
        grace_s: float,
        exit_process: Callable[[int], None] = os._exit,
    ) -> None:
        self._bridge = bridge
        self._grace_s = grace_s
        self._exit_process = exit_process

    @contextlib.contextmanager
    def monitor(self, operation_name: str, timeout_s: float):
        if timeout_s <= 0:
            raise ValueError("Command timeout must be greater than zero.")

        completed = threading.Event()

        def request_stop() -> None:
            if completed.is_set():
                return
            self._bridge.request_stop()
            message = (
                f"RAWES MCP command {operation_name!r} exceeded {timeout_s:.1f}s; "
                f"allowing {self._grace_s:.1f}s for safety shutdown.\n"
            )
            _write_watchdog_message(message)

        def kill_process() -> None:
            if completed.is_set():
                return
            message = (
                f"RAWES MCP command {operation_name!r} did not stop after its "
                "safety grace period; terminating the MCP process.\n"
            )
            _write_watchdog_message(message)
            self._exit_process(WATCHDOG_EXIT_CODE)

        stop_timer = threading.Timer(timeout_s, request_stop)
        kill_timer = threading.Timer(timeout_s + self._grace_s, kill_process)
        stop_timer.daemon = True
        kill_timer.daemon = True
        stop_timer.start()
        kill_timer.start()
        try:
            yield
        finally:
            completed.set()
            stop_timer.cancel()
            kill_timer.cancel()


class CalibrationBridge:
    """Own one MAVLink connection and serialize all calibration operations."""

    def __init__(self, default_port: str = "COM6", default_baud: int = 57600) -> None:
        self._default_port = default_port
        self._default_baud = default_baud
        self._session: RawesGCS | None = None
        self._port: str | None = None
        self._baud: int | None = None
        self._lock = threading.RLock()
        self._stop_requested = threading.Event()

    @property
    def connected(self) -> bool:
        return self._session is not None

    def status(self) -> dict[str, Any]:
        with self._lock:
            return self._status_snapshot()

    def _status_snapshot(self) -> dict[str, Any]:
        session = self._session
        return {
            "connected": session is not None,
            "port": self._port,
            "baud": self._baud,
            "system_id": session._target_system if session is not None else None,
            "component_id": session._target_component if session is not None else None,
        }

    def connect(
        self,
        port: str | None = None,
        baud: int | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            requested_port = port or self._default_port
            requested_baud = baud or self._default_baud
            if self._session is not None:
                if self._port == requested_port and self._baud == requested_baud:
                    return {**self.status(), "output": "Already connected."}
                raise RuntimeError(
                    "Already connected. Disconnect before changing port or baud."
                )

            output = io.StringIO()
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                self._session = repl._connect(requested_port, requested_baud)
            self._port = requested_port
            self._baud = requested_baud
            return {**self.status(), "output": output.getvalue().strip()}

    def disconnect(self) -> dict[str, Any]:
        with self._lock:
            if self._session is None:
                return {**self.status(), "output": "Already disconnected."}
            self._session.close()
            self._session = None
            self._port = None
            self._baud = None
            return {**self.status(), "output": "Disconnected."}

    def _invoke(
        self,
        operation: Callable[[RawesGCS], None],
    ) -> CommandResult:
        with self._lock:
            if self._session is None:
                raise RuntimeError("Not connected. Call connect first.")
            output = io.StringIO()
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                operation(self._session)
            return CommandResult(ok=True, output=output.getvalue().strip())

    def status_command(self) -> CommandResult:
        return self._invoke(repl._print_status)

    def reboot(self) -> CommandResult:
        return self._invoke(repl._cmd_reboot)

    def arm(self, duration_s: float | None = None) -> CommandResult:
        args = [] if duration_s is None else ["--duration", str(duration_s)]
        return self._invoke(lambda session: repl._cmd_arm(session, args))

    def get_parameter(self, name: str) -> CommandResult:
        return self._invoke(lambda session: repl._cmd_get(session, [name]))

    def set_parameter(self, name: str, value: float) -> CommandResult:
        return self._invoke(
            lambda session: repl._cmd_set(session, [name, str(value)])
        )

    def swash(self, args: list[str]) -> CommandResult:
        return self._invoke(lambda session: repl._cmd_swash(session, args))

    def servo(self, args: list[str]) -> CommandResult:
        return self._invoke(lambda session: repl._cmd_servo(session, args))

    def motor(self, args: list[str], *, force: bool = True) -> CommandResult:
        return self._invoke(
            lambda session: repl._cmd_motor(session, args, force=force)
        )

    def run(self, mode: str, args: list[str] | None = None) -> CommandResult:
        self._stop_requested.clear()
        try:
            return self._invoke(
                lambda session: _cmd_run(
                    session,
                    [mode, *(args or [])],
                    stop_requested=self._stop_requested.is_set,
                )
            )
        finally:
            self._stop_requested.clear()

    def request_stop(self) -> None:
        self._stop_requested.set()

    def stop_operation(self) -> dict[str, Any]:
        self.request_stop()
        return {
            **self._status_snapshot(),
            "stop_requested": True,
            "output": "Stop requested; active run will perform safety shutdown.",
        }

    def shutdown(self, *, force: bool = False) -> ShutdownResult:
        self._stop_requested.set()
        with self._lock:
            output = io.StringIO()
            ok = True
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                if self._session is not None:
                    print("Stopping MCP server: confirming hardware safe-off state ...")
                    ok = _disarm(self._session, timeout=5.0, force=True)
                    if not ok and not force:
                        print(
                            "[REFUSED] Hardware disarm was not confirmed; MCP server "
                            "will remain running. Retry shutdown_server(force=true) "
                            "only after independently confirming hardware safety."
                        )
                    else:
                        self._session.close()
                        self._session = None
                        self._port = None
                        self._baud = None
                        print("MAVLink disconnected; MCP server exiting.")
                else:
                    print("MCP server exiting; no MAVLink connection was open.")
            self._stop_requested.clear()
            return ShutdownResult(
                ok=ok,
                server_shutdown=ok or force,
                output=output.getvalue().strip(),
            )

    def watch(self, stream: str, args: list[str] | None = None) -> CommandResult:
        return self._invoke(
            lambda session: _cmd_watch(session, [stream, *(args or [])])
        )

    def logs(self, args: list[str] | None = None) -> CommandResult:
        return self._invoke(lambda session: _cmd_logs(session, args or []))

    def script(self, subcommand: str, args: list[str] | None = None) -> CommandResult:
        return self._invoke(
            lambda session: repl._cmd_script(
                session, [subcommand, *(args or [])]
            )
        )

    def config(self, subcommand: str, args: list[str] | None = None) -> CommandResult:
        return self._invoke(
            lambda session: repl._cmd_config(
                session, [subcommand, *(args or [])]
            )
        )

    def emergency_disarm(self) -> CommandResult:
        with self._lock:
            if self._session is None:
                raise RuntimeError("Not connected. Call connect first.")
            output = io.StringIO()
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                ok = _disarm(self._session, timeout=5.0, force=True)
            return CommandResult(ok=ok, output=output.getvalue().strip())

    def disarm(self) -> CommandResult:
        with self._lock:
            if self._session is None:
                raise RuntimeError("Not connected. Call connect first.")
            output = io.StringIO()
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                ok = _disarm(self._session, timeout=10.0, force=False)
            return CommandResult(ok=ok, output=output.getvalue().strip())


def create_server(
    default_port: str = "COM6",
    default_baud: int = 57600,
    shutdown_requested: threading.Event | None = None,
    command_timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S,
    long_command_timeout_s: float = DEFAULT_LONG_COMMAND_TIMEOUT_S,
    timeout_grace_s: float = DEFAULT_TIMEOUT_GRACE_S,
) -> tuple[MCPServer, CalibrationBridge]:
    bridge = CalibrationBridge(default_port, default_baud)
    watchdog = CommandWatchdog(bridge, grace_s=timeout_grace_s)
    shutdown_requested = shutdown_requested or threading.Event()
    server = MCPServer(
        name="rawes-calibrate",
        description=(
            "Control and diagnose RAWES calibration hardware through the same "
            "command dispatcher as python -m calibrate."
        ),
        instructions=(
            "Connect before issuing commands. Hardware operations are serialized. "
            "Use emergency_disarm immediately if behavior is unsafe. "
            "Every tool has a process watchdog; a timed-out operation gets a safety "
            "shutdown grace period before the MCP process is terminated. "
            "Each tool calls the shared calibration library directly; no calibrate "
            "command-line subprocess is launched."
        ),
    )

    async def _call(
        operation_name: str,
        operation: Callable[[], Any],
        timeout_s: float = command_timeout_s,
    ) -> Any:
        with watchdog.monitor(operation_name, timeout_s):
            return await anyio.to_thread.run_sync(operation)

    @server.tool()
    async def connect(
        port: str | None = None,
        baud: int | None = None,
    ) -> dict[str, Any]:
        """Connect to ArduPilot, defaulting to COM6 at 57600 baud."""
        return await _call("connect", lambda: bridge.connect(port, baud))

    @server.tool()
    async def connection_status() -> dict[str, Any]:
        """Report the current persistent MAVLink connection."""
        return await _call("connection_status", bridge.status)

    @server.tool()
    async def disconnect() -> dict[str, Any]:
        """Close the persistent MAVLink connection."""
        return await _call("disconnect", bridge.disconnect)

    @server.tool()
    async def stop_operation() -> dict[str, Any]:
        """Stop an active run through its normal safety-shutdown lifecycle."""
        return bridge.stop_operation()

    @server.tool()
    async def shutdown_server(force: bool = False) -> dict[str, Any]:
        """Safely disconnect hardware and terminate this MCP server process."""
        result = await _call(
            "shutdown_server",
            lambda: bridge.shutdown(force=force),
        )
        if result.server_shutdown:
            shutdown_requested.set()
        return {
            "ok": result.ok,
            "server_shutdown": result.server_shutdown,
            "output": result.output,
        }

    @server.tool()
    async def calibrate_help() -> str:
        """Return the complete calibrate command reference."""
        return repl._HELP

    @server.tool()
    async def scan_ports(baud: int = 57600) -> list[dict[str, Any]]:
        """Probe serial ports for ArduPilot heartbeats without connecting."""
        return await _call("scan_ports", lambda: _ping_ports(baud))

    async def _result(
        operation_name: str,
        operation: Callable[[], CommandResult],
        timeout_s: float = command_timeout_s,
    ) -> dict[str, Any]:
        result = await _call(operation_name, operation, timeout_s)
        return {"ok": result.ok, "output": result.output}

    @server.tool()
    async def hardware_status() -> dict[str, Any]:
        """Read the complete calibrate hardware/parameter status report."""
        return await _result(
            "hardware_status",
            bridge.status_command,
            long_command_timeout_s,
        )

    @server.tool()
    async def reboot() -> dict[str, Any]:
        """Reboot the connected Pixhawk."""
        return await _result("reboot", bridge.reboot)

    @server.tool()
    async def arm(duration_s: float | None = None) -> dict[str, Any]:
        """Arm in ACRO mode, optionally with a bounded duration."""
        timeout_s = (
            long_command_timeout_s
            if duration_s is None
            else max(command_timeout_s, duration_s + command_timeout_s)
        )
        return await _result("arm", lambda: bridge.arm(duration_s), timeout_s)

    @server.tool()
    async def disarm() -> dict[str, Any]:
        """Request a normal disarm and report confirmation."""
        return await _result("disarm", bridge.disarm)

    @server.tool()
    async def get_parameter(name: str) -> dict[str, Any]:
        """Read one ArduPilot parameter."""
        return await _result(
            "get_parameter",
            lambda: bridge.get_parameter(name),
        )

    @server.tool()
    async def set_parameter(name: str, value: float) -> dict[str, Any]:
        """Write and verify one ArduPilot parameter."""
        return await _result(
            "set_parameter",
            lambda: bridge.set_parameter(name, value),
        )

    @server.tool()
    async def swash(args: list[str]) -> dict[str, Any]:
        """Run a swash operation; args follow calibrate swash help."""
        return await _result("swash", lambda: bridge.swash(args))

    @server.tool()
    async def servo(args: list[str]) -> dict[str, Any]:
        """Run a servo operation; args follow calibrate servo help."""
        return await _result("servo", lambda: bridge.servo(args))

    @server.tool()
    async def motor(args: list[str], force: bool = True) -> dict[str, Any]:
        """Run a motor operation with the existing calibration safety lifecycle."""
        return await _result(
            "motor",
            lambda: bridge.motor(args, force=force),
            long_command_timeout_s,
        )

    @server.tool()
    async def run_mode(
        mode: str,
        args: list[str] | None = None,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        """Run a calibration mode such as passive, steady, or acro-manual."""
        return await _result(
            "run_mode",
            lambda: bridge.run(mode, args),
            timeout_s if timeout_s is not None else long_command_timeout_s,
        )

    @server.tool()
    async def watch(
        stream: str,
        args: list[str] | None = None,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        """Watch a telemetry stream using the persistent connection."""
        return await _result(
            "watch",
            lambda: bridge.watch(stream, args),
            timeout_s if timeout_s is not None else long_command_timeout_s,
        )

    @server.tool()
    async def logs(args: list[str] | None = None) -> dict[str, Any]:
        """List, download, or erase dataflash logs."""
        return await _result(
            "logs",
            lambda: bridge.logs(args),
            long_command_timeout_s,
        )

    @server.tool()
    async def script(
        subcommand: str,
        args: list[str] | None = None,
    ) -> dict[str, Any]:
        """Upload, list, or remove Lua scripts."""
        return await _result(
            "script",
            lambda: bridge.script(subcommand, args),
            long_command_timeout_s,
        )

    @server.tool()
    async def config(
        subcommand: str,
        args: list[str] | None = None,
    ) -> dict[str, Any]:
        """Check, show, fix, or apply the canonical hardware configuration."""
        return await _result(
            "config",
            lambda: bridge.config(subcommand, args),
            long_command_timeout_s,
        )

    @server.tool()
    async def emergency_disarm() -> dict[str, Any]:
        """Immediately send a force-disarm command and report confirmation."""
        result = await _call("emergency_disarm", bridge.emergency_disarm)
        return {"ok": result.ok, "output": result.output}

    atexit.register(bridge.disconnect)
    return server, bridge


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="RAWES calibration MCP server (stdio transport)"
    )
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument(
        "--command-timeout",
        type=float,
        default=DEFAULT_COMMAND_TIMEOUT_S,
        help="Watchdog deadline for one-shot tools in seconds (default: 30)",
    )
    parser.add_argument(
        "--long-command-timeout",
        type=float,
        default=DEFAULT_LONG_COMMAND_TIMEOUT_S,
        help="Watchdog deadline for run/watch/other long tools in seconds (default: 600)",
    )
    parser.add_argument(
        "--timeout-grace",
        type=float,
        default=DEFAULT_TIMEOUT_GRACE_S,
        help="Safety-shutdown grace before hard process exit in seconds (default: 10)",
    )
    return parser


def main() -> None:
    args = _build_parser().parse_args()
    shutdown_requested = threading.Event()
    server, bridge = create_server(
        args.port,
        args.baud,
        shutdown_requested=shutdown_requested,
        command_timeout_s=args.command_timeout,
        long_command_timeout_s=args.long_command_timeout,
        timeout_grace_s=args.timeout_grace,
    )

    async def run_until_shutdown() -> None:
        async def serve() -> None:
            try:
                await server.run_stdio_async()
            finally:
                shutdown_requested.set()

        async with anyio.create_task_group() as tasks:
            tasks.start_soon(serve)
            await anyio.to_thread.run_sync(shutdown_requested.wait)
            await anyio.sleep(0.25)
            tasks.cancel_scope.cancel()

    try:
        anyio.run(run_until_shutdown)
    finally:
        bridge.disconnect()


if __name__ == "__main__":
    main()
