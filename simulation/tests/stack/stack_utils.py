"""
stack_utils.py — shared constants and process helpers for RAWES stack tests.

Centralises the env-var names, logging setup, process launch/teardown, log
copying, and port cleanup so every stack test file can import from one place.
"""
import logging
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Environment variable names
# ---------------------------------------------------------------------------

STACK_ENV_FLAG  = "RAWES_RUN_STACK_INTEGRATION"
ARDUPILOT_ENV   = "RAWES_ARDUPILOT_PATH"
SIM_VEHICLE_ENV = "RAWES_SIM_VEHICLE"


# ---------------------------------------------------------------------------
# Process helpers
# ---------------------------------------------------------------------------

def _resolve_sim_vehicle() -> "Path | None":
    explicit = os.environ.get(SIM_VEHICLE_ENV)
    if explicit:
        candidate = Path(explicit)
        return candidate if candidate.is_file() else None

    ardupilot_root = os.environ.get(ARDUPILOT_ENV)
    if not ardupilot_root:
        return None

    candidate = Path(ardupilot_root) / "Tools" / "autotest" / "sim_vehicle.py"
    return candidate if candidate.is_file() else None


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


def _kill_by_port(port: int) -> None:
    """Kill any process still holding the given TCP port (Linux only).

    sim_vehicle.py spawns arducopter-heli in a different process group, so
    os.killpg() on sim_vehicle.py's PID may leave the SITL binary running.
    This function finds the remaining process via /proc/net/tcp and kills it.
    """
    import signal as _signal

    # Try fuser first (fast, available when psmisc is installed)
    try:
        subprocess.run(
            ["fuser", "-k", f"{port}/tcp"],
            timeout=5.0,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # Fallback: parse /proc/net/tcp to find the inode, then scan /proc/*/fd/
    try:
        hex_port = f"{port:04X}"
        inode = None
        with open("/proc/net/tcp") as f:
            for line in f:
                parts = line.split()
                if len(parts) < 10:
                    continue
                local_addr = parts[1]
                # local_addr format: "0100007F:1770" (ip little-endian, port big-endian hex)
                if ":" not in local_addr:
                    continue
                if local_addr.split(":")[1] == hex_port:
                    inode = parts[9]
                    break
        if inode is None:
            return

        # Find PIDs whose fd/ links point to this inode
        import glob as _glob
        target = f"socket:[{inode}]"
        for fd_path in _glob.glob("/proc/*/fd/*"):
            try:
                if os.readlink(fd_path) == target:
                    pid = int(fd_path.split("/")[2])
                    try:
                        os.kill(pid, _signal.SIGKILL)
                    except (ProcessLookupError, PermissionError):
                        pass
            except OSError:
                pass
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

_LOG_FMT     = "%(asctime)s %(name)-20s %(levelname)-8s %(message)s"
_LOG_DATEFMT = "%H:%M:%S"


def _configure_logging(log_path: Path) -> None:
    """Configure root logger: INFO to stdout + file handler at log_path."""
    logging.basicConfig(
        level=logging.INFO,
        format=_LOG_FMT,
        datefmt=_LOG_DATEFMT,
        force=True,
    )
    fh = logging.FileHandler(str(log_path), encoding="utf-8")
    fh.setFormatter(logging.Formatter(_LOG_FMT, datefmt=_LOG_DATEFMT))
    logging.getLogger().addHandler(fh)


# ---------------------------------------------------------------------------
# Log file copying
# ---------------------------------------------------------------------------

def copy_logs_to_dir(log_dir: Path, copies: dict) -> None:
    """Copy log files into log_dir, skipping any that don't exist yet."""
    log_dir.mkdir(exist_ok=True)
    for dest_name, src in copies.items():
        if Path(src).exists():
            shutil.copy2(src, log_dir / dest_name)


# ---------------------------------------------------------------------------
# Port availability check
# ---------------------------------------------------------------------------

# Default SITL ports — must match StackConfig in conftest.py
_SITL_GCS_PORT  = 5760   # TCP
_SITL_JSON_PORT = 9002   # UDP

_PORT_CHECKS = [
    ("127.0.0.1", _SITL_GCS_PORT,  "tcp",
     "SITL GCS port — a previous SITL process may still be running"),
    ("0.0.0.0",   _SITL_JSON_PORT, "udp",
     "mediator/sensor JSON port — a previous mediator or sensor worker may still be running"),
]


def check_ports_free(retry_s: float = 15.0, poll_interval: float = 0.5) -> None:
    """Raise RuntimeError if SITL or mediator ports are in use after retry_s seconds."""
    for host, port, proto, hint in _PORT_CHECKS:
        kind = socket.SOCK_STREAM if proto == "tcp" else socket.SOCK_DGRAM
        deadline = time.monotonic() + retry_s
        while True:
            s = socket.socket(socket.AF_INET, kind)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind((host, port))
                s.close()
                break
            except OSError as exc:
                s.close()
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"{proto.upper()} port {host}:{port} is still in use after "
                        f"{retry_s:.0f}s.\n"
                        f"  Hint: {hint}\n"
                        f"  Original error: {exc}"
                    ) from exc
                time.sleep(poll_interval)
