"""
stack_utils.py — Shared utilities for all RAWES ArduPilot SITL stack tests.

Centralises the subprocess helpers, port-checking, and logging setup that
are common to every test family that runs against an ArduPilot SITL instance
(main pumping-cycle tests AND the counter-torque motor tests).

Importing
---------
Any module inside simulation/ (or a subdirectory) can use:

    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[N]))  # reach simulation/
    from stack_utils import (
        STACK_ENV_FLAG, ARDUPILOT_ENV, SIM_VEHICLE_ENV,
        SITL_GCS_PORT, SITL_JSON_PORT, GCS_ADDRESS,
        _resolve_sim_vehicle, _launch_sitl,
        _terminate_process, _kill_by_port,
        _configure_logging, copy_logs_to_dir, check_ports_free,
    )
"""
from __future__ import annotations

import logging
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Environment variable names  (shared by all stack tests)
# ---------------------------------------------------------------------------

#: Set to "1" to allow stack integration tests to run
STACK_ENV_FLAG  = "RAWES_RUN_STACK_INTEGRATION"

#: Path to the ArduPilot repository root (used to locate sim_vehicle.py)
ARDUPILOT_ENV   = "RAWES_ARDUPILOT_PATH"

#: Direct path to sim_vehicle.py (alternative to ARDUPILOT_ENV)
SIM_VEHICLE_ENV = "RAWES_SIM_VEHICLE"

# ---------------------------------------------------------------------------
# Port / address constants
# ---------------------------------------------------------------------------

#: TCP port SITL listens on for GCS MAVLink connections
SITL_GCS_PORT  : int = 5760

#: UDP port where the mediator binds to receive SITL servo outputs
SITL_JSON_PORT : int = 9002

#: pymavlink connection string for the GCS
GCS_ADDRESS : str = f"tcp:127.0.0.1:{SITL_GCS_PORT}"


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------

def _resolve_sim_vehicle() -> Path | None:
    """
    Locate sim_vehicle.py from environment variables.

    Checks RAWES_SIM_VEHICLE first (explicit path), then
    RAWES_ARDUPILOT_PATH / Tools / autotest / sim_vehicle.py.

    Returns the Path if found and is a file, otherwise None.
    """
    explicit = os.environ.get(SIM_VEHICLE_ENV)
    if explicit:
        p = Path(explicit)
        return p if p.is_file() else None

    ardupilot_root = os.environ.get(ARDUPILOT_ENV)
    if not ardupilot_root:
        return None

    p = Path(ardupilot_root) / "Tools" / "autotest" / "sim_vehicle.py"
    return p if p.is_file() else None


# ---------------------------------------------------------------------------
# Process management
# ---------------------------------------------------------------------------

def _launch_sitl(
    sim_vehicle: Path,
    log_path: Path,
    add_param_file: "Path | None" = None,
) -> subprocess.Popen:
    """
    Launch ArduPilot SITL (heli frame, JSON physics backend).

    Parameters
    ----------
    sim_vehicle    : path to sim_vehicle.py
    log_path       : file to receive stdout + stderr from SITL
    add_param_file : if given, pass --add-param-file <path> (boot defaults)

    Returns
    -------
    Running Popen instance.

    Notes
    -----
    eeprom.bin is deleted before every launch so each run starts from
    copter-heli.parm defaults — identical to a fresh container.  Prevents
    stale param values (e.g. H_RSC_MODE=1) from accumulating across test
    runs and causing intermittent Motor Interlock arm failures.
    """
    eeprom = Path(sim_vehicle).parent.parent.parent / "eeprom.bin"
    if eeprom.exists():
        eeprom.unlink()
    cmd = [
        sys.executable, str(sim_vehicle),
        "--vehicle", "ArduCopter",
        "--frame",   "heli",
        "--custom-location=51.5074,-0.1278,50,0",
        "--model",   "JSON",
        "--sim-address", "127.0.0.1",
        "--no-rebuild",
        "--no-mavproxy",
    ]
    if add_param_file is not None:
        cmd += ["--add-param-file", str(add_param_file)]
    return subprocess.Popen(
        cmd,
        cwd=str(sim_vehicle.parent.parent.parent),
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _terminate_process(proc: subprocess.Popen) -> None:
    """
    Gracefully terminate a subprocess; SIGKILL if it doesn't stop within 10 s.

    Safe to call on an already-exited process (no-op).
    """
    if proc.poll() is not None:
        return
    import signal
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        proc.terminate()
    try:
        proc.wait(timeout=10.0)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except (ProcessLookupError, AttributeError):
        proc.kill()
    proc.wait(timeout=5.0)


def _kill_by_port(port: int) -> None:
    """
    Kill any process still holding the given TCP port (Linux / Docker only).

    sim_vehicle.py spawns arducopter-heli in a different process group so
    _terminate_process() may leave the SITL binary running.  This finds the
    remaining process via fuser (preferred) or /proc/net/tcp fallback and
    sends SIGKILL.
    """
    import signal as _signal

    # Try fuser first (fast, available when psmisc is installed in Docker image)
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
                if ":" not in local_addr:
                    continue
                if local_addr.split(":")[1] == hex_port:
                    inode = parts[9]
                    break
        if inode is None:
            return

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
# Port checking
# ---------------------------------------------------------------------------

def check_ports_free(
    ports: list[tuple[str, int, str, str]] | None = None,
    retry_s: float = 15.0,
    poll_interval: float = 0.5,
) -> None:
    """
    Verify that the required stack ports are free before launching processes.

    Parameters
    ----------
    ports : list of (host, port, proto, hint) tuples.
        proto is "tcp" or "udp".  hint is shown in the error message.
        Defaults to the standard SITL GCS + JSON ports.
    retry_s : how long to wait for a port to become free [s]
    poll_interval : polling interval [s]

    Raises
    ------
    RuntimeError if any port is still in use after retry_s seconds.
    """
    if ports is None:
        ports = [
            ("127.0.0.1", SITL_GCS_PORT,  "tcp",
             "SITL GCS port — a previous SITL process may still be running"),
            ("0.0.0.0",   SITL_JSON_PORT, "udp",
             "mediator JSON port — a previous mediator may still be running"),
        ]
    for host, port, proto, hint in ports:
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
                        f"{proto.upper()} port {host}:{port} still in use after "
                        f"{retry_s:.0f}s.\n  Hint: {hint}\n  Error: {exc}"
                    ) from exc
                time.sleep(poll_interval)


# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

def _configure_logging(log_file: Path) -> None:
    """
    Configure root logger to write DEBUG to log_file and INFO to stdout.

    Replaces any existing handlers.  Call once per test process.
    """
    fmt     = "%(asctime)s %(name)-20s %(levelname)-8s %(message)s"
    datefmt = "%H:%M:%S"
    fh = logging.FileHandler(str(log_file), encoding="utf-8")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(fmt, datefmt))
    sh = logging.StreamHandler(sys.stdout)
    sh.setLevel(logging.INFO)
    sh.setFormatter(logging.Formatter(fmt, datefmt))
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    root.handlers.clear()
    root.addHandler(fh)
    root.addHandler(sh)


# ---------------------------------------------------------------------------
# Log file management
# ---------------------------------------------------------------------------

def copy_logs_to_dir(log_dir: Path, copies: dict) -> None:
    """
    Copy log files into log_dir.

    Parameters
    ----------
    log_dir : destination directory (created if missing)
    copies  : {dest_filename: src_path_str_or_Path, ...}

    Skips any source file that does not exist (e.g. never written to).
    """
    log_dir.mkdir(exist_ok=True)
    for dest_name, src in copies.items():
        if Path(src).exists():
            shutil.copy2(src, log_dir / dest_name)
