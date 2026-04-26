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

import numpy as np
from frames import build_gps_yaw_frame

# ---------------------------------------------------------------------------
# Environment variable names
# ---------------------------------------------------------------------------

STACK_ENV_FLAG  = "RAWES_RUN_STACK_INTEGRATION"
ARDUPILOT_ENV   = "RAWES_ARDUPILOT_PATH"
SIM_VEHICLE_ENV = "RAWES_SIM_VEHICLE"


# ---------------------------------------------------------------------------
# ParamSetup — declarative parameter table with built-in verification
# ---------------------------------------------------------------------------

class ParamSetup:
    """Declarative ArduPilot parameter table that can set and verify itself.

    Typical usage (MAVLink-settable params)::

        setup = ParamSetup({
            "EK3_SRC1_POSXY": 0,
            "H_RSC_MODE":     1,
            ...
        })
        setup.apply(gcs, log=log)   # set all via MAVLink, then read back every
                                    # value; pytest.fail() on any mismatch

    For params that come from the boot param file (not settable via MAVLink,
    e.g. ARMING_SKIPCHK which silently fails as REAL32), skip the set phase
    and just verify::

        boot_check = ParamSetup({"ARMING_SKIPCHK": 65535, ...})
        boot_check.verify(gcs, log=log)

    Composition::

        combined = base_setup.merge(extra_setup)   # other overrides self
        extended = base_setup.update({"NEW_PARAM": 1.0})
    """

    def __init__(self, params: "dict[str, int | float]"):
        self._params: "dict[str, int | float]" = {
            k: (int(v) if isinstance(v, int) else float(v))
            for k, v in params.items()
        }

    # ── public API ───────────────────────────────────────────────────────────

    def apply(
        self,
        gcs,
        log: "logging.Logger | None" = None,
        set_timeout: float = 5.0,
    ) -> None:
        """Set every param via MAVLink, then verify all values match.

        Calls ``pytest.fail()`` with a full mismatch report if any param did
        not reach its expected value after the set-and-read-back cycle.
        """
        _log = log or logging.getLogger("ParamSetup")
        for name, value in self._params.items():
            ok = gcs.set_param(name, value, timeout=set_timeout)
            _log.info("  %-28s = %-10g  ACK=%s", name, value, ok)
        self.verify(gcs, log=_log)

    def verify(
        self,
        gcs,
        log: "logging.Logger | None" = None,
        tol: float = 1e-4,
        read_timeout: float = 30.0,
    ) -> None:
        """Verify all params via a single PARAM_REQUEST_LIST dump.

        Fetches the full param table once, then checks every expected param
        against it.  Params absent from the dump are reported as unknown to
        ArduPilot (not a timeout artefact).

        Use this (without ``apply``) for params set via the boot param file
        that cannot be reliably set via MAVLink (e.g. ARMING_SKIPCHK INT32).
        """
        import pytest

        _log = log or logging.getLogger("ParamSetup")
        _log.info("verify: fetching full param dump ...")
        all_params = gcs.fetch_all_params(timeout=read_timeout)
        _log.info("verify: got %d params from ArduPilot", len(all_params))

        mismatches: list[str] = []
        for name, expected in sorted(self._params.items()):
            if name not in all_params:
                mismatches.append(
                    f"  {name:<30s}  expected={expected}  got=NOT_IN_DUMP"
                )
                _log.error("verify: %s — not in param dump (unknown param)", name)
            else:
                actual = all_params[name]
                if abs(actual - expected) > tol:
                    mismatches.append(
                        f"  {name:<30s}  expected={expected}  got={actual}"
                    )
                    _log.error(
                        "verify: %s expected=%g got=%g (delta=%g)",
                        name, expected, actual, abs(actual - expected),
                    )
                else:
                    _log.debug("verify: %s = %g OK", name, actual)

        if mismatches:
            # For each NOT_IN_DUMP param, log all known params whose name contains
            # any token from the missing name — helps identify renamed params.
            missing_names = [
                m.split()[0]
                for m in mismatches if "NOT_IN_DUMP" in m
            ]
            if missing_names:
                tokens = {tok for name in missing_names for tok in name.split("_") if len(tok) >= 3}
                _log.info("verify: scanning dump for params containing tokens %s:", sorted(tokens))
                for pname in sorted(all_params):
                    if any(tok in pname for tok in tokens):
                        _log.info("  %s = %g", pname, all_params[pname])
            pytest.fail(
                f"Parameter mismatch after setup"
                f" ({len(mismatches)}/{len(self._params)} wrong):\n"
                + "\n".join(mismatches)
                + "\n\nLikely causes:\n"
                + "  - NOT_IN_DUMP: param name unknown to this ArduPilot build\n"
                + "    (see 'matching prefixes' lines in gcs.log for actual names)\n"
                + "  - Docker EEPROM contamination from a previous sequential test\n"
                + "  - Boot param file value ignored (EEPROM already has a value)\n"
                + "  - MAVLink set_param silently failed (INT32 sent as REAL32)\n"
                + "  - ARMING_SKIPCHK must be set via boot param file, not MAVLink\n"
            )

    def merge(self, other: "ParamSetup") -> "ParamSetup":
        """Return a new ParamSetup combining self and *other* (other wins)."""
        return ParamSetup({**self._params, **other._params})

    def update(self, params: "dict[str, float]") -> "ParamSetup":
        """Return a new ParamSetup with extra *params* merged in (they win)."""
        return ParamSetup({**self._params, **params})

    def write_parm_file(self, path: "Path | str") -> None:
        """Write params to a SITL ``--add-param-file`` text file."""
        lines = [f"{name} {value}\n" for name, value in self._params.items()]
        Path(path).write_text("".join(lines), encoding="utf-8")

    @classmethod
    def from_parm_file(cls, path: "Path | str") -> "ParamSetup":
        """Load a ParamSetup from an existing SITL parm file.

        Comment lines (starting with ``#``) and blank lines are ignored.
        """
        params: "dict[str, int | float]" = {}
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) >= 2:
                try:
                    raw = parts[1]
                    if "." in raw or "e" in raw.lower():
                        params[parts[0]] = float(raw)
                    else:
                        params[parts[0]] = int(raw)
                except ValueError:
                    pass
        return cls(params)

    def as_list(self) -> "list[tuple[str, float]]":
        """Return params as a list of (name, value) tuples (preserves dict order)."""
        return list(self._params.items())

    def __len__(self) -> int:
        return len(self._params)

    def __repr__(self) -> str:
        return f"ParamSetup({len(self._params)} params)"


# ---------------------------------------------------------------------------
# Process helpers
# ---------------------------------------------------------------------------

EXPECTED_ARDUPILOT_VERSION = "4.6.3"


def _check_ardupilot_version(sim_vehicle: Path) -> None:
    """
    Extract the version string embedded in the arducopter-heli binary and assert
    it matches EXPECTED_ARDUPILOT_VERSION.  Uses `strings` on the binary — no
    source file required and reflects exactly what will run.

    Raises RuntimeError with a clear message if the version does not match or
    cannot be determined.
    """
    import re
    arducopter = sim_vehicle.parent.parent.parent / "build" / "sitl" / "bin" / "arducopter-heli"
    if not arducopter.exists():
        raise RuntimeError(
            f"ArduPilot version check failed: binary not found at {arducopter}"
        )
    result = subprocess.run(
        ["strings", str(arducopter)],
        capture_output=True, text=True,
    )
    m = re.search(r"ArduCopter V([\d.]+)", result.stdout)
    if not m:
        raise RuntimeError(
            f"ArduPilot version check failed: could not find version string in {arducopter}"
        )
    actual = m.group(1)
    if actual != EXPECTED_ARDUPILOT_VERSION:
        raise RuntimeError(
            f"ArduPilot version mismatch: expected {EXPECTED_ARDUPILOT_VERSION}, "
            f"got {actual}. Rebuild the Docker image or update EXPECTED_ARDUPILOT_VERSION."
        )


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


def _launch_sitl(
    sim_vehicle: Path,
    log_path: Path,
    add_param_file: "Path | None" = None,
) -> subprocess.Popen:
    # Truncate the ArduCopter terminal log so each test run starts fresh.
    # /tmp/ArduCopter.log accumulates across runs in the container — clearing it
    # here makes it contain only output from this test's SITL instance.
    try:
        open("/tmp/ArduCopter.log", "w").close()
    except OSError:
        pass

    # Always wipe EEPROM before launch — no parameter carryover between tests.
    # Each test writes a complete per-test boot param file (--add-param-file)
    # that is the single source of truth for all ArduPilot parameters.
    # With a fresh EEPROM, boot-file params are applied as the only source;
    # sequential test contamination is impossible.
    eeprom = Path(sim_vehicle).parent.parent.parent / "eeprom.bin"
    if eeprom.exists():
        eeprom.unlink()

    cmd = [
        sys.executable,
        str(sim_vehicle),
        "--vehicle", "ArduCopter",
        "--frame", "heli",
        "--custom-location=51.5074,-0.1278,50,0",
        "--model", "JSON",
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
    )


def _prime_sitl_eeprom(
    sim_vehicle: Path,
    wait_s: float = 6.0,
) -> None:
    """
    Boot SITL briefly with --model heli and a minimal parm file (SCR_ENABLE 1
    only) to flush SCR_ENABLE into eeprom.bin, then kill it.

    Also saves the result to eeprom_pristine.bin so _launch_sitl() can restore
    from it before each test — giving a clean, known-good EEPROM state without
    the accumulation of 20-30 param writes that causes slow startup.

    Uses --model heli (internal physics, no JSON backend) so no sensor
    calibration is written — the real boot's --add-param-file supplies all
    needed EKF/sensor params without stale calibration interference.
    """
    ardupilot_root = Path(sim_vehicle).parent.parent.parent

    # arducopter-heli writes eeprom.bin to its CWD.  When launched via
    # sim_vehicle.py inside Docker, the CWD is the pytest working directory
    # (/rawes/simulation), NOT ardupilot_root.  Match that here.
    eeprom = Path.cwd() / "eeprom.bin"
    if eeprom.exists():
        eeprom.unlink()

    # Boot ArduPilot briefly to write SCR_ENABLE=1 into eeprom.bin.
    # Key observations:
    #   1. copter-heli.parm has "SCR_ENABLE 1"; defaults are written to EEPROM
    #      during the first boot, but Lua does NOT start until the NEXT boot.
    #   2. "Loaded defaults from ..." (and EEPROM init) only happens AFTER a
    #      MAVLink client connects to the serial port — we must open a TCP
    #      socket to port 5850 (instance -I9) to trigger it.
    #   3. Model must be "-M Heli" (capital H; lowercase "heli" is unrecognized,
    #      makes the binary print help and exit with code 1).
    #   4. cwd must be the same as the real SITL (pytest working directory) so
    #      that eeprom.bin is written to the location the real test will find.
    arducopter = ardupilot_root / "build" / "sitl" / "bin" / "arducopter-heli"
    copter_heli_parm = ardupilot_root / "Tools" / "autotest" / "default_params" / "copter-heli.parm"
    cmd = [
        str(arducopter),
        "-M", "Heli",
        "-s", "100",  # 100× speedup
        "-I9",        # instance 9 → serial0 on TCP port 5850 (avoids -I0 conflict)
        "-O", "51.5074,-0.1278,50,0",
        "--defaults", str(copter_heli_parm),
    ]
    devnull = open(os.devnull, "w")
    proc = subprocess.Popen(
        cmd,
        cwd=str(Path.cwd()),  # same CWD as real SITL (/rawes/simulation in Docker)
        stdin=subprocess.DEVNULL,
        stdout=devnull,
        stderr=devnull,
        start_new_session=True,
    )

    # Wait for SITL to open the serial port, then connect to trigger EEPROM init.
    time.sleep(2.0)
    _tcp_touch("127.0.0.1", 5850, duration_s=wait_s - 2.0)

    _terminate_process(proc)
    devnull.close()


def _tcp_touch(host: str, port: int, duration_s: float) -> None:
    """Open a TCP connection for duration_s seconds, then close it.

    Used by _prime_sitl_eeprom: ArduPilot's SITL only writes default params to
    EEPROM after a MAVLink client connects to the serial port.  A bare TCP
    connection (no MAVLink framing) is sufficient to trigger param init.
    """
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1.0)
            sock.connect((host, port))
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            sock.close()
            return
        except OSError:
            time.sleep(0.5)


def _terminate_process(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    import signal
    # Try SIGTERM first, then SIGKILL. Use os.kill (specific PID) rather than
    # os.killpg because processes may not be their own process group leader
    # (e.g. when start_new_session is not set).
    try:
        os.kill(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=10.0)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.kill(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    proc.wait(timeout=5.0)


def _kill_by_port(port: int, proto: str = "tcp") -> None:
    """Kill any process still holding the given port (Linux only).

    proto is "tcp" or "udp".

    sim_vehicle.py spawns arducopter-heli in a different process group, so
    os.killpg() on sim_vehicle.py's PID may leave the SITL binary running.
    This function finds the remaining process via /proc/net/tcp (or udp) and
    kills it.
    """
    import signal as _signal

    # Try fuser first (fast, available when psmisc is installed)
    try:
        subprocess.run(
            ["fuser", "-k", f"{port}/{proto}"],
            timeout=5.0,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # Fallback: parse /proc/net/{proto} to find the inode, then scan /proc/*/fd/
    try:
        hex_port = f"{port:04X}"
        inode = None
        with open(f"/proc/net/{proto}") as f:
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

def make_test_log_dir(sim_dir: Path, test_name: str) -> Path:
    """Create (or wipe and recreate) simulation/logs/{test_name}/.

    Called at the start of every stack test so stale logs from a previous
    run never survive into the current one.
    """
    test_log_dir = sim_dir / "logs" / test_name
    if test_log_dir.exists():
        shutil.rmtree(test_log_dir)
    test_log_dir.mkdir(parents=True, exist_ok=True)
    return test_log_dir


def copy_logs_to_dir(log_dir: Path, copies: dict) -> None:
    """Copy log files into log_dir, skipping any that don't exist yet."""
    log_dir.mkdir(exist_ok=True)
    for dest_name, src in copies.items():
        if Path(src).exists():
            shutil.copy2(src, log_dir / dest_name)


# ---------------------------------------------------------------------------
# Mediator launcher
# ---------------------------------------------------------------------------

def _launch_mediator(
    sim_dir: Path,
    repo_root: Path,
    log_path: Path,
    telemetry_log_path: "str | None" = None,
    events_log_path: "str | None" = None,
    tether_rest_length: "float | None" = None,
    initial_state: "dict | None" = None,
    startup_damp_seconds: "float | None" = None,
    lock_orientation: bool = False,
    run_id: "int | None" = None,
    base_k_ang: "float | None" = None,
    internal_controller: bool = False,
    extra_config: "dict | None" = None,
) -> subprocess.Popen:
    # Lazy import: config lives in simulation/, which must be on sys.path.
    # conftest.py and all test files add simulation/ to sys.path before importing
    # stack_utils, so this import will succeed.
    import config as _mcfg

    # Build config dict from defaults, then apply all overrides
    cfg = _mcfg.defaults()
    if initial_state is not None:
        cfg["pos0"]    = list(initial_state["pos"])
        # vel0 intentionally NOT overridden from initial_state: the kinematic
        # startup ramp needs vel0 from config.py defaults (~0.96 m/s) so the EKF
        # gets a velocity-derived yaw heading from frame 0.
        cfg["R0"]         = build_gps_yaw_frame(np.array(initial_state["R0"], dtype=float).reshape(3, 3)[:, 2]).tolist()
        cfg["omega_spin"] = float(initial_state["omega_spin"])
        if tether_rest_length is None and "rest_length" in initial_state:
            cfg["tether_rest_length"] = float(initial_state["rest_length"])
    if tether_rest_length is not None:
        cfg["tether_rest_length"] = float(tether_rest_length)
    if startup_damp_seconds is not None:
        cfg["startup_damp_seconds"] = float(startup_damp_seconds)
    if base_k_ang is not None:
        cfg["base_k_ang"] = float(base_k_ang)
    cfg["internal_controller"] = internal_controller
    cfg["lock_orientation"]    = lock_orientation
    if extra_config:
        import copy as _copy
        for _k, _v in extra_config.items():
            if _k == "trajectory" and isinstance(_v, dict):
                cfg["trajectory"].update(_v)
                for _sub in ("hold", "deschutter", "landing"):
                    if _sub in _v and isinstance(_v[_sub], dict):
                        _merged = _copy.deepcopy(
                            _mcfg.DEFAULTS["trajectory"].get(_sub, {})
                        )
                        _merged.update(_v[_sub])
                        cfg["trajectory"][_sub] = _merged
            elif _k in _mcfg.DEFAULTS:
                cfg[_k] = _v

    # Apply equilibrium collective AFTER extra_config merge so it isn't overridden.
    # warm_coll_rad seeds TensionPI integral at the exact equilibrium value on
    # kinematic exit — must survive the trajectory.deschutter extra_config merge.
    if initial_state is not None and "stack_coll_eq" in initial_state:
        cfg.setdefault("trajectory", {}).setdefault("deschutter", {})
        cfg["trajectory"]["deschutter"]["warm_coll_rad"] = float(initial_state["stack_coll_eq"])
    # tension_out is intentionally NOT read from initial_state["tension_eq_n"].
    # The equilibrium tension at stack_coll_eq (-0.18 rad) is ~345 N, but the
    # operational reel-out target stays at the config default (200 N) for energy
    # optimization.  The warm_coll_rad=-0.18 already gives the PI 0.10 rad of
    # downward headroom when tension rises above 200 N.

    # Write config to a temp file next to the log
    cfg_path = log_path.parent / (log_path.stem + "_mediator_config.json")
    _mcfg.save(cfg, str(cfg_path))

    cmd = [
        sys.executable,
        str(Path(sim_dir) / "mediator.py"),
        "--sitl-recv-port", "9002",
        "--sitl-send-port", "9003",
        "--log-level", "INFO",
        "--config", str(cfg_path),
    ]
    if telemetry_log_path is not None:
        cmd += ["--telemetry-log", telemetry_log_path]
    if events_log_path is not None:
        cmd += ["--events-log", events_log_path]
    if run_id is not None:
        cmd += ["--run-id", str(run_id)]
    return subprocess.Popen(
        cmd,
        cwd=str(repo_root),
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


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


def _launch_mediator_torque(
    torque_dir: Path,
    repo_root: Path,
    log_path: Path,
    omega_rotor: float,
    profile: str = "constant",
    tail_channel: int = 3,
    startup_hold_s: float = 45.0,
    events_log_path: "str | None" = None,
    startup_yaw_rate_deg_s: float = 0.0,
) -> subprocess.Popen:
    """Launch mediator_torque.py as a subprocess."""
    cmd = [
        sys.executable, str(torque_dir / "mediator_torque.py"),
        "--omega-rotor",       str(omega_rotor),
        "--startup-hold",      str(startup_hold_s),
        "--profile",           profile,
        "--tail-channel",      str(tail_channel),
        "--startup-yaw-rate",  str(startup_yaw_rate_deg_s),
        "--log-level",         "INFO",
    ]
    if events_log_path is not None:
        cmd += ["--events-log", events_log_path]
    return subprocess.Popen(
        cmd,
        cwd=str(repo_root),
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _launch_mediator_static(
    sim_dir: Path,
    repo_root: Path,
    log_path: Path,
    *,
    pos:        "np.ndarray",
    vel:        "np.ndarray",
    rpy:        "np.ndarray",
    accel_body: "np.ndarray",
    gyro:       "np.ndarray",
    port: int = 9002,
    events_log: "Path | None" = None,
) -> subprocess.Popen:
    """Launch mediator_static.py as a subprocess with fixed sensor values.

    The subprocess runs the SITL lockstep loop (recv_servos -> send_state)
    with constant sensor values.  No physics, no threading in the test.
    Used by test_arm_minimal and test_gps_fusion_layers.
    """
    def _fmt(arr) -> "list[str]":
        return [str(float(v)) for v in arr]

    cmd = [
        sys.executable, str(sim_dir / "mediator_static.py"),
        "--pos",   *_fmt(pos),
        "--vel",   *_fmt(vel),
        "--rpy",   *_fmt(rpy),
        "--accel", *_fmt(accel_body),
        "--gyro",  *_fmt(gyro),
        "--port",  str(port),
    ]
    if events_log is not None:
        cmd += ["--events-log", str(events_log)]
    return subprocess.Popen(
        cmd,
        cwd=str(repo_root),
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


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
