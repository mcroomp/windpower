"""
calibrate/params.py -- Param config, log download, script management.
"""
from __future__ import annotations

import os
import time

from pymavlink import mavutil

try:
    from pymavlink import mavftp as _mavftp_mod
    _HAS_MAVFTP = True
except ImportError:
    _HAS_MAVFTP = False

from .constants import (
    RawesGCS,
    _SIM_DIR,
    _AP_BASE_PARM_PATH, _RAWES_COMMON_PARM_PATH,
    _CALIBRATION_PARAM_PREFIXES,
    SCRIPTS_DIR,
    load_ap_params,
)
from .hw import _restart_scripting
from .util import _parse_flags


# ---------------------------------------------------------------------------
# Calibration param filter
# ---------------------------------------------------------------------------

def _is_calibration_param(name: str) -> bool:
    for prefix in _CALIBRATION_PARAM_PREFIXES:
        if name.startswith(prefix):
            return True
    return False


def _load_shared_hw_target_params() -> dict[str, float]:
    """Load hardware target params from shared sources, excluding SITL-only overrides."""
    raw = load_ap_params([_AP_BASE_PARM_PATH, _RAWES_COMMON_PARM_PATH])
    return {k: v for k, v in raw.items() if not _is_calibration_param(k)}


def _load_common_override_target_params() -> dict[str, float]:
    """Load only params explicitly overridden in rawes_common_defaults.parm."""
    raw = load_ap_params([_RAWES_COMMON_PARM_PATH])
    return {k: v for k, v in raw.items() if not _is_calibration_param(k)}


# Computed once at import time (reads .parm files from disk).
_CONFIG_TARGET_PARAMS_ALL    = _load_shared_hw_target_params()
_CONFIG_TARGET_PARAMS_COMMON = _load_common_override_target_params()


# ---------------------------------------------------------------------------
# Log download
# ---------------------------------------------------------------------------

def _download_latest_log(session: RawesGCS, dest_dir: str = ".") -> "str | None":
    """
    Download the most recent dataflash log using the MAVLink log-download protocol
    (LOG_REQUEST_LIST / LOG_REQUEST_DATA / LOG_DATA).

    Avoids pymavlink mavftp which uses /tmp/ internally (broken on Windows).
    Sends LOG_REQUEST_DATA with count=0xFFFFFFFF, reassembles out-of-order
    LOG_DATA packets, re-requests any gaps, and writes a .BIN file to dest_dir.
    """
    mav = session._mav
    sys_id  = session._target_system
    comp_id = session._target_component

    # -- 1. Enumerate logs ----------------------------------------------------
    print("  Requesting log list ...")
    mav.mav.log_request_list_send(sys_id, comp_id, 0, 0xFFFF)
    entries: dict[int, object] = {}
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        msg = session._recv(type="LOG_ENTRY", blocking=True, timeout=0.5)
        if msg is None:
            continue
        entries[msg.id] = msg
        if msg.id == msg.last_log_num:
            break
    if not entries:
        print("  No log entries returned.")
        return None

    latest   = entries[max(entries)]
    log_id   = latest.id
    log_size = latest.size
    print(f"  Latest log: id={log_id}  size={log_size} bytes")

    # -- 2. Download ----------------------------------------------------------
    os.makedirs(dest_dir, exist_ok=True)
    local = os.path.join(dest_dir, f"{log_id:08d}.BIN")

    def _request(ofs: int, count: int) -> None:
        mav.mav.log_request_data_send(sys_id, comp_id, log_id, ofs, count)

    print(f"  Downloading log {log_id} -> {local} ...")
    data = bytearray(log_size)
    received: set[int] = set()   # set of offsets written
    write_ptr = 0                # contiguous bytes confirmed written
    pending: dict[int, bytes] = {}  # out-of-order chunks: ofs -> bytes

    _request(0, 0xFFFFFFFF)
    last_progress = -1
    deadline = time.monotonic() + 120.0

    while write_ptr < log_size and time.monotonic() < deadline:
        msg = session._recv(type="LOG_DATA", blocking=True, timeout=2.0)
        if msg is None:
            _request(write_ptr, log_size - write_ptr)
            continue
        if msg.id != log_id or msg.count == 0:
            continue
        chunk = bytes(msg.data[:msg.count])
        ofs   = msg.ofs
        end   = ofs + len(chunk)
        if end <= log_size:
            data[ofs:end] = chunk
            pending[ofs] = chunk

        # Advance write_ptr over contiguous chunks
        while write_ptr in pending:
            write_ptr += len(pending.pop(write_ptr))

        pct = write_ptr * 100 // log_size
        if pct != last_progress and pct % 5 == 0:
            print(f"    {pct}%  ({write_ptr}/{log_size} bytes)", end="\r")
            last_progress = pct

    mav.mav.log_request_end_send(sys_id, comp_id)
    print()

    if write_ptr < log_size:
        print(f"  [WARN] Incomplete: {write_ptr}/{log_size} bytes")

    with open(local, "wb") as fh:
        fh.write(data[:write_ptr])

    size = os.path.getsize(local)
    print(f"  [OK] {size} bytes -> {local}")
    return local


def _cmd_logs(session: RawesGCS, args: list[str]) -> None:
    """logs [list] [fetch [--id N] [--dir PATH]]

    list          Print all dataflash log entries (id, size, date) the FC
                  reports via LOG_ENTRY.  No download.
    fetch         Download one log via the MAVLink LOG_REQUEST_DATA protocol.
                  --id N    log id to download (default: latest)
                  --dir D   destination directory (default: simulation/logs/calibrate)
    """
    schema = {"--id": "int", "--dir": "str"}
    sub = args[0].lower() if args else "fetch"
    rest = args[1:] if args else []
    try:
        _pos, flags = _parse_flags(rest, schema)
    except ValueError as e:
        print(f"  Error: {e}"); return

    mav     = session._mav
    sys_id  = session._target_system
    comp_id = session._target_component

    # ── enumerate logs ────────────────────────────────────────────────────────
    print("  [DBG] sending LOG_REQUEST_LIST (start=0 end=0xFFFF) ...")
    mav.mav.log_request_list_send(sys_id, comp_id, 0, 0xFFFF)
    entries: dict[int, object] = {}
    deadline = time.monotonic() + 10.0
    packets_seen = 0
    while time.monotonic() < deadline:
        msg = session._recv(type="LOG_ENTRY", blocking=True, timeout=0.5)
        if msg is None:
            print(f"  [DBG] timeout waiting for LOG_ENTRY (got {packets_seen} so far) "
                  f"-- {10.0 - (time.monotonic() - (deadline - 10.0)):.1f}s left")
            continue
        packets_seen += 1
        entries[msg.id] = msg
        print(f"  [DBG] LOG_ENTRY id={msg.id}  size={msg.size}  "
              f"last_log_num={msg.last_log_num}  "
              f"num_logs={msg.num_logs}")
        if msg.id == msg.last_log_num:
            print(f"  [DBG] received final entry (id == last_log_num={msg.last_log_num})")
            break

    if not entries:
        print("  [FAIL] No LOG_ENTRY messages received.")
        print("  Check: FC armed? dataflash enabled? MAVLink log stream active?")
        return

    # ── print list ────────────────────────────────────────────────────────────
    print(f"\n  {len(entries)} log(s) on FC:")
    for eid in sorted(entries):
        e = entries[eid]
        print(f"    id={eid:4d}  size={e.size:>10,} bytes")

    if sub == "list":
        return

    # ── select log to fetch ───────────────────────────────────────────────────
    if "--id" in flags:
        wanted = int(flags["--id"])
        if wanted not in entries:
            print(f"  [FAIL] Log id={wanted} not found (available: {sorted(entries)})")
            return
        entry = entries[wanted]
    else:
        entry = entries[max(entries)]

    log_id   = entry.id
    log_size = entry.size
    dest_dir = str(flags.get("--dir", os.path.join("simulation", "logs", "calibrate")))

    print(f"\n  Fetching log id={log_id}  size={log_size:,} bytes -> {dest_dir}/")
    if log_size == 0:
        print("  [FAIL] Log size is 0 -- nothing to download.")
        return

    os.makedirs(dest_dir, exist_ok=True)
    local = os.path.join(dest_dir, f"{log_id:08d}.BIN")

    def _request(ofs: int, count: int) -> None:
        print(f"  [DBG] LOG_REQUEST_DATA id={log_id} ofs={ofs} count={count}")
        mav.mav.log_request_data_send(sys_id, comp_id, log_id, ofs, count)

    data     = bytearray(log_size)
    pending: dict[int, bytes] = {}
    write_ptr = 0
    retries   = 0
    last_pct  = -1
    deadline  = time.monotonic() + 180.0

    _request(0, 0xFFFFFFFF)

    while write_ptr < log_size and time.monotonic() < deadline:
        msg = session._recv(type="LOG_DATA", blocking=True, timeout=2.0)
        if msg is None:
            retries += 1
            print(f"  [DBG] timeout waiting for LOG_DATA  write_ptr={write_ptr}  "
                  f"retries={retries}")
            _request(write_ptr, log_size - write_ptr)
            continue
        if msg.id != log_id or msg.count == 0:
            print(f"  [DBG] skipping LOG_DATA id={msg.id} count={msg.count}")
            continue
        chunk = bytes(msg.data[:msg.count])
        ofs   = msg.ofs
        end   = ofs + len(chunk)
        if end <= log_size:
            data[ofs:end] = chunk
            pending[ofs]  = chunk
        while write_ptr in pending:
            write_ptr += len(pending.pop(write_ptr))
        pct = write_ptr * 100 // log_size
        if pct != last_pct and pct % 10 == 0:
            print(f"    {pct:3d}%  ({write_ptr:,}/{log_size:,} bytes)", end="\r")
            last_pct = pct

    mav.mav.log_request_end_send(sys_id, comp_id)
    print()

    if write_ptr < log_size:
        print(f"  [WARN] Incomplete transfer: {write_ptr:,}/{log_size:,} bytes "
              f"({retries} retries)")
    with open(local, "wb") as fh:
        fh.write(data[:write_ptr])
    size = os.path.getsize(local)
    print(f"  [OK] {size:,} bytes -> {local}")


# ---------------------------------------------------------------------------
# Script management via MAVLink FTP
# ---------------------------------------------------------------------------

def _list_scripts(session: RawesGCS) -> None:
    """List files in /APM/scripts via MAVLink FTP."""
    if not _HAS_MAVFTP:
        print("  ERROR: pymavlink.mavftp not available -- upgrade pymavlink")
        return
    print(f"  Listing {SCRIPTS_DIR} ...")
    try:
        ftp = _mavftp_mod.MAVFTP(
            session._mav,
            target_system=session._target_system,
            target_component=session._target_component,
        )
        result = ftp.cmd_list([SCRIPTS_DIR])
        if ftp.list_result:
            for entry in ftp.list_result:
                if entry.is_dir:
                    print(f"    D {entry.name}/")
                else:
                    print(f"    F {entry.name}  ({entry.size_b} bytes)")
        else:
            print(f"  (no files found or directory does not exist; result={result})")
    except Exception as exc:
        print(f"  FTP list failed: {exc}")


def _remove_script(session: RawesGCS, filename: str) -> None:
    """Remove a single file from /APM/scripts via MAVLink FTP."""
    if not _HAS_MAVFTP:
        print("  ERROR: pymavlink.mavftp not available -- upgrade pymavlink")
        return
    remote = f"{SCRIPTS_DIR}/{os.path.basename(filename)}"
    print(f"  Removing {remote} ...")
    try:
        ftp = _mavftp_mod.MAVFTP(
            session._mav,
            target_system=session._target_system,
            target_component=session._target_component,
        )
        result = ftp.cmd_rm([remote])
        if result.error_code == 0:
            print(f"  [OK] Removed.")
        else:
            print(f"  [FAIL] {result}")
    except Exception as exc:
        print(f"  FTP operation failed: {exc}")


def _upload_script(session: RawesGCS, local_path: str,
                   restart: bool = True) -> None:
    """
    Upload a Lua script to /APM/scripts/ via MAVLink FTP.

    local_path  : path to .lua file on this machine
    restart     : if True, toggle SCR_ENABLE after upload to reload scripts
    """
    if not _HAS_MAVFTP:
        print("  ERROR: pymavlink.mavftp not available -- upgrade pymavlink")
        print("  Alternative: use Mission Planner -> Config -> MAVFtp")
        return
    if not os.path.isfile(local_path):
        print(f"  ERROR: file not found: {local_path}")
        return

    remote_path = f"{SCRIPTS_DIR}/{os.path.basename(local_path)}"
    print(f"  Uploading {local_path}")
    print(f"         -> {remote_path} ...")

    for attempt in range(1, 4):
        try:
            time.sleep(1.0)  # let connection settle before FTP
            ftp = _mavftp_mod.MAVFTP(
                session._mav,
                target_system=session._target_system,
                target_component=session._target_component,
            )
            put_ret = ftp.cmd_put([local_path, remote_path])
            if put_ret.error_code != 0:
                print(f"  Attempt {attempt}: cmd_put rejected: {put_ret}")
                continue
            # Pump the message loop until all write blocks are ACKed and the
            # session is terminated.  timeout must be > idle_detection_time (3.7 s).
            result = ftp.process_ftp_reply('CreateFile', timeout=30)
            if result.error_code == 0:
                print(f"  Upload OK (attempt {attempt}).")
                break
            print(f"  Attempt {attempt}: transfer incomplete: {result}")
        except Exception as exc:
            print(f"  Attempt {attempt} failed: {exc}")
    else:
        print("  WARNING: upload may not have completed -- verify with Mission Planner MAVFtp.")
        return

    if restart:
        _restart_scripting(session)
