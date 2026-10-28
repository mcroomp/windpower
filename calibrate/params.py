"""Calibration parameter targets and Lua script file operations."""
from __future__ import annotations

import os
from datetime import UTC, datetime
from pathlib import Path

from .constants import (
    LinkHubClient,
    SCRIPTS_DIR,
    _LOG_DIR,
    _AP_BASE_PARM_PATH,
    _CALIBRATION_PARAM_PREFIXES,
    _RAWES_COMMON_PARM_PATH,
    _RAWES_HARDWARE_PARM_PATH,
    load_ap_params,
)
from .hw import _restart_scripting


def _is_calibration_param(name: str) -> bool:
    return any(
        name.startswith(prefix)
        for prefix in _CALIBRATION_PARAM_PREFIXES
    )


def _load_shared_hw_target_params() -> dict[str, float]:
    raw = load_ap_params([
        _AP_BASE_PARM_PATH,
        _RAWES_COMMON_PARM_PATH,
        _RAWES_HARDWARE_PARM_PATH,
    ])
    return {
        name: value
        for name, value in raw.items()
        if not _is_calibration_param(name)
    }


def _load_common_override_target_params() -> dict[str, float]:
    raw = load_ap_params([
        _RAWES_COMMON_PARM_PATH,
        _RAWES_HARDWARE_PARM_PATH,
    ])
    return {
        name: value
        for name, value in raw.items()
        if not _is_calibration_param(name)
    }


_CONFIG_TARGET_PARAMS_ALL: dict[str, float] | None = None
_CONFIG_TARGET_PARAMS_COMMON: dict[str, float] | None = None


def _config_target_params(*, use_all: bool) -> dict[str, float]:
    global _CONFIG_TARGET_PARAMS_ALL, _CONFIG_TARGET_PARAMS_COMMON
    if use_all:
        if _CONFIG_TARGET_PARAMS_ALL is None:
            _CONFIG_TARGET_PARAMS_ALL = _load_shared_hw_target_params()
        return _CONFIG_TARGET_PARAMS_ALL
    if _CONFIG_TARGET_PARAMS_COMMON is None:
        _CONFIG_TARGET_PARAMS_COMMON = _load_common_override_target_params()
    return _CONFIG_TARGET_PARAMS_COMMON


def _list_scripts(session: LinkHubClient) -> None:
    print(f"  Listing {SCRIPTS_DIR} ...")
    entries = session.list_files(SCRIPTS_DIR)
    for entry in entries:
        marker = "D" if entry["is_dir"] else "F"
        suffix = "/" if entry["is_dir"] else f"  ({entry['size']} bytes)"
        print(f"    {marker} {entry['name']}{suffix}")


def _remove_script(session: LinkHubClient, filename: str) -> None:
    remote = f"{SCRIPTS_DIR}/{os.path.basename(filename)}"
    print(f"  Removing {remote} ...")
    session.remove_file(remote)
    print("  [OK] Removed.")


def _upload_script(
    session: LinkHubClient,
    local_path: str,
    restart: bool = True,
) -> None:
    if not os.path.isfile(local_path):
        raise FileNotFoundError(f"Script file not found: {local_path}")
    remote = f"{SCRIPTS_DIR}/{os.path.basename(local_path)}"
    print(f"  Uploading {local_path}")
    print(f"         -> {remote} ...")
    parent, name = SCRIPTS_DIR.rsplit("/", 1)
    entry = next(
        (entry for entry in session.list_files(parent) if entry["name"] == name),
        None,
    )
    if entry is None:
        session.create_directory(SCRIPTS_DIR)
    elif not entry["is_dir"]:
        raise NotADirectoryError(f"Remote script path is not a directory: {SCRIPTS_DIR}")
    written = session.upload_file(local_path, remote)
    print(f"  [OK] Uploaded {written} bytes.")
    if restart:
        _restart_scripting(session)


def _list_dataflash_logs(session: LinkHubClient) -> list[dict[str, object]]:
    logs = session.list_logs()
    if not logs:
        print("  No DataFlash logs found.")
        return []
    print("  ID        Size  UTC time")
    for entry in logs:
        timestamp = datetime.fromtimestamp(int(entry["time_utc"]), tz=UTC)
        print(
            f"  {int(entry['id']):>2}  {int(entry['size']):>10}  "
            f"{timestamp:%Y-%m-%d %H:%M:%S}"
        )
    return logs


def _fetch_dataflash_log(
    session: LinkHubClient,
    *,
    log_id: int | None,
    directory: str | Path = _LOG_DIR,
) -> Path | None:
    if log_id is None:
        logs = session.list_logs()
        if not logs:
            print("  No DataFlash logs found.")
            return None
        log_id = max(int(entry["id"]) for entry in logs)
    destination = Path(directory) / f"dataflash-{log_id}.BIN"
    print(f"  Downloading DataFlash log {log_id} -> {destination} ...")
    path = session.download_log(log_id, destination)
    print(f"  [OK] Downloaded {path.stat().st_size} bytes.")
    return path
