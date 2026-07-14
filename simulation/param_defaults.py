"""Load ArduPilot parameters from .parm files.

Parameter precedence (later files override earlier files):
1. ArduPilot heli baseline: ``simulation/tests/sitl/copter-heli.parm``
2. RAWES common defaults: ``simulation/tests/sitl/rawes_common_defaults.parm``
3. RAWES SITL-only overrides: ``simulation/tests/sitl/rawes_sitl_defaults.parm``

This keeps arduloop aligned to real ArduPilot parameters without hardcoded
controller values in Python.
"""

from __future__ import annotations

import re
import yaml
from functools import lru_cache
from pathlib import Path
from typing import Sequence
from arduloop.params import RateAxisParams


def _resolve_default_param_files() -> list[Path]:
    """Return parameter files in precedence order (base -> overrides)."""
    sim_root = Path(__file__).resolve().parent
    files: list[Path] = []

    ap_base = sim_root / "tests" / "sitl" / "copter-heli.parm"
    if ap_base.exists():
        files.append(ap_base)

    rawes_common = sim_root / "tests" / "sitl" / "rawes_common_defaults.parm"
    if rawes_common.exists():
        files.append(rawes_common)

    rawes_sitl_only = sim_root / "tests" / "sitl" / "rawes_sitl_defaults.parm"
    if rawes_sitl_only.exists():
        files.append(rawes_sitl_only)

    return files


def _resolve_rawes_lua_file() -> Path:
    """Return the canonical rawes.lua script path."""
    sim_root = Path(__file__).resolve().parent
    return sim_root / "scripts" / "rawes.lua"


def load_rawes_lua_constants(required_names: Sequence[str]) -> dict[str, float]:
    """Load named numeric constants from rawes.lua.

    Raises
    ------
    ValueError
        If one or more required constants are missing.
    """
    text = _resolve_rawes_lua_file().read_text(encoding="utf-8")
    values: dict[str, float] = {}
    missing: list[str] = []

    for name in required_names:
        m = re.search(
            rf"(?m)^\s*{re.escape(name)}\s*=\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))",
            text,
        )
        if m is not None:
            values[name] = float(m.group(1))
        else:
            missing.append(name)

    if missing:
        raise ValueError(
            "Missing required constants in rawes.lua: " + ", ".join(missing)
        )

    return values


def load_ap_params(param_files: Sequence[Path | str] | None = None) -> dict[str, float]:
    """Load merged ArduPilot params from one or more .parm files.

    Later files override earlier ones.
    """
    if param_files is None:
        param_files = _resolve_default_param_files()

    files = [Path(p) for p in (param_files or ())]
    if not files:
        raise FileNotFoundError("No ArduPilot .parm files found to load parameters")

    params: dict[str, float] = {}
    for parm_file in files:
        if not parm_file.exists():
            continue
        with open(parm_file, encoding="utf-8") as f:
            for raw_line in f:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if "#" in line:
                    line = line.split("#", 1)[0].strip()
                parts = line.split()
                if len(parts) < 2:
                    continue
                key, value_str = parts[0], parts[1]
                try:
                    params[key] = float(value_str)
                except ValueError:
                    continue
    return params


@lru_cache(maxsize=1)
def _load_default_ap_params_cached() -> dict[str, float]:
    """Load merged default .parm chain once per process."""
    return load_ap_params(tuple(_resolve_default_param_files()))


def get_ap_param(
    name: str,
    *,
    params: dict[str, float] | None = None,
    aliases: tuple[str, ...] = (),
) -> float:
    """Get a required ArduPilot parameter value (with optional aliases)."""
    p = params if params is not None else _load_default_ap_params_cached()
    if name in p:
        return p[name]
    for alias in aliases:
        if alias in p:
            return p[alias]
    searched = ", ".join((name, *aliases))
    raise ValueError(f"Missing required ArduPilot parameter(s): {searched}")


def _load_rate_axis(prefix: str, params: dict[str, float]) -> dict[str, float]:
    return {
        "P": get_ap_param(f"ATC_RAT_{prefix}_P", params=params),
        "I": get_ap_param(f"ATC_RAT_{prefix}_I", params=params),
        "D": get_ap_param(f"ATC_RAT_{prefix}_D", params=params),
        "FF": get_ap_param(f"ATC_RAT_{prefix}_FF", params=params),
        "IMAX": get_ap_param(f"ATC_RAT_{prefix}_IMAX", params=params),
        "FLTT": get_ap_param(f"ATC_RAT_{prefix}_FLTT", params=params),
        "FLTE": get_ap_param(f"ATC_RAT_{prefix}_FLTE", params=params),
        "FLTD": get_ap_param(f"ATC_RAT_{prefix}_FLTD", params=params),
        "D_FF": params.get(f"ATC_RAT_{prefix}_D_FF", 0.0),
        "PDMX": params.get(f"ATC_RAT_{prefix}_PDMX", 0.0),
        "SMAX": params.get(f"ATC_RAT_{prefix}_SMAX", 0.0),
        "ILMI": params.get(f"ATC_RAT_{prefix}_ILMI", 0.0),
    }


def load_rate_pid_params(parm_file=None, axis: str = "RLL"):
    """Load a single rate-axis PID parameter set from ArduPilot .parm sources.

    Every consumer (unit tests, simtests, and SITL) loads the *same* merged
    ``copter-heli.parm`` + ``rawes_common_defaults.parm`` +
    ``rawes_sitl_defaults.parm`` chain, so the gains are identical across paths.

    Parameters
    ----------
    parm_file : Path or str, optional
        Path to a single parameter file. If None (default), uses the merged
        precedence chain from :func:`_resolve_default_param_files`.
    axis : str
        ArduPilot rate-axis prefix: ``"RLL"`` (roll), ``"PIT"`` (pitch), or
        ``"YAW"``. Defaults to ``"RLL"``.

    Returns
    -------
    dict with keys:
        P, I, D, FF, IMAX, FLTT, FLTE, FLTD plus optional D_FF/PDMX/SMAX/ILMI
    All values are floats.

    Raises
    ------
    FileNotFoundError
        If the parameter file is not found.
    ValueError
        If any required parameter is missing from the file.
    """
    if parm_file is None:
        params = _load_default_ap_params_cached()
    else:
        params = load_ap_params([parm_file])
    return _load_rate_axis(axis, params)


def _make_axis_params_from_file(axis: str) -> RateAxisParams:
    """Build a :class:`RateAxisParams` for ``axis`` from the merged .parm chain."""
    return RateAxisParams.from_ap_dict(_load_default_ap_params_cached(), axis)


def make_roll_params_from_file():
    """Factory for the roll rate PID (``ATC_RAT_RLL_*``) from the .parm chain."""
    return _make_axis_params_from_file("RLL")


# Backwards-compatible alias. Historically this returned the roll axis and was
# (incorrectly) reused for pitch too; pitch now has its own factory below.
make_roll_pitch_params_from_file = make_roll_params_from_file


def make_pitch_params_from_file():
    """Factory for the pitch rate PID (``ATC_RAT_PIT_*``) from the .parm chain."""
    return _make_axis_params_from_file("PIT")


def make_yaw_params_from_file():
    """Factory for the yaw rate PID (``ATC_RAT_YAW_*``) from the .parm chain."""
    return _make_axis_params_from_file("YAW")


def _resolve_default_rotor_yaml() -> Path:
    """Return the path to the project default rotor YAML (beaupoil_2026)."""
    return Path(__file__).resolve().parent / "rotor_definitions" / "beaupoil_2026.yaml"


@lru_cache(maxsize=1)
def load_collective_phys_range() -> tuple[float, float]:
    """Return (col_at_thrust_0, col_at_thrust_1) in radians.

    Derives the collective blade-pitch angle at thrust=0 and thrust=1 by
    combining the physical calibration from the rotor YAML (control.col_min_rad /
    control.col_max_rad) with the ArduPilot H_COL_MIN / H_COL_MAX servo limits.
    The result is the single source of truth for the thrust→collective_rad
    mapping used by the Python physics runner.

    With the standard parm values H_COL_MIN=1000, H_COL_MAX=2000 (full servo
    span), col_at_thrust_0 = col_min_rad and col_at_thrust_1 = col_max_rad.
    """
    with open(_resolve_default_rotor_yaml(), encoding="utf-8") as _f:
        _rotor_data = yaml.safe_load(_f)
    _ctrl = _rotor_data["control"]
    col_min_rad = float(_ctrl["col_min_rad"])
    col_max_rad = float(_ctrl["col_max_rad"])
    params = _load_default_ap_params_cached()
    h_col_min_us = params.get("H_COL_MIN", 1000.0)
    h_col_max_us = params.get("H_COL_MAX", 2000.0)
    h_col_min_norm = (h_col_min_us - 1000.0) / 1000.0
    h_col_max_norm = (h_col_max_us - 1000.0) / 1000.0
    span = col_max_rad - col_min_rad
    return (
        col_min_rad + h_col_min_norm * span,
        col_min_rad + h_col_max_norm * span,
    )


def thrust_to_coll_rad(thrust: float) -> float:
    col_min, col_max = load_collective_phys_range()
    return col_min + float(thrust) * (col_max - col_min)


def coll_rad_to_thrust(collective_rad: float) -> float:
    """Map physical collective [rad] back to normalized thrust [0..1]."""
    col_min, col_max = load_collective_phys_range()
    span = col_max - col_min
    if abs(span) < 1e-12:
        return 0.0
    thrust = (float(collective_rad) - col_min) / span
    return float(max(0.0, min(1.0, thrust)))
