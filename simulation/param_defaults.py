"""Load ArduPilot parameters from .parm files.

Parameter precedence (later files override earlier files):
1. ArduPilot heli baseline: ``simulation/tests/sitl/copter-heli.parm``
2. RAWES overrides: ``simulation/tests/sitl/rawes_sitl_defaults.parm``

This keeps arduloop aligned to real ArduPilot parameters without hardcoded
controller values in Python.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence


def _resolve_default_param_files() -> list[Path]:
    """Return parameter files in precedence order (base -> overrides)."""
    sim_root = Path(__file__).resolve().parent
    files: list[Path] = []

    ap_base = sim_root / "tests" / "sitl" / "copter-heli.parm"
    if ap_base.exists():
        files.append(ap_base)

    rawes_override = sim_root / "tests" / "sitl" / "rawes_sitl_defaults.parm"
    if rawes_override.exists():
        files.append(rawes_override)

    return files


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


def get_ap_param(
    name: str,
    *,
    params: dict[str, float] | None = None,
    aliases: tuple[str, ...] = (),
) -> float:
    """Get a required ArduPilot parameter value (with optional aliases)."""
    p = params if params is not None else load_ap_params()
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


def load_rate_pid_params(parm_file=None):
    """Load roll-axis rate PID parameters from ArduPilot .parm sources.

    Parameters
    ----------
    parm_file : Path or str, optional
        Path to a single parameter file. If None (default), uses the merged
        precedence chain from :func:`_resolve_default_param_files`.

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
        params = load_ap_params()
    else:
        params = load_ap_params([parm_file])
    return _load_rate_axis("RLL", params)


def make_roll_pitch_params_from_file():
    """Factory for roll-axis rate PID parameters loaded from .parm sources.

    Uses :func:`load_rate_pid_params`, so values come from the merged ArduPilot
    baseline + RAWES override chain by default.
    """
    from arduloop import RateAxisParams
    
    params = load_rate_pid_params()
    return RateAxisParams(
        P=params["P"],
        I=params["I"],
        D=params["D"],
        FF=params["FF"],
        IMAX=params["IMAX"],
        FLTT=params["FLTT"],
        FLTE=params["FLTE"],
        FLTD=params["FLTD"],
        D_FF=params["D_FF"],
        PDMX=params["PDMX"],
        SMAX=params["SMAX"],
        ILMI=params["ILMI"],
    )


def make_yaw_params_from_file():
    """Factory for yaw rate PID parameters loaded from ArduPilot .parm files."""
    from arduloop import RateAxisParams

    params = _load_rate_axis("YAW", load_ap_params())
    return RateAxisParams(
        P=params["P"],
        I=params["I"],
        D=params["D"],
        FF=params["FF"],
        IMAX=params["IMAX"],
        FLTT=params["FLTT"],
        FLTE=params["FLTE"],
        FLTD=params["FLTD"],
        D_FF=params["D_FF"],
        PDMX=params["PDMX"],
        SMAX=params["SMAX"],
        ILMI=params["ILMI"],
    )


def load_attitude_params() -> dict[str, float]:
    """Load outer-loop/swash heli params required by arduloop."""
    p = load_ap_params()
    return {
        "ATC_ANG_RLL_P": get_ap_param("ATC_ANG_RLL_P", params=p),
        "ATC_ANG_PIT_P": get_ap_param("ATC_ANG_PIT_P", params=p),
        "ATC_ANG_YAW_P": get_ap_param("ATC_ANG_YAW_P", params=p),
        "ATC_ACCEL_R_MAX": get_ap_param("ATC_ACC_R_MAX", params=p, aliases=("ATC_ACCEL_R_MAX",)),
        "ATC_ACCEL_P_MAX": get_ap_param("ATC_ACC_P_MAX", params=p, aliases=("ATC_ACCEL_P_MAX",)),
        "ATC_ACCEL_Y_MAX": get_ap_param("ATC_ACC_Y_MAX", params=p, aliases=("ATC_ACCEL_Y_MAX",)),
        "ATC_RATE_R_MAX": p.get("ATC_RATE_R_MAX", 0.0),
        "ATC_RATE_P_MAX": p.get("ATC_RATE_P_MAX", 0.0),
        "ATC_RATE_Y_MAX": p.get("ATC_RATE_Y_MAX", 0.0),
        "ATC_INPUT_TC": get_ap_param("ATC_INPUT_TC", params=p),
        "ATC_HOVR_ROL_TRM": p.get("ATC_HOVR_ROL_TRM", 0.0),
        "ATC_PIRO_COMP": p.get("ATC_PIRO_COMP", 0.0),
        "H_SW_H3_PHANG": p.get("H_SW_H3_PHANG", p.get("H3_PHANG", 0.0)),
    }


def make_simtest_roll_pitch_params():
    """Factory for simtest-specific roll/pitch rate PID parameters.
    
    Simtests use the Python altitude-hold loop which generates smaller rate
    demands than ArduPilot's GUIDED attitude controller. Requires higher P gain
    (0.67) to track tightly. SITL uses lower gains (0.15) because ArduPilot
    produces larger rate targets.
    
    This is the SINGLE SOURCE OF TRUTH for simtest rate PID tuning.
    
    Returns
    -------
    RateAxisParams with proven-working gains for simtests: P=0.67, I=0.15, D=0.02, etc.
    """
    from arduloop import RateAxisParams
    
    return RateAxisParams(
        P=0.67, I=0.15, D=0.02, FF=0.0, IMAX=0.30,
        FLTT=40.0, FLTE=0.0, FLTD=40.0,
    )
