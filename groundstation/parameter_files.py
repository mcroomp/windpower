"""Dependency-free loading of ArduPilot parameter files."""

from collections.abc import Sequence
from pathlib import Path


def load_parameter_files(param_files: Sequence[Path | str]) -> dict[str, float]:
    """Merge parameter files in order, with later values overriding earlier ones."""
    files = [Path(p) for p in param_files]
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
