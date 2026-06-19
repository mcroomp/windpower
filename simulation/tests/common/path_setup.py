"""Shared import-path setup for tests."""

from __future__ import annotations

import sys
from pathlib import Path


def add_simulation_root() -> Path:
    """Ensure simulation/ is on sys.path and return that path."""
    sim_root = Path(__file__).resolve().parents[2]
    sim_root_str = str(sim_root)
    if sim_root_str not in sys.path:
        sys.path.insert(0, sim_root_str)
    return sim_root
