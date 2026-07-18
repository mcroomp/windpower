"""calibrate -- interactive hardware calibration/bench CLI (MAVLink over USB/SiK).

Run as: python -m calibrate [--port P] [--baud B] [--force] <verb> [args...]
"""
from .repl import main

__all__ = ["main"]
