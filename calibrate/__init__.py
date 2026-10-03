"""calibrate -- HTTP client for hardware calibration through LinkHub.

Run as: python -m calibrate [--server URL] [--force] <verb> [args...]
"""
def main() -> None:
    from .repl import main as repl_main

    repl_main()

__all__ = ["main"]
