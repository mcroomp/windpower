#!/usr/bin/env python3
"""
calibrate/__main__.py -- thin entry point; all logic lives in calibrate/repl.py.

Run as: python -m calibrate [--port P] [--baud B] [--force] <verb> [args...]
"""
from calibrate.repl import main

if __name__ == "__main__":
    main()
