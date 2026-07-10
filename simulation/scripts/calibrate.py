#!/usr/bin/env python3
"""
calibrate.py -- thin entry point; all logic lives in _calibrate/.
"""
import os
import sys

# Ensure simulation/scripts/ is on sys.path so `_calibrate` package is findable.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from _calibrate.repl import main  # noqa: E402

if __name__ == "__main__":
    main()
