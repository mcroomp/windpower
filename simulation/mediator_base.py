"""Shared setup helpers for all RAWES mediator subprocesses."""
from __future__ import annotations

import logging
import signal
import sys
from typing import Callable

# Single canonical log format for all mediators.
# Routes to stdout so output is captured when run as a subprocess
# (stack_utils redirects stdout to the log file).
_LOG_FORMAT  = "%(asctime)s %(levelname)-8s %(name)s: %(message)s"
_LOG_DATEFMT = "%H:%M:%S"


def setup_logging(log_level: str = "INFO") -> None:
    """Configure logging for a mediator subprocess.

    force=True re-initialises the root logger even if basicConfig was called
    before this point (e.g. by an importing module).
    """
    logging.basicConfig(
        level   = getattr(logging, log_level.upper(), logging.INFO),
        format  = _LOG_FORMAT,
        datefmt = _LOG_DATEFMT,
        stream  = sys.stdout,
        force   = True,
    )


def install_sigterm_handler() -> Callable[[], bool]:
    """Install a SIGTERM handler that sets a stop flag.

    Returns a no-arg callable that returns True once SIGTERM has been received.
    Use it in the mediator's main loop so the process exits cleanly and the
    finally block (event log close, socket close) always runs::

        is_stopped = install_sigterm_handler()
        while not is_stopped():
            ...
    """
    _state: list[bool] = [False]

    def _handler(_sig: int, _frame: object) -> None:
        _state[0] = True

    signal.signal(signal.SIGTERM, _handler)
    return lambda: _state[0]
