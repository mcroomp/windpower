"""Shared setup helpers for all RAWES mediator subprocesses."""
from __future__ import annotations

import logging
import signal
import sys
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from mediator_events import MediatorEventLog
    from sitl_interface import SITLInterface

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


def run_lockstep(
    sitl: "SITLInterface",
    step_fn: "Callable[[object, float], dict]",
    is_stopped: "Callable[[], bool]",
    *,
    log: "logging.Logger",
    ev: "MediatorEventLog | None" = None,
    heartbeat_fields: "Callable[[], dict] | None" = None,
) -> int:
    """Core SITL lockstep loop shared by all mediators.

    Receives one servo packet from SITL, calls step_fn(servos), and sends the
    returned state dict back to SITL. Repeats until is_stopped() returns True
    or the watchdog fires.

    step_fn(servos) -> dict
        ``servos`` is the normalised [-1, 1] servo array from SITLInterface.
        Mediators that need raw PWM µs values should close over the
        SITLInterface object and read ``sitl.last_pwm_raw`` directly.
        Returns a dict with keys: pos_ned, vel_ned, rpy_rad, accel_body,
        gyro_body (and optionally rpm_rad_s, battery_voltage).

    Heartbeat: every ~1 sim-second, logs ``sim_t`` to ``log`` (INFO) and
    writes a heartbeat event to ``ev`` if provided.  The event always contains
    ``t_sim``; if ``heartbeat_fields`` is given it is called each heartbeat and
    its returned dict is merged in (use this for mediator-specific fields like
    ``phase``, ``psi_dot_deg_s``, ``throttle``).

    Returns the total number of frames sent.
    """
    frame = 0
    last_hb = -1.0
    while not is_stopped():
        servos = sitl.recv_servos()
        if servos is None:
            if is_stopped():
                break
            log.error("SITL servo watchdog expired -- ArduPilot stopped responding")
            sys.exit(1)
        t = sitl.sim_now()
        state = step_fn(servos, t)
        sitl.send_state(**state)
        frame += 1
        if t - last_hb >= 1.0:
            last_hb = t
            log.info("sim_t=%.1f s", t)
            if ev is not None:
                extra = heartbeat_fields() if heartbeat_fields is not None else {}
                ev.write("heartbeat", t_sim=round(t, 1), **extra)
    return frame
