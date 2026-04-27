"""
comms.py — MAVLink communication boundary between ground controller and AP.

Two implementations share the same call pattern:

    VirtualComms     — simtest: latency queue + optional Gaussian noise
    MavlinkComms     — SITL/hardware: real MAVLink via gcs.py

Simtest loop (VirtualComms):

    comms = VirtualComms(latency_s=0.05, alt_noise_m=0.5)
    # 400 Hz physics step:
    comms.inject(t_sim, hub_alt_m)
    # 10 Hz ground step:
    tel     = comms.receive_telemetry(t_sim)
    cmd     = ground.step(t_sim, tension, rest_length,
                          hub_alt_m=tel.hub_alt_m if tel else prev_alt)
    comms.send_command(t_sim, cmd)
    ap_cmd  = comms.poll_ap_command(t_sim)
    if ap_cmd:
        ap.receive_command(ap_cmd, DT_PLANNER)

Stack-test loop (MavlinkComms):

    comms = MavlinkComms(gcs)
    # 10 Hz ground step:
    tel     = comms.receive_telemetry()            # non-blocking poll LOCAL_POSITION_NED
    cmd     = ground.step(t_sim, tension, rest_length,
                          hub_alt_m=tel.hub_alt_m if tel else prev_alt)
    comms.send_command(t_sim, cmd)                 # RAWES_TEN + RAWES_ALT + RAWES_SUB
    # No poll_ap_command — ArduPilot/Lua handles AP side directly.

Latency model (VirtualComms):
    Each injected telemetry sample is tagged with t_sim.  receive_telemetry(t_now)
    returns the most-recent sample where t_injected <= t_now - latency_s.
    send_command tags the outgoing TensionCommand; poll_ap_command(t_now) releases
    it once t_sent + latency_s <= t_now.
    With latency_s=0 (default) every inject is immediately available and every
    command is immediately deliverable — reproduces the current zero-latency model.
"""

from __future__ import annotations

import numpy as np
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pumping_planner import TensionCommand
    from gcs import GCSClient


# ---------------------------------------------------------------------------
# Telemetry packet (AP → ground downlink)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class HubTelemetry:
    """Downlinked hub state received by the ground station."""
    hub_alt_m: float


# ---------------------------------------------------------------------------
# VirtualComms — simtest implementation
# ---------------------------------------------------------------------------

class VirtualComms:
    """
    Simulated MAVLink comms for Python simtests.

    Parameters
    ----------
    latency_s   : one-way latency [s] for both uplink and downlink (default 0)
    alt_noise_m : std-dev of Gaussian noise added to hub_alt_m [m] (default 0)
    rng_seed    : optional seed for reproducible noise
    """

    def __init__(self,
                 latency_s  : float = 0.0,
                 alt_noise_m: float = 0.0,
                 rng_seed   : int | None = None) -> None:
        self._latency     = float(latency_s)
        self._alt_noise   = float(alt_noise_m)
        self._rng         = np.random.default_rng(rng_seed)
        # deque of (t_injected, raw_hub_alt_m)
        self._down_buf: deque[tuple[float, float]] = deque()
        # deque of (t_sent, TensionCommand)
        self._up_buf:   deque[tuple[float, object]] = deque()

    def inject(self, t_sim: float, hub_alt_m: float) -> None:
        """Feed current physics truth into the downlink buffer (call every 400 Hz step)."""
        self._down_buf.append((float(t_sim), float(hub_alt_m)))

    def receive_telemetry(self, t_now: float) -> HubTelemetry | None:
        """
        Return the most-recent downlink sample that has survived the one-way
        latency, or None if nothing has arrived yet.  Discards older samples.
        """
        deadline = float(t_now) - self._latency
        latest   = None
        while self._down_buf and self._down_buf[0][0] <= deadline:
            latest = self._down_buf.popleft()
        if latest is None:
            return None
        _, raw_alt = latest
        if self._alt_noise > 0.0:
            raw_alt += float(self._rng.normal(0.0, self._alt_noise))
        return HubTelemetry(hub_alt_m=raw_alt)

    def send_command(self, t_sim: float, cmd: "TensionCommand") -> None:
        """Queue a TensionCommand for delivery to the AP after one-way latency."""
        self._up_buf.append((float(t_sim), cmd))

    def poll_ap_command(self, t_now: float) -> "TensionCommand | None":
        """
        Return the oldest queued command whose latency has elapsed, or None.
        Call at 10 Hz in the test loop; forward result to ap.receive_command().
        """
        deadline = float(t_now) - self._latency
        if self._up_buf and self._up_buf[0][0] <= deadline:
            _, cmd = self._up_buf.popleft()
            return cmd
        return None


# ---------------------------------------------------------------------------
# MavlinkComms — SITL / hardware implementation
# ---------------------------------------------------------------------------

# Phase string → RAWES_SUB integer (matches rawes_modes.py constants)
_PHASE_TO_SUB: dict[str, int] = {
    "hold":       0,   # PUMP_HOLD
    "reel-out":   1,   # PUMP_REEL_OUT
    "transition": 2,   # PUMP_TRANSITION
    "reel-in":    3,   # PUMP_REEL_IN
}


class MavlinkComms:
    """
    Real MAVLink comms for SITL stack tests and hardware.

    send_command() sends RAWES_TEN + RAWES_ALT + RAWES_SUB via gcs.py.
    receive_telemetry() does a non-blocking poll of LOCAL_POSITION_NED.

    tension_measured_n is not forwarded — rawes.lua's TensionPI runs on
    RAWES_TEN (setpoint) only; the load-cell feedback path exists only in
    the Python TensionApController used by simtests.

    Parameters
    ----------
    gcs : GCSClient  open MAVLink connection (simulation/gcs.py)
    """

    def __init__(self, gcs: "GCSClient") -> None:
        self._gcs = gcs

    def receive_telemetry(self) -> HubTelemetry | None:
        """
        Non-blocking poll.  Returns latest LOCAL_POSITION_NED as HubTelemetry,
        or None if no message is buffered.
        """
        pos = self._gcs.recv_local_position_latest()
        if pos is None:
            return None
        _n, _e, ned_z = pos
        return HubTelemetry(hub_alt_m=float(-ned_z))

    def send_command(self, _t_sim: float, cmd: "TensionCommand") -> None:
        """
        Send tension setpoint, altitude target, and phase substate to the AP.
        t_sim is accepted for API symmetry with VirtualComms but is not used.
        """
        sub = _PHASE_TO_SUB.get(cmd.phase, 0)
        self._gcs.send_named_float("RAWES_TEN", cmd.tension_setpoint_n)
        self._gcs.send_named_float("RAWES_ALT", cmd.alt_m)
        self._gcs.send_named_float("RAWES_SUB", float(sub))
