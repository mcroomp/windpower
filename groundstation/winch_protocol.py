"""
winch_protocol.py -- Wire protocol between the ground-station planner and the
winch node (a separate physical node at the anchor: winch drum + load cell +
co-located anemometer, eventually its own dedicated hardware).

  GCS / planner node  --(WinchCommand: targets only)-->  Winch node
  Winch node          --(WinchTelemetry: sensed only)->  GCS / planner node

The fast tension-governing control loop itself (simulation.winch.
GovernedWinchController, hosted today by simulation.winch_node.
GovernedWinchNode) is NOT part of this protocol -- it stands in for winch-node
firmware that does not exist yet, so it lives in simulation/ alongside the
other not-yet-real-hardware stand-ins.  Only the cable boundary -- these two
dataclasses -- is genuine production wire format, used identically whether
the winch node is real hardware or the simulated stand-in.

No-leak guarantee: the planner can only see a WinchTelemetry, whose fields are
all quantities the drum/anchor hardware physically senses (load cell, drum
encoder, co-located anemometer).  Hub altitude / position / attitude never
cross this boundary.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WinchCommand:
    """Down-link (planner -> winch node): targets only.

    cruise_v        -- commanded cruise reel velocity [m/s], +out / -in / 0=hold
    tension_target  -- governor tension setpoint [N]
    """
    cruise_v:       float
    tension_target: float


@dataclass(frozen=True)
class WinchTelemetry:
    """Up-link (winch node -> planner): winch-measurable quantities only.

    tension_n     -- load cell [N]
    rest_length   -- drum encoder: tether rest length [m]
    speed_ms      -- reel speed [m/s], signed (+out)
    net_energy_j  -- drum mechanical energy, integral of T*v [J]
    wind_ned      -- co-located anemometer reading [NED, m/s]
    """
    tension_n:    float
    rest_length:  float
    speed_ms:     float
    net_energy_j: float
    wind_ned:     tuple
