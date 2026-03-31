"""
torque/torque_telemetry.py — Telemetry frame definition and I/O for the
counter-torque motor stack tests.

Mirrors the architecture of viz3d/telemetry.py so the same renderer pattern
works for both the full RAWES simulation and the isolated torque tests.

Frame format (JSON array)
--------------------------
Each element is a dict with the keys defined by TorqueTelemetryFrame.
Lists are used for arrays to ensure JSON-serialisable output.

Usage — recording inside a test
---------------------------------
    from torque_telemetry import TorqueTelemetryRecorder
    rec = TorqueTelemetryRecorder(meta={"test": "yaw_regulation"})
    rec.record(t=1.0, psi_deg=0.5, psi_dot_degs=0.3, throttle=0.74,
               omega_axle_rads=28.0, q_bearing_nm=0.14, q_motor_nm=0.14)
    rec.save("logs/torque_telemetry.json")

Usage — loading for visualisation
-----------------------------------
    from torque_telemetry import TorqueJSONSource
    src = TorqueJSONSource("logs/torque_telemetry.json")
    for frame in src.frames():
        print(frame.t, frame.psi_deg)
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Iterator, List, Optional


# ---------------------------------------------------------------------------
# TorqueTelemetryFrame
# ---------------------------------------------------------------------------

@dataclass
class TorqueTelemetryFrame:
    """One snapshot of the hub/motor system state."""

    # Time
    t: float = 0.0                   # simulation time [s]

    # Hub attitude state (from ArduPilot ATTITUDE message, NED → ENU negation)
    psi_deg: float = 0.0             # hub yaw angle [deg]   (ATTITUDE.yaw, negated)
    psi_dot_degs: float = 0.0        # hub yaw rate [deg/s]  (ATTITUDE.yawspeed, negated)
    roll_deg: float = 0.0            # hub roll [deg]         (ATTITUDE.roll, same sign)
    pitch_deg: float = 0.0           # hub pitch [deg]        (ATTITUDE.pitch, same sign)

    # Motor command — actual PWM output from ArduPilot (SERVO_OUTPUT_RAW)
    throttle: float = 0.0            # computed equilibrium (filled by visualiser)
    servo_pwm_us: int = 0            # actual Ch4 or Ch9 PWM from ArduPilot [µs]

    # Physics model state (from model.py, computed by mediator)
    omega_axle_rads: float = 0.0     # axle spin rate [rad/s]
    q_bearing_nm: float = 0.0        # bearing drag torque on hub [N·m]
    q_motor_nm: float = 0.0          # motor reaction torque on hub [N·m]

    # Phase flag
    phase: str = "DYNAMIC"           # "STARTUP" or "DYNAMIC"

    @property
    def psi_rad(self) -> float:
        return math.radians(self.psi_deg)

    @property
    def psi_dot_rads(self) -> float:
        return math.radians(self.psi_dot_degs)

    @property
    def net_torque_nm(self) -> float:
        """Net torque on hub = bearing − motor [N·m]."""
        return self.q_bearing_nm - self.q_motor_nm

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "TorqueTelemetryFrame":
        known = {f.name for f in cls.__dataclass_fields__.values()}
        return cls(**{k: v for k, v in d.items() if k in known})


# ---------------------------------------------------------------------------
# TorqueTelemetryRecorder
# ---------------------------------------------------------------------------

class TorqueTelemetryRecorder:
    """
    Accumulates TorqueTelemetryFrame objects during a test run.

    Shared base for all torque stack tests — every test creates a recorder,
    calls .record() inside its observation loop, and saves at the end.

    Meta dict is stored in the JSON header and can hold any test parameters
    (omega_axle, k_bearing, gear_ratio, pass/fail status, thresholds, etc.).
    """

    def __init__(self, meta: Optional[dict[str, Any]] = None) -> None:
        self._frames: List[TorqueTelemetryFrame] = []
        self._meta: dict = dict(meta or {})

    # ── Recording ─────────────────────────────────────────────────────────

    def record(
        self,
        t: float,
        psi_deg: float,
        psi_dot_degs: float,
        throttle: float = 0.0,
        omega_axle_rads: float = 0.0,
        q_bearing_nm: float = 0.0,
        q_motor_nm: float = 0.0,
        phase: str = "DYNAMIC",
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        servo_pwm_us: int = 0,
    ) -> None:
        """Append one telemetry frame."""
        self._frames.append(TorqueTelemetryFrame(
            t=t,
            psi_deg=psi_deg,
            psi_dot_degs=psi_dot_degs,
            throttle=throttle,
            omega_axle_rads=omega_axle_rads,
            q_bearing_nm=q_bearing_nm,
            q_motor_nm=q_motor_nm,
            phase=phase,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            servo_pwm_us=servo_pwm_us,
        ))

    def add_meta(self, key: str, value: Any) -> None:
        """Add or update a metadata entry (e.g. pass/fail result)."""
        self._meta[key] = value

    # ── Access ────────────────────────────────────────────────────────────

    @property
    def frames(self) -> List[TorqueTelemetryFrame]:
        return list(self._frames)

    @property
    def meta(self) -> dict:
        return dict(self._meta)

    def max_psi_dot_degs(self, t_min: float = 0.0) -> float:
        fs = [f for f in self._frames if f.t >= t_min]
        return max((abs(f.psi_dot_degs) for f in fs), default=0.0)

    # ── Persistence ───────────────────────────────────────────────────────

    def save(self, path: str | Path) -> Path:
        """
        Write telemetry to a JSON file.

        File structure::

            {
              "meta": { ... },
              "frames": [ {frame dict}, ... ]
            }
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        doc = {
            "meta":   self._meta,
            "frames": [f.to_dict() for f in self._frames],
        }
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(doc, fh, indent=2)
        return path


# ---------------------------------------------------------------------------
# TorqueJSONSource
# ---------------------------------------------------------------------------

class TorqueJSONSource:
    """
    Reads a JSON telemetry file written by TorqueTelemetryRecorder.save().

    Usage:
        src = TorqueJSONSource("logs/torque_telemetry.json")
        meta = src.meta
        for frame in src.frames():
            print(frame.t, frame.psi_deg, frame.throttle)
    """

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        self._doc: Optional[dict] = None

    def _load(self) -> None:
        if self._doc is None:
            with open(self._path, encoding="utf-8") as fh:
                self._doc = json.load(fh)

    @property
    def meta(self) -> dict:
        self._load()
        return dict(self._doc.get("meta", {}))

    def frames(self) -> Iterator[TorqueTelemetryFrame]:
        self._load()
        for d in self._doc.get("frames", []):
            try:
                yield TorqueTelemetryFrame.from_dict(d)
            except (KeyError, TypeError):
                continue
