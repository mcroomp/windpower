"""bode_attitude.py — open-loop cyclic→body-rate Bode at the design operating point.

One-off diagnostic, not a unit test.  Uses the controller-tuning rig in
``tests/unit/_controller_rig.py`` to sweep sinusoidal cyclic inputs
through tilt_lat (or tilt_lon) and report gain + phase on both body
axes per frequency.  Cross_ratio = off-axis / on-axis gain; values > 1
mean cyclic produces more motion on the wrong axis than the intended
one (gyroscopic precession dominating).

Run:
    .venv/Scripts/python.exe simulation/tests/oneoff/bode_attitude.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tests.unit._aero_probe import load_rotor
from tests.unit._controller_rig import probe_open_loop_plant


def main() -> None:
    rotor = load_rotor("beaupoil_2026")
    R_hub = np.eye(3)
    frequencies = [0.1, 0.5, 1.0, 2.0, 5.0, 10.0]

    for channel in ("tilt_lat", "tilt_lon"):
        print(f"\n=== open-loop Bode: {channel} ===")
        bode = probe_open_loop_plant(
            rotor, R_hub, omega_spin=28.0,
            input_channel=channel,
            frequencies_hz=frequencies, amplitude=0.05, cycles=8,
        )
        print(f"{'freq':>6} {'gx':>7} {'phx':>7} {'gy':>7} {'phy':>7} {'cross':>7}")
        for s in bode:
            print(f"{s.frequency_hz:>6.2f} {s.gain_x:>7.2f} {s.phase_x_deg:>+7.1f} "
                  f"{s.gain_y:>7.2f} {s.phase_y_deg:>+7.1f} {s.cross_ratio:>7.2f}")


if __name__ == "__main__":
    main()
