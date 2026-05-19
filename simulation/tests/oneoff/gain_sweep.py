"""gain_sweep.py — step-response sweep over inner-PID gains.

One-off diagnostic, not a unit test.  Uses the controller-tuning rig
to evaluate (kp, kd) combinations for the inner rate PID at the design
operating point.  Prints overshoot %, settling time, rise time, and
steady-state error per candidate.

Run:
    .venv/Scripts/python.exe simulation/tests/oneoff/gain_sweep.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tests.unit._aero_probe import load_rotor
from tests.unit._controller_rig import probe_step_response


def main() -> None:
    rotor = load_rotor("beaupoil_2026")
    R_hub = np.eye(3)
    candidates = [
        (2.0/3.0, 0.0,   0.0),   # legacy P-only
        (2.0/3.0, 0.0,   0.05),  # legacy + small kd
        (0.3,     0.0,   0.0),
        (0.3,     0.0,   0.05),
        (0.3,     0.2,   0.05),  # add integral
        (0.1,     0.0,   0.0),
    ]
    print(f"{'kp':>6} {'ki':>6} {'kd':>6}  {'overshoot%':>10} {'settle_s':>9} {'rise_s':>7} {'ss_err':>+7} {'diverged':>8}")
    for kp, ki, kd in candidates:
        m = probe_step_response(
            rotor, R_hub, omega_spin=28.0,
            kp_inner=kp, ki_inner=ki, kd_inner=kd, imax_inner=0.5,
            setpoint=1.0, channel="roll", duration_s=2.0,
        )
        print(f"{kp:>6.3f} {ki:>6.3f} {kd:>6.3f}  "
              f"{m.overshoot_pct:>10.1f} {m.settling_time_s:>9.3f} "
              f"{m.rise_time_s:>7.3f} {m.steady_state_error:>+7.3f} {str(m.diverged):>8}")


if __name__ == "__main__":
    main()
