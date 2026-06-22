"""ideal position feedback probe - one-off diagnostic, not a unit test."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SIM = ROOT / "simulation"
sys.path.insert(0, str(SIM))

from tests.unit.test_python_outer_loop_isolation import _run_ideal_constant_tether  # noqa: E402


def main() -> None:
    cases = [
        ("pos", dict(pos_perturb=np.array([5.0, 0.0, 0.0]), t_total=40.0)),
        ("vel", dict(vel_perturb=np.array([1.0, 0.0, 0.0]), t_total=40.0)),
        ("pulse", dict(force_pulse=(2.0, 4.0, np.array([20.0, 0.0, 0.0])), t_total=40.0)),
    ]
    gains = []
    for kp in [5.0, 10.0, 20.0, 40.0, 80.0]:
        for kd in [10.0, 20.0, 45.0, 80.0, 120.0]:
            gains.append((kp, kd))

    print("case kp kd final_dist max_dist final_speed max_speed final_pos")
    for case, kwargs in cases:
        rows = []
        for kp, kd in gains:
            r = _run_ideal_constant_tether(kp_pos=kp, kd_pos=kd, **kwargs)
            rows.append((r["final_dist"], r["max_dist"], r["final_speed"], kp, kd, r))
        rows.sort(key=lambda row: (row[0], row[2], row[1]))
        for final_dist, max_dist, final_speed, kp, kd, r in rows[:8]:
            print(
                f"{case:5s} {kp:6.1f} {kd:6.1f} {final_dist:10.3f} "
                f"{max_dist:9.3f} {final_speed:11.3f} {r['max_speed']:9.3f} "
                f"{np.array2string(r['final_pos'], precision=2, suppress_small=True)}"
            )


if __name__ == "__main__":
    main()
