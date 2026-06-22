"""constant tether disturbance probe - one-off diagnostic, not a unit test."""
from __future__ import annotations

import sys
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SIM = ROOT / "simulation"
sys.path.insert(0, str(SIM))

from tests.unit.test_full_loop_stability import (  # noqa: E402
    _KD_POS,
    _KP_POS,
    _T_SETTLE,
    _run_with_constant_tether_force,
)


def main() -> None:
    cases = [
        ("pos_offset", dict(pos_perturb=np.array([5.0, 0.0, 0.0]))),
        ("vel_kick", dict(vel_perturb=np.array([1.0, 0.0, 0.0]))),
        ("force_impulse", dict(force_pulse=(2.0, 4.0, np.array([20.0, 0.0, 0.0])))),
    ]
    gains = [
        (5.0, 10.0),
        (5.0, 20.0),
        (5.0, 45.0),
        (10.0, 20.0),
        (20.0, 45.0),
        (_KP_POS, _KD_POS),
    ]
    caps = [math.radians(10.0), math.radians(20.0), math.radians(30.0)]
    print("case kp kd cap final_n max_n final_dist max_dist final_speed max_speed "
          "max_tether_ang max_target_ang max_tilt max_rate max_omega final_pos")
    for name, kwargs in cases:
        for kp, kd in gains:
            for cap in caps:
                r = _run_with_constant_tether_force(
                    elevation_deg=30.0,
                    tether_tension_n=300.0,
                    t_total=_T_SETTLE,
                    kp_pos=kp,
                    kd_pos=kd,
                    pos_max_tilt_rad=cap,
                    return_history=False,
                    **kwargs,
                )
                print(
                    f"{name:13s} {kp:6.1f} {kd:6.1f} {np.degrees(cap):5.1f} "
                    f"{r.get('final_north', float('nan')):8.3f} "
                    f"{r.get('max_abs_north', float('nan')):7.3f} "
                    f"{r['final_dist']:10.3f} {r['max_dist']:9.3f} "
                    f"{r['final_speed']:11.3f} {r['max_speed']:9.3f} "
                    f"{np.degrees(r.get('max_angle', float('nan'))):14.3f} "
                    f"{np.degrees(r.get('max_target_angle', float('nan'))):14.3f} "
                    f"{r.get('max_tilt', float('nan')):8.3f} "
                    f"{r.get('max_rate_sp', float('nan')):8.3f} "
                    f"{r.get('max_omega_xy', float('nan')):9.3f} "
                    f"{np.array2string(r['final_pos'], precision=2, suppress_small=True)}"
                )


if __name__ == "__main__":
    main()
