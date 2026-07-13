"""vertical-only off-plane probe - one-off diagnostic, not a unit test."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SIM = ROOT / "simulation"
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(SIM / "tests" / "simtests"))

from arduloop import RateAxisParams
from controller import HeliCyclicController, compute_bz_altitude_hold, compute_rate_cmd_sqrt
from pumping_planner import TensionCommand  # noqa: F401 - documents matching command semantics
from simtest_ic import load_ic
from simtest_runner import PhysicsRunner
from telemetry_csv import TelRow, write_csv
from tests.simtests._rotor_helpers import BODY_Z_SLEW_RATE_RAD_S, dynamics_kwargs, load_default_rotor
from tests.simtests.test_steady_flight_offplane import _with_azimuth_offset

WIND = np.array([0.0, 10.0, 0.0])
DT = 2.5e-3
LOG_DIR = SIM / "logs" / "vertical_only_offplane_probe"


def _wrap_pi(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def _wind_azimuth_rad() -> float:
    return float(math.atan2(WIND[1], WIND[0]))


def _body_z_plane_azimuth(body_z: np.ndarray, fallback_pos: np.ndarray) -> float:
    """Return the tether-plane azimuth implied by current body-z horizontal direction."""
    h = np.asarray(body_z[:2], dtype=float)
    if float(np.linalg.norm(h)) > 1e-9:
        # compute_bz_altitude_hold defines body_z horizontal as -[cos(az), sin(az)].
        return float(math.atan2(-h[1], -h[0]))
    return float(math.atan2(fallback_pos[1], fallback_pos[0]))


def run(*, offset_deg: float = 30.0, duration_s: float = 30.0) -> dict:
    rotor = load_default_rotor()
    mass = float(dynamics_kwargs(rotor)["mass"])
    base_ic = load_ic()
    ic = _with_azimuth_offset(base_ic, math.radians(offset_deg))
    steps = int(round(duration_s / DT))

    runner = PhysicsRunner(rotor, ic, WIND)
    runner._acro = HeliCyclicController(
        rotor,
        
        P=0.67,
        I=0.15,
        D=0.02,
        IMAX=0.30,
        FLTT=40.0,
        FLTE=0.0,
        FLTD=40.0,
    )
    runner._acro._servo.reset(ic.coll_eq_rad)

    target_alt = float(-ic.pos[2])
    target_tension = 300.0
    target_el = float(np.arcsin(np.clip(target_alt / max(float(np.linalg.norm(ic.pos)), 0.1), -1.0, 1.0)))
    col_trim = float(ic.coll_eq_rad)
    alt_i = 0.0
    rest_now = float(ic.rest_length)
    telemetry = []
    tel_every = max(1, round(1.0 / (20.0 * DT)))

    wind_az = _wind_azimuth_rad()
    t_hist = np.zeros(steps)
    az_err_hist = np.zeros(steps)
    pos_hist = np.zeros((steps, 3))
    tension_hist = np.zeros(steps)
    body_z_goal = None
    last_roll_sp = 0.0
    last_pitch_sp = 0.0

    for step in range(steps):
        hub = runner.hub_state
        pos = np.asarray(hub["pos"], dtype=float)
        vel = np.asarray(hub["vel"], dtype=float)
        R = np.asarray(hub["R"], dtype=float)
        body_z = R[:, 2]
        tlen = float(np.linalg.norm(pos))
        orbit_az = float(math.atan2(pos[1], pos[0]))

        t_hist[step] = step * DT
        az_err_hist[step] = math.degrees(_wrap_pi(orbit_az - wind_az))
        pos_hist[step] = pos
        tension_hist[step] = runner.tension_now

        desired_el = float(np.arcsin(np.clip(target_alt / max(tlen, 0.1), -1.0, 1.0)))
        delta_el = float(np.clip(desired_el - target_el, -BODY_Z_SLEW_RATE_RAD_S * DT, BODY_Z_SLEW_RATE_RAD_S * DT))
        target_el += delta_el

        # Vertical-only idea: hold the current body-z horizontal azimuth, and only
        # alter the elevation/gravity-compensation part of the body-z target.
        body_z_az = _body_z_plane_azimuth(body_z, pos)
        body_z_goal = compute_bz_altitude_hold(
            pos,
            target_el,
            target_tension,
            mass,
            az_ref_rad=body_z_az,
        )
        rate_sp = compute_rate_cmd_sqrt(
            body_z,
            body_z_goal,
            R,
            kp=2.5,
            accel_max=4.0,
            dt=DT,
            kd=0.0,
        )
        last_roll_sp = float(rate_sp[0])
        last_pitch_sp = float(rate_sp[1])

        alt_m = float(-pos[2])
        vz_up = float(-vel[2])
        alt_err = target_alt - alt_m
        alt_i = float(np.clip(alt_i + 0.001 * alt_err * DT, -0.28 - col_trim, 0.10 - col_trim))
        col = float(np.clip(col_trim + 0.010 * alt_err - 0.040 * vz_up + alt_i, -0.28, 0.10))

        d_tension = runner.tension_now - target_tension
        rest_now += float(np.clip(0.01 * d_tension, -1.0, 1.0)) * DT

        sr = runner.step(DT, col, last_roll_sp, last_pitch_sp, runner.omega_body, rest_length=rest_now)
        sr["omega_body"] = runner.omega_body

        if step % tel_every == 0:
            telemetry.append(TelRow.from_physics(
                runner,
                sr,
                col,
                WIND,
                body_z_eq=body_z_goal,
                phase="vertical-only",
                tension_feedforward_n=target_tension,
                tension_ic_n=target_tension,
                collective_from_alt_ctrl=col,
                gnd_alt_cmd_m=target_alt,
                roll_sp_rads=last_roll_sp,
                pitch_sp_rads=last_pitch_sp,
            ))

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    write_csv(telemetry, LOG_DIR / "telemetry.csv")

    tail = max(1, round(5.0 / DT))
    return {
        "telemetry": LOG_DIR / "telemetry.csv",
        "az_start_deg": float(az_err_hist[0]),
        "az_final_mean_deg": float(np.mean(az_err_hist[-tail:])),
        "az_final_deg": float(az_err_hist[-1]),
        "cross_start_m": float(pos_hist[0, 0] * -1.0),
        "pos_start": pos_hist[0].tolist(),
        "pos_final": pos_hist[-1].tolist(),
        "tension_min_N": float(np.min(tension_hist)),
        "tension_max_N": float(np.max(tension_hist)),
        "tension_tail_mean_N": float(np.mean(tension_hist[-tail:])),
    }


def main() -> None:
    result = run()
    print("vertical-only off-plane probe")
    print(f"  telemetry: {result['telemetry']}")
    print(
        f"  azimuth error: start={result['az_start_deg']:+.2f} deg  "
        f"final_mean={result['az_final_mean_deg']:+.2f} deg  final={result['az_final_deg']:+.2f} deg"
    )
    print(f"  pos start: {result['pos_start']}")
    print(f"  pos final: {result['pos_final']}")
    print(
        f"  tension: min={result['tension_min_N']:.1f} N  "
        f"max={result['tension_max_N']:.1f} N  tail={result['tension_tail_mean_N']:.1f} N"
    )


if __name__ == "__main__":
    main()
