"""
test_steady_flight_ic_angle_only.py -- Python-only steady flight using only
arduloop GUIDED angle API at IC attitude.

Purpose
-------
Provide a minimal variant that does not run the Python RAWES mode logic
(elevation tracking, altitude PID, tension feedforward updates, etc.).

Control path in this test:
  - Constant collective = IC collective
  - Constant target attitude = IC roll/pitch/yaw
  - Set every AP tick via GuidedAttitudeController
    set_target_angle_and_rate_and_throttle(...)

No ground command updates are sent after start.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

from arduloop import (
    GuidedAttitudeController,
    HeliParams,
    make_roll_pitch_params,
    make_yaw_params,
    DataflashEquivRow,
    make_df_equiv_row,
    write_df_equiv_csv,
)
from frames import build_gps_yaw_frame
from telemetry_csv import TelRow, write_csv
from simtest_ic import load_ic
from simtest_runner import PhysicsRunner
from tests.simtests._rotor_helpers import load_default_rotor


_ROTOR = load_default_rotor()
_IC = load_ic()

DT = 1.0 / 400.0
AP_HZ = 50.0
AP_EVERY = max(1, round(1.0 / (AP_HZ * DT)))
WIND = np.array([0.0, 10.0, 0.0])


def _run_angle_ic_only(steps: int = 4000):
    """Run fixed-attitude guided control from IC with zero cyclic servo start."""
    # Match SITL startup attitude convention: preserve IC body_z but remap yaw
    # into a GPS-friendly frame (horizontal body_x).
    R0_gps = build_gps_yaw_frame(np.asarray(_IC.R0, dtype=float)[:, 2])
    ic_run = SimpleNamespace(
        pos=_IC.pos,
        vel=_IC.vel,
        R0=R0_gps,
        rest_length=_IC.rest_length,
        coll_eq_rad=_IC.coll_eq_rad,
        omega_spin=_IC.omega_spin,
        trim_tilt_lon=float(getattr(_IC, "trim_tilt_lon", 0.0)),
        trim_tilt_lat=float(getattr(_IC, "trim_tilt_lat", 0.0)),
    )
    runner = PhysicsRunner(_ROTOR, ic_run, WIND, col_min_rad=-0.28, col_max_rad=0.10)

    # Force cyclic to 0,0 at t=0 to test recovery toward IC attitude command.
    runner._acro._servo.reset(_IC.coll_eq_rad, tilt_lon=0.0, tilt_lat=0.0)

    rp = make_roll_pitch_params()
    yaw = make_yaw_params()
    hp = HeliParams(roll=rp, pitch=rp, yaw=yaw)

    guided = GuidedAttitudeController(hp)

    r_ic = Rotation.from_matrix(ic_run.R0)
    q_ic = r_ic.as_quat()
    euler_ic = r_ic.as_euler("xyz", degrees=True)
    roll_ic = float(euler_ic[0])
    pitch_ic = float(euler_ic[1])
    yaw_ic = float(euler_ic[2])

    col_norm = (_IC.coll_eq_rad - (-0.28)) / (0.10 - (-0.28)) * 2.0 - 1.0

    pos_hist = np.zeros((steps, 3))
    ten_hist = np.zeros(steps)
    axle_hist = np.zeros(steps)
    tel_rows: list[TelRow] = []
    df_rows: list[DataflashEquivRow] = []

    for step in range(steps):
        t_sim = step * DT

        obs = runner.observe()
        
        # Inject a disturbance at 5 seconds: brief rate pulse (larger: 25 deg/s)
        if step == 2000:  # t=5s, DT=1/400 -> frame 2000 out of 4000
            disturbance_rate = np.array([math.radians(25.0), 0.0, 0.0])  # body frame roll rate
            obs.gyro = obs.gyro + disturbance_rate

        if step % AP_EVERY == 0:
            guided.set_target_angle_and_rate_and_throttle(
                roll_ic,
                pitch_ic,
                yaw_ic,
                0.0,
                0.0,
                0.0,
                float(np.clip((col_norm + 1.0) * 0.5, 0.0, 1.0)),
                sim_time=t_sim,
            )

        q_body = Rotation.from_matrix(obs.R).as_quat()
        heli_out = guided.update(
            q_body,
            obs.gyro,
            DT,
            collective_norm=float(col_norm),
            sim_time=t_sim,
            pos_z_up_m=float(-obs.pos[2]),
            vel_z_up_mps=float(-obs.vel[2]),
        )
        df_rows.append(
            make_df_equiv_row(
                t=t_sim,
                R_body_ned=np.asarray(obs.R, dtype=float),
                gyro_body_rads=np.asarray(obs.gyro, dtype=float),
                guided=guided,
                heli_out=heli_out,
            )
        )
        sr = runner.step_guided(DT, _IC.coll_eq_rad, heli_out, rest_length=float(_IC.rest_length))

        # Extract guided controller state for telemetry
        tgt_euler = guided.attitude_target_euler_deg
        tgt_rate = guided.last_rate_target_rads
        R_now = np.asarray(obs.R, dtype=float)
        roll_now = math.atan2(float(R_now[2, 1]), float(R_now[2, 2]))
        pitch_now = -math.asin(float(np.clip(R_now[2, 0], -1.0, 1.0)))
        yaw_now = math.atan2(float(R_now[1, 0]), float(R_now[0, 0]))

        tel_rows.append(
            TelRow.from_physics(
                runner,
                sr,
                _IC.coll_eq_rad,
                WIND,
                body_z_eq=np.asarray(ic_run.R0[:, 2], dtype=float),
                phase="steady",
                mav_att_roll_deg=math.degrees(roll_now),
                mav_att_pitch_deg=math.degrees(pitch_now),
                mav_att_yaw_deg=math.degrees(yaw_now),
                mav_att_target_roll_deg=float(tgt_euler[0]),
                mav_att_target_pitch_deg=float(tgt_euler[1]),
                mav_att_target_yaw_deg=float(tgt_euler[2]),
                mav_att_target_roll_rate_rads=float(tgt_rate[0]),
                mav_att_target_pitch_rate_rads=float(tgt_rate[1]),
                mav_att_target_yaw_rate_rads=float(tgt_rate[2]),
            )
        )

        hub = runner.hub_state
        pos = hub["pos"]
        pos_hist[step] = pos
        ten_hist[step] = runner.tension_now

        R = np.asarray(hub["R"], dtype=float)
        body_z = R[:, 2]
        tlen = np.linalg.norm(pos)
        tdir = -pos / max(tlen, 0.1)
        axle_hist[step] = math.degrees(math.acos(float(np.clip(np.dot(body_z, tdir), -1.0, 1.0))))

    return {
        "pos": pos_hist,
        "tension": ten_hist,
        "axle_deg": axle_hist,
        "pos0": _IC.pos,
        "ic_q": q_ic,
        "telemetry": tel_rows,
        "df_equiv": df_rows,
    }


def test_steady_flight_ic_angle_only(simtest_log):
    """IC-angle-only guided control remains stable from zero cyclic initialization."""
    STEPS = 4000  # 10 s
    DRIFT_MAX = 15.0
    AXLE_MAX = 25.0

    data = _run_angle_ic_only(STEPS)
    write_csv(data["telemetry"], simtest_log.log_dir / "telemetry.csv")
    write_df_equiv_csv(data["df_equiv"], simtest_log.log_dir / "arduloop_df_equiv.csv")
    pos0 = data["pos0"]
    final = data["pos"][-1]
    drift = np.abs(final - pos0)

    simtest_log.write(
        [
            f"ic_angle_only drift_E={drift[1]:.3f}m drift_Z={drift[2]:.3f}m",
            f"tension_min={data['tension'].min():.2f}N tension_mean={data['tension'].mean():.2f}N",
            f"axle_max={data['axle_deg'].max():.2f}deg",
        ],
        f"ic_angle_only drift_E={drift[1]:.3f}m drift_Z={drift[2]:.3f}m",
    )

    failures = []
    if not np.all(np.isfinite(data["pos"])):
        failures.append("NaN/inf in position history")
    if drift[1] >= DRIFT_MAX:
        failures.append(f"East drift {drift[1]:.2f} m >= {DRIFT_MAX} m")
    if drift[2] >= DRIFT_MAX:
        failures.append(f"Vertical drift {drift[2]:.2f} m >= {DRIFT_MAX} m")
    if np.min(data["tension"]) < 0.01:
        failures.append("tether slack detected")
    if np.max(data["axle_deg"]) > AXLE_MAX:
        failures.append(f"axle misaligned > {AXLE_MAX} deg")

    assert not failures, "\n  ".join(failures)
