"""DataFlash-equivalent logging helpers for arduloop guided control traces.

This module writes a compact CSV that mirrors the key ArduPilot DataFlash
channels used for attitude/rate response comparisons in analysis scripts.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .attitude_heli import HeliRateOutput
from .guided import GuidedAttitudeController


@dataclass(frozen=True)
class DataflashEquivRow:
    """One DataFlash-equivalent sample for guided attitude/rate analysis."""

    t: float
    att_roll_deg: float
    att_pitch_deg: float
    att_yaw_deg: float
    des_roll_deg: float
    des_pitch_deg: float
    des_yaw_deg: float
    rate_roll_deg_s: float
    rate_pitch_deg_s: float
    rate_yaw_deg_s: float
    rate_des_roll_deg_s: float
    rate_des_pitch_deg_s: float
    rate_des_yaw_deg_s: float
    cyc_roll: float
    cyc_pitch: float
    cyc_yaw: float
    collective_norm_cmd: float

    @staticmethod
    def columns() -> list[str]:
        return [
            "t",
            "att_roll_deg",
            "att_pitch_deg",
            "att_yaw_deg",
            "des_roll_deg",
            "des_pitch_deg",
            "des_yaw_deg",
            "rate_roll_deg_s",
            "rate_pitch_deg_s",
            "rate_yaw_deg_s",
            "rate_des_roll_deg_s",
            "rate_des_pitch_deg_s",
            "rate_des_yaw_deg_s",
            "cyc_roll",
            "cyc_pitch",
            "cyc_yaw",
            "collective_norm_cmd",
        ]


def make_df_equiv_row(
    *,
    t: float,
    R_body_ned: np.ndarray,
    gyro_body_rads: np.ndarray,
    guided: GuidedAttitudeController,
    heli_out: HeliRateOutput,
) -> DataflashEquivRow:
    """Build one DataFlash-equivalent row from controller and physics state."""
    eul_actual = Rotation.from_matrix(np.asarray(R_body_ned, dtype=float)).as_euler("xyz", degrees=True)
    eul_target = guided.target_euler_deg
    rate_des = np.degrees(guided.last_rate_target_rads)
    rate_actual = np.degrees(np.asarray(gyro_body_rads, dtype=float))

    return DataflashEquivRow(
        t=float(t),
        att_roll_deg=float(eul_actual[0]),
        att_pitch_deg=float(eul_actual[1]),
        att_yaw_deg=float(eul_actual[2]),
        des_roll_deg=float(eul_target[0]),
        des_pitch_deg=float(eul_target[1]),
        des_yaw_deg=float(eul_target[2]),
        rate_roll_deg_s=float(rate_actual[0]),
        rate_pitch_deg_s=float(rate_actual[1]),
        rate_yaw_deg_s=float(rate_actual[2]),
        rate_des_roll_deg_s=float(rate_des[0]),
        rate_des_pitch_deg_s=float(rate_des[1]),
        rate_des_yaw_deg_s=float(rate_des[2]),
        cyc_roll=float(heli_out.roll_cyclic),
        cyc_pitch=float(heli_out.pitch_cyclic),
        cyc_yaw=float(heli_out.yaw_cmd),
        collective_norm_cmd=float(0.0 if heli_out.collective_norm_cmd is None else heli_out.collective_norm_cmd),
    )


def write_df_equiv_csv(rows: list[DataflashEquivRow], csv_path: Path) -> None:
    """Write DataFlash-equivalent rows to CSV with a stable schema."""
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    cols = DataflashEquivRow.columns()
    with csv_path.open("w", newline="", encoding="ascii") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(
                {
                    "t": f"{r.t:.6f}",
                    "att_roll_deg": f"{r.att_roll_deg:.9g}",
                    "att_pitch_deg": f"{r.att_pitch_deg:.9g}",
                    "att_yaw_deg": f"{r.att_yaw_deg:.9g}",
                    "des_roll_deg": f"{r.des_roll_deg:.9g}",
                    "des_pitch_deg": f"{r.des_pitch_deg:.9g}",
                    "des_yaw_deg": f"{r.des_yaw_deg:.9g}",
                    "rate_roll_deg_s": f"{r.rate_roll_deg_s:.9g}",
                    "rate_pitch_deg_s": f"{r.rate_pitch_deg_s:.9g}",
                    "rate_yaw_deg_s": f"{r.rate_yaw_deg_s:.9g}",
                    "rate_des_roll_deg_s": f"{r.rate_des_roll_deg_s:.9g}",
                    "rate_des_pitch_deg_s": f"{r.rate_des_pitch_deg_s:.9g}",
                    "rate_des_yaw_deg_s": f"{r.rate_des_yaw_deg_s:.9g}",
                    "cyc_roll": f"{r.cyc_roll:.9g}",
                    "cyc_pitch": f"{r.cyc_pitch:.9g}",
                    "cyc_yaw": f"{r.cyc_yaw:.9g}",
                    "collective_norm_cmd": f"{r.collective_norm_cmd:.9g}",
                }
            )
