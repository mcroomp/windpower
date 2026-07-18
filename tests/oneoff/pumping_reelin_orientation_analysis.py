"""pumping reel-in orientation analysis - one-off diagnostic, not a unit test."""
from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
CSV_PATH = ROOT / "simulation" / "logs" / "test_pumping_unified" / "telemetry.csv"


def _as_float(row: dict[str, str], name: str) -> float:
    return float(row[name])


def _optional_float(row: dict[str, str], name: str) -> float | None:
    value = row.get(name)
    if value in (None, ""):
        return None
    return float(value)


def _unit(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-12:
        return vec
    return vec / norm


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip(np.dot(_unit(a), _unit(b)), -1.0, 1.0))))


def _elev_deg(vec: np.ndarray) -> float:
    unit = _unit(vec)
    return math.degrees(math.asin(float(np.clip(unit[2], -1.0, 1.0))))


def _stats(values: list[float]) -> tuple[float, float, float, float]:
    arr = np.asarray(values, dtype=float)
    return float(np.mean(arr)), float(np.min(arr)), float(np.max(arr)), float(np.ptp(arr))


def _corr(a: list[float], b: list[float]) -> float:
    aa = np.asarray(a, dtype=float)
    bb = np.asarray(b, dtype=float)
    if len(aa) < 2 or float(np.std(aa)) < 1e-12 or float(np.std(bb)) < 1e-12:
        return float("nan")
    return float(np.corrcoef(aa, bb)[0, 1])


def main() -> None:
    with CSV_PATH.open(newline="") as handle:
        rows = list(csv.DictReader(handle))

    print(f"telemetry: {CSV_PATH}")
    for phase in ("cycle1_reel_in", "cycle2_reel_in", "cycle3_reel_in"):
        phase_rows = [row for row in rows if row["phase"] == phase]
        if not phase_rows:
            continue

        cmd_elev: list[float] = []
        act_elev: list[float] = []
        bz_err: list[float] = []
        tension: list[float] = []
        collective: list[float] = []
        altitude: list[float] = []
        target_altitude: list[float] = []
        tether_length: list[float] = []
        rest_length: list[float] = []
        winch_speed: list[float] = []
        aero_t: list[float] = []
        roll_sp: list[float] = []
        pitch_sp: list[float] = []

        for row in phase_rows:
            bz_eq = np.array([_as_float(row, "bz_eq_x"), _as_float(row, "bz_eq_y"), _as_float(row, "bz_eq_z")])
            bz_now = np.array([_as_float(row, "r02"), _as_float(row, "r12"), _as_float(row, "r22")])
            cmd_elev.append(_elev_deg(bz_eq))
            act_elev.append(_elev_deg(bz_now))
            bz_err.append(_angle_deg(bz_eq, bz_now))
            tension.append(_as_float(row, "tether_tension"))
            collective.append(_as_float(row, "collective_rad"))
            altitude.append(-_as_float(row, "pos_z"))
            target_altitude.append(_as_float(row, "gnd_alt_cmd_m"))
            tether_length.append(_as_float(row, "tether_length"))
            rest_length.append(_as_float(row, "tether_rest_length"))
            winch_speed.append(_as_float(row, "winch_speed_ms"))
            aero_t.append(_as_float(row, "aero_T"))
            roll_value = _optional_float(row, "roll_sp_rads")
            pitch_value = _optional_float(row, "pitch_sp_rads")
            if roll_value is not None:
                roll_sp.append(roll_value)
            if pitch_value is not None:
                pitch_sp.append(pitch_value)

        print(f"\n{phase}  rows={len(phase_rows)}  t={phase_rows[0]['t_sim']}..{phase_rows[-1]['t_sim']}s")
        for label, values in (
            ("cmd_body_z_elev_deg", cmd_elev),
            ("act_body_z_elev_deg", act_elev),
            ("body_z_err_deg", bz_err),
            ("tension_N", tension),
            ("collective_rad", collective),
            ("altitude_m", altitude),
            ("gnd_alt_cmd_m", target_altitude),
            ("tether_length_m", tether_length),
            ("rest_length_m", rest_length),
            ("winch_speed_mps", winch_speed),
            ("aero_T_N", aero_t),
        ):
            avg, low, high, pp = _stats(values)
            print(f"  {label:22s} avg={avg:9.4f} min={low:9.4f} max={high:9.4f} pp={pp:9.4f}")

        for label, values in (("roll_sp_rads", roll_sp), ("pitch_sp_rads", pitch_sp)):
            if values:
                avg, low, high, pp = _stats(values)
                print(f"  {label:22s} avg={avg:9.4f} min={low:9.4f} max={high:9.4f} pp={pp:9.4f}")
            else:
                print(f"  {label:22s} missing in telemetry")

        print("  correlations with cmd_body_z_elev_deg:")
        for label, values in (
            ("tether_length_m", tether_length),
            ("rest_length_m", rest_length),
            ("altitude_m", altitude),
            ("tension_N", tension),
            ("collective_rad", collective),
            ("aero_T_N", aero_t),
        ):
            print(f"    {label:18s} r={_corr(cmd_elev, values):+.4f}")
        for label, values in (("pitch_sp_rads", pitch_sp), ("roll_sp_rads", roll_sp)):
            if values:
                print(f"    {label:18s} r={_corr(cmd_elev, values):+.4f}")


if __name__ == "__main__":
    main()