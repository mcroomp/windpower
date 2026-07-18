"""Replay aero failure scan - one-off diagnostic, not a unit test."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys

import numpy as np

SIM = Path(__file__).resolve().parents[2]

from dynbem import RotorInputs, create_aero
from tests.simtests._rotor_helpers import load_default_rotor


def _f(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    if value == "":
        return 0.0
    return float(value)


def _vec(row: dict[str, str], *keys: str) -> np.ndarray:
    return np.array([_f(row, key) for key in keys], dtype=float)


def _row_metrics(row: dict[str, str]) -> dict[str, float]:
    pos = _vec(row, "pos_x", "pos_y", "pos_z")
    vel = _vec(row, "vel_x", "vel_y", "vel_z")
    aero = _vec(row, "aero_fx", "aero_fy", "aero_fz")
    tether = _vec(row, "tether_fx", "tether_fy", "tether_fz")
    body_z = _vec(row, "r02", "r12", "r22")
    body_z_norm = float(np.linalg.norm(body_z))
    if body_z_norm > 1e-9:
        body_z = body_z / body_z_norm
    wind = _vec(row, "wind_x", "wind_y", "wind_z")
    wind_h = wind[:2]
    wind_h_norm = float(np.linalg.norm(wind_h))
    if wind_h_norm > 1e-9:
        wind_h = wind_h / wind_h_norm
    else:
        wind_h = np.array([0.0, 1.0])

    tlen = float(np.linalg.norm(pos))
    radial = pos / max(tlen, 1e-9)
    gravity = np.array([0.0, 0.0, 49.05])
    net = aero + tether + gravity
    speed = float(np.linalg.norm(vel))
    rel_wind = wind - vel
    body_rel = np.array([
        _f(row, "r00") * rel_wind[0] + _f(row, "r10") * rel_wind[1] + _f(row, "r20") * rel_wind[2],
        _f(row, "r01") * rel_wind[0] + _f(row, "r11") * rel_wind[1] + _f(row, "r21") * rel_wind[2],
        _f(row, "r02") * rel_wind[0] + _f(row, "r12") * rel_wind[1] + _f(row, "r22") * rel_wind[2],
    ])

    return {
        "t": _f(row, "t_sim"),
        "alt": -_f(row, "pos_z"),
        "tlen": tlen,
        "speed": speed,
        "omega": _f(row, "omega_rotor"),
        "collective": _f(row, "collective_rad"),
        "tilt_lon": _f(row, "tilt_lon"),
        "tilt_lat": _f(row, "tilt_lat"),
        "tension": _f(row, "tether_tension"),
        "slack": _f(row, "tether_slack"),
        "aero_mag": float(np.linalg.norm(aero)),
        "aero_downwind": float(np.dot(aero[:2], wind_h)),
        "aero_up": -float(aero[2]),
        "aero_bodyz": float(np.dot(aero, body_z)),
        "aero_minus_bodyz": -float(np.dot(aero, body_z)),
        "aero_radial": float(np.dot(aero, radial)),
        "net_downwind": float(np.dot(net[:2], wind_h)),
        "net_up": -float(net[2]),
        "rel_body_z": float(body_rel[2]),
        "rel_body_xy": float(np.linalg.norm(body_rel[:2])),
        "aero_T": _f(row, "aero_T"),
        "aero_Q": _f(row, "aero_Q_spin"),
        "aero_v_axial": _f(row, "aero_v_axial"),
        "aero_v_inplane": _f(row, "aero_v_inplane"),
        "aero_v_i": _f(row, "aero_v_i"),
        "body_z_err_deg": _f(row, "body_z_err_deg"),
        "roll_sp": _f(row, "roll_sp_rads"),
        "pitch_sp": _f(row, "pitch_sp_rads"),
    }


def _project_force(row: dict[str, str], force: np.ndarray) -> dict[str, float]:
    body_z = _vec(row, "r02", "r12", "r22")
    body_z_norm = float(np.linalg.norm(body_z))
    if body_z_norm > 1e-9:
        body_z = body_z / body_z_norm
    wind = _vec(row, "wind_x", "wind_y", "wind_z")
    wind_h = wind[:2]
    wind_h_norm = float(np.linalg.norm(wind_h))
    if wind_h_norm > 1e-9:
        wind_h = wind_h / wind_h_norm
    else:
        wind_h = np.array([0.0, 1.0])
    return {
        "mag": float(np.linalg.norm(force)),
        "up": -float(force[2]),
        "downwind": float(np.dot(force[:2], wind_h)),
        "minus_bodyz": -float(np.dot(force, body_z)),
    }


def _rotor_inputs_from_row(row: dict[str, str]) -> RotorInputs:
    return RotorInputs(
        collective_rad=_f(row, "collective_rad"),
        tilt_lon=_f(row, "tilt_lon"),
        tilt_lat=_f(row, "tilt_lat"),
        R_hub=np.array([
            [_f(row, "r00"), _f(row, "r01"), _f(row, "r02")],
            [_f(row, "r10"), _f(row, "r11"), _f(row, "r12")],
            [_f(row, "r20"), _f(row, "r21"), _f(row, "r22")],
        ], dtype=float),
        v_hub_world=_vec(row, "vel_x", "vel_y", "vel_z"),
        wind_world=_vec(row, "wind_x", "wind_y", "wind_z"),
        omega_rad_s=_f(row, "omega_rotor"),
        rho_kg_m3=1.225,
        t=_f(row, "t_sim"),
    )


def _print_recompute_input(row: dict[str, str], index: int, model: str) -> None:
    rotor = load_default_rotor()
    aero = create_aero(rotor, model=model)
    inputs = _rotor_inputs_from_row(row)
    result, _ = aero.compute_forces(inputs, aero.initial_rotor_state())
    force = np.asarray(result.F_world, dtype=float)
    body_z = inputs.R_hub[:, 2]
    body_x = inputs.R_hub[:, 0]
    body_y = inputs.R_hub[:, 1]
    rel_wind_world = inputs.wind_world - inputs.v_hub_world
    rel_wind_body = inputs.R_hub.T @ rel_wind_world
    row_force = _vec(row, "aero_fx", "aero_fy", "aero_fz")
    recomputed = _project_force(row, force)
    logged = _project_force(row, row_force)

    print(f"first fresh {model} -F.bz sign flip: row={index} t={_f(row, 't_sim'):.4f}s")
    print(_fmt(_row_metrics(row)))
    print("RotorInputs:")
    print(f"  collective_rad = {inputs.collective_rad:+.9f}")
    print(f"  tilt_lon       = {inputs.tilt_lon:+.9f}")
    print(f"  tilt_lat       = {inputs.tilt_lat:+.9f}")
    print(f"  omega_rad_s    = {inputs.omega_rad_s:+.9f}")
    print(f"  t              = {inputs.t:+.9f}")
    print(f"  rho_kg_m3      = {inputs.rho_kg_m3:+.9f}")
    print(f"  v_hub_world    = {inputs.v_hub_world.tolist()}")
    print(f"  wind_world     = {inputs.wind_world.tolist()}")
    print(f"  rel_wind_world = {rel_wind_world.tolist()}")
    print(f"  rel_wind_body  = {rel_wind_body.tolist()}")
    print(f"  body_x_world   = {body_x.tolist()}")
    print(f"  body_y_world   = {body_y.tolist()}")
    print(f"  body_z_world   = {body_z.tolist()}")
    print("R_hub body-to-world rows:")
    for row_values in inputs.R_hub:
        print("  " + " ".join(f"{value:+.9f}" for value in row_values))
    print("Forces:")
    print(f"  logged_F_world     = {row_force.tolist()}")
    print(f"  recomputed_F_world = {force.tolist()}")
    print(
        f"  logged:     mag={logged['mag']:.6f} up={logged['up']:.6f} "
        f"downwind={logged['downwind']:.6f} -F.bz={logged['minus_bodyz']:.6f}"
    )
    print(
        f"  recomputed: mag={recomputed['mag']:.6f} up={recomputed['up']:.6f} "
        f"downwind={recomputed['downwind']:.6f} -F.bz={recomputed['minus_bodyz']:.6f}"
    )
    print(f"  Q_spin = {float(result.Q_spin):+.9f}")


def _scan_fresh_model_signs(rows: list[dict[str, str]], model: str) -> int | None:
    rotor = load_default_rotor()
    initial_sign: float | None = None
    for index, row in enumerate(rows):
        aero = create_aero(rotor, model=model)
        inputs = _rotor_inputs_from_row(row)
        result, _ = aero.compute_forces(inputs, aero.initial_rotor_state())
        force = np.asarray(result.F_world, dtype=float)
        body_z = inputs.R_hub[:, 2]
        minus_bodyz = -float(np.dot(force, body_z))
        sign = math.copysign(1.0, minus_bodyz or 1.0)
        if initial_sign is None:
            initial_sign = sign
        elif sign != initial_sign:
            _print_recompute_input(row, index, model)
            return index
    print(f"fresh {model}: no -F.bz sign flip across {len(rows)} rows")
    return None


def _compare_models(rows: list[dict[str, str]], center_index: int, radius: int, models: list[str]) -> None:
    rotor = load_default_rotor()
    print("model recompute from exact CSV row inputs (fresh model state):")
    for index in range(max(0, center_index - radius), min(len(rows), center_index + radius + 1)):
        row = rows[index]
        logged = _project_force(row, _vec(row, "aero_fx", "aero_fy", "aero_fz"))
        print(
            f"  row={index} t={_f(row, 't_sim'):.4f}s "
            f"logged: mag={logged['mag']:8.1f} up={logged['up']:8.1f} "
            f"down={logged['downwind']:8.1f} -F.bz={logged['minus_bodyz']:8.1f}"
        )
        inputs = _rotor_inputs_from_row(row)
        for model in models:
            aero = create_aero(rotor, model=model)
            state = aero.initial_rotor_state()
            result, _ = aero.compute_forces(inputs, state)
            proj = _project_force(row, np.asarray(result.F_world, dtype=float))
            print(
                f"    {model:<13} mag={proj['mag']:8.1f} up={proj['up']:8.1f} "
                f"down={proj['downwind']:8.1f} -F.bz={proj['minus_bodyz']:8.1f} "
                f"Q={float(result.Q_spin):+.4f}"
            )


def _fmt(metrics: dict[str, float]) -> str:
    return (
        f"t={metrics['t']:6.3f}s alt={metrics['alt']:6.2f}m tlen={metrics['tlen']:7.2f}m "
        f"T={metrics['tension']:7.1f}N omega={metrics['omega']:6.2f} col={metrics['collective']:+.4f} "
        f"tlon={metrics['tilt_lon']:+.4f} tlat={metrics['tilt_lat']:+.4f} "
        f"Faero={metrics['aero_mag']:8.1f}N up={metrics['aero_up']:8.1f} down={metrics['aero_downwind']:8.1f} "
        f"-F.bz={metrics['aero_minus_bodyz']:8.1f} net_up={metrics['net_up']:8.1f} "
        f"v_body_z={metrics['rel_body_z']:7.2f} v_body_xy={metrics['rel_body_xy']:7.2f} "
        f"aero_T={metrics['aero_T']:8.1f} Q={metrics['aero_Q']:8.2f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, nargs="?", default=Path("simulation/logs/test_ic_steady_flight/telemetry.csv"))
    parser.add_argument("--jump-n", type=float, default=250.0, help="force magnitude jump threshold [N/frame]")
    parser.add_argument("--force-n", type=float, default=2500.0, help="large force threshold [N]")
    parser.add_argument("--compare-models", action="store_true", help="recompute rows near first sign flip with fresh aero models")
    parser.add_argument("--compare-radius", type=int, default=2)
    parser.add_argument("--models", nargs="+", default=["oye", "pitt_peters", "quasi_static", "bem"])
    parser.add_argument("--scan-fresh-model", help="scan fresh recomputes and print the first -F.bz sign flip for this model")
    args = parser.parse_args()

    with args.csv.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if args.scan_fresh_model:
        _scan_fresh_model_signs(rows, args.scan_fresh_model)
        return
    metrics = [_row_metrics(row) for row in rows]

    print(f"rows={len(metrics)} csv={args.csv}")
    if not metrics:
        return

    first_nonfinite = None
    first_big_force = None
    first_big_jump = None
    first_up_sign_flip = None
    first_bodyz_sign_flip = None
    first_slack = None
    previous = metrics[0]
    initial_up_sign = math.copysign(1.0, metrics[0]["aero_up"] or 1.0)
    initial_bodyz_sign = math.copysign(1.0, metrics[0]["aero_minus_bodyz"] or 1.0)

    for index, current in enumerate(metrics):
        values = list(current.values())
        if first_nonfinite is None and not all(math.isfinite(value) for value in values):
            first_nonfinite = index
        if first_big_force is None and current["aero_mag"] > args.force_n:
            first_big_force = index
        if first_up_sign_flip is None and math.copysign(1.0, current["aero_up"] or 1.0) != initial_up_sign:
            first_up_sign_flip = index
        if first_bodyz_sign_flip is None and math.copysign(1.0, current["aero_minus_bodyz"] or 1.0) != initial_bodyz_sign:
            first_bodyz_sign_flip = index
        if first_slack is None and current["slack"] >= 0.5:
            first_slack = index
        if index > 0:
            jump = abs(current["aero_mag"] - previous["aero_mag"])
            if first_big_jump is None and jump > args.jump_n:
                first_big_jump = index
        previous = current

    labels = [
        ("first_nonfinite", first_nonfinite),
        ("first_aero_mag_gt_threshold", first_big_force),
        ("first_aero_mag_jump_gt_threshold", first_big_jump),
        ("first_aero_up_sign_flip", first_up_sign_flip),
        ("first_minus_F_dot_body_z_sign_flip", first_bodyz_sign_flip),
        ("first_tether_slack", first_slack),
    ]
    for label, index in labels:
        if index is None:
            print(f"{label}: none")
            continue
        print(f"{label}: index={index}")
        for j in range(max(0, index - 2), min(len(metrics), index + 3)):
            prefix = "  -> " if j == index else "     "
            print(prefix + _fmt(metrics[j]))

    max_force_i = max(range(len(metrics)), key=lambda i: metrics[i]["aero_mag"])
    min_up_i = min(range(len(metrics)), key=lambda i: metrics[i]["aero_up"])
    max_up_i = max(range(len(metrics)), key=lambda i: metrics[i]["aero_up"])
    print("extrema:")
    print("  max aero_mag: " + _fmt(metrics[max_force_i]))
    print("  min aero_up : " + _fmt(metrics[min_up_i]))
    print("  max aero_up : " + _fmt(metrics[max_up_i]))
    if args.compare_models:
        center = first_bodyz_sign_flip if first_bodyz_sign_flip is not None else first_up_sign_flip
        if center is None:
            center = max_force_i
        _compare_models(rows, center, args.compare_radius, args.models)


if __name__ == "__main__":
    main()