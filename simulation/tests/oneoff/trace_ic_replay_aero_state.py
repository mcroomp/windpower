"""IC replay aero-state trace - one-off diagnostic, not a unit test."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

SIM = Path(__file__).resolve().parents[2]
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))
SIMTESTS = SIM / "tests" / "simtests"
if str(SIMTESTS) not in sys.path:
    sys.path.insert(0, str(SIMTESTS))

from dynbem import RotorInputs, create_aero
from pumping_planner import TensionCommand
from tests.common.mock_ardupilot import MockArdupilot
from tests.simtests import test_generate_ic as icgen
from tests.simtests.simtest_runner import PhysicsRunner


def _project(row_force: np.ndarray, body_z: np.ndarray, wind: np.ndarray) -> dict[str, float]:
    wind_h = wind[:2].astype(float)
    wind_h /= max(float(np.linalg.norm(wind_h)), 1e-12)
    return {
        "mag": float(np.linalg.norm(row_force)),
        "up": -float(row_force[2]),
        "downwind": float(np.dot(row_force[:2], wind_h)),
        "minus_bodyz": -float(np.dot(row_force, body_z / max(float(np.linalg.norm(body_z)), 1e-12))),
    }


def _print_force(label: str, proj: dict[str, float]) -> None:
    print(
        f"    {label:<14} mag={proj['mag']:9.3f} up={proj['up']:9.3f} "
        f"down={proj['downwind']:9.3f} -F.bz={proj['minus_bodyz']:9.3f}"
    )


def main() -> None:
    d = json.loads(icgen._JSON_PATH.read_text())
    pos0 = np.array(d["pos"], dtype=float)
    vel0 = np.array(d["vel"], dtype=float)
    R0 = np.array(d["R0"], dtype=float).reshape(3, 3)
    rest = float(d["rest_length"])
    stack_coll = float(d["stack_coll_eq"])
    omega_spin = float(d["omega_spin"])
    tension_sp = float(d.get("tension_eq_n", 435.0))
    trim_tilt_lon = float(d.get("trim_tilt_lon", 0.0))
    trim_tilt_lat = float(d.get("trim_tilt_lat", 0.0))

    ic = SimpleNamespace(
        pos=pos0,
        vel=vel0,
        R0=R0,
        rest_length=rest,
        coll_eq_rad=stack_coll,
        omega_spin=omega_spin,
    )
    runner = PhysicsRunner(icgen._ROTOR, ic, icgen.WIND, col_min_rad=-0.28, col_max_rad=0.10)
    from controller import HeliCyclicController as _Heli
    runner._acro = _Heli(
        icgen._ROTOR, col_min_rad=-0.28, col_max_rad=0.10,
        P=0.67, I=0.15, D=0.02, IMAX=0.30,
        FLTT=40.0, FLTE=0.0, FLTD=40.0,
    )
    runner._acro._servo.reset(stack_coll)
    runner._acro.set_trim(trim_tilt_lon, trim_tilt_lat)

    ap = MockArdupilot.for_pumping(
        ic_pos=pos0,
        mass_kg=icgen.MASS,
        slew_rate_rad_s=icgen.BODY_Z_SLEW_RATE_RAD_S,
        warm_coll_rad=stack_coll,
        tension_ic=tension_sp,
        wind=icgen.WIND,
        dt=icgen._DT,
    )
    target_alt = float(-pos0[2])
    ap.receive_command(TensionCommand(
        tension_target_n=300.0,
        alt_m=target_alt,
        phase="reel-out",
    ), icgen._DT_CMD)

    planner_every = icgen._PLANNER_EVERY
    ap_every = icgen._AP_EVERY
    previous_minus_bodyz = None
    previous_slack = False
    first_flip_printed = False

    for step in range(icgen._STEADY_STEPS):
        t_sim = step * icgen._DT
        if step % planner_every == 0:
            ap.receive_command(TensionCommand(
                tension_target_n=300.0,
                alt_m=target_alt,
                phase="reel-out",
            ), icgen._DT_CMD)
        if step % ap_every == 0:
            ap.tick(t_sim, runner)

        hub_before = runner.hub_state
        rotor_state_before = runner._core._rotor_state
        rotor_arr_before = rotor_state_before.to_array().copy()
        tlon, tlat, col_act = runner._acro.step(
            ap.col_rad, ap.roll_sp, ap.pitch_sp, runner.omega_body, icgen._DT,
        )
        inputs = RotorInputs(
            collective_rad=col_act,
            tilt_lon=tlon,
            tilt_lat=tlat,
            R_hub=hub_before["R"],
            v_hub_world=hub_before["vel"],
            wind_world=icgen.WIND,
            omega_rad_s=runner.omega_spin,
            rho_kg_m3=1.225,
        )
        live_result, live_deriv = runner.aero.compute_forces(inputs, rotor_state_before)
        live_proj = _project(np.asarray(live_result.F_world, dtype=float), hub_before["R"][:, 2], icgen.WIND)

        sr = runner._core.step(icgen._DT, col_act, tlon, tlat, rest)
        tension = float(sr["tension_now"])
        slack = tension < 1.0

        if previous_minus_bodyz is not None and live_proj["minus_bodyz"] < 0.0 and not first_flip_printed:
            print("first live aero sign flip")
            print(
                f"  step={step} t={t_sim:.6f}s alt={-hub_before['pos'][2]:.3f}m "
                f"tlen={np.linalg.norm(hub_before['pos']):.6f}m tension_before={runner.tension_now:.3f}N "
                f"slack_after={slack} previous_slack={previous_slack}"
            )
            print(
                f"  inputs: col={col_act:+.6f} tlon={tlon:+.6f} tlat={tlat:+.6f} "
                f"omega={runner.omega_spin:.6f} vel={hub_before['vel'].tolist()}"
            )
            print(f"  body_z={hub_before['R'][:, 2].tolist()}")
            _print_force("live_state", live_proj)
            print(f"  Q_spin={float(live_result.Q_spin):+.6f}")
            print(f"  rotor_state_before={rotor_arr_before.tolist()}")
            print(f"  rotor_deriv={live_deriv.to_array().tolist()}")

            pitt = create_aero(icgen._ROTOR, model="pitt_peters")
            pitt_zero = pitt.initial_rotor_state()
            for label, arr in [
                ("pitt_zero", np.zeros_like(rotor_arr_before)),
                ("pitt_live_arr", rotor_arr_before),
                ("pitt_neg_live", -rotor_arr_before),
                ("pitt_half_live", 0.5 * rotor_arr_before),
            ]:
                state = pitt_zero.from_array(arr)
                result, deriv = pitt.compute_forces(inputs, state)
                _print_force(label, _project(np.asarray(result.F_world, dtype=float), hub_before["R"][:, 2], icgen.WIND))
                print(f"      Q_spin={float(result.Q_spin):+.6f} deriv={deriv.to_array().tolist()}")

            for model in ["oye", "pitt_peters", "quasi_static", "bem"]:
                aero = create_aero(icgen._ROTOR, model=model)
                state = aero.initial_rotor_state()
                result, _ = aero.compute_forces(inputs, state)
                _print_force(model, _project(np.asarray(result.F_world, dtype=float), hub_before["R"][:, 2], icgen.WIND))
                print(f"      Q_spin={float(result.Q_spin):+.6f}")
            first_flip_printed = True
            break

        previous_minus_bodyz = live_proj["minus_bodyz"]
        previous_slack = slack

    if not first_flip_printed:
        print("No live aero sign flip found in replay window.")


if __name__ == "__main__":
    main()