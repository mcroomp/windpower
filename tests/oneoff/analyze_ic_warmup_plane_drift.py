"""IC warmup plane drift force audit - one-off diagnostic, not a unit test."""

from __future__ import annotations

import math
import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

SIM = Path(__file__).resolve().parents[2]
SIMTESTS = SIM / "tests" / "simtests"

from dynbem import RotorInputs, create_aero, euler_step_omega, relax_inflow, solve_trim_cyclic
from simulation.frames import build_orb_frame
from simulation.pumping_planner import TensionCommand
from tests.common.mock_ardupilot import MockArdupilot
from tests.simtests import test_generate_ic as icgen
from tests.simtests.simtest_runner import PhysicsRunner
from simulation.controller import HeliCyclicController
from simulation.telemetry_csv import TelRow, write_csv


def _basis():
    wind_h = np.array(icgen.WIND[:2], dtype=float)
    wind_h /= max(float(np.linalg.norm(wind_h)), 1e-12)
    cross_h = np.array([-wind_h[1], wind_h[0]])
    return wind_h, cross_h


def _setup_runner(setup_model: str, runner_model: str):
    tether_hat = icgen._wind_aligned_tether_hat()
    pos0 = icgen.L_TETHER * tether_hat
    t_dir = -tether_hat
    R0 = build_orb_frame(t_dir)

    aero_est = create_aero(icgen._ROTOR, model=setup_model)
    state = aero_est.initial_rotor_state()
    omega_now = 20.0
    omega_min = icgen._ROTOR.autorotation.omega_min_rad_s or 0.5
    I_ode = icgen._ROTOR.autorotation.I_ode_kgm2 or 10.0
    dt_eq = 1.0 / 400.0
    spin_angle = 0.0
    for _ in range(8):
        inputs_eq = RotorInputs(
            collective_rad=icgen.STACK_COLL,
            tilt_lon=0.0,
            tilt_lat=0.0,
            R_hub=R0,
            v_hub_world=np.zeros(3),
            wind_world=icgen.WIND,
            omega_rad_s=omega_now,
            rho_kg_m3=1.225,
        )
        state = relax_inflow(aero_est, state, inputs_eq, n_steps=400, dt=dt_eq)
        for _ in range(200):
            result, deriv = aero_est.compute_forces(inputs_eq, state)
            state = state.from_array(state.to_array() + dt_eq * deriv.to_array())
            new_omega, spin_angle = euler_step_omega(
                omega_now, spin_angle, float(result.Q_spin), 0.0, I_ode, dt_eq,
            )
            omega_now = max(omega_min, new_omega)

    trim = solve_trim_cyclic(
        aero_est,
        state,
        RotorInputs(
            collective_rad=icgen.STACK_COLL,
            tilt_lon=0.0,
            tilt_lat=0.0,
            R_hub=R0,
            v_hub_world=np.zeros(3),
            wind_world=icgen.WIND,
            omega_rad_s=omega_now,
            rho_kg_m3=1.225,
        ),
        n_inflow_relax=200,
        dt_relax=dt_eq,
        tolerance_Nm=0.1,
    )
    state = trim.final_state

    f_est, _ = aero_est.compute_forces(
        RotorInputs(
            collective_rad=icgen.STACK_COLL,
            tilt_lon=0.0,
            tilt_lat=0.0,
            R_hub=R0,
            v_hub_world=np.zeros(3),
            wind_world=icgen.WIND,
            omega_rad_s=omega_now,
            rho_kg_m3=1.225,
        ),
        state,
    )
    T_est = max(-float(np.dot(f_est.F_world, t_dir)), 10.0)
    rest_length = icgen.L_TETHER - max(T_est / (icgen.TetherModel.EA_N / icgen.L_TETHER), 0.001)

    ic = SimpleNamespace(
        pos=np.asarray(pos0, dtype=float),
        vel=np.zeros(3),
        R0=R0,
        rest_length=float(rest_length),
        coll_eq_rad=float(icgen.STACK_COLL),
        omega_spin=float(omega_now),
    )
    runner = PhysicsRunner(
        icgen._ROTOR,
        ic,
        icgen.WIND,
        aero_model=runner_model,
        
    )
    runner._acro = HeliCyclicController(
        icgen._ROTOR,
        
        P=0.67,
        I=0.15,
        D=0.02,
        IMAX=0.30,
        FLTT=40.0,
        FLTE=0.0,
        FLTD=40.0,
    )
    runner._acro._servo.reset(icgen.STACK_COLL)
    runner._acro.set_trim(trim.tilt_lon, trim.tilt_lat)

    ap = MockArdupilot.for_pumping(
        ic_pos=pos0,
        mass_kg=icgen.MASS,
        slew_rate_rad_s=icgen.BODY_Z_SLEW_RATE_RAD_S,
        warm_coll_rad=icgen.STACK_COLL,
        tension_ic=icgen.IC_TARGET_TENSION_N,
        wind=icgen.WIND,
        dt=icgen._DT,
    )
    target_alt = float(-pos0[2])
    ap.receive_command(
        TensionCommand(
            tension_target_n=300.0,
            alt_m=target_alt,
            phase="reel-out",
        ),
        icgen._DT_CMD,
    )
    return runner, ap, rest_length, trim, pos0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compare-compute-ic", action="store_true")
    parser.add_argument("--collective-sweep", action="store_true")
    parser.add_argument("--force-balance", action="store_true")
    parser.add_argument("--aero-io", action="store_true")
    parser.add_argument("--aero-model", default="quasi_static", choices=["jit", "pitt_peters", "oye", "quasi_static", "bem"])
    parser.add_argument("--csv-out", type=Path)
    args = parser.parse_args()

    wind_h, cross_h = _basis()
    setup_model = args.aero_model
    runner_model = args.aero_model
    runner, ap, rest_now, trim, pos0 = _setup_runner(setup_model, runner_model)
    if args.aero_io:
        pos = np.asarray(runner.hub_state["pos"], dtype=float)
        vel = np.asarray(runner.hub_state["vel"], dtype=float)
        R = np.asarray(runner.hub_state["R"], dtype=float)
        body_x = R[:, 0]
        body_y = R[:, 1]
        body_z = R[:, 2]
        tether_to_anchor = -pos / np.linalg.norm(pos)
        rel_wind_world = icgen.WIND - vel
        rel_wind_body = R.T @ rel_wind_world
        rotor_state = runner._core._rotor_state  # noqa: SLF001 - one-off aero audit

        def dump_case(label: str, collective: float, tilt_lon: float, tilt_lat: float) -> None:
            inputs = RotorInputs(
                collective_rad=collective,
                tilt_lon=tilt_lon,
                tilt_lat=tilt_lat,
                R_hub=R,
                v_hub_world=vel,
                wind_world=icgen.WIND,
                omega_rad_s=runner.omega_spin,
                rho_kg_m3=1.225,
            )
            result, deriv = runner.aero.compute_forces(inputs, rotor_state)
            F = np.asarray(result.F_world, dtype=float)
            M = np.asarray(result.m_hub_world, dtype=float)
            F_body = R.T @ F
            print(label)
            print("  inputs:")
            print(f"    model={runner_model} setup_model={setup_model}")
            print(f"    collective_rad={inputs.collective_rad:+.6f}")
            print(f"    tilt_lon={inputs.tilt_lon:+.6f} tilt_lat={inputs.tilt_lat:+.6f}")
            print(f"    omega_rad_s={inputs.omega_rad_s:+.6f}")
            print(f"    t={inputs.t:+.6f} rho={inputs.rho_kg_m3:+.6f}")
            print(f"    pos_ned={pos.tolist()}")
            print(f"    v_hub_world={inputs.v_hub_world.tolist()}")
            print(f"    wind_world={inputs.wind_world.tolist()}")
            print(f"    rel_wind_world={rel_wind_world.tolist()}")
            print(f"    rel_wind_body={rel_wind_body.tolist()}")
            print("    R_hub columns in NED:")
            print(f"      body_x={body_x.tolist()}")
            print(f"      body_y={body_y.tolist()}")
            print(f"      body_z={body_z.tolist()}")
            print(f"    tether_to_anchor={tether_to_anchor.tolist()}")
            print("  outputs:")
            print(f"    F_world={F.tolist()}")
            print(f"    F_body={F_body.tolist()}")
            print(f"    m_hub_world={M.tolist()}")
            print(f"    Q_spin={float(result.Q_spin):+.6f}")
            print(f"    dstate={deriv.to_array().tolist()}")
            print("  projections:")
            print(f"    F_downwind={float(np.dot(F[:2], wind_h)):+.6f} N")
            print(f"    F_cross={float(np.dot(F[:2], cross_h)):+.6f} N")
            print(f"    F_up={float(-F[2]):+.6f} N")
            print(f"    F_dot_body_z={float(np.dot(F, body_z)):+.6f} N")
            print(f"    F_dot_tether_to_anchor={float(np.dot(F, tether_to_anchor)):+.6f} N")
            print(f"    rel_wind_dot_body_z={float(np.dot(rel_wind_world, body_z)):+.6f} m/s")
            print()

        print("IC clean-state aero input/output dump")
        print(f"  trim_tlon={trim.tilt_lon:+.6f} trim_tlat={trim.tilt_lat:+.6f}")
        dump_case("zero cyclic", icgen.STACK_COLL, 0.0, 0.0)
        dump_case("trim cyclic", icgen.STACK_COLL, trim.tilt_lon, trim.tilt_lat)
        return

    if args.force_balance:
        def print_balance(
            label: str,
            pos: np.ndarray,
            vel: np.ndarray,
            aero: np.ndarray,
            tether: np.ndarray,
            gravity: np.ndarray,
            *,
            collective_rad: float,
            tilt_lon: float,
            tilt_lat: float,
            omega_spin: float,
            tension_n: float | None,
        ) -> None:
            t_hat = pos / np.linalg.norm(pos)  # anchor -> hub radial
            radial_h = np.array([t_hat[0], t_hat[1]])
            radial_h /= max(float(np.linalg.norm(radial_h)), 1e-12)
            az_tan_h = np.array([-radial_h[1], radial_h[0]])
            residual = aero + tether + gravity
            print(label)
            print(f"  pos N/E/Z = {pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.2f}")
            print(f"  vel N/E/D = {vel[0]:+.3f}, {vel[1]:+.3f}, {vel[2]:+.3f} m/s")
            print(
                f"  inputs: collective={collective_rad:+.5f} rad, "
                f"tilt_lon={tilt_lon:+.5f} rad, tilt_lat={tilt_lat:+.5f} rad, "
                f"omega_spin={omega_spin:.3f} rad/s"
            )
            if tension_n is not None:
                print(f"  tension={tension_n:.2f} N")
            print("  components: +downwind is with wind, +up is upward, +radial is anchor->hub")
            print("            downwind    cross        up      radial_h    az_tan_h")
            for name, force in [("aero", aero), ("tether", tether), ("gravity", gravity), ("residual", residual)]:
                print(
                    f"  {name:8s} "
                    f"{float(np.dot(force[:2], wind_h)):+10.2f} "
                    f"{float(np.dot(force[:2], cross_h)):+9.2f} "
                    f"{float(-force[2]):+9.2f} "
                    f"{float(np.dot(force[:2], radial_h)):+10.2f} "
                    f"{float(np.dot(force[:2], az_tan_h)):+10.2f}"
                )
            print(f"  |residual| = {float(np.linalg.norm(residual)):.2f} N")
            print()

        pos = np.asarray(runner.hub_state["pos"], dtype=float)
        vel = np.asarray(runner.hub_state["vel"], dtype=float)
        R = np.asarray(runner.hub_state["R"], dtype=float)
        rotor_state = runner._core._rotor_state  # noqa: SLF001 - one-off force audit
        result, _ = runner.aero.compute_forces(
            RotorInputs(
                collective_rad=icgen.STACK_COLL,
                tilt_lon=0.0,
                tilt_lat=0.0,
                R_hub=R,
                v_hub_world=vel,
                wind_world=icgen.WIND,
                omega_rad_s=runner.omega_spin,
                rho_kg_m3=1.225,
            ),
            rotor_state,
        )
        tether, _ = runner.tether.compute(pos, vel, R)
        gravity = np.array([0.0, 0.0, icgen.MASS * icgen.G])
        print_balance(
            "clean wind-aligned start, STACK_COLL, zero cyclic",
            pos,
            vel,
            np.asarray(result.F_world),
            tether,
            gravity,
            collective_rad=icgen.STACK_COLL,
            tilt_lon=0.0,
            tilt_lat=0.0,
            omega_spin=runner.omega_spin,
            tension_n=float(runner.tether._last_info.get("tension", 0.0)),  # noqa: SLF001
        )

        result_trim, _ = runner.aero.compute_forces(
            RotorInputs(
                collective_rad=icgen.STACK_COLL,
                tilt_lon=trim.tilt_lon,
                tilt_lat=trim.tilt_lat,
                R_hub=R,
                v_hub_world=vel,
                wind_world=icgen.WIND,
                omega_rad_s=runner.omega_spin,
                rho_kg_m3=1.225,
            ),
            rotor_state,
        )
        tether_trim, _ = runner.tether.compute(pos, vel, R)
        print_balance(
            "clean wind-aligned start, STACK_COLL, trim cyclic",
            pos,
            vel,
            np.asarray(result_trim.F_world),
            tether_trim,
            gravity,
            collective_rad=icgen.STACK_COLL,
            tilt_lon=trim.tilt_lon,
            tilt_lat=trim.tilt_lat,
            omega_spin=runner.omega_spin,
            tension_n=float(runner.tether._last_info.get("tension", 0.0)),  # noqa: SLF001
        )

        saved_path = SIM / "steady_state_starting.json"
        saved = json.loads(saved_path.read_text(encoding="utf-8"))
        eq = saved["eq_physics"]
        saved_pos = np.asarray(saved["pos"], dtype=float)
        saved_aero = np.array([eq["aero_fx"], eq["aero_fy"], eq["aero_fz"]], dtype=float)
        saved_tether = np.array([eq["tether_fx"], eq["tether_fy"], eq["tether_fz"]], dtype=float)
        saved_gravity = np.array([0.0, 0.0, eq["mass_kg"] * icgen.G])
        saved_vel = np.asarray(saved["vel"], dtype=float)
        print_balance(
            "saved steady_state_starting.json eq_physics",
            saved_pos,
            saved_vel,
            saved_aero,
            saved_tether,
            saved_gravity,
            collective_rad=float(eq["collective_rad"]),
            tilt_lon=float(eq["tilt_lon"]),
            tilt_lat=float(eq["tilt_lat"]),
            omega_spin=float(eq["omega_spin"]),
            tension_n=float(saved.get("tension_eq_n", 0.0)),
        )
        return

    if args.collective_sweep:
        pos = np.asarray(runner.hub_state["pos"], dtype=float)
        vel = np.asarray(runner.hub_state["vel"], dtype=float)
        R = np.asarray(runner.hub_state["R"], dtype=float)
        t_hat = pos / np.linalg.norm(pos)
        gravity = np.array([0.0, 0.0, icgen.MASS * icgen.G])
        rotor_state = runner._core._rotor_state  # noqa: SLF001 - one-off force audit

        print("IC clean-state collective sweep")
        print(f"  pos={pos.tolist()} wind_h={wind_h.tolist()} cross_h={cross_h.tolist()}")
        print(f"  trim_tlon={trim.tilt_lon:+.5f} trim_tlat={trim.tilt_lat:+.5f}")
        print("  F_downwind is +with wind / -against wind. F_up is positive upward.")
        print()

        def eval_force(col: float, tlon: float, tlat: float):
            result, _ = runner.aero.compute_forces(
                RotorInputs(
                    collective_rad=col,
                    tilt_lon=tlon,
                    tilt_lat=tlat,
                    R_hub=R,
                    v_hub_world=vel,
                    wind_world=icgen.WIND,
                    omega_rad_s=runner.omega_spin,
                    rho_kg_m3=1.225,
                ),
                rotor_state,
            )
            tether, _ = runner.tether.compute(pos, vel, R)
            aero = np.asarray(result.F_world, dtype=float)
            net = aero + tether + gravity
            return aero, tether, net

        for label, tlon, tlat in [("zero_cyclic", 0.0, 0.0), ("trim_cyclic", trim.tilt_lon, trim.tilt_lat)]:
            print(label)
            print("  col_rad  F_aero_down  F_net_down  F_aero_up  F_net_up  F_radial_net  F_cross_net")
            rows = []
            for col in [-0.24, -0.21, -0.18, -0.15, -0.12, -0.09, -0.06, -0.03, 0.00, 0.03, 0.06, 0.09]:
                aero, tether, net = eval_force(col, tlon, tlat)
                row = dict(
                    col=col,
                    aero_down=float(np.dot(aero[:2], wind_h)),
                    net_down=float(np.dot(net[:2], wind_h)),
                    aero_up=float(-aero[2]),
                    net_up=float(-net[2]),
                    radial_net=float(np.dot(net, t_hat)),
                    cross_net=float(np.dot(net[:2], cross_h)),
                )
                rows.append(row)
                print(
                    f"  {col:+7.3f} {row['aero_down']:+12.2f} {row['net_down']:+11.2f} "
                    f"{row['aero_up']:+9.2f} {row['net_up']:+8.2f} {row['radial_net']:+12.2f} {row['cross_net']:+12.2f}"
                )

            near = [row for row in rows if -0.21 <= row["col"] <= -0.15]
            slope_down = (near[-1]["aero_down"] - near[0]["aero_down"]) / (near[-1]["col"] - near[0]["col"])
            slope_up = (near[-1]["aero_up"] - near[0]["aero_up"]) / (near[-1]["col"] - near[0]["col"])
            print(
                f"  local slope near STACK_COLL: dF_aero_down/dcol={slope_down:+.1f} N/rad, "
                f"dF_aero_up/dcol={slope_up:+.1f} N/rad"
            )
            print()
        return

    print("IC warmup plane drift audit")
    print(f"  aero_model={runner_model} setup_model={setup_model}")
    print(f"  start_pos={pos0.tolist()} wind_h={wind_h.tolist()} cross_h={cross_h.tolist()}")
    print(f"  trim_tlon={trim.tilt_lon:+.5f} rad trim_tlat={trim.tilt_lat:+.5f} rad")
    print("  cross coordinate is positive along cross_h; for WIND=[0,10,0], cross=-North")
    print()
    print(
        " t[s]      N      E    alt tension   col  v_down air_rel_d  v_cross "
        " F_aero_d  F_teth_d  F_net_d  F_aero_c  F_teth_c  F_net_c  tlon   tlat  roll_sp pitch_sp"
    )

    sample_every = int(round(0.1 / icgen._DT))
    print_every = int(round(5.0 / icgen._DT))
    cmd_every = icgen._PLANNER_EVERY
    ap_every = icgen._AP_EVERY
    rows = []
    tel_rows: list[TelRow] = []
    last_sr = None
    for step in range(icgen.WARMUP_STEPS):
        if step % cmd_every == 0:
            ap.receive_command(
                TensionCommand(
                    tension_target_n=300.0,
                    alt_m=float(-pos0[2]),
                    phase="reel-out",
                ),
                icgen._DT_CMD,
            )
        if step % ap_every == 0:
            ap.tick(step * icgen._DT, runner)
        v_winch = 0.0
        last_sr = runner.step(
            icgen._DT,
            ap.col_rad,
            ap.roll_sp,
            ap.pitch_sp,
            runner.omega_body,
            rest_length=rest_now,
        )

        if step % sample_every == 0 and last_sr is not None:
            pos = np.asarray(runner.hub_state["pos"], dtype=float)
            vel = np.asarray(runner.hub_state["vel"], dtype=float)
            aero = np.asarray(last_sr["aero_result"].F_world, dtype=float)
            tether = np.asarray(last_sr["tether_force"], dtype=float)
            net = aero + tether
            log_fields = ap.log_fields()
            bzg = log_fields["body_z_eq"]
            if args.csv_out is not None:
                tel_rows.append(TelRow.from_physics(
                    runner,
                    last_sr,
                    ap.col_rad,
                    icgen.WIND,
                    body_z_eq=bzg,
                    phase=f"ic_warmup_{runner_model}",
                    tension_feedforward_n=float(log_fields.get("tension_feedforward_n", 0.0)),
                    collective_from_alt_ctrl=float(log_fields.get("collective_from_alt_ctrl", ap.col_rad)),
                    gnd_alt_cmd_m=float(-pos0[2]),
                    winch_speed_ms=float(v_winch),
                    elevation_rad=float(log_fields.get("elevation_rad", 0.0)),
                    comms_ok=bool(log_fields.get("comms_ok", True)),
                    vib_corr=float(log_fields.get("vib_corr", 0.0)),
                    alt_pid_integral=float(log_fields.get("alt_pid_integral", 0.0)),
                    roll_sp_rads=float(ap.roll_sp),
                    pitch_sp_rads=float(ap.pitch_sp),
                ))
            cross = float(np.dot(pos[:2], cross_h))
            v_down = float(np.dot(vel[:2], wind_h))
            air_rel_down = float(np.dot(icgen.WIND[:2], wind_h) - v_down)
            v_cross = float(np.dot(vel[:2], cross_h))
            row = dict(
                t=runner.t_sim,
                N=float(pos[0]),
                E=float(pos[1]),
                alt=float(-pos[2]),
                tension=float(runner.tension_now),
                col=float(ap.col_rad),
                cross=cross,
                v_down=v_down,
                air_rel_down=air_rel_down,
                v_cross=v_cross,
                F_aero_d=float(np.dot(aero[:2], wind_h)),
                F_teth_d=float(np.dot(tether[:2], wind_h)),
                F_net_d=float(np.dot(net[:2], wind_h)),
                F_aero_c=float(np.dot(aero[:2], cross_h)),
                F_teth_c=float(np.dot(tether[:2], cross_h)),
                F_net_c=float(np.dot(net[:2], cross_h)),
                F_aero_N=float(aero[0]),
                F_teth_N=float(tether[0]),
                tlon=float(last_sr.get("tilt_lon", 0.0)),
                tlat=float(last_sr.get("tilt_lat", 0.0)),
                roll_sp=float(ap.roll_sp),
                pitch_sp=float(ap.pitch_sp),
                bz_goal_N=float(bzg[0]) if bzg is not None else 0.0,
            )
            rows.append(row)
            if step % print_every == 0:
                print(
                    f"{row['t']:5.1f} {row['N']:7.2f} {row['E']:7.2f} {row['alt']:6.2f} "
                    f"{row['tension']:7.1f} {row['col']:+6.3f} {row['v_down']:8.3f} "
                    f"{row['air_rel_down']:9.3f} {row['v_cross']:8.3f} "
                    f"{row['F_aero_d']:9.2f} {row['F_teth_d']:9.2f} {row['F_net_d']:8.2f} "
                    f"{row['F_aero_c']:9.2f} {row['F_teth_c']:9.2f} {row['F_net_c']:8.2f} "
                    f"{row['tlon']:+6.3f} {row['tlat']:+6.3f} {row['roll_sp']:+8.3f} "
                    f"{row['pitch_sp']:+8.3f}"
                )

    def window(name: str, start_s: float, end_s: float) -> list[float]:
        vals = [r[name] for r in rows if start_s <= r["t"] <= end_s]
        return vals

    def mean(name: str, start_s: float, end_s: float) -> float:
        vals = window(name, start_s, end_s)
        return float(np.mean(vals)) if vals else float("nan")

    def max_abs(name: str, start_s: float, end_s: float) -> float:
        vals = window(name, start_s, end_s)
        return float(max(vals, key=abs)) if vals else float("nan")

    print()
    for start, end in [(0, 5), (5, 15), (15, 30), (30, 60)]:
        print(
            f"mean {start:02d}-{end:02d}s: "
            f"F_aero_c={mean('F_aero_c', start, end):+.2f} N  "
            f"F_teth_c={mean('F_teth_c', start, end):+.2f} N  "
            f"F_net_c={mean('F_net_c', start, end):+.2f} N  "
            f"F_net_d={mean('F_net_d', start, end):+.2f} N  "
            f"peak_net_c={max_abs('F_net_c', start, end):+.2f} N  "
            f"peak_net_d={max_abs('F_net_d', start, end):+.2f} N  "
            f"v_down={mean('v_down', start, end):+.3f} m/s  "
            f"v_cross={mean('v_cross', start, end):+.3f} m/s  "
            f"cross={mean('cross', start, end):+.2f} m  "
            f"peak_tlon={max_abs('tlon', start, end):+.3f}  "
            f"peak_tlat={max_abs('tlat', start, end):+.3f}"
        )

    final = rows[-1]
    print()
    print(
        f"final: N={final['N']:.2f} E={final['E']:.2f} cross={final['cross']:.2f} "
        f"v_cross={final['v_cross']:.3f}"
    )
    if args.csv_out is not None:
        write_csv(tel_rows, args.csv_out)
        print(f"wrote telemetry CSV: {args.csv_out}")

    if args.compare_compute_ic:
        print()
        print("Running test_generate_ic._compute_ic() for current-code sanity check...")
        ic = icgen._compute_ic()
        pos = np.asarray(ic["pos0"], dtype=float)
        vel = np.asarray(ic["vel0"], dtype=float)
        print(
            f"_compute_ic final: N={pos[0]:.2f} E={pos[1]:.2f} "
            f"cross={float(np.dot(pos[:2], cross_h)):.2f} alt={-pos[2]:.2f} "
            f"v_cross={float(np.dot(vel[:2], cross_h)):.3f} "
            f"tension={ic['T_tether']:.2f} coll={ic['coll_settled']:.4f}"
        )


if __name__ == "__main__":
    main()