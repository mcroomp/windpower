"""
diagnose_landing.py -- Runs test_landing simulation and prints detailed diagnostics.

Usage:
    simulation/.venv/Scripts/python.exe simulation/analysis/diagnose_landing.py
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "unit"))

import rotor_definition as rd
from dynamics        import RigidBodyDynamics
from aero            import create_aero
from tether          import TetherModel
from controller      import OrbitTracker, col_min_for_altitude_rad, AcroController
from planner         import DeschutterPlanner, WindEstimator, quat_apply, quat_is_identity
from landing_planner import LandingPlanner
from winch           import WinchController
from simtest_ic      import load_ic

_IC    = load_ic()
_ROTOR = rd.default()

DT            = 1.0 / 400.0
ANCHOR        = np.zeros(3)
T_AERO_OFFSET = 45.0
WIND          = np.array([0.0, 10.0, 0.0])

T_REEL_OUT   =  0.0
T_REEL_IN    = 30.0
V_REEL_IN    =  0.4
XI_REEL_IN_DEG   = 80.0
BODY_Z_SLEW_RATE = _ROTOR.body_z_slew_rate_rad_s
_xi_start_deg = 30.0
T_TRANSITION  = math.radians(XI_REEL_IN_DEG - _xi_start_deg) / BODY_Z_SLEW_RATE + 1.5
TENSION_OUT   = 200.0
TENSION_IN    =  55.0
COL_MIN_RAD   = -0.28
COL_MAX_RAD   =  0.10
_AERO_TMP     = create_aero(_ROTOR)
COL_MIN_REEL_IN_RAD = col_min_for_altitude_rad(_AERO_TMP, XI_REEL_IN_DEG, _ROTOR.mass_kg)
TENSION_SAFETY_N = 496.0
V_LAND        = 0.5
COL_CRUISE    = 0.079
MIN_TETHER_M  = 2.0
T_FINAL_DROP_MAX = 15.0
T_PUMPING_END = T_REEL_OUT + T_REEL_IN
FLOOR_ALT_M   = 1.0


def _body_x_yaw_deg(R):  # noqa: ARG001
    """Yaw of body_x projected onto horizontal plane, degrees from East."""
    bx = R[:, 0]
    # East-projected reference
    dn = R[:, 2]
    east = np.array([0.0, 1.0, 0.0])
    ep = east - np.dot(east, dn) * dn
    norm = np.linalg.norm(ep)
    if norm < 1e-6:
        return 0.0
    bx_east = ep / norm
    by_east = np.cross(dn, bx_east)
    return math.degrees(math.atan2(np.dot(bx, by_east), np.dot(bx, bx_east)))


def _psi_skew_old(R_hub, v_rel, disk_normal):
    """Old East-based psi_skew."""
    v_ax = float(np.dot(v_rel, disk_normal))
    v_ip_vec = v_rel - v_ax * disk_normal
    v_ip = float(np.linalg.norm(v_ip_vec))
    if v_ip < 0.01:
        return 0.0
    v_ip_unit = v_ip_vec / v_ip
    east = np.array([0.0, 1.0, 0.0])
    ep = east - np.dot(east, disk_normal) * disk_normal
    if np.linalg.norm(ep) < 1e-6:
        north = np.array([1.0, 0.0, 0.0])
        ep = north - np.dot(north, disk_normal) * disk_normal
    bx = ep / np.linalg.norm(ep)
    by = np.cross(disk_normal, bx)
    return math.atan2(float(np.dot(v_ip_unit, by)), float(np.dot(v_ip_unit, bx)))


def _psi_skew_new(R_hub, v_rel, disk_normal):
    """New body-frame psi_skew."""
    v_ax = float(np.dot(v_rel, disk_normal))
    v_ip_vec = v_rel - v_ax * disk_normal
    v_ip = float(np.linalg.norm(v_ip_vec))
    if v_ip < 0.01:
        return 0.0
    v_ip_unit = v_ip_vec / v_ip
    bx = R_hub[:, 0]
    by = R_hub[:, 1]
    return math.atan2(float(np.dot(v_ip_unit, by)), float(np.dot(v_ip_unit, bx)))


def run():
    dyn    = RigidBodyDynamics(
        mass=_ROTOR.mass_kg, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=_IC.pos.tolist(), vel0=_IC.vel.tolist(),
        R0=_IC.R0, omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(rd.default())
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=_IC.rest_length,
                         axle_attachment_length=_ROTOR.axle_attachment_length_m)
    winch  = WinchController(rest_length=_IC.rest_length,
                              T_max_n=TENSION_SAFETY_N,
                              min_length=MIN_TETHER_M)
    trajectory = DeschutterPlanner(
        t_reel_out=T_REEL_OUT, t_reel_in=T_REEL_IN, t_transition=T_TRANSITION,
        v_reel_out=0.4, v_reel_in=V_REEL_IN,
        tension_out=TENSION_OUT, tension_in=TENSION_IN,
        wind_estimator=WindEstimator(seed_wind_ned=WIND),
        col_min_rad=COL_MIN_RAD, col_max_rad=COL_MAX_RAD,
        xi_reel_in_deg=XI_REEL_IN_DEG,
        col_min_reel_in_rad=COL_MIN_REEL_IN_RAD,
    )
    orbit_tracker = OrbitTracker(_IC.R0[:, 2], _IC.pos / np.linalg.norm(_IC.pos), BODY_Z_SLEW_RATE)
    acro = AcroController.from_rotor(rd.default(), use_servo=True)

    hub_state   = dyn.state
    omega_spin  = _IC.omega_spin
    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now    = tether._last_info.get("tension", 0.0)
    collective_rad = _IC.coll_eq_rad

    outer_phase     = "pumping"
    landing_planner = None
    land_cmd        = None
    land_phase      = None
    t_final_start   = None
    floor_hit       = False

    descent_time = _IC.rest_length / V_LAND + 10.0
    max_steps = int((T_PUMPING_END + descent_time + T_FINAL_DROP_MAX) / DT)

    # Snapshot every 5 s
    SNAP_INTERVAL = 5.0
    next_snap = 0.0

    MASS   = _ROTOR.mass_kg
    I_SPIN = 10.0
    G      = 9.81

    def total_energy(hs, ospin):
        pos = hs["pos"]; vel = hs["vel"]
        ke_trans = 0.5 * MASS * float(np.dot(vel, vel))
        pe_grav  = MASS * G * (-float(pos[2]))   # alt = -pos_z
        ke_spin  = 0.5 * I_SPIN * ospin**2
        return ke_trans + pe_grav + ke_spin

    E_prev    = None
    t_prev    = None
    P_aero_acc = 0.0   # accumulated aero power over snapshot interval
    n_acc      = 0

    print(f"{'t_s':>7}  {'phase':<12}  {'alt_m':>7}  {'tether_m':>9}  {'tension_N':>8}  "
          f"{'omega_rpm':>9}  {'T_N':>7}  {'coll':>7}  {'col_i':>7}  "
          f"{'E_mech_J':>10}  {'dE/dt_W':>9}  {'P_aero_W':>9}  {'balance_W':>10}")
    print("-" * 125)

    for i in range(max_steps):
        t_sim    = i * DT
        altitude = -hub_state["pos"][2]
        R_hub    = hub_state["R"]
        disk_normal = R_hub[:, 2]
        v_rel    = WIND - hub_state["vel"]

        if t_sim >= next_snap:
            T_thrust  = getattr(aero, 'last_T', float('nan'))
            phase_str = f"{outer_phase}/{land_phase}" if land_phase else outer_phase
            omega_rpm = omega_spin * 60 / (2 * math.pi)
            E_now     = total_energy(hub_state, omega_spin)
            if E_prev is not None and t_prev is not None:
                dt_snap  = t_sim - t_prev
                dE_dt    = (E_now - E_prev) / dt_snap
                P_aero_m = P_aero_acc / max(n_acc, 1)
                balance  = dE_dt - P_aero_m
            else:
                dE_dt = P_aero_m = balance = float('nan')
            col_i = land_cmd.get("col_i_rad", float('nan')) if land_cmd is not None else float('nan')
            print(f"{t_sim:7.1f}  {phase_str:<12}  {altitude:7.2f}  "
                  f"{winch.rest_length:9.2f}  {tension_now:8.1f}  "
                  f"{omega_rpm:9.1f}  {T_thrust:7.1f}  {collective_rad:7.4f}  {col_i:7.4f}  "
                  f"{E_now:10.0f}  {dE_dt:9.1f}  {P_aero_m:9.1f}  {balance:10.1f}")
            E_prev     = E_now
            t_prev     = t_sim
            P_aero_acc = 0.0
            n_acc      = 0
            next_snap += SNAP_INTERVAL

        # Phase switch
        if outer_phase == "pumping" and t_sim >= T_PUMPING_END:
            outer_phase = "landing"
            landing_planner = LandingPlanner(
                initial_body_z=orbit_tracker.bz_slerp,
                v_land=V_LAND, col_cruise=COL_CRUISE,
                min_tether_m=MIN_TETHER_M, anchor_ned=ANCHOR,
            )

        if outer_phase == "pumping":
            state_pkt = {
                "pos_ned": hub_state["pos"], "vel_ned": hub_state["vel"],
                "omega_spin": omega_spin, "body_z": hub_state["R"][:, 2],
                "tension_n": tension_now, "tether_length_m": winch.rest_length,
            }
            pump_cmd = trajectory.step(state_pkt, DT)
            winch.step(tension_now, DT)
            tether.rest_length = winch.rest_length
            _aq = pump_cmd["attitude_q"]
            _bz_target = (None if quat_is_identity(_aq)
                          else quat_apply(_aq, np.array([0.0, 0.0, -1.0])))
            body_z_eq = orbit_tracker.update(hub_state["pos"], DT, _bz_target)
            collective_rad = pump_cmd["collective_rad"]
            tilt_lon, tilt_lat = acro.update(hub_state, body_z_eq, DT,
                                             swashplate_phase_deg=0.0)
        else:
            state_pkt = {
                "pos_ned": hub_state["pos"], "vel_ned": hub_state["vel"],
                "body_z": hub_state["R"][:, 2],
                "tension_n": tension_now, "tether_length_m": winch.rest_length,
            }
            land_cmd = landing_planner.step(state_pkt, DT)
            winch.step(tension_now, DT)
            tether.rest_length = winch.rest_length
            land_phase = land_cmd["phase"]
            body_z_eq  = land_cmd["body_z_eq"]
            if land_phase == "final_drop":
                collective_rad = acro.slew_collective(0.0, DT)
                if t_final_start is None:
                    t_final_start = t_sim
                    print(f"\n  *** final_drop triggered at t={t_sim:.1f}s  alt={altitude:.2f}m  tether={winch.rest_length:.2f}m")
                if t_final_start is not None and (t_sim - t_final_start) > T_FINAL_DROP_MAX:
                    print(f"  *** final_drop timeout at t={t_sim:.1f}s")
                    break
            else:
                collective_rad = land_cmd["collective_rad"]
            tilt_lon, tilt_lat = acro.update(hub_state, body_z_eq, DT,
                                             swashplate_phase_deg=0.0)

        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        tension_now = tether._last_info.get("tension", 0.0)
        result = aero.compute_forces(
            collective_rad=collective_rad,
            tilt_lon=tilt_lon, tilt_lat=tilt_lat,
            R_hub=hub_state["R"], v_hub_world=hub_state["vel"],
            omega_rotor=omega_spin, wind_world=WIND,
            t=T_AERO_OFFSET + t_sim,
        )

        # Accumulate aero power for energy balance (tether excluded — rest_length changes make PE accounting discontinuous)
        v_now = hub_state["vel"]
        P_aero_acc += float(np.dot(result.F_world, v_now)) + aero.last_Q_spin * omega_spin
        n_acc += 1

        K_BASE_ANG = 50.0
        M_orbital  = result.M_orbital + tm - K_BASE_ANG * hub_state["omega"]
        omega_spin = max(0.5, omega_spin + aero.last_Q_spin / 10.0 * DT)
        hub_state  = dyn.step(result.F_world + tf, M_orbital, DT, omega_spin=omega_spin)

        altitude = -hub_state["pos"][2]
        if altitude <= FLOOR_ALT_M and not floor_hit:
            floor_hit = True
            print(f"\n  *** FLOOR HIT at t={t_sim:.1f}s  alt={altitude:.2f}m  tether={winch.tether_length_m:.2f}m")
            break

    if not floor_hit:
        print(f"\n  *** NO FLOOR HIT  t_end={t_sim:.1f}s  alt={altitude:.2f}m  tether={winch.tether_length_m:.2f}m")


if __name__ == "__main__":
    run()
