"""
test_yaw_rotation_invariance.py — Does the internal controller care about initial yaw?

For an axially symmetric body (Ixx = Iyy), a rotation around disk_normal is a
true symmetry of the physics: forces, torques, and angular momentum are all
unchanged.  The world-frame trajectory (pos, vel, body_z) should be identical
regardless of what yaw we start with.

If the Python internal controller IS yaw-invariant, both runs produce the same
NED trajectory.  If it is NOT (because body-frame cyclic errors depend on yaw),
the trajectories will diverge — and the test quantifies by how much.

This test was written to empirically answer the question:
  "Can we store a GPS-friendly R0 in steady_state_starting.json and use it in
   unit simtests without breaking the internal controller?"
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rotor_definition as rd
from dynamics    import RigidBodyDynamics
from aero        import create_aero
from tether      import TetherModel
from frames      import build_gps_yaw_frame
from controller  import OrbitTracker, AcroController
from simtest_log import SimtestLog, BadEventLog
from simtest_ic  import load_ic

_log = SimtestLog(__file__)
_IC  = load_ic()

DT           = 1.0 / 400.0
T_SIM        = 60.0
ANCHOR       = np.zeros(3)
ANCHOR.flags.writeable = False
WIND         = np.array([0.0, 10.0, 0.0])
WIND.flags.writeable = False
T_AERO_OFFSET = 45.0
BASE_K_ANG   = 50.0
OMEGA_SPIN_MIN = 0.5
Z_FLOOR      = -1.0


def _run(R0: np.ndarray) -> dict:
    """Run 60 s orbit starting from the given R0; return trajectory summary."""
    rotor  = rd.default()
    body_z = R0[:, 2]

    dyn = RigidBodyDynamics(
        mass=rotor.mass_kg, I_body=rotor.I_body_kgm2, I_spin=0.0,
        pos0=_IC.pos.copy(), vel0=_IC.vel.copy(),
        R0=R0, omega0=[0.0, 0.0, 0.0], z_floor=Z_FLOOR,
    )
    aero   = create_aero(rotor)
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=_IC.rest_length,
                         axle_attachment_length=rotor.axle_attachment_length_m)

    omega_spin   = float(_IC.omega_spin)
    hub_state    = dyn.state
    orbit_tracker = OrbitTracker(body_z, _IC.pos / np.linalg.norm(_IC.pos),
                                 rotor.body_z_slew_rate_rad_s)
    acro   = AcroController.from_rotor(rotor)
    events = BadEventLog()

    pos_hist = []
    bz_hist  = []

    for i in range(int(T_SIM / DT)):
        t = i * DT
        body_z_eq = orbit_tracker.update(hub_state["pos"], DT)
        tilt_lon, tilt_lat = acro.update(hub_state, body_z_eq, DT,
                                         swashplate_phase_deg=0.0)
        result = aero.compute_forces(
            collective_rad=_IC.stack_coll_eq,
            tilt_lon=tilt_lon, tilt_lat=tilt_lat,
            R_hub=hub_state["R"], v_hub_world=hub_state["vel"],
            omega_rotor=omega_spin, wind_world=WIND,
            t=T_AERO_OFFSET + t,
        )
        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        M_orbital = result.M_orbital + tm - BASE_K_ANG * hub_state["omega"]
        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / 10.0 * DT)
        hub_state = dyn.step(result.F_world + tf, M_orbital, DT,
                             omega_spin=omega_spin)
        pos_hist.append(hub_state["pos"].copy())
        bz_hist.append(hub_state["R"][:, 2].copy())
        events.check_floor(hub_state["pos"][2], t, "flight")

    pos_arr = np.array(pos_hist)
    bz_arr  = np.array(bz_hist)
    alts    = -pos_arr[:, 2]
    return {
        "pos":    pos_arr,
        "bz":     bz_arr,
        "min_alt": float(alts.min()),
        "events": events,
    }


@pytest.mark.simtest
@pytest.mark.timeout(180)
def test_gps_friendly_yaw_orbit_invariance():
    """
    Two runs with different initial yaw — same orbit should result:

      eq       : equilibrium R0 (build_orb_frame, body_x = East projected) — baseline
      gps_body : GPS-friendly R0 (body_x horizontal), AcroController (body frame)

    Both runs use the same AcroController.  For an axially-symmetric body
    (Ixx = Iyy), a yaw rotation is a physical symmetry and must not change the
    orbit.  If the aero model and controller are consistent — both using the hub
    body frame — the orbit is invariant under yaw.

    Root cause of the old failure: the aero model's Coleman correction computed
    psi_skew (the azimuth of the in-plane wind) in a fixed East-projected frame,
    while phi_az (blade azimuth) is measured from R_hub[:,0].  They only matched
    when R_hub was built with build_orb_frame (body_x = East).  The fix is to
    compute psi_skew in body frame (from R_hub[:,0], R_hub[:,1]) so the two
    azimuth references are always consistent.
    """
    body_z  = _IC.R0[:, 2]
    R0_eq   = _IC.R0
    R0_gps  = build_gps_yaw_frame(body_z)

    cos_delta     = float(np.clip(np.dot(R0_eq[:, 0], R0_gps[:, 0]), -1.0, 1.0))
    delta_yaw_deg = float(np.degrees(np.arccos(cos_delta)))

    r_eq       = _run(R0_eq)    # baseline
    r_gps_body = _run(R0_gps)  # GPS-friendly R0, body-frame controller

    def _err(r):
        pos_err = np.linalg.norm(r["pos"] - r_eq["pos"], axis=1)
        t_div   = float(np.argmax(pos_err > 1.0) * DT) if (pos_err > 1.0).any() else T_SIM
        return float(pos_err.max()), t_div

    body_max, body_tdiv = _err(r_gps_body)

    _log.write(
        [
            f"delta_yaw         = {delta_yaw_deg:.1f} deg",
            f"gps_body max_err  = {body_max:.3f} m   t_diverge_1m={body_tdiv:.1f} s"
            f"  events={r_gps_body['events'].summary() or 'none'}",
            f"eq       min_alt  = {r_eq['min_alt']:.2f} m",
            f"gps_body min_alt  = {r_gps_body['min_alt']:.2f} m",
        ],
        f"delta_yaw={delta_yaw_deg:.1f}deg  "
        f"body_err={body_max:.2f}m@{body_tdiv:.1f}s",
    )

    print(f"\n[yaw_invariance] delta_yaw={delta_yaw_deg:.1f} deg")
    print(f"  gps+body_frame: max_err={body_max:.3f} m  t_diverge={body_tdiv:.1f} s  "
          f"stable={not r_gps_body['events'] and r_gps_body['min_alt'] > 1.5}")

    # Baseline must be stable.
    assert not r_eq["events"], f"eq run crashed: {r_eq['events'].summary()}"
    assert r_eq["min_alt"] > 1.5, f"eq run too low: {r_eq['min_alt']:.2f} m"

    # GPS-friendly R0 with body-frame AcroController must also be stable and invariant.
    assert not r_gps_body["events"], \
        f"gps+body_frame crashed — aero/controller mismatch not fixed: {r_gps_body['events'].summary()}"
    assert r_gps_body["min_alt"] > 1.5, \
        f"gps+body_frame too low: {r_gps_body['min_alt']:.2f} m"
    assert body_max < 5.0, \
        f"gps+body_frame diverged {body_max:.2f} m from baseline — not yaw-invariant"
