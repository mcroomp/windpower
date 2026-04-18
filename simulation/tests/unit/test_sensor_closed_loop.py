"""
test_sensor_closed_loop.py — PhysicalSensor properties during 60-second flight.

Fills the validation gap between:
  - test_sensor.py          (unit tests at static/hand-crafted states)
  - full stack tests        (Docker required)

The control loop uses truth state (same as test_closed_loop_90s) to keep the
flight stable.  PhysicalSensor runs as a shadow computation alongside it,
receiving the same physics state at 400 Hz.  Sensor outputs are checked for
consistency properties that the EKF depends on.

Physical sensor design principle
----------------------------------
sensor.py is physically faithful: rpy[2] is the actual hub orientation yaw
from build_orb_frame(disk_normal), NOT a velocity-heading override.  The SITL
compass reads the same R_orb orientation, so compass and reported attitude always
agree → no EKF emergency yaw reset.

The GB4008 counter-torque motor keeps the electronics non-rotating relative to
the rotor spin axis, modelled as a K_YAW damping torque in the dynamics ODE.
The sensor faithfully reports whatever angular velocity the body has — no
stripping.  The yaw of the electronics platform is whatever build_orb_frame
gives (East-projection onto disk plane); its exact direction is unimportant
as long as it is steady.

What this specifically tests
-----------------------------
1. Sensor yaw tracks R_orb (honest orientation):
     rpy[2] must equal atan2(R[1,0], R[0,0]) from R_orb throughout flight.
     This verifies no yaw override is applied.

2. Sensor yaw changes slowly (orbital rate):
     The hub's orbital frame yaw changes at ~0.01-0.02 rad/s as disk_normal
     rotates during orbit.  Rapid changes (>1 rad/s) indicate a frame bug.

3. Gyro boundedness:
     gyro_body norm must stay below GYRO_MAX_RAD_S throughout.  A large value
     indicates a sensor or K_YAW dynamics bug.

4. Gyro spin isolation:
     The GB4008 motor keeps the electronics non-rotating via K_YAW dynamics.
     In steady flight, gyro_body norm must remain << omega_spin (the rotor
     spin speed), because the body itself is not spinning.

Sensor configuration
--------------------
gyro_sigma = accel_sigma = 0.0 — noiseless for determinism.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(120)]

import rotor_definition as rd
from dynamics    import RigidBodyDynamics
from aero        import create_aero
from tether      import TetherModel
from controller  import orbit_tracked_body_z_eq, compute_rate_cmd, RatePID
from swashplate  import SwashplateServoModel
from planner     import HoldPlanner
from sensor      import PhysicalSensor
from simtest_log import SimtestLog, BadEventLog
from simtest_ic  import load_ic

_log = SimtestLog(__file__)
_IC  = load_ic()

DT            = 1.0 / 400.0
T_SIM         = 60.0
T_AERO_OFFSET = 45.0
ANCHOR        = np.zeros(3)
POS0          = _IC.pos
VEL0          = _IC.vel
BODY_Z0       = _IC.R0[:, 2]
OMEGA_SPIN0   = _IC.omega_spin

I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

KP_OUTER = 2.5
KP_INNER = RatePID.DEFAULT_KP

WIND = np.array([0.0, 10.0, 0.0])   # NED: East wind

# Maximum plausible gyro_body norm [rad/s].  Orbital rate is ~0.2-0.3 rad/s;
# 5 rad/s is a generous upper bound that catches body-rate anomalies without
# being sensitive to short transients.
GYRO_MAX_RAD_S = 5.0

# Gyro must be well below rotor spin (factor by which omega_spin exceeds gyro).
# At omega_spin ~28 rad/s and orbital rate ~0.3 rad/s, the ratio is ~90x.
# Requiring 10x gives significant headroom; K_YAW dynamics keep body spin
# near zero so this ratio should be met easily.
SPIN_ISOLATION_FACTOR = 10.0


def _run(t_sim: float = T_SIM):
    """
    Run 60 s of flight with truth-state control.  PhysicalSensor runs in shadow.

    Returns
    -------
    history    : list of 10 Hz snapshots with pos, omega_spin, floor_hits
    sensor_log : list of 10 Hz sensor snapshots with yaw_sensor, yaw_true,
                 gyro_norm, omega_spin
    """
    dyn = RigidBodyDynamics(
        mass=5.0, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS0.tolist(), vel0=VEL0.tolist(),
        R0=_IC.R0, omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(rd.default())
    tether = TetherModel(anchor_ned=ANCHOR,
                         rest_length=_IC.rest_length,
                         axle_attachment_length=rd.default().axle_attachment_length_m)

    sensor = PhysicalSensor(
        home_ned_z  = _IC.home_z_ned,
        gyro_sigma  = 0.0,
        accel_sigma = 0.0,
        rng_seed    = 0,
    )

    hub_state    = dyn.state
    omega_spin   = OMEGA_SPIN0
    prev_vel     = hub_state["vel"].copy()

    trajectory     = HoldPlanner()
    ic_tether_dir0 = POS0 / np.linalg.norm(POS0)
    ic_body_z_eq0  = BODY_Z0.copy()

    pid_lon = RatePID(kp=KP_INNER)
    pid_lat = RatePID(kp=KP_INNER)
    servo   = SwashplateServoModel.from_rotor(rd.default())

    events     = BadEventLog()
    history    = []
    sensor_log = []   # 10 Hz

    for i in range(int(t_sim / DT)):
        t = i * DT

        # ----------------------------------------------------------------
        # Shadow sensor: compute sensor packet from current truth state.
        # Not fed back into the controller — used for sensor property checks.
        # ----------------------------------------------------------------
        accel_world_ned = (hub_state["vel"] - prev_vel) / DT if i > 0 else np.zeros(3)
        pkt = sensor.compute(
            pos_ned         = hub_state["pos"],
            vel_ned         = hub_state["vel"],
            R_hub           = hub_state["R"],
            omega_body      = hub_state["omega"],   # world-frame orbital omega
            accel_world_ned = accel_world_ned,
            dt              = DT,
        )

        # ----------------------------------------------------------------
        # Truth-state control (same as test_closed_loop_90s.py)
        # ----------------------------------------------------------------
        state_pkt = {
            "pos_ned":    hub_state["pos"],
            "vel_ned":    hub_state["vel"],
            "tension_n":  0.0,
            "omega_spin": omega_spin,
            "t_free":     t,
        }
        trajectory.step(state_pkt, DT)

        body_z_eq = orbit_tracked_body_z_eq(hub_state["pos"], ic_tether_dir0, ic_body_z_eq0)
        bz_now    = hub_state["R"][:, 2]
        rate_sp   = compute_rate_cmd(bz_now, body_z_eq, hub_state["R"],
                                     kp=KP_OUTER, kd=0.0)

        omega_body          = hub_state["R"].T @ hub_state["omega"]
        omega_body_orbital  = omega_body.copy()
        omega_body_orbital[2] = 0.0
        tilt_lon_cmd =  pid_lon.update(rate_sp[0], omega_body_orbital[0], DT)
        tilt_lat_cmd = -pid_lat.update(rate_sp[1], omega_body_orbital[1], DT)
        tilt_lon, tilt_lat = servo.step(tilt_lon_cmd, tilt_lat_cmd, DT)

        result = aero.compute_forces(
            collective_rad = _IC.stack_coll_eq,
            tilt_lon       = tilt_lon,
            tilt_lat       = tilt_lat,
            R_hub          = hub_state["R"],
            v_hub_world    = hub_state["vel"],
            omega_rotor    = omega_spin,
            wind_world     = WIND,
            t              = T_AERO_OFFSET + t,
        )

        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        F_net     = result.F_world + tf
        M_orbital = result.M_orbital + tm

        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)

        prev_vel  = hub_state["vel"].copy()
        hub_state = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)
        events.check_floor(hub_state["pos"][2], t, "flight")

        # 10 Hz snapshot
        if i % int(10.0 / DT) == 0:
            history.append({
                "t":          t,
                "pos":        hub_state["pos"].copy(),
                "omega_spin": omega_spin,
            })
            sensor_log.append({
                "t":              t,
                "yaw_sensor":     float(pkt["rpy"][2]),
                "orb_yaw":        float(pkt["orb_yaw_rad"]),
                "gyro_norm":      float(np.linalg.norm(pkt["gyro_body"])),
                "omega_spin":     omega_spin,
                "R":              hub_state["R"].copy(),
            })

    history.append({
        "t":          t_sim,
        "pos":        hub_state["pos"].copy(),
        "omega_spin": omega_spin,
    })
    return dict(events=events, history=history, sensor_log=sensor_log)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_sensor_stable():
    """Flight must stay above floor and maintain spin throughout 60 s."""
    r = _run()
    history = r["history"]
    min_alt = min(-s["pos"][2] for s in history)
    min_spin = min(s["omega_spin"] for s in history)
    lines = [f"t={s['t']:.1f}  alt={-s['pos'][2]:.2f} m  spin={s['omega_spin']:.1f}" for s in history]
    _log.write(
        lines,
        f"min_alt={min_alt:.2f} m  min_spin={min_spin:.2f} rad/s  {r['events'].summary()}",
    )
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    if min_alt < 1.0:
        failures.append(f"altitude dropped below 1 m: min_alt={min_alt:.2f} m")
    if min_spin < 5.0:
        failures.append(f"spin collapsed: min={min_spin:.2f} rad/s")
    assert not failures, "\n  ".join(failures)


def test_sensor_properties():
    """PhysicalSensor outputs must satisfy physical consistency constraints during flight."""
    r = _run()
    sensor_log = r["sensor_log"]
    failures = []

    # 1. rpy[2] must equal orb_yaw_rad (no velocity-heading override)
    yaw_violations = []
    for s in sensor_log:
        diff = abs(s["yaw_sensor"] - s["orb_yaw"])
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        if abs(diff) > 1e-9:
            yaw_violations.append(
                f"t={s['t']:.1f}s  rpy[2]={math.degrees(s['yaw_sensor']):.3f}  "
                f"orb_yaw={math.degrees(s['orb_yaw']):.3f}  diff={math.degrees(diff):.4f} deg"
            )
    if yaw_violations:
        failures.append(
            f"rpy[2] != orb_yaw_rad in {len(yaw_violations)} samples:\n    "
            + "\n    ".join(yaw_violations[:5])
        )

    # 2. Orbital yaw rate must stay below 1 rad/s
    YAW_RATE_MAX = 1.0
    yaw_rate_violations = []
    prev = None
    for s in sensor_log:
        if prev is not None:
            dt_snap = s["t"] - prev["t"]
            if dt_snap > 0:
                delta = s["orb_yaw"] - prev["orb_yaw"]
                delta = (delta + math.pi) % (2 * math.pi) - math.pi
                rate = abs(delta) / dt_snap
                if rate > YAW_RATE_MAX:
                    yaw_rate_violations.append(f"t={s['t']:.1f}s  yaw_rate={math.degrees(rate):.1f} deg/s")
        prev = s
    if yaw_rate_violations:
        failures.append(
            f"orbital yaw rate exceeded 1 rad/s in {len(yaw_rate_violations)} samples:\n    "
            + "\n    ".join(yaw_rate_violations[:5])
        )

    # 3. Gyro norm bounded
    peak_gyro = max(s["gyro_norm"] for s in sensor_log)
    if peak_gyro > GYRO_MAX_RAD_S:
        failures.append(
            f"gyro_body peak {peak_gyro:.3f} rad/s exceeded {GYRO_MAX_RAD_S} rad/s"
        )

    # 4. Gyro well below rotor spin
    min_ratio = float("inf")
    for s in sensor_log:
        if s["gyro_norm"] >= 1e-6:
            min_ratio = min(min_ratio, s["omega_spin"] / s["gyro_norm"])
    if min_ratio < SPIN_ISOLATION_FACTOR:
        failures.append(
            f"body spin in gyro: min(omega_spin/gyro_norm)={min_ratio:.1f} "
            f"(required >= {SPIN_ISOLATION_FACTOR})"
        )

    # 5. Electronics x-axis (yaw DOF) changes slowly
    elec_yaw_violations = []
    prev = None
    for s in sensor_log:
        if prev is not None:
            dt_snap = s["t"] - prev["t"]
            if dt_snap > 0:
                cos_a = float(np.clip(np.dot(s["R"][:, 0], prev["R"][:, 0]), -1.0, 1.0))
                rate  = math.acos(cos_a) / dt_snap
                if rate > YAW_RATE_MAX:
                    elec_yaw_violations.append(
                        f"t={s['t']:.1f}s  yaw_rate={math.degrees(rate):.1f} deg/s"
                    )
        prev = s
    if elec_yaw_violations:
        failures.append(
            f"electronics yaw R[:,0] changed too rapidly in {len(elec_yaw_violations)} samples:\n    "
            + "\n    ".join(elec_yaw_violations[:5])
        )

    # 6. R_hub must remain a valid rotation matrix
    R_violations = []
    for s in sensor_log:
        R = s["R"]
        det  = float(np.linalg.det(R))
        orth = float(np.max(np.abs(R.T @ R - np.eye(3))))
        if abs(det - 1.0) > 1e-6 or orth > 1e-6:
            R_violations.append(f"t={s['t']:.1f}s  det={det:.8f}  max_orth_err={orth:.2e}")
    if R_violations:
        failures.append(
            f"R_hub not a valid rotation matrix in {len(R_violations)} samples:\n    "
            + "\n    ".join(R_violations[:5])
        )

    _log.write(
        [f"peak_gyro={peak_gyro:.4f} rad/s  min_ratio={min_ratio:.1f}  "
         f"yaw_violations={len(yaw_violations)}  yaw_rate_violations={len(yaw_rate_violations)}  "
         f"R_violations={len(R_violations)}"],
        f"failures={len(failures)}",
    )
    assert not failures, "\n  ".join(failures)
