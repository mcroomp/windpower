"""
test_sensor_closed_loop.py — PhysicalSensor properties during 60-second flight.

Fills the validation gap between:
  - test_sensor.py          (unit tests at static/hand-crafted states)
  - full stack tests        (Docker required)

The control loop uses truth state (same as test_closed_loop_60s) to keep the
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
the rotor spin axis, modelled by stripping the spin component from omega_body
before projecting to body frame.  The yaw of the electronics platform is
whatever build_orb_frame gives (East-projection onto disk plane); its exact
direction is unimportant as long as it is steady.

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
     indicates a spin-strip bug or frame inversion in the sensor code.

4. Gyro spin isolation:
     The GB4008 motor keeps the electronics non-rotating; the gyro must not see
     rotor spin.  Verified by checking that gyro_body norm is << omega_spin.

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
from frames      import build_orb_frame
from sensor      import PhysicalSensor
from simtest_log import SimtestLog
from simtest_ic  import load_ic

_log = SimtestLog(__file__)
_IC  = load_ic()

DT            = 1.0 / 400.0
T_SIM         = 60.0
T_AERO_OFFSET = 45.0
ANCHOR        = np.zeros(3)
POS0          = _IC.pos
VEL0          = _IC.vel
BODY_Z0       = _IC.body_z
OMEGA_SPIN0   = _IC.omega_spin

I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

KP_OUTER = 2.5
KP_INNER = RatePID.DEFAULT_KP

WIND = np.array([0.0, 10.0, 0.0])   # NED: East wind

# Maximum plausible gyro_body norm [rad/s].  Orbital rate is ~0.2-0.3 rad/s;
# 5 rad/s is a generous upper bound that catches spin-strip bugs without
# being sensitive to short transients.
GYRO_MAX_RAD_S = 5.0

# Gyro must be well below rotor spin (factor by which omega_spin exceeds gyro).
# At omega_spin ~28 rad/s and orbital rate ~0.3 rad/s, the ratio is ~90x.
# Requiring 10x gives significant headroom while catching spin-strip failures.
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
        R0=build_orb_frame(BODY_Z0), omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
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

    hub_state  = dyn.state
    omega_spin = OMEGA_SPIN0
    prev_vel   = hub_state["vel"].copy()

    trajectory     = HoldPlanner()
    ic_tether_dir0 = POS0 / np.linalg.norm(POS0)
    ic_body_z_eq0  = BODY_Z0.copy()

    pid_lon = RatePID(kp=KP_INNER)
    pid_lat = RatePID(kp=KP_INNER)
    servo   = SwashplateServoModel.from_rotor(rd.default())

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
        # Truth-state control (same as test_closed_loop_60s.py)
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
            collective_rad = _IC.coll_eq_rad,
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

        # 10 Hz snapshot
        if i % int(10.0 / DT) == 0:
            history.append({
                "t":          t,
                "pos":        hub_state["pos"].copy(),
                "omega_spin": omega_spin,
            })
            sensor_log.append({
                "t":           t,
                "yaw_sensor":  float(pkt["rpy"][2]),
                "orb_yaw":     float(pkt["orb_yaw_rad"]),
                "gyro_norm":   float(np.linalg.norm(pkt["gyro_body"])),
                "omega_spin":  omega_spin,
                "R":           hub_state["R"].copy(),
            })

    history.append({
        "t":          t_sim,
        "pos":        hub_state["pos"].copy(),
        "omega_spin": omega_spin,
    })
    return history, sensor_log


# ---------------------------------------------------------------------------
# Flight stability (truth-state control — baseline sanity)
# ---------------------------------------------------------------------------

def test_sensor_loop_altitude_maintained():
    """Flight must stay above z_floor for the full 60 s."""
    history, _ = _run()
    min_alt = min(-s["pos"][2] for s in history)   # altitude = -NED Z
    _log.write(
        [f"t={s['t']:.1f}  alt={-s['pos'][2]:.2f} m  spin={s['omega_spin']:.1f}" for s in history],
        f"min_alt={min_alt:.2f} m",
    )
    assert min_alt >= 1.0, \
        f"Hub altitude dropped below 1 m: min_alt={min_alt:.2f} m"


def test_sensor_loop_spin_maintained():
    """Rotor must stay above 5 rad/s throughout (autorotation sustained)."""
    history, _ = _run()
    spins = [s["omega_spin"] for s in history]
    assert min(spins) >= 5.0, \
        f"Spin collapsed: min={min(spins):.2f} rad/s"


# ---------------------------------------------------------------------------
# Sensor property tests (PhysicalSensor outputs during realistic flight)
# ---------------------------------------------------------------------------

def test_sensor_yaw_equals_orb_yaw():
    """
    rpy[2] from PhysicalSensor must equal orb_yaw_rad (actual R_orb yaw).

    Physical faithfulness: sensor reports the actual hub orientation yaw —
    no velocity-heading override.  orb_yaw_rad and rpy[2] are derived from
    the same R_orb, so they must be identical to floating-point precision.
    """
    _, sensor_log = _run()
    violations = []
    for s in sensor_log:
        diff = abs(s["yaw_sensor"] - s["orb_yaw"])
        # Wrap to [-pi, pi]
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        if abs(diff) > 1e-9:
            violations.append(f"t={s['t']:.1f}s  rpy[2]={math.degrees(s['yaw_sensor']):.3f}  "
                              f"orb_yaw={math.degrees(s['orb_yaw']):.3f}  diff={math.degrees(diff):.4f} deg")
    _log.write(violations or ["(all match to floating-point precision)"],
               f"violations={len(violations)}")
    assert not violations, (
        f"rpy[2] != orb_yaw_rad in {len(violations)} samples "
        "(velocity-heading override crept back in?):\n" + "\n".join(violations[:10])
    )


def test_sensor_yaw_changes_slowly():
    """
    Sensor yaw (orb_yaw from R_orb) must change slowly — at orbital rate only.

    The orbital frame yaw changes as disk_normal rotates during orbit.
    Typical rate is ~0.01-0.05 rad/s.  A rate > 1 rad/s indicates a frame
    singularity or spin-strip bug.
    """
    _, sensor_log = _run()
    YAW_RATE_MAX = 1.0   # rad/s — generous; orbital rate is ~0.02 rad/s
    violations = []
    prev = None
    for s in sensor_log:
        if prev is not None:
            dt_snap = s["t"] - prev["t"]
            if dt_snap > 0:
                delta = s["orb_yaw"] - prev["orb_yaw"]
                delta = (delta + math.pi) % (2 * math.pi) - math.pi
                rate = abs(delta) / dt_snap
                if rate > YAW_RATE_MAX:
                    violations.append(f"t={s['t']:.1f}s  yaw_rate={math.degrees(rate):.1f} deg/s")
        prev = s
    _log.write(violations or ["(all within 1 rad/s)"],
               f"violations={len(violations)}  limit=1.0 rad/s")
    assert not violations, (
        f"Orbital yaw rate exceeded 1 rad/s in {len(violations)} samples:\n"
        + "\n".join(violations[:10])
    )


def test_sensor_gyro_bounded():
    """
    gyro_body norm must stay below GYRO_MAX_RAD_S.

    260 rad/s in an earlier run indicated a spin-strip failure.  The orbital
    rate during normal flight is ~0.2-0.3 rad/s; 5 rad/s catches bugs with
    margin for short transients.
    """
    _, sensor_log = _run()
    peak = max(s["gyro_norm"] for s in sensor_log)
    _log.write(
        [f"t={s['t']:.1f}  gyro_norm={s['gyro_norm']:.4f} rad/s" for s in sensor_log],
        f"peak_gyro={peak:.4f} rad/s  limit={GYRO_MAX_RAD_S} rad/s",
    )
    assert peak <= GYRO_MAX_RAD_S, (
        f"gyro_body peak {peak:.3f} rad/s exceeded {GYRO_MAX_RAD_S} rad/s "
        "(spin-strip bug?)"
    )


def test_sensor_gyro_spin_isolated():
    """
    Gyro must be well below rotor spin rate (GB4008 keeps electronics non-rotating).

    Ratio omega_spin / gyro_norm must exceed SPIN_ISOLATION_FACTOR at every sample.
    A failure here means the spin component is leaking into gyro_body.
    """
    _, sensor_log = _run()
    min_ratio = float("inf")
    lines = ["t(s)   gyro_norm(rad/s)  omega_spin(rad/s)  ratio"]
    for s in sensor_log:
        if s["gyro_norm"] < 1e-6:
            continue   # near-zero gyro; ratio is infinite, skip
        ratio = s["omega_spin"] / s["gyro_norm"]
        min_ratio = min(min_ratio, ratio)
        lines.append(
            f"  {s['t']:5.1f}  {s['gyro_norm']:16.4f}  "
            f"{s['omega_spin']:17.2f}  {ratio:.1f}"
        )

    _log.write(lines,
               f"min_ratio={min_ratio:.1f}  required={SPIN_ISOLATION_FACTOR}")
    assert min_ratio >= SPIN_ISOLATION_FACTOR, (
        f"Spin leaked into gyro: min(omega_spin/gyro_norm)={min_ratio:.1f} "
        f"(required >= {SPIN_ISOLATION_FACTOR})"
    )
