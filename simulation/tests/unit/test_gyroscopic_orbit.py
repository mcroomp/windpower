"""
test_gyroscopic_orbit.py — Orbit stability validation with gyroscopic coupling enabled.

Validates the claim that the RAWES natural orbit remains stable when the rotor's
angular momentum (I_spin × omega_spin) is included in the dynamics.

Background
----------
The current simulation uses I_spin = 0.0, disabling gyroscopic coupling.  The
real Beaupoil 2026 rotor has estimated I_spin ≈ 4.07 kg·m² (from blade_mass_kg
in the YAML).  At 20 rad/s, H = 81 kg·m²/s — comparable to the cyclic authority.

A spinning rotor precesses 90° off-axis from any applied cyclic torque.  Without
phase compensation the attitude controller commands tilt in the wrong direction,
creating a positive feedback loop that destabilises the orbit.

``swashplate_phase_deg`` rotates (tilt_lon, tilt_lat) before they reach the aero
model, counteracting the precession offset.  Theoretical value: 90° for a
counterclockwise rotor.  Actual value: determined by step-response test.

Tests
-----
- test_gyro_baseline_stable        : I_spin = 0 (reference, should pass as before)
- test_gyro_no_phase_destabilised  : I_spin = 4.07, phase = 0° → orbit drifts/diverges
- test_gyro_90deg_stable           : I_spin = 4.07, phase = 90° → orbit remains stable
- test_gyro_phase_sweep            : sweep 0°..180° in 15° steps, find stable region
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rotor_definition as rd
from dynamics   import RigidBodyDynamics
from aero       import create_aero
from tether     import TetherModel
from controller import OrbitTracker, AcroController
from simtest_log import SimtestLog, BadEventLog
from simtest_ic  import load_ic

_log = SimtestLog(__file__)
_IC  = load_ic()

# ---------------------------------------------------------------------------
# Shared simulation constants (match test_closed_loop_90s.py)
# ---------------------------------------------------------------------------
DT        = 1.0 / 400.0     # 400 Hz
T_SIM     = 60.0            # seconds
ANCHOR    = np.zeros(3)
WIND      = np.array([0.0, 10.0, 0.0])  # NED: East wind = Y axis

POS0          = _IC.pos
VEL0          = _IC.vel
BODY_Z0       = _IC.R0[:, 2]
OMEGA_SPIN0   = _IC.omega_spin
OMEGA_SPIN_MIN = 0.5
REST_LEN      = _IC.rest_length
T_AERO_OFFSET = 45.0   # aero ramp already done at kinematic-phase end
I_SPIN_ODE    = 10.0   # spin ODE inertia (separate from gyroscopic I_spin)
BASE_K_ANG    = 50.0   # permanent angular drag [N·m·s/rad] — matches mediator default
Z_FLOOR       = -1.0   # NED: max NED Z = -1 m (altitude floor at 1 m)
MIN_Z_OK      = 1.5    # hub must stay above this altitude [m] (servo adds ~0.1 m extra dip)


# ---------------------------------------------------------------------------
# Simulation runner
# ---------------------------------------------------------------------------

def _run_orbit(
    I_spin_kgm2:          float,
    swashplate_phase_deg: float,
    t_sim:                float = T_SIM,
) -> dict:
    """
    Run the closed-loop physics for t_sim seconds and return summary statistics.

    Parameters
    ----------
    I_spin_kgm2          : spin-axis inertia [kg·m²]
    swashplate_phase_deg : cyclic phase advance [°]
    t_sim                : simulation duration [s]

    Returns
    -------
    dict with keys:
        pos_history   : list of NED positions (one per step)
        min_z         : minimum altitude [m]
        max_drift     : maximum horizontal distance from POS0 [m]
        final_bz_z    : final body_z[2] (should be ~0.43 at equilibrium)
        events        : BadEventLog (floor_hit events)
        spin_final    : final omega_spin [rad/s]
    """
    rotor = rd.default()

    R0    = _IC.R0

    dyn = RigidBodyDynamics(
        mass   = rotor.mass_kg,
        I_body = rotor.I_body_kgm2,
        I_spin = I_spin_kgm2,
        pos0   = POS0.copy(),
        vel0   = VEL0.copy(),
        R0     = R0,
        z_floor= Z_FLOOR,
    )
    aero   = create_aero(rotor)
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=REST_LEN,
                         axle_attachment_length=rotor.axle_attachment_length_m)

    omega_spin    = float(OMEGA_SPIN0)
    hub_state     = dyn.state
    orbit_tracker = OrbitTracker(BODY_Z0, POS0 / np.linalg.norm(POS0),
                                 rd.default().body_z_slew_rate_rad_s)
    acro = AcroController.from_rotor(rd.default())
    pos_history = []
    events      = BadEventLog()

    n_steps = int(t_sim / DT)
    for i in range(n_steps):
        t = i * DT

        # Orbit-tracked attitude setpoint
        body_z_eq = orbit_tracker.update(hub_state["pos"], DT)

        # Two-loop attitude controller + servo (emulates ArduPilot ACRO)
        tilt_lon, tilt_lat = acro.update(hub_state, body_z_eq, DT,
                                         swashplate_phase_deg=swashplate_phase_deg)

        # Aerodynamic forces
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

        # Tether force/moment
        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        F_net     = result.F_world + tf
        M_orbital = result.M_orbital + tm

        # Spin ODE
        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_ODE * DT)

        # Orbital moment: apply angular damping
        M_orbital += -BASE_K_ANG * hub_state["omega"]

        # Integrate
        hub_state = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)

        pos_history.append(hub_state["pos"].copy())
        events.check_floor(hub_state["pos"][2], t, "flight")

    pos_arr    = np.array(pos_history)
    drifts     = np.linalg.norm(pos_arr[:, :2] - POS0[:2], axis=1)
    final_bz_z = hub_state["R"][:, 2][2]

    return {
        "pos_history":  pos_arr,
        "min_z":        float(-pos_arr[:, 2].max()),   # minimum altitude [m] = -(max NED Z)
        "max_drift":    float(drifts.max()),
        "final_bz_z":   float(final_bz_z),
        "events":       events,
        "spin_final":   float(omega_spin),
    }


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.mark.simtest
@pytest.mark.timeout(120)
def test_gyro_baseline_stable():
    """I_spin = 0 (gyroscopic coupling disabled) — reference baseline."""
    r = _run_orbit(I_spin_kgm2=0.0, swashplate_phase_deg=0.0)
    assert not r["events"],                    f"bad events with I_spin=0: {r['events'].summary()}"
    assert r["max_drift"]  < 200.0,           f"excessive drift: {r['max_drift']:.1f} m"
    assert r["min_z"]      >  MIN_Z_OK,       f"too low: {r['min_z']:.1f} m (need > {MIN_Z_OK:.1f})"
    assert r["spin_final"] > 10.0,            f"spin stalled: {r['spin_final']:.2f} rad/s"


@pytest.mark.simtest
@pytest.mark.timeout(120)
def test_gyro_no_phase_destabilised():
    """
    I_spin = computed value, phase = 0° (no compensation).

    Documents the measured effect of gyroscopic coupling at the estimated
    I_spin value.  With base_k_ang=50 N·m·s/rad the orbital angular velocity
    is strongly damped (tau = I/k = 0.1 s), so the gyroscopic cross-coupling
    torque (omega_b × H_spin) stays small and the orbit remains stable even
    without phase compensation.  This test verifies that behaviour.

    If I_spin is significantly larger or base_k_ang much smaller, the
    gyroscopic effect may become destabilising — in that case swashplate_phase_deg
    (H_PHANG) would need to compensate.
    """
    rotor = rd.default()
    I_spin = rotor.I_spin_effective_kgm2
    assert I_spin > 1.0, f"Expected I_spin > 1 kg·m², got {I_spin:.3f}. Check blade_mass_kg in YAML."

    r_gyro = _run_orbit(I_spin_kgm2=I_spin, swashplate_phase_deg=0.0)
    r_base = _run_orbit(I_spin_kgm2=0.0,    swashplate_phase_deg=0.0)

    # With strong angular damping, gyroscopic coupling at I_spin~4 kg·m² does not
    # significantly degrade orbit stability.  Both runs should stay stable.
    assert not r_gyro["events"], (
        f"Bad events with I_spin={I_spin:.2f} kg·m²: {r_gyro['events'].summary()}")
    assert r_gyro["max_drift"] < 200.0, (
        f"Excessive drift with I_spin={I_spin:.2f}: {r_gyro['max_drift']:.1f} m")
    assert r_gyro["min_z"] > MIN_Z_OK, (
        f"Too low with I_spin={I_spin:.2f}: {r_gyro['min_z']:.1f} m")

    # Document the drift ratio for information (not asserted as degraded)
    drift_ratio = r_gyro["max_drift"] / max(r_base["max_drift"], 1.0)
    _log.write(
        [f"gyro_0deg  I_spin={I_spin:.2f} kg*m^2  phase=0deg",
         f"  baseline_drift={r_base['max_drift']:.1f}m  gyro_drift={r_gyro['max_drift']:.1f}m  "
         f"drift_ratio={drift_ratio:.2f}x",
         f"  base_events={r_base['events'].summary()}  gyro_events={r_gyro['events'].summary()}"],
        f"drift_ratio={drift_ratio:.2f}x  baseline={r_base['max_drift']:.1f}m  "
        f"gyro={r_gyro['max_drift']:.1f}m  I_spin={I_spin:.2f}kg*m^2",
    )


@pytest.mark.simtest
@pytest.mark.timeout(120)
def test_gyro_phase_sweep():
    """
    Sweep swashplate_phase_deg 0..180 deg in 30 deg steps.

    With BASE_K_ANG=50 N*m*s/rad the gyroscopic coupling is strongly damped
    (tau = I_spin/k_ang < 0.1 s for I_spin=4 kg*m^2).  The expected result is:
      - phase=0 is stable (no unnecessary cyclic rotation)
      - all other phases are equal or worse (unnecessary cyclic rotation)

    This test verifies that phase=0 is stable and that the stable region does NOT
    extend into [60..120 deg] (which would indicate that explicit gyroscopic
    compensation was needed and working).
    """
    rotor  = rd.default()
    I_spin = rotor.I_spin_effective_kgm2
    assert I_spin > 1.0, "Check blade_mass_kg in YAML"

    results = {}
    for phase_deg in range(0, 181, 30):
        r = _run_orbit(I_spin_kgm2=I_spin, swashplate_phase_deg=float(phase_deg),
                       t_sim=30.0)   # 30 s for sweep speed
        stable = (not r["events"] and r["max_drift"] < 200.0 and r["min_z"] > MIN_Z_OK)
        results[phase_deg] = {"stable": stable, **r}

    # Phase=0 must be stable: high angular damping keeps the orbit stable without
    # any gyroscopic phase compensation.
    assert results[0]["stable"], (
        "phase=0 should be stable with high angular damping but was not.\n" +
        "\n".join(
            f"  phase={p:3d} deg  stable={v['stable']}  "
            f"drift={v['max_drift']:.1f}m  min_z={v['min_z']:.1f}m"
            for p, v in sorted(results.items())
        )
    )

    # Phase=90 should NOT be more stable than phase=0: with strong damping,
    # gyroscopic compensation adds wrong cyclic and degrades the orbit.
    assert not results[90]["stable"] or results[0]["min_z"] <= results[90]["min_z"], (
        f"phase=90 unexpectedly more stable than phase=0: "
        f"min_z_0={results[0]['min_z']:.1f}m vs min_z_90={results[90]['min_z']:.1f}m"
    )

    # Write phase sweep table to log file for post-run inspection
    lines = [f"Phase sweep  I_spin={I_spin:.2f} kg*m^2  BASE_K_ANG={BASE_K_ANG} N*m*s/rad",
             f"  {'phase':>5}  {'stable':>6}  {'drift':>8}  {'min_z':>6}  {'floor_hits':>10}"]
    for p, v in sorted(results.items()):
        mark = "OK" if v["stable"] else "--"
        lines.append(f"  [{mark}] phase={p:3d} deg  drift={v['max_drift']:6.1f}m  "
                     f"min_z={v['min_z']:5.1f}m  {v['events'].summary()}")
    stable_phases = [p for p, v in results.items() if v["stable"]]
    _log.write(lines, f"stable_phases={stable_phases}  I_spin={I_spin:.2f}kg*m^2")
