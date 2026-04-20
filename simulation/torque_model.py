"""
torque_model.py — Counter-torque motor simulation model

Simulates the RAWES stationary inner assembly + spinning rotor hub + GB4008 motor.
Goal: verify that ArduPilot's SITL can regulate yaw using the tail-rotor control
channel while the motor counter-rotates to maintain hub heading.

Physical setup
--------------
  Rotor hub : outer spinning shell (blades + hub) in autorotation; omega_rotor [rad/s]
  Hub       : stationary inner assembly (~1 kg); yaw DOF psi [rad], psi_dot [rad/s]
  Axle      : stationary central shaft; tether attaches at bottom -- does NOT rotate
  Gear      : 80:44 (rotor hub : motor pinion), so omega_motor = omega_rotor x (80/44)
  Motor     : GB4008 66KV BLDC; stator fixed to inner assembly, rotor geared to
              spinning outer rotor hub

Hub yaw model
--------------
The ESC holds the motor shaft at a commanded speed proportional to throttle, but
the motor shaft speed does not respond instantaneously -- it tracks the commanded
speed with a first-order lag (time constant MOTOR_TAU):

    d(omega_motor)/dt = (throttle x RPM_SCALE - omega_motor) / MOTOR_TAU

The mechanical gear coupling is instantaneous.  Once omega_motor is known, the
inner assembly yaw rate follows directly from the kinematic constraint:

    psi_dot = omega_rotor - omega_motor / GEAR_RATIO

psi_dot is an algebraic function of omega_motor and omega_rotor -- no hub inertia
term appears.  The ESC absorbs all load (bearing drag, swashplate friction) by
drawing more current; these forces are invisible to yaw dynamics.

Equilibrium
-----------
At steady state omega_motor = throttle x RPM_SCALE, so setting psi_dot = 0:

    throttle_eq = omega_rotor x GEAR_RATIO / RPM_SCALE

At omega_rotor = 28 rad/s: throttle_eq = 28 x 1.818 / 105 ~= 0.485.

Yaw drift
---------
  throttle < throttle_eq  -->  psi_dot > 0  (CW drift -- inner assembly follows hub)
  throttle > throttle_eq  -->  psi_dot < 0  (CCW drift -- inner assembly counter-rotates)
"""
from __future__ import annotations

import dataclasses

# ---------------------------------------------------------------------------
# Physical constants
# ---------------------------------------------------------------------------

#: Motor shaft speed per unit throttle [rad/s] -- GB4008 66KV x 15.2V (4S LiPo) ~= 105 rad/s
#: The ESC targets motor shaft at throttle x RPM_SCALE; MOTOR_TAU governs how fast it gets there.
RPM_SCALE: float = 105.0   # rad/s

#: Motor-side gear teeth / hub-side gear teeth  -->  omega_motor / omega_hub
#: Inner assembly yaw rate: psi_dot = omega_hub - omega_motor / GEAR_RATIO
GEAR_RATIO: float = 80.0 / 44.0

#: Autorotation spin rate of the outer rotor hub at design point (10 m/s wind, de Schutter 2018)
OMEGA_ROTOR_NOMINAL: float = 28.0   # rad/s  ~= 267 RPM

#: First-order lag time constant for motor shaft speed response to PWM command [s]
#: Represents ESC + motor electrical/mechanical inertia.  ~20 ms is typical for
#: small BLDC + ESC combinations at full-load step.
MOTOR_TAU: float = 0.02   # s


# ---------------------------------------------------------------------------
# Parameter + state containers
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class HubParams:
    """Physical parameters for the hub yaw model."""
    rpm_scale:  float = RPM_SCALE   # rad/s, motor shaft speed at throttle=1
    gear_ratio: float = GEAR_RATIO  # omega_motor / omega_hub  (80/44)
    motor_tau:  float = MOTOR_TAU   # s, first-order lag on motor shaft speed


@dataclasses.dataclass
class HubState:
    """Instantaneous state of the hub yaw model (NED: positive = CW from above)."""
    psi:         float = 0.0   # NED yaw angle [rad]
    psi_dot:     float = 0.0   # NED yaw rate [rad/s]
    omega_motor: float = 0.0   # motor shaft speed [rad/s]


# ---------------------------------------------------------------------------
# Kinematics
# ---------------------------------------------------------------------------

def step(
    state: HubState,
    omega_rotor: float,
    throttle: float,
    params: HubParams,
    dt: float,
) -> HubState:
    """
    Advance hub yaw state by dt.

    Motor shaft speed tracks the PWM command with a first-order lag:

        d(omega_motor)/dt = (throttle x RPM_SCALE - omega_motor) / MOTOR_TAU

    Gear coupling is instantaneous:

        psi_dot = omega_rotor - omega_motor / GEAR_RATIO

    Parameters
    ----------
    state       : current hub state (psi, psi_dot, omega_motor)
    omega_rotor : rotor hub angular speed [rad/s]
    throttle    : motor command [0, 1] (clamped internally)
    params      : HubParams
    dt          : time step [s]

    Returns
    -------
    New HubState at t + dt.
    """
    throttle        = max(0.0, min(1.0, throttle))
    omega_commanded = throttle * params.rpm_scale
    d_omega         = (omega_commanded - state.omega_motor) / params.motor_tau
    omega_motor_new = state.omega_motor + d_omega * dt
    psi_dot         = omega_rotor - omega_motor_new / params.gear_ratio
    psi             = state.psi + psi_dot * dt
    return HubState(psi=psi, psi_dot=psi_dot, omega_motor=omega_motor_new)


# ---------------------------------------------------------------------------
# Feedforward helper
# ---------------------------------------------------------------------------

def equilibrium_throttle(omega_rotor: float, params: HubParams) -> float:
    """
    Compute the motor throttle that holds psi_dot = 0 at a given rotor hub speed.

    At steady state omega_motor = throttle x RPM_SCALE, so:

        throttle_eq = omega_rotor x GEAR_RATIO / RPM_SCALE

    Returns throttle in [0, 1].
    """
    return min(1.0, max(0.0, omega_rotor * params.gear_ratio / params.rpm_scale))
