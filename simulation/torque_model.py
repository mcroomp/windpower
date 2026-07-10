"""
torque_model.py — Counter-torque motor simulation model

Simulates the RAWES stationary inner assembly + spinning rotor hub + GB4008 motor.
Goal: verify that ArduPilot's SITL can regulate yaw using the tail-rotor control
channel while the motor counter-rotates to maintain hub heading.

Rotation convention (US helicopter, baked into the whole stack)
---------------------------------------------------------------
Main rotor spins CCW viewed from above.  In NED with body-z DOWN, right-hand
rule gives CCW-from-above = angular-velocity vector along -body_z (UP), i.e.
NEGATIVE gyro:z().  Under aerodynamic drag the body experiences a CCW
reaction torque, so left to itself the inner assembly drifts CCW (gyro:z() < 0).
The GB4008 motor counters by applying CW torque to the body — controlled by
ATC_RAT_YAW PID with setpoint = 0.

Physical setup
--------------
  Rotor hub : outer spinning shell (blades + hub) in autorotation; omega_rotor [rad/s]
              is the SIGNED scalar spin rate.  omega_rotor > 0 = CCW from above
              (US convention).
  Hub       : stationary inner assembly (~1 kg); yaw DOF psi [rad], psi_dot [rad/s]
  Axle      : stationary central shaft; tether attaches at bottom -- does NOT rotate
  Gear      : 10:1 (omega_motor / omega_hub), so omega_motor = |omega_rotor| x 10
  Motor     : GB4008 66KV BLDC; stator fixed to inner assembly, rotor geared to
              spinning outer rotor hub.  Motor throttle in [0, 1]; positive
              throttle produces CW counter-torque on the body.

Hub yaw model (motor drives the hub inertia through the gear)
------------------------------------------------------------
The motor spins the inner hub assembly (a "wheel" with yaw inertia I_hub) via the
gear.  The ESC is a speed governor: it commands motor torque to drive the motor
shaft toward a target speed proportional to throttle, but the motor has FINITE
peak torque, so the speed cannot change instantaneously -- the motor must
accelerate the (gear-reflected) inertia:

    omega_target = throttle x RPM_SCALE
    Q            = clamp(ESC_KP x (omega_target - omega_motor), -Q_MAX, +Q_MAX)   [N.m]
    d(omega_motor)/dt = Q / J_total
    J_total      = I_hub / GEAR_RATIO^2 + I_motor      (hub inertia reflected to the
                                                        faster motor shaft, + rotor)

The ESC only sees motor RPM; the gear ratio enters solely through the reflected
inertia J_total.  The mechanical gear coupling itself is rigid, so the hub yaw
rate follows the motor speed kinematically (US convention, rotor CCW):

    psi_dot = -omega_rotor + omega_motor / GEAR_RATIO

Unlike the older algebraic (zero-inertia, ideal-speed-source) model, psi_dot is
now a proper dynamic state: throttle ripple is low-passed by the inertia and the
finite-torque slew limit (|d psi_dot/dt| <= Q_MAX / (J_total x GEAR_RATIO)), so
the plant no longer produces instantaneous tens-of-rad/s jumps.  Gear/motor
friction is small and neglected (at balance the governor commands ~0 torque).

Equilibrium
-----------
At steady state the governor holds omega_motor = throttle x RPM_SCALE, so
setting psi_dot = 0 gives the same feedforward as before:

    throttle_eq = omega_rotor x GEAR_RATIO / RPM_SCALE

At omega_rotor = 28 rad/s: throttle_eq = 28 x GEAR_RATIO / RPM_SCALE (update when RPM_SCALE changes).

Yaw drift
---------
  throttle < throttle_eq  -->  psi_dot < 0  (CCW drift -- body lags the motor's
                                              counter-CW push, rotor wins)
  throttle > throttle_eq  -->  psi_dot > 0  (CW drift -- motor pushes harder
                                              than needed)
  throttle = throttle_eq  -->  psi_dot = 0  (steady, body held against drag)
"""
from __future__ import annotations

import dataclasses

# ---------------------------------------------------------------------------
# Physical constants
# ---------------------------------------------------------------------------

#: Motor shaft speed per unit throttle [rad/s].
#: With 10:1 gear: omega_motor_eq = omega_rotor * GEAR_RATIO = 28 * 10 = 280 rad/s at nominal.
#: RPM_SCALE must satisfy throttle_eq = omega_rotor * GEAR_RATIO / RPM_SCALE in [0, 1].
#: Verify against actual motor KV + supply voltage for the new hardware.
RPM_SCALE: float = 578.0   # rad/s  (= OMEGA_ROTOR_NOMINAL * GEAR_RATIO / 0.485)

#: Motor-side gear teeth / hub-side gear teeth  -->  omega_motor / omega_hub
#: Inner assembly yaw rate: psi_dot = omega_hub - omega_motor / GEAR_RATIO
GEAR_RATIO: float = 10.0

#: Autorotation spin rate of the outer rotor hub at design point (10 m/s wind, de Schutter 2018)
OMEGA_ROTOR_NOMINAL: float = 28.0   # rad/s  ~= 267 RPM

#: Yaw moment of inertia of the inner hub assembly (electronics + structure,
#: EXCLUDING the rotor) about the spin axis [kg.m^2].  Reflected to the motor
#: shaft through the gear (J = I_hub / GEAR_RATIO^2), it sets how fast the motor
#: can change the hub yaw rate for a given torque.
HUB_INERTIA_KGM2: float = 0.02

#: GB4008 rotor moment of inertia at the motor shaft [kg.m^2] (small).
MOTOR_INERTIA_KGM2: float = 1.0e-5

#: ESC speed-governor gain -- commanded motor torque per rad/s of speed error
#: [N.m/(rad/s)].  Chosen so that tau = J_total / ESC_KP ≈ 40 ms.
#: With GEAR_RATIO=10: J_total = 0.02/100 + 1e-5 = 2.1e-4 kg.m² → ESC_KP = 2.1e-4 / 0.040 ≈ 5.2e-3.
#: (With the old 80:44 gear J_total ≈ 6.06e-3, ESC_KP was 0.15 — same 40ms target.)
#: Discrete stability requires ESC_KP × dt / J_total < 1  (dt=10ms gives ≈ 0.25, safe).
ESC_KP: float = 5.2e-3

#: GB4008 peak torque at the motor shaft [N.m].  Finite -> the motor cannot change
#: speed instantaneously; large throttle steps are slew-limited to Q_MAX / J_total.
ESC_Q_MAX: float = 2.0


# ---------------------------------------------------------------------------
# Parameter + state containers
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class HubParams:
    """Physical parameters for the hub yaw model."""
    rpm_scale:          float = RPM_SCALE           # rad/s, motor speed at throttle=1
    gear_ratio:         float = GEAR_RATIO          # omega_motor / omega_hub (80/44)
    hub_inertia_kgm2:   float = HUB_INERTIA_KGM2    # inner-hub yaw inertia (no rotor)
    motor_inertia_kgm2: float = MOTOR_INERTIA_KGM2  # motor rotor inertia at shaft
    esc_kp:             float = ESC_KP              # governor torque per rad/s error
    esc_q_max:          float = ESC_Q_MAX           # motor peak torque [N.m]

    def j_total(self) -> float:
        """Effective inertia at the motor shaft [kg.m^2]:  hub inertia reflected
        through the gear (I_hub / gear^2) plus the motor rotor inertia."""
        return self.hub_inertia_kgm2 / (self.gear_ratio ** 2) + self.motor_inertia_kgm2


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

    The ESC governor commands motor torque proportional to the motor-speed error,
    limited by the motor's finite peak torque; that torque accelerates the
    gear-reflected hub inertia:

        omega_target = throttle x RPM_SCALE
        Q            = clamp(esc_kp x (omega_target - omega_motor), -q_max, q_max)
        d(omega_motor)/dt = Q / J_total          J_total = I_hub/gear^2 + I_motor

    Rigid gear coupling then sets the hub yaw rate (US convention, rotor CCW):

        psi_dot = -omega_rotor + omega_motor / GEAR_RATIO

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
    throttle     = max(0.0, min(1.0, throttle))
    omega_target = throttle * params.rpm_scale

    # ESC speed governor -> commanded motor torque, capped by finite peak torque.
    err   = omega_target - state.omega_motor
    q_cmd = params.esc_kp * err
    q     = max(-params.esc_q_max, min(params.esc_q_max, q_cmd))

    # Finite torque accelerates the gear-reflected inertia: speed cannot jump.
    j_total         = params.j_total()
    omega_motor_new = state.omega_motor + (q / j_total) * dt
    if omega_motor_new < 0.0:
        omega_motor_new = 0.0   # motor speed magnitude cannot go negative

    # US convention: rotor CCW -> body drifts CCW (psi_dot < 0).  Motor
    # counter-rotation (omega_motor > 0) pushes psi_dot back toward zero.
    psi_dot = -omega_rotor + omega_motor_new / params.gear_ratio
    psi     = state.psi + psi_dot * dt
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
