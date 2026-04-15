"""
torque/model.py — Counter-torque motor simulation model

Simulates the RAWES stationary inner assembly + spinning rotor hub + GB4008 motor.
Goal: verify that ArduPilot's SITL can regulate yaw using the tail-rotor control
channel while the motor counter-rotates to maintain hub heading.

Physical setup
--------------
  Rotor hub : outer spinning shell (blades + hub) in autorotation; ω_rotor [rad/s]
  Hub       : stationary inner assembly (~1 kg); yaw DOF ψ [rad], ψ_dot [rad/s]
  Axle      : stationary central shaft; tether attaches at bottom — does NOT rotate
  Gear      : 80:44 (rotor hub : motor pinion), so ω_motor = ω_rotor × (80/44)
  Motor     : GB4008 66KV BLDC; stator fixed to inner assembly, rotor geared to
              spinning outer rotor hub

Gear direction
--------------
The motor stator is fixed to the stationary inner assembly.  The motor rotor is
geared to the spinning outer rotor hub (80 teeth on hub, 44 on motor pinion —
motor runs faster than the rotor hub).  When the motor produces electromagnetic
torque τ_em on its rotor, the equal and opposite reaction acts on the stator
(inner assembly).  The gear multiplies this reaction:

    Q_motor_on_hub = −(80/44) × τ_em_at_motor_shaft

A positive throttle → positive τ_em → negative Q_motor_on_hub (counter-rotating
against rotor hub spin to maintain inner assembly heading).

Hub equation of motion
----------------------
    I_hub × ψ̈ = Q_bearing + Q_motor_on_hub

    Q_bearing      = k_bearing × (ω_rotor − ψ_dot)      [viscous; drags inner assembly toward rotor hub]
    Q_motor_on_hub = −GEAR_RATIO × τ_motor(throttle, ω_motor)
    ω_motor        = ω_rotor × GEAR_RATIO

Motor model (DC motor equation)
--------------------------------
    τ = τ_stall × max(0, throttle − ω_motor_rel / ω_no_load)

    τ_stall  = V_bat / (KV_rad × R)   [stall torque at full throttle, ω=0]
    ω_no_load = KV_rad × V_bat         [no-load speed at full throttle]

The key identity: throttle scales the applied voltage (V_applied = throttle×V_bat),
so the no-load operating point shifts to throttle×ω₀, not ω₀.  Using
(throttle − ω/ω₀) rather than throttle×(1 − ω/ω₀) is the physically correct
form of the DC motor torque equation.

Motor is unidirectional (ESC cannot reverse), so τ ≥ 0.

Nominal operating point
-----------------------
  ω_rotor = 28 rad/s  (≈ 267 RPM, de Schutter 2018 at 10 m/s wind)
  At this speed ω_motor ≈ 51 rad/s; motor is well below no-load speed → good
  torque authority.  equilibrium_throttle() gives the exact feedforward value.
"""
from __future__ import annotations

import dataclasses
import math

# ---------------------------------------------------------------------------
# Physical constants for GB4008 66KV on 4S LiPo
# ---------------------------------------------------------------------------

#: Motor-side gear teeth / axle-side gear teeth  →  ω_motor / ω_axle
GEAR_RATIO: float = 80.0 / 44.0

#: GB4008 KV in SI units [rad/s per volt]
MOTOR_KV_RAD: float = 66.0 * (2.0 * math.pi / 60.0)   # ≈ 6.91 rad/s/V

#: Nominal battery voltage (4S LiPo) [V]
BATTERY_V: float = 15.2

#: Motor no-load speed at full throttle [rad/s]  (KV × V_bat)
MOTOR_OMEGA_NO_LOAD: float = MOTOR_KV_RAD * BATTERY_V  # ≈ 105 rad/s

#: Winding resistance from hardware.md [Ω]
MOTOR_RESISTANCE: float = 7.5

#: Stall torque at full throttle (ω=0): τ_stall = V_bat / (KV_rad × R)
#: Derivation: K_t = 1/KV_rad [N·m/A],  I_stall = V_bat/R [A]
#:             τ_stall = K_t × I_stall = V_bat / (KV_rad × R)
MOTOR_TAU_STALL: float = BATTERY_V / (MOTOR_KV_RAD * MOTOR_RESISTANCE)  # ≈ 0.293 N·m

#: Autorotation spin rate of the outer rotor hub at design point (10 m/s wind, de Schutter 2018)
OMEGA_ROTOR_NOMINAL: float = 28.0   # rad/s  ≈ 267 RPM


# ---------------------------------------------------------------------------
# Parameter + state containers
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class HubParams:
    """
    Tunable physical parameters for the hub yaw model.

    All values come with physically motivated defaults; adjust as needed.
    """

    # Hub moment of inertia about yaw axis [kg·m²]
    # Stationary assembly ≈ 1 kg; rough geometry gives I ≈ 0.007 kg·m²
    # (treat as a solid cylinder: I = 0.5 × m × r_eff²  with r_eff ≈ 0.12 m)
    I_hub: float = 0.007

    # Bearing viscous drag coefficient [N·m·s/rad]
    # At ω_rotor = 28 rad/s bearing drag ≈ 0.14 N·m  →  k = 0.005 N·m·s/rad
    # This is deliberately tunable — real bearing friction depends on lubrication,
    # preload, and temperature.
    k_bearing: float = 0.005

    # Motor parameters (at motor shaft) — derived from GB4008 66KV on 4S
    tau_stall: float = MOTOR_TAU_STALL       # N·m, stall torque at full throttle
    omega_no_load: float = MOTOR_OMEGA_NO_LOAD  # rad/s, no-load speed at full throttle
    gear_ratio: float = GEAR_RATIO           # ω_motor / ω_axle  (80/44)


@dataclasses.dataclass
class HubState:
    """Instantaneous yaw state of the stationary hub (NED: positive = CW from above)."""
    psi: float = 0.0       # NED yaw angle [rad]
    psi_dot: float = 0.0   # NED yaw rate [rad/s]


# ---------------------------------------------------------------------------
# Motor model
# ---------------------------------------------------------------------------

def motor_torque(throttle: float, omega_motor_rel: float, params: HubParams) -> float:
    """
    Motor torque at the motor shaft [N·m].

    DC motor equation:
        τ = τ_stall × max(0, throttle − ω_rel / ω_no_load)

    where ω_rel is the motor rotor speed *relative to the stator* (which is
    mounted on the hub).  throttle scales the applied voltage; the no-load point
    shifts to throttle×ω₀, so the correct form is (throttle − ω/ω₀), NOT
    throttle×(1 − ω/ω₀).

    The motor is unidirectional (ESC cannot reverse), so τ is clamped ≥ 0.

    Parameters
    ----------
    throttle        : motor command, nominally [0, 1] (clamped internally)
    omega_motor_rel : motor shaft speed relative to hub [rad/s]
                      = (ω_axle − ψ_dot) × gear_ratio
    params          : HubParams

    Returns
    -------
    Torque at motor shaft [N·m], always ≥ 0.
    """
    throttle = max(0.0, min(1.0, throttle))
    return max(0.0, params.tau_stall * (throttle - omega_motor_rel / params.omega_no_load))


# ---------------------------------------------------------------------------
# Dynamics
# ---------------------------------------------------------------------------

def _derivatives(
    state: HubState,
    omega_rotor: float,
    throttle: float,
    params: HubParams,
) -> tuple[float, float]:
    """
    Compute d/dt [ψ, ψ_dot] for the hub yaw degree of freedom.

    Returns
    -------
    (psi_ddot, psi_dot)  — i.e. the time derivatives of [ψ_dot, ψ]
    """
    # Bearing drag: viscous coupling between spinning rotor hub and stationary inner assembly
    Q_bearing = params.k_bearing * (omega_rotor - state.psi_dot)

    # Motor reaction on inner assembly (counter-rotates to maintain heading).
    # Motor back-EMF depends on relative speed of motor rotor vs stator (stator on inner assembly).
    omega_motor_rel = max(0.0, (omega_rotor - state.psi_dot) * params.gear_ratio)
    tau_shaft   = motor_torque(throttle, omega_motor_rel, params)
    Q_motor     = -params.gear_ratio * tau_shaft  # Newton's 3rd law through gear

    psi_ddot = (Q_bearing + Q_motor) / params.I_hub
    return psi_ddot, state.psi_dot


def step(
    state: HubState,
    omega_rotor: float,
    throttle: float,
    params: HubParams,
    dt: float,
) -> HubState:
    """
    RK4 integration step for hub yaw dynamics.

    Parameters
    ----------
    state       : current hub yaw state
    omega_rotor : rotor hub angular speed [rad/s] (assumed constant over dt)
    throttle    : motor throttle [0, 1]
    params      : HubParams
    dt          : time step [s]

    Returns
    -------
    New HubState at t + dt.
    """
    def deriv(s: HubState) -> tuple[float, float]:
        return _derivatives(s, omega_rotor, throttle, params)

    k1_pdot, k1_p = deriv(state)
    s2 = HubState(state.psi + 0.5 * dt * k1_p,  state.psi_dot + 0.5 * dt * k1_pdot)

    k2_pdot, k2_p = deriv(s2)
    s3 = HubState(state.psi + 0.5 * dt * k2_p,  state.psi_dot + 0.5 * dt * k2_pdot)

    k3_pdot, k3_p = deriv(s3)
    s4 = HubState(state.psi + dt * k3_p,         state.psi_dot + dt * k3_pdot)

    k4_pdot, k4_p = deriv(s4)

    new_psi_dot = state.psi_dot + (dt / 6.0) * (k1_pdot + 2*k2_pdot + 2*k3_pdot + k4_pdot)
    new_psi     = state.psi     + (dt / 6.0) * (k1_p    + 2*k2_p    + 2*k3_p    + k4_p)

    return HubState(psi=new_psi, psi_dot=new_psi_dot)


# ---------------------------------------------------------------------------
# Feedforward helper
# ---------------------------------------------------------------------------

def equilibrium_throttle(omega_rotor: float, params: HubParams) -> float:
    """
    Compute the motor throttle that holds ψ_dot = 0 at a given rotor hub speed.

    Derivation (steady state, ψ̈=0, ψ_dot=0):

        0 = k_bearing × ω_rotor − gear_ratio × τ_motor
        τ_motor_needed = k_bearing × ω_rotor / gear_ratio

    Inverted from the DC motor equation τ = τ_stall × (throttle − ω_rel/ω₀):

        throttle = τ_motor_needed / τ_stall + ω_motor_rel / ω_no_load

    where ω_motor_rel = ω_rotor × gear_ratio  (at ψ_dot=0, relative = absolute).

    Returns throttle ∈ [0, 1].  Returns 1.0 if motor is saturated.

    Parameters
    ----------
    omega_rotor : rotor hub angular speed [rad/s]
    params      : HubParams
    """
    tau_needed      = params.k_bearing * omega_rotor / params.gear_ratio
    omega_motor_rel = omega_rotor * params.gear_ratio
    throttle        = tau_needed / params.tau_stall + omega_motor_rel / params.omega_no_load
    return min(1.0, max(0.0, throttle))
