from . import _dynbem as _native

_omega_derivative_rust = _native.omega_derivative
_step_omega_rust = _native.step_omega

__all__ = ["omega_derivative", "step_omega"]

def omega_derivative(omega: float, Q_aero: float, motor_torque_Nm: float, I_ode_kgm2: float, bearing_friction_Nm: float = 0.0) -> float: ...
def step_omega(omega: float, spin_angle: float, Q_aero: float, motor_torque_Nm: float, I_ode_kgm2: float, dt: float, bearing_friction_Nm: float = 0.0) -> tuple[float, float]: ...
