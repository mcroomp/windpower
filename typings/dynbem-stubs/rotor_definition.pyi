"""Python rotor schema, including the validation methods attached at import."""

from collections.abc import Iterable as _Iterable, Sequence as _Sequence
from dataclasses import dataclass as _dataclass
from os import PathLike as _PathLike
from typing import Never as _Never, Self as _Self, overload as _overload
from . import _dynbem as _native

__all__ = [
    "LinearPolarParameters", "AutorotationProperties", "BladeGeometry",
    "ControlProperties", "FlapProperties", "ServoFlapActuation",
    "ServoFlapGeometry", "InertiaProperties", "KamanFlap", "RotorDefinition",
    "ValidationIssue", "load", "loads", "default",
]

class BladeGeometry(_native.BladeGeometry):
    def __new__(cls, n_blades: int, radius_m: float, root_cutout_m: float, chord_m: float, twist_deg: float = 0.0, n_elements: int = 20, r_stations_m: _Sequence[float] | None = None, chord_stations_m: _Sequence[float] | None = None, twist_stations_deg: _Sequence[float] | None = None, tip_loss: bool = True) -> _Self: ...
    def __init__(self, n_blades: int, radius_m: float, root_cutout_m: float, chord_m: float, twist_deg: float = 0.0, n_elements: int = 20, r_stations_m: _Sequence[float] | None = None, chord_stations_m: _Sequence[float] | None = None, twist_stations_deg: _Sequence[float] | None = None, tip_loss: bool = True) -> None: ...
    def validate(self) -> list[ValidationIssue]: ...

class KamanFlap:
    chord_fraction: float | None
    span_start_m: float | None
    span_end_m: float | None
    tau: float | None
    CM_gamma_per_rad: float | None
    swashplate_load_fraction: float | None
    notes: str
    def __init__(self, chord_fraction: float | None = None, span_start_m: float | None = None, span_end_m: float | None = None, tau: float | None = None, CM_gamma_per_rad: float | None = None, swashplate_load_fraction: float | None = None, notes: str = "") -> None: ...

class ServoFlapGeometry:
    C_M_delta_per_rad: float
    r_inner_m: float
    r_outer_m: float
    def __init__(self, C_M_delta_per_rad: float, r_inner_m: float, r_outer_m: float) -> None: ...
    def _to_rust(self) -> _native.ServoFlapGeometry: ...

class ServoFlapActuation:
    I_theta_kgm2: float
    damper_Nms_per_rad: float
    flap: ServoFlapGeometry
    ac_offset_m: float
    blade_Cm_AC: float
    def __init__(self, I_theta_kgm2: float, damper_Nms_per_rad: float, flap: ServoFlapGeometry, ac_offset_m: float = 0.0, blade_Cm_AC: float = 0.0) -> None: ...
    def _to_rust(self) -> _native.ServoFlapActuation: ...

class FlapProperties:
    I_blade_flap_kgm2: float
    omega_nr_rad_s: float
    def __init__(self, I_blade_flap_kgm2: float, omega_nr_rad_s: float = 0.0) -> None: ...
    def _to_rust(self) -> _native.FlapProperties: ...

class InertiaProperties:
    mass_kg: float | None
    I_body_kgm2: list[float]
    I_spin_kgm2: float | None
    blade_mass_kg: float | None
    stationary_assembly_mass_kg: float | None
    spinning_hub_shell_mass_kg: float | None
    I_blade_flap_kgm2: float | None
    def __init__(self, mass_kg: float | None = None, I_body_kgm2: _Iterable[float] | None = None, I_spin_kgm2: float | None = None, blade_mass_kg: float | None = None, stationary_assembly_mass_kg: float | None = None, spinning_hub_shell_mass_kg: float | None = None, I_blade_flap_kgm2: float | None = None) -> None: ...

class LinearPolarParameters:
    CL0: float
    CL_alpha_per_rad: float
    CD0: float
    alpha_stall_deg: float
    Re_design: float | None
    name: str
    source: str
    polar_csv: str | _PathLike[str] | None
    CD_structural: float
    Re_operating: float | None
    def __init__(self, CL0: float, CL_alpha_per_rad: float, CD0: float, alpha_stall_deg: float, Re_design: float | None = None, name: str = "", source: str = "", polar_csv: str | _PathLike[str] | None = None, CD_structural: float = 0.0, Re_operating: float | None = None) -> None: ...
    def validate(self) -> list[ValidationIssue]: ...

class ControlProperties:
    swashplate_pitch_gain_rad: float
    axle_attachment_length_m: float | None
    K_cyc: float | None
    swashplate_phase_deg: float | None
    servo_slew_rate_deg_s: float | None
    servo_travel_deg: float | None
    kaman_flap: KamanFlap | None
    def __init__(self, swashplate_pitch_gain_rad: float, axle_attachment_length_m: float | None = None, K_cyc: float | None = None, swashplate_phase_deg: float | None = None, servo_slew_rate_deg_s: float | None = None, servo_travel_deg: float | None = None, kaman_flap: KamanFlap | None = None) -> None: ...

class AutorotationProperties:
    I_ode_kgm2: float | None
    omega_min_rad_s: float | None
    omega_eq_rad_s: float | None
    def __init__(self, I_ode_kgm2: float | None = None, omega_min_rad_s: float | None = None, omega_eq_rad_s: float | None = None) -> None: ...

class RotorDefinition:
    blade: BladeGeometry
    airfoil: LinearPolarParameters
    control: ControlProperties | None
    inertia: InertiaProperties
    autorotation: AutorotationProperties
    servoflap: ServoFlapActuation | None
    flap: FlapProperties | None
    name: str
    description: str
    def __init__(self, blade: BladeGeometry, airfoil: LinearPolarParameters, control: ControlProperties | None = None, inertia: InertiaProperties | None = None, autorotation: AutorotationProperties | None = None, servoflap: ServoFlapActuation | None = None, flap: FlapProperties | None = None, name: str = "", description: str = "") -> None: ...
    @property
    def span_m(self) -> float: ...
    @property
    def r_cp_m(self) -> float: ...
    @property
    def disk_area_m2(self) -> float: ...
    @property
    def solidity(self) -> float: ...
    def validate(self) -> list[ValidationIssue]: ...

@_dataclass
class ValidationIssue:
    level: str
    field: str
    message: str
    def __str__(self) -> str: ...

@_overload
def _to_rust_control(control: None) -> None: ...
@_overload
def _to_rust_control(control: ControlProperties | _native.ControlProperties) -> _native.ControlProperties: ...
def _to_rust_defn(defn: RotorDefinition | _native.RotorDefinition) -> _native.RotorDefinition: ...
def load(path: str | _PathLike[str]) -> RotorDefinition: ...
def loads(text: str, base_dir: str | None = None) -> RotorDefinition: ...
def default() -> _Never: ...
