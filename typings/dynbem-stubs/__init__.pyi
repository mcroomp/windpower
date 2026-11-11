from typing import TypedDict as _TypedDict, Unpack as _Unpack, overload as _overload
from . import _dynbem as _native
from ._dynbem import (
    AeroResult as AeroResult,
    LinearPolar as LinearPolar,
    TabulatedPolar as TabulatedPolar,
    QuasiStaticRotorState as QuasiStaticRotorState,
    PittPetersRotorState as PittPetersRotorState,
    OyeRotorState as OyeRotorState,
    VpmRotorState as VpmRotorState,
    RotorInputs as RotorInputs,
    TrimResult as TrimResult,
    _QuasiStaticBEMLinear as _QuasiStaticBEMLinear,
    _QuasiStaticBEMTabulated as _QuasiStaticBEMTabulated,
    _PittPetersModelLinear as _PittPetersModelLinear,
    _PittPetersModelTabulated as _PittPetersModelTabulated,
    _OyeBEMModelLinear as _OyeBEMModelLinear,
    _OyeBEMModelTabulated as _OyeBEMModelTabulated,
    _VpmRotorLinear as _VpmRotorLinear,
    _VpmRotorTabulated as _VpmRotorTabulated,
    ControlProperties as _RustControl,
    RotorDefinition as _RustDefinition,
    _QuasiStaticModel, _PittPetersModel, _OyeModel, _VpmModel,
    relax_inflow_py as relax_inflow,
    solve_trim_cyclic_py as solve_trim_cyclic,
)
from .rotor_definition import (
    AutorotationProperties as AutorotationProperties,
    BladeGeometry as BladeGeometry,
    ControlProperties as ControlProperties,
    InertiaProperties as InertiaProperties,
    KamanFlap as KamanFlap,
    LinearPolarParameters as LinearPolarParameters,
    RotorDefinition as RotorDefinition,
    _to_rust_control as _to_rust_control,
    _to_rust_defn as _to_rust_defn,
)
from .factory import (
    create_aero as create_aero,
    build_polar as build_polar,
    load_tabulated_polar as load_tabulated_polar,
    _build_polar_from_defn as _build_polar_from_defn,
)
from .mechanical import omega_derivative as omega_derivative, step_omega as step_omega

_cyclic_coeffs_rust = _native.cyclic_coeffs

__all__ = [
    "cyclic_coeffs", "create_aero", "build_polar", "load_tabulated_polar",
    "solve_trim_cyclic", "relax_inflow", "LinearPolar", "TabulatedPolar",
    "AirfoilPolar", "KamanFlap", "BladeGeometry", "LinearPolarParameters",
    "InertiaProperties", "ControlProperties", "AutorotationProperties",
    "RotorDefinition", "QuasiStaticRotorState", "PittPetersRotorState",
    "OyeRotorState", "VpmRotorState", "RotorInputs", "AeroResult",
    "QuasiStaticBEM", "BEMModel", "PittPetersModel", "OyeBEMModel",
    "VpmRotor", "TrimResult",
]

AirfoilPolar: tuple[type[LinearPolar], type[TabulatedPolar]]

def cyclic_coeffs(tilt_lon: float, tilt_lat: float, control: ControlProperties | _RustControl | None = None) -> tuple[float, float]: ...
@_overload
def QuasiStaticBEM(defn: RotorDefinition | _RustDefinition, polar: LinearPolar, n_psi_elements: int = 36) -> _QuasiStaticBEMLinear: ...
@_overload
def QuasiStaticBEM(defn: RotorDefinition | _RustDefinition, polar: TabulatedPolar, n_psi_elements: int = 36) -> _QuasiStaticBEMTabulated: ...
@_overload
def QuasiStaticBEM(defn: RotorDefinition, polar: None = None, n_psi_elements: int = 36) -> _QuasiStaticModel: ...

BEMModel = QuasiStaticBEM

@_overload
def PittPetersModel(defn: RotorDefinition | _RustDefinition, polar: LinearPolar, n_psi_elements: int = 36) -> _PittPetersModelLinear: ...
@_overload
def PittPetersModel(defn: RotorDefinition | _RustDefinition, polar: TabulatedPolar, n_psi_elements: int = 36) -> _PittPetersModelTabulated: ...
@_overload
def PittPetersModel(defn: RotorDefinition, polar: None = None, n_psi_elements: int = 36) -> _PittPetersModel: ...

@_overload
def OyeBEMModel(defn: RotorDefinition | _RustDefinition, polar: LinearPolar, n_psi_elements: int = 36, coupling_k: float = 0.6) -> _OyeBEMModelLinear: ...
@_overload
def OyeBEMModel(defn: RotorDefinition | _RustDefinition, polar: TabulatedPolar, n_psi_elements: int = 36, coupling_k: float = 0.6) -> _OyeBEMModelTabulated: ...
@_overload
def OyeBEMModel(defn: RotorDefinition, polar: None = None, n_psi_elements: int = 36, coupling_k: float = 0.6) -> _OyeModel: ...

class _VpmConfig(_TypedDict, total=False):
    max_particles: int
    sigma: float
    relax: float
    nonlinear_lifting_line: bool
    tip_clustering: bool
    local_core: bool
    barnes_hut: bool
    bh_theta: float
    bh_min_particles: int
    merge_wake: bool
    merge_every: int
    merge_min_particles: int
    merge_kappa: float
    merge_chi_min: float
    merge_region_dist: float
    core_spread_nu: float
    strength_decay_tau_rev: float

@_overload
def VpmRotor(defn: RotorDefinition | _RustDefinition, polar: LinearPolar, **config: _Unpack[_VpmConfig]) -> _VpmRotorLinear: ...
@_overload
def VpmRotor(defn: RotorDefinition | _RustDefinition, polar: TabulatedPolar, **config: _Unpack[_VpmConfig]) -> _VpmRotorTabulated: ...
@_overload
def VpmRotor(defn: RotorDefinition, polar: None = None, **config: _Unpack[_VpmConfig]) -> _VpmModel: ...
