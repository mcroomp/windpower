"""Two-ended tether physics, with forces expressed in NED world coordinates."""

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np
from scipy.optimize import least_squares


DEFAULT_TETHER_MODEL = "tallak_quasi_static"


@dataclass(frozen=True)
class TetherProperties:
    """Nominal Dyneema SK75 1.9 mm properties; EA includes braid efficiency."""

    axial_stiffness_n: float = 109e9 * np.pi * (0.0019 / 2.0) ** 2 * 0.91
    diameter_m: float = 0.0019
    linear_mass_kg_m: float = 0.0021
    break_load_n: float = 620.0
    drag_coefficient: float = 1.1

    def __post_init__(self) -> None:
        for name in ("axial_stiffness_n", "diameter_m", "break_load_n"):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("linear_mass_kg_m", "drag_coefficient"):
            value = getattr(self, name)
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and nonnegative")


@dataclass(frozen=True)
class TetherInputs:
    """Physical endpoint kinematics and deployed unstretched length.

    Positive payout_rate_ms means material leaves the drum. Wind and gravity
    are spatially uniform in this first implementation.
    """

    anchor_pos_ned: np.ndarray
    anchor_vel_ned: np.ndarray
    attachment_pos_ned: np.ndarray
    attachment_vel_ned: np.ndarray
    rest_length_m: float
    payout_rate_ms: float = 0.0
    wind_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))
    air_density_kg_m3: float = 1.225
    gravity_ned: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 9.81]))

    def __post_init__(self) -> None:
        for name in (
            "anchor_pos_ned", "anchor_vel_ned", "attachment_pos_ned",
            "attachment_vel_ned", "wind_ned", "gravity_ned",
        ):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (3,) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must be a finite NED three-vector")
            object.__setattr__(self, name, value)
        if not np.isfinite(self.rest_length_m) or self.rest_length_m <= 0.0:
            raise ValueError("rest_length_m must be finite and positive")
        if not np.isfinite(self.payout_rate_ms):
            raise ValueError("payout_rate_ms must be finite")
        if not np.isfinite(self.air_density_kg_m3) or self.air_density_kg_m3 < 0.0:
            raise ValueError("air_density_kg_m3 must be finite and nonnegative")


@dataclass(frozen=True)
class TetherResult:
    """Forces ON the two attached bodies, never forces on the cable itself."""

    force_on_hub_ned: np.ndarray
    force_on_anchor_ned: np.ndarray
    positions_ned: np.ndarray
    length_m: float
    extension_m: float
    slack: bool
    distributed_force_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))
    elastic_energy_j: float = 0.0
    damping_power_w: float = 0.0
    iterations: int = 0
    residual_m: float = 0.0

    @property
    def hub_tension_n(self) -> float:
        return float(np.linalg.norm(self.force_on_hub_ned))

    @property
    def anchor_tension_n(self) -> float:
        return float(np.linalg.norm(self.force_on_anchor_ned))

    @classmethod
    def unloaded(cls, inputs: TetherInputs) -> "TetherResult":
        length = float(np.linalg.norm(inputs.attachment_pos_ned - inputs.anchor_pos_ned))
        return cls(
            np.zeros(3), np.zeros(3),
            np.array([inputs.anchor_pos_ned, inputs.attachment_pos_ned]),
            length, length - inputs.rest_length_m, True,
        )


class Tether(Protocol):
    properties: TetherProperties

    def evaluate(self, inputs: TetherInputs) -> TetherResult: ...


class SpringDamperTether:
    """The original straight, tension-only EA/chord-length spring.

    Retains zeta=0.05 and extension-only damping. Unlike the old hub-centre
    interface, damping uses endpoint-relative motion minus payout rate.
    """

    def __init__(
        self, *, properties: TetherProperties = TetherProperties(),
        zeta: float = 0.05, hub_mass: float = 5.0,
    ) -> None:
        if not np.isfinite(zeta) or zeta < 0.0:
            raise ValueError("zeta must be finite and nonnegative")
        if not np.isfinite(hub_mass) or hub_mass <= 0.0:
            raise ValueError("hub_mass must be finite and positive")
        self.properties = properties
        self.zeta = float(zeta)
        self.hub_mass = float(hub_mass)

    def damping(self, length_m: float) -> float:
        return self.zeta * 2.0 * float(np.sqrt(
            self.properties.axial_stiffness_n / length_m * self.hub_mass
        ))

    def evaluate(self, inputs: TetherInputs) -> TetherResult:
        chord = inputs.attachment_pos_ned - inputs.anchor_pos_ned
        length = float(np.linalg.norm(chord))
        extension = length - inputs.rest_length_m
        if length < 1e-6 or extension <= 0.0:
            return TetherResult.unloaded(inputs)
        tangent = chord / length
        stiffness = self.properties.axial_stiffness_n / length
        extension_rate = float(np.dot(
            inputs.attachment_vel_ned - inputs.anchor_vel_ned, tangent,
        )) - inputs.payout_rate_ms
        damping_force = self.damping(length) * max(0.0, extension_rate)
        tension = stiffness * extension + damping_force
        force = tension * tangent
        return TetherResult(
            -force, force,
            np.array([inputs.anchor_pos_ned, inputs.attachment_pos_ned]),
            length, extension, False,
            elastic_energy_j=self.properties.axial_stiffness_n * inputs.rest_length_m * (
                extension / inputs.rest_length_m - np.log1p(extension / inputs.rest_length_m)
            ),
            damping_power_w=damping_force * max(0.0, extension_rate),
        )


class TetherConvergenceError(RuntimeError):
    """The requested suspended-line equilibrium could not be resolved."""


class TallakQuasiStaticTether:
    """Extensible segmented equilibrium inspired by Tallak Tveide's solver.

    https://github.com/tallakt/TetherDragODESolver

    This is an independent wind/gravity boundary-value implementation, not a
    port of the rotating-line Julia ODE. No quarter multiplier is used.
    It models elastic equilibrium, not inertia, material hysteresis, ground
    contact, or payout momentum flux. Cached forces only warm-start the solve.
    """

    def __init__(
        self, *, properties: TetherProperties = TetherProperties(),
        segments: int = 16,
    ) -> None:
        if isinstance(segments, bool) or not isinstance(segments, int) or segments < 2:
            raise ValueError("segments must be an integer >= 2")
        self.properties = properties
        self.segments = segments
        self._anchor_force: np.ndarray | None = None
        self._tangents: np.ndarray | None = None
        self._segment_tensions: np.ndarray | None = None

    def evaluate(self, inputs: TetherInputs) -> TetherResult:
        chord = inputs.attachment_pos_ned - inputs.anchor_pos_ned
        distance = float(np.linalg.norm(chord))
        if distance < 1e-6:
            raise ValueError("quasi-static tether requires distinct endpoints")
        p = self.properties
        ds = inputs.rest_length_m / self.segments
        fractions = (np.arange(self.segments) + 0.5) / self.segments
        velocities = (
            (1.0 - fractions[:, None]) * inputs.anchor_vel_ned
            + fractions[:, None] * inputs.attachment_vel_ned
        )
        tangents = np.tile(chord / distance, (self.segments, 1))
        lengths = np.full(self.segments, distance / self.segments)
        if self._tangents is not None and self._segment_tensions is not None:
            tangents = self._tangents
            lengths = ds * (1.0 + self._segment_tensions / p.axial_stiffness_n)

        def segment_loads(directions: np.ndarray, spans: np.ndarray) -> np.ndarray:
            # Interpolate endpoint motion; axial material slip vanishes when
            # projected onto the local normal in this transverse-drag model.
            relative = inputs.wind_ned - velocities
            normal = relative - np.sum(relative * directions, axis=1)[:, None] * directions
            drag = (
                0.5 * inputs.air_density_kg_m3 * p.drag_coefficient * p.diameter_m
                * np.linalg.norm(normal, axis=1)[:, None] * normal * spans[:, None]
            )
            return drag + p.linear_mass_kg_m * ds * inputs.gravity_ned

        loads = segment_loads(tangents, lengths)
        if not np.any(loads):
            if distance <= inputs.rest_length_m:
                return TetherResult.unloaded(inputs)
            tension = p.axial_stiffness_n * (distance / inputs.rest_length_m - 1.0)
            force = tension * chord / distance
            return TetherResult(
                -force, force,
                np.linspace(inputs.anchor_pos_ned, inputs.attachment_pos_ned, self.segments + 1),
                distance, distance - inputs.rest_length_m, False,
                elastic_energy_j=0.5 * tension**2 / p.axial_stiffness_n * inputs.rest_length_m,
            )

        total_load = np.sum(loads, axis=0)
        sag = np.sqrt(max(0.0, 3.0 * distance * (inputs.rest_length_m - distance) / 8.0))
        initial_tension = max(
            p.axial_stiffness_n * (distance / inputs.rest_length_m - 1.0),
            float(np.linalg.norm(total_load)) * distance / max(8.0 * sag, 1.0),
            0.01,
        )
        anchor_force = (
            self._anchor_force.copy() if self._anchor_force is not None
            else initial_tension * chord / distance + 0.5 * total_load
        )
        residual_norm = float("inf")
        for iteration in range(1, 81):
            offsets = np.cumsum(loads, axis=0) - 0.5 * loads

            def geometry(force: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
                tensions = force - offsets
                magnitudes = np.linalg.norm(tensions, axis=1)
                if np.any(magnitudes < 1e-12):
                    magnitudes = np.maximum(magnitudes, 1e-12)
                directions = tensions / magnitudes[:, None]
                spans = ds * (1.0 + magnitudes / p.axial_stiffness_n)
                return directions, spans, magnitudes

            def residual(force: np.ndarray) -> np.ndarray:
                directions, spans, _ = geometry(force)
                return np.sum(directions * spans[:, None], axis=0) - chord

            def jacobian(force: np.ndarray) -> np.ndarray:
                directions, _, magnitudes = geometry(force)
                outer = directions[:, :, None] * directions[:, None, :]
                return ds * np.sum(
                    (np.eye(3) - outer) / magnitudes[:, None, None]
                    + np.eye(3) / p.axial_stiffness_n,
                    axis=0,
                )

            solved = least_squares(
                # SciPy's untyped source infers jac as str, but accepts callables.
                residual, anchor_force, jac=jacobian, x_scale="jac",  # pyright: ignore[reportArgumentType]
                ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=100,
            )
            anchor_force = solved.x
            residual_norm = float(np.linalg.norm(residual(anchor_force)))
            if not solved.success or residual_norm > 1e-6:
                raise TetherConvergenceError(
                    f"Tallak tether endpoint solve failed: {solved.message}; "
                    f"residual={residual_norm:.6g} m"
                )
            tangents, lengths, tensions = geometry(anchor_force)
            new_loads = segment_loads(tangents, lengths)
            if float(np.max(np.linalg.norm(new_loads - loads, axis=1))) < 1e-7:
                break
            loads = 0.5 * (loads + new_loads)
        else:
            raise TetherConvergenceError("Tallak tether distributed-load iteration failed after 80 iterations")

        self._anchor_force = anchor_force.copy()
        self._tangents = tangents.copy()
        self._segment_tensions = tensions.copy()
        positions = np.vstack((
            inputs.anchor_pos_ned,
            inputs.anchor_pos_ned + np.cumsum(tangents * lengths[:, None], axis=0),
        ))
        total_load = np.sum(loads, axis=0)
        length = float(np.sum(lengths))
        return TetherResult(
            total_load - anchor_force, anchor_force, positions,
            length, length - inputs.rest_length_m, False,
            distributed_force_ned=total_load,
            elastic_energy_j=float(0.5 * ds * np.sum(tensions**2) / p.axial_stiffness_n),
            iterations=iteration, residual_m=residual_norm,
        )


class ConstantTensionTether:
    """Diagnostic fixed-load cable; not a model of distributed line physics."""

    def __init__(self, tension_n: float) -> None:
        if not np.isfinite(tension_n) or tension_n < 0.0:
            raise ValueError("tension_n must be finite and nonnegative")
        self.tension_n = float(tension_n)
        self.properties = TetherProperties()

    def evaluate(self, inputs: TetherInputs) -> TetherResult:
        chord = inputs.attachment_pos_ned - inputs.anchor_pos_ned
        length = float(np.linalg.norm(chord))
        if length < 1e-6:
            return TetherResult.unloaded(inputs)
        force = self.tension_n * chord / length
        return TetherResult(
            -force, force,
            np.array([inputs.anchor_pos_ned, inputs.attachment_pos_ned]),
            length, length - inputs.rest_length_m, self.tension_n == 0.0,
        )


def make_tether(model: str, *, hub_mass: float, constant_tension_n: float = 300.0) -> Tether:
    match model:
        case "spring_damper":
            return SpringDamperTether(hub_mass=hub_mass)
        case "tallak_quasi_static":
            return TallakQuasiStaticTether()
        case "constant_tension":
            return ConstantTensionTether(constant_tension_n)
        case _:
            raise ValueError(f"Unknown tether model: {model!r}")
