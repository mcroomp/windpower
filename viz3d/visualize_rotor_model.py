"""Interactive 3D concept model of the RAWES rotor kite.

The rotor planform and tether attachment offset come from a DynBEM rotor
definition. Axial hub dimensions are visual assumptions until CAD or measured
dimensions are available.

Usage:
    python viz3d/visualize_rotor_model.py
    python viz3d/visualize_rotor_model.py --rotor path/to/rotor.yaml
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pyvista as pv
from dynbem import rotor_definition

import simulation


@dataclass(frozen=True)
class VisualAssumptions:
    """Dimensions not yet specified by the rotor definition, in metres."""

    rotor_hub_radius: float = 0.15
    rotor_hub_thickness: float = 0.08
    instrument_hub_radius: float = 0.15
    instrument_hub_thickness: float = 0.08
    axle_radius: float = 0.0125
    axle_protrusion: float = 0.10
    blade_thickness: float = 0.0225
    motor_radius: float = 0.105
    motor_height: float = 0.16
    antenna_length: float = 0.09
    antenna_radius: float = 0.008
    root_housing_length: float = 0.18
    root_housing_width: float = 0.14
    root_housing_height: float = 0.12
    structure_rod_radius: float = 0.014
    blade_link_length: float = 0.13
    blade_link_embed: float = 0.02


WOOD = "#d7a85d"
WOOD_EDGE = "#8b5a2b"
ROTOR_HUB = "#2f6f9f"
INSTRUMENT_HUB = "#c74332"
MOTOR = "#d9dde0"
AXLE = "#c4cbd0"
HARDWARE = "#b9c2c8"
ROOT_HOUSING = "#f1f0e8"
STRUCTURE = "#151718"
ANTENNA = "#111315"
TETHER = "#26282a"
BACKGROUND = "#e9ece8"
FLOOR = "#c9cec8"


def _default_rotor_path() -> Path:
    simulation_dir = Path(simulation.__file__).resolve().parent
    return simulation_dir / "rotor_definitions" / "beaupoil_2026.yaml"


def _blade_mesh(root_radius: float, tip_radius: float, chord: float,
                thickness: float) -> pv.PolyData:
    """Create one chamfer-tipped blade pointing along local +X."""
    half_chord = chord / 2.0
    half_thickness = thickness / 2.0
    chamfer = min(chord * 0.22, (tip_radius - root_radius) * 0.08)
    outline = np.array([
        [root_radius, -half_chord],
        [tip_radius - chamfer, -half_chord],
        [tip_radius, -half_chord + chamfer],
        [tip_radius, half_chord - chamfer],
        [tip_radius - chamfer, half_chord],
        [root_radius, half_chord],
    ])
    points = np.vstack([
        np.column_stack([outline, np.full(6, -half_thickness)]),
        np.column_stack([outline, np.full(6, half_thickness)]),
    ])
    faces: list[int] = [6, 5, 4, 3, 2, 1, 0, 6, 6, 7, 8, 9, 10, 11]
    for index in range(6):
        next_index = (index + 1) % 6
        faces.extend([4, index, next_index, next_index + 6, index + 6])
    return pv.PolyData(points, np.asarray(faces))


def _add_cylinder(plotter: pv.Plotter, *, center: tuple[float, float, float],
                  direction: tuple[float, float, float], radius: float,
                  height: float, color: str, metallic: float = 0.0) -> None:
    mesh = pv.Cylinder(
        center=center,
        direction=direction,
        radius=radius,
        height=height,
        resolution=72,
        capping=True,
    )
    plotter.add_mesh(
        mesh,
        color=color,
        smooth_shading=True,
        metallic=metallic,
        roughness=0.45,
    )


def build_scene(plotter: pv.Plotter, rotor_path: Path) -> None:
    rotor = rotor_definition.load(str(rotor_path))
    blade = getattr(rotor, "blade", rotor)
    assumptions = VisualAssumptions()

    rotor_hub_z = 0.0
    instrument_hub_z = (
        assumptions.rotor_hub_thickness / 2.0
        + assumptions.instrument_hub_thickness / 2.0
        + 0.035
    )
    axle_bottom_z = -assumptions.rotor_hub_thickness / 2.0 - assumptions.axle_protrusion
    axle_top_z = (
        instrument_hub_z
        + assumptions.instrument_hub_thickness / 2.0
        + assumptions.axle_protrusion
    )
    tether_z = axle_bottom_z

    _add_cylinder(
        plotter,
        center=(0.0, 0.0, rotor_hub_z),
        direction=(0.0, 0.0, 1.0),
        radius=assumptions.rotor_hub_radius,
        height=assumptions.rotor_hub_thickness,
        color=ROTOR_HUB,
        metallic=0.15,
    )

    housing_radius = float(blade.root_cutout_m) + 0.035
    housing_outer_radius = housing_radius + assumptions.root_housing_length / 2.0
    visible_blade_root = (
        housing_outer_radius
        + assumptions.blade_link_length
        - assumptions.blade_link_embed
    )
    blade_mesh = _blade_mesh(
        visible_blade_root,
        float(blade.radius_m),
        float(blade.chord_m),
        assumptions.blade_thickness,
    )
    housing_centers: list[np.ndarray] = []
    for blade_index in range(int(blade.n_blades)):
        angle_deg = blade_index * 360.0 / int(blade.n_blades)
        mesh = blade_mesh.copy()
        mesh.rotate_z(angle_deg, inplace=True)
        plotter.add_mesh(
            mesh,
            color=WOOD,
            edge_color=WOOD_EDGE,
            show_edges=True,
            smooth_shading=True,
            roughness=0.72,
        )

        angle = np.radians(angle_deg)
        radial = np.array([np.cos(angle), np.sin(angle), 0.0])
        housing_center = radial * housing_radius
        housing_centers.append(housing_center)

        link_end_radius = housing_radius - assumptions.root_housing_length / 2.0
        link_length = link_end_radius - assumptions.rotor_hub_radius
        link_center = radial * (assumptions.rotor_hub_radius + link_length / 2.0)
        link = pv.Cylinder(
            center=link_center,
            direction=radial,
            radius=assumptions.structure_rod_radius,
            height=link_length,
            resolution=32,
        )
        plotter.add_mesh(link, color=STRUCTURE, metallic=0.15, roughness=0.65)

        blade_link_center = radial * (
            housing_outer_radius + assumptions.blade_link_length / 2.0
        )
        blade_link = pv.Cylinder(
            center=blade_link_center,
            direction=radial,
            radius=assumptions.structure_rod_radius,
            height=assumptions.blade_link_length,
            resolution=32,
        )
        plotter.add_mesh(
            blade_link,
            color=STRUCTURE,
            metallic=0.15,
            roughness=0.65,
        )

        housing = pv.Box(bounds=(
            housing_radius - assumptions.root_housing_length / 2.0,
            housing_radius + assumptions.root_housing_length / 2.0,
            -assumptions.root_housing_width / 2.0,
            assumptions.root_housing_width / 2.0,
            -assumptions.root_housing_height / 2.0,
            assumptions.root_housing_height / 2.0,
        ))
        housing.rotate_z(angle_deg, inplace=True)
        plotter.add_mesh(
            housing,
            color=ROOT_HOUSING,
            edge_color="#c9c9c2",
            show_edges=True,
            roughness=0.75,
        )

    brace_z = 0.0
    for index, housing_center in enumerate(housing_centers):
        next_center = housing_centers[(index + 1) % len(housing_centers)]
        point_a = housing_center.copy()
        point_b = next_center.copy()
        point_a[2] = brace_z
        point_b[2] = brace_z
        brace = pv.Tube(
            pointa=point_a,
            pointb=point_b,
            radius=assumptions.structure_rod_radius,
            n_sides=16,
        )
        plotter.add_mesh(brace, color=STRUCTURE, metallic=0.1, roughness=0.75)

    _add_cylinder(
        plotter,
        center=(0.0, 0.0, instrument_hub_z),
        direction=(0.0, 0.0, 1.0),
        radius=assumptions.instrument_hub_radius,
        height=assumptions.instrument_hub_thickness,
        color=INSTRUMENT_HUB,
        metallic=0.35,
    )
    antenna_center = (
        assumptions.instrument_hub_radius + assumptions.antenna_length / 2.0,
        0.0,
        instrument_hub_z,
    )
    _add_cylinder(
        plotter,
        center=antenna_center,
        direction=(1.0, 0.0, 0.0),
        radius=assumptions.antenna_radius,
        height=assumptions.antenna_length,
        color=ANTENNA,
        metallic=0.15,
    )
    antenna_tip = (
        assumptions.instrument_hub_radius + assumptions.antenna_length,
        0.0,
        instrument_hub_z,
    )
    plotter.add_mesh(
        pv.Sphere(radius=assumptions.antenna_radius * 1.5, center=antenna_tip),
        color=ANTENNA,
        smooth_shading=True,
    )

    axle_center_z = (axle_top_z + axle_bottom_z) / 2.0
    axle_height = axle_top_z - axle_bottom_z
    _add_cylinder(
        plotter,
        center=(0.0, 0.0, axle_center_z),
        direction=(0.0, 0.0, 1.0),
        radius=assumptions.axle_radius,
        height=axle_height,
        color=AXLE,
        metallic=0.7,
    )

    attachment = np.array([0.0, 0.0, tether_z])
    tether_end = np.array([0.65, -0.35, tether_z - 1.5])
    tether = pv.Tube(
        pointa=attachment,
        pointb=tether_end,
        radius=0.012,
        n_sides=16,
    )
    plotter.add_mesh(tether, color=TETHER, roughness=0.9)

    floor_z = tether_end[2] - 0.04
    floor = pv.Plane(center=(0.0, 0.0, floor_z), i_size=6.5, j_size=6.5)
    plotter.add_mesh(floor, color=FLOOR, opacity=0.32, roughness=1.0)

    plotter.add_text(
        f"RAWES rotor concept  |  {rotor.name}\n"
        f"diameter {2 * blade.radius_m:.1f} m  |  "
        f"{blade.n_blades} blades x {blade.chord_m:.2f} m chord",
        position="upper_left",
        font_size=10,
        color="#263238",
    )
    plotter.add_axes(line_width=2, color="#4b555b")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rotor",
        type=Path,
        default=_default_rotor_path(),
        help="DynBEM rotor-definition YAML (default: beaupoil_2026.yaml)",
    )
    args = parser.parse_args()

    plotter = pv.Plotter(title="RAWES rotor-kite model", window_size=(1200, 850))
    plotter.set_background(BACKGROUND)
    build_scene(plotter, args.rotor)
    plotter.enable_anti_aliasing("ssaa")
    plotter.camera_position = [
        (4.0, -4.3, 2.9),
        (0.0, 0.0, -0.25),
        (0.0, 0.0, 1.0),
    ]
    plotter.show()


if __name__ == "__main__":
    main()