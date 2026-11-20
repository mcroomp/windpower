"""Ordered image-plane quadrilaterals; no square or affine shape constraint."""

import cv2
import numpy as np

type Point = tuple[float, float]
type Quad = tuple[Point, Point, Point, Point]

SHAPE_FIELDS = [
    *(f"corner_{i}_{axis}_px" for i in range(4) for axis in ("x", "y")),
    *(f"relative_corner_{i}_{axis}_px" for i in range(4) for axis in ("dx", "dy")),
    *(f"edge_{i}_px" for i in range(4)),
    *(f"edge_{i}_ratio" for i in range(4)),
    *(f"angle_{i}_deg" for i in range(4)),
    *(f"angle_{i}_delta_deg" for i in range(4)),
    "area_px2", "area_ratio",
]


def valid_quad(corners: Quad, width: int, height: int) -> bool:
    points = np.asarray(corners, dtype=np.float32)
    if points.shape != (4, 2) or not np.isfinite(points).all():
        return False
    edges = np.roll(points, -1, axis=0) - points
    return bool(
        (points[:, 0] >= 0).all() and (points[:, 0] < width).all()
        and (points[:, 1] >= 0).all() and (points[:, 1] < height).all()
        and (np.linalg.norm(edges, axis=1) >= 4).all()
        and cv2.isContourConvex(points)
        and cv2.contourArea(points, oriented=True) >= 32
    )


def to_quad(points: cv2.typing.MatLike) -> Quad:
    a, b, c, d = np.asarray(points).reshape(4, 2)
    return (
        (float(a[0]), float(a[1])), (float(b[0]), float(b[1])),
        (float(c[0]), float(c[1])), (float(d[0]), float(d[1])),
    )


def quad_center(corners: Quad) -> np.ndarray:
    p = np.asarray(corners, dtype=np.float64)
    # Diagonal intersection is the projected center of the physical square.
    t = np.linalg.solve(np.column_stack((p[2] - p[0], p[1] - p[3])), p[1] - p[0])[0]
    return p[0] + t * (p[2] - p[0])


def edge_lengths(corners: Quad) -> np.ndarray:
    points = np.asarray(corners, dtype=np.float64)
    return np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)


def corner_angles(corners: Quad) -> np.ndarray:
    points = np.asarray(corners, dtype=np.float64)
    before, after = np.roll(points, 1, axis=0) - points, np.roll(points, -1, axis=0) - points
    cosine = np.sum(before * after, axis=1) / (
        np.linalg.norm(before, axis=1) * np.linalg.norm(after, axis=1)
    )
    return np.degrees(np.arccos(np.clip(cosine, -1, 1)))


def shape_measurements(
    corners: Quad, reference: Quad, hub_displacement: tuple[float, float] | None,
) -> dict[str, float]:
    points = np.asarray(corners)
    baseline = np.asarray(reference)
    edges, angles = edge_lengths(corners), corner_angles(corners)
    ratios = edges / edge_lengths(reference)
    angle_delta = angles - corner_angles(reference)
    values = {}
    for i, point in enumerate(points):
        values[f"corner_{i}_x_px"], values[f"corner_{i}_y_px"] = float(point[0]), float(point[1])
        values[f"edge_{i}_px"], values[f"edge_{i}_ratio"] = float(edges[i]), float(ratios[i])
        values[f"angle_{i}_deg"] = float(angles[i])
        values[f"angle_{i}_delta_deg"] = float(angle_delta[i])
        if hub_displacement is not None:
            relative = point - baseline[i] - hub_displacement
            values[f"relative_corner_{i}_dx_px"] = float(relative[0])
            values[f"relative_corner_{i}_dy_px"] = float(relative[1])
    area = cv2.contourArea(np.asarray(corners, dtype=np.float32))
    values["area_px2"] = area
    values["area_ratio"] = area / cv2.contourArea(np.asarray(reference, dtype=np.float32))
    return values
