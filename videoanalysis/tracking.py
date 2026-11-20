"""Template localization, hub tracking, and independent physical-corner tracking."""

from dataclasses import dataclass
import math
from typing import Self

import cv2
import numpy as np

from videoanalysis.geometry import Quad, edge_lengths, quad_center, to_quad, valid_quad
from videoanalysis.photometry import TEMPLATE_SIGMA_PX

type Image = cv2.typing.MatLike
type Rect = tuple[int, int, int, int]


@dataclass(frozen=True)
class Region:
    name: str
    rect: Rect
    corners: Quad | None = None

    @classmethod
    def from_corners(cls, name: str, corners: Quad) -> Self:
        points = np.asarray(corners)
        left, top = np.floor(points.min(axis=0)).astype(int)
        right, bottom = np.ceil(points.max(axis=0)).astype(int) + 1
        return cls(name, (int(left), int(top), int(right - left), int(bottom - top)), corners)

    def validate(self, image: Image) -> None:
        x, y, width, height = self.rect
        if (
            width < 8 or height < 8 or x < 0 or y < 0
            or x + width > image.shape[1] or y + height > image.shape[0]
        ):
            raise ValueError(f"{self.name}: region must be at least 8x8 and inside the frame")
        if self.corners is not None and not valid_quad(self.corners, image.shape[1], image.shape[0]):
            raise ValueError(f"{self.name}: corners must form a convex clockwise quadrilateral inside the frame")


@dataclass(frozen=True)
class Match:
    status: str
    score: float
    peak_gap: float
    dx: float | None = None
    dy: float | None = None
    method: str = "template"
    tracked_points: int = 0
    inlier_ratio: float | None = None
    fb_error_px: float | None = None
    corners: Quad | None = None

    @property
    def valid(self) -> bool:
        return self.status == "ok"


@dataclass(frozen=True)
class PointFlow:
    points: Image
    keep: np.ndarray
    fb_error: Image


def track_points(
    before: Image, after: Image, points: Image, guesses: Image, radius: int, clahe: cv2.CLAHE | None,
    window_size: int = 21, max_level: int = 2,
) -> PointFlow | None:
    extent = np.concatenate((points.reshape(-1, 2), guesses.reshape(-1, 2)))
    left, top = np.maximum(0, np.floor(extent.min(axis=0)).astype(int) - radius - 12)
    right, bottom = np.minimum(
        [after.shape[1], after.shape[0]], np.ceil(extent.max(axis=0)).astype(int) + radius + 12,
    )
    offset = np.asarray([left, top], dtype=np.float32)
    previous = before[top:bottom, left:right]
    current = after[top:bottom, left:right]
    if clahe is not None:
        previous, current = clahe.apply(previous), clahe.apply(current)
    source = np.asarray(points - offset, dtype=np.float32)
    target = np.asarray(guesses - offset, dtype=np.float32)
    criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 0.01)
    following, status, error = cv2.calcOpticalFlowPyrLK(
        previous, current, source, target, winSize=(window_size, window_size), maxLevel=max_level,
        criteria=criteria, flags=cv2.OPTFLOW_USE_INITIAL_FLOW,
    )
    if following is None or status is None or error is None:
        return None
    # The reverse guess is known; coarse levels can jump between similar marker corners.
    returning, reverse_status, _ = cv2.calcOpticalFlowPyrLK(
        current, previous, following, source.copy(), winSize=(window_size, window_size), maxLevel=0,
        criteria=criteria, flags=cv2.OPTFLOW_USE_INITIAL_FLOW,
    )
    if returning is None or reverse_status is None:
        return None
    fb = np.linalg.norm((returning - source).reshape(-1, 2), axis=1)
    positions = (following + offset).reshape(-1, 2)
    keep = (
        (status.ravel() == 1) & (reverse_status.ravel() == 1)
        & (fb <= 1.0) & (error.ravel() <= 30)
        & np.isfinite(positions).all(axis=1)
        & (positions[:, 0] >= 0) & (positions[:, 0] < after.shape[1])
        & (positions[:, 1] >= 0) & (positions[:, 1] < after.shape[0])
    )
    return PointFlow(positions, keep, fb)


class TemplateTracker:
    def __init__(
        self,
        first_gray: Image,
        region: Region,
        search_radius: int = 60,
        min_score: float = 0.75,
        min_peak_gap: float = 0.03,
        normalized_input: bool = False,
    ) -> None:
        if first_gray.ndim != 2 or first_gray.dtype != np.uint8:
            raise ValueError("Tracking requires 8-bit grayscale frames")
        region.validate(first_gray)
        if search_radius < 1:
            raise ValueError("Search radius must be positive")
        if not math.isfinite(min_score) or not 0 < min_score <= 1:
            raise ValueError("Minimum score must be in (0, 1]")
        if not math.isfinite(min_peak_gap) or not 0 <= min_peak_gap <= 2:
            raise ValueError("Minimum peak gap must be in [0, 2]")
        self.region = region
        self.search_radius = search_radius
        self.min_score = min_score
        self.min_peak_gap = min_peak_gap
        self.normalized_input = normalized_input
        x, y, width, height = region.rect
        template_frame = (
            cv2.GaussianBlur(first_gray, (0, 0), TEMPLATE_SIGMA_PX) if normalized_input else first_gray
        )
        self.template = template_frame[y:y + height, x:x + width].copy()
        if float(self.template.std()) < 5:
            raise ValueError(f"{region.name}: insufficient texture; select a distinctive marking")
        self.position = (float(x), float(y))
        self.shape = first_gray.shape

    def update(self, gray: Image) -> Match:
        if gray.shape != self.shape or gray.dtype != np.uint8:
            raise ValueError("Frame dimensions or pixel type changed during tracking")
        x, y, width, height = self.region.rect
        last_x, last_y = self.position
        radius = self.search_radius
        left = max(0, round(last_x) - radius)
        top = max(0, round(last_y) - radius)
        right = min(gray.shape[1], round(last_x) + width + radius)
        bottom = min(gray.shape[0], round(last_y) + height + radius)
        search = gray[top:bottom, left:right]
        if self.normalized_input:
            search = cv2.GaussianBlur(search, (0, 0), TEMPLATE_SIGMA_PX)
        response = cv2.matchTemplate(search, self.template, cv2.TM_CCOEFF_NORMED)
        if not np.isfinite(response).all():
            return Match("nonfinite_score", 0.0, 0.0)
        _, score, _, (px, py) = cv2.minMaxLoc(response)
        competitors = response.copy()
        exclusion = max(3, min(width, height) // 4)
        competitors[
            max(0, py - exclusion):py + exclusion + 1,
            max(0, px - exclusion):px + exclusion + 1,
        ] = -np.inf
        second = float(competitors.max())
        gap = float(score - second) if math.isfinite(second) else 2.0
        if score < self.min_score:
            return Match("low_score", score, gap)
        if gap < self.min_peak_gap:
            return Match("ambiguous", score, gap)
        # A peak at an interior search boundary may be a clipped, larger motion.
        if (
            (px == 0 and left > 0) or (py == 0 and top > 0)
            or (px == response.shape[1] - 1 and right < gray.shape[1])
            or (py == response.shape[0] - 1 and bottom < gray.shape[0])
        ):
            return Match("search_boundary", score, gap)
        offset_x = self._subpixel(response[py, :], px)
        offset_y = self._subpixel(response[:, px], py)
        found_x, found_y = left + px + offset_x, top + py + offset_y
        self.position = (found_x, found_y)
        return Match("ok", score, gap, found_x - x, found_y - y)

    @staticmethod
    def _subpixel(values: Image, index: int) -> float:
        if index == 0 or index == len(values) - 1:
            return 0.0
        a, b, c = (float(v) for v in values[index - 1:index + 2])
        denominator = a - 2 * b + c
        if denominator >= -1e-8:
            return 0.0
        return float(np.clip(0.5 * (a - c) / denominator, -0.5, 0.5))


class PatchTracker:
    """Use absolute template anchors and illumination-normalized sparse flow between them."""

    def __init__(
        self, first_gray: Image, region: Region, search_radius: int = 60, min_score: float = 0.75,
        normalized_input: bool = False,
    ) -> None:
        self.anchor = TemplateTracker(
            first_gray, region, search_radius, min_score, normalized_input=normalized_input,
        )
        self.region = region
        self.previous_gray = first_gray
        self.points: Image | None = None
        self.origins: Image | None = None
        self.transform = np.eye(2, 3, dtype=np.float64)
        self.clahe = None if normalized_input else cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))

    def _anchor_points(self, gray: Image, match: Match) -> None:
        assert match.dx is not None and match.dy is not None
        x, y, w, h = self.region.rect
        x, y = round(x + match.dx), round(y + match.dy)
        mask = np.zeros(gray.shape, dtype=np.uint8)
        mask[max(0, y + 2):y + h - 2, max(0, x + 2):x + w - 2] = 255
        self.points = cv2.goodFeaturesToTrack(
            gray, maxCorners=60, qualityLevel=0.05, minDistance=3, mask=mask, blockSize=3,
        )
        self.origins = (
            self.points - np.asarray([match.dx, match.dy], dtype=np.float32)
            if self.points is not None else None
        )
        self.transform = np.asarray([[1, 0, match.dx], [0, 1, match.dy]], dtype=np.float64)
        self.previous_gray = gray

    def update(self, gray: Image) -> Match:
        candidate = self.anchor.update(gray)
        if candidate.valid:
            self._anchor_points(gray, candidate)
            return candidate
        if self.points is None or self.origins is None:
            return candidate
        if len(self.points) < 6:
            return self._lost(candidate, "insufficient_features")
        flow = track_points(
            self.previous_gray, gray, self.points, self.points, self.anchor.search_radius, self.clahe,
        )
        if flow is None:
            return self._lost(candidate, "flow_failed")
        keep = flow.keep
        if keep.sum() < 6 or keep.mean() < 0.6:
            return self._lost(candidate, "insufficient_flow")
        origins = self.origins.reshape(-1, 2)[keep]
        current = flow.points[keep]
        transform, inliers = cv2.estimateAffine2D(
            origins, current, method=cv2.RANSAC, ransacReprojThreshold=1.5,
            maxIters=1000, confidence=0.99,
        )
        if transform is None or inliers is None or not np.isfinite(transform).all():
            return self._lost(candidate, "inconsistent_flow")
        inliers = inliers.ravel() == 1
        if inliers.sum() < 6 or inliers.mean() < 0.7:
            return self._lost(candidate, "inconsistent_flow")
        change = transform[:, :2] @ np.linalg.inv(self.transform[:, :2])
        scales = np.linalg.svd(change, compute_uv=False)
        if np.linalg.det(change) <= 0 or scales.min() < 0.7 or scales.max() > 1.4:
            return self._lost(candidate, "implausible_flow")
        x, y, w, h = self.region.rect
        center = np.asarray([x + w / 2, y + h / 2, 1.0])
        motion = (transform @ center) - (self.transform @ center)
        if np.max(np.abs(motion)) >= self.anchor.search_radius:
            return self._lost(candidate, "search_boundary")
        dx, dy = transform @ center - center[:2]
        self.points = np.asarray(current[inliers].reshape(-1, 1, 2), dtype=np.float32)
        self.origins = np.asarray(origins[inliers].reshape(-1, 1, 2), dtype=np.float32)
        self.transform = transform
        self.previous_gray = gray
        self.anchor.position = (x + float(dx), y + float(dy))
        return Match(
            "ok", candidate.score, candidate.peak_gap, float(dx), float(dy), "flow",
            int(inliers.sum()), float(inliers.mean()), float(np.median(flow.fb_error[keep][inliers])),
        )

    def _lost(self, candidate: Match, status: str) -> Match:
        self.points = self.origins = None
        return Match(status, candidate.score, candidate.peak_gap, method="flow")


class QuadTracker:
    """Track four physical corners independently, never replace shape with translation."""

    def __init__(
        self, first_gray: Image, region: Region, search_radius: int = 60, min_score: float = 0.75,
        normalized_input: bool = False,
    ) -> None:
        region.validate(first_gray)
        if region.corners is None:
            raise ValueError("Blade markers require four physical corners")
        self.reference = region.corners
        self.region = region
        x, y, w, h = region.rect
        left, top = max(0, x - 4), max(0, y - 4)
        right, bottom = min(first_gray.shape[1], x + w + 4), min(first_gray.shape[0], y + h + 4)
        self.anchor = TemplateTracker(
            first_gray, Region(region.name, (left, top, right - left, bottom - top)),
            search_radius, min_score, normalized_input=normalized_input,
        )
        self.first_gray = self.previous_gray = first_gray
        self.previous: Quad | None = region.corners
        self.initialized = False
        self.clahe = None if normalized_input else cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))

    def _follow(
        self, source_gray: Image, gray: Image, source: Quad, guesses: Quad, candidate: Match, method: str,
    ) -> Match:
        shortest_edge = float(edge_lengths(source).min())
        window = max(5, min(21, int(shortest_edge / 2) | 1))
        # Keep each corner's pyramid neighborhood smaller than its neighboring edge.
        level = max(0, min(2, int(math.log2(max(1, shortest_edge / window)))))
        flow = track_points(
            source_gray, gray, np.asarray(source, dtype=np.float32).reshape(4, 1, 2),
            np.asarray(guesses, dtype=np.float32).reshape(4, 1, 2),
            self.anchor.search_radius, self.clahe,
            window, level,
        )
        if flow is None or not flow.keep.all():
            return Match("corner_flow_failed", candidate.score, candidate.peak_gap, method=method)
        corners = to_quad(flow.points)
        if not valid_quad(corners, gray.shape[1], gray.shape[0]):
            return Match("invalid_quadrilateral", candidate.score, candidate.peak_gap, method=method)
        ratios = edge_lengths(corners) / edge_lengths(source)
        motion = np.abs(np.asarray(corners) - source)
        if ratios.min() < 0.7 or ratios.max() > 1.4 or motion.max() >= self.anchor.search_radius:
            return Match("implausible_corners", candidate.score, candidate.peak_gap, method=method)
        dx, dy = quad_center(corners) - quad_center(self.reference)
        return Match(
            "ok", candidate.score, candidate.peak_gap, float(dx), float(dy),
            method, 4, None, float(flow.fb_error.max()), corners,
        )

    def update(self, gray: Image) -> Match:
        candidate = self.anchor.update(gray)
        if not self.initialized:
            self.initialized = True
            if not candidate.valid:
                self.previous = None
                return candidate
            return Match("ok", candidate.score, candidate.peak_gap, 0.0, 0.0, "reference", corners=self.reference)
        result = Match("corners_lost", candidate.score, candidate.peak_gap, method="corner_flow")
        if self.previous is not None:
            result = self._follow(
                self.previous_gray, gray, self.previous, self.previous, candidate, "corner_flow",
            )
        if not result.valid and candidate.valid:
            assert candidate.dx is not None and candidate.dy is not None
            guesses = to_quad(np.asarray(self.reference) + [candidate.dx, candidate.dy])
            result = self._follow(
                self.first_gray, gray, self.reference, guesses, candidate, "corner_reacquire",
            )
        if result.valid:
            assert result.corners is not None and result.dx is not None and result.dy is not None
            self.previous, self.previous_gray = result.corners, gray
            x, y, _, _ = self.anchor.region.rect
            self.anchor.position = (x + result.dx, y + result.dy)
        else:
            self.previous = None
        return result


def relative_displacement(blade: Match, hub: Match) -> tuple[float | None, float | None]:
    if not blade.valid or not hub.valid:
        return None, None
    assert blade.dx is not None and blade.dy is not None
    assert hub.dx is not None and hub.dy is not None
    return blade.dx - hub.dx, blade.dy - hub.dy
