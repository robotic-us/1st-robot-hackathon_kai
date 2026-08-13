"""ROS-independent geometry, tracking, and crop construction."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Iterable, Optional

import numpy as np


CLASS_IDS = {
    "unknown": 0,
    "fixed_wire": 1,
    "moving_wire": 2,
    "iron_tip": 3,
    "solder_wire": 4,
    "solder_joint": 5,
    "insulation": 6,
    "copper": 7,
}
REQUIRED_LABELS = ("fixed_wire", "moving_wire", "iron_tip", "solder_wire")


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]
    anchor_px: tuple[float, float]
    mask_area_px: float = 0.0
    track_id: int = -1
    position_mm: Optional[tuple[float, float]] = None

    def __post_init__(self) -> None:
        values = (*self.bbox_xyxy, *self.anchor_px, self.confidence)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("detection values must be finite")
        x1, y1, x2, y2 = self.bbox_xyxy
        if x2 < x1 or y2 < y1:
            raise ValueError("bbox must have non-negative size")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in 0..1")


@dataclass
class _Track:
    track_id: int
    label: str
    anchor_px: tuple[float, float]
    missed: int = 0


class CentroidTracker:
    """Small deterministic per-class tracker for stable observation IDs."""

    def __init__(
        self,
        *,
        max_distance_px: float = 80.0,
        max_missed: int = 5,
        smoothing_alpha: float = 0.65,
    ) -> None:
        if max_distance_px <= 0.0 or max_missed < 0:
            raise ValueError("invalid tracker limits")
        if not 0.0 < smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        self.max_distance_px = max_distance_px
        self.max_missed = max_missed
        self.smoothing_alpha = smoothing_alpha
        self._next_id = 1
        self._tracks: dict[int, _Track] = {}

    def update(self, detections: Iterable[Detection]) -> list[Detection]:
        for track in self._tracks.values():
            track.missed += 1
        assigned: set[int] = set()
        result: list[Detection] = []
        for detection in sorted(
            detections, key=lambda item: item.confidence, reverse=True
        ):
            candidate: Optional[_Track] = None
            candidate_distance = math.inf
            for track in self._tracks.values():
                if track.track_id in assigned or track.label != detection.label:
                    continue
                distance = math.dist(track.anchor_px, detection.anchor_px)
                if (
                    distance <= self.max_distance_px
                    and distance < candidate_distance
                ):
                    candidate = track
                    candidate_distance = distance
            if candidate is None:
                candidate = _Track(
                    track_id=self._next_id,
                    label=detection.label,
                    anchor_px=detection.anchor_px,
                )
                self._tracks[candidate.track_id] = candidate
                self._next_id += 1
            else:
                alpha = self.smoothing_alpha
                old_u, old_v = candidate.anchor_px
                new_u, new_v = detection.anchor_px
                candidate.anchor_px = (
                    alpha * new_u + (1.0 - alpha) * old_u,
                    alpha * new_v + (1.0 - alpha) * old_v,
                )
            candidate.missed = 0
            assigned.add(candidate.track_id)
            result.append(
                replace(
                    detection,
                    track_id=candidate.track_id,
                    anchor_px=candidate.anchor_px,
                )
            )
        expired = [
            track_id
            for track_id, track in self._tracks.items()
            if track.missed > self.max_missed
        ]
        for track_id in expired:
            del self._tracks[track_id]
        return result


class PlanarProjector:
    """Project rectified image pixels onto a fixed work plane."""

    def __init__(self, homography: Iterable[float], *, calibrated: bool):
        values = np.asarray(list(homography), dtype=np.float64)
        if values.size != 9:
            raise ValueError("homography must contain 9 values")
        self.matrix = values.reshape(3, 3)
        self.calibrated = bool(calibrated)
        if not np.isfinite(self.matrix).all():
            raise ValueError("homography must be finite")
        if self.calibrated and abs(np.linalg.det(self.matrix)) < 1.0e-12:
            raise ValueError("calibrated homography must be invertible")

    def project(self, point_px: tuple[float, float]) -> Optional[tuple[float, float]]:
        if not self.calibrated:
            return None
        projected = self.matrix @ np.array([*point_px, 1.0])
        if abs(projected[2]) < 1.0e-12:
            return None
        x_mm = float(projected[0] / projected[2])
        y_mm = float(projected[1] / projected[2])
        if not math.isfinite(x_mm) or not math.isfinite(y_mm):
            return None
        return x_mm, y_mm


@dataclass(frozen=True)
class GeometryResult:
    valid: bool
    calibrated: bool
    detections: tuple[Detection, ...]
    points_mm: dict[str, Optional[tuple[float, float]]]
    wire_error_mm: Optional[tuple[float, float]]
    joint_mm: Optional[tuple[float, float]]
    iron_to_joint_mm: Optional[float]
    solder_to_joint_mm: Optional[float]
    solder_to_insulation_mm: Optional[float]


def _best_by_label(detections: Iterable[Detection]) -> dict[str, Detection]:
    best: dict[str, Detection] = {}
    for detection in detections:
        previous = best.get(detection.label)
        if previous is None or detection.confidence > previous.confidence:
            best[detection.label] = detection
    return best


def _distance(
    first: Optional[tuple[float, float]],
    second: Optional[tuple[float, float]],
) -> Optional[float]:
    if first is None or second is None:
        return None
    return math.dist(first, second)


def _point_to_detection_box_distance(
    point_detection: Optional[Detection],
    region_detection: Optional[Detection],
    projector: PlanarProjector,
) -> Optional[float]:
    if point_detection is None or region_detection is None:
        return None
    point_mm = point_detection.position_mm
    if point_mm is None:
        return None
    u, v = point_detection.anchor_px
    x1, y1, x2, y2 = region_detection.bbox_xyxy
    nearest_px = (min(max(u, x1), x2), min(max(v, y1), y2))
    nearest_mm = projector.project(nearest_px)
    return _distance(point_mm, nearest_mm)


class GeometryEstimator:
    def __init__(
        self,
        projector: PlanarProjector,
        *,
        confidence_min: float = 0.5,
        tracker: Optional[CentroidTracker] = None,
    ) -> None:
        if not 0.0 <= confidence_min <= 1.0:
            raise ValueError("confidence_min must be in 0..1")
        self.projector = projector
        self.confidence_min = confidence_min
        self.tracker = tracker if tracker is not None else CentroidTracker()

    def update(self, raw_detections: Iterable[Detection]) -> GeometryResult:
        accepted = [
            item
            for item in raw_detections
            if item.confidence >= self.confidence_min
            and item.label in CLASS_IDS
        ]
        tracked = self.tracker.update(accepted)
        projected = [
            replace(
                item,
                position_mm=self.projector.project(item.anchor_px),
            )
            for item in tracked
        ]
        best = _best_by_label(projected)
        points = {
            label: best[label].position_mm if label in best else None
            for label in CLASS_IDS
            if label != "unknown"
        }
        fixed = points["fixed_wire"]
        moving = points["moving_wire"]
        if fixed is not None and moving is not None:
            wire_error = (fixed[0] - moving[0], fixed[1] - moving[1])
            joint = (
                0.5 * (fixed[0] + moving[0]),
                0.5 * (fixed[1] + moving[1]),
            )
        else:
            wire_error = None
            joint = points["solder_joint"]
        valid = self.projector.calibrated and all(
            points[label] is not None for label in REQUIRED_LABELS
        )
        return GeometryResult(
            valid=valid,
            calibrated=self.projector.calibrated,
            detections=tuple(projected),
            points_mm=points,
            wire_error_mm=wire_error,
            joint_mm=joint,
            iron_to_joint_mm=_distance(points["iron_tip"], joint),
            solder_to_joint_mm=_distance(points["solder_wire"], joint),
            solder_to_insulation_mm=_point_to_detection_box_distance(
                best.get("solder_wire"),
                best.get("insulation"),
                self.projector,
            ),
        )


def interaction_crop_box(
    detections: Iterable[Detection],
    image_shape: tuple[int, ...],
    *,
    margin_ratio: float = 0.35,
    minimum_size_px: int = 96,
) -> tuple[int, int, int, int]:
    """Build a square ConvNeXt crop from YOLO interaction detections."""
    if margin_ratio < 0.0 or minimum_size_px < 2:
        raise ValueError("invalid crop settings")
    height, width = image_shape[:2]
    relevant = [
        item
        for item in detections
        if item.label
        in {
            "fixed_wire",
            "moving_wire",
            "iron_tip",
            "solder_wire",
            "solder_joint",
        }
    ]
    if relevant:
        x1 = min(item.bbox_xyxy[0] for item in relevant)
        y1 = min(item.bbox_xyxy[1] for item in relevant)
        x2 = max(item.bbox_xyxy[2] for item in relevant)
        y2 = max(item.bbox_xyxy[3] for item in relevant)
        center_x = 0.5 * (x1 + x2)
        center_y = 0.5 * (y1 + y2)
        size = max(x2 - x1, y2 - y1, float(minimum_size_px))
        size *= 1.0 + 2.0 * margin_ratio
    else:
        center_x = width * 0.5
        center_y = height * 0.5
        size = max(float(minimum_size_px), min(width, height) * 0.5)
    size = min(size, float(width), float(height))
    left = int(round(max(0.0, min(width - size, center_x - size * 0.5))))
    top = int(round(max(0.0, min(height - size, center_y - size * 0.5))))
    right = int(round(left + size))
    bottom = int(round(top + size))
    return left, top, min(right, width), min(bottom, height)
