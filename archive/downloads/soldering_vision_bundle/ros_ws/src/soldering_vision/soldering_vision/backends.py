"""Detection backends.  AI dependencies are loaded only when selected."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Protocol

import cv2
import numpy as np

from .vision_core import CLASS_IDS, Detection


class DetectorBackend(Protocol):
    name: str

    def detect(self, image_bgr: np.ndarray) -> list[Detection]:
        ...


@dataclass(frozen=True)
class HsvRule:
    label: str
    lower: tuple[int, int, int]
    upper: tuple[int, int, int]
    min_area_px: float = 60.0


DEFAULT_HSV_RULES = (
    HsvRule("fixed_wire", (140, 80, 60), (179, 255, 255)),
    HsvRule("moving_wire", (40, 70, 50), (85, 255, 255)),
    HsvRule("iron_tip", (90, 80, 50), (135, 255, 255)),
    HsvRule("solder_wire", (20, 80, 80), (38, 255, 255)),
    HsvRule("insulation", (0, 0, 0), (179, 255, 45), 120.0),
)


class HsvRoiBackend:
    """Deterministic colored-marker baseline; not a production detector."""

    name = "hsv_baseline"

    def __init__(self, rules: tuple[HsvRule, ...] = DEFAULT_HSV_RULES):
        self.rules = rules

    def detect(self, image_bgr: np.ndarray) -> list[Detection]:
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("detector expects a BGR image")
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        kernel = np.ones((3, 3), dtype=np.uint8)
        detections: list[Detection] = []
        for rule in self.rules:
            mask = cv2.inRange(
                hsv,
                np.asarray(rule.lower, dtype=np.uint8),
                np.asarray(rule.upper, dtype=np.uint8),
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _hierarchy = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(contour))
            if area < rule.min_area_px:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if rule.label != "insulation":
                points = contour.reshape(-1, 2).astype(np.float32)
                image_center = np.array(
                    [image_bgr.shape[1] * 0.5, image_bgr.shape[0] * 0.5],
                    dtype=np.float32,
                )
                nearest = points[
                    np.argmin(np.linalg.norm(points - image_center, axis=1))
                ]
                anchor = float(nearest[0]), float(nearest[1])
            else:
                moments = cv2.moments(contour)
                if abs(moments["m00"]) <= 1.0e-6:
                    anchor = (x + width * 0.5, y + height * 0.5)
                else:
                    anchor = (
                        float(moments["m10"] / moments["m00"]),
                        float(moments["m01"] / moments["m00"]),
                    )
            detections.append(
                Detection(
                    label=rule.label,
                    confidence=min(0.99, 0.6 + area / 5000.0),
                    bbox_xyxy=(
                        float(x),
                        float(y),
                        float(x + width),
                        float(y + height),
                    ),
                    anchor_px=anchor,
                    mask_area_px=area,
                )
            )
        return detections


class UltralyticsBackend:
    """Ultralytics YOLO detect/segment/pose adapter."""

    name = "ultralytics_yolo"

    def __init__(
        self,
        model_path: str,
        *,
        device: str = "0",
        image_size: int = 640,
        confidence: float = 0.45,
        iou: float = 0.5,
        class_aliases_json: str = "{}",
    ) -> None:
        if not model_path or not Path(model_path).exists():
            raise FileNotFoundError(f"YOLO model not found: {model_path}")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is not installed in the active environment"
            ) from exc
        self.model = YOLO(model_path)
        self.device = device
        self.image_size = image_size
        self.confidence = confidence
        self.iou = iou
        aliases = json.loads(class_aliases_json)
        if not isinstance(aliases, dict):
            raise ValueError("class_aliases_json must encode an object")
        self.aliases = {str(key): str(value) for key, value in aliases.items()}

    @staticmethod
    def _anchor(
        box: tuple[float, float, float, float],
        contour: np.ndarray | None,
        keypoint: tuple[float, float] | None,
        image_shape: tuple[int, ...],
    ) -> tuple[float, float]:
        if keypoint is not None:
            return keypoint
        if contour is not None and contour.size:
            center = np.array(
                [image_shape[1] * 0.5, image_shape[0] * 0.5],
                dtype=np.float32,
            )
            points = contour.reshape(-1, 2).astype(np.float32)
            return tuple(points[np.argmin(np.linalg.norm(points - center, axis=1))])
        x1, y1, x2, y2 = box
        return 0.5 * (x1 + x2), 0.5 * (y1 + y2)

    def detect(self, image_bgr: np.ndarray) -> list[Detection]:
        predictions = self.model.predict(
            source=image_bgr,
            imgsz=self.image_size,
            conf=self.confidence,
            iou=self.iou,
            device=self.device,
            verbose=False,
        )
        if not predictions:
            return []
        result = predictions[0]
        if result.boxes is None:
            return []
        names = result.names
        boxes = result.boxes.xyxy.detach().cpu().numpy()
        classes = result.boxes.cls.detach().cpu().numpy().astype(int)
        confidences = result.boxes.conf.detach().cpu().numpy()
        contours = result.masks.xy if result.masks is not None else []
        keypoints = None
        if result.keypoints is not None and result.keypoints.xy is not None:
            keypoints = result.keypoints.xy.detach().cpu().numpy()
        detections: list[Detection] = []
        for index, (box_array, class_index, confidence) in enumerate(
            zip(boxes, classes, confidences)
        ):
            source_label = str(names[int(class_index)])
            label = self.aliases.get(source_label, source_label)
            if label not in CLASS_IDS:
                continue
            box = tuple(float(value) for value in box_array)
            contour = (
                np.asarray(contours[index], dtype=np.float32)
                if index < len(contours)
                else None
            )
            keypoint = None
            if keypoints is not None and index < len(keypoints):
                candidates = keypoints[index]
                if len(candidates) and np.isfinite(candidates[0]).all():
                    keypoint = tuple(float(value) for value in candidates[0])
            mask_area = float(cv2.contourArea(contour)) if contour is not None else 0.0
            detections.append(
                Detection(
                    label=label,
                    confidence=float(confidence),
                    bbox_xyxy=box,
                    anchor_px=self._anchor(
                        box, contour, keypoint, image_bgr.shape
                    ),
                    mask_area_px=mask_area,
                )
            )
        return detections
