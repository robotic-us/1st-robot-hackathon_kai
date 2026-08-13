#!/usr/bin/env python3
"""ROS 2 node: image -> YOLO geometry -> ConvNeXt process state."""

from __future__ import annotations

import math
import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from soldering_interfaces.msg import GeometryObservation, VisionObject

from .backends import HsvRoiBackend, UltralyticsBackend
from .process_classifier import (
    ConvNextProcessClassifier,
    DisabledClassifier,
)
from .vision_core import (
    CLASS_IDS,
    Detection,
    GeometryEstimator,
    PlanarProjector,
    interaction_crop_box,
)


def _number(value: float | None) -> float:
    return math.nan if value is None else float(value)


class VisionObserverNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_observer")
        image_topic = str(
            self.declare_parameter("image_topic", "/camera/image_raw").value
        )
        backend_name = str(
            self.declare_parameter("detector_backend", "heuristic").value
        )
        model_path = str(self.declare_parameter("yolo_model", "").value)
        device = str(self.declare_parameter("device", "0").value)
        image_size = int(self.declare_parameter("yolo_image_size", 640).value)
        confidence_min = float(
            self.declare_parameter("confidence_min", 0.55).value
        )
        iou = float(self.declare_parameter("yolo_iou", 0.5).value)
        aliases = str(
            self.declare_parameter("class_aliases_json", "{}").value
        )
        homography = self.declare_parameter(
            "homography",
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        ).value
        calibrated = bool(
            self.declare_parameter("homography_valid", False).value
        )
        inference_hz = float(
            self.declare_parameter("inference_hz", 10.0).value
        )
        process_model = str(
            self.declare_parameter("convnext_model", "").value
        )
        process_device = str(
            self.declare_parameter("convnext_device", "cuda").value
        )
        self._process_confidence_min = float(
            self.declare_parameter("process_confidence_min", 0.65).value
        )
        self._publish_annotated = bool(
            self.declare_parameter("publish_annotated", True).value
        )
        if inference_hz <= 0.0:
            raise ValueError("inference_hz must be positive")
        if backend_name == "heuristic":
            self._detector = HsvRoiBackend()
        elif backend_name in ("yolo", "ultralytics"):
            self._detector = UltralyticsBackend(
                model_path,
                device=device,
                image_size=image_size,
                confidence=confidence_min,
                iou=iou,
                class_aliases_json=aliases,
            )
        else:
            raise ValueError(f"unknown detector_backend: {backend_name}")
        if process_model:
            self._classifier = ConvNextProcessClassifier(
                process_model, device=process_device
            )
        else:
            self._classifier = DisabledClassifier()
        self._estimator = GeometryEstimator(
            PlanarProjector(homography, calibrated=calibrated),
            confidence_min=confidence_min,
        )
        self._bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_image: Image | None = None
        self._last_stamp_ns = -1
        self._sequence = 0
        self._observation_publisher = self.create_publisher(
            GeometryObservation, "/soldering/geometry_observation", 10
        )
        self._annotated_publisher = self.create_publisher(
            Image, "/soldering/vision/annotated", qos_profile_sensor_data
        )
        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_timer(1.0 / inference_hz, self._process_latest)
        self.get_logger().info(
            f"vision ready backend={self._detector.name} "
            f"calibrated={calibrated} image_topic={image_topic}"
        )

    def _on_image(self, message: Image) -> None:
        with self._frame_lock:
            self._latest_image = message

    @staticmethod
    def _stamp_ns(message: Image) -> int:
        return (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    def _process_latest(self) -> None:
        with self._frame_lock:
            message = self._latest_image
        if message is None:
            return
        stamp_ns = self._stamp_ns(message)
        if stamp_ns == self._last_stamp_ns:
            return
        self._last_stamp_ns = stamp_ns
        started = time.perf_counter()
        try:
            frame = self._bridge.imgmsg_to_cv2(message, "bgr8")
            detections = self._detector.detect(frame)
            geometry = self._estimator.update(detections)
            crop_box = interaction_crop_box(geometry.detections, frame.shape)
            left, top, right, bottom = crop_box
            classification = self._classifier.classify(
                frame[top:bottom, left:right]
            )
        except Exception as exc:  # noqa: BLE001 - publish failures as diagnostics
            self.get_logger().error(
                f"vision inference failed: {type(exc).__name__}: {exc}"
            )
            return
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        observation = self._build_message(
            message,
            geometry,
            classification.label,
            classification.confidence,
            elapsed_ms,
        )
        self._observation_publisher.publish(observation)
        if self._publish_annotated:
            annotated = self._annotate(
                frame,
                geometry.detections,
                crop_box,
                classification.label,
                classification.confidence,
                observation.valid,
            )
            annotated_message = self._bridge.cv2_to_imgmsg(
                annotated, encoding="bgr8"
            )
            annotated_message.header = message.header
            self._annotated_publisher.publish(annotated_message)

    def _build_message(
        self,
        image: Image,
        geometry,
        process_class: str,
        process_confidence: float,
        inference_ms: float,
    ) -> GeometryObservation:
        message = GeometryObservation()
        message.header = image.header
        message.sequence = self._sequence
        self._sequence += 1
        message.calibrated = geometry.calibrated
        message.detector_backend = self._detector.name
        message.inference_ms = inference_ms
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = self._stamp_ns(image)
        message.image_age_ms = max(0.0, (now_ns - stamp_ns) / 1.0e6)
        message.process_class = process_class
        message.process_confidence = process_confidence
        message.occluded = process_class == "occluded"
        process_reliable = (
            not self._classifier.enabled
            or process_confidence >= self._process_confidence_min
        )
        message.valid = (
            geometry.valid
            and process_reliable
            and process_class not in {"occluded", "unsafe"}
        )
        for item in geometry.detections:
            message.objects.append(self._object_message(item))
        self._assign_point(message, geometry, "fixed_wire")
        self._assign_point(message, geometry, "moving_wire")
        self._assign_point(message, geometry, "iron_tip")
        self._assign_point(message, geometry, "solder_wire")
        wire_error = geometry.wire_error_mm
        message.wire_error_x_mm = _number(
            None if wire_error is None else wire_error[0]
        )
        message.wire_error_y_mm = _number(
            None if wire_error is None else wire_error[1]
        )
        message.iron_to_joint_mm = _number(geometry.iron_to_joint_mm)
        message.solder_to_joint_mm = _number(geometry.solder_to_joint_mm)
        message.solder_to_insulation_mm = _number(
            geometry.solder_to_insulation_mm
        )
        return message

    @staticmethod
    def _assign_point(message, geometry, label: str) -> None:
        point = geometry.points_mm[label]
        setattr(message, label + "_valid", point is not None)
        setattr(message, label + "_x_mm", _number(None if point is None else point[0]))
        setattr(message, label + "_y_mm", _number(None if point is None else point[1]))

    @staticmethod
    def _object_message(detection: Detection) -> VisionObject:
        message = VisionObject()
        message.class_id = CLASS_IDS.get(detection.label, 0)
        message.class_name = detection.label
        message.track_id = detection.track_id
        message.confidence = detection.confidence
        x1, y1, x2, y2 = detection.bbox_xyxy
        message.bbox_x_px = x1
        message.bbox_y_px = y1
        message.bbox_width_px = x2 - x1
        message.bbox_height_px = y2 - y1
        message.anchor_u_px = detection.anchor_px[0]
        message.anchor_v_px = detection.anchor_px[1]
        message.mask_area_px = detection.mask_area_px
        message.position_valid = detection.position_mm is not None
        if detection.position_mm is None:
            message.x_mm = math.nan
            message.y_mm = math.nan
        else:
            message.x_mm, message.y_mm = detection.position_mm
        message.z_mm = 0.0 if message.position_valid else math.nan
        return message

    @staticmethod
    def _annotate(
        frame: np.ndarray,
        detections: tuple[Detection, ...],
        crop_box: tuple[int, int, int, int],
        process_class: str,
        process_confidence: float,
        valid: bool,
    ) -> np.ndarray:
        output = frame.copy()
        for item in detections:
            x1, y1, x2, y2 = (int(value) for value in item.bbox_xyxy)
            cv2.rectangle(output, (x1, y1), (x2, y2), (0, 220, 0), 2)
            anchor = tuple(int(value) for value in item.anchor_px)
            cv2.circle(output, anchor, 4, (0, 0, 255), -1)
            cv2.putText(
                output,
                f"{item.label} {item.confidence:.2f} id={item.track_id}",
                (x1, max(15, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        left, top, right, bottom = crop_box
        cv2.rectangle(output, (left, top), (right, bottom), (0, 180, 255), 2)
        cv2.putText(
            output,
            f"process={process_class}:{process_confidence:.2f} valid={valid}",
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0) if valid else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return output


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VisionObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
