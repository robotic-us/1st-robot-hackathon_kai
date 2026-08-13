#!/usr/bin/env python3
"""Publish a moving colored fixture scene for hardware-free integration tests."""

from __future__ import annotations

import math

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class SyntheticSceneNode(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_scene")
        self._publisher = self.create_publisher(
            Image, "/camera/image_raw", 10
        )
        self._bridge = CvBridge()
        self._frame = 0
        self.create_timer(0.1, self._publish)

    def _publish(self) -> None:
        image = np.full((480, 640, 3), 80, dtype=np.uint8)
        offset = int(round(8.0 * math.sin(self._frame * 0.08)))
        # BGR marker colors match the HSV baseline backend.
        cv2.line(image, (80, 235), (305, 235), (255, 0, 255), 16)
        cv2.line(
            image,
            (560, 250 + offset),
            (335, 250 + offset),
            (0, 255, 0),
            16,
        )
        cv2.line(image, (320, 50), (320, 215), (255, 0, 0), 14)
        cv2.line(image, (370, 430), (342, 275), (0, 255, 255), 10)
        cv2.rectangle(image, (70, 220), (170, 250), (15, 15, 15), 8)
        self._frame += 1
        message = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "synthetic_camera"
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SyntheticSceneNode()
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
