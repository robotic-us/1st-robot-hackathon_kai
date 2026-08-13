#!/usr/bin/env python3
"""High-level target distributor for six independent motor nodes."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from soldering_control.motor_axis_node import TARGET_QOS


TARGET_SCHEDULE_DEG = (
    (5.0, -3.0, 2.0, -2.0, 1.0, -1.0),
    (-5.0, 3.0, -2.0, 2.0, -1.0, 1.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (3.0, 3.0, -3.0, -3.0, 2.0, -2.0),
    (-3.0, -3.0, 3.0, 3.0, -2.0, 2.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
)


class SixMotorCoordinator(Node):
    """Publish setpoints only; motor-local control remains in motor nodes."""

    def __init__(self) -> None:
        super().__init__("six_motor_coordinator")
        self._segment_s = float(
            self.declare_parameter("segment_s", 5.0).value
        )
        if self._segment_s <= 0.0:
            raise ValueError("segment_s must be positive")
        self._publishers = [
            self.create_publisher(
                Float64, f"/motors/motor_{motor_id}/target_deg", TARGET_QOS
            )
            for motor_id in range(1, 7)
        ]
        self._segment = -1
        self._started_ns = self.get_clock().now().nanoseconds
        self.create_timer(0.05, self._on_tick)

    def _on_tick(self) -> None:
        elapsed_s = (
            self.get_clock().now().nanoseconds - self._started_ns
        ) / 1.0e9
        segment = min(
            int(elapsed_s / self._segment_s),
            len(TARGET_SCHEDULE_DEG) - 1,
        )
        if segment == self._segment:
            return
        self._segment = segment
        targets = TARGET_SCHEDULE_DEG[segment]
        for publisher, target_deg in zip(self._publishers, targets):
            msg = Float64()
            msg.data = target_deg
            publisher.publish(msg)
        self.get_logger().info(
            f"segment {segment + 1}/{len(TARGET_SCHEDULE_DEG)}: "
            f"targets={targets}deg"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SixMotorCoordinator()
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
